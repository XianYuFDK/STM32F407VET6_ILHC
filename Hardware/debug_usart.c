/**
 ******************************************************************************
 * @file    debug_usart.c
 * @brief   USART1 调试模块（VOFA+ 调参）
 *
 *          - VOFA+ JustFloat 数据帧：N*float + 0x00 0x00 0x80 0x7F
 *          - DMA 空闲接收 ASCII 命令：KPX=3.0、XVMAX=1600、STOP、ZERO
 *                            以及 DM 电机：DMID/DMEN/DMOFF/DMMODE/DMPOS 等
 *          - GOTO=x,y,z：上位机点击场地地图下发 OPS 全局定位移动目标，
 *            本任务每 20ms 周期执行一步 MecanumControl_GotoOPS，STOP 取消
 *          - 参数表可扩展：在 DebugParam_t 表中增加一项即可
 ******************************************************************************
 */
#include "debug_usart.h"
#include "usart.h"
#include "mecanum_control.h"
#include "ops.h"
#include "dm_j4310.h"
#include "hcan.h"

#include <string.h>


/* --------------------------- 调试参数 ------------------------------ */
#define DEBUG_RX_SIZE     256U
#define DEBUG_LINE_SIZE   64U
#define DEBUG_VOFA_TAIL0  0x00U
#define DEBUG_VOFA_TAIL1  0x00U
#define DEBUG_VOFA_TAIL2  0x80U
#define DEBUG_VOFA_TAIL3  0x7FU

/* ------------------------- DM 电机调试参数 ------------------------- */
#define DEBUG_DM_DEFAULT_ID     1U
#define DEBUG_DM_DEFAULT_MODE   DM_J4310_CTRL_MODE_MIT
#define DEBUG_DM_DEFAULT_POS    0.0f
#define DEBUG_DM_DEFAULT_VEL    0.0f
#define DEBUG_DM_DEFAULT_KP     2.0f
#define DEBUG_DM_DEFAULT_KD     0.5f
#define DEBUG_DM_DEFAULT_TORQUE 0.0f
#define DEBUG_DM_MODE_WAIT_MS   100U
#define DEBUG_HOST_TIMEOUT_MS   1000U
#define DEBUG_OPS_TIMEOUT_MS    200U

/* 可调参数表 */
typedef struct
{
  const char *name;
  float      *value;
  float       min;
  float       max;
} DebugParam_t;

static const DebugParam_t s_params[] =
{
  {"KPX",   &mKpx,   0.0f,   50.0f},
  {"KPY",   &mKpy,   0.0f,   50.0f},
  {"KPZ",   &mKpz,   0.0f,   50.0f},
  {"XVMAX", &XYVmax, 0.0f,   3000.0f},
  {"ZVMAX", &ZVmax,  0.0f,   3000.0f},
  {"XVMIN", &XYVmin, 0.0f,   100.0f},
  {"ZVMIN", &ZVmin,  0.0f,   100.0f},
};

/* --------------------------- 私有变量 ------------------------------ */
static uint8_t s_rx[DEBUG_RX_SIZE];
static uint8_t s_line[DEBUG_LINE_SIZE];
static uint16_t s_line_len;

static uint8_t s_tx[(4U * DEBUG_VOFA_CHANNELS) + 4U];

static volatile uint8_t s_stop_req;
static volatile uint8_t s_zero_req;

/* GOTO 全局定位移动目标（上位机场地地图点击下发） */
static volatile uint8_t s_goto_active;
static float s_goto_x;
static float s_goto_y;
static float s_goto_z;

/* DM 电机调试状态 */
static uint16_t s_dm_id;
static uint8_t  s_dm_mode;
static uint8_t  s_dm_active;
static float    s_dm_pos;
static float    s_dm_vel;
static float    s_dm_kp;
static float    s_dm_kd;
static float    s_dm_torque;
static DmJ4310Feedback_t s_dm_feedback;

static volatile uint8_t s_dm_enable_req;
static volatile uint8_t s_dm_disable_req;
static volatile uint8_t s_dm_zero_req;
static volatile uint8_t s_dm_mode_req;
static uint8_t s_dm_start_pending;
static uint32_t s_dm_mode_tick;
static volatile uint32_t s_host_last_tick;

/* --------------------------- 私有函数 ------------------------------ */

/**
 * @brief  ASCII 忽略大小写比较
 */
static uint8_t Debug_StrCaseCmp(const char *a, const char *b)
{
  while ((*a != '\0') && (*b != '\0'))
  {
    char ca = *a;
    char cb = *b;

    if ((ca >= 'a') && (ca <= 'z')) ca -= ('a' - 'A');
    if ((cb >= 'a') && (cb <= 'z')) cb -= ('a' - 'A');

    if (ca != cb)
    {
      return 1U;
    }

    ++a;
    ++b;
  }

  return ((*a == '\0') && (*b == '\0')) ? 0U : 1U;
}
/**
 * @brief  轻量浮点解析，仅支持小数，无指数
 */
static uint8_t Debug_ParseFloat(const char *s, float *out)
{
  float sign = 1.0f;
  float value = 0.0f;
  float scale = 0.1f;
  uint8_t has_digit = 0U;

  if ((s == NULL) || (out == NULL))
  {
    return 0U;
  }

  while ((*s == ' ') || (*s == '\t'))
  {
    ++s;
  }

  if (*s == '-')
  {
    sign = -1.0f;
    ++s;
  }
  else if (*s == '+')
  {
    ++s;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    value = (value * 10.0f) + (float)(*s - '0');
    has_digit = 1U;
    ++s;
  }

  if (*s == '.')
  {
    ++s;
    while ((*s >= '0') && (*s <= '9'))
    {
      value += (float)(*s - '0') * scale;
      scale *= 0.1f;
      has_digit = 1U;
      ++s;
    }
  }

  if (!has_digit)
  {
    return 0U;
  }

  *out = sign * value;
  return 1U;
}

/**
 * @brief  ASCII 忽略大小写前缀比较（a 的前 n 个字符与 b 比较）
 */
static uint8_t Debug_StrCaseCmpN(const char *a, const char *b, uint32_t n)
{
  uint32_t i;

  for (i = 0U; i < n; ++i)
  {
    char ca = a[i];
    char cb = b[i];

    if ((ca >= 'a') && (ca <= 'z')) ca -= ('a' - 'A');
    if ((cb >= 'a') && (cb <= 'z')) cb -= ('a' - 'A');

    if ((ca != cb) || (ca == '\0'))
    {
      return 1U;
    }
  }

  return 0U;
}

/**
 * @brief  解析逗号分隔的浮点列表，例如 "1200,800,90"
 * @return 解析出的个数（0 ~ max）
 */
static uint8_t Debug_ParseFloatList(const char *s, float *out, uint8_t max)
{
  uint8_t count = 0U;

  if ((s == NULL) || (out == NULL))
  {
    return 0U;
  }

  while ((*s != '\0') && (count < max))
  {
    float value;

    if (!Debug_ParseFloat(s, &value))
    {
      break;
    }

    out[count] = value;
    ++count;

    /* 跳过已解析的数字，直到逗号或结尾 */
    while ((*s != '\0') && (*s != ','))
    {
      ++s;
    }
    if (*s == ',')
    {
      ++s;
    }
  }

  return count;
}


/**
 * @brief  在参数表中查找并设置参数
 */
static void Debug_SetParam(const char *name, float value)
{
  uint32_t i;
  uint32_t count = sizeof(s_params) / sizeof(s_params[0]);

  for (i = 0U; i < count; ++i)
  {
    if (Debug_StrCaseCmp(name, s_params[i].name) == 0U)
    {
      if (value < s_params[i].min)
      {
        value = s_params[i].min;
      }
      else if (value > s_params[i].max)
      {
        value = s_params[i].max;
      }

      *s_params[i].value = value;
      return;
    }
  }
}

/**
 * @brief  设置 DM 电机调试参数
 */
static void Debug_SetDmValue(const char *name, float value)
{
  if (Debug_StrCaseCmp(name, "DMID") == 0U)
  {
    if ((value >= 1.0f) && (value <= 0x06FFU))
    {
      s_dm_id = (uint16_t)value;
    }
    return;
  }

  if (Debug_StrCaseCmp(name, "DMMODE") == 0U)
  {
    if ((value >= DM_J4310_CTRL_MODE_MIT) &&
        (value <= DM_J4310_CTRL_MODE_POS_VEL))
    {
      s_dm_mode = (uint8_t)value;
      s_dm_mode_req = 1U;
    }
    return;
  }

  if (Debug_StrCaseCmp(name, "DMPOS") == 0U)
  {
    if (value < DM_J4310_POS_MIN) { value = DM_J4310_POS_MIN; }
    if (value > DM_J4310_POS_MAX) { value = DM_J4310_POS_MAX; }
    s_dm_pos = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMVEL") == 0U)
  {
    if (value < DM_J4310_VEL_MIN) { value = DM_J4310_VEL_MIN; }
    if (value > DM_J4310_VEL_MAX) { value = DM_J4310_VEL_MAX; }
    s_dm_vel = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMKP") == 0U)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > DM_J4310_KP_MAX) { value = DM_J4310_KP_MAX; }
    s_dm_kp = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMKD") == 0U)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > DM_J4310_KD_MAX) { value = DM_J4310_KD_MAX; }
    s_dm_kd = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMTOR") == 0U)
  {
    if (value < DM_J4310_TORQUE_MIN) { value = DM_J4310_TORQUE_MIN; }
    if (value > DM_J4310_TORQUE_MAX) { value = DM_J4310_TORQUE_MAX; }
    s_dm_torque = value;
  }
}

/**
 * @brief  解析一条完整命令
 */
static void Debug_ParseLine(char *line)
{
  char *equal;
  float value;

  while ((*line == ' ') || (*line == '\t'))
  {
    ++line;
  }

  if (Debug_StrCaseCmp(line, "STOP") == 0U)
  {
    s_stop_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "PING") == 0U)
  {
    return;
  }

  if (Debug_StrCaseCmp(line, "ZERO") == 0U)
  {
    s_zero_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "DMEN") == 0U)
  {
    s_dm_enable_req = 1U;
    return;
  }

  if ((Debug_StrCaseCmp(line, "DMOFF") == 0U) ||
      (Debug_StrCaseCmp(line, "DMSTOP") == 0U))
  {
    s_dm_disable_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "DMZERO") == 0U)
  {
    s_dm_zero_req = 1U;
    return;
  }

  /* GOTO=x,y,z：OPS 全局定位移动目标，z 可省略（保持当前航向） */
  if ((Debug_StrCaseCmpN(line, "GOTO", 4U) == 0U) && (line[4] == '='))
  {
    float v[3];
    uint8_t n = Debug_ParseFloatList(line + 5U, v, 3U);

    if (n >= 2U)
    {
      if (v[0] < -3000.0f) { v[0] = -3000.0f; }
      if (v[0] > 3000.0f)  { v[0] = 3000.0f; }
      if (v[1] < -3000.0f) { v[1] = -3000.0f; }
      if (v[1] > 3000.0f)  { v[1] = 3000.0f; }

      s_goto_x = v[0];
      s_goto_y = v[1];
      /* 目标航向未给出时保持当前航向 */
      s_goto_z = (n >= 3U) ? v[2] : zangle;
      s_goto_active = 1U;
    }
    return;
  }

  equal = strchr(line, '=');
  if (equal == NULL)
  {
    return;
  }

  *equal = '\0';
  if (Debug_ParseFloat(equal + 1, &value))
  {
    if (((line[0] == 'D') || (line[0] == 'd')) &&
        ((line[1] == 'M') || (line[1] == 'm')))
    {
      Debug_SetDmValue(line, value);
    }
    else
    {
      Debug_SetParam(line, value);
    }
  }
}

/* --------------------------- 对外接口 ------------------------------ */

/**
 * @brief  初始化 USART1 调试接收
 */
void DebugUsart_Init(void)
{
  s_line_len = 0U;
  s_stop_req = 0U;
  s_zero_req = 0U;
  s_goto_active = 0U;
  s_goto_x = 0.0f;
  s_goto_y = 0.0f;
  s_goto_z = 0.0f;

  /* DM 电机默认值 */
  s_dm_id = DEBUG_DM_DEFAULT_ID;
  s_dm_mode = DEBUG_DM_DEFAULT_MODE;
  s_dm_active = 0U;
  s_dm_pos = DEBUG_DM_DEFAULT_POS;
  s_dm_vel = DEBUG_DM_DEFAULT_VEL;
  s_dm_kp = DEBUG_DM_DEFAULT_KP;
  s_dm_kd = DEBUG_DM_DEFAULT_KD;
  s_dm_torque = DEBUG_DM_DEFAULT_TORQUE;
  memset(&s_dm_feedback, 0, sizeof(DmJ4310Feedback_t));
  s_dm_enable_req = 0U;
  s_dm_disable_req = 0U;
  s_dm_zero_req = 0U;
  s_dm_mode_req = 0U;
  s_dm_start_pending = 0U;
  s_dm_mode_tick = 0U;
  s_host_last_tick = HAL_GetTick();

  /* 注册 USART1 专用接收回调，不影响 OPS 的 USART2 回调 */
  (void)HAL_UART_RegisterRxEventCallback(&huart1, DebugUsart_RxEventCallback);

  /* 启动空闲中断 + DMA 接收 */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

/**
 * @brief  发送一次 VOFA+ JustFloat 数据帧
 */
void DebugUsart_Send(void)
{
  float data[DEBUG_VOFA_CHANNELS];
  DmJ4310Feedback_t dmFb;
  uint32_t i;
  uint32_t len;
  uint32_t primask;

  /* 上位机失联时停止仍在运行的调试动作 */
  if ((uint32_t)(HAL_GetTick() - s_host_last_tick) > DEBUG_HOST_TIMEOUT_MS)
  {
    if (s_goto_active != 0U)
    {
      s_goto_active = 0U;
      MecanumControl_Stop();
    }
    if ((s_dm_active != 0U) || (s_dm_start_pending != 0U))
    {
      s_dm_active = 0U;
      s_dm_start_pending = 0U;
      (void)DmJ4310_Disable(s_dm_id);
    }
  }

  /* 处理等待执行的命令 */
  if (s_stop_req != 0U)
  {
    s_stop_req = 0U;
    s_goto_active = 0U;
    MecanumControl_Stop();
    s_dm_active = 0U;
    s_dm_start_pending = 0U;
    s_dm_enable_req = 0U;
    s_dm_mode_req = 0U;
    (void)DmJ4310_Disable(s_dm_id);
  }

  if (s_zero_req != 0U)
  {
    s_zero_req = 0U;
    s_goto_active = 0U;
    MecanumControl_Stop();
    OPS_ZeroCoordinates();
  }

  /* GOTO 定位移动：本任务 20ms 周期执行一步 P 控制并下发轮速，
     maxRpm 传 0 表示沿用当前调试限幅（XVMAX/ZVMAX），STOP 可随时取消 */
  if (s_goto_active != 0U)
  {
    if (OPS_IsOnline(DEBUG_OPS_TIMEOUT_MS) == 0U)
    {
      s_goto_active = 0U;
      MecanumControl_Stop();
    }
    else if (MecanumControl_GotoOPS(s_goto_x, s_goto_y, s_goto_z, 0.0f) != 0U)
    {
      s_goto_active = 0U;
      MecanumControl_Stop();
    }
  }

  /* DM 电机命令处理 */
  if (s_dm_disable_req != 0U)
  {
    s_dm_disable_req = 0U;
    s_dm_active = 0U;
    s_dm_start_pending = 0U;
    (void)DmJ4310_Disable(s_dm_id);
  }

  if (s_dm_mode_req != 0U)
  {
    uint8_t restart = (uint8_t)((s_dm_active != 0U) ||
                                (s_dm_start_pending != 0U) ||
                                (s_dm_enable_req != 0U));

    s_dm_mode_req = 0U;
    s_dm_active = 0U;
    s_dm_start_pending = 0U;
    if (restart != 0U)
    {
      (void)DmJ4310_Disable(s_dm_id);
    }
    if (DmJ4310_SetControlMode(s_dm_id, s_dm_mode) == DM_J4310_OK)
    {
      if (restart != 0U)
      {
        s_dm_mode_tick = HAL_GetTick();
        s_dm_start_pending = 1U;
      }
    }
  }

  if (s_dm_enable_req != 0U)
  {
    s_dm_enable_req = 0U;
    if ((s_dm_active == 0U) && (s_dm_start_pending == 0U))
    {
      (void)DmJ4310_Disable(s_dm_id);
      if (DmJ4310_SetControlMode(s_dm_id, s_dm_mode) == DM_J4310_OK)
      {
        s_dm_mode_tick = HAL_GetTick();
        s_dm_start_pending = 1U;
      }
    }
  }

  if (s_dm_zero_req != 0U)
  {
    s_dm_zero_req = 0U;
    (void)DmJ4310_SetZero(s_dm_id);
  }

  /* 控制模式写入后等待电机内部保存，再执行使能 */
  if ((s_dm_start_pending != 0U) &&
      ((uint32_t)(HAL_GetTick() - s_dm_mode_tick) >= DEBUG_DM_MODE_WAIT_MS))
  {
    if (DmJ4310_Enable(s_dm_id) == DM_J4310_OK)
    {
      s_dm_active = 1U;
    }
    s_dm_start_pending = 0U;
  }

  /* DM 电机使能后持续下发控制帧 */
  if (s_dm_active != 0U)
  {
    if (s_dm_mode == DM_J4310_CTRL_MODE_MIT)
    {
      (void)DmJ4310_MITControl(s_dm_id, s_dm_pos, s_dm_vel,
                               s_dm_kp, s_dm_kd, s_dm_torque);
    }
    else
    {
      (void)DmJ4310_PosVelControl(s_dm_id, s_dm_pos, s_dm_vel);
    }
  }

  /* 读取 DM 反馈快照 */
  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(&dmFb, &s_dm_feedback, sizeof(DmJ4310Feedback_t));
  if (primask == 0U)
  {
    __enable_irq();
  }

  data[0]  = pos_x;
  data[1]  = pos_y;
  data[2]  = zangle;
  data[3]  = devx;
  data[4]  = devy;
  data[5]  = devz;
  data[6]  = mKpx;
  data[7]  = mKpy;
  data[8]  = mKpz;
  data[9]  = XYVmax;
  data[10] = ZVmax;
  data[11] = (float)SpeedTarget[0];

  /* DM 电机调试反馈 */
  data[12] = (float)s_dm_id;
  data[13] = dmFb.position;
  data[14] = dmFb.velocity;
  data[15] = dmFb.torque;
  data[16] = (float)dmFb.status;
  data[17] = (float)dmFb.tempMos;
  data[18] = (float)dmFb.tempRotor;
  data[19] = s_dm_pos;
  data[20] = s_dm_vel;
  data[21] = s_dm_kp;
  data[22] = s_dm_kd;
  data[23] = s_dm_torque;

  for (i = 0U; i < DEBUG_VOFA_CHANNELS; ++i)
  {
    memcpy(&s_tx[4U * i], &data[i], 4U);
  }

  s_tx[4U * DEBUG_VOFA_CHANNELS]       = DEBUG_VOFA_TAIL0;
  s_tx[4U * DEBUG_VOFA_CHANNELS + 1U] = DEBUG_VOFA_TAIL1;
  s_tx[4U * DEBUG_VOFA_CHANNELS + 2U] = DEBUG_VOFA_TAIL2;
  s_tx[4U * DEBUG_VOFA_CHANNELS + 3U] = DEBUG_VOFA_TAIL3;
  len = sizeof(s_tx);

  if (huart1.gState == HAL_UART_STATE_READY)
  {
    (void)HAL_UART_Transmit_DMA(&huart1, s_tx, (uint16_t)len);
  }
}

/**
 * @brief  USART1 空闲接收回调
 */
void DebugUsart_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;

  if (huart->Instance != USART1)
  {
    return;
  }

  if (Size > sizeof(s_rx))
  {
    Size = (uint16_t)sizeof(s_rx);
  }

  for (i = 0U; i < Size; ++i)
  {
    uint8_t ch = s_rx[i];

    if ((ch == '\r') || (ch == '\n'))
    {
      if (s_line_len > 0U)
      {
        s_line[s_line_len] = '\0';
        s_line_len = 0U;
        s_host_last_tick = HAL_GetTick();
        Debug_ParseLine((char *)s_line);
      }
    }
    else if (s_line_len < (DEBUG_LINE_SIZE - 1U))
    {
      s_line[s_line_len++] = ch;
    }
    else
    {
      s_line_len = 0U;
    }
  }

  /* 重新启动空闲中断 + DMA 接收 */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}


/**
 * @brief  CAN 接收回调：解析 DM 电机反馈并缓存
 * @note   实现 hcan.c 中的 __weak 钩子
 */
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
  DmJ4310Feedback_t fb;
  uint8_t rxData[8];

  if ((hcan == NULL) || (hcan->Instance != CAN1))
  {
    return;
  }

  if ((hcanRxFrame.IDE != CAN_ID_STD) || (hcanRxFrame.DLC != 8U))
  {
    return;
  }

  memcpy(rxData, (const uint8_t *)hcanRxFrame.Data, sizeof(rxData));

  /* 忽略寄存器应答帧 */
  if ((rxData[1] <= 0x0FU) &&
      ((rxData[2] == 0x33U) || (rxData[2] == 0x55U) || (rxData[2] == 0xAAU)) &&
      (rxData[3] <= 81U))
  {
    return;
  }

  if (DmJ4310_DecodeFeedback(rxData, hcanRxFrame.DLC, &fb) != DM_J4310_OK)
  {
    return;
  }

  if ((fb.motorId == (uint8_t)s_dm_id) ||
      (hcanRxFrame.StdId == (uint32_t)s_dm_id))
  {
    memcpy(&s_dm_feedback, &fb, sizeof(DmJ4310Feedback_t));
  }
}
