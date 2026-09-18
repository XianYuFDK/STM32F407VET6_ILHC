/**
 ******************************************************************************
 * @file    debug_usart.c
 * @brief   USART1 调试模块（VOFA+ 调参）
 *
 *          - VOFA+ JustFloat 数据帧：N*float + 0x00 0x00 0x80 0x7F
 *          - DMA 空闲接收 ASCII 命令：KPX=3.0、XVMAX=1600、STOP、ZERO
 *                            以及 DM 电机：DMID/DMEN/DMOFF/DMMODE/DMPOS 等
 *          - 统一坐标约定（协议与底盘内部相同）：
 *            +X=车左、+Y=车头、+Z=逆时针；X/Y 直接对应 pos_x/pos_y，
 *            不存在交换或取反的适配层。
 *          - GOTO=x,y,z：上位机点击场地地图下发 OPS 全局定位移动目标（x=左右、
 *            y=前后，单位 cm 且保留 1 位小数；z=航向角），本任务每 20ms 周期执行
 *            一步 MecanumControl_GotoOPS，STOP 取消。固件内部仍用 mm 闭环。
 *          - WHEELEN/WHEELOFF：底盘四轮统一锁轴/释放，失能期间拒绝运动命令；
 *            失能后停车只清目标（MecanumControl_ClearTarget），绝不再发速度帧，
 *            否则ZDT_X42S会重新使能锁轴，表现为"失能了还是锁"
 *          - 参数表可扩展：在 DebugParam_t 表中增加一项即可
 ******************************************************************************
 */
#include "debug_usart.h"
#include "usart.h"
#include "mecanum_control.h"
#include "ops.h"
#include "dm_j4310.h"
#include "hcan.h"
#include "stepper_2835.h"
#include "zdt_x42s.h"

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

/* 可调参数项：名称 -> 变量指针 + 允许范围 */
typedef struct
{
  const char *name;
  float      *value;
  float       min;
  float       max;
} DebugParam_t;

/* 可调参数表：KPX 直接写 X=左右轴增益 mKpx，KPY 直接写 Y=前后轴增益 mKpy。 */
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
/* 接收异常只置位，恢复在默认任务执行；禁止中断内等待DMA停止。 */
static volatile uint8_t s_rx_recover;
static uint8_t s_rx_callbacks_ready;

static uint8_t s_tx[(4U * DEBUG_VOFA_CHANNELS) + 4U];

static volatile uint8_t s_stop_req;
static volatile uint8_t s_zero_req;
static volatile uint8_t s_offset_req;
static volatile float s_offset_x, s_offset_y;

/* GOTO 全局定位移动目标（上位机场地地图点击下发） */
static volatile uint8_t s_goto_active;
static float s_goto_x;
static float s_goto_y;
static float s_goto_z;

/* 手动速度命令必须持续刷新，PING不能延长其350ms有效期。 */
static volatile uint8_t s_manual_active;
static volatile int16_t s_manual_velocity[3];
static volatile uint32_t s_manual_tick;

/* 底盘四轮锁轴控制：接收中断只置请求，UART4发送由ZDT驱动非阻塞队列执行。
 * 0无请求、1使能（锁轴）、2失能（不锁轴）；同一周期后到的请求覆盖先到的。
 * s_wheel_enabled 由任务写、中断读：失能后不锁轴，GOTO/MANUAL/ZDT
 * 一律拒绝，必须显式 WHEELEN 恢复。main.c 启动时已使能四轮，初值为1。 */
static volatile uint8_t s_wheel_req;
static uint8_t s_wheel_enabled;

/* 单轮限时测试：中断发布请求，任务执行；阶段1等待使能，阶段2计时运行。
 * 测试最长5秒，不依赖主机心跳续期；不修改现有24通道遥测格式。 */
static volatile uint8_t s_zdt_req, s_zdt_active;
static volatile int16_t s_zdt_args[3];
static uint8_t s_zdt_addr;
static int16_t s_zdt_rpm;
static uint32_t s_zdt_tick, s_zdt_duration;

/* 文字应答采用事件队列，接收中断只入队，默认任务通过USART1 DMA发送。
 * ZDT切换至文字模式，避免JustFloat二进制内容干扰阅读；VOFA恢复遥测。
 * 队列满时丢弃新应答而不影响STOP处理；应答仅描述MCU状态，不代表电机回包。 */
static volatile uint8_t s_zdt_text_mode;
static volatile uint8_t s_ack_read, s_ack_write;
static uint8_t s_ack_queue[16];
static uint8_t s_ack_frames[16][4];
static uint8_t s_zdt_watch;
/* 单轮测试期间"期望的应答"：必须同时匹配 地址 + 功能码，状态码单独判读。
 * 只比地址是不够的：Stop(0xFE)/Enable(0xF3) 的回包同样是 4 字节且尾字节 0x6B，
 * 之前发过的命令的迟到回包会被当成速度(0xF6)命令的成功应答。 */
static uint8_t s_zdt_watch_cmd;
static uint32_t s_zdt_watch_tick;
static const char * const s_ack_text[] = {
  "ACK ZDT ACCEPTED\r\n",
  "ERR ZDT BUSY\r\n",
  "ERR ZDT FORMAT: ZDT=1..4,-300..300,1..5\r\n",
  "ACK ZDT RUN_REQUESTED (NO MOTOR ACK)\r\n",
  "ACK ZDT STOP_REQUESTED (NO MOTOR ACK)\r\n",
  "ACK ZDT CANCELLED (NO MOTOR ACK)\r\n",
  "WARN ZDT NO_VALID_REPLY IN 500MS\r\n",
  NULL, /* 事件7用于原始电机回包，不索引文本。 */
  "ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE\r\n",
  "ERR CAN DISABLED; DM/S28/S35 REJECTED\r\n",
  "ERR WHEEL DISABLED; WHEELEN FIRST\r\n",
  "ERR ZDT REPLY STATUS != 0x02 (SEE RAW FRAME)\r\n"
};

static void Debug_ZdtAck(uint8_t event)
{
  uint32_t mask = __get_PRIMASK();
  uint8_t next;
  __disable_irq();
  next = (uint8_t)((s_ack_write + 1U) % 16U);
  if (next != s_ack_read)
  {
    s_ack_queue[s_ack_write] = event;
    s_ack_write = next;
  }
  if (mask == 0U) __enable_irq();
}

/* 原始电机回包单独标记为事件7，通过同一个USART1发送队列输出。
 * 只在文字模式显示，正常VOFA模式仍持续排空接收队列以免积压旧回复。 */
static void Debug_ServiceZdtReplies(void)
{
  uint8_t frame[4], next, i;
  ZDT_X42S_ServiceRx();
  while (ZDT_X42S_PopReply(frame))
  {
    /* 只在"地址 + 期望功能码"都匹配时才认定本次测试收到了有效回包；
     * 状态码 0x02 才是命令正确应答，非 0x02（如 0xE2/0xEE 参数或保护错误）
     * 只报错、不当成功。原始 4 字节仍然照常打印，便于人工核对。 */
    if (s_zdt_watch && (frame[0] == s_zdt_addr) && (frame[1] == s_zdt_watch_cmd))
    {
      if (frame[2] == 0x02U)
      {
        s_zdt_watch = 0U;
      }
      else
      {
        s_zdt_watch = 0U;
        Debug_ZdtAck(11U);
      }
    }
    if (!s_zdt_text_mode) continue;
    {
      uint32_t mask = __get_PRIMASK();
      __disable_irq();
      next = (uint8_t)((s_ack_write + 1U) % 16U);
      if (next != s_ack_read)
      {
        for (i = 0U; i < 4U; ++i) s_ack_frames[s_ack_write][i] = frame[i];
        s_ack_queue[s_ack_write] = 7U;
        s_ack_write = next;
      }
      if (mask == 0U) __enable_irq();
    }
  }
  /* 仅检查本次目标电机是否有任一有效控制回包，不冒充逐条命令的事务确认。
   * 超时只给诊断提示，不延长测试时间；500ms后到达的回包仍正常输出。 */
  if (s_zdt_watch && (uint32_t)(HAL_GetTick() - s_zdt_watch_tick) >= 500U)
  {
    s_zdt_watch = 0U;
    Debug_ZdtAck(6U);
  }
}

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

/* 28/35 单次请求邮箱：串口中断只更新参数，任务提交 CAN。
 * 同一电机新请求替换尚未提交的旧请求；BUSY 最多重试1秒。
 * CANCEL/STOP只能撤销待发请求，不能停止已经发送的运动。 */
typedef struct
{
  uint32_t position;
  uint32_t queued_tick;
  uint16_t speed;
  uint8_t direction;
  uint8_t action; /* 0无请求，1回零，2机械目标，3原始计数 */
} DebugStepperRequest_t;
static volatile DebugStepperRequest_t s_stepper_req[2]; /* 35、28 */

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
 * @brief  解析逗号分隔的浮点列表，例如 "120.0,80.0,90.0"
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
/* 严格解析无符号十进制列表，拒绝溢出、负数、小数及多余字段。 */
static uint8_t Debug_ParseUnsignedList(const char *p, uint32_t *values, uint8_t count)
{
  uint8_t i;
  for (i = 0U; i < count; ++i)
  {
    uint32_t value = 0U;
    uint8_t digits = 0U;
    while (*p >= '0' && *p <= '9')
    {
      uint32_t digit = (uint32_t)(*p - '0');
      if (value > (UINT32_MAX - digit) / 10U) return 0U;
      value = value * 10U + digit;
      digits = 1U;
      ++p;
    }
    if (!digits) return 0U;
    values[i] = value;
    if (i + 1U < count)
    {
      if (*p != ',') return 0U;
      ++p;
    }
  }
  return *p == '\0';
}

static uint8_t Debug_ParseStepper(const char *line)
{
  uint8_t index;
  uint32_t v[3];
  uint8_t action;
  if (Debug_StrCaseCmpN(line, "S35", 3U) == 0U) index = 0U;
  else if (Debug_StrCaseCmpN(line, "S28", 3U) == 0U) index = 1U;
  else return 0U;
  line += 3;
  if (Debug_StrCaseCmp(line, "CANCEL") == 0U)
  {
    s_stepper_req[index].action = 0U;
    return 1U;
  }
  if (Debug_StrCaseCmp(line, "HOME") == 0U)
  {
    action = 1U;
    v[0] = v[1] = v[2] = 0U;
  }
  else if (Debug_StrCaseCmpN(line, "MOVE=", 5U) == 0U)
  {
    if (!Debug_ParseUnsignedList(line + 5, v, 2U)) return 1U;
    if (index == 0U)
    {
      if (v[0] < MOTOR35_HOME_HEIGHT - MOTOR35_MAX_TRAVEL ||
          v[0] > MOTOR35_HOME_HEIGHT || v[1] < 1U || v[1] > 2184U) return 1U;
    }
    else if (v[0] < MOTOR28_HOME_RADIUS ||
             v[0] > MOTOR28_HOME_RADIUS + MOTOR28_MAX_TRAVEL ||
             v[1] < 2U || v[1] > 65535U) return 1U;
    v[2] = v[1];
    v[1] = v[0];
    v[0] = 0U;
    action = 2U;
  }
  else if (Debug_StrCaseCmpN(line, "RAW=", 4U) == 0U)
  {
    if (!Debug_ParseUnsignedList(line + 4, v, 3U) ||
        v[0] > 1U || v[2] < 1U || v[2] > 65535U) return 1U;
    action = 3U;
  }
  else return 1U;
  s_stepper_req[index].direction = (uint8_t)v[0];
  s_stepper_req[index].position = v[1];
  s_stepper_req[index].speed = (uint16_t)v[2];
  s_stepper_req[index].queued_tick = HAL_GetTick();
  s_stepper_req[index].action = action;
  return 1U;
}

/* 短临界区保证请求不会在CAN提交期间被中断替换；驱动只提交邮箱，不等待。 */
static void Debug_ServiceSteppers(void)
{
  uint8_t i;
  for (i = 0U; i < 2U; ++i)
  {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t mask = __get_PRIMASK();
    uint16_t id = i == 0U ? MOTOR35_CAN_ID : MOTOR28_CAN_ID;
    __disable_irq();
    if ((uint32_t)(HAL_GetTick() - s_stepper_req[i].queued_tick) > DEBUG_HOST_TIMEOUT_MS)
      s_stepper_req[i].action = 0U;
    if (s_stepper_req[i].action == 1U)
      status = Motor_Homing(id);
    else if (s_stepper_req[i].action == 2U)
      status = i == 0U ? Motor35_AbsPosition(s_stepper_req[i].position, s_stepper_req[i].speed)
                       : Motor28_AbsPosition(s_stepper_req[i].position, s_stepper_req[i].speed);
    else if (s_stepper_req[i].action == 3U)
      status = Motor_AbsPosition(s_stepper_req[i].direction, id,
                                s_stepper_req[i].position, s_stepper_req[i].speed);
    if (status != HAL_BUSY) s_stepper_req[i].action = 0U;
    if (mask == 0U) __enable_irq();
  }
}

/* 严格解析三个整数速度分量，每轴范围 -300..300 RPM。 */
static uint8_t Debug_ParseManual(const char *s, int16_t *v)
{
  uint8_t i;
  for (i = 0U; i < 3U; ++i)
  {
    int sign = 1;
    int value = 0;
    if (*s == '-') { sign = -1; ++s; }
    else if (*s == '+') ++s;
    if (*s < '0' || *s > '9') return 0U;
    while (*s >= '0' && *s <= '9')
    {
      value = value * 10 + (*s++ - '0');
      if (value > 300) return 0U;
    }
    v[i] = (int16_t)(value * sign);
    if (i < 2U) { if (*s++ != ',') return 0U; }
    else if (*s != '\0') return 0U;
  }
  return 1U;
}

/* 底盘运动闸门：四轮失能后 GOTO/MANUAL 在解析阶段即被丢弃。
 * 由任务修改，接收中断只读，不阻塞也不改中断状态。 */
static uint8_t Debug_WheelReady(void)
{
  return (s_wheel_enabled != 0U) ? 1U : 0U;
}

/* 四轮失能后禁止再向 UART4 下发任何速度帧：ZDT_X42S 在速度模式下收到任意
 * 速度命令都会重新使能并锁轴，失能帧之后只要还有一条速度帧，轮子就会立刻
 * 重新锁住。因此失能后只清理软件目标，停车帧留给重新使能之后再发。 */
static void Debug_ChassisStop(void)
{
  if (Debug_WheelReady() != 0U)
  {
    MecanumControl_Stop();
  }
  else
  {
    MecanumControl_ClearTarget();
  }
}

/* OPS 复位/重连后坐标系会更换。旧 GOTO 和仍在生效的手动运动必须
 * 立即取消，避免继续按旧坐标参考驱动底盘；DM、28/35 步进功能不受影响。
 * 停车统一走 Debug_ChassisStop，遵守 WHEELOFF 的“只清目标、不发速度帧”。 */
static void Debug_CancelOpsSessionMotion(void)
{
  s_goto_active = 0U;
  s_manual_active = 0U;
  Debug_ChassisStop();
}

/* 结束单轮测试：被测轮发专用停止帧(0xFE 0x98)，四轮再统一恢复。
 * 测试开始时被测轮以外是 Disable（释放锁轴）；新驱动的失能闸门会拒绝
 * 这些地址的普通速度帧，因此启用状态下必须显式重新使能四轮，再走
 * Debug_ChassisStop 下发零速度恢复锁轴。失能状态下仍只清目标。 */
static void Debug_ZdtTestFinish(void)
{
  if ((s_wheel_enabled != 0U) && (s_wheel_req != 2U))
  {
    MecanumControl_Enable();
  }
  ZDT_X42S_Stop(s_zdt_addr);
  Debug_ChassisStop();
  s_zdt_active = 0U;
  s_zdt_watch = 0U;
}

static void Debug_ServiceManual(void)
{
  int16_t v[3];
  uint8_t active;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  active = s_manual_active;
  v[0] = s_manual_velocity[0];
  v[1] = s_manual_velocity[1];
  v[2] = s_manual_velocity[2];
  if (active && (uint32_t)(HAL_GetTick() - s_manual_tick) > 350U)
  {
    s_manual_active = 0U;
    v[0] = v[1] = v[2] = 0;
  }
  if (mask == 0U) __enable_irq();
  if (active == 0U) return;
  if (Debug_WheelReady() == 0U)
  {
    /* 失能状态下不得输出轮速，只清理目标，避免重新使能锁轴。 */
    MecanumControl_ClearTarget();
    return;
  }
  if (v[0] == 0 && v[1] == 0 && v[2] == 0) MecanumControl_Stop();
  /* MANUAL 与底盘统一坐标完全同序：X=左、Y=前、W=逆时针。 */
  else
  {
    MecanumControl_MoveVelocity((float)v[0], (float)v[1], (float)v[2]);
  }
}

/* 任务上下文执行四轮使能/失能：先取消运动并停车，再提交UART4发送队列。
 * 失能只释放锁轴，不改变DM、28/35状态，也不停止串口遥测。
 * 请求在中断中只置位，因此同一周期内只有最后一次状态切换生效。
 * 本函数在每个周期内最后执行，保证失能帧是该周期UART4上的最后一批帧；
 * 其余停车路径统一走Debug_ChassisStop，失能后不再产生速度帧。 */
static void Debug_ServiceWheel(void)
{
  uint8_t request;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  request = s_wheel_req;
  s_wheel_req = 0U;
  if (mask == 0U) __enable_irq();
  if (request == 0U) return;

  /* 切换使能状态前取消手动/GOTO并停车，避免带速使能或释放。 */
  s_manual_active = 0U;
  s_goto_active = 0U;
  MecanumControl_Stop();

  if (request == 1U)
  {
    /* 闸门在使能完成后才打开，使能等待期间不接受新的运动请求。 */
    MecanumControl_Enable();
    s_wheel_enabled = 1U;
    return;
  }

  /* 先关闭运动闸门，再清除本次调用前可能已被中断置位的运动请求。 */
  s_wheel_enabled = 0U;
  s_manual_active = 0U;
  s_goto_active = 0U;
  MecanumControl_Disable();
}

/* 任务上下文执行，禁止在接收中断中阻塞发送或延时。
 * STOP/ZERO/安装参数调整/四轮使能切换优先取消测试，且撤销尚未执行的请求。
 * 切入测试先停止四轮，随后只对选中地址重新使能、发送速度和专用停止帧。 */
static void Debug_ServiceZdt(void)
{
  uint8_t request, cancel;
  int16_t args[3];
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  /* 四轮使能切换必须取消测试：否则阶段1的重新使能会让已失能的轮子重新上电。 */
  cancel = (uint8_t)(s_stop_req || s_zero_req || s_offset_req || (s_wheel_req != 0U));
  request = s_zdt_req;
  args[0] = s_zdt_args[0]; args[1] = s_zdt_args[1]; args[2] = s_zdt_args[2];
  s_zdt_req = 0U;
  if (request && !cancel) s_zdt_active = 1U;
  if (mask == 0U) __enable_irq();
  if (cancel)
  {
    /* 先记住"当时是否在跑测试"，再收尾：Debug_ZdtTestFinish 会清 s_zdt_active，
     * 顺序写反会把 CANCELLED 应答吞掉。 */
    uint8_t was_active = s_zdt_active;
    if (was_active) Debug_ZdtTestFinish();
    if (was_active || request) Debug_ZdtAck(5U);
    s_zdt_active = 0U;
    s_zdt_watch = 0U;
    return;
  }
  if (request)
  {
    uint8_t addr, old_reply[4];
    /* 测试前丢弃旧回包，防止把启动时应答误认为此次测试回复。 */
    while (ZDT_X42S_PopReply(old_reply)) { }
    s_zdt_watch = 1U;
    s_zdt_watch_cmd = 0xF6U;   /* 本次测试只认速度帧(0xF6)的回包，见 Debug_ServiceZdtReplies */
    s_zdt_watch_tick = HAL_GetTick();
    /* 单轮测试的机械前提：被测轮以外**失能**（释放锁轴），而不是 Stop。
     * Stop 只是速度归零，在驱动器的 Hold 配置下轮子仍被锁住，会给悬空单轮
     * 测试带来额外机械阻力（相邻轮拖拽、电流偏大）。被测轮则先 Stop 复位内部
     * 状态，再 Enable，之后才真正受速度命令控制。
     * 测试结束/取消时用 MecanumControl_Stop() 把四轮恢复成"0 转速 + 锁轴"，
     * 与固件内部 s_wheel_enabled=1 的状态保持一致（速度帧本身即重新使能锁轴）。 */
    for (addr = 1U; addr <= 4U; ++addr)
    {
      if (addr == (uint8_t)args[0]) { ZDT_X42S_Stop(addr); }
      else                          { ZDT_X42S_Disable(addr); }
    }
    s_zdt_addr = (uint8_t)args[0];
    s_zdt_rpm = args[1];
    s_zdt_duration = (uint32_t)args[2] * 1000U;
    if (s_zdt_rpm == 0) { Debug_ZdtTestFinish(); Debug_ZdtAck(4U); return; }
    ZDT_X42S_Enable(s_zdt_addr);
    s_zdt_tick = HAL_GetTick();
  }
  if (s_zdt_active == 1U && (uint32_t)(HAL_GetTick() - s_zdt_tick) >= 100U)
  {
    ZDT_X42S_SpeedAcc(s_zdt_addr, s_zdt_rpm < 0 ? 1U : 0U,
                      (uint16_t)(s_zdt_rpm < 0 ? -s_zdt_rpm : s_zdt_rpm), 100U);
    s_zdt_tick = HAL_GetTick();
    s_zdt_active = 2U;
    Debug_ZdtAck(3U);
  }
  else if (s_zdt_active == 2U && (uint32_t)(HAL_GetTick() - s_zdt_tick) >= s_zdt_duration)
  {
    Debug_ZdtTestFinish();
    Debug_ZdtAck(4U);
  }
}

/* 严格校验两个十进制数，拒绝多余字段、NaN和尾随字符。 */
static uint8_t Debug_ParseOffset(const char *s, float *v)
{
  uint8_t i;
  for (i = 0U; i < 2U; ++i)
  {
    const char *start = s;
    uint8_t digits = 0U;
    if (*s == '+' || *s == '-') ++s;
    while (*s >= '0' && *s <= '9') { digits = 1U; ++s; }
    if (*s == '.')
    {
      ++s;
      while (*s >= '0' && *s <= '9') { digits = 1U; ++s; }
    }
    if (!digits || !Debug_ParseFloat(start, &v[i]) ||
        !(v[i] >= -500.0f && v[i] <= 500.0f)) return 0U;
    if (i == 0U) { if (*s++ != ',') return 0U; }
    else if (*s != '\0') return 0U;
  }
  return 1U;
}

/* CAN不可用时直接拒绝CAN电机命令，避免产生待执行请求。
 * 仅拦截DM、S28、S35命令；ZDT、STOP及串口其他功能保持可用。
 * 回复复用现有队列，由任务DMA发送，中断中不阻塞发送。
 * 这里**不再**切文字模式：CAN 故障不应连带关掉 24 通道遥测，
 * 否则上位机波形整体消失，且必须靠补发 VOFA 才能恢复。 */
static uint8_t Debug_RejectCanCommand(const char *line)
{
  if (hcan2.State != HAL_CAN_STATE_LISTENING &&
      (Debug_StrCaseCmpN(line, "DM", 2U) == 0U ||
       Debug_StrCaseCmpN(line, "S28", 3U) == 0U ||
       Debug_StrCaseCmpN(line, "S35", 3U) == 0U))
  {
    Debug_ZdtAck(9U);
    return 1U;
  }
  return 0U;
}

static void Debug_ParseLine(char *line)
{
  char *equal;
  float value;

  while ((*line == ' ') || (*line == '\t'))
  {
    ++line;
  }

  if (Debug_RejectCanCommand(line)) return;
  if (Debug_ParseStepper(line)) return;

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

  if (Debug_StrCaseCmpN(line, "OPSOFFSET=", 10U) == 0U)
  {
    float v[2];
    if (Debug_ParseOffset(line + 10U, v))
    {
      /* OPSOFFSET 与统一坐标同序：X=左偏移、Y=前偏移。
       * 例如 OPSOFFSET=60,-50 表示 OPS 装在车左60mm、车后50mm。 */
      s_offset_x = v[0]; s_offset_y = v[1];
      s_offset_req = 1U;
    }
    return;
  }

  if (Debug_StrCaseCmp(line, "VOFA") == 0U)
  {
    s_zdt_text_mode = 0U;
    return;
  }

  /* 底盘四轮锁轴/释放：中断只置请求，UART4发送由ZDT驱动非阻塞队列执行。
   * 失能只释放锁轴，不影响DM、28/35和串口遥测；失能期间的运动命令
   * 在本函数后面的MANUAL/GOTO/ZDT分支被丢弃，必须显式WHEELEN恢复。
   * STOP/ZERO/OPSOFFSET不是使能命令，失能后它们只清目标不发速度帧。 */
  if (Debug_StrCaseCmp(line, "WHEELEN") == 0U)
  {
    s_wheel_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "WHEELOFF") == 0U)
  {
    s_wheel_req = 2U;
    return;
  }

  /* ZDT=地址,有符号RPM,秒数；只允许底盘1~4号，限速300、限时1~5秒。
   * 测试期间拒绝新的单轮/MANUAL/GOTO命令，防止上位机周期刷新覆盖测试。
   * 接收成功仅表示请求入队，不代表电机已应答；STOP始终可以取消。 */
  if (Debug_StrCaseCmpN(line, "ZDT=", 4U) == 0U)
  {
    int16_t v[3];
    s_zdt_text_mode = 1U;
    if (!Debug_WheelReady())
    {
      Debug_ZdtAck(10U);
      return;
    }
    if (s_stop_req || s_zero_req || s_offset_req || s_zdt_active || s_zdt_req)
    {
      Debug_ZdtAck(1U);
      return;
    }
    if (
        Debug_ParseManual(line + 4U, v) && v[0] >= 1 && v[0] <= 4 && v[2] >= 1 && v[2] <= 5)
    {
      s_manual_active = s_goto_active = 0U;
      s_zdt_args[0] = v[0]; s_zdt_args[1] = v[1]; s_zdt_args[2] = v[2];
      s_zdt_req = 1U;
      Debug_ZdtAck(0U);
    }
    else Debug_ZdtAck(2U);
    return;
  }
  if ((s_zdt_active || s_zdt_req) &&
      (Debug_StrCaseCmpN(line, "MANUAL=", 7U) == 0U ||
       Debug_StrCaseCmpN(line, "GOTO=", 5U) == 0U)) return;

  if (Debug_StrCaseCmpN(line, "MANUAL=", 7U) == 0U)
  {
    int16_t v[3];
    if (!s_stop_req && !s_zero_req && Debug_WheelReady() &&
        Debug_ParseManual(line + 7U, v))
    {
      s_manual_velocity[0] = v[0];
      s_manual_velocity[1] = v[1];
      s_manual_velocity[2] = v[2];
      s_manual_tick = HAL_GetTick();
      s_goto_active = 0U;
      s_manual_active = 1U;
    }
    return;
  }

  /* GOTO=x,y,z：OPS 全局定位移动目标，x/y 单位 cm（1位小数），z 可省略（保持当前航向）。
   * 对外坐标范围 ±300.0 cm；按统一轴序直接换算为内部 mm。
   * 四轮失能时不接受新目标，避免释放状态下位置环持续输出轮速。 */
  if ((Debug_StrCaseCmpN(line, "GOTO", 4U) == 0U) && (line[4] == '='))
  {
    float v[3] = {0.0f, 0.0f, 0.0f};
    uint8_t n = Debug_ParseFloatList(line + 5U, v, 3U);

    if ((n >= 2U) && (Debug_WheelReady() != 0U))
    {
      if (v[0] < -300.0f) { v[0] = -300.0f; }
      if (v[0] > 300.0f)  { v[0] = 300.0f; }
      if (v[1] < -300.0f) { v[1] = -300.0f; }
      if (v[1] > 300.0f)  { v[1] = 300.0f; }

      /* cm → mm：协议只表达 0.1cm，转换为整数 mm 后直接进入位置环。
       * 正负分别加减 0.5 后向零取整，等价于按最近 1mm 取整。 */
      v[0] = (v[0] >= 0.0f)
           ? (float)(int32_t)((v[0] * 10.0f) + 0.5f)
           : (float)(int32_t)((v[0] * 10.0f) - 0.5f);
      v[1] = (v[1] >= 0.0f)
           ? (float)(int32_t)((v[1] * 10.0f) + 0.5f)
           : (float)(int32_t)((v[1] * 10.0f) - 0.5f);

      s_manual_active = 0U;
      /* GOTO=X,Y,Z 与底盘统一坐标同序：X=场地左、Y=场地前，单位已换算为 mm。 */
      s_goto_x = v[0];
      s_goto_y = v[1];
      if (n >= 3U)
      {
        /* 目标航向必须钳位并拒 NaN/Inf：否则 devz 可能变成 Inf，
         * 位置环里的回绕会永不退出（20ms 任务永久挂死）。
         * 取反写法 (!(x >= -3600 && x <= 3600)) 可同时拒绝 NaN。 */
        if (!(v[2] >= -3600.0f && v[2] <= 3600.0f))
        {
          s_goto_active = 0U;
          return;
        }
        s_goto_z = v[2];
      }
      else
      {
        /* n == 2：Z 未提供，保持当前航向。 */
        s_goto_z = zangle;
      }
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

/* USART1专用错误回调，不覆盖OPS和电机UART4的错误处理。
 * HAL在DMA接收错误后会终止接收；回调仅申请恢复，不操作TX。 */
static void DebugUsart_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) s_rx_recover = 1U;
}

/* 普通接收回调和任务共用启动逻辑。先清恢复标志再启动，避免覆盖
 * 启动完成后新到达的错误通知；短临界区内不调用任何阻塞等待接口。 */
static void DebugUsart_StartRx(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  s_rx_recover = 0U;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx)) == HAL_OK)
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  else
    s_rx_recover = 1U;
  if (mask == 0U) __enable_irq();
}

/* 默认任务周期恢复RX，不使用HAL_UART_DMAStop/Abort，避免连带中止TX。
 * HAL异步DMA终止尚未完成时先等待；失败保留标志，下周期继续尝试。
 * 恢复丢弃半条命令，且不更新主机心跳，原有失联保护继续生效。 */
static void DebugUsart_ServiceRx(void)
{
  if (!s_rx_recover) return;
  if (!s_rx_callbacks_ready)
  {
    /* HAL仅允许在发送状态空闲时注册；忙时不打断发送。 */
    if (huart1.gState != HAL_UART_STATE_READY) return;
    if (HAL_UART_RegisterRxEventCallback(&huart1, DebugUsart_RxEventCallback) != HAL_OK ||
        HAL_UART_RegisterCallback(&huart1, HAL_UART_ERROR_CB_ID, DebugUsart_ErrorCallback) != HAL_OK)
      return;
    s_rx_callbacks_ready = 1U;
  }
  if (HAL_DMA_GetState(huart1.hdmarx) == HAL_DMA_STATE_ABORT) return;
  if (HAL_UART_AbortReceive(&huart1) != HAL_OK) return;
  /* 部分DMA错误路径已清DMAR，但DMA句柄仍忙，单独清理RX数据流。 */
  if (HAL_DMA_GetState(huart1.hdmarx) != HAL_DMA_STATE_READY &&
      HAL_DMA_Abort(huart1.hdmarx) != HAL_OK) return;
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  s_line_len = 0U;
  DebugUsart_StartRx();
}

/* --------------------------- 对外接口 ------------------------------ */



/**
 * @brief  初始化 USART1 调试接收
 */
void DebugUsart_Init(void)
{
  memset((void *)s_stepper_req, 0, sizeof(s_stepper_req));
  s_offset_req = 0U;
  s_manual_active = 0U;
  s_line_len = 0U;
  s_stop_req = 0U;
  s_zero_req = 0U;
  s_goto_active = 0U;
  s_goto_x = 0.0f;
  s_goto_y = 0.0f;
  s_goto_z = 0.0f;
  /* main.c 在调用本函数前已使能四轮，这里只清除未处理的切换请求。 */
  s_wheel_req = 0U;
  s_wheel_enabled = 1U;

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

  /* CAN启动失败仍继续启用串口RX；提示排队等待DMA空闲，不阻塞控制任务。
   * 这里**不再**置 s_zdt_text_mode：旧实现让 CAN 失败顺带关掉整个 24 通道遥测
   * （只有收到 VOFA 才恢复），上位机表现为"连接后一直没有数据、之后串口卡死"。
   * 报错和遥测可以并存，因此只排队报错，不动遥测开关。 */
  if (hcan2.State != HAL_CAN_STATE_LISTENING)
  {
    Debug_ZdtAck(8U);
  }

  /* 启动失败也保留恢复请求，由默认任务重试。 */
  s_rx_callbacks_ready = 0U;
  s_rx_recover = 1U;

  DebugUsart_ServiceRx();
}

/**
 * @brief  发送一次 VOFA+ JustFloat 数据帧
 */
void DebugUsart_Send(void)
{
  float data[DEBUG_VOFA_CHANNELS];
  float user_x, user_y;      /* 统一坐标：X=左右(+左)、Y=前后(+车头) */
  float err_x, err_y;        /* 统一坐标下的位置误差 */
  DmJ4310Feedback_t dmFb;
  uint32_t i;
  uint32_t len;
  uint32_t primask;

  /* OPS 错误恢复和会话变化必须先于遥测/运动服务处理。OPS 重启后
   * 坐标系原点会变化，继续执行旧 GOTO 会产生错误方向，因此立即取消。 */
  OPS_ServiceRx();
  if (OPS_ConsumeSessionChanged() != 0U)
  {
    Debug_CancelOpsSessionMotion();
  }

  /* UART4发送失败只在任务上下文重试；正常发送由TX完成回调推进。 */
  ZDT_X42S_ServiceTx();
  DebugUsart_ServiceRx();
  Debug_ServiceZdtReplies();
  Debug_ServiceZdt();

  /* 上位机失联时停止仍在运行的调试动作 */
  if ((uint32_t)(HAL_GetTick() - s_host_last_tick) > DEBUG_HOST_TIMEOUT_MS)
  {
    s_stepper_req[0].action = s_stepper_req[1].action = 0U;
    if (s_goto_active != 0U)
    {
      s_goto_active = 0U;
      Debug_ChassisStop();
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
    s_stepper_req[0].action = s_stepper_req[1].action = 0U;
    s_manual_active = 0U;
    s_stop_req = 0U;
    s_goto_active = 0U;
    /* STOP 不是使能命令：四轮已失能时只清目标，不下发速度帧。 */
    Debug_ChassisStop();
    s_dm_active = 0U;
    s_dm_start_pending = 0U;
    s_dm_enable_req = 0U;
    s_dm_mode_req = 0U;
    (void)DmJ4310_Disable(s_dm_id);
  }

  if (s_zero_req != 0U)
  {
    s_manual_active = 0U;
    s_zero_req = 0U;
    s_goto_active = 0U;
    Debug_ChassisStop();
    OPS_ZeroCoordinates();
  }

  /* 应用安装参数前取消底盘动作；耗时停车/三角运算不在中断中执行。 */
  if (s_offset_req)
  {
    float x_mm, y_mm;
    primask = __get_PRIMASK();
    __disable_irq();
    x_mm = s_offset_x; y_mm = s_offset_y;   /* X=左偏移、Y=前偏移 */
    s_offset_req = 0U;
    s_manual_active = s_goto_active = 0U;
    if (primask == 0U) __enable_irq();
    Debug_ChassisStop();
    (void)OPS_SetMountOffset(x_mm, y_mm);
  }

  /* GOTO 定位移动：本任务 20ms 周期执行一步 P 控制并下发轮速，
     maxRpm 传 0 表示沿用当前调试限幅（XVMAX/ZVMAX），STOP 可随时取消 */
  if ((s_goto_active != 0U) && (Debug_WheelReady() == 0U))
  {
    /* 四轮已失能：位置环不得输出轮速，只取消目标。 */
    s_goto_active = 0U;
    MecanumControl_ClearTarget();
  }
  if (s_goto_active != 0U)
  {
    if (OPS_IsOnline(DEBUG_OPS_TIMEOUT_MS) == 0U)
    {
      s_goto_active = 0U;
      Debug_ChassisStop();
    }
    else if (MecanumControl_GotoOPS(s_goto_x, s_goto_y, s_goto_z, 0.0f) != 0U)
    {
      s_goto_active = 0U;
      Debug_ChassisStop();
    }
  }

  Debug_ServiceManual();

  /* 四轮使能切换在ZDT服务之后，也在所有停车/清目标路径之后执行：
   * 正在运行的测试先按同一请求取消，且失能帧是该周期UART4上的最后一批帧，
   * 不会被STOP/ZERO/OPSOFFSET/GOTO随后发出的速度帧重新使能锁轴。 */
  Debug_ServiceWheel();

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

  Debug_ServiceSteppers();

  /* 控制服务始终执行；DMA 忙时仅跳过遥测，禁止改写仍在发送的 s_tx。
     本函数由 defaultTask 单独调用，USART1 TX 缓冲不得交给其他任务共用。 */
  if (huart1.gState != HAL_UART_STATE_READY) return;

  /* 发送缓冲区仅在DMA空闲时改写；成功提交才消费事件，忙/失败留待下次重试。
   * 应答优先于遥测；切换瞬间可能仍有上一帧遥测在途，不截断正在发送的DMA。 */
  if (s_ack_read != s_ack_write)
  {
    uint16_t reply_len;
    if (s_ack_queue[s_ack_read] == 7U)
    {
      static const char hex[] = "0123456789ABCDEF";
      uint8_t j;
      /* 保留原始状态码，不将未知状态解释为成功；02仅代表命令正确应答。 */
      memcpy(s_tx, "ZDT RX: ", 8U);
      for (j = 0U; j < 4U; ++j)
      {
        uint8_t value = s_ack_frames[s_ack_read][j];
        s_tx[8U + j * 3U] = (uint8_t)hex[value >> 4];
        s_tx[9U + j * 3U] = (uint8_t)hex[value & 15U];
        s_tx[10U + j * 3U] = ' ';
      }
      s_tx[19] = '\r'; s_tx[20] = '\n';
      reply_len = 21U;
    }
    else
    {
      const char *reply = s_ack_text[s_ack_queue[s_ack_read]];
      reply_len = (uint16_t)strlen(reply);
      memcpy(s_tx, reply, reply_len);
    }
    if (HAL_UART_Transmit_DMA(&huart1, s_tx, reply_len) == HAL_OK)
      s_ack_read = (uint8_t)((s_ack_read + 1U) % 16U);
    return;
  }
  if (s_zdt_text_mode) return;

  /* 读取 DM 反馈快照 */
  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(&dmFb, &s_dm_feedback, sizeof(DmJ4310Feedback_t));
  if (primask == 0U)
  {
    __enable_irq();
  }

  /* 手动旋转/静止调试同样刷新补偿后位置，不依赖GOTO运行。 */
  MecanumControl_GetPose(&pos_x, &pos_y, &zangle);
  /* 底盘内部已经使用统一坐标：ch0=X=左右、ch1=Y=前后、ch2=Z=航向角。
   * ch6/ch7 直接回读 mKpx/mKpy；通道总数与帧格式不变。 */
  user_x = pos_x;
  user_y = pos_y;
  err_x = devx;
  err_y = devy;
  /* 对外位置/误差统一为 cm；内部 pose/误差仍为 mm，仅在此打包边界缩放。 */
  data[0]  = user_x * 0.1f;
  data[1]  = user_y * 0.1f;
  data[2]  = zangle;
  data[3]  = err_x * 0.1f;
  data[4]  = err_y * 0.1f;
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

  /* 恢复期间可能仍有旧DMA事件到达，不解析可能损坏的半帧。 */
  if (s_rx_recover) return;

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

  /* 正常包立即重启；失败后由任务清理RX状态并重试。 */
  DebugUsart_StartRx();
}


/**
 * @brief  CAN 接收回调：解析 DM 电机反馈并缓存
 * @note   实现 hcan.c 中的 __weak 钩子
 */
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
  DmJ4310Feedback_t fb;
  uint8_t rxData[8];

  if ((hcan == NULL) || (hcan->Instance != CAN2))
  {
    return;
  }

  if (hcanRxFrame.RTR != CAN_RTR_DATA || hcanRxFrame.DLC > 8U) return;

  if (hcanRxFrame.IDE == CAN_ID_EXT)
  {
    memcpy(rxData, (const uint8_t *)hcanRxFrame.Data, sizeof(rxData));
    (void)Stepper2835_OnRx(hcanRxFrame.ExtId, rxData, hcanRxFrame.DLC);
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
