/**
 ******************************************************************************
 * @file    debug_usart.c
 * @brief   USART1 调试模块（VOFA+ 调参）
 *
 *          - VOFA+ JustFloat 数据帧：0x55 0xAA + N*float + 0x00 0x00 0x80 0x7F
 *          - DMA 空闲接收 ASCII 命令：KPX=3.0、XVMAX=1600、STOP、ZERO
 *          - 参数表可扩展：在 DebugParam_t 表中增加一项即可
 ******************************************************************************
 */
#include "debug_usart.h"
#include "usart.h"
#include "mecanum_control.h"
#include "ops.h"

#include <string.h>


/* --------------------------- 调试参数 ------------------------------ */
#define DEBUG_RX_SIZE     256U
#define DEBUG_LINE_SIZE   64U
#define DEBUG_VOFA_HEAD0  0x55U
#define DEBUG_VOFA_HEAD1  0xAAU
#define DEBUG_VOFA_TAIL0  0x00U
#define DEBUG_VOFA_TAIL1  0x00U
#define DEBUG_VOFA_TAIL2  0x80U
#define DEBUG_VOFA_TAIL3  0x7FU

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

static uint8_t s_tx[2U + (4U * DEBUG_VOFA_CHANNELS) + 4U];

static volatile uint8_t s_stop_req;
static volatile uint8_t s_zero_req;

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

  if (Debug_StrCaseCmp(line, "ZERO") == 0U)
  {
    s_zero_req = 1U;
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
    Debug_SetParam(line, value);
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

  /* 注册 USART1 专用接收回调，不影响 OPS 的 USART2 回调 */
  (void)HAL_UART_RegisterRxEventCallback(&huart1, DebugUsart_RxEventCallback);

  /* 启动空闲中断 + DMA 接收 */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx));
}

/**
 * @brief  发送一次 VOFA+ JustFloat 数据帧
 */
void DebugUsart_Send(void)
{
  float data[DEBUG_VOFA_CHANNELS];
  uint32_t i;
  uint32_t len;

  /* 处理等待执行的命令 */
  if (s_stop_req != 0U)
  {
    s_stop_req = 0U;
    MecanumControl_Stop();
  }

  if (s_zero_req != 0U)
  {
    s_zero_req = 0U;
    OPS_ZeroCoordinates();
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

  s_tx[0] = DEBUG_VOFA_HEAD0;
  s_tx[1] = DEBUG_VOFA_HEAD1;

  for (i = 0U; i < DEBUG_VOFA_CHANNELS; ++i)
  {
    memcpy(&s_tx[2U + (4U * i)], &data[i], 4U);
  }

  s_tx[2U + (4U * DEBUG_VOFA_CHANNELS)]     = DEBUG_VOFA_TAIL0;
  s_tx[2U + (4U * DEBUG_VOFA_CHANNELS) + 1U] = DEBUG_VOFA_TAIL1;
  s_tx[2U + (4U * DEBUG_VOFA_CHANNELS) + 2U] = DEBUG_VOFA_TAIL2;
  s_tx[2U + (4U * DEBUG_VOFA_CHANNELS) + 3U] = DEBUG_VOFA_TAIL3;
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
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx));
}
