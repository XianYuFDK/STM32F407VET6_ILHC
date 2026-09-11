/**
 ******************************************************************************
 * @file    zdt_x42s.c
 * @brief   张大头 ZDT_X42S 第二代闭环步进电机驱动（硬件层）
 *
 *          指令格式参考《ZDT_X42S第二代闭环步进电机用户手册V1.0.4》：
 *          - 使能：地址 0xF3 0xAB 状态 同步 0x6B
 *          - 停止：地址 0xFE 0x98 同步 0x6B
 *          - 速度：地址 0xF6 方向 速度H 速度L 加速度 同步 0x6B
 *          - 校验尾固定 0x6B
 *
 *          波特率默认 115200，使用 UART4（PA0=TX，PA1=RX）。
 ******************************************************************************
 */
#include "zdt_x42s.h"
#include "usart.h"

/* --------------------------- 底层参数 ----------------------------- */
#define ZDT_X42S_UART              huart4
#define ZDT_X42S_TX_TIMEOUT_MS     100U
#define ZDT_X42S_FRAME_TAIL        0x6BU

/* --------------------------- 私有函数 ----------------------------- */

/**
 * @brief  向电机驱动器发送一帧命令
 * @param  cmd 命令缓冲区
 * @param  len 命令长度
 */
static void ZDT_X42S_Send(uint8_t *cmd, uint16_t len)
{
  if ((cmd == NULL) || (len == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(&ZDT_X42S_UART, cmd, len, ZDT_X42S_TX_TIMEOUT_MS);
}

/* --------------------------- 对外接口 ----------------------------- */

/**
 * @brief  使能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 */
void ZDT_X42S_Enable(uint8_t addr)
{
  uint8_t cmd[6];

  cmd[0] = addr;       /* 地址       */
  cmd[1] = 0xF3U;      /* 功能码     */
  cmd[2] = 0xABU;      /* 辅助码     */
  cmd[3] = 0x01U;      /* 使能状态   */
  cmd[4] = 0x00U;      /* 同步标志   */
  cmd[5] = ZDT_X42S_FRAME_TAIL;

  ZDT_X42S_Send(cmd, sizeof(cmd));
}

/**
 * @brief  失能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 */
void ZDT_X42S_Disable(uint8_t addr)
{
  uint8_t cmd[6];

  cmd[0] = addr;       /* 地址       */
  cmd[1] = 0xF3U;      /* 功能码     */
  cmd[2] = 0xABU;      /* 辅助码     */
  cmd[3] = 0x00U;      /* 使能状态   */
  cmd[4] = 0x00U;      /* 同步标志   */
  cmd[5] = ZDT_X42S_FRAME_TAIL;

  ZDT_X42S_Send(cmd, sizeof(cmd));
}

/**
 * @brief  立即停止指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 */
void ZDT_X42S_Stop(uint8_t addr)
{
  uint8_t cmd[5];

  cmd[0] = addr;       /* 地址       */
  cmd[1] = 0xFEU;      /* 功能码     */
  cmd[2] = 0x98U;      /* 辅助码     */
  cmd[3] = 0x00U;      /* 同步标志   */
  cmd[4] = ZDT_X42S_FRAME_TAIL;

  ZDT_X42S_Send(cmd, sizeof(cmd));
}

/**
 * @brief  速度模式控制电机连续转动（使用默认加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 */
void ZDT_X42S_Speed(uint8_t addr, uint8_t dir, uint16_t rpm)
{
  ZDT_X42S_SpeedAcc(addr, dir, rpm, ZDT_X42S_DEFAULT_ACC);
}

/**
 * @brief  速度模式控制电机连续转动（自定义加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 * @param  acc  加速度档位，0~255；0 为直接启动
 */
void ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
  uint8_t cmd[8];

  if (rpm > ZDT_X42S_MAX_RPM)
  {
    rpm = ZDT_X42S_MAX_RPM;
  }

  cmd[0] = addr;                          /* 地址         */
  cmd[1] = 0xF6U;                         /* 功能码       */
  cmd[2] = dir;                           /* 方向         */
  cmd[3] = (uint8_t)(rpm >> 8);           /* 速度高8位    */
  cmd[4] = (uint8_t)(rpm & 0xFFU);        /* 速度低8位    */
  cmd[5] = acc;                           /* 加速度档位   */
  cmd[6] = 0x00U;                         /* 同步标志     */
  cmd[7] = ZDT_X42S_FRAME_TAIL;           /* 校验尾       */

  ZDT_X42S_Send(cmd, sizeof(cmd));
}


/* UART4应答接收：中断只做四字节滑窗和入队，不打印、不发送运动命令。
 * 帧间断开超过20ms即丢弃残帧；噪声采用逐字节滑动重新同步。
 * 6B是固定校验字节而非CRC，因此有效格式不等于物理运动已完成。 */
static uint8_t s_rx_byte, s_rx_window[4], s_rx_count;
static uint8_t s_reply_queue[16][4];
static volatile uint8_t s_reply_read, s_reply_write, s_rx_fault;
static uint32_t s_rx_tick;

static void ZDT_X42S_RxComplete(UART_HandleTypeDef *uart)
{
  uint8_t i, next;
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - s_rx_tick) > 20U) s_rx_count = 0U;
  s_rx_tick = now;
  if (s_rx_count == 4U)
  {
    for (i = 0U; i < 3U; ++i) s_rx_window[i] = s_rx_window[i + 1U];
    s_rx_count = 3U;
  }
  s_rx_window[s_rx_count++] = s_rx_byte;
  if (s_rx_count == 4U && s_rx_window[0] != 0U &&
      (s_rx_window[1] == 0xF3U || s_rx_window[1] == 0xF6U || s_rx_window[1] == 0xFEU) &&
      s_rx_window[3] == 0x6BU)
  {
    next = (uint8_t)((s_reply_write + 1U) % 16U);
    if (next != s_reply_read)
    {
      for (i = 0U; i < 4U; ++i) s_reply_queue[s_reply_write][i] = s_rx_window[i];
      s_reply_write = next;
    }
    s_rx_count = 0U;
  }
  if (HAL_UART_Receive_IT(uart, &s_rx_byte, 1U) != HAL_OK) s_rx_fault = 1U;
}

static void ZDT_X42S_RxError(UART_HandleTypeDef *uart)
{
  (void)uart;
  /* 奇偶、帧、噪声或溢出错误交由任务恢复，避免在中断里阻塞。 */
  s_rx_fault = 1U;
}

HAL_StatusTypeDef ZDT_X42S_InitRx(void)
{
  s_rx_count = s_reply_read = s_reply_write = 0U;
  s_rx_fault = 1U;
  if (HAL_UART_RegisterCallback(&ZDT_X42S_UART, HAL_UART_RX_COMPLETE_CB_ID,
                                ZDT_X42S_RxComplete) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_RegisterCallback(&ZDT_X42S_UART, HAL_UART_ERROR_CB_ID,
                                ZDT_X42S_RxError) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_Receive_IT(&ZDT_X42S_UART, &s_rx_byte, 1U) != HAL_OK) return HAL_ERROR;
  s_rx_fault = 0U;
  return HAL_OK;
}

void ZDT_X42S_ServiceRx(void)
{
  if (s_rx_fault)
  {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* UART4没有RX DMA，AbortReceive只关闭接收并复位状态，不等待DMA。 */
    (void)HAL_UART_AbortReceive(&ZDT_X42S_UART);
    __HAL_UART_CLEAR_OREFLAG(&ZDT_X42S_UART);
    s_rx_count = 0U;
    if (HAL_UART_Receive_IT(&ZDT_X42S_UART, &s_rx_byte, 1U) == HAL_OK) s_rx_fault = 0U;
    if (mask == 0U) __enable_irq();
  }
}

uint8_t ZDT_X42S_PopReply(uint8_t reply[4])
{
  uint8_t i;
  uint32_t mask;
  if (reply == NULL) return 0U;
  mask = __get_PRIMASK();
  __disable_irq();
  if (s_reply_read == s_reply_write)
  {
    if (mask == 0U) __enable_irq();
    return 0U;
  }
  for (i = 0U; i < 4U; ++i) reply[i] = s_reply_queue[s_reply_read][i];
  s_reply_read = (uint8_t)((s_reply_read + 1U) % 16U);
  if (mask == 0U) __enable_irq();
  return 1U;
}
