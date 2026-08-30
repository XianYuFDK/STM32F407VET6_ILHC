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
