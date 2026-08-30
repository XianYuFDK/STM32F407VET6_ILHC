/**
 ******************************************************************************
 * @file    zdt_x42s.h
 * @brief   张大头 ZDT_X42S 第二代闭环步进电机驱动（硬件层）
 *
 *          - 使用 UART4（PA0=TX，PA1=RX）通信，默认 115200/8N1
 *          - X42S 出厂默认 Emm5.0 固件，采用 Emm 速度模式：
 *              地址 | 0xF6 | 方向 | 速度RPM高8位 | 速度RPM低8位 | 加速度 | 同步 | 0x6B
 *          - 支持使能、失能、立即停止、速度模式控制
 ******************************************************************************
 */
#ifndef __ZDT_X42S_H__
#define __ZDT_X42S_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------------------- 运动方向 ---------------------------- */
#define ZDT_X42S_DIR_CW     0U  /* 顺时针/正转 */
#define ZDT_X42S_DIR_CCW    1U  /* 逆时针/反转 */

/* ---------------------------- 默认参数 ---------------------------- */
#define ZDT_X42S_MAX_RPM        3000U  /* Emm 速度模式最大 3000 RPM */
#define ZDT_X42S_DEFAULT_ACC    8U     /* 加速档位，0 表示直接启动，1~255 越大加速越快 */

/* --------------------------- 对外接口 ----------------------------- */

/**
 * @brief  使能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 */
void ZDT_X42S_Enable(uint8_t addr);

/**
 * @brief  失能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 */
void ZDT_X42S_Disable(uint8_t addr);

/**
 * @brief  立即停止指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 */
void ZDT_X42S_Stop(uint8_t addr);

/**
 * @brief  速度模式控制电机连续转动（使用默认加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 */
void ZDT_X42S_Speed(uint8_t addr, uint8_t dir, uint16_t rpm);

/**
 * @brief  速度模式控制电机连续转动（自定义加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 * @param  acc  加速度档位，0~255；0 为直接启动
 */
void ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_X42S_H__ */
