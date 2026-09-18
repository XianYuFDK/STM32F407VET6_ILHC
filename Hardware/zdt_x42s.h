/**
 ******************************************************************************
 * @file    zdt_x42s.h
 * @brief   张大头 ZDT_X42S 第二代闭环步进电机驱动（硬件层）
 *
 *          - 使用 UART4（PA0=TX，PA1=RX）通信，默认 115200/8N1
 *          - X42S 出厂默认 Emm5.0 固件，采用 Emm 速度模式：
 *              地址 | 0xF6 | 方向 | 速度RPM高8位 | 速度RPM低8位 | 加速度 | 同步 | 0x6B
 *          - 支持使能、失能、立即停止、速度模式控制
 *          - UART4 TX 使用中断发送和固定长度优先队列，不阻塞控制任务
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

/** 初始化UART4非阻塞发送队列和TX完成回调；必须在MX_UART4_Init后、
 *  发送电机命令前调用。 */
HAL_StatusTypeDef ZDT_X42S_InitTx(void);
/** 默认任务周期调用；只重试上一次未成功提交的HAL发送，不阻塞。 */
void ZDT_X42S_ServiceTx(void);

/** 初始化UART4逐字节中断接收；必须在MX_UART4_Init后、发送电机命令前调用。
 * 仅解析F3/F6/FE四字节控制应答，固定校验尾6B；不处理位置/速度查询长帧。
 * 不覆盖USART1/USART2的回调，支持与TX中断同时运行。 */
HAL_StatusTypeDef ZDT_X42S_InitRx(void);
/** 默认任务周期调用；接收出错后仅重启RX，不中止TX。 */
void ZDT_X42S_ServiceRx(void);
/** 弹出最早一帧电机应答，返回1表示成功，0表示无数据；单任务消费。
 * reply至少4字节；队列最多缓存15帧，满时丢弃新帧。 */
uint8_t ZDT_X42S_PopReply(uint8_t reply[4]);


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
