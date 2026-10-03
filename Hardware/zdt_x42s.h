/**
 ******************************************************************************
 * @file    zdt_x42s.h
 * @brief   张大头 ZDT_X42S 第二代闭环步进电机驱动（硬件层）
 *
 *          - 使用 UART4（PA0=TX，PA1=RX）通信，默认 115200/8N1
 *          - X42S 出厂默认 Emm5.0 固件，采用 Emm 速度模式：
 *              地址 | 0xF6 | 方向 | 速度RPM高8位 | 速度RPM低8位 | 加速度 | 同步 | 0x6B
 *          - 支持使能、失能、立即停止、速度模式控制
 *          - UART4 发送为"中断发送 + 固定长度优先队列"，不阻塞控制任务；
 *            因此下发类接口的返回值只表示"是否已入队"，发送结果由
 *            ZDT_X42S_GetTxErrorCount() 反映
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

/** 初始化UART4非阻塞发送（中断发送 + 固定长度优先队列）。
 * 必须在 MX_UART4_Init 之后、提交任何电机命令之前调用；调度器启动前 main 里的
 * 首次停车/使能也依赖它。返回 HAL_ERROR 时不应继续下发运动命令。 */
HAL_StatusTypeDef ZDT_X42S_InitTx(void);
/** TIM7 1ms 回调调用；帧间空闲到期后以中断方式启动下一帧。 */
void ZDT_X42S_TxTick(void);
/** 默认任务周期调用；处理"发送卡住"与重试预算，超限时计入发送失败。不阻塞。 */
void ZDT_X42S_ServiceTx(void);

/** 初始化UART4逐字节中断接收；必须在MX_UART4_Init后、发送电机命令前调用。
 * 仅解析F3/F6/FE四字节控制应答，固定校验尾6B；不处理位置/速度查询长帧。
 * 不覆盖USART1/USART2的回调，与TX中断同时运行。 */
HAL_StatusTypeDef ZDT_X42S_InitRx(void);
/** 默认任务周期调用；接收出错后仅重启RX，不中止TX。 */
void ZDT_X42S_ServiceRx(void);
/** 弹出最早一帧电机应答，返回1表示成功，0表示无数据；单任务消费。
 * reply至少4字节；队列最多缓存15帧，满时丢弃新帧。 */
uint8_t ZDT_X42S_PopReply(uint8_t reply[4]);

/** 获取累计发送失败帧数；只升不降。
 * debug_usart.c 的 Debug_ServiceWheelFault 靠它变化来锁存底盘故障，因此它必须
 * 反映"帧最终没发出去"（在途超时或超过重试预算），而不只是队列是否收下。
 * 任务上下文读取，不在中断里使用。 */
uint32_t ZDT_X42S_GetTxErrorCount(void);


/**
 * @brief  使能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @retval HAL_OK 已入队；HAL_ERROR 未接受（未InitTx/队列满/参数非法）
 */
HAL_StatusTypeDef ZDT_X42S_Enable(uint8_t addr);

/**
 * @brief  失能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @retval HAL_OK 已入队；HAL_ERROR 未接受
 */
HAL_StatusTypeDef ZDT_X42S_Disable(uint8_t addr);

/**
 * @brief  立即停止指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @retval HAL_OK 已入队；HAL_ERROR 未接受
 */
HAL_StatusTypeDef ZDT_X42S_Stop(uint8_t addr);

/**
 * @brief  速度模式控制电机连续转动（使用默认加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 * @retval HAL_OK 已入队；HAL_ERROR 未接受
 */
HAL_StatusTypeDef ZDT_X42S_Speed(uint8_t addr, uint8_t dir, uint16_t rpm);

/**
 * @brief  速度模式控制电机连续转动（自定义加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 * @param  acc  加速度档位，0~255；0 为直接启动
 * @retval HAL_OK 已入队；HAL_ERROR 未接受。同类速度帧按地址覆盖，不排队积压
 */
HAL_StatusTypeDef ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_X42S_H__ */
