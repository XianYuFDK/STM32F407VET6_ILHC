/**
 ******************************************************************************
 * @file    debug_usart.h
 * @brief   USART1 调试模块（VOFA+ 调参）
 *
 *          - USART1：PA9=TX，PA10=RX，115200/8N1
 *          - TX：DMA 中断发送 VOFA+ JustFloat 数据帧
 *          - RX：DMA 空闲中断接收 ASCII 命令，可在线调节底盘 / DM 电机参数
 *
 *          VOFA+ 使用方式：
 *          - 数据协议选择 JustFloat
 *          - 坐标约定（对外统一）：+X=小车左方、+Y=小车正前方、+Z=逆时针为正；
 *            X为左右轴、Y为前后轴。24通道遥测、MANUAL、GOTO、OPSOFFSET 全部按此顺序，
 *            底盘内部仍沿用 pos_x=前后、pos_y=左右，只在边界处交换一次。
 *          - 串口终端发送：KPX=3.0（左右轴P）/ KPY=3.0（前后轴P）/ KPZ=10.0
 *                          XVMAX=1600 / ZVMAX=750
 *                          STOP / ZERO
 *                          PING（上位机运行心跳）
 *                          DMID=1 / DMEN / DMOFF / DMZERO
 *                          DMMODE=1 (MIT) / DMMODE=2 (位置速度)
 *                          DMPOS=3.14 / DMVEL=2 / DMKP=2 / DMKD=1 / DMTOR=0.5
 *          - 调试动作超过 1s 未收到任何命令/PING 时，底盘停车且 DM 失能
 ******************************************************************************
 */
#ifndef __DEBUG_USART_H__
#define __DEBUG_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* VOFA+ JustFloat 通道数：12 路底盘 + 12 路 DM 电机 */
#define DEBUG_VOFA_CHANNELS   24U

/**
 * @brief  初始化 USART1 调试接收（空闲中断 + DMA）
 * @note   注册失败或DMA启动失败由默认任务重试；接收错误也走同一恢复流程。
 */
void DebugUsart_Init(void);


/**
 * @brief  发送一次 VOFA+ JustFloat 数据帧，并处理待执行命令
 * @note   同时恢复异常中断的USART1接收，只复位RX，不主动中止TX。
 * @note   建议 10~50ms 周期调用
 */
void DebugUsart_Send(void);

/**
 * @brief  USART1 空闲接收回调（由 HAL 注册调用）
 */
void DebugUsart_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_USART_H__ */
