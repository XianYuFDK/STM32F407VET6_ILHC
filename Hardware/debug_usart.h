/**
 ******************************************************************************
 * @file    debug_usart.h
 * @brief   USART1 调试模块（VOFA+ 调参）
 *
 *          - USART1：PA9=TX，PA10=RX，115200/8N1
 *          - TX：DMA发送24通道；VOFA选择JustFloat，TELEM=1选择CRC1可靠遥测
 *          - RX：DMA 空闲中断接收 ASCII 命令，可在线调节底盘 / DM 电机参数
 *
 *          VOFA+ 使用方式：
 *          - 数据协议选择 JustFloat
 *          - 统一坐标约定：+X=车左、+Y=车头、+Z=逆时针；
 *            24通道遥测、MANUAL、GOTO、OPSOFFSET 与底盘内部全部按此顺序。
 *            遥测 ch0/ch1/ch3/ch4 与 GOTO 的 X/Y 使用 cm（1位小数）；底盘内部用 mm。
 *          - 串口终端发送：KPX=3.0（左右轴P）/ KPY=3.0（前后轴P）/ KPZ=10.0
 *                          XVMAX=1600 / ZVMAX=750
 *                          STOP / ZERO
 *                          VTRACK=1..6（按颜色居中跟踪）/ VTRACK=0（停止）
 *                          PING（上位机运行心跳）
 *                          VOFA（第三方JustFloat）/ TELEM=1（配套PC的序号/时间/CRC32）
 *                          DMID=3 / DMEN / DMOFF / DMZERO
 *                          DMMODE=1 (MIT) / DMMODE=2 (位置速度)
 *                          DMPOS=3.14 / DMVEL=2 / DMKP=2 / DMKD=1 / DMTOR=0.5
 *                          DMREAD=序号 / DMACCDEC=序号,加速度,负减速度
 *                          序号1..65535；ACC/DEC原始单位Krad/s²，失能反馈300ms内才接受
 *                          成对读写电机RAM并通过DMREG文字行回传真实回读，24通道不变
 *          - 调试动作超过 1s 未收到任何命令/PING 时，底盘停车、视觉跟踪退出且 DM 失能
 ******************************************************************************
 */
#ifndef __DEBUG_USART_H__
#define __DEBUG_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "debug_param_store.h"

/* VOFA+ JustFloat 通道数：12 路底盘 + 12 路 DM 电机 */
#define DEBUG_VOFA_CHANNELS   24U

/**
 * @brief  初始化 USART1 调试接收（空闲中断 + DMA）
 * @note   注册失败或DMA启动失败由默认任务重试；接收错误也走同一恢复流程。
 */
void DebugUsart_Init(void);


/**
 * @brief  通信任务独占USART1 TX，CRC1应答与24通道同包；不执行运动或Flash。
 */
void DebugUsart_Send(void);

/* 以下服务由RTOS_APP在应用状态锁内调用，禁止其他任务直接输出轮速。 */
void DebugUsart_ControlService(void);
void DebugUsart_ControlEmergency(void);
void DebugUsart_MechanismService(void);
void DebugUsart_MechanismEmergency(void);
void DebugUsart_ProcessPending(void);
void DebugUsart_ParamSnapshot(DebugParamValues *values);
void DebugUsart_ParamResult(ParamStoreResult result);
/* 仅通信任务调用；TX恢复在状态锁外执行。 */
void DebugUsart_CommunicationRecover(void);

/**
 * @brief  USART1 空闲接收回调（由 HAL 注册调用）
 */
void DebugUsart_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_USART_H__ */
