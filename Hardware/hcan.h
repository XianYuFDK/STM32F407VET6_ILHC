/**
 ******************************************************************************
 * @file    hcan.h
 * @brief   CAN 协议收发封装（移植自 tower/hcan.c）
 *
 *          - CAN1 标准 ID / 扩展 ID 帧发送
 *          - 长数据自动分包，每帧最多 8 字节
 *          - FIFO0 接收中断，保存最近一帧
 *
 *          使用：
 *            CAN_Start(&hcan1);                 // 使能滤波 + 接收中断 + 启动
 *            CAN_SendData(&hcan1, ID, data, 8);
 *            CAN_SendEXData(&hcan1, ID, data, 8);
 *            Can_SendCmd(ID, longData, len);    // 自动分包
 ******************************************************************************
 */
#ifndef __HCAN_H__
#define __HCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

/* 默认 CAN1 句柄 */
#define HCAN_CAN_NUM   &hcan1

/* 接收帧结构 */
typedef struct
{
  uint32_t StdId;   /* 标准 ID */
  uint32_t ExtId;   /* 扩展 ID */
  uint8_t  IDE;     /* 标准/扩展标志 */
  uint8_t  RTR;     /* 远程帧标志 */
  uint8_t  DLC;     /* 数据长度 */
  uint8_t  Data[8]; /* 数据 */
} HCanRxFrame_t;

extern volatile HCanRxFrame_t hcanRxFrame;  /* 最近接收帧 */
extern volatile uint8_t        hcanRxFlag;  /* 1 表示有新帧 */

/**
 * @brief  初始化 CAN 滤波、接收中断并启动 CAN
 */
HAL_StatusTypeDef CAN_Start(CAN_HandleTypeDef *hcan);

/**
 * @brief  发送标准 ID CAN 帧
 * @param  hcan  CAN 句柄
 * @param  ID    标准 ID
 * @param  pData 数据
 * @param  Len   数据长度 0~8
 * @return HAL_OK 已提交；HAL_BUSY 邮箱忙；HAL_ERROR 参数或 CAN 未启动。
 * @note 短帧只要求输入 Len 字节；内部补齐 HAL 所需的八字节存储。
 *       短临界区保护邮箱提交，无阻塞等待，恢复调用前中断状态。
 */
HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan, uint16_t ID,
                              const uint8_t *pData, uint16_t Len);

/**
 * @brief  发送扩展 ID CAN 帧
 * @param ID 29位扩展标识符，最大0x1FFFFFFF。
 * @note 长度与返回状态同 CAN_SendData；不会改变实际 DLC。
 */
HAL_StatusTypeDef CAN_SendEXData(CAN_HandleTypeDef *hcan, uint32_t ID,
                                const uint8_t *pData, uint16_t Len);

/**
 * @brief  长数据自动分包发送
 * @param  ID  起始扩展 ID，每包 ID 依次加 1
 * @param  cmd 数据
 * @param  len 数据长度
 * @return 继承发送函数返回值，遇首个失败停止；先前包可能已提交。
 * @note 每包最多八字节，末包保留真实 DLC；本接口没有整条消息队列或
 *       自动重试。超过可用邮箱数量的长命令可能部分提交，由调用方调度。
 */
HAL_StatusTypeDef Can_SendCmd(uint32_t ID, const uint8_t *cmd, uint8_t len);

/**
 * @brief  CAN 接收处理钩子，默认空实现，可被外部覆盖
 */
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);

/**
 * @brief  读取最近接收帧
 * @return 1 有新帧，0 无新帧
 */
uint8_t HCan_GetRxFrame(HCanRxFrame_t *frame);

/**
 * @brief  清除接收新帧标志
 */
void HCan_ClearRxFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __HCAN_H__ */
