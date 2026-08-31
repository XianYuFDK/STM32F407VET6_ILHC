/**
 ******************************************************************************
 * @file    hcan.c
 * @brief   CAN 协议收发封装
 *
 *          CAN1：
 *          - 滤波：全接收
 *          - 接收：FIFO0 中断，Rxfifo0MsgPendingCallback
 *          - 发送：标准帧/扩展帧，长数据自动分包
 ******************************************************************************
 */
#include "hcan.h"
#include <string.h>

/* 最近接收帧 */
volatile HCanRxFrame_t hcanRxFrame;
volatile uint8_t       hcanRxFlag = 0U;

/* --------------------------- 私有函数 ------------------------------ */

/**
 * @brief  CAN 滤波器初始化：全接收，FIFO0
 */
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
  if (sFilterConfig == NULL)
  {
    return;
  }

  sFilterConfig->FilterIdHigh = 0;
  sFilterConfig->FilterIdLow = 0;
  sFilterConfig->FilterMaskIdHigh = 0;
  sFilterConfig->FilterMaskIdLow = 0;
  sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig->FilterBank = 0;
  sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig->FilterActivation = ENABLE;
  /* F407 的 CAN1/CAN2 共用滤波器，Bank 0~13 分给 CAN1 */
  sFilterConfig->SlaveStartFilterBank = 14;
}

/* --------------------------- 对外接口 ------------------------------ */

/**
 * @brief  初始化 CAN 滤波、接收中断并启动 CAN
 */
HAL_StatusTypeDef CAN_Start(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef sFilterConfig;
  HAL_StatusTypeDef status;

  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  /* 全接收滤波 */
  CAN_Filter_ParamsInit(&sFilterConfig);
  status = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 使能 FIFO0 接收中断 */
  status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 启动 CAN */
  return HAL_CAN_Start(hcan);
}

/**
 * @brief  发送标准 ID CAN 帧
 */
HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan, uint16_t ID,
                              const uint8_t *pData, uint16_t Len)
{
  static CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mail_box;

  if ((hcan == NULL) || (pData == NULL) || (Len > 8U) || (ID > 0x07FFU))
  {
    return HAL_ERROR;
  }

  tx_header.StdId = ID;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = Len;

  return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)pData, &tx_mail_box);
}

/**
 * @brief  发送扩展 ID CAN 帧
 */
HAL_StatusTypeDef CAN_SendEXData(CAN_HandleTypeDef *hcan, uint32_t ID,
                                const uint8_t *pData, uint16_t Len)
{
  static CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mail_box;

  if ((hcan == NULL) || (pData == NULL) || (Len > 8U) || (ID > 0x1FFFFFFFUL))
  {
    return HAL_ERROR;
  }

  tx_header.StdId = 0;
  tx_header.ExtId = ID;
  tx_header.IDE = CAN_ID_EXT;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = Len;

  return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)pData, &tx_mail_box);
}

/**
 * @brief  长数据自动分包发送，每包 8 字节，扩展 ID 依次递增
 */
HAL_StatusTypeDef Can_SendCmd(uint32_t ID, const uint8_t *cmd, uint8_t len)
{
  uint8_t i;
  uint8_t n;
  uint8_t pack_num;
  uint8_t pack_last;
  uint8_t data[8];

  if ((cmd == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  if (len < 8U)
  {
    memset(data, 0, sizeof(data));
    for (i = 0U; i < len; ++i)
    {
      data[i] = cmd[i];
    }
    return CAN_SendEXData(HCAN_CAN_NUM, ID, data, 8U);
  }

  pack_num  = len / 8U;
  pack_last = len % 8U;

  for (n = 0U; n < pack_num; ++n)
  {
    memset(data, 0, sizeof(data));
    for (i = 0U; i < 8U; ++i)
    {
      data[i] = cmd[(uint16_t)n * 8U + i];
    }
    if (CAN_SendEXData(HCAN_CAN_NUM, ID + n, data, 8U) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (pack_last > 0U)
  {
    memset(data, 0, sizeof(data));
    for (i = 0U; i < pack_last; ++i)
    {
      data[i] = cmd[(uint16_t)pack_num * 8U + i];
    }
    if (CAN_SendEXData(HCAN_CAN_NUM, ID + pack_num, data, pack_last) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

/**
 * @brief  CAN 接收处理钩子，默认空实现
 * @note   用户可在其他文件中覆盖此后定义
 */
__weak void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
}

/**
 * @brief  读取最近接收帧
 */
uint8_t HCan_GetRxFrame(HCanRxFrame_t *frame)
{
  uint8_t is_new;
  uint32_t primask;

  if (frame == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  memcpy((uint8_t *)frame, (const uint8_t *)&hcanRxFrame, sizeof(HCanRxFrame_t));
  is_new = hcanRxFlag;
  hcanRxFlag = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return is_new;
}

/**
 * @brief  清除接收新帧标志
 */
void HCan_ClearRxFlag(void)
{
  hcanRxFlag = 0U;
}

/* ------------------------- HAL 接收回调 ---------------------------- */

/**
 * @brief  CAN1 FIFO0 接收到消息
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];

  if ((hcan == NULL) || (hcan->Instance != CAN1))
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
  {
    return;
  }

  hcanRxFrame.StdId = rx_header.StdId;
  hcanRxFrame.ExtId = rx_header.ExtId;
  hcanRxFrame.IDE   = rx_header.IDE;
  hcanRxFrame.RTR   = rx_header.RTR;
  hcanRxFrame.DLC   = rx_header.DLC;
  memcpy((uint8_t *)hcanRxFrame.Data, data, 8U);
  hcanRxFlag = 1U;

  CAN_Rx_Callback(hcan);
}
