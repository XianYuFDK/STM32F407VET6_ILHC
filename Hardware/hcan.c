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
  /* CAN2 是 CAN1 的从机：Bank0~13 属于 CAN1，CAN2 只能用 Bank14~27，
   * 这里用 14（与 SlaveStartFilterBank 一致）。写的是 CAN1 地址空间的滤波寄存器。 */
  sFilterConfig->FilterBank = 14;
  sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig->FilterActivation = ENABLE;
  /* F407 的 CAN1/CAN2 共用滤波器，Bank 0~13 分给 CAN1 */
  sFilterConfig->SlaveStartFilterBank = 14;
}

/* --------------------------- 对外接口 ------------------------------ */

/* 发送头与数据准备在调用栈上完成。短临界区保护“查空邮箱并提交”，
   防止另一任务或中断在 HAL 选定邮箱后抢占并写入同一邮箱。
   不在此等待发送完成；HAL_OK 只代表提交，HAL_BUSY 留给调用方重试。 */
static HAL_StatusTypeDef HCan_Submit(CAN_HandleTypeDef *hcan,
                                    CAN_TxHeaderTypeDef *header, const uint8_t *data)
{
  HAL_StatusTypeDef status;
  uint32_t mailbox;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (hcan->State != HAL_CAN_STATE_LISTENING)
    status = HAL_ERROR;
  else if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    status = HAL_BUSY;
  else
    status = HAL_CAN_AddTxMessage(hcan, header, (uint8_t *)data, &mailbox);
  if (primask == 0U) __enable_irq();
  return status;
}

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
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t payload[8] = {0};

  if ((hcan == NULL) || (pData == NULL) || (Len > 8U) || (ID > 0x07FFU))
  {
    return HAL_ERROR;
  }

  tx_header.StdId = ID;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = Len;

  /* HAL 底层读取固定八字节；短帧先补齐本地存储，DLC 仍为实际长度。 */
  memcpy(payload, pData, Len);
  return HCan_Submit(hcan, &tx_header, payload);
}

/**
 * @brief  发送扩展 ID CAN 帧
 */
HAL_StatusTypeDef CAN_SendEXData(CAN_HandleTypeDef *hcan, uint32_t ID,
                                const uint8_t *pData, uint16_t Len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t payload[8] = {0};

  if ((hcan == NULL) || (pData == NULL) || (Len > 8U) || (ID > 0x1FFFFFFFUL))
  {
    return HAL_ERROR;
  }

  tx_header.StdId = 0;
  tx_header.ExtId = ID;
  tx_header.IDE = CAN_ID_EXT;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = Len;

  memcpy(payload, pData, Len);
  return HCan_Submit(hcan, &tx_header, payload);
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
  HAL_StatusTypeDef status;

  if ((cmd == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  /* 提交第一包前检查整个 ID 范围，避免后续包越界才报告错误。 */
  if (ID > 0x1FFFFFFFUL - ((uint32_t)(len - 1U) / 8U)) return HAL_ERROR;

  if (len < 8U)
  {
    memset(data, 0, sizeof(data));
    for (i = 0U; i < len; ++i)
    {
      data[i] = cmd[i];
    }
    return CAN_SendEXData(HCAN_CAN_NUM, ID, data, len);
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
    status = CAN_SendEXData(HCAN_CAN_NUM, ID + n, data, 8U);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  if (pack_last > 0U)
  {
    memset(data, 0, sizeof(data));
    for (i = 0U; i < pack_last; ++i)
    {
      data[i] = cmd[(uint16_t)pack_num * 8U + i];
    }
    status = CAN_SendEXData(HCAN_CAN_NUM, ID + pack_num, data, pack_last);
    if (status != HAL_OK)
    {
      return status;
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
