/**
 ******************************************************************************
 * @file    ops.h
 * @brief   OPS 全局定位模块接收/解析驱动（硬件层）
 *
 *          协议格式（与 ops9-main 示例工程一致）：
 *            OPS -> MCU：0x5C | float32 x | float32 y | float32 z | CRC8（共 14 字节）
 *            MCU -> OPS：0xC5 | 命令字（共 2 字节）
 *          接收方式：USART2 空闲中断 + DMA 接收
 *
 *          坐标清零：与 ops9-main 的底盘偏移标定方式一致，
 *          不清除 OPS 模块本身，而是在本地记录“零点”并换算相对坐标。
 ******************************************************************************
 */
#ifndef __OPS_H__
#define __OPS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------------------- 协议常量 ---------------------------- */
#define OPS_FRAME_LEN               14U   /* 帧长：帧头 + X + Y + Z + CRC8 */
#define OPS_CMD_LEN                 2U    /* 命令长度：帧头 + 命令字      */

#define OPS_FRAME_HEADER            0x5CU /* OPS -> MCU 定位数据帧头    */
#define OPS_CMD_HEADER              0xC5U /* MCU -> OPS 命令帧头        */
#define OPS_CMD_MODE_INIT           0x22U /* OPS 初始化/查询命令        */
#define OPS_CMD_MODE_START          0x30U /* OPS 启动/持续输出命令      */

/* ---------------------------- 解析状态 ---------------------------- */
#define OPS_STATUS_IDLE             0U
#define OPS_STATUS_OK               1U
#define OPS_STATUS_HEADER_ERR       2U
#define OPS_STATUS_CRC_ERR          3U
#define OPS_STATUS_DATA_ERR         4U

/* ---------------------------- 帧结构体 ---------------------------- */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define OPS_PACKED __packed
#else
#define OPS_PACKED __attribute__((packed))
#endif

typedef OPS_PACKED struct
{
  uint8_t header;   /* 帧头 0x5C           */
  float   x;        /* 世界坐标系 X 坐标[m] */
  float   y;        /* 世界坐标系 Y 坐标[m] */
  float   z;        /* 航向角/偏航角        */
  uint8_t checksum; /* CRC8 校验值          */
} OPS_Frame_t;

typedef struct
{
  OPS_Frame_t       frame;
  volatile uint32_t valid_count; /* 成功解析帧数     */
  volatile uint32_t error_count; /* 帧头/CRC 失败数  */
  volatile uint8_t  status;      /* 最近一次解析状态 */
  volatile uint32_t last_update_tick; /* 最近有效帧时间，单位 ms */

  /* 坐标清零相关参数 */
  float             origin_x;     /* 零点对应的绝对 X 坐标  */
  float             origin_y;     /* 零点对应的绝对 Y 坐标  */
  volatile uint8_t  zero_enabled; /* 1 已清零，0 未清零     */
} OPS_Data_t;

/* ---------------------------- 对外接口 ---------------------------- */
void              OPS_Init(void);                       /* 初始化并启动接收                     */
void              OPS_Start(void);                      /* 重新启动空闲中断 + DMA 接收          */
HAL_StatusTypeDef OPS_SendCommand(uint8_t cmd);         /* 发送 0xC5 + 命令字                   */

const OPS_Data_t *OPS_GetData(void);                    /* 获取完整解析数据指针                 */

uint8_t OPS_GetPosition(float *x, float *y, float *z);        /* 读取清零后的坐标，成功返回 1 */
uint8_t OPS_GetAbsolutePosition(float *x, float *y, float *z); /* 读取 OPS 原始绝对坐标        */
uint8_t OPS_IsNew(void);                                  /* 是否有新数据                         */
void    OPS_ClearNew(void);                               /* 清除新数据标志                       */
uint8_t OPS_IsOnline(uint32_t timeout_ms);                 /* 定位数据是否在超时时间内更新         */

/* 安装偏移，单位 mm，±500，用**内部车体轴**表示：
 *   x_mm : 前后轴，+ 指向车尾（+50 = 装在中心后方 50mm）
 *   y_mm : 左右轴，+ 与车左同向（+60 = 装在中心左侧 60mm）
 * 对外协议 OPSOFFSET=X(左右,+左),Y(前后,+车头) 由 debug_usart.c 适配层换算后调用本函数。
 * 有效帧存在时同时以当前位置重新置零。 */
uint8_t OPS_SetMountOffset(float x_mm, float y_mm);

void OPS_ZeroCoordinates(void);                            /* 以当前 OPS 坐标作为零点               */
void OPS_ClearZero(void);                                   /* 取消清零，恢复补偿后的未清零坐标      */
void OPS_SetOrigin(float x, float y);                       /* 手动设置 X/Y 零点                     */
uint8_t OPS_IsZeroEnabled(void);                            /* 查询当前是否已坐标清零                */

#ifdef __cplusplus
}
#endif

#endif /* __OPS_H__ */
