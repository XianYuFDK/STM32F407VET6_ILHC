/**
 ******************************************************************************
 * @file    ops.h
 * @brief   OPS 全局定位模块接收/解析驱动（硬件层）
 *
 *          协议格式：
 *            V1：0x5C | float32 x | float32 y | float32 z | CRC8（共 14 字节）
 *            V2：0x5D | 版本 | 长度 | flags | seq | session_id |
 *                timestamp_ms | float32 x/y/z | CRC16（共 28 字节）
 *            MCU -> OPS：0xC5 | 命令字（共 2 字节）
 *          接收方式：USART2 空闲中断 + DMA 流式接收
 *
 *          统一坐标约定：+X=车左、+Y=车头、+Z=逆时针。
 *          坐标清零：不清除 OPS 模块本身，而是在本地记录零点。
 *          ZERO 同时建立平移原点和 XY 轴旋转基准，清零后 +Y 表示
 *          ZERO 时的车头方向。
 ******************************************************************************
 */
#ifndef __OPS_H__
#define __OPS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------------------- 协议常量 ---------------------------- */
#define OPS_FRAME_LEN_V1            14U   /* V1：帧头 + X/Y/Z + CRC8       */
#define OPS_FRAME_LEN_V2            28U   /* V2：头 + 元数据 + X/Y/Z + CRC16 */
#define OPS_RX_BUFFER_SIZE          64U   /* DMA/流式解析缓冲区              */
#define OPS_CMD_LEN                 2U    /* 命令长度：帧头 + 命令字         */

#define OPS_FRAME_HEADER_V1         0x5CU
#define OPS_FRAME_HEADER_V2         0x5DU
#define OPS_FRAME_VERSION_V2        0x01U

#define OPS_FLAG_POS_VALID          0x01U
#define OPS_FLAG_IMU_ONLINE         0x02U
#define OPS_FLAG_IMU_REBASED        0x04U
#define OPS_FLAG_ENC_VALID          0x08U
#define OPS_FLAG_GYRO_VALID         0x10U

#define OPS_CMD_HEADER              0xC5U /* MCU -> OPS 命令帧头         */
#define OPS_CMD_MODE_INIT           0x22U /* OPS 初始化/复位命令         */
#define OPS_CMD_DIR_0               0x30U /* 新协议方向 0                 */
#define OPS_CMD_DIR_1               0x31U /* 新协议方向 1                 */
#define OPS_CMD_DIR_2               0x32U /* 新协议方向 2                 */
#define OPS_CMD_DIR_3               0x33U /* 新协议方向 3                 */

/* ---------------------------- 解析状态 ---------------------------- */
#define OPS_STATUS_IDLE             0U
#define OPS_STATUS_OK               1U
#define OPS_STATUS_HEADER_ERR       2U
#define OPS_STATUS_CRC_ERR          3U
#define OPS_STATUS_DATA_ERR         4U

/* ---------------------------- 帧结构体 ---------------------------- */
/* 解析后使用主机自然对齐结构，字段从字节流逐个取出，避免 __packed 浮点访问。 */
typedef struct
{
  uint8_t  header;      /* V1:0x5C, V2:0x5D            */
  uint8_t  version;     /* V1=0, V2=1                  */
  uint8_t  length;      /* 14 或 28                    */
  uint8_t  flags;       /* V2 有效标志；V1 合成兼容值    */
  uint16_t seq;         /* V2 发包序号                 */
  uint32_t session_id;  /* V2 会话号                   */
  uint32_t timestamp_ms;/* V2 发送端毫秒时基            */
  float    x;           /* OPS 原始帧 x；统一映射见 ops.c [m] */
  float    y;           /* OPS 原始帧 y；统一映射见 ops.c [m] */
  float    z;           /* 航向角/偏航角 [rad]         */
  uint16_t checksum;    /* V1 CRC8 或 V2 CRC16          */
} OPS_Frame_t;

typedef struct
{
  OPS_Frame_t       frame;
  volatile uint32_t frame_count;   /* CRC 正确的帧数                 */
  volatile uint32_t valid_count;   /* 位姿有效且已发布的帧数         */
  volatile uint32_t error_count;   /* 帧头/长度/CRC 失败数           */
  volatile uint32_t crc_errors;    /* CRC 失败数                     */
  volatile uint32_t format_errors; /* 帧头/版本/长度失败数           */
  volatile uint8_t  status;        /* 最近一次解析状态               */
  volatile uint8_t  pose_valid;    /* 当前位姿是否可用               */
  volatile uint8_t  session_changed;/* V2 session_id 运行期发生变化  */
  volatile uint16_t seq;           /* 最近 V2 序号                   */
  volatile uint32_t session_id;    /* 最近 V2 会话号                 */
  volatile uint32_t timestamp_ms;  /* 最近 V2 发送端时间戳           */
  volatile uint32_t last_update_tick; /* 最近有效位姿的本地 tick，ms */

  /* 坐标清零相关参数 */
  float             origin_x;     /* 零点对应的 OPS 原始 x  */
  float             origin_y;     /* 零点对应的 OPS 原始 y  */
  volatile uint8_t  zero_enabled; /* 1 已清零，0 未清零     */
} OPS_Data_t;

/* 诊断不改变原取消码17和定位保护；原因是可组合的位掩码。 */
#define OPS_FAULT_SESSION_ID       1U
#define OPS_FAULT_TIMESTAMP_BACK   2U
#define OPS_FAULT_FRAME_GAP        4U
#define OPS_FAULT_PROTOCOL_V1      8U
#define OPS_FAULT_IMU_REBASED      16U
#define OPS_FAULT_RX_RESTART       32U
#define OPS_FAULT_RX_OVERFLOW      64U
#define OPS_FAULT_RX_QUEUE         128U
#define OPS_FAULT_RX_STALE         256U
#define OPS_FAULT_UART             512U
#define OPS_FAULT_RECOVERY_TIMEOUT 1024U
#define OPS_FAULT_RECOVERED        2048U /* 接收恢复通知，不代表取消 */

typedef struct {
  uint32_t id, reason, tick, previous_session, session;
  uint32_t previous_timestamp, timestamp, seq, flags;
  uint32_t gap_ms, valid_age_ms, rx_age_ms, uart_error, crc_errors, rx_overflows;
} OPS_Fault_t;

typedef struct {
  uint32_t tick, session, seq, timestamp, flags, pose_valid, valid_age_ms;
  uint32_t crc_errors, format_errors, uart_errors, last_uart_error;
  uint32_t rx_overflows, stale_packets, restart_failures, diagnostic_drops;
} OPS_Diagnostics_t;

uint8_t OPS_PeekFault(OPS_Fault_t *fault);
void OPS_FaultSent(uint32_t id);
void OPS_GetDiagnostics(OPS_Diagnostics_t *snapshot);

/* ---------------------------- 对外接口 ---------------------------- */
void              OPS_Init(void);                       /* 初始化并启动接收                     */
void              OPS_Start(void);                      /* 重新启动空闲中断 + DMA 接收          */
void              OPS_ServiceRx(void);                  /* 任务上下文恢复 USART2 接收           */
void              OPS_ProcessPending(void);             /* 通信任务解析一包，持有应用状态锁     */
HAL_StatusTypeDef OPS_SendCommand(uint8_t cmd);         /* 发送 0xC5 + 命令字                   */

const OPS_Data_t *OPS_GetData(void);                    /* 获取完整解析数据指针                 */

uint8_t OPS_GetPosition(float *x, float *y, float *z);        /* X=左、Y=前，成功返回 1 */
uint8_t OPS_GetAbsolutePosition(float *x, float *y, float *z); /* X=左、Y=前的未补偿绝对坐标 */
uint8_t OPS_IsNew(void);                                  /* 是否有新数据                         */
void    OPS_ClearNew(void);                               /* 清除新数据标志                       */
uint8_t OPS_IsOnline(uint32_t timeout_ms);                 /* 定位数据是否在超时时间内更新         */
uint8_t OPS_ConsumeSessionChanged(void);                   /* 读取并清除会话变化标志               */
uint8_t OPS_RecoveryPending(void);                         /* 短时V2接收恢复；必须暂停运动 */

/* 安装偏移，单位 mm，±500，统一坐标：
 *   x_mm : X=左右，+ 为车左（+60 = 装在中心左侧 60mm）
 *   y_mm : Y=前后，+ 为车头（-50 = 装在中心后方 50mm）
 * 只更新安装偏移，不自动执行 ZERO；OPSOFFSET 与 ZERO 是独立动作。 */
uint8_t OPS_SetMountOffset(float x_mm, float y_mm);
void OPS_GetMountOffset(float *x_mm, float *y_mm); /* 已应用的安装参数，单位mm */

void OPS_ZeroCoordinates(void);                            /* 当前 X/Y/Z 同时置零                   */
void OPS_ClearZero(void);                                   /* 取消清零，恢复绝对 X/Y/Z              */
void OPS_SetOrigin(float x, float y);                       /* 手动设置 X/Y 零点，同时归零 Z         */
uint8_t OPS_IsZeroEnabled(void);                            /* 查询当前是否已坐标清零                */

#ifdef __cplusplus
}
#endif

#endif /* __OPS_H__ */
