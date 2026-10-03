/**
 * @file vision.h
 * @brief USART3 工控机视觉协议 V1.1（坐标系修正版）收发驱动。
 *
 * 请求帧：66 TASK TARGET SEQ CRC8 77，共6字节；CRC8 覆盖字节0..3。
 * 响应帧：66 TASK STATUS SEQ PAYLOAD[10] CRC8 77，共16字节；CRC8 覆盖字节0..13。
 * CRC-8：poly=0x07、init=0x00、RefIn/RefOut=False、XorOut=0x00，Check("123456789")=0xF4。
 * 协议依据：E:/STM32/ILHC/jetson/STM32F407_视觉通信协议_V1.1_坐标系修正版_AI_Agent版.md。
 *
 * 注意：早期 V1.1 草案曾把车体坐标误写成"+X=前、+Y=左"，已被正式作废。
 * 本工程与修正版一致：+X=车左、+Y=车头、+Yaw=逆时针。TRACK 载荷的错误量
 * error_x>0 表示"需要向左修正"、error_y>0 表示"需要向前修正"，与
 * MecanumControl_MoveVelocity(vx=左, vy=前, vz=逆时针) 同序同号，不需要任何
 * 轴交换或取反；方向换算由 Jetson 按相机安装关系完成，固件不得再自行翻转。
 */
#ifndef ILHC_VISION_H
#define ILHC_VISION_H

#include "main.h"
#include <stdint.h>

#define VISION_FRAME_HEAD          0x66U
#define VISION_FRAME_TAIL          0x77U
#define VISION_REQUEST_LEN         6U
#define VISION_RESPONSE_LEN        16U
#define VISION_PAYLOAD_LEN         10U
/* 连续跟踪结果时效：规范§17 建议精对准超过150ms即判过期。
 * 这是相对V1.0（300ms）的行为变化——丢帧时会更早停下底盘。 */
#define VISION_TRACK_TIMEOUT_MS    150U
/* 批量任务的INDEX最大值，用于位图去重（规范§14建议至少支持1~16）。 */
#define VISION_BATCH_INDEX_MAX     16U

/* AC5的C99模式对_Static_assert支持有限，使用经典编译期断言惯用法。 */
typedef char Vision_RequestLenCheck[(VISION_REQUEST_LEN == 6U) ? 1 : -1];
typedef char Vision_ResponseLenCheck[(VISION_RESPONSE_LEN == 16U) ? 1 : -1];

typedef enum {
  VISION_TASK_STOP           = 0x00,
  VISION_TASK_MATERIALS_SCAN = 0x01,
  VISION_TASK_MATERIAL_TRACK = 0x02,
  VISION_TASK_RINGS_SCAN     = 0x03,
  VISION_TASK_RING_TRACK     = 0x04,
  VISION_TASK_OBSTACLE_TRACK = 0x06,
  VISION_TASK_MATERIAL_ORDER = 0x08,
  VISION_TASK_QR_SCAN        = 0x09
} VisionTask;

typedef enum {
  VISION_STATUS_OK           = 0x00,
  VISION_STATUS_NOT_FOUND    = 0x01,
  VISION_STATUS_COMPLETE     = 0x02,
  VISION_STATUS_BUSY         = 0x03,
  VISION_STATUS_BAD_ARGUMENT = 0x04,
  VISION_STATUS_ERROR        = 0x05,
  VISION_STATUS_INVALID_DATA = 0x06
} VisionStatus;

/* 非OK状态时 payload[0] 为错误详情（规范§15）。 */
typedef enum {
  VISION_ERR_NONE                = 0x00,
  VISION_ERR_UNSUPPORTED_TASK    = 0x01,
  VISION_ERR_INVALID_TARGET      = 0x02,
  VISION_ERR_CAMERA_UNAVAILABLE  = 0x03,
  VISION_ERR_MODEL_NOT_READY     = 0x04,
  VISION_ERR_CALIBRATION_MISSING = 0x05,
  VISION_ERR_QR_FORMAT_INVALID   = 0x06,
  VISION_ERR_INTERNAL_EXCEPTION  = 0x07,
  VISION_ERR_INTERNAL_TIMEOUT    = 0x08,
  VISION_ERR_PROTOCOL_STATE      = 0x09,
  VISION_ERR_SEQ_CONFLICT        = 0x0A
} VisionErrorDetail;

/* TRACK载荷的误差单位模式。两种模式必须使用不同的控制系数（规范§11.2）。 */
typedef enum {
  VISION_COORD_PIXEL_ERROR = 0x01,
  VISION_COORD_ROBOT_MM    = 0x02
} VisionCoordMode;

/* VALID_FLAGS位定义；控制某一轴前必须先检查对应位（规范§12）。 */
#define VISION_VALID_X      (1U << 0)
#define VISION_VALID_Y      (1U << 1)
#define VISION_VALID_YAW    (1U << 2)
#define VISION_VALID_STABLE (1U << 3)

/* 已通过帧头、帧尾、任务码、状态码和CRC-8检查的响应。
 * 非OK状态时 payload[0] 为 VisionErrorDetail，其余载荷字节按规范清零。 */
typedef struct {
  uint32_t received_tick;          /* 最后一个字节到达时的HAL毫秒时基 */
  uint8_t task;
  uint8_t status;
  uint8_t seq;                     /* 请求序号，回显自对应请求 */
  uint8_t payload[VISION_PAYLOAD_LEN];
} VisionResponse;

/* SCAN类载荷（规范§10）：MATERIALS_SCAN / RINGS_SCAN / MATERIAL_ORDER /
 * OBSTACLE_TRACK。对应规范建议名 VisionScanResult。 */
typedef struct {
  uint8_t target_id;
  uint8_t index;                   /* 本批序号，从1开始 */
  uint8_t total;                   /* 本批总数 */
  uint16_t x;                      /* 图像横坐标，像素 */
  uint16_t y;                      /* 图像纵坐标，像素 */
  int16_t extra;                   /* 尺寸/直径/宽度等，含义随TASK固定 */
  uint8_t confidence;              /* 置信度，0..100 */
} VisionTarget;

/* TRACK类载荷（规范§11）：MATERIAL_TRACK / RING_TRACK。
 * error_x/error_y/yaw_error_cdeg 的量纲由 coord_mode 决定。 */
typedef struct {
  uint8_t target_id;
  uint8_t coord_mode;              /* VisionCoordMode */
  uint8_t valid_flags;             /* VISION_VALID_* 位组合 */
  int16_t error_x;                 /* 正数=需要向左（+X）、负数=向右 */
  int16_t error_y;                 /* 正数=需要向前（+Y）、负数=向后 */
  int16_t yaw_error_cdeg;          /* 正数=需要逆时针，单位0.01度 */
  uint8_t confidence;              /* 置信度，0..100 */
} VisionTrack;

typedef struct {
  uint16_t group[4];
  uint8_t confidence;
} VisionQr;

/* 批量任务组装（规范§14）：按INDEX位图去重，COMPLETE时校验完整性。 */
typedef struct {
  uint8_t expected_total;          /* 各OK帧的TOTAL，必须一致 */
  uint8_t received_count;          /* 已收到的唯一结果数 */
  uint16_t received_bitmap;        /* INDEX 1..VISION_BATCH_INDEX_MAX 位图 */
  uint8_t total_conflict;          /* 1表示出现过不一致的TOTAL */
  uint8_t complete;                /* 1表示已收到COMPLETE帧 */
  uint8_t payload_count;           /* COMPLETE帧payload[0]：Jetson实际发送数 */
  uint8_t consistent;              /* 1表示COMPLETE的计数与本地位图一致 */
} VisionBatch;

typedef struct {
  uint32_t valid_frames;
  uint32_t checksum_errors;
  uint32_t format_errors;
  uint32_t queue_overflows;
  uint32_t uart_errors;
  uint32_t seq_mismatch;           /* 收到非当前活动SEQ的有效帧 */
} VisionStats;

/* 在MX_USART3_UART_Init()之后调用，启动逐字节中断接收。 */
HAL_StatusTypeDef Vision_Init(void);
/* 发起一个新的逻辑任务：分配新SEQ、组6字节请求帧并尝试提交。
 * 返回HAL_BUSY/HAL_ERROR时请求保持待发，用本轮或下轮的同一调用重发即可——
 * 只要TASK和TARGET没变就复用同一个SEQ，避免被Jetson当成新任务（规范§21）。
 * 返回值只表示HAL提交结果，不代表视觉端已执行。 */
HAL_StatusTypeDef Vision_SendRequest(uint8_t task, uint8_t target);
/* 停止当前视觉任务（等价的便捷形式，同样是新SEQ）。 */
HAL_StatusTypeDef Vision_Stop(void);
/* 已提交但未成功的请求是否仍在等待重发。 */
uint8_t Vision_IsRequestPending(void);
/* 当前活动任务的TASK与SEQ；应用层只接受两者都匹配的响应（规范§8）。 */
uint8_t Vision_GetActiveTask(void);
uint8_t Vision_GetActiveSeq(void);
/* 在任务中周期调用；发生HAL接收错误后重新挂载USART3接收。 */
void Vision_ServiceRx(void);
/* 取出一条完整且校验正确的响应；成功返回1，队列为空返回0。 */
uint8_t Vision_PopResponse(VisionResponse *response);
/* 清除已入队响应和未收完的残帧。SEQ可过滤迟到回包，这里主要用于切换任务前腾空队列。 */
void Vision_FlushResponses(void);
void Vision_GetStats(VisionStats *stats);

/* SCAN类载荷解码。状态非OK、任务不属于SCAN类或载荷非法时返回0。
 * 对应规范建议名 VisionScanResult。 */
uint8_t Vision_DecodeTarget(const VisionResponse *response, VisionTarget *target);
/* TRACK类载荷解码。任务不属于TRACK类、coord_mode不认识或载荷非法时返回0。 */
uint8_t Vision_DecodeTrack(const VisionResponse *response, VisionTrack *track);
uint8_t Vision_DecodeQr(const VisionResponse *response, VisionQr *qr);
/* 检查单条连续跟踪响应的载荷和 VISION_TRACK_TIMEOUT_MS 时效。应用层仍须处理
 * 后续NOT_FOUND/ERROR，收到这些状态后立即使之前的结果失效。 */
uint8_t Vision_IsTrackingFresh(const VisionResponse *response, uint32_t now_tick);
/* 正在组装的批量任务快照（§14）。收到新批量任务时自动清零。 */
void Vision_GetBatch(VisionBatch *batch);

#endif /* ILHC_VISION_H */
