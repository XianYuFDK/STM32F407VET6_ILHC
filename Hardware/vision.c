/**
 * @file vision.c
 * @brief USART3 工控机视觉协议 V1.1（坐标系修正版）收发驱动。
 *
 * 只负责帧的组包/收发/校验/解码与任务身份（SEQ）管理，不接触任何电机接口；
 * 视觉闭环在 vision_track.c，控制权仲裁在 debug_usart.c。
 */
#include "vision.h"
#include "usart.h"
#include <string.h>

#define VISION_QUEUE_SIZE          16U  /* 实际可存15帧，队满时丢弃新帧 */
#define VISION_INTERBYTE_GAP_MS    20U
#define VISION_VALID_FLAG_MASK     0x0FU /* VALID_FLAGS只允许bit0..3，其余必须为0 */

static uint8_t s_rx_byte;
static uint8_t s_tx_request[VISION_REQUEST_LEN];
static uint8_t s_window[VISION_RESPONSE_LEN];
static uint8_t s_window_count;
static uint32_t s_last_byte_tick;
static VisionResponse s_queue[VISION_QUEUE_SIZE];
static volatile uint8_t s_read, s_write, s_rx_fault;
static VisionStats s_stats;

/* 任务身份：SEQ由本端生成，代表"一个逻辑任务"。连续跟踪期间保持不变，
 * 序列响应必须与 active_task/active_seq 同时匹配才被采用（规范§8）。
 * 由任务上下文写入，接收中断只读，单字节访问在Cortex-M4上是原子的。 */
static volatile uint8_t s_seq;
static volatile uint8_t s_active_task;
static volatile uint8_t s_active_target;
static volatile uint8_t s_active_seq;
static volatile uint8_t s_tx_pending;
/* 批量任务组装状态，接收中断更新，任务上下文快照读取。 */
static VisionBatch s_batch;

/* ------------------------------ CRC-8 ------------------------------- */

/* CRC-8：poly=0x07、init=0x00、RefIn/RefOut=False、XorOut=0x00。
 * 逐位移位实现不占用256字节查表；与 OPS 协议那张 poly=0x31/init=0xFF 的
 * 反射表不是同一算法，不能互相复用。Check("123456789")=0xF4。 */
static uint8_t Vision_Crc8(const uint8_t *data, uint16_t len)
{
  uint8_t crc = 0x00U;
  uint8_t i;

  while (len-- != 0U) {
    crc ^= *data++;
    for (i = 0U; i < 8U; ++i) {
      if ((crc & 0x80U) != 0U) crc = (uint8_t)((crc << 1U) ^ 0x07U);
      else crc = (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

/* ---------------------------- 帧校验 -------------------------------- */

static uint8_t Vision_ValidTask(uint8_t task)
{
  switch (task) {
    case VISION_TASK_STOP:                /* 停止任务 */
    case VISION_TASK_MATERIALS_SCAN:      /* 物料扫描 */
    case VISION_TASK_MATERIAL_TRACK:      /* 物料跟踪 */
    case VISION_TASK_RINGS_SCAN:          /* 环扫描 */
    case VISION_TASK_RING_TRACK:          /* 环跟踪 */
    case VISION_TASK_OBSTACLE_TRACK:      /* 障碍物跟踪 */
    case VISION_TASK_MATERIAL_ORDER:      /* 物料排序 */
    case VISION_TASK_QR_SCAN:             /* QR 扫描 */
      return 1U;
    default:
      return 0U;
  }
}

/* TARGET 必须符合 TASK 的约束（规范§5与请求帧速查表）。 */
static uint8_t Vision_ValidTarget(uint8_t task, uint8_t target)
{
  if (task == VISION_TASK_MATERIAL_TRACK) return (target >= 1U && target <= 6U) ? 1U : 0U;
  if (task == VISION_TASK_RING_TRACK)     return (target >= 1U && target <= 3U) ? 1U : 0U;
  return (target == 0U) ? 1U : 0U;
}

static uint8_t Vision_ValidFrame(const uint8_t *frame)
{
  if (frame[0] != VISION_FRAME_HEAD || frame[VISION_RESPONSE_LEN - 1U] != VISION_FRAME_TAIL ||
      !Vision_ValidTask(frame[1]) || frame[2] > VISION_STATUS_INVALID_DATA) {
    ++s_stats.format_errors;
    return 0U;
  }
  /* CRC覆盖字节0..13，不含CRC自身与帧尾。 */
  if (Vision_Crc8(frame, VISION_RESPONSE_LEN - 2U) != frame[VISION_RESPONSE_LEN - 2U]) {
    ++s_stats.checksum_errors;
    return 0U;
  }
  return 1U;
}

/* --------------------------- 批量任务组装 ---------------------------- */

static void Vision_BatchReset(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  memset(&s_batch, 0, sizeof(s_batch));
  if (mask == 0U) __enable_irq();
}

/* 仅在USART3接收中断中调用，且已通过帧校验。只统计当前活动任务的批量结果，
 * 并对INDEX做位图去重——Jetson在收到同SEQ重发时会重放已缓存的结果帧，
 * 直接累加计数会重复统计（规范§14/§21）。 */
static void Vision_BatchAccumulate(const uint8_t *frame)
{
  /* SCAN载荷（规范§10）：payload[0]=TARGET_ID、payload[1]=INDEX、payload[2]=TOTAL，
   * 对应帧字节4/5/6。COMPLETE帧则用payload[0]表示实际结果数量。 */
  uint8_t payload0 = frame[4];
  uint8_t index = frame[5];
  uint8_t total = frame[6];

  if (frame[1] != VISION_TASK_MATERIALS_SCAN &&
      frame[1] != VISION_TASK_RINGS_SCAN &&
      frame[1] != VISION_TASK_MATERIAL_ORDER) return;
  /* 必须同时匹配活动任务与SEQ：批量状态只属于当前那个任务。 */
  if (frame[1] != s_active_task || frame[3] != s_active_seq) return;

  if (frame[2] == VISION_STATUS_OK) {
    if (s_batch.expected_total == 0U) s_batch.expected_total = total;
    else if (s_batch.expected_total != total) s_batch.total_conflict = 1U;
    if (index >= 1U && index <= VISION_BATCH_INDEX_MAX) {
      uint16_t bit = (uint16_t)(1U << (index - 1U));
      if ((s_batch.received_bitmap & bit) == 0U) {
        s_batch.received_bitmap |= bit;
        ++s_batch.received_count;
      }
    }
  } else if (frame[2] == VISION_STATUS_COMPLETE) {
    s_batch.complete = 1U;
    s_batch.payload_count = payload0;
    s_batch.consistent = (uint8_t)((s_batch.total_conflict == 0U &&
                                    s_batch.payload_count == s_batch.received_count) ? 1U : 0U);
  }
}

/* ---------------------------- 接收解析 ------------------------------ */

/* 仅在USART3接收完成中断中调用。载荷也可能含帧头、帧尾字节，因此候选帧无效时
 * 逐字节滑动窗口重新同步，不能按载荷中的标记直接截断，也不能清空整个缓冲——
 * 否则会连后面的好帧一起丢掉（规范§16/§24-5、6）。 */
static void Vision_ParseByte(uint8_t byte, uint32_t now)
{
  uint8_t next;
  if (s_window_count != 0U &&
      (uint32_t)(now - s_last_byte_tick) > VISION_INTERBYTE_GAP_MS)
    s_window_count = 0U;
  s_last_byte_tick = now;

  if (s_window_count == 0U && byte != VISION_FRAME_HEAD) return;
  if (s_window_count >= VISION_RESPONSE_LEN) s_window_count = 0U;
  s_window[s_window_count++] = byte;
  if (s_window_count != VISION_RESPONSE_LEN) return;

  if (Vision_ValidFrame(s_window)) {
    ++s_stats.valid_frames;
    if (s_window[1] != s_active_task || s_window[3] != s_active_seq) ++s_stats.seq_mismatch;
    next = (uint8_t)((s_write + 1U) % VISION_QUEUE_SIZE);
    if (next != s_read) {
      s_queue[s_write].received_tick = now;
      s_queue[s_write].task = s_window[1];
      s_queue[s_write].status = s_window[2];
      s_queue[s_write].seq = s_window[3];
      memcpy(s_queue[s_write].payload, &s_window[4], VISION_PAYLOAD_LEN);
      s_write = next;
    } else {
      ++s_stats.queue_overflows;
    }
    Vision_BatchAccumulate(s_window);
    s_window_count = 0U;
    return;
  }

  /* 当前候选帧损坏时，保留窗口中出现的第一个帧头。 */
  for (next = 1U; next < VISION_RESPONSE_LEN; ++next)
    if (s_window[next] == VISION_FRAME_HEAD) break;
  if (next == VISION_RESPONSE_LEN) {
    s_window_count = 0U;
  } else {
    s_window_count = (uint8_t)(VISION_RESPONSE_LEN - next);
    memmove(s_window, &s_window[next], s_window_count);
  }
}

static void Vision_RxComplete(UART_HandleTypeDef *uart)
{
  Vision_ParseByte(s_rx_byte, HAL_GetTick());
  if (HAL_UART_Receive_IT(uart, &s_rx_byte, 1U) != HAL_OK) s_rx_fault = 1U;
}

static void Vision_RxError(UART_HandleTypeDef *uart)
{
  (void)uart;
  ++s_stats.uart_errors;
  s_rx_fault = 1U;
}

HAL_StatusTypeDef Vision_Init(void)
{
  s_window_count = s_read = s_write = 0U;
  s_last_byte_tick = 0U;
  s_rx_fault = 1U;
  s_seq = 0U;
  s_active_task = VISION_TASK_STOP;
  s_active_target = 0U;
  s_active_seq = 0U;
  s_tx_pending = 0U;
  memset(&s_stats, 0, sizeof(s_stats));
  memset(&s_batch, 0, sizeof(s_batch));
  if (HAL_UART_RegisterCallback(&huart3, HAL_UART_RX_COMPLETE_CB_ID,
                                Vision_RxComplete) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_RegisterCallback(&huart3, HAL_UART_ERROR_CB_ID,
                                Vision_RxError) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U) != HAL_OK) return HAL_ERROR;
  s_rx_fault = 0U;
  return HAL_OK;
}

/* ---------------------------- 请求发送 ------------------------------ */

/* 提交当前待发请求帧。帧内容由 active_task/target/seq 现算，因此重发不会
 * 与SEQ脱节。HAL忙时保留待发标志，下个周期继续用同一SEQ重试。 */
static HAL_StatusTypeDef Vision_ServiceRequest(void)
{
  HAL_StatusTypeDef status;

  if (s_tx_pending == 0U) return HAL_OK;
  if (huart3.gState != HAL_UART_STATE_READY) return HAL_BUSY;

  s_tx_request[0] = VISION_FRAME_HEAD;
  s_tx_request[1] = (uint8_t)s_active_task;
  s_tx_request[2] = (uint8_t)s_active_target;
  s_tx_request[3] = (uint8_t)s_active_seq;
  s_tx_request[4] = Vision_Crc8(s_tx_request, 4U);
  s_tx_request[5] = VISION_FRAME_TAIL;

  status = HAL_UART_Transmit_IT(&huart3, s_tx_request, VISION_REQUEST_LEN);
  if (status == HAL_OK) s_tx_pending = 0U;
  return status;
}

HAL_StatusTypeDef Vision_SendRequest(uint8_t task, uint8_t target)
{
  if (!Vision_ValidTask(task) || !Vision_ValidTarget(task, target)) return HAL_ERROR;

  /* TASK和TARGET都没变就是同一个逻辑任务，复用原SEQ且不重发已成功的请求：
   * 换SEQ会被Jetson当成新任务；重复的TASK+TARGET+SEQ才会被识别为重发请求
   * （规范§21）。要真正重新执行同一任务，先发 STOP 再发新请求。 */
  if (task != s_active_task || target != s_active_target) {
    ++s_seq;                       /* uint8自然回绕，SEQ=0也是合法值 */
    s_active_task = task;
    s_active_target = target;
    s_active_seq = s_seq;
    s_tx_pending = 1U;
    Vision_BatchReset();
  }
  return Vision_ServiceRequest();
}

HAL_StatusTypeDef Vision_Stop(void)
{
  return Vision_SendRequest(VISION_TASK_STOP, 0U);
}

uint8_t Vision_IsRequestPending(void)
{
  return s_tx_pending;
}

uint8_t Vision_GetActiveTask(void)
{
  return (uint8_t)s_active_task;
}

uint8_t Vision_GetActiveSeq(void)
{
  return (uint8_t)s_active_seq;
}

void Vision_ServiceRx(void)
{
  uint32_t mask;
  if (!s_rx_fault) return;
  mask = __get_PRIMASK();
  __disable_irq();
  /* 仅重启USART3接收，不影响USART1遥测和UART4电机发送。 */
  if (HAL_UART_AbortReceive(&huart3) == HAL_OK) {
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    s_window_count = 0U;
    if (HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U) == HAL_OK)
      s_rx_fault = 0U;
  }
  if (mask == 0U) __enable_irq();
}

uint8_t Vision_PopResponse(VisionResponse *response)
{
  uint32_t mask;
  if (response == NULL) return 0U;
  mask = __get_PRIMASK();
  __disable_irq();
  if (s_read == s_write) {
    if (mask == 0U) __enable_irq();
    return 0U;
  }
  *response = s_queue[s_read];
  s_read = (uint8_t)((s_read + 1U) % VISION_QUEUE_SIZE);
  if (mask == 0U) __enable_irq();
  return 1U;
}

void Vision_FlushResponses(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  s_read = s_write;
  s_window_count = 0U;
  if (mask == 0U) __enable_irq();
}

void Vision_GetStats(VisionStats *stats)
{
  uint32_t mask;
  if (stats == NULL) return;
  mask = __get_PRIMASK();
  __disable_irq();
  *stats = s_stats;
  if (mask == 0U) __enable_irq();
}

void Vision_GetBatch(VisionBatch *batch)
{
  uint32_t mask;
  if (batch == NULL) return;
  mask = __get_PRIMASK();
  __disable_irq();
  *batch = s_batch;
  if (mask == 0U) __enable_irq();
}

/* ---------------------------- 载荷解码 ------------------------------ */

static uint16_t Vision_ReadU16Be(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8U) | (uint16_t)p[1]);
}

static int16_t Vision_ReadI16Be(const uint8_t *p)
{
  return (int16_t)Vision_ReadU16Be(p);
}

uint8_t Vision_DecodeTarget(const VisionResponse *response, VisionTarget *target)
{
  VisionTarget decoded;
  const uint8_t *p;
  if (response == NULL || target == NULL || response->status != VISION_STATUS_OK)
    return 0U;
  /* SCAN载荷只用于这四个任务；MATERIAL_TRACK/RING_TRACK 在V1.1改用TRACK载荷。 */
  switch (response->task) {
    case VISION_TASK_MATERIALS_SCAN:
    case VISION_TASK_RINGS_SCAN:
    case VISION_TASK_MATERIAL_ORDER:
    case VISION_TASK_OBSTACLE_TRACK:
      break;
    default:
      return 0U;
  }
  p = response->payload;
  decoded.target_id = p[0];
  decoded.index = p[1];
  decoded.total = p[2];
  decoded.x = Vision_ReadU16Be(&p[3]);
  decoded.y = Vision_ReadU16Be(&p[5]);
  decoded.extra = Vision_ReadI16Be(&p[7]);
  decoded.confidence = p[9];
  /* 规范§10的检查项；不再约束640x480，避免把分辨率写死在协议层。 */
  if (decoded.index == 0U || decoded.total == 0U ||
      decoded.total < decoded.index || decoded.confidence > 100U)
    return 0U;
  *target = decoded;
  return 1U;
}

uint8_t Vision_DecodeTrack(const VisionResponse *response, VisionTrack *track)
{
  VisionTrack decoded;
  const uint8_t *p;
  if (response == NULL || track == NULL || response->status != VISION_STATUS_OK)
    return 0U;
  if (response->task != VISION_TASK_MATERIAL_TRACK &&
      response->task != VISION_TASK_RING_TRACK) return 0U;
  p = response->payload;
  decoded.target_id = p[0];
  decoded.coord_mode = p[1];
  decoded.valid_flags = p[2];
  decoded.error_x = Vision_ReadI16Be(&p[3]);
  decoded.error_y = Vision_ReadI16Be(&p[5]);
  decoded.yaw_error_cdeg = Vision_ReadI16Be(&p[7]);
  decoded.confidence = p[9];
  /* 不认识的坐标模式整帧作废（规范§19-4）；bit4..7必须为0；置信度0..100。 */
  if ((decoded.coord_mode != VISION_COORD_PIXEL_ERROR &&
       decoded.coord_mode != VISION_COORD_ROBOT_MM) ||
      (decoded.valid_flags & (uint8_t)(~VISION_VALID_FLAG_MASK)) != 0U ||
      decoded.confidence > 100U)
    return 0U;
  *track = decoded;
  return 1U;
}

uint8_t Vision_DecodeQr(const VisionResponse *response, VisionQr *qr)
{
  VisionQr decoded;
  const uint8_t *p;
  uint8_t i;
  if (response == NULL || qr == NULL || response->task != VISION_TASK_QR_SCAN ||
      response->status != VISION_STATUS_OK) return 0U;
  p = response->payload;
  if (p[8] > 100U || p[9] != 0U) return 0U;
  for (i = 0U; i < 4U; ++i)
    decoded.group[i] = Vision_ReadU16Be(&p[2U * i]);
  decoded.confidence = p[8];
  *qr = decoded;
  return 1U;
}

uint8_t Vision_IsTrackingFresh(const VisionResponse *response, uint32_t now_tick)
{
  if (response == NULL || response->status != VISION_STATUS_OK) return 0U;
  /* 物料/圆环跟踪用TRACK载荷，障碍物检测按规范§10仍用SCAN载荷。 */
  if (response->task == VISION_TASK_MATERIAL_TRACK ||
      response->task == VISION_TASK_RING_TRACK) {
    VisionTrack track;
    if (!Vision_DecodeTrack(response, &track)) return 0U;
  } else if (response->task == VISION_TASK_OBSTACLE_TRACK) {
    VisionTarget scan;
    if (!Vision_DecodeTarget(response, &scan)) return 0U;
  } else {
    return 0U;
  }
  return (uint8_t)((uint32_t)(now_tick - response->received_tick) <=
                   VISION_TRACK_TIMEOUT_MS);
}
