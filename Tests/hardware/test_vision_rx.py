"""用主机HAL桩编译真实USART3视觉驱动，回放V1.1（坐标系修正版）响应帧。

覆盖三层：
  ① 协议层：CRC-8变体、6字节请求组帧与SEQ管理、16字节响应解析与重同步、批量组装；
  ② 应用层：两种COORD_MODE的增益方向、VALID_FLAGS逐轴放行、150ms时效、SEQ过滤；
  ③ 对端基准：规范§22的标准测试向量与《V1.1请求帧速查》的实测CRC表。
"""
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
STUB_MAIN = r"""
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef struct { void *Instance; uint32_t gState; } UART_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2 } HAL_StatusTypeDef;
typedef void (*pUART_CallbackTypeDef)(UART_HandleTypeDef *);
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define HAL_UART_RX_COMPLETE_CB_ID 1
#define HAL_UART_ERROR_CB_ID 2
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef *, int,
                                            pUART_CallbackTypeDef);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
#endif
"""

TEST_C = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vision.h"
#include "vision_track.h"
#include "usart.h"

UART_HandleTypeDef huart3;
static pUART_CallbackTypeDef rx_cb, error_cb;
static uint8_t *rx_address;
static uint32_t tick;
static unsigned abort_count;
static unsigned tx_count;
static uint8_t tx_request[VISION_REQUEST_LEN];

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __enable_irq(void) {}
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef *uart, int id,
                                            pUART_CallbackTypeDef cb) {
  assert(uart == &huart3);
  if (id == HAL_UART_RX_COMPLETE_CB_ID) rx_cb = cb;
  else if (id == HAL_UART_ERROR_CB_ID) error_cb = cb;
  else assert(0);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,
                                      uint8_t *data, uint16_t len) {
  assert(uart == &huart3 && len == 1);
  rx_address = data;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) {
  assert(uart == &huart3);
  ++abort_count;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                       const uint8_t *data, uint16_t len) {
  assert(uart == &huart3 && len == VISION_REQUEST_LEN);   /* V1.1固定6字节 */
  memcpy(tx_request, data, len);
  ++tx_count;
  uart->gState = HAL_UART_STATE_BUSY_TX;
  return HAL_OK;
}

/* --------------------------- 测试辅助 ---------------------------- */

/* 与固件独立的CRC-8参考实现（poly 0x07 / init 0x00 / 不反转）。 */
static uint8_t crc8(const uint8_t *data, unsigned len) {
  uint8_t crc = 0x00;
  unsigned i, b;
  for (i = 0; i < len; ++i) {
    crc ^= data[i];
    for (b = 0; b < 8; ++b)
      crc = (uint8_t)((crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U)
                                    : (uint8_t)(crc << 1));
  }
  return crc;
}

/* 浮点近似比较：增益系数是二进制不可精确表示的float，不能用 ==。 */
static int near_(float got, float want) {
  float d = got - want;
  if (d < 0.0f) d = -d;
  return (d < 0.01f) ? 1 : 0;
}

static void feed(uint8_t byte, uint32_t delay_ms) {
  tick += delay_ms;
  assert(rx_address && rx_cb);
  *rx_address = byte;
  rx_cb(&huart3);
}

static void send_bytes(const uint8_t *bytes, unsigned count) {
  unsigned i;
  for (i = 0; i < count; ++i) feed(bytes[i], 1);
}

/* 组一条V1.1响应帧：66 TASK STATUS SEQ PAYLOAD[10] CRC8 77。 */
static void make_frame(uint8_t frame[VISION_RESPONSE_LEN], uint8_t task,
                       uint8_t status, uint8_t seq, const uint8_t payload[10]) {
  frame[0] = 0x66;
  frame[1] = task;
  frame[2] = status;
  frame[3] = seq;
  memcpy(frame + 4, payload, 10);
  frame[14] = crc8(frame, 14);
  frame[15] = 0x77;
}

static void send_frame(const uint8_t frame[VISION_RESPONSE_LEN]) {
  send_bytes(frame, VISION_RESPONSE_LEN);
}

/* 请求帧结构断言：帧头、TASK、TARGET、SEQ、CRC、帧尾。 */
static void expect_request(uint8_t task, uint8_t target, uint8_t seq) {
  assert(tx_request[0] == 0x66);
  assert(tx_request[1] == task);
  assert(tx_request[2] == target);
  assert(tx_request[3] == seq);
  assert(tx_request[4] == crc8(tx_request, 4));
  assert(tx_request[5] == 0x77);
}

static void track_payload(uint8_t p[10], uint8_t id, uint8_t mode, uint8_t flags,
                          int16_t ex, int16_t ey, int16_t yaw, uint8_t conf) {
  p[0] = id; p[1] = mode; p[2] = flags;
  p[3] = (uint8_t)(ex >> 8);   p[4] = (uint8_t)(ex & 0xFF);
  p[5] = (uint8_t)(ey >> 8);   p[6] = (uint8_t)(ey & 0xFF);
  p[7] = (uint8_t)(yaw >> 8);  p[8] = (uint8_t)(yaw & 0xFF);
  p[9] = conf;
}

static void scan_payload(uint8_t p[10], uint8_t id, uint8_t index, uint8_t total,
                         uint16_t x, uint16_t y, int16_t extra, uint8_t conf) {
  p[0] = id; p[1] = index; p[2] = total;
  p[3] = (uint8_t)(x >> 8);     p[4] = (uint8_t)(x & 0xFF);
  p[5] = (uint8_t)(y >> 8);     p[6] = (uint8_t)(y & 0xFF);
  p[7] = (uint8_t)(extra >> 8); p[8] = (uint8_t)(extra & 0xFF);
  p[9] = conf;
}

/* 推一条TRACK响应并跑一个控制周期。 */
static void push_track(uint8_t status, uint8_t seq, const uint8_t p[10],
                       VisionTrackOutput *motion) {
  uint8_t f[VISION_RESPONSE_LEN];
  make_frame(f, VISION_TASK_MATERIAL_TRACK, status, seq, p);
  send_frame(f);
  VisionTrack_Service(tick, motion);
}

int main(void) {
  /* 规范§22与《V1.1请求帧速查》的请求帧：TASK / TARGET / SEQ / CRC8 */
  static const uint8_t req_vectors[][4] = {
    {0x00, 0x00, 0x02, 0x2F},   /* STOP */
    {0x01, 0x00, 0x03, 0x43},   /* MATERIALS_SCAN */
    {0x03, 0x00, 0x04, 0x80},   /* RINGS_SCAN */
    {0x08, 0x00, 0x05, 0x6B},   /* MATERIAL_ORDER */
    {0x09, 0x00, 0x06, 0x09},   /* QR_SCAN */
    {0x06, 0x00, 0x07, 0x49},   /* OBSTACLE_TRACK */
    {0x02, 0x01, 0x08, 0xDA},   /* MATERIAL_TRACK 红 */
    {0x02, 0x02, 0x09, 0xE2},   /* 绿 */
    {0x02, 0x03, 0x0A, 0xFE},   /* 蓝 */
    {0x02, 0x04, 0x0B, 0x92},   /* 黄 */
    {0x02, 0x05, 0x0C, 0x92},   /* 黑：与黄的CRC碰撞，属正常 */
    {0x02, 0x06, 0x0D, 0xAA},   /* 浅蓝 */
    {0x04, 0x01, 0x0E, 0xB5},   /* RING_TRACK 1 */
    {0x04, 0x02, 0x0F, 0x8D},   /* RING_TRACK 2 */
    {0x04, 0x03, 0x10, 0xC5},   /* RING_TRACK 3 */
  };
  /* 规范§22.3/§22.4/§22.5 的完整响应帧 */
  static const uint8_t spec_track[VISION_RESPONSE_LEN] = {
    0x66,0x02,0x00,0x35,0x01,0x02,0x0F,0x00,0x12,0xFF,0xF9,0x00,0x7D,0x5C,0xEA,0x77};
  static const uint8_t spec_qr[VISION_RESPONSE_LEN] = {
    0x66,0x09,0x00,0x01,0x00,0x9C,0x00,0x7B,0x02,0x04,0x00,0xE7,0x5F,0x00,0x9E,0x77};
  static const uint8_t spec_complete[VISION_RESPONSE_LEN] = {
    0x66,0x01,0x02,0x22,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x77};

  uint8_t frame[VISION_RESPONSE_LEN], bad[VISION_RESPONSE_LEN], payload[10];
  VisionResponse response;
  VisionTrack track;
  VisionTarget scan;
  VisionQr qr;
  VisionStats stats, before;
  VisionBatch batch;
  VisionTrackOutput motion;
  uint8_t seq;
  uint32_t v0;
  unsigned i;

  /* ============ ① CRC-8 变体锁定 ============ */
  assert(crc8((const uint8_t *)"123456789", 9) == 0xF4);   /* 规范§3自检值 */
  for (i = 0; i < sizeof(req_vectors) / sizeof(req_vectors[0]); ++i) {
    uint8_t body[4];
    body[0] = 0x66; body[1] = req_vectors[i][0];
    body[2] = req_vectors[i][1]; body[3] = req_vectors[i][2];
    assert(crc8(body, 4) == req_vectors[i][3]);
  }
  assert(crc8(spec_track, 14) == 0xEA);       /* 响应CRC覆盖前14字节 */
  assert(crc8(spec_qr, 14) == 0x9E);
  assert(crc8(spec_complete, 14) == 0x0A);

  /* ============ ② 请求组帧与SEQ管理 ============ */
  assert(Vision_Init() == HAL_OK);
  assert(Vision_SendRequest(VISION_TASK_MATERIAL_TRACK, 1) == HAL_OK);
  assert(Vision_GetActiveTask() == VISION_TASK_MATERIAL_TRACK);
  seq = Vision_GetActiveSeq();
  expect_request(0x02, 0x01, seq);
  assert(!Vision_IsRequestPending());

  /* 同一个逻辑任务重复调用：不换SEQ、也不重复发送。 */
  v0 = tx_count;
  assert(Vision_SendRequest(VISION_TASK_MATERIAL_TRACK, 1) == HAL_OK);
  assert(tx_count == v0 && Vision_GetActiveSeq() == seq);

  /* 新逻辑任务换新SEQ。（桩不模拟TX完成中断，需显式把gState复位。） */
  huart3.gState = HAL_UART_STATE_READY;
  assert(Vision_Stop() == HAL_OK);
  assert(Vision_GetActiveSeq() != seq);
  expect_request(0x00, 0x00, Vision_GetActiveSeq());

  /* HAL忙：保留待发，重试必须复用同一SEQ（换SEQ会被Jetson当成新任务）。 */
  huart3.gState = HAL_UART_STATE_BUSY_TX;
  assert(Vision_SendRequest(VISION_TASK_MATERIAL_TRACK, 1) == HAL_BUSY);
  assert(Vision_IsRequestPending());
  seq = Vision_GetActiveSeq();
  v0 = tx_count;
  huart3.gState = HAL_UART_STATE_READY;
  assert(Vision_SendRequest(VISION_TASK_MATERIAL_TRACK, 1) == HAL_OK);
  assert(tx_count == v0 + 1U && Vision_GetActiveSeq() == seq);
  expect_request(0x02, 0x01, seq);
  assert(!Vision_IsRequestPending());

  /* 参数非法必须拒绝且不发送。 */
  v0 = tx_count;
  assert(Vision_SendRequest(VISION_TASK_MATERIAL_TRACK, 0) == HAL_ERROR);
  assert(Vision_SendRequest(VISION_TASK_MATERIAL_TRACK, 7) == HAL_ERROR);
  assert(Vision_SendRequest(VISION_TASK_RING_TRACK, 4) == HAL_ERROR);
  assert(Vision_SendRequest(VISION_TASK_QR_SCAN, 1) == HAL_ERROR);
  assert(Vision_SendRequest(0x05, 0) == HAL_ERROR);
  assert(tx_count == v0);

  /* ============ ③ 响应解析：规范标准向量 ============ */
  assert(!Vision_PopResponse(&response));
  feed(0x42, 1);                              /* 帧前噪声 */
  send_frame(spec_track);
  assert(Vision_PopResponse(&response));
  assert(response.task == VISION_TASK_MATERIAL_TRACK);
  assert(response.status == VISION_STATUS_OK);
  assert(response.seq == 0x35);
  assert(Vision_DecodeTrack(&response, &track));
  assert(track.target_id == 1);
  assert(track.coord_mode == VISION_COORD_ROBOT_MM);
  assert(track.valid_flags == 0x0F);
  assert(track.error_x == 18 && track.error_y == -7 && track.yaw_error_cdeg == 125);
  assert(track.confidence == 92);
  /* TRACK载荷不得再被SCAN解码器接受。 */
  assert(!Vision_DecodeTarget(&response, &scan));

  /* 任务与SEQ都匹配的帧不计入seq_mismatch，不匹配的计一次。 */
  Vision_GetStats(&before);
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 0x6677, -100, 0, 80);
  make_frame(frame, VISION_TASK_MATERIAL_TRACK, VISION_STATUS_OK, seq, payload);
  send_frame(frame);
  Vision_GetStats(&stats);
  assert(stats.seq_mismatch == before.seq_mismatch);
  assert(Vision_PopResponse(&response));
  assert(Vision_DecodeTrack(&response, &track) && track.error_x == 0x6677);
  make_frame(frame, VISION_TASK_MATERIAL_TRACK, VISION_STATUS_OK,
             (uint8_t)(seq + 1U), payload);
  send_frame(frame);
  Vision_GetStats(&stats);
  assert(stats.seq_mismatch == before.seq_mismatch + 1U);
  assert(Vision_PopResponse(&response));

  /* 载荷中的0x66/0x77不能被当作帧边界（上一条已断言error_x==0x6677）。 */
  make_frame(frame, VISION_TASK_MATERIAL_TRACK, VISION_STATUS_OK, seq, payload);

  /* CRC错误：仅丢弃候选帧头，滑动找到下一条好帧。 */
  memcpy(bad, frame, VISION_RESPONSE_LEN);
  bad[14] ^= 1U;                              /* 破坏CRC */
  send_frame(bad);
  assert(!Vision_PopResponse(&response));
  feed(0x66, 1);
  feed(0x11, 1);
  send_frame(frame);
  assert(Vision_PopResponse(&response));
  assert(Vision_DecodeTrack(&response, &track));

  /* 帧尾错误同样重同步。 */
  memcpy(bad, frame, VISION_RESPONSE_LEN);
  bad[15] = 0x00;
  send_frame(bad);
  assert(!Vision_PopResponse(&response));
  send_frame(frame);
  assert(Vision_PopResponse(&response));

  /* 字节间隔过长时丢弃残帧（16字节帧的第4字节是SEQ）。 */
  feed(0x66, 1);
  feed(0x02, 1);
  feed(0x00, 21);
  send_frame(frame);
  assert(Vision_PopResponse(&response));
  assert(!Vision_PopResponse(&response));

  /* 未知任务号即使CRC正确也必须拒绝。 */
  make_frame(frame, 0x05, VISION_STATUS_OK, seq, payload);
  send_frame(frame);
  assert(!Vision_PopResponse(&response));

  /* STATUS允许到0x06 INVALID_DATA，非OK时payload[0]是错误详情。 */
  memset(payload, 0, sizeof(payload));
  payload[0] = VISION_ERR_QR_FORMAT_INVALID;
  make_frame(frame, VISION_TASK_MATERIAL_TRACK, VISION_STATUS_INVALID_DATA, seq, payload);
  send_frame(frame);
  assert(Vision_PopResponse(&response));
  assert(response.status == VISION_STATUS_INVALID_DATA);
  assert(response.payload[0] == VISION_ERR_QR_FORMAT_INVALID);
  assert(!Vision_DecodeTrack(&response, &track));    /* 非OK不解码 */

  /* QR与SCAN载荷。 */
  send_frame(spec_qr);
  assert(Vision_PopResponse(&response));
  assert(Vision_DecodeQr(&response, &qr));
  assert(qr.group[0] == 156 && qr.group[1] == 123 &&
         qr.group[2] == 516 && qr.group[3] == 231 && qr.confidence == 95);
  scan_payload(payload, 1, 1, 1, 320, 240, 64, 90);
  make_frame(frame, VISION_TASK_MATERIALS_SCAN, VISION_STATUS_OK, seq, payload);
  send_frame(frame);
  assert(Vision_PopResponse(&response));
  assert(Vision_DecodeTarget(&response, &scan));
  assert(scan.target_id == 1 && scan.index == 1 && scan.total == 1);
  assert(scan.x == 320 && scan.y == 240 && scan.extra == 64 && scan.confidence == 90);
  assert(!Vision_DecodeTrack(&response, &track));

  /* 规范§22.5 的COMPLETE整帧。 */
  send_frame(spec_complete);
  assert(Vision_PopResponse(&response));
  assert(response.task == VISION_TASK_MATERIALS_SCAN);
  assert(response.status == VISION_STATUS_COMPLETE);
  assert(response.seq == 0x22 && response.payload[0] == 3U);

  /* 队满丢新帧：16槽实存15。 */
  make_frame(frame, VISION_TASK_MATERIAL_TRACK, VISION_STATUS_OK, seq, payload);
  for (i = 0; i < 17U; ++i) send_frame(frame);
  for (i = 0; i < 15U; ++i) assert(Vision_PopResponse(&response));
  assert(!Vision_PopResponse(&response));

  /* 接收错误后重挂，Flush清空队列与残帧。 */
  assert(error_cb);
  error_cb(&huart3);
  Vision_ServiceRx();
  assert(abort_count == 1);
  send_frame(frame);
  Vision_FlushResponses();
  assert(!Vision_PopResponse(&response));
  send_frame(frame);
  assert(Vision_PopResponse(&response));

  /* 统计量：错误计数用下界，valid_frames用增量，避免帧数微调时脆断。 */
  Vision_GetStats(&before);
  assert(before.valid_frames > 0U);
  assert(before.checksum_errors >= 1U);
  assert(before.format_errors >= 1U);
  assert(before.queue_overflows >= 1U);
  assert(before.uart_errors == 1U);
  v0 = before.valid_frames;
  send_frame(frame);
  Vision_GetStats(&stats);
  assert(stats.valid_frames == v0 + 1U);

  /* ============ ④ 批量任务组装（规范§14） ============ */
  /* 切到批量任务：新任务会清零批量状态。 */
  huart3.gState = HAL_UART_STATE_READY;
  assert(Vision_SendRequest(VISION_TASK_MATERIALS_SCAN, 0) == HAL_OK);
  seq = Vision_GetActiveSeq();
  Vision_GetBatch(&batch);
  assert(batch.received_count == 0U && batch.complete == 0U);

  /* 非活动SEQ的批量帧不计入。 */
  scan_payload(payload, 1, 1, 3, 10, 10, 0, 90);
  make_frame(frame, VISION_TASK_MATERIALS_SCAN, VISION_STATUS_OK,
             (uint8_t)(seq ^ 0x5AU), payload);
  send_frame(frame);
  Vision_GetBatch(&batch);
  assert(batch.received_count == 0U);

  /* INDEX位图去重：同一INDEX的重复帧只算一次。 */
  scan_payload(payload, 1, 1, 3, 100, 100, 0, 90);
  make_frame(frame, VISION_TASK_MATERIALS_SCAN, VISION_STATUS_OK, seq, payload);
  send_frame(frame);
  send_frame(frame);
  scan_payload(payload, 1, 2, 3, 200, 200, 0, 90);
  make_frame(frame, VISION_TASK_MATERIALS_SCAN, VISION_STATUS_OK, seq, payload);
  send_frame(frame);
  Vision_GetBatch(&batch);
  assert(batch.expected_total == 3U);
  assert(batch.received_count == 2U);
  assert((batch.received_bitmap & 0x0003U) == 0x0003U);

  /* COMPLETE报3条而本地只有2条 ⇒ 判定不完整。 */
  payload[0] = 3U;
  make_frame(frame, VISION_TASK_MATERIALS_SCAN, VISION_STATUS_COMPLETE, seq, payload);
  send_frame(frame);
  Vision_GetBatch(&batch);
  assert(batch.complete == 1U && batch.payload_count == 3U);
  assert(batch.received_count == 2U && batch.consistent == 0U);

  /* 补齐第3条后重新COMPLETE即一致。 */
  scan_payload(payload, 1, 3, 3, 300, 300, 0, 90);
  make_frame(frame, VISION_TASK_MATERIALS_SCAN, VISION_STATUS_OK, seq, payload);
  send_frame(frame);
  payload[0] = 3U;
  make_frame(frame, VISION_TASK_MATERIALS_SCAN, VISION_STATUS_COMPLETE, seq, payload);
  send_frame(frame);
  Vision_GetBatch(&batch);
  assert(batch.received_count == 3U && batch.consistent == 1U);

  /* ============ ⑤ 应用层：TRACK闭环 ============ */
  huart3.gState = HAL_UART_STATE_READY;
  assert(VisionTrack_Start(1));
  VisionTrack_Service(tick, &motion);
  assert(!motion.move);
  expect_request(VISION_TASK_MATERIAL_TRACK, 1, Vision_GetActiveSeq());
  seq = Vision_GetActiveSeq();

  /* ROBOT_MM模式：默认vTrackKpMm=0.5RPM/mm、10mm死区、8~60RPM钳位。 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 100, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 50.0f) && motion.vy_rpm == 0.0f);
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, -100, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, -50.0f));
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_Y, 0, 100, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && motion.vx_rpm == 0.0f && near_(motion.vy_rpm, 50.0f));
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_Y, 0, -100, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vy_rpm, -50.0f));

  /* 死区：默认 VDBMM=2mm，|误差|<2mm 不动。 */
  assert(near_(vTrackDeadbandMm, 2.0f));      /* 锁住本次调参目标 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 1, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, -1, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);
  /* 刚出死区立刻抬到最小速度：3mm*0.5=1.5RPM → 8RPM。 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 3, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 8.0f));
  /* 出死区后仍有最小速度抬升：15mm*0.5=7.5RPM → 8RPM。 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 15, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 8.0f));
  /* 上限钳位：500mm*0.5=250RPM → 60RPM。 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 500, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 60.0f));

  /* VALID_FLAGS逐轴放行：X无效时该轴必须为0，不能把无效当成"误差恰好为0"。 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_Y, 100, 100, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && motion.vx_rpm == 0.0f && near_(motion.vy_rpm, 50.0f));
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, 0x00, 100, 100, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);

  /* PIXEL_ERROR模式用另一套系数：0.18RPM/像素、12像素死区、上限60RPM。 */
  track_payload(payload, 1, VISION_COORD_PIXEL_ERROR, VISION_VALID_X, 100, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 100.0f * 0.18f));
  track_payload(payload, 1, VISION_COORD_PIXEL_ERROR, VISION_VALID_X, 11, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);
  track_payload(payload, 1, VISION_COORD_PIXEL_ERROR, VISION_VALID_X, 400, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 60.0f));

  /* 死区与限速都是RAM可调参数：VDBMM/VDBPX动死区，VMIN/VMAX动输出上下限。 */
  vTrackDeadbandMm = 20.0f;
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 15, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);                       /* 15mm落进20mm死区 */
  vTrackDeadbandPx = 100.0f;
  track_payload(payload, 1, VISION_COORD_PIXEL_ERROR, VISION_VALID_X, 50, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);                       /* 50像素落进100像素死区 */
  vTrackDeadbandMm = 2.0f;
  vTrackDeadbandPx = 12.0f;
  vTrackMinRpm = 0.0f;
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 15, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 7.5f));   /* 关掉抬升后是纯比例 */
  vTrackMinRpm = 8.0f;
  vTrackMaxRpm = 30.0f;
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 500, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move && near_(motion.vx_rpm, 30.0f));  /* 上限改成30RPM */
  vTrackMaxRpm = 60.0f;                                /* 恢复默认，后续用例依赖默认值 */

  /* 不认识的COORD_MODE整帧作废（规范§19-4）；VALID_FLAGS高4位必须为0；
   * 置信度必须≤100。 */
  track_payload(payload, 1, 0x03, VISION_VALID_X, 100, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, 0xF0, 100, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 100, 0, 0, 101);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);

  /* 置信度低于vTrackConfMin(默认50)不参与控制，等于阈值则控制。 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 100, 0, 0, 49);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 100, 0, 0, 50);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move);

  /* 异色目标不驱动本次跟踪。 */
  track_payload(payload, 2, VISION_COORD_ROBOT_MM, VISION_VALID_X, 100, 0, 0, 90);
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(!motion.move);

  /* SEQ不匹配（旧任务迟到回包）必须丢弃。 */
  track_payload(payload, 1, VISION_COORD_ROBOT_MM, VISION_VALID_X, 100, 0, 0, 90);
  push_track(VISION_STATUS_OK, (uint8_t)(seq ^ 0xA5U), payload, &motion);
  assert(!motion.move);

  /* 150ms时效边界（规范§17）。 */
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move);
  tick += VISION_TRACK_TIMEOUT_MS;
  VisionTrack_Service(tick, &motion);
  assert(motion.move);
  tick += 1U;
  VisionTrack_Service(tick, &motion);
  assert(!motion.move);

  /* NOT_FOUND / BUSY 立即停用旧结果，但不退出跟踪。 */
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move);
  push_track(VISION_STATUS_NOT_FOUND, seq, payload, &motion);
  assert(!motion.move && VisionTrack_IsActive());
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move);
  push_track(VISION_STATUS_BUSY, seq, payload, &motion);
  assert(!motion.move && VisionTrack_IsActive());

  /* BAD_ARGUMENT / ERROR / INVALID_DATA 不可自愈：退出跟踪并回停止请求。 */
  push_track(VISION_STATUS_OK, seq, payload, &motion);
  assert(motion.move);
  push_track(VISION_STATUS_INVALID_DATA, seq, payload, &motion);
  assert(!motion.move && !VisionTrack_IsActive());
  seq = Vision_GetActiveSeq();                /* 退出前的TRACK序号 */
  huart3.gState = HAL_UART_STATE_READY;
  VisionTrack_Service(tick, &motion);         /* 停止请求在下一周期发出 */
  assert(Vision_GetActiveTask() == VISION_TASK_STOP);
  assert(Vision_GetActiveSeq() != seq);       /* STOP是新逻辑任务，必须换新SEQ */
  expect_request(0x00, 0x00, Vision_GetActiveSeq());

  puts("vision V1.1 RX + track tests passed");
  return 0;
}
"""


with tempfile.TemporaryDirectory() as temporary:
    temp = Path(temporary)
    (temp / "main.h").write_text(STUB_MAIN, encoding="utf-8")
    (temp / "usart.h").write_text(
        '#include "main.h"\nextern UART_HandleTypeDef huart3;\n', encoding="utf-8"
    )
    (temp / "test.c").write_text(TEST_C, encoding="utf-8")
    exe = temp / "vision_test.exe"
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
         "-I", str(temp), "-I", str(ROOT / "Hardware"),
         str(ROOT / "Hardware/vision.c"),
         str(ROOT / "Hardware/vision_track.c"), str(temp / "test.c"),
         "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)
