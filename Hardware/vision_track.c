/**
 * @file vision_track.c
 * @brief 视觉物料精对准闭环，按V1.1的TRACK载荷计算机体速度。
 *
 * 误差量已是车体控制轴语义（+X=左、+Y=前），本文件不再做任何方向翻转：
 * V1.0 那套"画面右偏→车后"的像素映射来自旧协议只给原始图像坐标，
 * V1.1 明确由Jetson完成相机到车体的方向换算。
 */
#include "vision_track.h"
#include "vision.h"

/* 像素模式系数（COORD_MODE=PIXEL_ERROR）：误差单位是像素，增益沿用V1的实车值。
 * 固件里没有"毫米/像素"标定系数，所以像素模式的死区只能按像素给、无法直接表达
 * "2mm"；需要毫米级死区时要让Jetson用ROBOT_MM模式。 */
#define VTRACK_PX_KP_RPM_PER_PX     0.18f
#define VTRACK_PX_DEADBAND_DEFAULT  12.0f      //像素，低于此误差不驱动

/* 毫米模式系数（COORD_MODE=ROBOT_MM）：误差单位是mm，规范§11.2要求与像素模式
 * 分开配置。0.5RPM/mm 使100mm误差产生50RPM，落在8~60的钳位区间内。
 * 不要直接复用位置环的 mKpx/mKpy——它们是 (mm/s)/mm，与轮侧RPM相差0.238倍。 */
#define VTRACK_MM_KP_DEFAULT        0.5f       //RPM/mm，误差100mm产生50RPM
#define VTRACK_MM_DEADBAND_DEFAULT  2.0f       //mm，低于此误差不驱动

/* 输出下限/上限是限幅而非增益，两种模式共用；增益与死区必须分开。
 * 死区收紧后要一并看最小时速：8RPM≈33.6mm/s，20ms控制周期就走约0.67mm，
 * 而2mm死区单侧只有2mm余量，一帧过冲就够穿出去，容易形成围绕目标的极限环
 * （停→起→停）。真出现来回抖时应先降 VMIN，不要只放大死区。 */
#define VTRACK_MIN_RPM_DEFAULT      8.0f
#define VTRACK_MAX_RPM_DEFAULT      60.0f
#define VTRACK_CONF_MIN_DEFAULT     50.0f

/* 视觉调参，可被 VCONF/VKPMM/VDBMM/VDBPX/VMIN/VMAX 覆盖。 */
float vTrackConfMin = VTRACK_CONF_MIN_DEFAULT;
float vTrackKpMm = VTRACK_MM_KP_DEFAULT;
float vTrackDeadbandMm = VTRACK_MM_DEADBAND_DEFAULT;
float vTrackDeadbandPx = VTRACK_PX_DEADBAND_DEFAULT;
float vTrackMinRpm = VTRACK_MIN_RPM_DEFAULT;
float vTrackMaxRpm = VTRACK_MAX_RPM_DEFAULT;

#define VTRACK_REQUEST_NONE         0U
#define VTRACK_REQUEST_STOP         1U
#define VTRACK_REQUEST_START        2U

static volatile uint8_t s_active;
static uint8_t s_color;
static uint8_t s_request;
static uint8_t s_request_sent;
static uint8_t s_track_valid;      /* 最近一条结果是否为新鲜且合法的TRACK帧 */
static VisionResponse s_latest_response;
static VisionTrack s_latest_track;

/* 死区 + 最小速度抬升 + 上限钳位。逐轴独立调用：只在对应VALID位存在时才调用，
 * 因此"该轴无效"不会被当成"误差恰好为0"（规范§12）。 */
static float VisionTrack_AxisRpm(int16_t error, float kp, float deadband)
{
  float value = (float)error;
  if (value > -deadband && value < deadband) return 0.0f;
  value *= kp;
  if (value > 0.0f && value < vTrackMinRpm) value = vTrackMinRpm;
  if (value < 0.0f && value > -vTrackMinRpm) value = -vTrackMinRpm;
  if (value > vTrackMaxRpm) value = vTrackMaxRpm;
  if (value < -vTrackMaxRpm) value = -vTrackMaxRpm;
  return value;
}

uint8_t VisionTrack_Start(uint8_t color)
{
  if (color < 1U || color > 6U) return 0U;
  Vision_FlushResponses();
  s_color = color;
  s_active = 1U;
  s_track_valid = 0U;
  s_request_sent = 0U;
  s_request = VTRACK_REQUEST_START;
  return 1U;
}

void VisionTrack_Stop(void)
{
  Vision_FlushResponses();
  s_active = 0U;
  s_track_valid = 0U;
  s_request_sent = 0U;
  s_request = VTRACK_REQUEST_STOP;
}

uint8_t VisionTrack_IsActive(void)
{
  return s_active;
}

void VisionTrack_Service(uint32_t now_tick, VisionTrackOutput *output)
{
  VisionResponse response;
  VisionTrack track;
  float kp, deadband;

  if (output == NULL) return;
  output->vx_rpm = output->vy_rpm = 0.0f;
  output->move = 0U;

  /* Vision_SendRequest 是幂等的：TASK+TARGET不变就复用同一个SEQ，HAL忙则下个
   * 周期继续提交，因此每周期调用不会白白消耗序号，也不会被Jetson当成新任务。 */
  if (s_request == VTRACK_REQUEST_STOP) {
    if (Vision_Stop() == HAL_OK) s_request = VTRACK_REQUEST_NONE;
  } else if (s_request == VTRACK_REQUEST_START) {
    if (Vision_SendRequest(VISION_TASK_MATERIAL_TRACK, s_color) == HAL_OK) {
      s_request = VTRACK_REQUEST_NONE;
      s_request_sent = 1U;
    }
  }

  if (!s_active) return;

  while (Vision_PopResponse(&response)) {
    /* 只采用当前活动任务、同一SEQ的结果；迟到的旧任务帧直接丢弃（规范§8）。 */
    if (!s_request_sent) continue;
    if (response.task != Vision_GetActiveTask() ||
        response.seq != Vision_GetActiveSeq()) continue;
    if (response.task != VISION_TASK_MATERIAL_TRACK) continue;

    if (response.status == VISION_STATUS_BAD_ARGUMENT ||
        response.status == VISION_STATUS_ERROR ||
        response.status == VISION_STATUS_INVALID_DATA) {
      /* 这三类错误不会自愈：退出跟踪，由调试层停车并回ACK上报。 */
      VisionTrack_Stop();
      break;
    }

    if (response.status != VISION_STATUS_OK ||
        !Vision_DecodeTrack(&response, &track) ||
        track.target_id != s_color ||
        (float)track.confidence < vTrackConfMin) {
      /* NOT_FOUND / BUSY / 低置信度 / 异色 / 载荷非法：本帧不能作为控制依据。
       * 保留s_latest_track里的旧值但不置有效位，等效于"立刻停用旧坐标"。 */
      s_track_valid = 0U;
      continue;
    }
    s_latest_response = response;
    s_latest_track = track;
    s_track_valid = 1U;
  }

  if (!s_active || !s_track_valid ||
      !Vision_IsTrackingFresh(&s_latest_response, now_tick)) {
    s_track_valid = 0U;
    return;
  }

  /* 两种坐标模式各用一套系数；误差已是车体轴语义，直接映射，无交换无取反。 */
  if (s_latest_track.coord_mode == VISION_COORD_ROBOT_MM) {
    kp = vTrackKpMm;
    deadband = vTrackDeadbandMm;
  } else {
    kp = VTRACK_PX_KP_RPM_PER_PX;
    deadband = vTrackDeadbandPx;
  }
  if ((s_latest_track.valid_flags & VISION_VALID_X) != 0U)
    output->vx_rpm = VisionTrack_AxisRpm(s_latest_track.error_x, kp, deadband);
  if ((s_latest_track.valid_flags & VISION_VALID_Y) != 0U)
    output->vy_rpm = VisionTrack_AxisRpm(s_latest_track.error_y, kp, deadband);
  /* YAW误差与STABLE位已解码保留，但本轮不驱动旋转（vz保持0）。 */
  output->move = (output->vx_rpm != 0.0f || output->vy_rpm != 0.0f) ? 1U : 0U;
}
