/**
 * @file vision_track.h
 * @brief 视觉物料精对准闭环，按V1.1的TRACK载荷计算机体速度。
 *
 * 只计算车体速度，不直接控制电机。调试任务负责停车、四轮使能闸门和
 * MANUAL/GOTO/ZDT 等控制权仲裁。
 *
 * 坐标与符号：协议（V1.1坐标系修正版）与本工程一致——+X=车左、+Y=车头，
 * TRACK载荷的 error_x>0 表示需要向左、error_y>0 表示需要向前，因此直接映射到
 * MecanumControl_MoveVelocity(vx=左, vy=前, vz=逆时针)，不做轴交换或取反。
 * 方向换算由Jetson按相机安装关系完成，固件不得再按图像u/v自行翻转。
 */
#ifndef ILHC_VISION_TRACK_H
#define ILHC_VISION_TRACK_H

#include <stdint.h>

typedef struct {
  float vx_rpm;             /* 车体X速度：正数向车左 */
  float vy_rpm;             /* 车体Y速度：正数向车头 */
  uint8_t move;             /* 1表示输出非零速度，0表示应停车 */
} VisionTrackOutput;

/* 视觉调参：在 debug_usart.c 的参数表里暴露为 VCONF / VKPMM / VDBMM / VDBPX /
 * VMIN / VMAX，支持 "名称=值" 设置与 GET 名称 回读。
 * 它们是 RAM 参数：不随底盘参数存Flash（Flash记录固定16字段、VERSION=2），
 * 掉电回到 vision_track.c 里的编译期默认值。 */
extern float vTrackConfMin;      /* 最小置信度阈值，0..100 */
extern float vTrackKpMm;         /* ROBOT_MM模式增益，RPM/mm */
extern float vTrackDeadbandMm;   /* ROBOT_MM模式死区，mm */
extern float vTrackDeadbandPx;   /* PIXEL_ERROR模式死区，像素 */
extern float vTrackMinRpm;       /* 出死区后的最小速度，RPM */
extern float vTrackMaxRpm;       /* 每轴速度上限，RPM */

/* 仅在默认任务调用。颜色编号范围1..6；开始时清除旧视觉结果。 */
uint8_t VisionTrack_Start(uint8_t color);
/* 取消跟踪并请求视觉端停止；调用后立即停止底盘由调试任务负责。 */
void VisionTrack_Stop(void);
/* 每20ms调用一次：发送待处理请求，消费视觉响应，计算本周期速度。 */
void VisionTrack_Service(uint32_t now_tick, VisionTrackOutput *output);
uint8_t VisionTrack_IsActive(void);

#endif /* ILHC_VISION_TRACK_H */
