/**
 ******************************************************************************
 * @file    mecanum_control.h
 * @brief   麦克纳姆轮底盘移动控制（硬件层）
 *
 *          参考 Logistics_Vehicle_F407_V2.7.4 开源电控底盘控制，调整如下：
 *          - UART4 速度模式控制 4 个 ZDT_X42S 电机
 *          - SpeedTarget[4] 保存四轮目标速度
 *          - chassis_move() 使用 OPS 全局定位反馈做位置环
 *          - SetMotorVoltageAndDirection() 负责实际下发 UART4 指令
 *
 *          坐标约定：
 *          - pos_x / pos_y：OPS 全局坐标，单位 mm
 *          - zangle：OPS 航向角，单位 deg
 *          - chassis_move(x, y, z)：目标全局坐标/航向角
 ******************************************************************************
 */
#ifndef __MECANUM_CONTROL_H__
#define __MECANUM_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ------------------------- 参考代码全局变量 ------------------------ */

/* 四轮目标速度，供底盘任务/输出任务使用 */
extern int SpeedTarget[4];

/* OPS 当前全局坐标（由 chassis_move 内部从 OPS 驱动刷新） */
extern float pos_x;
extern float pos_y;
extern float zangle;
/* 可在线调节的控制参数 */
extern float mKpx;
extern float mKpy;
extern float mKpz;
extern float XYVmax;
extern float ZVmax;
extern float XYVmin;
extern float ZVmin;
extern float devx;
extern float devy;
extern float devz;


/* ---------------------------- 数据类型 ---------------------------- */

/* 车体坐标系速度 */
typedef struct
{
  float x;     /* 前后速度：正数前进，单位 RPM */
  float y;     /* 左右速度：正数左移，单位 RPM */
  float z;     /* 旋转速度：正数逆时针，单位 RPM */
} MecanumSpeed_t;

/* 全局位姿 */
typedef struct
{
  float x;     /* 全局 X，单位 mm */
  float y;     /* 全局 Y，单位 mm */
  float yaw;   /* 航向角，单位 deg */
} MecanumPose_t;

/* --------------------------- 对外接口 ----------------------------- */

/* 与参考工程一致的接口 */
void SpeedTarget_stop(void);
void SetMotorVoltageAndDirection(int MotorSpeed1, int MotorSpeed2, int MotorSpeed3, int MotorSpeed4);
void numerical_limit(float *value, float max, float min, float dead_zone);
void chassis_move(int x, int y, int z);
void chassis_turn(int z);

/* 封装接口 */
void MecanumControl_Init(void);
void MecanumControl_Enable(void);
void MecanumControl_Disable(void);
void MecanumControl_Stop(void);
void MecanumControl_ClearTarget(void);
void MecanumControl_MoveVelocity(float vxRpm, float vyRpm, float vzRpm);
uint8_t MecanumControl_GotoOPS(float targetX, float targetY, float targetYaw, float maxRpm);
uint8_t MecanumControl_MoveTo(float targetX, float targetY, float targetYaw, float maxRpm);
void MecanumControl_GetPose(float *x, float *y, float *yaw);
void MecanumControl_SetPid(float kp, float ki, float kd);
void MecanumControl_SetYawPid(float kp, float ki, float kd);
uint8_t MecanumControl_IsNearTarget(void);
uint8_t MecanumControl_IsInTarget(void);

#ifdef __cplusplus
}
#endif

#endif /* __MECANUM_CONTROL_H__ */
