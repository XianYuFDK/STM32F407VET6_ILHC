/**
 ******************************************************************************
 * @file    mecanum_control.h
 * @brief   麦克纳姆轮底盘移动控制（硬件层）
 *
 *          参考 Logistics_Vehicle_F407_V2.7.4 开源电控底盘控制，调整如下：
 *          - UART4 速度模式控制 4 个 ZDT_X42S 电机
 *          - SpeedTarget[4] 保存四轮目标速度
 *          - chassis_move() 使用 OPS 全局定位反馈做位置环
 *          - SetMotorVoltageAndDirection() 负责向 UART4 非阻塞发送队列提交指令
 *
 *          统一坐标约定（内外一致）：
 *          - pos_x：左右坐标，+ 为车左
 *          - pos_y：前后坐标，+ 为车头
 *          - zangle：航向角，+ 为逆时针
 *          - chassis_move(x, y, z)：目标 X=左右、Y=前后，单位 mm
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

/* OPS 当前全局坐标（由 chassis_move 内部从 OPS 驱动刷新）
 * pos_x: +X=车左；pos_y: +Y=车头；单位 mm */
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

/* 车体坐标系速度（内部与对外同约定）：
 * +x 指向车左、+y 指向车头、+z 为逆时针。 */
typedef struct
{
  float x;     /* 左右速度：正数 → 车左，单位 RPM */
  float y;     /* 前后速度：正数 → 车头，单位 RPM */
  float z;     /* 旋转速度：正数 → 逆时针，单位 RPM */
} MecanumSpeed_t;

/* 全局位姿（内部与对外同约定：x=左右(+左)、y=前后(+前)、yaw=deg 逆时针正）。 */
typedef struct
{
  float x;     /* X=左右坐标，+车左，单位 mm */
  float y;     /* Y=前后坐标，+车头，单位 mm */
  float yaw;   /* 航向角，单位 deg，逆时针为正，范围 [-180,180] */
} MecanumPose_t;

/* --------------------------- 对外接口 ----------------------------- */

/* 与参考工程一致的接口 */
void SpeedTarget_stop(void);
void SetMotorVoltageAndDirection(int MotorSpeed1, int MotorSpeed2, int MotorSpeed3, int MotorSpeed4);
void Mecanum_NormalizeWheelSpeed(int speed[4], int limit);
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
