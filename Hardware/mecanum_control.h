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

/* 车体坐标系速度（**内部**定义，实车标定）
 * 注意：内部 +x 指向**车尾**、+y 与**车左**同向、+z 为逆时针；
 * 对外协议(上位机/Qt/比赛)的 +X=车左、+Y=车头、+Z=逆时针 由
 * debug_usart.c 的 Debug_UserToInternal()/Debug_InternalToUser() 转换，
 * 不要直接拿这里的符号对外使用（早期注释写"x 正数前进"与实车相反，已更正）。 */
typedef struct
{
  float x;     /* 内部前后轴：正数 → 车尾方向（与对外 +Y(车头) 反号），单位 RPM */
  float y;     /* 内部左右轴：正数 → 车左方向（与对外 +X 同号），单位 RPM */
  float z;     /* 旋转速度：正数 → 逆时针(自顶向下看)，与对外一致，单位 RPM */
} MecanumSpeed_t;

/* 全局位姿（**内部**定义：x=前后(正指向车尾)、y=左右(正=车左)、yaw=deg 逆时针正）。
 * 对外遥测 ch0/ch1 与 GOTO/OPSOFFSET 入参都经适配层转换，见 debug_usart.c。 */
typedef struct
{
  float x;     /* 内部前后坐标，单位 mm */
  float y;     /* 内部左右坐标，单位 mm */
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
