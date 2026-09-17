/**
 ******************************************************************************
 * @file    mecanum_control.c
 * @brief   麦克纳姆轮底盘移动控制（硬件层）
 *
 * 
 *
 *          当前工程控制任务每 20ms：
 *            chassis_move(X_target, Y_target, Z_target);
 *            SetMotorVoltageAndDirection(SpeedTarget[0..3]);
 *
 *          电机：ZDT_X42S Emm 速度模式，UART4，地址 1~4
 *          反馈：OPS 全局定位 OPS_GetPosition()
 *          控制：P 比例控制 + 数值限幅 + 速度斜坡 + 到位判断
 *
 *          统一坐标约定（内部、外部相同）：
 *            +X = 车左、-X = 车右
 *            +Y = 车头、-Y = 车尾
 *            +Z = 自顶向下逆时针
 ******************************************************************************
 */
#include "mecanum_control.h"
#include "zdt_x42s.h"
#include "ops.h"

#include <math.h>
#include <stdlib.h>

/* --------------------------- 速度默认参数 -------------------------- */
#define MECANUM_XYV_MAX_DEFAULT   1600.0f     // 单位：mm/s (X/Y轴)
#define MECANUM_ZV_MAX_DEFAULT    750.0f      // 单位：mm/s (Z轴)
#define MECANUM_XYV_MIN_DEFAULT   5.0f        // 单位：mm/s (X/Y轴)
#define MECANUM_ZV_MIN_DEFAULT    5.0f        // 单位：mm/s (Z轴) 
#define MECANUM_OPS_TIMEOUT_MS    200U        // OPS 数据超过该时间未更新则停车
#define MECANUM_RAD_TO_DEG        57.2957795f

/* --------------------------- 参考工程参数 -------------------------- */


/* 底盘定位 move Kp */
float mKpx = 2.3f;
float mKpy = 2.3f;
float mKpz = 9.0f;

/* 视觉微调 Kp（保留接口） */
float vKpx = 1.2f;
float vKpy = 1.2f;
float vKpz = 2.4f;
float cvKpz = 0.08f;

/* 限幅值 */
float XYVmax = 0.0f;
float ZVmax  = 0.0f;
float XYVmin = 0.0f;
float ZVmin  = 0.0f;

/* 四轮目标速度，供输出任务使用 */
int SpeedTarget[4] = {0, 0, 0, 0};

/* OPS 当前全局坐标：X=左右(+车左)、Y=前后(+车头)，单位 mm */
float pos_x = 0.0f;
float pos_y = 0.0f;
float zangle = 0.0f;

/* 上一层轮速，用于斜坡限制 */
int last_Speed[4] = {0, 0, 0, 0};

/* 到位状态 */
uint8_t in_pos     = 0;
uint8_t near_pos   = 0;
uint8_t delay_pos  = 0;

/* 当前误差，便于调试 */
float devx = 0.0f;
float devy = 0.0f;
float devz = 0.0f;

/* --------------------------- 电机命令 ------------------------------ */

/**
 * @brief  从 OPS 刷新位姿，并统一转换为 mm / deg
 * @note   OPS9 原始输出为 m / rad，航向角可能连续累计
 */
static void MecanumControl_UpdatePose(void)
{
  float raw_x;
  float raw_y;
  float raw_yaw;
  int32_t whole_turns;

  (void)OPS_GetPosition(&raw_x, &raw_y, &raw_yaw);
  pos_x = raw_x * 1000.0f;
  pos_y = raw_y * 1000.0f;
  zangle = raw_yaw * MECANUM_RAD_TO_DEG;
  whole_turns = (int32_t)(zangle / 360.0f);
  zangle -= (float)whole_turns * 360.0f;
  if (zangle > 180.0f)
  {
    zangle -= 360.0f;
  }
  else if (zangle < -180.0f)
  {
    zangle += 360.0f;
  }
}

/**
 * @brief  四轮目标速度清零
 */
void SpeedTarget_stop(void)
{
  SpeedTarget[0] = 0;
  SpeedTarget[1] = 0;
  SpeedTarget[2] = 0;
  SpeedTarget[3] = 0;
}

/**
 * @brief  四轮转速整体同比限幅（麦克纳姆必须整体缩放，禁止单轮硬裁剪）
 * @param  speed 四轮目标速度数组，单位 RPM，正负表示方向；就地修改
 * @param  limit 单轮允许的最大绝对值，单位 RPM
 *
 * @note   为什么不能对每轮单独裁剪：斜行、旋转或两者叠加时四轮目标值不相等，
 *         若各自硬截断到上限，四轮比例被破坏，合成的运动方向会偏离期望
 *         （典型表现：高速斜行时车头被"拽"向某一侧、原地旋转叠加平移时打滑）。
 *         正确做法是按 max|speed| 求一个公共系数 scale，四轮同乘，比例与方向不变。
 *         全程用 float 运算：整数除法会把 scale 截断成 0，导致"限幅后反而停车"。
 *
 *         量级说明：车体两轴分量在 numerical_limit 里是被"分别"限幅的
 *         （vx1/vx2/vy1/vy2 各自 ≤ XYVmax），因此 VX=vx1+vx2、VY=vy1-vy2
 *         最大可到 2*XYVmax，四轮最大约 (4*XYVmax + ZVmax)*0.238 ≈ 1701 RPM
 *         （默认 1600/750），超过 ZDT 单轮上限时就必须整体缩放。
 */
void Mecanum_NormalizeWheelSpeed(int speed[4], int limit)
{
  float max_abs = 0.0f;
  float scale;
  uint8_t i;

  if ((speed == NULL) || (limit <= 0))
  {
    return;
  }

  for (i = 0U; i < 4U; ++i)
  {
    float mag = (speed[i] < 0) ? -(float)speed[i] : (float)speed[i];
    if (mag > max_abs)
    {
      max_abs = mag;
    }
  }

  if (max_abs <= (float)limit)
  {
    return;                       /* 未超限：原样下发，不做任何缩放 */
  }

  scale = (float)limit / max_abs; /* float 除法，不会被截成 0 */
  for (i = 0U; i < 4U; ++i)
  {
    speed[i] = (int)((float)speed[i] * scale);
  }
}

/**
 * @brief  向四个 ZDT_X42S 电机下发速度命令
 * @param  MotorSpeed1~4 四轮目标速度，正负表示方向
 */
void SetMotorVoltageAndDirection(int MotorSpeed1, int MotorSpeed2,
                                 int MotorSpeed3, int MotorSpeed4)
{
  int motor_speed[4];
  /* 用户确认：俯视车头朝上，左前1、右前2、左后3、右后4。 */
  uint8_t motor_addr[4] = {1U, 2U, 3U, 4U};
  uint8_t i;

  motor_speed[0] = MotorSpeed1;
  motor_speed[1] = MotorSpeed2;
  motor_speed[2] = MotorSpeed3;
  motor_speed[3] = MotorSpeed4;

  /* 下发前做一次整体同比限幅：超出 ZDT 单轮上限时四轮同乘一个系数，
     绝不能让驱动层(ZDT_X42S_SpeedAcc)对单轮硬裁剪——那会破坏四轮比例。 */
  Mecanum_NormalizeWheelSpeed(motor_speed, (int)ZDT_X42S_MAX_RPM);

  for (i = 0U; i < 4U; ++i)
  {
    uint8_t dir = ZDT_X42S_DIR_CW;
    uint16_t rpm = 0U;

    if (motor_speed[i] < 0)
    {
      dir = ZDT_X42S_DIR_CCW;
      rpm = (uint16_t)(-motor_speed[i]);
    }
    else
    {
      rpm = (uint16_t)motor_speed[i];
    }

    /* Emm 速度模式：地址 + 0xF6 + 方向 + 速度 + 加速度0 + 同步 + 0x6B */
    ZDT_X42S_SpeedAcc(motor_addr[i], dir, rpm, 0U);

    /* 每条命令间隔 1ms，避免粘包 */
    HAL_Delay(1U);
  }
}

/**
 * @brief  按统一车体坐标计算四轮逻辑速度
 * @param  x_left  左右速度，+ 为车左
 * @param  y_forward 前后速度，+ 为车头
 * @param  z_ccw   航向速度，+ 为逆时针
 * @param  speed   输出四轮速度，顺序为左前、右前、左后、右后
 */
static void MecanumControl_CalcWheelSpeed(float x_left, float y_forward,
                                          float z_ccw, int speed[4])
{
  speed[0] = (int)( y_forward - x_left - z_ccw);  /* 左前 */
  speed[1] = (int)(-y_forward - x_left - z_ccw);  /* 右前 */
  speed[2] = (int)( y_forward + x_left - z_ccw);  /* 左后 */
  speed[3] = (int)(-y_forward + x_left - z_ccw);  /* 右后 */
}

/* --------------------------- 数值限幅 ------------------------------ */

/**
 * @brief  数值限幅：死区 + 最小速度 + 最大速度
 * @param  value    输入输出值
 * @param  max      最大值
 * @param  min      最小补偿速度
 * @param  dead_zone 死区
 */
void numerical_limit(float *value, float max, float min, float dead_zone)
{
  if (value == NULL)
  {
    return;
  }

  if (*value > dead_zone)
  {
    *value += min;
  }
  else if (*value < -dead_zone)
  {
    *value -= min;
  }

  if (*value > max)
  {
    *value = max;
  }
  else if (*value < -max)
  {
    *value = -max;
  }
}

/* ------------------------- OPS 位置闭环 ---------------------------- */

/**
 * @brief  底盘 OPS 全局定位移动（P 控制，参考开源底盘）
 * @param  x 目标全局 X，左右轴，+车左，单位 mm
 * @param  y 目标全局 Y，前后轴，+车头，单位 mm
 * @param  z 目标航向角，单位 deg
 * @note   调用本函数后还需周期调用 SetMotorVoltageAndDirection() 下发
 */
void chassis_move(int x, int y, int z)
{
  int speed[4] = {0, 0, 0, 0};
  float cmd_x = 0.0f;
  float cmd_y = 0.0f;
  float vz  = 0.0f;
  uint8_t i;


  /* 刷新 OPS 当前坐标 */
  MecanumControl_UpdatePose();

  /* 目标 - 当前：X 为左右、Y 为前后，统一坐标下直接构成负反馈。 */
  devx = (float)x - pos_x;
  devy = (float)y - pos_y;

  /* 最短航向误差，统一到 [-180, 180]
   * 必须用 O(1) 写法：目标航向来自串口（GOTO 的 z），若被构造成 Inf
   * （例如 GOTO=0,0,<40 位数字> 溢出成 +Inf），"while (devz > 180) devz -= 360"
   * 会永不退出，20ms 任务连同遥测/命令处理一起永久挂死。
   * 这里用 fmodf 归约到 (-360,360) 再补一次修正；同时对非有限值直接判为 0。 */
  devz = (float)z - zangle;
  if (!isfinite(devz))
  {
    devz = 0.0f;
  }
  devz = fmodf(devz, 360.0f);
  if (devz > 180.0f)       { devz -= 360.0f; }
  else if (devz < -180.0f) { devz += 360.0f; }

  /* 世界坐标误差 → 车体坐标，再分别应用轴 P 增益：
   *   X_body_error =  cosθ*devx + sinθ*devy
   *   Y_body_error = -sinθ*devx + cosθ*devy
   *   cmd_x = mKpx * X_body_error
   *   cmd_y = mKpy * Y_body_error
   * P 必须在旋转之后应用，否则航向不为 0 时 X/Y 增益会互相串轴。 */
  {
    float c = cosf(zangle * 3.1415926f / 180.0f);
    float s = sinf(zangle * 3.1415926f / 180.0f);

    cmd_x = mKpx * ( c * devx + s * devy);
    cmd_y = mKpy * (-s * devx + c * devy);
  }
  numerical_limit(&cmd_x, XYVmax, XYVmin, 5.0f);
  numerical_limit(&cmd_y, XYVmax, XYVmin, 5.0f);

  /* 航向环不参与旋转：devz 已在上面 wrap 到 [-180,180]，直接 P 控制。 */
  vz = mKpz * devz;
  numerical_limit(&vz, ZVmax, 0.0f, 5.0f);

  MecanumControl_CalcWheelSpeed(cmd_x, cmd_y, vz, speed);

  /* 速度斜坡限制 */
  for (i = 0U; i < 4U; ++i)
  {
    if ((speed[i] > 0) && (speed[i] > last_Speed[i]))
    {
      speed[i] = last_Speed[i] + 20;
    }
    else if ((speed[i] < 0) && (speed[i] < last_Speed[i]))
    {
      speed[i] = last_Speed[i] - 20;
    }

    SpeedTarget[i] = (int)(speed[i] * 0.238f);
    last_Speed[i]  = speed[i];
  }

  /* 到位判断 */
  if ((devx < 100.0f) && (devx > -100.0f) &&
      (devy < 100.0f) && (devy > -100.0f) &&
      (devz < 30.0f) && (devz > -30.0f))
  {
    near_pos = 1U;
  }
  else
  {
    near_pos = 0U;
  }

  if ((devx < 60.0f) && (devx > -60.0f) &&
      (devy < 60.0f) && (devy > -60.0f) &&
      (devz < 15.0f) && (devz > -15.0f))
  {
    if (delay_pos < 255U)
    {
      ++delay_pos;
    }
  }
  else
  {
    delay_pos = 0U;
  }

  if (delay_pos > 10U)
  {
    in_pos = 1U;
  }
  else
  {
    in_pos = 0U;
  }
}

/**
 * @brief  底盘原地转动
 * @param  z 目标航向角，单位 deg
 */
void chassis_turn(int z)
{
  chassis_move((int)pos_x, (int)pos_y, z);
}

/* --------------------------- 封装接口 ------------------------------ */

/**
 * @brief  初始化底盘控制器
 */
void MecanumControl_Init(void)
{
  SpeedTarget_stop();

  /* 速度限幅默认值，可通过 USART1 调试命令在线修改 */
  XYVmax = MECANUM_XYV_MAX_DEFAULT;
  ZVmax  = MECANUM_ZV_MAX_DEFAULT;
  XYVmin = MECANUM_XYV_MIN_DEFAULT;
  ZVmin  = MECANUM_ZV_MIN_DEFAULT;


  pos_x     = 0.0f;
  pos_y     = 0.0f;
  zangle    = 0.0f;
  last_Speed[0] = 0;
  last_Speed[1] = 0;
  last_Speed[2] = 0;
  last_Speed[3] = 0;

  in_pos    = 0U;
  near_pos  = 0U;
  delay_pos = 0U;

  devx = 0.0f;
  devy = 0.0f;
  devz = 0.0f;
}

/**
 * @brief  使能四个电机
 */
void MecanumControl_Enable(void)
{
  ZDT_X42S_Enable(1U);
  ZDT_X42S_Enable(2U);
  ZDT_X42S_Enable(3U);
  ZDT_X42S_Enable(4U);

  HAL_Delay(100U);
}

/**
 * @brief  失能四个电机（释放锁轴）
 * @note   只发送驱动器失能帧，不修改SpeedTarget，调用方应先停车再失能。
 *         失能后轮子不再保持位置，重新运动前必须MecanumControl_Enable。
 *         这里不等同于MecanumControl_Enable的100ms等待：失能后不应再发
 *         速度帧，无需等待驱动器进入可接收速度命令的状态。
 */
void MecanumControl_Disable(void)
{
  ZDT_X42S_Disable(1U);
  ZDT_X42S_Disable(2U);
  ZDT_X42S_Disable(3U);
  ZDT_X42S_Disable(4U);
}

/**
 * @brief  只清零底盘运动状态，不向电机下发任何速度帧
 * @note   用于四轮已失能的场合。ZDT_X42S 在速度模式下收到任意速度命令都会
 *         重新使能并锁轴，失能后再下发速度帧会把刚才的失能帧覆盖掉，表现为
 *         "失能了还是锁"，因此失能后只能清理软件目标。
 */
void MecanumControl_ClearTarget(void)
{
  SpeedTarget_stop();

  last_Speed[0] = 0;
  last_Speed[1] = 0;
  last_Speed[2] = 0;
  last_Speed[3] = 0;

  in_pos    = 0U;
  near_pos  = 0U;
  delay_pos = 0U;
}

/**
 * @brief  停止底盘并清零目标
 * @note   会向四轮下发速度0帧。四轮已失能时改用 MecanumControl_ClearTarget，
 *         否则速度0帧会重新使能电机并把轮子重新锁住。
 */
void MecanumControl_Stop(void)
{
  MecanumControl_ClearTarget();
  SetMotorVoltageAndDirection(0, 0, 0, 0);
}

/**
 * @brief  车体坐标系直接速度移动（MANUAL 手动、ZDT 单轮测试走这条路）
 * @param  vxRpm 左右速度，RPM：正数 → 车左
 * @param  vyRpm 前后速度，RPM：正数 → 车头
 * @param  vzRpm 旋转速度，RPM：正数 → 逆时针
 * @note   形参与对外 MANUAL=X(车左),Y(车头),W(逆时针) 完全同序、同符号，
 *         本函数不做坐标交换。四轮结果在 SetMotorVoltageAndDirection 里
 *         做整体同比限幅后下发。
 */
void MecanumControl_MoveVelocity(float vxRpm, float vyRpm, float vzRpm)
{
  int wheel[4];

  MecanumControl_CalcWheelSpeed(vxRpm, vyRpm, vzRpm, wheel);

  SetMotorVoltageAndDirection(wheel[0], wheel[1], wheel[2], wheel[3]);
}

/**
 * @brief  基于 OPS 全局定位执行一次 GOTO 控制
 * @return 1 到位，0 未到位
 */
uint8_t MecanumControl_GotoOPS(float targetX, float targetY, float targetYaw, float maxRpm)
{
  /* maxRpm>0 时按 RPM 换算最大控制量；传 0 则使用全局限幅（可由调试命令修改） */
  if (maxRpm > 0.0f)
  {
    XYVmax = maxRpm / 0.238f;
    ZVmax  = XYVmax * (750.0f / 1600.0f);
  }

  /* 无有效 OPS 数据时禁止移动 */
  if (OPS_IsOnline(MECANUM_OPS_TIMEOUT_MS) == 0U)
  {
    MecanumControl_Stop();
    return 0U;
  }

  chassis_move((int)targetX, (int)targetY, (int)targetYaw);
  SetMotorVoltageAndDirection(SpeedTarget[0], SpeedTarget[1],
                              SpeedTarget[2], SpeedTarget[3]);

  return (in_pos != 0U) ? 1U : 0U;
}

/**
 * @brief  GotoOPS 兼容别名
 */
uint8_t MecanumControl_MoveTo(float targetX, float targetY, float targetYaw, float maxRpm)
{
  return MecanumControl_GotoOPS(targetX, targetY, targetYaw, maxRpm);
}

/**
 * @brief  读取 OPS 当前位姿
 */
void MecanumControl_GetPose(float *x, float *y, float *yaw)
{
  MecanumControl_UpdatePose();

  if (x != NULL)   { *x = pos_x; }
  if (y != NULL)   { *y = pos_y; }
  if (yaw != NULL) { *yaw = zangle; }
}

/**
 * @brief  设置 X/Y 轴 P 控制比例系数
 */
void MecanumControl_SetPid(float kp, float ki, float kd)
{
  (void)ki;
  (void)kd;
  mKpx = kp;
  mKpy = kp;
}

/**
 * @brief  设置 Z 轴 P 控制比例系数
 */
void MecanumControl_SetYawPid(float kp, float ki, float kd)
{
  (void)ki;
  (void)kd;
  mKpz = kp;
}

/**
 * @brief  查询是否进入较大目标窗口
 */
uint8_t MecanumControl_IsNearTarget(void)
{
  return near_pos;
}

/**
 * @brief  查询是否进入最终目标窗口并保持足够次数
 */
uint8_t MecanumControl_IsInTarget(void)
{
  return in_pos;
}
