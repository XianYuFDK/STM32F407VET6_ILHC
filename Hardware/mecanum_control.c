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
static uint32_t s_control_dt_ms = 20U;
static uint32_t s_settle_ms;
/* 20ms坐标速度输出保留不足1RPM的余量，避免末端P输出被永久截成0。 */
static float s_world_rpm_remainder[4];

static void MecanumControl_ResetWorldRpm(void)
{
  uint8_t i;
  for (i = 0U; i < 4U; ++i) s_world_rpm_remainder[i] = 0.0f;
}

/* 长时间失调度不累计到位时间，斜坡最多采用100ms，避免恢复时突跳。 */
void MecanumControl_SetPeriod(uint32_t dt_ms)
{
  if (dt_ms > 100U) s_settle_ms = 0U;
  s_control_dt_ms = (dt_ms > 100U) ? 100U : dt_ms;
}

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
  MecanumControl_ResetWorldRpm();
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
 *         量级说明：位置环先完成世界坐标到车体坐标旋转，再分别对最终的
 *         cmd_x/cmd_y 各限幅一次，因此 |cmd_x|、|cmd_y| ≤ XYVmax，
 *         四轮最大约 (2*XYVmax + ZVmax)*0.238 ≈ 940 RPM（默认 1600/750）。
 *         四轮结果仍由本函数做整体同比缩放，绝不对单轮硬裁剪。
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

    /* 仅入队；驱动层用 UART4 中断串行发送，并在两帧之间留出空闲时间。
     * 本函数不等待传输完成，发送失败由调试层的故障闸门处理。 */
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

uint8_t chassis_move_reference(const float actual[3], const float target[3],
                             const float gain[3], const float feedforward[3],
                             uint8_t body_xy, float error[3], float command[3])
{
  uint8_t i;
  float dx,dy,angle;
  for(i=0U;i<3U;++i) { error[i]=0.0f;command[i]=0.0f; }
  for(i=0U;i<3U;++i) {
    if(!isfinite(actual[i]) || !isfinite(target[i]) || !isfinite(gain[i]) ||
       (feedforward && !isfinite(feedforward[i]))) return 0U;
  }
  error[0]=target[0]-actual[0];error[1]=target[1]-actual[1];
  /* O(1)最短航向差，179/-179对应2度转头。 */
  angle=fmodf(target[2]-actual[2],360.0f);
  if(angle>180.0f) angle-=360.0f;
  else if(angle< -180.0f) angle+=360.0f;
  error[2]=angle;dx=error[0];dy=error[1];
  if(body_xy) {
    float c=cosf(actual[2]*3.1415926f/180.0f);
    float s=sinf(actual[2]*3.1415926f/180.0f);
    dx=c*error[0]-s*error[1];dy=s*error[0]+c*error[1];
  }
  command[0]=gain[0]*dx;command[1]=gain[1]*dy;command[2]=gain[2]*angle;
  for(i=0U;i<3U;++i) {
    if(feedforward) command[i]+=feedforward[i];
  }
  for(i=0U;i<3U;++i) {
    if(!isfinite(error[i]) || !isfinite(command[i])) {
      uint8_t j;
      for(j=0U;j<3U;++j) { error[j]=0.0f;command[j]=0.0f; }
      return 0U;
    }
  }
  return 1U;
}

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
  float actual[3],target[3],gain[3],error[3],command[3];
  uint8_t i;


  /* 刷新 OPS 当前坐标 */
  MecanumControl_UpdatePose();

  /* 世界参考坐标与OPS实际坐标进入公共位置闭环，旧模式仍先转车体轴。 */
  actual[0]=pos_x;actual[1]=pos_y;actual[2]=zangle;
  target[0]=(float)x;target[1]=(float)y;target[2]=(float)z;
  gain[0]=mKpx;gain[1]=mKpy;gain[2]=mKpz;
  if(!chassis_move_reference(actual,target,gain,NULL,1U,error,command)) {
    SpeedTarget_stop();devx=devy=devz=0.0f;
    near_pos=in_pos=delay_pos=0U;s_settle_ms=0U;return;
  }
  devx=error[0];devy=error[1];devz=error[2];
  cmd_x=command[0];cmd_y=command[1];vz=command[2];
  numerical_limit(&cmd_x, XYVmax, XYVmin, 5.0f);
  numerical_limit(&cmd_y, XYVmax, XYVmin, 5.0f);

  /* 航向环不参与旋转：devz 已在上面 wrap 到 [-180,180]，直接 P 控制。 */
  numerical_limit(&vz, ZVmax, ZVmin, 5.0f);

  MecanumControl_CalcWheelSpeed(cmd_x, cmd_y, vz, speed);

  /* 速度斜坡限制：正反向、升速和降速按实际周期限制变化量，避免
   * “同方向降速”绕过斜坡造成突变。 */
  for (i = 0U; i < 4U; ++i)
  {
    int delta = speed[i] - last_Speed[i];

    if (delta > (int)s_control_dt_ms)
    {
      delta = (int)s_control_dt_ms;
    }
    else if (delta < -(int)s_control_dt_ms)
    {
      delta = -(int)s_control_dt_ms;
    }
    speed[i] = last_Speed[i] + delta;

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

  if ((devx < 5.0f) && (devx > -5.0f) &&
      (devy < 5.0f) && (devy > -5.0f) &&
      (devz < 0.5f) && (devz > -0.5f))
  {
    if (s_settle_ms < 220U) s_settle_ms += s_control_dt_ms;
    if (delay_pos < 255U)
    {
      ++delay_pos;
    }
  }
  else
  {
    delay_pos = 0U;
    s_settle_ms = 0U;
  }

  if (s_settle_ms >= 220U)
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
  s_settle_ms = 0U;

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

  /* 运行期由上层状态机等待100ms，不阻塞控制服务。 */
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
  MecanumControl_ResetWorldRpm();
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
  s_settle_ms = 0U;
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
  uint8_t i;

  MecanumControl_ResetWorldRpm();
  MecanumControl_CalcWheelSpeed(vxRpm, vyRpm, vzRpm, wheel);

  /* MANUAL 不走 chassis_move() 的斜坡分支，必须把实际手动轮速同步到
   * last_Speed，保证下一次 GOTO 的斜坡从当前运动状态继续，而不是从
   * 上一次 GOTO 遗留的旧速度继续。0.238 是 mm/s 到 RPM 的换算系数，
   * 与 chassis_move() 中 SpeedTarget 的换算保持一致。 */
  for (i = 0U; i < 4U; ++i)
  {
    last_Speed[i] = (int)((float)wheel[i] / 0.238f);
  }

  SetMotorVoltageAndDirection(wheel[0], wheel[1], wheel[2], wheel[3]);
}

void MecanumControl_MoveWorldVelocity(float vx,float vy,float omega,float yaw)
{
  float c,s,x,y,z,rpm[4],peak=0.0f,scale,total;
  int wheel[4];
  uint8_t i;
  if(!isfinite(vx) || !isfinite(vy) || !isfinite(omega) || !isfinite(yaw)) { MecanumControl_Stop();return; }
  c=cosf(yaw*0.0174532925f);s=sinf(yaw*0.0174532925f);
  /* 与chassis_move保持同一世界→车体旋转，不交换左右/前后轴。 */
  x=(c*vx-s*vy)*0.238f;y=(s*vx+c*vy)*0.238f;
  z=omega*0.0174532925f*MECANUM_ROTATION_LEVER_MM*0.238f;
  rpm[0]=y-x-z;rpm[1]=-y-x-z;rpm[2]=y+x-z;rpm[3]=-y+x-z;
  for(i=0U;i<4U;++i) {
    if(!isfinite(rpm[i])) { MecanumControl_Stop();return; }
    if(fabsf(rpm[i])>peak) peak=fabsf(rpm[i]);
  }
  /* 先同比限幅浮点轮速，再量化；误差余量绝不能越过电机上限。 */
  scale=peak>(float)ZDT_X42S_MAX_RPM?(float)ZDT_X42S_MAX_RPM/peak:1.0f;
  for(i=0U;i<4U;++i) {
    rpm[i]*=scale;
    /* 零命令/反向立即丢弃旧余量，停车或接管后不会吐出残留脉冲。 */
    if(rpm[i]==0.0f || fabsf(rpm[i])>=(float)ZDT_X42S_MAX_RPM ||
       rpm[i]*s_world_rpm_remainder[i]<0.0f) s_world_rpm_remainder[i]=0.0f;
    total=rpm[i]+s_world_rpm_remainder[i];
    wheel[i]=(int)total;
    s_world_rpm_remainder[i]=total-(float)wheel[i];
    last_Speed[i]=(int)((float)wheel[i]/0.238f);
  }
  SetMotorVoltageAndDirection(wheel[0],wheel[1],wheel[2],wheel[3]);
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
