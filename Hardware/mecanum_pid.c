/**
 ******************************************************************************
 * @file    mecanum_pid.c
 * @brief   麦克纳姆轮底盘通用 PID 控制器
 *
 *          PID_Update() 约定：
 *            error    = setpoint - feedback
 *            integral = integral + error * dt * ki
 *            output   = kp*error + integral + kd*(error-last_error)/dt
 *            输出限幅在 output_min ~ output_max 之间
 ******************************************************************************
 */
#include "mecanum_pid.h"

/* --------------------------- 私有函数 ----------------------------- */

/**
 * @brief  数值限幅
 * @retval 限幅后的值
 */
static float MecanumPid_Clamp(float value, float min, float max)
{
  if (value > max)
  {
    return max;
  }

  if (value < min)
  {
    return min;
  }

  return value;
}

/* --------------------------- 对外接口 ----------------------------- */

/**
 * @brief  初始化 PID 参数和输出限幅
 */
void MecanumPid_Init(MecanumPid_t *pid, float kp, float ki, float kd,
                     float output_min, float output_max)
{
  if (pid == NULL)
  {
    return;
  }

  pid->kp          = kp;
  pid->ki          = ki;
  pid->kd          = kd;
  pid->setpoint    = 0.0f;
  pid->integral    = 0.0f;
  pid->last_error  = 0.0f;
  pid->output_min  = output_min;
  pid->output_max  = output_max;
  pid->first_run   = 1U;
}

/**
 * @brief  设置 PID 目标值
 */
void MecanumPid_SetTarget(MecanumPid_t *pid, float setpoint)
{
  if (pid == NULL)
  {
    return;
  }

  pid->setpoint = setpoint;
}

/**
 * @brief  在线修改 PID 参数
 */
void MecanumPid_SetParams(MecanumPid_t *pid, float kp, float ki, float kd)
{
  if (pid == NULL)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

/**
 * @brief  设置输出限幅并约束积分项
 */
void MecanumPid_SetOutputLimit(MecanumPid_t *pid, float output_min, float output_max)
{
  if (pid == NULL)
  {
    return;
  }

  pid->output_min = output_min;
  pid->output_max = output_max;
  pid->integral   = MecanumPid_Clamp(pid->integral, output_min, output_max);
}

/**
 * @brief  清空积分项和历史误差，重新开始控制
 */
void MecanumPid_Reset(MecanumPid_t *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->integral   = 0.0f;
  pid->last_error = 0.0f;
  pid->first_run  = 1U;
}

/**
 * @brief  执行一次 PID 计算
 */
float MecanumPid_Update(MecanumPid_t *pid, float feedback, float dt)
{
  float error;
  float derivative;
  float output;

  /* 空指针或无效时间直接返回 0，避免除零 */
  if ((pid == NULL) || (dt <= 0.0f))
  {
    return 0.0f;
  }

  error = pid->setpoint - feedback;

  /* 积分限幅，防止积分饱和 */
  pid->integral += error * dt * pid->ki;
  pid->integral  = MecanumPid_Clamp(pid->integral, pid->output_min, pid->output_max);

  /* 首次计算没有上一拍误差，微分项置 0 */
  if (pid->first_run != 0U)
  {
    derivative = 0.0f;
    pid->first_run = 0U;
  }
  else
  {
    derivative = (error - pid->last_error) / dt;
  }

  output = (pid->kp * error) + pid->integral + (pid->kd * derivative);
  output = MecanumPid_Clamp(output, pid->output_min, pid->output_max);

  pid->last_error = error;

  return output;
}
