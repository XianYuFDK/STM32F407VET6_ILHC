/**
 ******************************************************************************
 * @file    mecanum_pid.h
 * @brief   麦克纳姆轮底盘通用 PID 控制器
 *          用于底盘速度/位置闭环，支持在线调参
 ******************************************************************************
 */
#ifndef __MECANUM_PID_H__
#define __MECANUM_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* --------------------------- PID 结构体 --------------------------- */
typedef struct
{
  float   kp;          /* 比例系数 */
  float   ki;          /* 积分系数 */
  float   kd;          /* 微分系数 */
  float   setpoint;    /* 目标值   */
  float   integral;    /* 积分累加 */
  float   last_error;  /* 上次误差 */
  float   output_min;  /* 输出下限 */
  float   output_max;  /* 输出上限 */
  uint8_t first_run;   /* 首次运行标志 */
} MecanumPid_t;

/* --------------------------- 对外接口 ----------------------------- */

/**
 * @brief  初始化 PID 参数和输出限幅
 * @param  pid        PID 结构体
 * @param  kp        比例系数
 * @param  ki        积分系数
 * @param  kd        微分系数
 * @param  output_min 输出下限
 * @param  output_max 输出上限
 */
void MecanumPid_Init(MecanumPid_t *pid, float kp, float ki, float kd,
                     float output_min, float output_max);

/**
 * @brief  设置 PID 目标值
 */
void MecanumPid_SetTarget(MecanumPid_t *pid, float setpoint);

/**
 * @brief  在线修改 PID 参数
 */
void MecanumPid_SetParams(MecanumPid_t *pid, float kp, float ki, float kd);

/**
 * @brief  设置输出限幅并约束积分项
 */
void MecanumPid_SetOutputLimit(MecanumPid_t *pid, float output_min, float output_max);

/**
 * @brief  清空积分项和历史误差，重新开始控制
 */
void MecanumPid_Reset(MecanumPid_t *pid);

/**
 * @brief  执行一次 PID 计算
 * @param  pid      PID 结构体
 * @param  feedback 反馈值
 * @param  dt       距上次计算的时间，单位秒
 * @retval 当前 PID 输出
 */
float MecanumPid_Update(MecanumPid_t *pid, float feedback, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __MECANUM_PID_H__ */
