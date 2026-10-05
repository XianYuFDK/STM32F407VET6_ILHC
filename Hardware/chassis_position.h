#ifndef ILHC_CHASSIS_POSITION_H
#define ILHC_CHASSIS_POSITION_H

#include <stdint.h>

/* 单点chassis_move与连续轨迹共用坐标闭环；不读OPS、不下发电机、不等待到位。
 * actual/target: OPS世界X/Y(mm)、航向(deg)，保留浮点精度。
 * error: 世界X/Y误差(mm)、最短航向误差(deg)。
 * body_xy=0: X/Y增益与输出使用世界轴；=1: 转车体轴后应用X/Y增益。
 * command = feedforward + gain * error；feedforward可为NULL。
 * X/Y输出mm/s。航向单位由gain[2]定义：轨迹为deg/s，旧chassis_move为
 * 混轮前等效线速度mm/s；原KPZ与轨迹TKPZ数值不可直接互换。
 * 非有限输入/输出返回0并清零；调用者负责限速、斜坡、STOP及安全门禁。
 * 各数组必须独立且非NULL（仅feedforward可以NULL）。
 */
uint8_t chassis_move_reference(const float actual[3], const float target[3],
                             const float gain[3], const float feedforward[3],
                             uint8_t body_xy, float error[3], float command[3]);

#endif
