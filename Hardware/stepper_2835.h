#ifndef ILHC_STEPPER_2835_H
#define ILHC_STEPPER_2835_H

#include "hcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 移植自物流车 V2.7.4 的 USER_Code/tower/tower.c，仅保留 28/35 驱动。 */
#define MOTOR35_CAN_ID       0x0300U  /* 35升降电机：CAN扩展帧基础ID */
#define MOTOR28_CAN_ID       0x0400U  /* 28伸缩电机：CAN扩展帧基础ID */
#define MOTOR35_DIR          0U       /* 35：从高处零点向下的协议方向 */
#define MOTOR28_DIR          0U       /* 28：从最小半径向外的协议方向 */
/* 以下是原车机械标定，单位为 0.1mm，换机构必须重新标定。 */
/* ------------------------- 35电机：Z轴升降标定 ------------------------- */
#define MOTOR35_HOME_HEIGHT  2030U    /* 原车回零高度203.0mm */
#define MOTOR35_MAX_TRAVEL   1600U    /* 相对回零高度向下最多160.0mm */
/* ------------------------- 28电机：悬臂伸缩标定 ----------------------- */
#define MOTOR28_HOME_RADIUS  1200U    /* 原车夹爪到转台中心的最小半径120.0mm */
#define MOTOR28_MAX_TRAVEL   1660U    /* 从最小半径额外伸出最多166.0mm */

typedef struct
{
    uint32_t count;           /* 收到回复的次数，不代表运动完成 */
    uint32_t last_tick;       /* 最近回复的 HAL 毫秒时间 */
    uint8_t length;
    uint8_t data[8];          /* 原始应答，未假定到位状态码 */
} Stepper2835Reply_t;

/* ======================== 公共协议接口（35 / 28 共用） ======================== */
/**
 * @brief 发送多圈回零命令。
 * @param id MOTOR35_CAN_ID 或 MOTOR28_CAN_ID。
 * @return HAL_OK 已提交；HAL_BUSY 邮箱忙；HAL_ERROR 参数/总线状态错误。
 * @note 依赖 CAN_Start 已成功，任务按需调用。会运动，不等待回零完成。
 *       回复不等于到位，机械限位和回零模式需按驱动器确认。
 */
HAL_StatusTypeDef Motor_Homing(uint16_t id);
/**
 * @brief 发送绝对位置指令，连续提交两个 CAN 扩展数据帧。
 * @param dir 0/1，仅表示原协议正反方向，不推定实体机构方向。
 * @param id MOTOR35_CAN_ID 或 MOTOR28_CAN_ID，第二帧使用 id+1。
 * @param step 0..UINT32_MAX，原协议位置计数，微步/角度关系由驱动器确定。
 * @param speed 0..65535 RPM（协议字段宽度，不是电机额定转速）。
 * @return HAL_OK 两包已提交；HAL_BUSY 不足两个空邮箱，未提交；HAL_ERROR 参数或 HAL 错误。
 * @note 任务调用；不等待发送完成或到位。总线故障时不能保证两包均送达；
 *       HAL_ERROR 后不要假定设备未收到第一包，需结合反馈处理。
 *       speed=0 的含义沿用驱动器协议，不能把此接口当作已验证的停机接口。
 */
HAL_StatusTypeDef Motor_AbsPosition(uint8_t dir, uint16_t id, uint32_t step, uint16_t speed);
/* ======================== 35电机接口：Z轴目标高度 ======================== */
/**
 * @brief 35步进电机绝对位置模式，控制Z轴高度，参考原Motor35_AbsPosition注释。
 * @param h 目标高度，单位 0.1mm；行程=限幅(2030-h,0,1600)。
 * @param speed 目标线速度，单位 mm/s；RPM=speed*30，超过 65535 拒绝发送。
 * @return 继承 Motor_AbsPosition 状态；换算越界返回 HAL_ERROR。
 * @note 位置计数=行程*4494/100，整数向下取整。参数只适用于原机构；
 *       新车必须标定。行程外目标被钳位，不会执行原目标值。
 * @example Motor35_AbsPosition(1000,50); // 目标100.0mm高度，线速度50mm/s
 */
HAL_StatusTypeDef Motor35_AbsPosition(uint32_t h, uint16_t speed);
/* ======================== 28电机接口：悬臂目标半径 ======================== */
/**
 * @brief 28步进电机绝对位置模式，控制悬臂长度，参考原Motor28_AbsPosition注释。
 * @param r 目标半径，单位 0.1mm；行程=限幅(r-1200,0,1660)。
 * @param speed 目标线速度，单位 mm/s；RPM=speed*53/100，整数截断。
 * @return 继承 Motor_AbsPosition 返回状态。
 * @note 位置计数=行程*3189/1000；speed=1 换算为 0 RPM，不自动提高速度。
 *       行程与系数为原车参数，需按新机构标定。
 *       r为夹爪中心到转台中心的总距离，不是从零点额外伸出的距离。
 * @example Motor28_AbsPosition(2000,50); // 目标200.0mm半径，线速度50mm/s
 */
HAL_StatusTypeDef Motor28_AbsPosition(uint32_t r, uint16_t speed);
/* ======================== 公共回复接口（两台电机分别缓存） ======================== */
/**
 * @brief 缓存一台电机最近收到的原始回复。
 * @param id 回复扩展 ID，仅识别配置的两台电机。
 * @param data 至少 length 字节的可读数据，函数内部复制，不保留指针。
 * @param length 有效数据长度 1..8。
 * @return 1 已缓存；0 非本模块 ID 或参数无效。
 * @note 仅由 CAN1 RX 中断分派调用，调用方先筛除标准帧和远程帧。
 *       不校验执行成功/到位码；新回复会覆盖旧回复，计数饱和于 UINT32_MAX。
 */
uint8_t Stepper2835_OnRx(uint32_t id, const uint8_t *data, uint8_t length);
/**
 * @brief 读取回复的原子快照，不清除缓存。
 * @param id 配置的电机扩展 ID。
 * @param reply 输出结构，成功时包含原始数据、长度、计数和最近时间。
 * @return 1 曾收到回复；0 参数无效或尚未收到。无效参数时输出不变。
 * @note 过期回复也返回 1。用无符号差值 HAL_GetTick()-last_tick 判断新鲜度，
 *       不可将返回值视为运动完成；临界区恢复调用前中断状态。
 */
uint8_t Stepper2835_GetReply(uint16_t id, Stepper2835Reply_t *reply);

#ifdef __cplusplus
}
#endif
#endif
