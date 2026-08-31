/**
 ******************************************************************************
 * @file    dm_j4310.h
 * @brief   达妙 DM-J4310-2EC V1.1 关节电机 CAN 驱动（协议层，全中文注释）
 *
 *          实现内容移植自 GitHub：
 *              https://github.com/dmBots/motor-control-routine
 *              stm32例程 / DMMotor_freertos.rar / User/bsp_can.c
 *
 *          仅保留两个最小完整驱动模式，另含控制模式切换：
 *              1. MIT 模式：位置、速度、Kp、Kd、前馈力矩
 *              2. 位置速度模式：目标位置 + 最大速度（ID 偏移 0x100）
 *
 *          通信参数：
 *              - CAN1，1 Mbps，标准帧
 *              - 电机 ID 由调用方指定
 *              - 反馈帧始终使用电机 ID 对应的标定范围
 *
 *          注意：电机内部控制模式（寄存器 10）MIT=1、位置速度=2，
 *          可使用 DmJ4310_SetControlMode() 切换；P_MAX / V_MAX / T_MAX 必须与电机调试助手中的
 *          P_MAX / V_MAX / T_MAX 一致，否则位置速度和力矩解析会错误。
 ******************************************************************************
 */
#ifndef __DM_J4310_H__
#define __DM_J4310_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ------------------------- 标定范围（需与调试工具一致） ------------------------ */

#define DM_J4310_POS_MIN        (-12.5f)   /* 位置最小值，单位 rad          */
#define DM_J4310_POS_MAX        12.5f      /* 位置最大值，单位 rad          */
#define DM_J4310_VEL_MIN        (-30.0f)   /* 速度最小值，单位 rad/s        */
#define DM_J4310_VEL_MAX        30.0f      /* 速度最大值，单位 rad/s        */
#define DM_J4310_TORQUE_MIN     (-10.0f)   /* 力矩最小值，单位 Nm           */
#define DM_J4310_TORQUE_MAX     10.0f      /* 力矩最大值，单位 Nm           */
#define DM_J4310_KP_MAX         500.0f     /* Kp 固定范围最大值             */
#define DM_J4310_KD_MAX         5.0f       /* Kd 固定范围最大值             */

/* 位置速度模式命令 ID 偏移 */
#define DM_J4310_CTRL_MODE_MIT     1U   /* 电机内部控制模式：MIT     */
#define DM_J4310_CTRL_MODE_POS_VEL 2U   /* 电机内部控制模式：位置速度 */
#define DM_J4310_POS_VEL_ID_OFFSET  0x100U

/* 反馈状态码（D0 高 4 位） */
#define DM_J4310_STATUS_DISABLED        0x00U  /* 失能            */
#define DM_J4310_STATUS_ENABLED         0x01U  /* 使能            */
#define DM_J4310_FAULT_OVER_VOLTAGE     0x08U  /* 过压            */
#define DM_J4310_FAULT_UNDER_VOLTAGE    0x09U  /* 欠压            */
#define DM_J4310_FAULT_OVER_CURRENT     0x0AU  /* 过流            */
#define DM_J4310_FAULT_MOS_OVER_TEMP    0x0BU  /* MOS 过温        */
#define DM_J4310_FAULT_ROTOR_OVER_TEMP  0x0CU  /* 线圈过温        */
#define DM_J4310_FAULT_LOST_COMM        0x0DU  /* 通信丢失        */
#define DM_J4310_FAULT_OVERLOAD         0x0EU  /* 过载            */

/* 驱动返回状态 */
typedef enum
{
  DM_J4310_OK  = 0U,   /* 成功 */
  DM_J4310_ERR = 1U    /* 参数或发送失败 */
} DmJ4310Result_t;

/* 单电机反馈数据 */
typedef struct
{
  uint8_t  motorId;     /* 电机 ID（D0 低 4 位）      */
  uint8_t  status;      /* 状态/故障码（D0 高 4 位）  */
  float    position;    /* 实际位置，单位 rad         */
  float    velocity;    /* 实际速度，单位 rad/s       */
  float    torque;      /* 实际力矩，单位 Nm          */
  uint8_t  tempMos;     /* MOS 温度，单位摄氏度       */
  uint8_t  tempRotor;   /* 电机线圈温度，单位摄氏度   */
  uint8_t  valid;       /* 1 表示本帧解析成功         */
} DmJ4310Feedback_t;

/* --------------------------- 对外接口 ----------------------------- */

/**
 * @brief  MIT 模式控制单电机
 * @param  canId   电机 ID
 * @param  pos     目标位置（rad）
 * @param  vel     目标速度（rad/s）
 * @param  kp      位置刚度，范围 0~500
 * @param  kd      位置阻尼，范围 0~5
 * @param  torque  前馈力矩（Nm）
 * @return DM_J4310_OK 成功，其他失败
 */
uint8_t DmJ4310_MITControl(uint16_t canId, float pos, float vel,
                           float kp, float kd, float torque);

/**
 * @brief  位置速度模式控制单电机
 * @param  canId  电机 ID
 * @param  pos    目标位置（rad）
 * @param  vel    目标最大速度（rad/s）
 * @return DM_J4310_OK 成功，其他失败
 */
uint8_t DmJ4310_PosVelControl(uint16_t canId, float pos, float vel);

/**
 * @brief  设置电机内部控制模式（寄存器 10）
 * @param  canId 电机 ID
 * @param  mode  DM_J4310_CTRL_MODE_MIT / DM_J4310_CTRL_MODE_POS_VEL
 * @return DM_J4310_OK 成功，其他失败
 */
uint8_t DmJ4310_SetControlMode(uint16_t canId, uint8_t mode);

/**
 * @brief  使能电机（命令帧：FF...FC）
 * @param  canId 电机 ID
 * @return DM_J4310_OK 成功，其他失败
 */
uint8_t DmJ4310_Enable(uint16_t canId);

/**
 * @brief  失能电机（命令帧：FF...FD）
 */
uint8_t DmJ4310_Disable(uint16_t canId);

/**
 * @brief  将当前位置保存为零点（命令帧：FF...FE）
 */
uint8_t DmJ4310_SetZero(uint16_t canId);

/**
 * @brief  解析 DM 电机反馈帧
 *
 *         标准反馈帧格式：
 *          D0：状态[7:4] | 电机ID[3:0]
 *          D1~D2：位置 16bit
 *          D3~D4：速度 12bit
 *          D4~D5：力矩 12bit
 *          D6：MOS 温度
 *          D7：电机线圈温度
 *
 * @param  data CAN 数据，至少 6 字节
 * @param  len  CAN 数据长度
 * @param  fb   输出反馈结构
 * @return DM_J4310_OK 成功，其他失败
 */
uint8_t DmJ4310_DecodeFeedback(const uint8_t *data, uint8_t len,
                               DmJ4310Feedback_t *fb);

#ifdef __cplusplus
}
#endif

#endif /* __DM_J4310_H__ */
