/**
 ******************************************************************************
 * @file    dm_j4310.c
 * @brief   达妙 DM-J4310-2EC V1.1 关节电机 CAN 驱动（协议层）
 *
 *          移植自 GitHub dmBots/motor-control-routine：
 *              stm32例程/DMMotor_freertos.rar/User/bsp_can.c
 *
 *          仅保留：
 *              - MIT 控制帧
 *              - 位置速度控制帧
 *              - 使能/失能/清零
 *              - 反馈帧解析
 *
 *          底层复用 Hardware/hcan.c：
 *              CAN_SendData() 发送标准 CAN 帧
 ******************************************************************************
 */
#include "dm_j4310.h"
#include "hcan.h"

#include <string.h>

/* --------------------------- 私有转换函数 ------------------------- */

/**
 * @brief  浮点数线性映射为无符号定点整数
 * @param  x     输入的浮点值
 * @param  x_min 映射最小值
 * @param  x_max 映射最大值
 * @param  bits  定点数位数（12 或 16）
 * @return 映射后的无符号整数
 */
static uint16_t DmJ4310_FloatToUint(float x, float x_min, float x_max, uint8_t bits)
{
  float span;
  float factor;
  uint32_t max_value;
  uint16_t result;

  if ((bits == 0U) || (bits > 16U) || (x_max <= x_min))
  {
    return 0U;
  }

  span = x_max - x_min;
  max_value = (1UL << bits) - 1UL;
  factor = (float)max_value / span;

  /* 输入越界时直接映射到端点，防止超出 12/16 位范围 */
  if (x <= x_min)
  {
    return 0U;
  }
  if (x >= x_max)
  {
    return (uint16_t)max_value;
  }

  result = (uint16_t)((x - x_min) * factor);
  return result;
}

/**
 * @brief  无符号定点整数线性映射为浮点数
 * @param  x     输入的无符号整数
 * @param  x_min 映射最小值
 * @param  x_max 映射最大值
 * @param  bits  定点数位数（12 或 16）
 * @return 映射后的浮点值
 */
static float DmJ4310_UintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits)
{
  float span;
  float factor;
  uint32_t max_value;

  if ((bits == 0U) || (bits > 16U) || (x_max <= x_min))
  {
    return 0.0f;
  }

  span = x_max - x_min;
  max_value = (1UL << bits) - 1UL;
  factor = (float)max_value;

  return ((float)x * span / factor) + x_min;
}

/**
 * @brief  发送电机系统命令帧
 * @param  canId 电机 ID
 * @param  cmd   命令字节：0xFC 使能、0xFD 失能、0xFE 清零
 * @return DM_J4310_OK 成功，其他失败
 */
static uint8_t DmJ4310_SendSystemCmd(uint16_t canId, uint8_t cmd)
{
  uint8_t data[8];

  /* 标准 CAN ID 不能超过 11 位，同时为 0x100/0x200 偏移保留空间 */
  if (canId > 0x06FFU)
  {
    return DM_J4310_ERR;
  }

  data[0] = 0xFFU;
  data[1] = 0xFFU;
  data[2] = 0xFFU;
  data[3] = 0xFFU;
  data[4] = 0xFFU;
  data[5] = 0xFFU;
  data[6] = 0xFFU;
  data[7] = cmd;

  return (CAN_SendData(HCAN_CAN_NUM, canId, data, sizeof(data)) == HAL_OK) ?
         DM_J4310_OK : DM_J4310_ERR;
}

/* --------------------------- 对外接口 ----------------------------- */

/**
 * @brief  MIT 模式控制单电机
 */
uint8_t DmJ4310_MITControl(uint16_t canId, float pos, float vel,
                           float kp, float kd, float torque)
{
  uint16_t pos_u;
  uint16_t vel_u;
  uint16_t kp_u;
  uint16_t kd_u;
  uint16_t torque_u;
  uint8_t data[8];

  if (canId > 0x06FFU)
  {
    return DM_J4310_ERR;
  }

  /* 按照参考驱动默认范围线性量化 */
  pos_u    = DmJ4310_FloatToUint(pos, DM_J4310_POS_MIN, DM_J4310_POS_MAX, 16U);
  vel_u    = DmJ4310_FloatToUint(vel, DM_J4310_VEL_MIN, DM_J4310_VEL_MAX, 12U);
  kp_u     = DmJ4310_FloatToUint(kp, 0.0f, DM_J4310_KP_MAX, 12U);
  kd_u     = DmJ4310_FloatToUint(kd, 0.0f, DM_J4310_KD_MAX, 12U);
  torque_u = DmJ4310_FloatToUint(torque, DM_J4310_TORQUE_MIN, DM_J4310_TORQUE_MAX, 12U);

  /* MIT 8 字节位打包，与官方示例逐位一致 */
  data[0] = (uint8_t)(pos_u >> 8);
  data[1] = (uint8_t)(pos_u & 0xFFU);
  data[2] = (uint8_t)(vel_u >> 4);
  data[3] = (uint8_t)(((vel_u & 0x0FU) << 4) | ((kp_u >> 8) & 0x0FU));
  data[4] = (uint8_t)(kp_u & 0xFFU);
  data[5] = (uint8_t)(kd_u >> 4);
  data[6] = (uint8_t)(((kd_u & 0x0FU) << 4) | ((torque_u >> 8) & 0x0FU));
  data[7] = (uint8_t)(torque_u & 0xFFU);

  return (CAN_SendData(HCAN_CAN_NUM, canId, data, sizeof(data)) == HAL_OK) ?
         DM_J4310_OK : DM_J4310_ERR;
}

/**
 * @brief  位置速度模式控制单电机
 */
uint8_t DmJ4310_PosVelControl(uint16_t canId, float pos, float vel)
{
  uint16_t cmd_id;
  uint8_t data[8];

  cmd_id = canId + DM_J4310_POS_VEL_ID_OFFSET;
  if ((canId > 0x06FFU) || (cmd_id > 0x07FFU))
  {
    return DM_J4310_ERR;
  }

  /* 位置和最大速度均为 float，小端序（STM32 默认小端） */
  memcpy(&data[0], &pos, sizeof(float));
  memcpy(&data[4], &vel, sizeof(float));

  return (CAN_SendData(HCAN_CAN_NUM, cmd_id, data, sizeof(data)) == HAL_OK) ?
         DM_J4310_OK : DM_J4310_ERR;
}

/**
 * @brief  设置电机内部控制模式（寄存器 10）
 */
uint8_t DmJ4310_SetControlMode(uint16_t canId, uint8_t mode)
{ 
  uint8_t data[8];

  if ((canId > 0x06FFU) ||
      ((mode != DM_J4310_CTRL_MODE_MIT) && (mode != DM_J4310_CTRL_MODE_POS_VEL)))
  {
    return DM_J4310_ERR;
  }

  /* 寄存器写命令：CAN ID 0x7FF，数据中携带电机 ID 和控制模式 */
  data[0] = (uint8_t)(canId & 0xFFU);
  data[1] = (uint8_t)((canId >> 8) & 0xFFU);
  data[2] = 0x55U;   /* 写寄存器命令 */
  data[3] = 10U;     /* RID：控制模式 */
  data[4] = mode;    /* 模式值：1=MIT，2=位置速度 */
  data[5] = 0x00U;
  data[6] = 0x00U;
  data[7] = 0x00U;

  return (CAN_SendData(HCAN_CAN_NUM, 0x07FFU, data, sizeof(data)) == HAL_OK) ?
         DM_J4310_OK : DM_J4310_ERR;
}

/**
 * @brief  使能电机
 */
uint8_t DmJ4310_Enable(uint16_t canId)
{
  return DmJ4310_SendSystemCmd(canId, 0xFCU);
}

/**
 * @brief  失能电机
 */
uint8_t DmJ4310_Disable(uint16_t canId)
{
  return DmJ4310_SendSystemCmd(canId, 0xFDU);
}

/**
 * @brief  把当前位置保存为零点
 */
uint8_t DmJ4310_SetZero(uint16_t canId)
{
  return DmJ4310_SendSystemCmd(canId, 0xFEU);
}

/**
 * @brief  解析 DM 电机反馈帧
 */
uint8_t DmJ4310_DecodeFeedback(const uint8_t *data, uint8_t len,
                               DmJ4310Feedback_t *fb)
{
  uint16_t pos_u;
  uint16_t vel_u;
  uint16_t torque_u;

  if ((data == NULL) || (fb == NULL) || (len < 6U))
  {
    return DM_J4310_ERR;
  }

  memset(fb, 0, sizeof(DmJ4310Feedback_t));

  /* D0 高 4 位为状态/故障码，低 4 位为电机 ID */
  fb->motorId = (uint8_t)(data[0] & 0x0FU);
  fb->status  = (uint8_t)(data[0] >> 4);

  /* 位拼接，顺序与官方参考 bsp_can.c 一致 */
  pos_u    = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
  vel_u    = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
  torque_u = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);

  fb->position = DmJ4310_UintToFloat(pos_u, DM_J4310_POS_MIN, DM_J4310_POS_MAX, 16U);
  fb->velocity = DmJ4310_UintToFloat(vel_u, DM_J4310_VEL_MIN, DM_J4310_VEL_MAX, 12U);
  fb->torque   = DmJ4310_UintToFloat(torque_u, DM_J4310_TORQUE_MIN, DM_J4310_TORQUE_MAX, 12U);

  if (len > 6U)
  {
    fb->tempMos = data[6];
  }
  if (len > 7U)
  {
    fb->tempRotor = data[7];
  }

  fb->valid = 1U;
  return DM_J4310_OK;
}
