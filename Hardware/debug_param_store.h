#ifndef DEBUG_PARAM_STORE_H
#define DEBUG_PARAM_STORE_H
#include <stdint.h>

/* 固定磁盘字段序；修改字段或语义必须升级存储版本。
 * 七项底盘参数、OPS X/Y(mm)、DM ID/模式/位置/速度/Kp/Kd/力矩。
 * 动作状态、GOTO、MANUAL、ZERO 原点均不在保存范围。 */
typedef struct { float value[16]; } DebugParamValues;
typedef enum {
  PARAM_STORE_DEFAULTS = 0, PARAM_STORE_LOADED, PARAM_STORE_SAVED,
  PARAM_STORE_PENDING, PARAM_STORE_ERROR
} ParamStoreResult;

/* Init 在调试 RX 启动前调用；Service 每周期在默认任务调用。 */
ParamStoreResult DebugParamStore_Init(DebugParamValues *values);
ParamStoreResult DebugParamStore_Service(const DebugParamValues *values, uint32_t now);
uint8_t DebugParamStore_Valid(const DebugParamValues *values);
#endif
