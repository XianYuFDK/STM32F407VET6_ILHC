#ifndef ILHC_RTOS_APP_H
#define ILHC_RTOS_APP_H

#include "cmsis_os2.h"
#include <stdint.h>

/* 应用任务唯一入口；Core/Src/freertos.c 只负责调用此接口。 */
enum { APP_CONTROL, APP_COMM, APP_MECHANISM, APP_MAINTENANCE, APP_TASK_COUNT };
typedef struct {
  volatile uint32_t runs, overruns, max_cycles, last_tick, stack_free_bytes;
} AppTaskStats;
extern AppTaskStats app_task_stats[APP_TASK_COUNT];
extern volatile uint32_t app_heap_free_bytes, app_heap_min_bytes;

uint8_t RTOS_APP_Init(void);
void RTOS_APP_Lock(void);
void RTOS_APP_Unlock(void);
/* 可由任务或优先级数值 >=5 的外设中断调用；内核启动前不调用RTOS API。 */
void RTOS_APP_NotifyComm(void);
void RTOS_APP_NotifyControl(void);
void RTOS_APP_NotifyMechanism(void);

#endif
