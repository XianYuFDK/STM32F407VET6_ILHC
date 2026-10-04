#include "app_internal.h"
#include "debug_usart.h"
#include "debug_param_store.h"
#include "FreeRTOS.h"
#include "task.h"
/// 维护任务：处理来自调试串口的参数存储请求，定期更新任务和堆栈统计信息。
void APP_MaintenanceTask(void *argument)
{
  uint32_t deadline = osKernelGetTickCount(), period = APP_PeriodTicks(50U);
  uint32_t stats_deadline = deadline;
  (void)argument;
  for (;;) {
    DebugParamValues values;
    ParamStoreResult result;
    uint32_t started = APP_Cycles();
    uint8_t i;
    RTOS_APP_Lock(); DebugUsart_ParamSnapshot(&values); RTOS_APP_Unlock();
    /* SPI/Flash等待期间不持有应用状态锁，底盘和通信可立即抢占。 */
    result = DebugParamStore_Service(&values, HAL_GetTick());
    RTOS_APP_Lock(); DebugUsart_ParamResult(result); RTOS_APP_Unlock();
    if ((int32_t)(osKernelGetTickCount() - stats_deadline) >= 0) {
      for (i = 0U; i < APP_TASK_COUNT; ++i)
        app_task_stats[i].stack_free_bytes = osThreadGetStackSpace(app_threads[i]);
      app_heap_free_bytes = (uint32_t)xPortGetFreeHeapSize();
      app_heap_min_bytes = (uint32_t)xPortGetMinimumEverFreeHeapSize();
      stats_deadline = osKernelGetTickCount() + APP_PeriodTicks(1000U);
    }
    APP_Record(APP_MAINTENANCE, started);
    APP_AdvanceDeadline(APP_MAINTENANCE, &deadline, period);
    osDelayUntil(deadline);
  }
}
