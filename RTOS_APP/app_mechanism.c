#include "app_internal.h"
#include "debug_usart.h"
/// 机制任务：处理来自调试串口的命令，控制舵机、步进电机等。
void APP_MechanismTask(void *argument)
{
  uint32_t deadline = osKernelGetTickCount(), period = APP_PeriodTicks(20U);
  (void)argument;
  for (;;) {
    uint32_t now = osKernelGetTickCount(), started;
    if ((int32_t)(now - deadline) >= 0) {
      started = APP_Cycles();
      RTOS_APP_Lock(); DebugUsart_MechanismService(); RTOS_APP_Unlock();
      APP_Record(APP_MECHANISM, started);
      APP_AdvanceDeadline(APP_MECHANISM, &deadline, period);
    } else {
      uint32_t flags = osThreadFlagsWait(APP_EVENT_WAKE, osFlagsWaitAny, deadline - now);
      if ((flags & osFlagsError) == 0U && (flags & APP_EVENT_WAKE)) {
        RTOS_APP_Lock(); DebugUsart_MechanismEmergency(); RTOS_APP_Unlock();
      }
    }
  }
}
