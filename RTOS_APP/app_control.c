#include "app_internal.h"
#include "debug_usart.h"
///  控制任务：处理来自调试串口的命令，控制电机、舵机等。
void APP_ControlTask(void *argument)
{
  uint32_t deadline = osKernelGetTickCount(), period = APP_PeriodTicks(20U);
  (void)argument;
  for (;;) {
    uint32_t now = osKernelGetTickCount(), started;
    if ((int32_t)(now - deadline) >= 0) {
      started = APP_Cycles();
      RTOS_APP_Lock(); DebugUsart_ControlService(); RTOS_APP_Unlock();
      APP_Record(APP_CONTROL, started);
      APP_AdvanceDeadline(APP_CONTROL, &deadline, period);
    } else {
      uint32_t flags = osThreadFlagsWait(APP_EVENT_WAKE, osFlagsWaitAny, deadline - now);
      if ((flags & osFlagsError) == 0U && (flags & APP_EVENT_WAKE)) {
        /* 急停提前唤醒只处理取消/停车，不额外执行轨迹、PID或累计停稳时间。 */
        RTOS_APP_Lock(); DebugUsart_ControlEmergency(); RTOS_APP_Unlock();
      }
    }
  }
}
