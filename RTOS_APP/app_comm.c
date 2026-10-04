#include "app_internal.h"
#include "app_rx.h"
#include "debug_usart.h"
#include "ops.h"
#include "vision.h"

void APP_CommTask(void *argument)
{
  uint32_t deadline = osKernelGetTickCount(), period = APP_PeriodTicks(20U);
  (void)argument;
  for (;;) {
    uint8_t i;
    uint32_t started = APP_Cycles(), now;
    /* 每轮最多4包；持续上传时让出至少一个tick，机构任务也能获得CPU。 */
    for (i = 0U; i < 4U; ++i) {
      RTOS_APP_Lock();
      OPS_ServiceRx(); OPS_ProcessPending(); DebugUsart_ProcessPending(); Vision_ServiceRx();
      RTOS_APP_Unlock();
      if (!APP_RX_Pending()) break;
    }
    /* USART1恢复和DMA发送只由本任务操作。 */
    DebugUsart_CommunicationRecover();
    now = osKernelGetTickCount();
    if ((int32_t)(now - deadline) >= 0) {
      RTOS_APP_Lock(); DebugUsart_Send(); RTOS_APP_Unlock();
      APP_AdvanceDeadline(APP_COMM, &deadline, period);
    }
    APP_Record(APP_COMM, started);
    if (APP_RX_Pending()) osDelay(1U);
    else {
      now = osKernelGetTickCount();
      if ((int32_t)(deadline - now) > 0)
        (void)osThreadFlagsWait(APP_EVENT_WAKE, osFlagsWaitAny, deadline - now);
    }
  }
}
