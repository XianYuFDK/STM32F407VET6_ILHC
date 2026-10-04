#include "app_internal.h"
#include "FreeRTOS.h"
#include "task.h"

osThreadId_t app_threads[APP_TASK_COUNT];
AppTaskStats app_task_stats[APP_TASK_COUNT];
volatile uint32_t app_heap_free_bytes, app_heap_min_bytes;
static osMutexId_t state_mutex;

static const osMutexAttr_t mutex_attributes = {
  .name = "app_state", .attr_bits = osMutexPrioInherit
};
static const osThreadAttr_t attributes[APP_TASK_COUNT] = {
  { .name = "chassis", .stack_size = 2048U, .priority = osPriorityHigh },
  { .name = "comm", .stack_size = 3072U, .priority = osPriorityAboveNormal },
  { .name = "mechanism", .stack_size = 2048U, .priority = osPriorityNormal },
  { .name = "maintenance", .stack_size = 1536U, .priority = osPriorityLow }
};

uint8_t RTOS_APP_Init(void)
{
  static const osThreadFunc_t entries[APP_TASK_COUNT] = {
    APP_ControlTask, APP_CommTask, APP_MechanismTask, APP_MaintenanceTask
  };
  uint8_t i;
  /* DWT只用于统计，不改变HAL TIM7或FreeRTOS SysTick时基。 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  state_mutex = osMutexNew(&mutex_attributes);
  if (!state_mutex) return 0U;
  for (i = 0U; i < APP_TASK_COUNT; ++i) {
    app_threads[i] = osThreadNew(entries[i], NULL, &attributes[i]);
    if (!app_threads[i]) return 0U;
  }
  return 1U;
}

void RTOS_APP_Lock(void)
{
  if (osMutexAcquire(state_mutex, osWaitForever) != osOK) Error_Handler();
}
void RTOS_APP_Unlock(void)
{
  if (osMutexRelease(state_mutex) != osOK) Error_Handler();
}
static void Notify(uint8_t task)
{
  if (app_threads[task] && osKernelGetState() == osKernelRunning)
    (void)osThreadFlagsSet(app_threads[task], APP_EVENT_WAKE);
}
void RTOS_APP_NotifyComm(void) { Notify(APP_COMM); }
void RTOS_APP_NotifyControl(void) { Notify(APP_CONTROL); }
void RTOS_APP_NotifyMechanism(void) { Notify(APP_MECHANISM); }

uint32_t APP_PeriodTicks(uint32_t milliseconds)
{
  uint32_t ticks = (uint32_t)(((uint64_t)osKernelGetTickFreq() * milliseconds + 999U) / 1000U);
  return ticks ? ticks : 1U;
}
uint32_t APP_Cycles(void) { return DWT->CYCCNT; }
void APP_Record(uint8_t task, uint32_t started)
{
  uint32_t elapsed = APP_Cycles() - started;
  if (elapsed > app_task_stats[task].max_cycles) app_task_stats[task].max_cycles = elapsed;
  ++app_task_stats[task].runs;
  app_task_stats[task].last_tick = osKernelGetTickCount();
}
void APP_AdvanceDeadline(uint8_t task, uint32_t *deadline, uint32_t period)
{
  uint32_t now = osKernelGetTickCount();
  *deadline += period;
  if ((int32_t)(*deadline - now) <= 0) {
    ++app_task_stats[task].overruns;
    *deadline = now + period; /* 超期跳过旧周期，禁止快速补跑控制。 */
  }
}
