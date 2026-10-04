#ifndef ILHC_APP_INTERNAL_H
#define ILHC_APP_INTERNAL_H

#include "rtos_app.h"
#include "main.h"

#define APP_EVENT_WAKE 1U
extern osThreadId_t app_threads[APP_TASK_COUNT];
uint32_t APP_PeriodTicks(uint32_t milliseconds);
uint32_t APP_Cycles(void);
void APP_Record(uint8_t task, uint32_t started);
void APP_AdvanceDeadline(uint8_t task, uint32_t *deadline, uint32_t period);
void APP_ControlTask(void *argument);
void APP_CommTask(void *argument);
void APP_MechanismTask(void *argument);
void APP_MaintenanceTask(void *argument);

#endif
