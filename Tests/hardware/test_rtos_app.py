"""编译真实RTOS_APP；用确定性时钟注入提前急停、超期和Flash阻塞。

验证应用调度逻辑和锁边界，不代替板上FreeRTOS抢占及最坏耗时测量。
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
OS = r'''
#ifndef TEST_OS_H
#define TEST_OS_H
#include <stdint.h>
#include <stddef.h>
typedef void *osThreadId_t;
typedef void *osMutexId_t;
typedef void (*osThreadFunc_t)(void *);
typedef enum {osKernelInactive,osKernelReady,osKernelRunning} osKernelState_t;
typedef enum {osPriorityLow=8,osPriorityNormal=24,osPriorityAboveNormal=32,osPriorityHigh=40} osPriority_t;
typedef struct {const char *name;uint32_t attr_bits;} osMutexAttr_t;
typedef struct {const char *name;uint32_t stack_size;osPriority_t priority;} osThreadAttr_t;
#define osMutexPrioInherit 2U
#define osWaitForever 0xFFFFFFFFU
#define osOK 0
#define osFlagsWaitAny 0U
#define osFlagsError 0x80000000U
#define osFlagsErrorTimeout 0xFFFFFFFEU
osThreadId_t osThreadNew(osThreadFunc_t,void *,const osThreadAttr_t *);
osMutexId_t osMutexNew(const osMutexAttr_t *);
int osMutexAcquire(osMutexId_t,uint32_t);
int osMutexRelease(osMutexId_t);
uint32_t osThreadFlagsSet(osThreadId_t,uint32_t);
uint32_t osThreadFlagsWait(uint32_t,uint32_t,uint32_t);
uint32_t osKernelGetTickCount(void);
uint32_t osKernelGetTickFreq(void);
osKernelState_t osKernelGetState(void);
uint32_t osThreadGetStackSpace(osThreadId_t);
void osDelay(uint32_t);
void osDelayUntil(uint32_t);
#endif
'''
MAIN = r'''
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef int HAL_StatusTypeDef;
typedef struct {int unused;} UART_HandleTypeDef;
typedef struct {uint32_t CTRL,CYCCNT;} TestDwt;
typedef struct {uint32_t DEMCR;} TestCore;
extern TestDwt test_dwt;
extern TestCore test_core;
extern uint32_t irq_mask;
#define DWT (&test_dwt)
#define CoreDebug (&test_core)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
static inline uint32_t __get_PRIMASK(void){return irq_mask;}
static inline void __disable_irq(void){irq_mask=1U;}
static inline void __enable_irq(void){irq_mask=0U;}
static inline void __DMB(void){__sync_synchronize();}
uint32_t HAL_GetTick(void);
void Error_Handler(void);
#endif
'''
CHECK = r'''
#include "app_internal.h"
#include "app_rx.h"
#include "debug_usart.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <setjmp.h>
TestDwt test_dwt; TestCore test_core;
uint32_t irq_mask;
static uint32_t now,until,event_at,flag_calls,delays,scans;
static uint32_t control_times[32],mechanism_times[32],emergency_times[32];
static unsigned control_count,mechanism_count,emergency_count,created;
static unsigned locked,heavy,flash_delay,event_pending;
static int fail_mutex,fail_thread=-1;
static osKernelState_t kernel=osKernelReady;
static jmp_buf finish;
static void check_time(void){if((int32_t)(now-until)>=0)longjmp(finish,1);}
osMutexId_t osMutexNew(const osMutexAttr_t *a){assert(a->attr_bits&osMutexPrioInherit);return fail_mutex?NULL:(void *)1;}
osThreadId_t osThreadNew(osThreadFunc_t f,void *arg,const osThreadAttr_t *a){
 static const osPriority_t priorities[]={osPriorityHigh,osPriorityAboveNormal,osPriorityNormal,osPriorityLow};
 static const osThreadFunc_t functions[]={APP_ControlTask,APP_CommTask,APP_MechanismTask,APP_MaintenanceTask};
 unsigned i=created++;assert(!arg&&f==functions[i]&&a->priority==priorities[i]&&a->stack_size>=1536);
 return (int)i==fail_thread?NULL:(void *)(uintptr_t)(i+1);
}
int osMutexAcquire(osMutexId_t m,uint32_t t){assert(m&&t==osWaitForever&&!locked&&!irq_mask);locked=1;return osOK;}
int osMutexRelease(osMutexId_t m){assert(m&&locked&&!irq_mask);locked=0;return osOK;}
uint32_t osThreadFlagsSet(osThreadId_t id,uint32_t f){assert(kernel==osKernelRunning&&id&&f==1);++flag_calls;return f;}
uint32_t osThreadFlagsWait(uint32_t f,uint32_t options,uint32_t timeout){
 assert(!locked&&f==1&&options==osFlagsWaitAny&&timeout);
 if(event_pending&&(uint32_t)(event_at-now)<=timeout){now=event_at;event_pending=0;return 1;}
 now+=timeout;check_time();return osFlagsErrorTimeout;
}
uint32_t osKernelGetTickCount(void){return now;}
uint32_t osKernelGetTickFreq(void){return 1000;}
osKernelState_t osKernelGetState(void){return kernel;}
uint32_t osThreadGetStackSpace(osThreadId_t id){assert(id&&!locked);++scans;return 512;}
size_t xPortGetFreeHeapSize(void){return 8192;}
size_t xPortGetMinimumEverFreeHeapSize(void){return 4096;}
void osDelay(uint32_t t){assert(!locked&&t==1);now+=t;++delays;check_time();}
void osDelayUntil(uint32_t t){assert(!locked);now=t;check_time();}
uint32_t HAL_GetTick(void){return now;}
void Error_Handler(void){assert(0);}
void DebugUsart_ControlService(void){assert(locked);control_times[control_count++]=now;DWT->CYCCNT+=17;if(heavy){heavy=0;now+=25;}}
void DebugUsart_ControlEmergency(void){assert(locked);emergency_times[emergency_count++]=now;}
void DebugUsart_MechanismService(void){assert(locked);mechanism_times[mechanism_count++]=now;}
void DebugUsart_MechanismEmergency(void){assert(locked);emergency_times[emergency_count++]=now;}
void DebugUsart_ParamSnapshot(DebugParamValues *v){assert(locked);memset(v,0,sizeof(*v));}
void DebugUsart_ParamResult(ParamStoreResult r){assert(locked&&r==PARAM_STORE_PENDING);}
ParamStoreResult DebugParamStore_Service(const DebugParamValues *v,uint32_t tick){
 assert(!locked&&v&&tick==now);
 if(flash_delay){
  /* Flash运行中应用锁仍可由高优先级底盘取得。 */
  RTOS_APP_Lock();DebugUsart_ControlEmergency();RTOS_APP_Unlock();now+=flash_delay;
 }
 return PARAM_STORE_PENDING;
}
static void consume(uint8_t port){AppRxPacket p;assert(locked);if(APP_RX_Fault(port))APP_RX_Reset(port);else (void)APP_RX_Take(port,&p);}
void DebugUsart_ProcessPending(void){consume(APP_RX_HOST);}
void OPS_ProcessPending(void){consume(APP_RX_OPS);}
void OPS_ServiceRx(void){assert(locked);}
void Vision_ServiceRx(void){assert(locked);}
void DebugUsart_Send(void){assert(locked);}
void DebugUsart_CommunicationRecover(void){assert(!locked);}
static void reset_run(uint32_t tick,uint32_t end){
 now=tick;until=end;locked=control_count=mechanism_count=emergency_count=event_pending=heavy=flash_delay=delays=scans=0;
 memset(app_task_stats,0,sizeof(AppTaskStats)*APP_TASK_COUNT);
}
int main(void){
 AppRxPacket p;uint8_t data[]={0,1,255};unsigned i;
 /* 内核启动前中断通知安全；所有创建失败必须反馈初始化失败。 */
 RTOS_APP_NotifyControl();assert(!flag_calls);
 fail_mutex=1;assert(!RTOS_APP_Init());fail_mutex=0;
 created=0;fail_thread=2;assert(!RTOS_APP_Init());fail_thread=-1;
 created=0;assert(RTOS_APP_Init()&&created==4);kernel=osKernelRunning;
 RTOS_APP_NotifyComm();RTOS_APP_NotifyControl();RTOS_APP_NotifyMechanism();assert(flag_calls==3);
 /* FIFO、二进制字节、接收时刻、队满隔离、代次和原有PRIMASK。 */
 for(i=0;i<7;++i)assert(APP_RX_Push(APP_RX_HOST,data,3,100+i));
 assert(!APP_RX_Push(APP_RX_HOST,data,3,999)&&app_rx_overflows[0]==1);
 assert(!APP_RX_Take(APP_RX_HOST,&p)&&!APP_RX_Push(APP_RX_HOST,data,3,1000));
 APP_RX_Reset(APP_RX_HOST);assert(!APP_RX_Pending());
 assert(APP_RX_Push(APP_RX_HOST,data,3,123));irq_mask=1;assert(APP_RX_Take(APP_RX_HOST,&p)&&irq_mask);irq_mask=0;
 assert(p.tick==123&&p.size==3&&!memcmp(p.data,data,3)&&p.epoch==APP_RX_Epoch(APP_RX_HOST));
 for(i=0;i<7;++i)assert(APP_RX_Push(APP_RX_OPS,data,3,200+i));
 for(i=0;i<7;++i){assert(APP_RX_Take(APP_RX_OPS,&p));assert(p.tick==200+i);}
 /* 7ms急停不能增加轨迹采样，下一周期仍为20ms。 */
 reset_run(0,61);event_pending=1;event_at=7;
 if(!setjmp(finish))APP_ControlTask(NULL);
 assert(control_count==4&&control_times[0]==0&&control_times[1]==20&&control_times[2]==40&&control_times[3]==60);
 assert(emergency_count==1&&emergency_times[0]==7&&!app_task_stats[APP_CONTROL].overruns);
 /* 25ms工作超期后下一次45ms，不补跑已经错过的20ms。 */
 reset_run(0,66);heavy=1;if(!setjmp(finish))APP_ControlTask(NULL);
 assert(control_count==3&&control_times[1]==45&&control_times[2]==65&&app_task_stats[APP_CONTROL].overruns==1);
 assert(app_task_stats[APP_CONTROL].max_cycles==17);
 /* 32位tick回绕不误判超期。 */
 reset_run(0xFFFFFFF0U,45);if(!setjmp(finish))APP_ControlTask(NULL);
 assert(control_count==4&&control_times[1]==4&&control_times[2]==24&&!app_task_stats[APP_CONTROL].overruns);
 reset_run(0,41);event_pending=1;event_at=7;if(!setjmp(finish))APP_MechanismTask(NULL);
 assert(mechanism_count==3&&mechanism_times[1]==20&&emergency_count==1&&emergency_times[0]==7);
 /* 四包预算后主动阻塞，连续上传不会无界占据CPU。 */
 reset_run(0,21);for(i=0;i<7;++i)assert(APP_RX_Push(APP_RX_HOST,data,3,0));
 if(!setjmp(finish)){APP_CommTask(NULL);}
 assert(delays==1&&!APP_RX_Pending());
 /* 栈扫描1Hz；SPI慢时不占用应用锁，维护超期不补跑。 */
 reset_run(0,2101);if(!setjmp(finish))APP_MaintenanceTask(NULL);
 assert(scans==12&&app_heap_free_bytes==8192&&app_heap_min_bytes==4096);
 reset_run(0,351);flash_delay=120;if(!setjmp(finish))APP_MaintenanceTask(NULL);
 assert(emergency_count==3&&app_task_stats[APP_MAINTENANCE].overruns==3);
 puts("PASS: task ownership, creation failure, ISR startup, queue faults, STOP deadline, overrun/wrap, RX budget, Flash unlock, 1Hz stats");
 return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="ilhc_rtos_app_") as d:
    p = Path(d)
    (p / "cmsis_os2.h").write_text(OS, encoding="utf-8")
    (p / "main.h").write_text(MAIN, encoding="utf-8")
    (p / "FreeRTOS.h").write_text("#include <stddef.h>\nsize_t xPortGetFreeHeapSize(void);\nsize_t xPortGetMinimumEverFreeHeapSize(void);\n", encoding="utf-8")
    (p / "task.h").write_text("", encoding="utf-8")
    (p / "check.c").write_text(CHECK, encoding="utf-8")
    command = ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", str(p), "-I", str(ROOT / "RTOS_APP"), "-I", str(ROOT / "Hardware")]
    command += [str(f) for f in sorted((ROOT / "RTOS_APP").glob("*.c"))]
    subprocess.run(command + [str(p / "check.c"), "-o", str(p / "check.exe")], check=True)
    subprocess.run([str(p / "check.exe")], check=True)
