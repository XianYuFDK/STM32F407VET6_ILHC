"""运行真实任务服务，注入发送失败、DMA卡住、使能等待和周期恢复事件。"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "Hardware/debug_usart.c").read_text(encoding="utf-8")


def function(name):
    start = source.rfind("static ", 0, source.index(name + "("))
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define HAL_OK 0
#define DM_J4310_OK 0
#define HAL_UART_STATE_READY 0
#define DEBUG_DM_MODE_WAIT_MS 100U
#define DM_J4310_CTRL_MODE_MIT 1U
static uint32_t tick, mask, errors, disables, modes, enables, last_id, ack;
static int fail_disable;
static uint8_t s_wheel_req, s_wheel_enabled=1, s_wheel_enable_pending;
static uint8_t s_wheel_fault, s_wheel_disable_pending, s_stop_in_progress;
static uint32_t s_wheel_enable_tick, s_wheel_fault_tick, s_wheel_error_seen;
static uint8_t s_manual_active, s_goto_active, s_zdt_active, s_zdt_req;
static uint8_t s_vision_req, s_vision_moving, vision_active;
static uint8_t VisionTrack_IsActive(void){return vision_active;}
static void VisionTrack_Stop(void){vision_active=0U;}
static uint8_t s_dm_active, s_dm_start_pending, s_dm_disable_pending;
static uint8_t s_dm_disable_fault, s_dm_enable_req, s_dm_mode_req;
static uint8_t s_dm_disable_req, s_dm_zero_req, s_dm_mode=1;
static uint16_t s_dm_id=1, s_dm_disable_id;
static uint32_t s_dm_disable_tick, s_dm_mode_tick;
static float s_dm_pos, s_dm_vel, s_dm_kp, s_dm_kd, s_dm_torque;
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __enable_irq(void){mask=0;}
static void Debug_ZdtAck(uint8_t event){ack=event;}
static uint32_t ZDT_X42S_GetTxErrorCount(void){return errors;}
static int ZDT_X42S_Disable(uint8_t addr){(void)addr;disables++;if(fail_disable){errors++;return 1;}return 0;}
static void MecanumControl_ClearTarget(void){}
static void MecanumControl_Stop(void){}
static void MecanumControl_Enable(void){enables++;}
static void MecanumControl_Disable(void){disables++;}
static int DmJ4310_Disable(uint16_t id){last_id=id;disables++;return fail_disable?1:0;}
static int DmJ4310_SetControlMode(uint16_t id,uint8_t mode){(void)id;(void)mode;modes++;return 0;}
static int DmJ4310_Enable(uint16_t id){(void)id;enables++;return 0;}
static int DmJ4310_SetZero(uint16_t id){(void)id;return 0;}
static int DmJ4310_MITControl(uint16_t id,float p,float v,float kp,float kd,float t)
{(void)id;(void)p;(void)v;(void)kp;(void)kd;(void)t;return 0;}
static int DmJ4310_PosVelControl(uint16_t id,float p,float v){(void)id;(void)p;(void)v;return 0;}
typedef struct {int gState;} UART_HandleTypeDef;
static UART_HandleTypeDef huart1;
static uint8_t s_tx_busy_seen;
static uint32_t s_tx_busy_tick, debug_tx_recoveries, aborts;
static int HAL_UART_AbortTransmit(UART_HandleTypeDef *p){aborts++;p->gState=0;return HAL_OK;}
'''

checks = r'''
int main(void){
 /* DMEN: CAN忙时不能越过失能阶段；恢复后仍等待100ms才使能。 */
 s_dm_enable_req=1;fail_disable=1;Debug_ServiceDm();
 assert(s_dm_disable_pending && !modes && !enables);
 tick=20;Debug_ServiceDm();assert(!modes && !enables);
 fail_disable=0;tick=40;Debug_ServiceDm();assert(modes==1 && !enables);
 tick=139;Debug_ServiceDm();assert(!enables);
 tick=140;Debug_ServiceDm();assert(enables==1 && s_dm_active);
 /* 失能绑定请求时的ID，即使别处改了ID也不会停错电机。 */
 Debug_RequestDmDisable();s_dm_id=2;fail_disable=1;
 tick=160;Debug_ServiceDmDisable();assert(last_id==1 && s_dm_disable_pending);
 tick=1140;Debug_ServiceDmDisable();assert(s_dm_disable_fault && !s_dm_disable_pending && ack==13);
 s_dm_enable_req=1;Debug_ServiceDm();assert(enables==1);
 s_dm_disable_req=1;fail_disable=0;Debug_ServiceDm();assert(!s_dm_disable_pending && !s_dm_disable_fault);
 /* 四轮使能等待不阻塞，期间闸门关闭；100ms后恢复。 */
 s_wheel_req=1;tick=2000;Debug_ServiceWheel();assert(!Debug_WheelReady());
 tick=2099;Debug_ServiceWheel();assert(!Debug_WheelReady());
 tick=2100;Debug_ServiceWheel();assert(Debug_WheelReady());
 /* 轮控发送失败，取消所有运动且故障保持，不能继续下发速度。 */
 errors=1;s_manual_active=s_goto_active=s_zdt_active=1;
 Debug_ServiceWheelFault();assert(s_wheel_fault && !Debug_WheelReady());
 assert(!s_manual_active && !s_goto_active && !s_zdt_active && ack==14);
 s_wheel_req=1;Debug_ServiceWheel();Debug_ServiceWheelFault();
 tick+=100;Debug_ServiceWheel();assert(Debug_WheelReady());
 /* TX卡住：超时只中止TX；正常DMA不被过早中止。覆盖tick回绕。 */
 huart1.gState=1;tick=0xfffffff0U;DebugUsart_ServiceTx();
 tick+=99;DebugUsart_ServiceTx();assert(!aborts);
 tick++;DebugUsart_ServiceTx();assert(aborts==1 && debug_tx_recoveries==1);
 DebugUsart_ServiceTx();assert(aborts==1);
 puts("runtime DM/wheel/TX fault state-machine tests passed");return 0;
}
'''

names = ["Debug_WheelReady", "Debug_ServiceWheel", "Debug_ServiceWheelFault",
         "Debug_RequestDmDisable", "Debug_ServiceDmDisable", "Debug_ServiceDm",
         "DebugUsart_ServiceTx"]
code = prelude + "\n".join(function(n) for n in names) + checks

# 从真实周期入口提取恢复调用段执行，防止函数只存在却未被周期调用。
start = source.index("void DebugUsart_Send(void)")
body = source.index("  {\n    uint32_t now = HAL_GetTick();", start)
end = source.index("  /* 上位机失联", body)
periodic = source[body:end]
periodic_code = r'''
#include <stdint.h>
#include <assert.h>
static uint32_t s_control_tick, services, consumed, stops, tick;
static uint8_t s_goto_active=1;
static uint32_t HAL_GetTick(void){return tick;}
static void MecanumControl_SetPeriod(uint32_t dt){(void)dt;}
static void DebugUsart_ServiceTx(void){}
static void Debug_ServiceWheelFault(void){}
static void OPS_ServiceRx(void){services++;}
static uint8_t OPS_ConsumeSessionChanged(void){return ++consumed==2;}
static void Debug_ChassisStop(void){stops++;}
static void DebugUsart_ServiceRx(void){}
static void Debug_ServiceZdtReplies(void){}
static void Debug_ServiceZdt(void){}
''' + "static void periodic(void){\n" + periodic + r'''
}
int main(void){periodic();assert(services==1 && s_goto_active);
tick=20;periodic();assert(services==2 && !s_goto_active && stops==1);return 0;}
'''

with tempfile.TemporaryDirectory() as directory:
    folder = Path(directory)
    for name, content in [("states", code), ("periodic", periodic_code)]:
        c, exe = folder / (name + ".c"), folder / (name + ".exe")
        c.write_text(content, encoding="utf-8")
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        str(c), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
