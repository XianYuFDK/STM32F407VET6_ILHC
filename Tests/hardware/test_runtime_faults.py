"""运行真实任务服务，注入发送失败、DMA卡住、使能等待和周期恢复事件。"""
from pathlib import Path
import subprocess
import re
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "Hardware/debug_usart.c").read_text(encoding="utf-8")


def function(name):
    m=re.search(r'^(?:static )?(?:void|uint8_t) '+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert m,name
    start=m.start();brace=source.index('{',start)
    depth,end=1,brace+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]


prelude = r'''
#include <stdint.h>
static uint8_t s_fast_pending __attribute__((unused));
#include <assert.h>
#include <stdio.h>
#define HAL_OK 0
#define DEBUG_HOST_TIMEOUT_MS 1000U
static uint8_t s_stop_req,s_zero_req,s_offset_req,ops_changed,trajectory_busy,trajectory_cancelled;
static float s_offset_x,s_offset_y;
static uint32_t s_host_last_tick,trajectory_cancel_calls,mechanism_notifications;
static struct {uint8_t action;} s_stepper_req[2];
static void Traj_Cancel(uint8_t reason){assert(reason<=19);if(trajectory_busy){trajectory_cancelled=1;++trajectory_cancel_calls;}}
static uint8_t OPS_ConsumeSessionChanged(void){uint8_t v=ops_changed;ops_changed=0;return v;}
static void OPS_ZeroCoordinates(void){}
static int OPS_SetMountOffset(float x,float y){(void)x;(void)y;return 1;}
static void RTOS_APP_NotifyMechanism(void){++mechanism_notifications;}

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
static void DmJ4310_ParamCancel(void){}
static void Debug_ServiceZdt(void){if(s_stop_req||s_zero_req||s_offset_req||s_wheel_req){s_zdt_active=s_zdt_req=0;}}

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
 /* 旧WHEELEN解析覆盖普通请求后，ISR独立失能锁存仍必须获胜。 */
 tick=3000;s_host_last_tick=tick;s_wheel_enabled=1;s_wheel_enable_pending=s_wheel_fault=0;
 s_wheel_error_seen=errors;s_fast_pending=2;s_wheel_req=1;
 s_manual_active=s_goto_active=s_zdt_active=vision_active=1;trajectory_busy=1;
 unsigned before_enables=enables;DebugUsart_ControlEmergency();
 assert(!s_wheel_enabled&&!s_manual_active&&!s_goto_active&&!s_zdt_active&&!vision_active);
 assert(enables==before_enables&&trajectory_cancelled);
 /* STOP撤销尚未执行的WHEELEN，失能底盘不会被重新使能。 */
 s_fast_pending=1;s_wheel_req=1;s_dm_active=1;DebugUsart_ControlEmergency();
 assert(!s_wheel_enabled&&!s_wheel_req&&enables==before_enables&&s_dm_disable_pending&&mechanism_notifications);
 fail_disable=1;DebugUsart_MechanismEmergency();assert(s_dm_disable_pending);
 fail_disable=0;DebugUsart_MechanismEmergency();assert(!s_dm_disable_pending);
 /* 模拟TBEGIN在急停ISR之后才提交RECEIVING，任务侧必须再取消。 */
 trajectory_cancelled=0;s_fast_pending=1;DebugUsart_ControlEmergency();
 assert(trajectory_cancelled&&trajectory_cancel_calls>=3);
 /* 旧DMEN请求不能越过独立DMOFF锁存。 */
 s_dm_active=s_dm_enable_req=1;s_fast_pending=8;DebugUsart_MechanismEmergency();
 assert(!s_dm_active&&!s_dm_enable_req&&enables==before_enables&&!(s_fast_pending&8));
 s_fast_pending=0;mask=1;DebugUsart_ControlEmergency();assert(mask==1);mask=0;
 puts("runtime DM/wheel/TX fault state-machine tests and ISR safety latch races passed");return 0;
}
'''

names = ["Debug_WheelReady", "Debug_ServiceWheel", "Debug_ServiceWheelFault",
         "Debug_RequestDmDisable", "Debug_ServiceDmDisable", "Debug_ServiceDm",
         "DebugUsart_ServiceTx", "Debug_ChassisStop", "Debug_ControlSafety", "DebugUsart_ControlEmergency", "DebugUsart_MechanismEmergency"]
code = prelude + "\n".join(function(n) for n in names) + checks

# 会话取消依旧在控制任务；HAL接收恢复归属通信任务。
start=source.index('static void Debug_ControlSafety(void)')
body=source.index('  if (OPS_ConsumeSessionChanged()',start)
end=source.index('  /* 上位机失联',body)
periodic=source[body:end]
comm=(ROOT/'RTOS_APP/app_comm.c').read_text(encoding='utf-8')
assert 'OPS_ServiceRx(); OPS_ProcessPending();' in comm
periodic_code=r'''
#include <stdint.h>
#include <assert.h>
static uint32_t consumed,stops;
static uint8_t s_goto_active=1;
static uint8_t OPS_ConsumeSessionChanged(void){return ++consumed==2;}
static void Traj_Cancel(uint8_t reason){assert(reason==17U);}
static void Debug_ChassisStop(void){stops++;}
'''+'static void periodic(void){\n'+periodic+r'''
}
int main(void){periodic();assert(consumed==1&&s_goto_active);
periodic();assert(consumed==2&&!s_goto_active&&stops==1);return 0;}
'''

with tempfile.TemporaryDirectory() as directory:
    folder = Path(directory)
    for name, content in [("states", code), ("periodic", periodic_code)]:
        c, exe = folder / (name + ".c"), folder / (name + ".exe")
        c.write_text(content, encoding="utf-8")
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        str(c), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
