"""编译真实视觉调度函数，验证启停、失能闸门与停车去重。"""
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "Hardware/debug_usart.c").read_text(encoding="utf-8")
start = source.index("static void Debug_ServiceVision(void)")
brace = source.index("{", start)
depth, end = 1, brace + 1
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
service = source[start:end]

prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define DEBUG_ACK_VTRACK_BUSY 24U
#define DEBUG_ACK_VTRACK_ERROR 25U
typedef struct {float vx_rpm, vy_rpm; uint8_t move;} VisionTrackOutput;
static volatile uint8_t s_vision_req;
static uint8_t s_vision_moving, s_zdt_active, s_zdt_req;
static uint8_t s_stop_req, s_zero_req, s_offset_req;
static uint8_t s_manual_active, s_goto_active;
static uint8_t wheel_ready=1, vision_active, mock_move, mock_error;
static uint8_t color_seen, ack_seen;
static uint32_t tick, mask, stop_calls, move_calls;
static float vx_seen, vy_seen, vz_seen;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1U;}
static void __enable_irq(void){mask=0U;}
static uint32_t HAL_GetTick(void){return tick;}
static uint8_t Debug_WheelReady(void){return wheel_ready;}
static void Debug_ZdtAck(uint8_t event){ack_seen=event;}
static void Debug_ChassisStop(void){stop_calls++;}
static uint8_t VisionTrack_IsActive(void){return vision_active;}
static uint8_t VisionTrack_Start(uint8_t color){color_seen=color;vision_active=1U;return 1U;}
static void VisionTrack_Stop(void){vision_active=0U;}
static void VisionTrack_Service(uint32_t now, VisionTrackOutput *out){
  (void)now;
  if(mock_error) vision_active=0U;
  out->move=(uint8_t)(vision_active && mock_move);
  out->vx_rpm=12.0f;out->vy_rpm=-18.0f;
}
static void MecanumControl_MoveVelocity(float x,float y,float z){
  move_calls++;vx_seen=x;vy_seen=y;vz_seen=z;
}
'''

checks = r'''
int main(void){
  s_manual_active=s_goto_active=1U;
  s_vision_req=2U;
  Debug_ServiceVision();
  assert(vision_active && color_seen==2U && !s_vision_req);
  assert(!s_manual_active && !s_goto_active && stop_calls==1U && !move_calls);

  mock_move=1U;Debug_ServiceVision();
  assert(move_calls==1U && s_vision_moving);
  assert(vx_seen==12.0f && vy_seen==-18.0f && vz_seen==0.0f);
  mock_move=0U;Debug_ServiceVision();
  assert(stop_calls==2U && !s_vision_moving);
  Debug_ServiceVision();assert(stop_calls==2U); /* 静止时不重复发停车帧。 */

  mock_move=1U;Debug_ServiceVision();assert(s_vision_moving);
  s_vision_req=0xFFU;Debug_ServiceVision();
  assert(!vision_active && !s_vision_moving && stop_calls==3U);

  s_zdt_active=1U;s_vision_req=1U;Debug_ServiceVision();
  assert(!vision_active && ack_seen==DEBUG_ACK_VTRACK_BUSY);
  s_zdt_active=0U;s_vision_req=1U;mock_move=0U;Debug_ServiceVision();
  assert(vision_active);
  mock_move=1U;Debug_ServiceVision();assert(s_vision_moving);
  wheel_ready=0U;Debug_ServiceVision();
  assert(!vision_active && !s_vision_moving && stop_calls==5U);

  wheel_ready=1U;s_vision_req=1U;mock_move=0U;Debug_ServiceVision();
  mock_error=1U;Debug_ServiceVision();
  assert(!vision_active && ack_seen==DEBUG_ACK_VTRACK_ERROR);
  mock_error=0U;s_vision_req=1U;Debug_ServiceVision();
  mock_move=1U;Debug_ServiceVision();assert(s_vision_moving);
  s_stop_req=1U;Debug_ServiceVision();
  assert(!vision_active && !s_vision_moving);
  puts("vision debug-control arbitration passed");
  return 0;
}
'''

with tempfile.TemporaryDirectory() as temporary:
    folder = Path(temporary)
    c_file, exe = folder / "test.c", folder / "test.exe"
    c_file.write_text(prelude + service + checks, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
         str(c_file), "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)

# 控制优先级还包含默认任务中的STOP、失联和四轮故障路径。
send = (source[source.index("static void Debug_ControlSafety(void)"):source.index("void DebugUsart_ControlEmergency(void)")] + source[source.index("void DebugUsart_ControlService(void)"):source.index("void DebugUsart_MechanismEmergency(void)")])
assert "s_vision_req = 0xFFU;" in send
assert send.index("Debug_ServiceVision();") < send.index("Debug_ServiceGoto();")
assert send.index("Debug_ServiceVision();") < send.index("Debug_ServiceWheel();")
assert "VisionTrack_Stop();" in source[source.index("static void Debug_ServiceWheelFault(void)"):]
