"""提取真实串口步进解析/服务函数，用主机GCC验证请求边界和重试，不连接设备。"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "Hardware/debug_usart.c").read_text(encoding="utf-8")


def function(name):
    start = source.rfind("static ", 0, source.index(name + "("))
    brace = source.index("{", start)
    level = 1
    end = brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define MOTOR35_CAN_ID 0x300
#define MOTOR28_CAN_ID 0x400
#define MOTOR35_HOME_HEIGHT 2030U
#define MOTOR35_MAX_TRAVEL 1600U
#define MOTOR28_HOME_RADIUS 1200U
#define MOTOR28_MAX_TRAVEL 1660U
#define DEBUG_HOST_TIMEOUT_MS 1000U
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY} HAL_StatusTypeDef;
typedef struct {uint32_t position, queued_tick; uint16_t speed; uint8_t direction, action;} DebugStepperRequest_t;
static volatile DebugStepperRequest_t s_stepper_req[2];
static uint32_t tick, mask, calls, last_position, last_speed, last_id, last_dir;
static HAL_StatusTypeDef result = HAL_OK;
static uint32_t HAL_GetTick(void) {return tick;}
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __enable_irq(void) {mask=0;}
static HAL_StatusTypeDef Motor_AbsPosition(uint8_t dir, uint16_t id, uint32_t pos, uint16_t speed)
{++calls; last_id=id; last_position=pos; last_speed=speed; last_dir=dir; return result;}
static HAL_StatusTypeDef Motor_Homing(uint16_t id) {return Motor_AbsPosition(0,id,0,0);}
static HAL_StatusTypeDef Motor35_AbsPosition(uint32_t pos, uint16_t speed)
{return Motor_AbsPosition(0,0x300,pos,speed);}
static HAL_StatusTypeDef Motor28_AbsPosition(uint32_t pos, uint16_t speed)
{return Motor_AbsPosition(0,0x400,pos,speed);}
'''
checks = r'''
int main(void) {
  uint32_t n;
  assert(Debug_ParseStepper("s35move=1000,50"));
  assert(calls==0); /* 中断解析阶段不发送CAN */
  Debug_ServiceSteppers();
  assert(calls==1 && last_id==0x300 && last_position==1000 && last_speed==50);
  Debug_ServiceSteppers(); assert(calls==1); /* 已提交命令不重复 */
  Debug_ParseStepper("S28MOVE=2000,50"); result=HAL_BUSY;
  Debug_ServiceSteppers(); assert(s_stepper_req[1].action==2);
  result=HAL_OK; Debug_ServiceSteppers();
  assert(last_id==0x400 && last_position==2000 && !s_stepper_req[1].action);
  Debug_ParseStepper("S35RAW=1,4294967295,65535");
  mask=1; Debug_ServiceSteppers();
  assert(mask==1 && last_position==UINT32_MAX && last_dir==1 && last_speed==65535);
  mask=0;
  Debug_ParseStepper("S35RAW=1,4294967296,1"); assert(!s_stepper_req[0].action);
  Debug_ParseStepper("S35RAW=2,100,1"); assert(!s_stepper_req[0].action);
  Debug_ParseStepper("S35MOVE=429,10"); assert(!s_stepper_req[0].action);
  Debug_ParseStepper("S35MOVE=1000,2185"); assert(!s_stepper_req[0].action);
  Debug_ParseStepper("S28MOVE=2000,1"); assert(!s_stepper_req[1].action);
  Debug_ParseStepper("S28MOVE=2000,50,1"); assert(!s_stepper_req[1].action);
  Debug_ParseStepper("S28MOVE=-2000,50"); assert(!s_stepper_req[1].action);
  Debug_ParseStepper("S28MOVE=2000.0,50"); assert(!s_stepper_req[1].action);
  Debug_ParseStepper("S28HOME"); Debug_ParseStepper("S28CANCEL");
  n=calls; Debug_ServiceSteppers(); assert(n==calls);
  Debug_ParseStepper("S35HOME"); tick=1001;
  Debug_ServiceSteppers(); assert(n==calls && !s_stepper_req[0].action);
  Debug_ParseStepper("S35HOME"); result=HAL_ERROR;
  Debug_ServiceSteppers(); assert(!s_stepper_req[0].action && mask==0);
  assert(!Debug_ParseStepper("KPX=1"));
  puts("Stepper command parsing / dispatch / retry tests passed");
  return 0;
}
'''
names = ["Debug_StrCaseCmp", "Debug_StrCaseCmpN", "Debug_ParseUnsignedList",
         "Debug_ParseStepper", "Debug_ServiceSteppers"]
with tempfile.TemporaryDirectory(prefix="ilhc_stepper_test_") as directory:
    folder = Path(directory)
    src = folder / "test.c"
    exe = folder / "test.exe"
    src.write_text(prelude + "\n".join(function(n) for n in names) + checks, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
