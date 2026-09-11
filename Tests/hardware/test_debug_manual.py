"""编译真实手动解析和服务函数，验证边界、超时与中断状态恢复。"""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[2] / "Hardware/debug_usart.c").read_text(encoding="utf-8")
def function(name):
    start = source.rfind("static ", 0, source.index(name + "("))
    brace = source.index("{", start)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]

prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
static volatile uint8_t s_manual_active;
static volatile int16_t s_manual_velocity[3];
static volatile uint32_t s_manual_tick;
static uint32_t tick, mask, stops, moves;
static int16_t output[3];
static uint32_t HAL_GetTick(void) {return tick;}
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __enable_irq(void) {mask=0;}
static void MecanumControl_Stop(void) {stops++;}
static void MecanumControl_MoveVelocity(float x,float y,float w)
{moves++; output[0]=x; output[1]=y; output[2]=w;}
'''
checks = r'''
int main(void) {
 int16_t v[3];
 const char *bad[]={"301,0,0","-301,0,0","60,0","60,0,0,1","1.5,0,0","nan,0,0","1,2,3x","1,2,","999999999999999,0,0"};
 unsigned i;
 assert(Debug_ParseManual("-300,+300,0",v));
 assert(v[0]==-300 && v[1]==300 && v[2]==0);
 for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(!Debug_ParseManual(bad[i],v));
 s_manual_active=1; s_manual_velocity[0]=60; s_manual_velocity[2]=30;
 tick=350; Debug_ServiceManual(); assert(moves==1 && output[0]==60 && output[2]==30);
 tick=351; Debug_ServiceManual(); assert(stops==1 && !s_manual_active);
 Debug_ServiceManual(); assert(stops==1 && moves==1);
 s_manual_active=1; s_manual_tick=UINT32_MAX-100; tick=249; mask=1;
 Debug_ServiceManual(); assert(moves==2 && mask==1);
 tick=250; Debug_ServiceManual(); assert(stops==2 && !s_manual_active && mask==1);
 s_manual_active=1; s_manual_tick=tick; s_manual_velocity[0]=s_manual_velocity[2]=0;
 mask=0; Debug_ServiceManual(); assert(stops==3 && !mask);
 puts("Manual parser / mixed velocity / timeout / tick wrap tests passed");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="ilhc_manual_test_") as directory:
    folder=Path(directory)
    src, exe=folder/"test.c", folder/"test.exe"
    src.write_text(prelude + function("Debug_ParseManual") + function("Debug_ServiceManual") + checks,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
