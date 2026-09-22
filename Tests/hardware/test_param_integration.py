"""编译真实调试参数桥接函数，验证字段顺序、启动恢复与无动作副作用。"""
from pathlib import Path
import re
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[2]
source=(ROOT/'Hardware/debug_usart.c').read_text(encoding='utf-8')
def function(name):
    match=re.search(r'static void '+name+r'\([^)]*\)\s*\{',source)
    start=match.start(); end=match.end(); depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]
PRELUDE=r'''
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "debug_param_store.h"
static float chassis[7]={1,2,3,4,5,6,7},offset[2]={60,-50};
static struct {float *value;} s_params[7]={{chassis},{chassis+1},{chassis+2},{chassis+3},{chassis+4},{chassis+5},{chassis+6}};
static uint16_t s_dm_id=11;
static uint8_t s_dm_mode=2,s_param_error_reported;
static float s_dm_pos=12,s_dm_vel=13,s_dm_kp=14,s_dm_kd=15,s_dm_torque=16;
static uint32_t mask,ack,count;
static ParamStoreResult result;
static DebugParamValues stored,last;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __enable_irq(void){mask=0;}
static uint32_t HAL_GetTick(void){return 100;}
static void OPS_GetMountOffset(float*x,float*y){*x=offset[0];*y=offset[1];}
static int OPS_SetMountOffset(float x,float y){assert(!mask);offset[0]=x;offset[1]=y;return 1;}
static void Debug_ZdtAck(uint8_t e){ack=e;count++;}
ParamStoreResult DebugParamStore_Init(DebugParamValues*v){assert(!mask);if(result==PARAM_STORE_LOADED)*v=stored;return result;}
ParamStoreResult DebugParamStore_Service(const DebugParamValues*v,uint32_t now){assert(!mask&&now==100);last=*v;return result;}
'''
CHECK=r'''
int main(void){
 DebugParamValues v;Debug_CaptureParams(&v);
 for(unsigned i=0;i<7;i++)assert(v.value[i]==(float)(i+1));
 assert(v.value[7]==60&&v.value[8]==-50&&v.value[9]==11&&v.value[10]==2);
 for(unsigned i=11;i<16;i++)assert(v.value[i]==(float)(i+1));
 mask=1;Debug_CaptureParams(&v);assert(mask==1);mask=0;
 stored=v;stored.value[0]=8;stored.value[7]=40;stored.value[8]=-20;
 stored.value[9]=3;stored.value[10]=1;stored.value[11]=0.5f;
 result=PARAM_STORE_LOADED;Debug_InitParams();assert(ack==15);
 Debug_CaptureParams(&v);assert(!memcmp(&v,&stored,sizeof(v)));
 result=PARAM_STORE_SAVED;Debug_ServiceParams();assert(ack==17&&!memcmp(&last,&stored,sizeof(last)));
 result=PARAM_STORE_ERROR;Debug_ServiceParams();assert(ack==18);unsigned n=count;
 Debug_ServiceParams();assert(count==n);
 Debug_InitParams();Debug_CaptureParams(&v);assert(!memcmp(&v,&stored,sizeof(v)));
 result=PARAM_STORE_DEFAULTS;Debug_InitParams();assert(ack==16);
 puts("PASS: 16-field mapping, restore, IRQ boundaries, save ACK, fault isolation; no motion dependencies");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='ilhc_param_integration_') as directory:
    p=Path(directory); code=PRELUDE+'\n'.join(function(n) for n in ['Debug_CaptureParams','Debug_InitParams','Debug_ServiceParams'])+CHECK
    (p/'test.c').write_text(code,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(ROOT/'Hardware'),str(p/'test.c'),'-o',str(p/'test.exe')],check=True)
    subprocess.run([str(p/'test.exe')],check=True)
init=source[source.index('void DebugUsart_Init(void)'):source.index('void DebugUsart_Send(void)')]
assert init.index('Debug_InitParams();')<init.index('DebugUsart_ServiceRx();')
send=source[source.index('void DebugUsart_Send(void)'):]
assert send.index('Debug_ServiceParams();')<send.index('if (huart1.gState != HAL_UART_STATE_READY) return;')
