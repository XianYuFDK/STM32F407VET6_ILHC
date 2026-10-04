"""编译实际轨迹参数表、设置和GET函数，验证串口参数到控制器的连接。"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
source=(ROOT/'Hardware/debug_usart.c').read_text(encoding='utf-8')
def function(name):
    start=source.rfind('static ',0,source.index(name+'('))
    brace=source.index('{',start);level=1;end=brace+1
    while level:
        level+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]

table=re.search(r'static const DebugParam_t s_params\[\]\s*=\s*\{.*?\n\};',source,re.S).group()
legacy=set(re.findall(r'&([A-Za-z]\w*)\s*,',table))
prelude="""
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "trajectory_buffer.h"
typedef struct {const char *name;float *value;float min,max;} DebugParam_t;
static uint32_t mask,last_ack;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __enable_irq(void){mask=0;}
static char s_ack_param[16][24];
static uint8_t s_ack_read,s_ack_write,s_ack_queue[16];
static void Debug_ZdtAck(uint8_t event){last_ack=event;}
"""
prelude+=''.join('static float '+name+';\n' for name in sorted(legacy))
for name in ('PARAM_UNKNOWN','PARAM_TEXT','TRAJ_BUSY','TRAJ_RANGE'):
    value=re.search(r'#define DEBUG_ACK_'+name+r'\s+(\d+)U',source).group(1)
    prelude+='#define DEBUG_ACK_'+name+' '+value+'U\n'
checks=r"""
int main(void) {
 unsigned i; float value; char expected[24]; float before;
 Traj_Init();
 for(i=0;i<sizeof(s_params)/sizeof(s_params[0]);i++) {
   const DebugParam_t *p=&s_params[i];
   if(p->name[0]!='T') continue;
   value=(p->min+p->max)*0.5f;
   assert(Debug_ParseFloat("80.5",&before));
   Debug_SetParam(p->name,value);assert(fabsf(*p->value-value)<.001f);
   s_ack_read=s_ack_write=0;Debug_ReplyParam(p->name);
   snprintf(expected,sizeof(expected),"%s=",p->name);
   assert(strncmp(s_ack_param[0],expected,strlen(expected))==0);
   assert(Debug_ParseFloat(s_ack_param[0]+strlen(expected),&before)==0); /* CR/LF必须先剥离 */
   s_ack_param[0][strlen(s_ack_param[0])-2]=0;
   assert(Debug_ParseFloat(s_ack_param[0]+strlen(expected),&before));
   assert(fabsf(before-value)<.0011f);
   before=*p->value;last_ack=0;Debug_SetParam(p->name,p->max+1);
   assert(last_ack==DEBUG_ACK_TRAJ_RANGE && *p->value==before);
   Debug_SetParam(p->name,NAN);assert(*p->value==before);
 }
 assert(!Debug_ParseFloat("nan",&value));assert(!Debug_ParseFloat("inf",&value));
 assert(!Debug_ParseFloat("80abc",&value));assert(!Debug_ParseFloat("1e3",&value));
 before=traj_control_params.speed_mm_s;
 Traj_ParseLine("TBEGIN=17,2,1,00000000,10",1000,1);
 Debug_SetParam("tvmax",80);assert(last_ack==DEBUG_ACK_TRAJ_BUSY);
 assert(traj_control_params.speed_mm_s==before);
 s_ack_read=s_ack_write=0;Debug_ReplyParam("TVMAX");assert(s_ack_write==1);
 puts("Trajectory parameter SET/GET, range and busy gates passed");return 0;
}
"""
with tempfile.TemporaryDirectory(prefix='ilhc_traj_param_') as directory:
    folder=Path(directory)
    (folder/'main.h').write_text('#include <stdint.h>\nstatic inline uint32_t __get_PRIMASK(void){return 0;}\n'
        'static inline void __disable_irq(void){}\nstatic inline void __enable_irq(void){}\n',encoding='utf-8')
    path=folder/'protocol.c';exe=folder/'protocol.exe'
    path.write_text(prelude+table+function('Debug_StrCaseCmp')+function('Debug_ParseFloat')+
                    function('Debug_SetParam')+function('Debug_ReplyParam')+checks,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(folder),'-I',str(ROOT/'Hardware'),
                    str(path),str(ROOT/'Hardware/trajectory_buffer.c'),'-o',str(exe),'-lm'],check=True)
    subprocess.run([str(exe)],check=True)
