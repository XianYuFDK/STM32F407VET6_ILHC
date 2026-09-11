"""使用真实服务函数验证单轮测试时序，不连接硬件。"""
from pathlib import Path
import subprocess, tempfile
source = (Path(__file__).resolve().parents[2]/"Hardware/debug_usart.c").read_text(encoding="utf-8")
def extract(name):
 start=source.rfind("static ",0,source.index(name+"(")); brace=source.index("{",start); end=brace+1; depth=1
 while depth:
  depth+=(source[end]=="{")-(source[end]=="}"); end+=1
 return source[start:end]
code=r"""
#include <stdint.h>
#include <assert.h>
static uint8_t s_zdt_req,s_zdt_active,s_zdt_addr,s_stop_req,s_zero_req,s_offset_req;
static int16_t s_zdt_args[3],s_zdt_rpm;
static uint8_t s_zdt_watch;
static uint32_t s_zdt_watch_tick;
static uint8_t ZDT_X42S_PopReply(uint8_t *p){(void)p;return 0;}
static uint32_t s_zdt_tick,s_zdt_duration,tick,stops,enables,speeds,last_addr,last_dir;
static void Debug_ZdtAck(uint8_t event){(void)event;}
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __enable_irq(void){}
static uint32_t HAL_GetTick(void){return tick;}
static void ZDT_X42S_Stop(uint8_t a){stops++;last_addr=a;}
static void ZDT_X42S_Enable(uint8_t a){enables++;last_addr=a;}
static void ZDT_X42S_SpeedAcc(uint8_t a,uint8_t d,uint16_t r,uint8_t c)
{speeds++;last_addr=a;last_dir=d;assert(r==50 && c==100);}
"""+extract("Debug_ParseManual")+extract("Debug_ServiceZdt")+r"""
int main(void){
 int16_t v[3];
 assert(Debug_ParseManual("3,-50,2",v)&&v[0]==3&&v[1]==-50&&v[2]==2);
 assert(!Debug_ParseManual("3,301,2",v)); assert(!Debug_ParseManual("3,50,2x",v));
 s_zdt_args[0]=3;s_zdt_args[1]=-50;s_zdt_args[2]=2;s_zdt_req=1;
 Debug_ServiceZdt();assert(stops==4&&enables==1&&speeds==0);
 tick=99;Debug_ServiceZdt();assert(speeds==0);
 tick=100;Debug_ServiceZdt();assert(speeds==1&&last_addr==3&&last_dir==1);
 tick=2099;Debug_ServiceZdt();assert(stops==4);
 tick=2100;Debug_ServiceZdt();assert(stops==5&&last_addr==3&&!s_zdt_active);
 s_zdt_args[0]=4;s_zdt_req=1;Debug_ServiceZdt();assert(enables==2);
 s_stop_req=1;Debug_ServiceZdt();assert(!s_zdt_active&&last_addr==4&&speeds==1);
 s_zdt_req=1;Debug_ServiceZdt();assert(!s_zdt_req&&enables==2);
 return 0;
}
"""
with tempfile.TemporaryDirectory() as d:
 src=Path(d)/"test.c"; exe=Path(d)/"test.exe";src.write_text(code,encoding="utf-8")
 subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-o",str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print("ZDT tests passed")
