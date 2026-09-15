"""验证真实CAN失败分支及命令隔离函数，不连接设备。"""
from pathlib import Path
import subprocess, tempfile
r=Path(__file__).resolve().parents[2]
s=(r/"Hardware/debug_usart.c").read_text(encoding="utf-8")
m=(r/"Core/Src/main.c").read_text(encoding="utf-8")
def block(text,start):
    a=text.index(start); b=text.index("{",a); n=1; e=b+1
    while n:
        n+=(text[e]=="{")-(text[e]=="}"); e+=1
    return text[a:e]
f=block(s,"static uint8_t Debug_RejectCanCommand(")
b=block(m,"if (CAN_Start(&hcan1) != HAL_OK)")
assert "Error_Handler" not in b
code=r'''
#include <stdint.h>
#include <assert.h>
#include <ctype.h>
#define HAL_OK 0
#define HAL_CAN_STATE_ERROR 5
#define HAL_CAN_STATE_LISTENING 2
typedef struct {unsigned IER;} Registers;
static Registers regs;
static struct {unsigned State, ErrorCode; Registers *Instance;} hcan1;
static unsigned start_result, event, count;
static uint8_t s_zdt_text_mode;
static int CAN_Start(void *p){(void)p;return start_result;}
static void Debug_ZdtAck(uint8_t e){event=e;count++;}
static uint8_t Debug_StrCaseCmpN(const char*a,const char*b,unsigned n){while(n--){if(toupper((unsigned char)*a)!=toupper((unsigned char)*b))return 1;if(!*a)return 0;a++;b++;}return 0;}
'''+f+"\nstatic void startup(void){"+b+"}\n"+r'''
int main(void){
 const char *bad[]={"DMEN","dmoff","DMPOS=1","S28HOME","s35RAW=0,1,1"};
 const char *good[]={"led on","LED OFF","STOP","ZDT=4,50,2","VOFA","PING"};
 hcan1.Instance=&regs;hcan1.ErrorCode=0x20000;regs.IER=2;
 start_result=1;startup();assert(hcan1.State==5 && regs.IER==0 && hcan1.ErrorCode==0x20000);
 for(unsigned i=0;i<5;i++){assert(Debug_RejectCanCommand(bad[i]));assert(event==9 && s_zdt_text_mode);}
 unsigned before=count;
 for(unsigned i=0;i<6;i++)assert(!Debug_RejectCanCommand(good[i]));
 assert(count==before);
 hcan1.State=2;regs.IER=2;start_result=0;startup();assert(hcan1.State==2 && regs.IER==2);
 for(unsigned i=0;i<5;i++)assert(!Debug_RejectCanCommand(bad[i]));
 return 0;
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"test.c"; x=Path(d)/"test.exe";p.write_text(code,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(p),"-o",str(x)],check=True)
    subprocess.run([str(x)],check=True)
print("CAN degraded startup and command isolation passed")
