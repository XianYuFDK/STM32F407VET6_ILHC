"""主机模拟HAL，验证真实UART4应答解析与错误恢复，不连接电机。"""
from pathlib import Path
import subprocess, tempfile
source=(Path(__file__).resolve().parents[2]/"Hardware/zdt_x42s.c").read_text(encoding="utf-8")
code=r"""
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef int HAL_StatusTypeDef;
typedef struct {int dummy;} UART_HandleTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_UART_RX_COMPLETE_CB_ID 0
#define HAL_UART_ERROR_CB_ID 1
static UART_HandleTypeDef huart4;
#define ZDT_X42S_UART huart4
static uint32_t tick,mask,restarts,aborts;
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __enable_irq(void){mask=0;}
static int HAL_UART_RegisterCallback(UART_HandleTypeDef *u,int id,void (*cb)(UART_HandleTypeDef*))
{(void)u;(void)id;(void)cb;return HAL_OK;}
static int HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n)
{(void)u;(void)b;assert(n==1);restarts++;return HAL_OK;}
static int HAL_UART_AbortReceive(UART_HandleTypeDef *u){(void)u;aborts++;return HAL_OK;}
#define __HAL_UART_CLEAR_OREFLAG(u) ((void)(u))
"""+source[source.index("/* UART4应答接收："):]+r"""
static void feed(uint8_t b){s_rx_byte=b;ZDT_X42S_RxComplete(&huart4);}
static void frame(uint8_t a,uint8_t f,uint8_t st){feed(a);feed(f);feed(st);feed(0x6b);}
int main(void){
 uint8_t b[4];int i;
 assert(ZDT_X42S_InitRx()==HAL_OK);
 feed(0x99);frame(4,0xf3,2);assert(ZDT_X42S_PopReply(b)&&b[0]==4&&b[1]==0xf3&&b[2]==2);
 assert(!ZDT_X42S_PopReply(b));
 frame(3,0xf6,0xe2);frame(4,0xfe,0xee);
 assert(ZDT_X42S_PopReply(b)&&b[2]==0xe2);assert(ZDT_X42S_PopReply(b)&&b[2]==0xee);
 feed(4);feed(0xf3);tick+=21;feed(2);feed(0x6b);assert(!ZDT_X42S_PopReply(b));
 frame(4,0xf6,2);assert(ZDT_X42S_PopReply(b));
 for(i=0;i<20;i++)frame(4,0xfe,2);
 for(i=0;i<15;i++)assert(ZDT_X42S_PopReply(b));
 assert(!ZDT_X42S_PopReply(b));assert(!ZDT_X42S_PopReply(NULL));
 ZDT_X42S_RxError(&huart4);ZDT_X42S_ServiceRx();assert(aborts==1&&!s_rx_fault);
 frame(4,0xf3,2);assert(ZDT_X42S_PopReply(b));return 0;
}
"""
with tempfile.TemporaryDirectory() as d:
 src=Path(d)/"test.c";exe=Path(d)/"test.exe";src.write_text(code,encoding="utf-8")
 subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-o",str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print("ZDT RX tests passed")
