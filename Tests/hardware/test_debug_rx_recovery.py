"""提取真实USART1恢复/接收函数，用HAL替身验证异常重试与TX隔离。"""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[2] / "Hardware/debug_usart.c").read_text(encoding="utf-8")

def function(name):
    # 匹配定义行，避免先匹配调用处或回调注册处。
    import re
    m = re.search(r"^(?:static )?void " + name + r"\([^;]*?\)\s*\{", source, re.M)
    assert m, name
    end, depth = m.end(), 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[m.start():end]

code = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_BUSY 2
#define HAL_UART_STATE_READY 0
#define HAL_UART_STATE_BUSY_TX 1
#define HAL_DMA_STATE_READY 0
#define HAL_DMA_STATE_BUSY 1
#define HAL_DMA_STATE_ABORT 2
#define HAL_UART_ERROR_CB_ID 1
#define DMA_IT_HT 1
#define DEBUG_LINE_SIZE 64U
#define USART1 ((void *)1)
typedef struct {int State;} DMA_HandleTypeDef;
typedef struct {void *Instance; int gState; DMA_HandleTypeDef *hdmarx;} UART_HandleTypeDef;
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef huart1 = {USART1, HAL_UART_STATE_READY, &dma};
static uint8_t s_rx[256], s_line[64], s_rx_callbacks_ready;
static volatile uint8_t s_rx_recover;
static uint16_t s_line_len;
static uint32_t s_host_last_tick, tick, mask;
static int start_result, abort_result, dma_abort_result, register_result;
static int start_calls, abort_calls, dma_abort_calls, ht_disabled, clear_calls, registered;
static int parse_calls, inject_error;
static char parsed[64];
static void DebugUsart_ErrorCallback(UART_HandleTypeDef *);
void DebugUsart_RxEventCallback(UART_HandleTypeDef *, uint16_t);
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __enable_irq(void) {
 mask=0;
 if(inject_error){inject_error=0;DebugUsart_ErrorCallback(&huart1);}
}
static uint32_t HAL_GetTick(void) {return tick;}
static int HAL_UART_RegisterRxEventCallback(UART_HandleTypeDef *h,void (*cb)(UART_HandleTypeDef *,uint16_t))
{assert(h==&huart1 && cb==DebugUsart_RxEventCallback);++registered;return register_result;}
static int HAL_UART_RegisterCallback(UART_HandleTypeDef *h,int id,void (*cb)(UART_HandleTypeDef *))
{assert(h==&huart1 && id==HAL_UART_ERROR_CB_ID && cb==DebugUsart_ErrorCallback);++registered;return register_result;}
static int HAL_DMA_GetState(DMA_HandleTypeDef *h) {return h->State;}
static int HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{assert(!mask && h==&huart1);++abort_calls;return abort_result;}
static int HAL_DMA_Abort(DMA_HandleTypeDef *h)
{assert(!mask && h==&dma);++dma_abort_calls;if(dma_abort_result==HAL_OK)h->State=HAL_DMA_STATE_READY;return dma_abort_result;}
static int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t size)
{assert(mask && h==&huart1 && p==s_rx && size==256);++start_calls;return start_result;}
#define __HAL_UART_CLEAR_OREFLAG(h) (++clear_calls)
#define __HAL_DMA_DISABLE_IT(h,it) (++ht_disabled)
static void Debug_ParseLine(char *line) {++parse_calls;strcpy(parsed,line);}
'''
for name in ("DebugUsart_ErrorCallback", "DebugUsart_StartRx", "DebugUsart_ServiceRx", "DebugUsart_RxEventCallback"):
    code += function(name) + "\n"
code += r'''
static void receive(const char *s){memcpy(s_rx,s,strlen(s));DebugUsart_RxEventCallback(&huart1,(uint16_t)strlen(s));}
int main(void) {
 int before;
 UART_HandleTypeDef other={(void *)2,0,&dma};
 /* 注册失败、首次DMA启动失败均保留恢复请求，并在后续周期恢复。 */
 s_rx_recover=1;register_result=HAL_ERROR;DebugUsart_ServiceRx();assert(s_rx_recover&&!s_rx_callbacks_ready&&!start_calls);
 register_result=HAL_OK;start_result=HAL_ERROR;DebugUsart_ServiceRx();assert(s_rx_recover&&s_rx_callbacks_ready&&start_calls==1);
 start_result=HAL_OK;DebugUsart_ServiceRx();assert(!s_rx_recover&&start_calls==2&&ht_disabled==1);
 /* 正常分段命令可解析，错误丢弃半条命令，不刷新心跳。 */
 tick=42;receive("ZD");assert(s_line_len==2);
 DebugUsart_ErrorCallback(&other);assert(!s_rx_recover);
 DebugUsart_ErrorCallback(&huart1);assert(s_rx_recover);
 receive("T=4,50,2\n");assert(!parse_calls&&s_host_last_tick==0);
 DebugUsart_ServiceRx();assert(!s_rx_recover&&s_line_len==0&&s_host_last_tick==0);
 receive("STOP\r\n");assert(parse_calls==1&&!strcmp(parsed,"STOP")&&s_host_last_tick==42);
 /* 接收回调重启失败由任务恢复，恢复不改变正在发送的TX状态。 */
 start_result=HAL_BUSY;receive("PING\n");assert(s_rx_recover);
 start_result=HAL_OK;huart1.gState=HAL_UART_STATE_BUSY_TX;DebugUsart_ServiceRx();assert(!s_rx_recover&&huart1.gState==HAL_UART_STATE_BUSY_TX);
 /* DMA异步终止期间等待，不重复终止；失败必须重试。 */
 s_rx_recover=1;dma.State=HAL_DMA_STATE_ABORT;before=abort_calls;DebugUsart_ServiceRx();assert(s_rx_recover&&abort_calls==before);
 dma.State=HAL_DMA_STATE_BUSY;dma_abort_result=HAL_ERROR;before=start_calls;DebugUsart_ServiceRx();assert(s_rx_recover&&start_calls==before);
 dma_abort_result=HAL_OK;DebugUsart_ServiceRx();assert(!s_rx_recover&&dma_abort_calls==2);
 s_rx_recover=1;abort_result=HAL_ERROR;before=start_calls;DebugUsart_ServiceRx();assert(s_rx_recover&&start_calls==before);
 abort_result=HAL_OK;DebugUsart_ServiceRx();assert(!s_rx_recover);
 /* 注册时TX忙不打断TX，下一周期空闲再注册。 */
 s_rx_callbacks_ready=0;s_rx_recover=1;before=registered;DebugUsart_ServiceRx();assert(s_rx_recover&&registered==before);
 huart1.gState=HAL_UART_STATE_READY;DebugUsart_ServiceRx();assert(!s_rx_recover&&s_rx_callbacks_ready);
 /* 新错误发生在重启后，不会被恢复代码覆盖；原有中断屏蔽状态也保留。 */
 inject_error=1;DebugUsart_StartRx();assert(s_rx_recover);
 DebugUsart_ServiceRx();assert(!s_rx_recover);
 mask=1;DebugUsart_StartRx();assert(mask==1);mask=0;
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="ilhc_rx_recovery_") as d:
    src, exe = Path(d) / "test.c", Path(d) / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("USART1 RX recovery tests passed")
