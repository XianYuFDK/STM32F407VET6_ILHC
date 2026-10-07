"""真实OPS接收/发布/恢复函数：短错续行与坐标系变化、超时严格取消。"""
from pathlib import Path
import re
import subprocess
import tempfile
import test_ops_protocol as base

ROOT=base.ROOT
ops=base.OPS
rx=base.rx_prelude.replace('typedef struct {void *Instance;uint32_t ErrorCode;} UART_HandleTypeDef;',r'''
typedef int HAL_StatusTypeDef;
typedef struct {int state;} DMA_HandleTypeDef;
typedef struct {void *Instance;uint32_t ErrorCode;DMA_HandleTypeDef *hdmarx;int RxState;} UART_HandleTypeDef;
''').replace('static UART_HandleTypeDef huart2={USART2,0U};',r'''
static DMA_HandleTypeDef dma={0};
static UART_HandleTypeDef huart2={USART2,0U,&dma,0};
''')
rx=re.sub(r'static int OPS_RestartReceive\(void\)\{[^\n]+\}',r'''
#define HAL_DMA_STATE_READY 0
#define HAL_DMA_STATE_ABORT 1
#define HAL_UART_STATE_BUSY_RX 1
#define DMA_IT_HT 1
#define __HAL_DMA_DISABLE_IT(h,i) ((void)(h),(void)(i))
static int clear_calls,disabled_it,aborts;
#undef __HAL_UART_DISABLE_IT
#undef __HAL_UART_CLEAR_PEFLAG
#define __HAL_UART_DISABLE_IT(h,i) ((void)(h),disabled_it|=(i))
#define __HAL_UART_CLEAR_PEFLAG(h) ((void)(h),++clear_calls)
static int HAL_DMA_GetState(DMA_HandleTypeDef *d){return d->state;}
static int HAL_DMA_Abort(DMA_HandleTypeDef *d){d->state=0;++aborts;return HAL_OK;}
static int HAL_UART_AbortReceive(UART_HandleTypeDef *h){h->RxState=0;++aborts;return HAL_OK;}
static int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *buf,uint16_t size){
 (void)buf;(void)size;++restarts;
 if(restart_result==HAL_OK){h->RxState=1;h->ErrorCode=0;disabled_it=0;}
 return restart_result;
}
''',rx)


def func(name):
    if name=='OPS_RestartReceive':
        return base.function(ops.replace('static HAL_StatusTypeDef OPS_RestartReceive(',
                                        'static uint32_t OPS_RestartReceive('),name).replace(
                                            'static uint32_t OPS_RestartReceive(',
                                            'static HAL_StatusTypeDef OPS_RestartReceive(')
    return base.function(ops,name)


code=base.prelude+rx+base.queue_source+'\n'+base.crc8_match.group(0)+'\n'
code+='\n'.join(func(n) for n in base.functions+['OPS_IsOnline','OPS_RestartReceive','OPS_ServiceRx',
    'HAL_UARTEx_RxEventCallback','OPS_ProcessPending','HAL_UART_ErrorCallback'])
code=code.replace('  s_ops.frame_count++;','  s_ops.frame_count++; if(inject_rx_error){inject_rx_error=0;HAL_UART_ErrorCallback(&huart2);}')
code+=r'''
static void packet(uint16_t seq,uint32_t timestamp,uint32_t session,uint8_t flags){
 uint8_t b[28]={0x5d,1,28,0};uint16_t crc;float x=1,y=2,z=0;
 b[3]=flags;memcpy(b+4,&seq,2);memcpy(b+6,&session,4);memcpy(b+10,&timestamp,4);
 memcpy(b+14,&x,4);memcpy(b+18,&y,4);memcpy(b+22,&z,4);
 crc=OPS_CalcCRC16(b,26);memcpy(b+26,&crc,2);
 memcpy(s_rx_buf,b,28);HAL_UARTEx_RxEventCallback(&huart2,28);OPS_ProcessPending();
}
static void reset(void){
 memset(&s_ops,0,sizeof(s_ops));s_parse_len=0;s_continuity_lost=s_session_pending=0;
 s_rx_recover=s_transport_pending=0;s_rx_fault_epoch=0;
 s_fault_count=s_fault_read=s_fault_write=0;s_diagnostic_drops=0;
 restart_result=0;tick=1000;huart2.RxState=0;huart2.ErrorCode=0;dma.state=0;
 APP_RX_Reset(APP_RX_OPS);packet(100,5000,4,27);
 assert(OPS_IsOnline(200)&&!OPS_ConsumeSessionChanged());
}
static void error(uint32_t bits){huart2.ErrorCode=bits;HAL_UART_ErrorCallback(&huart2);}
static void has(uint32_t reason){
 OPS_Fault_t d;uint32_t found=0;
 while(OPS_PeekFault(&d)){found|=d.reason;OPS_FaultSent(d.id);}
 assert(found&reason);
}
int main(void){
 reset();tick=1001;error(4);
 assert(!OPS_IsOnline(200)&&OPS_RecoveryPending()&&!OPS_ConsumeSessionChanged());
 assert(clear_calls&&disabled_it==15);
 uint32_t id=s_fault_id;tick=1010;error(4);error(8);
 assert(s_fault_id==id&&s_transport_tick==1000&&s_transport_uart_error==12);
 OPS_ServiceRx();assert(!s_rx_recover&&!OPS_IsOnline(200));
 tick=1020;packet(101,5020,4,27);
 assert(!OPS_RecoveryPending()&&OPS_IsOnline(200)&&!OPS_ConsumeSessionChanged());
 has(OPS_FAULT_RECOVERED);
 /* 错误ISR抢占帧发布：该旧包不能宣布恢复。 */
 reset();tick=1010;huart2.ErrorCode=4;inject_rx_error=1;packet(101,5010,4,27);
 assert(OPS_RecoveryPending()&&!OPS_IsOnline(200));
 OPS_ServiceRx();tick=1020;packet(102,5020,4,27);assert(!OPS_RecoveryPending()&&OPS_IsOnline(200));
 /* 23:44日志的seq41849/session4/时间209250后短FE可恢复。 */
 reset();s_ops.frame.seq=s_ops.seq=41849;
 s_ops.frame.timestamp_ms=s_ops.timestamp_ms=209250;s_ops.last_update_tick=s_last_frame_tick=209956;
 tick=209957;error(4);OPS_ServiceRx();tick=209976;packet(41853,209270,4,27);
 assert(!OPS_RecoveryPending()&&OPS_IsOnline(200)&&!OPS_ConsumeSessionChanged());
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1020;packet(100,5000,4,27);
 assert(OPS_RecoveryPending()&&!OPS_IsOnline(200)); /* 重复帧不续期。 */
 tick=1030;packet(101,5030,4,9);assert(OPS_RecoveryPending()&&!OPS_IsOnline(200));
 tick=1040;packet(102,5040,4,27);assert(!OPS_RecoveryPending()&&OPS_IsOnline(200));
 /* 恢复成功边界是最后有效帧起200ms，不是每次错误之后200ms。 */
 reset();tick=1199;error(4);OPS_ServiceRx();tick=1200;packet(101,5200,4,27);
 assert(!OPS_RecoveryPending()&&OPS_IsOnline(200)&&!OPS_ConsumeSessionChanged());
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1190;error(4);OPS_ServiceRx();
 tick=1201;assert(!OPS_RecoveryPending()&&OPS_ConsumeSessionChanged()&&!OPS_IsOnline(200));
 has(OPS_FAULT_RECOVERY_TIMEOUT);
 /* 不能靠积压/错误CRC/同时间或逆序序号恢复。 */
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1050;packet(101,5000,4,27);
 assert(OPS_RecoveryPending()&&!OPS_IsOnline(200));
 tick=1060;packet(99,5060,4,27);assert(OPS_RecoveryPending()&&!OPS_IsOnline(200));
 tick=1070;packet(102,5070,4,27);assert(!OPS_RecoveryPending()&&OPS_IsOnline(200));
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1020;
 uint8_t broken[28]={0x5d,1,28,27}; /* 错误CRC的残缺定位不能续行。 */
 memcpy(s_rx_buf,broken,28);HAL_UARTEx_RxEventCallback(&huart2,28);OPS_ProcessPending();
 assert(OPS_RecoveryPending()&&!OPS_IsOnline(200)&&s_ops.crc_errors);
 tick=1040;packet(101,5040,4,27);assert(!OPS_RecoveryPending()&&OPS_IsOnline(200));
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1201;packet(101,5201,4,27);
 assert(OPS_ConsumeSessionChanged());
 /* 真会话/回退/IMU重置照常取消，不能当作短传输错误续行。 */
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1020;packet(101,5020,5,27);
 assert(!OPS_RecoveryPending()&&OPS_ConsumeSessionChanged());has(OPS_FAULT_SESSION_ID);
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1020;packet(101,4999,4,27);
 assert(!OPS_RecoveryPending()&&OPS_ConsumeSessionChanged());has(OPS_FAULT_TIMESTAMP_BACK);
 reset();tick=1001;error(4);OPS_ServiceRx();tick=1020;packet(101,5020,4,31);
 assert(!OPS_RecoveryPending()&&OPS_ConsumeSessionChanged()&&!OPS_IsOnline(200));has(OPS_FAULT_IMU_REBASED);
 /* 接收重挂失败不延长期限；先前的回调排队包被清空。 */
 reset();tick=1001; /* 任务恢复必须执行队列清理。 */
 uint8_t stale[28]={0};assert(APP_RX_Push(APP_RX_OPS,stale,28,1001));
 error(4);restart_result=HAL_ERROR;OPS_ServiceRx();assert(s_rx_recover&&!APP_RX_Pending());
 tick=1201;assert(OPS_ConsumeSessionChanged());
 reset();tick=1001;error(4);dma.state=HAL_DMA_STATE_ABORT;OPS_ServiceRx();
 assert(s_rx_recover&&OPS_RecoveryPending());tick=1201;assert(OPS_ConsumeSessionChanged());
 /* 模差正常处理seq与timestamp回绕。 */
 reset();s_ops.frame.seq=s_ops.seq=65535;s_ops.frame.timestamp_ms=s_ops.timestamp_ms=0xfffffff0U;
 tick=1001;error(4);OPS_ServiceRx();tick=1020;packet(0,4,4,27);
 assert(!OPS_RecoveryPending()&&OPS_IsOnline(200)&&!OPS_ConsumeSessionChanged());
 /* V1没有会话标识，保持保守取消。 */
 reset();s_ops.frame.header=OPS_FRAME_HEADER_V1;tick=1001;error(4);
 assert(!OPS_RecoveryPending()&&OPS_ConsumeSessionChanged());
 puts("OPS短错恢复：暂停/20ms续行/200ms期限/重复与无效帧/重挂失败/真会话回退重置/序号时间回绕通过");
 return 0;
}
'''

with tempfile.TemporaryDirectory(prefix='ilhc-ops-recovery-') as directory:
    folder=Path(directory);source=folder/'test.c';exe=folder/'test.exe'
    source.write_text(code,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(ROOT/'RTOS_APP'),
                    str(source),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
