"""Real USART1 sender -> PC parser, DMA ownership and 50 Hz reply load.

No serial device, IDE, flash, or motors are accessed.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root/'Hardware/debug_usart.c').read_text(encoding='utf-8')


def function(name):
    match = re.search(r'^(?:static )?(?:void|uint32_t) '+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}');end += 1
    return source[match.start():end]


code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#define DEBUG_VOFA_CHANNELS 24U
#define DEBUG_VOFA_TAIL0 0U
#define DEBUG_VOFA_TAIL1 0U
#define DEBUG_VOFA_TAIL2 0x80U
#define DEBUG_VOFA_TAIL3 0x7FU
#define DEBUG_REPLY_MAX 100U
#define DEBUG_CRC_FRAME_LEN 112U
#define DEBUG_ACK_PARAM_TEXT 20U
#define DEBUG_ACK_STEPPER 26U
#define DEBUG_ACK_DM_PARAM 27U
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_UART_STATE_READY 0
typedef struct {int gState;} UART_HandleTypeDef;
typedef struct {float position,velocity,torque;uint8_t status,tempMos,tempRotor;} DmJ4310Feedback_t;
typedef struct {float acc,dec;uint16_t seq,id;uint8_t status;} DmJ4310ParamResult_t;
typedef struct {uint8_t kind;} DebugStepperEvent_t;
typedef struct {uint8_t kind;} TrajReply_t;
static UART_HandleTypeDef huart1;
static uint8_t s_tx[212],s_traj_reply_last,s_telemetry_crc,s_zdt_text_mode;
static uint8_t s_ack_read,s_ack_write,s_ack_queue[16],s_ack_frames[16][4];
static char s_ack_param[16][24];
static DebugStepperEvent_t s_ack_stepper[16];
static DmJ4310ParamResult_t s_ack_dm_param[16];
static const char *s_ack_text[]={"ACK TEST\r\n"};
static DmJ4310Feedback_t s_dm_feedback;
static uint16_t s_dm_id=3;
static float pos_x,pos_y,zangle,devx,devy,devz,mKpx=2.3f,mKpy=2.3f,mKpz=9;
static float XYVmax=1600,ZVmax=750,s_dm_pos,s_dm_vel,s_dm_kp,s_dm_kd,s_dm_torque=INFINITY;
static int SpeedTarget[4];
static uint32_t s_telemetry_seq,debug_tx_errors,tick;
static int traj_pending,tx_result,capture,calls,telemetry_count;
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __enable_irq(void){}
static void MecanumControl_GetPose(float *x,float *y,float *yaw){*x=392;*y=1070;*yaw=-2.8f;}
static uint16_t Traj_PeekReply(char *out,uint16_t max,TrajReply_t *token){
 const char *text="TSTAT 17 4 4 1 10760 0\r\n";
 if(!traj_pending)return 0;
 assert(max==100);strcpy(out,text);token->kind=1;return (uint16_t)strlen(text);
}
static void Traj_ReplySent(const TrajReply_t *token){assert(token->kind==1&&traj_pending);traj_pending=0;}
static uint16_t Debug_FormatStepperEvent(char *out,uint16_t max,const DebugStepperEvent_t *event){
 (void)event;assert(max==100);memset(out,'A',98);out[98]='\r';out[99]='\n';return 100;
}
static int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *data,uint16_t len){
 assert(h==&huart1 && h->gState==HAL_UART_STATE_READY && len<=212);++calls;
 if(tx_result!=HAL_OK)return tx_result;
 if(len==100 && !memcmp(data+96,"\0\0\x80\x7f",4))++telemetry_count;
 if(s_telemetry_crc&&!s_zdt_text_mode){
   unsigned off=len-112;uint32_t seq,crc;
   assert(!memcmp(data+off,"\xa5\x5a\x01\x18",4));
   memcpy(&seq,data+off+4,4);assert(seq==s_telemetry_seq);
   memcpy(&crc,data+len-4,4);
   assert(crc==Debug_TelemetryCrc(data+off,108));
 }
 if(capture)assert(fwrite(data,1,len,stdout)==len);
 return HAL_OK;
}
'''
# Sender uses the actual checksum helper; forward declaration for HAL assertions.
code = code.replace('static int HAL_UART_Transmit_DMA',
                    'static uint32_t Debug_TelemetryCrc(const uint8_t *,uint32_t);\nstatic int HAL_UART_Transmit_DMA')
code += function('Debug_TelemetryCrc')+'\n'+function('DebugUsart_Send')
code += r'''
static void queue_param(void){
 unsigned slot=s_ack_write,next=(slot+1)%16;
 assert(next!=s_ack_read);s_ack_queue[slot]=DEBUG_ACK_PARAM_TEXT;
 strcpy(s_ack_param[slot],"XVMIN=5.000\r\n");s_ack_write=(uint8_t)next;
}
int main(void){
#ifdef _WIN32
 _setmode(_fileno(stdout),_O_BINARY);
#endif
 assert(Debug_TelemetryCrc((const uint8_t *)"123456789",9)==0xCBF43926U);
 /* Reproduce 19 Hz: 31 ordinary replies replace 31 of 50 legacy slots. */
 for(unsigned i=0;i<50;++i){tick=i*20;if(i<31)queue_param();DebugUsart_Send();}
 assert(calls==50 && telemetry_count==19 && s_ack_read==s_ack_write);
 calls=0;s_ack_read=s_ack_write=0;s_telemetry_crc=1;capture=1;
 /* Printable binary feedback is not a GET reply. */
 memcpy(&s_dm_feedback,"KPX=49.000\r\n",12);
 for(unsigned i=0;i<50;++i){
   tick=i*20;if(i<31)queue_param();if(i%10==0)traj_pending=1;DebugUsart_Send();
 }
 assert(calls==50 && s_telemetry_seq==50 && s_ack_read==s_ack_write && !traj_pending);
 capture=0;
 /* No in-flight DMA writes and no consuming replies on a failed submit. */
 queue_param();uint8_t saved[212];memcpy(saved,s_tx,212);
 unsigned read_before=s_ack_read;huart1.gState=1;DebugUsart_Send();
 assert(!memcmp(saved,s_tx,212)&&s_ack_read==read_before&&s_telemetry_seq==50);
 huart1.gState=0;tx_result=HAL_ERROR;DebugUsart_Send();
 assert(s_ack_read==read_before&&s_telemetry_seq==50&&debug_tx_errors==1);
 tx_result=HAL_OK;DebugUsart_Send();assert(s_ack_read!=read_before&&s_telemetry_seq==51);
 /* Max-length reply still fits 115200 8N1, legacy/text controls retained. */
 s_ack_queue[s_ack_write]=DEBUG_ACK_STEPPER;s_ack_write=(s_ack_write+1)%16;
 DebugUsart_Send();assert(s_telemetry_seq==52);
 assert(sizeof(s_tx)*10.0/115200 < .020);
 /* Persistent TSTAT must not starve queued parameter replies. */
 for(unsigned i=0;i<10;++i)queue_param();
 for(unsigned i=0;i<20;++i){traj_pending=1;DebugUsart_Send();}
 assert(s_ack_read==s_ack_write && s_telemetry_seq==72);
 traj_pending=0;
 s_zdt_text_mode=1;queue_param();DebugUsart_Send();assert(s_telemetry_seq==72);
 s_zdt_text_mode=0;s_telemetry_crc=0;DebugUsart_Send();assert(telemetry_count==20);
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='ilhc_telemetry_') as directory:
    src,exe=Path(directory)/'test.c',Path(directory)/'test.exe'
    src.write_text(code,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(src),'-o',str(exe)],check=True)
    stream=subprocess.check_output([str(exe)])

sys.path.insert(0,str(root/'HostTools/ILHC_Debugger/ILHC_Qt_v2'))
import core
for size in (1,3,17,112,2048,len(stream)):
    parser=core.FrameParser();frames=[]
    for start in range(0,len(stream),size):frames.extend(parser.feed(stream[start:start+size]))
    assert len(frames)==50,(size,len(frames))
    assert all(abs(f[0]-39.2)<.001 and abs(f[1]-107)<.001 and abs(f[2]+2.8)<.001 for f in frames)
    assert parser.take_params()==[('XVMIN',5.)]*31
    assert len(parser.take_text())==5
    assert parser.err_bytes==parser.crc_errors==parser.lost_packets==0
print('Real USART1 sender + PC parser passed: legacy 19 Hz reproduced; CRC1 50/50 frames + 31 GET + 5 TSTAT, all chunk sizes; DMA busy/failure/max reply/legacy/text verified.')
