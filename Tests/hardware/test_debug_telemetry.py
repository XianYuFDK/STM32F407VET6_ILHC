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
ops_header=(root/'Hardware/ops.h').read_text(encoding='utf-8')
ops_types=ops_header[ops_header.index('#define OPS_FAULT_SESSION_ID'):ops_header.index('/* ---------------------------- 对外接口')]


def function(name):
    match = re.search(r'^(?:static )?(?:void|uint16_t|uint32_t) '+name+r'\([^;]*?\)\s*\{',source,re.M)
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
static uint32_t s_wire_credit,s_wire_tick,s_full_tick,wire_bytes;
static uint8_t s_wire_started,s_full_sent;
static int traj_pending,tx_result,capture,calls,telemetry_count;
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __enable_irq(void){}
static void MecanumControl_GetPose(float *x,float *y,float *yaw){*x=392;*y=1070;*yaw=-2.8f;}
static uint16_t Traj_PeekReply(char *out,uint16_t max,TrajReply_t *token){
 const char *text="TSTAT 17 4 4 1 10760 0\r\n";
 if(traj_pending==2)text="CCTRL 17 2 12 9000 130 800 -100 3056 2500 0\r\n";
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
 if(s_telemetry_crc==1&&!s_zdt_text_mode){
   unsigned off=len-112;uint32_t seq,crc;
   assert(!memcmp(data+off,"\xa5\x5a\x01\x18",4));
   memcpy(&seq,data+off+4,4);assert(seq==s_telemetry_seq);
   memcpy(&crc,data+len-4,4);
   assert(crc==Debug_TelemetryCrc(data+off,108));
 }
 if(s_telemetry_crc==2&&!s_zdt_text_mode){
   unsigned off=0;uint32_t crc;
   while(off+4<len && memcmp(data+off,"\xa5\x5a\x02",3))++off;
   assert(off+4<len && (data[off+3]==3||data[off+3]==24));
   assert(len-off==16U+4U*data[off+3]);
   memcpy(&crc,data+len-4,4);assert(crc==Debug_TelemetryCrc(data+off,len-off-4));
 }
 wire_bytes+=len;
 if(capture)assert(fwrite(data,1,len,stdout)==len);
 return HAL_OK;
}
'''
# Sender uses the actual checksum helper; forward declaration for HAL assertions.
code = code.replace('static int HAL_UART_Transmit_DMA',
                    'static uint32_t Debug_TelemetryCrc(const uint8_t *,uint32_t);\nstatic int HAL_UART_Transmit_DMA')
code += '\n'+ops_types+r'''
static uint8_t s_ops_fault_part,s_ops_snapshot_part,s_ops_fault_defer;
static uint32_t s_ops_fault_token,s_ops_snapshot_tick;
static OPS_Diagnostics_t s_ops_snapshot;
static OPS_Fault_t fault;static uint8_t fault_pending,fault_backlog;
uint8_t OPS_PeekFault(OPS_Fault_t *out){if(fault_pending)*out=fault;return fault_pending;}
void OPS_FaultSent(uint32_t id){assert(id==fault.id&&fault_pending);fault_pending=fault_backlog;if(fault_backlog)fault.id++;}
void OPS_GetDiagnostics(OPS_Diagnostics_t *out){memset(out,0,sizeof(*out));out->tick=tick;out->flags=11;out->pose_valid=1;}
'''
code += function('Debug_TelemetryCrc')+'\n'+function('Debug_OpsFaultReply')+'\n'+function('Debug_OpsSnapshotReply')+'\n'+function('DebugUsart_Send')
code += r'''
static void queue_param(void){
 unsigned slot=s_ack_write,next=(slot+1)%16;
 assert(next!=s_ack_read);s_ack_queue[slot]=DEBUG_ACK_PARAM_TEXT;
 strcpy(s_ack_param[slot],"XVMIN=5.000\r\n");s_ack_write=(uint8_t)next;
}
int main(int argc,char **argv){
#ifdef _WIN32
 _setmode(_fileno(stdout),_O_BINARY);
#endif
 assert(Debug_TelemetryCrc((const uint8_t *)"123456789",9)==0xCBF43926U);
 if(argc>1 && (!strcmp(argv[1],"fault") || !strcmp(argv[1],"fault-burst"))) {
   /* 冻结诊断两段必须先于取消状态；忙/失败不消费。极值文本不溢出212B。 */
   memset(&fault,0xff,sizeof(fault));fault.seq=65535;fault.flags=31;fault.reason=512;
   fault_pending=1;traj_pending=1;s_telemetry_crc=1;
   fault_backlog=!strcmp(argv[1],"fault-burst");
   huart1.gState=1;DebugUsart_Send();assert(fault_pending&&!s_ops_fault_part);
   huart1.gState=0;tx_result=HAL_ERROR;DebugUsart_Send();assert(fault_pending&&!s_ops_fault_part&&!s_telemetry_seq);
   tx_result=HAL_OK;capture=1;DebugUsart_Send();assert(fault_pending&&s_ops_fault_part&&traj_pending);
   tick=20;DebugUsart_Send();assert(fault_pending==fault_backlog&&!s_ops_fault_part&&traj_pending);
   tick=40;DebugUsart_Send();assert(!traj_pending&&s_telemetry_seq==3);
   return 0;
 }
 if(argc>1){
   (void)argv;s_telemetry_crc=2;capture=1;
   /* 回读与状态同时存在，发送预算包含回复；定位不得被上传/诊断替代。 */
   for(unsigned i=0;i<500;++i){
     tick=i*20;if(i%5==0)queue_param();if(i%5==0)traj_pending=i%10==0?1:2;
     DebugUsart_Send();assert(wire_bytes<=212+(tick*2400)/1000);
   }
   assert(s_telemetry_seq>=490 && s_ack_read==s_ack_write);
   /* 忙/失败仍不消费回复、信用、序号。 */
   capture=0;queue_param();unsigned credit=s_wire_credit,seq=s_telemetry_seq,read=s_ack_read;
   huart1.gState=1;DebugUsart_Send();assert(s_wire_credit==credit&&s_telemetry_seq==seq&&s_ack_read==read);
   huart1.gState=0;tick+=20;tx_result=HAL_ERROR;DebugUsart_Send();
   assert(s_telemetry_seq==seq&&s_ack_read==read&&debug_tx_errors==1);
   tx_result=HAL_OK;DebugUsart_Send();assert(s_telemetry_seq==seq+1);
   return 0;
 }
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
    wireless_stream=subprocess.check_output([str(exe),'wireless'])
    fault_stream=subprocess.check_output([str(exe),'fault'])
    burst_stream=subprocess.check_output([str(exe),'fault-burst'])

sys.path.insert(0,str(root/'HostTools/ILHC_Debugger/ILHC_Qt_v2'))
import core
from ops_diagnostics import parse_ops_text
for size in (1,3,17,112,len(fault_stream)):
    parser=core.FrameParser();frames=[]
    for start in range(0,len(fault_stream),size):frames.extend(parser.feed(fault_stream[start:start+size]))
    texts=parser.take_text();assert [t.split()[0] for t in texts]==['OPSE','OPSX','TSTAT']
    assert len(frames)==3 and all(len(t)<100 for t in texts)
    assert parse_ops_text(texts[0])['causes']==['USART2硬件错误']
    assert parse_ops_text(texts[1])['uart_error']==0xffffffff
print('OPS真实C发送：极值文本、DMA忙/失败留存、OPSE/OPSX先于TSTAT、分片PC解析通过。')
parser=core.FrameParser();parser.feed(burst_stream)
assert [t.split()[0] for t in parser.take_text()]==['OPSE','OPSX','TSTAT']
print('OPS连续故障队列仍及时交付TSTAT，不饿死取消回复。')
for size in (1,3,17,112,2048,len(stream)):
    parser=core.FrameParser();frames=[]
    for start in range(0,len(stream),size):frames.extend(parser.feed(stream[start:start+size]))
    assert len(frames)==50,(size,len(frames))
    assert all(abs(f[0]-39.2)<.001 and abs(f[1]-107)<.001 and abs(f[2]+2.8)<.001 for f in frames)
    assert parser.take_params()==[('XVMIN',5.)]*31
    assert len(parser.take_text())==5
    assert parser.err_bytes==parser.crc_errors==parser.lost_packets==0
print('Real USART1 sender + PC parser passed: legacy 19 Hz reproduced; CRC1 50/50 frames + 31 GET + 5 TSTAT, all chunk sizes; DMA busy/failure/max reply/legacy/text verified.')
for size in (1,3,17,28,112,2048,len(wireless_stream)):
    parser=core.FrameParser();frames=[];channels=[];params=[];texts=[]
    for start in range(0,len(wireless_stream),size):
        frames.extend(parser.feed(wireless_stream[start:start+size]));channels.extend(parser.frame_channels)
        params.extend(parser.take_params());texts.extend(parser.take_text())
    assert len(frames)>=490,(size,len(frames))
    assert 45<=channels.count(24)<=50
    assert parser.protocol=='CRC2' and parser.crc_errors==parser.lost_packets==0
    # 解析器参数缓存本来有64项上限，按工作线程实际方式及时消费。
    expected_params=64 if size==len(wireless_stream) else 100
    assert params==[('XVMIN',5.)]*expected_params
    assert len([t for t in texts if t.startswith(('TSTAT ','CCTRL '))])==100
    assert len([t for t in texts if t.startswith(('OPSD ','OPSR '))])==8
print('DL-20真实C发送/解析: %.1fHz位置、%.1fHz完整状态，%.1fB/s含100GET+50TSTAT+50CCTRL；2400B/s预算及DMA失败重试通过。'%
      (len(frames)/10,channels.count(24)/10,len(wireless_stream)/10))
