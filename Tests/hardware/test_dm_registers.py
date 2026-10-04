"""编译真实DM驱动和调试桥接，验证梯形参数事务，不连接硬件。"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT/'Hardware/debug_usart.c').read_text(encoding='utf-8')


def function(name):
    start = source.rfind('static ', 0, source.index(name+'('))
    brace = source.index('{', start)
    end, depth = brace+1, 1
    while depth:
        depth += (source[end] == '{')-(source[end] == '}')
        end += 1
    return source[start:end]


prelude = r'''
#include "dm_j4310.h"
#include "hcan.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
CAN_HandleTypeDef hcan2 = {CAN2, HAL_CAN_STATE_LISTENING};
uint32_t test_primask;
static uint32_t tick, sent;
static HAL_StatusTypeDef tx_status = HAL_OK;
static uint8_t bytes[32][8];
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *h,uint16_t id,const uint8_t *data,uint16_t len)
{(void)h;assert(id==0x7FF && len==8);if(tx_status==HAL_OK)memcpy(bytes[sent++%32],data,8);return tx_status;}
#define DEBUG_ACK_DM_PARAM 27U
#define DEBUG_HOST_TIMEOUT_MS 1000U
static uint8_t s_ack_write, s_ack_read, s_ack_queue[16];
static DmJ4310ParamResult_t s_ack_dm_param[16];
static uint8_t s_dm_active, s_dm_start_pending, s_dm_enable_req, s_dm_mode_req;
static uint8_t s_dm_zero_req, s_dm_disable_req, s_dm_disable_pending, s_dm_disable_fault, s_stop_req;
static uint32_t s_host_last_tick, s_dm_feedback_tick;
static uint16_t s_dm_id=3;
static DmJ4310Feedback_t s_dm_feedback;
'''

checks = r'''
static void step(void){tick+=20;s_host_last_tick=tick;Debug_ServiceDmParams();}
static void reply(uint8_t rid,float value,uint8_t command,uint16_t id){
 uint8_t data[8]={0}; data[0]=(uint8_t)id;data[1]=(uint8_t)(id>>8);
 data[2]=command;data[3]=rid;memcpy(data+4,&value,4);DmJ4310_ParamOnRx(data);
}
static void fresh(void){tick+=600;s_host_last_tick=s_dm_feedback_tick=tick;
 s_dm_feedback.valid=1;s_dm_feedback.status=0;s_ack_read=s_ack_write;sent=0;}
static DmJ4310ParamResult_t last(void){return s_ack_dm_param[(s_ack_write+15)%16];}
int main(void){
 /* 中断解析不提交CAN；任务逐项写，再发显式读。 */
 fresh();assert(Debug_ParseDmParams("DMACCDEC=10,0.002,-0.003"));assert(sent==0);
 step();assert(sent==1 && bytes[0][2]==0x55 && bytes[0][3]==4);
 {float a;memcpy(&a,bytes[0]+4,4);assert(fabsf(a-.002f)<1e-8f);}
 step();assert(bytes[1][2]==0x55 && bytes[1][3]==5);
 step();assert(bytes[2][2]==0x33 && bytes[2][3]==4);
 /* 写回显、错误节点、错误RID不能推进事务。 */
 reply(4,.002f,0x55,3);reply(4,.002f,0x33,4);reply(5,-.003f,0x33,3);
 step();assert(sent==3 && DmJ4310_ParamBusy());
 reply(4,.002f,0x33,3);step();step();assert(bytes[3][3]==5);
 reply(5,-.003f,0x33,3);step();assert(last().status==0 && last().seq==10);
 assert(fabsf(last().acc-.002f)<1e-8f && fabsf(last().dec+.003f)<1e-8f);
 /* 活动、使能等待、反馈过期/故障均拒绝，格式、符号、NaN拒绝。 */
 fresh();s_dm_active=1;Debug_ParseDmParams("DMREAD=11");assert(last().status==1);s_dm_active=0;
 s_dm_enable_req=1;Debug_ParseDmParams("DMREAD=11");assert(!DmJ4310_ParamBusy());s_dm_enable_req=0;
 s_dm_feedback_tick=tick-301;Debug_ParseDmParams("DMREAD=11");assert(!DmJ4310_ParamBusy());
 s_dm_feedback_tick=tick;s_dm_feedback.status=1;Debug_ParseDmParams("DMREAD=11");assert(!DmJ4310_ParamBusy());s_dm_feedback.status=0;
 Debug_ParseDmParams("DMACCDEC=12,nan,-2");assert(last().status==1);
 Debug_ParseDmParams("DMACCDEC=12");assert(last().status==1);
 Debug_ParseDmParams("DMACCDEC=12,2");assert(last().status==1);
 Debug_ParseDmParams("DMACCDEC=12,2,2");assert(last().status==1);
 Debug_ParseDmParams("DMACCDEC=12,2,-2,3");assert(last().status==1);
 Debug_ParseDmParams("DMREAD=65536");assert(!DmJ4310_ParamBusy());
 /* 邮箱BUSY重试，无回包超时；隔离窗口不接受立即重发。 */
 fresh();tx_status=HAL_BUSY;Debug_ParseDmParams("DMREAD=13");step();assert(!sent);
 tx_status=HAL_OK;step();assert(sent==1);
 tick+=400;s_host_last_tick=tick;Debug_ServiceDmParams();assert(last().status==3);
 s_dm_feedback_tick=tick;Debug_ParseDmParams("DMREAD=14");assert(last().status==1);
 /* 发送失败、取消、失联都不会被当成功。 */
 fresh();tx_status=HAL_ERROR;Debug_ParseDmParams("DMREAD=15");step();assert(last().status==2);
 tx_status=HAL_OK;fresh();Debug_ParseDmParams("DMREAD=16");s_stop_req=1;step();assert(last().status==5);s_stop_req=0;
 fresh();Debug_ParseDmParams("DMREAD=17");tick+=1001;Debug_ServiceDmParams();assert(last().status==5);
 /* 回读不一致/非法浮点均失败；队列满保留结果，PRIMASK原样恢复。 */
 fresh();Debug_ParseDmParams("DMACCDEC=18,2,-2");step();step();step();
 reply(4,3,0x33,3);step();assert(last().status==4);
 fresh();Debug_ParseDmParams("DMREAD=19");step();reply(4,NAN,0x33,3);step();assert(last().status==6);
 fresh();Debug_ParseDmParams("DMREAD=20");step();reply(4,2,0x33,3);step();step();reply(5,-2,0x33,3);
 s_ack_read=(s_ack_write+1)%16;test_primask=1;step();assert(DmJ4310_ParamBusy() && test_primask==1);
 s_ack_read=s_ack_write;step();assert(!DmJ4310_ParamBusy() && last().status==0 && test_primask==1);
 /* tick回绕时仍按无符号时差计时，不把隔离期变成长期拒绝。 */
 test_primask=0;tick=0xfffffff0U;fresh();Debug_ParseDmParams("DMREAD=21");
 step();assert(sent==1);reply(4,2,0x33,3);step();step();reply(5,-2,0x33,3);step();assert(last().status==0);
 puts("DM register CAN packing, read verification, cancellation and bridge gates passed");return 0;
}
'''

with tempfile.TemporaryDirectory(prefix='ilhc_dm_regs_') as directory:
    folder = Path(directory)
    src, exe = folder/'dm_regs.c', folder/'dm_regs.exe'
    names = ['Debug_DmParamReply', 'Debug_ServiceDmParams', 'Debug_ParseFloat',
             'Debug_StrCaseCmpN', 'Debug_ParseDmParams']
    src.write_text(prelude+'\n'.join(function(name) for name in names)+checks, encoding='utf-8')
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(ROOT/'Tests/hardware'), '-I'+str(ROOT/'Hardware'),
                    str(src), str(ROOT/'Hardware/dm_j4310.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

assert 'Debug_ParseDmParams(line)' in source
assert 'if (hcanRxFrame.StdId != 0x7FFU) DmJ4310_ParamOnRx(rxData);' in source
assert source.index('Debug_ServiceDmParams();') < source.index('  Debug_ServiceDm();')
