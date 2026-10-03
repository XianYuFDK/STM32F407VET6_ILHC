"""Compile real X driver/debug functions and verify CAN packets, errors and reply routing."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'Hardware/debug_usart.c').read_text(encoding='utf-8')

def function(name):
    start = source.rfind('static ', 0, source.index(name + '('))
    brace = source.index('{', start)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

header = r'''
#ifndef TEST_HCAN_H
#define TEST_HCAN_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY} HAL_StatusTypeDef;
typedef struct {uint32_t State;} CAN_HandleTypeDef;
#define HAL_CAN_STATE_LISTENING 1U
extern CAN_HandleTypeDef hcan2;
#define HCAN_CAN_NUM (&hcan2)
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_SendEXData(CAN_HandleTypeDef *hcan,uint32_t id,const uint8_t *data,uint16_t len);
HAL_StatusTypeDef Can_SendCmd(uint32_t id,const uint8_t *data,uint8_t len);
#endif
'''
prelude = r'''
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "stepper_2835.h"
#define DEBUG_HOST_TIMEOUT_MS 1000U
#define DEBUG_ACK_STEPPER 26U
typedef struct {uint32_t position,queued_tick;uint16_t speed;uint8_t direction,action;} DebugStepperRequest_t;
typedef struct {uint8_t index,kind,command,length,data[8];} DebugStepperEvent_t;
static volatile DebugStepperRequest_t s_stepper_req[2];
static DebugStepperEvent_t s_ack_stepper[16];
static uint8_t s_ack_read,s_ack_write,s_ack_queue[16],s_stepper_watch_cmd[2];
static uint32_t s_stepper_seen[2],s_stepper_watch_tick[2];
static uint32_t tick,mask,calls,free_mailboxes=3;
static HAL_StatusTypeDef result=HAL_OK;
CAN_HandleTypeDef hcan2={HAL_CAN_STATE_LISTENING};
static struct {uint32_t id;uint16_t len;uint8_t data[8];} sent[128];
uint32_t HAL_GetTick(void) {return tick;}
uint32_t __get_PRIMASK(void) {return mask;}
void __disable_irq(void) {mask=1;}
void __enable_irq(void) {mask=0;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan) {(void)hcan;return free_mailboxes;}
HAL_StatusTypeDef CAN_SendEXData(CAN_HandleTypeDef *hcan,uint32_t id,const uint8_t *data,uint16_t len)
{
 (void)hcan;
 if(result!=HAL_OK) return result;
 assert(calls<128 && len<=8);
 sent[calls].id=id;sent[calls].len=len;memcpy(sent[calls].data,data,len);++calls;
 return HAL_OK;
}
HAL_StatusTypeDef Can_SendCmd(uint32_t id,const uint8_t *data,uint8_t len)
{
 uint8_t offset=0,packet=0;
 while(offset<len) {
  uint8_t size=len-offset<8?len-offset:8;
  HAL_StatusTypeDef status=CAN_SendEXData(&hcan2,id+packet,data+offset,size);
  if(status!=HAL_OK) return status;
  offset+=size;++packet;
 }
 return HAL_OK;
}
'''
checks = r'''
static char last_message[100];
static void drain(void) {
 while(s_ack_read!=s_ack_write) {
  assert(s_ack_queue[s_ack_read]==DEBUG_ACK_STEPPER);
  assert(Debug_FormatStepperEvent(last_message,sizeof(last_message),&s_ack_stepper[s_ack_read])>0);
  s_ack_read=(s_ack_read+1)%16;
 }
}
static void reply(uint16_t id,uint8_t code,uint8_t status) {
 const uint8_t data[3]={code,status,0x6B};
 assert(Stepper2835_OnRx(id,data,3));
 Debug_ServiceStepperReplies();drain();
}
int main(void) {
 uint32_t n;
 assert(Debug_ParseStepper("s35move=1000,50"));
 assert(calls==0);drain();assert(strstr(last_message,"QUEUED"));
 Debug_ServiceSteppers();drain();
 assert(calls==2 && sent[0].id==0x100 && sent[1].id==0x101);
 /* 50mm/s -> 1500RPM -> raw15000; position raw46288(0xB4D0). */
 assert(sent[0].len==8 && sent[0].data[0]==0xFD && sent[0].data[6]==0x3A && sent[0].data[7]==0x98);
 assert(sent[1].len==8 && sent[1].data[0]==0xFD && sent[1].data[3]==0xB4 && sent[1].data[4]==0xD0);
 assert(sent[1].data[5]==1 && sent[1].data[6]==0 && sent[1].data[7]==0x6B);
 assert(strstr(last_message,"CAN_SUBMITTED"));
 /* Do not interleave two FD commands while awaiting a matching motor reply. */
 Debug_ParseStepper("S35RAW=1,4294967295,3000");Debug_ServiceSteppers();assert(calls==2);
 reply(0x100,0xF3,2);assert(s_stepper_watch_cmd[0]==0xFD);
 reply(0x100,0xFD,2);assert(!s_stepper_watch_cmd[0]);
 mask=1;Debug_ServiceSteppers();drain();
 assert(mask==1 && calls==4 && sent[2].data[1]==1 && sent[2].data[6]==0x75 && sent[2].data[7]==0x30);
 assert(sent[3].data[1]==0xFF && sent[3].data[4]==0xFF);mask=0;
 reply(0x100,0xFD,0xE2);assert(strstr(last_message,"FD E2 6B"));
 Debug_ParseStepper("S35RAW=1,4294967296,1");drain();assert(strstr(last_message,"FORMAT/RANGE"));
 Debug_ParseStepper("S35RAW=2,100,1");assert(!s_stepper_req[0].action);
 Debug_ParseStepper("S35RAW=0,100,3001");assert(!s_stepper_req[0].action);
 Debug_ParseStepper("S35MOVE=429,10");assert(!s_stepper_req[0].action);
 Debug_ParseStepper("S35MOVE=1000,101");assert(!s_stepper_req[0].action);
 Debug_ParseStepper("S28MOVE=2000,1");assert(!s_stepper_req[1].action);
 Debug_ParseStepper("S28MOVE=2000,5663");assert(!s_stepper_req[1].action);
 Debug_ParseStepper("S28MOVE=2000,50,1");assert(!s_stepper_req[1].action);
 Debug_ParseStepper("S28MOVE=-2000,50");assert(!s_stepper_req[1].action);
 Debug_ParseStepper("S28MOVE=2000.0,50");assert(!s_stepper_req[1].action);drain();
 /* Read-only query and explicit enable use the proper single CAN frame. */
 n=calls;Debug_ParseStepper("S35STATUS");Debug_ServiceSteppers();drain();
 assert(calls==n+1 && sent[n].id==0x100 && sent[n].len==2 && sent[n].data[0]==0x3A && sent[n].data[1]==0x6B);
 tick=499;Debug_ServiceStepperReplies();assert(s_ack_read==s_ack_write);
 tick=500;Debug_ServiceStepperReplies();drain();assert(strstr(last_message,"NO_REPLY CMD=3A"));
 reply(0x100,0x3A,1);assert(strstr(last_message,"S35 RX CAN=0100 DATA=3A 01 6B"));
 Debug_ServiceStepperReplies();assert(s_ack_read==s_ack_write);
 n=calls;Debug_ParseStepper("S35EN");Debug_ServiceSteppers();drain();
 assert(calls==n+1 && sent[n].len==5 && memcmp(sent[n].data,"\xF3\xAB\x01\x00\x6B",5)==0);
 reply(0x100,0xF3,2);
 /* BUSY retries, and insufficient mailboxes never submit half an FD command. */
 n=calls;free_mailboxes=1;Debug_ParseStepper("S28MOVE=2000,50");Debug_ServiceSteppers();
 assert(calls==n && s_stepper_req[1].action==2);
 free_mailboxes=3;Debug_ServiceSteppers();assert(calls==n+2 && !s_stepper_req[1].action);
 reply(0x200,0,0xEE);assert(!s_stepper_watch_cmd[1]);
 Debug_ParseStepper("S28HOME");Debug_ParseStepper("S28CANCEL");
 n=calls;Debug_ServiceSteppers();drain();assert(n==calls && strstr(last_message,"CANCELLED"));
 Debug_ParseStepper("S35HOME");tick+=1001;
 Debug_ServiceSteppers();drain();assert(n==calls && strstr(last_message,"CAN_TX_TIMEOUT"));
 Debug_ParseStepper("S35HOME");result=HAL_ERROR;
 Debug_ServiceSteppers();drain();assert(strstr(last_message,"CAN_TX_FAILED CMD=9A") && mask==0);
 result=HAL_OK;n=calls;
 assert(Motor_AbsPosition(0,0x100,100,3001)==HAL_ERROR && calls==n);
 assert(Motor35_AbsPosition(1000,101)==HAL_ERROR && calls==n);
 assert(Motor28_AbsPosition(2000,5663)==HAL_ERROR && calls==n);
 assert(!Debug_ParseStepper("KPX=1"));
 /* Full event queue cannot overwrite unread diagnostics or unmask interrupts. */
 n=s_ack_write;s_ack_read=(n+1)%16;mask=1;
 Debug_StepperEvent(0,0,0,NULL);assert(s_ack_write==n && mask==1);mask=0;
 puts("X stepper CAN bytes / debug routing / replies / retries passed");
 return 0;
}
'''
names = ['Debug_StepperEvent', 'Debug_FormatStepperEvent', 'Debug_ServiceStepperReplies',
         'Debug_StrCaseCmp', 'Debug_StrCaseCmpN', 'Debug_ParseUnsignedList',
         'Debug_ParseStepper', 'Debug_ServiceSteppers']
with tempfile.TemporaryDirectory(prefix='ilhc_stepper_test_') as directory:
    folder = Path(directory)
    (folder / 'hcan.h').write_text(header, encoding='utf-8')
    (folder / 'stepper_2835.h').write_text((ROOT / 'Hardware/stepper_2835.h').read_text(encoding='utf-8'), encoding='utf-8')
    src, exe = folder / 'test.c', folder / 'test.exe'
    src.write_text(prelude + (ROOT / 'Hardware/stepper_2835.c').read_text(encoding='utf-8')
                   + '\n'.join(function(n) for n in names) + checks, encoding='utf-8')
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(src), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
