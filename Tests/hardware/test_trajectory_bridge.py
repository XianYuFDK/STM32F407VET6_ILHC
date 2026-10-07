"""真实调试桥接与世界速度换算：中途STOP、心跳、OPS快照与四向轴序。"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]


def function(source,name):
    m=re.search(r'^(?:static )?(?:void|uint8_t) '+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert m,name
    end,depth=m.end(),1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[m.start():end]


source=(ROOT/'Hardware/debug_usart.c').read_text(encoding='utf-8')
code=r'''
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "trajectory_buffer.h"
#define DEBUG_HOST_TIMEOUT_MS 1000
#define DEBUG_OPS_TIMEOUT_MS 200
#include "mecanum_geometry.h"
#define ZDT_X42S_MAX_RPM 3000U
static float s_world_rpm_remainder[4];
static int last_Speed[4];
typedef struct {uint32_t valid_count,last_update_tick;} OPS_Data_t;
static OPS_Data_t ops={10,1000};
static uint32_t tick=1000,s_host_last_tick=1000,mask,cancels,stops,moves;
static uint8_t ready=1,online=1,active=1,action=1,output_allowed=1,cancel_in_step,recovering,inject_ops_error;
static uint32_t holds;
static uint8_t s_stop_req,s_zero_req,s_offset_req,s_manual_active,s_goto_active,s_zdt_req,s_zdt_active,s_vision_req;
static float wx,wy,wz,bx,by,bz;
static uint8_t seen_allowed;static TrajPose_t seen_pose;
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __enable_irq(void){mask=0;}
static uint8_t Debug_WheelReady(void){return ready;}
static uint8_t VisionTrack_IsActive(void){return 0;}
static uint8_t Debug_StrCaseCmp(const char *a,const char *b){return (uint8_t)strcmp(a,b);}
static uint8_t Debug_StrCaseCmpN(const char *a,const char *b,uint8_t n){return (uint8_t)strncmp(a,b,n);}
uint8_t Traj_ParseLine(const char *s,uint32_t now,uint8_t allowed){(void)now;seen_allowed=allowed;return strncmp(s,"T",1)==0;}
void Traj_Cancel(uint8_t reason){assert(reason==15);++cancels;output_allowed=0;}
uint8_t Traj_Busy(void){return active;}
uint8_t Traj_OutputAllowed(void){return output_allowed;}
void Traj_Hold(uint32_t now){assert(now==tick);++holds;}
uint8_t Traj_Step(uint32_t now,const TrajPose_t *pose,uint8_t allowed,float v[3]){
 (void)now;seen_pose=*pose;seen_allowed=allowed;v[0]=100;v[1]=200;v[2]=30;
 if(cancel_in_step)output_allowed=0;
 if(inject_ops_error)online=0;
 return action;
}
static const OPS_Data_t *OPS_GetData(void){return &ops;}
static uint8_t OPS_IsOnline(uint32_t age){(void)age;return online;}
static uint8_t OPS_RecoveryPending(void){return recovering;}
static void MecanumControl_GetPose(float *x,float *y,float *yaw){*x=1;*y=2;*yaw=90;}
static void Debug_ChassisStop(void){++stops;}
static void SetMotorVoltageAndDirection(int a,int b,int c,int d){
 bx=(-a-b+c+d)/4.0f;by=(a-b+c-d)/4.0f;bz=-(a+b+c+d)/4.0f;
}
static void MecanumControl_ResetWorldRpm(void);
static void MecanumControl_Stop(void){++stops;MecanumControl_ResetWorldRpm();}
void MecanumControl_MoveWorldVelocity(float x,float y,float z,float yaw);
'''
# 桥接提交速度另设包装，仍调用真实换算函数。
bridge=function(source,'Debug_ServiceTrajectory').replace('MecanumControl_MoveWorldVelocity(', 'capture_world(')
code+='static void capture_world(float x,float y,float z,float yaw){++moves;wx=x;wy=y;wz=z;MecanumControl_MoveWorldVelocity(x,y,z,yaw);}\n'
assert '__disable_irq' not in bridge
code+='static float mKpx=2.3f,mKpy=2.3f,mKpz=9,XYVmax=1600,ZVmax=750,XYVmin=10,ZVmin=10;\n'
code+='void Traj_UpdateChassisParameters(const float p[7]){assert(p[0]==mKpx&&p[3]==XYVmax); }\n'
code+=function(source,'Debug_SyncChassisParameters')+'\n'
code+=function(source,'Debug_TrajectoryRx')+'\n'+bridge+'\n'
mec=(ROOT/'Hardware/mecanum_control.c').read_text(encoding='utf-8')
code+=function(mec,'MecanumControl_ResetWorldRpm')+'\n'+function(mec,'MecanumControl_MoveWorldVelocity')
code+=r'''
#define DEBUG_GOTO_MOVE 1
static float s_goto_x,s_goto_y,s_goto_z;
static uint32_t s_goto_generation,goto_calls;
static void MecanumControl_ClearTarget(void){++stops;}
static uint8_t MecanumControl_GotoOPS(float x,float y,float z,float limit){
 (void)x;(void)y;(void)z;(void)limit;++goto_calls;return 0;
}
'''
code+=function(source,'Debug_ServiceGoto')+'\n'
code+=r'''
int main(void){
 recovering=1;online=0;Debug_ServiceTrajectory();assert(holds==1&&stops==1&&!moves);
 recovering=0;online=1;stops=0;
 Debug_ServiceTrajectory();assert(moves==1&&wx==100&&wy==200&&wz==30&&seen_allowed);
 assert(seen_pose.sequence==10&&seen_pose.pose_tick==1000&&mask==0);
 mask=1;Debug_ServiceTrajectory();assert(mask==1);mask=0;
 cancel_in_step=1;Debug_ServiceTrajectory();assert(moves==2&&stops==1);cancel_in_step=0;output_allowed=1;
 tick=2001;action=2;Debug_ServiceTrajectory();assert(!seen_allowed&&stops==2);
 tick=1000;online=0;Debug_ServiceTrajectory();assert(seen_pose.pose_tick==799);online=1;
 action=1;s_stop_req=1;Debug_ServiceTrajectory();assert(!seen_allowed&&moves==2);s_stop_req=0;
 assert(Debug_TrajectoryRx("TBEGIN=...")==1&&seen_allowed);
 s_manual_active=1;Debug_TrajectoryRx("TBEGIN=...");assert(!seen_allowed);s_manual_active=0;
 inject_ops_error=1;Debug_ServiceTrajectory();assert(moves==2&&stops==4);inject_ops_error=0;online=1;
 Debug_TrajectoryRx("STOP");Debug_TrajectoryRx("ZERO");Debug_TrajectoryRx("WHEELOFF");
 Debug_TrajectoryRx("GOTO=1,2");Debug_TrajectoryRx("MANUAL=1,2,3");Debug_TrajectoryRx("OPSOFFSET=0,0");
 assert(cancels==6);Debug_TrajectoryRx("PING");Debug_TrajectoryRx("GET KPX");assert(cancels==6);
 MecanumControl_MoveWorldVelocity(100,200,0,0);assert(fabsf(bx-23.8f)<1&&fabsf(by-47.6f)<1);
 MecanumControl_MoveWorldVelocity(100,200,0,90);assert(fabsf(bx+47.6f)<1&&fabsf(by-23.8f)<1);
 MecanumControl_MoveWorldVelocity(100,200,0,180);assert(fabsf(bx+23.8f)<1&&fabsf(by+47.6f)<1);
 MecanumControl_MoveWorldVelocity(100,200,0,270);assert(fabsf(bx-47.6f)<1&&fabsf(by+23.8f)<1);
 MecanumControl_MoveWorldVelocity(0,0,90,0);assert(fabsf(bz-90*0.0174532925f*MECANUM_ROTATION_LEVER_MM*.238f)<1);
 {uint32_t old=stops;MecanumControl_MoveWorldVelocity(NAN,0,0,0);assert(stops==old+1);}
 s_goto_active=DEBUG_GOTO_MOVE;online=0;recovering=1;Debug_ServiceGoto();
 assert(s_goto_active&&!goto_calls); /* 短错保留GOTO目标。 */
 recovering=0;online=1;Debug_ServiceGoto();assert(s_goto_active&&goto_calls==1);
 online=0;Debug_ServiceGoto();assert(!s_goto_active); /* 非恢复/超时继续停机。 */
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='ilhc-trajectory-bridge-') as d:
    folder=Path(d);src=folder/'test.c';exe=folder/'test.exe'
    src.write_text(code,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(ROOT/'Hardware'),str(src),'-o',str(exe),'-lm'],check=True)
    subprocess.run([str(exe)],check=True)
print('整批桥接STOP/心跳/快照与四向世界速度换算通过')
