"""Extract the real wheel mixer, quantizer, stop and ZDT output; stub only UART."""
from pathlib import Path
import ctypes
import math
import re

ROOT = Path(__file__).resolve().parents[2]
geometry_source=(ROOT/'Hardware/mecanum_geometry.h').read_text(encoding='utf-8')
WHEELBASE_MM=float(re.search(r'#define MECANUM_WHEELBASE_MM ([\d.]+)f',geometry_source).group(1))
TRACK_MM=float(re.search(r'#define MECANUM_TRACK_MM ([\d.]+)f',geometry_source).group(1))
ROTATION_LEVER_MM=(WHEELBASE_MM+TRACK_MM)/2


def function(source, name):
    start = source.index('void '+name+'(')
    brace = source.index('{', start)
    end, depth = brace+1, 1
    while depth:
        depth += (source[end] == '{')-(source[end] == '}')
        end += 1
    return source[start:end]


def write_motor_kernel(folder):
    source = (ROOT/'Hardware/mecanum_control.c').read_text(encoding='utf-8')
    prelude = '''#include <stdint.h>
#include <math.h>
#include <stddef.h>
#define ZDT_X42S_MAX_RPM 3000U
#define ZDT_X42S_DIR_CW 0U
#define ZDT_X42S_DIR_CCW 1U
#include "mecanum_geometry.h"
int last_Speed[4],SpeedTarget[4],captured_wheels[4];
static uint8_t in_pos,near_pos,delay_pos;
static uint32_t s_settle_ms;
static float s_world_rpm_remainder[4];
static void ZDT_X42S_SpeedAcc(uint8_t addr,uint8_t dir,uint16_t rpm,uint8_t acc){
 (void)acc;captured_wheels[addr-1]=dir?-(int)rpm:(int)rpm;
}
static void ZDT_X42S_Disable(uint8_t addr){(void)addr;}
'''
    names = ('MecanumControl_ResetWorldRpm', 'SpeedTarget_stop',
             'Mecanum_NormalizeWheelSpeed', 'SetMotorVoltageAndDirection',
             'MecanumControl_CalcWheelSpeed', 'MecanumControl_ClearTarget',
             'MecanumControl_Stop', 'MecanumControl_Disable',
             'MecanumControl_MoveVelocity', 'MecanumControl_MoveWorldVelocity')
    path = folder/'motor_kernel.c'
    path.write_text(prelude+'\n'.join(function(source, n) for n in names), encoding='utf-8')
    return path


def wheel_motion(wheels, yaw):
    a, b, c, d = wheels
    bx = (-a-b+c+d)/(4*.238)
    by = (a-b+c-d)/(4*.238)
    omega = -sum(wheels)/(4*.238*ROTATION_LEVER_MM)*180/math.pi
    angle = math.radians(yaw)
    return (math.cos(angle)*bx+math.sin(angle)*by,
            -math.sin(angle)*bx+math.cos(angle)*by, omega)


def wheel_engine(base):
    class WheelEngine(base):
        def __init__(self, dll):
            super().__init__(dll)
            self.dll.MecanumControl_MoveWorldVelocity.argtypes = [ctypes.c_float]*4
            self.dll.MecanumControl_MoveVelocity.argtypes = [ctypes.c_float]*3
            self.wheels = (ctypes.c_int*4).in_dll(dll, 'captured_wheels')
            self.dll.MecanumControl_Stop()
            self.diagnostics=[]

        def status(self):
            self.send('TSTATUS=17')
            while True:
                text=self.reply()
                if text is None:raise AssertionError('Missing TSTAT reply')
                if text.startswith('CSTALL '):self.diagnostics.append(text)
                elif text.startswith('TSTAT '):return tuple(map(int,text.split()[1:]))

        def step(self, allowed=1, move=False):
            action = super().step(allowed, move=False)
            if action == 1 and move:
                self.dll.MecanumControl_MoveWorldVelocity(*self.velocity, self.pose.yaw)
            elif action == 2 or not allowed or not self.dll.Traj_OutputAllowed():
                self.dll.MecanumControl_Stop()
            if move:
                vx, vy, omega = wheel_motion(self.wheels, self.pose.yaw)
                self.pose.x += vx*.02
                self.pose.y += vy*.02
                self.pose.yaw = (self.pose.yaw+omega*.02+180)%360-180
            return action
    return WheelEngine
