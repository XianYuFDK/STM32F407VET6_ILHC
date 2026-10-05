"""Actual C trajectory + actual integer-RPM world/mixer chain; no hardware."""
import ctypes
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import zlib
import json
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'Tests/hardware'))
from position_test_support import write_position_kernel
from test_trajectory_buffer import Engine,Pose
from hardware_coordinates import POINT
from motor_test_support import write_motor_kernel,wheel_engine

def function(source,name):
    start=source.index('void '+name+'(');brace=source.index('{',start);end=brace+1;level=1
    while level:level+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]

def run(legacy,single_axis=False):
    source=(ROOT/'Hardware/mecanum_control.c').read_text(encoding='utf-8')
    with tempfile.TemporaryDirectory(prefix='ilhc-exit-wheel-') as directory:
        folder=Path(directory)
        (folder/'main.h').write_text('#include <stdint.h>\nstatic inline uint32_t __get_PRIMASK(void){return 0;}\nstatic inline void __disable_irq(void){}\nstatic inline void __enable_irq(void){}\n')
        wheel=write_motor_kernel(folder)
        if legacy:
            # Only replace world output with its former direct integer mixer call.
            old='''void MecanumControl_MoveWorldVelocity(float vx,float vy,float omega,float yaw){
 float c,s;
 if(!isfinite(vx)||!isfinite(vy)||!isfinite(omega)||!isfinite(yaw)){MecanumControl_Stop();return;}
 c=cosf(yaw*0.0174532925f);s=sinf(yaw*0.0174532925f);
 MecanumControl_MoveVelocity((c*vx-s*vy)*0.238f,(s*vx+c*vy)*0.238f,
 omega*0.0174532925f*MECANUM_ROTATION_LEVER_MM*0.238f);
}'''
            wheel.write_text(wheel.read_text(encoding='utf-8').replace(
                function(source,'MecanumControl_MoveWorldVelocity'),old),encoding='utf-8')
        dll_path=folder/'exit.dll'
        trajectory=ROOT/'Hardware/trajectory_buffer.c'
        if legacy:
            # Historicalv2.1.5 comparison: retain its3s fault instead of the
            # current user-requested warning-and-continue policy.
            text=trajectory.read_text(encoding='utf-8')
            begin=text.index('  /* 用户选择短暂停滞继续纠偏')
            end=text.index('  if(dt>0 && now%200U<dt)',begin)
            text=text[:begin]+'  if((uint32_t)(now-progress_tick)>3000U) { fail(14);return 2U; }\n'+text[end:]
            trajectory=folder/'legacy_trajectory.c';trajectory.write_text(text,encoding='utf-8')
        subprocess.run(['gcc','-shared','-std=c99','-Wall','-Wextra','-Werror','-O2','-I',str(folder),'-I',str(ROOT/'Hardware'),
            str(trajectory),str(write_position_kernel(folder)),str(wheel),'-o',str(dll_path),'-lm'],check=True)
        dll=ctypes.CDLL(str(dll_path));dll.Traj_ParseLine.argtypes=[ctypes.c_char_p,ctypes.c_uint32,ctypes.c_uint8]
        dll.Traj_Step.argtypes=[ctypes.c_uint32,ctypes.POINTER(Pose),ctypes.c_uint8,ctypes.POINTER(ctypes.c_float)]
        dll.MecanumControl_MoveWorldVelocity.argtypes=[ctypes.c_float]*4
        e=wheel_engine(Engine)(dll);wheels=e.wheels
        points=[(0,0,0,0,1,0,0,2000),(1500,1500,2121,0,1,0,0,2000),(1500,3000,3621,0,1,0,0,2000)]
        if single_axis:
            points=[(1500,0,0,0,1,0,0,2000),(1500,1500,1500,0,1,0,0,2000),(1500,3000,3000,0,1,0,0,2000)]
            e.pose.x=150
        crc=zlib.crc32(b''.join(POINT.pack(*p) for p in points))
        e.send('CBEGIN=17,3,1,%08X,10,2300,2300,9000,1600000,750000,5000,5000'%crc)
        for i,p in enumerate(points):
            x,y,s,yaw,flags,travel,passed,lead=p
            e.send('CPOINT='+','.join(map(str,(17,i,x,y,yaw,s,flags,travel,passed,lead))))
        e.send('TCOMMIT=17')
        for _ in range(5):e.step()
        e.send('TRUN=17')
        for _ in range(1000):
            e.step(move=True)
            status=e.status()
            if status[1] in (6,8):break
        result=dict(model='old_integer_truncation' if legacy else 'fractional_rpm',status=status,
                    path='single_y' if single_axis else 'diagonal',
                    pose_mm=[e.pose.x,e.pose.y,e.pose.yaw],rpm=list(wheels),seconds=(e.now-1000)/1000)
        ctypes.windll.kernel32.FreeLibrary.argtypes=[ctypes.c_void_p]
        handle=dll._handle;dll=None;ctypes.windll.kernel32.FreeLibrary(handle)
        return result

def main():
    results=[run(old,axis) for axis in (False,True) for old in (True,False)]
    for old,new in zip(results[::2],results[1::2]):
        assert old['status'][1]==8 and old['status'][-1]==14,results
        assert new['status'][1]==6 and new['status'][-1]==0,results
    path=Path(__file__).with_name('exit_stop_repro_results_20261005.json')
    path.write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2),flush=True)

if __name__=='__main__':main()
