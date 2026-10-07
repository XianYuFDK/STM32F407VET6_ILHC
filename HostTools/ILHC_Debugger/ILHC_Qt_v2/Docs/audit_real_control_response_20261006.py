"""带执行惯性/OPS延迟的离线压力模型；不是实车性能承诺。"""
import collections
import json
import math
from pathlib import Path
import types

import coordinate_navigation as coordinate
import core
import competition_simulation as competition


def previous_controller():
    """仅在内存还原v2.1.14基线，先撤销新版提前制动，再撤销v2.1.15三段修复。"""
    source=Path(coordinate.__file__).read_text(encoding='utf-8')
    changes=[(
        "            # 剩余转角还需容纳响应/定位滞后期间的转动；不能等越过1deg门才减速。\n            # 保留非零衔接速度，解 angle = (omega²-join²)/(2a) + 0.32*omega。\n            accel=.7*math.radians(PIVOT_ACCEL_DEG_S2)*min(1.0,self.control['kpz']/9.0)\n            lag=accel*.32\n            braking=(math.sqrt(lag*lag+(join/ROTATION_LEVER_MM)**2+2*accel*abs(math.radians(angle)))-lag)*ROTATION_LEVER_MM",
        "            braking=math.sqrt(join*join+2*.7*math.radians(PIVOT_ACCEL_DEG_S2)*min(1.0,self.control['kpz']/9.0)*abs(math.radians(angle))*ROTATION_LEVER_MM**2)"),(
        "        if self._heading_index!=self.index:\n            self._heading_index=self.index;self._exit_heading=False\n        if distance<=target.get('turn_lead_mm',self.control['turn_lead_mm']):\n            self._exit_heading=True\n        desired_yaw = target['field_yaw_deg'] if self._exit_heading else target['travel_yaw_deg']",
        "        desired_yaw = target['travel_yaw_deg'] if distance > self.control['turn_lead_mm'] else target['field_yaw_deg']"),
        ("            if target['kind']=='STOP' and abs(remaining)<20 or remaining<=0:\n                desired=numerical_limit(gain*remaining,self.control['xyvmax'],self.control['xyvmin'])\n", ""),
        ("            if self._pivot_recover:\n                requested=numerical_limit(self.control['kpz']*(angle-.12*self._measured_yaw_rate),\n                                          self.control['zvmax'],self.control['zvmin'])\n", "")]
    for new,old in changes:
        if source.count(new)!=1:
            raise ValueError('压力模型基线还原片段已改变，请核对')
        source=source.replace(new,old)
    module=types.ModuleType('coordinate_before_real_fix')
    module.__file__=coordinate.__file__
    exec(compile(source,module.__file__,'exec'),module.__dict__)
    return module.CoordinateTracker


def simulate(program, scene, tracker_class, tau=.08, ops_delay=.04):
    tracker=tracker_class(program)
    first=tracker.reference_at(0)
    truth=(first['x_mm'],first['y_mm'],first['field_yaw_deg'])
    history=collections.deque([truth]*(round(ops_delay*50)+1),maxlen=round(ops_delay*50)+1)
    dynamics=[0.0,0.0,0.0]
    alpha=.02/(tau+.02)
    window=coordinate.StopWindow()
    trace=[];unsafe=None;done=False
    for tick in range(2000):
        measured=history[0]
        ref,velocity,omega=tracker.command(measured,None,None)
        c,s=math.cos(math.radians(truth[2])),math.sin(math.radians(truth[2]))
        desired=(c*velocity[0]+s*velocity[1],-s*velocity[0]+c*velocity[1],omega)
        for i in range(3):
            dynamics[i]+=alpha*(desired[i]-dynamics[i])
        candidate=(truth[0]+(c*dynamics[0]-s*dynamics[1])*.02,
                   truth[1]+(s*dynamics[0]+c*dynamics[1])*.02,truth[2]+dynamics[2]*.02)
        a=(*core.field_to_layout(*truth[:2]),-90-truth[2]);b=(*core.field_to_layout(*candidate[:2]),-90-candidate[2])
        why=scene.moving_pose_reason(a,b)
        if why or tracker.cross_track>75:
            unsafe=why or '偏离超过75mm';break
        truth=candidate;history.append(truth)
        goal=tracker.final_reference()
        eligible=(ref['segment_type']=='STOP' and math.dist(measured[:2],(goal['x_mm'],goal['y_mm']))<1 and
                  abs(coordinate.wrap(goal['field_yaw_deg']-measured[2]))<1 and math.hypot(*velocity)<=1 and abs(omega)<=1)
        done=window.update(measured,eligible,.02)>=200
        trace.append(dict(t=(tick+1)*.02,x=truth[0],y=truth[1],yaw=truth[2],
                          index=tracker.index,omega=dynamics[2],recovery=getattr(tracker,'_pivot_recover',False)))
        if done:break
    arc=[r for r in trace if r['index'] in tracker.pivots]
    reverse=0;last=None
    for row in arc:
        if abs(row['omega'])<5:continue
        sign=1 if row['omega']>0 else -1
        if last is not None and sign!=last:reverse+=1
        last=sign
    return dict(done=done,unsafe=unsafe,elapsed_s=len(trace)*.02,
                pivot_s=len(arc)*.02,pivot_rate_reversals=reverse,trace=trace)


def main():
    qt=Path(__file__).resolve().parents[1]
    session=qt/'records/runs/20261006_173705_418470_REAL_18d0f001'
    plan=json.loads((session/'runs/10eb2c61fbc3423ea6e5fe7521c0e41a/plan.json').read_text(encoding='utf-8'))
    program=plan['match']['legs'][3]['route']['waypoint_program']
    scene=competition.collision_scene(plan['match']['map_snapshot'],9)
    before=previous_controller();cases=[]
    for tau,delay in ((0,0),(.04,.02),(.08,.04),(.12,.04),(.16,.06)):
        cases.append(dict(tau_s=tau,ops_delay_s=delay,
            before=simulate(program,scene,before,tau,delay),
            after=simulate(program,scene,coordinate.CoordinateTracker,tau,delay)))
    output=qt/'Docs/real_control_response_20261006.json'
    output.write_text(json.dumps(cases,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([{**case,'before':{k:v for k,v in case['before'].items() if k!='trace'},
        'after':{k:v for k,v in case['after'].items() if k!='trace'}} for case in cases],ensure_ascii=False,indent=2))


if __name__=='__main__':main()
