"""对19:31场景比较路线和绕轮响应；只生成离线派生文件。"""
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import types

QT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(QT))
import competition_simulation as competition
import coordinate_navigation as coordinate
import core
from Docs.audit_real_control_response_20261006 import simulate


def controller_before_braking():
    # 仅还原v2.1.15/16的正常绕轮制动；保留其恢复、航向锁存和停靠逻辑。
    source=Path(coordinate.__file__).read_text(encoding='utf-8')
    start=source.index('            # 剩余转角还需容纳响应/定位滞后')
    end=source.index('\n            requested=math.copysign',start)
    old='            braking=math.sqrt(join*join+2*.7*math.radians(PIVOT_ACCEL_DEG_S2)*min(1.0,self.control[\'kpz\']/9.0)*abs(math.radians(angle))*ROTATION_LEVER_MM**2)'
    module=types.ModuleType('coordinate_v215_braking');module.__file__=coordinate.__file__
    exec(compile(source[:start]+old+source[end:],module.__file__,'exec'),module.__dict__)
    return module.CoordinateTracker


def main():
    directory=QT/'records/runs/20261006_193116_859875_REAL_56ce1058/runs/852e5b168c7a4f76964180cb4a71dcbe'
    original=json.loads((QT/'Docs/real_run_20261006_193128.json').read_text(encoding='utf-8'))
    for name,expected in original['closed_run_file_sha256'].items():
        assert hashlib.sha256((directory/name).read_bytes()).hexdigest()==expected
    match=json.loads((directory/'plan.json').read_text(encoding='utf-8'))['match']
    data=match['map_snapshot'];obstacles=match['sim_obstacles'];control=match['chassis_control']
    scene=competition.collision_scene(data,10,obstacles)
    started=time.perf_counter()
    route=coordinate.plan_route((1200,400),(400,1200),scene,data['competition']['lane_nodes'],
        0,goal_yaw=270,chassis_control=control,deadline=time.monotonic()+2)
    planning_s=time.perf_counter()-started
    assert route['skeleton_points']==[(1200,400),(400,400),(400,1200)]
    old_route=match['legs'][3]['route']
    report=dict(physical_motion_verified=False,nano_verified=False,source_plan=str(directory/'plan.json'),
        prior_control='PC v2.1.15/16 normal wheel braking',
        parameters=control,sim_obstacles=obstacles,planning_s=planning_s,
        route_before=dict(skeleton=old_route['skeleton_points'],predicted_s=old_route['predicted_tracking_s'],
                          length_mm=old_route['waypoint_program']['length_mm']),
        route_after=dict(skeleton=route['skeleton_points'],predicted_s=route['predicted_tracking_s'],
            length_mm=route['waypoint_program']['length_mm'],execution_cost=route['execution_cost'],
            program=route['waypoint_program']),
        braking_allowance_s=.32,models=[])
    before=controller_before_braking()
    program=match['legs'][2]['route']['waypoint_program']
    checked=competition.collision_scene(data,9,obstacles)
    for tau,delay in ((0,0),(.04,.02),(.08,.04),(.12,.04),(.16,.06)):
        a=simulate(program,checked,before,tau,delay)
        b=simulate(program,checked,coordinate.CoordinateTracker,tau,delay)
        assert b['done'] and b['unsafe'] is None
        report['models'].append(dict(tau_s=tau,ops_delay_s=delay,before=a,after=b))
    (QT/'Docs/adaptive_turn_response_20261006.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle,Circle
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False})
    fig,(ax,bx)=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for x1,y1,x2,y2,*_ in data['rects']:
        x,y=core.layout_to_field(x2,y2)
        ax.add_patch(Rectangle((x,y),y2-y1,x2-x1,color='#b8afa4',alpha=.7))
    for x,y in obstacles:
        ax.add_patch(Circle(core.layout_to_field(x,y),core.SIM_OBSTACLE_R_MM,color='#8c7399',alpha=.5))
    for path,color,label in ((old_route['skeleton_points'],'#b94d40','原骨架：多个短折返'),
                             (route['skeleton_points'],'#16856a','新骨架：倒行后转一次')):
        xy=[core.layout_to_field(*p) for p in path]
        ax.plot([p[0] for p in xy],[p[1] for p in xy],'-o',color=color,label=label)
    for p,label in (((1200,400),'粗加工'),((400,1200),'暂存')):
        x,y=core.layout_to_field(*p);ax.annotate(label,(x,y),xytext=(10,10),textcoords='offset points')
    ax.set(xlim=(800,2300),ylim=(700,2300),xlabel='场地 X（左）mm',ylabel='场地 Y（前）mm',
           title='19:31相同地图 / 相同模拟障碍')
    ax.invert_xaxis();ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='lower right')
    case=report['models'][2]
    for label,result,color in (('原制动',case['before'],'#b94d40'),('提前制动',case['after'],'#16856a')):
        rows=result['trace'];indices=coordinate.CoordinateTracker(program).pivots
        first=next(r['t'] for r in rows if r['index'] in indices)
        rows=[r for r in rows if first-.1<=r['t']<=first+4]
        bx.plot([r['t']-first for r in rows],[r['omega'] for r in rows],color=color,
                label=f"{label}，明显反向 {result['pivot_rate_reversals']} 次（>5°/s）")
    bx.axhline(0,color='#777777',linewidth=.8)
    bx.set(xlabel='距入弯时间 s',ylabel='模型角速度 °/s',title='离线压力模型：执行惯性80ms / OPS延迟40ms')
    bx.grid(alpha=.2);bx.legend()
    fig.suptitle('v2.1.17 路线与制动修复（模型结果，尚未更新实车复测）')
    fig.savefig(QT/'Docs/adaptive_turn_comparison_20261006.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k not in ('models','route_after')},ensure_ascii=False,indent=2))
    print('新路线模型用时',report['route_after']['predicted_s'])
    print('模型对照',[(c['tau_s'],c['ops_delay_s'],c['before']['pivot_s'],c['after']['pivot_s'],
        c['before']['pivot_rate_reversals'],c['after']['pivot_rate_reversals']) for c in report['models']])


if __name__=='__main__':main()
