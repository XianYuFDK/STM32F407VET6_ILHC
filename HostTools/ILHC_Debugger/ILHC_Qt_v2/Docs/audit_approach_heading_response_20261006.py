"""v2.1.18提前转向模型及21:12地图整轮验收，不连接串口。"""
import hashlib
import json
import math
from pathlib import Path
import sys
import time

QT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(QT))
import competition_simulation as competition
import coordinate_navigation as coordinate
from approach_heading import anticipate_route
from hardware_coordinates import make_coordinate_batch
from route_store import clear_memory
from tests.test_approach_heading import fixture
from tests.test_competition_simulation import runner_for,advance


def trace(program):
    tracker=coordinate.CoordinateTracker(program);first=tracker.reference_at(0);target=tracker.points[1]
    pose=first['x_mm'],first['y_mm'],first['field_yaw_deg'];rows=[]
    for tick in range(1500):
        ref,velocity,omega=tracker.command(pose,None,None)
        if tracker.index!=1:break
        rows.append(dict(t=tick*.02,gap_mm=math.dist(pose[:2],(target['x_mm'],target['y_mm'])),
            yaw_error_deg=abs(coordinate.wrap(target['field_yaw_deg']-pose[2])),speed_mm_s=math.hypot(*velocity)))
        pose=pose[0]+velocity[0]*.02,pose[1]+velocity[1]*.02,pose[2]+omega*.02
    return rows


def main():
    directory=QT/'records/runs/20261006_211155_339638_REAL_8badec59/runs/92ebbeb40d1246c6b9b2962ef2b4d9a3'
    closed=json.loads((QT/'Docs/approach_heading_real_20261006.json').read_text(encoding='utf-8'))
    for name,expected in closed['closed_file_sha256'].items():
        assert hashlib.sha256((directory/name).read_bytes()).hexdigest()==expected
    recorded=json.loads((directory/'plan.json').read_text(encoding='utf-8'))['match']
    data=recorded['map_snapshot'];control=recorded['chassis_control']
    clear_memory();started=time.perf_counter()
    match=competition.compile_match(data,coordinate_mode=True,chassis_control=control)
    first_s=time.perf_counter()-started
    clear_memory();started=time.perf_counter()
    cached=competition.compile_match(data,coordinate_mode=True,chassis_control=control)
    cached_s=time.perf_counter()-started
    assert cached['planning_stats']['search_calls']==0
    scene=competition.collision_scene(data)
    batch=make_coordinate_batch(match,(0,0,0),scene,token=17)
    assert batch['required_coordinate_caps']==8
    runner=runner_for(match)
    while runner.active:advance(runner)
    assert runner.status=='COMPLETE',runner.reason
    old,room=fixture();new=anticipate_route(old,room)
    a=trace(old['waypoint_program']);b=trace(new['waypoint_program'])
    report=dict(physical_motion_verified=False,nano_verified=False,source_run_id=directory.name,
        first_plan_s=first_s,cached_plan_s=cached_s,planning_stats=cached['planning_stats'],
        point_count=batch['point_count'],modeled_round_s=runner.elapsed_s,
        waypoint_changes=[dict(label=l['label'],changes=l['route'].get('approach_turns',[]),
            rejected=[r for r in l['route'].get('approach_turn_attempts',[]) if not r['accepted']]) for l in match['legs']],
        open_corner_model=dict(old_lead_mm=40,new_lead_mm=new['waypoint_program']['waypoints'][1]['turn_lead_mm'],
            old_first_under_30mm=next(r for r in a if r['gap_mm']<30),
            new_first_under_30mm=next(r for r in b if r['gap_mm']<30),before=a,after=b),
        closed_file_sha256=closed['closed_file_sha256'])
    (QT/'Docs/approach_heading_response_20261006.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False})
    fig,(ax,bx)=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    for rows,color,label in ((a,'#c35545','原40mm转向'),(b,'#16856c','新337mm提前转向')):
        near=[r for r in rows if r['gap_mm']<450]
        ax.plot([r['gap_mm'] for r in near],[r['yaw_error_deg'] for r in near],color=color,label=label)
        bx.plot([r['gap_mm'] for r in near],[r['speed_mm_s'] for r in near],color=color,label=label)
    for axis in (ax,bx):
        axis.set_xlim(450,0);axis.axvline(30,color='#777',linestyle='--',linewidth=1)
        axis.grid(alpha=.2);axis.set_xlabel('距普通拐点 mm');axis.legend()
    ax.axhline(1,color='#777',linestyle=':',linewidth=1);ax.set_ylabel('目标航向剩余误差 °')
    bx.set_ylabel('模型平移速度 mm/s')
    fig.suptitle('开阔90°普通弯：行进中完成车头，PASS同帧接下一点（离线模型）')
    fig.savefig(QT/'Docs/approach_heading_comparison_20261006.png',dpi=150);plt.close(fig)
    manifest_path=QT/'precomputed_routes/manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    known={r['file'] for r in manifest['routes']}
    for leg in cached['legs']:
        filename=leg['route']['route_reuse']['identity']+'.json.gz'
        if filename not in known:
            manifest['routes'].append(dict(file=filename,sha256=hashlib.sha256((manifest_path.parent/filename).read_bytes()).hexdigest()))
            known.add(filename)
    manifest.setdefault('additional_map_snapshots',[]).append(dict(source_run_id=directory.name,
        map_id=data['map_id'],map_version=data['map_version'],
        source_plan_sha256=hashlib.sha256((directory/'plan.json').read_bytes()).hexdigest()))
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print('21:12地图整轮完成',report['modeled_round_s'],'点数',report['point_count'],'缓存',cached_s,'首次',first_s)
    print('30mm处剩余航向',report['open_corner_model']['old_first_under_30mm']['yaw_error_deg'],
          report['open_corner_model']['new_first_under_30mm']['yaw_error_deg'])
    print('提前距离',[(c['label'],c['changes']) for c in report['waypoint_changes']])


if __name__=='__main__':main()
