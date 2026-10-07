"""v2.1.18发布及19:31场景验收；仅离线建模，不改实机日志。"""
import hashlib
import argparse
import json
import platform
import re
from pathlib import Path
import sys
import time

QT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(QT))
import competition_simulation as competition
import core
import numpy
import shapely
from hardware_coordinates import make_coordinate_batch
from route_store import ROOT, algorithm_revision, clear_memory
from tests.test_competition_simulation import runner_for, advance


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse-existing',action='store_true',help='复测已生成缓存，不强制冷搜索')
    parser.add_argument('--output',default='Docs/approach_heading_release_20261006.json',help='独立验收报告路径')
    args=parser.parse_args()
    report = dict(physical_motion_verified=False, nano_verified=False,
                  refresh_static_routes=not args.reuse_existing,
                  precompute=[], runs=[], algorithm=algorithm_revision(),
                  platform=platform.platform(), python=platform.python_version(),
                  numpy=numpy.__version__, shapely=shapely.__version__)
    identities = set()
    for map_name in ('competition_map.json', 'navigation_map.json'):
        data, _ = competition.with_competition_defaults(competition.nav.load_map(QT/map_name))
        for zone in (1, 2):
            started = time.perf_counter()
            print('precompute',map_name,'zone',zone,flush=True)
            competition.compile_match(data, zone=zone, coordinate_mode=True, refresh_static_routes=not args.reuse_existing)
            report['precompute'].append(dict(map=map_name, zone=zone, seconds=time.perf_counter()-started))
            for label in ('DISK', 'MEMORY'):
                if label == 'DISK':
                    clear_memory()
                started = time.perf_counter()
                match = competition.compile_match(data, zone=zone, coordinate_mode=True)
                seconds = time.perf_counter()-started
                assert match['planning_stats']['search_calls'] == 0
                identities.update(leg['route']['route_reuse']['identity'] for leg in match['legs'])
                started = time.perf_counter()
                batch = make_coordinate_batch(match, (0, 0, 0), competition.collision_scene(data), token=zone)
                batch_s = time.perf_counter()-started
                runner = runner_for(match)
                while runner.active:
                    advance(runner)
                assert runner.status == 'COMPLETE', runner.reason
                report['runs'].append(dict(map=map_name, zone=zone, pass_kind=label,
                    plan_wall_s=seconds, batch_wall_s=batch_s, point_count=batch['point_count'],
                    planning_stats=match['planning_stats'], modeled_round_s=runner.elapsed_s,
                    modeled_travel_s=sum(leg['route']['predicted_tracking_s'] for leg in match['legs'])))
    data = competition.load_profile()
    for obstacles in (((700, 1200),), competition.DEMO_OBSTACLES, ((1200, 1200), (297, 294))):
        for zone in (1, 2):
            started = time.perf_counter()
            match = competition.compile_match(data, zone=zone, sim_obstacles=obstacles, coordinate_mode=True)
            seconds = time.perf_counter()-started
            runner = runner_for(match)
            while runner.active:
                advance(runner)
            assert runner.status == 'COMPLETE', runner.reason
            report['runs'].append(dict(map='competition_map.json', zone=zone, pass_kind='OBSTACLES',
                obstacles=obstacles, plan_wall_s=seconds, planning_stats=match['planning_stats'],
                modeled_round_s=runner.elapsed_s))
    assert not any(name.startswith(('PySide6', 'pyqtgraph')) for name in sys.modules)
    # 实机本轮七参数也预计算，更新后首次启动不必重新搜索这组已知参数。
    actual_control=dict(kpx=1.5,kpy=1.5,kpz=9.05,xyvmax=930.0,zvmax=750.0,xyvmin=.5,zvmin=.5)
    data=competition.load_profile()
    for zone in (1,2):
        print('precompute REAL parameters zone',zone,flush=True)
        started=time.perf_counter()
        match=competition.compile_match(data,zone=zone,coordinate_mode=True,chassis_control=actual_control,
                                        refresh_static_routes=not args.reuse_existing)
        report['precompute'].append(dict(map='competition_map.json',zone=zone,control=actual_control,
                                        seconds=time.perf_counter()-started))
        clear_memory()
        match=competition.compile_match(data,zone=zone,coordinate_mode=True,chassis_control=actual_control)
        assert match['planning_stats']['search_calls']==0
        identities.update(leg['route']['route_reuse']['identity'] for leg in match['legs'])
        batch=make_coordinate_batch(match,(0,0,0),competition.collision_scene(data),token=zone)
        assert batch['required_coordinate_caps']==8
        runner=runner_for(match)
        while runner.active:advance(runner)
        assert runner.status=='COMPLETE',runner.reason
        report['runs'].append(dict(map='competition_map.json',zone=zone,pass_kind='REAL_PARAMETERS',
            chassis_control=actual_control,planning_stats=match['planning_stats'],point_count=batch['point_count'],
            modeled_round_s=runner.elapsed_s))
    plan_path=QT/'records/runs/20261006_193116_859875_REAL_56ce1058/runs/852e5b168c7a4f76964180cb4a71dcbe/plan.json'
    recorded=json.loads(plan_path.read_text(encoding='utf-8'))['match']
    data=recorded['map_snapshot']
    report['recorded_scene']=dict(plan_path=str(plan_path),sim_obstacles=recorded['sim_obstacles'],runs=[])
    for zone in (1,2):
        for with_obstacles in (False,True):
            print('19:31 map zone',zone,'obstacles',with_obstacles,flush=True)
            obstacles=recorded['sim_obstacles'] if with_obstacles else ()
            started=time.perf_counter()
            match=competition.compile_match(data,zone=zone,coordinate_mode=True,chassis_control=actual_control,
                sim_obstacles=obstacles,refresh_static_routes=not args.reuse_existing)
            cold_s=time.perf_counter()-started
            if not with_obstacles:
                clear_memory();started=time.perf_counter()
                match=competition.compile_match(data,zone=zone,coordinate_mode=True,chassis_control=actual_control)
                cached_s=time.perf_counter()-started
                assert match['planning_stats']['search_calls']==0
                identities.update(leg['route']['route_reuse']['identity'] for leg in match['legs'])
            else:cached_s=None
            batch=make_coordinate_batch(match,(0,0,0),competition.collision_scene(data,10,obstacles),token=zone)
            assert batch['required_coordinate_caps']==8
            runner=runner_for(match)
            while runner.active:advance(runner)
            assert runner.status=='COMPLETE',runner.reason
            legs=[]
            for i,leg in enumerate(match['legs']):
                route=leg['route'];program=route['waypoint_program']
                legs.append(dict(index=i,skeleton=route['skeleton_points'],length_mm=program['length_mm'],
                    predicted_s=route['predicted_tracking_s'],execution_cost=route['execution_cost'],
                    goal_yaw=program['waypoints'][-1]['layout_yaw_deg'],
                    pivots=[dict(wheel=p['pivot_wheel'],yaw=p['layout_yaw_deg'])
                            for p in program['waypoints'] if p.get('motion')=='WHEEL_PIVOT']))
            report['recorded_scene']['runs'].append(dict(zone=zone,with_obstacles=with_obstacles,
                cold_plan_s=cold_s,cached_plan_s=cached_s,planning_stats=match['planning_stats'],
                modeled_round_s=runner.elapsed_s,point_count=batch['point_count'],legs=legs))
    report['qt_imported'] = False
    manifest = dict(schema_version=1, algorithm=algorithm_revision(), maps=['competition_map.json', 'navigation_map.json'],
        zones=[1, 2], chassis_control=core.CHASSIS_DEFAULTS, margin_mm=10,
        footprint_mm=[core.CAR_LENGTH_MM, core.CAR_WIDTH_MM], routes=[])
    manifest['additional_chassis_control_profiles']=[actual_control]
    manifest['additional_map_snapshots']=[dict(source_run_id='852e5b168c7a4f76964180cb4a71dcbe',
        map_id=data['map_id'],map_version=data['map_version'],
        source_plan_sha256=hashlib.sha256(plan_path.read_bytes()).hexdigest())]
    cache_root = ROOT.resolve()
    assert cache_root == (QT/'precomputed_routes').resolve()
    for identity in sorted(identities):
        path = cache_root/(identity+'.json.gz')
        manifest['routes'].append(dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    # 保留旧散列缓存；源码签名自动失效，避免删除用户已有参数路线。
    (cache_root/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    output = QT/args.output
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
