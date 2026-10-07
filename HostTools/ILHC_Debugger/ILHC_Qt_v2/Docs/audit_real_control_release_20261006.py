"""离线生成发布路线与性能记录，不连接串口；清理仅限本任务新建缓存目录。"""
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
    parser.add_argument('--output',default='Docs/real_control_fix_release_20261006.json',help='独立验收报告路径')
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
        assert batch['required_coordinate_caps']==6
        runner=runner_for(match)
        while runner.active:advance(runner)
        assert runner.status=='COMPLETE',runner.reason
        report['runs'].append(dict(map='competition_map.json',zone=zone,pass_kind='REAL_PARAMETERS',
            chassis_control=actual_control,planning_stats=match['planning_stats'],point_count=batch['point_count'],
            modeled_round_s=runner.elapsed_s))
    report['qt_imported'] = False
    manifest = dict(schema_version=1, algorithm=algorithm_revision(), maps=['competition_map.json', 'navigation_map.json'],
        zones=[1, 2], chassis_control=core.CHASSIS_DEFAULTS, margin_mm=10,
        footprint_mm=[core.CAR_LENGTH_MM, core.CAR_WIDTH_MM], routes=[])
    manifest['additional_chassis_control_profiles']=[actual_control]
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
