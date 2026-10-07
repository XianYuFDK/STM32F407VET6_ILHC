"""离线生成发布路线与性能记录，不连接串口；清理仅限本任务新建缓存目录。"""
import hashlib
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
    report = dict(physical_motion_verified=False, nano_verified=False,
                  precompute=[], runs=[], algorithm=algorithm_revision(),
                  platform=platform.platform(), python=platform.python_version(),
                  numpy=numpy.__version__, shapely=shapely.__version__)
    identities = set()
    for map_name in ('competition_map.json', 'navigation_map.json'):
        data, _ = competition.with_competition_defaults(competition.nav.load_map(QT/map_name))
        for zone in (1, 2):
            started = time.perf_counter()
            print('precompute',map_name,'zone',zone,flush=True)
            competition.compile_match(data, zone=zone, coordinate_mode=True, refresh_static_routes=True)
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
    report['qt_imported'] = False
    manifest = dict(schema_version=1, algorithm=algorithm_revision(), maps=['competition_map.json', 'navigation_map.json'],
        zones=[1, 2], chassis_control=core.CHASSIS_DEFAULTS, margin_mm=10,
        footprint_mm=[core.CAR_LENGTH_MM, core.CAR_WIDTH_MM], routes=[])
    cache_root = ROOT.resolve()
    assert cache_root == (QT/'precomputed_routes').resolve()
    for identity in sorted(identities):
        path = cache_root/(identity+'.json.gz')
        manifest['routes'].append(dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    # 该目录由本次实现新建，仅删除旧算法产生的64位散列文件，保留其他文件。
    for path in cache_root.glob('*.json.gz'):
        assert path.resolve().is_relative_to(cache_root)
        if re.fullmatch(r'[0-9a-f]{64}\.json\.gz', path.name) and path.name[:-8] not in identities:
            path.unlink()
    (cache_root/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    output = QT/'Docs/route_journal_release_20261006.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
