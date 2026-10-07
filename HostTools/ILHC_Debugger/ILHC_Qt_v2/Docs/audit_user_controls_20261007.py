"""三站约40mm实体间隙校准后的整场、量化批次和固定缓存验收，无串口。"""
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
from unittest.mock import patch

QT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(QT))
import competition_simulation as competition
import core
from coordinate_navigation import wrap
from hardware_coordinates import make_coordinate_batch
from route_store import ROOT, algorithm_revision, clear_memory
from tests.test_competition_simulation import runner_for, advance

REAL_CONTROL = dict(kpx=1.5,kpy=1.5,kpz=9.05,xyvmax=930,zvmax=750,xyvmin=.5,zvmin=.5)


def main():
    output = QT/'Docs/user_controls_release_20261007.json'
    report = dict(status='RUNNING', physical_motion_verified=False, nano_verified=False,
                  platform=platform.platform(), algorithm=algorithm_revision(), runs=[])
    identities = set()
    try:
        for name in ('competition_map', 'navigation_map'):
            candidate = QT/f'{name}.json'
            data, added = competition.with_competition_defaults(competition.load_profile(candidate))
            assert not added
            for turn_mode in ('MOVING','WHEEL','STOP_TURN'):
                data['turn_mode']=turn_mode
                for label, control in (('DEFAULT', core.CHASSIS_DEFAULTS), ('REAL', REAL_CONTROL)):
                    for zone in (1,):
                        print(name, turn_mode, label, zone, 'precompute', flush=True)
                        started = time.perf_counter()
                        match = competition.compile_match(data, zone=zone, coordinate_mode=True,
                                                           chassis_control=control)
                        cold = time.perf_counter()-started
                        stats = match['planning_stats']
                        clear_memory()
                        with patch('coordinate_navigation.plan_route', side_effect=AssertionError('固定路线不得搜索')):
                            started = time.perf_counter()
                            match = competition.compile_match(data, zone=zone, coordinate_mode=True,
                                                               chassis_control=control)
                            disk = time.perf_counter()-started
                            assert match['planning_stats']['search_calls'] == 0
                            started = time.perf_counter()
                            memory_match = competition.compile_match(data, zone=zone, coordinate_mode=True,
                                                                      chassis_control=control)
                            memory = time.perf_counter()-started
                            assert memory_match['planning_stats']['search_calls'] == 0
                        identities.update(leg['route']['route_reuse']['identity'] for leg in match['legs'])
                        started = time.perf_counter()
                        batch = make_coordinate_batch(match, (0,0,0), competition.collision_scene(data), token=zone)
                        batch_s = time.perf_counter()-started
                        assert batch['required_coordinate_caps'] == 8
                        assert match['origin_mode'] == 'MEASURED_OPS_ZERO'
                        assert core.layout_to_field(*match['home']) == (0,0)
                        assert batch['points'][0][:2] == (0,0)
                        assert batch['points'][-1][:2] == (0,0)
                        assert batch['points'][-1][3] == 0
                        runner = runner_for(match)
                        stops = []
                        while runner.active:
                            stage = runner.stage
                            if runner.status == 'ACTION' and 'work_station' in stage:
                                pose = runner._field_pose(runner.sim.navigation_snapshot())
                                station = stage['work_station']
                                error = math.dist(pose[:2], core.layout_to_field(*data['competition']['stations'][station]))
                                yaw_error = abs(wrap(-90-pose[2]-stage['work_heading_deg']))
                                assert error < 1 and yaw_error < 1, (station,error,yaw_error)
                                if not stops or stops[-1]['station'] != station:
                                    stops.append(dict(station=station, error_mm=error, yaw_error_deg=yaw_error))
                            advance(runner)
                            assert runner.elapsed_s < 300, runner.reason
                        assert runner.status == 'COMPLETE', runner.reason
                        assert {s['station'] for s in stops} == {'raw','rough','storage'}
                        row = dict(map=name+'.json',control=label,zone=zone,turn_mode=turn_mode,
                            initial_plan_s=cold,initial_planning_stats=stats,disk_plan_s=disk,memory_plan_s=memory,
                            cached_search_calls=0,batch_s=batch_s,point_count=batch['point_count'],
                            modeled_round_s=runner.elapsed_s,work_stops=stops,
                            rough_to_storage_skeleton=match['legs'][3]['route']['skeleton_points'])
                        report['runs'].append(row)
                        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
                        print(json.dumps({k:v for k,v in row.items() if k!='work_stops'},ensure_ascii=False),flush=True)
        report['status'] = 'PASSED'
        manifest = dict(schema_version=1,algorithm=algorithm_revision(),date='2026-10-07',
                        maps=['competition_map.json','navigation_map.json'], zones=[1],turn_modes=['MOVING','WHEEL','STOP_TURN'],margin_mm=10,
                        footprint_mm=[core.CAR_LENGTH_MM,core.CAR_WIDTH_MM],
                        chassis_control=core.CHASSIS_DEFAULTS,additional_chassis_control_profiles=[REAL_CONTROL],
                        station_field_cm={'raw':[10,105],'rough':[190,105],'storage':[110,190]}, routes=[])
        for identity in sorted(identities):
            path = ROOT/(identity+'.json.gz')
            manifest['routes'].append(dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        report['route_count'] = len(manifest['routes'])
        (QT/'Docs/user_controls_manifest_20261007.json').write_text(
            json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    except Exception as exc:
        report.update(status='FAILED',error=repr(exc))
        raise
    finally:
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
