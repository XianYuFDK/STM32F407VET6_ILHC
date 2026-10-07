"""新作业点离线检查；保留原设备轮廓，不修改默认地图或发送指令。"""
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import time

QT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(QT))
import competition_simulation as competition
import core
from coordinate_navigation import plan_route, replay, wrap
from work_orientation import DEFAULT_WORK_AREAS, station_at, work_heading

FIELD_CM = {'raw': (10, 105), 'rough': (200, 105), 'storage': (110, 190)}
REAL_CONTROL = dict(kpx=1.5, kpy=1.5, kpz=9.05, xyvmax=930,
                    zvmax=750, xyvmin=.5, zvmin=.5)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    output = QT/'Docs/station_calibration_checks_20261007.json'
    report = json.loads(output.read_text(encoding='utf-8'))
    report['verification'] = []
    for row in report['maps']:
        source = QT/row['source']
        before = sha(source)
        original, _ = competition.with_competition_defaults(competition.nav.load_map(source))
        calibrated = competition.load_profile(QT/row['calibrated_profile'])
        # 配置确实使用重新标定的坐标；固定禁区和自定义配置不被替换。
        configured, added = competition.with_competition_defaults(calibrated)
        assert not added and configured == calibrated
        for key in ('rects', 'circles', 'bounds', 'drivable_polygons'):
            assert original[key] == calibrated[key], key
        for key in ('start_zones', 'staging'):
            assert original['competition'][key] == calibrated['competition'][key], key
        assert {k: list(v) for k,v in original['competition'].get('work_areas', DEFAULT_WORK_AREAS).items()} == calibrated['competition']['work_areas']
        for name, field in FIELD_CM.items():
            point = calibrated['competition']['stations'][name]
            assert core.layout_to_field(*point) == tuple(v*10 for v in field)
            assert station_at(calibrated['competition'], point) == name
            assert station_at(calibrated['competition'], original['competition']['stations'][name]) is None
            yaw = work_heading(calibrated['competition'], name)
            dx, dy = [a-b for a, b in zip(configured['competition']['work_areas'][name], point)]
            right = math.cos(math.radians(yaw-90)), math.sin(math.radians(yaw-90))
            assert abs((right[0]*dx+right[1]*dy)/math.hypot(dx,dy)-1) < 1e-12
        # 新地图原料停靠点不可行，应明确拒绝整轮，不能产出可下发批次。
        with tempfile.TemporaryDirectory(prefix='ilhc-calibration-') as cache:
            started = time.perf_counter()
            try:
                competition.compile_match(calibrated, coordinate_mode=True, route_cache_dir=cache)
            except ValueError as exc:
                reason = str(exc)
                assert '原料区圆盘' in reason, reason
            else:
                raise AssertionError('当前几何冲突不允许生成整轮路线')
        row['full_match_rejection'] = dict(seconds=time.perf_counter()-started, reason=reason)
        # 单独验证已合法的暂存新点；起点仍是原粗加工合法点，不能冒充三点整场验证。
        scene = competition.collision_scene(calibrated)
        config = calibrated['competition']
        start = original['competition']['stations']['rough']
        goal = config['stations']['storage']
        for label, control in (('DEFAULT', core.CHASSIS_DEFAULTS), ('REAL', REAL_CONTROL)):
            started = time.perf_counter()
            route = plan_route(start, goal, scene, config['lane_nodes'],
                               work_heading(original['competition'], 'rough'),
                               goal_yaw=work_heading(config, 'storage'), chassis_control=control,
                               deadline=time.monotonic()+5, interactive=False)
            samples, elapsed = replay(route['waypoint_program'], scene)
            last = samples[-1]
            end = core.field_to_layout(last['x_mm'], last['y_mm'])
            assert math.dist(end, goal) < 1
            assert abs(wrap(-90-last['field_yaw_deg']-work_heading(config, 'storage'))) < 1
            report['verification'].append(dict(map=row['source'], scope='STORAGE_LEG_ONLY',
                control=label, start_is_old_safe_rough_point=True, plan_wall_s=time.perf_counter()-started,
                modeled_travel_s=elapsed, end_error_mm=math.dist(end, goal),
                end_yaw_error_deg=abs(wrap(-90-last['field_yaw_deg']-work_heading(config, 'storage')))))
        assert sha(source) == before
        row['source_sha256'] = before
        row['calibrated_profile_sha256'] = sha(QT/row['calibrated_profile'])
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
