"""Jetson/PC共用的无GUI规划入口；只输出计划，不连接串口或驱动机构。"""
import argparse
import json
from pathlib import Path
import platform
import sys
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description='右侧塔吊比赛规划/固定路线预计算')
    parser.add_argument('command', choices=('plan', 'precompute', 'benchmark'))
    parser.add_argument('--map', type=Path, default=Path(__file__).with_name('competition_map.json'))
    parser.add_argument('--zone', choices=('1', '2', 'both'), default='both')
    parser.add_argument('--code', default='156+123+516+231')
    parser.add_argument('--control', type=Path, help='七项底盘参数JSON，缺省为模型默认参数')
    parser.add_argument('--obstacles', default='[]', help='布局mm的额外圆柱坐标JSON')
    parser.add_argument('--margin', type=float, default=10)
    parser.add_argument('--cache-dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--batch', action='store_true', help='同时离线编码/预检STM32坐标批次')
    parser.add_argument('--mapping', nargs=3, type=float, default=(0, 0, 0), metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--online-budget', type=float, default=2.0, help='有障碍时每路段搜索预算秒数')
    parser.add_argument('--static-budget', type=float, default=5.0, help='首次固定路线每路段优化预算秒数')
    args = parser.parse_args(argv)
    if sys.version_info < (3, 10):
        parser.error('本规划器使用Python3.10或更新版本；不修改JetPack系统Python')
    if not 0.1 <= args.static_budget <= 60:
        parser.error('static-budget须为0.1..60秒')
    if not 0.1 <= args.online_budget <= 10:
        parser.error('online-budget须为0.1..10秒')
    import competition_simulation as competition
    import core
    from route_store import clear_memory
    data = competition.nav.load_map(args.map)
    data, _ = competition.with_competition_defaults(data)
    obstacles = json.loads(args.obstacles)
    if args.command == 'precompute' and (obstacles or data.get('dynamic_rects') or data.get('dynamic_circles')):
        parser.error('固定路线只允许没有额外圆柱/动态障碍的地图')
    control = dict(core.CHASSIS_DEFAULTS)
    if args.control:
        control = json.loads(args.control.read_text(encoding='utf-8'))
        if set(control) != set(core.CHASSIS_DEFAULTS):
            parser.error('control须完整包含七项底盘参数')
    zones = (1, 2) if args.zone == 'both' else (int(args.zone),)
    result = dict(schema_version=1, platform=platform.platform(), machine=platform.machine(),
                  python=platform.python_version(), physical_motion_verified=False, runs=[])
    for zone in zones:
        passes = ('DISK', 'MEMORY') if args.command == 'benchmark' else (args.command.upper(),)
        for label in passes:
            if label == 'DISK':
                clear_memory()
            started = time.perf_counter()
            match = competition.compile_match(data, args.code, zone, args.margin, sim_obstacles=obstacles,
                    coordinate_mode=True, chassis_control=control, route_cache_dir=args.cache_dir,
                    refresh_static_routes=args.command == 'precompute', online_leg_budget_s=args.online_budget,
                    static_leg_budget_s=args.static_budget)
            wall_s = time.perf_counter()-started
            row = dict(zone=zone, pass_kind=label, plan_wall_s=wall_s, policy=match['route_policy'],
                       sources=[leg['route']['route_reuse']['source'] for leg in match['legs']],
                       search_calls=match['planning_stats']['search_calls'],
                       modeled_travel_s=sum(leg['route']['predicted_tracking_s'] for leg in match['legs']))
            if args.batch:
                from hardware_coordinates import make_coordinate_batch
                started = time.perf_counter()
                batch = make_coordinate_batch(match, args.mapping,
                    competition.collision_scene(data, args.margin, obstacles), token=zone)
                row.update(batch_wall_s=time.perf_counter()-started, point_count=batch['point_count'],
                           crc=batch['crc'], points=batch['points'], required_coordinate_caps=batch['required_coordinate_caps'])
            if args.command == 'plan':
                row['match'] = match
            result['runs'].append(row)
            print(json.dumps({k: v for k, v in row.items() if k not in ('match', 'points')}, ensure_ascii=False), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
