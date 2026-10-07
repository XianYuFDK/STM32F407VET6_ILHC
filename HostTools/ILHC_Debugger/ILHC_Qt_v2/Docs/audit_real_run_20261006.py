"""本次实机日志复核；只生成派生报告，不连接串口、不修改运动参数。"""
import argparse
import collections
import csv
import json
import math
from pathlib import Path
import statistics


def wrap(value):
    return (value + 180) % 360 - 180


def unwrap_series(values):
    result = [values[0]]
    for value in values[1:]:
        result.append(result[-1] + wrap(value-result[-1]))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent)
    args = parser.parse_args()
    session = args.session.resolve()
    analysis = json.loads((session / 'analysis.json').read_text(encoding='utf-8'))
    if len(analysis['runs']) != 1 or analysis['source'] != 'REAL':
        raise ValueError('本次复核要求一个 REAL 运行；请先运行 analyze_run.py')
    rid = analysis['runs'][0]['run_id']
    directory = session / 'runs' / rid
    plan = json.loads((directory / 'plan.json').read_text(encoding='utf-8'))
    start = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))
    result = json.loads((directory / 'result.json').read_text(encoding='utf-8'))
    rows = []
    with (directory / 'analysis.csv').open(encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            rows.append({k: float(v) if v and k not in (
                'execution_state', 'nominal_phase', 'station', 'reference_kind',
                'reference_switch', 'rate_window_valid') else v for k, v in row.items()})
    transport = []
    diagnostics = []
    frames = []
    commands = collections.Counter()
    transitions = []
    # 连接可能仍在记录。按文件大小读取快照；本轮 RUN_END 已闭合。
    snapshot_bytes = (session / 'events.jsonl').stat().st_size
    with (session / 'events.jsonl').open('rb') as stream:
        remaining = snapshot_bytes
        while remaining > 0:
            line = stream.readline(remaining)
            remaining -= len(line)
            if not line:
                break
            try:
                event = json.loads(line)
            except (ValueError, UnicodeDecodeError):
                continue
            if event.get('run_id') != rid:
                continue
            kind = event['event']
            if kind == 'TRANSPORT':
                transport.append(event)
            elif kind == 'FRAME':
                frames.append(event)
            elif kind == 'TX':
                commands[event['command']] += 1
            elif kind == 'RX_TEXT' and not event.get('text', '').startswith('TSTAT'):
                diagnostics.append(dict(t=event['monotonic'] - start['monotonic'], text=event['text']))
            elif kind == 'BATCH_STATE':
                pair = (event.get('state'), event.get('cursor'))
                if not transitions or pair != (transitions[-1]['state'], transitions[-1]['cursor']):
                    transitions.append(dict(state=pair[0], cursor=pair[1],
                                            t=event['monotonic'] - start['monotonic']))
    assert len(frames) == len(rows) == analysis['runs'][0]['frame_count']
    assert result['status'] == 'DONE' and transitions[-1]['cursor'] == len(plan['points']) - 1
    delta = {key: transport[-1][key] - transport[0][key] for key in (
        'crc_errors', 'lost_packets', 'duplicate_packets', 'invalid_pose_frames',
        'device_restarts', 'gui_queue_drops', 'observer_errors')}
    params = {key: [min(f['values'][index] for f in frames), max(f['values'][index] for f in frames)]
              for key, index in [('kpx', 6), ('kpy', 7), ('kpz', 8), ('xyvmax', 9), ('zvmax', 10)]}
    selected = lambda cursor: [row for row in rows if row['cursor'] == cursor]
    pivot_stats = []
    model_groups = []
    for point in plan['display_points']:
        if point['segment_type'] == 'PIVOT':
            center = point['pivot_center_mm']
            if not model_groups or center != model_groups[-1]['center'] or not model_groups[-1]['open']:
                model_groups.append(dict(center=center, samples=[], open=True))
            model_groups[-1]['samples'].append(point)
        elif model_groups:
            model_groups[-1]['open'] = False
    for cursor in (8, 11, 16, 19):
        data = selected(cursor)
        q, previous = plan['points'][cursor + 1], plan['points'][cursor]
        assert q[4] & 32
        desired = q[3] / 100
        cx, cy = q[8] / 10, q[9] / 10
        vx, vy = previous[0] / 10 - cx, previous[1] / 10 - cy
        anchor_errors = []
        for row in data:
            angle = math.radians(wrap(row['ops_yaw_deg'] - previous[3] / 100))
            rx = math.cos(angle) * vx + math.sin(angle) * vy
            ry = -math.sin(angle) * vx + math.cos(angle) * vy
            anchor_errors.append(math.hypot(row['ops_x_mm'] - cx - rx, row['ops_y_mm'] - cy - ry))
        tail = data[next(i for i, row in enumerate(data) if abs(wrap(row['ops_yaw_deg'] - desired)) < 10):]
        minimum_yaw, maximum_yaw = min(r['ops_yaw_deg'] for r in tail), max(r['ops_yaw_deg'] for r in tail)
        direction = 1 if wrap(desired-previous[3]/100) > 0 else -1
        first_cross = next((i for i, row in enumerate(data)
                            if direction*wrap(desired-row['ops_yaw_deg']) <= 0), None)
        correction = data[first_cross:] if first_cross is not None else []
        rate_signs = []
        for row in tail:
            rate = row['yaw_rate_deg_s']
            if rate != '' and abs(rate) >= 5:
                sign = 1 if rate > 0 else -1
                if not rate_signs or sign != rate_signs[-1]:
                    rate_signs.append(sign)
        pivot_stats.append(dict(target=cursor + 1, start_s=data[0]['time_s'], end_s=data[-1]['time_s'],
            observed_duration_s=data[-1]['time_s'] - data[0]['time_s'],
            tail_start_s=tail[0]['time_s'], tail_duration_s=tail[-1]['time_s'] - tail[0]['time_s'],
            tail_yaw_min_deg=minimum_yaw, tail_yaw_max_deg=maximum_yaw,
            tail_yaw_peak_to_peak_deg=maximum_yaw - minimum_yaw,
            after_first_cross_min_deg=min((r['ops_yaw_deg'] for r in correction), default=None),
            after_first_cross_max_deg=max((r['ops_yaw_deg'] for r in correction), default=None),
            tail_rate_reversals=len(rate_signs) - 1,
            anchor_max_mm=max(anchor_errors), anchor_rms_mm=math.sqrt(statistics.mean(v*v for v in anchor_errors)),
            model_pivot_s=len(model_groups[len(pivot_stats)]['samples']) / 50))
    stations = []
    for cursor in (9, 12, 17):
        data = selected(cursor)
        first_near = next(row for row in data if row['target_gap_mm'] != '' and row['target_gap_mm'] < 2)
        tail = [row for row in data if row['time_s'] >= data[-1]['time_s'] - 2]
        stations.append(dict(target=cursor + 1, first_under_2mm_s=first_near['time_s'],
            remaining_until_last_observed_s=data[-1]['time_s'] - first_near['time_s'],
            tail_yaw_span_deg=max(r['ops_yaw_deg'] for r in tail) - min(r['ops_yaw_deg'] for r in tail)))
    duration = result['monotonic'] - start['monotonic']
    radius = math.hypot(103, 109)
    summary = dict(session=str(session), run_id=rid, result=result['status'], duration_s=duration,
        events_snapshot_bytes=snapshot_bytes, frame_count=len(frames), point_count=len(plan['points']),
        station_mode=plan['station_mode'], parameters=plan['chassis_control'], telemetry_parameter_ranges=params,
        transport_delta=delta, diagnostics=diagnostics, transitions=transitions,
        pivots=pivot_stats, stations=stations, final_ops_mm_deg=[rows[-1][k] for k in (
            'ops_x_mm', 'ops_y_mm', 'ops_yaw_deg')],
        reconstructed_join_angular_speed_deg_s=math.degrees(80 / radius),
        accepted_tick_gaps_ms=dict(collections.Counter(round(r['dt_s']*1000) for r in rows if r['dt_s'] != '')),
        limits=['TSTAT 与遥测不同步，区段起止约有状态更新延迟；不是精确 MCU 切换时刻。',
                '航向来自 OPS，轮心偏移由 OPS 位姿推算，非外部真值或编码器测量。',
                '模型采用理想轮速响应，图中按进入 PIVOT 相对时间对齐；不含实车惯性、滑移和 OPS 延迟。',
                '四轮实际编码器、控制器实时角速度请求和固件二进制散列未回读。'])
    args.output.mkdir(parents=True, exist_ok=True)
    base = args.output / 'real_run_20261006_173828'
    base.with_suffix('.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(3, 2, figsize=(14, 13), constrained_layout=True)
    ax = axes[0, 0]
    for x0, y0, x1, y1, _ in plan['match']['map_snapshot']['rects']:
        ax.add_patch(Rectangle((2250-y1, 2250-x1), y1-y0, x1-x0, color='#bbb5ac', alpha=.55))
    ax.plot([r['x_mm'] for r in plan['display_points']], [r['y_mm'] for r in plan['display_points']],
            '--', color='#d08020', label='理想响应模型', linewidth=1.5)
    ax.plot([r['ops_x_mm'] for r in rows], [r['ops_y_mm'] for r in rows], color='#1687ad', label='实机 OPS', linewidth=.8)
    ax.invert_xaxis()
    ax.set_aspect('equal')
    ax.set(xlabel='场地 X（左，mm）', ylabel='场地 Y（前，mm）', title='整轮轨迹：23 点 DONE，92.82 秒')
    ax.legend()
    ax = axes[0, 1]
    for cursor, label in [(8, '第一次左上弯'), (16, '第二次左上弯')]:
        data = selected(cursor)
        ax.plot([r['time_s']-data[0]['time_s'] for r in data], unwrap_series([r['ops_yaw_deg'] for r in data]), label=label)
    group = model_groups[0]['samples']
    ax.plot([i/50 for i in range(len(group))], [wrap(90-r['field_yaw_deg']) for r in group], '--', color='black', label='日志内理想模型（约1.20s）')
    ax.axhline(90, color='#999999', linewidth=.8)
    ax.set(xlabel='从首次观测到绕轮阶段起（秒）', ylabel='OPS 航向（°）', title='左上弯：模型一次通过，实机反复越过90°')
    ax.legend()
    for axis, cursor, title in [(axes[1, 0], 8, '第一次：出弯摆动'), (axes[1, 1], 16, '第二次：出弯摆动')]:
        data = selected(cursor)
        tail_start = next(r['time_s'] for r in data if abs(wrap(r['ops_yaw_deg']-90)) < 10)
        data = [r for r in data if r['time_s'] >= tail_start]
        axis.plot([r['time_s'] for r in data], [r['ops_yaw_deg'] for r in data], label='实测航向')
        axis.axhspan(89, 91, alpha=.15, color='green', label='出弯航向门 ±1°')
        axis.axhline(90, color='#777777', linewidth=.8)
        axis.set(xlabel='运行开始后（秒）', ylabel='OPS 航向（°）', title=title)
        axis.set_ylim(84, 100)
        axis.legend()
    ax = axes[2, 0]
    data = [r for r in selected(12) if r['time_s'] > 49.5]
    ax.plot([r['time_s'] for r in data], [r['ops_x_mm']-220 for r in data], label='X 相对目标')
    ax.plot([r['time_s'] for r in data], [r['ops_y_mm']-1050 for r in data], label='Y 相对目标')
    ax.axhspan(-.5, .5, color='green', alpha=.15, label='严格停车带各轴参考 ±0.5mm（实际按径向）')
    ax.set(xlabel='运行开始后（秒）', ylabel='位置偏差（mm）', title='第2次原料区：靠近目标后反复微动，触发 CSTALL')
    ax.set_ylim(-3, 5)
    ax.legend(fontsize=8)
    ax = axes[2, 1]
    for cursor, label in [(9, '第一次暂存'), (17, '第二次暂存')]:
        data = selected(cursor)
        ax.plot([r['time_s']-data[0]['time_s'] for r in data], [r['ops_yaw_deg']-90 for r in data], label=label)
    ax.set(xlabel='出弯后首次观测到直线阶段起（秒）', ylabel='相对90°的航向偏差（°）', title='暂存进站：出弯残余摆动随后收敛')
    ax.axhline(0, color='#777777', linewidth=.8)
    ax.legend()
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.savefig(base.with_suffix('.png'), dpi=145)
    plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
