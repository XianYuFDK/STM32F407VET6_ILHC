"""复核 19:31 实机闭合运行；生成派生报告，不改原始日志或运动控制。"""
import collections
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

QT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(QT))
import competition_simulation as competition
import coordinate_navigation as coordinate
import pivot_turns


def wrap(angle):
    return (angle + 180) % 360 - 180


def main():
    session = QT / 'records/runs/20261006_193116_859875_REAL_56ce1058'
    rid = '852e5b168c7a4f76964180cb4a71dcbe'
    directory = session / 'runs' / rid
    start = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))
    result = json.loads((directory / 'result.json').read_text(encoding='utf-8'))
    plan = json.loads((directory / 'plan.json').read_text(encoding='utf-8'))
    metadata = start['metadata']
    rows = []
    with (directory / 'analysis.csv').open(encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            numeric = ('time_s', 'ops_x_mm', 'ops_y_mm', 'ops_yaw_deg', 'dt_s',
                       'speed_mm_s', 'yaw_rate_deg_s', 'cursor', 'status_age_s',
                       'model_position_error_mm', 'model_yaw_error_deg')
            rows.append(dict(row, **{k: float(row[k]) if row[k] else None for k in numeric}))
    snapshot_size = (session / 'events.jsonl').stat().st_size
    with (session / 'events.jsonl').open('rb') as stream:
        snapshot = stream.read(snapshot_size)
    events = []
    incomplete = 0
    for line in snapshot.splitlines():
        try:
            event = json.loads(line)
        except (ValueError, UnicodeDecodeError):
            incomplete += 1
            continue
        if event.get('run_id') == rid:
            events.append(event)
    frames = [e for e in events if e['event'] == 'FRAME']
    transport = [e for e in events if e['event'] == 'TRANSPORT']
    controls = []
    transitions = []
    for event in events:
        if event['event'] == 'BATCH_STATE':
            if not transitions or (event['state'], event['cursor']) != (transitions[-1]['state'], transitions[-1]['cursor']):
                transitions.append(dict(t=event['monotonic']-start['monotonic'],
                                        state=event['state'], cursor=event['cursor']))
        if event['event'] == 'RX_TEXT' and event.get('text', '').startswith('CCTRL '):
            data = list(map(int, event['text'].split()[1:]))
            assert data[0] == plan['id']
            controls.append(dict(t=event['monotonic']-start['monotonic'], target=data[1],
                                 flags=data[2], goal=data[3]/100, gap=data[4]/10,
                                 vx=data[5]/10, vy=data[6]/10, request=data[7]/100,
                                 measured=data[8]/100, settled=data[9]))
    assert len(frames) == len(rows) == 6183
    assert result['status'] == 'DONE' and transitions[-1]['cursor'] == 29
    delta = {k: transport[-1][k]-transport[0][k] for k in (
        'crc_errors', 'lost_packets', 'duplicate_packets', 'invalid_pose_frames',
        'device_restarts', 'gui_queue_drops', 'observer_errors')}
    ticks = [f['device_tick_ms'] for f in frames]
    tick_span = (ticks[-1]-ticks[0])/1000
    tx = [e for e in events if e['event'] == 'TX']
    rx_bytes = sum(len(bytes.fromhex(e['bytes_hex'])) for e in events if e['event'] == 'RX_BYTES')
    duration = result['monotonic']-start['monotonic']
    running = [s for s in transitions if s['state'] in ('RUNNING', 'DONE')]
    boundaries = {s['cursor']: s['t'] for s in running}
    point_stats = []
    for target in range(1, len(plan['points'])):
        part = [c for c in controls if c['target'] == target]
        changes = [dict(t=b['t'], previous=a['goal'], goal=b['goal'], gap=b['gap'])
                   for a, b in zip(part, part[1:]) if abs(wrap(b['goal']-a['goal'])) > 5]
        near = next((c for c in part if c['gap'] < 2), None)
        point_stats.append(dict(target=target, point=plan['points'][target],
            observed_start_s=boundaries.get(target-1), observed_end_s=boundaries.get(target),
            observed_duration_s=boundaries[target]-boundaries[target-1],
            goal_changes=changes, first_under_2mm_s=near['t'] if near else None,
            near_2mm_to_status_completion_s=boundaries[target]-near['t'] if near else None))
    pivots = []
    model_groups = []
    for sample in plan['display_points']:
        if sample['segment_type'] == 'PIVOT':
            if not model_groups or not model_groups[-1]['open']:
                model_groups.append(dict(open=True, samples=[]))
            model_groups[-1]['samples'].append(sample)
        elif model_groups:
            model_groups[-1]['open'] = False
    for target in (8, 19):
        part = [r for r in rows if r['cursor'] == target-1 and r['execution_state'] == 'RUNNING'
                and r['status_age_s'] is not None and 0 <= r['status_age_s'] <= .35]
        end, entry = plan['points'][target], plan['points'][target-1]
        normalized = [wrap(r['ops_yaw_deg']-end[3]/100) for r in part]
        crossing = next(i for i, angle in enumerate(normalized) if angle >= 0)
        near = next((r for r, angle in zip(part, normalized) if abs(angle) < 1), None)
        tail = part[next(i for i, angle in enumerate(normalized) if abs(angle) < 10):]
        signs = []
        for row in tail:
            if row['yaw_rate_deg_s'] is not None and abs(row['yaw_rate_deg_s']) >= 5:
                sign = math.copysign(1, row['yaw_rate_deg_s'])
                if not signs or sign != signs[-1]:
                    signs.append(sign)
        anchor_errors = []
        cx, cy = end[8]/10, end[9]/10
        dx, dy = entry[0]/10-cx, entry[1]/10-cy
        for row in part:
            angle = math.radians(wrap(row['ops_yaw_deg']-entry[3]/100))
            rx = cx+math.cos(angle)*dx+math.sin(angle)*dy
            ry = cy-math.sin(angle)*dx+math.cos(angle)*dy
            anchor_errors.append(math.dist((rx, ry), (row['ops_x_mm'], row['ops_y_mm'])))
        active_controls = [c for c in controls if c['target'] == target]
        recovery = next(c for c in active_controls if c['flags'] & 8)
        pivots.append(dict(target=target, start_s=boundaries[target-1], end_s=boundaries[target],
            observed_duration_s=boundaries[target]-boundaries[target-1],
            model_duration_s=len(model_groups[len(pivots)]['samples'])/50,
            first_under_1deg_s=near['time_s'],
            position_gap_when_first_under_1deg_mm=math.dist((near['ops_x_mm'], near['ops_y_mm']), (end[0]/10, end[1]/10)),
            overshoot_deg=max(normalized), tail_rate_reversals=max(0, len(signs)-1),
            recovery_start_s=recovery['t'], recovery_to_status_completion_s=boundaries[target]-recovery['t'],
            after_first_cross_span_deg=max(normalized[crossing:])-min(normalized[crossing:]),
            inferred_anchor_max_mm=max(anchor_errors),
            inferred_anchor_rms_mm=math.sqrt(statistics.mean(e*e for e in anchor_errors))))
    scene = competition.collision_scene(metadata['map_snapshot'], plan['margin_mm'], metadata['obstacles'])
    bare = competition.collision_scene(metadata['map_snapshot'], 0, metadata['obstacles'])
    violation_indices = []
    bare_violations = []
    clearance = []
    interval_violations = []
    previous = None
    for index, row in enumerate(rows):
        pose = (2250-row['ops_y_mm'], 2250-row['ops_x_mm'], row['ops_yaw_deg']+180)
        if scene.pose_reason(*pose):
            violation_indices.append(index)
        if bare.pose_reason(*pose):
            bare_violations.append(index)
        clearance.append(bare.sweep_clearance(bare.pose_polygon(*pose)))
        if previous is not None and row['dt_s'] is not None and scene.moving_pose_reason(previous, pose):
            interval_violations.append(index)
        previous = pose
    groups = []
    for index in violation_indices:
        if not groups or index > groups[-1][-1]+1:
            groups.append([])
        groups[-1].append(index)
    safe_stats = dict(rectangular_pad_mm=plan['margin_mm'], pose_violations=len(violation_indices),
        interval_violations=len(interval_violations), bare_body_pose_violations=len(bare_violations),
        minimum_sampled_body_clearance_mm=min(clearance),
        windows=[dict(from_s=rows[g[0]]['time_s'], to_s=rows[g[-1]]['time_s'], samples=len(g),
                      cursors=sorted({rows[i]['cursor'] for i in g}),
                      minimum_body_clearance_mm=min(clearance[i] for i in g)) for g in groups])
    legs = []
    intervals = ((1, 2), (2, 4), (4, 9), (9, 13), (13, 15), (15, 20), (20, 24), (24, 27))
    for leg, (a, b) in zip(plan['match']['legs'], intervals):
        route = leg['route']
        legs.append(dict(label=leg['label'], from_point=a, to_point=b,
            observed_s=boundaries[b]-boundaries[a], model_s=route['predicted_tracking_s'],
            skeleton=route['skeleton_points'], policy=route['optimality']['policy'],
            reuse=route['route_reuse']))
    # 比较同一地图、两障碍、车体、裕量、参数和右侧塔吊到站角；仅离线，不发布执行点表。
    alternative_program = coordinate.build_program([(1200, 400), (400, 400), (400, 1200)],
        0, goal_yaw=270, pass_mm=5, mode='GOAL_AWARE',
        control=dict(plan['chassis_control'], turn_lead_mm=40))
    alternative_program, _ = pivot_turns.corner_trial(alternative_program, 1,
        pivot_turns.wheel_geometry(scene), scene, lambda: False)
    alternative_samples, alternative_time = coordinate.replay(alternative_program, scene)
    original_program = plan['match']['legs'][3]['route']['waypoint_program']
    original_samples, original_time = coordinate.replay(original_program, scene)
    alternative = dict(verified_model_only=True, identical_recorded_scene=True,
        original_model_s=original_time, alternative_model_s=alternative_time,
        original_cost=coordinate.execution_cost(original_program, original_samples, original_time),
        alternative_cost=coordinate.execution_cost(alternative_program, alternative_samples, alternative_time),
        alternative_program=alternative_program,
        measured_real_alternative_s=None)
    hashes = {name: hashlib.sha256((directory/name).read_bytes()).hexdigest()
              for name in ('metadata.json', 'plan.json', 'result.json', 'telemetry.csv')}
    summary = dict(session=str(session), run_id=rid, physical_data=True,
        user_confirmed_transport='JDY-31 Bluetooth SPP', pc_version=metadata['pc_version'],
        firmware_caps_reply=[e['text'] for e in events if e['event']=='RX_TEXT' and e['text'].startswith('CCAPS ')],
        firmware_binary_verified=False, duration_s=duration, running_status_s=duration-boundaries[0],
        model_round_s=len(plan['display_points'])/50, points=len(plan['points']),
        frames=len(frames), frame_protocols=dict(collections.Counter(e['protocol'] for e in frames)),
        accepted_frames_per_device_second=(len(frames)-1)/tick_span,
        tick_span_s=tick_span, tick_gap_ms=dict(collections.Counter(b-a for a,b in zip(ticks,ticks[1:]))),
        transport_delta=delta, rx_bytes_per_run_second=rx_bytes/duration,
        tx_successful=all(e.get('successful') for e in tx),
        tx_types=dict(collections.Counter(e['command'].split('=')[0].split(' ')[0] for e in tx)),
        tx_duration_available=any('write_elapsed_s' in e for e in tx),
        params=plan['chassis_control'], extra_obstacles=metadata['obstacles'],
        planning_stats=plan['match']['planning_stats'], runtime_collision_protection=metadata['runtime_collision_protection'],
        diagnostics=[dict(t=e['monotonic']-start['monotonic'],text=e['text']) for e in events
                     if e['event']=='RX_TEXT' and not e['text'].startswith(('TSTAT ', 'CCTRL '))],
        pivots=pivots, legs=legs, point_statistics=point_stats, safety=safe_stats,
        alternative_rough_storage=alternative,
        final_ops_mm_deg=[rows[-1]['ops_x_mm'],rows[-1]['ops_y_mm'],rows[-1]['ops_yaw_deg']],
        events_snapshot_bytes=snapshot_size, snapshot_incomplete_lines=incomplete,
        closed_run_file_sha256=hashes,
        limitations=['TSTAT/CCTRL以接收时间记录，约有0.2秒采样和传输错位；区段时间是观测近似。',
            'OPS位姿及其差分不是外部实车位置或四轮编码器；安全复核受地图和定位准确性限制。',
            '10mm裕量是车体各局部轴矩形膨胀，旋转后不等同于外缘欧氏距离10mm。',
            '候选路线模型通过不证明实车用时或全局最优；本次不改变控制、地图、参数，不连接串口。',
            '本版TX没有耗时字段，不能从发包间隔推断每次write耗时。'])
    destination = QT / 'Docs/real_run_20261006_193128.json'
    destination.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    plot(summary, rows, controls, plan, alternative_samples, violation_indices)
    print(json.dumps({k:summary[k] for k in ('duration_s','frames','transport_delta','pivots','safety','final_ops_mm_deg')}, ensure_ascii=False, indent=2))


def plot(summary, rows, controls, plan, alternative, violations):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(3, 2, figsize=(14, 13), constrained_layout=True)
    ax = axes[0, 0]
    for x0, y0, x1, y1, _ in plan['match']['map_snapshot']['rects']:
        ax.add_patch(Rectangle((2250-y1,2250-x1),y1-y0,x1-x0,color='#b9afa3'))
    for x, y in summary['extra_obstacles']:
        ax.add_patch(Circle((2250-y,2250-x),plan['match']['obstacle_radius_mm'],color='#9167a6',alpha=.6))
    ax.plot([p['x_mm'] for p in plan['display_points']], [p['y_mm'] for p in plan['display_points']],
            '--',color='#adadad',label='日志内原模型',lw=1)
    ax.plot([r['ops_x_mm'] for r in rows],[r['ops_y_mm'] for r in rows],color='#1678a3',label='实际OPS',lw=1)
    ax.scatter([rows[i]['ops_x_mm'] for i in violations],[rows[i]['ops_y_mm'] for i in violations],
               s=8,color='#d24d36',label='侵入矩形安全裕量')
    ax.plot([p['x_mm'] for p in alternative],[p['y_mm'] for p in alternative],color='#338944',lw=2,label='同场景倒行候选（仅模型）')
    ax.set(xlabel='场地 X左 / mm',ylabel='场地 Y前 / mm',title='本轮路径及裕量复核')
    ax.invert_xaxis();ax.set_aspect('equal');ax.legend(fontsize=8)
    ax = axes[0, 1]
    labels = [l['label'].replace('第','').replace('前往','') for l in summary['legs']]
    positions = list(range(len(labels)))
    ax.barh([i-.18 for i in positions],[l['model_s'] for l in summary['legs']],.35,label='日志模型')
    ax.barh([i+.18 for i in positions],[l['observed_s'] for l in summary['legs']],.35,label='实机观测')
    ax.set_yticks(positions,labels,fontsize=8);ax.invert_yaxis()
    ax.set(xlabel='路段耗时 / s',title='路段耗时：实机均慢于理想模型');ax.legend(fontsize=8)
    for pivot, ax in zip(summary['pivots'],axes[1]):
        target = pivot['target'];origin=pivot['start_s']
        part = [r for r in rows if r['cursor']==target-1]
        ctrl = [c for c in controls if c['target']==target]
        ax.plot([r['time_s']-origin for r in part],[wrap(r['ops_yaw_deg']+180) for r in part],label='出口航向误差 / °')
        ax.plot([c['t']-origin for c in ctrl],[c['request'] for c in ctrl],label='MCU角速度请求 / °每秒')
        ax.plot([c['t']-origin for c in ctrl],[c['measured'] for c in ctrl],label='MCU测量角速度 / °每秒',alpha=.65)
        ax.axvspan(pivot['recovery_start_s']-origin,pivot['end_s']-origin,color='#efc476',alpha=.3,label='出口恢复')
        ax.axhline(0,color='gray',lw=.7)
        ax.set(xlabel='进入绕轮后 / s',title=f'绕轮点{target}：超调{pivot["overshoot_deg"]:.2f}°，纠偏反向一次')
        ax.legend(fontsize=7)
    ax = axes[2,0]
    t0=summary['point_statistics'][9]['observed_start_s'];t1=summary['point_statistics'][12]['observed_end_s']
    part=[r for r in rows if t0<=r['time_s']<=t1]
    ctrl=[c for c in controls if 10<=c['target']<=13]
    def unwrap(values):
        result=[values[0]]
        for v in values[1:]:result.append(result[-1]+wrap(v-result[-1]))
        return result
    angles = unwrap([r['ops_yaw_deg'] for r in part])
    times = [r['time_s'] for r in part]
    goals = []
    for c in ctrl:
        index = min(bisect.bisect_left(times, c['t']), len(part)-1)
        goals.append(angles[index]+wrap(c['goal']-part[index]['ops_yaw_deg']))
    ax.plot([r['time_s']-t0 for r in part],angles,label='实际OPS航向')
    ax.step([c['t']-t0 for c in ctrl],goals,where='post',label='MCU目标航向')
    ax.set(xlabel='离开粗加工后 / s',ylabel='展开航向 / °',title='第1次到暂存：多次转头来自规划目标切换');ax.legend(fontsize=8)
    ax=axes[2,1]
    ax.plot([r['time_s'] for r in rows],[r['speed_mm_s'] for r in rows],lw=.6,color='#1678a3')
    for p in summary['pivots']:
        ax.axvspan(p['start_s'],p['end_s'],color='#efc476',alpha=.5)
    ax.set(xlabel='RUN_START后 / s',ylabel='OPS位置差分速度 / mm每秒',title='实际速度：点位末端慢收敛，非匀速路径')
    for ax in axes.flat:ax.grid(alpha=.18)
    fig.suptitle('19:31 JDY-31实机分析：原始6183帧；OPS不是外部位置真值',fontsize=14)
    fig.savefig(QT/'Docs/real_run_20261006_193128.png',dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
