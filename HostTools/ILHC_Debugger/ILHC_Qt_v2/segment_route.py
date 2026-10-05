"""PC端点/圆弧执行格式；密集采样仅用于完整车体复检，不进入执行器。"""
import copy
import json
from pathlib import Path

from trajectory import _normalize, generate_trajectory
from trajectory_tracking import SegmentTracker

KIND = 'PC_SEGMENT_PROGRAM'


def build_segment_program(primitives):
    pieces = _normalize(primitives)
    if not pieces:
        raise ValueError('没有直线或圆弧段')
    segments = []
    for piece in pieces:
        if piece['kind'] == 'LINE':
            row = dict(kind='LINE', start=list(piece['start']), end=list(piece['end']))
        else:
            row = dict(kind='ARC', entry=list(piece['start']), exit=list(piece['end']),
                       center=list(piece['center']), radius_mm=piece['radius_mm'],
                       start_angle_deg=piece['start_angle_deg'], sweep_deg=piece['sweep_deg'],
                       direction='CCW' if piece['sweep_deg'] > 0 else 'CW')
        if piece['explicit_heading']:
            row['heading_mode'] = piece['heading_mode']
            if piece['heading_mode'] == 'FIXED':
                row['body_yaw_deg'] = piece['yaw_in']
        if segments and row['kind'] == segments[-1]['kind'] == 'LINE':
            previous = segments[-1]
            a, b, c = previous['start'], previous['end'], row['end']
            ux, uy, vx, vy = b[0]-a[0], b[1]-a[1], c[0]-b[0], c[1]-b[1]
            same_heading = (previous.get('heading_mode') == row.get('heading_mode') and
                            previous.get('body_yaw_deg') == row.get('body_yaw_deg'))
            if same_heading and abs(ux*vy-uy*vx) <= 1e-7*(ux*ux+uy*uy+vx*vx+vy*vy) and ux*vx+uy*vy > 0:
                previous['end'] = row['end']
                continue
        segments.append(row)
    tracker = SegmentTracker(segments)
    return dict(schema_version=1, kind=KIND, frame_id='LAYOUT_MM', final_stop=True,
                segments=segments, length_mm=tracker.length,
                start=tracker.reference_at(0), goal=tracker.reference_at(tracker.length))


def program_primitives(program):
    if not isinstance(program, dict) or program.get('schema_version') != 1 or \
            program.get('kind') != KIND or program.get('frame_id') != 'LAYOUT_MM' or \
            program.get('final_stop') is not True:
        raise ValueError('端点/圆弧程序格式或最终STOP非法')
    rows = program.get('segments')
    if not isinstance(rows, list) or not 0 < len(rows) <= 4096:
        raise ValueError('端点/圆弧程序需要1..4096段')
    # 不信任导出时的start/goal/length安全标记，执行时始终从几何重新计算。
    return copy.deepcopy(rows)


def validate_segment_program(program, scene, interrupted=lambda: False):
    rows = program_primitives(program)
    result = generate_trajectory(rows, scene, interrupted=interrupted, max_arc_angle_deg=3)
    if not result['trajectory_safe']:
        raise ValueError('端点/圆弧整车复检失败：'+result['trajectory_reason'])
    return rows


def export_segment_json(path, program, *, metadata=None):
    canonical = build_segment_program(program_primitives(program))
    canonical.update(hardware_ready=False)
    if metadata is not None:
        canonical['metadata'] = copy.deepcopy(metadata)
    Path(path).write_text(json.dumps(canonical, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
