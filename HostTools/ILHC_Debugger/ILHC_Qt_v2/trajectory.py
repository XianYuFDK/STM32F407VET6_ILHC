"""连续几何Trajectory：场地mm、真实车头/行驶切线、全轨迹整车检查；无串口接口。"""
from bisect import bisect_right
import json
import math
from pathlib import Path

from core import layout_to_field, field_to_layout
from navigation_planner import EPS, finite_number, point2

SAMPLE_SPACING_MM = 20.0
MAX_TRAJECTORY_POINTS = 100_000
YAW_CONVENTION = '场地+X为0°、+Y为90°；切线方向，角度unwrap后可超出±180°'
BODY_YAW_CONVENTION = '场地+X为0°、+Y为90°；field_yaw_deg为真实车头，tangent_yaw_deg为行驶切线；分别unwrap'


class TrajectoryError(ValueError):
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code = code


def unwrap_degrees(angles):
    """先归一到[-180,180)，此后用最短角差展开，避免179→-179换周跳变。"""
    result = []
    for angle in angles:
        value = finite_number(angle, '切线航向')
        if result:
            value = result[-1] + (value-result[-1]+180) % 360-180
        else:
            value = (value+180) % 360-180
        result.append(value)
    return result


def _delta(a, b):
    return (b-a+180) % 360-180


def _normalize(primitives):
    """接受LineSegment/ArcSegment对象或LINE/ARC字典，保留真实几何。"""
    pieces = []
    for index, source in enumerate(primitives):
        p = source.as_dict() if hasattr(source, 'as_dict') else dict(source)
        mode = p.get('heading_mode', 'TANGENT')
        if mode not in ('TANGENT', 'REVERSE_TANGENT', 'FIXED'):
            raise TrajectoryError('INVALID_INPUT', '未知车体航向模式：'+str(mode))
        kind = p.get('kind') or ('ARC' if 'radius_mm' in p else 'LINE')
        if kind == 'TURN':
            raise TrajectoryError('FALLBACK_REQUIRED', '第%d段保留原地转向，需要停转；无法生成连续切线Trajectory' % (index+1))
        if kind == 'LINE':
            a, b = point2(p['start']), point2(p['end'])
            length = math.dist(a, b)
            if length <= EPS:
                continue
            yaw = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
            piece = dict(kind=kind, start=a, end=b, length_mm=length,
                         yaw_in=yaw, yaw_out=yaw)
        elif kind == 'ARC':
            radius = finite_number(p['radius_mm'], '圆弧半径')
            sweep = finite_number(p['sweep_deg'], '圆弧扫角')
            angle = finite_number(p['start_angle_deg'], '圆弧起始角')
            if radius <= 0 or not EPS < abs(sweep) < 180 or (mode != 'FIXED' and abs(abs(sweep)-90) > EPS):
                raise TrajectoryError('INVALID_INPUT', '需要正半径圆弧：切线车头模式90°，固定车头模式0..180°')
            sign = 1 if sweep > 0 else -1
            if p.get('direction', 'CCW' if sign > 0 else 'CW') != ('CCW' if sign > 0 else 'CW'):
                raise TrajectoryError('INVALID_INPUT', '圆弧方向与扫角不一致')
            a, b, center = point2(p['entry']), point2(p['exit']), point2(p['center'])
            for endpoint, degrees in ((a, angle), (b, angle+sweep)):
                rad = math.radians(degrees)
                expected = (center[0]+radius*math.cos(rad), center[1]+radius*math.sin(rad))
                if math.dist(endpoint, expected) > 1e-5:
                    raise TrajectoryError('INVALID_INPUT', '圆弧切点/圆心/半径/起始角不一致')
            piece = dict(kind=kind, start=a, end=b, center=center, radius_mm=radius,
                         start_angle_deg=angle, sweep_deg=sweep,
                         length_mm=radius*math.radians(abs(sweep)),
                         yaw_in=angle+sign*90, yaw_out=angle+sweep+sign*90)
        else:
            raise TrajectoryError('INVALID_INPUT', '未知segment_type：%s' % kind)
        piece.update(heading_mode=mode, explicit_heading='heading_mode' in p,
                     tangent_in=piece['yaw_in'], tangent_out=piece['yaw_out'])
        if mode == 'FIXED':
            piece['yaw_in'] = piece['yaw_out'] = finite_number(p['body_yaw_deg'], '真实车体航向')
        elif mode == 'REVERSE_TANGENT':
            piece['yaw_in'] += 180
            piece['yaw_out'] += 180
        if pieces:
            previous = pieces[-1]
            if math.dist(previous['end'], piece['start']) > EPS:
                raise TrajectoryError('DISCONNECTED', '第%d段入口位置不连续' % (index+1))
            if abs(_delta(previous['yaw_out'], piece['yaw_in'])) > 1e-5:
                raise TrajectoryError('FALLBACK_REQUIRED', '第%d段车体切线/航向不连续，需要停转或安全圆弧；不强行连接' % (index+1))
            if abs(_delta(previous['tangent_out'], piece['tangent_in'])) > 1e-5:
                raise TrajectoryError('FALLBACK_REQUIRED', '第%d段行驶切线不连续，不能用固定车头掩盖直角换向' % (index+1))
        pieces.append(piece)
    return pieces


def _pose_at(piece, distance):
    fraction = max(0.0, min(1.0, distance/piece['length_mm']))
    if piece['kind'] == 'LINE':
        a, b = piece['start'], piece['end']
        xy = tuple(a[k]+fraction*(b[k]-a[k]) for k in range(2))
        yaw = piece['yaw_in']
    else:
        offset = fraction*piece['sweep_deg']
        angle = math.radians(piece['start_angle_deg']+offset)
        center, radius = piece['center'], piece['radius_mm']
        xy = (center[0]+radius*math.cos(angle), center[1]+radius*math.sin(angle))
        yaw = piece['yaw_in']+(0 if piece['heading_mode'] == 'FIXED' else offset)
    if distance <= EPS:
        xy = piece['start']
    elif piece['length_mm']-distance <= EPS:
        xy = piece['end']
    return (*xy, yaw)


def _tangent_at(piece, distance):
    return piece['tangent_in']+(piece['sweep_deg']*max(0, min(1, distance/piece['length_mm']))
                              if piece['kind'] == 'ARC' else 0)


def _motion_mode(body, tangent):
    delta = _delta(body, tangent)
    if abs(delta) < 1e-5:
        return 'FORWARD'
    if abs(abs(delta)-180) < 1e-5:
        return 'REVERSE'
    if abs(abs(delta)-90) < 1e-5:
        return 'STRAFE_LEFT' if delta > 0 else 'STRAFE_RIGHT'
    return 'DIAGONAL'


def _stations(pieces, spacing, interrupted, max_arc_angle=None):
    """全程s=0,20,40…；额外保留所有拼接点和终点，间距始终≤spacing。"""
    end, stations, global_index = 0.0, [0.0], 1
    for piece in pieces:
        begin = end
        end += piece['length_mm']
        while global_index*spacing < end-EPS:
            if interrupted():
                raise TrajectoryError('CANCELLED', 'Trajectory采样已取消或超过时限')
            stations.append(global_index*spacing)
            global_index += 1
            if len(stations) >= MAX_TRAJECTORY_POINTS:
                raise TrajectoryError('RESOURCE_LIMIT', 'Trajectory超过100000个采样点')
        stations.append(end)
        if abs(global_index*spacing-end) <= EPS:
            global_index += 1
        if len(stations) > MAX_TRAJECTORY_POINTS:
            raise TrajectoryError('RESOURCE_LIMIT', 'Trajectory超过100000个采样点')
        if max_arc_angle is not None and piece['kind'] == 'ARC':
            n = math.ceil(abs(piece['sweep_deg'])/max_arc_angle)
            if len(stations)+n-1 > MAX_TRAJECTORY_POINTS:
                raise TrajectoryError('RESOURCE_LIMIT', 'Trajectory超过100000个采样点')
            stations.extend(begin+piece['length_mm']*i/n for i in range(1,n))
    return sorted(set(stations))


def _ends(pieces):
    result, distance = [], 0.0
    for p in pieces:
        distance += p['length_mm']
        result.append(distance)
    return result


def _piece_at(pieces, ends, station):
    # 切点仅一行，归属后段；最终端点归属最后一段。
    index = min(bisect_right(ends, station), len(pieces)-1)
    begin = ends[index-1] if index else 0.0
    return pieces[index], station-begin


def _validate(samples, pieces, scene, spacing, interrupted):
    """重新检查导出的真实车体姿态及行驶切线，不复用已缓存的安全标记。"""
    if scene.footprint is None:
        raise TrajectoryError('NO_FOOTPRINT', '缺少真实矩形车体，不能核验Trajectory')
    if not samples or not pieces:
        raise TrajectoryError('EMPTY', '没有可采样的LineSegment/ArcSegment，切线航向未定义')
    ends = _ends(pieces)
    if abs(samples[0]['s_mm']) > EPS or abs(samples[-1]['s_mm']-ends[-1]) > EPS:
        raise TrajectoryError('INVALID_INPUT', 'Trajectory起终点累计弧长不匹配')
    previous_s, previous_yaw, previous_tangent = None, None, None
    for index, sample in enumerate(samples):
        if interrupted():
            raise TrajectoryError('CANCELLED', 'Trajectory整车检查已取消或超过时限')
        x, y, yaw, station = [finite_number(sample[k], k)
                              for k in ('x_mm', 'y_mm', 'field_yaw_deg', 's_mm')]
        if not -EPS <= station <= ends[-1]+EPS:
            raise TrajectoryError('INVALID_INPUT', 'Trajectory弧长超出几何范围')
        if previous_s is not None and not EPS < station-previous_s <= spacing+EPS:
            raise TrajectoryError('INVALID_INPUT', 'Trajectory弧长不递增或采样间隔超过20mm')
        piece, local = _piece_at(pieces, ends, station)
        lx, ly, layout_yaw = _pose_at(piece, local)
        expected_xy = layout_to_field(lx, ly)
        expected_yaw = -90-layout_yaw  # 场地/布局线性变换为(dx,dy)→(-dy,-dx)。
        expected_yaw = unwrap_degrees([expected_yaw])[0] if previous_yaw is None else \
                       previous_yaw+_delta(previous_yaw, expected_yaw)
        tangent = -90-_tangent_at(piece, local)
        tangent = unwrap_degrees([tangent])[0] if previous_tangent is None else previous_tangent+_delta(previous_tangent, tangent)
        if piece['explicit_heading'] or 'tangent_yaw_deg' in sample or 'motion_mode' in sample:
            if (abs(finite_number(sample['tangent_yaw_deg'], '行驶切线')-tangent) > 1e-5 or
                    sample['motion_mode'] != _motion_mode(-90-layout_yaw, -90-_tangent_at(piece, local))):
                raise TrajectoryError('INVALID_INPUT', 'Trajectory真实车头/行驶方向元数据不匹配')
        if (math.dist((x, y), expected_xy) > 1e-5 or abs(yaw-expected_yaw) > 1e-5 or
                sample['segment_type'] != piece['kind']):
            raise TrajectoryError('INVALID_INPUT', 'Trajectory第%d点与原几何切线不匹配' % index)
        px, py = field_to_layout(x, y)
        if not scene.pose_safe(px, py, -90-yaw):
            raise TrajectoryError('UNSAFE', 'Trajectory第%d点：%s' % (index, scene.pose_reason(px, py, -90-yaw)))
        previous_s, previous_yaw, previous_tangent = station, yaw, tangent
    # 完整线段扫掠；圆弧再按≤20mm且≤3°细分，每个姿态和采样间包络均检查。
    for index, piece in enumerate(pieces):
        if interrupted():
            raise TrajectoryError('CANCELLED', 'Trajectory整段扫掠检查已取消或超过时限')
        if piece['kind'] == 'LINE':
            reason = scene.translation_reason(piece['start'], piece['end'], piece['yaw_in'])
            if reason:
                raise TrajectoryError('UNSAFE', 'Trajectory第%d直线扫掠：%s' % (index+1, reason))
        else:
            count = max(math.ceil(piece['length_mm']/spacing), math.ceil(abs(piece['sweep_deg'])/3))
            previous = None
            for i in range(count+1):
                if interrupted():
                    raise TrajectoryError('CANCELLED', 'Trajectory圆弧扫掠检查已取消或超过时限')
                pose = _pose_at(piece, piece['length_mm']*i/count)
                reason = scene.pose_reason(*pose)
                if not reason and previous is not None:
                    reason = scene.arc_interval_reason(previous, pose, piece['center'])
                if reason:
                    raise TrajectoryError('UNSAFE', 'Trajectory第%d圆弧扫掠：%s' % (index+1, reason))
                previous = pose


def validate_trajectory(samples, primitives, scene, *, spacing_mm=SAMPLE_SPACING_MM,
                        interrupted=lambda: False):
    """按给定场景复核完整Trajectory；返回{ok,code,reason}，可用于障碍快照更新。"""
    try:
        spacing = _spacing(spacing_mm)
        _validate(samples, _normalize(primitives), scene, spacing, interrupted)
        return dict(ok=True, code='SAFE', reason='')
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        return dict(ok=False, code=getattr(exc, 'code', 'INVALID_INPUT'), reason=str(exc))


def _spacing(value):
    spacing = finite_number(value, '采样间隔')
    if not EPS < spacing <= SAMPLE_SPACING_MM:
        raise TrajectoryError('INVALID_INPUT', 'Trajectory采样间隔须在0..20mm内')
    return spacing


def generate_trajectory(primitives, scene, *, spacing_mm=SAMPLE_SPACING_MM,
                        interrupted=lambda: False, max_arc_angle_deg=None):
    """Line/Arc→连续Trajectory；显式车头模式另带切线和运动方式，整车复检后发布。"""
    result = dict(trajectory=[], trajectory_status='NOT_RUN', trajectory_safe=False,
                  trajectory_continuous=False, trajectory_reason='', trajectory_length_mm=0.0,
                  trajectory_spacing_mm=SAMPLE_SPACING_MM, trajectory_frame_id='FIELD_MM',
                  trajectory_yaw_convention=YAW_CONVENTION)
    try:
        if interrupted():
            raise TrajectoryError('CANCELLED', 'Trajectory生成已取消或超过时限')
        spacing = _spacing(spacing_mm)
        if max_arc_angle_deg is not None:
            max_arc_angle_deg=finite_number(max_arc_angle_deg, '圆弧角步长')
            if not EPS < max_arc_angle_deg <= 5:
                raise TrajectoryError('INVALID_INPUT', '圆弧角步长须在0..5°内')
        result['trajectory_spacing_mm'] = spacing
        primitives = list(primitives)
        pieces = _normalize(primitives)
        if not pieces:
            raise TrajectoryError('EMPTY', '没有可采样的LineSegment/ArcSegment，切线航向未定义')
        ends = _ends(pieces)
        samples, angles, tangents = [], [], []
        for station in _stations(pieces, spacing, interrupted, max_arc_angle_deg):
            if interrupted():
                raise TrajectoryError('CANCELLED', 'Trajectory生成已取消或超过时限')
            piece, distance = _piece_at(pieces, ends, station)
            lx, ly, yaw = _pose_at(piece, distance)
            fx, fy = layout_to_field(lx, ly)
            samples.append(dict(x_mm=fx, y_mm=fy, field_yaw_deg=0.0, s_mm=station,
                                segment_type=piece['kind']))
            tangent = _tangent_at(piece, distance)
            tangents.append(-90-tangent)
            if piece['explicit_heading']:
                samples[-1].update(tangent_yaw_deg=0.0, motion_mode=_motion_mode(-90-yaw, -90-tangent))
            angles.append(-90-yaw)
        for sample, yaw in zip(samples, unwrap_degrees(angles)):
            sample['field_yaw_deg'] = yaw
        for sample, tangent in zip(samples, unwrap_degrees(tangents)):
            if 'tangent_yaw_deg' in sample:
                sample['tangent_yaw_deg'] = tangent
        if any(p['explicit_heading'] for p in pieces):
            result['trajectory_yaw_convention'] = BODY_YAW_CONVENTION
        _validate(samples, pieces, scene, spacing, interrupted)
        from segment_route import build_segment_program
        program = build_segment_program(primitives)
        result.update(trajectory=samples, trajectory_status='READY', trajectory_safe=True,
                      trajectory_continuous=True, trajectory_length_mm=ends[-1], segment_program=program)
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        result.update(trajectory_status=getattr(exc, 'code', 'INVALID_INPUT'), trajectory_reason=str(exc))
    return result


def export_trajectory_json(path, result, *, metadata=None):
    """只导出完整且通过检查的Trajectory；写文件，不发任何运动命令。"""
    if (result.get('trajectory_status') != 'READY' or not result.get('trajectory_safe') or
            not result.get('trajectory_continuous') or not result.get('trajectory')):
        raise ValueError('没有连续且通过完整整车检查的Trajectory，不能导出：'+result.get('trajectory_reason', ''))
    data = dict(schema_version=2 if 'tangent_yaw_deg' in result['trajectory'][0] else 1, frame_id=result['trajectory_frame_id'],
                yaw_convention=result['trajectory_yaw_convention'],
                sample_spacing_mm=result['trajectory_spacing_mm'],
                length_mm=result['trajectory_length_mm'], model_safe=True, hardware_ready=False,
                trajectory=result['trajectory'])
    if metadata is not None:
        data['metadata'] = metadata
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
