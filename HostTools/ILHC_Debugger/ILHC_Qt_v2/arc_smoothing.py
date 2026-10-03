"""90°圆弧后处理：保留原A*台账，按整车姿态和连续包络安全检查后才替换拐角。"""
from dataclasses import dataclass
import math

from navigation_planner import EPS, A_FORWARD, A_BACKWARD, A_STRAFE, A_TURN_LEFT, A_TURN_RIGHT

ARC_RADII_MM = (120.0, 110.0, 100.0, 90.0, 80.0, 70.0, 60.0)
ARC_MAX_SAMPLE_MM = 20.0
ARC_MAX_SAMPLE_DEG = 3.0


@dataclass(frozen=True)
class ArcSegment:
    corner_index: int
    incoming_segment: int
    radius_mm: float
    entry: tuple[float, float]
    center: tuple[float, float]
    exit: tuple[float, float]
    direction: str
    start_angle_deg: float
    sweep_deg: float
    heading_in_deg: float
    heading_out_deg: float
    action: str
    sample_poses: tuple[tuple[float, float, float], ...]

    @property
    def length_mm(self):
        return self.radius_mm * math.pi / 2.0

    def as_dict(self):
        return dict(corner_index=self.corner_index, incoming_segment=self.incoming_segment,
                    radius_mm=self.radius_mm, entry=self.entry, center=self.center, exit=self.exit,
                    direction=self.direction, start_angle_deg=self.start_angle_deg,
                    sweep_deg=self.sweep_deg, heading_in_deg=self.heading_in_deg,
                    heading_out_deg=self.heading_out_deg, action=self.action,
                    length_mm=self.length_mm, sample_poses=self.sample_poses)


def _vector(segment):
    return (segment.direction, 0) if segment.axis == 'x' else (0, segment.direction)


def _angle_delta(a, b):
    return ((b-a+180.0) % 360.0)-180.0


def _candidate(incoming, outgoing, corner, radius):
    u, v = _vector(incoming), _vector(outgoing)
    cross = u[0]*v[1]-u[1]*v[0]
    p = corner.point
    entry = (p[0]-radius*u[0], p[1]-radius*u[1])
    exit_ = (p[0]+radius*v[0], p[1]+radius*v[1])
    center = (entry[0]+radius*v[0], entry[1]+radius*v[1])
    start_angle = math.degrees(math.atan2(entry[1]-center[1], entry[0]-center[0]))
    count = max(math.ceil(radius*math.pi/2/ARC_MAX_SAMPLE_MM),
                math.ceil(90/ARC_MAX_SAMPLE_DEG))
    samples = []
    for i in range(count+1):
        offset = cross*90.0*i/count
        ang = math.radians(start_angle+offset)
        point = (center[0]+radius*math.cos(ang), center[1]+radius*math.sin(ang))
        if i == 0:
            point = entry
        elif i == count:
            point = exit_
        yaw = (incoming.heading_deg+offset) % 360.0
        samples.append((point[0], point[1], yaw))
    return ArcSegment(corner.index, incoming.index, radius, entry, center, exit_,
                      'CCW' if cross > 0 else 'CW', start_angle, cross*90.0,
                      incoming.heading_deg, outgoing.heading_deg, incoming.action, tuple(samples))


def _arc_reason(arc, scene, interrupted):
    previous = None
    for pose in arc.sample_poses:
        if interrupted():
            return 'CANCELLED', '圆弧检查已取消或超过时限'
        # 每个采样点均用真实矩形+pad检查，不能只检查车心或局部顶点。
        if not scene.pose_safe(*pose):
            return 'POSE_COLLISION', scene.pose_reason(*pose)
        if previous is not None:
            reason = scene.arc_interval_reason(previous, pose, arc.center)
            if reason:
                return 'SWEEP_COLLISION', reason
        previous = pose
    return 'SAFE', ''


def _primitives(segments, steps, arcs, trim_start, trim_end):
    result, move_i = [], 0
    previous_heading = steps[0]['heading_deg']
    for row in steps:
        if row['kind'] == 'MOVE':
            segment = segments[move_i]
            u = _vector(segment)
            a = tuple(segment.start[k]+trim_start[move_i]*u[k] for k in range(2))
            b = tuple(segment.end[k]-trim_end[move_i]*u[k] for k in range(2))
            if math.dist(a, b) > EPS:
                result.append(dict(kind='LINE', start=a, end=b, heading_deg=segment.heading_deg,
                                   action=segment.action, length_mm=math.dist(a, b)))
            move_i += 1
        elif row['kind'] == 'TURN':
            arc = arcs.get(move_i-1)
            u = _vector(segments[move_i-1]) if arc is not None else (0, 0)
            turn_point = (arc.entry[0]+arc.radius_mm*u[0], arc.entry[1]+arc.radius_mm*u[1]) \
                         if arc is not None else None
            if (arc is not None and math.dist(turn_point, (row['x'], row['y'])) <= EPS and
                    abs(_angle_delta(arc.heading_out_deg, row['heading_deg'])) <= EPS):
                result.append(dict(kind='ARC', **arc.as_dict()))
            else:
                result.append(dict(kind='TURN', point=(row['x'], row['y']), action=row['action'],
                                   heading_in_deg=previous_heading, heading_out_deg=row['heading_deg'],
                                   length_mm=0.0))
        previous_heading = row['heading_deg']
    return result


def _check_and_points(primitives, steps, scene, interrupted):
    start = (steps[0]['x'], steps[0]['y'], steps[0]['heading_deg'])
    if not scene.pose_safe(*start):
        return [], scene.pose_reason(*start)
    points, previous = [(start[0], start[1])], start
    for piece in primitives:
        if interrupted():
            return [], '圆弧后处理已取消或超过时限'
        if piece['kind'] == 'LINE':
            a, b, yaw = piece['start'], piece['end'], piece['heading_deg']
            if math.dist(previous[:2], a) > EPS or abs(_angle_delta(previous[2], yaw)) > EPS:
                return [], '平滑轨迹直线入口不连续'
            reason = scene.translation_reason(a, b, yaw)
            if reason:
                return [], reason
            previous = (*b, yaw)
            points.append(b)
        elif piece['kind'] == 'ARC':
            first = piece['sample_poses'][0]
            if math.dist(previous[:2], first[:2]) > EPS or abs(_angle_delta(previous[2], first[2])) > EPS:
                return [], '平滑轨迹圆弧入口不连续'
            points.extend((x, y) for x, y, _yaw in piece['sample_poses'][1:])
            previous = piece['sample_poses'][-1]
        else:
            if math.dist(previous[:2], piece['point']) > EPS:
                return [], 'fallback原地转向位置不连续'
            reason = scene.turn_reason(piece['point'], previous[2], piece['heading_out_deg'])
            if reason:
                return [], reason
            previous = (*piece['point'], piece['heading_out_deg'])
    expected = steps[-1]['to_x'], steps[-1]['to_y'], steps[-1]['heading_deg']
    if math.dist(previous[:2], expected[:2]) > EPS or abs(_angle_delta(previous[2], expected[2])) > EPS:
        return [], '平滑轨迹终点或终点航向发生改变'
    return points, None


def smooth_90_corners(segments, corners, steps, scene, *, interrupted=lambda: False):
    """每个真实90°转弯依次试120..60mm；失败保留原台账，返回逐半径原因。"""
    result = dict(arcs=[], arc_fallbacks=[], arc_attempts=[], smoothed_primitives=[], smoothed_points=[],
                  smoothed_length=0.0, smoothing_status='NO_TURNABLE_CORNER',
                  smoothing_model_safe=False)
    if not steps:
        result.update(smoothing_status='INVALID_INPUT',
                      arc_fallbacks=[dict(corner_index=-1, code='NO_STEPS', reason='缺少动作台账', attempts=[])])
        return result
    if scene.footprint is None:
        result.update(smoothing_status='NO_FOOTPRINT', arc_fallbacks=[dict(
            corner_index=c.index, code='NO_FOOTPRINT', reason='缺少真实矩形车体，不能平滑', attempts=[])
            for c in corners])
        return result
    trim_start, trim_end = [0.0]*len(segments), [0.0]*len(segments)
    accepted, eligible = {}, 0
    pairs = [i for i in range(len(segments)-1) if segments[i].axis != segments[i+1].axis]
    if len(pairs) != len(corners):
        result['smoothing_status'] = 'INVALID_INPUT'
        result['arc_fallbacks'] = [dict(corner_index=-1, code='CORNER_MISMATCH',
                                      reason='角点与动作段不匹配', attempts=[])]
        return result
    for i, corner in zip(pairs, corners):
        if interrupted():
            result['smoothing_status'] = 'CANCELLED'
            return result
        before, after = segments[i], segments[i+1]
        u, v = _vector(before), _vector(after)
        cross = u[0]*v[1]-u[1]*v[0]
        reason = None
        if corner.steps != 1 or corner.action not in (A_TURN_LEFT, A_TURN_RIGHT):
            reason = '不是单次90°车体转向（可能只换轮模式或原地掉头）'
        elif before.action != after.action or before.action not in (A_FORWARD, A_BACKWARD, A_STRAFE):
            reason = '转弯前后行进模式不同或缺少平移动作；保留原动作'
        elif before.heading_deg is None or after.heading_deg is None:
            reason = '缺少真实入口/出口航向'
        elif abs(_angle_delta(before.heading_deg, after.heading_deg)-cross*90) > EPS:
            reason = '车体转向与几何圆弧方向不一致'
        elif math.dist(before.end, corner.point) > EPS or math.dist(after.start, corner.point) > EPS:
            reason = '角点位置与两侧动作段不匹配'
        if reason:
            result['arc_fallbacks'].append(dict(corner_index=corner.index, code='NOT_TURNABLE',
                                               reason=reason, attempts=[]))
            continue
        eligible += 1
        attempts = []
        for radius in ARC_RADII_MM:
            arc = _candidate(before, after, corner, radius)
            if (radius > before.length_mm-trim_start[i]+EPS or
                    radius > after.length_mm-trim_end[i+1]+EPS):
                code, why = 'SHORT_SEGMENT', '切点超出可用直线长度，或与相邻圆弧重叠'
            else:
                code, why = _arc_reason(arc, scene, interrupted)
            attempts.append(dict(radius_mm=radius, code=code, reason=why,
                                 entry=arc.entry, center=arc.center, exit=arc.exit, direction=arc.direction))
            if code == 'CANCELLED':
                result['smoothing_status'] = 'CANCELLED'
                return result
            if code == 'SAFE':
                accepted[i] = arc
                trim_end[i], trim_start[i+1] = radius, radius
                break
        else:
            result['arc_fallbacks'].append(dict(corner_index=corner.index, code='ALL_RADII_FAILED',
                reason='120、110、100、90、80、70、60mm均失败；保留原直角与原地转向', attempts=attempts))
        result['arc_attempts'].append(dict(corner_index=corner.index, attempts=attempts))
    pieces = _primitives(segments, steps, accepted, trim_start, trim_end)
    points, reason = _check_and_points(pieces, steps, scene, interrupted)
    if reason:
        code = 'CANCELLED' if interrupted() else 'UNSAFE_TRAJECTORY'
        result['smoothing_status'] = code
        result['arc_fallbacks'].append(dict(corner_index=-1, code=code, reason=reason, attempts=[]))
        return result
    result.update(arcs=list(accepted.values()), smoothed_primitives=pieces, smoothed_points=points,
                  smoothed_length=sum(p['length_mm'] for p in pieces), smoothing_model_safe=True,
                  smoothing_status=('COMPLETE' if len(accepted) == eligible else 'PARTIAL')
                  if accepted else 'FALLBACK' if eligible else 'NO_TURNABLE_CORNER')
    return result
