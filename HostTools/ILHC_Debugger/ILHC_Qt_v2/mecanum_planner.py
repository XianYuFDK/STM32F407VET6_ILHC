"""麦轮路径：真实车头与行驶方向独立，安全斜线捷径和固定车头圆弧。仅PC。"""
import math
from core import field_to_layout

from arc_smoothing import ARC_RADII_MM
from navigation_planner import EPS, DEFAULT_COST_FORWARD, DEFAULT_COST_BACKWARD, DEFAULT_COST_LATERAL
from trajectory import generate_trajectory


class FixedHeadingScene:
    """车道搜索仍检查完整真实矩形，只将行进方向与指定车头解耦。"""
    def __init__(self, scene, yaw):
        self.scene, self.yaw = scene, yaw

    def __getattr__(self, name):
        return getattr(self.scene, name)

    def pose_reason(self, x, y, _yaw):
        return self.scene.pose_reason(x, y, self.yaw)

    def pose_safe(self, x, y, _yaw):
        return self.scene.pose_safe(x, y, self.yaw)

    def translation_reason(self, a, b, _yaw):
        return self.scene.translation_reason(a, b, self.yaw)

    def arc_interval_reason(self, a, b, center):
        return self.scene.arc_interval_reason((*a[:2], self.yaw), (*b[:2], self.yaw), center)


def shallow_diagonal(a, b):
    """长车道中小于12°的斜线容易产生不必要的折线；不是斜向出入库。"""
    major, minor = sorted((abs(b[0]-a[0]), abs(b[1]-a[1])), reverse=True)
    return major >= 500 and EPS < minor < major*math.tan(math.radians(12))


def movement_factor(delta_deg):
    """沿用导航的方向权重；仅为规划代价，不伪装成实车速度。"""
    longitudinal = math.cos(math.radians(delta_deg))
    lateral = math.sin(math.radians(delta_deg))
    factor = DEFAULT_COST_FORWARD if longitudinal >= 0 else DEFAULT_COST_BACKWARD
    return math.hypot(longitudinal*factor, lateral*DEFAULT_COST_LATERAL)


def edge_cost(a, b, yaw):
    tangent = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
    return math.dist(a, b)*movement_factor(tangent-yaw)


def motion_metrics(result):
    """按弧长积分真实车头与切线关系，区分倒退和连续长横移。"""
    forward = reverse = lateral = equivalent = longest = run = 0.0
    samples = result['trajectory']
    for a, b in zip(samples, samples[1:]):
        distance = b['s_mm']-a['s_mm']
        delta = ((a.get('tangent_yaw_deg', a['field_yaw_deg'])-a['field_yaw_deg'])+
                 (b.get('tangent_yaw_deg', b['field_yaw_deg'])-b['field_yaw_deg']))/2
        c, s = math.cos(math.radians(delta)), abs(math.sin(math.radians(delta)))
        if c >= 0:
            forward += distance*c
        else:
            reverse -= distance*c
        lateral += distance*s
        equivalent += distance*movement_factor(delta)
        run = run+distance if abs(c) < math.sin(math.radians(15)) else 0.0
        longest = max(longest, run)
    return dict(forward_mm=forward, reverse_mm=reverse, lateral_mm=lateral,
                longest_strafe_mm=longest, equivalent_cost_mm=equivalent)


def shortcuts(points, scene, yaw, cancelled):
    """最远可见点贪心捷径；整车扫掠穿过禁区/设备/圆柱的斜线不可用。"""
    result, i = [points[0]], 0
    while i < len(points)-1:
        if cancelled():
            raise ValueError('比赛流程规划已取消')
        j = len(points)-1
        while j > i+1 and (shallow_diagonal(points[i], points[j]) or
                          scene.translation_reason(points[i], points[j], yaw)):
            j -= 1
        result.append(points[j]); i = j
    return result


def straight_lane_route(points, scene, yaw, cancelled):
    """浅斜边改为短接近段+直车道；整车通不过时保留原候选。"""
    result = [points[0]]
    for a, b in zip(points, points[1:]):
        if cancelled():
            raise ValueError('比赛流程规划已取消')
        choices = []
        if shallow_diagonal(a, b):
            choices.extend(([a, (a[0], b[1]), b], [a, (b[0], a[1]), b]))
            major = 0 if abs(b[0]-a[0]) > abs(b[1]-a[1]) else 1
            minor = 1-major
            span = min(abs(b[major]-a[major])/3, max(150, 2*abs(b[minor]-a[minor])))
            sign = 1 if b[major] > a[major] else -1
            entry, exit_ = list(a), list(b)
            entry[minor] = exit_[minor] = (a[minor]+b[minor])/2
            entry[major] += sign*span
            exit_[major] -= sign*span
            choices.append([a, tuple(entry), tuple(exit_), b])
        valid = [r for r in choices if all(math.dist(p, q) > EPS and
                 scene.translation_reason(p, q, yaw) is None for p, q in zip(r, r[1:]))]
        if valid:
            selected = min(valid, key=lambda r: sum(math.dist(p, q) for p, q in zip(r, r[1:])))
            result.extend(selected[1:])
        else:
            result.append(b)
    return result


def fixed_primitives(points, yaw, scene, cancelled):
    """任意夹角安全圆弧倒角，保持车头；每角尝试120..60mm，不强行连接。"""
    vectors, lengths = [], []
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if length <= EPS:
            raise ValueError('麦轮路径含重合点')
        lengths.append(length)
        vectors.append(((b[0]-a[0])/length, (b[1]-a[1])/length))
    trims_in, trims_out = [0.0]*len(lengths), [0.0]*len(lengths)
    arcs, attempts = {}, []
    for i, p in enumerate(points[1:-1]):
        u, v = vectors[i:i+2]
        turn = math.atan2(u[0]*v[1]-u[1]*v[0], u[0]*v[0]+u[1]*v[1])
        if abs(turn) <= 1e-8:
            continue
        if abs(turn) >= math.pi-1e-8:
            raise ValueError('fallback：不能连续连接180度折返')
        sign = 1 if turn > 0 else -1
        log = []
        for radius in ARC_RADII_MM:
            if cancelled():
                raise ValueError('比赛流程规划已取消')
            trim = radius*math.tan(abs(turn)/2)
            if trim > lengths[i]-trims_in[i]+EPS or trim > lengths[i+1]+EPS:
                log.append(dict(radius_mm=radius, code='SHORT_SEGMENT'))
                continue
            entry = (p[0]-trim*u[0], p[1]-trim*u[1])
            exit_ = (p[0]+trim*v[0], p[1]+trim*v[1])
            center = (entry[0]-sign*radius*u[1], entry[1]+sign*radius*u[0])
            angle = math.degrees(math.atan2(entry[1]-center[1], entry[0]-center[0]))
            piece = dict(kind='ARC', entry=entry, exit=exit_, center=center, radius_mm=radius,
                         start_angle_deg=angle, sweep_deg=math.degrees(turn),
                         direction='CCW' if sign > 0 else 'CW', heading_mode='FIXED', body_yaw_deg=yaw)
            # 单弧整车复检包含<=20mm、<=3°及采样之间的连续包络。
            checked = generate_trajectory([piece], scene, interrupted=cancelled)
            log.append(dict(radius_mm=radius, code=checked['trajectory_status'], reason=checked['trajectory_reason']))
            if checked['trajectory_safe']:
                trims_out[i] = trims_in[i+1] = trim
                arcs[i] = piece
                break
        else:
            raise ValueError('fallback：麦轮圆弧所有半径失败：'+str(log))
        attempts.append(dict(corner_index=i, attempts=log))
    primitives = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        u = vectors[i]
        start = tuple(a[k]+trims_in[i]*u[k] for k in range(2))
        end = tuple(b[k]-trims_out[i]*u[k] for k in range(2))
        if math.dist(start, end) > EPS:
            primitives.append(dict(kind='LINE', start=start, end=end, heading_mode='FIXED', body_yaw_deg=yaw))
        if i in arcs:
            primitives.append(arcs[i])
    return primitives, attempts


def fixed_route(points, yaw, scene, cancelled, preflight, search):
    failures, candidates = [], []
    shortened = shortcuts(points, scene, yaw, cancelled)
    straight = shortcuts(straight_lane_route(points, scene, yaw, cancelled), scene, yaw, cancelled)
    for route in (straight, shortened, points):
        if route in candidates:
            continue
        candidates.append(route)
        try:
            pieces, attempts = fixed_primitives(route, yaw, scene, cancelled)
            result = generate_trajectory(pieces, scene, interrupted=cancelled)
            if not result['trajectory_safe']:
                raise ValueError(result['trajectory_reason'])
            combined = dict(result, points=points, driving_points=route, smoothed_primitives=pieces,
                            smoothed_points=[field_to_layout(p['x_mm'], p['y_mm'])
                                             for p in result['trajectory']],
                            arcs=[p for p in pieces if p['kind'] == 'ARC'], arc_attempts=attempts,
                            arc_fallbacks=[], smoothing_status='COMPLETE', smoothing_model_safe=True,
                            smoothed_length=result['trajectory_length_mm'], search=search,
                            planner='MECANUM_FIXED_HEADING', body_yaw_deg=yaw,
                            shortcut_fallbacks=list(failures))
            why = preflight(combined, scene, cancelled)
            if why:
                raise ValueError('实际连续跟踪预演：'+why)
            return combined
        except ValueError as exc:
            if cancelled():
                raise ValueError('比赛流程规划已取消') from exc
            failures.append(str(exc))
    raise ValueError('; '.join(failures))
