# -*- coding: utf-8 -*-
"""Pure, bounded A* + continuous collision checks (no Qt, serial, or motor output).

Coordinates: map LAYOUT millimetres (same as FieldView), not OPS/body coordinates.
Translations use the current heading; turns are checked with conservative sweeps.
Allowed polygons are an explicit whitelist, not obstacle complements.
The built-in/default scene is an UNVERIFIED preview, never a real-car approval.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
import time
from typing import Callable

from shapely.affinity import translate
from shapely.geometry import Point, LineString, Polygon, MultiPoint, box
from shapely.ops import unary_union
from shapely.prepared import prep

EPS = 1e-7
MAX_CELLS = 100_000
MAX_OBSTACLES = 512
MAX_POLYGON_VERTICES = 4096


def finite_number(value, name="value") -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数值")
    return value


def point2(value, name="坐标") -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{name}必须有两个分量")
    return finite_number(value[0], name), finite_number(value[1], name)


def length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _line(a, b):
    return Point(a) if a == b else LineString((a, b))


def _rects(rects):
    if len(rects) > MAX_OBSTACLES:
        raise ValueError("矩形障碍数量超限")
    result = []
    for r in rects:
        if len(r) != 5:
            raise ValueError("矩形须为[x0,y0,x1,y1,name]")
        x0, y0, x1, y1 = [finite_number(v, "矩形") for v in r[:4]]
        if x0 >= x1 or y0 >= y1:
            raise ValueError("矩形边界顺序非法")
        result.append((x0, y0, x1, y1, str(r[4])))
    return tuple(result)


def _circles(circles):
    if len(circles) > MAX_OBSTACLES:
        raise ValueError("圆障碍数量超限")
    result = []
    for c in circles:
        if len(c) != 4:
            raise ValueError("圆须为[cx,cy,r,name]")
        x, y, r = [finite_number(v, "圆障碍") for v in c[:3]]
        if r <= 0:
            raise ValueError("圆半径必须大于0")
        result.append((x, y, r, str(c[3])))
    return tuple(result)


def allowed_area(polygons, bounds):
    """Union BEFORE containment: a robot may straddle adjoining lane rectangles.

    Each entry: [[x,y],...] or {"outer":[...], "holes":[[[x,y],...],...]}
    No make_valid/buffer(0): invalid user geometry is rejected, never silently fixed.
    """
    if polygons is None:
        return box(*bounds)
    if not polygons or len(polygons) > MAX_OBSTACLES:
        raise ValueError("合法行驶区域不能为空或超过数量限制")
    parts = []
    for spec in polygons:
        outer = spec.get("outer") if isinstance(spec, dict) else spec
        holes = spec.get("holes", []) if isinstance(spec, dict) else []
        if outer is None or len(outer) < 3:
            raise ValueError("行驶区域多边形至少需要三个点")
        if len(outer) + sum(len(h) for h in holes) > MAX_POLYGON_VERTICES:
            raise ValueError("行驶区域顶点数超限")
        if any(len(h) < 3 for h in holes):
            raise ValueError("多边形孔至少需要三个点")
        poly = Polygon([point2(p) for p in outer],
                       [[point2(p) for p in h] for h in holes])
        if not poly.is_valid or poly.is_empty or poly.area <= 0:
            raise ValueError("行驶区域多边形自交、为空或退化")
        if not box(*bounds).covers(poly):
            raise ValueError("行驶区域超出场地边界")
        parts.append(poly)
    area = unary_union(parts)
    if not area.is_valid or area.is_empty:
        raise ValueError("行驶区域合并失败")
    return area


@dataclass(frozen=True)
class Footprint:
    """Length along nose; width across nose; yaw from LAYOUT +x, CCW degrees.

    This yaw is obtained by transforming the nose VECTOR, not by adding an OPS
    angle to a display angle. margin is additional uncertainty/clearance budget.
    """
    length_mm: float
    width_mm: float
    yaw_deg: float

    def vertices(self, margin=0.0):
        a = (self.length_mm * 0.5 + margin)
        b = (self.width_mm * 0.5 + margin)
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        return tuple((c*x - s*y, s*x + c*y)
                     for x, y in ((-a, -b), (a, -b), (a, b), (-a, b)))


def as_footprint(value):
    if value is None:
        return None
    if isinstance(value, Footprint):
        vals = (value.length_mm, value.width_mm, value.yaw_deg)
    elif isinstance(value, dict):
        vals = (value["length_mm"], value["width_mm"], value["yaw_deg"])
    else:
        vals = value
    if len(vals) != 3:
        raise ValueError("车体参数须为(length_mm,width_mm,layout_yaw_deg)")
    a, b, yaw = [finite_number(v, "车体参数") for v in vals]
    if not (0 < a <= 5000 and 0 < b <= 5000):
        raise ValueError("车体长度/宽度须在0..5000mm内")
    return Footprint(a, b, yaw % 360.0)


def rotation_sweep(footprint, pad, from_deg, to_deg):
    """绝对角度转向扫掠：相邻采样姿态的凸包加圆弧弦高，覆盖采样间隙。"""
    fp = as_footprint(footprint)
    if fp is None:
        return None
    a0 = finite_number(from_deg, "起始航向")
    delta = (finite_number(to_deg, "目标航向") - a0) % 360.0
    if delta > 180.0:
        delta -= 360.0
    local = Footprint(fp.length_mm, fp.width_mm, 0.0).vertices(pad)
    n = max(1, math.ceil(abs(delta) / 15.0))
    shapes = []
    for k in range(n + 1):
        ang = math.radians(a0 + delta * k / n)
        c, s = math.cos(ang), math.sin(ang)
        shapes.append(Polygon([(c*x-s*y, s*x+c*y) for x, y in local]))
    if abs(delta) <= EPS:
        return shapes[0]
    radius = max(math.hypot(x, y) for x, y in local)
    error = radius * (1.0 - math.cos(math.radians(delta / n) / 2.0)) + 1.0
    hulls = [unary_union((a, b)).convex_hull for a, b in zip(shapes, shapes[1:])]
    # buffer 的圆弧以弦近似，稍放大半径以保证外扩量不小于误差界。
    return unary_union(hulls).buffer(error / math.cos(math.pi / 64.0), quad_segs=16)


class CollisionScene:
    """Exact polygon sweeps for fixed-orientation rectangular robots.

    Rectangular obstacles: exact convex Minkowski sums in centre space.
    Circular obstacles: distance to (centre minus robot polygon) <= circle radius;
    this uses true radius, NOT a polygonal approximation to a circle.
    Allowed regions: entire swept polygon must be covered (works for holes and
    concave lanes); checking corners alone would NOT be sufficient.
    pad-only compatibility mode uses a conservative square footprint, not a car.
    """
    def __init__(self, rects, circles, bounds, pad=0.0, footprint=None,
                 drivable_polygons=None, *, sim_rects=None, sim_circles=None,
                 dynamic_rects=None, dynamic_circles=None):
        if len(bounds) != 4:
            raise ValueError("bounds须含四个数")
        self.bounds = tuple(finite_number(v, "边界") for v in bounds)
        x0, y0, x1, y1 = self.bounds
        if x0 >= x1 or y0 >= y1:
            raise ValueError("场地边界顺序非法")
        self.pad = finite_number(pad, "安全裕量/膨胀")
        if not 0 <= self.pad <= 5000:
            raise ValueError("安全裕量/膨胀须在0..5000mm内")
        self.rects = _rects(list(rects) + list(sim_rects or ()) + list(dynamic_rects or ()))
        self.circles = _circles(list(circles) + list(sim_circles or ()) + list(dynamic_circles or ()))
        self.footprint = as_footprint(footprint)
        if self.footprint is None:
            p = self.pad
            self.offsets = ((-p, -p), (p, -p), (p, p), (-p, p)) if p else ((0.0, 0.0),)
            self.body_offsets = self.offsets
        else:
            self.offsets = self.footprint.vertices(self.pad)
            self.body_offsets = self.footprint.vertices(0.0)
        self.ex = max(abs(x) for x, _ in self.offsets)
        self.ey = max(abs(y) for _, y in self.offsets)
        self.allowed = allowed_area(drivable_polygons, self.bounds)
        self.allowed_prepared = prep(self.allowed)
        self.whole_rectangle = self.allowed.equals(box(*self.bounds))
        self.rect_shapes = tuple((box(*r[:4]), r[4]) for r in self.rects)
        # Reflection of a convex robot footprint; hull is exact, including for a point.
        self.forbidden_rects = []
        for rx0, ry0, rx1, ry1, name in self.rects:
            corners = ((rx0, ry0), (rx1, ry0), (rx1, ry1), (rx0, ry1))
            hull = MultiPoint([(x-dx, y-dy) for x, y in corners
                               for dx, dy in self.offsets]).convex_hull
            self.forbidden_rects.append((prep(hull), name))
        self.forbidden_circles = tuple(
            (MultiPoint([(x-dx, y-dy) for dx, dy in self.offsets]).convex_hull, r, name)
            for x, y, r, name in self.circles)

    def sweep(self, a, b, body_only=False):
        offsets = self.body_offsets if body_only else self.offsets
        return MultiPoint([(x+dx, y+dy) for x, y in (a, b)
                           for dx, dy in offsets]).convex_hull

    def segment_reason(self, a, b):
        a, b = point2(a), point2(b)
        x0, y0, x1, y1 = self.bounds
        if (min(a[0], b[0]) - self.ex < x0 - EPS or
            max(a[0], b[0]) + self.ex > x1 + EPS or
            min(a[1], b[1]) - self.ey < y0 - EPS or
            max(a[1], b[1]) + self.ey > y1 + EPS):
            return "车体/安全裕量超出场地边界"
        line = _line(a, b)
        for obstacle, name in self.forbidden_rects:
            if obstacle.intersects(line):
                return name
        for poly, radius, name in self.forbidden_circles:
            if line.distance(poly) <= radius + EPS:
                return name
        if not self.whole_rectangle and not self.allowed_prepared.covers(self.sweep(a, b)):
            return "车体/安全裕量越出合法行驶区域"
        return None

    def validate(self, points):
        if not points:
            return "路径为空"
        try:
            ps = [point2(p) for p in points]
            if len(ps) == 1:
                return self.segment_reason(ps[0], ps[0])
            for i, (a, b) in enumerate(zip(ps, ps[1:])):
                reason = self.segment_reason(a, b)
                if reason:
                    return f"第{i+1}段：{reason}"
        except (TypeError, ValueError, IndexError) as exc:
            return f"非法路径：{exc}"
        return None

    def turn_reason(self, point, from_deg, to_deg, sweep=None):
        """按实际车心复查原地转向；可复用已缓存的局部扫掠。"""
        if self.footprint is None:
            return None
        if sweep is None:
            sweep = rotation_sweep(self.footprint, self.pad, from_deg, to_deg)
        moved = translate(sweep, xoff=point[0], yoff=point[1])
        return self.geometry_reason(moved, "转向车体/安全裕量越出合法行驶区域")

    def geometry_reason(self, swept, boundary_reason="车体/安全裕量越出合法行驶区域"):
        """完整扫掠白名单包含检查，固定/模拟/动态障碍共用同一场景快照。"""
        if not self.allowed_prepared.covers(swept):
            return boundary_reason
        for obstacle, name in self.rect_shapes:
            if swept.intersects(obstacle):
                return name
        for x, y, radius, name in self.circles:
            if swept.distance(Point(x, y)) <= radius + EPS:
                return name
        return None

    def pose_polygon(self, x, y, yaw):
        """真实矩形车体加裕量，航向为布局绝对角；缺少真实车体时拒绝。"""
        if self.footprint is None:
            raise ValueError("pose_safe需要真实矩形车体参数，不能用车心/膨胀兼容模型")
        x, y, yaw = (finite_number(x, "x"), finite_number(y, "y"), finite_number(yaw, "yaw"))
        fp = Footprint(self.footprint.length_mm, self.footprint.width_mm, yaw % 360.0)
        polygon = Polygon([(x+dx, y+dy) for dx, dy in fp.vertices(self.pad)])
        if polygon.is_empty or not polygon.is_valid or polygon.area <= 0:
            raise ValueError("车体姿态矩形退化或非法")
        return polygon

    def pose_reason(self, x, y, yaw):
        try:
            return self.geometry_reason(self.pose_polygon(x, y, yaw))
        except (ValueError, TypeError, OverflowError) as exc:
            return str(exc)

    def pose_safe(self, x, y, yaw):
        """只有完整车体及安全裕量在合法区内、且不碰任何障碍时才返回True。"""
        return self.pose_reason(x, y, yaw) is None

    def translation_reason(self, a, b, yaw):
        swept = unary_union((self.pose_polygon(*a, yaw), self.pose_polygon(*b, yaw))).convex_hull
        return self.geometry_reason(swept)

    def arc_interval_reason(self, a, b, center):
        """相邻圆弧姿态凸包+所有车体顶点的弦高界，覆盖完整采样间隙。"""
        polygon, other = self.pose_polygon(*a), self.pose_polygon(*b)
        delta = abs((b[2]-a[2]+180) % 360-180)
        # 车心绕圆心运动、车体绕车心旋转是两个独立运动；固定车头弧线仍有车心弦高。
        angle_a = math.degrees(math.atan2(a[1]-center[1], a[0]-center[0]))
        angle_b = math.degrees(math.atan2(b[1]-center[1], b[0]-center[0]))
        path_delta = abs((angle_b-angle_a+180) % 360-180)
        if abs(path_delta-delta) < 1e-5 and delta > EPS:
            radius = max(math.dist(center, v) for v in polygon.exterior.coords[:-1])
            error = radius*(1-math.cos(math.radians(delta)/2))+EPS
        else:
            orbit = max(math.dist(center, a[:2]), math.dist(center, b[:2]))
            body = math.hypot(self.footprint.length_mm/2+self.pad, self.footprint.width_mm/2+self.pad)
            error = orbit*(1-math.cos(math.radians(path_delta)/2))+body*(1-math.cos(math.radians(delta)/2))+EPS
        envelope = unary_union((polygon, other)).convex_hull.buffer(error/math.cos(math.pi/64), quad_segs=16)
        return self.geometry_reason(envelope)

    def moving_pose_reason(self, a, b):
        """实际小步运动：线性车心位移+最短角旋转的连续矩形扫掠。"""
        first, last = self.pose_polygon(*a), self.pose_polygon(*b)
        delta = abs((b[2]-a[2]+180) % 360-180)
        radius = math.hypot(self.footprint.length_mm/2+self.pad, self.footprint.width_mm/2+self.pad)
        error = radius*(1-math.cos(math.radians(delta)/2))+EPS
        sweep = unary_union((first, last)).convex_hull
        if delta > EPS:
            sweep = sweep.buffer(error/math.cos(math.pi/64), quad_segs=16)
        return self.geometry_reason(sweep)

    def body_clearance(self, points):
        """Body OUTER EDGE clearance to all obstacles + allowed-region boundary."""
        if self.footprint is None or not points:
            return None
        best = math.inf
        for a, b in zip(points, points[1:] or points):
            swept = self.sweep(a, b, body_only=True)
            best = min(best, self.sweep_clearance(swept))
        return None if not math.isfinite(best) else best

    def sweep_clearance(self, swept):
        """车体扫掠外缘净空；旋转使用保守包络，给出净空下界。"""
        if not self.allowed.covers(swept):
            return 0.0
        best = swept.distance(self.allowed.boundary)
        for obstacle, _ in self.rect_shapes:
            best = min(best, swept.distance(obstacle))
        for x, y, radius, _ in self.circles:
            best = min(best, max(0.0, swept.distance(Point(x, y)) - radius))
        return best


def centre_clearance(points, rects, circles):
    """Exact continuous distance from CENTRE LINE to obstacle surfaces (no boundary).

    Kept separate from body_clearance/boundary_clearance to prevent misleading
    'real margin' displays. Empty obstacles => +inf; empty path => None.
    """
    if not points:
        return None
    ps = [point2(p) for p in points]
    geom = Point(ps[0]) if len(ps) == 1 else LineString(ps)
    distances = [geom.distance(box(*r[:4])) for r in _rects(rects)]
    distances += [max(0.0, geom.distance(Point(c[0], c[1])) - c[2]) for c in _circles(circles)]
    return min(distances, default=math.inf)


def simplify_checked(points, segment_reason: Callable):
    """String-pull only through validated edges; also checks 1/2-point paths."""
    ps = [point2(p) for p in points]
    if not ps:
        return []
    if len(ps) == 1:
        if segment_reason(ps[0], ps[0]):
            raise ValueError("单点路径碰撞")
        return ps
    for a, b in zip(ps, ps[1:]):
        reason = segment_reason(a, b)
        if reason:
            raise ValueError(f"原始路径包含碰撞段：{reason}")
    out, i = [ps[0]], 0
    while i < len(ps) - 1:
        for j in range(len(ps)-1, i, -1):
            if segment_reason(ps[i], ps[j]) is None:
                out.append(ps[j])
                i = j
                break
        else:
            raise ValueError("路径简化没有合法后继")
    return out


def _unique(points):
    out = []
    for p in points:
        if not out or math.dist(out[-1], p) > EPS:
            out.append(p)
    return out


@dataclass(frozen=True)
class LineSegment:
    """折线里一段**轴对齐**直线（Manhattan 路径不允许斜段）。"""
    index: int
    start: tuple[float, float]
    end: tuple[float, float]
    axis: str
    direction: int
    length_mm: float
    heading_deg: object
    reverse: bool
    action: str

    def as_dict(self):
        return {"index": self.index, "start": (self.start[0], self.start[1]),
                "end": (self.end[0], self.end[1]), "axis": self.axis,
                "direction": self.direction, "length_mm": self.length_mm,
                "heading_deg": self.heading_deg, "reverse": self.reverse,
                "action": self.action}


@dataclass(frozen=True)
class Corner:
    """两个方向不同的直线段之间的 90° 角点；原地转向不产生角点。"""
    index: int
    point: tuple[float, float]
    heading_in_deg: object
    heading_out_deg: object
    turn: str
    steps: int
    action: str

    def as_dict(self):
        return {"index": self.index, "point": (self.point[0], self.point[1]),
                "heading_in_deg": self.heading_in_deg,
                "heading_out_deg": self.heading_out_deg,
                "turn": self.turn, "steps": self.steps, "action": self.action}


def path_axes(points, steps=None):
    """Manhattan 折线 → (LineSegment 列表, Corner 列表)。

    压缩规则**只有一条**：删掉与前一点重合的点（零长度段）。共线中间点由上游
    `simplify_collinear` 合并——本函数**不**删共线节点，也不做任何视线直连。
    **绝不生成任意斜直连**：某一段 dx 与 dy 都不为 0 就 raise ValueError。

    `steps` 给定时（规划结果的动作台账）按坐标配对，把当时的 heading/action 原样
    挂到线段与角点上，不重新推断语义；对不上就动作留空串，几何检查照做。

    Corner 只由几何（行进轴改变）定义。要分清两件事：
    * 几何角点**未必**有原地转向——"横移 → 后退"这种只是轮模式变了，航向没动；
    * 原地转向也**未必**产生角点——转完继续沿同一条直线走时，该顶点是被合并掉的。
    所以角点的 `action` 按**坐标与台账顺序**找两段之间的转向，找不到就是空串，
    `steps` 是那个点上实际转过的 90° 数（0 表示只换了轮模式）。角点数不能当步数用。
    """
    ps = _unique([point2(p) for p in points])
    for idx, (a, b) in enumerate(zip(ps, ps[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        if abs(dx) > EPS and abs(dy) > EPS:
            raise ValueError("第%d段既非水平也非垂直（不接受斜直连）：%s → %s"
                             % (idx + 1, a, b))
    norm_steps = []
    for row in (steps or ()):
        if not isinstance(row, dict):
            continue
        if not all(k in row for k in ("x", "y", "to_x", "to_y")):
            continue
        try:
            src = (float(row["x"]), float(row["y"]), float(row["to_x"]), float(row["to_y"]))
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in src):
            continue
        norm_steps.append((tuple(round(v, 2) for v in src),
                           row.get("heading_deg"), str(row.get("action", ""))))
    segments = []
    matched_rows = []
    cursors = {}
    for a, b in zip(ps, ps[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        axis = "x" if abs(dy) <= EPS else "y"      # 沿哪条轴走
        along = dx if axis == "x" else dy
        direction = 1 if along > EPS else -1
        key = (round(a[0], 2), round(a[1], 2), round(b[0], 2), round(b[1], 2))
        hd, act = None, ""
        matched_row = None
        for k in range(cursors.get(key, 0), len(norm_steps)):
            if (norm_steps[k][0] == key and
                    norm_steps[k][2] in (A_FORWARD, A_BACKWARD, A_STRAFE)):
                cursors[key] = k + 1
                hd, act = norm_steps[k][1], norm_steps[k][2]
                matched_row = k
                break
        matched_rows.append(matched_row)
        if axis == "x":
            axis_deg = 0.0 if direction > 0 else 180.0
        else:
            axis_deg = 90.0 if direction > 0 else 270.0
        reverse = False
        if isinstance(hd, (int, float)) and math.isfinite(float(hd)):
            hd = float(hd)
            reverse = act in (A_BACKWARD, A_STRAFE) if act else \
                      abs(((hd - axis_deg + 180.0) % 360.0) - 180.0) > 45.0
        else:
            hd = None
        segments.append(LineSegment(index=len(segments), start=(a[0], a[1]),
                                    end=(b[0], b[1]), axis=axis, direction=direction,
                                    length_mm=math.dist(a, b), heading_deg=hd,
                                    reverse=reverse, action=act))
    corners = []
    for idx, (seg, nxt) in enumerate(zip(segments, segments[1:])):
        if seg.axis == nxt.axis:
            continue                               # 同轴＝共线，上游应已合并
        d_in = (1 if seg.axis == "x" else 0) * seg.direction, \
               (1 if seg.axis == "y" else 0) * seg.direction
        d_out = (1 if nxt.axis == "x" else 0) * nxt.direction, \
                (1 if nxt.axis == "y" else 0) * nxt.direction
        cross = d_in[0] * d_out[1] - d_in[1] * d_out[0]
        turn = "LEFT" if cross > 0 else "RIGHT"     # 逆时针为正，与 +Z 一致
        if seg.heading_deg is not None and nxt.heading_deg is not None:
            delta = ((nxt.heading_deg - seg.heading_deg + 180.0) % 360.0) - 180.0
            if delta > 1.0:
                turn = "LEFT"
            elif delta < -1.0:
                turn = "RIGHT"
        action, k90 = "", 0
        first, last = matched_rows[idx], matched_rows[idx + 1]
        if first is not None and last is not None:
            for _key, _hd, act in norm_steps[first + 1:last]:
                if act in (A_TURN_LEFT, A_TURN_RIGHT, A_TURN_AROUND):
                    action = action or act
                    k90 += 2 if act == A_TURN_AROUND else 1
        corners.append(Corner(index=len(corners), point=(nxt.start[0], nxt.start[1]),
                              heading_in_deg=seg.heading_deg,
                              heading_out_deg=nxt.heading_deg,
                              turn=turn, steps=k90, action=action))
    return segments, corners


def simplify_collinear(points, segment_reason: Callable, check: bool = True):
    """只合并共线节点：删掉同一直线上重复的中间点，**不**做任意视线直连。

    Manhattan（4 邻域）规划的输出必须是直角折线：每段 dx==0 或 dy==0。
    simplify_checked 的 string-pull 会把直角拉成斜线，所以规划走这一条；
    这里只删除与前后点共线且同向的中间点，方向改变的拐点一律保留。

    check=True（默认）逐段检查原始路径与合并结果；规划在含转向的路径上会传
    check=False —— 那种情况下每段必须按"当时航向"的车体单独复查（调用方已做），
    而共线合并只把同一直线上同向的若干段并成一条，扫掠区域不变，所以跳过重复检查
    不会放过任何碰撞。
    """
    ps = [point2(p) for p in points]
    if not ps:
        return []
    if len(ps) == 1:
        if check and segment_reason(ps[0], ps[0]):
            raise ValueError("单点路径碰撞")
        return ps
    if check:
        for a, b in zip(ps, ps[1:]):
            reason = segment_reason(a, b)
            if reason:
                raise ValueError(f"原始路径包含碰撞段：{reason}")
    out = [ps[0]]
    for k in range(1, len(ps) - 1):
        ax, ay = out[-1]
        bx, by = ps[k]
        cx, cy = ps[k + 1]
        cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        forward = (bx - ax) * (cx - bx) + (by - ay) * (cy - by) > 0
        if abs(cross) > EPS or not forward:
            out.append(ps[k])          # 方向变了（含掉头）才保留这个点
    out.append(ps[-1])
    merged = _unique(out)
    if check:
        for a, b in zip(merged, merged[1:]):
            reason = segment_reason(a, b)
            if reason:
                raise ValueError(f"合并后路径碰撞：{reason}")
    return merged


def plan(start, goal, *, grid, pad, rects, circles, bounds,
         footprint=None, drivable_polygons=None, geometry_verified=False,
         cancel=None, time_limit_s=8.0,
         start_heading_deg=0.0, goal_heading_deg=None,
         allow_strafe=True, strafe_polygons=None, strafe_run_limit_mm=None,
         cost_forward=None, cost_backward=None,
         cost_lateral=None, turn_penalty_mm=None, smooth_arcs=True,
         sim_rects=None, sim_circles=None, dynamic_rects=None, dynamic_circles=None):
    """Heading-aware Manhattan A*：状态 = (格号 i, 格号 j, 航向 0..3)。

    平移仍是严格 4 邻域（上下左右），但按**当前航向**分类计价：
    FORWARD = 步长×cost_forward、BACKWARD = 步长×cost_backward、STRAFE = 步长×cost_lateral。
    航向改变只在原地做 90° 的整数倍（TURN_LEFT/RIGHT/AROUND），每个 90° 记
    turn_penalty_mm；原A*台账仍是直角曼哈顿线。smooth_arcs启用时，安全圆弧
    后处理另存arcs/smoothed_primitives/smoothed_points，失败角点明确fallback。
    allow_strafe=False 完全禁止普通道路横移；strafe_run_limit_mm=N 只禁**长距离**横移：
    一次连续横移不得超过 N mm，超了就改用原地转向+直行（0 = 完全禁横移，None = 不限）。
    口径是"单次连续"：横移被前进/后退/转向打断后重新计数，不是全程总量。
    strafe_polygons 给出"操作区"多边形，落在里面的格子不受上面两条限制。
    状态随之扩成 (格 i, 格 j, 航向, 区外连续横移量)：精确累计，不按桶反复取整。
    四个航向保留真实初始角并相差90°；平移动作按最近主轴分类，车体姿态不取整。
    操作区内距离不消耗额度，穿过操作区不清零；前进/后退/转向清零。
    启发式 = 曼哈顿距离 × 最便宜的可行动作系数（可采纳且一致 ⇒ 加权代价仍最优）。
    回溯输出在 result['steps'] 里逐条给出 x, y, heading, action（同向直行合并，
    转向是独立的零位移步）。整车碰撞、合法区域、固定与动态障碍检查全部保留：
    每次平移都用**当时航向**的车体朝向整段检查；原地转向要求起止航向之间**采样旋转
    扫出区域的并集**（外加采样误差外扩）完全空闲。原台账保留原地转向，
    安全后处理的圆弧轨迹单独返回，不改变原搜索图与动作代价。
    """
    t0 = time.monotonic()
    base = dict(ok=False, search_ok=False, model_safe=False, execution_safe=False,
                hardware_ready=False, geometry_verified=bool(geometry_verified),
                points=[], raw=[], steps=[], length=0.0, search_cost=0.0, expanded=0,
                segments=[], corners=[], axis_matched=False,
                lateral_mm=0.0, turn_cost_mm=0.0, turn_count=0,
                grid=grid, pad=pad, min_clearance=None, body_clearance=None,
                boundary_clearance=None, start_in_inflate=None, goal_in_inflate=None,
                reason="", code="", frame_id="LAYOUT_MM", footprint=None,
                start_heading_deg=start_heading_deg, goal_heading_deg=goal_heading_deg,
                allow_strafe=bool(allow_strafe), strafe_run_limit_mm=None,
                max_strafe_run_mm=0.0, max_outside_strafe_run_mm=0.0)
    base.update(arcs=[], arc_fallbacks=[], arc_attempts=[], smoothed_primitives=[], smoothed_points=[],
                smoothed_length=0.0, smoothing_status="NOT_RUN", smoothing_model_safe=False)
    base.update(trajectory=[], trajectory_status="NOT_RUN", trajectory_safe=False,
                trajectory_continuous=False, trajectory_reason="", trajectory_length_mm=0.0,
                trajectory_spacing_mm=20.0, trajectory_frame_id="FIELD_MM",
                trajectory_yaw_convention="场地+X为0°、+Y为90°；切线方向，角度unwrap后可超出±180°")
    def failure(code, reason, **values):
        return dict(base, code=code, reason=reason, **values)
    try:
        start, goal = point2(start, "起点"), point2(goal, "目标")
        step = finite_number(grid, "网格")
        if not 10.0 <= step <= 500.0:
            raise ValueError("网格步长必须在10..500mm内")
        limit = finite_number(time_limit_s, "规划时限")
        if not 0 < limit <= 60:
            raise ValueError("规划时限须在0..60秒内")
        actual_start_yaw = finite_number(start_heading_deg, "起始航向") % 360.0
        h_start = heading_index(actual_start_yaw)
        heading_offset = ((actual_start_yaw - h_start * 90.0 + 180.0) % 360.0) - 180.0
        h_goal = None
        if goal_heading_deg is not None:
            actual_goal_yaw = finite_number(goal_heading_deg, "目标航向") % 360.0
            h_goal = heading_index(actual_goal_yaw - heading_offset)
            if abs(((actual_goal_yaw - (h_goal*90.0 + heading_offset) + 180) % 360)-180) > 1e-6:
                raise ValueError("目标航向须与起始航向相差90°整数倍；当前不支持任意角度转向")
        fwd = finite_number(DEFAULT_COST_FORWARD if cost_forward is None else cost_forward,
                            "前进代价系数")
        bwd = finite_number(DEFAULT_COST_BACKWARD if cost_backward is None else cost_backward,
                            "后退代价系数")
        lat = finite_number(DEFAULT_COST_LATERAL if cost_lateral is None else cost_lateral,
                            "横移代价系数")
        turn_pen = finite_number(DEFAULT_TURN_PENALTY_MM if turn_penalty_mm is None
                                 else turn_penalty_mm, "转向等效代价")
        if min(fwd, bwd, lat) <= 0.0 or turn_pen < 0.0:
            raise ValueError("动作代价系数必须为正（转向等效代价可为 0）")
        strafe_limit = None
        if strafe_run_limit_mm is not None:
            strafe_limit = finite_number(strafe_run_limit_mm, "横移上限")
            if strafe_limit < 0.0:
                raise ValueError("横移上限不能为负（0 等于完全禁横移）")
        base["strafe_run_limit_mm"] = strafe_limit
        scene = CollisionScene(rects, circles, bounds, pad, footprint, drivable_polygons,
                               sim_rects=sim_rects, sim_circles=sim_circles,
                               dynamic_rects=dynamic_rects, dynamic_circles=dynamic_circles)
        base.update(grid=step, pad=scene.pad,
                    start_heading_deg=actual_start_yaw,
                    goal_heading_deg=None if h_goal is None else actual_goal_yaw)
        if scene.footprint:
            delta_yaw = abs(((scene.footprint.yaw_deg - actual_start_yaw + 180.0) % 360.0)
                            - 180.0)
            if delta_yaw > 1e-6:
                raise ValueError("footprint 的航向必须等于 start_heading_deg："
                                 "车体朝向要与起始航向一致，不能静默换一个朝向求解")
            base["footprint"] = dict(length_mm=scene.footprint.length_mm,
                                     width_mm=scene.footprint.width_mm,
                                     yaw_deg=scene.footprint.yaw_deg)
        scene_polygons = drivable_polygons
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        return failure("INVALID_INPUT", str(exc))

    def heading_deg(h):
        """四个状态相差90°，整车姿态保留实际初始角，绝不取整车体。"""
        return (h * 90.0 + heading_offset) % 360.0

    scenes = {}
    def scene_for(h):
        """每个航向一套 CollisionScene：车体朝向随航向变，碰撞检查必须跟着变。"""
        if h not in scenes:
            fp = scene.footprint
            scenes[h] = CollisionScene(
                scene.rects, scene.circles, scene.bounds, scene.pad,
                None if fp is None else (fp.length_mm, fp.width_mm, heading_deg(h)),
                scene_polygons)
        return scenes[h]

    def interrupted():
        return ((cancel is not None and cancel.is_set()) or time.monotonic()-t0 > limit)
    if interrupted():
        return failure("CANCELLED", "规划已取消或超过时限")
    start_reason = scene_for(h_start).segment_reason(start, start)
    goal_heads = tuple(range(HEADING_COUNT)) if h_goal is None else (h_goal,)
    goal_reasons = {h: scene_for(h).segment_reason(goal, goal) for h in goal_heads}
    first_bad_goal = next((r for r in goal_reasons.values() if r), None)
    base.update(start_in_inflate=start_reason, goal_in_inflate=first_bad_goal)
    if start_reason:
        return failure("INVALID_START", "起点：" + start_reason)
    if first_bad_goal is not None and all(goal_reasons.values()):
        return failure("INVALID_GOAL", "目标：" + first_bad_goal)

    def finish(trace, expanded, cost):
        """按"当时航向"逐段复查 + 组装折线与 (x, y, heading, action) 台账。

        不能再用单一航向的 scene 去 validate 整条折线：路径里可能有原地转向，
        每段的合法性与它当时的车体朝向绑定。转向另查采样扫掠（见 turn_free_point）。
        连续横移上限在这里再查一遍：搜索判过一次，合并/回溯不能再放出更长的横移。
        """
        if interrupted():
            return failure("CANCELLED", "规划已取消或超过时限", expanded=expanded)
        run_at = [0.0] * len(trace)
        outside_run_at = [0.0] * len(trace)
        run_mm = 0.0
        limited_run = 0.0
        for idx in range(1, len(trace)):
            prev_pt, prev_h, _prev_act = trace[idx - 1]
            pt_, _h_, act_ = trace[idx]
            if act_ in (A_TURN_LEFT, A_TURN_RIGHT, A_TURN_AROUND):
                if not turn_free_point(prev_pt, prev_h, _h_):
                    return failure("INVALID_PATH", "第%d步转向：扫掠区域不空闲" % (idx + 1),
                                   expanded=expanded)
                run_at[idx] = run_mm = 0.0        # 转向打断连续横移
                limited_run = 0.0
                continue
            reason = scene_for(prev_h).segment_reason(prev_pt, pt_)
            if reason:
                return failure("INVALID_PATH", "第%d步：%s" % (idx + 1, reason),
                               expanded=expanded)
            used = advance_strafe(prev_pt, pt_, limited_run) if act_ == A_STRAFE else 0.0
            if used is None:
                return failure("INVALID_PATH",
                               "第%d步：操作区外连续横移超过限制" % (idx + 1), expanded=expanded)
            limited_run = used
            outside_run_at[idx] = (outside_run_at[idx-1] + outside_strafe_mm(prev_pt, pt_)
                                   if act_ == A_STRAFE else 0.0)
            run_mm = (run_mm + math.dist(prev_pt, pt_)) if act_ == A_STRAFE else 0.0
            run_at[idx] = run_mm
        pts = []
        for pt_, _h_, _act_ in trace:
            if not pts or math.dist(pts[-1], pt_) > EPS:
                pts.append(pt_)
        raw = _unique(pts)
        try:
            # check=False：合并只把同向共线段并成一条，扫掠区域不变，合法性已逐段确认
            points = simplify_collinear(raw, scene.segment_reason, check=False)
        except ValueError as exc:
            return failure("INVALID_PATH", str(exc), expanded=expanded)
        total_cost = lateral_mm = turn_cost_mm = 0.0
        turn_count = 0
        for idx in range(1, len(trace)):
            prev_pt, prev_h, _a = trace[idx - 1]
            pt_, h_, act_ = trace[idx]
            if act_ in (A_TURN_LEFT, A_TURN_RIGHT, A_TURN_AROUND):
                k = turn_steps(prev_h, h_)
                turn_count += 1
                turn_cost_mm += k * turn_pen
                total_cost += k * turn_pen
                continue
            dist = math.dist(prev_pt, pt_)
            total_cost += dist * move_factor(act_, fwd, bwd, lat)
            if act_ == A_STRAFE:
                lateral_mm += dist
        steps = [{"x": trace[0][0][0], "y": trace[0][0][1],
                  "to_x": trace[0][0][0], "to_y": trace[0][0][1],
                  "heading_deg": heading_deg(trace[0][1]), "action": A_START,
                  "kind": "START", "distance_mm": 0.0, "cost_mm": 0.0,
                  "strafe_run_mm": 0.0, "outside_strafe_run_mm": 0.0}]
        prev_pt, prev_h = trace[0][0], trace[0][1]
        for idx in range(1, len(trace)):
            pt_, h_, act_ = trace[idx]
            is_turn = act_ in (A_TURN_LEFT, A_TURN_RIGHT, A_TURN_AROUND)
            last = steps[-1]
            same_direction = ((last["to_x"]-last["x"])*(pt_[0]-prev_pt[0]) +
                              (last["to_y"]-last["y"])*(pt_[1]-prev_pt[1])) > EPS
            if (not is_turn and same_direction and last["action"] == act_ and
                    last["heading_deg"] == heading_deg(h_)):
                steps[-1]["to_x"], steps[-1]["to_y"] = pt_
                steps[-1]["strafe_run_mm"] = run_at[idx]
                steps[-1]["outside_strafe_run_mm"] = outside_run_at[idx]
                prev_pt, prev_h = pt_, h_
                continue
            steps.append({"x": prev_pt[0], "y": prev_pt[1], "to_x": pt_[0], "to_y": pt_[1],
                          "heading_deg": heading_deg(h_), "action": act_,
                          "kind": "TURN" if is_turn else "MOVE",
                          "distance_mm": 0.0, "strafe_run_mm": run_at[idx],
                          "outside_strafe_run_mm": outside_run_at[idx],
                          "cost_mm": 0.0 if not is_turn
                          else turn_steps(prev_h, h_) * turn_pen})
            prev_pt, prev_h = pt_, h_
        for row in steps:
            dist = math.dist((row["x"], row["y"]), (row["to_x"], row["to_y"]))
            row["distance_mm"] = dist
            if row["kind"] == "MOVE":
                row["cost_mm"] = dist * move_factor(row["action"], fwd, bwd, lat)
        # 共线压缩后的直角折线 → 线段 + 90° 角点（动作/航向语义沿用上面的台账；
        # 对不上就留空串 + 报出 axis_matched=False，绝不重新猜动作）
        # 几何共线压缩不能代表动作切换；动作分段保留每条 MOVE 的端点。
        action_points = _unique([trace[0][0]] +
                                [(row["to_x"], row["to_y"]) for row in steps
                                 if row["kind"] == "MOVE"])
        segments, corners = path_axes(action_points, steps)
        axis_matched = all(s.action and s.heading_deg is not None for s in segments)
        clearance = centre_clearance(points, scene.rects, scene.circles)
        bx0, by0, bx1, by1 = scene.bounds
        bc = min(min(p[0]-bx0, bx1-p[0], p[1]-by0, by1-p[1]) for p in points)
        body_clearance = None
        if scene.footprint is not None:
            best = scene_for(trace[0][1]).body_clearance([trace[0][0]])
            body_turns = {}
            for idx in range(1, len(trace)):
                prev_pt, prev_h, _a = trace[idx - 1]
                pt_, _h_, act_ = trace[idx]
                if act_ in (A_TURN_LEFT, A_TURN_RIGHT, A_TURN_AROUND):
                    pair = (prev_h, _h_)
                    if pair not in body_turns:
                        body_turns[pair] = rotation_sweep(scene.footprint, 0.0,
                                                        heading_deg(prev_h), heading_deg(_h_))
                    swept = translate(body_turns[pair], xoff=prev_pt[0], yoff=prev_pt[1])
                    best = min(best, scene.sweep_clearance(swept))
                    continue
                c = scene_for(prev_h).body_clearance([prev_pt, pt_])
                if c is not None:
                    best = min(best, c)
            body_clearance = None if not math.isfinite(best) else best
        result = dict(base, ok=True, search_ok=True, model_safe=True,
                    # Only explicit full body geometry may be run in the SIMULATOR.
                    execution_safe=scene.footprint is not None,
                    points=points, raw=raw, steps=steps, length=length(points),
                    segments=segments, corners=corners, axis_matched=axis_matched,
                    search_cost=cost, trace_cost=total_cost, expanded=expanded,
                    lateral_mm=lateral_mm, turn_cost_mm=turn_cost_mm, turn_count=turn_count,
                    max_strafe_run_mm=max(run_at),
                    max_outside_strafe_run_mm=max(outside_run_at),
                    min_clearance=clearance, boundary_clearance=bc,
                    body_clearance=body_clearance, code="OK", reason="")
        if smooth_arcs:
            from arc_smoothing import smooth_90_corners
            result.update(smooth_90_corners(segments, corners, steps, scene,
                                           interrupted=interrupted))
            if result['smoothing_model_safe']:
                from trajectory import generate_trajectory
                result.update(generate_trajectory(result['smoothed_primitives'], scene, interrupted=interrupted))
            else:
                result.update(trajectory_status="UNAVAILABLE", trajectory_reason="圆弧后处理未提供安全的完整几何轨迹")
            if interrupted():
                return failure("CANCELLED", "规划、平滑或Trajectory已取消或超过时限", expanded=expanded)
        else:
            result["smoothing_status"] = "DISABLED"
            result.update(trajectory_status="DISABLED", trajectory_reason="圆弧后处理已关闭")
        return result

    x0, y0, x1, y1 = scene.bounds
    # floor, never round: no out-of-field cells (e.g. 2450 in a 2400 field).
    nx = math.floor((x1-x0)/step) + 1
    ny = math.floor((y1-y0)/step) + 1
    if nx*ny > MAX_CELLS:
        return failure("RESOURCE_LIMIT", "网格数量超过100000；使用更粗网格，不减少安全裕量")

    def xy(cell):
        return (x0 + cell[0]*step, y0 + cell[1]*step)

    def clamp_cell(p):
        return (min(nx-1, max(0, round((p[0]-x0)/step))),
                min(ny-1, max(0, round((p[1]-y0)/step))))

    free_cache = {}
    edge_cache = {}

    def free(cell, h):
        """格点在该航向下整车是否合法（车体朝向随航向变，缓存键含航向）。"""
        key = (cell, h)
        if key not in free_cache:
            free_cache[key] = scene_for(h).segment_reason(xy(cell), xy(cell)) is None
        return free_cache[key]

    def edge_free(a, b, h):
        """a→b 这条边在 h 航向下的整段检查（缓存键含航向）。"""
        key = (a, b, h)
        if key not in edge_cache:
            edge_cache[key] = scene_for(h).segment_reason(xy(a), xy(b)) is None
        return edge_cache[key]

    def point_edge_free(a, b, h):
        return scene_for(h).segment_reason(a, b) is None

    # 每种航向对只构造一次扫掠，再按准确车心平移；不量化碰撞缓存坐标。
    turn_cache = {}
    sweep_cache = {}

    def turn_free_point(p, from_h, to_h):
        if scene.footprint is None:
            return True
        pair = (from_h, to_h)
        if pair not in sweep_cache:
            sweep_cache[pair] = rotation_sweep(scene.footprint, scene.pad,
                                              heading_deg(from_h), heading_deg(to_h))
        key = (p, from_h, to_h)
        if key not in turn_cache:
            turn_cache[key] = scene.turn_reason(p, heading_deg(from_h), heading_deg(to_h),
                                                sweep_cache[pair]) is None
        return turn_cache[key]

    def turn_free(cell, from_h, to_h):
        return turn_free_point(xy(cell), from_h, to_h)

    # 操作区豁免只覆盖区内距离；区外距离跨操作区继续累计，只有非横移动作清零。
    strafe_area = None
    if strafe_polygons:
        try:
            strafe_area = allowed_area(strafe_polygons, scene.bounds)
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            return failure("INVALID_INPUT", "横移许可区非法：" + str(exc))
    outside_cache = {}

    def outside_strafe_mm(a, b):
        if strafe_area is None:
            return math.dist(a, b)
        key = (a, b)
        if key not in outside_cache:
            outside_cache[key] = _line(a, b).difference(strafe_area).length
        return outside_cache[key]

    def advance_strafe(a, b, used):
        if allow_strafe and strafe_limit is None:
            return 0.0
        used += outside_strafe_mm(a, b)
        effective_limit = strafe_limit if allow_strafe else 0.0
        return used if used <= effective_limit + EPS else None

    def sequence_run(origin, seq, initial=0.0):
        used, pos = initial, origin
        for target, _h, action in seq:
            used = advance_strafe(pos, target, used) if action == A_STRAFE else 0.0
            if used is None:
                return None
            pos = target
        return used

    def connector_seq(legs, h_from, h_target, force_align=False):
        """连接段保留横移和转向两种候选；区外额度逐段精确累计。"""
        seq, cost, used, h = [], 0.0, 0.0, h_from
        pos = None
        for la, lb in legs:
            leg_mm = math.dist(la, lb)
            if leg_mm <= EPS:
                continue
            if pos is not None and math.dist(pos, la) > EPS:
                return None
            di = 0 if abs(lb[0]-la[0]) <= EPS else (1 if lb[0] > la[0] else -1)
            dj = 0 if abs(lb[1]-la[1]) <= EPS else (1 if lb[1] > la[1] else -1)
            act = action_for_move(h, di, dj)
            next_used = advance_strafe(la, lb, used) if act == A_STRAFE else 0.0
            if act == A_STRAFE and (force_align or next_used is None):
                want = heading_index(math.degrees(math.atan2(dj, di)))
                if force_align == "backward":
                    want = (want + 2) % HEADING_COUNT
                if not turn_free_point(la, h, want):
                    return None
                seq.append((la, want, turn_action(h, want)))
                cost += turn_steps(h, want) * turn_pen
                h, act, next_used = want, action_for_move(want, di, dj), 0.0
            if not point_edge_free(la, lb, h):
                return None
            seq.append((lb, h, act))
            cost += leg_mm * move_factor(act, fwd, bwd, lat)
            used, pos = next_used, lb
        if h != h_target:
            if pos is None or not turn_free_point(pos, h, h_target):
                return None
            seq.append((pos, h_target, turn_action(h, h_target)))
            cost += turn_steps(h, h_target) * turn_pen
            used = 0.0
        return cost, seq, used

    def connectors(a, b, h, h_target):
        seen = set()
        for corner in ((b[0], a[1]), (a[0], b[1])):
            for align in (False, True, "backward"):
                built = connector_seq(((a, corner), (corner, b)), h, h_target, align)
                if built is not None:
                    key = tuple(built[1])
                    if key not in seen:
                        seen.add(key)
                        yield built

    # 直接候选是上界；只有达到可采纳下界时才允许跳过搜索。
    direct_trace, direct_cost = None, math.inf
    target_h = h_start if h_goal is None else h_goal
    if start == goal:
        if target_h == h_start:
            return finish([(start, h_start, A_START)], 0, 0.0)
        if turn_free_point(start, h_start, target_h):
            direct_trace = [(start, h_start, A_START),
                            (start, target_h, turn_action(h_start, target_h))]
            direct_cost = turn_steps(h_start, target_h) * turn_pen
    else:
        for cost, seq, _used in connectors(start, goal, h_start, target_h):
            if cost < direct_cost:
                direct_trace = [(start, h_start, A_START)] + seq
                direct_cost = cost
    min_move_cost = min(fwd, bwd, lat)
    direct_lower = (abs(start[0]-goal[0])+abs(start[1]-goal[1])) * min_move_cost
    if h_goal is not None:
        direct_lower += turn_steps(h_start, h_goal) * turn_pen
    if direct_trace is not None and direct_cost <= direct_lower + EPS:
        return finish(direct_trace, 0, direct_cost)

    def attach(p, h, from_point):
        """保留连接候选的代价、序列与末尾额度；不抢先选掉另一种 L 形。"""
        i0, j0 = clamp_cell(p)
        for i in range(max(0, i0-8), min(nx, i0+9)):
            if interrupted():
                return
            for j in range(max(0, j0-8), min(ny, j0+9)):
                cell = (i, j)
                if not free(cell, h):
                    continue
                a, b = (p, xy(cell)) if from_point else (xy(cell), p)
                for cost, seq, used in connectors(a, b, h, h):
                    yield cell, cost, seq, used

    sources = {}
    for cell, cost, seq, used in attach(start, h_start, True):
        state = (cell[0], cell[1], h_start, used)
        if state not in sources or cost < sources[state][0]:
            sources[state] = (cost, seq)
    goals = {}
    for h in goal_heads:
        for cell, cost, seq, _used in attach(goal, h, False):
            goals.setdefault((cell[0], cell[1], h), []).append((cost, seq))
    if interrupted():
        return failure("CANCELLED", "规划已取消或超过时限")
    if not sources or not goals:
        if direct_trace is not None:
            return finish(direct_trace, 0, direct_cost)
        hint = ("；已完全禁横移：起点/终点附近若转不动，需放宽横移上限或移离边界"
                if not allow_strafe or strafe_limit == 0 else "")
        return failure("NO_CONNECTION", "起点/目标没有合法网格接入（含横移/转向约束）" + hint)

    def h_state(st):
        p = xy(st)
        value = (abs(p[0]-goal[0])+abs(p[1]-goal[1])) * min_move_cost
        if h_goal is not None:
            value += turn_steps(st[2], h_goal) * turn_pen
        return value

    terminal = (-1, -1, -1, -1)
    came = {s: None for s in sources}
    scores = {s: entry[0] for s, entry in sources.items()}
    heap = [(entry[0]+h_state(s), entry[0], s) for s, entry in sources.items()
            if entry[0]+h_state(s) <= direct_cost + EPS]
    goal_seq = None
    if direct_trace is not None:
        scores[terminal], came[terminal] = direct_cost, None
        heap.append((direct_cost, direct_cost, terminal))
    heapq.heapify(heap)
    expanded = 0
    while heap:
        _, g, cur = heapq.heappop(heap)
        if g != scores.get(cur):
            continue
        if interrupted():
            return failure("CANCELLED", "规划已取消或超过时限", expanded=expanded)
        expanded += 1
        if cur == terminal:
            if came[terminal] is None:
                return finish(direct_trace, expanded, g)
            states = []
            st = came[terminal]
            while st is not None:
                states.append(st)
                st = came[st]
            states.reverse()
            trace = [(start, h_start, A_START)] + sources[states[0]][1]
            for prev, nxt in zip(states, states[1:]):
                if nxt[2] != prev[2]:
                    trace.append((xy(prev), nxt[2], turn_action(prev[2], nxt[2])))
                else:
                    trace.append((xy(nxt), nxt[2],
                                  action_for_move(nxt[2], nxt[0]-prev[0], nxt[1]-prev[1])))
            trace.extend(goal_seq)
            return finish(trace, expanded, g)
        for cost, seq in goals.get(cur[:3], ()):
            if sequence_run(xy(cur), seq, cur[3]) is None:
                continue
            ng = g + cost
            if ng < scores.get(terminal, math.inf):
                scores[terminal], came[terminal], goal_seq = ng, cur, seq
                heapq.heappush(heap, (ng, ng, terminal))
        for di, dj in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            cell = (cur[0]+di, cur[1]+dj)
            if not (0 <= cell[0] < nx and 0 <= cell[1] < ny):
                continue
            act = action_for_move(cur[2], di, dj)
            used = advance_strafe(xy(cur), xy(cell), cur[3]) if act == A_STRAFE else 0.0
            if used is None:
                continue
            nxt = (cell[0], cell[1], cur[2], used)
            ng = g + step * move_factor(act, fwd, bwd, lat)
            if ng + h_state(nxt) > scores.get(terminal, math.inf) + EPS:
                continue
            if not free(cell, nxt[2]) or not edge_free((cur[0], cur[1]), cell, cur[2]):
                continue
            if ng < scores.get(nxt, math.inf):
                scores[nxt], came[nxt] = ng, cur
                heapq.heappush(heap, (ng+h_state(nxt), ng, nxt))
        for dh in (1, 2, 3):
            nxt = (cur[0], cur[1], (cur[2]+dh) % HEADING_COUNT, 0.0)
            ng = g + turn_steps(cur[2], nxt[2]) * turn_pen
            if ng + h_state(nxt) > scores.get(terminal, math.inf) + EPS:
                continue
            if not free((cur[0], cur[1]), nxt[2]) or not turn_free((cur[0], cur[1]), cur[2], nxt[2]):
                continue
            if ng < scores.get(nxt, math.inf):
                scores[nxt], came[nxt] = ng, cur
                heapq.heappush(heap, (ng+h_state(nxt), ng, nxt))
        if len(scores) > 400000:
            return failure("RESOURCE_LIMIT", "搜索状态超过400000；请使用更粗网格", expanded=expanded)
    return failure("NO_PATH", "通路封死或当前网格无法表示安全路径；更换目标/路线或细化网格", expanded=expanded)


def approach_point(anchor, outward, *, margin, rects, circles, bounds,
                   footprint=None, drivable_polygons=None, standoff=0.0,
                   max_mm=2000.0, step=5.0):
    """从功能区锚点沿 outward 向外找一个"停得下、且车体四周留得开"的车心。

    锚点取功能区朝场内的一侧（圆盘用圆心、设备用朝向场地那条边的中点），
    outward 是由功能区指向场内的单位向量。合法性与净空都由**同一个**
    CollisionScene 判定，所以"这里给的接近点"和"plan() 能规划到的目标"
    永远是同一套模型。

    standoff 是要求车体**外缘**到最近障碍（含合法行驶区边界）的净空，不是
    "从刚好合法再走多远"。判据用连续的车体净空而不是"扫到第一个合法点"：
    后者会整段跳过窄的可停窗口（实测 pad=60mm 时原料区那条缝只有几十毫米宽，
    而它位于"刚好合法距离"的内侧），于是明明能停却报没有位置。

    找不到满足 standoff 的点时 ok=False，并把最靠内、仅仅"合法但净空不足"的
    fallback_point 一并给出，由调用方决定是否明示出来——绝不静默采用它。
    """
    try:
        anchor = point2(anchor, "锚点")
        ux, uy = point2(outward, "方向")
        norm = math.hypot(ux, uy)
        if norm <= EPS:
            raise ValueError("方向向量不能为零")
        ux, uy = ux / norm, uy / norm
        scene = CollisionScene(rects, circles, bounds, margin, footprint, drivable_polygons)
        standoff = max(0.0, finite_number(standoff, "站立距离"))
        limit = finite_number(max_mm, "搜索上限")
        if not 0 < limit <= 10000:
            raise ValueError("搜索上限须在0..10000mm内")
        step = finite_number(step, "搜索步长")
        if not 0.5 <= step <= 100:
            raise ValueError("搜索步长须在0.5..100mm内")
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        return {"ok": False, "point": None, "distance": None, "required": None,
                "clearance": None, "pad": None, "footprint": None,
                "fallback_point": None, "fallback_clearance": None,
                "code": "INVALID_INPUT", "reason": str(exc)}

    def at(d):
        return (anchor[0] + ux * d, anchor[1] + uy * d)

    required = None
    fallback = None
    d = 0.0
    while d <= limit + EPS:
        point = at(d)
        if scene.segment_reason(point, point) is None:
            if required is None:
                required = d
            clearance = scene.body_clearance([point, point])
            if clearance is None:                 # 兼容模式：没有车体模型，只保证合法
                clearance = math.inf
            if clearance + EPS >= standoff:
                return {"ok": True, "point": point, "distance": d,
                        "required": required, "clearance": (None if math.isinf(clearance) else clearance),
                        "pad": scene.pad,
                        "footprint": None if scene.footprint is None else
                                     dict(length_mm=scene.footprint.length_mm,
                                          width_mm=scene.footprint.width_mm,
                                          yaw_deg=scene.footprint.yaw_deg),
                        "fallback_point": None, "fallback_clearance": None,
                        "code": "OK", "reason": ""}
            if fallback is None:
                fallback = (point, None if math.isinf(clearance) else clearance)
        d += step
    if required is None:
        return {"ok": False, "point": None, "distance": None, "required": None,
                "clearance": None, "pad": scene.pad, "footprint": None,
                "fallback_point": None, "fallback_clearance": None,
                "code": "NO_LEGAL_POSE",
                "reason": "沿该方向 %.0fmm 内没有合法停车位" % limit}
    return {"ok": False, "point": None, "distance": None, "required": required,
            "clearance": None, "pad": scene.pad, "footprint": None,
            "fallback_point": None if fallback is None else fallback[0],
            "fallback_clearance": None if fallback is None else fallback[1],
            "code": "NO_STANDOFF",
            "reason": "沿该方向 %.0fmm 内没有既合法、又能留出 %.0fmm 车体净空的停车位%s"
                      % (limit, standoff,
                         "" if fallback is None else
                         "；最近合法点 %s 净空仅 %s"
                         % (tuple(round(v, 1) for v in fallback[0]),
                            "未知" if fallback[1] is None else "%.0fmm" % fallback[1]))}


# ---------------- 航向状态与动作代价（4 邻域平移 + 原地转向，不做圆弧） ----------------
# 状态 = (格号 i, 格号 j, 航向 h)；h ∈ {0,1,2,3} 对应布局帧 0/90/180/270°（逆时针）。
# 平移仍是严格 4 邻域：相对当前航向分为 前进/后退/横移，代价 = 步长 × 系数；
# 转向只在原地做 90° 的整数倍（TURN_LEFT/RIGHT/AROUND），按 90° 次数计罚，没有圆弧。
HEADING_COUNT = 4
HEADING_STEP_DEG = 90.0
A_FORWARD = "FORWARD"
A_BACKWARD = "BACKWARD"
A_STRAFE = "STRAFE"
A_TURN_LEFT = "TURN_LEFT"
A_TURN_RIGHT = "TURN_RIGHT"
A_TURN_AROUND = "TURN_AROUND"
A_START = "START"
A_GOAL = "GOAL"
DEFAULT_COST_FORWARD = 1.0
DEFAULT_COST_BACKWARD = 1.15
DEFAULT_COST_LATERAL = 1.8
DEFAULT_TURN_PENALTY_MM = 180.0     # 每 90° 原地转向的等效代价（mm）；180° 记两次


def heading_index(deg) -> int:
    """任意角度 → 0..3 的航向号（就近取整到 90° 的整数倍，逆时针为正）。"""
    return int(round(finite_number(deg, "航向") / HEADING_STEP_DEG)) % HEADING_COUNT


def heading_deg(index: int) -> float:
    return (int(index) % HEADING_COUNT) * HEADING_STEP_DEG


def heading_vector(index: int):
    """航向对应的前向单位向量（布局帧，x 右 y 上，逆时针）。"""
    rad = math.radians(heading_deg(index))
    return (math.cos(rad), math.sin(rad))


def action_for_move(heading_idx: int, di: int, dj: int) -> str:
    """把一个 4 邻域平移按当前航向分类为 前进/后退/横移。"""
    fx, fy = heading_vector(heading_idx)
    dot = di * fx + dj * fy
    if dot > 0.5:
        return A_FORWARD
    if dot < -0.5:
        return A_BACKWARD
    return A_STRAFE


def move_factor(action: str, cost_forward=DEFAULT_COST_FORWARD,
                cost_backward=DEFAULT_COST_BACKWARD,
                cost_lateral=DEFAULT_COST_LATERAL) -> float:
    """单位距离的代价系数：横移按 cost_lateral 计，前进最便宜。"""
    if action == A_FORWARD:
        return float(cost_forward)
    if action == A_BACKWARD:
        return float(cost_backward)
    if action == A_STRAFE:
        return float(cost_lateral)
    raise ValueError(f"未知动作：{action}")


def turn_action(from_idx: int, to_idx: int) -> str:
    """两个航向之间的最短原地转向动作名。"""
    delta = (int(to_idx) - int(from_idx)) % HEADING_COUNT
    if delta == 1:
        return A_TURN_LEFT
    if delta == 3:
        return A_TURN_RIGHT
    if delta == 2:
        return A_TURN_AROUND
    raise ValueError("航向未改变，不是转向动作")


def turn_steps(from_idx: int, to_idx: int) -> int:
    """最短转向需要几个 90°（180° 记 2）。"""
    delta = (int(to_idx) - int(from_idx)) % HEADING_COUNT
    return 0 if delta == 0 else (2 if delta == 2 else 1)


def load_map(path):
    path = Path(path)
    if path.stat().st_size > 2_000_000:
        raise ValueError("地图JSON超过2MB")
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if data.get("schema_version") != 1 or data.get("frame_id") != "LAYOUT_MM":
        raise ValueError("地图须为schema_version=1，frame_id=LAYOUT_MM")
    for key in ("map_id", "map_version", "geometry_verified", "bounds", "drivable_polygons", "rects", "circles"):
        if key not in data:
            raise ValueError(f"地图缺字段：{key}")
    if not isinstance(data["geometry_verified"], bool):
        raise ValueError("geometry_verified须为布尔值")
    scene = CollisionScene(data["rects"], data["circles"], data["bounds"],
                           drivable_polygons=data["drivable_polygons"],
                           dynamic_rects=data.get("dynamic_rects"),
                           dynamic_circles=data.get("dynamic_circles"))
    if scene.bounds != (0.0, 0.0, 2400.0, 2400.0):
        raise ValueError("当前FieldView仅支持2400×2400mm布局")
    return data
