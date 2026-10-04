"""PC连续跟踪几何：实际位置单调投影、100mm弧长lookahead，无航点到位门。"""
from bisect import bisect_right
import math

from core import layout_to_field, field_to_layout, SEND_HZ
from navigation_planner import EPS, finite_number, point2
from trajectory import _normalize, _ends, _pose_at, _tangent_at, _motion_mode

LOOKAHEAD_MM = 100.0


class TrajectoryTracker:
    def __init__(self, samples, primitives):
        if len(samples) < 2:
            raise ValueError('连续跟踪需要至少两个Trajectory样本')
        self.pieces = _normalize(primitives)
        if not self.pieces:
            raise ValueError('缺少连续几何')
        self.ends = _ends(self.pieces)
        self.length = self.ends[-1]
        self.progress = 0.0
        self.last_position = None
        self.cross_track = 0.0
        self.yaws = [finite_number(samples[0]['field_yaw_deg'], '起始切线航向')]
        self.tangent_yaws = [finite_number(samples[0].get('tangent_yaw_deg', samples[0]['field_yaw_deg']), '起始行驶切线')]
        for before, after in zip(self.pieces, self.pieces[1:]):
            change = before['yaw_out']-before['yaw_in']
            self.yaws.append(self.yaws[-1]-change)
            self.tangent_yaws.append(self.tangent_yaws[-1]-(before['tangent_out']-before['tangent_in']))
        if abs(finite_number(samples[-1]['s_mm'], '最终弧长')-self.length) > 1e-5:
            raise ValueError('Trajectory与连续几何总长不一致')

    def reference_at(self, station):
        station = max(0.0, min(self.length, finite_number(station, '参考弧长')))
        index = min(bisect_right(self.ends, station), len(self.pieces)-1)
        begin = self.ends[index-1] if index else 0.0
        piece = self.pieces[index]
        lx, ly, yaw = _pose_at(piece, station-begin)
        fx, fy = layout_to_field(lx, ly)
        reference = dict(x_mm=fx, y_mm=fy, field_yaw_deg=self.yaws[index]-(yaw-piece['yaw_in']),
                         s_mm=station, segment_type='STOP' if station >= self.length-EPS else piece['kind'])
        if piece['explicit_heading']:
            tangent = _tangent_at(piece, station-begin)
            reference.update(tangent_yaw_deg=self.tangent_yaws[index]-(tangent-piece['tangent_in']),
                             motion_mode=_motion_mode(-90-yaw, -90-tangent))
        return reference

    def update_progress(self, position):
        position = point2(position, '实际位置')
        travel = 0.0 if self.last_position is None else math.dist(self.last_position, position)
        # 只在上一进度前方、实际位移+一段采样长度内投影，防止U形/自交路线跳到后支。
        high = min(self.length, self.progress+travel+20.0)
        low = self.progress
        lx, ly = field_to_layout(*position)
        best = (math.inf, low)
        first = max(0, bisect_right(self.ends, low)-1)
        for index in range(first, len(self.pieces)):
            piece = self.pieces[index]
            begin = self.ends[index-1] if index else 0.0
            if begin > high+EPS:
                break
            a, b = max(0.0, low-begin), min(piece['length_mm'], high-begin)
            if b < a-EPS:
                continue
            candidates = [a, b]
            if piece['kind'] == 'LINE':
                start, end = piece['start'], piece['end']
                ux, uy = ((end[k]-start[k])/piece['length_mm'] for k in range(2))
                candidates.append(max(a, min(b, (lx-start[0])*ux+(ly-start[1])*uy)))
            else:
                center = piece['center']
                angle = math.degrees(math.atan2(ly-center[1], lx-center[0]))
                sign = 1 if piece['sweep_deg'] > 0 else -1
                offset = ((angle-piece['start_angle_deg'])*sign) % 360
                for turn in (offset, offset-360):
                    local = math.radians(turn)*piece['radius_mm']
                    candidates.append(max(a, min(b, local)))
            for local in candidates:
                px, py, _yaw = _pose_at(piece, local)
                distance, station = math.hypot(lx-px, ly-py), begin+local
                # 等距时保留较早分支；进度永不回退。
                if distance < best[0]-EPS or abs(distance-best[0]) <= EPS and station < best[1]:
                    best = distance, station
        self.progress = max(self.progress, best[1])
        self.cross_track = best[0]
        self.last_position = position
        return self.progress

    def command(self, pose, speed_limit, yaw_rate_limit):
        x, y, yaw = (finite_number(value, '实际姿态') for value in pose)
        self.update_progress((x, y))
        ref = self.reference_at(self.progress+LOOKAHEAD_MM)
        dx, dy = ref['x_mm']-x, ref['y_mm']-y
        distance = math.hypot(dx, dy)
        yaw_error = (ref['field_yaw_deg']-yaw+180) % 360-180
        speed_limit = max(0.0, finite_number(speed_limit, '模拟限速'))
        yaw_rate_limit = max(0.0, finite_number(yaw_rate_limit, '模拟角速度'))
        # 前视窗口提前按最小圆弧限速，进入小半径时保留角速度余量。
        index = min(bisect_right(self.ends, self.progress), len(self.pieces)-1)
        for i in range(index, len(self.pieces)):
            begin = self.ends[i-1] if i else 0.0
            if begin > ref['s_mm']+EPS:
                break
            piece = self.pieces[i]
            if piece['kind'] == 'ARC' and piece['heading_mode'] != 'FIXED':
                speed_limit = min(speed_limit, piece['radius_mm']*math.radians(yaw_rate_limit)*.8)
        speed = min(speed_limit, distance*4.0) if ref['segment_type'] == 'STOP' else speed_limit
        # 100mm参考保留；车体沿当前投影处的切线/曲率运动，避免直接追前视弦而切内。
        # 弧长只来自实际位置的单调投影，不按帧数推进，也没有中间采样点到位门。
        here = self.reference_at(self.progress)
        ahead = self.reference_at(self.progress+speed/SEND_HZ)
        vx = SEND_HZ*(ahead['x_mm']-here['x_mm'])+6*(here['x_mm']-x)
        vy = SEND_HZ*(ahead['y_mm']-here['y_mm'])+6*(here['y_mm']-y)
        magnitude = math.hypot(vx, vy)
        if magnitude > speed_limit and magnitude > EPS:
            vx, vy = vx*speed_limit/magnitude, vy*speed_limit/magnitude
        heading_error = (here['field_yaw_deg']-yaw+180) % 360-180
        omega = SEND_HZ*(ahead['field_yaw_deg']-here['field_yaw_deg'])+6*heading_error
        omega = max(-yaw_rate_limit, min(yaw_rate_limit, omega))
        if ref['segment_type'] == 'STOP' and distance <= .5 and abs(yaw_error) <= .3:
            vx, vy, omega = 0.0, 0.0, 0.0
        return ref, (vx, vy), omega
