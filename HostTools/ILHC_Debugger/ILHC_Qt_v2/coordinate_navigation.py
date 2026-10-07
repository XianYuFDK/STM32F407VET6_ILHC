"""PC关键坐标定位：车体轴位置闭环，以及经完整扫掠校验的指定圆心过弯。"""
import copy
import heapq
import itertools
import json
import math
import time
from pathlib import Path

import navigation_planner as nav
from core import CHASSIS_DEFAULTS, SEND_HZ

KIND = 'PC_COORDINATE_PROGRAM'
DEFAULT_CONTROL = dict(CHASSIS_DEFAULTS, turn_lead_mm=200.0)
from mecanum_geometry import ROTATION_LEVER_MM
RPM_PER_MM_S = .238
MAX_WHEEL_RPM = 3000.0


def numerical_limit(value, maximum, compensation):
    # 与firmware numerical_limit(...,5.0f)一致：死区内保留P输出，区外加补偿。
    if value > 5:
        value += compensation
    elif value < -5:
        value -= compensation
    return max(-maximum, min(maximum, value))


def wrap(angle):
    return (angle+180) % 360-180


class StopWindow:
    """Same200ms whole-pose span check as the coordinate firmware."""
    def __init__(self):
        self.bounds=None
        self.elapsed_ms=0

    def update(self,pose,eligible,dt):
        if not eligible or not 0<dt<=.2:
            self.bounds=None;self.elapsed_ms=0
            return 0
        if self.bounds is None:
            self.anchor_yaw=pose[2]
            self.bounds=[pose[0],pose[0],pose[1],pose[1],0.0,0.0]
            self.elapsed_ms=0
            return 0
        for i,value in enumerate((pose[0],pose[1],wrap(pose[2]-self.anchor_yaw))):
            self.bounds[2*i]=min(self.bounds[2*i],value)
            self.bounds[2*i+1]=max(self.bounds[2*i+1],value)
        if (math.hypot(self.bounds[1]-self.bounds[0],self.bounds[3]-self.bounds[2])>.2 or
                self.bounds[5]-self.bounds[4]>.2):
            self.bounds=None;self.elapsed_ms=0
            return 0
        self.elapsed_ms+=round(dt*1000)
        return self.elapsed_ms


class CoordinateTracker:
    def __init__(self, program):
        from core import layout_to_field
        if (not isinstance(program, dict) or program.get('kind') != KIND or program.get('schema_version') not in (2,3) or
                program.get('frame_id') != 'LAYOUT_MM'):
            raise ValueError('关键坐标程序格式错误或旧参数模型，请重新规划')
        rows = copy.deepcopy(program.get('waypoints'))
        if not isinstance(rows, list) or not 2 <= len(rows) <= 4096:
            raise ValueError('关键坐标需要2..4096个点')
        self.points = [];self.pivots={}
        for i, row in enumerate(rows):
            x, y = nav.point2((row['x_mm'], row['y_mm']), '关键坐标')
            yaw = nav.finite_number(row['layout_yaw_deg'], '关键坐标车头')
            travel = nav.finite_number(row.get('travel_yaw_deg', yaw), '行驶车头')
            expected = 'START' if i == 0 else 'STOP' if i == len(rows)-1 else ('STOP' if row.get('kind')=='STOP' else 'PASS')
            if row.get('kind') != expected:
                raise ValueError('首点START、中间PASS或STOP、末点STOP')
            radius = nav.finite_number(row.get('pass_mm', 0), '通过范围')
            if expected == 'PASS' and not 0 < radius <= 100 or expected != 'PASS' and radius != 0:
                raise ValueError('通过范围须为0..100mm，STOP不可提前放行')
            fx, fy = layout_to_field(x, y)
            self.points.append(dict(x_mm=fx, y_mm=fy, field_yaw_deg=-90-yaw,
                                    travel_yaw_deg=-90-travel, kind=expected, pass_mm=radius))
            if 'turn_lead_mm' in row:
                lead=nav.finite_number(row['turn_lead_mm'],'点位提前转向距离')
                if not 0<lead<=2000:raise ValueError('点位提前转向距离须0..2000mm')
                self.points[-1]['turn_lead_mm']=lead
            if row.get('finish_heading_before_pass'):
                self.points[-1]['finish_heading_before_pass']=True
            if row.get('motion') is not None:
                if row['motion'] not in ('PIVOT','WHEEL_PIVOT') or i==0 or program['schema_version']!=3:
                    raise ValueError('指定圆心动作格式错误或旧程序版本')
                from pivot_turns import geometry
                previous=rows[i-1]
                center=nav.point2(row.get('pivot_center_mm'),'旋转圆心')
                arc_radius,angle=geometry((previous['x_mm'],previous['y_mm']),(x,y),center,
                                          previous['layout_yaw_deg'],yaw)
                cx,cy=layout_to_field(*center)
                self.pivots[i]=dict(center=(cx,cy),radius=arc_radius,angle=-math.radians(angle))
                if row['motion']=='WHEEL_PIVOT':
                    from pivot_turns import wheel_anchor
                    dimensions=program.get('wheel_geometry')
                    if not isinstance(dimensions,dict):raise ValueError('麦轮支点缺少轮心尺寸')
                    anchors={wheel:wheel_anchor((previous['x_mm'],previous['y_mm']),previous['layout_yaw_deg'],dimensions,wheel)
                             for wheel in ('FL','FR','BL','BR')}
                    wheel=row.get('pivot_wheel') or min(anchors,key=lambda name:math.dist(center,anchors[name]))
                    if wheel not in anchors or math.dist(center,anchors[wheel])>.5:
                        raise ValueError('旋转圆心不在指定麦轮轮心位置')
                    self.pivots[i]['wheel']=wheel
        self.control = dict(DEFAULT_CONTROL, **program.get('control', {}))
        if set(self.control) != set(DEFAULT_CONTROL):
            raise ValueError('未知坐标控制参数')
        for key, value in self.control.items():
            self.control[key] = nav.finite_number(value, key)
            upper = 2000 if key=='turn_lead_mm' else 50 if key.startswith('kp') else 100 if key.endswith('min') else 3000
            if not (0 < self.control[key] <= upper if key=='turn_lead_mm' else 0 <= self.control[key] <= upper):
                raise ValueError('坐标控制参数超出范围：'+key)
        self.ends, total = [0.0], 0.0
        for a, b in zip(self.points, self.points[1:]):
            length = math.dist((a['x_mm'], a['y_mm']), (b['x_mm'], b['y_mm']))
            if length <= nav.EPS and len(self.points) != 2 and not (a['kind'] in ('START','STOP') and b['kind']=='STOP'):
                raise ValueError('关键坐标不可重合；旋转必须显式动作')
            if len(self.ends) in self.pivots:
                arc=self.pivots[len(self.ends)]
                length=arc['radius']*abs(arc['angle'])
            total += length
            self.ends.append(total)
        self.length, self.progress, self.cross_track = total, 0.0, 0.0
        self.index = 1
        self.pieces = rows[1:]  # 公共模拟器的段统计接口。
        self.wheel_velocity_mm_s = (0.0,)*4
        self._rpm_remainder = [0.0]*4
        self.stop_window=StopWindow()
        self.intermediate_stop_window=StopWindow()
        self._pivot_index=None;self._pivot_rate=0.0
        self._pivot_recover=False
        self._heading_index=None;self._exit_heading=False
        self._previous_pose_yaw=None;self._measured_yaw_rate=0.0
        self._last_motion=((0,0),0);self._command_velocity=(0.0,0.0)

    def hold(self):
        self._pivot_rate = 0.0
        self._command_velocity = (0.0, 0.0)
        self._last_motion = ((0.0, 0.0), 0.0)
        self._previous_pose_yaw = None
        self._measured_yaw_rate = 0.0
        self.wheel_velocity_mm_s = (0.0,)*4
        self._rpm_remainder = [0.0]*4
        self.stop_window = StopWindow()
        self.intermediate_stop_window = StopWindow()

    def _reference(self, index, yaw=None):
        p = self.points[index]
        return dict(x_mm=p['x_mm'], y_mm=p['y_mm'],
                    field_yaw_deg=p['field_yaw_deg'] if yaw is None else yaw,
                    s_mm=self.ends[index], segment_type='STOP' if p['kind']=='STOP' else 'PIVOT' if index in self.pivots else 'COORDINATE',
                    segment_index=max(1, index), segment_count=len(self.points)-1)

    def reference_at(self, station):
        if station <= nav.EPS:
            return self._reference(0)
        if station >= self.length-nav.EPS:
            return self._reference(len(self.points)-1)
        return self._reference(self.index)

    def final_reference(self):
        # 零距离回库/原地对齐的首末坐标相同，但目标航向可以不同。
        return self._reference(len(self.points)-1)

    def command(self, pose, speed_limit, yaw_rate_limit):
        # 保留公共Tracker调用签名；坐标模式限幅只取底盘参数快照，不用旧250/120帽。
        x, y, yaw = (nav.finite_number(v, '实际坐标') for v in pose)
        if self._previous_pose_yaw is not None:
            self._measured_yaw_rate=wrap(yaw-self._previous_pose_yaw)*SEND_HZ
        self._previous_pose_yaw=yaw
        target = self.points[self.index]
        distance = math.hypot(target['x_mm']-x, target['y_mm']-y)
        # 只推进下一个点，绝不跳到自交路径的后支；PASS不检查停稳。
        from pivot_turns import PIVOT_POSITION_GATE_MM,PIVOT_YAW_GATE_DEG,WHEEL_PIVOT_JOIN_MM_S
        pivot=self.index in self.pivots
        gate=min(target['pass_mm'],PIVOT_POSITION_GATE_MM) if pivot else target['pass_mm']
        yaw_gate=PIVOT_YAW_GATE_DEG if pivot or self.index+1 in self.pivots or target.get('finish_heading_before_pass') else 30
        arrived=distance<=gate and abs(wrap(target['field_yaw_deg']-yaw))<yaw_gate
        intermediate_stop = target['kind']=='STOP' and self.index < len(self.points)-1
        stable = (distance < 1 and abs(wrap(target['field_yaw_deg']-yaw)) < 1
                  and math.hypot(*self._command_velocity)<=1 and abs(self._measured_yaw_rate)<=1)
        stopped = self.intermediate_stop_window.update((x,y,yaw),intermediate_stop and stable,1/SEND_HZ)>=200
        if target['kind']=='PASS' and arrived or stopped:
            self.index += 1
            self.intermediate_stop_window=StopWindow()
            target = self.points[self.index]
            distance = math.hypot(target['x_mm']-x, target['y_mm']-y)
        if self.index in self.pivots:
            return self._pivot_command((x,y,yaw),distance)
        self._pivot_index=None;self._pivot_rate=0.0
        a, b = self.points[self.index-1], target
        ux, uy = b['x_mm']-a['x_mm'], b['y_mm']-a['y_mm']
        span = math.hypot(ux, uy)
        projection = 0 if span <= nav.EPS else max(0, min(span, ((x-a['x_mm'])*ux+(y-a['y_mm'])*uy)/span))
        self.progress = max(self.progress, self.ends[self.index-1]+projection)
        # 转角提前换点允许切内；偏差计算包含相邻骨架，运行安全另查完整车体扫掠。
        distances = []
        for i in range(max(0, self.index-2), self.index):
            p, q = self.points[i:i+2]
            dx, dy = q['x_mm']-p['x_mm'], q['y_mm']-p['y_mm']
            norm = dx*dx+dy*dy
            t = 0 if norm <= nav.EPS**2 else max(0, min(1, ((x-p['x_mm'])*dx+(y-p['y_mm'])*dy)/norm))
            distances.append(math.hypot(x-p['x_mm']-t*dx, y-p['y_mm']-t*dy))
        self.cross_track = min(distances)
        # 长直线沿行驶车头；接近关键坐标时开始纠正出口车头，路径由闭环响应产生。
        if self._heading_index!=self.index:
            self._heading_index=self.index;self._exit_heading=False
        if distance<=target.get('turn_lead_mm',self.control['turn_lead_mm']):
            self._exit_heading=True
        desired_yaw = target['field_yaw_deg'] if self._exit_heading else target['travel_yaw_deg']
        ref = self._reference(self.index, yaw+wrap(desired_yaw-yaw))
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        ex, ey = target['x_mm']-x, target['y_mm']-y
        body_x, body_y = c*ex+s*ey, -s*ex+c*ey
        # 数学车体轴body_x为前、body_y为右；底盘KPX作用左右，KPY作用前后。
        u = numerical_limit(self.control['kpy']*body_x, self.control['xyvmax'], self.control['xyvmin'])
        v = numerical_limit(self.control['kpx']*body_y, self.control['xyvmax'], self.control['xyvmin'])
        wheel_straight=(bool(self.pivots.get(self.index+1,{}).get('wheel')) or
                        bool(self.pivots.get(self.index-1,{}).get('wheel')))
        if wheel_straight and span>nav.EPS:
            # Finite-distance braking removes the P-controller's slow approach tail.
            # Only the two straight legs adjacent to a wheel pivot use this profile.
            tx,ty=ux/span,uy/span
            forward,right=c*tx+s*ty,-s*tx+c*ty
            gain=self.control['kpy']*forward**2+self.control['kpx']*right**2
            accel=600.0*min(1.0,gain/2.3)
            remaining=ex*tx+ey*ty
            # 入弯PASS以非零速度衔接，STOP仍按剩余距离制动至零。
            next_arc=self.pivots.get(self.index+1,{})
            join=min(WHEEL_PIVOT_JOIN_MM_S,self.control['xyvmax'],self.control['zvmax']*next_arc.get('radius',0)/ROTATION_LEVER_MM) if target['kind']=='PASS' and next_arc.get('wheel') and self.control['kpz']>0 and accel>0 else 0.0
            desired=math.copysign(min(self.control['xyvmax'],math.sqrt(join*join+2*.7*accel*abs(remaining))),remaining)
            if target['kind']=='STOP' and abs(remaining)<20 or remaining<=0:
                desired=numerical_limit(gain*remaining,self.control['xyvmax'],self.control['xyvmin'])
            previous=sum(a*b for a,b in zip(self._command_velocity,(tx,ty)))
            desired=max(previous-accel/SEND_HZ,min(previous+accel/SEND_HZ,desired)) if accel else 0.0
            along=u*forward+v*right
            u,v=u+(desired-along)*forward,v+(desired-along)*right
            magnitude=max(abs(u),abs(v))
            scale=min(1.0,self.control['xyvmax']/magnitude) if magnitude else 1.0
            u,v=u*scale,v*scale
        # 大角度纠偏时仍平移；1600上限的20%仍有320mm/s，会在转头前冲入窄道。
        if abs(wrap(desired_yaw-yaw)) > 30:
            u, v = u*.05, v*.05
        # chassis_move的Z控制量也是轮缘线速度，先限幅，再通过臂长转deg/s。
        r = numerical_limit(self.control['kpz']*wrap(desired_yaw-yaw), self.control['zvmax'], self.control['zvmin'])
        omega = math.degrees(r/ROTATION_LEVER_MM)
        if target['kind']=='STOP' and distance <= .5 and abs(wrap(desired_yaw-yaw)) <= .3:
            # 与实机同一严格停车带；不用20ms补角把定位噪声放大为转动脉冲。
            u, v, omega = 0.0, 0.0, 0.0
        return self._wheel_output(ref,u,v,math.radians(omega)*ROTATION_LEVER_MM,c,s)

    def _pivot_command(self,pose,gap):
        from pivot_turns import rotate,PIVOT_ACCEL_DEG_S2,WHEEL_PIVOT_JOIN_MM_S
        x,y,yaw=pose;arc=self.pivots[self.index];center=arc['center'];radius=arc['radius']
        start,end=self.points[self.index-1],self.points[self.index]
        total=arc['angle'];direction=1 if total>0 else -1
        phase=direction*math.radians(wrap(yaw-start['field_yaw_deg']))
        phase=max(0,min(abs(total),phase))
        offset=rotate((start['x_mm']-center[0],start['y_mm']-center[1]),direction*phase)
        rx,ry=center[0]+offset[0],center[1]+offset[1]
        position_phase=direction*wrap(math.degrees(math.atan2(y-center[1],x-center[0])-
                         math.atan2(start['y_mm']-center[1],start['x_mm']-center[0])))
        fraction=max(0,min(abs(total),math.radians(position_phase)))
        nearest=rotate((start['x_mm']-center[0],start['y_mm']-center[1]),direction*fraction)
        self.cross_track=max(math.hypot(x-rx,y-ry),math.hypot(x-center[0]-nearest[0],y-center[1]-nearest[1]))
        self.progress=max(self.progress,self.ends[self.index-1]+radius*fraction)
        angle=wrap(end['field_yaw_deg']-yaw)
        if self._pivot_index!=self.index:
            self._pivot_recover=False
        if arc.get('wheel') and (direction*angle<=0 or abs(angle)<=1 and gap>min(end['pass_mm'],2)):
            self._pivot_recover=True
        requested=numerical_limit(self.control['kpz']*angle,self.control['zvmax'],self.control['zvmin'])
        if arc.get('wheel') and self.control['kpz']>0:
            join=min(WHEEL_PIVOT_JOIN_MM_S,self.control['xyvmax'],self.control['zvmax']*radius/ROTATION_LEVER_MM)/radius*ROTATION_LEVER_MM if end['kind']=='PASS' else 0.0
            # 剩余转角还需容纳响应/定位滞后期间的转动；不能等越过1deg门才减速。
            # 保留非零衔接速度，解 angle = (omega²-join²)/(2a) + 0.32*omega。
            accel=.7*math.radians(PIVOT_ACCEL_DEG_S2)*min(1.0,self.control['kpz']/9.0)
            lag=accel*.32
            braking=(math.sqrt(lag*lag+(join/ROTATION_LEVER_MM)**2+2*accel*abs(math.radians(angle)))-lag)*ROTATION_LEVER_MM
            requested=math.copysign(min(self.control['zvmax'],braking),angle)
            if self._pivot_recover:
                requested=numerical_limit(self.control['kpz']*(angle-.12*self._measured_yaw_rate),
                                          self.control['zvmax'],self.control['zvmin'])
        if self._pivot_index!=self.index:
            self._pivot_index=self.index;self._pivot_rate=0.0
            if arc.get('wheel') and self.control['kpz']>0:
                inherited=(-offset[1]*self._command_velocity[0]+offset[0]*self._command_velocity[1])/radius**2*ROTATION_LEVER_MM
                self._pivot_rate=math.copysign(min(self.control['zvmax'],max(0.0,inherited*direction)),direction)
        increment=math.radians(PIVOT_ACCEL_DEG_S2)*ROTATION_LEVER_MM/SEND_HZ
        r=max(self._pivot_rate-increment,min(self._pivot_rate+increment,requested))
        c,s=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
        ex,ey=rx-x,ry-y
        correction_u=numerical_limit(self.control['kpy']*(c*ex+s*ey),self.control['xyvmax'],self.control['xyvmin'])
        correction_v=numerical_limit(self.control['kpx']*(-s*ex+c*ey),self.control['xyvmax'],self.control['xyvmin'])
        omega=r/ROTATION_LEVER_MM
        fx,fy=-omega*(ry-center[1]),omega*(rx-center[0])
        u,v=c*fx+s*fy+correction_u,-s*fx+c*fy+correction_v
        magnitude=max(abs(u),abs(v))
        scale=min(1.0,self.control['xyvmax']/magnitude) if magnitude else 1.0
        u,v,r=u*scale,v*scale,r*scale
        self._pivot_rate=r
        if end['kind']=='STOP' and gap<=.5 and abs(angle)<=.3:
            u=v=r=0.0;self._pivot_rate=0.0
        ref=self._reference(self.index)
        ref.update(x_mm=rx,y_mm=ry,field_yaw_deg=start['field_yaw_deg']+math.degrees(direction*phase),
                   pivot_center_mm=center,pivot_radius_mm=radius,pivot_recovery=self._pivot_recover)
        if arc.get('wheel'):ref['pivot_wheel']=arc['wheel']
        return self._wheel_output(ref,u,v,r,c,s)

    def _wheel_output(self,ref,u,v,r,c,s):
        # Same integer wheel mixing for ordinary coordinates and fixed-center turns.
        wheels = (u-v-r, u+v+r, u+v-r, u-v+r)
        scale = min(1.0, MAX_WHEEL_RPM/(RPM_PER_MM_S*max(map(abs,wheels)))) if any(wheels) else 1.0
        self._command_velocity=(scale*(c*u-s*v),scale*(s*u+c*v))
        quantized = []
        for i, w in enumerate(wheels):
            rpm = w*scale*RPM_PER_MM_S
            if rpm == 0 or abs(rpm) >= MAX_WHEEL_RPM or rpm*self._rpm_remainder[i] < 0:
                self._rpm_remainder[i] = 0.0
            total = rpm+self._rpm_remainder[i]
            integer = int(total)
            self._rpm_remainder[i] = total-integer
            quantized.append(integer/RPM_PER_MM_S)
        wheels = tuple(quantized)
        self.wheel_velocity_mm_s = wheels
        u = sum(wheels)/4
        v = (-wheels[0]+wheels[1]+wheels[2]-wheels[3])/4
        r = (-wheels[0]+wheels[1]-wheels[2]+wheels[3])/4
        self._last_motion=((c*u-s*v,s*u+c*v),math.degrees(r/ROTATION_LEVER_MM))
        return ref,*self._last_motion


def _goal_headings(points, start_yaw, goal_yaw):
    """四类车头的动态规划同时考虑离站、途中转向及到站角，允许短侧移接头。"""
    states = {float(start_yaw) % 360: (0.0, [])}
    for index, (a, b) in enumerate(zip(points, points[1:])):
        distance = math.dist(a, b)
        tangent = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])) % 360
        choices = {tangent, (tangent+180) % 360, float(start_yaw) % 360}
        if goal_yaw is not None:
            choices.add(float(goal_yaw) % 360)
        if len(points)>2 and index==0 and distance<=300:
            choices = {float(start_yaw) % 360}
        next_states = {}
        for heading in choices:
            lateral = abs(math.sin(math.radians(tangent-heading)))*distance
            if lateral>500+nav.EPS:
                continue
            # 长侧移比倒行更贵；终点车头在末段短接头内可保持不再转回。
            reverse = max(0, -math.cos(math.radians(tangent-heading)))*distance
            options = [(score+abs(wrap(heading-previous))+.02*lateral+.001*reverse,
                        sequence+[heading]) for previous,(score,sequence) in states.items()]
            if options:
                next_states[heading] = min(options, key=lambda item: item[0])
        if not next_states:
            raise ValueError('作业航向组合没有满足横移限制的候选')
        states = next_states
    if not states:
        return []
    return min(states.items(), key=lambda item: item[1][0]+(
        abs(wrap(goal_yaw-item[0])) if goal_yaw is not None else 0))[1][1]


def build_program(points, start_yaw, *, goal_yaw=None, pass_mm=100, mode='MIN_TURN', control=None, constraints=None, departure_mm=0):
    points = [nav.point2(p, '骨架点') for p in points]
    if departure_mm and len(points)>1 and math.dist(points[0],points[1])>departure_mm*2:
        # 同一直线插入离站过渡坐标，先保持原航向短移，再转为纵向行驶。
        a,b=points[:2]
        t=departure_mm/math.dist(a,b)
        points.insert(1,(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
    headings, current = [], float(start_yaw)
    for a, b in zip(points, points[1:]):
        tangent = current if math.dist(a,b) <= nav.EPS else math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
        if mode == 'FIXED' or len(points)>2 and not headings and math.dist(a,b)<=300:
            h = start_yaw
        elif mode == 'FORWARD':
            h = current+wrap(tangent-current)
        elif mode == 'REVERSE':
            h = current+wrap(tangent+180-current)
        else:
            h = min((current+wrap(tangent-current), current+wrap(tangent+180-current)),
                    key=lambda h: abs(h-current))
        headings.append(h)
        current = h
    if mode == 'GOAL_AWARE':
        headings = _goal_headings(points, start_yaw, goal_yaw)
    rows = [dict(x_mm=points[0][0], y_mm=points[0][1], layout_yaw_deg=start_yaw,
                 kind='START', pass_mm=0)]
    for i, point in enumerate(points[1:], 1):
        last = i == len(points)-1
        yaw = (headings[-1] if goal_yaw is None else goal_yaw) if last else headings[i]
        yaw = rows[-1]['layout_yaw_deg']+wrap(yaw-rows[-1]['layout_yaw_deg'])
        radius = 0 if last else min(pass_mm, math.dist(points[i-1], point)*.25,
                                   math.dist(point, points[i+1])*.25)
        rows.append(dict(x_mm=point[0], y_mm=point[1], layout_yaw_deg=yaw,
                         travel_yaw_deg=headings[i-1], kind='STOP' if last else 'PASS', pass_mm=radius))
    program = dict(schema_version=2, kind=KIND, frame_id='LAYOUT_MM', hardware_ready=False, control_source='CHASSIS_MOVE',
                   waypoints=rows, control=dict(DEFAULT_CONTROL, **(control or {})))
    if constraints and constraints.get('turn_mode')=='STOP_TURN':
        stopped_rows=[rows[0]]
        for row in rows[1:]:
            previous=stopped_rows[-1]
            travel=row.get('travel_yaw_deg',row['layout_yaw_deg'])
            if abs(wrap(travel-previous['layout_yaw_deg']))>.01:
                stopped_rows.append(dict(x_mm=previous['x_mm'],y_mm=previous['y_mm'],layout_yaw_deg=travel,
                    travel_yaw_deg=travel,kind='STOP',pass_mm=0))
            stopped_rows.append(dict(x_mm=row['x_mm'],y_mm=row['y_mm'],layout_yaw_deg=travel,
                travel_yaw_deg=travel,kind='STOP',pass_mm=0))
            if abs(wrap(row['layout_yaw_deg']-travel))>.01:
                stopped_rows.append(dict(x_mm=row['x_mm'],y_mm=row['y_mm'],layout_yaw_deg=row['layout_yaw_deg'],
                    travel_yaw_deg=row['layout_yaw_deg'],kind='STOP',pass_mm=0))
        program['waypoints']=stopped_rows
    if constraints:
        program['constraints'] = copy.deepcopy(constraints)
    tracker = CoordinateTracker(program)
    program.update(start=tracker.reference_at(0), goal=tracker.final_reference(),
                   length_mm=tracker.length, reference_frame_id='FIELD_MM')
    return program


def replay(program, scene, cancelled=lambda: False, sweep_cache=None):
    """与运行同一控制器/50Hz步长，完整矩形及相邻姿态扫掠；不按端点安全放行。"""
    from core import SEND_HZ, field_to_layout
    from trajectory import _motion_mode
    tracker = CoordinateTracker(program)
    first = tracker.reference_at(0)
    pose = first['x_mm'], first['y_mm'], first['field_yaw_deg']
    to_layout = lambda p: (*field_to_layout(*p[:2]), -90-p[2])
    why = scene.pose_reason(*to_layout(pose))
    if why:
        raise ValueError('坐标起点：'+why)
    constraints = program.get('constraints', {})
    lateral_limit = constraints.get('strafe_run_limit_mm')
    if lateral_limit is not None:
        lateral_limit = nav.finite_number(lateral_limit, '连续横移上限')
        if lateral_limit < 0:
            raise ValueError('连续横移上限不可为负')
    operations = nav.allowed_area(constraints['strafe_polygons'], scene.bounds) if constraints.get('strafe_polygons') else None
    samples, settled, traveled, lateral_run = [], 0, 0.0, 0.0
    best_progress, best_error, last_progress = 0.0, math.inf, 0
    for i in range(7500):
        if i % 16 == 0 and cancelled():
            raise ValueError('关键坐标预演已取消')
        ref, velocity, omega = tracker.command(pose, None, None)
        candidate = pose[0]+velocity[0]/SEND_HZ, pose[1]+velocity[1]/SEND_HZ, pose[2]+omega/SEND_HZ
        key = (pose, candidate)
        if sweep_cache is not None and key in sweep_cache:
            why = sweep_cache[key]
        else:
            why = scene.moving_pose_reason(to_layout(pose), to_layout(candidate))
            if sweep_cache is not None and len(sweep_cache)<65536:
                sweep_cache[key] = why
        if why or tracker.cross_track > 75:
            raise ValueError('关键坐标控制器实际扫掠：'+(why or '偏离骨架超过75mm'))
        delta = math.degrees(math.atan2(velocity[1],velocity[0]))-pose[2]
        is_lateral = math.hypot(*velocity)>1e-8 and abs(math.cos(math.radians(delta))) < math.sin(math.radians(15))
        if is_lateral:
            in_operation = operations is not None and operations.covers(scene.pose_polygon(*to_layout(pose)))
            if not in_operation:
                lateral_run += math.hypot(*velocity)/SEND_HZ
                if constraints.get('allow_strafe') is False or lateral_limit is not None and lateral_run > lateral_limit+nav.EPS:
                    raise ValueError('坐标控制预演超过区外连续横移限制')
        else:
            lateral_run = 0.0
        traveled += math.dist(pose[:2], candidate[:2])
        pose = candidate
        goal = tracker.final_reference()
        local = tracker.points[tracker.index]
        if i==0 or last_index!=tracker.index:
            best_error=math.inf;last_progress=i
        last_index=tracker.index
        error = math.dist(pose[:2],(local['x_mm'],local['y_mm']))+abs(wrap(local['field_yaw_deg']-pose[2]))
        if tracker.progress>best_progress+1 or error<best_error-.1:
            best_progress,best_error,last_progress=tracker.progress,error,i
        done = (tracker.index==len(tracker.points)-1 and ref['segment_type']=='STOP' and math.dist(pose[:2], (goal['x_mm'],goal['y_mm'])) < 1 and
                abs(wrap(goal['field_yaw_deg']-pose[2])) < 1 and math.hypot(*velocity) <= 1 and abs(omega) <= 1)
        settled = tracker.stop_window.update(pose,done,1/SEND_HZ)//20
        samples.append(dict(x_mm=pose[0], y_mm=pose[1], field_yaw_deg=pose[2], s_mm=traveled,
                            segment_type='STOP' if settled >= 10 else 'PIVOT' if 'pivot_center_mm' in ref else 'COORDINATE',
                            tangent_yaw_deg=math.degrees(math.atan2(velocity[1],velocity[0])) if math.hypot(*velocity) else pose[2]))
        samples[-1]['motion_mode'] = _motion_mode(pose[2], samples[-1]['tangent_yaw_deg'])
        if 'pivot_center_mm' in ref:
            samples[-1].update(pivot_center_mm=ref['pivot_center_mm'],pivot_radius_mm=ref['pivot_radius_mm'],segment_index=tracker.index)
            if 'pivot_wheel' in ref:samples[-1]['pivot_wheel']=ref['pivot_wheel']
        if settled >= 10:
            return samples, (i+1)/SEND_HZ
        if i-last_progress>3*SEND_HZ:
            raise ValueError('关键坐标控制预演持续3秒无进展')
    raise ValueError('关键坐标控制预演超时')


def _cached_replay(program, scene, cancelled, sweeps, cache):
    if cancelled():
        raise ValueError('关键坐标预演已取消')
    if cache is None:
        return replay(program,scene,cancelled,sweeps)
    identity = json.dumps(program,sort_keys=True,separators=(',',':'))
    saved = cache.get(identity)
    if saved is not None:
        if isinstance(saved,str):
            raise ValueError(saved)
        return saved
    try:
        value = replay(program,scene,cancelled,sweeps)
    except ValueError as exc:
        if cancelled():
            raise
        if len(cache)<128:
            cache[identity] = str(exc)
        raise
    # 以实际轨迹样本计预算，避免扩展骨架后1024条密集轨迹占满Nano内存。
    while cache and (len(cache)>=128 or
          sum(len(v[0]) for v in cache.values() if isinstance(v,tuple))+len(value[0])>30000):
        cache.pop(next(iter(cache)))
    if len(value[0])<=30000:
        cache[identity]=value
    return value


def _seed_route(start, goal, scene, nodes, start_yaw, *, goal_yaw=None, cancelled=lambda: False, constraints=None, graph_cache=None, chassis_control=None, replay_cache=None, first_safe=False, incumbent=None):
    """A*保留无圆弧骨架候选；每个候选用坐标控制实际扫掠验证。"""
    from competition_simulation import _lane_graph
    from core import field_to_layout
    for p, h in ((start, start_yaw), (goal, start_yaw if goal_yaw is None else goal_yaw)):
        why = scene.pose_reason(*p, h)
        if why:
            raise ValueError('关键坐标起点/停靠点姿态非法：'+why)
    failures, fallback_results, candidate_count, limit_hit = [], [], 0, False
    cache = {} if graph_cache is None else graph_cache.setdefault(('coordinate_scene', scene), {})
    sweeps = cache.setdefault('coordinate_sweeps', {})
    for refined, extended in ((False,False),(False,True),(True,True)):
        links = _lane_graph(start, goal, scene, nodes, cancelled, refined, cache)
        # 切线图连通也可能缺少麦轮侧移连接：横向停靠位须以纵向车头进入。
        # 每种姿态使用独立几何缓存，候选仍须通过实际控制器完整扫掠。
        from mecanum_planner import FixedHeadingScene
        links = {p:list(targets) for p, targets in links.items()}
        connected, pending = {tuple(start)}, [tuple(start)]
        while pending:
            if cancelled():
                raise ValueError('关键坐标规划已取消')
            for neighbor in links.get(pending.pop(), ()):
                if neighbor not in connected:
                    connected.add(neighbor); pending.append(neighbor)
        disconnected = tuple(goal) not in connected
        # 先搜索原切线/当前车头图，再补正交短侧移，最后细化障碍车道。
        # 每轮独立预算，增加麦轮连接不能挤掉原图的有效候选。
        headings = (start_yaw,start_yaw+90) if extended else (start_yaw,) if disconnected else ()
        for heading in headings:
            extra = _lane_graph(start, goal, FixedHeadingScene(scene, heading), nodes, cancelled, refined,
                                cache.setdefault(('body_heading', heading), {}))
            for a, targets in extra.items():
                for b in targets:
                    if (math.dist(a,b)<=400 or disconnected and (a[0]==b[0] or a[1]==b[1])) and b not in links.get(a, ()):
                        links.setdefault(a, []).append(b)
        diagonal = any(a[0]!=b[0] and a[1]!=b[1] for a, targets in links.items() for b in targets)
        heuristic = (lambda p: math.dist(p,goal)) if diagonal else (lambda p: sum(abs(p[k]-goal[k]) for k in (0,1)))
        frontier, counter, visits = [(heuristic(start), 0, 0.0, [tuple(start)])], itertools.count(1), {}
        seen = {(tuple(start),)}
        for expanded in range(1,2001):
            if cancelled():
                raise ValueError('关键坐标规划已取消')
            if not frontier:
                break
            _score, _order, cost, route = heapq.heappop(frontier)
            if route[-1] == tuple(goal):
                if len(route) == 1:
                    route = route+[tuple(goal)]
                candidate_count += 1
                for mode in ('MIN_TURN', 'FORWARD', 'FIXED'):
                    for radius, lead, departure in ((100,200,0),(100,60,0),(60,60,0),(20,40,0),
                                                    (5,40,0),(5,200,0),(5,10,0),(5,40,80),(5,60,120)):
                        program = build_program(route, start_yaw, goal_yaw=goal_yaw, pass_mm=radius, mode=mode,
                                                control=dict(chassis_control or {},turn_lead_mm=lead), constraints=constraints,
                                                departure_mm=departure)
                        try:
                            samples, elapsed = _cached_replay(program, scene, cancelled, sweeps, replay_cache)
                            result = dict(ok=True, execution_safe=True, axis_matched=True,
                                skeleton_points=list(route),
                                search=dict(refined=refined,extended=extended,expanded=expanded,
                                            candidates=candidate_count,skeleton_cost_mm=cost,turn_penalty_mm=120),
                                optimality=dict(proven=False,policy='FIRST_SAFE_NON_FIXED_OR_BOUNDED_FIXED_FALLBACK',
                                                objective='skeleton length + 120mm per bend; replay safety',
                                                reason='bounded search, prefix pruning, finite graph and first safe replay'),
                                points=[(w['x_mm'],w['y_mm']) for w in program['waypoints']],
                                planner='COORDINATE_'+mode, waypoint_program=program, arcs=[],
                                arc_fallbacks=[], arc_attempts=[], smoothed_primitives=[],
                                smoothed_points=[field_to_layout(p['x_mm'],p['y_mm']) for p in samples],
                                smoothing_status='COORDINATE_NO_ARC', smoothing_model_safe=True,
                                trajectory=samples, trajectory_safe=True, trajectory_continuous=True,
                                trajectory_status='COORDINATE_REPLAY', trajectory_length_mm=samples[-1]['s_mm'],
                                predicted_tracking_s=elapsed, length=program['length_mm'], smoothed_length=samples[-1]['s_mm'],
                                start_heading_deg=start_yaw, goal_heading_deg=goal_yaw, turn_count=0, steps=[],
                                segments=[], corners=[], hardware_ready=False,
                                body_clearance=None, min_clearance=None, max_strafe_run_mm=0,
                                simulation_only=True, reason='关键坐标闭环实际矩形扫掠通过')
                            from mecanum_planner import motion_metrics
                            metrics=motion_metrics(result)
                            result['motion_metrics']=metrics
                            result['max_strafe_run_mm']=metrics['longest_strafe_mm']
                            if metrics['longest_strafe_mm']>500:
                                raise ValueError('坐标候选存在超过500mm长横移，需要调整车头或绕行')
                            if incumbent is not None:
                                incumbent(result)
                            if mode != 'FIXED' or first_safe:
                                return result
                            fallback_results.append(result)
                            break
                        except ValueError as exc:
                            if cancelled():
                                raise
                            failures.append(str(exc))
                if candidate_count >= 4 and fallback_results:
                    from mecanum_planner import motion_metrics
                    return min(fallback_results, key=lambda r: r['predicted_tracking_s']+
                               (motion_metrics(r)['equivalent_cost_mm']-r['trajectory_length_mm'])/
                               max(1.0,r['waypoint_program']['control']['xyvmax']))
                continue
            state = tuple(route[-3:])
            visits[state] = visits.get(state, 0)+1
            if visits[state] > 2:
                continue
            for target in links.get(route[-1], ()):
                if target in route:
                    continue
                trial = route+[target]
                turn = False
                if len(route)>1:
                    a, b = route[-2:]
                    cross = (b[0]-a[0])*(target[1]-b[1])-(b[1]-a[1])*(target[0]-b[0])
                    turn = abs(cross)>nav.EPS
                    if not turn:
                        if (b[0]-a[0])*(target[0]-b[0])+(b[1]-a[1])*(target[1]-b[1]) < 0:
                            continue
                        trial = route[:-1]+[target]
                new = cost+math.dist(route[-1],target)+(120 if turn else 0)
                key = tuple(trial)
                if key in seen:
                    continue
                seen.add(key)
                heapq.heappush(frontier,(new+heuristic(target),next(counter),new,trial))
        limit_hit |= bool(frontier)
    if fallback_results:
        return min(fallback_results, key=lambda r: r['predicted_tracking_s'])
    raise ValueError('关键坐标无安全候选；fallback：'+('; '.join(dict.fromkeys(failures))[-400:] or
                     ('搜索预算耗尽，不能判定物理无路' if limit_hit else '车道不连通')))


CONTROL_SETTINGS = ((100,200,0),(100,60,0),(60,60,0),(20,40,0),(5,40,0),
                    (5,200,0),(5,10,0),(5,40,80),(5,60,120))
MAX_OPTIMIZATION_SKELETONS = 12


class OptimizationDeadline(TimeoutError):
    pass


def _coordinate_graph(start, goal, scene, nodes, yaw, refined, extended, cache, cancelled):
    from competition_simulation import _lane_graph
    from mecanum_planner import FixedHeadingScene
    links = {a: set(bs) for a, bs in _lane_graph(start, goal, scene, nodes, cancelled, refined, cache).items()}
    connected, pending = {tuple(start)}, [tuple(start)]
    while pending:
        if cancelled():
            raise ValueError('关键坐标规划已取消')
        for b in links.get(pending.pop(), ()):
            if b not in connected:
                connected.add(b); pending.append(b)
    disconnected = tuple(goal) not in connected
    for heading in ((yaw,yaw+90) if extended else (yaw,) if disconnected else ()):
        extra = _lane_graph(start, goal, FixedHeadingScene(scene, heading), nodes, cancelled, refined,
                            cache.setdefault(('body_heading', heading), {}))
        for a, bs in extra.items():
            for b in bs:
                if math.dist(a,b)<=400 or disconnected and (a[0]==b[0] or a[1]==b[1]):
                    links.setdefault(a, set()).add(b)
    return links


def _replay_result(program, samples, elapsed, route, cost, mode, start_yaw, goal_yaw, phase):
    from core import field_to_layout
    from mecanum_planner import motion_metrics
    result = dict(ok=True, execution_safe=True, axis_matched=True, skeleton_points=list(route),
        search=dict(refined=phase[0],extended=phase[1],skeleton_cost_mm=cost,turn_penalty_mm=120),
        points=[(w['x_mm'],w['y_mm']) for w in program['waypoints']], planner='COORDINATE_'+mode,
        waypoint_program=program, arcs=[], arc_fallbacks=[], arc_attempts=[], smoothed_primitives=[],
        smoothed_points=[field_to_layout(p['x_mm'],p['y_mm']) for p in samples],
        smoothing_status='COORDINATE_NO_ARC', smoothing_model_safe=True, trajectory=samples,
        trajectory_safe=True, trajectory_continuous=True, trajectory_status='COORDINATE_REPLAY',
        trajectory_length_mm=samples[-1]['s_mm'], predicted_tracking_s=elapsed,
        length=program['length_mm'], smoothed_length=samples[-1]['s_mm'],
        start_heading_deg=start_yaw, goal_heading_deg=goal_yaw, turn_count=0, steps=[], segments=[],
        corners=[], hardware_ready=False, body_clearance=None, min_clearance=None,
        simulation_only=True, reason='关键坐标闭环实际矩形扫掠通过')
    result['motion_metrics'] = motion_metrics(result)
    result['execution_cost']=execution_cost(program,samples,elapsed)
    result['max_strafe_run_mm'] = result['motion_metrics']['longest_strafe_mm']
    if result['max_strafe_run_mm']>500:
        raise ValueError('坐标候选存在超过500mm长横移，需要调整车头或绕行')
    return result


def execution_cost(program,samples,elapsed):
    # 预演首帧已积分一次，不能漏算START到首帧的转角；终点残差也不能奖励未收敛角。
    initial=-90-program['waypoints'][0]['layout_yaw_deg']
    final=-90-program['waypoints'][-1]['layout_yaw_deg']
    angles=[initial]+[p['field_yaw_deg'] for p in samples]
    turns=[wrap(b-a) for a,b in zip(angles,angles[1:])]
    signs=[math.copysign(1,v) for v in turns if abs(v)>.1]
    total=sum(map(abs,turns));reversals=sum(a!=b for a,b in zip(signs,signs[1:]))
    residual=abs(wrap(final-angles[-1]))
    return dict(modeled_time_s=elapsed,yaw_total_deg=total,yaw_residual_deg=residual,
        yaw_reversals=reversals,score_s=elapsed+.015*(total+residual)+.2*reversals)


def _interactive_route(start, goal, scene, nodes, yaw, goal_yaw, interrupted, constraints, graph_cache, control):
    """Find one fully replayed route for a click; keep the batch optimizer separate."""
    cache=graph_cache.setdefault(('coordinate_scene',scene),{})
    sweeps=cache.setdefault('coordinate_sweeps',{});replays={};tested=0
    local_deadline=time.monotonic()+.35
    def local_interrupted():
        if interrupted():
            return True
        if time.monotonic()>=local_deadline:
            raise OptimizationDeadline()
        return False
    routes=[[tuple(start),tuple(goal)]]
    if start[0]!=goal[0] and start[1]!=goal[1]:
        routes.extend(([tuple(start),(goal[0],start[1]),tuple(goal)],
                       [tuple(start),(start[0],goal[1]),tuple(goal)]))
    try:
        for route in routes:
            cost=sum(math.dist(a,b) for a,b in zip(route,route[1:]))+120*(len(route)-2)
            for mode in ('MIN_TURN','FORWARD','FIXED'):
                for radius,lead,departure in ((5,40,0),(5,10,0),(5,40,80)):
                    if local_interrupted():
                        raise ValueError('关键坐标规划已取消')
                    program=build_program(route,yaw,goal_yaw=goal_yaw,pass_mm=radius,mode=mode,
                        control=dict(control or {},turn_lead_mm=lead),constraints=constraints,departure_mm=departure)
                    tested+=1
                    try:
                        samples,elapsed=_cached_replay(program,scene,local_interrupted,sweeps,replays)
                        result=_replay_result(program,samples,elapsed,route,cost,mode,yaw,goal_yaw,(False,False))
                        return _interactive_result(result,tested,'DIRECT_OR_ONE_BEND')
                    except ValueError:
                        if interrupted():
                            raise
    except OptimizationDeadline:
        # A local attempt never consumes the rest of the global graph-search budget.
        if interrupted():
            raise ValueError('关键坐标规划已取消') from None
    result=_seed_route(start,goal,scene,nodes,yaw,goal_yaw=goal_yaw,cancelled=interrupted,
        constraints=constraints,graph_cache=graph_cache,chassis_control=control,replay_cache=replays,first_safe=True)
    return _interactive_result(result,tested,'LANE_SEARCH')


def _interactive_result(result,tested,source):
    result['search'].update(interactive=True,fast_attempts=tested,source=source)
    result['optimality']=dict(proven=False,policy='INTERACTIVE_FIRST_VERIFIED_SAFE',
        objective='quick feasible route with complete body/control replay',
        global_continuous_proven=False,global_time_proven=False,
        reason='interactive feasibility search; controller combinations not exhaustively compared')
    return result


def _plan_route(start, goal, scene, nodes, start_yaw, *, goal_yaw=None, cancelled=lambda: False,
               constraints=None, graph_cache=None, chassis_control=None, deadline=None, interactive=False,
               pivot_turns=True):
    """有限车道候选按模型用时与航向变化评分；允许略长的倒行路线。

    保留完整扫掠安全上界，五种车头策略和九组控制设置比较。
    绕轮扩展有独立预算，不宣称全局最优或绕轮组合穷举。
    """
    from coordinate_search import RankedRoutes
    graph_cache = {} if graph_cache is None else graph_cache
    cache = graph_cache.setdefault(('coordinate_scene', scene), {})
    sweeps = cache.setdefault('coordinate_sweeps', {})
    replays = {}
    seed_error = None
    def interrupted():
        if cancelled():
            return True
        if deadline is not None and time.monotonic()>=deadline:
            raise OptimizationDeadline()
        return False
    try:
        if interrupted():
            raise ValueError('关键坐标规划已取消')
    except OptimizationDeadline:
        raise ValueError('规划时间预算耗尽，尚未获得通过完整预演的安全路线') from None
    for label,p,h in (('起点',start,start_yaw),('目标停靠点',goal,start_yaw if goal_yaw is None else goal_yaw)):
        why=scene.pose_reason(*p,h)
        if why:
            raise ValueError(label+'整车姿态非法：'+why)
    if interactive:
        try:
            result=_interactive_route(start,goal,scene,nodes,start_yaw,goal_yaw,interrupted,
                                      constraints,graph_cache,chassis_control)
            if cancelled():
                raise ValueError('关键坐标规划已取消')
            return result
        except OptimizationDeadline:
            raise ValueError('规划时间预算耗尽，尚未获得通过完整预演的安全路线') from None
    best=None
    def key(result):
        if 'execution_cost' not in result:
            result['execution_cost']=execution_cost(result['waypoint_program'],result['trajectory'],result['predicted_tracking_s'])
        return (round(result['execution_cost']['score_s']*10),result['search']['skeleton_cost_mm'],
                result['predicted_tracking_s'],result['motion_metrics']['equivalent_cost_mm'])
    def retain_seed(result):
        nonlocal best
        if best is None or key(result)<key(best):
            best=result
    # 比赛到站姿态已知时，先比较直接/单弯的倒行与终点朝向策略。
    # 独立小预算防止车道图的首条可行小折返消耗全部在线时间；超时保留已验证结果。
    if goal_yaw is not None:
        cutoff=min(deadline if deadline is not None else math.inf,time.monotonic()+.4)
        def seed_interrupted():
            if interrupted():return True
            if time.monotonic()>=cutoff:raise OptimizationDeadline()
            return False
        routes=[[tuple(start),tuple(goal)]]
        if start[0]!=goal[0] and start[1]!=goal[1]:
            routes.extend(([tuple(start),(goal[0],start[1]),tuple(goal)],
                           [tuple(start),(start[0],goal[1]),tuple(goal)]))
        identities=set()
        try:
            for route in routes:
                cost=sum(math.dist(a,b) for a,b in zip(route,route[1:]))+120*(len(route)-2)
                for mode in ('GOAL_AWARE','REVERSE','MIN_TURN'):
                    program=build_program(route,start_yaw,goal_yaw=goal_yaw,pass_mm=5,mode=mode,
                        control=dict(chassis_control or {},turn_lead_mm=40),constraints=constraints)
                    identity=json.dumps(program,sort_keys=True,separators=(',',':'))
                    if identity in identities:continue
                    identities.add(identity)
                    try:
                        samples,elapsed=_cached_replay(program,scene,seed_interrupted,sweeps,replays)
                        result=_replay_result(program,samples,elapsed,route,cost,mode,start_yaw,goal_yaw,(False,False))
                        if pivot_turns:
                            from pivot_turns import select_turns
                            result=select_turns(result,scene,cancelled,cutoff)
                        retain_seed(result)
                    except ValueError:
                        if seed_interrupted():raise ValueError('关键坐标规划已取消')
        except OptimizationDeadline:
            pass
    try:
        seed = _seed_route(start,goal,scene,nodes,start_yaw,goal_yaw=goal_yaw,cancelled=interrupted,
                    constraints=constraints,graph_cache=graph_cache,chassis_control=chassis_control,replay_cache=replays,
                    incumbent=retain_seed)
        retain_seed(seed)
    except OptimizationDeadline:
        if best is None:
            raise ValueError('规划时间预算耗尽，尚未获得通过完整预演的安全路线') from None
    except ValueError as exc:
        if cancelled():
            raise
        seed_error = str(exc)
    baseline = None if best is None else dict(skeleton_cost_mm=best['search']['skeleton_cost_mm'],
                                              predicted_tracking_s=best['predicted_tracking_s'])
    if best is not None:
        key(best)
        baseline['execution_cost']=dict(best['execution_cost'])
    if best is not None and pivot_turns:
        from pivot_turns import select_turns
        best=select_turns(best,scene,cancelled,deadline)
    tested, safe, candidates, evaluated, phases = 0, 0, 0, set(), []
    timed_out = False
    for refined, extended in ((False,False),(False,True),(True,True)):
        search = None
        try:
            links = _coordinate_graph(start,goal,scene,nodes,start_yaw,refined,extended,cache,interrupted)
            search = RankedRoutes(links,start,goal,interrupted)
            # 几何排序用于有限候选枚举，不再用较短骨架剪掉模型更快的外侧倒行。
            upper_cost=max(search.initial_lower_bound+400,
                           best['search']['skeleton_cost_mm'] if best is not None else search.initial_lower_bound+400)
            while candidates<MAX_OPTIMIZATION_SKELETONS:
                found = search.next(upper_cost)
                if found is None:
                    break
                cost, route = found
                signature = tuple(route)
                if signature in evaluated:
                    continue
                evaluated.add(signature); candidates += 1
                programs = set()
                for mode in ('GOAL_AWARE','MIN_TURN','FORWARD','REVERSE','FIXED'):
                    for radius, lead, departure in CONTROL_SETTINGS:
                        if interrupted():
                            raise ValueError('关键坐标规划已取消')
                        program = build_program(route,start_yaw,goal_yaw=goal_yaw,pass_mm=radius,mode=mode,
                            control=dict(chassis_control or {},turn_lead_mm=lead),constraints=constraints,
                            departure_mm=departure)
                        identity = json.dumps(program,sort_keys=True,separators=(',',':'))
                        if identity in programs:
                            continue
                        programs.add(identity); tested += 1
                        try:
                            samples, elapsed = _cached_replay(program,scene,interrupted,sweeps,replays)
                            result = _replay_result(program,samples,elapsed,route,cost,mode,start_yaw,goal_yaw,
                                                     (refined,extended))
                            safe += 1
                            if best is None or key(result)<key(best):
                                candidate=result
                                if pivot_turns:
                                    from pivot_turns import select_turns
                                    candidate=select_turns(result,scene,cancelled,deadline)
                                if best is None or key(candidate)<key(best):
                                    best = candidate
                        except ValueError:
                            if interrupted():
                                raise
        except OptimizationDeadline:
            if cancelled():
                raise ValueError('关键坐标规划已取消') from None
            timed_out = True
            phases.append(dict(refined=refined,extended=extended,expanded=search.expanded if search else 0,
                candidates=search.yielded if search else 0,graph_lower_bound_mm=None,
                remaining_lower_bound_mm=None,complete=False,limit_hit=True,time_limit_hit=True))
            break
        upper = upper_cost
        complete = search.remaining_lower_bound>upper+nav.EPS or not search.frontier
        phases.append(dict(refined=refined,extended=extended,expanded=search.expanded,
            candidates=search.yielded,graph_lower_bound_mm=search.initial_lower_bound if math.isfinite(search.initial_lower_bound) else None,
            remaining_lower_bound_mm=search.remaining_lower_bound if math.isfinite(search.remaining_lower_bound) else None,
            complete=complete,limit_hit=not complete and (search.limit_hit or candidates>=MAX_OPTIMIZATION_SKELETONS)))
    if cancelled():
        raise ValueError('关键坐标规划已取消')
    if best is None:
        budget_hit = timed_out or any(p['limit_hit'] for p in phases)
        raise ValueError('关键坐标无安全候选；'+('搜索预算耗尽，不能判定物理无路' if budget_hit else
                        seed_error or '有限候选均未通过安全预演'))
    proven = not pivot_turns and not timed_out and len(phases)==3 and tuple(best['skeleton_points']) in evaluated and all(
        p['complete'] for p in phases)
    best['search'].update(expanded=sum(p['expanded'] for p in phases),candidates=candidates,
                          replay_tested=tested,replay_safe=safe,phases=phases,time_limit_hit=timed_out)
    key(best)
    best['optimality'] = dict(proven=proven,policy='VERIFIED_TIME_AND_YAW_COST',
        objective='100ms bands of modeled time + .015s/degree yaw and final residual + .2s/yaw reversal; geometry then time break ties',
        scope='three finite lane graphs within 400mm geometric cost slack, five heading policies x nine controller settings',
        cost_tolerance_mm=nav.EPS,
        global_continuous_proven=False,global_time_proven=False,baseline=baseline,
        reason='finite geometric slack and heading policies exhausted' if proven else
               'bounded heading and wheel-turn search; best verified incumbent retained')
    return best


def plan_route(start,goal,scene,nodes,start_yaw,*,goal_yaw=None,cancelled=lambda:False,
               constraints=None,graph_cache=None,chassis_control=None,deadline=None,interactive=False,
               pivot_turns=True,approach_turns=True,turn_mode=None):
    if turn_mode not in (None,'MOVING','WHEEL','STOP_TURN'):
        raise ValueError('未知转弯模式')
    if turn_mode is not None:
        pivot_turns=turn_mode=='WHEEL'
        approach_turns=turn_mode!='STOP_TURN'
        constraints=dict(constraints or {},turn_mode=turn_mode)
    # 从现有每段总预算中给提前转向复核留时间，仍只交付完整安全预演的结果。
    search_deadline=deadline
    if approach_turns and deadline is not None:
        search_deadline=deadline-min(.5,max(0,(deadline-time.monotonic())*.25))
    result=_plan_route(start,goal,scene,nodes,start_yaw,goal_yaw=goal_yaw,cancelled=cancelled,
        constraints=constraints,graph_cache=graph_cache,chassis_control=chassis_control,
        deadline=search_deadline,interactive=interactive,pivot_turns=pivot_turns)
    if pivot_turns and interactive:
        from pivot_turns import select_turns
        result=select_turns(result,scene,cancelled,deadline)
    if approach_turns:
        from approach_heading import anticipate_route
        result=anticipate_route(result,scene,cancelled,deadline)
    if cancelled():raise ValueError('关键坐标规划已取消')
    result['turn_mode']=turn_mode or 'WHEEL'
    result['waypoint_program']['turn_mode']=turn_mode or 'WHEEL'
    return result


def export_json(path, program):
    CoordinateTracker(program)
    Path(path).write_text(json.dumps(program, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
