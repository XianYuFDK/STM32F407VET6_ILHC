"""2027智能搬运初赛：静态赛场、连续路径与两批任务状态机，仅用于PC模拟。"""
import copy
import heapq
import itertools
import math
import queue
import re
from pathlib import Path

import core
import navigation_planner as nav
from arc_smoothing import smooth_90_corners
from trajectory import generate_trajectory

COLORS = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}
DEFAULT_CODE = '156+123+516+231'
RUN_LIMIT_S = 180.0
# 演示预设：中路圆柱迫使绕行，其余圆柱限制备用外侧车道；不是现场公布坐标。
DEMO_OBSTACLES = ((1200, 1700), (220, 700), (220, 1700), (700, 220))


def parse_task_code(text):
    """第二批暂存按同色第一层映射；第四组只决定第二批粗加工位置。"""
    text = str(text).strip()
    if not re.fullmatch(r'[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}', text):
        raise ValueError('任务码须为四组三位数，例如156+123+516+231')
    first, slots, second, rough2 = [tuple(map(int, p)) for p in text.split('+')]
    if len(set(first)) != 3 or len(set(second)) != 3 or set(first) != set(second):
        raise ValueError('两批必须为相同三种不同颜色，才能完成同色码垛')
    if set(slots) != {1, 2, 3} or set(rough2) != {1, 2, 3}:
        raise ValueError('粗加工位置必须是1、2、3的排列')
    storage = dict(zip(first, slots))
    return [dict(colors=first, rough_slots=slots, storage_slots=slots),
            dict(colors=second, rough_slots=rough2, storage_slots=tuple(storage[c] for c in second))]


def load_profile(path=None):
    path = path or Path(__file__).with_name('competition_map.json')
    return nav.load_map(path)


def collision_scene(data, margin=10, sim_obstacles=()):
    return nav.CollisionScene(data['rects'], data['circles'], data['bounds'], margin,
                              (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, 0), data['drivable_polygons'],
                              sim_circles=core.sim_obstacle_circles(sim_obstacles))


def obstacle_snapshot(data, points):
    """静态圆柱快照独立保存，不混入固定地图；非法放置明确拒绝。"""
    points = [nav.point2(p, '比赛模拟障碍') for p in points]
    if len(points) > core.SIM_OBSTACLE_MAX:
        raise ValueError('比赛模拟障碍最多%d个' % core.SIM_OBSTACLE_MAX)
    circles = list(data['circles'])
    disk_scene = nav.CollisionScene([], [], data['bounds'], 0,
        (2*core.SIM_OBSTACLE_R_MM, 2*core.SIM_OBSTACLE_R_MM, 0), data['drivable_polygons'])
    for x, y in points:
        why = core.obstacle_placement_blocked(x, y, rects=data['rects'], circles=circles,
                                              bounds=data['bounds'])
        if why:
            raise ValueError('比赛模拟障碍放置非法：'+why)
        # 障碍自身也必须完整位于车道区域。
        if not disk_scene.pose_safe(x, y, 0):
            raise ValueError('比赛模拟障碍超出可行驶区域')
        circles.extend(core.sim_obstacle_circles([(x, y)]))
    return points


def _ledger(points):
    """图搜索生成Manhattan骨架；保持完整原动作和可平滑Corner定义。"""
    def row(kind, a, b, yaw, action):
        return dict(kind=kind, x=a[0], y=a[1], to_x=b[0], to_y=b[1], heading_deg=yaw,
                    action=action, distance_mm=math.dist(a, b))
    headings = [math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])) % 360 for a, b in zip(points, points[1:])]
    rows = [row('START', points[0], points[0], headings[0], 'START')]
    previous = headings[0]
    for a, b, yaw in zip(points, points[1:], headings):
        delta = (yaw-previous+180) % 360-180
        if abs(delta) > nav.EPS:
            action = 'TURN_AROUND' if abs(delta) == 180 else 'TURN_LEFT' if delta > 0 else 'TURN_RIGHT'
            rows.append(row('TURN', a, a, yaw, action))
        rows.append(row('MOVE', a, b, yaw, 'FORWARD'))
        previous = yaw
    segments, corners = nav.path_axes(points, rows)
    return segments, corners, rows


def _tracking_reason(result, scene, cancelled):
    """100mm参考会产生切内偏差；预演真实控制器，不能只凭规划线安全放行。"""
    sim = core.Simulator(queue.Queue(), queue.Queue())
    sim.handle_line('ZERO')
    first = result['trajectory'][0]
    sim.hold = first['x_mm'], first['y_mm']
    sim.zval = 90-first['field_yaw_deg']
    sim.submit_navigation_trajectory(sim.begin_navigation(), result['trajectory'],
                                     result['smoothed_primitives'], (0, 0, 0), scene)
    for i in range(7500):
        if i % 32 == 0 and cancelled():
            raise ValueError('比赛流程规划已取消')
        sim.make_frame(i/core.SEND_HZ)
        if not sim._nav_active:
            return sim._nav_fault or None
    return '控制器预演超时'


def plan_leg(start, goal, scene, nodes, cancelled=lambda: False):
    """在静态车道图上搜索可整车平滑路线；失败明确拒绝，不强行连弧。"""
    if tuple(start) == tuple(goal):
        raise ValueError('比赛停靠点不能与出发点重合')
    for point in (start, goal):
        if not any(scene.pose_safe(*point, yaw) for yaw in (0, 90, 180, 270)):
            raise ValueError('比赛停靠点没有合法整车姿态：'+str(point))
    vertices = sorted(set(tuple(p) for p in nodes) | {tuple(start), tuple(goal)})
    links = {p: [] for p in vertices}
    for a, b in itertools.combinations(vertices, 2):
        if a[0] != b[0] and a[1] != b[1]:
            continue
        yaw = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
        if scene.translation_reason(a, b, yaw) is None:
            links[a].append(b); links[b].append(a)
    counter = itertools.count()
    queue = [(0.0, next(counter), [tuple(start)])]
    expanded, reasons = 0, []
    while queue and expanded < 5000:
        if cancelled():
            raise ValueError('比赛流程规划已取消')
        cost, _order, route = heapq.heappop(queue)
        expanded += 1
        if route[-1] == tuple(goal):
            points = [route[0]]
            for p in route[1:]:
                if len(points) >= 2 and ((points[-2][0] == points[-1][0] == p[0]) or
                                         (points[-2][1] == points[-1][1] == p[1])):
                    points[-1] = p
                else:
                    points.append(p)
            smoothed = smooth_90_corners(*_ledger(points), scene, interrupted=cancelled)
            result = generate_trajectory(smoothed['smoothed_primitives'], scene)
            if result['trajectory_safe']:
                combined = dict(points=points, **smoothed, **result)
                why = _tracking_reason(combined, scene, cancelled)
                if why is None:
                    return combined
                reasons.append('实际连续跟踪预演：'+why)
                continue
            reasons.append(result.get('trajectory_reason', smoothed['smoothing_status']))
            continue
        for target in links[route[-1]]:
            if target in route:
                continue
            turn = len(route) > 1 and ((route[-2][0] == route[-1][0]) != (route[-1][0] == target[0]))
            # 排除共线掉头；任务点的朝向切换在停车区显式旋转。
            if len(route) > 1 and not turn:
                a, b = route[-2], route[-1]
                if (b[0]-a[0])*(target[0]-b[0])+(b[1]-a[1])*(target[1]-b[1]) < 0:
                    continue
            heapq.heappush(queue, (cost+math.dist(route[-1], target)+(120 if turn else 0),
                                  next(counter), route+[target]))
    raise ValueError('比赛路段无安全连续路径：%s→%s；fallback：%s' %
                     (start, goal, '; '.join(reasons[-3:]) or '静态车道图不连通/搜索超限'))


def compile_match(data, task_code=DEFAULT_CODE, zone=1, margin=10, cancelled=lambda: False,
                  sim_obstacles=()):
    """预检整轮；站点仅来自可编辑配置，原始比赛地图不被覆盖。"""
    data = copy.deepcopy(data)
    batches = parse_task_code(task_code)
    if zone not in (1, 2):
        raise ValueError('出发区必须为1或2')
    obstacles = obstacle_snapshot(data, sim_obstacles)
    scene = collision_scene(data, margin, obstacles)
    config = data['competition']
    home = tuple(config['start_zones'][str(zone)])
    staging = tuple(config['staging'][str(zone)])
    launch_yaw = -90 if zone == 1 else 90
    lateral = (staging[0], home[1])
    for a, b in ((home, lateral), (lateral, staging)):
        why = scene.translation_reason(a, b, launch_yaw)
        if why:
            raise ValueError('启停区出入扫掠不安全：'+why)
    stages = []
    def maneuver(label, target, yaw):
        stages.append(dict(kind='MANEUVER', label=label, target=tuple(target), yaw=yaw))
    maneuver('出库横移：保持车头，移开启停区墙角', lateral, launch_yaw)
    maneuver('出库：进入可旋转停车区', staging, launch_yaw)
    current, current_yaw = staging, launch_yaw
    legs = []
    route_cache = {}
    def route_between(a, b):
        key = tuple(a), tuple(b)
        if key not in route_cache:
            route_cache[key] = plan_leg(a, b, scene, config['lane_nodes'], cancelled)
        return copy.deepcopy(route_cache[key])
    def travel(key, label):
        nonlocal current, current_yaw
        target = tuple(config['stations'][key])
        result = route_between(current, target)
        first = result['trajectory'][0]
        yaw = -90-first['field_yaw_deg']
        if abs((yaw-current_yaw+180) % 360-180) > nav.EPS:
            why = scene.turn_reason(current, current_yaw, yaw)
            if why:
                raise ValueError(label+'：出发停车区航向调整不安全：'+why)
            maneuver('停车区对齐下一路段航向', current, yaw)
        stages.append(dict(kind='TRAVEL', label=label, route=result))
        legs.append(dict(label=label, route=result))
        current, current_yaw = target, -90-result['trajectory'][-1]['field_yaw_deg']
    def action(kind, label, **fields):
        stages.append(dict(kind=kind, label=label, duration_s=.5, **fields))
    travel('qr', '前往二维码板')
    action('SCAN', '模拟扫码并显示任务码')
    for batch_i, batch in enumerate(batches, 1):
        travel('raw', '第%d批：前往原料区' % batch_i)
        for i, color in enumerate(batch['colors']):
            action('RAW_PICK', '第%d批取料%d：%s→车载仓' % (batch_i, i+1, COLORS[color]),
                   batch=batch_i, color=color)
        travel('rough', '第%d批：前往粗加工区' % batch_i)
        for color, slot in zip(batch['colors'], batch['rough_slots']):
            action('ROUGH_PLACE', '第%d批粗加工放置：%s→%d号环' % (batch_i, COLORS[color], slot),
                   batch=batch_i, color=color, slot=slot)
        for color, slot in zip(batch['colors'], batch['rough_slots']):
            action('ROUGH_PICK', '第%d批粗加工取回：%d号环%s→车载仓' % (batch_i, slot, COLORS[color]),
                   batch=batch_i, color=color, slot=slot)
        travel('storage', '第%d批：前往暂存区' % batch_i)
        for color, slot in zip(batch['colors'], batch['storage_slots']):
            action('STORAGE_PLACE', '第%d批暂存%s：%s→%d号环' %
                   (batch_i, '平放' if batch_i == 1 else '同色码垛', COLORS[color], slot),
                   batch=batch_i, color=color, slot=slot)
    # 返回安全停车区，再以固定车头横移入库；这些是显式麦轮动作，不伪造切线Trajectory。
    result = route_between(current, staging)
    yaw = -90-result['trajectory'][0]['field_yaw_deg']
    why = scene.turn_reason(current, current_yaw, yaw)
    if why:
        raise ValueError('返程对齐不安全：'+why)
    maneuver('返程停车区对齐', current, yaw)
    stages.append(dict(kind='TRAVEL', label='返回出发启停区前的停车区', route=result))
    legs.append(dict(label=stages[-1]['label'], route=result))
    yaw = -90-result['trajectory'][-1]['field_yaw_deg']
    why = scene.turn_reason(staging, yaw, launch_yaw)
    if why:
        raise ValueError('入库航向调整不安全：'+why)
    maneuver('入库前对齐车头', staging, launch_yaw)
    maneuver('入库：保持航向驶向启停区边', lateral, launch_yaw)
    maneuver('入库横移：返回抽签启停区', home, launch_yaw)
    return dict(schema_version=1, kind='PRELIMINARY_PC_SIMULATION', task_code=task_code,
                zone=zone, home=home, start_yaw=launch_yaw, stages=stages, legs=legs,
                batches=batches, map_snapshot=copy.deepcopy(data), margin_mm=margin,
                sim_obstacles=obstacles, obstacle_frame_id='LAYOUT_MM',
                obstacle_radius_mm=core.SIM_OBSTACLE_R_MM, obstacle_height_mm=core.SIM_OBSTACLE_H_MM,
                run_limit_s=RUN_LIMIT_S, hardware_ready=False)


class PoseManeuver:
    """工作区/出入库的显式整车平移或原地旋转，复用模拟器的提交前扫掠保护。"""
    def __init__(self, start, target):
        self.start, self.target = tuple(start), dict(target)
        self.length = math.dist(start[:2], (target['x_mm'], target['y_mm']))
        self.progress = self.cross_track = 0.0

    def reference_at(self, _station):
        return dict(self.target, s_mm=self.length, segment_type='STOP')

    def command(self, pose, speed_limit, yaw_rate_limit):
        x, y, yaw = pose
        dx, dy = self.target['x_mm']-x, self.target['y_mm']-y
        distance = math.hypot(dx, dy)
        delta = (self.target['field_yaw_deg']-yaw+180) % 360-180
        speed = min(speed_limit, distance*4) if distance >= .2 else distance*core.SEND_HZ
        omega = max(-yaw_rate_limit, min(yaw_rate_limit, delta*6 if abs(delta) >= .05 else delta*core.SEND_HZ))
        self.progress = max(self.progress, self.length-distance)
        velocity = (0, 0) if distance < 1e-9 else (speed*dx/distance, speed*dy/distance)
        return self.reference_at(0), velocity, 0 if abs(delta) < 1e-9 else omega


class CompetitionRunner:
    """只消费新模拟积分帧；每个作业动作完成后才更新仓位和统计。"""
    def __init__(self, sim, match, mapping=(0, 0, 0), validity=lambda: True):
        self.sim, self.match, self.mapping, self.validity = sim, match, mapping, validity
        self.scene = collision_scene(match['map_snapshot'], match['margin_mm'], match.get('sim_obstacles', ()))
        self.status, self.reason = 'READY', ''
        self.index, self.elapsed_s, self.action_elapsed_s = -1, 0.0, 0.0
        self.cargo, self.rough, self.storage = [], {}, {}
        self.grabs = self.placements = 0
        self.display_code = ''
        self.history = []
        self.actual_trace = []
        self.epoch = self.goal_id = None
        self.frame_seq = sim.navigation_snapshot()['frame_seq']

    @property
    def active(self):
        return self.status in ('RUNNING', 'ACTION')

    @property
    def stage(self):
        return self.match['stages'][self.index] if 0 <= self.index < len(self.match['stages']) else None

    def start(self):
        if self.status != 'READY':
            raise ValueError('每轮只有一次启动机会；重新准备后才能启动')
        self.status = 'RUNNING'
        try:
            self._next()
        except Exception as exc:
            self._fail(exc)
            raise

    def cancel(self, reason='比赛模拟已取消'):
        if self.active:
            self.status, self.reason = 'CANCELLED', reason
            self.sim.cancel_navigation()

    def _fail(self, reason):
        self.status, self.reason = 'FAULT', str(reason)
        self.sim.cancel_navigation()

    def _next(self):
        self.index += 1
        self.action_elapsed_s = 0.0
        if self.index >= len(self.match['stages']):
            if self.cargo or self.rough or self.grabs != 12 or self.placements != 12 or any(
                    len(v) != 2 or v[0] != v[1] for v in self.storage.values()) or len(self.storage) != 3:
                raise ValueError('最终搬运库存/统计不一致')
            self.status = 'COMPLETE'
            self.history.append(dict(kind='COMPLETE', elapsed_s=self.elapsed_s, grabs=self.grabs, placements=self.placements))
            return
        stage = self.stage
        self.history.append(dict(kind='ENTER', index=self.index, label=stage['label'], elapsed_s=self.elapsed_s))
        self.epoch = self.sim.begin_navigation()
        self.goal_id = None
        if stage['kind'] == 'TRAVEL':
            if len(self.cargo) not in (0, 3):
                raise ValueError('物料必须放入车载仓后才能跨区域运行')
            r = stage['route']
            self.goal_id = self.sim.submit_navigation_trajectory(self.epoch, r['trajectory'],
                r['smoothed_primitives'], self.mapping, self.scene, self.validity)
            self.status = 'RUNNING'
        elif stage['kind'] == 'MANEUVER':
            self.goal_id = self.sim.submit_navigation_maneuver(self.epoch, stage['target'], stage['yaw'],
                                                              self.mapping, self.scene, self.validity)
            self.status = 'RUNNING'
        else:
            self.status = 'ACTION'
            self.action_pose = self._field_pose(self.sim.navigation_snapshot())

    def _field_pose(self, snap):
        if snap['hold'] is None:
            raise ValueError('比赛实际定位丢失')
        mx, my, angle = self.mapping
        c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        x, y = snap['hold']
        pose = mx+c*x+s*y, my-s*x+c*y, 90-angle-snap['yaw']
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('比赛实际位置/航向非法')
        return pose

    def _commit_action(self):
        stage = self.stage
        kind = stage['kind']
        if kind == 'SCAN':
            parse_task_code(self.match['task_code'])
            self.display_code = self.match['task_code']
        else:
            color, slot = stage['color'], stage.get('slot')
            if not self.display_code:
                raise ValueError('未扫码，不能执行搬运')
            if kind in ('RAW_PICK', 'ROUGH_PICK'):
                if len(self.cargo) >= 3 or color in self.cargo:
                    raise ValueError('车载仓已满或重复颜色')
                if kind == 'ROUGH_PICK':
                    if self.rough.get(slot) != color:
                        raise ValueError('粗加工取料位置/颜色错误')
                    del self.rough[slot]
                self.cargo.append(color); self.grabs += 1
            else:
                if not self.cargo or self.cargo[0] != color:
                    raise ValueError('放置顺序或车载物料错误')
                if kind == 'ROUGH_PLACE':
                    if slot in self.rough:
                        raise ValueError('粗加工环已占用')
                    self.rough[slot] = color
                elif kind == 'STORAGE_PLACE':
                    stack = self.storage.get(slot, [])
                    if (stage['batch'] == 1 and stack) or (stage['batch'] == 2 and stack != [color]):
                        raise ValueError('暂存区必须在正确同色第一层上码垛')
                    self.storage[slot] = stack+[color]
                else:
                    raise ValueError('未知机械作业')
                self.cargo.pop(0); self.placements += 1
        self.history.append(dict(kind=kind, index=self.index, elapsed_s=self.elapsed_s,
                                 color=stage.get('color'), slot=stage.get('slot'), cargo=list(self.cargo)))

    def tick(self):
        if not self.active:
            return
        try:
            snap = self.sim.navigation_snapshot()
            if not self.validity() or snap['epoch'] != self.epoch or snap['fault'] or not snap['wheel_enabled']:
                raise ValueError('地图/控制权/模拟会话已改变：'+snap['fault'])
            if self.goal_id is not None and snap['goal_id'] != self.goal_id:
                raise ValueError('比赛阶段目标ID已被替换')
            pose = self._field_pose(snap)
            layout = core.field_to_layout(*pose[:2])
            why = self.scene.pose_reason(*layout, -90-pose[2])
            if why:
                raise ValueError('比赛实际车体不安全：'+why)
            if snap['frame_seq'] == self.frame_seq:
                return
            dt = (snap['frame_seq']-self.frame_seq)/core.SEND_HZ
            self.frame_seq = snap['frame_seq']
            self.elapsed_s += dt
            self.actual_trace.append(dict(x_mm=pose[0], y_mm=pose[1], field_yaw_deg=pose[2],
                                          elapsed_s=self.elapsed_s, stage_index=self.index))
            if self.elapsed_s > RUN_LIMIT_S:
                raise ValueError('超过初赛180秒运行时限')
            if self.status == 'ACTION':
                if snap['tracking'] or snap['manual'] is not None or snap['goto'] is not None:
                    raise ValueError('机械作业期间底盘不能运行')
                if math.dist(pose[:2], self.action_pose[:2]) > 1 or abs((pose[2]-self.action_pose[2]+180) % 360-180) > 1:
                    raise ValueError('机械作业期间实际底盘位置/航向改变')
                self.action_elapsed_s += dt
                if self.action_elapsed_s > 15:
                    raise ValueError('作业停止超过15秒')
                if self.action_elapsed_s >= self.stage['duration_s']:
                    self._commit_action(); self._next()
            elif snap['tracking_status'] == 'COMPLETE' and snap['completed_id'] == self.goal_id:
                if snap['settled_frames'] < 10 or snap['speed_mm_s'] > 1 or abs(snap['yaw_rate_deg_s']) > 1:
                    raise ValueError('阶段终点未停稳')
                if self.stage['kind'] == 'TRAVEL':
                    final = self.stage['route']['trajectory'][-1]
                    target = final['x_mm'], final['y_mm'], final['field_yaw_deg']
                else:
                    target = (*core.layout_to_field(*self.stage['target']), -90-self.stage['yaw'])
                if math.dist(pose[:2], target[:2]) >= 1 or abs((pose[2]-target[2]+180) % 360-180) >= 1:
                    raise ValueError('阶段实际终点位置/航向未到位')
                self._next()
            elif not snap['tracking']:
                raise ValueError('比赛阶段运动已中断')
        except Exception as exc:
            self._fail(exc)

    def export_snapshot(self):
        return dict(status=self.status, reason=self.reason, elapsed_s=self.elapsed_s, stage_index=self.index,
                    display_code=self.display_code, cargo=self.cargo, rough=self.rough, storage=self.storage,
                    correct_grabs=self.grabs, correct_placements=self.placements,
                    history=self.history, actual_trace=self.actual_trace, match=self.match, hardware_ready=False)
