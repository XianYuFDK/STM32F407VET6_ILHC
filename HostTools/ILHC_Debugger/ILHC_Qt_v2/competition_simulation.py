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
from mecanum_planner import edge_cost, shallow_diagonal, motion_metrics

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
            if not sim._nav_fault:
                result['predicted_tracking_s'] = (i+1)/core.SEND_HZ
            return sim._nav_fault or None
    return '控制器预演超时'


def _lane_graph(start, goal, scene, nodes, cancelled, refined, cache):
    """补齐车道交点；必要时按障碍尺寸添加绕行平行线，不把车道图当通路白名单。"""
    anchors = set(tuple(p) for p in nodes) | {tuple(start), tuple(goal)}
    xs, ys = {p[0] for p in anchors}, {p[1] for p in anchors}
    base_xs, base_ys = tuple(xs), tuple(ys)
    if refined:
        for x, y, radius, _name in scene.circles:
            for half in (scene.footprint.length_mm/2, scene.footprint.width_mm/2):
                clearance = radius+half+scene.pad+10
                xs.update((x-clearance, x+clearance))
                ys.update((y-clearance, y+clearance))
            # 障碍附近的原车道中心线允许偏移，所有新增姿态仍走真实矩形检查。
            for coords, base, center in ((xs, base_xs, x), (ys, base_ys, y)):
                nearby = [v for v in base if abs(v-center) < radius+220]
                coords.update(v+offset for v in nearby for offset in (-25, 25))
    fixed_heading = hasattr(scene, 'yaw')
    key = (tuple(sorted(xs)), tuple(sorted(ys)), tuple(sorted((tuple(start), tuple(goal)))) if fixed_heading else None)
    if key in cache:
        return cache[key]
    vertices = []
    for i, p in enumerate(itertools.product(sorted(xs), sorted(ys))):
        if i % 32 == 0 and cancelled():
            raise ValueError('比赛流程规划已取消')
        if any(scene.pose_safe(*p, yaw) for yaw in (0, 90)):
            vertices.append(p)
    links, columns, rows = {p: [] for p in vertices}, {}, {}
    for p in vertices:
        columns.setdefault(p[0], []).append(p)
        rows.setdefault(p[1], []).append(p)
    count = 0
    for group in list(columns.values())+list(rows.values()):
        for a, b in itertools.combinations(group, 2):
            count += 1
            if count % 32 == 0 and cancelled():
                raise ValueError('比赛流程规划已取消')
            yaw = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
            if scene.translation_reason(a, b, yaw) is None:
                links[a].append(b); links[b].append(a)
    if fixed_heading:
        # 站点的短轴向接头容易容不下60mm倒角；允许安全的斜向接近/离开，不能先判成无路。
        for endpoint in (tuple(start), tuple(goal)):
            for i, p in enumerate(vertices):
                if i % 32 == 0 and cancelled():
                    raise ValueError('比赛流程规划已取消')
                if p == endpoint or p in links.get(endpoint, ()):
                    continue
                if scene.translation_reason(endpoint, p, scene.yaw) is None:
                    links.setdefault(endpoint, []).append(p)
                    links[p].append(endpoint)
    cache[key] = links
    return links


def plan_leg(start, goal, scene, nodes, cancelled=lambda: False, *, graph_cache=None, body_yaw=None):
    """A*搜索唯一骨架并提前检查转弯；先补车道交点，再尝试障碍旁的偏移车道。"""
    actual_scene = scene
    if body_yaw is not None:
        from mecanum_planner import FixedHeadingScene, fixed_route
        scene = FixedHeadingScene(scene, body_yaw)
    if tuple(start) == tuple(goal):
        raise ValueError('比赛停靠点不能与出发点重合')
    for point in (start, goal):
        if not any(scene.pose_safe(*point, yaw) for yaw in (0, 90, 180, 270)):
            raise ValueError('比赛停靠点没有合法整车姿态：'+str(point))
    cache = {} if graph_cache is None else graph_cache
    reasons, limit_hit, corner_cache = [], False, {}
    heuristic = (lambda p: math.dist(p, goal)) if body_yaw is not None else \
                (lambda p: abs(p[0]-goal[0])+abs(p[1]-goal[1]))
    for refined in (False, True):
        links = _lane_graph(start, goal, scene, nodes, cancelled, refined, cache)
        # 真正断开的图先做线性连通性判定，不能枚举起点一侧的所有循环绕路。
        connected, pending = {tuple(start)}, [tuple(start)]
        while pending:
            if cancelled():
                raise ValueError('比赛流程规划已取消')
            for neighbor in links.get(pending.pop(), ()):
                if neighbor not in connected:
                    connected.add(neighbor); pending.append(neighbor)
        if tuple(goal) not in connected:
            continue
        counter = itertools.count()
        frontier = [(heuristic(start), next(counter), 0.0, [tuple(start)])]
        seen, expanded, candidates, visits = {(tuple(start),)}, 0, 0, {}
        while frontier and expanded < 5000:
            if cancelled():
                raise ValueError('比赛流程规划已取消')
            _score, _order, cost, route = heapq.heappop(frontier)
            expanded += 1
            if len(route) >= 3:
                # 只检查A*实际展开的转弯，不为尚未选择的几万条邻边逐一采样圆弧。
                corner_key = tuple(route[-3:])
                if body_yaw is None:
                    if corner_key not in corner_cache:
                        local = smooth_90_corners(*_ledger(corner_key), scene, interrupted=cancelled)
                        corner_cache[corner_key] = not local['arc_fallbacks']
                    if not corner_cache[corner_key]:
                        continue
                # 固定车头会先拉直为安全斜线；短轴向接头可能被消除，不能提前按90°判死。
                # 保留不同前缀的少量候选，兼顾相邻圆弧裁剪和整段实际控制预演。
                state = tuple(route[-4:])
                visits[state] = visits.get(state, 0)+1
                if visits[state] > 3:
                    limit_hit = True
                    continue
            if route[-1] == tuple(goal):
                candidates += 1
                if body_yaw is not None:
                    try:
                        return fixed_route(route, body_yaw, actual_scene, cancelled, _tracking_reason,
                                           dict(refined=refined, expanded=expanded, candidates=candidates, vertices=len(links)))
                    except ValueError as exc:
                        if cancelled():
                            raise
                        reasons.append(str(exc))
                    continue
                smoothed = smooth_90_corners(*_ledger(route), scene, interrupted=cancelled)
                result = generate_trajectory(smoothed['smoothed_primitives'], scene)
                if result['trajectory_safe']:
                    combined = dict(points=route, **smoothed, **result)
                    why = _tracking_reason(combined, scene, cancelled)
                    if why is None:
                        combined['search'] = dict(refined=refined, expanded=expanded,
                                                  candidates=candidates, vertices=len(links))
                        return combined
                    reasons.append('实际连续跟踪预演：'+why)
                else:
                    reasons.append(result.get('trajectory_reason', smoothed['smoothing_status']))
                continue
            for target in links.get(route[-1], ()):
                if target in route:
                    continue
                turn = False
                if len(route) > 1:
                    a, b = route[-2], route[-1]
                    ux, uy, vx, vy = b[0]-a[0], b[1]-a[1], target[0]-b[0], target[1]-b[1]
                    turn = abs(ux*vy-uy*vx) > nav.EPS
                trial = route+[target]
                if len(route) > 1 and not turn:
                    a, b = route[-2], route[-1]
                    if (b[0]-a[0])*(target[0]-b[0])+(b[1]-a[1])*(target[1]-b[1]) < 0:
                        continue
                    # 入队前合并同向直线，防止几千种等价节点组合耗尽搜索预算。
                    trial = route[:-1]+[target]
                key = tuple(trial)
                if key in seen:
                    continue
                seen.add(key)
                distance_cost = math.dist(route[-1], target) if body_yaw is None else edge_cost(route[-1], target, body_yaw)
                # 浅斜边可作障碍绕行fallback；正常直车道优先，避免站点偏移被拉成整段斜线。
                shape_cost = (220+min(abs(target[k]-route[-1][k]) for k in range(2))) \
                    if body_yaw is not None and shallow_diagonal(route[-1], target) else 0
                new_cost = cost+distance_cost+(120 if turn else 0)+shape_cost
                heapq.heappush(frontier, (new_cost+heuristic(target), next(counter), new_cost, trial))
        limit_hit |= bool(frontier)
    reason = '; '.join(dict.fromkeys(reasons))[-600:] or (
        '候选搜索预算已用尽，不能判定物理通路不存在' if limit_hit else
        '车道不连通或没有满足60..120mm圆弧与整车扫掠的候选')
    raise ValueError('比赛路段无安全连续路径：%s→%s；fallback：%s' % (start, goal, reason))


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
    # 用户默认车头朝场地+Y（界面0°）；LAYOUT +x朝场地-Y，所以实际布局角为180°。
    launch_yaw = 180
    lateral = (staging[0], home[1])
    diagonal_docking = scene.translation_reason(home, staging, launch_yaw) is None
    docking_edges = ((home, staging),) if diagonal_docking else ((home, lateral), (lateral, staging))
    for a, b in docking_edges:
        why = scene.translation_reason(a, b, launch_yaw)
        if why:
            raise ValueError('启停区出入扫掠不安全：'+why)
    stages = []
    def maneuver(label, target, yaw):
        stages.append(dict(kind='MANEUVER', label=label, target=tuple(target), yaw=yaw))
    if diagonal_docking:
        maneuver('麦轮斜向出库：保持车头进入停车区', staging, launch_yaw)
    else:
        maneuver('出库横移：保持车头，移开启停区墙角', lateral, launch_yaw)
        maneuver('出库：进入可旋转停车区', staging, launch_yaw)
    current, current_yaw = staging, launch_yaw
    legs = []
    route_cache = {}
    graph_cache = {}
    def route_between(a, b, heading):
        key = tuple(a), tuple(b), heading % 360
        if key not in route_cache:
            failures, candidates = [], []
            # 优先当前车头；矩形车体固定方向绕行，不因换向强制旋转。
            try:
                candidates.append(plan_leg(a, b, scene, config['lane_nodes'], cancelled,
                                  graph_cache=graph_cache.setdefault(heading % 180, {}), body_yaw=heading))
            except ValueError as exc:
                if cancelled():
                    raise
                failures.append(str(exc))
            # 长横移也必须比较正交车头，不能仅凭几何路线短就保持原航向。
            lower_bound = math.dist(a, b)
            if not candidates or candidates[0]['trajectory_length_mm'] > lower_bound*1.3 or \
                    motion_metrics(candidates[0])['lateral_mm'] >= 500:
                for yaw in (heading+90, heading-90):
                    if scene.turn_reason(a, heading, yaw):
                        continue
                    try:
                        candidates.append(plan_leg(a, b, scene, config['lane_nodes'], cancelled,
                                          graph_cache=graph_cache.setdefault(yaw % 180, {}), body_yaw=yaw))
                    except ValueError as exc:
                        if cancelled():
                            raise
                        failures.append(str(exc))
            # 固定车头候选仍有长横移时，也比较圆弧中连续转动车头的前进/倒退方案。
            if not candidates or all(motion_metrics(r)['longest_strafe_mm'] > 500 for r in candidates):
                try:
                    forward = plan_leg(a, b, scene, config['lane_nodes'], cancelled,
                                       graph_cache=graph_cache.setdefault('tangent', {}))
                    for mode in ('TANGENT', 'REVERSE_TANGENT'):
                        pieces = [dict(p, heading_mode=mode) for p in forward['smoothed_primitives']]
                        if mode == 'REVERSE_TANGENT':
                            for p in pieces:
                                p['action'] = 'BACKWARD'
                                for k in ('heading_deg', 'heading_in_deg', 'heading_out_deg'):
                                    if k in p:
                                        p[k] = (p[k]+180) % 360
                                if 'sample_poses' in p:
                                    p['sample_poses'] = [(x, y, (yaw+180)%360) for x, y, yaw in p['sample_poses']]
                        result = dict(forward, smoothed_primitives=pieces,
                                      arcs=[p for p in pieces if p['kind'] == 'ARC'],
                                      planner='MECANUM_'+mode, **generate_trajectory(pieces, scene, interrupted=cancelled))
                        yaw = -90-result['trajectory'][0]['field_yaw_deg'] if result['trajectory_safe'] else heading
                        if result['trajectory_safe'] and not scene.turn_reason(a, heading, yaw):
                            why = _tracking_reason(result, scene, cancelled)
                            if why is None:
                                candidates.append(result)
                except ValueError as exc:
                    if cancelled():
                        raise
                    failures.append(str(exc))
            if not candidates:
                raise ValueError('比赛路段无安全麦轮路径；fallback：'+'; '.join(failures))
            def cost(r):
                yaw = -90-r['trajectory'][0]['field_yaw_deg']
                angle = abs((yaw-heading+180) % 360-180)
                # 转向按真实120°/s上限+比例收敛和停稳开销估计；行驶为50Hz控制预演计时。
                turn_s = 0 if angle < nav.EPS else angle/120+1.0
                # 方向权重来自普通导航1/1.15/1.8，不改模拟器250mm/s积分速度。
                # 只计算实际发生的出发旋转；下一站沿用已选车头，不虚构每站恢复。
                metrics = motion_metrics(r)
                direction_s = (metrics['equivalent_cost_mm']-r['trajectory_length_mm'])/250
                return r['predicted_tracking_s']+turn_s+direction_s
            chosen = min(candidates, key=cost)
            chosen['motion_metrics'] = motion_metrics(chosen)
            chosen['selection'] = dict(policy='DIRECTION_COST_AND_VERIFIED_TIME',
                direction_weights=dict(forward=nav.DEFAULT_COST_FORWARD, reverse=nav.DEFAULT_COST_BACKWARD,
                                       lateral=nav.DEFAULT_COST_LATERAL),
                candidates=[dict(planner=r['planner'], travel_s=r['predicted_tracking_s'],
                                 body_yaw_deg=-90-r['trajectory'][0]['field_yaw_deg'],
                                 cost_s=cost(r), length_mm=r['trajectory_length_mm'],
                                 motion_metrics=motion_metrics(r)) for r in candidates],
                fallbacks=failures)
            route_cache[key] = chosen
        return copy.deepcopy(route_cache[key])
    def travel(key, label):
        nonlocal current, current_yaw
        target = tuple(config['stations'][key])
        result = route_between(current, target, current_yaw)
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
    # 返回安全停车区；真实车头对齐后斜向入库，斜线不安全则保留已检查的L形横移。
    result = route_between(current, staging, current_yaw)
    yaw = -90-result['trajectory'][0]['field_yaw_deg']
    why = scene.turn_reason(current, current_yaw, yaw)
    if why:
        raise ValueError('返程对齐不安全：'+why)
    if abs((yaw-current_yaw+180) % 360-180) > nav.EPS:
        maneuver('返程停车区对齐', current, yaw)
    stages.append(dict(kind='TRAVEL', label='返回出发启停区前的停车区', route=result))
    legs.append(dict(label=stages[-1]['label'], route=result))
    yaw = -90-result['trajectory'][-1]['field_yaw_deg']
    why = scene.turn_reason(staging, yaw, launch_yaw)
    if why:
        raise ValueError('入库航向调整不安全：'+why)
    if abs((yaw-launch_yaw+180) % 360-180) > nav.EPS:
        maneuver('入库前对齐车头', staging, launch_yaw)
    if diagonal_docking:
        maneuver('麦轮斜向入库：保持车头返回启停区', home, launch_yaw)
    else:
        maneuver('入库：保持航向驶向启停区边', lateral, launch_yaw)
        maneuver('入库横移：返回抽签启停区', home, launch_yaw)
    return dict(schema_version=2, kind='PRELIMINARY_PC_SIMULATION', task_code=task_code,
                planner='MECANUM_BODY_AND_TANGENT', diagonal_docking=diagonal_docking,
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
        # 进入1°/s停稳门之前消除小角度残差，避免转向完成后下一平移被判隐式旋转。
        omega = max(-yaw_rate_limit, min(yaw_rate_limit, delta*6 if abs(delta) >= .2 else delta*core.SEND_HZ))
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
