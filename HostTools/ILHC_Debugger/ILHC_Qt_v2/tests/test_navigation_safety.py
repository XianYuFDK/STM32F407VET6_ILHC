from __future__ import annotations
import copy
import heapq
import json
import math
from pathlib import Path
import queue
import random
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import core
import navigation_planner as nav
from tests.headless_support import make_window, close_window, advance

class CoreSafetyTests(unittest.TestCase):
    def check_path(self, result, **scene_args):
        self.assertTrue(result['ok'],result.get('reason'))
        scene=nav.CollisionScene(**scene_args)
        self.assertIsNone(scene.validate(result['points']))
        self.assertIsNone(scene.validate(result['raw']))
        self.assertAlmostEqual(result['length'],core.path_length(result['points']))
        self.assertAlmostEqual(result['length'],core.path_length(result['raw']))
        # 加权代价含动作系数与转向罚，只会 ≥ 几何长度；两者必须与逐步台账自洽
        self.assertGreaterEqual(result['search_cost']+1e-6,result['length'])
        self.assertAlmostEqual(result['search_cost'],
                               result.get('trace_cost',result['search_cost']),places=6)
        self.assertAlmostEqual(sum(s['cost_mm'] for s in result['steps']),
                               result['search_cost'],places=6)

    def test_default_material_goal_rejected(self):
        r=core.plan_path((2250,2250),(1200,2200))
        self.assertFalse(r['ok']); self.assertFalse(r['execution_safe']); self.assertEqual(r['points'],[])

    def test_first_and_last_connectors_checked(self):
        r=core.plan_path((390,1110),(1170,640))
        if r['ok']:
            self.check_path(r,rects=core.FIELD_FORBIDDEN_RECTS,circles=core.FIELD_FORBIDDEN_CIRCLES,
                            bounds=(0,0,2400,2400),pad=150)
        else:
            self.assertIn(r['code'],('INVALID_START','INVALID_GOAL','NO_CONNECTION','NO_PATH'))

    def test_coarse_grid_edge_cannot_jump_over_cylinder(self):
        kwargs=dict(rects=[],circles=[(750,1000,25,'pillar')],bounds=(0,0,2000,2000),pad=150)
        r=core.plan_path((500,1000),(1000,1000),grid=500,**kwargs)
        self.check_path(r,**kwargs)
        self.assertGreater(len(r['points']),2)

    def test_vehicle_centre_on_outer_boundary_rejected(self):
        self.assertFalse(core.plan_path((0,300),(500,300))['ok'])

    def test_nondivisible_grid_cannot_escape_wall(self):
        r=core.plan_path((500,2000),(1900,2000),grid=175,pad=0,
                         rects=[(1150,0,1250,2400,'wall')],circles=[],bounds=(0,0,2400,2400))
        self.assertFalse(r['ok']); self.assertEqual(r['code'],'NO_PATH')

    def test_clearance_uses_custom_scene(self):
        r=core.plan_path((100,100),(400,100),pad=0,rects=[],circles=[(250,500,25,'custom')],bounds=(0,0,1000,1000))
        self.assertTrue(r['ok']); self.assertAlmostEqual(r['min_clearance'],375)
        self.assertEqual(r['boundary_clearance'],100)

    def test_continuous_clearance_cannot_skip_midpoint(self):
        self.assertEqual(core.path_clearance([(0,0),(99,0)],[],[(49.5,0,25,'p')]),0)

    def test_clearance_single_point_and_empty_obstacles(self):
        self.assertEqual(core.path_clearance([(0,0)],[],[(100,0,25,'p')]),75)
        self.assertTrue(math.isinf(core.path_clearance([(0,0),(20,0)],[],[])))
        self.assertIsNone(core.path_clearance([],[],[]))

    def test_two_point_simplification_checks_segment(self):
        with self.assertRaises(ValueError):
            core.simplify_path([(0,0),(100,0)],[],[(50,0,10,'p')])

    def test_simplification_cannot_fallback_to_bad_adjacent_edge(self):
        with self.assertRaises(ValueError):
            core.simplify_path([(0,0),(100,0),(200,0)],[],[(50,0,10,'p')])

    def test_fixed_rect_not_disc_and_diagonal_corner(self):
        kwargs=dict(rects=[(550,550,1000,1000,'zone')],circles=[],bounds=(0,0,2400,2400),pad=0)
        self.assertTrue(core.plan_path((1160,700),(1160,900),footprint=(280,260,0),**kwargs)['ok'])
        # 状态含航向后只支持 4 个 90° 倍数航向：footprint 给 45° 与起始航向 0° 不一致，
        # 必须显式拒绝（INVALID_INPUT），不能静默按 0° 求解
        bad=core.plan_path((1160,700),(1160,900),footprint=(280,260,45),**kwargs)
        self.assertFalse(bad['ok'])
        self.assertEqual(bad['code'],'INVALID_INPUT')

    def test_whole_body_outside_boundary(self):
        r=core.plan_path((100,300),(700,300),pad=0,footprint=(280,260,0),rects=[],circles=[])
        self.assertFalse(r['ok']); self.assertIn('边界',r['reason'])

    def test_pad_only_plan_is_not_car_execution_approval(self):
        r=core.plan_path((500,1200),(1900,1200))
        self.assertTrue(r['ok']); self.assertFalse(r['execution_safe']); self.assertFalse(r['hardware_ready'])

    def test_fixed_body_is_checked_and_hardware_gate_stays_closed(self):
        # 状态含航向：footprint 的 180° 必须与起始航向一致
        kwargs=dict(footprint=(280,260,180),pad=10,geometry_verified=True,start_heading_deg=180.0)
        r=core.plan_path((2250,2250),(330,1200),**kwargs)
        self.assertTrue(r['ok'],r); self.assertTrue(r['execution_safe'])
        self.assertFalse(r['hardware_ready']); self.assertGreaterEqual(r['body_clearance'],9.99)

    def test_legal_region_not_just_absence_of_obstacles(self):
        regions=[[[100,100],[1900,100],[1900,500],[100,500]]]
        r=core.plan_path((300,300),(1700,900),rects=[],circles=[],bounds=(0,0,2000,2000),
                        pad=0,drivable_polygons=regions,footprint=(100,100,0))
        self.assertFalse(r['ok']); self.assertIn('行驶区域',r['reason'])

    def test_swept_polygon_not_just_corners(self):
        # All rectangle corners are outside the hole, but the body covers the hole.
        region=[dict(outer=[[0,0],[1000,0],[1000,1000],[0,1000]],
                     holes=[[[490,490],[510,490],[510,510],[490,510]]])]
        scene=nav.CollisionScene([],[],(0,0,1000,1000),0,(100,100,0),region)
        self.assertIsNotNone(scene.segment_reason((500,500),(500,500)))

    def test_concave_lane_sweep_cannot_cut_corner(self):
        regions=[[[0,0],[1000,0],[1000,300],[300,300],[300,1000],[0,1000]]]
        scene=nav.CollisionScene([],[],(0,0,1000,1000),0,(100,100,0),regions)
        self.assertIsNone(scene.segment_reason((150,850),(150,150)))
        self.assertIsNotNone(scene.segment_reason((150,850),(850,150)))

    def test_adjacent_legal_rectangles_must_be_unioned(self):
        regions=[[[0,0],[500,0],[500,1000],[0,1000]],[[500,0],[1000,0],[1000,1000],[500,1000]]]
        scene=nav.CollisionScene([],[],(0,0,1000,1000),0,(200,200,0),regions)
        self.assertIsNone(scene.segment_reason((250,500),(750,500)))

    def test_self_intersecting_map_rejected(self):
        r=core.plan_path((100,100),(800,800),pad=0,rects=[],circles=[],bounds=(0,0,1000,1000),
                        drivable_polygons=[[[0,0],[1000,1000],[0,1000],[1000,0]]])
        self.assertEqual(r['code'],'INVALID_INPUT')

    def test_empty_whitelist_rejected(self):
        r=core.plan_path((100,100),(800,800),pad=0,rects=[],circles=[],drivable_polygons=[])
        self.assertFalse(r['ok'])

    def test_nonfinite_and_invalid_inputs_return_failure_not_exception(self):
        for kwargs in (dict(start=(math.nan,100)),dict(goal=(math.inf,100)),dict(grid=math.nan),
                       dict(grid=0),dict(grid=5),dict(pad=-1),dict(bounds=(100,0,0,100)),
                       dict(circles=[(0,0,-3,'p')]),dict(footprint=(0,260,0))):
            with self.subTest(kwargs=kwargs):
                args=dict(start=(400,400),goal=(1600,1600)); args.update(kwargs)
                self.assertFalse(core.plan_path(**args)['ok'])

    def test_cancelled_search(self):
        event=threading.Event(); event.set()
        r=core.plan_path((2250,2250),(330,1200),cancel=event)
        self.assertEqual(r['code'],'CANCELLED'); self.assertEqual(r['points'],[])

    def test_bounded_runtime(self):
        r=core.plan_path((2250,2250),(330,1200),grid=10,time_limit_s=0.001)
        self.assertEqual(r['code'],'CANCELLED')

    def test_start_equals_goal_no_division_by_zero(self):
        r=core.plan_path((1200,1200),(1200,1200))
        self.assertTrue(r['ok']); self.assertEqual(r['length'],0); self.assertEqual(r['points'],[(1200,1200)])

    def test_raw_connectors_and_cost_are_included(self):
        r=core.plan_path((2249,2248),(329,1201),grid=175)
        self.assertTrue(r['ok'],r)
        self.assertEqual(r['raw'][0],(2249,2248)); self.assertEqual(r['raw'][-1],(329,1201))
        self.assertGreaterEqual(r['search_cost']+1e-6,core.path_length(r['raw']))
        self.assertLessEqual(r['length'],r['search_cost']+1e-6)

    def test_independent_dijkstra_40_scenes(self):
        """独立参照：**状态=(格, 航向)** 的 Dijkstra，与 plan() 同一套动作代价。

        规划已升级为航向状态 + 分流代价（前进 1.0 / 后退 1.15 / 横移 1.8）
        + 原地转向 180mm/90°，参照必须同模型，否则比的是两个不同的寻路问题。
        这些场景是 pad-only（不传 footprint）：碰撞模型与航向无关、转向扫掠判据不生效，
        所以参照里只需要算节点与边。
        """
        COST = {"FORWARD": 1.0, "BACKWARD": 1.15, "STRAFE": 1.8}
        TURN = 180.0
        def act_of(h, di, dj):
            fx, fy = math.cos(math.radians(90*h)), math.sin(math.radians(90*h))
            dot = di*fx + dj*fy
            return "FORWARD" if dot > 0.5 else ("BACKWARD" if dot < -0.5 else "STRAFE")
        def turn90(a, b):
            d = (b - a) % 4
            return 0 if d == 0 else (2 if d == 2 else 1)
        def leg_cost(la, lb, h):
            if math.dist(la, lb) <= 1e-9:
                return 0.0
            di = 0 if abs(lb[0]-la[0]) <= 1e-9 else (1 if lb[0] > la[0] else -1)
            dj = 0 if abs(lb[1]-la[1]) <= 1e-9 else (1 if lb[1] > la[1] else -1)
            return math.dist(la, lb) * COST[act_of(h, di, dj)]
        rng=random.Random(7429)
        for case in range(40):
            circles=[(rng.uniform(300,700),rng.uniform(200,800),rng.uniform(20,65),'p') for _ in range(3)]
            rects=[(450,0,550,650,'wall')]
            scene=nav.CollisionScene(rects,circles,(0,0,1000,1000),10)
            start=(100.,500.);goal=(900.,500.)
            r=core.plan_path(start,goal,grid=100,pad=10,rects=rects,circles=circles,bounds=(0,0,1000,1000))
            nodes=[(i*100.,j*100.) for i in range(11) for j in range(11)
                   if scene.segment_reason((i*100.,j*100.),(i*100.,j*100.)) is None]
            node_set=set(nodes)
            def attach(p,h_idx,reverse):
                """轴对齐 L 形连接，代价按动作分类累加（与 plan() 同规则）。"""
                out={}
                i0=round(p[0]/100);j0=round(p[1]/100)
                for i in range(max(0,i0-8),min(11,i0+9)):
                    for j in range(max(0,j0-8),min(11,j0+9)):
                        n=(i*100.,j*100.)
                        if n not in node_set or scene.segment_reason(n,n):continue
                        for corner in ((n[0],p[1]),(p[0],n[1])):
                            legs=((p,corner),(corner,n)) if not reverse else ((n,corner),(corner,p))
                            if all(scene.segment_reason(a,b) is None for a,b in legs):
                                out[n]=sum(leg_cost(a,b,h_idx) for a,b in legs);break
                return out
            src={(c[0],c[1],0):v for c,v in attach(start,0,False).items()}
            ends={}
            for h in range(4):          # 终点航向不约束
                ends.update({(c[0],c[1],h):v for c,v in attach(goal,h,True).items()})
            dist=dict(src);q=[(d,s) for s,d in src.items()];heapq.heapify(q);best=math.inf
            # 直线可达时 plan() 走 L 形快路径（同朝向、两端点直连）
            for corner in ((goal[0],start[1]),(start[0],goal[1])):
                if (scene.segment_reason(start,corner) is None and
                        scene.segment_reason(corner,goal) is None):
                    best=min(best,leg_cost(start,corner,0)+leg_cost(corner,goal,0));break
            while q:
                d,s=heapq.heappop(q)
                if d!=dist[s] or d>=best: continue
                if s in ends: best=min(best,d+ends[s])
                for di,dj in ((-1,0),(1,0),(0,-1),(0,1)):
                    n=(s[0]+di*100,s[1]+dj*100,s[2])
                    if (n[0],n[1]) not in node_set or scene.segment_reason((s[0],s[1]),(n[0],n[1])):
                        continue
                    nd=d+100.0*COST[act_of(s[2],di,dj)]
                    if nd<dist.get(n,math.inf): dist[n]=nd;heapq.heappush(q,(nd,n))
                for dh in (1,2,3):
                    n=(s[0],s[1],(s[2]+dh)%4)
                    nd=d+turn90(s[2],n[2])*TURN
                    if nd<dist.get(n,math.inf): dist[n]=nd;heapq.heappush(q,(nd,n))
            with self.subTest(scene=case):
                self.assertEqual(r['ok'],math.isfinite(best))
                if r['ok']: self.assertAlmostEqual(r['search_cost'],best,places=6)


class UiLogicTests(unittest.TestCase):
    def setUp(self): self.w=make_window()
    def tearDown(self): close_window(self.w)
    def plan(self, target=(330,1200)):
        self.w.sim.make_frame(0)
        r=self.w.plan_to(*target)
        self.w.sim.make_frame(0)      # 规划耗时可能超过 350ms 定位闸门，重新喂帧
        self.assertIsNotNone(r); self.assertTrue(r['ok'],r)
        return r
    def start(self,target=(2250,1600)):
        # 连续Trajectory跟踪要求初始车头对齐切线；旧固定航向横移不再是跟踪模式。
        angle=math.degrees(math.atan2(target[1]-2250,target[0]-2250))
        self.w.sim.zval=180+angle
        self.plan(target); self.w._toggle_follow(); self.assertIsNotNone(self.w.follow)
        return self.w.planned_result

    def test_strafe_limit_check_reaches_the_planner(self):
        """勾上「限制长距离横移」后上限必须真的进规划器；调到 0 = 完全禁横移，如实报错。"""
        self.w.sim.make_frame(0)
        self.w.strafe_limit_check.setChecked(True)
        self.w.strafe_limit_spin.setValue(50.0)          # 50cm
        r = self.w.plan_to(330, 1200)
        self.w.sim.make_frame(0)
        self.assertIsNotNone(r)
        self.assertTrue(r['ok'], r['reason'])
        self.assertEqual(r['strafe_run_limit_mm'], 500.0)
        self.assertLessEqual(r['max_strafe_run_mm'], 500.0 + 1e-6)
        self.assertIn('单次最长连续横移', self.w.plan_info.text)
        # 上限 0：启停区角落离两墙各 150mm，横移和原地转向都不成立，必须明说
        self.w.strafe_limit_spin.setValue(0.0)
        bad = self.w.plan_to(330, 1200)
        self.w.sim.make_frame(0)
        self.assertIsNotNone(bad)
        self.assertFalse(bad['ok'])
        self.assertEqual(bad['code'], 'NO_CONNECTION')
        self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())

    def test_plan_does_not_emit_serial_commands(self):
        self.plan(); self.assertTrue(self.w.line_q.empty()); self.assertTrue(self.w.urgent_q.empty())

    def test_simulation_stays_gated_from_real_serial(self):
        self.plan(); self.w.worker=object(); self.w._toggle_follow()
        self.assertIsNone(self.w.follow);self.assertTrue(self.w.line_q.empty());self.w.worker=None

    def test_unsafe_goal_cannot_start_simulation(self):
        r=self.w.plan_to(1200,2200)
        self.assertFalse(r['ok']);self.w._toggle_follow();self.assertIsNone(self.w.follow)

    def test_cannot_start_unverified_injected_point_list(self):
        self.w.planned_points=[(2250,2250),(1200,2200)]
        self.w._toggle_follow();self.assertIsNone(self.w.follow)

    def test_stop_button_stops_current_goal_not_only_timer(self):
        self.start(); advance(self.w,20)
        self.w._toggle_follow(); snap=self.w.sim.navigation_snapshot(); before=snap['hold']
        self.assertIsNone(snap['goto']);advance(self.w,50)
        self.assertEqual(self.w.sim.navigation_snapshot()['hold'],before)

    def test_clear_path_stops_current_goal(self):
        self.start();self.w._clear_path();self.assertIsNone(self.w.follow)
        self.assertIsNone(self.w.sim.navigation_snapshot()['goto'])

    def test_timeout_stops_current_goal(self):
        self.start();self.w.follow['t0']=time.monotonic()-26
        self.w._follow_step();self.assertIsNone(self.w.follow);self.assertIsNone(self.w.sim.goto)

    def test_old_zero_errors_not_current_goal_completion(self):
        self.start();self.w.latest=(0.0,)*24
        self.assertFalse(self.w._at_final_stop())
        for _ in range(10):self.w._follow_step()
        self.assertIsNotNone(self.w.follow);self.assertEqual(self.w.follow['progress_s_mm'],0)

    def test_same_frame_cannot_accumulate_settle(self):
        self.start((2150,2250))
        for _ in range(200):
            self.w.sim.make_frame(0.1)
            if self.w.sim.navigation_snapshot()['settled_frames']==1: break
        for _ in range(10): self.w._follow_step()
        self.assertIsNotNone(self.w.follow);self.assertEqual(self.w.follow['settled'],1)
        advance(self.w,10);self.assertIsNone(self.w.follow)

    def test_rejected_target_invalidates_old_plan(self):
        self.plan();self.w.plan_to(700,700);self.assertEqual(self.w.planned_points,[])
        self.assertIsNone(self.w.planned_result)

    def test_mapping_change_invalidates_plan(self):
        self.plan();self.w.map_theta_spin.v=90;self.w._apply_map_mapping()
        self.assertEqual(self.w.planned_points,[])

    def test_replan_cancels_old_snapshot(self):
        self.start();self.plan((330,1200))
        self.assertIsNone(self.w.follow);self.assertIsNone(self.w.sim.goto)

    def test_zero_offset_wheel_changes_invalidate_navigation(self):
        for cmd in ('STOP','ZERO','WHEELOFF','OPSOFFSET=60,-50','GOTO=0,10,0','MANUAL=1,0,0'):
            with self.subTest(cmd=cmd):
                w=make_window()
                try:
                    w.sim.zval=90;w.sim.make_frame(0)
                    w.plan_to(2250,1600);w._toggle_follow();self.assertIsNotNone(w.follow)
                    w.send_line(cmd);self.assertIsNone(w.follow);self.assertIsNone(w.sim.goto)
                    self.assertEqual(w.planned_points,[])
                finally:close_window(w)

    def test_changed_map_without_signal_is_detected_before_start(self):
        self.plan();self.w.nav_map['map_version']=99;self.w._toggle_follow()
        self.assertIsNone(self.w.follow)

    def test_wheel_disabled_cannot_start(self):
        self.plan();self.w.sim.handle_line('WHEELOFF');self.w._toggle_follow()
        self.assertIsNone(self.w.follow)

    def test_actual_start_drift_requires_replan(self):
        self.plan();self.w.sim.hold=(100,0);self.w._toggle_follow()
        self.assertIsNone(self.w.follow);self.assertEqual(self.w.planned_points,[])

    def test_old_goal_epoch_cannot_restart(self):
        self.start();old=self.w.follow['epoch'];self.w._stop_follow()
        with self.assertRaises(ValueError):
            self.w.sim.submit_navigation_goal(old,(100,0,0),lambda a,b:None)

    def test_stale_sim_pose_stops(self):
        self.start();self.w.sim.last_frame_monotonic=time.monotonic()-1
        self.w._follow_step();self.assertIsNone(self.w.follow);self.assertIsNone(self.w.sim.goto)

    def test_goal_yaw_enters_planning_context_without_implicit_rotation(self):
        self.w.map_yaw_combo.v=0
        before=self.w.sim.hold,self.w.sim.zval
        ctx=self.w._prepare_plan(330,1200)
        self.assertIsNotNone(ctx)
        expected=self.w._layout_yaw_for(self.w._field_to_body_yaw(0))
        self.assertAlmostEqual(ctx['kwargs']['goal_heading_deg'],expected)
        self.assertEqual((self.w.sim.hold,self.w.sim.zval),before)
        self.assertTrue(self.w.line_q.empty())

    def test_route_executes_in_real_simulator_with_full_body_checks(self):
        result=self.start(target=(2250,1900));ctx=self.w._planned_context;scene=self.w._scene_for_context(ctx)
        self.assertEqual(result['turn_count'],0)
        self.assertIsNotNone(self.w.follow)
        previous=(2250,2250)
        with patch.object(core.random,'gauss',return_value=0):
            for _ in range(2000):
                advance(self.w)
                current=self.w._ops_to_field(*self.w.sim.navigation_snapshot()['hold'])
                self.assertIsNone(scene.segment_reason(previous,current))
                previous=current
                if self.w.follow is None:break
        self.assertIsNone(self.w.follow)
        self.assertLess(math.dist(previous,(2250,1900)),1)
        self.assertTrue(self.w.line_q.empty());self.assertTrue(self.w.urgent_q.empty())

    def test_fallback_turn_route_is_refused_by_continuous_tracker(self):
        """保留原地转向的fallback不能伪装成连续Trajectory。"""
        result=self.plan()                      # 默认目标 (330,1200) 会规划出转向
        self.assertGreater(result['turn_count'],0)
        self.w._toggle_follow()
        self.assertIsNone(self.w.follow)
        self.assertIn("转向", self.w.map_status.text)

    def test_old_background_result_is_ignored(self):
        context=self.w._prepare_plan(330,1200)
        result=core.plan_path(context['start'],context['goal'],**context['kwargs'])
        self.w._clear_path()
        self.assertIsNone(self.w._finish_plan(result,context));self.assertEqual(self.w.planned_points,[])

    def test_async_request_then_poll(self):
        self.w._request_plan(330,1200)
        deadline=time.monotonic()+10  # public optimizer budget is 8s
        while self.w._plan_future is not None and time.monotonic()<deadline:
            self.w._poll_plan();time.sleep(0.005)
        self.assertIsNone(self.w._plan_future)
        self.assertTrue(self.w.planned_result['ok'])
        self.assertTrue(self.w.line_q.empty())

    def test_cancel_pending_background_search(self):
        self.w._request_plan(330,1200); self.w._clear_path(); self.w._poll_plan()
        self.assertEqual(self.w.planned_points,[]);self.assertIsNone(self.w.planned_result)

    def test_actual_motion_guard_cancels_collision(self):
        epoch=self.w.sim.begin_navigation()
        count=[0]
        def guard(a,b):
            count[0]+=1
            return None if count[0]==1 else 'new obstruction'
        self.w.sim.submit_navigation_goal(epoch,(100,0,0),guard)
        before=self.w.sim.hold;self.w.sim.make_frame(1)
        self.assertEqual(self.w.sim.hold,before);self.assertIsNone(self.w.sim.goto)
        self.assertIn('new obstruction',self.w.sim.navigation_snapshot()['fault'])

    def test_cancel_drops_simulators_own_pending_goal(self):
        sim=self.w.sim
        sim.line_q.put("GOTO=100,0,0")
        sim.cancel_navigation()
        sim._service_commands()
        self.assertIsNone(sim.goto)
        before=sim.hold;sim.make_frame(1)
        self.assertEqual(sim.hold,before)

    def test_urgent_stop_prevents_queued_goal_replay(self):
        sim=self.w.sim
        sim.line_q.put("GOTO=100,0,0")
        sim.urgent_q.put("STOP")
        sim._service_commands()
        self.assertIsNone(sim.goto)
        self.assertTrue(sim.line_q.empty())

    def test_mapping_roundtrip(self):
        for angle in (0,37,90,-180):
            self.w.map_theta=angle;self.w.map_ox=125;self.w.map_oy=-300
            p=self.w._field_to_ops(*self.w._ops_to_field(-400,230))
            self.assertAlmostEqual(p[0],-400);self.assertAlmostEqual(p[1],230)

    def test_queue_cancel_discards_only_motion(self):
        q=core.CommandQueue()
        for cmd in ('KPX=3','GOTO=0,100,0','MANUAL=1,0,0','GET XVMAX'):q.put(cmd)
        core.discard_motion_commands(q)
        self.assertEqual(q.get_nowait(),'KPX=3');self.assertEqual(q.get_nowait(),'GET XVMAX')
        self.assertTrue(q.empty())

class MapDataComplianceTests(unittest.TestCase):
    """地图数据必须与官方场地尺寸图一致，功能区接近点对任意航向都停得下。

    这两条守卫针对一次真实故障：快速目标曾写死 (1200,2200)，离原料区圆盘表面
    只有 90mm，而 280×260 车体轴向要 140mm、斜向要 205mm，于是"点击原料区 →
    规划失败"。数据必须由几何现算，而不是抄一个看着差不多的坐标。
    """
    MAP = nav.load_map(Path(__file__).resolve().parents[1] / 'navigation_map.json')

    def rects_named(self, name):
        return [r for r in self.MAP['rects'] if r[4] == name]

    def test_map_matches_rulebook_dimensions(self):
        rb = self.MAP['rulebook']
        x0, y0, x1, y1 = self.MAP['bounds']
        self.assertAlmostEqual(x1 - x0, rb['field_size_mm'][0])
        self.assertAlmostEqual(y1 - y0, rb['field_size_mm'][1])
        bw, bh = rb['material_block_mm']
        blocks = self.rects_named('中央物料区')
        self.assertEqual(len(blocks), 4)
        for xa, ya, xb, yb, _ in blocks:
            self.assertAlmostEqual(xb - xa, bw)
            self.assertAlmostEqual(yb - ya, bh)
        xs = sorted({r[0] for r in blocks} | {r[2] for r in blocks})
        gaps = [round(b - a, 6) for a, b in zip(xs, xs[1:])]
        self.assertIn(round(rb['material_corridor_mm'], 6), gaps)
        self.assertAlmostEqual(min(r[0] for r in blocks) - x0,
                               x1 - max(r[2] for r in blocks))       # 对称 ⇒ 外边距相等
        (sx0, sy0, sx1, sy1, _), = self.rects_named('暂存区设备')
        self.assertAlmostEqual(sx1 - sx0, rb['storage_rect_mm'][0])
        self.assertAlmostEqual(sy1 - sy0, rb['storage_rect_mm'][1])
        (rx0, ry0, rx1, ry1, _), = self.rects_named('粗加工区设备')
        self.assertAlmostEqual(rx1 - rx0, rb['rough_rect_mm'][0])
        self.assertAlmostEqual(ry1 - ry0, rb['rough_rect_mm'][1])
        zw, zh = rb['start_zone_mm']
        for cx, cy in core.ZONE_CENTER.values():
            self.assertAlmostEqual(cx, x1 - zw / 2)                  # 贴右边缘
            self.assertTrue(abs(cy - (y1 - zh / 2)) < 1e-6 or abs(cy - (y0 + zh / 2)) < 1e-6)

    def test_turntable_declared_position_and_radius(self):
        rb = self.MAP['rulebook']
        (cx, cy, r, name), = [c for c in self.MAP['circles'] if c[3] == '原料区圆盘']
        x0, y0, x1, y1 = self.MAP['bounds']
        self.assertAlmostEqual(cy, y1)                    # 图纸：圆心压在场地边界上
        lo, hi = rb['raw_turntable_center_from_right_mm']
        self.assertLessEqual(lo - 1e-6, x1 - cx)
        self.assertLessEqual(x1 - cx, hi + 1e-6)          # 位置必须落在规则允许区间内
        declared = rb.get('raw_turntable_radius_mm')
        if declared is None:
            self.assertFalse(self.MAP['geometry_verified'],
                             '半径未实测时不得声称 geometry_verified')
        else:
            self.assertAlmostEqual(r, declared)
        self.assertGreater(r, 0)

    def test_quick_targets_are_legal_for_every_heading(self):
        for name, anchor, outward in core.QUICK_ANCHORS:
            for pad in (10.0, 30.0, 60.0):
                for yaw in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
                    res = core.approach_target(
                        anchor, outward, margin=pad,
                        footprint=(core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, yaw),
                        rects=self.MAP['rects'], circles=self.MAP['circles'],
                        bounds=self.MAP['bounds'],
                        drivable_polygons=self.MAP['drivable_polygons'])
                    tag = '%s pad=%.0f yaw=%.0f' % (name, pad, yaw)
                    self.assertTrue(res['ok'], tag + '：' + str(res['reason']))
                    scene = nav.CollisionScene(self.MAP['rects'], self.MAP['circles'],
                                               self.MAP['bounds'], pad,
                                               (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, yaw),
                                               self.MAP['drivable_polygons'])
                    self.assertIsNone(scene.segment_reason(res['point'], res['point']),
                                      tag + '：现算的点被同一模型判为碰撞')
                    self.assertGreaterEqual(res['distance'] + 1e-6, res['required'])
                    self.assertGreater(math.dist(res['point'], anchor), 0.0)

    def test_hardcoded_quick_target_removed_and_still_illegal(self):
        """旧写死坐标必须已删除，且在同一模型下仍判非法（防止校验被放松）。"""
        self.assertFalse(hasattr(core, 'QUICK_TARGETS'), '写死坐标表必须换成锚点+现算')
        scene = nav.CollisionScene(self.MAP['rects'], self.MAP['circles'], self.MAP['bounds'],
                                   core.NAV_MARGIN_MM,
                                   (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, -180.0),
                                   self.MAP['drivable_polygons'])
        self.assertIsNotNone(scene.segment_reason((1200.0, 2200.0), (1200.0, 2200.0)))


class SimulatedObstacleTests(unittest.TestCase):
    """模拟障碍（φ50×100mm 圆，最多 4 个）：放置合法性、随机撒点、规划必须绕开。

    障碍是运行时的演示物体，不写进地图数据，但与地图障碍同格式并入同一个规划场景，
    所以"放得下"与"绕得开"用的是同一套 CollisionScene。
    """
    MAP = nav.load_map(Path(__file__).resolve().parents[1] / 'navigation_map.json')

    def scene(self, pad=10.0, yaw=-180.0, extra_circles=()):
        return nav.CollisionScene(self.MAP['rects'],
                                  list(self.MAP['circles']) + list(extra_circles),
                                  self.MAP['bounds'], pad,
                                  (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, yaw),
                                  self.MAP['drivable_polygons'])

    def test_circle_table_is_phi50(self):
        (x, y, r, name), = core.sim_obstacle_circles([(100.0, 200.0)])
        self.assertEqual((x, y, r), (100.0, 200.0, 25.0))
        self.assertEqual(name, core.SIM_OBSTACLE_NAME)
        self.assertEqual(core.SIM_OBSTACLE_MAX, 4)

    def test_placement_rejects_conflicts_and_edge(self):
        kw = dict(rects=self.MAP['rects'], circles=self.MAP['circles'], bounds=self.MAP['bounds'])
        self.assertEqual(core.obstacle_placement_blocked(1200.0, 2400.0, **kw), '原料区圆盘')
        self.assertEqual(core.obstacle_placement_blocked(700.0, 700.0, **kw), '中央物料区')
        self.assertIsNone(core.obstacle_placement_blocked(1200.0, 2050.0, **kw))
        # φ50 + 20mm 余量 ⇒ 圆心至少离边 45mm（取顶边 x=600：避开原料区圆盘与物料区）
        self.assertEqual(core.obstacle_placement_blocked(600.0, 2360.0, **kw), '场地边缘')
        self.assertIsNone(core.obstacle_placement_blocked(600.0, 2350.0, **kw))
        self.assertEqual(core.obstacle_placement_blocked(600.0, 40.0, **kw), '场地边缘')
        # 已有障碍之间也要让开
        placed = core.sim_obstacle_circles([(1200.0, 2050.0)])
        self.assertEqual(core.obstacle_placement_blocked(
            1200.0, 2095.0, rects=self.MAP['rects'],
            circles=list(self.MAP['circles']) + placed, bounds=self.MAP['bounds']),
            core.SIM_OBSTACLE_NAME)
        self.assertIsNone(core.obstacle_placement_blocked(
            1200.0, 2125.0, rects=self.MAP['rects'],
            circles=list(self.MAP['circles']) + placed, bounds=self.MAP['bounds']))

    def test_random_placement_is_legal_and_reports_shortfall(self):
        pts, why = core.random_obstacle_points(4, rects=self.MAP['rects'],
                                              circles=self.MAP['circles'],
                                              bounds=self.MAP['bounds'],
                                              rng=random.Random(7))
        self.assertEqual(len(pts), 4)
        self.assertEqual(why, "")
        for i, (x, y) in enumerate(pts):
            self.assertIsNone(core.obstacle_placement_blocked(
                x, y, rects=self.MAP['rects'],
                circles=list(self.MAP['circles']) + core.sim_obstacle_circles(pts[:i]),
                bounds=self.MAP['bounds']), (x, y))
        pair_min = min(math.dist(a, b) for i, a in enumerate(pts) for b in pts[i + 1:])
        self.assertGreater(pair_min, 2 * core.SIM_OBSTACLE_R_MM + 20.0 - 1e-6)
        # 放不下时如实少放并说明，不返回非法点
        tiny, why2 = core.random_obstacle_points(9, rects=[], circles=[],
                                                bounds=(0.0, 0.0, 200.0, 200.0),
                                                rng=random.Random(3))
        self.assertLess(len(tiny), 9)
        self.assertIn("找不到合法位置", why2)

    def test_random_placement_keeps_clear_of_given_area(self):
        pts, why = core.random_obstacle_points(
            3, rects=self.MAP['rects'], circles=self.MAP['circles'], bounds=self.MAP['bounds'],
            keep_clear=[(1200.0, 1200.0, 400.0)], rng=random.Random(11))
        self.assertTrue(pts)
        self.assertEqual(why, "")
        for x, y in pts:
            self.assertGreaterEqual(math.hypot(x - 1200.0, y - 1200.0), 400.0)

    def test_plan_must_avoid_simulated_obstacle(self):
        start, goal = (2250.0, 2250.0), (330.0, 1200.0)
        feet = (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, -180.0)
        heading = -180.0
        common = dict(grid=100.0, pad=10.0, footprint=feet, start_heading_deg=heading,
                      rects=self.MAP['rects'], bounds=self.MAP['bounds'],
                      drivable_polygons=self.MAP['drivable_polygons'])
        base = core.plan_path(start, goal, circles=self.MAP['circles'], **common)
        self.assertTrue(base['ok'], base['reason'])
        pts = base['points']
        i = len(pts) // 2 - 1
        mx = (pts[i][0] + pts[i + 1][0]) / 2.0
        my = (pts[i][1] + pts[i + 1][1]) / 2.0
        self.assertIsNone(core.obstacle_placement_blocked(
            mx, my, rects=self.MAP['rects'], circles=self.MAP['circles'], bounds=self.MAP['bounds']))
        obstacle = core.sim_obstacle_circles([(mx, my)])
        detour = core.plan_path(start, goal,
                                circles=list(self.MAP['circles']) + obstacle, **common)
        self.assertTrue(detour['ok'], detour['reason'])
        # 每段用它自己的航向复查（路径可能含原地转向，单一航向的 scene 代表不了）
        for row in detour['steps']:
            if row['kind'] != 'MOVE':
                continue
            self.assertIsNone(self.scene(yaw=row['heading_deg'],
                                         extra_circles=obstacle).segment_reason(
                (row['x'], row['y']), (row['to_x'], row['to_y'])), row)
        # 车心线到障碍表面留有余量，且路径确实改道
        self.assertGreater(core.path_clearance(detour['points'], [], obstacle), 0.0)
        self.assertNotEqual(detour['points'], base['points'])


class ManhattanPlanTests(unittest.TestCase):
    """严格 4 邻域曼哈顿规划：输出折线每段必须 dx==0 或 dy==0（含首末连接段）。"""
    MAP = nav.load_map(Path(__file__).resolve().parents[1] / 'navigation_map.json')

    def plan(self, start, goal, extra_circles=(), grid=100.0, pad=10.0):
        return core.plan_path(start, goal, grid=grid, pad=pad,
                              footprint=(core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, -180.0),
                              start_heading_deg=-180.0,
                              rects=self.MAP['rects'],
                              circles=list(self.MAP['circles']) + list(extra_circles),
                              bounds=self.MAP['bounds'],
                              drivable_polygons=self.MAP['drivable_polygons'])

    def assert_manhattan(self, start, goal, pts, tag):
        self.assertEqual(tuple(pts[0]), tuple(start), tag + ' 起点被改动')
        self.assertEqual(tuple(pts[-1]), tuple(goal), tag + ' 终点被改动')
        for a, b in zip(pts, pts[1:]):
            self.assertTrue(abs(a[0]-b[0]) < 1e-9 or abs(a[1]-b[1]) < 1e-9,
                            '%s 出现斜段 %s→%s' % (tag, a, b))

    def test_every_segment_is_axis_aligned(self):
        """覆盖：同侧直达、需绕行、端点不在网格上、同列、长对角。"""
        cases = [((2250.0, 2250.0), (1200.0, 2050.0)),
                 ((2250.0, 2250.0), (330.0, 1200.0)),
                 ((2250.0, 2250.0), (1200.0, 330.0)),
                 ((2249.0, 2248.0), (329.0, 1201.0)),      # 两个端点都不在网格上
                 ((2250.0, 2250.0), (2250.0, 1600.0)),     # 同列
                 ((330.0, 330.0), (2100.0, 2100.0))]       # 长对角 ⇒ 必须折线
        for start, goal in cases:
            r = self.plan(start, goal)
            self.assertTrue(r['ok'], '%s→%s: %s' % (start, goal, r['reason']))
            self.assert_manhattan(start, goal, r['points'], 'points')
            self.assert_manhattan(start, goal, r['raw'], 'raw')
            # 每段轴对齐 ⇒ 折线欧氏长度 == 几何长度；加权代价含动作系数与转向罚，
            # 只会 ≥ 几何长度，且必须与逐步台账自洽
            self.assertAlmostEqual(r['length'], core.path_length(r['raw']), places=6)
            self.assertGreaterEqual(r['search_cost'] + 1e-6, r['length'])
            self.assertAlmostEqual(sum(s['cost_mm'] for s in r['steps']),
                                   r['search_cost'], places=6)

    def test_obstacles_keep_right_angles_and_clearance(self):
        for extra in ((), core.sim_obstacle_circles([(1200.0, 1200.0)]),
                      core.sim_obstacle_circles([(1200.0, 1200.0), (2000.0, 1200.0)])):
            r = self.plan((2250.0, 2250.0), (330.0, 1200.0), extra_circles=extra)
            self.assertTrue(r['ok'], r['reason'])
            self.assert_manhattan((2250.0, 2250.0), (330.0, 1200.0), r['points'], 'points')
            self.assert_manhattan((2250.0, 2250.0), (330.0, 1200.0), r['raw'], 'raw')
            if extra:
                self.assertGreater(core.path_clearance(r['points'], [], extra), 0.0)

    def test_simplification_keeps_right_angles_and_never_lengthens(self):
        """共线合并只删同方向中间点：直角保留、长度不变、航点数不增加。"""
        r = self.plan((2250.0, 2250.0), (330.0, 1200.0))
        self.assertGreater(len(r['raw']), 2)
        self.assertLessEqual(len(r['points']), len(r['raw']))
        self.assertAlmostEqual(core.path_length(r['points']),
                               core.path_length(r['raw']), places=6)
        turns = 0
        for a, b, c in zip(r['points'], r['points'][1:], r['points'][2:]):
            d1 = (b[0]-a[0], b[1]-a[1])
            d2 = (c[0]-b[0], c[1]-b[1])
            if (d1[0]*d2[1] - d1[1]*d2[0]) != 0.0:
                turns += 1
        self.assertGreaterEqual(turns, 1, '折线至少要有一次直角，不能被拉成一条斜线')


class HeadingPlanTests(unittest.TestCase):
    """状态=(格, 航向) 的动作代价、台账输出（x,y,heading,action）与横移罚单调性。"""
    MAP = nav.load_map(Path(__file__).resolve().parents[1] / 'navigation_map.json')
    START = (2250.0, 2250.0)
    GOAL = (330.0, 1200.0)
    FEET = (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, 0.0)
    ACTIONS = {"START", "FORWARD", "BACKWARD", "STRAFE",
               "TURN_LEFT", "TURN_RIGHT", "TURN_AROUND"}

    def plan(self, **kw):
        base = dict(grid=100.0, pad=10.0, footprint=self.FEET, start_heading_deg=0.0,
                    rects=self.MAP['rects'], circles=self.MAP['circles'],
                    bounds=self.MAP['bounds'],
                    drivable_polygons=self.MAP['drivable_polygons'])
        base.update(kw)
        return core.plan_path(self.START, self.GOAL, **base)

    def test_steps_report_x_y_heading_action(self):
        """回溯必须给出 x,y,heading,action，且动作与"当时航向"自洽；没有圆弧。"""
        r = self.plan()
        self.assertTrue(r['ok'], r['reason'])
        self.assertTrue({s['action'] for s in r['steps']} <= self.ACTIONS)
        for s in r['steps']:
            for key in ("x", "y", "to_x", "to_y", "heading_deg", "action", "kind",
                        "distance_mm", "cost_mm"):
                self.assertIn(key, s)
            self.assertIn(s['heading_deg'], (0.0, 90.0, 180.0, 270.0))
            if s['kind'] != 'MOVE':
                self.assertEqual(s['distance_mm'], 0.0)      # 转向/起点零位移
                continue
            dx, dy = s['to_x'] - s['x'], s['to_y'] - s['y']
            self.assertTrue(abs(dx) < 1e-9 or abs(dy) < 1e-9, s)   # 轴对齐：没有圆弧
            fx, fy = nav.heading_vector(nav.heading_index(s['heading_deg']))
            dot = (1 if dx > 1e-9 else -1 if dx < -1e-9 else 0) * fx + \
                  (1 if dy > 1e-9 else -1 if dy < -1e-9 else 0) * fy
            want = "FORWARD" if dot > 0.5 else ("BACKWARD" if dot < -0.5 else "STRAFE")
            self.assertEqual(s['action'], want, s)
        self.assertAlmostEqual(sum(s['cost_mm'] for s in r['steps']), r['search_cost'], places=6)
        self.assertAlmostEqual(r['trace_cost'], r['search_cost'], places=6)

    def test_raised_lateral_cost_never_increases_lateral_distance(self):
        """提高横移系数后，同一对起终点的横移总距离不会增加（需求 6）。"""
        base = self.plan(cost_lateral=1.0)
        self.assertTrue(base['ok'], base['reason'])
        self.assertGreater(base['lateral_mm'], 0.0)   # 该场景确实用了横移，否则本测试空转
        prev = base['lateral_mm']
        for lat in (1.4, 1.8, 3.0, 8.0):
            r = self.plan(cost_lateral=lat)
            self.assertTrue(r['ok'], r['reason'])
            self.assertLessEqual(r['lateral_mm'], prev + 1e-6,
                                 "lateral=%.1f 时横移距离反而变多" % lat)
            prev = r['lateral_mm']

    def test_cost_weights_are_applied_per_action(self):
        r = self.plan()
        for s in r['steps']:
            if s['kind'] != 'MOVE':
                continue
            factor = {"FORWARD": 1.0, "BACKWARD": 1.15, "STRAFE": 1.8}[s['action']]
            self.assertAlmostEqual(s['cost_mm'], s['distance_mm'] * factor, places=6)
        self.assertAlmostEqual(r['turn_cost_mm'], sum(s['cost_mm'] for s in r['steps']
                                                     if s['kind'] == 'TURN'), places=6)

    def test_forbidding_strafe_turns_instead(self):
        """禁横移 + 有转向空间时改用原地转向；操作区多边形可放开横移。"""
        kw = dict(grid=100.0, pad=10.0, footprint=self.FEET, start_heading_deg=0.0,
                  rects=self.MAP['rects'], circles=self.MAP['circles'],
                  bounds=self.MAP['bounds'],
                  drivable_polygons=self.MAP['drivable_polygons'])
        r = core.plan_path((1200.0, 2050.0), (1200.0, 2100.0), allow_strafe=False, **kw)
        self.assertTrue(r['ok'], r['reason'])
        acts = [s['action'] for s in r['steps']]
        self.assertNotIn("STRAFE", acts)
        self.assertTrue(any(a.startswith("TURN") for a in acts), acts)
        whole = [[[0, 0], [2400, 0], [2400, 2400], [0, 2400]]]
        r2 = core.plan_path((1200.0, 2050.0), (1200.0, 2100.0), allow_strafe=False,
                            strafe_polygons=whole, **kw)
        self.assertTrue(r2['ok'], r2['reason'])
        self.assertTrue(any(s['action'] == "STRAFE" for s in r2['steps']))

    def test_start_heading_must_match_footprint(self):
        """车体朝向与起始航向不一致必须显式拒绝，不能静默换朝向求解。"""
        bad = core.plan_path(self.START, self.GOAL, grid=100.0, pad=10.0,
                             footprint=(core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, 90.0),
                             start_heading_deg=0.0, rects=self.MAP['rects'],
                             circles=self.MAP['circles'], bounds=self.MAP['bounds'],
                             drivable_polygons=self.MAP['drivable_polygons'])
        self.assertFalse(bad['ok'])
        self.assertEqual(bad['code'], 'INVALID_INPUT')


class StrafeRunLimitTests(unittest.TestCase):
    """「禁止长距离横移」= 单次**连续**横移上限；不是完全禁横移，也不是全程总量。"""

    MAP = nav.load_map(Path(__file__).resolve().parents[1] / 'navigation_map.json')
    FEET = (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, 0.0)
    POCKET = (2250.0, 2250.0)        # 启停区1中心：离两墙各 150mm，原地转不动
    SIDE = (2250.0, 800.0)           # 正侧方 1450mm：只能横移过去

    def plan(self, start, goal, **kw):
        base = dict(grid=100.0, pad=10.0, footprint=self.FEET, start_heading_deg=0.0,
                    goal_heading_deg=0.0, rects=self.MAP['rects'],
                    circles=self.MAP['circles'], bounds=self.MAP['bounds'],
                    drivable_polygons=self.MAP['drivable_polygons'])
        base.update(kw)
        return core.plan_path(start, goal, **base)

    def test_short_strafe_lets_the_car_leave_the_start_pocket(self):
        """出启停区角落要横移 150mm：上限 500mm 时照常可用，0 才是完全禁横移。"""
        r = self.plan(self.POCKET, (1200.0, 2100.0), strafe_run_limit_mm=500.0)
        self.assertTrue(r['ok'], r['reason'])
        self.assertIn('STRAFE', [s['action'] for s in r['steps']])
        self.assertGreater(r['lateral_mm'], 0.0)
        self.assertLessEqual(r['max_strafe_run_mm'], 500.0 + 1e-6)
        # 上限 0 = 完全禁横移：角落里既不能横移也转不动，必须如实拒绝
        zero = self.plan(self.POCKET, (1200.0, 2100.0), strafe_run_limit_mm=0.0)
        self.assertFalse(zero['ok'])
        self.assertEqual(zero['code'], 'NO_CONNECTION')

    def test_limit_zero_matches_the_old_hard_forbid(self):
        """上限 0 与 allow_strafe=False 必须等价（同一约束的两种写法）。"""
        for start, goal in ((self.POCKET, (1200.0, 2100.0)),
                            ((1200.0, 2050.0), (1200.0, 2100.0))):
            a = self.plan(start, goal, strafe_run_limit_mm=0.0)
            b = self.plan(start, goal, allow_strafe=False)
            self.assertEqual((a['ok'], a['code'], round(a['length'], 6)),
                             (b['ok'], b['code'], round(b['length'], 6)))

    def test_long_continuous_strafe_is_cut_to_the_limit(self):
        """转向很贵时规划器本来会一口气横移 1450mm；加上限后必须切成一节一节。"""
        base = self.plan(self.POCKET, self.SIDE, turn_penalty_mm=5000.0)
        self.assertTrue(base['ok'], base['reason'])
        self.assertGreater(base['max_strafe_run_mm'], 500.0)     # 不限时确实有长横移
        prev_cost = base['search_cost']
        for limit in (500.0, 300.0):
            r = self.plan(self.POCKET, self.SIDE, turn_penalty_mm=5000.0,
                          strafe_run_limit_mm=limit)
            self.assertTrue(r['ok'], r['reason'])
            self.assertLessEqual(r['max_strafe_run_mm'], limit + 1e-6,
                                 "上限 %.0fmm 下仍然连续横移了 %.0fmm"
                                 % (limit, r['max_strafe_run_mm']))
            self.assertGreaterEqual(r['search_cost'] + 1e-6, prev_cost,
                                    "收紧上限不可能让最优代价变小")
            prev_cost = r['search_cost']

    def test_steps_ledger_carries_the_running_strafe_length(self):
        """台账每步的 strafe_run_mm 必须与动作自洽，且不超上限（非操作区）。"""
        r = self.plan(self.POCKET, self.SIDE, turn_penalty_mm=5000.0,
                      strafe_run_limit_mm=500.0)
        self.assertTrue(r['ok'], r['reason'])
        run = 0.0
        for s in r['steps']:
            run = run + s['distance_mm'] if s['action'] == 'STRAFE' else 0.0
            self.assertAlmostEqual(s['strafe_run_mm'], run, places=6, msg=str(s))
            self.assertLessEqual(run, 500.0 + 1e-6, str(s))
        self.assertAlmostEqual(r['max_strafe_run_mm'], max(s['strafe_run_mm']
                                                           for s in r['steps']), places=6)

    def test_operation_area_is_exempt_from_the_limit(self):
        """操作区（strafe_polygons）内不受上限约束，横移额度不占。"""
        whole = [[[0, 0], [2400, 0], [2400, 2400], [0, 2400]]]
        r = self.plan(self.POCKET, self.SIDE, turn_penalty_mm=5000.0,
                      strafe_run_limit_mm=500.0, strafe_polygons=whole)
        self.assertTrue(r['ok'], r['reason'])
        self.assertGreater(r['max_strafe_run_mm'], 500.0)
        # 上限 0（完全禁横移）时操作区也必须照样生效，否则区内连出库都做不到
        zero = self.plan(self.POCKET, self.SIDE, strafe_run_limit_mm=0.0,
                         strafe_polygons=whole)
        self.assertTrue(zero['ok'], zero['reason'])
        self.assertGreater(zero['max_strafe_run_mm'], 0.0)

    def test_raising_the_limit_never_increases_cost(self):
        """上限放宽＝可行集变大：最优代价单调不增，可行解不会反而消失。"""
        prev = None
        for limit in (150.0, 200.0, 300.0, 500.0, 1000.0, 5000.0, None):
            r = self.plan(self.POCKET, (330.0, 1200.0), strafe_run_limit_mm=limit)
            self.assertTrue(r['ok'], "上限 %s：%s" % (limit, r['reason']))
            self.assertLessEqual(r['max_strafe_run_mm'], (limit or math.inf) + 1e-6)
            if prev is not None:
                self.assertLessEqual(r['search_cost'], prev + 1e-6,
                                     "上限放宽后代价反而变大")
            prev = r['search_cost']

    def test_no_limit_keeps_the_previous_behaviour(self):
        """不设上限时结果与"给了个足够大的上限"完全一致（默认行为没变）。"""
        for start, goal in ((self.POCKET, (330.0, 1200.0)),
                            (self.POCKET, (1200.0, 2100.0))):
            a = self.plan(start, goal)
            b = self.plan(start, goal, strafe_run_limit_mm=100000.0)
            self.assertEqual((a['ok'], round(a['length'], 6), round(a['search_cost'], 6)),
                             (b['ok'], round(b['length'], 6), round(b['search_cost'], 6)))
            self.assertEqual(a['strafe_run_limit_mm'], None)


class PathAxesTests(unittest.TestCase):
    """Manhattan 共线压缩 + 90° 角点识别：只删共线节点，绝不生成斜直连。"""

    MAP = nav.load_map(Path(__file__).resolve().parents[1] / 'navigation_map.json')

    @staticmethod
    def axes(points, steps=None):
        return core.geometry_axes(points, steps)

    def test_line_has_two_segments_and_no_corner(self):
        segs, corners = self.axes([(0, 0), (300, 0), (900, 0)])
        self.assertEqual([(s.start, s.end) for s in segs],
                         [((0.0, 0.0), (300.0, 0.0)), ((300.0, 0.0), (900.0, 0.0))])
        self.assertEqual(corners, [])                      # 共线不成角
        self.assertEqual([s.axis for s in segs], ['x', 'x'])
        self.assertEqual([s.direction for s in segs], [1, 1])
        self.assertAlmostEqual(sum(s.length_mm for s in segs), 900.0)

    def test_l_shape_has_two_segments_and_one_left_corner(self):
        segs, corners = self.axes([(0, 0), (500, 0), (500, 400)])
        self.assertEqual(len(segs), 2)
        self.assertEqual(segs[0].axis, 'x')
        self.assertEqual((segs[1].axis, segs[1].direction), ('y', 1))
        self.assertEqual(len(corners), 1)
        c = corners[0]
        self.assertEqual(c.point, (500.0, 0.0))
        self.assertEqual(c.turn, 'LEFT')                   # +x 转 +y 是逆时针
        self.assertEqual(c.steps, 0)                       # 没给台账：角点不代表转过
        self.assertEqual(c.action, '')

    def test_z_shape_has_three_segments_and_two_opposite_corners(self):
        segs, corners = self.axes([(0, 0), (400, 0), (400, 300), (900, 300)])
        self.assertEqual([s.axis for s in segs], ['x', 'y', 'x'])
        self.assertEqual([s.direction for s in segs], [1, 1, 1])
        self.assertEqual([c.turn for c in corners], ['LEFT', 'RIGHT'])
        self.assertEqual([c.point for c in corners], [(400.0, 0.0), (400.0, 300.0)])

    def test_u_shape_keeps_axis_alignment(self):
        """∩ 形（右→上→左）是连续两次左转；∪ 形才是两次右转。"""
        segs, corners = self.axes([(0, 0), (600, 0), (600, 400), (0, 400)])
        self.assertEqual([s.axis for s in segs], ['x', 'y', 'x'])
        self.assertEqual([s.direction for s in segs], [1, 1, -1])
        self.assertEqual([c.turn for c in corners], ['LEFT', 'LEFT'])
        self.assertEqual(corners[-1].point, (600.0, 400.0))
        for s in segs:                                     # U 形横梁没有被拉成斜线
            self.assertTrue(s.start[0] == s.end[0] or s.start[1] == s.end[1])
        segs_down, corners_down = self.axes([(0, 400), (600, 400), (600, 0), (0, 0)])
        self.assertEqual([c.turn for c in corners_down], ['RIGHT', 'RIGHT'])

    def test_only_duplicate_points_are_dropped(self):
        """geometry_axes 的压缩只有"删重复点"这一条：共线中间点在这里**不**删。

        （共线合并是 simplify_collinear 的职责，规划走的是它；别把两条规则混成一条。）
        """
        segs, _ = self.axes([(0, 0), (0, 0), (100, 0), (200, 0)])
        self.assertEqual(len(segs), 2, '只应删掉重复点，共线合并归 simplify_collinear')
        self.assertEqual((segs[0].start, segs[0].end), ((0.0, 0.0), (100.0, 0.0)))

    def test_diagonal_segment_is_rejected(self):
        for pts in ([(0, 0), (100, 100)], [(0, 0), (100, 0), (200, 100)]):
            with self.assertRaises(ValueError) as ctx:
                self.axes(pts)
            self.assertIn('斜直连', str(ctx.exception))

    def test_corner_and_turn_are_not_the_same_thing(self):
        """几何角点未必有转向；原地转向也未必留角点。"""
        # ① 横移→后退：轴变了（角点），航向没变、台账里没有转向
        segs, corners = self.axes(
            [(0, 0), (0, 100), (300, 100)],
            steps=[{"x": 0, "y": 0, "to_x": 0, "to_y": 100,
                    "heading_deg": 0.0, "action": "STRAFE"},
                   {"x": 0, "y": 100, "to_x": 300, "to_y": 100,
                    "heading_deg": 0.0, "action": "BACKWARD"}])
        self.assertEqual(len(corners), 1)
        self.assertEqual(corners[0].heading_in_deg, corners[0].heading_out_deg)
        self.assertEqual((corners[0].steps, corners[0].action), (0, ''))
        self.assertEqual(segs[0].action, 'STRAFE')
        self.assertTrue(segs[0].reverse)                   # 沿 y 走、航向 0° ⇒ 横移
        # ② 同一直线上先原地转向再走：顶点被共线合并，没有角点，但台账里有转向
        segs, corners = self.axes(
            [(0, 0), (600, 0)],
            steps=[{"x": 0, "y": 0, "to_x": 0, "to_y": 0,
                    "heading_deg": 0.0, "action": "START"},
                   {"x": 0, "y": 0, "to_x": 0, "to_y": 0,
                    "heading_deg": 180.0, "action": "TURN_AROUND"},
                   {"x": 0, "y": 0, "to_x": 600, "to_y": 0,
                    "heading_deg": 180.0, "action": "BACKWARD"}])
        self.assertEqual(len(segs), 1)
        self.assertEqual(corners, [])
        self.assertEqual(segs[0].action, 'BACKWARD')
        self.assertTrue(segs[0].reverse)
        self.assertEqual(segs[0].heading_deg, 180.0)

    def test_real_plan_segments_keep_ledger_semantics(self):
        """真实规划：每段/每角都来自台账，折线严格轴对齐。"""
        r = core.plan_path((2250.0, 2250.0), (330.0, 1200.0), grid=100.0, pad=10.0,
                           footprint=(core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, 0.0),
                           start_heading_deg=0.0, goal_heading_deg=0.0,
                           rects=self.MAP['rects'], circles=self.MAP['circles'],
                           bounds=self.MAP['bounds'],
                           drivable_polygons=self.MAP['drivable_polygons'])
        self.assertTrue(r['ok'], r['reason'])
        self.assertTrue(r['axis_matched'])
        segs, corners = r['segments'], r['corners']
        self.assertEqual(len(segs), len(r['points']) - 1)
        self.assertAlmostEqual(sum(s.length_mm for s in segs), r['length'], places=6)
        ledger = {(round(s['x'], 2), round(s['y'], 2), round(s['to_x'], 2),
                   round(s['to_y'], 2)): s for s in r['steps'] if s['kind'] == 'MOVE'}
        for seg in segs:
            self.assertIn(seg.action, ('FORWARD', 'BACKWARD', 'STRAFE'), seg.as_dict())
            key = (round(seg.start[0], 2), round(seg.start[1], 2),
                   round(seg.end[0], 2), round(seg.end[1], 2))
            self.assertIn(key, ledger, seg.as_dict())
            self.assertEqual(seg.action, ledger[key]['action'])
            self.assertEqual(seg.heading_deg, ledger[key]['heading_deg'])
            self.assertEqual(seg.axis, 'x' if abs(seg.start[1] - seg.end[1]) <= 1e-9 else 'y')
        # 角点数 == 折线里方向改变的顶点数
        changes = 0
        for a, b, c in zip(r['points'], r['points'][1:], r['points'][2:]):
            d1 = (b[0] - a[0], b[1] - a[1])
            d2 = (c[0] - b[0], c[1] - b[1])
            if (d1[0] * d2[1] - d1[1] * d2[0]) != 0.0:
                changes += 1
        self.assertEqual(len(corners), changes)
        turns = {s['action']: s for s in r['steps'] if s['kind'] == 'TURN'}
        self.assertEqual(sum(1 for c in corners if c.action), r['turn_count'])
        for c in corners:
            self.assertIn(c.steps, (0, 1, 2))
            if c.action:
                self.assertEqual(c.point, (turns[c.action]['x'], turns[c.action]['y']))


if __name__=='__main__':unittest.main(verbosity=2)
