"""连续PC跟踪：实位置进度/100mm参考、终点停稳、运动扫掠与故障注入。"""
import copy
import math
import queue
import threading
import unittest
from unittest.mock import patch

import core
import navigation_planner as nav
from arc_smoothing import smooth_90_corners
from trajectory import generate_trajectory
from trajectory_tracking import TrajectoryTracker
from tests.test_arc_smoothing import scene, ledger, L
from tests.headless_support import make_window, close_window, advance


def prepared(points=L):
    geometry = scene()
    smoothed = smooth_90_corners(*ledger(points), geometry)
    result = generate_trajectory(smoothed['smoothed_primitives'], geometry)
    if result['trajectory_status'] != 'READY':
        raise AssertionError(result)
    return result, smoothed['smoothed_primitives'], geometry


def simulator(points=L, mapping=(0, 0, 0), validity=None):
    result, primitives, geometry = prepared(points)
    sim = core.Simulator(queue.Queue(), queue.Queue())
    sim.handle_line('ZERO')
    mx, my, angle = mapping
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    first = result['trajectory'][0]
    dx, dy = first['x_mm']-mx, first['y_mm']-my
    sim.hold = (c*dx-s*dy, s*dx+c*dy)
    sim.zval = 90-angle-first['field_yaw_deg']
    sim.make_frame(0)
    epoch = sim.begin_navigation()
    goal = sim.submit_navigation_trajectory(epoch, result['trajectory'], primitives, mapping, geometry, validity)
    return sim, result, primitives, geometry, epoch, goal


class MonotoneProgressTests(unittest.TestCase):
    def test_actual_projection_monotone_when_position_jitters_backward(self):
        r, primitives, _geometry = prepared([(500, 500), (1500, 500)])
        tracker = TrajectoryTracker(r['trajectory'], primitives)
        seen = []
        for x in (500, 510, 525, 520, 550, 546, 580):
            seen.append(tracker.update_progress(core.layout_to_field(x, 500)))
        self.assertEqual(seen, [0, 10, 25, 25, 50, 50, 80])

    def test_lookahead_is_one_hundred_mm_arclength_with_exact_arc_yaw(self):
        r, primitives, _geometry = prepared()
        tracker = TrajectoryTracker(r['trajectory'], primitives)
        for x in range(500, 1341, 10):
            tracker.update_progress(core.layout_to_field(x, 500))
        actual = (*core.layout_to_field(1340, 500), -90)
        ref, _v, _w = tracker.command(actual, 250, 120)
        self.assertEqual(tracker.progress, 840)
        self.assertEqual(ref['s_mm'], 940)
        self.assertEqual(ref['segment_type'], 'ARC')
        offset = .5  # 圆弧从s=880开始，60mm弧长/120mm半径。
        self.assertAlmostEqual(ref['x_mm'], 2250-(620-120*math.cos(offset)))
        self.assertAlmostEqual(ref['y_mm'], 2250-(1380+120*math.sin(offset)))
        self.assertAlmostEqual(ref['field_yaw_deg'], -90-math.degrees(offset))

    def test_closed_loop_start_cannot_jump_to_identical_endpoint(self):
        from tests.test_trajectory import arc
        primitives = [arc(angle=-90+i*90) for i in range(4)]
        result = generate_trajectory(primitives, scene())
        tracker = TrajectoryTracker(result['trajectory'], primitives)
        first = result['trajectory'][0]
        tracker.update_progress((first['x_mm'], first['y_mm']))
        self.assertEqual(tracker.progress, 0)
        for station in range(5, 705, 5):
            ref = tracker.reference_at(station)
            tracker.update_progress((ref['x_mm'], ref['y_mm']))
        self.assertAlmostEqual(tracker.progress, 700)
        self.assertAlmostEqual(tracker.reference_at(500)['field_yaw_deg'], -90-math.degrees(500/120))

    def test_progress_is_not_advanced_by_reference_or_frame_count(self):
        r, primitives, _geometry = prepared()
        tracker = TrajectoryTracker(r['trajectory'], primitives)
        first = r['trajectory'][0]
        for _ in range(100):
            ref, _velocity, _omega = tracker.command((first['x_mm'], first['y_mm'], first['field_yaw_deg']), 250, 120)
            self.assertEqual(ref['s_mm'], 100)
        self.assertEqual(tracker.progress, 0)


class SimulatorContinuousTests(unittest.TestCase):
    def test_straight_l_z_u_complete_without_intermediate_stops(self):
        routes = ([(500, 500), (1500, 500)], L,
                  [(500, 500), (1500, 500), (1500, 900), (2000, 900)],
                  [(500, 500), (1500, 500), (1500, 900), (500, 900)])
        for route in routes:
            with self.subTest(route=route):
                sim, r, _primitives, geometry, epoch, goal = simulator(route)
                previous, progress = None, -1
                minimum_speed = math.inf
                for index in range(1800):
                    before = sim.navigation_snapshot()
                    sim.make_frame(index/50)
                    snap = sim.navigation_snapshot()
                    actual = (*core.field_to_layout(*sim.hold), -90-(90-snap['yaw']))
                    self.assertIsNone(geometry.pose_reason(*actual))
                    if previous is not None:
                        self.assertIsNone(geometry.moving_pose_reason(previous, actual))
                    previous = actual
                    self.assertGreaterEqual(snap['progress_s_mm'], progress)
                    progress = snap['progress_s_mm']
                    self.assertEqual(snap['goal_id'], goal)  # 整条轨迹只有一次接受，不能逐点goal。
                    self.assertIsNone(snap['goto'])
                    self.assertEqual(snap['fault'], '')
                    if snap['reference'] and snap['reference']['segment_type'] != 'STOP':
                        minimum_speed = min(minimum_speed, snap['speed_mm_s'])
                        self.assertAlmostEqual(snap['reference']['s_mm']-snap['progress_s_mm'], 100)
                        self.assertEqual(snap['settled_frames'], 0)
                        self.assertNotEqual(snap['completed_id'], goal)
                    if not snap['active']:
                        break
                self.assertEqual(snap['tracking_status'], 'COMPLETE', snap)
                self.assertEqual(snap['completed_id'], goal)
                self.assertGreater(minimum_speed, 0)
                self.assertGreaterEqual(snap['settled_frames'], 10)
                self.assertLess(math.dist(core.field_to_layout(*sim.hold), route[-1]), 1)
                self.assertEqual(snap['speed_mm_s'], 0)
                self.assertLess(abs((90-snap['yaw']-r['trajectory'][-1]['field_yaw_deg']+180) % 360-180), 1)
                self.assertTrue(sim.line_q.empty() and sim.urgent_q.empty())

    def test_field_ops_mapping_rotation_offset_does_not_change_tracking(self):
        sim, r, _primitives, _geometry, _epoch, goal = simulator(mapping=(125, -300, 37))
        for index in range(1300):
            sim.make_frame(index/50)
            snap = sim.navigation_snapshot()
            if not snap['active']:
                break
        self.assertEqual(snap['tracking_status'], 'COMPLETE', snap)
        c, s = math.cos(math.radians(37)), math.sin(math.radians(37))
        actual = (125+c*sim.hold[0]+s*sim.hold[1], -300-s*sim.hold[0]+c*sim.hold[1])
        last = r['trajectory'][-1]
        self.assertLess(math.dist(actual, (last['x_mm'], last['y_mm'])), 1)
        self.assertEqual(snap['completed_id'], goal)

    def test_only_final_stop_needs_position_yaw_and_zero_velocity(self):
        sim, result, _primitives, _scene, _epoch, goal = simulator([(500, 500), (1000, 500)])
        for index in range(300):
            sim.make_frame(index/50)
            if sim.navigation_snapshot()['settled_frames'] == 1:
                break
        snap = sim.navigation_snapshot()
        self.assertEqual(snap['settled_frames'], 1)
        self.assertNotEqual(snap['completed_id'], goal)
        # 注入5°航向误差和角速度冻结：位置已经到终点，但不允许完成。
        sim.zval += 5
        sim._nav_yaw_rate_deg_s = 0
        for index in range(12):
            sim.make_frame(index/50)
        snap = sim.navigation_snapshot()
        self.assertLess(math.dist(sim.hold, (result['trajectory'][-1]['x_mm'], result['trajectory'][-1]['y_mm'])), 1)
        self.assertEqual(snap['settled_frames'], 0)
        self.assertNotEqual(snap['completed_id'], goal)
        sim.cancel_navigation()

    def test_guard_rejects_actual_motion_without_position_or_yaw_commit(self):
        sim, _r, _p, _s, _e, _g = simulator()
        before = sim.hold, sim.zval
        sim._nav_pose_guard = lambda a, b: '注入动态障碍'
        sim.make_frame(0)
        self.assertEqual((sim.hold, sim.zval), before)
        snap = sim.navigation_snapshot()
        self.assertEqual(snap['tracking_status'], 'FAULT')
        self.assertIn('动态障碍', snap['fault'])
        self.assertEqual(snap['speed_mm_s'], 0)
        self.assertFalse(snap['tracking'])

    def test_dynamic_obstacle_in_true_scene_guard_stops_before_contact(self):
        sim, _r, _p, _s, _e, _g = simulator([(500, 500), (1500, 500)])
        obstacle_scene = scene(dynamic_rects=[(810, 480, 830, 520, '动态障碍')])
        sim._nav_pose_guard = lambda a, b: obstacle_scene.moving_pose_reason(
            (*core.field_to_layout(a[0], a[1]), a[2]-180),
            (*core.field_to_layout(b[0], b[1]), b[2]-180))
        for index in range(120):
            sim.make_frame(index/50)
            if not sim.navigation_snapshot()['active']:
                break
        self.assertIn('动态障碍', sim.navigation_snapshot()['fault'])
        self.assertIsNone(obstacle_scene.pose_reason(*core.field_to_layout(*sim.hold), sim._relative_heading()-180))

    def test_nan_pose_xy_jump_yaw_jump_and_guard_exception_fail_closed(self):
        for fault in ('nan_xy', 'nan_yaw', 'xy_jump', 'yaw_jump', 'guard_exception'):
            with self.subTest(fault=fault):
                sim, _r, _p, _s, _e, _g = simulator()
                if fault == 'nan_xy': sim.hold = (math.nan, sim.hold[1])
                if fault == 'nan_yaw': sim.zval = math.nan
                if fault == 'xy_jump': sim.hold = (sim.hold[0]+100, sim.hold[1])
                if fault == 'yaw_jump': sim.zval += 90
                if fault == 'guard_exception':
                    sim._nav_pose_guard = lambda a, b: (_ for _ in ()).throw(RuntimeError('保护计算异常'))
                sim.make_frame(0)
                snap = sim.navigation_snapshot()
                self.assertEqual(snap['tracking_status'], 'FAULT', snap)
                self.assertFalse(snap['tracking'] or snap['active'])
                self.assertIsNone(snap['reference'])
                self.assertEqual(snap['speed_mm_s'], 0)

    def test_frozen_motion_watchdog_and_timeout(self):
        for mode in ('no_progress', 'timeout'):
            sim, _r, _p, _s, _e, _g = simulator()
            if mode == 'no_progress': sim._nav_speed_mm_s = 0
            else: sim._nav_timeout = .05
            for index in range(400):
                sim.make_frame(index/50)
                if not sim.navigation_snapshot()['active']:
                    break
            snap = sim.navigation_snapshot()
            self.assertEqual(snap['tracking_status'], 'FAULT')
            self.assertIn('无进展' if mode == 'no_progress' else '超时', snap['fault'])

    def test_stop_zero_wheeloff_and_queued_stale_motion_cancel_immediately(self):
        for command in ('STOP', 'ZERO', 'WHEELOFF', 'OPSOFFSET=60,-50'):
            sim, _r, _p, _s, _epoch, _goal = simulator()
            sim.make_frame(.02)
            sim.line_q.put('GOTO=50,60,0')
            sim.urgent_q.put('MANUAL=20,0,0')
            sim.handle_line(command)
            before = sim.hold, sim.zval
            self.assertFalse(sim.navigation_snapshot()['tracking'])
            self.assertIsNone(sim.navigation_snapshot()['reference'])
            sim._service_commands()
            for index in range(20): sim.make_frame(index/50)
            self.assertEqual((sim.hold, sim.zval), before)
            self.assertIsNone(sim.goto)

    def test_stop_or_version_change_inside_guard_cannot_commit_late_motion(self):
        for mode in ('stop', 'version'):
            state = {'version': 1}
            sim, _r, _p, _s, _epoch, _goal = simulator(validity=lambda: state['version'] == 1)
            before = sim.hold, sim.zval
            def guard(a, b):
                if mode == 'stop': sim.cancel_navigation()
                else: state['version'] = 2
                return None
            sim._nav_pose_guard = guard
            sim.make_frame(0)
            self.assertEqual((sim.hold, sim.zval), before)
            self.assertFalse(sim.navigation_snapshot()['tracking'])

    def test_cancel_inside_acceptance_pose_guard_cannot_restore_tracker(self):
        result, primitives, geometry = prepared()
        for mode in ('stop', 'version'):
            sim = core.Simulator(queue.Queue(), queue.Queue())
            sim.handle_line('ZERO'); sim.hold=(1750, 1750); sim.zval=180
            epoch = sim.begin_navigation()
            state = {'version': 1}
            def guard(a, b):
                if mode == 'stop': sim.cancel_navigation()
                else: state['version'] = 2
                return None
            with patch.object(geometry, 'moving_pose_reason', side_effect=guard):
                with self.assertRaises(ValueError):
                    sim.submit_navigation_trajectory(epoch, result['trajectory'], primitives,
                                                     (0, 0, 0), geometry, lambda: state['version'] == 1)
            self.assertFalse(sim.navigation_snapshot()['tracking'])
            self.assertEqual(sim.navigation_snapshot()['goal_id'], 0)
            sim.cancel_navigation()

    def test_stop_during_acceptance_recheck_rejects_old_epoch(self):
        result, primitives, geometry = prepared()
        sim = core.Simulator(queue.Queue(), queue.Queue())
        sim.handle_line('ZERO'); sim.hold=(1750, 1750); sim.zval=180
        epoch = sim.begin_navigation()
        entered, release = threading.Event(), threading.Event()
        actual_validate = core.validate_trajectory
        failures = []
        def blocked(*args, **kwargs):
            entered.set()
            if not release.wait(2): raise RuntimeError('测试复检超时')
            return actual_validate(*args, **kwargs)
        def submit():
            try: sim.submit_navigation_trajectory(epoch, result['trajectory'], primitives, (0, 0, 0), geometry)
            except Exception as exc: failures.append(exc)
        with patch.object(core, 'validate_trajectory', side_effect=blocked):
            thread = threading.Thread(target=submit)
            thread.start()
            self.assertTrue(entered.wait(2))
            sim.cancel_navigation()  # 不等待重几何检查的锁。
            self.assertFalse(sim.navigation_snapshot()['active'])
            release.set(); thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIn('失效', str(failures[0]))
        self.assertFalse(sim.navigation_snapshot()['tracking'])


class ContinuousUiTests(unittest.TestCase):
    def setUp(self): self.w = make_window()
    def tearDown(self): close_window(self.w)

    def start(self):
        self.w.nav_map['rects'] = []; self.w.nav_map['circles'] = []
        self.w.sim.hold=(1750, 1750); self.w.sim.zval=180; self.w.sim.make_frame(0)
        ctx = self.w._prepare_plan(1500, 1500); ctx['kwargs']['goal_heading_deg']=90
        result = nav.plan(ctx['start'], ctx['goal'], **ctx['kwargs'])
        self.w._finish_plan(result, ctx)
        self.w._toggle_follow()
        self.assertIsNotNone(self.w.follow, self.w.map_status.text)
        return result, ctx

    def test_skeleton_continuous_reference_layers_and_simulation_end(self):
        r, ctx = self.start()
        self.assertEqual(self.w.map_view.skeleton, r['points'])
        self.assertEqual(self.w.map_view.points, r['smoothed_points'])
        self.assertEqual(self.w.map_view.reference['s_mm'], 100)
        for _ in range(800):
            advance(self.w)
            if self.w.follow is None: break
        self.assertIsNone(self.w.follow)
        self.assertIn('最终STOP', self.w.map_status.text)
        self.assertIsNone(self.w.map_view.reference)
        self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())

    def test_stop_clear_replan_map_replace_and_version_cancel_before_next_tick(self):
        for action in ('stop', 'clear', 'replan', 'version', 'replace'):
            with self.subTest(action=action):
                self.start(); advance(self.w, 10)
                sim = self.w.sim
                if action == 'stop': self.w.send_line('STOP')
                elif action == 'clear': self.w._clear_path()
                elif action == 'replan': self.w._prepare_plan(1400, 1500)
                elif action == 'version': self.w.nav_map['map_version'] += 1
                else: self.w.nav_map = dict(self.w.nav_map, map_version=99)
                before = sim.hold, sim.zval
                self.assertIsNone(self.w.follow)
                self.assertFalse(sim.navigation_snapshot()['tracking'])
                self.assertIsNone(self.w.map_view.reference)
                for index in range(5): sim.make_frame(index/50)
                self.assertEqual((sim.hold, sim.zval), before)

    def test_stale_pose_or_hardware_takeover_stops_with_no_serial_navigation(self):
        for action in ('stale', 'hardware'):
            self.start()
            if action == 'stale': self.w.sim.last_frame_monotonic -= 1
            else: self.w.worker=object()
            self.w._follow_step()
            self.assertIsNone(self.w.follow)
            self.assertFalse(self.w.sim.navigation_snapshot()['tracking'])
            self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())
            self.w.worker=None

    def test_bad_start_yaw_and_forced_false_completion_cannot_be_accepted(self):
        self.start(); self.w._stop_follow()
        self.w.sim.zval += 90
        self.w.sim.make_frame(0)
        ctx = self.w._prepare_plan(1500, 500)
        r = nav.plan(ctx['start'], ctx['goal'], **ctx['kwargs'])
        self.w._finish_plan(r, ctx)
        self.w._toggle_follow()
        self.assertIsNone(self.w.follow)
        self.start()
        sim = self.w.sim
        sim._nav_completed_id = self.w.follow['goal_id']
        sim._nav_status = 'COMPLETE'; sim._nav_settled = 10
        sim._nav_velocity = (10, 0)
        self.assertFalse(self.w._at_final_stop())
        self.w._clear_path()


if __name__ == '__main__': unittest.main()
