"""比赛文档任务语义、整轮两个启停区、真实控制器及作业/取消故障注入。"""
import copy
import math
import queue
import unittest
from unittest.mock import patch

import core
import competition_simulation as competition


def runner_for(match, validity=lambda: True):
    sim = core.Simulator(queue.Queue(), queue.Queue())
    # 参数一致性门禁不能用默认模拟器验证另一组实机参数。
    for name,value in (match.get('chassis_control') or {}).items():
        setattr(sim,name,value)
    sim.handle_line('ZERO')
    sim.hold = core.layout_to_field(*match['home'])
    sim.zval = 180+match['start_yaw']
    sim.make_frame(0)
    runner = competition.CompetitionRunner(sim, copy.deepcopy(match), validity=validity)
    runner.start()
    return runner


def advance(runner, count=1):
    for _ in range(count):
        runner.sim.make_frame(runner.elapsed_s)
        runner.tick()


def until(runner, predicate, limit=9000):
    for _ in range(limit):
        if predicate(runner) or not runner.active:
            return
        advance(runner)
    raise AssertionError('比赛测试等待超时')


class CompetitionCodeTests(unittest.TestCase):
    def test_official_code_and_second_stack_by_color_not_second_rough_slot(self):
        first, second = competition.parse_task_code(competition.DEFAULT_CODE)
        self.assertEqual(first['colors'], (1, 5, 6))
        self.assertEqual(second['rough_slots'], (2, 3, 1))
        self.assertEqual(second['storage_slots'], (2, 1, 3))
        self.assertEqual(dict(zip(first['colors'], first['storage_slots'])),
                         dict(zip(second['colors'], second['storage_slots'])))

    def test_another_code_preserves_each_color_stack(self):
        first, second = competition.parse_task_code('642+312+264+123')
        self.assertEqual(second['storage_slots'], (2, 3, 1))
        self.assertEqual(set(first['colors']), set(second['colors']))

    def test_invalid_code_colors_slots_and_different_batches_rejected(self):
        for code in ('123+123', '000+123+000+123', '112+123+121+123',
                     '123+112+321+123', '123+123+456+123', '123+123+321+234',
                     '123＋123＋321＋123', '123+123+321+123junk'):
            with self.subTest(code=code), self.assertRaises(ValueError):
                competition.parse_task_code(code)


class CompetitionFlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = competition.load_profile()
        cls.matches = {z: competition.compile_match(cls.data, zone=z) for z in (1, 2)}

    def test_two_start_zones_complete_entire_round_with_actual_body_sweeps(self):
        for zone, match in self.matches.items():
            with self.subTest(zone=zone):
                runner = runner_for(match)
                previous = None
                while runner.active:
                    advance(runner)
                    snap = runner.sim.navigation_snapshot()
                    lx, ly = core.field_to_layout(*snap['hold'])
                    actual = lx, ly, -180+snap['yaw']
                    self.assertIsNone(runner.scene.pose_reason(*actual))
                    if previous is not None:
                        self.assertIsNone(runner.scene.moving_pose_reason(previous, actual))
                    previous = actual
                self.assertEqual(runner.status, 'COMPLETE', runner.reason)
                self.assertLess(runner.elapsed_s, 180)
                self.assertEqual((runner.grabs, runner.placements), (12, 12))
                self.assertEqual(runner.storage, {1: [1, 1], 2: [5, 5], 3: [6, 6]})
                self.assertEqual(runner.cargo, []); self.assertEqual(runner.rough, {})
                self.assertEqual((lx, ly), tuple(match['home']))
                self.assertEqual(runner.display_code, competition.DEFAULT_CODE)
                self.assertTrue(runner.sim.line_q.empty() and runner.sim.urgent_q.empty())
                self.assertIsNone(runner.sim.goto)

    def test_task_order_requires_all_rough_placements_before_retrieval_and_second_batch(self):
        stages = self.matches[1]['stages']
        kinds = [s['kind'] for s in stages]
        self.assertLess(kinds.index('SCAN'), kinds.index('RAW_PICK'))
        for batch in (1, 2):
            placements = [i for i, s in enumerate(stages) if s['kind'] == 'ROUGH_PLACE' and s['batch'] == batch]
            grabs = [i for i, s in enumerate(stages) if s['kind'] == 'ROUGH_PICK' and s['batch'] == batch]
            self.assertEqual(len(placements), 3); self.assertEqual(len(grabs), 3)
            self.assertLess(max(placements), min(grabs))
        first_store = [i for i, s in enumerate(stages) if s['kind'] == 'STORAGE_PLACE' and s['batch'] == 1]
        second_pick = [i for i, s in enumerate(stages) if s['kind'] == 'RAW_PICK' and s['batch'] == 2]
        self.assertLess(max(first_store), min(second_pick))

    def test_mecanum_round_compares_direction_cost_and_keeps_diagonal_docking(self):
        for zone, match in self.matches.items():
            with self.subTest(zone=zone):
                self.assertTrue(match['diagonal_docking'])
                maneuvers = [s for s in match['stages'] if s['kind'] == 'MANEUVER']
                self.assertEqual(maneuvers[0]['target'], competition.departure_reference(self.data,zone)[1])
                self.assertEqual(maneuvers[-1]['target'], tuple(match['home']))
                self.assertEqual(match['start_yaw'], 180)  # 初始车头仍+Y，界面0°。
                headings = {round(p['field_yaw_deg'], 2) for l in match['legs'] for p in l['route']['trajectory']}
                self.assertGreater(len(headings), 1)
                modes = {p['motion_mode'] for l in match['legs'] for p in l['route']['trajectory']}
                self.assertTrue({'FORWARD','REVERSE','DIAGONAL'} <= modes)
                for l in match['legs']:
                    self.assertTrue(l['route']['trajectory_safe'])
                    self.assertIn('selection', l['route'])
                    self.assertEqual(l['route']['selection']['policy'], 'DIRECTION_COST_AND_VERIFIED_TIME')
                for leg in (match['legs'][2], match['legs'][5]):
                    self.assertLess(leg['route']['motion_metrics']['longest_strafe_mm'], 1)
                    self.assertAlmostEqual(leg['route']['trajectory_length_mm'], 1630)

    def test_screenshot_one_obstacle_has_straight_lanes_and_no_raw_to_rough_strafe(self):
        from mecanum_planner import shallow_diagonal
        match = competition.compile_match(self.data, sim_obstacles=[(700, 1200)])
        for leg in match['legs']:
            points = leg['route'].get('driving_points', leg['route']['points'])
            self.assertFalse(any(shallow_diagonal(a, b) for a, b in zip(points, points[1:])), leg['label'])
        for leg in (match['legs'][2], match['legs'][5]):
            self.assertLess(leg['route']['motion_metrics']['lateral_mm'], 1)
            self.assertEqual({p['motion_mode'] for p in leg['route']['trajectory']}, {'FORWARD'})
        runner = runner_for(match)
        until(runner, lambda r: not r.active)
        self.assertEqual(runner.status, 'COMPLETE', runner.reason)
        self.assertEqual((runner.grabs, runner.placements), (12, 12))

    def test_blocked_diagonal_docking_uses_safe_l_maneuvers(self):
        match = competition.compile_match(self.data, sim_obstacles=[(2340,2070)])
        scene = competition.collision_scene(self.data, sim_obstacles=match['sim_obstacles'])
        staging = tuple(self.data['competition']['staging']['1'])
        self.assertIsNotNone(scene.translation_reason(match['home'], staging, match['start_yaw']))
        self.assertFalse(match['diagonal_docking'])
        self.assertEqual(match['stages'][0]['target'], (2100,2250))
        self.assertEqual(match['stages'][1]['target'], staging)
        runner = runner_for(match)
        until(runner, lambda r:not r.active)
        self.assertEqual(runner.status, 'COMPLETE', runner.reason)
        self.assertEqual((runner.grabs,runner.placements), (12,12))

    def test_code_display_waits_for_qr_arrival_and_scan_action(self):
        runner = runner_for(self.matches[1])
        until(runner, lambda r: r.stage['kind'] == 'SCAN')
        self.assertEqual(runner.display_code, '')
        self.assertEqual(runner.grabs, 0)
        advance(runner, 24)
        self.assertEqual(runner.display_code, '')
        advance(runner, 2)
        self.assertEqual(runner.display_code, competition.DEFAULT_CODE)

    def test_no_new_frame_cannot_advance_clock_actions_or_stages(self):
        runner = runner_for(self.matches[1])
        until(runner, lambda r: r.status == 'ACTION')
        state = runner.index, runner.elapsed_s, runner.action_elapsed_s, runner.grabs
        for _ in range(100): runner.tick()
        self.assertEqual((runner.index, runner.elapsed_s, runner.action_elapsed_s, runner.grabs), state)

    def test_same_round_can_only_start_once(self):
        runner = runner_for(self.matches[1])
        with self.assertRaises(ValueError): runner.start()
        runner.cancel()
        with self.assertRaises(ValueError): runner.start()

    def test_stop_and_version_changes_cancel_during_motion_and_work_without_restart(self):
        for phase in ('motion', 'work'):
            for mode in ('stop', 'version'):
                version = {'value': 1}
                runner = runner_for(self.matches[1], lambda: version['value'] == 1)
                if phase == 'work': until(runner, lambda r: r.status == 'ACTION')
                if mode == 'stop': runner.sim.handle_line('STOP')
                else: version['value'] = 2
                before = runner.sim.hold, runner.sim.zval
                advance(runner, 30)
                self.assertEqual(runner.status, 'FAULT')
                self.assertEqual((runner.sim.hold, runner.sim.zval), before)
                self.assertFalse(runner.sim.navigation_snapshot()['tracking'])

    def test_missing_cargo_fails_without_counting_a_placement(self):
        runner = runner_for(self.matches[1])
        until(runner, lambda r: r.stage['kind'] == 'ROUGH_PLACE')
        runner.cargo.clear()
        advance(runner, 26)
        self.assertEqual(runner.status, 'FAULT')
        self.assertEqual(runner.placements, 0)
        self.assertIn('物料', runner.reason)

    def test_wrong_first_layer_cannot_count_a_stack(self):
        runner = runner_for(self.matches[1])
        until(runner, lambda r: r.stage['kind'] == 'STORAGE_PLACE' and r.stage['batch'] == 2)
        count = runner.placements
        runner.storage[runner.stage['slot']] = [3]
        advance(runner, 26)
        self.assertEqual(runner.status, 'FAULT')
        self.assertEqual(runner.placements, count)
        self.assertIn('同色', runner.reason)

    def test_timeout_and_overlong_station_action_stop(self):
        runner = runner_for(self.matches[1]); runner.elapsed_s = 179.99
        advance(runner)
        self.assertEqual(runner.status, 'FAULT'); self.assertIn('180秒', runner.reason)
        runner = runner_for(self.matches[1])
        until(runner, lambda r: r.status == 'ACTION')
        runner.stage['duration_s'] = 16
        advance(runner, 755)
        self.assertEqual(runner.status, 'FAULT'); self.assertIn('15秒', runner.reason)

    def test_fake_completion_and_motion_during_action_are_rejected(self):
        runner = runner_for(self.matches[1])
        sim = runner.sim
        sim._nav_tracker = None; sim._nav_active = False
        sim._nav_status = 'COMPLETE'; sim._nav_completed_id = runner.goal_id; sim._nav_settled = 10
        advance(runner)
        self.assertEqual(runner.status, 'FAULT'); self.assertIn('未到位', runner.reason)
        runner = runner_for(self.matches[1])
        until(runner, lambda r: r.status == 'ACTION')
        runner.sim.hold = (runner.sim.hold[0]+2, runner.sim.hold[1])
        advance(runner)
        self.assertEqual(runner.status, 'FAULT'); self.assertIn('作业期间', runner.reason)

    def test_prepare_rejects_occupied_station_bad_margin_and_cancellation(self):
        data = copy.deepcopy(self.data)
        data['competition']['stations']['qr'] = [700, 700]
        for kwargs in ({'data': data}, {'data': self.data, 'margin': 30},
                       {'data': self.data, 'cancelled': lambda: True}):
            with self.assertRaises(ValueError): competition.compile_match(**kwargs)

    def test_preflight_rejects_real_tracking_collision_even_if_geometry_is_safe(self):
        scene = competition.collision_scene(self.data)
        with patch.object(competition, '_tracking_reason', return_value='注入实际控制偏差'):
            with self.assertRaisesRegex(ValueError, '实际连续跟踪预演'):
                competition.plan_leg((2100, 300), (2180, 1200), scene,
                                     [(2100, 300), (2100, 1200), (2180, 1200)])


class CompetitionObstacleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = competition.load_profile()
        cls.matches = {z: competition.compile_match(cls.data, zone=z,
                          sim_obstacles=competition.DEMO_OBSTACLES) for z in (1, 2)}

    def test_four_obstacles_both_zones_complete_with_every_actual_rectangle_sweep_safe(self):
        for zone, match in self.matches.items():
            with self.subTest(zone=zone):
                runner = runner_for(match)
                self.assertEqual(len(runner.scene.circles), len(self.data['circles'])+4)
                previous = None
                while runner.active:
                    advance(runner)
                    snap = runner.sim.navigation_snapshot()
                    x, y = core.field_to_layout(*snap['hold'])
                    actual = x, y, -180+snap['yaw']
                    self.assertIsNone(runner.scene.pose_reason(*actual))
                    if previous is not None:
                        self.assertIsNone(runner.scene.moving_pose_reason(previous, actual))
                    previous = actual
                self.assertEqual(runner.status, 'COMPLETE', runner.reason)
                self.assertLess(runner.elapsed_s, 180)
                self.assertEqual((runner.grabs, runner.placements), (12, 12))
                self.assertEqual(runner.storage, {1: [1, 1], 2: [5, 5], 3: [6, 6]})
                self.assertEqual((x, y), tuple(match['home']))
                self.assertFalse(runner.match['hardware_ready'])

    def test_central_blocker_forces_detour_instead_of_only_drawing_an_obstacle(self):
        match = self.matches[1]
        route = match['legs'][2]['route']  # 原料→粗加工原为直线，障碍后必须绕行。
        self.assertGreater(route['trajectory_length_mm'], 1630+1000)
        self.assertGreater(len(route['points']), 2)
        fixed = competition.collision_scene(self.data)
        with_obstacle = competition.collision_scene(self.data, sim_obstacles=competition.DEMO_OBSTACLES)
        self.assertIsNone(fixed.translation_reason((1200, 2030), (1200, 400), -90))
        self.assertIn('模拟障碍', with_obstacle.translation_reason((1200, 2030), (1200, 400), -90))

    def test_obstacle_snapshot_export_is_independent_and_map_has_no_duplicate_cylinders(self):
        data = copy.deepcopy(self.data); points = [[1200, 1700]]
        match = competition.compile_match(data, sim_obstacles=points)
        points[0][0] = 100; data['circles'].clear()
        self.assertEqual(match['sim_obstacles'], [(1200., 1700.)])
        self.assertEqual(match['map_snapshot']['circles'], self.data['circles'])
        saved = runner_for(match).export_snapshot()['match']
        self.assertEqual(saved['obstacle_frame_id'], 'LAYOUT_MM')
        self.assertEqual(saved['obstacle_radius_mm'], 25)
        self.assertEqual(saved['obstacle_height_mm'], 100)

    def test_invalid_obstacles_are_rejected_without_ignoring_them(self):
        for points in ([(700, 700)], [(20, 400)], [(1200, 1700)]*2,
                       [(float('nan'), 1200)], [(1200, float('inf'))],
                       [(300, 300)]*5):
            with self.subTest(points=points), self.assertRaises(ValueError):
                competition.compile_match(self.data, sim_obstacles=points)

    def test_occupied_start_or_station_and_disconnected_lanes_refuse_entire_round(self):
        for points, reason in (([(2250, 2250)], '出入扫掠'),
                               ([(2180, 1200)], '停靠点'),
                               ([(1200, 1700), (2100, 1700), (300, 1700)], 'fallback')):
            with self.subTest(points=points), self.assertRaisesRegex(ValueError, reason):
                competition.compile_match(self.data, sim_obstacles=points)

    def test_actual_pose_on_obstacle_stops_without_committing_more_motion(self):
        runner = runner_for(self.matches[1])
        runner.sim.hold = core.layout_to_field(1200, 1700)
        before = runner.sim.hold, runner.sim.zval
        advance(runner, 2)
        self.assertEqual(runner.status, 'FAULT')
        self.assertEqual((runner.sim.hold, runner.sim.zval), before)
        self.assertFalse(runner.sim.navigation_snapshot()['tracking'])
        self.assertEqual((runner.grabs, runner.placements), (0, 0))


class AvoidanceOptimizationTests(unittest.TestCase):
    def test_screenshot_two_cylinders_complete_both_zones_with_all_actual_sweeps_safe(self):
        # 截图中心障碍约LAYOUT(1200,1200)，另一障碍由FIELD(195.6,195.3)cm换算。
        for zone in (1, 2):
            match = competition.compile_match(competition.load_profile(), zone=zone,
                                               sim_obstacles=[(1200, 1200), (297, 294)])
            runner = runner_for(match)
            previous = None
            while runner.active:
                advance(runner)
                snap = runner.sim.navigation_snapshot()
                actual = (*core.field_to_layout(*snap['hold']), -180+snap['yaw'])
                self.assertIsNone(runner.scene.pose_reason(*actual))
                if previous is not None:
                    self.assertIsNone(runner.scene.moving_pose_reason(previous, actual))
                previous = actual
            self.assertEqual(runner.status, 'COMPLETE', runner.reason)
            self.assertLess(runner.elapsed_s, 180)
            self.assertEqual((runner.grabs, runner.placements), (12, 12))
            self.assertEqual(runner.storage, {1: [1, 1], 2: [5, 5], 3: [6, 6]})
            self.assertEqual(actual[:2], tuple(match['home']))
            self.assertLess(max(leg['route']['search']['expanded'] for leg in match['legs']), 5000)

    def test_small_screenshot_coordinate_error_also_finds_safe_full_round(self):
        match = competition.compile_match(competition.load_profile(),
                                          sim_obstacles=[(1188, 1210), (294, 297)])
        runner = runner_for(match)
        until(runner, lambda r: not r.active)
        self.assertEqual(runner.status, 'COMPLETE', runner.reason)
        self.assertLess(runner.elapsed_s, 180)

    def test_adaptive_parallel_lanes_work_when_every_original_lane_is_blocked(self):
        points = [(1000, 500), (1000, 1500), (500, 1000), (1500, 1000)]
        scene = competition.nav.CollisionScene([], [], (0, 0, 2400, 2400), 10, (280, 260, 0),
                                               sim_circles=core.sim_obstacle_circles(points))
        anchors = [(500, 500), (500, 1500), (1500, 500), (1500, 1500)]
        route = competition.plan_leg((500, 500), (1500, 1500), scene, anchors)
        self.assertTrue(route['search']['refined'])
        self.assertTrue(route['trajectory_safe'])
        self.assertEqual(route['points'][0], (500, 500))
        self.assertEqual(route['points'][-1], (1500, 1500))
        self.assertIsNone(competition._tracking_reason(route, scene, lambda: False))

    def test_completed_ninety_degree_maneuver_has_no_residual_yaw_before_translation(self):
        scene = competition.nav.CollisionScene([], [], (0, 0, 2400, 2400), 10, (280, 260, 0))
        sim = core.Simulator(queue.Queue(), queue.Queue())
        sim.handle_line('ZERO'); sim.hold = core.layout_to_field(1200, 1200); sim.zval = 180
        sim.submit_navigation_maneuver(sim.begin_navigation(), (1200, 1200), 90, (0, 0, 0), scene)
        for i in range(500):
            sim.make_frame(i*.02)
            if not sim.navigation_snapshot()['tracking']: break
        snap = sim.navigation_snapshot()
        self.assertEqual(snap['tracking_status'], 'COMPLETE', snap['fault'])
        self.assertAlmostEqual((snap['yaw']-180) % 360, 90, places=6)
        sim.submit_navigation_maneuver(sim.begin_navigation(), (1200, 1400), 90, (0, 0, 0), scene)
        for i in range(500):
            sim.make_frame(i*.02)
            if not sim.navigation_snapshot()['tracking']: break
        self.assertEqual(sim.navigation_snapshot()['tracking_status'], 'COMPLETE')
        self.assertEqual(core.field_to_layout(*sim.hold), (1200, 1400))


if __name__ == '__main__': unittest.main()
