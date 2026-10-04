"""真实车头/切线解耦、倒退/斜行、矩形与圆弧间隙检查；仅PC。"""
import copy
import json
import math
import queue
import tempfile
from pathlib import Path
import unittest

import core
import navigation_planner as nav
import trajectory
from mecanum_planner import fixed_route, fixed_primitives, motion_metrics, straight_lane_route, shallow_diagonal
from competition_simulation import _tracking_reason


def scene(bounds=(0, 0, 2400, 2400), circles=(), footprint=(280, 260, 0)):
    return nav.CollisionScene([], list(circles), bounds, 10, footprint)


class MecanumPlanningTests(unittest.TestCase):
    def test_direction_cost_distinguishes_forward_reverse_and_long_strafe(self):
        rows = []
        for yaw in (0, 180, 90):
            pieces = [dict(kind='LINE', start=(600, 800), end=(1600, 800), heading_mode='FIXED', body_yaw_deg=yaw)]
            rows.append(motion_metrics(trajectory.generate_trajectory(pieces, scene())))
        self.assertAlmostEqual(rows[0]['equivalent_cost_mm'], 1000)
        self.assertAlmostEqual(rows[1]['equivalent_cost_mm'], 1150)
        self.assertAlmostEqual(rows[2]['equivalent_cost_mm'], 1800)
        self.assertAlmostEqual(rows[2]['longest_strafe_mm'], 1000)
        self.assertLess(rows[0]['longest_strafe_mm'], 1)

    def test_straight_lane_replaces_unnecessary_shallow_diagonal_but_checks_whole_body(self):
        original = [(500, 500), (600, 1500), (1600, 1500)]
        r = fixed_route(original, 0, scene(), lambda: False, _tracking_reason, {})
        self.assertFalse(any(shallow_diagonal(a, b) for a, b in zip(r['driving_points'], r['driving_points'][1:])))
        self.assertTrue(r['trajectory_safe'])
        blocked = scene(circles=[(630, 1000, 25, '阻挡一侧直车道')], footprint=(30,20,0))
        points = straight_lane_route(original, blocked, 0, lambda: False)
        self.assertEqual(points[0], original[0]); self.assertEqual(points[-1], original[-1])
        self.assertTrue(all(blocked.translation_reason(a,b,0) is None for a,b in zip(points,points[1:])))

    def test_reverse_has_opposite_body_and_tangent_without_turning(self):
        pieces = [dict(kind='LINE', start=(1200, 800), end=(1200, 1500), heading_mode='REVERSE_TANGENT')]
        r = dict(trajectory.generate_trajectory(pieces, scene()), smoothed_primitives=pieces)
        self.assertTrue(r['trajectory_safe'], r)
        self.assertEqual({p['motion_mode'] for p in r['trajectory']}, {'REVERSE'})
        self.assertTrue(all(abs(abs(p['field_yaw_deg']-p['tangent_yaw_deg'])-180) < 1e-5 for p in r['trajectory']))
        self.assertIsNone(_tracking_reason(r, scene(), lambda: False))

    def test_diagonal_translation_keeps_real_body_heading(self):
        r = fixed_route([(800, 800), (800, 1300), (1300, 1300)], 0, scene(), lambda: False,
                        _tracking_reason, {})
        self.assertEqual(len(r['driving_points']), 2)
        self.assertAlmostEqual(r['trajectory_length_mm'], math.sqrt(2)*500)
        self.assertEqual({p['motion_mode'] for p in r['trajectory']}, {'DIAGONAL'})
        self.assertEqual({p['field_yaw_deg'] for p in r['trajectory']}, {-90})

    def test_fixed_heading_cannot_hide_sharp_velocity_corner(self):
        r = trajectory.generate_trajectory([
            dict(kind='LINE', start=(800, 800), end=(800, 1200), heading_mode='FIXED', body_yaw_deg=0),
            dict(kind='LINE', start=(800, 1200), end=(1200, 1200), heading_mode='FIXED', body_yaw_deg=0)], scene())
        self.assertEqual(r['trajectory_status'], 'FALLBACK_REQUIRED')
        self.assertEqual(r['trajectory'], [])

    def test_fixed_and_reverse_arc_body_yaw_and_unwrap_are_continuous(self):
        pieces, _ = fixed_primitives([(600, 600), (1100, 600), (1100, 1100)], 89, scene(), lambda: False)
        for mode in ('FIXED', 'REVERSE_TANGENT'):
            ps = [dict(p, heading_mode=mode) for p in pieces]
            r = dict(trajectory.generate_trajectory(ps, scene()), smoothed_primitives=ps)
            self.assertTrue(r['trajectory_safe'], r)
            self.assertTrue(all(abs(a['field_yaw_deg']-b['field_yaw_deg']) <= 20
                                for a, b in zip(r['trajectory'], r['trajectory'][1:])))
            self.assertIsNone(_tracking_reason(r, scene(), lambda: False))
            if mode == 'FIXED':
                self.assertEqual({p['field_yaw_deg'] for p in r['trajectory']}, {-179})

    def test_checks_real_rectangle_orientation_in_narrow_corridor(self):
        corridor = scene(bounds=(0, 0, 2000, 330), footprint=(500, 200, 0))
        line = dict(kind='LINE', start=(400, 165), end=(1500, 165), heading_mode='FIXED', body_yaw_deg=0)
        self.assertTrue(trajectory.generate_trajectory([line], corridor)['trajectory_safe'])
        line['body_yaw_deg'] = 90
        self.assertEqual(trajectory.generate_trajectory([line], corridor)['trajectory_status'], 'UNSAFE')

    def test_fixed_arc_interval_includes_center_sagitta_even_with_zero_yaw_change(self):
        # 小矩形随大圆平移：端点都安全，障碍处在弦与圆之间；必须检查采样间包络。
        center, radius = (1000, 1000), 1000
        a = (center[0]+radius*math.cos(math.radians(-1.5)), center[1]+radius*math.sin(math.radians(-1.5)), 0)
        b = (a[0], center[1]+radius*math.sin(math.radians(1.5)), 0)
        s = nav.CollisionScene([], [(2000, 1000, .001, '弧间障碍')], (0, 0, 3000, 3000), 0, (.02, .02, 0))
        self.assertTrue(s.pose_safe(*a)); self.assertTrue(s.pose_safe(*b))
        self.assertIsNotNone(s.arc_interval_reason(a, b, center))

    def test_invalid_modes_and_tampered_heading_metadata_are_rejected(self):
        line = dict(kind='LINE', start=(800, 800), end=(1400, 800), heading_mode='FIXED', body_yaw_deg=90)
        r = trajectory.generate_trajectory([line], scene())
        for key, value in (('field_yaw_deg', 0), ('tangent_yaw_deg', 123), ('motion_mode', 'FORWARD')):
            samples = copy.deepcopy(r['trajectory']); samples[1][key] = value
            self.assertFalse(trajectory.validate_trajectory(samples, [line], scene())['ok'])
        for mode in ('UNKNOWN', None):
            line['heading_mode'] = mode
            self.assertEqual(trajectory.generate_trajectory([line], scene())['trajectory_status'], 'INVALID_INPUT')

    def test_export_distinguishes_body_and_driving_direction(self):
        line = dict(kind='LINE', start=(800, 800), end=(1400, 800), heading_mode='FIXED', body_yaw_deg=90)
        r = trajectory.generate_trajectory([line], scene())
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'trajectory.json'
            trajectory.export_trajectory_json(path, r)
            saved = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(saved['schema_version'], 2)
        self.assertFalse(saved['hardware_ready'])
        self.assertNotEqual(saved['trajectory'][0]['field_yaw_deg'], saved['trajectory'][0]['tangent_yaw_deg'])

    def test_non_ninety_degree_diagonal_corner_is_smooth_and_safe(self):
        pieces, attempts = fixed_primitives([(500, 500), (1100, 500), (1400, 1000)], 0, scene(), lambda: False)
        arc, = [p for p in pieces if p['kind'] == 'ARC']
        self.assertGreater(arc['sweep_deg'], 50); self.assertLess(arc['sweep_deg'], 70)
        self.assertEqual(attempts[0]['attempts'][0]['radius_mm'], 120)
        r = dict(trajectory.generate_trajectory(pieces, scene()), smoothed_primitives=pieces)
        self.assertTrue(r['trajectory_safe'], r)
        self.assertIsNone(_tracking_reason(r, scene(), lambda: False))

    def test_reverse_stop_and_snapshot_change_cancel_before_more_motion(self):
        pieces = [dict(kind='LINE', start=(1200, 800), end=(1200, 1500), heading_mode='REVERSE_TANGENT')]
        r = trajectory.generate_trajectory(pieces, scene())
        for operation in ('STOP', 'VERSION'):
            valid = {'value': True}
            sim = core.Simulator(queue.Queue(), queue.Queue()); sim.handle_line('ZERO')
            p = r['trajectory'][0]; sim.hold = p['x_mm'], p['y_mm']; sim.zval = 90-p['field_yaw_deg']
            sim.submit_navigation_trajectory(sim.begin_navigation(), r['trajectory'], pieces, (0, 0, 0), scene(),
                                             lambda: valid['value'])
            for i in range(30): sim.make_frame(i*.02)
            if operation == 'STOP': sim.handle_line('STOP')
            else: valid['value'] = False
            before = sim.hold, sim.zval
            for i in range(10): sim.make_frame(i*.02)
            self.assertEqual((sim.hold, sim.zval), before)
            self.assertFalse(sim.navigation_snapshot()['tracking'])

    def test_all_arc_radii_fail_explicitly_without_forcing_corner(self):
        with self.assertRaisesRegex(ValueError, '所有半径失败'):
            fixed_primitives([(600,600),(1100,600),(1100,1100)], 0,
                             scene(circles=[(1000,600,25,'阻挡圆弧')]), lambda:False)
