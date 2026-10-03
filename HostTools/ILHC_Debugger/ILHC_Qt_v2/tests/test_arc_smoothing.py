"""90°圆弧：独立几何、真实整车、半径回退及界面快照回归。"""
import json
import math
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from shapely.geometry import Point, Polygon

import core
import navigation_planner as nav
from arc_smoothing import ARC_RADII_MM, smooth_90_corners, _candidate, _arc_reason
from tests.headless_support import make_window, close_window, WindowMethods


def scene(**changes):
    params = dict(rects=[], circles=[], bounds=(0, 0, 2400, 2400),
                  pad=10, footprint=(280, 260, 0))
    params.update(changes)
    return nav.CollisionScene(**params)


def ledger(points, headings=None, actions=None):
    """构造完整动作台账，path_axes仍使用生产实现配对。"""
    headings = headings or [math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])) % 360
                            for a, b in zip(points, points[1:])]
    actions = actions or ['FORWARD'] * (len(points)-1)
    def row(kind, a, b, yaw, action):
        return dict(kind=kind, x=a[0], y=a[1], to_x=b[0], to_y=b[1],
                    heading_deg=yaw, action=action, distance_mm=math.dist(a, b))
    rows = [row('START', points[0], points[0], headings[0], 'START')]
    previous = headings[0]
    for a, b, yaw, action in zip(points, points[1:], headings, actions):
        delta = (yaw-previous+180) % 360-180
        if abs(delta) > nav.EPS:
            turn = 'TURN_AROUND' if abs(delta) == 180 else 'TURN_LEFT' if delta > 0 else 'TURN_RIGHT'
            rows.append(row('TURN', a, a, yaw, turn))
        rows.append(row('MOVE', a, b, yaw, action))
        previous = yaw
    segments, corners = nav.path_axes(points, rows)
    return segments, corners, rows


def smooth(points, geometry=None, **changes):
    return smooth_90_corners(*ledger(points, **changes), geometry or scene())


L = [(500, 500), (1500, 500), (1500, 1500)]


class PoseSafeTests(unittest.TestCase):
    def test_real_rectangle_and_margin_respect_yaw_and_boundary(self):
        s = scene()
        self.assertTrue(s.pose_safe(150, 140, 0))  # 完整车体+10mm刚好包含。
        self.assertFalse(s.pose_safe(149.9, 140, 0))
        self.assertFalse(s.pose_safe(150, 140, 45))
        self.assertTrue(s.pose_safe(500, 500, 405))
        self.assertAlmostEqual(s.pose_polygon(500, 500, 0).area, 300*280)

    def test_whole_body_cannot_cover_a_hole_even_when_all_corners_inside(self):
        drivable = [dict(outer=[(0, 0), (2400, 0), (2400, 2400), (0, 2400)],
                        holes=[[(480, 480), (520, 480), (520, 520), (480, 520)]])]
        s = scene(drivable_polygons=drivable)
        body = s.pose_polygon(500, 500, 0)
        self.assertTrue(all(s.allowed.covers(Point(p)) for p in body.exterior.coords[:-1]))
        self.assertFalse(s.pose_safe(500, 500, 0))

    def test_fixed_simulated_dynamic_rectangles_and_circles_all_block(self):
        for group in ('', 'sim_', 'dynamic_'):
            for shape, obstacle in (('rects', (645, 495, 655, 505, group+'rect')),
                                    ('circles', (651, 500, 2, group+'circle'))):
                with self.subTest(group=group, shape=shape):
                    s = scene(**{group+shape: [obstacle]})
                    self.assertFalse(s.pose_safe(500, 500, 0))
                    self.assertTrue(s.pose_safe(500, 900, 90))

    def test_touching_obstacle_rejected(self):
        self.assertFalse(scene(rects=[(650, 450, 660, 550, '接触')]).pose_safe(500, 500, 0))
        self.assertFalse(scene(circles=[(655, 500, 5, '相切')]).pose_safe(500, 500, 0))

    def test_missing_footprint_and_nonfinite_pose_rejected(self):
        self.assertFalse(scene(footprint=None).pose_safe(500, 500, 0))
        for pose in ((math.nan, 500, 0), (500, math.inf, 0), (500, 500, math.nan), ('bad', 500, 0)):
            self.assertFalse(scene().pose_safe(*pose))


class ArcGeometryTests(unittest.TestCase):
    def test_entry_center_exit_and_sampling_both_turn_directions_all_quadrants(self):
        for u in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            for sign in (-1, 1):
                v = (-sign*u[1], sign*u[0])
                p = (1200, 1200)
                points = [(p[0]-500*u[0], p[1]-500*u[1]), p,
                          (p[0]+500*v[0], p[1]+500*v[1])]
                r = smooth(points)
                self.assertEqual(r['smoothing_status'], 'COMPLETE', r)
                arc, = r['arcs']
                self.assertEqual(arc.radius_mm, 120)
                self.assertEqual(arc.entry, (p[0]-120*u[0], p[1]-120*u[1]))
                self.assertEqual(arc.exit, (p[0]+120*v[0], p[1]+120*v[1]))
                self.assertEqual(arc.center, (p[0]-120*u[0]+120*v[0], p[1]-120*u[1]+120*v[1]))
                self.assertEqual(arc.direction, 'CCW' if sign > 0 else 'CW')
                self.assertEqual(arc.sweep_deg, sign*90)
                self.assertAlmostEqual(r['smoothed_length'], 1000-240+math.pi*60)
                self.assertEqual(r['smoothed_points'][0], points[0])
                self.assertEqual(r['smoothed_points'][-1], points[-1])
                for a, b in zip(arc.sample_poses, arc.sample_poses[1:]):
                    delta = abs((b[2]-a[2]+180) % 360-180)
                    self.assertLessEqual(delta, 3+nav.EPS)
                    self.assertLessEqual(math.radians(delta)*arc.radius_mm, 20+nav.EPS)
                    self.assertAlmostEqual(math.dist(arc.center, a[:2]), 120)
                    self.assertTrue(scene().pose_safe(*a))

    def test_radius_order_shrinks_for_short_segments(self):
        r = smooth([(500, 500), (600, 500), (600, 700)])
        self.assertEqual(r['arcs'][0].radius_mm, 100)
        attempts = r['arc_attempts'][0]['attempts']
        self.assertEqual([(a['radius_mm'], a['code']) for a in attempts],
                         [(120, 'SHORT_SEGMENT'), (110, 'SHORT_SEGMENT'), (100, 'SAFE')])

    def test_every_radius_failed_returns_explicit_fallback_and_original_turn(self):
        points = [(500, 500), (550, 500), (550, 550)]
        r = smooth(points)
        self.assertEqual(r['smoothing_status'], 'FALLBACK')
        self.assertEqual(r['arcs'], [])
        fallback, = r['arc_fallbacks']
        self.assertEqual(fallback['code'], 'ALL_RADII_FAILED')
        self.assertEqual(tuple(a['radius_mm'] for a in fallback['attempts']), ARC_RADII_MM)
        self.assertTrue(all(a['code'] == 'SHORT_SEGMENT' for a in fallback['attempts']))
        self.assertEqual(r['smoothed_points'], points)
        self.assertEqual([p['kind'] for p in r['smoothed_primitives']], ['LINE', 'TURN', 'LINE'])
        self.assertEqual(r['smoothed_length'], 100)
        self.assertTrue(r['smoothing_model_safe'])

    def test_adjacent_z_and_u_arcs_cannot_overlap_shared_segment(self):
        for final in ((2000, 700), (500, 700)):
            points = [(500, 500), (1500, 500), (1500, 700), final]
            r = smooth(points)
            self.assertEqual(r['smoothing_status'], 'COMPLETE', r)
            first, second = r['arcs']
            self.assertEqual((first.radius_mm, second.radius_mm), (120, 80))
            self.assertEqual(first.exit, second.entry)
            self.assertEqual([p['kind'] for p in r['smoothed_primitives']], ['LINE', 'ARC', 'ARC', 'LINE'])
            self.assertEqual(r['smoothed_points'][-1], final)

    def test_adjacent_corner_can_fallback_after_first_arc_consumes_space(self):
        r = smooth([(500, 500), (1500, 500), (1500, 650), (2000, 650)])
        self.assertEqual(r['smoothing_status'], 'PARTIAL')
        self.assertEqual(len(r['arcs']), 1)
        self.assertEqual(r['arc_fallbacks'][0]['code'], 'ALL_RADII_FAILED')
        self.assertIn('TURN', [p['kind'] for p in r['smoothed_primitives']])

    def test_backward_arc_and_noncardinal_yaw_preserved(self):
        for yaw, action in ((.2, 'FORWARD'), (180.2, 'BACKWARD')):
            r = smooth(L, headings=[yaw, (yaw+90) % 360], actions=[action, action])
            self.assertEqual(r['smoothing_status'], 'COMPLETE', r)
            arc = r['arcs'][0]
            self.assertEqual(arc.action, action)
            self.assertAlmostEqual(arc.sample_poses[0][2], yaw)
            self.assertAlmostEqual(arc.sample_poses[-1][2], (yaw+90) % 360)

    def test_real_turn_while_strafing_preserves_body_yaw_and_mode(self):
        r = smooth(L, headings=[90, 180], actions=['STRAFE', 'STRAFE'])
        self.assertEqual(r['smoothing_status'], 'COMPLETE', r)
        arc = r['arcs'][0]
        self.assertEqual(arc.action, 'STRAFE')
        self.assertEqual((arc.heading_in_deg, arc.heading_out_deg), (90, 180))
        self.assertEqual(arc.sample_poses[15][2], 135)

    def test_mode_change_corner_and_mixed_actions_are_not_forced_to_turn(self):
        r = smooth(L, headings=[0, 0], actions=['FORWARD', 'STRAFE'])
        self.assertEqual(r['smoothing_status'], 'NO_TURNABLE_CORNER')
        self.assertEqual(r['arcs'], [])
        self.assertEqual(r['arc_fallbacks'][0]['code'], 'NOT_TURNABLE')
        self.assertEqual(r['smoothed_points'], L)
        mixed = smooth(L, headings=[0, 90], actions=['FORWARD', 'BACKWARD'])
        self.assertEqual(mixed['arcs'], [])
        self.assertEqual(mixed['arc_fallbacks'][0]['code'], 'NOT_TURNABLE')

    def test_straight_and_turn_only_preserve_original_trajectory(self):
        r = smooth([(500, 500), (1500, 500)])
        self.assertEqual(r['arcs'], [])
        self.assertEqual(r['smoothing_status'], 'NO_TURNABLE_CORNER')
        rows = [dict(kind='START', x=500, y=500, to_x=500, to_y=500, heading_deg=0, action='START'),
                dict(kind='TURN', x=500, y=500, to_x=500, to_y=500, heading_deg=90, action='TURN_LEFT')]
        r = smooth_90_corners([], [], rows, scene())
        self.assertEqual(r['smoothed_primitives'][0]['kind'], 'TURN')
        self.assertEqual(r['smoothed_points'], [(500, 500)])


class ArcCollisionTests(unittest.TestCase):
    def test_obstacle_shrinks_radius_without_changing_safe_raw_corner(self):
        # 大圆弧在拐角内侧切得更深；原L、原地转向与小圆弧均有足够空间。
        obstacle = (1464, 536, 1, '内侧小障碍')
        s = scene(footprint=(20, 10, 0), pad=0, circles=[obstacle])
        segments, corners, rows = ledger(L)
        self.assertIsNone(s.translation_reason(L[0], L[1], 0))
        self.assertIsNone(s.turn_reason(L[1], 0, 90))
        self.assertIsNone(s.translation_reason(L[1], L[2], 90))
        r = smooth_90_corners(segments, corners, rows, s)
        self.assertTrue(r['arcs'], r)
        self.assertLess(r['arcs'][0].radius_mm, 120)
        self.assertEqual(r['arc_attempts'][0]['attempts'][0]['code'], 'POSE_COLLISION')
        self.assertTrue(s.pose_safe(*r['arcs'][0].sample_poses[15]))

    def test_drivable_hole_prevents_all_arcs_but_keeps_original_l_safe(self):
        drivable = [dict(outer=[(0, 0), (2400, 0), (2400, 2400), (0, 2400)],
                        holes=[[(1400, 512), (1488, 512), (1488, 600), (1400, 600)]])]
        s = scene(footprint=(10, 10, 0), pad=0, drivable_polygons=drivable)
        r = smooth(L, s)
        self.assertEqual(r['smoothing_status'], 'FALLBACK', r)
        self.assertTrue(r['smoothing_model_safe'])
        self.assertTrue(all(a['code'] == 'POSE_COLLISION' for a in r['arc_fallbacks'][0]['attempts']))

    def test_simulated_and_dynamic_obstacles_participate_in_arc_checks(self):
        for group in ('sim_circles', 'dynamic_circles'):
            r = smooth(L, scene(footprint=(20, 10, 0), pad=0,
                                **{group: [(1464, 536, 1, group)]}))
            self.assertTrue(r['arcs'], r)
            self.assertLess(r['arcs'][0].radius_mm, 120)
            self.assertIn(group, r['arc_attempts'][0]['attempts'][0]['reason'])

    def test_between_sample_collision_is_rejected_by_continuous_sweep(self):
        s = scene(footprint=(1, 1, 0), pad=0)
        segments, corners, _rows = ledger(L)
        arc = _candidate(segments[0], segments[1], corners[0], 120)
        angle = math.radians(arc.start_angle_deg+1.5)
        obstacle = (arc.center[0]+120*math.cos(angle), arc.center[1]+120*math.sin(angle), .05, '采样间障碍')
        s = scene(footprint=(1, 1, 0), pad=0, circles=[obstacle])
        self.assertTrue(all(s.pose_safe(*p) for p in arc.sample_poses))
        self.assertEqual(_arc_reason(arc, s, lambda: False), ('SWEEP_COLLISION', '采样间障碍'))

    def test_dense_independent_poses_are_inside_each_sampling_envelope(self):
        # 用0.1°姿态构造真实车体；包络由生产geometry_reason逐段交给测试截获。
        for headings in ((.2, 90.2), (180.2, 270.2)):
            s = scene()
            segments, corners, _rows = ledger(L, headings=list(headings),
                                             actions=['FORWARD' if headings[0] < 1 else 'BACKWARD']*2)
            arc = _candidate(segments[0], segments[1], corners[0], 120)
            envelopes = []
            original = s.geometry_reason
            def capture(shape, *args):
                if len(shape.exterior.coords) > 5:
                    envelopes.append(shape)
                return original(shape, *args)
            with patch.object(s, 'geometry_reason', side_effect=capture):
                self.assertEqual(_arc_reason(arc, s, lambda: False)[0], 'SAFE')
            self.assertEqual(len(envelopes), 30)
            for i, envelope in enumerate(envelopes):
                for j in range(31):
                    offset = i*3+j*.1
                    angle = math.radians(arc.start_angle_deg+offset)
                    x = arc.center[0]+120*math.cos(angle)
                    y = arc.center[1]+120*math.sin(angle)
                    # 独立按长宽/旋转矩阵构造，不调用pose_polygon/Footprint.vertices。
                    c, t = math.cos(math.radians(headings[0]+offset)), math.sin(math.radians(headings[0]+offset))
                    body = Polygon([(x+c*dx-t*dy, y+t*dx+c*dy)
                                    for dx, dy in ((-150, -140), (150, -140), (150, 140), (-150, 140))])
                    self.assertTrue(envelope.covers(body), (headings, offset))

    def test_unsafe_leftover_line_rejects_complete_smoothed_trajectory(self):
        r = smooth(L, scene(rects=[(800, 480, 820, 520, '原直线障碍')]))
        self.assertEqual(r['smoothing_status'], 'UNSAFE_TRAJECTORY')
        self.assertFalse(r['smoothing_model_safe'])
        self.assertEqual(r['arcs'], [])
        self.assertEqual(r['smoothed_points'], [])

    def test_missing_body_and_cancellation_never_force_an_arc(self):
        r = smooth(L, scene(footprint=None))
        self.assertEqual(r['smoothing_status'], 'NO_FOOTPRINT')
        self.assertFalse(r['smoothing_model_safe'])
        r = smooth_90_corners(*ledger(L), scene(), interrupted=lambda: True)
        self.assertEqual(r['smoothing_status'], 'CANCELLED')
        self.assertEqual(r['arcs'], [])


class ArcIntegrationTests(unittest.TestCase):
    def test_plan_adds_arcs_and_preserves_original_search_and_ledger(self):
        kwargs = dict(grid=100, pad=10, rects=[], circles=[], bounds=(0, 0, 2400, 2400),
                      footprint=(280, 260, 0), start_heading_deg=0, goal_heading_deg=90)
        raw = core.plan_path(L[0], L[-1], **kwargs, smooth_arcs=False)
        result = core.plan_path(L[0], L[-1], **kwargs)
        self.assertTrue(result['ok'], result['reason'])
        self.assertTrue(result['smoothing_model_safe'])
        self.assertEqual(result['smoothing_status'], 'COMPLETE')
        self.assertEqual(len(result['arcs']), 1)
        for key in ('points', 'steps', 'segments', 'corners', 'length', 'search_cost', 'trace_cost'):
            self.assertEqual(result[key], raw[key], key)
        self.assertLess(result['smoothed_length'], raw['length'])
        cancelled = threading.Event()
        cancelled.set()
        rejected = core.plan_path(L[0], L[-1], **kwargs, cancel=cancelled)
        self.assertFalse(rejected['ok'])
        self.assertEqual(rejected['arcs'], [])

    def test_dynamic_obstacle_rejects_start_in_search_too(self):
        r = core.plan_path(L[0], L[-1], rects=[], circles=[], footprint=(280, 260, 0),
                           dynamic_rects=[(490, 490, 510, 510, '动态障碍')])
        self.assertEqual(r['code'], 'INVALID_START')
        self.assertIn('动态障碍', r['reason'])

    def setUp(self):
        self.w = make_window()

    def tearDown(self):
        close_window(self.w)

    def present_arc(self):
        self.w.nav_map['rects'] = []
        self.w.nav_map['circles'] = []
        self.w.sim.hold = (1750, 1750)  # 布局(500,500)，布局航向180°。
        self.w.sim.zval = 180
        self.w.sim.make_frame(0)
        ctx = self.w._prepare_plan(1500, 1500)
        self.assertIsNotNone(ctx)
        ctx['kwargs']['goal_heading_deg'] = 90
        r = nav.plan(ctx['start'], ctx['goal'], **ctx['kwargs'])
        self.assertTrue(r['ok'], r['reason'])
        self.assertTrue(r['arcs'], r)
        return self.w._finish_plan(r, ctx), ctx

    def test_ui_draws_arc_preview_exports_geometry_and_does_not_send_motion(self):
        r, _ctx = self.present_arc()
        self.assertEqual(self.w.map_view.points, r['smoothed_points'])
        self.assertEqual(self.w.planned_points, r['points'])
        self.assertIn('圆心', self.w.plan_text.text)
        self.assertIn('圆弧', self.w.plan_info.text)
        self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())
        with tempfile.TemporaryDirectory(prefix='ilhc-arc-export-') as directory:
            path = Path(directory)/'plan.json'
            dialog = SimpleNamespace(getSaveFileName=lambda *a: (str(path), ''))
            with patch.dict(WindowMethods._export_navigation_plan.__globals__, QFileDialog=dialog):
                self.w._export_navigation_plan()
            data = json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(data['arcs'][0]['radius_mm'], 120)
            self.assertEqual(data['arcs'][0]['entry'], list(r['arcs'][0].entry))
            self.assertEqual(data['smoothed_primitives'][1]['kind'], 'ARC')
            self.assertTrue(data['arc_attempts'])
        self.w.sim.make_frame(0)
        self.w._toggle_follow()
        self.assertIsNotNone(self.w.follow)
        self.w._toggle_follow()
        self.assertIsNone(self.w.follow)

    def test_dynamic_snapshot_invalidates_old_result_and_is_used_for_new_plan(self):
        _r, ctx = self.present_arc()
        prior = self.w.planned_result
        self.w.set_dynamic_obstacles(circles=[(500, 500, 5, '动态物体')])
        self.assertIsNone(self.w.planned_result)
        self.assertNotEqual(ctx['signature'], self.w._navigation_signature())
        self.assertIsNone(self.w._finish_plan(prior, ctx))
        self.w.sim.make_frame(0)
        new = self.w.plan_to(1500, 1500)
        self.assertEqual(new['code'], 'INVALID_START')
        self.assertIn('动态物体', new['reason'])
        self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())


if __name__ == '__main__':
    unittest.main()
