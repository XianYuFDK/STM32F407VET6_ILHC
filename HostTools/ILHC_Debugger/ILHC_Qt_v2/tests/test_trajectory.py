"""连续Trajectory：独立位置/切线/累计弧长、整车复检、JSON和界面回归。"""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import core
import navigation_planner as nav
import trajectory as traj
from arc_smoothing import ArcSegment
from tests.headless_support import make_window, close_window, WindowMethods
from tests.test_arc_smoothing import scene, ledger, L
from arc_smoothing import smooth_90_corners


def line(a, b, **kwargs):
    return dict(kind='LINE', start=a, end=b, **kwargs)


def arc(center=(1200, 1200), radius=120, angle=-90, sweep=90):
    def xy(deg):
        rad = math.radians(deg)
        return (center[0]+radius*math.cos(rad), center[1]+radius*math.sin(rad))
    # 台账的车体yaw故意与切线不同，Trajectory必须直接由几何求切线。
    return ArcSegment(0, 0, radius, xy(angle), center, xy(angle+sweep),
                      'CCW' if sweep > 0 else 'CW', angle, sweep, 42, 132, 'STRAFE', ())


def l_primitives(geometry=None):
    result = smooth_90_corners(*ledger(L), geometry or scene())
    if not result['smoothing_model_safe']:
        raise AssertionError(result)
    return result['smoothed_primitives']


class TrajectoryGeometryTests(unittest.TestCase):
    def test_line_segment_object_exact_twenty_mm_grid_and_short_endpoint(self):
        segment = nav.LineSegment(0, (500, 500), (553, 500), 'x', 1, 53, 90, False, 'STRAFE')
        r = core.generate_trajectory([segment], scene())
        self.assertEqual(r['trajectory_status'], 'READY', r)
        samples = r['trajectory']
        self.assertEqual([p['s_mm'] for p in samples], [0, 20, 40, 53])
        self.assertEqual([p['x_mm'] for p in samples], [1750]*4)
        self.assertEqual([p['y_mm'] for p in samples], [1750, 1730, 1710, 1697])
        self.assertEqual([p['field_yaw_deg'] for p in samples], [-90]*4)
        self.assertEqual(set(samples[0]), {'x_mm', 'y_mm', 'field_yaw_deg', 's_mm', 'segment_type'})
        self.assertEqual(r['trajectory_frame_id'], 'FIELD_MM')
        self.assertTrue(r['trajectory_safe'] and r['trajectory_continuous'])

    def test_global_station_grid_continues_across_line_boundary(self):
        r = traj.generate_trajectory([line((500, 500), (535, 500)), line((535, 500), (575, 500))], scene())
        self.assertEqual(r['trajectory_status'], 'READY', r)
        self.assertEqual([p['s_mm'] for p in r['trajectory']], [0, 20, 35, 40, 60, 75])

    def test_line_arc_line_length_endpoints_and_tangent_join(self):
        primitives = l_primitives()
        r = traj.generate_trajectory(primitives, scene())
        self.assertEqual(r['trajectory_status'], 'READY', r)
        samples = r['trajectory']
        self.assertAlmostEqual(r['trajectory_length_mm'], 1760+120*math.pi/2)
        self.assertEqual((samples[0]['x_mm'], samples[0]['y_mm']), (1750, 1750))
        self.assertEqual((samples[-1]['x_mm'], samples[-1]['y_mm']), (750, 750))
        self.assertEqual((samples[0]['field_yaw_deg'], samples[-1]['field_yaw_deg']), (-90, -180))
        self.assertEqual(len({p['s_mm'] for p in samples}), len(samples))
        entry = next(p for p in samples if abs(p['s_mm']-880) < nav.EPS)
        exit_ = next(p for p in samples if abs(p['s_mm']-(880+120*math.pi/2)) < nav.EPS)
        self.assertEqual(entry['segment_type'], 'ARC')
        self.assertEqual(exit_['segment_type'], 'LINE')
        for before, after in zip(samples, samples[1:]):
            self.assertGreater(after['s_mm'], before['s_mm'])
            self.assertLessEqual(after['s_mm']-before['s_mm'], 20+nav.EPS)
            self.assertLessEqual(math.dist((before['x_mm'], before['y_mm']), (after['x_mm'], after['y_mm'])),
                                 after['s_mm']-before['s_mm']+nav.EPS)
        for p in samples:
            if p['segment_type'] == 'ARC':
                offset = (p['s_mm']-880)/120  # 独立真弧长公式，不用生成器helper。
                lx, ly = 1380+120*math.sin(offset), 620-120*math.cos(offset)
                self.assertAlmostEqual(p['x_mm'], 2250-ly)
                self.assertAlmostEqual(p['y_mm'], 2250-lx)
                self.assertAlmostEqual(p['field_yaw_deg'], -90-math.degrees(offset))

    def test_arc_segment_object_cw_ccw_all_radii_uses_true_arclength(self):
        for radius in (120, 110, 100, 90, 80, 70, 60):
            for sweep in (90, -90):
                a = arc(radius=radius, sweep=sweep)
                r = traj.generate_trajectory([a], scene())
                self.assertEqual(r['trajectory_status'], 'READY', r)
                self.assertAlmostEqual(r['trajectory_length_mm'], radius*math.pi/2)
                points = r['trajectory']
                for p in points:
                    lx, ly = 2250-p['y_mm'], 2250-p['x_mm']
                    self.assertAlmostEqual(math.hypot(lx-1200, ly-1200), radius)
                    offset = math.copysign(math.degrees(p['s_mm']/radius), sweep)
                    expected = -90-(-90+offset+math.copysign(90, sweep))
                    self.assertAlmostEqual((p['field_yaw_deg']-expected+180) % 360-180, 0)

    def test_unwrap_179_to_minus179_and_multiple_revolutions(self):
        self.assertEqual(traj.unwrap_degrees([179, -179, -178, 179]), [179, 181, 182, 179])
        for sweep in (90, -90):
            arcs = [arc(angle=181+i*sweep, sweep=sweep) for i in range(4)]
            r = traj.generate_trajectory(arcs, scene())
            self.assertEqual(r['trajectory_status'], 'READY', r)
            points = r['trajectory']
            self.assertAlmostEqual(points[-1]['field_yaw_deg']-points[0]['field_yaw_deg'], -4*sweep)
            self.assertTrue(all(abs(b['field_yaw_deg']-a['field_yaw_deg']) < 20
                                for a, b in zip(points, points[1:])))
        # CW输入起始切线场地179°，下一采样已跨越180°，不会写成-179°。
        r = traj.generate_trajectory([arc(angle=181, sweep=-90)], scene())
        self.assertAlmostEqual(r['trajectory'][0]['field_yaw_deg'], 179)
        self.assertGreater(r['trajectory'][1]['field_yaw_deg'], 180)

    def test_adjacent_arcs_exact_join_is_single_station(self):
        points = [(500, 500), (1500, 500), (1500, 700), (2000, 700)]
        smooth = smooth_90_corners(*ledger(points), scene())
        r = traj.generate_trajectory(smooth['smoothed_primitives'], scene())
        self.assertEqual(r['trajectory_status'], 'READY', r)
        join_s = 880+120*math.pi/2
        joins = [p for p in r['trajectory'] if abs(p['s_mm']-join_s) < nav.EPS]
        self.assertEqual(len(joins), 1)
        self.assertEqual(joins[0]['segment_type'], 'ARC')
        self.assertAlmostEqual(joins[0]['field_yaw_deg'], -180)

    def test_fallback_turn_and_sharp_corner_are_not_fake_continuous_paths(self):
        points = [(500, 500), (550, 500), (550, 550)]
        smooth = smooth_90_corners(*ledger(points), scene())
        r = traj.generate_trajectory(smooth['smoothed_primitives'], scene())
        self.assertEqual(r['trajectory_status'], 'FALLBACK_REQUIRED')
        self.assertIn('停转', r['trajectory_reason'])
        r2 = traj.generate_trajectory([line(L[0], L[1]), line(L[1], L[2])], scene())
        self.assertEqual(r2['trajectory_status'], 'FALLBACK_REQUIRED')
        self.assertEqual(r2['trajectory'], [])
        self.assertFalse(r2['trajectory_continuous'])

    def test_empty_missing_body_invalid_geometry_and_sample_resource_limit(self):
        self.assertEqual(traj.generate_trajectory([], scene())['trajectory_status'], 'EMPTY')
        self.assertEqual(traj.generate_trajectory([line((500, 500), (600, 500))],
                                                 scene(footprint=None))['trajectory_status'], 'NO_FOOTPRINT')
        for value in (0, -1, 21, math.nan):
            self.assertEqual(traj.generate_trajectory([line((500, 500), (600, 500))], scene(),
                                                      spacing_mm=value)['trajectory_status'], 'INVALID_INPUT')
        bad = arc().as_dict()
        for changes in ({'radius_mm': -1}, {'sweep_deg': 45}, {'entry': (0, 0)}, {'direction': 'CW'}):
            r = traj.generate_trajectory([dict(bad, **changes)], scene())
            self.assertEqual(r['trajectory_status'], 'INVALID_INPUT', r)
        r = traj.generate_trajectory([line((500, 500), (600, 500)), line((601, 500), (700, 500))], scene())
        self.assertEqual(r['trajectory_status'], 'DISCONNECTED')
        with patch.object(traj, 'MAX_TRAJECTORY_POINTS', 10):
            self.assertEqual(traj.generate_trajectory([line((500, 500), (900, 500))], scene())['trajectory_status'], 'RESOURCE_LIMIT')


class TrajectorySafetyTests(unittest.TestCase):
    def test_new_tangent_yaw_is_checked_instead_of_original_strafe_heading(self):
        obstacle = (1100, 490, 1110, 510, '旧横移车头姿态没有覆盖此障碍')
        s = scene(footprint=(280, 60, 90), pad=0, rects=[obstacle])
        self.assertIsNone(s.translation_reason((500, 500), (1000, 500), 90))
        r = traj.generate_trajectory([line((500, 500), (1000, 500), heading_deg=90, action='STRAFE')], s)
        self.assertEqual(r['trajectory_status'], 'UNSAFE')
        self.assertIn('旧横移', r['trajectory_reason'])
        self.assertEqual(r['trajectory'], [])

    def test_final_line_sweep_rejects_obstacle_between_export_samples(self):
        s = scene(footprint=(1, 1, 0), pad=0, circles=[(510.5, 500, .01, '采样间圆')])
        self.assertTrue(s.pose_safe(500, 500, 0) and s.pose_safe(520, 500, 0))
        r = traj.generate_trajectory([line((500, 500), (520, 500))], s)
        self.assertEqual(r['trajectory_status'], 'UNSAFE')
        self.assertIn('直线扫掠', r['trajectory_reason'])

    def test_final_arc_sweep_rejects_between_export_and_check_samples(self):
        a = arc()
        primitives = [a]
        empty = scene(footprint=(1, 1, 0), pad=0)
        safe = traj.generate_trajectory(primitives, empty)
        angle = math.radians(-88.5)
        obstacle = (1200+120*math.cos(angle), 1200+120*math.sin(angle), .05, '弧采样间圆')
        s = scene(footprint=(1, 1, 0), pad=0, dynamic_circles=[obstacle])
        self.assertTrue(all(s.pose_safe(2250-p['y_mm'], 2250-p['x_mm'], -90-p['field_yaw_deg'])
                            for p in safe['trajectory']))
        r = traj.generate_trajectory(primitives, s)
        self.assertEqual(r['trajectory_status'], 'UNSAFE')
        self.assertIn('圆弧扫掠', r['trajectory_reason'])

    def test_every_pose_and_interval_rechecked_against_latest_scene(self):
        primitives = l_primitives()
        r = traj.generate_trajectory(primitives, scene())
        s = scene(dynamic_rects=[(1000, 490, 1010, 510, '新动态障碍')])
        check = core.validate_trajectory(r['trajectory'], primitives, s)
        self.assertFalse(check['ok'])
        self.assertEqual(check['code'], 'UNSAFE')
        self.assertIn('新动态障碍', check['reason'])

    def test_full_vehicle_and_drivable_hole_not_just_centers(self):
        s = scene(drivable_polygons=[dict(outer=[(0, 0), (2400, 0), (2400, 2400), (0, 2400)],
                                         holes=[[(550, 580), (560, 580), (560, 590), (550, 590)]])])
        self.assertTrue(s.allowed.covers(s.pose_polygon(500, 500, 0).centroid))
        r = traj.generate_trajectory([line((500, 500), (1000, 500))], s)
        self.assertEqual(r['trajectory_status'], 'UNSAFE')
        self.assertFalse(r['trajectory_safe'])

    def test_dense_independent_rectangle_poses_safe_along_complete_exported_path(self):
        primitives = l_primitives()
        s = scene()
        result = traj.generate_trajectory(primitives, s)
        self.assertTrue(result['trajectory_safe'])
        for angle in [i*.1 for i in range(901)]:
            theta = math.radians(angle)
            x, y = 1380+120*math.sin(theta), 620-120*math.cos(theta)
            self.assertTrue(s.pose_safe(x, y, angle))
        self.assertTrue(core.validate_trajectory(result['trajectory'], primitives, s)['ok'])

    def test_tampered_sample_position_yaw_type_and_station_rejected(self):
        primitives = l_primitives()
        r = traj.generate_trajectory(primitives, scene())
        for changes in ({'x_mm': 1751}, {'field_yaw_deg': 270}, {'segment_type': 'ARC'},
                        {'s_mm': 0}, {'s_mm': 40}, {'s_mm': math.nan}):
            samples = copy.deepcopy(r['trajectory'])
            samples[1].update(changes)
            check = traj.validate_trajectory(samples, primitives, scene())
            self.assertFalse(check['ok'], changes)
        wrapped = copy.deepcopy(r['trajectory'])
        wrapped[-1]['field_yaw_deg'] += 360
        self.assertFalse(traj.validate_trajectory(wrapped, primitives, scene())['ok'])

    def test_cancellation_during_generation_and_final_check_never_publishes(self):
        r = traj.generate_trajectory(l_primitives(), scene(), interrupted=lambda: True)
        self.assertEqual(r['trajectory_status'], 'CANCELLED')
        self.assertEqual(r['trajectory'], [])
        s = scene()
        cancelled = [False]
        original = s.pose_safe
        def cancel_after_pose(*pose):
            value = original(*pose)
            cancelled[0] = True
            return value
        with patch.object(s, 'pose_safe', side_effect=cancel_after_pose):
            r = traj.generate_trajectory(l_primitives(), s, interrupted=lambda: cancelled[0])
        self.assertEqual(r['trajectory_status'], 'CANCELLED')
        self.assertFalse(r['trajectory_safe'])


class TrajectoryIntegrationTests(unittest.TestCase):
    def test_plan_automatically_returns_safe_trajectory_without_changing_raw_ledger(self):
        options = dict(grid=100, pad=10, rects=[], circles=[], bounds=(0, 0, 2400, 2400),
                       footprint=(280, 260, 0), goal_heading_deg=90)
        result = core.plan_path(L[0], L[-1], **options)
        raw = core.plan_path(L[0], L[-1], **options, smooth_arcs=False)
        self.assertEqual(result['trajectory_status'], 'READY', result)
        self.assertFalse(result['hardware_ready'])
        for key in ('points', 'steps', 'segments', 'corners', 'search_cost'):
            self.assertEqual(result[key], raw[key])

    def test_json_export_five_fields_and_rejects_fallback(self):
        r = traj.generate_trajectory([arc()], scene())
        with tempfile.TemporaryDirectory(prefix='ilhc-trajectory-json-') as directory:
            path = Path(directory)/'trajectory.json'
            core.export_trajectory_json(path, r)
            data = json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(data['frame_id'], 'FIELD_MM')
            self.assertFalse(data['hardware_ready'])
            self.assertEqual(data['trajectory'], r['trajectory'])
            self.assertEqual(set(data['trajectory'][0]), {'x_mm', 'y_mm', 'field_yaw_deg', 's_mm', 'segment_type'})
            rejected = traj.generate_trajectory([line(L[0], L[1]), line(L[1], L[2])], scene())
            with self.assertRaises(ValueError):
                core.export_trajectory_json(path, rejected)

    def setUp(self):
        self.w = make_window()

    def tearDown(self):
        close_window(self.w)

    def present(self):
        self.w.nav_map['rects'] = []
        self.w.nav_map['circles'] = []
        self.w.sim.hold = (1750, 1750)
        self.w.sim.zval = 180
        self.w.sim.make_frame(0)
        ctx = self.w._prepare_plan(1500, 1500)
        ctx['kwargs']['goal_heading_deg'] = 90
        result = nav.plan(ctx['start'], ctx['goal'], **ctx['kwargs'])
        self.assertEqual(result['trajectory_status'], 'READY', result)
        self.w._finish_plan(result, ctx)
        return self.w.planned_result, ctx

    def test_ui_arrows_and_both_exports_no_motion_commands(self):
        r, ctx = self.present()
        self.assertEqual(self.w.map_view.trajectory, r['trajectory'])
        self.assertIn('Trajectory', self.w.plan_text.text)
        with tempfile.TemporaryDirectory(prefix='ilhc-trajectory-ui-') as directory:
            for name, method in (('trajectory.json', '_export_trajectory'), ('plan.json', '_export_navigation_plan')):
                path = Path(directory)/name
                dialog = SimpleNamespace(getSaveFileName=lambda *a: (str(path), ''))
                with patch.dict(getattr(WindowMethods, method).__globals__, QFileDialog=dialog):
                    getattr(self.w, method)()
                data = json.loads(path.read_text(encoding='utf-8'))
                self.assertEqual(data['trajectory'], r['trajectory'])
            data = json.loads((Path(directory)/'trajectory.json').read_text(encoding='utf-8'))
            self.assertEqual(data['metadata']['collision_snapshot']['footprint'], list(ctx['kwargs']['footprint']))
        self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())
        self.assertIsNone(self.w.sim.goto)

    def test_ui_refuses_corrupted_or_stale_trajectory_export(self):
        r, ctx = self.present()
        r['trajectory'][1]['field_yaw_deg'] += 360
        dialog = SimpleNamespace(getSaveFileName=lambda *a: self.fail('不安全Trajectory不能弹出导出对话框'))
        with patch.dict(WindowMethods._export_trajectory.__globals__, QFileDialog=dialog):
            self.w._export_trajectory()
        self.assertFalse(r['trajectory_safe'])
        self.assertIsNone(self.w.map_view.trajectory)
        self.assertIn('复检失败', self.w.logs[-1])
        self.w.set_dynamic_obstacles(circles=[(800, 800, 5, '新动态')])
        self.w.planned_result, self.w._planned_context = r, ctx  # 过期结果不能恢复导出。
        with patch.dict(WindowMethods._export_trajectory.__globals__, QFileDialog=dialog):
            self.w._export_trajectory()
        self.assertIsNone(self.w.planned_result)
        self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())


if __name__ == '__main__':
    unittest.main()
