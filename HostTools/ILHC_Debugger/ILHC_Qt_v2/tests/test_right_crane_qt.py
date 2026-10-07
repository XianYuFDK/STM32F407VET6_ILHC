"""真实Qt站点点击和快速目标的作业航向，使用未启动模拟器。"""
import argparse
import os
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import main
import core
import competition_simulation as competition


class RightCraneQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.w = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.w.nav_map = competition.load_profile()
        self.w.sim = core.Simulator(self.w.frame_q, self.w.line_q, self.w.urgent_q)
        self.w.sim.handle_line('ZERO')
        self.w.sim.make_frame(0)

    def tearDown(self):
        self.w.close()
        self.w._planner_pool.shutdown(wait=True, cancel_futures=True)

    def test_station_click_overrides_free_heading_but_ordinary_click_does_not(self):
        for station, expected in (('raw', 180), ('rough', 0), ('storage', 259.695153531234)):
            with self.subTest(station=station), patch.object(self.w._planner_pool, 'submit') as submit:
                self.w._request_plan(*self.w.nav_map['competition']['stations'][station])
                self.assertEqual(submit.call_args.kwargs['goal_heading_deg'], expected)
                self.assertEqual(self.w._plan_context_pending['work_station'], station)
                self.w._plan_future = None
        with patch.object(self.w._planner_pool, 'submit') as submit:
            self.w._request_plan(1200, 1200)
            self.assertNotIn('work_station', self.w._plan_context_pending)
            self.assertEqual(submit.call_args.kwargs['goal_heading_deg'],
                             submit.call_args.kwargs['start_heading_deg'])
            self.w._plan_future = None

    def test_quick_targets_use_right_crane_heading_for_clearance_and_route(self):
        for name, anchor, outward in core.QUICK_ANCHORS:
            with self.subTest(name=name), patch.object(self.w._planner_pool, 'submit') as submit:
                result = self.w._request_plan_anchor(name, anchor, outward)
                self.assertTrue(result['ok'])
                heading = submit.call_args.kwargs['goal_heading_deg']
                required = {'原料区': 180, '粗加工区': 0, '暂存区': 259.695153531234}[name]
                self.assertLess(abs((heading-required+180)%360-180), 1e-7)
                point = result['point']
                scene = competition.collision_scene(self.w.nav_map, self.w.plan_pad_spin.value()*10)
                self.assertIsNone(scene.pose_reason(*point, heading))
                self.w._plan_future = None

    def test_simulation_preparation_preserves_dynamic_and_fixed_geometry(self):
        self.w.nav_map.pop('competition')
        self.w.nav_map['dynamic_circles'] = [[1200, 1200, 25, '当前动态圆柱']]
        with patch.object(self.w._planner_pool, 'submit') as submit:
            self.w._start_competition()
            data = submit.call_args.args[1]
            self.assertEqual(data['dynamic_circles'], [[1200, 1200, 25, '当前动态圆柱']])
            self.assertEqual(data['rects'], self.w.nav_map['rects'])
            self.assertEqual(data['circles'], self.w.nav_map['circles'])
            self.w._competition_future = None

    def test_qt_float_readback_reuses_cli_integer_margin_cache(self):
        from route_store import clear_memory
        competition.compile_match(self.w.nav_map, margin=10, coordinate_mode=True)
        clear_memory()
        with patch('coordinate_navigation.plan_route', side_effect=AssertionError('Qt必须复用已生成路线')):
            self.w._start_competition()
            match = self.w._competition_future.result(timeout=5)
            self.assertEqual(match['planning_stats']['search_calls'], 0)
            self.assertTrue(all(leg['route']['route_reuse']['source']=='DISK' for leg in match['legs']))
            self.w._competition_future = None


if __name__ == '__main__':
    unittest.main(verbosity=2)
