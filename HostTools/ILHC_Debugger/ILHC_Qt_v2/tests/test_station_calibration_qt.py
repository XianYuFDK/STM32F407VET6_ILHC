"""实测目标与比赛配置一致；设备冲突和新增障碍不能被快捷按钮绕过。"""
import argparse
import copy
import os
from pathlib import Path
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import competition_simulation as competition
import core
import main
from work_orientation import work_heading


class StationCalibrationQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.w = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.w.nav_map = competition.load_profile(Path(main.BASE_DIR)/'navigation_map.json')
        self.w.sim = core.Simulator(self.w.frame_q, self.w.line_q, self.w.urgent_q)
        self.w.sim.handle_line('ZERO')
        self.w.sim.make_frame(0)

    def tearDown(self):
        self.w.close()
        self.w._planner_pool.shutdown(wait=True, cancel_futures=True)

    def test_display_tracks_loaded_device_geometry_without_old_background_outlines(self):
        data = competition.load_profile(Path(main.BASE_DIR)/'navigation_map.json')
        view = self.w.map_view
        view.set_navigation_map(data)
        self.assertTrue(all(not item.isVisible() for item in view._device_background))
        rough = view.device_geometry['粗加工区设备']
        self.assertEqual((rough[2]-rough[0], rough[3]-rough[1]), (580,150))
        self.assertEqual(view.device_geometry['原料区圆盘'], (1200,2470,150))
        count = len(view.scene_obj.items())
        data['circles'][0][0] += 20
        view.set_navigation_map(data)
        self.assertEqual(view.device_geometry['原料区圆盘'], (1220,2470,150))
        self.assertEqual(len(view.scene_obj.items()),count)

    def test_quick_buttons_use_exact_measured_station_and_right_crane_heading(self):
        for name, station, field in (('原料区','raw',(100,1050)), ('粗加工区','rough',(1900,1050)), ('暂存区','storage',(1100,1900))):
            with self.subTest(station=station), patch.object(self.w._planner_pool, 'submit') as submit:
                result = self.w._request_plan_anchor(name, (0,0), (1,0))
                self.assertTrue(result['ok'])
                self.assertEqual(core.layout_to_field(*result['point']), field)
                self.assertEqual(submit.call_args.kwargs['goal_heading_deg'],
                                 work_heading(self.w.nav_map['competition'], station))
                self.assertEqual(self.w._plan_context_pending['work_station'], station)
                self.w._plan_future = None

    def test_explicit_conflicting_profile_is_rejected_instead_of_moving_measured_point(self):
        data = copy.deepcopy(self.w.nav_map)
        rect = next(r for r in data['rects'] if r[4]=='粗加工区设备')
        rect[1], rect[3] = 150, 300
        self.w.nav_map = data
        with patch.object(self.w._planner_pool, 'submit') as submit:
            result = self.w._request_plan_anchor('粗加工区', (0,0), (1,0))
            self.assertFalse(result['ok'])
            self.assertEqual(result['reason'], '粗加工区设备')
            submit.assert_not_called()

    def test_new_obstacle_at_station_is_rejected_before_submission(self):
        self.w.sim_obstacles = [tuple(self.w.nav_map['competition']['stations']['storage'])]
        with patch.object(self.w._planner_pool, 'submit') as submit:
            result = self.w._request_plan_anchor('暂存区', (0,0), (1,0))
            self.assertFalse(result['ok'])
            self.assertIn('模拟障碍', result['reason'])
            submit.assert_not_called()

    def test_increased_clearance_rejects_measured_point_without_silently_shifting_it(self):
        self.w.plan_pad_spin.setValue(15)
        with patch.object(self.w._planner_pool, 'submit') as submit:
            result = self.w._request_plan_anchor('粗加工区', (0,0), (1,0))
            self.assertFalse(result['ok'])
            submit.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
