"""Real Qt map-click smoke tests (requires installed GUI dependencies).

Uses an offscreen QApplication, QTest mouse events, the real FieldView signal,
MainWindow, ThreadPoolExecutor and result polling timer. Never opens a serial port.
"""
from __future__ import annotations
from concurrent.futures import CancelledError
import argparse
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtTest import QTest, QSignalSpy
import core
import main


class MapClickQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ilhc-qt-click-')
        self.w = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.w._planner_error_log = Path(self.temp.name) / 'planner.log'
        # Unstarted thread: production simulator state, no background motion.
        self.w.sim = core.Simulator(self.w.frame_q, self.w.line_q, self.w.urgent_q)
        self.w.sim.handle_line('ZERO')
        self.w.sim.make_frame(0.0)
        self.w._select_page(2)
        self.w.show()
        self.app.processEvents()
        QTest.qWait(30)
        self.w.sim.make_frame(0.0)
        self.errors = []
        self.excepthook = patch.object(main.sys, 'excepthook',
                                      side_effect=lambda t, e, tb: self.errors.append((t, e, tb)))
        self.excepthook.start()

    def tearDown(self):
        self.w.close()
        self.w._planner_pool.shutdown(wait=True, cancel_futures=True)
        self.app.processEvents()
        self.excepthook.stop()
        self.temp.cleanup()

    def click_layout(self, x, y):
        self.w.sim.make_frame(0.0)
        view = self.w.map_view
        pos = view.mapFromScene(main.QPointF(x, view.sy(y)))
        self.assertTrue(view.viewport().rect().contains(pos))
        spy = QSignalSpy(view.gotoRequested)
        QTest.mouseClick(view.viewport(), main.Qt.LeftButton, main.Qt.NoModifier, pos)
        self.assertEqual(spy.count(), 1, 'FieldView click signal was not emitted')
        return spy.at(0)

    def wait_plan(self):
        deadline = time.monotonic() + 15.0
        while self.w._plan_future is not None and time.monotonic() < deadline:
            # 必须用 processEvents+sleep 等：QTest.qWait 期间工作线程拿不到 GIL，
            # 再快的规划也会被判成"规划已取消或超过时限"。
            self.app.processEvents()
            time.sleep(0.01)
        self.assertIsNone(self.w._plan_future, 'no completion from Qt polling timer')
        self.assertEqual(self.errors, [], 'unhandled Qt slot exception')

    def assert_no_command_sent(self):
        """规划只算不下发：队列里允许出现 1Hz 参数回读（GET …）与心跳，
        但绝不能有给机构下命令的条目。

        不要用 line_q.empty()：参数回读轮询每秒会往同一个队列放一条 GET，
        真串口/真实事件循环下该断言必然误报（曾导致 run_tests.py --qt 长期变红，
        把真正的回归盖住）。
        """
        queued = list(self.w.line_q.queue)
        self.assertEqual(core.commanding_commands(queued), [],
                         '规划模式不得下发机构命令：%s' % queued)
        self.assertEqual(core.commanding_commands(list(self.w.urgent_q.queue)), [],
                         '紧急队列里不得有机构命令：%s' % list(self.w.urgent_q.queue))
        for item in queued:
            self.assertTrue(str(item).upper().startswith('GET '), '意外的队列条目：%s' % item)

    def test_mouse_click_reaches_planner_and_draws_route(self):
        self.click_layout(330, 1200)
        self.wait_plan()
        self.assertIsNotNone(self.w.planned_result, self.w.map_status.text())
        self.assertTrue(self.w.planned_result['ok'], self.w.planned_result)
        self.assertGreater(self.w.map_view.path_item.path().elementCount(), 1)
        self.assertIn('已规划', self.w.map_status.text())
        self.assert_no_command_sent()
        self.assertIsNone(self.w.sim.goto)

    def test_invalid_click_displays_failure_without_motion(self):
        self.click_layout(700, 700)
        self.wait_plan()
        self.assertIn('规划失败', self.w.map_status.text())
        self.assertEqual(self.w.planned_points, [])
        self.assert_no_command_sent()

    def test_safe_arc_preview_uses_samples_and_exports_arc_geometry(self):
        self.w.nav_map['rects'] = []
        self.w.nav_map['circles'] = []
        self.w.sim.hold = (1750, 1750)
        self.w.sim.zval = 180
        self.w.sim.make_frame(0)
        ctx = self.w._prepare_plan(1500, 1500)
        self.assertIsNotNone(ctx)
        ctx['kwargs']['goal_heading_deg'] = 90
        result = core.plan_path(ctx['start'], ctx['goal'], **ctx['kwargs'])
        self.assertTrue(result['arcs'], result)
        self.w._finish_plan(result, ctx)
        view = self.w.map_view
        path = view.path_item.path()
        self.assertEqual(path.elementCount(), len(result['smoothed_points']))
        for i, (x, y) in enumerate(result['smoothed_points']):
            self.assertAlmostEqual(path.elementAt(i).x, x)
            self.assertAlmostEqual(path.elementAt(i).y, view.sy(y))
        self.assertIn('圆心', self.w.plan_text.toPlainText())
        export = Path(self.temp.name)/'arc.json'
        with patch.object(main.QFileDialog, 'getSaveFileName', return_value=(str(export), '')):
            self.w._export_navigation_plan()
        data = main.json.loads(export.read_text(encoding='utf-8'))
        self.assertEqual(data['arcs'][0]['direction'], result['arcs'][0].direction)
        self.assertEqual(data['arcs'][0]['radius_mm'], 120)
        self.assertTrue(result['trajectory_safe'])
        arrows = view.trajectory_arrows.path()
        self.assertGreater(arrows.elementCount(), 0)
        self.assertEqual(arrows.elementCount() % 5, 0)
        first = result['trajectory'][0]
        # 实际图元首箭头的轴向，独立检查场地→布局→Qt两次镜像的方向。
        yaw = main.math.radians(first['field_yaw_deg'])
        self.assertAlmostEqual(arrows.elementAt(1).x-arrows.elementAt(0).x, -36*main.math.sin(yaw))
        self.assertAlmostEqual(arrows.elementAt(1).y-arrows.elementAt(0).y, 36*main.math.cos(yaw))
        trajectory_export = Path(self.temp.name)/'trajectory.json'
        button, = [b for b in self.w.findChildren(main.QPushButton) if b.text() == '导出Trajectory JSON']
        with patch.object(main.QFileDialog, 'getSaveFileName', return_value=(str(trajectory_export), '')):
            button.click()
        exported = main.json.loads(trajectory_export.read_text(encoding='utf-8'))
        self.assertEqual(exported['trajectory'], result['trajectory'])
        self.assertEqual(exported['frame_id'], 'FIELD_MM')
        self.assertFalse(exported['hardware_ready'])
        self.w.sim.make_frame(0)
        self.w._toggle_follow()
        self.assertIsNotNone(self.w.follow, self.w.map_status.text())
        self.assertTrue(self.w.sim.navigation_snapshot()['tracking'])
        self.assertGreater(view.reference_item.path().elementCount(), 0)
        self.w._clear_path()
        self.assertIsNone(self.w.follow)
        self.assertFalse(self.w.sim.navigation_snapshot()['tracking'])
        self.assertEqual(view.reference_item.path().elementCount(), 0)
        self.assert_no_command_sent()

    def start_continuous_arc(self):
        self.w.nav_map['rects'] = []
        self.w.nav_map['circles'] = []
        self.w.sim.hold = (1750, 1750)
        self.w.sim.zval = 180
        self.w.sim.make_frame(0)
        self.w.map_yaw_combo.setCurrentIndex(3)  # FIELD180° -> LAYOUT90°，使用真实UI目标航向。
        result = self.w.plan_to(1500, 1500)
        self.assertTrue(result['trajectory_safe'], result)
        self.w.sim.make_frame(0)
        self.w._toggle_follow()
        self.assertIsNotNone(self.w.follow, self.w.map_status.text())
        return result

    def test_continuous_arc_tracks_to_final_stop_and_draws_four_layers(self):
        result = self.start_continuous_arc()
        view, sim = self.w.map_view, self.w.sim
        self.assertEqual(view.skeleton_item.path().elementCount(), len(result['points']))
        self.assertGreater(view.path_item.path().elementCount(), 3)
        self.assertGreater(view.reference_item.path().elementCount(), 0)
        first_ref = sim.navigation_snapshot()['reference']
        self.assertEqual(first_ref['s_mm'], 100)
        scene = self.w._scene_for_context(self.w._planned_context)
        previous, progress = None, 0
        with patch.object(core.random, 'gauss', return_value=0):
            for index in range(800):
                self.w.frame_q.put((self.w.t0_monotonic+index*.02, sim.make_frame(index*.02)))
                self.w._process_frames()
                snap = sim.navigation_snapshot()
                lx, ly = self.w._ops_to_field(*snap['hold'])
                actual = lx, ly, self.w._layout_yaw_for(snap['yaw'])
                self.assertIsNone(scene.pose_reason(*actual))
                if previous is not None:
                    self.assertIsNone(scene.moving_pose_reason(previous, actual))
                previous = actual
                self.assertGreaterEqual(snap['progress_s_mm'], progress)
                progress = snap['progress_s_mm']
                self.assertIsNone(sim.goto)
                if snap['reference'] and snap['reference']['segment_type'] != 'STOP':
                    self.assertGreater(snap['speed_mm_s'], 0)
                self.w._follow_step()
                if self.w.follow is None:
                    break
        self.w._update_map_trail()
        self.assertIsNone(self.w.follow)
        self.assertIn('最终STOP', self.w.map_status.text())
        self.assertEqual(snap['settled_frames'], 10)
        self.assertEqual(snap['fault'], '')
        self.assertLess(main.math.dist((lx, ly), (1500, 1500)), 1)
        self.assertGreater(view.trail_item.path().elementCount(), 1)
        self.assertGreater(view.skeleton_item.path().elementCount(), 1)
        self.assertGreater(view.path_item.path().elementCount(), 3)
        self.assertEqual(view.reference_item.path().elementCount(), 0)
        self.assert_no_command_sent()

    def start_competition(self, zone=1):
        self.w.zone_combo.setCurrentIndex(zone-1)
        button, = [b for b in self.w.findChildren(main.QPushButton) if b.text() == '一键比赛模拟']
        button.click()
        deadline = time.monotonic()+12
        while self.w._competition_future is not None and time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(.01)
        self.assertIsNone(self.w._competition_future, self.w.competition_status.text())
        self.assertIsNotNone(self.w.competition, self.w.competition_status.text())
        self.assertTrue(self.w.competition.active, self.w.competition.reason)
        return self.w.competition

    def test_one_key_complete_match_both_start_zones_and_real_json_export(self):
        for zone in (1, 2):
            runner = self.start_competition(zone)
            self.assertEqual(runner.display_code, '')
            self.assertGreater(self.w.map_view.skeleton_item.path().elementCount(), 10)
            self.assertGreater(self.w.map_view.path_item.path().elementCount(), 50)
            self.assertFalse(self.w.competition_start.isEnabled())
            with patch.object(core.random, 'gauss', return_value=0):
                for i in range(9200):
                    self.w.frame_q.put((self.w.t0_monotonic+i*.02, self.w.sim.make_frame(i*.02)))
                    self.w._process_frames()
                    self.w._poll_competition()
                    if not runner.active:
                        break
            self.assertEqual(runner.status, 'COMPLETE', runner.reason)
            self.assertEqual((runner.grabs, runner.placements), (12, 12))
            self.assertEqual(runner.storage, {1: [1, 1], 2: [5, 5], 3: [6, 6]})
            self.assertEqual(core.field_to_layout(*self.w.sim.hold), tuple(runner.match['home']))
            self.assertIn('COMPLETE', self.w.competition_status.text())
            self.assertIn('156+123+516+231', self.w.competition_status.text())
            self.assertTrue(self.w.competition_start.isEnabled())
            self.w._update_map_trail()
            self.assertGreater(self.w.map_view.trail_item.path().elementCount(), 500)
            self.assertEqual(self.w.map_view.reference_item.path().elementCount(), 0)
            export = Path(self.temp.name)/('match%d.json' % zone)
            button, = [b for b in self.w.findChildren(main.QPushButton) if b.text() == '导出比赛记录']
            with patch.object(main.QFileDialog, 'getSaveFileName', return_value=(str(export), '')):
                button.click()
            saved = main.json.loads(export.read_text(encoding='utf-8'))
            self.assertFalse(saved['hardware_ready'])
            self.assertEqual(saved['correct_grabs'], 12)
            self.assertEqual(len(saved['match']['legs']), 8)
            self.assertGreater(len(saved['actual_trace']), 1000)
            self.assert_no_command_sent()

    def test_match_stop_work_phase_and_map_changes_stop_before_next_frame(self):
        for operation in ('stop', 'version', 'replan'):
            runner = self.start_competition()
            if operation == 'stop':
                for i in range(2000):
                    self.w.sim.make_frame(i*.02)
                    self.w._poll_competition()
                    if runner.status == 'ACTION': break
                self.assertEqual(runner.status, 'ACTION')
                self.w.send_line('STOP')
                self.w.sim._service_commands()
            elif operation == 'version': self.w.nav_map['map_version'] += 1
            else: self.w._prepare_plan(2200, 1800)
            before = self.w.sim.hold, self.w.sim.zval
            count = runner.grabs, runner.placements
            self.assertEqual(runner.status, 'CANCELLED')
            self.assertFalse(self.w.sim.navigation_snapshot()['tracking'])
            self.assertEqual(self.w.map_view.reference_item.path().elementCount(), 0)
            for i in range(50):
                self.w.sim.make_frame(i*.02); self.w._poll_competition()
            self.assertEqual((self.w.sim.hold, self.w.sim.zval), before)
            self.assertEqual((runner.grabs, runner.placements), count)

    def test_obstacle_scene_button_preserves_four_cylinders_for_complete_match_and_json(self):
        import competition_simulation as competition
        button, = [b for b in self.w.findChildren(main.QPushButton) if b.text() == '障碍比赛场景']
        button.click()
        self.assertEqual(self.w.sim_obstacles, list(competition.DEMO_OBSTACLES))
        self.assertEqual(len(self.w.map_view._obstacle_items), 4)
        self.assertFalse(self.w.sim.navigation_snapshot()['tracking'])
        runner = self.start_competition()
        self.assertEqual(self.w.sim_obstacles, list(competition.DEMO_OBSTACLES))
        self.assertEqual(len(self.w.map_view._obstacle_items), 4)
        self.assertIn('障碍4', self.w.competition_status.text())
        for i in range(9200):
            self.w.sim.make_frame(i*.02); self.w._poll_competition()
            if not runner.active: break
        self.assertEqual(runner.status, 'COMPLETE', runner.reason)
        export = Path(self.temp.name)/'obstacle-match.json'
        with patch.object(main.QFileDialog, 'getSaveFileName', return_value=(str(export), '')):
            self.w._export_competition()
        saved = main.json.loads(export.read_text(encoding='utf-8'))
        self.assertEqual(saved['match']['sim_obstacles'], [list(p) for p in competition.DEMO_OBSTACLES])
        self.assertEqual(saved['match']['obstacle_radius_mm'], 25)
        self.assertEqual(saved['correct_grabs'], 12)
        self.assert_no_command_sent()

    def test_manual_and_random_obstacles_are_preserved_by_match_start(self):
        self.w._load_competition_obstacles()
        self.w._clear_obstacles()
        self.w.obstacle_mode_check.setChecked(True)
        self.click_layout(1200, 1700)
        self.assertEqual(len(self.w.sim_obstacles), 1)
        self.assertLess(main.math.dist(self.w.sim_obstacles[0], (1200, 1700)), 5)
        with patch.object(core, 'random_obstacle_points', return_value=([(220, 700), (220, 1700), (700, 220)], '')):
            self.w._random_obstacles()
        runner = self.start_competition()
        self.assertEqual(runner.match['sim_obstacles'], self.w.sim_obstacles)
        self.assertEqual(len(runner.scene.circles), len(self.w.nav_map['circles'])+4)
        self.assertEqual(len(self.w.map_view._obstacle_items), 4)
        self.w._clear_path()
        self.assert_no_command_sent()

    def test_obstacle_edits_cancel_preflight_motion_and_work_without_late_restart(self):
        self.w._load_competition_obstacles()
        self.w._start_competition()
        future = self.w._competition_future
        self.w._clear_obstacles()
        self.assertIsNone(self.w._competition_future)
        try: future.result(timeout=5)
        except (ValueError, CancelledError): pass
        self.w._poll_competition()
        self.assertIsNone(self.w.competition)
        for phase in ('motion', 'work'):
            self.w._load_competition_obstacles()
            runner = self.start_competition()
            if phase == 'work':
                for i in range(2000):
                    self.w.sim.make_frame(i*.02); self.w._poll_competition()
                    if runner.status == 'ACTION': break
                self.assertEqual(runner.status, 'ACTION')
            before = self.w.sim.hold, self.w.sim.zval
            counts = runner.grabs, runner.placements
            self.w._clear_obstacles()
            self.assertEqual(runner.status, 'CANCELLED')
            for i in range(30):
                self.w.sim.make_frame(i*.02); self.w._poll_competition()
            self.assertEqual((self.w.sim.hold, self.w.sim.zval), before)
            self.assertEqual((runner.grabs, runner.placements), counts)
        self.assert_no_command_sent()

    def test_direct_obstacle_list_change_is_caught_before_next_core_motion(self):
        self.w._load_competition_obstacles()
        runner = self.start_competition()
        before = self.w.sim.hold, self.w.sim.zval
        self.w.sim_obstacles[0] = (1200, 1600)  # 绕过GUI编辑入口，仍必须由冻结快照门禁保护。
        self.w.sim.make_frame(.02); self.w._poll_competition()
        self.assertEqual(runner.status, 'FAULT')
        self.assertEqual((self.w.sim.hold, self.w.sim.zval), before)
        self.assertFalse(self.w.sim.navigation_snapshot()['tracking'])
        self.assert_no_command_sent()

    def test_obstacle_occupied_station_refuses_start_and_preserves_actual_vehicle_pose(self):
        self.w._load_competition_obstacles()
        self.w.sim_obstacles = [(2180, 1200)]
        self.w._obstacles_changed()
        before = self.w.sim.hold, self.w.sim.zval
        self.w._start_competition()
        deadline = time.monotonic()+12
        while self.w._competition_future is not None and time.monotonic() < deadline:
            self.app.processEvents(); time.sleep(.01)
        self.assertIsNone(self.w._competition_future)
        self.assertIsNone(self.w.competition)
        self.assertIn('停靠点', self.w.competition_status.text())
        self.assertEqual((self.w.sim.hold, self.w.sim.zval), before)
        self.assertFalse(self.w.sim.navigation_snapshot()['tracking'])
        self.assertEqual(len(self.w.map_view._obstacle_items), 1)
        self.assert_no_command_sent()

    def test_match_preflight_cancellation_bad_code_and_hardware_are_rejected(self):
        self.w._start_competition()
        self.assertIsNotNone(self.w._competition_future)
        self.w._clear_path('STOP during match preparation')
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            self.app.processEvents(); time.sleep(.01)
        self.assertIsNone(self.w.competition)
        self.assertFalse(self.w.sim.navigation_snapshot()['tracking'])
        self.w.competition_code.setText('123+111+321+123')
        self.w._start_competition()
        self.assertIn('拒绝', self.w.competition_status.text())
        self.assertIsNone(self.w._competition_future)
        self.w.worker = object()
        try:
            self.w._start_competition()
            self.assertIn('不连接实车', self.w.competition_status.text())
            self.assertIsNone(self.w._competition_future)
        finally:
            self.w.worker = None
        self.assert_no_command_sent()

    def test_map_version_change_cancels_core_and_all_planned_layers_immediately(self):
        self.start_continuous_arc()
        sim = self.w.sim
        sim.make_frame(.02)
        self.w._follow_step()
        self.w.nav_map['map_version'] += 1
        before = sim.hold, sim.zval
        self.assertIsNone(self.w.follow)
        self.assertFalse(sim.navigation_snapshot()['tracking'])
        for item in (self.w.map_view.skeleton_item, self.w.map_view.path_item,
                     self.w.map_view.reference_item, self.w.map_view.trajectory_arrows):
            self.assertEqual(item.path().elementCount(), 0)
        for index in range(10):
            sim.make_frame(index*.02)
        self.assertEqual((sim.hold, sim.zval), before)
        self.assert_no_command_sent()

    def test_trajectory_arrows_follow_unwrapped_cw_and_ccw_tangents_and_clear(self):
        import math
        from arc_smoothing import ArcSegment
        import navigation_planner as nav
        s = nav.CollisionScene([], [], (0, 0, 2400, 2400), 10, (280, 260, 0))
        for sweep in (90, -90):
            center, radius, angle = (1200, 1200), 120, 181
            def xy(deg):
                return (center[0]+radius*math.cos(math.radians(deg)), center[1]+radius*math.sin(math.radians(deg)))
            arc = ArcSegment(0, 0, radius, xy(angle), center, xy(angle+sweep),
                             'CCW' if sweep > 0 else 'CW', angle, sweep, 0, 90, 'FORWARD', ())
            result = core.generate_trajectory([arc], s)
            self.assertTrue(result['trajectory_safe'], result)
            view = self.w.map_view
            view.set_trajectory(result['trajectory'])
            path = view.trajectory_arrows.path()
            selected, last_s = [], -math.inf
            for i, p in enumerate(result['trajectory']):
                if p['s_mm']-last_s >= 40-nav.EPS or i == len(result['trajectory'])-1:
                    selected.append(p)
                    last_s = p['s_mm']
            self.assertEqual(path.elementCount(), 5*len(selected))
            for index, p in enumerate(selected):
                # 画出来的箭头必须沿几何圆弧切线，不能因angle>180而画反。
                theta = math.radians(angle+math.copysign(math.degrees(p['s_mm']/radius), sweep))
                tangent_x = -math.copysign(1, sweep)*math.sin(theta)
                tangent_y = math.copysign(1, sweep)*math.cos(theta)
                a, b = path.elementAt(index*5), path.elementAt(index*5+1)
                self.assertAlmostEqual(b.x-a.x, 36*tangent_x)
                self.assertAlmostEqual(b.y-a.y, -36*tangent_y)
            view.set_path(None)
            self.assertEqual(view.trajectory_arrows.path().elementCount(), 0)
        self.assert_no_command_sent()

    def test_dynamic_snapshot_is_drawn_and_invalidates_old_plan(self):
        self.click_layout(330, 1200)
        self.wait_plan()
        self.assertIsNotNone(self.w.planned_result)
        old_signature = self.w._navigation_signature()
        self.w.set_dynamic_obstacles(rects=[(2000, 1000, 2050, 1050, '动态矩形')],
                                     circles=[(1900, 900, 25, '动态圆')])
        self.assertIsNone(self.w.planned_result)
        self.assertNotEqual(old_signature, self.w._navigation_signature())
        self.assertEqual(self.w.map_view.path_item.path().elementCount(), 0)
        self.assertEqual(len(self.w.map_view._navigation_overlay),
                         2+len(self.w.nav_map['rects'])+len(self.w.nav_map['circles'])+2)
        self.assert_no_command_sent()

    def test_slot_submission_exception_is_shown_not_swallowed(self):
        with patch.object(self.w._planner_pool, 'submit', side_effect=RuntimeError('submit failure')):
            self.click_layout(330, 1200)
        self.assertEqual(self.errors, [])
        self.assertIn('submit failure', self.w.map_status.text())
        self.assertIsNone(self.w._plan_future)
        self.assertTrue(self.w._planner_error_log.exists())
        self.assert_no_command_sent()

    def test_quick_target_button_reaches_planner(self):
        buttons = [b for b in self.w.findChildren(main.QPushButton) if b.text() == '暂存区']
        self.assertEqual(len(buttons), 1)
        self.w.sim.make_frame(0.0)
        QTest.mouseClick(buttons[0], main.Qt.LeftButton)
        self.wait_plan()
        self.assertIsNotNone(self.w.planned_result, self.w.map_status.text())
        self.assertTrue(self.w.planned_result['ok'])
        self.assert_no_command_sent()


if __name__ == '__main__':
    unittest.main(verbosity=2)
