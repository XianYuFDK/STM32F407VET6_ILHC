import argparse
import copy
import json
import os
import queue
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import main
import core
from tests.test_coordinate_navigation import fixture


class UserControlsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ilhc-user-controls-')
        self.addCleanup(self.temp.cleanup)
        env = patch.dict(os.environ, ILHC_RUN_LOG_DIR=self.temp.name)
        env.start()
        self.addCleanup(env.stop)
        self.w = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.addCleanup(self.w.close)

    def test_logging_disabled_no_files_and_toggle_closes_current(self):
        j = self.w.run_journal
        j.start('SIM', {})
        self.w.run_log_check.setChecked(False)
        self.assertIsNone(j.begin_run('SIM_PATH', {}))
        self.assertFalse(j.emit('TX', command='GOTO=1,2,3'))
        self.assertEqual(list(Path(self.temp.name).iterdir()), [])
        self.w.run_log_check.setChecked(True)
        j.begin_run('SIM_PATH', {})
        first = j.path
        j.emit('TEST')
        self.w.run_log_check.setChecked(False)
        self.assertIsNone(j.active_run)
        self.assertIsNone(j.begin_run('DIRECT_COMMAND', {}))
        j.close()
        rows = [json.loads(x) for x in (first/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertTrue(any(r.get('status') == 'RECORDING_DISABLED' for r in rows))
        self.assertEqual(len(list(Path(self.temp.name).iterdir())), 1)

    def test_parameter_readback_hydrates_and_preserves_edit(self):
        row = self.w.chassis_rows['KPX']
        self.assertFalse(row.spin.isEnabled())
        self.assertEqual(row.spin.text(), '')
        self.w._apply_param_readback('KPX', 1.5)
        self.assertEqual(row.spin.value(), 1.5)
        row.spin.setValue(1.7)
        row.set_readback(1.5)
        self.assertEqual(row.spin.value(), 1.7)
        with patch.object(self.w, 'send_line') as send:
            row._send()
            self.assertTrue(any(c.args[0] == 'KPX=1.7' for c in send.call_args_list))
        row.set_readback(1.65)
        self.assertEqual(row.spin.value(), 1.65)
        row.reset_readback()
        self.assertFalse(row.send_button.isEnabled())

    def test_work_points_cm_atomic_validation_and_save(self):
        w = self.w
        self.assertEqual([s.value() for s in w.work_point_spins['rough']], [190, 105])
        w.work_point_spins['rough'][1].setValue(106)
        self.assertTrue(w._apply_work_points(), w.work_point_status.text())
        self.assertEqual(core.layout_to_field(*w.nav_map['competition']['stations']['rough']), (1900, 1060))
        before = copy.deepcopy(dict(w.nav_map))
        w.work_point_spins['rough'][0].setValue(60)
        w.work_point_spins['rough'][1].setValue(160)
        self.assertFalse(w._apply_work_points())
        self.assertEqual(dict(w.nav_map), before)
        w._sync_work_points()
        target = Path(self.temp.name)/'custom-map.json'
        with patch.object(main.QFileDialog, 'getSaveFileName', return_value=(str(target), '')):
            w._save_work_point_map()
        loaded = main.nav.load_map(target)
        self.assertEqual(loaded['competition']['stations'], w.nav_map['competition']['stations'])

    def test_sim_work_wait_pause_freezes_action_timer(self):
        import competition_simulation as competition
        from tests.test_competition_simulation import runner_for, advance
        data = competition.load_profile()
        home = data['competition']['start_zones']['1']
        match = dict(map_snapshot=data, margin_mm=10, home=home, start_yaw=180,
                     task_code='156+123+516+231', stages=[
                         dict(kind='SCAN',label='scan',duration_s=1),
                         dict(kind='MANEUVER',label='finish',target=home,yaw=180)])
        runner = runner_for(match)
        advance(runner, 5)
        runner.sim.set_navigation_paused(True)
        before = (runner.index, runner.elapsed_s, runner.action_elapsed_s)
        advance(runner, 1000)
        self.assertEqual((runner.index, runner.elapsed_s, runner.action_elapsed_s), before)
        self.assertTrue(runner.active, runner.reason)
        runner.sim.set_navigation_paused(False)
        for _ in range(100):
            advance(runner)
            if runner.index == 1:
                break
        self.assertEqual(runner.index, 1, runner.reason)
        self.assertEqual(runner.status, 'RUNNING', runner.reason)

    def test_sim_pause_freezes_and_continues_same_goal(self):
        program, scene = fixture()
        sim = core.Simulator(queue.Queue(), queue.Queue())
        sim.handle_line('ZERO')
        start = program['start']
        sim.hold = (start['x_mm'], start['y_mm'])
        sim.zval = 90-start['field_yaw_deg']
        epoch = sim.begin_navigation()
        sim.submit_navigation_coordinates(epoch, program, (0, 0, 0), scene)
        for _ in range(5):
            sim.make_frame(sim._t + .02)
        sim.set_navigation_paused(True)
        before = sim.navigation_snapshot()
        for _ in range(1000):
            sim.make_frame(sim._t + .02)
        after = sim.navigation_snapshot()
        for field in ('hold', 'yaw', 'progress_s_mm', 'tracking_elapsed_s', 'epoch', 'goal_id'):
            self.assertEqual(before[field], after[field], field)
        self.assertEqual(after['speed_mm_s'], 0)
        sim.set_navigation_paused(False)
        for _ in range(10):
            sim.make_frame(sim._t + .02)
        self.assertNotEqual(sim.navigation_snapshot()['hold'], before['hold'])
        sim.handle_line('STOP')
        with self.assertRaises(ValueError):
            sim.set_navigation_paused(False)


if __name__ == '__main__':
    unittest.main()
