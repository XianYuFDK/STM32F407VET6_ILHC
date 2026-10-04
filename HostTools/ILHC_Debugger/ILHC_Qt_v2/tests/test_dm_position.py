"""验证回转单位、曲线、到位证据、示教持久化及真实 Qt 交互。"""
import argparse
import math
import os
import queue
import struct
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import core
import main
from dm_motion import (Arrival, ReferenceMove, motor_rad, validate_target,
                       save_profile, load_profile)


class MotionTests(unittest.TestCase):
    def test_unit_ratio_and_limits(self):
        self.assertAlmostEqual(motor_rad(90, 2), math.pi)
        for args in ((360, 3), (0, 0), (float("nan"), 1)):
            with self.assertRaises(ValueError):
                motor_rad(*args)
        with self.assertRaises(ValueError):
            validate_target(181, 1, -180, 180)
        with self.assertRaises(ValueError):
            validate_target(0, 1, 10, -10)

    def test_reference_bounds_speed_acceleration_and_reverse(self):
        for start, target in ((0, 90), (90, -90), (10, 10)):
            m = ReferenceMove(start, target, 15, 30)
            dt = m.duration / 10000
            points = [m.position(i*dt) for i in range(10001)]
            velocities = [(b-a)/dt for a,b in zip(points,points[1:])]
            accelerations = [(b-a)/dt for a,b in zip(velocities,velocities[1:])]
            self.assertAlmostEqual(points[0], start)
            self.assertAlmostEqual(points[-1], target)
            self.assertLessEqual(max(map(abs, velocities)), 15.001)
            self.assertLessEqual(max(map(abs, accelerations)), 30.001)
            self.assertTrue(all(min(start,target)-1e-8 <= p <= max(start,target)+1e-8 for p in points))

    def test_arrival_requires_new_continuous_samples(self):
        a = Arrival(0.5, 1, 0.3)
        self.assertFalse(a.update(1, 0, 0, False))
        self.assertFalse(a.update(1.1, 0, 0))
        self.assertFalse(a.update(1.1, 0, 0))
        self.assertFalse(a.update(1.3, 0, 2))
        self.assertFalse(a.update(1.4, 0, 0))
        self.assertFalse(a.update(1.6, 0, 0))
        self.assertTrue(a.update(1.8, 0, 0))
        self.assertFalse(a.update(2.3, 0, 0))  # 中断不能算连续停稳

    def test_latest_reference_preserves_other_commands(self):
        q = core.CommandQueue()
        for cmd in ("DMPOS=1", "S35MOVE=1000,20", "DMVEL=1", "GET XVMIN"):
            q.put(cmd)
        core.put_dm_reference(q, "DMPOS=2")
        self.assertEqual(list(q.queue), ["S35MOVE=1000,20", "DMVEL=1", "GET XVMIN", "DMPOS=2"])
        core.discard_dm_targets(q)
        self.assertEqual(list(q.queue), ["S35MOVE=1000,20", "GET XVMIN"])
        self.assertEqual(q.unfinished_tasks, 2)

    def test_simulator_posvel_zero_speed_and_no_mit_gain_dependency(self):
        sim = core.Simulator(queue.Queue(), queue.Queue())
        for cmd in ("DMMODE=2", "DMKP=0", "DMPOS=1", "DMVEL=0", "DMEN"):
            sim.handle_line(cmd)
        self.assertEqual(sim.make_frame(0)[13], 0)
        sim.handle_line("DMVEL=1")
        self.assertGreater(sim.make_frame(.02)[13], 0)


class PositionQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.w = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.p = self.w.dm_position_panel
        self.p.timer.stop()
        self.p.path = Path(self.temp.name)/"dm_positions.json"
        self.w.sim = core.Simulator(self.w.frame_q, self.w.line_q, self.w.urgent_q,
                                    text_q=self.w.fw_text_q)
        self.w.dm_mode_combo.setCurrentIndex(1)
        self.clock = time.monotonic()
        self.w.latest = self.w.sim.make_frame(0)
        self.w.latest_received_monotonic = self.clock

    def tearDown(self):
        self.w.close()
        self.temp.cleanup()

    def drain(self):
        for q in (self.w.urgent_q, self.w.line_q):
            while not q.empty():
                self.w.sim.handle_line(q.get_nowait())
        while not self.w.fw_text_q.empty():
            self.p.drive.handle_reply(self.w.fw_text_q.get_nowait())

    def read_drive(self, write=False):
        self.p.drive.request(write)
        self.assertIsNotNone(self.p.drive.pending)
        self.p.drive.submit(self.p.drive.pending['seq'])
        self.drain()

    def frame(self, position=None, velocity=None):
        v = list(self.w.sim.make_frame(self.clock))
        if position is not None:
            v[13] = math.radians(position)
        if velocity is not None:
            v[14] = math.radians(velocity)
        self.w.latest = tuple(v)
        self.w.latest_received_monotonic = self.clock

    def ready(self, smooth=True):
        self.p.apply_mode(2)
        self.drain()
        self.clock += .4
        self.frame()
        # 现有曲线/示教专项继续验证PC参考路径；内置梯形另有寄存器专项。
        self.p.smooth.setChecked(smooth)
        self.read_drive()
        self.p.enable()
        self.drain()
        self.frame()

    def test_mode_selection_only_switches_interface(self):
        self.assertEqual(self.w.dm_mode_stack.currentIndex(), 1)
        self.assertTrue(self.w.line_q.empty())
        self.assertTrue(self.w.urgent_q.empty())

    def test_drive_register_write_and_simulated_readback(self):
        d = self.p.drive
        d.acc.setValue(.002)
        d.dec.setValue(.003)
        self.read_drive(True)
        self.assertAlmostEqual(self.w.sim.dm_acc, .002)
        self.assertAlmostEqual(self.w.sim.dm_dec, -.003)
        self.assertTrue(d.actual['simulated'])
        self.assertIn('模拟回读', d.readback.text())
        self.assertFalse(self.w.sim.dm_active)

    def test_drive_stale_response_mismatch_and_timeout(self):
        d = self.p.drive
        d.request(True)
        seq = d.pending['seq']
        self.assertFalse(d.handle_reply('DMREG %d 4 0 40000000 C0000000' % seq))
        self.assertFalse(d.handle_reply('DMREG %d 3 0 40000000 C0000000' % (seq % 65535+1)))
        self.assertIsNotNone(d.pending)
        d.handle_reply('DMREG %d 3 0 40400000 C0000000' % seq)
        self.assertIsNone(d.actual)
        self.assertIn('不一致', d.status.text())
        d.request(False)
        d.pending['deadline'] = 0
        d.tick()
        self.assertIsNone(d.pending)
        self.assertIn('未收到', d.status.text())

    def test_drive_requires_disabled_and_stop_cancels_deferred_send(self):
        d = self.p.drive
        self.w.latest = tuple(list(self.w.latest[:16])+[1]+list(self.w.latest[17:]))
        d.request(False)
        self.assertIsNone(d.pending)
        self.frame()
        d.request(False)
        seq = d.pending['seq']
        self.w.send_line('DMEN')
        self.assertNotIn('DMEN', list(self.w.line_q.queue))
        self.p.stop()
        d.submit(seq)
        self.assertIsNone(d.pending)
        self.assertFalse(any(str(c).startswith('DMREAD=') for c in self.w.line_q.queue))

    def test_internal_trapezoid_requires_readback_and_sends_final_target_once(self):
        with patch('dm_panel.time.monotonic', side_effect=lambda: self.clock):
            self.ready()
            self.p.smooth.setChecked(False)
            self.assertTrue(self.p.move(15))
            targets = [c for c in self.w.line_q.queue if c.startswith('DMPOS=')]
            self.assertEqual(len(targets), 1)
            self.assertAlmostEqual(float(targets[0].partition('=')[2]), math.radians(15), places=6)
            self.drain()
            self.clock += .05
            self.frame()
            self.p.tick()
            self.assertFalse(any(c.startswith('DMPOS=') for c in self.w.line_q.queue))
            self.p.stop()
            self.drain()
            self.p.drive.reset()
            self.p.enable()
            self.drain()
            self.frame()
            self.assertFalse(self.p.move(15))
            self.assertIn('确认回读', self.p.status.text())

    def test_drive_parser_handles_split_repeated_mixed_telemetry(self):
        line = b'DMREG 123 3 0 3B03126F BB449BA6\r\n'
        parser = core.FrameParser()
        parser.feed(line[:15])
        parser.feed(line[15:]+line)
        parser.feed(struct.pack('<24f', *([0.0]*24))+b'\x00\x00\x80\x7f'+line)
        replies = parser.take_text()
        self.assertEqual(len(replies), 3)
        value = core.dm_register_feedback(replies[0])
        self.assertAlmostEqual(value['acc'], .002, places=7)
        self.assertAlmostEqual(value['dec'], -.003, places=7)
        self.assertIsNone(core.dm_register_feedback('DMREG 123 3 0 123 BAD'))

    def test_drive_session_reset_rejects_late_response(self):
        d = self.p.drive
        d.request(False)
        seq = d.pending['seq']
        self.p.session_reset()
        self.assertFalse(d.handle_reply('DMREG %d 3 0 40000000 C0000000' % seq))
        self.assertIsNone(d.actual)
        self.w.dm_mode_combo.setCurrentIndex(0)
        self.assertEqual(self.w.dm_mode_stack.currentIndex(), 0)
        self.assertTrue(self.w.line_q.empty())

    def test_apply_mode_and_enable_keep_current_position(self):
        self.w.sim.fb_pos = math.radians(25)
        self.frame()
        with patch("dm_panel.time.monotonic", side_effect=lambda: self.clock):
            self.ready()
        self.assertEqual(self.w.sim.dm_mode, 2)
        self.assertAlmostEqual(self.w.sim.dm_pos, math.radians(25), places=5)
        self.assertEqual(self.w.sim.dm_vel, 0)
        self.assertFalse(self.p.active)
        self.assertEqual(self.w.sim.fb_status, 1)

    def test_switch_to_mit_requires_explicit_apply_before_enable(self):
        with patch("dm_panel.time.monotonic", side_effect=lambda: self.clock):
            self.ready()
            self.w.dm_mode_combo.setCurrentIndex(0)
            self.p.enable()
            self.assertIn("先应用所选模式", self.p.status.text())
            self.assertTrue(self.w.line_q.empty())

    def test_corrupt_profile_does_not_replace_positions(self):
        self.p.positions = {"pickup": 0, "slot1": 15}
        self.p.save()
        original = self.p.profile()
        original["settings"]["ratio"] = .001
        import json
        self.p.path.write_text(json.dumps(original), encoding="utf-8")
        self.p.load()
        self.assertEqual(self.p.positions, {"pickup": 0, "slot1": 15})
        self.assertEqual(self.p.value("ratio"), 1)
        self.assertIn("无法加载", self.p.status.text())

    def test_no_move_until_mode_enabled_and_limits_valid(self):
        with patch("dm_panel.time.monotonic", side_effect=lambda: self.clock):
            self.assertFalse(self.p.move(5))
            self.ready()
            self.assertFalse(self.p.move(181))
            self.p.inputs["ratio"].setValue(10)
            self.assertFalse(self.p.move(90))
        self.assertIsNone(self.p.active)

    def test_stale_telemetry_aborts_and_clears_reference(self):
        with patch("dm_panel.time.monotonic", side_effect=lambda: self.clock):
            self.ready()
            self.assertTrue(self.p.move(15))
            core.put_dm_reference(self.w.line_q, "DMPOS=0.2")
            self.clock += .4
            self.p.tick()
        self.assertIsNone(self.p.active)
        self.assertEqual(list(self.w.urgent_q.queue), ["DMOFF"])
        self.assertFalse(any(str(c).startswith("DMPOS=") for c in self.w.line_q.queue))

    def test_external_target_stops_flow_without_restarting(self):
        with patch("dm_panel.time.monotonic", side_effect=lambda: self.clock):
            self.ready()
            self.assertTrue(self.p.move(15))
            self.w.send_line("DMPOS=1")
        self.assertIsNone(self.p.active)
        self.assertNotIn("DMPOS=1", list(self.w.line_q.queue))
        self.assertIn("DMOFF", list(self.w.urgent_q.queue))

    def test_persistence_and_zero_invalidation(self):
        self.p.preset_inputs["slot1"].setValue(25)
        self.p.store("slot1")
        data = load_profile(self.p.path)
        self.assertEqual(data["positions"], {"slot1": 25})
        self.p.positions.clear()
        self.p.load()
        self.assertEqual(self.p.positions["slot1"], 25)
        self.p.reference.setChecked(True)
        self.w.send_line("DMZERO")
        self.assertFalse(self.p.reference.isChecked())
        before = self.p.path.read_bytes()
        data["positions"]["slot1"] = float("nan")
        with self.assertRaises(ValueError):
            save_profile(self.p.path, data)
        self.assertEqual(self.p.path.read_bytes(), before)

    def test_flow_requires_arrival_then_manual_release_and_return(self):
        self._run_flow(True)

    def test_internal_trapezoid_preset_flow(self):
        self._run_flow(False)

    def _run_flow(self, smooth):
        with patch("dm_panel.time.monotonic", side_effect=lambda: self.clock):
            self.ready(smooth)
            self.p.positions = {"pickup": 0, "slot1": 5}
            self.p.reference.setChecked(True)
            self.p.start_flow()
            self.assertEqual(self.p.flow, "to_slot")
            self.p.return_flow()
            self.assertEqual(self.p.flow, "to_slot")
            for _ in range(180):
                self.clock += .05
                self.drain()
                self.frame()
                self.p.tick()
                if self.p.flow == "await_release":
                    break
            self.assertEqual(self.p.flow, "await_release")
            self.assertIsNone(self.p.active)
            self.assertTrue(self.p.release.isEnabled())
            for _ in range(8):
                self.clock += .05
                self.frame()
                self.p.tick()
            self.assertEqual(self.p.flow, "await_release")
            self.p.return_flow()
            self.assertEqual(self.p.flow, "to_pickup")
            for _ in range(180):
                self.clock += .05
                self.drain()
                self.frame()
                self.p.tick()
                if self.p.flow == "idle":
                    break
            self.assertEqual(self.p.flow, "idle")
            self.assertAlmostEqual(self.w.latest[13], 0, places=3)

    def test_switching_mode_stops_running_goal(self):
        with patch("dm_panel.time.monotonic", side_effect=lambda: self.clock):
            self.ready()
            self.p.move(15)
            self.w.dm_mode_combo.setCurrentIndex(0)
        self.assertIsNone(self.p.active)
        self.assertIn("DMOFF", list(self.w.urgent_q.queue))


if __name__ == "__main__":
    unittest.main()
