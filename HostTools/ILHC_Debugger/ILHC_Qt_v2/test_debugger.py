"""无硬件回归：坐标、导航入口、步进命令和模拟器停止语义。"""
import argparse
import os
import queue
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import core
import main


class DebuggerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.window = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.forward_invert_default = self.window.manual_invert[0].isChecked()
        # 原协议测试使用未反转基线，实车默认方向另行验证。
        self.window.manual_invert[0].setChecked(False)
        # 使用未启动的模拟器作为离线命令接收端，测试不会打开串口。
        self.window.sim = core.Simulator(queue.Queue(), queue.Queue())

    def tearDown(self):
        self.window.close()

    def test_origin_and_axes(self):
        self.assertEqual(core.layout_to_field(2250, 2250), (0, 0))
        self.assertEqual(core.layout_to_field(2150, 2150), (100, 100))
        self.assertEqual(core.layout_to_field(*core.ZONE_CENTER[2]), (2100, 0))
        self.assertEqual(self.window._ops_to_field(0, 0), (2250, 2250))
        # 原图左100mm是旋转后屏幕向上，对应固件OPS的-X。
        self.assertEqual(self.window._field_to_ops(2150, 2250), (-100, 0))
        # 屏幕向左100mm对应固件OPS的-Y。
        self.assertEqual(self.window._field_to_ops(*core.field_to_layout(100, 0)), (0, -100))

    def test_mapping_roundtrip(self):
        for angle in (0, 37, 90, -180):
            self.window.map_theta = angle
            self.window.map_ox, self.window.map_oy = 125, -300
            x, y = self.window._field_to_ops(*self.window._ops_to_field(-400, 230))
            self.assertAlmostEqual(x, -400)
            self.assertAlmostEqual(y, 230)

    def test_clockwise_view_and_click_inverse(self):
        view = self.window.map_view
        transform = view.transform()
        origin = transform.map(main.QPointF(0, 0))
        right = transform.map(main.QPointF(100, 0))
        down = transform.map(main.QPointF(0, 100))
        self.assertAlmostEqual(right.x(), origin.x())
        self.assertGreater(right.y(), origin.y())
        self.assertLess(down.x(), origin.x())
        # 场景目标经显示变换后再逆变换仍为原位置，缩放不改变命令坐标。
        point = main.QPointF(2150, view.sy(2250))
        screen = view.viewportTransform().map(point)
        inverse, valid = view.viewportTransform().inverted()
        self.assertTrue(valid)
        restored = inverse.map(screen)
        self.assertAlmostEqual(restored.x(), point.x())
        self.assertAlmostEqual(restored.y(), point.y())

    def test_zone_two_does_not_change_field_origin(self):
        self.window.zone_combo.setCurrentIndex(1)
        self.window._set_start_zone()
        self.assertEqual((self.window.map_ox, self.window.map_oy), (2100, 0))
        self.assertEqual(self.window._ops_to_field(0, 0), core.ZONE_CENTER[2])

    def test_navigation_and_home_share_checks(self):
        self.window.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        self.window._goto_field(2150, 2250)
        self.assertEqual(self.window.line_q.get_nowait(), "GOTO=-100,0,0")
        self.window.latest = (-1650.0, -1650.0, 0.0) + (0.0,) * 21
        self.window._goto_home()
        self.assertTrue(self.window.line_q.empty())

    def test_stepper_buttons_units_and_uint32(self):
        pos, speed, direction, steps, rpm = self.window.stepper_widgets[35]
        pos.setValue(100.0)
        speed.setValue(50)
        self.window._send_stepper(35, False)
        self.assertEqual(self.window.line_q.get_nowait(), "S35MOVE=1000,50")
        direction.setCurrentIndex(1)
        steps.setValue(4294967295)
        rpm.setValue(100)
        self.window._send_stepper(35, True)
        self.assertEqual(self.window.line_q.get_nowait(), "S35RAW=1,4294967295,100")
        self.assertEqual(core.stepper_move_command(28, 200, 50), "S28MOVE=2000,50")
        with self.assertRaises(ValueError):
            core.stepper_move_command(28, 200, 1)

    def test_cancel_clears_unsent_commands(self):
        self.window.send_line("S28HOME")
        self.window.send_line("S28CANCEL")
        self.assertTrue(self.window.line_q.empty())
        self.assertEqual(self.window.urgent_q.get_nowait(), "S28CANCEL")

    def test_simulator_stop_and_zero(self):
        sim = self.window.sim
        sim.handle_line("DMEN")
        sim.handle_line("STOP")
        self.assertEqual(sim.dm_active, 0)
        sim.handle_line("GOTO=1000,1000,0")
        sim.handle_line("ZERO")
        self.assertIsNone(sim.goto)
        sim.handle_line("S28MOVE=2000,50")
        self.assertEqual(sim.stepper_commands[28], "S28MOVE=2000,50")
        sim.handle_line("S28CANCEL")
        self.assertIsNone(sim.stepper_commands[28])

    def keyboard_event(self, key, pressed=True, repeat=False):
        from PySide6.QtGui import QKeyEvent
        event = QKeyEvent(main.QEvent.KeyPress if pressed else main.QEvent.KeyRelease,
                          key, main.Qt.NoModifier, "", repeat)
        self.app.sendEvent(self.window.keyboard_pad, event)

    def test_keyboard_combinations_release_and_slow(self):
        w = self.window
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_W)
        self.assertEqual(w.manual_vector, (60, 0, 0))
        self.keyboard_event(main.Qt.Key_A)
        self.assertEqual(w.manual_vector, (42, 42, 0))
        self.keyboard_event(main.Qt.Key_Q)
        self.assertEqual(w.manual_vector, (42, 42, 30))
        self.keyboard_event(main.Qt.Key_Shift)
        self.assertEqual(w.manual_vector, (13, 13, 9))
        self.keyboard_event(main.Qt.Key_A, False)
        self.assertEqual(w.manual_vector, (18, 0, 9))
        self.keyboard_event(main.Qt.Key_W, False)
        self.keyboard_event(main.Qt.Key_Q, False)
        self.assertIsNone(w.manual_vector)
        self.assertTrue(w.keyboard_enabled)
        self.assertFalse(w.manual_timer.isActive())
        self.assertIn("MANUAL=0,0,0", list(w.urgent_q.queue))

    def test_vehicle_forward_default_is_reversed(self):
        w = self.window
        self.assertTrue(self.forward_invert_default)
        self.assertFalse(w.manual_invert[1].isChecked())
        self.assertFalse(w.manual_invert[2].isChecked())
        w.manual_invert[0].setChecked(True)
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_W)
        self.assertEqual(w.manual_vector, (-60, 0, 0))
        self.keyboard_event(main.Qt.Key_W, False)
        self.keyboard_event(main.Qt.Key_S)
        self.assertEqual(w.manual_vector, (60, 0, 0))
        self.keyboard_event(main.Qt.Key_A)
        self.keyboard_event(main.Qt.Key_Q)
        self.assertEqual(w.manual_vector, (42, 42, 30))

    def test_keyboard_opposites_space_repeat_and_escape(self):
        w = self.window
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_W)
        self.keyboard_event(main.Qt.Key_W, False, repeat=True)
        self.assertEqual(w.manual_vector, (60, 0, 0))
        self.keyboard_event(main.Qt.Key_S)
        self.assertIsNone(w.manual_vector)
        self.keyboard_event(main.Qt.Key_S, False)
        self.assertEqual(w.manual_vector, (60, 0, 0))
        self.keyboard_event(main.Qt.Key_Space)
        self.keyboard_event(main.Qt.Key_W, repeat=True)
        self.assertIsNone(w.manual_vector)
        self.keyboard_event(main.Qt.Key_Escape)
        self.assertFalse(w.keyboard_enabled)
        self.keyboard_event(main.Qt.Key_D)
        self.assertIsNone(w.manual_vector)

    def test_keyboard_focus_loss_page_and_stop_disarm(self):
        w = self.window
        for action in (lambda: self.app.sendEvent(w.keyboard_pad, main.QEvent(main.QEvent.FocusOut)),
                       lambda: w._select_page(0), lambda: w.send_line("STOP")):
            w._keyboard_toggle()
            self.keyboard_event(main.Qt.Key_W)
            action()
            self.assertFalse(w.keyboard_enabled)
            self.assertFalse(w.keyboard_keys)
            self.assertIsNone(w.manual_vector)
            self.assertTrue(w.line_q.empty())
            self.keyboard_event(main.Qt.Key_W, repeat=True)
            self.assertIsNone(w.manual_vector)

    def test_keyboard_disabled_and_other_widgets_do_not_drive(self):
        from PySide6.QtGui import QKeyEvent
        w = self.window
        self.keyboard_event(main.Qt.Key_W)
        self.assertIsNone(w.manual_vector)
        w._keyboard_toggle()
        self.app.sendEvent(w.command_entry, QKeyEvent(main.QEvent.KeyPress, main.Qt.Key_W,
                                                     main.Qt.NoModifier, "w"))
        self.assertIsNone(w.manual_vector)
        self.assertEqual(w.command_entry.text(), "w")

    def test_keyboard_exit_button_and_real_focus_change(self):
        from PySide6.QtTest import QTest
        w = self.window
        w.show()
        w._select_page(3)
        w.activateWindow()
        self.app.processEvents()
        QTest.mouseClick(w.keyboard_button, main.Qt.LeftButton)
        self.assertTrue(w.keyboard_enabled)
        QTest.keyPress(w.keyboard_pad, main.Qt.Key_W)
        self.assertIsNotNone(w.manual_vector)
        QTest.mouseClick(w.keyboard_exit, main.Qt.LeftButton)
        self.assertFalse(w.keyboard_enabled)
        self.assertIsNone(w.manual_vector)
        QTest.mouseClick(w.keyboard_button, main.Qt.LeftButton)
        QTest.keyPress(w.keyboard_pad, main.Qt.Key_W)
        w.manual_speed.setFocus()
        self.app.processEvents()
        self.assertFalse(w.keyboard_enabled)
        self.assertIsNone(w.manual_vector)

    def test_manual_hold_release_and_page_exit(self):
        w = self.window
        w._manual_start((1, 0, 1))
        self.assertEqual(w.line_q.get_nowait(), "MANUAL=60,0,30")
        w._manual_tick()
        w._manual_stop()
        self.assertIsNone(w.manual_vector)
        self.assertFalse(w.manual_timer.isActive())
        self.assertTrue(w.line_q.empty())

        self.assertEqual(w.urgent_q.get_nowait(), "STOP")
        w._manual_tick()
        self.assertTrue(w.line_q.empty())
        w._manual_start((-1, 0, -1))
        w._select_page(0)
        self.assertIsNone(w.manual_vector)
        self.assertTrue(w.line_q.empty())

    def test_continuous_latest_velocity_has_priority_without_backlog(self):
        w = self.window
        w.line_q.put("KPX=2")
        w.line_q.put("KPY=3")
        w._manual_start((1, 0, 0))
        for _ in range(20):
            w._manual_tick()
        w._manual_start((0, 1, 0))
        self.assertEqual(w.manual_timer.interval(), 50)
        self.assertEqual(w.line_q.qsize(), 3)
        self.assertEqual(w.line_q.get_nowait(), "MANUAL=0,60,0")
        self.assertEqual(w.line_q.get_nowait(), "KPX=2")
        self.assertEqual(w.line_q.get_nowait(), "KPY=3")

    def test_serial_idle_read_is_short_and_stop_precedes_velocity(self):
        q = core.CommandQueue()
        urgent = queue.Queue()
        q.put("MANUAL=60,0,0")
        urgent.put("STOP")
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), q, urgent)
        writes, reads = [], []

        class Port:
            is_open = True
            in_waiting = 0
            def read(self, size):
                reads.append(size)
                return b""
            def write(self, data):
                writes.append(data)
                if data.startswith(b"MANUAL="):
                    worker.stop_flag = True
            def close(self):
                self.is_open = False

        with patch.object(core.serial, "Serial", return_value=Port()) as serial_open:
            worker.run()
        self.assertEqual(serial_open.call_args.kwargs["timeout"], 0.01)
        self.assertEqual(reads, [1])
        self.assertEqual(writes, [b"VOFA\n", b"STOP\n", b"MANUAL=60,0,0\n"])

    def test_manual_stop_zero_goto_cancel_renewal(self):
        for cmd in ("STOP", "ZERO", "GOTO=100,100,0"):
            self.window._clear_command_queues()
            self.window._manual_start((1, 0, 0))
            self.window.send_line(cmd)
            self.assertIsNone(self.window.manual_vector)
            self.assertFalse(self.window.manual_timer.isActive())

    def test_manual_simulation_timeout_and_invalid(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("MANUAL=60,0,30")
        sim.make_frame(0)
        self.assertGreater(sim.hold[0], 0)
        self.assertGreater(sim.zval, 0)
        for bad in ("MANUAL=301,0,0", "MANUAL=60,0", "MANUAL=60,0,0,1", "MANUAL=nan,0,0", "MANUAL=1.2,0,0"):
            sim.handle_line(bad)
            self.assertEqual(sim.manual, (60, 0, 30))
        sim.manual_tick -= 1
        sim.handle_line("PING")
        hold = sim.hold
        sim.make_frame(0.02)
        self.assertIsNone(sim.manual)
        self.assertEqual(sim.hold, hold)
        sim.handle_line("MANUAL=60,0,0")
        sim.handle_line("STOP")
        self.assertIsNone(sim.manual)

    def test_offset_apply_stops_manual_and_sends_pair(self):
        w = self.window
        self.assertEqual(w.ops_offset_x.value(), -50)
        self.assertEqual(w.ops_offset_y.value(), 60)
        w._manual_start((0, 0, 1))
        w.ops_offset_x.setValue(-52.5)
        w._apply_ops_offset()
        self.assertIsNone(w.manual_vector)
        self.assertEqual(w.line_q.get_nowait(), "OPSOFFSET=-52.5,60.0")
        self.assertEqual(w.urgent_q.get_nowait(), "STOP")

    def test_offset_file_roundtrip_and_invalid(self):
        w = self.window
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "offset.json")
            w.ops_offset_x.setValue(-48.5)
            with patch.object(main.QFileDialog, "getSaveFileName", return_value=(path, "")):
                w._save_ops_offset()
            w._reset_ops_offset()
            with patch.object(main.QFileDialog, "getOpenFileName", return_value=(path, "")):
                w._load_ops_offset()
            self.assertEqual(w.ops_offset_x.value(), -48.5)
            self.assertTrue(w.line_q.empty())
            Path(path).write_text('{"version":1,"x_mm":999,"y_mm":60}', encoding="utf-8")
            with patch.object(main.QFileDialog, "getOpenFileName", return_value=(path, "")):
                w._load_ops_offset()
            self.assertEqual(w.ops_offset_x.value(), -48.5)
            self.assertIn("加载失败", w.ops_offset_status.text())

    def test_offset_simulator_residual_and_validation(self):
        sim = self.window.sim
        sim.handle_line("MANUAL=0,0,30")
        sim.handle_line("OPSOFFSET=0,0")
        self.assertIsNone(sim.manual)
        self.assertIsNone(sim.goto)
        sim.zval = 90
        with patch.object(core.random, "gauss", return_value=0):
            uncompensated = sim.make_frame(0)
        self.assertAlmostEqual((uncompensated[0]**2 + uncompensated[1]**2)**0.5, 110.4536, places=3)
        sim.handle_line("OPSOFFSET=-50,60")
        for bad in ("OPSOFFSET=nan,60", "OPSOFFSET=-501,60", "OPSOFFSET=-50,60,0", "OPSOFFSET=-50,", "OPSOFFSET=1e2,60"):
            sim.handle_line(bad)
            self.assertEqual(sim.ops_offset, (-50,60))
        sim.zval = 180
        with patch.object(core.random, "gauss", return_value=0):
            compensated = sim.make_frame(0.02)
        self.assertAlmostEqual(compensated[0], 0)
        self.assertAlmostEqual(compensated[1], 0)

    def test_wheel_lock_commands_track_state_and_stop_manual(self):
        w = self.window
        self.assertIsNone(w.wheel_state)
        w._manual_start((1, 0, 0))
        w.send_line("WHEELOFF")
        # 切换锁轴前先停止键盘续发，命令本身走普通队列。
        self.assertIsNone(w.manual_vector)
        self.assertFalse(w.manual_timer.isActive())
        self.assertIs(w.wheel_state, False)
        self.assertEqual(w.line_q.get_nowait(), "WHEELOFF")
        self.assertIn("失能", w.wheel_status.text())
        # STOP 只停车，不改变锁轴状态。
        w.send_line("STOP")
        self.assertIs(w.wheel_state, False)
        w.send_line("WHEELEN")
        self.assertIs(w.wheel_state, True)
        self.assertEqual(w.line_q.get_nowait(), "WHEELEN")
        self.assertIn("使能", w.wheel_status.text())

    def test_wheel_disabled_blocks_goto_keyboard_and_manual(self):
        w = self.window
        w.send_line("WHEELOFF")
        w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        w._goto_field(2150, 2250)
        # 队列里只有失能命令本身，GOTO 未入队。
        self.assertEqual(w.line_q.get_nowait(), "WHEELOFF")
        self.assertTrue(w.line_q.empty())
        self.assertIn("失能", w.map_status.text())
        w._keyboard_toggle()
        self.assertFalse(w.keyboard_enabled)
        w._manual_start((1, 0, 0))
        self.assertIsNone(w.manual_vector)
        # 重新使能后 GOTO 与手动恢复。
        w.send_line("WHEELEN")
        w._goto_field(2150, 2250)
        self.assertEqual(w.line_q.get_nowait(), "WHEELEN")
        self.assertEqual(w.line_q.get_nowait(), "GOTO=-100,0,0")

    def test_simulator_wheel_gate_and_freeze(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("WHEELOFF")
        self.assertFalse(sim.wheel_enabled)
        sim.handle_line("MANUAL=60,0,30")
        sim.handle_line("GOTO=100,100,0")
        self.assertIsNone(sim.manual)
        self.assertIsNone(sim.goto)
        hold = sim.hold
        sim.make_frame(0.02)
        self.assertEqual(sim.hold, hold)          # 失能后位置冻结
        sim.handle_line("WHEELEN")
        sim.handle_line("MANUAL=60,0,30")
        self.assertEqual(sim.manual, (60, 0, 30))

    def test_console_command_updates_wheel_state(self):
        w = self.window
        w.command_entry.setText("wheeloff")
        w._send_console()
        self.assertIs(w.wheel_state, False)
        self.assertEqual(w.line_q.get_nowait(), "wheeloff")
        w.command_entry.setText("WHEELEN")
        w._send_console()
        self.assertIs(w.wheel_state, True)


if __name__ == "__main__":
    unittest.main()
