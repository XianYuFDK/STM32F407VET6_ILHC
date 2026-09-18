"""无硬件回归：坐标、导航入口、步进命令和模拟器停止语义。"""
import argparse
import os
import queue
import struct
import time
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
        self.forward_invert_default = self.window.manual_invert[1].isChecked()
        # 原协议测试使用未反转基线，实车默认方向另行验证。
        self.window.manual_invert[1].setChecked(False)
        # 使用未启动的模拟器作为离线命令接收端，测试不会打开串口。
        self.window.sim = core.Simulator(queue.Queue(), queue.Queue())

    def tearDown(self):
        self.window.close()

    def test_origin_and_axes(self):
        self.assertEqual(core.layout_to_field(2250, 2250), (0, 0))
        self.assertEqual(core.layout_to_field(2150, 2150), (100, 100))
        self.assertEqual(core.layout_to_field(*core.ZONE_CENTER[2]), (2100, 0))
        self.assertEqual(self.window._ops_to_field(0, 0), (2250, 2250))
        # 内部测试输入为mm：场地向上100mm = 内部 +100mm；协议发送时对应 GOTO Y=10.0cm。
        self.assertEqual(self.window._field_to_ops(2150, 2250), (0, 100))
        # 内部测试输入为mm：场地向左100mm = 内部 +100mm；协议发送时对应 GOTO X=10.0cm。
        self.assertEqual(self.window._field_to_ops(*core.field_to_layout(100, 0)), (100, 0))

    def test_telemetry_direction_matches_field_axes(self):
        """验收桩：向前=+Y、向左=+X；地图上前进→屏幕上、左移→屏幕左，且点击方向一致。

        屏幕方向来自 test_clockwise_view_and_click_inverse 已断言的 rotate(90)：
        场景 +x 向下、+y 向左，而 layout_x=场景x、layout_y 与场景y 同向。
        """
        w = self.window
        origin = w._ops_to_field(0.0, 0.0)
        self.assertEqual(origin, (2250.0, 2250.0))
        # 遥测 ch0=X=左右、ch1=Y=前后（cm）：前进10.0cm → 场地Y增大 → screen up = layout_x 减小
        forward = w._ops_to_field(0.0, 100.0)
        self.assertEqual(forward, (origin[0] - 100.0, origin[1]))
        # 左移10.0cm → 场地X增大 → screen left = layout_y 减小
        left = w._ops_to_field(100.0, 0.0)
        self.assertEqual(left, (origin[0], origin[1] - 100.0))
        # 点击机器人正前方10.0cm 必须下发 +Y（GOTO 的 Y=前后）
        self.assertEqual(w._field_to_ops(*forward), (0.0, 100.0))
        # 点击机器人正左方10.0cm 必须下发 +X（GOTO 的 X=左右）
        self.assertEqual(w._field_to_ops(*left), (100.0, 0.0))
        # 标定角下往返自洽，且前进方向仍是+Y
        for angle in (0, 37, -90, 180):
            w.map_theta = angle
            bx, by = w._field_to_ops(*w._ops_to_field(120.0, -80.0))
            self.assertAlmostEqual(bx, 120.0)
            self.assertAlmostEqual(by, -80.0)
            gx, gy = w._field_to_ops(*w._ops_to_field(0.0, 100.0))
            self.assertAlmostEqual(gx, 0.0)
            self.assertAlmostEqual(gy, 100.0)

    def test_icon_and_trail_share_one_mapping(self):
        """图标与轨迹必须来自同一变换。

        _update_map_trail 是 _ops_to_field 的向量化副本（历史重复实现），
        坐标改动最容易漏掉这一处；此处直接比较两者的落点是否重合。
        """
        w = self.window
        for angle in (0, 37, -90):
            w.map_theta = angle
            w.map_ox, w.map_oy = 125.0, -300.0
            frame = (140.0, -260.0, 15.0) + (0.0,) * 21
            w.latest = frame
            w.traj_ring.clear()
            w.traj_ring.append(0.0, (frame[0] * core.OPS_CM_TO_MM,
                                     frame[1] * core.OPS_CM_TO_MM))
            w._render_ui()
            w._update_map_trail()
            fx, fy = w._ops_to_field(frame[0] * core.OPS_CM_TO_MM,
                                     frame[1] * core.OPS_CM_TO_MM)
            icon = w.map_view.car_item.rect().center()
            self.assertAlmostEqual(icon.x(), fx, places=6)
            self.assertAlmostEqual(icon.y(), w.map_view.sy(fy), places=6)
            path = w.map_view.trail_item.path()
            self.assertEqual(path.elementCount(), 1)
            element = path.elementAt(0)
            self.assertAlmostEqual(element.x, fx, places=6)
            self.assertAlmostEqual(element.y, w.map_view.sy(fy), places=6)

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
        self.assertEqual(self.window.line_q.get_nowait(), "GOTO=0.0,10.0,0.0")
        # 新约定下同一物理位置(左1650/前1650)的遥测为正值，映射到与旧用例相同的layout(600,600)，
        # 因此到启停区1的直线仍穿越中央物料区。
        self.window.latest = (165.0, 165.0, 0.0) + (0.0,) * 21
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
        sim.handle_line("GOTO=100.0,100.0,0.0")
        sim.handle_line("ZERO")
        self.assertIsNone(sim.goto)
        sim.handle_line("S28MOVE=2000,50")
        self.assertEqual(sim.stepper_commands[28], "S28MOVE=2000,50")
        sim.handle_line("S28CANCEL")
        self.assertIsNone(sim.stepper_commands[28])

    def test_simulator_zero_resets_heading_reference(self):
        sim = self.window.sim
        sim.zval = 15.0
        sim.handle_line("ZERO")
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0.0)
        self.assertAlmostEqual(frame[2], 0.0)
        sim.zval = 40.0
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0.02)
        self.assertAlmostEqual(frame[2], 25.0)

    def test_simulator_goto_cm_and_telemetry_cm(self):
        sim = self.window.sim
        sim.handle_line("GOTO=10.0,20.0,0.0")
        self.assertEqual(sim.goto, (100.0, 200.0, 0.0))
        sim.hold = (100.0, 200.0)           # 统一(X=左, Y=前) mm，正好位于目标
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0.02)
        self.assertAlmostEqual(frame[0], 10.0)   # ch0=X(左右) cm
        self.assertAlmostEqual(frame[1], 20.0)   # ch1=Y(前后) cm

    def keyboard_event(self, key, pressed=True, repeat=False):
        from PySide6.QtGui import QKeyEvent
        event = QKeyEvent(main.QEvent.KeyPress if pressed else main.QEvent.KeyRelease,
                          key, main.Qt.NoModifier, "", repeat)
        self.app.sendEvent(self.window.keyboard_pad, event)

    def test_keyboard_combinations_release_and_slow(self):
        w = self.window
        w._keyboard_toggle()
        # manual_vector 与 MANUAL 同为(X=左右, Y=前后, Z=旋转)。
        self.keyboard_event(main.Qt.Key_W)
        self.assertEqual(w.manual_vector, (0, 60, 0))
        self.keyboard_event(main.Qt.Key_A)
        self.assertEqual(w.manual_vector, (42, 42, 0))
        self.keyboard_event(main.Qt.Key_Q)
        self.assertEqual(w.manual_vector, (42, 42, 30))
        self.keyboard_event(main.Qt.Key_Shift)
        self.assertEqual(w.manual_vector, (13, 13, 9))
        self.keyboard_event(main.Qt.Key_A, False)
        self.assertEqual(w.manual_vector, (0, 18, 9))
        self.keyboard_event(main.Qt.Key_W, False)
        self.keyboard_event(main.Qt.Key_Q, False)
        self.assertIsNone(w.manual_vector)
        self.assertTrue(w.keyboard_enabled)
        self.assertFalse(w.manual_timer.isActive())
        self.assertIn("MANUAL=0,0,0", list(w.urgent_q.queue))

    def test_manual_direction_defaults_not_inverted(self):
        """车头方向反了已在固件协议边界统一修正，三个反向开关默认全部关闭。"""
        w = self.window
        self.assertFalse(self.forward_invert_default)
        self.assertFalse(w.manual_invert[0].isChecked())   # 0=左右反向
        self.assertFalse(w.manual_invert[1].isChecked())   # 1=前后反向
        self.assertFalse(w.manual_invert[2].isChecked())   # 2=旋转反向
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_W)
        self.assertEqual(w.manual_vector, (0, 60, 0))      # W = +Y = 车头
        self.keyboard_event(main.Qt.Key_W, False)
        self.keyboard_event(main.Qt.Key_S)
        self.assertEqual(w.manual_vector, (0, -60, 0))     # S = -Y = 车尾
        self.keyboard_event(main.Qt.Key_S, False)
        self.keyboard_event(main.Qt.Key_A)
        self.assertEqual(w.manual_vector, (60, 0, 0))      # A = +X = 车左
        self.keyboard_event(main.Qt.Key_A, False)
        # 兜底开关仍在：切换复选框会先停车并退出键盘遥控，需重新接管
        w.manual_invert[0].setChecked(True)                # 勾"左右反向"
        self.assertIsNone(w.manual_vector)
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_A)
        self.assertEqual(w.manual_vector, (-60, 0, 0))

    def test_keyboard_opposites_space_repeat_and_escape(self):
        w = self.window
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_W)
        self.keyboard_event(main.Qt.Key_W, False, repeat=True)
        self.assertEqual(w.manual_vector, (0, 60, 0))
        self.keyboard_event(main.Qt.Key_S)
        self.assertIsNone(w.manual_vector)
        self.keyboard_event(main.Qt.Key_S, False)
        self.assertEqual(w.manual_vector, (0, 60, 0))
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
        # _manual_start 的入参与 MANUAL 同序：X=左右、Y=前后、Z=旋转。
        w._manual_start((0, 1, 1))
        self.assertEqual(w.line_q.get_nowait(), "MANUAL=0,60,30")
        w._manual_tick()
        w._manual_stop()
        self.assertIsNone(w.manual_vector)
        self.assertFalse(w.manual_timer.isActive())
        self.assertTrue(w.line_q.empty())

        self.assertEqual(w.urgent_q.get_nowait(), "STOP")
        w._manual_tick()
        self.assertTrue(w.line_q.empty())
        w._manual_start((0, -1, -1))
        w._select_page(0)
        self.assertIsNone(w.manual_vector)
        self.assertTrue(w.line_q.empty())

    def test_continuous_latest_velocity_has_priority_without_backlog(self):
        w = self.window
        w.line_q.put("KPX=2")
        w.line_q.put("KPY=3")
        w._manual_start((0, 1, 0))          # 前进
        for _ in range(20):
            w._manual_tick()
        w._manual_start((1, 0, 0))          # 左移
        self.assertEqual(w.manual_timer.interval(), 50)
        self.assertEqual(w.line_q.qsize(), 3)
        self.assertEqual(w.line_q.get_nowait(), "MANUAL=60,0,0")
        self.assertEqual(w.line_q.get_nowait(), "KPX=2")
        self.assertEqual(w.line_q.get_nowait(), "KPY=3")

    def test_firmware_text_is_surfaced_not_swallowed(self):
        """固件切到文字模式时的 ASCII 报错必须能被界面看到。

        实车证据：CAN 启动失败时固件只发一行
        "ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE" 并停止 24 通道遥测；
        旧实现把非 JustFloat 字节全部静默丢弃，用户完全看不到原因。
        """
        parser = core.FrameParser()
        frame = struct.pack("<24f", *range(24)) + core.FRAME_TAIL
        self.assertEqual(len(parser.feed(frame)), 1)
        self.assertEqual(parser.take_text(), [])            # 纯遥测不产生文本
        self.assertEqual(len(parser.feed(frame)), 1)
        err = b"ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE\r\n"
        self.assertEqual(len(parser.feed(err)), 0)
        self.assertIn("ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE",
                      parser.take_text())
        self.assertEqual(parser.take_text(), [])            # 取走后清空
        # 同一行重复出现只报一次
        parser.feed(err)
        self.assertEqual(parser.take_text(), [])

    def test_firmware_text_and_vofa_notice_reach_log(self):
        w = self.window
        w.fw_text_q.put("固件文本: ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE")
        w.fw_text_q.put("上位机: 1.5s 未收到遥测，已补发 VOFA 恢复波形（第 1/6 次）")
        w._process_frames()
        text = w.console.toPlainText()
        self.assertIn("ERR CAN START FAILED", text)
        self.assertIn("已补发 VOFA", text)
        self.assertTrue(w.fw_text_q.empty())

    def test_serial_worker_resends_vofa_when_silent(self):
        """板子在上位机已连接时复位后，必须能自动补发 VOFA 恢复遥测。"""
        writes = []

        class Port:
            is_open = True
            in_waiting = 0

            def read(self, size):
                return b""

            def write(self, data):
                writes.append(data)

            def close(self):
                self.is_open = False

        text_q = queue.Queue()
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), core.CommandQueue(),
                                   queue.Queue(), text_q=text_q)
        with patch.object(core.serial, "Serial", return_value=Port()), \
             patch.object(core, "VOFA_RETRY_GAP_S", 0.05), \
             patch.object(core, "VOFA_MAX_RETRIES", 3):
            worker.start()
            deadline = time.monotonic() + 4.0
            while len(writes) < 4 and time.monotonic() < deadline:
                time.sleep(0.02)
            worker.stop_flag = True
            worker.join(timeout=2.0)
        self.assertEqual(writes[:1], [b"VOFA\n"])                    # 连接时先发一次
        self.assertGreaterEqual(writes.count(b"VOFA\n"), 2)          # 静默后补发
        self.assertLessEqual(writes.count(b"VOFA\n"), 4)             # 达上限后停止
        notes = []
        while not text_q.empty():
            notes.append(text_q.get_nowait())
        self.assertTrue(any("补发 VOFA" in n for n in notes))

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
        for cmd in ("STOP", "ZERO", "GOTO=10.0,10.0,0.0"):
            self.window._clear_command_queues()
            self.window._manual_start((1, 0, 0))
            self.window.send_line(cmd)
            self.assertIsNone(self.window.manual_vector)
            self.assertFalse(self.window.manual_timer.isActive())

    def test_manual_simulation_timeout_and_invalid(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("MANUAL=0,60,30")          # Y=车头=+60、Z=逆时针
        sim.make_frame(0)
        self.assertGreater(sim.hold[1], 0)         # +Y 指向车头
        self.assertAlmostEqual(sim.hold[0], 0.0)
        self.assertGreater(sim.zval, 0)
        for bad in ("MANUAL=301,0,0", "MANUAL=60,0", "MANUAL=60,0,0,1", "MANUAL=nan,0,0", "MANUAL=1.2,0,0"):
            sim.handle_line(bad)
            self.assertEqual(sim.manual, (0, 60, 30))
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
        self.assertEqual(w.ops_offset_x.value(), 60)
        self.assertEqual(w.ops_offset_y.value(), -50)
        w._manual_start((0, 0, 1))
        w.ops_offset_y.setValue(-52.5)
        w._apply_ops_offset()
        self.assertIsNone(w.manual_vector)
        self.assertEqual(w.line_q.get_nowait(), "OPSOFFSET=60.0,-52.5")
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

    def test_offset_file_v1_migrates_axis_order_once(self):
        w = self.window
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offset-v1.json"
            # v1: x_mm=前后、y_mm=左右。迁移后 X 左右=60、Y 前后=-50。
            path.write_text('{"version":1,"x_mm":-50,"y_mm":60}', encoding="utf-8")
            with patch.object(main.QFileDialog, "getOpenFileName", return_value=(str(path), "")):
                w._load_ops_offset()
        self.assertEqual(w.ops_offset_x.value(), 60)
        self.assertEqual(w.ops_offset_y.value(), -50)
        self.assertIn("迁移", w.ops_offset_status.text())

    def test_offset_simulator_residual_and_validation(self):
        sim = self.window.sim
        sim.handle_line("MANUAL=0,0,30")
        sim.handle_line("OPSOFFSET=0,0")
        self.assertIsNone(sim.manual)
        self.assertIsNone(sim.goto)
        sim.zval = 90
        with patch.object(core.random, "gauss", return_value=0):
            uncompensated = sim.make_frame(0)
        # 90° 旋转时，未配置偏移留下与安装半径和参考航向相关的残差。
        self.assertAlmostEqual((uncompensated[0]**2 + uncompensated[1]**2)**0.5, 11.04536, places=3)
        sim.handle_line("OPSOFFSET=60,-50")        # 统一坐标 X=左60、Y=前-50
        for bad in ("OPSOFFSET=nan,60", "OPSOFFSET=-501,60", "OPSOFFSET=-50,60,0", "OPSOFFSET=-50,", "OPSOFFSET=1e2,60"):
            sim.handle_line(bad)
            self.assertEqual(sim.ops_offset, (60, -50))
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
        self.assertEqual(w.line_q.get_nowait(), "GOTO=0.0,10.0,0.0")

    def test_simulator_wheel_gate_and_freeze(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("WHEELOFF")
        self.assertFalse(sim.wheel_enabled)
        sim.handle_line("MANUAL=60,0,30")
        sim.handle_line("GOTO=10.0,10.0,0.0")
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

    def test_page_header_button_detaches_page(self):
        from PySide6.QtTest import QTest
        w = self.window
        w.show()
        page = w.pages[6]
        buttons = [b for b in page.findChildren(main.QPushButton) if b.text() == "独立窗口"]
        self.assertTrue(buttons)
        QTest.mouseClick(buttons[0], main.Qt.LeftButton)
        self.assertIn(6, w.detached)
        self.assertIs(w.detached[6].page(), page)
        self.assertIs(w.pages[6].window(), w.detached[6])
        self.assertIs(w.slots[6].currentWidget(), w.placeholders[6])
        # 换父窗口后控件不能被 Qt 因 setParent 而留在隐藏状态。
        self.assertFalse(page.isHidden())
        self.app.processEvents()
        self.assertTrue(page.isVisible())
        w._reattach_page(6)
        self.app.processEvents()
        self.assertFalse(page.isHidden())
        self.assertIs(w.slots[6].currentWidget(), page)
        w._select_page(6)          # 主界面回到该页后才应该可见
        self.app.processEvents()
        self.assertTrue(page.isVisible())

    def test_page_detach_keeps_stack_index_and_returns_page(self):
        w = self.window
        self.assertEqual(w.stack.count(), len(w.pages))
        w._select_page(2)
        w._detach_page(3)
        # 拆窗口不能改变主界面下标：导航、_render_ui 都按固定下标工作。
        self.assertEqual(w.stack.count(), len(w.pages))
        self.assertEqual(w.stack.currentIndex(), 2)
        win = w.detached[3]
        self.assertIs(win.page(), w.pages[3])
        self.assertIs(w.pages[3].window(), win)
        self.assertIs(w.slots[3].currentWidget(), w.placeholders[3])
        self.assertTrue(w.nav_buttons[3].property("detached"))
        # 导航按钮指向已拆出的页面时，主界面显示占位卡而不是空白。
        w._select_page(3)
        self.assertEqual(w.stack.currentIndex(), 3)
        self.assertIs(w.slots[3].currentWidget(), w.placeholders[3])
        w._reattach_page(3)
        self.assertNotIn(3, w.detached)
        self.assertIs(w.slots[3].currentWidget(), w.pages[3])
        self.assertIs(w.pages[3].parent(), w.slots[3])
        self.assertFalse(w.nav_buttons[3].property("detached"))

    def test_detached_window_topmost_and_close_returns_page(self):
        w = self.window
        w._detach_page(4)
        win = w.detached[4]
        self.assertFalse(win.is_topmost())
        win.top_btn.setChecked(True)       # 等价于点击独立窗口里的“置顶”
        self.assertTrue(win.is_topmost())
        self.assertTrue(bool(win.windowFlags() & main.Qt.WindowStaysOnTopHint))
        self.assertEqual(win.top_btn.text(), "已置顶")
        self.assertTrue(w.placeholder_top_checks[4].isChecked())
        win.top_btn.setChecked(False)
        self.assertFalse(win.is_topmost())
        self.assertEqual(win.top_btn.text(), "置顶")
        self.assertFalse(w.placeholder_top_checks[4].isChecked())
        win.close()                        # 关闭独立窗口 = 页面回到主窗口
        self.assertNotIn(4, w.detached)
        self.assertIs(w.slots[4].currentWidget(), w.pages[4])

    def test_placeholder_checkbox_and_menu_action_set_topmost_pref(self):
        w = self.window
        w._set_page_topmost(5, True)
        self.assertTrue(w.topmost_pref[5])
        w._detach_page(5)
        # 分离时沿用预置的置顶偏好。
        self.assertTrue(w.detached[5].is_topmost())
        w._reattach_all()
        self.assertFalse(w.detached)
        self.assertIs(w.slots[5].currentWidget(), w.pages[5])

    def test_topmost_toggle_keeps_window_visible_and_placed(self):
        """setWindowFlag 会隐藏窗口，置顶/取消置顶都必须重新显示。"""
        w = self.window
        w.show()
        w._detach_page(3)
        win = w.detached[3]
        self.app.processEvents()
        self.assertTrue(win.isVisible())
        geometry = win.geometry()
        win.top_btn.setChecked(True)
        self.app.processEvents()
        self.assertTrue(win.is_topmost())
        self.assertTrue(win.isVisible())
        self.assertEqual(win.geometry(), geometry)
        win.top_btn.setChecked(False)
        self.app.processEvents()
        self.assertFalse(win.is_topmost())
        self.assertTrue(win.isVisible())
        self.assertEqual(win.geometry(), geometry)
        # 非活动状态下的重复调用不应把窗口藏起来。
        win.set_topmost(False)
        self.assertTrue(win.isVisible())

    def test_detached_window_is_not_owned_by_main_window(self):
        """独立窗口不能挂主窗口做父级。

        Windows 上带父级的顶层窗口是 owned window，会永久压在主窗口上面，
        用户无法把独立窗口换到主窗口下面；无父级窗口必须自带样式表补回外观。
        """
        w = self.window
        w._detach_page(3)
        win = w.detached[3]
        self.assertIsNone(win.parentWidget())
        self.assertTrue(win.isWindow())
        self.assertTrue(win.styleSheet())
        self.assertEqual(win.styleSheet(), w.styleSheet())
        # 拆出去的页面控件本身仍属于独立窗口。
        self.assertIs(w.pages[3].window(), win)

    def test_main_close_closes_detached_windows(self):
        w = self.window
        w.show()
        w._detach_page(2)
        win = w.detached[2]
        self.app.processEvents()
        self.assertTrue(win.isVisible())
        w.close()                       # 关主窗口不能让独立窗口留在桌面上
        self.app.processEvents()
        self.assertFalse(w.detached)
        self.assertFalse(win.isVisible())

    def test_detached_page_keeps_rendering_while_other_page_selected(self):
        w = self.window
        w._detach_page(2)                  # 比赛地图拆出去后仍要画轨迹
        w._select_page(6)
        frame = (140.0, -260.0, 15.0) + (0.0,) * 21
        w.latest = frame
        w.traj_ring.clear()
        w.traj_ring.append(0.0, (frame[0] * core.OPS_CM_TO_MM,
                                 frame[1] * core.OPS_CM_TO_MM))
        w._render_ui()
        self.assertEqual(w.map_view.trail_item.path().elementCount(), 1)

    def test_activation_guard_keeps_keyboard_remote_in_detached_window(self):
        w = self.window
        w._detach_page(3)
        win = w.detached[3]
        w.isActiveWindow = lambda: False
        win.isActiveWindow = lambda: True
        self.assertTrue(w._own_window_active())
        w._manual_start((10, 10, 0), raw=True)
        self.assertIsNotNone(w.manual_vector)
        w._activation_guard()
        self.assertIsNotNone(w.manual_vector)   # 焦点在独立窗口：不误停
        win.isActiveWindow = lambda: False
        w._activation_guard()
        self.assertIsNone(w.manual_vector)      # 整个应用失焦才停车


if __name__ == "__main__":
    unittest.main()
