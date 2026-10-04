"""无硬件验证串口退出竞态、超时和跨会话错误隔离。"""
import argparse
import os
import queue
import struct
import threading
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import core
import main


class Port:
    def __init__(self):
        self.is_open = True
        self.in_waiting = 0
        self.calls = []
        self.reading = threading.Event()
        self.release = threading.Event()

    def write(self, data):
        assert self.is_open
        self.calls.append(("write", data, threading.get_ident()))
        return len(data)

    def read(self, size):
        self.reading.set()
        self.release.wait(2)
        return b""

    def reset_output_buffer(self):
        self.calls.append(("reset", None, threading.get_ident()))

    def close(self):
        self.calls.append(("close", None, threading.get_ident()))
        self.is_open = False


class SerialLifecycleTests(unittest.TestCase):
    def test_stop_timestamp_requires_a_complete_successful_write(self):
        worker=core.SerialWorker('TEST',115200,queue.Queue(),queue.Queue())
        worker.ser=Port()
        with patch.object(worker.ser,'write',return_value=2):
            self.assertFalse(worker._write_line('STOP'))
        self.assertEqual(worker.last_stop_write_monotonic,0)
        with patch.object(core.time,'monotonic',return_value=123.5):
            self.assertTrue(worker._write_line('STOP'))
        self.assertEqual(worker.last_stop_write_monotonic,123.5)
        self.assertTrue(worker._write_line('GOTO=0,0,0'))
        self.assertEqual(worker.last_stop_write_monotonic,123.5)

    def test_stop_while_port_is_opening(self):
        port = Port()
        opening = threading.Event()
        release = threading.Event()
        def open_port(*args, **kwargs):
            opening.set()
            release.wait(2)
            return port
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), queue.Queue())
        with patch.object(core.serial, "Serial", side_effect=open_port):
            worker.start()
            try:
                self.assertTrue(opening.wait(2))
                worker.request_stop(True)
                self.assertEqual(port.calls, [])
            finally:
                release.set()
                worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual([c[1] for c in port.calls],
                         [b"STOP\n", b"DMSTOP\n", b"DMOFF\n", None])

    def test_stop_during_read_keeps_all_io_in_worker(self):
        port = Port()
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), queue.Queue())
        with patch.object(core.serial, "Serial", return_value=port):
            worker.start()
            try:
                self.assertTrue(port.reading.wait(2))
                worker.request_stop(True)
                self.assertTrue(port.is_open)
                self.assertEqual(len(port.calls), 1)
            finally:
                port.release.set()
                worker.request_stop(True)
                worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual([c[1] for c in port.calls],
                         [b"VOFA\n", b"STOP\n", b"DMSTOP\n", b"DMOFF\n", None])
        self.assertEqual({c[2] for c in port.calls}, {worker.ident})

    def test_write_timeout_preserves_rx_and_does_not_replay_command(self):
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), queue.Queue())
        worker.line_q.put("MANUAL=60,0,0")
        worker.line_q.put("PING")
        packet = struct.pack("<24f", *range(24)) + core.FRAME_TAIL

        class TimeoutPort(Port):
            def read(self, size):
                return packet

            def write(self, data):
                super().write(data)
                if data.startswith(b"MANUAL"):
                    raise core.serial.SerialTimeoutException("测试超时")
                if data.endswith(b"PING\n"):
                    worker.stop_flag = True
                return len(data)

        port = TimeoutPort()
        errors = []
        worker.err_cb = errors.append
        with patch.object(core.serial, "Serial", return_value=port):
            worker.run()
        self.assertFalse(errors)
        self.assertFalse(worker.frame_q.empty())
        self.assertEqual([c[1] for c in port.calls if c[0] == "write"],
                         [b"VOFA\n", b"MANUAL=60,0,0\n", b"!\nPING\n"])

    def test_repeated_timeouts_or_device_loss_close_connection(self):
        for failure in (core.serial.SerialTimeoutException, core.serial.SerialException):
            with self.subTest(failure=failure):
                errors = []
                worker = core.SerialWorker("TEST", 115200, queue.Queue(), queue.Queue(), err_cb=errors.append)
                worker.line_q.put("PING")
                worker.line_q.put("PING")
                port = Port()
                port.release.set()
                with patch.object(core.serial, "Serial", return_value=port), \
                     patch.object(port, "write", side_effect=failure("测试故障")):
                    worker.run()
                self.assertEqual(len(errors), 1)
                self.assertFalse(port.is_open)


class WindowLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.window = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))

    def tearDown(self):
        self.window.worker = None
        self.window._closing_worker = None
        self.window.close()

    def test_old_error_does_not_stop_new_session(self):
        current = object()
        self.window.worker = current
        with patch.object(main.QMessageBox, "critical") as dialog:
            self.window._on_worker_error(object(), "旧连接错误")
        self.assertIs(self.window.worker, current)
        dialog.assert_not_called()

    def test_reconnect_and_sim_wait_for_previous_worker(self):
        port = Port()
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), queue.Queue())
        with patch.object(core.serial, "Serial", return_value=port) as serial_open:
            worker.start()
            try:
                self.assertTrue(port.reading.wait(2))
                self.window._closing_worker = worker
                self.window.toggle_connect(True)
                self.window.toggle_sim(True)
                self.assertIsNone(self.window.worker)
                self.assertIsNone(self.window.sim)
                self.assertEqual(serial_open.call_count, 1)
            finally:
                worker.request_stop(False)
                port.release.set()
                worker.join(2)
        self.assertTrue(self.window._previous_worker_finished())

    def test_nonfinite_telemetry_renders(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            values = [0.0] * 24
            values[12] = values[16] = value
            self.window.latest = values
            self.window._render_ui()
            self.assertIn("无效", self.window.dm_status_big.text())

    def test_close_waits_for_worker_before_destroying_window(self):
        port = Port()
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), queue.Queue())
        with patch.object(core.serial, "Serial", return_value=port):
            worker.start()
            try:
                self.assertTrue(port.reading.wait(2))
                self.window.worker = worker
                event = main.QCloseEvent()
                with patch.object(main.QTimer, "singleShot") as retry:
                    self.window.closeEvent(event)
                self.assertFalse(event.isAccepted())
                retry.assert_called_once()
                self.assertTrue(port.is_open)
            finally:
                worker.request_stop(True)
                port.release.set()
                worker.join(2)
        event = main.QCloseEvent()
        self.window.closeEvent(event)
        self.assertTrue(event.isAccepted())


if __name__ == "__main__":
    unittest.main()
