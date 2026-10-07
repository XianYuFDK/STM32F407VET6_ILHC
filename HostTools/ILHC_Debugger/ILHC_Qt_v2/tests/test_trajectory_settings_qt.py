"""旧轨迹调参移除后的真实Qt入口、会话与终端兼容回归，无串口。"""
import argparse
import os
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import main
import core


class TrajectorySettingsQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.w = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        opened = threading.Event()
        opened.set()
        self.w.worker = SimpleNamespace(opened=opened)

    def tearDown(self):
        self.w.worker = None
        self.w.sim = None
        self.w.real_match = None
        self.w._real_future = None
        self.w.close()
        self.w._planner_pool.shutdown(wait=True, cancel_futures=True)

    def test_only_current_pages_and_chassis_shortcut_remain(self):
        expected = ['总览', '实时波形', '比赛地图', '底盘调参', 'DM 电机',
                    '数据记录', '命令终端', '28 / 35 步进', '视觉跟踪']
        self.assertEqual(self.w.page_names, expected)
        self.assertEqual(self.w.stack.count(), len(expected))
        self.assertEqual([b.text() for b in self.w.nav_buttons], expected)
        for attribute in ('trajectory_rows', 'trajectory_status', 'trajectory_page_index',
                          'trajectory_param_timer', '_trajectory_queue', '_trajectory_pending'):
            self.assertFalse(hasattr(self.w, attribute), attribute)
        buttons = self.w.pages[2].findChildren(main.QPushButton)
        self.assertNotIn('轨迹速度调参', [b.text() for b in buttons])
        shortcut = next(b for b in buttons if b.text() == '底盘调参')
        shortcut.click()
        self.assertEqual(self.w.stack.currentIndex(), 3)

    def test_page_switch_and_session_reset_have_no_obsolete_queries(self):
        for index in range(len(self.w.pages)):
            self.w._select_page(index)
        commands = list(self.w.line_q.queue)
        self.assertTrue(commands)  # 视觉页仍需文字回读。
        self.assertTrue(all(command.startswith('GET ') for command in commands))
        self.assertFalse(any(command[4:] in core.TRAJECTORY_NAMES for command in commands))
        self.w._clear_command_queues()
        self.assertFalse(list(self.w.line_q.queue))
        self.w._apply_param_readback('KPX', 6)
        self.assertIn('6', self.w.chassis_rows['KPX'].readback.text())

    def test_legacy_terminal_replies_remain_visible_without_page(self):
        with patch.object(self.w, 'log') as log:
            self.w.send_line('GET TVMAX')
            self.w._apply_param_readback('TVMAX', 500)
            self.w.fw_text_q.put('固件文本: ERR TRAJ PARAM')
            self.w._process_frames()
        self.assertIn('GET TVMAX', list(self.w.line_q.queue))
        messages = [call.args[0] for call in log.call_args_list]
        self.assertTrue(any('TVMAX=500' in message for message in messages))
        self.assertTrue(any('ERR TRAJ PARAM' in message for message in messages))

    def test_legacy_terminal_writes_still_require_idle_hardware(self):
        self.w.send_line('TVMAX=500')
        self.assertEqual(list(self.w.line_q.queue), ['TVMAX=500'])
        self.w._drain_queue(self.w.line_q)
        for state in ('uploading', 'running', 'simulated', 'disconnected'):
            with self.subTest(state=state), patch.object(self.w, 'log') as log:
                self.w._real_future = object() if state == 'uploading' else None
                self.w.real_match = SimpleNamespace(active=True) if state == 'running' else None
                self.w.sim = object() if state == 'simulated' else None
                if state == 'disconnected':
                    self.w.worker.opened.clear()
                self.w.send_line('TVMAX=250')
                self.assertFalse(list(self.w.line_q.queue))
                log.assert_called()
                self.assertEqual(log.call_args.args[1], 'warn')


if __name__ == '__main__':
    unittest.main(verbosity=2)
