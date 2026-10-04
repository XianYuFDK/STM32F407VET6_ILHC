"""离屏轨迹调参：真实界面、队列、回读、JSON和执行门禁，无串口。"""
import argparse
import json
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import main
import core

class TrajectorySettingsQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.app=main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.w=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
        self.w.trajectory_param_timer.stop()
        opened=threading.Event();opened.set()
        self.w.worker=SimpleNamespace(opened=opened)

    def tearDown(self):
        self.w.worker=None;self.w.close()
        self.w._planner_pool.shutdown(wait=True,cancel_futures=True)

    def test_new_page_edit_and_default_do_not_send(self):
        self.assertEqual(self.w.page_names[-1],'轨迹调参')
        self.assertEqual(len(self.w.trajectory_rows),12)
        row=self.w.trajectory_rows['TVMAX'];row.spin.setValue(80)
        self.assertEqual(list(self.w.line_q.queue),[])
        self.w._default_trajectory_settings()
        self.assertEqual(row.spin.value(),500)
        self.assertEqual(list(self.w.line_q.queue),[])
        self.assertIn('—',row.readback.text())

    def test_fast_preset_is_local_until_sent(self):
        self.w.trajectory_rows['TVMAX'].spin.setValue(80)
        self.w._fast_trajectory_settings()
        self.assertEqual(self.w.trajectory_rows['TVMAX'].spin.value(),500)
        self.assertEqual(self.w.trajectory_rows['TVARC'].spin.value(),150)
        self.assertFalse(self.w._trajectory_queue)
        self.assertEqual(list(self.w.line_q.queue),[])

    def test_one_setting_sends_then_waits_for_actual_readback(self):
        self.w._send_trajectory_setting('TVMAX','80')
        self.assertEqual(list(self.w.line_q.queue),[])
        self.w._service_trajectory_settings()
        self.assertEqual(list(self.w.line_q.queue),['TVMAX=80','GET TVMAX'])
        self.assertIn('—',self.w.trajectory_rows['TVMAX'].readback.text())
        self.w._apply_param_readback('TVMAX',80)
        self.assertIn('80',self.w.trajectory_rows['TVMAX'].readback.text())
        self.assertEqual(self.w.trajectory_rows['TVMAX'].spin.value(),500)
        self.assertIn('收到STM32回读',self.w.trajectory_status.text())

    def test_send_all_is_paced_and_confirmed_one_at_a_time(self):
        self.w._send_all_trajectory_settings()
        for name,_,_,_,default,_ in core.TRAJECTORY_PARAMS:
            self.w._service_trajectory_settings()
            first=list(self.w.line_q.queue)
            self.w._service_trajectory_settings()
            self.assertEqual(first,list(self.w.line_q.queue))
            self.assertEqual(first[-1],'GET '+name)
            self.w._apply_param_readback(name,default)
        self.assertEqual(len(list(self.w.line_q.queue)),24)
        self.assertFalse(self.w._trajectory_pending)
        self.assertIn('收到STM32回读',self.w.trajectory_status.text())

    def test_mismatched_readback_aborts_remaining_writes(self):
        self.w._send_all_trajectory_settings();self.w._service_trajectory_settings()
        self.w._apply_param_readback('TVMAX',80)
        self.assertFalse(self.w._trajectory_queue)
        self.assertIn('不符',self.w.trajectory_status.text())

    def test_busy_and_sim_mode_refuse_settings_including_console(self):
        self.w.real_match=SimpleNamespace(active=True)
        self.w._send_trajectory_setting('TVMAX','80')
        self.w.send_line('TVMAX=80')
        self.assertFalse(self.w._trajectory_queue)
        self.assertEqual(list(self.w.line_q.queue),[])
        self.w.real_match=None;self.w.sim=object()
        self.w._send_trajectory_setting('TVMAX','80')
        self.assertIn('PC模拟',self.w.trajectory_status.text());self.w.sim=None

    def test_pending_write_blocks_match_start(self):
        self.w._send_trajectory_setting('TVMAX','80')
        self.w._start_real_match()
        self.assertIsNone(self.w._real_future)
        self.assertIn('参数正在',self.w.competition_status.text())

    def test_timeout_and_old_firmware_do_not_claim_success(self):
        self.w._send_all_trajectory_settings()
        with patch.object(main.time,'monotonic',return_value=10):self.w._service_trajectory_settings()
        with patch.object(main.time,'monotonic',return_value=14):self.w._service_trajectory_settings()
        self.assertFalse(self.w._trajectory_queue)
        self.assertIn('超时',self.w.trajectory_status.text())
        self.w._send_all_trajectory_settings();self.w._service_trajectory_settings()
        self.w.fw_text_q.put('固件文本: ERR PARAM UNKNOWN; GET KPX')
        self.w._process_frames()
        self.assertIn('未确认生效',self.w.trajectory_status.text())
        self.assertFalse(self.w._trajectory_pending)

    def test_session_reset_clears_pending_and_readbacks(self):
        self.w._apply_param_readback('TVMAX',80)
        self.w._send_all_trajectory_settings();self.w._service_trajectory_settings()
        self.w._clear_command_queues()
        self.assertFalse(self.w._trajectory_queue);self.assertFalse(self.w._trajectory_pending)
        self.assertIn('—',self.w.trajectory_rows['TVMAX'].readback.text())

    def test_json_load_save_and_invalid_file_are_local_and_atomic(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'settings.json'
            self.w.trajectory_rows['TVMAX'].spin.setValue(80)
            with patch.object(main.QFileDialog,'getSaveFileName',return_value=(str(path),'')):
                self.w._save_trajectory_settings()
            self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['parameters']['TVMAX'],80)
            self.w.trajectory_rows['TVMAX'].spin.setValue(250)
            with patch.object(main.QFileDialog,'getOpenFileName',return_value=(str(path),'')):
                self.w._load_trajectory_settings()
            self.assertEqual(self.w.trajectory_rows['TVMAX'].spin.value(),80)
            data=json.loads(path.read_text(encoding='utf-8'));data['parameters']['TKPX']=999
            data['parameters']['TVMAX']=100
            path.write_text(json.dumps(data),encoding='utf-8')
            with patch.object(main.QFileDialog,'getOpenFileName',return_value=(str(path),'')):
                self.w._load_trajectory_settings()
            self.assertEqual(self.w.trajectory_rows['TVMAX'].spin.value(),80)
            self.assertIn('加载失败',self.w.trajectory_status.text())
            self.assertEqual(list(self.w.line_q.queue),[])

if __name__=='__main__': unittest.main(verbosity=2)
