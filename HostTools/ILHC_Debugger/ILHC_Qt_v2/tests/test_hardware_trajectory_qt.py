"""离屏Qt实机按钮与取消测试，用未启动串口替身，不打开端口。"""
import argparse
import os
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import main
import core
import competition_simulation as competition


class OnlineWorkerStub(SimpleNamespace):
    # 真SerialWorker后台持续更新时间；替身同样持续在线，GUI绘图耗时不伪造失联。
    @property
    def last_frame_monotonic(self): return self.__dict__.get('frame_monotonic',time.monotonic())


class HardwareQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.w=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
        self.w.nav_map=competition.load_profile()
        self.w.map_ox=self.w.map_oy=self.w.map_theta=0
        opened=threading.Event();opened.set()
        self.w.worker=OnlineWorkerStub(opened=opened,parser=core.FrameParser())
        self.w.latest=[0.0]*24;self.w.wheel_state=True

    def tearDown(self):
        self.w._clear_path();self.w.worker=None;self.w.close()
        self.w._planner_pool.shutdown(wait=True,cancel_futures=True)

    def test_missing_competition_config_refuses_full_match(self):
        self.w.nav_map.pop('competition')
        self.w._start_real_match()
        self.assertIsNone(self.w._real_future)
        self.assertIn('competition',self.w.competition_status.text())
        self.assertEqual(list(self.w.line_q.queue),[])

    def test_wrong_actual_start_refuses_without_zero(self):
        self.w.latest[0]=1
        self.w._start_real_match()
        self.assertIsNone(self.w._real_future);self.assertIn('5mm',self.w.competition_status.text())
        self.assertNotIn('ZERO',list(self.w.urgent_q.queue))

    def test_real_preflight_caps_and_stop_discard_pending_points(self):
        self.w._start_real_match()
        future=self.w._real_future
        self.assertIsNotNone(future)
        future.result(timeout=30)
        self.w._poll_real_match()
        job=self.w.real_match;self.assertIsNotNone(job,self.w.competition_status.text())
        self.assertFalse(self.w.nav_map['geometry_verified'])
        self.assertFalse(job.batch['match']['map_snapshot']['geometry_verified'])
        self.assertEqual(job.batch['station_mode'],'AUTO_ROUTE')
        self.assertEqual(job.batch['waits'],{})
        self.assertIn('自动跑图',self.w.real_match_start.text())
        self.assertFalse(hasattr(self.w,'real_match_continue'))
        self.assertGreater(self.w.map_view.skeleton_item.path().elementCount(),1)
        self.assertGreater(self.w.map_view.trajectory_arrows.path().elementCount(),1)
        self.assertEqual(job.state,'CAPS');self.assertIn('TCAPS',list(self.w.line_q.queue))
        # 启停区规划+10mm正好贴边，实际0.4mm定位误差消耗裕量后仍有保留1mm。
        self.assertIsNone(self.w._real_scene.pose_reason(2250.4,2250.4,-180))
        self.w.fw_text_q.put('固件文本: TCAPS 1 4096 3');self.w._process_frames()
        self.assertEqual(job.state,'BEGIN')
        self.w.fw_text_q.put('固件文本: TSTAT %d 1 0 0 0 0'%job.batch['id']);self.w._process_frames()
        self.assertTrue(any(c.startswith('TPOINT=') for c in self.w.line_q.queue))
        self.w.send_line('STOP')
        self.assertEqual(job.state,'CANCELLED')
        self.assertFalse(any(c.startswith(('TPOINT=','TRUN=')) for c in self.w.line_q.queue))
        self.w._poll_real_match();self.w.competition_status.setText('新模拟准备')
        self.w._poll_real_match();self.assertEqual(self.w.competition_status.text(),'新模拟准备')

    def test_map_change_during_preflight_cancels(self):
        self.w._start_real_match()
        event=self.w._real_cancel;self.w.nav_map['map_version']+=1
        self.w._clear_path('地图版本改变')
        self.assertTrue(event.is_set());self.assertIsNone(self.w._real_future)
        self.assertIn('STOP',list(self.w.urgent_q.queue))

    def click_path(self):
        self.w.latest[:3]=[15,15,0]
        self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.w._plan_future.result(timeout=15)
        self.w._poll_plan()
        self.assertIsNotNone(self.w._real_future,self.w.map_status.text())
        self.w._real_future.result(timeout=15)
        self.w._poll_real_match()
        return self.w.real_match

    def test_real_click_automatically_uploads_all_then_runs_without_goto(self):
        job=self.click_path()
        self.assertIsNotNone(job,self.w.competition_status.text())
        self.assertFalse(self.w.nav_map['geometry_verified'])
        self.assertEqual(job.batch['kind'],'STM32_POINT_PATH')
        self.assertEqual(job.state,'CAPS')
        job.handle_reply('TCAPS 1 4096 3')
        job.handle_reply('TSTAT %d 1 0 0 0 0'%job.batch['id'])
        while job.state=='UPLOADING':
            job.handle_reply('TSTAT %d 1 %d 0 0 0'%(job.batch['id'],job.sent))
        self.assertEqual(job.state,'VERIFYING')
        self.assertFalse(any(s.startswith('TRUN=') for s in self.w.line_q.queue))
        job.handle_reply('TSTAT %d 3 %d 0 0 0'%(job.batch['id'],job.sent))
        commands=list(self.w.line_q.queue)
        self.assertEqual(sum(s.startswith('TPOINT=') for s in commands),job.batch['point_count'])
        self.assertEqual(commands[-1],'TRUN=%d'%job.batch['id'])
        self.assertFalse(any(s.startswith('GOTO=') for s in commands))
        self.w.send_line('STOP')
        self.assertEqual(job.state,'CANCELLED')
        self.assertFalse(any(s.startswith(('TPOINT=','TRUN=')) for s in self.w.line_q.queue))

    def test_click_stale_ops_cannot_upload_with_nominal_map(self):
        self.w.latest[:3]=[15,15,0];self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.w._plan_future.result(timeout=15)
        self.w.worker.frame_monotonic=time.monotonic()-core.TELEMETRY_WARN_S-1
        self.w._poll_plan()
        self.assertIsNone(self.w._real_future)
        self.assertIn('过期',self.w.map_status.text())
        self.assertFalse(any(s.startswith(('TBEGIN=','GOTO=')) for s in self.w.line_q.queue))

    def test_click_disabled_wheels_cannot_upload_with_nominal_map(self):
        self.w.latest[:3]=[15,15,0];self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.w._plan_future.result(timeout=15)
        self.w.wheel_state=False
        self.w._poll_plan()
        self.assertIsNone(self.w._real_future)
        self.assertIn('失能',self.w.map_status.text())
        self.assertFalse(any(s.startswith(('TBEGIN=','GOTO=')) for s in self.w.line_q.queue))

    def test_map_change_cancels_click_upload(self):
        job=self.click_path();self.w.nav_map['map_version']+=1
        self.w._poll_real_match()
        self.assertEqual(job.state,'CANCELLED')
        self.assertIn('STOP',list(self.w.urgent_q.queue))

    def test_real_click_supports_calibration_offsets_beyond_old_goto_range(self):
        self.w.map_ox=4000;self.w.latest[:3]=[-385,15,0]
        self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.w._plan_future.result(timeout=15);self.w._poll_plan()
        self.assertIsNotNone(self.w._real_future,self.w.map_status.text())
        batch,_=self.w._real_future.result(timeout=15)
        self.assertEqual(batch['points'][0][0],-38500)
        self.assertFalse(any(s.startswith('GOTO=') for s in self.w.line_q.queue))


    def _begin_home_after_stop(self):
        self.w.latest[:3] = [16.2, 26.7, 36.9]  # 用户错误13截图实际OPS
        self.w.latest_received_monotonic = 99.98
        self.w.worker.frame_monotonic = 100.0
        with patch.object(main.time, 'monotonic', return_value=100.0):
            self.w.send_line('STOP')
            self.w._goto_home()
        self.assertEqual(self.w._direct_home['state'], 'WAIT_STOP')
        self.assertFalse(any(c.startswith('GOTO=') for c in self.w.line_q.queue))
        return self.w._direct_home

    def _home_frames(self, start, count=13, pose=None):
        for i in range(count):
            tick = start+i*.02
            self.w.worker.frame_monotonic = tick
            self.w.latest_received_monotonic = tick
            if pose is not None:
                self.w.latest[:3] = pose
            with patch.object(main.time, 'monotonic', return_value=tick):
                self.w._poll_direct_home()

    def test_normal_home_always_plans_even_with_click_planning_disabled(self):
        self.w.plan_click_check.setChecked(False)
        with patch.object(self.w, '_request_plan') as planner:
            self.w._goto_home()
        planner.assert_called_once_with(*core.ZONE_CENTER[1], execute_real=True, home_return=True)
        self.assertIsNone(self.w._direct_home)
        self.assertFalse(self.w._home_after_stop)
        self.assertEqual(list(self.w.line_q.queue), [])

    def test_stop_home_waits_for_written_stop_and_fresh_stationary_pose(self):
        self.w.nav_map['drivable'] = []  # 停止恢复不调用地图几何门禁
        with patch.object(self.w, '_request_plan', side_effect=AssertionError('不应规划')):
            ctx = self._begin_home_after_stop()
        self._home_frames(100.02)
        self.assertEqual(ctx['state'], 'WAIT_STOP', '未成功写STOP不得发GOTO')
        self.w.worker.last_stop_write_monotonic = 100.28
        self._home_frames(100.30, count=5)
        self.assertEqual(ctx['state'], 'WAIT_STOP', '不足200ms不得发GOTO')
        with patch.object(main.time, 'monotonic', return_value=100.38):
            for _ in range(20): self.w._poll_direct_home()
        self.assertEqual(ctx['state'], 'WAIT_STOP', '同一帧不能累加停稳时间')
        self._home_frames(100.40, count=10)
        commands = list(self.w.line_q.queue)
        self.assertEqual(commands, ['GOTO=0.0,0.0,0.0'])
        self.assertEqual(ctx['state'], 'SENT')
        self.assertFalse(self.w._home_after_stop)
        self._home_frames(100.62, pose=[0,0,0])
        self.assertIsNone(self.w._direct_home)
        self.assertIn('停稳', self.w.map_status.text())
        self.assertEqual(list(self.w.line_q.queue), commands, '回库目标仅发送一次')
        self.assertNotIn('ZERO', list(self.w.urgent_q.queue))
        with patch.object(self.w, '_request_plan') as planner:
            self.w._goto_home()
        planner.assert_called_once()

    def test_moving_after_stop_cannot_dispatch_home(self):
        ctx = self._begin_home_after_stop()
        self.w.worker.last_stop_write_monotonic = 100.01
        for i in range(15):
            self._home_frames(100.02+i*.02, count=1, pose=[16.2+i*.1,26.7,36.9])
        self.assertEqual(ctx['state'], 'WAIT_STOP')
        self.assertEqual(list(self.w.line_q.queue), [])
        self._home_frames(100.34)
        self.assertEqual(ctx['state'], 'SENT')

    def test_fault13_home_bypasses_planner_and_uses_selected_zone_mapping(self):
        self.w.zone_combo.setCurrentIndex(1)
        self.w.map_ox=2100; self.w.map_oy=0; self.w.map_theta=20
        self.w.latest[:3]=[16.2,26.7,36.9]
        self.w.real_match=SimpleNamespace(active=False,state='CANCELLED',reason='STM32 FAULT：错误13',reported_terminal=True)
        self.w.worker.frame_monotonic=100
        with patch.object(main.time, 'monotonic', return_value=100), patch.object(self.w, '_request_plan') as planner:
            self.w._goto_home()
        planner.assert_not_called()
        self.w.worker.last_stop_write_monotonic=100.01
        self._home_frames(100.02)
        self.assertEqual(list(self.w.line_q.queue), ['GOTO=0.0,0.0,340.0'])
        self.assertFalse(self.w._home_after_stop)
        self.w._clear_path()
        with patch.object(self.w, '_request_plan') as planner:
            self.w._goto_home()
        planner.assert_called_once()  # 旧FAULT记录不能反复授予直接回库

    def test_cancelling_home_cannot_leave_queued_goto(self):
        ctx=self._begin_home_after_stop()
        self.w.send_line('STOP')
        self.assertIsNone(self.w._direct_home)
        self.assertEqual(list(self.w.line_q.queue), [])
        ctx=self._begin_home_after_stop()
        self.w.worker.last_stop_write_monotonic=100.01
        self._home_frames(100.02)
        self.assertEqual(ctx['state'], 'SENT')
        self.w._clear_path('重新规划')
        self.assertIsNone(self.w._direct_home)
        self.assertEqual(list(self.w.line_q.queue), [])
        self.assertIn('STOP', list(self.w.urgent_q.queue))

    def test_home_cancels_for_map_wheels_stale_pose_timeout_or_session_change(self):
        for fault in ('map','wheels','stale','timeout','session'):
            with self.subTest(fault=fault):
                self.w.wheel_state=True
                ctx=self._begin_home_after_stop()
                self.w.worker.last_stop_write_monotonic=100.01
                poll_time=100.1
                if fault=='map': self.w.nav_map['map_version']+=1
                if fault=='wheels': self.w.wheel_state=False
                if fault=='stale': self.w.worker.frame_monotonic=95
                if fault=='timeout': poll_time=106; self.w.worker.frame_monotonic=106
                if fault=='session':
                    self.w._drain_queue(self.w.urgent_q)
                    self.w.worker=OnlineWorkerStub(opened=ctx['worker'].opened,parser=core.FrameParser(),frame_monotonic=100.1)
                with patch.object(main.time, 'monotonic', return_value=poll_time):
                    self.w._poll_direct_home()
                self.assertIsNone(self.w._direct_home)
                self.assertEqual(list(self.w.line_q.queue), [])
                if fault=='session':
                    self.assertEqual(list(self.w.urgent_q.queue), [], '不把旧会话STOP发往新会话')

    def test_normal_cancel_and_completion_do_not_grant_direct_return(self):
        for state,reason in [('DONE',''),('CANCELLED','地图、参数或连接变化')]:
            with self.subTest(state=state):
                self.w.real_match=SimpleNamespace(active=False,state=state,reason=reason,reported_terminal=True)
                with patch.object(self.w, '_request_plan') as planner:
                    self.w._goto_home()
                planner.assert_called_once()
                self.assertFalse(self.w._home_after_stop)


    def test_new_serial_session_drops_sent_home_and_recovery_permission(self):
        ctx=self._begin_home_after_stop()
        self.w.worker.last_stop_write_monotonic=100.01
        self._home_frames(100.02)
        self.assertEqual(ctx['state'],'SENT')
        self.w._drain_queue(self.w.urgent_q)
        self.w.worker=OnlineWorkerStub(opened=ctx['worker'].opened,parser=core.FrameParser())
        self.w._poll_direct_home()
        self.assertEqual(list(self.w.line_q.queue),[])
        self.assertEqual(list(self.w.urgent_q.queue),[])
        self.assertFalse(self.w._home_after_stop)


    def test_manual_movement_cancels_direct_home_and_clears_stop_exception(self):
        for send_target in (False, True):
            with self.subTest(send_target=send_target):
                ctx=self._begin_home_after_stop()
                if send_target:
                    self.w.worker.last_stop_write_monotonic=100.01
                    self._home_frames(100.02)
                    self.assertEqual(ctx['state'],'SENT')
                self.w._manual_start((1,0,0))
                self.assertIsNone(self.w._direct_home)
                self.assertFalse(self.w._home_after_stop)
                commands=list(self.w.line_q.queue)
                self.assertTrue(any(c.startswith('MANUAL=') for c in commands))
                self.assertFalse(any(c.startswith('GOTO=') for c in commands))
                self.w._manual_stop()
                self.w._drain_queue(self.w.line_q)


    def test_normal_home_with_negative_zero_heading_plans_and_uploads_to_zero(self):
        self.w.latest[:3]=[104.5,103.1,-.005]
        self.w.latest_received_monotonic=time.monotonic()
        self.w.map_yaw_combo.setCurrentIndex(self.w.map_yaw_combo.findData(90.0))
        self.w._goto_home()
        self.assertIsNone(self.w._direct_home)
        self.assertIsNotNone(self.w._plan_future)
        self.w._plan_future.result(timeout=15)
        self.w.latest_received_monotonic=time.monotonic()
        self.w._poll_plan()
        self.assertIsNotNone(self.w._real_future,self.w.map_status.text())
        batch,_=self.w._real_future.result(timeout=15)
        self.assertEqual(batch['points'][-1][:2],(0,0))
        self.assertEqual(batch['points'][-1][3],0)
        self.assertEqual(batch['points'][0][4]&~1,0)
        self.assertFalse(self.w._home_after_stop)
        self.assertFalse(any(c.startswith('GOTO=') for c in self.w.line_q.queue))
        self.assertEqual(self.w.map_yaw_combo.currentData(),90.0)


if __name__=='__main__':unittest.main()
