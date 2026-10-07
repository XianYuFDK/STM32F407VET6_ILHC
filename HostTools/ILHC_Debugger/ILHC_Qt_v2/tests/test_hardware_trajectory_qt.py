"""离屏Qt实机按钮与取消测试，用未启动串口替身，不打开端口。"""
import argparse
import copy
import os
import threading
import tempfile
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
        # OnlineWorkerStub走REAL代码路径，但日志只能进入测试临时目录。
        temporary=tempfile.TemporaryDirectory(prefix='ilhc-hardware-qt-')
        self.addCleanup(temporary.cleanup)
        environment=patch.dict(os.environ,ILHC_RUN_LOG_DIR=temporary.name)
        environment.start()
        self.addCleanup(environment.stop)
        self.w=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
        self.w.nav_map=competition.load_profile()
        self.w.map_ox=self.w.map_oy=self.w.map_theta=0
        opened=threading.Event();opened.set()
        self.w.worker=OnlineWorkerStub(opened=opened,parser=core.FrameParser())
        self.w.latest=[0.0]*24;self.w.wheel_state=True

    def read_parameters(self):
        pending=getattr(self.w, '_coordinate_read', None)
        if pending is None: return
        for name in core.CHASSIS_NAMES:
            pending['next']=0
            self.w._poll_coordinate_parameters()
            self.w._apply_param_readback(name,core.CHASSIS_DEFAULTS[core.SIM_PARAM_ATTRS[name]])
        self.w._poll_coordinate_parameters()

    def test_parameter_readback_timeout_and_stop_cannot_start(self):
        self.w._start_real_match()
        pending=self.w._coordinate_read
        self.assertIsNone(self.w._real_future)
        self.assertFalse(any(c.startswith('CBEGIN=') for c in self.w.line_q.queue))
        pending['deadline']=0
        self.w._poll_real_match()
        self.assertIsNone(self.w._coordinate_read)
        self.assertIsNone(self.w._real_future)
        self.w._start_real_match()
        self.w.send_line('STOP')
        self.read_parameters()
        self.assertIsNone(self.w._real_future)

    def test_coordinate_parameter_loss_retries_only_missing_without_starting_motion(self):
        callbacks=[]
        with patch.object(main.time,'monotonic',return_value=100):
            self.w._read_coordinate_parameters(callbacks.append)
            for i,name in enumerate(core.CHASSIS_NAMES):
                with patch.object(main.time,'monotonic',return_value=100+i*.15):
                    self.w._poll_coordinate_parameters()
                if name!='KPZ':
                    self.w._apply_param_readback(name,core.CHASSIS_DEFAULTS[core.SIM_PARAM_ATTRS[name]])
            self.assertFalse(callbacks)
            with patch.object(main.time,'monotonic',return_value=101.2):
                self.w._poll_coordinate_parameters()
            self.assertEqual(self.w._coordinate_read['attempts']['KPZ'],2)
            self.assertTrue(all(v==1 for k,v in self.w._coordinate_read['attempts'].items() if k!='KPZ'))
            self.w._apply_param_readback('KPZ',9)
            with patch.object(main.time,'monotonic',return_value=101.3):
                self.w._poll_coordinate_parameters()
        self.assertEqual(len(callbacks),1)
        self.assertEqual(callbacks[0],core.CHASSIS_DEFAULTS)
        self.assertFalse(any(c.startswith(('CBEGIN=','TRUN=')) for c in self.w.line_q.queue))

    def test_coordinate_parameter_timeout_names_missing_and_session_change_is_distinct(self):
        self.w._read_coordinate_parameters(lambda values:self.fail('不得发车'))
        pending=self.w._coordinate_read
        pending['sent'].update(core.CHASSIS_NAMES)
        pending['values'].update(core.CHASSIS_DEFAULTS)
        del pending['values']['zvmin']
        pending['attempts']['ZVMIN']=4;pending['deadline']=0
        self.w._poll_coordinate_parameters()
        self.assertIn('ZVMIN',self.w.map_status.text())
        self.assertIn('请求4次',self.w.map_status.text())
        self.assertNotIn('会话',self.w.map_status.text())
        self.w._read_coordinate_parameters(lambda values:self.fail('不得发车'))
        self.w.worker=None
        self.w._poll_coordinate_parameters()
        self.assertIn('串口会话已改变',self.w.map_status.text())

    def tearDown(self):
        self.w._clear_path();self.w.worker=None;self.w.close()
        self.w._planner_pool.shutdown(wait=True,cancel_futures=True)

    def test_runtime_collision_switch_only_controls_map_collision_check(self):
        job=self.click_path()
        job.state='RUNNING'
        self.w.latest[:3]=[105,0,0]
        self.assertTrue(self.w.real_collision_check.isChecked())
        self.w.real_collision_check.setChecked(False)
        with patch.object(job,'tick'):
            self.w._poll_real_match()
        self.assertEqual(job.state,'RUNNING')
        self.assertIn('地图碰撞保护关闭',self.w.map_status.text())
        self.w.real_collision_check.setChecked(True)
        with patch.object(job,'tick'):
            self.w._poll_real_match()
        self.assertEqual(job.state,'CANCELLED')
        self.assertIn('二维码板',job.reason)
        self.assertIn('触发位置X=105.0cm',job.reason)
        self.assertEqual(job.batch['collision_trigger']['check'],'当前车体')
        self.assertIn('STOP',list(self.w.urgent_q.queue))

    def test_stale_telemetry_cancels_with_specific_reason_even_collision_off(self):
        job=self.click_path();job.state='RUNNING'
        self.w.real_collision_check.setChecked(False)
        self.w.worker.frame_monotonic=99.7
        with patch.object(main.time,'monotonic',return_value=100):
            self.w._poll_real_match()
        self.assertEqual(job.state,'RUNNING','300ms仍未超过原350ms门限')
        self.w.worker.frame_monotonic=99.6
        with patch.object(main.time,'monotonic',return_value=100):
            self.w._poll_real_match()
        self.assertEqual(job.state,'CANCELLED')
        self.assertIn('OPS遥测过期：400ms',job.reason)
        self.assertNotIn('地图、参数或连接变化',job.reason)
        self.assertEqual(job.batch['validation_trigger']['telemetry_age_ms'],400)
        self.assertEqual(job.batch['validation_trigger']['changed_fields'],[])
        self.assertIn('STOP',list(self.w.urgent_q.queue))

    def test_silent_calibration_change_reports_changed_field_and_snapshot(self):
        job=self.click_path();job.state='RUNNING'
        self.w.map_ox+=10
        self.w._poll_real_match()
        self.assertEqual(job.state,'CANCELLED')
        self.assertEqual(job.reason,'导航配置已变化：X偏移')
        self.assertEqual(job.batch['validation_trigger']['changed_fields'],['X偏移'])

    def test_disabling_map_collision_does_not_disable_wheel_or_link_gate(self):
        job=self.click_path();job.state='RUNNING'
        self.w.real_collision_check.setChecked(False)
        self.w.wheel_state=False
        self.w._poll_real_match()
        self.assertEqual(job.state,'CANCELLED')
        self.assertIn('STOP',list(self.w.urgent_q.queue))

    def test_screenshot_current_pose_is_clear_and_sweep_reason_has_previous_pose(self):
        job=self.click_path();job.state='RUNNING'
        self.w.latest[:3]=[22.4,100.8,11.8]
        p=(*core.field_to_layout(224,1008),11.8-180)
        self.assertIsNone(self.w._real_scene.pose_reason(*p))
        self.w._real_previous_pose=(2250,1200,180)
        with patch.object(job,'tick'), patch.object(self.w._real_scene,'moving_pose_reason',return_value='二维码板'):
            self.w._poll_real_match()
        self.assertEqual(job.state,'CANCELLED')
        self.assertIn('相邻帧扫掠',job.reason)
        self.assertIn('前帧X=105.0cm',job.reason)
        self.assertEqual(job.batch['collision_trigger']['field_y_mm'],1008)

    def test_missing_competition_config_is_loaded_before_parameter_snapshot(self):
        self.w.nav_map.pop('competition')
        geometry={k:copy.deepcopy(self.w.nav_map[k]) for k in ('map_id','map_version','rects','circles','drivable_polygons')}
        self.w._start_real_match()
        self.assertEqual(self.w.nav_map['competition']['start_zones'],{'1':[2250,2250],'2':[2250,150]})
        self.assertIsNotNone(self.w._coordinate_read)
        for key,value in geometry.items():self.assertEqual(self.w.nav_map[key],value)
        signature=self.w._navigation_signature()
        from concurrent.futures import Future
        with patch.object(self.w._planner_pool,'submit',return_value=Future()) as submit:
            self.read_parameters()
            submit.assert_called_once()
        self.assertIsNotNone(self.w._real_future)
        self.assertEqual(signature,self.w._real_context['signature'])
        self.assertIn('自动加载',self.w.console.toPlainText())

    def test_missing_profile_at_zone_two_preserves_live_ops_and_obstacles(self):
        self.w.nav_map.pop('coordinate_reference',None)  # Legacy zone-based map compatibility.
        self.w.nav_map.pop('competition');self.w.zone_combo.setCurrentIndex(1)
        self.w.latest[0]=210.;self.w.sim_obstacles=[(700,1200)]
        self.w._start_real_match()
        from concurrent.futures import Future
        with patch.object(self.w._planner_pool,'submit',return_value=Future()):self.read_parameters()
        self.assertIsNotNone(self.w._real_future)
        self.assertEqual(self.w._real_context['zone'],2)
        self.assertEqual(self.w._real_context['obstacles'],((700,1200),))
        self.assertEqual(self.w.latest[0],210.)
        self.assertNotIn('ZERO',self.w.urgent_q.queue)

    def test_invalid_explicit_profile_is_rejected_without_upload(self):
        self.w.nav_map['competition']['start_zones']['1']=[-100,2250]
        self.w._start_real_match()
        self.assertIsNone(self.w._coordinate_read)
        self.assertIsNone(self.w._real_future)
        self.assertIn('start_zones.1',self.w.competition_status.text())
        self.assertFalse(any(c.startswith('CBEGIN') for c in self.w.line_q.queue))

    def test_wrong_actual_start_refuses_without_zero(self):
        self.w.latest[0]=1
        self.w._start_real_match()
        self.read_parameters()
        self.assertIsNone(self.w._real_future);self.assertIn('5mm',self.w.competition_status.text())
        self.assertNotIn('ZERO',list(self.w.urgent_q.queue))

    def test_real_preflight_caps_and_stop_discard_pending_points(self):
        self.w._start_real_match()
        self.read_parameters()
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
        self.assertEqual(job.state,'CAPS');self.assertIn('CCAPS',list(self.w.line_q.queue))
        # 启停区规划+10mm正好贴边，实际0.4mm定位误差消耗裕量后仍有保留1mm。
        self.assertIsNone(self.w._real_scene.pose_reason(2250.4,2250.4,-180))
        self.w.fw_text_q.put('固件文本: CCAPS 8 2048 3');self.w._process_frames()
        self.assertEqual(job.state,'BEGIN')
        self.w.fw_text_q.put('固件文本: TSTAT %d 1 0 0 0 0'%job.batch['id']);self.w._process_frames()
        self.assertTrue(any(c.startswith('CPOINT=') for c in self.w.line_q.queue))
        self.w.send_line('STOP')
        self.assertEqual(job.state,'CANCELLED')
        self.assertFalse(any(c.startswith(('CPOINT=','TRUN=')) for c in self.w.line_q.queue))
        self.w._poll_real_match();self.w.competition_status.setText('新模拟准备')
        self.w._poll_real_match();self.assertEqual(self.w.competition_status.text(),'新模拟准备')

    def test_map_change_during_preflight_cancels(self):
        self.w._start_real_match()
        self.read_parameters()
        event=self.w._real_cancel;self.w.nav_map['map_version']+=1
        self.w._clear_path('地图版本改变')
        self.assertTrue(event.is_set());self.assertIsNone(self.w._real_future)
        self.assertIn('STOP',list(self.w.urgent_q.queue))

    def click_path(self):
        self.w.latest[:3]=[15,15,0]
        self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.read_parameters()
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
        job.handle_reply('CCAPS 8 2048 3')
        job.handle_reply('TSTAT %d 1 0 0 0 0'%job.batch['id'])
        while job.state=='UPLOADING':
            job.handle_reply('TSTAT %d 1 %d 0 0 0'%(job.batch['id'],job.sent))
        self.assertEqual(job.state,'VERIFYING')
        self.assertFalse(any(s.startswith('TRUN=') for s in self.w.line_q.queue))
        job.handle_reply('TSTAT %d 3 %d 0 0 0'%(job.batch['id'],job.sent))
        commands=list(self.w.line_q.queue)
        self.assertEqual(sum(s.startswith('CPOINT=') for s in commands),job.batch['point_count'])
        self.assertEqual(commands[-1],'TRUN=%d'%job.batch['id'])
        self.assertFalse(any(s.startswith('GOTO=') for s in commands))
        self.w.send_line('STOP')
        self.assertEqual(job.state,'CANCELLED')
        self.assertFalse(any(s.startswith(('CPOINT=','TRUN=')) for s in self.w.line_q.queue))

    def test_click_stale_ops_cannot_upload_with_nominal_map(self):
        self.w.latest[:3]=[15,15,0];self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.read_parameters()
        self.w._plan_future.result(timeout=15)
        self.w.worker.frame_monotonic=time.monotonic()-core.TELEMETRY_WARN_S-1
        self.w._poll_plan()
        self.assertIsNone(self.w._real_future)
        self.assertIn('过期',self.w.map_status.text())
        self.assertFalse(any(s.startswith(('CBEGIN=','GOTO=')) for s in self.w.line_q.queue))

    def test_click_disabled_wheels_cannot_upload_with_nominal_map(self):
        self.w.latest[:3]=[15,15,0];self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.read_parameters()
        self.w._plan_future.result(timeout=15)
        self.w.wheel_state=False
        self.w._poll_plan()
        self.assertIsNone(self.w._real_future)
        self.assertIn('失能',self.w.map_status.text())
        self.assertFalse(any(s.startswith(('CBEGIN=','GOTO=')) for s in self.w.line_q.queue))

    def test_map_change_cancels_click_upload(self):
        job=self.click_path();self.w.nav_map['map_version']+=1
        self.w._poll_real_match()
        self.assertEqual(job.state,'CANCELLED')
        self.assertIn('STOP',list(self.w.urgent_q.queue))

    def test_real_click_supports_calibration_offsets_beyond_old_goto_range(self):
        self.w.nav_map.pop('coordinate_reference',None)  # Legacy explicit-mapping protocol coverage.
        self.w.map_ox=4000;self.w.latest[:3]=[-385,15,0]
        self.w.latest_received_monotonic=time.monotonic()
        self.w._on_map_click(2100,1500)
        self.read_parameters()
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
        self.w.nav_map.pop('coordinate_reference',None)  # Explicit mapping is only supported by legacy maps.
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
        self.read_parameters()
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
