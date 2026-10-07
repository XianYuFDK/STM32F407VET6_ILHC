"""接收端日志、会话隔离、写盘故障与离线时钟/角度分析。"""
import csv
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import core
from run_journal import RunJournal
from analyze_run import analyze,velocity
from tests.test_crc_telemetry import packet


class JournalTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.log=RunJournal(self.tmp.name,core.CHANNELS);self.addCleanup(self.log.close)

    def events(self,path):
        return [json.loads(line) for line in (path/'events.jsonl').read_text(encoding='utf-8').splitlines()]

    def test_capture_before_gui_queue_overflow_crc_rejection_and_raw_bytes(self):
        path=self.log.start('REAL',{'mapping':[0,0,0]});rid=self.log.begin_run('COORDINATE_BATCH',{}, {'points':[]})
        worker=core.SerialWorker('NO_PORT',115200,queue.Queue(maxsize=1),queue.Queue(),queue.Queue(),event_cb=self.log.emit)
        corrupt=bytearray(packet(1));corrupt[-1]^=1
        raw=packet(0)+corrupt+packet(2)
        worker._receive(raw,10)
        self.log.end_run('CANCELLED','有效遥测过期',validation_trigger={'age':.4});self.log.close()
        events=self.events(path);frames=[e for e in events if e['event']=='FRAME']
        self.assertEqual([e['device_sequence'] for e in frames],[0,2])
        self.assertTrue(all(e['run_id']==rid for e in frames))
        self.assertEqual(bytes.fromhex(next(e['bytes_hex'] for e in events if e['event']=='RX_BYTES')),raw)
        self.assertEqual(worker.queue_drops,1)
        with (path/'runs'/rid/'telemetry.csv').open(encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
        self.assertEqual(len(rows),2);self.assertEqual(len(rows[0]),30)
        result=json.loads((path/'runs'/rid/'result.json').read_text(encoding='utf-8'))
        self.assertEqual(result['validation_trigger']['age'],.4)
        self.assertEqual(self.log.error,'')

    def test_sessions_and_runs_separate_and_nan_is_explicit_null(self):
        path=self.log.start('SIM',{})
        first=self.log.begin_run('SIM_PATH',{},{});self.log.emit('FRAME',values=(1.,2.,3.,float('nan'))+(0.,)*20)
        second=self.log.begin_run('SIM_MATCH',{},{});self.log.emit('FRAME',values=(2.,3.,4.)+(0.,)*21)
        second_path=self.log.path
        self.log.close();events=self.events(path)+self.events(second_path)
        self.assertEqual([e['status'] for e in events if e['event']=='RUN_END'],['SUPERSEDED','SESSION_CLOSED'])
        self.assertIsNone(next(e for e in events if e['event']=='FRAME')['values'][3])
        self.assertNotEqual(path,second_path)
        self.assertTrue((path/'runs'/first/'plan.json').exists());self.assertTrue((second_path/'runs'/second/'plan.json').exists())
        new=self.log.start('REAL',{});self.log.begin_run('MANUAL',{},{});self.log.close()
        self.assertNotEqual(path,new)
        self.assertFalse(json.loads((path/'session.json').read_text(encoding='utf-8'))['physical_data'])
        self.assertTrue(json.loads((new/'session.json').read_text(encoding='utf-8'))['physical_data'])

    def test_disk_failure_does_not_raise_on_observer_or_write_motion(self):
        with patch.object(Path,'mkdir',side_effect=PermissionError('test disk denied')):
            self.log.start('REAL',{})
            self.log.begin_run('MANUAL',{})
            until=time.monotonic()+1
            while not self.log.error and time.monotonic()<until:time.sleep(.005)
            self.assertIn('test disk denied',self.log.error)
            for _ in range(20):self.log.emit('FRAME',values=(0.,)*24)
            self.log.close()
            self.assertFalse(self.log._thread.is_alive())

    def test_idle_creates_no_files_and_finished_run_cannot_grow(self):
        for source,kind in [('REAL','COORDINATE_BATCH'),('SIM','SIM_MATCH')]:
            with self.subTest(source=source):
                path=self.log.start(source,{})
                for _ in range(100):self.assertFalse(self.log.emit('FRAME',values=(0.,)*24))
                self.assertFalse(path.exists())
                rid=self.log.begin_run(kind,{},{});self.log.emit('FRAME',values=(0.,)*24)
                self.log.end_run('DONE')
                self.assertIsNone(self.log.active_run)
                self.log._thread.join(timeout=2)
                self.assertFalse(self.log._thread.is_alive(),'运行结束自动关闭，不依赖断连')
                sizes={p:p.stat().st_size for p in path.rglob('*') if p.is_file()}
                for _ in range(100):self.assertFalse(self.log.emit('FRAME',values=(1.,)*24))
                self.assertEqual(sizes,{p:p.stat().st_size for p in sizes})
                self.assertTrue((path/'runs'/rid/'result.json').exists())
                self.assertEqual([e['event'] for e in self.events(path)][-2:],['RUN_END','SESSION_END'])
                new_id=self.log.begin_run(kind,{},{});new_path=self.log.path
                self.log.end_run('STOP_REQUESTED');self.log.close()
                self.assertNotEqual(rid,new_id);self.assertNotEqual(path,new_path)
                self.assertTrue((new_path/'runs'/new_id/'result.json').exists())

    def test_full_queue_still_saves_end_without_waiting_for_writer(self):
        log=RunJournal(self.tmp.name,core.CHANNELS,capacity=1)
        self.addCleanup(log.close)
        entered=threading.Event();release=threading.Event();original=log._write
        def delayed(*args):
            entered.set();release.wait(timeout=2);original(*args)
        with patch.object(log,'_write',side_effect=delayed):
            path=log.start('REAL',{});rid=log.begin_run('MANUAL',{})
            self.assertTrue(entered.wait(timeout=1))
            self.assertFalse(log.emit('FRAME',values=(0.,)*24))
            started=time.monotonic();log.end_run('STOP_REQUESTED','STOP')
            elapsed=time.monotonic()-started
            release.set();log.close()
        self.assertLess(elapsed,.2,'结束日志不得等待磁盘，影响STOP处理')
        result=json.loads((path/'runs'/rid/'result.json').read_text(encoding='utf-8'))
        self.assertEqual(result['status'],'STOP_REQUESTED')
        self.assertEqual(self.events(path)[-1]['event'],'SESSION_END')

    def test_serial_short_write_is_failure_not_success(self):
        events=[]
        worker=core.SerialWorker('NO_PORT',115200,queue.Queue(),queue.Queue(),queue.Queue(),event_cb=lambda e,**v:events.append((e,v)))
        class Serial:
            is_open=True
            def write(self,_):return 1
            def reset_output_buffer(self):pass
        worker.ser=Serial()
        self.assertFalse(worker._write_line('GOTO=1,2,3'))
        self.assertEqual(len(events),1);self.assertFalse(events[0][1]['successful'])

    def test_analysis_matches_progress_and_fields_without_crossing_paths(self):
        path=self.log.start('REAL',{});stamp=time.monotonic()
        plan={'id':7,'points':[[0,0,0,0,0,9000,5,100],[1000,0,1000,9000,1,9000,5,100]],
            'stations':{'1':['暂存区']},'display_points':[dict(s_mm=0,x_mm=0,y_mm=0,field_yaw_deg=0),dict(s_mm=100,x_mm=100,y_mm=0,field_yaw_deg=0)]}
        rid=self.log.begin_run('COORDINATE_BATCH',{'mapping':[0,0,0]},plan)
        self.log.emit('RX_TEXT',monotonic=stamp,text='TSTAT 7 4 2 0 1000 0')
        for i,yaw in enumerate([90,91,89,91,89,91,89,91,89]):
            self.log.emit('FRAME',monotonic=stamp+.01+i*.02,device_tick_ms=i*20,values=(10.,0.,float(yaw))+(0.,)*21)
        self.log.end_run('DONE');self.log.close();report,_=analyze(path/'runs'/rid)
        run=report['runs'][0];self.assertEqual(run['model_position_rms_mm'],0)
        self.assertEqual(run['model_matched_frames'],9);self.assertTrue(run['suspected_oscillation_windows'])
        with (path/'runs'/rid/'analysis.csv').open(encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
        self.assertEqual(float(rows[0]['model_yaw_error_deg']),0)
        self.assertEqual(float(rows[0]['candidate_yaw_error_deg']),0)
        self.assertEqual(rows[0]['station'],'暂存区')

    def test_waiting_station_does_not_use_next_leg_as_active_target(self):
        path=self.log.start('REAL',{});stamp=time.monotonic()
        rid=self.log.begin_run('COORDINATE_BATCH',{'mapping':[0,0,0]},
            {'id':7,'points':[[0,0,0,0,0,0,5,100],[1000,0,1000,9000,3,9000,5,100],[1000,5000,6000,0,1,0,5,100]]})
        self.log.emit('RX_TEXT',monotonic=stamp,text='TSTAT 7 5 3 1 1000 0')
        self.log.emit('FRAME',monotonic=stamp+.01,device_tick_ms=20,values=(10,0,90)+(0,)*21)
        self.log.end_run('CANCELLED');self.log.close();analyze(path)
        with (path/'runs'/rid/'analysis.csv').open(encoding='utf-8-sig') as f:row=next(csv.DictReader(f))
        self.assertEqual(row['nominal_phase'],'STOPPED');self.assertEqual(float(row['target_gap_mm']),0)
        self.assertEqual(float(row['candidate_yaw_error_deg']),0)

    def test_wrap_clock_rollover_restart_and_irregular_device_timing(self):
        def frame(tick,yaw,host):return dict(device_tick_ms=tick,monotonic=host,values=(0,0,yaw))
        dt,speed,rate=velocity(frame(0xfffffff0,179,1),frame(4,-179,2))
        self.assertAlmostEqual(dt,.02);self.assertAlmostEqual(rate,100)
        self.assertEqual(velocity(frame(1000,0,0),frame(0,0,1)),(None,None,None))
        self.assertEqual(velocity(frame(0,0,0),frame(500,0,.02)),(None,None,None))
        self.assertEqual(velocity(frame(20,0,0),frame(20,1,.02)),(None,None,None))

    def test_actual_controller_snapshot_is_separate_from_reconstructed_heading_and_expires(self):
        parser=core.FrameParser()
        text='CCTRL 7 1 13 9000 173 123 -45 -3000 -2400 0'
        parser.feed((text+'\r\n') .encode()*2)
        self.assertEqual(parser.take_text(),[text,text],'相同控制状态必须持续记录')
        path=self.log.start('REAL',{});stamp=time.monotonic()
        rid=self.log.begin_run('COORDINATE_BATCH',{'mapping':[0,0,0]},
            {'id':7,'points':[[0,0,0,0,0,0,5,100],[1000,0,1000,9000,1,0,5,100]]})
        self.log.emit('RX_TEXT',monotonic=stamp,text='TSTAT 7 4 2 0 0 0')
        self.log.emit('RX_TEXT',monotonic=stamp,text=text)
        for elapsed in (.02,.5):
            self.log.emit('FRAME',monotonic=stamp+elapsed,device_tick_ms=int(elapsed*1000),values=(0,0,0)+(0,)*21)
        self.log.end_run('DONE');self.log.close();analyze(path)
        with (path/'runs'/rid/'analysis.csv').open(encoding='utf-8-sig') as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(float(rows[0]['candidate_yaw_deg']),0,'重建值不冒充实测控制目标')
        self.assertEqual(float(rows[0]['real_controller_goal_yaw_deg']),90)
        self.assertEqual(float(rows[0]['real_controller_yaw_request_deg_s']),-30)
        self.assertEqual(float(rows[0]['real_controller_vx_mm_s']),12.3)
        self.assertEqual(rows[1]['real_controller_goal_yaw_deg'],'','过期控制快照不延用')

    def test_simulation_uses_model_time_not_fast_offline_host_time(self):
        a=dict(protocol='SIM',simulation_time_s=1,monotonic=10,values=(0,0,0))
        b=dict(protocol='SIM',simulation_time_s=1.02,monotonic=10.001,values=(1,0,1))
        dt,speed,rate=velocity(a,b)
        self.assertAlmostEqual(dt,.02);self.assertAlmostEqual(speed,500);self.assertAlmostEqual(rate,50)
        path=self.log.start('SIM',{});rid=self.log.begin_run('SIM_PATH',{'mapping':[0,0,0]},{})
        self.log.emit('FRAME',protocol='SIM',simulation_time_s=1,values=(0,0,0)+(0,)*21,
            reference=dict(x_mm=0,y_mm=100,field_yaw_deg=90,segment_index=1,segment_type='COORDINATE'))
        self.log.end_run('COMPLETE');self.log.close();report,_=analyze(path)
        with (path/'runs'/rid/'analysis.csv').open(encoding='utf-8-sig') as f:row=next(csv.DictReader(f))
        self.assertEqual(row['reference_kind'],'SIM_CONTROLLER_TARGET');self.assertEqual(float(row['candidate_yaw_error_deg']),0)
        self.assertEqual(float(row['target_gap_mm']),100);self.assertFalse(report['physical_data'])


if __name__=='__main__':unittest.main(verbosity=2)
