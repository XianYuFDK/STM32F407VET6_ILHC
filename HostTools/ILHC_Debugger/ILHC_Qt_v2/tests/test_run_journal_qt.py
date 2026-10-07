"""真实Qt运行入口与后台日志；不启动串口或模拟运动线程。"""
import argparse
import json
import os
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import main
import core


class JournalQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=main.QApplication.instance() or main.QApplication([])
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.env=patch.dict(os.environ,ILHC_RUN_LOG_DIR=self.tmp.name);self.env.start()
        self.w=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
        self.w.sim=core.Simulator(self.w.frame_q,self.w.line_q,self.w.urgent_q)
    def tearDown(self):
        self.w.close();self.w._planner_pool.shutdown(wait=True,cancel_futures=True)
        self.env.stop();self.tmp.cleanup()

    def test_batch_initial_stop_does_not_end_run_and_pause_keeps_recording(self):
        self.w._journal_begin('COORDINATE_BATCH',{'chassis_control':core.CHASSIS_DEFAULTS})
        rid=self.w.run_journal.active_run
        self.w.sim.handle_line('STOP')
        self.assertEqual(self.w.run_journal.active_run,rid)
        self.w.sim.make_frame(0)
        self.w.run_journal.end_run('DONE')
        self.w._process_frames()
        self.w.run_journal.close()
        path=self.w.run_journal.path
        events=[json.loads(v) for v in (path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertTrue(any(e['event']=='FRAME' and e['run_id']==rid for e in events))
        self.assertFalse(json.loads((path/'session.json').read_text(encoding='utf-8'))['physical_data'])
        self.assertIn(str(path),self.w.run_log_status.text())

    def test_manual_command_starts_run_and_zero_ends_it(self):
        self.w._ensure_run_journal();self.w.sim.handle_line('MANUAL=1,0,0')
        self.assertIsNotNone(self.w.run_journal.active_run)
        self.w.sim.handle_line('MANUAL=0,0,0');self.assertIsNone(self.w.run_journal.active_run)

    def test_user_stop_ends_real_and_sim_logs_without_disconnect_and_restart_is_new(self):
        sim=self.w.sim
        for source in ('REAL','SIM'):
            with self.subTest(source=source):
                self.w.run_journal.close();self.w._journal_source=None
                worker=core.SerialWorker('NO_PORT',115200,self.w.frame_q,self.w.line_q,self.w.urgent_q)
                self.w.worker=worker if source=='REAL' else None
                self.w.sim=None if source=='REAL' else sim
                self.w._ensure_run_journal()
                self.assertFalse(self.w.run_journal.path.exists())
                self.w._journal_begin('COORDINATE_BATCH' if source=='REAL' else 'SIM_MATCH',{})
                rid=self.w.run_journal.active_run;path=self.w.run_journal.path
                self.w.send_line('STOP')
                self.assertIsNone(self.w.run_journal.active_run)
                self.w.run_journal._thread.join(timeout=2)
                self.assertFalse(self.w.run_journal._thread.is_alive())
                source_obj=worker if source=='REAL' else sim
                source_obj.event_cb('FRAME',values=(0.,)*24)
                self.w._journal_begin('COORDINATE_BATCH' if source=='REAL' else 'SIM_MATCH',{})
                self.assertNotEqual(self.w.run_journal.active_run,rid)
                self.assertNotEqual(self.w.run_journal.path,path)
                self.w.run_journal.end_run('DONE');self.w.run_journal.close()
                events=[json.loads(v) for v in (path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
                self.assertEqual(events[-1]['event'],'SESSION_END')
                self.assertFalse(any(e['event']=='FRAME' for e in events))
                self.w.worker=None;self.w.sim=sim

    def test_zero_manual_while_idle_does_not_create_a_run(self):
        self.w.send_line('MANUAL=0,0,0')
        self.w.sim.handle_line('MANUAL=0,0,0')
        self.assertIsNone(self.w.run_journal.active_run)
        self.assertFalse(self.w.run_journal.path.exists())

    def test_firmware_ops_details_are_saved_before_cancel_closes_log(self):
        self.w._journal_begin('COORDINATE_BATCH',{'id':17,'points':[]})
        path=self.w.run_journal.path
        for text in ['OPSE 7 512 100 3 3 80 80 9','OPSX 7 11 0 20 0 8 2 1','TSTAT 17 7 4 1 2193 17']:
            self.w.sim.event_cb('RX_TEXT',text=text)
        self.w.run_journal.end_run('CANCELLED','STM32 CANCELLED：OPS会话变化')
        self.w.run_journal.close()
        events=[json.loads(x) for x in (path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        details=[e for e in events if e['event']=='OPS_DIAGNOSTIC']
        self.assertEqual(len(details),2)
        self.assertEqual(details[0]['causes'],['USART2硬件错误'])
        self.assertEqual(details[1]['uart_error'],8)
        self.assertLess(details[-1]['sequence'],next(e['sequence'] for e in events if e['event']=='RUN_END'))


if __name__=='__main__':unittest.main(verbosity=2)
