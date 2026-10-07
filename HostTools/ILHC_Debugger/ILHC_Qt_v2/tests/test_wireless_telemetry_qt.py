"""DL-20选择、慢通道显示与DM新鲜度，离屏且不连接设备。"""
import argparse
import os
import time
import unittest
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import core
import main
from tests.test_wireless_telemetry import packet


class WirelessQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.window=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))

    def tearDown(self):
        self.window.worker=None;self.window.close()

    def test_bluetooth_default_and_dl20_independent_slow_state_are_visible(self):
        w=self.window
        self.assertEqual(w.link_mode_combo.currentData(),'wired')
        self.assertEqual(w.link_mode_combo.itemData(1),'wired')
        worker=core.SerialWorker('TEST',115200,w.frame_q,w.line_q,link_mode='dl20')
        worker.opened.set();w.worker=worker
        values=[0.]*24;values[12]=w.dm_rows['DMID'].spin.value()
        worker._receive(packet(0,0,values)+packet(1,20,(1.,2.,3.)),time.monotonic())
        w._process_frames();w._render_ui();w._status_tick()
        self.assertIn('CRC2',w.footer_link.text());self.assertIn('5Hz',w.footer_link.text())
        self.assertEqual(w.latest[:3],(1.,2.,3.))
        self.assertEqual(w.dm_position_panel.snapshot()[0][12],values[12])

    def test_recent_pose_cannot_refresh_stale_dm_feedback(self):
        w=self.window;worker=core.SerialWorker('TEST',115200,w.frame_q,w.line_q,link_mode='dl20')
        worker.opened.set();w.worker=worker
        values=[0.]*24;values[12]=w.dm_rows['DMID'].spin.value()
        worker._receive(packet(0,0,values),time.monotonic()-.4)
        worker._receive(packet(1,400,(1.,2.,3.)),time.monotonic())
        w._process_frames()
        self.assertLess(time.monotonic()-w.latest_received_monotonic,.1)
        with self.assertRaisesRegex(ValueError,'超时'):w.dm_position_panel.snapshot()


if __name__=='__main__':unittest.main()
