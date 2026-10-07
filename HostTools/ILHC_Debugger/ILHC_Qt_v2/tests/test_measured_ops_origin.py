"""Measured departure zero: actual Qt entry points, simulator reset and stale-frame rejection."""
import argparse
import copy
import os
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import core
import main
from competition_simulation import departure_reference


class MeasuredOpsOriginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.w = main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
        self.w.sim = core.Simulator(self.w.frame_q,self.w.line_q,self.w.urgent_q)
        self.w.sim.handle_line('ZERO')
        self.w.sim.make_frame(0)

    def tearDown(self):
        self.w.close()
        self.w._planner_pool.shutdown(wait=True,cancel_futures=True)

    def zero(self, button=False):
        if button:
            self.w._set_start_zone()
        else:
            self.w.send_line('ZERO')
        pending = self.w._ops_zero_pending
        self.assertIsNotNone(pending)
        return pending

    def test_zone_selection_never_offsets_measured_zero_or_home(self):
        stations=copy.deepcopy(self.w.nav_map['competition']['stations'])
        self.w.zone_combo.setCurrentIndex(1)
        self.w.map_ox,self.w.map_oy,self.w.map_theta=2100,15,90
        self.zero(button=True)
        self.assertEqual((self.w.map_ox,self.w.map_oy,self.w.map_theta),(0,0,0))
        self.assertEqual(core.layout_to_field(*self.w._home_layout()),(0,0))
        self.assertEqual(self.w.nav_map['competition']['stations'],stations)
        for station,xy in (('raw',(100,1050)),('rough',(1900,1050)),('storage',(1100,1900))):
            self.assertEqual(self.w._field_to_ops(*stations[station]),xy)
        self.assertFalse(self.w.zone_combo.isEnabled())
        self.assertFalse(self.w.map_apply_button.isEnabled())

    def test_actual_simulator_zero_is_followed_by_three_fresh_zero_frames(self):
        self.w.sim.hold=(300,420)
        self.w.sim.zval=37
        self.zero()
        self.w.sim._service_commands()
        self.assertIsNotNone(self.w._ops_zero_pending['sent'])
        for index in range(3):
            values=self.w.sim.make_frame((index+1)*.02)
            self.w.frame_q.put((self.w.sim.last_frame_monotonic,values))
            self.w._process_frames()
        self.assertIsNone(self.w._ops_zero_pending)
        self.assertEqual(self.w._ops_zero_result['state'],'SIM_CONFIRMED')
        self.assertEqual(self.w.sim.navigation_snapshot()['hold'],(0,0))
        self.assertEqual(self.w.sim.navigation_snapshot()['yaw'],0)
        self.assertFalse(self.w._ops_zero_block_reason())

    def test_unsent_old_duplicate_and_nonzero_frames_cannot_confirm(self):
        p=self.zero()
        p['simulated']=False  # Exercise real telemetry path without opening a serial port.
        now=time.monotonic()
        for i in range(4): self.w._poll_ops_zero(now+i*.02,[0]*24)
        self.assertEqual(p['samples'],0)
        self.w._journal_wire_event(self.w.sim,'TX',command='ZERO',successful=False)
        self.assertIsNone(p['sent'])
        self.w._journal_wire_event(self.w.sim,'SIM_COMMAND',command='ZERO',successful=True)
        stamp=p['sent']
        for _ in range(5):self.w._poll_ops_zero(stamp,[0]*24)
        self.assertEqual(p['samples'],0)
        self.w._poll_ops_zero(stamp+.01,[0]*24)
        self.w._poll_ops_zero(stamp+.01,[0]*24)
        self.assertEqual(p['samples'],1)
        self.w._poll_ops_zero(stamp+.02,[2,0,0]+[0]*21)
        self.assertEqual(p['samples'],0)
        self.assertIsNotNone(self.w._ops_zero_pending)
        for offset in (.03,.05,.07):self.w._poll_ops_zero(stamp+offset,[0]*24)
        self.assertIsNone(self.w._ops_zero_pending)
        self.assertEqual(self.w._ops_zero_result['state'],'TELEMETRY_CONFIRMED')

    def test_mount_offset_reset_also_synchronizes_map_and_requires_feedback(self):
        self.w.map_ox,self.w.map_oy,self.w.map_theta=100,200,90
        self.w.send_line('OPSOFFSET=53,-39.5')
        self.assertEqual((self.w.map_ox,self.w.map_oy,self.w.map_theta),(0,0,0))
        self.assertEqual(self.w._ops_zero_pending['command'],'OPSOFFSET=53,-39.5')
        self.w.sim._service_commands()
        for index in range(3):
            values=self.w.sim.make_frame(index*.02)
            self.w._poll_ops_zero(self.w.sim.last_frame_monotonic,values)
        self.assertIsNone(self.w._ops_zero_pending)

    def test_mapping_controls_cannot_override_measured_station_frame(self):
        self.w.map_ox_spin.setValue(210)
        self.w.map_theta_spin.setValue(90)
        self.w._apply_map_mapping()
        self.assertEqual((self.w.map_ox,self.w.map_oy,self.w.map_theta),(0,0,0))

    def test_wait_and_timeout_block_motion_and_upload_then_explicit_retry(self):
        p=self.zero()
        with patch.object(self.w._planner_pool,'submit') as submit:
            self.w._goto_home()
            self.w._start_competition()
            self.w._start_real_match()
            submit.assert_not_called()
        self.w.send_line('GOTO=10,20,0')
        self.w.send_line('MANUAL=100,0,0')
        self.assertFalse(any(str(c).startswith(('GOTO=','MANUAL=100')) for c in self.w.line_q.queue))
        with self.assertRaisesRegex(ValueError,'置零'):self.w._plan_preconditions()
        p['requested']-=4
        self.w._poll_ops_zero()
        self.assertIsNone(self.w._ops_zero_pending)
        self.assertIn('置零未确认',self.w._ops_zero_block_reason())
        self.zero()
        self.assertFalse(self.w._ops_zero_failure)

    def test_zero_does_not_enable_disabled_wheels(self):
        self.w.sim.handle_line('WHEELOFF')
        self.zero()
        self.w.sim._service_commands()
        self.assertFalse(self.w.sim.navigation_snapshot()['wheel_enabled'])

    def test_new_source_cannot_complete_old_reset(self):
        p=self.zero()
        self.w.sim=core.Simulator(self.w.frame_q,self.w.line_q,self.w.urgent_q)
        self.w._poll_ops_zero(time.monotonic(),[0]*24)
        self.assertIsNone(self.w._ops_zero_pending)
        self.assertIn('连接变化',self.w._ops_zero_failure)
        self.assertIsNone(p['sent'])

    def test_home_button_and_match_reference_are_independent_of_zone_center(self):
        self.w.zone_combo.setCurrentIndex(1)
        self.w.nav_map['competition']['start_zones']['1']=[2250,2100]
        self.w.nav_map['competition']['start_zones']['2']=[2000,300]
        for zone in (1,2):
            home,staging=departure_reference(self.w.nav_map,zone)
            self.assertEqual(core.layout_to_field(*home),(0,0))
            self.assertEqual(core.layout_to_field(*staging),(150,150))
        with patch.object(self.w,'_request_plan') as request:
            self.w._goto_home()
            self.assertEqual(core.layout_to_field(*request.call_args.args),(0,0))
        self.w.nav_map.pop('coordinate_reference')
        self.assertEqual(departure_reference(self.w.nav_map,2)[0],(2000,300))

    def test_offline_zero_cannot_change_mapping_or_fake_confirmation(self):
        self.w.sim=None
        self.w.map_ox=123
        self.w.send_line('ZERO')
        self.assertEqual(self.w.map_ox,123)
        self.assertIsNone(getattr(self.w,'_ops_zero_pending',None))


if __name__=='__main__':
    unittest.main(verbosity=2)
