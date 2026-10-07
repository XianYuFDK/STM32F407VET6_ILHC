"""Pause/resume against the actual C engine, no serial connection."""
import ctypes
import unittest
import test_coordinate_batch as support
from test_trajectory_buffer import Pose

class UserPauseTests(unittest.TestCase):
    setUpClass = classmethod(support.CoordinateFirmwareTests.setUpClass.__func__)
    tearDownClass = classmethod(support.CoordinateFirmwareTests.tearDownClass.__func__)
    setUp = support.CoordinateFirmwareTests.setUp
    sync = support.CoordinateFirmwareTests.sync
    upload = support.CoordinateFirmwareTests.upload
    small_batch = support.CoordinateFirmwareTests.small_batch

    def start_run(self):
        batch = self.small_batch()
        p = batch['points'][0]
        self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
        self.upload(batch)
        for _ in range(25): self.advance()
        self.assertEqual(self.job.state,'RUNNING',self.job.reason)
        return batch

    def advance(self, move=True):
        action=self.e.step(move=move)
        while True:
            text=self.e.reply()
            if not text:break
            self.job.handle_reply(text)
        self.job.tick()
        return action

    def test_pause_longer_than_deadline_resume_same_batch_then_complete(self):
        batch=self.start_run()
        self.job.pause()
        self.assertFalse(self.dll.Traj_OutputAllowed())
        self.advance(False)
        self.assertEqual(self.job.state,'PAUSED',self.job.reason)
        self.assertTrue(self.dll.Traj_Busy())
        cursor=self.job.cursor;pose=(self.e.pose.x,self.e.pose.y,self.e.pose.yaw)
        for _ in range(9500):
            self.assertEqual(self.advance(False),2)
            self.assertEqual(tuple(self.e.velocity),(0,0,0))
        self.assertEqual(self.job.state,'PAUSED',self.job.reason)
        self.assertEqual(self.job.cursor,cursor)
        self.assertEqual(pose,(self.e.pose.x,self.e.pose.y,self.e.pose.yaw))
        self.job.continue_run();self.advance(False)
        self.assertEqual(self.job.state,'RUNNING',self.job.reason)
        for _ in range(5000):
            self.advance()
            if not self.job.active:break
        self.assertEqual(self.job.state,'DONE',self.job.reason)
        self.assertEqual(sum(c.startswith('TRUN=') for c in self.job.commands),1)
        self.assertEqual(sum(c.startswith('CPOINT=') for c in self.job.commands),len(batch['points']))

    def test_pause_keeps_stale_pose_host_and_jump_protection(self):
        for fault in ('stale','host','jump'):
            with self.subTest(fault=fault):
                self.setUp();self.start_run();self.job.pause();self.advance(False)
                if fault=='stale':
                    self.e.now+=201
                    self.dll.Traj_Step(self.e.now,ctypes.byref(self.e.pose),1,self.e.velocity)
                elif fault=='host':self.e.step(allowed=0)
                else:
                    self.e.pose.x+=51;self.e.step()
                while True:
                    text=self.e.reply()
                    if not text:break
                    self.job.handle_reply(text)
                self.assertEqual(self.job.state,'CANCELLED')
                self.assertFalse(self.dll.Traj_OutputAllowed())

    def test_stop_cannot_be_resumed_and_old_firmware_is_rejected(self):
        self.start_run()
        self.job.capabilities=8
        with self.assertRaisesRegex(ValueError,'CCAPS9'):self.job.pause()
        self.job.capabilities=9;self.job.pause();self.advance(False)
        self.job.cancel();self.e.step();self.e.send('TCONTINUE=17');self.e.step()
        self.assertFalse(self.dll.Traj_Busy())
        self.assertFalse(self.dll.Traj_OutputAllowed())

    def test_pause_ack_timeout_stops_even_if_running_status_arrives(self):
        self.start_run()
        self.job.send=lambda _:None
        self.job.pause()
        for _ in range(155):self.advance(False)
        self.assertEqual(self.job.state,'CANCELLED')
        self.assertIn('未确认',self.job.reason)

if __name__=='__main__':unittest.main()
