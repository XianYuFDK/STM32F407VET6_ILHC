"""实际坐标/旧轨迹C核心：接收暂停不失去点位、不虚增停稳时间、不复活取消批次。"""
import ctypes
import math
import unittest
import test_coordinate_batch as coord
import test_trajectory_buffer as legacy


class RecoveryHoldTests(coord.CoordinateFirmwareTests):
    def ready(self):
        batch=self.small_batch();first=batch['points'][0]
        self.e.pose=coord.Pose(first[0]/10,first[1]/10,first[3]/100,1,self.e.now)
        self.upload(batch)
        self.dll.Traj_Hold.argtypes=[ctypes.c_uint32]
        return batch

    def hold(self,n=8):
        before=self.e.status()
        for _ in range(n):
            self.e.now+=20;self.dll.Traj_Hold(self.e.now)
        after=self.e.status()
        self.assertEqual(before[1:5],after[1:5])

    def finish(self):
        for _ in range(1500):
            self.e.step(move=True)
            status=self.e.status()
            self.assertNotIn(status[1],(7,8),status)
            if status[1]==6:return
        self.fail('恢复后没有完成')

    def test_hold_during_motion_preserves_point_and_resumes(self):
        self.ready()
        for _ in range(20):self.e.step(move=True)
        self.assertEqual(self.e.status()[1],4)
        self.hold();self.finish()

    def test_hold_clears_final_stop_settle_window(self):
        batch=self.ready();final=batch['points'][-1]
        for _ in range(1500):
            self.e.step(move=True)
            if math.dist((self.e.pose.x,self.e.pose.y),(final[0]/10,final[1]/10))<.5:break
        else:self.fail('没有接近终点')
        self.e.pose.x,self.e.pose.y=final[0]/10,final[1]/10
        self.e.pose.yaw=final[3]/100
        for _ in range(4):self.e.step()
        self.assertEqual(self.e.status()[1],4)
        self.hold(n=5)
        for _ in range(8):
            self.e.step();self.assertEqual(self.e.status()[1],4,'暂停前时间不能凑成200ms')
        self.finish()

    def test_hold_cannot_restore_canceled_batch(self):
        self.ready();self.e.step(move=True)
        self.dll.Traj_Cancel(17);self.hold()
        self.assertFalse(self.dll.Traj_OutputAllowed())
        self.e.step();self.assertEqual(self.e.status()[1],7)

    def test_legacy_dense_trajectory_resumes_after_hold(self):
        self.e=legacy.Engine(self.dll)
        self.assertEqual(self.e.upload(legacy.straight())[1],3)
        self.e.send('TRUN=17');self.e.step()
        for _ in range(10):self.e.step(move=True)
        self.hold();self.finish()


if __name__=='__main__':
    names=[n for n in RecoveryHoldTests.__dict__ if n.startswith('test_')]
    result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(RecoveryHoldTests(n) for n in names))
    raise SystemExit(not result.wasSuccessful())
