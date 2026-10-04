"""真实C轨迹参数：调速、独立纠偏、转头限速和执行期间写保护。"""
import ctypes
import math
import unittest
import test_trajectory_buffer as native

class Settings(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in
               ('speed_mm_s','kp_x','kp_y','kp_yaw','yaw_rate_deg_s',
                'rotate_rate_deg_s','rotate_acc_deg_s2','hold_speed_mm_s','lookahead_mm','accel_mm_s2','decel_mm_s2','arc_speed_mm_s')]

class TrajectorySettingsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        native.TrajectoryFirmwareTests.setUpClass.__func__(cls)
        cls.dll.Traj_SetControlParam.argtypes = [ctypes.POINTER(ctypes.c_float),ctypes.c_float,ctypes.c_float,ctypes.c_float]
        cls.dll.Traj_SetControlParam.restype = ctypes.c_uint8

    @classmethod
    def tearDownClass(cls):
        native.TrajectoryFirmwareTests.tearDownClass.__func__(cls)

    def setUp(self):
        self.e = native.Engine(self.dll)
        self.params = Settings.in_dll(self.dll, 'traj_control_params')

    def setting(self, name, value, low=0.1, high=1000):
        address = ctypes.addressof(self.params) + getattr(Settings, name).offset
        return self.dll.Traj_SetControlParam(ctypes.cast(address,ctypes.POINTER(ctypes.c_float)),value,low,high)

    def start(self, points=None):
        self.assertEqual(self.e.upload(points or native.straight())[1],3)
        self.e.send('TRUN=17'); self.e.step()

    def test_translation_speed_changes_real_control_output(self):
        for speed in (80,250,600):
            with self.subTest(speed=speed):
                self.setUp(); self.assertEqual(self.setting('speed_mm_s',speed),1)
                self.start()
                peak=0
                for _ in range(600):
                    self.e.step(move=True);peak=max(peak,self.e.velocity[1])
                    if abs(peak-speed)<.01: break
                self.assertAlmostEqual(peak,speed,places=2)
                self.assertEqual(self.e.status()[1],4)

    def test_x_y_and_heading_gains_are_independent(self):
        self.setting('kp_x',2); self.setting('kp_y',3); self.setting('kp_yaw',4)
        self.start(); self.e.pose.x=1; self.e.pose.yaw=.5; self.e.step()
        self.assertAlmostEqual(self.e.velocity[0],-2,places=3)
        self.assertAlmostEqual(self.e.velocity[2],-2,places=3)
        self.setUp(); self.setting('kp_x',2); self.setting('kp_y',3)
        self.start([(q[1],q[0],*q[2:]) for q in native.straight()])
        self.e.pose.y=1; self.e.step()
        self.assertAlmostEqual(self.e.velocity[1],-3,places=3)

    def test_moving_heading_rate_cap_applies(self):
        self.setting('kp_yaw',30); self.setting('yaw_rate_deg_s',5)
        self.start(); self.e.pose.yaw=1; self.e.step()
        self.assertAlmostEqual(self.e.velocity[2],-5,places=3)

    def test_rotation_acceleration_speed_and_hold_limits(self):
        self.setting('rotate_rate_deg_s',12); self.setting('rotate_acc_deg_s2',30)
        self.setting('hold_speed_mm_s',5); self.setting('kp_x',12)
        self.start([(0,0,0,0,native.STOP),(0,0,0,9000,native.STOP|native.ROTATE)])
        for _ in range(10): self.e.step()
        self.e.pose.x=1
        self.e.step(); self.assertAlmostEqual(self.e.velocity[0],-5,places=3)
        self.assertAlmostEqual(self.e.velocity[2],.6,places=3)
        for _ in range(30): self.e.step()
        self.assertAlmostEqual(self.e.velocity[2],12,places=3)

    def test_nonfinite_range_and_busy_writes_do_not_change_values(self):
        for value in (float('nan'),float('inf'),-1,1001):
            self.assertEqual(self.setting('speed_mm_s',value,20,1000),3)
            self.assertEqual(self.params.speed_mm_s,500)
        self.e.send('TBEGIN=17,51,1,00000000,10')
        self.assertEqual(self.setting('speed_mm_s',80,20,1000),2)
        self.e.dll.Traj_Cancel(15); self.e.step()
        self.assertEqual(self.setting('speed_mm_s',80,20,1000),1)
        self.setUp(); self.start()
        for name,_ in Settings._fields_:
            self.assertEqual(self.setting(name,10),2)
        self.assertEqual(self.params.speed_mm_s,500)

    def test_ready_and_waiting_batches_also_reject_mutation(self):
        self.e.upload(native.straight())
        self.assertEqual(self.setting('speed_mm_s',80),2)
        self.setUp(); self.start(native.straight(wait=True))
        for _ in range(300):
            self.e.step(move=True)
            if self.e.status()[1]==5: break
        self.assertEqual(self.e.status()[1],5)
        self.assertEqual(self.setting('kp_yaw',3),2)

    def test_lower_speed_does_not_relax_pose_safety(self):
        self.setting('speed_mm_s',80)
        self.start(); self.e.pose.x=9.1; self.e.step()
        self.assertEqual(self.e.status()[-1],13)
        self.assertFalse(self.dll.Traj_OutputAllowed())

    def test_init_restores_defaults(self):
        self.setting('speed_mm_s',80); self.setting('kp_x',3)
        self.dll.Traj_Init()
        self.assertEqual(list(getattr(self.params,n) for n,_ in Settings._fields_),
                         [500,6,6,6,120,60,180,60,100,600,1000,150])

if __name__=='__main__': unittest.main(verbosity=2)
