"""真实C高速档：加速、入弯制动、圆弧限速、停靠与偏差/STOP故障注入。"""
import math
import unittest
import test_trajectory_settings as settings
import test_trajectory_buffer as native
from hardware_trajectory import ARC

class TrajectorySpeedTests(settings.TrajectorySettingsTests):
    # 使用同一真实C编译与参数绑定，只在本模块运行提速专项目标。
    def test_straight_500_reaches_cruise_and_finishes_faster_than_250(self):
        results=[]
        for cruise in (250,500):
            self.setUp();self.setting('speed_mm_s',cruise)
            points=[(0,i*200,i*200,0,native.STOP if i==100 else 0) for i in range(101)]
            self.start(points);begin=self.e.now;peak=0;last=0
            for _ in range(2000):
                action=self.e.step(move=True);status=self.e.status()
                command=self.e.velocity[1] if action==1 else 0
                peak=max(peak,command)
                if command>last:self.assertLessEqual(command-last,600*.02+.02)
                last=command
                if status[1] in (6,8):break
            self.assertEqual(status[1],6,status)
            self.assertAlmostEqual(peak,cruise,places=2)
            self.assertLess(abs(self.e.pose.y-2000),1)
            results.append((cruise,(self.e.now-begin)/1000,peak))
        self.assertLess(results[1][1],results[0][1]-.5)
        print('2m straight real C (cruise,time,peak):',results)

    def curved(self):
        # 800mm直线 + 半径120mm左转90度 + 600mm直线，航向跟随切线。
        points=[(0,i*200,i*200,0,0) for i in range(41)]
        radius=120
        for degree in range(3,91,3):
            a=math.radians(degree)
            points.append((round(radius*(1-math.cos(a))*10),round((800+radius*math.sin(a))*10),
                           round((800+radius*a)*10),degree*100,ARC))
        for i in range(1,31):
            points.append((1200+i*200,9200,round((800+radius*math.pi/2+i*20)*10),9000,
                           native.STOP if i==30 else 0))
        return points

    def test_500_brakes_before_arc_and_arc_output_stays_capped(self):
        self.start(self.curved());peak=0;before=[];arc_speeds=[]
        for _ in range(3000):
            # Engine积分在step返回后立即执行，保存本次计算前的实际位置。
            pose_y=self.e.pose.y;pose_x=self.e.pose.x
            action=self.e.step(move=True);status=self.e.status()
            mag=math.hypot(*self.e.velocity[:2]) if action==1 else 0
            peak=max(peak,mag)
            if pose_x<.1 and 650<pose_y<795:before.append((pose_y,mag))
            if status[4]/10>805 and status[4]/10<980:arc_speeds.append(mag)
            if status[1] in (6,8):break
        self.assertEqual(status[1],6,status)
        self.assertGreater(peak,490)
        self.assertTrue(before);self.assertLess(before[-1][1],210)
        self.assertTrue(arc_speeds);self.assertLessEqual(max(arc_speeds),150.01)
        self.assertLess(abs(self.e.pose.y-920),1)
        self.assertLess(abs(self.e.pose.x-720),1)

    def test_arc_cap_is_independent_of_body_yaw(self):
        points=self.curved()
        points=[(*q[:3],0,q[4]) for q in points]
        self.start(points);arc_peak=0
        for _ in range(3000):
            self.e.step(move=True);status=self.e.status()
            if 805<status[4]/10<980:arc_peak=max(arc_peak,math.hypot(*self.e.velocity[:2]))
            if status[1] in (6,8):break
        self.assertEqual(status[1],6,status)
        self.assertGreater(arc_peak,140);self.assertLessEqual(arc_peak,150.01)

    def test_accel_setting_controls_startup_ramp(self):
        for accel in (100,600,1200):
            self.setUp();self.setting('accel_mm_s2',accel,50,3000)
            self.start();self.e.step()
            self.assertAlmostEqual(self.e.velocity[1],accel*.02,places=3)

    def test_decel_setting_moves_braking_before_arc(self):
        speeds=[]
        for decel in (200,1000):
            self.setUp();self.setting('decel_mm_s2',decel,100,4000)
            self.start(self.curved())
            for _ in range(1000):
                self.e.step(move=True)
                if self.e.pose.y>=650:break
            self.assertEqual(self.e.status()[1],4)
            speeds.append(math.hypot(*self.e.velocity[:2]))
        self.assertLess(speeds[0],speeds[1]-80)

    def test_near_limit_deviation_slows_then_outside_margin_stops(self):
        self.start()
        for _ in range(35):self.e.step(move=True)
        normal=self.e.velocity[1]
        self.e.pose.x=6
        self.e.step()
        self.assertEqual(self.e.status()[1],4)
        self.assertLess(self.e.velocity[1],normal*.5)
        self.e.pose.x=9.1;self.e.step()
        self.assertEqual(self.e.status()[-1],13)
        self.assertEqual(list(self.e.velocity),[0,0,0])

    def test_stop_cancels_at_500_and_new_batch_starts_from_zero(self):
        self.start()
        for _ in range(60):self.e.step(move=True)
        self.assertGreater(self.e.velocity[1],490)
        self.dll.Traj_Cancel(15);self.assertFalse(self.dll.Traj_OutputAllowed())
        self.assertEqual(self.e.step(),2)
        self.assertEqual(list(self.e.velocity),[0,0,0])
        self.e.pose.x=self.e.pose.y=self.e.pose.yaw=0
        self.start();self.e.step()
        self.assertAlmostEqual(self.e.velocity[1],12,places=3)

if __name__=='__main__':unittest.main(verbosity=2)
