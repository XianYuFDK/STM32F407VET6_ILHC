"""真实日志暴露的出口反向追逐、阈值切换及毫米级sqrt制动回归。"""
import math
import unittest

import core
import coordinate_navigation as coordinate
import competition_simulation as competition
from tests.test_pivot_turns import CORNERS, baseline
import pivot_turns


class RealControlFixTests(unittest.TestCase):
    def pivot_tracker(self):
        scene=competition.collision_scene(competition.load_profile())
        program,_=pivot_turns.corner_trial(baseline(CORNERS[0],scene)['waypoint_program'],
            1,pivot_turns.wheel_geometry(scene),scene,lambda:False)
        return coordinate.CoordinateTracker(program)

    def test_exit_heading_latches_when_distance_jitters_across_lead(self):
        program=coordinate.build_program([(500,500),(500,1500)],90,goal_yaw=180,
            mode='FIXED',control=dict(turn_lead_mm=40))
        program['waypoints'][1]['travel_yaw_deg']=90
        tracker=coordinate.CoordinateTracker(program)
        target=tracker.points[1]
        refs=[]
        for distance in (41,39,40.1,39.8,42):
            ref,*_=tracker.command((target['x_mm']-distance,target['y_mm'],target['travel_yaw_deg']),None,None)
            refs.append(ref['field_yaw_deg'])
        self.assertEqual(refs[0],target['travel_yaw_deg'])
        self.assertEqual(refs[1:],[target['field_yaw_deg']]*4)

    def test_missed_pivot_exit_does_not_keep_nonzero_rotation_at_zero_error(self):
        tracker=self.pivot_tracker();tracker.index=2
        end=tracker.points[2]
        _,velocity,omega=tracker.command((end['x_mm']+10,end['y_mm'],end['field_yaw_deg']),None,None)
        self.assertEqual(tracker.index,2,'不能放宽位置门或跳过点')
        self.assertTrue(tracker._pivot_recover)
        self.assertAlmostEqual(omega,0,places=5)
        self.assertGreater(math.hypot(*velocity),0,'继续位置收敛')
        tracker._previous_pose_yaw=None
        _,_,omega=tracker.command((end['x_mm']+10,end['y_mm'],end['field_yaw_deg']-2),None,None)
        self.assertLess(abs(omega),8,'恢复阶段不能以30.56deg/s衔接下限反向追角')
        self.assertTrue(tracker._pivot_recover,'恢复状态不能因再次离开1deg区域而退出')

    def test_terminal_wheel_straight_uses_low_speed_position_convergence(self):
        tracker=self.pivot_tracker();tracker.index=3
        previous,end=tracker.points[2:4]
        dx,dy=end['x_mm']-previous['x_mm'],end['y_mm']-previous['y_mm']
        span=math.hypot(dx,dy);tx,ty=dx/span,dy/span
        tracker._command_velocity=(100*tx,100*ty)
        pose=(end['x_mm']-1.7*tx,end['y_mm']-1.7*ty,end['field_yaw_deg'])
        _,velocity,_=tracker.command(pose,None,None)
        # 有限加速度会先减速；持续给相同小偏差后应收敛到P速度，而非sqrt速度。
        for _ in range(50):
            _,velocity,_=tracker.command(pose,None,None)
        self.assertLess(math.hypot(*velocity),6)
        self.assertEqual(tracker.index,3)

    def test_motor_lag_and_pose_delay_finish_without_repeated_pivot_reversals(self):
        from Docs.audit_real_control_response_20261006 import simulate,previous_controller
        scene=competition.collision_scene(competition.load_profile(),9)
        program,_=pivot_turns.corner_trial(baseline(CORNERS[0],scene,
            dict(kpx=1.5,kpy=1.5,kpz=9.05,xyvmax=930,zvmax=750,xyvmin=.5,zvmin=.5))['waypoint_program'],
            1,pivot_turns.wheel_geometry(scene),scene,lambda:False)
        for tau,delay in ((.04,.02),(.08,.04),(.12,.04),(.16,.06)):
            with self.subTest(tau=tau,delay=delay):
                result=simulate(program,scene,coordinate.CoordinateTracker,tau,delay)
                self.assertTrue(result['done'],{k:v for k,v in result.items() if k!='trace'})
                self.assertIsNone(result['unsafe'])
                self.assertLessEqual(result['pivot_rate_reversals'],1)
        before=simulate(program,scene,previous_controller(),.08,.04)
        self.assertGreater(before['pivot_rate_reversals'],3,'必须复现旧非零衔接项造成的重复反向')


if __name__=='__main__':
    unittest.main()
