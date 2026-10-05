"""关键坐标闭环：车体轴变换/四轮叠加、通过门、完整扫掠、取消及比赛。"""
import copy
import math
import queue
import threading
import unittest
from unittest.mock import patch

import core
import competition_simulation as competition
from coordinate_navigation import build_program, CoordinateTracker, replay, wrap, plan_route
from navigation_planner import CollisionScene
from tests.test_competition_simulation import runner_for, advance


def fixture():
    scene = CollisionScene([], [], (0,0,3000,3000), 10, (280,260,0), None)
    program = build_program([(500,500),(1500,500),(1500,1500)], 0, control={'turn_lead_mm':60})
    return program, scene


class CoordinateTests(unittest.TestCase):
    def test_sub_rpm_commands_accumulate_and_stop_discards_residual(self):
        program=build_program([(1000,1000),(1300,1300)],-180)
        tracker=CoordinateTracker(program);goal=tracker.final_reference()
        pose=(goal['x_mm']-1,goal['y_mm']-1,goal['field_yaw_deg'])
        velocities=[]
        for _ in range(1000):
            _ref,velocity,_omega=tracker.command(pose,0,0)
            velocities.append(velocity)
            for w in tracker.wheel_velocity_mm_s:
                self.assertAlmostEqual(w*.238,round(w*.238))
        for i in (0,1):
            self.assertAlmostEqual(sum(v[i] for v in velocities)/len(velocities),2.3,delta=.005)
        for _ in range(5):
            _ref,velocity,omega=tracker.command((goal['x_mm'],goal['y_mm'],goal['field_yaw_deg']),0,0)
            self.assertEqual(velocity,(0,0));self.assertEqual(omega,0)
            self.assertEqual(tracker._rpm_remainder,[0]*4)

    def test_body_error_and_four_wheel_mixing_translate_and_turn_together(self):
        program, _scene = fixture()
        tracker = CoordinateTracker(program)
        first = tracker.reference_at(0)
        # 世界误差沿+/-轴，改变实际车头仍得到正确世界平移，同时有角速度。
        pose = first['x_mm'], first['y_mm'], first['field_yaw_deg']+45
        _ref, velocity, omega = tracker.command(pose, 250, 120)
        target = tracker.points[1]
        error = target['x_mm']-pose[0],target['y_mm']-pose[1]
        self.assertGreater(sum(a*b for a,b in zip(error,velocity)), 0)
        self.assertGreater(math.hypot(*velocity), 0)
        self.assertLess(omega, 0)
        self.assertEqual(len(tracker.wheel_velocity_mm_s), 4)
        self.assertGreater(len(set(tracker.wheel_velocity_mm_s)), 1)

    def test_shortest_angle_both_directions_across_180(self):
        self.assertEqual(wrap(-170-170), 20)
        self.assertEqual(wrap(170-(-170)), -20)

    def test_chassis_axis_gains_compensation_and_angular_units(self):
        # GUI朝+Y=0deg，即数学yaw90；左右误差10mm，前后误差10mm。
        program=build_program([(1000,1000),(1300,1300)],-180,goal_yaw=-175,
            control=dict(kpx=4,kpy=2,kpz=10,xyvmax=100,zvmax=60,xyvmin=5,zvmin=2))
        tracker=CoordinateTracker(program);goal=tracker.final_reference()
        outputs=[tracker.command((goal['x_mm']-10,goal['y_mm']-10,90),0,0) for _ in range(1000)]
        # 实际整数电机输出的长期平均必须保持七项参数指定的P增益/补偿。
        self.assertAlmostEqual(sum(o[1][0] for o in outputs)/len(outputs),45,delta=.005)
        self.assertAlmostEqual(sum(o[1][1] for o in outputs)/len(outputs),25,delta=.005)
        self.assertAlmostEqual(sum(o[2] for o in outputs)/len(outputs),math.degrees(-52/270),delta=.005)
        # 小于5的P输出保留，不能把numerical_limit的补偿死区误写成零速度。
        _ref,velocity,_omega=tracker.command((goal['x_mm']-1,goal['y_mm']-1,85),0,0)
        self.assertGreater(math.hypot(*velocity),0)

    def test_every_chassis_parameter_affects_coordinate_command(self):
        base=dict(kpx=2,kpy=2,kpz=9,xyvmax=300,zvmax=60,xyvmin=0,zvmin=0)
        for key, changed in dict(kpx=4,kpy=4,kpz=18,xyvmax=600,zvmax=120,xyvmin=10,zvmin=10).items():
            outputs=[]
            for setting in (base,dict(base,**{key:changed})):
                program=build_program([(1000,1000),(1500,1500)],-180,goal_yaw=-175,control=setting)
                tracker=CoordinateTracker(program);goal=tracker.final_reference()
                offset=1000 if key=='xyvmax' else 10
                yaw=90 if key!='zvmax' else 105
                # 远离目标时也使用同一设定车头，单独测各参数影响。
                tracker.points[-1]['travel_yaw_deg']=goal['field_yaw_deg']
                _ref,velocity,omega=tracker.command((goal['x_mm']-offset,goal['y_mm']-offset,yaw),250,120)
                outputs.append((*velocity,omega))
            self.assertNotEqual(outputs[0],outputs[1],key)

    def test_coordinate_speed_uses_chassis_limit_without_old_250_cap(self):
        program=build_program([(500,500),(2000,500)],0)
        tracker=CoordinateTracker(program);first=tracker.reference_at(0)
        _ref,velocity,_omega=tracker.command((first['x_mm'],first['y_mm'],first['field_yaw_deg']),250,120)
        self.assertAlmostEqual(math.hypot(*velocity),1600,delta=1/.238)

    def test_changed_parameters_reject_stale_program_and_cancel_active(self):
        program,scene=fixture()
        sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
        sim.hold=program['start']['x_mm'],program['start']['y_mm'];sim.zval=90-program['start']['field_yaw_deg']
        sim.handle_line('XVMAX=400')
        with self.assertRaisesRegex(ValueError,'底盘参数'):
            sim.submit_navigation_coordinates(sim.begin_navigation(),program,(0,0,0),scene)
        current=build_program([(500,500),(1500,500)],0,
                              control=dict(sim.chassis_parameters(),turn_lead_mm=60))
        sim.submit_navigation_coordinates(sim.begin_navigation(),current,(0,0,0),scene)
        sim.make_frame(0);pose=sim.hold,sim.zval
        sim.handle_line('KPX=3');sim.make_frame(.02)
        self.assertFalse(sim.navigation_snapshot()['active'])
        self.assertEqual((sim.hold,sim.zval),pose)
        self.assertEqual(sim.chassis_parameters()['kpx'],3)

    def test_parameter_change_during_acceptance_cannot_install_stale_tracker(self):
        program,scene=fixture()
        sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
        sim.hold=program['start']['x_mm'],program['start']['y_mm'];sim.zval=90-program['start']['field_yaw_deg']
        epoch=sim.begin_navigation();real=replay
        def change(*args):
            sim.handle_line('KPZ=18')
            return real(*args)
        with patch('coordinate_navigation.replay',side_effect=change):
            with self.assertRaises(ValueError):sim.submit_navigation_coordinates(epoch,program,(0,0,0),scene)
        self.assertFalse(sim.navigation_snapshot()['tracking'])

    def test_zero_control_reports_no_progress_instead_of_hanging(self):
        program,scene=fixture();program['control']['xyvmax']=0
        with self.assertRaisesRegex(ValueError,'无进展'):replay(program,scene)

    def test_final_stop_deadband_does_not_inject_yaw_pulses_or_teleport(self):
        _program,scene=fixture()
        for zmax in (750,0):
            program=build_program([(1000,1000),(1000,1000)],-180.02,goal_yaw=-180,
                                  control={'zvmax':zmax})
            sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
            sim.handle_line('ZVMAX='+str(zmax))
            sim.hold=program['start']['x_mm'],program['start']['y_mm'];sim.zval=90-program['start']['field_yaw_deg']
            sim.submit_navigation_coordinates(sim.begin_navigation(),program,(0,0,0),scene)
            for i in range(100):
                sim.make_frame(i/core.SEND_HZ)
                if not sim.navigation_snapshot()['active']:break
            snap=sim.navigation_snapshot()
            self.assertEqual(snap['tracking_status'],'COMPLETE')
            self.assertAlmostEqual(snap['yaw'],-.02,places=10)

    def test_coordinate_runtime_stall_keeps_control_and_stop_can_cancel(self):
        program,scene=fixture()
        sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
        sim.hold=program['start']['x_mm'],program['start']['y_mm'];sim.zval=90-program['start']['field_yaw_deg']
        sim.submit_navigation_coordinates(sim.begin_navigation(),program,(0,0,0),scene)
        tracker=sim._nav_tracker
        with patch.object(tracker,'command',return_value=(tracker._reference(1),(0,0),0)):
            for i in range(200):sim.make_frame(i/core.SEND_HZ)
        snap=sim.navigation_snapshot()
        self.assertTrue(snap['active']);self.assertEqual(snap['tracking_status'],'RECOVERING')
        sim.handle_line('STOP');self.assertFalse(sim.navigation_snapshot()['active'])

    def test_nondefault_chassis_snapshot_is_shared_by_entire_match(self):
        sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
        for command in ('KPX=3','KPY=2','KPZ=18','XVMAX=500','ZVMAX=400','XVMIN=3','ZVMIN=2'):
            sim.handle_line(command)
        parameters=sim.chassis_parameters()
        match=competition.compile_match(competition.load_profile(),coordinate_mode=True,chassis_control=parameters)
        for leg in match['legs']:
            self.assertEqual({k:leg['route']['waypoint_program']['control'][k] for k in parameters},parameters)
        sim.hold=core.layout_to_field(*match['home']);sim.zval=180+match['start_yaw']
        runner=competition.CompetitionRunner(sim,match);runner.start()
        for _ in range(9000):
            if not runner.active:break
            advance(runner)
        self.assertEqual(runner.status,'COMPLETE',runner.reason)
        self.assertEqual((runner.grabs,runner.placements),(12,12))

    def test_same_coordinate_hold_and_heading_alignment(self):
        _program, scene = fixture()
        for yaw in (0, 90):
            program=build_program([(1000,1000),(1000,1000)],0,goal_yaw=yaw)
            samples,_elapsed=replay(program,scene)
            self.assertEqual(samples[-1]['segment_type'],'STOP')
            self.assertAlmostEqual(samples[-1]['x_mm'],program['goal']['x_mm'])
            self.assertLess(abs(wrap(samples[-1]['field_yaw_deg']-program['goal']['field_yaw_deg'])),1)
            sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
            sim.hold=program['start']['x_mm'],program['start']['y_mm'];sim.zval=90-program['start']['field_yaw_deg']
            sim.submit_navigation_coordinates(sim.begin_navigation(),program,(0,0,0),scene)
            for i in range(500):
                sim.make_frame(i/core.SEND_HZ)
                if not sim.navigation_snapshot()['active']:break
            self.assertEqual(sim.navigation_snapshot()['tracking_status'],'COMPLETE')
        # 两个端点合法仍不能跳过中间旋转矩形的扫掠。
        tight=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
        unsafe=build_program([(155,155),(155,155)],0,goal_yaw=90)
        with self.assertRaisesRegex(ValueError,'扫掠'):replay(unsafe,tight)

    def test_lateral_limit_is_checked_during_replay(self):
        scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
        program=build_program([(500,500),(500,1500)],0,mode='FIXED',
                              constraints={'strafe_run_limit_mm':100})
        with self.assertRaisesRegex(ValueError,'横移限制'):replay(program,scene)

    def test_pass_switches_while_moving_and_final_stop_settles(self):
        program, scene = fixture()
        tracker = CoordinateTracker(program)
        target = tracker.points[1]
        before = (target['x_mm']+20,target['y_mm'],target['field_yaw_deg'])
        ref, velocity, _omega = tracker.command(before,250,120)
        self.assertEqual(tracker.index, 2)
        self.assertGreater(math.hypot(*velocity), 100)
        samples, _elapsed = replay(program,scene)
        self.assertEqual(samples[-1]['segment_type'],'STOP')
        self.assertFalse(any(p['segment_type']=='ARC' for p in samples))

    def test_mid_path_obstacle_rejected_even_if_waypoints_safe(self):
        program, _scene = fixture()
        scene = CollisionScene([],[(1000,500,25,'新增途中障碍')],(0,0,3000,3000),10,(280,260,0),None)
        for row in program['waypoints']:
            self.assertIsNone(scene.pose_reason(row['x_mm'],row['y_mm'],row['layout_yaw_deg']))
        with self.assertRaisesRegex(ValueError,'扫掠'):
            replay(program,scene)

    def test_stop_and_map_change_cancel_without_residual_motion(self):
        for cause in ('stop','map'):
            program, scene = fixture()
            sim = core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
            sim.hold=program['start']['x_mm'],program['start']['y_mm'];sim.zval=90-program['start']['field_yaw_deg']
            sim.submit_navigation_coordinates(sim.begin_navigation(),program,(0,0,0),scene)
            sim.make_frame(0);before=sim.hold,sim.zval
            if cause=='stop': sim.cancel_navigation()
            else: sim._nav_validity=lambda:False
            sim.make_frame(.02)
            self.assertEqual((sim.hold,sim.zval),before)
            self.assertFalse(sim.navigation_snapshot()['tracking'])

    def test_cancel_during_acceptance_cannot_restore_old_control(self):
        program, scene = fixture()
        sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
        sim.hold=program['start']['x_mm'],program['start']['y_mm'];sim.zval=90-program['start']['field_yaw_deg']
        epoch=sim.begin_navigation();entered=threading.Event();release=threading.Event();errors=[]
        real=replay
        def blocked(*args):
            entered.set();self.assertTrue(release.wait(2));return real(*args)
        def submit():
            try:sim.submit_navigation_coordinates(epoch,program,(0,0,0),scene)
            except ValueError as exc:errors.append(exc)
        with patch('coordinate_navigation.replay',side_effect=blocked):
            thread=threading.Thread(target=submit);thread.start();self.assertTrue(entered.wait(2))
            sim.cancel_navigation();release.set();thread.join(2)
        self.assertFalse(thread.is_alive());self.assertEqual(len(errors),1)
        self.assertFalse(sim.navigation_snapshot()['tracking'])

    def test_invalid_program_and_control_are_rejected(self):
        program,_scene=fixture()
        for change in (lambda p:p['waypoints'][-1].update(kind='PASS'),
                       lambda p:p['control'].update(kpx=float('nan')),
                       lambda p:p['waypoints'][1].update(x_mm=p['waypoints'][0]['x_mm'],y_mm=p['waypoints'][0]['y_mm'])):
            bad=copy.deepcopy(program);change(bad)
            with self.assertRaises(ValueError):CoordinateTracker(bad)

    def test_both_start_zones_complete_without_arc_generator_or_dense_execution(self):
        for zone in (1,2):
            with patch.object(competition,'smooth_90_corners',side_effect=AssertionError('不得生成圆弧')):
                match=competition.compile_match(competition.load_profile(),zone=zone,coordinate_mode=True)
            for stage in match['stages']:
                if stage['kind']=='TRAVEL':
                    self.assertEqual(stage['route']['arcs'],[])
                    from mecanum_planner import motion_metrics
                    route=stage['route']
                    self.assertLess(motion_metrics(route)['longest_strafe_mm'],100,stage['label'])
                    rows=route['trajectory']
                    yaw_travel=sum(abs(wrap(b['field_yaw_deg']-a['field_yaw_deg']))
                                   for a,b in zip(rows,rows[1:]))
                    self.assertLessEqual(yaw_travel,181,stage['label'])
                    # 默认无障碍场景不应通过小方圈调整航向；各轴不折返。
                    for axis in (0,1):
                        directions={1 if b[axis]>a[axis] else -1 for a,b in zip(route['points'],route['points'][1:])
                                    if abs(b[axis]-a[axis])>1e-6}
                        self.assertLessEqual(len(directions),1,stage['label'])
                    stage['route'].pop('trajectory');stage['route'].pop('smoothed_primitives')
            runner=runner_for(match)
            for _ in range(9000):
                if not runner.active:break
                advance(runner)
            self.assertEqual(runner.status,'COMPLETE',runner.reason)
            self.assertEqual((runner.grabs,runner.placements),(12,12))

    def test_two_obstacle_zone2_return_uses_safe_mecanum_connector(self):
        data=competition.load_profile()
        scene=competition.collision_scene(data,sim_obstacles=((1200,1200),(297,294)))
        result=plan_route((400,1200),(2100,300),scene,data['competition']['lane_nodes'],-180)
        self.assertTrue(result['execution_safe'])
        self.assertEqual(result['arcs'],[])
        # 默认1600参数不能沿原窄角点切入黄区，实际闭环须有绕行余量。
        samples,_elapsed=replay(result['waypoint_program'],scene)
        self.assertEqual(samples[-1]['segment_type'],'STOP')
        previous=None
        for row in samples:
            pose=(*core.field_to_layout(row['x_mm'],row['y_mm']),-90-row['field_yaw_deg'])
            self.assertIsNone(scene.pose_reason(*pose))
            if previous is not None:
                self.assertIsNone(scene.moving_pose_reason(previous,pose))
            previous=pose


if __name__=='__main__':unittest.main()
