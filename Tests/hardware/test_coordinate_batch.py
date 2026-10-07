"""真实C关键坐标整批接入：不打开串口、不驱动硬件。"""
import ctypes
import collections
import math
import unittest
import zlib
from pathlib import Path
import subprocess
from position_test_support import write_position_kernel
from motor_test_support import write_motor_kernel,wheel_engine,wheel_motion

import test_trajectory_buffer as legacy
from test_trajectory_buffer import Engine,Pose
import core
import competition_simulation as competition
from hardware_coordinates import POINT,make_coordinate_batch
from hardware_trajectory import BatchUploader


class CoordinateFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        legacy.TrajectoryFirmwareTests.setUpClass.__func__(cls)
        handle=cls.dll._handle;cls.dll=None
        ctypes.windll.kernel32.FreeLibrary.argtypes=[ctypes.c_void_p]
        ctypes.windll.kernel32.FreeLibrary(handle)
        folder=Path(cls.temp.name)
        subprocess.run(['gcc','-shared','-std=c99','-Wall','-Wextra','-Werror','-O2','-I',str(folder),
            '-I',str(legacy.ROOT/'Hardware'),str(legacy.ROOT/'Hardware/trajectory_buffer.c'),
            str(write_position_kernel(folder)),str(write_motor_kernel(folder)),'-o',str(cls.binary),'-lm'],check=True)
        cls.dll=ctypes.CDLL(str(cls.binary))
        cls.dll.Traj_ParseLine.argtypes=[ctypes.c_char_p,ctypes.c_uint32,ctypes.c_uint8]
        cls.dll.Traj_Step.argtypes=[ctypes.c_uint32,ctypes.POINTER(Pose),ctypes.c_uint8,ctypes.POINTER(ctypes.c_float)]
        cls.dll.Traj_PeekReply.argtypes=[ctypes.c_char_p,ctypes.c_uint16,ctypes.POINTER(legacy.Reply)]
        cls.dll.Traj_ReplySent.argtypes=[ctypes.POINTER(legacy.Reply)]
        cls.dll.Traj_UpdateChassisParameters.argtypes=[ctypes.POINTER(ctypes.c_float)]

    @classmethod
    def tearDownClass(cls):
        legacy.TrajectoryFirmwareTests.tearDownClass.__func__(cls)

    def setUp(self):
        self.e=wheel_engine(Engine)(self.dll)
        self.parameters=dict(core.CHASSIS_DEFAULTS)
        self.sync()

    def sync(self):
        values=(ctypes.c_float*7)(*(self.parameters[core.SIM_PARAM_ATTRS[k]] for k in core.CHASSIS_NAMES))
        self.dll.Traj_UpdateChassisParameters(values)

    def upload(self,batch):
        self.job=BatchUploader(batch,self.e.send,clock=lambda:self.e.now/1000)
        self.job.start()
        for _ in range(4000):
            text=self.e.reply()
            if text:self.job.handle_reply(text)
            else:self.e.step()
            if self.job.state in ('STARTING','CANCELLED'):break
        self.assertEqual(self.job.state,'STARTING',self.job.reason)
        self.assertEqual(sum(c.startswith('CPOINT=') for c in self.job.commands),len(batch['points']))
        self.assertEqual(sum(c.startswith('TRUN=') for c in self.job.commands),1)

    def small_batch(self,mapping=(0,0,0)):
        from coordinate_navigation import plan_route
        from navigation_planner import CollisionScene
        from hardware_coordinates import make_coordinate_path_batch
        scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
        route=plan_route((1000,1000),(1000,2000),scene,[],90,chassis_control=self.parameters)
        return make_coordinate_path_batch(route,(1000,1000),90,None,mapping,scene,{'map_version':1},token=17)

    def test_real_c_approach_heading_pass_rejects_ten_degree_error_without_stop(self):
        from tests.test_approach_heading import fixture
        from approach_heading import anticipate_route
        from hardware_coordinates import make_coordinate_path_batch
        route,scene=fixture();route=anticipate_route(route,scene)
        self.parameters.update(route['waypoint_program']['control']);self.parameters.pop('turn_lead_mm');self.sync()
        batch=make_coordinate_path_batch(route,(600,600),0,90,(0,0,0),scene,{'map_version':1},token=17)
        first=batch['points'][0];target=batch['points'][1]
        self.e.pose=Pose(first[0]/10,first[1]/10,first[3]/100,1,self.e.now)
        self.upload(batch)
        for _ in range(1000):
            self.e.step(move=True)
            while True:
                reply=self.e.reply()
                if not reply:break
                self.job.handle_reply(reply)
            self.job.tick()
            if math.dist((self.e.pose.x,self.e.pose.y),(target[0]/10,target[1]/10))<20:break
        else:self.fail('没有接近普通拐点')
        self.e.pose.x,self.e.pose.y=target[0]/10,target[1]/10
        self.e.pose.yaw=target[3]/100+10
        self.assertEqual(self.e.step(move=False),1)
        while self.e.reply():pass
        self.assertEqual(self.e.status()[3],0,'10deg误差不能放行到下一点')
        self.e.pose.yaw=target[3]/100+.5
        self.assertEqual(self.e.step(move=False),1,'完成车头后同帧衔接，不发停车动作2')
        while self.e.reply():pass
        self.assertEqual(self.e.status()[3],1)

    def test_real_c_wheel_lag_and_delayed_ops_finish_with_strict_gates(self):
        from tests.test_pivot_turns import CORNERS,baseline
        from pivot_turns import corner_trial,wheel_geometry
        from coordinate_navigation import _replay_result,replay
        from hardware_coordinates import make_coordinate_path_batch
        data=competition.load_profile();scene=competition.collision_scene(data)
        checked=competition.collision_scene(data,9)
        for tau,delay in ((.04,.02),(.08,.04),(.12,.04),(.16,.06)):
            with self.subTest(tau=tau,delay=delay):
                self.setUp()
                self.parameters.update(kpx=1.5,kpy=1.5,kpz=9.05,xyvmax=930,zvmax=750,xyvmin=.5,zvmin=.5)
                self.sync()
                route=baseline(CORNERS[0],scene,self.parameters)
                program,_=corner_trial(route['waypoint_program'],1,wheel_geometry(scene),scene,lambda:False)
                samples,elapsed=replay(program,scene)
                route=_replay_result(program,samples,elapsed,route['points'],route['length'],'FORWARD',
                    route['start_heading_deg'],route['goal_heading_deg'],(False,False))
                batch=make_coordinate_path_batch(route,CORNERS[0][0],route['start_heading_deg'],
                    route['goal_heading_deg'],(0,0,0),scene,data,token=17)
                first=batch['points'][0];truth=[first[0]/10,first[1]/10,first[3]/100]
                self.e.pose=Pose(*truth,1,self.e.now)
                self.upload(batch)
                history=collections.deque([tuple(truth)]*(round(delay*50)+1),maxlen=round(delay*50)+1)
                actual_wheels=[0.0]*4;alpha=.02/(tau+.02)
                previous=(*core.field_to_layout(*truth[:2]),180+truth[2]);arc_sign=None;reversals=0
                controls=0
                for _ in range(2000):
                    sensed=history[0]
                    self.e.pose.x,self.e.pose.y,self.e.pose.yaw=sensed
                    action=self.e.step(move=False)
                    if action==1:
                        self.dll.MecanumControl_MoveWorldVelocity(*self.e.velocity,self.e.pose.yaw)
                    elif action==2:
                        self.dll.MecanumControl_Stop()
                    for i in range(4):actual_wheels[i]+=alpha*(self.e.wheels[i]-actual_wheels[i])
                    vx,vy,omega=wheel_motion(actual_wheels,truth[2])
                    truth=[truth[0]+vx*.02,truth[1]+vy*.02,truth[2]+omega*.02]
                    history.append(tuple(truth))
                    pose=(*core.field_to_layout(*truth[:2]),180+truth[2])
                    self.assertIsNone(checked.moving_pose_reason(previous,pose),'惯性过渡仍需完整车体扫掠')
                    previous=pose
                    while True:
                        text=self.e.reply()
                        if not text:break
                        if text.startswith('CCTRL '):
                            self.assertLessEqual(len(text.encode('ascii'))+2,100,'真实DMA回复槽不得溢出')
                            fields=list(map(int,text.split()[1:]));self.assertEqual(len(fields),10)
                            self.assertEqual(fields[0],17)
                            controls+=1
                        self.job.handle_reply(text)
                    if self.job.cursor==1 and abs(omega)>5:
                        sign=1 if omega>0 else -1
                        if arc_sign is not None and sign!=arc_sign:reversals+=1
                        arc_sign=sign
                    self.job.tick()
                    if not self.job.active:break
                self.assertEqual(self.job.state,'DONE',(tau,delay,self.job.reason))
                self.assertLessEqual(reversals,0 if tau<=.08 else 1,
                    '中等延迟下不能越过目标后再反向追角；强延迟仍保留严格恢复')
                self.assertGreater(controls,10,'必须获得实际C控制快照，而非仅重建目标')
                last=batch['points'][-1]
                self.assertLess(math.dist(sensed[:2],(last[0]/10,last[1]/10)),1)
                self.assertLess(abs((sensed[2]-last[3]/100+180)%360-180),1)

    def test_missed_pivot_exit_reports_recovery_without_rotation_floor(self):
        # 延迟不一定错过出口，且5Hz邮箱可能漏掉短暂恢复；明确注入错位验证该分支。
        batch=self.pivot_batch()
        pivot=next(i for i,p in enumerate(batch['points']) if p[4]&16)
        end=batch['points'][pivot]
        first=batch['points'][0]
        self.e.pose=Pose(first[0]/10,first[1]/10,first[3]/100,1,self.e.now)
        self.upload(batch)
        for _ in range(2000):
            self.e.step(move=True)
            while True:
                reply=self.e.reply()
                if not reply:break
                self.job.handle_reply(reply)
            self.job.tick()
            yaw_gap=abs((self.e.pose.yaw-end[3]/100+180)%360-180)
            if self.job.cursor==pivot-1 and yaw_gap<5:break
        else:self.fail('未进入绕轮出口前的测试位置')
        injected=(end[0]/10+10,end[1]/10,end[3]/100)
        self.assertLess(math.dist((self.e.pose.x,self.e.pose.y),injected[:2]),50)
        self.e.pose.x,self.e.pose.y,self.e.pose.yaw=injected
        snapshots=[]
        for _ in range(30):
            self.e.step(move=False)
            while True:
                reply=self.e.reply()
                if not reply:break
                if reply.startswith('CCTRL '):snapshots.append(list(map(int,reply.split()[1:])))
                self.job.handle_reply(reply)
            self.job.tick()
        self.assertEqual(self.job.cursor,pivot-1,'10mm误差不能放宽为通过出口')
        recovery=[row for row in snapshots if row[1]==pivot and row[2]&8]
        self.assertTrue(recovery,'明确错过出口时应报告恢复')
        last=recovery[-1]
        self.assertTrue(last[2]&4)
        self.assertEqual(last[7],0,'目标航向已到时旋转请求必须趋于零')
        self.assertGreater(math.hypot(last[5],last[6]),0,'位置误差仍需纠正')

    def test_eight_matches_c_execute_all_coordinates_with_swept_body(self):
        data=competition.load_profile();results=[]
        for obstacles in ((),((700,1200),),competition.DEMO_OBSTACLES,((1200,1200),(297,294))):
            for zone in (1,2):
                self.setUp()
                match=competition.compile_match(data,zone=zone,sim_obstacles=obstacles,coordinate_mode=True)
                scene=competition.collision_scene(data,sim_obstacles=obstacles)
                batch=make_coordinate_batch(match,(0,0,0),scene,token=17)
                self.assertLess(len(batch['points']),100)
                first=batch['points'][0];self.e.pose=Pose(first[0]/10,first[1]/10,first[3]/100,1,self.e.now)
                self.upload(batch)
                actual_scene=competition.collision_scene(data,9,obstacles)
                previous=None;pass_motion=0;previous_cursor=0;visited=set();work_stops=set()
                for _ in range(8500):
                    action=self.e.step(move=True)
                    pose=(*core.field_to_layout(self.e.pose.x,self.e.pose.y),180+self.e.pose.yaw)
                    reason=actual_scene.pose_reason(*pose)
                    if previous and not reason:reason=actual_scene.moving_pose_reason(previous,pose)
                    self.assertIsNone(reason,(zone,obstacles,pose,reason))
                    previous=pose
                    text=self.e.reply()
                    if text:self.job.handle_reply(text)
                    # C完成站点停车的同一帧，实际模型车右须对准作业区。
                    native_cursor=self.job.cursor
                    if action==2 and native_cursor in batch['stations']:
                        from work_orientation import work_heading
                        labels=batch['stations'][native_cursor]
                        station='raw' if any('取料' in label for label in labels) else 'rough' if any('粗加工' in label for label in labels) else 'storage' if any('暂存' in label for label in labels) else None
                        if station:
                            required=work_heading(data['competition'],station)
                            self.assertLess(abs((180+self.e.pose.yaw-required+180)%360-180),1)
                            work_stops.add(native_cursor)
                    self.job.tick()
                    if self.job.cursor>previous_cursor and not batch['points'][self.job.cursor][4]&1:
                        if action==1 and math.hypot(*self.e.velocity[:2])>1:pass_motion+=1
                    visited.add(self.job.cursor)
                    previous_cursor=self.job.cursor
                    if not self.job.active:break
                self.assertEqual(self.job.state,'DONE',(zone,obstacles,self.job.reason,self.e.status()))
                self.assertGreater(pass_motion,0)
                self.assertEqual(len(work_stops),6,'两批六次作业停靠必须经过实际C到位判断')
                self.assertTrue(set(batch["stations"]).issubset(visited))
                self.assertFalse(any(c.startswith("TRESUME=") for c in self.job.commands))
                self.assertEqual(sum(c.startswith('CPOINT=') for c in self.job.commands),len(batch['points']))
                results.append((len(obstacles),zone,len(batch['points']),round(self.e.now/1000,2)))
        print('C关键坐标全场回放:',results)

    def test_protocol_crc_and_parameters(self):
        b=self.small_batch()
        self.e.send('CCAPS');self.assertEqual(self.e.reply(),'CCAPS 9 2048 3')
        self.parameters['kpx']=4;self.sync()
        job=BatchUploader(b,self.e.send,clock=lambda:self.e.now/1000);job.start()
        job.handle_reply(self.e.reply());job.handle_reply(self.e.reply())
        self.assertEqual(job.state,'CANCELLED');self.assertIn('参数',job.reason)
        self.setUp()
        bad=dict(b,crc=b['crc']^1)
        job=BatchUploader(bad,self.e.send,clock=lambda:self.e.now/1000);job.start()
        for _ in range(20):
            text=self.e.reply()
            if text:job.handle_reply(text)
            if not job.active:break
        self.assertEqual(job.state,'CANCELLED');self.assertIn('CRC',job.reason)

    def test_four_outer_corners_both_directions_use_real_c_pivot_controller(self):
        from tests.test_pivot_turns import CORNERS,baseline
        from pivot_turns import select_turns
        from hardware_coordinates import make_coordinate_path_batch
        data=competition.load_profile();scene=competition.collision_scene(data)
        actual_scene=competition.collision_scene(data,9);results=[]
        for points in CORNERS:
            for reverse in (False,True):
                self.setUp();route=list(reversed(points)) if reverse else points
                result=select_turns(baseline(route,scene),scene,lambda:False)
                batch=make_coordinate_path_batch(result,route[0],result['start_heading_deg'],
                    result['goal_heading_deg'],(0,0,0),scene,data,token=17)
                self.assertEqual(batch['coordinate_version'],3);self.assertEqual(batch['pivot_count'],1)
                first=batch['points'][0]
                self.e.pose=Pose(first[0]/10,first[1]/10,first[3]/100,1,self.e.now)
                self.upload(batch);previous=None;arc_frames=0;maximum_radial=0;maximum_anchor=0
                boundary_speeds=[]
                for _ in range(1500):
                    old_xy=(self.e.pose.x,self.e.pose.y)
                    action=self.e.step(move=True)
                    arc_index=next(i for i,q in enumerate(batch['points']) if q[4]&32)
                    for boundary in batch['points'][arc_index-1:arc_index+1]:
                        if math.dist(old_xy,(boundary[0]/10,boundary[1]/10))<10:
                            boundary_speeds.append(math.dist(old_xy,(self.e.pose.x,self.e.pose.y))/.02)
                        if math.dist((self.e.pose.x,self.e.pose.y),(boundary[0]/10,boundary[1]/10))<3:
                            self.assertEqual(action,1,'入弯/出弯不得发动作2停车或等待200ms')
                    pose=(*core.field_to_layout(self.e.pose.x,self.e.pose.y),180+self.e.pose.yaw)
                    why=actual_scene.pose_reason(*pose)
                    if previous and not why:why=actual_scene.moving_pose_reason(previous,pose)
                    self.assertIsNone(why,(route,pose,why))
                    previous=pose
                    text=self.e.reply()
                    if text:self.job.handle_reply(text)
                    target=min(self.job.cursor+1,len(batch['points'])-1)
                    if batch['points'][target][4]&16:
                        q=batch['points'][target];entry=batch['points'][target-1]
                        radius=math.dist(entry[:2],q[8:10])/10
                        maximum_radial=max(maximum_radial,abs(math.dist((self.e.pose.x,self.e.pose.y),(q[8]/10,q[9]/10))-radius))
                        start=batch['points'][self.job.cursor]
                        angle=math.radians(start[3]/100);dx=(q[8]-start[0])/10;dy=(q[9]-start[1])/10
                        lateral=math.cos(angle)*dx-math.sin(angle)*dy
                        forward=math.sin(angle)*dx+math.cos(angle)*dy
                        angle=math.radians(self.e.pose.yaw)
                        anchor=(self.e.pose.x+math.cos(angle)*lateral+math.sin(angle)*forward,
                                self.e.pose.y-math.sin(angle)*lateral+math.cos(angle)*forward)
                        maximum_anchor=max(maximum_anchor,math.dist(anchor,(q[8]/10,q[9]/10)))
                        arc_frames+=1
                    self.job.tick()
                    if not self.job.active:break
                self.assertEqual(self.job.state,'DONE',(route,self.job.reason,self.e.status()))
                self.assertGreater(arc_frames,30);self.assertLess(maximum_radial,10)
                self.assertLess(maximum_anchor,2,'支点轮心须保持在原位置，不能仅验证车心半径')
                self.assertTrue(boundary_speeds)
                self.assertGreater(min(boundary_speeds),20,'实机整数轮速回放：进出弯不能降至零速再重启')
                results.append((route[1],reverse,round(self.e.now/1000,2),round(maximum_anchor,3),round(min(boundary_speeds),2)))
        print('真实C四角指定圆心:',results)

    def test_new_pivot_batch_refuses_old_firmware_before_begin_or_points(self):
        from tests.test_pivot_turns import CORNERS,baseline
        from pivot_turns import select_turns
        from hardware_coordinates import make_coordinate_path_batch
        data=competition.load_profile();scene=competition.collision_scene(data)
        result=select_turns(baseline(CORNERS[0],scene),scene,lambda:False)
        batch=make_coordinate_path_batch(result,CORNERS[0][0],result['start_heading_deg'],
            result['goal_heading_deg'],(0,0,0),scene,data,token=17)
        for version in (1,2,3,4,5,6,7):
            sent=[];job=BatchUploader(batch,sent.append,clock=lambda:0)
            job.start();job.handle_reply('CCAPS %d 2048 3'%version)
            self.assertEqual(job.state,'CANCELLED');self.assertIn('CCAPS8',job.reason)
            self.assertFalse(any(c.startswith(('CBEGIN=','CPOINT=','TRUN=')) for c in sent))

    def test_plain_coordinate_batch_also_requires_calibrated_geometry_firmware(self):
        batch=self.small_batch()
        self.assertEqual(batch['coordinate_version'],1,'普通直线仍保持旧22字节点格式')
        for version in (1,2,3,4,5,6,7):
            sent=[];job=BatchUploader(batch,sent.append,clock=lambda:0)
            job.start();job.handle_reply('CCAPS %d 2048 3'%version)
            self.assertEqual(job.state,'CANCELLED');self.assertIn('CCAPS8',job.reason)
            self.assertFalse(any(c.startswith(('CBEGIN=','CPOINT=','TRUN=')) for c in sent))

    def pivot_batch(self,mapping=(0,0,0),stop_on_exit=False):
        from tests.test_pivot_turns import CORNERS,baseline
        from pivot_turns import select_turns
        from coordinate_navigation import replay,_replay_result
        from hardware_coordinates import make_coordinate_path_batch
        data=competition.load_profile();scene=competition.collision_scene(data)
        route=CORNERS[0];result=select_turns(baseline(route,scene,self.parameters),scene,lambda:False)
        self.assertTrue(result.get('pivots'),result.get('pivot_attempts'))
        if stop_on_exit:
            program=result['waypoint_program']
            program['waypoints']=program['waypoints'][:3]
            program['waypoints'][-1].update(kind='STOP',pass_mm=0)
            samples,elapsed=replay(program,scene)
            result=_replay_result(program,samples,elapsed,route,0,'PIVOT',90,180,(False,False))
        return make_coordinate_path_batch(result,route[0],90,180,mapping,scene,data,token=17)

    def test_pivot_center_mapping_and_final_arc_stop_use_real_motor_output(self):
        data=competition.load_profile();actual=competition.collision_scene(data,9)
        for mapping in ((4000,-800,35),(-1200,1000,-170),(0,0,90)):
            self.setUp()
            self.parameters.update(kpx=3,kpy=4,kpz=12,xyvmax=350,zvmax=250,xyvmin=3,zvmin=2)
            self.sync();batch=self.pivot_batch(mapping,stop_on_exit=True)
            first=batch['points'][0];self.e.pose=Pose(first[0]/10,first[1]/10,first[3]/100,1,self.e.now)
            self.upload(batch);ox,oy,theta=mapping
            c,s=math.cos(math.radians(theta)),math.sin(math.radians(theta));previous=None
            for _ in range(3000):
                self.e.step(move=True)
                x,y=self.e.pose.x,self.e.pose.y
                pose=(*core.field_to_layout(ox+c*x+s*y,oy-s*x+c*y),180+self.e.pose.yaw+theta)
                self.assertIsNone(actual.pose_reason(*pose),(mapping,pose))
                if previous:self.assertIsNone(actual.moving_pose_reason(previous,pose),(mapping,pose))
                previous=pose;text=self.e.reply()
                if text:self.job.handle_reply(text)
                self.job.tick()
                if not self.job.active:break
            self.assertEqual(self.job.state,'DONE',(mapping,self.job.reason))
            end=batch['points'][-1]
            self.assertLess(math.dist((self.e.pose.x,self.e.pose.y),(end[0]/10,end[1]/10)),.6)
            self.assertLess(abs((self.e.pose.yaw-end[3]/100+180)%360-180),.3)
            self.assertEqual(list(self.e.velocity),[0,0,0])

    def test_pivot_center_is_covered_by_crc_and_geometry_verification(self):
        import copy
        from hardware_coordinates import PIVOT_POINT
        batch=self.pivot_batch()
        for change in ('crc','geometry'):
            self.setUp();bad=copy.deepcopy(batch)
            index=next(i for i,p in enumerate(bad['points']) if p[4]&16)
            row=list(bad['points'][index]);row[8]+=1000;bad['points'][index]=tuple(row)
            if change=='geometry':bad['crc']=zlib.crc32(b''.join(PIVOT_POINT.pack(*p) for p in bad['points']))
            job=BatchUploader(bad,self.e.send,clock=lambda:self.e.now/1000);job.start()
            for _ in range(150):
                text=self.e.reply()
                if text:job.handle_reply(text)
                else:self.e.step()
                if not job.active:break
            self.assertEqual(job.state,'CANCELLED',(change,job.reason))
            self.assertFalse(any(line.startswith('TRUN=') for line in job.commands))
            self.assertFalse(self.dll.Traj_OutputAllowed())

    def test_exit_staging_and_next_stop_complete_through_integer_motors(self):
        # The old mixer stalled at (149.118,149.118), progress210mm, FAULT14.
        for dx,dy in ((150,150),(150,148),(0,150),(-150,-150)):
            with self.subTest(target=(dx,dy)):
                self.setUp()
                points=[(0,0,0,0,1,0,0,2000),
                        (dx*10,dy*10,round(math.hypot(dx,dy)*10),0,1,0,0,2000),
                        (dx*10,(dy+150)*10,round((math.hypot(dx,dy)+150)*10),0,1,0,0,2000)]
                crc=zlib.crc32(b''.join(POINT.pack(*p) for p in points))
                self.e.send('CBEGIN=17,3,1,%08X,10,2300,2300,9000,1600000,750000,5000,5000'%crc)
                self.e.reply()
                for i,p in enumerate(points):
                    x,y,s,yaw,flags,travel,passed,lead=p
                    self.e.send('CPOINT='+','.join(map(str,(17,i,x,y,yaw,s,flags,travel,passed,lead))))
                    self.e.reply()
                self.e.send('TCOMMIT=17')
                for _ in range(5):self.e.step();self.e.reply()
                self.e.send('TRUN=17');self.e.reply()
                visited=set()
                for _ in range(1200):
                    self.e.step(move=True)
                    status=self.e.status();visited.add(status[3])
                    if status[1] in (6,8):break
                self.assertEqual(status[1],6,status)
                self.assertIn(1,visited)
                self.assertLess(math.dist((self.e.pose.x,self.e.pose.y),(dx,dy+150)),.6)
                self.assertEqual(list(self.e.wheels),[0]*4)

    def test_fractional_rpm_mean_direction_limits_and_reset(self):
        # Independent inverse kinematics measures output, including mixed signs/yaw.
        for request in ((2,2,0,0),(-2,1,.1,35),(.1,-.2,-.02,-170),(16000,16000,2000,90)):
            self.dll.MecanumControl_Stop()
            samples=[]
            for _ in range(1000):
                self.dll.MecanumControl_MoveWorldVelocity(*request)
                self.assertLessEqual(max(map(abs,self.e.wheels)),3000)
                samples.append(wheel_motion(self.e.wheels,request[3]))
            if max(map(abs,request[:3]))<100:
                average=tuple(sum(s[i] for s in samples)/len(samples) for i in range(3))
                for actual,expected in zip(average,request):self.assertAlmostEqual(actual,expected,delta=.005)
        for reset in ('MecanumControl_Stop','MecanumControl_ClearTarget','MecanumControl_Disable'):
            self.dll.MecanumControl_Stop()
            self.dll.MecanumControl_MoveWorldVelocity(1,1,0,0)
            getattr(self.dll,reset)()
            self.dll.MecanumControl_MoveWorldVelocity(1,1,0,0)
            self.assertEqual(list(self.e.wheels),[0]*4,reset)
        self.dll.MecanumControl_MoveWorldVelocity(-1,-1,0,0)
        self.assertTrue(all(w>=0 for w in self.e.wheels[:2]))
        for _ in range(10):self.dll.MecanumControl_MoveWorldVelocity(0,0,0,0)
        self.assertEqual(list(self.e.wheels),[0]*4)
        self.dll.MecanumControl_MoveWorldVelocity(1,1,0,0)
        self.dll.MecanumControl_MoveVelocity(0,0,0)
        self.dll.MecanumControl_MoveWorldVelocity(1,1,0,0)
        self.assertEqual(list(self.e.wheels),[0]*4)

    def test_fault_text_matches_coordinate_three_second_gate(self):
        b=self.small_batch();self.upload(b)
        self.job.handle_reply('TSTAT 17 8 %d 0 0 14'%len(b['points']))
        self.assertEqual(self.job.state,'CANCELLED')
        self.assertIn('3秒无进展',self.job.reason)

    def test_stationary_ops_noise_can_finish_initial_stop(self):
        # Physically stationary, but +/-0.03mm and +/-0.03deg at50Hz make
        # instantaneous differentiation exceed1mm/s and terminal yaw snapping.
        b=self.small_batch();p=b['points'][0]
        self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
        self.upload(b)
        origin=(self.e.pose.x,self.e.pose.y,self.e.pose.yaw)
        started=False
        for i in range(200):
            noise=.03 if i%2 else -.03
            self.e.pose.x=origin[0]+noise;self.e.pose.y=origin[1]-noise
            self.e.pose.yaw=origin[2]+noise
            self.e.step()
            if math.hypot(*self.e.velocity[:2])>10:
                started=True;break
            status=self.e.status()
            if status[1]==8:break
        self.assertTrue(started,self.e.status())

    def test_real_drift_cannot_finish_initial_stop(self):
        for axis,increment in (('x',.04),('yaw',.03)):
            self.setUp();b=self.small_batch();p=b['points'][0]
            self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
            self.upload(b)
            for _ in range(40):
                # 2mm/s or1.5deg/s physical drift, within the startup pose gate.
                setattr(self.e.pose,axis,getattr(self.e.pose,axis)+increment)
                self.e.step()
                self.assertLess(math.hypot(*self.e.velocity[:2]),10)
                self.assertEqual(self.e.status()[3],0)

    def test_three_second_stall_warns_then_recovers_without_abort_or_skip(self):
        b=self.small_batch();p=b['points'][0]
        self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
        self.upload(b)
        for _ in range(30):self.e.step(move=True);self.e.reply()
        before=self.e.pose.x,self.e.pose.y,self.e.pose.yaw
        warning=[]
        for _ in range(180):
            self.e.step()
            while (text:=self.e.reply()):
                self.job.handle_reply(text)
                if text.startswith('CSTALL '):warning.append(text)
            self.job.tick()
        self.assertEqual(self.e.status()[1],4)
        self.assertEqual(self.job.state,'RUNNING')
        self.assertEqual((self.e.pose.x,self.e.pose.y,self.e.pose.yaw),before)
        self.assertEqual(len(warning),1)
        self.assertIn('继续纠偏',self.job.warning)
        self.assertNotIn('STOP',self.job.commands)
        self.assertEqual(len(self.job.batch['stall_diagnostics']),1)
        for _ in range(1200):
            self.e.step(move=True)
            while (text:=self.e.reply()):self.job.handle_reply(text)
            self.job.tick()
            if not self.job.active:break
        self.assertEqual(self.job.state,'DONE',self.job.reason)
        self.assertEqual(self.job.warning,'')

    def test_stall_does_not_disable_stop_and_total_timeout(self):
        for cause in ('STOP','timeout'):
            self.setUp();b=self.small_batch();p=b['points'][0]
            self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
            self.upload(b)
            for _ in range(200):self.e.step();self.e.reply()
            self.assertEqual(self.e.status()[1],4)
            if cause=='STOP':self.e.send('TABORT=17');self.e.step()
            else:
                for _ in range(8900):
                    self.e.step();self.e.reply()
                    if self.e.status()[1]==8:break
            self.assertIn(self.e.status()[1],(7,8))
            self.assertFalse(self.dll.Traj_OutputAllowed())

    def test_stall_diagnostic_retry_and_old_send_ack_keep_new_generation(self):
        b=self.small_batch();p=b['points'][0]
        self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
        self.upload(b)
        for _ in range(200):self.e.step()
        buf=ctypes.create_string_buffer(100);old=legacy.Reply();again=legacy.Reply()
        self.assertGreater(self.dll.Traj_PeekReply(buf,100,ctypes.byref(old)),0)
        first=buf.value;self.assertTrue(first.startswith(b'CSTALL '));self.assertEqual(old.kind,3)
        self.dll.Traj_PeekReply(buf,100,ctypes.byref(again))
        self.assertEqual(buf.value,first);self.assertEqual(again.generation,old.generation)
        # A real small movement resets the stall timer, then a second frozen episode.
        dx=b['points'][-1][0]/10-self.e.pose.x;dy=b['points'][-1][1]/10-self.e.pose.y
        span=math.hypot(dx,dy)
        self.e.pose.x+=10*dx/span;self.e.pose.y+=10*dy/span
        for _ in range(200):self.e.step()
        self.dll.Traj_ReplySent(ctypes.byref(old))
        new=legacy.Reply();self.dll.Traj_PeekReply(buf,100,ctypes.byref(new))
        self.assertEqual(new.kind,3,(b['points'],first,buf.value,new.generation,old.generation,
                                    self.e.pose.x,self.e.pose.y,self.e.pose.yaw,list(self.e.velocity)))
        self.assertGreater(new.generation,old.generation)
        self.assertNotEqual(buf.value,first)
        self.dll.Traj_ReplySent(ctypes.byref(new))
        self.dll.Traj_PeekReply(buf,100,ctypes.byref(new))
        self.assertTrue(buf.value.startswith(b'TSTAT '))

    def test_rotated_mapping_and_nondefault_body_parameters(self):
        for mapping in ((4000,-800,35),(-1200,1000,-170),(0,0,90)):
            self.setUp()
            self.parameters.update(kpx=3.1,kpy=1.7,kpz=7,xyvmax=800,zvmax=500,xyvmin=8,zvmin=6)
            self.sync();b=self.small_batch(mapping);p=b['points'][0]
            self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
            self.upload(b)
            for _ in range(3000):
                self.e.step(move=True)
                reply=self.e.reply()
                if reply:self.job.handle_reply(reply)
                self.job.tick()
                if not self.job.active:break
            self.assertEqual(self.job.state,'DONE',self.job.reason)
            end=b['points'][-1]
            self.assertLess(math.dist((self.e.pose.x,self.e.pose.y),(end[0]/10,end[1]/10)),.6)
            self.assertLess(abs((self.e.pose.yaw-end[3]/100+180)%360-180),.3)

    def test_duplicate_conflict_and_legacy_capability_do_not_run(self):
        b=self.small_batch()
        job=BatchUploader(b,self.e.send,clock=lambda:self.e.now/1000);job.start()
        self.assertFalse(job.handle_reply('TCAPS 1 4096 3'))
        self.assertEqual(job.state,'CAPS')
        job.handle_reply(self.e.reply());job.handle_reply(self.e.reply())
        point=next(c for c in job.commands if c.startswith('CPOINT='))
        self.e.send(point)
        self.assertEqual(self.e.status()[1],1)
        values=point.split(',');values[2]=str(int(values[2])+1)
        self.e.send(','.join(values))
        self.assertEqual(self.e.status()[1],8)
        self.assertFalse(self.dll.Traj_OutputAllowed())
        self.assertFalse(any(c.startswith('TRUN=') for c in job.commands))

    def test_stop_parameter_change_pose_loss_and_deviation(self):
        for cause in ('stop','parameters','stale','deviation','disabled'):
            self.setUp();b=self.small_batch();p=b['points'][0]
            self.e.pose=Pose(p[0]/10,p[1]/10,p[3]/100,1,self.e.now)
            self.upload(b)
            for _ in range(25):self.e.step(move=True)
            if cause=='stop':self.e.send('TABORT=17')
            if cause=='parameters':self.parameters['kpy']=3;self.sync()
            if cause=='deviation':
                # 连续小步漂移，不利用OPS跳变检测代替路径偏差检测。
                for _ in range(5):self.e.pose.y+=20;self.e.step()
            if cause=='stale':
                self.e.now+=220
                action=self.dll.Traj_Step(self.e.now,ctypes.byref(self.e.pose),1,self.e.velocity)
            else:action=self.e.step(allowed=0 if cause=='disabled' else 1)
            self.assertIn(self.e.status()[1],(7,8),cause)
            self.assertFalse(self.dll.Traj_OutputAllowed())
            self.assertEqual(list(self.e.velocity),[0,0,0],cause)


if __name__=='__main__':unittest.main(verbosity=2)
