"""用真实C接收/跟踪代码离线回放整轮比赛；不访问任何串口或GPIO。"""
import ctypes
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT=Path(__file__).resolve().parents[2]
QT=ROOT/'HostTools/ILHC_Debugger/ILHC_Qt_v2'
sys.path.insert(0,str(QT))
import core
import competition_simulation as competition
from hardware_trajectory import POINT,STOP,WAIT,ROTATE,BatchUploader,make_match_batch,make_path_batch,wrap


class Pose(ctypes.Structure):
    _fields_=[('x',ctypes.c_float),('y',ctypes.c_float),('yaw',ctypes.c_float),
              ('sequence',ctypes.c_uint32),('pose_tick',ctypes.c_uint32)]


class Reply(ctypes.Structure):
    _fields_=[('generation',ctypes.c_uint32),('kind',ctypes.c_uint8)]


class Engine:
    def __init__(self,dll):
        self.dll=dll;self.now=1000;self.pose=Pose(0,0,0,1,self.now);self.velocity=(ctypes.c_float*3)()
        self.dll.Traj_Init()

    def send(self,line,allowed=1):
        return self.dll.Traj_ParseLine(line.encode(),self.now,allowed)

    def step(self,allowed=1,move=False):
        self.now+=20;self.pose.sequence+=1;self.pose.pose_tick=self.now
        action=self.dll.Traj_Step(self.now,ctypes.byref(self.pose),allowed,self.velocity)
        if move and action==1:
            self.pose.x+=self.velocity[0]*.02;self.pose.y+=self.velocity[1]*.02
            self.pose.yaw=wrap(self.pose.yaw+self.velocity[2]*.02)
        return action

    def reply(self):
        buf=ctypes.create_string_buffer(100);token=Reply()
        n=self.dll.Traj_PeekReply(buf,100,ctypes.byref(token))
        if n:self.dll.Traj_ReplySent(ctypes.byref(token))
        return buf.value.decode().strip() if n else None

    def status(self):
        self.send('TSTATUS=17')
        result=self.reply()
        return tuple(map(int,result.split()[1:]))

    def upload(self,points,crc=None):
        checksum=zlib.crc32(b''.join(POINT.pack(*p) for p in points)) if crc is None else crc
        self.send('TBEGIN=17,%d,1,%08X,10'%(len(points),checksum));self.reply()
        for i,(x,y,s,yaw,flags) in enumerate(points):
            self.send('TPOINT=17,%d,%d,%d,%d,%d,%d'%(i,x,y,yaw,s,flags))
        self.send('TCOMMIT=17')
        for _ in range(math.ceil(len(points)/64)+1):self.step()
        return self.status()


def straight(wait=False):
    return [(0,i*200,i*200,0,(STOP|WAIT if wait and i==10 else STOP if i==50 else 0)) for i in range(51)]


class TrajectoryFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(prefix='ilhc-trajectory-native-')
        folder=Path(cls.temp.name)
        (folder/'main.h').write_text('#include <stdint.h>\nstatic inline uint32_t __get_PRIMASK(void){return 0;}\n'
            'static inline void __disable_irq(void){}\nstatic inline void __enable_irq(void){}\n',encoding='utf-8')
        cls.binary=folder/'trajectory.dll'
        subprocess.run(['gcc','-shared','-std=c99','-Wall','-Wextra','-Werror','-O2','-I',str(folder),
            str(ROOT/'Hardware/trajectory_buffer.c'),'-o',str(cls.binary),'-lm'],check=True)
        cls.dll=ctypes.CDLL(str(cls.binary))
        cls.dll.Traj_ParseLine.argtypes=[ctypes.c_char_p,ctypes.c_uint32,ctypes.c_uint8]
        cls.dll.Traj_Step.argtypes=[ctypes.c_uint32,ctypes.POINTER(Pose),ctypes.c_uint8,ctypes.POINTER(ctypes.c_float)]
        cls.dll.Traj_PeekReply.argtypes=[ctypes.c_char_p,ctypes.c_uint16,ctypes.POINTER(Reply)]
        cls.dll.Traj_ReplySent.argtypes=[ctypes.POINTER(Reply)]
        cls.dll.Traj_CompleteStation.argtypes=[ctypes.c_uint32,ctypes.c_uint16]

    @classmethod
    def tearDownClass(cls):
        # Windows需先卸载DLL，才能清理临时目录。
        handle=cls.dll._handle;cls.dll=None
        ctypes.windll.kernel32.FreeLibrary.argtypes=[ctypes.c_void_p]
        ctypes.windll.kernel32.FreeLibrary(handle)
        cls.temp.cleanup()

    def setUp(self):self.e=Engine(self.dll)

    def test_normal_home_negative_zero_batches_validate_and_execute_in_real_c(self):
        data=competition.load_profile()
        start=core.field_to_layout(1045,1031)
        for zone in (1,2):
            for actual_yaw in (-.005,-.02,.02):
                with self.subTest(zone=zone,yaw=actual_yaw):
                    layout_yaw=180+actual_yaw
                    scene=competition.collision_scene(data,10)
                    kwargs=dict(grid=100,pad=10,footprint=(280,260,layout_yaw),
                        start_heading_deg=layout_yaw,goal_heading_deg=180,rects=data['rects'],
                        circles=data['circles'],bounds=data['bounds'],drivable_polygons=data['drivable_polygons'],
                        strafe_run_limit_mm=500,turn_penalty_mm=180,geometry_verified=False)
                    route=core.plan_home_path(start,core.ZONE_CENTER[zone],**kwargs)
                    self.assertTrue(route['ok'],route.get('reason'))
                    batch=make_path_batch(route,start,layout_yaw,180,(0,0,0),scene,data,token=17)
                    self.e=Engine(self.dll)
                    self.assertEqual(self.e.upload(batch['points'])[1],3,'真实C结构校验必须READY')
                    self.e.pose.x=1045;self.e.pose.y=1031;self.e.pose.yaw=actual_yaw
                    self.e.send('TRUN=17')
                    previous=(*start,layout_yaw)
                    actual_scene=competition.collision_scene(data,9)
                    for _ in range(8000):
                        self.e.step(move=True)
                        pose=(*core.field_to_layout(self.e.pose.x,self.e.pose.y),180+self.e.pose.yaw)
                        # 与PC运行保护一致，实际扫掠保留1mm余量用于测量/跟踪误差。
                        self.assertIsNone(actual_scene.moving_pose_reason(previous,pose))
                        previous=pose
                        status=self.e.status()
                        self.assertNotIn(status[1],(7,8),status)
                        if status[1]==6:break
                    else:self.fail('正常回库未在运行期限内DONE')
                    target=core.layout_to_field(*core.ZONE_CENTER[zone])
                    self.assertLess(math.dist((self.e.pose.x,self.e.pose.y),target),1)
                    self.assertLess(abs(wrap(self.e.pose.yaw)),1)

    def test_ready_does_not_move_until_run_and_no_stop_at_samples(self):
        self.assertEqual(self.e.upload(straight())[1],3)
        for _ in range(15):self.assertEqual(self.e.step(),0)
        self.e.send('TRUN=17');self.assertEqual(self.e.step(),2)
        actions=[]
        for _ in range(500):
            actions.append(self.e.step(move=True))
            if self.e.status()[1] in (6,8):break
        self.assertEqual(self.e.status()[1],6,self.e.status())
        self.assertEqual(actions.count(2),1)
        self.assertLess(abs(self.e.pose.y-1000),5)

    def test_crc_count_sequence_and_shape_fail_closed(self):
        self.assertEqual(self.e.upload(straight(),crc=123)[-1],4)
        self.e=Engine(self.dll);self.e.send('TBEGIN=17,51,1,00000000,10')
        self.e.send('TPOINT=17,1,0,200,0,200,0');self.assertEqual(self.e.status()[-1],3)
        self.e=Engine(self.dll);self.e.send('TBEGIN=17,51,1,00000000,10')
        self.e.send('TCOMMIT=17');self.assertEqual(self.e.status()[-1],4)
        for mutate in (lambda p:p.__setitem__(0,(0,0,0,0,STOP|WAIT)),
                       lambda p:p.__setitem__(2,(0,900,900,0,0)),
                       lambda p:p.__setitem__(50,(*p[-1][:4],WAIT))):
            self.e=Engine(self.dll);points=straight();mutate(points)
            self.assertEqual(self.e.upload(points)[-1],7)
            self.assertFalse(self.dll.Traj_OutputAllowed())

    def test_duplicates_cannot_change_content_or_replay_start(self):
        self.e.send('TBEGIN=17,51,1,00000000,10')
        self.e.send('TPOINT=17,0,0,0,0,0,0');self.e.send('TPOINT=17,0,0,0,0,0,0')
        self.assertEqual(self.e.status()[2],1)
        self.e.send('TPOINT=17,0,0,1,0,0,0');self.assertEqual(self.e.status()[-1],3)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=999')
        self.assertEqual(self.e.step(),0);self.assertEqual(self.e.status()[1],3)

    def test_stop_disabled_stale_jump_and_wrong_start(self):
        for reason in (15,16,17,19):
            self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=17');self.e.step()
            self.dll.Traj_Cancel(reason);self.assertFalse(self.dll.Traj_OutputAllowed())
            self.assertEqual(self.e.step(),2);self.assertEqual(self.e.status()[1],7)
            self.e.send('TRUN=17');self.assertEqual(self.e.step(),0)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.pose.y=6;self.e.send('TRUN=17')
        self.e.step();self.assertEqual(self.e.status()[-1],9)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=17');self.e.step()
        self.e.step(allowed=0);self.assertEqual(self.e.status()[-1],5)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=17');self.e.step()
        self.e.pose.y=51;self.e.step();self.assertEqual(self.e.status()[-1],11)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=17');self.e.step()
        self.e.now+=201
        self.dll.Traj_Step(self.e.now,ctypes.byref(self.e.pose),1,self.e.velocity)
        self.assertEqual(self.e.status()[-1],8)

    def test_watchdogs_and_deviation(self):
        self.e.send('TBEGIN=17,51,1,00000000,10');self.e.now+=3001;self.e.step()
        self.assertEqual(self.e.status()[-1],6)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=17');self.e.step()
        for _ in range(252):self.e.step()
        self.assertEqual(self.e.status()[-1],14)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=17');self.e.step()
        self.e.pose.x=9.1;self.e.step();self.assertEqual(self.e.status()[-1],13)
        self.e=Engine(self.dll);self.e.upload(straight());self.e.send('TRUN=17');self.e.step()
        self.e.now+=101;self.e.pose.pose_tick=self.e.now
        self.dll.Traj_Step(self.e.now,ctypes.byref(self.e.pose),1,self.e.velocity)
        self.assertEqual(self.e.status()[-1],10)

    def test_wait_resume_keeps_batch_and_wrong_gate_does_not_release(self):
        self.e.upload(straight(wait=True));self.e.send('TRUN=17');self.e.step()
        for _ in range(300):
            self.e.step(move=True)
            if self.e.status()[1]==5:break
        self.assertEqual(self.e.status()[1:4],(5,51,10))
        self.e.send('TRESUME=17,11');self.e.step();self.assertEqual(self.e.status()[1],5)
        self.e.send('TRUN=17');self.e.step();self.assertEqual(self.e.status()[1],5)
        self.e.send('TRESUME=17,10');self.e.step();self.assertEqual(self.e.status()[1],4)
        self.e=Engine(self.dll);self.e.upload(straight(wait=True));self.e.send('TRUN=17');self.e.step()
        for _ in range(300):
            self.e.step(move=True)
            if self.e.status()[1]==5:break
        self.e.pose.x+=5.1;self.e.step();self.assertEqual(self.e.status()[-1],12)

    def test_entire_run_deadline_includes_station_wait(self):
        self.e.upload(straight(wait=True));self.e.send('TRUN=17');self.e.step()
        for _ in range(300):
            self.e.step(move=True)
            if self.e.status()[1]==5:break
        self.assertEqual(self.e.status()[1],5)
        for _ in range(9001):self.e.step()
        self.assertEqual(self.e.status()[1],8);self.assertEqual(self.e.status()[-1],10)

    def test_future_crane_completion_requires_current_wait_and_cannot_revive_cancelled_batch(self):
        self.e.upload(straight(wait=True));self.e.send('TRUN=17');self.e.step()
        self.assertEqual(self.dll.Traj_CompleteStation(17,10),0)
        for _ in range(300):
            self.e.step(move=True)
            if self.e.status()[1]==5:break
        self.assertEqual(self.e.status()[1],5)
        self.assertEqual(self.dll.Traj_CompleteStation(18,10),0)
        self.assertEqual(self.dll.Traj_CompleteStation(17,11),0)
        self.assertEqual(self.dll.Traj_CompleteStation(17,10),1)
        self.e.step();self.assertEqual(self.e.status()[1],4)
        self.dll.Traj_Cancel(15)
        self.assertEqual(self.dll.Traj_CompleteStation(17,10),0)
        self.e.step();self.assertEqual(self.e.status()[1],7)

    def test_rotation_holds_center_with_wheel_lag_and_turn_induced_drift(self):
        # 注入转头引起的中心漂移与80ms轮速响应，不能仅按指令理想积分。
        points=[(0,0,0,0,STOP),(0,0,0,9000,STOP|ROTATE)]
        self.assertEqual(self.e.upload(points)[1],3)
        self.e.send('TRUN=17');self.e.step()
        actual=[0.0,0.0,0.0];old_omega=0.0;held=False;peak_gap=0.0
        for _ in range(600):
            action=self.e.step()
            command=list(self.e.velocity) if action==1 else [0.0,0.0,0.0]
            self.assertLessEqual(abs(command[2]),60.01)
            if action==1:
                self.assertLessEqual(abs(command[2]-old_omega),3.61)
                held |= abs(command[0])+abs(command[1])>0.1
            old_omega=command[2]
            for i in range(3):actual[i]+=(command[i]-actual[i])*.2
            turn=actual[2]*.02
            self.e.pose.x+=actual[0]*.02+.33*turn
            self.e.pose.y+=actual[1]*.02+.47*turn
            self.e.pose.yaw=wrap(self.e.pose.yaw+turn)
            peak_gap=max(peak_gap,math.hypot(self.e.pose.x,self.e.pose.y))
            status=self.e.status()
            if status[1] in (6,8):break
        self.assertEqual(status[1],6,status)
        self.assertTrue(held,'转头必须提供X/Y保持指令')
        self.assertLess(peak_gap,8.0)
        self.assertLess(math.hypot(self.e.pose.x,self.e.pose.y),5)
        self.assertLess(abs(wrap(self.e.pose.yaw-90)),1)

    def test_rotation_position_fault_still_stops_outside_margin(self):
        self.e.upload([(0,0,0,0,STOP),(0,0,0,9000,STOP|ROTATE)])
        self.e.send('TRUN=17');self.e.step()
        for _ in range(20):self.e.step()
        self.e.pose.x=9
        self.assertEqual(self.e.step(),2)
        self.assertEqual(self.e.status()[-1],13)
        self.assertFalse(self.dll.Traj_OutputAllowed())
        self.assertEqual(list(self.e.velocity),[0,0,0])

    def test_screenshot_click_fallback_completes_with_turn_drift_and_wheel_lag(self):
        import navigation_planner as nav
        data=nav.load_map(QT/'navigation_map.json')
        scene=competition.collision_scene(data,10)
        start=(2250,2250);goal=(1200,1200)
        route=core.plan_path(start,goal,grid=100,pad=10,footprint=(280,260,180),
            start_heading_deg=180,goal_heading_deg=180,rects=scene.rects,circles=scene.circles,
            bounds=scene.bounds,drivable_polygons=data['drivable_polygons'],turn_penalty_mm=180)
        self.assertTrue(route['ok'],route['reason'])
        batch=make_path_batch(route,start,180,180,(0,0,0),scene,data,token=17)
        self.assertEqual(batch['execution_mode'],'STOP_TURN_FALLBACK')
        turns=[(i,p) for i,p in enumerate(batch['points']) if p[4]&ROTATE]
        self.assertEqual(turns[0][1],(1500,2500,4000,9000,STOP|ROTATE))
        self.assertEqual(self.e.upload(batch['points'])[1],3)
        self.e.send('TRUN=17');self.e.step()
        actual=[0.0,0.0,0.0];previous=None;rotated=False
        for _ in range(3000):
            action=self.e.step()
            status=self.e.status()
            if status[1] in (6,8):break
            command=list(self.e.velocity) if action==1 else [0.0,0.0,0.0]
            for i in range(3):actual[i]+=(command[i]-actual[i])*.2
            turn=actual[2]*.02;rotated |= abs(turn)>.01
            self.e.pose.x+=actual[0]*.02+.33*turn
            self.e.pose.y+=actual[1]*.02+.47*turn
            self.e.pose.yaw=wrap(self.e.pose.yaw+turn)
            pose=(*core.field_to_layout(self.e.pose.x,self.e.pose.y),self.e.pose.yaw-180)
            self.assertIsNone(scene.pose_reason(*pose))
            if previous:self.assertIsNone(scene.moving_pose_reason(previous,pose))
            previous=pose
        self.assertEqual(status[1],6,status)
        self.assertTrue(rotated)
        self.assertLess(math.hypot(self.e.pose.x-1050,self.e.pose.y-1050),5)
        self.assertLess(abs(self.e.pose.yaw),1)

    def test_terminal_rotation_and_wrapped_heading(self):
        points=[(0,0,0,0,0),(0,200,200,0,STOP),(0,200,200,9000,STOP|ROTATE)]
        self.assertEqual(self.e.upload(points)[1],3)
        self.e.send('TRUN=17');self.e.step()
        for _ in range(500):
            self.e.step(move=True)
            if self.e.status()[1] in (6,8):break
        self.assertEqual(self.e.status()[1],6,self.e.status());self.assertLess(abs(self.e.pose.yaw-90),1)
        self.e=Engine(self.dll)
        self.e.upload([(0,0,0,17900,0),(0,200,200,-17900,STOP)])
        self.e.pose.yaw=179;self.e.send('TRUN=17');self.e.step();self.e.step(move=True)
        self.assertLess(abs(self.e.velocity[2]),40)

    def test_clicked_path_upload_once_initial_rotation_and_local_execution(self):
        for continuous in (False,True):
            with self.subTest(continuous=continuous):
                self.e=Engine(self.dll)
                data=competition.load_profile();data['geometry_verified']=True
                scene=competition.collision_scene(data,10,((1200,1200),(297,294)))
                start=(2100,2100);goal=(2100,1500)
                route=core.plan_path(start,goal,grid=100,pad=10,footprint=(280,260,180),
                    start_heading_deg=180,goal_heading_deg=180,rects=scene.rects,circles=scene.circles,
                    bounds=scene.bounds,drivable_polygons=data['drivable_polygons'],geometry_verified=True)
                self.assertTrue(route['ok'],route['reason'])
                if continuous:
                    primitives=[dict(kind='LINE',start=start,end=goal)]
                    route.update(core.generate_trajectory(primitives,scene),smoothed_primitives=primitives)
                batch=make_path_batch(route,start,180,180,(0,0,0),scene,data,token=17)
                self.e.pose.x=self.e.pose.y=150
                sent=[]
                def transmit(line):sent.append(line);self.e.send(line)
                job=BatchUploader(batch,transmit,clock=lambda:self.e.now/1000)
                job.start();previous=None
                for _ in range(2000):
                    reply=self.e.reply()
                    if reply:job.handle_reply(reply)
                    job.tick()
                    if not job.active:break
                    self.e.step(move=True)
                    pose=(*core.field_to_layout(self.e.pose.x,self.e.pose.y),self.e.pose.yaw-180)
                    self.assertIsNone(scene.pose_reason(*pose))
                    if previous:self.assertIsNone(scene.moving_pose_reason(previous,pose))
                    previous=pose
                self.assertEqual(job.state,'DONE',(job.reason,self.e.status()))
                self.assertLess(math.dist((self.e.pose.x,self.e.pose.y),core.layout_to_field(*goal)),5)
                self.assertLess(abs(self.e.pose.yaw),1)
                self.assertEqual(sum(s.startswith('TPOINT=') for s in sent),batch['point_count'])
                self.assertEqual(sum(s.startswith('TRUN=') for s in sent),1)
                self.assertFalse(any(s.startswith(('GOTO=','TRESUME=')) for s in sent))

    def test_initial_stop_requires_fresh_settled_pose_before_rotation(self):
        self.assertEqual(self.e.upload([(0,0,0,0,STOP),(0,0,0,9000,STOP|ROTATE)])[1],3)
        self.e.send('TRUN=17');self.e.step()
        for _ in range(9):
            self.assertEqual(self.e.step(),1)
            self.assertEqual(tuple(self.e.velocity),(0,0,0))
        self.assertEqual(self.e.step(),2)
        self.assertEqual(self.e.step(),1)
        self.assertGreater(self.e.velocity[2],0)

    def test_entire_competition_upload_then_local_execution_all_obstacle_cases(self):
        outcomes=[]
        for label,obstacles in [('none',()),('one',((700,1200),)),('four',competition.DEMO_OBSTACLES),
                                ('screenshot',((1200,1200),(297,294)))]:
            for zone in (1,2):
                with self.subTest(scene=label,zone=zone):
                    self.e=Engine(self.dll)
                    data=competition.load_profile();data['geometry_verified']=True
                    match=competition.compile_match(data,zone=zone,sim_obstacles=obstacles)
                    scene=competition.collision_scene(data,10,obstacles)
                    mapping=(0,0,0) if zone==1 else (2100,0,0)
                    batch=make_match_batch(match,mapping,scene,token=17)
                    sent=[]
                    def transmit(line):sent.append(line);self.e.send(line)
                    job=BatchUploader(batch,transmit,clock=lambda:self.e.now/1000)
                    job.start();stopped=set();previous=None
                    for _ in range(10000):
                        text=self.e.reply()
                        if text:job.handle_reply(text)
                        job.tick()
                        self.assertNotEqual(job.state,'WAITING','自动跑图不得要求继续信号')
                        if not job.active:break
                        action=self.e.step(move=True)
                        if action==2:
                            status=self.e.status()
                            job.handle_reply('TSTAT '+' '.join(map(str,status)))
                            index=status[3]
                            if index in batch['stations']:
                                stopped.add(index)
                                q=batch['points'][index]
                                self.assertLess(math.dist((self.e.pose.x,self.e.pose.y),(q[0]/10,q[1]/10)),5)
                                self.assertLess(abs(wrap(self.e.pose.yaw-q[3]/100)),1)
                        if job.state in ('RUNNING','WAITING','RESUMING'):
                            pose=(*core.field_to_layout(mapping[0]+self.e.pose.x,self.e.pose.y),self.e.pose.yaw-180)
                            self.assertIsNone(scene.pose_reason(*pose),(label,zone,pose))
                            if previous:self.assertIsNone(scene.moving_pose_reason(previous,pose))
                            previous=pose
                    self.assertEqual(job.state,'DONE',(label,zone,job.state,job.reason,self.e.status()))
                    self.assertEqual(stopped,set(batch['stations']))
                    point_positions=[i for i,c in enumerate(sent) if c.startswith('TPOINT=')]
                    run_positions=[i for i,c in enumerate(sent) if c.startswith('TRUN=')]
                    self.assertEqual(len(point_positions),len(batch['points']))
                    self.assertEqual(len(run_positions),1);self.assertLess(max(point_positions),run_positions[0])
                    self.assertFalse(any(c.startswith('TRESUME=') for c in sent))
                    outcomes.append((label,zone,batch['point_count'],round((self.e.now-1000)/1000,2)))
        print('C整批回放(含离线上传时间/停稳，不是实车计时)：',outcomes)


if __name__=='__main__':unittest.main(verbosity=2)
