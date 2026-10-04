"""完整比赛整批上传：OPS坐标、整数点/CRC32、上传窗口、作业停靠与继续。"""
import math
import re
import secrets
import struct
import time
import zlib

import core
from trajectory import validate_trajectory, generate_trajectory

STOP, WAIT, ROTATE, ARC = 1, 2, 4, 8
POINT = struct.Struct('<iiIhH')
CAPACITY = 4096
STATE_NAMES = {0:'IDLE',1:'RECEIVING',2:'VERIFYING',3:'READY',4:'RUNNING',5:'WAITING',6:'DONE',7:'CANCELLED',8:'FAULT'}
ERRORS = {1:'上传头格式/容量错误',2:'点字段/范围错误',3:'点序号或重传内容错误',4:'点数或CRC不一致',
          5:'底盘未就绪/心跳过期',6:'上传中断超过3秒',7:'轨迹间隔/停靠/转头定义非法',8:'OPS定位过期/非法',
          9:'实际起点或车头未对齐',10:'180秒运行超时或控制调度超期',11:'OPS位置/航向跳变',
          12:'等待作业期间车辆移动',13:'车体偏差超过规划裕量',14:'5秒无进展',15:'STOP/其他运动接管',
          16:'上位机失联',17:'OPS会话变化',19:'调试串口接收异常'}


def wrap(angle):
    return (angle+180)%360-180


def make_match_batch(match, mapping, scene, *, token=None, cancelled=lambda: False,
                     station_mode='AUTO_ROUTE'):
    """默认自动跑图：站点仅STOP，停稳后MCU继续；未来真实机构可显式使用WAIT。"""
    if station_mode not in ('AUTO_ROUTE','WAIT_FOR_ACTION'):
        raise ValueError('未知站点执行模式')
    # 用户采用当前比赛地图；几何核实标志仅作元数据，不拦截上传。
    ox, oy, theta = map(float, mapping)
    if not all(math.isfinite(v) for v in (ox,oy,theta)):
        raise ValueError('坐标标定非法')
    if not 5 <= math.floor(scene.pad) <= 100:
        raise ValueError('实机整批安全裕量必须在5到100mm内')
    c,s = math.cos(math.radians(theta)),math.sin(math.radians(theta))
    field_points, stations, records = [], {}, []
    station = 0.0
    home = core.layout_to_field(*match['home'])
    current = (*home, -90-match['start_yaw'])

    def append(p, flags=0, tangent=None):
        nonlocal current
        if cancelled():
            raise ValueError('实机比赛预检已取消')
        x,y,body_yaw = p
        q = (round((c*(x-ox)-s*(y-oy))*10),round((s*(x-ox)+c*(y-oy))*10),
             round(station*10),round(wrap(90-theta-body_yaw)*100),flags)
        if q[3] == 18000:
            q = (*q[:3],-18000,q[4])
        if not (-100000<=q[0]<=100000 and -100000<=q[1]<=100000 and 0<=q[2]<=10000000):
            raise ValueError('轨迹超过STM32坐标/弧长范围')
        if records and q[:4] == records[-1][:4]:
            # 转角量化后为零时没有新的ROTATE边，首点尤其只能保留STOP。
            old = records[-1];records[-1]=(*old[:4],old[4]|(flags & ~ROTATE))
        elif records and q[2] == records[-1][2] and not flags&ROTATE:
            # 弧角采样与20mm网格可能量化为相同s；合并近点，不能上传零弧长平移。
            old=records[-1]
            if math.dist(old[:2],q[:2])>2 or abs(wrap((q[3]-old[3])/100))>0.1:
                raise ValueError('相同量化弧长包含非零运动，拒绝批次')
            records[-1]=(*old[:4],old[4]|flags)
        else:
            records.append(q);field_points.append(dict(x_mm=x,y_mm=y,field_yaw_deg=body_yaw,s_mm=station,
                segment_type='ROTATE' if flags&ROTATE else 'ARC' if flags&ARC else 'LINE',
                tangent_yaw_deg=body_yaw if tangent is None else tangent))
        current = tuple(p)
        if len(records)>CAPACITY:
            raise ValueError('整轮轨迹超过4096点，不能截断或边运行边补点')

    append(current)
    for stage in match['stages']:
        if cancelled():
            raise ValueError('实机比赛预检已取消')
        if stage['kind']=='MANEUVER':
            target = (*core.layout_to_field(*stage['target']), -90-stage['yaw'])
            length = math.dist(current[:2], target[:2])
            turn = abs(wrap(target[2]-current[2]))
            if length<1e-6:
                if turn>1e-6:
                    old=records[-1];records[-1]=(*old[:4],old[4]|STOP)
                    append(target,STOP|ROTATE)
            else:
                if turn>1e-6:
                    raise ValueError('实机MANEUVER不能同时平移和转头')
                start, begin = current, station
                n=math.ceil(length/20)
                for i in range(1,n+1):
                    station=begin+length*i/n
                    append((start[0]+(target[0]-start[0])*i/n,start[1]+(target[1]-start[1])*i/n,target[2]),
                           STOP if i==n else 0,math.degrees(math.atan2(target[1]-start[1],target[0]-start[0])))
        elif stage['kind']=='TRAVEL':
            r=stage['route']
            checked=validate_trajectory(r['trajectory'],r['smoothed_primitives'],scene)
            if not checked['ok']:
                raise ValueError('实机轨迹复检失败：'+checked['reason'])
            dense = generate_trajectory(r['smoothed_primitives'],scene,
                max_arc_angle_deg=3,interrupted=cancelled)
            if not dense['trajectory_safe']:
                raise ValueError('实机圆弧细采样失败：'+dense['trajectory_reason'])
            first=r['trajectory'][0]
            if math.dist(current[:2],(first['x_mm'],first['y_mm']))>0.1 or abs(wrap(current[2]-first['field_yaw_deg']))>0.01:
                raise ValueError('比赛路段缺少出发平移/转头连接')
            begin=station
            for p in dense['trajectory'][1:]:
                station=begin+p['s_mm']
                append((p['x_mm'],p['y_mm'],p['field_yaw_deg']),ARC if p['segment_type']=='ARC' else 0,
                       p.get('tangent_yaw_deg',p['field_yaw_deg']))
            old=records[-1];records[-1]=(*old[:4],old[4]|STOP)
        else:
            old=records[-1];records[-1]=(*old[:4],old[4]|STOP|(WAIT if station_mode=='WAIT_FOR_ACTION' else 0))
            label=('二维码站跑图停靠（预设任务码：'+match['task_code']+'）') if stage['kind']=='SCAN' else stage['label']
            stations.setdefault(len(records)-1,[]).append(label)
    if len(records)<2 or records[-1][4]&WAIT:
        raise ValueError('比赛缺少最终回家STOP')
    records[-1]=(*records[-1][:4],records[-1][4]|STOP)
    # 检查传输量化后的实际矩形和每对点的连续扫掠，不能只校验浮点原轨迹。
    previous=None
    for i,q in enumerate(records):
        if cancelled():
            raise ValueError('实机比赛预检已取消')
        x,y=q[0]/10,q[1]/10
        fx,fy=ox+c*x+s*y,oy-s*x+c*y
        pose=(*core.field_to_layout(fx,fy),q[3]/100+theta-180)
        reason=scene.pose_reason(*pose)
        if previous is not None and not reason:
            reason=scene.turn_reason(previous[:2],previous[2],pose[2]) if q[4]&ROTATE else scene.moving_pose_reason(previous,pose)
        if reason:
            raise ValueError('整批量化点%d车体不安全：%s'%(i,reason))
        previous=pose
    payload=b''.join(POINT.pack(*q) for q in records)
    return dict(schema_version=1,kind='STM32_PRELOADED_MATCH',physical_motion_verified=False,
                station_mode=station_mode,station_settle_ms=200,stations=stations,
                point_fields=['x_0.1mm','y_0.1mm','s_0.1mm','ops_yaw_0.01deg','flags'],
                id=token or secrets.randbelow(0xFFFFFFFF)+1, points=records,
                waits=stations if station_mode=='WAIT_FOR_ACTION' else {},
                crc=zlib.crc32(payload)&0xFFFFFFFF, map_version=match['map_snapshot']['map_version'],
                margin_mm=math.floor(scene.pad), mapping=(ox,oy,theta), field_points=field_points,
                match=match, point_count=len(records), length_mm=station)


def make_path_batch(route, start, start_yaw, goal_yaw, mapping, scene, map_snapshot,
                    *, token=None, cancelled=lambda: False):
    """点击目标整批执行；航向均为布局角。连续路径优先，fallback显式停转。"""
    if not route.get('ok') or not route.get('axis_matched') or not route.get('execution_safe'):
        raise ValueError('点击路径未通过完整车体执行检查')
    stages=[]
    continuous=bool(route.get('trajectory_safe') and route.get('trajectory_continuous'))
    def maneuver(target, yaw, label):
        stages.append(dict(kind='MANEUVER',target=tuple(target),yaw=float(yaw),label=label))
    if continuous:
        first=route['trajectory'][0]
        if math.dist(core.layout_to_field(*start),(first['x_mm'],first['y_mm']))>0.1:
            raise ValueError('点击路径与实际规划起点不匹配')
        maneuver(start,-90-first['field_yaw_deg'],'出发车头对齐（整车转头复检）')
        stages.append(dict(kind='TRAVEL',route=route,label='连续避障路径'))
        maneuver(core.field_to_layout(route['trajectory'][-1]['x_mm'],route['trajectory'][-1]['y_mm']),
                 goal_yaw,'目标车头对齐并STOP')
    else:
        # 所有圆弧半径失败时绝不强行平滑；保留已校验台账的停靠/转头。
        for row in route.get('steps',[]):
            if row['kind']=='START': continue
            maneuver((row['to_x'],row['to_y']),row['heading_deg'],row['action'])
        if not stages: raise ValueError('点击路径没有可执行动作')
        maneuver(stages[-1]['target'],goal_yaw,'目标车头对齐并STOP')
    match=dict(home=tuple(start),start_yaw=float(start_yaw),map_snapshot=map_snapshot,
               task_code='',stages=stages)
    batch=make_match_batch(match,mapping,scene,token=token,cancelled=cancelled)
    batch.update(kind='STM32_POINT_PATH',execution_mode='CONTINUOUS' if continuous else 'STOP_TURN_FALLBACK',
                 fallback_reason='' if continuous else route.get('trajectory_reason','连续路径不可用，按台账停转'),
                 collision_snapshot=dict(frame_id='LAYOUT_MM',rects=list(scene.rects),circles=list(scene.circles),
                     bounds=scene.bounds,pad_mm=scene.pad,
                     footprint=dict(length_mm=scene.footprint.length_mm,width_mm=scene.footprint.width_mm),
                     drivable_polygons=map_snapshot['drivable_polygons']))
    return batch


class BatchUploader:
    """一次提交整批；分包ACK仅用于接收流控，所有点收到并校验后才发送TRUN。"""
    def __init__(self, batch, send, *, clock=time.monotonic, valid=lambda:True):
        self.batch,self.send,self.clock,self.valid=batch,send,clock,valid
        self.state='IDLE';self.reason='';self.received=self.sent=self.cursor=0
        self.progress=0;self.window=0;self.last_reply=self.last_query=clock()
        self.active=False;self.commands=[]

    def emit(self,line):
        if not self.valid():
            self.cancel('地图、参数或连接已变化');return
        self.commands.append(line);self.send(line)

    def start(self):
        if self.state!='IDLE':
            raise ValueError('整批上传不能重复启动')
        self.state='CAPS';self.active=True;self.last_reply=self.clock();self.emit('TCAPS')

    def cancel(self,reason='已取消'):
        if not self.active:
            return
        self.state='CANCELLED';self.reason=reason;self.active=False
        # 调用方清除排队点/启动命令并优先STOP；取消不调用emit，失效会话也要停车。
        self.send('TABORT=%d'%self.batch['id']);self.send('STOP')

    def _chunk(self):
        for i in range(self.sent,min(self.sent+self.window,len(self.batch['points']))):
            x,y,station,yaw,flags=self.batch['points'][i]
            self.emit('TPOINT=%d,%d,%d,%d,%d,%d,%d'%(self.batch['id'],i,x,y,yaw,station,flags))
            self.sent=i+1

    def handle_reply(self,text):
        if not self.active:
            return False
        text=text.removeprefix('固件文本: ').strip()
        caps=re.fullmatch(r'TCAPS (\d+) (\d+) (\d+)',text)
        if caps and self.state=='CAPS':
            version,capacity,self.window=map(int,caps.groups())
            if version!=1 or capacity<len(self.batch['points']) or not 1<=self.window<=3:
                self.cancel('STM32版本/容量/窗口不兼容');return True
            self.state='BEGIN';self.last_reply=self.clock()
            b=self.batch
            self.emit('TBEGIN=%d,%d,%d,%08X,%d'%(b['id'],len(b['points']),b['map_version'],b['crc'],b['margin_mm']))
            return True
        stat=re.fullmatch(r'TSTAT (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)',text)
        if not stat:
            return False
        job,state,received,cursor,progress,error=map(int,stat.groups())
        if job!=self.batch['id']:
            return False
        if not 0<=cursor<len(self.batch['points']) or not 0<=received<=self.sent or state not in STATE_NAMES:
            self.cancel('STM32状态字段不一致');return True
        if state in (1,2,3) and self.state in ('STARTING','RUNNING','WAITING','RESUMING'):
            return True
        if state in (2,3,4,5,6) and received!=len(self.batch['points']):
            self.cancel('未完整接收却报告可运行/完成');return True
        if state in (4,5) and self.state not in ('STARTING','RUNNING','WAITING','RESUMING'):
            self.cancel('未发送TRUN却报告车辆已运行');return True
        if progress/10>self.batch['length_mm']+0.2:
            self.cancel('STM32弧长进度超过批次');return True
        self.last_reply=self.clock();self.cursor=cursor;self.progress=progress/10
        if state in (7,8) or error:
            reason=ERRORS.get(error,str(error))
            if error==13 and cursor>0 and cursor+1<len(self.batch['points']) and self.batch['points'][cursor+1][4]&ROTATE:
                reason='原地转头中心偏差超限（允许%.1fmm）'%(self.batch['margin_mm']-2)
            self.cancel('STM32 %s：%s'%(STATE_NAMES[state],reason));return True
        if state==1 and self.state in ('BEGIN','UPLOADING'):
            if received<self.received:
                self.cancel('STM32接收进度回退');return True
            self.received=received
            if received==self.sent:
                if received==len(self.batch['points']):
                    self.state='VERIFYING';self.emit('TCOMMIT=%d'%job)
                else:
                    self.state='UPLOADING';self._chunk()
        elif state==3 and self.state=='VERIFYING':
            self.state='STARTING';self.emit('TRUN=%d'%job)
        elif state==4 and self.state in ('STARTING','RUNNING','WAITING','RESUMING'):
            self.state='RUNNING'
        elif state==5 and self.state in ('STARTING','RUNNING','WAITING','RESUMING'):
            if cursor not in self.batch['waits']:
                self.cancel('STM32停在未知作业点');return True
            if self.state!='RESUMING': self.state='WAITING'
        elif state==6:
            if self.state not in ('RUNNING','STARTING','RESUMING') or cursor!=len(self.batch['points'])-1 or received!=len(self.batch['points']):
                self.cancel('STM32完成状态不一致');return True
            self.state='DONE';self.active=False
        return True

    def resume(self):
        if self.state!='WAITING' or not self.active:
            raise ValueError('当前未在等待作业完成')
        self.state='RESUMING';self.emit('TRESUME=%d,%d'%(self.batch['id'],self.cursor))

    def tick(self):
        if not self.active:
            return
        if not self.valid():
            self.cancel('地图、参数或连接变化');return
        now=self.clock()
        if now-self.last_reply>3:
            self.cancel('3秒未收到轨迹状态；整批已停止');return
        if now-self.last_query>.75:
            self.last_query=now
            self.emit('TCAPS' if self.state=='CAPS' else 'TSTATUS=%d'%self.batch['id'])
