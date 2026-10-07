"""关键坐标整批协议：传端点/车头/通过门；密集预演仅保留在PC。"""
import copy
import math
import json
import secrets
import struct
import zlib

import core
from coordinate_navigation import CoordinateTracker, build_program, replay, wrap

POINT=struct.Struct('<iiIhHhHH')
PIVOT_POINT=struct.Struct('<iiIhHhHHii')
CAPACITY=2048
STOP,WAIT=1,2
PIVOT=16
WHEEL_PIVOT=32
APPROACH_HEADING=64


def make_coordinate_batch(match,mapping,scene,*,token=None,cancelled=lambda:False,station_mode='AUTO_ROUTE'):
    if station_mode not in ('AUTO_ROUTE','WAIT_FOR_ACTION'):
        raise ValueError('未知站点模式')
    parameters=copy.deepcopy(match.get('chassis_control'))
    if not isinstance(parameters,dict) or set(parameters)!=set(core.CHASSIS_DEFAULTS):
        raise ValueError('实机坐标必须使用已回读的完整底盘参数快照')
    parameters={k:round(float(v)*1000)/1000 for k,v in parameters.items()}
    ox,oy,theta=map(float,mapping)
    if not all(math.isfinite(v) for v in (ox,oy,theta)) or not 5<=math.floor(scene.pad)<=100:
        raise ValueError('坐标映射/安全裕量非法')
    c,s=math.cos(math.radians(theta)),math.sin(math.radians(theta))
    records=[];stations={};display=[];station=0.0
    replay_cache={};sweep_cache={}
    current=(*match['home'],float(match['start_yaw']))

    def encode(row,lead):
        nonlocal station
        fx,fy=core.layout_to_field(row['x_mm'],row['y_mm'])
        x,y=round((c*(fx-ox)-s*(fy-oy))*10),round((s*(fx-ox)+c*(fy-oy))*10)
        yaw=round(wrap(row['layout_yaw_deg']+180-theta)*100)
        travel=round(wrap(row.get('travel_yaw_deg',row['layout_yaw_deg'])+180-theta)*100)
        yaw=-18000 if yaw==18000 else yaw
        travel=-18000 if travel==18000 else travel
        if not -100000<=x<=100000 or not -100000<=y<=100000:
            raise ValueError('实机坐标超出±10000mm')
        center=None
        if row.get('motion') in ('PIVOT','WHEEL_PIVOT'):
            fx,fy=core.layout_to_field(*row['pivot_center_mm'])
            center=(round((c*(fx-ox)-s*(fy-oy))*10),round((s*(fx-ox)+c*(fy-oy))*10))
            if any(abs(v)>100000 for v in center):raise ValueError('实机旋转圆心超出±10000mm')
        if records:
            station+=(math.dist(records[-1][:2],center)*math.radians(abs(wrap((yaw-records[-1][3])/100)))/10
                      if center is not None else math.dist((x,y),records[-1][:2])/10)
        encoded=(x,y,round(station*10),yaw,(STOP if row['kind'] in ('START','STOP') else 0)|(PIVOT if center is not None else 0)|
                (WHEEL_PIVOT if row.get('motion')=='WHEEL_PIVOT' else 0)|
                (APPROACH_HEADING if row.get('finish_heading_before_pass') else 0),
                travel,round(row.get('pass_mm',0)*10),round(lead*10))
        return encoded+center if center is not None else encoded

    def decode(q,kind):
        x,y=q[0]/10,q[1]/10
        lx,ly=core.field_to_layout(ox+c*x+s*y,oy-s*x+c*y)
        row=dict(x_mm=lx,y_mm=ly,layout_yaw_deg=q[3]/100+theta-180,
                    travel_yaw_deg=q[5]/100+theta-180,pass_mm=q[6]/10,kind=kind)
        if q[4]&APPROACH_HEADING:row['finish_heading_before_pass']=True
        if q[4]&PIVOT and kind!='START':
            cx,cy=q[8]/10,q[9]/10
            row.update(motion='WHEEL_PIVOT' if q[4]&WHEEL_PIVOT else 'PIVOT',
                       pivot_center_mm=list(core.field_to_layout(ox+c*cx+s*cy,oy-s*cx+c*cy)))
        return row

    def add_program(program):
        nonlocal current
        if cancelled(): raise ValueError('实机坐标预检已取消')
        tracker=CoordinateTracker(program)
        if any(abs(tracker.control[k]-parameters[k])>0.00051 for k in parameters):
            raise ValueError('路段参数与整批底盘快照不一致')
        rows=program['waypoints'];start=rows[0]
        if math.dist(current[:2],(start['x_mm'],start['y_mm']))>0.1 or abs(wrap(current[2]-start['layout_yaw_deg']))>0.02:
            raise ValueError('关键坐标路段缺少姿态连接')
        begin=len(records)-1
        if not records:
            records.append(encode(rows[0],tracker.control['turn_lead_mm']));begin=0
        for row in rows[1:]:
            records.append(encode(row,row.get('turn_lead_mm',tracker.control['turn_lead_mm'])))
        if len(records)>CAPACITY: raise ValueError('整场关键坐标超过2048个，不能截断或运行中补发')
        quant=copy.deepcopy(program)
        section=records[begin:]
        quant['waypoints']=[decode(q,'START' if i==0 else 'STOP' if i==len(section)-1 or q[4]&STOP else 'PASS')
                            for i,q in enumerate(section)]
        for row,q in zip(quant['waypoints'],section):row['turn_lead_mm']=q[7]/10
        quant['waypoints'][0]['pass_mm']=0
        quant['control']=dict(parameters,turn_lead_mm=section[-1][7]/10)
        # 两批重复路段只预演一次，缓存仍绑定同一场景/映射/量化程序/底盘快照。
        identity=json.dumps(quant,sort_keys=True,separators=(',',':'),allow_nan=False)
        if identity not in replay_cache:
            replay_cache[identity]=replay(quant,scene,cancelled,sweep_cache)
        samples,_elapsed=replay_cache[identity]
        for a,b in zip(samples,samples[1:]):
            if math.hypot(b['x_mm']-a['x_mm'],b['y_mm']-a['y_mm'])>50 or abs(wrap(b['field_yaw_deg']-a['field_yaw_deg']))>15:
                raise ValueError('当前参数预演超过STM32单帧定位跳变门限')
        # 上一段实际模型停靠点与下一段量化起点之间也检查完整车体扫掠。
        if display:
            a,b=display[-1],samples[0]
            why=scene.moving_pose_reason((*core.field_to_layout(a['x_mm'],a['y_mm']),-90-a['field_yaw_deg']),
                                        (*core.field_to_layout(b['x_mm'],b['y_mm']),-90-b['field_yaw_deg']))
            if why: raise ValueError('量化坐标路段衔接不安全：'+why)
        display.extend(dict(p,s_mm=p['s_mm']+records[begin][2]/10) for p in samples)
        end=rows[-1];current=(end['x_mm'],end['y_mm'],end['layout_yaw_deg'])

    for stage in match['stages']:
        if cancelled(): raise ValueError('实机坐标预检已取消')
        if stage['kind']=='TRAVEL':
            add_program(stage['route']['waypoint_program'])
        elif stage['kind']=='MANEUVER':
            target=(*stage['target'],stage['yaw'])
            if math.dist(current[:2],target[:2])<1e-8 and abs(wrap(target[2]-current[2]))<1e-8:
                continue
            add_program(build_program([current[:2],target[:2]],current[2],goal_yaw=target[2],
                                      mode='FIXED',control=parameters))
        else:
            if not records: raise ValueError('站点之前缺少坐标运动')
            q=records[-1];records[-1]=(*q[:4],q[4]|STOP|(WAIT if station_mode=='WAIT_FOR_ACTION' else 0),*q[5:])
            stations.setdefault(len(records)-1,[]).append(stage['label'])
    if len(records)<2 or records[-1][4]&WAIT: raise ValueError('整批缺少最终回库STOP')
    if not records[-1][4]&STOP: raise ValueError('整批末点必须STOP')
    version=3 if any(q[4]&WHEEL_PIVOT for q in records) else 2 if any(q[4]&PIVOT for q in records) else 1
    if version>=2:records=[q if len(q)==10 else q+(0,0) for q in records]
    encoding=PIVOT_POINT if version>=2 else POINT
    field_points=[]
    for q in records:
        row=decode(q,'STOP' if q[4]&STOP else 'PASS')
        fx,fy=core.layout_to_field(row['x_mm'],row['y_mm'])
        field_points.append(dict(x_mm=fx,y_mm=fy,field_yaw_deg=-90-row['layout_yaw_deg'],s_mm=q[2]/10,
                                 segment_type='STOP' if q[4]&STOP else 'PIVOT' if q[4]&PIVOT else 'COORDINATE'))
    return dict(schema_version=2,kind='STM32_PRELOADED_MATCH',coordinate=True,execution_mode='COORDINATES',
                physical_motion_verified=False,turn_mode=match.get('turn_mode','WHEEL'),station_mode=station_mode,station_settle_ms=200,
                coordinate_version=version,pivot_count=sum(bool(q[4]&PIVOT) for q in records),
                required_coordinate_caps=8,
                stations=stations,waits=stations if station_mode=='WAIT_FOR_ACTION' else {},
                id=token or secrets.randbelow(0xFFFFFFFF)+1,points=records,point_count=len(records),
                point_fields=['x_0.1mm','y_0.1mm','s_0.1mm','ops_yaw_0.01deg','flags',
                              'travel_yaw_0.01deg','pass_0.1mm','turn_lead_0.1mm']+
                             (['pivot_center_x_0.1mm','pivot_center_y_0.1mm'] if version>=2 else []),
                crc=zlib.crc32(b''.join(encoding.pack(*q) for q in records))&0xFFFFFFFF,
                chassis_control=parameters,map_version=match['map_snapshot']['map_version'],margin_mm=math.floor(scene.pad),
                mapping=(ox,oy,theta),field_points=field_points,display_points=display,match=match,length_mm=records[-1][2]/10)


def make_coordinate_path_batch(route,start,start_yaw,goal_yaw,mapping,scene,map_snapshot,**kwargs):
    if not route.get('execution_safe') or not route.get('waypoint_program'):
        raise ValueError('实机坐标路径尚未通过完整预演')
    p=route['waypoint_program'];tracker=CoordinateTracker(p)
    initial=route.get('coordinate_prefix',p)
    if math.dist(start,(initial['waypoints'][0]['x_mm'],initial['waypoints'][0]['y_mm']))>0.1 or abs(wrap(start_yaw-initial['waypoints'][0]['layout_yaw_deg']))>0.02:
        raise ValueError('坐标路径实际起点不一致')
    final=route.get('coordinate_suffix',p)
    if goal_yaw is not None and abs(wrap(goal_yaw-final['waypoints'][-1]['layout_yaw_deg']))>0.02:
        raise ValueError('坐标路径目标车头不一致')
    match=dict(home=tuple(start),start_yaw=start_yaw,map_snapshot=map_snapshot,task_code='',turn_mode=route.get('turn_mode','WHEEL'),
               chassis_control={k:tracker.control[k] for k in core.CHASSIS_DEFAULTS},
               stages=[dict(kind='TRAVEL',route=route,label='关键坐标避障路径')])
    if route.get('coordinate_prefix'):
        match['stages'].insert(0,dict(kind='TRAVEL',route={'waypoint_program':route['coordinate_prefix'],'points':[start,start]},label='回库起点真实航向对齐'))
    if route.get('coordinate_suffix'):
        tail=route['coordinate_suffix']
        match['stages'].append(dict(kind='TRAVEL',route={'waypoint_program':tail,'points':[(r['x_mm'],r['y_mm']) for r in tail['waypoints']]},label='回库接近段'))
    batch=make_coordinate_batch(match,mapping,scene,**kwargs)
    batch.update(kind='STM32_POINT_PATH',fallback_reason='')
    return batch
