"""Fixed-center turns for coordinate programs; geometry and controller replay required."""
import copy
import math
import time
import navigation_planner as nav
from mecanum_geometry import WHEEL_GEOMETRY,ROTATION_LEVER_MM

PIVOT_ACCEL_DEG_S2=180.0
PIVOT_POSITION_GATE_MM=2.0
PIVOT_YAW_GATE_DEG=1.0
WHEEL_PIVOT_JOIN_MM_S=80.0

def wrap(angle):
    return (angle+180)%360-180

def rotate(vector,angle):
    c,s=math.cos(angle),math.sin(angle)
    return c*vector[0]-s*vector[1],s*vector[0]+c*vector[1]

def geometry(start,end,center,start_yaw,end_yaw):
    center=nav.point2(center,'旋转圆心')
    radius=math.dist(start,center)
    delta=wrap(end_yaw-start_yaw)
    if not 40<=radius<=2000 or not .01<abs(delta)<=120:
        raise ValueError('指定圆心转弯半径须40..2000mm，转角须0..120deg')
    offset=rotate((start[0]-center[0],start[1]-center[1]),math.radians(delta))
    if math.dist((center[0]+offset[0],center[1]+offset[1]),end)>.5:
        raise ValueError('指定圆心转弯的端点/车头角度不一致')
    return radius,delta

def ideal_sweep(scene,entry,exit,center,yaw_in,yaw_out,cancelled):
    radius,delta=geometry(entry,exit,center,yaw_in,yaw_out)
    steps=max(1,math.ceil(abs(delta)/5))
    previous=(*entry,yaw_in)
    for i in range(1,steps+1):
        if cancelled():
            raise ValueError('指定圆心规划已取消')
        angle=delta*i/steps
        offset=rotate((entry[0]-center[0],entry[1]-center[1]),math.radians(angle))
        pose=(center[0]+offset[0],center[1]+offset[1],yaw_in+angle)
        why=scene.arc_interval_reason(previous,pose,center)
        if why:
            raise ValueError('指定圆心理想车体连续扫掠：'+why)
        previous=pose
    return radius,delta

def wheel_geometry(scene):
    """Wheel-center dimensions are separate from the collision rectangle."""
    values=getattr(scene,'wheel_geometry',None)
    if values is None:
        values=copy.deepcopy(WHEEL_GEOMETRY)
    base=nav.finite_number(values['wheelbase_mm'],'麦轮前后轴距')
    track=nav.finite_number(values['track_mm'],'麦轮左右轮距')
    if not 40<=base<=scene.footprint.length_mm or not 40<=track<=scene.footprint.width_mm:
        raise ValueError('麦轮轮心轴距/轮距须在车体矩形内')
    if abs((base+track)/2-ROTATION_LEVER_MM)>.01:
        raise ValueError('轮心半轴距+半轮距须与当前底盘旋转臂长一致，请同步标定底盘')
    return dict(wheelbase_mm=base,track_mm=track,verified=bool(values.get('verified',False)))

def wheel_anchor(entry,yaw,dimensions,wheel):
    if wheel not in ('FL','FR','BL','BR'):raise ValueError('支点须为FL/FR/BL/BR麦轮')
    front=dimensions['wheelbase_mm']/2*(1 if wheel[0]=='F' else -1)
    left=dimensions['track_mm']/2*(1 if wheel[1]=='L' else -1)
    dx,dy=rotate((front,left),math.radians(yaw))
    return entry[0]+dx,entry[1]+dy

def corner_trial(program,index,dimensions,scene,cancelled):
    rows=program['waypoints'];a,b,c=rows[index-1:index+2]
    before=(a['x_mm'],a['y_mm']);corner=(b['x_mm'],b['y_mm']);after=(c['x_mm'],c['y_mm'])
    n,m=math.dist(before,corner),math.dist(corner,after)
    front,left=dimensions['wheelbase_mm']/2,dimensions['track_mm']/2
    if n<front+left+10 or m<abs(left-front)+10:
        raise ValueError('麦轮支点转弯前后直线空间不足')
    u=((corner[0]-before[0])/n,(corner[1]-before[1])/n)
    v=((after[0]-corner[0])/m,(after[1]-corner[1])/m)
    if abs(u[0]*v[0]+u[1]*v[1])>1e-6:
        raise ValueError('当前指定圆心候选只处理直角弯')
    delta=90 if u[0]*v[1]-u[1]*v[0]>0 else -90
    incoming=b.get('travel_yaw_deg',b['layout_yaw_deg'])
    outgoing=c.get('travel_yaw_deg',c['layout_yaw_deg'])
    tangent=math.degrees(math.atan2(u[1],u[0]))
    if min(abs(wrap(incoming-tangent)),abs(wrap(incoming-tangent-180)))>2 or abs(wrap(outgoing-incoming-delta))>2:
        raise ValueError('车头策略与指定圆心几何转角不一致')
    forward=abs(wrap(incoming-tangent))<=2
    wheel=('F' if forward else 'B')+('L' if (delta>0)==forward else 'R')
    entry=(corner[0]-u[0]*(front+left),corner[1]-u[1]*(front+left))
    center=wheel_anchor(entry,incoming,dimensions,wheel)
    offset=rotate((entry[0]-center[0],entry[1]-center[1]),math.radians(delta))
    exit=(center[0]+offset[0],center[1]+offset[1])
    radius=math.dist(entry,center)
    ideal_sweep(scene,entry,exit,center,incoming,incoming+delta,cancelled)
    result=copy.deepcopy(program)
    # 50Hz下80mm/s每帧约1.6mm；过小的通过范围会漏过出口并回头纠偏。
    # 入弯1mm、出弯2mm均须满足原1deg航向门，并完整预演车身扫掠。
    first=dict(x_mm=entry[0],y_mm=entry[1],layout_yaw_deg=incoming,
               travel_yaw_deg=incoming,kind='PASS',pass_mm=1.0)
    last=dict(x_mm=exit[0],y_mm=exit[1],layout_yaw_deg=incoming+delta,
              travel_yaw_deg=incoming,kind='PASS',pass_mm=2.0,
              motion='WHEEL_PIVOT',pivot_wheel=wheel,pivot_center_mm=list(center))
    result['waypoints'][index:index+1]=[first,last]
    result['schema_version']=3
    result['wheel_geometry']=copy.deepcopy(dimensions)
    return result,dict(corner_mm=corner,entry_mm=entry,exit_mm=exit,center_mm=center,
                       radius_mm=radius,angle_deg=delta,entry_yaw_deg=incoming,exit_yaw_deg=incoming+delta,
                       pivot_wheel=wheel,wheel_geometry=copy.deepcopy(dimensions))

def outer_corner(point,bounds):
    x,y=point;x1,y1,x2,y2=bounds
    band=min(450.0,(x2-x1)/4,(y2-y1)/4)
    return (x<=x1+band or x>=x2-band) and (y<=y1+band or y>=y2-band)

def select_turns(result,scene,cancelled,deadline=None):
    """Prefer verified turns at the four outer corners; retain baseline on failure.

    For other corners only accept a fully replayed faster candidate. Tests and
    runtime use the same controller; ideal centerline clearance never suffices.
    """
    from coordinate_navigation import CoordinateTracker,replay,_replay_result,OptimizationDeadline
    if not result.get('waypoint_program'):
        return result
    program=result['waypoint_program'];rows=program['waypoints']
    if any(row.get('motion') in ('PIVOT','WHEEL_PIVOT') for row in rows):
        return result
    indices=[]
    for i in range(1,len(rows)-1):
        a,b,c=rows[i-1:i+2]
        u=(b['x_mm']-a['x_mm'],b['y_mm']-a['y_mm']);v=(c['x_mm']-b['x_mm'],c['y_mm']-b['y_mm'])
        if math.hypot(*u)>nav.EPS and math.hypot(*v)>nav.EPS and abs(u[0]*v[0]+u[1]*v[1])<nav.EPS:
            indices.append(i)
    if not indices:
        return result
    try:dimensions=wheel_geometry(scene)
    except ValueError as exc:
        result['pivot_attempts']=[dict(accepted=False,reason=str(exc))]
        return result
    cutoff=min(deadline if deadline is not None else math.inf,time.monotonic()+.75)
    def interrupted():
        if cancelled():return True
        if time.monotonic()>=cutoff:raise OptimizationDeadline()
        return False
    selected=[];attempts=[];best_samples=result['trajectory'];best_elapsed=result['predicted_tracking_s']
    baseline_elapsed=best_elapsed;current=copy.deepcopy(program)
    try:
        # Reverse order keeps preceding indices stable after inserting entry/exit.
        for index in reversed(indices):
            row=current['waypoints'][index]
            preferred=(program.get('constraints',{}).get('turn_mode')=='WHEEL' or
                       outer_corner((row['x_mm'],row['y_mm']),scene.bounds))
            for radius in (math.hypot(dimensions['wheelbase_mm']/2,dimensions['track_mm']/2),):
                if interrupted():raise ValueError('指定圆心规划已取消')
                try:
                    trial,metadata=corner_trial(current,index,dimensions,scene,interrupted)
                    samples,elapsed=replay(trial,scene,interrupted)
                    from mecanum_planner import motion_metrics
                    if motion_metrics({'trajectory':samples})['longest_strafe_mm']>500:
                        raise ValueError('指定圆心候选超过500mm连续横移限制')
                    if preferred or elapsed<best_elapsed:
                        tracker=CoordinateTracker(trial)
                        trial.update(start=tracker.reference_at(0),goal=tracker.final_reference(),length_mm=tracker.length)
                        current,best_samples,best_elapsed=trial,samples,elapsed
                        metadata['preferred_outer_corner']=preferred
                        selected.append(metadata)
                        attempts.append(dict(corner_index=index,radius_mm=radius,accepted=True))
                        break
                    attempts.append(dict(corner_index=index,radius_mm=radius,accepted=False,reason='模型未提速'))
                except ValueError as exc:
                    if interrupted():raise
                    attempts.append(dict(corner_index=index,radius_mm=radius,accepted=False,reason=str(exc)))
    except OptimizationDeadline:
        pass
    if cancelled():raise ValueError('指定圆心规划已取消')
    if not selected:
        result['pivot_attempts']=attempts
        return result
    updated=_replay_result(current,best_samples,best_elapsed,result['skeleton_points'],
        result['search']['skeleton_cost_mm'],'PIVOT',result['start_heading_deg'],result['goal_heading_deg'],
        (result['search'].get('refined',False),result['search'].get('extended',False)))
    updated['search']=copy.deepcopy(result['search'])
    updated['optimality']=copy.deepcopy(result.get('optimality',{}))
    updated['optimality'].update(base_graph_proven=updated['optimality'].get('proven',False),proven=False,
        reason='selected skeleton extended with verified fixed-center turns; pivot variants not exhaustively optimized')
    updated.update(pivots=list(reversed(selected)),pivot_attempts=attempts,
                   pivot_baseline_tracking_s=baseline_elapsed,smoothing_status='WHEEL_PIVOT',
                   reason='指定麦轮支点旋转及完整车体实际扫掠通过')
    return updated
