"""普通坐标提前转向距离估计；角度增益沿用底盘轮缘速度单位。"""
import copy
import math
import time
from mecanum_geometry import ROTATION_LEVER_MM


def turn_time(angle, gate, angular_gain, maximum):
    saturation=maximum/angular_gain
    linear=max(0.0,(angle-max(gate,saturation))/maximum)
    tail=max(0.0,math.log(max(gate,min(angle,saturation))/gate)/angular_gain)
    return linear+tail


def approach_lead(control, travel_yaw, goal_yaw, pass_mm, span):
    """估计剩余P接近时间，使大转角尽量在最后30mm以前完成；仍须完整扫掠验证。"""
    configured=float(control['turn_lead_mm'])
    angle=abs((goal_yaw-travel_yaw+180)%360-180)
    gain=float(control['kpz'])
    maximum=math.degrees(float(control['zvmax'])/ROTATION_LEVER_MM)
    if angle<=30 or gain<=0 or maximum<=0 or span<1:
        return configured
    angular_gain=math.degrees(gain/ROTATION_LEVER_MM)
    slow=turn_time(angle,30,angular_gain,maximum)
    tail=turn_time(30,1,angular_gain,maximum)
    # 大于30deg阶段平移仍按现有5%限速；额外0.2s为估算余量，非实测延迟。
    exponent=min(6.0,max(control['kpx'],control['kpy'])*(.05*slow+tail+.2))
    finish=max(30.0,float(pass_mm)*1.5)
    return min(2000.0,max(configured,min(float(span),finish*math.exp(exponent))))


def anticipate_route(result,scene,cancelled=lambda:False,deadline=None):
    """在已安全路线中逐点提前转向；过早转头撞禁区时缩短窗口，预算内保留安全结果。"""
    from coordinate_navigation import CoordinateTracker,replay,_replay_result,OptimizationDeadline
    current=copy.deepcopy(result['waypoint_program'])
    samples=result['trajectory'];elapsed=result['predicted_tracking_s']
    cutoff=min(time.monotonic()+.5,deadline if deadline is not None else math.inf)
    def interrupted():
        if cancelled():return True
        if time.monotonic()>=cutoff:raise OptimizationDeadline()
        return False
    accepted=[];attempts=[]
    try:
        for i in range(1,len(current['waypoints'])):
            row=current['waypoints'][i];previous=current['waypoints'][i-1]
            if row.get('motion') in ('PIVOT','WHEEL_PIVOT'):continue
            baseline=row.get('turn_lead_mm',current['control']['turn_lead_mm'])
            span=math.dist((row['x_mm'],row['y_mm']),(previous['x_mm'],previous['y_mm']))
            requested=approach_lead(current['control'],row.get('travel_yaw_deg',row['layout_yaw_deg']),
                                    row['layout_yaw_deg'],row['pass_mm'],span)
            if requested<=baseline+.1:continue
            # 最先试估计窗口；局部几何不允许时依次尝试更晚、仍早于原值的窗口。
            for lead in (requested,(requested+baseline)/2,(requested+3*baseline)/4,(requested+7*baseline)/8):
                if interrupted():raise ValueError('提前转向规划已取消')
                trial=copy.deepcopy(current);trial['waypoints'][i]['turn_lead_mm']=round(lead,1)
                trial['waypoints'][i]['finish_heading_before_pass']=True
                try:
                    trial_samples,trial_elapsed=replay(trial,scene,interrupted)
                    from mecanum_planner import motion_metrics
                    if motion_metrics({'trajectory':trial_samples})['longest_strafe_mm']>500:
                        raise ValueError('提前转向后连续横移超过500mm')
                except ValueError as exc:
                    if cancelled():raise
                    attempts.append(dict(index=i,lead_mm=round(lead,1),accepted=False,reason=str(exc)))
                    continue
                current,samples,elapsed=trial,trial_samples,trial_elapsed
                accepted.append(dict(index=i,old_lead_mm=baseline,new_lead_mm=round(lead,1)))
                attempts.append(dict(index=i,lead_mm=round(lead,1),accepted=True))
                break
    except OptimizationDeadline:
        pass
    if cancelled():raise ValueError('提前转向规划已取消')
    if not accepted:
        result['approach_turn_attempts']=attempts
        return result
    tracker=CoordinateTracker(current)
    current.update(start=tracker.reference_at(0),goal=tracker.final_reference(),length_mm=tracker.length)
    updated=_replay_result(current,samples,elapsed,result['skeleton_points'],result['search']['skeleton_cost_mm'],
        'APPROACH',result['start_heading_deg'],result['goal_heading_deg'],
        (result['search'].get('refined',False),result['search'].get('extended',False)))
    for key in ('search','optimality','pivots','pivot_attempts','pivot_baseline_tracking_s','smoothing_status'):
        if key in result:updated[key]=copy.deepcopy(result[key])
    updated.setdefault('optimality',{}).update(base_graph_proven=updated.get('optimality',{}).get('proven',False),proven=False,
        reason='verified heading anticipation added; finite safe lead windows are not exhaustive')
    updated.update(approach_turns=accepted,approach_turn_attempts=attempts)
    return updated
