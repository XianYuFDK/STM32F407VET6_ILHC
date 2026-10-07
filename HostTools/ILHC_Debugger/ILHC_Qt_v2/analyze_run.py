"""离线运行分析：保留原始记录，另生成时间序列CSV和诊断JSON，不发命令。"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import re
from ops_diagnostics import parse_ops_text,summarize


def wrap(angle):
    return (angle+180)%360-180


def velocity(previous, frame):
    """优先设备时钟；重启、长空窗、乱序不连线求导，跨360度取最短角差。"""
    if previous is None:return None, None, None
    ticks=[f.get('device_tick_ms') for f in (previous,frame)]
    if frame.get('protocol')=='SIM' and all(f.get('simulation_time_s') is not None for f in (previous,frame)):
        dt=frame['simulation_time_s']-previous['simulation_time_s']
    else:
        dt=((ticks[1]-ticks[0])&0xffffffff)*.001 if all(t is not None for t in ticks) else frame['monotonic']-previous['monotonic']
    if not 0<dt<=.2:return None, None, None
    a,b=previous['values'],frame['values']
    return dt, math.hypot(b[0]-a[0],b[1]-a[1])*10/dt, wrap(b[2]-a[2])/dt


def analyze(folder, run_id=None):
    folder=Path(folder).resolve()
    if (folder/'metadata.json').exists():
        run_id=folder.name;folder=folder.parent.parent
    header=json.loads((folder/'session.json').read_text(encoding='utf-8'))
    if header.get('schema_version')!=1:raise ValueError('未知日志版本')
    report=dict(source=header['source'],physical_data=header['physical_data'],
        reference_limits='候选航向由点表、距离和最后一次TSTAT重建；不是固件实时目标回读。模型按进度对齐，不能证明实车控制器状态。',
        feedback_limits=header['firmware_feedback_limits'],runs=[])
    starts={};states={};previous={};series={};transport={};readbacks={};sim_bases={};controls={};heading_latches=set();ops_records={}
    pattern=re.compile(r'TSTAT (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)')
    control_pattern=re.compile(r'CCTRL '+r' '.join([r'(-?\d+)']*10))
    last_sequence=0;report['sequence_gaps']=0
    with (folder/'events.jsonl').open(encoding='utf-8') as f:
        for number,line in enumerate(f,1):
            try:e=json.loads(line)
            except json.JSONDecodeError:
                report.setdefault('incomplete_json_lines',[]).append(number);continue
            event=e['event'];rid=e.get('run_id')
            report['sequence_gaps']+=max(0,e['sequence']-last_sequence-1)
            last_sequence=e['sequence']
            if event=='TRANSPORT':transport=e
            if event=='PARAM':readbacks[e['name']]=e['value']
            if event=='SESSION_END':report['dropped_events']=e['dropped_events']
            if rid is None or (run_id is not None and run_id!=rid):continue
            if event=='RUN_START':
                path=folder/'runs'/rid
                plan=json.loads((path/'plan.json').read_text(encoding='utf-8')) if (path/'plan.json').exists() else {}
                model=plan.get('display_points',[])
                # 重复经过同一区域必须按路径进度对齐，不在整张图上最近点匹配。
                model=sorted(model,key=lambda p:p['s_mm'])
                starts[rid]=(e,plan,model,[p['s_mm'] for p in model]);series[rid]=[]
            if rid not in starts:continue
            if event=='BATCH_STATE':states[rid]=(e,e['cursor'],e['progress_mm'],e['state'])
            if event=='RX_TEXT':
                diagnostic=parse_ops_text(e['text'])
                if diagnostic is not None:
                    ops_records.setdefault(rid,[]).append(dict(diagnostic,host_monotonic_s=e['monotonic']))
                control=control_pattern.fullmatch(e['text'])
                if control and int(control[1])==starts[rid][1].get('id'):
                    controls[rid]=(e,tuple(map(int,control.groups())))
                stat=pattern.fullmatch(e['text'])
                if stat and int(stat[1])==starts[rid][1].get('id'):
                    name={4:'RUNNING',5:'WAITING',6:'DONE'}.get(int(stat[2]),'INACTIVE')
                    states[rid]=(e,int(stat[4]),int(stat[5])/10,name)
            if event!='FRAME':continue
            values=e['values']
            if len(values)<3 or any(v is None or not math.isfinite(v) for v in values[:3]):continue
            first,plan,model,distances=starts[rid]
            elapsed=e['monotonic']-first['monotonic']
            if e.get('protocol')=='SIM' and e.get('simulation_time_s') is not None:
                elapsed=e['simulation_time_s']-sim_bases.setdefault(rid,e['simulation_time_s'])
            row=dict(time_s=elapsed,host_elapsed_s=e['monotonic']-first['monotonic'],device_tick_ms=e.get('device_tick_ms'),
                present_channels=e.get('present_channels',24),full_state_age_ms=None if e.get('full_state_device_tick_ms') is None else
                    ((e['device_tick_ms']-e['full_state_device_tick_ms'])&0xffffffff),
                ops_x_mm=values[0]*10,ops_y_mm=values[1]*10,ops_yaw_deg=values[2],cursor=None,
                progress_mm=None,execution_state='',status_age_s=None,candidate_yaw_deg=None,candidate_yaw_error_deg=None,
                model_position_error_mm=None,model_yaw_error_deg=None,target_gap_mm=None,
                nominal_phase='',station='',reference_kind='',reference_switch=False,rate_window_valid=False,
                controller_age_s=None,real_controller_target=None,real_controller_flags=None,
                real_controller_goal_yaw_deg=None,real_controller_gap_mm=None,
                real_controller_vx_mm_s=None,real_controller_vy_mm_s=None,
                real_controller_yaw_request_deg_s=None,real_controller_yaw_measured_deg_s=None,
                real_controller_settled_ms=None)
            dt,speed,rate=velocity(previous.get(rid),e)
            row.update(dt_s=dt,speed_mm_s=speed,yaw_rate_deg_s=rate,rate_window_valid=dt is not None)
            points=plan.get('points',[]);state=states.get(rid)
            if state and points:
                stamp,cursor,progress,status=state;age=e['monotonic']-stamp['monotonic']
                row.update(cursor=cursor,progress_mm=progress,status_age_s=age,execution_state=status)
                # 批量读的设备帧时间可能早于同包TSTAT；只用此前状态，不反向套新状态。
                if status in ('RUNNING','WAITING','DONE') and 0<=age<=.35 and 0<=cursor<len(points):
                    row['reference_kind']='RECONSTRUCTED_BATCH_TARGET'
                    target=min(cursor+1,len(points)-1) if status=='RUNNING' else cursor;q=points[target]
                    gap=math.dist((row['ops_x_mm'],row['ops_y_mm']),(q[0]/10,q[1]/10))
                    row['target_gap_mm']=gap
                    row['station']='；'.join(plan.get('stations',{}).get(str(target),[]))
                    row['nominal_phase']='STOPPED' if status!='RUNNING' else 'WHEEL_PIVOT' if q[4]&32 else 'PIVOT' if q[4]&16 else 'TRAVEL' if gap>q[7]/10 else 'EXIT_HEADING'
                    if not q[4]&16 or status!='RUNNING':
                        key=(rid,target)
                        if plan.get('required_coordinate_caps',0)>=6 and gap<=q[7]/10:
                            heading_latches.add(key)
                        heading=(q[5] if status=='RUNNING' and gap>q[7]/10 and key not in heading_latches else q[3])/100
                        row.update(candidate_yaw_deg=heading,candidate_yaw_error_deg=wrap(heading-values[2]))
                    if model:
                        index=min(bisect.bisect_left(distances,progress),len(model)-1)
                        if index and abs(distances[index-1]-progress)<abs(distances[index]-progress):index-=1
                        sample=model[index];ox,oy,theta=first['metadata']['mapping'];c=math.cos(math.radians(theta));s=math.sin(math.radians(theta))
                        fx,fy=ox+c*row['ops_x_mm']+s*row['ops_y_mm'],oy-s*row['ops_x_mm']+c*row['ops_y_mm']
                        row.update(model_position_error_mm=math.dist((fx,fy),(sample['x_mm'],sample['y_mm'])),
                            model_yaw_error_deg=wrap((90-values[2]-theta)-sample['field_yaw_deg']))
            control=controls.get(rid)
            if control:
                stamp,data=control;age=e['monotonic']-stamp['monotonic']
                if 0<=age<=.35:
                    _,target,flags,goal,gap,vx,vy,request,measured,settled=data
                    row.update(controller_age_s=age,real_controller_target=target,real_controller_flags=flags,
                        real_controller_goal_yaw_deg=goal/100,real_controller_gap_mm=gap/10,
                        real_controller_vx_mm_s=vx/10,real_controller_vy_mm_s=vy/10,
                        real_controller_yaw_request_deg_s=request/100,real_controller_yaw_measured_deg_s=measured/100,
                        real_controller_settled_ms=settled)
            if e.get('protocol')=='SIM' and e.get('reference'):
                ref=e['reference'];ox,oy,theta=first['metadata']['mapping']
                c=math.cos(math.radians(theta));s=math.sin(math.radians(theta))
                fx,fy=ox+c*row['ops_x_mm']+s*row['ops_y_mm'],oy-s*row['ops_x_mm']+c*row['ops_y_mm']
                heading=90-theta-ref['field_yaw_deg']
                row.update(reference_kind='SIM_CONTROLLER_TARGET',cursor=ref.get('segment_index'),
                    nominal_phase=ref.get('segment_type',''),candidate_yaw_deg=wrap(heading),
                    candidate_yaw_error_deg=wrap(heading-values[2]),
                    target_gap_mm=math.dist((fx,fy),(ref['x_mm'],ref['y_mm'])))
            prior=series[rid][-1] if series[rid] else None
            if prior and prior['cursor']==row['cursor'] and row['cursor'] is not None and prior['candidate_yaw_deg'] is not None and row['candidate_yaw_deg'] is not None:
                row['reference_switch']=abs(wrap(prior['candidate_yaw_deg']-row['candidate_yaw_deg']))>5
            series[rid].append(row);previous[rid]=e
    for rid,rows in series.items():
        path=folder/'runs'/rid
        if rows:
            with (path/'analysis.csv').open('w',newline='',encoding='utf-8-sig') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        result=json.loads((path/'result.json').read_text(encoding='utf-8')) if (path/'result.json').exists() else dict(status='UNFINISHED')
        windows=[]
        # 以2秒窗口统计显著反向；只做可疑窗口提示，不能把正常拐弯直接定为故障。
        for start in range(0,len(rows),50):
            part=[r for r in rows[start:start+101] if r['time_s']-rows[start]['time_s']<=2]
            signs=[math.copysign(1,r['yaw_rate_deg_s']) for r in part if r['yaw_rate_deg_s'] is not None and abs(r['yaw_rate_deg_s'])>=5]
            reversals=sum(a!=b for a,b in zip(signs,signs[1:]))
            angles=[r['ops_yaw_deg'] for r in part]
            excursion=max((abs(wrap(a-angles[0])) for a in angles),default=0)
            if reversals>=3 and excursion>=1:
                windows.append(dict(from_s=part[0]['time_s'],to_s=part[-1]['time_s'],yaw_rate_reversals=reversals,
                    excursion_deg=excursion,cursors=sorted({r['cursor'] for r in part if r['cursor'] is not None}),
                    stations=sorted({r['station'] for r in part if r['station']})))
        errors=[r['model_position_error_mm'] for r in rows if r['model_position_error_mm'] is not None]
        report['runs'].append(dict(run_id=rid,kind=starts[rid][0]['kind'],frame_count=len(rows),result=result,
            ops_diagnostics=summarize(ops_records.get(rid,[])),
            duration_s=rows[-1]['time_s'] if rows else 0,valid_rate_pairs=sum(r['rate_window_valid'] for r in rows),
            candidate_target_switches=sum(r['reference_switch'] for r in rows),suspected_oscillation_windows=windows,
            model_position_rms_mm=math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
            model_matched_frames=len(errors),analysis_csv=str(path/'analysis.csv') if rows else None))
    report.update(last_transport=transport,last_parameter_readbacks=readbacks)
    destination=folder/('analysis_'+run_id+'.json' if run_id else 'analysis.json')
    destination.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    return report,destination


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',help='会话目录或单次runs目录');parser.add_argument('--run-id')
    args=parser.parse_args();report,path=analyze(args.folder,args.run_id)
    print(json.dumps(dict(report=str(path),source=report['source'],runs=len(report['runs'])),ensure_ascii=False))


if __name__=='__main__':main()
