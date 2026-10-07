"""复现八个角向的非零衔接速度，并与保存的v2.1.9响应比较。"""
import json
import math
from pathlib import Path
import sys

QT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(QT))
import core
import competition_simulation as competition
from coordinate_navigation import plan_route
from pivot_turns import wheel_anchor
from tests.test_pivot_turns import CORNERS


def response(row):
    first=row['program']['waypoints'][0]
    previous=(*core.layout_to_field(first['x_mm'],first['y_mm']),-90-first['layout_yaw_deg'])
    times=[];speeds=[];rates=[];boundary=[];anchors=[];arc=[]
    pivot=row['pivot']
    for i,p in enumerate(row['trajectory']):
        current=(p['x_mm'],p['y_mm'],p['field_yaw_deg'])
        speed=math.dist(previous[:2],current[:2])*core.SEND_HZ
        rate=abs((current[2]-previous[2]+180)%360-180)*core.SEND_HZ
        times.append((i+1)/core.SEND_HZ);speeds.append(speed);rates.append(rate)
        layout=core.field_to_layout(*previous[:2])
        if min(math.dist(layout,pivot[key]) for key in ('entry_mm','exit_mm'))<10:
            boundary.append(speed)
        if p.get('pivot_wheel'):
            arc.append(times[-1])
            anchor=wheel_anchor(core.field_to_layout(*current[:2]),-90-current[2],pivot['wheel_geometry'],pivot['pivot_wheel'])
            anchors.append(math.dist(anchor,pivot['center_mm']))
        previous=current
    return dict(time_s=times,speed_mm_s=speeds,rate_deg_s=rates,
                minimum_boundary_speed_mm_s=min(boundary),maximum_anchor_error_mm=max(anchors),
                pivot_interval_s=[arc[0],arc[-1]],elapsed_s=times[-1])


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    docs=Path(__file__).resolve().parent
    old=json.loads((docs/'smooth_wheel_pivot_four_corners_20261006.json').read_text(encoding='utf-8'))
    data=competition.load_profile();scene=competition.collision_scene(data);rows=[];metrics=[]
    for base in CORNERS:
        for reverse in (False,True):
            points=list(reversed(base)) if reverse else base
            start=math.degrees(math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0]))
            end=math.degrees(math.atan2(points[2][1]-points[1][1],points[2][0]-points[1][0]))
            result=plan_route(points[0],points[2],scene,data['competition']['lane_nodes'],start,goal_yaw=end,interactive=True)
            assert result['execution_safe'] and len(result.get('pivots',[]))==1
            row=dict(corner_mm=points[1],reverse=reverse,pivot=result['pivots'][0],program=result['waypoint_program'],trajectory=result['trajectory'])
            before=response(old[len(rows)]);after=response(row)
            assert after['minimum_boundary_speed_mm_s']>20 and after['maximum_anchor_error_mm']<2
            metrics.append(dict(corner_mm=points[1],reverse=reverse,
                                old_minimum_boundary_speed_mm_s=before['minimum_boundary_speed_mm_s'],
                                new_minimum_boundary_speed_mm_s=after['minimum_boundary_speed_mm_s'],
                                maximum_anchor_error_mm=after['maximum_anchor_error_mm'],elapsed_s=after['elapsed_s']))
            rows.append(row)
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for row,color,label in ((old[0],'#929ba6','v2.1.9：仍向零速制动'),(rows[0],'#087f5b','v2.1.11：非零速度衔接')):
        r=response(row)
        axes[0].plot(r['time_s'],r['speed_mm_s'],color=color,label=label)
        axes[1].plot(r['time_s'],r['rate_deg_s'],color=color)
        for ax in axes:ax.axvspan(*r['pivot_interval_s'],color=color,alpha=.08)
    axes[0].legend();axes[0].set_ylabel('车心平移速度 / mm/s')
    axes[1].set_ylabel('角速度绝对值 / °/s');axes[1].set_xlabel('离线模型时间 / s')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('同一麦轮固定支点：进出弯保持移动，末端仍停车\n50Hz整数轮速离线模型，不代表实车测量')
    fig.tight_layout();fig.savefig(docs/'moving_wheel_pivot_speed_comparison_20261006.png',dpi=140)
    for name,value in (('moving_wheel_pivot_four_corners_20261006.json',rows),('moving_wheel_pivot_metrics_20261006.json',metrics)):
        (docs/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(metrics,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
