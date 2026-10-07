"""离线绘制粗加工→暂存的实际控制模型，不连接硬件。"""
import json
from pathlib import Path
import sys

QT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(QT))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import competition_simulation as competition
import coordinate_navigation as coordinate


def main():
    data=competition.load_profile();scene=competition.collision_scene(data)
    match=competition.compile_match(data,coordinate_mode=True)
    assert match['planning_stats']['search_calls']==0
    current=next(leg['route'] for leg in match['legs'] if tuple(leg['route']['skeleton_points'][0])==tuple(data['competition']['stations']['rough'])
        and tuple(leg['route']['skeleton_points'][-1])==tuple(data['competition']['stations']['storage']))
    old=coordinate._seed_route((1200,400),(400,1200),scene,data['competition']['lane_nodes'],0,goal_yaw=270)
    fig,axes=plt.subplots(1,2,figsize=(10,4.5),dpi=150)
    for route,label,color in ((old,'Previous middle route','#a66f49'),(current,'Tail + wheel turn','#009dc5')):
        samples=route['trajectory']
        axes[0].plot([p['x_mm']/10 for p in samples],[p['y_mm']/10 for p in samples],label=label,color=color)
        initial=-90-route['waypoint_program']['waypoints'][0]['layout_yaw_deg'];angle=[initial];previous=initial
        for p in samples:
            angle.append(angle[-1]+coordinate.wrap(p['field_yaw_deg']-previous));previous=p['field_yaw_deg']
        axes[1].plot([i/50 for i in range(len(angle))],angle,label=label,color=color)
    axes[0].invert_xaxis();axes[0].set_aspect('equal');axes[0].set_xlabel('Field X left (cm)');axes[0].set_ylabel('Field Y forward (cm)')
    axes[1].set_xlabel('Model time (s)');axes[1].set_ylabel('Unwrapped mathematical heading (deg)')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Controller replay comparison - no physical motion verified')
    fig.tight_layout();fig.savefig(QT/'Docs/route_journal_comparison_20261006.png');plt.close(fig)
    result=dict(physical_motion_verified=False,comparison=[])
    for route,label in ((old,'previous_middle'),(current,'tail_wheel')):
        cost=coordinate.execution_cost(route['waypoint_program'],route['trajectory'],route['predicted_tracking_s'])
        result['comparison'].append(dict(label=label,skeleton_points=route['skeleton_points'],predicted_tracking_s=route['predicted_tracking_s'],
            yaw_total_deg=cost['yaw_total_deg'],motion_metrics=route['motion_metrics'],pivot_count=len(route.get('pivots',[]))))
    (QT/'Docs/route_journal_comparison_20261006.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
