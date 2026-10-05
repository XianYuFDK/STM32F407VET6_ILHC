"""关键坐标闭环PC回放、完整车体扫掠及可分享动图，不连接硬件。"""
import copy
import io
import json
import math
import queue
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import core
import competition_simulation as competition
from tests.test_competition_simulation import runner_for, advance

OUT = Path(__file__).parent


def field_plot(ax, data, obstacles=()):
    from matplotlib.patches import Rectangle, Circle
    ax.set_facecolor('#e2e6ea')
    for x1, y1, x2, y2, _name in data['rects']:
        ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,facecolor='#a89282',edgecolor='#8e796a'))
    for x,y,r,_name in data['circles']:
        ax.add_patch(Circle((x,y),r,facecolor='#808994'))
    for x,y in obstacles:
        ax.add_patch(Circle((x,y),core.SIM_OBSTACLE_R_MM,facecolor='#242a33'))
    for zone,(x,y) in core.ZONE_CENTER.items():
        ax.add_patch(Rectangle((x-150,y-150),300,300,facecolor='#315de3',alpha=.6))
        ax.text(x,y,'启停区%d'%zone,ha='center',va='center',color='white',fontsize=8)
    for name,(x,y) in data['competition']['stations'].items():
        ax.text(x,y+80,name,ha='center',fontsize=8,color='#343d4c')
    ax.set(xlim=(0,2400),ylim=(0,2400),xlabel='布局 X / mm',ylabel='布局 Y / mm')
    ax.set_aspect('equal');ax.grid(alpha=.22)


def setup_plot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    return plt


def main():
    plt=setup_plot()
    data=competition.load_profile()
    cases=[('无障碍',()),('单障碍',((700,1200),)),
           ('四障碍',competition.DEMO_OBSTACLES),('两障碍',((1200,1200),(297,294)))]
    results=[];plots=[];default_match=None
    for case,obstacles in cases:
        for zone in (1,2):
            match=competition.compile_match(data,zone=zone,sim_obstacles=obstacles,coordinate_mode=True)
            if default_match is None: default_match=match
            count=sum(len(l['route']['waypoint_program']['waypoints'])-1 for l in match['legs'])
            samples=sum(len(l['route']['trajectory']) for l in match['legs'])
            # 证明实际执行器不依赖密集点表或原始规划对象。
            sparse=copy.deepcopy(match)
            for stage in sparse['stages']:
                if stage['kind']=='TRAVEL':
                    stage['route'].pop('trajectory');stage['route'].pop('smoothed_primitives')
            runner=runner_for(sparse);previous=None
            while runner.active:
                advance(runner)
                snap=runner.sim.navigation_snapshot()
                pose=(*core.field_to_layout(*snap['hold']),-180+snap['yaw'])
                assert runner.scene.pose_reason(*pose) is None
                if previous: assert runner.scene.moving_pose_reason(previous,pose) is None
                previous=pose
            assert runner.status=='COMPLETE',runner.reason
            assert (runner.grabs,runner.placements)==(12,12)
            row=dict(case=case,zone=zone,coordinate_target_count=count,display_sample_count=samples,
                     elapsed_s=round(runner.elapsed_s,2),speed_limit_mm_s=runner.sim._nav_speed_mm_s,
                     status=runner.status,grabs=runner.grabs,placements=runner.placements,
                     full_body_sweeps_safe=True,runtime_sample_table=False,hardware_ready=False)
            results.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
            if case in ('无障碍','两障碍'): plots.append((row,match,runner.actual_trace,obstacles))
    (OUT/'coordinate_navigation_results_20261004.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    fig,axes=plt.subplots(2,2,figsize=(11,10),layout='constrained')
    for ax,(row,match,trace,obstacles) in zip(axes.flat,plots):
        field_plot(ax,data,obstacles)
        for leg in match['legs']:
            planned=[core.field_to_layout(p['x_mm'],p['y_mm']) for p in leg['route']['trajectory']]
            ax.plot(*zip(*planned),color='#3dbe85',lw=3,alpha=.65)
            ends=leg['route']['points'][1:]
            ax.scatter(*zip(*ends),s=12,facecolors='none',edgecolors='#13643b',zorder=5)
        actual=[core.field_to_layout(p['x_mm'],p['y_mm']) for p in trace]
        ax.plot(*zip(*actual),color='#3166cf',lw=1)
        ax.set_title('%s / 区%d · %d个坐标目标 · %.2fs · 全车扫掠通过'%(row['case'],row['zone'],row['coordinate_target_count'],row['elapsed_s']))
    fig.suptitle('PC关键坐标闭环执行：绿色规划、圆点衔接点、蓝色实际轨迹（250mm/s模型）',fontsize=12)
    fig.savefig(OUT/'coordinate_navigation_match_preview_20261004.png',dpi=140);plt.close(fig)

    # 二维码→原料区：只交关键坐标程序，动画显示实际车心和实际车头。
    leg=default_match['legs'][1]['route']
    scene=competition.collision_scene(data)
    sim=core.Simulator(queue.Queue(),queue.Queue());sim.handle_line('ZERO')
    first=leg['waypoint_program']['start'];sim.hold=first['x_mm'],first['y_mm'];sim.zval=90-first['field_yaw_deg']
    sim.submit_navigation_coordinates(sim.begin_navigation(),leg['waypoint_program'],(0,0,0),scene);states=[]
    for i in range(7500):
        sim.make_frame(i/core.SEND_HZ);snap=sim.navigation_snapshot()
        if i%6==0 or not snap['active']: states.append(copy.deepcopy(snap))
        if not snap['active']: break
    assert snap['tracking_status']=='COMPLETE',snap
    from PIL import Image
    from matplotlib.patches import Polygon
    fig,ax=plt.subplots(figsize=(7,7),layout='constrained')
    field_plot(ax,data)
    planned=[core.field_to_layout(p['x_mm'],p['y_mm']) for p in leg['trajectory']]
    ax.plot(*zip(*planned),color='#3dbe85',lw=4,alpha=.7,label='规划')
    points=leg['points']
    ax.scatter(*zip(*points),s=45,facecolors='none',edgecolors='#126e46',label='关键坐标',zorder=5)
    car=Polygon([(0,0)]*4,facecolor='#ff9f43',edgecolor='white',zorder=8);ax.add_patch(car)
    nose,=ax.plot([],[],color='white',lw=3,zorder=9)
    trace,=ax.plot([],[],color='#3166cf',lw=2,label='实际轨迹',zorder=6)
    reference,=ax.plot([],[],'o',color='#ae56d8',markersize=7,label='当前参考',zorder=7)
    title=ax.set_title('');ax.legend(loc='upper left',fontsize=8)
    images=[];actual=[]
    for state in states:
        x,y=core.field_to_layout(*state['hold']);yaw=math.radians(-180+state['yaw'])
        fx,fy=math.cos(yaw),math.sin(yaw);lx,ly=-fy,fx
        car.set_xy([(x+u*fx+v*lx,y+u*fy+v*ly) for u,v in ((140,130),(140,-130),(-140,-130),(-140,130))])
        nose.set_data([x,x+140*fx],[y,y+140*fy]);actual.append((x,y));trace.set_data(*zip(*actual))
        ref=state['reference'];rx,ry=core.field_to_layout(ref['x_mm'],ref['y_mm']);reference.set_data([rx],[ry])
        title.set_text('二维码→原料（无圆弧）：目标%d/%d %s · %.2fs\n车头%.1f° · %s'%(ref['segment_index'],ref['segment_count'],ref['segment_type'],
                       state['tracking_elapsed_s'],state['yaw']%360,state['tracking_status']))
        fig.canvas.draw()
        images.append(Image.frombytes('RGBA',fig.canvas.get_width_height(),fig.canvas.buffer_rgba()).convert('RGB').resize((700,700)))
    images[0].save(OUT/'coordinate_navigation_qr_raw_20261004.gif',save_all=True,append_images=images[1:],duration=120,loop=0,optimize=True)
    fig.savefig(OUT/'coordinate_navigation_qr_raw_final_20261004.png',dpi=140);plt.close(fig)
    print('预览PNG/GIF已保存；未连接硬件',flush=True)


if __name__=='__main__':
    main()
