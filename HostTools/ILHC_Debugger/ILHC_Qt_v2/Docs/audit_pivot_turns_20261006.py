"""Reproduce eight automatic corner plans and export their actual body/center paths."""
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import core
import competition_simulation as competition
from coordinate_navigation import plan_route
from pivot_turns import wheel_anchor,wrap

CORNERS=[[(2100,1200),(2100,2100),(1200,2100)],
         [(1200,2100),(300,2100),(300,1200)],
         [(300,1200),(300,300),(1200,300)],
         [(1200,300),(2100,300),(2100,1200)]]

def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Polygon,Rectangle
    plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    data=competition.load_profile();scene=competition.collision_scene(data)
    fig,axes=plt.subplots(2,2,figsize=(11,10));rows=[]
    for index,base in enumerate(CORNERS):
        for reverse in (False,True):
            points=list(reversed(base)) if reverse else base
            start=math.degrees(math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0]))
            end=math.degrees(math.atan2(points[2][1]-points[1][1],points[2][0]-points[1][0]))
            kwargs=dict(goal_yaw=end,interactive=True)
            old=plan_route(points[0],points[2],scene,data['competition']['lane_nodes'],start,pivot_turns=False,**kwargs)
            result=plan_route(points[0],points[2],scene,data['competition']['lane_nodes'],start,**kwargs)
            assert len(result.get('pivots',[]))==1 and result['execution_safe']
            pivot=result['pivots'][0]
            rows.append(dict(corner_mm=points[1],reverse=reverse,pivot=pivot,
                             baseline_model_s=old['predicted_tracking_s'],pivot_model_s=result['predicted_tracking_s'],
                             program=result['waypoint_program'],trajectory=result['trajectory']))
            if reverse:continue
            ax=axes.flat[index]
            for rect in data['rects']:
                x1,y1,x2,y2=rect[:4]
                ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,color='#f2cc54',alpha=.7))
            for circle in data['circles']:
                ax.add_patch(Circle(circle[:2],circle[2],color='#c3a276',alpha=.7))
            xs,ys=zip(*old['skeleton_points']);ax.plot(xs,ys,'--',color='#7b8da0',label='原折线骨架')
            xy=[core.field_to_layout(p['x_mm'],p['y_mm']) for p in result['trajectory']]
            ax.plot(*zip(*xy),color='#087f5b',lw=2,label='整数轮速模型实际车心')
            cx,cy=pivot['center_mm'];ax.scatter([cx],[cy],marker='+',s=170,color='#d85d16',label='左前轮固定支点',zorder=20)
            for point in (pivot['entry_mm'],pivot['exit_mm']):
                ax.plot([cx,point[0]],[cy,point[1]],':',color='#d85d16')
            arc=[p for p in result['trajectory'] if p['segment_type']=='PIVOT']
            for degrees,color in ((0,'#3176c0'),(45,'#a95bba'),(90,'#087f5b')):
                p=min(arc,key=lambda p:abs(wrap(-90-p['field_yaw_deg']-pivot['entry_yaw_deg']-degrees)))
                pose=(*core.field_to_layout(p['x_mm'],p['y_mm']),-90-p['field_yaw_deg'])
                polygon=scene.pose_polygon(*pose)
                ax.add_patch(Polygon(list(polygon.exterior.coords),facecolor=color,edgecolor=color,alpha=.16,label='车身转角%d°'%degrees))
                for wheel in ('FL','FR','BL','BR'):
                    wx,wy=wheel_anchor(pose[:2],pose[2],pivot['wheel_geometry'],wheel)
                    ax.add_patch(Circle((wx,wy),8,color='#d85d16' if wheel==pivot['pivot_wheel'] else color,alpha=.9,zorder=15))
                ax.arrow(pose[0],pose[1],60*math.cos(math.radians(pose[2])),60*math.sin(math.radians(pose[2])),
                         color=color,head_width=15,length_includes_head=True,zorder=12)
            ax.set_title('外角 (%d, %d) mm · 左前轮支点旋转90°'%(points[1][0],points[1][1]))
            ax.set_xlim((1600,2400) if points[1][0]>1200 else (0,800))
            ax.set_ylim((1600,2400) if points[1][1]>1200 else (0,800));ax.set_aspect('equal')
            ax.set_xlabel('布局 X / mm');ax.set_ylabel('布局 Y / mm');ax.grid(alpha=.2)
    axes.flat[0].legend(loc='lower left',fontsize=8)
    fig.suptitle('绕左前麦轮连续转弯：橙色轮心保持在同一位置\n轮心轴距206 mm / 左右轮心距218 mm；减速入弯→绕轮转90°→衔接直行；离线模型')
    fig.tight_layout()
    out=Path(__file__).resolve().parent
    fig.savefig(out/'smooth_wheel_pivot_four_corners_20261006.png',dpi=160)
    (out/'smooth_wheel_pivot_four_corners_20261006.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print([(row['corner_mm'],row['reverse'],row['baseline_model_s'],row['pivot_model_s']) for row in rows])

if __name__=='__main__':main()
