"""Offline click planning benchmark; no serial or motion."""
import cProfile
import json
from pathlib import Path
import pstats
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import core
import competition_simulation as competition

def run():
    data=competition.load_profile()
    cases=[('near', (2100,2100),(2100,2030),180),
           ('qr',(2100,2100),(2180,1200),180),
           ('center',(2100,2100),(1200,1200),180),
           ('raw',(2180,1200),(1200,2030),180),
           ('rough',(1200,2030),(1200,400),89.5),
           ('storage',(1200,400),(400,1200),89.5),
           ('blocked',(2100,2100),(1100,1600),180)]
    rows=[]
    for name,start,goal,yaw in cases:
        t=time.monotonic()
        result=core.plan_coordinate_path(start,goal,coordinate_nodes=data['competition']['lane_nodes'],
            footprint=(280,260,yaw),start_heading_deg=yaw,goal_heading_deg=yaw,
            rects=data['rects'],circles=data['circles'],bounds=data['bounds'],
            drivable_polygons=data['drivable_polygons'],pad=10,interactive='--quick' in sys.argv)
        row=dict(name=name,wall_s=round(time.monotonic()-t,4),ok=result['ok'],
                 code=result.get('code'),reason=result.get('reason'),search=result.get('search'))
        rows.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
    return rows

if __name__=='__main__':
    profiler=cProfile.Profile();profiler.enable();rows=run();profiler.disable()
    prefix=sys.argv[1] if len(sys.argv)>1 else 'click_planner'
    root=Path(__file__).parent
    (root/(prefix+'.json')).write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    with (root/(prefix+'-profile.txt')).open('w',encoding='utf-8') as stream:
        pstats.Stats(profiler,stream=stream).sort_stats('cumulative').print_stats(35)
