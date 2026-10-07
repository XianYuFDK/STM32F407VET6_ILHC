"""以只存在于本进程的控制器比较21:12日志程序，检查提前转向扫掠。"""
import json
import math
from pathlib import Path
import sys
QT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(QT))
import coordinate_navigation as coordinate
import competition_simulation as competition
from approach_heading import approach_lead

BASE=coordinate.CoordinateTracker


class AdaptiveTracker(BASE):
    def __init__(self,program):
        super().__init__(program)
        self.leads=[]
        rows=program['waypoints']
        for i,row in enumerate(rows):
            self.leads.append(approach_lead(self.control,row.get('travel_yaw_deg',row['layout_yaw_deg']),
                row['layout_yaw_deg'],row['pass_mm'],0 if i==0 else
                math.dist((rows[i-1]['x_mm'],rows[i-1]['y_mm']),(row['x_mm'],row['y_mm']))))

    def command(self,*args):
        self.control['turn_lead_mm']=self.leads[self.index]
        return super().command(*args)


def main():
    coordinate.CoordinateTracker=AdaptiveTracker
    directory=QT/'records/runs/20261006_211155_339638_REAL_8badec59/runs/92ebbeb40d1246c6b9b2962ef2b4d9a3'
    match=json.loads((directory/'plan.json').read_text(encoding='utf-8'))['match']
    scene=competition.collision_scene(match['map_snapshot'],10,match['sim_obstacles'])
    for i,leg in enumerate(match['legs']):
        program=leg['route']['waypoint_program']
        try:
            samples,elapsed=coordinate.replay(program,scene)
            print(i,'OK',elapsed,AdaptiveTracker(program).leads)
        except ValueError as exc:print(i,'UNSAFE',str(exc),AdaptiveTracker(program).leads)


if __name__=='__main__':main()
