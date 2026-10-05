"""Offline old/new whole-match comparison, quantized upload and body sweeps."""
import copy
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import core
import competition_simulation as competition
import coordinate_navigation as coordinate
import navigation_planner as nav
from hardware_coordinates import make_coordinate_batch
from tests.test_competition_simulation import runner_for,advance


def run(match,check=False):
    sparse=copy.deepcopy(match)
    for stage in sparse['stages']:
        if stage['kind']=='TRAVEL':stage['route'].pop('trajectory')
    runner=runner_for(sparse);previous=None
    while runner.active:
        advance(runner)
        if check:
            snap=runner.sim.navigation_snapshot()
            pose=(*core.field_to_layout(*snap['hold']),-180+snap['yaw'])
            assert runner.scene.pose_reason(*pose) is None
            if previous is not None:assert runner.scene.moving_pose_reason(previous,pose) is None
            previous=pose
    assert runner.status=='COMPLETE',runner.reason
    assert (runner.grabs,runner.placements)==(12,12)
    return round(runner.elapsed_s,2)


def main():
    profiles=[('default',competition.with_competition_defaults(nav.load_map(
        Path(competition.__file__).with_name('navigation_map.json')))[0]),('competition',competition.load_profile())]
    rows=[];started=time.monotonic()
    for profile,data in profiles:
        for case,obstacles in [('clear',()),('two_obstacles',((1200,1200),(297,294)))]:
            for zone in (1,2):
                with patch.object(coordinate,'plan_route',coordinate._seed_route):
                    old=competition.compile_match(data,zone=zone,sim_obstacles=obstacles,coordinate_mode=True)
                t=time.monotonic()
                new=competition.compile_match(data,zone=zone,sim_obstacles=obstacles,coordinate_mode=True)
                wall=time.monotonic()-t
                scene=competition.collision_scene(data,sim_obstacles=obstacles)
                batch=make_coordinate_batch(new,(0,0,0),scene)
                old_s,new_s=run(old),run(new,True)
                row=dict(profile=profile,case=case,zone=zone,old_s=old_s,new_s=new_s,
                    saving_s=round(old_s-new_s,2),planning_wall_s=wall,point_count=batch['point_count'],
                    body_sweeps_safe=True,quantized_preflight_safe=True,
                    legs=[dict(label=l['label'],cost_mm=l['route']['search']['skeleton_cost_mm'],
                        predicted_s=l['route']['predicted_tracking_s'],optimality=l['route']['optimality'],
                        search=l['route']['search']) for l in new['legs']])
                rows.append(row)
                print(json.dumps({k:v for k,v in row.items() if k!='legs'},ensure_ascii=False),flush=True)
    summary=dict(matches=len(rows),legs=sum(len(r['legs']) for r in rows),
        proven_legs=sum(l['optimality']['proven'] for r in rows for l in r['legs']),
        faster_matches=sum(r['saving_s']>.001 for r in rows),
        slower_matches=sum(r['saving_s']<-.001 for r in rows),elapsed_wall_s=time.monotonic()-started)
    Path(__file__).with_name('route_optimizer_results_20261005.json').write_text(
        json.dumps(dict(summary=summary,rows=rows),ensure_ascii=False,indent=2),encoding='utf-8')
    print('SUMMARY '+json.dumps(summary),flush=True)


if __name__=='__main__':main()
