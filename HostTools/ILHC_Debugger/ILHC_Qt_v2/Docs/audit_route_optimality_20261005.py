"""Independent graph optimum + controller candidate audit. Offline, no serial.

Graph lower bounds do not prove full vehicle/control optimality. Counterexamples
use the actual controller replay and identical start/end position and heading.
"""
import heapq
import itertools
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import competition_simulation as competition
import coordinate_navigation as coordinate
import navigation_planner as nav
from mecanum_planner import FixedHeadingScene,motion_metrics
import core


def connected(links,start,goal):
    found,pending={tuple(start)},[tuple(start)]
    while pending:
        for p in links.get(pending.pop(),()):
            if p not in found:found.add(p);pending.append(p)
    return tuple(goal) in found


def graph_for(start,goal,scene,nodes,yaw,refined,extended,cache):
    base=competition._lane_graph(start,goal,scene,nodes,lambda:False,refined,cache)
    graph={p:set(targets) for p,targets in base.items()}
    disconnected=not connected(base,start,goal)
    headings=(yaw,yaw+90) if extended else (yaw,) if disconnected else ()
    for heading in headings:
        extra=competition._lane_graph(start,goal,FixedHeadingScene(scene,heading),nodes,
                        lambda:False,refined,cache.setdefault(('fixed',heading),{}))
        for a,targets in extra.items():
            for b in targets:
                if math.dist(a,b)<=400 or disconnected and (a[0]==b[0] or a[1]==b[1]):
                    graph.setdefault(a,set()).add(b)
    return graph


def dijkstra(graph,start,goal):
    """Exact finite graph length + 120 per bend, using predecessor state.

    Independent of production A*, no heuristic, prefix quota or expansion cap.
    It relaxes controller/body turning and permits revisiting position with a
    different incoming direction; therefore it is a graph lower bound only.
    """
    counter=itertools.count();initial=(None,tuple(start))
    best={initial:0.};queue=[(0.,next(counter),initial,[tuple(start)])]
    while queue:
        cost,_,state,path=heapq.heappop(queue)
        if cost>best[state]+nav.EPS:continue
        previous,current=state
        if current==tuple(goal):return cost,path
        for target in graph.get(current,()):
            penalty=0
            if previous is not None:
                u=(current[0]-previous[0],current[1]-previous[1])
                v=(target[0]-current[0],target[1]-current[1])
                cross=u[0]*v[1]-u[1]*v[0]
                if abs(cross)>nav.EPS:penalty=120
                elif u[0]*v[0]+u[1]*v[1]<0:continue
            total=cost+math.dist(current,target)+penalty;new=(current,target)
            if total>=best.get(new,math.inf)-nav.EPS:continue
            best[new]=total;heapq.heappush(queue,(total,next(counter),new,path+[target]))
    return None,None


def skeleton_cost(points):
    total=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
    for a,b,c in zip(points,points[1:],points[2:]):
        if abs((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))>nav.EPS:total+=120
    return total


def better_replays(route,scene):
    """Compare same skeleton/boundary poses with every existing mode/setting."""
    points=route['skeleton_points'];original=route['waypoint_program']
    start_yaw=original['waypoints'][0]['layout_yaw_deg']
    goal_yaw=original['waypoints'][-1]['layout_yaw_deg']
    controls=dict(original['control']);cache={};safe=[];tested=0
    settings=((100,200,0),(100,60,0),(60,60,0),(20,40,0),(5,40,0),
              (5,200,0),(5,10,0),(5,40,80),(5,60,120))
    for mode in ('MIN_TURN','FORWARD','FIXED'):
        for radius,lead,departure in settings:
            tested+=1
            try:
                program=coordinate.build_program(points,start_yaw,goal_yaw=goal_yaw,pass_mm=radius,
                    mode=mode,control=dict(controls,turn_lead_mm=lead),departure_mm=departure,
                    constraints=original.get('constraints'))
                samples,elapsed=coordinate.replay(program,scene,sweep_cache=cache)
                metrics=motion_metrics({'trajectory':samples})
                if metrics['longest_strafe_mm']>500:continue
                safe.append(dict(mode=mode,pass_mm=radius,lead_mm=lead,departure_mm=departure,
                                 predicted_s=elapsed,length_mm=samples[-1]['s_mm'],
                                 waypoint_program=program))
            except ValueError:pass
    if not safe:return dict(tested=tested,safe=0,fastest=None)
    selected=min(safe,key=lambda r:r['predicted_s'])
    return dict(tested=tested,safe=len(safe),fastest=selected)


def main():
    # Also audit the actual default navigation geometry with auto-loaded config.
    profiles=[('默认地图自动配置',competition.with_competition_defaults(
              nav.load_map(Path(competition.__file__).with_name('navigation_map.json')))[0]),
              ('比赛地图',competition.load_profile())]
    cases=[('无障碍',()),('两障碍',((1200,1200),(297,294)))]
    rows=[];started=time.monotonic()
    for profile,data in profiles:
        for case,obstacles in cases:
            for zone in (1,2):
                match=competition.compile_match(data,zone=zone,sim_obstacles=obstacles,coordinate_mode=True)
                scene=competition.collision_scene(data,sim_obstacles=obstacles)
                cache={}
                for index,leg in enumerate(match['legs']):
                    result=leg['route'];points=result['skeleton_points']
                    start,goal=points[0],points[-1];yaw=result['start_heading_deg']
                    search=result['search']
                    graph=graph_for(start,goal,scene,data['competition']['lane_nodes'],yaw,
                                    search['refined'],search['extended'],cache)
                    lower,path=dijkstra(graph,start,goal)
                    actual=skeleton_cost(points)
                    assert abs(actual-search['skeleton_cost_mm'])<.001
                    assert lower is not None and actual+.001>=lower
                    row=dict(profile=profile,case=case,zone=zone,index=index,label=leg['label'],
                             skeleton_cost_mm=actual,graph_lower_bound_mm=lower,
                             graph_gap_pct=100*(actual/lower-1),graph_lower_bound_route=path,
                             selected_s=result['predicted_tracking_s'],search=search,global_optimality_proven=False)
                    if actual-lower>.001:
                        row['lower_route_replay']=better_replays(dict(result,skeleton_points=path),scene)
                    # Different controller settings can be tested independently
                    # on one profile/zone; full 27-setting comparisons per leg.
                    if profile=='比赛地图' and case=='无障碍' and zone==1:
                        comparison=better_replays(result,scene);row['replay_comparison']=comparison
                        best=comparison['fastest']
                        row['safe_faster_alternative']=best is not None and best['predicted_s']+.001<row['selected_s']
                        row['time_saving_s']=max(0,row['selected_s']-best['predicted_s']) if best else 0
                    rows.append(row)
                    print(json.dumps({k:v for k,v in row.items() if k not in ('graph_lower_bound_route','replay_comparison')},
                                     ensure_ascii=False),flush=True)
    summary=dict(legs=len(rows),equal_graph_lower_bound=sum(r['graph_gap_pct']<.001 for r in rows),
                 worse_than_relaxed_graph=sum(r['graph_gap_pct']>=.001 for r in rows),
                 max_graph_gap_pct=max(r['graph_gap_pct'] for r in rows),
                 feasible_graph_lower_bound_examples=sum(
                     r.get('lower_route_replay',{}).get('safe',0)>0 for r in rows),
                 full_replay_comparisons=sum('replay_comparison' in r for r in rows),
                 safe_faster_examples=sum(r.get('safe_faster_alternative',False) for r in rows),
                 elapsed_wall_s=time.monotonic()-started)
    output=Path(__file__).with_name('route_optimality_results_20261005.json')
    output.write_text(json.dumps(dict(summary=summary,rows=rows),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print('SUMMARY '+json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
