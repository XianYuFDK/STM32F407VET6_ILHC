"""可复现PC比赛计时和每帧矩形扫掠复检，不连接硬件。"""
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import core
import competition_simulation as competition
from tests.test_competition_simulation import runner_for, advance


def main(output_name='mecanum_results_20261003.json', cases=None):
    results = []
    cases = cases or [('无障碍', (), {1:97.96}),
             ('四障碍预设', competition.DEMO_OBSTACLES, {1:108.76}),
             ('截图两障碍', ((1200, 1200), (297, 294)), {1:141.28, 2:148.72})]
    for name, obstacles, previous_times in cases:
        for zone in (1, 2):
            start = time.perf_counter()
            match = competition.compile_match(competition.load_profile(), zone=zone, sim_obstacles=obstacles)
            planning_s = time.perf_counter()-start
            runner = runner_for(match)
            previous, max_yaw_change = None, 0
            first_yaw = None
            while runner.active:
                advance(runner)
                snapshot = runner.sim.navigation_snapshot()
                pose = (*core.field_to_layout(*snapshot['hold']), -180+snapshot['yaw'])
                assert runner.scene.pose_reason(*pose) is None
                if previous is not None:
                    assert runner.scene.moving_pose_reason(previous, pose) is None
                previous = pose
                if first_yaw is None:
                    first_yaw = pose[2]
                max_yaw_change = max(max_yaw_change, abs((pose[2]-first_yaw+180)%360-180))
            assert runner.status == 'COMPLETE', runner.reason
            assert (runner.grabs, runner.placements) == (12,12)
            assert previous[:2] == tuple(match['home'])
            modes = sorted({p['motion_mode'] for leg in match['legs'] for p in leg['route']['trajectory']})
            row = dict(case=name, zone=zone, planning_s=round(planning_s,3), elapsed_s=round(runner.elapsed_s,2),
                       previous_elapsed_s=previous_times.get(zone), grabs=runner.grabs, placements=runner.placements,
                       body_sweeps_safe=True, max_body_heading_change_deg=round(max_yaw_change,3),
                       diagonal_docking=match['diagonal_docking'], motion_modes=modes,
                       travel_length_mm=round(sum(l['route']['trajectory_length_mm'] for l in match['legs']),2),
                       longest_strafe_mm=round(max(l['route']['motion_metrics']['longest_strafe_mm'] for l in match['legs']),2),
                       raw_to_rough=[dict(length_mm=round(match['legs'][i]['route']['trajectory_length_mm'],2),
                                         longest_strafe_mm=round(match['legs'][i]['route']['motion_metrics']['longest_strafe_mm'],2),
                                         planner=match['legs'][i]['route']['planner']) for i in (2,5)],
                       hardware_ready=False)
            results.append(row)
            print(json.dumps(row,ensure_ascii=False),flush=True)
    target = Path(__file__).with_name(output_name)
    target.write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__ == '__main__':
    main()
