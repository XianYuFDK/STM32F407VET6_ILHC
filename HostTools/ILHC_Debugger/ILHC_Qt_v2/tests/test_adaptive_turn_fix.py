"""19:31日志场景：动态避障倒行候选和绕轮提前制动的行为回归。"""
import math
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import competition_simulation as competition
import coordinate_navigation as coordinate
import navigation_planner as nav
import pivot_turns
from Docs.audit_real_control_response_20261006 import simulate
from tests.test_pivot_turns import CORNERS, baseline

ACTUAL_CONTROL = dict(kpx=1.5, kpy=1.5, kpz=9.05, xyvmax=930,
                     zvmax=750, xyvmin=.5, zvmin=.5)
OBSTACLES = ((1191.1764705882354,692.6470588235293),
             (301.764705882353,2114.1176470588234))


class AdaptiveTurnFixTests(unittest.TestCase):
    def scene(self):
        data, _ = competition.with_competition_defaults(nav.load_map(Path(__file__).parents[1]/'navigation_map.json'))
        return data, competition.collision_scene(data,10,OBSTACLES)

    def test_obstacle_route_keeps_crane_goal_and_uses_one_rear_wheel_turn(self):
        data,scene=self.scene()
        route=coordinate.plan_route((1200,400),(400,1200),scene,data['competition']['lane_nodes'],
            0,goal_yaw=270,chassis_control=ACTUAL_CONTROL,deadline=time.monotonic()+2)
        self.assertEqual(route['skeleton_points'],[(1200,400),(400,400),(400,1200)])
        points=route['waypoint_program']['waypoints']
        turns=[p for p in points if p.get('motion')=='WHEEL_PIVOT']
        self.assertEqual(len(turns),1)
        self.assertEqual(turns[0]['pivot_wheel'],'BL')
        self.assertAlmostEqual(coordinate.wrap(points[-1]['layout_yaw_deg']-270),0)
        self.assertLess(route['execution_cost']['yaw_total_deg'],92)
        self.assertEqual(route['optimality']['policy'],'VERIFIED_TIME_AND_YAW_COST')
        samples,_=coordinate.replay(route['waypoint_program'],scene)
        self.assertEqual(samples[-1]['segment_type'],'STOP')

    def test_verified_direct_incumbent_survives_lane_search_deadline(self):
        data,scene=self.scene()
        with patch.object(coordinate,'_seed_route',side_effect=coordinate.OptimizationDeadline):
            route=coordinate.plan_route((1200,400),(400,1200),scene,data['competition']['lane_nodes'],
                0,goal_yaw=270,chassis_control=ACTUAL_CONTROL,deadline=time.monotonic()+2)
        self.assertTrue(route['execution_safe'])
        self.assertLess(route['execution_cost']['yaw_total_deg'],92)
        self.assertFalse(route['optimality']['proven'])

    def test_delayed_motion_brakes_before_goal_and_retains_moving_gates(self):
        scene=competition.collision_scene(competition.load_profile())
        route=baseline(CORNERS[0],scene,ACTUAL_CONTROL)
        program,_=pivot_turns.corner_trial(route['waypoint_program'],1,
            pivot_turns.wheel_geometry(scene),scene,lambda:False)
        checked=competition.collision_scene(competition.load_profile(),9)
        for tau,delay in ((.04,.02),(.08,.04)):
            with self.subTest(tau=tau,delay=delay):
                result=simulate(program,checked,coordinate.CoordinateTracker,tau,delay)
                self.assertTrue(result['done'])
                self.assertIsNone(result['unsafe'])
                self.assertEqual(result['pivot_rate_reversals'],0)
                self.assertLess(result['pivot_s'],2.3)
                # 剩余正常出口门仍严格；已到目标但位置错开不能直接切下点。
                tracker=coordinate.CoordinateTracker(program);tracker.index=2
                target=tracker.points[2]
                tracker.command((target['x_mm']+10,target['y_mm'],target['field_yaw_deg']),None,None)
                self.assertEqual(tracker.index,2)
                self.assertTrue(tracker._pivot_recover)


if __name__=='__main__':unittest.main()
