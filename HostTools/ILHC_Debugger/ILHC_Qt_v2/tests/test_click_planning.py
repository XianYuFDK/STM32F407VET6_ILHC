"""Interactive planning uses the actual body/controller and preserves cancellation."""
import threading
import unittest
from unittest.mock import patch
import core
import coordinate_navigation as coordinate
from navigation_planner import CollisionScene

class ClickPlanningTests(unittest.TestCase):
    def setUp(self):
        self.scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)

    def test_direct_mecanum_route_avoids_graph_and_replays_whole_body(self):
        with patch.object(coordinate,'_coordinate_graph',side_effect=AssertionError('unnecessary graph')):
            result=coordinate.plan_route((500,500),(1400,650),self.scene,[],0,goal_yaw=0,interactive=True)
        self.assertEqual(len(result['skeleton_points']),2)
        self.assertTrue(result['execution_safe']);self.assertFalse(result['optimality']['proven'])
        self.assertEqual(result['optimality']['policy'],'INTERACTIVE_FIRST_VERIFIED_SAFE')
        samples,elapsed=coordinate.replay(result['waypoint_program'],self.scene)
        self.assertEqual(samples[-1]['segment_type'],'STOP')
        self.assertEqual(elapsed,result['predicted_tracking_s'])

    def test_obstructed_direct_route_uses_safe_graph_detour(self):
        scene=CollisionScene([(900,600,1100,1400,'wall')],[],(0,0,3000,3000),10,(280,260,0),None)
        result=coordinate.plan_route((500,1000),(1500,1000),scene,
            [(500,400),(1500,400),(500,1600),(1500,1600)],0,goal_yaw=0,interactive=True)
        self.assertGreater(len(result['skeleton_points']),2)
        self.assertTrue(result['execution_safe'])
        samples,_=coordinate.replay(result['waypoint_program'],scene)
        self.assertEqual(samples[-1]['segment_type'],'STOP')

    def test_short_lateral_departure_accepts_fixed_heading_in_corner(self):
        scene=CollisionScene([],[],(0,0,3000,3000),0,(280,260,90),None)
        result=coordinate.plan_route((150,150),(240,150),scene,[],90,goal_yaw=90,interactive=True)
        self.assertEqual(result['planner'],'COORDINATE_FIXED')
        self.assertTrue(result['execution_safe'])

    def test_invalid_start_and_goal_rejected_before_search(self):
        args=dict(footprint=(280,260,0),bounds=(0,0,3000,3000),rects=[],circles=[],interactive=True)
        with patch.object(coordinate,'_interactive_route',side_effect=AssertionError('unnecessary search')):
            for start,goal,code in (((20,20),(500,500),'INVALID_START'),((500,500),(20,20),'INVALID_GOAL')):
                result=core.plan_coordinate_path(start,goal,**args)
                self.assertFalse(result['ok']);self.assertEqual(result['code'],code)
                self.assertIn('整车',result['reason'])

    def test_strict_lateral_constraint_is_not_bypassed_by_direct_shortcut(self):
        with self.assertRaises(ValueError):
            coordinate.plan_route((500,500),(500,800),self.scene,[],0,goal_yaw=0,
                chassis_control={'kpz':0},constraints={'strafe_run_limit_mm':0},interactive=True)

    def test_cancel_during_replay_never_returns_fast_result(self):
        event=threading.Event();real=coordinate._cached_replay
        def cancel(*args):
            event.set();return real(*args)
        with patch.object(coordinate,'_cached_replay',side_effect=cancel):
            with self.assertRaisesRegex(ValueError,'取消'):
                coordinate.plan_route((500,500),(1400,650),self.scene,[],0,
                    cancelled=event.is_set,interactive=True)

    def test_optimizer_retains_fixed_incumbent_when_seed_later_hits_deadline(self):
        corner=CollisionScene([],[],(0,0,3000,3000),0,(280,260,90),None)
        route=[(150,150),(240,150)]
        program=coordinate.build_program(route,90,goal_yaw=90,mode='FIXED')
        samples,elapsed=coordinate.replay(program,corner)
        seed=coordinate._replay_result(program,samples,elapsed,route,90,'FIXED',90,90,(False,False))
        def timeout(*args,**kwargs):
            kwargs['incumbent'](seed)
            raise coordinate.OptimizationDeadline()
        with patch.object(coordinate,'_seed_route',side_effect=timeout), \
             patch.object(coordinate,'_coordinate_graph',side_effect=coordinate.OptimizationDeadline()):
            result=coordinate.plan_route((150,150),(240,150),corner,[],90,goal_yaw=90)
        self.assertTrue(result['execution_safe']);self.assertFalse(result['optimality']['proven'])
        self.assertTrue(result['search']['time_limit_hit'])

    def test_interactive_zero_budget_and_cancel_remain_distinct(self):
        args=dict(footprint=(280,260,0),bounds=(0,0,3000,3000),rects=[],circles=[],interactive=True,time_limit_s=0)
        self.assertEqual(core.plan_coordinate_path((500,500),(1500,500),**args)['code'],'TIME_LIMIT')
        event=threading.Event();event.set()
        self.assertEqual(core.plan_coordinate_path((500,500),(1500,500),cancel=event,**args)['code'],'CANCELLED')

if __name__=='__main__':unittest.main()
