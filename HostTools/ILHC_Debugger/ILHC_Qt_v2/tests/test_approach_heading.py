"""普通点提前转向、严格车头通过门及逐点下传的一致性。"""
import math
import time
import unittest
from pathlib import Path
import competition_simulation as competition
import coordinate_navigation as coordinate
from approach_heading import anticipate_route
from navigation_planner import CollisionScene
from hardware_coordinates import make_coordinate_path_batch,APPROACH_HEADING
from tests.test_adaptive_turn_fix import ACTUAL_CONTROL


def fixture():
    scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
    points=[(600,600),(1600,600),(1600,1600)]
    program=coordinate.build_program(points,0,pass_mm=5,mode='FORWARD',
        control=dict(ACTUAL_CONTROL,turn_lead_mm=40))
    samples,elapsed=coordinate.replay(program,scene)
    result=coordinate._replay_result(program,samples,elapsed,points,2120,'FORWARD',0,90,(False,False))
    return result,scene


class ApproachHeadingTests(unittest.TestCase):
    def test_large_corner_finishes_heading_before_last_30mm(self):
        old,scene=fixture();new=anticipate_route(old,scene)
        row=new['waypoint_program']['waypoints'][1]
        self.assertGreater(row['turn_lead_mm'],300)
        self.assertTrue(row['finish_heading_before_pass'])
        target=coordinate.CoordinateTracker(new['waypoint_program']).points[1]
        location=target['x_mm'],target['y_mm']
        def error_at_30mm(program):
            tracker=coordinate.CoordinateTracker(program)
            first=tracker.reference_at(0);pose=first['x_mm'],first['y_mm'],first['field_yaw_deg']
            for _ in range(1000):
                _,velocity,omega=tracker.command(pose,None,None)
                pose=pose[0]+velocity[0]*.02,pose[1]+velocity[1]*.02,pose[2]+omega*.02
                if tracker.index==1 and math.dist(pose[:2],location)<30:
                    return abs(coordinate.wrap(target['field_yaw_deg']-pose[2]))
            self.fail('未进入拐点前30mm')
        self.assertGreater(error_at_30mm(old['waypoint_program']),10)
        self.assertLess(error_at_30mm(new['waypoint_program']),1)

    def test_pass_gate_requires_orientation_and_still_advances_same_frame(self):
        old,scene=fixture();program=anticipate_route(old,scene)['waypoint_program']
        tracker=coordinate.CoordinateTracker(program);target=tracker.points[1]
        tracker.command((target['x_mm'],target['y_mm'],target['field_yaw_deg']+10),None,None)
        self.assertEqual(tracker.index,1)
        _,velocity,_=tracker.command((target['x_mm'],target['y_mm'],target['field_yaw_deg']+.5),None,None)
        self.assertEqual(tracker.index,2)
        self.assertGreater(math.hypot(*velocity),1,'完成航向的PASS同帧衔接，不新增停稳等待')

    def test_hardware_encodes_individual_lead_and_heading_flag(self):
        old,scene=fixture();new=anticipate_route(old,scene)
        batch=make_coordinate_path_batch(new,(600,600),0,90,(0,0,0),scene,{'map_version':1},token=17)
        row=new['waypoint_program']['waypoints'][1]
        self.assertEqual(batch['points'][1][7],round(row['turn_lead_mm']*10))
        self.assertTrue(batch['points'][1][4]&APPROACH_HEADING)
        self.assertEqual(batch['required_coordinate_caps'],8)
        self.assertEqual(batch['points'][-1][7],400,'各点窗口不能被最后一点覆盖')

    def test_deadline_keeps_verified_route_and_cancellation_does_not_return_it(self):
        old,scene=fixture()
        result=anticipate_route(old,scene,deadline=time.monotonic()-1)
        self.assertEqual(result['waypoint_program'],old['waypoint_program'])
        with self.assertRaisesRegex(ValueError,'取消'):
            anticipate_route(old,scene,cancelled=lambda:True)

    def test_narrow_lane_shortens_early_window_without_ignoring_block_sweep(self):
        data,_=competition.with_competition_defaults(competition.nav.load_map(Path(__file__).parents[1]/'navigation_map.json'))
        scene=competition.collision_scene(data)
        points=[(1200,2030),(1200,1950),(1200,400)]
        program=coordinate.build_program(points,180,goal_yaw=0,mode='FORWARD',pass_mm=5,
            control=dict(ACTUAL_CONTROL,turn_lead_mm=40))
        samples,elapsed=coordinate.replay(program,scene)
        old=coordinate._replay_result(program,samples,elapsed,points,1630,'FORWARD',180,0,(False,False))
        new=anticipate_route(old,scene)
        self.assertTrue(any(not a['accepted'] and '中央物料区' in a['reason']
                            for a in new['approach_turn_attempts']))
        final=new['waypoint_program']['waypoints'][-1]
        self.assertGreater(final['turn_lead_mm'],40)
        self.assertLess(final['turn_lead_mm'],140)
        coordinate.replay(new['waypoint_program'],scene)


if __name__=='__main__':unittest.main()
