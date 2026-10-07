"""Fixed-center turns: four actual map corners, swept body and parameter limits."""
import copy
import math
import threading
import unittest
from unittest.mock import patch
import core
import competition_simulation as competition
import coordinate_navigation as coordinate
import pivot_turns

CORNERS=[[(2100,1200),(2100,2100),(1200,2100)],
         [(1200,2100),(300,2100),(300,1200)],
         [(300,1200),(300,300),(1200,300)],
         [(1200,300),(2100,300),(2100,1200)]]

def baseline(points,scene,control=None):
    yaw=math.degrees(math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0]))
    final=math.degrees(math.atan2(points[2][1]-points[1][1],points[2][0]-points[1][0]))
    failures=[]
    for radius,lead,departure in coordinate.CONTROL_SETTINGS:
        if departure:continue
        program=coordinate.build_program(points,yaw,goal_yaw=final,mode='FORWARD',pass_mm=radius,
                                        control=dict(control or {},turn_lead_mm=lead))
        try:
            samples,elapsed=coordinate.replay(program,scene);break
        except ValueError as exc:failures.append(str(exc))
    else:raise ValueError('; '.join(failures))
    result=coordinate._replay_result(program,samples,elapsed,points,
        math.dist(points[0],points[1])+math.dist(points[1],points[2])+120,
        'FORWARD',yaw,final,(False,False))
    result['optimality']=dict(proven=True)
    return result

class PivotTurnsTests(unittest.TestCase):
    def setUp(self):
        self.data=competition.load_profile();self.scene=competition.collision_scene(self.data)

    def test_automatic_click_planner_selects_all_four_corners_both_directions(self):
        for base in CORNERS:
            for reverse in (False,True):
                points=list(reversed(base)) if reverse else base
                yaw=math.degrees(math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0]))
                end=math.degrees(math.atan2(points[2][1]-points[1][1],points[2][0]-points[1][0]))
                result=coordinate.plan_route(points[0],points[2],self.scene,
                    self.data['competition']['lane_nodes'],yaw,goal_yaw=end,interactive=True)
                self.assertEqual(len(result.get('pivots',[])),1,result.get('pivot_attempts'))
                self.assertTrue(result['execution_safe'])
                self.assertEqual(result['trajectory'][-1]['segment_type'],'STOP')

    def test_all_four_outer_corners_in_both_directions_execute_safe_pivots(self):
        results=[]
        for points in CORNERS:
            for reverse in (False,True):
                route=list(reversed(points)) if reverse else points
                with self.subTest(route=route):
                    old=baseline(route,self.scene)
                    result=pivot_turns.select_turns(old,self.scene,lambda:False)
                    self.assertTrue(result.get('pivots'),result.get('pivot_attempts'))
                    self.assertEqual(result['waypoint_program']['schema_version'],3)
                    self.assertFalse(result['optimality']['proven'])
                    samples,elapsed=coordinate.replay(result['waypoint_program'],self.scene)
                    self.assertEqual(samples[-1]['segment_type'],'STOP')
                    self.assertLess(elapsed,7.7,'提前制动增加理想用时，仍须快于旧8.86s停转方案')
                    self.assertTrue(any(row['segment_type']=='PIVOT' for row in samples))
                    pivot=result['pivots'][0]
                    anchors=[pivot_turns.wheel_anchor(core.field_to_layout(p['x_mm'],p['y_mm']),
                        -90-p['field_yaw_deg'],pivot['wheel_geometry'],pivot['pivot_wheel'])
                        for p in samples if p.get('pivot_wheel')]
                    self.assertGreater(len(anchors),30)
                    self.assertLess(max(math.dist(anchor,pivot['center_mm']) for anchor in anchors),2)
                    previous=tuple(core.layout_to_field(*route[0]));boundary_speeds=[]
                    for p in samples:
                        xy=(p['x_mm'],p['y_mm'])
                        layout=core.field_to_layout(*previous)
                        if min(math.dist(layout,pivot[key]) for key in ('entry_mm','exit_mm'))<10:
                            boundary_speeds.append(math.dist(previous,xy)*core.SEND_HZ)
                        previous=xy
                    self.assertTrue(boundary_speeds)
                    self.assertGreater(min(boundary_speeds),20,'正常进出弯须保持运动，不能只删除200ms等待')
                    results.append((route[1],reverse,result['pivots'][0]['radius_mm'],old['predicted_tracking_s'],elapsed))
        print('PC四角指定圆心:',results)

    def test_wheel_body_sweep_in_narrow_corner_is_rejected_before_control(self):
        program=coordinate.build_program([(1900,1200),(1900,1900),(1200,1900)],90,goal_yaw=180,mode='FORWARD')
        with self.assertRaisesRegex(ValueError,'扫掠'):
            pivot_turns.corner_trial(program,1,pivot_turns.wheel_geometry(self.scene),self.scene,lambda:False)

    def test_arc_entry_and_exit_have_stricter_yaw_gate_than_ordinary_pass(self):
        old=baseline(CORNERS[0],self.scene)
        program,_=pivot_turns.corner_trial(old['waypoint_program'],1,pivot_turns.wheel_geometry(self.scene),self.scene,lambda:False)
        tracker=coordinate.CoordinateTracker(program)
        entry=tracker.points[1];exit=tracker.points[2]
        tracker.command((entry['x_mm'],entry['y_mm'],entry['field_yaw_deg']+10),0,0)
        self.assertEqual(tracker.index,1)
        tracker.command((entry['x_mm'],entry['y_mm'],entry['field_yaw_deg']),0,0)
        self.assertEqual(tracker.index,2,'入弯实际位置/航向到位后立即衔接，不等待200ms')
        tracker.command((exit['x_mm'],exit['y_mm'],exit['field_yaw_deg']+10),0,0)
        self.assertEqual(tracker.index,2)
        tracker.command((exit['x_mm'],exit['y_mm'],exit['field_yaw_deg']),0,0)
        self.assertEqual(tracker.index,3,'出弯实际到位后同一帧衔接直线')

    def test_center_motion_and_wheel_limits_with_nondefault_parameters(self):
        settings=dict(kpx=3,kpy=4,kpz=12,xyvmax=350,zvmax=250,xyvmin=3,zvmin=2)
        old=baseline(CORNERS[2],self.scene,settings)
        result=pivot_turns.select_turns(old,self.scene,lambda:False)
        self.assertTrue(result.get('pivots'),result.get('pivot_attempts'))
        samples,_=coordinate.replay(result['waypoint_program'],self.scene)
        arc=[p for p in samples if p['segment_type']=='PIVOT']
        self.assertGreater(len(arc),30)
        self.assertGreater(math.dist((arc[0]['x_mm'],arc[0]['y_mm']),(arc[-1]['x_mm'],arc[-1]['y_mm'])),50)

    def test_malformed_center_or_old_schema_cannot_silently_run_as_chord(self):
        old=baseline(CORNERS[0],self.scene)
        program,_=pivot_turns.corner_trial(old['waypoint_program'],1,pivot_turns.wheel_geometry(self.scene),self.scene,lambda:False)
        for change in (lambda p:p.update(schema_version=2),
                       lambda p:p['waypoints'][2].update(pivot_center_mm=[0,0]),
                       lambda p:p['waypoints'][2].update(pivot_wheel='BR'),
                       lambda p:p['waypoints'][0].update(motion='PIVOT')):
            invalid=copy.deepcopy(program);change(invalid)
            with self.assertRaises(ValueError):coordinate.CoordinateTracker(invalid)

    def test_backward_turns_use_rear_wheel_anchors(self):
        observed=set()
        for points in (CORNERS[0],list(reversed(CORNERS[0]))):
            yaw=math.degrees(math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0]))+180
            end=math.degrees(math.atan2(points[2][1]-points[1][1],points[2][0]-points[1][0]))+180
            program=coordinate.build_program(points,yaw-180,goal_yaw=end-180,mode='FORWARD',pass_mm=5,control=dict(turn_lead_mm=10))
            for row in program['waypoints']:
                row['travel_yaw_deg']=row.get('travel_yaw_deg',row['layout_yaw_deg'])+180
                row['layout_yaw_deg']+=180
            trial,metadata=pivot_turns.corner_trial(program,1,pivot_turns.wheel_geometry(self.scene),self.scene,lambda:False)
            samples,_=coordinate.replay(trial,self.scene)
            self.assertEqual(samples[-1]['segment_type'],'STOP')
            observed.add(metadata['pivot_wheel'])
        self.assertEqual(observed,{'BL','BR'})

    def test_cancel_even_after_successful_arc_replay_cannot_return_result(self):
        old=baseline(CORNERS[0],self.scene);event=threading.Event();real=coordinate.replay
        def cancel(*args):
            result=real(*args);event.set();return result
        with patch.object(coordinate,'replay',side_effect=cancel):
            with self.assertRaisesRegex(ValueError,'取消'):
                pivot_turns.select_turns(old,self.scene,event.is_set)

    def test_no_new_motion_when_optimizer_budget_already_exhausted(self):
        old=baseline(CORNERS[0],self.scene)
        result=pivot_turns.select_turns(old,self.scene,lambda:False,deadline=0)
        self.assertIs(result,old)
        self.assertNotIn('pivots',result)

if __name__=='__main__':unittest.main()
