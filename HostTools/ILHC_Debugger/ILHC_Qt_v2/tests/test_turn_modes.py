import argparse
import copy
import math
import os
import time
import unittest
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import competition_simulation as competition
import coordinate_navigation as coordinate
import core
from route_store import RouteStore


class TurnModeTests(unittest.TestCase):
    def test_modes_generate_distinct_safe_programs_and_wire_flags(self):
        data=competition.load_profile()
        scene=competition.collision_scene(data)
        start,goal=(1150,350),(350,1150)
        for mode in ('MOVING','WHEEL','STOP_TURN'):
            with self.subTest(mode=mode):
                route=coordinate.plan_route(start,goal,scene,[],0,goal_yaw=270,
                    turn_mode=mode,deadline=time.monotonic()+6)
                rows=route['waypoint_program']['waypoints']
                self.assertEqual(route['turn_mode'],mode)
                if mode=='WHEEL':
                    self.assertTrue(any(r.get('motion')=='WHEEL_PIVOT' for r in rows))
                else:
                    self.assertFalse(any(r.get('motion') for r in rows))
                if mode=='STOP_TURN':
                    self.assertTrue(all(r['kind']=='STOP' for r in rows[1:]))
                    for a,b in zip(rows,rows[1:]):
                        moved=math.hypot(a['x_mm']-b['x_mm'],a['y_mm']-b['y_mm'])>1
                        if moved:self.assertAlmostEqual(coordinate.wrap(b['layout_yaw_deg']-a['layout_yaw_deg']),0)
                # Actual encoder, quantization and complete replay, not just UI labels.
                from hardware_coordinates import make_coordinate_batch
                match=dict(home=start,start_yaw=0,turn_mode=mode,chassis_control=core.CHASSIS_DEFAULTS,map_snapshot=data,
                           stages=[dict(kind='TRAVEL',route=route)])
                batch=make_coordinate_batch(match,(0,0,0),scene)
                self.assertEqual(batch['turn_mode'],mode)
                if mode=='STOP_TURN':self.assertTrue(all(q[4]&1 for q in batch['points']))
                if mode=='MOVING':self.assertFalse(any(q[4]&16 for q in batch['points']))

    def test_mode_is_part_of_cache_identity(self):
        data=competition.load_profile()
        contexts=[]
        for mode in ('MOVING','WHEEL','STOP_TURN'):
            data['turn_mode']=mode
            contexts.append(RouteStore(data,10,core.CHASSIS_DEFAULTS,(280,260)).context)
        self.assertEqual(len(set(contexts)),3)

    def test_intermediate_stop_does_not_complete_before_final_turn(self):
        program=coordinate.build_program([(500,500),(800,500)],0,goal_yaw=90,
            constraints={'turn_mode':'STOP_TURN'})
        tracker=coordinate.CoordinateTracker(program)
        self.assertEqual(len(program['waypoints']),3)
        pose=tracker.points[1]
        for _ in range(9):tracker.command((pose['x_mm'],pose['y_mm'],pose['field_yaw_deg']),None,None)
        self.assertEqual(tracker.index,1)
        for _ in range(5):tracker.command((pose['x_mm'],pose['y_mm'],pose['field_yaw_deg']),None,None)
        self.assertEqual(tracker.index,2)

    def test_selector_changes_signature_and_metadata(self):
        import main
        app=main.QApplication.instance() or main.QApplication([])
        window=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
        try:
            original=copy.deepcopy(window.nav_map['competition']['stations'])
            before=window._navigation_signature()
            window.turn_mode_combo.setCurrentIndex(window.turn_mode_combo.findData('STOP_TURN'))
            self.assertEqual(window.nav_map['turn_mode'],'STOP_TURN')
            self.assertNotEqual(before,window._navigation_signature())
            self.assertEqual(window.nav_map['competition']['stations'],original)
            self.assertEqual(window._journal_metadata()['turn_mode'],'STOP_TURN')
        finally:
            window.close()
            window._planner_pool.shutdown(wait=True,cancel_futures=True)

if __name__=='__main__':unittest.main()
