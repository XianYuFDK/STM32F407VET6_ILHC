"""右侧塔吊实际作业姿态、固定路线隔离及新增障碍回归，无串口。"""
import gzip
import json
import math
import tempfile
import unittest
from unittest.mock import patch

import competition_simulation as competition
import core
from coordinate_navigation import build_program, replay, wrap
from hardware_coordinates import make_coordinate_batch
from route_store import RouteStore, clear_memory
from tests.test_competition_simulation import runner_for, advance
from work_orientation import work_heading, station_at


class WorkOrientationTests(unittest.TestCase):
    def test_right_vector_points_at_each_area_and_moves_with_station(self):
        config = competition.load_profile()['competition']
        config.pop('work_heading_field_deg', None)  # 旧地图仍按设备中心求航向。
        for station, expected in (('raw', 180), ('rough', 0), ('storage', 259.695153531234)):
            self.assertAlmostEqual(work_heading(config, station), expected)
            for point in (config['stations'][station], (1000, 1200)):
                yaw = work_heading(config, station, point)
                angle = math.radians(yaw-90)
                vector = math.cos(angle), math.sin(angle)
                target = config['work_areas'][station]
                towards = target[0]-point[0], target[1]-point[1]
                self.assertAlmostEqual(sum(a*b for a, b in zip(vector, towards))/math.hypot(*towards), 1)

    def test_invalid_work_area_cannot_silently_use_old_heading(self):
        config = competition.load_profile()['competition']
        for value in (None, {}, {'raw': [math.nan, 0]}, {'raw': config['stations']['raw']}):
            with self.subTest(value=value), self.assertRaises((ValueError, TypeError)):
                work_heading(dict(config, work_areas=value), 'raw')
        self.assertIsNone(station_at({}, (400, 1200)))
        point = config['stations']['storage']
        self.assertEqual(station_at(config, point), 'storage')
        self.assertIsNone(station_at(config, (point[0]+5, point[1])))
        self.assertIsNone(station_at(config, (400, 1200)))


class RouteStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.data = competition.load_profile()
        self.store = RouteStore(self.data, 10, core.CHASSIS_DEFAULTS, (280, 260), root=self.temp.name)
        self.start, self.goal = (1200, 1700), (1200, 2030)
        program = build_program([self.start, self.goal], 180, goal_yaw=180, mode='FIXED')
        samples, _ = replay(program, competition.collision_scene(self.data))
        self.route = dict(waypoint_program=program, execution_safe=True, trajectory_safe=True, trajectory=samples)
        self.store.put(self.start, self.goal, 180, 180, self.route)

    def tearDown(self):
        clear_memory()
        self.temp.cleanup()

    def test_disk_roundtrip_and_returned_copy_cannot_mutate_saved_program(self):
        clear_memory()
        route = self.store.get(self.start, self.goal, 180.0, -180)
        self.assertEqual(route['route_reuse']['source'], 'DISK')
        route['waypoint_program']['waypoints'][-1]['layout_yaw_deg'] = 90
        again = self.store.get(self.start, self.goal, 180, 180)
        self.assertEqual(again['waypoint_program']['waypoints'][-1]['layout_yaw_deg'], 180)

    def test_cli_and_qt_equivalent_integer_float_values_share_disk_routes(self):
        clear_memory()
        control = {k: int(v) if float(v).is_integer() else v for k, v in core.CHASSIS_DEFAULTS.items()}
        store = RouteStore(self.data, 10.0, control, (280.0, 260.0), root=self.temp.name)
        route = store.get(tuple(map(float, self.start)), tuple(map(float, self.goal)), 180.0, 180.0)
        self.assertEqual(route['route_reuse']['source'], 'DISK')

    def test_geometry_gain_margin_and_algorithm_changes_invalidate_cache(self):
        variants = [dict(self.data, map_version='changed'),
                    dict(self.data, rects=self.data['rects']+[[1100, 1750, 1300, 1800, '新禁区']]),
                    dict(self.data, wheel_geometry={'wheelbase_mm': 300, 'track_mm': 218})]
        for data in variants:
            self.assertIsNone(RouteStore(data, 10, core.CHASSIS_DEFAULTS, (280, 260), root=self.temp.name)
                              .get(self.start, self.goal, 180, 180))
        self.assertIsNone(RouteStore(self.data, 15, core.CHASSIS_DEFAULTS, (280, 260), root=self.temp.name)
                          .get(self.start, self.goal, 180, 180))
        self.assertIsNone(RouteStore(self.data, 10, dict(core.CHASSIS_DEFAULTS, kpx=7), (280, 260), root=self.temp.name)
                          .get(self.start, self.goal, 180, 180))
        with patch('route_store.algorithm_revision', return_value='changed'):
            self.assertIsNone(RouteStore(self.data, 10, core.CHASSIS_DEFAULTS, (280, 260), root=self.temp.name)
                              .get(self.start, self.goal, 180, 180))

    def test_corrupted_or_cancelled_cache_cannot_supply_motion(self):
        clear_memory()
        path = self.store.path(self.store.identity(self.start, self.goal, 180, 180))
        envelope = json.loads(gzip.decompress(path.read_bytes()))
        envelope['route']['waypoint_program']['waypoints'][-1]['layout_yaw_deg'] = 90
        path.write_bytes(gzip.compress(json.dumps(envelope).encode('utf-8')))
        self.assertIsNone(self.store.get(self.start, self.goal, 180, 180))
        with self.assertRaisesRegex(ValueError, '取消'):
            self.store.get(self.start, self.goal, 180, 180, lambda: True)


class RightCraneMatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = competition.load_profile()
        cls.matches = {zone: competition.compile_match(cls.data, zone=zone, coordinate_mode=True)
                       for zone in (1, 2)}

    def test_no_obstacles_use_disk_routes_and_new_code_keeps_correct_actions(self):
        clear_memory()
        with patch('coordinate_navigation.plan_route', side_effect=AssertionError('无障碍不得重新搜索')):
            match = competition.compile_match(self.data, task_code='642+312+264+123', margin=10.0, coordinate_mode=True)
        self.assertEqual(match['planning_stats']['search_calls'], 0)
        self.assertEqual(match['route_policy'], 'FIXED_VERIFIED')
        self.assertEqual([s['color'] for s in match['stages'] if s['kind']=='RAW_PICK'], [6, 4, 2, 2, 6, 4])
        self.assertFalse(any(s['route']['route_reuse']['source']=='SEARCH' for s in match['stages'] if s['kind']=='TRAVEL'))

    def test_both_zones_actual_station_heading_and_quantized_batch_are_correct(self):
        for zone, match in self.matches.items():
            with self.subTest(zone=zone):
                runner = runner_for(match)
                actions = set()
                while runner.active:
                    stage = runner.stage
                    if runner.status == 'ACTION' and 'work_heading_deg' in stage:
                        pose = runner._field_pose(runner.sim.navigation_snapshot())
                        self.assertLess(abs(wrap(-90-pose[2]-stage['work_heading_deg'])), 1)
                        actions.add(stage['work_station'])
                    advance(runner)
                self.assertEqual(runner.status, 'COMPLETE', runner.reason)
                self.assertEqual(actions, {'raw', 'rough', 'storage'})
                self.assertLess(runner.elapsed_s, 180)
                batch = make_coordinate_batch(match, (0, 0, 0), competition.collision_scene(self.data), token=zone)
                for index, labels in batch['stations'].items():
                    q = batch['points'][index]
                    station = 'raw' if any('取料' in label for label in labels) else 'rough' if any('粗加工' in label for label in labels) else 'storage' if any('暂存' in label for label in labels) else None
                    if station:
                        self.assertLess(abs(wrap(q[3]/100-180-work_heading(self.data['competition'], station))), .02)

    def test_added_obstacle_forces_revalidation_and_safe_alternative(self):
        match = competition.compile_match(self.data, sim_obstacles=[(1200, 1200)], coordinate_mode=True)
        self.assertEqual(match['route_policy'], 'ADAPTIVE_VERIFIED')
        self.assertGreater(match['planning_stats']['search_calls'], 0)
        self.assertTrue(all(l['route']['route_reuse']['extra_obstacles'] for l in match['legs']))
        runner = runner_for(match)
        while runner.active:
            advance(runner)
        self.assertEqual(runner.status, 'COMPLETE', runner.reason)


if __name__ == '__main__':
    unittest.main(verbosity=2)
