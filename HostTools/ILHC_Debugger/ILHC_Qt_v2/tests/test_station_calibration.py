"""上位机标定值、右侧作业净空与固定禁区的统一检查。"""
import queue
from pathlib import Path
import unittest

import competition_simulation as competition
import core
from work_orientation import work_heading


class StationCalibrationTests(unittest.TestCase):
    def test_both_maps_use_measured_display_coordinates_without_second_mount_compensation(self):
        for name in ('competition_map', 'navigation_map'):
            data = competition.load_profile(Path(competition.__file__).with_name(name+'.json'))
            scene = competition.collision_scene(data)
            bare = competition.collision_scene(data, 0)
            for station, field in (('raw',(100,1050)), ('rough',(1900,1050)), ('storage',(1100,1900))):
                with self.subTest(map=name,station=station):
                    point = data['competition']['stations'][station]
                    self.assertEqual(core.layout_to_field(*point), field)
                    yaw = work_heading(data['competition'], station)
                    self.assertIsNone(scene.pose_reason(*point, yaw))
                    body = bare.pose_polygon(*point, yaw)
                    if station == 'raw':
                        x,y,r,_ = next(c for c in data['circles'] if c[3]=='原料区圆盘')
                        clearance = body.distance(competition.nav.Point(x,y))-r
                    else:
                        label = '粗加工区设备' if station=='rough' else '暂存区设备'
                        shape = next(shape for shape,text in bare.rect_shapes if text==label)
                        clearance = body.distance(shape)
                    # 暂存车身改为平行设备边缘后，现有轮廓的模型净空为70mm。
                    self.assertAlmostEqual(clearance, {'storage':70,'rough':140,'raw':40}[station])

    def test_work_heading_stays_parallel_to_edge_when_station_moves_along_it(self):
        for name in ('competition_map', 'navigation_map'):
            config = competition.load_profile(Path(competition.__file__).with_name(name+'.json'))['competition']
            for station, expected in (('raw',180),('rough',0),('storage',270)):
                self.assertEqual(work_heading(config,station),expected)
                point = config['stations'][station]
                moved = (point[0],point[1]+50) if station=='storage' else (point[0]+50,point[1])
                self.assertEqual(work_heading(config,station,moved),expected)
            for bad in (float('nan'),float('inf')):
                with self.assertRaises(ValueError):
                    work_heading(dict(config,work_heading_field_deg={'storage':bad}),'storage')

    def test_verified_device_dimensions_keep_measured_clearance_and_fixed_exclusions(self):
        root = Path(competition.__file__).parent
        for name in ('competition_map', 'navigation_map'):
            old = competition.load_profile(root/'Docs'/f'{name}_before_station_calibration_20261007.json')
            new = competition.load_profile(root/(name+'.json'))
            for key in ('bounds','drivable_polygons'):
                self.assertEqual(old[key],new[key])
            for a,b in zip(old['rects'],new['rects']):
                if a[4]=='粗加工区设备':
                    self.assertEqual((b[2]-b[0],b[3]-b[1]),(580,150))
                else:
                    self.assertEqual(a,b)
            for a,b in zip(old['circles'],new['circles']):
                self.assertEqual(b[2],150)
                self.assertEqual(a[3],b[3])
                self.assertEqual(a[0],b[0])

    def test_storage_to_zone_two_route_tolerates_bounded_arrival_pose_residual(self):
        data = competition.load_profile(Path(competition.__file__).with_name('navigation_map.json'))
        match = competition.compile_match(data, zone=2, coordinate_mode=True)
        self.assertGreaterEqual(match['handoff_reserve_mm'], 5)
        program = match['legs'][-1]['route']['waypoint_program']
        point = core.layout_to_field(*data['competition']['stations']['storage'])
        yaw = work_heading(data['competition'], 'storage')
        scene = competition.collision_scene(data)
        for dx,dy,direction in ((.6,.6,1),(.6,-.6,-1),(-.6,.6,-1),(-.6,-.6,1)):
            with self.subTest(dx=dx,dy=dy,direction=direction):
                sim = core.Simulator(queue.Queue(), queue.Queue())
                sim.handle_line('ZERO')
                sim.hold = point[0]+dx, point[1]+dy
                sim.zval = 180+yaw+.9*direction
                sim.make_frame(0)
                epoch = sim.begin_navigation()
                sim.submit_navigation_coordinates(epoch, program, (0,0,0), scene)
                for tick in range(9000):
                    sim.make_frame(tick/core.SEND_HZ)
                    snap = sim.navigation_snapshot()
                    if not snap['tracking']:
                        break
                self.assertEqual(snap['tracking_status'], 'COMPLETE', snap['fault'])
                self.assertFalse(snap['fault'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
