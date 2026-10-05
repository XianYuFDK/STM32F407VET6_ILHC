"""端点执行：无点表、圆弧同时转头、旧模式对比、完整比赛与故障注入。"""
import copy
import json
import math
from pathlib import Path
import queue
import tempfile
import threading
import unittest
from unittest.mock import patch

import core
import competition_simulation as competition
from segment_route import build_segment_program, export_segment_json, validate_segment_program
from trajectory_tracking import SegmentTracker, TrajectoryTracker
from tests.test_continuous_tracking import prepared
from tests.test_competition_simulation import runner_for, advance


def segment_sim(result, scene):
    sim = core.Simulator(queue.Queue(), queue.Queue())
    sim.handle_line('ZERO')
    first = result['segment_program']['start']
    sim.hold = first['x_mm'], first['y_mm']
    sim.zval = 90-first['field_yaw_deg']
    sim.make_frame(0)
    sim.submit_navigation_segments(sim.begin_navigation(), result['segment_program'], (0, 0, 0), scene)
    return sim


class SegmentRouteTests(unittest.TestCase):
    def test_collinear_endpoints_merge_and_json_contains_no_sample_table(self):
        program = build_segment_program([dict(kind='LINE', start=(500, 500), end=(800, 500)),
                                         dict(kind='LINE', start=(800, 500), end=(1500, 500))])
        self.assertEqual(len(program['segments']), 1)
        self.assertEqual(program['segments'][0]['end'], [1500, 500])
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'route.json'
            export_segment_json(path, program)
            loaded = json.loads(path.read_text(encoding='utf-8'))
        self.assertNotIn('trajectory', loaded)
        self.assertEqual(loaded['segments'], program['segments'])
        self.assertFalse(loaded['hardware_ready'])

    def test_segment_tracker_needs_only_geometry_and_matches_old_commands(self):
        result, rows, _scene = prepared()
        new, old = SegmentTracker(rows), TrajectoryTracker(result['trajectory'], rows)
        for station in range(0, int(new.length), 5):
            r = new.reference_at(station)
            pose = r['x_mm'], r['y_mm'], r['field_yaw_deg']
            nr, nv, nw = new.command(pose, 250, 120)
            or_, ov, ow = old.command(pose, 250, 120)
            self.assertEqual(nr, or_)
            self.assertEqual(nv, ov)
            self.assertEqual(nw, ow)
        self.assertFalse(hasattr(new, 'samples'))

    def test_arc_moves_and_changes_heading_without_waiting_at_endpoints(self):
        result, rows, scene = prepared()
        sim = segment_sim(result, scene)
        count = len(sim._nav_tracker.pieces)
        self.assertLess(count, len(result['trajectory'])/10)
        moving_arc, minimum_arc_speed, previous = [], math.inf, None
        for i in range(1600):
            sim.make_frame(i/core.SEND_HZ)
            snap = sim.navigation_snapshot()
            actual = (*core.field_to_layout(*snap['hold']), -180+snap['yaw'])
            self.assertIsNone(scene.pose_reason(*actual))
            if previous:
                self.assertIsNone(scene.moving_pose_reason(previous, actual))
            previous = actual
            ref = snap['reference']
            if ref and ref['segment_type']=='ARC' and snap['progress_s_mm'] > 900:
                moving_arc.append(snap['yaw'])
                minimum_arc_speed = min(minimum_arc_speed, snap['speed_mm_s'])
            if not snap['active']:
                break
        self.assertEqual(snap['tracking_status'], 'COMPLETE', snap)
        self.assertEqual(snap['execution_representation'], 'SEGMENTS')
        self.assertGreater(max(moving_arc)-min(moving_arc), 30)
        self.assertGreater(minimum_arc_speed, 100)
        self.assertGreaterEqual(snap['settled_frames'], 10)

    def test_program_still_requires_full_rectangular_collision_recheck(self):
        result, _rows, scene = prepared()
        # 圆弧车体覆盖区加入障碍，端点安全不能代替中途车体检查。
        center = result['segment_program']['segments'][1]['center']
        from tests.test_arc_smoothing import scene as make_scene
        scene = make_scene(circles=[(center[0]+85, center[1]-85, 25, '弯道新增障碍')])
        with self.assertRaises(ValueError):
            validate_segment_program(result['segment_program'], scene)

    def test_invalid_geometry_and_turn_fallback_are_not_implicitly_connected(self):
        for rows in ([dict(kind='LINE', start=(0,0), end=(100,0)), dict(kind='LINE',start=(200,0),end=(300,0))],
                     [dict(kind='LINE', start=(0,0),end=(100,0)),dict(kind='LINE',start=(100,0),end=(100,100))],
                     [dict(kind='TURN')]):
            with self.assertRaises(ValueError):
                build_segment_program(rows)

    def test_stop_and_map_version_change_cancel_without_residual_motion(self):
        for mode in ('stop', 'map'):
            result, _rows, scene = prepared()
            sim = segment_sim(result, scene)
            sim.make_frame(0)
            before = sim.hold, sim.zval
            if mode=='stop':
                sim.cancel_navigation()
            else:
                sim._nav_validity = lambda: False
            sim.make_frame(.02)
            self.assertEqual((sim.hold, sim.zval), before)
            self.assertFalse(sim.navigation_snapshot()['tracking'])

    def test_stop_during_segment_recheck_does_not_block_or_accept_old_epoch(self):
        result, _rows, scene = prepared()
        sim = core.Simulator(queue.Queue(), queue.Queue())
        sim.handle_line('ZERO')
        sim.hold = result['segment_program']['start']['x_mm'], result['segment_program']['start']['y_mm']
        sim.zval = 90-result['segment_program']['start']['field_yaw_deg']
        epoch = sim.begin_navigation()
        entered, release, failures = threading.Event(), threading.Event(), []
        real = validate_segment_program
        def blocked(*args, **kwargs):
            entered.set()
            if not release.wait(2): raise RuntimeError('复检等待超时')
            return real(*args, **kwargs)
        def submit():
            try: sim.submit_navigation_segments(epoch, result['segment_program'], (0,0,0), scene)
            except Exception as exc: failures.append(exc)
        with patch('segment_route.validate_segment_program', side_effect=blocked):
            thread = threading.Thread(target=submit)
            thread.start()
            self.assertTrue(entered.wait(2))
            sim.cancel_navigation()
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertFalse(sim.navigation_snapshot()['tracking'])


class SegmentCompetitionTests(unittest.TestCase):
    def test_entire_match_executes_after_removing_all_dense_route_points(self):
        match = competition.compile_match(competition.load_profile(), zone=1)
        for stage in match['stages']:
            if stage['kind']=='TRAVEL':
                stage['route'].pop('trajectory')
                stage['route'].pop('smoothed_primitives')
        runner = runner_for(match)
        for _ in range(9000):
            if not runner.active: break
            advance(runner)
        self.assertEqual(runner.status, 'COMPLETE', runner.reason)
        self.assertEqual((runner.grabs, runner.placements), (12, 12))


if __name__ == '__main__':
    unittest.main(verbosity=2)
