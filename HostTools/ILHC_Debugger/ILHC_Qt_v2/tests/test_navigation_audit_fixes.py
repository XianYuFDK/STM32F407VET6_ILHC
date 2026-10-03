"""2026-10-02 算法审查回归：独立几何、代价对照与真实界面方法。"""
import heapq
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union

import navigation_planner as nav
from tests.headless_support import make_window, close_window, WindowMethods


def options(**changes):
    values = dict(grid=100, pad=10, rects=[], circles=[], bounds=(0, 0, 2400, 2400),
                  footprint=(280, 260, 0), start_heading_deg=0, goal_heading_deg=0,
                  time_limit_s=8)
    values.update(changes)
    return values


class AuditPlannerFixTests(unittest.TestCase):
    def test_direct_candidate_does_not_skip_cheaper_turns(self):
        r = nav.plan((500, 500), (500, 1500), **options())
        self.assertTrue(r['ok'], r['reason'])
        self.assertEqual(r['search_cost'], 1360)
        self.assertEqual(r['trace_cost'], 1360)
        self.assertEqual(r['lateral_mm'], 0)
        self.assertEqual(r['turn_count'], 2)

    def test_cheapest_move_heuristic_matches_independent_dijkstra(self):
        # 开放小网格，端点恰在格点：所有连接边都是下列网格动作的合法展开。
        # 不调用生产 action/turn/cost helper，避免把同一实现错误抄进参照。
        vectors = ((1, 0), (0, 1), (-1, 0), (0, -1))
        for factors in ((1, 1.15, 1.8), (2, .4, .6), (.8, 1.5, .3)):
            start, goal = (1, 1, 0), (8, 7, 0)
            dist, queue = {start: 0}, [(0, start)]
            while queue:
                cost, st = heapq.heappop(queue)
                if cost != dist[st]:
                    continue
                if st == goal:
                    break
                edges = []
                for dx, dy in vectors:
                    x, y, h = st[0]+dx, st[1]+dy, st[2]
                    if not (0 <= x <= 10 and 0 <= y <= 10):
                        continue
                    dot = dx*vectors[h][0] + dy*vectors[h][1]
                    factor = factors[0] if dot > 0 else factors[1] if dot < 0 else factors[2]
                    edges.append(((x, y, h), 100*factor))
                for delta in (1, 2, 3):
                    edges.append(((st[0], st[1], (st[2]+delta) % 4),
                                  60 * (2 if delta == 2 else 1)))
                for nxt, weight in edges:
                    nc = cost + weight
                    if nc < dist.get(nxt, math.inf):
                        dist[nxt] = nc
                        heapq.heappush(queue, (nc, nxt))
            r = nav.plan((100, 100), (800, 700), **options(
                pad=0, footprint=None, bounds=(0, 0, 1000, 1000),
                cost_forward=factors[0], cost_backward=factors[1],
                cost_lateral=factors[2], turn_penalty_mm=60))
            self.assertTrue(r['ok'], r['reason'])
            self.assertAlmostEqual(r['search_cost'], dist[goal])
            self.assertAlmostEqual(r['trace_cost'], dist[goal])

    def test_turn_probe_is_not_returned_as_safe_in_place(self):
        obstacle = (1233.2976841766317, 1386.2029970703127, .9866133478979671, '反例障碍')
        local_probe = Point(obstacle[0]-1200, obstacle[1]-1200)
        actual = Polygon(nav.Footprint(280, 260, 122).vertices())
        self.assertLessEqual(actual.distance(local_probe), obstacle[2])
        scene = nav.CollisionScene([], [obstacle], (0, 0, 2400, 2400), 10, (280, 260, 90))
        self.assertIsNotNone(scene.turn_reason((1200, 1200), 90, 180))
        r = nav.plan((1200, 1200), (1200, 1200), **options(
            footprint=(280, 260, 90), start_heading_deg=90, goal_heading_deg=180,
            circles=[obstacle]))
        self.assertTrue(r['ok'], r['reason'])  # 移到净空处转向，再回来。
        self.assertGreater(r['search_cost'], 180)
        self.assertGreater(r['length'], 0)
        for row in r['steps']:
            if row['kind'] == 'TURN' and row['x'] == 1200 and row['y'] == 1200:
                self.assertNotEqual(row['heading_deg'], 180)

    def test_sample_gap_sweep_covers_dense_rotations_for_all_start_headings(self):
        for begin in (0, 15, 90, 180, 270, 359.8):
            for delta in (90, -90, 180):
                sweep = nav.rotation_sweep((280, 260, begin), 10, begin, begin+delta)
                # 采样步长0.5°，与生产15°采样独立；检查完整矩形，而非只检查车心。
                for i in range(abs(delta)*2+1):
                    angle = begin + math.copysign(i*.5, delta)
                    actual = Polygon(nav.Footprint(280, 260, angle).vertices(10))
                    self.assertTrue(sweep.covers(actual), (begin, delta, angle))

    def test_arbitrary_initial_yaw_remains_exact_in_every_pose(self):
        for yaw in (.2, 15, 45, 89, 180.2, 359.8):
            r = nav.plan((700, 700), (1700, 1100), **options(
                footprint=(280, 260, yaw), start_heading_deg=yaw, goal_heading_deg=yaw))
            self.assertTrue(r['ok'], (yaw, r['reason']))
            self.assertAlmostEqual(r['start_heading_deg'], yaw)
            self.assertAlmostEqual(r['steps'][0]['heading_deg'], yaw)
            self.assertAlmostEqual(r['steps'][-1]['heading_deg'], yaw)
            for row in r['steps']:
                residual = ((row['heading_deg']-yaw+45) % 90)-45
                self.assertAlmostEqual(residual, 0)

    def test_actual_rotated_body_still_rejects_illegal_boundary_start(self):
        r = nav.plan((2250, 2250), (2000, 2100), **options(
            footprint=(280, 260, .2), start_heading_deg=.2, goal_heading_deg=.2))
        self.assertEqual(r['code'], 'INVALID_START')
        mismatch = nav.plan((700, 700), (1700, 1100), **options(start_heading_deg=.2))
        self.assertEqual(mismatch['code'], 'INVALID_INPUT')

    def test_collinear_turn_preserves_action_segments(self):
        r = nav.plan((500, 300), (500, 1500), **options(
            pad=0, bounds=(0, 0, 2400, 1635),
            rects=[(0, 0, 350, 1000, '左墙'), (650, 0, 1000, 1000, '右墙')],
            footprint=(280, 260, 90), start_heading_deg=90, goal_heading_deg=0))
        self.assertTrue(r['ok'], r['reason'])
        self.assertTrue(r['axis_matched'])
        self.assertEqual(r['points'], [(500., 300.), (500., 1500.)])
        moves = [row for row in r['steps'] if row['kind'] == 'MOVE']
        self.assertGreaterEqual(len(r['segments']), 2)
        self.assertEqual(len(r['segments']), len(moves))
        for seg, row in zip(r['segments'], moves):
            self.assertEqual((seg.start, seg.end), ((row['x'], row['y']), (row['to_x'], row['to_y'])))
            self.assertEqual((seg.heading_deg, seg.action), (row['heading_deg'], row['action']))

    def test_operation_zone_does_not_exempt_outside_part(self):
        zone = [(450, 450), (550, 450), (550, 550), (450, 550)]
        r = nav.plan((500, 500), (500, 1500), **options(
            strafe_run_limit_mm=0, strafe_polygons=[zone]))
        self.assertTrue(r['ok'], r['reason'])
        for row in r['steps']:
            if row['action'] == 'STRAFE':
                line = LineString([(row['x'], row['y']), (row['to_x'], row['to_y'])])
                self.assertLessEqual(line.difference(Polygon(zone)).length, nav.EPS)

    def test_operation_zone_only_deducts_its_inside_distance(self):
        zone = [(450, 650), (550, 650), (550, 850), (450, 850)]
        r = nav.plan((500, 500), (500, 1500), **options(
            strafe_run_limit_mm=800, strafe_polygons=[zone], turn_penalty_mm=5000))
        self.assertTrue(r['ok'], r['reason'])
        self.assertEqual(r['search_cost'], 1800)
        self.assertEqual(r['max_strafe_run_mm'], 1000)
        self.assertEqual(r['max_outside_strafe_run_mm'], 800)
        self.assertEqual(r['turn_count'], 0)

    def test_fine_grid_does_not_repeatedly_round_up_strafe_allowance(self):
        costs = []
        for grid in (100, 10):
            r = nav.plan((800, 800), (800, 1350), **options(
                grid=grid, strafe_run_limit_mm=500, turn_penalty_mm=5000))
            self.assertTrue(r['ok'], r['reason'])
            self.assertLessEqual(r['max_strafe_run_mm'], 500+nav.EPS)
            self.assertLess(r['search_cost'], 2000)
            costs.append(r['search_cost'])
        self.assertLessEqual(costs[1], costs[0]+nav.EPS)

    def test_body_clearance_includes_turn_only_path(self):
        r = nav.plan((1200, 1200), (1200, 1200), **options(goal_heading_deg=90))
        self.assertTrue(r['ok'], r['reason'])
        self.assertEqual(r['length'], 0)
        self.assertIsNotNone(r['body_clearance'])
        self.assertLess(r['body_clearance'], 1200-140)

    def test_revisiting_same_segment_and_corner_uses_correct_ledger_occurrence(self):
        points = [(0, 0), (100, 0), (100, 100), (0, 100), (0, 0), (100, 0), (100, 100)]
        def row(a, b, heading, action):
            return dict(x=a[0], y=a[1], to_x=b[0], to_y=b[1],
                        heading_deg=heading, action=action)
        steps = [row((0, 0), (100, 0), 0, 'FORWARD'),
                 row((100, 0), (100, 0), 90, 'TURN_LEFT'),
                 row((100, 0), (100, 100), 90, 'FORWARD'),
                 row((100, 100), (0, 100), 90, 'STRAFE'),
                 row((0, 100), (0, 0), 90, 'BACKWARD'),
                 row((0, 0), (0, 0), 0, 'TURN_RIGHT'),
                 row((0, 0), (100, 0), 0, 'FORWARD'),
                 row((100, 0), (100, 0), 180, 'TURN_AROUND'),
                 row((100, 0), (100, 100), 180, 'STRAFE')]
        segments, corners = nav.path_axes(points, steps)
        self.assertEqual(segments[-1].heading_deg, 180)
        self.assertEqual(segments[-1].action, 'STRAFE')
        repeated_corners = [c for c in corners if c.point == (100, 0)]
        self.assertEqual([(c.action, c.steps) for c in repeated_corners],
                         [('TURN_LEFT', 1), ('TURN_AROUND', 2)])


class AuditUiFixTests(unittest.TestCase):
    def setUp(self):
        self.w = make_window()

    def tearDown(self):
        close_window(self.w)

    def test_actual_ui_accepts_non_cardinal_yaw_without_sending_commands(self):
        self.w.nav_map['rects'] = []
        self.w.nav_map['circles'] = []
        self.w.sim.hold = (1050, 1050)  # 布局(1200,1200)，离边界与障碍有足够净空。
        self.w.sim.zval = .2
        self.w.sim.make_frame(0)
        r = self.w.plan_to(1800, 1200)
        self.assertTrue(r['ok'], r['reason'])
        self.assertAlmostEqual(r['start_heading_deg'], 180.2)
        self.assertTrue(self.w.line_q.empty() and self.w.urgent_q.empty())

    def test_json_export_roundtrips_segments_and_corners(self):
        self.w.sim.make_frame(0)
        r = self.w.plan_to(2100, 2050)
        self.assertTrue(r['ok'], r['reason'])
        self.assertTrue(r['segments'] and r['corners'])
        with tempfile.TemporaryDirectory(prefix='ilhc-export-test-') as folder:
            path = Path(folder) / 'plan.json'
            dialog = SimpleNamespace(getSaveFileName=lambda *a: (str(path), ''))
            with patch.dict(WindowMethods._export_navigation_plan.__globals__, QFileDialog=dialog):
                self.w._export_navigation_plan()
            data = json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(data['segments'][0]['action'], r['segments'][0].action)
            self.assertEqual(data['corners'][0]['action'], r['corners'][0].action)
            self.assertEqual(data['steps'], r['steps'])
            self.assertEqual(data['map_snapshot'], self.w.nav_map)

    def test_unmatched_action_segments_are_rejected_before_presenting_success(self):
        context = self.w._prepare_plan(2100, 2050)
        r = nav.plan(context['start'], context['goal'], **context['kwargs'])
        r['axis_matched'] = False
        result = self.w._finish_plan(r, context)
        self.assertFalse(result['ok'])
        self.assertIsNone(self.w.planned_result)
        self.assertIn('台账不匹配', result['reason'])

    def test_quantized_turn_checks_actual_sweep_instead_of_skipping_turn(self):
        self.w.nav_map['rects'] = []
        self.w.nav_map['circles'] = []
        context = self.w._prepare_plan(2100, 2050)
        context['start'] = (1200, 1200)
        context['kwargs']['circles'] = [(1233.2976841766317, 1386.2029970703127,
                                        .9866133478979671, '反例障碍')]
        result = {'start_heading_deg': 90, 'steps': [dict(
            x=1200, y=1200, to_x=1200, to_y=1200, heading_deg=180, kind='TURN')]}
        self.assertEqual(self.w._validate_quantized(result, context), '反例障碍')

    def test_every_turn_appears_in_ui_ledger_even_without_a_corner(self):
        self.w.nav_map['rects'] = []
        self.w.nav_map['circles'] = []
        self.w.sim.hold = (1750, 1750)  # 布局(500,500)
        self.w.sim.make_frame(0)
        r = self.w.plan_to(500, 1500)
        self.assertTrue(r['ok'], r['reason'])
        self.assertEqual(len(r['corners']), 0)
        self.assertEqual(r['turn_count'], 2)
        text = self.w.plan_text.text
        self.assertIn('TURN_LEFT', text)
        self.assertIn('TURN_RIGHT', text)
        self.assertNotIn('GOTO=', text)


if __name__ == '__main__':
    unittest.main(verbosity=2)
