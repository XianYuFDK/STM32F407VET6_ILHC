"""Map-click regressions using actual production imports and actual UI methods.

No PySide6 objects here: QWidget sinks are fakes, while the planner thread,
geometry and simulator are real. See test_map_click_qt.py for real Qt events.
"""
from __future__ import annotations

import ast
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import core
from tests.headless_support import ROOT, namespace, make_window, close_window


class MapClickRegressionTests(unittest.TestCase):
    def setUp(self):
        self.w = make_window()
        self.temp = tempfile.TemporaryDirectory(prefix='ilhc-click-test-')
        self.w._planner_error_log = Path(self.temp.name) / 'planner-errors.log'

    def tearDown(self):
        close_window(self.w)
        self.temp.cleanup()

    def wait_result(self):
        deadline = time.monotonic() + 10.0
        while self.w._plan_future is not None and time.monotonic() < deadline:
            self.w._poll_plan()
            time.sleep(0.005)
        self.assertIsNone(self.w._plan_future, 'planner did not finish')

    def assert_no_motion(self):
        self.assertTrue(self.w.line_q.empty())
        self.assertTrue(self.w.urgent_q.empty())
        self.assertIsNone(self.w.follow)
        if self.w.sim is not None:
            self.assertIsNone(self.w.sim.navigation_snapshot()['goto'])

    def assert_failed_visibly(self, text):
        self.assertIn(text, self.w.map_status.text)
        self.assertEqual(self.w.plan_info.props.get('state'), 'bad')
        self.assertEqual(self.w.planned_points, [])
        self.assertIsNone(self.w.planned_result)
        self.assertIsNone(self.w._plan_future)
        self.assertIsNone(self.w._plan_context_pending)
        self.assert_no_motion()

    def test_threading_binding_comes_from_main_import(self):
        source = ast.parse((ROOT / 'main.py').read_text(encoding='utf-8-sig'))
        actual = [a.name for n in source.body if isinstance(n, ast.Import) for a in n.names]
        self.assertIn('threading', actual)
        self.assertIn('threading', namespace)
        self.assertEqual(namespace['threading'].__name__, 'threading')

    def test_actual_click_handler_starts_worker_and_displays_result(self):
        self.w._on_map_click(330.0, 1200.0)
        self.assertIn('正在规划', self.w.map_status.text)
        self.wait_result()
        self.assertTrue(self.w.planned_result['ok'])
        self.assertEqual(self.w._planned_context['goal'], (330.0, 1200.0))
        self.assertTrue(self.w.map_view.points)
        self.assertIn('已规划', self.w.map_status.text)
        self.assert_no_motion()

    def test_offline_click_plans_without_simulator_or_serial(self):
        self.w.sim = None
        self.w._on_map_click(330.0, 1200.0)
        self.wait_result()
        self.assertTrue(self.w.planned_result['ok'])
        self.assertEqual(self.w.planned_result['mode'], 'OFFLINE_PREVIEW_ONLY')
        self.assert_no_motion()

    def test_home_button_uses_async_planning_in_planning_mode(self):
        self.w.zone_combo.v = 2
        self.w._goto_home()
        self.wait_result()
        self.assertTrue(self.w.planned_result['ok'])
        self.assertEqual(self.w._planned_context['goal'], core.ZONE_CENTER[2])
        self.assert_no_motion()

    def test_invalid_goal_shows_rejection_instead_of_silent_click(self):
        self.w._on_map_click(700.0, 700.0)
        self.wait_result()
        self.assert_failed_visibly('规划失败')

    def test_nonfinite_goal_is_rejected_before_worker(self):
        self.w._on_map_click(float('nan'), 1200.0)
        self.assert_failed_visibly('规划拒绝')

    def test_missing_production_import_is_not_injected_by_test_helper(self):
        # Mutation: deliberately recreate the shipped bug. The actual handler
        # must show NameError, not succeed using this test module's globals.
        with patch.dict(namespace):
            namespace.pop('threading', None)
            self.w._on_map_click(330.0, 1200.0)
        self.assert_failed_visibly('NameError')
        self.assertIn('threading', self.w.map_status.text)
        log = self.w._planner_error_log.read_text(encoding='utf-8')
        self.assertIn('Traceback', log)
        self.assertIn("name 'threading' is not defined", log)

    def test_submit_error_is_visible_and_next_click_can_recover(self):
        with patch.object(self.w._planner_pool, 'submit', side_effect=RuntimeError('pool unavailable')):
            self.w._on_map_click(330.0, 1200.0)
        self.assert_failed_visibly('pool unavailable')
        self.w.sim.make_frame(0.0)
        self.w._on_map_click(330.0, 1200.0)
        self.wait_result()
        self.assertTrue(self.w.planned_result['ok'])
        self.assert_no_motion()

    def test_event_creation_error_is_visible(self):
        with patch.object(namespace['threading'], 'Event', side_effect=RuntimeError('event failure')):
            self.w._on_map_click(330.0, 1200.0)
        self.assert_failed_visibly('event failure')

    def test_unexpected_preparation_error_is_visible(self):
        with patch.object(self.w, '_prepare_plan', side_effect=RuntimeError('snapshot failure')):
            self.w._on_map_click(330.0, 1200.0)
        self.assert_failed_visibly('snapshot failure')

    def test_worker_error_is_visible(self):
        with patch.object(core, 'plan_path', side_effect=RuntimeError('geometry failure')):
            self.w._on_map_click(330.0, 1200.0)
            self.wait_result()
        self.assert_failed_visibly('geometry failure')
        self.assertIn('规划计算失败', self.w.map_status.text)

    def test_result_presentation_error_clears_partial_plan(self):
        with patch.object(self.w, '_finish_plan', side_effect=ValueError('presentation failure')):
            self.w._on_map_click(330.0, 1200.0)
            self.wait_result()
        self.assert_failed_visibly('presentation failure')
        self.assertIn('规划结果处理失败', self.w.map_status.text)

    def test_log_io_error_does_not_hide_original_error(self):
        # Opening a directory as a log file must fail without breaking UI handling.
        self.w._planner_error_log = Path(self.temp.name)
        with patch.object(self.w._planner_pool, 'submit', side_effect=RuntimeError('original error')):
            self.w._on_map_click(330.0, 1200.0)
        self.assert_failed_visibly('original error')
        self.assertTrue(any('日志写入失败' in text for text in self.w.logs))

    def test_stale_exception_cannot_clear_newer_plan(self):
        old = self.w._prepare_plan(2250.0, 1600.0)
        self.w.sim.make_frame(0.0)
        self.w._on_map_click(330.0, 1200.0)
        self.wait_result()
        current = self.w.planned_result
        before = self.w.map_status.text
        self.assertIsNone(self.w._report_plan_exception('计算', RuntimeError('old error'), old))
        self.assertIs(self.w.planned_result, current)
        self.assertEqual(self.w.map_status.text, before)
        self.assert_no_motion()

    def test_cancel_clears_pending_request_and_event_reference(self):
        self.w._on_map_click(330.0, 1200.0)
        old_event = self.w._plan_cancel
        self.w._clear_path()
        self.w._poll_plan()
        self.assertTrue(old_event.is_set())
        self.assertIsNone(self.w._plan_cancel)
        self.assertIsNone(self.w._plan_context_pending)
        self.assertEqual(self.w.planned_points, [])
        self.assert_no_motion()

    def test_latest_click_wins(self):
        self.w._on_map_click(330.0, 1200.0)
        self.w.sim.make_frame(0.0)
        self.w._on_map_click(1200.0, 330.0)
        self.wait_result()
        self.assertTrue(self.w.planned_result['ok'])
        self.assertEqual(self.w._planned_context['goal'], (1200.0, 330.0))
        self.assertEqual(self.w.map_view.points[-1], (1200.0, 330.0))
        self.assert_no_motion()


if __name__ == '__main__':
    unittest.main(verbosity=2)
