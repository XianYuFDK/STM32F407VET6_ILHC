"""无硬件回归：坐标、导航入口、步进命令和模拟器停止语义。"""
import argparse
import math
import os
import queue
import re
import struct
import time
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import core
import main


class DebuggerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.window = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.forward_invert_default = self.window.manual_invert[1].isChecked()
        # 原协议测试使用未反转基线，实车默认方向另行验证。
        self.window.manual_invert[1].setChecked(False)
        # 使用未启动的模拟器作为离线命令接收端，测试不会打开串口。
        self.window.sim = core.Simulator(queue.Queue(), queue.Queue())
        self.window.sim.handle_line("ZERO")
        self.window.sim.make_frame(0.0)

    def _wait_plan(self):
        deadline = time.monotonic() + 10.0
        while self.window._plan_future is not None and time.monotonic() < deadline:
            self.app.processEvents()
            self.window._poll_plan()
            time.sleep(0.005)
        self.assertIsNone(self.window._plan_future, "后台规划超时")

    def tearDown(self):
        self.window.close()

    def test_origin_and_axes(self):
        self.assertEqual(core.layout_to_field(2250, 2250), (0, 0))
        self.assertEqual(core.layout_to_field(2150, 2150), (100, 100))
        self.assertEqual(core.layout_to_field(*core.ZONE_CENTER[2]), (2100, 0))
        self.assertEqual(self.window._ops_to_field(0, 0), (2250, 2250))
        # 内部测试输入为mm：场地向上100mm = 内部 +100mm；协议发送时对应 GOTO Y=10.0cm。
        self.assertEqual(self.window._field_to_ops(2150, 2250), (0, 100))
        # 内部测试输入为mm：场地向左100mm = 内部 +100mm；协议发送时对应 GOTO X=10.0cm。
        self.assertEqual(self.window._field_to_ops(*core.field_to_layout(100, 0)), (100, 0))

    def test_telemetry_direction_matches_field_axes(self):
        """验收桩：向前=+Y、向左=+X；地图上前进→屏幕上、左移→屏幕左，且点击方向一致。

        屏幕方向来自 test_clockwise_view_and_click_inverse 已断言的 rotate(90)：
        场景 +x 向下、+y 向左，而 layout_x=场景x、layout_y 与场景y 同向。
        """
        w = self.window
        origin = w._ops_to_field(0.0, 0.0)
        self.assertEqual(origin, (2250.0, 2250.0))
        # 遥测 ch0=X=左右、ch1=Y=前后（cm）：前进10.0cm → 场地Y增大 → screen up = layout_x 减小
        forward = w._ops_to_field(0.0, 100.0)
        self.assertEqual(forward, (origin[0] - 100.0, origin[1]))
        # 左移10.0cm → 场地X增大 → screen left = layout_y 减小
        left = w._ops_to_field(100.0, 0.0)
        self.assertEqual(left, (origin[0], origin[1] - 100.0))
        # 点击机器人正前方10.0cm 必须下发 +Y（GOTO 的 Y=前后）
        self.assertEqual(w._field_to_ops(*forward), (0.0, 100.0))
        # 点击机器人正左方10.0cm 必须下发 +X（GOTO 的 X=左右）
        self.assertEqual(w._field_to_ops(*left), (100.0, 0.0))
        # 标定角下往返自洽，且前进方向仍是+Y
        for angle in (0, 37, -90, 180):
            w.map_theta = angle
            bx, by = w._field_to_ops(*w._ops_to_field(120.0, -80.0))
            self.assertAlmostEqual(bx, 120.0)
            self.assertAlmostEqual(by, -80.0)
            gx, gy = w._field_to_ops(*w._ops_to_field(0.0, 100.0))
            self.assertAlmostEqual(gx, 0.0)
            self.assertAlmostEqual(gy, 100.0)

    def test_icon_and_trail_share_one_mapping(self):
        """图标与轨迹必须来自同一变换。

        _update_map_trail 是 _ops_to_field 的向量化副本（历史重复实现），
        坐标改动最容易漏掉这一处；此处直接比较两者的落点是否重合。

        车心用四个顶点的均值：`QGraphicsPolygonItem.boundingRect()` 对"旋转 +
        宽笔迹"的外扩不对称（实测 3px 笔在 15° 时上下 1.449 / 左右 1.837），
        其 center() 与真实车心差 0.19mm，不能作为几何中心使用。
        """
        w = self.window
        for angle in (0, 37, -90):
            w.map_theta = angle
            w.map_ox, w.map_oy = 125.0, -300.0
            frame = (140.0, -260.0, 15.0) + (0.0,) * 21
            w.latest = frame
            w.traj_ring.clear()
            w.traj_ring.append(0.0, (frame[0] * core.OPS_CM_TO_MM,
                                     frame[1] * core.OPS_CM_TO_MM))
            w._render_ui()
            w._update_map_trail()
            fx, fy = w._ops_to_field(frame[0] * core.OPS_CM_TO_MM,
                                     frame[1] * core.OPS_CM_TO_MM)
            icon, _nose = self._icon_center_and_nose(w.map_view)
            self.assertAlmostEqual(icon[0], fx, places=6)
            self.assertAlmostEqual(icon[1], w.map_view.sy(fy), places=6)
            path = w.map_view.trail_item.path()
            self.assertEqual(path.elementCount(), 1)
            element = path.elementAt(0)
            self.assertAlmostEqual(element.x, fx, places=6)
            self.assertAlmostEqual(element.y, w.map_view.sy(fy), places=6)

    # ---------------- 车体图标：28cm × 26cm 轮廓 + 随航向旋转 ----------------
    @staticmethod
    def _icon_center_and_nose(view):
        """从实际画出的多边形反推车心与前向向量（场景坐标，y 已按 sy() 翻转）。"""
        poly = view.car_item.polygon()
        pts = [main.QPointF(poly.at(i)) for i in range(poly.count())]
        cx = sum(p.x() for p in pts) / len(pts)
        cy = sum(p.y() for p in pts) / len(pts)
        wedge = [main.QPointF(view.car_nose.polygon().at(i)) for i in range(3)]
        apex = max(wedge, key=lambda p: math.hypot(p.x() - cx, p.y() - cy))
        return (cx, cy), (apex.x()-cx, apex.y()-cy)

    def test_car_icon_is_scaled_car_body(self):
        """地图上的车必须是 28cm×26cm 的真实轮廓，车心落在映射位置上。"""
        w = self.window
        w.map_ox = w.map_oy = w.map_theta = 0.0
        w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21      # 航向0 ⇒ 车头朝世界+Y = 屏幕上
        w._render_ui()
        view = w.map_view
        poly = view.car_item.polygon()
        self.assertEqual(poly.count(), 4)
        pts = [main.QPointF(poly.at(i)) for i in range(4)]
        edges = [math.hypot(pts[i].x() - pts[(i + 1) % 4].x(),
                            pts[i].y() - pts[(i + 1) % 4].y()) for i in range(4)]
        self.assertAlmostEqual(min(edges), core.CAR_WIDTH_MM, places=6)    # 26 cm
        self.assertAlmostEqual(max(edges), core.CAR_LENGTH_MM, places=6)   # 28 cm
        center, nose = self._icon_center_and_nose(view)
        fx, fy = w._ops_to_field(0.0, 0.0)
        self.assertAlmostEqual(center[0], fx, places=6)
        self.assertAlmostEqual(center[1], view.sy(fy), places=6)
        # 前向长度 = 半个车长，方向 = 世界前向过同一次映射
        length = math.hypot(*nose)
        self.assertAlmostEqual(length, core.CAR_LENGTH_MM / 2.0, places=6)
        exp_nose = w._ops_dir_to_map(*w._body_nose_left(0.0)[0])
        exp_left = w._ops_dir_to_map(*w._body_nose_left(0.0)[1])
        self.assertAlmostEqual(nose[0] / length, exp_nose[0], places=6)     # 绘制帧 x
        self.assertAlmostEqual(-nose[1] / length, exp_nose[1], places=6)    # 场景 y 取反
        # 车左 = 短边反向（pt(hl,hw)→pt(hl,-hw) = -2*(车宽/2)*车左）
        edge = (pts[1].x() - pts[0].x(), pts[1].y() - pts[0].y())
        side = math.hypot(*edge)
        self.assertAlmostEqual(side, core.CAR_WIDTH_MM, places=6)
        self.assertAlmostEqual(-edge[0] / side, exp_left[0], places=6)
        self.assertAlmostEqual(edge[1] / side, exp_left[1], places=6)
        # 屏幕上：航向 0° 时车头朝上、车左朝左（屏幕 y 向下为正）
        t = view.viewportTransform()
        origin = t.map(main.QPointF(*center))
        head = t.map(main.QPointF(center[0] + nose[0], center[1] + nose[1])) - origin
        flank = t.map(main.QPointF(center[0] - edge[0], center[1] - edge[1])) - origin
        self.assertLess(head.y(), 0.0)                 # 车头向上
        self.assertLess(abs(head.x()), abs(head.y()))
        self.assertLess(flank.x(), 0.0)                # 车左向左
        self.assertLess(abs(flank.y()), abs(flank.x()))

    def test_map_heading_readout_matches_icon(self):
        """地图里的航向数字与画出来的车头必须指向同一方向（历史上曾差 90°）。"""
        w = self.window
        w.map_ox = w.map_oy = 0.0
        for theta in (0.0, 30.0, -90.0, 145.0):
            w.map_theta = theta
            for h in (0.0, 45.0, 120.0, -60.0, 180.0):
                w.latest = (0.0, 0.0, h) + (0.0,) * 21
                w._render_ui()
                view = w.map_view
                _center, nose = self._icon_center_and_nose(view)
                # 车头向量：场景 → 绘制帧（向量只把 y 取反），再转场地角
                # （场地 +X=屏幕左=-绘制y、+Y=屏幕上=-绘制x ⇒ 用户角=atan2(-my,-mx)）
                mx, my = nose[0], -nose[1]
                drawn = math.degrees(math.atan2(-my, -mx)) % 360.0
                match = re.search(r"航向=(-?[\d.]+)°", w.map_position.text())
                self.assertIsNotNone(match)
                shown = float(match.group(1))
                self.assertAlmostEqual((drawn - shown + 180.0) % 360.0 - 180.0, 0.0, places=2)

    def test_field_target_yaw_is_inverse_of_readout(self):
        """「目标航向=场地N°」下发后，遥测回来必须正好显示 N°。"""
        w = self.window
        for theta in (0.0, 30.0, -90.0, 145.0):
            w.map_theta = theta
            for h in (0.0, 90.0, 180.0, -45.0, 123.0):
                self.assertAlmostEqual(w._field_to_body_yaw(w._field_heading(h)) % 360.0,
                                       h % 360.0, places=6)
        w.map_theta = 0.0
        w.map_yaw_combo.setCurrentIndex(2)                     # 用户车头90° = 屏幕左
        self.assertAlmostEqual(w._field_heading(w._target_yaw_ops()), 90.0, places=6)
        w.map_yaw_combo.setCurrentIndex(1)                     # 用户车头0° = 屏幕上
        self.assertAlmostEqual(w._field_heading(w._target_yaw_ops()), 0.0, places=6)
        w.map_yaw_combo.setCurrentIndex(0)                     # 保持当前
        w.latest = (0.0, 0.0, 33.0) + (0.0,) * 21
        self.assertAlmostEqual(w._target_yaw_ops(), 33.0, places=6)

    def test_car_icon_basis_matches_actual_travel(self):
        """不依赖任何角度约定：指令 (左, 前) 的位移方向必须与图标自身基一致。

        前进沿车头、左移沿车左、右移朝 −车左。横移时车头本来就不该跟着转，
        所以判据不能用"车头 = 位移方向"，而要用图标画出来的两个基向量合成：
        期望位移 = 左右分量×车左 + 前后分量×车头。
        """
        w = self.window
        sim = core.Simulator(queue.Queue(), queue.Queue())
        w.sim = sim
        w.map_ox = w.map_oy = w.map_theta = 0.0
        view = w.map_view
        cases = [(0.0, 200, 0), (0.0, 0, 200), (0.0, -200, 0), (0.0, 0, -200),
                 (0.0, 200, 200), (60.0, 0, 200), (-90.0, 200, 0), (150.0, 200, 0)]
        with patch.object(core.random, "gauss", return_value=0):
            for h0, lrpm, frpm in cases:
                sim.handle_line("ZERO")
                sim.zval = h0                    # ZERO 之后再设，得到相对航向 h0
                sim.manual = None
                sim.goto = None
                start = None
                for i in range(40):
                    if i == 5:
                        sim.handle_line("MANUAL=%d,%d,0" % (lrpm, frpm))
                    w.frame_q.put((w.t0_monotonic + 0.02 * i, sim.make_frame(0.02 * i)))
                    w._process_frames()
                    w._render_ui()
                    if i == 10:
                        start = w._ops_to_field(w.latest[0] * core.OPS_CM_TO_MM,
                                                w.latest[1] * core.OPS_CM_TO_MM)
                    sim.manual_tick = time.monotonic()      # 续期，避免 350ms 超时停车
                end = w._ops_to_field(w.latest[0] * core.OPS_CM_TO_MM,
                                      w.latest[1] * core.OPS_CM_TO_MM)
                travel = (end[0] - start[0], -(end[1] - start[1]))     # 绘制帧 → 场景
                self.assertGreater(math.hypot(*travel), 100.0,
                                   "h=%.0f MANUAL=(%d,%d) 没动起来" % (h0, lrpm, frpm))
                # 图标自身基（场景坐标）；(nose, left) 手性为 cross=-1 ⇒ left=(ny,-nx)
                _center, raw_nose = self._icon_center_and_nose(view)
                nl = math.hypot(*raw_nose)
                nose = (raw_nose[0] / nl, raw_nose[1] / nl)
                left = (nose[1], -nose[0])
                exp = (lrpm * left[0] + frpm * nose[0], lrpm * left[1] + frpm * nose[1])
                dot = travel[0] * exp[0] + travel[1] * exp[1]
                cross = travel[0] * exp[1] - travel[1] * exp[0]
                angle = abs(math.degrees(math.atan2(cross, dot)))
                self.assertLess(angle, 2.0, "h=%.0f MANUAL=(%d,%d)：位移与图标基差 %.1f°"
                                % (h0, lrpm, frpm, angle))

    # ---------------- A* 路径规划（只算不下发） ----------------
    def _clear_queues(self, w):
        while not w.line_q.empty():
            w.line_q.get_nowait()
        while not w.urgent_q.empty():
            w.urgent_q.get_nowait()

    def test_astar_straight_when_unobstructed(self):
        r = core.plan_path((2250.0, 2250.0), (1200.0, 2250.0), pad=0.0)
        self.assertTrue(r["ok"])
        self.assertEqual(r["points"], [(2250.0, 2250.0), (1200.0, 2250.0)])
        self.assertAlmostEqual(r["length"], 1050.0, places=6)
        # 默认膨胀下车体尺寸生效：原料区圆盘附近的目标会被标记为"落在膨胀区内"
        r2 = core.plan_path((2250.0, 2250.0), (1200.0, 2250.0))
        self.assertFalse(r2["ok"])
        self.assertFalse(r2["execution_safe"])
        self.assertEqual(r2["points"], [])
        self.assertEqual(r2["goal_in_inflate"], "原料区圆盘")

    def test_astar_detour_is_collision_free(self):
        """绕行路径的每一段都必须避开（按车体膨胀后的）禁区，且余量可报告。"""
        start, goal = (2250.0, 2250.0), (330.0, 1200.0)
        r = core.plan_path(start, goal)
        self.assertTrue(r["ok"])
        self.assertEqual(r["points"][0], start)
        self.assertEqual(r["points"][-1], goal)
        self.assertGreater(len(r["points"]), 2)                 # 确实绕了
        rects, circles = core.obstacles_with_pad(r["pad"])
        legs = len(r["points"]) - 1
        for k in range(legs):                            # 内部各段必须干净
            self.assertIsNone(core.seg_blocked(r["points"][k][0], r["points"][k][1],
                                               r["points"][k + 1][0], r["points"][k + 1][1],
                                               rects, circles))
        straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
        self.assertGreater(r["length"], straight)               # 比直线长
        self.assertLess(r["length"], straight * 1.6)            # 但代价可接受
        self.assertGreater(r["min_clearance"], 0.0)             # 真实余量 > 0
        # 拉直：航点数明显少于网格原始路径
        self.assertLess(len(r["points"]), len(r["raw"]))

    def test_astar_reports_sealed_corridors(self):
        """膨胀到把 40cm 走廊封死时，必须报"没有可行路径"而不是硬穿。"""
        r = core.plan_path((2250.0, 2250.0), (1200.0, 330.0), pad=400.0)
        self.assertFalse(r["ok"])
        self.assertEqual(r["points"], [])
        self.assertIn(r["code"], ("INVALID_START", "NO_PATH"))

    def test_astar_finer_grid_keeps_path_clean(self):
        r = core.plan_path((2250.0, 2250.0), (330.0, 1200.0), grid=50.0)
        self.assertTrue(r["ok"])
        rects, circles = core.obstacles_with_pad(r["pad"])
        legs = len(r["points"]) - 1
        for k in range(legs):
            self.assertIsNone(core.seg_blocked(r["points"][k][0], r["points"][k][1],
                                               r["points"][k + 1][0], r["points"][k + 1][1],
                                               rects, circles))
        self.assertGreater(r["min_clearance"], 0.0)

    def test_map_click_plans_without_sending_anything(self):
        """本阶段的核心约定：规划只算不下发，一条 GOTO 都不许进串口队列。"""
        w = self.window
        w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21      # 车在启停区1中心
        self._clear_queues(w)
        target = core.QUICK_ANCHORS[1]                # 暂存区：直线穿越物料区，必须绕
        w._request_plan_anchor(target[0], target[1], target[2])
        self._wait_plan()
        # 不要用 line_q.empty()：参数回读轮询每秒会往同一队列放一条 GET，
        # 真实事件循环下该断言必会误报（曾把 run_tests.py --qt 长期盖红）。
        queued = list(w.line_q.queue)
        self.assertEqual(core.commanding_commands(queued), [],
                         "规划模式不得下发机构命令：%s" % queued)
        for item in queued:
            self.assertTrue(str(item).upper().startswith("GET "),
                            "规划不该产生别的队列条目：%s" % item)
        self.assertTrue(w.urgent_q.empty())
        self.assertGreater(len(w.planned_points), 2)  # 有绕行航点
        self.assertGreater(w.map_view.path_item.path().elementCount(), 1)   # 画了路径
        self.assertIn("场地(", w.plan_text.toPlainText())                 # 给出了坐标
        for row in w.planned_result['steps']:
            if row['kind'] == 'TURN':
                self.assertIn(row['action'], w.plan_text.toPlainText())   # 原地转向不遗漏
        if w.planned_result.get('turn_count'):
            self.assertNotIn("GOTO=", w.plan_text.toPlainText())          # 不伪造固定航向执行台账
        self.assertIn("场地", w.plan_text.toPlainText())
        self.assertIn("航点", w.plan_info.text())
        self.assertIn("未下发", w.map_status.text())
        # 目标必须是**现算**的合法接近点：在锚点向场内一侧，且与规划终点一致
        self.assertGreater(w.map_target[0], target[1][0])
        self.assertLess(math.dist(w.map_target, w.planned_points[-1]), 2.0)
        # 航点里给出的 GOTO 值必须能反算回同一个场地位置（将来就是照这个下发）
        ox, oy = w._field_to_ops(*w.planned_points[1])
        back = w._ops_to_field(ox, oy)
        self.assertAlmostEqual(back[0], w.planned_points[1][0], places=6)
        self.assertAlmostEqual(back[1], w.planned_points[1][1], places=6)

    def test_plan_rejects_target_inside_forbidden_zone(self):
        w = self.window
        w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        self._clear_queues(w)
        w.map_view.gotoRequested.emit(700.0, 700.0)   # 中央物料区内部
        self._wait_plan()
        self.assertIn("规划失败", w.map_status.text())
        self.assertEqual(w.plan_info.property("state"), "bad")
        self.assertEqual(w.planned_points, [])
        self.assertTrue(w.line_q.empty())

    def test_plan_switch_off_restores_direct_goto(self):
        w = self.window
        w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        w.plan_click_check.setChecked(False)
        self._clear_queues(w)
        w.map_view.gotoRequested.emit(2150.0, 2250.0)
        self.assertEqual(w.line_q.get_nowait(), "GOTO=0.0,10.0,0.0")
        self.assertEqual(w.planned_points, [])

    def test_quick_target_and_home_plan_when_enabled(self):
        w = self.window
        w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        self._clear_queues(w)
        # 旧写死坐标 (1200,2200) 离圆盘表面只有 90mm，280×260 车体停不下：必须仍被拒绝。
        w._on_map_click(1200.0, 2200.0)
        self._wait_plan()
        self.assertTrue(w.line_q.empty())
        self.assertEqual(w.planned_points, [])
        # 功能区按钮改为按当前航向+裕量现算接近点：原料区目标应当合法且能规划。
        w._clear_path()
        name, anchor, outward = core.QUICK_ANCHORS[0]
        res = w._request_plan_anchor(name, anchor, outward)
        self._wait_plan()
        self.assertTrue(res and res["ok"], w.map_status.text())
        self.assertTrue(w.planned_points)
        self.assertGreater(anchor[1] - w.map_target[1], 100.0)   # 沿 -y 让开圆盘
        self.assertTrue(w.line_q.empty())
        w._clear_path()
        w.zone_combo.setCurrentIndex(0)
        w._goto_home()                                # 回启停区1
        self._wait_plan()
        self.assertIsNone(w.sim.goto)
        self.assertTrue(w.line_q.empty())
        self.assertTrue(w.planned_points)
        self.assertFalse(w._home_after_stop)

    def test_follow_is_simulation_only_and_stops_on_stop(self):
        w = self.window
        w.sim = None
        w.planned_points = [(2250.0, 2250.0), (2150.0, 2250.0)]
        self._clear_queues(w)
        w._toggle_follow()
        self.assertIsNone(w.follow)
        self.assertTrue(w.line_q.empty())
        w.sim = core.Simulator(queue.Queue(), queue.Queue())
        w.sim.handle_line("ZERO")
        w.sim.zval = 90   # FIELD航向0°，与首段切线一致。
        w.sim.make_frame(0.0)
        self.assertTrue(w.plan_to(2250, 1600)["ok"])
        w._toggle_follow()
        self.assertIsNotNone(w.follow)
        self.assertTrue(w.line_q.empty(), "模拟专用接口不得冒充固件GOTO协议")
        w.latest = (0.0,) * 24  # old zero error cannot prove a NEW target completed
        w._follow_step()
        self.assertEqual(w.follow["progress_s_mm"], 0)
        w._toggle_follow()
        self.assertIsNone(w.follow)
        self.assertIsNone(w.sim.goto)
        frozen = w.sim.hold
        w.sim.make_frame(1.0)
        self.assertEqual(w.sim.hold, frozen)
        w.send_line("STOP")
        self.assertEqual(w.planned_points, [])
        self.assertFalse(w.map_view.path_item.path().elementCount())

    def test_planned_path_is_drivable_in_simulation(self):
        """端到端连续跟踪：终点停稳且真实矩形车体扫掠无碰撞。"""
        w = self.window
        sim = core.Simulator(queue.Queue(), queue.Queue())
        w.sim = sim
        w.map_ox = w.map_oy = w.map_theta = 0.0
        sim.handle_line("ZERO")
        sim.zval = 90
        w.frame_q.put((w.t0_monotonic, sim.make_frame(0.0)))
        w._process_frames()
        self.assertTrue(w.plan_to(2250, 1600)["trajectory_safe"])
        sim.make_frame(0.0)                # 秒级规划后位姿已过 350ms 闸门，重新喂帧再启动
        target = w.map_target
        self.assertIsNotNone(target, w.map_status.text())
        self.assertGreater(len(w.planned_result["trajectory"]), 2)
        self._clear_queues(w)
        w._toggle_follow()
        self.assertIsNotNone(w.follow)
        hit, t = None, 0.0
        scene = w._scene_for_context(w._planned_context)
        previous = w._ops_to_field(*sim.hold)
        with patch.object(core.random, "gauss", return_value=0):
            for step in range(1200):                       # 上限 24 秒仿真时间
                t += 0.02
                while not w.line_q.empty():                # 无线程：手动把命令喂给模拟器
                    sim.handle_line(w.line_q.get_nowait())
                w.frame_q.put((w.t0_monotonic + t, sim.make_frame(t)))
                w._process_frames()
                w._follow_step()                          # 20ms显示/完成检查。
                px, py = w._ops_to_field(*sim.hold)
                hit = hit or scene.segment_reason(previous, (px, py))
                previous = (px, py)
                if w.follow is None:
                    break
        self.assertIsNone(w.follow)
        self.assertIsNone(hit, "全程整车扫掠不应越界或碰撞（撞到 %s）" % hit)
        ex, ey = w._ops_to_field(w.latest[0] * core.OPS_CM_TO_MM,
                                 w.latest[1] * core.OPS_CM_TO_MM)
        self.assertLess(math.hypot(ex - target[0], ey - target[1]), 1.0,w.map_status.text())

    def test_mapping_roundtrip(self):
        for angle in (0, 37, 90, -180):
            self.window.map_theta = angle
            self.window.map_ox, self.window.map_oy = 125, -300
            x, y = self.window._field_to_ops(*self.window._ops_to_field(-400, 230))
            self.assertAlmostEqual(x, -400)
            self.assertAlmostEqual(y, 230)

    def test_clockwise_view_and_click_inverse(self):
        view = self.window.map_view
        transform = view.transform()
        origin = transform.map(main.QPointF(0, 0))
        right = transform.map(main.QPointF(100, 0))
        down = transform.map(main.QPointF(0, 100))
        self.assertAlmostEqual(right.x(), origin.x())
        self.assertGreater(right.y(), origin.y())
        self.assertLess(down.x(), origin.x())
        # 场景目标经显示变换后再逆变换仍为原位置，缩放不改变命令坐标。
        point = main.QPointF(2150, view.sy(2250))
        screen = view.viewportTransform().map(point)
        inverse, valid = view.viewportTransform().inverted()
        self.assertTrue(valid)
        restored = inverse.map(screen)
        self.assertAlmostEqual(restored.x(), point.x())
        self.assertAlmostEqual(restored.y(), point.y())

    def test_zone_two_does_not_change_field_origin(self):
        self.window.zone_combo.setCurrentIndex(1)
        self.window._set_start_zone()
        self.assertEqual((self.window.map_ox, self.window.map_oy), (2100, 0))
        self.assertEqual(self.window._ops_to_field(0, 0), core.ZONE_CENTER[2])

    def test_start_zone_calibration_sets_real_field_heading_and_forward_axis(self):
        w = self.window
        for index, zone, heading in ((0, 1, 0), (1, 2, 0)):
            w.zone_combo.setCurrentIndex(index)
            w._set_start_zone()
            w.sim.handle_line('ZERO')
            w.sim.make_frame(0)
            snap = w.sim.navigation_snapshot()
            self.assertAlmostEqual(w._field_heading(snap['yaw']), heading)
            home = core.layout_to_field(*w._ops_to_field(*snap['hold']))
            forward = core.layout_to_field(*w._ops_to_field(0, 100))
            self.assertAlmostEqual(forward[0]-home[0], 0)
            self.assertAlmostEqual(forward[1]-home[1], 100)

    def test_normal_simulator_start_uses_selected_zone_field_heading(self):
        w = self.window
        w.toggle_sim(False)
        w.toggle_sim(True)
        try:
            self.assertAlmostEqual(w._field_heading(w.sim.navigation_snapshot()['yaw']), 0)
            self.assertEqual(w._ops_to_field(*w.sim.navigation_snapshot()['hold']), core.ZONE_CENTER[1])
            nose, _left = w._body_nose_left(w.sim.navigation_snapshot()['yaw'])
            dx, dy = w._ops_dir_to_map(*nose)
            self.assertAlmostEqual(dx, -1)
            self.assertAlmostEqual(dy, 0)
            self.assertEqual(w.map_theta, 0)
        finally:
            w.toggle_sim(False)

    def test_target_heading_cardinal_directions_match_y_zero_convention(self):
        w = self.window
        for theta in (0,37,-90,145):
            w.map_theta = theta
            for heading, expected in ((0,(0,1)), (90,(1,0)), (180,(0,-1)), (270,(-1,0))):
                ops = w._field_to_body_yaw(heading)
                nose, _left = w._body_nose_left(ops)
                lx, ly = w._ops_dir_to_map(*nose)
                self.assertAlmostEqual(-ly, expected[0])
                self.assertAlmostEqual(-lx, expected[1])
                self.assertAlmostEqual(w._field_heading(ops), heading)
                self.assertAlmostEqual(w._field_math_heading(ops), (90-heading)%360)

    def test_simulation_home_bypasses_path_checks(self):
        self.window.plan_click_check.setChecked(False)   # 本用例测"直接下发 GOTO"这条路
        self.window.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        self.window._goto_field(2150, 2250)
        self.assertEqual(self.window.line_q.get_nowait(), "GOTO=0.0,10.0,0.0")
        # 新约定下同一物理位置(左1650/前1650)的遥测为正值，映射到与旧用例相同的layout(600,600)，
        # 因此到启停区1的直线仍穿越中央物料区。
        self.window.latest = (165.0, 165.0, 0.0) + (0.0,) * 21
        self.window.send_line("STOP")
        self.window._goto_home()
        self.assertEqual(self.window.sim.goto,(0,0,0))
        self.assertTrue(self.window.line_q.empty())

    def test_stepper_buttons_units_and_uint32(self):
        pos, speed, direction, steps, rpm = self.window.stepper_widgets[35]
        pos.setValue(100.0)
        speed.setValue(50)
        self.window._send_stepper(35, False)
        self.assertEqual(self.window.line_q.get_nowait(), "S35MOVE=1000,50")
        direction.setCurrentIndex(1)
        steps.setValue(4294967295)
        rpm.setValue(100)
        self.window._send_stepper(35, True)
        self.assertEqual(self.window.line_q.get_nowait(), "S35RAW=1,4294967295,100")
        self.assertEqual(core.stepper_move_command(28, 200, 50), "S28MOVE=2000,50")
        with self.assertRaises(ValueError):
            core.stepper_move_command(28, 200, 1)

    def test_stepper_query_enable_and_rpm_limits(self):
        panel = self.window.stepper_widgets[35][0].parentWidget()
        buttons = {b.text(): b for b in panel.findChildren(main.QPushButton)}
        buttons["读取状态（不运动）"].click()
        self.assertEqual(self.window.line_q.get_nowait(), "S35STATUS")
        self.assertIn("不提供", self.window.stepper_status[35].text())
        buttons["使能电机（锁轴）"].click()
        self.assertEqual(self.window.line_q.get_nowait(), "S35EN")
        self.assertEqual(self.window.stepper_widgets[35][-1].maximum(), 3000)
        with self.assertRaises(ValueError):
            core.stepper_move_command(35, 100, 101)
        with self.assertRaises(ValueError):
            core.stepper_move_command(28, 200, 5663)

    def test_stepper_real_feedback_updates_page(self):
        self.window.fw_text_q.put("固件文本: S35 RX CAN=0100 DATA=3A 09 6B")
        self.window._process_frames()
        self.assertIn("已使能", self.window.stepper_status[35].text())
        self.assertIn("堵转保护已触发", self.window.stepper_status[35].text())
        self.window.fw_text_q.put("固件文本: S35 RX CAN=0100 DATA=FD E2 6B")
        self.window._process_frames()
        self.assertIn("驱动拒绝", self.window.stepper_status[35].text())
        self.window._clear_command_queues()
        self.assertIn("尚未读取", self.window.stepper_status[35].text())

    def test_text_replies_survive_binary_in_same_read(self):
        parser = core.FrameParser()
        frame = struct.pack("<24f", *([0.0] * 24)) + core.FRAME_TAIL
        first = b"ACK S35 CAN_SUBMITTED CMD=3A (NO MOTOR ACK)\r\n"
        self.assertEqual(len(parser.feed(first + frame)), 1)
        self.assertEqual(parser.take_text(), [first.decode().strip()])
        self.assertEqual(len(parser.feed(b"XVMIN=5.000\r\n" + frame)), 1)
        self.assertEqual(parser.take_params(), [("XVMIN", 5.0)])
        reply = b"S35 RX CAN=0100 DATA=3A 01 6B\r\n"
        parser.feed(reply[:9])
        parser.feed(reply[9:] + frame + reply + frame)
        self.assertEqual(parser.take_text(), [reply.decode().strip()] * 2)

    def test_cancel_clears_unsent_commands(self):
        self.window.send_line("S28HOME")
        self.window.send_line("S28CANCEL")
        self.assertTrue(self.window.line_q.empty())
        self.assertEqual(self.window.urgent_q.get_nowait(), "S28CANCEL")

    def test_simulator_stop_and_zero(self):
        sim = self.window.sim
        sim.handle_line("DMEN")
        sim.handle_line("STOP")
        self.assertEqual(sim.dm_active, 0)
        sim.handle_line("GOTO=100.0,100.0,0.0")
        sim.handle_line("ZERO")
        self.assertIsNone(sim.goto)
        sim.handle_line("S28MOVE=2000,50")
        self.assertEqual(sim.stepper_commands[28], "S28MOVE=2000,50")
        sim.handle_line("S28CANCEL")
        self.assertIsNone(sim.stepper_commands[28])

    def test_simulator_zero_resets_heading_reference(self):
        sim = self.window.sim
        sim.zval = 15.0
        sim.handle_line("ZERO")
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0.0)
        self.assertAlmostEqual(frame[2], 0.0)
        sim.zval = 40.0
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0.02)
        self.assertAlmostEqual(frame[2], 25.0)

    def test_simulator_goto_cm_and_telemetry_cm(self):
        sim = self.window.sim
        sim.handle_line("GOTO=10.0,20.0,0.0")
        self.assertEqual(sim.goto, (100.0, 200.0, 0.0))
        sim.hold = (100.0, 200.0)           # 统一(X=左, Y=前) mm，正好位于目标
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0.02)
        self.assertAlmostEqual(frame[0], 10.0)   # ch0=X(左右) cm
        self.assertAlmostEqual(frame[1], 20.0)   # ch1=Y(前后) cm

    def keyboard_event(self, key, pressed=True, repeat=False):
        from PySide6.QtGui import QKeyEvent
        event = QKeyEvent(main.QEvent.KeyPress if pressed else main.QEvent.KeyRelease,
                          key, main.Qt.NoModifier, "", repeat)
        self.app.sendEvent(self.window.keyboard_pad, event)

    def test_keyboard_combinations_release_and_slow(self):
        w = self.window
        w._keyboard_toggle()
        # manual_vector 与 MANUAL 同为(X=左右, Y=前后, Z=旋转)。
        self.keyboard_event(main.Qt.Key_W)
        self.assertEqual(w.manual_vector, (0, 60, 0))
        self.keyboard_event(main.Qt.Key_A)
        self.assertEqual(w.manual_vector, (42, 42, 0))
        self.keyboard_event(main.Qt.Key_Q)
        self.assertEqual(w.manual_vector, (42, 42, 30))
        self.keyboard_event(main.Qt.Key_Shift)
        self.assertEqual(w.manual_vector, (13, 13, 9))
        self.keyboard_event(main.Qt.Key_A, False)
        self.assertEqual(w.manual_vector, (0, 18, 9))
        self.keyboard_event(main.Qt.Key_W, False)
        self.keyboard_event(main.Qt.Key_Q, False)
        self.assertIsNone(w.manual_vector)
        self.assertTrue(w.keyboard_enabled)
        self.assertFalse(w.manual_timer.isActive())
        self.assertIn("MANUAL=0,0,0", list(w.urgent_q.queue))

    def test_manual_direction_defaults_not_inverted(self):
        """车头方向反了已在固件协议边界统一修正，三个反向开关默认全部关闭。"""
        w = self.window
        self.assertFalse(self.forward_invert_default)
        self.assertFalse(w.manual_invert[0].isChecked())   # 0=左右反向
        self.assertFalse(w.manual_invert[1].isChecked())   # 1=前后反向
        self.assertFalse(w.manual_invert[2].isChecked())   # 2=旋转反向
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_W)
        self.assertEqual(w.manual_vector, (0, 60, 0))      # W = +Y = 车头
        self.keyboard_event(main.Qt.Key_W, False)
        self.keyboard_event(main.Qt.Key_S)
        self.assertEqual(w.manual_vector, (0, -60, 0))     # S = -Y = 车尾
        self.keyboard_event(main.Qt.Key_S, False)
        self.keyboard_event(main.Qt.Key_A)
        self.assertEqual(w.manual_vector, (60, 0, 0))      # A = +X = 车左
        self.keyboard_event(main.Qt.Key_A, False)
        # 兜底开关仍在：切换复选框会先停车并退出键盘遥控，需重新接管
        w.manual_invert[0].setChecked(True)                # 勾"左右反向"
        self.assertIsNone(w.manual_vector)
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_A)
        self.assertEqual(w.manual_vector, (-60, 0, 0))

    def test_keyboard_opposites_space_repeat_and_escape(self):
        w = self.window
        w._keyboard_toggle()
        self.keyboard_event(main.Qt.Key_W)
        self.keyboard_event(main.Qt.Key_W, False, repeat=True)
        self.assertEqual(w.manual_vector, (0, 60, 0))
        self.keyboard_event(main.Qt.Key_S)
        self.assertIsNone(w.manual_vector)
        self.keyboard_event(main.Qt.Key_S, False)
        self.assertEqual(w.manual_vector, (0, 60, 0))
        self.keyboard_event(main.Qt.Key_Space)
        self.keyboard_event(main.Qt.Key_W, repeat=True)
        self.assertIsNone(w.manual_vector)
        self.keyboard_event(main.Qt.Key_Escape)
        self.assertFalse(w.keyboard_enabled)
        self.keyboard_event(main.Qt.Key_D)
        self.assertIsNone(w.manual_vector)

    def test_keyboard_focus_loss_page_and_stop_disarm(self):
        w = self.window
        for action in (lambda: self.app.sendEvent(w.keyboard_pad, main.QEvent(main.QEvent.FocusOut)),
                       lambda: w._select_page(0), lambda: w.send_line("STOP")):
            w._keyboard_toggle()
            self.keyboard_event(main.Qt.Key_W)
            action()
            self.assertFalse(w.keyboard_enabled)
            self.assertFalse(w.keyboard_keys)
            self.assertIsNone(w.manual_vector)
            self.assertTrue(w.line_q.empty())
            self.keyboard_event(main.Qt.Key_W, repeat=True)
            self.assertIsNone(w.manual_vector)

    def test_keyboard_disabled_and_other_widgets_do_not_drive(self):
        from PySide6.QtGui import QKeyEvent
        w = self.window
        self.keyboard_event(main.Qt.Key_W)
        self.assertIsNone(w.manual_vector)
        w._keyboard_toggle()
        self.app.sendEvent(w.command_entry, QKeyEvent(main.QEvent.KeyPress, main.Qt.Key_W,
                                                     main.Qt.NoModifier, "w"))
        self.assertIsNone(w.manual_vector)
        self.assertEqual(w.command_entry.text(), "w")

    def test_keyboard_exit_button_and_real_focus_change(self):
        from PySide6.QtTest import QTest
        w = self.window
        w.show()
        w._select_page(3)
        w.activateWindow()
        self.app.processEvents()
        QTest.mouseClick(w.keyboard_button, main.Qt.LeftButton)
        self.assertTrue(w.keyboard_enabled)
        QTest.keyPress(w.keyboard_pad, main.Qt.Key_W)
        self.assertIsNotNone(w.manual_vector)
        QTest.mouseClick(w.keyboard_exit, main.Qt.LeftButton)
        self.assertFalse(w.keyboard_enabled)
        self.assertIsNone(w.manual_vector)
        QTest.mouseClick(w.keyboard_button, main.Qt.LeftButton)
        QTest.keyPress(w.keyboard_pad, main.Qt.Key_W)
        w.manual_speed.setFocus()
        self.app.processEvents()
        self.assertFalse(w.keyboard_enabled)
        self.assertIsNone(w.manual_vector)

    def test_manual_hold_release_and_page_exit(self):
        w = self.window
        # _manual_start 的入参与 MANUAL 同序：X=左右、Y=前后、Z=旋转。
        w._manual_start((0, 1, 1))
        self.assertEqual(w.line_q.get_nowait(), "MANUAL=0,60,30")
        w._manual_tick()
        w._manual_stop()
        self.assertIsNone(w.manual_vector)
        self.assertFalse(w.manual_timer.isActive())
        self.assertTrue(w.line_q.empty())

        self.assertEqual(w.urgent_q.get_nowait(), "STOP")
        w._manual_tick()
        self.assertTrue(w.line_q.empty())
        w._manual_start((0, -1, -1))
        w._select_page(0)
        self.assertIsNone(w.manual_vector)
        self.assertTrue(w.line_q.empty())

    def test_continuous_latest_velocity_has_priority_without_backlog(self):
        w = self.window
        w.line_q.put("KPX=2")
        w.line_q.put("KPY=3")
        w._manual_start((0, 1, 0))          # 前进
        for _ in range(20):
            w._manual_tick()
        w._manual_start((1, 0, 0))          # 左移
        self.assertEqual(w.manual_timer.interval(), 50)
        self.assertEqual(w.line_q.qsize(), 3)
        self.assertEqual(w.line_q.get_nowait(), "MANUAL=60,0,0")
        self.assertEqual(w.line_q.get_nowait(), "KPX=2")
        self.assertEqual(w.line_q.get_nowait(), "KPY=3")

    def test_firmware_text_is_surfaced_not_swallowed(self):
        """固件切到文字模式时的 ASCII 报错必须能被界面看到。

        实车证据：CAN 启动失败时固件只发一行
        "ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE" 并停止 24 通道遥测；
        旧实现把非 JustFloat 字节全部静默丢弃，用户完全看不到原因。
        """
        parser = core.FrameParser()
        frame = struct.pack("<24f", *range(24)) + core.FRAME_TAIL
        self.assertEqual(len(parser.feed(frame)), 1)
        self.assertEqual(parser.take_text(), [])            # 纯遥测不产生文本
        self.assertEqual(len(parser.feed(frame)), 1)
        err = b"ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE\r\n"
        self.assertEqual(len(parser.feed(err)), 0)
        self.assertIn("ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE",
                      parser.take_text())
        self.assertEqual(parser.take_text(), [])            # 取走后清空
        # 同一行重复出现只报一次
        parser.feed(err)
        self.assertEqual(parser.take_text(), [])

    def test_firmware_text_and_vofa_notice_reach_log(self):
        w = self.window
        w.fw_text_q.put("固件文本: ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE")
        w.fw_text_q.put("上位机: 1.5s 未收到遥测，已补发 VOFA 恢复波形（第 1/6 次）")
        w._process_frames()
        text = w.console.toPlainText()
        self.assertIn("ERR CAN START FAILED", text)
        self.assertIn("已补发 VOFA", text)
        self.assertTrue(w.fw_text_q.empty())

    def test_serial_worker_resends_vofa_when_silent(self):
        """板子在上位机已连接时复位后，必须能自动补发 VOFA 恢复遥测。"""
        writes = []

        class Port:
            is_open = True
            in_waiting = 0

            def read(self, size):
                return b""

            def write(self, data):
                writes.append(data)
                return len(data)

            def close(self):
                self.is_open = False

        text_q = queue.Queue()
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), core.CommandQueue(),
                                   queue.Queue(), text_q=text_q)
        with patch.object(core.serial, "Serial", return_value=Port()), \
             patch.object(core, "VOFA_RETRY_GAP_S", 0.05), \
             patch.object(core, "VOFA_MAX_RETRIES", 3):
            worker.start()
            deadline = time.monotonic() + 4.0
            while len(writes) < 4 and time.monotonic() < deadline:
                time.sleep(0.02)
            worker.stop_flag = True
            worker.join(timeout=2.0)
        self.assertEqual(writes[:1], [b"VOFA\n"])                    # 连接时先发一次
        self.assertGreaterEqual(writes.count(b"VOFA\n"), 2)          # 静默后补发
        self.assertLessEqual(writes.count(b"VOFA\n"), 4)             # 达上限后停止
        notes = []
        while not text_q.empty():
            notes.append(text_q.get_nowait())
        self.assertTrue(any("补发 VOFA" in n for n in notes))

    def test_serial_idle_read_is_short_and_stop_precedes_velocity(self):
        q = core.CommandQueue()
        urgent = queue.Queue()
        q.put("MANUAL=60,0,0")
        urgent.put("STOP")
        worker = core.SerialWorker("TEST", 115200, queue.Queue(), q, urgent)
        writes, reads = [], []

        class Port:
            is_open = True
            in_waiting = 0
            def read(self, size):
                reads.append(size)
                return b""
            def write(self, data):
                writes.append(data)
                if data.startswith(b"MANUAL="):
                    worker.stop_flag = True
                return len(data)
            def close(self):
                self.is_open = False

        with patch.object(core.serial, "Serial", return_value=Port()) as serial_open:
            worker.run()
        self.assertEqual(serial_open.call_args.kwargs["timeout"], 0.01)
        self.assertEqual(reads, [1])
        self.assertEqual(writes, [b"VOFA\n", b"STOP\n", b"MANUAL=60,0,0\n"])

    def test_manual_stop_zero_goto_cancel_renewal(self):
        for cmd in ("STOP", "ZERO", "GOTO=10.0,10.0,0.0"):
            self.window._clear_command_queues()
            self.window._manual_start((1, 0, 0))
            self.window.send_line(cmd)
            self.assertIsNone(self.window.manual_vector)
            self.assertFalse(self.window.manual_timer.isActive())

    def test_manual_simulation_timeout_and_invalid(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("MANUAL=0,60,30")          # Y=车头=+60、Z=逆时针
        sim.make_frame(0)
        self.assertGreater(sim.hold[1], 0)         # +Y 指向车头
        self.assertAlmostEqual(sim.hold[0], 0.0)
        self.assertGreater(sim.zval, 0)
        for bad in ("MANUAL=301,0,0", "MANUAL=60,0", "MANUAL=60,0,0,1", "MANUAL=nan,0,0", "MANUAL=1.2,0,0"):
            sim.handle_line(bad)
            self.assertEqual(sim.manual, (0, 60, 30))
        sim.manual_tick -= 1
        sim.handle_line("PING")
        hold = sim.hold
        sim.make_frame(0.02)
        self.assertIsNone(sim.manual)
        self.assertEqual(sim.hold, hold)
        sim.handle_line("MANUAL=60,0,0")
        sim.handle_line("STOP")
        self.assertIsNone(sim.manual)

    def test_offset_apply_stops_manual_and_sends_pair(self):
        w = self.window
        self.assertEqual(w.ops_offset_x.value(), 60)
        self.assertEqual(w.ops_offset_y.value(), -50)
        w._manual_start((0, 0, 1))
        w.ops_offset_y.setValue(-52.5)
        w._apply_ops_offset()
        self.assertIsNone(w.manual_vector)
        self.assertEqual(w.line_q.get_nowait(), "OPSOFFSET=60.0,-52.5")
        self.assertEqual(w.urgent_q.get_nowait(), "STOP")

    def test_offset_file_roundtrip_and_invalid(self):
        w = self.window
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "offset.json")
            w.ops_offset_x.setValue(-48.5)
            with patch.object(main.QFileDialog, "getSaveFileName", return_value=(path, "")):
                w._save_ops_offset()
            w._reset_ops_offset()
            with patch.object(main.QFileDialog, "getOpenFileName", return_value=(path, "")):
                w._load_ops_offset()
            self.assertEqual(w.ops_offset_x.value(), -48.5)
            self.assertTrue(w.line_q.empty())
            Path(path).write_text('{"version":1,"x_mm":999,"y_mm":60}', encoding="utf-8")
            with patch.object(main.QFileDialog, "getOpenFileName", return_value=(path, "")):
                w._load_ops_offset()
            self.assertEqual(w.ops_offset_x.value(), -48.5)
            self.assertIn("加载失败", w.ops_offset_status.text())

    def test_offset_file_v1_migrates_axis_order_once(self):
        w = self.window
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offset-v1.json"
            # v1: x_mm=前后、y_mm=左右。迁移后 X 左右=60、Y 前后=-50。
            path.write_text('{"version":1,"x_mm":-50,"y_mm":60}', encoding="utf-8")
            with patch.object(main.QFileDialog, "getOpenFileName", return_value=(str(path), "")):
                w._load_ops_offset()
        self.assertEqual(w.ops_offset_x.value(), 60)
        self.assertEqual(w.ops_offset_y.value(), -50)
        self.assertIn("迁移", w.ops_offset_status.text())

    def test_simulator_heading_and_zero_physical_axes(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.zval = 90
        sim.handle_line("MANUAL=0,60,0")
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0)
        self.assertGreater(frame[0], 0)  # +90°车头朝世界+X
        self.assertAlmostEqual(frame[1], 0)
        sim.handle_line("ZERO")
        sim.handle_line("MANUAL=0,60,0")
        with patch.object(core.random, "gauss", return_value=0):
            frame = sim.make_frame(0)
        self.assertAlmostEqual(frame[0], 0)
        self.assertGreater(frame[1], 0)  # ZERO后按新车头建立+Y

    def test_simulator_hold_and_invalid_goto(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("MANUAL=0,60,0")
        for line in ("GOTO=0,0,3601", "GOTO=1,2,bad", "GOTO=1,2,", "GOTO=1,2,0,4"):
            sim.handle_line(line)
            self.assertEqual(sim.manual, (0, 60, 0))
        sim.handle_line("GOTOHOLD=0,0,0")
        sim.make_frame(0)
        self.assertIsNotNone(sim.goto)
        sim.hold = (100, 0)
        sim.make_frame(0)
        self.assertLess(sim.hold[0], 100)
        sim.handle_line("STOP")
        self.assertIsNone(sim.goto)

    def test_offset_simulator_residual_and_validation(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("MANUAL=0,0,30")
        sim.handle_line("OPSOFFSET=0,0")
        self.assertIsNone(sim.manual)
        self.assertIsNone(sim.goto)
        sim.zval = 90
        with patch.object(core.random, "gauss", return_value=0):
            uncompensated = sim.make_frame(0)
        # 90° 旋转时，未配置偏移留下与安装半径和参考航向相关的残差。
        self.assertAlmostEqual((uncompensated[0]**2 + uncompensated[1]**2)**0.5, 11.04536, places=3)
        sim.handle_line("OPSOFFSET=60,-50")        # 统一坐标 X=左60、Y=前-50
        for bad in ("OPSOFFSET=nan,60", "OPSOFFSET=-501,60", "OPSOFFSET=-50,60,0", "OPSOFFSET=-50,", "OPSOFFSET=1e2,60"):
            sim.handle_line(bad)
            self.assertEqual(sim.ops_offset, (60, -50))
        sim.zval = 180
        with patch.object(core.random, "gauss", return_value=0):
            compensated = sim.make_frame(0.02)
        self.assertAlmostEqual(compensated[0], 0)
        self.assertAlmostEqual(compensated[1], 0)

    def test_wheel_lock_commands_track_state_and_stop_manual(self):
        w = self.window
        self.assertIsNone(w.wheel_state)
        w._manual_start((1, 0, 0))
        w.send_line("WHEELOFF")
        # 切换锁轴前先停止键盘续发，命令本身走普通队列。
        self.assertIsNone(w.manual_vector)
        self.assertFalse(w.manual_timer.isActive())
        self.assertIs(w.wheel_state, False)
        self.assertEqual(w.line_q.get_nowait(), "WHEELOFF")
        self.assertIn("失能", w.wheel_status.text())
        # STOP 只停车，不改变锁轴状态。
        w.send_line("STOP")
        self.assertIs(w.wheel_state, False)
        w.send_line("WHEELEN")
        self.assertIs(w.wheel_state, True)
        self.assertEqual(w.line_q.get_nowait(), "WHEELEN")
        self.assertIn("使能", w.wheel_status.text())

    def test_wheel_disabled_blocks_goto_keyboard_and_manual(self):
        w = self.window
        w.send_line("WHEELOFF")
        w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        w._goto_field(2150, 2250)
        # 队列里只有失能命令本身，GOTO 未入队。
        self.assertEqual(w.line_q.get_nowait(), "WHEELOFF")
        self.assertTrue(w.line_q.empty())
        self.assertIn("失能", w.map_status.text())
        w._keyboard_toggle()
        self.assertFalse(w.keyboard_enabled)
        w._manual_start((1, 0, 0))
        self.assertIsNone(w.manual_vector)
        # 重新使能后 GOTO 与手动恢复。
        w.send_line("WHEELEN")
        w._goto_field(2150, 2250)
        self.assertEqual(w.line_q.get_nowait(), "WHEELEN")
        self.assertEqual(w.line_q.get_nowait(), "GOTO=0.0,10.0,0.0")

    def test_gotohold_stops_manual_and_is_queued(self):
        w = self.window
        w._manual_start((60, 0, 0))
        w.send_line("GOTOHOLD=10.0,20.0,30.0")
        self.assertIsNone(w.manual_vector)
        self.assertFalse(w.manual_timer.isActive())
        self.assertEqual(w.urgent_q.get_nowait(), "STOP")
        self.assertEqual(w.line_q.get_nowait(), "GOTOHOLD=10.0,20.0,30.0")

    def test_simulator_wheel_gate_and_freeze(self):
        sim = self.window.sim
        sim.handle_line("ZERO")
        sim.handle_line("WHEELOFF")
        self.assertFalse(sim.wheel_enabled)
        sim.handle_line("MANUAL=60,0,30")
        sim.handle_line("GOTO=10.0,10.0,0.0")
        self.assertIsNone(sim.manual)
        self.assertIsNone(sim.goto)
        hold = sim.hold
        sim.make_frame(0.02)
        self.assertEqual(sim.hold, hold)          # 失能后位置冻结
        sim.handle_line("WHEELEN")
        sim.handle_line("MANUAL=60,0,30")
        self.assertEqual(sim.manual, (60, 0, 30))

    def test_console_command_updates_wheel_state(self):
        w = self.window
        w.command_entry.setText("wheeloff")
        w._send_console()
        self.assertIs(w.wheel_state, False)
        self.assertEqual(w.line_q.get_nowait(), "wheeloff")
        w.command_entry.setText("WHEELEN")
        w._send_console()
        self.assertIs(w.wheel_state, True)

    def test_page_header_button_detaches_page(self):
        from PySide6.QtTest import QTest
        w = self.window
        w.show()
        page = w.pages[6]
        buttons = [b for b in page.findChildren(main.QPushButton) if b.text() == "独立窗口"]
        self.assertTrue(buttons)
        QTest.mouseClick(buttons[0], main.Qt.LeftButton)
        self.assertIn(6, w.detached)
        self.assertIs(w.detached[6].page(), page)
        self.assertIs(w.pages[6].window(), w.detached[6])
        self.assertIs(w.slots[6].currentWidget(), w.placeholders[6])
        # 换父窗口后控件不能被 Qt 因 setParent 而留在隐藏状态。
        self.assertFalse(page.isHidden())
        self.app.processEvents()
        self.assertTrue(page.isVisible())
        w._reattach_page(6)
        self.app.processEvents()
        self.assertFalse(page.isHidden())
        self.assertIs(w.slots[6].currentWidget(), page)
        w._select_page(6)          # 主界面回到该页后才应该可见
        self.app.processEvents()
        self.assertTrue(page.isVisible())

    def test_page_detach_keeps_stack_index_and_returns_page(self):
        w = self.window
        self.assertEqual(w.stack.count(), len(w.pages))
        w._select_page(2)
        w._detach_page(3)
        # 拆窗口不能改变主界面下标：导航、_render_ui 都按固定下标工作。
        self.assertEqual(w.stack.count(), len(w.pages))
        self.assertEqual(w.stack.currentIndex(), 2)
        win = w.detached[3]
        self.assertIs(win.page(), w.pages[3])
        self.assertIs(w.pages[3].window(), win)
        self.assertIs(w.slots[3].currentWidget(), w.placeholders[3])
        self.assertTrue(w.nav_buttons[3].property("detached"))
        # 导航按钮指向已拆出的页面时，主界面显示占位卡而不是空白。
        w._select_page(3)
        self.assertEqual(w.stack.currentIndex(), 3)
        self.assertIs(w.slots[3].currentWidget(), w.placeholders[3])
        w._reattach_page(3)
        self.assertNotIn(3, w.detached)
        self.assertIs(w.slots[3].currentWidget(), w.pages[3])
        self.assertIs(w.pages[3].parent(), w.slots[3])
        self.assertFalse(w.nav_buttons[3].property("detached"))

    def test_detached_window_topmost_and_close_returns_page(self):
        w = self.window
        w._detach_page(4)
        win = w.detached[4]
        self.assertFalse(win.is_topmost())
        win.top_btn.setChecked(True)       # 等价于点击独立窗口里的“置顶”
        self.assertTrue(win.is_topmost())
        self.assertTrue(bool(win.windowFlags() & main.Qt.WindowStaysOnTopHint))
        self.assertEqual(win.top_btn.text(), "已置顶")
        self.assertTrue(w.placeholder_top_checks[4].isChecked())
        win.top_btn.setChecked(False)
        self.assertFalse(win.is_topmost())
        self.assertEqual(win.top_btn.text(), "置顶")
        self.assertFalse(w.placeholder_top_checks[4].isChecked())
        win.close()                        # 关闭独立窗口 = 页面回到主窗口
        self.assertNotIn(4, w.detached)
        self.assertIs(w.slots[4].currentWidget(), w.pages[4])

    def test_placeholder_checkbox_and_menu_action_set_topmost_pref(self):
        w = self.window
        w._set_page_topmost(5, True)
        self.assertTrue(w.topmost_pref[5])
        w._detach_page(5)
        # 分离时沿用预置的置顶偏好。
        self.assertTrue(w.detached[5].is_topmost())
        w._reattach_all()
        self.assertFalse(w.detached)
        self.assertIs(w.slots[5].currentWidget(), w.pages[5])

    def test_topmost_toggle_keeps_window_visible_and_placed(self):
        """setWindowFlag 会隐藏窗口，置顶/取消置顶都必须重新显示。"""
        w = self.window
        w.show()
        w._detach_page(3)
        win = w.detached[3]
        self.app.processEvents()
        self.assertTrue(win.isVisible())
        geometry = win.geometry()
        win.top_btn.setChecked(True)
        self.app.processEvents()
        self.assertTrue(win.is_topmost())
        self.assertTrue(win.isVisible())
        self.assertEqual(win.geometry(), geometry)
        win.top_btn.setChecked(False)
        self.app.processEvents()
        self.assertFalse(win.is_topmost())
        self.assertTrue(win.isVisible())
        self.assertEqual(win.geometry(), geometry)
        # 非活动状态下的重复调用不应把窗口藏起来。
        win.set_topmost(False)
        self.assertTrue(win.isVisible())

    def test_detached_window_is_not_owned_by_main_window(self):
        """独立窗口不能挂主窗口做父级。

        Windows 上带父级的顶层窗口是 owned window，会永久压在主窗口上面，
        用户无法把独立窗口换到主窗口下面；无父级窗口必须自带样式表补回外观。
        """
        w = self.window
        w._detach_page(3)
        win = w.detached[3]
        self.assertIsNone(win.parentWidget())
        self.assertTrue(win.isWindow())
        self.assertTrue(win.styleSheet())
        self.assertEqual(win.styleSheet(), w.styleSheet())
        # 拆出去的页面控件本身仍属于独立窗口。
        self.assertIs(w.pages[3].window(), win)

    def test_main_close_closes_detached_windows(self):
        w = self.window
        w.show()
        w._detach_page(2)
        win = w.detached[2]
        self.app.processEvents()
        self.assertTrue(win.isVisible())
        w.close()                       # 关主窗口不能让独立窗口留在桌面上
        self.app.processEvents()
        self.assertFalse(w.detached)
        self.assertFalse(win.isVisible())

    def test_detached_page_keeps_rendering_while_other_page_selected(self):
        w = self.window
        w._detach_page(2)                  # 比赛地图拆出去后仍要画轨迹
        w._select_page(6)
        frame = (140.0, -260.0, 15.0) + (0.0,) * 21
        w.latest = frame
        w.traj_ring.clear()
        w.traj_ring.append(0.0, (frame[0] * core.OPS_CM_TO_MM,
                                 frame[1] * core.OPS_CM_TO_MM))
        w._render_ui()
        self.assertEqual(w.map_view.trail_item.path().elementCount(), 1)

    def test_activation_guard_keeps_keyboard_remote_in_detached_window(self):
        w = self.window
        w._detach_page(3)
        win = w.detached[3]
        w.isActiveWindow = lambda: False
        win.isActiveWindow = lambda: True
        self.assertTrue(w._own_window_active())
        w._manual_start((10, 10, 0), raw=True)
        self.assertIsNotNone(w.manual_vector)
        w._activation_guard()
        self.assertIsNotNone(w.manual_vector)   # 焦点在独立窗口：不误停
        win.isActiveWindow = lambda: False
        w._activation_guard()
        self.assertIsNone(w.manual_vector)      # 整个应用失焦才停车


    def test_param_reply_parsed_and_repeated_values_delivered(self):
        """GET <名称> 的文字应答要能和 24 通道遥测共存，且重复值不能被去重吃掉。

        XVMIN/ZVMIN 没有遥测通道，回读值只能来自这条文本行；日志用的
        _seen_text 去重会把"连续两次相同数值"的第二条丢掉，所以参数行必须
        走独立通路。
        """
        parser = core.FrameParser()
        frame = struct.pack("<24f", *range(24)) + core.FRAME_TAIL
        self.assertEqual(len(parser.feed(frame)), 1)
        reply = b"XVMIN=5.000\r\n"
        parser.feed(reply)
        self.assertEqual(parser.take_params(), [("XVMIN", 5.0)])
        self.assertEqual(parser.take_text(), [])            # 参数行不进日志文本
        self.assertEqual(parser.take_params(), [])          # 取走后清空
        # 数值相同的重复回读同样要交付（日志去重不适用于状态量）
        parser.feed(reply)
        parser.feed(reply)
        self.assertEqual(parser.take_params(), [("XVMIN", 5.0), ("XVMIN", 5.0)])
        # 名字大小写在固件里统一回显大写，数值支持负号与小数
        parser.feed(b"zvmin=-2.500\r\n")
        self.assertEqual(parser.take_params(), [("ZVMIN", -2.5)])
        # 非"名称=数值"的可读行仍按固件文本处理
        parser.feed(b"ERR PARAM UNKNOWN; GET KPX|KPY|KPY\r\n")
        self.assertEqual(parser.take_params(), [])
        self.assertIn("ERR PARAM UNKNOWN", parser.take_text()[0])

    def test_serial_worker_routes_param_replies_to_param_queue(self):
        """工作线程把参数回读行分流转发，不能混进"固件文本"日志。"""
        param_q = queue.Queue()
        text_q = queue.Queue()
        worker = core.SerialWorker("COM_NONE", 115200, queue.Queue(), queue.Queue(),
                                   text_q=text_q, param_q=param_q)
        worker.parser.feed(b"ZVMIN=7.500\r\n")
        worker._flush_firmware_text()
        self.assertEqual(param_q.get_nowait(), ("ZVMIN", 7.5))
        self.assertTrue(text_q.empty())

    def test_param_readback_updates_row_without_channel(self):
        """没有遥测通道的参数行必须由文字回读点亮"回读"栏。"""
        w = self.window
        self.assertIsNone(w.chassis_rows["XVMIN"].readback_channel)
        self.assertIsNone(w.chassis_rows["ZVMIN"].readback_channel)
        self.assertEqual(w.chassis_rows["XVMIN"].readback.text(), "回读 —")
        w.param_q.put(("XVMIN", 7.5))
        w.param_q.put(("ZVMIN", 0.0))
        w._process_frames()
        self.assertEqual(w.chassis_rows["XVMIN"].readback.text(), "回读 7.5")
        self.assertEqual(w.chassis_rows["ZVMIN"].readback.text(), "回读 0")
        self.assertTrue(w.param_q.empty())
        # 未知名称不得抛异常，也不改动任何行
        w.param_q.put(("NOSUCH", 1.0))
        w._process_frames()
        self.assertEqual(w.chassis_rows["KPX"].readback.text(), "回读 —")

    def test_poll_queries_only_params_without_telemetry_channel(self):
        """轮询只问没有通道位的参数，一次一条，避免挤占遥测帧。"""
        w = self.window
        self.assertTrue(w.line_q.empty())
        w._poll_param_readback()
        self.assertEqual(w.line_q.get_nowait(), "GET XVMIN")
        w._poll_param_readback()
        self.assertEqual(w.line_q.get_nowait(), "GET ZVMIN")
        w._poll_param_readback()
        self.assertEqual(w.line_q.get_nowait(), "GET XVMIN")
        self.assertTrue(w.line_q.empty())

    def test_sending_param_requests_immediate_readback(self):
        """写入后立刻回读一次：这两个参数没有波形可以对照。"""
        w = self.window
        w.chassis_rows["XVMIN"].spin.setValue(8.0)
        w.chassis_rows["XVMIN"]._send()
        self.assertEqual(w.line_q.get_nowait(), "XVMIN=8")
        self.assertEqual(w.line_q.get_nowait(), "GET XVMIN")

    def test_simulator_answers_param_query(self):
        """演示模式也要给出回读值，否则界面在 --simulate 下永远显示"—"。"""
        w = self.window
        w.sim.param_q = w.param_q
        w.sim.handle_line("GET xvmin")
        self.assertEqual(w.param_q.get_nowait(), ("XVMIN", 5.0))
        # 固件里没有的参数不回任何内容
        w.sim.handle_line("GET NOSUCH")
        self.assertTrue(w.param_q.empty())


class VisionPageTests(unittest.TestCase):
    """视觉跟踪页（协议 V1.1）：页面注册、启停命令、参数回读与互斥。"""

    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.window = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.window.sim = core.Simulator(queue.Queue(), queue.Queue())
        self.window.sim.handle_line("ZERO")
        self.window.sim.make_frame(0.0)
        main.MainWindow._drain_queue(self.window.line_q)

    def tearDown(self):
        self.window.close()

    def _color_buttons(self):
        page = self.window.pages[self.window.vision_page_index]
        return [b for b in page.findChildren(main.QPushButton)
                if b.text().split()[0] in [n for _c, n in core.VISION_COLORS]]

    def test_page_registered_last_without_shifting_existing_pages(self):
        """新页面必须追加在末尾：既有测试与 _render_ui 依赖页面下标。"""
        w = self.window
        self.assertEqual(w.page_names[8], "视觉跟踪")
        self.assertEqual(w.vision_page_index, 8)
        self.assertEqual(w.page_names[-1], "轨迹调参")
        self.assertEqual(w.stack.count(), len(w.pages))
        self.assertEqual(w.page_names[:8], ["总览", "实时波形", "比赛地图", "底盘调参",
                                            "DM 电机", "数据记录", "命令终端", "28 / 35 步进"])
        self.assertEqual(w.pages[4].findChild(main.QLabel, "PageTitle").text(), "DM 电机")

    def test_page_has_six_colors_and_six_param_rows(self):
        w = self.window
        self.assertEqual(len(self._color_buttons()), len(core.VISION_COLORS))
        self.assertEqual([r.cmd for r in w.vision_rows.values()],
                         [cmd for cmd, _l, _lo, _hi, _d, _r in core.VISION_PARAMS])
        # 24 通道遥测里没有视觉位，这 6 行只能靠 GET 文字回读。
        for row in w.vision_rows.values():
            self.assertIsNone(row.readback_channel)
        self.assertEqual(len(core.VISION_PARAMS), 6)

    def test_color_buttons_send_the_bound_color(self):
        """循环建按钮最容易写成闭包全绑最后一个值，这里逐个点真按钮。"""
        w = self.window
        for btn in self._color_buttons():
            btn.click()
        got = [w.line_q.get_nowait() for _ in range(len(core.VISION_COLORS))]
        self.assertEqual(got, ["VTRACK=%d" % c for c, _n in core.VISION_COLORS])
        self.assertTrue(w.line_q.empty())

    def test_stop_button_sends_vtrack_zero(self):
        w = self.window
        page = w.pages[w.vision_page_index]
        stop = [b for b in page.findChildren(main.QPushButton) if b.text().startswith("■ 停止跟踪")]
        self.assertTrue(stop)
        stop[0].click()
        self.assertEqual(w.line_q.get_nowait(), "VTRACK=0")

    def test_vision_param_readback_updates_row(self):
        w = self.window
        self.assertEqual(w.vision_rows["VDBMM"].readback.text(), "回读 —")
        w.param_q.put(("VDBMM", 2.0))
        w.param_q.put(("VMIN", 8.0))
        w._process_frames()
        self.assertEqual(w.vision_rows["VDBMM"].readback.text(), "回读 2")
        self.assertEqual(w.vision_rows["VMIN"].readback.text(), "回读 8")

    def test_unknown_param_readback_is_logged_not_swallowed(self):
        """回归守卫：参数回读行在解析层就被分流出了日志，若界面没有对应行又静默
        丢弃，`GET XXX` 就完全没有可见反馈（曾把 VDBMM 整个吞掉）。"""
        w = self.window
        before = w.console.toPlainText()
        w.param_q.put(("NOSUCHPARAM", 1.5))
        w._process_frames()
        self.assertTrue(w.param_q.empty())
        self.assertIn("回读 NOSUCHPARAM=1.5", w.console.toPlainText())
        self.assertNotEqual(before, w.console.toPlainText())

    def test_param_echo_line_is_diverted_away_from_log(self):
        """根因锁定：`名称=数值` 在 FrameParser 里就被截走，不进日志文本。"""
        parser = core.FrameParser()
        parser.feed(b"VDBMM=2.000\r\n")
        self.assertEqual(parser.take_params(), [("VDBMM", 2.0)])
        self.assertEqual(parser.take_text(), [])

    def test_refresh_readback_queries_all_vision_params(self):
        w = self.window
        w._refresh_vision_readback()
        got = [w.line_q.get_nowait() for _ in range(len(core.VISION_PARAMS))]
        self.assertEqual(got, ["GET %s" % cmd for cmd, _l, _lo, _hi, _d, _r in core.VISION_PARAMS])
        self.assertTrue(w.line_q.empty())

    def test_entering_vision_page_refreshes_readback(self):
        w = self.window
        w._select_page(w.vision_page_index)
        got = [w.line_q.get_nowait() for _ in range(len(core.VISION_PARAMS))]
        self.assertEqual(got, ["GET %s" % cmd for cmd, _l, _lo, _hi, _d, _r in core.VISION_PARAMS])

    def test_sending_vision_param_requests_immediate_readback(self):
        w = self.window
        w.vision_rows["VDBMM"].spin.setValue(5.0)
        w.vision_rows["VDBMM"]._send()
        self.assertEqual(w.line_q.get_nowait(), "VDBMM=5")
        self.assertEqual(w.line_q.get_nowait(), "GET VDBMM")

    def test_simulator_answers_and_stores_vision_params(self):
        w = self.window
        w.sim.param_q = w.param_q
        w.sim.handle_line("GET VDBMM")
        self.assertEqual(w.param_q.get_nowait(), ("VDBMM", 2.0))
        w.sim.handle_line("VMIN=3")
        self.assertEqual(w.sim.vtrack_min, 3.0)
        w.sim.handle_line("VMIN=999")
        self.assertEqual(w.sim.vtrack_min, 60.0)
        w.sim.handle_line("VTRACK=3")
        self.assertEqual(w.sim.vtrack_color, 3)
        w.sim.handle_line("VTRACK=9")
        self.assertEqual(w.sim.vtrack_color, 3, "非法颜色不得改变跟踪状态")

    def test_vtrack_stops_keyboard_remote(self):
        """固件启动跟踪会清掉 manual，但上位机 20Hz MANUAL 流必须自己停，
        否则会把固件立刻拉回手动、与视觉速度互相打架。"""
        w = self.window
        w.manual_vector = (20, 0, 0)
        w.send_line("VTRACK=1")
        self.assertIsNone(w.manual_vector)
        self.assertFalse(w.keyboard_enabled)

    def test_queued_vtrack_is_discarded_on_stop(self):
        """STOP 之后的队列里不能残留一条待发的跟踪启动。"""
        q = core.CommandQueue()
        for cmd in ("VTRACK=1", "KPX=3", "MANUAL=10,0,0"):
            q.put(cmd)
        core.discard_motion_commands(q)
        self.assertEqual(list(q.queue), ["KPX=3"])
        # 安全命令绝不能被一起丢掉
        q.put("STOP")
        core.discard_motion_commands(q)
        self.assertEqual(list(q.queue), ["KPX=3", "STOP"])

    def test_vtrack_counts_as_commanding(self):
        self.assertEqual(core.commanding_commands(["VTRACK=1", "GET KPX", "PING"]), ["VTRACK=1"])


class SimulatedObstacleUiTests(unittest.TestCase):
    """模拟障碍界面：点击放置、上限/冲突拒绝、旧计划失效、重规划绕开、随机补齐、清除。

    离屏 Qt + 真实后台规划线程；障碍是运行时演示物体，不写进地图数据。
    """
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.w = main.MainWindow(argparse.Namespace(port=None, baud=115200, simulate=False))
        self.w.sim = core.Simulator(self.w.frame_q, self.w.line_q, self.w.urgent_q)
        self.w.sim.handle_line("ZERO")
        self.w.sim.make_frame(0.0)
        self.w.map_ox = self.w.map_oy = self.w.map_theta = 0.0
        self.w.latest = (0.0, 0.0, 0.0) + (0.0,) * 21
        self.w._select_page(2)

    def tearDown(self):
        self.w.worker = None
        self.w.close()
        self.w._planner_pool.shutdown(wait=True, cancel_futures=True)

    def plan(self, fx, fy):
        # 本版仿真行驶不执行路径中转向：障碍用例统一用高转向代价，得到固定航向的可执行路线
        self.w.turn_penalty_spin.setValue(5000.0)
        self.w.sim.make_frame(0.0)      # 规划可能超过 350ms 定位闸门，先刷新位姿
        self.w._request_plan(fx, fy)
        deadline = time.monotonic() + 20.0
        while self.w._plan_future is not None and time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.01)
        self.assertIsNone(self.w._plan_future, "规划未在 20s 内结束")
        return self.w.planned_result or {}

    def test_click_places_obstacle_without_planning_or_sending(self):
        w = self.w
        w.obstacle_mode_check.setChecked(True)
        while not w.line_q.empty():
            w.line_q.get_nowait()
        before = w.planned_result
        w._on_map_click(1200.0, 2050.0)
        self.assertEqual(w.sim_obstacles, [(1200.0, 2050.0)])
        self.assertEqual(core.commanding_commands(list(w.line_q.queue)), [])
        self.assertIs(w.planned_result, before)          # 放置模式不触发规划
        self.assertIn("1/4", w.obstacle_info.text())
        self.assertEqual(len(w.map_view._obstacle_items), 1)

    def test_placement_rejects_overlap_limit_and_car(self):
        w = self.w
        w.obstacle_mode_check.setChecked(True)
        w._on_map_click(1200.0, 2400.0)                  # 正是原料区圆盘
        self.assertEqual(w.sim_obstacles, [])
        self.assertIn("原料区圆盘", w.map_status.text())
        w._on_map_click(20.0, 1200.0)                    # 贴边
        self.assertEqual(w.sim_obstacles, [])
        for x, y in ((900.0, 2050.0), (1500.0, 2050.0), (1200.0, 1980.0), (600.0, 1200.0)):
            w._on_map_click(x, y)
        self.assertEqual(len(w.sim_obstacles), core.SIM_OBSTACLE_MAX)
        w._on_map_click(1300.0, 1900.0)                  # 第 5 个必须被上限挡住
        self.assertEqual(len(w.sim_obstacles), core.SIM_OBSTACLE_MAX)
        self.assertIn("上限", w.map_status.text())
        w._clear_obstacles()
        w._on_map_click(2250.0, 2250.0)                  # 压在车上（车在启停区1中心）
        self.assertEqual(w.sim_obstacles, [])
        self.assertIn("压", w.map_status.text())

    def test_plan_detours_and_old_plan_is_invalidated(self):
        w = self.w
        base = self.plan(330.0, 1200.0)
        self.assertTrue(base.get("ok"), w.map_status.text())
        pts = list(w.planned_points)
        i = len(pts) // 2 - 1
        mx = (pts[i][0] + pts[i + 1][0]) / 2.0
        my = (pts[i][1] + pts[i + 1][1]) / 2.0
        w.obstacle_mode_check.setChecked(True)
        w._on_map_click(mx, my)
        self.assertEqual(len(w.sim_obstacles), 1)
        self.assertEqual(w.planned_points, [])                        # 旧计划立刻失效
        self.assertFalse(w.map_view.path_item.path().elementCount())
        again = self.plan(330.0, 1200.0)
        self.assertTrue(again.get("ok"), w.map_status.text())
        self.assertNotEqual(list(w.planned_points), pts)              # 确实改道
        self.assertGreater(core.path_clearance(
            list(w.planned_points), [], core.sim_obstacle_circles(w.sim_obstacles)), 0.0)
        # 仿真行驶前的复查用的是同一套（含障碍）几何
        self.assertIsNone(w._scene_for_context(w._planned_context).validate(list(w.planned_points)))

    def test_random_fill_avoids_car_and_clear_removes(self):
        w = self.w
        w._random_obstacles()
        self.assertEqual(len(w.sim_obstacles), core.SIM_OBSTACLE_MAX)
        self.assertIn("%d/%d" % (core.SIM_OBSTACLE_MAX, core.SIM_OBSTACLE_MAX), w.obstacle_info.text())
        for i, (x, y) in enumerate(w.sim_obstacles):
            self.assertIsNone(core.obstacle_placement_blocked(
                x, y, rects=w.nav_map["rects"],
                circles=list(w.nav_map["circles"]) + core.sim_obstacle_circles(w.sim_obstacles[:i]),
                bounds=w.nav_map["bounds"]), (x, y))
        keep = core.CAR_HALF_DIAG_MM + core.SIM_OBSTACLE_R_MM + 20.0
        for x, y in w.sim_obstacles:                                   # 不许压在车上
            self.assertGreater(math.hypot(x - 2250.0, y - 2250.0), keep - 1e-6)
        w._clear_obstacles()
        self.assertEqual(w.sim_obstacles, [])
        self.assertEqual(w.map_view._obstacle_items, [])


if __name__ == "__main__":
    unittest.main()
