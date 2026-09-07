# -*- coding: utf-8 -*-
"""ILHC Control Station v2.0 — PySide6 + PyQtGraph UI.

保留 v1.1 的通信/安全核心，重做桌面 UI：
- PySide6 Qt Widgets
- PyQtGraph 实时波形
- QGraphicsView 比赛场地地图
- 高优先级 STOP/DMOFF、GUI 心跳、遥测超时
- 底盘/DM 参数调节、CSV 记录、命令行、模拟模式
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import os
import queue
import sys
import time
from pathlib import Path

import numpy as np

import core

# --selftest 在加载 PySide6/PyQtGraph 之前执行，便于无 GUI 环境验证核心。
if "--selftest" in sys.argv:
    core.selftest()
    raise SystemExit(0)

try:
    import pyqtgraph as pg
    from PySide6.QtCore import QObject, QPointF, QRectF, Qt, QTimer, QUrl, Signal
    from PySide6.QtGui import (
        QBrush,
        QColor,
        QCloseEvent,
        QDesktopServices,
        QFont,
        QPainter,
        QPainterPath,
        QPen,
    )
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFileDialog,
        QFrame,
        QGraphicsEllipseItem,
        QGraphicsLineItem,
        QGraphicsPathItem,
        QGraphicsRectItem,
        QGraphicsScene,
        QGraphicsSimpleTextItem,
        QGraphicsView,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QSlider,
        QSpinBox,
        QStackedWidget,
        QTextEdit,
        QTreeWidget,
        QTreeWidgetItem,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:  # 让缺依赖时给出清晰提示
    print("缺少 Qt UI 依赖：%s" % exc)
    print("请在项目目录执行：python -m pip install -r requirements.txt")
    raise SystemExit(2)


BASE_DIR = Path(__file__).resolve().parent
ACCENT = "#4FC3F7"
GREEN = "#5AD68A"
AMBER = "#FFB74D"
RED = "#FF5D67"
FG = "#DCE3ED"
FG_DIM = "#8F9BAB"
PANEL = "#1E232B"
PANEL_2 = "#242A33"
BG = "#14181E"
PLOT_BG = "#171C23"
GRID = "#303843"


class UiBridge(QObject):
    workerError = Signal(str)


class StatCard(QFrame):
    def __init__(self, title: str, value: str = "—", unit: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("StatCard")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(3)
        self.title = QLabel(title)
        self.title.setObjectName("CardTitle")
        lay.addWidget(self.title)
        row = QHBoxLayout()
        row.setSpacing(6)
        self.value = QLabel(value)
        self.value.setObjectName("CardValue")
        self.unit = QLabel(unit)
        self.unit.setObjectName("CardUnit")
        row.addWidget(self.value)
        row.addWidget(self.unit)
        row.addStretch(1)
        lay.addLayout(row)

    def set_value(self, value: str, unit: str | None = None):
        self.value.setText(value)
        if unit is not None:
            self.unit.setText(unit)


class ParamRow(QFrame):
    sendRequested = Signal(str, str)

    def __init__(
        self,
        cmd: str,
        label: str,
        lo: float,
        hi: float,
        default: float,
        readback_channel: int | None,
        integer: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self.setObjectName("ParamRow")
        self.cmd = cmd
        self.lo = lo
        self.hi = hi
        self.readback_channel = readback_channel
        self.integer = integer

        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 7, 10, 7)
        lay.setSpacing(10)

        cmd_lb = QLabel(cmd)
        cmd_lb.setObjectName("ParamCmd")
        cmd_lb.setFixedWidth(68)
        lay.addWidget(cmd_lb)

        desc = QLabel(label)
        desc.setObjectName("ParamDesc")
        desc.setMinimumWidth(130)
        lay.addWidget(desc)

        if integer:
            spin = QSpinBox()
            spin.setRange(int(lo), int(hi))
            spin.setValue(int(default))
            spin.setFixedWidth(92)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(lo), int(hi))
            slider.setValue(int(default))
        else:
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            span = max(abs(lo), abs(hi), abs(hi - lo))
            spin.setDecimals(3 if span <= 100 else 1)
            spin.setSingleStep(max((hi - lo) / 200.0, 0.01))
            spin.setValue(default)
            spin.setFixedWidth(100)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000)
            slider.setValue(self._float_to_slider(default))

        self.spin = spin
        self.slider = slider
        lay.addWidget(spin)
        lay.addWidget(slider, 1)

        self.readback = QLabel("回读 —")
        self.readback.setObjectName("Readback")
        self.readback.setMinimumWidth(105)
        lay.addWidget(self.readback)

        send = QPushButton("发送")
        send.setObjectName("SmallPrimary")
        send.setFixedWidth(62)
        send.clicked.connect(self._send)
        lay.addWidget(send)

        self.slider.valueChanged.connect(self._slider_changed)
        self.spin.valueChanged.connect(self._spin_changed)

    def _float_to_slider(self, value: float) -> int:
        if self.hi <= self.lo:
            return 0
        return int(round((float(value) - self.lo) / (self.hi - self.lo) * 1000.0))

    def _slider_to_float(self, value: int) -> float:
        return self.lo + (self.hi - self.lo) * float(value) / 1000.0

    def _slider_changed(self, value: int):
        self.spin.blockSignals(True)
        if self.integer:
            self.spin.setValue(value)
        else:
            self.spin.setValue(self._slider_to_float(value))
        self.spin.blockSignals(False)

    def _spin_changed(self, value):
        self.slider.blockSignals(True)
        if self.integer:
            self.slider.setValue(int(value))
        else:
            self.slider.setValue(self._float_to_slider(float(value)))
        self.slider.blockSignals(False)

    def _send(self):
        if self.integer:
            value = str(int(self.spin.value()))
        else:
            value = ("%.4f" % float(self.spin.value())).rstrip("0").rstrip(".")
        self.sendRequested.emit(self.cmd, value)

    def command_text(self) -> str:
        if self.integer:
            return "%s=%d" % (self.cmd, int(self.spin.value()))
        value = ("%.4f" % float(self.spin.value())).rstrip("0").rstrip(".")
        return "%s=%s" % (self.cmd, value)

    def set_readback(self, value: float):
        if not math.isfinite(value):
            self.readback.setText("回读 —")
            return
        if self.integer:
            text = str(int(round(value)))
        else:
            text = ("%.3f" % value).rstrip("0").rstrip(".")
        self.readback.setText("回读 " + text)


class FieldView(QGraphicsView):
    gotoRequested = Signal(float, float)

    FIELD = 2400.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("FieldView")
        self.scene_obj = QGraphicsScene(self)
        self.setScene(self.scene_obj)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.NoDrag)
        self.click_enabled = True
        self._trail_points: list[QPointF] = []
        self._build_field()

    @classmethod
    def sy(cls, field_y: float) -> float:
        return cls.FIELD - field_y

    def _add_centered_text(self, text: str, fx: float, fy: float, color: str, size=10, bold=False):
        item = QGraphicsSimpleTextItem(text)
        font = QFont("Microsoft YaHei UI", size)
        font.setBold(bold)
        item.setFont(font)
        item.setBrush(QColor(color))
        self.scene_obj.addItem(item)
        br = item.boundingRect()
        item.setPos(fx - br.width() / 2, self.sy(fy) - br.height() / 2)
        return item

    def _field_rect(self, x0, y0, x1, y1, fill, edge="#4A5260", width=1.0, z=2):
        item = QGraphicsRectItem(x0, self.sy(y1), x1 - x0, y1 - y0)
        item.setBrush(QBrush(QColor(fill)))
        item.setPen(QPen(QColor(edge), width))
        item.setZValue(z)
        self.scene_obj.addItem(item)
        return item

    def _field_circle(self, cx, cy, r, fill, edge="#4A5260", width=1.0, z=3):
        item = QGraphicsEllipseItem(cx - r, self.sy(cy) - r, 2 * r, 2 * r)
        item.setBrush(QBrush(QColor(fill)))
        item.setPen(QPen(QColor(edge), width))
        item.setZValue(z)
        self.scene_obj.addItem(item)
        return item

    def _build_field(self):
        self.scene_obj.clear()
        self.scene_obj.setSceneRect(-180, -180, 2760, 2760)

        floor = QGraphicsRectItem(0, 0, self.FIELD, self.FIELD)
        floor.setBrush(QBrush(QColor("#D2D6DC")))
        floor.setPen(QPen(QColor("#59616D"), 6))
        floor.setZValue(0)
        self.scene_obj.addItem(floor)

        for v in range(0, 2401, 300):
            p = QPen(QColor("#AAB1BA"), 1)
            p.setCosmetic(True)
            self.scene_obj.addLine(v, 0, v, self.FIELD, p)
            self.scene_obj.addLine(0, self.sy(v), self.FIELD, self.sy(v), p)

        # 中央物料区
        for x0 in (550, 1400):
            for y0 in (550, 1400):
                self._field_rect(x0, y0, x0 + 450, y0 + 450, "#F6F0CB", "#B7AB69", 2)

        # 启停区
        self._field_rect(2100, 2100, 2400, 2400, "#245BE8", "#153A9E", 2)
        self._field_rect(2100, 0, 2400, 300, "#245BE8", "#153A9E", 2)
        self._add_centered_text("启停区1", 2250, 2250, "#FFFFFF", 10, True)
        self._add_centered_text("启停区2", 2250, 150, "#FFFFFF", 10, True)

        # 原料区
        self._field_circle(1200, 2400, 110, "#EDF1F5", "#4B535F", 2)
        for hx, hy in ((1200, 2445), (1160, 2375), (1240, 2375)):
            self._field_circle(hx, hy, 20, "#7D8793", "#4B535F", 1)
        self._add_centered_text("原料区", 1200, 2255, "#454B55", 10, True)

        # 暂存区
        self._field_rect(0, 910, 150, 1490, "#EDF1F5", "#4B535F", 2)
        for hy in (1050, 1200, 1350):
            self._field_circle(75, hy, 24, "#7D8793", "#4B535F", 1)
        self._add_centered_text("暂存区", 255, 1200, "#454B55", 10, True)

        # 粗加工区
        self._field_rect(1000, 0, 1400, 150, "#EDF1F5", "#4B535F", 2)
        for hx in (1075, 1200, 1325):
            self._field_circle(hx, 75, 24, "#7D8793", "#4B535F", 1)
        self._add_centered_text("粗加工区", 1200, 255, "#454B55", 10, True)

        # 二维码板
        self._field_rect(2388, 1170, 2400, 1230, "#171A1E", "#171A1E", 1)
        self._add_centered_text("二维码板", 2275, 1200, "#454B55", 9, True)

        # 动态图元
        self.trail_item = QGraphicsPathItem()
        self.trail_item.setPen(QPen(QColor(ACCENT), 5))
        self.trail_item.setZValue(10)
        self.scene_obj.addItem(self.trail_item)

        self.car_item = QGraphicsEllipseItem()
        self.car_item.setBrush(QBrush(QColor("#FF9F43")))
        self.car_item.setPen(QPen(QColor("#FFFFFF"), 3))
        self.car_item.setZValue(13)
        self.scene_obj.addItem(self.car_item)

        self.dir_item = QGraphicsLineItem()
        self.dir_item.setPen(QPen(QColor("#FF9F43"), 7))
        self.dir_item.setZValue(12)
        self.scene_obj.addItem(self.dir_item)

        self.target_h = QGraphicsLineItem()
        self.target_v = QGraphicsLineItem()
        for item in (self.target_h, self.target_v):
            item.setPen(QPen(QColor(RED), 7))
            item.setZValue(14)
            item.setVisible(False)
            self.scene_obj.addItem(item)

        # 场地坐标角标
        self._add_centered_text("0", 0, -80, "#8B95A3", 8)
        self._add_centered_text("2400 mm", 2400, -80, "#8B95A3", 8)

        self.fitInView(self.scene_obj.sceneRect(), Qt.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.scene_obj.sceneRect(), Qt.KeepAspectRatio)

    def mousePressEvent(self, event):
        if self.click_enabled and event.button() == Qt.LeftButton:
            p = self.mapToScene(event.position().toPoint())
            fx = min(self.FIELD, max(0.0, float(p.x())))
            fy = min(self.FIELD, max(0.0, self.FIELD - float(p.y())))
            if 0 <= p.x() <= self.FIELD and 0 <= p.y() <= self.FIELD:
                self.gotoRequested.emit(fx, fy)
                return
        super().mousePressEvent(event)

    def set_pose(self, fx: float, fy: float, yaw_deg: float):
        if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(yaw_deg)):
            return
        sx, sy = fx, self.sy(fy)
        r = 48.0
        self.car_item.setRect(sx - r, sy - r, 2 * r, 2 * r)
        th = math.radians(yaw_deg)
        ex = fx + 180.0 * math.cos(th)
        ey = fy + 180.0 * math.sin(th)
        self.dir_item.setLine(sx, sy, ex, self.sy(ey))

    def set_trail(self, x: np.ndarray, y: np.ndarray):
        if len(x) == 0:
            self.trail_item.setPath(QPainterPath())
            return
        path = QPainterPath(QPointF(float(x[0]), self.sy(float(y[0]))))
        for xx, yy in zip(x[1:], y[1:]):
            path.lineTo(float(xx), self.sy(float(yy)))
        self.trail_item.setPath(path)

    def set_target(self, fx: float | None, fy: float | None):
        if fx is None or fy is None:
            self.target_h.setVisible(False)
            self.target_v.setVisible(False)
            return
        sx, sy = float(fx), self.sy(float(fy))
        d = 65.0
        self.target_h.setLine(sx - d, sy, sx + d, sy)
        self.target_v.setLine(sx, sy - d, sx, sy + d)
        self.target_h.setVisible(True)
        self.target_v.setVisible(True)


class WavePage(QWidget):
    windowChanged = Signal(float)
    clearRequested = Signal()

    DEFAULT_VISIBLE = {0, 1, 2, 11, 13, 14}

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        top = QFrame()
        top.setObjectName("Panel")
        tl = QHBoxLayout(top)
        tl.setContentsMargins(14, 10, 14, 10)
        tl.addWidget(QLabel("实时遥测波形"))
        tl.addStretch(1)
        tl.addWidget(QLabel("时间窗"))
        self.window_combo = QComboBox()
        self.window_combo.addItems(["10 s", "30 s", "60 s", "120 s"])
        self.window_combo.setCurrentText("30 s")
        self.window_combo.currentTextChanged.connect(
            lambda s: self.windowChanged.emit(float(s.split()[0]))
        )
        tl.addWidget(self.window_combo)
        clear = QPushButton("清除波形")
        clear.clicked.connect(self.clearRequested.emit)
        tl.addWidget(clear)
        lay.addWidget(top)

        body = QHBoxLayout()
        body.setSpacing(10)

        channel_panel = QFrame()
        channel_panel.setObjectName("Panel")
        channel_panel.setFixedWidth(225)
        cp = QVBoxLayout(channel_panel)
        cp.setContentsMargins(12, 12, 12, 12)
        title = QLabel("通道选择")
        title.setObjectName("SectionTitle")
        cp.addWidget(title)
        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.setRootIsDecorated(True)
        cp.addWidget(self.tree, 1)
        body.addWidget(channel_panel)

        plots_wrap = QWidget()
        grid = QGridLayout(plots_wrap)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(10)
        self.plots = {}
        self.curves = {}
        self.items = {}

        pg.setConfigOptions(antialias=False, background=PLOT_BG, foreground=FG_DIM)
        for g, pos in core.GROUP_AX_POS.items():
            p = pg.PlotWidget()
            p.setObjectName("Plot")
            p.showGrid(x=True, y=True, alpha=0.22)
            p.setTitle(core.GROUP_NAMES[g], color=ACCENT, size="10pt")
            p.setLabel("bottom", "t", units="s")
            p.getPlotItem().setMenuEnabled(False)
            p.getPlotItem().hideButtons()
            p.addLegend(offset=(8, 8), labelTextColor=FG_DIM)
            self.plots[g] = p
            grid.addWidget(p, pos[0], pos[1])
        body.addWidget(plots_wrap, 1)
        lay.addLayout(body, 1)

        parents = {}
        for g in sorted(core.GROUP_NAMES):
            root = QTreeWidgetItem([core.GROUP_NAMES[g]])
            root.setFlags(root.flags() & ~Qt.ItemIsUserCheckable)
            self.tree.addTopLevelItem(root)
            root.setExpanded(True)
            parents[g] = root

        for idx, key, label, unit, group in core.CHANNELS:
            item = QTreeWidgetItem([label + (("  [%s]" % unit) if unit else "")])
            item.setData(0, Qt.UserRole, idx)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(0, Qt.Checked if idx in self.DEFAULT_VISIBLE else Qt.Unchecked)
            item.setForeground(0, QBrush(QColor(core.COLORS[idx])))
            parents[group].addChild(item)
            self.items[idx] = item
            curve = self.plots[group].plot(
                [], [], pen=pg.mkPen(core.COLORS[idx], width=1.5), name=label
            )
            curve.setVisible(idx in self.DEFAULT_VISIBLE)
            self.curves[idx] = curve

        self.tree.itemChanged.connect(self._item_changed)
        self._yrange_counter = 0

    def _item_changed(self, item: QTreeWidgetItem, _col: int):
        idx = item.data(0, Qt.UserRole)
        if idx is None:
            return
        self.curves[int(idx)].setVisible(item.checkState(0) == Qt.Checked)

    def update_data(self, view, latest_t: float, window_s: float):
        if view is None:
            return
        t, data = view
        if len(t) == 0:
            return
        start = np.searchsorted(t, latest_t - window_s)
        t = t[start:]
        data = data[start:]
        if len(t) == 0:
            return

        for idx, _key, _label, _unit, _group in core.CHANNELS:
            if not self.curves[idx].isVisible():
                continue
            col = data[:, idx]
            if not np.all(np.isfinite(col)):
                col = np.where(np.isfinite(col), col, np.nan)
            self.curves[idx].setData(t, col, connect="finite")

        for plot in self.plots.values():
            plot.setXRange(max(0.0, latest_t - window_s), latest_t + 0.02, padding=0)

        self._yrange_counter += 1
        if self._yrange_counter % 10 == 0:
            self._update_y_ranges(data)

    def _update_y_ranges(self, data: np.ndarray):
        ranges = {}
        for idx, _key, _label, _unit, group in core.CHANNELS:
            if not self.curves[idx].isVisible():
                continue
            col = data[:, idx]
            finite = col[np.isfinite(col)]
            if finite.size == 0:
                continue
            lo, hi = float(finite.min()), float(finite.max())
            if group in ranges:
                a, b = ranges[group]
                ranges[group] = min(a, lo), max(b, hi)
            else:
                ranges[group] = lo, hi
        for group, plot in self.plots.items():
            if group not in ranges:
                continue
            lo, hi = ranges[group]
            if hi <= lo:
                span = max(1.0, abs(lo) * 0.1)
                lo, hi = lo - span, hi + span
            else:
                pad = (hi - lo) * 0.12
                lo, hi = lo - pad, hi + pad
            plot.setYRange(lo, hi, padding=0)


class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setWindowTitle("ILHC Control Station v2.0 — STM32F407VET6")
        self.resize(1500, 930)
        self.setMinimumSize(1180, 760)

        # 后端状态
        self.frame_q = queue.Queue(maxsize=4000)
        self.line_q = queue.Queue()
        self.urgent_q = queue.Queue()
        self.worker = None
        self.sim = None
        self.recorder = None
        self.t0_monotonic = time.monotonic()
        self.latest = None
        self.latest_t = 0.0
        self.window_s = 30.0
        self.ring = core.RingBuffer(int(self.window_s * core.SEND_HZ) + 100, core.FRAME_FLOATS)
        self.traj_ring = core.RingBuffer(int(self.window_s * core.SEND_HZ) + 100, 2)
        self.paused = False
        self.map_ox = 0.0
        self.map_oy = 0.0
        self.map_theta = 0.0
        self.map_target = None
        self.last_heartbeat_enqueue = 0.0
        self.fps_count = 0
        self.fps = 0.0
        self.fps_t = time.monotonic()
        self.send_count = 0
        self.cmd_history = []
        self.hist_idx = 0

        self.bridge = UiBridge()
        self.bridge.workerError.connect(self._on_worker_error)

        self._build_ui()
        self._load_style()
        self.refresh_ports()
        self._start_timers()

        if args.baud:
            i = self.baud_combo.findText(str(args.baud))
            if i >= 0:
                self.baud_combo.setCurrentIndex(i)
        if args.simulate:
            self.toggle_sim(True)
        elif args.port:
            idx = self.port_combo.findData(args.port)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
            else:
                self.port_combo.addItem(args.port, args.port)
                self.port_combo.setCurrentIndex(self.port_combo.count() - 1)
            self.toggle_connect(True)

    # ---------------- UI ----------------
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)

        main.addWidget(self._build_topbar())

        center = QHBoxLayout()
        center.setContentsMargins(0, 0, 0, 0)
        center.setSpacing(0)
        center.addWidget(self._build_sidebar())

        content = QFrame()
        content.setObjectName("Content")
        cl = QVBoxLayout(content)
        cl.setContentsMargins(16, 16, 16, 14)
        cl.setSpacing(0)
        self.stack = QStackedWidget()
        cl.addWidget(self.stack)
        center.addWidget(content, 1)
        main.addLayout(center, 1)

        main.addWidget(self._build_footer())

        self.pages = [
            self._build_dashboard_page(),
            self._build_wave_page(),
            self._build_map_page(),
            self._build_chassis_page(),
            self._build_dm_page(),
            self._build_record_page(),
            self._build_console_page(),
        ]
        for page in self.pages:
            self.stack.addWidget(page)
        self._select_page(0)

    def _build_topbar(self):
        bar = QFrame()
        bar.setObjectName("TopBar")
        bar.setFixedHeight(72)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(18, 10, 18, 10)
        lay.setSpacing(10)

        titlebox = QVBoxLayout()
        titlebox.setSpacing(0)
        title = QLabel("ILHC CONTROL STATION")
        title.setObjectName("AppTitle")
        sub = QLabel("STM32F407 · OPS · DM Motor · 24-CH Telemetry")
        sub.setObjectName("AppSubtitle")
        titlebox.addWidget(title)
        titlebox.addWidget(sub)
        lay.addLayout(titlebox)
        lay.addSpacing(18)

        self.status_pill = QLabel("● 未连接")
        self.status_pill.setObjectName("StatusPill")
        self.status_pill.setProperty("state", "idle")
        lay.addWidget(self.status_pill)
        lay.addStretch(1)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        lay.addWidget(self.port_combo)
        refresh = QPushButton("刷新")
        refresh.clicked.connect(self.refresh_ports)
        lay.addWidget(refresh)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800"])
        self.baud_combo.setCurrentText(str(core.DEFAULT_BAUD))
        lay.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.setObjectName("ConnectButton")
        self.connect_btn.clicked.connect(lambda: self.toggle_connect())
        lay.addWidget(self.connect_btn)

        self.sim_btn = QPushButton("模拟")
        self.sim_btn.clicked.connect(lambda: self.toggle_sim())
        lay.addWidget(self.sim_btn)

        self.pause_btn = QPushButton("暂停")
        self.pause_btn.clicked.connect(self.toggle_pause)
        lay.addWidget(self.pause_btn)

        self.record_top_btn = QPushButton("● 记录")
        self.record_top_btn.setObjectName("RecordButton")
        self.record_top_btn.clicked.connect(self.toggle_record)
        lay.addWidget(self.record_top_btn)

        stop = QPushButton("■  全局急停")
        stop.setObjectName("EmergencyButton")
        stop.clicked.connect(lambda: self.send_line("STOP"))
        lay.addWidget(stop)
        return bar

    def _build_sidebar(self):
        side = QFrame()
        side.setObjectName("Sidebar")
        side.setFixedWidth(176)
        lay = QVBoxLayout(side)
        lay.setContentsMargins(10, 16, 10, 14)
        lay.setSpacing(6)

        sec = QLabel("WORKSPACE")
        sec.setObjectName("SidebarSection")
        lay.addWidget(sec)

        names = ["总览", "实时波形", "比赛地图", "底盘调参", "DM 电机", "数据记录", "命令终端"]
        self.nav_buttons = []
        for i, name in enumerate(names):
            btn = QPushButton(name)
            btn.setProperty("nav", True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda _=False, x=i: self._select_page(x))
            lay.addWidget(btn)
            self.nav_buttons.append(btn)

        lay.addStretch(1)
        ver = QLabel("v2.0 Qt\nPySide6 + PyQtGraph")
        ver.setObjectName("VersionLabel")
        lay.addWidget(ver)
        return side

    def _build_footer(self):
        bar = QFrame()
        bar.setObjectName("Footer")
        bar.setFixedHeight(32)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(14, 0, 14, 0)
        lay.setSpacing(20)
        self.footer_link = QLabel("Telemetry —")
        self.footer_fps = QLabel("0 fps")
        self.footer_rx = QLabel("RX 0 B")
        self.footer_err = QLabel("Err 0")
        self.footer_dm = QLabel("DM —")
        for w in (self.footer_link, self.footer_fps, self.footer_rx, self.footer_err):
            lay.addWidget(w)
        lay.addStretch(1)
        lay.addWidget(self.footer_dm)
        return bar

    def _page_shell(self, title: str, subtitle: str):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)
        head = QFrame()
        head.setObjectName("PageHeader")
        hl = QVBoxLayout(head)
        hl.setContentsMargins(2, 0, 2, 4)
        hl.setSpacing(2)
        t = QLabel(title)
        t.setObjectName("PageTitle")
        s = QLabel(subtitle)
        s.setObjectName("PageSubtitle")
        hl.addWidget(t)
        hl.addWidget(s)
        lay.addWidget(head)
        return page, lay

    def _build_dashboard_page(self):
        page, lay = self._page_shell("系统总览", "实时查看底盘定位、DM 电机和通信链路状态")

        cards = QGridLayout()
        cards.setSpacing(10)
        self.cards = {
            "x": StatCard("OPS X", "—", "mm"),
            "y": StatCard("OPS Y", "—", "mm"),
            "yaw": StatCard("航向角", "—", "°"),
            "dm_pos": StatCard("DM 位置", "—", "rad"),
            "dm_vel": StatCard("DM 速度", "—", "rad/s"),
            "temp": StatCard("DM 温度", "—", "°C"),
        }
        for i, card in enumerate(self.cards.values()):
            cards.addWidget(card, 0, i)
        lay.addLayout(cards)

        mid = QHBoxLayout()
        mid.setSpacing(10)

        # OPS mini plot
        ops_frame = QFrame()
        ops_frame.setObjectName("Panel")
        ol = QVBoxLayout(ops_frame)
        ol.setContentsMargins(14, 12, 14, 12)
        title = QLabel("OPS POSITION")
        title.setObjectName("SectionTitle")
        ol.addWidget(title)
        self.dash_ops_plot = pg.PlotWidget()
        self.dash_ops_plot.showGrid(x=True, y=True, alpha=0.2)
        self.dash_ops_plot.getPlotItem().hideButtons()
        self.dash_ops_plot.getPlotItem().setMenuEnabled(False)
        self.dash_x = self.dash_ops_plot.plot(pen=pg.mkPen(core.COLORS[0], width=1.7), name="X")
        self.dash_y = self.dash_ops_plot.plot(pen=pg.mkPen(core.COLORS[1], width=1.7), name="Y")
        ol.addWidget(self.dash_ops_plot, 1)
        mid.addWidget(ops_frame, 2)

        dm_frame = QFrame()
        dm_frame.setObjectName("Panel")
        dl = QVBoxLayout(dm_frame)
        dl.setContentsMargins(14, 12, 14, 12)
        title = QLabel("DM MOTOR")
        title.setObjectName("SectionTitle")
        dl.addWidget(title)
        self.dash_dm_plot = pg.PlotWidget()
        self.dash_dm_plot.showGrid(x=True, y=True, alpha=0.2)
        self.dash_dm_plot.getPlotItem().hideButtons()
        self.dash_dm_plot.getPlotItem().setMenuEnabled(False)
        self.dash_dm_pos = self.dash_dm_plot.plot(pen=pg.mkPen(core.COLORS[13], width=1.7), name="Pos")
        self.dash_dm_vel = self.dash_dm_plot.plot(pen=pg.mkPen(core.COLORS[14], width=1.7), name="Vel")
        dl.addWidget(self.dash_dm_plot, 1)
        mid.addWidget(dm_frame, 2)

        status = QFrame()
        status.setObjectName("Panel")
        sl = QVBoxLayout(status)
        sl.setContentsMargins(16, 14, 16, 14)
        st = QLabel("SYSTEM HEALTH")
        st.setObjectName("SectionTitle")
        sl.addWidget(st)
        self.health_link = QLabel("● 通信链路：未连接")
        self.health_telemetry = QLabel("● 遥测：—")
        self.health_dm = QLabel("● DM：—")
        self.health_record = QLabel("● 记录：未记录")
        for lb in (self.health_link, self.health_telemetry, self.health_dm, self.health_record):
            lb.setObjectName("HealthItem")
            sl.addWidget(lb)
        sl.addStretch(1)
        mid.addWidget(status, 1)
        lay.addLayout(mid, 1)
        return page

    def _build_wave_page(self):
        page, lay = self._page_shell("实时波形", "24 通道遥测，PyQtGraph 高频绘制")
        self.wave_page = WavePage()
        self.wave_page.windowChanged.connect(self._set_window)
        self.wave_page.clearRequested.connect(self.clear_data)
        lay.addWidget(self.wave_page, 1)
        return page

    def _build_map_page(self):
        page, lay = self._page_shell("比赛场地", "点击场地发送 GOTO；v2 保留静态禁区/直线路径安全检查")

        controls = QFrame()
        controls.setObjectName("Panel")
        cl = QHBoxLayout(controls)
        cl.setContentsMargins(12, 10, 12, 10)
        cl.setSpacing(8)

        cl.addWidget(QLabel("启停区"))
        self.zone_combo = QComboBox()
        self.zone_combo.addItem("启停区1（右上）", 1)
        self.zone_combo.addItem("启停区2（右下）", 2)
        cl.addWidget(self.zone_combo)
        origin = QPushButton("置为原点并归零")
        origin.setObjectName("WarningButton")
        origin.clicked.connect(self._set_start_zone)
        cl.addWidget(origin)

        cl.addSpacing(12)
        cl.addWidget(QLabel("原点 X"))
        self.map_ox_spin = QDoubleSpinBox()
        self.map_ox_spin.setRange(-5000, 5000)
        self.map_ox_spin.setDecimals(0)
        self.map_ox_spin.setFixedWidth(80)
        cl.addWidget(self.map_ox_spin)
        cl.addWidget(QLabel("Y"))
        self.map_oy_spin = QDoubleSpinBox()
        self.map_oy_spin.setRange(-5000, 5000)
        self.map_oy_spin.setDecimals(0)
        self.map_oy_spin.setFixedWidth(80)
        cl.addWidget(self.map_oy_spin)
        cl.addWidget(QLabel("旋转"))
        self.map_theta_spin = QDoubleSpinBox()
        self.map_theta_spin.setRange(-360, 360)
        self.map_theta_spin.setSuffix("°")
        self.map_theta_spin.setFixedWidth(84)
        cl.addWidget(self.map_theta_spin)
        apply_map = QPushButton("应用映射")
        apply_map.clicked.connect(self._apply_map_mapping)
        cl.addWidget(apply_map)

        cl.addSpacing(12)
        cl.addWidget(QLabel("目标航向"))
        self.map_yaw_combo = QComboBox()
        self.map_yaw_combo.addItem("保持当前", None)
        for deg in (0, 90, 180, 270):
            self.map_yaw_combo.addItem("场地 %d°" % deg, float(deg))
        cl.addWidget(self.map_yaw_combo)
        self.map_click_check = QCheckBox("点击下发")
        self.map_click_check.setChecked(True)
        self.map_click_check.toggled.connect(lambda x: setattr(self.map_view, "click_enabled", x))
        cl.addWidget(self.map_click_check)
        cl.addStretch(1)
        stop = QPushButton("■ STOP")
        stop.setObjectName("EmergencyButton")
        stop.clicked.connect(lambda: self.send_line("STOP"))
        cl.addWidget(stop)
        lay.addWidget(controls)

        quick = QFrame()
        quick.setObjectName("Panel")
        ql = QHBoxLayout(quick)
        ql.setContentsMargins(12, 8, 12, 8)
        ql.addWidget(QLabel("快速前往"))
        for name, (fx, fy) in core.QUICK_TARGETS:
            b = QPushButton(name)
            b.clicked.connect(lambda _=False, x=fx, y=fy: self._goto_field(x, y))
            ql.addWidget(b)
        home = QPushButton("回启停区")
        home.clicked.connect(self._goto_home)
        ql.addWidget(home)
        ql.addStretch(1)
        self.map_status = QLabel("点击地图 = 下发 GOTO；若路径穿越固定禁区将拒绝执行")
        self.map_status.setObjectName("HintLabel")
        ql.addWidget(self.map_status)
        lay.addWidget(quick)

        self.map_view = FieldView()
        self.map_view.gotoRequested.connect(self._goto_field)
        lay.addWidget(self.map_view, 1)
        return page

    def _build_chassis_page(self):
        page, lay = self._page_shell("底盘调参", "OPS 定位 P 系数、速度限幅与底盘安全控制")

        safe = QFrame()
        safe.setObjectName("Panel")
        sl = QHBoxLayout(safe)
        sl.setContentsMargins(12, 10, 12, 10)
        stop = QPushButton("■ 底盘急停 STOP")
        stop.setObjectName("EmergencyButton")
        stop.clicked.connect(lambda: self.send_line("STOP"))
        zero = QPushButton("⊙ OPS 置零 ZERO")
        zero.setObjectName("WarningButton")
        zero.clicked.connect(lambda: self.send_line("ZERO"))
        sendall = QPushButton("一键下发全部参数")
        sendall.setObjectName("PrimaryButton")
        sendall.clicked.connect(self._send_all_chassis)
        sl.addWidget(stop)
        sl.addWidget(zero)
        sl.addStretch(1)
        sl.addWidget(sendall)
        lay.addWidget(safe)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        body = QWidget()
        bl = QVBoxLayout(body)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(7)
        self.chassis_rows = {}
        for cmd, label, lo, hi, dflt, rb in core.CHASSIS_PARAMS:
            row = ParamRow(cmd, label, lo, hi, dflt, rb, False)
            row.sendRequested.connect(lambda c, v: self.send_line("%s=%s" % (c, v)))
            self.chassis_rows[cmd] = row
            bl.addWidget(row)
        bl.addStretch(1)
        scroll.setWidget(body)
        lay.addWidget(scroll, 1)
        return page

    def _build_dm_page(self):
        page, lay = self._page_shell("DM 电机", "MIT / 位置速度模式，实时反馈与故障状态")

        controls = QFrame()
        controls.setObjectName("Panel")
        cl = QHBoxLayout(controls)
        cl.setContentsMargins(12, 10, 12, 10)
        en = QPushButton("▶ 使能 DMEN")
        en.setObjectName("SuccessButton")
        en.clicked.connect(lambda: self.send_line("DMEN"))
        off = QPushButton("■ 失能 DMOFF")
        off.setObjectName("EmergencyButton")
        off.clicked.connect(lambda: self.send_line("DMOFF"))
        zero = QPushButton("⊙ 零点 DMZERO")
        zero.setObjectName("WarningButton")
        zero.clicked.connect(lambda: self.send_line("DMZERO"))
        cl.addWidget(en)
        cl.addWidget(off)
        cl.addWidget(zero)
        cl.addSpacing(18)
        cl.addWidget(QLabel("控制模式"))
        self.dm_mode_combo = QComboBox()
        self.dm_mode_combo.addItem("MIT 模式", 1)
        self.dm_mode_combo.addItem("位置速度模式", 2)
        self.dm_mode_combo.currentIndexChanged.connect(self._dm_mode_changed)
        cl.addWidget(self.dm_mode_combo)
        self.dm_mode_note = QLabel("Kp/Kd/Torque 参与力矩合成")
        self.dm_mode_note.setObjectName("HintLabel")
        cl.addWidget(self.dm_mode_note)
        cl.addStretch(1)
        lay.addWidget(controls)

        fb = QGridLayout()
        fb.setSpacing(10)
        self.dm_cards = {
            "pos": StatCard("实际位置", "—", "rad"),
            "vel": StatCard("实际速度", "—", "rad/s"),
            "tor": StatCard("实际力矩", "—", "Nm"),
            "tmos": StatCard("MOS 温度", "—", "°C"),
            "trotor": StatCard("线圈温度", "—", "°C"),
            "id": StatCard("电机 ID", "—", ""),
        }
        for i, card in enumerate(self.dm_cards.values()):
            fb.addWidget(card, 0, i)
        lay.addLayout(fb)

        self.dm_status_big = QLabel("DM 状态：—")
        self.dm_status_big.setObjectName("DmStatus")
        lay.addWidget(self.dm_status_big)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        body = QWidget()
        bl = QVBoxLayout(body)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(7)
        self.dm_rows = {}
        for cmd, label, lo, hi, dflt, rb, is_int in core.DM_PARAMS:
            row = ParamRow(cmd, label, lo, hi, dflt, rb, is_int)
            row.sendRequested.connect(lambda c, v: self.send_line("%s=%s" % (c, v)))
            self.dm_rows[cmd] = row
            bl.addWidget(row)
        bl.addStretch(1)
        scroll.setWidget(body)
        lay.addWidget(scroll, 1)
        return page

    def _build_record_page(self):
        page, lay = self._page_shell("数据记录", "保存 24 通道遥测为 CSV，或导出当前内存波形")

        panel = QFrame()
        panel.setObjectName("Panel")
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(18, 18, 18, 18)
        pl.setSpacing(12)
        self.record_state = QLabel("未记录")
        self.record_state.setObjectName("RecordState")
        self.record_rows = QLabel("记录行数：0")
        self.record_path = QLabel("文件：—")
        self.record_path.setObjectName("PathLabel")
        self.record_path.setWordWrap(True)
        pl.addWidget(self.record_state)
        pl.addWidget(self.record_rows)
        pl.addWidget(self.record_path)
        row = QHBoxLayout()
        self.record_page_btn = QPushButton("● 开始记录")
        self.record_page_btn.setObjectName("RecordButton")
        self.record_page_btn.clicked.connect(self.toggle_record)
        openf = QPushButton("打开记录文件夹")
        openf.clicked.connect(self._open_record_folder)
        export = QPushButton("导出当前缓冲…")
        export.clicked.connect(self._export_buffer)
        row.addWidget(self.record_page_btn)
        row.addWidget(openf)
        row.addWidget(export)
        row.addStretch(1)
        pl.addLayout(row)
        lay.addWidget(panel)

        info = QFrame()
        info.setObjectName("Panel")
        il = QVBoxLayout(info)
        il.setContentsMargins(18, 18, 18, 18)
        il.addWidget(QLabel("CSV 字段"))
        cols = "t_s, wallclock, " + ", ".join(c[1] for c in core.CHANNELS)
        desc = QLabel(cols)
        desc.setWordWrap(True)
        desc.setObjectName("HintLabel")
        il.addWidget(desc)
        il.addStretch(1)
        lay.addWidget(info, 1)
        return page

    def _build_console_page(self):
        page, lay = self._page_shell("命令终端", "直接发送固件 ASCII 调试命令，查看上位机运行日志")
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setObjectName("Console")
        lay.addWidget(self.console, 1)

        row = QHBoxLayout()
        self.command_entry = QLineEdit()
        self.command_entry.setPlaceholderText("例如：KPX=3.0 / GOTO=1000,600,90 / DMEN / STOP")
        self.command_entry.returnPressed.connect(self._send_console)
        send = QPushButton("发送")
        send.setObjectName("PrimaryButton")
        send.clicked.connect(self._send_console)
        row.addWidget(self.command_entry, 1)
        row.addWidget(send)
        lay.addLayout(row)
        self.log("Qt v2 UI 已启动。", "info")
        return page

    def _load_style(self):
        path = BASE_DIR / "style.qss"
        if path.exists():
            self.setStyleSheet(path.read_text(encoding="utf-8"))

    def _select_page(self, idx: int):
        if hasattr(self, "stack") and idx < self.stack.count():
            self.stack.setCurrentIndex(idx)
        for i, b in enumerate(self.nav_buttons):
            b.setProperty("active", i == idx)
            b.style().unpolish(b)
            b.style().polish(b)

    # ---------------- 串口/模拟 ----------------
    def refresh_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        if core.serial is None:
            self.port_combo.addItem("未安装 pyserial", None)
            return
        ports = list(core.serial.tools.list_ports.comports())
        for p in ports:
            label = p.device if p.description == p.device else "%s  ·  %s" % (p.device, p.description)
            self.port_combo.addItem(label, p.device)
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    @staticmethod
    def _drain_queue(q):
        while True:
            try:
                q.get_nowait()
            except queue.Empty:
                return

    def _clear_command_queues(self):
        self._drain_queue(self.line_q)
        self._drain_queue(self.urgent_q)

    def toggle_connect(self, force_on=None):
        want = force_on if force_on is not None else self.worker is None
        if want and self.worker is None:
            if core.serial is None:
                QMessageBox.warning(self, "缺少依赖", "未安装 pyserial。\n请执行：python -m pip install -r requirements.txt")
                return
            if self.sim is not None:
                self.toggle_sim(False)
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "串口", "请先选择有效串口。")
                return
            baud = int(self.baud_combo.currentText())
            self._clear_command_queues()
            self.worker = core.SerialWorker(
                port, baud, self.frame_q, self.line_q, self.urgent_q,
                err_cb=lambda msg: self.bridge.workerError.emit(msg),
            )
            self.worker.start()
            self.connect_btn.setText("断开")
            self._set_link_status("正在打开 %s…" % port, "warn")
            self.log("正在连接 %s @ %d 8N1" % (port, baud), "info")
        elif not want and self.worker is not None:
            old = self.worker
            self.worker = None
            try:
                old.request_stop(safe=True)
            except Exception:
                pass
            self._clear_command_queues()
            self.connect_btn.setText("连接")
            self._set_link_status("未连接", "idle")
            self.log("串口已请求安全断开（STOP/DMSTOP/DMOFF）", "info")

    def toggle_sim(self, force_on=None):
        want = force_on if force_on is not None else self.sim is None
        if want and self.sim is None:
            if self.worker is not None:
                self.toggle_connect(False)
            self._clear_command_queues()
            self.sim = core.Simulator(self.frame_q, self.line_q, self.urgent_q)
            self.sim.start()
            self.sim_btn.setText("停止模拟")
            self._set_link_status("模拟模式 · 50 Hz", "sim")
            self.log("模拟模式已开启。", "info")
        elif not want and self.sim is not None:
            self.sim.stop_flag = True
            self.sim = None
            self._clear_command_queues()
            self.sim_btn.setText("模拟")
            self._set_link_status("未连接", "idle")
            self.log("模拟模式已关闭。", "info")

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.setText("继续" if self.paused else "暂停")
        self.log("波形缓冲已%s" % ("暂停" if self.paused else "继续"), "info")

    def send_line(self, text: str):
        text = str(text).strip()
        if not text:
            return
        if self.worker is None and self.sim is None:
            self.log("未连接，命令未发送：%s" % text, "warn")
            return
        cmd = text.upper().split("=", 1)[0].strip()
        if cmd in core.URGENT_COMMANDS:
            self._drain_queue(self.line_q)
            self.urgent_q.put(text)
        else:
            self.line_q.put(text)
        self.send_count += 1
        self.log("TX> %s" % text, "tx")

    def _enqueue_heartbeat(self):
        now = time.monotonic()
        if (
            self.worker is not None
            and self.worker.opened.is_set()
            and now - self.last_heartbeat_enqueue >= core.HEARTBEAT_INTERVAL_S
        ):
            self.urgent_q.put("PING")
            self.last_heartbeat_enqueue = now

    def _on_worker_error(self, msg: str):
        self.log(msg, "warn")
        if self.worker is not None:
            old = self.worker
            self.worker = None
            try:
                old.request_stop(safe=False)
            except Exception:
                pass
        self.connect_btn.setText("连接")
        self._set_link_status("串口异常", "error")
        QMessageBox.critical(self, "串口错误", msg)

    # ---------------- 数据刷新 ----------------
    def _start_timers(self):
        self.data_timer = QTimer(self)
        self.data_timer.timeout.connect(self._process_frames)
        self.data_timer.start(20)

        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self._render_ui)
        self.render_timer.start(50)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._status_tick)
        self.status_timer.start(250)

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.timeout.connect(self._enqueue_heartbeat)
        self.heartbeat_timer.start(100)

    def _process_frames(self):
        got = 0
        try:
            while got < 200:
                t, vals = self.frame_q.get_nowait()
                got += 1
                tr = t - self.t0_monotonic
                self.latest = vals
                self.latest_t = tr
                if not self.paused:
                    self.ring.append(tr, vals)
                    self.traj_ring.append(tr, (vals[0], vals[1]))
                if self.recorder is not None:
                    self.recorder.write(t, vals)
        except queue.Empty:
            pass
        self.fps_count += got

    def _render_ui(self):
        v = self.latest
        if v is not None:
            self.cards["x"].set_value("%.0f" % v[0])
            self.cards["y"].set_value("%.0f" % v[1])
            self.cards["yaw"].set_value("%.1f" % v[2])
            self.cards["dm_pos"].set_value("%.3f" % v[13])
            self.cards["dm_vel"].set_value("%.3f" % v[14])
            self.cards["temp"].set_value("%.1f" % max(v[17], v[18]))

            self.dm_cards["pos"].set_value("%.3f" % v[13])
            self.dm_cards["vel"].set_value("%.3f" % v[14])
            self.dm_cards["tor"].set_value("%.3f" % v[15])
            self.dm_cards["tmos"].set_value("%.1f" % v[17])
            self.dm_cards["trotor"].set_value("%.1f" % v[18])
            self.dm_cards["id"].set_value("%d" % int(v[12]))

            for row in list(self.chassis_rows.values()) + list(self.dm_rows.values()):
                if row.readback_channel is not None:
                    row.set_readback(v[row.readback_channel])

            status = int(v[16]) & 0xFF
            text, fault = core.DM_STATUS.get(status, ("未知 0x%02X" % status, True))
            self.dm_status_big.setText("DM 状态：0x%02X  %s" % (status, text))
            self.dm_status_big.setProperty("fault", fault)
            self.dm_status_big.setProperty("enabled", status == 0x01)
            self.dm_status_big.style().unpolish(self.dm_status_big)
            self.dm_status_big.style().polish(self.dm_status_big)

            self.health_dm.setText("● DM：%s" % text)
            self.footer_dm.setText("DM[%d] %s" % (int(v[12]), text))

            fx, fy = self._ops_to_field(v[0], v[1])
            self.map_view.set_pose(fx, fy, self.map_theta + v[2])

        view = self.ring.view()
        self.wave_page.update_data(view, self.latest_t, self.window_s)
        self._update_dashboard_plots(view)
        self._update_map_trail()
        if self.map_target is None:
            self.map_view.set_target(None, None)
        else:
            self.map_view.set_target(*self.map_target)

        if self.recorder is not None:
            self.record_rows.setText("记录行数：%d" % self.recorder.rows)

    def _update_dashboard_plots(self, view):
        if view is None:
            return
        t, d = view
        if len(t) == 0:
            return
        start = np.searchsorted(t, self.latest_t - min(self.window_s, 20.0))
        t = t[start:]
        d = d[start:]
        self.dash_x.setData(t, d[:, 0])
        self.dash_y.setData(t, d[:, 1])
        self.dash_dm_pos.setData(t, d[:, 13])
        self.dash_dm_vel.setData(t, d[:, 14])
        if len(t):
            xmin = max(0.0, float(t[-1]) - min(self.window_s, 20.0))
            xmax = float(t[-1]) + 0.02
            self.dash_ops_plot.setXRange(xmin, xmax, padding=0)
            self.dash_dm_plot.setXRange(xmin, xmax, padding=0)

    def _update_map_trail(self):
        view = self.traj_ring.view()
        if view is None:
            return
        _t, d = view
        if len(d) == 0:
            return
        stride = max(1, len(d) // 700)
        x = d[::stride, 0]
        y = d[::stride, 1]
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        fx = self.map_ox + c * x - s * y
        fy = self.map_oy + s * x + c * y
        self.map_view.set_trail(fx, fy)

    def _status_tick(self):
        now = time.monotonic()
        dt = now - self.fps_t
        if dt >= 1.0:
            self.fps = self.fps_count / dt
            self.fps_count = 0
            self.fps_t = now
        self.footer_fps.setText("%.0f fps" % self.fps)

        if self.sim is not None:
            self._set_link_status("模拟模式 · 50 Hz", "sim")
            self.footer_link.setText("Telemetry SIM")
            self.footer_rx.setText("SIM 50 Hz")
            self.footer_err.setText("Err 0")
            self.health_link.setText("● 通信链路：模拟模式")
            self.health_telemetry.setText("● 遥测：50 Hz 仿真")
        elif self.worker is not None:
            w = self.worker
            if not w.opened.is_set():
                self._set_link_status("正在打开串口…", "warn")
                self.health_link.setText("● 通信链路：正在打开")
            else:
                last = w.last_frame_monotonic
                if last <= 0:
                    self._set_link_status("串口已打开 · 等待遥测", "warn")
                    self.health_telemetry.setText("● 遥测：等待数据")
                else:
                    age = now - last
                    if age <= core.TELEMETRY_WARN_S:
                        self._set_link_status("在线 · %.0f Hz" % self.fps, "ok")
                        self.health_telemetry.setText("● 遥测：正常 · %.0f ms" % (age * 1000))
                    elif age <= core.TELEMETRY_TIMEOUT_S:
                        self._set_link_status("遥测延迟 · %.0f ms" % (age * 1000), "warn")
                        self.health_telemetry.setText("● 遥测：延迟")
                    else:
                        self._set_link_status("遥测超时 · %.1f s" % age, "error")
                        self.health_telemetry.setText("● 遥测：超时")
                self.health_link.setText("● 通信链路：串口已打开")
            self.footer_link.setText("Telemetry UART")
            self.footer_rx.setText("RX %.1f KB" % (w.parser.bytes_in / 1024.0))
            self.footer_err.setText("Err %d" % w.parser.err_bytes)
        else:
            self._set_link_status("未连接", "idle")
            self.footer_link.setText("Telemetry —")
            self.footer_rx.setText("RX 0 B")
            self.footer_err.setText("Err 0")
            self.health_link.setText("● 通信链路：未连接")
            self.health_telemetry.setText("● 遥测：—")

    def _set_link_status(self, text: str, state: str):
        self.status_pill.setText("● " + text)
        self.status_pill.setProperty("state", state)
        self.status_pill.style().unpolish(self.status_pill)
        self.status_pill.style().polish(self.status_pill)

    # ---------------- 波形/记录 ----------------
    def _set_window(self, seconds: float):
        self.window_s = float(seconds)
        cap = int(self.window_s * core.SEND_HZ) + 100
        self.ring.resize(cap)
        self.traj_ring.resize(cap)

    def clear_data(self):
        self.ring.clear()
        self.traj_ring.clear()
        self.map_target = None
        self.map_view.set_target(None, None)
        self.map_view.set_trail(np.array([]), np.array([]))
        self.log("数据缓冲已清除。", "info")

    def toggle_record(self):
        if self.recorder is None:
            folder = BASE_DIR / "records"
            self.recorder = core.CsvRecorder(str(folder))
            self.record_top_btn.setText("■ 停止记录")
            self.record_page_btn.setText("■ 停止记录")
            self.record_state.setText("● 记录中")
            self.record_path.setText("文件：" + self.recorder.path)
            self.health_record.setText("● 记录：进行中")
            self.log("开始记录 -> %s" % self.recorder.path, "info")
        else:
            rows, path = self.recorder.rows, self.recorder.path
            self.recorder.close()
            self.recorder = None
            self.record_top_btn.setText("● 记录")
            self.record_page_btn.setText("● 开始记录")
            self.record_state.setText("未记录")
            self.health_record.setText("● 记录：未记录")
            self.log("停止记录，共 %d 行 -> %s" % (rows, path), "info")

    def _open_record_folder(self):
        folder = BASE_DIR / "records"
        folder.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))

    def _export_buffer(self):
        view = self.ring.view()
        if view is None:
            QMessageBox.information(self, "导出", "当前波形缓冲为空。")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出当前缓冲", "ILHC_buffer.csv", "CSV (*.csv)")
        if not path:
            return
        t, data = view
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["t_s"] + ["ch_%s" % c[1] for c in core.CHANNELS])
            for i in range(len(t)):
                w.writerow(["%.3f" % t[i]] + ["%.5g" % v if math.isfinite(v) else "" for v in data[i]])
        self.log("已导出 %d 行 -> %s" % (len(t), path), "info")

    # ---------------- 参数 ----------------
    def _send_all_chassis(self):
        for row in self.chassis_rows.values():
            self.send_line(row.command_text())

    def _dm_mode_changed(self):
        mode = int(self.dm_mode_combo.currentData())
        if mode == 1:
            self.dm_mode_note.setText("Kp/Kd/Torque 参与力矩合成")
        else:
            self.dm_mode_note.setText("位置速度模式：DMKP / DMKD / DMTOR 被电机忽略")
        self.send_line("DMMODE=%d" % mode)

    # ---------------- 地图 ----------------
    def _apply_map_mapping(self):
        self.map_ox = float(self.map_ox_spin.value())
        self.map_oy = float(self.map_oy_spin.value())
        self.map_theta = float(self.map_theta_spin.value())
        self.log("场地映射：原点(%.0f, %.0f)，旋转 %.1f°" % (self.map_ox, self.map_oy, self.map_theta), "info")

    def _ops_to_field(self, x: float, y: float):
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        return self.map_ox + c * x - s * y, self.map_oy + s * x + c * y

    def _field_to_ops(self, fx: float, fy: float):
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        dx, dy = fx - self.map_ox, fy - self.map_oy
        return c * dx + s * dy, -s * dx + c * dy

    def _set_start_zone(self):
        zone = int(self.zone_combo.currentData())
        zx, zy = core.ZONE_CENTER[zone]
        self.map_ox_spin.setValue(zx)
        self.map_oy_spin.setValue(zy)
        self._apply_map_mapping()
        self.send_line("ZERO")
        self.traj_ring.clear()
        self.map_target = None
        self.map_status.setText("启停区%d 已设为 OPS 原点，并发送 ZERO" % zone)
        self.log("启停区%d -> 原点(%.0f, %.0f)，已发送 ZERO" % (zone, zx, zy), "info")

    def _target_yaw_ops(self):
        field_yaw = self.map_yaw_combo.currentData()
        if field_yaw is None:
            return float(self.latest[2]) if self.latest is not None else 0.0
        return float(field_yaw) - self.map_theta

    def _goto_field(self, fx: float, fy: float):
        blocked = core.field_point_blocked(fx, fy)
        if blocked:
            self.map_status.setText("拒绝 GOTO：目标位于【%s】" % blocked)
            self.log("GOTO 拒绝：目标(%.0f, %.0f) 位于%s" % (fx, fy, blocked), "warn")
            return

        if self.latest is not None and math.isfinite(self.latest[0]) and math.isfinite(self.latest[1]):
            sx, sy = self._ops_to_field(self.latest[0], self.latest[1])
            blocked = core.field_path_blocked(sx, sy, fx, fy)
            if blocked:
                self.map_status.setText("拒绝直线 GOTO：路径穿越【%s】，请先选择中间安全点" % blocked)
                self.log("GOTO 拒绝：直线路径穿越%s" % blocked, "warn")
                return

        tx, ty = self._field_to_ops(fx, fy)
        yaw = self._target_yaw_ops()
        self.map_target = (fx, fy)
        self.send_line("GOTO=%.0f,%.0f,%.0f" % (tx, ty, yaw))
        self.map_status.setText("目标：场地(%.0f, %.0f) → OPS GOTO=%.0f,%.0f,%.0f" % (fx, fy, tx, ty, yaw))

    def _goto_home(self):
        zone = int(self.zone_combo.currentData())
        zx, zy = core.ZONE_CENTER[zone]
        home_yaw = (270.0 if zone == 1 else 90.0) - self.map_theta
        tx, ty = self._field_to_ops(zx, zy)
        self.map_target = (zx, zy)
        self.send_line("GOTO=%.0f,%.0f,%.0f" % (tx, ty, home_yaw))
        self.map_status.setText("返回启停区%d" % zone)

    # ---------------- 控制台 ----------------
    def _send_console(self):
        text = self.command_entry.text().strip()
        if not text:
            return
        self.send_line(text)
        self.cmd_history.append(text)
        self.hist_idx = len(self.cmd_history)
        self.command_entry.clear()

    def log(self, text: str, tag: str = "info"):
        if not hasattr(self, "console"):
            return
        color = {"tx": GREEN, "warn": RED, "info": FG_DIM}.get(tag, FG_DIM)
        stamp = time.strftime("%H:%M:%S")
        safe = html.escape(str(text))
        self.console.append('<span style="color:%s">[%s] %s</span>' % (color, stamp, safe))

    def closeEvent(self, event: QCloseEvent):
        try:
            if self.recorder is not None:
                self.recorder.close()
                self.recorder = None
            if self.worker is not None:
                self.worker.request_stop(safe=True)
                self.worker = None
            if self.sim is not None:
                self.sim.stop_flag = True
                self.sim = None
        finally:
            event.accept()


def parse_args():
    ap = argparse.ArgumentParser(description="ILHC Control Station v2.0")
    ap.add_argument("--port", help="启动时连接指定串口，如 COM6")
    ap.add_argument("--baud", type=int, default=core.DEFAULT_BAUD)
    ap.add_argument("--simulate", action="store_true", help="启动即进入模拟模式")
    ap.add_argument("--selftest", action="store_true", help="运行 core.py 无 GUI 自检")
    return ap.parse_args()


def main():
    args = parse_args()
    if args.selftest:
        core.selftest()
        return 0
    app = QApplication(sys.argv)
    app.setApplicationName("ILHC Control Station")
    app.setOrganizationName("ILHC")
    w = MainWindow(args)
    w.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
