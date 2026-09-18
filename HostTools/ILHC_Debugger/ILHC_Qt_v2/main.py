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
import json
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
    from PySide6.QtCore import QEvent, QObject, QPointF, QRectF, Qt, QTimer, QUrl, Signal
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
        QMenu,
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
        # 仅旋转显示：Qt屏幕Y向下，正90°即顺时针；点击由mapToScene逆变换。
        self.rotate(90)
        self._build_field()

    @classmethod
    def sy(cls, field_y: float) -> float:
        return cls.FIELD - field_y

    def _add_centered_text(self, text: str, fx: float, fy: float, color: str, size=10, bold=False):
        item = QGraphicsSimpleTextItem(text)
        # 地图约2400场景单位，缩放后仍需保持文字可读。
        font = QFont("Microsoft YaHei UI", round(size * 3.3))
        font.setBold(bold)
        item.setFont(font)
        item.setBrush(QColor(color))
        item.setZValue(20)
        self.scene_obj.addItem(item)
        br = item.boundingRect()
        # 抵消视图旋转，地图文字保持水平，位置仍随地图旋转。
        item.setTransformOriginPoint(br.center())
        item.setRotation(-90)
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

        # 用户坐标以启停区1中心为原点，绘图内部仍使用左下角坐标。
        axis_pen = QPen(QColor("#C0392B"), 4)
        self.scene_obj.addLine(2250, self.sy(2250), 1800, self.sy(2250), axis_pen)
        self.scene_obj.addLine(1800, self.sy(2250), 1850, self.sy(2280), axis_pen)
        self.scene_obj.addLine(1800, self.sy(2250), 1850, self.sy(2220), axis_pen)
        self.scene_obj.addLine(2250, self.sy(2250), 2250, self.sy(1800), axis_pen)
        self.scene_obj.addLine(2250, self.sy(1800), 2220, self.sy(1850), axis_pen)
        self.scene_obj.addLine(2250, self.sy(1800), 2280, self.sy(1850), axis_pen)
        self._add_centered_text("+Y 上", 1770, 2310, "#C0392B", 10, True)
        self._add_centered_text("+X 左", 2310, 1750, "#C0392B", 10, True)
        self._add_centered_text("(0.0, 0.0)", 2350, 2250, "#FFFFFF", 9)
        self._add_centered_text("(210.0, 0.0)", 2350, 150, "#FFFFFF", 9)
        for value in range(0, 2401, 600):
            label = "%.1f" % ((2250 - value) / core.OPS_CM_TO_MM)
            self._add_centered_text(label, value, -80, "#8B95A3", 8)
            self._add_centered_text(label, -100, value, "#8B95A3", 8)

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


class DetachedPageWindow(QMainWindow):
    """把一个调参页拆成独立顶层窗口，并提供窗口置顶开关。

    页面控件仍是主窗口里的同一个对象（信号、定时器、键盘遥控都不重建），
    只是父窗口改成这里；关闭窗口时通过 reattachRequested 交还主窗口。

    注意：本窗口刻意不带 Qt 父级。带父级的顶层窗口在 Windows 上是 owned window，
    会永久压在主窗口上面无法换层；代价是要自己复制一份主窗口样式表。
    """

    reattachRequested = Signal(int)
    topmostToggled = Signal(int, bool)

    def __init__(self, index: int, name: str, parent=None):
        super().__init__(parent, Qt.Window)
        self.page_index = index
        self.page_name = name
        self._page = None
        self.setWindowTitle("%s · ILHC 独立窗口" % name)
        self.setMinimumSize(520, 360)

        root = QWidget()
        self.setCentralWidget(root)
        self.root_lay = QVBoxLayout(root)
        self.root_lay.setContentsMargins(0, 0, 0, 0)
        self.root_lay.setSpacing(0)

        bar = QFrame()
        bar.setObjectName("DetachBar")
        bl = QHBoxLayout(bar)
        bl.setContentsMargins(12, 6, 8, 6)
        bl.setSpacing(8)
        title = QLabel(name)
        title.setObjectName("DetachTitle")
        bl.addWidget(title)
        bl.addStretch(1)

        # 标题栏按钮全部 NoFocus：点击置顶/取回不会把焦点从键盘遥控区抢走。
        self.top_btn = QPushButton("置顶")
        self.top_btn.setCheckable(True)
        self.top_btn.setFocusPolicy(Qt.NoFocus)
        self.top_btn.setToolTip("让该窗口保持在其他窗口之上")
        self.top_btn.toggled.connect(self._on_top_toggled)
        bl.addWidget(self.top_btn)

        back = QPushButton("取回主窗口")
        back.setFocusPolicy(Qt.NoFocus)
        back.setToolTip("把页面放回主窗口并关闭本窗口")
        back.clicked.connect(lambda: self.reattachRequested.emit(self.page_index))
        bl.addWidget(back)
        self.root_lay.addWidget(bar)

    def take_page(self, page: QWidget):
        self._page = page
        self.root_lay.addWidget(page, 1)
        # setParent 会把控件置为隐藏，换父窗口后必须显式 show。
        page.show()

    def page(self) -> QWidget | None:
        return self._page

    def is_topmost(self) -> bool:
        return bool(self.windowFlags() & Qt.WindowStaysOnTopHint)

    def set_topmost(self, on: bool):
        """程序设置置顶（不触发 topmostToggled，避免重复日志）。"""
        on = bool(on)
        self.top_btn.blockSignals(True)
        self.top_btn.setChecked(on)
        self.top_btn.blockSignals(False)
        self._apply_topmost(on)

    def _on_top_toggled(self, on: bool):
        self._apply_topmost(bool(on))
        self.topmostToggled.emit(self.page_index, bool(on))

    def _apply_topmost(self, on: bool):
        on = bool(on)
        self.top_btn.setText("已置顶" if on else "置顶")
        if self.is_topmost() == on:
            return
        # setWindowFlag 会重建原生窗口并把它隐藏，所以必须先记下可见性和焦点，
        # 改完标志后再恢复；否则取消置顶后窗口会直接“消失”。
        was_visible = self.isVisible()
        focus = QApplication.focusWidget()
        focus_inside = focus is not None and self.isAncestorOf(focus)
        self.setWindowFlag(Qt.WindowStaysOnTopHint, on)
        if was_visible:
            self.show()
            self.raise_()
        if focus_inside and focus is not None:
            focus.setFocus(Qt.OtherFocusReason)

    def closeEvent(self, event: QCloseEvent):
        self.reattachRequested.emit(self.page_index)
        super().closeEvent(event)


class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setWindowTitle("ILHC Control Station v2.0 — STM32F407VET6")
        self.resize(1500, 930)
        self.setMinimumSize(1180, 760)

        # 后端状态
        self.frame_q = queue.Queue(maxsize=4000)
        self.line_q = core.CommandQueue()
        self.urgent_q = queue.Queue()
        # 固件文字应答/错误与上位机提示（如补发 VOFA）都显示到日志，
        # 否则固件在文字模式下报的 CAN 失败等信息会被解析器静默丢弃。
        self.fw_text_q = queue.Queue(maxsize=200)
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
        # 四轮锁轴状态：None未请求 / True已请求使能 / False已请求失能。
        # 固件无状态回读，这里只记录本机发出的最后一条请求。
        self.wheel_state = None
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
        self.page_names = [
            "总览",
            "实时波形",
            "比赛地图",
            "底盘调参",
            "DM 电机",
            "数据记录",
            "命令终端",
            "28 / 35 步进",
        ]
        # 每个页面固定占一个槽位（QStackedWidget）；页面被拆到独立窗口时，
        # 槽位里换成占位卡，槽位下标永远等于页面下标。
        self.slots = []
        self.placeholders = []
        self.placeholder_top_checks = {}
        self.detached = {}
        self.page_windows = {}
        self.topmost_pref = {}
        self._reattaching = set()

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
            self._build_stepper_page(),
        ]
        for i, page in enumerate(self.pages):
            slot = QStackedWidget()
            placeholder = self._build_detach_placeholder(i)
            slot.addWidget(placeholder)
            slot.addWidget(page)
            slot.setCurrentWidget(page)
            self.slots.append(slot)
            self.placeholders.append(placeholder)
            self.stack.addWidget(slot)
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

        stop = QPushButton("■  底盘 / DM 停止")
        stop.setToolTip("停止底盘和DM，撤销待发28/35请求；不能停止已执行的28/35运动")
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

        self.nav_buttons = []
        for i, name in enumerate(self.page_names):
            btn = QPushButton(name)
            btn.setProperty("nav", True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setToolTip("左键切换页面；右键可拆成独立窗口、设置窗口置顶")
            btn.setContextMenuPolicy(Qt.CustomContextMenu)
            btn.customContextMenuRequested.connect(
                lambda pos, x=i: self._show_nav_menu(x, pos))
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
        hl = QHBoxLayout(head)
        hl.setContentsMargins(2, 0, 2, 4)
        hl.setSpacing(10)
        text = QVBoxLayout()
        text.setSpacing(2)
        t = QLabel(title)
        t.setObjectName("PageTitle")
        s = QLabel(subtitle)
        s.setObjectName("PageSubtitle")
        text.addWidget(t)
        text.addWidget(s)
        hl.addLayout(text)
        hl.addStretch(1)
        detach = QPushButton("独立窗口")
        detach.setToolTip("把该页面拆成可移动的独立窗口；独立窗口中可勾选「置顶」")
        detach.setCursor(Qt.PointingHandCursor)
        # 构建时 self.pages 尚未填好，点击时再解析下标。
        detach.clicked.connect(lambda _=False, w=page: self._detach_page(self._page_index_of(w)))
        hl.addWidget(detach)
        lay.addWidget(head)
        return page, lay

    def _build_dashboard_page(self):
        page, lay = self._page_shell("系统总览", "实时查看底盘定位、DM 电机和通信链路状态")

        cards = QGridLayout()
        cards.setSpacing(10)
        self.cards = {
            "x": StatCard("OPS X 左右", "—", "cm"),
            "y": StatCard("OPS Y 前后", "—", "cm"),
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
        self.dash_ops_plot.setLabel("left", "", units="cm")
        self.dash_x = self.dash_ops_plot.plot(pen=pg.mkPen(core.COLORS[0], width=1.7), name="X 左右")
        self.dash_y = self.dash_ops_plot.plot(pen=pg.mkPen(core.COLORS[1], width=1.7), name="Y 前后")
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
        page, lay = self._page_shell("比赛场地", "启停区1中心 (0.0,0.0)；向左 +X，向上 +Y，单位 cm；航向0°向左、90°向上")

        controls = QFrame()
        controls.setObjectName("Panel")
        cl = QHBoxLayout(controls)
        cl.setContentsMargins(12, 10, 12, 10)
        cl.setSpacing(8)

        cl.addWidget(QLabel("启停区"))
        self.zone_combo = QComboBox()
        self.zone_combo.addItem("启停区1（右下）", 1)
        self.zone_combo.addItem("启停区2（左下）", 2)
        cl.addWidget(self.zone_combo)
        origin = QPushButton("在所选区校准 OPS 零点")
        origin.setObjectName("WarningButton")
        origin.clicked.connect(self._set_start_zone)
        cl.addWidget(origin)

        cl.addSpacing(12)
        cl.addWidget(QLabel("OPS零点 X"))
        self.map_ox_spin = QDoubleSpinBox()
        self.map_ox_spin.setRange(-500.0, 500.0)
        self.map_ox_spin.setDecimals(1)
        self.map_ox_spin.setSuffix(" cm")
        self.map_ox_spin.setFixedWidth(96)
        cl.addWidget(self.map_ox_spin)
        cl.addWidget(QLabel("Y"))
        self.map_oy_spin = QDoubleSpinBox()
        self.map_oy_spin.setRange(-500.0, 500.0)
        self.map_oy_spin.setDecimals(1)
        self.map_oy_spin.setSuffix(" cm")
        self.map_oy_spin.setFixedWidth(96)
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
        self.map_position = QLabel("场地位置：等待遥测（启停区1中心为0点）")
        lay.addWidget(self.map_position)
        lay.addWidget(self.map_view, 1)
        return page

    def _build_stepper_page(self):
        page, lay = self._page_shell("28 / 35 步进电机", "CAN 绝对位置与回零调试；参数编辑后点击按钮才下发")
        note = QLabel("机械换算沿用原车标定。35：高度43–203 mm；28：半径120–286 mm（不是伸出量）。\n"
                      "STOP 仅停止底盘/DM；28/35 尚无已验证停机协议。取消待发不会停止已启动运动。\n"
                      "当前24通道不含28/35反馈：命令入队不代表CAN发送成功、回零完成或到位。")
        note.setWordWrap(True)
        lay.addWidget(note)
        self.stepper_widgets = {}
        for motor, (title, label, lo, hi, default, vlo, vhi, can_id) in core.STEPPER_CONFIG.items():
            panel = QFrame()
            panel.setObjectName("Panel")
            grid = QGridLayout(panel)
            grid.addWidget(QLabel("%s · CAN 0x%03X" % (title, can_id)), 0, 0, 1, 4)
            position = QDoubleSpinBox()
            position.setRange(lo, hi)
            position.setDecimals(1)
            position.setSuffix(" mm")
            position.setValue(default)
            speed = QSpinBox()
            speed.setRange(vlo, vhi)
            speed.setSuffix(" mm/s")
            speed.setValue(10)
            grid.addWidget(QLabel(label), 1, 0)
            grid.addWidget(position, 1, 1)
            grid.addWidget(QLabel("线速度（协议范围）"), 1, 2)
            grid.addWidget(speed, 1, 3)
            move = QPushButton("执行机械目标")
            move.clicked.connect(lambda _=False, m=motor: self._send_stepper(m, False))
            grid.addWidget(move, 1, 4)
            direction = QComboBox()
            direction.addItems(["方向 0", "方向 1"])
            steps = QDoubleSpinBox()
            steps.setDecimals(0)
            steps.setRange(0, 4294967295)
            rpm = QSpinBox()
            rpm.setRange(1, 65535)
            rpm.setValue(10)
            rpm.setSuffix(" RPM")
            grid.addWidget(direction, 2, 0)
            grid.addWidget(steps, 2, 1)
            grid.addWidget(QLabel("绝对位置计数 / 转速"), 2, 2)
            grid.addWidget(rpm, 2, 3)
            raw = QPushButton("执行原始绝对位置")
            raw.clicked.connect(lambda _=False, m=motor: self._send_stepper(m, True))
            grid.addWidget(raw, 2, 4)
            home = QPushButton("执行电机回零")
            home.clicked.connect(lambda _=False, m=motor: self.send_line("S%dHOME" % m))
            cancel = QPushButton("取消待发（不停车）")
            cancel.clicked.connect(lambda _=False, m=motor: self.send_line("S%dCANCEL" % m))
            grid.addWidget(home, 3, 0, 1, 2)
            grid.addWidget(cancel, 3, 3, 1, 2)
            self.stepper_widgets[motor] = (position, speed, direction, steps, rpm)
            lay.addWidget(panel)
        lay.addStretch(1)
        return page

    def _send_stepper(self, motor, raw):
        position, speed, direction, steps, rpm = self.stepper_widgets[motor]
        if raw:
            command = "S%dRAW=%d,%d,%d" % (motor, direction.currentIndex(), int(steps.value()), rpm.value())
        else:
            command = core.stepper_move_command(motor, position.value(), speed.value())
        self.send_line(command)

    def _build_chassis_page(self):
        page, lay = self._page_shell("底盘调参", "OPS 定位 P 系数、速度限幅与底盘安全控制")

        safe = QFrame()
        safe.setObjectName("Panel")
        sl = QVBoxLayout(safe)
        sl.setContentsMargins(12, 10, 12, 10)
        sl.setSpacing(8)
        top_row = QHBoxLayout()
        stop = QPushButton("■ 底盘急停 STOP")
        stop.setObjectName("EmergencyButton")
        stop.clicked.connect(lambda: self.send_line("STOP"))
        zero = QPushButton("⊙ OPS 置零 ZERO")
        zero.setObjectName("WarningButton")
        zero.clicked.connect(lambda: self.send_line("ZERO"))
        sendall = QPushButton("一键下发全部参数")
        sendall.setObjectName("PrimaryButton")
        sendall.clicked.connect(self._send_all_chassis)
        top_row.addWidget(stop)
        top_row.addWidget(zero)
        top_row.addStretch(1)
        top_row.addWidget(sendall)
        sl.addLayout(top_row)

        # 四轮锁轴：使能=电机保持位置；失能=轮子可自由推动。
        # 固件在失能期间丢弃 GOTO/MANUAL/ZDT，必须显式重新使能。
        wheel_row = QHBoxLayout()
        wheel_row.setSpacing(8)
        wheel_en = QPushButton("使能电机（锁轴）")
        wheel_en.setObjectName("SuccessButton")
        wheel_en.setToolTip("四轮统一使能并保持位置。固件上电时默认已使能，先停车再使能。")
        wheel_en.clicked.connect(lambda: self.send_line("WHEELEN"))
        wheel_off = QPushButton("失能电机（不锁轴）")
        wheel_off.setObjectName("WarningButton")
        wheel_off.setToolTip("四轮统一失能，轮子可自由推动。会先停车；失能期间地图 GOTO、\n"
                            "键盘遥控和 ZDT 单轮测试都会被固件拒绝，需重新使能。")
        wheel_off.clicked.connect(lambda: self.send_line("WHEELOFF"))
        self.wheel_status = QLabel("")
        self.wheel_status.setObjectName("HintLabel")
        self.wheel_status.setWordWrap(True)
        wheel_row.addWidget(wheel_en)
        wheel_row.addWidget(wheel_off)
        wheel_row.addWidget(self.wheel_status, 1)
        sl.addLayout(wheel_row)
        self._update_wheel_status()
        lay.addWidget(safe)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        body = QWidget()
        bl = QVBoxLayout(body)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(7)
        compensation = QFrame()
        compensation.setObjectName("Panel")
        cg = QGridLayout(compensation)
        cg.addWidget(QLabel("OPS 安装偏心补偿 · 单位 mm"), 0, 0, 1, 4)
        self.ops_offset_x = QDoubleSpinBox()
        self.ops_offset_y = QDoubleSpinBox()
        for spin, value in ((self.ops_offset_x, 60), (self.ops_offset_y, -50)):
            spin.setRange(-500, 500)
            spin.setDecimals(1)
            spin.setSingleStep(1)
            spin.setValue(value)
            spin.setSuffix(" mm")
        cg.addWidget(QLabel("X 左右偏移（左+ / 右−）"), 1, 0)
        cg.addWidget(self.ops_offset_x, 1, 1)
        cg.addWidget(QLabel("Y 前后偏移（前+ / 后−）"), 1, 2)
        cg.addWidget(self.ops_offset_y, 1, 3)
        for i, (label, handler) in enumerate((("应用补偿并置零", self._apply_ops_offset),
                                              ("保存到电脑", self._save_ops_offset),
                                              ("加载文件", self._load_ops_offset),
                                              ("填入初始值", self._reset_ops_offset))):
            button = QPushButton(label)
            button.clicked.connect(handler)
            cg.addWidget(button, 2, i)
        self.ops_offset_status = QLabel("默认：X=左60 / Y=后-50 mm。应用会停车并重新置零；调参只写RAM，断电恢复默认。")
        self.ops_offset_status.setWordWrap(True)
        cg.addWidget(self.ops_offset_status, 3, 0, 1, 4)
        self.ops_drift = QLabel("补偿后位置：等待遥测；置零后原地旋转，观察 X(左右)/Y(前后) 是否接近0。")
        self.ops_drift.setWordWrap(True)
        cg.addWidget(self.ops_drift, 4, 0, 1, 4)
        self.ops_offset_hint = QLabel("下发顺序与坐标定义一致：OPSOFFSET=X(左+),Y(前+)；左60/后50 对应 OPSOFFSET=60.0,-50.0。")
        self.ops_offset_hint.setWordWrap(True)
        cg.addWidget(self.ops_offset_hint, 5, 0, 1, 4)
        bl.addWidget(compensation)
        self.manual_vector = None
        self.manual_timer = QTimer(self)
        self.manual_timer.timeout.connect(self._manual_tick)
        self.manual_timer.setInterval(50)
        self.manual_timer.setTimerType(Qt.PreciseTimer)
        panel = QFrame()
        panel.setObjectName("Panel")
        grid = QGridLayout(panel)
        grid.addWidget(QLabel("键盘遥控 · WASD 平移 / Q E 转向（车体方向）"), 0, 0, 1, 4)
        self.manual_speed = QSpinBox()
        self.manual_speed.setRange(1, 300)
        self.manual_speed.setValue(60)
        self.manual_turn = QSpinBox()
        self.manual_turn.setRange(1, 300)
        self.manual_turn.setValue(30)
        grid.addWidget(QLabel("平移分量 RPM"), 1, 0)
        grid.addWidget(self.manual_speed, 1, 1)
        grid.addWidget(QLabel("旋转分量 RPM"), 1, 2)
        grid.addWidget(self.manual_turn, 1, 3)
        self.keyboard_enabled = False
        self.keyboard_keys = set()
        self.keyboard_button = QPushButton("进入键盘遥控")
        self.keyboard_button.setFocusPolicy(Qt.NoFocus)
        self.keyboard_button.clicked.connect(self._keyboard_toggle)
        grid.addWidget(self.keyboard_button, 2, 0, 1, 2)
        self.keyboard_exit = QPushButton("退出遥控 / 停车（Esc）")
        self.keyboard_exit.clicked.connect(self._manual_stop)
        grid.addWidget(self.keyboard_exit, 2, 2, 1, 2)
        self.keyboard_pad = QFrame()
        self.keyboard_pad.setFocusPolicy(Qt.StrongFocus)
        self.keyboard_pad.installEventFilter(self)
        key_grid = QGridLayout(self.keyboard_pad)
        self.keyboard_labels = {}
        for key, label, row, col in ((Qt.Key_Q, "Q 左转", 0, 0), (Qt.Key_W, "W 前进", 0, 1),
                                     (Qt.Key_E, "E 右转", 0, 2), (Qt.Key_A, "A 左移", 1, 0),
                                     (Qt.Key_S, "S 后退", 1, 1), (Qt.Key_D, "D 右移", 1, 2)):
            item = QLabel(label)
            item.setAlignment(Qt.AlignCenter)
            item.setMinimumHeight(38)
            item.setStyleSheet("background: #242A33; border: 1px solid #38434F; border-radius: 6px; padding: 6px;")
            self.keyboard_labels[key] = item
            key_grid.addWidget(item, row, col)
        key_grid.addWidget(QLabel("Shift：30%低速    空格：停车    Esc：退出遥控\n"
                                 "支持 W+A / W+Q 等组合；松键停车，点击其他控件或切出窗口自动退出。"), 2, 0, 1, 3)
        grid.addWidget(self.keyboard_pad, 3, 0, 1, 4)
        self.manual_status = QLabel("待机 · 点击进入遥控后使用键盘；不依赖 OPS")
        self.manual_status.setWordWrap(True)
        grid.addWidget(self.manual_status, 6, 0, 1, 4)
        grid.addWidget(QLabel("俯视，车头朝上：左前 1 ｜右前 2 ｜左后 3 ｜右后 4"), 7, 0, 1, 4)
        self.manual_invert = []
        for i, label in enumerate(("左右反向", "前后反向", "旋转反向")):
            check = QCheckBox(label)
            # 2026-09-16：固件已在协议边界统一做 180° 旋转（车头方向反了已修正），
            # 「按W后退」的根因消失，因此三个方向反向开关全部默认关闭；
            # 它们只作为键盘手动这一路的兜底手段保留。
            check.setChecked(False)
            check.toggled.connect(self._manual_stop)
            self.manual_invert.append(check)
            grid.addWidget(check, 8, i)
        bl.addWidget(panel)
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

    def _apply_ops_offset(self):
        if self.worker is None and self.sim is None:
            self.ops_offset_status.setText("未连接，补偿参数未发送。")
            return
        self._manual_stop()
        self.send_line("STOP")
        # OPSOFFSET 与界面控件、固件内部均为 X=车左、Y=车头，直接同序发送。
        command = core.ops_offset_command(self.ops_offset_x.value(), self.ops_offset_y.value())
        self.send_line(command)
        self.ops_offset_status.setText("已请求：%s；固件停车并置零。无参数回读，请通过旋转遥测验证。" % command)

    def _reset_ops_offset(self):
        self.ops_offset_x.setValue(60)
        self.ops_offset_y.setValue(-50)
        self.ops_offset_status.setText("已填入 X=左60 / Y=后-50 mm；点击应用才下发。")

    def _save_ops_offset(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存OPS补偿", "ops-offset.json", "JSON (*.json)")
        if not path:
            return
        try:
            # 文件键与统一坐标一致：x_mm=X 左右偏移、y_mm=Y 前后偏移。
            Path(path).write_text(json.dumps({"version": 2, "x_mm": self.ops_offset_x.value(),
                                             "y_mm": self.ops_offset_y.value()}, ensure_ascii=False, indent=2), encoding="utf-8")
            self.ops_offset_status.setText("已保存到电脑；不代表已写入单片机Flash。")
        except OSError as exc:
            self.ops_offset_status.setText("保存失败：%s" % exc)

    def _load_ops_offset(self):
        path, _ = QFileDialog.getOpenFileName(self, "加载OPS补偿", "", "JSON (*.json)")
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            version = int(data["version"])
            if version == 1:
                # v1 的 x_mm/y_mm 分别是旧“前后/左右”，只在加载时迁移一次。
                left_mm = float(data["y_mm"])
                forward_mm = float(data["x_mm"])
                migrated = True
            elif version == 2:
                left_mm = float(data["x_mm"])         # x_mm=X 左右偏移
                forward_mm = float(data["y_mm"])      # y_mm=Y 前后偏移
                migrated = False
            else:
                raise ValueError("不支持的参数文件版本")
            core.ops_offset_command(left_mm, forward_mm)
            self.ops_offset_x.setValue(left_mm)
            self.ops_offset_y.setValue(forward_mm)
            self.ops_offset_status.setText(
                ("已加载旧版参数并迁移到统一坐标；" if migrated else "已加载；") +
                "点击应用才下发，不会自动启动车辆。")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.ops_offset_status.setText("加载失败：%s" % exc)

    def _manual_start(self, vector, raw=False):
        """vector 与 MANUAL 完全同序：X=车左、Y=车头、W=逆时针。"""
        if self.worker is None and self.sim is None:
            self.log("请先连接串口或开启模拟", "warn")
            return
        if self.wheel_state is False:
            self.manual_status.setText("四轮已请求失能：固件会拒绝 MANUAL，请先「使能电机（锁轴）」")
            return
        if not raw:
            vector = (vector[0] * self.manual_speed.value(),
                      vector[1] * self.manual_speed.value(),
                      vector[2] * self.manual_turn.value())
        vector = tuple(-v if c.isChecked() else v for v, c in zip(vector, self.manual_invert))
        self.manual_vector = vector
        self._manual_tick()
        if not self.manual_timer.isActive():
            self.manual_timer.start()
        self.manual_status.setText("手动运行：左右 X %d / 前后 Y %d / 旋转 Z %d RPM · 松开停车" % vector)

    def _keyboard_toggle(self):
        if self.keyboard_enabled:
            self._manual_stop()
            return
        if self.worker is None and self.sim is None:
            self.manual_status.setText("请先连接串口或开启模拟")
            return
        if self.worker is not None and not self.worker.opened.is_set():
            self.manual_status.setText("串口尚未就绪")
            return
        if self.wheel_state is False:
            self.manual_status.setText("四轮已请求失能：固件会拒绝 MANUAL，请先「使能电机（锁轴）」")
            return
        self.send_line("STOP")  # 接管前取消原有GOTO，禁止旧目标继续运行。
        self.keyboard_enabled = True
        self.keyboard_keys.clear()
        self.keyboard_button.setText("键盘已接管")
        self.keyboard_pad.setFocus(Qt.OtherFocusReason)
        self.manual_status.setText("键盘已接管 · WASD平移，Q/E转向，Shift低速，空格停车")

    def _keyboard_refresh(self):
        keys = self.keyboard_keys
        for key, label in self.keyboard_labels.items():
            label.setStyleSheet("background: %s; border-radius: 6px; padding: 6px;" %
                               ("#19799B" if key in keys else "#242A33"))
        forward = int(Qt.Key_W in keys) - int(Qt.Key_S in keys)
        left = int(Qt.Key_A in keys) - int(Qt.Key_D in keys)
        turn = int(Qt.Key_Q in keys) - int(Qt.Key_E in keys)
        if not (forward or left or turn):
            if self.manual_vector is not None:
                self.manual_vector = None
                self.manual_timer.stop()
                self._drain_queue(self.line_q)
                self.urgent_q.put("MANUAL=0,0,0")
            self.manual_status.setText("键盘遥控待机 · 按住运行，松开停车")
            return
        scale = 0.3 if Qt.Key_Shift in keys else 1.0
        # 斜向平移归一化，避免两个轴同时按下时总速度增加sqrt(2)。
        speed = self.manual_speed.value() * scale / max(1.0, math.hypot(forward, left))
        # 元组与 MANUAL 同序：X=左右、Y=前后、Z=旋转。
        self._manual_start((round(left * speed), round(forward * speed),
                            round(turn * self.manual_turn.value() * scale)), True)

    def eventFilter(self, watched, event):
        if watched is getattr(self, "keyboard_pad", None):
            if event.type() == QEvent.FocusOut:
                self._manual_stop()
            if getattr(self, "keyboard_enabled", False):
                if event.type() in (QEvent.KeyPress, QEvent.KeyRelease, QEvent.ShortcutOverride):
                    key = event.key()
                    if key in (Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D, Qt.Key_Q, Qt.Key_E,
                               Qt.Key_Shift, Qt.Key_Space, Qt.Key_Escape):
                        if event.type() == QEvent.ShortcutOverride:
                            event.accept()
                            return True
                        if event.isAutoRepeat():
                            return True
                        if key == Qt.Key_Escape:
                            self._manual_stop()
                        elif key == Qt.Key_Space:
                            self.keyboard_keys.clear()
                            self._keyboard_refresh()
                        else:
                            if event.type() == QEvent.KeyPress:
                                self.keyboard_keys.add(key)
                            else:
                                self.keyboard_keys.discard(key)
                            self._keyboard_refresh()
                        return True
        return super().eventFilter(watched, event)

    def _manual_tick(self):
        if self.manual_vector is not None:
            # 原子替换旧速度目标，不被参数积压饿死，也不积累过期方向。
            # manual_vector 与 MANUAL 都是统一坐标，直接发送。
            left, forward, turn = self.manual_vector
            self.line_q.put("MANUAL=%d,%d,%d" % (left, forward, turn))

    def _manual_stop(self):
        if hasattr(self, "keyboard_enabled"):
            if self.keyboard_enabled:
                self.manual_status.setText("已退出键盘遥控 · 点击进入后重新接管")
            self.keyboard_enabled = False
            self.keyboard_keys.clear()
            self.keyboard_button.setText("进入键盘遥控")
            for label in self.keyboard_labels.values():
                label.setStyleSheet("background: #242A33; border: 1px solid #38434F; border-radius: 6px; padding: 6px;")
        if getattr(self, "manual_vector", None) is not None:
            self.manual_vector = None
            self.manual_timer.stop()
            self.manual_status.setText("已请求停车 · 再次按住可运行")
            self.send_line("STOP")

    def changeEvent(self, event):
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
            # 焦点可能只是转到独立窗口（例如在分离的底盘页里按 WASD），
            # 等事件循环走完再看哪个窗口真正在前台。
            QTimer.singleShot(0, self._activation_guard)
        super().changeEvent(event)

    def _own_window_active(self) -> bool:
        if self.isActiveWindow():
            return True
        return any(win.isActiveWindow() for win in getattr(self, "detached", {}).values())

    def _activation_guard(self):
        if not self._own_window_active():
            self._manual_stop()

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
        self.command_entry.setPlaceholderText("例如：KPX=3.0 / GOTO=100.0,60.0,90.0 / DMEN / STOP")
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

    # ---------------- 独立窗口（页面拆分 + 窗口置顶） ----------------
    def _page_index_of(self, page: QWidget) -> int:
        try:
            return self.pages.index(page)
        except (AttributeError, ValueError):
            return -1

    def _build_detach_placeholder(self, idx: int) -> QWidget:
        holder = QWidget()
        lay = QVBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)
        panel = QFrame()
        panel.setObjectName("Panel")
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(18, 18, 18, 18)
        pl.setSpacing(10)
        title = QLabel("「%s」正在独立窗口中显示" % self.page_names[idx])
        title.setObjectName("SectionTitle")
        title.setWordWrap(True)
        hint = QLabel("关闭独立窗口或点击「取回主窗口」，页面会回到主界面这个位置。")
        hint.setObjectName("HintLabel")
        hint.setWordWrap(True)
        row = QHBoxLayout()
        back = QPushButton("取回主窗口")
        back.setObjectName("PrimaryButton")
        back.clicked.connect(lambda _=False, i=idx: self._reattach_page(i))
        top = QCheckBox("窗口置顶")
        top.setToolTip("让该独立窗口保持在其他窗口之上")
        top.toggled.connect(lambda on, i=idx: self._set_page_topmost(i, on))
        row.addWidget(back)
        row.addWidget(top)
        row.addStretch(1)
        pl.addWidget(title)
        pl.addWidget(hint)
        pl.addLayout(row)
        lay.addWidget(panel)
        lay.addStretch(1)
        self.placeholder_top_checks[idx] = top
        return holder

    def _detach_page(self, idx: int, topmost: bool | None = None):
        if idx < 0 or idx >= len(getattr(self, "pages", [])):
            return
        win = self.detached.get(idx)
        if win is not None:
            if topmost:
                win.set_topmost(True)
            win.show()
            win.raise_()
            win.activateWindow()
            return
        page = self.pages[idx]
        win = self.page_windows.get(idx)
        if win is None:
            # 不传 parent：带父级的顶层窗口在 Windows 上是 owned window，
            # 会永久压在主窗口上面；独立窗口之间才能换层。
            win = DetachedPageWindow(idx, self.page_names[idx])
            win.reattachRequested.connect(
                lambda i, w=win: self._reattach_page(i, source=w))
            win.topmostToggled.connect(self._on_detached_topmost_toggled)
            self.page_windows[idx] = win
        # 无父级窗口不会继承主窗口样式表，每次拆出时复制一份。
        win.setStyleSheet(self.styleSheet())
        # 页面控件直接换父窗口：信号、定时器和键盘遥控都不重建。
        win.take_page(page)
        self.detached[idx] = win
        want_top = bool(topmost) if topmost is not None else bool(self.topmost_pref.get(idx, False))
        win.set_topmost(want_top)
        self._sync_placeholder_topmost(idx)
        self.slots[idx].setCurrentWidget(self.placeholders[idx])
        self._place_detached_window(win, idx)
        win.show()
        win.raise_()
        win.activateWindow()
        self._refresh_nav_state()
        self.log("「%s」已拆成独立窗口%s" %
                 (self.page_names[idx], "（窗口置顶）" if win.is_topmost() else ""), "info")

    def _place_detached_window(self, win: DetachedPageWindow, idx: int):
        if getattr(win, "_placed", False):
            return
        win._placed = True
        win.resize(860, 660)
        if self.isVisible():
            base = self.pos()
            win.move(base.x() + 120 + 26 * idx, base.y() + 70 + 22 * idx)

    def _reattach_page(self, idx: int, source=None):
        if not hasattr(self, "detached"):
            return
        # win.close() 会再发一次 reattachRequested，避免重入重复加回页面。
        if idx in self._reattaching:
            return
        win = self.detached.pop(idx, None)
        if win is None and source is None:
            return
        if idx < 0 or idx >= len(self.pages):
            return
        self._reattaching.add(idx)
        try:
            page = self.pages[idx]
            slot = self.slots[idx]
            # addWidget 自动换回主窗口父级，独立窗口的布局会随之移除该项。
            if slot.indexOf(page) < 0:
                slot.addWidget(page)
            slot.setCurrentWidget(page)
            if win is not None and win is not source:
                win.close()
        finally:
            self._reattaching.discard(idx)
        self._refresh_nav_state()
        self.log("「%s」已回到主窗口" % self.page_names[idx], "info")

    def _reattach_all(self):
        for idx in list(getattr(self, "detached", {})):
            self._reattach_page(idx)

    def _set_page_topmost(self, idx: int, on: bool):
        on = bool(on)
        self.topmost_pref[idx] = on
        win = self.detached.get(idx)
        if win is not None:
            win.set_topmost(on)
        self._sync_placeholder_topmost(idx)

    def _sync_placeholder_topmost(self, idx: int):
        check = self.placeholder_top_checks.get(idx)
        if check is None:
            return
        win = self.detached.get(idx)
        on = win.is_topmost() if win is not None else bool(self.topmost_pref.get(idx, False))
        if check.isChecked() != on:
            check.blockSignals(True)
            check.setChecked(on)
            check.blockSignals(False)

    def _on_detached_topmost_toggled(self, idx: int, on: bool):
        self.topmost_pref[idx] = bool(on)
        self._sync_placeholder_topmost(idx)
        self.log("「%s」独立窗口%s" % (self.page_names[idx], "已置顶" if on else "取消置顶"), "info")

    def _refresh_nav_state(self):
        if not hasattr(self, "nav_buttons"):
            return
        for i, b in enumerate(self.nav_buttons):
            b.setProperty("detached", i in getattr(self, "detached", {}))
            b.style().unpolish(b)
            b.style().polish(b)

    def _show_nav_menu(self, idx: int, pos):
        btn = self.nav_buttons[idx]
        win = self.detached.get(idx)
        menu = QMenu(self)
        act_detach = menu.addAction("拆成独立窗口")
        act_detach.setEnabled(win is None)
        act_detach.triggered.connect(lambda _=False, i=idx: self._detach_page(i))
        act_back = menu.addAction("取回主窗口")
        act_back.setEnabled(win is not None)
        act_back.triggered.connect(lambda _=False, i=idx: self._reattach_page(i))
        menu.addSeparator()
        act_top = menu.addAction("窗口置顶")
        act_top.setCheckable(True)
        act_top.setChecked(win.is_topmost() if win is not None else bool(self.topmost_pref.get(idx, False)))
        act_top.setEnabled(win is not None)
        act_top.triggered.connect(lambda on, i=idx: self._set_page_topmost(i, on))
        if self.detached:
            menu.addSeparator()
            act_all = menu.addAction("取回全部独立窗口")
            act_all.triggered.connect(lambda _=False: self._reattach_all())
        menu.exec(btn.mapToGlobal(pos))

    def _select_page(self, idx: int):
        self._manual_stop()
        if hasattr(self, "stack") and 0 <= idx < self.stack.count():
            self.stack.setCurrentIndex(idx)
        # 页面已拆出去时，导航按钮同时把独立窗口抬到最前。
        win = getattr(self, "detached", {}).get(idx)
        if win is not None and win.isVisible():
            win.raise_()
            win.activateWindow()
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
        self._manual_stop()
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
                text_q=self.fw_text_q,
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
        # 锁轴切换前先停止键盘续发，避免失能后仍周期发送MANUAL。
        if cmd in ("STOP", "ZERO", "GOTO", "OPSOFFSET", "WHEELEN", "WHEELOFF"):
            self._manual_stop()
        if cmd in core.URGENT_COMMANDS:
            self._drain_queue(self.line_q)
            self.urgent_q.put(text)
        else:
            self.line_q.put(text)
        # 终端手输同一命令也要同步界面状态，避免与实际请求不一致。
        if cmd == "WHEELEN":
            self.wheel_state = True
            self._update_wheel_status()
        elif cmd == "WHEELOFF":
            self.wheel_state = False
            self._update_wheel_status()
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
        self._manual_stop()
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
                    # 遥测为 cm，轨迹缓冲/地图几何仍统一使用 mm。
                    self.traj_ring.append(tr, (vals[0] * core.OPS_CM_TO_MM,
                                               vals[1] * core.OPS_CM_TO_MM))
                if self.recorder is not None:
                    self.recorder.write(t, vals)
        except queue.Empty:
            pass
        self.fps_count += got
        # 固件文字应答（如 CAN 启动失败）与上位机提示，转成日志而非静默丢弃。
        for _ in range(10):
            try:
                text = self.fw_text_q.get_nowait()
            except queue.Empty:
                break
            tag = "warn" if any(k in text.upper() for k in ("ERR", "FAIL", "错误", "失败")) else "info"
            self.log(text, tag)

    def _render_ui(self):
        v = self.latest
        if v is not None:
            self.cards["x"].set_value("%.1f" % v[0])
            self.cards["y"].set_value("%.1f" % v[1])
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

            self.ops_drift.setText("中心位置：X 左右 %.1f / Y 前后 %.1f cm ｜距零点 %.1f cm ｜航向 %.1f°" % (v[0], v[1], math.hypot(v[0], v[1]), v[2]))
            fx, fy = self._ops_to_field(v[0] * core.OPS_CM_TO_MM,
                                        v[1] * core.OPS_CM_TO_MM)
            self.map_view.set_pose(fx, fy, self.map_theta + v[2])
            ux, uy = core.layout_to_field(fx, fy)
            # 固件在 ZERO（在所选启停区校准）时把航向也归零，因此遥测 v[2] 已是
            # "相对校准朝向"的角度；场地约定 0°=场地+X(左)、90°=场地+Y(上)，
            # 校准时车头指向场地+X ⇒ 场地方向 = 车体方向 − map_theta。
            heading = (v[2] - self.map_theta) % 360.0
            self.map_position.setText("场地 X(左)=%.1f cm   Y(前)=%.1f cm   航向=%.1f°（0°左 / 90°上）"
                                      % (ux / core.OPS_CM_TO_MM,
                                         uy / core.OPS_CM_TO_MM, heading))

        # 只绘制可见页面：主窗口当前页 + 已拆到独立窗口的页面。
        active = {self.stack.currentIndex()} | set(self.detached)
        if 1 in active:
            self.wave_page.update_data(self.ring.view(), self.latest_t, self.window_s)
        if 0 in active:
            self._update_dashboard_plots(self.ring.view())
        if 2 in active:
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
        # 与 _ops_to_field 同一变换的向量化副本：协议 X=左右、Y=前后。
        fx, fy = core.field_to_layout(self.map_ox + c * x + s * y,
                                      self.map_oy - s * x + c * y)
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
    def _update_wheel_status(self):
        """刷新四轮锁轴提示。固件无状态回读，只能显示本机发出的请求。"""
        if not hasattr(self, "wheel_status"):
            return
        if self.wheel_state is None:
            text = "电机：未请求（固件上电默认已使能；无回读，请以轮子能否推动确认）"
        elif self.wheel_state:
            text = "电机：已请求使能（锁轴）；无回读，请以轮子能否推动确认"
        else:
            text = "电机：已请求失能（不锁轴）；GOTO、键盘遥控与 ZDT 单轮测试将被固件拒绝"
        self.wheel_status.setText(text)

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
        # 两个 SpinBox 对外为 cm，内部地图几何继续用 mm。
        self.map_ox = float(self.map_ox_spin.value()) * core.OPS_CM_TO_MM
        self.map_oy = float(self.map_oy_spin.value()) * core.OPS_CM_TO_MM
        self.map_theta = float(self.map_theta_spin.value())
        self.log("场地映射：原点(%.1f, %.1f) cm，旋转 %.1f°"
                 % (self.map_ox_spin.value(), self.map_oy_spin.value(), self.map_theta), "info")

    def _ops_to_field(self, x_mm: float, y_mm: float):
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        # 输入是已换算为 mm 的遥测 x=X=左右、y=Y=前后，与场地坐标(+X向左、+Y向上)轴序一致，
        # 因此这里只做一次绕 map_theta 的旋转，不再交换或取反。
        return core.field_to_layout(self.map_ox + c * x_mm + s * y_mm,
                                    self.map_oy - s * x_mm + c * y_mm)

    def _field_to_ops(self, fx: float, fy: float):
        """场地显示坐标 → 固件 OPS 内部 mm，发送 GOTO 前再除以 10 转为 cm。"""
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        ux, uy = core.layout_to_field(fx, fy)
        px, py = ux - self.map_ox, uy - self.map_oy
        return c * px - s * py, s * px + c * py

    def _set_start_zone(self):
        zone = int(self.zone_combo.currentData())
        zx, zy = core.ZONE_CENTER[zone]
        ux, uy = core.layout_to_field(zx, zy)
        self.map_ox_spin.setValue(ux / core.OPS_CM_TO_MM)
        self.map_oy_spin.setValue(uy / core.OPS_CM_TO_MM)
        self._apply_map_mapping()
        self.send_line("ZERO")
        self.traj_ring.clear()
        self.map_target = None
        self.map_status.setText("OPS零点校准至场地(%.1f, %.1f) cm；场地原点固定在启停区1"
                                % (ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM))
        self.log("启停区%d -> OPS零点(%.1f, %.1f) cm，请求 ZERO"
                 % (zone, ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM), "info")

    def _target_yaw_ops(self):
        """场地方向 → 固件 GOTO 的 Z（车体相对航向，ZERO 时已归零）。

        关系：场地方向 = 车体方向 − map_theta（见 _render_ui 的 heading），取逆即可。
        """
        field_yaw = self.map_yaw_combo.currentData()
        if field_yaw is None:
            return float(self.latest[2]) if self.latest is not None else 0.0
        return float(field_yaw) + self.map_theta

    def _goto_field(self, fx: float, fy: float, yaw_override=None):
        """场地坐标下发 GOTO。

        约定（与界面上"启停区为原点、无负坐标"一致）：
        - 场地坐标原点 = 所选启停区（用「在所选区校准 OPS 零点」把 OPS 零点对到该区）；
        - 场地坐标恒为 0..FIELD 的非负值，**GOTO 不允许负坐标/越界**，越界直接拒绝；
        - 遥测/调参通道仍显示机器人相对坐标，小车跑出启停区出现负值属正常（用于调参）。
        - 地图内部用 mm 做碰撞检查，发送 GOTO 前换算为 cm（1 位小数）。
        """
        if not (0.0 <= fx <= self.map_view.FIELD and 0.0 <= fy <= self.map_view.FIELD):
            self.map_status.setText("拒绝 GOTO：目标(%.1f, %.1f) cm 超出场地范围 0..%.1f cm"
                                    % (fx / core.OPS_CM_TO_MM, fy / core.OPS_CM_TO_MM,
                                       self.map_view.FIELD / core.OPS_CM_TO_MM))
            self.log("GOTO 拒绝：场地坐标越界(%.1f, %.1f) cm，场地为 0..%.1f cm 无负坐标"
                     % (fx / core.OPS_CM_TO_MM, fy / core.OPS_CM_TO_MM,
                        self.map_view.FIELD / core.OPS_CM_TO_MM), "warn")
            return
        if self.worker is None and self.sim is None:
            self.log("未连接，导航未发送", "warn")
            return
        if self.wheel_state is False:
            self.map_status.setText("四轮已请求失能：固件会拒绝 GOTO，请先「使能电机（锁轴）」")
            self.log("GOTO 未发送：四轮已请求失能", "warn")
            return
        if self.latest is None or not all(math.isfinite(v) for v in self.latest[:3]):
            self.log("无有效定位，导航未发送", "warn")
            return
        if self.worker is not None and (not self.worker.opened.is_set() or
                time.monotonic() - self.worker.last_frame_monotonic > core.TELEMETRY_WARN_S):
            self.log("定位遥测过期，导航未发送", "warn")
            return
        blocked = core.field_point_blocked(fx, fy)
        if blocked:
            self.map_status.setText("拒绝 GOTO：目标位于【%s】" % blocked)
            self.log("GOTO 拒绝：目标(%.1f, %.1f) cm 位于%s"
                     % (fx / core.OPS_CM_TO_MM, fy / core.OPS_CM_TO_MM, blocked), "warn")
            return

        if self.latest is not None and math.isfinite(self.latest[0]) and math.isfinite(self.latest[1]):
            sx, sy = self._ops_to_field(self.latest[0] * core.OPS_CM_TO_MM,
                                        self.latest[1] * core.OPS_CM_TO_MM)
            blocked = core.field_path_blocked(sx, sy, fx, fy)
            if blocked:
                self.map_status.setText("拒绝直线 GOTO：路径穿越【%s】，请先选择中间安全点" % blocked)
                self.log("GOTO 拒绝：直线路径穿越%s" % blocked, "warn")
                return

        tx, ty = self._field_to_ops(fx, fy)
        yaw = self._target_yaw_ops() if yaw_override is None else yaw_override
        self.map_target = (fx, fy)
        tx_cm, ty_cm = tx / core.OPS_CM_TO_MM, ty / core.OPS_CM_TO_MM
        self.send_line("GOTO=%.1f,%.1f,%.1f" % (tx_cm, ty_cm, yaw))
        ux, uy = core.layout_to_field(fx, fy)
        self.map_status.setText("目标：场地(%.1f, %.1f) cm → OPS GOTO=%.1f,%.1f,%.1f"
                                % (ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM,
                                   tx_cm, ty_cm, yaw))

    def _goto_home(self):
        zone = int(self.zone_combo.currentData())
        zx, zy = core.ZONE_CENTER[zone]
        home_yaw = (0.0 if zone == 1 else 180.0) + self.map_theta
        self._goto_field(zx, zy, home_yaw)

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
        self._manual_stop()
        try:
            # 先把页面收回主窗口，独立窗口不会再拦截退出。
            self._reattach_all()
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
