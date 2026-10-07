# -*- coding: utf-8 -*-
"""ILHC Control Station v2.1.23 — PySide6 + PyQtGraph UI.

保留 v1.1 的通信/安全核心，重做桌面 UI：
- PySide6 Qt Widgets
- PyQtGraph 实时波形
- QGraphicsView 比赛场地地图
- 高优先级 STOP/DMOFF、GUI 心跳、遥测超时
- 底盘/DM 参数调节、CSV 记录、命令行、模拟模式
"""

from __future__ import annotations

import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import csv
import html
import json
import math
import os
import queue
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np

import core
from run_journal import RunJournal
from ops_diagnostics import parse_ops_text
try:
    import navigation_planner as nav
except ImportError as exc:
    print("缺少规划依赖：%s；请运行 python -m pip install -r requirements.txt" % exc)
    raise SystemExit(2)

# --selftest 在加载 PySide6/PyQtGraph 之前执行，便于无 GUI 环境验证核心。
if "--selftest" in sys.argv:
    core.selftest()
    raise SystemExit(0)

try:
    import pyqtgraph as pg
    from PySide6.QtCore import QEvent, QObject, QPointF, QRectF, QSize, Qt, QTimer, QUrl, Signal
    from PySide6.QtGui import (
        QBrush,
        QColor,
        QCloseEvent,
        QDesktopServices,
        QFont,
        QPainter,
        QPainterPath,
        QPen,
        QPolygonF,
    )
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDialog,
        QDoubleSpinBox,
        QFileDialog,
        QFrame,
        QGraphicsEllipseItem,
        QGraphicsLineItem,
        QGraphicsPathItem,
        QGraphicsPolygonItem,
        QGraphicsRectItem,
        QGraphicsScene,
        QGraphicsSimpleTextItem,
        QGraphicsView,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QLayout,
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
    workerError = Signal(object, str)


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
        self.readback_driven = False
        self.readback_known = False
        self.input_dirty = False

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

        send = self.send_button = QPushButton("发送")
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
        self.input_dirty = True
        self.spin.blockSignals(True)
        if self.integer:
            self.spin.setValue(value)
        else:
            self.spin.setValue(self._slider_to_float(value))
        self.spin.blockSignals(False)

    def _spin_changed(self, value):
        self.input_dirty = True
        self.slider.blockSignals(True)
        if self.integer:
            self.slider.setValue(int(value))
        else:
            self.slider.setValue(self._float_to_slider(float(value)))
        self.slider.blockSignals(False)

    def _send(self):
        if self.readback_driven and not self.readback_known:
            return
        self.input_dirty = False
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

    def reset_readback(self):
        self.readback_known = self.input_dirty = False
        self.readback.setText('等待回读')
        self.spin.clear()
        self.spin.setEnabled(False)
        self.slider.setEnabled(False)
        self.send_button.setEnabled(False)

    def set_readback(self, value: float):
        if not math.isfinite(value):
            self.readback.setText("回读 —")
            return
        if self.integer:
            text = str(int(round(value)))
        else:
            text = ("%.3f" % value).rstrip("0").rstrip(".")
        self.readback.setText("回读 " + text)
        if self.readback_driven:
            if not self.readback_known or not self.input_dirty:
                self.spin.blockSignals(True)
                self.slider.blockSignals(True)
                self.spin.setValue(int(round(value)) if self.integer else value)
                self.slider.setValue(int(round(value)) if self.integer else self._float_to_slider(value))
                self.spin.blockSignals(False)
                self.slider.blockSignals(False)
            self.readback_known = True
            self.spin.setEnabled(True)
            self.slider.setEnabled(True)
            self.send_button.setEnabled(True)


class StepperCardGrid(QWidget):
    """宽窗口双列、窄窗口单列，不让隐藏的原始参数撑宽页面。"""
    def __init__(self, cards):
        super().__init__()
        self.cards = cards
        self.columns = 0
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(16)
        self.grid.setSizeConstraint(QLayout.SetNoConstraint)
        self.arrange(2)

    def minimumSizeHint(self):
        return QSize(0, super().minimumSizeHint().height())

    def arrange(self, columns):
        if columns == self.columns:
            return
        self.columns = columns
        for i, card in enumerate(self.cards):
            self.grid.removeWidget(card)
            self.grid.addWidget(card, i//columns, i % columns, alignment=Qt.AlignTop)
        self.grid.setColumnStretch(0, 1)
        self.grid.setColumnStretch(1, 1 if columns == 2 else 0)
        self.updateGeometry()

    def resizeEvent(self, event):
        self.arrange(2 if event.size().width() >= 1100 else 1)
        super().resizeEvent(event)


class MotorAddressRow(QFrame):
    """电机地址是离散设置，单独用整数输入，保持参数回读接口。"""
    sendRequested = Signal(str, str)

    def __init__(self):
        super().__init__()
        self.setObjectName("DmAddressSetting")
        self.cmd, self.readback_channel, self.integer = "DMID", 12, True
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(QLabel("电机 ID"))
        self.spin = QSpinBox()
        self.spin.setRange(1, 1791)
        self.spin.setValue(3)
        self.spin.setFixedWidth(86)
        self.spin.setToolTip("DM 地址；修改后需点击应用 ID，电机应先失能")
        self.spin.setAccelerated(False)
        layout.addWidget(self.spin)
        self.readback = QLabel("回读 —")
        self.readback.setObjectName("DmAddressReadback")
        self.readback.setMinimumWidth(78)
        layout.addWidget(self.readback)
        send = QPushButton("应用 ID")
        send.setToolTip("发送 DMID，仅在电机失能且无待执行动作时生效")
        send.clicked.connect(lambda: self.sendRequested.emit(self.cmd, str(self.spin.value())))
        layout.addWidget(send)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)

    def command_text(self):
        return "DMID=%d" % self.spin.value()

    def set_readback(self, value):
        self.readback.setText("回读 %d" % round(value) if math.isfinite(value) else "回读 —")


class FieldView(QGraphicsView):
    gotoRequested = Signal(float, float)

    FIELD = 2400.0                  # 场地边长（地图单位 = mm）
    CAR_LEN_MM = core.CAR_LENGTH_MM  # 车长 28 cm，沿车头方向
    CAR_WID_MM = core.CAR_WIDTH_MM   # 车宽 26 cm，沿车左方向

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
        self._pose_key = None
        self._target_key = object()
        self.scene_obj.clear()
        # 留白只够放刻度文字（绘图帧的 -80/-100 标注），让场地尽量占满视图
        self.scene_obj.setSceneRect(-150, -150, 2700, 2700)

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

        device_background_before = set(self.scene_obj.items())
        # 原料区
        self._field_circle(1200, 2400, 150, "#EDF1F5", "#4B535F", 2)
        for hx, hy in ((1200, 2445), (1160, 2375), (1240, 2375)):
            self._field_circle(hx, hy, 20, "#7D8793", "#4B535F", 1)
        self._add_centered_text("原料区", 1200, 2255, "#454B55", 10, True)

        # 暂存区
        self._field_rect(0, 910, 150, 1490, "#EDF1F5", "#4B535F", 2)
        for hy in (1050, 1200, 1350):
            self._field_circle(75, hy, 24, "#7D8793", "#4B535F", 1)
        self._add_centered_text("暂存区", 255, 1200, "#454B55", 10, True)

        # 粗加工区
        self._field_rect(910, 0, 1490, 150, "#EDF1F5", "#4B535F", 2)
        for hx in (1050, 1200, 1350):
            self._field_circle(hx, 75, 24, "#7D8793", "#4B535F", 1)
        self._add_centered_text("粗加工区", 1200, 255, "#454B55", 10, True)

        self._device_background = list(set(self.scene_obj.items()) - device_background_before)
        # 二维码板
        self._field_rect(2388, 1170, 2400, 1230, "#171A1E", "#171A1E", 1)
        self._add_centered_text("二维码板", 2275, 1200, "#454B55", 9, True)

        # 动态图元
        self.trail_item = QGraphicsPathItem()
        self.trail_item.setPen(QPen(QColor(ACCENT), 5))
        self.trail_item.setZValue(10)
        self.scene_obj.addItem(self.trail_item)

        self.skeleton_item = QGraphicsPathItem()
        self.skeleton_item.setPen(QPen(QColor('#F0BD55'), 3, Qt.DashLine))
        self.skeleton_item.setZValue(9)
        self.scene_obj.addItem(self.skeleton_item)
        self.pivot_item=QGraphicsPathItem()
        self.pivot_item.setPen(QPen(QColor('#FF8A4C'),2,Qt.DashLine))
        self.pivot_item.setZValue(12)
        self.scene_obj.addItem(self.pivot_item)

        # A* 规划路径（只显示，不下发）：虚线 + 航点圆点
        self.path_item = QGraphicsPathItem()
        plan_pen = QPen(QColor(GREEN), 5)
        plan_pen.setStyle(Qt.SolidLine)
        self.path_item.setPen(plan_pen)
        self.path_item.setZValue(11)
        self.scene_obj.addItem(self.path_item)

        self.path_dots = QGraphicsPathItem()
        self.path_dots.setPen(QPen(Qt.NoPen))
        self.path_dots.setBrush(QBrush(QColor(GREEN)))
        self.path_dots.setZValue(12)
        self.scene_obj.addItem(self.path_dots)

        self.trajectory_arrows = QGraphicsPathItem()
        self.trajectory_arrows.setPen(QPen(QColor(ACCENT), 3))
        self.trajectory_arrows.setZValue(12.5)
        self.scene_obj.addItem(self.trajectory_arrows)

        self.reference_item = QGraphicsPathItem()
        self.reference_item.setPen(QPen(QColor('#D28BFF'), 4))
        self.reference_item.setZValue(15)
        self.scene_obj.addItem(self.reference_item)

        # 车体：按实际尺寸绘制的顶视轮廓，随航向旋转（长28cm 沿车头、宽26cm 沿车左）
        self.car_item = QGraphicsPolygonItem()
        self.car_item.setBrush(QBrush(QColor("#FF9F43")))
        self.car_item.setPen(QPen(QColor("#FFFFFF"), 3))
        self.car_item.setZValue(13)
        self.scene_obj.addItem(self.car_item)

        # 车头楔形（前缘两点 + 车心）：让朝向一眼可辨，不靠颜色深浅猜
        self.car_nose = QGraphicsPolygonItem()
        self.car_nose.setBrush(QBrush(QColor("#FFF3D8")))
        self.car_nose.setPen(QPen(QColor("#B4651B"), 2))
        self.car_nose.setZValue(14)
        self.scene_obj.addItem(self.car_nose)

        self.target_h = QGraphicsLineItem()
        self.target_v = QGraphicsLineItem()
        for item in (self.target_h, self.target_v):
            item.setPen(QPen(QColor(RED), 7))
            item.setZValue(14)
            item.setVisible(False)
            self.scene_obj.addItem(item)

        # 用户坐标的绘图锚点；实测地图零点由实际出发位置执行OPS ZERO建立。
        axis_pen = QPen(QColor("#C0392B"), 4)
        self.scene_obj.addLine(2250, self.sy(2250), 1800, self.sy(2250), axis_pen)
        self.scene_obj.addLine(1800, self.sy(2250), 1850, self.sy(2280), axis_pen)
        self.scene_obj.addLine(1800, self.sy(2250), 1850, self.sy(2220), axis_pen)
        self.scene_obj.addLine(2250, self.sy(2250), 2250, self.sy(1800), axis_pen)
        self.scene_obj.addLine(2250, self.sy(1800), 2220, self.sy(1850), axis_pen)
        self.scene_obj.addLine(2250, self.sy(1800), 2280, self.sy(1850), axis_pen)
        self._add_centered_text("+Y 上", 1770, 2310, "#C0392B", 10, True)
        self._add_centered_text("+X 左", 2310, 1750, "#C0392B", 10, True)
        self._add_centered_text("OPS (0.0, 0.0)", 2350, 2250, "#FFFFFF", 9)
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

    def set_navigation_map(self, data):
        """Show the SAME whitelist/obstacles used by planning; decorative base is reference."""
        for item in self._device_background:
            item.setVisible(False)
        for item in getattr(self, "_navigation_overlay", []):
            self.scene_obj.removeItem(item)
        self._navigation_overlay = []
        allowed = QPainterPath()
        for region in data["drivable_polygons"]:
            outer = region.get("outer") if isinstance(region, dict) else region
            holes = region.get("holes", []) if isinstance(region, dict) else []
            part = QPainterPath()
            part.setFillRule(Qt.OddEvenFill)
            for ring in [outer] + holes:
                part.moveTo(float(ring[0][0]), self.sy(float(ring[0][1])))
                for x, y in ring[1:]:
                    part.lineTo(float(x), self.sy(float(y)))
                part.closeSubpath()
            allowed = allowed.united(part)
        whole = QPainterPath()
        whole.addRect(0, 0, self.FIELD, self.FIELD)
        denied = QGraphicsPathItem(whole.subtracted(allowed))
        denied.setBrush(QBrush(QColor(180, 50, 50, 90)))
        denied.setPen(QPen(Qt.NoPen))
        denied.setZValue(7)
        self.scene_obj.addItem(denied)
        self._navigation_overlay.append(denied)
        outline = QGraphicsPathItem(allowed)
        outline.setPen(QPen(QColor("#5AD68A"), 3, Qt.DashLine))
        outline.setZValue(8)
        self.scene_obj.addItem(outline)
        self._navigation_overlay.append(outline)
        self.device_geometry = {}
        for x0, y0, x1, y1, name in list(data["rects"]) + list(data.get("dynamic_rects", [])):
            item = self._field_rect(x0, y0, x1, y1, "#80646464", "#BD6D6D", 3, 8)
            self._navigation_overlay.append(item)
            if name in ('粗加工区设备', '暂存区设备'):
                self.device_geometry[name] = (x0,y0,x1,y1)
                cx,cy=(x0+x1)/2,(y0+y1)/2
                for offset in (-150,0,150):
                    hx,hy=(cx+offset,cy) if name=='粗加工区设备' else (cx,cy+offset)
                    self._navigation_overlay.append(self._field_circle(hx,hy,24,'#7D8793','#4B535F',1,9))
                lx,ly=(cx,y1+105) if name=='粗加工区设备' else (x1+105,cy)
                self._navigation_overlay.append(self._add_centered_text(name.removesuffix('设备'),lx,ly,'#454B55',10,True))
        for x, y, r, name in list(data["circles"]) + list(data.get("dynamic_circles", [])):
            item = self._field_circle(x, y, r, "#80646464", "#BD6D6D", 3, 8)
            self._navigation_overlay.append(item)
            if name=='原料区圆盘':
                self.device_geometry[name] = (x,y,r)
                for dx,dy in ((0,45),(-40,-25),(40,-25)):
                    self._navigation_overlay.append(self._field_circle(x+dx,y+dy,20,'#7D8793','#4B535F',1,9))
                self._navigation_overlay.append(self._add_centered_text('原料区',x,y-r-35,'#454B55',10,True))

    def set_pose(self, cx: float, cy: float, nose, left):
        """按地图帧设置车体姿态：中心 (cx, cy) 与两个单位方向向量。

        nose/left 是**地图绘制帧**里的向量，由 MainWindow 用与轨迹相同的
        `_ops_to_field` 线性部分推出。绘制帧是统一坐标系的一次镜像旋转
        （行列式 −1），所以角度不能直接相加传递，这里只吃向量——这样车体
        方向与轨迹走向必然出自同一次变换。
        """
        if not all(math.isfinite(v) for v in (cx, cy, nose[0], nose[1], left[0], left[1])):
            return
        key = (cx,cy,*nose,*left)
        if self._pose_key == key:
            return
        self._pose_key = key
        hl = self.CAR_LEN_MM / 2.0
        hw = self.CAR_WID_MM / 2.0
        nx, ny = float(nose[0]), float(nose[1])
        lx, ly = float(left[0]), float(left[1])

        def pt(fwd: float, side: float) -> QPointF:
            return QPointF(cx + fwd * nx + side * lx, self.sy(cy + fwd * ny + side * ly))

        self.car_item.setPolygon(QPolygonF([pt(hl, hw), pt(hl, -hw), pt(-hl, -hw), pt(-hl, hw)]))
        # 三角尖端明确指向车头；旧图形底边在前、尖端朝车心，用户容易看反。
        self.car_nose.setPolygon(QPolygonF([pt(hl, 0), pt(0, hw*.6), pt(0, -hw*.6)]))

    def set_trail(self, x: np.ndarray, y: np.ndarray):
        if len(x) == 0:
            self.trail_item.setPath(QPainterPath())
            return
        path = QPainterPath()
        previous = None
        for xx, yy in zip(x, y):
            xx, yy = float(xx), float(yy)
            if not (math.isfinite(xx) and math.isfinite(yy)) or max(abs(xx),abs(yy)) > 1e6:
                previous = None
                continue
            if previous is None or math.hypot(xx-previous[0],yy-previous[1]) > 500:
                path.moveTo(xx, self.sy(yy))
            else:
                path.lineTo(xx, self.sy(yy))
            previous = xx, yy
        self.trail_item.setPath(path)

    def set_path(self, points, waypoint_points=None):
        """绘制布局mm轨迹；圆弧采样只连线，航点标记由waypoint_points给定。"""
        self.set_trajectory(None)
        self.set_pivots(None)
        if not points:
            self.path_item.setPath(QPainterPath())
            self.path_dots.setPath(QPainterPath())
            self.set_skeleton(None)
            self.set_reference(None)
            return
        pts = [(float(px), float(py)) for px, py in points]
        path = QPainterPath(QPointF(pts[0][0], self.sy(pts[0][1])))
        for px, py in pts[1:]:
            path.lineTo(px, self.sy(py))
        self.path_item.setPath(path)
        dots = QPainterPath()
        for px, py in (pts if waypoint_points is None else waypoint_points):
            dots.addEllipse(QPointF(px, self.sy(py)), 16.0, 16.0)
        self.path_dots.setPath(dots)

    def set_skeleton(self, points):
        path = QPainterPath()
        for index, (x, y) in enumerate(points or ()):
            if index == 0:
                path.moveTo(x, self.sy(y))
            else:
                path.lineTo(x, self.sy(y))
        self.skeleton_item.setPath(path)

    def set_pivots(self,pivots):
        path=QPainterPath()
        for pivot in pivots or ():
            cx,cy=pivot['center_mm'];path.addEllipse(QPointF(cx,self.sy(cy)),9,9)
            path.moveTo(cx-18,self.sy(cy));path.lineTo(cx+18,self.sy(cy))
            path.moveTo(cx,self.sy(cy-18));path.lineTo(cx,self.sy(cy+18))
            for x,y in (pivot['entry_mm'],pivot['exit_mm']):
                path.moveTo(cx,self.sy(cy));path.lineTo(x,self.sy(y))
        self.pivot_item.setPath(path)

    def set_reference(self, sample):
        path = QPainterPath()
        if sample is not None:
            x, y = core.field_to_layout(sample['x_mm'], sample['y_mm'])
            sy = self.sy(y)
            yaw = math.radians(sample['field_yaw_deg'])
            dx, dy = -math.sin(yaw), math.cos(yaw)
            path.addEllipse(QPointF(x, sy), 12, 12)
            path.moveTo(x, sy)
            path.lineTo(x+40*dx, sy+40*dy)
            path.lineTo(x+24*dx-9*dy, sy+24*dy+9*dx)
            path.moveTo(x+40*dx, sy+40*dy)
            path.lineTo(x+24*dx+9*dy, sy+24*dy-9*dx)
        self.reference_item.setPath(path)

    def set_trajectory(self, samples):
        """场地坐标Trajectory的切线方向箭头；矢量变换与位置变换一致。"""
        arrows = QPainterPath()
        last_s, last_type = -math.inf, None
        for index, sample in enumerate(samples or ()):
            station, kind = sample['s_mm'], sample['segment_type']
            if kind == 'ROTATE':
                continue
            interval = 40.0 if kind in ('ARC','PIVOT') else 100.0
            if (kind == last_type and station-last_s < interval-nav.EPS and index != len(samples)-1):
                continue
            x, y = core.field_to_layout(sample['x_mm'], sample['y_mm'])
            yaw = math.radians(sample.get('tangent_yaw_deg', sample['field_yaw_deg']))
            # FIELD→LAYOUT: (-dy,-dx)，LAYOUT→Qt再反转y，避免镜像箭头方向画反。
            dx, dy = -math.sin(yaw), math.cos(yaw)
            sy = self.sy(y)
            tip = QPointF(x+18*dx, sy+18*dy)
            arrows.moveTo(x-18*dx, sy-18*dy)
            arrows.lineTo(tip)
            arrows.moveTo(x+4*dx-9*dy, sy+4*dy+9*dx)
            arrows.lineTo(tip)
            arrows.lineTo(x+4*dx+9*dy, sy+4*dy-9*dx)
            last_s, last_type = station, kind
        self.trajectory_arrows.setPath(arrows)

    def set_sim_obstacles(self, points):
        """绘制模拟障碍：黑色实心圆（φ50mm），points 是布局帧中心点列。

        与地图自带障碍同尺寸口径（r 取 core.SIM_OBSTACLE_R_MM），z 值放在路径之上、
        车体之下，所以车压上去时能看出压着障碍。
        """
        for item in getattr(self, "_obstacle_items", []):
            self.scene_obj.removeItem(item)
        self._obstacle_items = []
        r = float(core.SIM_OBSTACLE_R_MM)
        for px, py in points:
            item = QGraphicsEllipseItem(float(px) - r, self.sy(float(py)) - r, 2 * r, 2 * r)
            item.setBrush(QBrush(QColor("#0E1014")))          # 黑
            item.setPen(QPen(QColor("#E8ECF1"), 2))           # 浅色描边，灰色场地上看得见
            item.setZValue(12.5)
            self.scene_obj.addItem(item)
            self._obstacle_items.append(item)

    def set_target(self, fx: float | None, fy: float | None):
        if self._target_key == (fx,fy):
            return
        self._target_key = (fx,fy)
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
    @property
    def nav_map(self):
        return self._nav_map

    @nav_map.setter
    def nav_map(self, data):
        if hasattr(self, '_nav_map'):
            self._clear_path('地图已替换，连续跟踪已取消')
        self._nav_map = core.NavigationMap(data, self._on_map_data_changed)
        self._sync_work_points()

    def _on_map_data_changed(self):
        self._clear_path('地图版本/几何改变，连续跟踪已取消')

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setWindowTitle("ILHC Control Station v2.1.23 — 三种转弯模式 / 实测OPS出发零点 / STM32F407VET6")
        self.resize(1500, 930)
        self.setMinimumSize(1180, 760)

        # 后端状态
        self.frame_q = queue.Queue(maxsize=4000)
        self.line_q = core.CommandQueue()
        self.urgent_q = queue.Queue()
        # 固件文字应答/错误与上位机提示（如补发 VOFA）都显示到日志，
        # 否则固件在文字模式下报的 CAN 失败等信息会被解析器静默丢弃。
        self.fw_text_q = queue.Queue(maxsize=200)
        # 参数回读（固件文字应答 "<名称>=<值>"）：刷新"回读"栏，不进日志。
        self.param_q = queue.Queue(maxsize=200)
        self.param_poll_index = 0
        self.worker = None
        self._closing_worker = None
        self.sim = None
        self.recorder = None
        self.run_journal = RunJournal(BASE_DIR/'records'/'runs', core.CHANNELS)
        self._journal_source = None
        self._journal_warning = ''
        self.t0_monotonic = time.monotonic()
        self.latest = None
        self.latest_t = 0.0
        self.latest_received_monotonic = 0.0
        self.window_s = 30.0
        self.ring = core.RingBuffer(int(self.window_s * core.SEND_HZ) + 100, core.FRAME_FLOATS)
        self.traj_ring = core.RingBuffer(int(self.window_s * core.SEND_HZ) + 100, 2)
        self._map_trail_ring = core.RingBuffer(int(self.window_s * core.SEND_HZ) + 100, 2)
        self._map_pose_filter = core.MapPoseFilter()
        self._display_source = None
        self._display_pose = None
        self._display_pose_ok = True
        self.paused = False
        # 四轮锁轴状态：None未请求 / True已请求使能 / False已请求失能。
        # 固件无状态回读，这里只记录本机发出的最后一条请求。
        self.wheel_state = None
        self.map_ox = 0.0
        self.map_oy = 0.0
        self.map_theta = 0.0
        self.map_target = None
        # A* 路径规划：本阶段只计算与显示，不向 STM32 下发任何命令
        self.planned_points = []      # 地图绘制帧的航点表（含起点）
        self.follow = None            # continuous trajectory tracking, simulator only
        self.real_match = None
        self._home_after_stop = False
        self._direct_home = None
        self._real_future = self._real_cancel = self._real_context = None
        self.competition = None
        self._competition_future = self._competition_cancel = None
        # 模拟障碍（φ50×100mm 圆柱，顶视 r=25mm）：运行时演示物体，**不写进地图数据**
        self.sim_obstacles = []
        self.planned_result = None
        self._planned_context = None
        self._plan_request_id = 0
        self._plan_future = None
        self._plan_cancel = None
        self._plan_context_pending = None
        self._planner_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="ILHC-planner")
        self._planner_error_log = BASE_DIR / "logs" / "ilhc-planner-error.log"
        self.nav_map = nav.load_map(BASE_DIR / "navigation_map.json")
        self.last_heartbeat_enqueue = 0.0
        self.fps_count = 0
        self.fps = 0.0
        self.fps_t = time.monotonic()
        self.ui_refresh_count = 0
        self.ui_refresh_hz = 0.0
        self._rx_previous_bytes = 0
        self._rx_bytes_per_second = 0.0
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
            "视觉跟踪",
        ]
        # 视觉参数没有遥测通道，进入该页面时批量刷新一次回读（见 _select_page）。
        self.vision_page_index = self.page_names.index("视觉跟踪")
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
            self._build_vision_page(),
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

        self.link_mode_combo=QComboBox()
        self.link_mode_combo.addItem('DL-20无线','dl20')
        self.link_mode_combo.addItem('USB / 蓝牙SPP','wired')
        self.link_mode_combo.setCurrentIndex(1)
        self.link_mode_combo.setToolTip('连接前选择。DL-20：位置50Hz、完整状态5Hz；USB或高速SPP：完整24通道50Hz。')
        lay.addWidget(self.link_mode_combo)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.setObjectName("ConnectButton")
        self.connect_btn.clicked.connect(lambda: self.toggle_connect())
        lay.addWidget(self.connect_btn)

        self.sim_btn = QPushButton("模拟")
        self.sim_btn.clicked.connect(lambda: self.toggle_sim())
        lay.addWidget(self.sim_btn)

        self.pause_btn = QPushButton("暂停波形")
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
        ver = QLabel("v2.1.23 Qt\nThree turn modes")
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
        self.footer_fps = QLabel("刷新 0 Hz · 遥测 0 Hz")
        self.footer_fps.setMinimumWidth(190)
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
        page, lay = self._page_shell("比赛场地", "实测出发点 OPS 零点 (0,0)；置零时车左 +X、车头 +Y；单位 cm，航向0°朝前")

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
        origin = QPushButton("当前位置置零并同步地图")
        origin.setToolTip("先摆回测量三个作业点时的实际出发位置和朝向；回库返回这个零点。")
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
        self.map_apply_button = apply_map
        self._update_origin_controls()

        cl.addSpacing(12)
        cl.addWidget(QLabel("目标航向"))
        self.map_yaw_combo = QComboBox()
        self.map_yaw_combo.addItem("保持当前", None)
        for deg, direction in ((0, '上 +Y'), (90, '左 +X'), (180, '下 −Y'), (270, '右 −X')):
            self.map_yaw_combo.addItem("%d° %s" % (deg, direction), float(deg))
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
        for name, anchor, outward in core.QUICK_ANCHORS:
            b = QPushButton(name)
            b.setToolTip("按右侧塔吊作业航向与裕量规划接近点；实机整批执行，模拟先规划后执行")
            b.clicked.connect(lambda _=False, n=name, a=anchor, o=outward:
                              self._request_plan_anchor(n, a, o))
            ql.addWidget(b)
        home = QPushButton("回出发零点")
        home.setToolTip("正常回库走避障规划；手动STOP/保护停车后可直接回库，恢复车头0°")
        home.clicked.connect(self._goto_home)
        ql.addWidget(home)
        ql.addStretch(1)
        self.map_status = QLabel("点击地图 = 规划避障路径；实机整批上传执行，模拟先规划后执行")
        self.map_status.setObjectName("HintLabel")
        ql.addWidget(self.map_status)
        lay.addWidget(quick)
        lay.addWidget(self._build_competition_controls())

        # 地图占满剩余高度，规划面板放到右侧固定宽度侧栏（不挤压地图）
        body = QHBoxLayout()
        body.setSpacing(10)
        left = QVBoxLayout()
        left.setSpacing(6)
        self.map_position = QLabel("场地位置：等待遥测（实测出发位置置零）")
        self.map_position.setWordWrap(True)
        self.map_position.setFixedHeight(44)
        left.addWidget(self.map_position)
        self.map_view = FieldView()
        self.map_view.set_navigation_map(self.nav_map)
        self.map_view.gotoRequested.connect(self._on_map_click)
        left.addWidget(self.map_view, 1)
        body.addLayout(left, 1)
        body.addWidget(self._build_plan_side())
        lay.addLayout(body, 1)
        return page

    def _turn_mode_changed(self):
        self._clear_path('转弯模式改变，请重新规划')
        self.nav_map['turn_mode']=self.turn_mode_combo.currentData()
        self.log('转弯模式：'+self.turn_mode_combo.currentText(),'info')

    def _build_competition_controls(self):
        panel = QFrame()
        panel.setObjectName('Panel')
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 8, 12, 8)
        row = QHBoxLayout()
        row.addWidget(QLabel('转弯模式'))
        self.turn_mode_combo=QComboBox()
        for label,value in (('行进中转向','MOVING'),('优先绕轮转向','WHEEL'),('到点后转向','STOP_TURN')):
            self.turn_mode_combo.addItem(label,value)
        self.turn_mode_combo.setCurrentIndex(max(0,self.turn_mode_combo.findData(self.nav_map.get('turn_mode','WHEEL'))))
        self.turn_mode_combo.setToolTip('切换会取消旧规划；停车点与作业朝向保持不变。到点后转向先停稳再原地转头。')
        self.turn_mode_combo.currentIndexChanged.connect(self._turn_mode_changed)
        row.addWidget(self.turn_mode_combo)
        row.addWidget(QLabel('完整初赛模拟'))
        row.addWidget(QLabel('模拟二维码'))
        self.competition_code = QLineEdit('156+123+516+231')
        self.competition_code.setMaxLength(15)
        self.competition_code.setFixedWidth(190)
        self.competition_code.setToolTip('四组任务码；到达二维码板后才模拟读取并显示。第二批暂存按同色第一层码垛。')
        self.competition_code.textEdited.connect(lambda _: self._clear_path('任务码改变，比赛模拟已取消'))
        row.addWidget(self.competition_code)
        start = QPushButton('一键比赛模拟')
        self.competition_start = start
        start.setToolTip('仅PC：保留地图上的模拟障碍，从地图配置的出发零点开始，整轮避障预检后自动完成任务。')
        start.clicked.connect(self._start_competition)
        row.addWidget(start)
        stop = QPushButton('停止比赛')
        stop.clicked.connect(lambda: self._stop_navigation('比赛模拟已取消'))
        row.addWidget(stop)
        export = QPushButton('导出比赛记录')
        export.clicked.connect(self._export_competition)
        row.addWidget(export)
        row.addStretch(1)
        layout.addLayout(row)
        real_row=QHBoxLayout()
        self.real_match_start=QPushButton('实机自动跑图：整批上传并运行')
        self.real_match_start.setToolTip('全部路径先缓存并校验，STM32本地执行；站点位置/航向/停稳满足后自动继续，全程无需作业确认。按当前地图规划，需有效OPS。')
        self.real_match_start.clicked.connect(self._start_real_match)
        real_row.addWidget(self.real_match_start)
        real_stop=QPushButton('停止实机轨迹')
        real_stop.clicked.connect(lambda:self._stop_navigation('实机比赛已取消'))
        real_row.addWidget(real_stop)
        real_export=QPushButton('导出实机批次')
        real_export.clicked.connect(self._export_real_match)
        real_row.addWidget(real_export)
        run_logs=QPushButton('打开运行日志')
        run_logs.clicked.connect(self._open_run_logs)
        real_row.addWidget(run_logs)
        self.run_log_check = QCheckBox('记录运行日志')
        self.run_log_check.setChecked(self.run_journal.enabled)
        self.run_log_check.setToolTip('实机和模拟共用；取消勾选立即结束当前日志，不影响车辆。重新勾选后从下一次运行开始记录。')
        self.run_log_check.toggled.connect(self._set_run_logging)
        real_row.addWidget(self.run_log_check)
        self.motion_pause_btn = QPushButton('暂停车辆')
        self.motion_pause_btn.clicked.connect(self._toggle_motion_pause)
        real_row.addWidget(self.motion_pause_btn)
        tuning = QPushButton('底盘调参')
        tuning.clicked.connect(lambda: self._select_page(self.page_names.index('底盘调参')))
        real_row.addWidget(tuning)
        self.real_collision_check = QCheckBox('实际车体碰撞保护')
        self.real_collision_check.setChecked(True)
        self.real_collision_check.setToolTip('控制实机运行时的地图车体/扫掠碰撞停车；关闭后不因截图中的地图碰撞原因中止批次。')
        self.real_collision_check.toggled.connect(self._real_collision_changed)
        real_row.addStretch(1)
        layout.addLayout(real_row)
        settings_row = QHBoxLayout()
        settings_row.addWidget(self.real_collision_check)
        settings_row.addStretch(1)
        self._build_work_points()
        self.work_points_button = QPushButton('作业点坐标设置…')
        self.work_points_button.setMinimumHeight(36)
        self.work_points_button.clicked.connect(self._show_work_points)
        settings_row.addWidget(self.work_points_button)
        layout.addLayout(settings_row)
        orientation_hint = QLabel('右侧塔吊：到站朝向作业区；无额外障碍时复用固定路线。')
        orientation_hint.setObjectName('HintLabel')
        layout.addWidget(orientation_hint)
        self.run_log_status=QLabel('运行日志：实机/模拟运行时自动保存，跑完或STOP后结束，下次运行另建日志。')
        self.run_log_status.setWordWrap(True)
        self.run_log_status.setObjectName('HintLabel')
        layout.addWidget(self.run_log_status)
        self.competition_status = QLabel('准备：点击地图放障碍或随机补齐；比赛沿用当前障碍。选择启停区和任务码后启动，每轮180秒。')
        self.competition_status.setWordWrap(True)
        self.competition_status.setObjectName('HintLabel')
        layout.addWidget(self.competition_status)
        self.zone_combo.currentIndexChanged.connect(self._nav_parameters_changed)
        return panel

    def _build_work_points(self):
        dialog = QDialog(self)
        self.work_points_dialog = dialog
        dialog.setObjectName('WorkPointDialog')
        dialog.setWindowTitle('作业点坐标设置')
        dialog.setMinimumSize(560, 360)
        dialog.resize(620, 520)
        outer = QVBoxLayout(dialog)
        outer.setContentsMargins(24, 20, 24, 20)
        outer.setSpacing(16)
        title = QLabel('作业点坐标')
        title.setObjectName('SectionTitle')
        outer.addWidget(title)
        hint = QLabel('使用地图上方的场地坐标：X 向左，Y 向前，单位 cm。\n关闭窗口不会应用未确认的修改。')
        hint.setObjectName('HintLabel')
        hint.setWordWrap(True)
        outer.addWidget(hint)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        box = QFrame()
        box.setObjectName('WorkPointTable')
        grid = QGridLayout(box)
        grid.setContentsMargins(12, 12, 12, 12)
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(12)
        grid.setSizeConstraint(QLayout.SetMinimumSize)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        self.work_point_spins = {}
        for col, text in enumerate(('作业点', 'X · 左右（cm）', 'Y · 前后（cm）')):
            label = QLabel(text)
            label.setObjectName('HintLabel')
            grid.addWidget(label, 0, col)
        for row, (key, label) in enumerate((('qr', '二维码'), ('raw', '原料区'),
                                            ('rough', '粗加工区'), ('storage', '暂存区')), 1):
            grid.addWidget(QLabel(label), row, 0)
            pair = []
            for col, axis in ((1, 'X'), (2, 'Y')):
                spin = QDoubleSpinBox()
                spin.setRange(-300, 300)
                spin.setDecimals(1)
                spin.setSingleStep(.5)
                spin.setSuffix(' cm')
                spin.setMinimumSize(150, 44)
                spin.setAccessibleName(label + ' ' + axis + ' 坐标，厘米')
                grid.addWidget(spin, row, col)
                pair.append(spin)
            self.work_point_spins[key] = pair
        scroll.setWidget(box)
        outer.addWidget(scroll, 1)
        self.work_point_status = QLabel('应用将使旧路线失效；作业朝向保持与设备边缘平行。')
        self.work_point_status.setWordWrap(True)
        self.work_point_status.setMinimumHeight(48)
        self.work_point_status.setObjectName('HintLabel')
        outer.addWidget(self.work_point_status)
        actions = QHBoxLayout()
        close = QPushButton('关闭')
        close.clicked.connect(dialog.reject)
        actions.addWidget(close)
        actions.addStretch(1)
        save = QPushButton('应用并保存地图…')
        save.clicked.connect(self._save_work_point_map)
        actions.addWidget(save)
        apply = QPushButton('应用坐标')
        apply.setObjectName('PrimaryButton')
        apply.clicked.connect(self._apply_work_points)
        actions.addWidget(apply)
        for button in (close, save, apply):
            button.setMinimumHeight(40)
            button.setAutoDefault(False)
            button.setDefault(False)
        outer.addLayout(actions)
        self._sync_work_points()
        return dialog

    def _show_work_points(self):
        dialog = self.work_points_dialog
        if not dialog.isVisible():
            self._sync_work_points()
            self.work_point_status.setText('应用将使旧路线失效；作业朝向保持与设备边缘平行。')
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def _sync_work_points(self):
        if not hasattr(self, 'work_point_spins'):
            return
        for key, pair in self.work_point_spins.items():
            point = self.nav_map.get('competition', {}).get('stations', {}).get(key)
            for spin in pair:
                spin.setEnabled(point is not None)
            if point is None:
                continue
            xy = core.layout_to_field(*point)
            for spin, value in zip(pair, xy):
                spin.setValue(value / 10)

    def _apply_work_points(self):
        from competition_simulation import collision_scene
        from work_orientation import work_heading
        try:
            data = copy.deepcopy(dict(self.nav_map))
            config = data['competition']
            updates = {}
            for key, pair in self.work_point_spins.items():
                field = [spin.value()*10 for spin in pair]
                target = list(core.field_to_layout(*field))
                old = config['stations'][key]
                config['lane_nodes'] = [target if list(node)==list(old) else node
                                        for node in config.get('lane_nodes', [])]
                config['stations'][key] = target
                updates[key] = [v/10 for v in field]
            scene = collision_scene(data, self.plan_pad_spin.value()*10, self.sim_obstacles)
            for key, point in config['stations'].items():
                heading = work_heading(config, key)
                if heading is not None:
                    reason = scene.pose_reason(*point, heading)
                    if reason:
                        raise ValueError(key + ' 停靠位置不可用：' + reason)
                elif not (0 <= point[0] <= 2400 and 0 <= point[1] <= 2400):
                    raise ValueError('二维码点超出场地')
            if config['stations'] == self.nav_map['competition']['stations']:
                self.work_point_status.setText('作业点未变化，保留当前路线。')
                return True
            data.setdefault('station_calibration', {})['field_coordinates_cm'] = updates
            data['map_version'] = int(data.get('map_version', 0)) + 1
            self.nav_map = data
            self.map_view.set_navigation_map(self.nav_map)
            self.work_point_status.setText('四个点已应用，旧路线已取消；请重新规划。需要下次使用时保存地图。')
            return True
        except (ValueError, KeyError, TypeError) as exc:
            self.work_point_status.setText('点位未应用：' + str(exc))
            return False

    def _save_work_point_map(self):
        if not self._apply_work_points():
            return
        path, _ = QFileDialog.getSaveFileName(self, '保存地图及作业点',
                                               str(BASE_DIR / 'navigation_map.json'), 'JSON (*.json)')
        if not path:
            return
        try:
            target = Path(path)
            temporary = target.with_name(target.name + '.tmp')
            temporary.write_text(json.dumps(dict(self.nav_map), ensure_ascii=False, indent=2), encoding='utf-8')
            temporary.replace(target)
            self.work_point_status.setText('地图及作业点已保存：' + path)
        except OSError as exc:
            self.work_point_status.setText('保存失败：' + str(exc))

    def _build_plan_side(self) -> QWidget:
        """地图右侧的 A* 规划侧栏（本阶段只计算与显示，不下发 STM32）。"""
        side = QFrame()
        side.setObjectName("Panel")
        side.setFixedWidth(316)
        sv = QVBoxLayout(side)
        sv.setContentsMargins(14, 12, 14, 12)
        sv.setSpacing(9)

        head = QHBoxLayout()
        head.setSpacing(8)
        title = QLabel("A* 路径规划")
        title.setObjectName("SectionTitle")
        head.addWidget(title)
        head.addStretch(1)
        badge = QLabel("整批轨迹执行")
        badge.setObjectName("PlanBadge")
        badge.setToolTip("实机点击规划成功后整批上传并执行；PC模拟先规划，再点击执行。未连接时仅预览。")
        head.addWidget(badge)
        sv.addLayout(head)

        hint = QLabel("点地图或快速目标 → 算出绕开禁区的航点路径")
        hint.setObjectName("HintLabel")
        hint.setWordWrap(True)
        sv.addWidget(hint)

        self.plan_click_check = QCheckBox("点击地图 = 规划路径")
        self.plan_click_check.setChecked(True)
        self.plan_click_check.setToolTip(
            "打开：实机点击或快速目标规划避障后整批上传并执行；PC模拟先规划后执行；\n"
            "取消：恢复「点击即下发 GOTO」。")
        self.plan_click_check.toggled.connect(self._on_plan_switch)
        sv.addWidget(self.plan_click_check)

        self.strafe_limit_check = QCheckBox("限制长距离横移")
        self.strafe_limit_check.setChecked(False)
        self.strafe_limit_check.setToolTip(
            "勾选后**一次连续横移**不得超过下面的「横移上限」，超了就改用原地转向+直行。\n"
            "连续横移被前进/后退/转向打断后重新计数，不是全程总量。\n"
            "地图 JSON 里的 strafe_polygons（操作区）内不受限。缺省不勾＝不限。")
        self.strafe_limit_check.toggled.connect(self._nav_parameters_changed)
        sv.addWidget(self.strafe_limit_check)

        params = QGridLayout()
        params.setHorizontalSpacing(8)
        params.setVerticalSpacing(6)
        params.addWidget(QLabel("网格"), 0, 0)
        self.plan_grid_spin = QDoubleSpinBox()
        self.plan_grid_spin.setRange(1.0, 50.0)
        self.plan_grid_spin.setDecimals(1)
        self.plan_grid_spin.setSuffix(" cm")
        self.plan_grid_spin.setValue(core.GRID_MM / core.OPS_CM_TO_MM)
        self.plan_grid_spin.setToolTip("10..500mm，整条边均检查碰撞；过粗可能找不到窄通路，不能减小安全外形强行通过。")
        params.addWidget(self.plan_grid_spin, 0, 1)
        params.addWidget(QLabel("额外裕量"), 1, 0)
        self.plan_pad_spin = QDoubleSpinBox()
        self.plan_pad_spin.setRange(0.0, 40.0)
        self.plan_pad_spin.setDecimals(1)
        self.plan_pad_spin.setSuffix(" cm")
        self.plan_pad_spin.setValue(core.NAV_MARGIN_MM / core.OPS_CM_TO_MM)
        self.plan_pad_spin.setToolTip(
            "随真实航向旋转的280×260mm矩形整车之外的额外安全裕量。\n"
            "0不代表忽略车体；真实尺寸、突出物和误差须实测。")
        self.plan_grid_spin.valueChanged.connect(self._nav_parameters_changed)
        self.plan_pad_spin.valueChanged.connect(self._nav_parameters_changed)
        self.map_yaw_combo.currentIndexChanged.connect(self._nav_parameters_changed)
        params.addWidget(self.plan_pad_spin, 1, 1)
        params.addWidget(QLabel("转向等效代价"), 2, 0)
        self.turn_penalty_spin = QDoubleSpinBox()
        self.turn_penalty_spin.setRange(0.0, 5000.0)
        self.turn_penalty_spin.setDecimals(0)
        self.turn_penalty_spin.setSingleStep(20.0)
        self.turn_penalty_spin.setSuffix(" mm")
        self.turn_penalty_spin.setValue(nav.DEFAULT_TURN_PENALTY_MM)
        self.turn_penalty_spin.setToolTip(
            "每次原地 90° 转向的等效代价（mm，180° 记两次）。调高＝尽量别转向；\n"
            "例如本版仿真行驶不执行路径中转向时，把它调高就能得到可执行的路线。")
        self.turn_penalty_spin.valueChanged.connect(self._nav_parameters_changed)
        params.addWidget(self.turn_penalty_spin, 2, 1)
        params.addWidget(QLabel("横移上限"), 3, 0)
        self.strafe_limit_spin = QDoubleSpinBox()
        self.strafe_limit_spin.setRange(0.0, 200.0)
        self.strafe_limit_spin.setDecimals(0)
        self.strafe_limit_spin.setSingleStep(10.0)
        self.strafe_limit_spin.setSuffix(" cm")
        self.strafe_limit_spin.setValue(core.STRAFE_RUN_LIMIT_MM / core.OPS_CM_TO_MM)
        self.strafe_limit_spin.setToolTip(
            "单次连续横移上限（0 = 完全禁横移）。\n"
            "完全禁横移要当心：启停区中心离两墙各 150mm，原地 90° 转向要 ≈208mm 净空，\n"
            "在角落里转不动，那样任何目标都会报「没有合法网格接入」。留 30cm 以上才有出库余量。")
        self.strafe_limit_spin.valueChanged.connect(self._nav_parameters_changed)
        params.addWidget(self.strafe_limit_spin, 3, 1)
        self.strafe_limit_spin.setEnabled(False)
        self.strafe_limit_check.toggled.connect(self.strafe_limit_spin.setEnabled)
        params.setColumnStretch(1, 1)
        sv.addLayout(params)
        self.plan_optimize_check=QCheckBox("充分优化点击路线（较慢）")
        self.plan_optimize_check.setToolTip("默认快速寻找通过完整车体运动预演的路线。勾选后在8秒预算内比较更多绕行和车头方案；整场比赛无额外障碍时复用固定路线。")
        self.plan_optimize_check.toggled.connect(self._nav_parameters_changed)
        sv.addWidget(self.plan_optimize_check)

        # 模拟障碍（φ50×100mm 圆柱，最多 4 个）：演示物体，只活在运行时，不进地图数据
        obs = QGridLayout()
        obs.setHorizontalSpacing(8)
        obs.setVerticalSpacing(6)
        self.obstacle_mode_check = QCheckBox("点击地图 = 放障碍")
        self.obstacle_mode_check.setToolTip(
            "打开后，点地图放下一个 φ50×100mm 模拟障碍（最多 %d 个），此时点击只放障碍、"
            "不规划也不下发。障碍只影响规划与仿真，不写进地图数据；关闭后点击恢复为规划。"
            % core.SIM_OBSTACLE_MAX)
        self.obstacle_mode_check.toggled.connect(self._on_obstacle_mode)
        obs.addWidget(self.obstacle_mode_check, 0, 0, 1, 2)
        rand = QPushButton("随机补齐")
        rand.setToolTip("避开固定禁区、已有障碍和车当前位置随机放置，直到放满 %d 个。"
                        % core.SIM_OBSTACLE_MAX)
        rand.clicked.connect(self._random_obstacles)
        obs.addWidget(rand, 1, 0)
        clr = QPushButton("清除障碍")
        clr.clicked.connect(lambda: self._clear_obstacles("已清除模拟障碍"))
        obs.addWidget(clr, 1, 1)
        self.obstacle_info = QLabel()
        self.obstacle_info.setObjectName("HintLabel")
        self.obstacle_info.setWordWrap(True)
        obs.addWidget(self.obstacle_info, 2, 0, 1, 2)
        sv.addLayout(obs)
        self._update_obstacle_info()

        row = QHBoxLayout()
        row.setSpacing(8)
        clear_path = QPushButton("清除路径")
        clear_path.clicked.connect(lambda: self._clear_path("已清除规划路径"))
        row.addWidget(clear_path)
        self.follow_btn = QPushButton("执行规划路径")
        self.follow_btn.setObjectName("PrimaryButton")
        self.follow_btn.setToolTip(
            "PC与实机关键坐标闭环；实机一次上传全部目标，PASS连续过点，站点停稳自动继续。\n"
            "中间点不停稳，只有最终STOP点要求位置+航向+停稳。\n"
            "实机按当前地图检查，需有效OPS及已使能底盘；STOP立即取消。")
        self.follow_btn.clicked.connect(self._toggle_follow)
        row.addWidget(self.follow_btn, 1)
        sv.addLayout(row)

        self.plan_info = QLabel("未规划")
        self.plan_info.setObjectName("PlanInfo")
        self.plan_info.setWordWrap(True)
        self.plan_info.setProperty("state", "idle")
        sv.addWidget(self.plan_info)
        legend = QLabel('黄色虚线：规划骨架（比赛含斜向接近）\n绿色箭头：行驶方向 · 紫色：参考车头 · 蓝色：实际轨迹')
        legend.setObjectName('HintLabel')
        legend.setWordWrap(True)
        sv.addWidget(legend)

        self.plan_text = QTextEdit()
        self.plan_text.setObjectName("PlanText")
        self.plan_text.setReadOnly(True)
        self.plan_text.setPlaceholderText("航点台账\n#1 场地(…)cm → GOTO=…")
        sv.addWidget(self.plan_text, 1)

        tools = QHBoxLayout()
        load_map = QPushButton("加载行驶区域")
        load_map.clicked.connect(self._load_navigation_map)
        export_plan = QPushButton("导出计划")
        export_plan.clicked.connect(self._export_navigation_plan)
        tools.addWidget(load_map)
        tools.addWidget(export_plan)
        sv.addLayout(tools)
        export_trajectory = QPushButton("导出Trajectory JSON")
        export_trajectory.setToolTip("导出20mm连续采样、切线航向及累计弧长；完整整车复检通过才可导出。")
        export_trajectory.clicked.connect(self._export_trajectory)
        sv.addWidget(export_trajectory)
        export_segments = QPushButton("导出PC路径 JSON")
        export_segments.setToolTip("PC模拟关键坐标程序：位置、车头、PASS/STOP和闭环参数；不下发实机。")
        export_segments.clicked.connect(self._export_segments)
        sv.addWidget(export_segments)
        note = QLabel("默认地图仅演示，未核实比赛灰色车道。连续轨迹整车校验不代表实车放行。\n"
                      "原料区旧目标太近时会拒绝；可先点击布局(1200,2100)演示接近点。\n"
                      "取消规划勾选会进入原始直接GOTO调试，须人工检查。")
        note.setObjectName("HintLabel")
        note.setWordWrap(True)
        sv.addWidget(note)
        # 完整比赛栏占用高度后，规划控件保持可读，通过滚动查看而不压扁输入框。
        side.setMinimumHeight(max(850, side.minimumSizeHint().height()))
        scroll = QScrollArea()
        scroll.setFixedWidth(332)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(side)
        return scroll

    def _build_stepper_page(self):
        page, lay = self._page_shell("28 / 35 步进电机", "升降与伸缩独立调试 · 编辑参数后点击执行")
        page.setObjectName("StepperPage")
        notice = QFrame()
        notice.setObjectName("StepperNotice")
        notice_layout = QVBoxLayout(notice)
        notice_layout.setContentsMargins(14, 10, 14, 10)
        notice_text = QLabel("先读取状态，再使能并执行目标。取消待发只撤销未发送指令；顶部停止仅作用于底盘 / DM。")
        notice_text.setWordWrap(True)
        notice_layout.addWidget(notice_text)
        lay.addWidget(notice)

        body = QWidget()
        body.setObjectName("StepperBody")
        body_layout = QVBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 4, 0)
        body_layout.setSpacing(16)
        self.stepper_widgets, self.stepper_status = {}, {}
        self.stepper_card_grid = StepperCardGrid([
            self._build_stepper_card(motor, config)
            for motor, config in core.STEPPER_CONFIG.items()])
        body_layout.addWidget(self.stepper_card_grid)

        details = QFrame()
        details.setObjectName("StepperCard")
        detail_layout = QVBoxLayout(details)
        detail_layout.setContentsMargins(16, 12, 16, 12)
        toggle = QPushButton("＋ 通讯与机械标定说明")
        toggle.setObjectName("StepperDisclosure")
        toggle.setCheckable(True)
        detail_layout.addWidget(toggle)
        note = QLabel("通讯：STM32 CAN2 · 1 Mbps；35 节点地址 1、28 节点地址 2，电机须设 CAN1_MAP。\n"
                      "位置协议：X 固件角度模式，S_PosTDP 须为 Disable。原始角度单位为 0.1°，驱动回复显示在卡片和日志中。\n"
                      "机械换算沿用原车标定：35 高度 43–203 mm；28 半径 120–286 mm，半径不是伸出量。\n"
                      "上述范围需要按当前机构核对。28/35 尚无已验证停机协议，取消待发不会停止已经启动的运动。")
        note.setObjectName("StepperHint")
        note.setWordWrap(True)
        note.setVisible(False)
        detail_layout.addWidget(note)
        toggle.toggled.connect(lambda opened: (
            note.setVisible(opened), toggle.setText(("－" if opened else "＋") + " 通讯与机械标定说明")))
        body_layout.addWidget(details)
        body_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setObjectName("StepperScroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(body)
        lay.addWidget(scroll, 1)
        return page

    def _build_stepper_card(self, motor, config):
        title, label, lo, hi, default, vlo, vhi, can_id = config
        panel = QFrame()
        panel.setObjectName("StepperCard")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)
        header = QHBoxLayout()
        header.setSpacing(12)
        badge = QLabel(str(motor))
        badge.setObjectName("StepperNumber")
        badge.setAlignment(Qt.AlignCenter)
        badge.setFixedSize(48, 48)
        header.addWidget(badge)
        heading = QVBoxLayout()
        heading.setSpacing(3)
        name = QLabel(title.split(" ", 1)[-1])
        name.setObjectName("StepperTitle")
        heading.addWidget(name)
        address = QLabel("CAN 0x%03X · 节点 %d" % (can_id, 1 if motor == 35 else 2))
        address.setObjectName("StepperHint")
        heading.addWidget(address)
        header.addLayout(heading, 1)
        layout.addLayout(header)

        status = QLabel("尚未读取驱动状态，请先点击读取状态。")
        status.setObjectName("StepperFeedback")
        status.setWordWrap(True)
        status.setMinimumHeight(56)
        layout.addWidget(status)
        self.stepper_status[motor] = status
        diagnostics = QHBoxLayout()
        diagnostics.setSpacing(8)
        query = QPushButton("读取状态（不运动）")
        query.clicked.connect(lambda _=False, m=motor: self.send_line("S%dSTATUS" % m))
        enable = QPushButton("使能电机（锁轴）")
        enable.setObjectName("SuccessButton")
        enable.clicked.connect(lambda _=False, m=motor: self.send_line("S%dEN" % m))
        diagnostics.addWidget(query, 1)
        diagnostics.addWidget(enable, 1)
        layout.addLayout(diagnostics)

        section = QLabel("机械位置目标")
        section.setObjectName("StepperSection")
        layout.addWidget(section)
        fields = QGridLayout()
        fields.setHorizontalSpacing(12)
        fields.setVerticalSpacing(6)
        position = QDoubleSpinBox()
        position.setObjectName("StepperTarget")
        position.setRange(lo, hi)
        position.setDecimals(1)
        position.setSuffix(" mm")
        position.setValue(default)
        position.setMinimumHeight(48)
        speed = QSpinBox()
        speed.setRange(vlo, vhi)
        speed.setSuffix(" mm/s")
        speed.setValue(10)
        speed.setMinimumHeight(48)
        fields.addWidget(QLabel(label), 0, 0)
        fields.addWidget(QLabel("线速度"), 0, 1)
        fields.addWidget(position, 1, 0)
        fields.addWidget(speed, 1, 1)
        fields.setColumnStretch(0, 3)
        fields.setColumnStretch(1, 2)
        layout.addLayout(fields)
        hint = QLabel("原标定范围 %.0f–%.0f mm · 默认 %.0f mm 为原标定零点%s" %
                      (lo, hi, default, "；半径不是伸出量" if motor == 28 else ""))
        hint.setObjectName("StepperHint")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        move = QPushButton("执行机械目标")
        move.setObjectName("PrimaryButton")
        move.setMinimumHeight(44)
        move.clicked.connect(lambda _=False, m=motor: self._send_stepper(m, False))
        layout.addWidget(move)
        actions = QHBoxLayout()
        actions.setSpacing(8)
        home = QPushButton("执行电机回零")
        home.setObjectName("WarningButton")
        home.clicked.connect(lambda _=False, m=motor: self.send_line("S%dHOME" % m))
        cancel = QPushButton("取消待发（不停车）")
        cancel.setToolTip("只取消尚未提交的指令，不会停止已启动运动")
        cancel.clicked.connect(lambda _=False, m=motor: self.send_line("S%dCANCEL" % m))
        actions.addWidget(home, 1)
        actions.addWidget(cancel, 1)
        layout.addLayout(actions)

        raw_toggle = QPushButton("＋ 原始角度调试")
        raw_toggle.setObjectName("StepperDisclosure")
        raw_toggle.setCheckable(True)
        layout.addWidget(raw_toggle)
        raw_body = QWidget()
        raw_body.setObjectName("StepperRawBody")
        raw_layout = QGridLayout(raw_body)
        raw_layout.setContentsMargins(0, 4, 0, 0)
        raw_layout.setHorizontalSpacing(12)
        raw_layout.setVerticalSpacing(8)
        direction = QComboBox()
        direction.addItems(["方向 0", "方向 1"])
        steps = QDoubleSpinBox()
        steps.setDecimals(0)
        steps.setRange(0, 4294967295)
        steps.setSuffix(" ×0.1°")
        rpm = QSpinBox()
        rpm.setRange(1, 3000)
        rpm.setValue(10)
        rpm.setSuffix(" RPM")
        for row, (text, control) in enumerate((("绝对位置角度", steps), ("方向", direction), ("电机转速", rpm))):
            raw_layout.addWidget(QLabel(text), row, 0)
            raw_layout.addWidget(control, row, 1)
        raw_layout.setColumnStretch(1, 1)
        raw = QPushButton("执行原始绝对位置")
        raw.clicked.connect(lambda _=False, m=motor: self._send_stepper(m, True))
        raw_layout.addWidget(raw, 3, 0, 1, 2)
        layout.addWidget(raw_body)
        raw_body.setVisible(False)
        raw_toggle.toggled.connect(lambda opened, body=raw_body, button=raw_toggle: (
            body.setVisible(opened), button.setText(("－" if opened else "＋") + " 原始角度调试")))
        self.stepper_widgets[motor] = (position, speed, direction, steps, rpm)
        return panel

    def _send_stepper(self, motor, raw):
        position, speed, direction, steps, rpm = self.stepper_widgets[motor]
        if raw:
            command = "S%dRAW=%d,%d,%d" % (motor, direction.currentIndex(), int(steps.value()), rpm.value())
        else:
            command = core.stepper_move_command(motor, position.value(), speed.value())
        self.send_line(command)

    def _build_chassis_page(self):
        page, lay = self._page_shell("底盘调参", "PC与实机关键坐标、单点GOTO共用七项底盘参数")

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
        refresh = QPushButton("重新回读并填入")
        refresh.clicked.connect(self._refresh_chassis_inputs)
        top_row.addWidget(refresh)
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
        for spin, value in ((self.ops_offset_x, 53.0), (self.ops_offset_y, -39.5)):
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
        self.ops_offset_status = QLabel("上位机初始值：X=左53 / Y=后39.5 mm。点击应用才下发；保存到电脑仅保存参数文件。")
        self.ops_offset_status.setWordWrap(True)
        cg.addWidget(self.ops_offset_status, 3, 0, 1, 4)
        self.ops_drift = QLabel("补偿后位置：等待遥测；置零后原地旋转，观察 X(左右)/Y(前后) 是否接近0。")
        self.ops_drift.setWordWrap(True)
        cg.addWidget(self.ops_drift, 4, 0, 1, 4)
        self.ops_offset_hint = QLabel("下发顺序与坐标定义一致：OPSOFFSET=X(左+),Y(前+)；左53/后39.5 对应 OPSOFFSET=53.0,-39.5。")
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
        note = QLabel('编辑仅填输入；点击发送并确认回读后生效。新PC坐标路径按已生效参数预演，发送参数会取消旧模拟路径，请重新规划。')
        note.setWordWrap(True)
        bl.addWidget(note)
        for cmd, label, lo, hi, dflt, rb in core.CHASSIS_PARAMS:
            row = ParamRow(cmd, label, lo, hi, dflt, rb, False)
            row.readback_driven = True
            row.reset_readback()

            def _send_param(c, v):
                # 写入后立刻回读一次：XVMIN/ZVMIN 没有遥测通道，只能靠文字应答确认。
                self.send_line("%s=%s" % (c, v))
                self._request_param_readback(c)

            row.sendRequested.connect(_send_param)
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
        self.ops_offset_x.setValue(53.0)
        self.ops_offset_y.setValue(-39.5)
        self.ops_offset_status.setText("已填入 X=左53 / Y=后39.5 mm；点击应用才下发。")

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
        if (self.follow is not None or self.planned_points or self._plan_future is not None or
                self._direct_home is not None):
            self._clear_path("手动控制接管，旧规划失效")
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
            if any((left, forward, turn)):
                self._home_after_stop = False
                if self.run_journal.active_run is None:
                    self._journal_begin('MANUAL', {'vector':self.manual_vector})
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
        controls.setObjectName("DmToolbar")
        toolbar = QVBoxLayout(controls)
        toolbar.setContentsMargins(14, 12, 14, 12)
        toolbar.setSpacing(10)
        config = QHBoxLayout()
        config.setSpacing(12)
        self.dm_rows = {"DMID": MotorAddressRow()}
        self.dm_rows["DMID"].sendRequested.connect(lambda c, v: self.send_line("%s=%s" % (c, v)))
        config.addWidget(self.dm_rows["DMID"])
        config.addStretch(1)
        config.addWidget(QLabel("控制模式"))
        self.dm_mode_combo = QComboBox()
        self.dm_mode_combo.addItem("MIT 模式", 1)
        self.dm_mode_combo.addItem("位置速度模式", 2)
        self.dm_mode_combo.setMinimumWidth(154)
        self.dm_mode_combo.currentIndexChanged.connect(self._dm_mode_changed)
        config.addWidget(self.dm_mode_combo)
        apply_mode = QPushButton("应用模式")
        apply_mode.setToolTip("先失能再应用；模式选择只切换界面，不自动使能")
        apply_mode.clicked.connect(lambda: self.dm_position_panel.apply_mode(
            int(self.dm_mode_combo.currentData())))
        config.addWidget(apply_mode)
        toolbar.addLayout(config)
        cl = QHBoxLayout()
        cl.setSpacing(10)
        en = QPushButton("▶ 使能 DMEN")
        en.setObjectName("SuccessButton")
        en.clicked.connect(lambda: self.dm_position_panel.enable())
        off = QPushButton("■ 失能 DMOFF")
        off.setObjectName("EmergencyButton")
        off.clicked.connect(lambda: self.send_line("DMOFF"))
        zero = QPushButton("⊙ 零点 DMZERO")
        zero.setObjectName("WarningButton")
        zero.clicked.connect(lambda: self.send_line("DMZERO"))
        cl.addWidget(en)
        cl.addWidget(off)
        cl.addWidget(zero)
        cl.addStretch(1)
        self.dm_mode_note = QLabel("MIT · Kp / Kd / 前馈力矩")
        self.dm_mode_note.setObjectName("HintLabel")
        self.dm_mode_note.setMinimumWidth(300)
        self.dm_mode_note.setWordWrap(True)
        cl.addWidget(self.dm_mode_note)
        toolbar.addLayout(cl)
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
        self.dm_status_big.setProperty("enabled", False)
        lay.addWidget(self.dm_status_big)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        body = QWidget()
        bl = QVBoxLayout(body)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(7)
        self.dm_mode_stack = QStackedWidget()
        mit_page = QWidget()
        mit_layout = QVBoxLayout(mit_page)
        mit_layout.setContentsMargins(0, 0, 0, 0)
        mit_hint = QLabel("MIT 力矩 = Kp × 位置误差 + Kd × 速度误差 + 前馈力矩；参数发送后生效。")
        mit_hint.setObjectName("HintLabel")
        mit_layout.addWidget(mit_hint)
        for cmd, label, lo, hi, dflt, rb, is_int in core.DM_PARAMS:
            if cmd == "DMID":
                continue
            row = ParamRow(cmd, label, lo, hi, dflt, rb, is_int)
            row.sendRequested.connect(lambda c, v: self.send_line("%s=%s" % (c, v)))
            self.dm_rows[cmd] = row
            mit_layout.addWidget(row)
        mit_layout.addStretch(1)
        self.dm_mode_stack.addWidget(mit_page)
        from dm_panel import PositionPanel
        self.dm_position_panel = PositionPanel(self, BASE_DIR / "dm_positions.json")
        self.dm_position_panel.commandRequested.connect(self.send_line)
        self.dm_mode_stack.addWidget(self.dm_position_panel)
        bl.addWidget(self.dm_mode_stack, 1)
        scroll.setWidget(body)
        lay.addWidget(scroll, 1)
        return page

    def _build_vision_page(self):
        page, lay = self._page_shell("视觉跟踪", "协议 V1.1 物料精对准：启停跟踪与增益调参（会驱动底盘）")

        controls = QFrame()
        controls.setObjectName("Panel")
        cl = QHBoxLayout(controls)
        cl.setContentsMargins(12, 10, 12, 10)
        cl.addWidget(QLabel("按颜色启动"))
        for color, name in core.VISION_COLORS:
            btn = QPushButton("%s %d" % (name, color))
            btn.setObjectName("SuccessButton")
            btn.setToolTip("VTRACK=%d：让固件经 USART3 请求工控机跟踪%s物料并接管底盘" % (color, name))
            # 默认参数绑定：循环里直接闭包会全部取到最后一次迭代的值。
            btn.clicked.connect(lambda _=False, c=color, n=name: self._start_vtrack(c, n))
            cl.addWidget(btn)
        cl.addSpacing(18)
        stop = QPushButton("■ 停止跟踪 VTRACK=0")
        stop.setObjectName("EmergencyButton")
        stop.clicked.connect(self._stop_vtrack)
        cl.addWidget(stop)
        refresh = QPushButton("↻ 刷新回读")
        refresh.setToolTip("逐个 GET 这 6 个参数：它们没有遥测通道，只能靠文字应答")
        refresh.clicked.connect(self._refresh_vision_readback)
        cl.addWidget(refresh)
        cl.addStretch(1)
        lay.addWidget(controls)

        self.vision_state = QLabel("视觉状态：未跟踪")
        self.vision_state.setObjectName("DmStatus")
        lay.addWidget(self.vision_state)

        note = QLabel(
            "· 启动前需四轮已使能，否则固件回 ERR WHEEL DISABLED / ERR VTRACK BUSY；"
            "启动会取消 GOTO 与键盘手动并停车，STOP 也会自动停止跟踪（无需再发 VTRACK=0）。\n"
            "· 这 6 个参数是「RAM 参数」：不存 Flash，掉电回到固件默认值（VDBMM=2mm、VMIN=8、VMAX=60）。\n"
            "· 死区与最小速度必须配对调：2mm 死区配 8RPM（约 33.6mm/s）时一个控制周期就能冲过死区，"
            "目标附近容易出现「停→起→停」来回抖；真出现就先降 VMIN，不要只把死区放大。\n"
            "· 毫米模式（VDBMM/VKPMM）与像素模式（VDBPX）各用一套系数，由工控机下发的 COORD_MODE 决定；"
            "像素模式无法表达「2mm」——固件里没有毫米/像素标定系数。\n"
            "· 跟踪中新结果超过 150ms 未到即停车；工控机不回包或丢目标时只停底盘，不会回停止帧，"
            "所以那种停法在工控机侧看不到。\n"
            "· 已知缺口：没有「启动确认超时」。工控机若在跟踪期间重启，固件不会自动重发请求，"
            "需手动 VTRACK=0 再 VTRACK=1（换新序号）才能恢复。")
        note.setWordWrap(True)
        note.setObjectName("HintLabel")
        lay.addWidget(note)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        body = QWidget()
        bl = QVBoxLayout(body)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(7)
        self.vision_rows = {}
        for cmd, label, lo, hi, dflt, rb in core.VISION_PARAMS:
            row = ParamRow(cmd, label, lo, hi, dflt, rb, False)

            def _send_vision_param(c, v):
                # 与底盘页同款：写入后立刻回读一次，这些参数没有波形可以对照。
                self.send_line("%s=%s" % (c, v))
                self._request_param_readback(c)

            row.sendRequested.connect(_send_vision_param)
            self.vision_rows[cmd] = row
            bl.addWidget(row)
        bl.addStretch(1)
        scroll.setWidget(body)
        lay.addWidget(scroll, 1)
        return page

    def _start_vtrack(self, color: int, name: str):
        if self.wheel_state is False:
            self.log("四轮已请求失能：固件会拒绝 VTRACK，请先「使能电机（锁轴）」", "warn")
        self.send_line(core.vtrack_command(color))
        self.vision_state.setText("视觉状态：已请求跟踪%s · 等固件 ACK" % name)

    def _stop_vtrack(self):
        self.send_line(core.vtrack_command(0))
        self.vision_state.setText("视觉状态：已请求停止跟踪")

    def _refresh_vision_readback(self):
        """逐个 GET 视觉参数：24 通道遥测里没有视觉位，只有文字应答这一条路。

        刻意不并入 1Hz 的 _poll_param_readback：那会把无通道参数的轮询周期从 2s
        拉到 8s，每条应答还要占一个遥测帧位；这里改为进页面或手动点按钮时批量刷新。
        """
        for cmd, _label, _lo, _hi, _dflt, _rb in core.VISION_PARAMS:
            self._request_param_readback(cmd)

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
        self.command_entry.setPlaceholderText(
            "例如：KPX=3.0 / GOTO=100.0,60.0,90.0 / GOTOHOLD=100.0,60.0,90.0 / DMEN / STOP")
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
        # 视觉参数没有遥测通道（24 通道里没有视觉位），进入该页时刷新一次回读，
        # 否则那 6 行会长期停在"回读 —"。getattr 兜底：无 GUI 套件用 WindowMethods 桩。
        if idx == getattr(self, "vision_page_index", -1):
            self._refresh_vision_readback()

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
        if hasattr(self, "dm_position_panel"):
            self.dm_position_panel.session_reset()
        self._clear_path()
        self.latest = None
        self._map_pose_filter = core.MapPoseFilter()
        self._display_source = self._display_pose = None
        self._display_pose_ok = True
        self.latest_received_monotonic = 0.0
        self._home_after_stop = False
        self._drain_queue(self.frame_q)
        self._manual_stop()
        self._drain_queue(self.line_q)
        self._drain_queue(self.urgent_q)
        self._drain_queue(self.param_q)
        for row in self.chassis_rows.values():
            row.reset_readback()
        for status in self.stepper_status.values():
            status.setText("本次连接尚未读取驱动状态。")

    def toggle_connect(self, force_on=None):
        want = force_on if force_on is not None else self.worker is None
        if want and self.worker is None:
            if not self._previous_worker_finished():
                self.log("串口仍在关闭，请稍后连接。", "warn")
                return
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
            worker = core.SerialWorker(
                port, baud, self.frame_q, self.line_q, self.urgent_q,
                text_q=self.fw_text_q,
                param_q=self.param_q,
                link_mode=self.link_mode_combo.currentData(),
            )
            worker.err_cb = lambda msg, source=worker: self.bridge.workerError.emit(source, msg)
            self.worker = worker
            self._ensure_run_journal()
            self.worker.start()
            self.connect_btn.setText("断开")
            self._set_link_status("正在打开 %s…" % port, "warn")
            self.log("正在连接 %s @ %d 8N1" % (port, baud), "info")
        elif not want and self.worker is not None:
            self._manual_stop()
            self._stop_follow("已断开串口，停止沿路径行驶")
            self._stop_worker(safe=True)
            self._clear_command_queues()
            self.connect_btn.setText("连接")
            self._set_link_status("未连接", "idle")
            self.log("串口已请求安全断开（STOP/DMSTOP/DMOFF）", "info")

    def _previous_worker_finished(self):
        if self._closing_worker is not None:
            if self._closing_worker.is_alive():
                return False
            self._closing_worker.join()
            self._closing_worker = None
            self.run_journal.close()
            self._journal_source = None
            self._drain_queue(self.frame_q)
            self._drain_queue(self.fw_text_q)
        return True

    def _stop_worker(self, safe):
        self._clear_path()
        old = self.worker
        self.worker = None
        self._closing_worker = old
        old.request_stop(safe=safe)
        # 有界等待；驱动异常阻塞时保留引用，禁止开启下一会话。
        old.join(timeout=0.5)
        self._previous_worker_finished()

    def toggle_sim(self, force_on=None):
        want = force_on if force_on is not None else self.sim is None
        if want and self.sim is None:
            if self.worker is not None:
                self.toggle_connect(False)
            if not self._previous_worker_finished():
                self.log("串口仍在关闭，请稍后开启模拟。", "warn")
                return
            self._clear_command_queues()
            self.sim = core.Simulator(self.frame_q, self.line_q, self.urgent_q,
                                      param_q=self.param_q, text_q=self.fw_text_q)
            self._ensure_run_journal()
            # Static start at the selected zero; no uncontrolled demonstration orbit.
            self.sim.handle_line("ZERO")
            # OPS ZERO的0°是其前向基准；选区校准同时设置场地方向，不能只改显示标签。
            self._set_start_zone()
            self.sim.handle_line('ZERO')
            self._clear_command_queues()
            self.sim.start()
            self.sim_btn.setText("停止模拟")
            self._set_link_status("模拟模式 · 50 Hz", "sim")
            self.log("模拟模式已开启。", "info")
        elif not want and self.sim is not None:
            self._stop_follow("已关闭模拟，停止沿路径行驶")
            old_sim = self.sim
            old_sim.cancel_navigation()
            old_sim.stop_flag = True
            if old_sim.is_alive():
                old_sim.join(timeout=0.5)
            self.sim = None
            self.run_journal.close()
            self._journal_source = None
            self._clear_command_queues()
            self.sim_btn.setText("模拟")
            self._set_link_status("未连接", "idle")
            self.log("模拟模式已关闭。", "info")

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.setText("继续波形" if self.paused else "暂停波形")
        self.log("波形缓冲已%s" % ("暂停" if self.paused else "继续"), "info")

    def send_line(self, text: str):
        text = str(text).strip()
        if not text:
            return
        if self.worker is None and self.sim is None:
            self.log("未连接，命令未发送：%s" % text, "warn")
            return
        cmd = text.upper().split("=", 1)[0].strip()
        zero_reason = self._ops_zero_block_reason()
        if zero_reason and cmd in ('GOTO','GOTOHOLD','MANUAL','ZDT','VTRACK') and text.upper().replace(' ','') not in ('MANUAL=0,0,0','VTRACK=0'):
            self.log(zero_reason, 'warn')
            return
        if cmd in ('ZERO','OPSOFFSET') and self._uses_ops_origin():
            self._poll_ops_zero()
            if getattr(self, '_ops_zero_pending', None) is not None:
                self.log('正在等待置零回读，请勿重复置零', 'warn')
                return
            self._begin_ops_zero(text.upper())
        # 保留终端旧轨迹协议的写入门禁，不再依赖已移除的调参页面。
        if cmd in core.TRAJECTORY_NAMES:
            if self.sim is not None or self.worker is None or not self.worker.opened.is_set():
                self.log('旧轨迹参数仅可发送到已连接的STM32', 'warn')
                return
            if self._real_future is not None or (self.real_match is not None and self.real_match.active):
                self.log('实机批次准备/运行中，请先STOP再修改旧轨迹参数', 'warn')
                return
        if self.worker is not None and cmd in core.CHASSIS_NAMES and '=' in text:
            if self._real_future is not None or (self.real_match is not None and self.real_match.active):
                self.log('实机坐标批次准备/运行中，请先STOP再修改底盘参数', 'warn')
                return
            self._clear_path('底盘参数修改，旧坐标预演失效')
        if self.worker is None and self.sim is not None and cmd in core.CHASSIS_NAMES and '=' in text:
            self._clear_path('底盘参数已发送，旧坐标预演失效；请重新规划')
        if hasattr(self, "dm_position_panel"):
            if self.dm_position_panel.observe_command(text) is False:
                return
            if cmd == "DMMODE" and self.dm_position_panel.requested_mode in (1, 2):
                self.dm_mode_combo.blockSignals(True)
                self.dm_mode_combo.setCurrentIndex(self.dm_position_panel.requested_mode-1)
                self.dm_mode_combo.blockSignals(False)
                self.dm_mode_stack.setCurrentIndex(self.dm_position_panel.requested_mode-1)
                self.dm_mode_note.setText("模式请求已入队；尚无电机模式寄存器回读")
        # 锁轴切换前先停止键盘续发，避免失能后仍周期发送MANUAL。
        # VTRACK 也必须停：固件接到跟踪请求时会清掉 manual/goto 并停车，但上位机
        # 这边 20Hz 的 MANUAL 流不会自己停，会把固件立刻拉回手动、与视觉速度互相打架。
        if cmd in ("STOP", "ZERO", "GOTO", "GOTOHOLD", "OPSOFFSET", "WHEELEN", "WHEELOFF",
                   "VTRACK"):
            self._manual_stop()
        # Every control/frame change invalidates navigation, including console commands.
        # _clear_path uses simulator cancellation directly; it never recursively sends STOP.
        if cmd in ("STOP", "ZERO", "OPSOFFSET", "WHEELEN", "WHEELOFF", "GOTO", "GOTOHOLD", "MANUAL", "ZDT", "VTRACK"):
            self._clear_path("%s：旧规划已失效" % cmd)
        if cmd in ('STOP', 'WHEELOFF'):
            self._home_after_stop = True
        elif cmd in ('GOTO', 'GOTOHOLD', 'MANUAL', 'ZDT', 'VTRACK'):
            self._home_after_stop = False
        self._ensure_run_journal()
        manual_zero = text.upper().replace(' ','')=='MANUAL=0,0,0'
        if cmd in ('GOTO','GOTOHOLD','MANUAL','ZDT','VTRACK') and not manual_zero and self.run_journal.active_run is None:
            self._journal_begin(cmd, {'command':text})
        self.run_journal.emit('COMMAND_QUEUED', command=text)
        if cmd in ('STOP','ZERO','WHEELOFF','OPSOFFSET') or manual_zero:
            self.run_journal.end_run('STOP_REQUESTED', text)
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
        if cmd in ('ZERO', 'OPSOFFSET') and getattr(self, '_ops_zero_pending', None) is not None:
            self.map_status.setText('置零请求已入队；等待发送后连续三帧 OPS 零位姿，三处实测点保持不变')
        if cmd.startswith(("S28", "S35")):
            motor = int(cmd[1:3])
            self.stepper_status[motor].setText(
                "模拟模式不提供电机硬件反馈。" if self.sim is not None else
                "请求已入队，等待STM32诊断回复（需使用配套新版固件）。")

    def _enqueue_heartbeat(self):
        now = time.monotonic()
        if (
            self.worker is not None
            and self.worker.opened.is_set()
            and now - self.last_heartbeat_enqueue >= core.HEARTBEAT_INTERVAL_S
        ):
            self.urgent_q.put("PING")
            self.last_heartbeat_enqueue = now

    def _on_worker_error(self, source, msg: str):
        if source is not self.worker:
            return
        self._manual_stop()
        self.log(msg, "warn")
        if self.worker is not None:
            self._stop_worker(safe=False)
        self.connect_btn.setText("连接")
        self._set_link_status("串口异常", "error")
        QMessageBox.critical(self, "串口错误", msg)

    # ---------------- 数据刷新 ----------------
    def _start_timers(self):
        self.data_timer = QTimer(self)
        self.data_timer.setTimerType(Qt.PreciseTimer)
        self.data_timer.timeout.connect(self._process_frames)
        self.data_timer.start(20)

        self.render_timer = QTimer(self)
        self.render_timer.setTimerType(Qt.PreciseTimer)
        self.render_timer.timeout.connect(self._render_ui)
        self.render_timer.start(20)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._status_tick)
        self.status_timer.start(250)

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.timeout.connect(self._enqueue_heartbeat)
        self.heartbeat_timer.start(100)

        self.param_timer = QTimer(self)
        self.param_timer.timeout.connect(self._poll_param_readback)
        self.param_timer.start(int(core.PARAM_POLL_S * 1000))

        # 仿真沿路径行驶的推进定时器（仅模拟模式会启动）
        self.follow_timer = QTimer(self)
        self.follow_timer.timeout.connect(self._follow_step)
        self.follow_timer.setInterval(20)

        self.plan_poll_timer = QTimer(self)
        self.plan_poll_timer.timeout.connect(self._poll_plan)
        self.plan_poll_timer.start(50)
        self.competition_timer = QTimer(self)
        self.competition_timer.timeout.connect(self._poll_competition)
        self.competition_timer.start(20)
        self.real_match_timer=QTimer(self)
        self.real_match_timer.timeout.connect(self._poll_real_match)
        self.real_match_timer.start(50)

    def _param_rows(self):
        """命令名 -> 参数行控件（底盘页、DM 页与视觉页合并）。"""
        rows = dict(self.chassis_rows)
        rows.update(self.dm_rows)
        rows.update(self.vision_rows)
        return rows

    def _apply_param_readback(self, name, value):
        """固件文字回读刷新界面：没有遥测通道的参数只有这一条路。

        界面没有对应参数行时，必须把这一行**打到日志**再返回。参数回读行在
        FrameParser 里就被分流出了日志通道（PARAM_ECHO_RE → param_q），如果这里
        再静默 return，`GET XXX` 就完全没有可见反馈，看起来像"板子没反应"——
        VDBMM 这类未注册到界面的参数曾经就是这样被整个吞掉的。
        """
        pending = getattr(self, '_coordinate_read', None)
        if pending is not None and str(name).upper() in pending['sent']:
            try:
                number = float(value)
                if math.isfinite(number):
                    pending['values'][core.SIM_PARAM_ATTRS[str(name).upper()]] = number
            except (TypeError, ValueError):
                pass
        row = self._param_rows().get(str(name).upper())
        if row is None:
            self.log("回读 %s=%s（界面无该参数行）" % (name, value), "info")
            return
        try:
            row.set_readback(float(value))
        except (TypeError, ValueError):
            return

    def _request_param_readback(self, name):
        """只发 GET，不走 send_line：轮询不该刷 TX 日志或触发停车路径。"""
        if self.worker is None and self.sim is None:
            return
        self.line_q.put(core.param_query(name))

    def _poll_param_readback(self):
        """慢速兜底回读：轮询没有遥测通道的可调参数（当前是 XVMIN/ZVMIN）。

        每次只发一条 GET，文字应答每帧最多占用一个遥测帧位；写入后的即时回读
        由参数行的"发送"负责，这里只保证界面长期与实际 RAM 值一致。
        """
        if getattr(self,'_coordinate_read',None) is not None:
            return  # 发车前快照自行重试，暂停后台GET，避免争用回复队列。
        names = [cmd for cmd, _label, _lo, _hi, _dflt, rb in core.CHASSIS_PARAMS if rb is None]
        if not names:
            return
        name = names[self.param_poll_index % len(names)]
        self.param_poll_index += 1
        self._request_param_readback(name)

    def _process_frames(self):
        self._poll_ops_zero()
        warning=self.run_journal.error or ('日志队列丢失%d条记录'%self.run_journal.dropped if self.run_journal.dropped else '')
        if warning and warning!=self._journal_warning:
            self._journal_warning=warning
            self.run_log_status.setText('运行日志异常：'+warning)
            self.log('运行日志异常：'+warning,'warn')
        if not warning:
            self._update_run_log_status()
        got = 0
        try:
            while got < 200:
                t, vals = self.frame_q.get_nowait()
                got += 1
                tr = t - self.t0_monotonic
                self.latest = vals
                self.latest_t = tr
                self.latest_received_monotonic = t
                self._poll_ops_zero(t, vals)
                self._display_pose, self._display_pose_ok = self._map_pose_filter.update(vals,t)
                self._display_source = vals
                self._poll_direct_home()
                if not self.paused:
                    self.ring.append(tr, vals)
                    # 遥测为 cm，轨迹缓冲/地图几何仍统一使用 mm。
                    self.traj_ring.append(tr, (vals[0] * core.OPS_CM_TO_MM,
                                               vals[1] * core.OPS_CM_TO_MM))
                    self._map_trail_ring.append(tr, (self._display_pose[0]*10,self._display_pose[1]*10)
                                                if self._display_pose_ok else (math.nan,math.nan))
                if self.recorder is not None:
                    self.recorder.write(t, vals)
        except queue.Empty:
            pass
        self.fps_count += got
        self._poll_direct_home()
        # 固件文字应答（如 CAN 启动失败）与上位机提示，转成日志而非静默丢弃。
        for _ in range(10):
            try:
                text = self.fw_text_q.get_nowait()
            except queue.Empty:
                break
            if text.removeprefix('固件文本: ').startswith('CCTRL '):
                continue  # 接收端已自动记录，5Hz控制快照不刷界面日志。
            if text.removeprefix('固件文本: ').startswith(('TSTAT ', 'TCAPS ', 'CCAPS ', 'CSTALL ')):
                if self.real_match is not None:
                    self.real_match.handle_reply(text)
                continue
            tag = "warn" if any(k in text.upper() for k in ("ERR", "FAIL", "错误", "失败")) else "info"
            self.log(text, tag)
            if hasattr(self, "dm_position_panel"):
                self.dm_position_panel.drive.handle_reply(text)
            feedback = core.stepper_feedback(text)
            if feedback:
                motor, message = feedback
                self.stepper_status[motor].setText(message)
        # 参数回读：直接写对应行的"回读"栏，不占日志。
        for _ in range(20):
            try:
                name, value = self.param_q.get_nowait()
            except queue.Empty:
                break
            self._apply_param_readback(name, value)

    def _render_ui(self):
        self.ui_refresh_count += 1
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
            dm_id = str(int(v[12])) if math.isfinite(v[12]) else "—"
            self.dm_cards["id"].set_value(dm_id)

            for row in (list(self.chassis_rows.values()) + list(self.dm_rows.values()) +
                        list(self.vision_rows.values())):
                if row.readback_channel is not None:
                    row.set_readback(v[row.readback_channel])

            status = int(v[16]) & 0xFF if math.isfinite(v[16]) else None
            status_label = "0x%02X" % status if status is not None else "无效"
            text, fault = core.DM_STATUS.get(status, ("未知/无效 %s" % status_label, True))
            self.dm_status_big.setText("DM 状态：%s  %s" % (status_label, text))
            if (self.dm_status_big.property("fault") != fault or
                    self.dm_status_big.property("enabled") != (status == 0x01)):
                self.dm_status_big.setProperty("fault", fault)
                self.dm_status_big.setProperty("enabled", status == 0x01)
                self.dm_status_big.style().unpolish(self.dm_status_big)
                self.dm_status_big.style().polish(self.dm_status_big)

            self.health_dm.setText("● DM：%s" % text)
            self.footer_dm.setText("DM[%s] %s" % (dm_id, text))

            self.ops_drift.setText("中心位置：X 左右 %.1f / Y 前后 %.1f cm ｜距零点 %.1f cm ｜车体航向 %.1f°" % (v[0], v[1], math.hypot(v[0], v[1]), v[2]))
            pose=self._display_pose if self._display_source is v else v[:3]
            if pose is not None:
                fx, fy = self._ops_to_field(pose[0] * core.OPS_CM_TO_MM,
                                            pose[1] * core.OPS_CM_TO_MM)
                # 车体图标与轨迹共用同一次映射：方向向量过 _ops_dir_to_map，不传角度。
                nose_w, left_w = self._body_nose_left(pose[2])
                self.map_view.set_pose(fx, fy,
                                       self._ops_dir_to_map(*nose_w),
                                       self._ops_dir_to_map(*left_w))
                ux, uy = core.layout_to_field(fx, fy)
                # 先舍入再取模，避免 359.96° 显示成 "360.0"。
                heading = round(self._field_heading(pose[2]), 1) % 360.0
                self.map_position.setText("场地 X(左)=%.1f cm   Y(前)=%.1f cm   航向=%.1f°（0°上 / 90°左）"
                                          % (ux / core.OPS_CM_TO_MM,
                                             uy / core.OPS_CM_TO_MM, heading))

                if self._display_source is v and not self._display_pose_ok:
                    self.map_position.setText(self.map_position.text()+" · 异常跳点已隔离（显示上一有效位置）")
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
        ring = (self._map_trail_ring if self._display_source is self.latest and self._display_source is not None
                else self.traj_ring)
        key = (id(ring), ring.revision, self.map_theta,self.map_ox,self.map_oy)
        if getattr(self,'_drawn_trail_key',None) == key:
            return
        self._drawn_trail_key = key
        view = ring.view()
        if view is None:
            return
        _t, d = view
        if len(d) == 0:
            return
        stride = max(1, len(d) // 700)
        # Preserve gap markers while reducing points. A simple [::stride] can
        # skip the sole NaN and reconnect old/new poses after ZERO or a jump.
        valid = np.isfinite(d).all(axis=1)
        breaks = np.flatnonzero(valid[1:] != valid[:-1])+1
        indices = np.unique(np.r_[np.arange(0,len(d),stride),breaks,
                                  np.maximum(0,breaks-1),len(d)-1])
        x = d[indices, 0]
        y = d[indices, 1]
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        # 与 _ops_to_field 同一变换的向量化副本：协议 X=左右、Y=前后。
        fx, fy = core.field_to_layout(self.map_ox + c * x + s * y,
                                      self.map_oy - s * x + c * y)
        self.map_view.set_trail(fx, fy)

    def _status_tick(self):
        self._previous_worker_finished()
        if self._closing_worker is not None:
            self._set_link_status("正在关闭串口…", "warn")
            return
        now = time.monotonic()
        dt = now - self.fps_t
        if dt >= 1.0:
            self.fps = self.fps_count / dt
            self.ui_refresh_hz = self.ui_refresh_count / dt
            self.ui_refresh_count = 0
            current_bytes = self.worker.parser.bytes_in if self.worker is not None else 0
            self._rx_bytes_per_second = max(0,current_bytes-self._rx_previous_bytes)/dt
            self._rx_previous_bytes = current_bytes
            self.fps_count = 0
            self.fps_t = now
        self.footer_fps.setText("刷新 %.0f Hz · 遥测 %.0f Hz" % (self.ui_refresh_hz,self.fps))

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
            self.footer_link.setText(('DL-20 CRC2 · 状态5Hz' if w.parser.protocol=='CRC2' else "UART CRC1")
                                     if w.parser.crc_frames else "UART 旧帧(无校验)")
            if w.parser.protocol=='CRC2':
                self.health_telemetry.setToolTip('X/Y/航向高频更新；其余21通道5Hz，日志记录实际刷新时间。')
            self.footer_rx.setText("RX %.1f KB/s" % (self._rx_bytes_per_second / 1024.0))
            self.footer_err.setText(("CRC %d · 丢包 %d" % (w.parser.crc_errors,w.parser.lost_packets))
                                   if w.parser.crc_frames else "失步 %d B" % w.parser.err_bytes)
            self.footer_err.setToolTip("失步丢弃 %d 字节；无效定位 %d 帧；队列丢帧 %d；重复/乱序 %d" % (
                w.parser.err_bytes,w.parser.invalid_pose_frames,getattr(w,'queue_drops',0),w.parser.duplicate_packets))
        else:
            self._set_link_status("未连接", "idle")
            self.footer_link.setText("Telemetry —")
            self.footer_rx.setText("RX 0 B")
            self.footer_err.setText("Err 0")
            self.health_link.setText("● 通信链路：未连接")
            self.health_telemetry.setText("● 遥测：—")

    def _set_link_status(self, text: str, state: str):
        if self.status_pill.text() != "● " + text:
            self.status_pill.setText("● " + text)
        if self.status_pill.property("state") != state:
            self.status_pill.setProperty("state", state)
            self.status_pill.style().unpolish(self.status_pill)
            self.status_pill.style().polish(self.status_pill)

    # ---------------- 波形/记录 ----------------
    def _set_window(self, seconds: float):
        self.window_s = float(seconds)
        cap = int(self.window_s * core.SEND_HZ) + 100
        self.ring.resize(cap)
        self.traj_ring.resize(cap)
        self._map_trail_ring.resize(cap)

    def clear_data(self):
        self.ring.clear()
        self.traj_ring.clear()
        self._map_trail_ring.clear()
        self.map_target = None
        self.map_view.set_target(None, None)
        self.map_view.set_trail(np.array([]), np.array([]))
        self._clear_path()
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

    def _refresh_chassis_inputs(self):
        for name, row in self.chassis_rows.items():
            row.reset_readback()
            self._request_param_readback(name)

    def _send_all_chassis(self):
        if any(not row.readback_known for row in self.chassis_rows.values()):
            self.log('底盘参数尚未完整回读，拒绝发送默认值。', 'warn')
            return
        for row in self.chassis_rows.values():
            row.input_dirty = False
            self.send_line(row.command_text())
            self._request_param_readback(row.cmd)

    def _dm_mode_changed(self):
        mode = int(self.dm_mode_combo.currentData())
        self.dm_mode_stack.setCurrentIndex(mode-1)
        if self.dm_position_panel.active or self.dm_position_panel.flow != "idle":
            self.dm_position_panel.stop()
        if mode == 1:
            self.dm_mode_note.setText("MIT · Kp / Kd / 前馈力矩")
        else:
            self.dm_mode_note.setText("位置速度 · 电机内部闭环")

    # ---------------- 地图 ----------------
    def _uses_ops_origin(self):
        return self.nav_map.get('coordinate_reference', {}).get('mode') == 'MEASURED_OPS_ZERO'

    def _update_origin_controls(self):
        measured = self._uses_ops_origin()
        for widget in (self.zone_combo,self.map_ox_spin,self.map_oy_spin,self.map_theta_spin,self.map_apply_button):
            widget.setEnabled(not measured)
            widget.setToolTip('当前地图使用实测出发零点与三处实测坐标；不按启停区中心偏移。' if measured else '')

    def _home_layout(self):
        if self._uses_ops_origin():
            return core.field_to_layout(0, 0)
        return core.ZONE_CENTER[int(self.zone_combo.currentData())]

    def _begin_ops_zero(self, command='ZERO'):
        self.map_ox = self.map_oy = self.map_theta = 0.0
        for widget in (self.map_ox_spin,self.map_oy_spin,self.map_theta_spin):
            widget.setValue(0)
        self._ops_zero_failure = ''
        self._ops_zero_pending = dict(source=self.worker if self.worker is not None else self.sim,
                                      requested=time.monotonic(), sent=None, last=None, samples=0, command=command,
                                      simulated=self.worker is None)
        self._ops_zero_result = dict(state='PENDING', mapping=[0,0,0], mode='MEASURED_OPS_ZERO')
        self.traj_ring.clear()
        self._map_trail_ring.clear()
        self.map_target = None
        self.map_status.setText('已请求当前位置置零；等待发送后连续三帧 OPS 零位姿，三处实测点保持不变')

    def _poll_ops_zero(self, stamp=None, values=None):
        pending = getattr(self, '_ops_zero_pending', None)
        if pending is None:
            return
        source = self.worker if self.worker is not None else self.sim
        if source is not pending['source'] or time.monotonic()-pending['requested'] > 3:
            self._ops_zero_pending = None
            self._ops_zero_failure = '置零未确认：连接变化或3秒未收到零位姿，请在实测出发点重新置零'
            self._ops_zero_result = dict(state='FAILED', reason=self._ops_zero_failure)
            self.map_status.setText(self._ops_zero_failure)
            self.log(self._ops_zero_failure,'warn')
            return
        if stamp is not None and pending['simulated']:
            snap = source.navigation_snapshot()
            if snap['hold'] is None:
                return
            stamp = snap['frame_time']
            values = (snap['hold'][0]/10,snap['hold'][1]/10,snap['yaw'])
        if stamp is None or pending['sent'] is None or stamp <= pending['sent'] or (pending['last'] is not None and stamp <= pending['last']):
            return
        pending['last'] = stamp
        zero = (all(math.isfinite(v) for v in values[:3]) and math.hypot(values[0],values[1]) <= .1
                and abs((values[2]+180)%360-180) <= .2)
        pending['samples'] = pending['samples']+1 if zero else 0
        if pending['samples'] >= 3:
            self._ops_zero_pending = None
            self._ops_zero_result = dict(state='SIM_CONFIRMED' if pending['simulated'] else 'TELEMETRY_CONFIRMED',
                                        mapping=[0,0,0],confirmed_monotonic=stamp,
                                        method='three fresh model snapshots' if pending['simulated'] else
                                        'successful reset write plus three fresh zero-pose frames; not device command ACK')
            self.map_status.setText('OPS零位姿已回读，地图同步为(0,0)/0°；回库返回本次实测出发零点')
            self.log(self.map_status.text(),'info')

    def _ops_zero_block_reason(self):
        if getattr(self, '_ops_zero_pending', None) is not None:
            return '正在等待 OPS 置零回读，暂不能规划或运行'
        if self._uses_ops_origin() and (self.map_ox,self.map_oy,self.map_theta) != (0,0,0):
            return '实测坐标映射不一致，请在测量时的出发位置置零并同步地图'
        return getattr(self, '_ops_zero_failure', '')

    def _apply_map_mapping(self):
        if self._uses_ops_origin():
            self.log('实测OPS零点地图使用一致坐标，请使用当前位置置零并同步地图','info')
            return
        self._clear_path("地图标定改变，旧路径已失效")
        # 两个 SpinBox 对外为 cm，内部地图几何继续用 mm。
        self.map_ox = float(self.map_ox_spin.value()) * core.OPS_CM_TO_MM
        self.map_oy = float(self.map_oy_spin.value()) * core.OPS_CM_TO_MM
        self.map_theta = float(self.map_theta_spin.value())
        self.run_journal.emit('MAPPING', mapping=[self.map_ox,self.map_oy,self.map_theta])
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

    def _ops_dir_to_map(self, dx: float, dy: float):
        """统一坐标(OPS 世界帧)的方向向量 → 地图绘制帧的方向向量。

        `_ops_to_field` 的线性部分是 `(dx,dy) → (s·dx − c·dy, −c·dx − s·dy)`，
        行列式 −1：绘制帧是世界帧的镜像，所以方向必须走同一次变换，不能把
        角度直接相加（车头图标与轨迹走向不一致就是这么来的）。
        """
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        return (s * dx - c * dy, -c * dx - s * dy)

    @staticmethod
    def _body_nose_left(yaw_ops: float):
        """车体前向 / 车左 在世界帧的单位向量。

        固件 `chassis_move` 用 `body = R(zangle)·世界误差`，即 `世界 = R(−zangle)·body`，
        所以车头(车体 +Y)在世界帧是 `(sin z, cos z)`、车左(车体 +X) 是 `(cos z, −sin z)`；
        +90° 时车头朝世界 +X，与 AGENTS.md 的口径一致。
        """
        h = math.radians(yaw_ops)
        return (math.sin(h), math.cos(h)), (math.cos(h), -math.sin(h))

    def _set_start_zone(self):
        if self._uses_ops_origin():
            self.send_line('ZERO')
            return
        zone = int(self.zone_combo.currentData())
        zx, zy = core.ZONE_CENTER[zone]
        ux, uy = core.layout_to_field(zx, zy)
        self.map_ox_spin.setValue(ux / core.OPS_CM_TO_MM)
        self.map_oy_spin.setValue(uy / core.OPS_CM_TO_MM)
        self.map_theta_spin.setValue(0.0)
        self._apply_map_mapping()
        self.send_line("ZERO")
        self.traj_ring.clear()
        self._map_trail_ring.clear()
        self.map_target = None
        self.map_status.setText("启停区%d校准：场地(%.1f, %.1f) cm，车头%.0f°；场地原点固定在启停区1"
                                % (zone, ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM, 0))
        self.log("启停区%d -> OPS零点(%.1f, %.1f) cm，请求 ZERO"
                 % (zone, ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM), "info")

    def _field_heading(self, yaw_ops: float) -> float:
        """用户车头角：0°朝+Y（上），90°朝+X（左）；与OPS ZERO的前向基准一致。"""
        return (float(yaw_ops) + self.map_theta) % 360.0

    def _field_math_heading(self, yaw_ops: float) -> float:
        """内部几何角：+X为0°，用于Trajectory/碰撞/停稳判断，不使用界面车头角。"""
        return (90.0 - float(yaw_ops) - self.map_theta) % 360.0

    def _field_to_body_yaw(self, field_yaw: float) -> float:
        """场地目标航向 → 固件 GOTO 的 Z（`_field_heading` 的逆）。"""
        return (float(field_yaw) - self.map_theta) % 360.0

    def _target_yaw_ops(self):
        """场地目标航向 → 固件 GOTO 的 Z；"保持当前"直接沿用当前车体航向。"""
        field_yaw = self.map_yaw_combo.currentData()
        if field_yaw is None:
            return float(self.latest[2]) if self.latest is not None else 0.0
        return self._field_to_body_yaw(float(field_yaw))

    def _goto_field(self, fx: float, fy: float, yaw_override=None):
        """场地坐标下发 GOTO。

        约定（与界面上"启停区为原点、无负坐标"一致）：
        - 实测地图使用出发时OPS零点；旧地图保留显式选区映射；
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
        # 与规划用同一份几何（地图数据 + 模拟障碍），不再读 core.py 里的旧副本
        rects = list(self.nav_map["rects"]) + list(self.nav_map.get("dynamic_rects", []))
        circles = (list(self.nav_map["circles"]) + self._obstacle_circles() +
                   list(self.nav_map.get("dynamic_circles", [])))
        blocked = core.blocked_at(fx, fy, rects, circles)
        if blocked:
            self.map_status.setText("拒绝 GOTO：目标位于【%s】" % blocked)
            self.log("GOTO 拒绝：目标(%.1f, %.1f) cm 位于%s"
                     % (fx / core.OPS_CM_TO_MM, fy / core.OPS_CM_TO_MM, blocked), "warn")
            return

        if self.latest is not None and math.isfinite(self.latest[0]) and math.isfinite(self.latest[1]):
            sx, sy = self._ops_to_field(self.latest[0] * core.OPS_CM_TO_MM,
                                        self.latest[1] * core.OPS_CM_TO_MM)
            blocked = core.seg_blocked(sx, sy, fx, fy, rects, circles)
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
        if self._ops_zero_block_reason():
            self.map_status.setText(self._ops_zero_block_reason())
            return
        zone = int(self.zone_combo.currentData())
        zx, zy = self._home_layout()
        self._observe_protective_stop()
        direct = getattr(self, '_home_after_stop', False)
        if direct and self.worker is not None and self.sim is None:
            self._start_direct_home(zone, zx, zy)
            return
        if direct and self.sim is not None and self.worker is None:
            # 手动STOP/保护停车后的回库允许穿过规划禁区。
            self._clear_path('模拟直接回启停区，旧比赛和路径已取消')
            self._manual_stop()
            if self.wheel_state is False or not self.sim.navigation_snapshot()['wheel_enabled']:
                self.map_status.setText('模拟轮已失能，请先使能再回启停区')
                return
            tx, ty = self._field_to_ops(zx, zy)
            yaw = self._field_to_body_yaw(0.0)
            if not all(math.isfinite(v) for v in (tx, ty, yaw)) or max(abs(tx),abs(ty))>3000:
                self.map_status.setText('模拟回库坐标标定超出GOTO范围')
                return
            try:
                self.sim.return_home_direct(tx,ty,yaw)
            except ValueError as exc:
                self.map_status.setText(str(exc))
                return
            self._home_after_stop = False
            self.log('SIM直接回启停区%d：跳过路径/障碍限制' % zone,'info')
            self.map_target=(zx,zy)
            self.map_status.setText('模拟直接回启停区%d：跳过路径/障碍限制，车头恢复0°；STOP可取消' % zone)
            return
        # 正常回库始终规划；点击规划开关不赋予越过禁区的权限。
        self._request_plan(zx, zy, execute_real=self.worker is not None, home_return=True)

    def _observe_protective_stop(self):
        job = getattr(self, 'real_match', None)
        if (job is not None and not getattr(job, 'home_stop_reported', False) and
                job.state == 'CANCELLED' and job.reason.startswith((
                    'STM32 FAULT', 'STM32 CANCELLED', '实际车体碰撞保护', '3秒未收到轨迹状态'))):
            job.home_stop_reported = True
            self._home_after_stop = True
        runner = getattr(self, 'competition', None)
        if runner is not None and runner.status == 'FAULT' and not getattr(runner, 'home_stop_reported', False):
            runner.home_stop_reported = True
            self._home_after_stop = True

    def _stop_navigation(self, message):
        self.send_line('STOP')
        self.map_status.setText(message)
        self.competition_status.setText(message)

    def _start_direct_home(self, zone, zx, zy):
        # 仅由手动STOP/保护停车后的回库入口调用，不经过地图几何检查。
        self._clear_path('停止后直接回库，旧路径已取消')
        self._manual_stop()
        worker = self.worker
        now = time.monotonic()
        if (worker is None or not worker.opened.is_set() or self.wheel_state is False or
                self.latest is None or not all(math.isfinite(v) for v in self.latest[:3]) or
                now-worker.last_frame_monotonic > core.TELEMETRY_WARN_S):
            self.map_status.setText('直接回库拒绝：请连接STM32、使能底盘并保持有效OPS定位')
            return
        tx, ty = self._field_to_ops(zx, zy)
        yaw = self._field_to_body_yaw(0.0)
        if not all(math.isfinite(v) for v in (tx,ty,yaw)) or max(abs(tx),abs(ty)) > 3000:
            self.map_status.setText('直接回库坐标超出GOTO范围')
            return
        self._direct_home = dict(worker=worker, signature=self._navigation_signature(), zone=zone,
            layout=(zx,zy), target=(tx,ty,yaw), requested=now, deadline=now+5,
            state='WAIT_STOP', sample=None, settled=0.0)
        self._real_send('STOP')
        self.map_status.setText('直接回启停区%d：等待STOP发送及新鲜定位停稳，随后直达；STOP可取消' % zone)

    def _poll_direct_home(self):
        ctx = getattr(self, '_direct_home', None)
        if ctx is None:
            return
        now = time.monotonic()
        worker = self.worker
        if (worker is not ctx['worker'] or self.sim is not None or not worker.opened.is_set() or
                self.wheel_state is False or ctx['signature'] != self._navigation_signature() or
                self.latest is None or not all(math.isfinite(v) for v in self.latest[:3]) or
                now-worker.last_frame_monotonic > core.TELEMETRY_WARN_S or now > ctx['deadline']):
            self._direct_home = None
            self.map_target = None
            core.discard_motion_commands(self.line_q)
            self._home_after_stop = worker is ctx['worker'] and self.sim is None
            if worker is ctx['worker'] and worker.opened.is_set() and self.sim is None:
                self._real_send('STOP')
            self.map_status.setText('直接回库已取消：连接/定位/底盘状态变化或超时')
            return
        tick = self.latest_received_monotonic
        stop_written = getattr(worker, 'last_stop_write_monotonic', 0.0)
        if ctx['state'] == 'WAIT_STOP' and (stop_written < ctx['requested'] or tick <= stop_written):
            return
        old = ctx['sample']
        if ctx['state'] == 'WAIT_STOP' and old is not None and old[0] <= stop_written:
            old = None
            ctx['settled'] = 0.0
        if old is not None and tick <= old[0]:
            return
        pose = (self.latest[0]*10, self.latest[1]*10, self.latest[2])
        ctx['sample'] = (tick, pose)
        if old is None:
            return
        dt = tick-old[0]
        still = (0 < dt <= .2 and math.dist(pose[:2],old[1][:2])/dt <= 5 and
                 abs((pose[2]-old[1][2]+180)%360-180)/dt <= 2)
        if ctx['state'] == 'SENT':
            target = ctx['target']
            still &= math.dist(pose[:2],target[:2]) < 5 and abs((pose[2]-target[2]+180)%360-180) < 1
        ctx['settled'] = ctx['settled']+dt if still else 0.0
        if ctx['settled'] < .2:
            return
        if ctx['state'] == 'WAIT_STOP':
            # STOP与GOTO隔开，防止固件同一接收批次的STOP锁存丢弃新GOTO。
            tx,ty,yaw = ctx['target']
            command = 'GOTO=%.1f,%.1f,%.1f' % (tx/10,ty/10,yaw)
            ctx.update(state='SENT', settled=0.0, deadline=now+max(30,math.dist(pose[:2],(tx,ty))/100*3+10))
            self._home_after_stop = False
            self._real_send(command)
            self.log('TX> '+command, 'tx')
            self.map_target = ctx['layout']
            self.map_status.setText('停止后直接回启停区%d：跳过规划/障碍限制，车头恢复0°；STOP可取消' % ctx['zone'])
        else:
            self._direct_home = None
            self.map_target = None
            self.run_journal.end_run('COMPLETE','直接回库位置/航向到位并停稳')
            self.map_status.setText('已直接回到启停区%d，位置/航向到位并停稳' % ctx['zone'])

    # ---------------- A* planning / FIXED-HEADING simulator execution ----------------
    def _on_map_click(self, fx: float, fy: float):
        if getattr(self, "obstacle_mode_check", None) is not None and \
                self.obstacle_mode_check.isChecked():
            self._place_obstacle(fx, fy)      # 放置模式优先：不规划、不下发
            return
        if self.plan_click_check.isChecked():
            self._request_plan(fx, fy, execute_real=self.worker is not None)
        else:
            self._goto_field(fx, fy)

    def _on_plan_switch(self, on: bool):
        self._clear_path("规划模式已切换，旧计划失效")
        self.map_status.setText("点击地图 = 实机规划后整批执行；模拟先规划后执行" if on else
                                "直接GOTO调试模式：非自动导航，执行前须人工检查整车路径")

    # ---------------- 模拟障碍（φ50×100mm，≤4 个，演示用） ----------------
    def _on_obstacle_mode(self, on: bool):
        if on:
            self.map_status.setText("点击地图 = 放下一个模拟障碍（%d/%d，φ%.0fmm）"
                                    % (len(self.sim_obstacles), core.SIM_OBSTACLE_MAX,
                                       core.SIM_OBSTACLE_R_MM * 2.0))
        else:
            self.map_status.setText("点击地图 = 实机规划后整批执行；模拟先规划后执行"
                                    if self.plan_click_check.isChecked()
                                    else "直接GOTO调试模式：非自动导航，执行前须人工检查整车路径")
        self._update_obstacle_info()

    def _update_obstacle_info(self):
        if not hasattr(self, "obstacle_info"):
            return
        n = len(self.sim_obstacles)
        text = "模拟障碍 %d/%d · φ%.0f×%.0fmm" % (n, core.SIM_OBSTACLE_MAX,
                                              core.SIM_OBSTACLE_R_MM * 2.0,
                                              core.SIM_OBSTACLE_H_MM)
        if n:
            text += "；只影响规划与仿真，不改地图数据"
        if hasattr(self, "obstacle_mode_check") and self.obstacle_mode_check.isChecked():
            text += "；【放置模式】点地图即放下一个"
        self.obstacle_info.setText(text)

    def _obstacle_circles(self):
        """模拟障碍 → circles 表（与地图 circles 同格式，直接并入规划场景）。"""
        return core.sim_obstacle_circles(self.sim_obstacles)

    def _obstacles_changed(self, msg=None):
        """障碍变化：重画 + 让旧计划立刻失效（障碍也进 _navigation_signature）。"""
        self.map_view.set_sim_obstacles(self.sim_obstacles)
        self._update_obstacle_info()
        self._clear_path(msg or "模拟障碍已改变，旧计划失效")
        if msg:
            self.map_status.setText(msg + "；旧计划已失效")

    def _place_obstacle(self, fx, fy):
        """点击放置一个模拟障碍：只改演示场景，不规划、不下发任何命令。"""
        if len(self.sim_obstacles) >= core.SIM_OBSTACLE_MAX:
            message = "模拟障碍已到上限 %d 个（先「清除障碍」）" % core.SIM_OBSTACLE_MAX
            self.map_status.setText(message)
            self.log(message, "warn")
            return None
        data = self.nav_map
        circles = list(data["circles"]) + self._obstacle_circles()
        hit = core.obstacle_placement_blocked(fx, fy, rects=data["rects"],
                                             circles=circles, bounds=data["bounds"])
        if hit:
            message = "这个位置放不下：与【%s】冲突（φ%.0f 障碍另留 %.0fmm 余量）" % (
                hit, core.SIM_OBSTACLE_R_MM * 2.0, 20.0)
            self.map_status.setText(message)
            self.log(message, "warn")
            return None
        # 不能压在车上：否则之后每次规划都会以"起点：模拟障碍"失败
        try:
            start, yaw, _mode = self._plan_preconditions()
            scene = nav.CollisionScene(
                data["rects"], circles + core.sim_obstacle_circles([(fx, fy)]), data["bounds"],
                self.plan_pad_spin.value() * core.OPS_CM_TO_MM,
                (core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, self._layout_yaw_for(yaw)),
                data["drivable_polygons"])
            on_car = scene.segment_reason(start, start)
        except (TypeError, ValueError, KeyError, IndexError):
            on_car = None       # 无定位也无模拟时不判"压车"，只按障碍冲突判定
        if on_car:
            message = "这个位置压在车上（车体/裕量与【%s】冲突），换个地方" % on_car
            self.map_status.setText(message)
            self.log(message, "warn")
            return None
        self.sim_obstacles.append((float(fx), float(fy)))
        ux, uy = core.layout_to_field(fx, fy)
        self._obstacles_changed("已放置模拟障碍 %d/%d：场地(%.1f, %.1f)cm" % (
            len(self.sim_obstacles), core.SIM_OBSTACLE_MAX,
            ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM))
        return (fx, fy)

    def _clear_obstacles(self, msg=None):
        if not self.sim_obstacles:
            if msg:
                self.map_status.setText("本来就没有模拟障碍")
            return
        self.sim_obstacles = []
        self._obstacles_changed(msg or "已清除模拟障碍")

    def _random_obstacles(self):
        """随机补齐到上限：避开固定禁区、已有障碍、场地边缘和车当前位置。"""
        free = core.SIM_OBSTACLE_MAX - len(self.sim_obstacles)
        if free <= 0:
            message = "模拟障碍已放满 %d 个（先「清除障碍」）" % core.SIM_OBSTACLE_MAX
            self.map_status.setText(message)
            self.log(message, "warn")
            return []
        try:
            start, _yaw, _mode = self._plan_preconditions()
            keep = [(start[0], start[1],
                     core.CAR_HALF_DIAG_MM + core.SIM_OBSTACLE_R_MM + 20.0)]
        except (TypeError, ValueError, KeyError, IndexError):
            keep = []           # 无定位/无模拟：只避开固定障碍与场地边缘
        data = self.nav_map
        pts, why = core.random_obstacle_points(
            free, rects=data["rects"],
            circles=list(data["circles"]) + self._obstacle_circles(),
            bounds=data["bounds"], keep_clear=keep)
        if pts:
            self.sim_obstacles.extend(pts)
            self._obstacles_changed("随机放置 %d 个模拟障碍（%d/%d）" % (
                len(pts), len(self.sim_obstacles), core.SIM_OBSTACLE_MAX))
        if why:
            self._set_plan_info(why, "bad")
            self.map_status.setText(why)
            self.log(why, "warn")
        return pts

    def _request_plan_anchor(self, name, anchor, outward):
        """功能区按钮：按右侧塔吊作业航向和裕量计算合法接近点，再规划。

        写死坐标会在车体尺寸、航向或裕量变化时变成非法停车位——原料区就出过这个问题
        （旧坐标离圆盘表面只有 90mm，280×260 车体轴向要 140mm、斜向要 205mm）。
        已标定地图使用实测目标并复查 CollisionScene；未标定地图仍现算接近点。
        两种入口均在几何冲突时明确拒绝，不能悄悄挪动实测点。
        """
        # 实测站点必须和比赛整轮使用同一目标，不能再次以名义设备锚点推一个不同停靠点。
        if self.nav_map.get('station_calibration'):
            from competition_simulation import collision_scene
            from work_orientation import work_heading
            station = {'原料区': 'raw', '粗加工区': 'rough', '暂存区': 'storage'}.get(name)
            try:
                config = self.nav_map['competition']
                point = nav.point2(config['stations'][station], '实测作业停靠点')
                heading = work_heading(config, station)
                scene = collision_scene(self.nav_map,
                    self.plan_pad_spin.value()*core.OPS_CM_TO_MM, self.sim_obstacles)
                reason = scene.pose_reason(*point, heading)
            except (ValueError, TypeError, KeyError, IndexError) as exc:
                reason = str(exc)
            if reason:
                message = '%s：实测停靠点不可用（%s），请核对设备轮廓或裕量' % (name, reason)
                self._clear_path()
                self._set_plan_info(message, 'bad')
                self.map_status.setText(message)
                self.log(message, 'warn')
                return dict(ok=False, reason=reason)
            ux, uy = core.layout_to_field(*point)
            self.log('%s：使用实测停靠点 场地(%.1f, %.1f)cm' %
                (name, ux/core.OPS_CM_TO_MM, uy/core.OPS_CM_TO_MM), 'info')
            self._request_plan(*point, execute_real=self.worker is not None, work_station=station)
            return dict(ok=True, point=point, calibrated=True)
        try:
            _start, yaw, _mode = self._plan_preconditions()
            work_station = {'原料区': 'raw', '粗加工区': 'rough', '暂存区': 'storage'}.get(name)
            goal_heading = (math.degrees(math.atan2(-outward[1], -outward[0]))+90) % 360 if work_station else self._layout_yaw_for(yaw)
            data = copy.deepcopy(self.nav_map)
            data["circles"] = list(data["circles"]) + self._obstacle_circles()
            res = core.approach_target(
                anchor, outward,
                margin=self.plan_pad_spin.value() * core.OPS_CM_TO_MM,
                footprint=(core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, goal_heading),
                rects=list(data["rects"]) + list(data.get("dynamic_rects", [])),
                circles=list(data["circles"]) + list(data.get("dynamic_circles", [])), bounds=data["bounds"],
                drivable_polygons=data["drivable_polygons"])
        except (TypeError, ValueError, KeyError, IndexError) as exc:
            message = "%s：无法取接近点（%s）" % (name, exc)
            self._clear_path()
            self._set_plan_info(message, "bad")
            self.map_status.setText(message)
            self.log(message, "warn")
            return None
        if not res["ok"]:
            message = "%s：接近点不可用（%s）" % (name, res["reason"])
            if res.get("fallback_point"):
                fx, fy = res["fallback_point"]
                ux, uy = core.layout_to_field(fx, fy)
                message += "；建议改用场地(%.1f, %.1f)cm 并人工确认走法" % (
                    ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM)
            self._clear_path()
            self._set_plan_info(message, "bad")
            self.map_status.setText(message)
            self.log(message, "warn")
            return res
        fx, fy = res["point"]
        ux, uy = core.layout_to_field(fx, fy)
        clear = res["clearance"]
        self.log("%s：按作业航向/裕量取接近点 场地(%.1f, %.1f)cm，离锚点 %.0fmm（刚好合法需 %.0fmm）%s"
                 % (name, ux / core.OPS_CM_TO_MM, uy / core.OPS_CM_TO_MM,
                    res["distance"], res["required"],
                    "" if clear is None else "，车体外缘净空 %.0fmm" % clear), "info")
        self._request_plan(fx, fy, execute_real=self.worker is not None, work_station=work_station)
        return res

    def _navigation_signature(self):
        """A plan belongs to one map + coordinate calibration + body + link session."""
        return (self.map_ox, self.map_oy, self.map_theta,
                self.plan_grid_spin.value(), self.plan_pad_spin.value(),
                core.CAR_LENGTH_MM, core.CAR_WIDTH_MM,
                self.map_yaw_combo.currentData(), id(self.sim), id(self.worker),
                bool(self.strafe_limit_check.isChecked()), self.strafe_limit_spin.value(),
                self.turn_penalty_spin.value(),
                self.plan_optimize_check.isChecked(),
                tuple(sorted(self.sim.chassis_parameters().items())) if self.sim is not None and self.worker is None else None,
                json.dumps(self.nav_map, sort_keys=True, ensure_ascii=False),
                # 模拟障碍参与签名：放了/清了障碍，旧计划必须失效
                tuple(round(v, 3) for pt in self.sim_obstacles for v in pt))

    def set_dynamic_obstacles(self, rects=(), circles=()):
        """更新布局mm动态障碍快照，作废旧轨迹与正在计算的结果。"""
        validated = nav.CollisionScene([], [], self.nav_map["bounds"],
                                       dynamic_rects=rects, dynamic_circles=circles)
        self._clear_path("动态障碍已更新；请重新规划")
        self.nav_map["dynamic_rects"] = [list(r) for r in validated.rects]
        self.nav_map["dynamic_circles"] = [list(c) for c in validated.circles]
        self.map_view.set_navigation_map(self.nav_map)

    def _nav_parameters_changed(self, *_):
        self._clear_path("参数改变，旧计划已作废；请重新规划")

    def _prepare_plan(self, fx, fy):
        # Cancel first, even when the NEW target turns out to be invalid.
        self._clear_path()
        try:
            if self.sim is not None and self.worker is None:
                self.sim.chassis_parameters(flush=True)
            goal = nav.point2((fx, fy), "目标")
            start, yaw, mode = self._plan_preconditions()
            # 目标航向进入原规划台账；连续跟踪仍需通过切线与整车复检。
            choice = self.map_yaw_combo.currentData()
            layout_yaw = self._layout_yaw_for(yaw)
            goal_layout_yaw = (layout_yaw if choice is None else
                               self._layout_yaw_for(self._field_to_body_yaw(float(choice))))
            data = copy.deepcopy(self.nav_map)
            # 模拟障碍并入规划场景：它们只影响本次计算，不改动地图数据本身
            data["circles"] = list(data["circles"]) + self._obstacle_circles()
            kwargs = dict(grid=self.plan_grid_spin.value()*10.0,
                          pad=self.plan_pad_spin.value()*10.0,
                          footprint=(core.CAR_LENGTH_MM, core.CAR_WIDTH_MM, layout_yaw),
                          start_heading_deg=layout_yaw, goal_heading_deg=goal_layout_yaw,
                          # 不勾＝不限；勾上才按「横移上限」限制单次连续横移
                          allow_strafe=True,
                          strafe_run_limit_mm=(self.strafe_limit_spin.value() * core.OPS_CM_TO_MM
                                               if self.strafe_limit_check.isChecked() else None),
                          turn_penalty_mm=self.turn_penalty_spin.value(),
                          strafe_polygons=data.get("strafe_polygons") or None,
                          rects=data["rects"], circles=data["circles"], bounds=data["bounds"],
                          dynamic_rects=data.get("dynamic_rects"), dynamic_circles=data.get("dynamic_circles"),
                          drivable_polygons=data["drivable_polygons"],
                          geometry_verified=data["geometry_verified"])
            return dict(request_id=self._plan_request_id, signature=self._navigation_signature(),
                        start=start, goal=goal, yaw=yaw, mode=mode, kwargs=kwargs,
                        plan_id=self._plan_request_id, map_id=data["map_id"], map_version=data["map_version"])
        except (TypeError, ValueError, KeyError, IndexError) as exc:
            message = "规划拒绝：" + str(exc)
            self._set_plan_info(message, "bad")
            self.map_status.setText(message)
            self.log(message, "warn")
            return None

    def _validate_quantized(self, res, context):
        """按**每段的航向**复查量化后的坐标，返回第一处不合格的原因或 None。

        不能用单一航向的 scene.validate(整条折线)：路径里可能有原地转向，每段的
        合法性与当时车体朝向绑定——实测那样会误报"第2段：中央物料区"把合法路径毙掉。
        起点保留真实位置，转向也按量化后的车心重新检查。
        """
        k = context["kwargs"]
        fp = k.get("footprint")
        scenes = {}
        def scene_at(deg):
            key = float(deg) % 360.0
            if key not in scenes:
                scenes[key] = nav.CollisionScene(
                    k["rects"], k["circles"], k["bounds"], k["pad"],
                    None if fp is None else (fp[0], fp[1], key), k["drivable_polygons"],
                    dynamic_rects=k.get("dynamic_rects"), dynamic_circles=k.get("dynamic_circles"))
            return scenes[key]
        def quantize(pt_):
            if math.dist(pt_, context["start"]) <= nav.EPS:
                return context["start"]
            ox, oy = self._field_to_ops(float(pt_[0]), float(pt_[1]))
            return self._ops_to_field(round(ox), round(oy))
        previous_heading = res["start_heading_deg"]
        for row in res.get("steps", []):
            scene = scene_at(row["heading_deg"])
            if row["kind"] == "TURN":
                reason = scene.turn_reason(quantize((row["x"], row["y"])),
                                           previous_heading, row["heading_deg"])
            else:
                reason = scene.segment_reason(
                    quantize((row["x"], row["y"])), quantize((row["to_x"], row["to_y"])))
            if reason:
                return reason
            previous_heading = row["heading_deg"]
        return None

    @staticmethod
    def _scene_for_context(context):
        k = context["kwargs"]
        return nav.CollisionScene(k["rects"], k["circles"], k["bounds"],
                                  k["pad"], k["footprint"], k["drivable_polygons"],
                                  dynamic_rects=k.get("dynamic_rects"), dynamic_circles=k.get("dynamic_circles"))

    def _report_plan_exception(self, stage, exc, context=None):
        """Fail visibly and invalidate the plan; never turn an error into a GOTO.

        This method runs on the GUI thread. Background workers only compute.
        Stale requests are discarded rather than replacing the current UI state.
        """
        if context is not None and context.get("request_id") != self._plan_request_id:
            return None
        details = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
        self._clear_path()
        message = "规划%s失败：%s: %s" % (stage, type(exc).__name__, exc)
        self._set_plan_info(message, "bad")
        self.map_status.setText(message)
        self.log(message, "warn")
        # A Qt signal-slot exception is otherwise easily missed when launched
        # from a BAT/double click. Keep the original traceback on disk too.
        path = Path(getattr(self, "_planner_error_log",
                            BASE_DIR / "logs" / "ilhc-planner-error.log"))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as fh:
                fh.write("\n[%s] %s\n%s" %
                         (time.strftime("%Y-%m-%d %H:%M:%S"), message, details))
        except OSError as log_error:
            # Read-only project folders must not hide the original failure.
            self.log("诊断日志写入失败：%s\n%s" % (log_error, details), "warn")
        else:
            self.log("规划异常详情：%s" % path, "warn")
        return {"ok": False, "execution_safe": False, "points": [], "reason": message}

    def _plan_preconditions(self):
        """规划的起点/航向/模式：串口优先、其次模拟、最后离线预览。

        取值与校验集中在这一处，供 _prepare_plan 与功能区接近点计算共用——
        起始车体碰撞姿态与路径规划必须采用同一实际航向。
        """
        if self._ops_zero_block_reason():
            raise ValueError(self._ops_zero_block_reason())
        if self.worker is not None:
            if (not self.worker.opened.is_set() or self.latest is None or
                    time.monotonic() - self.latest_received_monotonic > core.TELEMETRY_WARN_S or
                    not all(math.isfinite(v) for v in self.latest[:3])):
                raise ValueError("实时OPS定位缺失/过期；不回退成启停区位置")
            return (self._ops_to_field(self.latest[0]*10.0, self.latest[1]*10.0),
                    float(self.latest[2]), "SERIAL_PREVIEW_ONLY")
        if self.sim is not None:
            snap = self.sim.navigation_snapshot()
            if snap["hold"] is None or time.monotonic()-snap["frame_time"] > core.TELEMETRY_WARN_S:
                raise ValueError("模拟位姿未就绪/过期；请开启模拟并等待首帧")
            if snap["manual"] is not None or snap["goto"] is not None:
                raise ValueError("当前仍有手动/GOTO运动；先按STOP再规划")
            return self._ops_to_field(*snap["hold"]), snap["yaw"], "SIMULATION"
        zone = int(self.zone_combo.currentData())
        return self._home_layout(), 0.0, "OFFLINE_PREVIEW_ONLY"

    def _layout_yaw_for(self, yaw_ops):
        """OPS 航向 → 车体在布局帧里的航向（规划与接近点共用同一换算）。"""
        nose, _left = self._body_nose_left(yaw_ops)
        dx, dy = self._ops_dir_to_map(*nose)
        return math.degrees(math.atan2(dy, dx))

    def _request_plan(self, fx, fy, *, execute_real=False, home_return=False, coordinate_control=None,
                      work_station=None):
        """UI entry: CPU/geometry work runs outside Qt; timer only applies results."""
        if self.worker is not None and self.sim is None and coordinate_control is None:
            self._read_coordinate_parameters(lambda values: self._request_plan(
                fx, fy, execute_real=execute_real, home_return=home_return, coordinate_control=values,
                work_station=work_station))
            return
        context = None
        try:
            if execute_real:
                self._manual_stop()
                self._real_send('STOP')
            context = self._prepare_plan(fx, fy)
            if context is None:
                return
            context['execute_real']=execute_real
            context['home_return']=home_return
            if home_return:
                context['kwargs']['goal_heading_deg'] = self._layout_yaw_for(self._field_to_body_yaw(0.0))
            else:
                from work_orientation import station_at, work_heading
                config = self.nav_map.get('competition') or {}
                selected_station = work_station or station_at(config, context['goal'])
                if selected_station:
                    context['kwargs']['goal_heading_deg'] = work_heading(config, selected_station, context['goal'])
                    context['work_station'] = selected_station
            event = threading.Event()
            self._plan_cancel = event
            self._plan_context_pending = context
            planner = core.plan_home_path if home_return else core.plan_path
            if context['mode'] == 'SIMULATION' or coordinate_control is not None:
                planner = core.plan_coordinate_home_path if home_return and coordinate_control is not None else core.plan_coordinate_path
                if home_return and coordinate_control is not None:
                    context['kwargs']['home_staging'] = self.nav_map.get('competition',{}).get('staging',{}).get(str(self.zone_combo.currentData()))
                context['kwargs']['turn_mode'] = self.nav_map.get('turn_mode','WHEEL')
                context['kwargs']['coordinate_nodes'] = copy.deepcopy(self.nav_map.get('competition', {}).get('lane_nodes'))
                context['kwargs']['wheel_geometry'] = copy.deepcopy(self.nav_map.get('wheel_geometry'))
                context['kwargs']['chassis_control'] = self.sim.chassis_parameters() if coordinate_control is None else coordinate_control
                context['kwargs']['interactive'] = not self.plan_optimize_check.isChecked() and not home_return
            self._plan_future = self._planner_pool.submit(
                planner,
                context["start"], context["goal"], cancel=event, **context["kwargs"])
            self._set_plan_info("正在规划（只计算，未下发）…", "idle")
            self.map_status.setText("正在规划；可取消或点击新目标")
        except Exception as exc:
            # Covers preparation, Event creation and submit(), not only the
            # asynchronous calculation. v2.1.1 silently failed here on click.
            self._report_plan_exception("启动", exc, context)

    def _poll_plan(self):
        future = self._plan_future
        if future is None or not future.done():
            return
        context = self._plan_context_pending
        self._plan_future = None
        self._plan_context_pending = None
        if context is None or context.get("request_id") != self._plan_request_id:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._report_plan_exception("计算", exc, context)
            return
        try:
            accepted=self._finish_plan(result, context)
            if accepted is not None and accepted.get('ok') and context.get('execute_real'):
                self._start_real_path(accepted, context)
        except Exception as exc:
            # Coordinate quantization / geometry recheck / presentation can
            # fail as well. Clear an incomplete result instead of keeping it.
            self._report_plan_exception("结果处理", exc, context)

    def plan_to(self, fx: float, fy: float):
        """Synchronous test/programmatic API. UI uses _request_plan(), not this."""
        context = None
        try:
            context = self._prepare_plan(fx, fy)
            if context is None:
                return None
            result = core.plan_path(context["start"], context["goal"], **context["kwargs"])
            return self._finish_plan(result, context)
        except Exception as exc:
            return self._report_plan_exception("计算", exc, context)

    def _finish_plan(self, result, context):
        if (context is None or context["request_id"] != self._plan_request_id or
                context["signature"] != self._navigation_signature()):
            return None    # Old map/calibration/link/result can NEVER regain control.
        res = dict(result)
        if res.get("ok") and not res.get("axis_matched"):
            res.update(ok=False, execution_safe=False, code="INVALID_PATH",
                       reason="动作分段与台账不匹配，请重新规划")
        if res.get("ok") and not res.get('waypoint_program'):
            # The legacy command quantizes to 1mm. Validate the exact same points
            # which would be encoded, rather than check one path and send another.
            rounded = []
            real_path=self.worker is not None and self.sim is None
            coordinate_limit=10000.0 if real_path else 3000.0
            for p in res["points"]:
                ox, oy = self._field_to_ops(*p)
                if not all(math.isfinite(v) and abs(v) <= coordinate_limit for v in (ox, oy)):
                    res.update(ok=False, execution_safe=False,
                               reason="目标超过实机轨迹的±1000cm范围" if real_path else "目标超过旧GOTO的±300cm范围")
                    break
                rounded.append(self._ops_to_field(round(ox), round(oy)))
            if res.get("ok"):
                # Do not move the actual start by command rounding.
                rounded[0] = context["start"]
                reason = self._validate_quantized(res, context)
                if reason:
                    res.update(ok=False, execution_safe=False, reason="坐标量化后不安全：" + reason)
                else:
                    res["points"] = rounded
                    res["length"] = core.path_length(rounded)
                    # 净空是原规划的保守下界；量化后各段和转向已分别复检。
                    k = context["kwargs"]
                    res["min_clearance"] = core.path_clearance(
                        rounded, list(k["rects"]) + list(k.get("dynamic_rects") or ()),
                        list(k["circles"]) + list(k.get("dynamic_circles") or ()))
        if res.get("ok") and res.get("turn_count"):
            self.log("原台账含 %d 次原地转向（每 90°≈%.0fmm 等效代价）；"
                     "%s" % (res["turn_count"], context["kwargs"]["turn_penalty_mm"],
                     "连续仿真采用已复检的平滑切线轨迹" if res.get("trajectory_safe") else
                     "连续Trajectory不可用，仅查看台账与fallback"),
                     "info" if res.get("trajectory_safe") else "warn")
        if not res.get("ok"):
            self.planned_result = None
            self.planned_points = []
            self.map_view.set_path(None)
            self.plan_text.setPlainText("")
            msg = "规划失败：" + res.get("reason", "未知原因")
            self._set_plan_info(msg, "bad")
            self.map_status.setText(msg)
            self.log(msg, "warn")
            return res
        res.update(plan_id=context["plan_id"], map_id=context["map_id"],
                   map_version=context["map_version"], mode=context["mode"],
                   fixed_yaw_ops=context["yaw"], hardware_ready=False)
        self.planned_result = res
        self._planned_context = context
        self.planned_points = list(res["points"])
        if res.get('waypoint_program'):
            self.map_view.set_path(res['smoothed_points'], waypoint_points=res['points'])
        elif res.get("arcs") and res.get("smoothing_model_safe"):
            preview = res["smoothed_points"]
            markers = [preview[0], preview[-1]]
            for arc in res["arcs"]:
                markers.extend((arc.entry, arc.exit))
            self.map_view.set_path(preview, waypoint_points=markers)
        else:
            self.map_view.set_path(self.planned_points)
        self.map_view.set_trajectory(res.get("trajectory") if res.get("trajectory_safe") else None)
        self.map_view.set_skeleton(self.planned_points)
        self.map_view.set_pivots(res.get('pivots'))
        self.map_target = context["goal"]
        self.plan_text.setPlainText("\n".join(self._waypoint_lines()))
        self._set_plan_info(self._plan_summary(res), self._plan_state(res))
        self.map_status.setText("已规划（未下发）：" + self._plan_hint(res))
        self.log("A* #%d：%d航点，%.2fm；按动作航向整车校验通过，未下发"
                 % (res["plan_id"], max(0, len(res["points"])-1), res["length"]/1000), "info")
        return res

    def _waypoint_lines(self):
        yaw = (self.planned_result or {}).get("fixed_yaw_ops", 0.0)
        res = self.planned_result or {}
        program = res.get('waypoint_program')
        lines = (["PC关键坐标闭环（无圆弧）；中间PASS不等待停稳，末点STOP停稳"] if program else
                 ["规划台账；实机执行使用整批轨迹上传。起始OPS航向 %.2f°" % yaw])
        if res.get('pivots'):
            lines[0]='关键坐标闭环与连续麦轮支点转弯；减速入弯→绕指定轮旋转→衔接直行，最终STOP停稳'
            for pivot in res['pivots']:
                fx,fy=core.layout_to_field(*pivot['center_mm'])
                wheel={'FL':'左前','FR':'右前','BL':'左后','BR':'右后'}.get(pivot.get('pivot_wheel'),'指定')
                lines.append('麦轮支点 %s轮：场地(%.1f,%.1f)cm，转角%+.0f°；轮心位置保持，整车实际扫掠通过'%
                             (wheel,fx/10,fy/10,pivot['angle_deg']))
        if program:
            lines.append('控制参数来自底盘调参：'+', '.join('%s=%g'%(name,program['control'][core.SIM_PARAM_ATTRS[name]]) for name in core.CHASSIS_NAMES))
            optimality=res.get('optimality',{})
            if optimality:
                if optimality.get('policy')=='INTERACTIVE_FIRST_VERIFIED_SAFE':
                    lines.append('快速点击规划：完整车体运动预演通过；未穷举最优路线')
                else:
                    lines.append('路线优化：安全骨架代价优先，同代价选择模型更快方案；'+
                    ('有限候选范围内已证明最优' if optimality.get('proven') else '当前搜索预算内的最佳安全方案'))
        display_rows=[]
        if program:
            for part in (res.get('coordinate_prefix'),program,res.get('coordinate_suffix')):
                if part: display_rows.extend(part['waypoints'][1:])
        targets=[(r['x_mm'],r['y_mm']) for r in display_rows] if program else self.planned_points[1:]
        for k, (px, py) in enumerate(targets, 1):
            ox, oy = self._field_to_ops(px, py)
            ux, uy = core.layout_to_field(px, py)
            if program:
                row = display_rows[k-1]
                lines.append("#%d %s 场地(%6.1f,%6.1f)cm 车头%.2f° 通过范围%.1fmm" %
                             (k, row['kind'], ux/10, uy/10, (180+row['layout_yaw_deg']+180)%360-180, row['pass_mm']))
            elif res.get("turn_count"):
                lines.append("#%d 场地(%6.1f,%6.1f)cm（动作与转向见下方）" % (k, ux/10, uy/10))
            else:
                lines.append("#%d 场地(%6.1f,%6.1f)cm → GOTO=%.1f,%.1f,%.2f"
                             % (k, ux/10, uy/10, ox/10, oy/10, yaw))
        for row in res.get("steps", []):
            if row["kind"] == "START":
                continue
            lines.append("  %s 布局(%.1f,%.1f)→(%.1f,%.1f)mm 航向%.2f° 距离%.1fmm"
                         % (row["action"], row["x"], row["y"], row["to_x"], row["to_y"],
                            row["heading_deg"], row["distance_mm"]))
        segments, corners = res.get("segments") or [], res.get("corners") or []
        if segments or corners:
            lines.append("")
            lines.append("动作分段 %d 直线段 / %d 个90°角点（原地转向不计角点）"
                         % (len(segments), len(corners)))
            for seg in segments:
                lines.append("  段%d %s %s %.0fmm 航向%s 动作%s"
                             % (seg.index + 1, "水平" if seg.axis == "x" else "垂直",
                                "正向" if seg.direction > 0 else "反向", seg.length_mm,
                                "—" if seg.heading_deg is None else "%.0f°" % seg.heading_deg,
                                seg.action or "—"))
            for c in corners:
                same = (c.heading_in_deg is not None and
                        c.heading_in_deg == c.heading_out_deg)
                lines.append("  角%d 场地(%6.1f,%6.1f)cm %s"
                             % (c.index + 1, (2250.0 - c.point[1]) / 10,
                                (2250.0 - c.point[0]) / 10,
                                "航向不变：只换轮模式（不是转向）" if same else
                                "%s 转%.0f°→%.0f°" % (c.turn, c.heading_in_deg or 0.0,
                                                      c.heading_out_deg or 0.0)))
                if c.action:
                    lines.append("      （该点 %s，共 %d×90°）" % (c.action, c.steps))
                elif c.steps:
                    lines.append("      （该点转了 %d×90°，台账里无对应转向行）" % c.steps)
        if res.get("smoothing_status") not in (None, "NOT_RUN", "DISABLED", "COORDINATE_NO_ARC"):
            lines.append("圆弧平滑：%s，平滑轨迹长%.1fmm（布局mm坐标）" % (
                res["smoothing_status"], res.get("smoothed_length", 0.0)))
            for arc in res.get("arcs", []):
                lines.append("  角%d R=%.0fmm %s 入口(%.1f,%.1f) 圆心(%.1f,%.1f) 出口(%.1f,%.1f) 航向%.2f°→%.2f°"
                             % (arc.corner_index+1, arc.radius_mm, arc.direction, *arc.entry,
                                *arc.center, *arc.exit, arc.heading_in_deg, arc.heading_out_deg))
            for fallback in res.get("arc_fallbacks", []):
                lines.append("  角%d fallback [%s]：%s" % (
                    fallback["corner_index"]+1, fallback["code"], fallback["reason"]))
        if res.get("trajectory_status"):
            lines.append(("控制响应预演 [%s]：%d帧，长%.1fmm，场地坐标/实际车头/角度unwrap" if program else
                          "Trajectory [%s]：%d点，长%.1fmm，场地坐标/切线航向/角度unwrap") % (
                res["trajectory_status"], len(res.get("trajectory", [])), res.get("trajectory_length_mm", 0.0)))
            if res.get("trajectory_reason"):
                lines.append("  Trajectory fallback：" + res["trajectory_reason"])
        if res.get('segment_program'):
            rows = res['segment_program']['segments']
            lines.append('PC端点执行：%d段（直线%d / 圆弧%d），运行不保存采样点表；仅站点STOP停稳' %
                         (len(rows), sum(p['kind']=='LINE' for p in rows), sum(p['kind']=='ARC' for p in rows)))
        if res.get('waypoint_program'):
            lines.append('坐标闭环：%d个目标，麦轮支点转弯%d处；STOP检查位置/航向/停稳' %
                         (len(res['waypoint_program']['waypoints'])-1,len(res.get('pivots',[]))))
        return lines

    @staticmethod
    def _plan_summary(res):
        if res.get('waypoint_program'):
            c = res['waypoint_program']['control']
            return '坐标闭环：%d个关键航点目标 · 麦轮支点%d处 · 预演%.2fs\n底盘参数 KPX/KPY/KPZ=%g/%g/%g，XVMAX=%g\n实际响应整车扫掠通过；实机整批上传' % (
                len(res['waypoint_program']['waypoints'])-1,len(res.get('pivots',[])),res['predicted_tracking_s'],c['kpx'],c['kpy'],c['kpz'],c['xyvmax'])
        centre, body = res.get("min_clearance"), res.get("body_clearance")
        ctext = "无列出障碍" if centre is None or not math.isfinite(centre) else "%.1fmm" % centre
        btext = "未核算" if body is None else "%.1fmm" % body
        limit = res.get("strafe_run_limit_mm")
        strafe = "" if limit is None else " · 单次最长连续横移 %.0fmm（区外 %.0fmm，上限 %.0fmm）" % (
            res.get("max_strafe_run_mm") or 0.0, res.get("max_outside_strafe_run_mm") or 0.0, limit)
        if res.get("arcs") or res.get("arc_fallbacks"):
            strafe += " · 圆弧%d处 / fallback%d处" % (len(res.get("arcs", [])), len(res.get("arc_fallbacks", [])))
        if res.get("arcs") and res.get("smoothing_model_safe"):
            strafe += " · 平滑轨迹%.2fm（整车扫掠通过）" % (res["smoothed_length"]/1000)
        if res.get("trajectory_safe"):
            strafe += " · 连续Trajectory %d点（整车复检通过）" % len(res["trajectory"])
        elif res.get("trajectory_reason"):
            strafe += "\nTrajectory不可用：" + res["trajectory_reason"]
        return ("原台账%d 个航点 · 长 %.2fm · 展开 %d格 · 网格 %.1fcm%s\n"
                "原台账整车外缘净空下界 %s（含转向与区域边界）\n车心到障碍 %s（不是车体余量）\n%s"
                % (max(0, len(res["points"])-1), res["length"]/1000, res["expanded"],
                   res["grid"]/10, strafe, btext, ctext,
                   "按当前地图校验；整车复检通过可整批上传STM32执行"))

    @staticmethod
    def _plan_hint(res):
        if res.get('waypoint_program'):
            if res.get('pivots'):return '连续麦轮支点旋转已完整预演；减速入弯、转完衔接下一段'
            return 'PC坐标定位：位置误差转车体轴，与航向纠偏混合；途中点不等停稳'
        if not res.get("execution_safe"):
            return "仅参考，未通过整车执行检查"
        if res.get("trajectory_safe"):
            return "连续Trajectory已复检，可100mm lookahead仿真跟踪；只有最终STOP停稳"
        if res.get("arcs"):
            return "已显示圆弧预览；整条连续Trajectory未通过复检，不能跟踪"
        if res.get("turn_count"):
            return "含原地转向：可查看动作台账，本版仿真行驶不执行"
        return "连续Trajectory不可用，不能启动跟踪；请查看fallback原因"

    @staticmethod
    def _plan_state(res):
        if not res.get("ok") or not res.get("execution_safe"):
            return "bad"
        return "ok"

    def _set_plan_info(self, text, state="idle"):
        self.plan_info.setText(text)
        self.plan_info.setProperty("state", state)
        self.plan_info.style().unpolish(self.plan_info)
        self.plan_info.style().polish(self.plan_info)

    def _clear_path(self, msg=None):
        home = getattr(self, '_direct_home', None)
        self._direct_home = None
        if home is not None:
            core.discard_motion_commands(self.line_q)
        if (home is not None and home['state'] == 'SENT' and
                self.worker is home['worker'] and self.sim is None and self.worker.opened.is_set()):
            self._real_send('STOP')
        self._observe_protective_stop()
        self._cancel_real_match(msg or '清路径/重规划，实机整批已取消')
        self._cancel_competition(msg or '清路径/重新规划，比赛模拟已取消')
        self._stop_follow()    # actual simulator goal cancellation, not just timer
        self._plan_request_id = getattr(self, "_plan_request_id", 0) + 1
        event = getattr(self, "_plan_cancel", None)
        if event is not None:
            event.set()
        self._plan_cancel = None
        future = getattr(self, "_plan_future", None)
        if future is not None:
            future.cancel()
        self._plan_future = None
        self._plan_context_pending = None
        self._planned_context = None
        self.planned_result = None
        self.planned_points = []
        self.map_target = None
        if hasattr(self, "map_view"):
            self.map_view.set_path(None)
        if hasattr(self, "plan_text"):
            self.plan_text.setPlainText("")
        if hasattr(self, "plan_info"):
            self._set_plan_info(msg or "未规划 / 旧计划失效", "idle")
        if msg:
            self.log(msg, "info")

    def _cancel_competition(self, reason):
        event = getattr(self, '_competition_cancel', None)
        if event is not None:
            event.set()
        future = getattr(self, '_competition_future', None)
        if future is not None:
            future.cancel()
        self._competition_cancel = self._competition_future = None
        runner = getattr(self, 'competition', None)
        if future is not None and hasattr(self, 'competition_status'):
            self.competition_status.setText(reason)
        if runner is not None and runner.active:
            runner.cancel(reason)
            self.run_journal.end_run(runner.status, reason)
            if hasattr(self, 'competition_status'):
                self.competition_status.setText(reason)
            self.map_view.set_reference(None)
        if hasattr(self, 'competition_start'):
            self.competition_start.setEnabled(True)

    def _start_competition(self):
        from competition_simulation import parse_task_code, with_competition_defaults, compile_match
        if self._ops_zero_block_reason():
            self.competition_status.setText(self._ops_zero_block_reason());return
        if self.sim is None or self.worker is not None:
            self.competition_status.setText('请先开启PC模拟模式；完整比赛模拟不连接实车。')
            return
        if getattr(self, '_competition_future', None) is not None or (self.competition is not None and self.competition.active):
            self.competition_status.setText('本轮已启动，请先停止比赛再准备新一轮。')
            return
        self._clear_path('准备完整初赛模拟')
        self._manual_stop()
        try:
            parse_task_code(self.competition_code.text())
            parameters = self.sim.chassis_parameters(flush=True)
            snap = self.sim.navigation_snapshot()
            if not snap['wheel_enabled'] or self.wheel_state is False:
                raise ValueError('模拟轮已失能，请先使能')
            # 模拟与实机使用同一地图快照，额外障碍不能在准备比赛时被静默删除。
            data, _ = with_competition_defaults(self.nav_map)
            self.nav_map = data
            self.map_view.set_navigation_map(data)
            self.map_view.set_sim_obstacles(self.sim_obstacles)
            self._update_obstacle_info()
            self.map_ox = self.map_oy = self.map_theta = 0.0
            for widget in (self.map_ox_spin, self.map_oy_spin, self.map_theta_spin):
                widget.setValue(0)
            self.competition = None
            self._home_after_stop = False
            zone = int(self.zone_combo.currentData())
            event = threading.Event()
            self._competition_cancel = event
            self._competition_context = dict(sim=self.sim, mapping=(0, 0, 0), snapshot=copy.deepcopy(data),
                                             chassis_control=parameters,
                                             signature=self._navigation_signature(), zone=zone,
                                             obstacles=tuple(tuple(p) for p in self.sim_obstacles))
            self._competition_future = self._planner_pool.submit(compile_match, data,
                self.competition_code.text(), zone, self.plan_pad_spin.value()*core.OPS_CM_TO_MM, event.is_set,
                self._competition_context['obstacles'], coordinate_mode=True, chassis_control=parameters)
            self.competition_start.setEnabled(False)
            self.competition_status.setText('启动前预检：包含%d个模拟障碍，准备固定或避障路线并检查作业航向；尚未发车…' % len(self.sim_obstacles))
        except Exception as exc:
            self._cancel_competition(str(exc))
            self.competition_status.setText('比赛模拟拒绝：'+str(exc))

    @staticmethod
    def _discard_trajectory_commands(q):
        names=('CCAPS','CBEGIN','CPOINT','TCAPS','TBEGIN','TPOINT','TCOMMIT','TRUN','TABORT','TSTATUS','TRESUME','TPAUSE','TCONTINUE')
        with q.mutex:
            kept=[line for line in q.queue if str(line).split('=',1)[0] not in names]
            q.queue.clear();q.queue.extend(kept)

    def _real_send(self,line):
        if line.startswith(('GOTO=', 'GOTOHOLD=')) and self.run_journal.active_run is None:
            self._journal_begin('DIRECT_HOME', {'command':line})
        self.run_journal.emit('COMMAND_QUEUED', command=line)
        if line=='STOP':
            self._discard_trajectory_commands(self.line_q)
            core.discard_motion_commands(self.line_q)
            self.urgent_q.put('STOP')
        elif line.startswith('TPAUSE='):
            self.urgent_q.put(line)
        elif line.startswith('TABORT='):
            self._discard_trajectory_commands(self.line_q)
            self.urgent_q.put(line)
        else:
            self.line_q.put(line)

    def _cancel_real_match(self,reason):
        self._coordinate_read = None
        event=getattr(self,'_real_cancel',None)
        future=getattr(self,'_real_future',None)
        if event is not None: event.set()
        if future is not None: future.cancel()
        self._real_future=self._real_cancel=None
        job=getattr(self,'real_match',None)
        if job is not None and job.active:
            job.cancel(reason)
            self.run_journal.end_run(job.state, reason, validation_trigger=(getattr(self,'_real_context',None) or {}).get('validation_failure'))
            job.reported_terminal=True
            if hasattr(self,'competition_status'): self.competition_status.setText('实机比赛已取消：'+reason)
            if hasattr(self,'map_view'): self.map_view.set_reference(None)
        elif future is not None: self._real_send('STOP')
        if hasattr(self,'real_match_start'):
            self.real_match_start.setEnabled(True)
        if hasattr(self,'follow_btn'):
            self.follow_btn.setText('执行规划路径')

    def _start_real_match(self, *, coordinate_control=None):
        from competition_simulation import compile_match,collision_scene,with_competition_defaults
        from hardware_trajectory import make_match_batch
        if self._ops_zero_block_reason():
            self.competition_status.setText(self._ops_zero_block_reason());return
        worker=self.worker
        if worker is None or self.sim is not None or not worker.opened.is_set():
            self.competition_status.setText('实机比赛需要已连接STM32；PC模拟仍使用一键比赛模拟。');return
        if self._real_future is not None or (self.real_match is not None and self.real_match.active):
            self.competition_status.setText('已有实机批次，请先停止。');return
        try:
            configured,added=with_competition_defaults(self.nav_map)
            if added:
                self.nav_map=configured
                self.map_view.set_navigation_map(self.nav_map)
                self.map_view.set_sim_obstacles(self.sim_obstacles)
                self.log('实机运行已自动加载比赛启停区、站点及车道配置：'+', '.join(added),'info')
        except (ValueError,OSError) as exc:
            self.competition_status.setText('比赛配置自动加载失败：'+str(exc));return
        if self.wheel_state is False or self.latest is None or not all(math.isfinite(v) for v in self.latest[:3]) or time.monotonic()-worker.last_frame_monotonic>core.TELEMETRY_WARN_S:
            self.competition_status.setText('底盘失能或遥测过期，拒绝实机上传。');return
        if coordinate_control is None:
            self._read_coordinate_parameters(lambda values: self._start_real_match(coordinate_control=values))
            return
        self._home_after_stop = False
        self._clear_path('准备实机整批比赛')
        self.real_match = None
        self._manual_stop()
        self._real_send('STOP')
        data=copy.deepcopy(self.nav_map);zone=int(self.zone_combo.currentData())
        mapping=(self.map_ox,self.map_oy,self.map_theta)
        v=self.latest
        actual=core.layout_to_field(*self._ops_to_field(v[0]*10,v[1]*10))
        from competition_simulation import departure_reference
        home=core.layout_to_field(*departure_reference(data,zone)[0])
        if math.dist(actual,home)>5 or abs((self._field_heading(v[2])+180)%360-180)>3:
            self.competition_status.setText('请先在实测出发位置置零并同步地图，起点需在5mm/3°内；上传不自动置零。');return
        event=threading.Event();self._real_cancel=event
        ctx=dict(worker=worker,signature=self._navigation_signature(),mapping=mapping,event=event,
                 code=self.competition_code.text(), zone=zone,
                 obstacles=tuple(tuple(p) for p in self.sim_obstacles))
        self._real_context=ctx
        code=self.competition_code.text();margin=self.plan_pad_spin.value()*10
        def prepare():
            match=compile_match(data,code,zone,margin,event.is_set,ctx['obstacles'],
                                coordinate_mode=True,chassis_control=coordinate_control)
            scene=collision_scene(data,margin,ctx['obstacles'])
            return make_match_batch(match,mapping,scene,cancelled=event.is_set),scene
        self._real_future=self._planner_pool.submit(prepare)
        self.real_match_start.setEnabled(False)
        self.competition_status.setText('实机整轮准备及预检中：无额外障碍时复用固定路线，有障碍时验证或重规划；车辆停车等待，可停止取消。')

    def _real_valid(self,ctx):
        now=time.monotonic();worker=self.worker
        age=None if worker is None else max(0.0,now-worker.last_frame_monotonic)
        changed=[]
        reason=''
        if ctx['event'].is_set(): reason='实机批次取消信号已触发'
        elif worker is not ctx['worker']: reason='串口连接会话已变化'
        elif self.sim is not None: reason='运行模式已切换到模拟'
        elif self.latest is None: reason='尚无有效OPS定位数据'
        elif not all(math.isfinite(v) for v in self.latest[:3]): reason='OPS定位包含非法数值'
        elif self.wheel_state is False: reason='底盘轮使能已关闭'
        elif not worker.opened.is_set(): reason='串口连接已断开'
        else:
            signature=self._navigation_signature()
            if ctx['signature']!=signature:
                names=('X偏移','Y偏移','坐标旋转','规划网格','安全裕量','车长','车宽','目标航向',
                       '模拟会话','串口会话','横移限制开关','横移上限','转弯代价','优化选项',
                       '模拟底盘参数','地图配置','障碍配置')
                changed=[name for name,a,b in zip(names,ctx['signature'],signature) if a!=b]
                reason='导航配置已变化：'+('、'.join(changed) or '签名结构')
            elif ctx['code']!=self.competition_code.text(): reason='比赛任务码已变化'
            elif ctx['zone']!=int(self.zone_combo.currentData()): reason='启停区选择已变化'
            elif age>core.TELEMETRY_WARN_S:
                reason='OPS遥测过期：%.0fms未收到有效帧（上限%.0fms）'%(age*1000,core.TELEMETRY_WARN_S*1000)
        if reason:
            ctx['validation_failure']=dict(reason=reason,monotonic=now,
                telemetry_age_ms=None if age is None else round(age*1000,3),
                telemetry_limit_ms=core.TELEMETRY_WARN_S*1000,
                ops_pose_cm_deg=None if self.latest is None else [v if math.isfinite(v) else None for v in self.latest[:3]],
                wheel_enabled=self.wheel_state,link_open=bool(worker is not None and worker.opened.is_set()),
                changed_fields=changed)
        return not reason

    def _start_real_path(self,route,context):
        """点击目标和执行按钮共用整批协议；不把航点逐条转换成GOTO。"""
        from hardware_trajectory import make_path_batch
        try:
            if self.worker is None or self.sim is not None or not self.worker.opened.is_set():
                raise ValueError('请先连接STM32')
            if not route or not context or context['signature']!=self._navigation_signature():
                raise ValueError('没有当前有效路径，请重新规划')
            if self.wheel_state is False or self.latest is None or not all(math.isfinite(v) for v in self.latest[:3]):
                raise ValueError('底盘失能或OPS定位非法')
            if time.monotonic()-self.worker.last_frame_monotonic>core.TELEMETRY_WARN_S:
                raise ValueError('OPS遥测过期')
            actual=self._ops_to_field(self.latest[0]*10,self.latest[1]*10)
            if math.dist(actual,context['start'])>5 or abs((self.latest[2]-context['yaw']+180)%360-180)>3:
                raise ValueError('实际起点/车头已改变，需重新规划（5mm/3°）')
            if not route.get('waypoint_program'):
                self._request_plan(*context['goal'],execute_real=True,home_return=context.get('home_return',False),
                                   work_station=context.get('work_station'))
                return
            self._home_after_stop = False
            self._cancel_real_match('新点击路径接管')
            self.real_match=None
            self._manual_stop()
            self._real_send('STOP')
            event=threading.Event();self._real_cancel=event
            ctx=dict(worker=self.worker,signature=context['signature'],mapping=(self.map_ox,self.map_oy,self.map_theta),
                     event=event,code=self.competition_code.text(),zone=int(self.zone_combo.currentData()),
                     obstacles=tuple(tuple(p) for p in self.sim_obstacles),navigation_context=copy.deepcopy(context))
            self._real_context=ctx
            scene=self._scene_for_context(context)
            data=copy.deepcopy(self.nav_map);route=copy.deepcopy(route)
            start=context['start'];start_yaw=context['kwargs']['start_heading_deg']
            goal_yaw=context['kwargs']['goal_heading_deg']
            def prepare():
                return make_path_batch(route,start,start_yaw,goal_yaw,ctx['mapping'],scene,data,
                                       cancelled=event.is_set),scene
            self._real_future=self._planner_pool.submit(prepare)
            self.real_match_start.setEnabled(False)
            self.map_status.setText('实机点击路径预检中：起终点转头、整批量化后矩形扫掠；车辆停车等待')
        except Exception as exc:
            self._cancel_real_match(str(exc))
            self.map_status.setText('实机点击路径拒绝：'+str(exc))
            self.competition_status.setText('实机点击路径拒绝：'+str(exc))

    def _read_coordinate_parameters(self, callback):
        """每次实机预检独立GET快照；CBEGIN由MCU再次核对RAM参数。"""
        self._clear_path('读取实机底盘参数，车辆停车等待')
        self._manual_stop()
        self._real_send('STOP')
        if self.worker is None or self.sim is not None or not self.worker.opened.is_set():
            self.map_status.setText('请先连接STM32')
            return
        now = time.monotonic()
        self._drain_queue(self.param_q)
        self._coordinate_read = dict(worker=self.worker, signature=self._navigation_signature(),
                                     callback=callback, values={}, sent=set(), next=now, deadline=now+6,
                                     attempts={}, last_sent={})
        self.map_status.setText('正在回读七项底盘参数，完成后预演关键坐标')
        self.competition_status.setText('正在回读七项底盘参数；尚未上传或发车')

    def _poll_coordinate_parameters(self):
        pending = getattr(self, '_coordinate_read', None)
        if pending is None:
            return
        now = time.monotonic()
        reason=''
        if self.worker is not pending['worker'] or self.sim is not None:
            reason='串口会话已改变'
        elif not self.worker.opened.is_set():
            reason='串口连接已断开'
        elif pending['signature'] != self._navigation_signature():
            reason='地图、定位映射或规划设置已改变'
        elif now > pending['deadline']:
            missing=[name for name in core.CHASSIS_NAMES if core.SIM_PARAM_ATTRS[name] not in pending['values']]
            reason='超时，未收到 '+', '.join('%s（请求%d次）'%(n,pending['attempts'].get(n,0)) for n in missing)
        if reason:
            self._coordinate_read = None
            self.map_status.setText('实机底盘参数回读失败：'+reason+'；请重新规划')
            self.competition_status.setText(self.map_status.text())
            self.log(self.map_status.text(),'warn')
            return
        if len(pending['values']) == len(core.CHASSIS_NAMES):
            self._coordinate_read = None
            pending['callback'](dict(pending['values']))
            return
        missing = [name for name in core.CHASSIS_NAMES if core.SIM_PARAM_ATTRS[name] not in pending['values']]
        eligible=[name for name in missing if pending['attempts'].get(name,0)<4 and
                  now-pending['last_sent'].get(name,-float('inf'))>=.6]
        if eligible and now >= pending['next']:
            name = min(eligible,key=lambda n:(n in pending['sent'],pending['last_sent'].get(n,0)))
            pending['sent'].add(name)
            pending['attempts'][name]=pending['attempts'].get(name,0)+1
            pending['last_sent'][name]=now
            self._request_param_readback(name)
            pending['next'] = now + .12

    def _real_collision_changed(self, enabled):
        self._real_previous_pose = None
        job=getattr(self,'real_match',None)
        if job is not None:
            job.batch['runtime_collision_protection']=bool(enabled)
        self.log('实机运行地图碰撞保护：'+('开启' if enabled else '关闭'),'info')

    def _refresh_motion_pause(self):
        job = self.real_match
        if job is not None and job.active:
            state = job.state
            self.motion_pause_btn.setText({'PAUSING':'正在暂停…', 'PAUSED':'继续车辆',
                'CONTINUING':'正在继续…'}.get(state, '暂停车辆'))
            self.motion_pause_btn.setEnabled(state in ('RUNNING','WAITING','RESUMING','PAUSED'))
        elif self.sim is not None and self.sim.navigation_snapshot()['active']:
            self.motion_pause_btn.setText('继续车辆' if self.sim.navigation_snapshot().get('paused') else '暂停车辆')
            self.motion_pause_btn.setEnabled(True)
        else:
            self.motion_pause_btn.setText('暂停车辆')
            self.motion_pause_btn.setEnabled(False)

    def _toggle_motion_pause(self):
        try:
            if self.real_match is not None and self.real_match.active:
                if self.real_match.state == 'PAUSED':
                    self.real_match.continue_run()
                else:
                    self.real_match.pause()
                state = self.real_match.state
            elif self.sim is not None:
                paused = not self.sim.navigation_snapshot().get('paused', False)
                self.sim.set_navigation_paused(paused)
                if self.follow is not None:
                    if paused:
                        self.follow['pause_started'] = time.monotonic()
                    else:
                        self.follow['t0'] += time.monotonic()-self.follow.pop('pause_started', time.monotonic())
                state = 'PAUSED' if paused else 'RUNNING'
            else:
                raise ValueError('当前没有可暂停的轨迹')
            self.run_journal.emit('USER_MOTION_PAUSE', state=state)
            self.log('车辆轨迹：' + state)
        except ValueError as exc:
            self.log(str(exc), 'warn')
            self.competition_status.setText(str(exc))
        self._refresh_motion_pause()

    def _poll_real_match(self):
        self._refresh_motion_pause()
        self._poll_coordinate_parameters()
        from hardware_trajectory import BatchUploader
        from competition_simulation import collision_scene
        future=getattr(self,'_real_future',None)
        if future is not None and future.done():
            self._real_future=None;ctx=self._real_context
            try:
                batch,scene=future.result()
                if not self._real_valid(ctx): raise ValueError(ctx['validation_failure']['reason'])
                if 'navigation_context' in ctx:
                    plan=ctx['navigation_context']
                    if (math.dist(self._ops_to_field(self.latest[0]*10,self.latest[1]*10),plan['start'])>5 or
                            abs((self.latest[2]-plan['yaw']+180)%360-180)>3):
                        raise ValueError('预检期间实际起点/车头改变，请重新规划')
                batch['runtime_collision_protection']=self.real_collision_check.isChecked()
                self.real_match=BatchUploader(batch,self._real_send,valid=lambda:self._real_valid(ctx),
                    invalid_reason=lambda:ctx['validation_failure']['reason'])
                self._journal_begin('COORDINATE_BATCH', batch)
                # 规划裕量供跟踪误差消耗；本地预算保留1mm，实际车体再检查该剩余裕量。
                if 'navigation_context' in ctx:
                    runtime_context=copy.deepcopy(ctx['navigation_context'])
                    runtime_context['kwargs']['pad']=1
                    self._real_scene=self._scene_for_context(runtime_context)
                else:
                    self._real_scene=collision_scene(batch['match']['map_snapshot'],1,ctx['obstacles'])
                self.traj_ring.clear()
                self._map_trail_ring.clear()
                self.traj_ring.resize(int(300*core.SEND_HZ)+100)
                self._map_trail_ring.resize(int(300*core.SEND_HZ)+100)
                self.map_view.set_trail([], [])
                self._real_previous_pose = None
                display = batch.get('display_points', batch['field_points'])
                self.map_view.set_path([core.field_to_layout(p['x_mm'],p['y_mm']) for p in display])
                self.map_view.set_pivots([pivot for stage in batch['match']['stages']
                    if stage['kind']=='TRAVEL' for pivot in stage['route'].get('pivots',[])])
                skeleton=[batch['match']['home']]
                for stage in batch['match']['stages']:
                    if stage['kind']=='MANEUVER': skeleton.append(stage['target'])
                    elif stage['kind']=='TRAVEL': skeleton.extend(stage['route']['points'][1:])
                self.map_view.set_skeleton(skeleton)
                self.map_view.set_trajectory(display)
                self.plan_text.setPlainText('\n'.join('%02d %s'%(i+1,s['label']) for i,s in enumerate(batch['match']['stages'])))
                self.real_match.start()
                self.follow_btn.setText('停止跟踪')
                if batch['kind']=='STM32_POINT_PATH':
                    self.log('实机点击路径整批上传：id=%d，%d点，%s%s' % (batch['id'],batch['point_count'],
                        batch['execution_mode'],('；fallback：'+batch['fallback_reason']) if batch['fallback_reason'] else ''),'info')
                else:
                    self.log('实机自动跑图整批上传：id=%d，%d点，%d个站点停稳后自动继续'%(batch['id'],batch['point_count'],len(batch['stations'])),'info')
            except Exception as exc:
                self._cancel_real_match(str(exc));self.competition_status.setText('实机预检拒绝：'+str(exc))
                self.map_status.setText('实机预检拒绝：'+str(exc));return
        job=getattr(self,'real_match',None)
        if job is None: return
        if future is not None and not future.done(): return
        if not job.active and getattr(job, 'reported_terminal', False): return
        was_active=job.active
        if job.active:
            job.tick()
            if (job.active and job.state in ('RUNNING','WAITING','RESUMING','PAUSING','PAUSED','CONTINUING') and self.latest is not None
                    and self.real_collision_check.isChecked()):
                v=self.latest;lx,ly=self._ops_to_field(v[0]*10,v[1]*10)
                why=self._real_scene.pose_reason(lx,ly,self._field_heading(v[2])-180)
                pose=(lx,ly,self._field_heading(v[2])-180)
                previous=getattr(self, '_real_previous_pose', None)
                collision_kind='当前车体'
                if previous is not None and not why:
                    why=self._real_scene.moving_pose_reason(previous,pose)
                    collision_kind='相邻帧扫掠'
                self._real_previous_pose=pose
                if why:
                    fx,fy=core.layout_to_field(lx,ly)
                    job.batch['collision_trigger']=dict(reason=why,check=collision_kind,field_x_mm=fx,field_y_mm=fy,
                        heading_deg=self._field_heading(v[2]),previous_layout_pose=previous,
                        layout_pose=pose,telemetry_time=self.latest_received_monotonic)
                    detail='触发位置X=%.1fcm Y=%.1fcm，航向%.1f°'%(fx/10,fy/10,self._field_heading(v[2]))
                    if collision_kind=='相邻帧扫掠':
                        prev_x,prev_y=core.layout_to_field(*previous[:2])
                        detail='前帧X=%.1fcm Y=%.1fcm → '%(prev_x/10,prev_y/10)+detail
                    job.cancel('实际车体碰撞保护：%s · %s（%s）'%(why,collision_kind,detail))
            if job.active and job.state in ('RUNNING','WAITING','RESUMING','PAUSING','PAUSED','CONTINUING'):
                i=job.cursor;target=min(job.batch['length_mm'],job.progress+100)
                while i+1<len(job.batch['points']) and job.batch['points'][i][2]/10<target and not job.batch['points'][i][4]&1:
                    i+=1
                if job.batch.get('coordinate'):
                    i=min(job.cursor+1,len(job.batch['field_points'])-1)
                self.map_view.set_reference(job.batch['field_points'][i])
        labels='；'.join(job.batch['stations'].get(job.cursor,[]))
        snapshot=(job.state,job.cursor,job.progress,job.received,job.reason)
        if snapshot!=getattr(self,'_journal_last_batch',None):
            self._journal_last_batch=snapshot
            self.run_journal.emit('BATCH_STATE', batch_id=job.batch['id'],state=job.state,
                cursor=job.cursor,progress_mm=job.progress,received=job.received,reason=job.reason)
        title='实机点击路径' if job.batch['kind']=='STM32_POINT_PATH' else '实机自动跑图'
        if job.batch.get('execution_mode')=='STOP_TURN_FALLBACK':
            title+='（停转回退）'
        status='%s %s · %d/%d点已缓存 · 进度%.0fmm%s%s'%(title,job.state,job.received,
            len(job.batch['points']),job.progress,(' · 站点自动启停：'+labels) if labels else '',(' · '+job.reason) if job.reason else '')
        if job.active and getattr(job,'warning',''):status+=' · '+job.warning
        if not self.real_collision_check.isChecked(): status+=' · 地图碰撞保护关闭'
        self.competition_status.setText(status)
        if job.batch['kind']=='STM32_POINT_PATH': self.map_status.setText(status)
        if not job.active:
            trigger=(getattr(self,'_real_context',None) or {}).get('validation_failure')
            if job.state=='CANCELLED' and trigger and trigger['reason']==job.reason:
                job.batch['validation_trigger']=copy.deepcopy(trigger)
            self.run_journal.end_run(job.state,job.reason,validation_trigger=trigger,
                collision_trigger=job.batch.get('collision_trigger'))
            self._observe_protective_stop()
            job.reported_terminal = True
            self.real_match_start.setEnabled(True);self.map_view.set_reference(None)
            self.follow_btn.setText('执行规划路径')
            if was_active: self.log('实机批次%s：%s'%(job.state,job.reason),'info' if job.state=='DONE' else 'warn')

    def _export_real_match(self):
        job=self.real_match
        if job is None: self.competition_status.setText('尚无实机批次记录');return
        path,_=QFileDialog.getSaveFileName(self,'导出实机整批轨迹','ilhc-hardware-batch.json','JSON (*.json)')
        if path:
            try:
                Path(path).write_text(json.dumps(dict(batch=job.batch,state=job.state,reason=job.reason,
                    received=job.received,cursor=job.cursor,progress_mm=job.progress),ensure_ascii=False,indent=2,
                    allow_nan=False,default=lambda o:o.as_dict()),encoding='utf-8')
            except Exception as exc: self.log('实机批次导出失败：'+str(exc),'warn')

    def _poll_competition(self):
        from competition_simulation import CompetitionRunner
        future = getattr(self, '_competition_future', None)
        if future is not None and future.done():
            ctx = self._competition_context
            self._competition_future = None
            try:
                match = future.result()
                event = self._competition_cancel
                if (event is None or event.is_set() or self.sim is not ctx['sim'] or self.worker is not None or
                        ctx['signature'] != self._navigation_signature()):
                    raise ValueError('比赛预检期间地图/参数/链路变化')
                sim = ctx['sim']
                with sim._state_lock:
                    sim.cancel_navigation()
                    sim.ops_reference_yaw = 0.0
                    sim.hold = core.layout_to_field(*match['home'])
                    sim.zval = 180+match['start_yaw']
                    sim.make_frame(sim._t)
                self.traj_ring.clear()
                self._map_trail_ring.clear()
                self.traj_ring.resize(int(190*core.SEND_HZ)+100)
                self._map_trail_ring.resize(int(190*core.SEND_HZ)+100)
                self.map_view.set_trail([], [])
                skeleton, smooth, arrows, markers, station = [match['home']], [match['home']], [], [match['home']], 0.0
                for stage in match['stages']:
                    if stage['kind'] == 'MANEUVER':
                        if math.dist(smooth[-1], stage['target']) > nav.EPS:
                            station += math.dist(smooth[-1], stage['target'])
                            smooth.append(stage['target']); skeleton.append(stage['target'])
                    elif stage['kind'] == 'TRAVEL':
                        r = stage['route']
                        skeleton.extend(r['points'][1:]); smooth.extend(r['smoothed_points'][1:])
                        arrows.extend(dict(p, s_mm=p['s_mm']+station) for p in r['trajectory'])
                        if r.get('waypoint_program'):
                            markers.extend(r['points'][1:])
                        else:
                            markers.extend(p.get('end', p.get('exit')) for p in r['segment_program']['segments'])
                        station += r['trajectory_length_mm']
                self.map_view.set_path(smooth, waypoint_points=markers)
                pivots=[pivot for stage in match['stages'] if stage['kind']=='TRAVEL'
                        for pivot in stage['route'].get('pivots',[])]
                self.map_view.set_pivots(pivots)
                self.map_view.set_skeleton(skeleton)
                self.map_view.set_trajectory(arrows)
                self.plan_text.setPlainText('\n'.join('%02d %s%s' % (i+1, s['label'],
                    (' · 坐标目标%d个' % (len(s['route']['waypoint_program']['waypoints'])-1)
                     if s['route'].get('waypoint_program') else ' · 端点/圆弧%d段' % len(s['route']['segment_program']['segments']))
                    if s['kind']=='TRAVEL' else '')
                    for i, s in enumerate(match['stages'])))
                self._set_plan_info('坐标闭环比赛预检通过：%d个障碍；%s，平移/转头同时控制，实际响应扫掠复检' %
                    (len(match['sim_obstacles']),('麦轮支点转弯%d处'%len(pivots)) if pivots else '无圆弧'), 'warn')
                self.map_status.setText('完整初赛自动模拟：扫码→两批搬运→同色码垛→返回')
                def valid():
                    return (self.sim is sim and self.worker is None and self.nav_map == ctx['snapshot'] and
                            tuple(tuple(p) for p in self.sim_obstacles) == ctx['obstacles'] and
                            (self.map_ox, self.map_oy, self.map_theta) == ctx['mapping'] and
                            sim.chassis_parameters() == ctx['chassis_control'] and
                            not event.is_set())
                self.competition = CompetitionRunner(sim, match, ctx['mapping'], valid)
                self._journal_begin('SIM_MATCH', match)
                self.competition.start()
                self.log('SIM ONLY 初赛一键启动：扫码→两批原料/粗加工/暂存→同色码垛→返回启停区%d' % ctx['zone'], 'info')
            except Exception as exc:
                self._cancel_competition(str(exc))
                self.competition_status.setText('比赛模拟启动失败：'+str(exc))
                return
        runner = getattr(self, 'competition', None)
        self._observe_protective_stop()
        if runner is None or not runner.active:
            return
        snap = runner.sim.navigation_snapshot()
        if time.monotonic()-snap['frame_time'] > core.TELEMETRY_WARN_S:
            runner.cancel('模拟定位过期，比赛已停止')
            self._home_after_stop = True
        else:
            runner.tick()
        self._observe_protective_stop()
        self.map_view.set_reference(runner.sim.navigation_snapshot()['reference'] if runner.active else None)
        label = runner.stage['label'] if runner.stage else '返回启停区，全部任务完成'
        cargo = ','.join(str(c) for c in runner.cargo) or '空'
        self.competition_status.setText('%s · 障碍%d · %.1f/180s · %s · 任务码 %s · 车载[%s] · 抓取%d/12 放置%d/12%s' % (
            'PAUSED' if snap.get('paused') else runner.status, len(runner.match['sim_obstacles']), runner.elapsed_s, label, runner.display_code or '未读取', cargo,
            runner.grabs, runner.placements, (' · '+runner.reason) if runner.reason else ''))
        if not runner.active:
            self.run_journal.end_run(runner.status, runner.reason)
            self.competition_start.setEnabled(True)
            self.map_view.set_reference(None)
            self.log('比赛模拟%s：%.1fs，抓取%d，放置%d；%s' % (
                runner.status, runner.elapsed_s, runner.grabs, runner.placements, runner.reason),
                'info' if runner.status == 'COMPLETE' else 'warn')

    def _export_competition(self):
        runner = getattr(self, 'competition', None)
        if runner is None:
            self.competition_status.setText('尚无比赛模拟记录')
            return
        path, _ = QFileDialog.getSaveFileName(self, '导出完整比赛模拟记录', 'ilhc-competition.json', 'JSON (*.json)')
        if path:
            try:
                Path(path).write_text(json.dumps(runner.export_snapshot(), ensure_ascii=False, indent=2,
                    allow_nan=False, default=lambda obj: obj.as_dict()), encoding='utf-8')
            except (ValueError, TypeError, AttributeError, OSError) as exc:
                self.competition_status.setText('比赛记录导出失败：'+str(exc))

    def _toggle_follow(self):
        if self.worker is not None and self.sim is None:
            if self._real_future is not None or (self.real_match is not None and self.real_match.active):
                self._stop_navigation('已停止实机轨迹')
                return
            self._start_real_path(self.planned_result, self._planned_context)
            return
        if self.follow is not None:
            self._stop_navigation("已取消连续跟踪并停止模拟车辆")
            return
        if self.sim is None or self.worker is not None:
            self.map_status.setText("请先连接STM32或开启PC模拟")
            return
        res, ctx = self.planned_result, self._planned_context
        if (not res or not ctx or not res.get("execution_safe") or
                ctx["signature"] != self._navigation_signature()):
            self.map_status.setText("没有连续安全Trajectory（含停转/转向fallback时不能跟踪），请重新规划")
            return
        sim = self.sim
        snap = sim.navigation_snapshot()
        if (not snap["wheel_enabled"] or self.wheel_state is False or snap["hold"] is None or
                time.monotonic()-snap["frame_time"] > core.TELEMETRY_WARN_S or
                snap["manual"] is not None or snap["goto"] is not None):
            self.map_status.setText("模拟定位未就绪/过期、轮失能或已有运动，拒绝启动")
            return
        current = self._ops_to_field(*snap["hold"])
        if (math.dist(current, ctx["start"]) > 5 or
                abs((snap["yaw"]-ctx["yaw"]+180) % 360-180) > 1e-6):
            self._clear_path("实际起点/航向已改变，请重新规划")
            return
        if not res.get("trajectory_safe") or not res.get("trajectory_continuous"):
            self.map_status.setText("没有连续安全Trajectory（含停转/转向fallback时不能跟踪），请重新规划")
            return
        scene = self._scene_for_context(ctx)
        mapping = (self.map_ox, self.map_oy, self.map_theta)
        map_snapshot = copy.deepcopy(self.nav_map)
        # 模拟器工作线程只读取普通Python状态，不读取Qt控件。
        def valid():
            return (self.sim is sim and self.worker is None and self.nav_map == map_snapshot and
                    (self.map_ox, self.map_oy, self.map_theta) == mapping)
        self._manual_stop()
        if self.planned_result is not res or ctx["signature"] != self._navigation_signature():
            self.map_status.setText("控制模式已变化，请重新规划")
            return
        try:
            epoch = sim.begin_navigation()
            program = res.get('waypoint_program', res.get('segment_program'))
            submit = sim.submit_navigation_coordinates if res.get('waypoint_program') else sim.submit_navigation_segments
            goal_id = submit(epoch, program, mapping, scene, validity=valid)
        except (ValueError, TypeError, RuntimeError) as exc:
            sim.cancel_navigation()
            self._clear_path("连续跟踪启动拒绝："+str(exc))
            self.map_status.setText("连续跟踪启动拒绝："+str(exc))
            return
        now = time.monotonic()
        self._home_after_stop = False
        self.follow = dict(sim=sim, epoch=epoch, goal_id=goal_id, scene=scene, signature=ctx["signature"],
                           mapping=mapping, final=dict(program['goal']), t0=now,
                           timeout=max(25.0, res["trajectory_length_mm"]/max(1.0, sim._nav_speed_mm_s)*3+5),
                           frame_seq=-1, settled=0, progress_s_mm=0.0)
        if res.get('waypoint_program'):
            self.follow['timeout'] = max(25.0,res['predicted_tracking_s']*3+5)
        self._journal_begin('SIM_PATH', res)
        self.follow_btn.setText("停止跟踪")
        self.follow_timer.start()
        self.map_view.set_reference(sim.navigation_snapshot()["reference"])
        self.log("SIM ONLY %s程序已接受（id=%d），途中目标不等待停稳" %
                 ('关键坐标' if res.get('waypoint_program') else '端点/圆弧', goal_id), "info")

    def _stop_follow(self, msg=None):
        f = getattr(self, 'follow', None)
        self.follow = None
        sim = f["sim"] if f is not None else getattr(self, 'sim', None)
        # 接受整条Trajectory的复检阶段尚未建立UI follow时，STOP也必须撤销epoch。
        if sim is not None:
            snap=sim.navigation_snapshot()
            if f is not None or snap["active"] or snap["tracking_status"]=='DIRECT_RETURN':
                sim.cancel_navigation()
        if hasattr(self, 'map_view'):
            self.map_view.set_reference(None)
        if f is None:
            return
        self.run_journal.end_run('COMPLETE' if snap['tracking_status']=='COMPLETE' else 'CANCELLED', msg or '')
        core.discard_motion_commands(self.line_q)
        core.discard_motion_commands(self.urgent_q)
        self.follow_timer.stop()
        self.follow_btn.setText("执行规划路径")
        self.planned_result = None
        self._planned_context = None
        self._set_plan_info("连续跟踪结束/已取消；再次运行需重新规划", "idle")
        if msg:
            self.map_status.setText(msg)
            self.log(msg, "info")

    def _at_final_stop(self, snap=None):
        f = self.follow
        if f is None:
            return False
        snap = snap or f["sim"].navigation_snapshot()
        if (snap["epoch"] != f["epoch"] or snap["goal_id"] != f["goal_id"] or
                snap["completed_id"] != f["goal_id"] or snap["tracking_status"] != "COMPLETE" or
                snap["hold"] is None or time.monotonic()-snap["frame_time"] > core.TELEMETRY_WARN_S or
                snap["settled_frames"] < 10 or snap["speed_mm_s"] > 1 or abs(snap["yaw_rate_deg_s"]) > 1):
            return False
        lx, ly = self._ops_to_field(*snap["hold"])
        actual = core.layout_to_field(lx, ly)
        final = f["final"]
        return (math.dist(actual, (final["x_mm"], final["y_mm"])) < 1 and
                abs((self._field_math_heading(snap["yaw"])-final["field_yaw_deg"]+180) % 360-180) < 1)

    def _follow_step(self):
        f = self.follow
        if f is None:
            return
        if self.worker is not None or self.sim is not f["sim"] or f["signature"] != self._navigation_signature():
            self._stop_follow("地图/标定/参数/链路改变，连续跟踪已取消")
            return
        snap = f["sim"].navigation_snapshot()
        if (snap["epoch"] != f["epoch"] or snap["goal_id"] != f["goal_id"] or
                snap["fault"] or not snap["wheel_enabled"]):
            self._home_after_stop = True
            self._stop_follow("模拟控制权失效，连续跟踪已停止："+snap["fault"])
            return
        if time.monotonic()-snap["frame_time"] > core.TELEMETRY_WARN_S:
            self._home_after_stop = True
            self._stop_follow("模拟定位过期，连续跟踪已停止")
            return
        if snap.get("paused"):
            self.map_status.setText("模拟轨迹已暂停，可继续车辆")
            return
        if time.monotonic()-f["t0"] > f["timeout"]:
            self._home_after_stop = True
            self._stop_follow("连续跟踪总超时，已停止")
            return
        if self._at_final_stop(snap):
            self._stop_follow("最终STOP点位置+航向到位并连续停稳200ms（PC仿真）")
            return
        if not snap["active"] or not snap["tracking"]:
            self._stop_follow("连续跟踪状态失效，已停止")
            return
        if snap["frame_seq"] == f["frame_seq"]:
            return
        f.update(frame_seq=snap["frame_seq"], settled=snap["settled_frames"],
                 progress_s_mm=snap["progress_s_mm"])
        self.map_view.set_reference(snap["reference"])
        ref = snap["reference"]
        if ref:
            self.map_status.setText("SIM %s %d/%d段 %s · s=%.1fmm · 参考X/Y=(%.1f,%.1f)mm Yaw=%.1f° · %s" % (
                '坐标闭环' if snap['execution_representation']=='COORDINATES' else '端点执行',
                ref.get('segment_index', 1), ref.get('segment_count', 1), ref['segment_type'], snap["progress_s_mm"],
                ref["x_mm"], ref["y_mm"], (90-ref["field_yaw_deg"]) % 360, snap["tracking_status"]))

    def _load_navigation_map(self):
        path, _ = QFileDialog.getOpenFileName(self, "加载合法行驶区域与障碍", "", "JSON (*.json)")
        if not path:
            return
        self._clear_path("开始加载地图，旧计划已作废")
        try:
            self.nav_map = nav.load_map(path)
            self._sync_work_points()
            mode=self.nav_map.get('turn_mode','WHEEL')
            index=self.turn_mode_combo.findData(mode)
            if index<0:raise ValueError('地图转弯模式无效')
            self.turn_mode_combo.blockSignals(True)
            self.turn_mode_combo.setCurrentIndex(index)
            self.turn_mode_combo.blockSignals(False)
            self._ops_zero_pending = None
            self._ops_zero_failure = ''
            self._ops_zero_result = dict(state='MAP_CHANGED')
            self._update_origin_controls()
            if self._uses_ops_origin():
                self.map_ox = self.map_oy = self.map_theta = 0.0
                for widget in (self.map_ox_spin,self.map_oy_spin,self.map_theta_spin):
                    widget.setValue(0)
                self._ops_zero_failure = '已加载实测OPS地图，请在测量时的出发位置与朝向置零并同步地图'
            self.map_view.set_navigation_map(self.nav_map)
            self.map_status.setText("已加载地图 %s v%s；整车复检通过可整批执行" %
                                    (self.nav_map["map_id"], self.nav_map["map_version"]))
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.map_status.setText("地图加载失败：" + str(exc))

    def _export_navigation_plan(self):
        if not self.planned_result or not self._planned_context:
            self.log("无有效计划，不能导出", "warn")
            return
        if self._planned_context["signature"] != self._navigation_signature():
            self._clear_path("计划版本过期，不能导出")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出规划快照（非指令）", "ilhc-plan.json", "JSON (*.json)")
        if not path:
            return
        data = copy.deepcopy(self.planned_result)
        for key in ("segments", "corners", "arcs"):
            data[key] = [item.as_dict() if hasattr(item, "as_dict") else item
                         for item in data.get(key, [])]
        data["map_snapshot"] = self.nav_map
        data["mapping"] = dict(ox_mm=self.map_ox, oy_mm=self.map_oy, theta_deg=self.map_theta)
        for key in ("min_clearance", "body_clearance", "boundary_clearance"):
            if data.get(key) is not None and not math.isfinite(data[key]):
                data[key] = None
        try:
            Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        except (OSError, ValueError, TypeError) as exc:
            self.log("导出失败：" + str(exc), "warn")

    def _export_trajectory(self):
        res, ctx = self.planned_result, self._planned_context
        if not res or not ctx:
            self.log("无有效计划，不能导出Trajectory", "warn")
            return
        if ctx["signature"] != self._navigation_signature():
            self._clear_path("计划版本过期，不能导出Trajectory")
            return
        if not res.get("trajectory_safe") or not res.get("trajectory_continuous"):
            self.log("Trajectory不可导出："+res.get("trajectory_reason", "未通过完整整车检查"), "warn")
            return
        if res.get('waypoint_program'):
            try:
                from coordinate_navigation import replay
                samples, elapsed = replay(res['waypoint_program'], self._scene_for_context(ctx))
                path, _ = QFileDialog.getSaveFileName(self, '导出坐标控制预演轨迹', 'ilhc-coordinate-replay.json', 'JSON (*.json)')
                if path:
                    Path(path).write_text(json.dumps(dict(kind='PC_COORDINATE_REPLAY', hardware_ready=False,
                        waypoint_program=res['waypoint_program'], trajectory=samples, elapsed_s=elapsed),
                        ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
                    self.log('坐标控制预演轨迹已导出；仅PC模拟', 'info')
            except (OSError, ValueError, TypeError) as exc:
                self.log('坐标预演导出失败：'+str(exc), 'warn')
            return
        check = core.validate_trajectory(res["trajectory"], res["smoothed_primitives"],
                                         self._scene_for_context(ctx))
        if not check["ok"]:
            res.update(trajectory=[], trajectory_safe=False, trajectory_continuous=False,
                       trajectory_status=check["code"], trajectory_reason=check["reason"])
            self.map_view.set_trajectory(None)
            self.plan_text.setPlainText("\n".join(self._waypoint_lines()))
            self._set_plan_info(self._plan_summary(res), "warn")
            self.log("Trajectory导出前复检失败："+check["reason"], "warn")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出Trajectory（仅几何，不下发）", "ilhc-trajectory.json", "JSON (*.json)")
        if not path:
            return
        try:
            k = ctx["kwargs"]
            core.export_trajectory_json(path, res, metadata=dict(
                plan_id=res["plan_id"], map_id=res["map_id"], map_version=res["map_version"],
                geometry_verified=res.get("geometry_verified", False),
                collision_frame_id="LAYOUT_MM", collision_snapshot={key: k.get(key) for key in (
                    "rects", "circles", "dynamic_rects", "dynamic_circles", "bounds", "pad", "footprint", "drivable_polygons")}))
            self.log("已导出Trajectory：%d点，%.1fmm（未下发）" % (len(res["trajectory"]), res["trajectory_length_mm"]), "info")
        except (OSError, ValueError, TypeError) as exc:
            self.log("Trajectory导出失败："+str(exc), "warn")

    def _export_segments(self):
        from segment_route import export_segment_json, validate_segment_program
        res, ctx = self.planned_result, self._planned_context
        if not res or not ctx or not (res.get('segment_program') or res.get('waypoint_program')) or ctx['signature'] != self._navigation_signature():
            self.log('没有当前版本的安全PC路径程序，请重新规划', 'warn')
            return
        try:
            if res.get('waypoint_program'):
                from coordinate_navigation import replay, export_json
                replay(res['waypoint_program'], self._scene_for_context(ctx))
                path, _ = QFileDialog.getSaveFileName(self, '导出PC关键坐标程序', 'ilhc-coordinates.json', 'JSON (*.json)')
                if path:
                    export_json(path, res['waypoint_program'])
                    self.log('关键坐标程序已保存到电脑；未下发实机', 'info')
                return
            validate_segment_program(res['segment_program'], self._scene_for_context(ctx))
            path, _ = QFileDialog.getSaveFileName(self, '导出PC端点/圆弧程序', 'ilhc-segments.json', 'JSON (*.json)')
            if not path:
                return
            export_segment_json(path, res['segment_program'], metadata=dict(
                map_id=res['map_id'], map_version=res['map_version'],
                collision_frame_id='LAYOUT_MM', collision_snapshot={key: copy.deepcopy(ctx['kwargs'].get(key)) for key in (
                    'rects', 'circles', 'dynamic_rects', 'dynamic_circles', 'bounds', 'pad', 'footprint', 'drivable_polygons')}))
            self.log('已导出端点/圆弧程序：%d段，无密集点表（仅PC，未下发）' %
                     len(res['segment_program']['segments']), 'info')
        except (OSError, ValueError, TypeError) as exc:
            self.log('端点/圆弧导出失败：'+str(exc), 'warn')

    # ---------------- 控制台 ----------------
    def _send_console(self):
        text = self.command_entry.text().strip()
        if not text:
            return
        self.send_line(text)
        self.cmd_history.append(text)
        self.hist_idx = len(self.cmd_history)
        self.command_entry.clear()

    def _journal_metadata(self):
        return dict(pc_version='2.1.23',turn_mode=self.nav_map.get('turn_mode','WHEEL'),mapping=[self.map_ox,self.map_oy,self.map_theta],
            ops_zero_calibration=copy.deepcopy(getattr(self,'_ops_zero_result',{'state':'NOT_REQUESTED'})),
            python_version=sys.version,connection=dict(source='REAL' if self.worker is not None else 'SIM',
                port=getattr(self.worker,'port',None),baud=getattr(self.worker,'baud',None),
                link_mode=getattr(self.worker,'link_mode',None)),
            map_snapshot=copy.deepcopy(self.nav_map),obstacles=copy.deepcopy(self.sim_obstacles),
            chassis_telemetry=None if self.latest is None else list(self.latest[6:11]),
            ops_offset_inputs_mm=[self.ops_offset_x.value(),self.ops_offset_y.value()],
            ops_offset_inputs_are_not_readback=True,task_code=self.competition_code.text(),
            zone=int(self.zone_combo.currentData()),runtime_collision_protection=self.real_collision_check.isChecked())

    def _ensure_run_journal(self):
        source=self.worker if self.worker is not None else self.sim
        if source is None:
            return False
        if self._journal_source is not source:
            self._journal_source=source
            self._journal_session_metadata=self._journal_metadata()
            self.run_journal.start('REAL' if self.worker is not None else 'SIM',self._journal_session_metadata)
            source.event_cb=lambda event, **data:self._journal_wire_event(source,event,**data)
            self._update_run_log_status()
        return True

    def _journal_wire_event(self, source, event, **data):
        # 运行在线程中；只读会话身份并入队，不读取Qt控件，也不改变控制输出。
        if source is self._journal_source:
            command=data.get('command','').strip().upper()
            pending = getattr(self, '_ops_zero_pending', None)
            if pending is not None and pending['source'] is source and command==pending['command'] and event in ('TX','SIM_COMMAND') and data.get('successful'):
                pending['sent'] = time.monotonic()
            if event in ('TX','SIM_COMMAND') and data.get('successful'):
                if command.startswith(('GOTO=', 'GOTOHOLD=', 'MANUAL=', 'ZDT=', 'VTRACK=')) and command!='MANUAL=0,0,0' and self.run_journal.active_run is None:
                    self.run_journal.begin_run('DIRECT_COMMAND',self._journal_session_metadata,{'command':command})
            self.run_journal.emit(event, **data)
            if event=='RX_TEXT':
                diagnostic=parse_ops_text(data.get('text',''))
                if diagnostic is not None:self.run_journal.emit('OPS_DIAGNOSTIC',**diagnostic)
            if self.run_journal.active_kind not in ('COORDINATE_BATCH','SIM_MATCH','SIM_PATH') and event in ('TX','SIM_COMMAND') and data.get('successful') and command in ('STOP','WHEELOFF','ZERO','MANUAL=0,0,0'):
                self.run_journal.end_run('STOP_REQUESTED',command+'已交给控制端')

    def _journal_begin(self, kind, plan):
        if self._ensure_run_journal():
            metadata=self._journal_metadata()
            metadata['chassis_control']=copy.deepcopy(plan.get('chassis_control'))
            self.run_journal.begin_run(kind,metadata,plan)
            self._journal_last_batch=None
            self._update_run_log_status()

    def _set_run_logging(self, enabled):
        self.run_journal.set_enabled(enabled)
        self._update_run_log_status()

    def _update_run_log_status(self):
        if not self.run_journal.enabled:
            text='运行日志已关闭（实机/模拟）；勾选后记录下一次运行。'
        elif self.run_journal.active_run is not None:
            text='运行日志正在记录：'+str(self.run_journal.path)
        elif self.run_journal._sequence:
            text='运行日志已结束，下次运行另建：'+str(self.run_journal.path)
        else:
            text='运行日志待机：开始运行后记录，跑完或STOP后结束。'
        if self.run_log_status.text()!=text:
            self.run_log_status.setText(text)

    def _open_run_logs(self):
        folder=self.run_journal.root
        folder.mkdir(parents=True,exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))

    def log(self, text: str, tag: str = "info"):
        self.run_journal.emit('UI_MESSAGE', text=str(text), tag=tag)
        if not hasattr(self, "console"):
            return
        color = {"tx": GREEN, "warn": RED, "info": FG_DIM}.get(tag, FG_DIM)
        stamp = time.strftime("%H:%M:%S")
        safe = html.escape(str(text))
        self.console.append('<span style="color:%s">[%s] %s</span>' % (color, stamp, safe))

    def closeEvent(self, event: QCloseEvent):
        if hasattr(self, "dm_position_panel"):
            self.dm_position_panel.timer.stop()
            self.dm_position_panel.session_reset()
        self._clear_path()
        self._planner_pool.shutdown(wait=False, cancel_futures=True)
        self._manual_stop()
        if self.worker is not None:
            self._stop_worker(safe=True)
        if not self._previous_worker_finished():
            # 不在后台线程仍使用Qt桥接对象时销毁窗口；稍后自动重试退出。
            event.ignore()
            QTimer.singleShot(100, self.close)
            return
        try:
            # 先把页面收回主窗口，独立窗口不会再拦截退出。
            self._reattach_all()
            if self.recorder is not None:
                self.recorder.close()
                self.recorder = None
            if self.sim is not None:
                self.sim.cancel_navigation()
                self.sim.stop_flag = True
                if self.sim.is_alive():
                    self.sim.join(timeout=0.5)
                self.sim = None
        finally:
            self.run_journal.close()
            event.accept()


def parse_args():
    ap = argparse.ArgumentParser(description="ILHC Control Station v2.1.23")
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
