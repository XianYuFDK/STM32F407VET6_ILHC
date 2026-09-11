# -*- coding: utf-8 -*-
"""
ILHC 调试上位机 v1.1
======================================================================
配套工程：STM32F407VET6_ILHC（Hardware/debug_usart.c 调试模块）

协议（与固件一一对应）：
  遥测（USART1 -> PC，50 Hz）：
      24 x float(小端) + 0x00 0x00 0x80 0x7F，共 100 字节/帧
  命令（PC -> USART1，ASCII，每行一条，\\r 或 \\n 结尾，大小写不敏感）：
      STOP / ZERO
      KPX= KPY= KPZ=          (0 ~ 50)
      XVMAX= ZVMAX=           (0 ~ 3000)
      XVMIN= ZVMIN=           (0 ~ 100)
      DMID= DMMODE= DMPOS= DMVEL= DMKP= DMKD= DMTOR=
      DMEN / DMOFF / DMSTOP / DMZERO
  固件无文本应答，参数回读通过遥测通道（mKpx / dm_cmd_xxx 等）实现。

功能：
  1. 串口连接（115200 8N1），实时解析 JustFloat 帧
  2. 24 通道分组波形 + 比赛场地地图（点击地图下发 GOTO，小车驶向目标点）
  3. 底盘 / DM 电机可视化调参（滑条 + 输入框 + 一键下发 + 回读）
  4. 命令行（历史记录）、CSV 数据记录、DM 故障码中文提示
  5. 模拟模式：复刻固件解析行为，无硬件也可演示/测试
  6. v1.1 安全增强：GUI 心跳、急停优先、遥测超时、地图静态禁区保护

运行：python ilhc_debugger.py [--simulate] [--port COMx] [--baud 115200]
自检：python ilhc_debugger.py --selftest
"""

import argparse
import csv
import math
import os
import queue
import random
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
# --selftest 必须能在无桌面/CI 环境运行；GUI 模式仍使用 TkAgg。
matplotlib.use("Agg" if "--selftest" in sys.argv else "TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

import numpy as np  # noqa: E402
try:
    import serial  # noqa: E402
    import serial.tools.list_ports  # noqa: E402
except ImportError:
    serial = None  # --selftest / --simulate 仍可使用；真实串口连接时给出明确提示

APP_VERSION = "v1.1"
FRAME_TAIL = b"\x00\x00\x80\x7F"
FRAME_FLOATS = 24
FRAME_DATA_LEN = 4 * FRAME_FLOATS
FRAME_LEN = FRAME_DATA_LEN + 4                 # 100 字节
DEFAULT_BAUD = 115200
SEND_HZ = 50                                   # 固件 20 ms 一帧
HEARTBEAT_INTERVAL_S = 0.20                      # GUI 存活心跳，5 Hz
TELEMETRY_WARN_S = 0.35                          # 遥测延迟黄色阈值
TELEMETRY_TIMEOUT_S = 1.00                       # 遥测超时红色阈值
URGENT_COMMANDS = {"STOP", "DMSTOP", "DMOFF"}
APP_NAME = "ILHC 调试上位机"

# ======================================================================
# 通道定义（与 debug_usart.c 中 data[0..23] 严格一致）
#   (下标, 键名, 显示名, 单位, 分组)
# ======================================================================
G_CHASSIS_POS = 0      # OPS 定位
G_CHASSIS_PID = 1      # 底盘参数 / 轮速
G_DM_FB = 2            # DM 电机反馈
G_DM_CMD = 3           # DM 电机目标

CHANNELS = [
    (0,  "pos_x",        "OPS X 坐标",   "mm",   G_CHASSIS_POS),
    (1,  "pos_y",        "OPS Y 坐标",   "mm",   G_CHASSIS_POS),
    (2,  "zangle",       "航向角",       "°",    G_CHASSIS_POS),
    (3,  "devx",         "X 轴误差",     "",     G_CHASSIS_POS),
    (4,  "devy",         "Y 轴误差",     "",     G_CHASSIS_POS),
    (5,  "devz",         "航向误差",     "",     G_CHASSIS_POS),
    (6,  "mKpx",         "X 轴 P",       "",     G_CHASSIS_PID),
    (7,  "mKpy",         "Y 轴 P",       "",     G_CHASSIS_PID),
    (8,  "mKpz",         "航向 P",       "",     G_CHASSIS_PID),
    (9,  "XYVmax",       "XY 限幅",      "",     G_CHASSIS_PID),
    (10, "ZVmax",        "Z 限幅",       "",     G_CHASSIS_PID),
    (11, "SpeedTarget0", "1 号轮目标",   "",     G_CHASSIS_PID),
    (12, "dm_id",        "DM 电机 ID",   "",     G_DM_FB),
    (13, "dm_pos",       "DM 实际位置",  "rad",  G_DM_FB),
    (14, "dm_vel",       "DM 实际速度",  "rad/s", G_DM_FB),
    (15, "dm_torque",    "DM 实际力矩",  "Nm",   G_DM_FB),
    (16, "dm_status",    "DM 状态码",    "",     G_DM_FB),
    (17, "dm_t_mos",     "DM MOS 温度",  "°C",   G_DM_FB),
    (18, "dm_t_rotor",   "DM 线圈温度",  "°C",   G_DM_FB),
    (19, "dm_cmd_pos",   "DM 目标位置",  "rad",  G_DM_CMD),
    (20, "dm_cmd_vel",   "DM 目标速度",  "rad/s", G_DM_CMD),
    (21, "dm_kp",        "DM Kp",        "",     G_DM_CMD),
    (22, "dm_kd",        "DM Kd",        "",     G_DM_CMD),
    (23, "dm_torque_cmd", "DM 前馈力矩", "Nm",   G_DM_CMD),
]

GROUP_NAMES = {
    G_CHASSIS_POS: "OPS 定位",
    G_CHASSIS_PID: "底盘参数 / 轮速",
    G_DM_FB:       "DM 电机反馈",
    G_DM_CMD:      "DM 电机目标",
}

# 每个分组对应的波形子图位置（2x2）
GROUP_AX_POS = {G_CHASSIS_POS: (0, 0), G_CHASSIS_PID: (0, 1),
                G_DM_FB: (1, 0), G_DM_CMD: (1, 1)}

# DM 状态码 -> (中文含义, 是否故障)
DM_STATUS = {
    0x00: ("失能", False), 0x01: ("使能", False),
    0x08: ("过压", True),  0x09: ("欠压", True),
    0x0A: ("过流", True),  0x0B: ("MOS 过温", True),
    0x0C: ("线圈过温", True), 0x0D: ("通信丢失", True),
    0x0E: ("过载", True),
}

# 底盘可调参数（与 debug_usart.c 参数表一致）：
#   (命令, 显示名, 最小, 最大, 默认, 回读通道下标或 None)
CHASSIS_PARAMS = [
    ("KPX",   "X 轴 P 系数",   0.0, 50.0,   2.3,  6),
    ("KPY",   "Y 轴 P 系数",   0.0, 50.0,   2.3,  7),
    ("KPZ",   "航向 P 系数",   0.0, 50.0,   9.0,  8),
    ("XVMAX", "X/Y 速度限幅",  0.0, 3000.0, 1600.0, 9),
    ("ZVMAX", "航向速度限幅",  0.0, 3000.0, 750.0, 10),
    ("XVMIN", "X/Y 最小补偿",  0.0, 100.0,  5.0,  None),
    ("ZVMIN", "航向最小补偿",  0.0, 100.0,  5.0,  None),
]

# DM 可调参数：(命令, 显示名, 最小, 最大, 默认, 回读通道, 是否整数)
DM_PARAMS = [
    ("DMID",  "电机 ID",     1.0, 1791.0,  1.0,  12, True),
    ("DMPOS", "目标位置 rad", -12.5, 12.5, 0.0,  19, False),
    ("DMVEL", "目标速度 rad/s", -30.0, 30.0, 0.0, 20, False),
    ("DMKP",  "MIT Kp",      0.0, 500.0,   2.0,  21, False),
    ("DMKD",  "MIT Kd",      0.0, 5.0,     0.5,  22, False),
    ("DMTOR", "前馈力矩 Nm",  -10.0, 10.0,  0.0,  23, False),
]

# 曲线配色（24 色）
COLORS = ["#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8", "#fff176",
          "#4db6ac", "#f06292", "#7986cb", "#aed581", "#ff8a65", "#90a4ae",
          "#26c6da", "#ffa726", "#66bb6a", "#ef5350", "#ab47bc", "#ffee58",
          "#26a69a", "#ec407a", "#5c6bc0", "#9ccc65", "#ff7043", "#78909c"]

# 深色主题
BG        = "#1b1d22"   # 窗口背景
BG_PANEL  = "#22252b"   # 面板背景
BG_ENTRY  = "#15171b"   # 输入框背景
FG        = "#d9dee6"   # 前景文字
FG_DIM    = "#8b93a1"   # 次要文字
ACCENT    = "#4fc3f7"   # 强调色
PLOT_BG   = "#1e2127"
GRID_COL  = "#343945"

# 启停区中心（场地坐标 mm，原点=场地左下角）
ZONE_CENTER = {1: (2250.0, 2250.0), 2: (2250.0, 150.0)}
# 快速前往目标点（避开功能区边缘的可达点）
QUICK_TARGETS = [
    ("原料区", (1200.0, 2200.0)),
    ("暂存区", (330.0, 1200.0)),
    ("粗加工区", (1200.0, 330.0)),
]

# 地图直线 GOTO 的静态禁区（与 _draw_field 中场地几何一致）。
# 注意：这里按设备实际绘制边界判断，未额外膨胀车体半径；若车体较宽，建议
# 在固件路径规划或后续上位机 A* 中加入车体半宽 + 安全裕量。
FIELD_FORBIDDEN_RECTS = [
    (550.0, 550.0, 1000.0, 1000.0, "中央物料区"),
    (1400.0, 550.0, 1850.0, 1000.0, "中央物料区"),
    (550.0, 1400.0, 1000.0, 1850.0, "中央物料区"),
    (1400.0, 1400.0, 1850.0, 1850.0, "中央物料区"),
    (0.0, 910.0, 150.0, 1490.0, "暂存区设备"),
    (1000.0, 0.0, 1400.0, 150.0, "粗加工区设备"),
    (2388.0, 1170.0, 2400.0, 1230.0, "二维码板"),
]
FIELD_FORBIDDEN_CIRCLES = [
    (1200.0, 2400.0, 110.0, "原料区圆盘"),
]

MPL_RC = {
    "figure.facecolor": BG_PANEL, "axes.facecolor": PLOT_BG,
    "axes.edgecolor": GRID_COL, "axes.labelcolor": FG,
    "xtick.color": FG_DIM, "ytick.color": FG_DIM,
    "grid.color": GRID_COL, "grid.linewidth": 0.6,
    "text.color": FG, "font.size": 8.5,
    "font.family": ["Microsoft YaHei", "SimHei", "Microsoft YaHei UI", "sans-serif"],
}


# ======================================================================
# 帧解析器：字节流 -> 24 通道 float
# ======================================================================
class FrameParser:
    """标准 VOFA+ JustFloat 解析器。

    帧格式：24*float + 0x00 0x00 0x80 0x7F（100 字节）。
    首次以帧尾同步；同步后按固定 100 字节帧长解析。这样即使 payload 中
    某个 float 的字节序列恰好等于 JustFloat 帧尾（例如 +inf），也不会
    把 payload 内部字节误判为真正帧尾。失步时自动回到搜索模式。
    """

    def __init__(self):
        self.buf = bytearray()
        self.frames_ok = 0
        self.bytes_in = 0
        self.err_bytes = 0
        self.synced = False

    def _accept(self, payload):
        values = struct.unpack("<%df" % FRAME_FLOATS, bytes(payload))
        self.frames_ok += 1
        return values

    def feed(self, data):
        """输入任意长度字节流，返回解析出的 float 元组列表。"""
        self.buf += data
        self.bytes_in += len(data)
        frames = []

        while True:
            # 已同步后严格按固定长度取帧，不再搜索 payload 内部的伪帧尾。
            if self.synced:
                if len(self.buf) < FRAME_LEN:
                    break
                if self.buf[FRAME_DATA_LEN:FRAME_LEN] == FRAME_TAIL:
                    frames.append(self._accept(self.buf[:FRAME_DATA_LEN]))
                    del self.buf[:FRAME_LEN]
                    continue
                # 固定位置帧尾不匹配：失步，转入重新同步。
                self.synced = False

            # 未同步：跳过不足 96 字节之前出现的帧尾（它可能位于 payload 内）。
            search_from = 0
            found = -1
            while True:
                i = self.buf.find(FRAME_TAIL, search_from)
                if i < 0:
                    break
                if i >= FRAME_DATA_LEN:
                    found = i
                    break
                search_from = i + 1

            if found < 0:
                # 保留最多 96 字节数据 + 3 字节可能被截断的帧尾。
                keep = FRAME_DATA_LEN + len(FRAME_TAIL) - 1
                if len(self.buf) > keep:
                    self.err_bytes += len(self.buf) - keep
                    del self.buf[:-keep]
                break

            start = found - FRAME_DATA_LEN
            if start > 0:
                self.err_bytes += start
            frames.append(self._accept(self.buf[start:found]))
            del self.buf[:found + len(FRAME_TAIL)]
            self.synced = True

        return frames


# ======================================================================
# numpy 环形缓冲：col0 = 相对时间，col1..N = 通道值
# 写入 O(1)；读取直接切片成 numpy 数组，绘制路径无 Python 级循环
# ======================================================================
class RingBuffer:
    def __init__(self, cap, cols):
        self.cols = cols
        self._alloc(cap)

    def _alloc(self, cap):
        self.cap = max(8, int(cap))
        self.buf = np.zeros((self.cap, self.cols + 1))
        self.head = 0
        self.count = 0

    def append(self, t, values):
        self.buf[self.head, 0] = t
        self.buf[self.head, 1:] = values
        self.head = (self.head + 1) % self.cap
        if self.count < self.cap:
            self.count += 1

    def resize(self, cap):
        old = self.view()
        self._alloc(cap)
        if old is not None:
            t, d = old
            if len(t) > self.cap:
                t, d = t[-self.cap:], d[-self.cap:]
            self.buf[:len(t), 0] = t
            self.buf[:len(t), 1:] = d
            self.head = len(t) % self.cap
            self.count = len(t)

    def clear(self):
        self.head = 0
        self.count = 0

    def view(self):
        """返回时间升序的 (t 数组, data 数组)；空缓冲返回 None"""
        if self.count == 0:
            return None
        if self.count < self.cap:
            b = self.buf[:self.count]
            return b[:, 0], b[:, 1:]
        b = np.concatenate((self.buf[self.head:], self.buf[:self.head]))
        return b[:, 0], b[:, 1:]


# ======================================================================
# 串口工作线程：收（解析遥测）+ 发（ASCII 命令）
# ======================================================================
class SerialWorker(threading.Thread):
    def __init__(self, port, baud, frame_q, line_q, urgent_q=None, err_cb=None):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.frame_q, self.line_q = frame_q, line_q
        self.urgent_q = urgent_q if urgent_q is not None else queue.Queue()
        self.err_cb = err_cb
        self.parser = FrameParser()
        self.stop_flag = False
        self.ser = None
        self.opened = threading.Event()
        self.last_frame_monotonic = 0.0
        self.opened_monotonic = 0.0
        self._write_lock = threading.Lock()

    def _write_line(self, line):
        if not self.ser or not self.ser.is_open:
            return False
        data = str(line).encode("ascii", "ignore") + b"\n"
        with self._write_lock:
            self.ser.write(data)
        return True

    @staticmethod
    def _drain_one(q):
        try:
            return q.get_nowait()
        except queue.Empty:
            return None

    def request_stop(self, safe=True):
        """请求工作线程退出；真实串口断开前 best-effort 主动停车/失能。"""
        # 先置退出标志并清除普通队列，避免安全指令之后又发送旧 GOTO/调参命令。
        self.stop_flag = True
        while True:
            try:
                self.line_q.get_nowait()
            except queue.Empty:
                break
        if safe:
            for cmd in ("STOP", "DMSTOP", "DMOFF"):
                try:
                    self._write_line(cmd)
                except Exception:
                    break

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, bytesize=8,
                                     parity=serial.PARITY_NONE, stopbits=1,
                                     timeout=0.05, write_timeout=0.10)
            self.opened_monotonic = time.monotonic()
            self.opened.set()
        except Exception as e:  # 打开失败，通知 UI
            if self.err_cb:
                self.err_cb("打开 %s 失败：%s" % (self.port, e))
            return
        try:
            while not self.stop_flag:
                # 急停/失能命令优先于读数据和普通参数命令。
                while True:
                    line = self._drain_one(self.urgent_q)
                    if line is None:
                        break
                    self._write_line(line)

                data = self.ser.read(2048)
                if data:
                    now = time.monotonic()
                    for f in self.parser.feed(data):
                        self.last_frame_monotonic = now
                        self._push((now, f))

                # 读完后再次检查急停，最大额外等待约为串口 timeout=50 ms。
                while True:
                    line = self._drain_one(self.urgent_q)
                    if line is None:
                        break
                    self._write_line(line)

                if self.stop_flag:
                    break

                # 普通命令分小批发送；每发一条都检查是否出现新的急停命令。
                for _ in range(16):
                    if self.stop_flag or not self.urgent_q.empty():
                        break
                    line = self._drain_one(self.line_q)
                    if line is None:
                        break
                    self._write_line(line)
        except Exception as e:
            if self.err_cb and not self.stop_flag:
                self.err_cb("串口异常断开：%s" % e)
        finally:
            self.opened.clear()
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

    def _push(self, item):
        try:
            self.frame_q.put_nowait(item)
        except queue.Full:
            try:
                self.frame_q.get_nowait()   # 丢弃最旧帧，保持实时性
                self.frame_q.put_nowait(item)
            except queue.Empty:
                pass


# ======================================================================
# 模拟器：复刻 debug_usart.c 的命令解析 / 参数回读行为（无硬件演示用）
# ======================================================================
def ops_offset_command(x_mm, y_mm):
    """安装偏移：车前为正X、车左为正Y；成对发送毫米参数。"""
    values = (float(x_mm), float(y_mm))
    if any(not math.isfinite(v) or abs(v) > 500 for v in values):
        raise ValueError("安装偏移必须在 -500..500 mm 内")
    return "OPSOFFSET=%.1f,%.1f" % values


class Simulator(threading.Thread):
    def __init__(self, frame_q, line_q, urgent_q=None):
        super().__init__(daemon=True)
        self.frame_q, self.line_q = frame_q, line_q
        self.urgent_q = urgent_q if urgent_q is not None else queue.Queue()
        self.stop_flag = False
        # 底盘参数（与固件默认值一致）
        self.kpx, self.kpy, self.kpz = 2.3, 2.3, 9.0
        self.xyvmax, self.zvmax = 1600.0, 750.0
        self.xyvmin, self.zvmin = 5.0, 5.0
        # DM 参数
        self.dm_id, self.dm_mode, self.dm_active = 1, 1, 0
        self.dm_pos = self.dm_vel = self.dm_tor = 0.0
        self.dm_kp, self.dm_kd = 2.0, 0.5
        # 模拟反馈
        self.fb_pos, self.fb_vel, self.fb_tor = 0.0, 0.0, 0.0
        self.fb_status, self.fb_tmos, self.fb_trotor = 0, 35.0, 33.0
        self.zero_x, self.zero_y, self.zero_z = 0.0, 0.0, 0.0
        # GOTO 定位移动模拟（点击场地地图后小车驶向目标）
        self.hold = None        # (x, y) 固定点位；None = 演示巡航
        self.goto = None        # (x, y, z) 目标
        self.zval = 0.0         # 当前航向（deg）
        self.ops_offset = (-50.0, 60.0)
        self.ops_reference_yaw = 0.0
        self.manual = None
        self.manual_tick = 0.0
        self._t = 0.0

    # ---------- 命令解析（镜像 Debug_ParseLine / Debug_SetDmValue） ----------
    @staticmethod
    def _clamp(v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    def handle_line(self, line):
        line = line.strip().upper()
        if not line:
            return
        if line.startswith("OPSOFFSET="):
            try:
                parts = line[10:].split(",")
                if len(parts) != 2:
                    return
                # 与固件一致，只接受十进制小数，不接受指数或空白。
                for p in parts:
                    number = p.lstrip("+-")
                    if p[:2] in ("++", "--", "+-", "-+") or number.count(".") > 1 or not number.replace(".", "").isascii() or not number.replace(".", "").isdigit():
                        return
                x, y = map(float, parts)
                ops_offset_command(x, y)
            except (ValueError, OverflowError):
                return
            self.ops_offset = (x, y)
            self.manual = self.goto = None
            self.hold = (0.0, 0.0)
            self.ops_reference_yaw = self.zval
            return
        if line.startswith("MANUAL="):
            parts = line[7:].split(",")
            if len(parts) != 3 or any(not p.lstrip("+-").isascii() or not p.lstrip("+-").isdigit() for p in parts):
                return
            try:
                v = tuple(int(p) for p in parts)
            except ValueError:
                return
            if any(abs(x) > 300 for x in v):
                return
            self.manual = v
            self.manual_tick = time.monotonic()
            self.goto = None
            if self.hold is None:
                self.hold = (600.0 * math.sin(0.25 * self._t), 450.0 * math.cos(0.19 * self._t))
            return
        if line == "STOP":
            self.manual = None
            if self.hold is None:                  # 原地停住（同固件急停）
                self.hold = (600.0 * math.sin(0.25 * self._t),
                             450.0 * math.cos(0.19 * self._t))
            self.goto = None                       # 取消 GOTO 目标（同固件）
            return
        if line == "ZERO":
            self.ops_reference_yaw = self.zval
            self.manual = None
            if self.hold is None:                  # 巡航中收到归零：停在当前位置
                self.hold = (600.0 * math.sin(0.25 * self._t),
                             450.0 * math.cos(0.19 * self._t))
            self.hold = (0.0, 0.0)                 # 以当前位置为新原点
            return
        if line.startswith("GOTO="):
            try:
                parts = [float(p) for p in line[5:].split(",") if p.strip()]
            except ValueError:
                return
            if len(parts) >= 2:
                self.manual = None
                if self.hold is None:              # 从演示巡航位置切入定位模式
                    self.hold = (600.0 * math.sin(0.25 * self._t),
                                 450.0 * math.cos(0.19 * self._t))
                self.goto = (self._clamp(parts[0], -3000, 3000),
                             self._clamp(parts[1], -3000, 3000),
                             parts[2] if len(parts) >= 3 else self.zval)
            return
        if line == "DMEN":
            self.dm_active = 1
            self.fb_status = 0x01
            return
        if line in ("DMOFF", "DMSTOP"):
            self.dm_active = 0
            self.fb_status = 0x00
            return
        if line == "DMZERO":
            self.fb_pos = 0.0
            return
        if "=" not in line:
            return
        name, _, val = line.partition("=")
        try:
            v = float(val)
        except ValueError:
            return
        if name == "KPX":    self.kpx = self._clamp(v, 0, 50)
        elif name == "KPY":  self.kpy = self._clamp(v, 0, 50)
        elif name == "KPZ":  self.kpz = self._clamp(v, 0, 50)
        elif name == "XVMAX": self.xyvmax = self._clamp(v, 0, 3000)
        elif name == "ZVMAX": self.zvmax = self._clamp(v, 0, 3000)
        elif name == "XVMIN": self.xyvmin = self._clamp(v, 0, 100)
        elif name == "ZVMIN": self.zvmin = self._clamp(v, 0, 100)
        elif name == "DMID":
            if 1 <= v <= 0x6FF:
                self.dm_id = int(v)
        elif name == "DMMODE":
            if 1 <= v <= 2:
                self.dm_mode = int(v)
        elif name == "DMPOS": self.dm_pos = self._clamp(v, -12.5, 12.5)
        elif name == "DMVEL": self.dm_vel = self._clamp(v, -30, 30)
        elif name == "DMKP":  self.dm_kp = self._clamp(v, 0, 500)
        elif name == "DMKD":  self.dm_kd = self._clamp(v, 0, 5)
        elif name == "DMTOR": self.dm_tor = self._clamp(v, -10, 10)

    # ---------- 生成一帧遥测（镜像 DebugUsart_Send 的 data[0..23]） ----------
    def make_frame(self, t):
        self._t = t
        if self.manual is not None:
            if time.monotonic() - self.manual_tick > 0.350:
                self.manual = None
            else:
                vx, vy, wz = self.manual
                angle = math.radians(self.zval)
                px, py = self.hold
                # 演示换算，不代表实车轮径、轮距标定结果。
                self.hold = (px + (vx * math.cos(angle) - vy * math.sin(angle)) / 0.238 / SEND_HZ,
                             py + (vx * math.sin(angle) + vy * math.cos(angle)) / 0.238 / SEND_HZ)
                self.zval += wz / SEND_HZ
        n = lambda a=1.0: random.gauss(0, a)     # noqa: E731
        if self.goto is not None and self.hold is not None:
            # GOTO 定位模式：以 500mm/s 限速驶向目标，航向最短路径逼近
            tx, ty, tz = self.goto
            px, py = self.hold
            ddx, ddy = tx - px, ty - py
            dist = math.hypot(ddx, ddy)
            if dist > 6.0:
                step = min(500.0 / SEND_HZ, dist)
                px += ddx / dist * step
                py += ddy / dist * step
                self.hold = (px, py)
            dz = (tz - self.zval + 180.0) % 360.0 - 180.0
            if abs(dz) > 1.0:
                self.zval += self._clamp(dz, -120.0 / SEND_HZ, 120.0 / SEND_HZ)
            else:
                self.zval = tz
                if dist <= 6.0:
                    self.goto = None             # 到位，原地保持
            pos_x = px + 3 * n()
            pos_y = py + 3 * n()
            zangle = (self.zval + 0.5 * n() + 180.0) % 360.0 - 180.0
            devx, devy = px - tx, py - ty        # 真实目标误差
            devz = dz
            spd0 = self._clamp(dist * 0.5, -500.0, 500.0) + 10 * n()
        elif self.hold is not None:
            # 到位保持 / STOP 后静止在原地
            px, py = self.hold
            pos_x = px + 3 * n()
            pos_y = py + 3 * n()
            zangle = (self.zval + 0.5 * n() + 180.0) % 360.0 - 180.0
            devx = devy = devz = 0.0
            spd0 = 10 * n()
        else:
            pos_x = self.zero_x + 600.0 * math.sin(0.25 * t) + 3 * n()
            pos_y = self.zero_y + 450.0 * math.cos(0.19 * t) + 3 * n()
            zangle = ((self.zero_z + 20.0 * math.sin(0.4 * t) + 0.8 * n())
                      + 180.0) % 360.0 - 180.0
            devx = 12.0 * math.sin(0.5 * t) + 2 * n()
            devy = 9.0 * math.cos(0.42 * t) + 2 * n()
            devz = 1.5 * math.sin(0.6 * t) + 0.3 * n()
            spd0 = 380.0 * math.sin(0.8 * t) + 15 * n()
            self.zval = zangle
        if self.dm_active:
            # 位置以限速逼近目标，模拟闭环
            err = self.dm_pos - self.fb_pos
            vmax = max(0.5, abs(self.dm_vel))
            step = self._clamp(err, -vmax / SEND_HZ, vmax / SEND_HZ)
            self.fb_pos += step
            self.fb_vel = step * SEND_HZ
            self.fb_tor = self._clamp(self.dm_kp * err * 0.01, -10, 10)
            self.fb_status = 0x01
            self.fb_tmos = min(70.0, self.fb_tmos + 0.006) + 0.05 * n()
            self.fb_trotor = min(80.0, self.fb_trotor + 0.008) + 0.05 * n()
        else:
            self.fb_vel *= 0.9
            self.fb_tor *= 0.9
            self.fb_status = 0x00
            self.fb_tmos = max(32.0, self.fb_tmos - 0.01) + 0.02 * n()
            self.fb_trotor = max(30.0, self.fb_trotor - 0.012) + 0.02 * n()
        if self.hold is not None:
            angle, ref = math.radians(self.zval), math.radians(self.ops_reference_yaw)
            dc, ds = math.cos(angle) - math.cos(ref), math.sin(angle) - math.sin(ref)
            ex, ey = -50.0 - self.ops_offset[0], 60.0 - self.ops_offset[1]
            pos_x -= dc * ex - ds * ey
            pos_y -= ds * ex + dc * ey
        return (
            pos_x, pos_y, zangle, devx, devy, devz,
            self.kpx, self.kpy, self.kpz, self.xyvmax, self.zvmax, spd0,
            float(self.dm_id), self.fb_pos, self.fb_vel, self.fb_tor,
            float(self.fb_status), self.fb_tmos, self.fb_trotor,
            self.dm_pos, self.dm_vel, self.dm_kp, self.dm_kd, self.dm_tor,
        )

    def run(self):
        t0 = time.monotonic()
        next_t = 0.0
        while not self.stop_flag:
            # 与真实串口一致：急停/失能优先。
            while True:
                try:
                    self.handle_line(self.urgent_q.get_nowait())
                except queue.Empty:
                    break
            while True:
                try:
                    self.handle_line(self.line_q.get_nowait())
                except queue.Empty:
                    break
            t = time.monotonic() - t0
            next_t += 1.0 / SEND_HZ
            delay = next_t - t
            if delay > 0:
                time.sleep(delay)
            self._push((time.monotonic(), self.make_frame(t)))

    def _push(self, item):
        try:
            self.frame_q.put_nowait(item)
        except queue.Full:
            try:
                self.frame_q.get_nowait()
                self.frame_q.put_nowait(item)
            except queue.Empty:
                pass


# ======================================================================
# CSV 记录器
# ======================================================================
class CsvRecorder:
    def __init__(self, folder):
        os.makedirs(folder, exist_ok=True)
        name = "ILHC_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(folder, name)
        self.f = open(self.path, "w", newline="", encoding="utf-8-sig")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_s", "wallclock"] + ["%s" % c[1] for c in CHANNELS])
        self.rows = 0
        self.t0 = None

    def write(self, t, values):
        if self.t0 is None:
            self.t0 = t
        self.w.writerow(["%.3f" % (t - self.t0),
                         time.strftime("%H:%M:%S")]
                        + ["%.5g" % v if math.isfinite(v) else "" for v in values])
        self.rows += 1
        if self.rows % 50 == 0:
            self.f.flush()

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass


# ======================================================================
# 主应用（tkinter GUI）
# ======================================================================
class App:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        root.title("%s %s  —  STM32F407VET6_ILHC USART1 调试" % (APP_NAME, APP_VERSION))
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry("%dx%d+%d+%d" % (min(1420, sw - 60), min(880, sh - 100),
                                       30, 20))
        root.minsize(1180, 700)
        root.configure(bg=BG)

        # 数据
        self.frame_q = queue.Queue(maxsize=4000)
        self.line_q = queue.Queue()
        self.urgent_q = queue.Queue()
        self.t0_monotonic = time.monotonic()
        self.last_heartbeat_enqueue = 0.0
        self.window_s = 30.0
        cap = int(self.window_s * SEND_HZ) + 100
        self.ring = RingBuffer(cap, FRAME_FLOATS)
        self.traj_ring = RingBuffer(cap, 2)
        self._set_window(self.window_s)
        self.paused = bool(getattr(args, "ui_test", False))
        self.slow_ms = 2000 if getattr(args, "ui_test", False) else 400
        self.latest = None
        self.latest_t = 0.0
        # 波形绘制调度：低频全量重建背景 + 高频 blit 增量画曲线
        self.force_full = True
        self.last_full = 0.0
        self.last_blit = 0.0
        self.last_traj = 0.0
        self.bg = {}
        self.last_canvas_wh = None
        # 场地地图状态
        self.map_bg = None
        self.map_last_wh = None
        self.map_theta = 0.0
        self.map_ox = 0.0
        self.map_oy = 0.0
        self.map_target = None
        self.worker = None
        self.sim = None
        self.recorder = None
        self.fps_cnt = 0
        self.fps = 0
        self.fps_t = time.monotonic()
        self.cmd_history = []
        self.hist_idx = -1
        self.send_count = 0

        self._build_style()
        self._build_ui()
        self._bind_close()
        if self.paused:
            self.btn_pause.configure(text=" ▶ 继续 ", bg="#8d6e63")
        self.root.after(40, self._tick)
        self.root.after(self.slow_ms, self._slow_tick)

        # 启动参数
        self.refresh_ports()
        if args.baud:
            self.baud_var.set(str(args.baud))
        if args.simulate or getattr(args, "ui_test", False):
            self.toggle_sim(True)
        elif args.port:
            self.port_var.set(args.port)
            self.toggle_connect(True)

    # ------------------------------------------------------------------
    # UI 基础
    # ------------------------------------------------------------------
    def _build_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=BG, foreground=FG, fieldbackground=BG_ENTRY,
                        font=("Microsoft YaHei UI", 9))
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=BG_PANEL)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("Panel.TLabel", background=BG_PANEL, foreground=FG)
        style.configure("Dim.TLabel", background=BG_PANEL, foreground=FG_DIM)
        style.configure("TButton", background="#2c313a", foreground=FG,
                        borderwidth=0, focusthickness=0)
        style.map("TButton",
                  background=[("active", "#3a4150"), ("pressed", "#4a5568")])
        style.configure("TEntry", insertcolor=FG)
        style.configure("TCombobox", fieldbackground=BG_ENTRY, background="#2c313a",
                        foreground=FG, arrowcolor=FG)
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background="#22252b", foreground=FG_DIM,
                        padding=(14, 6))
        style.map("TNotebook.Tab",
                  background=[("selected", "#3a4150")],
                  foreground=[("selected", ACCENT)])
        style.configure("TCheckbutton", background=BG_PANEL, foreground=FG,
                        focuscolor=BG_PANEL)
        style.map("TCheckbutton", background=[("active", BG_PANEL)],
                  indicatorcolor=[("selected", ACCENT)])
        style.configure("TRadiobutton", background=BG_PANEL, foreground=FG,
                        focuscolor=BG_PANEL)
        style.configure("Horizontal.TScale", background=BG_PANEL,
                        troughcolor=BG_ENTRY, sliderthickness=12)
        style.map("Horizontal.TScale", background=[("active", ACCENT)])

    def _mk_btn(self, parent, text, cmd, color=None, width=None, **kw):
        bg = color or "#2c313a"
        b = tk.Button(parent, text=text, command=cmd, bg=bg, fg="#ffffff",
                      activebackground="#4a5568", activeforeground="#ffffff",
                      relief="flat", bd=0, font=("Microsoft YaHei UI", 9, "bold"),
                      cursor="hand2", **kw)
        if width:
            b.configure(width=width)
        return b

    # ------------------------------------------------------------------
    # UI 结构
    # ------------------------------------------------------------------
    def _build_ui(self):
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=(8, 4))
        self._build_toolbar(top)

        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=8, pady=4)

        # 左：波形 / 轨迹
        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)
        self.nb_plot = ttk.Notebook(left)
        self.nb_plot.pack(fill="both", expand=True)
        self.nb_plot.bind("<<NotebookTabChanged>>", self._on_plot_tab_changed)
        self._build_wave_tab()
        self._build_map_tab()

        # 右：控制面板
        right = ttk.Frame(main, width=444)
        right.pack_propagate(False)
        right.pack(side="right", fill="y", padx=(8, 0))
        self.nb_ctrl = ttk.Notebook(right)
        self.nb_ctrl.pack(fill="both", expand=True)
        self._build_chassis_tab()
        self._build_dm_tab()
        self._build_console_tab()
        self._build_record_tab()

        # 底部状态栏
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=8, pady=(0, 6))
        self.lbl_status = tk.Label(bar, text="● 未连接", bg=BG, fg=FG_DIM,
                                   font=("Microsoft YaHei UI", 9, "bold"), anchor="w")
        self.lbl_status.pack(side="left")
        self.lbl_fps = tk.Label(bar, text="帧率 0/s", bg=BG, fg=FG_DIM)
        self.lbl_fps.pack(side="left", padx=(18, 0))
        self.lbl_err = tk.Label(bar, text="错误字节 0", bg=BG, fg=FG_DIM)
        self.lbl_err.pack(side="left", padx=(18, 0))
        self.lbl_bytes = tk.Label(bar, text="接收 0 B", bg=BG, fg=FG_DIM)
        self.lbl_bytes.pack(side="left", padx=(18, 0))
        self.lbl_dm = tk.Label(bar, text="DM: —", bg=BG, fg=FG_DIM,
                               font=("Microsoft YaHei UI", 9, "bold"))
        self.lbl_dm.pack(side="right")

    # ------------------------- 顶部工具栏 ------------------------------
    def _build_toolbar(self, top):
        ttk.Label(top, text="串口").pack(side="left")
        self.port_var = tk.StringVar()
        self.cmb_port = ttk.Combobox(top, textvariable=self.port_var, width=16,
                                     state="readonly")
        self.cmb_port.pack(side="left", padx=(4, 4))
        self._mk_btn(top, "刷新", self.refresh_ports, width=5).pack(side="left")

        ttk.Label(top, text="波特率").pack(side="left", padx=(14, 0))
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.cmb_baud = ttk.Combobox(top, textvariable=self.baud_var, width=8,
                                     values=["9600", "19200", "38400", "57600",
                                             "115200", "230400", "460800"],
                                     state="readonly")
        self.cmb_baud.pack(side="left", padx=(4, 4))

        self.btn_conn = self._mk_btn(top, "  连接  ", lambda: self.toggle_connect(),
                                     color="#2e7d32")
        self.btn_conn.pack(side="left", padx=(10, 0))

        self.btn_sim = self._mk_btn(top, " 模拟模式 ", lambda: self.toggle_sim(),
                                    color="#455a64")
        self.btn_sim.pack(side="left", padx=(8, 0))

        ttk.Label(top, text="时间窗").pack(side="left", padx=(18, 0))
        self.win_var = tk.StringVar(value="30 s")
        cmb = ttk.Combobox(top, textvariable=self.win_var, width=6, state="readonly",
                           values=["10 s", "30 s", "60 s", "120 s"])
        cmb.pack(side="left", padx=(4, 0))
        cmb.bind("<<ComboboxSelected>>", self._on_window_change)

        self.btn_pause = self._mk_btn(top, " ⏸ 暂停 ", self.toggle_pause, width=8)
        self.btn_pause.pack(side="left", padx=(14, 0))
        self._mk_btn(top, " 清除 ", self.clear_data, width=6).pack(side="left", padx=(4, 0))
        self._mk_btn(top, " 帮助 ", self.show_help, width=6).pack(side="left", padx=(4, 0))
        self.btn_record_top = self._mk_btn(top, " ● 记录 ", self.toggle_record,
                                           color="#b71c1c", width=8)
        self.btn_record_top.pack(side="left", padx=(14, 0))

    # ------------------------- 波形页 ----------------------------------
    def _build_wave_tab(self):
        page = ttk.Frame(self.nb_plot)
        page.pack(fill="both", expand=True)
        self.nb_plot.add(page, text=" 通道波形 ")

        matplotlib.rcParams.update(MPL_RC)
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.fig.subplots_adjust(left=0.06, right=0.985, top=0.94, bottom=0.07,
                                 hspace=0.30, wspace=0.22)
        # 画布放进 pack_propagate(False) 容器，避免画布 resize 与 pack
        # 互相拉扯导致布局膨胀（右侧面板被挤出窗口的问题）
        holder = ttk.Frame(page, style="Panel.TFrame")
        holder.pack(fill="both", expand=True)
        holder.pack_propagate(False)
        self.canvas = FigureCanvasTkAgg(self.fig, master=holder)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.ax_group = {}
        for g, pos in GROUP_AX_POS.items():
            ax = self.fig.add_subplot(2, 2, pos[0] * 2 + pos[1] + 1)
            ax.grid(True, which="both", alpha=0.6)
            ax.set_title(GROUP_NAMES[g], fontsize=9, color=ACCENT, pad=4)
            ax.tick_params(labelsize=7.5)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            self.ax_group[g] = ax

        # 为全部通道创建曲线（默认隐藏）
        self.lines = {}
        self.cb_vars = {}
        for (idx, key, label, unit, group) in CHANNELS:
            ln, = self.ax_group[group].plot([], [], color=COLORS[idx], lw=1.1,
                                            label="%s%s" % (label, ("(%s)" % unit) if unit else ""))
            ln.set_visible(False)
            self.lines[idx] = ln
        self._relegend()

        # 底部通道选择：分组下拉菜单（单行按钮，勾选状态见计数与菜单勾号）
        sel = ttk.Frame(page, style="Panel.TFrame")
        sel.pack(fill="x", side="bottom")
        ttk.Label(sel, text="通道选择：", style="Panel.TLabel").pack(
            side="left", padx=(8, 4), pady=4)
        self.grp_buttons = {}
        defaults = {0, 1, 2, 11, 13, 14}
        for g in sorted(GROUP_AX_POS):
            mb = tk.Menubutton(sel, text="", relief="flat", cursor="hand2",
                               bg="#2c313a", fg=FG_DIM,
                               activebackground="#3a4150",
                               activeforeground="#ffffff",
                               font=("Microsoft YaHei UI", 8, "bold"),
                               padx=8, pady=2)
            menu = tk.Menu(mb, tearoff=0, bg=BG_ENTRY, fg=FG,
                           activebackground="#3a4150",
                           activeforeground="#ffffff",
                           font=("Microsoft YaHei UI", 8))
            mb.configure(menu=menu)
            mb.pack(side="left", padx=(0, 6), pady=4)
            self.grp_buttons[g] = (mb, menu)
        for (idx, key, label, unit, group) in CHANNELS:
            var = tk.BooleanVar(value=idx in defaults)
            self.cb_vars[idx] = var
            _mb, menu = self.grp_buttons[group]
            menu.add_checkbutton(label=label, variable=var,
                                 command=self._on_menu_check)
        # 勾选后应用可见性并刷新按钮计数
        self._on_menu_check()

    def _relegend(self):
        for g, ax in self.ax_group.items():
            handles = [self.lines[idx] for (idx, k, l, u, gg) in CHANNELS
                       if gg == g and self.lines[idx].get_visible()]
            if handles:
                ax.legend(handles=handles, fontsize=6.8, loc="upper left",
                          ncol=2, framealpha=0.25, facecolor=PLOT_BG,
                          edgecolor=GRID_COL, labelcolor=FG,
                          handlelength=1.4, columnspacing=0.8, borderpad=0.3)
            else:
                if ax.get_legend():
                    ax.get_legend().remove()

    def _on_menu_check(self):
        """菜单勾选变化：应用曲线可见性 + 刷新各分组按钮的计数与颜色"""
        for idx, var in self.cb_vars.items():
            self.lines[idx].set_visible(bool(var.get()))
        self._relegend()
        self.force_full = True
        counts = {g: [0, 0] for g in GROUP_AX_POS}
        for (idx, _k, _l, _u, g) in CHANNELS:
            counts[g][1] += 1
            if self.cb_vars[idx].get():
                counts[g][0] += 1
        for g, (mb, _menu) in self.grp_buttons.items():
            on, total = counts[g]
            mb.configure(text=" %s %d/%d ▾" % (GROUP_NAMES[g], on, total),
                         fg=ACCENT if on else FG_DIM)

    # ------------------------- 场地地图页 --------------------------------
    # 初赛场地（图1，2400x2400mm，原点=场地左下角，依示意图 1:1 绘制）
    FIELD = 2400.0

    def _build_map_tab(self):
        page = ttk.Frame(self.nb_plot)
        page.pack(fill="both", expand=True)
        self.nb_plot.add(page, text=" 比赛场地地图 ")

        # 起始启停区设置：车放在所选区内，区中心即坐标原点
        bar0 = ttk.Frame(page, style="Panel.TFrame")
        bar0.pack(fill="x")
        ttk.Label(bar0, text="起始设置：", style="Panel.TLabel").pack(
            side="left", padx=(8, 4), pady=4)
        self.start_zone_var = tk.IntVar(value=1)
        for zone, text in ((1, "启停区1（右上）"), (2, "启停区2（右下）")):
            tk.Radiobutton(bar0, text=text, variable=self.start_zone_var,
                           value=zone, command=self._on_start_zone_changed,
                           bg=BG_PANEL, fg=FG, selectcolor=BG_ENTRY,
                           activebackground=BG_PANEL, highlightthickness=0,
                           cursor="hand2").pack(side="left", padx=(0, 6))
        self._mk_btn(bar0, "⊙ 置为原点并归零", self._set_start_zone,
                     color="#e65100").pack(side="left", padx=(6, 0), pady=2)
        self.lbl_start = tk.Label(bar0, text="未设置（OPS 坐标=场地坐标）",
                                  bg=BG_PANEL, fg=FG_DIM,
                                  font=("Microsoft YaHei UI", 8))
        self.lbl_start.pack(side="left", padx=(10, 0))

        # 快速前往各功能区
        bar_q = ttk.Frame(page, style="Panel.TFrame")
        bar_q.pack(fill="x")
        ttk.Label(bar_q, text="快速前往：", style="Panel.TLabel").pack(
            side="left", padx=(8, 4), pady=3)
        for name, (qx, qy) in QUICK_TARGETS:
            self._mk_btn(bar_q, name,
                         lambda a=qx, b=qy: self._goto_field(a, b),
                         width=8).pack(side="left", padx=(0, 5), pady=2)
        self._mk_btn(bar_q, "回启停区", self._goto_home,
                     width=8).pack(side="left", padx=(0, 5), pady=2)
        ttk.Label(bar_q, text="（按上方“目标航向”到达）", style="Dim.TLabel").pack(
            side="left", padx=(6, 0))

        # 坐标映射 / 目标航向 / 点击下发开关 / 急停
        bar = ttk.Frame(page, style="Panel.TFrame")
        bar.pack(fill="x")
        ttk.Label(bar, text="OPS→场地映射：原点X", style="Panel.TLabel").pack(
            side="left", padx=(8, 2), pady=4)
        self.map_ox_var = tk.StringVar(value="0")
        ttk.Entry(bar, textvariable=self.map_ox_var, width=6,
                  justify="center").pack(side="left")
        ttk.Label(bar, text="Y", style="Panel.TLabel").pack(side="left", padx=(4, 2))
        self.map_oy_var = tk.StringVar(value="0")
        ttk.Entry(bar, textvariable=self.map_oy_var, width=6,
                  justify="center").pack(side="left")
        ttk.Label(bar, text="旋转°", style="Panel.TLabel").pack(side="left", padx=(4, 2))
        self.map_th_var = tk.StringVar(value="0")
        ttk.Entry(bar, textvariable=self.map_th_var, width=5,
                  justify="center").pack(side="left")
        self._mk_btn(bar, " 应用映射 ", self._apply_map_mapping,
                     width=9).pack(side="left", padx=(6, 0))

        ttk.Label(bar, text="目标航向", style="Panel.TLabel").pack(
            side="left", padx=(14, 2))
        self.map_yaw_var = tk.StringVar(value="保持当前")
        ttk.Combobox(bar, textvariable=self.map_yaw_var, width=8, state="readonly",
                     values=["保持当前", "场地 0°", "场地 90°",
                             "场地 180°", "场地 270°"]).pack(side="left")

        self.map_click_var = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text="点击下发", variable=self.map_click_var,
                       bg=BG_PANEL, fg=FG, activebackground=BG_PANEL,
                       activeforeground=ACCENT, selectcolor=BG_ENTRY,
                       font=("Microsoft YaHei UI", 8),
                       highlightthickness=0, cursor="hand2").pack(
            side="left", padx=(10, 0))
        self._mk_btn(bar, " ■ 停止 ", self._quick("STOP"), color="#c62828",
                     width=7).pack(side="left", padx=(10, 0), pady=2)

        self.lbl_map = tk.Label(
            page, text="在地图上点击 = 小车移动到该位置（下发 GOTO 命令，STOP 取消）",
            bg=BG_PANEL, fg=FG_DIM, font=("Microsoft YaHei UI", 8), anchor="w")
        self.lbl_map.pack(fill="x", padx=8)

        holder = ttk.Frame(page, style="Panel.TFrame")
        holder.pack(fill="both", expand=True)
        holder.pack_propagate(False)
        self.map_fig = Figure(figsize=(5, 5), dpi=100)
        self.map_fig.subplots_adjust(left=0.06, right=0.985, top=0.97, bottom=0.07)
        self.map_ax = self.map_fig.add_subplot(111)
        self._draw_field(self.map_ax)

        # 动态元素：轨迹 / 车体 / 航向 / 目标
        self.map_trail, = self.map_ax.plot([], [], color=ACCENT, lw=1.2,
                                           alpha=0.85, zorder=5)
        self.map_car, = self.map_ax.plot([], [], "o", ms=9, mfc="#ff9800",
                                         mec="#ffffff", mew=1.2, zorder=7)
        self.map_dir, = self.map_ax.plot([], [], color="#ff9800", lw=2.5,
                                         zorder=6)
        self.map_tgt, = self.map_ax.plot([], [], marker="x", ms=14, mew=2.5,
                                         color="#ef5350", ls="none", zorder=8)
        self.map_dyn = [self.map_trail, self.map_dir, self.map_car, self.map_tgt]

        self.map_canvas = FigureCanvasTkAgg(self.map_fig, master=holder)
        self.map_canvas.get_tk_widget().pack(fill="both", expand=True)
        self.map_canvas.mpl_connect("button_press_event", self._on_map_click)

    def _draw_field(self, ax):
        """按初赛场地示意图绘制静态地图（作为 blit 背景）"""
        f = self.FIELD
        ax.set_facecolor(PLOT_BG)
        ax.set_xlim(-170, f + 170)
        ax.set_ylim(-170, f + 170)
        ax.set_aspect("equal")
        ax.set_xticks(range(0, 2401, 600))
        ax.set_yticks(range(0, 2401, 600))
        ax.tick_params(labelsize=7.5)
        # 参考网格（300mm）
        for v in range(0, 2401, 300):
            ax.plot([v, v], [0, f], color=GRID_COL, lw=0.5, alpha=0.5, zorder=1)
            ax.plot([0, f], [v, v], color=GRID_COL, lw=0.5, alpha=0.5, zorder=1)
        # 场地底板
        ax.add_patch(Rectangle((0, 0), f, f, facecolor="#c9ccd2",
                               edgecolor="#3a3d45", lw=2, zorder=2))
        # 中央 4 块 450x450 黄色物料区（中心对称，间隙 400）
        for x0 in (550, 1400):
            for y0 in (550, 1400):
                ax.add_patch(Rectangle((x0, y0), 450, 450, facecolor="#f6f1cf",
                                       edgecolor="#b9b47e", lw=1, zorder=3))
        # 启停区 1 / 2（右上 / 右下，300x300 蓝色）
        ax.add_patch(Rectangle((2100, 2100), 300, 300, facecolor="#1e4fd6",
                               edgecolor="#123091", lw=1, zorder=3))
        ax.add_patch(Rectangle((2100, 0), 300, 300, facecolor="#1e4fd6",
                               edgecolor="#123091", lw=1, zorder=3))
        ax.text(2250, 2250, "启停区1", color="white", fontsize=10,
                ha="center", va="center", zorder=4)
        ax.text(2250, 150, "启停区2", color="white", fontsize=10,
                ha="center", va="center", zorder=4)
        # 原料区（顶边中央圆盘 + 3 孔）
        ax.add_patch(Circle((1200, 2400), 110, facecolor="#eceff3",
                            edgecolor="#3a3d45", lw=1.5, zorder=3))
        for (hx, hy) in ((1200, 2445), (1160, 2375), (1240, 2375)):
            ax.add_patch(Circle((hx, hy), 20, facecolor="#8b93a1",
                                edgecolor="#3a3d45", lw=0.8, zorder=4))
        ax.text(1200, 2255, "原料区", color="#454a54", fontsize=10,
                ha="center", va="center", zorder=4)
        # 暂存区（左边中段 150x580 + 3 孔）
        ax.add_patch(Rectangle((0, 910), 150, 580, facecolor="#eceff3",
                               edgecolor="#3a3d45", lw=1.2, zorder=3))
        for hy in (1050, 1200, 1350):
            ax.add_patch(Circle((75, hy), 24, facecolor="#8b93a1",
                                edgecolor="#3a3d45", lw=0.8, zorder=4))
        ax.text(265, 1200, "暂存区", color="#454a54", fontsize=10, rotation=90,
                ha="center", va="center", zorder=4)
        # 粗加工区（底边中段 400x150 + 3 孔）
        ax.add_patch(Rectangle((1000, 0), 400, 150, facecolor="#eceff3",
                               edgecolor="#3a3d45", lw=1.2, zorder=3))
        for hx in (1075, 1200, 1325):
            ax.add_patch(Circle((hx, 75), 24, facecolor="#8b93a1",
                                edgecolor="#3a3d45", lw=0.8, zorder=4))
        ax.text(1200, 255, "粗加工区", color="#454a54", fontsize=10,
                ha="center", va="center", zorder=4)
        # 二维码板（右边中段）
        ax.add_patch(Rectangle((2388, 1170), 12, 60, facecolor="#17181b",
                               edgecolor="none", zorder=3))
        ax.text(2295, 1100, "二维码板", color="#454a54", fontsize=8, rotation=90,
                ha="center", va="center", zorder=4)

    # OPS 坐标 <-> 场地坐标（原点平移 + 旋转）
    def _ops_to_field(self, x, y):
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        return (self.map_ox + c * x - s * y, self.map_oy + s * x + c * y)

    def _field_to_ops(self, fx, fy):
        th = math.radians(self.map_theta)
        c, s = math.cos(th), math.sin(th)
        dx, dy = fx - self.map_ox, fy - self.map_oy
        return (c * dx + s * dy, -s * dx + c * dy)

    def _apply_map_mapping(self):
        try:
            self.map_ox = float(self.map_ox_var.get())
            self.map_oy = float(self.map_oy_var.get())
            self.map_theta = float(self.map_th_var.get())
        except ValueError:
            messagebox.showwarning(APP_NAME, "映射参数需为数字")
            return
        self.log("场地映射已更新：原点(%.0f, %.0f) 旋转 %.1f°"
                 % (self.map_ox, self.map_oy, self.map_theta), "info")

    def _on_start_zone_changed(self):
        """切换单选时把所选启停区中心填入映射原点（还需点按钮确认生效）"""
        zx, zy = ZONE_CENTER[self.start_zone_var.get()]
        self.map_ox_var.set("%.0f" % zx)
        self.map_oy_var.set("%.0f" % zy)

    def _set_start_zone(self):
        """把所选启停区设为坐标原点：区中心=OPS(0,0)，并下发 ZERO 归零。

        前提：车子已物理摆放在该启停区内。"""
        zone = self.start_zone_var.get()
        zx, zy = ZONE_CENTER[zone]
        self.map_ox_var.set("%.0f" % zx)
        self.map_oy_var.set("%.0f" % zy)
        self._apply_map_mapping()
        self.send_line("ZERO")               # OPS 当前位置置零
        self.traj_ring.clear()               # 全新出发，清掉旧轨迹
        self.map_target = None
        self.lbl_start.configure(
            text="起始区：启停区%d · 原点(%.0f, %.0f) · 已归零" % (zone, zx, zy),
            fg=ACCENT)
        self.lbl_map.configure(
            text="已设启停区%d 为原点，点击地图开始移动（STOP 取消）" % zone,
            fg="#ffb74d")
        self.log("起始启停区：启停区%d，映射原点(%.0f, %.0f)，已发送 ZERO 归零"
                 % (zone, zx, zy), "info")

    def _target_yaw_ops(self):
        """按航向选择框换算出固件需要的 OPS 系目标航向角"""
        yaw_sel = self.map_yaw_var.get()
        if yaw_sel == "保持当前":
            return self.latest[2] if self.latest is not None else 0.0
        return float(yaw_sel.split()[1]) - self.map_theta

    @staticmethod
    def _point_in_rect(x, y, rect):
        x0, y0, x1, y1, _name = rect
        return x0 <= x <= x1 and y0 <= y <= y1

    @staticmethod
    def _seg_intersects_rect(x0, y0, x1, y1, rect):
        """Liang-Barsky 线段/轴对齐矩形相交测试。"""
        rx0, ry0, rx1, ry1, _name = rect
        dx, dy = x1 - x0, y1 - y0
        p = (-dx, dx, -dy, dy)
        q = (x0 - rx0, rx1 - x0, y0 - ry0, ry1 - y0)
        u0, u1 = 0.0, 1.0
        for pi, qi in zip(p, q):
            if abs(pi) < 1e-12:
                if qi < 0:
                    return False
                continue
            r = qi / pi
            if pi < 0:
                if r > u1:
                    return False
                u0 = max(u0, r)
            else:
                if r < u0:
                    return False
                u1 = min(u1, r)
        return u0 <= u1

    @staticmethod
    def _seg_intersects_circle(x0, y0, x1, y1, circle):
        cx, cy, r, _name = circle
        dx, dy = x1 - x0, y1 - y0
        den = dx * dx + dy * dy
        if den <= 1e-12:
            return math.hypot(x0 - cx, y0 - cy) <= r
        u = ((cx - x0) * dx + (cy - y0) * dy) / den
        u = min(1.0, max(0.0, u))
        px, py = x0 + u * dx, y0 + u * dy
        return math.hypot(px - cx, py - cy) <= r

    def _field_point_blocked(self, x, y):
        for rect in FIELD_FORBIDDEN_RECTS:
            if self._point_in_rect(x, y, rect):
                return rect[4]
        for cx, cy, r, name in FIELD_FORBIDDEN_CIRCLES:
            if math.hypot(x - cx, y - cy) <= r:
                return name
        return None

    def _field_path_blocked(self, x0, y0, x1, y1):
        for rect in FIELD_FORBIDDEN_RECTS:
            if self._seg_intersects_rect(x0, y0, x1, y1, rect):
                return rect[4]
        for circle in FIELD_FORBIDDEN_CIRCLES:
            if self._seg_intersects_circle(x0, y0, x1, y1, circle):
                return circle[3]
        return None

    def _goto_field(self, fx, fy):
        """向场地坐标 (fx, fy) 下发 GOTO（地图点击与快速按钮共用）。

        v1.1 在下发前做静态目标点/直线路径禁区检查。这里只负责阻止明显穿越
        固定设备，不等价于考虑车体尺寸的完整避障路径规划。
        """
        blocked = self._field_point_blocked(fx, fy)
        if blocked:
            self.lbl_map.configure(
                text="拒绝 GOTO：目标点位于禁区【%s】" % blocked, fg="#ef5350")
            self.log("GOTO 被安全检查拒绝：目标(%.0f, %.0f) 位于%s" %
                     (fx, fy, blocked), "warn")
            return

        v = self.latest
        if v is not None and math.isfinite(v[0]) and math.isfinite(v[1]):
            sx, sy = self._ops_to_field(v[0], v[1])
            blocked = self._field_path_blocked(sx, sy, fx, fy)
            if blocked:
                self.lbl_map.configure(
                    text="拒绝直线 GOTO：路径穿越【%s】，请先点击中间安全点绕行" % blocked,
                    fg="#ef5350")
                self.log("GOTO 被安全检查拒绝：直线路径穿越%s" % blocked, "warn")
                return

        tx, ty = self._field_to_ops(fx, fy)
        z = self._target_yaw_ops()
        self.map_target = (fx, fy)
        self.send_line("GOTO=%.0f,%.0f,%.0f" % (tx, ty, z))
        self.lbl_map.configure(
            text="目标：场地 (%.0f, %.0f) mm  →  已下发 GOTO=%.0f,%.0f,%.0f"
                 % (fx, fy, tx, ty, z), fg="#ffb74d")

    def _goto_home(self):
        """返回当前起始启停区，到点后朝向场地内侧"""
        zone = self.start_zone_var.get()
        zx, zy = ZONE_CENTER[zone]
        home_yaw = (270.0 if zone == 1 else 90.0) - self.map_theta
        tx, ty = self._field_to_ops(zx, zy)
        self.map_target = (zx, zy)
        self.send_line("GOTO=%.0f,%.0f,%.0f" % (tx, ty, home_yaw))
        self.lbl_map.configure(
            text="返回启停区%d (%.0f, %.0f)  →  已下发 GOTO=%.0f,%.0f,%.0f"
                 % (zone, zx, zy, tx, ty, home_yaw), fg="#ffb74d")

    def _on_map_click(self, event):
        if (event.inaxes is not self.map_ax or event.button != 1
                or event.xdata is None or event.ydata is None):
            return
        if not self.map_click_var.get():
            return
        fx = min(self.FIELD, max(0.0, float(event.xdata)))
        fy = min(self.FIELD, max(0.0, float(event.ydata)))
        self._goto_field(fx, fy)

    def _map_update(self, now):
        """场地地图增量绘制（约 10 Hz）：背景为静态场地，仅重画小车/轨迹/目标"""
        self.last_traj = now
        widget = self.map_canvas.get_tk_widget()
        wh = (widget.winfo_width(), widget.winfo_height())
        if wh[0] < 50:                       # 尚未完成布局
            return
        if self._sync_fig_size(self.map_fig, self.map_canvas):
            self.map_bg = None               # 渲染尺寸已跟上画布，重建背景
        if self.map_bg is None or self.map_last_wh != wh:
            self.map_last_wh = wh
            vis = [a.get_visible() for a in self.map_dyn]
            for a in self.map_dyn:
                a.set_visible(False)
            try:
                self.map_canvas.draw()
                self.map_bg = self.map_canvas.copy_from_bbox(self.map_ax.bbox)
            except Exception:
                self.map_bg = None
                return
            finally:
                for a, v0 in zip(self.map_dyn, vis):
                    a.set_visible(v0)
        try:
            self.map_canvas.restore_region(self.map_bg)
        except Exception:
            self.map_bg = None
            return

        # 历史轨迹（原始 OPS 坐标 -> 场地坐标，抽稀）
        view = self.traj_ring.view()
        if view is not None and len(view[1]) >= 2:
            d = view[1]
            stride = max(1, len(d) // 900)
            x = d[::stride, 0]
            y = d[::stride, 1]
            th = math.radians(self.map_theta)
            c, s = math.cos(th), math.sin(th)
            self.map_trail.set_data(self.map_ox + c * x - s * y,
                                    self.map_oy + s * x + c * y)
        else:
            self.map_trail.set_data([], [])

        # 小车位置 + 航向
        v = self.latest
        if v is not None and math.isfinite(v[0]) and math.isfinite(v[1]):
            cx, cy = self._ops_to_field(v[0], v[1])
            self.map_car.set_data([cx], [cy])
            th = math.radians(self.map_theta + v[2])
            ln = 170.0
            self.map_dir.set_data([cx, cx + ln * math.cos(th)],
                                  [cy, cy + ln * math.sin(th)])

        # 目标标记
        if self.map_target is not None:
            self.map_tgt.set_data([self.map_target[0]], [self.map_target[1]])
        else:
            self.map_tgt.set_data([], [])

        for a in self.map_dyn:
            self.map_ax.draw_artist(a)
        try:
            self.map_canvas.blit(self.map_ax.bbox)
        except Exception:
            self.map_bg = None

    # ------------------------- 底盘调参页 -------------------------------
    def _build_chassis_tab(self):
        page = ttk.Frame(self.nb_ctrl, style="Panel.TFrame")
        self.nb_ctrl.add(page, text=" 底盘调参 ")

        # 急停区
        zone = ttk.Frame(page, style="Panel.TFrame")
        zone.pack(fill="x", padx=10, pady=(10, 4))
        self._mk_btn(zone, "■ 底盘急停 (STOP)", self._quick("STOP"),
                     color="#c62828", height=2).pack(fill="x", ipady=6)
        self._mk_btn(zone, "⊙ OPS 置零 (ZERO)", self._quick("ZERO"),
                     color="#e65100").pack(fill="x", pady=(6, 0), ipady=4)

        ttk.Label(page, text="— 定位 P 系数 / 速度限幅（下固件自动限幅）—",
                  style="Dim.TLabel").pack(anchor="w", padx=10, pady=(10, 2))

        self.chassis_rows = {}
        body = ttk.Frame(page, style="Panel.TFrame")
        body.pack(fill="x", padx=10)
        for (cmd, label, lo, hi, dflt, rb) in CHASSIS_PARAMS:
            self._make_param_row(body, cmd, label, lo, hi, dflt, rb,
                                 self.chassis_rows)

        self._mk_btn(page, "⇊ 一键下发全部底盘参数", self._send_all_chassis,
                     color="#283593").pack(fill="x", padx=10, pady=(10, 4), ipady=3)

        tip = tk.Label(page, text="提示：KPX/KPY/KPZ 回读通道 6/7/8，XVMAX/ZVMAX\n"
                                  "回读通道 9/10；XVMIN/ZVMIN 固件无回读。",
                       bg=BG_PANEL, fg=FG_DIM, font=("Microsoft YaHei UI", 8),
                       justify="left", anchor="w")
        tip.pack(fill="x", padx=10, pady=(4, 8))

    def _make_param_row(self, parent, cmd, label, lo, hi, dflt, rb_ch, store):
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(fill="x", pady=3)
        ttk.Label(row, text=cmd, style="Panel.TLabel", width=6,
                  font=("Consolas", 9, "bold")).pack(side="left")
        ttk.Label(row, text=label, style="Dim.TLabel", width=13, anchor="w").pack(
            side="left", padx=(0, 4))
        var = tk.StringVar(value=self._fmt(dflt))
        entry = ttk.Entry(row, textvariable=var, width=7, justify="center")
        entry.pack(side="left", padx=(2, 6))
        scale_var = tk.DoubleVar(value=dflt)
        scale = ttk.Scale(row, variable=scale_var, from_=lo, to=hi,
                          command=lambda v, c=cmd, sv=var, s=scale_var:
                          self._scale_moved(c, sv, s, v))
        scale.pack(side="left", fill="x", expand=True, padx=(0, 6))
        entry.bind("<Return>", lambda _e, c=cmd: self._validate_param(c))
        entry.bind("<FocusOut>", lambda _e, c=cmd: self._validate_param(c, quiet=True))
        self._mk_btn(row, "发送", lambda c=cmd: self._send_param(c),
                     width=5).pack(side="left")
        rb = ttk.Label(row, text="—", style="Dim.TLabel", width=11,
                       font=("Consolas", 8), anchor="e")
        rb.pack(side="left", padx=(6, 0))
        store[cmd] = {"var": var, "scale": scale_var, "lo": lo, "hi": hi,
                      "rb": rb, "rb_ch": rb_ch, "cmd": cmd, "entry": entry}

    @staticmethod
    def _fmt(v):
        return ("%.0f" % v) if abs(v - round(v)) < 1e-9 else ("%.2f" % v)

    @staticmethod
    def _fmt_readback(v):
        """回读数值格式化：2000 -> '2000'，5.5 -> '5.5'，-8.0 -> '-8'"""
        if not math.isfinite(v):
            return "—"
        s = "%.3f" % v
        s = s.rstrip("0").rstrip(".")
        return s if s not in ("", "-", "-0") else "0"

    def _scale_moved(self, cmd, entry_var, scale_var, raw):
        # 滑条 -> 输入框。输入框 -> 滑条在 _validate_param 中完成。
        entry_var.set(self._fmt(float(raw)))
        _ = (cmd, scale_var)

    def _param_row(self, cmd):
        return self.chassis_rows.get(cmd) or self.dm_rows.get(cmd)

    def _validate_param(self, cmd, quiet=False):
        row = self._param_row(cmd)
        if row is None:
            return None
        raw = row["var"].get().strip()
        try:
            value = float(raw)
            if not math.isfinite(value):
                raise ValueError
        except ValueError:
            if not quiet:
                messagebox.showwarning(APP_NAME, "%s 参数必须是有限数字" % cmd)
            row["var"].set(self._fmt(row["scale"].get()))
            return None
        clamped = min(row["hi"], max(row["lo"], value))
        if cmd == "DMID":
            clamped = int(round(clamped))
        row["scale"].set(clamped)
        row["var"].set(str(int(clamped)) if cmd == "DMID" else self._fmt(clamped))
        if not quiet and clamped != value:
            self.log("%s 输入 %.6g 已限幅为 %s" %
                     (cmd, value, row["var"].get()), "warn")
        return clamped

    def _send_param(self, cmd):
        if self._validate_param(cmd) is None:
            return
        row = self._param_row(cmd)
        self.send_line("%s=%s" % (cmd, row["var"].get()))

    # ------------------------- DM 电机页 --------------------------------
    def _build_dm_tab(self):
        page = ttk.Frame(self.nb_ctrl, style="Panel.TFrame")
        self.nb_ctrl.add(page, text=" DM 电机 ")

        zone = ttk.Frame(page, style="Panel.TFrame")
        zone.pack(fill="x", padx=10, pady=(10, 0))
        self._mk_btn(zone, "▶ 使能 DMEN", self._quick("DMEN"),
                     color="#2e7d32").pack(side="left", fill="x", expand=True,
                                           ipady=4)
        self._mk_btn(zone, "■ 失能 DMOFF", self._quick("DMOFF"),
                     color="#c62828").pack(side="left", fill="x", expand=True,
                                           padx=(6, 0), ipady=4)
        self._mk_btn(zone, "⊙ 零点 DMZERO", self._quick("DMZERO"),
                     color="#e65100").pack(fill="x", side="left", expand=True,
                                           padx=(6, 0), ipady=3)

        # 控制模式
        modef = ttk.Frame(page, style="Panel.TFrame")
        modef.pack(fill="x", padx=10, pady=(10, 0))
        ttk.Label(modef, text="控制模式：", style="Panel.TLabel").pack(side="left")
        self.dm_mode_var = tk.IntVar(value=1)
        rb1 = tk.Radiobutton(modef, text="1 MIT", variable=self.dm_mode_var, value=1,
                             command=lambda: self._dm_mode_changed(1),
                             bg=BG_PANEL, fg=FG, selectcolor=BG_ENTRY,
                             activebackground=BG_PANEL, highlightthickness=0,
                             cursor="hand2")
        rb2 = tk.Radiobutton(modef, text="2 位置速度", variable=self.dm_mode_var,
                             value=2, command=lambda: self._dm_mode_changed(2),
                             bg=BG_PANEL, fg=FG, selectcolor=BG_ENTRY,
                             activebackground=BG_PANEL, highlightthickness=0,
                             cursor="hand2")
        rb1.pack(side="left", padx=(8, 4))
        rb2.pack(side="left", padx=4)
        self.lbl_dm_mode = ttk.Label(modef, text="（DMEN 时写入寄存器 10）",
                                     style="Dim.TLabel")
        self.lbl_dm_mode.pack(side="left", padx=(6, 0))

        self.dm_rows = {}
        body = ttk.Frame(page, style="Panel.TFrame")
        body.pack(fill="x", padx=10, pady=(6, 0))
        for (cmd, label, lo, hi, dflt, rb, is_int) in DM_PARAMS:
            self._make_param_row(body, cmd, label, lo, hi, dflt, rb, self.dm_rows)
            _ = is_int

        self.lbl_mode_note = tk.Label(page, text="位置速度模式：DMKP / DMKD / DMTOR 会被电机忽略",
                                      bg=BG_PANEL, fg=FG_DIM,
                                      font=("Microsoft YaHei UI", 8))
        self.lbl_mode_note.pack(anchor="w", padx=10)

        # 实时反馈区
        ttk.Label(page, text="— 实时反馈（遥测通道）—", style="Dim.TLabel").pack(
            anchor="w", padx=10, pady=(10, 2))
        fb = ttk.Frame(page, style="Panel.TFrame")
        fb.pack(fill="x", padx=10)
        self.dm_fb_labels = {}
        for i, (key, title) in enumerate([
                ("pos", "实际位置 rad"), ("vel", "实际速度 rad/s"), ("tor", "实际力矩 Nm"),
                ("tmos", "MOS 温度 °C"), ("trotor", "线圈温度 °C"), ("id", "电机 ID")]):
            cell = ttk.Frame(fb, style="Panel.TFrame")
            cell.grid(row=i // 3, column=i % 3, sticky="nsew", padx=2, pady=2)
            fb.columnconfigure(i % 3, weight=1)
            ttk.Label(cell, text=title, style="Dim.TLabel",
                      font=("Microsoft YaHei UI", 8)).pack(anchor="w")
            lb = tk.Label(cell, text="—", bg=BG_PANEL, fg=ACCENT,
                          font=("Consolas", 9, "bold"), anchor="w")
            lb.pack(anchor="w")
            self.dm_fb_labels[key] = lb

        self.lbl_dm_status_big = tk.Label(page, text="DM 状态：—", bg=BG_PANEL,
                                          fg=FG_DIM, font=("Microsoft YaHei UI", 10, "bold"),
                                          anchor="w", padx=8, pady=6)
        self.lbl_dm_status_big.pack(fill="x", padx=10, pady=(8, 8))

    def _dm_id_int(self, cmd):
        v = self.dm_rows[cmd]["var"].get()
        try:
            self.dm_rows[cmd]["var"].set(str(int(float(v))))
        except (ValueError, tk.TclError):
            pass

    def _dm_mode_changed(self, mode):
        self.dm_mode_var.set(mode)
        self.lbl_mode_note.configure(
            text="位置速度模式：DMKP / DMKD / DMTOR 会被电机忽略"
            if mode == 2 else "MIT 模式：Kp/Kd/Torque 参与力矩合成")
        self.send_line("DMMODE=%d" % mode)

    # ------------------------- 命令行页 ---------------------------------
    def _build_console_tab(self):
        page = ttk.Frame(self.nb_ctrl, style="Panel.TFrame")
        self.nb_ctrl.add(page, text=" 命令行 ")

        ttk.Label(page, text="发送日志（命令每行一条，\\r 或 \\n 结尾）",
                  style="Dim.TLabel").pack(anchor="w", padx=10, pady=(8, 2))
        self.txt_log = tk.Text(page, bg=BG_ENTRY, fg=FG, height=16, relief="flat",
                               font=("Consolas", 9), state="disabled", wrap="none",
                               insertbackground=FG, selectbackground="#334455")
        self.txt_log.pack(fill="both", expand=True, padx=10)
        self.txt_log.tag_config("tx", foreground="#81c784")
        self.txt_log.tag_config("info", foreground=FG_DIM)
        self.txt_log.tag_config("warn", foreground="#ef5350")

        quick = ttk.Frame(page, style="Panel.TFrame")
        quick.pack(fill="x", padx=10, pady=(6, 0))
        for cmd, color in [("STOP", "#c62828"), ("ZERO", "#e65100"),
                           ("DMEN", "#2e7d32"), ("DMOFF", "#8d6e63"),
                           ("DMZERO", "#455a64")]:
            self._mk_btn(quick, cmd, self._quick(cmd), color=color,
                         width=7).pack(side="left", padx=(0, 3))

        entryf = ttk.Frame(page, style="Panel.TFrame")
        entryf.pack(fill="x", padx=10, pady=(6, 10))
        self.cmd_var = tk.StringVar()
        self.ent_cmd = ttk.Entry(entryf, textvariable=self.cmd_var,
                                 font=("Consolas", 10))
        self.ent_cmd.pack(side="left", fill="x", expand=True, ipady=3)
        self.ent_cmd.bind("<Return>", lambda e: self._send_from_entry())
        self.ent_cmd.bind("<Up>", lambda e: self._cmd_history(-1))
        self.ent_cmd.bind("<Down>", lambda e: self._cmd_history(1))
        self._mk_btn(entryf, "发送", self._send_from_entry,
                     color="#283593", width=8).pack(side="left", padx=(6, 0))
        self.log("已启动。命令示例：KPX=3.0 / XVMAX=1200 / STOP / DMEN", "info")

    def _send_from_entry(self):
        text = self.cmd_var.get().strip()
        if text:
            self.send_line(text)
            self.cmd_history.append(text)
            self.hist_idx = len(self.cmd_history)
            self.cmd_var.set("")

    def _cmd_history(self, step):
        if not self.cmd_history:
            return
        self.hist_idx = max(0, min(len(self.cmd_history),
                                   self.hist_idx + step))
        if self.hist_idx < len(self.cmd_history):
            self.cmd_var.set(self.cmd_history[self.hist_idx])
        else:
            self.cmd_var.set("")

    # ------------------------- 数据记录页 -------------------------------
    def _build_record_tab(self):
        page = ttk.Frame(self.nb_ctrl, style="Panel.TFrame")
        self.nb_ctrl.add(page, text=" 数据记录 ")

        ttk.Label(page, text="将 24 通道遥测数据流保存为 CSV（UTF-8 BOM，\n"
                             "可直接用 Excel / pandas 打开分析）。",
                  style="Dim.TLabel", justify="left").pack(anchor="w", padx=10,
                                                           pady=(10, 8))
        self._mk_btn(page, "● 开始记录", self.toggle_record,
                     color="#b71c1c").pack(fill="x", padx=10, ipady=5)
        f2 = ttk.Frame(page, style="Panel.TFrame")
        f2.pack(fill="x", padx=10, pady=(8, 0))
        ttk.Label(f2, text="状态：", style="Panel.TLabel").pack(side="left")
        self.lbl_rec = ttk.Label(f2, text="未记录", style="Panel.TLabel")
        self.lbl_rec.pack(side="left")
        f3 = ttk.Frame(page, style="Panel.TFrame")
        f3.pack(fill="x", padx=10, pady=(6, 0))
        ttk.Label(f3, text="行数：", style="Panel.TLabel").pack(side="left")
        self.lbl_rec_rows = ttk.Label(f3, text="0", style="Panel.TLabel",
                                      font=("Consolas", 9, "bold"))
        self.lbl_rec_rows.pack(side="left")
        ttk.Label(page, text="文件：", style="Panel.TLabel").pack(
            anchor="w", padx=10, pady=(10, 0))
        self.lbl_rec_path = tk.Label(page, text="—", bg=BG_PANEL, fg=ACCENT,
                                     font=("Consolas", 8), anchor="w",
                                     wraplength=340, justify="left")
        self.lbl_rec_path.pack(fill="x", padx=10)
        self._mk_btn(page, "打开记录文件夹", self._open_record_folder,
                     width=18).pack(padx=10, pady=(14, 0), anchor="w")

        ttk.Separator(page).pack(fill="x", padx=10, pady=14)
        ttk.Label(page, text="导出当前波形缓冲为 CSV", style="Panel.TLabel").pack(
            anchor="w", padx=10)
        self._mk_btn(page, "导出…", self._export_buffer, width=10).pack(
            padx=10, pady=(6, 0), anchor="w")

    def _open_record_folder(self):
        folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "records"))
        os.makedirs(folder, exist_ok=True)
        try:
            os.startfile(folder)  # noqa: B606 Windows-only
        except Exception:
            messagebox.showinfo("路径", folder)

    def _export_buffer(self):
        view = self.ring.view()
        if view is None:
            messagebox.showinfo("导出", "当前缓冲为空")
            return
        t, data = view
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile="ILHC_buffer.csv",
                                            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["t_s"] + ["ch_%s" % c[1] for c in CHANNELS])
            for i in range(len(t)):
                w.writerow(["%.3f" % t[i]] +
                           ["%.5g" % v if math.isfinite(v) else ""
                            for v in data[i]])
        self.log("已导出 %d 行缓冲 -> %s" % (len(t), path), "info")

    # ------------------------------------------------------------------
    # 逻辑
    # ------------------------------------------------------------------
    def refresh_ports(self):
        if serial is None:
            self.cmb_port["values"] = []
            self.port_var.set("")
            self.lbl_status.configure(text="● 未安装 pyserial（模拟模式可用）", fg="#ffb74d")
            return
        ports = ["%s  %s" % (p.device, p.description) if p.description != p.device
                 else p.device for p in serial.tools.list_ports.comports()]
        self.cmb_port["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _cur_port(self):
        v = self.port_var.get()
        return v.split("  ")[0] if v else ""

    def toggle_connect(self, force_on=None):
        want = force_on if force_on is not None else (self.worker is None)
        if want and self.worker is None:
            if serial is None:
                messagebox.showerror(APP_NAME,
                                     "未安装 pyserial，无法连接真实串口。\n"
                                     "请执行：pip install pyserial\n"
                                     "或使用模拟模式。")
                return
            if self.sim is not None:
                self.toggle_sim(False)
            port = self._cur_port()
            if not port:
                messagebox.showwarning(APP_NAME, "请先选择串口（或点刷新）")
                return
            try:
                baud = int(self.baud_var.get())
            except ValueError:
                baud = DEFAULT_BAUD
            self.worker = SerialWorker(port, baud, self.frame_q, self.line_q,
                                       urgent_q=self.urgent_q, err_cb=self._worker_err)
            self.worker.start()
            self.last_heartbeat_enqueue = 0.0
            self.btn_conn.configure(text="  断开  ", bg="#8d6e63")
            self.lbl_status.configure(text="● 正在打开 %s @ %d..." % (port, baud),
                                      fg="#ffb74d")
            self.log("正在打开串口：%s @ %d 8N1" % (port, baud), "info")
        elif not want and self.worker is not None:
            w = self.worker
            self.worker = None
            w.request_stop(safe=True)
            self._clear_command_queues()
            self.btn_conn.configure(text="  连接  ", bg="#2e7d32")
            self.lbl_status.configure(text="● 未连接", fg=FG_DIM)
            self.log("串口已安全断开（已尝试 STOP / DMSTOP / DMOFF）", "info")

    def toggle_sim(self, force_on=None):
        want = force_on if force_on is not None else (self.sim is None)
        if want and self.sim is None:
            if self.worker is not None:
                self.toggle_connect(False)
            self.sim = Simulator(self.frame_q, self.line_q, self.urgent_q)
            self.sim.start()
            self.btn_sim.configure(bg="#00695c", text=" 模拟中… ")
            self.lbl_status.configure(text="● 模拟模式（未连接真实硬件）", fg="#4dd0e1")
            self.log("模拟模式已开启：遥测/回读均为仿真数据", "info")
        elif not want and self.sim is not None:
            self.sim.stop_flag = True
            self.sim = None
            self.btn_sim.configure(bg="#455a64", text=" 模拟模式 ")
            self.lbl_status.configure(text="● 未连接", fg=FG_DIM)
            self.log("模拟模式已关闭", "info")

    def toggle_pause(self):
        self.paused = not self.paused
        if not self.paused:
            self.force_full = True
        self.btn_pause.configure(text=" ▶ 继续 " if self.paused else " ⏸ 暂停 ",
                                 bg="#8d6e63" if self.paused else "#2c313a")

    def clear_data(self):
        self.ring.clear()
        self.traj_ring.clear()
        self.latest = None
        self.latest_t = 0.0
        self.force_full = True
        self.log("数据缓冲已清除", "info")

    def _set_window(self, seconds):
        """设置时间窗并按新容量重建历史缓冲"""
        self.window_s = seconds
        cap = int(seconds * SEND_HZ) + 100
        self.ring.resize(cap)
        self.traj_ring.resize(cap)
        self.force_full = True

    def _on_window_change(self, _e=None):
        try:
            seconds = float(self.win_var.get().split()[0])
        except ValueError:
            seconds = 30.0
        self._set_window(seconds)

    def toggle_record(self):
        if self.recorder is None:
            folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "records")
            self.recorder = CsvRecorder(folder)
            self.btn_record_top.configure(text=" ■ 停止记录 ")
            self.lbl_rec.configure(text="记录中…", foreground="#ef5350")
            self.lbl_rec_path.configure(text=self.recorder.path)
            self.log("开始记录 -> %s" % self.recorder.path, "info")
        else:
            self.recorder.close()
            self.log("停止记录，共 %d 行 -> %s" % (self.recorder.rows,
                                                 self.recorder.path), "info")
            self.recorder = None
            self.btn_record_top.configure(text=" ● 记录 ")
            self.lbl_rec.configure(text="未记录", foreground=FG)

    # ------------------------- 发送 ------------------------------------
    def _quick(self, cmd):
        return lambda: self.send_line(cmd)

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

    def _enqueue_heartbeat(self, now):
        """GUI 主循环产生心跳；GUI 卡死时这里不再执行，固件看门狗才会超时。"""
        if (self.worker is not None and self.worker.opened.is_set() and
                now - self.last_heartbeat_enqueue >= HEARTBEAT_INTERVAL_S):
            try:
                self.urgent_q.put_nowait("PING")
                self.last_heartbeat_enqueue = now
            except queue.Full:
                pass

    def send_line(self, text):
        text = str(text).strip()
        if not text:
            return
        if self.worker is not None or self.sim is not None:
            cmd = text.upper().split("=", 1)[0].strip()
            if cmd in URGENT_COMMANDS:
                # 急停/失能前丢弃尚未发送的普通运动/调参命令，避免 STOP 后又执行旧命令。
                self._drain_queue(self.line_q)
                self.urgent_q.put(text)
            else:
                self.line_q.put(text)
            self.send_count += 1
            self.log("TX> %s" % text, "tx")
        else:
            self.log("未连接，命令未发送：%s" % text, "warn")
            self.lbl_status.configure(text="● 未连接（命令未发送）", fg=FG_DIM)

    def log(self, text, tag="info"):
        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", "[%s] %s\n" % (time.strftime("%H:%M:%S"), text),
                            tag)
        if float(self.txt_log.index("end-1c").split(".")[0]) > 400:
            self.txt_log.delete("1.0", "30.0")
        self.txt_log.configure(state="disabled")
        self.txt_log.see("end")

    def _send_all_chassis(self):
        for (cmd, *_rest) in CHASSIS_PARAMS:
            self._send_param(cmd)

    def _worker_err(self, msg):
        # 串口线程回调 -> 转回主线程
        self.root.after(0, lambda: self._on_worker_err(msg))

    def _on_worker_err(self, msg):
        self.log(msg, "warn")
        if self.worker is not None:
            self.toggle_connect(False)
            messagebox.showerror(APP_NAME, msg)

    # ------------------------------------------------------------------
    # 周期刷新
    # ------------------------------------------------------------------
    def _tick(self):
        now_tick = time.monotonic()
        self._enqueue_heartbeat(now_tick)
        # 取队列帧 -> 环形缓冲（O(1) 写入）
        got = 0
        try:
            while got < 200:
                t, vals = self.frame_q.get_nowait()
                got += 1
                tr = t - self.t0_monotonic     # 相对时间（s），用于时间轴
                self.latest = vals
                self.latest_t = tr
                if not self.paused:
                    self.ring.append(tr, vals)
                    self.traj_ring.append(tr, (vals[0], vals[1]))
                if self.recorder is not None:
                    self.recorder.write(t, vals)
        except queue.Empty:
            pass
        self.fps_cnt += got

        # 绘制调度：仅当有新数据/需要重建时才画；blit 增量 + 低频全量
        if (got or self.force_full) and not self.paused:
            now = time.monotonic()
            try:
                tab = self.nb_plot.index(self.nb_plot.select())
            except tk.TclError:
                tab = 0
            if tab == 0:
                if self.force_full or now - self.last_full >= 0.6:
                    self._wave_clean(now)
                elif now - self.last_blit >= 0.07:
                    self._wave_blit(now)
            elif tab == 1 and now - self.last_traj >= 0.1:
                self._map_update(now)
        self.root.after(50, self._tick)

    def _on_plot_tab_changed(self, _e=None):
        self.force_full = True
        self.map_bg = None          # 切页后地图背景需按新画布尺寸重建

    @staticmethod
    def _sync_fig_size(fig, canvas):
        """把 figure 渲染尺寸同步到控件当前像素。

        隐藏页重新显示时 <Configure> 可能尚未投递，figure 仍是旧尺寸，
        直接绘制会导致背景与画布错位/裁剪。返回是否发生了同步。"""
        w = canvas.get_tk_widget().winfo_width()
        h = canvas.get_tk_widget().winfo_height()
        if w < 50 or h < 50:
            return False
        dpi = fig.dpi
        fw, fh = fig.get_size_inches()
        if abs(fw * dpi - w) > 2 or abs(fh * dpi - h) > 2:
            fig.set_size_inches(w / dpi, h / dpi)
            return True
        return False

    # ------------------------- 波形绘制（blit 加速） -------------------
    def _visible_series(self):
        """取可见通道的 (t, v) numpy 数组，窗口外数据裁掉，非有限值置 NaN 断线"""
        view = self.ring.view()
        out = {}
        if view is None:
            return out
        t, data = view
        start = np.searchsorted(t, self.latest_t - self.window_s)
        if start > 0:
            t = t[start:]
            data = data[start:]
        for (idx, _k, _l, _u, gg) in CHANNELS:
            if not self.cb_vars[idx].get():
                continue
            col = data[:, idx]
            if not np.all(np.isfinite(col)):
                col = np.where(np.isfinite(col), col, np.nan)
            out[idx] = (t, col)
        return out

    def _wave_clean(self, now):
        """全量刷新（约 1.7 Hz）：先画纯静态背景（网格/图例/刻度）并缓存，
        供两次全量刷新之间的 blit 增量绘制复用。"""
        self.force_full = False
        self.last_full = self.last_blit = now
        if self._sync_fig_size(self.fig, self.canvas):
            # 渲染尺寸刚同步，让 blit 的尺寸检查以新画布为准
            widget = self.canvas.get_tk_widget()
            self.last_canvas_wh = (widget.winfo_width(), widget.winfo_height())
        t_now = self.latest_t

        # 按分组统计可见通道的 y 范围
        yrange = {}
        view = self.ring.view()
        if view is not None:
            _t, data = view
            start = np.searchsorted(_t, t_now - self.window_s)
            if start > 0:
                data = data[start:]
            for (idx, _k, _l, _u, gg) in CHANNELS:
                if not self.cb_vars[idx].get():
                    continue
                col = data[:, idx]
                if not np.any(np.isfinite(col)):
                    continue
                lo = float(np.nanmin(col))
                hi = float(np.nanmax(col))
                if gg in yrange:
                    a, b = yrange[gg]
                    yrange[gg] = (min(a, lo), max(b, hi))
                else:
                    yrange[gg] = (lo, hi)

        for g, ax in self.ax_group.items():
            ax.set_xlim(t_now - self.window_s, t_now + 0.02)
            if g in yrange:
                lo, hi = yrange[g]
                if math.isfinite(lo) and math.isfinite(hi) and hi > lo:
                    pad = (hi - lo) * 0.12 + 1e-9
                    ax.set_ylim(lo - pad, hi + pad)
                else:
                    ax.set_ylim(-1, 1)
            else:
                ax.set_ylim(-1, 1)

        # 隐藏全部曲线画纯背景 -> 缓存各子图背景 -> 立即 blit 曲线
        vis = {i: ln.get_visible() for i, ln in self.lines.items()}
        for ln in self.lines.values():
            ln.set_visible(False)
        try:
            self.canvas.draw()
            self.bg = {g: self.canvas.copy_from_bbox(ax.bbox)
                       for g, ax in self.ax_group.items()}
        except Exception:
            self.bg = {}
        finally:
            for i, v in vis.items():
                self.lines[i].set_visible(v)
        self._wave_blit(now)

    def _wave_blit(self, now, series=None):
        """增量绘制（约 14 Hz）：恢复静态背景 + 仅重画曲线，开销为全量的约 1/8"""
        self.last_blit = now
        if not self.bg:
            self.force_full = True
            return
        widget = self.canvas.get_tk_widget()
        wh = (widget.winfo_width(), widget.winfo_height())
        if self.last_canvas_wh != wh:      # 窗口尺寸变化 -> 重建背景
            self.last_canvas_wh = wh
            self.force_full = True
            return
        if series is None:
            series = self._visible_series()
        try:
            for g, ax in self.ax_group.items():
                self.canvas.restore_region(self.bg[g])
                for (idx, _k, _l, _u, gg) in CHANNELS:
                    if gg == g and idx in series:
                        ln = self.lines[idx]
                        ln.set_data(series[idx][0], series[idx][1])
                        ax.draw_artist(ln)
                self.canvas.blit(ax.bbox)
        except Exception:
            self.force_full = True

    def _slow_tick(self):
        # 帧率统计
        now = time.monotonic()
        if now - self.fps_t >= 1.0:
            self.fps = int(self.fps_cnt / (now - self.fps_t))
            self.fps_cnt = 0
            self.fps_t = now
        self.lbl_fps.configure(text="帧率 %d/s" % self.fps)

        # 串口状态：区分“线程启动”“串口打开”“真正收到遥测”。
        if self.worker is not None:
            w = self.worker
            if not w.opened.is_set():
                self.lbl_status.configure(text="● 正在打开 %s @ %d..." % (w.port, w.baud),
                                          fg="#ffb74d")
            elif w.last_frame_monotonic <= 0:
                self.lbl_status.configure(text="● 串口已打开，等待遥测 %s" % w.port,
                                          fg="#ffb74d")
            else:
                age = now - w.last_frame_monotonic
                if age <= TELEMETRY_WARN_S:
                    self.lbl_status.configure(
                        text="● 在线 %s @ %d · 遥测 %.0f ms" % (w.port, w.baud, age * 1000),
                        fg="#81c784")
                elif age <= TELEMETRY_TIMEOUT_S:
                    self.lbl_status.configure(
                        text="● 遥测延迟 %s · %.0f ms" % (w.port, age * 1000),
                        fg="#ffb74d")
                else:
                    self.lbl_status.configure(
                        text="● 遥测超时 %s · %.1f s" % (w.port, age), fg="#ef5350")

        p = self.worker.parser if self.worker is not None else None
        if p is not None:
            self.lbl_err.configure(text="错误字节 %d" % p.err_bytes)
            self.lbl_bytes.configure(text="接收 %.1f KB" % (p.bytes_in / 1024.0))
        elif self.sim is not None:
            self.lbl_err.configure(text="错误字节 0（模拟）")
            self.lbl_bytes.configure(text="模拟 50 Hz")

        # 参数回读
        v = self.latest
        if v is not None:
            for row in list(self.chassis_rows.values()) + list(self.dm_rows.values()):
                if row["rb_ch"] is not None:
                    val = v[row["rb_ch"]]
                    if math.isfinite(val):
                        row["rb"].configure(text="回读 " + self._fmt_readback(val))
            # DM 反馈区
            self.dm_fb_labels["pos"].configure(text="%.3f" % v[13])
            self.dm_fb_labels["vel"].configure(text="%.3f" % v[14])
            self.dm_fb_labels["tor"].configure(text="%.3f" % v[15])
            self.dm_fb_labels["tmos"].configure(text="%.1f" % v[17])
            self.dm_fb_labels["trotor"].configure(text="%.1f" % v[18])
            self.dm_fb_labels["id"].configure(text="%d" % int(v[12]))
            status = int(v[16]) & 0xFF
            text, fault = DM_STATUS.get(status, ("未知(0x%02X)" % status, True))
            color = "#ef5350" if fault else ("#81c784" if status == 0x01 else FG_DIM)
            self.lbl_dm.configure(text="DM[%d]: %s" % (int(v[12]), text), fg=color)
            self.lbl_dm_status_big.configure(text="DM 状态：0x%02X  %s" % (status, text),
                                             fg=color)
        if self.recorder is not None:
            self.lbl_rec_rows.configure(text=str(self.recorder.rows))
        self.root.after(self.slow_ms, self._slow_tick)

    # ------------------------------------------------------------------
    def show_help(self):
        messagebox.showinfo(
            "帮助 — 协议速查",
            "【硬件连接】USART1 (PA9=TX, PA10=RX)，115200 8N1，USB-TTL 交叉连接。\n\n"
            "【遥测】50 Hz，100 字节/帧：\n"
            "  24×float(小端) + 0x00 0x00 0x80 0x7F\n\n"
            "【命令】每行一条，\\r 或 \\n 结尾，大小写不敏感：\n"
            "  底盘：STOP / ZERO / GOTO=x,y,z（点击场地地图自动下发）/ PING\n"
            "        KPX= / KPY= / KPZ= (0~50)\n"
            "        XVMAX= ZVMAX= (0~3000) / XVMIN= ZVMIN= (0~100)\n"
            "  DM 电机：DMID= (1~1791) / DMMODE= 1|2\n"
            "        DMPOS= (±12.5) / DMVEL= (±30) / DMKP= (0~500)\n"
            "        DMKD= (0~5) / DMTOR= (±10) / DMEN / DMOFF / DMZERO\n\n"
            "【场地地图】需固件支持 GOTO 命令（重新编译烧录 debug_usart.c）。\n"
            "  ① 车子摆进启停区 → 选“启停区1/2” → 点“置为原点并归零”：\n"
            "     区中心即坐标原点，同时下发 ZERO 让 OPS 归零；\n"
            "  ② 点击地图或用“快速前往”按钮即可让小车移动；\n"
            "  ③ v1.1 会拒绝目标点落入固定设备、或直线路径明显穿越设备；\n"
            "     当前不含车体半径膨胀/A* 自动绕障，必要时请用中间点绕行；\n"
            "  ④ 若 OPS 零点朝向与场地不一致，用映射“旋转°”校正。\n\n"
            "【v1.1 安全】PING 由 GUI 主循环产生；STOP/DMOFF 高优先级；\n"
            "断开/退出会尝试 STOP + DMSTOP + DMOFF。\n\n"
            "固件无文本应答，参数以遥测回读通道为准。\n"
            "详见工程 Hardware/调试指令手册.md。")

    def _bind_close(self):
        def on_close():
            try:
                if self.recorder:
                    self.recorder.close()
                    self.recorder = None
                if self.worker:
                    w = self.worker
                    self.worker = None
                    w.request_stop(safe=True)
                if self.sim:
                    self.sim.stop_flag = True
                    self.sim = None
                self._clear_command_queues()
                self.root.after(120, self.root.destroy)
            except Exception:
                self.root.destroy()
        self.root.protocol("WM_DELETE_WINDOW", on_close)


# ======================================================================
# 自检（无 GUI）：解析器 + 模拟器回环
# ======================================================================
def selftest():
    print("[1] FrameParser 基本解析 ...")
    p = FrameParser()
    frame0 = struct.pack("<24f", *range(24)) + FRAME_TAIL
    frame1 = struct.pack("<24f", *[_ + 100 for _ in range(24)]) + FRAME_TAIL
    frames = p.feed(b"garbage" + frame0 + frame1[:40])
    assert len(frames) == 1 and abs(frames[0][5] - 5.0) < 1e-6
    frames += p.feed(frame1[40:] + b"\x00" * 7)
    assert len(frames) == 2 and abs(frames[1][23] - 123.0) < 1e-6
    print("    ok, frames=%d err_bytes=%d" % (p.frames_ok, p.err_bytes))

    print("[1b] payload 内嵌 JustFloat 帧尾（+inf）...")
    p_inf = FrameParser()
    vals_inf = [float(i) for i in range(24)]
    vals_inf[7] = float("inf")
    raw_inf = struct.pack("<24f", *vals_inf) + FRAME_TAIL
    out_inf = p_inf.feed(raw_inf[:31]) + p_inf.feed(raw_inf[31:])
    assert len(out_inf) == 1 and math.isinf(out_inf[0][7])
    print("    ok")

    print("[2] 坏帧 / 伪帧头 ...")
    p2 = FrameParser()
    bad = b"\x01\x02" * 50 + b"\x11\x22\x33\x44"          # 无合法帧尾
    frames = p2.feed(bad + frame0)
    assert len(frames) == 1 and p2.err_bytes > 0
    n0 = p2.frames_ok
    p2.feed(b"\x00" * 1000)
    assert p2.frames_ok == n0
    print("    ok, err_bytes=%d" % p2.err_bytes)

    print("[3] 模拟器命令解析（镜像固件限幅）...")
    sim = Simulator(queue.Queue(), queue.Queue())
    sim.handle_line("kpx=3.0");   assert sim.kpx == 3.0
    sim.handle_line("KPX=999");   assert sim.kpx == 50.0        # 上限
    sim.handle_line("KPZ=-5");    assert sim.kpz == 0.0         # 下限
    sim.handle_line("XVMAX=2000"); assert sim.xyvmax == 2000.0
    sim.handle_line("DMID=0.5");  assert sim.dm_id == 1         # <1 忽略
    sim.handle_line("DMID=3");    assert sim.dm_id == 3
    sim.handle_line("DMVEL=99");  assert sim.dm_vel == 30.0
    sim.handle_line("DMPOS=-99"); assert sim.dm_pos == -12.5
    sim.handle_line("DMKD=2.5");  assert sim.dm_kd == 2.5
    sim.handle_line("DMEN");      assert sim.dm_active == 1
    vals = sim.make_frame(1.0)
    assert abs(vals[6] - 50.0) < 1e-6          # ch6 回读 KPX
    assert abs(vals[20] - 30.0) < 1e-6         # ch20 回读 DMVEL
    assert abs(vals[16] - 1.0) < 1e-6          # ch16 状态=使能
    sim.handle_line("DMOFF");     assert sim.dm_active == 0
    print("    ok")

    print("[4] 模拟器 x 解析器 回环 ...")
    q = queue.Queue()
    sim2 = Simulator(q, queue.Queue())
    for i in range(10):
        f = sim2.make_frame(i / 50.0)
        parser = FrameParser()
        raw = struct.pack("<24f", *f) + FRAME_TAIL
        out = parser.feed(raw[:37]) + parser.feed(raw[37:])
        assert len(out) == 1
        # 与 float32 往返后的值比对
        ref = struct.unpack("<24f", struct.pack("<24f", *f))
        for a, b in zip(out[0], ref):
            assert abs(a - b) < 1e-5
    print("    ok")
    print("[5] GOTO 命令解析与运动仿真（场地地图联动）...")
    sim3 = Simulator(queue.Queue(), queue.Queue())
    sim3.handle_line("goto=1200,800,90")
    assert sim3.goto == (1200.0, 800.0, 90.0)
    sim3.handle_line("GOTO=9999,800")            # x 限幅 + z 省略
    assert sim3.goto is not None
    assert sim3.goto[0] == 3000.0 and len(sim3.goto) == 3
    sim3.handle_line("STOP")
    assert sim3.goto is None and sim3.hold is not None   # STOP 取消并停住

    sim4 = Simulator(queue.Queue(), queue.Queue())
    sim4.handle_line("GOTO=1000,600")
    start = sim4.make_frame(0.0)
    d0 = math.hypot(start[0] - 1000, start[1] - 600)
    f = start
    for i in range(250):                          # 5 s，应到达目标
        f = sim4.make_frame(i / 50.0)
    d1 = math.hypot(f[0] - 1000, f[1] - 600)
    assert d1 < d0 - 300 and d1 < 50.0            # 移动到位
    assert sim4.goto is None                      # 到位后清除目标
    print("    ok")

    print("[6] 启停区置零流程（区中心=原点）...")
    sim5 = Simulator(queue.Queue(), queue.Queue())
    for i in range(30):                           # 先巡航一段时间
        sim5.make_frame(i / 50.0)
    sim5.handle_line("ZERO")                      # 车放在启停区后归零
    assert sim5.hold == (0.0, 0.0)
    f = sim5.make_frame(1.0)
    assert abs(f[0]) < 5.0 and abs(f[1]) < 5.0    # 归零后位置读 ~0
    sim5.handle_line("GOTO=600,0,0")
    for i in range(150):
        f = sim5.make_frame(i / 50.0)
    assert math.hypot(f[0] - 600, f[1] - 0) < 50.0   # 从原点出发移动
    print("    ok")

    print("[7] 场地 GOTO 静态禁区检查...")
    app_stub = object.__new__(App)
    assert app_stub._field_point_blocked(600.0, 600.0) == "中央物料区"
    assert app_stub._field_point_blocked(1200.0, 2200.0) is None
    assert app_stub._field_path_blocked(2250.0, 2250.0, 330.0, 1200.0) is not None
    assert app_stub._field_path_blocked(2250.0, 2250.0, 1200.0, 2200.0) is None
    print("    ok")

    print("[8] STOP 高优先级清空普通命令...")
    broker = object.__new__(App)
    broker.worker, broker.sim = object(), None
    broker.line_q, broker.urgent_q = queue.Queue(), queue.Queue()
    broker.send_count = 0
    broker.log = lambda *_a, **_k: None
    broker.lbl_status = None
    App.send_line(broker, "GOTO=100,200,0")
    assert broker.line_q.qsize() == 1
    App.send_line(broker, "STOP")
    assert broker.line_q.empty() and broker.urgent_q.get_nowait() == "STOP"
    print("    ok")

    print("\n全部自检通过 ✔")


# ======================================================================
def main():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    ap = argparse.ArgumentParser(description=APP_NAME)
    ap.add_argument("--port", help="直接连接指定串口，如 COM3")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--simulate", action="store_true", help="启动即进入模拟模式")
    ap.add_argument("--ui-test", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--selftest", action="store_true", help="运行无 GUI 自检")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
