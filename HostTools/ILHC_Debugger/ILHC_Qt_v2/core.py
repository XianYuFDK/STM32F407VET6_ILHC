# -*- coding: utf-8 -*-
"""ILHC Qt v2 core.

从 ILHC 调试上位机 v1.1 提取的非 GUI 核心：
- JustFloat 24 通道解析
- O(1) 环形缓冲
- 串口工作线程 + 高优先级急停队列
- 模拟器
- CSV 记录器
- 场地静态禁区几何判断
"""

import csv
import math
import os
import queue
import random
import struct
import threading
import time

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

APP_VERSION = "v2.0 Qt"
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
        self._t = 0.0

    # ---------- 命令解析（镜像 Debug_ParseLine / Debug_SetDmValue） ----------
    @staticmethod
    def _clamp(v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    def handle_line(self, line):
        line = line.strip().upper()
        if not line:
            return
        if line == "STOP":
            if self.hold is None:                  # 原地停住（同固件急停）
                self.hold = (600.0 * math.sin(0.25 * self._t),
                             450.0 * math.cos(0.19 * self._t))
            self.goto = None                       # 取消 GOTO 目标（同固件）
            return
        if line == "ZERO":
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

FIELD_SIZE = 2400.0


def point_in_rect(x, y, rect):
    x0, y0, x1, y1, _name = rect
    return x0 <= x <= x1 and y0 <= y <= y1


def seg_intersects_rect(x0, y0, x1, y1, rect):
    """Liang-Barsky 线段/轴对齐矩形相交测试。"""
    rx0, ry0, rx1, ry1, _name = rect
    if point_in_rect(x0, y0, rect) or point_in_rect(x1, y1, rect):
        return True
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


def seg_intersects_circle(x0, y0, x1, y1, circle):
    cx, cy, r, _name = circle
    dx, dy = x1 - x0, y1 - y0
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.hypot(x0 - cx, y0 - cy) <= r
    u = ((cx - x0) * dx + (cy - y0) * dy) / den
    u = min(1.0, max(0.0, u))
    px, py = x0 + u * dx, y0 + u * dy
    return math.hypot(px - cx, py - cy) <= r


def field_point_blocked(x, y):
    for rect in FIELD_FORBIDDEN_RECTS:
        if point_in_rect(x, y, rect):
            return rect[4]
    for cx, cy, r, name in FIELD_FORBIDDEN_CIRCLES:
        if math.hypot(x - cx, y - cy) <= r:
            return name
    return None


def field_path_blocked(x0, y0, x1, y1):
    for rect in FIELD_FORBIDDEN_RECTS:
        if seg_intersects_rect(x0, y0, x1, y1, rect):
            return rect[4]
    for circle in FIELD_FORBIDDEN_CIRCLES:
        if seg_intersects_circle(x0, y0, x1, y1, circle):
            return circle[3]
    return None


def selftest():
    """不依赖 Qt 的核心自检。"""
    print('[1] FrameParser 基本解析 ...')
    p = FrameParser()
    frame0 = struct.pack('<24f', *range(24)) + FRAME_TAIL
    out = p.feed(b'garbage' + frame0[:35]) + p.feed(frame0[35:])
    assert len(out) == 1 and abs(out[0][5] - 5.0) < 1e-6
    print('    ok')

    print('[2] payload 内嵌帧尾 (+inf) ...')
    vals = [float(i) for i in range(24)]
    vals[7] = float('inf')
    raw = struct.pack('<24f', *vals) + FRAME_TAIL
    p = FrameParser()
    out = p.feed(raw[:61]) + p.feed(raw[61:])
    assert len(out) == 1 and math.isinf(out[0][7])
    print('    ok')

    print('[3] Simulator 参数/DM ...')
    sim = Simulator(queue.Queue(), queue.Queue(), queue.Queue())
    sim.handle_line('KPX=999')
    sim.handle_line('DMEN')
    assert sim.kpx == 50.0 and sim.dm_active == 1
    print('    ok')

    print('[4] 场地禁区 ...')
    assert field_point_blocked(600, 600) == '中央物料区'
    assert field_point_blocked(1200, 2200) is None
    assert field_path_blocked(2250, 2250, 330, 1200) is not None
    assert field_path_blocked(2250, 2250, 1200, 2200) is None
    print('    ok')
    print('全部核心自检通过 ✔')
