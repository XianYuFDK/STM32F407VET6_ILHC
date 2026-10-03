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
import copy
import functools
import heapq
from collections import deque
import re
import math
import os
import queue
import random
import struct
import threading
import time

import numpy as np


class CommandQueue(queue.Queue):
    """普通命令保序；待发MANUAL只保留最新目标，并优先于普通参数。

    Queue在持锁状态调用_put，替换与工作线程取出不会交叉；STOP仍走独立紧急队列。
    """
    def _put(self, item):
        if str(item).strip().upper().startswith("MANUAL="):
            kept = deque(cmd for cmd in self.queue
                         if not str(cmd).strip().upper().startswith("MANUAL="))
            self.unfinished_tasks -= len(self.queue) - len(kept)
            self.queue = kept
            self.queue.appendleft(item)
        else:
            super()._put(item)

# 会给机构下命令的命令名：用于"规划/预览模式不得下发"这类断言与诊断。
# 注意它与 discard_motion_commands 的集合**刻意不同**：后者只清"可以丢弃的旧目标"
# （GOTO/GOTOHOLD/MANUAL/ZDT/VTRACK），绝不能把 STOP/ZERO/WHEEL* 这类安全命令一起丢掉。
COMMANDING_COMMANDS = {
    "GOTO", "GOTOHOLD", "MANUAL", "ZDT", "VTRACK", "STOP", "ZERO", "OPSOFFSET",
    "WHEELEN", "WHEELOFF", "DMEN", "DMOFF", "DMSTOP", "DMZERO", "DMMODE",
    "DMID", "DMPOS", "DMVEL", "DMKP", "DMKD", "DMTOR",
    "S28MOVE", "S28RAW", "S28HOME", "S28CANCEL",
    "S35MOVE", "S35RAW", "S35HOME", "S35CANCEL", "S35EN", "S28EN",
}


def commanding_commands(items):
    """从命令序列里挑出会给机构下命令的条目（参数回读 GET/心跳 PING 不算）。"""
    out = []
    for item in items:
        name = str(item).strip().upper().split("=", 1)[0].strip()
        if name in COMMANDING_COMMANDS:
            out.append(item)
    return out


def discard_motion_commands(command_queue):
    """Remove queued motion atomically; an active simulator goal is cancelled separately.

    VTRACK 也在此列：它会让固件接管底盘闭环，属于"可丢弃的旧目标"。若不丢，
    用户先点「跟踪」再按 STOP 时，排在 STOP 后面的那条 VTRACK 会在停止之后把
    跟踪重新拉起来（与 MANUAL/GOTO 是同一类竞态）。
    """
    with command_queue.mutex:
        kept = deque(cmd for cmd in command_queue.queue
                     if str(cmd).strip().upper().split("=", 1)[0] not in
                     {"GOTO", "GOTOHOLD", "MANUAL", "ZDT", "VTRACK"})
        removed = len(command_queue.queue) - len(kept)
        command_queue.queue.clear()
        command_queue.queue.extend(kept)
        command_queue.unfinished_tasks = max(0, command_queue.unfinished_tasks-removed)
        command_queue.not_full.notify_all()
        if command_queue.unfinished_tasks == 0:
            command_queue.all_tasks_done.notify_all()


try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

APP_VERSION = "v2.1.2 Qt A* map click fix"
FRAME_TAIL = b"\x00\x00\x80\x7F"
FRAME_FLOATS = 24
FRAME_DATA_LEN = 4 * FRAME_FLOATS
FRAME_LEN = FRAME_DATA_LEN + 4                 # 100 字节
DEFAULT_BAUD = 115200
SEND_HZ = 50                                   # 固件 20 ms 一帧
HEARTBEAT_INTERVAL_S = 0.20                      # GUI 存活心跳，5 Hz
TELEMETRY_WARN_S = 0.35                          # 遥测延迟黄色阈值
TELEMETRY_TIMEOUT_S = 1.00                       # 遥测超时红色阈值
URGENT_COMMANDS = {"STOP", "DMSTOP", "DMOFF", "S28CANCEL", "S35CANCEL"}
APP_NAME = "ILHC 调试上位机"
# OPS 移动协议统一用 cm；地图几何/内部运动模拟仍用 mm，避免改变既有半径和限幅。
OPS_CM_TO_MM = 10.0

# 固件在 CAN 启动失败或 ZDT 文字调试时会进入文字模式并停止 24 通道遥测，
# 只有收到 VOFA 才恢复。连接瞬间的那次 VOFA 可能落在固件启动空窗里丢失，
# 因此这里在收不到遥测时自动补发，避免"板子复位后串口再也收不到数据"。
VOFA_RETRY_GAP_S = 1.5      # 超过该秒数没有可解析帧就补发一次 VOFA
VOFA_MAX_RETRIES = 6        # 每次中断最多补发次数；收到任意一帧后重新计数
FIRMWARE_TEXT_MAX = 160     # 单行固件文本长度上限
PARAM_POLL_S = 1.0          # 无遥测通道参数的文字回读轮询周期
# 固件参数回读应答行：GET <名称> 的回复固定为 "<名称>=<值>"。
PARAM_ECHO_RE = re.compile(r"^([A-Za-z][A-Za-z0-9_]{0,15})=(-?(?:\d+(?:\.\d+)?|\.\d+))$")

# ======================================================================
# 通道定义（与 debug_usart.c 中 data[0..23] 严格一致）
#   (下标, 键名, 显示名, 单位, 分组)
# ======================================================================
G_CHASSIS_POS = 0      # OPS 定位
G_CHASSIS_PID = 1      # 底盘参数 / 轮速
G_DM_FB = 2            # DM 电机反馈
G_DM_CMD = 3           # DM 电机目标

CHANNELS = [
    (0,  "pos_x",        "OPS X 坐标（左右）", "cm",   G_CHASSIS_POS),
    (1,  "pos_y",        "OPS Y 坐标（前后）", "cm",   G_CHASSIS_POS),
    (2,  "zangle",       "航向角",       "°",    G_CHASSIS_POS),
    (3,  "devx",         "X 轴误差（左右）", "cm", G_CHASSIS_POS),
    (4,  "devy",         "Y 轴误差（前后）", "cm", G_CHASSIS_POS),
    (5,  "devz",         "航向误差",     "",     G_CHASSIS_POS),
    (6,  "mKpx",         "X 轴 P（左右）", "",   G_CHASSIS_PID),
    (7,  "mKpy",         "Y 轴 P（前后）", "",   G_CHASSIS_PID),
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
    ("KPX",   "X 轴 P 系数（左右）",   0.0, 50.0,   2.3,  6),
    ("KPY",   "Y 轴 P 系数（前后）",   0.0, 50.0,   2.3,  7),
    ("KPZ",   "航向 P 系数",   0.0, 50.0,   9.0,  8),
    ("XVMAX", "X/Y 速度限幅",  0.0, 3000.0, 1600.0, 9),
    ("ZVMAX", "航向速度限幅",  0.0, 3000.0, 750.0, 10),
    ("XVMIN", "X/Y 最小补偿",  0.0, 100.0,  5.0,  None),
    ("ZVMIN", "航向最小补偿",  0.0, 100.0,  5.0,  None),
]

# DM 可调参数：(命令, 显示名, 最小, 最大, 默认, 回读通道, 是否整数)
DM_PARAMS = [
    ("DMID",  "电机 ID",     1.0, 1791.0,  3.0,  12, True),
    ("DMPOS", "目标位置 rad", -12.5, 12.5, 0.0,  19, False),
    ("DMVEL", "目标速度 rad/s", -30.0, 30.0, 0.0, 20, False),
    ("DMKP",  "MIT Kp",      0.0, 500.0,   2.0,  21, False),
    ("DMKD",  "MIT Kd",      0.0, 5.0,     0.5,  22, False),
    ("DMTOR", "前馈力矩 Nm",  -10.0, 10.0,  0.0,  23, False),
]

# 视觉跟踪（协议 V1.1）：颜色编号与固件 debug_usart.c、Jetson 完全一致，不要另建第二套映射。
VISION_COLORS = [
    (1, "红"), (2, "绿"), (3, "蓝"), (4, "黄"), (5, "黑"), (6, "浅蓝"),
]

# 视觉可调参数：(命令, 显示名, 最小, 最大, 默认, 回读通道)。
# 字段与 CHASSIS_PARAMS 同序；视觉参数没有遥测通道，第 6 项恒为 None，只能靠 GET 文字回读。
# 默认值必须与固件 vision_track.c 的编译期默认值一致，否则界面初值会与实际生效值不符。
VISION_PARAMS = [
    ("VCONF", "置信度阈值 %",       0.0,  100.0,  50.0, None),
    ("VKPMM", "毫米模式增益 RPM/mm", 0.0,    5.0,   0.5, None),
    ("VDBMM", "毫米模式死区 mm",     0.0,  100.0,   2.0, None),
    ("VDBPX", "像素模式死区 像素",   0.0,  200.0,  12.0, None),
    ("VMIN",  "出死区最小速度 RPM",  0.0,   60.0,   8.0, None),
    ("VMAX",  "每轴速度上限 RPM",    0.0, 3000.0,  60.0, None),
]


def vtrack_command(color: int) -> str:
    """VTRACK 命令：1..6 按颜色启动物料跟踪，0 停止。"""
    return "VTRACK=%d" % int(color)


# 模拟器参数回读用的 命令名 -> 属性名 映射（与固件 s_params 同名）。
SIM_PARAM_ATTRS = {
    "KPX": "kpx", "KPY": "kpy", "KPZ": "kpz",
    "XVMAX": "xyvmax", "ZVMAX": "zvmax",
    "XVMIN": "xyvmin", "ZVMIN": "zvmin",
    "VCONF": "vtrack_conf", "VKPMM": "vtrack_kpmm", "VDBMM": "vtrack_dbmm",
    "VDBPX": "vtrack_dbpx", "VMIN": "vtrack_min", "VMAX": "vtrack_max",
}

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

# 车体外形（与地图同一 mm 单位；供地图图标使用，后续也可用于禁区膨胀）
CAR_LENGTH_MM = 280.0   # 车长 28 cm，沿车头方向
CAR_WIDTH_MM = 260.0    # 车宽 26 cm，沿车左方向


def layout_to_field(x, y):
    """旋转后屏幕坐标：启停区1中心为零，屏幕左为+X、上为+Y，与协议轴序一致
    （X=左右轴、Y=前后轴）。"""
    return 2250.0 - y, 2250.0 - x


def field_to_layout(x, y):
    return 2250.0 - y, 2250.0 - x


STEPPER_CONFIG = {
    35: ("35 升降电机", "目标高度", 43.0, 203.0, 203.0, 1, 100, 0x100),
    28: ("28 伸缩电机", "目标半径", 120.0, 286.0, 120.0, 2, 5662, 0x200),
}


def stepper_move_command(motor, position_mm, speed):
    """机械位置以0.1mm整数发送，避免MCU小数解析与单位歧义。"""
    _, _, lo, hi, _, vlo, vhi, _ = STEPPER_CONFIG[motor]
    if not (math.isfinite(position_mm) and lo <= position_mm <= hi
            and vlo <= speed <= vhi and int(speed) == speed):
        raise ValueError("步进目标或速度超范围")
    return "S%dMOVE=%d,%d" % (motor, round(position_mm * 10), speed)


def stepper_feedback(text):
    """解读真实固件诊断；返回(电机号, 显示文字)，不由TX日志推断硬件状态。"""
    match = re.search(r"\bS(28|35)\s+(.+)", str(text))
    if not match:
        return None
    motor, message = int(match.group(1)), match.group(2).strip()
    data_match = re.fullmatch(r"RX CAN=([0-9A-Fa-f]+) DATA=([0-9A-Fa-f ]+)", message)
    if data_match:
        try:
            data = bytes.fromhex(data_match.group(2))
        except ValueError:
            return motor, "回复数据格式异常：" + message
        if len(data) < 3 or data[-1] != 0x6B:
            return motor, "回复长度或校验异常：" + message
        if data[0] == 0x3A and len(data) == 3:
            flags = data[1]
            return motor, "驱动已回包 · %s · %s · %s" % (
                "已使能" if flags & 1 else "未使能",
                "堵转保护已触发" if flags & 8 else "无堵转保护",
                "已到位" if flags & 2 else "未到位")
        if len(data) == 3:
            status = {0x02: "驱动确认命令正确（不代表到位）",
                      0x9F: "驱动返回动作完成",
                      0xE2: "驱动拒绝：参数或执行条件不满足，请查使能/校准/保护",
                      0xEE: "驱动拒绝：命令格式错误"}.get(data[1])
            if status:
                return motor, status + " · " + message
        return motor, "收到驱动回复 · " + message
    for key, value in (("QUEUED", "STM32已接收请求，等待CAN提交"),
                       ("CAN_SUBMITTED", "已提交CAN邮箱，等待驱动回包"),
                       ("CAN_TX_FAILED", "CAN提交失败，请检查固件CAN状态"),
                       ("CAN_TX_TIMEOUT", "CAN邮箱持续忙，待发请求已超时"),
                       ("NO_REPLY", "500ms未收到匹配回包，请检查CAN接线、收发器、速率和地址"),
                       ("FORMAT/RANGE", "STM32拒绝：命令格式或参数超范围"),
                       ("CANCELLED", "已取消待发请求；已启动的电机不会因此停车")):
        if message.startswith(key):
            return motor, value
    return motor, message
# 快速前往的**功能区锚点**（尺寸取自官方场地图）：名称 → (锚点, 由功能区指向场内的单位方向)。
# 锚点取功能区朝场内的一侧：原料区圆盘用圆心（圆心压在场地边界上）、暂存区用右缘中点、
# 粗加工区用上缘中点。
# 真正下发的目标必须用 approach_target() 按**当前航向+当前裕量**算出来，不能写死坐标：
# 曾经写死的 (1200,2200) 对 280×260 车体是非法停车位（离圆盘表面只有 90mm，车体要 140~205mm），
# 于是"点击原料区 → 规划失败"。守卫见 tests/test_navigation_safety.py 的
# test_quick_targets_are_legal_for_all_headings。
QUICK_ANCHORS = [
    ("原料区", (1200.0, 2400.0), (0.0, -1.0)),
    ("暂存区", (150.0, 1200.0), (1.0, 0.0)),
    ("粗加工区", (1200.0, 150.0), (0.0, 1.0)),
]
APPROACH_STANDOFF_MM = 60.0     # 接近点要求的车体外缘净空（含合法区边界；按机构取料需求调）

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
        self._carry = bytearray()
        self._text_lines = []
        self._seen_text = []
        self._param_lines = []

    def _scan_text(self, data):
        """从字节流里拾取可读 ASCII 行（固件的文字应答/错误行）。

        固件切到文字模式后只发 ASCII 应答且都以 CRLF 结尾，而 JustFloat 遥测
        几乎不会出现"连续可读字符 + 换行"，因此按"可读字节累积、遇换行成行、
        遇二进制字节清空候选"提取即可。这样 CAN 启动失败之类的报错能在界面
        直接看到，而不是被解析器静默丢弃。
        """
        for b in data:
            if b != 10:
                if 32 <= b < 127 or b == 13:
                    self._carry.append(b)
                    del self._carry[:-FIRMWARE_TEXT_MAX]
                else:
                    del self._carry[:]
                continue
            # 在遇到换行时立即消费；同一次read中后续遥测不能擦掉已完成的文字行。
            raw = bytes(self._carry)
            self._carry.clear()
            line = raw.replace(b"\r", b"").strip().decode("ascii", "ignore")
            # 参数回读行是状态而不是日志：GET XVMIN 连续两次值相同也必须每次都
            # 交付，因此先于 _seen_text 去重判断，单独收集。
            echo = PARAM_ECHO_RE.match(line)
            if echo:
                try:
                    self._param_lines.append((echo.group(1).upper(), float(echo.group(2))))
                except ValueError:
                    pass
                del self._param_lines[:-64]
                continue
            repeat_stepper = re.search(r"\bS(?:28|35)\s+", line) is not None
            if len(line) >= 6 and any(c.isalpha() for c in line) and (repeat_stepper or line not in self._seen_text):
                self._seen_text.append(line)
                del self._seen_text[:-32]
                self._text_lines.append(line[:FIRMWARE_TEXT_MAX])

    def take_text(self):
        """取出累积的固件文字行（取走后清空）。"""
        lines = self._text_lines
        self._text_lines = []
        return lines

    def take_params(self):
        """取出累积的固件参数回读行 [(名称, 数值), ...]（取走后清空）。"""
        lines = self._param_lines
        self._param_lines = []
        return lines

    def _accept(self, payload):
        values = struct.unpack("<%df" % FRAME_FLOATS, bytes(payload))
        self.frames_ok += 1
        return values

    def feed(self, data):
        """输入任意长度字节流，返回解析出的 float 元组列表。"""
        self.buf += data
        self.bytes_in += len(data)
        self._scan_text(data)
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
    def __init__(self, port, baud, frame_q, line_q, urgent_q=None, err_cb=None,
                 text_q=None, param_q=None):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.frame_q, self.line_q = frame_q, line_q
        self.urgent_q = urgent_q if urgent_q is not None else queue.Queue()
        self.err_cb = err_cb
        # 固件文字应答与上位机提示都走这个队列，界面只做显示。
        self.text_q = text_q
        # 参数回读（"名称=值"行）单独走一个队列，界面用来刷"回读"栏而不刷日志。
        self.param_q = param_q
        self.parser = FrameParser()
        self.stop_flag = False
        self.ser = None
        self.opened = threading.Event()
        self.last_frame_monotonic = 0.0
        self.opened_monotonic = 0.0
        self._safe_stop = False
        self._write_timeouts = 0
        self._tx_resync = False
        self._vofa_last = 0.0
        self._vofa_tries = 0

    def _write_line(self, line):
        if not self.ser or not self.ser.is_open:
            return False
        data = str(line).encode("ascii", "ignore") + b"\n"
        # 超时可能只发出半条命令；先用非法后缀终结残行，禁止重放运动命令。
        if self._tx_resync:
            data = b"!\n" + data
        try:
            written = self.ser.write(data)
            if written != len(data):
                raise serial.SerialTimeoutException("串口短写")
        except serial.SerialTimeoutException:
            self._write_timeouts += 1
            self._tx_resync = True
            self.ser.reset_output_buffer()
            self._note("上位机: 写入超时，命令未确认且不重发：%s" % line)
            if self._write_timeouts >= 3:
                raise serial.SerialException("连续 3 次写入超时，停止连接")
            return False
        self._write_timeouts = 0
        self._tx_resync = False
        return True

    @staticmethod
    def _drain_one(q):
        try:
            return q.get_nowait()
        except queue.Empty:
            return None

    def _note(self, msg):
        """向上位机界面发一条提示（不弹窗、不中断串口）。"""
        if self.text_q is None:
            return
        try:
            self.text_q.put_nowait(msg)
        except queue.Full:
            pass

    def _flush_firmware_text(self):
        # 参数回读行先分流：它们是周期性状态，进日志会把提示刷掉。
        for name, value in self.parser.take_params():
            if self.param_q is None:
                continue
            try:
                self.param_q.put_nowait((name, value))
            except queue.Full:
                pass
        for line in self.parser.take_text():
            self._note("固件文本: " + line)

    def _maybe_resend_vofa(self):
        """遥测中断时自动补发 VOFA。

        固件在 CAN 启动失败或 ZDT 文字调试后处于文字模式，只发文字应答不发
        24 通道遥测；板子在上位机已连接时复位，连接瞬间那次 VOFA 就丢了。
        这里按 VOFA_RETRY_GAP_S 补发，覆盖固件 OPS_Init 约 1.3 秒的启动空窗。
        """
        if self._vofa_tries >= VOFA_MAX_RETRIES:
            return
        now = time.monotonic()
        last = self.last_frame_monotonic or self.opened_monotonic
        if not last or (now - last) < VOFA_RETRY_GAP_S:
            return
        if (now - self._vofa_last) < VOFA_RETRY_GAP_S:
            return
        self._vofa_last = now
        self._vofa_tries += 1
        if not self._write_line("VOFA"):
            return
        self._note("上位机: %.1fs 未收到遥测，已补发 VOFA 恢复波形（第 %d/%d 次）"
                   % (now - last, self._vofa_tries, VOFA_MAX_RETRIES))

    def request_stop(self, safe=True):
        """请求工作线程退出；真实串口断开前 best-effort 主动停车/失能。"""
        # 调用方只提交停止请求；所有串口操作（含停车和close）只在工作线程执行。
        self._safe_stop = self._safe_stop or safe
        self.stop_flag = True
        while True:
            try:
                self.line_q.get_nowait()
            except queue.Empty:
                break

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, bytesize=8,
                                     parity=serial.PARITY_NONE, stopbits=1,
                                     timeout=0.01, write_timeout=0.10)
            self.opened_monotonic = time.monotonic()
            self.opened.set()
        except Exception as e:  # 打开失败，通知 UI
            if self.err_cb:
                self.err_cb("打开 %s 失败：%s" % (self.port, e))
            return
        try:
            # 固件可能停留在ZDT文字模式；连接后只恢复遥测，不触发运动。
            if not self.stop_flag:
                self._write_line("VOFA")
                self._vofa_last = time.monotonic()
            while not self.stop_flag:
                # 急停/失能命令优先于读数据和普通参数命令。
                while not self.stop_flag:
                    line = self._drain_one(self.urgent_q)
                    if line is None:
                        break
                    self._write_line(line)

                # 有数据就立即处理；无数据最多等10ms，避免大块read拖延控制发送。
                data = self.ser.read(min(2048, self.ser.in_waiting) or 1)
                if data:
                    now = time.monotonic()
                    for f in self.parser.feed(data):
                        self.last_frame_monotonic = now
                        self._vofa_tries = 0        # 收到遥测即重置补发预算
                        self._push((now, f))
                    self._flush_firmware_text()

                # 文字模式或板子复位后自动补发 VOFA，避免永久收不到遥测。
                self._maybe_resend_vofa()

                # 读完后再次检查急停，读取引入的额外等待最多约10ms。
                while not self.stop_flag:
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
            if self._safe_stop:
                for cmd in ("STOP", "DMSTOP", "DMOFF"):
                    try:
                        self._write_line(cmd)
                    except Exception:
                        break
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
def param_query(name: str) -> str:
    """参数回读命令：固件回复"<名称>=<值>"文字行。"""
    return "GET " + str(name).strip().upper()


def ops_offset_command(left_mm, forward_mm):
    """安装偏移协议命令：X=左右安装偏移(左+ / 右−)、Y=前后安装偏移(前+ / 后−)，单位mm。
    例如 OPSOFFSET=60.0,-50.0 表示 OPS 装在车左60mm、车后50mm。"""
    values = (float(left_mm), float(forward_mm))
    if any(not math.isfinite(v) or abs(v) > 500 for v in values):
        raise ValueError("安装偏移必须在 -500..500 mm 内")
    return "OPSOFFSET=%.1f,%.1f" % values


def _sim_atomic(method):
    """UI cancellation and simulator integration cannot interleave half an update."""
    @functools.wraps(method)
    def locked(self, *args, **kwargs):
        with self._state_lock:
            return method(self, *args, **kwargs)
    return locked


class NavigationMap(dict):
    """界面地图顶层变更立即作废导航；深拷贝为普通快照，不能复制监听器。"""
    def __init__(self, data, on_change):
        super().__init__(data)
        self.on_change = on_change

    def __setitem__(self, key, value):
        changed = key not in self or self[key] != value
        if changed:
            self.on_change()
        super().__setitem__(key, value)

    def pop(self, key, *default):
        existed = key in self
        if existed:
            self.on_change()
        return super().pop(key, *default)

    def popitem(self):
        if self:
            self.on_change()
        return super().popitem()

    def setdefault(self, key, default=None):
        if key not in self:
            self[key] = default
        return self[key]

    def __ior__(self, other):
        self.update(other)
        return self

    def __delitem__(self, key):
        if key in self:
            self.on_change()
        super().__delitem__(key)

    def update(self, *args, **kwargs):
        updated = dict(self)
        updated.update(*args, **kwargs)
        if updated != self:
            self.on_change()
            super().update(*args, **kwargs)

    def clear(self):
        if self:
            self.on_change()
            super().clear()

    def __deepcopy__(self, memo):
        return copy.deepcopy(dict(self), memo)


class Simulator(threading.Thread):
    def __init__(self, frame_q, line_q, urgent_q=None, param_q=None):
        super().__init__(daemon=True)
        self._state_lock = threading.RLock()
        self.frame_seq = 0
        self.last_frame_monotonic = 0.0
        self._nav_epoch = 0
        self._nav_goal_id = 0
        self._nav_completed_id = 0
        self._nav_active = False
        self._nav_guard = None
        self._nav_fault = ""
        self._nav_speed_mm_s = 250.0  # kinematic preview only, not a motor RPM
        self._nav_yaw_rate_deg_s = 120.0
        self._nav_tracker = None
        self._nav_pose_guard = None
        self._nav_validity = None
        self._nav_mapping = None
        self._nav_reference = None
        self._nav_progress = self._nav_cross_track = 0.0
        self._nav_velocity = (0.0, 0.0)
        self._nav_omega = 0.0
        self._nav_settled = 0
        self._nav_status = 'IDLE'
        self._nav_elapsed = self._nav_last_progress_time = 0.0
        self._nav_best_error = math.inf
        self._nav_last_pose = None
        self.frame_q, self.line_q = frame_q, line_q
        self.urgent_q = urgent_q if urgent_q is not None else queue.Queue()
        # 演示模式的参数回读直接给出数值，不经过串口字节流。
        self.param_q = param_q
        self.stop_flag = False
        # 底盘参数（与固件默认值一致）
        self.kpx, self.kpy, self.kpz = 2.3, 2.3, 9.0
        self.xyvmax, self.zvmax = 1600.0, 750.0
        self.xyvmin, self.zvmin = 5.0, 5.0
        # 视觉跟踪参数（与固件 vision_track.c 的编译期默认值一致）与当前跟踪颜色。
        self.vtrack_conf, self.vtrack_kpmm = 50.0, 0.5
        self.vtrack_dbmm, self.vtrack_dbpx = 2.0, 12.0
        self.vtrack_min, self.vtrack_max = 8.0, 60.0
        self.vtrack_color = 0            # 0=未跟踪；1..6 为颜色编号
        # DM 参数
        self.dm_id, self.dm_mode, self.dm_active = 3, 1, 0
        self.dm_pos = self.dm_vel = self.dm_tor = 0.0
        self.dm_kp, self.dm_kd = 2.0, 0.5
        # 模拟反馈
        self.fb_pos, self.fb_vel, self.fb_tor = 0.0, 0.0, 0.0
        self.fb_status, self.fb_tmos, self.fb_trotor = 0, 35.0, 33.0
        self.zero_x, self.zero_y, self.zero_z = 0.0, 0.0, 0.0
        # GOTO 定位移动模拟（点击场地地图后小车驶向目标）
        self.hold = None        # (X=左, Y=前)，单位 mm；None = 演示巡航
        self.goto_hold = False
        self.goto = None        # (X=左, Y=前, Z=逆时针)，单位 mm/deg
        self.zval = 0.0         # 当前航向（deg）
        self.ops_offset = (60.0, -50.0)  # 统一坐标：车左60mm、车后50mm
        self.ops_reference_yaw = 0.0
        self.manual = None
        self.manual_tick = 0.0
        # 四轮锁轴状态：与固件一致，上电已使能；失能期间拒绝运动命令。
        self.wheel_enabled = True
        self._t = 0.0
        # 仅记录调试目标，不伪造28/35硬件位置或到位反馈。
        self.stepper_commands = {28: None, 35: None}

    @_sim_atomic
    def cancel_navigation(self):
        """Simulator-only cancellation; NO fake STM32 ACK or hardware protocol."""
        # Same lock as queue pop+handle: no already-popped old GOTO can be applied
        # AFTER cancellation. Only simulator queues are touched (never a real port).
        discard_motion_commands(self.line_q)
        discard_motion_commands(self.urgent_q)
        self._nav_epoch += 1
        self._nav_active = False
        self._nav_guard = None
        self._nav_tracker = self._nav_pose_guard = self._nav_validity = None
        self._nav_reference = None
        self._nav_velocity, self._nav_omega = (0.0, 0.0), 0.0
        self._nav_status = 'COMPLETE' if self._nav_status == 'COMPLETE' else 'CANCELLED'
        self.manual = None
        self.goto = None
        self.goto_hold = False
        if self.hold is None:
            self.hold = (600.0 * math.sin(0.25 * self._t),
                         450.0 * math.cos(0.19 * self._t))
        return self._nav_epoch

    @_sim_atomic
    def begin_navigation(self):
        if not self.wheel_enabled:
            raise ValueError("模拟轮已失能")
        epoch = self.cancel_navigation()
        self._nav_active = True
        self._nav_fault = ""
        self._nav_status = 'ACCEPTING'
        return epoch

    def submit_navigation_trajectory(self, epoch, samples, primitives, mapping, scene, validity=None):
        """一次接受整条连续Trajectory；昂贵复检不持运动锁，STOP可立即抢占。"""
        from trajectory_tracking import TrajectoryTracker, LOOKAHEAD_MM
        frozen_samples, frozen_primitives = copy.deepcopy(samples), copy.deepcopy(primitives)
        check = validate_trajectory(frozen_samples, frozen_primitives, scene)
        if not check['ok']:
            raise ValueError('Trajectory复检失败：'+check['reason'])
        tracker = TrajectoryTracker(frozen_samples, frozen_primitives)
        mx, my, angle = (float(value) for value in mapping)
        if not all(math.isfinite(v) for v in (mx, my, angle)):
            raise ValueError('模拟坐标标定非有限值')
        c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        def to_layout(pose):
            fx, fy = mx+c*pose[0]+s*pose[1], my-s*pose[0]+c*pose[1]
            lx, ly = field_to_layout(fx, fy)
            return lx, ly, -90-(90-angle-pose[2])
        def guard(a, b):
            return scene.moving_pose_reason(to_layout(a), to_layout(b))
        with self._state_lock:
            if epoch != self._nav_epoch or not self._nav_active or not self.wheel_enabled:
                raise ValueError('模拟导航会话已失效')
            if validity is not None and not validity():
                raise ValueError('地图版本或模拟会话已变化')
            if self.hold is None or self.goto is not None or self.manual is not None:
                raise ValueError('模拟器已有运动或缺少实际定位')
            x, y = self.hold
            yaw = self._relative_heading()
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError('模拟实际姿态非法')
            first = tracker.reference_at(0.0)
            fx, fy = mx+c*x+s*y, my-s*x+c*y
            if math.dist((fx, fy), (first['x_mm'], first['y_mm'])) > 5:
                raise ValueError('实际起点与Trajectory起点不一致')
            if abs((90-angle-yaw-first['field_yaw_deg']+180) % 360-180) > 5:
                raise ValueError('实际车头与起始切线不一致；请对齐航向后重新规划')
            reason = guard((x, y, yaw), (x, y, yaw))
            if reason:
                raise ValueError('实际车体姿态不安全：'+reason)
            if epoch != self._nav_epoch or not self._nav_active or not self.wheel_enabled:
                raise ValueError('接受检查期间模拟导航会话已失效')
            if validity is not None and not validity():
                raise ValueError('接受检查期间地图版本或模拟会话已变化')
            self._nav_goal_id += 1
            self._nav_tracker, self._nav_pose_guard = tracker, guard
            self._nav_mapping, self._nav_validity = (mx, my, angle), validity
            self._nav_reference = tracker.reference_at(LOOKAHEAD_MM)
            self._nav_progress = self._nav_cross_track = 0.0
            self._nav_velocity, self._nav_omega = (0.0, 0.0), 0.0
            self._nav_elapsed = self._nav_last_progress_time = 0.0
            self._nav_best_error, self._nav_settled = math.inf, 0
            self._nav_best_progress = 0.0
            self._nav_last_pose = (x, y, yaw)
            self._nav_timeout = max(25.0, tracker.length/max(1.0, self._nav_speed_mm_s)*3+5)
            self._nav_status = 'TRACKING'
            return self._nav_goal_id

    def submit_navigation_maneuver(self, epoch, target_layout, target_layout_yaw, mapping, scene, validity=None):
        """比赛停车区/出入库显式麦轮动作：完整平移或原地旋转，不能同时隐式旋转平移。"""
        from competition_simulation import PoseManeuver
        mx, my, angle = map(float, mapping)
        lx, ly = map(float, target_layout)
        target_yaw = float(target_layout_yaw)
        if not all(math.isfinite(v) for v in (mx, my, angle, lx, ly, target_yaw)):
            raise ValueError('比赛动作目标非法')
        c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        def to_layout(pose):
            x, y = field_to_layout(mx+c*pose[0]+s*pose[1], my-s*pose[0]+c*pose[1])
            return x, y, -180+angle+pose[2]
        def guard(a, b):
            return scene.moving_pose_reason(to_layout(a), to_layout(b))
        with self._state_lock:
            if epoch != self._nav_epoch or not self._nav_active or not self.wheel_enabled:
                raise ValueError('比赛动作会话已失效')
            if validity is not None and not validity():
                raise ValueError('比赛地图/会话已变化')
            if self.hold is None or self.manual is not None or self.goto is not None:
                raise ValueError('比赛动作缺少实际定位或已被接管')
            x, y = self.hold
            yaw = self._relative_heading()
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError('实际姿态非法')
            a = to_layout((x, y, yaw))
            moving = math.dist(a[:2], (lx, ly)) > 1
            turning = abs((target_yaw-a[2]+180) % 360-180) > 1e-6
            if moving and turning:
                raise ValueError('须先在安全停车区旋转，再做固定航向平移')
            why = (scene.translation_reason(a[:2], (lx, ly), a[2]) if moving else
                   scene.turn_reason(a[:2], a[2], target_yaw))
            if why:
                raise ValueError('比赛动作连续扫掠不安全：'+why)
            if epoch != self._nav_epoch or not self._nav_active or (validity is not None and not validity()):
                raise ValueError('比赛动作检查期间会话已失效')
            fx, fy = layout_to_field(lx, ly)
            tracker = PoseManeuver((mx+c*x+s*y, my-s*x+c*y, 90-angle-yaw),
                                  dict(x_mm=fx, y_mm=fy, field_yaw_deg=-90-target_yaw))
            self._nav_goal_id += 1
            self._nav_tracker, self._nav_pose_guard = tracker, guard
            self._nav_mapping, self._nav_validity = (mx, my, angle), validity
            self._nav_reference = tracker.reference_at(0)
            self._nav_progress = self._nav_cross_track = 0.0
            self._nav_velocity, self._nav_omega = (0.0, 0.0), 0.0
            self._nav_elapsed = self._nav_last_progress_time = 0.0
            self._nav_best_error, self._nav_settled, self._nav_best_progress = math.inf, 0, 0.0
            self._nav_last_pose = (x, y, yaw)
            self._nav_timeout = max(25.0, tracker.length/max(1, self._nav_speed_mm_s)*3+5)
            self._nav_status = 'MANEUVER'
            return self._nav_goal_id

    def _trajectory_step(self):
        """在50Hz实际状态锁内追踪；先检验，再一次提交位置和航向，故障无残留位移。"""
        tracker = self._nav_tracker
        if tracker is None:
            return
        try:
            epoch = self._nav_epoch
            if not self._nav_active or not self.wheel_enabled:
                raise ValueError('模拟控制权或四轮使能失效')
            if self._nav_validity is not None and not self._nav_validity():
                raise ValueError('地图版本/标定/会话变化')
            if epoch != self._nav_epoch or tracker is not self._nav_tracker:
                return
            if self.hold is None or self.manual is not None or self.goto is not None:
                raise ValueError('实际定位丢失或外部运动接管')
            x, y = self.hold
            yaw = self._relative_heading()
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError('实际位置/航向非有限值')
            if math.dist((x, y), self._nav_last_pose[:2]) > 50:
                raise ValueError('实际定位跳变超过50mm')
            if abs((yaw-self._nav_last_pose[2]+180) % 360-180) > 15:
                raise ValueError('实际航向跳变超过15°')
            dt = 1.0/SEND_HZ
            self._nav_elapsed += dt
            mx, my, angle = self._nav_mapping
            c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
            actual = (mx+c*x+s*y, my-s*x+c*y, 90-angle-yaw)
            ref, (vx, vy), omega = tracker.command(actual, self._nav_speed_mm_s, self._nav_yaw_rate_deg_s)
            if tracker.cross_track > 75:
                raise ValueError('偏离轨迹超过75mm')
            candidate = (x+(c*vx-s*vy)*dt, y+(s*vx+c*vy)*dt, yaw-omega*dt)
            reason = self._nav_pose_guard((x, y, yaw), candidate)
            if reason:
                raise ValueError('实际车体扫掠碰撞：'+reason)
            # 即使保护回调期间发生STOP/版本变化，旧控制步也不能再提交位置或航向。
            if epoch != self._nav_epoch or tracker is not self._nav_tracker or not self._nav_active:
                return
            if self._nav_validity is not None and not self._nav_validity():
                raise ValueError('扫掠检查期间地图/会话变化')
            self.hold = candidate[:2]
            self.zval += candidate[2]-yaw
            self._nav_last_pose = candidate
            self._nav_reference, self._nav_progress = ref, tracker.progress
            self._nav_cross_track = tracker.cross_track
            self._nav_velocity, self._nav_omega = (vx, vy), omega
            final = tracker.reference_at(tracker.length)
            new_field = (mx+c*candidate[0]+s*candidate[1], my-s*candidate[0]+c*candidate[1])
            distance = math.dist(new_field, (final['x_mm'], final['y_mm']))
            yaw_error = abs((90-angle-candidate[2]-final['field_yaw_deg']+180) % 360-180)
            error = distance+yaw_error
            if tracker.progress > getattr(self, '_nav_best_progress', -1)+1 or error < self._nav_best_error-.1:
                self._nav_best_progress, self._nav_best_error = tracker.progress, error
                self._nav_last_progress_time = self._nav_elapsed
            settled = (ref['segment_type'] == 'STOP' and distance < 1 and yaw_error < 1 and
                       math.hypot(vx, vy) <= 1 and abs(omega) <= 1)
            self._nav_settled = self._nav_settled+1 if settled else 0
            self._nav_status = 'FINAL_STOP' if ref['segment_type'] == 'STOP' else 'TRACKING'
            if self._nav_settled >= 10:
                self._nav_completed_id = self._nav_goal_id
                self._nav_active, self._nav_tracker = False, None
                self._nav_status = 'COMPLETE'
            elif self._nav_elapsed-self._nav_last_progress_time > 3:
                raise ValueError('连续跟踪持续3秒无进展')
            elif self._nav_elapsed > self._nav_timeout:
                raise ValueError('连续跟踪总超时')
        except Exception as exc:
            self.cancel_navigation()
            self._nav_fault, self._nav_status = str(exc), 'FAULT'

    @_sim_atomic
    def submit_navigation_goal(self, epoch, target_mm_deg, guard):
        """Atomic local acceptance, correlated by epoch + id; never goes to serial.

        The real firmware does not implement this simulator API. Future hardware
        execution needs its own versioned ACK/DONE protocol and safety review.
        """
        if epoch != self._nav_epoch or not self._nav_active or not self.wheel_enabled:
            raise ValueError("模拟导航会话失效或轮已失能")
        vals = tuple(float(v) for v in target_mm_deg)
        if len(vals) != 3 or not all(math.isfinite(v) for v in vals):
            raise ValueError("模拟目标非有限数值")
        if abs(vals[0]) > 3000 or abs(vals[1]) > 3000 or abs(vals[2]) > 3600:
            raise ValueError("模拟目标超过旧GOTO协议范围，禁止静默钳位")
        if self.hold is None or self.goto is not None or self.manual is not None:
            raise ValueError("模拟器未停稳或已有活动目标")
        if abs((vals[2] - self._relative_heading() + 180) % 360 - 180) > 1e-6:
            raise ValueError("当前导航只支持固定航向，不能隐式转向")
        reason = guard(self.hold, vals[:2])
        if reason:
            raise ValueError("实际起点到航点不安全：" + reason)
        self._nav_goal_id += 1
        self.goto = vals
        self.goto_hold = False
        self._nav_guard = guard
        return self._nav_goal_id

    @_sim_atomic
    def navigation_snapshot(self):
        return {"epoch": self._nav_epoch, "goal_id": self._nav_goal_id,
                "completed_id": self._nav_completed_id, "active": self._nav_active,
                "fault": self._nav_fault, "frame_seq": self.frame_seq,
                "frame_time": self.last_frame_monotonic, "hold": self.hold,
                "yaw": self._relative_heading(), "goto": self.goto,
                "manual": self.manual, "wheel_enabled": self.wheel_enabled,
                "tracking": self._nav_tracker is not None, "tracking_status": self._nav_status,
                "reference": None if self._nav_reference is None else dict(self._nav_reference),
                "progress_s_mm": self._nav_progress, "cross_track_mm": self._nav_cross_track,
                "speed_mm_s": math.hypot(*self._nav_velocity), "yaw_rate_deg_s": self._nav_omega,
                "settled_frames": self._nav_settled, "tracking_elapsed_s": self._nav_elapsed}

    # ---------- 命令解析（镜像 Debug_ParseLine / Debug_SetDmValue） ----------
    @staticmethod
    def _clamp(v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    def _relative_heading(self, zdeg=None):
        """把内部连续航向换算为相对于最近一次 ZERO/OPSOFFSET 的角度。"""
        z = self.zval if zdeg is None else zdeg
        return (z - self.ops_reference_yaw + 180.0) % 360.0 - 180.0

    @_sim_atomic
    def handle_line(self, line):
        line = line.strip().upper()
        if not line:
            return
        cmd = line.split("=", 1)[0]
        if cmd in ("STOP", "ZERO", "OPSOFFSET", "WHEELEN", "WHEELOFF"):
            if self._nav_active:
                self._nav_fault = "外部命令取消导航：" + cmd
            self.cancel_navigation()
        elif cmd in ("MANUAL", "GOTO", "GOTOHOLD") and self._nav_active:
            self._nav_fault = "外部命令取消导航：" + cmd
            self.cancel_navigation()
        # 四轮锁轴/释放：与固件一致，失能相当于停车并取消手动/GOTO。
        # 演示模式不伪造UART4应答，只维护状态并让运动命令被拒绝。
        if line in ("WHEELEN", "WHEELOFF"):
            self.wheel_enabled = line == "WHEELEN"
            self.manual = None
            self.goto = None
            if self.hold is None:                  # 巡航中切换：停在当前位置
                self.hold = (600.0 * math.sin(0.25 * self._t),
                             450.0 * math.cos(0.19 * self._t))
            return
        if line.startswith(("S28", "S35")):
            motor = int(line[1:3])
            if line[3:] == "CANCEL":
                self.stepper_commands[motor] = None
            elif line[3:] == "HOME" or line[3:].startswith(("MOVE=", "RAW=")):
                self.stepper_commands[motor] = line
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
                left, forward = map(float, parts)      # 协议 X=车左, Y=车头
                ops_offset_command(left, forward)
            except (ValueError, OverflowError):
                return
            # OPSOFFSET 与统一坐标完全同序：X=车左、Y=车头。
            self.ops_offset = (left, forward)
            self.manual = self.goto = None
            if self.hold is None:
                self.hold = (600.0 * math.sin(0.25 * self._t), 450.0 * math.cos(0.19 * self._t))
            return
        if line.startswith("MANUAL="):
            if not self.wheel_enabled:             # 与固件一致：失能不运动
                return
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
            self.dm_active = 0
            self.stepper_commands = {28: None, 35: None}
            if self.hold is None:                  # 原地停住（同固件急停）
                self.hold = (600.0 * math.sin(0.25 * self._t),
                             450.0 * math.cos(0.19 * self._t))
            self.goto = None                       # 取消 GOTO 目标（同固件）
            return
        if line == "ZERO":
            self.ops_reference_yaw = self.zval
            self.manual = None
            self.goto = None
            if self.hold is None:                  # 巡航中收到归零：停在当前位置
                self.hold = (600.0 * math.sin(0.25 * self._t),
                             450.0 * math.cos(0.19 * self._t))
            self.hold = (0.0, 0.0)                 # 以当前位置为新原点
            return
        if line.startswith(("GOTO=", "GOTOHOLD=")):
            if not self.wheel_enabled:             # 与固件一致：失能不接受新目标
                return
            try:
                fields = line.split("=", 1)[1].split(",")
                if any(not re.fullmatch(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)", p.strip()) for p in fields):
                    return
                parts = [float(p) for p in fields]
            except ValueError:
                return
            if len(parts) in (2, 3) and all(math.isfinite(p) for p in parts):
                if len(parts) == 3 and not -3600 <= parts[2] <= 3600:
                    return
                self.goto_hold = line.startswith("GOTOHOLD=")
                self.manual = None
                if self.hold is None:              # 从演示巡航位置切入定位模式
                    self.hold = (600.0 * math.sin(0.25 * self._t),
                                 450.0 * math.cos(0.19 * self._t))
                # 协议 X/Y 为 cm；模拟器内部 hold/位置仍用 mm。
                self.goto = (self._clamp(parts[0], -300.0, 300.0) * OPS_CM_TO_MM,
                             self._clamp(parts[1], -300.0, 300.0) * OPS_CM_TO_MM,
                             parts[2] if len(parts) >= 3 else self._relative_heading())
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
        if line.startswith("VTRACK"):
            # 演示模式只记录状态：模拟器没有视觉源，不会因此驱动底盘。
            # 真实固件在此处会经 USART3 请求 Jetson 并驱动闭环，模拟不替代该验证。
            _, _, val = line.partition("=")
            if val.isdigit() and 0 <= int(val) <= 6:
                self.vtrack_color = int(val)
            return
        if line.startswith("GET "):
            # 与固件一致：只回读参数表里的可调参数，名称非法不回任何内容。
            target = line[4:].strip()
            attr = SIM_PARAM_ATTRS.get(target)
            if attr is None or self.param_q is None:
                return
            try:
                self.param_q.put_nowait((target, float(getattr(self, attr))))
            except queue.Full:
                pass
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
        elif name == "VCONF": self.vtrack_conf = self._clamp(v, 0, 100)
        elif name == "VKPMM": self.vtrack_kpmm = self._clamp(v, 0, 5)
        elif name == "VDBMM": self.vtrack_dbmm = self._clamp(v, 0, 100)
        elif name == "VDBPX": self.vtrack_dbpx = self._clamp(v, 0, 200)
        elif name == "VMIN":  self.vtrack_min = self._clamp(v, 0, 60)
        elif name == "VMAX":  self.vtrack_max = self._clamp(v, 0, 3000)
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
    @_sim_atomic
    def make_frame(self, t):
        self._t = t
        self.frame_seq += 1
        self.last_frame_monotonic = time.monotonic()
        self._trajectory_step()
        if self.manual is not None:
            if time.monotonic() - self.manual_tick > 0.350:
                self.manual = None
            else:
                # MANUAL 与 hold 都是统一坐标：X=车左、Y=车头、W=逆时针。
                left_rpm, forward_rpm, wz = self.manual
                angle = math.radians(self._relative_heading())
                px, py = self.hold
                # 演示换算，不代表实车轮径、轮距标定结果。
                self.hold = (px + (left_rpm * math.cos(angle) + forward_rpm * math.sin(angle)) / 0.238 / SEND_HZ,
                             py + (-left_rpm * math.sin(angle) + forward_rpm * math.cos(angle)) / 0.238 / SEND_HZ)
                self.zval += wz / SEND_HZ
        n = lambda a=1.0: random.gauss(0, a)     # noqa: E731
        if self.goto is not None and self.hold is not None:
            # GOTO 定位模式：以 500mm/s 限速驶向目标，航向最短路径逼近。
            # GOTO 与 hold 都是统一坐标：X=车左、Y=车头、Z=逆时针。
            tx, ty, tz = self.goto
            px, py = self.hold
            ddx, ddy = tx - px, ty - py
            dist = math.hypot(ddx, ddy)
            if dist > 1e-9:
                speed = self._nav_speed_mm_s if self._nav_active else 500.0
                step = min(speed / SEND_HZ, dist)
                candidate = (px + ddx / dist * step, py + ddy / dist * step)
                reason = self._nav_guard((px, py), candidate) if self._nav_guard else None
                if reason:
                    self._nav_fault = "轨迹碰撞保护：" + reason
                    self.cancel_navigation()
                else:
                    px, py = candidate
                    self.hold = (px, py)
                dist = math.hypot(tx - px, ty - py)
            dz = (tz - self._relative_heading() + 180.0) % 360.0 - 180.0
            if abs(dz) > 1.0:
                self.zval += self._clamp(dz, -120.0 / SEND_HZ, 120.0 / SEND_HZ)
            else:
                self.zval = self.ops_reference_yaw + tz
                if dist <= 1e-6 and not self.goto_hold:
                    self.goto = None             # 到位，原地保持
                    if self._nav_active:
                        self._nav_completed_id = self._nav_goal_id
                        self._nav_guard = None
            pos_x = px + 3 * n()
            pos_y = py + 3 * n()
            zangle = (self.zval + 0.5 * n() + 180.0) % 360.0 - 180.0
            devx, devy = tx - px, ty - py        # 真实目标误差
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
        if self._nav_reference is not None and self.hold is not None:
            mx, my, angle = self._nav_mapping
            c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
            dx, dy = self._nav_reference['x_mm']-mx, self._nav_reference['y_mm']-my
            devx, devy = c*dx-s*dy-self.hold[0], s*dx+c*dy-self.hold[1]
            devz = (90-angle-self._nav_reference['field_yaw_deg']-self._relative_heading()+180) % 360-180
            spd0 = math.hypot(*self._nav_velocity)
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
            ca, sa = math.cos(angle), math.sin(angle)
            cr, sr = math.cos(ref), math.sin(ref)
            # 配置偏移与实际安装不一致时，旋转会留下安装半径残差。
            # 统一坐标下：pos = 真实中心相对位移 + [R(-yaw)-R(-ref)]*(m_phys-m_cfg)。
            dx = 60.0 - self.ops_offset[0]
            dy = -50.0 - self.ops_offset[1]
            ex = (ca * dx + sa * dy) - (cr * dx + sr * dy)
            ey = (-sa * dx + ca * dy) - (-sr * dx + cr * dy)
            pos_x += cr * ex - sr * ey
            pos_y += sr * ex + cr * ey
        zangle = self._relative_heading(zangle)
        # 与 debug_usart.c 完全同序：ch0=X=车左、ch1=Y=车头、ch3/ch4 误差、
        # ch6/ch7 分别回读 mKpx/mKpy；位置/误差对外为 cm，hold 仍为 mm。
        return (
            pos_x / OPS_CM_TO_MM, pos_y / OPS_CM_TO_MM, zangle,
            devx / OPS_CM_TO_MM, devy / OPS_CM_TO_MM, devz,
            self.kpx, self.kpy, self.kpz, self.xyvmax, self.zvmax, spd0,
            float(self.dm_id), self.fb_pos, self.fb_vel, self.fb_tor,
            float(self.fb_status), self.fb_tmos, self.fb_trotor,
            self.dm_pos, self.dm_vel, self.dm_kp, self.dm_kd, self.dm_tor,
        )

    @_sim_atomic
    def _service_commands(self):
        """Queue consumption and handling share the cancellation lock."""
        for _ in range(64):
            try:
                self.handle_line(self.urgent_q.get_nowait())
            except queue.Empty:
                break
        for _ in range(16):
            if not self.urgent_q.empty():
                break
            try:
                self.handle_line(self.line_q.get_nowait())
            except queue.Empty:
                break

    def run(self):
        t0 = time.monotonic()
        next_t = 0.0
        while not self.stop_flag:
            self._service_commands()
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


def blocked_at(x, y, rects, circles):
    """点是否落在给定障碍集合里；返回障碍名或 None。"""
    for rect in rects:
        if point_in_rect(x, y, rect):
            return rect[4]
    for cx, cy, r, name in circles:
        if math.hypot(x - cx, y - cy) <= r:
            return name
    return None


def seg_blocked(x0, y0, x1, y1, rects, circles):
    """线段是否与给定障碍集合相交；返回障碍名或 None。"""
    for rect in rects:
        if seg_intersects_rect(x0, y0, x1, y1, rect):
            return rect[4]
    for circle in circles:
        if seg_intersects_circle(x0, y0, x1, y1, circle):
            return circle[3]
    return None


def field_point_blocked(x, y):
    return blocked_at(x, y, FIELD_FORBIDDEN_RECTS, FIELD_FORBIDDEN_CIRCLES)


def field_path_blocked(x0, y0, x1, y1):
    return seg_blocked(x0, y0, x1, y1, FIELD_FORBIDDEN_RECTS, FIELD_FORBIDDEN_CIRCLES)


# ======================================================================
# A* 路径规划（仅上位机计算与显示，不下发 STM32）
#   场地离散成网格，禁区按车体尺寸向外膨胀（把车当质点）；8 邻域搜索，
#   代价=实际步长、启发=到目标的欧氏距离（可采纳 ⇒ 结果最优）；
#   出网格路径后用"视线可达"拉直一次，消除 45° 锯齿。
# ======================================================================
GRID_MM = 100.0                 # 默认网格步长
CAR_INFLATE_MM = 150.0          # 旧pad-only兼容模型；不是任意航向下的整车半径
NAV_MARGIN_MM = 10.0            # 固定航向整车矩形之外的额外裕量（演示参数，需实测）
# 「禁止长距离横移」的默认上限：一次连续横移不超过它，超了改用原地转向+直行。
# 0 = 完全禁横移（注意启停区中心离两墙只有 150mm，原地转不动，那样任何目标都规划不出来）；
# None = 不限。横移被前进/后退/转向打断后重新计数，不是全程总量。
STRAFE_RUN_LIMIT_MM = 500.0
CAR_HALF_DIAG_MM = math.hypot(CAR_LENGTH_MM / 2.0, CAR_WIDTH_MM / 2.0)   # ≈191.0


def obstacles_with_pad(pad, rects=None, circles=None):
    """把禁区按 pad 向外膨胀（车体尺寸 → 点模型）。"""
    rects = FIELD_FORBIDDEN_RECTS if rects is None else rects
    circles = FIELD_FORBIDDEN_CIRCLES if circles is None else circles
    return ([(x0 - pad, y0 - pad, x1 + pad, y1 + pad, name) for x0, y0, x1, y1, name in rects],
            [(cx, cy, r + pad, name) for cx, cy, r, name in circles])


def simplify_path(points, rects, circles):
    """兼容接口：连相邻段和两点路径也必须完整无碰撞。"""
    from navigation_planner import simplify_checked
    return simplify_checked(points, lambda a, b: seg_blocked(*a, *b, rects, circles))


def path_length(points):
    from navigation_planner import length
    return length(points)


def path_clearance(points, rects=None, circles=None, sample=50.0):
    """车心到障碍表面的连续最小距离；不含车体、边界。sample仅保留兼容。"""
    from navigation_planner import centre_clearance
    return centre_clearance(points,
                            FIELD_FORBIDDEN_RECTS if rects is None else rects,
                            FIELD_FORBIDDEN_CIRCLES if circles is None else circles)


# ---------------- 模拟障碍（φ50×100mm 圆柱，顶视 r=25mm 圆） ----------------
# 与 navigation_map.json 里的固定障碍同格式（circles: x, y, r, name），因此能直接并入
# 规划场景；但它们**不写进地图数据**——地图是实测场地，模拟障碍只是演示用的临时物体。
SIM_OBSTACLE_R_MM = 25.0        # φ50mm
SIM_OBSTACLE_H_MM = 100.0       # 高度（规划是二维的，这里只作注释）
SIM_OBSTACLE_MAX = 4            # 数量上限
SIM_OBSTACLE_NAME = "模拟障碍"


def sim_obstacle_circles(points, radius=SIM_OBSTACLE_R_MM, name=SIM_OBSTACLE_NAME):
    """模拟障碍中心点列 → circles 表（与地图 circles 同格式）。"""
    return [(float(x), float(y), float(radius), name) for x, y in points]


def obstacle_placement_blocked(x, y, radius=SIM_OBSTACLE_R_MM, rects=None, circles=None,
                               bounds=None, margin=20.0):
    """放置检查：障碍圆不与固定禁区/已有障碍/场地边缘重叠；返回冲突名或 None。

    判据是"把既有障碍按 radius+margin 膨胀，再看圆心是否落在其中"：对圆障碍精确，
    对矩形保守（矩形按 x/y 各扩 pad，略大于真正的圆角外扩）。
    """
    rects = FIELD_FORBIDDEN_RECTS if rects is None else rects
    circles = FIELD_FORBIDDEN_CIRCLES if circles is None else circles
    pad = max(0.0, float(radius)) + max(0.0, float(margin))
    hit = blocked_at(x, y, *obstacles_with_pad(pad, rects, circles))
    if hit:
        return hit
    x0, y0, x1, y1 = bounds if bounds else (0.0, 0.0, FIELD_SIZE, FIELD_SIZE)
    if not (x0 + pad <= x <= x1 - pad and y0 + pad <= y <= y1 - pad):
        return "场地边缘"
    return None


def random_obstacle_points(count, *, rects=None, circles=None, bounds=None,
                           radius=SIM_OBSTACLE_R_MM, keep_clear=(), margin=20.0,
                           tries=600, rng=None):
    """随机撒障碍中心点：避开固定禁区/已有障碍/场地边缘/keep_clear 附近。

    keep_clear 是 (x, y, clear_mm) 列表，通常放"车当前位置 + 半个车对角线 + 半径"，
    避免障碍直接压在车上——那样之后每次规划都会以"起点：模拟障碍"失败。
    返回 (points, reason)：放不满 count 时按实际数量返回并说明原因，绝不返回非法点。
    """
    rects = FIELD_FORBIDDEN_RECTS if rects is None else rects
    circles = FIELD_FORBIDDEN_CIRCLES if circles is None else circles
    rng = random if rng is None else rng
    x0, y0, x1, y1 = bounds if bounds else (0.0, 0.0, FIELD_SIZE, FIELD_SIZE)
    chosen = []
    for _ in range(max(0, int(count))):
        placed = None
        for _try in range(max(1, int(tries))):
            x = rng.uniform(x0, x1)
            y = rng.uniform(y0, y1)
            if obstacle_placement_blocked(x, y, radius, rects,
                                          list(circles) + sim_obstacle_circles(chosen, radius),
                                          bounds, margin):
                continue
            if any(math.hypot(x - kx, y - ky) < clear for kx, ky, clear in keep_clear):
                continue
            placed = (x, y)
            break
        if placed is None:
            return chosen, ("随机尝试 %d 次仍找不到合法位置，只放置了 %d/%d 个"
                            % (int(tries), len(chosen), int(count)))
        chosen.append(placed)
    return chosen, ""


def approach_target(anchor, outward, *, margin=NAV_MARGIN_MM, footprint=None,
                    rects=None, circles=None, bounds=None, drivable_polygons=None,
                    standoff=APPROACH_STANDOFF_MM, max_mm=2000.0):
    """功能区锚点 → 按当前航向/裕量算出的合法接近点（布局 mm）。

    只是 navigation_planner.approach_point 的默认参数包装，方便上位机/测试直接调用；
    返回体见该函数（ok=False 时绝不返回未经验证的点）。
    """
    from navigation_planner import approach_point
    return approach_point(anchor, outward, margin=margin,
                          rects=FIELD_FORBIDDEN_RECTS if rects is None else rects,
                          circles=FIELD_FORBIDDEN_CIRCLES if circles is None else circles,
                          bounds=(0.0, 0.0, FIELD_SIZE, FIELD_SIZE) if bounds is None else bounds,
                          footprint=footprint, drivable_polygons=drivable_polygons,
                          standoff=standoff, max_mm=max_mm)


def plan_path(start, goal, grid=GRID_MM, pad=CAR_INFLATE_MM,
              rects=None, circles=None, bounds=None, *, footprint=None,
              drivable_polygons=None, geometry_verified=False, cancel=None,
              time_limit_s=8.0, start_heading_deg=0.0, goal_heading_deg=None,
              allow_strafe=True, strafe_polygons=None, strafe_run_limit_mm=None,
              cost_forward=None,
              cost_backward=None, cost_lateral=None, turn_penalty_mm=None, smooth_arcs=True,
              sim_rects=None, sim_circles=None, dynamic_rects=None, dynamic_circles=None):
    """布局mm坐标规划。ok只代表指定几何模型通过，不是实车通行许可。

    footprint=None：兼容pad膨胀模型，只能参考，execution_safe=False。
    footprint=(长mm,宽mm,布局航向deg)：真实整车矩形，pad为额外安全裕量。
    start_heading_deg / goal_heading_deg：状态含航向；goal 传 None 表示不约束终点航向。
    allow_strafe=False 完全禁止普通道路横移；strafe_run_limit_mm=N 只禁**长距离**横移
    （一次连续横移不超过 N mm，0 = 完全禁横移，None = 不限）；strafe_polygons 是
    允许横移的操作区多边形，区内不受上面两条限制。
    cost_forward/cost_backward/cost_lateral/turn_penalty_mm 为动作代价（None 用默认值）。
    drivable_polygons必须来自核对过的地图；None只是全矩形演示场地。
    结果里的 segments/corners 是共线压缩后的 LineSegment/Corner（geometry_axes 可单独调用）。
    smooth_arcs默认启用；arcs是安全90°圆弧，smoothed_primitives/points是完整平滑轨迹；
    arc_fallbacks/arc_attempts返回保留拐角与逐半径失败原因，原points/steps不变。
    sim_rects/circles、dynamic_rects/circles与固定障碍共同参与整车碰撞检查。
    trajectory为场地mm坐标的20mm连续采样，field_yaw_deg使用unwrap后的切线方向；
    trajectory_safe只在最终完整矩形扫掠检查通过时为True，原地停转fallback不伪装成连续轨迹。
    本版本任何结果的hardware_ready都为False，不提供实车路径执行。
    """
    from navigation_planner import plan
    return plan(start, goal, grid=grid, pad=pad,
                rects=FIELD_FORBIDDEN_RECTS if rects is None else rects,
                circles=FIELD_FORBIDDEN_CIRCLES if circles is None else circles,
                bounds=(0.0, 0.0, FIELD_SIZE, FIELD_SIZE) if bounds is None else bounds,
                footprint=footprint, drivable_polygons=drivable_polygons,
                geometry_verified=geometry_verified, cancel=cancel,
                time_limit_s=time_limit_s,
                start_heading_deg=start_heading_deg, goal_heading_deg=goal_heading_deg,
                allow_strafe=allow_strafe, strafe_polygons=strafe_polygons,
                strafe_run_limit_mm=strafe_run_limit_mm,
                cost_forward=cost_forward, cost_backward=cost_backward,
                cost_lateral=cost_lateral, turn_penalty_mm=turn_penalty_mm,
                smooth_arcs=smooth_arcs, sim_rects=sim_rects, sim_circles=sim_circles,
                dynamic_rects=dynamic_rects, dynamic_circles=dynamic_circles)


def geometry_axes(points, steps=None):
    """Manhattan 折线 → (LineSegment, Corner)（供界面/测试直接调用，无 Qt 依赖）。"""
    from navigation_planner import path_axes
    return path_axes(points, steps)


def generate_trajectory(primitives, scene, **kwargs):
    """LineSegment/ArcSegment或LINE/ARC几何→场地mm连续Trajectory（纯预览，无运动输出）。"""
    from trajectory import generate_trajectory as generate
    return generate(primitives, scene, **kwargs)


def export_trajectory_json(path, result, *, metadata=None):
    """导出经过完整整车复检的Trajectory JSON。"""
    from trajectory import export_trajectory_json as export
    return export(path, result, metadata=metadata)


def validate_trajectory(samples, primitives, scene, **kwargs):
    """按最新障碍快照复核切线Trajectory，包括所有姿态和采样间整车扫掠。"""
    from trajectory import validate_trajectory as validate
    return validate(samples, primitives, scene, **kwargs)


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

    print('[3] Simulator 参数/DM/视觉 ...')
    sim = Simulator(queue.Queue(), queue.Queue(), queue.Queue())
    sim.handle_line('KPX=999')
    sim.handle_line('DMEN')
    assert sim.kpx == 50.0 and sim.dm_active == 1
    # 视觉参数必须能被设置/限幅并支持 GET 回读，否则 --simulate 下 6 个回读栏恒为"—"。
    sim.handle_line('VDBMM=5')
    sim.handle_line('VMAX=99999')
    assert sim.vtrack_dbmm == 5.0 and sim.vtrack_max == 3000.0
    sim.handle_line('VTRACK=4')
    assert sim.vtrack_color == 4
    assert SIM_PARAM_ATTRS['VDBMM'] == 'vtrack_dbmm' and vtrack_command(1) == 'VTRACK=1'
    print('    ok')

    print('[4] 场地禁区 ...')
    assert field_point_blocked(600, 600) == '中央物料区'
    assert field_point_blocked(1200, 2200) is None
    assert field_path_blocked(2250, 2250, 330, 1200) is not None
    assert field_path_blocked(2250, 2250, 1200, 2200) is None
    print('    ok')

    print('[5] 四轮使能/失能闸门 ...')
    sim = Simulator(queue.Queue(), queue.Queue(), queue.Queue())
    sim.handle_line('WHEELOFF')
    sim.handle_line('MANUAL=60,0,0')
    sim.handle_line('GOTO=10.0,10.0,0.0')
    assert sim.wheel_enabled is False and sim.manual is None and sim.goto is None
    sim.handle_line('WHEELEN')
    sim.handle_line('MANUAL=60,0,0')
    assert sim.wheel_enabled is True and sim.manual == (60, 0, 0)
    print('    ok')

    print('[6] A* 路径规划 ...')
    straight = plan_path((2250.0, 2250.0), (1200.0, 2250.0), pad=0.0)
    assert straight['ok'] and len(straight['points']) == 2
    assert abs(straight['length'] - 1050.0) < 1e-6
    detour = plan_path((2250.0, 2250.0), (330.0, 1200.0))
    assert detour['ok'] and len(detour['points']) > 2
    assert detour['min_clearance'] > 0.0
    pad_rects, pad_circles = obstacles_with_pad(detour['pad'])
    for k in range(len(detour['points']) - 1):
        assert seg_blocked(detour['points'][k][0], detour['points'][k][1],
                           detour['points'][k + 1][0], detour['points'][k + 1][1],
                           pad_rects, pad_circles) is None
    assert plan_path((2250.0, 2250.0), (1200.0, 330.0), pad=400.0)['ok'] is False
    print('    ok')
    print('全部核心自检通过 ✔')
