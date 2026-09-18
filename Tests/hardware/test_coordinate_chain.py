"""统一坐标链路护栏。

工程统一约定（2026-09-17）：
    +X = 车左，-X = 车右
    +Y = 车头，-Y = 车尾
    +Z = 逆时针

固件内部、串口协议、遥测、麦轮控制均直接使用该轴序和符号，不再建立
“内部前后/左右”与“外部 X/Y”之间的交换或取反层。
"""
from pathlib import Path


root = Path(__file__).resolve().parents[2]
ops = (root / "Hardware/ops.c").read_text(encoding="utf-8")
mec = (root / "Hardware/mecanum_control.c").read_text(encoding="utf-8")
debug = (root / "Hardware/debug_usart.c").read_text(encoding="utf-8")
debug_h = (root / "Hardware/debug_usart.h").read_text(encoding="utf-8")


def flat(text):
    """折叠空白，使文本断言不受缩进/对齐影响。"""
    return " ".join(text.split())


d, m, o, dh = flat(debug), flat(mec), flat(ops), flat(debug_h)


def must(text, needle, where):
    assert needle in text, "%s 缺少预期实现：%s" % (where, needle)


def must_not(text, needle, where):
    assert needle not in text, "%s 出现了禁止的旧坐标逻辑：%s" % (where, needle)


# ---------------- 1) 遥测直接输出统一坐标，无适配层 ----------------
must(d, "user_x = pos_x;", "遥测 X 直接取 pos_x")
must(d, "user_y = pos_y;", "遥测 Y 直接取 pos_y")
must(d, "err_x = devx;", "遥测 X 误差直接取 devx")
must(d, "err_y = devy;", "遥测 Y 误差直接取 devy")
must(d, "data[0] = user_x * 0.1f;", "遥测 ch0=X左右（mm→cm）")
must(d, "data[1] = user_y * 0.1f;", "遥测 ch1=Y前后（mm→cm）")
must(d, "data[2] = zangle;", "遥测 ch2=Z航向")
must(d, "data[3] = err_x * 0.1f;", "遥测 ch3=X误差（mm→cm）")
must(d, "data[4] = err_y * 0.1f;", "遥测 ch4=Y误差（mm→cm）")
must(d, "data[5] = devz;", "遥测 ch5")
must(d, "data[6] = mKpx;", "遥测 ch6=X左右增益")
must(d, "data[7] = mKpy;", "遥测 ch7=Y前后增益")
must(d, "data[8] = mKpz;", "遥测 ch8")

for old in (
    "Debug_UserToInternal",
    "Debug_InternalToUser",
    "data[0] = pos_y",
    "data[1] = -pos_x",
    "data[0] = -pos_y",
    "data[1] = pos_x",
    "data[3] = devy",
    "data[4] = -devx",
):
    must_not(d, old, "遥测与坐标轴")

# ---------------- 2) 三个协议命令直接使用 X/Y ----------------
must(d, "MecanumControl_MoveVelocity((float)v[0], (float)v[1], (float)v[2]);",
     "MANUAL 原序直接下发")
must(d, "s_goto_x = v[0];", "GOTO X=左右")
must(d, "s_goto_y = v[1];", "GOTO Y=前后")
must(d, "s_offset_x = v[0]; s_offset_y = v[1];", "OPSOFFSET X/Y")
must(d, "if (v[0] < -300.0f)", "GOTO X协议范围=±300.0cm")
must(d, "if (v[1] < -300.0f)", "GOTO Y协议范围=±300.0cm")
must_not(d, "if (v[0] < -3000.0f)", "GOTO不得按mm解析")
must_not(d, "if (v[1] < -3000.0f)", "GOTO不得按mm解析")

for old in (
    "MoveVelocity(-v[1], v[0], v[2])",
    "MoveVelocity(-v[1], -v[0], v[2])",
    "MoveVelocity(v[1], v[0], v[2])",
    "s_goto_x = -v[1]",
    "s_goto_y = -v[0]",
    "s_offset_x = -v[1]",
    "s_offset_y = -v[0]",
):
    must_not(d, old, "命令解析")

# ---------------- 3) 参数表与变量同义 ----------------
must(d, '{"KPX", &mKpx, 0.0f, 50.0f},', "参数表 KPX -> X/左右")
must(d, '{"KPY", &mKpy, 0.0f, 50.0f},', "参数表 KPY -> Y/前后")
must_not(d, '{"KPX", &mKpy,', "禁止 KPX/KPY 交叉映射")
must_not(d, '{"KPY", &mKpx,', "禁止 KPX/KPY 交叉映射")

# ---------------- 4) OPS 原始帧到统一坐标的唯一固定映射 ----------------
# X=-raw_y、Y=-raw_x；所有映射调用必须共用这组固定公式。
must(o, "*x = -raw_y;", "原始帧 → 统一 X")
must(o, "*y = -raw_x;", "原始帧 → 统一 Y")
must(o, "*raw_x = -y;", "统一 X → 原始帧 x")
must(o, "*raw_y = -x;", "统一 Y → 原始帧 y")
must_not(o, "OPS_RAW_AXIS_MODE", "固定映射不得保留方向模式")
must_not(o, "case 0U:", "固定映射不得保留模式分支")
must_not(o, "default:", "固定映射不得保留模式默认分支")
must(o, "OPS_MapUnifiedToRaw(mount_x, mount_y, &rx, &ry);", "安装偏移统一→原始")
must(o, "OPS_MapRawToUnified(px, py, x, y);", "绝对位置原始→统一")
must(o, "float dx = dc * rx - ds * ry;", "偏心矩阵 x")
must(o, "float dy = ds * rx + dc * ry;", "偏心矩阵 y")
must(o, "OPS_MapRawToUnified(raw_cx, raw_cy, x, y);", "补偿后原始→统一")
must(o, "OPS_MapUnifiedToRaw(x, y, &raw_x, &raw_y);", "SetOrigin 统一→原始")
must(o, "*z = zero ? (yaw - zero_yaw) : yaw;", "ZERO 同时归零航向")
must(o, "s_mount_x_mm = 60.0f;", "默认安装 X=车左60mm")
must(o, "s_mount_y_mm = -50.0f;", "默认安装 Y=车后50mm")
must(o, "s_mount_x_mm = x_mm;", "SetMountOffset X")
must(o, "s_mount_y_mm = y_mm;", "SetMountOffset Y")
must_not(o, "rx = -s_mount_x_mm * 0.001f;", "安装偏移不得绕过轴模式")
must_not(o, "ry = s_mount_y_mm * 0.001f;", "安装偏移不得绕过轴模式")
must_not(o, "s_ops.origin_x = -x;", "SetOrigin 不得绕过轴模式")
must_not(o, "s_ops.origin_y = y;", "SetOrigin 不得绕过轴模式")

# 位置环误差与 ops.c 坐标方向必须成对，否则是正反馈。
must(m, "devx = (float)x - pos_x;", "chassis_move X 误差")
must(m, "devy = (float)y - pos_y;", "chassis_move Y 误差")
must_not(m, "devx = pos_x - (float)x;", "chassis_move X 误差反号")
must_not(m, "devy = pos_y - (float)y;", "chassis_move Y 误差反号")
must(m, "MecanumControl_UpdatePose();", "chassis_move 刷新位姿")

# ---------------- 5) 麦轮公式与输出层 ----------------
must(m, "speed[0] = (int)( y_forward - x_left - z_ccw);", "左前轮")
must(m, "speed[1] = (int)(-y_forward - x_left - z_ccw);", "右前轮")
must(m, "speed[2] = (int)( y_forward + x_left - z_ccw);", "左后轮")
must(m, "speed[3] = (int)(-y_forward + x_left - z_ccw);", "右后轮")
must(m, "MecanumControl_CalcWheelSpeed(vxRpm, vyRpm, vzRpm, wheel);",
     "MANUAL 走统一混控")
must_not(m, "s_motor_dir_invert", "移动控制层不得再做 2/4 轮极性补偿")
must_not(m, "wheel[0] = (int32_t)-(vxRpm + vyRpm + vzRpm);", "旧混控公式")
must_not(m, "speed[1] = (int) (vy1 - vy2 - vx1 - vx2 - vz);", "旧轴通道公式")

# ---------------- 6) 帧格式和既有命令不受影响 ----------------
must(dh, "#define DEBUG_VOFA_CHANNELS 24U", "通道数")
for tail in ("DEBUG_VOFA_TAIL0 0x00U", "DEBUG_VOFA_TAIL1 0x00U",
             "DEBUG_VOFA_TAIL2 0x80U", "DEBUG_VOFA_TAIL3 0x7FU"):
    must(d, tail, "帧尾")
must(d, "data[23] = s_dm_torque;", "24通道末位")
must_not(d, "data[24]", "通道数越界")
must(d, "HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx))", "USART1 DMA空闲接收")
for cmd in ('"STOP"', '"PING"', '"ZERO"', '"WHEELEN"', '"WHEELOFF"', '"VOFA"'):
    must(d, "Debug_StrCaseCmp(line, %s)" % cmd, "既有命令" + cmd)

for name, text in (("debug_usart.c", d), ("mecanum_control.c", m), ("ops.c", o)):
    must_not(text, "Debug_UserToInternal", name)
    must_not(text, "Debug_InternalToUser", name)

print("Unified coordinate guards passed:")
print("  GUI/protocol/internal/telemetry use X=left, Y=front, Z=CCW directly")
print("  OPS raw axes use fixed mapping X=-raw_y, Y=-raw_x")
print("  mecanum mixer, 24-channel JustFloat frame and existing commands unchanged")
