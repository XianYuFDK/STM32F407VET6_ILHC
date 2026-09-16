"""坐标链路护栏：核对固件只在边界层做一次 X/Y 交换，且底层解算与帧格式未被改动。

对外统一约定：+X=小车左方、+Y=小车正前方、+Z=逆时针为正；X 为左右轴、Y 为前后轴。
底盘内部沿用 pos_x=前后、pos_y=左右、zangle=航向角，交换只允许出现在下列边界：
  1) 24 通道遥测打包：ch0/ch1、ch3/ch4、ch6/ch7
  2) MANUAL / GOTO / OPSOFFSET 入参
  3) KPX/KPY 参数表指针
  4) ops.c 的置零符号方向 + chassis_move 的误差定义（两者必须成对，否则正反馈）
本文件用源码文本锁定这些映射点。Debug_ParseLine 依赖过多（同 test_debug_wheel.py
的说明），因此沿用该文件的文本核对方式，不编译整条解析链。
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
    assert needle not in text, "%s 不应再出现（会导致双重交换/符号冲突）：%s" % (where, needle)


# ---------------- 1) 24 通道遥测：交换 + 仅前后轴取反 ----------------
# 实车实测内部 pos_x 指向车尾、pos_y 指向车左（按 W 才是车头、按 A 才是车左），
# 因此 ch0/ch1、ch3/ch4 交换，且**只有前后轴取反**；ch6/ch7 只是参数回读，交换即可。
must(d, "data[0] = pos_y;", "遥测ch0")
must(d, "data[1] = -pos_x;", "遥测ch1")
must(d, "data[3] = devy;", "遥测ch3")
must(d, "data[4] = -devx;", "遥测ch4")
must(d, "data[6] = mKpy;", "遥测ch6")
must(d, "data[7] = mKpx;", "遥测ch7")
must(d, "data[2] = zangle;", "遥测ch2")
must(d, "data[5] = devz;", "遥测ch5")
must(d, "data[8] = mKpz;", "遥测ch8")
for old in ("data[0] = pos_x;", "data[1] = pos_y;", "data[0] = -pos_y;",
            "data[3] = devx;", "data[4] = devy;", "data[3] = -devy;",
            "data[6] = mKpx;", "data[7] = mKpy;"):
    must_not(d, old, "遥测打包")

# ---------------- 2) 三个协议入参：交换 + 仅前后轴取反 ----------------
must(d, "MecanumControl_MoveVelocity(-v[1], v[0], v[2]);", "MANUAL")
must_not(d, "MecanumControl_MoveVelocity(-v[1], -v[0], v[2])", "MANUAL")
must_not(d, "MecanumControl_MoveVelocity(v[1], v[0], v[2])", "MANUAL")
must_not(d, "MecanumControl_MoveVelocity(v[0], v[1], v[2])", "MANUAL")
must(d, "s_goto_x = -v[1]; s_goto_y = v[0];", "GOTO")
must_not(d, "s_goto_x = -v[1]; s_goto_y = -v[0];", "GOTO")
must_not(d, "s_goto_x = v[1]; s_goto_y = v[0];", "GOTO")
must_not(d, "s_goto_x = v[0]; s_goto_y = v[1];", "GOTO")
must(d, "s_offset_x = -v[1]; s_offset_y = v[0];", "OPSOFFSET")
must_not(d, "s_offset_x = -v[1]; s_offset_y = -v[0];", "OPSOFFSET")
must_not(d, "s_offset_x = v[1]; s_offset_y = v[0];", "OPSOFFSET")
must_not(d, "s_offset_x = v[0]; s_offset_y = v[1];", "OPSOFFSET")
# 只在前后轴取反：整体不能出现 -v[0]（左右轴必须保持与内部同向，否则 A/D 会反）
assert "-v[0]" not in d, "左右轴不得取反（实车：A/D 反了就是这里多加了负号）"
assert d.count("-v[1]") >= 3, "MANUAL/GOTO/OPSOFFSET 三个命令都要交换并取反前后轴"
# 三个命令各自的解析长度/分派不得改动
must(d, 'Debug_StrCaseCmpN(line, "MANUAL=", 7U)', "MANUAL分派")
must(d, 'Debug_StrCaseCmpN(line, "GOTO", 4U)', "GOTO分派")
must(d, 'Debug_StrCaseCmpN(line, "OPSOFFSET=", 10U)', "OPSOFFSET分派")
# MANUAL 每轴范围仍是 ±300
must(d, "if (value > 300) return 0U;", "MANUAL范围")

# ---------------- 3) KPX=左右轴增益、KPY=前后轴增益 ----------------
must(d, '{"KPX", &mKpy, 0.0f, 50.0f},', "参数表KPX")
must(d, '{"KPY", &mKpx, 0.0f, 50.0f},', "参数表KPY")
must_not(d, '{"KPX", &mKpx,', "参数表KPX")
must_not(d, '{"KPY", &mKpy,', "参数表KPY")
for name in ('{"KPZ", &mKpz, 0.0f, 50.0f},', '{"XVMAX", &XYVmax, 0.0f, 3000.0f},',
             '{"ZVMAX", &ZVmax, 0.0f, 3000.0f},', '{"XVMIN", &XYVmin, 0.0f, 100.0f},',
             '{"ZVMIN", &ZVmin, 0.0f, 100.0f},'):
    must(d, name, "参数表其余项")

# ---------------- 4) 置零符号与误差定义必须成对出现 ----------------
must(o, "*x = px - ox - dx;", "ops.c置零分支")
must(o, "*y = py - oy - dy;", "ops.c置零分支")
must_not(o, "*x = ox - px + dx;", "ops.c置零分支")
must_not(o, "*y = oy - py + dy;", "ops.c置零分支")
must(o, "*x = px - dx;", "ops.c未置零分支")
must(o, "*y = py - dy;", "ops.c未置零分支")
must(o, "s_mount_x_mm = -50.0f;", "安装偏差异常默认值")
must(o, "s_mount_y_mm = 60.0f;", "安装偏差异常默认值")
must(o, "s_mount_x_mm = x_mm;", "SetMountOffset形参顺序")
must(o, "s_mount_y_mm = y_mm;", "SetMountOffset形参顺序")
must(m, "devx = (float)x - pos_x;", "chassis_move误差")
must(m, "devy = (float)y - pos_y;", "chassis_move误差")
must_not(m, "devx = pos_x - (float)x;", "chassis_move误差")
must_not(m, "devy = pos_y - (float)y;", "chassis_move误差")
# 未置零与置零两种状态下符号必须一致（同号），不允许只改其中一条分支
must(m, "MecanumControl_UpdatePose();", "chassis_move刷新位姿")

# ---------------- 5) 两条麦轮公式必须同约定（前后/左右不得再互换） ----------------
for line in ("speed[0] = (int)-(vy1 - vy2 + vx1 + vx2 + vz);",
             "speed[1] = (int) (vx1 + vx2 - vy1 + vy2 - vz);",
             "speed[2] = (int)-(vx1 + vx2 - vy1 + vy2 + vz);",
             "speed[3] = (int) (vy1 - vy2 + vx1 + vx2 - vz);",
             "wheel[0] = (int32_t)-(vxRpm + vyRpm + vzRpm);",
             "wheel[1] = (int32_t) (vxRpm - vyRpm - vzRpm);",
             "wheel[2] = (int32_t)-(vxRpm - vyRpm + vzRpm);",
             "wheel[3] = (int32_t) (vxRpm + vyRpm - vzRpm);"):
    must(m, line, "麦轮解算公式")
# 2026-09-16 实车修正：原 chassis_move 把车体前后(vx1+vx2)与左右(vy1-vy2)送进了相反的槽位，
# 表现为 MANUAL 方向正常而 GOTO=0,1000,0 横移。以下两条是修正前的错误写法，不得回归。
must_not(m, "speed[1] = (int) (vy1 - vy2 - vx1 - vx2 - vz);", "chassis_move轴通道")
must_not(m, "speed[2] = (int)-(vy1 - vy2 - vx1 - vx2 + vz);", "chassis_move轴通道")
assert "chassis_move" in m and "MecanumControl_MoveVelocity" in m

# ---------------- 6) 帧格式与其余功能不得受影响 ----------------
must(dh, "#define DEBUG_VOFA_CHANNELS 24U", "通道数")
must(d, "#define DEBUG_VOFA_TAIL0 0x00U", "帧尾")
must(d, "#define DEBUG_VOFA_TAIL1 0x00U", "帧尾")
must(d, "#define DEBUG_VOFA_TAIL2 0x80U", "帧尾")
must(d, "#define DEBUG_VOFA_TAIL3 0x7FU", "帧尾")
must(d, "data[23] = s_dm_torque;", "24通道末位")
must_not(d, "data[24]", "通道数越界")
must(d, "HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx))", "USART1 DMA空闲接收")
for cmd in ('"STOP"', '"PING"', '"ZERO"', '"WHEELEN"', '"WHEELOFF"', '"VOFA"'):
    must(d, "Debug_StrCaseCmp(line, %s)" % cmd, "既有命令" + cmd)

print("Coordinate chain guards passed:")
print("  telemetry ch0/ch1, ch3/ch4, ch6/ch7 swapped exactly once")
print("  MANUAL/GOTO/OPSOFFSET each swapped exactly once; KPX=left-right, KPY=forward-back")
print("  ops.c zero branch is physical-positive; chassis_move error is target-current")
print("  mecanum formulas, 24-channel JustFloat frame and existing commands unchanged")
