"""编译真实四轮使能/失能服务函数，验证切换顺序、闸门与"失能后不再发速度帧"。"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "Hardware/debug_usart.c").read_text(encoding="utf-8")
mecanum_h = (root / "Hardware/mecanum_control.h").read_text(encoding="utf-8")
mecanum_c = (root / "Hardware/mecanum_control.c").read_text(encoding="utf-8")


def function(name):
    start = source.rfind("static ", 0, source.index(name + "("))
    brace = source.index("{", start)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


# Debug_ServiceWheel 不调用HAL_GetTick，因此不提供该桩，避免-Wunused-function。
wheel_prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
static volatile uint8_t s_wheel_req;
static uint8_t s_wheel_enabled = 1U;
static volatile uint8_t s_manual_active;
static volatile uint8_t s_goto_active;
static uint32_t mask, calls, order[8], order_len;
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask = 1U;}
static void __enable_irq(void) {mask = 0U;}
static void log_call(uint32_t id) {if (order_len < 8U) order[order_len++] = id;}
static void MecanumControl_Stop(void) {calls++; log_call(1U);}
static void MecanumControl_Enable(void) {calls++; log_call(2U);}
static void MecanumControl_Disable(void) {calls++; log_call(3U);}
'''

wheel_checks = r'''
int main(void) {
 /* 无请求：不触碰UART4，也不改动运动状态 */
 s_manual_active = 1U; s_goto_active = 1U; calls = 0U; order_len = 0U;
 Debug_ServiceWheel();
 assert(calls == 0U && s_manual_active == 1U && s_goto_active == 1U);
 assert(Debug_WheelReady() == 1U);

 /* 使能请求：先停车再使能，闸门在使能完成后才打开，保留PRIMASK */
 s_wheel_req = 1U; calls = 0U; order_len = 0U;
 s_wheel_enabled = 0U; s_manual_active = 1U; s_goto_active = 1U; mask = 1U;
 Debug_ServiceWheel();
 assert(s_wheel_req == 0U);
 assert(calls == 2U && order[0] == 1U && order[1] == 2U);
 assert(s_manual_active == 0U && s_goto_active == 0U);
 assert(s_wheel_enabled == 1U && Debug_WheelReady() == 1U);
 assert(mask == 1U);

 /* 失能请求：先停车再失能，闸门关闭，恢复调用前的中断状态 */
 s_wheel_req = 2U; calls = 0U; order_len = 0U;
 s_manual_active = 1U; s_goto_active = 1U; mask = 0U;
 Debug_ServiceWheel();
 assert(s_wheel_req == 0U);
 assert(calls == 2U && order[0] == 1U && order[1] == 3U);
 assert(s_wheel_enabled == 0U && Debug_WheelReady() == 0U);
 assert(s_manual_active == 0U && s_goto_active == 0U);
 assert(mask == 0U);

 /* 已失能时重复请求不产生UART4发送 */
 calls = 0U;
 Debug_ServiceWheel();
 assert(calls == 0U && s_wheel_enabled == 0U);

 /* 失能后重新使能仍先停车 */
 s_wheel_req = 1U; calls = 0U; order_len = 0U;
 Debug_ServiceWheel();
 assert(calls == 2U && order[0] == 1U && order[1] == 2U && s_wheel_enabled == 1U);

 puts("Wheel enable/disable service / ordering / gate tests passed");
 return 0;
}
'''

# Debug_ChassisStop 不调用HAL_GetTick/PRIMASK，桩保持最小，避免-Wunused-function。
stop_prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
static uint8_t s_wheel_enabled = 1U;
static uint32_t speed_frames, clear_calls;
static void MecanumControl_Stop(void) {speed_frames++;}
static void MecanumControl_ClearTarget(void) {clear_calls++;}
'''

stop_checks = r'''
int main(void) {
 /* 使能状态：正常停车，允许下发速度0帧 */
 s_wheel_enabled = 1U; speed_frames = 0U; clear_calls = 0U;
 Debug_ChassisStop();
 assert(speed_frames == 1U && clear_calls == 0U);

 /* 失能状态：只清目标，绝不下发速度帧（否则X42S会重新使能锁轴） */
 s_wheel_enabled = 0U; speed_frames = 0U; clear_calls = 0U;
 Debug_ChassisStop();
 assert(speed_frames == 0U && clear_calls == 1U);

 /* 重新使能后恢复下发速度帧 */
 s_wheel_enabled = 1U; speed_frames = 0U; clear_calls = 0U;
 Debug_ChassisStop();
 assert(speed_frames == 1U && clear_calls == 0U);

 puts("Chassis stop gate test passed (no speed frame while disabled)");
 return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="ilhc_wheel_test_") as directory:
    folder = Path(directory)
    for name, prelude, extra, checks in (
        ("wheel", wheel_prelude,
         function("Debug_WheelReady") + function("Debug_ServiceWheel"), wheel_checks),
        ("stop", stop_prelude,
         function("Debug_WheelReady") + function("Debug_ChassisStop"), stop_checks),
    ):
        src, exe = folder / (name + ".c"), folder / (name + ".exe")
        src.write_text(prelude + extra + checks, encoding="utf-8")
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)],
                       check=True)
        subprocess.run([str(exe)], check=True)

# Debug_ParseLine 依赖过多，解析侧闸门与初始化用文本核对。
flat = " ".join(source.split())
parse = source[source.index("static void Debug_ParseLine("):source.index("static void DebugUsart_ErrorCallback(")]
fparse = " ".join(parse.split())
assert 'Debug_StrCaseCmp(line, "WHEELEN")' in fparse
assert 'Debug_StrCaseCmp(line, "WHEELOFF")' in fparse
assert 'Debug_WheelReady() && Debug_ParseManual(line + 7U, v)' in fparse
assert '(n >= 2U) && (Debug_WheelReady() != 0U)' in fparse
assert 'Debug_ZdtAck(10U)' in fparse
assert 's_offset_req || (s_wheel_req != 0U)' in flat

init = source[source.index("void DebugUsart_Init(void)"):source.index("void DebugUsart_Send(void)")]
assert "s_wheel_req = 0U; s_wheel_enabled = 1U;" in " ".join(init.split())

# 任务体：失能切换必须最后执行，且所有停车路径都走带闸门的Debug_ChassisStop。
fsend = " ".join(source[source.index("void DebugUsart_Send(void)"):].split())
assert 0 < fsend.index("Debug_ServiceZdt();") < fsend.index("Debug_ServiceWheel();")
assert fsend.index("if (s_stop_req != 0U)") < fsend.index("Debug_ServiceManual();")
assert fsend.index("Debug_ServiceManual();") < fsend.index("Debug_ServiceWheel();")
# 任务体内不得出现未经闸门的停车调用，失能帧才会是该周期UART4上的最后一批帧。
assert "MecanumControl_Stop()" not in fsend, "任务体必须改用Debug_ChassisStop"
assert "MecanumControl_MoveVelocity" not in fsend
# OPS session 变化通过专用取消函数停车，任务体仍只是普通停车路径调用方。
assert fsend.count("Debug_ChassisStop();") == 6, fsend.count("Debug_ChassisStop();")
assert "Debug_CancelOpsSessionMotion();" in fsend

# 闸门实现：使能状态走停车，失能状态只清目标。
chassis_stop = " ".join(function("Debug_ChassisStop").split())
assert "if (Debug_WheelReady() != 0U)" in chassis_stop
assert "MecanumControl_Stop();" in chassis_stop
assert "MecanumControl_ClearTarget();" in chassis_stop

# 手动服务在失能时只清目标，不得回落到MoveVelocity。
manual = " ".join(function("Debug_ServiceManual").split())
assert "if (Debug_WheelReady() == 0U)" in manual
assert "MecanumControl_ClearTarget();" in manual

# 事件10必须是新增的失能拒绝文本，否则ZDT会显示错误的ACK。
# 数组含一个NULL占位（事件7），字符串内还有逗号，因此按标记而不是逗号切分。
arr = source[source.index("static const char * const s_ack_text[] = {"):]
arr = arr[:arr.index("};")]
elements = re.findall(r'NULL|"(?:[^"\\]|\\.)*"', arr)
assert len(elements) == 12, elements
assert elements[7] == "NULL", elements
assert "ERR WHEEL DISABLED" in elements[10], elements[10]
# 事件11：单轮测试收到的回包状态码不是 0x02（参数/保护错误）时必须报错而不是当成功。
assert "ERR ZDT REPLY STATUS" in elements[11], elements[11]

# 驱动层：头文件声明ClearTarget，Stop拆分为"清目标 + 下发速度0帧"。
assert "void MecanumControl_ClearTarget(void);" in mecanum_h
stop_body = mecanum_c[mecanum_c.index("void MecanumControl_Stop(void)"):]
stop_body = stop_body[:stop_body.index("\n}")]
fstop = " ".join(stop_body.split())
assert "MecanumControl_ClearTarget();" in fstop
assert "SetMotorVoltageAndDirection(0, 0, 0, 0);" in fstop

assert "void MecanumControl_Disable(void);" in mecanum_h
body = mecanum_c[mecanum_c.index("void MecanumControl_Disable(void)"):]
body = body[:body.index("}")]
for addr in "1234":
    assert "ZDT_X42S_Disable(%sU);" % addr in body

# ClearTarget 只清软件状态，绝不触碰UART4。
clear_body = mecanum_c[mecanum_c.index("void MecanumControl_ClearTarget(void)"):]
clear_body = clear_body[:clear_body.index("\n}")]
assert "SetMotorVoltageAndDirection" not in clear_body
assert "ZDT_X42S_" not in clear_body

print("Firmware wheel lock command, gate and driver wiring passed")
