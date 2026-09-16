"""编译真实 Debug_ParseLine，运行时验证三个协议的 X/Y 交换与钳位，不连接设备。

对外协议：MANUAL=X(左右),Y(前后),W；GOTO=X(场地左右),Y(场地前后),Z；
OPSOFFSET=X(左右偏移),Y(前后偏移)；KPX=左右轴增益、KPY=前后轴增益。
MANUAL 暂存协议原序（交换在 Debug_ServiceManual 调用 MoveVelocity 时完成，
由 test_debug_manual.py 覆盖），GOTO/OPSOFFSET/参数表在此处直接落到内部顺序。
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "Hardware/debug_usart.c").read_text(encoding="utf-8")


def function(name):
    start = source.rfind("static ", 0, source.index(name + "("))
    brace = source.index("{", start)
    level = 1
    end = brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def block(start_marker, end_marker):
    start = source.index(start_marker)
    end = source.index(end_marker, start) + len(end_marker)
    return source[start:end]


prelude = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
/* 底盘内部量：形参/变量名保持(前后, 左右)，参数表按对外命名映射 */
static float mKpx = 2.3f, mKpy = 2.3f, mKpz = 9.0f;
static float XYVmax = 1600.0f, ZVmax = 750.0f, XYVmin = 5.0f, ZVmin = 5.0f;
static float zangle = 0.0f;
/* Debug_ParseLine 触及的全部状态 */
static volatile uint8_t s_stop_req, s_zero_req, s_offset_req, s_manual_active;
static volatile float s_offset_x, s_offset_y;
static volatile uint8_t s_goto_active;
static float s_goto_x, s_goto_y, s_goto_z;
static volatile int16_t s_manual_velocity[3];
static volatile uint32_t s_manual_tick;
static uint8_t s_wheel_enabled = 1U;
static volatile uint8_t s_wheel_req;
static int16_t s_zdt_args[3];
static volatile uint8_t s_zdt_active, s_zdt_req, s_zdt_text_mode;
static volatile uint8_t s_dm_enable_req, s_dm_disable_req, s_dm_zero_req;
static uint32_t tick, mask;
static uint32_t HAL_GetTick(void) {return tick;}
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __enable_irq(void) {mask=0;}
/* 与本用例无关的分支：步进/CAN拒绝/DM参数/ZDT应答 */
static uint8_t Debug_ParseStepper(const char *line) {(void)line; return 0U;}
static uint8_t Debug_RejectCanCommand(const char *line) {(void)line; return 0U;}
static void Debug_SetDmValue(const char *n, float v) {(void)n; (void)v;}
static void Debug_ZdtAck(uint8_t e) {(void)e;}
'''

checks = r'''
int main(void) {
  char line[64];

  /* 解析路径只用读写状态，不开关中断；这里显式覆盖三个桩的用法 */
  mask = __get_PRIMASK();
  __disable_irq();
  __enable_irq();
  assert(mask == 0U);

  /* MANUAL：暂存协议原序 X(左右),Y(前后),W；交换在服务层完成 */
  strcpy(line, "MANUAL=70,60,30");
  Debug_ParseLine(line);
  assert(s_manual_active == 1U);
  assert(s_manual_velocity[0] == 70 && s_manual_velocity[1] == 60 && s_manual_velocity[2] == 30);
  strcpy(line, "manual=-100,0,25");                 /* 大小写不敏感 */
  tick += 10U;
  Debug_ParseLine(line);
  assert(s_manual_velocity[0] == -100 && s_manual_velocity[1] == 0 && s_manual_velocity[2] == 25);
  strcpy(line, "MANUAL=301,0,0");                    /* 超范围仍拒绝 */
  Debug_ParseLine(line);
  assert(s_manual_velocity[0] == -100);

  /* GOTO：协议 X=车左 → 内部 s_goto_y（同向）；Y=车头 → 内部 s_goto_x 并取反
     （内部 +前后 指向车尾） */
  zangle = 12.0f;
  strcpy(line, "GOTO=1000,2000,45");
  Debug_ParseLine(line);
  assert(s_goto_active == 1U);
  assert(s_goto_x == -2000.0f && s_goto_y == 1000.0f && s_goto_z == 45.0f);
  strcpy(line, "GOTO=1000,2000");                    /* 省略Z：保持当前航向 */
  Debug_ParseLine(line);
  assert(s_goto_x == -2000.0f && s_goto_y == 1000.0f && s_goto_z == 12.0f);
  strcpy(line, "GOTO=-5000,250");                    /* 两轴仍按±3000钳位，交换取反在钳位之后 */
  Debug_ParseLine(line);
  assert(s_goto_x == -250.0f && s_goto_y == -3000.0f);
  strcpy(line, "GOTO=1000");                         /* 少于两个数：不接受新目标 */
  Debug_ParseLine(line);
  assert(s_goto_x == -250.0f && s_goto_y == -3000.0f);

  /* OPSOFFSET：协议 X=车左 → s_offset_y 同向；Y=车头 → s_offset_x 取反 */
  strcpy(line, "OPSOFFSET=60,-50");                  /* 车左60mm、车后50mm */
  Debug_ParseLine(line);
  assert(s_offset_req == 1U);
  assert(s_offset_x == 50.0f && s_offset_y == 60.0f);
  s_offset_req = 0U;
  strcpy(line, "OPSOFFSET=-501,0");                  /* 超范围仍拒绝 */
  Debug_ParseLine(line);
  assert(s_offset_req == 0U);

  /* 参数表：KPX 指向左右轴增益 mKpy，KPY 指向前后的 mKpx */
  strcpy(line, "KPX=7");
  Debug_ParseLine(line);
  assert(mKpy == 7.0f && mKpx == 2.3f);
  strcpy(line, "KPY=4");
  Debug_ParseLine(line);
  assert(mKpx == 4.0f && mKpy == 7.0f);
  strcpy(line, "KPZ=5");
  Debug_ParseLine(line);
  assert(mKpz == 5.0f && mKpx == 4.0f && mKpy == 7.0f);
  strcpy(line, "KPX=99");                            /* 超限按上限钳位到50，写的仍是 mKpy */
  Debug_ParseLine(line);
  assert(mKpy == 50.0f && mKpx == 4.0f);

  /* 失能闸门：MANUAL/GOTO 在解析阶段即被丢弃 */
  s_wheel_enabled = 0U;
  s_manual_active = 0U;
  s_goto_active = 0U;
  s_goto_x = s_goto_y = 0.0f;
  strcpy(line, "MANUAL=70,60,30");
  Debug_ParseLine(line);
  assert(s_manual_active == 0U);
  strcpy(line, "GOTO=1000,2000,45");
  Debug_ParseLine(line);
  assert(s_goto_active == 0U && s_goto_x == 0.0f && s_goto_y == 0.0f);
  s_wheel_enabled = 1U;

  /* 与坐标无关的既有命令仍可用 */
  strcpy(line, "STOP");
  Debug_ParseLine(line);
  assert(s_stop_req == 1U);

  /* 适配层反向映射：内部(-2000,1000) -> 用户 X=+1000(左)、Y=+2000(前) */
  {
    float ux = 0.0f, uy = 0.0f;
    Debug_InternalToUser(-2000.0f, 1000.0f, &ux, &uy);
    assert(ux == 1000.0f && uy == 2000.0f);
  }

  puts("ParseLine X/Y boundary mapping passed:");
  puts("  GOTO=X(left),Y(front) -> internal forward=-Y, lateral=+X  (adapter layer)");
  puts("  OPSOFFSET=X(left),Y(front) -> internal forward=-Y, lateral=+X (adapter layer)");
  puts("  MANUAL passed through adapter: forward=-Y, lateral=+X; KPX->mKpy, KPY->mKpx");
  return 0;
}
'''

names = ["Debug_StrCaseCmp", "Debug_StrCaseCmpN", "Debug_ParseFloat", "Debug_ParseFloatList",
         "Debug_ParseManual", "Debug_WheelReady", "Debug_ParseOffset", "Debug_SetParam",
         "Debug_UserToInternal", "Debug_InternalToUser", "Debug_ParseLine"]
typedef = block("typedef struct\n{\n  const char *name;", "} DebugParam_t;")
table = block("static const DebugParam_t s_params[] =", "\n};")
code = prelude + typedef + "\n" + table + "\n" + "\n".join(function(n) for n in names) + checks

with tempfile.TemporaryDirectory(prefix="ilhc_parse_axis_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
