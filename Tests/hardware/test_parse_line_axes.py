"""编译真实 Debug_ParseLine，运行时验证统一 X/Y 协议与钳位，不连接设备。

对外协议：MANUAL=X(左右),Y(前后),W；GOTO=X(场地左右),Y(场地前后),Z，X/Y 单位 cm；
OPSOFFSET=X(左右偏移),Y(前后偏移)；KPX=左右轴增益、KPY=前后轴增益。
协议、固件内部和底盘控制变量使用同一轴序、同一符号，不再交换或取反。
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
/* 底盘内部量同样使用 X=左右、Y=前后，参数表直接对应。 */
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

  /* MANUAL：X(左右),Y(前后),W 原序暂存。 */
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

  /* GOTO：X=车左、Y=车头，直接写入同名的统一坐标目标。 */
  zangle = 12.0f;
  strcpy(line, "GOTO=100.0,200.0,45.0");
  Debug_ParseLine(line);
  assert(s_goto_active == 1U);
  assert(s_goto_x == 1000.0f && s_goto_y == 2000.0f && s_goto_z == 45.0f);
  strcpy(line, "GOTO=100.0,200.0");                  /* 省略Z：保持当前航向 */
  Debug_ParseLine(line);
  assert(s_goto_x == 1000.0f && s_goto_y == 2000.0f && s_goto_z == 12.0f);
  strcpy(line, "GOTO=0.1,-0.1,0.0");                 /* 0.1cm必须换算成1mm */
  Debug_ParseLine(line);
  assert(s_goto_x == 1.0f && s_goto_y == -1.0f);
  strcpy(line, "GOTO=-500.0,25.0");                  /* 协议按±300.0cm钳位并换算为mm */
  Debug_ParseLine(line);
  assert(s_goto_x == -3000.0f && s_goto_y == 250.0f);
  strcpy(line, "GOTO=100.0");                        /* 少于两个数：不接受新目标 */
  Debug_ParseLine(line);
  assert(s_goto_x == -3000.0f && s_goto_y == 250.0f);

  /* OPSOFFSET：X=车左、Y=车头，直接暂存同轴量。 */
  strcpy(line, "OPSOFFSET=60,-50");                  /* 车左60mm、车后50mm */
  Debug_ParseLine(line);
  assert(s_offset_req == 1U);
  assert(s_offset_x == 60.0f && s_offset_y == -50.0f);
  s_offset_req = 0U;
  strcpy(line, "OPSOFFSET=-501,0");                  /* 超范围仍拒绝 */
  Debug_ParseLine(line);
  assert(s_offset_req == 0U);

  /* 参数表：KPX 写 mKpx（左右），KPY 写 mKpy（前后）。 */
  strcpy(line, "KPX=7");
  Debug_ParseLine(line);
  assert(mKpx == 7.0f && mKpy == 2.3f);
  strcpy(line, "KPY=4");
  Debug_ParseLine(line);
  assert(mKpx == 7.0f && mKpy == 4.0f);
  strcpy(line, "KPZ=5");
  Debug_ParseLine(line);
  assert(mKpz == 5.0f && mKpx == 7.0f && mKpy == 4.0f);
  strcpy(line, "KPX=99");                            /* 超限按上限钳位到50，写的仍是 mKpx */
  Debug_ParseLine(line);
  assert(mKpx == 50.0f && mKpy == 4.0f);

  /* 失能闸门：MANUAL/GOTO 在解析阶段即被丢弃 */
  s_wheel_enabled = 0U;
  s_manual_active = 0U;
  s_goto_active = 0U;
  s_goto_x = s_goto_y = 0.0f;
  strcpy(line, "MANUAL=70,60,30");
  Debug_ParseLine(line);
  assert(s_manual_active == 0U);
  strcpy(line, "GOTO=100.0,200.0,45.0");
  Debug_ParseLine(line);
  assert(s_goto_active == 0U && s_goto_x == 0.0f && s_goto_y == 0.0f);
  s_wheel_enabled = 1U;

  /* 与坐标无关的既有命令仍可用 */
  strcpy(line, "STOP");
  Debug_ParseLine(line);
  assert(s_stop_req == 1U);

  puts("ParseLine unified X/Y protocol passed:");
  puts("  GOTO/OPSOFFSET/MANUAL use X=left,Y=front directly");
  puts("  KPX->mKpx, KPY->mKpy");
  return 0;
}
'''

names = ["Debug_StrCaseCmp", "Debug_StrCaseCmpN", "Debug_ParseFloat", "Debug_ParseFloatList",
         "Debug_ParseManual", "Debug_WheelReady", "Debug_ParseOffset", "Debug_SetParam",
         "Debug_ParseLine"]
typedef = block("typedef struct\n{\n  const char *name;", "} DebugParam_t;")
table = block("static const DebugParam_t s_params[] =", "\n};")
code = prelude + typedef + "\n" + table + "\n" + "\n".join(function(n) for n in names) + checks

with tempfile.TemporaryDirectory(prefix="ilhc_parse_axis_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
