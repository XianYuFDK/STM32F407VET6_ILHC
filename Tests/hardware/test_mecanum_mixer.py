"""编译真实麦轮混控与 ZDT 输出层，验证统一坐标下的逻辑有效方向。

统一坐标（2026-09-17）：俯视车头朝上，+X=车左、+Y=车头、+Z=逆时针，
左前1、右前2、左后3、右后4。混控公式直接输出逻辑轮速，
SetMotorVoltageAndDirection() 只把正号转 CW、负号转 CCW。
"""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[2] / "Hardware/mecanum_control.c").read_text(
    encoding="utf-8")


def function(name):
    start = source.index("void " + name + "(")
    brace = source.index("{", start)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>

#define ZDT_X42S_MAX_RPM 3000U
#define ZDT_X42S_DIR_CW  0U
#define ZDT_X42S_DIR_CCW 1U

static int out_dir[4];
static int out_rpm[4];
static int out_calls;

static void Mecanum_NormalizeWheelSpeed(int *speed, int limit)
{
  (void)speed;
  (void)limit;
}

static void ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
  (void)acc;
  assert(addr >= 1U && addr <= 4U);
  out_dir[addr - 1U] = (int)dir;
  out_rpm[addr - 1U] = (int)rpm;
  ++out_calls;
}
'''

checks = r'''
static void reset_output(void)
{
  unsigned i;
  out_calls = 0;
  for (i = 0U; i < 4U; ++i)
  {
    out_dir[i] = -1;
    out_rpm[i] = -1;
  }
}

int main(void)
{
  /* W：Y=+60（向车头），最终 ZDT 方向为 [CW,CCW,CW,CCW]。 */
  reset_output();
  MecanumControl_MoveVelocity(0.0f, 60.0f, 0.0f);
  assert(out_calls == 4);
  assert(out_dir[0] == ZDT_X42S_DIR_CW && out_dir[1] == ZDT_X42S_DIR_CCW);
  assert(out_dir[2] == ZDT_X42S_DIR_CW && out_dir[3] == ZDT_X42S_DIR_CCW);
  assert(out_rpm[0] == 60 && out_rpm[1] == 60 && out_rpm[2] == 60 && out_rpm[3] == 60);

  /* A：X=+60（向车左），最终 ZDT 方向为 [CCW,CCW,CW,CW]。 */
  reset_output();
  MecanumControl_MoveVelocity(60.0f, 0.0f, 0.0f);
  assert(out_calls == 4);
  assert(out_dir[0] == ZDT_X42S_DIR_CCW && out_dir[1] == ZDT_X42S_DIR_CCW);
  assert(out_dir[2] == ZDT_X42S_DIR_CW && out_dir[3] == ZDT_X42S_DIR_CW);

  /* Q：Z=+60（逆时针），最终 ZDT 方向为四轮 CCW。 */
  reset_output();
  MecanumControl_MoveVelocity(0.0f, 0.0f, 60.0f);
  assert(out_calls == 4);
  assert(out_dir[0] == ZDT_X42S_DIR_CCW && out_dir[1] == ZDT_X42S_DIR_CCW);
  assert(out_dir[2] == ZDT_X42S_DIR_CCW && out_dir[3] == ZDT_X42S_DIR_CCW);

  /* S：Y=-60（向车尾），最终 ZDT 方向为 [CCW,CW,CCW,CW]。 */
  reset_output();
  MecanumControl_MoveVelocity(0.0f, -60.0f, 0.0f);
  assert(out_calls == 4);
  assert(out_dir[0] == ZDT_X42S_DIR_CCW && out_dir[1] == ZDT_X42S_DIR_CW);
  assert(out_dir[2] == ZDT_X42S_DIR_CCW && out_dir[3] == ZDT_X42S_DIR_CW);

  puts("Unified-coordinate mecanum mixer and ZDT direction tests passed");
  return 0;
}
'''

implementation = function("SetMotorVoltageAndDirection") + "\n\n" + \
    function("MecanumControl_CalcWheelSpeed") + "\n\n" + \
    function("MecanumControl_MoveVelocity")
code = prelude + implementation + checks

with tempfile.TemporaryDirectory(prefix="ilhc_mecanum_mixer_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src),
                    "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
