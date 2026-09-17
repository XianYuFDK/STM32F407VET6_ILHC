"""运行时验证 chassis_move 的 X/Y 轴反馈符号和 Kp 归属。

重点：Kp 必须在世界误差旋转到车体坐标之后应用，否则航向不为 0 时
X 的 P 会串到 Y 轴车身分量，表现成“左右 P 反了”。
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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MECANUM_RAD_TO_DEG 57.2957795f

float mKpx = 1.0f, mKpy = 1.0f, mKpz = 0.0f;
float XYVmax = 1600.0f, ZVmax = 750.0f;
float XYVmin = 0.0f, ZVmin = 0.0f;
float pos_x, pos_y, zangle;
float devx, devy, devz;
int SpeedTarget[4], last_Speed[4];
uint8_t in_pos, near_pos, delay_pos;

static float sim_x_m, sim_y_m, sim_yaw_rad;

static uint8_t OPS_GetPosition(float *x, float *y, float *z)
{
  *x = sim_x_m;
  *y = sim_y_m;
  *z = sim_yaw_rad;
  return 1U;
}
'''


checks = r'''
static void reset_run(float x_mm, float y_mm, float yaw_deg,
                      float target_x_mm, float target_y_mm,
                      float kpx, float kpy)
{
  unsigned i;
  sim_x_m = x_mm / 1000.0f;
  sim_y_m = y_mm / 1000.0f;
  sim_yaw_rad = yaw_deg * 3.1415926536f / 180.0f;
  mKpx = kpx;
  mKpy = kpy;
  delay_pos = 0U;
  for (i = 0U; i < 4U; ++i)
  {
    last_Speed[i] = 0;
    SpeedTarget[i] = 0;
  }
  chassis_move((int)target_x_mm, (int)target_y_mm, 0);
  /* 第二周期让速度斜坡退出，验证的是稳态 P 输出而不是首周期 20RPM 斜坡。 */
  chassis_move((int)target_x_mm, (int)target_y_mm, 0);
}

static void expect_wheels(int a, int b, int c, int d)
{
  if (!(abs(last_Speed[0] - a) <= 1 && abs(last_Speed[1] - b) <= 1 &&
        abs(last_Speed[2] - c) <= 1 && abs(last_Speed[3] - d) <= 1))
  {
    fprintf(stderr, "wheels got [%d %d %d %d], expected [%d %d %d %d]\n",
            last_Speed[0], last_Speed[1], last_Speed[2], last_Speed[3],
            a, b, c, d);
    fflush(stderr);
    assert(0);
  }
}

int main(void)
{
  /* yaw=0：X 正误差必须驱动“左移”轮速，并且只使用 mKpx。 */
  reset_run(0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 3.0f, 20.0f);
  expect_wheels(-6, -6, 6, 6);

  /* X 负误差无条件反向，证明 devx=目标-当前的反馈方向正确。 */
  reset_run(0.0f, 0.0f, 0.0f, -2.0f, 0.0f, 3.0f, 20.0f);
  expect_wheels(6, 6, -6, -6);

  /* yaw=0：Y 轴误差只使用 mKpy。 */
  reset_run(0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, 7.0f);
  expect_wheels(14, -14, 14, -14);

  /* yaw=90°：世界 +Y 误差旋转成车体 +X，增益必须仍是 mKpx。
     旧“先乘 Kp 再旋转”会给左边叠加 mKpy，此断言会直接失败。 */
  reset_run(0.0f, 0.0f, 90.0f, 0.0f, 2.0f, 3.0f, 20.0f);
  expect_wheels(-6, -6, 6, 6);

  /* yaw=90°：世界 +X 误差应转成车体 -Y，使用 mKpy。 */
  reset_run(0.0f, 0.0f, 90.0f, 2.0f, 0.0f, 3.0f, 7.0f);
  expect_wheels(-14, 14, -14, 14);

  puts("chassis X/Y P gain and feedback sign tests passed");
  return 0;
}
'''


implementation = (
    function("MecanumControl_UpdatePose") + "\n\n" +
    function("MecanumControl_CalcWheelSpeed") + "\n\n" +
    function("numerical_limit") + "\n\n" +
    function("chassis_move")
)
code = prelude + implementation + checks

with tempfile.TemporaryDirectory(prefix="ilhc_chassis_pid_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src),
                    "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
