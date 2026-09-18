"""Compile and run the real chassis_move() controller with an OPS stub.

The test verifies the world-to-body rotation, X/Y gain separation, final
axis limiting, minimum-speed compensation, and sign symmetry.
"""
from pathlib import Path
import subprocess
import tempfile


source = (Path(__file__).resolve().parents[2] /
          "Hardware/mecanum_control.c").read_text(encoding="utf-8")


def function(name):
    start = source.index("void " + name + "(")
    brace = source.index("{", start)
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prelude = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MECANUM_RAD_TO_DEG 57.2957795f

float mKpx = 1.0f, mKpy = 1.0f, mKpz = 1.0f;
float XYVmax = 1600.0f, ZVmax = 750.0f;
float XYVmin = 0.0f, ZVmin = 0.0f;
float pos_x, pos_y, zangle;
float devx, devy, devz;
int SpeedTarget[4], last_Speed[4];
uint8_t in_pos, near_pos, delay_pos;
void chassis_move(int x, int y, int z);

static float sim_x_m, sim_y_m, sim_yaw_rad;
static uint8_t OPS_GetPosition(float *x, float *y, float *z)
{
  *x = sim_x_m;
  *y = sim_y_m;
  *z = sim_yaw_rad;
  return 1U;
}

static float expected_limit(float value, float max, float min)
{
  if (value > 5.0f)
  {
    value += min;
  }
  else if (value < -5.0f)
  {
    value -= min;
  }

  if (value > max)
  {
    value = max;
  }
  else if (value < -max)
  {
    value = -max;
  }
  return value;
}

static void expected_wheels(float current_x, float current_y, float current_yaw,
                            int target_x, int target_y, int target_yaw,
                            float kpx, float kpy, float kpz,
                            float xymax, float xymin,
                            float zmax, float zmin, int expected[4])
{
  float rad = current_yaw * 3.1415926f / 180.0f;
  float c = cosf(rad);
  float s = sinf(rad);
  float dx = (float)target_x - current_x;
  float dy = (float)target_y - current_y;
  float body_x = c * dx + s * dy;
  float body_y = -s * dx + c * dy;
  float cmd_x = kpx * body_x;
  float cmd_y = kpy * body_y;
  float dz = (float)target_yaw - current_yaw;
  float vz;

  cmd_x = expected_limit(cmd_x, xymax, xymin);
  cmd_y = expected_limit(cmd_y, xymax, xymin);

  dz = fmodf(dz, 360.0f);
  if (dz > 180.0f)       { dz -= 360.0f; }
  else if (dz < -180.0f) { dz += 360.0f; }
  vz = expected_limit(kpz * dz, zmax, zmin);

  expected[0] = (int)( cmd_y - cmd_x - vz);
  expected[1] = (int)(-cmd_y - cmd_x - vz);
  expected[2] = (int)( cmd_y + cmd_x - vz);
  expected[3] = (int)(-cmd_y + cmd_x - vz);
}

static void run_case(float current_x, float current_y, float current_yaw,
                     int target_x, int target_y, int target_yaw,
                     float kpx, float kpy, float kpz,
                     float xymax, float xymin,
                     float zmax, float zmin, int expected[4])
{
  unsigned i;
  unsigned cycle;

  sim_x_m = current_x / 1000.0f;
  sim_y_m = current_y / 1000.0f;
  sim_yaw_rad = current_yaw * 3.1415926f / 180.0f;
  mKpx = kpx;
  mKpy = kpy;
  mKpz = kpz;
  XYVmax = xymax;
  XYVmin = xymin;
  ZVmax = zmax;
  ZVmin = zmin;

  for (i = 0U; i < 4U; ++i)
  {
    SpeedTarget[i] = 0;
    last_Speed[i] = 0;
  }
  in_pos = near_pos = delay_pos = 0U;
  devx = devy = devz = 0.0f;

  /* Constant simulated pose: let the existing slew limiter settle. */
  for (cycle = 0U; cycle < 300U; ++cycle)
  {
    chassis_move(target_x, target_y, target_yaw);
  }

  for (i = 0U; i < 4U; ++i)
  {
    int delta = last_Speed[i] - expected[i];
    if (delta < 0) delta = -delta;
    if (delta > 1)
    {
      fprintf(stderr,
              "wheel %u: got %d expected %d (yaw %.1f, kp %.1f/%.1f)\n",
              i, last_Speed[i], expected[i], current_yaw, kpx, kpy);
      assert(0);
    }
  }
}
'''


checks = r'''
int main(void)
{
  int expected[4];
  int negative[4];

  /* yaw=0, pure world X: changing KPY must not change the X command. */
  expected_wheels(0.0f, 0.0f, 0.0f, 20, 0, 0,
                  1.0f, 10.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 0.0f, 20, 0, 0,
           1.0f, 10.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, expected);
  expected_wheels(0.0f, 0.0f, 0.0f, 20, 0, 0,
                  1.0f, 50.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 0.0f, 20, 0, 0,
           1.0f, 50.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, expected);

  /* yaw=90, pure world X becomes pure body Y. */
  expected_wheels(0.0f, 0.0f, 90.0f, 20, 0, 90,
                  2.0f, 5.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 90.0f, 20, 0, 90,
           2.0f, 5.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, expected);

  /* yaw=45, simultaneous world errors: verify the rotated command. */
  expected_wheels(0.0f, 0.0f, 45.0f, 10, 10, 45,
                  2.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 45.0f, 10, 10, 45,
           2.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, expected);

  /* At yaw=45 with world errors (10,10), body Y is zero.  KPY must not
     affect the body X command. */
  expected_wheels(0.0f, 0.0f, 45.0f, 10, 10, 45,
                  2.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 45.0f, 10, 10, 45,
           2.0f, 300.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, expected);

  /* At yaw=45 with world errors (-10,10), body X is zero.  KPX must not
     affect the body Y command. */
  expected_wheels(0.0f, 0.0f, 45.0f, -10, 10, 45,
                  3.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 45.0f, -10, 10, 45,
           300.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, expected);

  /* Final X/Y commands are limited individually, not after decomposition. */
  expected_wheels(0.0f, 0.0f, 45.0f, 1000, 1000, 45,
                  100.0f, 100.0f, 0.0f, 50.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 45.0f, 1000, 1000, 45,
           100.0f, 100.0f, 0.0f, 50.0f, 0.0f, 1000.0f, 0.0f, expected);

  /* XYVmin is applied once to each final command, not to each trig term. */
  expected_wheels(0.0f, 0.0f, 45.0f, 10, 10, 45,
                  1.0f, 1.0f, 0.0f, 1000.0f, 5.0f, 1000.0f, 0.0f,
                  expected);
  run_case(0.0f, 0.0f, 45.0f, 10, 10, 45,
           1.0f, 1.0f, 0.0f, 1000.0f, 5.0f, 1000.0f, 0.0f, expected);

  /* ZVmin is an active yaw-loop parameter. */
  expected_wheels(0.0f, 0.0f, 0.0f, 0, 0, 10,
                  1.0f, 1.0f, 1.0f, 1000.0f, 0.0f, 100.0f, 5.0f,
                  expected);
  run_case(0.0f, 0.0f, 0.0f, 0, 0, 10,
           1.0f, 1.0f, 1.0f, 1000.0f, 0.0f, 100.0f, 5.0f, expected);

  /* Positive/negative commands remain symmetric. */
  expected_wheels(0.0f, 0.0f, 30.0f, 20, 30, 30,
                  2.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  expected);
  expected_wheels(0.0f, 0.0f, 30.0f, -20, -30, 30,
                  2.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f,
                  negative);
  assert(negative[0] == -expected[0]);
  assert(negative[1] == -expected[1]);
  assert(negative[2] == -expected[2]);
  assert(negative[3] == -expected[3]);
  run_case(0.0f, 0.0f, 30.0f, 20, 30, 30,
           2.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, expected);
  run_case(0.0f, 0.0f, 30.0f, -20, -30, 30,
           2.0f, 3.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 0.0f, negative);

  puts("chassis_move rotation, gain, limit, min and symmetry tests passed");
  return 0;
}
'''


implementation = "\n".join(
    function(name) for name in (
        "MecanumControl_UpdatePose",
        "MecanumControl_CalcWheelSpeed",
        "numerical_limit",
        "chassis_move",
    )
)
code = prelude + implementation + checks

with tempfile.TemporaryDirectory(prefix="ilhc_chassis_move_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
         str(src), "-lm", "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)
