"""Runtime math checks for the OPS ZERO coordinate frame.

The test compiles the real mapping, compensation, ZERO and position-copy
functions from Hardware/ops.c. It models OPS raw coordinates from a synthetic
unified world pose, then verifies that ZERO creates both a translation origin
and a rotated XY frame whose +Y is the heading at ZERO time.
"""
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
OPS = (ROOT / "Hardware/ops.c").read_text(encoding="utf-8")


def function(name):
    match = re.search(
        r"^(?:static )?(?:uint8_t|void) " + re.escape(name) + r"\(",
        OPS,
        re.M,
    )
    if match is None:
        raise RuntimeError("function not found: " + name)
    start = OPS.rfind("\n", 0, match.start()) + 1
    brace = OPS.index("{", match.start())
    end, depth = brace + 1, 1
    while depth:
        depth += (OPS[end] == "{") - (OPS[end] == "}")
        end += 1
    return OPS[start:end]


prelude = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

static struct {
  struct { float x, y, z; } frame;
  unsigned valid_count;
  uint8_t pose_valid;
  float origin_x, origin_y;
  uint8_t zero_enabled;
} s_ops;

static float s_mount_x_mm = 60.0f;
static float s_mount_y_mm = -50.0f;
static float s_reference_yaw;
static float s_origin_yaw;
static uint8_t s_new_flag;
static unsigned mask;

static unsigned __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask = 1U; }
static void __enable_irq(void) { mask = 0U; }

static void OPS_MapRawToUnified(float raw_x, float raw_y, float *x, float *y);
static void OPS_MapUnifiedToRaw(float x, float y, float *raw_x, float *raw_y);
static void OPS_RotateWorldToZero(float world_x, float world_y,
                                  float zero_yaw, float *x, float *y);
static void OPS_ZeroCoordinates(void);
static uint8_t OPS_CopyPosition(float *x, float *y, float *z, uint8_t absolute);

static void nearf(float actual, float expected, const char *label)
{
  if (fabsf(actual - expected) >= 0.00001f)
  {
    printf("%s: got %.8f expected %.8f\n", label, actual, expected);
  }
  assert(fabsf(actual - expected) < 0.00001f);
}

/* OPS raw sensor pose for a vehicle center and heading in unified world.
 * mount = (left, forward) = (60mm, -50mm). */
static void pose(float yaw, float center_x, float center_y)
{
  const float mount_x = 0.060f;
  const float mount_y = -0.050f;
  float c = cosf(yaw);
  float s = sinf(yaw);
  float sensor_x = center_x + c * mount_x + s * mount_y;
  float sensor_y = center_y - s * mount_x + c * mount_y;

  OPS_MapUnifiedToRaw(sensor_x, sensor_y, &s_ops.frame.x, &s_ops.frame.y);
  s_ops.frame.z = yaw;
  s_ops.valid_count = 1U;
  s_ops.pose_valid = 1U;
  s_new_flag = 1U;
}

static void check_after_motion(float zero_yaw,
                               float zero_x, float zero_y,
                               float current_yaw,
                               float current_x, float current_y,
                               float expected_x, float expected_y,
                               float expected_z,
                               const char *label)
{
  float x, y, z;

  pose(zero_yaw, zero_x, zero_y);
  OPS_ZeroCoordinates();
  pose(current_yaw, current_x, current_y);
  OPS_CopyPosition(&x, &y, &z, 0U);

  nearf(x, expected_x, label);
  nearf(y, expected_y, label);
  nearf(z, expected_z, label);
}
'''


checks = r'''
int main(void)
{
  float x, y;
  float raw_x, raw_y;
  float c, s;
  const float pi2 = 1.5707963268f;
  const float pi6 = 0.5235987756f;

  /* Test 1: ZERO at yaw=0, then move world +Y by 1m. */
  check_after_motion(0.0f, 2.0f, -1.0f,
                     0.0f, 2.0f, 0.0f,
                     0.0f, 1.0f, 0.0f, "test1");

  /* Test 2: ZERO at +30deg, then move along the current heading. */
  c = cosf(pi6);
  s = sinf(pi6);
  check_after_motion(pi6, 0.0f, 0.0f,
                     pi6, s, c,
                     0.0f, 1.0f, 0.0f, "test2");

  /* Test 3: ZERO at -30deg, then move along the current heading. */
  c = cosf(-pi6);
  s = sinf(-pi6);
  check_after_motion(-pi6, 0.0f, 0.0f,
                     -pi6, s, c,
                     0.0f, 1.0f, 0.0f, "test3");

  /* Test 4: ZERO at +90deg, then move world +X. */
  check_after_motion(pi2, 0.0f, 0.0f,
                     pi2, 1.0f, 0.0f,
                     0.0f, 1.0f, 0.0f, "test4");

  /* Test 5: ZERO at 37deg, then move along the current left by 0.6m. */
  c = cosf(0.6457718232f);
  s = sinf(0.6457718232f);
  check_after_motion(0.6457718232f, 0.0f, 0.0f,
                     0.6457718232f, 0.6f * c, -0.6f * s,
                     0.6f, 0.0f, 0.0f, "test5");

  /* Test 6: ZERO position is not the OPS world origin. */
  c = cosf(0.4363323130f);
  s = sinf(0.4363323130f);
  check_after_motion(0.4363323130f, 2.3f, -1.7f,
                     0.4363323130f, 2.3f + s, -1.7f + c,
                     0.0f, 1.0f, 0.0f, "test6");

  /* Test 7: pure rotation around the body center must cancel mount offset. */
  check_after_motion(0.0f, 0.0f, 0.0f,
                     pi2, 0.0f, 0.0f,
                     0.0f, 0.0f, pi2, "test7");

  /* Test 8: raw -> unified -> raw is reversible. */
  OPS_MapRawToUnified(1.2f, -0.7f, &x, &y);
  OPS_MapUnifiedToRaw(x, y, &raw_x, &raw_y);
  nearf(raw_x, 1.2f, "test8 raw x");
  nearf(raw_y, -0.7f, "test8 raw y");
  OPS_MapRawToUnified(-3.4f, 2.1f, &x, &y);
  OPS_MapUnifiedToRaw(x, y, &raw_x, &raw_y);
  nearf(raw_x, -3.4f, "test8 raw x2");
  nearf(raw_y, 2.1f, "test8 raw y2");

  puts("OPS ZERO frame math tests passed");
  return 0;
}
'''


names = [
    "OPS_MapRawToUnified",
    "OPS_MapUnifiedToRaw",
    "OPS_RotateWorldToZero",
    "OPS_CopyPosition",
    "OPS_ZeroCoordinates",
]
code = prelude + "\n".join(function(name) for name in names) + checks

with tempfile.TemporaryDirectory(prefix="ilhc_ops_zero_frame_") as directory:
    folder = Path(directory)
    source = folder / "test.c"
    executable = folder / "test.exe"
    source.write_text(code, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
         str(source), "-lm", "-o", str(executable)],
        check=True,
    )
    subprocess.run([str(executable)], check=True)
