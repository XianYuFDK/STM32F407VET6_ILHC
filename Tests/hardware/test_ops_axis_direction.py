"""Runtime checks for the single fixed OPS raw-axis mapping.

The raw-to-unified mapping is fixed at:
    X = -raw_y
    Y = -raw_x

The test verifies the forward/inverse mapping, absolute output, zeroing,
mount-offset compensation, and OPS_SetOrigin conversion.
"""
from pathlib import Path
import re
import subprocess
import tempfile


root = Path(__file__).resolve().parents[2]
ops = (root / "Hardware/ops.c").read_text(encoding="utf-8")


def function(source, name):
    point = re.search(
        r"^(?:static )?(?:uint8_t|void) " + re.escape(name) + r"\(",
        source,
        re.M,
    ).start()
    start = source.rfind("\n", 0, point) + 1
    brace = source.index("{", point)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


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

static void nearf(float actual, float expected, const char *label) {
  if (fabsf(actual - expected) >= 0.00001f) {
    printf("%s: got %.8f expected %.8f\n", label, actual, expected);
  }
  assert(fabsf(actual - expected) < 0.00001f);
}

static void expected_map(float raw_x, float raw_y, float *x, float *y) {
  *x = -raw_y;
  *y = -raw_x;
}

static void expected_inverse(float x, float y, float *raw_x, float *raw_y) {
  *raw_x = -y;
  *raw_y = -x;
}

static void set_pose(float yaw, float center_x, float center_y) {
  float center_raw_x, center_raw_y;
  float mount_raw_x, mount_raw_y;
  float c = cosf(yaw);
  float s = sinf(yaw);

  expected_inverse(center_x, center_y, &center_raw_x, &center_raw_y);
  expected_inverse(0.06f, -0.05f, &mount_raw_x, &mount_raw_y);

  s_ops.frame.x = center_raw_x + c * mount_raw_x - s * mount_raw_y;
  s_ops.frame.y = center_raw_y + s * mount_raw_x + c * mount_raw_y;
  s_ops.frame.z = yaw;
  s_ops.valid_count = 1U;
  s_ops.pose_valid = 1U;
  s_new_flag = 1U;
}
'''


checks = r'''
static void check_direction(void) {
  float x, y, z;
  float raw_x, raw_y;
  float mapped_x, mapped_y;
  float back_x, back_y;

  expected_map(0.123f, -0.456f, &mapped_x, &mapped_y);
  OPS_MapRawToUnified(0.123f, -0.456f, &x, &y);
  nearf(x, mapped_x, "raw-to-unified x");
  nearf(y, mapped_y, "raw-to-unified y");

  expected_inverse(mapped_x, mapped_y, &raw_x, &raw_y);
  OPS_MapUnifiedToRaw(mapped_x, mapped_y, &back_x, &back_y);
  nearf(back_x, raw_x, "unified-to-raw x");
  nearf(back_y, raw_y, "unified-to-raw y");

  set_pose(0.3f, 0.0f, 0.0f);
  OPS_ZeroCoordinates();
  set_pose(1.1f, 0.1f, -0.2f);
  OPS_CopyPosition(&x, &y, &z, 0U);
  nearf(x, 0.1f, "zeroed x");
  nearf(y, -0.2f, "zeroed y");
  nearf(z, 0.8f, "zeroed z");

  set_pose(0.0f, 0.0f, 0.0f);
  OPS_ZeroCoordinates();
  set_pose(0.0f, 0.0f, 0.1f);
  OPS_CopyPosition(&x, &y, &z, 0U);
  nearf(x, 0.0f, "forward does not change x");
  nearf(y, 0.1f, "forward increases y");

  set_pose(0.0f, -0.1f, 0.0f);
  OPS_CopyPosition(&x, &y, &z, 0U);
  nearf(x, -0.1f, "right decreases x");
  nearf(y, 0.0f, "right does not change y");

  set_pose(0.7f, 0.11f, -0.22f);
  expected_map(s_ops.frame.x, s_ops.frame.y, &mapped_x, &mapped_y);
  OPS_CopyPosition(&x, &y, &z, 1U);
  nearf(x, mapped_x, "absolute x");
  nearf(y, mapped_y, "absolute y");
  nearf(z, 0.7f, "absolute z");

  set_pose(0.4f, 0.0f, 0.0f);
  OPS_SetOrigin(0.12f, -0.34f);
  expected_inverse(0.12f, -0.34f, &raw_x, &raw_y);
  nearf(s_ops.origin_x, raw_x, "SetOrigin raw x");
  nearf(s_ops.origin_y, raw_y, "SetOrigin raw y");
  assert(s_ops.zero_enabled == 1U);
  nearf(s_origin_yaw, 0.4f, "SetOrigin yaw");
}

int main(void) {
  check_direction();
  puts("OPS fixed axis direction X=-raw_y, Y=-raw_x passed");
  return 0;
}
'''


names = [
    "OPS_MapRawToUnified",
    "OPS_MapUnifiedToRaw",
    "OPS_CopyPosition",
    "OPS_ZeroCoordinates",
    "OPS_SetOrigin",
]
code = prelude + "\n".join(function(ops, name) for name in names) + checks

with tempfile.TemporaryDirectory(prefix="ilhc_ops_axis_direction_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(
        [
            "gcc",
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(src),
            "-lm",
            "-o",
            str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe)], check=True)
