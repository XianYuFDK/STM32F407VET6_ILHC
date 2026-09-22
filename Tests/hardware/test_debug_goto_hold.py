"""Compile the real GOTO service and verify one-shot versus hold semantics.

The test uses stubs for the motor/OPS boundary and exercises the real task-side
state transitions. It does not connect to hardware.
"""
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "Hardware/debug_usart.c").read_text(encoding="utf-8")


def function(name):
    start = source.rindex("static ", 0, source.index(name + "("))
    brace = source.index("{", start)
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>

#define DEBUG_OPS_TIMEOUT_MS 200U
#define DEBUG_GOTO_MOVE 1U
#define DEBUG_GOTO_HOLD 2U

static volatile uint8_t s_goto_active;
static uint32_t s_goto_generation;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __enable_irq(void){}
static float s_goto_x, s_goto_y, s_goto_z;
static uint8_t wheel_ready = 1U;
static uint8_t ops_online = 1U;
static uint8_t reached = 0U;
static unsigned goto_calls = 0U;
static unsigned stop_calls = 0U;
static unsigned clear_calls = 0U;

static uint8_t Debug_WheelReady(void) { return wheel_ready; }
static uint8_t OPS_IsOnline(uint32_t timeout_ms)
{
  (void)timeout_ms;
  return ops_online;
}
static uint8_t MecanumControl_GotoOPS(float x, float y, float z, float max_rpm)
{
  (void)x; (void)y; (void)z; (void)max_rpm;
  ++goto_calls;
  return reached;
}
static void MecanumControl_ClearTarget(void) { ++clear_calls; }
static void Debug_ChassisStop(void) { ++stop_calls; }
'''

checks = r'''
int main(void)
{
  /* Hold keeps running after arrival and on later external displacement. */
  s_goto_active = DEBUG_GOTO_HOLD;
  s_goto_x = 10.0f; s_goto_y = 20.0f; s_goto_z = 30.0f;
  reached = 1U;
  Debug_ServiceGoto();
  Debug_ServiceGoto();
  Debug_ServiceGoto();
  assert(goto_calls == 3U);
  assert(s_goto_active == DEBUG_GOTO_HOLD);
  assert(stop_calls == 0U);

  reached = 0U;                       /* Simulate a manual external disturbance. */
  Debug_ServiceGoto();
  assert(goto_calls == 4U);
  assert(s_goto_active == DEBUG_GOTO_HOLD);
  assert(stop_calls == 0U);

  /* Normal GOTO exits and stops after arrival. */
  s_goto_active = DEBUG_GOTO_MOVE;
  reached = 1U;
  Debug_ServiceGoto();
  assert(s_goto_active == 0U);
  assert(stop_calls == 1U);

  /* Inactive and still-moving states remain well-defined. */
  s_goto_active = 0U;
  Debug_ServiceGoto();
  assert(goto_calls == 5U);

  s_goto_active = DEBUG_GOTO_MOVE;
  reached = 0U;
  Debug_ServiceGoto();
  assert(goto_calls == 6U);
  assert(s_goto_active == DEBUG_GOTO_MOVE);
  assert(stop_calls == 1U);

  /* Wheel disable and OPS loss cancel hold safely. */
  s_goto_active = DEBUG_GOTO_HOLD;
  wheel_ready = 0U;
  Debug_ServiceGoto();
  assert(s_goto_active == 0U);
  assert(clear_calls == 1U);
  assert(stop_calls == 1U);

  wheel_ready = 1U;
  ops_online = 0U;
  s_goto_active = DEBUG_GOTO_HOLD;
  Debug_ServiceGoto();
  assert(s_goto_active == 0U);
  assert(stop_calls == 2U);

  puts("GOTO one-shot and GOTOHOLD continuous position-loop tests passed");
  return 0;
}
'''

code = prelude + function("Debug_ServiceGoto") + checks

with tempfile.TemporaryDirectory(prefix="ilhc_goto_hold_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)
