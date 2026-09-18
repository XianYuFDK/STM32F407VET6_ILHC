"""Periodic OPS session handling and WHEELOFF-safe cancellation tests."""
from pathlib import Path
import re
import subprocess
import tempfile


source = (Path(__file__).resolve().parents[2] /
          "Hardware/debug_usart.c").read_text(encoding="utf-8")


def function(name):
    match = re.search(r"^(?:static )?(?:void|uint8_t) " +
                      re.escape(name) + r"\(.*?\)\s*\{",
                      source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


# The default task must service OPS and consume the session flag every cycle.
send = function("DebugUsart_Send")
assert send.index("OPS_ServiceRx();") < send.index("OPS_ConsumeSessionChanged()")
assert (send.index("OPS_ConsumeSessionChanged()") <
        send.index("Debug_CancelOpsSessionMotion();"))
assert (send.index("Debug_CancelOpsSessionMotion();") <
        send.index("DebugUsart_ServiceRx();"))

cancel = function("Debug_CancelOpsSessionMotion")
assert "s_goto_active = 0U;" in cancel
assert "s_manual_active = 0U;" in cancel
assert "Debug_ChassisStop();" in cancel


prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>

static volatile uint8_t s_goto_active;
static volatile uint8_t s_manual_active;
static uint8_t s_wheel_enabled = 1U;
static unsigned speed_frames;
static unsigned clear_calls;
static unsigned service_calls;
static uint8_t session_changed;

static void MecanumControl_Stop(void) { ++speed_frames; }
static void MecanumControl_ClearTarget(void) { ++clear_calls; }
static void OPS_ServiceRx(void) { ++service_calls; }
static uint8_t OPS_ConsumeSessionChanged(void)
{
  uint8_t changed = session_changed;
  session_changed = 0U;
  return changed;
}
'''


checks = r'''
static void service_cycle(void)
{
  OPS_ServiceRx();
  if (OPS_ConsumeSessionChanged() != 0U)
  {
    Debug_CancelOpsSessionMotion();
  }
}

static void reset_counts(void)
{
  speed_frames = 0U;
  clear_calls = 0U;
  service_calls = 0U;
}

int main(void)
{
  /* Runtime GOTO plus active manual motion: session change cancels both. */
  s_wheel_enabled = 1U;
  s_goto_active = 1U;
  s_manual_active = 1U;
  session_changed = 1U;
  reset_counts();
  service_cycle();
  assert(s_goto_active == 0U);
  assert(s_manual_active == 0U);
  assert(speed_frames == 1U && clear_calls == 0U);
  assert(service_calls == 1U);

  /* No new session event: no motion state is touched. */
  s_goto_active = 1U;
  s_manual_active = 1U;
  session_changed = 0U;
  reset_counts();
  service_cycle();
  assert(s_goto_active == 1U && s_manual_active == 1U);
  assert(speed_frames == 0U && clear_calls == 0U);

  /* WHEELOFF: cancel state and clear targets, but never emit a speed frame. */
  s_wheel_enabled = 0U;
  s_goto_active = 1U;
  s_manual_active = 1U;
  session_changed = 1U;
  reset_counts();
  service_cycle();
  assert(s_goto_active == 0U && s_manual_active == 0U);
  assert(speed_frames == 0U && clear_calls == 1U);

  /* The session flag is consumed once; the next cycle must not repeat stop. */
  reset_counts();
  service_cycle();
  assert(speed_frames == 0U && clear_calls == 0U);

  puts("OPS runtime session cancellation tests passed");
  return 0;
}
'''


implementation = "\n".join(
    function(name) for name in (
        "Debug_WheelReady",
        "Debug_ChassisStop",
        "Debug_CancelOpsSessionMotion",
    )
)
code = prelude + implementation + checks

with tempfile.TemporaryDirectory(prefix="ilhc_ops_session_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src),
         "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)
