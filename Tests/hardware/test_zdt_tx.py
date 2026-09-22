"""Compile the real ZDT_UART4 send helper and verify bounded retries.

The test uses a synthetic HAL: the first attempts can fail, while the final
attempt succeeds. It checks both recovery and the cumulative failure counter
without touching a board.
"""
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "Hardware/zdt_x42s.c").read_text(encoding="utf-8")


def function(name):
    match = re.search(
        r"(?:static\s+)?(?:HAL_StatusTypeDef|uint32_t)\s+" +
        re.escape(name) + r"\s*\([^;]*\)\s*\{",
        SOURCE,
        re.S,
    )
    if match is None:
        raise RuntimeError("function not found: " + name)
    start = match.start()
    brace = SOURCE.index("{", match.start())
    end, depth = brace + 1, 1
    while depth:
        depth += (SOURCE[end] == "{") - (SOURCE[end] == "}")
        end += 1
    return SOURCE[start:end]


prelude = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

typedef int HAL_StatusTypeDef;
typedef struct { int dummy; } UART_HandleTypeDef;

#define HAL_OK      0
#define HAL_ERROR   1
#define HAL_BUSY    2
#define HAL_TIMEOUT 3
#define ZDT_X42S_UART huart4
#define ZDT_X42S_TX_TIMEOUT_MS 10U
#define ZDT_X42S_TX_RETRY_COUNT 2U

static UART_HandleTypeDef huart4;
static uint32_t s_tx_error_count;
static unsigned transmit_calls;
static unsigned fail_attempts;
static unsigned delay_calls;

static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,
                                            const uint8_t *data,
                                            uint16_t len,
                                            uint32_t timeout)
{
  (void)uart;
  (void)data;
  (void)len;
  (void)timeout;
  ++transmit_calls;
  return (transmit_calls <= fail_attempts) ? HAL_TIMEOUT : HAL_OK;
}

static void HAL_Delay(uint32_t delay_ms)
{
  (void)delay_ms;
  ++delay_calls;
}
'''

checks = r'''
int main(void)
{
  uint8_t frame[1] = {0x6BU};

  fail_attempts = 2U;
  transmit_calls = 0U;
  delay_calls = 0U;
  assert(ZDT_X42S_Send(frame, sizeof(frame)) == HAL_OK);
  assert(transmit_calls == 3U);
  assert(delay_calls == 2U);
  assert(s_tx_error_count == 0U);

  fail_attempts = 3U;
  transmit_calls = 0U;
  delay_calls = 0U;
  assert(ZDT_X42S_Send(frame, sizeof(frame)) == HAL_TIMEOUT);
  assert(transmit_calls == 3U);
  assert(delay_calls == 3U);
  assert(s_tx_error_count == 1U);
  assert(ZDT_X42S_GetTxErrorCount() == 1U);

  assert(ZDT_X42S_Send(NULL, sizeof(frame)) == HAL_ERROR);
  assert(ZDT_X42S_GetTxErrorCount() == 2U);

  return 0;
}
'''

code = prelude + function("ZDT_X42S_Send") + "\n" + \
    function("ZDT_X42S_GetTxErrorCount") + "\n" + checks

with tempfile.TemporaryDirectory(prefix="ilhc_zdt_tx_") as directory:
    folder = Path(directory)
    source = folder / "test.c"
    executable = folder / "test.exe"
    source.write_text(code, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
         str(source), "-o", str(executable)],
        check=True,
    )
    subprocess.run([str(executable)], check=True)

print("ZDT TX retry/error-count tests passed")
