"""Compile the real UART4 TX queue and verify serialization/priorities.

The test uses a HAL stub that records each HAL_UART_Transmit_IT() call.  It
does not connect to a motor; TX completion is triggered explicitly so the
queue state machine and the real frame bytes are exercised.
"""
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
zdt_source = (ROOT / "Hardware/zdt_x42s.c").read_text(encoding="utf-8")
mecanum_source = (ROOT / "Hardware/mecanum_control.c").read_text(encoding="utf-8")


def zdt_tx_implementation():
    start = zdt_source.index("/* --------------------------- 底层参数")
    end = zdt_source.index("/* UART4应答接收：")
    return zdt_source[start:end]


def mecanum_functions():
    """Extract the real normalize and four-wheel submit functions."""
    names = ("Mecanum_NormalizeWheelSpeed", "SetMotorVoltageAndDirection")
    result = []
    for name in names:
        marker = "void " + name + "("
        start = mecanum_source.index(marker)
        brace = mecanum_source.index("{", start)
        level = 1
        end = brace + 1
        while level:
            level += (mecanum_source[end] == "{") - (mecanum_source[end] == "}")
            end += 1
        result.append(mecanum_source[start:end])
    return "\n".join(result)


ZDT_PRELUDE = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

typedef int HAL_StatusTypeDef;
typedef struct { int dummy; } UART_HandleTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_UART_TX_COMPLETE_CB_ID 0
#define ZDT_X42S_DIR_CW 0U
#define ZDT_X42S_DIR_CCW 1U
#define ZDT_X42S_MAX_RPM 3000U
#define ZDT_X42S_DEFAULT_ACC 8U

void ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc);

static UART_HandleTypeDef huart4;
static uint32_t tick;
static uint32_t mask;
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask = 1U; }
static void __enable_irq(void) { mask = 0U; }

typedef struct
{
  uint8_t data[8];
  uint8_t len;
} TxCapture_t;

static TxCapture_t s_capture[128];
static uint8_t s_capture_count;
static uint8_t s_tx_hardware_busy;
static void (*s_complete_cb)(UART_HandleTypeDef *);

static HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef *uart,
                                                    int callback_id,
                                                    void (*callback)(UART_HandleTypeDef *))
{
  assert(uart == &huart4);
  assert(callback_id == HAL_UART_TX_COMPLETE_CB_ID);
  s_complete_cb = callback;
  return HAL_OK;
}

static HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                              const uint8_t *data,
                                              uint16_t len)
{
  assert(uart == &huart4);
  assert(s_tx_hardware_busy == 0U);
  assert(len > 0U && len <= 8U);
  assert(s_capture_count < 128U);
  memcpy(s_capture[s_capture_count].data, data, len);
  s_capture[s_capture_count].len = (uint8_t)len;
  ++s_capture_count;
  s_tx_hardware_busy = 1U;
  return HAL_OK;
}

static void finish_tx(void)
{
  assert(s_tx_hardware_busy != 0U);
  assert(s_complete_cb != NULL);
  s_tx_hardware_busy = 0U;
  s_complete_cb(&huart4);
}

static void drain_to(uint8_t target)
{
  uint32_t guard = 0U;
  while ((s_capture_count < target) || (s_tx_hardware_busy != 0U))
  {
    assert(guard++ < 1000U);
    finish_tx();
  }
  assert(s_capture_count == target);
}

static void check_frame(uint8_t index, const uint8_t *expected, uint8_t len)
{
  assert(index < s_capture_count);
  if (s_capture[index].len != len)
  {
    fprintf(stderr, "frame %u length %u expected %u first=%02X\n",
            (unsigned)index, (unsigned)s_capture[index].len, (unsigned)len,
            (unsigned)s_capture[index].data[0]);
  }
  assert(s_capture[index].len == len);
  assert(memcmp(s_capture[index].data, expected, len) == 0);
}
'''


ZDT_CHECKS = r'''
int main(void)
{
  static const uint8_t enable1[6] = {1U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};
  static const uint8_t enable2[6] = {2U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};
  static const uint8_t enable3[6] = {3U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};
  static const uint8_t enable4[6] = {4U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};
  uint8_t base;

  assert(ZDT_X42S_InitTx() == HAL_OK);
  assert(s_complete_cb != NULL);

  /* 1) Four enable frames are serial and keep 1/2/3/4 order. */
  tick = 0U;
  ZDT_X42S_Enable(1U);
  ZDT_X42S_Enable(2U);
  ZDT_X42S_Enable(3U);
  ZDT_X42S_Enable(4U);
  drain_to(4U);
  check_frame(0U, enable1, sizeof(enable1));
  check_frame(1U, enable2, sizeof(enable2));
  check_frame(2U, enable3, sizeof(enable3));
  check_frame(3U, enable4, sizeof(enable4));

  /* 2) New speed replaces the old queued speed for the same address.
   *    A frame already in flight cannot be replaced, but it cannot be
   *    duplicated either; the remaining frames stay in 1/2/3/4 order. */
  tick = 100U;
  base = s_capture_count;
  ZDT_X42S_SpeedAcc(1U, ZDT_X42S_DIR_CW, 100U, 7U);
  ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 200U, 7U);
  ZDT_X42S_SpeedAcc(3U, ZDT_X42S_DIR_CW, 300U, 7U);
  ZDT_X42S_SpeedAcc(4U, ZDT_X42S_DIR_CW, 400U, 7U);
  ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 250U, 7U);
  drain_to((uint8_t)(base + 4U));
  {
    static const uint8_t speed1[8] = {1U, 0xF6U, 0U, 0U, 100U, 7U, 0U, 0x6BU};
    static const uint8_t speed2[8] = {2U, 0xF6U, 0U, 0U, 250U, 7U, 0U, 0x6BU};
    static const uint8_t speed3[8] = {3U, 0xF6U, 0U, 0x01U, 44U, 7U, 0U, 0x6BU};
    static const uint8_t speed4[8] = {4U, 0xF6U, 0U, 0x01U, 144U, 7U, 0U, 0x6BU};
    check_frame(base, speed1, sizeof(speed1));
    check_frame((uint8_t)(base + 1U), speed2, sizeof(speed2));
    check_frame((uint8_t)(base + 2U), speed3, sizeof(speed3));
    check_frame((uint8_t)(base + 3U), speed4, sizeof(speed4));
  }

  /* 3) STOP has priority over queued normal speeds.  The current frame
   *    is allowed to finish first because aborting it would split a frame. */
  tick = 200U;
  base = s_capture_count;
  ZDT_X42S_SpeedAcc(1U, ZDT_X42S_DIR_CW, 101U, 7U);
  ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 102U, 7U);
  ZDT_X42S_SpeedAcc(3U, ZDT_X42S_DIR_CW, 103U, 7U);
  ZDT_X42S_SpeedAcc(4U, ZDT_X42S_DIR_CW, 104U, 7U);
  ZDT_X42S_Stop(2U);
  drain_to((uint8_t)(base + 4U));
  {
    static const uint8_t stop2[5] = {2U, 0xFEU, 0x98U, 0x00U, 0x6BU};
    check_frame((uint8_t)(base + 1U), stop2, sizeof(stop2));
    assert(s_capture[base + 2U].data[0] == 3U);
    assert(s_capture[base + 3U].data[0] == 4U);
  }

  /* 4) WHEELOFF drops all pending speeds and rejects subsequent speed
   *    frames for the disabled addresses. */
  tick = 300U;
  base = s_capture_count;
  ZDT_X42S_SpeedAcc(1U, ZDT_X42S_DIR_CW, 111U, 7U);
  /* A broadcast speed must not survive a subsequent address-specific
   * WHEELOFF; otherwise it could re-enable the just-disabled motors. */
  ZDT_X42S_SpeedAcc(0U, ZDT_X42S_DIR_CW, 110U, 7U);
  ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 112U, 7U);
  ZDT_X42S_SpeedAcc(3U, ZDT_X42S_DIR_CW, 113U, 7U);
  ZDT_X42S_SpeedAcc(4U, ZDT_X42S_DIR_CW, 114U, 7U);
  ZDT_X42S_Disable(1U);
  ZDT_X42S_Disable(2U);
  ZDT_X42S_Disable(3U);
  ZDT_X42S_Disable(4U);
  drain_to((uint8_t)(base + 5U));
  {
    static const uint8_t disable1[6] = {1U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    static const uint8_t disable2[6] = {2U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    static const uint8_t disable3[6] = {3U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    static const uint8_t disable4[6] = {4U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    check_frame((uint8_t)(base + 1U), disable1, sizeof(disable1));
    check_frame((uint8_t)(base + 2U), disable2, sizeof(disable2));
    check_frame((uint8_t)(base + 3U), disable3, sizeof(disable3));
    check_frame((uint8_t)(base + 4U), disable4, sizeof(disable4));
  }
  {
    uint8_t before = s_capture_count;
    ZDT_X42S_SpeedAcc(1U, ZDT_X42S_DIR_CW, 999U, 7U);
    ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 999U, 7U);
    assert(s_capture_count == before);
    assert(s_tx_hardware_busy == 0U);
  }

  return 0;
}
'''


MECANUM_PRELUDE = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#define ZDT_X42S_MAX_RPM 3000U
#define ZDT_X42S_DIR_CW 0U
#define ZDT_X42S_DIR_CCW 1U

static uint8_t s_addresses[4];
static uint8_t s_call_count;

static void ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
  assert(s_call_count < 4U);
  assert(dir == 0U);
  assert(acc == 0U);
  assert(rpm == (uint16_t)(s_call_count * 10U + 10U));
  s_addresses[s_call_count++] = addr;
}
'''


MECANUM_CHECKS = r'''
int main(void)
{
  SetMotorVoltageAndDirection(10, 20, 30, 40);
  assert(s_call_count == 4U);
  assert(s_addresses[0] == 1U && s_addresses[1] == 2U);
  assert(s_addresses[2] == 3U && s_addresses[3] == 4U);
  return 0;
}
'''


with tempfile.TemporaryDirectory(prefix="ilhc_zdt_tx_") as directory:
    folder = Path(directory)

    zdt_c = folder / "zdt_tx_test.c"
    zdt_exe = folder / "zdt_tx_test.exe"
    zdt_c.write_text(ZDT_PRELUDE + zdt_tx_implementation() + ZDT_CHECKS,
                     encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(zdt_c),
         "-o", str(zdt_exe)],
        check=True,
    )
    subprocess.run([str(zdt_exe)], check=True)

    motor_c = folder / "mecanum_tx_test.c"
    motor_exe = folder / "mecanum_tx_test.exe"
    motor_c.write_text(MECANUM_PRELUDE + mecanum_functions() + MECANUM_CHECKS,
                       encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(motor_c),
         "-o", str(motor_exe)],
        check=True,
    )
    subprocess.run([str(motor_exe)], check=True)

assert "HAL_Delay(1U)" not in mecanum_source
print("ZDT UART4 TX queue and four-wheel submit tests passed")
