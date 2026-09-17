"""Host-side replay test for Hardware/ops.c V1/V2 framing.

The test extracts the real parser functions from ops.c, feeds synthetic byte
streams, and checks validation, resynchronization and session handling.
It does not access hardware.
"""
from pathlib import Path
import re
import struct
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
OPS = (ROOT / "Hardware/ops.c").read_text(encoding="utf-8")

for required in (
    "void OPS_ServiceRx(void)",
    "void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)",
    "#define OPS_RX_BUFFER_SIZE",
):
    if required not in OPS and required not in (
        (ROOT / "Hardware/ops.h").read_text(encoding="utf-8")):
        raise RuntimeError("missing regression guard: " + required)


def function(source, name):
    pattern = (r"^(?:static )?(?:uint8_t|uint16_t|void) " +
               re.escape(name) + r"\(")
    match = re.search(pattern, source, re.M)
    if match is None:
        raise RuntimeError("function not found: " + name)
    start = source.rfind("\n", 0, match.start()) + 1
    brace = source.index("{", match.start())
    level = 1
    end = brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


crc8_match = re.search(
    r"static const uint8_t s_crc8_table\[256\]\s*=\s*\{.*?\};",
    OPS,
    re.S,
)
if crc8_match is None:
    raise RuntimeError("CRC8 table not found")

crc8_table = [int(value, 16) for value in re.findall(
    r"0x([0-9A-Fa-f]{2})", crc8_match.group(0))]
if len(crc8_table) != 256:
    raise RuntimeError("CRC8 table length is %d" % len(crc8_table))


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def crc8(data):
    crc = 0xFF
    for byte in data:
        crc = crc8_table[crc ^ byte]
    return crc


def v2_frame(flags, seq, session, timestamp, x, y, z):
    body = struct.pack(
        "<BBBBHIIfff",
        0x5D, 1, 28, flags, seq, session, timestamp, x, y, z,
    )
    return body + struct.pack("<H", crc16(body))


def v1_frame(x, y, z):
    body = struct.pack("<Bfff", 0x5C, x, y, z)
    return body + bytes([crc8(body)])


def bytes_c(values):
    return ",".join("0x%02X" % value for value in values)


valid_v2 = v2_frame(
    0x01 | 0x02 | 0x08, 7, 0x11223344, 123456, 1.25, -0.5, 0.25)
invalid_v2 = v2_frame(
    0x01 | 0x08, 8, 0x11223344, 123457, 2.0, -1.0, 0.5)
session_v2 = v2_frame(
    0x01 | 0x02 | 0x08, 9, 0x55667788, 123458, 3.5, 4.5, 0.75)
valid_v1 = v1_frame(0.5, 1.0, -0.25)

prelude = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#define OPS_FRAME_LEN_V1 14U
#define OPS_FRAME_LEN_V2 28U
#define OPS_RX_BUFFER_SIZE 64U
#define OPS_FRAME_HEADER_V1 0x5CU
#define OPS_FRAME_HEADER_V2 0x5DU
#define OPS_FRAME_VERSION_V2 0x01U
#define OPS_FLAG_POS_VALID 0x01U
#define OPS_FLAG_IMU_ONLINE 0x02U
#define OPS_FLAG_ENC_VALID 0x08U
#define OPS_STATUS_IDLE 0U
#define OPS_STATUS_OK 1U
#define OPS_STATUS_HEADER_ERR 2U
#define OPS_STATUS_CRC_ERR 3U
#define OPS_STATUS_DATA_ERR 4U

typedef struct {
  uint8_t header, version, length, flags;
  uint16_t seq;
  uint32_t session_id, timestamp_ms;
  float x, y, z;
  uint16_t checksum;
} OPS_Frame_t;

typedef struct {
  OPS_Frame_t frame;
  uint32_t frame_count, valid_count, error_count;
  uint32_t crc_errors, format_errors;
  uint8_t status, pose_valid, session_changed;
  uint16_t seq;
  uint32_t session_id, timestamp_ms;
  uint32_t last_update_tick;
  float origin_x, origin_y;
  uint8_t zero_enabled;
} OPS_Data_t;

static OPS_Data_t s_ops;
static uint8_t s_parse_buf[OPS_RX_BUFFER_SIZE];
static uint16_t s_parse_len;
static uint8_t s_session_pending;
static float s_reference_yaw, s_origin_yaw;
static uint8_t s_new_flag;

static uint32_t HAL_GetTick(void) { return 1234U; }
'''

functions = [
    "OPS_CalcCRC8",
    "OPS_VerifyCRC8",
    "OPS_CalcCRC16",
    "OPS_VerifyCRC16",
    "OPS_DropFirstByte",
    "OPS_FrameValuesValid",
    "OPS_PublishFrame",
    "OPS_PublishV1Frame",
    "OPS_PublishV2Frame",
    "OPS_ParseByte",
]

check = r'''
static void feed(const uint8_t *data, unsigned len)
{
  unsigned i;
  for (i = 0U; i < len; ++i) OPS_ParseByte(data[i]);
}

static void near(float a, float b)
{
  assert(fabsf(a - b) < 0.00001f);
}

int main(void)
{
  static const uint8_t valid_v2[] = { %s };
  static const uint8_t invalid_v2[] = { %s };
  static const uint8_t session_v2[] = { %s };
  static const uint8_t valid_v1[] = { %s };
  const uint8_t noise[] = {0x00U, 0x11U, 0x5DU, 0x00U};
  uint32_t valid_before;

  memset(&s_ops, 0, sizeof(s_ops));
  feed(valid_v2, sizeof(valid_v2));
  assert(s_ops.frame_count == 1U);
  assert(s_ops.valid_count == 1U);
  assert(s_ops.pose_valid == 1U);
  assert(s_ops.session_id == 0x11223344U);
  near(s_ops.frame.x, 1.25f);
  near(s_ops.frame.y, -0.5f);
  near(s_ops.frame.z, 0.25f);

  valid_before = s_ops.valid_count;
  feed(invalid_v2, sizeof(invalid_v2));
  assert(s_ops.frame_count == 2U);
  assert(s_ops.valid_count == valid_before);
  assert(s_ops.pose_valid == 0U);

  feed(valid_v2, sizeof(valid_v2));
  assert(s_ops.pose_valid == 1U);
  assert(s_ops.valid_count == valid_before + 1U);

  feed(noise, sizeof(noise));
  feed(valid_v2, sizeof(valid_v2));
  assert(s_ops.format_errors > 0U);
  assert(s_ops.pose_valid == 1U);

  /* Split a V2 frame across arbitrary callback boundaries. */
  memset(&s_ops, 0, sizeof(s_ops));
  feed(valid_v2, 7U);
  assert(s_ops.frame_count == 0U);
  feed(&valid_v2[7], (unsigned)(sizeof(valid_v2) - 7U));
  assert(s_ops.frame_count == 1U);
  assert(s_ops.pose_valid == 1U);

  /* A CRC failure must not poison the next valid frame. */
  memset(&s_ops, 0, sizeof(s_ops));
  {
    uint8_t bad[sizeof(valid_v2)];
    memcpy(bad, valid_v2, sizeof(bad));
    bad[26] ^= 0x01U;
    feed(bad, sizeof(bad));
    feed(valid_v2, sizeof(valid_v2));
    assert(s_ops.crc_errors == 1U);
    assert(s_ops.pose_valid == 1U);
  }

  /* V1 compatibility remains available. */
  memset(&s_ops, 0, sizeof(s_ops));
  feed(valid_v1, sizeof(valid_v1));
  assert(s_ops.frame_count == 1U);
  assert(s_ops.pose_valid == 1U);
  near(s_ops.frame.x, 0.5f);
  near(s_ops.frame.y, 1.0f);
  near(s_ops.frame.z, -0.25f);

  /* A runtime session change is visible and rebases the local origin. */
  memset(&s_ops, 0, sizeof(s_ops));
  feed(valid_v2, sizeof(valid_v2));
  feed(session_v2, sizeof(session_v2));
  assert(s_ops.session_changed == 1U);
  assert(s_ops.session_id == 0x55667788U);
  near(s_ops.origin_x, 3.5f);
  near(s_ops.origin_y, 4.5f);

  puts("OPS protocol: V2 flags/CRC16, split stream, noise resync, CRC retry, V1 and session passed");
  return 0;
}
''' % (
    bytes_c(valid_v2),
    bytes_c(invalid_v2),
    bytes_c(session_v2),
    bytes_c(valid_v1),
)

code = prelude
code += crc8_match.group(0) + "\n"
code += "\n".join(function(OPS, name) for name in functions)
code += "\n" + check

with tempfile.TemporaryDirectory(prefix="ilhc_ops_protocol_") as directory:
    folder = Path(directory)
    source = folder / "test_ops_protocol.c"
    executable = folder / "test_ops_protocol.exe"
    source.write_text(code, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
         str(source), "-lm", "-o", str(executable)],
        check=True,
    )
    subprocess.run([str(executable)], check=True)
