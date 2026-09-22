"""编译真实 Debug_ReplyParam，验证 GET 参数回读的文本格式、队列与闸门。

XVMIN/ZVMIN 在24通道遥测里没有通道位，上位机的"回读"栏只能靠这条文字应答；
本测试用桩替换参数表和应答队列，直接跑固件里的回读实现。
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "Hardware/debug_usart.c").read_text(encoding="utf-8")
header = (root / "Hardware/debug_usart.h").read_text(encoding="utf-8")


def function(name):
    start = source.rfind("static ", 0, source.index(name + "("))
    brace = source.index("{", start)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


# 事件号从源码里取，避免测试自带一份会和固件漂移的常量。
unknown_event = re.search(r"#define DEBUG_ACK_PARAM_UNKNOWN (\d+)U", source)
text_event = re.search(r"#define DEBUG_ACK_PARAM_TEXT\s+(\d+)U", source)
assert unknown_event and text_event, "缺少参数回读事件号定义"

prelude = '''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

/* 参数表桩：只保留按名字查表需要的三项，范围与固件一致。 */
static float XYVmax = 1600.0f, XYVmin = 5.0f, ZVmin = 5.0f;

typedef struct
{
  const char *name;
  float      *value;
  float       min;
  float       max;
} DebugParam_t;

static const DebugParam_t s_params[] =
{
  {"XVMAX", &XYVmax, 0.0f, 3000.0f},
  {"XVMIN", &XYVmin, 0.0f, 100.0f},
  {"ZVMIN", &ZVmin,  0.0f, 100.0f},
};

static char s_ack_param[16][24];
static volatile uint8_t s_ack_read, s_ack_write;
static uint8_t s_ack_queue[16];
static uint32_t mask, ack_events, last_ack;
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask = 1U;}
static void __enable_irq(void) {mask = 0U;}
static void Debug_ZdtAck(uint8_t event) {ack_events++; last_ack = event;}
''' + "#define DEBUG_ACK_PARAM_UNKNOWN %sU\n#define DEBUG_ACK_PARAM_TEXT %sU\n" % (
    unknown_event.group(1), text_event.group(1))

checks = r'''
int main(void) {
  /* 1) 基本回读：名称回显大写、固定三位小数、按当前槽位入队 */
  s_ack_write = 0U; s_ack_read = 0U; mask = 0U; ack_events = 0U;
  Debug_ReplyParam("XVMIN");
  assert(strcmp(s_ack_param[0], "XVMIN=5.000\r\n") == 0);
  assert(s_ack_queue[0] == DEBUG_ACK_PARAM_TEXT);
  assert(s_ack_write == 1U);
  assert(ack_events == 0U);
  assert(mask == 0U);                 /* 调用前中断是开的，必须恢复为开 */

  /* 2) 名称忽略大小写，值取参数表指向的实时变量 */
  ZVmin = 7.5f;
  Debug_ReplyParam("zvmin");
  assert(strcmp(s_ack_param[1], "ZVMIN=7.500\r\n") == 0);
  assert(s_ack_write == 2U);

  /* 3) 有遥测通道的参数同样可回读，用于核对波形与RAM值 */
  XYVmax = 1234.5678f;
  Debug_ReplyParam("XVMAX");
  assert(strcmp(s_ack_param[2], "XVMAX=1234.568\r\n") == 0);

  /* 4) 0 与负值：整数部分为 0 也要输出，负号保留 */
  XYVmin = 0.0f;
  Debug_ReplyParam("XVMIN");
  assert(strcmp(s_ack_param[3], "XVMIN=0.000\r\n") == 0);
  ZVmin = -2.5f;
  Debug_ReplyParam("ZVMIN");
  assert(strcmp(s_ack_param[4], "ZVMIN=-2.500\r\n") == 0);

  /* 5) 位数：<1 的值补零、大整数不截断 */
  XYVmin = 0.0049f;
  Debug_ReplyParam("XVMIN");
  assert(strcmp(s_ack_param[5], "XVMIN=0.005\r\n") == 0);
  XYVmax = 3000.0f;
  Debug_ReplyParam("XVMAX");
  assert(strcmp(s_ack_param[6], "XVMAX=3000.000\r\n") == 0);
  assert(s_ack_write == 7U);

  /* 6) 名称非法：只回一条静态ERR，不占文本槽 */
  s_ack_write = 0U; s_ack_read = 0U; ack_events = 0U;
  Debug_ReplyParam("XVMINX");
  Debug_ReplyParam("XVMIN ");
  Debug_ReplyParam("");
  assert(s_ack_write == 0U);
  assert(ack_events == 3U && last_ack == DEBUG_ACK_PARAM_UNKNOWN);

  /* 7) 队列满：丢弃新应答，绝不覆盖尚未发送的槽位 */
  s_ack_write = 5U; s_ack_read = 6U; mask = 0U;
  memset(s_ack_param[5], 'Z', sizeof(s_ack_param[5]));
  Debug_ReplyParam("XVMIN");
  assert(s_ack_write == 5U);
  assert(s_ack_param[5][0] == 'Z' && s_ack_param[5][23] == 'Z');

  /* 8) 关中断上下文：PRIMASK 保持原样，不在临界区内开中断 */
  s_ack_write = 8U; s_ack_read = 10U; mask = 1U;
  Debug_ReplyParam("XVMIN");
  assert(mask == 1U);
  assert(s_ack_write == 9U);
  assert(strcmp(s_ack_param[8], "XVMIN=0.005\r\n") == 0);

  puts("Param readback reply format, queue and gate tests passed");
  return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="ilhc_param_readback_") as directory:
    folder = Path(directory)
    src, exe = folder / "param.c", folder / "param.exe"
    src.write_text(prelude + function("Debug_StrCaseCmp") + function("Debug_ReplyParam") + checks,
                   encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)],
                   check=True)
    subprocess.run([str(exe)], check=True)

# ---------------- 接线核对：命令入口、发送分支、帧格式不变 ----------------

# GET <名称> 必须在 '=' 解析之前，否则 "GET XVMIN" 会被当成无名参数丢弃。
parse = source[source.index("static void Debug_ParseLine("):source.index("static void DebugUsart_ErrorCallback(")]
flat_parse = " ".join(parse.split())
assert 'Debug_StrCaseCmpN(line, "GET ", 4U)' in flat_parse
assert "Debug_ReplyParam(line + 4U);" in flat_parse
assert flat_parse.index('Debug_StrCaseCmpN(line, "GET ", 4U)') < flat_parse.index('strchr(line, \'=\')')

# 发送分支：动态文本走槽位，静态表仍走原分支。
send = source[source.index("void DebugUsart_Send(void)"):]
flat_send = " ".join(send.split())
assert "s_ack_queue[s_ack_read] == DEBUG_ACK_PARAM_TEXT" in flat_send
assert "s_ack_param[s_ack_read]" in flat_send
assert flat_send.index("DEBUG_ACK_PARAM_TEXT") < flat_send.index("s_ack_text[s_ack_queue[s_ack_read]]")
# 应答仍然优先于遥测，且忙时不改写正在发送的 s_tx。
assert flat_send.index("s_ack_read != s_ack_write") < flat_send.index("if (s_zdt_text_mode) return;")

# 被测参数仍在固件参数表里，GET 才能查到；遥测保持 24 通道不变。
assert re.search(r'\{"XVMIN",\s*&XYVmin,\s*0\.0f,\s*100\.0f\}', source)
assert re.search(r'\{"ZVMIN",\s*&ZVmin,\s*0\.0f,\s*100\.0f\}', source)
assert "#define DEBUG_VOFA_CHANNELS   24U" in header
assert "data[24]" not in flat_send and "data[25]" not in flat_send
assert "data[23] = s_dm_torque;" in flat_send

print("Firmware param readback (GET) format, queue, gate and wiring passed")
