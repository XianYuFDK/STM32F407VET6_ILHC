"""编译真实 UART4 非阻塞发送队列，验证组帧、覆盖、优先级、失败上报与恢复。

整段 TX 区域（底层参数 → TX 完成回调 → 对外接口）按注释标记原样切出编译，
所以断言的是固件真正会跑的代码，不是复刻品。

手工模拟 TX 完成中断和 TIM7 毫秒中断推进队列：HAL_UART_Transmit_IT 只记录
字节并把 gState 置忙，由测试调用完成回调收尾并验证帧间空闲。

失败上报是本次改造的安全契约：改成非阻塞后，上层仍靠
ZDT_X42S_GetTxErrorCount() 的变化锁存底盘故障，因此这里必须能证明
"帧最终没发出去"会递增计数。
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "Hardware/zdt_x42s.c").read_text(encoding="utf-8")
MAIN = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")

BEGIN = "/* --------------------------- 底层参数"
END = "/* UART4应答接收："
assert BEGIN in SOURCE and END in SOURCE, "TX 区域标记缺失"
region = SOURCE[SOURCE.index(BEGIN):SOURCE.index(END)]


def strip_comments(text):
    """结构守卫只看代码：注释里可以讨论 HAL_Delay，代码里不行。"""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


code_only = strip_comments(region)

# 结构守卫：不得退回阻塞发送，也不得有帧间延时。
assert "HAL_Delay" not in code_only, "TX 路径不应出现 HAL_Delay"
assert "HAL_UART_Transmit(" not in code_only, "TX 路径不应退回阻塞式 HAL_UART_Transmit"
assert "HAL_UART_Transmit_IT(" in code_only, "TX 路径应使用中断发送"
assert "HAL_IncTick();\n    ZDT_X42S_TxTick();" in MAIN, "TIM7 必须推进帧间空闲"

prelude = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>

typedef int HAL_StatusTypeDef;
typedef struct { int gState; } UART_HandleTypeDef;

#define HAL_OK      0
#define HAL_ERROR   1
#define HAL_BUSY    2
#define HAL_TIMEOUT 3
#define HAL_UART_STATE_READY 0
#define HAL_UART_STATE_BUSY_TX 1
#define HAL_UART_TX_COMPLETE_CB_ID 0

/* 头文件里提供、但不在被切出的 TX 区域内的宏 */
#define ZDT_X42S_DIR_CW     0U
#define ZDT_X42S_DIR_CCW    1U
#define ZDT_X42S_MAX_RPM    3000U
#define ZDT_X42S_DEFAULT_ACC 8U

/* 头文件提供的原型：区域内 ZDT_X42S_Speed 会先调用 SpeedAcc */
HAL_StatusTypeDef ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc);

static UART_HandleTypeDef huart4;
static uint32_t tick, mask;
static unsigned it_calls, abort_calls, cb_registered;
static uint8_t  wire[64];
static unsigned wire_len;

static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask = 1U; }
static void __enable_irq(void) { mask = 0U; }

typedef void (*uart_cb_t)(UART_HandleTypeDef *);
static HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef *u, int id, uart_cb_t cb)
{
  (void)u; (void)id; (void)cb;
  ++cb_registered;
  return HAL_OK;
}

static HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,
                                              const uint8_t *data, uint16_t len)
{
  unsigned i;
  if (u->gState != HAL_UART_STATE_READY) return HAL_BUSY;
  assert(len <= 8U);
  for (i = 0U; i < len; ++i) wire[wire_len++] = data[i];
  ++it_calls;
  u->gState = HAL_UART_STATE_BUSY_TX;
  return HAL_OK;
}

static HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{
  ++abort_calls;
  u->gState = HAL_UART_STATE_READY;
  return HAL_OK;
}
'''

# 队列推进助手放在 TX 区域之后：它们要调用区域里定义的 static 完成回调。
drivers = r'''
/* 测试辅助：模拟一帧发完（真实 HAL 先置 READY 再回调），跨毫秒排空队列。 */
static void reset_capture(void)
{
  wire_len = 0U;
  it_calls = 0U;
  huart4.gState = HAL_UART_STATE_READY;   /* 每段从空闲总线开始 */
  s_tx_gap_pending = 0U;
}
static void complete_tx(void)
{
  huart4.gState = HAL_UART_STATE_READY;
  ZDT_X42S_TxCompleteCallback(&huart4);
}
static void drain_tx(void)
{
  unsigned guard = 0U;
  while ((huart4.gState != HAL_UART_STATE_READY || s_tx_count != 0U) && (guard++ < 64U))
  {
    if (huart4.gState != HAL_UART_STATE_READY) complete_tx();
    tick += ZDT_X42S_TX_GAP_MS;
    ZDT_X42S_TxTick();
  }
  assert(guard < 64U);
}
'''

checks = r'''
int main(void)
{
  unsigned i, before;

  /* InitTx 之前提交必须被拒绝且计入失败，不能静默丢弃。 */
  s_tx_ready = 0U;
  assert(ZDT_X42S_Enable(1U) == HAL_ERROR);
  assert(ZDT_X42S_GetTxErrorCount() == 1U);

  assert(ZDT_X42S_InitTx() == HAL_OK);
  assert(cb_registered == 1U);
  assert(ZDT_X42S_GetTxErrorCount() == 0U);   /* InitTx 复位计数 */

  /* 使能帧：入队即提交第一帧，逐字节比对。 */
  reset_capture();
  assert(ZDT_X42S_Enable(1U) == HAL_OK);
  assert(it_calls == 1U && wire_len == 6U);
  assert(wire[0] == 1U && wire[1] == 0xF3U && wire[2] == 0xABU &&
         wire[3] == 0x01U && wire[4] == 0x00U && wire[5] == 0x6BU);
  complete_tx();
  assert(it_calls == 1U);                      /* 队列已空，不再提交 */

  /* 速度帧按地址覆盖：同地址只发最后一条，不积压旧速度。 */
  reset_capture();
  assert(ZDT_X42S_Enable(1U) == HAL_OK);       /* 先占线，后续都进队列 */
  assert(ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 100U, 0U) == HAL_OK);
  assert(ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 200U, 0U) == HAL_OK);  /* 覆盖 */
  assert(ZDT_X42S_SpeedAcc(4U, ZDT_X42S_DIR_CW, 400U, 0U) == HAL_OK);
  drain_tx();                                  /* 2 条速度帧，不是 3 条 */
  /* 帧序：使能(6B)、2/200(8B)、4/400(8B) */
  assert(wire_len == 22U);
  assert(wire[6] == 2U && wire[9] == 0U && wire[10] == 0xC8U);   /* 2 号 200RPM */
  assert(wire[14] == 4U && wire[17] == 0x01U && wire[18] == 0x90U); /* 4 号 400RPM */

  /* 失能清掉同地址的旧速度，且在其他地址的速度之前发送。 */
  reset_capture();
  assert(ZDT_X42S_Enable(1U) == HAL_OK);       /* 占线 */
  assert(ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CW, 100U, 0U) == HAL_OK);
  assert(ZDT_X42S_SpeedAcc(3U, ZDT_X42S_DIR_CW, 100U, 0U) == HAL_OK);
  assert(ZDT_X42S_Disable(2U) == HAL_OK);
  drain_tx();
  assert(wire_len == 20U);                     /* 旧的 2 号速度必须消失 */
  assert(wire[6] == 2U && wire[7] == 0xF3U && wire[8] == 0xABU && wire[9] == 0x00U);
  assert(wire[12] == 3U && wire[13] == 0xF6U);

  /* WHEELOFF 路径会先排四个零速，再排四个失能；失能后不准残留零速。 */
  reset_capture();
  assert(ZDT_X42S_Enable(1U) == HAL_OK);       /* 占线 */
  for (i = 1U; i <= 4U; ++i)
    assert(ZDT_X42S_SpeedAcc((uint8_t)i, ZDT_X42S_DIR_CW, 0U, 0U) == HAL_OK);
  for (i = 1U; i <= 4U; ++i)
    assert(ZDT_X42S_Disable((uint8_t)i) == HAL_OK);
  drain_tx();
  assert(wire_len == 30U);                     /* 1 个使能 + 4 个失能 */
  for (i = 0U; i < wire_len; ++i) assert(wire[i] != 0xF6U);

  /* 上一帧 TC 后须留出至少一个完整毫秒的空闲，不阻塞任务。 */
  reset_capture();
  tick = 3000U;
  assert(ZDT_X42S_Enable(1U) == HAL_OK);
  assert(ZDT_X42S_Enable(2U) == HAL_OK);
  complete_tx();
  assert(it_calls == 1U);
  tick = 3001U;
  ZDT_X42S_TxTick();
  assert(it_calls == 1U);
  tick = 3002U;
  ZDT_X42S_TxTick();
  assert(it_calls == 2U);
  complete_tx();

  /* 队列满：返回 HAL_ERROR 且计入失败（安全帧不覆盖、不静默丢弃）。 */
  reset_capture();
  assert(ZDT_X42S_InitTx() == HAL_OK);
  assert(ZDT_X42S_Enable(1U) == HAL_OK);       /* 占线 */
  for (i = 0U; i < ZDT_X42S_TX_QUEUE_SIZE; ++i)
  {
    assert(ZDT_X42S_Stop((uint8_t)(i + 1U)) == HAL_OK);
  }
  before = ZDT_X42S_GetTxErrorCount();
  assert(ZDT_X42S_Stop(200U) == HAL_ERROR);
  assert(ZDT_X42S_GetTxErrorCount() == before + 1U);

  /* 在途超时：只中止 TX；超过重试预算则计一次失败并丢弃该帧。 */
  assert(ZDT_X42S_InitTx() == HAL_OK);
  reset_capture();
  abort_calls = 0U;
  tick = 1000U;
  assert(ZDT_X42S_Enable(1U) == HAL_OK);       /* gState=BUSY_TX，在途 */
  tick = 1009U;
  ZDT_X42S_ServiceTx();
  assert(abort_calls == 0U);                   /* 未到单帧上限，不打断 */
  tick = 1010U;
  ZDT_X42S_ServiceTx();
  assert(abort_calls == 1U);                   /* 只中止 UART4 的 TX */
  assert(ZDT_X42S_GetTxErrorCount() == 0U);    /* 还没到重试预算 */
  tick = 2010U;
  ZDT_X42S_ServiceTx();
  assert(ZDT_X42S_GetTxErrorCount() == 1U);    /* 预算耗尽 → 上报失败 */
  i = it_calls;
  tick = 2110U;
  ZDT_X42S_ServiceTx();
  assert(it_calls == i);                       /* 帧已丢弃，不再无限重试 */

  /* 正常帧不会被误判：完成中断推进后队列继续流动，也不会触发看门狗。 */
  assert(ZDT_X42S_InitTx() == HAL_OK);
  reset_capture();
  abort_calls = 0U;
  tick = 5000U;
  assert(ZDT_X42S_SpeedAcc(1U, ZDT_X42S_DIR_CW, 50U, 0U) == HAL_OK);
  assert(ZDT_X42S_SpeedAcc(2U, ZDT_X42S_DIR_CCW, 60U, 0U) == HAL_OK);
  drain_tx();                                  /* 由毫秒中断跨帧推进 */
  assert(it_calls == 2U && abort_calls == 0U);
  tick = 5020U;                                /* 下一个周期总线已空闲 */
  ZDT_X42S_ServiceTx();
  assert(abort_calls == 0U);
  assert(ZDT_X42S_GetTxErrorCount() == 0U);

  puts("ZDT UART4 non-blocking TX queue tests passed");
  return 0;
}
'''

code = prelude + region + drivers + checks

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

print("ZDT TX 非阻塞队列测试通过")
