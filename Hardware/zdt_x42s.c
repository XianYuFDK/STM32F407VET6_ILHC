/**
 ******************************************************************************
 * @file    zdt_x42s.c
 * @brief   张大头 ZDT_X42S 第二代闭环步进电机驱动（硬件层）
 *
 *          指令格式参考《ZDT_X42S第二代闭环步进电机用户手册V1.0.4》：
 *          - 使能：地址 0xF3 0xAB 状态 同步 0x6B
 *          - 停止：地址 0xFE 0x98 同步 0x6B
 *          - 速度：地址 0xF6 方向 速度H 速度L 加速度 同步 0x6B
 *          - 校验尾固定 0x6B
 *
 *          波特率默认 115200，使用 UART4（PA0=TX，PA1=RX）。
 ******************************************************************************
 */
#include "zdt_x42s.h"
#include "usart.h"

/* --------------------------- 底层参数 ----------------------------- */
#define ZDT_X42S_UART                  huart4
#define ZDT_X42S_FRAME_TAIL            0x6BU
#define ZDT_X42S_TX_QUEUE_SIZE         16U    /* 未开始发送的帧上限 */
#define ZDT_X42S_TX_FRAME_MAX          8U     /* 最长帧：速度模式 8 字节 */
#define ZDT_X42S_TX_GAP_MS             2U     /* 毫秒时基下保证至少 1ms 帧间空闲 */
#define ZDT_X42S_TX_SLOT_TIMEOUT_MS    10U    /* 单帧在途上限 */
#define ZDT_X42S_TX_RETRY_BUDGET_MS    1000U  /* 同一帧的累计重试预算 */

/* --------------------------- TX 队列 ------------------------------ */

/*
 * UART4 由四个 ZDT 电机共用，必须严格串行。控制任务只把帧压入固定长度队列，
 * 由 HAL_UART_Transmit_IT 的完成回调记录 TC，再由 TIM7 毫秒中断跨过帧间空闲
 * 后启动下一帧，因此轮速下发不阻塞控制周期。
 *
 * 队列只保存"尚未开始发送"的帧，正在发送的那一帧放在 s_tx_active 里，所以
 * 同一时刻只有一个帧占用 UART4。普通速度帧按地址覆盖未发送的旧帧（最新目标
 * 优先，避免积压旧速度）；使能/失能/停止帧不参与覆盖，并且始终优先于速度帧。
 * 停止/失能还清除同地址尚未发送的旧速度，防止安全帧之后重新启动。
 *
 * 选择 IT 而不是 TX DMA：UART4 已有接收中断和 IRQ 入口，增加 TX DMA 还要再配
 * 通道与中断；单帧只有 5~8 字节，IT 完成回调足以保证串行且不阻塞任务。
 *
 * 失败上报（安全契约，不能因为改成非阻塞就丢掉）：本文件把"帧未能在限期内
 * 发完"折算成 s_tx_error_count 递增，供 debug_usart.c 的 Debug_ServiceWheelFault
 * 锁存故障（取消运动 + 尝试失能 + 等显式 WHEELEN）。判定只有两个来源，都在
 * 任务上下文完成，因此 s_tx_error_count 只被任务读写，不需要 ISR 同步：
 *   1) 单帧占用总线超过 ZDT_X42S_TX_SLOT_TIMEOUT_MS（正常 8 字节 @115200 只需
 *      0.7ms，由完成中断收尾）；
 *   2) 同一帧反复提交不出去，累计超过 ZDT_X42S_TX_RETRY_BUDGET_MS。
 * 刻意不把 UART 错误标志当成发送失败：过载(ORE)等错误是接收侧的，总线噪声
 * 不应该把整车锁存成故障。
 */
typedef enum
{
  ZDT_X42S_TX_SPEED = 0U,   /* 普通速度帧：同地址可被更新目标覆盖 */
  ZDT_X42S_TX_SAFETY        /* 使能/失能/停止：不覆盖，且优先发送 */
} ZDT_X42S_TxKind_t;

typedef struct
{
  uint8_t data[ZDT_X42S_TX_FRAME_MAX];
  uint8_t len;
  uint8_t addr;
  uint8_t kind;
} ZDT_X42S_TxFrame_t;

static ZDT_X42S_TxFrame_t s_tx_queue[ZDT_X42S_TX_QUEUE_SIZE];
static uint8_t  s_tx_count;
static ZDT_X42S_TxFrame_t s_tx_active;      /* 已取出、尚未发送成功的那一帧 */
static volatile uint8_t s_tx_active_valid;
static uint8_t  s_tx_ready;                 /* InitTx 完成后才允许提交 */
static uint32_t s_tx_active_tick;           /* 当前 active 帧首次尝试时刻（重试预算起点） */
static uint32_t s_tx_busy_tick;             /* 本帧开始占用总线的时刻 */
static volatile uint8_t s_tx_gap_pending;
static uint32_t s_tx_gap_tick;              /* 上一帧 TC 的时刻 */
static uint32_t s_tx_error_count;           /* 仅任务上下文读写 */

static void ZDT_X42S_TxKick(void);
static void ZDT_X42S_TxCompleteCallback(UART_HandleTypeDef *huart);

/* --------------------------- 私有函数 ----------------------------- */

static void ZDT_X42S_TxRemoveAtLocked(uint8_t index)
{
  uint8_t i;

  for (i = index; (uint8_t)(i + 1U) < s_tx_count; ++i)
  {
    s_tx_queue[i] = s_tx_queue[i + 1U];
  }
  if (s_tx_count > 0U) --s_tx_count;
}

/* 队首选择：安全帧优先，同类内部保持提交顺序。 */
static uint8_t ZDT_X42S_TxSelectLocked(uint8_t *index)
{
  uint8_t i;

  for (i = 0U; i < s_tx_count; ++i)
  {
    if (s_tx_queue[i].kind != ZDT_X42S_TX_SPEED)
    {
      *index = i;
      return 1U;
    }
  }
  if (s_tx_count > 0U)
  {
    *index = 0U;
    return 1U;
  }
  return 0U;
}

/* 入队。返回 1 表示已接受（含被同地址速度帧覆盖），0 表示被拒绝。
 * 提交方是任务上下文（含调度器启动前的 main），消费方是 TX 完成中断，
 * 因此这里用短临界区保护队列。 */
static uint8_t ZDT_X42S_TxPush(const ZDT_X42S_TxFrame_t *frame)
{
  uint8_t i;
  uint8_t result = 0U;
  uint32_t mask;

  if ((frame == NULL) || (frame->len == 0U) || (frame->len > ZDT_X42S_TX_FRAME_MAX))
  {
    ++s_tx_error_count;
    return 0U;
  }
  if (s_tx_ready == 0U)
  {
    /* InitTx 之前提交：不静默丢弃，按发送失败上报，让次序错误暴露出来。 */
    ++s_tx_error_count;
    return 0U;
  }

  mask = __get_PRIMASK();
  __disable_irq();

  /* 停止/失能必须清除尚未发送的旧速度，否则安全帧优先出队后，旧速度
   * 仍会跟在后面发出，使驱动器再次启动或锁轴。 */
  if ((frame->kind == ZDT_X42S_TX_SAFETY) &&
      ((frame->data[1] == 0xFEU) ||
       ((frame->data[1] == 0xF3U) && (frame->data[3] == 0U))))
  {
    i = 0U;
    while (i < s_tx_count)
    {
      if ((s_tx_queue[i].kind == ZDT_X42S_TX_SPEED) &&
          ((frame->addr == 0U) || (s_tx_queue[i].addr == frame->addr)))
      {
        ZDT_X42S_TxRemoveAtLocked(i);
      }
      else ++i;
    }
  }

  if (frame->kind == ZDT_X42S_TX_SPEED)
  {
    for (i = 0U; i < s_tx_count; ++i)
    {
      if ((s_tx_queue[i].kind == ZDT_X42S_TX_SPEED) &&
          (s_tx_queue[i].addr == frame->addr))
      {
        s_tx_queue[i] = *frame;
        result = 1U;
        break;
      }
    }
  }

  if ((result == 0U) && (s_tx_count < ZDT_X42S_TX_QUEUE_SIZE))
  {
    s_tx_queue[s_tx_count] = *frame;
    ++s_tx_count;
    result = 1U;
  }

  if (mask == 0U) __enable_irq();

  if (result == 0U)
  {
    /* 队列满：帧被丢弃。轮速积压到满说明发送侧已经卡住，按失败上报。 */
    ++s_tx_error_count;
  }
  return result;
}

/* 取出并提交下一帧。任务与 TX 完成中断都会调用，必须保证同一时刻只有
 * 一帧在途：s_tx_active_valid 表示"已取出但还没发出去"。 */
static void ZDT_X42S_TxKick(void)
{
  uint8_t index;
  uint32_t mask;

  if (s_tx_ready == 0U) return;

  mask = __get_PRIMASK();
  __disable_irq();
  if ((s_tx_gap_pending != 0U) &&
      ((uint32_t)(HAL_GetTick() - s_tx_gap_tick) < ZDT_X42S_TX_GAP_MS))
  {
    if (mask == 0U) __enable_irq();
    return;
  }
  s_tx_gap_pending = 0U;
  if (s_tx_active_valid == 0U)
  {
    if (ZDT_X42S_TxSelectLocked(&index) == 0U)
    {
      if (mask == 0U) __enable_irq();
      return;
    }
    s_tx_active = s_tx_queue[index];
    ZDT_X42S_TxRemoveAtLocked(index);
    s_tx_active_valid = 1U;
    s_tx_active_tick = HAL_GetTick();
  }
  if (mask == 0U) __enable_irq();

  /* HAL 未接受时保留 s_tx_active_valid，由后续周期重试或由重试预算判失败。 */
  if (HAL_UART_Transmit_IT(&ZDT_X42S_UART, s_tx_active.data, s_tx_active.len) == HAL_OK)
  {
    s_tx_busy_tick = HAL_GetTick();
  }
}

/* 组帧、入队并尝试提交。返回 HAL_OK 只表示"已接受进队列"，
 * 不代表电机收到，也不代表 HAL 已经发出去；发送结果看
 * ZDT_X42S_GetTxErrorCount()。 */
static HAL_StatusTypeDef ZDT_X42S_TxSubmit(const uint8_t *cmd, uint8_t len,
                                           uint8_t addr, uint8_t kind)
{
  ZDT_X42S_TxFrame_t frame;
  uint8_t i;

  if (cmd == NULL) return HAL_ERROR;

  frame.len = len;
  frame.addr = addr;
  frame.kind = kind;
  for (i = 0U; i < ZDT_X42S_TX_FRAME_MAX; ++i) frame.data[i] = 0U;
  for (i = 0U; (i < len) && (i < ZDT_X42S_TX_FRAME_MAX); ++i) frame.data[i] = cmd[i];

  if (ZDT_X42S_TxPush(&frame) == 0U) return HAL_ERROR;

  ZDT_X42S_TxKick();
  return HAL_OK;
}

/* ------------------------- 任务级服务接口 -------------------------- */

/* 初始化 UART4 非阻塞发送：注册 TX 完成回调并复位队列。
 * 必须在 MX_UART4_Init 之后、提交任何电机命令之前调用。 */
HAL_StatusTypeDef ZDT_X42S_InitTx(void)
{
  s_tx_count = 0U;
  s_tx_active_valid = 0U;
  s_tx_busy_tick = 0U;
  s_tx_gap_pending = 0U;
  s_tx_gap_tick = 0U;
  s_tx_active_tick = 0U;
  s_tx_error_count = 0U;
  s_tx_ready = 0U;

  if (HAL_UART_RegisterCallback(&ZDT_X42S_UART, HAL_UART_TX_COMPLETE_CB_ID,
                                ZDT_X42S_TxCompleteCallback) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_tx_ready = 1U;
  return HAL_OK;
}

/* TIM7 的 1ms 中断推进帧间空闲期。仅尝试启动下一帧，不执行超时判定，
 * 因而失败计数仍只由任务上下文读写。启动阶段尚无默认任务，也依赖此入口。 */
void ZDT_X42S_TxTick(void)
{
  if ((s_tx_ready != 0U) && (s_tx_gap_pending != 0U)) ZDT_X42S_TxKick();
}

/* 默认任务周期调用。只处理"卡住"和"重试预算"，不阻塞、不主动等总线：
 * - 正常一帧 0.7ms 就由完成中断收尾，若本周期开始时仍非 READY 且已超过
 *   单帧上限，说明提交卡住，只中止 UART4 的 TX（不碰 RX 和命令接收）；
 * - 同一帧反复提交不出去时给出结论：计一次发送失败并丢弃该帧。丢弃而不是
 *   无限重试，避免它永久占住队首，把后面的使能/失能安全帧一起堵死。 */
void ZDT_X42S_ServiceTx(void)
{
  uint32_t now;
  uint32_t mask;

  if (s_tx_ready == 0U) return;

  now = HAL_GetTick();

  mask = __get_PRIMASK();
  __disable_irq();
  if (ZDT_X42S_UART.gState != HAL_UART_STATE_READY)
  {
    if ((uint32_t)(now - s_tx_busy_tick) >= ZDT_X42S_TX_SLOT_TIMEOUT_MS)
      (void)HAL_UART_AbortTransmit(&ZDT_X42S_UART);
    else
    {
      if (mask == 0U) __enable_irq();
      return;
    }
  }

  if ((s_tx_active_valid != 0U) &&
      ((uint32_t)(now - s_tx_active_tick) >= ZDT_X42S_TX_RETRY_BUDGET_MS))
  {
    s_tx_active_valid = 0U;
    ++s_tx_error_count;
  }
  if (mask == 0U) __enable_irq();

  ZDT_X42S_TxKick();
}

/* TX 完成回调：本帧已离开 UART4，清掉在途标记并启动帧间空闲计时。
 * 必须在这里清 s_tx_active_valid——否则 TxKick 会认为该帧还没发出去，
 * 把同一帧反复重发。
 * 调度器启动前（main 里首次停车/使能）由 TIM7 毫秒中断继续发送；
 * 回调里的 TxKick 会看到空闲期未满并立即返回。 */
static void ZDT_X42S_TxCompleteCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
  s_tx_active_valid = 0U;
  s_tx_gap_tick = HAL_GetTick();
  s_tx_gap_pending = 1U;
  ZDT_X42S_TxKick();
}

/* --------------------------- 对外接口 ----------------------------- */

/**
 * @brief  使能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @retval HAL_OK 已入队；HAL_ERROR 未接受（未初始化/队列满/参数非法）
 */
HAL_StatusTypeDef ZDT_X42S_Enable(uint8_t addr)
{
  uint8_t cmd[6];

  cmd[0] = addr;       /* 地址       */
  cmd[1] = 0xF3U;      /* 功能码     */
  cmd[2] = 0xABU;      /* 辅助码     */
  cmd[3] = 0x01U;      /* 使能状态   */
  cmd[4] = 0x00U;      /* 同步标志   */
  cmd[5] = ZDT_X42S_FRAME_TAIL;

  return ZDT_X42S_TxSubmit(cmd, (uint8_t)sizeof(cmd), addr, ZDT_X42S_TX_SAFETY);
}

/**
 * @brief  失能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @retval HAL_OK 已入队；HAL_ERROR 未接受
 */
HAL_StatusTypeDef ZDT_X42S_Disable(uint8_t addr)
{
  uint8_t cmd[6];

  cmd[0] = addr;       /* 地址       */
  cmd[1] = 0xF3U;      /* 功能码     */
  cmd[2] = 0xABU;      /* 辅助码     */
  cmd[3] = 0x00U;      /* 使能状态   */
  cmd[4] = 0x00U;      /* 同步标志   */
  cmd[5] = ZDT_X42S_FRAME_TAIL;

  return ZDT_X42S_TxSubmit(cmd, (uint8_t)sizeof(cmd), addr, ZDT_X42S_TX_SAFETY);
}

/**
 * @brief  立即停止指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @retval HAL_OK 已入队；HAL_ERROR 未接受
 */
HAL_StatusTypeDef ZDT_X42S_Stop(uint8_t addr)
{
  uint8_t cmd[5];

  cmd[0] = addr;       /* 地址       */
  cmd[1] = 0xFEU;      /* 功能码     */
  cmd[2] = 0x98U;      /* 辅助码     */
  cmd[3] = 0x00U;      /* 同步标志   */
  cmd[4] = ZDT_X42S_FRAME_TAIL;

  return ZDT_X42S_TxSubmit(cmd, (uint8_t)sizeof(cmd), addr, ZDT_X42S_TX_SAFETY);
}

/**
 * @brief  速度模式控制电机连续转动（使用默认加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 */
HAL_StatusTypeDef ZDT_X42S_Speed(uint8_t addr, uint8_t dir, uint16_t rpm)
{
  return ZDT_X42S_SpeedAcc(addr, dir, rpm, ZDT_X42S_DEFAULT_ACC);
}

/**
 * @brief  速度模式控制电机连续转动（自定义加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 * @param  acc  加速度档位，0~255；0 为直接启动
 * @retval HAL_OK 已入队；HAL_ERROR 未接受
 */
HAL_StatusTypeDef ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
  uint8_t cmd[8];

  if (rpm > ZDT_X42S_MAX_RPM)
  {
    rpm = ZDT_X42S_MAX_RPM;
  }

  cmd[0] = addr;                          /* 地址         */
  cmd[1] = 0xF6U;                         /* 功能码       */
  cmd[2] = dir;                           /* 方向         */
  cmd[3] = (uint8_t)(rpm >> 8);           /* 速度高8位    */
  cmd[4] = (uint8_t)(rpm & 0xFFU);        /* 速度低8位    */
  cmd[5] = acc;                           /* 加速度档位   */
  cmd[6] = 0x00U;                         /* 同步标志     */
  cmd[7] = ZDT_X42S_FRAME_TAIL;           /* 校验尾       */

  return ZDT_X42S_TxSubmit(cmd, (uint8_t)sizeof(cmd), addr, ZDT_X42S_TX_SPEED);
}

uint32_t ZDT_X42S_GetTxErrorCount(void)
{
  return s_tx_error_count;
}

/* UART4应答接收：中断只做四字节滑窗和入队，不打印、不发送运动命令。
 * 帧间断开超过20ms即丢弃残帧；噪声采用逐字节滑动重新同步。
 * 6B是固定校验字节而非CRC，因此有效格式不等于物理运动已完成。 */
static uint8_t s_rx_byte, s_rx_window[4], s_rx_count;
static uint8_t s_reply_queue[16][4];
static volatile uint8_t s_reply_read, s_reply_write, s_rx_fault;
static uint32_t s_rx_tick;

static void ZDT_X42S_RxComplete(UART_HandleTypeDef *uart)
{
  uint8_t i, next;
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - s_rx_tick) > 20U) s_rx_count = 0U;
  s_rx_tick = now;
  if (s_rx_count == 4U)
  {
    for (i = 0U; i < 3U; ++i) s_rx_window[i] = s_rx_window[i + 1U];
    s_rx_count = 3U;
  }
  if (s_rx_count >= 4U)
  {
    /* 防御：窗口索引越界保护。正常流程由"满窗搬移 + 接收成功清零"维持
     * s_rx_count ∈ {0,1,2,3}，这里再兜底一次，任何异常路径都不会写越界。 */
    s_rx_count = 0U;
  }
  s_rx_window[s_rx_count++] = s_rx_byte;
  if (s_rx_count == 4U && s_rx_window[0] != 0U &&
      (s_rx_window[1] == 0xF3U || s_rx_window[1] == 0xF6U || s_rx_window[1] == 0xFEU) &&
      s_rx_window[3] == 0x6BU)
  {
    next = (uint8_t)((s_reply_write + 1U) % 16U);
    if (next != s_reply_read)
    {
      for (i = 0U; i < 4U; ++i) s_reply_queue[s_reply_write][i] = s_rx_window[i];
      s_reply_write = next;
    }
    s_rx_count = 0U;
  }
  if (HAL_UART_Receive_IT(uart, &s_rx_byte, 1U) != HAL_OK) s_rx_fault = 1U;
}

static void ZDT_X42S_RxError(UART_HandleTypeDef *uart)
{
  (void)uart;
  /* 奇偶、帧、噪声或溢出错误交由任务恢复，避免在中断里阻塞。 */
  s_rx_fault = 1U;
}

HAL_StatusTypeDef ZDT_X42S_InitRx(void)
{
  s_rx_count = s_reply_read = s_reply_write = 0U;
  s_rx_fault = 1U;
  if (HAL_UART_RegisterCallback(&ZDT_X42S_UART, HAL_UART_RX_COMPLETE_CB_ID,
                                ZDT_X42S_RxComplete) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_RegisterCallback(&ZDT_X42S_UART, HAL_UART_ERROR_CB_ID,
                                ZDT_X42S_RxError) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_Receive_IT(&ZDT_X42S_UART, &s_rx_byte, 1U) != HAL_OK) return HAL_ERROR;
  s_rx_fault = 0U;
  return HAL_OK;
}

void ZDT_X42S_ServiceRx(void)
{
  if (s_rx_fault)
  {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* UART4没有RX DMA，AbortReceive只关闭接收并复位状态，不等待DMA。 */
    (void)HAL_UART_AbortReceive(&ZDT_X42S_UART);
    __HAL_UART_CLEAR_OREFLAG(&ZDT_X42S_UART);
    s_rx_count = 0U;
    if (HAL_UART_Receive_IT(&ZDT_X42S_UART, &s_rx_byte, 1U) == HAL_OK) s_rx_fault = 0U;
    if (mask == 0U) __enable_irq();
  }
}

uint8_t ZDT_X42S_PopReply(uint8_t reply[4])
{
  uint8_t i;
  uint32_t mask;
  if (reply == NULL) return 0U;
  mask = __get_PRIMASK();
  __disable_irq();
  if (s_reply_read == s_reply_write)
  {
    if (mask == 0U) __enable_irq();
    return 0U;
  }
  for (i = 0U; i < 4U; ++i) reply[i] = s_reply_queue[s_reply_read][i];
  s_reply_read = (uint8_t)((s_reply_read + 1U) % 16U);
  if (mask == 0U) __enable_irq();
  return 1U;
}
