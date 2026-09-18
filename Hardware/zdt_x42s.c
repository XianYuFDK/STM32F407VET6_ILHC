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
#define ZDT_X42S_TX_QUEUE_SIZE         16U
#define ZDT_X42S_TX_FRAME_MAX          8U
#define ZDT_X42S_ENABLE_READY_MS       100U

/* --------------------------- TX 队列 ------------------------------ */

/*
 * UART4由四个ZDT电机共用，必须严格串行。控制任务只向固定长度队列
 * 提交帧；HAL_UART_Transmit_IT()完成回调再启动下一帧。
 *
 * 队列只保存尚未开始的帧，正在发送的帧放在s_tx_active中，因此不会
 * 出现两帧同时占用UART4。普通速度帧按地址覆盖同一条尚未发送的旧帧，
 * 使能/失能/停止帧不参与覆盖，并且始终优先于普通速度帧。
 *
 * 选择IT而不是DMA：UART4已经有接收中断和IRQ入口，增加TX DMA需要再配置
 * 通道与中断；单帧只有5~8字节，IT完成回调足以保证串行且不阻塞控制任务。
 */
typedef enum
{
  ZDT_X42S_TX_SPEED = 0U,
  ZDT_X42S_TX_ENABLE,
  ZDT_X42S_TX_DISABLE,
  ZDT_X42S_TX_STOP
} ZDT_X42S_TxKind_t;

typedef struct
{
  uint8_t data[ZDT_X42S_TX_FRAME_MAX];
  uint8_t len;
  uint8_t addr;
  uint8_t kind;
  uint32_t not_before_tick;
} ZDT_X42S_TxFrame_t;

static ZDT_X42S_TxFrame_t s_tx_queue[ZDT_X42S_TX_QUEUE_SIZE];
static uint8_t s_tx_queue_count;
static ZDT_X42S_TxFrame_t s_tx_active;
static volatile uint8_t s_tx_active_valid;
static volatile uint8_t s_tx_inflight;
static uint8_t s_tx_initialized;
static uint32_t s_tx_drop_count;

/* 地址0为广播地址；1~4是四个底盘轮电机。失能后拒绝普通速度帧，
 * 避免速度命令把已经释放的驱动器重新使能并锁轴。 */
static uint8_t s_motor_enabled[256];
static uint32_t s_motor_ready_tick[256];

static void ZDT_X42S_TxKick(void);

static void ZDT_X42S_RemoveQueueAtLocked(uint8_t index)
{
  uint8_t i;

  for (i = index; (uint8_t)(i + 1U) < s_tx_queue_count; ++i)
  {
    s_tx_queue[i] = s_tx_queue[i + 1U];
  }
  if (s_tx_queue_count > 0U)
  {
    --s_tx_queue_count;
  }
}

static void ZDT_X42S_RemovePendingSpeedsLocked(uint8_t addr)
{
  uint8_t i = 0U;

  while (i < s_tx_queue_count)
  {
    if ((s_tx_queue[i].kind == ZDT_X42S_TX_SPEED) &&
        ((addr == 0U) || (s_tx_queue[i].addr == addr) ||
         (s_tx_queue[i].addr == 0U)))
    {
      ZDT_X42S_RemoveQueueAtLocked(i);
    }
    else
    {
      ++i;
    }
  }
}

static void ZDT_X42S_RefreshBroadcastStateLocked(void)
{
  uint8_t i;
  uint8_t all_enabled = 1U;
  uint32_t ready_tick = 0U;

  for (i = 1U; i <= 4U; ++i)
  {
    if (s_motor_enabled[i] == 0U)
    {
      all_enabled = 0U;
    }
    if (s_motor_ready_tick[i] > ready_tick)
    {
      ready_tick = s_motor_ready_tick[i];
    }
  }

  s_motor_enabled[0] = all_enabled;
  s_motor_ready_tick[0] = ready_tick;
}

static uint8_t ZDT_X42S_QueuePush(const ZDT_X42S_TxFrame_t *frame)
{
  uint8_t i;
  uint8_t result = 0U;
  uint32_t mask;

  if ((frame == NULL) || (frame->len == 0U) ||
      (frame->len > ZDT_X42S_TX_FRAME_MAX))
  {
    return 0U;
  }

  mask = __get_PRIMASK();
  __disable_irq();

  if ((frame->kind == ZDT_X42S_TX_SPEED) &&
      (s_motor_enabled[frame->addr] == 0U))
  {
    ++s_tx_drop_count;
    if (mask == 0U) __enable_irq();
    return 0U;
  }

  if ((frame->kind == ZDT_X42S_TX_DISABLE) ||
      (frame->kind == ZDT_X42S_TX_STOP))
  {
    ZDT_X42S_RemovePendingSpeedsLocked(frame->addr);
  }

  if (frame->kind == ZDT_X42S_TX_SPEED)
  {
    for (i = 0U; i < s_tx_queue_count; ++i)
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

  if (result == 0U)
  {
    /* 队列满时，安全帧可以挤掉一条尚未发送的普通速度帧；同类型
     * 安全帧和普通速度帧都不允许无界增长。 */
    if ((s_tx_queue_count >= ZDT_X42S_TX_QUEUE_SIZE) &&
        (frame->kind != ZDT_X42S_TX_SPEED))
    {
      for (i = 0U; i < s_tx_queue_count; ++i)
      {
        if (s_tx_queue[i].kind == ZDT_X42S_TX_SPEED)
        {
          ZDT_X42S_RemoveQueueAtLocked(i);
          break;
        }
      }
    }

    if (s_tx_queue_count < ZDT_X42S_TX_QUEUE_SIZE)
    {
      s_tx_queue[s_tx_queue_count] = *frame;
      ++s_tx_queue_count;
      result = 1U;
    }
  }

  if (result == 0U)
  {
    ++s_tx_drop_count;
  }

  if (mask == 0U) __enable_irq();
  return result;
}

static uint8_t ZDT_X42S_TxSelectLocked(uint8_t *index, uint32_t now)
{
  uint8_t i;

  /* 安全帧优先，且保持同一优先级内部的FIFO顺序。 */
  for (i = 0U; i < s_tx_queue_count; ++i)
  {
    if (s_tx_queue[i].kind != ZDT_X42S_TX_SPEED)
    {
      *index = i;
      return 1U;
    }
  }

  /* 速度帧保持1、2、3、4的提交顺序；仅当队首已经到使能等待
   * 时间才发送，避免普通速度积压期间越过等待边界。 */
  for (i = 0U; i < s_tx_queue_count; ++i)
  {
    if (s_tx_queue[i].kind == ZDT_X42S_TX_SPEED)
    {
      if ((int32_t)(now - s_tx_queue[i].not_before_tick) >= 0)
      {
        *index = i;
        return 1U;
      }
      return 0U;
    }
  }

  return 0U;
}

static void ZDT_X42S_TxKick(void)
{
  uint32_t mask;
  uint32_t now;
  uint8_t index;
  HAL_StatusTypeDef status;

  if (s_tx_initialized == 0U)
  {
    return;
  }

  mask = __get_PRIMASK();
  __disable_irq();
  if (s_tx_inflight != 0U)
  {
    if (mask == 0U) __enable_irq();
    return;
  }

  if (s_tx_active_valid == 0U)
  {
    now = HAL_GetTick();
    if (ZDT_X42S_TxSelectLocked(&index, now) == 0U)
    {
      if (mask == 0U) __enable_irq();
      return;
    }
    s_tx_active = s_tx_queue[index];
    ZDT_X42S_RemoveQueueAtLocked(index);
    s_tx_active_valid = 1U;
  }

  /* 先置inflight，再启动HAL；这一段保持关中断，避免极短帧在HAL
   * 返回前触发完成回调，导致同一UART出现并发提交。 */
  s_tx_inflight = 1U;
  status = HAL_UART_Transmit_IT(&ZDT_X42S_UART,
                                s_tx_active.data,
                                s_tx_active.len);
  if (status != HAL_OK)
  {
    /* HAL未接受这帧时没有占用UART；放回队列重新仲裁，后续若已到达
     * STOP/WHEELOFF安全帧，会先于普通速度发送。 */
    (void)ZDT_X42S_QueuePush(&s_tx_active);
    s_tx_active_valid = 0U;
    s_tx_inflight = 0U;
  }
  if (mask == 0U) __enable_irq();
}

static void ZDT_X42S_TxCompleteCallback(UART_HandleTypeDef *huart)
{
  (void)huart;

  s_tx_inflight = 0U;
  s_tx_active_valid = 0U;
  ZDT_X42S_TxKick();
}

/* --------------------------- 对外接口 ----------------------------- */

HAL_StatusTypeDef ZDT_X42S_InitTx(void)
{
  uint16_t i;
  uint32_t mask;

  mask = __get_PRIMASK();
  __disable_irq();
  s_tx_queue_count = 0U;
  s_tx_active_valid = 0U;
  s_tx_inflight = 0U;
  s_tx_drop_count = 0U;
  for (i = 0U; i < 256U; ++i)
  {
    s_motor_enabled[i] = 0U;
    s_motor_ready_tick[i] = 0U;
  }
  if (mask == 0U) __enable_irq();

  if (HAL_UART_RegisterCallback(&ZDT_X42S_UART,
                                HAL_UART_TX_COMPLETE_CB_ID,
                                ZDT_X42S_TxCompleteCallback) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_tx_initialized = 1U;
  return HAL_OK;
}

void ZDT_X42S_ServiceTx(void)
{
  ZDT_X42S_TxKick();
}

/**
 * @brief  使能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @note   使能帧入队后，普通速度帧会等待100ms使能稳定时间；等待由
 *         非阻塞发送状态机完成，控制任务不会调用HAL_Delay()。
 */
void ZDT_X42S_Enable(uint8_t addr)
{
  ZDT_X42S_TxFrame_t frame;
  uint32_t ready_tick = HAL_GetTick() + ZDT_X42S_ENABLE_READY_MS;
  uint16_t i;
  uint32_t mask;

  frame.data[0] = addr;
  frame.data[1] = 0xF3U;
  frame.data[2] = 0xABU;
  frame.data[3] = 0x01U;
  frame.data[4] = 0x00U;
  frame.data[5] = ZDT_X42S_FRAME_TAIL;
  frame.len = 6U;
  frame.addr = addr;
  frame.kind = ZDT_X42S_TX_ENABLE;
  frame.not_before_tick = 0U;

  if (ZDT_X42S_QueuePush(&frame) == 0U)
  {
    return;
  }

  mask = __get_PRIMASK();
  __disable_irq();
  if (addr == 0U)
  {
    for (i = 0U; i < 256U; ++i)
    {
      s_motor_enabled[i] = 1U;
      s_motor_ready_tick[i] = ready_tick;
    }
  }
  else
  {
    s_motor_enabled[addr] = 1U;
    s_motor_ready_tick[addr] = ready_tick;
    ZDT_X42S_RefreshBroadcastStateLocked();
  }
  if (mask == 0U) __enable_irq();

  ZDT_X42S_TxKick();
}

/**
 * @brief  失能指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @note   先把该地址尚未发送的速度帧清掉，再提交失能安全帧。失能后
 *         普通速度帧直接被丢弃，不能重新锁轴。
 */
void ZDT_X42S_Disable(uint8_t addr)
{
  ZDT_X42S_TxFrame_t frame;
  uint16_t i;
  uint32_t mask;

  frame.data[0] = addr;
  frame.data[1] = 0xF3U;
  frame.data[2] = 0xABU;
  frame.data[3] = 0x00U;
  frame.data[4] = 0x00U;
  frame.data[5] = ZDT_X42S_FRAME_TAIL;
  frame.len = 6U;
  frame.addr = addr;
  frame.kind = ZDT_X42S_TX_DISABLE;
  frame.not_before_tick = 0U;

  (void)ZDT_X42S_QueuePush(&frame);

  mask = __get_PRIMASK();
  __disable_irq();
  if (addr == 0U)
  {
    for (i = 0U; i < 256U; ++i)
    {
      s_motor_enabled[i] = 0U;
      s_motor_ready_tick[i] = 0U;
    }
  }
  else
  {
    s_motor_enabled[addr] = 0U;
    s_motor_ready_tick[addr] = 0U;
    ZDT_X42S_RefreshBroadcastStateLocked();
  }
  if (mask == 0U) __enable_irq();

  ZDT_X42S_TxKick();
}

/**
 * @brief  立即停止指定地址电机
 * @param  addr 电机地址，1~255；0 为广播地址
 * @note   停止是安全帧，优先于尚未发送的普通速度帧；当前正在发送的
 *         单帧不能被中断，避免在总线上产生半帧。
 */
void ZDT_X42S_Stop(uint8_t addr)
{
  ZDT_X42S_TxFrame_t frame;

  frame.data[0] = addr;
  frame.data[1] = 0xFEU;
  frame.data[2] = 0x98U;
  frame.data[3] = 0x00U;
  frame.data[4] = ZDT_X42S_FRAME_TAIL;
  frame.len = 5U;
  frame.addr = addr;
  frame.kind = ZDT_X42S_TX_STOP;
  frame.not_before_tick = 0U;

  (void)ZDT_X42S_QueuePush(&frame);
  ZDT_X42S_TxKick();
}

/**
 * @brief  速度模式控制电机连续转动（使用默认加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 */
void ZDT_X42S_Speed(uint8_t addr, uint8_t dir, uint16_t rpm)
{
  ZDT_X42S_SpeedAcc(addr, dir, rpm, ZDT_X42S_DEFAULT_ACC);
}

/**
 * @brief  速度模式控制电机连续转动（自定义加速度）
 * @param  addr 电机地址，1~255；0 为广播地址
 * @param  dir  方向：ZDT_X42S_DIR_CW / ZDT_X42S_DIR_CCW
 * @param  rpm  速度，单位 RPM，范围 0~3000
 * @param  acc  加速度档位，0~255；0 为直接启动
 * @note   同一地址尚未发送的旧速度会被新速度覆盖，队列不会积压历史
 *         控制量；已失能地址的速度帧会被拒绝。
 */
void ZDT_X42S_SpeedAcc(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
  ZDT_X42S_TxFrame_t frame;
  uint32_t mask;

  if (rpm > ZDT_X42S_MAX_RPM)
  {
    rpm = ZDT_X42S_MAX_RPM;
  }

  mask = __get_PRIMASK();
  __disable_irq();
  if (s_motor_enabled[addr] == 0U)
  {
    if (mask == 0U) __enable_irq();
    return;
  }
  frame.not_before_tick = s_motor_ready_tick[addr];
  if (mask == 0U) __enable_irq();

  frame.data[0] = addr;
  frame.data[1] = 0xF6U;
  frame.data[2] = dir;
  frame.data[3] = (uint8_t)(rpm >> 8);
  frame.data[4] = (uint8_t)(rpm & 0xFFU);
  frame.data[5] = acc;
  frame.data[6] = 0x00U;
  frame.data[7] = ZDT_X42S_FRAME_TAIL;
  frame.len = 8U;
  frame.addr = addr;
  frame.kind = ZDT_X42S_TX_SPEED;

  (void)ZDT_X42S_QueuePush(&frame);
  ZDT_X42S_TxKick();
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
