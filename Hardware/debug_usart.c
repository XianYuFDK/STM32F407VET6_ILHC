/**
 ******************************************************************************
 * @file    debug_usart.c
 * @brief   USART1 调试模块（VOFA+ 调参）
 *
 *          - VOFA+ JustFloat 数据帧：N*float + 0x00 0x00 0x80 0x7F
 *          - DMA 空闲接收 ASCII 命令：KPX=3.0、XVMAX=1600、STOP、ZERO
 *                            以及 DM 电机：DMID/DMEN/DMOFF/DMMODE/DMPOS 等
 *          - 统一坐标约定（协议与底盘内部相同）：
 *            +X=车左、+Y=车头、+Z=逆时针；X/Y 直接对应 pos_x/pos_y，
 *            不存在交换或取反的适配层。
 *          - GOTO=x,y,z：上位机点击场地地图下发 OPS 全局定位移动目标（x=左右、
 *            y=前后，单位 cm 且保留 1 位小数；z=航向角），本任务每 20ms 周期执行
 *            一步 MecanumControl_GotoOPS，到位后停止，STOP 取消。固件内部仍用 mm 闭环。
 *          - GOTOHOLD=x,y,z：参数与 GOTO 相同，到位后继续位置闭环保持，
 *            STOP/WHEELOFF/OPS 掉线或会话变化/主机失联时取消。
 *          - WHEELEN/WHEELOFF：底盘四轮统一锁轴/释放，失能期间拒绝运动命令；
 *            失能后停车只清目标（MecanumControl_ClearTarget），绝不再发速度帧，
 *            否则ZDT_X42S会重新使能锁轴，表现为"失能了还是锁"
 *          - 参数表可扩展：在 DebugParam_t 表中增加一项即可
 ******************************************************************************
 */
#include "debug_usart.h"
#include "usart.h"
#include "mecanum_control.h"
#include "ops.h"
#include "vision_track.h"
#include "dm_j4310.h"
#include "hcan.h"
#include "stepper_2835.h"
#include "zdt_x42s.h"
#include "debug_param_store.h"

#include <string.h>
#include <stdio.h>


/* --------------------------- 调试参数 ------------------------------ */
#define DEBUG_RX_SIZE     256U
#define DEBUG_LINE_SIZE   64U
#define DEBUG_VOFA_TAIL0  0x00U
#define DEBUG_VOFA_TAIL1  0x00U
#define DEBUG_VOFA_TAIL2  0x80U
#define DEBUG_VOFA_TAIL3  0x7FU

/* ------------------------- DM 电机调试参数 ------------------------- */
#define DEBUG_DM_DEFAULT_ID     3U
#define DEBUG_DM_DEFAULT_MODE   DM_J4310_CTRL_MODE_MIT
#define DEBUG_DM_DEFAULT_POS    0.0f
#define DEBUG_DM_DEFAULT_VEL    0.0f
#define DEBUG_DM_DEFAULT_KP     2.0f
#define DEBUG_DM_DEFAULT_KD     0.5f
#define DEBUG_DM_DEFAULT_TORQUE 0.0f
#define DEBUG_DM_MODE_WAIT_MS   100U
#define DEBUG_HOST_TIMEOUT_MS   1000U
#define DEBUG_OPS_TIMEOUT_MS    200U
#define DEBUG_GOTO_MOVE         1U
#define DEBUG_GOTO_HOLD         2U

/* 可调参数项：名称 -> 变量指针 + 允许范围 */
typedef struct
{
  const char *name;
  float      *value;
  float       min;
  float       max;
} DebugParam_t;

/* 可调参数表：KPX 直接写 X=左右轴增益 mKpx，KPY 直接写 Y=前后轴增益 mKpy。
 *
 * 末尾三项是视觉跟踪调参，属于"RAM参数"：Debug_CaptureParams 只快照前7项
 * （Flash记录固定为16字段、VERSION=2），因此它们可设置、可 GET 回读，但不随
 * 底盘参数存Flash，掉电回到编译期默认值。要让它们持久化必须升级Flash记录版本
 * 并迁移旧记录，属于独立改动。 */
static const DebugParam_t s_params[] =
{
  {"KPX",   &mKpx,   0.0f,   50.0f},
  {"KPY",   &mKpy,   0.0f,   50.0f},
  {"KPZ",   &mKpz,   0.0f,   50.0f},
  {"XVMAX", &XYVmax, 0.0f,   3000.0f},
  {"ZVMAX", &ZVmax,  0.0f,   3000.0f},
  {"XVMIN", &XYVmin, 0.0f,   100.0f},
  {"ZVMIN", &ZVmin,  0.0f,   100.0f},
  {"VCONF", &vTrackConfMin,    0.0f, 100.0f},
  {"VKPMM", &vTrackKpMm,       0.0f,   5.0f},
  {"VDBMM", &vTrackDeadbandMm, 0.0f, 100.0f},
  {"VDBPX", &vTrackDeadbandPx, 0.0f, 200.0f},
  {"VMIN",  &vTrackMinRpm,     0.0f,  60.0f},
  {"VMAX",  &vTrackMaxRpm,     0.0f, 3000.0f},
};

/* --------------------------- 私有变量 ------------------------------ */
static uint8_t s_rx[DEBUG_RX_SIZE];
static uint8_t s_line[DEBUG_LINE_SIZE];
static uint16_t s_line_len;
/* 接收异常只置位，恢复在默认任务执行；禁止中断内等待DMA停止。 */
static volatile uint8_t s_rx_recover;
static uint8_t s_rx_callbacks_ready;

static uint8_t s_tx[(4U * DEBUG_VOFA_CHANNELS) + 4U];
static uint8_t s_tx_busy_seen;
static uint32_t s_tx_busy_tick;
volatile uint32_t debug_tx_recoveries, debug_tx_errors;

static volatile uint8_t s_stop_req;
static volatile uint8_t s_stop_in_progress;
static volatile uint8_t s_zero_req;
static volatile uint8_t s_offset_req;
static volatile float s_offset_x, s_offset_y;

/* GOTO 全局定位移动目标。
 * s_goto_active: 0=空闲，DEBUG_GOTO_MOVE=移动到点后结束，
 * DEBUG_GOTO_HOLD=到点后持续位置闭环保持。 */
static volatile uint8_t s_goto_active;
static volatile uint32_t s_goto_generation;
static float s_goto_x;
static float s_goto_y;
static float s_goto_z;

/* 手动速度命令必须持续刷新，PING不能延长其350ms有效期。 */
static volatile uint8_t s_manual_active;
static volatile int16_t s_manual_velocity[3];
static volatile uint32_t s_manual_tick;

/* 视觉跟踪命令由USART1接收中断发布，默认任务执行。
 * 0无请求，1..6开始跟踪对应颜色，0xFF停止跟踪。 */
static volatile uint8_t s_vision_req;
static uint8_t s_vision_moving;

/* 底盘四轮锁轴控制：接收中断只置请求，UART4帧由任务入队。
 * 0无请求、1使能（锁轴）、2失能（不锁轴）；同一周期后到的请求覆盖先到的。
 * s_wheel_enabled 由任务写、中断读：失能后不锁轴，GOTO/MANUAL/ZDT
 * 一律拒绝，必须显式 WHEELEN 恢复。main.c 启动时已使能四轮，初值为1。 */
static volatile uint8_t s_wheel_req;
static uint8_t s_wheel_enabled;
static uint8_t s_wheel_enable_pending, s_wheel_fault, s_wheel_disable_pending;
static uint32_t s_wheel_enable_tick, s_wheel_fault_tick, s_wheel_error_seen;
static uint32_t s_control_tick;

/* 单轮限时测试：中断发布请求，任务执行；阶段1等待使能，阶段2计时运行。
 * 测试最长5秒，不依赖主机心跳续期；不修改现有24通道遥测格式。 */
static volatile uint8_t s_zdt_req, s_zdt_active;
static volatile int16_t s_zdt_args[3];
static uint8_t s_zdt_addr;
static int16_t s_zdt_rpm;
static uint32_t s_zdt_tick, s_zdt_duration;

/* 文字应答采用事件队列，接收中断只入队，默认任务通过USART1 DMA发送。
 * ZDT切换至文字模式，避免JustFloat二进制内容干扰阅读；VOFA恢复遥测。
 * 队列满时丢弃新应答而不影响STOP处理；应答仅描述MCU状态，不代表电机回包。 */
static volatile uint8_t s_zdt_text_mode;
static volatile uint8_t s_ack_read, s_ack_write;
static uint8_t s_ack_queue[16];
static uint8_t s_ack_frames[16][4];
/* 参数回读应答：GET <名称> 的文本按队列槽存放，中断只写 s_ack_write 槽、
 * 默认任务只读 s_ack_read 槽，与 s_ack_frames 同一套无锁约定。
 * 24 字节足够最长一条 "XVMAX=3000.000\r\n"。 */
static char s_ack_param[16][24];
typedef struct
{
  uint8_t index, kind, command, length;
  uint8_t data[8];
} DebugStepperEvent_t;
static DebugStepperEvent_t s_ack_stepper[16];
static uint8_t s_zdt_watch;
/* 单轮测试期间"期望的应答"：必须同时匹配 地址 + 功能码，状态码单独判读。
 * 只比地址是不够的：Stop(0xFE)/Enable(0xF3) 的回包同样是 4 字节且尾字节 0x6B，
 * 之前发过的命令的迟到回包会被当成速度(0xF6)命令的成功应答。 */
static uint8_t s_zdt_watch_cmd;
static uint32_t s_zdt_watch_tick;
/* 文字应答事件号：19为静态"参数名非法"（表中最后一条），
 * 20为动态参数回读文本（文本在 s_ack_param 槽里，见 Debug_ReplyParam）。 */
#define DEBUG_ACK_PARAM_UNKNOWN 19U
#define DEBUG_ACK_PARAM_TEXT    20U
#define DEBUG_ACK_VTRACK_START  21U
#define DEBUG_ACK_VTRACK_STOP   22U
#define DEBUG_ACK_VTRACK_FORMAT 23U
#define DEBUG_ACK_VTRACK_BUSY   24U
#define DEBUG_ACK_VTRACK_ERROR  25U
#define DEBUG_ACK_STEPPER       26U
static const char * const s_ack_text[] = {
  "ACK ZDT ACCEPTED\r\n",
  "ERR ZDT BUSY\r\n",
  "ERR ZDT FORMAT: ZDT=1..4,-300..300,1..5\r\n",
  "ACK ZDT RUN_REQUESTED (NO MOTOR ACK)\r\n",
  "ACK ZDT STOP_REQUESTED (NO MOTOR ACK)\r\n",
  "ACK ZDT CANCELLED (NO MOTOR ACK)\r\n",
  "WARN ZDT NO_VALID_REPLY IN 500MS\r\n",
  NULL, /* 事件7用于原始电机回包，不索引文本。 */
  "ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE\r\n",
  "ERR CAN DISABLED; DM/S28/S35 REJECTED\r\n",
  "ERR WHEEL DISABLED; WHEELEN FIRST\r\n",
  "ERR ZDT REPLY STATUS != 0x02 (SEE RAW FRAME)\r\n",
  "ERR GOTO INVALID HEADING\r\n",
  "ERR DM DISABLE TX FAILED; DMOFF TO RETRY\r\n",
  "ERR WHEEL TX FAILED; WHEELEN TO RECOVER\r\n",
  "ACK PARAM LOADED FROM FLASH\r\n",
  "INFO PARAM DEFAULTS; NO VALID FLASH RECORD\r\n",
  "ACK PARAM SAVED TO FLASH\r\n",
  "ERR PARAM FLASH; UNSAVED CHANGES LOST ON POWER OFF\r\n",
  /* 这条提示是应答表里最长的一条，必须留在 s_tx(100字节) 之内：
   * 再加参数名时要同时核对长度（test_debug_wheel.py 有长度守卫）。 */
  "ERR PARAM UNKNOWN; GET KPX|KPY|KPZ|XVMAX|ZVMAX|XVMIN|ZVMIN|VCONF|VKPMM|VDBMM|VDBPX|VMIN|VMAX\r\n",
  NULL, /* 事件20为动态参数回读文本。 */
  "ACK VTRACK START REQUESTED\r\n",
  "ACK VTRACK STOP REQUESTED\r\n",
  "ERR VTRACK FORMAT; USE VTRACK=0..6\r\n",
  "ERR VTRACK BUSY\r\n",
  "ERR VTRACK VISION ERROR\r\n",
  NULL /* 事件26为28/35命令诊断或CAN原始回复。 */
};

static void Debug_ZdtAck(uint8_t event)
{
  uint32_t mask = __get_PRIMASK();
  uint8_t next;
  __disable_irq();
  next = (uint8_t)((s_ack_write + 1U) % 16U);
  if (next != s_ack_read)
  {
    s_ack_queue[s_ack_write] = event;
    s_ack_write = next;
  }
  if (mask == 0U) __enable_irq();
}

/* 原始电机回包单独标记为事件7，通过同一个USART1发送队列输出。
 * 只在文字模式显示，正常VOFA模式仍持续排空接收队列以免积压旧回复。 */
static void Debug_ServiceZdtReplies(void)
{
  uint8_t frame[4], next, i;
  ZDT_X42S_ServiceRx();
  ZDT_X42S_ServiceTx();
  while (ZDT_X42S_PopReply(frame))
  {
    /* 只在"地址 + 期望功能码"都匹配时才认定本次测试收到了有效回包；
     * 状态码 0x02 才是命令正确应答，非 0x02（如 0xE2/0xEE 参数或保护错误）
     * 只报错、不当成功。原始 4 字节仍然照常打印，便于人工核对。 */
    if (s_zdt_watch && (frame[0] == s_zdt_addr) && (frame[1] == s_zdt_watch_cmd))
    {
      if (frame[2] == 0x02U)
      {
        s_zdt_watch = 0U;
      }
      else
      {
        s_zdt_watch = 0U;
        Debug_ZdtAck(11U);
      }
    }
    if (!s_zdt_text_mode) continue;
    {
      uint32_t mask = __get_PRIMASK();
      __disable_irq();
      next = (uint8_t)((s_ack_write + 1U) % 16U);
      if (next != s_ack_read)
      {
        for (i = 0U; i < 4U; ++i) s_ack_frames[s_ack_write][i] = frame[i];
        s_ack_queue[s_ack_write] = 7U;
        s_ack_write = next;
      }
      if (mask == 0U) __enable_irq();
    }
  }
  /* 仅检查本次目标电机是否有任一有效控制回包，不冒充逐条命令的事务确认。
   * 超时只给诊断提示，不延长测试时间；500ms后到达的回包仍正常输出。 */
  if (s_zdt_watch && (uint32_t)(HAL_GetTick() - s_zdt_watch_tick) >= 500U)
  {
    s_zdt_watch = 0U;
    Debug_ZdtAck(6U);
  }
}

/* DM 电机调试状态 */
static uint16_t s_dm_id;
static uint8_t  s_dm_mode;
static uint8_t  s_dm_active;
static float    s_dm_pos;
static float    s_dm_vel;
static float    s_dm_kp;
static float    s_dm_kd;
static float    s_dm_torque;
static DmJ4310Feedback_t s_dm_feedback;

static volatile uint8_t s_dm_enable_req;
static volatile uint8_t s_dm_disable_req;
static volatile uint8_t s_dm_zero_req;
static volatile uint8_t s_dm_mode_req;
static uint8_t s_dm_start_pending;
static uint32_t s_dm_mode_tick;
static uint8_t s_dm_disable_pending, s_dm_disable_fault;
static uint16_t s_dm_disable_id;
static uint32_t s_dm_disable_tick;
static volatile uint32_t s_host_last_tick;

/* 28/35 单次请求邮箱：串口中断只更新参数，任务提交 CAN。
 * 同一电机新请求替换尚未提交的旧请求；BUSY 最多重试1秒。
 * CANCEL/STOP只能撤销待发请求，不能停止已经发送的运动。 */
typedef struct
{
  uint32_t position;
  uint32_t queued_tick;
  uint16_t speed;
  uint8_t direction;
  uint8_t action; /* 0无请求，1回零，2机械目标，3角度目标，4读状态，5使能 */
} DebugStepperRequest_t;
static volatile DebugStepperRequest_t s_stepper_req[2]; /* 35、28 */
static uint32_t s_stepper_seen[2], s_stepper_watch_tick[2];
static uint8_t s_stepper_watch_cmd[2];

/* kind: 0请求入队、1取消待发、2格式错误、3提交成功、4提交失败、
 * 5待发超时、6原始回包、7电机回包超时。中断中只入队，任务中格式化。 */
static void Debug_StepperEvent(uint8_t index, uint8_t kind, uint8_t command,
                               const Stepper2835Reply_t *reply)
{
  uint32_t mask = __get_PRIMASK();
  uint8_t next;
  __disable_irq();
  next = (uint8_t)((s_ack_write + 1U) % 16U);
  if (next != s_ack_read)
  {
    DebugStepperEvent_t *event = &s_ack_stepper[s_ack_write];
    memset(event, 0, sizeof(*event));
    event->index = index;
    event->kind = kind;
    event->command = command;
    if (reply != NULL)
    {
      event->length = reply->length;
      memcpy(event->data, reply->data, sizeof(event->data));
    }
    s_ack_queue[s_ack_write] = DEBUG_ACK_STEPPER;
    s_ack_write = next;
  }
  if (mask == 0U) __enable_irq();
}

static uint16_t Debug_FormatStepperEvent(char *out, uint16_t capacity,
                                         const DebugStepperEvent_t *event)
{
  static const char * const messages[] = {
    "ACK S%u QUEUED (NO MOTOR ACK)\r\n",
    "ACK S%u CANCELLED UNSENT ONLY\r\n",
    "ERR S%u FORMAT/RANGE\r\n",
    "ACK S%u CAN_SUBMITTED CMD=%02X (NO MOTOR ACK)\r\n",
    "ERR S%u CAN_TX_FAILED CMD=%02X\r\n",
    "WARN S%u CAN_TX_TIMEOUT\r\n",
    NULL,
    "WARN S%u NO_REPLY CMD=%02X IN 500MS\r\n"
  };
  unsigned int motor = event->index == 0U ? 35U : 28U;
  int n;
  if (capacity == 0U) return 0U;
  if (event->kind == 6U)
  {
    uint8_t j;
    n = snprintf(out, capacity, "S%u RX CAN=%04X DATA=", motor,
                 (unsigned int)(event->index == 0U ? MOTOR35_CAN_ID : MOTOR28_CAN_ID));
    if (n < 0 || n >= capacity) return 0U;
    for (j = 0U; j < event->length && j < 8U; ++j)
    {
      int added = snprintf(out + n, capacity - n, "%02X ", (unsigned int)event->data[j]);
      if (added < 0 || added >= capacity - n) return 0U;
      n += added;
    }
    if (n + 2 >= capacity) return 0U;
    out[n++] = '\r'; out[n++] = '\n'; out[n] = '\0';
  }
  else
  {
    if (event->kind >= sizeof(messages) / sizeof(messages[0])) return 0U;
    n = snprintf(out, capacity, messages[event->kind], motor, (unsigned int)event->command);
  }
  return n > 0 && n < capacity ? (uint16_t)n : 0U;
}

static void Debug_ServiceStepperReplies(void)
{
  uint8_t i;
  for (i = 0U; i < 2U; ++i)
  {
    Stepper2835Reply_t reply;
    uint16_t id = i == 0U ? MOTOR35_CAN_ID : MOTOR28_CAN_ID;
    if (Stepper2835_GetReply(id, &reply) && reply.count != s_stepper_seen[i])
    {
      s_stepper_seen[i] = reply.count;
      Debug_StepperEvent(i, 6U, 0U, &reply);
      /* 功能码必须匹配；EE格式错误使用功能码00。迟到的其他回包不能消除超时。 */
      if (reply.length >= 3U && reply.data[reply.length - 1U] == 0x6BU &&
          (reply.data[0] == s_stepper_watch_cmd[i] ||
           (reply.data[0] == 0U && reply.data[1] == 0xEEU)))
        s_stepper_watch_cmd[i] = 0U;
    }
    if (s_stepper_watch_cmd[i] != 0U &&
        (uint32_t)(HAL_GetTick() - s_stepper_watch_tick[i]) >= 500U)
    {
      Debug_StepperEvent(i, 7U, s_stepper_watch_cmd[i], NULL);
      s_stepper_watch_cmd[i] = 0U;
    }
  }
}

/* 快照只复制参数，Flash 操作全部在短临界区之外。 */
static uint8_t s_param_error_reported;
static void Debug_CaptureParams(DebugParamValues *v)
{
  uint32_t i, mask = __get_PRIMASK();
  __disable_irq();
  for (i = 0U; i < 7U; ++i) v->value[i] = *s_params[i].value;
  OPS_GetMountOffset(&v->value[7], &v->value[8]);
  v->value[9] = (float)s_dm_id; v->value[10] = (float)s_dm_mode;
  v->value[11] = s_dm_pos; v->value[12] = s_dm_vel;
  v->value[13] = s_dm_kp; v->value[14] = s_dm_kd; v->value[15] = s_dm_torque;
  if (mask == 0U) __enable_irq();
}

/* 启动时仅恢复数值，不置任何使能/运动请求，不恢复ZERO参考系。 */
static void Debug_InitParams(void)
{
  DebugParamValues v;
  uint32_t i;
  ParamStoreResult result;
  Debug_CaptureParams(&v);
  result = DebugParamStore_Init(&v);
  s_param_error_reported = (result == PARAM_STORE_ERROR);
  if (result == PARAM_STORE_LOADED) {
    for (i = 0U; i < 7U; ++i) *s_params[i].value = v.value[i];
    (void)OPS_SetMountOffset(v.value[7], v.value[8]);
    s_dm_id = (uint16_t)v.value[9]; s_dm_mode = (uint8_t)v.value[10];
    s_dm_pos = v.value[11]; s_dm_vel = v.value[12];
    s_dm_kp = v.value[13]; s_dm_kd = v.value[14]; s_dm_torque = v.value[15];
    Debug_ZdtAck(15U);
  }
  else Debug_ZdtAck(result == PARAM_STORE_ERROR ? 18U : 16U);
}

static void Debug_ServiceParams(void)
{
  DebugParamValues v;
  ParamStoreResult result;
  Debug_CaptureParams(&v);
  result = DebugParamStore_Service(&v, HAL_GetTick());
  if (result == PARAM_STORE_SAVED) Debug_ZdtAck(17U);
  else if (result == PARAM_STORE_ERROR && !s_param_error_reported) {
    s_param_error_reported = 1U;
    Debug_ZdtAck(18U);
  }
}

/* --------------------------- 私有函数 ------------------------------ */




/**
 * @brief  ASCII 忽略大小写比较
 */
static uint8_t Debug_StrCaseCmp(const char *a, const char *b)
{
  while ((*a != '\0') && (*b != '\0'))
  {
    char ca = *a;
    char cb = *b;

    if ((ca >= 'a') && (ca <= 'z')) ca -= ('a' - 'A');
    if ((cb >= 'a') && (cb <= 'z')) cb -= ('a' - 'A');

    if (ca != cb)
    {
      return 1U;
    }

    ++a;
    ++b;
  }

  return ((*a == '\0') && (*b == '\0')) ? 0U : 1U;
}
/**
 * @brief  轻量浮点解析，仅支持小数，无指数
 */
static uint8_t Debug_ParseFloat(const char *s, float *out)
{
  float sign = 1.0f;
  float value = 0.0f;
  float scale = 0.1f;
  uint8_t has_digit = 0U;

  if ((s == NULL) || (out == NULL))
  {
    return 0U;
  }

  while ((*s == ' ') || (*s == '\t'))
  {
    ++s;
  }

  if (*s == '-')
  {
    sign = -1.0f;
    ++s;
  }
  else if (*s == '+')
  {
    ++s;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    value = (value * 10.0f) + (float)(*s - '0');
    has_digit = 1U;
    ++s;
  }

  if (*s == '.')
  {
    ++s;
    while ((*s >= '0') && (*s <= '9'))
    {
      value += (float)(*s - '0') * scale;
      scale *= 0.1f;
      has_digit = 1U;
      ++s;
    }
  }

  while ((*s == ' ') || (*s == '\t')) ++s;
  if (!has_digit || !((value >= 0.0f) && (value <= 3.402823466e38F)) ||
      ((*s != '\0') && (*s != ',')))
  {
    return 0U;
  }

  *out = sign * value;
  return 1U;
}

/**
 * @brief  ASCII 忽略大小写前缀比较（a 的前 n 个字符与 b 比较）
 */
static uint8_t Debug_StrCaseCmpN(const char *a, const char *b, uint32_t n)
{
  uint32_t i;

  for (i = 0U; i < n; ++i)
  {
    char ca = a[i];
    char cb = b[i];

    if ((ca >= 'a') && (ca <= 'z')) ca -= ('a' - 'A');
    if ((cb >= 'a') && (cb <= 'z')) cb -= ('a' - 'A');

    if ((ca != cb) || (ca == '\0'))
    {
      return 1U;
    }
  }

  return 0U;
}

/**
 * @brief  解析逗号分隔的浮点列表，例如 "120.0,80.0,90.0"
 * @return 解析出的个数（0 ~ max）
 */
static uint8_t Debug_ParseFloatList(const char *s, float *out, uint8_t max)
{
  uint8_t count = 0U;

  if ((s == NULL) || (out == NULL))
  {
    return 0U;
  }

  while ((*s != '\0') && (count < max))
  {
    float value;

    if (!Debug_ParseFloat(s, &value))
    {
      return 0U;
    }

    out[count] = value;
    ++count;

    /* 跳过已解析的数字，直到逗号或结尾 */
    while ((*s != '\0') && (*s != ','))
    {
      ++s;
    }
    if (*s == ',')
    {
      ++s;
      if (*s == '\0' || count == max) return 0U;
    }
  }

  return count;
}


/**
 * @brief  在参数表中查找并设置参数
 */
static void Debug_SetParam(const char *name, float value)
{
  uint32_t i;
  uint32_t count = sizeof(s_params) / sizeof(s_params[0]);

  for (i = 0U; i < count; ++i)
  {
    if (Debug_StrCaseCmp(name, s_params[i].name) == 0U)
    {
      if (value < s_params[i].min)
      {
        value = s_params[i].min;
      }
      else if (value > s_params[i].max)
      {
        value = s_params[i].max;
      }

      *s_params[i].value = value;
      return;
    }
  }
}

/**
 * @brief  回读一个可调参数，把 "名称=值" 文本放入文字应答队列
 *
 * XVMIN/ZVMIN 在24通道遥测里没有通道位，上位机的"回读"栏无法从波形取得
 * 当前值；这条 GET <名称> 应答补上该方向，读回的是参数表指向的实时 RAM 值
 * （含Flash恢复值和本次会话的修改）。名称非法回 ERR，便于区分"没有该参数"
 * 和"没有收到应答"。应答走与ZDT/Flash相同的队列：中断只入队，默认任务在
 * DMA空闲时优先发送，不改变 JustFloat 帧格式和通道总数。
 * @param  name  参数名，大小写不敏感
 */
static void Debug_ReplyParam(const char *name)
{
  uint32_t i, count, mask, slot, next;
  int32_t scaled, whole, frac;
  char *out;
  char tmp[8];
  uint8_t k, n;

  count = sizeof(s_params) / sizeof(s_params[0]);

  for (i = 0U; i < count; ++i)
  {
    if (Debug_StrCaseCmp(name, s_params[i].name) != 0U) continue;

    /* 组包与入队必须在同一临界区：ZDT服务在任务上下文也会推进 s_ack_write，
     * 分两步做会把文本写进别人的槽位。 */
    mask = __get_PRIMASK();
    __disable_irq();
    slot = s_ack_write;
    next = (uint8_t)((slot + 1U) % 16U);
    if (next != s_ack_read)
    {
      out = s_ack_param[slot];
      n = 0U;
      /* 名称回显参数表中的规范大写，上位机据此匹配界面行。 */
      while ((s_params[i].name[n] != '\0') && (n < 12U))
      {
        out[n] = s_params[i].name[n];
        ++n;
      }
      out[n] = '=';
      ++n;
      /* 不用printf的%f：Keil精简库不带浮点格式化，这里按0.001定点输出。 */
      scaled = (int32_t)((*s_params[i].value * 1000.0f) +
                         ((*s_params[i].value >= 0.0f) ? 0.5f : -0.5f));
      if (scaled < 0)
      {
        out[n] = '-';
        ++n;
        scaled = -scaled;
      }
      whole = scaled / 1000;
      frac = scaled % 1000;
      k = 0U;
      if (whole == 0) tmp[k++] = '0';
      while ((whole != 0) && (k < 8U))
      {
        tmp[k] = (char)('0' + (whole % 10));
        ++k;
        whole /= 10;
      }
      while (k > 0U)
      {
        --k;
        out[n] = tmp[k];
        ++n;
      }
      out[n]      = '.';
      out[n + 1U] = (char)('0' + ((frac / 100) % 10));
      out[n + 2U] = (char)('0' + ((frac / 10) % 10));
      out[n + 3U] = (char)('0' + (frac % 10));
      n = (uint8_t)(n + 4U);
      out[n]      = '\r';
      out[n + 1U] = '\n';
      out[n + 2U] = '\0';
      s_ack_queue[slot] = DEBUG_ACK_PARAM_TEXT;
      s_ack_write = (uint8_t)next;
    }
    if (mask == 0U) __enable_irq();
    return;
  }

  Debug_ZdtAck(DEBUG_ACK_PARAM_UNKNOWN);
}

/**
 * @brief  设置 DM 电机调试参数
 */
static void Debug_SetDmValue(const char *name, float value)
{
  if (Debug_StrCaseCmp(name, "DMID") == 0U)
  {
    if (!s_dm_active && !s_dm_start_pending && !s_dm_disable_pending && !s_dm_disable_fault &&
        (value >= 1.0f) && (value <= 0x06FFU))
    {
      s_dm_id = (uint16_t)value;
    }
    return;
  }

  if (Debug_StrCaseCmp(name, "DMMODE") == 0U)
  {
    if ((value >= DM_J4310_CTRL_MODE_MIT) &&
        (value <= DM_J4310_CTRL_MODE_POS_VEL))
    {
      s_dm_mode = (uint8_t)value;
      s_dm_mode_req = 1U;
    }
    return;
  }

  if (Debug_StrCaseCmp(name, "DMPOS") == 0U)
  {
    if (value < DM_J4310_POS_MIN) { value = DM_J4310_POS_MIN; }
    if (value > DM_J4310_POS_MAX) { value = DM_J4310_POS_MAX; }
    s_dm_pos = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMVEL") == 0U)
  {
    if (value < DM_J4310_VEL_MIN) { value = DM_J4310_VEL_MIN; }
    if (value > DM_J4310_VEL_MAX) { value = DM_J4310_VEL_MAX; }
    s_dm_vel = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMKP") == 0U)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > DM_J4310_KP_MAX) { value = DM_J4310_KP_MAX; }
    s_dm_kp = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMKD") == 0U)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > DM_J4310_KD_MAX) { value = DM_J4310_KD_MAX; }
    s_dm_kd = value;
    return;
  }

  if (Debug_StrCaseCmp(name, "DMTOR") == 0U)
  {
    if (value < DM_J4310_TORQUE_MIN) { value = DM_J4310_TORQUE_MIN; }
    if (value > DM_J4310_TORQUE_MAX) { value = DM_J4310_TORQUE_MAX; }
    s_dm_torque = value;
  }
}

/**
 * @brief  解析一条完整命令
 */
/* 严格解析无符号十进制列表，拒绝溢出、负数、小数及多余字段。 */
static uint8_t Debug_ParseUnsignedList(const char *p, uint32_t *values, uint8_t count)
{
  uint8_t i;
  for (i = 0U; i < count; ++i)
  {
    uint32_t value = 0U;
    uint8_t digits = 0U;
    while (*p >= '0' && *p <= '9')
    {
      uint32_t digit = (uint32_t)(*p - '0');
      if (value > (UINT32_MAX - digit) / 10U) return 0U;
      value = value * 10U + digit;
      digits = 1U;
      ++p;
    }
    if (!digits) return 0U;
    values[i] = value;
    if (i + 1U < count)
    {
      if (*p != ',') return 0U;
      ++p;
    }
  }
  return *p == '\0';
}

static uint8_t Debug_ParseStepper(const char *line)
{
  uint8_t index;
  uint32_t v[3];
  uint8_t action;
  if (Debug_StrCaseCmpN(line, "S35", 3U) == 0U) index = 0U;
  else if (Debug_StrCaseCmpN(line, "S28", 3U) == 0U) index = 1U;
  else return 0U;
  line += 3;
  if (Debug_StrCaseCmp(line, "CANCEL") == 0U)
  {
    s_stepper_req[index].action = 0U;
    Debug_StepperEvent(index, 1U, 0U, NULL);
    return 1U;
  }
  if (Debug_StrCaseCmp(line, "HOME") == 0U)
  {
    action = 1U;
    v[0] = v[1] = v[2] = 0U;
  }
  else if (Debug_StrCaseCmp(line, "STATUS") == 0U || Debug_StrCaseCmp(line, "EN") == 0U)
  {
    action = Debug_StrCaseCmp(line, "STATUS") == 0U ? 4U : 5U;
    v[0] = v[1] = v[2] = 0U;
  }
  else if (Debug_StrCaseCmpN(line, "MOVE=", 5U) == 0U)
  {
    if (!Debug_ParseUnsignedList(line + 5, v, 2U)) goto invalid;
    if (index == 0U)
    {
      if (v[0] < MOTOR35_HOME_HEIGHT - MOTOR35_MAX_TRAVEL ||
          v[0] > MOTOR35_HOME_HEIGHT || v[1] < 1U || v[1] > STEPPER_X_MAX_RPM / 30U) goto invalid;
    }
    else if (v[0] < MOTOR28_HOME_RADIUS ||
             v[0] > MOTOR28_HOME_RADIUS + MOTOR28_MAX_TRAVEL ||
             v[1] < 2U || v[1] > (STEPPER_X_MAX_RPM * 100U + 99U) / 53U) goto invalid;
    v[2] = v[1];
    v[1] = v[0];
    v[0] = 0U;
    action = 2U;
  }
  else if (Debug_StrCaseCmpN(line, "RAW=", 4U) == 0U)
  {
    if (!Debug_ParseUnsignedList(line + 4, v, 3U) ||
        v[0] > 1U || v[2] < 1U || v[2] > STEPPER_X_MAX_RPM) goto invalid;
    action = 3U;
  }
  else goto invalid;
  s_stepper_req[index].direction = (uint8_t)v[0];
  s_stepper_req[index].position = v[1];
  s_stepper_req[index].speed = (uint16_t)v[2];
  s_stepper_req[index].queued_tick = HAL_GetTick();
  s_stepper_req[index].action = action;
  Debug_StepperEvent(index, 0U, 0U, NULL);
  return 1U;
invalid:
  Debug_StepperEvent(index, 2U, 0U, NULL);
  return 1U;
}

/* 短临界区保证请求不会在CAN提交期间被中断替换；驱动只提交邮箱，不等待。 */
static void Debug_ServiceSteppers(void)
{
  uint8_t i;
  for (i = 0U; i < 2U; ++i)
  {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t mask = __get_PRIMASK();
    uint16_t id = i == 0U ? MOTOR35_CAN_ID : MOTOR28_CAN_ID;
    uint8_t command = 0U;
    __disable_irq();
    if (s_stepper_req[i].action != 0U &&
        (uint32_t)(HAL_GetTick() - s_stepper_req[i].queued_tick) > DEBUG_HOST_TIMEOUT_MS)
    {
      s_stepper_req[i].action = 0U;
      Debug_StepperEvent(i, 5U, 0U, NULL);
    }
    /* 等前一条命令的匹配回包或500ms超时，避免连续FD指令的分包互相穿插。 */
    if (s_stepper_req[i].action == 0U || s_stepper_watch_cmd[i] != 0U)
    {
      if (mask == 0U) __enable_irq();
      continue;
    }
    if (s_stepper_req[i].action == 1U)
    {
      command = 0x9AU;
      status = Motor_Homing(id);
    }
    else if (s_stepper_req[i].action == 2U)
    {
      command = 0xFDU;
      status = i == 0U ? Motor35_AbsPosition(s_stepper_req[i].position, s_stepper_req[i].speed)
                       : Motor28_AbsPosition(s_stepper_req[i].position, s_stepper_req[i].speed);
    }
    else if (s_stepper_req[i].action == 3U)
    {
      command = 0xFDU;
      status = Motor_AbsPosition(s_stepper_req[i].direction, id,
                                s_stepper_req[i].position, s_stepper_req[i].speed);
    }
    else if (s_stepper_req[i].action == 4U)
    {
      command = 0x3AU;
      status = Stepper2835_ReadStatus(id);
    }
    else if (s_stepper_req[i].action == 5U)
    {
      command = 0xF3U;
      status = Stepper2835_Enable(id);
    }
    if (status == HAL_OK)
    {
      s_stepper_watch_cmd[i] = command;
      s_stepper_watch_tick[i] = HAL_GetTick();
      Debug_StepperEvent(i, 3U, command, NULL);
    }
    else if (status != HAL_BUSY)
      Debug_StepperEvent(i, 4U, command, NULL);
    if (status != HAL_BUSY) s_stepper_req[i].action = 0U;
    if (mask == 0U) __enable_irq();
  }
}

/* 严格解析三个整数速度分量，每轴范围 -300..300 RPM。 */
static uint8_t Debug_ParseManual(const char *s, int16_t *v)
{
  uint8_t i;
  for (i = 0U; i < 3U; ++i)
  {
    int sign = 1;
    int value = 0;
    if (*s == '-') { sign = -1; ++s; }
    else if (*s == '+') ++s;
    if (*s < '0' || *s > '9') return 0U;
    while (*s >= '0' && *s <= '9')
    {
      value = value * 10 + (*s++ - '0');
      if (value > 300) return 0U;
    }
    v[i] = (int16_t)(value * sign);
    if (i < 2U) { if (*s++ != ',') return 0U; }
    else if (*s != '\0') return 0U;
  }
  return 1U;
}

/* 底盘运动闸门：四轮失能后 GOTO/MANUAL 在解析阶段即被丢弃。
 * 由任务修改，接收中断只读，不阻塞也不改中断状态。 */
static uint8_t Debug_WheelReady(void)
{
  return (s_wheel_enabled != 0U && s_wheel_enable_pending == 0U &&
          s_wheel_fault == 0U && s_wheel_req == 0U && s_stop_in_progress == 0U) ? 1U : 0U;
}

/* 四轮失能后禁止再向 UART4 下发任何速度帧：ZDT_X42S 在速度模式下收到任意
 * 速度命令都会重新使能并锁轴，失能帧之后只要还有一条速度帧，轮子就会立刻
 * 重新锁住。因此失能后只清理软件目标，停车帧留给重新使能之后再发。 */
static void Debug_ChassisStop(void)
{
  if (s_wheel_enabled != 0U && s_wheel_fault == 0U)
  {
    MecanumControl_Stop();
  }
  else
  {
    MecanumControl_ClearTarget();
  }
}

/* 结束单轮测试：被测轮发专用停止帧(0xFE 0x98)，四轮再统一恢复。
 * 测试开始时被测轮以外是 Disable（释放锁轴），必须在结束时把它们恢复成
 * "0 转速 + 锁轴"，否则固件以为 s_wheel_enabled=1 而实际有三个轮子是自由状态；
 * 速度帧本身即重新使能锁轴，所以这里走 Debug_ChassisStop 即可（失能状态下
 * 它只清目标，不会反把轮子锁上）。 */
static void Debug_ZdtTestFinish(void)
{
  ZDT_X42S_Stop(s_zdt_addr);
  Debug_ChassisStop();
  s_zdt_active = 0U;
  s_zdt_watch = 0U;
}

static void Debug_ServiceManual(void)
{
  int16_t v[3];
  uint8_t active;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  active = s_manual_active;
  v[0] = s_manual_velocity[0];
  v[1] = s_manual_velocity[1];
  v[2] = s_manual_velocity[2];
  if (active && (uint32_t)(HAL_GetTick() - s_manual_tick) > 350U)
  {
    s_manual_active = 0U;
    v[0] = v[1] = v[2] = 0;
  }
  if (mask == 0U) __enable_irq();
  if (active == 0U) return;
  if (Debug_WheelReady() == 0U)
  {
    /* 失能状态下不得输出轮速，只清理目标，避免重新使能锁轴。 */
    MecanumControl_ClearTarget();
    return;
  }
  if (v[0] == 0 && v[1] == 0 && v[2] == 0) MecanumControl_Stop();
  /* MANUAL 与底盘统一坐标完全同序：X=左、Y=前、W=逆时针。 */
  else
  {
    MecanumControl_MoveVelocity((float)v[0], (float)v[1], (float)v[2]);
  }
}

/* 视觉任务只在默认任务操作底盘。接收中断仅发布启停请求，确保STOP、
 * 四轮失能和发送故障仍由调试层统一仲裁。没有新鲜目标时只停车一次，
 * 不在每个20ms周期重复挤占UART4发送队列。 */
static void Debug_ServiceVision(void)
{
  uint8_t request, was_active;
  uint32_t mask = __get_PRIMASK();
  VisionTrackOutput output;

  __disable_irq();
  request = s_vision_req;
  s_vision_req = 0U;
  if (mask == 0U) __enable_irq();

  if (request == 0xFFU) {
    VisionTrack_Stop();
    if (s_vision_moving) Debug_ChassisStop();
    s_vision_moving = 0U;
  } else if (request != 0U) {
    if (!Debug_WheelReady() || s_zdt_active || s_zdt_req ||
        s_stop_req || s_zero_req || s_offset_req) {
      Debug_ZdtAck(DEBUG_ACK_VTRACK_BUSY);
    } else {
      s_manual_active = s_goto_active = 0U;
      Debug_ChassisStop();
      (void)VisionTrack_Start(request);
      s_vision_moving = 0U;
    }
  }

  if (VisionTrack_IsActive() &&
      (!Debug_WheelReady() || s_stop_req || s_zero_req || s_offset_req)) {
    VisionTrack_Stop();
    if (s_vision_moving) Debug_ChassisStop();
    s_vision_moving = 0U;
  }

  was_active = VisionTrack_IsActive();
  VisionTrack_Service(HAL_GetTick(), &output);
  if (was_active && !VisionTrack_IsActive())
    Debug_ZdtAck(DEBUG_ACK_VTRACK_ERROR);
  if (!VisionTrack_IsActive()) {
    if (s_vision_moving) Debug_ChassisStop();
    s_vision_moving = 0U;
    return;
  }
  if (output.move) {
    MecanumControl_MoveVelocity(output.vx_rpm, output.vy_rpm, 0.0f);
    s_vision_moving = 1U;
  } else if (s_vision_moving) {
    Debug_ChassisStop();
    s_vision_moving = 0U;
  }
}

/* GOTO 位置闭环服务。
 * DEBUG_GOTO_MOVE: 到位后取消目标并停车。
 * DEBUG_GOTO_HOLD: 到位后继续保持目标和位置环，人工扰动后自动纠回。 */
static void Debug_ServiceGoto(void)
{
  uint8_t reached;
  float x, y, z;
  uint32_t mask, generation;

  if (s_goto_active == 0U) return;

  if (Debug_WheelReady() == 0U)
  {
    s_goto_active = 0U;
    MecanumControl_ClearTarget();
    return;
  }

  if (OPS_IsOnline(DEBUG_OPS_TIMEOUT_MS) == 0U)
  {
    s_goto_active = 0U;
    Debug_ChassisStop();
    return;
  }

  mask = __get_PRIMASK();
  __disable_irq();
  x = s_goto_x; y = s_goto_y; z = s_goto_z;
  generation = s_goto_generation;
  if (mask == 0U) __enable_irq();
  reached = MecanumControl_GotoOPS(x, y, z, 0.0f);
  mask = __get_PRIMASK();
  __disable_irq();
  reached = (reached != 0U && s_goto_active == DEBUG_GOTO_MOVE &&
             generation == s_goto_generation) ? 1U : 0U;
  if (reached) s_goto_active = 0U;
  if (mask == 0U) __enable_irq();
  if (reached) Debug_ChassisStop();
}

/* 任务上下文执行四轮使能/失能：先取消运动并停车，再把UART4帧入队。
 * 失能只释放锁轴，不改变DM、28/35状态，也不停止串口遥测。
 * 请求在中断中只置位，因此同一周期内只有最后一次状态切换生效。
 * 本函数在每个周期内最后执行，保证失能帧是该周期UART4上的最后一批帧；
 * 其余停车路径统一走Debug_ChassisStop，失能后不再产生速度帧。 */
static void Debug_ServiceWheel(void)
{
  uint8_t request;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  request = s_wheel_req;
  s_wheel_req = 0U;
  if (mask == 0U) __enable_irq();
  if (request == 0U)
  {
    if (s_wheel_enable_pending &&
        (uint32_t)(HAL_GetTick() - s_wheel_enable_tick) >= 100U)
    {
      s_wheel_enable_pending = 0U;
      s_wheel_enabled = 1U;
    }
    return;
  }
  s_manual_active = s_goto_active = 0U;
  s_vision_req = 0U;
  if (VisionTrack_IsActive()) VisionTrack_Stop();
  s_vision_moving = 0U;
  s_wheel_enable_pending = 0U;
  s_wheel_enabled = 0U;
  /* 显式请求开启新一轮恢复；发送失败会在周期尾重新锁存故障。 */
  s_wheel_fault = s_wheel_disable_pending = 0U;
  MecanumControl_Stop();
  if (request == 1U)
  {
    MecanumControl_Enable();
    s_wheel_enable_tick = HAL_GetTick();
    s_wheel_enable_pending = 1U;
  }
  else
    MecanumControl_Disable();
}

/* 总线发送失败后取消运动，并尝试失能；成功提交也不冒充电机 ACK。
 * 故障保留到显式 WHEELEN。重试最多1秒，期间禁止速度命令重新使能。 */
static void Debug_ServiceWheelFault(void)
{
  uint32_t errors = ZDT_X42S_GetTxErrorCount();
  if (errors != s_wheel_error_seen && s_wheel_fault == 0U)
  {
    s_wheel_fault = s_wheel_disable_pending = 1U;
    s_wheel_fault_tick = HAL_GetTick();
    s_wheel_enable_pending = 0U;
    s_wheel_enabled = 0U;
    s_manual_active = s_goto_active = s_zdt_active = s_zdt_req = 0U;
    s_vision_req = 0U;
    if (VisionTrack_IsActive()) VisionTrack_Stop();
    s_vision_moving = 0U;
    MecanumControl_ClearTarget();
    Debug_ZdtAck(14U);
  }
  if (s_wheel_disable_pending)
  {
    uint8_t addr, ok = 1U;
    for (addr = 1U; addr <= 4U; ++addr)
      if (ZDT_X42S_Disable(addr) != HAL_OK) ok = 0U;
    if (ok || (uint32_t)(HAL_GetTick() - s_wheel_fault_tick) >= 1000U)
      s_wheel_disable_pending = 0U;
  }
  s_wheel_error_seen = ZDT_X42S_GetTxErrorCount();
}

/* 失能请求绑定原电机ID，CAN忙时由后续周期重试。 */
static void Debug_RequestDmDisable(void)
{
  s_dm_active = s_dm_start_pending = 0U;
  if (!s_dm_disable_pending)
  {
    s_dm_disable_id = s_dm_id;
    s_dm_disable_tick = HAL_GetTick();
    s_dm_disable_pending = 1U;
    s_dm_disable_fault = 0U;
  }
}

static void Debug_ServiceDmDisable(void)
{
  if (!s_dm_disable_pending) return;
  if (DmJ4310_Disable(s_dm_disable_id) == DM_J4310_OK)
  {
    s_dm_disable_pending = 0U;
  }
  else if ((uint32_t)(HAL_GetTick() - s_dm_disable_tick) >= 1000U)
  {
    s_dm_disable_pending = 0U;
    s_dm_disable_fault = 1U;
    s_dm_enable_req = s_dm_mode_req = 0U;
    Debug_ZdtAck(13U);
  }
}

/* 任务上下文执行，禁止在接收中断中阻塞发送或延时。
 * STOP/ZERO/安装参数调整/四轮使能切换优先取消测试，且撤销尚未执行的请求。
 * 切入测试先停止四轮，随后只对选中地址重新使能、发送速度和专用停止帧。 */
static void Debug_ServiceZdt(void)
{
  uint8_t request, cancel;
  int16_t args[3];
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  /* 四轮使能切换必须取消测试：否则阶段1的重新使能会让已失能的轮子重新上电。 */
  cancel = (uint8_t)(s_stop_req || s_zero_req || s_offset_req || (s_wheel_req != 0U));
  request = s_zdt_req;
  args[0] = s_zdt_args[0]; args[1] = s_zdt_args[1]; args[2] = s_zdt_args[2];
  s_zdt_req = 0U;
  if (request && !cancel) s_zdt_active = 1U;
  if (mask == 0U) __enable_irq();
  if (cancel)
  {
    /* 先记住"当时是否在跑测试"，再收尾：Debug_ZdtTestFinish 会清 s_zdt_active，
     * 顺序写反会把 CANCELLED 应答吞掉。 */
    uint8_t was_active = s_zdt_active;
    if (was_active) Debug_ZdtTestFinish();
    if (was_active || request) Debug_ZdtAck(5U);
    s_zdt_active = 0U;
    s_zdt_watch = 0U;
    return;
  }
  if (request)
  {
    uint8_t addr, old_reply[4];
    /* 测试前丢弃旧回包，防止把启动时应答误认为此次测试回复。 */
    while (ZDT_X42S_PopReply(old_reply)) { }
    s_zdt_watch = 1U;
    s_zdt_watch_cmd = 0xF6U;   /* 本次测试只认速度帧(0xF6)的回包，见 Debug_ServiceZdtReplies */
    s_zdt_watch_tick = HAL_GetTick();
    /* 单轮测试的机械前提：被测轮以外**失能**（释放锁轴），而不是 Stop。
     * Stop 只是速度归零，在驱动器的 Hold 配置下轮子仍被锁住，会给悬空单轮
     * 测试带来额外机械阻力（相邻轮拖拽、电流偏大）。被测轮则先 Stop 复位内部
     * 状态，再 Enable，之后才真正受速度命令控制。
     * 测试结束/取消时用 MecanumControl_Stop() 把四轮恢复成"0 转速 + 锁轴"，
     * 与固件内部 s_wheel_enabled=1 的状态保持一致（速度帧本身即重新使能锁轴）。 */
    for (addr = 1U; addr <= 4U; ++addr)
    {
      if (addr == (uint8_t)args[0]) { ZDT_X42S_Stop(addr); }
      else                          { ZDT_X42S_Disable(addr); }
    }
    s_zdt_addr = (uint8_t)args[0];
    s_zdt_rpm = args[1];
    s_zdt_duration = (uint32_t)args[2] * 1000U;
    if (s_zdt_rpm == 0) { Debug_ZdtTestFinish(); Debug_ZdtAck(4U); return; }
    ZDT_X42S_Enable(s_zdt_addr);
    s_zdt_tick = HAL_GetTick();
  }
  if (s_zdt_active == 1U && (uint32_t)(HAL_GetTick() - s_zdt_tick) >= 100U)
  {
    ZDT_X42S_SpeedAcc(s_zdt_addr, s_zdt_rpm < 0 ? 1U : 0U,
                      (uint16_t)(s_zdt_rpm < 0 ? -s_zdt_rpm : s_zdt_rpm), 100U);
    s_zdt_tick = HAL_GetTick();
    s_zdt_active = 2U;
    Debug_ZdtAck(3U);
  }
  else if (s_zdt_active == 2U && (uint32_t)(HAL_GetTick() - s_zdt_tick) >= s_zdt_duration)
  {
    Debug_ZdtTestFinish();
    Debug_ZdtAck(4U);
  }
}

/* 严格校验两个十进制数，拒绝多余字段、NaN和尾随字符。 */
static uint8_t Debug_ParseOffset(const char *s, float *v)
{
  uint8_t i;
  for (i = 0U; i < 2U; ++i)
  {
    const char *start = s;
    uint8_t digits = 0U;
    if (*s == '+' || *s == '-') ++s;
    while (*s >= '0' && *s <= '9') { digits = 1U; ++s; }
    if (*s == '.')
    {
      ++s;
      while (*s >= '0' && *s <= '9') { digits = 1U; ++s; }
    }
    if (!digits || !Debug_ParseFloat(start, &v[i]) ||
        !(v[i] >= -500.0f && v[i] <= 500.0f)) return 0U;
    if (i == 0U) { if (*s++ != ',') return 0U; }
    else if (*s != '\0') return 0U;
  }
  return 1U;
}

/* CAN不可用时直接拒绝CAN电机命令，避免产生待执行请求。
 * 仅拦截DM、S28、S35命令；ZDT、STOP及串口其他功能保持可用。
 * 回复复用现有队列，由任务DMA发送，中断中不阻塞发送。
 * 这里**不再**切文字模式：CAN 故障不应连带关掉 24 通道遥测，
 * 否则上位机波形整体消失，且必须靠补发 VOFA 才能恢复。 */
static uint8_t Debug_RejectCanCommand(const char *line)
{
  if (hcan2.State != HAL_CAN_STATE_LISTENING &&
      (Debug_StrCaseCmpN(line, "DM", 2U) == 0U ||
       Debug_StrCaseCmpN(line, "S28", 3U) == 0U ||
       Debug_StrCaseCmpN(line, "S35", 3U) == 0U))
  {
    Debug_ZdtAck(9U);
    return 1U;
  }
  return 0U;
}

static void Debug_ParseLine(char *line)
{
  char *equal;
  float value;

  while ((*line == ' ') || (*line == '\t'))
  {
    ++line;
  }

  if (Debug_RejectCanCommand(line)) return;
  if (Debug_ParseStepper(line)) return;

  if (Debug_StrCaseCmp(line, "STOP") == 0U)
  {
    s_stop_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "PING") == 0U)
  {
    return;
  }

  if (Debug_StrCaseCmp(line, "ZERO") == 0U)
  {
    s_zero_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "DMEN") == 0U)
  {
    s_dm_enable_req = 1U;
    return;
  }

  if ((Debug_StrCaseCmp(line, "DMOFF") == 0U) ||
      (Debug_StrCaseCmp(line, "DMSTOP") == 0U))
  {
    s_dm_disable_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "DMZERO") == 0U)
  {
    s_dm_zero_req = 1U;
    return;
  }

  if (Debug_StrCaseCmpN(line, "OPSOFFSET=", 10U) == 0U)
  {
    float v[2];
    if (Debug_ParseOffset(line + 10U, v))
    {
      /* OPSOFFSET 与统一坐标同序：X=左偏移、Y=前偏移。
       * 例如 OPSOFFSET=60,-50 表示 OPS 装在车左60mm、车后50mm。 */
      s_offset_x = v[0]; s_offset_y = v[1];
      s_offset_req = 1U;
    }
    return;
  }

  if (Debug_StrCaseCmp(line, "VOFA") == 0U)
  {
    s_zdt_text_mode = 0U;
    return;
  }

  /* GET <名称>：回读一个可调参数（如 GET XVMIN），应答走文字队列。
   * 24通道遥测没有 XVMIN/ZVMIN 的通道位，上位机的"回读"栏只能靠这条
   * 文本应答取得当前 RAM 值；格式固定为 "<名称>=<值>"，KPX 等有通道的
   * 参数同样可回读，便于核对遥测通道与RAM值是否一致。 */
  if (Debug_StrCaseCmpN(line, "GET ", 4U) == 0U)
  {
    Debug_ReplyParam(line + 4U);
    return;
  }

  /* 底盘四轮锁轴/释放：中断只置请求，UART4帧由任务入队。
   * 失能只释放锁轴，不影响DM、28/35和串口遥测；失能期间的运动命令
   * 在本函数后面的MANUAL/GOTO/ZDT分支被丢弃，必须显式WHEELEN恢复。
   * STOP/ZERO/OPSOFFSET不是使能命令，失能后它们只清目标不发速度帧。 */
  if (Debug_StrCaseCmp(line, "WHEELEN") == 0U)
  {
    s_wheel_req = 1U;
    return;
  }

  if (Debug_StrCaseCmp(line, "WHEELOFF") == 0U)
  {
    s_wheel_req = 2U;
    return;
  }

  /* VTRACK=1..6按颜色启动物料居中跟踪，VTRACK=0停止。
   * 中断只提交请求，视觉帧发送和底盘运动均由默认任务执行。 */
  if (Debug_StrCaseCmpN(line, "VTRACK=", 7U) == 0U)
  {
    const char *color = line + 7U;
    if (color[0] < '0' || color[0] > '6' || color[1] != '\0')
    {
      Debug_ZdtAck(DEBUG_ACK_VTRACK_FORMAT);
    }
    else if (color[0] == '0')
    {
      s_vision_req = 0xFFU;
      Debug_ZdtAck(DEBUG_ACK_VTRACK_STOP);
    }
    else if (!Debug_WheelReady())
    {
      Debug_ZdtAck(10U);
    }
    else if (s_stop_req || s_zero_req || s_offset_req ||
             s_zdt_active || s_zdt_req)
    {
      Debug_ZdtAck(DEBUG_ACK_VTRACK_BUSY);
    }
    else
    {
      s_vision_req = (uint8_t)(color[0] - '0');
      Debug_ZdtAck(DEBUG_ACK_VTRACK_START);
    }
    return;
  }

  /* ZDT=地址,有符号RPM,秒数；只允许底盘1~4号，限速300、限时1~5秒。
   * 测试期间拒绝新的单轮/MANUAL/GOTO命令，防止上位机周期刷新覆盖测试。
   * 接收成功仅表示请求入队，不代表电机已应答；STOP始终可以取消。 */
  if (Debug_StrCaseCmpN(line, "ZDT=", 4U) == 0U)
  {
    int16_t v[3];
    s_zdt_text_mode = 1U;
    if (!Debug_WheelReady())
    {
      Debug_ZdtAck(10U);
      return;
    }
    if (s_stop_req || s_zero_req || s_offset_req || s_zdt_active || s_zdt_req ||
        VisionTrack_IsActive() || (s_vision_req >= 1U && s_vision_req <= 6U))
    {
      Debug_ZdtAck(1U);
      return;
    }
    if (
        Debug_ParseManual(line + 4U, v) && v[0] >= 1 && v[0] <= 4 && v[2] >= 1 && v[2] <= 5)
    {
      s_manual_active = s_goto_active = 0U;
      s_zdt_args[0] = v[0]; s_zdt_args[1] = v[1]; s_zdt_args[2] = v[2];
      s_zdt_req = 1U;
      Debug_ZdtAck(0U);
    }
    else Debug_ZdtAck(2U);
    return;
  }
  if ((s_zdt_active || s_zdt_req || VisionTrack_IsActive() ||
       (s_vision_req >= 1U && s_vision_req <= 6U)) &&
      (Debug_StrCaseCmpN(line, "MANUAL=", 7U) == 0U ||
       Debug_StrCaseCmpN(line, "GOTO=", 5U) == 0U ||
       Debug_StrCaseCmpN(line, "GOTOHOLD=", 9U) == 0U)) return;

  if (Debug_StrCaseCmpN(line, "MANUAL=", 7U) == 0U)
  {
    int16_t v[3];
    if (!s_stop_req && !s_zero_req && Debug_WheelReady() &&
        Debug_ParseManual(line + 7U, v))
    {
      s_manual_velocity[0] = v[0];
      s_manual_velocity[1] = v[1];
      s_manual_velocity[2] = v[2];
      s_manual_tick = HAL_GetTick();
      s_goto_active = 0U;
      s_manual_active = 1U;
    }
    return;
  }

  /* GOTO=x,y,z：OPS 全局定位移动到目标后结束。
   * GOTOHOLD=x,y,z：到达目标后继续位置闭环保持，人工扰动后自动纠回。
   * 两者 x/y 单位 cm（1位小数），z 可省略（保持当前航向）；
   * 对外坐标范围 ±300.0 cm，四轮失能时不接受新目标。 */
  {
    const char *args = NULL;
    uint8_t hold_mode = 0U;

    if ((Debug_StrCaseCmpN(line, "GOTO", 4U) == 0U) && (line[4] == '='))
    {
      args = line + 5U;
    }
    else if (Debug_StrCaseCmpN(line, "GOTOHOLD=", 9U) == 0U)
    {
      args = line + 9U;
      hold_mode = 1U;
    }

    if (args != NULL)
    {
      float v[3] = {0.0f, 0.0f, 0.0f};
      uint8_t n = Debug_ParseFloatList(args, v, 3U);

      if ((n >= 2U) && (Debug_WheelReady() != 0U))
      {
        /* 全部字段有效且无停车请求后才提交新状态。 */
        if (s_stop_req || s_zero_req || s_offset_req) return;
        if (n >= 3U && !(v[2] >= -3600.0f && v[2] <= 3600.0f))
        {
          Debug_ZdtAck(12U);
          return;
        }
        if (v[0] < -300.0f) { v[0] = -300.0f; }
        if (v[0] > 300.0f)  { v[0] = 300.0f; }
        if (v[1] < -300.0f) { v[1] = -300.0f; }
        if (v[1] > 300.0f)  { v[1] = 300.0f; }

        /* cm → mm：协议只表达 0.1cm，转换为整数 mm 后直接进入位置环。
         * 正负分别加减 0.5 后向零取整，等价于按最近 1mm 取整。 */
        v[0] = (v[0] >= 0.0f)
             ? (float)(int32_t)((v[0] * 10.0f) + 0.5f)
             : (float)(int32_t)((v[0] * 10.0f) - 0.5f);
        v[1] = (v[1] >= 0.0f)
             ? (float)(int32_t)((v[1] * 10.0f) + 0.5f)
             : (float)(int32_t)((v[1] * 10.0f) - 0.5f);

        s_manual_active = 0U;
        /* 与底盘统一坐标同序：X=场地左、Y=场地前，单位已换算为 mm。 */
        s_goto_x = v[0];
        s_goto_y = v[1];
        if (n >= 3U)
        {
          s_goto_z = v[2];
        }
        else
        {
          /* Z 未提供：保持当前航向。 */
          s_goto_z = zangle;
        }
        ++s_goto_generation;
        s_goto_active = (hold_mode != 0U) ? DEBUG_GOTO_HOLD : DEBUG_GOTO_MOVE;
      }
      return;
    }
  }

  equal = strchr(line, '=');
  if (equal == NULL)
  {
    return;
  }

  *equal = '\0';
  if (strchr(equal + 1, ',') == NULL && Debug_ParseFloat(equal + 1, &value))
  {
    if (((line[0] == 'D') || (line[0] == 'd')) &&
        ((line[1] == 'M') || (line[1] == 'm')))
    {
      Debug_SetDmValue(line, value);
    }
    else
    {
      Debug_SetParam(line, value);
    }
  }
}

/* USART1专用错误回调，不覆盖OPS和电机UART4的错误处理。
 * HAL在DMA接收错误后会终止接收；回调仅申请恢复，不操作TX。 */
static void DebugUsart_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) s_rx_recover = 1U;
}

/* 普通接收回调和任务共用启动逻辑。先清恢复标志再启动，避免覆盖
 * 启动完成后新到达的错误通知；短临界区内不调用任何阻塞等待接口。 */
static void DebugUsart_StartRx(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  s_rx_recover = 0U;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx, sizeof(s_rx)) == HAL_OK)
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  else
    s_rx_recover = 1U;
  if (mask == 0U) __enable_irq();
}

/* 默认任务周期恢复RX，不使用HAL_UART_DMAStop/Abort，避免连带中止TX。
 * HAL异步DMA终止尚未完成时先等待；失败保留标志，下周期继续尝试。
 * 恢复丢弃半条命令，且不更新主机心跳，原有失联保护继续生效。 */
static void DebugUsart_ServiceRx(void)
{
  if (!s_rx_recover) return;
  if (!s_rx_callbacks_ready)
  {
    /* HAL仅允许在发送状态空闲时注册；忙时不打断发送。 */
    if (huart1.gState != HAL_UART_STATE_READY) return;
    if (HAL_UART_RegisterRxEventCallback(&huart1, DebugUsart_RxEventCallback) != HAL_OK ||
        HAL_UART_RegisterCallback(&huart1, HAL_UART_ERROR_CB_ID, DebugUsart_ErrorCallback) != HAL_OK)
      return;
    s_rx_callbacks_ready = 1U;
  }
  if (HAL_DMA_GetState(huart1.hdmarx) == HAL_DMA_STATE_ABORT) return;
  if (HAL_UART_AbortReceive(&huart1) != HAL_OK) return;
  /* 部分DMA错误路径已清DMAR，但DMA句柄仍忙，单独清理RX数据流。 */
  if (HAL_DMA_GetState(huart1.hdmarx) != HAL_DMA_STATE_READY &&
      HAL_DMA_Abort(huart1.hdmarx) != HAL_OK) return;
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  s_line_len = 0U;
  DebugUsart_StartRx();
}

/* TX 单独恢复，保持 RX 和命令接收运行。100ms 远大于 100B 帧的线速时间。 */
static void DebugUsart_ServiceTx(void)
{
  uint32_t now = HAL_GetTick();
  if (huart1.gState == HAL_UART_STATE_READY)
  {
    s_tx_busy_seen = 0U;
    return;
  }
  if (s_tx_busy_seen == 0U)
  {
    s_tx_busy_seen = 1U;
    s_tx_busy_tick = now;
  }
  else if ((uint32_t)(now - s_tx_busy_tick) >= 100U)
  {
    if (HAL_UART_AbortTransmit(&huart1) == HAL_OK)
    {
      ++debug_tx_recoveries;
      s_tx_busy_seen = 0U;
    }
  }
}

static void Debug_ServiceDm(void)
{
  /* DM 状态机：先完成失能提交，再改模式/使能。 */
  if (s_dm_disable_req != 0U)
  {
    s_dm_disable_req = 0U;
    s_dm_enable_req = s_dm_mode_req = 0U;
    Debug_RequestDmDisable();
  }
  if (!s_dm_disable_pending && !s_dm_disable_fault && s_dm_start_pending != 2U &&
      (s_dm_mode_req || s_dm_enable_req))
  {
    uint8_t restart = s_dm_active || s_dm_start_pending || s_dm_enable_req;
    Debug_RequestDmDisable();
    s_dm_enable_req = restart;
    s_dm_mode_req = 0U;
    s_dm_start_pending = 2U; /* 等待失能提交完成 */
  }
  Debug_ServiceDmDisable();
  if (!s_dm_disable_pending && !s_dm_disable_fault && s_dm_start_pending == 2U)
  {
    if (DmJ4310_SetControlMode(s_dm_id, s_dm_mode) == DM_J4310_OK)
    {
      s_dm_start_pending = s_dm_enable_req ? 1U : 0U;
      s_dm_enable_req = 0U;
      s_dm_mode_tick = HAL_GetTick();
    }
  }
  if (s_dm_zero_req && !s_dm_disable_pending && !s_dm_disable_fault)
  {
    if (DmJ4310_SetZero(s_dm_id) == DM_J4310_OK) s_dm_zero_req = 0U;
  }
  if (s_dm_start_pending == 1U && !s_dm_disable_pending && !s_dm_disable_fault &&
      (uint32_t)(HAL_GetTick() - s_dm_mode_tick) >= DEBUG_DM_MODE_WAIT_MS)
  {
    if (DmJ4310_Enable(s_dm_id) == DM_J4310_OK)
    {
      s_dm_active = 1U;
      s_dm_start_pending = 0U;
    }
  }

  /* DM 电机使能后持续下发控制帧 */
  if (s_dm_active != 0U)
  {
    if (s_dm_mode == DM_J4310_CTRL_MODE_MIT)
    {
      (void)DmJ4310_MITControl(s_dm_id, s_dm_pos, s_dm_vel,
                               s_dm_kp, s_dm_kd, s_dm_torque);
    }
    else
    {
      (void)DmJ4310_PosVelControl(s_dm_id, s_dm_pos, s_dm_vel);
    }
  }

}

/* --------------------------- 对外接口 ------------------------------ */



/**
 * @brief  初始化 USART1 调试接收
 */
void DebugUsart_Init(void)
{
  memset((void *)s_stepper_req, 0, sizeof(s_stepper_req));
  memset(s_stepper_seen, 0, sizeof(s_stepper_seen));
  memset(s_stepper_watch_cmd, 0, sizeof(s_stepper_watch_cmd));
  memset(s_stepper_watch_tick, 0, sizeof(s_stepper_watch_tick));
  s_offset_req = 0U;
  s_manual_active = 0U;
  s_vision_req = 0U;
  s_vision_moving = 0U;
  s_line_len = 0U;
  s_stop_req = 0U;
  s_zero_req = 0U;
  s_goto_active = 0U;
  s_goto_x = 0.0f;
  s_goto_y = 0.0f;
  s_goto_z = 0.0f;
  /* main.c 在调用本函数前已使能四轮，这里只清除未处理的切换请求。 */
  s_wheel_req = 0U;
  s_wheel_enabled = 1U;

  /* DM 电机默认值 */
  s_dm_id = DEBUG_DM_DEFAULT_ID;
  s_dm_mode = DEBUG_DM_DEFAULT_MODE;
  s_dm_active = 0U;
  s_dm_pos = DEBUG_DM_DEFAULT_POS;
  s_dm_vel = DEBUG_DM_DEFAULT_VEL;
  s_dm_kp = DEBUG_DM_DEFAULT_KP;
  s_dm_kd = DEBUG_DM_DEFAULT_KD;
  s_dm_torque = DEBUG_DM_DEFAULT_TORQUE;
  memset(&s_dm_feedback, 0, sizeof(DmJ4310Feedback_t));
  s_dm_enable_req = 0U;
  s_dm_disable_req = 0U;
  s_dm_zero_req = 0U;
  s_dm_mode_req = 0U;
  s_dm_start_pending = 0U;
  s_dm_mode_tick = 0U;
  Debug_InitParams(); /* RX 启动前恢复，避免覆盖已经收到的调参命令。 */
  s_host_last_tick = HAL_GetTick();
  s_control_tick = s_host_last_tick;
  s_wheel_error_seen = 0U;

  /* CAN启动失败仍继续启用串口RX；提示排队等待DMA空闲，不阻塞控制任务。
   * 这里**不再**置 s_zdt_text_mode：旧实现让 CAN 失败顺带关掉整个 24 通道遥测
   * （只有收到 VOFA 才恢复），上位机表现为"连接后一直没有数据、之后串口卡死"。
   * 报错和遥测可以并存，因此只排队报错，不动遥测开关。 */
  if (hcan2.State != HAL_CAN_STATE_LISTENING)
  {
    Debug_ZdtAck(8U);
  }

  /* 启动失败也保留恢复请求，由默认任务重试。 */
  s_rx_callbacks_ready = 0U;
  s_rx_recover = 1U;
  DebugUsart_ServiceRx();
}

/**
 * @brief  发送一次 VOFA+ JustFloat 数据帧
 */
void DebugUsart_Send(void)
{
  float data[DEBUG_VOFA_CHANNELS];
  float user_x, user_y;      /* 统一坐标：X=左右(+左)、Y=前后(+车头) */
  float err_x, err_y;        /* 统一坐标下的位置误差 */
  DmJ4310Feedback_t dmFb;
  uint32_t i;
  uint32_t len;
  uint32_t primask;

  {
    uint32_t now = HAL_GetTick();
    MecanumControl_SetPeriod(now - s_control_tick);
    s_control_tick = now;
  }
  DebugUsart_ServiceTx();
  Debug_ServiceWheelFault();
  /* OPS 错误恢复和会话变化必须先于遥测/运动服务处理。OPS 重启后
   * 坐标系原点会变化，继续执行旧 GOTO 会产生错误方向，因此立即取消。 */
  OPS_ServiceRx();
  if (OPS_ConsumeSessionChanged() != 0U)
  {
    s_goto_active = 0U;
    Debug_ChassisStop();
  }


  DebugUsart_ServiceRx();
  Debug_ServiceZdtReplies();
  Debug_ServiceZdt();

  /* 上位机失联时停止仍在运行的调试动作 */
  if ((uint32_t)(HAL_GetTick() - s_host_last_tick) > DEBUG_HOST_TIMEOUT_MS)
  {
    s_stepper_req[0].action = s_stepper_req[1].action = 0U;
    s_dm_enable_req = s_dm_mode_req = s_dm_zero_req = 0U;
    if (VisionTrack_IsActive() || (s_vision_req >= 1U && s_vision_req <= 6U))
      s_vision_req = 0xFFU;
    if (s_goto_active != 0U)
    {
      s_goto_active = 0U;
      Debug_ChassisStop();
    }
    if ((s_dm_active != 0U) || (s_dm_start_pending != 0U))
    {
      s_dm_active = 0U;
      s_dm_start_pending = 0U;
      Debug_RequestDmDisable();
    }
  }

  /* 处理等待执行的命令 */
  primask = __get_PRIMASK();
  __disable_irq();
  i = s_stop_req;
  s_stop_req = 0U;
  s_stop_in_progress = (i != 0U);
  if (i != 0U)
  {
    s_manual_active = s_goto_active = 0U;
  }
  if (primask == 0U) __enable_irq();
  if (i != 0U)
  {
    s_stepper_req[0].action = s_stepper_req[1].action = 0U;
    s_manual_active = 0U;
    s_goto_active = 0U;
    s_vision_req = 0xFFU;
    s_vision_moving = 0U;
    /* STOP 不是使能命令：四轮已失能时只清目标，不下发速度帧。 */
    Debug_ChassisStop();
    s_dm_active = 0U;
    s_dm_start_pending = 0U;
    s_dm_enable_req = 0U;
    s_dm_mode_req = 0U;
    Debug_RequestDmDisable();
    s_stop_in_progress = 0U;
  }

  if (s_zero_req != 0U)
  {
    s_manual_active = 0U;
    s_zero_req = 0U;
    s_goto_active = 0U;
    s_vision_req = 0xFFU;
    s_vision_moving = 0U;
    Debug_ChassisStop();
    OPS_ZeroCoordinates();
  }

  /* 应用安装参数前取消底盘动作；耗时停车/三角运算不在中断中执行。 */
  if (s_offset_req)
  {
    float x_mm, y_mm;
    primask = __get_PRIMASK();
    __disable_irq();
    x_mm = s_offset_x; y_mm = s_offset_y;   /* X=左偏移、Y=前偏移 */
    s_offset_req = 0U;
    s_manual_active = s_goto_active = 0U;
    s_vision_req = 0xFFU;
    s_vision_moving = 0U;
    if (primask == 0U) __enable_irq();
    Debug_ChassisStop();
    (void)OPS_SetMountOffset(x_mm, y_mm);
  }

  Debug_ServiceVision();

  Debug_ServiceGoto();

  Debug_ServiceManual();

  /* 四轮使能切换在ZDT服务之后，也在所有停车/清目标路径之后执行：
   * 正在运行的测试先按同一请求取消，且失能帧是该周期UART4上的最后一批帧，
   * 不会被STOP/ZERO/OPSOFFSET/GOTO随后发出的速度帧重新使能锁轴。 */
  Debug_ServiceWheel();
  Debug_ServiceWheelFault();

  Debug_ServiceDm();

  Debug_ServiceStepperReplies();
  Debug_ServiceSteppers();

  Debug_ServiceParams(); /* DMA忙也必须推进保存；每周期至多一个短Flash步骤。 */

  /* 控制服务始终执行；DMA 忙时仅跳过遥测，禁止改写仍在发送的 s_tx。
     本函数由 defaultTask 单独调用，USART1 TX 缓冲不得交给其他任务共用。 */
  if (huart1.gState != HAL_UART_STATE_READY) return;

  /* 发送缓冲区仅在DMA空闲时改写；成功提交才消费事件，忙/失败留待下次重试。
   * 应答优先于遥测；切换瞬间可能仍有上一帧遥测在途，不截断正在发送的DMA。 */
  if (s_ack_read != s_ack_write)
  {
    uint16_t reply_len;
    if (s_ack_queue[s_ack_read] == 7U)
    {
      static const char hex[] = "0123456789ABCDEF";
      uint8_t j;
      /* 保留原始状态码，不将未知状态解释为成功；02仅代表命令正确应答。 */
      memcpy(s_tx, "ZDT RX: ", 8U);
      for (j = 0U; j < 4U; ++j)
      {
        uint8_t value = s_ack_frames[s_ack_read][j];
        s_tx[8U + j * 3U] = (uint8_t)hex[value >> 4];
        s_tx[9U + j * 3U] = (uint8_t)hex[value & 15U];
        s_tx[10U + j * 3U] = ' ';
      }
      s_tx[19] = '\r'; s_tx[20] = '\n';
      reply_len = 21U;
    }
    else if (s_ack_queue[s_ack_read] == DEBUG_ACK_STEPPER)
    {
      reply_len = Debug_FormatStepperEvent((char *)s_tx, sizeof(s_tx),
                                           &s_ack_stepper[s_ack_read]);
      if (reply_len == 0U)
      {
        s_ack_read = (uint8_t)((s_ack_read + 1U) % 16U);
        return;
      }
    }
    else if (s_ack_queue[s_ack_read] == DEBUG_ACK_PARAM_TEXT)
    {
      /* 动态文本按槽存放：读槽与写槽永不相等，任务侧取用安全。 */
      const char *reply = s_ack_param[s_ack_read];
      reply_len = (uint16_t)strlen(reply);
      memcpy(s_tx, reply, reply_len);
    }
    else
    {
      const char *reply = s_ack_text[s_ack_queue[s_ack_read]];
      reply_len = (uint16_t)strlen(reply);
      memcpy(s_tx, reply, reply_len);
    }
    if (HAL_UART_Transmit_DMA(&huart1, s_tx, reply_len) == HAL_OK)
      s_ack_read = (uint8_t)((s_ack_read + 1U) % 16U);
    return;
  }
  if (s_zdt_text_mode) return;

  /* 读取 DM 反馈快照 */
  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(&dmFb, &s_dm_feedback, sizeof(DmJ4310Feedback_t));
  if (primask == 0U)
  {
    __enable_irq();
  }

  /* 手动旋转/静止调试同样刷新补偿后位置，不依赖GOTO运行。 */
  MecanumControl_GetPose(&pos_x, &pos_y, &zangle);
  /* 底盘内部已经使用统一坐标：ch0=X=左右、ch1=Y=前后、ch2=Z=航向角。
   * ch6/ch7 直接回读 mKpx/mKpy；通道总数与帧格式不变。 */
  user_x = pos_x;
  user_y = pos_y;
  err_x = devx;
  err_y = devy;
  /* 对外位置/误差统一为 cm；内部 pose/误差仍为 mm，仅在此打包边界缩放。 */
  data[0]  = user_x * 0.1f;
  data[1]  = user_y * 0.1f;
  data[2]  = zangle;
  data[3]  = err_x * 0.1f;
  data[4]  = err_y * 0.1f;
  data[5]  = devz;
  data[6]  = mKpx;
  data[7]  = mKpy;
  data[8]  = mKpz;
  data[9]  = XYVmax;
  data[10] = ZVmax;
  data[11] = (float)SpeedTarget[0];

  /* DM 电机调试反馈 */
  data[12] = (float)s_dm_id;
  data[13] = dmFb.position;
  data[14] = dmFb.velocity;
  data[15] = dmFb.torque;
  data[16] = (float)dmFb.status;
  data[17] = (float)dmFb.tempMos;
  data[18] = (float)dmFb.tempRotor;
  data[19] = s_dm_pos;
  data[20] = s_dm_vel;
  data[21] = s_dm_kp;
  data[22] = s_dm_kd;
  data[23] = s_dm_torque;

  for (i = 0U; i < DEBUG_VOFA_CHANNELS; ++i)
  {
    memcpy(&s_tx[4U * i], &data[i], 4U);
  }

  s_tx[4U * DEBUG_VOFA_CHANNELS]       = DEBUG_VOFA_TAIL0;
  s_tx[4U * DEBUG_VOFA_CHANNELS + 1U] = DEBUG_VOFA_TAIL1;
  s_tx[4U * DEBUG_VOFA_CHANNELS + 2U] = DEBUG_VOFA_TAIL2;
  s_tx[4U * DEBUG_VOFA_CHANNELS + 3U] = DEBUG_VOFA_TAIL3;
  len = sizeof(s_tx);

  if (huart1.gState == HAL_UART_STATE_READY)
  {
    if (HAL_UART_Transmit_DMA(&huart1, s_tx, (uint16_t)len) != HAL_OK)
      ++debug_tx_errors;
  }
}

/**
 * @brief  USART1 空闲接收回调
 */
void DebugUsart_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;

  if (huart->Instance != USART1)
  {
    return;
  }

  /* 恢复期间可能仍有旧DMA事件到达，不解析可能损坏的半帧。 */
  if (s_rx_recover) return;

  if (Size > sizeof(s_rx))
  {
    Size = (uint16_t)sizeof(s_rx);
  }

  for (i = 0U; i < Size; ++i)
  {
    uint8_t ch = s_rx[i];

    if ((ch == '\r') || (ch == '\n'))
    {
      if (s_line_len > 0U)
      {
        s_line[s_line_len] = '\0';
        s_line_len = 0U;
        s_host_last_tick = HAL_GetTick();

        Debug_ParseLine((char *)s_line);
      }
    }
    else if (s_line_len < (DEBUG_LINE_SIZE - 1U))
    {
      s_line[s_line_len++] = ch;
    }
    else
    {
      s_line_len = 0U;
    }
  }

  /* 正常包立即重启；失败后由任务清理RX状态并重试。 */
  DebugUsart_StartRx();
}


/**
 * @brief  CAN 接收回调：解析 DM 电机反馈并缓存
 * @note   实现 hcan.c 中的 __weak 钩子
 */
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
  DmJ4310Feedback_t fb;
  uint8_t rxData[8];

  if ((hcan == NULL) || (hcan->Instance != CAN2))
  {
    return;
  }

  if (hcanRxFrame.RTR != CAN_RTR_DATA || hcanRxFrame.DLC > 8U) return;

  if (hcanRxFrame.IDE == CAN_ID_EXT)
  {
    memcpy(rxData, (const uint8_t *)hcanRxFrame.Data, sizeof(rxData));
    (void)Stepper2835_OnRx(hcanRxFrame.ExtId, rxData, hcanRxFrame.DLC);
    return;
  }

  if ((hcanRxFrame.IDE != CAN_ID_STD) || (hcanRxFrame.DLC != 8U))
  {
    return;
  }

  memcpy(rxData, (const uint8_t *)hcanRxFrame.Data, sizeof(rxData));

  /* 忽略寄存器应答帧 */
  if ((rxData[1] <= 0x0FU) &&
      ((rxData[2] == 0x33U) || (rxData[2] == 0x55U) || (rxData[2] == 0xAAU)) &&
      (rxData[3] <= 81U))
  {
    return;
  }

  if (DmJ4310_DecodeFeedback(rxData, hcanRxFrame.DLC, &fb) != DM_J4310_OK)
  {
    return;
  }

  if ((fb.motorId == (uint8_t)s_dm_id) ||
      (hcanRxFrame.StdId == (uint32_t)s_dm_id))
  {
    memcpy(&s_dm_feedback, &fb, sizeof(DmJ4310Feedback_t));
  }
}
