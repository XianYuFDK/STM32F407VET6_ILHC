/**
 ******************************************************************************
 * @file    ops.c
 * @brief   OPS 全局定位模块接收/解析驱动（硬件层）
 *
 *          - 使用 USART2 空闲中断 + DMA 接收（DMA1_Stream5 / Channel4）
 *          - 同时兼容 V1/V2：
 *              V1 14B：0x5C | float x | float y | float z | CRC8
 *              V2 28B：0x5D | ver | len | flags | seq | session_id |
 *                      timestamp_ms | float x/y/z | CRC16
 *          - 发送 2 字节命令：0xC5 0x22 复位、0xC5 0x30（旧启动兼容）、
 *            0xC5 0x32（新协议方向 2）
 *          - CRC8/CRC16 与 ops9-main 工程实现一致
 *          - 统一坐标约定：+X=车左、+Y=车头、+Z=逆时针。
 *            坐标清零采用“本地零点偏移”方式，与 ops9-main 底盘标定一致：
 *              清零后 X/Y = 相对原点位移 - 偏心旋转位移
 *              Z = 当前航向 - 清零时航向，航向也归零
 *            偏心旋转位移把"OPS 安装点"换算回"车体旋转中心"，见 OPS_CopyPosition()。
 *
 * 使用方法：
 *     OPS_Init();                    // 在 MX_USART2_UART_Init() 之后调用
 *     ...
 *     float x, y, z;
 *     if (OPS_GetPosition(&x, &y, &z)) { ... }
 *     OPS_ZeroCoordinates();         // 以当前位置作为坐标零点
 ******************************************************************************
 */
#include "ops.h"
#include "usart.h"
#include <math.h>
#include <string.h>

/* OPS 原始帧到统一坐标的唯一固定映射：
 *   X = -raw_y
 *   Y = -raw_x
 *
 * 位置、绝对位置、SetOrigin 和安装偏心补偿必须共用这组映射，
 * 不再提供方向模式配置。
 */

/* ---------------------------- 私有变量 ---------------------------- */
static uint8_t             s_rx_buf[OPS_RX_BUFFER_SIZE];   /* DMA 接收缓冲区 */
static uint8_t             s_parse_buf[OPS_RX_BUFFER_SIZE];/* 流式解析缓冲   */
static uint16_t            s_parse_len;              /* 解析缓冲有效长度      */
static volatile uint8_t    s_rx_recover;             /* USART2 错误恢复请求   */
static uint8_t             s_session_pending;        /* 新会话等待首个有效位姿 */
static OPS_Data_t          s_ops;                    /* 解析结果              */
/* OPS 光学中心相对底盘中心的安装偏移，单位 mm，统一坐标：
 *   s_mount_x_mm : X=左右，+ 为车左，+60 = 装在中心左侧 60mm
 *   s_mount_y_mm : Y=前后，+ 为车头，-50 = 装在中心后方 50mm
 * 默认值即实车安装：车后 50mm、车左 60mm。 */
static float s_mount_x_mm = 60.0f;
static float s_mount_y_mm = -50.0f;
static float s_reference_yaw; /* 首个有效帧航向，建立未清零坐标参考 */
static float s_origin_yaw;    /* ZERO时航向，与原始零点成对保存 */
static volatile uint8_t    s_new_flag;               /* 新数据标志            */

/* CRC8：生成多项式 G(x)=x^8+x^5+x^4+1，初值 0xFF（DJI RM CRC8 查表） */
static const uint8_t s_crc8_table[256] =
{
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

/* ---------------------------- 私有函数 ---------------------------- */

/**
 * @brief  计算 CRC8 校验值
 * @param  data 数据指针
 * @param  len  数据长度
 * @retval CRC8 校验值
 */
static uint8_t OPS_CalcCRC8(const uint8_t *data, uint32_t len)
{
  uint8_t crc = 0xFFU;

  while (len-- != 0U)
  {
    crc = s_crc8_table[(uint8_t)(crc ^ *data++)];
  }

  return crc;
}

/**
 * @brief  校验整帧 CRC8（最后一字节为校验值）
 * @param  buf 帧缓冲区
 * @param  len 帧长度（含校验值）
 * @retval 1 校验通过，0 校验失败
 */
static uint8_t OPS_VerifyCRC8(const uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len <= 2U))
  {
    return 0U;
  }

  return (OPS_CalcCRC8(buf, (uint32_t)(len - 1U)) == buf[len - 1U]) ? 1U : 0U;
}

/**
 * @brief  计算 CRC16（初值 0xFFFF，反射多项式 0x8408）
 * @param  data 数据指针
 * @param  len  数据长度
 * @retval CRC16 校验值
 */
static uint16_t OPS_CalcCRC16(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFU;
  uint8_t bit;

  while (len-- != 0U)
  {
    crc ^= (uint16_t)(*data++);
    for (bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0x8408U);
      }
      else
      {
        crc = (uint16_t)(crc >> 1);
      }
    }
  }

  return crc;
}

/**
 * @brief  校验 V2 帧 CRC16（小端存放）
 * @param  buf 帧缓冲区
 * @param  len 帧长度（含校验值）
 * @retval 1 校验通过，0 校验失败
 */
static uint8_t OPS_VerifyCRC16(const uint8_t *buf, uint16_t len)
{
  uint16_t expected;
  uint16_t actual;

  if ((buf == NULL) || (len <= 2U))
  {
    return 0U;
  }

  expected = (uint16_t)buf[len - 2U] | ((uint16_t)buf[len - 1U] << 8);
  actual = OPS_CalcCRC16(buf, (uint32_t)(len - 2U));
  return (actual == expected) ? 1U : 0U;
}

/**
 * @brief  丢弃解析缓冲首字节，用于逐字节重新同步
 */
static void OPS_DropFirstByte(void)
{
  if (s_parse_len > 1U)
  {
    memmove(s_parse_buf, &s_parse_buf[1], (size_t)(s_parse_len - 1U));
    s_parse_len--;
  }
  else
  {
    s_parse_len = 0U;
  }
}

/**
 * @brief  重新启动 USART2 空闲中断 + DMA 接收
 * @retval HAL 执行状态
 */
static HAL_StatusTypeDef OPS_RestartReceive(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_buf, sizeof(s_rx_buf));
  if ((status == HAL_OK) && (huart2.hdmarx != NULL))
  {
    /* 流式解析不需要半传输回调，避免高帧率时产生额外中断 */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    s_rx_recover = 0U;
  }
  else if (status != HAL_OK)
  {
    s_rx_recover = 1U;
  }

  return status;
}

/**
 * @brief  将 OPS 原始帧 X/Y 映射为统一坐标 X=左、Y=前
 * @param  raw_x OPS 原始帧 x
 * @param  raw_y OPS 原始帧 y
 * @param  x     统一 X 输出
 * @param  y     统一 Y 输出
 */
static void OPS_MapRawToUnified(float raw_x, float raw_y, float *x, float *y)
{
  *x = -raw_y;
  *y = -raw_x;
}

/**
 * @brief  将统一坐标 X=左、Y=前映射回 OPS 原始帧 X/Y
 * @param  x     统一 X
 * @param  y     统一 Y
 * @param  raw_x OPS 原始帧 x 输出
 * @param  raw_y OPS 原始帧 y 输出
 */
static void OPS_MapUnifiedToRaw(float x, float y, float *raw_x, float *raw_y)
{
  *raw_x = -y;
  *raw_y = -x;
}

/**
 * @brief  拷贝坐标并支持绝对/清零两种方式
 * @param  x X 坐标输出
 * @param  y Y 坐标输出
 * @param  z Z 坐标输出
 * @param  absolute 1 返回统一坐标的绝对位置（不补偿），0 返回补偿和清零后坐标
 * @retval 1 有新数据，0 无新数据或参数为空
 */
static uint8_t OPS_CopyPosition(float *x, float *y, float *z, uint8_t absolute)
{
  uint8_t is_new;
  uint32_t primask;

  if ((x == NULL) || (y == NULL) || (z == NULL))
  {
    return 0U;
  }

  /* 只在短临界区快照；三角运算在恢复中断后执行。 */
  {
    float px, py, yaw, ox, oy, ref, zero_yaw, rx, ry;
    float mount_x, mount_y;
    uint8_t zero, valid;
    primask = __get_PRIMASK();
    __disable_irq();
    px = s_ops.frame.x; py = s_ops.frame.y; yaw = s_ops.frame.z;
    ox = s_ops.origin_x; oy = s_ops.origin_y;
    zero_yaw = s_origin_yaw;
    zero = s_ops.zero_enabled;
    valid = ((s_ops.pose_valid != 0U) && (s_ops.valid_count != 0U)) ? 1U : 0U;
    ref = zero ? s_origin_yaw : s_reference_yaw;
    mount_x = s_mount_x_mm * 0.001f;
    mount_y = s_mount_y_mm * 0.001f;
    OPS_MapUnifiedToRaw(mount_x, mount_y, &rx, &ry);
    is_new = s_new_flag;
    s_new_flag = 0U;
    if (primask == 0U) __enable_irq();
    *z = zero ? (yaw - zero_yaw) : yaw;
    if (zero)
    {
      /* 航向零点同样按本地 ZERO 计算，并归一化到 [-π, π]。 */
      *z = fmodf(*z, 2.0f * 3.1415926536f);
      if (*z > 3.1415926536f)       { *z -= 2.0f * 3.1415926536f; }
      else if (*z < -3.1415926536f) { *z += 2.0f * 3.1415926536f; }
    }
    OPS_MapRawToUnified(px, py, x, y);
    if (valid == 0U)
    {
      *x = 0.0f;
      *y = 0.0f;
      return 0U;
    }
    /* 绝对接口只做轴映射，不叠加安装偏心补偿。 */
    if (!absolute)
    {
      /* 安装偏心补偿：先在 OPS 原始帧内把安装点换算回车体旋转中心，
       * 再按固定映射转换到统一坐标 X=左、Y=前。 */
      float dc = cosf(yaw) - cosf(ref);
      float ds = sinf(yaw) - sinf(ref);
      float dx = dc * rx - ds * ry;
      float dy = ds * rx + dc * ry;
      float raw_cx;
      float raw_cy;

      if (zero)
      {
        raw_cx = (px - ox) - dx;
        raw_cy = (py - oy) - dy;
      }
      else
      {
        raw_cx = px - dx;
        raw_cy = py - dy;
      }
      OPS_MapRawToUnified(raw_cx, raw_cy, x, y);
    }
  }

  return is_new;
}

/* ---------------------------- 对外接口 ---------------------------- */

/**
 * @brief  向 OPS 模块发送一条命令
 * @param  cmd 命令字
 * @retval HAL 执行状态
 */
HAL_StatusTypeDef OPS_SendCommand(uint8_t cmd)
{
  uint8_t buf[OPS_CMD_LEN];

  buf[0] = OPS_CMD_HEADER;
  buf[1] = cmd;

  return HAL_UART_Transmit(&huart2, buf, OPS_CMD_LEN, 100U);
}

/**
 * @brief  启动 USART2 空闲中断 + DMA 接收
 */
void OPS_Start(void)
{
  s_parse_len = 0U;
  if (OPS_RestartReceive() != HAL_OK)
  {
    s_rx_recover = 1U;
  }
  else
  {
    s_rx_recover = 0U;
  }
}

/**
 * @brief  默认任务上下文恢复 USART2 接收
 * @note   错误回调只置位，本函数负责中止残留 DMA、清错误标志并重挂接收。
 */
void OPS_ServiceRx(void)
{
  if (s_rx_recover == 0U)
  {
    return;
  }

  if ((huart2.hdmarx != NULL) &&
      (HAL_DMA_GetState(huart2.hdmarx) == HAL_DMA_STATE_ABORT))
  {
    return;
  }

  if (huart2.RxState == HAL_UART_STATE_BUSY_RX)
  {
    if (HAL_UART_AbortReceive(&huart2) != HAL_OK)
    {
      return;
    }
  }

  if ((huart2.hdmarx != NULL) &&
      (HAL_DMA_GetState(huart2.hdmarx) != HAL_DMA_STATE_READY))
  {
    if (HAL_DMA_Abort(huart2.hdmarx) != HAL_OK)
    {
      return;
    }
  }

  __HAL_UART_CLEAR_PEFLAG(&huart2);
  s_parse_len = 0U;
  if (OPS_RestartReceive() != HAL_OK)
  {
    s_rx_recover = 1U;
  }
  else
  {
    s_rx_recover = 0U;
  }
}

/**
 * @brief  初始化 OPS 模块并开始接收
 * @note   等待 OPS 模块上电稳定后，依次发送初始化/启动命令，
 *         与 ops9-main 示例工程保持一致
 */
void OPS_Init(void)
{
  memset(&s_ops, 0, sizeof(s_ops));
  s_parse_len = 0U;
  s_rx_recover = 0U;
  s_session_pending = 0U;
  s_mount_x_mm = 60.0f;
  s_mount_y_mm = -50.0f;
  s_reference_yaw = s_origin_yaw = 0.0f;
  s_ops.status = OPS_STATUS_IDLE;
  s_new_flag  = 0U;

  /* 等待 OPS 模块启动 */
  HAL_Delay(500U);
  (void)OPS_SendCommand(OPS_CMD_MODE_INIT);
  /* 0x22 会触发 OPS 控制器复位，等待其重新启动 */
  HAL_Delay(500U);
  /* 旧 OPS 用 0x30 启动；新 OPS 同时把 0x30 解释为方向 0。
   * 先兼容旧启动，再显式切回新协议文档中的方向 2，避免坐标轴被 180° 翻转。 */
  (void)OPS_SendCommand(OPS_CMD_MODE_START);
  HAL_Delay(20U);
  (void)OPS_SendCommand(OPS_CMD_DIR_2);

  /* 启动空闲中断 + DMA 接收 */
  OPS_Start();
}

/**
 * @brief  获取完整解析数据指针
 * @retval OPS_Data_t 指针
 */
const OPS_Data_t *OPS_GetData(void)
{
  return &s_ops;
}

/**
 * @brief  读取清零后的坐标（成功后自动清除新数据标志）
 * @param  x X 坐标输出
 * @param  y Y 坐标输出
 * @param  z 航向角输出
 * @retval 1 有新数据，0 无新数据或参数为空
 */
uint8_t OPS_GetPosition(float *x, float *y, float *z)
{
  return OPS_CopyPosition(x, y, z, 0U);
}

/**
 * @brief  读取 OPS 原始绝对坐标（成功后自动清除新数据标志）
 * @param  x X 坐标输出
 * @param  y Y 坐标输出
 * @param  z 航向角输出
 * @retval 1 有新数据，0 无新数据或参数为空
 */
uint8_t OPS_GetAbsolutePosition(float *x, float *y, float *z)
{
  return OPS_CopyPosition(x, y, z, 1U);
}

/**
 * @brief  查询是否有新定位数据
 * @retval 1 有新数据，0 无新数据
 */
uint8_t OPS_IsNew(void)
{
  return (s_new_flag != 0U) ? 1U : 0U;
}

/**
 * @brief  清除新数据标志
 */
void OPS_ClearNew(void)
{
  s_new_flag = 0U;
}

/**
 * @brief  判断 OPS 定位数据是否在线
 * @param  timeout_ms 允许的最大更新时间间隔
 * @retval 1 在线，0 尚无数据或数据已超时
 */
uint8_t OPS_IsOnline(uint32_t timeout_ms)
{
  uint32_t last_tick = s_ops.last_update_tick;

  if ((s_ops.pose_valid == 0U) || (s_ops.valid_count == 0U))
  {
    return 0U;
  }

  return ((uint32_t)(HAL_GetTick() - last_tick) <= timeout_ms) ? 1U : 0U;
}

/**
 * @brief  以当前 OPS 位置作为坐标零点
 * @note   必须收到过有效 OPS 帧后调用；清零后：
 *          X/Y为统一物理正向（左→+X增大、前→+Y增大）并加偏心旋转补偿，
 *          Z同时以当前航向为零点（+ 为逆时针）；是否置零不再改变X/Y的符号方向
 */
void OPS_ZeroCoordinates(void)
{
  uint32_t primask;

  if ((s_ops.pose_valid == 0U) || (s_ops.valid_count == 0U))
  {
    return; /* 尚未收到有效数据，不执行清零 */
  }

  primask = __get_PRIMASK();
  __disable_irq();
  s_ops.origin_x     = s_ops.frame.x;
  s_ops.origin_y     = s_ops.frame.y;
  s_origin_yaw       = s_ops.frame.z;
  s_ops.zero_enabled = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * @brief  取消坐标清零，恢复 OPS 原始绝对坐标
 */
void OPS_ClearZero(void)
{
  s_ops.zero_enabled = 0U;
}

/**
 * @brief  手动设置 X/Y 坐标零点（用于标定）
 * @param  x 零点对应的统一 X（+车左）
 * @param  y 零点对应的统一 Y（+车头）
 * @note   同时把当前航向设为 Z 零点。
 */
void OPS_SetOrigin(float x, float y)
{
  float raw_x;
  float raw_y;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  OPS_MapUnifiedToRaw(x, y, &raw_x, &raw_y);
  s_ops.origin_x     = raw_x;
  s_ops.origin_y     = raw_y;
  s_origin_yaw       = s_ops.frame.z;
  s_ops.zero_enabled = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * @brief  查询当前是否已坐标清零
 * @retval 1 已清零，0 未清零
 */
uint8_t OPS_IsZeroEnabled(void)
{
  return (s_ops.zero_enabled != 0U) ? 1U : 0U;
}

/**
 * @brief  读取并清除 V2 session_id 变化标志
 * @retval 1 运行期会话发生变化，0 无变化
 */
uint8_t OPS_ConsumeSessionChanged(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t changed;

  __disable_irq();
  changed = s_ops.session_changed;
  s_ops.session_changed = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return changed;
}

/**
 * @brief  过滤 NaN/Inf 和明显越界值
 */
static uint8_t OPS_FrameValuesValid(const OPS_Frame_t *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

  if (!((frame->x == frame->x) && (frame->y == frame->y) && (frame->z == frame->z)))
  {
    return 0U;
  }

  if ((fabsf(frame->x) >= 1000.0f) ||
      (fabsf(frame->y) >= 1000.0f) ||
      (fabsf(frame->z) >= 1000000.0f))
  {
    return 0U;
  }

  return 1U;
}

/**
 * @brief  发布 CRC 正确的 V1/V2 帧，并处理会话变化
 * @param  frame      已解析帧
 * @param  pose_valid 1 表示位姿可用
 */
static void OPS_PublishFrame(const OPS_Frame_t *frame, uint8_t pose_valid)
{
  uint8_t session_changed = 0U;

  if (frame->header == OPS_FRAME_HEADER_V2)
  {
    if (s_ops.session_id != frame->session_id)
    {
      if (s_ops.session_id != 0U)
      {
        session_changed = 1U;
      }
      s_ops.session_id = frame->session_id;
      s_session_pending = 1U;
    }
    s_ops.seq = frame->seq;
    s_ops.timestamp_ms = frame->timestamp_ms;
  }
  else if (s_ops.session_id != 0U)
  {
    /* 从 V2 回落到无 session 的 V1 流，按一次会话切换处理。 */
    s_ops.session_id = 0U;
    s_session_pending = 1U;
    session_changed = 1U;
  }

  s_ops.frame_count++;

  if (pose_valid != 0U)
  {
    if ((s_ops.valid_count == 0U) || (s_session_pending != 0U))
    {
      s_reference_yaw = frame->z;
      if (s_session_pending != 0U)
      {
        s_ops.origin_x = frame->x;
        s_ops.origin_y = frame->y;
        s_origin_yaw = frame->z;
        s_session_pending = 0U;
      }
    }

    memcpy(&s_ops.frame, frame, sizeof(OPS_Frame_t));
    s_ops.pose_valid = 1U;
    s_ops.valid_count++;
    s_ops.last_update_tick = HAL_GetTick();
    s_ops.status = OPS_STATUS_OK;
    s_new_flag = 1U;
  }
  else
  {
    s_ops.pose_valid = 0U;
    s_ops.status = OPS_STATUS_DATA_ERR;
  }

  if (session_changed != 0U)
  {
    s_ops.session_changed = 1U;
  }
}

/**
 * @brief  从解析缓冲发布 V1 帧
 */
static void OPS_PublishV1Frame(void)
{
  OPS_Frame_t frame;

  memset(&frame, 0, sizeof(frame));
  frame.header = OPS_FRAME_HEADER_V1;
  frame.version = 0U;
  frame.length = OPS_FRAME_LEN_V1;
  frame.flags = OPS_FLAG_POS_VALID | OPS_FLAG_IMU_ONLINE | OPS_FLAG_ENC_VALID;
  memcpy(&frame.x, &s_parse_buf[1], sizeof(frame.x));
  memcpy(&frame.y, &s_parse_buf[5], sizeof(frame.y));
  memcpy(&frame.z, &s_parse_buf[9], sizeof(frame.z));
  frame.checksum = s_parse_buf[13];

  OPS_PublishFrame(&frame, OPS_FrameValuesValid(&frame));
}

/**
 * @brief  从解析缓冲发布 V2 帧
 */
static void OPS_PublishV2Frame(void)
{
  OPS_Frame_t frame;
  uint8_t required_flags;
  uint8_t pose_valid;

  memset(&frame, 0, sizeof(frame));
  frame.header = s_parse_buf[0];
  frame.version = s_parse_buf[1];
  frame.length = s_parse_buf[2];
  frame.flags = s_parse_buf[3];
  memcpy(&frame.seq, &s_parse_buf[4], sizeof(frame.seq));
  memcpy(&frame.session_id, &s_parse_buf[6], sizeof(frame.session_id));
  memcpy(&frame.timestamp_ms, &s_parse_buf[10], sizeof(frame.timestamp_ms));
  memcpy(&frame.x, &s_parse_buf[14], sizeof(frame.x));
  memcpy(&frame.y, &s_parse_buf[18], sizeof(frame.y));
  memcpy(&frame.z, &s_parse_buf[22], sizeof(frame.z));
  frame.checksum = (uint16_t)s_parse_buf[26] |
                   (uint16_t)((uint16_t)s_parse_buf[27] << 8);

  required_flags = OPS_FLAG_POS_VALID | OPS_FLAG_IMU_ONLINE | OPS_FLAG_ENC_VALID;
  pose_valid = (((frame.flags & required_flags) == required_flags) &&
                (OPS_FrameValuesValid(&frame) != 0U)) ? 1U : 0U;

  OPS_PublishFrame(&frame, pose_valid);
}

/**
 * @brief  逐字节解析 OPS 字节流并自动重新同步
 * @param  byte 新收到的字节
 */
static void OPS_ParseByte(uint8_t byte)
{
  uint16_t expected;
  uint8_t header;
  uint8_t crc_ok;

  if (s_parse_len >= (uint16_t)sizeof(s_parse_buf))
  {
    s_ops.format_errors++;
    s_ops.error_count++;
    OPS_DropFirstByte();
  }

  s_parse_buf[s_parse_len++] = byte;

  for (;;)
  {
    if (s_parse_len == 0U)
    {
      return;
    }

    header = s_parse_buf[0];
    if (header == OPS_FRAME_HEADER_V1)
    {
      expected = OPS_FRAME_LEN_V1;
    }
    else if (header == OPS_FRAME_HEADER_V2)
    {
      if (s_parse_len < 3U)
      {
        return;
      }
      if ((s_parse_buf[1] != OPS_FRAME_VERSION_V2) ||
          (s_parse_buf[2] != OPS_FRAME_LEN_V2))
      {
        s_ops.format_errors++;
        s_ops.error_count++;
        s_ops.status = OPS_STATUS_HEADER_ERR;
        OPS_DropFirstByte();
        continue;
      }
      expected = OPS_FRAME_LEN_V2;
    }
    else
    {
      s_ops.format_errors++;
      s_ops.error_count++;
      s_ops.status = OPS_STATUS_HEADER_ERR;
      OPS_DropFirstByte();
      continue;
    }

    if (s_parse_len < expected)
    {
      return;
    }

    if (header == OPS_FRAME_HEADER_V1)
    {
      crc_ok = OPS_VerifyCRC8(s_parse_buf, OPS_FRAME_LEN_V1);
    }
    else
    {
      crc_ok = OPS_VerifyCRC16(s_parse_buf, OPS_FRAME_LEN_V2);
    }

    if (crc_ok != 0U)
    {
      if (header == OPS_FRAME_HEADER_V1)
      {
        OPS_PublishV1Frame();
      }
      else
      {
        OPS_PublishV2Frame();
      }

      if (s_parse_len > expected)
      {
        memmove(s_parse_buf, &s_parse_buf[expected],
                (size_t)(s_parse_len - expected));
      }
      s_parse_len = (uint16_t)(s_parse_len - expected);
    }
    else
    {
      s_ops.crc_errors++;
      s_ops.error_count++;
      s_ops.status = OPS_STATUS_CRC_ERR;
      /* 丢首字节继续找下一个帧头，避免噪声后长期错位。 */
      OPS_DropFirstByte();
    }
  }
}

/* ----------------------- HAL UART RX 事件回调 ----------------------- */

/**
 * @brief  USART2 空闲/接收完成回调，完成 OPS 字节流解析
 * @param  huart UART 句柄
 * @param  Size  本次接收字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;

  /* 仅处理 USART2 */
  if (huart->Instance != USART2)
  {
    return;
  }

  if (Size > (uint16_t)sizeof(s_rx_buf))
  {
    Size = (uint16_t)sizeof(s_rx_buf);
  }

  for (i = 0U; i < Size; ++i)
  {
    OPS_ParseByte(s_rx_buf[i]);
  }

  /* 解析完成后重新启动接收 */
  (void)OPS_RestartReceive();
}

/**
 * @brief  USART2 HAL 错误回调：中断中只置恢复请求，任务中重挂接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    s_ops.error_count++;
    s_rx_recover = 1U;
  }
}

/* 成对更新安装偏移；参数为统一坐标 X=左右(+左)、Y=前后(+前)，由任务停车后调用。未写入Flash。 */
uint8_t OPS_SetMountOffset(float x_mm, float y_mm)
{
  uint32_t mask;
  if (!(x_mm >= -500.0f && x_mm <= 500.0f &&
        y_mm >= -500.0f && y_mm <= 500.0f)) return 0U;
  mask = __get_PRIMASK();
  __disable_irq();
  s_mount_x_mm = x_mm;
  s_mount_y_mm = y_mm;
  if ((s_ops.pose_valid != 0U) && (s_ops.valid_count != 0U))
  {
    s_ops.origin_x = s_ops.frame.x;
    s_ops.origin_y = s_ops.frame.y;
    s_origin_yaw = s_ops.frame.z;
    s_ops.zero_enabled = 1U;
  }
  if (mask == 0U) __enable_irq();
  return 1U;
}
