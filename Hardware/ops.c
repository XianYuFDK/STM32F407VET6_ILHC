/**
 ******************************************************************************
 * @file    ops.c
 * @brief   OPS 全局定位模块接收/解析驱动（硬件层）
 *
 *          - 使用 USART2 空闲中断 + DMA 接收（DMA1_Stream5 / Channel4）
 *          - 解析 14 字节定位帧：0x5C | float x | float y | float z | CRC8
 *          - 发送 2 字节初始化/启动命令：0xC5 0x22 / 0xC5 0x30
 *          - CRC8 使用 DJI RM CRC8_CRC16.c 中的查表算法，与 ops9-main 兼容
 *          - 坐标清零采用“本地零点偏移”方式，与 ops9-main 底盘标定一致：
 *              清零后 X = OPS绝对X - 零点X - 偏心旋转位移X
 *              清零后 Y = OPS绝对Y - 零点Y - 偏心旋转位移Y
 *              Z 为航向角，不清零
 *            偏心旋转位移把"OPS 安装点"换算回"车体旋转中心"，见 OPS_CopyPosition()
 *            里的矩阵推导（内部 x=前后且正指向车尾、y=左右且正指向车左）。
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

/* ---------------------------- 私有变量 ---------------------------- */
static uint8_t             s_rx_buf[OPS_FRAME_LEN];  /* DMA 接收缓冲区        */
static OPS_Data_t          s_ops;                    /* 解析结果              */
/* OPS 光学中心相对底盘中心的安装偏移，单位 mm，用**内部车体轴**表示：
 *   s_mount_x_mm : 沿内部 x 轴（前后轴，**+ 指向车尾**）—— +50 = 装在中心后方 50mm
 *   s_mount_y_mm : 沿内部 y 轴（左右轴，+ 与车左同向）—— +60 = 装在中心左侧 60mm
 * 默认值即实车安装：车后 50mm、车左 60mm。
 * 上位机协议用对外坐标下发（OPSOFFSET=X左右(+左),Y前后(+车头)），例如实际装在
 * 车左60/车后50 时下发 OPSOFFSET=60,-50，经 debug_usart.c 适配层换算成本处 (50,60)。
 * 注意：该默认值必须与 OPS_CopyPosition() 里的偏心补偿矩阵配套，见那里的推导。 */
static float s_mount_x_mm = 50.0f;
static float s_mount_y_mm = 60.0f;
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
 * @brief  重新启动 USART2 空闲中断 + DMA 接收
 * @retval HAL 执行状态
 */
static HAL_StatusTypeDef OPS_RestartReceive(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_buf, OPS_FRAME_LEN);
  if ((status == HAL_OK) && (huart2.hdmarx != NULL))
  {
    /* 定长 14 字节帧不需要 7 字节半传输回调 */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }

  return status;
}

/**
 * @brief  拷贝坐标并支持绝对/清零两种方式
 * @param  x X 坐标输出
 * @param  y Y 坐标输出
 * @param  z Z 坐标输出
 * @param  absolute 1 返回 OPS 原始绝对坐标，0 返回清零后坐标
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
    float px, py, yaw, ox, oy, ref, rx, ry;
    uint8_t zero, valid;
    primask = __get_PRIMASK();
    __disable_irq();
    px = s_ops.frame.x; py = s_ops.frame.y; yaw = s_ops.frame.z;
    ox = s_ops.origin_x; oy = s_ops.origin_y;
    zero = s_ops.zero_enabled;
    valid = (s_ops.valid_count != 0U);
    ref = zero ? s_origin_yaw : s_reference_yaw;
    /* 实车实测（2026-09-17）：车**左移**时 OPS 原始 y 变小，而本工程约定"左 = +X"，
     * 因此 OPS 原始 y 轴与"车左"**反号**（原始帧右手系：+x=车尾、+y=车右）。
     * 处理：内部/对外 y = -原始 y；偏心补偿在原始帧里做，安装偏移的 y 分量
     * 也要按原始帧取号（s_mount_y_mm 仍约定 + = 车左）。 */
    rx =  s_mount_x_mm * 0.001f;   /* + = 车后（沿 +x_raw） */
    ry = -s_mount_y_mm * 0.001f;   /* +y_raw = 车右，而 s_mount_y_mm 约定 + = 车左 ⇒ 取反 */
    is_new = s_new_flag;
    s_new_flag = 0U;
    if (primask == 0U) __enable_irq();
    *z = yaw;
    *x = px; *y = -py;             /* x 同向；y 反号 ⇒ 左移时输出增大 */
    /* 原始绝对接口保持线缆数据，避免补偿叠加。 */
    if (!absolute && valid)
    {
      /* 安装偏心补偿：把"OPS 测到的点"换算回"车体旋转中心"。
       * 传感器刚性固定在车体上，其相对中心的位置随车体一起旋转：
       *     传感器原始帧坐标 = 中心 + R(Δθ)·m_raw
       * 原始帧是**右手系**（+x=车尾、+y=车右），因此 R 用标准 CCW 矩阵即可；
       * m_raw = (rx, ry) 为该偏移在原始帧里的分量（见上面取号）。
       * 数值自检（传感器装车后50/车左60 ⇒ m_raw=(+0.05,-0.06)，Δθ=+90°）：
       *     R(90°)·m_raw=(+0.06,+0.05)，减去 m_raw ⇒ 需减掉 (+0.01,+0.11)，
       *     与下面公式一致 ⇒ 原地旋转时补偿后的中心保持不变（不画圆）。 */
      float dc = cosf(yaw) - cosf(ref);
      float ds = sinf(yaw) - sinf(ref);
      float dx = dc * rx - ds * ry;
      float dy = ds * rx + dc * ry;
      if (zero)
      {
        /* 置零后仍为物理正向：车向前→x增大、车向左→y增大，与非置零状态一致。
         * 原实现为 ox-px 的“ZERO反号”，使符号随是否置零翻转，且位置环
         * 只在置零状态下才是负反馈；去掉反号后由 chassis_move 的
         * devx=tgt-cur 保证方向一致，轮速指令逐周期不变。 */
        *x = px - ox - dx;
        *y = -(py - oy - dy);    /* 输出端 y 取反：原始帧(+车右) → 内部/对外(+车左) */
      }
      else
      {
        *x = px - dx;
        *y = -(py - dy);         /* 同上；符号不随是否置零变化 */
      }
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
  (void)OPS_RestartReceive();
}

/**
 * @brief  初始化 OPS 模块并开始接收
 * @note   等待 OPS 模块上电稳定后，依次发送初始化/启动命令，
 *         与 ops9-main 示例工程保持一致
 */
void OPS_Init(void)
{
  memset(&s_ops, 0, sizeof(s_ops));
  s_mount_x_mm = 50.0f;
  s_mount_y_mm = 60.0f;
  s_reference_yaw = s_origin_yaw = 0.0f;
  s_ops.status = OPS_STATUS_IDLE;
  s_new_flag  = 0U;

  /* 等待 OPS 模块启动 */
  HAL_Delay(500U);
  (void)OPS_SendCommand(OPS_CMD_MODE_INIT);
  /* 0x22 会触发 OPS 控制器复位，等待其重新启动 */
  HAL_Delay(500U);
  (void)OPS_SendCommand(OPS_CMD_MODE_START);

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

  if (s_ops.valid_count == 0U)
  {
    return 0U;
  }

  return ((uint32_t)(HAL_GetTick() - last_tick) <= timeout_ms) ? 1U : 0U;
}

/**
 * @brief  以当前 OPS 位置作为坐标零点
 * @note   必须收到过有效 OPS 帧后调用；清零后：
 *          X/Y为物理正向（前→+X增大、左→+Y增大）并加偏心旋转补偿，
 *          Z保持绝对航向角；是否置零不再改变X/Y的符号方向
 */
void OPS_ZeroCoordinates(void)
{
  uint32_t primask;

  if (s_ops.valid_count == 0U)
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
 * @param  x 零点对应的绝对 X
 * @param  y 零点对应的绝对 Y
 */
void OPS_SetOrigin(float x, float y)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_ops.origin_x     = x;
  s_ops.origin_y     = y;
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

/* ----------------------- HAL UART RX 事件回调 ----------------------- */

/**
 * @brief  USART2 空闲/接收完成回调，完成 OPS 帧解析
 * @param  huart UART 句柄
 * @param  Size  本次接收字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  OPS_Frame_t frame;

  /* 仅处理 USART2 */
  if (huart->Instance != USART2)
  {
    return;
  }

  /* 长度正确且帧头正确时校验 CRC8 */
  if ((Size == OPS_FRAME_LEN) && (s_rx_buf[0] == OPS_FRAME_HEADER))
  {
    if (OPS_VerifyCRC8(s_rx_buf, OPS_FRAME_LEN) != 0U)
    {
      memcpy(&frame, s_rx_buf, sizeof(OPS_Frame_t));

      /* 拒绝 NaN、无穷值和明显越界值，避免异常数据进入运动解算 */
      if ((frame.x == frame.x) && (frame.y == frame.y) && (frame.z == frame.z) &&
          (fabsf(frame.x) < 1000.0f) && (fabsf(frame.y) < 1000.0f) &&
          (fabsf(frame.z) < 1000000.0f))
      {
        if (s_ops.valid_count == 0U) s_reference_yaw = frame.z;
        memcpy(&s_ops.frame, &frame, sizeof(OPS_Frame_t));
        s_ops.valid_count++;
        s_ops.last_update_tick = HAL_GetTick();
        s_ops.status = OPS_STATUS_OK;
        s_new_flag   = 1U;
      }
      else
      {
        s_ops.error_count++;
        s_ops.status = OPS_STATUS_DATA_ERR;
      }
    }
    else
    {
      s_ops.error_count++;
      s_ops.status = OPS_STATUS_CRC_ERR;
    }
  }
  else
  {
    s_ops.error_count++;
    s_ops.status = OPS_STATUS_HEADER_ERR;
  }

  /* 解析完成后重新启动接收 */
  (void)OPS_RestartReceive();
}

/* 成对更新安装偏移；由任务停车后调用。未写入Flash。 */
uint8_t OPS_SetMountOffset(float x_mm, float y_mm)
{
  uint32_t mask;
  if (!(x_mm >= -500.0f && x_mm <= 500.0f &&
        y_mm >= -500.0f && y_mm <= 500.0f)) return 0U;
  mask = __get_PRIMASK();
  __disable_irq();
  s_mount_x_mm = x_mm;
  s_mount_y_mm = y_mm;
  if (s_ops.valid_count != 0U)
  {
    s_ops.origin_x = s_ops.frame.x;
    s_ops.origin_y = s_ops.frame.y;
    s_origin_yaw = s_ops.frame.z;
    s_ops.zero_enabled = 1U;
  }
  if (mask == 0U) __enable_irq();
  return 1U;
}
