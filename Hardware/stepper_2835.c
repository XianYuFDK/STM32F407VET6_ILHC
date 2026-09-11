/**
 * @file    stepper_2835.c
 * @brief   张大头 35 / 28 步进电机驱动，移植自原工程 tower.c。
 *
 *          35 电机：控制 Z 轴升降，输入为目标高度 h。
 *          28 电机：控制悬臂伸缩，输入为夹爪中心到转台中心的半径 r。
 *          两者共用 CAN 回零、绝对位置报文，仅机械换算与电机 ID 不同。
 *
 *          阅读顺序：公共协议 → 35 电机升降 → 28 电机伸缩 → 回复处理。
 *          注释沿用原工程“功能 / 参数 / 返回 / 调用示例”的说明方式。
 */
#include "stepper_2835.h"
#include <string.h>

/* 两台电机独立缓存：下标 0 对应 35 电机，下标 1 对应 28 电机。 */
static volatile Stepper2835Reply_t s_reply[2];

/* 将配置的 CAN ID 转换为缓存下标；其他 ID 返回 -1，不进行访问。 */
static int Stepper2835_Index(uint16_t id)
{
    if (id == MOTOR35_CAN_ID) return 0;
    if (id == MOTOR28_CAN_ID) return 1;
    return -1;
}

/* ======================= 公共协议：35 / 28 共用 ======================= */

/**
 * @brief   步进电机多圈回零位控制，对应原工程 Motor_Homing()。
 * @param   id 电机扩展 ID：35 使用 MOTOR35_CAN_ID，28 使用 MOTOR28_CAN_ID。
 * @return  HAL_OK 命令已提交；HAL_BUSY 邮箱忙；HAL_ERROR 参数或总线错误。
 * @note    发送 9A 02 00 6B，保持原回零模式。调用后不等待回零完成。
 *          本函数会触发实际运动，与 OPS 的本地坐标清零不是同一操作。
 * @example Motor_Homing(MOTOR35_CAN_ID);  // 35 升降电机回零
 *          Motor_Homing(MOTOR28_CAN_ID);  // 28 伸缩电机回零
 */
HAL_StatusTypeDef Motor_Homing(uint16_t id)
{
    const uint8_t data[4] = {0x9A, 0x02, 0x00, 0x6B};
    if (Stepper2835_Index(id) < 0) return HAL_ERROR;
    return CAN_SendEXData(HCAN_CAN_NUM, id, data, sizeof(data));
}

/**
 * @brief   步进电机绝对位置模式，对应原工程 Motor_AbsPosition()。
 * @param   dir 目标位置方向，0 / 1；实际机械正反方向由接线和驱动配置决定。
 * @param   id 35 电机为 0x300，28 电机为 0x400（以头文件配置为准）。
 * @param   step 目标位置计数，范围 0..UINT32_MAX，不是高度或长度。
 * @param   speed 电机转速，单位 RPM，16 位字段；不是线速度 mm/s。
 * @return  HAL_OK 两包已提交；HAL_BUSY 无足够邮箱；HAL_ERROR 参数或提交错误。
 * @note    原注释称 step 为“步数、一步 1.8度”；本驱动只原样发送计数，
 *          实际角度还取决于驱动器微步配置，不能据此直接认定 1计数=1.8度。
 *          两个扩展帧分别使用 id、id+1，高字节在前；提交成功不代表到位。
 * @example Motor_AbsPosition(0, MOTOR35_CAN_ID, 1000, 1000);
 *          // 35 电机以 1000 RPM 向方向 0 的绝对位置计数 1000 运动。
 */
HAL_StatusTypeDef Motor_AbsPosition(uint8_t dir, uint16_t id, uint32_t step, uint16_t speed)
{
    /* 保留原协议的加减速字段、第二包命令字、绝对位置标志及校验尾。 */
    uint8_t data[16] = {0xFD, 0, 0xAF, 0xFF, 0xAF, 0xFF, 0, 0,
                        0xFD, 0, 0, 0, 0, 0x01, 0x00, 0x6B};
    uint32_t primask;
    HAL_StatusTypeDef status;
    if (dir > 1U || Stepper2835_Index(id) < 0) return HAL_ERROR;
    data[1] = dir;
    data[6] = (uint8_t)(speed >> 8);
    data[7] = (uint8_t)speed;
    data[9] = (uint8_t)(step >> 24);
    data[10] = (uint8_t)(step >> 16);
    data[11] = (uint8_t)(step >> 8);
    data[12] = (uint8_t)step;

    /* 先确认有两个邮箱，减少仅提交半条指令；临界区只覆盖快速提交。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((HCAN_CAN_NUM)->State != HAL_CAN_STATE_LISTENING)
        status = HAL_ERROR;
    else if (HAL_CAN_GetTxMailboxesFreeLevel(HCAN_CAN_NUM) < 2U)
        status = HAL_BUSY;
    else
        status = Can_SendCmd(id, data, sizeof(data));
    if (primask == 0U) __enable_irq();
    /* HAL_OK 仅表示提交成功，不保证总线送达或机械到位。 */
    return status;
}

/* ======================= 35 电机：Z 轴升降控制 ======================= */

/**
 * @brief   35 步进电机绝对位置模式，控制 Z 轴目标高度。
 * @param   h 目标高度，单位 0.1mm，例如 1000 表示 100.0mm。
 * @param   speed Z 轴线速度，单位 mm/s，例如 50 表示 50mm/s。
 * @return  继承公共位置接口的状态；换算后 RPM 超过 65535 返回 HAL_ERROR。
 * @note    按原车标定：回零高度 203.0mm，向下位移最多 160.0mm。
 *          输入高度 h → 向下行程 max(2030-h,0) → 限幅 1600
 *          → 位置计数=行程×44.94；电机 RPM=线速度×30。
 *          因此目标高度越小，发送的位置计数越大；超范围目标会被钳位。
 *          这些数值来自原车机构，换丝杆、减速机构或零点后需要重新标定。
 * @example Motor35_AbsPosition(1000, 50);
 *          // 目标高度100.0mm、线速度50mm/s：行程1030，计数46288，转速1500RPM。
 */
HAL_StatusTypeDef Motor35_AbsPosition(uint32_t h, uint16_t speed)
{
    /* 35：回零点在高处，将目标高度换算成从高处向下的行程，避免无符号减法下溢。 */
    uint32_t travel = h >= MOTOR35_HOME_HEIGHT ? 0U : MOTOR35_HOME_HEIGHT - h;
    uint32_t rpm = (uint32_t)speed * 30U;
    if (travel > MOTOR35_MAX_TRAVEL) travel = MOTOR35_MAX_TRAVEL;
    /* 原车每 0.1mm 对应 44.94 个协议计数；限幅后乘法不会溢出。 */
    if (rpm > 65535U) return HAL_ERROR;
    return Motor_AbsPosition(MOTOR35_DIR, MOTOR35_CAN_ID, travel * 4494U / 100U, (uint16_t)rpm);
}

/* ======================= 28 电机：悬臂伸缩控制 ======================= */

/**
 * @brief   28 步进电机绝对位置模式，控制悬臂伸出后的目标半径。
 * @param   r 夹爪中心到转台中心的目标距离，单位 0.1mm；不是额外伸出量。
 * @param   speed 悬臂伸缩线速度，单位 mm/s。
 * @return  继承公共位置接口状态：HAL_OK / HAL_BUSY / HAL_ERROR。
 * @note    按原车标定：最小半径120.0mm，额外伸出行程最多166.0mm。
 *          输入半径 r → 伸出行程 max(r-1200,0) → 限幅1660
 *          → 位置计数=行程×3.189；电机 RPM=线速度×0.53，取整数。
 *          原示例 Motor28_AbsPosition(1000,50) 小于最小半径，实际被钳位到
 *          120.0mm、位置计数0，并不会到达100.0mm，不能直接照抄原示例含义。
 * @example Motor28_AbsPosition(2000, 50);
 *          // 目标半径200.0mm、线速度50mm/s：伸出800，计数2551，转速26RPM。
 */
HAL_StatusTypeDef Motor28_AbsPosition(uint32_t r, uint16_t speed)
{
    /* 28：从最小半径向外伸出，先处理小于零点的输入，避免无符号减法下溢。 */
    uint32_t travel = r <= MOTOR28_HOME_RADIUS ? 0U : r - MOTOR28_HOME_RADIUS;
    uint32_t rpm = (uint32_t)speed * 53U / 100U;
    if (travel > MOTOR28_MAX_TRAVEL) travel = MOTOR28_MAX_TRAVEL;
    /* 原车每 0.1mm 对应 3.189 个协议计数，按正数截断，与原接口一致。 */
    return Motor_AbsPosition(MOTOR28_DIR, MOTOR28_CAN_ID, travel * 3189U / 1000U, (uint16_t)rpm);
}

/* ======================= 公共回复：按电机 ID 分开缓存 ======================= */

/**
 * @brief   接收回调内部缓存回复：0x300 更新35电机，0x400 更新28电机。
 * @param   id 已由外层确认的扩展数据帧 ID。
 * @param   data 回复内容，至少包含 length 字节；函数内复制，不保留指针。
 * @param   length 有效字节数，范围1..8。
 * @return  1 已缓存；0 参数错误或非本模块电机。
 * @note    对应原 Response_Handle() 的电机分派，但不沿用“收到帧即完成”。
 *          当前仅缓存原始回复、时间和计数，未解析到位码；由 CAN RX 中断调用。
 */
uint8_t Stepper2835_OnRx(uint32_t id, const uint8_t *data, uint8_t length)
{
    int index;
    if (id > 65535U || data == NULL || length == 0U || length > 8U) return 0U;
    index = Stepper2835_Index((uint16_t)id);
    if (index < 0) return 0U;
    memset((void *)s_reply[index].data, 0, sizeof(s_reply[index].data));
    memcpy((void *)s_reply[index].data, data, length);
    s_reply[index].length = length;
    s_reply[index].last_tick = HAL_GetTick();
    /* 饱和计数，防止溢出回到零后 GetReply 误判为从未接收。 */
    if (s_reply[index].count != UINT32_MAX) ++s_reply[index].count;
    return 1U;
}

/**
 * @brief   任务读取指定电机的回复快照，不清除另一台电机或当前电机的缓存。
 * @param   id MOTOR35_CAN_ID / MOTOR28_CAN_ID。
 * @param   reply 输出结构体，不能为 NULL。
 * @return  1 曾收到回复；0 参数错误或尚未收到。返回1不代表回复仍新鲜或已到位。
 * @note    使用 HAL_GetTick()-reply.last_tick 判断时效；复制时恢复原中断状态。
 * @example Stepper2835_GetReply(MOTOR28_CAN_ID, &reply); // 读取28伸缩电机回复
 */
uint8_t Stepper2835_GetReply(uint16_t id, Stepper2835Reply_t *reply)
{
    int index = Stepper2835_Index(id);
    uint32_t primask;
    if (index < 0 || reply == NULL) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(reply, (const void *)&s_reply[index], sizeof(*reply));
    if (primask == 0U) __enable_irq();
    return reply->count != 0U;
}
