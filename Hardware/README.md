# Hardware

存放本项目的自定义硬件/外设驱动代码（如传感器、电机、LED、按键等）。

## 使用约定
- 与 `Core/`、`Drivers/`、`Middlewares/` 平级，位于工程根目录。
- 每个硬件模块通常为 `xxx.c` + `xxx.h`，放在这一层目录下。
- 头文件在工程中已加入包含路径：`../Hardware`（Keil 与 EIDE 均已配置）。
- 新增源文件后，请在 EIDE 工程（`.eide/eide.yml`）的 `virtualFolder` 中把对应的 `.c` 文件加入，或在 Keil 工程中把文件加入 `Hardware` 分组。

## 调试指令手册

- [调试指令手册.md](调试指令手册.md)

## OPS 定位驱动

- ops.c / ops.h：OPS 全局定位模块接收与解析驱动
  - USART2 + 空闲中断 + DMA（DMA1_Stream5 / Ch4）
  - 上行帧：0x5C | float x | float y | float z | CRC8（14 字节）
  - 下行命令：0xC5 0x22（初始化）、0xC5 0x30（启动）
  - 使用：OPS_Init() 初始化；OPS_GetPosition(&x, &y, &z) 读取新坐标
  - 坐标清零：OPS_ZeroCoordinates() 以当前位置为原点，OPS_ClearZero() 恢复绝对坐标，OPS_SetOrigin(x, y) 手动设零点

## 底盘移动控制

- zdt_x42s.c / zdt_x42s.h：张大头 ZDT_X42S 闭环步进电机驱动
  - UART4（PA0=TX，PA1=RX），115200/8N1
  - Emm 固件速度模式：地址 + 0xF6 + 方向 + 速度 + 加速度 + 同步 + 0x6B
  - 支持使能、失能、立即停止、速度模式控制
- mecanum_control.c / mecanum_control.h：麦克纳姆轮底盘控制
  - O 型麦轮四轮速度解算
  - 速度模式连续移动：MecanumControl_MoveVelocity(vx, vy, vz)
  - OPS 全局定位 GOTO：MecanumControl_GotoOPS(x, y, yaw, maxRpm)
  - 参考开源底盘：chassis_move(x, y, z) + SetMotorVoltageAndDirection(SpeedTarget[0..3])
  - 通过 OPS_GetPosition() 读取定位反馈，P 比例控制 + 斜坡限制 + 到位判断
  - OPS 原始 m/rad 在底盘层统一转换为 mm/deg，定位数据超过 200ms 未更新自动停车
- 调用顺序：MX_UART4_Init -> OPS_Init -> MecanumControl_Init -> MecanumControl_Enable

## USART1 调试模块

- debug_usart.c / debug_usart.h：USART1 调试调参
  - PA9=TX，PA10=RX，115200/8N1
  - TX：DMA 发送 VOFA+ JustFloat 数据帧
  - RX：DMA 空闲中断接收 ASCII 命令
  - 命令示例：KPX=3.0、KPY=3.0、KPZ=10.0、XVMAX=1600、ZVMAX=750、STOP、ZERO
  - DM 电机命令：DMID=1、DMEN、DMOFF、DMSTOP、DMZERO
  - DM 控制命令：DMMODE=1/2、DMPOS=3.14、DMVEL=2、DMKP=2、DMKD=1、DMTOR=0.5
  - VOFA+ 通道：0~11 为底盘，12~23 为 DM（ID/位置/速度/力矩/状态/温度/目标值）
  - 标准 JustFloat：24×float32 + 4 字节帧尾；上位机每 200ms 自动发送 PING 心跳
  - 参数表在 debug_usart.c 中集中管理，新增参数只需添加一项
## CAN 协议模块

- hcan.c / hcan.h：移植自 tower/hcan.c
  - CAN1：滤波全接收 + FIFO0 接收中断
  - 标准帧：CAN_SendData(hcan, ID, data, len)
  - 扩展帧：CAN_SendEXData(hcan, ID, data, len)
  - 长数据分包：Can_SendCmd(ID, data, len)，每包最多 8 字节
  - 接收：HCan_GetRxFrame(&frame) 读取最近一帧
  - CAN 波特率：1 Mbps（Prescaler=6, BS1=3TQ, BS2=3TQ）
## DM-J4310-2EC V1.1 驱动层（仅驱动，无应用层）

当前只保留 DM 电机驱动层，塔吊应用层已删除。

- dm_j4310.c / dm_j4310.h：达妙 DM-J4310-2EC V1.1 关节电机 CAN 驱动
  - 移植自 GitHub：https://github.com/dmBots/motor-control-routine
  - 参考源码：stm32例程/DMMotor_freertos.rar/User/bsp_can.c
  - 支持 MIT 模式、位置速度模式、使能/失能、零点保存、反馈解析
  - 支持控制模式寄存器 10 切换：MIT=1、位置速度=2
  - CAN1 1Mbps，标准帧；位置速度模式命令 ID = 电机 ID + 0x100
  - P_MAX/V_MAX/T_MAX 宏在 dm_j4310.h 中，需与电机调试工具一致

### 对外接口

- DmJ4310_MITControl(canId, pos, vel, kp, kd, torque)
- DmJ4310_PosVelControl(canId, pos, vel)
- DmJ4310_SetControlMode(canId, mode)
- DmJ4310_Enable(canId)
- DmJ4310_Disable(canId)
- DmJ4310_SetZero(canId)
- DmJ4310_DecodeFeedback(data, len, &feedback)
