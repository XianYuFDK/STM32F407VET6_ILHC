# Hardware

### 2026-09-10 驱动修正与注释

28/35 和 OLED 的头文件已逐个说明参数单位、返回值、边界、刷新与调用限制。CAN 公共发送层改为局部发送头，短临界区保护邮箱提交；短帧先复制到八字节本地缓冲，避免 HAL 固定读取八字节时越过四字节回零命令。DLC 仍保持实际长度。分包提前检查整个 ID 范围，保留 HAL_BUSY 返回；本接口仍不保证总线原子送达。

28/35 在 CAN 未启动时返回 HAL_ERROR，回复计数饱和防止回绕误判。OLED 修复12像素字模填充位、非整页图片超范围绘制及滚动重新开始问题；原字库 GBK 注释已转 UTF-8，字模数据未改变。调试 USART1 在 DMA 忙时跳过遥测打包，不覆盖在途发送缓冲，控制处理仍继续执行。

主机回归测试说明见 `Tests/hardware/README.md`；这些检查不替代控制器兼容性与机械行程实测。

## 28 / 35 步进电机 CAN 驱动

`stepper_2835.c/.h` 移植自原工程 `USER_Code/tower/tower.c/.h` 的张大头步进电机部分。复用当前 CAN1（PA11 RX / PA12 TX、1Mbps），不需要新增串口、定时器或重新启动 CAN。它和 UART4 的 `zdt_x42s` 属于不同报文接口。

| 接口 | 用途 |
| --- | --- |
| `Motor_Homing(MOTOR35_CAN_ID)` / `Motor_Homing(MOTOR28_CAN_ID)` | 发送原协议多圈回零命令，可能产生运动 |
| `Motor_AbsPosition(dir,id,step,speed)` | 方向 0/1、原协议位置计数、RPM；默认支持 0x300/0x400 |
| `Motor35_AbsPosition(h,speed)` | 原车 Z 高度换算，h 单位 0.1mm、speed 单位 mm/s |
| `Motor28_AbsPosition(r,speed)` | 原车伸缩半径换算，r 单位 0.1mm、speed 单位 mm/s |
| `Stepper2835_GetReply(id,&reply)` | 读取原始回复、计数、时间戳快照，不代表到位 |

位置指令保留原始 16 字节格式，通过 `Can_SendCmd()` 发送两个 8 字节扩展帧（ID 与 ID+1）。示例 `Motor_AbsPosition(0,0x300,1000,1000)` 的报文为：

```text
扩展 ID 0x300：FD 00 AF FF AF FF 03 E8
扩展 ID 0x301：FD 00 00 03 E8 01 00 6B
回零扩展 ID 0x300/0x400：9A 02 00 6B（DLC=4）
```

换算保留原车参数：35 的行程为 `clamp(2030-h,0,1600)`，每单位行程乘 44.94 得到协议位置计数，RPM=`speed*30`；28 的行程为 `clamp(r-1200,0,1660)`，计数乘 3.189，RPM=`speed*0.53`，整数结果向下截断。原车注释中的机械范围和这里的实际限幅不完全一致，必须以新机构标定为准，不能直接认定为新车安全行程。

发送返回 `HAL_OK` 仅说明提交成功；两个邮箱不足返回 `HAL_BUSY`，应在任务后续周期重试，不能在中断中等待。连续发送两台位置指令可能遇到忙，不要默认同一周期都成功。35 的速度换算超出 16 位时拒绝发送，避免原代码截断回绕。

CAN 扩展数据帧通过 `debug_usart.c` 的现有 `CAN_Rx_Callback()` 分派给本驱动；保留标准帧 DM 分支。原项目仅根据 ID 设置“完成标志”，未校验应答内容，本次不沿用该到位判断。回复协议确认前只能检查原始数据及新鲜度。

只移植驱动和原车换算接口，未加入 `Tower_Control`、舵机、G6220、启动自动回零或新串口调试命令。当前 `STOP` / PING 失联保护不作用于这两台电机。首次调用回零/位置接口前，需要确定机械零点、行程及驱动器配置；协议计数与微步/实际角度的关系需按驱动器确认。

```c
#include "stepper_2835.h"

/* 在 CAN 已启动的任务中按需调用；发送结果需由调用方处理。 */
HAL_StatusTypeDef status = Motor_AbsPosition(0, MOTOR35_CAN_ID, 1000, 100);
/* status == HAL_BUSY 时留到后续任务周期重试，不忙等待。 */
```

## OLED 显示驱动

- `OLED_SoftSPI.c/.h`、`ZJY_oledfont.h`：移植自 Logistics_Vehicle_F407_V2.7.4_OpenSource 的 `USER_Code/OLED_SoftSPI/`，保留 `SoftSPI_OLED_*` 接口和原始字模。
- 采用软件 SPI，128×64 可见像素；`GRAM[144][8]` 的后 16 列用于滚动暂存，占 1152 字节 RAM。字库仅由驱动源文件包含。
- 接线：SCL/CLK → PB13，SDA/DIN → PC3，RES → PE2，DC → PE3，CS → PE4；GND 共地，供电按屏幕模块规格连接。这里的 SCL/SDA 是 SPI 时钟/数据，不是 I²C。
- GPIO 在 `SoftSPI_OLED_Init()` 内按端口分别初始化，`.ioc` 同步预留引脚；改接线时同步修改头文件宏、端口时钟及 `.ioc`。
- `main.c` 已调用初始化，上电复位、清屏；初始化包含约 220ms 阻塞等待。保留原工程控制器初始化序列，具体屏幕型号兼容性需实机确认。
- 字符支持 6×8、6×12、8×16、12×24；汉字通过原字库索引显示，不是 UTF-8 字符串渲染。坐标是像素，不是页号。
- 绘制函数修改显存，随后调用 `SoftSPI_OLED_Refresh()`；`SoftSPI_OLED_Clear()` 会同时清除显存并刷新。
- 普通字符/汉字 `mode=1` 为正常点亮，`mode=0` 为反色；`ShowBN()` 保留原阴码字库处理方式，显示效果按原字模解释。
- 修正跨端口 GPIO 初始化、显存/字库索引越界、64 像素字模长度溢出、画线端点及零半径圆死循环。
- `SoftSPI_OLED_ScrollDisplay()` 改为单步滚动，每次一列，需由任务周期调用；同一实例只维护一组滚动状态。不要使用原版依赖无限循环的调用方式。
- 刷新为同步软件 SPI，应在同一任务内串行操作，按显示需要低频刷新；不要在中断中调用，也不要每个字符单独刷新。

任务中使用示例（初始化已在 main 中完成）：

```c
#include "OLED_SoftSPI.h"

SoftSPI_OLED_Clear();
SoftSPI_OLED_ShowString(0, 0, (uint8_t *)"ILHC READY", 16, 1);
SoftSPI_OLED_ShowNum(0, 16, 1234, 4, 16, 1);
SoftSPI_OLED_Refresh();
```

原工程还有依赖硬件 SPI1 的 `OLED_SPI.c`，本次移植的是主任务使用的软件 SPI 版本，两套接口不能混用。原始参考路径：`E:\STM32\ILHC\开源代码\Logistics_Vehicle_F407_V2\2025智能物流搬运_电控_XAUT_20250811\Logistics_Vehicle_F407_V2.7.4_OpenSource\USER_Code\OLED_SoftSPI`。

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
  - USART2 + 空闲中断 + DMA（DMA1_Stream5 / Ch4），64 字节流式解析缓冲
  - 上行 V1：0x5C | float x | float y | float z | CRC8（14 字节，兼容旧 OPS）
  - 上行 V2：0x5D | ver=1 | len=28 | flags | seq | session_id | timestamp |
    float x/y/z | CRC16，小端；要求 POS_VALID/IMU_ONLINE/ENC_VALID 同时有效
  - 下行命令：0xC5 0x22（复位）；兼容旧 OPS 先发 0xC5 0x30，
    再发 0xC5 0x32 选择新协议方向 2，避免方向命令生效后坐标轴翻转
  - 使用：OPS_Init() 初始化；OPS_GetPosition(&x, &y, &z) 读取新坐标；
    OPS_IsOnline() 判断位姿是否在超时窗口内；
    OPS_ConsumeSessionChanged() 读取 V2 复位/重连事件
  - session_id 运行期变化时，任务层取消旧 GOTO；接收端自动把新会话首帧
    重设为本地零点参考，避免 OPS 复位后沿用旧原点
  - USART2 错误回调只置恢复请求，默认任务通过 OPS_ServiceRx() 重挂 DMA
  - 统一轴序：X=左右（+车左）、Y=前后（+车头）、Z 逆时针为正；内部与协议同序同号，
    不再做 X/Y 交换或取反。OPS 原始帧到统一坐标固定为 `X=-raw_y、Y=-raw_x`，
    不存在方向模式或其他分支。位置、绝对坐标、ZERO、`OPS_SetOrigin()` 和
    安装偏心补偿共用这组固定映射。
  - 坐标清零：OPS_ZeroCoordinates() 以当前位置和航向为原点，OPS_ClearZero() 恢复绝对 X/Y/Z，OPS_SetOrigin(x, y) 手动设 X/Y 零点并归零航向

## 底盘移动控制

- zdt_x42s.c / zdt_x42s.h：张大头 ZDT_X42S 闭环步进电机驱动
  - UART4（PA0=TX，PA1=RX），115200/8N1
  - Emm 固件速度模式：地址 + 0xF6 + 方向 + 速度 + 加速度 + 同步 + 0x6B
  - 支持使能、失能、立即停止、速度模式控制
- mecanum_control.c / mecanum_control.h：麦克纳姆轮底盘控制
  - O 型麦轮四轮速度解算
  - 速度模式连续移动：MecanumControl_MoveVelocity(vxRpm, vyRpm, vzRpm)，参数顺序为
    (X=左右, Y=前后, Z=旋转)；协议 `MANUAL=X,Y,W` 原序传入，不交换、不取反。
    实车轮序为俯视左前1、右前2、左后3、右后4；麦轮矩阵保持 O 型逻辑轮速公式，
    SetMotorVoltageAndDirection() 只把逻辑轮速正号转 CW、负号转 CCW，不再额外翻转
    2/4 号电机。W 的最终下发图案为 `[+ - + -]`，A 左移为 `[- - + +]`
  - OPS 全局定位 GOTO：MecanumControl_GotoOPS(x, y, yaw, maxRpm)，x=X=左右、y=Y=前后；
    协议 `GOTO=X,Y,Z` 原序传入
  - 参考开源底盘：chassis_move(x, y, z) + SetMotorVoltageAndDirection(SpeedTarget[0..3])，
    形参按统一顺序 x=X=左右、y=Y=前后
  - 通过 OPS_GetPosition() 读取定位反馈，P 比例控制 + 斜坡限制 + 到位判断；误差定义为
    `目标 - 当前`，与 ops.c 置零后的物理正向坐标配套（两者必须成对，否则位置环为正反馈）
  - 24 通道遥测直接输出 ch0/ch1=X(左右)/Y(前后)、ch3/ch4、ch6/ch7；位置和误差对外
    以 cm 输出（1 位小数），麦轮混控与 ZDT 方向映射保持上一版逻辑
  - OPS 原始 m/rad 在底盘层统一转换为 mm/deg；GOTO 的 cm 在协议边界换算为 mm 后进入位置环，
    定位数据超过 200ms 未更新自动停车
  - 四轮锁轴：MecanumControl_Enable() 使能并保持位置（内部等待 100ms）、
    MecanumControl_Disable() 失能不锁轴（不等待）、MecanumControl_Stop() 停车并下发
    速度 0 帧、MecanumControl_ClearTarget() 只清目标不发帧
  - 失能后必须用 ClearTarget：ZDT_X42S 在速度模式下收到速度命令会重新使能锁轴
  - 调用顺序：MX_UART4_Init -> OPS_Init -> MecanumControl_Init -> MecanumControl_Enable

## USART1 调试模块

- debug_usart.c / debug_usart.h：USART1 调试调参
  - PA9=TX，PA10=RX，115200/8N1
  - TX：DMA 发送 VOFA+ JustFloat 数据帧
  - RX：DMA 空闲中断接收 ASCII 命令
  - 命令示例：KPX=3.0、KPY=3.0、KPZ=10.0、XVMAX=1600、ZVMAX=750、STOP、ZERO
  - 轴归属：KPX 写 mKpx（X 左右）、KPY 写 mKpy（Y 前后），范围仍 0~50
  - MANUAL=X,Y,W、GOTO=X,Y,Z、OPSOFFSET=X(左右偏移),Y(前后偏移)，
    三者均按统一坐标直接使用；GOTO 的 X/Y 与遥测 ch0/ch1、ch3/ch4 使用 cm（1 位小数），
    OPSOFFSET 仍使用 mm
  - 四轮锁轴命令：WHEELEN（使能/锁轴）、WHEELOFF（失能/不锁轴），失能期间拒绝
    GOTO/MANUAL/ZDT；锁轴切换排在每周期最后，失能后所有停车路径只清目标不发速度帧
    （Debug_ChassisStop），见调试指令手册
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

## PCB接口分配（2026-09-11）

USART3用于摄像头预留，PE9 PWM用于夹爪舵机预留；PD0/PD1分别为VM开关/补光灯，高有效、默认关；PD2/PD3上拉按键输入，低有效。接口宏在Core/Inc/main.h，初始化在Core/Src/gpio.c。完整接线、上电行为及CAN/USB冲突见根目录PCB主控引脚说明.md。
