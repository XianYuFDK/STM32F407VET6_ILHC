# 工程导航与维护约定

2026-09-16：修复"WHEELOFF失能后轮子仍然锁轴"。根因是任务体时序：DebugUsart_Send先执行
Debug_ServiceWheel发出失能帧，之后s_stop_req/s_zero_req/s_offset_req/GOTO分支又调用
MecanumControl_Stop()发出速度0帧；ZDT_X42S在速度模式下收到任意速度命令都会重新使能
锁轴，因此失能帧被覆盖。Qt上位机点击「失能电机」时先发STOP再发WHEELOFF，两条命令落在
同一个20ms周期，必然复现。修复：mecanum_control.c拆出MecanumControl_ClearTarget（只清
SpeedTarget/last_Speed/in_pos等，不碰UART4），MecanumControl_Stop改为ClearTarget +
SetMotorVoltageAndDirection(0,0,0,0)；debug_usart.c新增Debug_ChassisStop（使能状态发速度
帧、失能状态只清目标），任务体六处停车路径统一改用它，GOTO分支在失能时只取消目标，手动
服务在失能时只清目标；Debug_ServiceWheel移到Debug_ServiceManual之后，保证失能帧是该周期
UART4上的最后一批帧。回归Tests/hardware/test_debug_wheel.py新增Debug_ChassisStop编译用例
与"任务体不得出现裸MecanumControl_Stop"文本断言，test_debug_manual.py补闸门用例；
固件EIDE编译通过，必须重新烧录。

2026-09-15：新增底盘四轮锁轴命令WHEELEN（使能/锁轴）和WHEELOFF（失能/不锁轴）。
接收中断只置s_wheel_req，任务Debug_ServiceWheel先取消GOTO/手动并停车，再发UART4
使能或失能帧；使能闸门在MecanumControl_Enable完成后才打开。失能期间GOTO/MANUAL
在Debug_ParseLine解析阶段丢弃，ZDT回复ERR WHEEL DISABLED需先WHEELEN；STOP和主机
失联不改变锁轴状态。mecanum_control.c新增MecanumControl_Disable（1~4号
ZDT_X42S_Disable，不等待，与Enable的100ms等待不对称）；ZDT测试遇锁轴切换按既有
STOP/ZERO/OPSOFFSET规则取消，避免测试阶段1重新使能已失能的轮子。Qt「底盘调参」
新增两个按钮，失能状态下本地也拒绝地图GOTO/键盘遥控，命令终端手输同步界面状态。
无状态回读。测试Tests/hardware/test_debug_wheel.py、test_debug_zdt.py（新增锁轴
取消用例）与Qt test_debugger；固件EIDE编译通过，必须重新烧录。

2026-09-13：Qt连续遥控改为50ms PreciseTimer；core.CommandQueue合并最新MANUAL并优先普通参数，
STOP仍走紧急队列。串口read改为已有字节/空闲1字节，超时10ms；仅绘制当前页面图表。
固件350ms超时未改，未实车验证顿挫原因；新增最新目标队列、短读取及发送优先级回归。

2026-09-12：Qt手动区改为键盘遥控，WASD平移、Q/E旋转、Shift30%低速、空格停车、Esc退出。
仅主动接管且键盘区有焦点时生效，失焦/切页/断连清除按键，不自动恢复。
复用MANUAL和350ms固件超时；松开键优先发送MANUAL=0,0,0，运动中退出使用STOP。
固件本次未改动，回归包含Qt键事件、组合/反向/重复键、失焦和模拟器超时。

2026-09-11：仅修复USART1接收异常恢复。debug_usart.c注册专用ErrorCallback，
中断置s_rx_recover，DebugUsart_Send入口任务恢复RX；启动/重启失败下周期重试，
等待RX DMA异步终止完成，只AbortReceive及RX DMA，不主动中止TX。
恢复清除半条命令，不刷新主机心跳。测试：Tests/hardware/test_debug_rx_recovery.py。
其余代码审查问题（STOP竞态、DM失能、输入校验等）尚未修复。

2026-09-10 Qt/步进更新：Qt地图顺时针旋转90°后，固定右下启停区1中心为场地零点，屏幕左+X、上+Y，左下区2=(2100,0)；
OPS遥测/GOTO保持固件坐标，由main.py转换。新增28/35页：机械目标、原始位置/RPM、回零、取消待发。
debug_usart.c支持S35/S28的MOVE=h10或r10,speed、RAW=dir,count,rpm、HOME、CANCEL。
中断只写每电机一个最新请求，任务提交CAN，BUSY最多重试1秒，ERROR不重试。
STOP/失联只撤销28/35待发，不停止已发送运动；没有验证过的28/35停机协议。
24通道遥测不变，无28/35回读。详见Qt README和调试指令手册新增小节，以本更新为准。
回归入口：Qt目录python -m unittest -v test_debugger；根目录python Tests/hardware/test_debug_stepper.py。

2026-09-10 修正：CAN 短帧发送先补齐八字节本地存储，DLC 不变；局部发送头和短临界区保护邮箱提交。Can_SendCmd 提前检查整段 ID，并保留 HAL_BUSY。28/35 未启动 CAN 返回 HAL_ERROR，回复计数饱和。USART1 DMA 忙时只跳过遥测打包，继续控制服务；要求单任务发送。OLED 接口已补详细中文注释，修复图片/12像素字体填充位及滚动复位；字库注释按 GBK 转回 UTF-8。新增 `Tests/hardware` 主机回归测试，替换 HAL 验证真实 CAN/28/35/OLED 源码，不加入固件工程。剩余机械标定、28/35停止协议与统一动作层仍需后续处理。

2026-09-10 补充：`Hardware/stepper_2835.c/.h` 为 28/35 步进电机 CAN 扩展帧驱动，默认 ID 为 0x400/0x300，复用 `hcan.c`。提供 `Motor_Homing`、`Motor_AbsPosition`、`Motor35_AbsPosition`、`Motor28_AbsPosition` 和原始回复快照。位置指令两个 8 字节帧，ID 与 ID+1；邮箱不足返回 HAL_BUSY，需任务后续周期重试。机械换算参数来自原车，必须按新机构标定。接收由 `debug_usart.c` 既有 CAN 钩子分派；没有自动回零、到位判定或新增串口命令，STOP/心跳保护尚不覆盖 28/35。具体接口见 `Hardware/README.md`，当前分层及改进建议见根目录 `工程结构分析.md`。

本文件适用于整个 `STM32F407VET6_ILHC` 工程，供后续 AI 快速定位代码。根据 2026-09-08 工作区源码整理；若代码变化，以实际源码和构建清单为准，并同步更新本文件。

2026-09-10 更新：移植软件 SPI OLED。入口为 `Hardware/OLED_SoftSPI.c/.h`，字库 `Hardware/ZJY_oledfont.h` 只在驱动源文件中包含。128×64 屏幕，显存 144×8 字节（含滚动暂存区）。引脚 PB13=CLK、PC3=DIN、PE2=RES、PE3=DC、PE4=CS，由驱动初始化 GPIO，已同步 `.ioc` 和两份构建清单。

`main.c` 在外设初始化完成后的 `USER CODE BEGIN 2` 首先调用 `SoftSPI_OLED_Init()`，随后启动 CAN 和其他模块。OLED 初始化约阻塞 220ms，默认清屏，没有新增显示任务。绘制后需手动 `SoftSPI_OLED_Refresh()`，坐标单位为像素；建议同一任务低频更新。`SoftSPI_OLED_ScrollDisplay()` 已改为每次调用移动一列，不能按原版无限循环行为理解。接线、模式和调用示例见 `Hardware/README.md` 的 OLED 小节。

## 1. 快速认识工程

- 用途：工创物流搬运小车底层控制、OPS 全局定位、麦克纳姆轮运动与 DM 电机调试。
- 主控：STM32F407VET6，Cortex-M4F；HAL + FreeRTOS + CMSIS-RTOS2，C99。
- 时钟配置：8MHz HSE，经 PLL 配置为 SYSCLK 168MHz、APB1 42MHz、APB2 84MHz。核对时同时查看 `main.c`、HAL 配置及 `system_stm32f4xx.c`，不要只依据其中某个默认宏。
- 当前链接区域：Flash `0x08000000` 起、512KB；SRAM `0x20000000` 起、128KB。不要把未纳入链接配置的存储区算作可直接使用的 RAM。
- 工程根目录是本文件所在目录；EIDE/Keil 构建工作目录是子目录 `MDK-ARM/`，两者不能混淆。
- 自定义模块主要在 `Hardware/`；该目录同时包含协议驱动、底盘算法和调试动作调度，不是纯粹的硬件抽象层。
- DM 目前只有驱动及调试入口，没有塔吊应用层；RC 遥控已移除。

建议首次阅读顺序：本文件 → `Core/Src/main.c` → `Core/Src/freertos.c` → 本次任务相关的 `Hardware/*.h` 和 `.c`。通常无需遍历全部 HAL 与 FreeRTOS 源码。

## 2. 目录地图

```text
STM32F407VET6_ILHC/
├── AGENTS.md                         本工程导航
├── STM32F407VET6_ILHC.ioc             CubeMX 外设/引脚/中间件配置
├── Core/
│   ├── Inc/                          公共句柄声明、HAL 与 FreeRTOS 配置
│   └── Src/                          启动流程、外设初始化、中断、默认任务
├── Hardware/
│   ├── ops.c / ops.h                 OPS 接收、CRC、位姿缓存、本地坐标清零
│   ├── zdt_x42s.c / zdt_x42s.h       张大头 Emm 串口电机协议
│   ├── mecanum_control.c / .h        麦轮解算、OPS 位置控制、四轮输出
│   ├── hcan.c / hcan.h               CAN 收发与接收钩子
│   ├── dm_j4310.c / dm_j4310.h       DM-J4310-2EC V1.1 协议驱动
│   ├── debug_usart.c / .h            调参、调试动作、JustFloat 遥测
│   ├── mecanum_pid.c / .h            历史保留文件，当前未加入构建
│   ├── README.md                     模块概览
│   └── 调试指令手册.md               命令、单位、范围与操作说明
├── Drivers/                          厂商 HAL、CMSIS 与设备头文件
├── Middlewares/Third_Party/FreeRTOS/  RTOS、CMSIS-RTOS2、heap_4、CM4F 移植层
├── MDK-ARM/
│   ├── .eide/eide.yml                EIDE 源文件清单及 AC5 构建配置
│   ├── STM32F407VET6_ILHC.uvprojx     Keil 工程
│   ├── startup_stm32f407xx.s         向量表与启动代码
│   └── build/STM32F407VET6_ILHC/      EIDE 参数、日志和固件产物
└── HostTools/ILHC_Debugger/
    ├── ilhc_debugger.py              Tkinter/Matplotlib 上位机与模拟器
    ├── README.md                     原版上位机使用说明
    ├── 启动上位机.bat                原版启动入口
    └── ILHC_Qt_v2/
        ├── main.py                   PySide6 界面、交互与心跳调度
        ├── core.py                   协议、串口、模拟器、CSV、地图几何
        ├── style.qss                 Qt 样式
        ├── requirements.txt          PySide6、pyqtgraph、pyserial、numpy
        └── README.md                 Qt 版本安装、运行与自检说明
```

## 3. 初始化与运行链路

`Core/Src/main.c` 当前依次执行：

1. `HAL_Init()`、`SystemClock_Config()`。
2. `MX_GPIO_Init()`、`MX_DMA_Init()`、USART1、CAN1、TIM1、TIM6、USART2、USART3、UART4 初始化。
3. 在 `USER CODE BEGIN 2` 中调用 `CAN_Start(&hcan1)`，失败进入 `Error_Handler()`。
4. `OPS_Init()` → `MecanumControl_Init()` → `MecanumControl_Enable()` → `DebugUsart_Init()`。
5. 初始化 RTOS 内核、创建默认任务、启动调度器。

`Core/Src/freertos.c` 当前只有一个显式创建的业务任务 `defaultTask`，正常优先级，栈 512 字节。循环为 `DebugUsart_Send(); osDelay(20);`。RTOS tick 为 1kHz，使用 `heap_4`，堆配置为 15360 字节。

`DebugUsart_Send()` 不只是发送函数：它还处理 STOP/ZERO、GOTO 位置控制、DM 模式切换和周期控制、主机失联保护。移除或降低其调用频率会同时改变运动控制行为。实际循环周期是处理耗时加 20ms 延时；UART4 阻塞发送及命令间延时会增加周期，不能视为严格的 50Hz。

HAL 毫秒时基由 TIM7 中断和 `HAL_TIM_PeriodElapsedCallback()` 维护；RTOS 使用自己的系统 tick。TIM1 PWM 和 TIM6 已初始化，但当前业务代码没有启动它们来驱动控制任务。

## 4. 外设映射与中断归属

| 外设 | 引脚 / 通信参数 | 当前用途及接收方式 |
| --- | --- | --- |
| USART1 | PA9 TX、PA10 RX；115200、8N1 | 调试；RX DMA2 Stream2 Ch4 + IDLE，TX DMA2 Stream7 Ch4 |
| USART2 | PD5 TX、PD6 RX；115200、8N1 | OPS；RX DMA1 Stream5 Ch4 + IDLE，命令阻塞发送 |
| USART3 | PB10 TX、PB11 RX；115200、8N1 | 已初始化，当前业务未接入 |
| UART4 | PA0 TX、PA1 RX；115200、8N1 | 四个张大头电机，当前阻塞发送，无 UART4 DMA 接收解析 |
| CAN1 | PA11 RX、PA12 TX；1Mbps | DM 电机；FIFO0 接收中断、全接收滤波 |

- 外设初始化与 DMA 绑定在 `Core/Src/usart.c`、`can.c`、`dma.c`；IRQ 入口在 `stm32f4xx_it.c`。
- USART2 使用 `Hardware/ops.c` 中的全局 `HAL_UARTEx_RxEventCallback()`。
- USART1 通过 `HAL_UART_RegisterRxEventCallback()` 注册 `DebugUsart_RxEventCallback()`；必须保留 `USE_HAL_UART_REGISTER_CALLBACKS=1`。
- USART1/2 RX DMA 为 NORMAL 模式，回调后重启；当前关闭 HT 半传输中断。新增回调时不能重复定义同名 HAL 回调或覆盖另一串口的入口。
- CAN 链路：`CAN1_RX0_IRQHandler()` → HAL → `hcan.c` 的 `HAL_CAN_RxFifo0MsgPendingCallback()` → `CAN_Rx_Callback()`。
- `CAN_Rx_Callback()` 在 `hcan.c` 为弱实现，在 `debug_usart.c` 被覆盖以解析 DM 反馈。增加 CAN 设备时在现有钩子分派，不要新增第二个强定义。
- `HCan_GetRxFrame()` 读取并清除“最近一帧”标志，不是接收队列；不能依靠它保证逐帧消费。
- CAN1 滤波器分区使用 `SlaveStartFilterBank=14`；自动重发和自动 Bus-Off 恢复已开启。

## 5. 协议与控制约定

### OPS 与底盘

- OPS 上行：`0x5C + float32 x + float32 y + float32 z + CRC8`，14 字节。CRC 算法以 `ops.c` 实现为准。
- OPS 原始坐标与 `OPS_GetPosition()` / `OPS_GetAbsolutePosition()` 返回单位为 **m、rad**；`mecanum_control.c` 内统一转换为 **mm、deg**。
- `OPS_ZeroCoordinates()` 仅在本地记录 X/Y 原点；不重置 OPS 本体，也不将 yaw 清零。`OPS_ClearZero()` 恢复绝对 X/Y。
- `OPS_Init()` 发送 `0xC5 0x22` 和 `0xC5 0x30`，包含启动等待；不是可在中断里调用的轻量操作。
- `chassis_move(x,y,z)` 计算位置误差、P 控制、限幅、速度斜坡及到位状态；由 `SetMotorVoltageAndDirection()` 实际下发轮速。
- `MecanumControl_GotoOPS()` 封装计算与输出，输入 mm/deg；OPS 超过 200ms 未更新时停车。调试层会在离线或到位后取消 GOTO。
- `MecanumControl_MoveVelocity(vxRpm,vyRpm,vzRpm)` 的输入是 RPM 形式的速度分量，不是 m/s 或 rad/s。
- 实际位置控制是 P 控制；`SetPid` / `SetYawPid` 的兼容名称不代表完整 PID，修改前查看实现。不要自动重新加入 `mecanum_pid.c`。
- `SpeedTarget[0..3]` 按顺序发送至电机地址 1..4；轮位、方向、轮径和换算系数需要结合实体车标定。
- 速度默认值和比例系数在 `mecanum_control.c`；`ZDT_X42S_MAX_RPM=3000` 是驱动代码限幅，不代表任何负载下均可达到的实测转速。

### DM 与 CAN

- `Core/Src/can.c` 是 CubeMX 初始化；`Hardware/hcan.c` 是 CAN 协议通道封装，不要因文件名相近合并或删除。
- `dm_j4310.c` 复用 `CAN_SendData()`，提供 MIT、位置速度、使能/失能、保存零点、控制模式寄存器写入及反馈解码。
- MIT 模式值 1，位置速度模式值 2；位置速度命令标准 ID 为电机 ID + `0x100`。
- DM 使用 rad、rad/s、Nm。头文件的量化范围必须和电机参数一致：当前位置 ±12.5、速度 ±30、力矩 ±10，Kp 0..500、Kd 0..5。
- 调试层采用禁用、写模式、等待 100ms、使能的顺序；目前未读回模式寄存器确认。
- CAN/DM 的发送成功表示提交给 HAL/邮箱成功，不代表电机已经执行。反馈 ID、状态和温度由接收链路解析。

### 调试串口与上位机

- 下行命令为 ASCII，每行以 CR 或 LF 结束，命令不区分大小写。参数表 `s_params` 和解析入口 `Debug_ParseLine()` 位于 `debug_usart.c`。
- 遥测为 **24 个小端 float32 + `00 00 80 7F`**，共 100 字节，标准 VOFA+ JustFloat，没有额外 `55 AA` 帧头。
- 通道 0..11 是底盘位姿、误差、参数和首轮目标速度；12..23 是 DM ID、反馈和目标参数。完整映射查看 `DebugUsart_Send()` 及调试手册。
- 固件不提供普通文本 ACK；不要把打印文本混入同一遥测流。
- `PING` 建议每 200ms 发送；固件超过 1s 没收到完整命令行时停止活动的 GOTO/DM 调试动作。两版上位机当前均由 GUI 主循环产生心跳。
- `STOP` 取消 GOTO、停车并失能 DM；`ZERO` 先取消定位移动并停车，再置本地 X/Y 原点。
- `WHEELEN`/`WHEELOFF` 只切换四轮锁轴，不改变 DM 和 28/35；失能期间拒绝 GOTO/MANUAL/ZDT，需显式重新使能。无锁轴状态回读。锁轴切换每周期最后执行；**失能后任何路径都不得再向 UART4 发速度帧**，因为 ZDT_X42S 速度命令会重新使能锁轴，停车只能走 `MecanumControl_ClearTarget()` / `Debug_ChassisStop()`。
- 修改帧格式、通道或命令范围时，同时核对固件、原版 `ilhc_debugger.py`、Qt `core.py/main.py` 和中文手册。两版上位机并未共用同一个协议模块。

## 6. 构建与验证

EIDE 与 Keil 共用源文件，但维护各自清单。当前 EIDE 配置名为 `STM32F407VET6_ILHC`，工具链 AC5、C99、单精度硬件浮点。已核对本机编译器目录为 `D:\keil51\Arm\ARMCC`，不要套用其他项目的 AC6 路径。

在 `MDK-ARM/` 下执行现有 EIDE 构建参数：

```powershell
& 'C:\Users\Yuzi\.vscode\extensions\cl.eide-3.27.2\res\tools\win32\unify_builder\unify_builder.exe' -p 'build\STM32F407VET6_ILHC\builder.params' --no-color
```

该路径是当前机器配置，换机或扩展升级后先查找实际安装位置。`builder.params` 是 EIDE 生成文件；缺失或源清单过期时通过 EIDE 重新生成，不将它作为唯一配置源。固件产物位于 `MDK-ARM/build/STM32F407VET6_ILHC/`，包括同名 `.hex`；调试文件名以实际构建输出为准。

从工程根目录运行上位机无 GUI 自检：

```powershell
py -3 HostTools\ILHC_Debugger\ilhc_debugger.py --selftest
py -3 HostTools\ILHC_Debugger\ILHC_Qt_v2\main.py --selftest
```

原版需要 NumPy、Matplotlib，实串口需要 pyserial；Qt 完整运行依赖见其 `requirements.txt`，`--selftest` 在加载 Qt 界面前执行。可使用各版本的 `--simulate` 查看无硬件演示。

按变更范围验证：固件改动编译；协议改动验证两版解析/模拟器；文档改动检查路径和描述即可。上位机模拟器不能证明 MCU 调度、CAN 应答或真实机械运动正确。报告时区分静态检查、编译、自检和实机验证，不能沿用旧构建结果声称本次验证成功。

## 7. 后续修改规则与已知边界

- 新增或修改的自定义代码注释、维护说明使用中文，保持实现简洁、可扩展，优先局部修改。
- 保留现有用户改动；开始前检查 `git status`。未被要求时不恢复 RC、完整塔吊应用层或独立 PID 架构。
- 新增 `.c` 文件需要同步 EIDE `virtualFolder` 与 Keil `Hardware` 分组；当前并非自动扫描全部 `Hardware/*.c`。
- CubeMX 自定义代码尽量放 `USER CODE` 块；引脚、DMA、中断和 HAL 配置变更同时核对 `.ioc`。当前存在手工维护的初始化/IRQ 代码，再生成后必须检查差异。
- 中断回调只做必要解析和状态更新；阻塞发送、等待、复杂控制放任务中。共享结构快照保护需恢复进入临界区前的 PRIMASK，不可无条件开中断。
- 新增 UART4 轮速输出前先确认四轮使能状态：ZDT_X42S 在速度模式下收到速度命令会重新使能并锁轴，失能后多一条速度帧就会把失能帧覆盖掉。停车分两条路径，`MecanumControl_Stop()` 会发速度 0 帧，`MecanumControl_ClearTarget()` 只清软件目标。
- DMA 发送缓冲区在传输完成前不能重写；新增任务或增加局部缓冲区时核对默认任务 512 字节栈及 RTOS 堆。
- OPS 当前按单个 14 字节帧接收，没有完整的字节流重同步/拼帧机制；不要描述成任意拆包粘包均可恢复。
- 主机失联保护在调试任务中执行，不是所有底层运动 API 的统一保护。UART4 发送仍阻塞且未解析电机应答；DM 模式切换也未读回确认。
- `main()` 启动时会使能底盘。调试程序真实串口连接和命令可能产生运动；阅读、文档和构建任务本身不要求自动烧录或发送运动命令。
- 上位机地图支持固定障碍与直线路径检查；不等于固件避障，也未实现完整车体膨胀和自动绕障。

## 8. 按任务快速定位

| 任务 | 优先查看 |
| --- | --- |
| 引脚、波特率、DMA、IRQ | `Core/Src/usart.c`、`can.c`、`dma.c`、`stm32f4xx_it.c`、`.ioc` |
| 初始化、任务频率或栈 | `Core/Src/main.c`、`freertos.c`、`Core/Inc/FreeRTOSConfig.h` |
| OPS 数据、CRC、置零 | `Hardware/ops.c`、`ops.h` |
| 底盘方向、速度、位置控制 | `Hardware/mecanum_control.c`、`zdt_x42s.c` |
| DM 报文与量化范围 | `Hardware/dm_j4310.c`、`dm_j4310.h`、`hcan.c` |
| 新增调参、动作或遥测通道 | `Hardware/debug_usart.c`、`debug_usart.h`、`调试指令手册.md` |
| Qt 界面、地图或波形 | `HostTools/ILHC_Debugger/ILHC_Qt_v2/main.py`、`style.qss` |
| Qt 协议、串口、模拟器、自检 | `HostTools/ILHC_Debugger/ILHC_Qt_v2/core.py` |
| 文件未编译、链接符号缺失 | `MDK-ARM/.eide/eide.yml`、`.uvprojx`、实际构建日志 |

历史移植来源记录在对应驱动头文件及 `Hardware/README.md`。外部 OPS、张大头手册和开源物流车代码不属于本仓库构建输入；需要再次核对协议时先确认参考文件的实际位置和版本。

2026-09-10 手动底盘更新：MANUAL=vx,vy,w，每轴整数±300 RPM，独立350ms续期，PING不续期；串口中断存请求，任务服务调用MoveVelocity；与GOTO互斥，STOP/ZERO取消。Qt底盘页按住运行、松开/失焦/切页停车，含方向反向开关；地址俯视左前1右前2左后3右后4。协议说明见调试手册。

2026-09-10 OPS偏心更新：F407 ops.c默认rx=-50mm/ry=+60mm（车后50左60），GetPosition在ZERO反号之前按航向去除偏心旋转位移；原始GetAbsolutePosition不变，ZERO同时保存yaw。OPSOFFSET=x,y（±500mm）成对调参，任务停车取消手动/GOTO后应用并置零；RAM参数，Qt可保存/加载电脑JSON，无Flash持久化和参数回读。24通道不变，发送遥测前GetPose刷新补偿位置。测试Tests/hardware/test_ops_offset.py。

2026-09-10：debug_usart.c新增ZDT=addr,rpm,seconds单轮测试，地址1~4、±300RPM、1~5秒。任务状态机停止四轮→使能指定轮→等待100ms→运行→FE98停止；不依赖PING续期，测试中忽略MANUAL/GOTO/ZDT，STOP/ZERO/OPSOFFSET取消。24通道遥测不变，无电机ACK。

2026-09-10：ZDT新增文字ACK/ERR队列，USART1 DMA空闲时优先发送；接收ZDT自动暂停二进制遥测，VOFA恢复。REQUESTED仅表示调用驱动，不表示HAL成功或电机应答。

2026-09-10：zdt_x42s.c新增UART4单字节IT接收（注册专用RxComplete/Error回调），仅解析F3/F6/FE四字节6B控制回包。main首次使能前InitRx；debug任务ServiceRx/PopReply，文字模式ZDT RX显示，500ms目标地址无回包提示（非逐命令事务超时），VOFA模式丢弃显示但持续接收。测试Tests/hardware/test_zdt_rx.py。

2026-09-11 GPIO分配：USART3 PB10/PB11摄像头预留；PE9 TIM1_CH1夹爪舵机预留，原频率未改且未启动PWM。PD0 VM_EN、PD1 CAMERA_LIGHT_EN推挽输出默认低、高有效；PD2/PD3 START_KEY1/2上拉轮询输入、低有效，无消抖/启动动作。源码和.ioc已同步。详见PCB主控引脚说明.md；VM若供电机，开启后需等待上电并重新使能，当前不自动开启VM。

2026-09-11 USART1早期启动提示：main在MX_USART1_UART_Init后调用DebugUsart_SendStartup，独立const缓冲区DMA发送一次UTF-8/ASCII启动消息；提交失败由DebugUsart_Send开头重试。不表示USB连接检测或MCU RX已正常，未改变默认JustFloat遥测。

2026-09-11：PB2 COMM_LED高有效、默认低；只支持LED ON/LED OFF完整行指令，忽略大小写，默认任务执行GPIO。无自动心跳/接收闪烁，无文字回执，不触发电机。GPIO/ioc及PCB说明已同步。

## 2026-09-11：CAN启动失败隔离

`main.c`中`CAN_Start`失败不再进入`Error_Handler`：关闭CAN中断，保留ErrorCode，置HAL_CAN_STATE_ERROR，继续独立串口驱动及RTOS初始化。不自动重试CAN；修复硬件后重新启动。此改动针对CAN_Start失败，未将所有MX外设初始化错误改为可忽略。

`DebugUsart_Init`在CAN不可用时进入文字模式，并通过既有任务DMA发送队列输出：
`ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE`

CAN不可用时DM/S28/S35指令直接拒绝，不缓存动作，回复：
`ERR CAN DISABLED; DM/S28/S35 REJECTED`

底层HCan_Submit原有LISTENING检查继续保护所有CAN发送。LED、ZDT、STOP等独立功能不受此CAN失败阻断。发送`VOFA`加换行可恢复波形输出。CAN失败提示不代表PA10已通过实测。

测试：`python Tests/hardware/test_can_degraded.py`，以及既有接收恢复、步进、GPIO和驱动回归；固件编译通过，尚未烧录实测。

## 2026-09-12：移除临时串口通信测试

已删除USART1 READY启动横幅及DebugUsart_SendStartup接口，删除led on/led off解析与LED任务服务。历史章节中的这些测试功能不再生效。PB2仍初始化为低电平输出，保留作未来状态灯。正式VOFA调参、MANUAL/ZDT/DM/28/35控制、应答、RX异常恢复及CAN失败隔离保持不变；命令仍需CR或LF结尾。
