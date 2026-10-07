# ILHC PCB主控引脚说明

更新：2026-10-07。已按当前GPIO、串口、CAN、TIM、OLED、Flash驱动及CubeMX配置核对。对应STM32F407VET6_ILHC工程，主控STM32F407VET6，LQFP100封装。

本文件是新PCB的信号分配和接线约定，不是完成电气审查的原理图。GPIO编号、芯片物理脚号和连接器脚号分别列示；芯片脚号依据ST数据手册DS8626 Rev 12的LQFP100顶视图（图13）及引脚表（表9），不能混用LQFP64/144封装。CSV清单见同目录`PCB主控引脚配置_20261007.csv`。

## 1. 全部已分配信号

方向以MCU为参照；逻辑接口按3.3V设计，外设电源单独按器件规格确定。

| 功能 | GPIO/信号 | LQFP100脚号 | 模式/外设 | 连接对象及有效电平 | 当前固件状态 |
|---|---|---|---|---|---|
| 调试发送 | PA9 | 68 | USART1_TX，AF7 | USB转串口RX | 已实现DMA发送 |
| 调试接收 | PA10 | 69 | USART1_RX，AF7 | USB转串口TX | 已实现空闲中断+DMA及异常恢复 |
| OPS发送 | PD5 | 86 | USART2_TX，AF7 | OPS的RX | 已配置 |
| OPS接收 | PD6 | 87 | USART2_RX，AF7 | OPS的TX | 已实现定位接收 |
| 视觉串口发送 | PB10 | 47 | USART3_TX，AF7 | 工控机的RX | 已接入视觉协议V1.1任务请求发送（6字节+CRC8+SEQ），115200/8N1 |
| 视觉串口接收 | PB11 | 48 | USART3_RX，AF7 | 工控机的TX | 已接入视觉协议V1.1定长响应接收（16字节） |
| 底盘电机发送 | PA0 | 23 | UART4_TX，AF8 | ZDT驱动器RX | Emm协议发送 |
| 底盘电机接收 | PA1 | 24 | UART4_RX，AF8 | ZDT驱动器TX | F3/F6/FE应答接收 |
| CAN发送 | PB6 | 92 | CAN2_TX，AF9 | CAN收发器TXD | 已启用CAN2，1Mbps |
| CAN接收 | PB5 | 91 | CAN2_RX，AF9 | CAN收发器RXD | DM、28/35电机共用总线 |
| 夹爪舵机信号 | PE9 | 38 | TIM1_CH1，AF1 | 舵机信号输入 | PWM资源预留，尚未启动 |
| 通信指示灯 | PB2 | 36 | 推挽输出，低速 | 板载LED，高电平亮 | led on点亮；led off熄灭；默认关闭 |
| VM电源开关 | PD0 | 81 | 推挽输出，低速 | 外部电源开关EN，高电平开启 | 初始化输出低，默认关闭 |
| 摄像头补光灯 | PD1 | 82 | 推挽输出，低速 | 灯驱动EN，高电平开启 | 初始化输出低，默认关闭 |
| 启动按键1 | PD2 | 83 | 上拉输入 | 按键另一端接GND，低电平按下 | 轮询输入，无EXTI/消抖/启动动作 |
| 启动按键2 | PD3 | 84 | 上拉输入 | 按键另一端接GND，低电平按下 | 轮询输入，无EXTI/消抖/启动动作 |
| OLED时钟 | PB13 | 52 | GPIO推挽输出 | OLED CLK/SCL | 软件SPI |
| OLED数据 | PC3 | 18 | GPIO推挽输出 | OLED DIN/SDA | 软件SPI，不是I2C |
| OLED复位 | PE2 | 1 | GPIO推挽输出 | OLED RES | OLED驱动管理 |
| OLED数据/命令 | PE3 | 2 | GPIO推挽输出 | OLED DC | OLED驱动管理 |
| OLED片选 | PE4 | 3 | GPIO推挽输出 | OLED CS | OLED驱动管理 |
| Flash片选 | PA4 | 29 | GPIO推挽输出，上拉 | W25Q128 /CS，低有效 | 驱动初始化为高 |
| Flash时钟 | PA5 | 30 | SPI1_SCK，AF5 | W25Q128 CLK | 模式0，5.25MHz |
| Flash数据输入 | PA6 | 31 | SPI1_MISO，AF5 | W25Q128 DO/IO1 | MCU接收 |
| Flash数据输出 | PA7 | 32 | SPI1_MOSI，AF5 | W25Q128 DI/IO0 | MCU发送 |
| 下载调试数据 | PA13 | 72 | SWDIO | 调试器SWDIO | 保留，不分配给业务 |
| 下载调试时钟 | PA14 | 76 | SWCLK | 调试器SWCLK | 保留，不分配给业务 |
| 系统复位 | NRST | 14 | 专用引脚 | 调试器/复位按键 | 保留 |
| 高速晶振输入 | PH0 | 12 | HSE OSC_IN | 8MHz晶振电路 | 系统时钟168MHz |
| 高速晶振输出 | PH1 | 13 | HSE OSC_OUT | 8MHz晶振电路 | 使用晶体模式，非BYPASS |
| 启动配置 | BOOT0 | 94 | 专用引脚 | 正常运行下拉，预留拉高入口 | PB2兼作BOOT1，保留下拉 |

所有串口当前为115200、8数据位、1停止位、无校验。电机功率供电不从GPIO或MCU的3.3V电源获取。

USART1当前用于JDY-31蓝牙SPP上位机通信，也可接USB转TTL调试；MCU TX接模块RX，MCU RX接模块TX。两种接口共用PA9/PA10，若同时布置，应通过跳线或切换电路选择接收来源。

Flash是独立的硬件SPI1参数存储接口，不与OLED的软件SPI共线。`SPIFlash_Init()`负责实际GPIO/SPI初始化，参数存储初始化会调用它；驱动接受3.3V W25Q128的JEDEC ID 0xEF4018或0xEF7018。/CS复位阶段需上拉；采用普通SPI时/WP、/HOLD（或对应IO2/IO3）按所选器件手册接线。

## 2. 本次新增GPIO及上电行为

- PD0/PD1先写低锁存值再配置输出，防止初始化切换时误开启。
- 复位到GPIO初始化之间不能靠软件保证输出低：在外部EN控制端设置可靠下拉，例如10kΩ，并确认开关芯片确实为高有效。
- PD0仅控制电源开关EN/驱动级，不直接串入VM功率路径，不直接接高压MOS管栅极。
- PD1仅控制灯驱动EN/驱动级，不直接给补光灯供电。
- PD2/PD3使用内部上拉；PCB可各加约10kΩ外部上拉至3.3V，按键接地。长线按键考虑接口保护与滤波。
- 本次只配置GPIO，没有自动启动比赛、自动开电源或自动点灯，也没有按键消抖。后续应用层进行消抖和按下沿判断。

代码使用示例（未自动调用）：

```c
/* 开启/关闭VM电源开关。 */
HAL_GPIO_WritePin(VM_EN_GPIO_Port, VM_EN_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(VM_EN_GPIO_Port, VM_EN_Pin, GPIO_PIN_RESET);
/* 开启补光灯。 */
HAL_GPIO_WritePin(CAMERA_LIGHT_EN_GPIO_Port, CAMERA_LIGHT_EN_Pin, GPIO_PIN_SET);
/* 按键按下为低；这里仅获取原始电平，需另做消抖。 */
if (HAL_GPIO_ReadPin(START_KEY1_GPIO_Port, START_KEY1_Pin) == GPIO_PIN_RESET)
{
  /* 提交启动请求，由应用层决定是否执行。 */
}
```

**VM启动顺序提醒：** 若VM给底盘驱动器供电，默认关闭时main里的首次电机使能不会得到执行。后续应用需要按“开启VM→等待驱动器上电就绪→重新发送使能→发送运动指令”的顺序处理。现有ZDT单轮测试只重新使能，不会自动拉高PD0。此时即使串口应答正常，电机未上电仍无法运动。

## 3. 舵机和摄像头预留的边界

- USART3是工控机视觉结果串口，不是摄像头图像总线；已接收视觉协议V1.1响应（16字节+CRC8+SEQ），不要将PB10/PB11误接到并行图像接口。
- PE9只预留TIM1_CH1。当前PSC=1679、ARR=65535，在168MHz定时器时钟下周期约655.36ms，并非通常的20ms舵机周期；尚未启动PWM。
- 本次不自动改变频率或输出中位脉冲，避免夹爪意外动作。确定舵机型号、供电、频率和安全脉宽后再实现驱动和机械限位。
- 舵机电源使用独立适配的电源支路并共地，核实3.3V控制信号是否满足其输入要求，不从GPIO或MCU小电流3.3V支路供电。

## 4. PCB连接器脚序建议

以下为新PCB建议，不代表现有模块连接器脚序；画板前必须对照外设接口。

| 接口名 | 建议脚序 | 注意 |
|---|---|---|
| DEBUG | 1 GND / 2 PA9-TX / 3 PA10-RX / 4 3V3_REF | 3V3_REF是电平参考，外部供电接入需独立规划防回灌 |
| CAMERA | 1 CAMERA_VCC / 2 GND / 3 PB10-TX / 4 PB11-RX | CAMERA_VCC电压根据摄像头选型 |
| OPS | 1 OPS_VCC / 2 GND / 3 PD5-TX / 4 PD6-RX | OPS_VCC电压根据模块选型 |
| ZDT_UART | 1 GND / 2 PA0-TX / 3 PA1-RX | 电机VM电源另走功率接口；只适用于TTL串口 |
| CAN_BUS | 1 CANH / 2 CANL / 3 GND | 来自CAN收发器总线侧，不是直接来自MCU引脚 |
| GRIPPER | 1 SERVO_VCC / 2 GND / 3 PE9-PWM | 对照实际舵机线序，防反接 |
| KEY1 | 1 PD2 / 2 GND | 低有效 |
| KEY2 | 1 PD3 / 2 GND | 低有效 |
| OLED | GND / OLED_VCC / PB13 / PC3 / PE2 / PE3 / PE4 | 顺序按所选屏幕调整，勿按I2C屏接线 |
| FLASH（板内） | /CS=PA4 / CLK=PA5 / DO=PA6 / DI=PA7 | W25Q128，3.3V；按实际封装确定Flash自身脚号 |
| SWD | 3V3_REF / GND / PA13 / PA14 / NRST | 保留可接触的下载接口或测试点 |

VM_EN与补光灯EN优先连接板内驱动电路；若需外接控制板，接口至少包含EN与GND，并核实输入电压兼容。

多台ZDT共用TTL串口时，还需确认驱动器手册支持的总线接法；不能未经核实就将多个推挽TX输出直接并联。

## 5. 重要引脚冲突

1. **当前固件使用CAN2：PB5接CAN收发器RXD，PB6接TXD。** 当前未实现USB CDC，调试串口使用USART1。
2. 原开发板PD2/PD3连接TF卡电路；新PCB已分配启动按键，不再沿用该TF接线。
3. 原开发板PA0连接WKUP支路、PB2连接BOOT1/LED。新PCB不必照搬PA0的按键，避免与UART4重复分配。
4. PB13、PC3、PE2~PE4归OLED，PA4~PA7归Flash，PB10/PB11归视觉串口，PE9归舵机，不作为通用空闲接口再次使用。
5. MCU逻辑电源不能跟随VM一同切断，否则PD0无法负责重新开启VM。VM负载支路与常供电主控支路需明确划分。

## 6. 尚未分配的GPIO

以下仅针对新PCB、当前固件；是否适合某个外设还要查AF映射和电气特性。

| 端口 | 未分配GPIO |
|---|---|
| A | PA2、PA3、PA8、PA11、PA12、PA15 |
| B | PB0、PB1、PB3、PB4、PB7、PB8、PB9、PB12、PB14、PB15 |
| C | PC0~PC2、PC4~PC15 |
| D | PD4、PD7~PD15 |
| E | PE0、PE1、PE5~PE8、PE10~PE15 |

PB2涉及启动配置，PB3/PB4/PA15涉及调试复用，PC14/PC15涉及低速晶振；PA11/PA12当前未配置USB，若后续设计USB FS接口需预留。不能只因未被当前代码使用就忽略这些用途。TIM6已初始化但未启动；TIM7是HAL时基，不要另作普通业务定时器。

## 7. 最小系统与发布前检查

以下为LQFP100专用电源脚，必须在原理图中连接，不能因为没有GPIO初始化代码而遗漏。

| 信号 | 芯片物理脚号 | 连接要求 |
|---|---|---|
| VDD | 11、19、28、50、75、100 | 主控3.3V，全部连接并就近去耦 |
| VSS | 10、27、74、99 | 全部接GND |
| VDDA | 22 | 接模拟3.3V并去耦 |
| VSSA | 20 | 接GND |
| VREF+ | 21 | 本工程未使用ADC，可接VDDA并去耦 |
| VBAT | 6 | 未使用后备电池时接VDD |
| VCAP_1、VCAP_2 | 49、73 | 正常内部稳压模式下各接2.2µF陶瓷电容到GND，不能接3.3V或外设 |

- 核对LQFP100封装方向、全部电源和地脚，按ST参考供电方案配置VDDA/VREF+、VBAT、VCAP及各电源去耦；VCAP不是外设电源输出。
- 保留BOOT0、复位和SWD恢复通道，不把启动配置脚做成不受控的悬空输入。
- 晶振及其负载按实际器件规格设计，匹配当前8MHz HSE固件。
- MCU控制区、通信收发器与电机/舵机功率回路分区布线，检查回流、电源纹波和接口保护。
- CAN收发器采用与MCU逻辑兼容的型号，终端电阻按整条总线两端配置，不给每个节点都固定并联终端。
- 先不连接执行机构验证PD0/PD1复位及开关电平、PD2/PD3按键电平，再进行带负载验证。
- 摄像头供电、舵机供电与脉宽、VM电压/电流、补光灯功率及开关电路型号尚未确认，不能据此文件直接完成电源器件选型。

官方封装与最小系统核对依据：[ST STM32F405/407数据手册DS8626，LQFP100图13及供电章节](https://www.st.com/resource/en/datasheet/stm32f407ve.pdf)。信号用途以本工程源码、CubeMX配置与本文件为准。

PB2现已分配通信LED，不再是空闲GPIO。BOOT0正常保持低，保留PB2原板BOOT1下拉；通过led on/led off指令控制，均须追加换行；亮灭成功说明接收解析及任务GPIO控制链路执行，不代表电机通信正常。

## 8. 本次核对范围（2026-10-07）

信号分配核对：`Core/Inc/main.h`、`Core/Src/gpio.c`、`usart.c`、`can.c`、`tim.c`、`main.c`、`Hardware/OLED_SoftSPI.h`及其初始化、`Hardware/spi_flash.c`、`debug_param_store.c`、`STM32F407VET6_ILHC.ioc`。当前业务与预留信号共25个GPIO，另保留SWD 2个GPIO、HSE 2个GPIO；未分配GPIO共53个。未发现当前信号表内部的GPIO重复分配。

本次修正文档旧版遗漏的PA4~PA7 Flash、空闲列表误含PB5/PB6及漏列PA11/PA12问题，补充芯片物理脚号和电源脚。仅更新引脚说明与CSV，未修改固件引脚或控制逻辑。
