# Hardware

## RTOS_APP任务归属（2026-10-04）

应用任务已拆分到根目录 [RTOS_APP](../RTOS_APP/README.md)：底盘20ms控制与轮速归chassis，USART1/OPS解析、遥测和串口恢复归comm，DM/步进机构归mechanism，Flash参数保存归maintenance。原defaultTask已经移除；下文历史描述中的任务归属以此处和RTOS_APP维护说明为准。

2026-10-04：点击目标规划复用trajectory_buffer整批缓存/本地执行。首点允许仅STOP位，支持从实际起始车头执行显式ROTATE；首点仍必须s=0，不能WAIT/ROTATE/ARC。其余协议和取消保护不变；编译及真实C离线回放通过，未烧录或实车运行。

### 2026-10-03 完整比赛整批轨迹

新增trajectory_buffer.c/.h：发车前缓存全部路段，CRC/结构复检后本地连续跟踪OPS。当前为自动跑图，7处站点位置/航向/停稳满足后自动继续，无需人工作业确认或TRESUME；最后返回启停区DONE。未来塔吊显式WAIT模式由Traj_CompleteStation释放当前站点，取消后不能复活旧路径。4096点/64KB，RAM总计约90KB/128KB。EIDE/Keil均已加入源文件；实机接收/运行代码已编译，未烧录或实物验证。
新MecanumControl_MoveWorldVelocity保留世界/车体轴序；旋转轮距参数MECANUM_ROTATION_LEVER_MM须按实车标定。
完整协议、操作及测试见[整批轨迹说明](../HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/STM32_BATCH_TRAJECTORY_20261003.md)。

### 2026-10-03 DM内部梯形参数桥接

dm_j4310.c/.h新增ACC/DEC成对异步读写及真实回读确认，debug_usart.c接入DMREAD和DMACCDEC。
只修改电机RAM，单位Krad/s²，DEC为负；失能及300ms内实际CAN反馈才受理。
STOP/DMOFF/心跳失联取消，发送失败/回读超时明确返回。24通道遥测、模式切换与Flash记录不变。
接口、状态码及部分写入处理见调试指令手册顶部；真实C专项见Tests/hardware/test_dm_registers.py。

### 2026-09-10 驱动修正与注释

28/35 和 OLED 的头文件已逐个说明参数单位、返回值、边界、刷新与调用限制。CAN 公共发送层改为局部发送头，短临界区保护邮箱提交；短帧先复制到八字节本地缓冲，避免 HAL 固定读取八字节时越过四字节回零命令。DLC 仍保持实际长度。分包提前检查整个 ID 范围，保留 HAL_BUSY 返回；本接口仍不保证总线原子送达。

28/35 在 CAN 未启动时返回 HAL_ERROR，回复计数饱和防止回绕误判。OLED 修复12像素字模填充位、非整页图片超范围绘制及滚动重新开始问题；原字库 GBK 注释已转 UTF-8，字模数据未改变。调试 USART1 在 DMA 忙时跳过遥测打包，不覆盖在途发送缓冲，控制处理仍继续执行。

主机回归测试说明见 `Tests/hardware/README.md`；这些检查不替代控制器兼容性与机械行程实测。

## 28 / 35 步进电机 CAN 驱动

`stepper_2835.c/.h` 使用张大头X固件协议，复用当前 CAN2（PB5 RX / PB6 TX、1Mbps）。CAN收发器的RXD接PB5、TXD接PB6，H/L侧接电机CANH/CANL。它和 UART4 的 `zdt_x42s` 属于不同报文接口。

| 接口 | 用途 |
| --- | --- |
| `Motor_Homing(MOTOR35_CAN_ID)` / `Motor_Homing(MOTOR28_CAN_ID)` | 发送原协议多圈回零命令，可能产生运动 |
| `Motor_AbsPosition(dir,id,step,speed)` | 方向0/1、X固件角度整数（0.1°）、RPM 0–3000；默认支持0x100/0x200 |
| `Stepper2835_ReadStatus(id)` / `Stepper2835_Enable(id)` | 读取状态 / 显式使能锁轴 |
| `Motor35_AbsPosition(h,speed)` | 原车 Z 高度换算，h 单位 0.1mm、speed 单位 mm/s |
| `Motor28_AbsPosition(r,speed)` | 原车伸缩半径换算，r 单位 0.1mm、speed 单位 mm/s |
| `Stepper2835_GetReply(id,&reply)` | 读取原始回复、计数、时间戳快照，不代表到位 |

位置指令保留原始 16 字节格式，通过 `Can_SendCmd()` 发送两个 8 字节扩展帧（ID 与 ID+1）。示例 `Motor_AbsPosition(0,0x100,1000,1000)` 的报文为：

```text
扩展 ID 0x100：FD 00 AF FF AF FF 27 10
扩展 ID 0x101：FD 00 00 03 E8 01 00 6B
回零扩展 ID 0x100/0x200：9A 02 00 6B（DLC=4）
```

换算保留原车参数：35 的行程为 `clamp(2030-h,0,1600)`，每单位行程乘 44.94 得到协议位置计数，RPM=`speed*30`；28 的行程为 `clamp(r-1200,0,1660)`，计数乘 3.189，RPM=`speed*0.53`，整数结果向下截断。原车注释中的机械范围和这里的实际限幅不完全一致，必须以新机构标定为准，不能直接认定为新车安全行程。

发送返回 `HAL_OK` 仅说明提交成功；两个邮箱不足返回 `HAL_BUSY`，应在任务后续周期重试，不能在中断中等待。RPM发送前乘10转换为0.1RPM，超过3000RPM拒绝发送。X固件位置要求S_PosTDP=Disable，角度不受MStep细分设置影响。

CAN 扩展数据帧通过 `debug_usart.c` 的现有 `CAN_Rx_Callback()` 分派给本驱动；保留标准帧 DM 分支。原项目仅根据 ID 设置“完成标志”，未校验应答内容，本次不沿用该到位判断。回复协议确认前只能检查原始数据及新鲜度。

串口调试支持S35/S28的MOVE、RAW、HOME、CANCEL，以及STATUS状态查询和EN显式使能；详细范围见调试指令手册。驱动原始回复和CAN提交/超时诊断经USART1返回上位机，不占24个遥测通道。当前STOP/PING失联仅撤销步进待发请求，不能停止已启动的步进运动。机构换算系数保留原车数值，尚需按实际丝杆和减速比标定。

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
  - 下行命令：0xC5 0x22（复位），等待 OPS 重启后发 0xC5 0x32
    选择新协议方向 2；不再发送旧的 0xC5 0x30 启动握手
  - 使用：OPS_Init() 初始化；OPS_GetPosition(&x, &y, &z) 读取新坐标；
    OPS_IsOnline() 判断位姿是否在超时窗口内；
    OPS_ConsumeSessionChanged() 读取 V2 复位/重连事件
  - session_id 运行期变化时，任务层取消旧 GOTO；接收端自动把新会话首帧
    重设为本地零点参考，避免 OPS 复位后沿用旧原点
  - USART2 错误回调只置恢复请求，通信任务通过 OPS_ServiceRx() 重挂 DMA
  - 统一轴序：X=左右（+车左）、Y=前后（+车头）、Z 逆时针为正；内部与协议同序同号，
    不再做 X/Y 交换或取反。OPS 原始帧到统一坐标固定为 `X=-raw_x、Y=raw_y`，
    不存在方向模式或其他分支。位置、绝对坐标、ZERO、`OPS_SetOrigin()` 和
    安装偏心补偿共用这组固定映射。
  - 坐标清零：OPS_ZeroCoordinates() 以当前位置为平移原点，并把 X/Y 轴旋转到
    ZERO 时的车体方向，同时归零 Z；OPS_ClearZero() 恢复绝对 X/Y/Z，
    OPS_SetOrigin(x, y) 手动设 X/Y 零点并归零航向

## 底盘移动控制

- zdt_x42s.c / zdt_x42s.h：张大头 ZDT_X42S 闭环步进电机驱动
  - UART4（PA0=TX，PA1=RX），115200/8N1
  - Emm 固件速度模式：地址 + 0xF6 + 方向 + 速度 + 加速度 + 同步 + 0x6B
  - 支持使能、失能、立即停止、速度模式控制
  - **发送为非阻塞**：`ZDT_X42S_InitTx()` 建立 UART4 中断发送队列，
    `ZDT_X42S_ServiceTx()` 由底盘任务每周期推进；控制任务只把帧压入固定长度队列，
    轮速下发不再阻塞控制周期。同地址速度帧覆盖尚未发送的旧帧，停止/失能清除同地址
    待发送的旧速度，防止安全帧之后重新启动。TX 完成后由 TIM7 毫秒中断留出至少 1ms
    帧间空闲，再发下一帧；调度器启动前也能推进，所以 `InitTx` 必须在首次命令之前调用。
  - 因为下发不再当场发送，`ZDT_X42S_Enable/Disable/Stop/Speed/SpeedAcc` 的返回值只表示
    "是否已入队"，**不代表电机收到**；发送结果由 `ZDT_X42S_GetTxErrorCount()` 反映
    （单帧在途超时或超过 1s 重试预算计入失败），`debug_usart.c` 的
    `Debug_ServiceWheelFault` 靠它的变化锁存底盘故障
- mecanum_control.c / mecanum_control.h：麦克纳姆轮底盘控制
  - O 型麦轮四轮速度解算
  - 速度模式连续移动：MecanumControl_MoveVelocity(vxRpm, vyRpm, vzRpm)，参数顺序为
    (X=左右, Y=前后, Z=旋转)；协议 `MANUAL=X,Y,W` 原序传入，不交换、不取反。
    实车轮序为俯视左前1、右前2、左后3、右后4；麦轮矩阵保持 O 型逻辑轮速公式，
    SetMotorVoltageAndDirection() 只把逻辑轮速正号转 CW、负号转 CCW，不再额外翻转
    2/4 号电机。W 的最终下发图案为 `[+ - + -]`，A 左移为 `[- - + +]`
  - OPS 全局定位 GOTO/GOTOHOLD：MecanumControl_GotoOPS(x, y, yaw, maxRpm)，
    x=X=左右、y=Y=前后；`GOTO=X,Y,Z` 到位后停止，`GOTOHOLD=X,Y,Z` 到位后持续位置闭环保持
  - 参考开源底盘：chassis_move(x, y, z) + SetMotorVoltageAndDirection(SpeedTarget[0..3])，
    形参按统一顺序 x=X=左右、y=Y=前后
  - 通过 OPS_GetPosition() 读取定位反馈，P 比例控制 + 斜坡限制 + 到位判断；误差定义为
    `目标 - 当前`，与 ops.c 置零后的物理正向坐标配套（两者必须成对，否则位置环为正反馈）
  - 24 通道遥测直接输出 ch0/ch1=X(左右)/Y(前后)、ch3/ch4、ch6/ch7；位置和误差对外
    以 cm 输出（1 位小数），麦轮混控与 ZDT 方向映射保持上一版逻辑
  - OPS 原始 m/rad 在底盘层统一转换为 mm/deg；GOTO 的 cm 在协议边界换算为 mm 后进入位置环，
    定位数据超过 200ms 未更新自动停车
  - 四轮锁轴：MecanumControl_Enable() 使能并保持位置（100ms 稳定期由 debug 层状态机
    等待，不阻塞控制服务）、
    MecanumControl_Disable() 失能不锁轴（不等待）、MecanumControl_Stop() 停车并下发
    速度 0 帧、MecanumControl_ClearTarget() 只清目标不发帧
  - 失能后必须用 ClearTarget：ZDT_X42S 在速度模式下收到速度命令会重新使能锁轴
  - 调用顺序：MX_UART4_Init -> OPS_Init -> ZDT_X42S_InitTx -> ZDT_X42S_InitRx ->
    MecanumControl_Init -> MecanumControl_Stop -> MecanumControl_Enable -> DebugUsart_Init

## USART1 调试模块

- debug_usart.c / debug_usart.h：USART1 调试调参
  - PA9=TX，PA10=RX，115200/8N1
  - TX：DMA 发送 VOFA+ JustFloat 数据帧
  - RX：DMA 空闲中断接收 ASCII 命令
  - 命令示例：KPX=3.0、KPY=3.0、KPZ=10.0、XVMAX=1600、ZVMAX=750、STOP、ZERO
  - 轴归属：KPX 写 mKpx（X 左右）、KPY 写 mKpy（Y 前后），范围仍 0~50
  - MANUAL=X,Y,W、GOTO=X,Y,Z、GOTOHOLD=X,Y,Z、OPSOFFSET=X(左右偏移),Y(前后偏移)，
    这些命令均按统一坐标直接使用；GOTO/GOTOHOLD 的 X/Y 与遥测 ch0/ch1、ch3/ch4 使用 cm（1 位小数），
    OPSOFFSET 仍使用 mm
  - 四轮锁轴命令：WHEELEN（使能/锁轴）、WHEELOFF（失能/不锁轴），失能期间拒绝
    GOTO/GOTOHOLD/MANUAL/ZDT；锁轴切换排在每周期最后，失能后所有停车路径只清目标不发速度帧
    （Debug_ChassisStop），见调试指令手册
  - DM 电机命令：DMID=3、DMEN、DMOFF、DMSTOP、DMZERO
  - DM 控制命令：DMMODE=1/2、DMPOS=3.14、DMVEL=2、DMKP=2、DMKD=1、DMTOR=0.5
  - VOFA+ 通道：0~11 为底盘，12~23 为 DM（ID/位置/速度/力矩/状态/温度/目标值）
  - 标准 JustFloat：24×float32 + 4 字节帧尾；上位机每 200ms 自动发送 PING 心跳
  - 参数表在 debug_usart.c 中集中管理，新增参数只需添加一项
## CAN 协议模块

- hcan.c / hcan.h：移植自 tower/hcan.c
  - CAN2（PB5 RX / PB6 TX）：滤波全接收 + FIFO0 接收中断
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
  - CAN2 1Mbps，标准帧；位置速度模式命令 ID = 电机 ID + 0x100
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

USART3已接入视觉协议V1响应接收，PE9 PWM用于夹爪舵机预留；PD0/PD1分别为VM开关/补光灯，高有效、默认关；PD2/PD3上拉按键输入，低有效。接口宏在Core/Inc/main.h，初始化在Core/Src/gpio.c。完整接线、上电行为及CAN/USB冲突见根目录PCB主控引脚说明.md。

### USART3视觉通信与物料精对准（V1.1坐标系修正版，2026-09-29）

协议依据：`E:/STM32/ILHC/jetson/STM32F407_视觉通信协议_V1.1_坐标系修正版_AI_Agent版.md`。
**不要参考已被作废的早期 V1.1 草案**——它把车体坐标误写成"+X=前、+Y=左"，修正版已统一为
"+X=左、+Y=前、+Yaw=逆时针"，与本工程坐标契约一致。

`vision.c/.h`在USART3发 **6字节** 请求 `66 TASK TARGET SEQ CRC8 77`、收 **16字节** 响应
`66 TASK STATUS SEQ PAYLOAD[10] CRC8 77`；`CRC-8` 为 poly=0x07/init=0x00/不反转，
请求覆盖前4字节、响应覆盖前14字节（自检值 `Check("123456789")=0xF4`）。
`SEQ` 由固件生成：`Vision_SendRequest()` 内部递增序号并维护 `active_task/active_target/active_seq`，
同一逻辑任务重发复用同一序号，HAL忙时保留待发（换序号会被Jetson当成新任务）。
`main.c`在USART3初始化后调用`Vision_Init()`；USART3中断做帧头、任务码、状态码、帧尾和CRC校验，
逐字节滑动重同步（不清空整个缓冲、不按`0x77`截帧），完整响应存入15个可用槽位的队列；
通信任务调用`Vision_ServiceRx()`恢复串口接收错误。
应用层用`Vision_PopResponse()`取结果，**只采用 `task` 与 `seq` 同时匹配当前活动任务的结果**，
再按任务调用`Vision_DecodeTrack()`（TRACK载荷）、`Vision_DecodeTarget()`（SCAN载荷）或
`Vision_DecodeQr()`。批量任务按 INDEX 位图去重并在 COMPLETE 时校验完整性（`Vision_GetBatch()`）；
当前没有命令会触发批量任务，该状态机目前只被回归测试覆盖。
`vision_track.c/.h`在任务上下文消费TRACK结果，按 `COORD_MODE` 选像素或毫米系数计算车体X/Y速度：
方向语义与工程坐标契约一致（`ERROR_X>0`向左、`ERROR_Y>0`向前），**不做任何轴交换或取反**；
只驱动 `VALID_FLAGS` 置位的轴，YAW 已解码但暂不参与控制。
USART1调试命令`VTRACK=1..6`按颜色启用，`VTRACK=0`或`STOP`停止。
目标丢失、低置信度、`NOT_FOUND`、`BUSY`或结果超过150ms时停车；视觉端报错、四轮失能、
TX故障或主机超过1秒无命令/PING时退出跟踪。
**这些停车路径都不会向工控机发停止帧**——只有显式停止、取消类动作（STOP/ZERO/OPSOFFSET/
WHEELEN/WHEELOFF）和视觉端报错才发，所以工控机侧看不到"固件因数据过期而停车"。
调参（默认值）：`VCONF` 置信度阈值(50)、`VKPMM` 毫米模式增益(0.5 RPM/mm)、
`VDBMM` 毫米模式死区(**2 mm**)、`VDBPX` 像素模式死区(12 像素)、`VMIN` 最小时速(8 RPM)、
`VMAX` 每轴上限(60 RPM)。六项都是 **RAM参数**，可用 `GET` 回读但不随底盘参数存Flash。
**2mm 死区必须配合 `VMIN` 一起看**：8RPM≈33.6mm/s，一个 20ms 周期就走约0.67mm，
单侧 2mm 余量容易被一帧过冲穿过，出现"停→起→停"就该先降 `VMIN` 而不是放大死区。


2026-09-19：当前坐标/故障恢复/任务时序修复见 [修复说明](../修复说明_20260919.md)。
# SPI Flash 参数持久化（2026-09-20）

天空星板载W25Q128已接入：PA4片选、PA5/6/7=SPI1，新增独立
`spi_flash.c/.h`与`debug_param_store.c/.h`。在线调参稳定2秒后自动保存，
重启自动恢复；仅使用最后8KB，双扇区日志+CRC32+提交标记。
详见[Flash参数保存说明](Flash参数保存说明.md)。


## 2026-10-04 公共坐标位置闭环

`chassis_position.h`声明浮点参考接口`chassis_move_reference`，实现在既有`mecanum_control.c`中，无新增编译单元。原`chassis_move`、轨迹起点保持、转头保持车心与行进纠偏共用该核心；轨迹使用同一OPS快照，保留单调进度、前馈与STOP停稳。单点底盘参数与轨迹参数的轴和航向单位保留各自范围。说明见[坐标控制说明](../HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/CHASSIS_POSITION_20261004.md)。
