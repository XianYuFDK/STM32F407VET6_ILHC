# 工程导航与维护约定

## 当前坐标契约（2026-09-17，优先级最高）

全工程统一使用以下车体/场地坐标，**不再建立任何内外坐标交换或取反层**：

- `+X = 车左`，`-X = 车右`
- `+Y = 车头`，`-Y = 车尾`
- `+Z = 逆时针`

固件变量、串口协议、遥测、底盘位置环和 Qt 内部状态都直接使用该轴序：

- `pos_x` 就是 X（左右，+左），`pos_y` 就是 Y（前后，+前）；
- `MANUAL/GOTO/OPSOFFSET` 的 X/Y 原义传递，不再交换；
- `KPX -> mKpx`、`KPY -> mKpy`；
- 麦轮公式：`[+Y-X-Z, -Y-X-Z, +Y+X-Z, -Y+X-Z]`；
- OPS 原始帧到统一坐标固定为 `X=-raw_y`、`Y=-raw_x`，不保留方向模式或分支；
  位置、置零、原点设置和偏心补偿必须共用该固定映射；
- OPS 默认安装统一坐标 `(X=左60, Y=后-50)mm`，协议下发
  `OPSOFFSET=60.0,-50.0`；
- Qt `manual_vector` 与控件偏移也使用 `(X,Y,Z)`，不保留旧的
  `(前后,左右)` 本地顺序。

本文件下方较早的坐标描述中，凡出现“内部前后/左右”“交换后取反”
`Debug_UserToInternal`/`Debug_InternalToUser`、`KPX->mKpy`、或把
`pos_x` 描述为前后轴的段落，均属于历史记录，已被本节取代，不得据此实现。

2026-09-17 主接收端协议升级：`Hardware/ops.c/.h` 现同时解析旧
`0x5C`/14B/CRC8 与新 `0x5D`/28B/CRC16 帧；USART2 改为 64B 流式 DMA 解析，
支持 flags、session_id、拆包重同步和错误回调任务级重挂。初始化先发旧
`C5 30` 兼容旧 OPS，再发 `C5 32` 固定新协议方向 2；V2 session 变化会取消
旧 GOTO 并重设本地原点参考。回归入口：`Tests/hardware/test_ops_protocol.py`。

2026-09-17 单位变更（最新）：移动/定位协议的 `GOTO=X,Y,Z` 中 X/Y 改为 **cm、保留 1 位小数**，
24 通道遥测的 ch0/ch1（位置）和 ch3/ch4（误差）也改为 cm；F407 内部 PID、限幅和到位阈值仍使用 mm。
协议边界按 `1 cm = 10 mm` 换算，GOTO 对外范围 ±300.0cm（内部 ±3000mm）；OPSOFFSET 与步进命令继续使用 mm。
Qt 地图几何/碰撞检查继续用 mm，但显示、地图原点输入和 GOTO 发送全部使用 cm。

2026-09-17 轮控回退（最新，撤销上一轮 2/4 号输出极性补偿）：实车按 W 变成旋转后，已从
`SetMotorVoltageAndDirection()` 删除 `s_motor_dir_invert` 和额外取反逻辑，恢复为“逻辑轮速正号转
CW、负号转 CCW”。混控公式保持不变；W 最终 ZDT 图案仍是 `[+ - + -]`，A 为 `[- - + +]`，
Q 为 `[- - - -]`。不要在移动控制层再次加入 2/4 轮极性翻转；回归入口
`Tests/hardware/test_mecanum_mixer.py`。

2026-09-17 回退（**推翻**下面第一条的 y 取反）：实车 `GOTO=0,1000,0` 表现为"先前 1 米、再横移"，
且地图显示航向≈90°。结论：
① **"车实际向右 / 上位机显示向左"不是矛盾，而是坐标系混淆**：地图抬头写明"航向0°左、90°上"，
车头朝上(+Y)时车的**自身右侧 = 场地 +X = 地图左边**，两者其实是同一个运动；
② 航向 ≠ 目标航向时，世界坐标 GOTO **本来就必须横着走**（验收测试10），横移本身正常；
③ 真正待查的是"横向过冲不收敛"（X 到 +700mm 而目标 0）与"航向没到 0°"这两条；
④ 下面那条"横向跑飞 ⇒ 测量符号反"的推理**前提不成立**：那次跑飞发生在仍带 `chassis_move`
轴通道互换的旧固件上，互换本身就足以把前进误差灌进横向轮通道、造成同样现象。
而 y 取反一旦方向错就会把横向环变成**正反馈（跑飞）**，风险不对称 ⇒ 已回退：
`ops.c` 恢复 `rx=+s_mount_x_mm`、`ry=+s_mount_y_mm`、输出 `*y = py - ...`（y 不取反），
只保留"左手轴对 ⇒ dx/dy 里 rx 项取反"这一处（对应"原地旋转算出位移"的反馈）；
`core.py` 模拟器同步回退；`test_coordinate_chain.py` 增加 `must_not(o, "*y = -(py - dy);")`。
**判别 OPS 左右轴方向的正确实验（不经过闭环）**：把车抬离地面，发 `MANUAL=100,0,0`（应左移），
看遥测 X —— **增大** ⇒ 当前假设成立；**减小** ⇒ OPS 原始 y 实为车右，按 `ops.c` 注释把
rx/ry 取号互换并把输出 y 一并取反（两行）。确认前**不要跑 GOTO**（符号错会跑飞）。
`Code=32188`，11 个固件套件 + Qt 31 用例全通过。

2026-09-17 追加：**OPS 左右轴符号修正（横向跑飞/显示镜像的根因）**。实车现象：`GOTO=14,222,-9`
（|Y|≫|X|，几乎纯前进）却让车**横向飞快跑**，且地图显示与实际方向**左右相反**。
推理：① 轮子侧 `+vy → 车左` 已实测（A/D 那次，走 MoveVelocity，与 chassis_move 无关）；
② 几乎纯前进的指令造成横向大位移 ⇒ 横向环是**正反馈** ⇒ 测量侧与指令侧反号；
③ 几何必然：OPS 原始帧是**右手系**，已实测 `+x_raw = 车尾`（车向车头走 pos_x 减小），
故 `+y_raw` 必为**车右**（`(车尾,车左)` 是左手系，不可能同时成立）。
改法（**只动测量侧，A/D 指令侧不动**）：`ops.c` 的偏心补偿改回**原始帧标准 CCW 矩阵**
（`dx = dc*rx - ds*ry`、`dy = ds*rx + dc*ry`），其中安装偏移取原始帧分量
`ry = -s_mount_y_mm*0.001f`（`s_mount_y_mm` 仍是"车左+"对外语义，默认 +60 = 车左60mm 不变），
输出端 `*y = -(py - oy - dy)` / `*y = -(py - dy)` 把原始帧 +车右 翻成内部 +车左。
同时 `core.py` 模拟器的同一模型同步（`ey = -(60 - ops_offset[1])` 且 `pos_y -= -(ds*ex + dc*ey)`）。
`test_ops_offset.py` 的合成轨迹改回"传感器在原始帧里按 R(Δθ)·(0.05,-0.06) 绕中心旋转"，
修正后各角度补偿结果恒等于中心（原地旋转不画圆）；`test_coordinate_chain.py` 锁死新的矩阵与
两处 y 取反，并把"未取反的 y 输出"列为 must_not（防横向正反馈回归）。
**注意**：上一轮我按"内部轴对是左手系"把 rx 取反的写法（`-dc*rx`）是**错的**，已回退——
那次改动虽然有 `test_ops_offset.py` 通过，但那个测试的合成模型是我自己按同一假设写的，
属于自证；本轮改为以"原始帧右手系 + 实车正反馈证据"为准。
`Code=32200`，11 个固件套件 + Qt 31 用例全通过。

2026-09-17 坐标系/麦轮/OPS/ZDT/上位机**全链路整治**（按"资深嵌入式运动控制 + 代码维护"要求逐层核实后修改）：
三块子系统的逐行取证（mecanum_control、zdt_x42s+debug_usart、ops+Qt）各有一份审计结论，据此改动如下。
**① 坐标适配层（新增，唯一转换点）**：`debug_usart.c` 顶部新增 `Debug_UserToInternal()` /
`Debug_InternalToUser()` 两个 static 函数，把"对外 X=车左/Y=车头/Z=逆时针"与"内部 x=前后(**正指向
车尾**)/y=左右(正=车左)"的映射集中在一处；遥测打包、MANUAL、GOTO、OPSOFFSET 四处边界全部改为调用
它们（原先四处各写一遍 `-v[1]`/`pos_y`）。`test_coordinate_chain.py` 增加"适配层数学关系"与
"适配层只允许出现在 debug_usart.c"两条断言，禁止二次交换。
**② 麦轮四轮整体同比限幅（新增，修真 bug）**：全工程原先没有整体缩放，四轮最终各自被
`zdt_x42s.c` 的 `if (rpm > 3000) rpm = 3000;` **单轮硬裁剪** ⇒ 斜行+旋转时四轮比例被破坏、方向偏移；
且车体两轴分量在 `numerical_limit` 里是"分别"限幅的（vx1/vx2/vy1/vy2 各自 ≤ XYVmax），
`VX=vx1+vx2` 最大可达 2·XYVmax，四轮峰值 ≈ (4·XYVmax+ZVmax)·0.238 ≈ 1701 RPM。现新增
`Mecanum_NormalizeWheelSpeed()`（float 运算，`scale = limit/max_abs`，四轮同乘），在
`SetMotorVoltageAndDirection()` 下发前统一调用，两条路径（MANUAL/GOTO）都被覆盖。
**③ OPS 偏心补偿矩阵修正（修真 bug，"原地旋转画圆"的根因）**：内部 (x=前后,y=左右) 相对
"z 向上、航向逆时针"是**左手轴对**，因此世界旋转必须用 `m_world = (-rx, ry)`：
`dx = -dc*rx - ds*ry; dy = -ds*rx + dc*ry;`（原为 `+dc*rx`/`+ds*rx`，前后项的旋转方向被镜像）。
配套把默认安装偏移由 `(-50,+60)` 改为物理真值 `(+50,+60)`（内部 +50=车后50mm、+60=车左60mm），
两者互相抵消过所以以前"看起来能跑"。`test_ops_offset.py` 用合成"传感器绕中心旋转"轨迹做端到端验证：
修正后 0/90/180/360/-90° 补偿结果恒为 (0,0)，平移+旋转也精确复原（=验收测试13）。
**④ ZDT 单轮测试其余三轮改为 Disable**：原为 4 轮全 `Stop`（Hold 下仍锁轴，给悬空单轮测试带来阻力）；
现被测轮 Stop+Enable、其余三轮 Disable（释放锁轴），测试结束/取消统一走新增的
`Debug_ZdtTestFinish()`（被测轮 Stop + `Debug_ChassisStop()` 把四轮恢复成"0 转速+锁轴"，
与固件内部 `s_wheel_enabled=1` 保持一致）。
**⑤ ZDT ACK 校验升级**：原先只比 `frame[0]`（地址），Stop/Enable 的迟到回包会被当成速度帧成功应答；
现新增 `s_zdt_watch_cmd`，必须 **地址 + 功能码(0xF6)** 同时匹配，且**状态码 == 0x02** 才算成功，
非 0x02 新增事件11 文本 `ERR ZDT REPLY STATUS != 0x02 (SEE RAW FRAME)`（原先 0xE2/0xEE 与成功无区别）。
**⑥ Qt 侧**：`manual_vector` 保持**(前后, 左右, 旋转)物理顺序**，`_manual_tick()` 发送前交换为协议
顺序 `MANUAL=X(左右),Y(前后),W`；"反向"开关顺序保持（前后反向/左右反向/旋转反向）。
**⑦ 串口健壮性**：`zdt_x42s.c` 滑窗写入前补 `if (s_rx_count >= 4U) s_rx_count = 0U;` 越界兜底。
**未改动（经核实本来就正确）**：世界→车体的旋转矩阵（`chassis_move` 229-237，θ 用弧度、
`R(-θ)·diag(mKpx,mKpy)`，等价于 `VX=cosθ·Kpx·devx+sinθ·Kpy·devy`）、`devz` 的 ±180° wrap
（217-226，在增益与到位判断之前）、`zangle` 的 [-180,180] 归一化、到位窗口(60mm/15°/11 周期去抖、
near 100mm/30°)、`devx=目标-当前` 的负反馈配对、轮序（俯视车头朝上：左前1/右前2/左后3/右后4）、
ZDT F6/Emm 帧格式、USART1 的 DMA+空闲接收与行缓冲边界、Qt 的 JustFloat/ASCII 双模式解析
（`_scan_text` 遇到二进制字节即丢弃候选行）与地图单次交换（`field_to_layout` 只做显示旋转+偏移）。
**已知遗留（未改，需实车确认）**：a) P 增益乘在旋转之前，`KPX≠KPY` 时行进方向会有偏差（默认两者
都是 2.3 时精确等价，故未动公式，已在 `chassis_move` 注明）；b) `ZDT_X42S_SpeedAcc` 内仍保留
单轮裁剪（现在因上游整体限幅而不会触发，属于最后一道保险）；c) `vKpx/vKpy/vKpz/cvKpz` 四个全局量
声明后全工程未使用；d) OPS 的 `s_mount_x_mm` 符号结论依赖"传感器 +x 与车头同向"这一安装假设，
若实车旋转测试仍画圆，把 `ops.c` 补偿矩阵的两个 rx 项符号再翻一次即可（一行）。
**回归**：11 个固件套件 + Qt 31 用例全通过；`Code=32192`，0 错误 0 警告。

2026-09-16 GOTO 横移修复（实车："手动 `GOTO=0,1000,0` 想让车沿 Y 前进 1 米，车却直接向右移动"）：
**这正是此前记录的 `chassis_move` 轴通道互换，实车把它证实了**——原公式把车体"前后"(vx1+vx2)与
"左右"(vy1−vy2)两个分量送进了相反的槽位（`chassis_move` 实际等价于 `MoveVelocity(vx=左右, vy=前后)`），
所以 MANUAL（走 MoveVelocity）方向正常、而 GOTO（走 chassis_move 的位置环）前进变横移。之前 OPS 是死的、
GOTO 直接被拒，所以这个 bug 一直没暴露；OPS 修好后位置环真跑起来就露出来了。
修复：只对调 `speed[1]`/`speed[2]` 的交叉项（`speed[0]`/`speed[3]` 对两个分量对称，不改）：
`speed[1] = (int) (vx1 + vx2 - vy1 + vy2 - vz);`、`speed[2] = (int)-(vx1 + vx2 - vy1 + vy2 + vz);`。
数值等价验证：406 组样本修正后与 `MoveVelocity(vx=B, vy=A)` **逐轮完全一致（0 处不一致）**，修正前 404 处
不一致；"向前"(A=0,B=100) 的轮子模式由 `(-100,-100,+100,+100)`（前后互顶）变为 `(-100,+100,-100,+100)`
（与实车已验证的 MANUAL 前进模式相同）。护栏：`test_coordinate_chain.py` 第 5 节改为断言修正后的四式，
并把修正前的两行列入 `must_not`，防止回归。复位后 `Code=31916`，12 套回归全过。
**注意：本次同样没能烧录**（SWD `connect under reset failed`，板子无法 attach），需自行烧录
`MDK-ARM/build/STM32F407VET6_ILHC/STM32F407VET6_ILHC.hex`（21:27:08，SHA256 前缀 B8BB16E886603C17）。
**验收**：`GOTO=0,1000,0` 沿车头前进 1 米；`GOTO=1000,0` 向车左平移 1 米；`GOTO=0,0,90` 原地转到 90°；
地图点击导航走同一路径，随之恢复正常。同时确认 MANUAL 的 W/S/A/D 未受影响（两条公式现在同约定）。

2026-09-16 车头方向修正（实车两次反馈：先"x、y 车头方向反了，按 W 的移动方向才是车头"，再
"键盘遥控 A/D 左右反了、地图点击坐标的换算不对"）：由此确认**底盘内部坐标系与真实车体是镜像
关系**——内部 `pos_x` 指向车尾，而 `pos_y` 与车左**同向**。第一轮按"相差 180°"把两个轴都取了
反，实车 A/D 立刻反了，直接证明左右轴**不需要**取反：正确修法是**在四个对外边界统一做一次镜像
（交换后只把前后轴取反）**，底层解算与内部坐标系完全不动：
`debug_usart.c` 遥测 `data[0]=pos_y`、`data[1]=-pos_x`、`data[3]=devy`、`data[4]=-devx`
（ch6/ch7 只交换不取反）；`MANUAL` → `MecanumControl_MoveVelocity(-v[1], v[0], v[2])`；
`GOTO` → `s_goto_x=-v[1]`、`s_goto_y=v[0]`；`OPSOFFSET` → `s_offset_x=-v[1]`、`s_offset_y=v[0]`。
这也解释了团队此前在 Qt 里勾"前后反向"救 W 的历史：`MANUAL` 被界面补偿过，但 **GOTO 与遥测从未
补偿**，所以坐标一直是反的。Qt 侧同步：三个"反向"开关全部默认关闭（`manual_invert` 由 `i==0`
改为全 False，根因已在固件修掉，开关只作兜底）；`test_vehicle_forward_default_is_reversed` 改写为
`test_manual_direction_defaults_not_inverted`；**`core.Simulator` 必须同步镜像**（MANUAL/GOTO/
OPSOFFSET 解析、遥测打包、偏心默认值改为内部(+50,+60)），否则 simulate 模式下点击地图会朝镜像
方向跑——实车反馈的"地图点击坐标的换算不对"就是这个原因。镜像在 GOTO 环路里自洽（目标与测量
一起镜像），**GOTO 控制律与地图显示本身无需改动**（地图只消费遥测）。回归：`test_coordinate_chain.py`
断言四处边界只对前后轴取反（并 `assert "-v[0]" not in d`，防止再犯 A/D 那个错），
`test_parse_line_axes.py`、`test_debug_manual.py`、Qt `test_debugger.py` 同步更新，12 套全过。
在板验证（上一版 180° 实现）：遥测缓冲 `s_tx` 中 ch0..ch3 为 `0x80000000`(-0.0)；DMA2_Stream7
抓到 `EN=1` 且 NDTR 85→41→13 的在途帧。**本镜像版已编译（Code=31908），但当时板子无法 SWD
attach（VTref=3.3V 却 connect under reset 失败），尚未烧录。**
**遗留待实测**：① `ops.c` 默认安装偏移 `(-50,+60)` 与"车左60/车后50"不符（内部应为 `(+50,+60)`，
即下发 `OPSOFFSET=60,-50`），默认值疑似反号，需实车确认后再改默认；② 实体方向按验收表逐项确认：
W→车头、S→车尾、A→车左、D→车右；③ OPS 无数据时无法远程验证坐标符号（OPS 目前完全静默）。

2026-09-16 CAN1→CAN2 改造与在板诊断（含 J-Link 实测）：应要求把整条 CAN 链路从 CAN1 换到 CAN2。
改动：`Core/Src/can.c` 句柄 `hcan1`→`hcan2`、`MX_CAN1_Init`→`MX_CAN2_Init`（Instance=CAN2，
时序参数与原来相同；MspInit/DeInit 改为 **CAN1+CAN2 双时钟使能** + **PB5=CAN2_RX/PB6=CAN2_TX
(AF9)** + NVIC `CAN2_RX0_IRQn` 优先级5）；`Hardware/hcan.h` 的 `HCAN_CAN_NUM` 改指 `&hcan2`；
`Hardware/hcan.c` 的滤波器 **FilterBank 0→14**（CAN2 是 CAN1 的从机，滤波器寄存器在 CAN1
地址空间，只能用 Bank14~27，且 CAN1 时钟必须使能）；`stm32f4xx_it.c` 的 `CAN1_RX0_IRQHandler`
→`CAN2_RX0_IRQHandler`；`main.c` 改 `MX_CAN2_Init()`/`CAN_Start(&hcan2)`；`.ioc` 同步
（CAN2.*、PB5/PB6、NVIC.CAN2_RX0、functionlistsort 的 MX_CAN2_Init）；测试桩
`Tests/hardware/{main.h,regression.c}` 与 `test_can_degraded.py` 同步改名。
**实测结论（J-Link SWD 1000kHz，不停CPU读内存）**：
① 固件侧 CAN2 配置正确——`hcan2.Instance=0x40006800`、`Init` 五项与原来一致、`BTR=0x00220005`
（1 Mbps）、`GPIOB MODER/AFRL` 显示 PB5/PB6=AF9、NVIC 里 CAN2_RX0(IRQ64) 已使能；
② 但 **PB5 悬空**：把 PB5 内部上拉打开后 `GPIOB IDR` 的 bit5 由 0 变 1，且 **CAN2 `MSR.INAK`
立刻由 1 变 0**（bxCAN 一旦看到隐性位就退出初始化模式）——证明之前一直卡在初始化模式纯粹是
"没有收发器接到 PB5"，不是 MCU 配置问题；③ 因此 `HAL_CAN_Start` 等 `INAK` 清零超时，
`ErrorCode=0x00020000 (HAL_CAN_ERROR_TIMEOUT)`，`State=5`——注意 **本 HAL 里
`HAL_CAN_STATE_ERROR=5`、`LISTENING=2`**，且 `CAN_HandleTypeDef` 无 Lock 成员、
`FunctionalState` 是 1 字节，故 Instance(+0)/Init(+4)/State(+32)/ErrorCode(+36)。
**本 HAL 的 `HAL_CAN_Start` 会等待 INAK 清零并可能返回 HAL_TIMEOUT**（旧认知"Start 不检查
总线"不适用于此版本）。
串口侧：已把 CAN 故障与遥测**彻底解耦**——`DebugUsart_Init` 与 `Debug_RejectCanCommand` 都
不再置 `s_zdt_text_mode`，只排队报错；`test_can_degraded.py` 增加断言（CAN 失败必须报错且
**不得**置文字模式）。实测 CAN 正处于失败状态时，USART1 遥测照样连续输出（gState 在
READY/BUSY_TX 间循环、DMA2_Stream7 的 NDTR 连续递减、PA9=AF7 且空闲为高）。
OPS 侧：`s_ops` 全 0（`valid_count=0` **且** `error_count=0`）、USART2 `SR` 无 RXNE/ORE/FE、
RX DMA `NDTR` 恒为 14/14 ⇒ **OPS 一个字节都没发过来**，不是坐标换算问题。
串口接线提醒：本机两个 CH340 中 **COM9 才是调试口（常被上位机占用）**，COM14 恒为 0 字节，
排查时不要抓错口。
遗留：① CAN 收发器必须实际接到 PB5/PB6 并上电，否则 CAN2 同样起不来；② USART1 TX 缺超时
恢复——`HAL_UART_Transmit_DMA` 返回值被忽略且以 `gState` 作为闸门，一旦 TX 卡在 BUSY_TX
遥测会静默永久停止，建议加"连续 N 周期非 READY 就 Abort 复位"；③ 收发器与总线
（CANH/CANL 短路、终端电阻、DM 电机供电）仍需万用表排查。

2026-09-16 遥测静默修复（上位机侧，无需烧录）：实车抓到 COM14 原始数据，遥测帧正常
（ch6/ch7=2.3、ch8=9.0、ch9=1600、ch10=750）之后紧跟一行 ASCII
`ERR CAN START FAILED; CAN DISABLED; USART1 AVAILABLE`，随后串口再无数据。根因链：
CAN1 启动失败 → `DebugUsart_Init` 置 `s_zdt_text_mode=1`（`debug_usart.c:1040-1044`）
关掉全部 24 通道遥测 → 只有 `VOFA` 能恢复 → 旧上位机**只在打开串口那一刻发一次 VOFA
（`core.py:385`）**，板子若在上位机已连接时复位/上电（或那次 VOFA 落进 `OPS_Init` 约 1.3s
的启动空窗），之后再无补发机制 → 永久 0 字节。同时 `FrameParser` 只认 JustFloat 帧，
固件这行关键报错被**静默丢弃**，界面上完全看不到。修复（`ILHC_Qt_v2`）：
① `SerialWorker` 增加 VOFA 自动补发——超过 `VOFA_RETRY_GAP_S=1.5s` 没有可解析帧就补发，
每次中断最多 `VOFA_MAX_RETRIES=6` 次，收到任一帧即重置计数，覆盖复位后 1.3s 启动空窗；
② `FrameParser` 增加 `_scan_text`/`take_text`，把固件文字应答（CAN 失败、ZDT 应答等）
提取出来经新增的 `text_q` 显示到日志，不再吞掉；③ `main.py` 新增 `fw_text_q` 并在
`_process_frames` 里按 ERR/FAIL 关键字着色。回归：Qt `test_debugger.py` 新增
`test_firmware_text_is_surfaced_not_swallowed`、`test_firmware_text_and_vofa_notice_reach_log`、
`test_serial_worker_resends_vofa_when_silent`，共 31 用例全过。**仍待处理**：CAN1 启动失败
本身未解决（DM/S28/S35 全部不可用），且 `CAN_Start` 的失败步骤与 HAL 错误码没有上报，
无法定位是 `HAL_CAN_ConfigFilter`/`HAL_CAN_ActivateNotification`/`HAL_CAN_Start` 哪一步；
建议固件改为 CAN 失败**不再关闭遥测**（只报一次错误）并在报错里带上 `ErrorCode`。
注意 bxCAN 进入 NORMAL 模式不需要总线上有其它节点，所以 START 失败通常不是 CANH/CANL
接线问题。

2026-09-16 坐标统一：对外统一为 +X=小车左方（左右轴）、+Y=小车正前方（前后轴）、+Z=逆时针为正，
无论是否按过 ZERO 都是物理正向（左移 X 增大、前移 Y 增大）。ops.c 去掉"保留既有 ZERO 反号"分支，
置零时改为 `*x = px - ox - dx`、`*y = py - oy - dy`（原为 `GetPosition = -(原始相对位移 - 偏心旋转位移)`）；
未置零分支、yaw 不置零、`OPS_GetAbsolutePosition()`/`OPS_GetData()->frame` 仍返回原始数据、偏心补偿
`中心相对位移 = 原始OPS位移 - (R(yaw)-R(参考yaw))*r` 的形式均未变。mecanum_control.c 的 chassis_move()
误差同步改为 `devx = 目标 - pos_x`、`devy = 目标 - pos_y`（原为 当前-目标），与 ops.c 必须成对出现，
否则位置环变成正反馈；置零状态下逐周期轮速与改动前完全相同，底层两条麦轮公式本轮刻意未改。
debug_usart.c 只在边界各交换一次：遥测经适配层后 `data[0]=user_x*0.1f`、`data[1]=user_y*0.1f`、
`data[3]=err_x*0.1f`、`data[4]=err_y*0.1f`、`data[6]=mKpy`、`data[7]=mKpx`（ch2/ch5/ch8 及其余通道、
24 通道数与 JustFloat 帧尾不变；固件内部变量仍是
`pos_x`=前后、`pos_y`=左右，交换只发生在打包/解析边界）；`KPX` 改指左右轴增益 `mKpy`、`KPY` 改指前后轴
增益 `mKpx`，两个命令名和 0~50 范围不变。协议：`MANUAL=X(左右),Y(前后),W`，固件调用
`MecanumControl_MoveVelocity(v[1], v[0], v[2])`（该 C 接口形参顺序仍是(前后,左右,旋转)）；
`GOTO=X(场地左右),Y(场地前后),Z`（Z 可省略则保持当前航向，x/y 为 ±300.0cm；固件内部换算为 ±3000mm）；`OPSOFFSET=X(左右安装偏移,
左+右−),Y(前后安装偏移,前+后−)`（±500mm），物理默认安装（车左60mm、车后50mm）现下发
`OPSOFFSET=60.0,-50.0`，`OPS_SetMountOffset` 形参顺序仍是(前后,左右)。Qt：地图 +X=屏幕左、+Y=屏幕上，
与协议轴序重合，`_ops_to_field`/`_field_to_ops` 不再交换或取反、只做一次 `map_theta` 标定旋转，地图几何、
ZONE_CENTER、禁区/圆形障碍、快速目标和已有 map_theta 标定值均未改变；键盘遥控按键语义不变，且
"实车确认原方向按 W 后退→默认勾选前后反向"仍然有效，只作用于键盘手动这一路；Qt 的
`core.ops_offset_command(left_mm, forward_mm)` 按协议顺序取参，界面两个 spinbox 仍标 前后偏移/左右偏移，
JSON 键 `x_mm`=前后、`y_mm`=左右保持不变（与线序相反）；遥测显示名改为「OPS X 坐标（左右）」
「OPS Y 坐标（前后）」
「X 轴误差（左右）」「Y 轴误差（前后）」「X 轴 P（左右）」「Y 轴 P（前后）」，底盘参数标签为
「X 轴 P 系数（左右）」「Y 轴 P 系数（前后）」，CSV 列名 `pos_x/pos_y/devx/devy…` 不变。回归：新增
Tests/hardware/test_coordinate_chain.py（源码文本锁定上述边界交换，并断言 24 通道帧格式与麦轮公式未被波及）
与 test_parse_line_axes.py（编译真实 Debug_ParseLine，验证 GOTO/OPSOFFSET/KPX/KPY 的内部落点、±300.0cm→mm 钳位、
省略 Z、超限钳位到 50 与 WHEELOFF 失能闸门）；test_ops_offset.py、test_debug_manual.py 按新符号/新顺序更新；
Qt 新增 test_telemetry_direction_matches_field_axes。**当时的"已知未修"问题**（chassis_move 与
MecanumControl_MoveVelocity 两条麦轮公式的轴通道归属互换）已于同日按实车证据单独修复，见本文件
最上面 2026-09-16 GOTO 横移修复条目。

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
OPS遥测/GOTO保持固件坐标，由main.py转换（已由 2026-09-16 坐标统一更新，见上）。新增28/35页：机械目标、原始位置/RPM、回零、取消待发。
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
    ├── ILHC_Qt_v2.zip                Qt 版本打包备份
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

- OPS 上行同时支持：V1 `0x5C + float32 x + float32 y + float32 z + CRC8`（14 字节）；
  V2 `0x5D + ver/len/flags + seq + session_id + timestamp + float32 x/y/z + CRC16`（28 字节）。
  V2 位姿必须同时满足 `POS_VALID/IMU_ONLINE/ENC_VALID` 才会发布；CRC/字段定义以 `ops.c` 为准。
- OPS 原始坐标与 `OPS_GetPosition()` / `OPS_GetAbsolutePosition()` 返回单位为 **m、rad**；`mecanum_control.c` 内统一转换为 **mm、deg**。
- `OPS_ZeroCoordinates()` 在本地同时记录 X/Y 原点和当前 Z 零点；不重置 OPS 本体。`OPS_ClearZero()` 恢复绝对 X/Y/Z。置零分支不再反号：`GetPosition = 原始相对位移 - 偏心旋转位移`，与非置零分支同为物理正向（统一坐标 `pos_x` 向左增大、`pos_y` 向前增大，`Z` 以清零姿态为 0）。
- `OPS_Init()` 发送 `0xC5 0x22` 复位，先发 `0xC5 0x30` 兼容旧 OPS 启动，再发 `0xC5 0x32`
  固定新协议方向 2；包含启动等待，不是可在中断里调用的轻量操作。USART2 错误恢复由
  `OPS_ServiceRx()` 在默认任务上下文执行。
- `chassis_move(x,y,z)` 计算位置误差、P 控制、限幅、速度斜坡及到位状态；由 `SetMotorVoltageAndDirection()` 实际下发轮速。误差定义为 `目标 - 当前`（`devx`=X左右、`devy`=Y前后），必须与 ops.c 的置零正向坐标成对，否则位置环变正反馈；麦轮矩阵保持统一坐标公式，输出层只做逻辑符号到 CW/CCW 的直接映射，不得再加入 2/4 轮极性翻转。
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
- 对外与内部统一坐标：+X=左右轴（车左+）、+Y=前后轴（车前+）、+Z=逆时针为正，置零与否符号一致；固件 `pos_x` 就是 X，`pos_y` 就是 Y，遥测、`MANUAL`/`GOTO`/`OPSOFFSET` 和 `KPX`/`KPY` 均直接使用该轴序，不存在交换层。Qt `core.py/main.py` 使用相同轴序（CSV 列名仍为 `pos_x/pos_y/devx/devy`）。
- 固件不提供普通文本 ACK；不要把打印文本混入同一遥测流。
- `PING` 建议每 200ms 发送；固件超过 1s 没收到完整命令行时停止活动的 GOTO/DM 调试动作。上位机由 GUI 主循环产生心跳。
- `STOP` 取消 GOTO、停车并失能 DM；`ZERO` 先取消定位移动并停车，再置本地 X/Y 原点。
- `WHEELEN`/`WHEELOFF` 只切换四轮锁轴，不改变 DM 和 28/35；失能期间拒绝 GOTO/MANUAL/ZDT，需显式重新使能。无锁轴状态回读。锁轴切换每周期最后执行；**失能后任何路径都不得再向 UART4 发速度帧**，因为 ZDT_X42S 速度命令会重新使能锁轴，停车只能走 `MecanumControl_ClearTarget()` / `Debug_ChassisStop()`。
- 修改帧格式、通道或命令范围时，同时核对固件、Qt `core.py/main.py`、`Hardware/debug_usart.h` 和中文手册。原 Tkinter 上位机（`ilhc_debugger.py`）已在 2026-09-16 的提交中删除，协议只由 Qt `core.py` 镜像，不要再引用它。

## 6. 构建与验证

EIDE 与 Keil 共用源文件，但维护各自清单。当前 EIDE 配置名为 `STM32F407VET6_ILHC`，工具链 AC5、C99、单精度硬件浮点。已核对本机编译器目录为 `D:\keil51\Arm\ARMCC`，不要套用其他项目的 AC6 路径。

在 `MDK-ARM/` 下执行现有 EIDE 构建参数：

```powershell
& 'C:\Users\Yuzi\.vscode\extensions\cl.eide-3.27.2\res\tools\win32\unify_builder\unify_builder.exe' -p 'build\STM32F407VET6_ILHC\builder.params' --no-color
```

该路径是当前机器配置，换机或扩展升级后先查找实际安装位置。`builder.params` 是 EIDE 生成文件；缺失或源清单过期时通过 EIDE 重新生成，不将它作为唯一配置源。固件产物位于 `MDK-ARM/build/STM32F407VET6_ILHC/`，包括同名 `.hex`；调试文件名以实际构建输出为准。

从工程根目录运行上位机无 GUI 自检：

```powershell
py -3 HostTools\ILHC_Debugger\ILHC_Qt_v2\main.py --selftest
```

Qt 完整运行依赖见其 `requirements.txt`，`--selftest` 在加载 Qt 界面前执行，可用 `--simulate` 查看无硬件演示。注意 `core.selftest()` 末尾的中文与 `✔` 输出在默认 cp936 控制台会抛 `UnicodeEncodeError` 并以非 0 退出，验证时先设 `PYTHONIOENCODING=utf-8`。

按变更范围验证：固件改动编译；协议改动验证两版解析/模拟器；文档改动检查路径和描述即可。上位机模拟器不能证明 MCU 调度、CAN 应答或真实机械运动正确。报告时区分静态检查、编译、自检和实机验证，不能沿用旧构建结果声称本次验证成功。

## 7. 后续修改规则与已知边界

- 新增或修改的自定义代码注释、维护说明使用中文，保持实现简洁、可扩展，优先局部修改。
- 保留现有用户改动；开始前检查 `git status`。未被要求时不恢复 RC、完整塔吊应用层或独立 PID 架构。
- 新增 `.c` 文件需要同步 EIDE `virtualFolder` 与 Keil `Hardware` 分组；当前并非自动扫描全部 `Hardware/*.c`。
- CubeMX 自定义代码尽量放 `USER CODE` 块；引脚、DMA、中断和 HAL 配置变更同时核对 `.ioc`。当前存在手工维护的初始化/IRQ 代码，再生成后必须检查差异。
- 中断回调只做必要解析和状态更新；阻塞发送、等待、复杂控制放任务中。共享结构快照保护需恢复进入临界区前的 PRIMASK，不可无条件开中断。
- 新增 UART4 轮速输出前先确认四轮使能状态：ZDT_X42S 在速度模式下收到速度命令会重新使能并锁轴，失能后多一条速度帧就会把失能帧覆盖掉。停车分两条路径，`MecanumControl_Stop()` 会发速度 0 帧，`MecanumControl_ClearTarget()` 只清软件目标。
- DMA 发送缓冲区在传输完成前不能重写；新增任务或增加局部缓冲区时核对默认任务 512 字节栈及 RTOS 堆。
- OPS 当前使用 64 字节 DMA 缓冲和逐字节流式解析，支持 V1/V2 拆包、粘包和噪声后重同步；
  但仍然依赖真实 USART2/DMA 中断时序和 OPS 端 CRC 正确，不能把主机回放测试等同于实车验证。
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

2026-09-10 手动底盘更新：MANUAL=vx,vy,w，每轴整数±300 RPM，独立350ms续期，PING不续期；串口中断存请求，任务服务调用MoveVelocity；与GOTO互斥，STOP/ZERO取消。Qt底盘页按住运行、松开/失焦/切页停车，含方向反向开关；地址俯视左前1右前2左后3右后4。协议说明见调试手册。（已由 2026-09-16 坐标统一更新：MANUAL=X(左右),Y(前后),W，见上）

2026-09-10 OPS偏心更新：F407 ops.c默认rx=-50mm/ry=+60mm（车后50左60），GetPosition在ZERO反号之前按航向去除偏心旋转位移；原始GetAbsolutePosition不变，ZERO同时保存yaw。OPSOFFSET=x,y（±500mm）成对调参，任务停车取消手动/GOTO后应用并置零；RAM参数，Qt可保存/加载电脑JSON，无Flash持久化和参数回读。24通道不变，发送遥测前GetPose刷新补偿位置。测试Tests/hardware/test_ops_offset.py。（已由 2026-09-16 坐标统一更新：置零不再反号，OPSOFFSET=X(左右),Y(前后)，见上）

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
