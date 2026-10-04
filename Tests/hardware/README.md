# 驱动主机回归测试

完整比赛整批轨迹：`python Tests/hardware/test_trajectory_buffer.py`（GCC及Qt目录现有Python依赖）。编译真实C接收/跟踪模块，与Python上传器端到端运行8个自动跑图场景、每轮7处站点实际启停、零TRESUME及取消/CRC/定位/调度等故障。点击路径另验证完整上传、初始/最终转头、连续路径和停转fallback、逐帧矩形扫掠和最终DONE。保留未来塔吊WAIT完成门专项，错误/过期/取消后完成不能续跑。不打开串口。
`python Tests/hardware/test_trajectory_bridge.py`检查真实USART1轨迹桥接的取消竞态/OPS快照/心跳与四向世界速度换算。配套`test_debug_rx_recovery.py`增加异常取消和超长命令整行丢弃。均为离线/HAL替身，不证明真实电机和总线时序已经验证。

DM内置梯形参数：`python Tests/hardware/test_dm_registers.py`（需要GCC）。
编译真实DM驱动和调试桥接，验证CAN读写/显式回读、错误节点/RID、失能和反馈新鲜度门禁、
邮箱忙/发送失败/超时、部分写入不一致、非法浮点、STOP/失联取消、队列满、PRIMASK恢复、tick回绕。
不连接硬件，不证明电机固件支持或实际梯形曲线效果。

X固件28/35步进通信：`python Tests/hardware/test_stepper_x_can.py`（需要GCC）。
编译真实步进驱动及调试解析/调度函数，验证FD双包、RPM缩放、角度边界、状态/使能报文、
邮箱不足重试、匹配回复与超时、事件队列及中断状态恢复。旧`test_debug_stepper.py`入口转调此测试。
该测试使用HAL模拟，不代表实物CAN接线和运动已经验证。

Flash参数保存：`test_spi_flash.py` 编译真实SPI驱动验证命令、ID、页边界、忙位和超时；
`test_param_store.py` 编译真实存储状态机，模拟逐字节掉电、擦除中断、扇区轮换、CRC回退及故障；
`test_param_integration.py` 编译真实调试桥接函数，验证16字段映射、恢复和临界区边界。
均为主机模拟，未连接实际Flash。

OPS 协议回放专项：`python Tests/hardware/test_ops_protocol.py`。
提取真实 `Hardware/ops.c` 的 V1/V2 解析函数，以合成字节流验证 V2 flags、
CRC16、拆包、噪声后重同步、CRC 失败恢复、V1 兼容和 session_id 变化。
该测试不连接 USART2，也不代表真实 DMA/中断时序已实测。

USART1接收恢复专项：`python Tests/hardware/test_debug_rx_recovery.py`。
提取实际恢复函数和接收回调，验证注册/启动/重启失败重试、DMA异步终止等待、
半条命令丢弃、不刷新心跳、TX忙时隔离，以及重启后新错误不被覆盖。
该测试为HAL模拟，不代表实机噪声、DMA寄存器时序已验证。

坐标链路专项：`python Tests/hardware/test_coordinate_chain.py`。
不编译固件，直接读取 `Hardware/ops.c`、`mecanum_control.c`、`debug_usart.c/.h` 的源码文本，
核对全工程统一坐标：`+X=车左、+Y=车头、+Z=逆时针`，24 通道遥测 ch0/ch1、ch3/ch4、ch6/ch7 直接输出；
位置/误差由内部 mm 在打包边界转换为 cm（1 位小数）；
`MANUAL`/`GOTO`/`OPSOFFSET` 三个入参直接使用 X/Y；`KPX` 写 `mKpx`、`KPY` 写 `mKpy`；
以及 OPS 原始帧通过固定映射 `X=-raw_x、Y=raw_y` 统一转换、
置零物理正向和 `chassis_move` 的
`目标 - 当前` 误差（两者必须成对，否则位置环变成正反馈）。同时断言 24 通道 JustFloat 帧格式、
麦轮解算公式和既有命令未被本次改动波及。
沿用 `test_debug_wheel.py` 的文本核对方式，不编译整条解析链；`MANUAL` 超时与原序输出由
`test_debug_manual.py` 编译真实服务函数覆盖，`OPSOFFSET` 的坐标约定由 `test_ops_offset.py`
覆盖，Qt 侧轴序由 `python -m unittest -v test_debugger` 的
`test_telemetry_direction_matches_field_axes` 覆盖。源码文本断言不代表实机运动方向已实测。

OPS 原始轴方向专项：`python Tests/hardware/test_ops_axis_direction.py`（需要 GCC）。
编译真实映射函数、坐标清零、位置换算和
`OPS_SetOrigin()`，验证固定映射的前向/反向换算、绝对输出、安装偏心补偿以及
向前只增加 Y、向左只增加 X。该测试为合成坐标，不代表实车安装方向已实测。

OPS ZERO 坐标系专项：`python Tests/hardware/test_ops_zero_frame.py`（需要 GCC）。
提取真实映射、偏心补偿、`OPS_ZeroCoordinates()` 和 `OPS_CopyPosition()` 编译运行，
覆盖 ZERO 时航向 0°/+30°/−30°/+90°、非原点 ZERO、ZERO 后左传、
原地旋转偏心补偿及映射可逆性。该测试不连接设备。

解析层轴序专项：`python Tests/hardware/test_parse_line_axes.py`（需要 GCC）。
抽取真实 `Debug_ParseLine`、解析辅助函数和 `s_params` 参数表，用桩替身提供 HAL 与状态变量后编译运行，
在运行时断言：`GOTO=X(左右),Y(前后)` 的 cm 输入原序落到 `s_goto_x`=X / `s_goto_y`=Y（含两轴 ±300.0cm 钳位与 cm→mm 取整、
省略 Z 时保持当前航向、少于两个数时不接受新目标）；`OPSOFFSET=X(左右),Y(前后)` 落到
`s_offset_x`=X / `s_offset_y`=Y（`OPSOFFSET=60,-50` 即车左60mm、车后50mm，越界仍拒绝）；
`KPX` 写 `mKpx`、`KPY` 写 `mKpy`（超限钳位到 50）；`MANUAL` 按协议原序暂存并原样传给
`MoveVelocity`（由 `test_debug_manual.py` 覆盖）；
以及 `WHEELOFF` 失能后解析层仍丢弃 `MANUAL`/`GOTO`。该测试不连接设备，也不验证机械运动方向。

视觉协议回放专项：`python Tests/hardware/test_vision_rx.py`（需要 GCC）。
编译真实 `Hardware/vision.c` 与 `vision_track.c`，用 HAL 替身回放 V1.1 响应帧，覆盖：
CRC-8 变体（规范§22 的 5 个标准向量 + 《V1.1 请求帧速查》15 条请求帧实测 CRC，含"黄/黑 CRC 碰撞"）、
6 字节请求组帧与 SEQ 管理（新任务换序号、同一逻辑任务重发复用同一序号、HAL 忙时保留待发）、
16 字节响应解析与滑动重同步（帧前噪声、载荷内嵌 `0x66`/`0x77`、CRC 错、帧尾错、字节间隔超时）、
`STATUS=0x06 INVALID_DATA` 与 `payload[0]` 错误详情、队列深度、接收错误重挂、
批量任务 INDEX 位图去重与 COMPLETE 完整性；以及应用层的两种 `COORD_MODE` 增益、
`VALID_FLAGS` 逐轴放行、150ms 时效和 SEQ 过滤。
该测试不连接 USART3，**不代表与 Jetson 的真实联调已验证**；两端 CRC 变体一致是联调前提。

视觉调度仲裁专项：`python Tests/hardware/test_vision_control.py`（需要 GCC）。
从 `debug_usart.c` 抠出 `Debug_ServiceVision` 编译，验证启停、四轮闸门、停车去重与 ACK 上报。
它与 `test_parse_line_axes.py`（`VTRACK=` 语法落点）、`test_debug_wheel.py`（VTRACK 应答文本下标）
共同覆盖视觉命令链路，都不涉及帧字节，因此协议版本变化（V1.0→V1.1）不影响这三个套件。

使用主机 GCC 直接编译 `hcan.c`、`stepper_2835.c`、`OLED_SoftSPI.c`，以本目录 HAL 替身代替寄存器访问，不连接设备。测试头文件只能用于本测试，禁止加入固件包含路径。

在工程根目录执行（输出目录为已有 EIDE 构建目录）：

```powershell
gcc -std=c99 -Wall -Wextra -Wno-sign-compare -ITests/hardware -IHardware Tests/hardware/regression.c Hardware/hcan.c Hardware/stepper_2835.c Hardware/OLED_SoftSPI.c -o MDK-ARM/build/STM32F407VET6_ILHC/hardware_regression.exe
if ($LASTEXITCODE -eq 0) { & .\MDK-ARM\build\STM32F407VET6_ILHC\hardware_regression.exe }
```

覆盖位置指令的实际字节和扩展 ID、四字节回零 DLC/补零、邮箱不足不发送、CAN 未启动、临界区状态恢复、分包 ID 越界、速度换算越界、回复快照、1像素高位图、12像素字符不改写下方行、零半径圆、画线端点和非法绘制参数。

此测试不包含 `debug_usart.c` 的 RTOS 服务，不模拟真实 SPI 时序、CAN 仲裁或电机动作。USART1 DMA 所有权修复由源码检查与固件编译验证；设备通信仍需实机验证。
