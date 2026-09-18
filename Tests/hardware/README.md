# 驱动主机回归测试

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
以及 OPS 原始帧通过固定映射 `X=-raw_y、Y=-raw_x` 统一转换、
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

解析层轴序专项：`python Tests/hardware/test_parse_line_axes.py`（需要 GCC）。
抽取真实 `Debug_ParseLine`、解析辅助函数和 `s_params` 参数表，用桩替身提供 HAL 与状态变量后编译运行，
在运行时断言：`GOTO=X(左右),Y(前后)` 的 cm 输入原序落到 `s_goto_x`=X / `s_goto_y`=Y（含两轴 ±300.0cm 钳位与 cm→mm 取整、
省略 Z 时保持当前航向、少于两个数时不接受新目标）；`OPSOFFSET=X(左右),Y(前后)` 落到
`s_offset_x`=X / `s_offset_y`=Y（`OPSOFFSET=60,-50` 即车左60mm、车后50mm，越界仍拒绝）；
`KPX` 写 `mKpx`、`KPY` 写 `mKpy`（超限钳位到 50）；`MANUAL` 按协议原序暂存并原样传给
`MoveVelocity`（由 `test_debug_manual.py` 覆盖）；
以及 `WHEELOFF` 失能后解析层仍丢弃 `MANUAL`/`GOTO`。该测试不连接设备，也不验证机械运动方向。

使用主机 GCC 直接编译 `hcan.c`、`stepper_2835.c`、`OLED_SoftSPI.c`，以本目录 HAL 替身代替寄存器访问，不连接设备。测试头文件只能用于本测试，禁止加入固件包含路径。

在工程根目录执行（输出目录为已有 EIDE 构建目录）：

```powershell
gcc -std=c99 -Wall -Wextra -Wno-sign-compare -ITests/hardware -IHardware Tests/hardware/regression.c Hardware/hcan.c Hardware/stepper_2835.c Hardware/OLED_SoftSPI.c -o MDK-ARM/build/STM32F407VET6_ILHC/hardware_regression.exe
if ($LASTEXITCODE -eq 0) { & .\MDK-ARM\build\STM32F407VET6_ILHC\hardware_regression.exe }
```

覆盖位置指令的实际字节和扩展 ID、四字节回零 DLC/补零、邮箱不足不发送、CAN 未启动、临界区状态恢复、分包 ID 越界、速度换算越界、回复快照、1像素高位图、12像素字符不改写下方行、零半径圆、画线端点和非法绘制参数。

此测试不包含 `debug_usart.c` 的 RTOS 服务，不模拟真实 SPI 时序、CAN 仲裁或电机动作。USART1 DMA 所有权修复由源码检查与固件编译验证；设备通信仍需实机验证。
