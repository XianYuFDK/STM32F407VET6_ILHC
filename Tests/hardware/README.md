# 驱动主机回归测试

USART1接收恢复专项：`python Tests/hardware/test_debug_rx_recovery.py`。
提取实际恢复函数和接收回调，验证注册/启动/重启失败重试、DMA异步终止等待、
半条命令丢弃、不刷新心跳、TX忙时隔离，以及重启后新错误不被覆盖。
该测试为HAL模拟，不代表实机噪声、DMA寄存器时序已验证。

使用主机 GCC 直接编译 `hcan.c`、`stepper_2835.c`、`OLED_SoftSPI.c`，以本目录 HAL 替身代替寄存器访问，不连接设备。测试头文件只能用于本测试，禁止加入固件包含路径。

在工程根目录执行（输出目录为已有 EIDE 构建目录）：

```powershell
gcc -std=c99 -Wall -Wextra -Wno-sign-compare -ITests/hardware -IHardware Tests/hardware/regression.c Hardware/hcan.c Hardware/stepper_2835.c Hardware/OLED_SoftSPI.c -o MDK-ARM/build/STM32F407VET6_ILHC/hardware_regression.exe
if ($LASTEXITCODE -eq 0) { & .\MDK-ARM\build\STM32F407VET6_ILHC\hardware_regression.exe }
```

覆盖位置指令的实际字节和扩展 ID、四字节回零 DLC/补零、邮箱不足不发送、CAN 未启动、临界区状态恢复、分包 ID 越界、速度换算越界、回复快照、1像素高位图、12像素字符不改写下方行、零半径圆、画线端点和非法绘制参数。

此测试不包含 `debug_usart.c` 的 RTOS 服务，不模拟真实 SPI 时序、CAN 仲裁或电机动作。USART1 DMA 所有权修复由源码检查与固件编译验证；设备通信仍需实机验证。
