# 天空星 SPI Flash 调试参数保存

2026-09-21：记录版本升级为v2，首次加载v1记录时仅将DM地址改为3，其他参数保留，
随后自动提交新记录。已保存的v2记录仍保留用户显式设置的DMID。

当前实现用于 STM32F407VET6 主控板上的 W25Q128，OPS 板固件不需修改。

| 主控引脚 | Flash 信号 | 配置 |
|---|---|---|
| PA4 | /CS | GPIO 输出，默认高，上拉 |
| PA5 | CLK | SPI1 SCK，AF5 |
| PA6 | DO | SPI1 MISO，AF5 |
| PA7 | DI | SPI1 MOSI，AF5 |

SPI Mode 0、8 bit、MSB first；APB2=84MHz，16分频得到5.25MHz。
按 JEDEC ID 接受 `EF4018` / `EF7018`，未知型号或通信异常时禁用保存、继续正常调试。
引脚依据：[嘉立创 SPI-FLASH 应用](https://wiki.lckfb.com/zh-hans/tkx/tkx-stm32f407vxt6/beginner/spi.html)。
器件参考：[Winbond W25Q128JV 文档](https://www.winbond.com/hq/support/documentation/?__locale=en&pno=W25Q128JV)。

## 使用

仍按原来的串口命令或 Qt 在线调参，不需要额外保存命令。
参数稳定 **2秒** 后自动写入，正常约2.1秒，扇区轮换需要额外擦除和逐页校验时间。
看到 `ACK PARAM SAVED TO FLASH` 才表示这次参数已写入并回读校验成功。
**收到此消息前立即断电，可能只恢复上一份已保存参数。**

保存的16个数值：

- 底盘：`KPX / KPY / KPZ / XVMAX / ZVMAX / XVMIN / ZVMIN`。
- OPS：`OPSOFFSET` 的 X/Y，单位mm，+X左、+Y前。
- DM：`DMID / DMMODE / DMPOS / DMVEL / DMKP / DMKD / DMTOR`。

重启在串口接收开启前恢复参数；不恢复 GOTO、MANUAL、使能请求、步进动作和 ZERO 原点。
DM上电仍保持调试禁用状态，后续显式DMEN时使用已恢复的DM调试数值。
底盘原有启动停车/锁轴行为保持不变。电脑上的参数文件和界面输入框不是 Flash 内容回读；
连接不会自动下发OPS偏移，但主动点击应用会覆盖设备参数。

启动/保存消息通过原有USART1事件队列输出，不改变24通道遥测：

| 消息 | 含义 |
|---|---|
| ACK PARAM LOADED FROM FLASH | 启动已恢复有效记录 |
| INFO PARAM DEFAULTS; NO VALID FLASH RECORD | 空白/无兼容有效记录，使用固件默认值 |
| ACK PARAM SAVED TO FLASH | 本次参数快照已提交且回读校验通过 |
| ERR PARAM FLASH; UNSAVED CHANGES LOST ON POWER OFF | 初始化或保存失败，本次启动停止继续写入 |

事件队列拥塞时文字可能丢弃；保存过程中继续改参会在下一轮保存最新值，验收时应停止调参再等保存应答。
故障时RAM调参仍可用，检查焊接/型号/写保护后重启再试；驱动不会自动解除芯片写保护。
恢复默认值可重新下发所需默认参数，再等自动保存。不提供整片擦除指令。

## 文件和存储布局

- `spi_flash.c/.h`：独立SPI1底层驱动，包含GPIO/SPI初始化、JEDEC读取、状态读取、页编程和4KB扇区擦除。
- `debug_param_store.c/.h`：独立参数记录与保存状态机，不依赖电机或串口模块。
- `debug_usart.c`：负责参数快照、启动恢复和应答；`ops.c`提供安装偏移读取接口。
- 启用HAL SPI，并同步EIDE、Keil源文件清单和`.ioc`引脚配置。

仅预留Flash最后8KB：`0xFFE000..0xFFFFFF`，这两个扇区不能再用于字库或其他数据。
其余区域不擦除。每扇区16个256B页；每页一条88B记录，包含magic、版本、序号、
64B参数、CRC32、提交标记。只有版本、参数范围、CRC及提交标记全部有效的记录才加载。
32位序号按回绕比较；修改字段布局或语义时必须升级版本。

追加写满一个扇区后，擦除另一个不包含最新有效记录的扇区。
正文先写入和回读校验，再单独写提交标记，最后再次读取确认。
断电发生在新记录提交前，上一条记录仍可恢复；第一次保存中断且无旧记录时使用默认值。
相同参数不重复写，连续调参合并保存，每16次记录追加通常才需要一次扇区擦除。

所有Flash访问仅在默认任务或启动阶段执行，ISR不擦写。
运行期每周期推进一个短步骤，不循环等待擦写忙位；SPI单次事务超时2ms，
芯片忙超过1秒报告故障。4KB擦除校验分16个周期，每周期读取一页。
SPI事务期间中断保持可用，只有参数快照复制使用短临界区。

`.ioc`已记录SPI1和PA4~PA7；当前SPI初始化由`SPIFlash_Init`拥有。
若CubeMX重新生成`MX_SPI1_Init`，需检查初始化归属，避免生成代码改动模式、分频或片选。

## 验证与实机验收

主机测试（无硬件）：

```
python Tests/hardware/test_spi_flash.py
python Tests/hardware/test_param_store.py
python Tests/hardware/test_param_integration.py
```

分别覆盖真实驱动命令/页边界/故障、真实存储状态机的逐字节掉电与扇区轮换、
真实调试桥接的字段恢复与中断边界；不能替代实机电气和断电测试。

烧录新F407固件后：

1. 连接调试串口，确认没有`ERR PARAM FLASH`。
2. 发`KPX=3.1`、`KPY=2.8`、`OPSOFFSET=60,-50`，停止改参，等待保存成功消息。
3. 断电重启，查看加载成功消息，遥测ch6/ch7应为3.1/2.8；确认未恢复旧运动动作。
4. 再次修改参数，在保存消息前断电，应恢复上一份完整记录；消息后断电应恢复新值。
5. 用调试器观察`spi_flash_jedec_id`确认型号；无Flash/通信故障时仍应能调参及使用原控制服务。
