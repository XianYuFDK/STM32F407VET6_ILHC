# RTOS_APP 应用任务层

`Core/Src/freertos.c` 的 `MX_FREERTOS_Init()` 调用 `RTOS_APP_Init()` 创建应用任务。任务循环、调度周期、互斥锁和接收队列统一放在本目录；硬件协议与控制状态机仍保留在 `Hardware/`。

## 任务职责

| 任务 | 文件 | 优先级 | 周期 | 栈字节数 | 职责 |
| --- | --- | --- | --- | --- | --- |
| chassis | app_control.c | High | 20ms | 2048 | 底盘安全、轨迹、GOTO、视觉、手动控制及轮速输出 |
| comm | app_comm.c | AboveNormal | 通知唤醒，遥测20ms | 3072 | USART1/OPS接收解析、RX/TX恢复、应答和遥测 |
| mechanism | app_mechanism.c | Normal | 20ms | 2048 | DM电机与步进电机状态机 |
| maintenance | app_maintenance.c | Low | 50ms | 1536 | 参数Flash服务、每秒采集栈和堆余量 |

保留内核IDLE任务，软件定时器任务仍关闭。栈尺寸在 `rtos_app.c` 的 `attributes` 中定义，动态分配来自现有15360字节FreeRTOS堆；不能把任务栈重复加到链接器RAM总量上。

## 调度和状态保护

- 应用共享状态由启用优先级继承的 `app_state` 互斥锁保护。任务调用 `DebugUsart_*Service` 时按当前入口约定持锁；ISR不获取此锁。
- 正常底盘控制周期为20ms。超期跳过旧周期，避免连续补跑轨迹/PID；STOP提前唤醒只执行取消和停车，不增加一次轨迹控制积分。
- 运行期轮速由 `chassis` 唯一提交，UART4底层队列和TIM7发送推进沿用原驱动。新增任务不得成为第二个轮速发送者。
- USART1 TX由 `comm` 独占；DMA忙时不覆盖在途发送缓冲。TX恢复在应用锁外执行。
- Flash服务在应用锁外执行；只在短暂的参数快照和保存结果入队时持锁。

## 接收与故障

USART1和OPS USART2接收ISR复制DMA数据、重启接收并向 `app_rx.c` 固定队列提交。完整文本/OPS解析移到通信任务。两路各8槽，环形队列可用7槽，每包上限256字节。每包保留接收时间戳和队列epoch，解析延迟不刷新数据的新鲜程度。

通信任务每轮最多处理4次接收服务；积压未清时至少让出1个tick。队列满/超长或接收恢复失败走取消/停车或定位失效路径，丢弃故障队列中的旧包，避免旧数据在恢复后继续执行。

STOP、WHEELOFF、ZERO、DMOFF使用独立快速锁存位。普通解析不能覆盖这些安全请求；中断通知只在内核运行后调用CMSIS接口。通知入口可由任务和优先级数值不小于5的外设中断调用。

## 调试统计

可在调试器中查看：

- `app_task_stats[APP_CONTROL/APP_COMM/APP_MECHANISM/APP_MAINTENANCE]`：`runs`、`overruns`、`max_cycles`、`last_tick`、`stack_free_bytes`。
- `app_heap_free_bytes`、`app_heap_min_bytes`：当前和运行以来最小堆余量，每秒更新。
- `app_rx_overflows[APP_RX_HOST/APP_RX_OPS]`：接收队列溢出计数。

DWT的 `max_cycles` 是服务调用开始到返回的经过周期，含抢占/等待；不是纯CPU占用率。栈余量来自运行时高水位，不是未上板即可确认的实测数据。栈溢出钩子触发系统复位。

## 工程接入

Keil `MDK-ARM/STM32F407VET6_ILHC.uvprojx` 与EIDE `MDK-ARM/.eide/eide.yml` 均需包含六个 `.c` 文件及 `../RTOS_APP` include路径。CubeMX自定义入口位于 `freertos.c` 的USER CODE区域；`.ioc`不再声明旧 `defaultTask`。重新生成后检查任务声明、任务入口、include与中断差异。

## 2026-10-04 离线验证

- `Tests/hardware/test_rtos_app.py`：真实任务源码的任务归属、初始化失败、启动前ISR、队列故障、STOP唤醒、周期超期/回绕、RX预算、Flash解锁和1Hz统计测试通过。
- `Tests/hardware/test_*.py`：30个测试脚本全部退出码0，包含12项真实C轨迹回放测试以及OPS/USART1恢复、轮控/机构故障专项。
- Qt上位机 `run_tests.py`：6组核心自检及239项无GUI测试通过。本轮未运行Qt GUI验收。
- 最新Keil构建：Target `STM32F407VET6_ILHC`，0 errors/0 warnings；Code=56924，RO-data=11284，RW-data=696，ZI-data=96008。Flash=68904字节，RAM=96704字节，主SRAM128KiB剩余34368字节。
- 构建日志：`validation/STM32F407VET6_ILHC-STM32F407VET6_ILHC-build.log`。HEX：`../MDK-ARM/STM32F407VET6_ILHC/STM32F407VET6_ILHC.hex`。

以上为主机静态测试、真实C离线回放和编译结果；未烧录、未连接实车，未验证真实调度延迟、电机执行或上板栈/堆高水位。
