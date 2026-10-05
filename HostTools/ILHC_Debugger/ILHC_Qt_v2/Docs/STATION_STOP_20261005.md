# v2.1.6：取消短暂无进展自动终止，修复停稳误判

用户确认已烧录v2.1.5配套固件，随后截图显示在原料区附近、进度7818mm、位置约(220,1050)mm、航向269.5°时，因3秒无进展取消整轮。按用户随后要求，新版关键坐标运行不再因3秒无进展触发FAULT14；改为报告目标误差并继续纠偏，不自动跳过站点。

此次还复现了另一项独立缺陷：物理位置不动，仅为OPS每个20ms样本加入±0.03mm、±0.03°抖动，v2.1.5真实C执行器就无法完成起点STOP，最终FAULT14。此前整数RPM修复不能消除此问题。没有该次实机原始流，不能断言截图的实际噪声恰好等于这个测试值。

旧判断对20ms位置差分求速度，要求每次都低于1mm/s、1deg/s；这相当于位置每帧变化不得超过0.02mm。定位抖动容易反复清零200ms停稳计时。旧终点补角还将小角度误差乘50，例如0.2°误差会直接请求10deg/s，容易在停靠时反复转动。

现在的处理如下：

- 关键坐标3秒无进展只发`CSTALL`提示，并持续调用闭环；恢复运动后自动清除提示。PC模拟运行保持相同策略。
- 在原有严格停车带（位置≤0.5mm、航向≤0.3°）内清零平移与旋转输出，取消乘50的末端补角。实际姿态继续使用OPS，不把显示或模型坐标改成目标值。
- 连续200ms检查整个新鲜OPS窗口的范围，平面范围≤0.2mm、角度范围≤0.2°。检查所有中间样本，而非仅比较首尾，避免漏掉来回振动。原有完成位置<1mm、航向<1°及请求速度≤1的门限保留。
- 真正到位停稳后才切下一点；PASS保持运行。用户STOP、失能、OPS失效/跳变、主机失联、偏离骨架和180秒总运行上限仍有效。

`CSTALL id target gap10 yaw100 measured_v10 measured_w100 command_v10 command_w100 settled_ms`包含目标点序号、位置/航向误差、测得及请求速度、停稳时间。上位机状态显示“暂时无进展，继续纠偏”，导出的实机批次在`stall_diagnostics`保存最近32条诊断。提示不会发STOP或TABORT，也不会伪造DONE。

预演仍拒绝无法生成安全可执行路线的控制候选；实际运行取消的是3秒无进展自动终止。现有七项底盘RAM参数、整车扫掠、上传CRC与参数快照不改变。

验证日志：

最终6组核心自检及309项PC无GUI通过（235.391s）、53项真实Qt地图/实机入口通过（392.046s）、12项C坐标整组通过（224.607s）、16项原轨迹C兼容通过（66.646s），所有进程退出0。C诊断重试/旧发送ACK不覆盖新诊断、扩充位置/角度真实漂移另行通过。八轮真实C加整数电机完整路线全部DONE，19～34目标，整车矩形/相邻扫掠、全部站点、PASS时运动和零TRESUME通过；模型用时47.74/48.26、54.62/52.96、67.26/67.62、79.60/82.08秒，不是实车计时。

- 修复前噪声复现：`RTOS_APP/validation/station-stop-repro-before-20261005.log`。
- 噪声、真实漂移、3秒告警后恢复、STOP与总超时：`RTOS_APP/validation/station-stop-stall-targeted-20261005.log`。
- 真实C执行器及整数电机完整路线：`RTOS_APP/validation/station-stop-coordinate-c-final-20261005.log`。
- 诊断重试与位置/角度漂移扩充：`RTOS_APP/validation/station-stop-diagnostic-targeted-final-20261005.log`。
- 原轨迹C兼容：`RTOS_APP/validation/station-stop-legacy-c-final-20261005.log`。
- PC无GUI：`Docs/station_stop_headless_final_20261005.log`。
- Qt地图及实机入口：`Docs/station_stop_qt_final_20261005.log`。
- 构建：`RTOS_APP/validation/station-stop-build-20261005/MDK-ARM-STM32F407VET6_ILHC-build.log`。

EIDE工程`MDK-ARM`、配置`STM32F407VET6_ILHC`构建0错误0警告，Flash75820B/RAM97024B。HEX为`MDK-ARM/build/STM32F407VET6_ILHC/STM32F407VET6_ILHC.hex`，PC版本v2.1.6。需要重新烧录此次固件并重启上位机；本次未连接串口、未烧录或驱动实车。
