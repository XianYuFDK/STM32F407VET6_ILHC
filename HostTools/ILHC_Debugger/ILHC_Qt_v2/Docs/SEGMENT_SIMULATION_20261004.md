# PC直线端点与圆弧执行模拟

2026-10-04，按用户要求先修改上位机并检查模拟效果。本轮未修改STM32固件、实机点表协议，未连接串口、烧录或驱动车辆。此前Git检查点仍为本地`f38e7cb`，本轮改动保留在工作区。

## 运行方式

重启`main.py`，点击“模拟”，加载`competition_map.json`，在“比赛地图”选择启停区及任务码，点击“一键比赛模拟”。普通地图点击路径的“执行规划路径”也采用新段执行入口。地图继续显示骨架、平滑路径、参考点、实际轨迹和车头；比赛路径的标记点改为直线端点/圆弧出口。

右侧规划信息显示段数。“导出端点/圆弧 JSON”保存当前单条点击路径的几何程序，导出前重新检查当前地图及障碍；它不是STM32上传格式。

## 算法与格式

`segment_route.py`生成`PC_SEGMENT_PROGRAM`。同方向且车头策略一致的共线直线合并，执行器只保存几何段及累计长度：

- LINE：起点、终点，可选车头策略。
- ARC：入口、出口、圆心、半径、起始角、转角、CW/CCW，可选车头策略。
- 整条行驶程序终点STOP；完整比赛每站单独行驶程序，站点停稳后自动进入作业模拟/下一段。

`SegmentTracker`按实际车位置投影更新单调进度，使用100mm前视实时求参考X/Y/车头角。直线参考沿线推进；圆弧参考按圆心/半径解析计算，位置与航向同时变化。保留麦轮固定车头、切线前进、切线倒退策略以及角度unwrap。中间端点不要求停车，站点和最终STOP仍要求位置、航向及连续10帧停稳。无法安全平滑的弯角仍明确fallback，由原停转流程处理，不把不连续几何强连成圆弧。

原`TrajectoryTracker(samples, primitives)`兼容入口保留，与新执行器共用几何控制核心。新PC点击及比赛行驶使用`Simulator.submit_navigation_segments`，接受时不要求点表。密集采样仍用于地图箭头、JSON旧格式及完整碰撞复检；本轮不是移除安全采样，也不是减少实机协议点数。

坐标约定：`frame_id=LAYOUT_MM`用于segments的起终点、圆心、圆弧角和FIXED车头角。程序`start`/`goal`是执行参考对象，X/Y来自`layout_to_field`，单位mm；其中`field_yaw_deg`为内部数学角，场地+X为0°，GUI车头角按`90-field_yaw_deg`显示，仍是朝场地+Y为0°、朝+X为90°。执行接受时从几何重建，不以导出的start/goal/length作为安全证明。`hardware_ready=false`明确表示PC格式。

## 安全与比较范围

入场重新做完整矩形车体+裕量检查，完整线段扫掠，圆弧按≤20mm弧长且≤3°角步长检查姿态及相邻包络；运行逐帧检查实际车体及扫掠。STOP、地图版本/模拟会话变化、失能及异常继续取消。检查阶段使用取消谓词，不长时间持有模拟状态锁。

PC仍采用原250mm/s、120°/s模型，前馈及坐标纠偏控制律不变，以隔离执行格式变化的影响。已有模拟跟踪器原本就解析几何；这次改变的是执行入口、运行存储及显示/导出，不能把点表减少解释为同速比赛自动提速。实机500mm/s配置不在本轮范围。

## 模拟结果

两个启停区分别运行无障碍、单障碍、四障碍测试夹具及截图两障碍，共8轮，全部COMPLETE，12次抓取/12次放置模拟及返回完成，每帧完整车体扫掠通过。验证脚本在执行前删除所有TRAVEL的trajectory和smoothed_primitives，证明运行不依赖它们。四障碍仅作为离线夹具，未恢复GUI固定预设。

|场景|启停区1/2段数|显示复检样本数|启停区1/2用时(s)|
|---|---|---|---|
|无障碍|20 / 20|679 / 679|89.96 / 89.94|
|单障碍|20 / 20|679 / 679|89.96 / 87.66|
|四障碍|28 / 28|835 / 835|103.04 / 100.74|
|两障碍|36 / 38|1199 / 1282|126.44 / 132.38|

二维码→原料区仅3段（直线→圆弧→直线），8.44s，弯道持续平移并改变车头方向。这一示例采用切线倒退，车头与运动切线可以相反。

- [转弯动画](segment_route_qr_raw_20261004.gif)
- [完整比赛路径及实际轨迹](segment_route_match_preview_20261004.png)
- [实际Qt界面离屏预览](segment_route_qt_preview_20261004.png)
- [二维码→原料区几何JSON](segment_route_qr_raw_20261004.json)
- [8轮结果](segment_route_results_20261004.json)
- 复现：`python Docs/verify_segment_route_20261004.py`

## 验证记录

新增8项专项：无点表执行、旧新控制结果一致、共线合并/JSON、圆弧平移转头、全矩形障碍拒绝、非法连接与fallback拒绝、STOP/地图取消、接受复检期间取消、删除比赛所有行驶点表后完整运行。

6组核心自检、257项无GUI通过，记录`segment_route_regression_final_20261004.log`。该次Qt174项中的一项在同时生成离线回放/图片时因“模拟定位过期”取消，保留原记录；该测试使用无后台供帧的Simulator，同步供帧后执行阶段复检，重负载时可能触发现有定位保护。未放宽保护门限。单独复现通过，记录`segment_route_qt_isolated_20261004.log`。

全部Qt单独复跑时174项断言均通过，但同一进程在解释器退出阶段发生原生异常，退出码-1073740791，不能记为完整进程成功，记录`segment_route_qt_final_20261004.log`。未确定原生异常根因。随后六模块各自独立进程运行，174项均通过且全部退出码0：

|模块|项数|日志（Docs目录）|退出码|
|---|---:|---|---:|
|test_debugger|90|segment_route_qt_test_debugger_20261004.log|0|
|test_serial_lifecycle|9|segment_route_qt_test_serial_lifecycle_20261004.log|0|
|test_map_click_qt|24|segment_route_qt_test_map_click_qt_20261004.log|0|
|tests.test_dm_position|22|segment_route_qt_tests_test_dm_position_20261004.log|0|
|tests.test_hardware_trajectory_qt|19|segment_route_qt_tests_test_hardware_trajectory_qt_20261004.log|0|
|tests.test_trajectory_settings_qt|10|segment_route_qt_tests_test_trajectory_settings_qt_20261004.log|0|

模拟通过不证明实车侧滑、OPS偏差或固件FAULT13已经解决。本轮实机仍使用原缓存点表协议。
