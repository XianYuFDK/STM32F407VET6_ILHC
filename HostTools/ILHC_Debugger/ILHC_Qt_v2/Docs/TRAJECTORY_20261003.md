# 连续Trajectory生成与导出（2026-10-03）

本文件记录生成阶段。后续已开放PC模拟器连续跟踪，见[连续跟踪说明](CONTINUOUS_TRACKING_20261003.md)；STM32执行仍未接入。

新增 `trajectory.py`：将相连且切线连续的 `LineSegment` / `ArcSegment`（或平滑后的 `LINE/ARC` 字典）转换成20mm采样的连续几何Trajectory。只生成、复检、显示和导出，没有STM32接口、协议或运动执行接入。

## 五字段与坐标

每个采样点只包含以下五个字段，JSON可直接序列化：

| 字段 | 含义 |
| --- | --- |
| `x_mm` | 场地+X坐标，mm，沿用界面场地坐标系 |
| `y_mm` | 场地+Y坐标，mm |
| `field_yaw_deg` | 场地坐标中的运动切线航向，+X为0°、+Y为90°，已经unwrap |
| `s_mm` | 从轨迹起点累计的真实几何弧长，mm |
| `segment_type` | `LINE`或`ARC`；切点归属后段，最终端点归属最后一段 |

这里的场地坐标固定原点在启停区1，+X为屏幕左、+Y为屏幕上，与 `core.layout_to_field` 一致。原规划和碰撞场景仍使用 `LAYOUT_MM`，两者不得混用：

```text
field_x = 2250 - layout_y
field_y = 2250 - layout_x
field_direction = (-layout_dy, -layout_dx)
field_yaw = -90 - layout_tangent_yaw  （再unwrap）
```

Trajectory航向直接由几何切线计算，**不是原台账中的车头航向，也不是OPS的Z角**。原来的前进/后退/横移语义与初末车头航向约束仍保存在 `steps/segments/arcs`，不拿Trajectory替换原动作或直接作为GOTO命令。

## 采样与角度连续

- 全程统一使用 `s=0,20,40,60…` 的采样网格，不在每个新段重新计数。
- 额外包含每个直线/圆弧连接点和最后终点，所以相邻点的弧长间距始终≤20mm；短末段保留真实端点。
- 直线位置按真实线段插值，Yaw为 `atan2(dy,dx)`；圆弧位置按真实半径和扫角计算，Yaw为圆弧的有向切线方向，沿弧连续变化。
- `s_mm`的圆弧部分使用 `radius*abs(sweep_rad)`，不使用采样折线的弦长近似。
- 切点只有一行，连续性同时检查位置和有向切线；允许相邻圆弧直接相接。
- `unwrap_degrees`先把首角规范到 `[-180,180)`，然后累计最短角差：`179,-179,-178`变成`179,181,182`。多圈路线可超过360°，不会在导出时重新取模。
- 保留原地转向的fallback、模式切换产生的真实直角、180°折返不满足切线连续性，明确 `FALLBACK_REQUIRED`，不发布假连续Trajectory；原A*与圆弧fallback信息仍保留供查看。

## 最终完整车体复检

生成全部五字段样本后，重新用其切线航向（转换回布局坐标）检查完整矩形车体及 `pad`，不复用原车头航向的安全结论：

1. 每个最终样本的完整车体必须被drivable白名单覆盖，且不接触固定/模拟/动态障碍。
2. 每个完整直线段用两个姿态矩形的凸包检查连续扫掠，覆盖20mm采样点之间的小障碍。
3. 圆弧独立按≤20mm且≤3°重新细分，检查每个姿态；每对相邻姿态用凸包加最大车体顶点弦高外扩，覆盖采样间隙。
4. 校验样本的位置、切线、类型、累计弧长、严格递增及首尾与原几何一致；角度被重新取模或样本被篡改会失败。

圆弧扫掠包络与原圆弧后处理共用 `CollisionScene.arc_interval_reason`，不因最终输出间隔较大而丢失碰撞。弦高外扩是保守检查，可能拒绝紧贴边界的实际可行轨迹。

任何一步失败均清空 `trajectory` 并给出状态/原因，`trajectory_safe=False`。缺少真实车体、非法输入、时限/取消与超过100000点都有明确结果。场景含最新障碍快照；地图和障碍更新仍作废旧计划。

## 返回值与接口

规划成功后，在现有安全圆弧后处理基础上自动生成Trajectory，原 `points/steps/segments/corners/search_cost`不变。新增字段：

```text
trajectory                     五字段点列；失败时[]
trajectory_status              READY / FALLBACK_REQUIRED / UNSAFE / DISCONNECTED /
                               INVALID_INPUT / NO_FOOTPRINT / EMPTY / CANCELLED /
                               RESOURCE_LIMIT / UNAVAILABLE / DISABLED / NOT_RUN
trajectory_safe                全轨迹完整车体复检通过
trajectory_continuous          位置和切线连续且通过最终复检
trajectory_reason              明确失败或fallback说明
trajectory_length_mm           精确累计长度
trajectory_spacing_mm          默认20
trajectory_frame_id            FIELD_MM
trajectory_yaw_convention      场地切线角、unwrap说明
```

`core.generate_trajectory(primitives, scene)`也可独立使用；`core.validate_trajectory(samples, primitives, scene)`用指定场景重新复查，返回 `{ok, code, reason}`；`core.export_trajectory_json(path, result, metadata=...)`只导出READY且安全的完整Trajectory。

```python
import core

result = core.plan_path(
    (500, 500), (1500, 1500), grid=100, pad=10,
    rects=[], circles=[], bounds=(0, 0, 2400, 2400),
    footprint=(280, 260, 0), goal_heading_deg=90)
assert result['trajectory_safe']
core.export_trajectory_json('ilhc-trajectory.json', result)
```

`smooth_arcs=False`时，规划结果的Trajectory标记DISABLED；原安全平滑几何未成功时标记UNAVAILABLE。纯原地转向或原地单点没有运动切线，不伪造LINE或ARC。

## 地图与JSON

地图新增方向箭头，使用Trajectory的 `field_yaw_deg` 和统一场地/布局矢量变换，再处理Qt的y轴翻转，避免镜像或跨180°后箭头方向画反。直线约每100mm、圆弧约每40mm显示一支，保留类型切换处及终点，避免20mm样本箭头挤在一起。只有通过完整复检的Trajectory显示箭头；清除路径、失败、动态障碍更新时一并清除。

现有“导出计划”JSON包含Trajectory和失败状态；新增“导出Trajectory JSON”按钮导出独立五字段点列。后者在写文件前用当前场景再次复检，拒绝过期、碰撞或被改动的样本。文件包含坐标系、采样间隔、精确长度、角度约定、`hardware_ready:false`，以及界面导出时的地图版本和碰撞场景快照（含模拟/动态障碍）。

未添加Trajectory执行器，未接STM32，固件、协议与地图数据未改。现有固定航向仿真执行沿用原动作，不使用新的切线航向或密集采样点。

## 验证

新增20项专项回归，覆盖20mm全局网格/切点/短终点、真实弧长、左右圆弧所有半径、179→−179及多圈unwrap、相邻圆弧拼接、需要停转fallback、切线姿态与原动作姿态不同导致的碰撞、采样间微小障碍、孔洞、动态场景复检、篡改、取消、资源限制及两种JSON导出。

新增真实离屏Qt圆弧箭头方向检查，扩展原圆弧预览测试以验证独立导出按钮、实际图元、清除箭头和不发送运动命令。

完整命令：`python run_tests.py --qt`。6组核心自检、166项无GUI测试、102项离屏Qt测试全部通过。
回归日志：`trajectory_regression_20261003.log`。
