# 修复说明

## 版本与范围

基于用户上传的 ILHC_Qt_v2.zip 修改，修复版本 v2.1.1。本包只改上位机，没有添加、烧录或修改 STM32 固件，没有修改GitHub远端，没有使用任何新固件指令。

原始输入和各文件摘要见 `manifest.json`。代码差异见 `changes.patch`。PNG预览图片沿用原包，**不是修复版实际截图**。

## 规划契约

`core.plan_path()` 保留原位置参数，新增关键字：

```
footprint=(length_mm, width_mm, layout_yaw_deg)
drivable_polygons=[...]
geometry_verified=False
cancel=threading.Event()
time_limit_s=8.0
```

所有几何使用LAYOUT_MM，不能传OPS或cm坐标。`footprint` 航向从布局+x轴起算；UI由现有“车头向量→布局向量”变换产生，不假定OPS航向与布局航向相同。

有 `footprint` 时 pad 是额外裕量，按矩形长宽各增加2×pad；无 `footprint` 时 pad 是旧兼容膨胀模型，采用保守正方形，默认150mm不能代表任意朝向整车。后者返回 `execution_safe=False`。

矩形障碍在车心配置空间使用精确凸Minkowski和；圆障碍用线段到反射矩形的距离与真实半径比较。整车在固定姿态沿直线运动的扫掠是首尾矩形顶点的凸包，白名单必须覆盖**整个**扫掠。白名单允许多区域合并、孔洞及非凸边界。障碍接触按碰撞；包含额外裕量的车体可与场地白名单边界相切（数值容差1e-7mm）。

有限栅格不是连续空间最优规划：首尾与8格半径邻域中所有无碰撞可见格点连接，完整计入接入距离，随后在增广图上A*。网格可能漏掉窄通道，要明确NO_PATH/NO_CONNECTION，不能减小车体外形来强行通过。

搜索用欧氏启发式、非负距离代价，跳过过时堆条目。找到离散路线后做连续碰撞检查的视线拉直，不宣称平滑后全局连续最优。没有曲率、速度轨迹或转向规划。

结果含 `ok/search_ok/model_safe/execution_safe/hardware_ready`。任何结果 `hardware_ready=False`；`execution_safe` 仅允许当前仿真门控使用。`geometry_verified` 只是加载地图的用户声明，不是系统自动认证。

## 执行与取消

没有给真实串口伪造ACK/DONE协议。仿真通过Simulator专用锁保护API接受航点并返回epoch/goal_id，完成后记录对应id。这些是PC内存中的仿真状态，不是STM32反馈。

模拟只固定当前航向，保持原坐标转换；调试GOTO依旧独立。固定航向规划会拒绝显式不同目标航向，避免一条校验时的矩形路径执行成转向轨迹。模拟导航速度250mm/s仅是运动学演示参数，不是电机RPM。

取消流程先撤销界面控制权，再在模拟器状态锁内清掉运动队列、当前GOTO/manual和会话；同一把锁包住模拟命令队列取出+应用。避免“STOP之后，之前取出的旧GOTO重新生效”。不会递归调用send_line(STOP)。

到位要求当前epoch/id完成、目标位置/角度相符、模拟器goto已清除，以及3个不同的新模拟帧确认。过期帧、超时、无进展、控制者变化、地图签名改变均取消。

异步规划采用一个ThreadPoolExecutor，主线程定时轮询Future。后台不操作Qt或串口。请求取消或签名过期后不接收旧结果。当前闭合窗口前取消规划、关闭池、停止模拟器。实际Qt退出流程仍需在用户有Qt环境的电脑验证。

## 地图与外形不能凭代码补齐

上传包没有可核实的灰色道路完整多边形，因此内置 `navigation_map.json` 保留旧外框与障碍，并明确 `geometry_verified=false`。没有把猜测的外侧车道宽度写成赛题事实，也没有为了让原料区目标可达去缩小圆盘或机器人。

真实外形默认沿用上传代码280×260mm。用户之前曾口述270×250mm，二者不同，必须结合相机、机械臂收拢/展开与线束实际轮廓核对。当前接口是固定矩形包络；非矩形突出物可用更大的保守包围矩形，不能只按裸底盘尺寸。

原料按钮目标布局(1200,2200)在当前设备模型下危险，会被拒绝。布局(1200,2100)只是演示接近位，不是取放位置标定。

## 后续实车前必须完成

1. 现场地图白名单和设备位置、完整车体外形、误差裕量核实。
2. 确认板上固件与上位机的XY/Yaw变换；这次没有MCU源包，未擅自更改矩阵。
3. 新版实车导航命令的版本、帧校验、目标编号、ACK/DONE、唯一控制权和失联保护。
4. 验证低速逐段停车，再设计速度前馈、轮速限制、碰撞检查后的连续转向轨迹。
5. 接雷达时用时间对齐后的位姿与外参；未知/遮挡不是空闲。

## 实现参考

- Shapely 官方手册（多边形包含关系、距离与凸包）：https://shapely.readthedocs.io/en/stable/manual.html
- Qt QTimer 官方文档：https://doc.qt.io/qtforpython-6/PySide6/QtCore/QTimer.html
- Python concurrent.futures 官方文档：https://docs.python.org/3/library/concurrent.futures.html

以上只是几何与线程API参考，不是当前工程实车安全性证明。
