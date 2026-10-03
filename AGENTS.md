# 工程导航与维护约定

## 2026-10-03 完整比赛静态障碍（当前有效）

- 用户要求比赛仿真加入障碍；仅PC静态圆柱，不增加动态障碍、雷达或STM32接入。
- 完整比赛启动保留sim_obstacles，compile_match冻结LAYOUT_MM坐标，最多4个φ50×100mm。
  固定地图与障碍快照独立保存，不重复合并；碰撞scene用于搜索/平滑/Trajectory、实际控制预演、MANEUVER和运行扫掠。
- “障碍比赛场景”按钮显式加载初赛地图和4个演示障碍，中央障碍迫使绕行；可清除后点击地图/随机补齐。
  预设为演示坐标，不能声称现场坐标。没有障碍仍允许比赛；任意随机摆放不能保证整轮可行。
- 非法放置、出入库/停靠区占用和连续路线封路明确拒绝发车；保留障碍，不复位车辆或强行平滑。
  障碍编辑取消预检/运行/作业，冻结列表有效性门禁在每步提交前保护绕过GUI的修改；迟到预检不得启动。
- 比赛栏显示障碍数量，JSON新增sim_obstacles、LAYOUT_MM、radius25/height100；固定地图快照不混入圆柱。
- 4障碍两启停区均约122.6s整轮完成12抓取/12放置，全实际矩形扫掠安全。
  新增6项无GUI、5项Qt；6组自检+206项无GUI+112项Qt通过，日志competition_obstacles_regression_20261003.log。
  使用说明和截图见HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/COMPETITION_SIMULATION_20261003.md。

## 2026-10-03 完整初赛流程模拟（当前有效）

- 用户要求不需要动态障碍/雷达，参考比赛文档新增完整流程路径模拟；仅改PC上位机。
  依据`E:/STM32/ILHC/命题与运行.txt`第3/4/6/7/8页及`评分与规则.txt`第2/3页，并查看PDF图1。
  决赛现场公布，不能伪造决赛流程。初赛一键→二维码→两批原料/粗加工/暂存→返回抽签启停区。
- 新增`competition_simulation.py`、`competition_map.json`，比赛地图“一键比赛模拟”后台预检后自动启动。
  可输入四组任务码；第二批粗加工取第四组，暂存按同色第一批所在环码垛，不能套用第四组。
  物料每次抓一件并入车载仓，粗加工全部放好再按序取回；正常抓取12/放置12，库存三环同色双层。
- 独立名义静态场景：400mm中间车道、450mm黄色禁区、圆盘按正文直径300mm、粗加工/暂存580×150mm。
  原navigation_map保留；启停区/停车区/设备接近点/静态车道节点可配置，geometry_verified=false。
- 图搜索Manhattan骨架→复用圆弧/Trajectory全矩形校验→生产控制器真实运动预演；检查100mm参考切内偏差。
  所有路线安全后才发车；失败显式fallback。启停区不能旋转，采用经完整扫掠校验的固定车头麦轮出入库。
  停车区显式MANEUVER旋转后对齐下段，区间连续跟踪20mm样本仍不停车；不能把这些动作伪装切线Trajectory。
- CompetitionRunner只消耗新积分帧，RUNNING/ACTION/COMPLETE/CANCELLED/FAULT；整轮180秒，单作业15秒上限。
  抓放/扫码为0.5秒逻辑模拟，无真实传感器或机构ACK。没有调试倒计时或转盘等待的物理模型。
- STOP、清路径、重规划、地图/任务码/启停区/参数/标定/链路变化取消预检和运行，作业也立即停止计数。
  每阶段会话/目标ID、实际终点/停稳门、作业期间静止和库存语义均检查；伪造到位/缺料/错色码垛不能成功。
- 地图显示整轮骨架、平滑轨迹、参考与实际轨迹；显示任务码/阶段/车载/统计/计时，导出完整比赛JSON及实际FIELD轨迹。
  两个启停区默认例码156+123+516+231均102.2s完成；新增15项无GUI/3项Qt，6组自检+200项无GUI+107项Qt通过。
  最终完整回归含输入深拷贝与规划侧栏可读布局修订。说明/日志在`HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/COMPETITION_SIMULATION_20261003.md`
  与`competition_regression_20261003.log`；未改固件/协议，未接STM32、雷达或动态障碍。

## 2026-10-03 PC连续轨迹跟踪（当前有效，覆盖下方仅预览/固定航向逐点执行约定）

- 用户明确要求修改PC模拟器为连续跟踪，完成模拟及故障注入后停止；不得接STM32。
  新增`trajectory_tracking.py`，整条安全Trajectory一次接受一个epoch/goal_id，不逐样本GOTO或等待到位。
- 实际FIELD位置投影到LINE/ARC精确几何，进度单调；前向窗口=实际位移+20mm，同距取早分支。
  100mm弧长lookahead给出连续X/Y/unwrap切线Yaw，初始位置/切线须在5mm/5°内，不隐式原地旋转。
- core50Hz更新实际XY及航向，PC限速250mm/s、120°/s，小半径限速；中间点不停车。
  最终STOP必须实际位置<1mm、航向<1°、速度≤1mm/s、角速度≤1°/s，连续10个积分帧200ms。
  GUI另核对会话和完成ID/实际终点；旧误差、重复轮询或伪造完成ID不得判成功。
- 接受前全轨迹矩形复检，运行每步真实矩形+pad连续扫掠；drivable完整包含、固定/模拟/动态障碍都检查。
  实际运动包络是端点矩形凸包+旋转弦高外扩，不能只检查规划车心或固定航向。
- STOP、清路径、重规划、地图版本/几何变化同步取消core跟踪；可观察地图顶层变更先取消再写入。
  每积分步读取普通Python快照/标定/链路有效性，不从模拟器工作线程读取Qt控件。
  重几何接受检查在锁外，最后接受/运动提交再次检查epoch与有效性，取消与XY/yaw提交共锁。
- NaN/Inf、定位>50mm或航向>15°跳变、碰撞/检查异常、轨迹偏离>75mm、3秒无进展/总超时停止。
  接受检查与运动保护回调中的取消/版本变化不得恢复跟踪或提交迟到运动。
- 地图黄色虚线Manhattan骨架、绿色连续轨迹、紫色参考姿态、蓝色实际轨迹；保留真实轨迹历史。
  清路径同时清规划与参考；连续生成失败/TURN/真实直角/fallback拒绝执行，旧手动GOTO调试保留。
- 新增19项无GUI专项、2项真实Qt专项，完整6组核心自检、185项无GUI、104项Qt通过。
  说明/完整日志在`HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/CONTINUOUS_TRACKING_20261003.md`
  与`continuous_tracking_regression_20261003.log`；仅改上位机及测试/文档，未连接STM32。

## 2026-10-03 连续Trajectory（当前有效）

- 用户要求LineSegment/ArcSegment连续Trajectory：新增`trajectory.py`，core薄包装；成功规划自动从
  安全`smoothed_primitives`生成，原A*及动作台账不改。不得接STM32或用密集点替换旧GOTO执行。
- 输出五字段x_mm/y_mm/field_yaw_deg/s_mm/segment_type，**统一FIELD_MM场地坐标**，与原
  LAYOUT_MM规划坐标区别明确。位置=(2250-layout_y,2250-layout_x)，切线航向=-90-layout_yaw。
  航向由几何有向切线计算，不取原车头/OPS Z角；真实矩形按该新切线航向重新检查。
- 全局s=0,20,40…，额外保留连接点和终点，间距≤20mm；圆弧s用真实弧长；切点仅一行并归后段。
  unwrap累计最短角差，多圈可超过360°，不得导出前再取模。位置与有向切线必须连续。
- 保留TURN/真实直角/掉头明确FALLBACK_REQUIRED，不把unwrap当平滑、不强行连接或发布假连续样本。
  EMPTY/NO_FOOTPRINT/INVALID_INPUT/UNSAFE/CANCELLED/RESOURCE_LIMIT等失败清空trajectory且safe=false。
- 最终检查所有样本矩形+pad、drivable完整包含、固定/模拟/动态障碍；全直线连续凸包扫掠，圆弧
  再按≤20mm且≤3°及凸包+顶点弦高包络检查完整间隙。共用CollisionScene.arc_interval_reason。
- 输出trajectory_status/safe/continuous/reason/length_mm/spacing_mm/frame_id/yaw_convention。
  core.validate_trajectory可按最新场景重检，包括样本几何/弧长/类型/unwrap一致性。
- 地图箭头正确处理FIELD→LAYOUT镜像和Qt y翻转；直线100mm、圆弧40mm稀疏显示，失败/清除即清空。
  原计划JSON包含Trajectory；独立“导出Trajectory JSON”导出前重新复检并保存碰撞快照，禁止过期导出。
- 新增20项专项及1项Qt箭头测试，扩展Qt真实导出按钮验证；文档及日志：
  `HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/TRAJECTORY_20261003.md`、`trajectory_regression_20261003.log`。
  6组核心自检、166项无GUI、102项离屏Qt全部通过。固件/地图/协议未改，未连STM32；完成后停止。

## 2026-10-02 90°圆弧平滑（当前有效，覆盖下方禁止圆弧的旧阶段约定）

- 用户明确要求90°圆弧；原A*曼哈顿台账仍保留，安全后处理由`arc_smoothing.py`实现。
- 每个真实单次90°、同模式且航向/几何方向一致的Corner严格按120/110/100/90/80/70/60mm尝试。
  输出ArcSegment入口切点、圆心、出口切点、CW/CCW、真实航向和姿态采样，不取整车体。
  只换轮模式/掉头/混合动作保留；相邻圆弧扣除已占用的直线，禁止切点重叠。
- CollisionScene.pose_safe(x,y,yaw)以真实矩形+pad完整包含于drivable白名单并排除所有固定/模拟/动态障碍。
  禁止退回车心、顶点或pad-only模型假装完整车体。接触障碍拒绝，区域边界完整覆盖可接受。
- 圆弧同时满足≤20mm弧长与≤3°角采样，含入口/出口；每姿态检查，再用相邻矩形凸包+顶点弦高外扩
  覆盖采样间隙。剩余直线、保留原地转向与整条轨迹位置/航向连续性都复检。
- 全部半径失败明确ALL_RADII_FAILED并记录7次原因；绝不强行平滑。整轨迹失败不发布平滑几何。
- 新增arcs/arc_attempts/arc_fallbacks/smoothed_primitives/smoothed_points/smoothed_length/
  smoothing_status/smoothing_model_safe；原points/steps/segments/corners/搜索代价不变。
  地图显示圆弧采样轨迹，台账/JSON显示所有切点、方向和fallback；净空仍注明原台账。
- 固定/模拟/动态障碍都进入规划快照；地图可含dynamic_rects/circles；UI线程set_dynamic_obstacles更新
  完整快照、取消旧计划/仿真目标；输入复制及版本签名拒绝旧结果。未新增雷达数据接入或时空预测。
- 仿真执行器仍只支持固定航向，含圆弧明确拒绝。实车整条路径执行未开放，固件/地图/协议未改。
- 新增25项专项、2项Qt测试；6组核心自检、146项无GUI、101项离屏Qt全部通过；实现与验证详见
  `HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/ARC_SMOOTHING_20261002.md`及回归日志。

## 2026-10-02 路径算法审查修复（当前有效，覆盖下方旧算法细节）

- 修复上位机6个反例：重复旋转导致转向漏检、非轴向实际航向被拒、L形直接返回非最优路线、
  共线动作信息丢失、操作区端点豁免覆盖整段道路、dataclass无法导出JSON。
- 四个航向状态保留实际初始角、相差90°；车体碰撞模型不取整。动作按最近主轴分类；
  目标航向须与起始角相差90°整数倍。真实姿态/裕量越界仍正确拒绝，不能删掉一致性或边界校验。
- 转向：用零角局部车体顶点旋转到绝对角；相邻采样姿态凸包加含pad半径弦高外扩覆盖采样间隙。
  按航向对缓存扫掠，按准确车心判碰撞，量化后的转向也复检；整车净空包含转向的保守下界。
- L形候选作搜索上界；启发式为曼哈顿×最低平移系数+必要净转向代价；保留不同连接额度候选，
  终点连接按到达状态逐段复查。最优性只限当前有界网格及枚举的连接图，不宣称连续空间全局最优。
- 横移额度精确记录区外距离，区内不计额度但穿过操作区不清零；前进/后退/转向清零。
  不再用6档连续向上取整。保留400000状态上限、时限和取消，超限明确失败。
  `max_strafe_run_mm`是完整物理连续横移；新增`max_outside_strafe_run_mm`及台账
  `outside_strafe_run_mm`是用于限额的区外连续横移，两者界面同时显示。
- `points`是几何共线压缩；`segments`是MOVE动作分段，同一直线的动作/航向切换点保留，
  所以段数可以多于`len(points)-1`。角点按坐标与台账出现顺序匹配；`reverse`表示后退或横移主动作。
  UI完整显示steps里的所有转向，axis_matched=false不能显示成功，含转向路线不显示固定航向GOTO台账。
- JSON导出转换segments/corners的as_dict。仿真仍只执行固定航向，实车整条路径执行未开放。
- 回归：6组核心自检、121项无GUI（新增17项审查修复回归）、99项离屏Qt通过。
  详见`HostTools/ILHC_Debugger/ILHC_Qt_v2/Docs/NAVIGATION_FIXES_20261002.md`。
  只改上位机及测试/文档；固件/地图/协议未变，本次无需烧录；未验证实机。

## 2026-10-02 共线压缩输出 LineSegment / Corner（当前有效）

- 需求：实现 Manhattan 路径共线压缩与 90° 角点识别；**只能删除共线节点**；保留 A* 输出的
  动作与航向语义；输出 LineSegment 与 Corner 对象；**不得生成任意斜直连**；补直线/L/Z/U 形测试。
- 实现（`navigation_planner.py`）：新增 `LineSegment` 与 `Corner`（`@dataclass(frozen=True)`，
  各带 `as_dict()` 供 JSON/CSV）与 `path_axes(points, steps=None)`；`core.geometry_axes()` 是
  给界面/测试的薄包装。规划结果新增 `segments` / `corners` / `axis_matched`，
  只修改成功分支（失败分支保持 `[]`/`False`，与其他结果字段的约定一致）。**没有新的节点删除规则**：
  共线合并仍只由既有的 `simplify_collinear`（`check=False`）负责，`path_axes` 只做两件事——
  删掉与前一点重合的点（零长度段）、把拐点识别成角点。某段 `dx` 与 `dy` 都不为 0 直接
  `raise ValueError`（这正是 string-pull 会犯的错，也是本模块禁止的"任意斜直连"）。
- **角点 ≠ 转向**（这次实测踩到的坑，别再按顺序发动作令牌）：
  * 几何角点未必有转向——"横移 → 后退"只是轮模式变了、航向没动（实测
    启停区1→暂存区 第1个角点就是这种）；"180° 原地掉头"也未必留角点——转完继续沿同一条
    直线走时那个顶点被共线合并掉了。
  * 因此 `Corner.action` 按**坐标**去台账 `steps` 里找转向动作，找不到就是空串（真的没转），
    `Corner.steps` = 该点实际转过的 90° 数（0 表示只换轮模式，2 表示掉头）。
    角点数**不能**当步数用（`len(segments) != len(corners) + 1` 是合法的）。
  * `Segment`/`Corner` 的 `heading_deg`/`action` 全部**照抄**台账，不重新推断；
    配对用 0.01mm 规整化的坐标（UI 量化会改坐标，配不上就留空串 + `axis_matched=False`，绝不猜）。
  * `axis_matched` 只看**线段**（每条线段都必须配上台账行）：真实规划里线段与角点都由台账决定，
    若某条线段配不上就必须报警。
- 语义字段：线段 `axis`(x/y=沿哪条轴走) / `direction`(±1) / `length_mm` / `heading_deg` /
  `reverse`(航向与该方向相差 >1° ⇒ 后退或横移) / `action`；角点 `turn`(LEFT/RIGHT，
  逆时针为正、与 +Z 一致) / `heading_in_deg` / `heading_out_deg` / `steps` / `action`。
  `heading_in == heading_out` 的角点就是"只换轮模式"。
- UI（`main.py::_waypoint_lines`）：台账末尾追加「压缩后 N 直线段 / M 个90°角点」明细；
  航向不变的角点明写"航向不变：只换轮模式（不是转向）"，有转向的行补「该点 TURN_X，共 n×90°」。
- 测试：`tests/test_navigation_safety.py::PathAxesTests`（8 个）——直线（两段、无角点）、
  L 形（两段一个左角）、Z 形（三段两角、左右相反）、U 形（∩ 形两个左转、∪ 形两个右转，
  且每段仍轴对齐）、**只删重复点**（共线合并不在 `path_axes` 里做）、斜段被拒、
  角点≠转向的两种情形、以及真实规划里"每段航向/动作与台账逐条一致 + 角点数 == 折线方向改变数
  + 有转向的角点数 == `turn_count`"。回归：`run_tests.py` 104 用例、`run_tests.py --qt`
  99 用例全过。只动上位机（`navigation_planner.py`/`core.py`/`main.py`/`tests/`），
  **固件与协议未变、无需烧录**；未连硬件。
- 顺带修掉一处老误报（**产品行为未变**）：`test_debugger.py::test_map_click_plans_without_sending_anything`
  的 `line_q.empty()` 改成 `core.commanding_commands(...) == []` + "队列里除 `GET …` 外不得有
  任何条目"。这就是 2026-09-29 记过的那个坑（参数回读轮询每秒放一条 `GET`），当时只改了
  `test_map_click_qt.py`，`test_debugger.py` 这处漏改；本次因整类运行触发而暴露。
  追踪确认队列里只有一条 `GET XVMIN`，没有下发任何机构命令。

## 2026-09-30 「禁横移」改成「限制长距离横移」（当前有效）

- 起因：勾「道路禁横移」后**任何点位都规划不出来**。定位到失败在**起点侧**（插桩后
  `sources=0`，目标侧 18 个格全好），所以换目标没用。根因链：① 启停区中心离两墙各 150mm；
  ② 禁横移后唯一换向方式是原地 90° 转向；③ 转向的采样扫掠判据要求**每个方向**都有
  ≈207.7mm 净空（半对角线 191 + pad 10 + 扫掠外扩 2.63；扫掠里含 45° 姿态，那个姿态伸得最远，
  所以不能用车长一半 140 估）；④ 150 < 207.7 ⇒ 起点和它周围所有 L 形拐角都转不动，
  连接段必有一条垂直边 ⇒ `attach()` 返回空；⑤ 就算接进去了，起点航向线（y=2250）整条离墙
  150mm、线上处处转不动，禁横移又不能在线上横出去 ⇒ 车被困死（网格调细到 5cm 让起点落在
  网格线上时，报的从 `NO_CONNECTION` 变成 `NO_PATH`，同一个根因）。pad 改 0 也没用（仍要 193.7mm）。
- 需求：不是完全禁止横移，而是**禁止长距离横移**——短横移（出库、对位微调）照常可用。
- 口径（已与用户确认）：**单次连续横移 ≤ N**，默认 **N=500mm**。连续横移被前进/后退/转向
  打断后重新计数，**不是全程总量**。已知代价（用户知情接受）：打断后可以再接一段，所以
  转向代价被调得极高时会出现"阶梯"式横向推进（见下面实测）。
- 实现（`navigation_planner.py`）：新参数 `strafe_run_limit_mm`（None=不限，0=完全禁横移）。
  规则：**操作区（`strafe_polygons`）内不受限**，否则 `已连续横移 + 本次横移 ≤ N` 才放行；
  `allow_strafe=False` 与 `N=0` 等价（有测试锁等价性）。状态随之扩成 **(格 i, 格 j, 航向, 连续横移桶)**：
  桶按网格步长精确离散（`floor(N/step) ≤ 8` 时），细网格下退化为最多 6 档并**向上取整**
  （只会高估已用横移量 ⇒ 判据更严，绝不会放行更长的横移）。前进/后退/转向把桶清零。
  `connector_seq()` 也按上限判定：垂直边若横移会超限，就退回"原地转向对齐再直行"的老行为；
  它现在返回末尾剩余的连续横移量，`attach()` 用它给出**接进网格那一刻**的横移量（起点侧是
  连接段末尾剩下的，终点侧是连接段开头要接着走的，终点侧在到点时与到达状态一起判），
  两个 L 形都能走时优先选**不消耗横移额度**的那个。`finish()` 逐段复查里再自查一遍上限
  （合并/回溯不许放出更长的横移），台账每步新增 `strafe_run_mm`，结果新增
  `max_strafe_run_mm` 与 `strafe_run_limit_mm`。
- 修掉一个测试当场抓出来的真 bug：操作区多边形的解析守卫原写成"上限 > 0 才解析"，
  于是 `N=0`（完全禁横移）时操作区**完全失效**，区内连出库都做不到。现为"给了上限就解析"。
- UI（`main.py`）：勾选框改为「限制长距离横移」，参数区新增「横移上限」（0..200cm，
  默认 50cm，未勾选时置灰）；`_prepare_plan` 传 `strafe_run_limit_mm`，`_navigation_signature`
  带上勾选状态与上限，结果卡多一行「单次最长连续横移 X mm（上限 Y mm）」。**不再传
  `allow_strafe=False`**——完全禁横移现在靠把上限调到 0 表达，侧栏提示里写明那是陷阱。
- 实测（默认 pad=10mm、grid=100mm、起点=启停区1中心）：
  * 上限 500/300mm：原料区→(1200,2100) 成功，1.20m，横移 150mm（就是出角落那一下）。
  * 上限 0：`NO_CONNECTION`，提示改为「已完全禁横移：起点/终点附近若转不动…放宽横移上限」。
  * 转向等效代价调到 5000mm（本版仿真行驶不执行路径中转向时会这么调）时，不限上限会一路
    横移 1450mm（最长连续 800mm）；上限 500 后变成 450/500/500 三段横移夹 50~150mm 前进后退，
    代价 2718→2932；上限 300 则四段 300mm，代价 3148。**阶梯是口径的已知后果**，
    要堵死就得改成"全程横移总量"口径（另一维状态，未做）。
  * 上限放宽 ⇒ 可行集变大 ⇒ 最优代价单调不增（已锁）；不设上限的结果与"给个极大上限"逐位相同。
- 测试：`tests/test_navigation_safety.py::StrafeRunLimitTests`（7 个：短横移出库可用、
  上限 0 与 allow_strafe=False 等价、长横移被切到上限内且代价单调、台账 `strafe_run_mm`
  自洽且不超限、操作区豁免、上限单调性、不设上限＝行为不变）与
  `UiLogicTests::test_strafe_limit_check_reaches_the_planner`（勾选/置灰/上限真的进规划器/
  上限 0 时如实失败且不下发任何命令）。回归：`run_tests.py` 95 用例、`run_tests.py --qt`
  96 用例全过。只动上位机 `navigation_planner.py`/`core.py`/`main.py`/`tests/`，
  **固件与协议未变、无需重新烧录**；未连硬件。

## 2026-09-29 A* 状态升级为 (格, 航向) + 动作代价（当前有效）

- 需求：① 状态从 (ix,iy) 升为 (ix,iy,heading)；② 前进/后退/横移/转向分别计价，默认
  forward=1.0、backward=1.15、lateral=1.8、turn_penalty=180mm 等效；③ 启发式仍用曼哈顿距离 ×
  最小移动代价；④ 回溯必须同时给出 x,y,heading,action；⑤ 普通道路可配 allow_strafe=false、
  操作区可允许横移；⑥ 测试证明提高 lateral 系数后横移总距离不增加；⑦ 不做圆弧；⑧ 串口/STM32 不动。
- 实现（都在 `navigation_planner.py`）：
  * 航向 4 值（布局帧 0/90/180/270° 逆时针，`heading_index/heading_deg/heading_vector`）⇒
    **每个航向一套 `CollisionScene`**：车体朝向随航向变，平移的整段检查用"当时航向"的模型。
  * 平移仍是严格 4 邻域，按当前航向分类（`action_for_move` = 前进/后退/横移），代价 = 步长 ×
    `move_factor`；启发式 `h = 曼哈顿(mm) × cost_forward`（最便宜动作 ⇒ 可采纳且一致 ⇒ 加权最优）。
  * **转向只在原地**做 90° 整数倍（`TURN_LEFT/RIGHT/AROUND`），每 90° 记 `turn_penalty_mm`；
    没有圆弧，输出折线仍是直角。转向可行性：把车体在起止航向之间**采样旋转（15° 步长）**扫出的
    区域并集，再按"外接半径×(1−cos7.5°)+1mm"外扩，要求整块在合法区域内且不碰障碍
    （`turn_sweep`/`turn_free_point`）。**不要改回"外接圆盘"判据**：圆盘要 402mm，会把 400mm
    走廊里的转向全否掉——禁横移时整条走廊都走不通（实测 NO_CONNECTION）。
  * 连接段（起点/终点不在网格上）走轴对齐 L 形；**禁横移时靠原地转向对齐每条边**，最后转回该侧
    状态要求的航向（`connector_seq`），动作序列存进 attach 结果供回溯复用。起点贴墙角
    （启停区中心离两墙各 150mm）时原地转不动 ⇒ 禁横移会以 NO_CONNECTION 明确报出并给提示，
    这是物理正确的拒绝，不是 bug。
  * `finish()` 逐段按"当时航向"复查（**不能**用单一航向的 scene.validate 整条折线：路径可能含
    原地转向）；共线合并传 `check=False`（只把同向共线段并成一条，扫掠区域不变）。
  * 输出：`points`/`raw`（几何折线、直角保留）、**`steps`**（x,y,to_x,to_y,heading_deg,action,
    kind,distance_mm,cost_mm；同向直行合并、转向零位移成步）、`search_cost`（加权）、`trace_cost`
    （按 steps 复算，必须 == search_cost）、`lateral_mm`/`turn_cost_mm`/`turn_count`。
  * **安全校验**：`footprint` 的航向必须等于 `start_heading_deg`，否则 INVALID_INPUT——
    不然等于静默换一个车体朝向求解。
- UI：规划侧栏新增「道路禁横移」（默认关）与「转向等效代价」（默认 180mm）；`_prepare_plan` 传
  start=goal=当前布局航向 + `allow_strafe` + `turn_penalty_mm`，并读地图 JSON 的 `strafe_polygons`
  作为操作区。**仿真行驶只执行固定航向、不执行路径中转向**：`turn_count>0` 的路线会被明确拒绝
  （提示改写规划或只看台账）——不拿单一航向的模型去"复查"各段朝向不同的路径。
- 测试：`HeadingPlanTests`（台账四要素与动作自洽、加权系数、**提高 lateral 后横移总距离不增加**、
  禁横移改用转向 + 操作区放开、航向/车体一致性拒绝）；`test_independent_dijkstra_40_scenes` 的
  独立参照升级为 **(格,航向) 状态 Dijkstra**（同一动作模型）。回归：`run_tests.py` 88 用例、
  `run_tests.py --qt` 88 用例全过。未连硬件、未烧录。
- 代价变化（预期）：会"绕路换朝向"。例：启停区1→暂存区 2.97m / 加权 3472（含 2 次转向、150mm 横移）。
- 连带注意事项：① 规划比原来慢（4 倍状态 + 转向扫掠），**Qt 测试里不能再用 `QTest.qWait` 等规划**
  （qWait 期间工作线程拿不到 GIL，会被误判"超过时限"），要用 `processEvents()+sleep`；
  ② headless/测试里规划后要**重新喂帧**（350ms 定位闸门 vs 秒级规划时间）；
  ③ UI 的量化复检改为**按段航向**（`_validate_quantized`），否则转向路径会被误报
  "坐标量化后不安全：第2段：中央物料区"。

## 2026-09-29 navigation_planner 改为严格 4 邻域曼哈顿搜索（当前有效）

- 需求：A* 从 8 邻域改为**严格 4 邻域（上下左右）**、启发式用曼哈顿距离、输出直角折线，
  且**不允许**再把直角拉成斜线；串口与 STM32 不动。
- `plan()`：邻居集合固定为 `((-1,0),(1,0),(0,-1),(0,1))`，每步代价恒为 `step`，
  启发式 `h_manhattan`（可采纳且一致 ⇒ 网格代价最优）；**每条相邻边仍整段检查**
  （`edge_free → segment_reason`），不是只看目标格是否空闲。
- **连接段也必须轴对齐**：`attach()` 改成 L 形连接（先横后竖 / 先竖后横，两段都整段检查，
  代价取曼哈顿距离），返回 `{cell: (cost, corner)}`，还原路径时把 corner 插进折线；
  "直线可达"的快路径也从 `[start, goal]` 换成 `l_shape()` 的 L 形——否则斜线会从这两处漏进输出。
  端点仍精确，不替换、不吸附。
- 简化：`plan()` 改用新增的 `simplify_collinear()`（**只合并共线节点**，方向改变的拐点一律保留），
  不再用 `simplify_checked()` 的任意视线直连。`simplify_checked` 原样保留，
  `core.simplify_path()` 兼容接口仍走它。
- **不变量**：每段 `dx==0 或 dy==0` ⇒ 折线欧氏长度 == 曼哈顿搜索代价
  （`search_cost == path_length(raw)` 继续成立）。
- 测试：新增 `tests/test_navigation_safety.py::ManhattanPlanTests`——6 组用例逐段断言轴对齐
  （含两端都不在网格上、同列、长对角、有/无模拟障碍），共线合并不加长且保留直角。
  `test_independent_dijkstra_40_scenes` 的**独立参照改成 4 邻域曼哈顿 Dijkstra + L 形连接**
  （原参照是 8 邻域欧氏，度量不同必然不符——改的是参照，不是放宽断言）。
- 回归：`run_tests.py` 82 用例、`run_tests.py --qt` 96 用例全过。未连硬件、未烧录。
- 代价变化（预期）：路径变为曼哈顿长度。例：启停区1→暂存区 2.67m→2.95m，
  恰好等于 `|Δx|+|Δy|` 下界，说明仍是该度量下的最优。

## 2026-09-29 模拟障碍 φ50×100mm（≤4 个，点击/随机放置）（当前有效）

- 需求：加黑色模拟障碍 φ50×100mm（顶视 r=25mm 圆），支持点击放置与随机，最多 4 个，
  放置后规划必须绕开。
- 实现：`core.SIM_OBSTACLE_R_MM/H_MM/MAX/NAME`；`sim_obstacle_circles()`（转 circles 表）、
  `obstacle_placement_blocked()`（判据是"把既有障碍按 radius+margin 膨胀再看圆心"——
  圆精确、矩形保守）、`random_obstacle_points()`（避开固定禁区/已有障碍/场地边缘/keep_clear；
  放不满时如实返回少放数量与原因，绝不返回非法点；`rng=` 可注入 `random.Random` 供测试确定化）。
- **障碍是运行时演示物体，不写进 `navigation_map.json`**（那是实测场地数据）。
  `MainWindow.sim_obstacles` 存布局坐标，`_obstacle_circles()` 并入 `_prepare_plan` 与
  `_request_plan_anchor` 的 `circles`，因此 `_scene_for_context`（仿真行驶前复查）自动带上。
- **障碍进 `_navigation_signature`**：放置/清除/随机都走 `_obstacles_changed()` → 重画 +
  `_clear_path()`，旧计划立刻失效——不让旧路径去执行一个已经变了的场景。
- 界面：规划侧栏新增「点击地图 = 放障碍」「随机补齐」「清除障碍」与计数行；放置模式**优先于**
  规划开关（放置时既不规划也不下发）。`FieldView.set_sim_obstacles()` 画黑色实心圆 + 浅色描边，
  z=12.5（在规划路径之上、车体之下，车压上去看得出来）。
- 随机放置带 `keep_clear`（车心 + `CAR_HALF_DIAG_MM` + 半径 + 20mm）：否则障碍可能压在车上，
  之后每次规划都会以"起点：模拟障碍"失败。
- 连带：`_goto_field`（直接 GOTO 调试通道）的目标/直线检查改为读 **`nav_map` 几何 + 模拟障碍**，
  不再读 `core.FIELD_FORBIDDEN_*` 旧副本，少一份漂移来源。
- `tests/headless_support.py`：假控件补 `setChecked`/`set_sim_obstacles`，`make_window` 增加
  `sim_obstacles` 与 `obstacle_info`/`obstacle_mode_check`——该 harness 按固定属性名造假控件，
  **新增界面状态必须同步这里**，否则无 GUI 套件会整片红。
- 回归：`run_tests.py` 79 用例、`run_tests.py --qt` 96 用例全过。新增
  `tests/test_navigation_safety.py::SimulatedObstacleTests`（φ50 表、冲突/边缘判定、随机合法性
  与放不满说明、keep_clear、**规划必须绕开且整条路径在含障碍的同一模型下合法**）与
  `test_debugger.py::SimulatedObstacleUiTests`（点击放置不规划不下发、上限/冲突/压车拒绝、
  旧计划失效+重规划改道、随机不压车+清除）。未连硬件、未烧录。
- 端到端实测（离屏）：堵住中间+右侧走廊后，启停区1→暂存区 的规划改为
  (2250,2250)→(400,2000)→(350,1200)，车心线距障碍 823mm。

## 2026-09-29 上位机视觉跟踪页 + 参数回读"黑洞"修复（当前有效）

- 起因：现场发 `GET VDBMM` **完全无反应**，看着像板子没响应。**根因在上位机，不在固件**：
  参数回读行在 `core.FrameParser` 里就被 `PARAM_ECHO_RE` 分流进 `param_q`、
  **根本不进日志文本**（`core.py` 的 `_scan_text`：匹配 `^名称=数值$` 就 `continue`），
  而 `main._apply_param_readback` 当时对"界面没有对应参数行"的名字直接 `return`——
  视觉那 6 个参数当时没有对应的参数行，于是整条回读被静默吞掉。
- **诊断这类"没反应"的正确探针**（重要，别再误判成固件故障）：
  ① 若板上是旧固件，`GET VDBMM` 会回 `ERR PARAM UNKNOWN; GET ...`，那行**不匹配**正则、
  会照常进日志；**什么都看不到**才说明固件回的是 `VDBMM=数值` 形式。
  ② 想确认固件版本，发 `GET ZZZ`（不存在的名字）：它回的那条提示文本会进日志，
  里面列出全部参数名——**含 `VDBMM|VDBPX|VMIN|VMAX` 就是新固件**。
  ③ 参数设置命令（`VMIN=2` 这类）**本来就没有任何应答**，且这 6 个视觉参数不在 Flash
  快照里，连 `ACK PARAM SAVED TO FLASH` 都不会有（KPX 那类在 Flash 记录里才会有）。
- 修法：`_apply_param_readback` 遇到界面没有参数行的名字改为**打一行日志**（`回读 X=Y（界面无该参数行）`）
  而不是静默返回；`test_unknown_param_readback_is_logged_not_swallowed` 锁住这条。
- 新增「视觉跟踪」页（**下标 8，追加在末尾**——`page_names`/`self.pages`/`_build_vision_page()`
  三处同序插入；既有测试与 `_render_ui` 依赖页面下标，只能往后加）：6 色启动按钮
  （`core.VISION_COLORS`）+ 停止 + 刷新回读 + 6 个参数行（复用 `ParamRow`，回读通道全 `None`）。
  进页面时 `_select_page` 钩子里批量刷新一次回读，**刻意不并入 1Hz 的 `_poll_param_readback`**：
  那会把无通道参数的轮询周期从 2s 拉到 8s，每条应答还要占一个遥测帧位。
- 顺带修掉两个真实竞态（都是"新页面会发 VTRACK"才暴露出来的）：
  ① `send_line` 的"停止键盘续发"名单补 `VTRACK`——固件启动跟踪时会清 `s_manual_active`，
  但上位机 20Hz 的 MANUAL 流不会自己停，会把固件立刻拉回手动、与视觉速度互相打架；
  ② `core.discard_motion_commands` 的丢弃集合补 `VTRACK`——否则"先点跟踪、再按 STOP"时，
  排在 STOP 之后的那条 `VTRACK=1` 会在停止之后把跟踪重新拉起来（与 GOTO/MANUAL 同类竞态）。
  `core.COMMANDING_COMMANDS` 也补了 `VTRACK`，让 test_map_click_qt 的"规划模式不得下发"守卫覆盖它。
- 模拟器：补 6 个视觉参数（设置+限幅+`GET` 回读）与 `VTRACK=` 状态记录，`--simulate` 下回读
  不再恒为"—"。**但 `Simulator` 没有 `text_q`**，固件那些 `ACK VTRACK ...` 文本只在真实串口下出现；
  页面上的状态标签只反映本机请求，不代表工控机已开始识别。
- 回归：`run_tests.py --qt` **92 用例全过**（新增 14 个视觉用例，含"逐个点真按钮"以抓
  闭包全绑最后值的经典 bug）；`core.selftest()` 第[3]步加了视觉参数断言。
- **未做/未验证**：未连硬件、未烧录。**目前板上是否已是新固件仍未确认**——用上面 ② 的
  `GET ZZZ` 探针判。24 通道遥测里**没有视觉位**，所以页面上看不到实时 EX/EY/FLAGS/CONF
  （那些只在工控机自己的 VNC 画面上）；要让上位机也看到，需固件侧另开可 `GET` 的状态量或占通道，
  属独立改动。

## 2026-09-29 功能区接近点改为几何现算 + 地图数据合规守卫（当前有效）

- 现象：点击「原料区」显示"规划失败"。根因**不是规划器**：`core.QUICK_TARGETS` 里
  写死的 (1200,2200) 对 280×260 车体是**非法停车位**——离原料区圆盘（(1200,2400) r=110）
  表面只有 90mm，而车体轴向要 140/150mm、斜向（(150+140)/√2）要 **205mm**，
  于是新模型的固定航向整车校验如实拒绝（`INVALID_GOAL`）。用户自己的
  `tests/test_navigation_safety.py::test_default_material_goal_rejected` 与
  `test_unsafe_goal_cannot_start_simulation` 早就把"该点必须被拒"写死了，
  **是数据没跟上模型**（这批坐标是"车还是质点"时代挑的）。
- 修法（已做）：`QUICK_TARGETS` 换成 `core.QUICK_ANCHORS`（名称 → 功能区锚点 +
  由功能区指向场内的单位方向：原料区用圆心、暂存区用右缘中点、粗加工区用上缘中点），
  目标改为**每次现算**：`core.approach_target()` → `navigation_planner.approach_point()`。
  界面 `_request_plan_anchor()` 用 `_plan_preconditions()`（新抽出的公共前置：起点/航向/模式）
  与 `_layout_yaw_for()` 取**同一个**航向，再规划；算不出就明说，不猜点。
- `approach_point(anchor, outward, margin, footprint, standoff, step=5)`：沿方向逐点判定，
  **判据是车体外缘净空**（`CollisionScene.body_clearance`，含合法行驶区边界），不是
  "扫到第一个合法点"。后者会把窄的可停窗口整段跳过去——实测 pad=60mm 时原料区那条
  合法缝只有几十毫米宽、且位于"刚好合法距离"的**内侧**，粗扫 25mm 直接漏掉并误报"没有
  合法停车位"（这个缺陷是本次新加的守卫测试当场抓出来的）。找不到满足 standoff 的点时
  `ok=False, code=NO_STANDOFF`，并把"合法但净空不足"的 `fallback_point` 一并返回，
  由界面**明示建议**（"建议改用场地(x,y)cm 并人工确认走法"），绝不静默采用。
- 地图数据：`navigation_map.json` 升 `map_version=2`，新增 `rulebook` 块记录官方场地图尺寸
  （2400/450/400/150×580/400×150/300×300）与**带区间的可变项**：转盘圆心距右边缘
  `1100–1300`、暂存区孔位距左边缘 `75–85`、以及 `raw_turntable_radius_mm: null`
  （**图上没有 R/⌀，属未实测**；为 null 时守卫会要求 `geometry_verified` 保持 false）。
  当前几何一律取区间中值——比赛当天若摆到区间边缘，rects/circles 必须按实测改。
- 新增守卫（`tests/test_navigation_safety.py::MapDataComplianceTests`）：
  ① `test_map_matches_rulebook_dimensions`（地图几何 vs rulebook 标注）；
  ② `test_turntable_declared_position_and_radius`（圆心在边界上、x 落在允许区间、
     半径未实测时不得声称 verified）；
  ③ `test_quick_targets_are_legal_for_every_heading`（3 个功能区 × 裕量 10/30/60mm ×
     8 个航向，现算的点必须被同一模型判为合法、且净空 ≥ standoff）；
  ④ `test_hardcoded_quick_target_removed_and_still_illegal`（旧坐标必须已删除且仍判非法）。
- 顺带修掉两个把回归盖红的测试问题（**产品行为未变**）：
  ① `test_map_click_qt.py` 的 4 处 `line_q.empty()` 改为新增的
     `core.commanding_commands()` 判定——参数回读轮询每秒往同一队列放一条 `GET …`，
     真实事件循环下 `empty()` 必然误报；`COMMANDING_COMMANDS` 与
     `discard_motion_commands` 的集合**刻意不同**（后者只清可丢的旧目标，绝不含 STOP）。
  ② 保留了"规划不得下发机构命令"的强度：断言队列里除 `GET …` 外不得有任何机构命令。
- **仍待处理（本次未改）**：`QTest.qWait` 等待期间占住 GIL 会饿死规划线程，使 8s 时限
  在**没有任何人取消**的情况下触发，报的却是"规划已取消或超过时限"（实测同一规划
  qWait 等待 8243ms/CANCELLED，`processEvents+sleep` 等待 13.8ms/OK）。建议：
  测试改用 `processEvents()+sleep`（或直接等 future）；规划器把 `TIME_LIMIT` 与
  `CANCELLED` 分开报，且不要让"被抢占"的时间吃掉预算。
- 回归：`run_tests.py`（74 用例）与 `run_tests.py --qt`（78 用例）**全过**；
  端到端实测：原料区 → (1200,2100)、暂存区 → (350,1200)、粗加工区 → (1200,340)，
  三者都不下发机构命令，旧坐标 (1200,2200) 仍被拒。未连硬件、未烧录。

## 2026-09-29 视觉协议升级到 V1.1（坐标系修正版，当前有效）

- 需求：把固件从视觉协议 V1.0（请求4字节无校验、响应15字节逐字节异或）升到 **V1.1**。
  对端 Jetson 的 `GongChuang/` 实现**默认就是 V1.1**（可 `--protocol v1` 退回），帧长与校验
  两端不匹配时**不会静默出错，而是完全不通**——固件的 4 字节请求会被按 6 字节消费后校验失败，
  按规范§15 要求静默丢弃，所以联调时看到的现象是"发了请求、工控机毫无反应"。
- **权威文档**：`E:/STM32/ILHC/jetson/STM32F407_视觉通信协议_V1.1_坐标系修正版_AI_Agent版.md`
  （请求帧速查：同目录 `V1.1_请求帧速查.md`，含全部 TASK×TARGET 的实测 CRC）。
  **`Jetson_视觉通信协议_V1.1_AI_Agent版.md` 是不带"坐标系修正版"的早期草案，已被作废**：
  它把车体坐标写成 `+X=前、+Y=左`（TRACK 载荷用 `ERROR_A/ERROR_B`）。修正版第 9 行明确
  "不得参考旧草案"，并统一为 `+X=左、+Y=前、+Yaw=逆时针`——**与本工程 2026-09-17 的坐标契约
  完全一致**，所以 TRACK 载荷到 `MecanumControl_MoveVelocity(vx=左, vy=前)` **不需要轴交换或取反**。
  由此**删掉了 V1.0 那套"图像 u/v → 车体轴"的翻转**（`error_x=x-320`、`vy=-f(error_x)`）：
  规范§11.1 明确方向换算由 Jetson 按相机安装完成，固件不得再自行翻转。别把这套翻转翻回去。
- 协议要点（全部有回归锁定）：请求 **6 字节** `66 TASK TARGET SEQ CRC8 77`、响应 **16 字节**
  `66 TASK STATUS SEQ PAYLOAD[10] CRC8 77`；`CRC-8` 为 **poly=0x07 / init=0x00 / 不反转**，
  请求覆盖前4字节、响应覆盖前14字节，自检值 `Check("123456789")=0xF4`。
  `SEQ` 由本端生成、每个新逻辑任务+1、连续跟踪期间不变，**同一逻辑任务重发必须复用同一 SEQ**
  （换 SEQ 会被 Jetson 当成新任务）；只接受 `task` 与 `seq` 同时匹配活动任务的结果，
  这**取代**了 V1.0 用 `received_tick >= s_start_tick` 近似配对迟到回包的做法。
  TRACK 载荷 = `TARGET_ID / COORD_MODE / VALID_FLAGS / ERROR_X(i16) / ERROR_Y(i16) /
  YAW_CDEG(i16) / CONFIDENCE`；`COORD_MODE` 分像素(0x01)与车体毫米(0x02)**两套系数**；
  **只有 `VALID_FLAGS` 置位的轴才驱动**（未置位输出0，不能当成"误差恰好为0"）；
  `unknown coord_mode` 整帧作废。时效 **300→150ms**（规范§17，丢帧时更早停车，属行为变化）。
  STATUS 扩到 `0x06 INVALID_DATA`，错误帧 `payload[0]=error_detail`。
- 实现：`vision.c/.h` 加 `Vision_Crc8()`（逐位移位、不查表）与 BE 读写、
  `Vision_SendRequest()` 内部管理 SEQ/待发/活动任务、`Vision_DecodeTrack()`、
  SCAN 的 INDEX 位图去重与 COMPLETE 校验（`Vision_GetBatch()`，规范§14 强制项）。
  **`ops.c` 那张 `s_crc8_table` 不能复用**：它是 poly=0x31/init=0xFF/**反射**的 DJI 变体
  （Check=0xA1），且是 `static`；两者是不同算法，改 init 也换不过来。
  `vision_track.c/.h` 按 `COORD_MODE` 选系数（像素 0.18 RPM/像素+`VDBPX`(默认12px) 死区；
  毫米 0.5 RPM/mm+`VDBMM`(**默认2mm**) 死区），输出钳 `VMIN`~`VMAX`（默认 8~60 RPM），vz 恒 0。
  **死区是2026-09-29按现场要求从10mm收到2mm的**：8RPM≈33.6mm/s、20ms周期走约0.67mm，
  2mm单侧余量容易被一帧过冲穿过；现场若出现目标附近"停→起→停"的极限环，先降 `VMIN`
  （如 `VMIN=3`），不要只把死区改回去。像素模式无法表达"2mm"——固件没有毫米/像素标定系数，
  需要毫米级死区就得让工控机用 `COORD_MODE=0x02`。
  **`VisionTrack_Start/Stop/Service/IsActive` 与 `VisionTrackOutput` 的签名刻意保持不变**，
  ACK 事件表 0..25 也未动 —— 因此 `debug_usart.c` 的仲裁层和
  `test_vision_control/test_parse_line_axes/test_debug_wheel/test_runtime_faults` 都不用改。
- 新增调参 `VCONF`/`VKPMM`/`VDBMM`/`VDBPX`/`VMIN`/`VMAX` 加在 `s_params[]` **末尾**
  （`Debug_SetParam`/`Debug_ReplyParam` 按 `sizeof` 遍历，自动可设置可回读）。
  **它们是 RAM 参数、不存 Flash**：Flash 记录是固定 `float value[16]` + `VERSION=2`，且有效性
  校验要求 `size == sizeof(DebugParamValues)`，扩字段必须升 v3 并迁移旧记录，属独立改动，本次刻意没做。
  **应答文本已接近缓冲上限**：`ERR PARAM UNKNOWN; GET ...` 那条现在是 **94 字节**，而
  `s_tx` 是 `4*24+4=100` 字节；`test_debug_wheel.py` 加了长度守卫（阈值96，余量只有2字节），
  **再加参数名会直接触发守卫失败**，届时要么精简该提示、要么扩大 `s_tx`。
- 测试：`Tests/hardware/test_vision_rx.py` 重写为 16 字节帧 + CRC-8 + SEQ，含规范§22 的
  5 个标准向量与速查表 15 条实测 CRC（覆盖全部 TASK×TARGET 与"黄/黑 CRC 碰撞"）。
  改动过程中该测试抓到一个真实偏移错误：SCAN 载荷是 `[0]TARGET_ID [1]INDEX [2]TOTAL`，
  批量组装原先按 V1.0 的字段序读成了 `frame[4]=INDEX`。
- 回归：`Tests/hardware/` 11 个套件全过；AC5 **全量重编**（51 个 C 文件）0 错误 0 警告，
  `Code=39460 RO-data=10400 RW-data=524 ZI-data=25156`。磁盘上唯一可比的历史构建日志是
  09-21 的 38764/10072/472/24536，**早于 09-27 的视觉功能本身，不是本次的同口径基线**。
- **未做/未验证**：未烧录、未上电、未与 Jetson 联调（本次只做静态与回放验证）。
  YAW 与 STABLE 位已解码但未接控制；批量任务没有命令触发，其状态机目前只被测试覆盖；
  `getBatch` 之外的 SCAN/QR 启动 API 也未接入调试命令。
- 已知语义缺口（规范也不要求，属设计取舍）：**没有"启动确认超时"**。若 Jetson 在跟踪期间重启，
  它会恢复成空闲态不再推流，而固件不会自动重发请求（§21："启动命令成功后不需要周期性重发"），
  于是底盘停在原地且只在 150ms 时效上表现为"没数据"，恢复需显式 `VTRACK=0` 再 `VTRACK=1`
  （换新 SEQ）。台架联调时按这条排查。

## 2026-09-29 Qt 比赛地图 A* 路径规划（仿真验证，当前有效）

- 需求：点地图/快速目标能自动算出绕开中央物料区的 **GOTO 航点路径**；这一阶段
  **只验证算法可行性，不接 STM32**（不下发任何命令）。
- 位置：算法在 `core.plan_path()`（纯函数、无 Qt），界面是 `main.py` 地图页**右侧固定宽度侧栏**
  `_build_plan_side()`（在 QHBoxLayout 里与地图并列，不挤占地图高度）；绘制用
  `FieldView.set_path()`（绿色虚线 + 航点圆点，z=11/12，在轨迹之上、车体之下）。
  **没有改固件、没有改协议、不需要烧录。**
- 算法要点：场地按网格离散（默认 `core.GRID_MM`=10cm），禁区按 `pad` 向外膨胀
  （默认 `core.CAR_INFLATE_MM`=15cm，把 28×26cm 车当质点）；8 邻域 A*，
  `g`=实际步长、`h`=到目标欧氏距离（可采纳⇒最优）；**斜向必须禁止切角**
  （`if di and dj and (blocked[i+di][j] or blocked[i][j+dj]): continue`）——不加这条
  时路径会贴着障碍拐角"削"进去，实测能侵入膨胀区约 41mm/h。最后用"视线可达"拉直
  （`simplify_path`），所以 5/10/20cm 网格给出的航点数与长度接近。
- **膨胀量不是精度、是模型**：13.0/14.0cm = 半宽/半长（只在已知航向时成立）、
  19.1cm = 半对角线（`core.CAR_HALF_DIAG_MM`，与航向无关）。中央物料区之间的走廊只有
  400mm 净宽，**膨胀 ≥200mm 就会把它们封死**：此时 `plan_path` 返回
  `ok=False, reason="没有可行路径（禁区把通路封死…）"`，界面照实显示，绝不硬穿
  （回归 `test_astar_reports_sealed_corridors` 锁住这条）。
- 规划结果另外给出**真实余量** `min_clearance`（`path_clearance()` 逐段采样到**未膨胀**
  真实障碍表面的最小距离）：≥半对角线=任意航向可过、≥半宽=沿走廊可过、<半宽=会刮。
  默认参数下走中间走廊约 180mm，正好卡在"能过但要直着走"这一档。
- 起点/目标落在**膨胀后**的禁区里（例如 原料区 快速目标离圆盘只有 90mm）时，
  `plan_path` 仍会出路径，但把 `start_in_inflate`/`goal_in_inflate` 报出来，界面提示
  "末端只能贴到附近"。目标点在**真实**禁区内则由界面层用 `field_point_blocked()` 直接拒绝。
- 界面约定（**这是本阶段最需要守住的一条**）：
  `plan_click_check` 勾选时，`_on_map_click()` 只调 `plan_to()`（计算+显示+列出 GOTO 文本），
  **一条命令都不进队列**；取消勾选才走原来的 `_goto_field()` 直接下发。
  回归 `test_map_click_plans_without_sending_anything` 断言点击后 `line_q`/`urgent_q` 全空。
- 「仿真沿路径行驶」`_toggle_follow()`：**仅 `self.sim is not None` 时可用**（模拟不是 STM32），
  200ms 一拍按航点逐个下发 GOTO，到位判据用固件上报的误差（`|devx|,|devy|<2cm 且 |devz|<3°`，
  刚下发那一拍先跳过以免用上一目标的误差误判），单航点 25s 超时即停；
  STOP/ZERO、断开、关模拟都会 `_clear_path()` 并停止。
  `test_planned_path_is_drivable_in_simulation` 在仿真里整条走完并断言全程车心不进入真实禁区。
- 版式（同日）：规划面板做成地图右侧 316px 固定宽度侧栏，地图因此拿回整页高度
  （此前它被上面几行面板挤到页面下半部）。新增 QSS 类 `#PlanText`（等宽航点台账）、
  `#PlanInfo`（结果卡，`[state=ok|warn|bad|idle]`）、`#PlanBadge`（「不下发 STM32」标记）；
  结果卡配色由 `_plan_state()` 决定：余量 < 半宽 → `bad`、< 半对角线或起/终点落在膨胀区内
  → `warn`、否则 `ok`。`FieldView` 场景留白由 ±180 收到 ±150（刻度文字画在 -80/-100，仍够），
  场地在视图里更大。
- 回归：`test_serial_lifecycle` + `test_debugger` 共 74 用例全过；`core.selftest()` 增加
  第 [6] 步（直线=2 航点、绕行各段干净、膨胀 400 判不可达）。未连硬件、未烧录。
- 下一步（未做）：车体膨胀后的 `field_path_blocked` 用于**下发前**的整条路径检查、
  以及把规划结果真正接进 STM32（需要先决定是逐航点 GOTO 还是固件侧路径跟随）。

## 2026-09-27 Qt 比赛地图车体图标与航向口径修正（当前有效）

- 需求：地图小车从"圆点 + 方向线"改为长 28cm、宽 26cm 的车体轮廓，航向变化要看得出来。
- 根因（本次一并修正）：`_ops_to_field` 的线性部分是 `(s·dx−c·dy, −c·dx−s·dy)`，行列式 **−1**
  —— 地图绘制帧是统一坐标系的**镜像旋转**，旧代码 `set_pose(map_theta + v[2])` 把航向当角度
  直接相加，车头必然错。仿真可复现：按 W 前进，车点向上走、箭头却指向下。
- 车头方向的唯一依据是固件 `mecanum_control.c:317-322` 的 `body = R(zangle)·世界误差`，
  即 `世界 = R(−zangle)·body` ⇒ **车头在世界帧 = (sin z, cos z)、世界角 = 90°−z**，
  车左 = (cos z, −sin z)。四处独立佐证一致：该旋转公式、"ZERO 后 +Y=当时车头 / +X=当时车左"、
  模拟器 `make_frame` 的手动运动学、以及 `test_simulator_heading_and_zero_physical_axes`。
- 据此**同时修正四处**（用户已确认）：① 地图车体图标；②「航向=…（0°左/90°上）」数字；
  ③ 图例口径（保持原文，现在才真正成立）；④「目标航向=场地N°」下拉框。核心是互逆的
  `_field_heading(z)=90−z−map_theta` 与 `_field_to_body_yaw(F)=90−map_theta−F`。
  **地图航向数字是顺时针增大的**（0°左→90°上→180°右→270°下），与固件 z 的逆时针相反，
  这是有意保留的地图口径，不要再当 bug 翻回去。
- 实现：`core.CAR_LENGTH_MM=280`/`CAR_WIDTH_MM=260`；`FieldView.set_pose(cx,cy,nose,left)`
  改吃**绘制帧单位向量**（四角 = 车心 ± 车长/2·nose ± 车宽/2·left），新增 `car_nose` 楔形
  （前缘两点 + 车心），删除原 `dir_item`；`MainWindow._ops_dir_to_map()` 提供与轨迹同源的线性
  变换，`_body_nose_left()` 给出车头/车左。方向不再经过任何角度约定，镜像帧问题从根上消除。
- 回归：`test_serial_lifecycle` + `test_debugger` 共 64 用例全过（新增
  `test_car_icon_is_scaled_car_body`、`test_map_heading_readout_matches_icon`、
  `test_field_target_yaw_is_inverse_of_readout`、`test_car_icon_basis_matches_actual_travel`）。
  守卫已验证会咬人：注入旧的"世界角=z"约定 → 三个用例失败；车左取镜像 → 位移用例失败；
  车长车宽对调 → 三个用例失败。
- 注意：`QGraphicsPolygonItem.boundingRect()` 对"旋转 + 宽笔迹"的外扩**不对称**（实测 3px 笔
  在 15° 时上下 1.449 / 左右 1.837，中心差 0.19mm），取车心必须用顶点均值，不能用它。
- 只动上位机 `core.py`/`main.py`/`test_debugger.py`，固件与协议未变、无需重新烧录；未连硬件。
- 仍需台架/实车确认一次：真机 ZERO 后按 W，地图上车应向上走且车头朝上、数字显示 90°（map_theta=0）。

## 2026-09-23 UART4 实车失动排查（当前有效）

- 实车反馈：改为 UART4 中断队列后，键盘/GOTO 不动，`WHEELOFF` 也未让四轮失能；
  上电锁轴，上位机未显示 `ERR WHEEL TX FAILED`。这说明问题需要沿 UART4 出线和
  驱动器收帧继续实测，不能仅凭 HAL 提交成功认定驱动器已执行。
- 已发现并修复确定的队列错误：停止/失能安全帧优先于速度帧出队，但旧速度帧原先
  仍留在队列里，会在安全帧之后发送并重新使能；现在停止/失能入队时清除同地址
  尚未发送的旧速度。此错误解释失能后再锁轴的风险，不单独证明所有运动失效的原因。
- 旧版阻塞发送每帧后有 `HAL_Delay(1)`，新队列原先在 TC 后立刻发下一帧。
  为排除驱动器需要帧间空闲的可能，现由 TIM7 毫秒中断在 TC 后等待至少 1ms
  再启动下一帧，控制任务不等待；启动时的初始四轮命令也由 TIM7 推进。
  这推翻下文“无需帧间延时”的未上电推断，但是否是失动根因仍待悬空实测。
- `HAL_UART_Transmit_IT` 返回及 TX 完成中断只证明 MCU 发送流程完成，不能证明
  PA0 波形、接线、电机参数或驱动器执行。未烧录这次修复，需先核对新 HEX 与
  UART4 PA0 实际帧，再在悬空状态验证 `WHEELOFF`/`WHEELEN`、单轮和手动运动。

## 2026-09-23 UART4 改非阻塞发送队列（当前有效）

- 背景：轮速下发原先每条命令都等 TC 再 `HAL_Delay(1)`，四条命令约占控制周期 6.8ms，
  串口抖动时还要叠加 `ZDT_X42S_Send` 的 10ms×3 次重试（最坏四帧 ≈128ms 阻塞）。
- **安全语义按方案 (a) 保留**：发送失败仍然要取消运动、尝试失能、锁存到显式 `WHEELEN`。
  这是本次改造的硬约束，不能因为改成非阻塞就丢掉。
- 实现：`ZDT_X42S_InitTx()` 注册 TX 完成回调并复位队列；控制任务只把帧压入固定长度队列
  （16 帧 × 8 字节），由 `HAL_UART_Transmit_IT` 的完成回调链式推进；完成回调里
  **必须清 `s_tx_active_valid`**，否则 `TxKick` 会把同一帧反复重发（这个 bug 是
  重写后的 `test_zdt_tx.py` 抓出来的）。同地址速度帧覆盖尚未发送的旧帧；使能/失能/停止
  帧不参与覆盖且始终优先。队列只用 192 字节 RAM，没有采用被回退版本的
  per-address 门控数组（`s_motor_enabled[256]`+`s_motor_ready_tick[256]` 共 1.5KB）——
  失能/使能闸门本来就由 debug 层的 `Debug_WheelReady()` 与 `WHEELEN` 状态机负责，
  在驱动层再存一份会成为第二个真相来源。
- **失败上报刻意不依赖 UART 错误标志**：过载(ORE)等是接收侧错误，总线噪声不该把整车
  锁存成故障。判定只有两个来源，且都在任务上下文完成，因此 `s_tx_error_count` 只被任务
  读写、不需要 ISR 同步：① 单帧占用总线超过 `ZDT_X42S_TX_SLOT_TIMEOUT_MS`(10ms，
  正常 8 字节 @115200 只要 0.7ms)；② 同一帧累计超过 `ZDT_X42S_TX_RETRY_BUDGET_MS`(1s)。
  超预算时**丢弃该帧并计一次失败**，避免它永久占住队首把后面的安全帧一起堵死。
- 队列满、`InitTx` 之前提交，也会计一次失败 —— 不静默丢弃，让次序错误暴露出来。
- 接口语义变化：`ZDT_X42S_Enable/Disable/Stop/Speed/SpeedAcc` 的返回值现在只表示
  **"是否已入队"**，不代表 HAL 已发出、更不代表电机收到；发送结果看
  `ZDT_X42S_GetTxErrorCount()`。`MecanumControl_*` 是 void，不受影响。
- 接线：`main.c` 在 `OPS_Init()` 之后依次调用 `ZDT_X42S_InitTx()` / `ZDT_X42S_InitRx()`
  （调度器启动前没有任务推进队列，靠完成中断链式发送，顺序不能反）；
  `debug_usart.c` 的 `Debug_ServiceZdtReplies()` 在 `ZDT_X42S_ServiceRx()` 之后
  增加 `ZDT_X42S_ServiceTx()`。
- **`Debug_ServiceWheelFault` 一行未改**：它本来就靠"`GetTxErrorCount` 是否变化"锁存，
  与错误怎么产生解耦，所以 `test_runtime_faults.py` 也不需要改（它把那两个函数桩掉了）。
- 被回退分支的教训：`809932a` 的队列实现里 ERROR 回调只置 `s_rx_fault`，
  **完全没有上报 TX 失败**（只有无访问器的 `s_tx_drop_count`），等于把上面那条安全契约
  弄丢了。这很可能就是它被 `git reset` 回退的原因；因此本次是**在主线之上重做**，
  不是把它捡回来。
- 连带修改：`Tests/hardware/test_zdt_tx.py` 全部重写（原子 `ZDT_X42S_Send` 已删除），
  覆盖组帧逐字节、同地址覆盖、安全帧优先、队列满、`InitTx` 前提交、在途超时只中止 TX、
  超预算丢弃、正常完成不误判；并含结构守卫"TX 区域不得出现 `HAL_Delay` 或阻塞式
  `HAL_UART_Transmit(`"（守卫先剥掉注释，避免命中说明文字）。
- 尺寸（AC5 全量重编，0 错误 0 警告）：相对档0+档1 的 36144/452/24572 →
  Code 36800、RW-data 460、ZI-data 24764，即队列净增约 656 字节 Code + 200 字节 RAM。
- 回归：`Tests/hardware/` 23/23、C 回归、Qt selftest 与 60 用例全通过。
  **未上电实测**：四个驱动器在连续饱和发送下是否丢帧、以及 TX 超时/重试预算的
  实际触发时机，都必须在悬空状态下确认。

## 2026-09-23 FreeRTOS 任务精简与 UART4 帧间延时（当前有效）

- **任务数从 3 个降到 2 个。** `configUSE_TIMERS` 置 0 后内核不再创建 `Tmr Svc`
  任务（本工程从未使用任何软件定时器或事件标志），只剩 `defaultTask` 与内核 `IDLE`。
- **四个宏必须同进同退**，否则分别撞到四处编译错误（都已实测）：
  `configUSE_TIMERS=0`、`configUSE_OS2_TIMER=0`（`CMSIS_RTOS_V2/freertos_os2.h:223`）、
  `INCLUDE_xTimerPendFunctionCall=0`（`FreeRTOS/Source/timers.c:41`）、
  `configUSE_OS2_EVENTFLAGS_FROM_ISR=0`（`freertos_os2.h:207`）。
  只关闭 CMSIS-RTOS2 包装层，不影响裸 FreeRTOS API；ISR 本来也不调用任何 RTOS API。
  改回时四个都要翻回 1。
- 收益（`unify_builder --rebuild` 实测）：Code 39184→36144、RW-data 472→452、
  ZI-data 24920→24572；`prvTimerTask`/`xTimerCreateTimerTask`/`osTimerNew`/
  `xTimerPendFunctionCall` 在 `.map` 中各出现 0 次。**省的主要是 flash 和一个调度任务，
  RAM 只收回了 348 字节**，原因见下一条。
- **已知未收回：1116 字节死内存（这条是本次实测才发现的反直觉点）。**
  `cmsis_os2.c` 的弱函数 `vApplicationGetTimerTaskMemory` 只受
  `configSUPPORT_STATIC_ALLOCATION` 守护、**不看 `configUSE_TIMERS`**，所以
  `Timer_TCB`(92B) 与 `Timer_Stack[configTIMER_TASK_STACK_DEPTH]`(256 字 = 1024B)
  仍然占着 `cmsis_os2.o(.bss)`（`.map` 的 Data 行可见）。链接器只删掉了该函数体 24 字节：
  这段 `.bss` 与仍在使用的 `Idle_TCB`/`Idle_Stack` 同属一个节，ARM 链接器按节回收，
  无法只丢掉其中两个死符号。两条回收路线，**都还没做**（128KB SRAM 目前只用 19.1%，
  暂不值得为此引入风险）：
  ① `configSUPPORT_STATIC_ALLOCATION=0`：整块 1720B `.bss` 消失，idle 任务改从 heap 取
  （约 604B 堆），净省静态 RAM，但等于换了内核分配模型；
  ② 调小 `configTIMER_TASK_STACK_DEPTH`（现在只被这个死数组使用）：省约 1KB，
  但重新开启定时器时必须记得改回 256 —— **有坑，不推荐**。
- **`SetMotorVoltageAndDirection()` 删掉每条命令后的 `HAL_Delay(1U)`。** 依据：
  `HAL_UART_Transmit` 阻塞模式返回前无条件等待 `UART_FLAG_TC`（见
  `stm32f4xx_hal_uart.c` 该函数末尾），末位停止位发完时线路已空闲，四帧本就严格串行，
  不存在粘包。原先白付约 4ms/周期，串口抖动时还要叠加 `ZDT_X42S_Send` 的重试超时。
- 连带影响与守卫：`Tests/hardware/test_mecanum_mixer.py` 不再提供 `HAL_Delay` 桩，
  **若固件重新引入帧间延时，该测试会以 `-Werror=implicit-function-declaration` 拦下**
  （已用注入方式验证过会失败）。这与既有 `must_not` 文本锁是同类护栏。
- 未做：未上电实测，需在悬空状态下确认四个 ZDT 驱动器在高帧率下均不丢帧。
  （非阻塞发送队列已于同日单独完成，见上一条。）
- 回归：`Tests/hardware/` 23/23、C 回归、Qt selftest 与 60 用例全通过；
  AC5 全量重编 0 错误 0 警告。未烧录。

## 2026-09-22 底盘参数文字回读 `GET 名称`（当前有效）

- 背景：`XVMIN`/`ZVMIN` 只在 `s_params` 里有写入路径，遥测 `data[0..23]` 没有它们
  的通道位，Qt `core.CHASSIS_PARAMS` 第6字段因此是 `None`，「回读」栏恒显示「回读 —」。
- 固件 `debug_usart.c` 新增 `Debug_ReplyParam` 与 `GET <名称>`：回读参数表指向的
  实时 RAM 值，应答固定 `"<名称>=<值>\r\n"`（三位小数、名称回显大写），未知名回
  `ERR PARAM UNKNOWN`（`s_ack_text` 事件19；动态文本为事件20，存 `s_ack_param[16][24]`
  按队列槽双缓冲，组包与入队在同一临界区）。
- **24 通道 100 字节 JustFloat 帧格式、通道数和 `DebugUsart_Init` 行为均未改变**；
  应答复用 ZDT/Flash 的文字队列，DMA 空闲时优先发送，每帧最多占一条文字应答。
- 不用 `printf("%f")`：Keil 精简库不带浮点格式化，按 0.001 定点输出。
- Qt 侧：`core.PARAM_ECHO_RE`/`take_params()` 把参数行从日志文本里分流（日志去重
  不适用于状态量），`SerialWorker(param_q=...)` 转发，`main._apply_param_readback`
  刷新对应行「回读」栏；连接后每秒轮流回读一个无通道参数（`core.PARAM_POLL_S`），
  参数行发送后立刻补读一次。模拟器同样应答 `GET`。
- 只改了 XVMIN/ZVMIN 的可观测性，不去改它们的控制语义；`OPSOFFSET`、锁轴、DM
  仍无状态回读。
- 回归：`Tests/hardware/test_param_readback.py`（编译真实函数）、
  `Tests/hardware/test_parse_line_axes.py` 与 `test_debug_wheel.py`（应答表下标+1）、
  Qt `python -m unittest test_debugger`。未烧录、未做实车验证。

## 2026-09-21 塔吊地址分配（当前有效）

- 35升降节点ID=1，扩展帧0x100/0x101；28伸缩节点ID=2，扩展帧0x200/0x201。
- DM默认ID=3，MIT标准帧0x003，位置速度标准帧0x103。DMID仍可显式修改并保存。
- Flash记录升为v2；读取v1时只将DM地址迁移为3，保留其他调参值，再由自动保存提交v2。
- Qt地址显示、默认DMID与模拟器同步。该配置不会自动改写实体驱动器的节点地址。

## 2026-09-20 SPI Flash 参数持久化（当前有效）

- `Hardware/spi_flash.c/.h` 独立管理天空星W25Q128：PA4片选、PA5/6/7=SPI1 AF5，Mode0、5.25MHz。
- `Hardware/debug_param_store.c/.h` 使用最后8KB双扇区追加日志、CRC32及最后提交标记；不得与字库等共用此区域。
- 默认任务对底盘7项参数、OPS安装X/Y和DM七项数值自动保存，稳定2秒后开始，成功输出`ACK PARAM SAVED TO FLASH`。
- `DebugUsart_Init`启动RX前恢复；不恢复运动/使能请求、ZERO原点。Flash失败只报错，本次启动停止写入，调参及控制继续。
- `.ioc`已登记引脚；实际GPIO/SPI初始化由`SPIFlash_Init`负责，再生成CubeMX后需检查初始化归属。
- 说明及验收：`Hardware/Flash参数保存说明.md`。回归：`test_spi_flash.py`、`test_param_store.py`、`test_param_integration.py`。

## 2026-09-19 审查修复（当前有效）

- 统一世界→车体旋转为 `body_x=c*devx-s*devy`、`body_y=s*devx+c*devy`；
  与 OPS ZERO 一致。+90° 时目标世界+X应输出车体前进。
- OPS RX恢复和会话事件在 `DebugUsart_Send()` 每周期处理。相同会话号时间戳
  回退/长失联也取消旧GOTO；重复帧不刷新在线时间；IMU_REBASED锁存位置失效。
- GOTO整条校验通过后才提交目标；非法航向/尾随字符不改变旧运动状态。
  STOP消费、目标快照和到位取消使用短临界区，目标代次防止误取消新GOTO。
- ZDT公开接口返回HAL提交结果；任务检测发送故障后取消运动并尝试失能，
  最多重试1秒，故障锁存到显式WHEELEN；这不等于物理电机ACK确认。
- DM失能绑定原ID并重试最多1秒；失败报错并阻止再使能，DMOFF显式重试。
  模式/使能必须等待失能提交成功；不能在活动/失能待处理/故障期间改DMID。
- UART1遥测TX超过100ms仍忙时仅AbortTransmit，不中断RX。
- 默认任务栈2048字节，启用溢出检测，绝对周期50Hz；超期跳过旧周期。
  调试变量 `default_task_stack_free`（字节）、`default_task_overruns`。
  位置环斜坡1000控制单位/s，到位持续220ms；dt上限100ms。
  轮子使能100ms等待由任务状态机完成；UART4发送本身仍为有限阻塞。
- Qt手动运动和ZERO后的坐标按同一旋转约定；模拟器支持GOTOHOLD。
  模拟器是界面演示，不替代真实控制器闭环回归或实车验收。
- 配套OPS工程 `E:/STM32/ILHC/ops9-main/code` 已修复DIR分段脉冲遗漏，
  IMU掉线后冻结位置并锁存无效，须显式复位OPS重建坐标（ZERO不能解除）。
  启动代次存于BKP DR1/DR2/DR3；无VBAT完全掉电时接收端用时间戳/失联兜底。
- 本轮不烧录、不发送运动命令。两块MCU都需更新固件。
  详细行为和验收见 `修复说明_20260919.md`。

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
- OPS 原始帧到统一坐标固定为 `X=-raw_x`、`Y=raw_y`，不保留方向模式或分支；
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
支持 flags、session_id、拆包重同步和错误回调任务级重挂。初始化先发
`C5 22`，等待 OPS 重启后发 `C5 32` 固定新协议方向 2；不再发送 `C5 30`。V2 session 变化会取消
旧 GOTO 并重设本地原点参考。回归入口：`Tests/hardware/test_ops_protocol.py`。

2026-09-19 ZERO 坐标系修复：`OPS_ZeroCoordinates()` 继续保存 ZERO 时的
OPS 原始位置和 `s_origin_yaw`。`OPS_CopyPosition()` 在完成安装偏心补偿并映射到
统一世界坐标后，使用 `x0=c*world_x-s*world_y`、`y0=s*world_x+c*world_y`
（`c=cos(s_origin_yaw)`、`s=sin(s_origin_yaw)`）把世界位移旋转到 ZERO 车体轴。
因此 ZERO 后 `+Y` 表示 ZERO 时的车头，`+X` 表示当时车左。新增回归入口
`Tests/hardware/test_ops_zero_frame.py`。

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

`Core/Src/freertos.c` 当前只有一个显式创建的业务任务 `defaultTask`，正常优先级，栈 2048 字节，已启用栈溢出检测（`configCHECK_FOR_STACK_OVERFLOW=2`，溢出钩子直接 `NVIC_SystemReset()`）。循环采用绝对周期 50Hz：`DebugUsart_Send()` 之后 `osDelayUntil` 到下一个绝对时刻，超期不补跑、只累加 `default_task_overruns`，`default_task_stack_free` 记录栈余量供调试器读取。除 `defaultTask` 外只剩内核 `IDLE` 任务（2026-09-23 起 `Tmr Svc` 已随 `configUSE_TIMERS=0` 移除，见文首该条）。RTOS tick 为 1kHz，使用 `heap_4`，堆配置为 15360 字节。

`DebugUsart_Send()` 不只是发送函数：它还处理 STOP/ZERO、GOTO 位置控制、DM 模式切换和周期控制、主机失联保护。移除或降低其调用频率会同时改变运动控制行为。循环按绝对周期调度：单次执行小于 20ms 时周期稳定在 50Hz；一旦超过 20ms 则跳过旧周期、按实际耗时运行并计入超期，不补跑。超期的主要来源仍是 UART4 阻塞发送（四条轮速命令，最坏每帧 10ms×3 次尝试）。

HAL 毫秒时基由 TIM7 中断和 `HAL_TIM_PeriodElapsedCallback()` 维护；RTOS 使用自己的系统 tick。TIM1 PWM 和 TIM6 已初始化，但当前业务代码没有启动它们来驱动控制任务。

## 4. 外设映射与中断归属

| 外设 | 引脚 / 通信参数 | 当前用途及接收方式 |
| --- | --- | --- |
| USART1 | PA9 TX、PA10 RX；115200、8N1 | 调试；RX DMA2 Stream2 Ch4 + IDLE，TX DMA2 Stream7 Ch4 |
| USART2 | PD5 TX、PD6 RX；115200、8N1 | OPS；RX DMA1 Stream5 Ch4 + IDLE，命令阻塞发送 |
| USART3 | PB10 TX、PB11 RX；115200、8N1 | 视觉工控机（Jetson）；RX 逐字节中断收16字节响应，TX 中断发6字节请求 |
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
- `OPS_ZeroCoordinates()` 在本地同时记录 OPS 原始 X/Y 原点和当前航向；不重置 OPS 本体。
  `OPS_GetPosition()` 先完成偏心补偿和 raw→unified 映射，再平移并按 ZERO 航向旋转 X/Y，
  同时令 Z 相对 ZERO 航向。`OPS_ClearZero()` 恢复绝对 X/Y/Z。
- `OPS_Init()` 发送 `0xC5 0x22` 复位，等待 OPS 重启后发 `0xC5 0x32`
  固定新协议方向 2；不再发送 `0xC5 0x30`。该过程包含启动等待，不是可在中断里调用的轻量操作。USART2 错误恢复由
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
- DMA 发送缓冲区在传输完成前不能重写；新增任务或增加局部缓冲区时核对默认任务 2048 字节栈及 RTOS 堆。
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
