# v2.1.2 地图点击修复说明

## 故障与复现

修复对象：上一版完整工程 `ILHC_Qt_v2_Astar_fixed_v2_1_1.zip`。
该版本 `main.py` 的 `_request_plan()` 调用了 `threading.Event()`，但模块顶部没有 `import threading`。
地图点击或快速目标触发规划时，在后台计算提交之前抛出：

```text
NameError: name 'threading' is not defined
```

旧版 `_prepare_plan()` 会先清除旧计划，因此界面只留下“未规划 / 旧计划失效”，没有显示新错误。
这不是需要取消保护、缩小车体或修改网格才能解决的问题。
原始复现输出见 `click_failure_v2_1_1.log`。

## 本次改动

1. `main.py` 显式导入 `threading`；它是 Python 标准库，不增加 pip 依赖。
2. 对规划准备、Event 创建、线程池提交、后台结果以及结果处理分别保护。
   异常会清除不完整计划，在右侧状态和地图提示区显示异常类型及原因。
   原始 traceback 保存到工程的 `logs/ilhc-planner-error.log`。
   日志目录不可写时保留界面提示，并在命令终端记录原因。
3. 过时请求的异常不会清除新的有效计划。取消时清理取消标记引用。
4. 修正 `tests/headless_support.py`：只执行 main.py 实际存在的导入，
   不再由测试辅助器额外注入 threading 等全局变量来掩盖生产代码漏导入。
5. 添加 16 项地图点击逻辑回归，并添加 4 项真实 Qt 鼠标/信号/定时器测试入口。
   后四项本环境没有执行，不能视为通过。

不改变 A*、地图、安全裕量、车体尺寸、坐标、旧串口协议和实车执行门控。
没有新增第三方依赖；`requirements.txt` 与 v2.1.1 相同。

## 用户操作

建议解压到新的目录，双击 `run_simulate.bat`。
标题应显示 **v2.1.2 — 地图点击修复**。
已经正常运行 v2.1.1 的 Python 环境不用因为这次更新重新安装依赖。
进入比赛地图，保持“点击地图 = 规划路径”和目标航向“保持当前”，点击“暂存区”或可达空白区域。
先应显示“正在规划”，随后显示路径和航点；这一步仍不会发车。
点击障碍物或不可达目标会明确显示失败，而不是静默退出。
“原料区”旧目标过近被拒绝是原有整车几何检查，不是本次点击故障。

仅修复当前漏导入的最小改法是在旧 `main.py` 顶部导入区加 `import threading` 后重启。
但推荐使用完整 v2.1.2，以同时获得异常可见性和回归修复。

## 验证边界

实际执行：原54项安全测试 + 新16项地图点击逻辑测试，共70项通过；6组核心自检通过。
控件是桩对象，调用的方法及全局导入取自实际 main.py，计算线程、碰撞模型和模拟器是真实实现。
这不是完整Qt窗口集成测试，也不是Windows或实车验收。

本环境缺少 PySide6 / PyQtGraph / pyserial，依赖安装失败，未能运行完整Qt测试。
已提供 `test_map_click_qt.py`：真实 QTest 鼠标事件 -> FieldView 信号 -> 后台规划 -> Qt定时器 -> 绘制结果。
在安装依赖的电脑上运行 `run_tests.bat`（或 `python run_tests.py --qt`）进行完整Qt验收。
不连接实车，不打开真实串口，不解锁STM32自动导航。

## 为什么上一版测试通过却漏了这个问题

旧 headless_support.py 把测试自身导入的 threading 放进被测方法的 globals，
导致缺 import 的 main.py 在测试中仍能正常访问 threading。
本版从生产源文件提取并执行真实导入，增加删除 threading 导入的变异检查：
再次人为制造该错误时必须显示 NameError、清除计划且不发运动命令。
旧54项通过记录放在 `history_v2_1_1` 仅作历史证据，不能替代本版实际测试。

## 参考

- Python 官方 threading 文档：https://docs.python.org/3/library/threading.html
- Qt for Python QTest 文档：https://doc.qt.io/qtforpython-6/PySide6/QtTest/QTest.html
- Qt for Python QSignalSpy 文档：https://doc.qt.io/qtforpython-6/PySide6/QtTest/QSignalSpy.html
