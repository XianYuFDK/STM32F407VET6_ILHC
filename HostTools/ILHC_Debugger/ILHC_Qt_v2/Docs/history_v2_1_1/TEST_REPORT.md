# 实际测试记录

## 已执行

环境：Python 3.13.5，Shapely 2.1.2，numpy已安装。

- 所有Python源文件 `py_compile`：通过，清单见 `syntax_check.json`。
- `python main.py --selftest`：6组核心自检通过。
- `python -m unittest tests.test_navigation_safety -v`：54项通过。其中一项包含40组固定随机种子的独立Dijkstra对照；不是把40组计成40个独立单元测试。
- 测试覆盖原审查的首尾接入、跨格碰撞、出界、净空、停止后继续运动、旧误差到位、计划失效与仿真门控，并新增白名单孔洞/非凸区域、固定姿态整车、后台取消、旧会话重放和队列取消竞态。
- 无GUI端到端测试使用真实Simulator逐步运动和真实MainWindow方法，每帧检查整车扫掠，直到暂存区目标完成。

原始最终运行输出：`test_headless.log`。

## 没有执行，不能视为通过

- **完整Qt窗口/渲染/事件集成回归**：本环境未安装PySide6、PyQtGraph、pyserial。尝试pip安装时DNS不可用，无法获取依赖。`python run_tests.py --qt`明确返回2，不虚报测试通过；记录见 `test_qt_environment.log`。
- Windows `.bat` 实际双击启动。
- 真实串口、STM32固件和实体小车运动。
- 实际灰色车道合法性与设备摆放测量。

`tests/headless_support.py` 是直接从main.py抽取真实类方法并搭配控件桩；它验证逻辑，不是Qt图形测试，也不是机械安全认证。原Qt测试已按新契约更新并保留在包内，请在有完整依赖的电脑运行 `run_tests.bat`。
