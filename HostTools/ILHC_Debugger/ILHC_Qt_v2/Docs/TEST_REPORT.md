# v2.1.2 实际验证记录

环境：Python 3.13.5，Shapely 2.1.2，NumPy 2.3.5。

## 已执行

- 对v2.1.1仅修正测试辅助器导入方式，运行真实 `_request_plan()`：复现 `NameError: name 'threading' is not defined`。
- 加入真实 import threading 和规划异常可见性修复后，运行 `python run_tests.py`。
- 6组核心自检通过。
- 70项无GUI回归通过（54项已有安全测试＋16项地图点击回归）。
- Python源文件语法检查：详见 `syntax_check.json`。
- 全部日志原件：`test_headless_v2_1_2.log`；故障复现：`click_failure_v2_1_1.log`。

无GUI测试使用控件桩，但主窗口方法和模块导入均来自实际main.py，不再额外注入缺失threading。
规划线程、碰撞检测和模拟器使用实际实现。测试断言地图规划不发送运动命令。

## 未执行

- 完整Qt窗口、绘制、鼠标事件及定时器集成测试。
- Windows BAT双击启动。
- 实际串口、STM32和车辆运动。

原因：环境缺少PySide6、PyQtGraph、pyserial；两次pip依赖安装均未成功。
提供了 `test_map_click_qt.py` 和 `python run_tests.py --qt`，没有把未执行测试标为通过。
本次未改变算法、车体参数、地图、坐标变换或实车门控。
