# ILHC Control Station v2.0 (Qt)

基于原 `ILHC_debugger_v1_1_optimized.py` 的 PySide6 UI 重构版。

## 技术栈

- PySide6 / Qt Widgets：主 UI
- PyQtGraph：实时波形
- QGraphicsView：比赛场地地图
- pyserial：USART 通信
- NumPy：环形缓冲与数据处理

## 保留的 v1.1 核心

- 24 × float + `00 00 80 7F` JustFloat 遥测解析
- payload 内嵌 `+inf` 帧尾防误判
- 50 Hz 遥测
- STOP / DMSTOP / DMOFF 高优先级队列
- GUI 主线程 5 Hz 心跳 PING
- 断开/关闭时 best-effort `STOP -> DMSTOP -> DMOFF`
- 遥测延迟/超时状态
- 模拟模式
- CSV 数据记录
- 比赛场地静态禁区与直线路径检查

## 安装

建议 Python 3.10+：

```bash
python -m pip install -r requirements.txt
```

## 运行

模拟模式：

```bash
python main.py --simulate
```

真实串口：

```bash
python main.py --port COM6 --baud 115200
```

普通启动后也可以在顶部选择串口再点“连接”。

## 自检

核心逻辑自检不需要打开 Qt 窗口：

```bash
python main.py --selftest
```

或者：

```bash
python -c "import core; core.selftest()"
```

## 文件结构

```text
ILHC_Qt_v2/
├─ main.py          # PySide6 UI + 应用控制层
├─ core.py          # 协议/串口/模拟器/环形缓冲/CSV/安全几何
├─ style.qss        # 深色工业风主题
├─ requirements.txt
├─ README.md
├─ install_windows.bat
├─ run.bat
└─ run_simulate.bat
```

## 说明

当前地图安全功能与 v1.1 一样：阻止目标点落入固定设备以及明显的直线穿越，但**尚未加入车体半径膨胀和 A* 自动绕障**。要做完整路径规划，需要确定小车宽度/长度或中心到最外轮廓半径。


## Windows 快速启动

首次使用双击 `install_windows.bat` 安装依赖。之后：

- `run_simulate.bat`：无硬件演示
- `run.bat`：正常启动后选择 COM 口
