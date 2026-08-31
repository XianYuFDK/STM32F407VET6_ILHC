@echo off
rem ILHC 调试上位机启动脚本（需已安装 Python 3.8+ 及 pyserial / numpy / matplotlib）
chcp 65001 >nul
cd /d "%~dp0"
python ilhc_debugger.py %*
if errorlevel 1 pause
