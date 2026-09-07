@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo [ILHC] 安装 PySide6 / PyQtGraph / pyserial / numpy ...
python -m pip install -r requirements.txt
if errorlevel 1 (
  echo.
  echo 安装失败，请检查 Python 与网络。
  pause
  exit /b 1
)
echo.
echo 安装完成。
pause
