# -*- coding: utf-8 -*-
"""Run offline checks; --qt additionally requires/executes the full Qt suites."""
from __future__ import annotations
import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qt',action='store_true',help='Also run the full offscreen Qt tests')
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    commands=[[sys.executable,'main.py','--selftest'],
              [sys.executable,'-m','unittest','tests.test_navigation_safety',
               'tests.test_map_click_regression','tests.test_navigation_audit_fixes',
               'tests.test_arc_smoothing','tests.test_trajectory','tests.test_continuous_tracking',
               'tests.test_competition_simulation','-v']]
    if args.qt:
        missing=[name for name in ('PySide6','pyqtgraph','serial','shapely','numpy')
                 if importlib.util.find_spec(name) is None]
        if missing:
            print('无法运行Qt回归，缺少：'+', '.join(missing),file=sys.stderr)
            print('请执行 python -m pip install -r requirements.txt',file=sys.stderr)
            return 2
        commands.append([sys.executable,'-m','unittest','test_debugger','test_serial_lifecycle','test_map_click_qt','-v'])
    env=dict(os.environ,QT_QPA_PLATFORM=os.environ.get('QT_QPA_PLATFORM','offscreen'))
    for cmd in commands:
        print('\nRUN: '+' '.join(cmd),flush=True)
        rc=subprocess.run(cmd,cwd=root,env=env,check=False).returncode
        if rc: return rc
    print('\n完成。'+('含Qt回归。' if args.qt else '本次仅无GUI测试；完整Qt验收请加 --qt。'))
    return 0

if __name__=='__main__':raise SystemExit(main())
