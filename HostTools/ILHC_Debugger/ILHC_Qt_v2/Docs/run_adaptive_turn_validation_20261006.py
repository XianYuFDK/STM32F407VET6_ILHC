"""v2.1.17离线串行验收；不连接用户串口，不使用真实记录目录落盘。"""
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

QT=Path(__file__).resolve().parents[1]
REPO=QT.parents[2]
STATUS=QT/'Docs/adaptive_turn_validation_status_20261006.json'
ENV=dict(os.environ,PYTHONIOENCODING='utf-8',PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',
    QT_QPA_PLATFORM='offscreen',PYTHONPATH=str(QT),ILHC_RUN_LOG_DIR=str(QT/'records/offline_validation_20261006_v217'))
JOBS=(
    ('headless',QT,['run_tests.py']),
    ('qt',QT,['-m','unittest','test_debugger','test_serial_lifecycle','test_map_click_qt',
        'tests.test_dm_position','tests.test_hardware_trajectory_qt','tests.test_right_crane_qt',
        'tests.test_trajectory_settings_qt','tests.test_run_journal_qt','tests.test_wireless_telemetry_qt','-v']),
    ('legacy_c',REPO,['Tests/hardware/test_trajectory_buffer.py']),
    ('native_commands',REPO,['Tests/hardware/test_parse_line_axes.py']),
    ('native_bridge',REPO,['Tests/hardware/test_trajectory_bridge.py']),
)


def main():
    state=dict(pid=os.getpid(),complete=False,jobs={})
    def save():STATUS.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
    folder=Path(ENV['ILHC_RUN_LOG_DIR']);folder.mkdir(parents=True,exist_ok=True)
    (folder/'VALIDATION_ONLY.txt').write_text('OFFLINE ONLY; NO HARDWARE',encoding='utf-8')
    save()
    for name,cwd,args in JOBS:
        log=QT/('Docs/adaptive_turn_'+name+'_20261006.log');started=time.perf_counter()
        with log.open('w',encoding='utf-8') as stream:
            proc=subprocess.Popen([sys.executable,'-B','-u',*args],cwd=cwd,env=ENV,
                stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT)
            state['jobs'][name]=dict(pid=proc.pid,status='running',log=str(log));save()
            rc=proc.wait()
        state['jobs'][name].update(status='finished',returncode=rc,elapsed_s=round(time.perf_counter()-started,3));save()
    state.update(complete=True,returncode=int(any(j['returncode'] for j in state['jobs'].values())))
    save();return state['returncode']


if __name__=='__main__':raise SystemExit(main())
