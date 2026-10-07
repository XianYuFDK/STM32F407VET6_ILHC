"""DL-20修复串行离线验收；不连接串口，不访问真实运行记录。"""
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

QT=Path(__file__).resolve().parents[1];REPO=QT.parents[2]
STATUS=QT/'Docs/dl20_validation_status_20261006.json'
ENV=dict(os.environ,PYTHONIOENCODING='utf-8',PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',
    QT_QPA_PLATFORM='offscreen',PYTHONPATH=str(QT),ILHC_RUN_LOG_DIR=str(QT/'records/offline_validation_20261006_v216'))
JOBS=(
    ('native_sender',REPO,['Tests/hardware/test_debug_telemetry.py']),
    ('native_commands',REPO,['Tests/hardware/test_parse_line_axes.py']),
    ('native_rx',REPO,['Tests/hardware/test_debug_rx_recovery.py']),
    ('native_params',REPO,['Tests/hardware/test_param_readback.py']),
    ('headless',QT,['run_tests.py']),
    ('qt',QT,['-m','unittest','test_debugger','test_serial_lifecycle','test_map_click_qt',
         'tests.test_dm_position','tests.test_hardware_trajectory_qt','tests.test_right_crane_qt',
         'tests.test_trajectory_settings_qt','tests.test_run_journal_qt','tests.test_wireless_telemetry_qt','-v']),
    ('cache',QT,['Docs/audit_real_control_release_20261006.py','--output','Docs/dl20_release_20261006.json']),
)


def now():return datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()


def main():
    state=dict(pid=os.getpid(),started=now(),complete=False,jobs={})
    def save():STATUS.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
    Path(ENV['ILHC_RUN_LOG_DIR']).mkdir(parents=True,exist_ok=True)
    (Path(ENV['ILHC_RUN_LOG_DIR'])/'VALIDATION_ONLY.txt').write_text('OFFLINE TEST ONLY; NO HARDWARE',encoding='utf-8')
    save()
    for name,cwd,args in JOBS:
        log=QT/('Docs/dl20_'+name+'_20261006.log');started=time.perf_counter()
        with log.open('w',encoding='utf-8') as stream:
            process=subprocess.Popen([sys.executable,'-B','-u',*args],cwd=cwd,env=ENV,
                stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT)
            state['jobs'][name]=dict(pid=process.pid,status='running',log=str(log));save()
            rc=process.wait()
        state['jobs'][name].update(status='finished',returncode=rc,elapsed_s=round(time.perf_counter()-started,3));save()
    state.update(complete=True,finished=now(),returncode=int(any(j['returncode'] for j in state['jobs'].values())))
    save();return state['returncode']


if __name__=='__main__':raise SystemExit(main())
