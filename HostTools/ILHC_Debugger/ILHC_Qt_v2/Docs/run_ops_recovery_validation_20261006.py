"""接收恢复回归；隔离日志并保存每组退出状态。"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

QT=Path(__file__).resolve().parents[1]
REPO=QT.parents[2]
OUT=REPO/'RTOS_APP/validation'
env=dict(os.environ,PYTHONIOENCODING='utf-8',QT_QPA_PLATFORM='offscreen',
         ILHC_RUN_LOG_DIR=str(QT/'records/offline_validation_20261006_ops_recovery'))
native=('test_ops_protocol','test_ops_rx_recovery','test_ops_axis_direction','test_ops_zero_frame',
        'test_ops_offset','test_debug_telemetry','test_debug_rx_recovery','test_parse_line_axes',
        'test_trajectory_bridge','test_ops_trajectory_hold','test_trajectory_buffer','test_coordinate_batch')
jobs=[(n,REPO,[str(REPO/'Tests/hardware'/(n+'.py'))]) for n in native]
jobs.append(('pc_targeted',QT,['-m','unittest','-v','tests.test_ops_diagnostics','tests.test_run_journal',
    'tests.test_run_journal_qt','tests.test_wireless_telemetry','tests.test_mixed_telemetry','tests.test_crc_telemetry']))
jobs.append(('qt_trajectory',QT,['-m','unittest','-v','tests.test_hardware_trajectory_qt','test_serial_lifecycle']))
status={}
selection=sys.argv[1:]
if selection:
    jobs=[job for job in jobs if job[0] in selection]
    if len(jobs)!=len(selection):raise ValueError('unknown or repeated validation job')
suffix='-final' if selection else ''
status_path=OUT/('ops-rx-recovery-validation-status'+suffix+'-20261006.json')
if selection and status_path.exists():status=json.loads(status_path.read_text(encoding='utf-8'))
for name,cwd,args in jobs:
    started=time.monotonic();log=OUT/('ops-rx-recovery-'+name+suffix+'-20261006.log')
    with log.open('w',encoding='utf-8') as stream:
        process=subprocess.run([sys.executable,'-B',*args],cwd=cwd,env=env,stdout=stream,stderr=subprocess.STDOUT)
    status[name]=dict(returncode=process.returncode,elapsed_s=round(time.monotonic()-started,3),log=str(log))
    status_path.write_text(json.dumps(status,indent=2),encoding='utf-8')
    print(name,process.returncode,flush=True)
raise SystemExit(any(row['returncode'] for row in status.values()))
