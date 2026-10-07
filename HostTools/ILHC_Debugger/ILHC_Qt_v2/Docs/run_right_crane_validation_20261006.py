"""串行离线验收：记录每组退出码，避免规划缓存和Qt同时争用内存。"""
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

QT = Path(__file__).resolve().parents[1]
REPO = QT.parents[2]
STATUS = QT / 'Docs/right_crane_validation_status_20261006.json'
ENV = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUNBUFFERED='1',
           PYTHONDONTWRITEBYTECODE='1', QT_QPA_PLATFORM='offscreen', PYTHONPATH=str(QT))
JOBS = (
    ('headless', QT, ['run_tests.py']),
    ('qt', QT, ['-m', 'unittest', 'test_map_click_qt', 'tests.test_hardware_trajectory_qt',
                'tests.test_right_crane_qt', 'tests.test_trajectory_settings_qt', '-v']),
    ('coordinate_c', REPO / 'Tests/hardware', ['-m', 'unittest', 'test_coordinate_batch', '-v']),
)


def now():
    return datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()


def main():
    state = dict(pid=os.getpid(), started=now(), complete=False, jobs={})
    def save():
        temporary = STATUS.with_suffix('.tmp')
        temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(STATUS)
    save()
    for name, cwd, args in JOBS:
        log = QT / ('Docs/right_crane_'+name+'_20261006.log')
        started = time.perf_counter()
        with log.open('w', encoding='utf-8') as stream:
            process = subprocess.Popen([sys.executable, '-B', '-u', *args], cwd=cwd, env=ENV,
                                       stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT)
            state['jobs'][name] = dict(pid=process.pid, status='running', log=str(log))
            save()
            code = process.wait()
        state['jobs'][name].update(status='finished', returncode=code,
                                 elapsed_s=round(time.perf_counter()-started, 3))
        save()
    state.update(complete=True, finished=now(), returncode=int(any(j['returncode'] for j in state['jobs'].values())))
    save()
    return state['returncode']


if __name__ == '__main__':
    raise SystemExit(main())
