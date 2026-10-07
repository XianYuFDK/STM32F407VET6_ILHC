"""Detached offline regression runner; records each subprocess exit code."""
import concurrent.futures
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

QT = Path(__file__).resolve().parents[1]
REPO = QT.parents[2]
STATUS = QT / 'Docs/moving_wheel_pivot_background_status_20261006.json'
ENV = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUNBUFFERED='1',
           QT_QPA_PLATFORM='offscreen', PYTHONPATH=str(QT))
JOBS = {
    'headless': (QT, ['run_tests.py'], QT / 'Docs/moving_wheel_pivot_headless_background_final_20261006.log'),
    'qt': (QT, ['-m', 'unittest', 'test_map_click_qt', 'tests.test_hardware_trajectory_qt', '-v'], QT / 'Docs/moving_wheel_pivot_qt_final_20261006.log'),
    'coordinate_c': (REPO / 'Tests/hardware', ['-m', 'unittest', 'test_coordinate_batch', '-v'],
                     REPO / 'RTOS_APP/validation/moving-wheel-pivot-c-background-final-20261006.log'),
    'legacy_c': (REPO / 'Tests/hardware', ['-m', 'unittest', 'test_trajectory_buffer',
                    'test_trajectory_bridge', 'test_trajectory_param_protocol',
                    'test_param_integration', 'test_param_readback', 'test_rtos_app',
                    'test_coordinate_chain', '-v'],
                 REPO / 'RTOS_APP/validation/moving-wheel-pivot-legacy-background-final-20261006.log'),
}
lock = threading.Lock()
state = {'runner_pid': os.getpid(), 'started': datetime.datetime.now().isoformat(),
         'jobs': {}, 'complete': False}


def save():
    temporary = STATUS.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(STATUS)


def run(name, job):
    cwd, args, log = job
    started = time.monotonic()
    with log.open('w', encoding='utf-8') as output:
        process = subprocess.Popen([sys.executable, '-u', *args], cwd=cwd, env=ENV,
                                   stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT)
        with lock:
            state['jobs'][name] = {'pid': process.pid, 'log': str(log), 'status': 'running'}
            save()
        code = process.wait()
    with lock:
        state['jobs'][name].update(status='finished', returncode=code,
                                  elapsed_seconds=round(time.monotonic() - started, 3))
        save()


if __name__ == '__main__':
    save()
    # Planner replay caches can be large: serialize suites to avoid competing
    # with the user's running simulator and exhausting Windows commit memory.
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
        futures = [pool.submit(run, name, job) for name, job in JOBS.items()]
        for future in futures:
            future.result()
    state['complete'] = True
    state['finished'] = datetime.datetime.now().isoformat()
    state['returncode'] = int(any(job['returncode'] != 0 for job in state['jobs'].values()))
    save()
    sys.exit(state['returncode'])
