"""Repeat the native coordinate suite alone after Qt releases its planner caches."""
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

qt = Path(__file__).resolve().parents[1]
repo = qt.parents[2]
status = qt / 'Docs/smooth_wheel_pivot_coordinate_retry_status_20261006.json'
state = {'runner_pid': os.getpid(), 'started': datetime.datetime.now().isoformat(), 'status': 'waiting_for_qt'}
status.write_text(json.dumps(state, indent=2), encoding='utf-8')
original = qt / 'Docs/smooth_wheel_pivot_background_status_20261006.json'
while json.loads(original.read_text(encoding='utf-8'))['jobs'].get('qt', {}).get('status') != 'finished':
    time.sleep(10)
log = repo / 'RTOS_APP/validation/smooth-wheel-pivot-c-retry-final-20261006.log'
env = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUNBUFFERED='1', QT_QPA_PLATFORM='offscreen', PYTHONPATH=str(qt))
state.update(status='running', log=str(log))
status.write_text(json.dumps(state, indent=2), encoding='utf-8')
started = time.monotonic()
with log.open('w', encoding='utf-8') as output:
    result = subprocess.run([sys.executable, '-u', '-m', 'unittest', 'test_coordinate_batch', '-v'],
                            cwd=repo / 'Tests/hardware', env=env, stdin=subprocess.DEVNULL,
                            stdout=output, stderr=subprocess.STDOUT)
state.update(status='finished', returncode=result.returncode, elapsed_seconds=round(time.monotonic()-started, 3),
             finished=datetime.datetime.now().isoformat())
status.write_text(json.dumps(state, indent=2), encoding='utf-8')
sys.exit(result.returncode)
