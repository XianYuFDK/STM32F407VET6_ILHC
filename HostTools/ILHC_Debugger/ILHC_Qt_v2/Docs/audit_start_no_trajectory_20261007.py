"""Offline evidence audit; never opens a serial port or edits original logs."""
import collections
import ctypes
import hashlib
import json
from pathlib import Path
import sys

QT = Path(__file__).resolve().parents[1]
ROOT = QT.parents[2]
FOLDER = QT / 'records/runs/20261007_023125_741650_REAL_278a34a4'
OUTPUT = Path(__file__).with_name('start_no_trajectory_checks_20261007.json')
sys.path.insert(0, str(QT))
sys.path.insert(0, str(ROOT / 'Tests/hardware'))
from analyze_run import analyze
from test_trajectory_buffer import Engine, TrajectoryFirmwareTests, straight


def digest_files(paths):
    return {str(p.relative_to(FOLDER)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def main():
    raw = sorted(p for p in FOLDER.rglob('*') if p.is_file() and p.name in
                 ('session.json', 'events.jsonl', 'telemetry.csv', 'metadata.json', 'plan.json', 'result.json'))
    before = digest_files(raw)
    events = [json.loads(line) for line in (FOLDER / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
    first = next(e for e in events if e['event'] == 'RUN_START')
    frames = [e for e in events if e['event'] == 'FRAME']
    tx = [e for e in events if e['event'] == 'TX']
    begin = next(e for e in tx if e['command'].startswith('CBEGIN='))
    new_id = int(begin['command'].split('=')[1].split(',')[0])
    statuses = [e for e in events if e['event'] == 'RX_TEXT' and e['text'].startswith('TSTAT ')]
    assert len(raw) == 7 and len(frames) == 157
    assert all(int(e['text'].split()[1]) != new_id for e in statuses)
    assert all(not e['command'].startswith(('CPOINT=', 'TCOMMIT=', 'TRUN=')) for e in tx)
    assert all(e['values'][:3] == [0.0, 0.0, 0.0] for e in frames)
    timeline = []
    for e in events:
        if (e['event'] == 'TX' and e['command'].startswith(('CCAPS', 'CBEGIN=', 'TSTATUS=')) or
            e['event'] == 'RX_TEXT' and e['text'].startswith(('CCAPS ', 'TSTAT ')) or
            e['event'] in ('BATCH_STATE', 'RUN_END') or
            e['event'] == 'COMMAND_QUEUED' and e.get('command', '').startswith(('TABORT=', 'STOP'))):
            timeline.append(dict(time_s=e['monotonic']-first['monotonic'], event=e['event'],
                                 details={k:v for k,v in e.items() if k not in
                                          ('monotonic','wall_time','sequence','event','run_id')}))
    # Reproduce the current C parser's rejection response using its actual source.
    cls = TrajectoryFirmwareTests
    cls.setUpClass()
    try:
        e = Engine(cls.dll)
        assert e.upload(straight())[1] == 3
        e.send('TRUN=17')
        for _ in range(1500):
            e.step(move=True)
            if e.status()[1] == 6:
                break
        assert e.status()[1] == 6
        cls.dll.Traj_UpdateChassisParameters.argtypes = [ctypes.POINTER(ctypes.c_float)]
        cls.dll.Traj_UpdateChassisParameters((ctypes.c_float*7)(1.5,1.5,9.05,930,750,.5,.5))
        e.send(begin['command'], allowed=0)
        denied = e.reply()
        assert denied.split()[:3] == ['TSTAT','17','6'], denied
        e.send(begin['command'], allowed=1)
        accepted = e.reply()
        assert accepted.split()[:4] == ['TSTAT',str(new_id),'1','0'], accepted
        native = dict(blocked_upload_response=denied, allowed_upload_response=accepted,
                      scope='Demonstrates gate behavior and valid CBEGIN fields; does not identify live gate flags.')
    finally:
        cls.tearDownClass()
    _, analysis_path = analyze(FOLDER)
    assert digest_files(raw) == before
    transport = [e for e in events if e['event']=='TRANSPORT']
    peers = []
    for name in ('20261007_022934_742109_REAL_0fc64382','20261007_023008_993191_REAL_1309a967',
                 '20261007_010413_202633_REAL_1f813300'):
        data = [json.loads(line) for line in (FOLDER.parent/name/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        peers.append(dict(folder=name, final=next(e for e in reversed(data) if e['event']=='RUN_END'),
                          last_tstat=next(e['text'] for e in reversed(data) if e['event']=='RX_TEXT' and e['text'].startswith('TSTAT '))))
    report = dict(source=str(FOLDER),raw_sha256=before,raw_files_unchanged=True,
                  event_counts=dict(collections.Counter(e['event'] for e in events)),
                  new_batch_id=new_id, timeline=timeline,native_reproduction=native,
                  frame_count=len(frames),device_elapsed_s=(frames[-1]['device_tick_ms']-frames[0]['device_tick_ms'])/1000,
                  transport_first=transport[0],transport_last=transport[-1],related_runs=peers,
                  analysis_report=str(analysis_path),
                  confirmed='New batch never established; all replies are old DONE; no points/commit/run sent; PC cancels after 3s.',
                  unknown='Firmware did not log upload gate flags. Wheel disabled/fault, pending commands or competing controllers cannot be distinguished. Host TX success is not MCU acknowledgment.')
    OUTPUT.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(report=str(OUTPUT),native=native,raw_files_unchanged=True),ensure_ascii=False))


if __name__ == '__main__':
    main()
