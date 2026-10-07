"""复核23:44出库取消；仅生成派生报告，不更改原始日志。"""
import csv
import hashlib
import json
from datetime import datetime, timezone, timedelta
from pathlib import Path

QT=Path(__file__).resolve().parents[1]
FOLDER=QT/'records/runs/20261006_234427_088205_REAL_bcfc2280'


def main():
    events=[json.loads(s) for s in (FOLDER/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    report=json.loads((FOLDER/'analysis.json').read_text(encoding='utf-8'))
    rid=report['runs'][0]['run_id']
    trun=next(e for e in events if e['event']=='TX' and e.get('command','').startswith('TRUN='))
    canceled=next(e for e in events if e['event']=='RX_TEXT' and e.get('text')=='TSTAT 3774468927 7 29 1 2147 17')
    fault=next(e for e in events if e['event']=='RX_TEXT' and e.get('text','').startswith('OPSE '))
    transports=[e for e in events if e['event']=='TRANSPORT']
    keys=('crc_errors','lost_packets','duplicate_packets','invalid_pose_frames','device_restarts','gui_queue_drops','observer_errors')
    frames=list(csv.DictReader((FOLDER/'telemetry.csv').open(encoding='utf-8-sig',newline='')))
    ticks=[int(row['device_tick_ms']) for row in frames]
    directory=FOLDER/'runs'/rid
    plan=json.loads((directory/'plan.json').read_text(encoding='utf-8'))
    commands=[dict(time_s=e['monotonic']-trun['monotonic'],command=e['command'])
              for e in events if e['event']=='TX' and e['monotonic']>=trun['monotonic']
              and e.get('command','').split('=')[0] in ('STOP','TABORT','ZERO','OPSOFFSET','WHEELOFF')]
    raw_paths=[FOLDER/name for name in ('session.json','events.jsonl','telemetry.csv')]
    raw_paths += [directory/name for name in ('metadata.json','plan.json','telemetry.csv','result.json')]
    result=dict(source_directory=str(FOLDER),run_id=rid,batch_id=3774468927,
        conclusion='USART2 HAL frame error FE caused generic OPS protection cancellation 17',
        trun_time=datetime.fromtimestamp(trun['wall_time'],timezone(timedelta(hours=8))).isoformat(),
        cancel_time=datetime.fromtimestamp(canceled['wall_time'],timezone(timedelta(hours=8))).isoformat(),
        seconds_trun_to_first_fault_rx=fault['monotonic']-trun['monotonic'],
        seconds_trun_to_cancel_rx=canceled['monotonic']-trun['monotonic'],
        terminal_tstat=canceled['text'],progress_mm=214.7,
        initial_points=plan['points'][:3],ops_diagnostics=report['runs'][0]['ops_diagnostics'],
        transport_deltas={k:transports[-1][k]-transports[0][k] for k in keys},
        telemetry_frames=len(frames),max_device_tick_gap_ms=max(b-a for a,b in zip(ticks,ticks[1:])),
        final_ops=dict(x_cm=float(frames[-1]['pos_x']),y_cm=float(frames[-1]['pos_y']),yaw_deg=float(frames[-1]['zangle'])),
        stop_zero_offset_commands_after_trun=commands,
        flags27_has_imu_rebased=False,actual_session_change_observed=False,
        electrical_origin_proven=False,repeated_callback_is_separate_electrical_errors_proven=False,
        prior_queue_overflow_and_diagnostic_drop_counters='1 and 14 already present in both pre-fault snapshots; no increase observed there',
        raw_file_sha256={str(p.relative_to(FOLDER)):hashlib.sha256(p.read_bytes()).hexdigest() for p in raw_paths})
    destination=QT/'Docs/early_stop_234427_20261006.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('seconds_trun_to_first_fault_rx','seconds_trun_to_cancel_rx','transport_deltas','telemetry_frames','max_device_tick_gap_ms','final_ops')},ensure_ascii=False))


if __name__=='__main__':main()
