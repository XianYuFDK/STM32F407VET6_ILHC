"""对照00:49/51/53/55四次归零与手动归位截图；原始文件仅计算SHA。"""
import collections
import csv
import hashlib
import json
import math
from pathlib import Path

QT=Path(__file__).resolve().parents[1]
NAMES=('20261007_004928_847148_REAL_64d0883a','20261007_005132_363166_REAL_69a56e16',
       '20261007_005333_756637_REAL_322e3a93','20261007_005547_927416_REAL_12514cbe')


def main():
    report=dict(source='REAL',conclusion='All four controllers reached reported OPS origin; physical return differs from reported pose',
        user_reported_same_physical_zero_from_prior_conversation=True,
        manual_correction_repeatability='User confirmed all four corrections are approximately the same; no per-run manual measurements recorded',
        screenshot_after_manual=dict(x_mm=14,y_mm=-51,yaw_deg=-.6,position_norm_mm=math.hypot(14,51)),
        screenshot_run_identity_verified=False,post_done_manual_motion_recorded=False,
        exact_encoder_or_imu_cause_proven=False,route_target_modified=False,runs=[])
    for name in NAMES:
        folder=QT/'records/runs'/name
        analysis=json.loads((folder/'analysis.json').read_text(encoding='utf-8'))
        run=analysis['runs'][0];path=folder/'runs'/run['run_id']
        rows=list(csv.DictReader((path/'telemetry.csv').open(encoding='utf-8-sig',newline='')))
        events=[json.loads(s) for s in (folder/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        plan=json.loads((path/'plan.json').read_text(encoding='utf-8'))
        metadata=json.loads((path/'metadata.json').read_text(encoding='utf-8'))['metadata']
        pose=lambda row:dict(x_mm=float(row['pos_x'])*10,y_mm=float(row['pos_y'])*10,yaw_deg=float(row['zangle']))
        final=pose(rows[-1]);final['position_error_mm']=math.hypot(final['x_mm'],final['y_mm'])
        transport=[e for e in events if e['event']=='TRANSPORT']
        diag=run['ops_diagnostics'];snapshots=diag['snapshots'];faults=diag['faults']
        counts=collections.Counter(f.get('OPSE',{}).get('cause_mask') for f in faults)
        d=[s for s in snapshots if s['kind']=='OPSD'];r=[s for s in snapshots if s['kind']=='OPSR']
        command_changes=[e['command'] for e in events if e['event']=='TX' and
                         e.get('command','').split('=')[0] in ('ZERO','OPSOFFSET','STOP','MANUAL','GOTO')]
        keys=('crc_errors','lost_packets','slow_writes','device_restarts','gui_queue_drops','observer_errors')
        uart_delta={k:r[-1][k]-r[0][k] for k in ('uart_error_count','rx_overflows','stale_packets','restart_failures','diagnostic_drops')}
        uart_delta.update(ops_crc_errors=d[-1]['crc_errors']-d[0]['crc_errors'],
                          ops_format_errors=d[-1]['format_errors']-d[0]['format_errors'])
        recovered=[f for f in faults if f.get('OPSE',{}).get('cause_mask',0)&2048]
        recovery_gaps=[f['OPSX']['frame_gap_ms'] for f in recovered if 'OPSX' in f]
        status=[e['text'] for e in events if e['event']=='RX_TEXT' and e.get('text','').startswith('TSTAT')][-1]
        raw=[folder/n for n in ('session.json','events.jsonl','telemetry.csv')]
        raw += [path/n for n in ('metadata.json','plan.json','telemetry.csv','result.json')]
        # 固定安装矢量66.1mm、终点yaw<0.017°，起终点差分补偿只约0.02mm。
        offset=math.hypot(53,39.5)
        row=dict(folder=name,run_id=run['run_id'],status=run['result']['status'],duration_s=run['duration_s'],
            frames=run['frame_count'],initial_pose=pose(rows[0]),final_pose=final,terminal_tstat=status,
            target=plan['points'][-1],points=len(plan['points']),nominal_length_mm=plan['points'][-1][2]/10,
            mapping=metadata['mapping'],chassis_control=metadata['chassis_control'],
            ops_offset_inputs_mm=metadata['ops_offset_inputs_mm'],ops_offset_inputs_are_not_readback=True,
            sessions=sorted({s['session_id'] for s in d}),fault_masks={str(k):v for k,v in counts.items()},
            recovered_gap_ms=recovery_gaps,hard_continuity_events=[f for f in faults if f.get('OPSE',{}).get('cause_mask',0)&(1|2|4|8|16|1024)],
            snapshot_counter_deltas=uart_delta,
            host_transport_deltas={k:transport[-1][k]-transport[0][k] for k in keys},
            commands_changing_origin_offset_or_manual_motion=command_changes,
            final_mount_compensation_norm_mm=2*offset*abs(math.sin(math.radians(final['yaw_deg'])/2)),
            raw_file_sha256={str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest() for p in raw})
        assert row['status']=='DONE' and final['position_error_mm']<1 and not row['hard_continuity_events']
        report['runs'].append(row)
    report['screenshot_move_if_latest_done']=dict(
        dx_mm=14-report['runs'][-1]['final_pose']['x_mm'],
        dy_mm=-51-report['runs'][-1]['final_pose']['y_mm'],
        yaw_delta_deg=-.6-report['runs'][-1]['final_pose']['yaw_deg'])
    destination=QT/'Docs/home_return_0049_0055_20261007.json'
    destination.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for row in report['runs']:
        print(row['folder'],row['status'],{k:round(v,5) for k,v in row['final_pose'].items()},
              'UART delta',row['snapshot_counter_deltas']['uart_error_count'])


if __name__=='__main__':main()
