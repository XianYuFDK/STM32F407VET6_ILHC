"""复核本次发布产物与原始日志；不启动GUI或连接串口。"""
import ast
import hashlib
import json
from pathlib import Path
import sys

QT=Path(__file__).resolve().parents[1]
REPO=QT.parents[2]
sys.path.insert(0,str(QT))
import route_store


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    report=dict(physical_motion_verified=False,nano_verified=False)
    paths=list(QT.glob('*.py'))
    paths += [QT/p for p in (
        'tests/test_adaptive_turn_fix.py','tests/test_pivot_turns.py','tests/test_wireless_telemetry_qt.py',
        'tests/test_hardware_trajectory_qt.py','Docs/audit_adaptive_turn_release_20261006.py',
        'Docs/audit_adaptive_turn_response_20261006.py','Docs/run_adaptive_turn_validation_20261006.py',
        'Docs/verify_adaptive_turn_release_20261006.py','Docs/audit_real_control_response_20261006.py')]
    paths += [REPO/'Tests/hardware/test_coordinate_batch.py']
    for path in paths:ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path),feature_version=(3,10))
    report['python_310_syntax_files']=len(paths)
    status=json.loads((QT/'Docs/adaptive_turn_validation_status_20261006.json').read_text(encoding='utf-8'))
    assert status['complete'] and status['returncode']==0
    report['validation_status']=status
    native_log=(REPO/'RTOS_APP/validation/adaptive-turn-coordinate-c-20261006.log').read_text(encoding='utf-8-sig')
    assert 'Ran 20 tests' in native_log and '\nOK' in native_log
    report['native_coordinate_tests']=20
    log=(REPO/'RTOS_APP/validation/adaptive-turn-lag-c-strict-20261006.log').read_text(encoding='utf-8-sig')
    assert 'Ran 1 test' in log and '\nOK' in log
    firmware=REPO/'MDK-ARM/build/STM32F407VET6_ILHC/STM32F407VET6_ILHC_v2.1.17_ADAPTIVE_CCAPS7.hex'
    expected='40609bcf45d2ec5628a09f0f8dfa002887844354b78b46ea3772a1418c1688c3'
    assert sha(firmware)==expected==sha(firmware.with_name('STM32F407VET6_ILHC.hex'))
    report['firmware']=dict(path=str(firmware),sha256=expected)
    audit=json.loads((QT/'Docs/adaptive_turn_release_20261006.json').read_text(encoding='utf-8'))
    manifest_path=QT/'precomputed_routes/manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    assert audit['algorithm']==manifest['algorithm']==route_store.algorithm_revision()
    for row in manifest['routes']:assert sha(manifest_path.parent/row['file'])==row['sha256']
    report['static_routes']=len(manifest['routes'])
    report['complete_modeled_matches']=len(audit['runs'])+len(audit['recorded_scene']['runs'])
    directory=QT/'records/runs/20261006_193116_859875_REAL_56ce1058/runs/852e5b168c7a4f76964180cb4a71dcbe'
    original=json.loads((QT/'Docs/real_run_20261006_193128.json').read_text(encoding='utf-8'))
    for name,expected in original['closed_run_file_sha256'].items():assert sha(directory/name)==expected
    report['closed_run_file_sha256']=original['closed_run_file_sha256']
    data=json.loads((directory/'plan.json').read_text(encoding='utf-8'))['match']['map_snapshot']
    manifest['additional_map_snapshots']=[dict(source_run_id=directory.name,map_id=data['map_id'],
        map_version=data['map_version'],source_plan_sha256=sha(directory/'plan.json'))]
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    report['algorithm']=audit['algorithm']
    (QT/'Docs/adaptive_turn_release_checks_20261006.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('发布复核通过：语法',len(paths),'固定缓存',report['static_routes'],'整场模型',report['complete_modeled_matches'])
    print('固件与原始闭合日志SHA256验证通过；实车及Nano仍待复测。')


if __name__=='__main__':main()
