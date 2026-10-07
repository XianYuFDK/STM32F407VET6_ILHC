"""只读复核诊断发布；不连接硬件，不覆盖历史验证报告。"""
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
def load(path):return json.loads(path.read_text(encoding='utf-8-sig'))


def main():
    build=load(REPO/'RTOS_APP/validation/ops-diagnostics-build-final-20261006.json')
    assert build['status']=='ok' and build['metrics']['errors']==build['metrics']['warnings']==0
    native=load(REPO/'RTOS_APP/validation/ops-diagnostics-native-status-20261006.json')
    assert len(native)==8 and all(row['returncode']==0 for row in native.values())
    log=(QT/'Docs/ops_diagnostics_pc_20261006.log').read_text(encoding='utf-8-sig')
    assert 'Ran 38 tests' in log and '\nOK' in log
    paths=[QT/name for name in ('ops_diagnostics.py','main.py','analyze_run.py','run_journal.py',
        'tests/test_ops_diagnostics.py','tests/test_run_journal_qt.py','tests/headless_support.py','run_tests.py',
        'Docs/verify_ops_diagnostics_20261006.py')]
    paths += [REPO/'Tests/hardware'/name for name in ('test_ops_protocol.py','test_debug_telemetry.py')]
    for path in paths:ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path),feature_version=(3,10))
    manifest=load(QT/'precomputed_routes/manifest.json')
    algorithm=route_store.algorithm_revision()
    assert algorithm==manifest['algorithm']=='ba1e0b99663a9e9d9f497f11e6db09c51858a357bb265a1f93e54e76aca06df1'
    assert len(manifest['routes'])==36
    for row in manifest['routes']:assert sha(QT/'precomputed_routes'/row['file'])==row['sha256']
    original=load(QT/'Docs/early_stop_ops_20261006_231215.json')
    for name,expected in original['raw_file_sha256'].items():
        assert sha(Path(original['source_directory'])/name)==expected
    folder=REPO/'MDK-ARM/build/STM32F407VET6_ILHC'
    firmware=folder/'STM32F407VET6_ILHC_v2.1.18_OPS_DIAG_CCAPS8.hex'
    assert sha(firmware)==sha(folder/'STM32F407VET6_ILHC.hex')
    old='f483bf7eec67621534052b6873b5fd843a7185ac10121bc643527291b11f3cf9'
    assert sha(folder/'STM32F407VET6_ILHC_v2.1.18_APPROACH_CCAPS8.hex')==old
    offsets=[line.strip() for line in (REPO/'Hardware/ops.c').read_text(encoding='utf-8-sig').splitlines()
             if '53.0f' in line or '-39.5f' in line]
    assert len(offsets)>=2
    report=dict(complete=True,transport='JDY-31 Bluetooth SPP',physical_motion_verified=False,
        flashed=False,serial_connected=False,python_310_syntax_files=len(paths),pc_tests=38,
        native=native,build_metrics=build['metrics'],firmware=dict(path=str(firmware),sha256=sha(firmware)),
        old_firmware_sha256=old,algorithm=algorithm,static_routes=36,
        original_raw_file_sha256=original['raw_file_sha256'],user_mount_offsets=offsets,
        initial_telemetry_fixture_failure='uint16_t extraction support fixed; initial log retained',
        precise_historical_subcause_recoverable=False)
    (QT/'Docs/ops_diagnostics_checks_20261006.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('OPS诊断复核通过：38 PC/Qt，8真实C，构建0错误0警告，36缓存及7原始日志SHA不变。')
    print('新固件SHA256:',report['firmware']['sha256'])


if __name__=='__main__':main()
