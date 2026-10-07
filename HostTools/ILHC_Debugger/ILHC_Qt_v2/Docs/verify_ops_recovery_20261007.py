"""复核接收恢复最终产物、验证状态及原始日志；不访问硬件。"""
import ast
import hashlib
import json
from pathlib import Path
import sys

QT=Path(__file__).resolve().parents[1]
REPO=QT.parents[2]
OUT=REPO/'RTOS_APP/validation'
sys.path.insert(0,str(QT))
import route_store


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))


def main():
    initial=load(OUT/'ops-rx-recovery-validation-status-20261006.json')
    final=load(OUT/'ops-rx-recovery-validation-status-final-20261006.json')
    assert len(initial)==14 and initial['pc_targeted']['returncode']==1
    assert all(v['returncode']==0 for k,v in initial.items() if k!='pc_targeted')
    assert final['pc_targeted']['returncode']==0 and all(v['returncode']==0 for v in final.values())
    merged={**initial,**final}
    for key,count in [('pc_targeted',38),('qt_trajectory',38),('test_ops_trajectory_hold',4),
                      ('test_coordinate_batch',21),('test_trajectory_buffer',16)]:
        text=Path(merged[key]['log']).read_text(encoding='utf-8-sig')
        assert ('Ran '+str(count)+' tests') in text and '\nOK' in text
    assert 'OPS短错恢复' in Path(merged['test_ops_rx_recovery']['log']).read_text(encoding='utf-8-sig')
    build=load(OUT/'ops-rx-recovery-build-20261006.json')
    assert build['status']=='ok' and build['metrics']['errors']==build['metrics']['warnings']==0
    folder=REPO/'MDK-ARM/build/STM32F407VET6_ILHC'
    firmware=folder/'STM32F407VET6_ILHC_v2.1.18_OPS_RECOVERY_CCAPS8.hex'
    expected='5aeddfb56af2668a74691ea9a97039f24b27a9884a46d268a08b543d628ec026'
    assert sha(firmware)==sha(folder/'STM32F407VET6_ILHC.hex')==expected
    assert sha(folder/'STM32F407VET6_ILHC_v2.1.18_OPS_DIAG_CCAPS8.hex')=='5daf3139b8b57b23013a27478d993463d59e5f2ff2ef61704d3a580ec41046a6'
    manifest=load(QT/'precomputed_routes/manifest.json')
    assert route_store.algorithm_revision()==manifest['algorithm']=='ba1e0b99663a9e9d9f497f11e6db09c51858a357bb265a1f93e54e76aca06df1'
    assert len(manifest['routes'])==36
    for row in manifest['routes']:assert sha(QT/'precomputed_routes'/row['file'])==row['sha256']
    raw_checks={}
    for name in ('early_stop_234427_20261006.json','early_stop_ops_20261006_231215.json'):
        original=load(QT/'Docs'/name)
        for filename,value in original['raw_file_sha256'].items():
            assert sha(Path(original['source_directory'])/filename)==value
        raw_checks[name]=original['raw_file_sha256']
    paths=[QT/name for name in ('ops_diagnostics.py','hardware_trajectory.py','tests/test_ops_diagnostics.py',
        'Docs/run_ops_recovery_validation_20261006.py','Docs/verify_ops_recovery_20261007.py')]
    paths += [REPO/'Tests/hardware'/name for name in ('test_ops_protocol.py','test_ops_rx_recovery.py',
                 'test_ops_trajectory_hold.py','test_trajectory_bridge.py')]
    for p in paths:ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p),feature_version=(3,10))
    sources=[REPO/'Hardware'/name for name in ('ops.c','ops.h','debug_usart.c','trajectory_buffer.c','trajectory_buffer.h')]
    assert 'static float s_mount_x_mm = 53.0f;' in sources[0].read_text(encoding='utf-8-sig')
    assert 'static float s_mount_y_mm = -39.5f;' in sources[0].read_text(encoding='utf-8-sig')
    result=dict(complete=True,transport='JDY-31 SPP',coordinate_caps=8,physical_motion_verified=False,
        flashed=False,serial_connected=False,build_metrics=build['metrics'],firmware=dict(path=str(firmware),sha256=expected),
        validation=merged,initial_validation=initial,final_validation=final,
        tests=dict(native_scripts=9,hold=4,legacy=16,coordinate=21,pc=38,qt=38),
        algorithm=manifest['algorithm'],static_routes=36,python_310_files=len(paths),
        original_raw_sha256=raw_checks,source_sha256={str(p.relative_to(REPO)):sha(p) for p in sources},
        initial_fixture_errors_resolved=True,initial_failures=[
          'Bridge stop count expected3 corrected4 after earlier offline stop',
          'New RX fixture unused inject_rx_error fixed by actual publication-race injection',
          'Hold fixture Y-only check caused teleport; corrected XY distance, initial log retained',
          'PC nonexistent protocol module names corrected to existing tests; initial status retained',
          'GOTO extra fixture missing IRQ mocks; initial bridge-extra-final log retained'])
    (QT/'Docs/ops_recovery_checks_20261007.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('恢复修复复核通过：全部最终验证退出0，0错误0警告，36缓存与14原始文件SHA保持。')
    print('HEX SHA256:',expected)


if __name__=='__main__':main()
