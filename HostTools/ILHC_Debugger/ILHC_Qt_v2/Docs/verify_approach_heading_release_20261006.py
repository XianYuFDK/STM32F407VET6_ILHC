"""复核v2.1.18产物及闭合日志；保留首轮失败与最终复跑各自状态。"""
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
def read(name):return json.loads((QT/'Docs'/name).read_text(encoding='utf-8-sig'))


def main():
    report=dict(physical_motion_verified=False,nano_verified=False)
    paths=list(QT.glob('*.py'))
    paths += [QT/p for p in (
        'tests/test_approach_heading.py','tests/test_coordinate_navigation.py','tests/test_route_optimality.py',
        'tests/test_hardware_trajectory.py','tests/test_hardware_trajectory_qt.py',
        'Docs/audit_approach_heading_release_20261006.py','Docs/audit_approach_heading_response_20261006.py',
        'Docs/audit_approach_heading_20261006.py','Docs/run_approach_heading_validation_20261006.py',
        'Docs/verify_approach_heading_release_20261006.py','Docs/audit_real_control_response_20261006.py')]
    paths += [REPO/'Tests/hardware/test_coordinate_batch.py']
    for path in paths:ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path),feature_version=(3,10))
    report['python_310_syntax_files']=len(paths)
    initial=read('approach_heading_validation_status_20261006.json')
    final=read('approach_heading_headless_final_status_20261006.json')
    assert initial['complete'] and final['complete'] and final['returncode']==0
    for name,job in initial['jobs'].items():
        if name!='headless':assert job['status']=='finished' and job['returncode']==0
    report.update(initial_validation=initial,final_headless=final)
    for name,count in [('coordinate_c',21),('qt',197),('legacy_c',16),('headless_final',368)]:
        log=(QT/'Docs'/('approach_heading_'+name+'_20261006.log')).read_text(encoding='utf-8-sig')
        assert ('Ran '+str(count)+' tests') in log and '\nOK' in log
    report['tests']=dict(core_selftests=6,headless=368,qt=197,native_coordinate=21,legacy_c=16)
    firmware=REPO/'MDK-ARM/build/STM32F407VET6_ILHC/STM32F407VET6_ILHC_v2.1.18_APPROACH_CCAPS8.hex'
    expected='f483bf7eec67621534052b6873b5fd843a7185ac10121bc643527291b11f3cf9'
    assert sha(firmware)==expected==sha(firmware.with_name('STM32F407VET6_ILHC.hex'))
    report['firmware']=dict(path=str(firmware),sha256=expected)
    audit=read('approach_heading_release_20261006.json')
    response=read('approach_heading_response_20261006.json')
    manifest_path=QT/'precomputed_routes/manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    assert audit['algorithm']==manifest['algorithm']==route_store.algorithm_revision()
    for row in manifest['routes']:assert sha(manifest_path.parent/row['file'])==row['sha256']
    report['static_routes']=len(manifest['routes'])
    report['complete_modeled_matches']=len(audit['runs'])+len(audit['recorded_scene']['runs'])+1
    assert response['planning_stats']['search_calls']==0
    directory=QT/'records/runs/20261006_211155_339638_REAL_8badec59/runs/92ebbeb40d1246c6b9b2962ef2b4d9a3'
    original=read('approach_heading_real_20261006.json')
    for name,expected in original['closed_file_sha256'].items():
        assert sha(directory/name)==expected==response['closed_file_sha256'][name]
    report['closed_run_file_sha256']=original['closed_file_sha256']
    report.update(algorithm=audit['algorithm'],complete=True,resolved=True)
    (QT/'Docs/approach_heading_release_checks_20261006.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('发布复核通过：语法',len(paths),'固定缓存',report['static_routes'],'整场模型',report['complete_modeled_matches'])
    print('固件与原始闭合日志SHA256通过；实车及Nano待复测。')


if __name__=='__main__':main()
