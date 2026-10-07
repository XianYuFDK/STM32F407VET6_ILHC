"""Read the completed replay report, verify encoded mode semantics and publish its cache manifest."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
from unittest.mock import patch

QT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(QT))
import competition_simulation as competition
import core
from hardware_coordinates import make_coordinate_batch
from route_store import algorithm_revision
from audit_turn_modes_20261007 import REAL_CONTROL

report=json.loads((QT/'Docs/turn_modes_release_20261007.json').read_text(encoding='utf-8'))
manifest=json.loads((QT/'Docs/turn_modes_manifest_20261007.json').read_text(encoding='utf-8'))
assert report['status']=='PASSED' and len(report['runs'])==12
assert report['algorithm']==manifest['algorithm']==algorithm_revision()
checks=[]
for result in report['runs']:
    data=competition.load_profile(QT/result['map'])
    mode=data['turn_mode']=result['turn_mode']
    control=core.CHASSIS_DEFAULTS if result['control']=='DEFAULT' else REAL_CONTROL
    with patch('coordinate_navigation.plan_route',side_effect=AssertionError('cache miss')):
        match=competition.compile_match(data,zone=1,coordinate_mode=True,chassis_control=control)
    batch=make_coordinate_batch(match,(0,0,0),competition.collision_scene(data))
    assert batch['turn_mode']==match['turn_mode']==mode
    points=batch['points']
    if mode!='WHEEL':assert not any(p[4]&16 for p in points)
    if mode=='STOP_TURN':
        assert all(p[4]&1 for p in points)
        for a,b in zip(points,points[1:]):
            if a[:2]!=b[:2]:assert (a[3]-b[3])%36000==0, (a,b)
    if mode=='WHEEL':assert any(p[4]&32 for p in points)
    checks.append(dict(map=result['map'],control=result['control'],mode=mode,points=len(points),
                       wheel_pivots=sum(bool(p[4]&32) for p in points),
                       stops=sum(bool(p[4]&1) for p in points)))
for item in manifest['routes']:
    assert hashlib.sha256((QT/'precomputed_routes'/item['file']).read_bytes()).hexdigest()==item['sha256']
backup=QT/'Docs/route_manifest_before_turn_modes_20261007.json'
if not backup.exists():shutil.copyfile(QT/'precomputed_routes/manifest.json',backup)
shutil.copyfile(QT/'Docs/turn_modes_manifest_20261007.json',QT/'precomputed_routes/manifest.json')
output=dict(status='PASSED',algorithm=algorithm_revision(),unit_qt_tests=69,model_rounds=12,
            physical_motion_verified=False,firmware_changed=False,wire_checks=checks,
            routes=len(manifest['routes']))
(QT/'Docs/turn_modes_checks_20261007.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(output,ensure_ascii=False))
