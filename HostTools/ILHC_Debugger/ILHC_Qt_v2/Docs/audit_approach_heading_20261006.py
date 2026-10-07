"""21:12实机日志中的普通拐点提前转向时机；只读原始文件。"""
import hashlib
import json
from pathlib import Path

QT=Path(__file__).resolve().parents[1]


def main():
    session=QT/'records/runs/20261006_211155_339638_REAL_8badec59'
    rid='92ebbeb40d1246c6b9b2962ef2b4d9a3';directory=session/'runs'/rid
    plan=json.loads((directory/'plan.json').read_text(encoding='utf-8'))
    data=(session/'events.jsonl').read_bytes()
    events=[json.loads(l) for l in data.splitlines()]
    controls=[]
    for event in events:
        if event.get('run_id')==rid and event['event']=='RX_TEXT' and event['text'].startswith('CCTRL '):
            fields=list(map(int,event['text'].split()[1:]))
            if fields[0]==plan['id']:
                controls.append(dict(t=event['monotonic'],index=fields[1],flags=fields[2],yaw=fields[3]/100,
                    gap_mm=fields[4]/10,speed_mm_s=(fields[5]**2+fields[6]**2)**.5/10,omega=fields[7]/100,
                    measured_omega=fields[8]/100))
    cases=[]
    for i,p in enumerate(plan['points'][1:],1):
        if p[4]&16 or abs((p[3]-p[5]+18000)%36000-18000)<100:continue
        rows=[x for x in controls if x['index']==i]
        switched=next((x for x in rows if x['flags']&2),None)
        near=[x for x in rows if x['gap_mm']<20]
        cases.append(dict(index=i,goal=p[3]/100,travel=p[5]/100,lead_mm=p[7]/10,flags=p[4],
            observed_heading_switch=switched,near_20mm=near))
    report=dict(source_run_id=rid,pc_version='2.1.17',caps=[e['text'] for e in events
        if e['event']=='RX_TEXT' and e['text'].startswith('CCAPS ')],cases=cases,
        closed_file_sha256={name:hashlib.sha256((directory/name).read_bytes()).hexdigest()
                            for name in ('plan.json','metadata.json','result.json','telemetry.csv')})
    (QT/'Docs/approach_heading_real_20261006.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for case in cases:
        near=case['near_20mm']
        print(case['index'], 'travel/goal',case['travel'],case['goal'],'lead',case['lead_mm'],
            'switch',None if case['observed_heading_switch'] is None else
            round(case['observed_heading_switch']['gap_mm'],1),
            'near',[(r['gap_mm'],r['speed_mm_s'],r['omega'],r['measured_omega']) for r in near[:6]])


if __name__=='__main__':main()
