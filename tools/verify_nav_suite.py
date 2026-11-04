#!/usr/bin/env python3
"""Read-only B review from AGX action logs and server passive odometry."""
import argparse
import hashlib
import json
import math
from pathlib import Path

def read(path):
    return [json.loads(line) for line in path.read_text().splitlines()]

def angle(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))

def review(run):
    events=read(run/'agx/nav-suite.jsonl');motion=read(run/'motion.jsonl')
    expected={'straight':4,'turn':4,'uturn':4,'near':4,'cancel':5,'unreachable':6}
    cases=[r for r in events if r['event']=='case_result']
    summary=[r for r in events if r['event']=='summary']
    od=[]
    for row in motion:
        if row['topic']=='/odom':
            msg=row['message'];s=msg['header']['stamp'];p=msg['pose']['pose'];v=msg['twist']['twist']
            od.append(dict(stamp=s['sec']+s['nanosec']/1e9,x=p['position']['x'],y=p['position']['y'],
                yaw=angle(p['orientation']),speed=abs(v['linear']['x'])+abs(v['angular']['z'])))
    results=[]
    for case in cases:
        available=[r for r in od if r['stamp']<=case['sim_s']]
        observed=max(available,key=lambda r:r['stamp']) if available else None
        ok=bool(observed and 0<=case['sim_s']-observed['stamp']<=.3 and observed['speed']<.01)
        xy=yaw=None
        if observed and case['name'] in ('straight','turn','uturn','near'):
            x,y,a=case['target'];xy=math.hypot(observed['x']-x,observed['y']-y)
            yaw=abs(math.atan2(math.sin(observed['yaw']-a),math.cos(observed['yaw']-a)))
            ok=ok and xy<=.03 and yaw<=.03
        ok=ok and case['status']==expected.get(case['name']) and case['stopped'] and case['passed']
        results.append(dict(case=case['name'],passed=bool(ok),server_odom=observed,
                            xy_error_m=xy,yaw_error_rad=yaw,status=case['status']))
    safe=[r for r in motion if r['topic']=='/fpga/cmd_vel_safe']
    nonzero=sum(any(abs(v)>0 for a in ('linear','angular') for v in r['message'][a].values()) for r in safe)
    permits=[r['message']['data'] for r in motion if r['topic']=='/fpga/motion_permit']
    cancellation=[r for r in events if r['event']=='cancel_response']
    goals=[r for r in events if r['event']=='goal_response']
    ids=[tuple(r['goal_id']) for r in goals]
    files=['agx/nav-suite.jsonl','motion.jsonl','run-config.json']
    passed=bool(len(cases)==6 and {r['name'] for r in cases}==set(expected) and
        summary and summary[-1]['passed'] and all(r['passed'] for r in results) and
        len(ids)==6 and len(set(ids))==6 and all(r['accepted'] for r in goals) and
        cancellation and cancellation[0]['return_code']==0 and cancellation[0]['goals_canceling']==1 and
        nonzero>0 and permits and any(permits) and not permits[-1])
    return dict(passed=passed,source_revision=json.loads((run/'run-config.json').read_text())['source_revision'],
        cases=results,server_safe_nonzero_samples=nonzero,server_safe_samples=len(safe),
        final_permit=permits[-1] if permits else None,
        inputs_sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in files},
        scope='single fixed simulation suite; no physical robot or fault-recovery acceptance')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=review(a.run)
    with a.out.open('x') as f: json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2,allow_nan=False))
    raise SystemExit(0 if result['passed'] else 1)
