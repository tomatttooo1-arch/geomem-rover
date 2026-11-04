"""Cross-check live gate approvals against independent unchanged offline instruments."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics

import numpy as np
from tools.rtabmap_ros_audit import audit
from tools.rtabmap_ros_evaluate import evaluate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--truth',required=True,type=Path);p.add_argument('--alignment',required=True,type=Path)
    a=p.parse_args();raw=a.run/'observations.jsonl';log=a.run/'gate-events.jsonl'
    events=[json.loads(l) for l in raw.read_text().splitlines()]
    gates=[json.loads(l) for l in log.read_text().splitlines()]
    score=evaluate(events,np.loadtxt(a.truth),json.loads(a.alignment.read_text()))
    health=audit(events);runner=json.loads((a.run/'runner-result.json').read_text())
    assert runner['replay_exit_code']==0 and runner['all_owned_processes_exited'] and runner['readonly_database_unchanged']
    assert len(score['complete_records'])==1 and score['all_frames']['published']==180
    pubs={e['stamp_ns']:e for e in events if e['event']=='publish_begin'}
    rawposes={e['stamp_ns']:e for e in events if e['event']=='localization_pose'}
    outputs=[output for e in gates for output in e.get('approved',[])]
    stamps=[o['stamp_ns'] for o in outputs];assert len(stamps)==len(set(stamps))
    for output in outputs:assert output['pose']==rawposes[output['stamp_ns']]['pose']
    indices={pubs[s]['index'] for s in stamps}
    healthy=set(health['all_frames']['numeric_valid_localization_with_link_and_odom_indices'])
    bad=[s for s in score['scored_localizations'] if s['stamp_ns'] in stamps and not s['good']]
    received={kind:{e['stamp_ns']:e for e in gates if e['event']=='received_'+kind}
              for kind in ('odom','odom_info','info','localization_pose')}
    latency={}
    for kind,rows in received.items():
        differences=[(e['monotonic_ns']-rows[e['stamp_ns']]['monotonic_ns'])/1e9 for e in events
                     if e['event']==kind and e['stamp_ns'] in rows]
        latency[kind]={'paired':len(differences),'observer_minus_gate_median_s':statistics.median(differences) if differences else None,
                       'observer_minus_gate_max_s':max(differences) if differences else None}
    acquisitions={e['stamp_ns']:e for e in gates if e['event']=='acquisition'}
    completion=[]
    for stamp,acquisition in acquisitions.items():
        if all(stamp in rows for rows in received.values()):
            delay=(max(rows[stamp]['monotonic_ns'] for rows in received.values())-acquisition['monotonic_ns'])/1e9
            completion.append({'index':pubs[stamp]['index'],'seconds':delay,'healthy_offline':pubs[stamp]['index'] in healthy})
    result={'summary':{'published':len(pubs),'gate_rgb_received':len(acquisitions),'gate_approved':len(stamps),
        'healthy_offline':len(healthy),'approved_indices':sorted(indices),'false_approvals':bad,
        'approved_outside_numeric_health':sorted(indices-healthy),
        'large_error_inputs':[{'index':r['index'],'error_m':r['error_m'],'approved':r['stamp_ns'] in stamps}
                              for r in score['scored_localizations'] if r['error_m'] is not None and r['error_m']>.15],
        'first_recovery_approved':(next((i for i in sorted(indices) if i>=score['recovery']['window'][0]),None)
                                   if score['recovery'] is not None else None),
        'gate_healthy_completion_over_half_second':[r for r in completion if r['healthy_offline'] and r['seconds']>=.5],
        'pose_equality_against_raw':True,'database_unchanged':True,
        'raw_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),'gate_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest()},
        'receipt_comparison':latency,'gate_completion':completion,'score':score,'numeric_audit':health,
        'protocol':'live gate diagnostic outputs; original independent numeric health and fixed-map truth scoring; same recording CPU probe',
        'timing_note':'monotonic log timestamps are callback processing completion times, not middleware receipt timestamps; differences include conversion and policy costs'}
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result['summary'].items() if k not in ('approved_indices','large_error_inputs','gate_healthy_completion_over_half_second')},indent=2))


if __name__=='__main__':main()
