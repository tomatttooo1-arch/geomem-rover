"""Independent-recording audit: no fitted map alignment or absolute-error claim."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
from host.localization_gate import pose_matrix, pose_valid
from tools.rtabmap_ros_audit import audit

OPTICAL_IN_TRACKED = pose_matrix({'position':dict(x=0.,y=0.,z=0.),
    'orientation':dict(x=-.5,y=.5,z=-.5,w=.5)})


def relative_error(estimate_a, estimate_b, truth_a, truth_b):
    expected=np.linalg.inv(truth_a)@truth_b
    observed=np.linalg.inv(estimate_a)@estimate_b
    error=np.linalg.inv(expected)@observed
    return float(np.linalg.norm(error[:3,3])), math.degrees(math.acos(float(np.clip((np.trace(error[:3,:3])-1)/2,-1,1))))


def motion_pairs(localizations, truth, allowed=None):
    """One-second pairs, nearest GT <20ms, endpoint interval within 50ms.

    Conjugate tracked-frame motion into the color optical frame before scoring.
    No current-run/global alignment is estimated. Coverage stays explicit.
    """
    rows=[]
    for row in sorted(localizations,key=lambda r:r['stamp_ns']):
        pose=row['pose']['pose']; stamp=row['stamp_ns']
        if allowed is not None and stamp not in allowed:continue
        if not pose_valid(pose):continue
        nearest=int(np.argmin(np.abs(truth[:,0]-stamp/1e9)));gt=truth[nearest]
        if abs(gt[0]-stamp/1e9)>=.02:continue
        tpose={'position':dict(zip(('x','y','z'),gt[1:4])),
               'orientation':dict(zip(('x','y','z','w'),gt[4:8]))}
        if not pose_valid(tpose):continue
        rows.append((stamp,pose_matrix(pose)@OPTICAL_IN_TRACKED,pose_matrix(tpose)))
    values=[]
    for i,(stamp,estimate,reference) in enumerate(rows):
        candidates=rows[i+1:]
        if not candidates:break
        later=min(candidates,key=lambda r:abs((r[0]-stamp)/1e9-1.))
        dt=(later[0]-stamp)/1e9
        if abs(dt-1.)>.05:continue
        translation,angle=relative_error(estimate,later[1],reference,later[2])
        values.append({'from_ns':stamp,'to_ns':later[0],'dt_s':dt,'translation_error_m':translation,'rotation_error_deg':angle})
    return {'eligible_endpoint_frames':len(rows),'pairs':len(values),
            'translation_rmse_m':float(np.sqrt(np.mean([r['translation_error_m']**2 for r in values]))) if values else None,
            'rotation_rmse_deg':float(np.sqrt(np.mean([r['rotation_error_deg']**2 for r in values]))) if values else None,
            'records':values,'limitation':'relative motion only; an arbitrary fixed global pose error cancels and is undetectable'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--truth',required=True,type=Path);a=p.parse_args()
    events=[json.loads(l) for l in (a.run/'observations.jsonl').read_text().splitlines()]
    run=json.loads((a.run/'runner-result.json').read_text());health=audit(events)
    pubs={r['stamp_ns']:r for r in events if r['event']=='publish_begin'}
    info=[r for r in events if r['event']=='info'];local=[r for r in events if r['event']=='localization_pose']
    gate_path=a.run/'gate-events.jsonl';gates=[json.loads(l) for l in gate_path.read_text().splitlines()] if gate_path.exists() else []
    approved=[o for r in gates for o in r.get('approved',[])];stamps={o['stamp_ns'] for o in approved}
    assert len(stamps)==len(approved)
    raw={r['stamp_ns']:r for r in local}
    for o in approved:assert o['pose']==raw[o['stamp_ns']]['pose']
    with sqlite3.connect(f'file:{a.run}/map.db?mode=ro',uri=True) as db:ids={r[0] for r in db.execute('SELECT id FROM Node')}
    healthy_indices=set(health['all_frames']['numeric_valid_localization_with_link_and_odom_indices'])
    healthy_stamps={s for s,pub in pubs.items() if pub['index'] in healthy_indices}
    matched={r['stamp_ns'] for r in info if r.get('loop_closure_id') in ids or r.get('proximity_detection_id') in ids}
    truth=np.loadtxt(a.truth)
    complete=[r for r in events if r['event']=='complete']
    summary={'published':len(pubs),'lost':len(health['all_frames']['lost_indices']),
             'healthy_odometry':len(health['all_frames']['numeric_valid_odom_and_info_indices']),
             'numeric_healthy_map_localizations':len(healthy_stamps),'links_to_database_node_ids':len(matched),
             'gate_started':gate_path.exists(),'gate_rgb_received':sum(r['event']=='acquisition' for r in gates),
             'gate_approved':len(stamps),'gate_approved_outside_numeric_health':sorted(stamps-healthy_stamps),
             'gate_first_approved_index':min((pubs[s]['index'] for s in stamps),default=None),
             'map_nodes_after':len(ids),'database_sha256_after':hashlib.sha256((a.run/'map.db').read_bytes()).hexdigest(),
             'replay_exit_code':run['replay_exit_code'],'all_owned_processes_exited':run['all_owned_processes_exited'],
             'complete_records':complete,'absolute_accuracy':'not evaluated by this relative-only audit; world-frame alignment is not an input'}
    result={'summary':summary,'numeric_audit':health,
        'relative_motion_all_numeric_poses':motion_pairs(local,truth),
        'relative_motion_healthy_endpoints':motion_pairs(local,truth,healthy_stamps),
        'relative_motion_gate_endpoints':motion_pairs(local,truth,stamps),
        'source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [a.run/'observations.jsonl',a.truth]},
        'protocol':'relative-only recording audit; no prior alignment applied or current trajectory fit; fixed camera_link to optical extrinsic'}
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in summary.items() if k!='complete_records'},indent=2))


if __name__=='__main__':main()
