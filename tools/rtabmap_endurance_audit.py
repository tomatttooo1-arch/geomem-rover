"""Audit same-process looping replay by cycle; never merge repeated source IDs."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
from tools.rtabmap_ros_audit import audit
from tools.rtabmap_independent_audit import motion_pairs


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--truth',required=True,type=Path);a=p.parse_args()
    events=[json.loads(l) for l in (a.run/'observations.jsonl').read_text().splitlines()]
    gates=[json.loads(l) for l in (a.run/'gate-events.jsonl').read_text().splitlines()]
    telemetry=[json.loads(l) for l in (a.run/'process-telemetry.jsonl').read_text().splitlines()]
    runner=json.loads((a.run/'runner-result.json').read_text());truth=np.loadtxt(a.truth)
    pubs=[r for r in events if r['event']=='publish_begin'];lookup={r['stamp_ns']:r for r in pubs}
    assert len(lookup)==len(pubs) and all(b['stamp_ns']>a['stamp_ns'] for a,b in zip(pubs,pubs[1:]))
    groups=defaultdict(list)
    for r in pubs:groups[r['cycle']].append(r)
    approvals=[o for r in gates for o in r.get('approved',[])];approved={o['stamp_ns'] for o in approvals}
    assert len(approved)==len(approvals) and approved <= lookup.keys()
    local={r['stamp_ns']:r for r in events if r['event']=='localization_pose'}
    for o in approvals:assert o['pose']==local[o['stamp_ns']]['pose']
    summaries={}
    for cycle,publications in groups.items():
        stamps={r['stamp_ns'] for r in publications}
        subset=[r for r in events if r.get('stamp_ns') in stamps and r['event'] in ('publish_begin','odom','odom_info','info','localization_pose')]
        h=audit(subset)
        healthy_indices=set(h['all_frames']['numeric_valid_localization_with_link_and_odom_indices'])
        healthy={s for s in stamps if lookup[s]['index'] in healthy_indices}
        granted=approved&stamps
        assert granted<=healthy
        if cycle:assert not granted&{r['stamp_ns'] for r in publications[:2]}
        source_loc=[{**local[s],'stamp_ns':lookup[s]['source_stamp_ns']} for s in stamps if s in local]
        source_approved={lookup[s]['source_stamp_ns'] for s in granted}
        motion=motion_pairs(source_loc,truth,source_approved)
        summaries[cycle]={'published':len(publications),'lost':len(h['all_frames']['lost_indices']),
                          'healthy_map_localizations':len(healthy),'approved':len(granted),
                          'first_approved_source_index':min((lookup[s]['index'] for s in granted),default=None),
                          'relative_motion_gate_endpoints':motion}
    boundaries=[r for r in events if r['event']=='repeat_boundary']
    assert len(boundaries)==len(groups)-1 and all(r['gate_acknowledged'] for r in boundaries)
    processes=defaultdict(list)
    for sample in telemetry:
        for proc in sample['processes']:processes[proc['name']].append({'monotonic_ns':sample['monotonic_ns'],**proc})
    stats={}
    for name,rows in processes.items():
        ids={r['pid'] for r in rows};assert len(ids)==1
        memory=[r['rss_kib'] for r in rows if 'rss_kib' in r]
        stats[name]={'pid':next(iter(ids)),'samples':len(rows),'rss_first_kib':memory[0],'rss_last_kib':memory[-1],
                     'rss_min_kib':min(memory),'rss_max_kib':max(memory)}
    complete=[r for r in events if r['event']=='complete'];assert len(complete)==1
    requested=next(r for r in events if r['event']=='start')['options']['repeat_duration_seconds']
    assert complete[0]['repeat_wall_seconds'] >= requested
    assert runner['replay_exit_code']==0 and runner['all_owned_processes_exited'] and runner['readonly_database_unchanged']
    assert not complete[0]['feedback_timeout_indices'] and not complete[0]['slam_feedback_timeout_indices'] and not complete[0]['missing_stamp_ns']
    for cycle,publications in groups.items():
        first=publications[0]['monotonic_ns'];last=publications[-1]['monotonic_ns']
        summaries[cycle]['rss_median_kib']={name:float(np.median([r['rss_kib'] for r in rows if first <= r['monotonic_ns'] <= last and 'rss_kib' in r]))
            for name,rows in processes.items() if any(first <= r['monotonic_ns'] <= last and 'rss_kib' in r for r in rows)}
    summary={'published':len(pubs),'cycles_started':len(groups),'boundaries':len(boundaries),
             'repeat_wall_seconds_including_drain':complete[0]['repeat_wall_seconds'],
             'requested_repeat_seconds':requested,
             'first_to_last_publication_seconds':(pubs[-1]['monotonic_ns']-pubs[0]['monotonic_ns'])/1e9,
             'lost':sum(r['lost'] for r in summaries.values()),'approved':len(approved),
             'healthy_map_localizations':sum(r['healthy_map_localizations'] for r in summaries.values()),
             'gate_rgb_received':sum(r['event']=='acquisition' for r in gates),
             'max_gate_pending':max(r.get('pending_count',0) for r in gates),
             'max_gate_approved_cache':max(r.get('approved_count',0) for r in gates),
             'strictly_increasing_stamps':True,'boundary_first_two_not_approved':True,
             'approved_pose_equality_and_numeric_health':True,'stable_process_ids':True,
             'all_owned_processes_exited':runner['all_owned_processes_exited'],'replay_exit_code':runner['replay_exit_code'],
             'readonly_database_unchanged':runner['readonly_database_unchanged'],'completion':complete[0]}
    result={'summary':summary,'cycles':summaries,'processes':stats,'boundaries':boundaries,
        'source_hashes':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [a.run/'observations.jsonl',a.run/'gate-events.jsonl',a.run/'process-telemetry.jsonl']},
        'limitations':'synthetic repetitions of one recording, not continuous independent physical motion; no absolute cross-recording accuracy; RSS is not unique allocated memory; only gate policy caches observed, not DDS queues'}
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
