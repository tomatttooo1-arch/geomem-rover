"""Causal per-acquisition ledger. Raw health, gate decision and later revocation differ.

Legacy callbacks were logged after processing. Never call that DDS arrival time,
or use a later 'retired stamp' message to overwrite the original rejection.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sqlite3

from host.localization_gate import KINDS, LocalizationGate


def remove_prefix(text, prefix):
    """``str.removeprefix`` for Python 3.8, the declared reference interpreter;
    the standard-library method only exists from Python 3.9.

    Same semantics as the standard-library method: the prefix is stripped only
    when it actually matches, and an unmatched prefix -- including the empty
    prefix, which always matches and strips nothing -- leaves the string as it is.
    """
    return text[len(prefix):] if text.startswith(prefix) else text


def read_events(path):
    with Path(path).open() as stream:return [json.loads(line) for line in stream]


def reason_class(reason):
    if reason=='no_prior_map_match':return 'no_prior_match'
    if reason=='odometry_lost':return 'odometry_lost'
    return 'invalid_'+reason if not reason.startswith('invalid_') else reason


def ledger(events, gates, prior_ids):
    publications=[r for r in events if r['event']=='publish_begin']
    counts=Counter(r['stamp_ns'] for r in publications)
    if any(n!=1 for n in counts.values()):raise ValueError('duplicate publication stamp')
    pubs={r['stamp_ns']:r for r in publications}
    streams={k:defaultdict(list) for k in KINDS}
    for row in events:
        if row['event'] in streams:streams[row['event']][row['stamp_ns']].append(row)
    policy=LocalizationGate('audit',prior_ids)
    rows={};issues=[]
    for stamp,pub in pubs.items():
        flags=[];rejections={}
        for kind in KINDS:
            candidates=streams[kind][stamp]
            if len(candidates)!=1:
                flags.append(kind+(':missing' if not candidates else ':duplicate'))
            else:
                try:why=policy.message_reason(kind,candidates[0])
                except (KeyError,TypeError,ValueError):why='malformed_message'
                if why:flags.append(kind+':'+why);rejections[kind]=why
        rows[stamp]={'stamp_ns':stamp,'source_stamp_ns':pub.get('source_stamp_ns',stamp),
            'index':pub['index'],'cycle':pub.get('cycle',0),'raw_health_flags':flags,
            'raw_numeric_healthy':not flags,'raw_rejections':rejections,
            'primary_reason':None,'first_block':None,'gate_callbacks':[],
            'publication_monotonic_ns':pub['monotonic_ns']}
    pending={};seen_acquisitions=Counter();last_trigger={'reason':'initialization','stamp_ns':None}
    outputs=[];epoch_events=[]

    def block(stamp,reason,event,**detail):
        nonlocal last_trigger
        row=rows[stamp]
        if row['first_block'] is None:
            row['first_block']={'reason':reason,'event':event['event'],'monotonic_ns':event['monotonic_ns'],**detail}
        last_trigger={'reason':row['first_block']['reason'],'stamp_ns':stamp}

    def retire(stamp,reason,event,**detail):
        block(stamp,reason,event,**detail)
        if rows[stamp]['primary_reason'] is None:
            rows[stamp]['primary_reason']=rows[stamp]['first_block']['reason']
        pending.pop(stamp,None)

    for event in gates:
        kind=event['event'];stamp=event.get('stamp_ns');why=event.get('reason','')
        if kind=='state_transition':
            epoch_events.append({k:event[k] for k in ('monotonic_ns','epoch','state','reason','stamp_ns')})
            if why.startswith('replay_discontinuity:'):
                for s in list(pending):retire(s,'replay_boundary',event)
                last_trigger={'reason':'replay_boundary','stamp_ns':stamp}
            elif why=='incomplete_acquisition_timeout':
                diagnostic=event.get('diagnostic',{})
                if diagnostic.get('before') is not None and diagnostic.get('after') is not None:
                    expired=set(diagnostic['before']['pending'])-set(diagnostic['after']['pending'])
                    candidates=[s for s in pending if str(s) in expired]
                else:
                    candidates=[s for s,p in pending.items() if (event['monotonic_ns']-p['begin_ns'])/1e9>=.5
                                or event.get('ros_ns',s)-s>10**9]
                # Old logs have no pending snapshots. Their tested single-pending
                # protocol still identifies the one affected frame unambiguously.
                if not candidates and not diagnostic and len(pending)==1:candidates=list(pending)
                for s in candidates:
                    retire(s,'message_timeout',event,missing_kinds=sorted(set(KINDS)-pending[s]['kinds']))
                last_trigger={'reason':'message_timeout','stamp_ns':stamp}
            elif why in ('localization_stream_timeout','clock_rollback'):
                last_trigger={'reason':why,'stamp_ns':stamp}
                if why=='clock_rollback':
                    for s in list(pending):retire(s,'clock_rollback',event)
            continue
        if kind=='acquisition':
            if stamp not in rows:issues.append({'kind':'unknown_acquisition','stamp_ns':stamp});continue
            seen_acquisitions[stamp]+=1;rows[stamp]['acquisition']=event
            if stamp>event['ros_ns']:
                retire(stamp,'acquisition_clock_behind',event,clock_ns=event['ros_ns'])
            elif seen_acquisitions[stamp]>1:
                retire(stamp,'duplicate_acquisition',event)
            elif event['ros_ns']-stamp>10**9:
                retire(stamp,'acquisition_too_old',event)
            else:pending[stamp]={'begin_ns':event['monotonic_ns'],'kinds':set()}
            continue
        if not kind.startswith('received_'):continue
        stream=remove_prefix(kind,'received_')
        if stream not in KINDS:continue
        if stamp not in rows:issues.append({'kind':'unknown_gate_message','stamp_ns':stamp});continue
        row=rows[stamp];row['gate_callbacks'].append({k:event[k] for k in ('event','monotonic_ns','reason','state','ros_ns')})
        if 'diagnostic' in event:row['gate_callbacks'][-1]['diagnostic']=event['diagnostic']
        for output in event.get('approved',[]):
            s=output['stamp_ns'];outputs.append(s)
            if s not in rows:raise ValueError('approval of unknown input')
            raw=streams['localization_pose'][s]
            if len(raw)!=1 or output['pose']!=raw[0]['pose']:raise ValueError('approved pose differs from raw')
            if not rows[s]['raw_numeric_healthy']:raise ValueError('approval outside raw health')
            if rows[s]['first_block']:raise ValueError('approval after recorded rejection')
            rows[s]['primary_reason']='approved';pending.pop(s,None)
        if stamp not in pending:
            if row['primary_reason'] is None:
                retire(stamp,'message_without_registered_acquisition',event)
            continue
        item=pending[stamp]
        if stream in item['kinds']:block(stamp,'duplicate_message',event,stream=stream)
        item['kinds'].add(stream)
        if why=='unexpected_or_retired_stamp':
            # If retirement happened in ingest's tick, the following reason has
            # overwritten it. Keep this inferred case separate from explicit tick.
            age=(event['monotonic_ns']-item['begin_ns'])/1e9
            # A healthy partial frame leaves the previous global reason intact.
            # Do not mistake that inherited reason for rejection of this frame.
            after=event.get('diagnostic',{}).get('after')
            retired=(str(stamp) not in after['pending']) if after else (len(item['kinds'])==len(KINDS) or age>=.5)
            if retired:
                retire(stamp,'retirement_after_deadline' if age>=.5 else 'unexplained_retirement',event,
                       processing_log_age_s=age)
                continue
        if stream in row['raw_rejections']:
            block(stamp,reason_class(row['raw_rejections'][stream]),event,stream=stream)
        if len(item['kinds'])==len(KINDS):
            if row['first_block']:row['primary_reason']=row['first_block']['reason']
            elif why=='confirming_healthy_acquisitions':
                row['primary_reason']='reconfirmation';row['confirmation_trigger']=last_trigger.copy()
            else:row['primary_reason']='unexplained_complete_frame'
            pending.pop(stamp,None)
    if len(outputs)!=len(set(outputs)):raise ValueError('duplicate approval')
    for stamp,row in rows.items():
        if row['primary_reason'] is None:row['primary_reason']='unresolved_incomplete_frame'
        if stamp not in seen_acquisitions:row['missing_gate_acquisition']=True
        callbacks=row['gate_callbacks'];acquisition=row.get('acquisition')
        if acquisition and callbacks:
            row['postprocessing_completion_s']=(max(x['monotonic_ns'] for x in callbacks)-acquisition['monotonic_ns'])/1e9
            row['publication_to_acquisition_log_s']=(acquisition['monotonic_ns']-row['publication_monotonic_ns'])/1e9
    result_rows=list(rows.values())
    def summary(values):
        return {'published':len(values),'approved':sum(r['primary_reason']=='approved' for r in values),
                'raw_numeric_healthy':sum(r['raw_numeric_healthy'] for r in values),
                'primary_reasons':dict(Counter(r['primary_reason'] for r in values)),
                'healthy_unapproved_reasons':dict(Counter(r['primary_reason'] for r in values if r['raw_numeric_healthy'] and r['primary_reason']!='approved')),
                'raw_flags':dict(Counter(f for r in values for f in r['raw_health_flags'])),
                'confirmation_triggers':dict(Counter(r['confirmation_trigger']['reason'] for r in values if 'confirmation_trigger' in r))}
    cycles=sorted({r['cycle'] for r in result_rows})
    return {'summary':summary(result_rows),'cycles':{str(c):summary([r for r in result_rows if r['cycle']==c]) for c in cycles},
            'rows':result_rows,'issues':issues,'epoch_events':epoch_events,
            'protocol':'one row per actual published stamp; approval means emitted at least once, not subsequent consumer uptime',
            'timing_limit':'legacy timestamps are post-processing; new optional diagnostic stages are application callback timings, never DDS arrival times'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args()
    with sqlite3.connect((a.run/'map.db').resolve().as_uri()+'?mode=ro&immutable=1',uri=True) as db:ids={r[0] for r in db.execute('SELECT id FROM Node')}
    sources=[a.run/'observations.jsonl',a.run/'gate-events.jsonl']
    result=ledger(read_events(sources[0]),read_events(sources[1]),ids)
    result['source_sha256']={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}
    with a.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()
