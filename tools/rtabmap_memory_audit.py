"""Compare allocator accounting and proc pages at fixed cycle/idle checkpoints."""
import argparse
import bisect
import json
from pathlib import Path
import statistics
from tools.rtabmap_memory_probe import allocator_totals


def window_summary(samples,name,begin,end):
    selected=[(s,r) for s in samples if begin<=s['monotonic_ns']<end
              for r in s['processes'] if r['name']==name and not r.get('exited')]
    keys=('Rss_kib','Pss_kib','Anonymous_kib','Private_Dirty_kib','Pss_Anon_kib','Pss_File_kib','Pss_Shmem_kib')
    return {'samples':len(selected),'begin_ns':begin,'end_ns':end,
            **{k:statistics.median(r[k] for _,r in selected if k in r) for k in keys if any(k in r for _,r in selected)}}


def analyze(run):
    events=[json.loads(l) for l in (run/'observations.jsonl').open()]
    samples=[json.loads(l) for l in (run/'memory-telemetry.jsonl').open()]
    name='localization_target' if any(r['name']=='localization_target' for s in samples for r in s['processes']) else 'localization'
    pubs=[r for r in events if r['event']=='publish_begin'];times=[r['monotonic_ns'] for r in pubs]
    checkpoints=[]
    for event in events:
        if event['event']=='memory_checkpoint':
            row={k:event[k] for k in ('cycle','phase','monotonic_ns')}
            row['allocator_response']=event['allocator']
            row['allocator_before']=allocator_totals(run/'allocator'/Path(event['allocator']['before']).name)
            row['allocator_after']=allocator_totals(run/'allocator'/Path(event['allocator']['after']).name)
            checkpoints.append(row)
    curve=[]
    for s in samples:
        i=bisect.bisect_right(times,s['monotonic_ns'])-1
        for r in s['processes']:
            if r['name']==name and not r.get('exited'):
                curve.append({'monotonic_ns':s['monotonic_ns'],'source_index':pubs[i]['index'] if i>=0 else None,
                    'cycle':pubs[i].get('cycle',0) if i>=0 else None,**r})
    trims=[r for r in checkpoints if r['phase']=='trim']
    trim_comparison=None
    if trims:
        t=trims[0]['allocator_response']
        before=window_summary(samples,name,t['begin_ns']-5*10**9,t['begin_ns'])
        after=window_summary(samples,name,t['end_ns']+10**9,t['end_ns']+6*10**9)
        trim_comparison={'before':before,'after':after,
            'released_rss_kib':before['Rss_kib']-after['Rss_kib'] if before['samples']>=2 and after['samples']>=2 else None,
            'released_pss_kib':before['Pss_kib']-after['Pss_kib'] if before['samples']>=2 and after['samples']>=2 else None,
            'in_use_proxy_change_bytes':trims[0]['allocator_after']['in_use_proxy_bytes']-trims[0]['allocator_before']['in_use_proxy_bytes'],
            'rule':'disjoint steady idle windows; never pair a pre-trim event with a nearest post-trim sample'}
    pids={r['pid'] for r in curve};runner=json.loads((run/'runner-result.json').read_text())
    complete=[r for r in events if r['event']=='complete']
    summary={'target_name':name,'target_pids':sorted(pids),'same_target_pid':len(pids)==1,
        'published':len(pubs),'cycles_started':max((r.get('cycle',0) for r in pubs),default=-1)+1,
        'complete':complete,'runner':runner,'target_samples':len(curve),
        'trim':trim_comparison}
    return {'summary':summary,'checkpoints':checkpoints,'curve':curve,
            'limits':['allocator in-use proxy is not an exact heap object inventory',
                      'source_index on proc samples identifies latest publication, not callback completion',
                      'profiler changes workload; its frame approvals are not an availability improvement',
                      'short repetition cannot establish a long-term bound or reproduce all full-recording allocations']}


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);p.add_argument('--plot',type=Path)
    a=p.parse_args();result=analyze(a.run)
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    if a.plot:
        if a.plot.exists():raise ValueError('plot exists')
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(10,4.8),layout='constrained');curve=result['curve'];start=curve[0]['monotonic_ns']
        for key,label in [('Rss_kib','Process RSS'),('Pss_kib','Process PSS')]:
            ax.plot([(r['monotonic_ns']-start)/1e9 for r in curve],[r[key]/1024 for r in curve],label=label,linewidth=1.4)
        points=result['checkpoints']
        for key,label in [('in_use_proxy_bytes','Allocator in-use proxy'),('arena_free_bytes','Allocator free bytes')]:
            ax.plot([(r['monotonic_ns']-start)/1e9 for r in points],[r['allocator_after'][key]/2**20 for r in points],'.-',label=label)
        for r in points:
            if r['phase']=='trim':ax.axvline((r['monotonic_ns']-start)/1e9,color='black',linestyle='--',label='Diagnostic trim')
        ax.set(xlabel='Wall time (s)',ylabel='MiB',title=a.run.name+' — localization memory accounting');ax.grid(alpha=.2);ax.legend(loc='best');fig.savefig(a.plot,dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in result['summary'].items() if k not in ('complete','runner')},indent=2))


if __name__=='__main__':main()
