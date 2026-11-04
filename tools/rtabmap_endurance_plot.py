"""Standalone diagnostic plot; approved/input is a frame ratio, not motion uptime."""
import argparse
import json
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/fpga-matplotlib-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    events=[json.loads(l) for l in (a.run/'observations.jsonl').read_text().splitlines()]
    gates=[json.loads(l) for l in (a.run/'gate-events.jsonl').read_text().splitlines()]
    telemetry=[json.loads(l) for l in (a.run/'process-telemetry.jsonl').read_text().splitlines()]
    pubs=[r for r in events if r['event']=='publish_begin'];start=pubs[0]['monotonic_ns']
    approved={o['stamp_ns'] for r in gates for o in r.get('approved',[])}
    figure,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for name in ['odometry','localization','gate','replay']:
        points=[((row['monotonic_ns']-start)/6e10,proc['rss_kib']/1024)
                for row in telemetry for proc in row['processes'] if proc['name']==name and 'rss_kib' in proc]
        axes[0].plot([p[0] for p in points],[p[1] for p in points],label=name)
    axes[0].set_ylabel('Process RSS (MiB)');axes[0].legend(ncol=4);axes[0].grid(alpha=.2)
    times=np.array([(r['monotonic_ns']-start)/6e10 for r in pubs]);bins=np.arange(0,np.ceil(times.max())+1)
    total,_=np.histogram(times,bins);good,_=np.histogram([t for t,r in zip(times,pubs) if r['stamp_ns'] in approved],bins)
    ratio=np.divide(good,total,out=np.zeros(len(good),float),where=total>0)*100
    axes[1].bar(bins[:-1]+.5,ratio,width=.8,label='approved / input frames per minute')
    axes[1].set_ylim(0,100);axes[1].set_ylabel('Approved frame ratio (%)');axes[1].set_xlabel('Wall time since first publication (min)');axes[1].legend()
    for event in events:
        if event['event']=='repeat_boundary':
            for ax in axes:ax.axvline((event['monotonic_ns']-start)/6e10,color='gray',linestyle='--',alpha=.6)
    figure.suptitle('Same-process repeated desk2 recording; dashed lines are artificial cycle boundaries')
    figure.tight_layout();figure.savefig(a.output,dpi=160);plt.close(figure)


if __name__=='__main__':main()
