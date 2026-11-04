"""Fixed v3 experiment: start-policy compatibility and geometric quality ablations."""
import argparse
import heapq
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from tools.planner_model.aligned_v2 import PlannerV2
from tools.planner_model.aligned_v3 import PlannerV3
from tools.planner_model.evaluate import Geometry, Planner, distribution, length
from tools.planner_model.validate_v2 import serial, summary as summary_v2, oracle, tolerance_edge_controls
from tools.b1_costmap.nav2_ref import InflationParams, inflate_exact


def independent_cost(m,start,goal):
    """Heap Dijkstra, independent of the scan implementation and its descent."""
    m=m.copy();sx,sy=start;m[sy,sx]=0
    m[0,:]=m[-1,:]=m[:,0]=m[:,-1]=254
    h,w=m.shape;dist={tuple(start):0};queue=[(0,sx,sy)]
    while queue:
        cost,x,y=heapq.heappop(queue)
        if cost!=dist[(x,y)]:continue
        if (x,y)==tuple(goal):return cost
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                if not(dx or dy):continue
                a,b=x+dx,y+dy
                if not(0<=a<w and 0<=b<h) or m[b,a]>=253:continue
                if dx and dy and (m[y,a]>=253 or m[b,x]>=253):continue
                new=cost+int(50+.8*int(m[b,a]))*(14 if dx and dy else 10)
                if new<dist.get((a,b),math.inf):
                    dist[(a,b)]=new;heapq.heappush(queue,(new,a,b))
    return None


def path_cost(m,path):
    total=0
    for a,b in zip(path[:-1],path[1:]):
        x,y=map(int,b);d=np.abs(b-a)
        assert np.max(d)==1 and np.all(d<=1)
        total+=int(50+.8*int(m[y,x]))*(14 if np.all(d) else 10)
    return total


def fresh_cases():
    # Fixed layouts/seed committed before the first run; not real vehicle data.
    for i in range(12):
        m=np.zeros((80,80),np.int32)
        m[0,:]=m[-1,:]=m[:,0]=m[:,-1]=254
        if i%3==0:
            m[:,39]=254;m[23+i%4:53+i%4,39]=0
            m[10:16,20:26]=254;m[60:65,54:60]=254
        elif i%3==1:
            m[:,27]=254;m[12:38,27]=0
            m[:,53]=254;m[43:71,53]=0
        else:
            for x,y in [(27,17),(50,26),(33,56),(60,60)]:m[y:y+3,x:x+4]=254
            m[8:25,65:71]=255
        m=inflate_exact(m,InflationParams()).astype(np.uint8)
        start=(14,40);assert m[start[1],start[0]]<253
        yy,xx=np.nonzero(m<253);goals=[(int(x),int(y)) for x,y in zip(xx,yy) if (x,y)!=start]
        rng=np.random.RandomState(20265102+i)
        targets=[goals[int(j)] for j in rng.choice(len(goals),25,replace=False)]
        yield 'fresh_%02d'%i,m,start,targets


def controls(v3,v1):
    checks=[]
    for cost in [0,253,254,255]:
        m=np.zeros((48,48),np.uint8);m[20,8]=cost
        r=v3.plan_cells(m,(8,20),(38,20))
        effective=m.copy();effective[20,8]=0
        assert r['success'] and not Geometry(effective).collisions(r['path'])
        hits=Geometry(m).collisions(r['path'])
        assert bool(hits)==(cost>=253) and all((h[0],h[1])==(8,20) for h in hits)
        checks.append(dict(name='clear_start_%d'%cost,passed=True,original_hits=hits))
    m=np.zeros((48,48),np.uint8);m[19:22,7:10]=254
    assert v3.plan_cells(m,(8,20),(38,20))['status']=='no_path'
    checks.append(dict(name='enclosed_start',passed=True))
    m=np.zeros((48,48),np.uint8);m[20,9]=254;m[20,8]=253
    r=v3.plan_cells(m,(8,20),(38,20));assert r['success']
    effective=m.copy();effective[20,8]=0
    assert not Geometry(effective).collisions(r['path']) and m[20,9]==254
    checks.append(dict(name='blocked_neighbor_not_cleared',passed=True))
    # Independent optimal weighted cost on fixed small clutter maps.
    rng=np.random.RandomState(20265102)
    for i in range(12):
        m=np.zeros((28,28),np.uint8)
        m[rng.random_sample(m.shape)<.13]=254
        m[14,3]=0;m[14,24]=0
        free=m==0;m[free]=rng.choice([0,0,0,50,150],int(free.sum()))
        r=v3.plan_cells(m,(3,14),(24,14),variant=1,tolerance=0)
        ref=independent_cost(m,(3,14),(24,14))
        assert r['success']==(ref is not None)
        if ref is not None:assert path_cost(m,r['path'])==ref
        checks.append(dict(name='dijkstra_%d'%i,passed=True,cost=ref))
    # Reuse literal floating tolerance edge tests with actual pinned NavFn.
    edges=tolerance_edge_controls(v1,v3)
    return checks,edges


def summarize(rows,variant):
    # Keep existing summary definition, including complete NavFn-success denominator.
    rr=[]
    for r in rows:
        q=dict(r);v=r['variants'][str(variant)]
        q.update(v2_success=v['success'],v2_length=v['length'],v2_clearance=v['clearance'],
                 fixed_equal=v['fixed_equal'],safety_hits=v['effective_hits'],status=v['status'],rounds=v['rounds'])
        rr.append(q)
    result=summary_v2(rr)
    result['original_map_unsafe_paths']=sum(bool(r['variants'][str(variant)]['original_hits']) for r in rows)
    result['original_safety_gate']=result['original_map_unsafe_paths']==0
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--baseline',default='runs/planner-model-final-20261102')
    ap.add_argument('--v2',default='runs/planner-v2-delivery-20261102')
    ap.add_argument('--out',required=True)
    a=ap.parse_args();out=Path(a.out);base=Path(a.baseline);v2dir=Path(a.v2)
    if out.exists():ap.error('output must not exist')
    out.mkdir(parents=True)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    (out/'CODE_COMMIT.txt').write_text(commit+'\n')
    subprocess.run(['cmake','-S','tools/planner_model','-B',str(out/'build'),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(out/'build'),'-j2'],check=True)
    subprocess.run(['ctest','--output-on-failure'],cwd=str(out/'build'),check=True)
    lib=out/'build/libplanner_model.so';v1=Planner(lib);v2=PlannerV2(lib);v3=PlannerV3(lib)
    cc,edges=controls(v3,v1)
    (out/'controls.json').write_text(json.dumps(cc,default=serial,indent=2)+'\n')
    (out/'tolerance_edges.json').write_text(json.dumps(edges,default=serial,indent=2)+'\n')
    oldrows=[json.loads(x) for x in (v2dir/'pairs.jsonl').read_text().splitlines()]
    oldpaths={(r['dataset'],r['frame'],r['pair']):r for r in map(json.loads,(v2dir/'paths.jsonl').read_text().splitlines())}
    local=np.load(base/'input/maps.npz')['maps'];large=np.load(base/'results/large_maps.npz')['maps']
    newmaps=np.load(v2dir/'new_maps.npz')['maps']
    tasks=[]
    for r in oldrows:
        m=(local[r['frame']-1] if r['frame']<=376 else large[r['frame']-1001]) if r['dataset']=='legacy' else newmaps[r['frame']-1]
        tasks.append((r['dataset'],r['frame'],r['pair'],m,tuple(r['start']),tuple(r['goal']),r['source'],r))
    freshmaps=[];freshrequests=[]
    for frame,(label,m,start,targets) in enumerate(fresh_cases(),1):
        freshmaps.append(m);freshrequests.append(dict(source=label,start=start,targets=targets))
        for pair,goal in enumerate(targets):tasks.append(('fresh',frame,pair,m,start,goal,label,None))
    np.savez_compressed(out/'fresh_maps.npz',maps=np.asarray(freshmaps))
    (out/'fresh_requests.json').write_text(json.dumps(freshrequests,default=serial,indent=2)+'\n')
    records=[];allpaths=[]
    # Stream evidence as it is produced; preserve partial output on any failure.
    with (out/'pairs.jsonl').open('w') as rf,(out/'paths.jsonl').open('w') as pf:
      for i,(dataset,frame,pair,m,start,goal,source,old) in enumerate(tasks):
        before=m.tobytes();effective=m.copy();effective[start[1],start[0]]=0
        geom=Geometry(effective);original_geom=Geometry(m)
        nav,_=v1.run(m,start,goal,'navfn')
        previous=oldpaths.get((dataset,frame,pair))
        stable=previous is None or np.array_equal(nav,np.asarray(previous['navfn']).reshape(-1,2))
        if previous is not None:
            old_v2=v2.plan_cells(m,start,goal)
            stable=stable and np.array_equal(old_v2['path'],np.asarray(previous['v2']).reshape(-1,2)) and old_v2['status']==old['status']
        variants={};paths={};functional=True
        expected=oracle(effective,start,goal)
        for variant in [0,1,2]:
            f=v3.plan_cells(m,start,goal,mode='float',variant=variant)
            r=v3.plan_cells(m,start,goal,mode='fixed',variant=variant)
            same=f['status']==r['status'] and f['rounds']==r['rounds'] and np.array_equal(f['path'],r['path'])
            hits=geom.collisions(r['path']);original_hits=original_geom.collisions(r['path'])
            assert not hits and all((x,y)==start for x,y,c in original_hits)
            assert not r['success'] or (np.array_equal(r['path'][0],start) and np.allclose(r['path'][-1],r['resolved_goal'],atol=1e-12))
            functional=functional and r['status']==expected
            variants[str(variant)]=dict(success=r['success'],status=r['status'],rounds=r['rounds'],fixed_equal=same,
                length=length(r['path']),clearance=geom.clearance(r['path']),original_clearance=original_geom.clearance(r['path']),
                effective_hits=hits,original_hits=original_hits,resolved_goal=r['resolved_goal'])
            paths[str(variant)]=r['path']
        weighted,smoothed=variants['1'],variants['2']
        smoothing_ok=(weighted['status']==smoothed['status'] and (not weighted['success'] or (
            np.array_equal(paths['1'][[0,-1]],paths['2'][[0,-1]]) and
            smoothed['length']<=weighted['length']+1e-10 and smoothed['clearance']+1e-8>=weighted['clearance'])))
        assert before==m.tobytes()
        row=dict(dataset=dataset,source=source,frame=frame,pair=pair,start=start,goal=goal,start_cost=int(m[start[1],start[0]]),
            old_success=old['old_success'] if old else bool(len(v1.run(m,start,goal,'float')[0])),navfn_success=bool(len(nav)),navfn_length=length(nav),
            navfn_clearance=geom.clearance(nav),navfn_original_clearance=original_geom.clearance(nav),navfn_hits=original_geom.collisions(nav),
            navfn_effective_hits=geom.collisions(nav),baseline_unchanged=stable,expected_status=expected,
            functional_match=functional,smoothing_ok=smoothing_ok,cells=m.size,variants=variants)
        path=dict(dataset=dataset,frame=frame,pair=pair,navfn=nav,variants=paths)
        rf.write(json.dumps(row,default=serial)+'\n');rf.flush()
        pf.write(json.dumps(path,default=serial)+'\n');pf.flush()
        records.append(row)
        if (i+1)%100==0:print('v3 pairs',i+1,'/',len(tasks),flush=True)
    subsets=dict(legacy_all=[r for r in records if r['dataset']=='legacy'],
        legacy_holdout=[r for r in records if r['dataset']=='legacy' and 100<r['frame']<=376],
        legacy_large=[r for r in records if r['dataset']=='legacy' and r['frame']>376],
        synthetic=[r for r in records if r['dataset']=='synthetic'],fresh=[r for r in records if r['dataset']=='fresh'])
    summaries={name:{str(v):summarize(rr,v) for v in [0,1,2]} for name,rr in subsets.items()}
    checks=dict(navfn_unchanged=all(r['baseline_unchanged'] for r in records),functional_reference=all(r['functional_match'] for r in records),
        smooth_nonworsening=all(r['smoothing_ok'] for r in records),float_fixed=all(v['fixed_equal'] for r in records for v in r['variants'].values()),
        zero_effective_hits=all(not v['effective_hits'] for r in records for v in r['variants'].values()))
    result=dict(code_commit=commit,seed=20265102,checks=checks,summaries=summaries)
    (out/'summary.json').write_text(json.dumps(result,default=serial,indent=2)+'\n')
    print(json.dumps(result,default=serial,indent=2),flush=True)
    if not all(checks.values()):raise SystemExit('v3 correctness FAILED; inspect preserved evidence')


if __name__=='__main__':main()
