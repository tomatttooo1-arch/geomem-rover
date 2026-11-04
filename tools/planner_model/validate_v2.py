"""Fixed v2 regression plus independent synthetic correctness/quality evaluation."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
from tools.b1_costmap.nav2_ref import InflationParams, inflate_exact
from tools.planner_model.aligned_v2 import (InputContractError, MapMetadata, PoseSample,
                                           PlannerV2, grid_to_world)
from tools.planner_model.evaluate import Geometry, Planner, distribution, length
from tools.planner_model.success_probe import reachable

SEED = 20263102


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def oracle(m, start, goal, tolerance=.15):
    """Independent BFS reachability; no wavefront potentials or C++ planner calls."""
    if m[start[1], start[0]]>=253 or min(start)==0 or start[0]==m.shape[1]-1 or start[1]==m.shape[0]-1:
        return 'invalid_start'
    gx, gy = map(int, goal)
    if m[gy, gx]>=253:
        return 'invalid_goal'
    seen = reachable(m, tuple(start))
    if seen[gy, gx]:
        return 'exact'
    h, w = m.shape
    geometry = Geometry(m)
    px, py = float(gx)*.05, float(gy)*.05
    y = py-tolerance
    while y <= py+tolerance:
        x = px-tolerance
        while x <= px+tolerance:
            if x>=0 and y>=0:
                a,b = int(math.floor(x/.05+.5)),int(math.floor(y/.05+.5))
                if 0<=a<w and 0<=b<h and seen[b,a]:
                    if not geometry.collisions(np.array([[a,b],[x/.05,y/.05]])):
                        return 'within_tolerance'
            x += .05
        y += .05
    return 'no_path'


def generated_cases():
    params = InflationParams()
    cases = []
    for i in range(8):
        m = np.zeros((80,80), np.int32)
        m[0,:]=m[-1,:]=m[:,0]=m[:,-1]=254
        kind=i%4
        if kind==0:
            m[1:79,40]=254;m[24:52,40]=0
            m[12:17,20:27]=254;m[58:63,53:59]=254
        elif kind==1:
            m[1:79,25]=254;m[12:36,25]=0
            m[1:79,50]=254;m[44:70,50]=0
        elif kind==2:
            for x,y in [(25,20),(50,22),(32,55),(61,58)]:
                m[y:y+4,x:x+5]=254
        else:
            m[12:35,56:66]=255
            m[55:60,25:35]=254
        if i>=4:
            m[7:11,35:40]=254  # fixed second layout variation, no tuning
        m = inflate_exact(m,params).astype(np.uint8)
        start=(14,40)
        assert m[start[1],start[0]]<253
        yy,xx=np.nonzero(m<253)
        possible=[(int(x),int(y)) for x,y in zip(xx,yy) if (x,y)!=start]
        rng=np.random.RandomState(SEED+i)
        targets=[possible[int(j)] for j in rng.choice(len(possible),20,replace=False)]
        edge=[p for p in possible if min(p[0],p[1],79-p[0],79-p[1])<=12 and p not in targets]
        edge.sort(key=lambda p:(-((p[0]-start[0])**2+(p[1]-start[1])**2),p[1],p[0]))
        assert len(edge)>=5
        cases.append(('layout_%d'%(i+1),m,start,targets+edge[:5]))
    for i,strip in enumerate([35,40,45]):
        m=np.zeros((80,80),np.int32)
        m[0,:]=m[-1,:]=m[:,0]=m[:,-1]=254
        m[:,strip]=255
        m=inflate_exact(m,params).astype(np.uint8)
        targets=[(x,y) for x in [strip+1,strip+2,strip+5,70] for y in [12,23,35,48,60,68]]
        targets.append((25,40))
        cases.append(('unknown_split_%d'%(i+1),m,(14,40),targets))
    return cases


def compare(v1,v2,m,start,goal,label,index,pair,old=None,expected=None,old_path=None):
    geom=Geometry(m)
    before=hashlib.sha256(m.tobytes()).hexdigest()
    old_wave,_=v1.run(m,start,goal,'float')
    nav,_=v1.run(m,start,goal,'navfn')
    a=v2.plan_cells(m,start,goal,mode='float')
    b=v2.plan_cells(m,start,goal,mode='fixed')
    same=(a['status']==b['status'] and a['rounds']==b['rounds'] and np.array_equal(a['path'],b['path']) and
          (not a['success'] or np.array_equal(a['resolved_goal'],b['resolved_goal'])))
    hits=geom.collisions(b['path'])
    if b['success']:
        assert np.array_equal(b['path'][0],start)
        assert np.max(np.abs(b['path'][-1]-goal))<=3.00000001
        if b['status']=='exact':assert np.allclose(b['path'][-1],goal,atol=1e-12)
    stable=True
    if old is not None:
        stable=(bool(len(old_wave))==old['float_success'] and bool(len(nav))==old['navfn_success'])
        stable=stable and np.array_equal(old_wave,np.asarray(old_path['wave']).reshape((-1,2)))
        stable=stable and np.array_equal(nav,np.asarray(old_path['navfn']).reshape((-1,2)))
    assert before==hashlib.sha256(m.tobytes()).hexdigest()
    row=dict(source=label,frame=index,pair=pair,start=list(start),goal=list(goal),start_cost=int(m[start[1],start[0]]),
             old_success=bool(len(old_wave)),navfn_success=bool(len(nav)),v2_success=b['success'],status=b['status'],
             resolved_goal=b['resolved_goal'],rounds=b['rounds'],cells=m.size,fixed_equal=same,
             safety_hits=hits,navfn_hits=geom.collisions(nav),
             old_length=length(old_wave),navfn_length=length(nav),v2_length=length(b['path']),
             navfn_clearance=geom.clearance(nav),v2_clearance=geom.clearance(b['path']),
             baseline_unchanged=stable,expected_status=expected,
             functional_match=(expected is None or b['status']==expected))
    path=dict(source=label,frame=index,pair=pair,v1=old_wave,navfn=nav,v2=b['path'])
    return row,path


def summary(rows):
    base=[r for r in rows if r['navfn_success']]
    both=[r for r in base if r['v2_success'] and r['navfn_length']>1e-12]
    ratios=[r['v2_length']/r['navfn_length'] for r in both]
    navclear=[r['navfn_clearance'] for r in base]
    clear=[r['v2_clearance'] for r in base if r['v2_success']]
    n=len(base)
    rate=sum(r['v2_success'] for r in base)/n if n else None
    lengths=distribution(ratios)
    cnav=float(np.median(navclear)) if navclear else None
    cnew=float(np.median(clear)) if clear else None
    fixed=sum(r['fixed_equal'] for r in base)/n if n else None
    result=dict(requests=len(rows),old_success=sum(r['old_success'] for r in rows),navfn_success=n,
                v2_success=sum(r['v2_success'] for r in rows),all_request_success_rate=sum(r['v2_success'] for r in rows)/len(rows) if rows else None,
                v2_on_navfn_success=sum(r['v2_success'] for r in base),success_rate=rate,
                status_counts=dict(Counter(r['status'] for r in rows)),length_ratio=lengths,
                safety_cells=sum(len(r['safety_hits']) for r in rows),navfn_unsafe_paths=sum(bool(r['navfn_hits']) for r in rows),
                clearance_navfn_median=cnav,clearance_v2_median=cnew,fixed_match_rate=fixed,
                all_outputs_fixed_equal=all(r['fixed_equal'] for r in rows),
                baseline_unchanged=all(r['baseline_unchanged'] for r in rows),
                functional_matches=sum(r['functional_match'] for r in rows if r['expected_status'] is not None),
                functional_total=sum(r['expected_status'] is not None for r in rows),
                rounds=distribution([r['rounds'] for r in rows if r['rounds']]),
                cycles=distribution([r['rounds']*r['cells']*2 for r in rows if r['rounds']]))
    result['gates']=dict(safety=n>0 and not any(r['safety_hits'] for r in base),
                         success=rate is not None and rate>=.99,
                         length=bool(ratios) and lengths['median']<=1.05 and lengths['p95']<=1.15,
                         clearance=bool(clear) and bool(navclear) and cnew>=cnav-1,
                         fixed=fixed is not None and fixed>=.99)
    return result


def alignment_controls(v2):
    m=np.zeros((80,80),np.uint8);m[38:43,38:43]=254
    meta=MapMetadata('odom',12,123000,.05,10.,-3.,math.pi/2,'cell_center')
    start=np.array([10.2,10.1]);goal=np.array([23.25,16.1])
    sw,gw=grid_to_world(meta,[start,goal])
    pose=PoseSample('odom',12,123000,float(sw[0]),float(sw[1]),'test_fixture')
    good=v2.plan(m,meta,pose,gw,'odom')
    assert good['success'] and np.allclose(good['world_path'][0],sw)
    assert np.allclose(good['world_path'][-1],gw) and good['start_cell']==(10,10)
    assert not Geometry(m).collisions(good['path'])
    wrong=v2.plan_cells(m,(40,40),(23,16))
    assert wrong['status']=='invalid_start'
    checks=[dict(name='rotated_translated_true_pose',status=good['status'],expected='exact',success=True),
            dict(name='incorrect_window_center',status=wrong['status'],expected='invalid_start',success=False)]
    for name,badpose,goalframe in [
        ('missing_pose',None,'odom'),('wrong_pose_frame',replace(pose,frame_id='map'),'odom'),
        ('wrong_sequence',replace(pose,sequence=13),'odom'),
        ('wrong_timestamp',replace(pose,stamp_ns=123001),'odom'),
        ('wrong_goal_frame',pose,'map'),
        ('missing_provenance',replace(pose,provenance=''),'odom')]:
        try:v2.plan(m,meta,badpose,gw,goalframe)
        except InputContractError as error:checks.append(dict(name=name,status=str(error),expected='input_error',success=False))
        else:raise AssertionError(name)
    # Actual source/goal segments, not just representative cell vertices, must be checked.
    boundary=np.zeros((20,20),np.uint8);boundary[10,10]=255
    bm=MapMetadata('odom',1,1,.05,0.,0.,0.,'navfn_round')
    bp=PoseSample('odom',1,1,.475,.5,'test_fixture') # raw (9.5,10), rounds to blocked (10,10)
    br=v2.plan(boundary,bm,bp,(.7,.5),'odom')
    assert not br['success']
    checks.append(dict(name='actual_pose_on_unknown_boundary',status=br['status'],expected='failure',success=False))
    return dict(checks=checks,_map=m,world_request=dict(map=meta.__dict__,pose=pose.__dict__,goal_world=gw),
                path=good['path'],world_path=good['world_path'])


def tolerance_edge_controls(v1, v2):
    # Compare both directions at 0 / 1 / 2 / 3-cell floating tolerance boundaries
    # against the actual pinned C++ NavFn wrapper, not an idealized square.
    m=np.zeros((48,48),np.uint8);m[:,24]=255
    rows=[]
    for start,goals in [((8,20),[(25,20),(26,20),(27,20)]),
                        ((38,20),[(23,20),(22,20),(21,20)])]:
        for goal in goals:
            for tolerance in [0.,.05,.10,.15,float(np.nextafter(.15,0.))]:
                cap=m.size+2;x=np.zeros(cap);y=np.zeros(cap)
                n=v1.lib.navfn_plan(m,48,48,*start,*goal,x,y,cap,tolerance,1)
                nav=np.column_stack((x[:max(0,n)],y[:max(0,n)]))
                a=v2.plan_cells(m,start,goal,tolerance=tolerance,mode='float')
                b=v2.plan_cells(m,start,goal,tolerance=tolerance,mode='fixed')
                same=bool(n>0)==b['success']
                if n>0 and b['success']:
                    same=same and np.allclose(nav[-1],b['resolved_goal'],atol=1e-12,rtol=0.)
                equal=(a['status']==b['status'] and np.array_equal(a['path'],b['path']))
                safe=not Geometry(m).collisions(b['path'])
                rows.append(dict(start=start,goal=goal,tolerance=tolerance,navfn_success=n>0,
                                 v2_status=b['status'],resolved_goal=b['resolved_goal'],
                                 navfn_endpoint=nav[-1] if n>0 else None,
                                 matches_navfn=bool(same),fixed_equal=equal,safe=safe))
    assert all(r['matches_navfn'] and r['fixed_equal'] and r['safe'] for r in rows)
    return rows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--baseline',default='runs/planner-model-final-20261102')
    ap.add_argument('--out',required=True)
    a=ap.parse_args();out=Path(a.out);base=Path(a.baseline)
    if out.exists() and any(out.iterdir()):ap.error('--out must be a new or empty directory')
    out.mkdir(parents=True,exist_ok=True)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    (out/'CODE_COMMIT.txt').write_text(commit+'\n')
    subprocess.run(['cmake','-S','tools/planner_model','-B',str(out/'build'),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(out/'build'),'-j2'],check=True)
    subprocess.run(['ctest','--output-on-failure'],cwd=str(out/'build'),check=True)
    library=out/'build/libplanner_model.so';v1=Planner(library);v2=PlannerV2(library)
    edges=tolerance_edge_controls(v1,v2)
    (out/'tolerance_edges.json').write_text(json.dumps(edges,default=serial,indent=2)+'\n')
    controls=alignment_controls(v2)
    np.save(out/'aligned_example_costmap.npy',controls.pop('_map'))
    wr=controls['world_request']
    request=dict(map_sha256=sha(out/'aligned_example_costmap.npy'),metadata=wr['map'],pose=wr['pose'],
                 goal=dict(frame_id=wr['map']['frame_id'],x=float(wr['goal_world'][0]),y=float(wr['goal_world'][1])))
    (out/'aligned_example_request.json').write_text(json.dumps(request,indent=2)+'\n')
    cli=[sys.executable,'-m','tools.planner_model.plan_v2_request','--map',str(out/'aligned_example_costmap.npy'),
         '--request',str(out/'aligned_example_request.json'),'--lib',str(library),'--out',str(out/'aligned_example_result.json')]
    subprocess.run(cli,check=True)
    bad=dict(request,map_sha256='0'*64)
    (out/'bad_checksum_request.json').write_text(json.dumps(bad,indent=2)+'\n')
    trial=cli.copy();trial[trial.index('--request')+1]=str(out/'bad_checksum_request.json')
    trial[trial.index('--out')+1]=str(out/'bad_checksum_result_must_not_exist.json')
    negative=subprocess.run(trial,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    assert negative.returncode==2 and 'map_checksum_mismatch' in negative.stderr
    assert not (out/'bad_checksum_result_must_not_exist.json').exists()
    (out/'cli_negative.log').write_text(negative.stderr)
    (out/'alignment_controls.json').write_text(json.dumps(controls,default=serial,indent=2)+'\n')
    previous=[json.loads(x) for x in (base/'results/pairs.jsonl').read_text().splitlines()]
    oldpaths={(p['frame'],p['pair']):p for p in map(json.loads,(base/'results/paths.jsonl').read_text().splitlines())}
    local=np.load(base/'input/maps.npz')['maps'];large=np.load(base/'results/large_maps.npz')['maps']
    records=[];paths=[]
    for i,r in enumerate(previous):
        m=local[r['frame']-1] if r['frame']<=376 else large[r['frame']-1001]
        row,path=compare(v1,v2,m,tuple(r['start']),tuple(r['goal']),r['source'],r['frame'],r['pair'],r,
                         old_path=oldpaths[(r['frame'],r['pair'])])
        records.append(row);paths.append(path)
        if (i+1)%250==0:print('legacy regression pairs',i+1,flush=True)
    new=[];newpaths=[];case_info=[];case_maps=[]
    for i,(label,m,start,targets) in enumerate(generated_cases(),1):
        for j,goal in enumerate(targets):
            expected=oracle(m,start,goal)
            row,path=compare(v1,v2,m,start,goal,label,i,j,expected=expected)
            new.append(row);newpaths.append(path)
        case_maps.append(m)
        case_info.append(dict(source=label,start=start,targets=targets))
        print('independent synthetic case',label,'done',flush=True)
    np.savez_compressed(out/'new_maps.npz',maps=np.asarray(case_maps))
    (out/'new_requests.json').write_text(json.dumps(case_info,default=serial,indent=2)+'\n')
    (out/'pairs.jsonl').write_text(''.join(json.dumps(dict(dataset='legacy',**r),default=serial)+'\n' for r in records)+
                                 ''.join(json.dumps(dict(dataset='synthetic',**r),default=serial)+'\n' for r in new))
    (out/'paths.jsonl').write_text(''.join(json.dumps(dict(dataset='legacy',**p),default=serial)+'\n' for p in paths)+
                                 ''.join(json.dumps(dict(dataset='synthetic',**p),default=serial)+'\n' for p in newpaths))
    subsets={'legacy_all':records,'legacy_holdout':[r for r in records if 100<r['frame']<=376],
             'legacy_valid_start':[r for r in records if r['start_cost']<253],
             'legacy_large':[r for r in records if r['frame']>376],
             'synthetic_all':new,'synthetic_layouts':[r for r in new if r['source'].startswith('layout')],
             'synthetic_tolerance':[r for r in new if r['source'].startswith('unknown_split')]}
    summaries={name:summary(rr) for name,rr in subsets.items()}
    checks=dict(tolerance_edge_cases_match_navfn=all(r['matches_navfn'] for r in edges),
                old_implementation_unchanged=all(r['baseline_unchanged'] for r in records),
                zero_unsafe_paths=not any(r['safety_hits'] for r in records+new),
                float_fixed_identical=all(r['fixed_equal'] for r in records+new),
                new_functional_reference_matches=all(r['functional_match'] for r in new),
                legacy_blocked_starts_rejected=all(r['status']=='invalid_start' for r in records if r['start_cost']>=253))
    result=dict(code_commit=commit,baseline_commit=(base/'CODE_COMMIT.txt').read_text().strip(),seed=SEED,
                baseline_pairs_sha256=sha(base/'results/pairs.jsonl'),checks=checks,summaries=summaries)
    (out/'summary.json').write_text(json.dumps(result,default=serial,indent=2)+'\n')
    print(json.dumps(result,default=serial,indent=2),flush=True)
    if not all(checks.values()):raise SystemExit('v2 correctness validation FAILED; inspect preserved evidence')


if __name__=='__main__':main()
