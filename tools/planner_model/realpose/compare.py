"""Compare fixed v3 variants 1/2 with continuous-goal pinned NavFn on extracted bags."""
import argparse
from collections import Counter
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from tools.planner_model.aligned_v2 import (F64,U8,InputContractError,MapMetadata,PoseSample,world_to_grid,grid_to_world,checked_map)
from tools.planner_model.aligned_v3 import PlannerV3
from tools.planner_model.evaluate import Geometry,distribution
from tools.planner_model.realpose.core import MissingInput,Transforms,select_map,to_raw,from_raw,inside,first_exit,transform_xy
from tools.planner_model.validate_v2 import serial

FROZEN='21069ecedb8865e32ca801b6fc9244e5ffbc3835'


def verify_frozen():
    for name in ['model_v3.cpp','model_v3.hpp','aligned_v3.py','model.cpp','model_v2.cpp','vendor/navfn.cpp']:
        path='tools/planner_model/'+name
        expected=subprocess.check_output(['git','show',FROZEN+':'+path])
        if Path(path).read_bytes()!=expected:raise ValueError('frozen_v3_source_changed:'+path)


class NavFn:
    def __init__(self,library):
        self.lib=ctypes.CDLL(str(Path(library).resolve()))
        self.lib.navfn_realpose.argtypes=[U8]+[ctypes.c_int]*4+[ctypes.c_double]*4+[F64,F64,ctypes.c_int]
        self.lib.navfn_realpose.restype=ctypes.c_int
    def plan(self,m,start,goal,resolution):
        h,w=m.shape;cap=w*h+2;x=np.zeros(cap);y=np.zeros(cap)
        n=self.lib.navfn_realpose(m,w,h,*start,*goal,resolution,.15,x,y,cap)
        if n<0:raise ValueError('navfn_invalid_arguments')
        return np.column_stack((x[:n],y[:n]))


def append_actual_start(path,actual):
    if len(path) and not np.array_equal(path[0],actual):return np.vstack((actual,path))
    return path


def metric(path,geometry,resolution):
    return dict(success=bool(len(path)),length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()*resolution) if len(path) else None,
                clearance=geometry.clearance(path),hits=geometry.collisions(path))


def choose_goal(grid,request,plans,transforms,start_world):
    t=request['stamp_ns'];frame=grid['frame_id']
    goal_tf,proof=transforms.lookup(frame,request['frame_id'],t)
    requested=transform_xy(goal_tf,[request['goal']])[0]
    raw=to_raw(grid,[requested])[0];low=np.array([1.,1.]);high=np.array([78.,78.])
    if inside(raw,(low,high)):return requested,dict(kind='original_goal',goal_tf=proof,requested_world=requested)
    candidates=[p for p in plans if p['stamp_ns']>=t and p['stamp_ns']-t<=1000000000]
    for plan in sorted(candidates,key=lambda p:p['stamp_ns']):
        transform,evidence=transforms.lookup(frame,plan['frame_id'],t)
        points=transform_xy(transform,plan['points'])
        if np.linalg.norm(points[-1]-requested)>.15+1e-9:continue
        if np.linalg.norm(points[0]-start_world)>.15+1e-9:continue
        local=first_exit(to_raw(grid,points),low,high)
        try:physical=first_exit(to_raw(grid,points),[0.,0.],[80.,80.])
        except MissingInput:physical=None  # A goal in the closed outer border may not leave the physical map.
        return from_raw(grid,[local])[0],dict(kind='global_path_exit',plan_stamp_ns=plan['stamp_ns'],
            requested_world=requested,effective_exit_grid=local,physical_exit_grid=physical,path_tf=evidence)
    raise MissingInput('outside_goal_without_associated_global_path')


def summarize(rows,variant,requested):
    base=[r for r in rows if r['navfn']['success']]
    success=[r for r in base if r[variant]['success']]
    ratios=[r[variant]['length']/r['navfn']['length'] for r in success if r['navfn']['length']>1e-12]
    length=distribution(ratios)
    nc=[r['navfn']['clearance'] for r in base]
    cc=[r[variant]['clearance'] for r in success]
    rate=len(success)/len(base) if base else None
    fixed=all(r[variant]['fixed_equal'] for r in rows)
    gates=dict(safety=bool(rows) and not any(r[variant]['hits'] for r in rows),
        success=rate is not None and rate>=.99,
        length=bool(ratios) and length['median']<=1.05 and length['p95']<=1.15,
        clearance=bool(nc) and bool(cc) and float(np.median(cc))>=float(np.median(nc))-1,
        fixed=bool(rows) and fixed)
    return dict(all_requests=requested,accepted_requests=len(rows),navfn_success=len(base),candidate_success=sum(r[variant]['success'] for r in rows),
        coverage=rate,length_ratio=length,clearance_candidate_median=float(np.median(cc)) if cc else None,
        clearance_navfn_median=float(np.median(nc)) if nc else None,all_outputs_fixed_equal=fixed,gates=gates)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',required=True);ap.add_argument('--lib',required=True);ap.add_argument('--navfn-lib',required=True);ap.add_argument('--out',required=True)
    a=ap.parse_args();verify_frozen();src=Path(a.input);out=Path(a.out)
    if out.exists():ap.error('output must not exist')
    out.mkdir(parents=True)
    extraction=json.loads((src/'extraction.json').read_text())
    events=[json.loads(s) for s in (src/'events.jsonl').read_text().splitlines()]
    requests=[r for r in events if r['kind']=='request']
    decode_request_errors=[e for e in extraction['decode_errors'] if e['topic']==extraction['request_topic']]
    if not requests and not extraction['counts'].get(extraction['request_topic'],0):
        # A result proxy is explicitly separate, deduplicated by full frame/path, not called a captured request.
        seen=set()
        for r in events:
            if r['kind']!='plan' or not r['points']:continue
            key=(r['epoch'],r['frame_id'],json.dumps(r['points']))
            if key in seen:continue
            seen.add(key);requests.append(dict(r,kind='request',goal=r['points'][-1],source='plan_result_proxy',request_id='proxy:%d'%len(requests)))
    contexts={}
    for epoch in sorted(set(e['epoch'] for e in events)):
        ee=[e for e in events if e['epoch']==epoch]
        try:tf=Transforms([e for e in ee if e['kind']=='tf']);error=None
        except MissingInput as exc:tf=None;error=str(exc)
        contexts[epoch]=dict(tf=tf,error=error,maps=[e for e in ee if e['kind']=='map'],plans=[e for e in ee if e['kind']=='plan'])
    planner=PlannerV3(a.lib);nav=NavFn(a.navfn_lib);rows=[];paths=[];seen=set()
    rejects=[dict(index=None,request=None,reason='request_decode_error:'+r['reason'],received_ns=r['received_ns']) for r in decode_request_errors]
    request_count=len(requests)+len(decode_request_errors)
    for i,request in enumerate(requests):
        try:
            key=(request['epoch'],request['request_id'])
            if key in seen:raise MissingInput('duplicate_request_id')
            seen.add(key);ctx=contexts[request['epoch']]
            if ctx['error']:raise MissingInput(ctx['error'])
            grid=select_map(ctx['maps'],request['stamp_ns'])
            transform,tfproof=ctx['tf'].lookup(grid['frame_id'],'base_link',request['stamp_ns'])
            actual_world=transform_xy(transform,[[0,0]])[0]
            goal,goalproof=choose_goal(grid,request,ctx['plans'],ctx['tf'],actual_world)
            meta=MapMetadata(grid['frame_id'],i,request['stamp_ns'],grid['resolution'],*grid['origin'],'navfn_round')
            pose=PoseSample(meta.frame_id,i,meta.stamp_ns,*map(float,actual_world),
                'recorded_pose' if extraction['source_kind']=='real' else 'synthetic_pose')
            m=checked_map(np.asarray(grid['costs']).reshape(grid['height'],grid['width']))
            start,actual=world_to_grid(meta,actual_world,m.shape);_,rawgoal=world_to_grid(meta,goal,m.shape)
            effective=m.copy();effective[start[1],start[0]]=0;geometry=Geometry(effective)
            navpath=append_actual_start(nav.plan(m,start,rawgoal,meta.resolution),actual)
            row=dict(index=i,request=request,source_map_stamp_ns=grid['stamp_ns'],map_age_ns=request['stamp_ns']-grid['stamp_ns'],
                start_world=actual_world,start_cell=start,original_start_cost=int(m[start[1],start[0]]),local_goal_world=goal,
                tf_provenance=tfproof,goal_provenance=goalproof,encoding=grid['encoding'],navfn=metric(navpath,geometry,meta.resolution))
            pp=dict(index=i,navfn=navpath)
            for variant in [1,2]:
                results=[]
                for mode in ['float','fixed']:
                    if variant==2:r=planner.plan(m,meta,pose,goal,meta.frame_id,mode=mode)
                    else:
                        r=planner.plan_cells(m,start,rawgoal,meta.resolution,.15,mode,variant=variant)
                        r['path']=append_actual_start(r['path'],actual)
                        if geometry.collisions(r['path']):r.update(success=False,status='safety_rejected',path=np.empty((0,2)))
                    results.append(r)
                f,r=results;name='v3_%d'%variant
                row[name]=dict(metric(r['path'],geometry,meta.resolution),status=r['status'],
                    fixed_equal=f['status']==r['status'] and np.array_equal(f['path'],r['path']),
                    original_map_hits=Geometry(m).collisions(r['path']))
                pp[name]=r['path']
            rows.append(row);paths.append(pp)
        except (MissingInput,InputContractError,ValueError,KeyError,TypeError) as exc:
            rejects.append(dict(index=i,request=request,reason=str(exc)))
    for name,data in [('pairs',rows),('paths',paths),('rejected',rejects)]:
        (out/(name+'.jsonl')).write_text(''.join(json.dumps(r,default=serial,allow_nan=False)+'\n' for r in data))
    summary=dict(code_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),frozen_v3=FROZEN,
        source_kind=extraction['source_kind'],requests=request_count,accepted=len(rows),rejected=len(rejects),
        rejection_reasons=dict(Counter(r['reason'] for r in rejects)),
        request_sources=dict(Counter(r['source'] for r in requests)),below_200=len(rows)<200,
        variants={str(v):summarize(rows,'v3_%d'%v,request_count) for v in [1,2]},
        real_pose_confirmation=extraction['source_kind']=='real' and len(rows)>=200 and all(all(summarize(rows,'v3_%d'%v,request_count)['gates'].values()) for v in [1,2]))
    (out/'summary.json').write_text(json.dumps(summary,default=serial,indent=2)+'\n')
    print(json.dumps(summary,default=serial,indent=2))


if __name__=='__main__':main()
