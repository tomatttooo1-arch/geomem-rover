"""Preregistered host-only B evaluation; no ROS, HLS, or device access."""
import argparse
import ctypes as C
import hashlib
import heapq
import json
import math
from collections import Counter
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
GOLDEN = '21069ecedb8865e32ca801b6fc9244e5ffbc3835'
INF = 0xffffff
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS')
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags='C_CONTIGUOUS')
U64 = np.ctypeslib.ndpointer(dtype=np.uint64, flags='C_CONTIGUOUS')
F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
PI, PU = C.POINTER(C.c_int), C.POINTER(C.c_uint)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def digest(a):
    return hashlib.sha256(a.tobytes()).hexdigest()


def observed_source(out):
    source = ROOT/'tools/planner_model/model_v3.cpp'
    for name in ['model_v3.cpp', 'model_v3.hpp', 'model_v2.hpp']:
        rel = 'tools/planner_model/'+name
        assert (ROOT/rel).read_bytes() == subprocess.check_output(['git', 'show', GOLDEN+':'+rel], cwd=ROOT)
    s = source.read_text()
    anchor = '  int tx=goal_x,ty=goal_y;'
    assert s.count(anchor) == 1
    s = s.replace('#include "model_v3.hpp"', '#include "model_v3.hpp"\n#include <type_traits>\nstatic uint64_t* observed=nullptr;\nextern "C" void golden_observe(uint64_t* p){observed=p;}\n#define wave_plan_v3 golden_plan_v3')
    s = s.replace(anchor, '  if(observed && std::is_same<T,uint64_t>::value)\n    for(int i=0;i<w*h;++i)observed[i]=static_cast<uint64_t>(potential[i]);\n'+anchor)
    path = out/'model_v3_observed.cpp'
    path.write_text(s)
    subprocess.run(['g++', '-shared', '-fPIC', '-O2', '-std=c++14', '-Wall', '-Wextra', '-ffp-contract=off', '-I'+str(source.parent), str(path), '-o', str(out/'libgolden_observed.so')], check=True)
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Golden:
    def __init__(self, library, name, observer=False):
        self.lib = C.CDLL(str(library.resolve()))
        self.call = getattr(self.lib, name)
        self.call.argtypes = [U8]+[C.c_int]*4+[C.c_double]*4+[C.c_int]+[F64,F64,C.c_int,PU,PI,F64,C.c_int]
        self.call.restype = C.c_int
        self.observer = observer
        if observer:
            self.lib.golden_observe.argtypes = [U64]
            self.lib.golden_observe.restype = None

    def plan(self, m, start, goal):
        h,w = m.shape
        x,y = np.zeros(m.size+2),np.zeros(m.size+2)
        field = np.full(m.shape, (2**64-1)//4, np.uint64)
        status,rounds,resolved = C.c_int(),C.c_uint(),np.full(2,np.nan)
        if self.observer:
            self.lib.golden_observe(field)
        n = self.call(m,w,h,*start,*goal,.05,.15,1,x,y,m.size+2,C.byref(rounds),C.byref(status),resolved,1)
        assert n >= 0 or status.value == 6
        inf = INF if w==80 else 0x1fffffff
        field = np.where(field == (2**64-1)//4, inf, field)
        assert np.all(field <= inf)
        return dict(status=status.value, rounds=rounds.value, path=np.column_stack((x[:max(0,n)],y[:max(0,n)])), field=field.astype(np.uint32), field_valid=rounds.value>0 and status.value!=7, resolved=resolved)


class Candidate:
    def __init__(self, library):
        self.lib = C.CDLL(str(library.resolve()))
        self.lib.hwref_plan.argtypes = [U8]+[C.c_int]*4+[C.c_double]*2+[C.c_uint,U32,F64,F64,U8,PI,PU,PI,F64,C.c_int]
        self.lib.hwref_plan.restype = C.c_int
        self.lib.hwref_k.restype = C.c_uint
        self.k = self.lib.hwref_k()
        self.lib.hwref_entry.argtypes = [C.c_uint]
        self.lib.hwref_entry.restype = C.c_uint
        self.lib.hwref_add.argtypes = [C.c_uint,C.c_uint]
        self.lib.hwref_add.restype = C.c_uint

    def plan(self,m,start,goal,limit=0,diagnostic=True):
        h,w = m.shape
        field = np.full(m.shape, INF if w==80 else 0x1fffffff, np.uint32)
        x,y,flags = np.zeros(m.size+2),np.zeros(m.size+2),np.zeros(max(m.size,limit),np.uint8)
        state,rounds,valid,resolved = C.c_int(),C.c_uint(),C.c_int(),np.full(2,np.nan)
        n = self.lib.hwref_plan(m,w,h,*start,*goal,limit,field,x,y,flags,C.byref(state),C.byref(rounds),C.byref(valid),resolved,int(diagnostic))
        return dict(status=state.value,rounds=rounds.value,field_valid=bool(valid.value),field=field,path=np.column_stack((x[:n],y[:n])),flags=flags[:rounds.value],resolved=resolved)


def fingerprint(r):
    return dict(status=r['status'], field_valid=r['field_valid'], field_sha256=digest(r['field']) if r['field_valid'] else None,
                path_sha256=digest(r['path']), points=len(r['path']), resolved_bits=r['resolved'].tobytes().hex())


def equivalent(a,b):
    return (fingerprint(a)==fingerprint(b) and a['path'].tobytes()==b['path'].tobytes()
            and a['resolved'].tobytes()==b['resolved'].tobytes()
            and (not a['field_valid'] or a['field'].tobytes()==b['field'].tobytes()))


def dataset():
    rows = [json.loads(s) for s in (ROOT/'runs/planner-v3-final-20261102/pairs.jsonl').read_text().splitlines()]
    paths = [json.loads(s) for s in (ROOT/'runs/planner-v3-final-20261102/paths.jsonl').read_text().splitlines()]
    legacy = np.load(ROOT/'runs/planner-model-final-20261102/input/maps.npz')['maps']
    large = np.load(ROOT/'runs/planner-model-final-20261102/results/large_maps.npz')['maps']
    synthetic = np.load(ROOT/'runs/planner-v2-delivery-20261102/new_maps.npz')['maps']
    fresh = np.load(ROOT/'runs/planner-v3-final-20261102/fresh_maps.npz')['maps']
    assert len(rows)==len(paths)==2024
    for i,(r,p) in enumerate(zip(rows,paths)):
        key = (r['dataset'],r['frame'],r['pair'])
        assert key == (p['dataset'],p['frame'],p['pair'])
        f=r['frame']
        if r['dataset']=='legacy':
            m=legacy[f-1] if f<=376 else large[f-1001]
        elif r['dataset']=='synthetic':m=synthetic[f-1]
        else:
            assert r['dataset']=='fresh'
            m=fresh[f-1]
        yield i,r,np.ascontiguousarray(m),np.array(p['variants']['1'],np.float64).reshape(-1,2)


def independent_field(m,start):
    m=m.copy();h,w=m.shape
    sx,sy=start;m[sy,sx]=0;m[0,:]=m[-1,:]=m[:,0]=m[:,-1]=254
    d=np.full(m.shape,INF,np.uint32);d[sy,sx]=0;queue=[(0,sx,sy)]
    while queue:
        c,x,y=heapq.heappop(queue)
        if c!=d[y,x]:continue
        for b in range(max(0,y-1),min(h,y+2)):
            for a in range(max(0,x-1),min(w,x+2)):
                if (a==x and b==y) or m[b,a]>=253:continue
                diagonal=a!=x and b!=y
                if diagonal and (m[y,a]>=253 or m[b,x]>=253):continue
                v=c+int(50+.8*int(m[b,a]))*(14 if diagonal else 10)
                if v<int(d[b,a]):d[b,a]=v;heapq.heappush(queue,(v,a,b))
    return d


def controls():
    blank=np.zeros((80,80),np.uint8)
    def q(name,m=blank,start=(10,10),goal=(65.,65.),limit=None):
        return dict(name=name,map=m.copy(),start=start,goal=goal,limit=limit)
    a=[]
    a.append(q('all_blocked',np.full_like(blank,254)))
    m=blank.copy();m[9:12,9:12]=254;a.append(q('enclosed_start',m))
    a.append(q('left_boundary_start',start=(0,40)))
    a.append(q('bottom_boundary_start',start=(40,79)))
    a.append(q('edge_goal',goal=(0.,40.)))
    m=blank.copy();m[65,65]=253;a.append(q('blocked_goal_253',m))
    m[65,65]=255;a.append(q('unknown_goal_255',m))
    m=blank.copy();m[10,10]=253;a.append(q('clear_start_253',m))
    m[10,10]=255;a.append(q('clear_start_255',m))
    a.append(q('same_cell',goal=(10.,10.)))
    a.append(q('same_blocked_cell',m,goal=(10.,10.)))
    m=np.full_like(blank,254)
    for t in range(10,21):m[t,t]=0
    a.append(q('no_corner_cut',m,goal=(20.,20.)))
    a.append(q('max_entry_cost',np.full_like(blank,252),start=(1,1),goal=(78.,78.)))
    a.append(q('fractional_goal',goal=(45.2,35.1)))
    a.append(q('nonfinite_goal',goal=(math.nan,35.)))
    a.append(q('outside_goal',goal=(80.,40.)))
    a.append(q('K1_trigger',goal=(78.,78.),limit=1))
    m=np.full_like(blank,254)
    for j,y in enumerate(range(2,77,2)):
        m[y,2:78]=0
        if y<76:m[y+1,77 if j%2==0 else 2]=0
    a.append(q('snake_production_K',m,start=(2,2),goal=(2.,76.)))
    m=blank.copy();m[:,40]=254;a.append(q('disconnected_goal',m,goal=(65.,10.)))
    m=blank.copy();m[20:25,20:25]=254;m[22,22]=0
    a.append(q('tolerance_fallback',m,goal=(22.,22.)))
    assert len(a)==20
    return a


def write_vector(folder,key,m,start,goal,g,c,limit):
    folder.mkdir()
    m.tofile(folder/'map.u8')
    if g['field_valid']:
        v=g['field'].ravel()
        packed=np.column_stack((v&255,(v>>8)&255,(v>>16)&255)).astype(np.uint8)
        packed.tofile(folder/'field.u24le')
    else:(folder/'field.u24le').write_bytes(b'')
    g['path'].astype('<f8').tofile(folder/'path.f64le')
    flags=c['flags']
    flags.tofile(folder/'changed.u8')
    # Line protocol is deliberately readable by a standalone C++ stdlib runner.
    req='%d %d %.17g %.17g %d\n'%(*start,*goal,limit)
    (folder/'request.txt').write_text(req)
    expected='%d %d %d %d\n'%(g['status'],len(g['path']),int(g['field_valid']),c['rounds'])
    (folder/'expected.txt').write_text(expected)
    return dict(key=key,folder=folder.name,status=g['status'],field_valid=g['field_valid'],points=len(g['path']),rounds=c['rounds'],limit=limit,files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(folder.iterdir())})


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--phase',choices=['profile','verify'],required=True)
    args=ap.parse_args();out=args.out.resolve();out.mkdir(exist_ok=True,parents=True)
    candidate=Candidate(out/'build/libplanner_hwref.so')
    all_data=list(dataset())
    if args.phase=='profile':
        observed_sha=observed_source(out)
        golden=Golden(out/'libgolden_observed.so','golden_plan_v3',True)
        original=Golden(out/'build/libplanner_model.so','wave_plan_v3')
        records=[];differences=[];rounds={80:Counter(),400:Counter()}
        status_names=['exact','within_tolerance','invalid_start','invalid_goal','no_path','safety_rejected','invalid_arguments','not_converged']
        for i,row,m,old_path in all_data:
            start,goal=row['start'],row['goal']
            g=golden.plan(m,start,goal);o=original.plan(m,start,goal);c=candidate.plan(m,start,goal)
            observer_ok=(g['status']==o['status'] and g['path'].tobytes()==o['path'].tobytes() and g['resolved'].tobytes()==o['resolved'].tobytes() and g['rounds']==o['rounds'])
            archived_ok=(old_path.tobytes()==o['path'].tobytes() and status_names[o['status']]==row['variants']['1']['status'])
            match=equivalent(g,c)
            if not(observer_ok and archived_ok and match):differences.append(dict(index=i,observer_ok=observer_ok,archived_ok=archived_ok,match=match,golden=fingerprint(g),candidate=fingerprint(c)))
            rounds[m.shape[0]][c['rounds']]+=1
            records.append(dict(index=i,dataset=row['dataset'],frame=row['frame'],pair=row['pair'],shape=list(m.shape),golden_rounds=g['rounds'],candidate_rounds=c['rounds'],expected=fingerprint(g),observer_ok=observer_ok,archived_ok=archived_ok,match=match))
            if (i+1)%200==0:print('profile',i+1,'/',len(all_data),flush=True)
        (out/'profile.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
        k=max(max(r) for r in rounds.values())+1
        report=dict(phase='profile',golden_commit=GOLDEN,observed_source_sha256=observed_sha,counts={str(w):sum(r.values()) for w,r in rounds.items()},rounds={str(w):dict(sorted(r.items())) for w,r in rounds.items()},recommended_K=k,differences=differences)
        dump(out/'profile_summary.json',report);print(json.dumps(report),flush=True)
        assert not differences
        return
    profile=[json.loads(s) for s in (out/'profile.jsonl').read_text().splitlines()]
    k=json.loads((out/'profile_summary.json').read_text())['recommended_K']
    assert candidate.k==k,(candidate.k,k)
    golden=Golden(out/'libgolden_observed.so','golden_plan_v3',True)
    differences=[];rows=[]
    for (i,row,m,_),p in zip(all_data,profile):
        c=candidate.plan(m,row['start'],row['goal'],k)
        if fingerprint(c)!=p['expected']:differences.append(dict(index=i,expected=p['expected'],actual=fingerprint(c)))
        assert c['rounds']==p['candidate_rounds']
        assert c['flags'].tolist()==([1]*(c['rounds']-1)+[0] if c['rounds'] else [])
        rows.append(dict(index=i,match=fingerprint(c)==p['expected'],rounds=c['rounds']))
    # Exhaustive conversion and explicit saturation/size-rejection checks.
    for c in range(253):assert candidate.lib.hwref_entry(c)==int(50+.8*c),(c,candidate.lib.hwref_entry(c),int(50+.8*c))
    for a,b in [(0,0),(0,3514),(INF,0),(INF,1),(INF-1,1),(INF-3514,3514),(INF-3515,3514),(0,2**32-1)]:assert candidate.lib.hwref_add(a,b)==min(INF,a+b)
    outside=candidate.plan(np.zeros((400,400),np.uint8),(10,10),(20.,20.),k,False)
    assert outside['status']==8 and outside['rounds']==0 and not outside['field_valid']
    rng=np.random.RandomState(20261103);oracle_checks=[]
    for i in range(12):
        m=rng.randint(0,253,(80,80)).astype(np.uint8);m[rng.random_sample((80,80))<.18]=254
        c=candidate.plan(m,(10,10),(60.,60.),0)
        # Goal may be blocked; choose a legal fixed goal to obtain the field.
        m[60,60]=0;c=candidate.plan(m,(10,10),(60.,60.),0)
        d=independent_field(m,(10,10));assert c['field_valid'] and np.array_equal(d,c['field'])
        oracle_checks.append(dict(case=i,all_cells=6400,match=True,rounds=c['rounds']))
    vector_dir=out/'vectors';vector_dir.mkdir(exist_ok=True)
    assert not list(vector_dir.iterdir()),'Use a new output directory, do not overwrite vectors'
    selected=[]
    for name in ['legacy','synthetic','fresh']:
        pool=[item for item in all_data if item[1]['dataset']==name and item[2].shape==(80,80)]
        selected.extend(pool[int(j)] for j in np.linspace(0,len(pool)-1,60,dtype=int))
    manifest=[]
    for i,row,m,_ in selected:
        c=candidate.plan(m,row['start'],row['goal'],k);g=golden.plan(m,row['start'],row['goal']);assert equivalent(g,c)
        manifest.append(write_vector(vector_dir/('%03d'%len(manifest)),dict(index=i,dataset=row['dataset'],frame=row['frame'],pair=row['pair']),m,row['start'],row['goal'],g,c,k))
    control_results=[]
    for q in controls():
        m,start,goal=q['map'],q['start'],q['goal'];limit=k if q['limit'] is None else q['limit']
        c=candidate.plan(m,start,goal,limit);g=golden.plan(m,start,goal)
        if c['status']==7:
            assert not c['field_valid'] and len(c['path'])==0 and c['flags'].tolist()==[1]*limit
            expected=dict(c);expected['field_valid']=False
        else:
            assert equivalent(c,g),(q['name'],fingerprint(c),fingerprint(g))
            expected=g
        if q['name'] in ['K1_trigger','snake_production_K']:assert c['status']==7,q['name']
        control_results.append(dict(name=q['name'],status=c['status'],rounds=c['rounds'],unbounded_rounds=g['rounds'],match_or_expected_K_rejection=True))
        manifest.append(write_vector(vector_dir/('%03d'%len(manifest)),q['name'],m,start,goal,expected,c,limit))
    dump(out/'vector_manifest.json',manifest)
    dump(out/'bounded.json',rows)
    report=dict(phase='verify',K=k,total=2024,hardware_80=1964,host_only_400=60,differences=differences,entry_values_checked=253,saturation_checks=8,size_rejection=True,dijkstra=oracle_checks,controls=control_results,vectors=len(manifest),vector_manifest_sha256=hashlib.sha256((out/'vector_manifest.json').read_bytes()).hexdigest())
    dump(out/'verification.json',report);print(json.dumps(report),flush=True)
    assert not differences


if __name__=='__main__':main()
