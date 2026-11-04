"""Deterministic host source-operation counts for frozen v3 smoothing, not timing."""
import argparse
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from tools.planner_model.aligned_v3 import PlannerV3
from tools.planner_model.hwref.verify import ROOT, GOLDEN, dataset, dump


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    out=args.out.resolve()
    s=subprocess.check_output(['git','show',GOLDEN+':tools/planner_model/model_v3.cpp'],cwd=ROOT).decode()
    s=s.replace('#include "model_v3.hpp"', '#include "model_v3.hpp"\nstatic bool sm_active=false;\nstatic uint64_t sm_counts[5]={};\nextern "C" void smoothing_reset(){sm_active=false;for(int i=0;i<5;++i)sm_counts[i]=0;}\nextern "C" uint64_t smoothing_count(int i){return sm_counts[i];}')
    def replace_once(old,new):
        nonlocal s
        assert s.count(old)==1,old
        s=s.replace(old,new)
    replace_once('  const double eps=1e-10;', '  if(sm_active)++sm_counts[1];\n  const double eps=1e-10;')
    replace_once('for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x){', 'for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x){\n    if(sm_active)++sm_counts[2];')
    replace_once('  double dx=bx-ax,dy=by-ay,den=dx*dx+dy*dy;', '  if(sm_active)++sm_counts[4];\n  double dx=bx-ax,dy=by-ay,den=dx*dx+dy*dy;')
    replace_once('  double radius=std::sqrt(best)+1e-9;', '  if(sm_active)++sm_counts[3];\n  double radius=std::sqrt(best)+1e-9;')
    replace_once('  if(n<3)return n;', '  if(n<3)return n;\n  sm_active=true;')
    replace_once('    for(int j=n-1;j>i+1;--j){', '    for(int j=n-1;j>i+1;--j){\n      ++sm_counts[0];')
    replace_once('  return int(outx.size());', '  sm_active=false;\n  return int(outx.size());')
    source=out/'model_v3_smoothing_counted.cpp';source.write_text(s)
    lib=out/'libsmoothing_counted.so'
    subprocess.run(['g++','-O2','-shared','-fPIC','-std=c++14','-ffp-contract=off','-I'+str(ROOT/'tools/planner_model'),str(source),'-o',str(lib)],check=True)
    count=PlannerV3(lib);plain=PlannerV3(out/'build/libplanner_model.so')
    count.lib.smoothing_reset.argtypes=[];count.lib.smoothing_reset.restype=None
    count.lib.smoothing_count.argtypes=[C.c_int];count.lib.smoothing_count.restype=C.c_uint64
    rows=[]
    for i,row,m,path in dataset():
        if m.shape!=(80,80) or not len(path):continue
        count.lib.smoothing_reset()
        c=count.plan_cells(m,row['start'],row['goal'],variant=2)
        p=plain.plan_cells(m,row['start'],row['goal'],variant=2)
        assert c['status']==p['status'] and c['path'].tobytes()==p['path'].tobytes()
        rows.append(dict(index=i,raw_points=len(path),smooth_points=len(c['path']),chord_candidates=count.lib.smoothing_count(0),safety_calls=count.lib.smoothing_count(1),safety_cells=count.lib.smoothing_count(2),clearance_calls=count.lib.smoothing_count(3),obstacle_distance_evaluations=count.lib.smoothing_count(4)))
    keys=['raw_points','smooth_points','chord_candidates','safety_calls','safety_cells','clearance_calls','obstacle_distance_evaluations']
    summary={key:dict(median=float(np.median([r[key] for r in rows])),p95=float(np.percentile([r[key] for r in rows],95)),max=max(r[key] for r in rows)) for key in keys}
    dump(out/'smoothing_counts.json',dict(scope='80x80 successful variant1 requests; variant2 outputs verified against uninstrumented frozen implementation',requests=len(rows),golden_commit=GOLDEN,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),summary=summary,rows=rows))
    print(json.dumps(dict(requests=len(rows),summary=summary)))


if __name__=='__main__':main()
