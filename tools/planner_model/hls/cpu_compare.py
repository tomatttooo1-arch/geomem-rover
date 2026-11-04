"""Verify same-input full fields; time only after a conservative host-idle gate."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[3]
GOLDEN='21069ecedb8865e32ca801b6fc9244e5ffbc3835'


def host_state():
    counts={n:0 for n in ['vivado','vitis_hls','xsim','xsimk','cc1plus']}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:name=(p/'comm').read_text().strip()
        except (OSError,PermissionError):continue
        if name in counts:counts[name]+=1
    load=os.getloadavg()
    return dict(time=time.time(), load=list(load), processes=counts,
                idle_gate=load[0]<=4 and load[1]<=4 and not any(counts.values()))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--vectors',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--time',action='store_true',help='requires idle gate; verify-only otherwise')
    a=ap.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    base=ROOT/'tools/planner_model'
    for name in ['model_v3.cpp','model_v3.hpp','model_v2.hpp']:
        assert (base/name).read_bytes()==subprocess.check_output(['git','show',GOLDEN+':tools/planner_model/'+name],cwd=ROOT)
    source=(base/'model_v3.cpp').read_text()
    include='#include "model_v3.hpp"';anchor='  int tx=goal_x,ty=goal_y;'
    assert source.count(include)==source.count(anchor)==1
    source=source.replace(include,include+'\n#include <type_traits>\nstatic uint64_t* field_sink=nullptr;')
    source=source.replace(anchor,'  if(field_sink && std::is_same<T,uint64_t>::value){\n    for(int i=0;i<w*h;++i)field_sink[i]=static_cast<uint64_t>(potential[i]);\n    *status=0;return 0;\n  }\n'+anchor)
    source+='''
extern "C" int v3_field_only(const uint8_t* m,int sx,int sy,int gx,int gy,uint64_t* field,uint32_t* rounds){
  double x[6402],y[6402],resolved[2];int status=6;field_sink=field;
  wave_plan_v3(m,80,80,sx,sy,gx,gy,.05,.15,1,x,y,6402,rounds,&status,resolved,1);
  field_sink=nullptr;return status;
}
'''
    observed=out/'model_v3_field_observed.cpp';observed.write_text(source)
    binary=out/'cpu_compare'
    command=['g++','-O2','-std=c++14','-Wall','-Wextra','-Wno-unused-parameter','-ffp-contract=off',
             '-I'+str(base),'-I'+str(base/'vendor'),str(Path(__file__).with_suffix('.cpp')),str(observed),str(base/'vendor/navfn.cpp'),'-o',str(binary)]
    with (out/'compile.log').open('w') as f:subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,check=True)
    args=[str(binary),str(a.vectors.resolve()/'input.bin'),str(a.vectors.resolve()/'expected.bin')]
    with (out/'verify.json').open('w') as f:subprocess.run(args+['verify'],stdout=f,check=True)
    state=host_state()
    meta=dict(golden_v3=GOLDEN,observed_source_sha256=hashlib.sha256(observed.read_bytes()).hexdigest(),
              vectors_sha256={n:hashlib.sha256((a.vectors/n).read_bytes()).hexdigest() for n in ['input.bin','expected.bin']},
              compiler=command,host_before=state,warmup=3,repeats=20,
              scope='complete field, allocation/init/input copy/output copy; no tolerance/descent/smoothing; B caller buffers preallocated, v3/NavFn allocate as original',
              NavFn='same cleared source, no stop_at_start, 6400^2 guard and all priority buffers must be empty; compare reachability only, potentials differ')
    if a.time and state['idle_gate']:
        with (out/'timings.jsonl').open('w') as f:subprocess.run(args+['time'],stdout=f,check=True)
        after=host_state();meta['host_after']=after;meta['timing_valid']=after['idle_gate']
    else:
        meta['timing_valid']=False;meta['timing_skipped']='not requested' if not a.time else 'host idle gate failed'
    (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    print((out/'verify.json').read_text().strip());print(json.dumps(dict(timing_valid=meta['timing_valid'],host=state)))


if __name__=='__main__':main()
