"""Two fixed real-source maps for hardware capacity design, CPU only, one run each."""
import argparse,hashlib,json,subprocess
from collections import deque
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from tools.b1_costmap import nav2_ref
ROOT=Path(__file__).resolve().parents[3]
SOURCES=[
 dict(name='classroom',path='${FPGA_SRC}/runs/realcar-first-hour-20261016/map_review/20261015_190638.pgm',sha256='8f7adc051fb45fd877770100b5fddde3d4227b948251d7d24ba7b7ed4a7b12ae',provenance='Team real classroom RGB-D mapping export, 2026-10-15; PROGRESS-22 and REAL_CAR_NAV_GOLIVE_20261016.'),
 dict(name='tum_fr2',path='${FPGA_SRC}/runs/real-car-nav-20261015-results/host_map7_q_graph3dof/export/20261015_130708.pgm',sha256='a4945280e888b75fb72dc0d9507440afc4429720d22de47938bc6cba2f69807a',provenance='Real TUM RGB-D fr2 recording, host RTAB-Map replay with FORCE_3DOF=1; PREREG_HANDHELD_RTABMAP_20261015; not a team robot drive.')]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,j):p.write_text(json.dumps(j,ensure_ascii=False,indent=2)+'\n')
def bfs(walk,start):
 h,w=walk.shape;d=np.full((h,w),-1,np.int32);d[start]=0;q=deque([start]);last=start
 while q:
  y,x=q.popleft();last=(y,x)
  for yy,xx in ((y-1,x),(y,x-1),(y,x+1),(y+1,x)):
   if 0<=yy<h and 0<=xx<w and walk[yy,xx] and d[yy,xx]<0:
    d[yy,xx]=d[y,x]+1;q.append((yy,xx))
 return last,d

def endpoints(cost):
 walk=cost<253;walk[0]=False;walk[-1]=False;walk[:,0]=False;walk[:,-1]=False
 unseen=walk.copy();best=None;best_size=0
 for y,x in np.argwhere(walk):
  if not unseen[y,x]:continue
  _,d=bfs(walk,(int(y),int(x)));mask=d>=0;unseen[mask]=False
  n=int(mask.sum())
  if n>best_size:best_size=n;best=(int(y),int(x))
 if best_size<2:raise ValueError('fixed source has no usable component; do not replace it')
 a,_=bfs(walk,best);b,d=bfs(walk,a)
 return [a[1],a[0]],[b[1],b[0]],best_size,int(d[b])

def coarse(source):
 p=Path(source['path']);assert sha(p)==source['sha256']
 meta=yaml.safe_load(p.with_suffix('.yaml').read_text());a=np.asarray(Image.open(p))
 assert a.ndim==2 and a.dtype==np.uint8 and abs(float(meta['resolution'])-.05)<1e-8
 assert int(meta.get('negate',0))==0 and meta['origin'][2]==0
 probability=(255-a.astype(np.float64))/255.
 occ=np.full(a.shape,-1,np.int16)
 occ[probability>float(meta['occupied_thresh'])]=100
 occ[probability<float(meta['free_thresh'])]=0
 occ=np.flipud(occ)
 h,w=occ.shape;pad=np.full(((h+1)//2*2,(w+1)//2*2),-1,np.int16);pad[:h,:w]=occ
 blocks=pad.reshape(pad.shape[0]//2,2,pad.shape[1]//2,2)
 lethal=(blocks==100).any(axis=(1,3));unknown=(blocks==-1).any(axis=(1,3))
 c=np.zeros(lethal.shape,np.int16);c[unknown]=-1;c[lethal]=100
 assert max(c.shape)<=200,'do not crop source or resize geometry'
 oy,ox=(200-c.shape[0])//2,(200-c.shape[1])//2
 canvas=np.full((200,200),-1,np.int16);canvas[oy:oy+c.shape[0],ox:ox+c.shape[1]]=c
 master=nav2_ref.master_from_grid(canvas)
 params=nav2_ref.InflationParams(resolution=.1,inflation_radius=.45,cost_scaling_factor=3.,robot_radius=.30)
 cost=nav2_ref.inflate_nav2(master,params).astype(np.uint8)
 start,goal,n,steps=endpoints(cost)
 info=dict(**source,yaml_sha256=sha(p.with_suffix('.yaml')),source_shape=list(a.shape),source_resolution_m=.05,coarse_shape=list(c.shape),grid_shape=[200,200],resolution_m=.1,known_footprint_m=[w*.05,h*.05],origin_m=[float(meta['origin'][0])-ox*.1,float(meta['origin'][1])-oy*.1,0.],unknown_padding_cells=[ox,oy],start=start,goal=goal,largest_component_cells=n,endpoint_cardinal_distance=steps,request_provenance='Deterministic farthest BFS endpoints in largest free component; no recorded robot request or pose claim.',coarsen='2x2 conservative: lethal wins, then unknown, else free; never upscale or crop',cost_histogram={str(int(v)):int(k) for v,k in zip(*np.unique(cost,return_counts=True))})
 return cost,info

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 if a.out.exists():ap.error('out must be new; no overwriting or repeated runs')
 out=a.out.resolve();out.mkdir(parents=True)
 cmd=['g++','-std=c++14','-O2','-Wall','-Wextra','-ffp-contract=off',str(Path(__file__).with_name('profile_reference.cpp')),'-o',str(out/'profile_reference')]
 with (out/'build.log').open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
 dump(out/'run-identity.json',dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),reference_sha256=sha(ROOT/'tools/planner_model/planner_hw_ref.hpp'),profile_source_sha256=sha(Path(__file__)),cpp_sha256=sha(Path(__file__).with_name('profile_reference.cpp')),compile=cmd,measurements='round counts only, no CPU latency or speedup measurement'))
 results=[]
 for source in SOURCES:
  d=out/source['name'];d.mkdir();cost,info=coarse(source);cost.tofile(d/'map.u8');info['input_sha256']=sha(d/'map.u8');dump(d/'input.json',info)
  run=[str(out/'profile_reference'),str(d/'map.u8'),*map(str,info['start']),str(d/'reference')]
  with (d/'reference.log').open('w') as f:subprocess.run(run,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=900)
  measured=json.loads((d/'reference.log').read_text());measured.update(name=source['name'],input_sha256=info['input_sha256'],field_sha256=sha(d/'reference.field.u32le'),flags_sha256=sha(d/'reference.changed.u8'),selected_start=info['start'],selected_goal=info['goal'])
  results.append(measured);print(json.dumps(measured),flush=True)
 rounds=[x['rounds'] for x in results]
 summary=dict(cases=results,sample_count=2,round_min=min(rounds),round_median=float(np.median(rounds)),round_max=max(rounds),planned_global_k_cap=256,observed_margin_to_cap=256-max(rounds),infinity_80='0x00ffffff',infinity_200='0x1fffffff',limitations='Only two fixed real-source topology samples, not a global convergence guarantee or recorded-navigation validation. Final hardware comparison uses one frozen80 case plus classroom200 only.')
 dump(out/'summary.json',summary)
 print(json.dumps({k:v for k,v in summary.items() if k!='cases'}),flush=True)
if __name__=='__main__':main()
