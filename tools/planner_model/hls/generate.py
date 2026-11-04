"""Freeze all 200 old B inputs plus 12 fixed pressure fields and K64 snake; no resampling."""
import argparse,hashlib,json,math,struct,subprocess
from pathlib import Path
import numpy as np
from tools.planner_model.hwref.verify import independent_field, controls

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pack(path,cases):
 with path.open('wb') as f:
  f.write(b'FPGAFIN01'+struct.pack('<I',len(cases)))
  for c in cases:f.write(struct.pack('<5i',*c['request'])+c['map'].tobytes())
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
 manifest=json.loads((a.source.parent/'vector_manifest.json').read_text());assert len(manifest)==200
 cases=[]
 for i,item in enumerate(manifest):
  folder=a.source/item['folder']
  for n,s in item['files'].items():assert sha(folder/n)==s
  sx,sy,gx,gy,k=(folder/'request.txt').read_text().split();oldgoal=(float(gx),float(gy))
  goal=[int(math.floor(x+.5)) if math.isfinite(x) and -.5<=x<80 else -1 for x in oldgoal]
  limit=int(k) if item['key'] in ['K1_trigger','snake_production_K'] else 64
  cases.append(dict(name='B_%03d'%i,map=np.fromfile(folder/'map.u8',np.uint8).reshape(80,80),request=[int(sx),int(sy),*goal,limit],source=item,
                    conversion='PS rejected nonfinite/outside goal -> -1' if -1 in goal else 'PS rounded representative goal'))
 rng=np.random.RandomState(20261103)
 for i in range(12):
  m=rng.randint(0,253,(80,80)).astype(np.uint8);m[rng.random_sample((80,80))<.18]=254;m[60,60]=0
  cases.append(dict(name='pressure_%02d'%i,map=m,request=[10,10,60,60,64],source=None))
 snake=next(q for q in controls() if q['name']=='snake_production_K')
 cases.append(dict(name='snake_K64',map=snake['map'],request=[2,2,2,76,64],source=None))
 compiler=['g++','-std=c++14','-O2','-Wall','-Wextra','-ffp-contract=off','tools/planner_model/hls/golden.cpp','-o',str(out/'golden')]
 subprocess.run(compiler,check=True)
 proof=[]
 for name,subset in [('all',cases),('cosim',cases[180:])]:
  root=out/name;root.mkdir();pack(root/'input.bin',subset)
  subprocess.run([str(out/'golden'),str(root/'input.bin'),str(root/'expected.bin')],check=True)
  raw=(root/'expected.bin').read_bytes();assert raw[:8]==b'FPGAFGO01' and struct.unpack_from('<I',raw,8)[0]==len(subset)
  size=12+6400*3+64;assert len(raw)==12+len(subset)*size
  golden=root/'golden_tb_out';golden.mkdir();(golden/'outputs.bin').write_bytes(raw[12:])
  for i,c in enumerate(subset):
   off=12+i*size;status,rounds,valid=struct.unpack_from('<3I',raw,off)
   f=np.frombuffer(raw,np.uint8,6400*3,off+12).reshape(6400,3).astype(np.uint32);field=(f[:,0]|f[:,1]<<8|f[:,2]<<16).reshape(80,80)
   flags=list(raw[off+12+19200:off+size]);assert flags[rounds:]==[255]*(64-rounds)
   if valid:assert flags[:rounds]==[1]*(rounds-1)+[0]
   if c['source'] and c['source']['field_valid']:
    previous=np.fromfile(a.source/c['source']['folder']/'field.u24le',np.uint8).reshape(6400,3).astype(np.uint32)
    previous=(previous[:,0]|previous[:,1]<<8|previous[:,2]<<16).reshape(80,80)
    assert np.array_equal(previous,field),(c['name'],'legacy_field_changed')
   if c['source'] is None:
    assert valid and status==0
    assert np.array_equal(field,independent_field(c['map'],c['request'][:2])),c['name']
   if name=='all':proof.append(dict(name=c['name'],request=c['request'],status=status,rounds=rounds,field_valid=bool(valid),
      map_sha256=hashlib.sha256(c['map'].tobytes()).hexdigest(),source=c['source'],conversion=c.get('conversion')))
  (root/'sha256.json').write_text(json.dumps({p.name:sha(p) for p in root.iterdir() if p.is_file()},indent=2)+'\n')
 (out/'vectors.json').write_text(json.dumps(proof,indent=2)+'\n')
 print('vectors',len(cases),'cosim subset',len(cases[180:]),'pressure Dijkstra exact',13)
if __name__=='__main__':main()
