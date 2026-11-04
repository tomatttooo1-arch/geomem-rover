"""Run the frozen 33 RTL cases in budgeted batches with all adjacent transitions."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess

INPUT_SIZE=20+6400
OUTPUT_SIZE=12+6400*3+64


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def records(path,magic,size):
    raw=Path(path).read_bytes()
    if raw[:8]!=magic or len(raw)<12:raise ValueError('invalid header')
    count=struct.unpack_from('<I',raw,8)[0]
    if count!=33 or len(raw)!=12+count*size:raise ValueError('invalid count or truncated records')
    return [raw[12+i*size:12+(i+1)*size] for i in range(count)]


def prepare(vectors,out):
    inputs=records(vectors/'input.bin',b'FPGAFIN01',INPUT_SIZE)
    outputs=records(vectors/'expected.bin',b'FPGAFGO01',OUTPUT_SIZE)
    original=(vectors/'golden_tb_out/outputs.bin').read_bytes()
    if b''.join(outputs)!=original:raise ValueError('protocol and frozen TB golden differ')
    groups=[list(range(20))]
    for start in range(20,33,3):groups.append(list(range(start-1,min(start+3,33))))
    assert sum(map(len,groups))==38
    assert set(i for g in groups for i in g)==set(range(33))
    assert set((a,b) for g in groups for a,b in zip(g,g[1:]))==set((i,i+1) for i in range(32))
    out.mkdir(parents=True,exist_ok=False)
    manifest=dict(source={n:digest(vectors/n) for n in ['input.bin','expected.bin','golden_tb_out/outputs.bin']},
                  unique_cases=33,total_calls=38,adjacent_pairs=32,batches=[])
    for number,indices in enumerate(groups):
        folder=out/('batch-%02d'%number);folder.mkdir();golden=folder/'golden_tb_out';golden.mkdir()
        (folder/'input.bin').write_bytes(b'FPGAFIN01'+struct.pack('<I',len(indices))+b''.join(inputs[i] for i in indices))
        raw=b''.join(outputs[i] for i in indices)
        (folder/'expected.bin').write_bytes(b'FPGAFGO01'+struct.pack('<I',len(indices))+raw)
        (golden/'outputs.bin').write_bytes(raw)
        manifest['batches'].append(dict(folder=folder.name,indices=indices,files={n:digest(folder/n) for n in ['input.bin','expected.bin','golden_tb_out/outputs.bin']}))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('prepared fixed33: 6 batches, 38 calls incl 5 overlaps, all32 adjacent transitions',flush=True)


def run(skill,project,out,source):
    manifest=json.loads((out/'manifest.json').read_text());source_sha=digest(source)
    env=dict(os.environ,OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1')
    reconstructed={};summaries=[]
    for batch in manifest['batches']:
        folder=out/batch['folder']
        for name,sha in batch['files'].items():
            if digest(folder/name)!=sha:raise ValueError('frozen batch changed: '+name)
        complete=folder/'complete.json'
        if complete.exists():
            saved=json.loads(complete.read_text())
            if saved['source_sha256']!=source_sha:raise ValueError('source changed since completed batch')
            attempt=Path(saved['attempt'])
            print('RECHECK '+batch['folder'],flush=True)
        else:
            attempt=folder/('attempt-'+datetime.now().strftime('%Y%m%d_%H%M%S'));attempt.mkdir()
            print('RUN '+batch['folder']+' indices='+str(batch['indices']),flush=True)
            with (attempt/'cosim.log').open('w') as log:
                subprocess.run(['bash',str(skill/'scripts/hls_run.sh'),str(project),'solution','cosim','--timeout','900',
                    '--argv',str(folder/'input.bin')+' '+str(folder/'expected.bin'),'--cosim-opts','-rtl verilog -enable_dataflow_profiling'],
                    stdout=log,stderr=subprocess.STDOUT,env=env,check=True)
            shutil.copytree(project/'solution/sim/wrapc_pc/tb_out',attempt/'rtl')
            for kind in ['cosim','dataflow']:
                with (attempt/(kind+'.json')).open('w') as f:
                    subprocess.run(['bash',str(skill/'scripts/hls_report.sh'),kind,str(project/'solution')],stdout=f,check=True,env=env)
        with (attempt/'bitexact.log').open('w') as f:
            subprocess.run(['python3',str(skill/'scripts/bitexact_check.py'),str(folder/'golden_tb_out'),str(attempt/'rtl')],stdout=f,stderr=subprocess.STDOUT,check=True,env=env)
        raw=(attempt/'rtl/outputs.bin').read_bytes()
        if len(raw)!=len(batch['indices'])*OUTPUT_SIZE:raise ValueError('RTL output size mismatch')
        for n,index in enumerate(batch['indices']):
            record=raw[n*OUTPUT_SIZE:(n+1)*OUTPUT_SIZE]
            if index in reconstructed and reconstructed[index]!=record:raise ValueError('overlap changed output')
            reconstructed[index]=record
        report=json.loads((attempt/'cosim.json').read_text())
        if report['summary']['status']!='Pass':raise ValueError('RTL report did not pass')
        summaries.append(dict(folder=batch['folder'],indices=batch['indices'],summary=report['summary']))
        if digest(source)!=source_sha:raise ValueError('source changed during RTL')
        complete.write_text(json.dumps(dict(source_sha256=source_sha,attempt=str(attempt),bitexact=True),indent=2)+'\n')
        print('PASS '+batch['folder'],flush=True)
    if set(reconstructed)!=set(range(33)):raise ValueError('missing unique RTL cases')
    raw=b''.join(reconstructed[i] for i in range(33));expected=b''.join((out/b['folder']/'golden_tb_out/outputs.bin').read_bytes()[(j*OUTPUT_SIZE):((j+1)*OUTPUT_SIZE)] for b in manifest['batches'] for j,i in enumerate(b['indices']) if b is manifest['batches'][0] or j>0)
    if raw!=expected:raise ValueError('ordered reconstruction mismatch')
    (out/'reconstructed').mkdir(exist_ok=True);(out/'reconstructed/outputs.bin').write_bytes(raw)
    if hashlib.sha256(raw).hexdigest()!=manifest['source']['golden_tb_out/outputs.bin']:raise ValueError('original fixed33 SHA mismatch')
    (out/'complete.json').write_text(json.dumps(dict(unique_cases=33,total_calls=38,adjacent_pairs=32,source_sha256=source_sha,
        reconstructed_bytes=len(raw),reconstructed_sha256=hashlib.sha256(raw).hexdigest(),batches=summaries),indent=2)+'\n')
    print('FINAL33 PASS reconstructed bytes='+str(len(raw)),flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('mode',choices=['prepare','run'])
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--vectors',type=Path)
    ap.add_argument('--skill',type=Path);ap.add_argument('--project',type=Path);ap.add_argument('--source',type=Path)
    a=ap.parse_args()
    if a.mode=='prepare':
        if not a.vectors:ap.error('--vectors required')
        prepare(a.vectors.resolve(),a.out.resolve())
    else:
        if not all([a.skill,a.project,a.source]):ap.error('--skill --project --source required')
        run(a.skill.resolve(),a.project.resolve(),a.out.resolve(),a.source.resolve())


if __name__=='__main__':main()
