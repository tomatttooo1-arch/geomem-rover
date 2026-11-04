"""Run one gated HLS baseline inside ONE caller-owned slot.py lease."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


def check_synth_pragmas(report):
    try:
        upstream=report['upstream']
        pragma=upstream['synthesisPragmaReportContent']
        log=upstream['synthesisLogContent']
    except (KeyError,TypeError) as exc:
        raise RuntimeError('missing required synth report fields') from exc
    if not isinstance(pragma,str) or not isinstance(log,str) or not pragma.strip() or not log.strip():
        raise RuntimeError('empty or invalid synth report fields')
    if re.search(r'HLS 207-55',log) or 'Ignored Pragmas' in pragma:
        raise RuntimeError('pragma gate failed; stop before RTL, inspect synth.json')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--skill',type=Path,required=True)
    ap.add_argument('--project',type=Path,required=True)
    ap.add_argument('--vectors',type=Path,required=True)
    ap.add_argument('--ledger',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    skill=a.skill.resolve();project=a.project.resolve();vectors=a.vectors.resolve();ledger=a.ledger.resolve()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    solution=project/'solution'
    env=os.environ.copy();env['OMP_NUM_THREADS']='4';env['OPENBLAS_NUM_THREADS']='1'
    def run(args,name):
        print('BASELINE_STAGE '+name,flush=True)
        with (out/(name+'.log')).open('w') as f:
            subprocess.run([str(x) for x in args],stdout=f,stderr=subprocess.STDOUT,env=env,check=True)
    def flow(step,extra=None):
        args=['bash',skill/'scripts/hls_run.sh',project,'solution',step,'--timeout','900']
        if step in ('csim','cosim'):args+=['--argv',str(vectors/'input.bin')+' '+str(vectors/'expected.bin')]
        if extra:args+=extra
        run(args,step)
    def report(kind):
        path=out/(kind+'.json')
        with path.open('w') as f:
            subprocess.run(['bash',str(skill/'scripts/hls_report.sh'),kind,str(solution)],stdout=f,check=True,env=env)
        return json.loads(path.read_text())
    def bitexact(golden,candidate,label):
        run(['python3',skill/'scripts/bitexact_check.py',golden,candidate],label)
    # The short baseline C output is checked against the frozen protocol oracle
    # and the previous actual HLS C golden before any RTL or physical work.
    flow('csim')
    current=solution/'csim/build/tb_out'
    bitexact(vectors/'golden_tb_out',current,'csim-vs-protocol')
    bitexact(ledger/'golden/csim',current,'csim-vs-frozen-csim')
    report('basic-info')
    flow('csynth')
    synth=report('synth')
    check_synth_pragmas(synth)
    flow('cosim',['--cosim-opts','-rtl verilog -enable_dataflow_profiling'])
    rtl=solution/'sim/wrapc_pc/tb_out'
    bitexact(ledger/'golden/csim',rtl,'rtl-vs-frozen-csim')
    bitexact(vectors/'golden_tb_out',rtl,'rtl-vs-protocol')
    report('cosim')
    report('dataflow')
    saved=ledger/'golden/cosim'
    if saved.exists():bitexact(saved,rtl,'rtl-vs-frozen-rtl')
    else:shutil.copytree(rtl,saved)
    flow('impl')
    report('impl')
    (out/'complete.json').write_text(json.dumps(dict(project=str(project),vectors=str(vectors),
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        csim_bitexact=True,rtl_bitexact=True,post_route_report=True),indent=2)+'\n')
    print('BASELINE_COMPLETE '+str(out),flush=True)


if __name__=='__main__':main()
