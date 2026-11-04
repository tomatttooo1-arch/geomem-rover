"""Calibrate retained/free/mmap accounting before interpreting robot processes."""
import argparse
import json
import os
from pathlib import Path
import select
import subprocess
import time
from tools.rtabmap_memory_probe import allocator_request, allocator_totals, proc_sample


def wait_line(process, expected):
    end=time.monotonic()+10
    while time.monotonic()<end:
        if select.select([process.stdout],[],[],.2)[0]:
            line=process.stdout.readline()
            if line.strip()==expected.encode():return
            if not line:raise RuntimeError('calibration child exited')
    raise TimeoutError(expected)


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('executable',type=Path)
    p.add_argument('--preload',type=Path,help='test exact direct-ELF heaptrack launch with observer preload')
    a=p.parse_args();a.output.mkdir(exist_ok=False);results={}
    for mode in ('held','freed','mmap'):
        out=a.output/mode;out.mkdir();environment=os.environ.copy();environment['FPGA_ALLOCATOR_DIR']=str(out)
        if a.preload:
            environment.update(LD_PRELOAD=str(a.preload),FPGA_ALLOCATOR_EXECUTABLE=str(a.executable))
        cmd=['heaptrack','-o',str(out/'heaptrack'),str(a.executable),mode]
        proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(out/'stderr.log').open('wb'),env=environment,bufsize=0)
        rows={}
        try:
            wait_line(proc,'ready');deadline=time.monotonic()+5
            while not (out/'control.sock').exists() and time.monotonic()<deadline:time.sleep(.02)
            targets=[]
            for root in Path('/proc').glob('[0-9]*'):
                try:
                    argv=(root/'cmdline').read_bytes().split(b'\0')
                    if argv[0]==str(a.executable).encode():targets.append(int(root.name))
                except (FileNotFoundError,ProcessLookupError,PermissionError):pass
            if len(targets)!=1:raise RuntimeError('ambiguous calibration target')
            maps=Path(f'/proc/{targets[0]}/maps').read_text()
            if 'libheaptrack_preload.so' not in maps or (a.preload and str(a.preload) not in maps):
                raise RuntimeError('expected profiling libraries absent from actual calibration target')
            (out/'target-maps.txt').write_text(maps)
            def sample(name,trim=False):
                response=allocator_request(out/'control.sock','trim' if trim else 'snapshot');time.sleep(.1)
                rows[name]={'allocator':allocator_totals(response['after']),'proc':proc_sample(targets[0]),'response':response}
            sample('baseline');proc.stdin.write(b'a');proc.stdin.flush();wait_line(proc,'allocated');sample('allocated')
            if mode!='held':proc.stdin.write(b'f');proc.stdin.flush();wait_line(proc,'freed')
            sample('before_trim');sample('after_trim',True)
            proc.stdin.write(b'q');proc.stdin.flush();proc.wait(timeout=15)
            if proc.returncode:raise RuntimeError('heaptrack calibration exited nonzero')
        finally:
            if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
        (out/'measurements.json').write_text(json.dumps(rows,indent=2)+'\n');results[mode]=rows
        traces=list(out.glob('heaptrack*.gz'))+list(out.glob('heaptrack*.zst'))
        if len(traces)!=1:raise RuntimeError('missing heaptrack trace')
        with (out/'heaptrack.txt').open('w') as f:subprocess.run(['heaptrack_print',str(traces[0])],stdout=f,check=True)
    def proxy(mode,phase):return results[mode][phase]['allocator']['in_use_proxy_bytes']
    def rss(mode,phase):return results[mode][phase]['proc']['Rss_kib']*1024
    checks={
      'malloc_allocation_visible':proxy('held','allocated')-proxy('held','baseline')>60*2**20,
      'held_survives_trim':proxy('held','after_trim')-proxy('held','baseline')>60*2**20,
      'freed_proxy_drops':proxy('freed','allocated')-proxy('freed','before_trim')>60*2**20,
      'freed_pages_retained_before_trim':rss('freed','before_trim')-rss('freed','baseline')>20*2**20,
      'trim_releases_retained_pages':rss('freed','before_trim')-rss('freed','after_trim')>8*2**20,
      'mmap_rss_visible':rss('mmap','allocated')-rss('mmap','baseline')>60*2**20,
      'mmap_not_misclassified_as_malloc':abs(proxy('mmap','allocated')-proxy('mmap','baseline'))<8*2**20,
      'munmap_releases_pages':rss('mmap','allocated')-rss('mmap','before_trim')>60*2**20,
      'held_stack_resolved':'calibration_allocate' in (a.output/'held/heaptrack.txt').read_text(),
      'freed_stack_resolved':'calibration_allocate' in (a.output/'freed/heaptrack.txt').read_text(),
    }
    (a.output/'checks.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2))
    if not all(checks.values()):raise SystemExit(2)


if __name__=='__main__':main()
