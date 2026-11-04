"""Lease one nationals small build slot, retain the token, release only our own lease."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--purpose',required=True);ap.add_argument('--log',type=Path,required=True)
    ap.add_argument('--wait-limit',type=int,default=7200)
    ap.add_argument('--single-vivado',action='store_true')
    ap.add_argument('command',nargs=argparse.REMAINDER)
    a=ap.parse_args();cmd=a.command
    if cmd and cmd[0]=='--':cmd=cmd[1:]
    if not cmd:ap.error('command required')
    a.log.parent.mkdir(parents=True,exist_ok=True)
    lock_root=Path('${FPGA_LOCKS}')
    token=uuid.uuid4().hex;slot=None;child=None
    def record(event,**kw):
        row=dict(time=datetime.now(timezone.utc).isoformat(),event=event,who='dev2',token=token,**kw)
        with a.log.open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        print(json.dumps(row,ensure_ascii=False),flush=True)
    def stop(sig,frame):raise KeyboardInterrupt('signal %d'%sig)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    start=time.monotonic()
    try:
        while slot is None:
            mem=int(next(s.split()[1] for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:')))
            active_vivado=bool(subprocess.run(['pgrep','-x','vivado'],stdout=subprocess.DEVNULL).returncode==0) if a.single_vivado else False
            if mem>=16777216 and not active_vivado:
                for n in [4]:
                    p=lock_root/('nat_build_slot_%d'%n)
                    try:p.mkdir(mode=0o700)
                    except FileExistsError:continue
                    slot=p
                    lease=dict(who='dev2',token=token,pid=os.getpid(),purpose=a.purpose)
                    (p/'lease.json').write_text(json.dumps(lease)+'\n')
                    (p/'owner').write_text('dev2 %s pid %d token %s %s\n'%(a.purpose,os.getpid(),token,datetime.now().astimezone().isoformat()))
                    record('acquired',slot=str(p),purpose=a.purpose,mem_available_kib=mem)
                    break
            if slot is None:
                if time.monotonic()-start>=a.wait_limit:raise TimeoutError('no free slot within wait limit')
                record('waiting',mem_available_kib=mem,active_vivado=active_vivado)
                time.sleep(5)
        child=subprocess.Popen(cmd,start_new_session=True)
        rc=child.wait();record('command_finished',returncode=rc)
        return rc
    except (KeyboardInterrupt,TimeoutError) as exc:
        record('interrupted',reason=str(exc))
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        return 130
    finally:
        if slot is not None:
            lease=json.loads((slot/'lease.json').read_text())
            if lease.get('who')!='dev2' or lease.get('token')!=token:
                record('release_refused',slot=str(slot),reason='lease ownership changed')
            else:
                (slot/'owner').unlink();(slot/'lease.json').unlink();slot.rmdir()
                record('released',slot=str(slot))


if __name__=='__main__':raise SystemExit(main())
