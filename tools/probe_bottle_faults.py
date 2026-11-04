#!/usr/bin/env python3
"""Fault probes in a held, successful bottle session; only its own processes.

Run after run_agx_bottle.py --hold-after-scenario 180 prints
SCENARIO_COMPLETE_HOLD into agx-session.log. Records deliberate fault windows
separately from the clean bottle acceptance run. Never publishes a motion permit.
"""
import argparse
import json
import math
import os
import shlex
import signal
import subprocess
import time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('--ssh-alias',default='agx');p.add_argument('--port',type=int,default=18731)
    args=p.parse_args();run=args.run.resolve();session='bottle-'+run.name
    root='/home/robot/fpga';remote_out=root+'/runs/'+session
    revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    release=root+'/releases/'+revision[:12]
    container='fpga-dev1-'+session
    ssh=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=10',args.ssh_alias]
    if 'SCENARIO_COMPLETE_HOLD' not in (run/'agx-session.log').read_text():
        raise SystemExit('Scenario must have completed successfully and still be held')
    stream=(run/'fault-probe.jsonl').open('x');results=[];suspended=None;paused=False

    def log(event,**fields):
        row={'event':event,'wall_monotonic':time.monotonic(),**fields}
        stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush();print(json.dumps(row,ensure_ascii=False),flush=True)

    def command(argv):
        log('command',argv=argv)
        r=subprocess.run(argv,capture_output=True,text=True,timeout=20)
        log('command_result',returncode=r.returncode,stdout=r.stdout,stderr=r.stderr)
        r.check_returncode();return r.stdout

    def remote(words):return command(ssh+[shlex.join(words)])

    def ros(words):
        return remote(['env','FPGA_AGX_WORKSPACE='+root,'bash',release+'/tools/agx_rootfs.sh',
                       root+'/rootfs/humble-arm64-bfa8450-20260907',release]+words)

    def rows():
        with (run/'motion.jsonl').open('rb') as f:
            f.seek(max(0,(run/'motion.jsonl').stat().st_size-300000))
            lines=f.read().splitlines()
        result=[]
        for line in lines:
            try:result.append(json.loads(line))
            except (ValueError,UnicodeDecodeError):pass
        return result

    def moving(r):return abs(r['message']['linear']['x'])+abs(r['message']['angular']['z'])>.001

    def wait_motion():
        begin=time.monotonic()
        ros(['ros2','topic','pub','--once','/fpga/patrol','std_msgs/msg/Empty','{}'])
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            recent=[r for r in rows() if r['wall_monotonic']>=begin and r['topic']=='/fpga/cmd_vel_safe' and moving(r)]
            if recent:
                log('positive_motion_control',sample=recent[-1]);return
            time.sleep(.1)
        raise RuntimeError('No actual fresh nonzero protected command before fault')

    def stop_window(start,physical=True):
        time.sleep(3)
        recent=[r for r in rows() if start+1.5<r['wall_monotonic']<start+3]
        safe=[r for r in recent if r['topic']=='/fpga/cmd_vel_safe']
        odom=[r for r in recent if r['topic']=='/odom']
        zero=bool(len(safe)>=5 and not any(moving(r) for r in safe))
        still=bool(odom and all(abs(r['message']['twist']['twist']['linear']['x'])+
                    abs(r['message']['twist']['twist']['angular']['z'])<.01 for r in odom)) if physical else None
        log('stop_window',begin=start+1.5,end=start+3,safe_samples=len(safe),odom_samples=len(odom),
            zero_commands=zero,physical_stopped=still)
        return zero and (still if physical else True)

    def runtime_stats():
        code="import json,pathlib; r=[json.loads(x) for x in pathlib.Path(%r).read_text().splitlines()]; f=[x for x in r if x['event']=='frame_processed']; print(json.dumps({'frames':len(f),'last':f[-1]['frame_id'] if f else None}))" % (remote_out+'/runtime.jsonl')
        return json.loads(remote(['python3','-c',code]))

    def controller_pids():
        code="import pathlib,json; a=[]\nfor p in pathlib.Path('/proc').glob('[0-9]*/cmdline'):\n try:\n  v=p.read_bytes().split(b'\\0');\n  if v[0].endswith(b'/nav2_controller/controller_server'):a.append(int(p.parent.name))\n except (OSError,IndexError):pass\nprint(json.dumps(a))"
        return json.loads(remote(['python3','-c',code]))

    def resume_check(before):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if runtime_stats()['frames']>before['frames']:break
            time.sleep(.3)
        else:raise RuntimeError('New AGX frames did not resume')
        return stop_window(time.monotonic())

    def world_pause(value):
        command(['docker','exec',container,'ign','service','-s','/world/fpga_room/control',
                 '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
                 '--timeout','2000','--req','pause: '+str(value).lower()])

    try:
        # Freeze the actual SSH transport; local Nav2 and simulator stay alive.
        wait_motion();controllers=controller_pids();assert len(controllers)==1
        matches=[]
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:argv=path.read_bytes().split(b'\0')
            except OSError:continue
            if argv[0].endswith(b'ssh') and b'-N' in argv and f'127.0.0.1:{args.port}:127.0.0.1:{args.port}'.encode() in argv:
                matches.append(int(path.parent.name))
        if len(matches)!=1:raise RuntimeError('Cannot uniquely identify this tunnel')
        before=runtime_stats();suspended=matches[0];log('ssh_suspended',pid=suspended)
        start=time.monotonic();os.kill(suspended,signal.SIGSTOP)
        stopped=stop_window(start)
        before=runtime_stats()
        os.kill(suspended,signal.SIGCONT);suspended=None;log('ssh_resumed')
        restored=resume_check(before)
        results.append({'case':'ssh_transport_suspended','stopped':stopped,'no_old_motion_on_resume':restored,
                        'nav2_remained_alive':controller_pids()==controllers})
        # Pause simulation time while the runtime's wall watchdog remains live.
        wait_motion();before=runtime_stats();start=time.monotonic();world_pause(True);paused=True
        stopped=stop_window(start,physical=False)
        before=runtime_stats()
        world_pause(False);paused=False;log('clock_resumed')
        restored=resume_check(before)
        results.append({'case':'simulation_clock_paused','zero_commands_while_paused':stopped,
                        'no_old_motion_on_resume':restored,'nav2_remained_alive':controller_pids()==controllers})
        # Kill only this runtime; leave Nav2 producing commands to test the permit guard.
        wait_motion();start=time.monotonic()
        code="import pathlib,os,signal,json; ids=[]\nfor p in pathlib.Path('/proc').glob('[0-9]*/cmdline'):\n try:\n  a=p.read_bytes().decode().split('\\0');\n  if a[1:3]==['-m','host.ros_runtime'] and %r in a:ids.append(int(p.parent.name))\n except (OSError,UnicodeDecodeError):pass\nassert len(ids)==1,ids\nprint(json.dumps({'killed_runtime_pid':ids[0]}),flush=True)\nos.kill(ids[0],signal.SIGKILL)" % ('/workspace/runs/'+session+'/runtime.jsonl')
        remote(['python3','-c',code]);stopped=stop_window(start)
        unguarded=[r for r in rows() if r['wall_monotonic']>start+1.5 and r['topic']=='/cmd_vel' and moving(r)]
        results.append({'case':'runtime_killed','stopped':stopped,'nav2_remained_alive':controller_pids()==controllers,
                        'nav2_still_sent_nonzero_commands':bool(unguarded)})
        passed=all(all(v is True for k,v in case.items() if k!='case') for case in results)
        log('fault_probe_result',passed=passed,cases=results)
        if not passed:raise RuntimeError('Fault probe failed; see original windows')
    finally:
        if suspended is not None:os.kill(suspended,signal.SIGCONT)
        if paused:world_pause(False)
        remote(['touch',remote_out+'/finish-hold'])
        stream.close()


if __name__=='__main__':main()
