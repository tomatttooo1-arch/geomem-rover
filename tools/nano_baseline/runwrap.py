"""Run a command, append one JSON line with wall time, child CPU and peak RSS."""
import json, resource, subprocess, sys, time
out = sys.argv[1]; tag = sys.argv[2]; cmd = sys.argv[3:]
t0 = time.time(); w0 = time.perf_counter()
p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
wall = time.perf_counter() - w0
ru = resource.getrusage(resource.RUSAGE_CHILDREN)
rec = {"tag": tag, "start_unix": t0, "wall_s": round(wall, 3), "rc": p.returncode,
       "child_user_s": ru.ru_utime, "child_sys_s": ru.ru_stime, "child_maxrss_kb": ru.ru_maxrss,
       "stdout": p.stdout.strip().splitlines()[-3:], "stderr_tail": p.stderr.strip().splitlines()[-5:], "cmd": cmd}
with open(out, "a") as fp:
    fp.write(json.dumps(rec) + "\n")
sys.exit(p.returncode)
