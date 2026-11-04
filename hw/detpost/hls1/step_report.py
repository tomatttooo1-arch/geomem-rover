"""一步的记账：csynth 顶层资源/时钟、各模块 LUT 前几名与循环 II、pragma 警告计数；追加到 qor 表。
用法：python3 step_report.py <工程目录> <步名> [--csv 表] [--golden 目录 --cand csim 输出目录]"""
import argparse, glob, json, os, re, subprocess, sys
S = "${FPGA_HOME}/dev1/.claude-accounts/claude_dev13/skills/fpga-hls/scripts"
ap = argparse.ArgumentParser(); ap.add_argument("proj"); ap.add_argument("step"); ap.add_argument("--csv")
a = ap.parse_args()
sol = os.path.join(a.proj, "sol1")
d = json.loads(subprocess.run([S + "/hls_report.sh", "synth", sol], capture_output=True, text=True, check=True).stdout)
s = d["summary"]; r = s["resources"]
logs = sorted(glob.glob(sol + "/agent_logs/csynth_*.log"))
log = open(logs[-1], errors="ignore").read() if logs else ""
w207 = len(re.findall(r"WARNING: \[HLS 207-55\d\d\]", log))
rpt = open(sol + "/syn/report/csynth.rpt", errors="ignore").read()
ign = re.search(r"Ignored Pragmas", rpt) is not None
iis = []
for m in s.get("modules", []):
    for lp in m.get("loops", []) or []:
        if lp.get("pipeline_ii") not in (None, "", "-"):
            iis.append("%s:II=%s" % (lp["name"], lp["pipeline_ii"]))
mods = sorted(s.get("modules", []), key=lambda m: -int(m["resources"]["LUT"]))[:6]
print("%s LUT=%s FF=%s DSP=%s BRAM18=%s URAM=%s est=%sns lat=%s..%s  207-55xx=%d IgnoredPragmas=%s" % (
    a.step, r["LUT"], r["FF"], r["DSP"], r["BRAM_18K"], r["URAM"], s["estimated_clock_ns"], s["latency_best"], s["latency_worst"], w207, ign))
for m in mods:
    print("   %-48s LUT=%-6s FF=%-6s DSP=%s BRAM=%s" % (m["name"][:48], m["resources"]["LUT"], m["resources"]["FF"], m["resources"]["DSP"], m["resources"]["BRAM_18K"]))
print("   II:", " ".join(iis)[:600])
if a.csv:
    new = not os.path.exists(a.csv)
    with open(a.csv, "a") as f:
        if new: f.write("step,LUT,FF,DSP,BRAM18,URAM,est_ns,lat_best,lat_worst,w207_55,ignored_pragmas,loop_II\n")
        f.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%s,\"%s\"\n" % (a.step, r["LUT"], r["FF"], r["DSP"], r["BRAM_18K"], r["URAM"], s["estimated_clock_ns"], s["latency_best"], s["latency_worst"], w207, ign, " ".join(iis)))
