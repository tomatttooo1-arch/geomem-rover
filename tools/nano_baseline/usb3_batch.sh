#!/bin/bash
# Gemini 2 USB3 stream re-measurement batch (2026-09-26). Board-side, detached.
# Usage: usb3_batch.sh <probe_binary> <out_dir> [repeats]
# Each config: 2 s settle counted separately, then a measurement window long
# enough for >=450 frames at the requested rate. Configs are round-robin
# interleaved across repeats so slow drift does not land on one config.
set -u
PROBE=$1; OUT=$2; REP=${3:-3}
export LD_LIBRARY_PATH=$(ls -d ~/fpga/orbbec/OrbbecSDK_v2.9.3_*_linux_arm64/lib)
mkdir -p "$OUT"
{ date -Is; uname -a; nvpmodel -q 2>/dev/null | head -1; lsusb -t; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor;
  cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | tr '\n' ' '; echo; uptime; top -bn1 | head -12; } > "$OUT/env.txt" 2>&1
idle() {  # whole-system busy % over N seconds with no camera process
  python3 - "$1" <<'PY'
import sys, time
def rd():
    v=list(map(int,open('/proc/stat').readline().split()[1:11])); t=sum(v); return t-v[3]-v[4], t
b0,t0=rd(); time.sleep(float(sys.argv[1])); b1,t1=rd()
print('{"idle_window_s": %s, "system_cpu_busy_pct": %.2f}' % (sys.argv[1], 100.0*(b1-b0)/(t1-t0)))
PY
}
CONFIGS=(
  "d640_30|--depth 640x400@30:Y16 --seconds 20"
  "d1280_30|--depth 1280x800@30:Y16 --seconds 20"
  "d320_30|--depth 320x200@30:Y16 --seconds 20"
  "d1280_15|--depth 1280x800@15:Y16 --seconds 32"
  "d640_30_c720|--depth 640x400@30:Y16 --color 1280x720@30:MJPG --seconds 20"
  "d1280_30_c1080|--depth 1280x800@30:Y16 --color 1920x1080@30:MJPG --seconds 20"
  "d640_30_c720_imu|--depth 640x400@30:Y16 --color 1280x720@30:MJPG --imu --seconds 20"
)
idle 20 > "$OUT/idle_before.json"
for r in $(seq 1 "$REP"); do
  for c in "${CONFIGS[@]}"; do
    name=${c%%|*}; args=${c#*|}
    timeout 120 "$PROBE" $args --dump "$OUT/${name}_r$r.csv" > "$OUT/${name}_r$r.json" 2> "$OUT/${name}_r$r.err"
    echo "$(date -Is) ${name}_r$r rc=$? args=$args" >> "$OUT/runs.log"
    sleep 3
  done
done
idle 20 > "$OUT/idle_after.json"
echo "DONE $(date -Is)" >> "$OUT/runs.log"
