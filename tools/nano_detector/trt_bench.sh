#!/bin/bash
# Pure-inference timing of the YOLOv8n-seg FP16 engines with trtexec (board-side, detached).
# The enqueue thread is pinned to CPU cluster 1 (cores 4,5) for every arm, so the A/B pair
# 640 vs 320 never crosses clusters; order 640, 320, 640 (drift check).
# Usage: trt_bench.sh <engine_dir> <out_dir>
set -u
E=$1; OUT=$2; T=/usr/src/tensorrt/bin/trtexec; CORES=${CORES:-4,5}
mkdir -p "$OUT"
{ date -Is; nvpmodel -q 2>/dev/null | head -1; uptime; cat /proc/loadavg; } > "$OUT/env.txt"
tegrastats --interval 500 --logfile "$OUT/tegrastats.log" & TS=$!
sleep 5
i=0
for s in 640 320 640; do
  i=$((i + 1)); tag=trt_${s}_r$i
  echo "$(date +%s.%N) start $tag" >> "$OUT/phases.log"
  taskset -c $CORES $T --loadEngine="$E/yolov8n-seg-$s-fp16.engine" --warmUp=2000 --duration=20 --percentile=50,95,99 \
    --exportTimes="$OUT/$tag.times.json" > "$OUT/$tag.log" 2>&1
  echo "$(date +%s.%N) end $tag rc=$?" >> "$OUT/phases.log"
  sleep 3
done
kill $TS
python3 - "$OUT" <<'PY' > "$OUT/summary.json"
import json, sys, glob, os, re, statistics
d = sys.argv[1]; res = {}
def pct(v, q):
    v = sorted(v); return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]
for f in sorted(glob.glob(os.path.join(d, "*.times.json"))):
    tag = os.path.basename(f).split(".")[0]
    t = json.load(open(f))
    lat = [x["latencyMs"] for x in t]; gpu = [x["computeMs"] for x in t]
    log = open(os.path.join(d, tag + ".log")).read()
    thr = re.search(r"Throughput: ([\d.]+) qps", log)
    res[tag] = {"n": len(t), "latency_ms": {"median": round(statistics.median(lat), 3), "p95": round(pct(lat, .95), 3), "max": round(max(lat), 3)},
                "gpu_compute_ms": {"median": round(statistics.median(gpu), 3), "p95": round(pct(gpu, .95), 3), "max": round(max(gpu), 3)},
                "throughput_qps": float(thr.group(1)) if thr else None}
print(json.dumps(res, indent=1))
PY
echo DONE
