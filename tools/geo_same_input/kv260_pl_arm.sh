#!/bin/bash
# P8 arm C on the real KV260 PL, same recording as the Nano arms. Run ON THE KV260 after
# `sudo xmutil loadapp m7_pl` succeeded (docs/design/KV260_BRINGUP_CHECKLIST_20260928.md step 8).
#   sudo bash kv260_pl_arm.sh <bundle_dir> [out_dir]
# <bundle_dir> is the unpacked kv260_bundle.tgz made on the Nano: code/, prep_rec30/, nano_ref/.
# Runs, each 1 warm-up + 3 x 310 frames, then compares voxel by voxel:
#   A_kv_p15 / A_kv_free  repo CPU reference on the KV260 A53 (local reference, and the A53's own number)
#   C_pl_p15 / C_pl_free  backend "pl" with DevMemPlDevice (/dev/mem registers + a DMA buffer provider; no fallback)
# DMA buffer provider (docs/design/PL_DMA_PROVIDERS_20260927.md): auto = u-dma-buf, else dma_heap (CMA, root,
# cached + DC CVAC/CIVAC); /dev/mem physmem only when named. Force one with, e.g.
#   sudo FPGA_M7_DMA_PROVIDER=dmaheap FPGA_M7_DMA_HEAP=reserved bash kv260_pl_arm.sh <bundle_dir>
#   sudo FPGA_M7_DMA_PROVIDER=physmem FPGA_M7_PHYS_BASE=0x70000000 FPGA_M7_PHYS_SIZE=0x2000000 bash kv260_pl_arm.sh <bundle_dir>
# (variables must come after sudo: sudo drops the caller's environment). The provider used is printed and written
# to env.txt, dma_provider.json and kv_summary.txt.
# References: A_kv_p15 (same board) and the Nano's A_p15 dump (same input, other CPU).
# Writes <out>/DONE or <out>/FAILED; summary with tools/geo_same_input/summarize.py-like columns in <out>/kv_summary.txt.
set -u
BUN=$(readlink -f "$1"); OUT=${2:-$BUN/runs/kv_$(date +%Y%m%d_%H%M%S)}
# PREP / REF may be overridden (e.g. PREP=$BUN/prep_tum200 for the TUM subset; the Nano
# dump compare is skipped when $REF/A_p15.dump does not exist).
R=$BUN/code; PREP=${PREP:-$BUN/prep_rec30}; REF=${REF:-$BUN/nano_ref}
PASSES=${PASSES:-3}; WARMUP=${WARMUP:-1}; HZ=${HZ:-15}
mkdir -p "$OUT/runs"; PH="$OUT/phases.log"
ph() { echo "$(date +%s.%N) $*" >> "$PH"; }
fail() { ph "FAILED $*"; echo "$*" | tee "$OUT/FAILED"; exit 1; }
DEVICE=${DEVICE:-devmem}   # DEVICE=mock only for a dry run of this script without PL (e.g. on the Nano)
command -v g++ >/dev/null || fail "g++ missing (arm A compiles hw/m7/frame_depth.cpp; dma_heap compiles its cache helper): sudo apt install g++"
python3 -c 'import numpy' 2>/dev/null || fail "numpy missing: sudo apt install python3-numpy"
{ echo "## ls /sys/class/u-dma-buf /sys/class/udmabuf /dev/dma_heap"; ls -l /sys/class/u-dma-buf /sys/class/udmabuf /dev/dma_heap 2>&1
  echo "## cma"; grep -i cma /proc/meminfo 2>&1; echo "## cmdline"; cat /proc/cmdline
  echo "## FPGA_M7_* environment"; env | grep '^FPGA_M7_' ; } > "$OUT/dma_inventory.txt" 2>&1
if [ "$DEVICE" = devmem ]; then
[ "$(id -u)" = 0 ] || fail "run with sudo: DevMemPlDevice maps /dev/mem (and dma_heap needs root for /proc/self/pagemap)"
# Open the PL device once with the same rules py_arm.py uses; on failure print every provider's reason.
PYTHONPATH="$R" python3 - "$OUT/dma_provider.json" > "$OUT/dma_provider.txt" 2>&1 <<'PY'
import json, sys
from host.m7_pl_device import DevMemPlDevice, PlDeviceUnavailable
try:
    d = DevMemPlDevice.open()
except PlDeviceUnavailable as exc:
    print("UNAVAILABLE %s" % exc)
    print("counts %s" % dict(exc.counts))
    sys.exit(3)
info = d.describe()
d.close()
json.dump(info, open(sys.argv[1], "w"), indent=1, sort_keys=True)
print("DMA buffer provider: %s (cached=%s, heap=%s, region_pa=%s, barrier=%s)" % (
    info["provider"], info["cached"], info.get("heap", "-"), info.get("region_pa", "-"), info["barrier"]))
print("attempts: %s" % " | ".join(info["attempts"]))
PY
rc=$?; cat "$OUT/dma_provider.txt"
[ $rc = 0 ] || fail "no usable DMA buffer provider for the Python PL device ($(head -1 "$OUT/dma_provider.txt")); inventory in $OUT/dma_inventory.txt; fallback route: docs/design/PL_DMA_PROVIDERS_20260927.md section 7.1 (build u-dma-buf)"
fi
EXPECT=$(python3 -c "import json;print(json.load(open('$PREP/meta.json'))['depth_sha256'])")
GOT=$(sha256sum "$PREP/depth_u16.bin" | cut -d' ' -f1)
[ "$EXPECT" = "$GOT" ] || fail "depth_u16.bin sha256 $GOT != meta $EXPECT (copy damaged)"
{ date -Is; echo "device=$DEVICE"; uname -a; cat /etc/os-release | head -2; xmutil listapps 2>&1
  cat "$OUT/dma_provider.txt" 2>/dev/null; cat "$OUT/dma_inventory.txt"
  nproc; cat /proc/loadavg; g++ --version | head -1; python3 -c 'import numpy,sys;print("numpy",numpy.__version__,"python",sys.version.split()[0])'
  cat "$R/COMMIT"; echo "depth sha256 $GOT"; top -bn1 | head -12; } > "$OUT/env.txt" 2>&1
export PYTHONPATH="$R"
run() { tag=$1; shift; ph "start $tag"; "$@" > "$OUT/runs/$tag.stdout" 2> "$OUT/runs/$tag.stderr"; rc=$?; ph "end $tag rc=$rc"; sleep 3; return $rc; }
for mode in p15 free; do
  if [ $mode = p15 ]; then PACE=$HZ; else PACE=0; fi
  run A_kv_$mode taskset -c 1 python3 "$R/tools/geo_same_input/py_arm.py" --prep "$PREP" --arm a --out "$OUT/runs/A_kv_$mode" \
      --passes $PASSES --warmup $WARMUP --pace-hz $PACE || fail A_kv_$mode
  run C_pl_$mode taskset -c 1 python3 "$R/tools/geo_same_input/py_arm.py" --prep "$PREP" --arm c --device $DEVICE --out "$OUT/runs/C_pl_$mode" \
      --passes $PASSES --warmup $WARMUP --pace-hz $PACE || fail "C_pl_$mode (see $OUT/runs/C_pl_$mode.stderr)"
done
python3 "$R/tools/geo_same_input/compare.py" --ref "$OUT/runs/A_kv_p15/flags.dump" \
  --cand C_pl_p15="$OUT/runs/C_pl_p15/flags.dump" --cand C_pl_free="$OUT/runs/C_pl_free/flags.dump" \
  --cand A_kv_free="$OUT/runs/A_kv_free/flags.dump" \
  $( [ -f "$REF/A_p15.dump" ] && echo --cand nano_A_p15="$REF/A_p15.dump" ) --out "$OUT/compare_kv.json" > "$OUT/compare_kv.stdout" 2>&1
echo "compare rc=$?" >> "$OUT/compare_kv.stdout"
if [ -f "$REF/A_p15.dump" ]; then
python3 "$R/tools/geo_same_input/compare.py" --ref "$REF/A_p15.dump" \
  --cand C_pl_p15="$OUT/runs/C_pl_p15/flags.dump" --out "$OUT/compare_vs_nano.json" >> "$OUT/compare_kv.stdout" 2>&1
echo "compare_vs_nano rc=$?" >> "$OUT/compare_kv.stdout"
fi
python3 - "$OUT" > "$OUT/kv_summary.txt" <<'EOF'
import csv, json, os, statistics, sys
d = sys.argv[1]
def pct(v, q): v = sorted(v); return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]
for tag in ("A_kv_p15", "A_kv_free", "C_pl_p15", "C_pl_free"):
    rows = list(csv.DictReader(open(os.path.join(d, "runs", tag, "frames.csv"))))
    s = json.load(open(os.path.join(d, "runs", tag, "summary.json")))
    col = lambda k: [float(r[k]) for r in rows if r[k] != ""]
    line = [tag, s["backend_name"], "frames=%d" % len(rows), "late=%d" % s["paced_late_starts"]]
    for k in ("core_ms", "device_ms", "e2e_ms"):
        v = col(k)
        if v: line.append("%s med/p95/max %.2f/%.2f/%.2f" % (k, statistics.median(v), pct(v, .95), max(v)))
    cpu = s["measured_cpu_user_s"] + s["measured_cpu_sys_s"]
    line.append("proc_core_equiv %.2f cpu_ms/frame %.2f" % (cpu / s["measured_wall_s"], cpu / len(rows) * 1e3))
    line.append("counts %s" % s["backend_counts"])
    pl = s.get("pl_device")
    if pl:
        line.append("dma %s cached=%s heap=%s region_pa=%s" % (pl["provider"], pl["cached"], pl.get("heap", "-"),
                                                               pl.get("region_pa", "-")))
    print(" | ".join(line))
print(open(os.path.join(d, "compare_kv.stdout")).read())
EOF
ph DONE; touch "$OUT/DONE"; cat "$OUT/kv_summary.txt"
