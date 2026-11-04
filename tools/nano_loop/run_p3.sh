#!/bin/bash
# Board-side wrapper for one nano_loop run with tegrastats (1 s) and a summary.
# usage: run_p3.sh <out_dir> [nano_loop args...]      (run under nohup for long runs)
# Writes <out_dir>/DONE or FAILED (by nano_loop), plus <out_dir>/summary.json.
set -u
OUT=$1; shift
SRC=${P3_SRC:-$HOME/fpga/p3/code/src}
ENGINE=${P3_ENGINE:-$HOME/fpga/p2/w/yolov8n-seg-640-fp16.engine}
cd "$SRC" || exit 2
export PYTHONPATH=$HOME/fpga/p2/pylib:$SRC
mkdir -p "$(dirname "$OUT")"
tegrastats --interval 1000 --logfile "$OUT.tegra.log" &
TG=$!
python3 tools/nano_loop/nano_loop.py --engine "$ENGINE" --out "$OUT" "$@" > "$OUT.log" 2>&1
rc=$?
kill $TG 2>/dev/null
wait $TG 2>/dev/null
[ -d "$OUT" ] && mv "$OUT.tegra.log" "$OUT/tegrastats.log" && mv "$OUT.log" "$OUT/stdout.log"
[ -d "$OUT" ] && python3 tools/nano_loop/summarize.py "$OUT" > /dev/null 2> "$OUT/summarize.err"
echo "rc=$rc"
exit $rc
