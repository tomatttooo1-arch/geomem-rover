#!/bin/bash
# 检测后处理价值闸：KV260 板上测量编排（未上板执行过；由主控择机安排，跑之前取 kv260 锁、确认机器安静）。
# 测量项与冻结阈值见 runs/nationals-prelim-20261102/DETPOST0/README.md §4；判读用 gate_eval.py。
#
# 用法：bash run_board_bench.sh <帧目录> <输出目录>
#   帧目录：优先用 dump_dpu_out.py 抓的真实 DPU 输出（dpu_*.bin）；没有时用 gen_inputs.py model 的仿真帧（model_*.bin），
#           gate_eval.py 会把来源写进结论。
#   环境变量：
#     GEO_CMD   几何核 rec30 回放命令，必须含 {OUT}（输出目录占位）。默认沿用上板二 kv2.sh 的产品驱动口径：
#               cd $GEO_STAGE && python3 scripts/py_arm_d2.py --prep $GEO_DATA/prep_rec30 --arm c --device devmem --out {OUT} --passes 1 --warmup 0
#     PERIOD_MS 并发时后处理的调用周期（= 一帧识别的周期；默认 60，按 DPU 中位时延改）
#     CPU       后处理钉的核（默认 3）；NC 类别数（默认 33）
set -u
F=${1:?frames dir}; O=${2:?out dir}
HERE=$(cd "$(dirname "$0")" && pwd)
BIN=$HERE/bench_detpost
CPU=${CPU:-3}; NC=${NC:-33}; PERIOD_MS=${PERIOD_MS:-60}
: "${GEO_CMD:?set GEO_CMD (with {OUT})}"
FR=$(ls "$F"/dpu_*.bin 2>/dev/null || ls "$F"/model_*.bin)
[ -n "$FR" ] || { echo "no frames in $F"; exit 1; }
mkdir -p "$O"
[ -x "$BIN" ] || g++ -O2 -mcpu=cortex-a53 -std=c++14 -I"$HERE/.." -o "$BIN" "$HERE/bench_detpost.cpp" \
  "$HERE/vitis_yolov8_port.cpp" "$HERE/detpost_fast.cpp" "$HERE/../detpost_ref.cpp" "$HERE/../detpost_host.cpp" || exit 1
{ echo "date $(date -Is)"; uname -a; uptime; cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq 2>/dev/null | tr '\n' ' '; echo;
  cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null; echo "frames: $(echo $FR | wc -w) from $F";
  sha256sum $FR | head -3; ps -eo pid,pcpu,cmd --sort=-pcpu | head -8; } > "$O/env.txt" 2>&1

geo() {  # $1 = 输出子目录名
  local out="$O/$1"; mkdir -p "$out"
  local cmd=${GEO_CMD//\{OUT\}/$out/geo}
  bash -c "$cmd" > "$out/geo.stdout" 2>&1; echo "geo $1 rc=$?"
}
"$BIN" --variant check $FR > "$O/check_fast.json" || { echo "fast != ref, stop"; exit 1; }
solo() {  # 后处理单跑：三种 C 实现各 2000 次、NumPy 300 次，冷缓存、钉核
  for v in ours fast vai; do "$BIN" --variant $v --iters 2000 --cpu $CPU --cold 1 --nc $NC --out "$O/solo_${v}_$1.json" $FR; done
  python3 "$HERE/bench_detpost_np.py" --iters 300 --cpu $CPU --nc $NC --out "$O/solo_numpy_$1.json" $FR
}
conc() {  # $1 = variant，$2 = 周期（0 = 背靠背压力），$3 = 标签
  local n=$(( $2 > 0 ? 40000 / $2 + 200 : 20000 ))
  "$BIN" --variant $1 --iters $n --cpu $CPU --cold 1 --nc $NC --period-ms $2 --out "$O/conc_$3_post.json" $FR &
  local pid=$!; sleep 1
  geo "geo_$3"
  kill $pid 2>/dev/null; wait $pid 2>/dev/null
}
# 交替顺序（A B C C B A），两头的几何基线用来估漂移
solo a
geo geo_base1
conc fast "$PERIOD_MS" fast_rate
conc ours "$PERIOD_MS" ours_rate
conc vai "$PERIOD_MS" vai_rate
conc ours 0 ours_stress
conc vai 0 vai_stress
geo geo_base2
solo b
echo "done -> $O ; next: python3 $HERE/gate_eval.py --run $O --dpu-timing <dpu_timing.json>"
