#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# vv_run.sh —— vivado-checks 统一入口（本项目新写，非上游文件）
# 用法：
#   vv_run.sh lint   [--top T] [--part P] [--inc DIR]... [--define X]... [--out DIR] SRC...   # SRC=文件/目录/.f
#   vv_run.sh lint   --xpr proj.xpr [--top T] [--out DIR]
#   vv_run.sh elab   (同 lint 参数) [--skip-ip]                 # synth_design -rtl，解析 [Synth 8-xxx]
#   vv_run.sh elab-log <vivado.log|runme.log> [--out DIR]      # 不起 Vivado，只解析已有日志
#   vv_run.sh timing --dcp routed.dcp [--out DIR] [--skip-methodology]
#   vv_run.sh timing --xpr proj.xpr [--run impl_1] [--out DIR]
#   vv_run.sh id-info TIMING-16                                # 上游 get_id_info.sh
# 环境：VV_TIMEOUT(秒,默认900) VV_THREADS(默认4) VV_SETTINGS(默认 2022.2 settings64.sh)
#       VV_NO_UDEV_PRELOAD=1 关闭 libudev 预加载（本机 2022.2 许可库段错误的规避，见 porting_notes.md）
# 终端只打 VV_SUMMARY 摘要与输出目录；完整日志在 <out>/vivado.log。
set -u
SKILL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS="$SKILL_DIR/scripts"
REF="$SKILL_DIR/reference"
VV_SETTINGS="${VV_SETTINGS:-/tools/Xilinx/Vivado/2022.2/settings64.sh}"
VV_TIMEOUT="${VV_TIMEOUT:-900}"

die() { echo "vv_run: $*" >&2; exit 2; }
[ $# -ge 1 ] || { sed -n '3,15p' "${BASH_SOURCE[0]}"; exit 2; }
CHECK="$1"; shift

OUT="" ; SRCS=() ; INCS=() ; DEFS=()
export VV_TOP="" VV_PART="" VV_XPR="" VV_DCP="" VV_RUN="" VV_SKIP_IP="" VV_SKIP_METHODOLOGY=""
LOGIN=""
while [ $# -gt 0 ]; do
  case "$1" in
    --top) VV_TOP="$2"; shift 2;;
    --part) VV_PART="$2"; shift 2;;
    --inc) INCS+=("$(readlink -f "$2")"); shift 2;;
    --define) DEFS+=("$2"); shift 2;;
    --out) OUT="$2"; shift 2;;
    --xpr) VV_XPR="$(readlink -f "$2")"; shift 2;;
    --dcp) VV_DCP="$(readlink -f "$2")"; shift 2;;
    --run) VV_RUN="$2"; shift 2;;
    --skip-ip) VV_SKIP_IP=1; shift;;
    --skip-methodology) VV_SKIP_METHODOLOGY=1; shift;;
    -h|--help) sed -n '3,15p' "${BASH_SOURCE[0]}"; exit 0;;
    *) if [ "$CHECK" = elab-log ] || [ "$CHECK" = id-info ]; then LOGIN="$1"; else SRCS+=("$(readlink -f "$1")"); fi; shift;;
  esac
done

if [ "$CHECK" = id-info ]; then exec bash "$REF/vivado-timing-methodology-checks/scripts/get_id_info.sh" "$LOGIN"; fi

TS="$(date +%Y%m%d-%H%M%S)"
OUT="${OUT:-$PWD/vivado_checks_out/$CHECK-$TS}"
mkdir -p "$OUT" || die "cannot create $OUT"
OUT="$(readlink -f "$OUT")"
: > "$OUT/summary.txt"
export VV_OUT="$OUT" VV_SRCS="${SRCS[*]:-}" VV_INCS="${INCS[*]:-}" VV_DEFS="${DEFS[*]:-}"
echo "VV_SUMMARY out=$OUT"

run_vivado() {  # $1 = tcl
  ( cd "$OUT" || exit 97
    # shellcheck disable=SC1090
    source "$VV_SETTINGS" >/dev/null 2>&1
    if [ -z "${VV_NO_UDEV_PRELOAD:-}" ] && [ -e /lib/x86_64-linux-gnu/libudev.so.1 ]; then
      export LD_PRELOAD="/lib/x86_64-linux-gnu/libudev.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
    fi
    timeout "$VV_TIMEOUT" vivado -mode batch -nojournal -log "$OUT/vivado.log" -source "$1" > "$OUT/vivado.stdout" 2>&1
  )
  local rc=$?
  echo "VV_SUMMARY vivado_rc=$rc"
  [ $rc = 124 ] && echo "VV_SUMMARY TIMEOUT after ${VV_TIMEOUT}s —— 按纪律停下并记录，不要盲目重跑"
  grep -h '^VV_SUMMARY\|^VV_WARN\|^VV_ERROR' "$OUT/vivado.stdout" | grep -v '^VV_SUMMARY out='
  return $rc
}

case "$CHECK" in
  lint)
    run_vivado "$SCRIPTS/vv_lint.tcl"
    if [ -s "$OUT/linter.rpt" ]; then
      python3 -I "$REF/vivado-rtl-lint/parse_lint_report.py" "$OUT/linter.rpt" "$OUT/linter.csv"
      python3 -I "$SCRIPTS/vv_post.py" lint "$OUT/linter.csv" "$OUT/vivado.log" | tee -a "$OUT/summary.txt"
    fi
    grep -m5 '^ERROR' "$OUT/vivado.log" 2>/dev/null
    ;;
  elab)
    run_vivado "$SCRIPTS/vv_elab.tcl"
    LOGIN="$OUT/vivado.log" ;&
  elab-log)
    [ -n "$LOGIN" ] && [ -f "$LOGIN" ] || die "need a log file"
    python3 -I "$REF/vivado-rtl-elaboration-analysis/parse_elab_messages.py" "$LOGIN" "$OUT/messages.csv" > "$OUT/parse_elab.out" 2>&1
    tail -n 3 "$OUT/parse_elab.out"
    [ -f "$OUT/messages.csv" ] && python3 -I "$SCRIPTS/vv_post.py" elab "$OUT/messages.csv" "$REF" | tee -a "$OUT/summary.txt"
    grep -E -m8 '^(ERROR|CRITICAL WARNING): \[Synth 8-' "$LOGIN"
    ;;
  timing)
    run_vivado "$SCRIPTS/vv_timing.tcl"
    [ -f "$OUT/timing_summary.rpt" ] && python3 -I "$SCRIPTS/vv_post.py" timing "$OUT/timing_summary.rpt" | tee -a "$OUT/summary.txt"
    [ -f "$OUT/methodology_counts.csv" ] && { echo "VV_SUMMARY methodology_counts:"; column -t -s, "$OUT/methodology_counts.csv" | head -40; }
    ;;
  *) die "unknown check '$CHECK' (lint|elab|elab-log|timing|id-info)";;
esac
echo "VV_SUMMARY done; 全量日志 $OUT/vivado.log ；摘要 $OUT/summary.txt"
