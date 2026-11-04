#!/usr/bin/env bash
# remote_run_template.sh — 在远端构建机上起一次整板 Vivado 构建，并与登录会话脱钩（ssh 断线不影响）。
# 由我们的远端启动器改写成通用模板（原版在本地启动器之外只做了路径替换，未改构建 Tcl）。本模板未在提交包内重跑。
# usage: remote_run_template.sh <build_dir 必须不存在> <env_file> <build_tcl>
#   env_file : 构建 Tcl 需要的环境变量（输入 IP、ip_repo 等路径已改写成远端副本路径）
#   VIVADO_SETTINGS : 远端 Vivado settings64.sh（默认 /tools/Xilinx/Vivado/2022.2/settings64.sh）
# 产物: <build_dir>/（Vivado 输出）、<build_dir>.log、<build_dir>.pid（只按这个 pid 停）、<build_dir>.rc、同级 LEDGER.txt 的 START/EXIT 行
# 注意: rc=0 只说明 Vivado 跑完，负 WNS 也是 rc=0；拉回后必须再判时序。
set -u
OUT=${1:?build dir}; ENVF=${2:?env file}; TCL=${3:?build tcl}
SET=${VIVADO_SETTINGS:-/tools/Xilinx/Vivado/2022.2/settings64.sh}
LEDGER=$(dirname "$OUT")/LEDGER.txt
[ -e "$OUT" ] && { echo "build dir exists: $OUT" >&2; exit 2; }
[ -f "$TCL" ] || { echo "no build tcl: $TCL" >&2; exit 2; }
echo "$(date -Is) START build out=$OUT env=$ENVF tcl=$TCL" >> "$LEDGER"
setsid nohup bash -c '
  OUT=$0; ENVF=$1; TCL=$2; LEDGER=$3; SET=$4
  source "$SET" >/dev/null 2>&1
  source "$ENVF"
  mkdir -p "$OUT.run" && cd "$OUT.run" || exit 97
  export BUILD_OUT="$OUT"
  vivado -mode batch -nojournal -source "$TCL" > "$OUT.log" 2>&1 &
  echo $! > "$OUT.pid"; wait $!; rc=$?
  if [ $rc = 0 ]; then echo DONE >> "$OUT.log"
  elif [ $rc = 3 ]; then echo "POSTPLACE_STOP exit=3" >> "$OUT.log"
  else echo "FAILED exit=$rc" >> "$OUT.log"; fi
  echo "$(date -Is) EXIT build out=$OUT rc=$rc" >> "$LEDGER"
  echo $rc > "$OUT.rc"
' "$OUT" "$ENVF" "$TCL" "$LEDGER" "$SET" > /dev/null 2>&1 < /dev/null &
echo "launched remote build $OUT"
