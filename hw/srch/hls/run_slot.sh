#!/usr/bin/env bash
# SRCHK3：取一个空构建槽（只用 1..3；MemAvailable >= 12 GB）后调 build.sh，用完先删 owner 再 rmdir。
# 用法：run_slot.sh <工程名> <step>...
name=$1; shift
here="$(cd "$(dirname "$0")" && pwd)"
L=${FPGA_LOCKS}
slot=""
while [ -z "$slot" ]; do
  for i in 1 2 3 5; do
    m=$(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo)
    if [ "$m" -ge 12 ] && mkdir $L/nat_build_slot_$i 2>/dev/null; then
      slot=$L/nat_build_slot_$i; echo "dev1 ${SR_TAG:-SRCHK5} hls $name ($*) pid $$ $(date -Iseconds)" > $slot/owner
      echo "$(date -Iseconds) TAKE nat_build_slot_$i dev1 ${SR_TAG:-SRCHK5} $name pid $$" >> $L/HISTORY.log; break
    fi
  done
  [ -z "$slot" ] && sleep 30
done
trap '[ -n "$slot" ] && rm -f $slot/owner && rmdir $slot && echo "$(date -Iseconds) RELEASE $(basename $slot) dev1 ${SR_TAG:-SRCHK5} $name pid $$" >> $L/HISTORY.log' EXIT
echo "SLOT $slot $(date +%T)"
if [ -n "$SR_EXISTING" ]; then   # 在已建好的工程上续跑（不重建、不重拷源码），如 cosim / impl
  run=${FPGA_RUN}
  root=${SR_BUILD_ROOT:-$run/SRCHK5/build}
  vec=${SR_VEC:-"$run/SRCHK1/data/trace_v1.bin $run/SRCHK1/data/trace_v2.bin"}
  source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh >/dev/null 2>&1
  ${FPGA_HOME}/dev1/.claude-global/skills/fpga-hls/scripts/hls_run.sh --argv "$vec" "$root/$name" sol1 "$@"
else
  "$here/build.sh" "$name" "$@"
fi
rc=$?
echo "ALL_DONE rc=$rc $(date +%T)"
exit $rc
