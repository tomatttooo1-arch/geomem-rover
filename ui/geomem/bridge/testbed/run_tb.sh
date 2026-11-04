#!/bin/sh
# GeoMem 联调试验台入口：把一个 integ 脚本整个放进 `unshare -rn` 隔离网络命名空间里跑
# （hub/回放服务器/桥/无头 Chrome 全在里面；宿主任何接口都不监听 hub 端口，owner 10-14）。
#
#   TB_TOOLS=<装了 ws、playwright-core 的 node_modules 所在目录> \
#   sh ui/geomem/bridge/testbed/run_tb.sh <integ 脚本名，如 t2_a1_clock.mjs> [脚本参数...]
#
# 脚本从 ui/geomem/tools/test/ 复制到 $TB_TOOLS（node 从脚本所在目录向上找 node_modules）。
set -eu
WT=$(cd "$(dirname "$0")/../../../.." && pwd)
: "${TB_TOOLS:?TB_TOOLS 须指向含 node_modules 的目录}"
mkdir -p "$TB_TOOLS/integ"
cp "$WT"/ui/geomem/tools/test/*.mjs "$TB_TOOLS/"
cp "$WT"/ui/geomem/tools/test/integ/*.mjs "$TB_TOOLS/integ/"
SCRIPT=$1; shift
export GEOMEM_TB_NETNS=1 GEOMEM_WT="$WT"
exec unshare -rn sh -c 'ip link set lo up && cd "$0/integ" && exec node "$@"' "$TB_TOOLS" "$SCRIPT" "$@"
