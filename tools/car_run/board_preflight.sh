#!/usr/bin/env bash
# 上车前只读体检（在服务器上跑，ssh 到 Nano 执行；不改任何状态，10-25）。
#
#   bash tools/car_run/board_preflight.sh [--ref origin/main | --expect <sha>] [--host jetson]
#
# 每项一行 PASS / FAIL / INFO，末尾汇总；退出码 = FAIL 项数（Nano 不可达时为 1，其余项不跑）。
# 查：Nano 可达与主机名；console-current 指向的目录与 COMMIT 是否等于期望提交；车栈镜像在且含 rotation-shim；
#     nvpmodel 档位；jetson_clocks 频率（INFO）；/data 与 / 剩余空间；KV260 可达（ping＋ssh，数据面 HELLO 为 INFO）；
#     车栈状态（check_current.py，INFO；10-29 起附车栈自身体检 health.json 一行）；Orbbec 相机在 USB 上；
#     10-26：KV 已装比赛快照 COMMIT 是否等于期望提交（deploy_kv.sh 部署），KV 服务若在跑其 AR 是否开（INFO）。
# 唯一会新建东西的是 rotation-shim 检查用的 docker run --rm（§4b 的原命令，用完即删的临时容器）。
# 远端路径可用环境变量换（测试用）：FPGA_CAR_CURRENT FPGA_KV_SSH_CONF FPGA_IMAGE FPGA_DISK_PATHS FPGA_HUB_PORT
#     KV BUILD_ID 默认取 FPGA 档（tools/car_run/fpga_profiles.env；FPGA_PROFILE=a2 回退，10-26）。
set -uo pipefail
source "$(cd "$(dirname "$0")" && pwd)/fpga_profile.sh"; fpga_profile || exit 2

REF=origin/main
EXPECT=""
HOST=jetson
ROOT=/home/robot/fpga/dev1
NVP=MAXN_SUPER
MINGB=5
REPO=""
usage() { sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; }
while [ $# -gt 0 ]; do
  case "$1" in
    --ref) REF=${2:?}; shift 2 ;;
    --expect) EXPECT=${2:?}; shift 2 ;;
    --host) HOST=${2:?}; shift 2 ;;
    --root) ROOT=${2:?}; shift 2 ;;
    --repo) REPO=${2:?}; shift 2 ;;
    --expect-nvpmodel) NVP=${2:?}; shift 2 ;;
    --min-free-gb) MINGB=${2:?}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done
if [ -z "$EXPECT" ]; then
  [ -n "$REPO" ] || REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel 2>/dev/null) || REPO=.
  EXPECT=$(git -C "$REPO" rev-parse --verify -q "$REF^{commit}") || { echo "解析不了 ref：$REF（或用 --expect <sha>）" >&2; exit 2; }
fi
[[ "$EXPECT" =~ ^[0-9a-f]{7,40}$ ]] || { echo "期望提交号不像 sha：$EXPECT" >&2; exit 2; }

ARGS=(
  "$EXPECT" "$ROOT"
  "${FPGA_IMAGE:-fpga-ros:humble-nav-orbbec-scipy-20261017}"
  "${FPGA_CAR_CURRENT:-/data/fpga/car/state/car.current}"
  "${FPGA_HUB_PORT:-47801}"
  "${FPGA_KV_SERVER:-198.51.100.79:47800}"
  "${FPGA_KV_BUILD_ID:-$FPGA_BUILD_ID}"
  "${FPGA_KV_SSH_CONF:-/home/robot/.ssh/geomem-kv.conf}"
  "$NVP" "$MINGB"
  "${FPGA_DISK_PATHS:-/data /}"
  "${FPGA_KV_SOURCE:-/opt/geomem-dev3-kv/source}"
)
REMOTE="bash -s --"; for a in "${ARGS[@]}"; do REMOTE+=" $(printf '%q' "$a")"; done
# shellcheck disable=SC2206
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 ${DEPLOY_SSH_OPTS:-} "$HOST")

echo "== 上车体检 $HOST（期望提交 $EXPECT）"
out=$("${SSH[@]}" "$REMOTE" <<'REMOTE_EOF'
set -u
EXPECT=$1 ROOT=$2 IMAGE=$3 CURRENT=$4 HUBPORT=$5 KVSERVER=$6 KVBUILD=$7 KVCONF=$8 NVP=$9 MINGB=${10} DISKS=${11}
KVSRC=${12}
LINK=$ROOT/console-current
COMP=ui/geomem/tools/competition
echo "PASS Nano 可达：$(hostname)"
# --- 部署版本
if [ -L "$LINK" ]; then
  tgt=$(readlink "$LINK"); name=$(basename "$tgt"); commit=$(head -1 "$LINK/COMMIT" 2>/dev/null)
  short=${EXPECT:0:7}
  if [ -z "$commit" ]; then echo "FAIL console-current -> $tgt，但没有 COMMIT（car_up 会以 8 退出）"
  elif [ "$name" != "console-direct-$short" ] && [ "${commit:0:7}" = "$short" ]; then
    echo "FAIL console-current -> $tgt：COMMIT=$commit 对，但目录名不是 console-direct-$short（半截部署？）"
  elif [ "$commit" = "$EXPECT" ] || { [ ${#commit} -lt 40 ] && [ ${#commit} -ge 7 ] && [ "${EXPECT#"$commit"}" != "$EXPECT" ]; } \
       || { [ ${#EXPECT} -lt 40 ] && [ "${commit#"$EXPECT"}" != "$commit" ]; }; then
    if [ "$name" = "console-direct-$short" ]; then echo "PASS console-current -> $name，COMMIT=$commit"
    else echo "FAIL console-current -> $tgt：COMMIT 对但目录名不是 console-direct-$short"; fi
  else echo "FAIL console-current -> $tgt，COMMIT=$commit，期望 $EXPECT（用 deploy_nano.sh 部署）"; fi
else
  echo "FAIL 没有 $LINK 软链（用 deploy_nano.sh 部署）"
fi
# --- 镜像与 rotation-shim
if docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "PASS 镜像在：$IMAGE"
  shim=$(timeout 60 docker run --rm "$IMAGE" grep -i rotation-shim /opt/fpga/packages.tsv 2>/dev/null | head -1)
  if [ -n "$shim" ]; then echo "PASS 镜像含 rotation-shim：$shim"
  else echo "FAIL 镜像里 /opt/fpga/packages.tsv 没有 rotation-shim：起车栈前设 FPGA_NAV_ROTATION_SHIM=false"; fi
else
  echo "FAIL 镜像不在：$IMAGE（docker image inspect 失败）"
fi
# --- 功率档与频率
q=$(sudo -n nvpmodel -q 2>/dev/null || nvpmodel -q 2>/dev/null)
mode=$(echo "$q" | sed -n 's/.*Power Mode: *//p' | head -1)
if [ -z "$mode" ]; then echo "FAIL nvpmodel -q 读不出档位"
elif [ "$mode" = "$NVP" ]; then echo "PASS nvpmodel：$mode"
else echo "FAIL nvpmodel：$mode，期望 $NVP（切档：sudo nvpmodel -m <号>；Nano super：0=15W 1=25W 2=MAXN_SUPER 3=7W）"; fi
clk=$(sudo -n jetson_clocks --show 2>/dev/null || jetson_clocks --show 2>/dev/null)
if [ -n "$clk" ]; then
  gpu=$(echo "$clk" | grep -m1 '^GPU' | grep -o 'CurrentFreq=[0-9]*' | cut -d= -f2)
  cpus=$(echo "$clk" | grep '^cpu[0-9]' | grep -o 'CurrentFreq=[0-9]*' | cut -d= -f2 | sort -n | uniq | tr '\n' ' ')
  emc=$(echo "$clk" | grep -m1 '^EMC' | grep -o 'CurrentFreq=[0-9]*' | cut -d= -f2)
  echo "INFO jetson_clocks：GPU ${gpu:-?} Hz；CPU ${cpus:-?}Hz；EMC ${emc:-?} Hz"
else
  echo "INFO jetson_clocks --show 读不出（需 sudo）"
fi
# --- 磁盘
for d in $DISKS; do
  if [ -d "$d" ]; then
    avail=$(df -Pk "$d" | awk 'NR==2{print $4}'); gb=$((avail / 1048576))
    if [ "$gb" -ge "$MINGB" ]; then echo "PASS 剩余空间 $d：${gb} GB"
    else echo "FAIL 剩余空间 $d：${gb} GB（< ${MINGB} GB）"; fi
  else echo "FAIL $d 不存在"; fi
done
# --- KV260（经 Nano 直连口）
kvip=${KVSERVER%:*}
if ping -c 1 -W 2 "$kvip" >/dev/null 2>&1; then echo "PASS KV260 ping 通：$kvip"
else echo "FAIL KV260 ping 不通：$kvip（查网线 / KV 是否开机）"; fi
if [ -f "$KVCONF" ] && ssh -F "$KVCONF" -o BatchMode=yes -o ConnectTimeout=8 geomem-kv260 true >/dev/null 2>&1 </dev/null; then
  echo "PASS KV260 ssh 通（$KVCONF geomem-kv260）"
else echo "FAIL KV260 ssh 不通（$KVCONF geomem-kv260）"; fi
# --- 10-26 KV 比赛快照与车屏 AR（只读：cat / pgrep / 读 ready.json、ar_status.json）
if [ -f "$KVCONF" ]; then
  kvcmd=$(cat <<'KV_EOF'
src=$1
head -1 "$src/COMMIT" 2>/dev/null | sed 's/^/COMMIT=/'
test -x "$src/tools/kv_link/ar_helper/build/aarch64/m7ar_present" && echo HELPER=yes
for p in $(pgrep -f '[k]v_server.py'); do
  a=$(tr '\0' ' ' < "/proc/$p/cmdline"); echo "PROC=$a"
  o=$(echo "$a" | sed -n 's/.*--out \([^ ]*\).*/\1/p'); sed 's/^/STATUS=/' "$o/ar_status.json" 2>/dev/null
done
true
KV_EOF
)
  kvinfo=$(ssh -F "$KVCONF" -o BatchMode=yes -o ConnectTimeout=8 geomem-kv260 "bash -s -- $(printf '%q' "$KVSRC")" <<< "$kvcmd" 2>/dev/null)
  kvc=$(echo "$kvinfo" | sed -n 's/^COMMIT=//p' | head -1)
  if [ -z "$kvc" ]; then echo "FAIL KV 没有已装比赛快照（$KVSRC/COMMIT 读不到；用 deploy_kv.sh 部署）"
  elif [ "$kvc" = "$EXPECT" ] || [ "${kvc#"$EXPECT"}" != "$kvc" ] || [ "${EXPECT#"$kvc"}" != "$EXPECT" ]; then
    if echo "$kvinfo" | grep -qx HELPER=yes; then echo "PASS KV 已装快照 COMMIT=$kvc（含 AR 助手）"
    else echo "FAIL KV 已装快照 COMMIT=$kvc 对，但没有 AR 助手二进制（车屏会退回 off；用 deploy_kv.sh 重部署）"; fi
  else echo "FAIL KV 已装快照 COMMIT=$kvc，期望 $EXPECT（用 deploy_kv.sh 部署；Nano 和 KV 要同一提交）"; fi
  proc=$(echo "$kvinfo" | sed -n 's/^PROC=//p' | head -1)
  st=$(echo "$kvinfo" | sed -n 's/^STATUS=//p' | head -1)
  if [ -z "$proc" ]; then echo "INFO KV 服务未在跑（起栈时 nano_up 会起，AR 状态行见其日志）"
  else
    case "$proc" in *"--ar drm"*) echo "INFO KV 服务在跑，AR 开（车屏应亮）${st:+；$st}" ;;
      *) echo "INFO KV 服务在跑，AR 关（车屏不亮）${st:+；$st}" ;; esac
  fi
fi
probe=$LINK/tools/kv_link/hello_connect_loop.py
if [ -f "$probe" ]; then
  h=$(cd "$LINK" && PYTHONPATH=. timeout 15 python3 "$probe" --server "$KVSERVER" --trials 1 --hello-timeout-s 1.0 \
      --expect-build-id "$KVBUILD" --out /dev/stdout 2>&1)
  if echo "$h" | grep -q '"ok": true'; then echo "INFO KV260 服务在 $KVSERVER，BUILD_ID $KVBUILD"
  else echo "INFO KV260 服务 $KVSERVER 未应答或 BUILD_ID 不符（车栈未起时正常；起车栈时 car_up：未应答退 9；BUILD_ID 不符则告警、用实际那个照常起栈，未登记的走 Nano CPU 融合）"; fi
fi
# --- 车栈
cc=$LINK/$COMP/check_current.py
if [ ! -e "$CURRENT" ] && [ ! -L "$CURRENT" ]; then echo "INFO 车栈未起（无 $CURRENT）"
elif [ ! -f "$cc" ]; then echo "INFO 车栈记录在（$(readlink "$CURRENT")），但找不到 check_current.py"
else
  r=$(python3 "$cc" --current "$CURRENT" --hub-port "$HUBPORT" 2>&1); rc=$?
  if [ "$rc" = 0 ]; then echo "INFO 车栈在跑：$(basename "$(readlink -f "$CURRENT")")（check_current 0）"
  else echo "INFO 车栈记录过期：$(basename "$(readlink "$CURRENT")")（check_current $rc；console_remote.sh up 会先收尾）"; fi
  # 10-29 the running stack's own health check (hub, host/hub/health.py -> <run>/health.json; INFO only)
  hj="$(readlink -f "$CURRENT")/health.json"
  if [ -f "$hj" ]; then
    echo "INFO 车栈体检：$(python3 -c 'import json,sys
r=json.load(open(sys.argv[1]))
bad=["%s（%s；%s）" % (c.get("title"), c.get("detail"), c.get("hint") or "") for c in r.get("checks") or [] if c.get("level") in ("red", "yellow")]
print("%s %s%s" % (r.get("overall"), r.get("summary"), ("：" + "；".join(bad)) if bad else ""))' "$hj" 2>/dev/null || echo "health.json 读不出")"
  fi
fi
# --- 相机
cam=$(lsusb -d 2bc5: 2>/dev/null | head -1)
if [ -n "$cam" ]; then echo "PASS Orbbec 相机在 USB：$cam"
else echo "FAIL 没有 Orbbec 相机（lsusb -d 2bc5: 为空；插 USB 3 口）"; fi
REMOTE_EOF
)
rc=$?
if ! echo "$out" | grep -q '^PASS Nano 可达'; then
  echo "FAIL Nano 不可达（$HOST，ssh 返回 $rc）"
  echo "== 汇总：FAIL 1（其余项未检）"
  exit 1
fi
echo "$out"
n=$(echo "$out" | grep -c '^FAIL')
echo "== 汇总：PASS $(echo "$out" | grep -c '^PASS')，FAIL $n，INFO $(echo "$out" | grep -c '^INFO')"
exit "$n"
