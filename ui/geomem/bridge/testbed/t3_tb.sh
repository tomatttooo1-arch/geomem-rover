#!/bin/bash
# GeoMem 联调第 2 棒 T3 试验台（宿主侧）。车由 tools/car_run/car_up.sh sim 起；本脚本只做：
#   t3_tb.sh check                      互斥检查（car.current / wd-rtabloc-g4* / run_full / Gazebo / 1 分钟负载 ≤ 24），不满足退出 1
#   t3_tb.sh run <名> <输出目录> <integ 脚本> [参数...]
#                                       不加 -p：hub/桥端口都不上宿主），跑 node 脚本，输出写 <输出目录>/console.log；前台等它退出，
#                                       期间替脚本放第二只瓶（见 bottle）
#   t3_tb.sh bottle <i> <输出目录>      在 Gazebo 里加第 i 个位置的第二只瓶（P1 (1.25,0.25)、P2 (1.25,-0.90)；第 3 棒远瓶
#                                       Q1..Q3 = i 3..5：(1.9,-1.3)、(1.9,1.3)、(1.6,-1.1)），模型名 bottle_prop_<i+1>，写 <输出目录>/bottle_<i>.done
#   t3_tb.sh remove <i> <输出目录>      删掉第 i 个位置放的那只瓶（第 3 棒），写 <输出目录>/remove_<i>.done
#   t3_tb.sh a10 <输出目录>             A10：不连/桥/fpgapc 交错各两次，每窗 120 s，读 hub 进程宿主 /proc/<pid>/stat
#                                       （A10_LOADWAIT=0：每窗前不等负载 ≤ 24，预注册 §3.5）
# 预注册：ui/geomem/docs/STAGE1_INTEG_PREREG.md 第 2 棒。容器按名字停，进程按 pid，不用 pkill -f。
set -u
WT=$(cd "$(dirname "$0")/../../../.." && pwd)
: "${CAR_STATE_DIR:=${FPGA_HOME}/dev1/.t/wde}" "${TB_CPUS:=24-35}" "${A10_SECS:=120}" "${A10_SETTLE:=10}"
NODE=${FPGA_HOME}/.nvm/versions/node/v20.20.0
CHROME_DIR=${FPGA_HOME}/dev1/tmp/chrome
IMAGE=fpga-dev1-sim:humble
ts() { date '+%Y-%m-%d %H:%M:%S'; }

car_env() {
  RUN=$(readlink "$CAR_STATE_DIR/car.current" 2>/dev/null) || { echo "没有车在跑" >&2; exit 2; }
  read -r GZ STACK HUB < <(python3 - "$RUN/car.json" <<'EOF'
import json, sys
j = json.load(open(sys.argv[1]))
c, h = j["containers"], j["hub"]
print(c["gz"], c["stack"], "%s:%s" % (h["bind"], h["port"]))
EOF
)
}

cmd_check() {
  local ok=0 la
  [ -e "$CAR_STATE_DIR/car.current" ] && { echo "car.current 存在 -> $(readlink "$CAR_STATE_DIR/car.current")"; ok=1; }
  docker ps --format '{{.Names}}' | grep -E '^wd-rtabloc-g4' && { echo "有 wd-rtabloc-g4* 容器"; ok=1; }
  docker ps --format '{{.Names}}' | grep -E '^wde-car-' && { echo "有车容器"; ok=1; }
  # 只认 argv[1] 是 run_full.sh 的进程（命令行里提到它的外壳不算，免得自己匹配自己）
  ps -eo pid=,args= | awk '$3 ~ /run_full\.sh$/ && $2 ~ /(^|\/)(ba)?sh$/' | grep . && { echo "有 run_full 在跑"; ok=1; }
  la=$(cut -d' ' -f1 /proc/loadavg)
  python3 -c "import sys; sys.exit(0 if float('$la') <= 24 else 1)" || { echo "1 分钟负载 $la > 24"; ok=1; }
  echo "$(ts) check: load1=$la $( [ $ok = 0 ] && echo OK || echo BLOCKED)"
  return $ok
}

tb_container() {   # tb_container <名> <输出目录> <命令...>：前台跑完
  local name=$1 out=$2; shift 2
  mkdir -p "$out"
  docker run --rm --name "geomem-t3-$name" --network "container:$GZ" --user "$(id -u):$(id -g)" --shm-size 1g \
    --cpuset-cpus "$TB_CPUS" -e HOME=/tmp -e GEOMEM_TB_NETNS=1 -e GEOMEM_WT="$WT" -e GEOMEM_PY=/usr/bin/python3 \
    -e GEOMEM_HUB="$HUB" -e PYTHONDONTWRITEBYTECODE=1 \
    -v "$WT:$WT:ro" -v "$CHROME_DIR:$CHROME_DIR:ro" -v "$NODE:$NODE:ro" -v "$TB_TOOLS_ROOT:$TB_TOOLS_ROOT:ro" \
    -v "$RUN:/o:ro" -v "$out:$out" --entrypoint bash "$IMAGE" -c "$*" > "$out/console.log" 2>&1
}

prep_tools() {
  : "${TB_TOOLS:?TB_TOOLS 须指向含 node_modules 的目录}"
  mkdir -p "$TB_TOOLS/integ"
  cp "$WT"/ui/geomem/tools/test/*.mjs "$TB_TOOLS/"
  cp "$WT"/ui/geomem/tools/test/integ/*.mjs "$TB_TOOLS/integ/"
  # node_modules 可能是软链到别处：把整个上两级（scratch 根）只读挂进去
  TB_TOOLS_ROOT=${TB_TOOLS_ROOT:-$(dirname "$(dirname "$TB_TOOLS")")}
}

cmd_bottle() {
  local i=$1 out=$2 x y req rc
  case $i in 1) x=1.25 y=0.25 ;; 2) x=1.25 y=-0.90 ;; 3) x=1.9 y=-1.3 ;; 4) x=1.9 y=1.3 ;; 5) x=1.6 y=-1.1 ;; *) echo "位置 1..5" >&2; return 2 ;; esac
  req=$(python3 - "$WT/robot_sim/fpga_sim/worlds/room_carmount_d2.sdf" "$i" "$x" "$y" <<'EOF'
import re, sys
src = open(sys.argv[1], encoding="utf-8").read()
m = re.search(r'<model name="bottle_prop">.*?</model>', src, re.S)
model = m.group(0).replace('name="bottle_prop"', 'name="bottle_prop_%s"' % (int(sys.argv[2]) + 1))
model = re.sub(r'<pose>1\.2500 -0\.4500 0 0 0 0\.0000</pose>', '<pose>%s %s 0 0 0 0</pose>' % (sys.argv[3], sys.argv[4]), model, count=1)
assert "<pose>%s %s 0" % (sys.argv[3], sys.argv[4]) in model
sdf = "<sdf version='1.8'>" + model.replace('"', "'") + "</sdf>"
print('sdf: "%s"' % sdf)
EOF
) || return 1
  docker exec "$GZ" bash -c 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; ign service -s /world/fpga_room/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 5000 --req "$0"' "$req" \
    > "$out/bottle_$i.ign.txt" 2>&1; rc=$?
  echo "$(ts) P$i ($x,$y) ign rc=$rc $(tr '\n' ' ' < "$out/bottle_$i.ign.txt")" | tee "$out/bottle_$i.done"
}

cmd_remove() {
  local i=$1 out=$2 rc
  docker exec "$GZ" bash -c 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; ign service -s /world/fpga_room/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 5000 --req "name: \"bottle_prop_$0\" type: MODEL"' "$((i + 1))" \
    > "$out/remove_$i.ign.txt" 2>&1; rc=$?
  echo "$(ts) remove bottle_prop_$((i + 1)) ign rc=$rc $(tr '\n' ' ' < "$out/remove_$i.ign.txt")" | tee "$out/remove_$i.done"
}

cmd_run() {
  local name=$1 out=$2 script=$3; shift 3
  prep_tools; mkdir -p "$out"
  ( tb_container "$name" "$out" "cd '$TB_TOOLS/integ' && exec '$NODE/bin/node' '$script' $*" ) & local pid=$!
  # 替脚本放瓶：脚本写 need_bottle_<i>，这里放好后写 bottle_<i>.done
  while kill -0 "$pid" 2>/dev/null; do
    for i in 1 2 3 4 5; do
      if [ -e "$out/need_bottle_$i" ] && [ ! -e "$out/bottle_$i.done" ]; then cmd_bottle "$i" "$out"; fi
      if [ -e "$out/need_remove_$i" ] && [ ! -e "$out/remove_$i.done" ]; then cmd_remove "$i" "$out"; fi
    done
    sleep 1
  done
  wait "$pid"; echo "$(ts) run $name rc=$?"
}

stat_ticks() { awk '{print $14+$15}' "/proc/$1/stat"; }
hub_pid() { docker top "$STACK" -eo pid,args | awk '/host\.hub\.hub_ros/ && !/awk/ {print $1; exit}'; }

cmd_a10() {
  local out=$1 arm k pid0 pid t0 t1 c0 c1 la0 la1 tck name w=0
  prep_tools; mkdir -p "$out"
  tck=$(getconf CLK_TCK); pid0=$(hub_pid)
  [ -n "$pid0" ] || { echo "找不到 hub 进程" >&2; return 1; }
  echo "$(ts) A10 hub pid $pid0 CLK_TCK $tck secs $A10_SECS settle $A10_SETTLE" | tee "$out/a10.log"
  echo '{"window":0}' > /dev/null
  : > "$out/a10.jsonl"
  for arm in none bridge fpgapc none bridge fpgapc; do
    w=$((w + 1)); name="a10-$w-$arm"
    [ "${A10_LOADWAIT:-1}" = 1 ] && for _ in $(seq 60); do python3 -c "import sys; sys.exit(0 if float(open('/proc/loadavg').read().split()[0]) <= 24 else 1)" && break; echo "$(ts) w$w load $(cut -d' ' -f1 /proc/loadavg) > 24, wait" | tee -a "$out/a10.log"; sleep 10; done
    case $arm in
      bridge) ( tb_container "$name" "$out/w$w" "cd '$TB_TOOLS/integ' && exec '$NODE/bin/node' t3_a10_bridge.mjs '$out/w$w' /o/token.txt $((A10_SECS + A10_SETTLE + 30))" ) &
              for _ in $(seq 120); do grep -q 'A10 bridge up' "$out/w$w/console.log" 2>/dev/null && break; sleep 0.5; done ;;
      fpgapc)  ( tb_container "$name" "$out/w$w" "cd '$WT' && exec python3 pc/fpgapc.py --host '${HUB%:*}' --port '${HUB##*:}' --token-file /o/token.txt listen --duration-s $((A10_SECS + A10_SETTLE + 30))" ) &
              for _ in $(seq 120); do grep -q '^LINK up' "$out/w$w/console.log" 2>/dev/null && break; sleep 0.5; done ;;
      none)   mkdir -p "$out/w$w" ;;
    esac
    up=$(grep -m1 -E 'A10 bridge up|^LINK up' "$out/w$w/console.log" 2>/dev/null | cut -c1-200)
    sleep "$A10_SETTLE"
    pid=$(hub_pid); la0=$(cut -d' ' -f1-3 /proc/loadavg); t0=$(date +%s.%N); c0=$(stat_ticks "$pid0")
    sleep "$A10_SECS"
    c1=$(stat_ticks "$pid0"); t1=$(date +%s.%N); la1=$(cut -d' ' -f1-3 /proc/loadavg)
    [ "$arm" != none ] && { docker stop -t 10 "geomem-t3-$name" > /dev/null 2>&1; wait; }
    python3 - "$w" "$arm" "$pid0" "$pid" "$c0" "$c1" "$t0" "$t1" "$tck" "$la0" "$la1" "$up" <<'EOF' | tee -a "$out/a10.jsonl"
import json, sys
w, arm, pid0, pid, c0, c1, t0, t1, tck, la0, la1, up = sys.argv[1:]
dt = float(t1) - float(t0)
print(json.dumps({"window": int(w), "arm": arm, "hub_pid": int(pid0), "same_pid": pid == pid0,
                  "cpu_pct_one_core": round((int(float(c1)) - int(float(c0))) / int(tck) / dt * 100, 3),
                  "ticks": int(float(c1)) - int(float(c0)), "wall_s": round(dt, 3), "load_start": la0, "load_end": la1,
                  "client_up": up}, ensure_ascii=False))
EOF
    sleep 5
  done
  python3 - "$out/a10.jsonl" <<'EOF' | tee "$out/a10_verdict.json"
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
by = {a: [r["cpu_pct_one_core"] for r in rows if r["arm"] == a] for a in ("none", "bridge", "fpgapc")}
mean = {a: sum(v) / len(v) for a, v in by.items()}
same = all(r["same_pid"] for r in rows) and len({r["hub_pid"] for r in rows}) == 1
clients_up = all(r["client_up"] for r in rows if r["arm"] != "none")
res = abs(by["none"][0] - by["none"][1])
diff = abs(mean["bridge"] - mean["fpgapc"])
if not same or not clients_up:
    v = "TESTBED-INVALID"
elif res > 1.0:
    v = "UNRESOLVABLE"
elif not (mean["none"] < mean["bridge"] and mean["none"] < mean["fpgapc"]):
    v = "TESTBED-INVALID"
else:
    v = "PASS" if diff <= 1.0 else "FAIL"
print(json.dumps({"by_arm": by, "mean": {k: round(x, 3) for k, x in mean.items()}, "none_spread": round(res, 3),
                  "bridge_minus_fpgapc": round(mean["bridge"] - mean["fpgapc"], 3), "same_pid": same,
                  "clients_up": clients_up, "verdict": v}, ensure_ascii=False))
EOF
}

case ${1:-} in
  check) cmd_check ;;
  run) car_env; shift; cmd_run "$@" ;;
  bottle) car_env; shift; cmd_bottle "$@" ;;
  remove) car_env; shift; cmd_remove "$@" ;;
  a10) car_env; shift; cmd_a10 "$@" ;;
  *) sed -n '2,14p' "$0"; exit 2 ;;
esac
