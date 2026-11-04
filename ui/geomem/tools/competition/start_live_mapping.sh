#!/usr/bin/env bash
# On Nano: attach a read-only display feed to the current car stack.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
current=/data/fpga/car/state/car.current
test -f "$current/READY"
run_dir=$(readlink -f "$current")
run=$(basename "$run_dir")
container="wde-car-stack-$run"
if ! test -s "$current/mapping_status.json"; then
  echo '本趟未启用建图，使用车栈已有地图显示。'; exit 0
fi
state=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["state"])' "$current/mapping_status.json")
if test "$state" != mapping; then
  echo '当前没有运行中的 RTAB-Map 建图，保留已加载底图。'; exit 0
fi
scope=$(python3 "$here/wait_runtime_info.py" --current "$current" --run-dir "$run_dir")
out="/data/fpga/rtabmap/operator-live/$run"
# 10-22: a feed started from an older live_mapping.py is restarted (the copy below overwrites the file on disk,
# so the running version is recorded next to its pid)
# 10-30: + map_data_gate.py (the feed imports its graph rule; it also runs as the assembler's /mapData gate)
feed_sha=$(cat "$here/live_mapping.py" "$here/map_data_gate.py" | sha256sum | cut -c1-16)
docker exec "$container" mkdir -p "$out"
for file in live_mapping.py live_mapping.yaml map_data_gate.py; do docker cp "$here/$file" "$container:$out/$file"; done
docker exec -i "$container" bash -s -- "$out" "$run" "$scope" "$feed_sha" <<'INNER'
set -euo pipefail
out=$1; run=$2; scope=$3; feed_sha=$4
set +u
source /opt/ros/humble/setup.bash
set -u
running() {
  test -f "$out/$1.pid" || return 1
  local pid
  pid=$(cat "$out/$1.pid")
  test -r "/proc/$pid/cmdline" && tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Fq "$2"
}
# 10-30 graph gate: the assembler takes /mapData through map_data_gate.py (forwarded only when the map graph changed,
# >= every 10 s; a parked car no longer makes it re-assemble the whole cloud every second). An assembler of an older
# start (subscribed to /mapData itself) is replaced; the new one loads the current map from rtabmap's get_map_data.
gated=/operator_map/mapData_gated
if test -f "$out/assembler.pid" && running assembler "$out/live_mapping.yaml" && ! running assembler "mapData:=$gated"; then
  pid=$(cat "$out/assembler.pid"); kill "$pid"
  for _ in $(seq 20); do test -d "/proc/$pid" || break; sleep .25; done
  echo "ungated map_assembler replaced"
fi
if running gate "$out/map_data_gate.py" && [ "$(cat "$out/gate.sha" 2>/dev/null)" != "$feed_sha" ]; then
  pid=$(cat "$out/gate.pid"); kill "$pid"
  for _ in $(seq 20); do test -d "/proc/$pid" || break; sleep .25; done
  echo "gate restarted for $feed_sha"
fi
if ! running gate "$out/map_data_gate.py"; then
  nohup python3 "$out/map_data_gate.py" --in-topic /mapData --out-topic "$gated" >"$out/gate.log" 2>&1 </dev/null &
  echo $! >"$out/gate.pid"
  echo "$feed_sha" >"$out/gate.sha"
fi
if ! running assembler "mapData:=$gated"; then
  nohup /opt/ros/humble/lib/rtabmap_util/map_assembler --ros-args -r __ns:=/operator_map -r "mapData:=$gated" -r /operator_map/rtabmap/get_map_data:=/rtabmap/get_map_data --params-file "$out/live_mapping.yaml" >"$out/assembler.log" 2>&1 </dev/null &
  echo $! >"$out/assembler.pid"
fi
if running feed "$out/live_mapping.py" && [ "$(cat "$out/feed.sha" 2>/dev/null)" != "$feed_sha" ]; then
  pid=$(cat "$out/feed.pid"); kill "$pid"
  for _ in $(seq 20); do test -d "/proc/$pid" || break; sleep .25; done
  echo "feed restarted for live_mapping.py $feed_sha"
fi
if ! running feed "$out/live_mapping.py"; then
  nohup python3 "$out/live_mapping.py" --session "$run" --scope "$scope" >"$out/feed.log" 2>&1 </dev/null &
  echo $! >"$out/feed.pid"
  echo "$feed_sha" >"$out/feed.sha"
fi
INNER
python3 - "$run" "$scope" <<'CHECK'
import json, sys, time, urllib.request
for attempt in range(15):
    try:
        with urllib.request.urlopen('http://127.0.0.1:47802/status', timeout=1) as response:
            data = json.load(response)
        # 10-26: after a 重置地图 the feed follows /fpga/runtime_info (scopeSource topic), newer than the file
        if data.get('session') == sys.argv[1] and (data.get('scope') == sys.argv[2] or data.get('scopeSource') == 'topic'):
            break
    except (OSError, ValueError):
        pass
    time.sleep(.5)
else:
    raise SystemExit('三维显示通道未就绪；检查 operator-live 当前会话 feed.log。')
CHECK
echo "RTAB-Map 只读三维通道：$run；Nano 回环 47802。"
