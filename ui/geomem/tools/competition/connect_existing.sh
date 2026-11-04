#!/usr/bin/env bash
# Lab connection only: attach to an existing Nano stack. Never starts motors/camera/stack.
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../../.." && pwd)
out="$repo/runs/dev3-competition-connection"
mkdir -p "$out"
umask 077
ssh -o BatchMode=yes jetson 'mkdir -p /home/robot/fpga/dev3/operator-feed'
scp -q "$repo/ui/geomem/tools/competition/check_current.py" jetson:/home/robot/fpga/dev3/operator-feed/
ssh -o BatchMode=yes jetson 'python3 /home/robot/fpga/dev3/operator-feed/check_current.py --hub-port 47801' || {
  echo 'Nano 车栈尚未 READY；保持原网站只读相机，未启动底盘。' >&2; exit 3;
}
# Copy the current session credential without printing it. No old-session token reuse.
scp -q jetson:/data/fpga/car/state/car.current/token.txt "$out/token.next"
if test -f "$out/token.txt" && ! cmp -s "$out/token.txt" "$out/token.next"; then
  echo '车栈会话已更换，请先结束本次 bridge，再重新连接。' >&2
  rm -f "$out/token.next"; exit 4
fi
mv "$out/token.next" "$out/token.txt"
# Reuse RTAB-Map's optimized RGB-D map; no camera or car-stack restart.
scp -q "$repo/ui/geomem/tools/competition/"{live_mapping.py,live_mapping.yaml,map_data_gate.py,start_live_mapping.sh,wait_runtime_info.py} jetson:/home/robot/fpga/dev3/operator-feed/
ssh nano 'bash /home/robot/fpga/dev3/operator-feed/start_live_mapping.sh'
if ! test -f "$out/mapping-tunnel.pid" || ! kill -0 "$(cat "$out/mapping-tunnel.pid")" 2>/dev/null; then
  nohup ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -L 127.0.0.1:28792:127.0.0.1:47802 jetson >"$out/mapping-tunnel.log" 2>&1 </dev/null &
  echo $! >"$out/mapping-tunnel.pid"
  sleep 1
  kill -0 "$(cat "$out/mapping-tunnel.pid")" || exit 7
fi
if ! test -f "$out/hub-tunnel.pid" || ! kill -0 "$(cat "$out/hub-tunnel.pid")" 2>/dev/null; then
  nohup ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -L 127.0.0.1:28791:127.0.0.1:47801 jetson >"$out/hub-tunnel.log" 2>&1 </dev/null &
  echo $! >"$out/hub-tunnel.pid"
  sleep 1
  kill -0 "$(cat "$out/hub-tunnel.pid")" || exit 5
fi
if ! test -f "$out/bridge.pid" || ! kill -0 "$(cat "$out/bridge.pid")" 2>/dev/null; then
  mkdir -p "$out/basemaps"
  nohup python3 "$repo/ui/geomem/bridge/geomem_bridge.py" --hub 127.0.0.1:28791 --token-file "$out/token.txt" --listen 127.0.0.1:8790 --basemap-watch "$out/basemaps" --stats-out "$out/bridge-stats.json" >"$out/bridge.log" 2>&1 </dev/null &
  echo $! >"$out/bridge.pid"
  sleep 1
  kill -0 "$(cat "$out/bridge.pid")" || exit 6
fi
echo '现有车栈转发桥已启动：服务器回环 8790；没有发送行驶命令。'
