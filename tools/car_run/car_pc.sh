#!/bin/bash
# Line E (2026-10-14): a PC command (pc/fpgapc.py) to the running car's hub, e.g.
#   bash tools/car_run/car_pc.sh find 瓶子 | select ID | cancel | estop | release --epoch E | yield | probe
#   | goto --forward-m 1.0 | goto --turn-deg 90   (10-17: one goal relative to the car, through the runner)
#   | teleop --v 0.1 --seconds 2 | teleop --keys   (10-18: remote control through the hub; with --nav nav2 the
#     arbitration mode: the first command cancels navigation, new find/select/goto are refused while in hand)
#   | mapping-stop   (10-18: CAR_MAPPING=1 -- save + export the in-stack map, Nav2 loads it)
#   (probe: is the hub reachable and is the token accepted -- the status query; fpgapc has no "status")
# It runs fpgapc in the stack container with the run's token (docker exec; nothing is started in the background) and
# connects to the address the hub of THIS run is bound to, read from the run (car_common.sh car_hub_addr: car.json
# "hub" {bind, port}, else marks.txt "ready hub ADDR:PORT"); nothing recorded -> exit 2, never a 127.0.0.1 guess.
# Host: the hub listens on 127.0.0.1 (or CAR_HUB_BIND, a 127/8 address: the stack's network namespace is
#   --network none, loopback only) inside the car's namespace, no host port.
# Board: the stack is --network host and the hub listens only on the car's LAN address (CAR_HUB_BIND, owner 10-14),
#   so the container reaches it there. A PC on another machine uses fpgapc.py --host <car IP> with a copy of the token.
# env: CAR_RUN (default: the car CAR_STATE_DIR/car.current points to)
#      CAR_HUB_HOST (override the recorded address)  HUBPORT (override the recorded port)
set -u
source "$(cd "$(dirname "$0")" && pwd)/car_common.sh"; car_default_dirs   # board /data/fpga/car/state, host /data1/...
: "${CAR_DOCKER:=docker}"
RUN=${CAR_RUN:-$(readlink "$CAR_STATE_DIR/car.current" 2>/dev/null)}
[ -n "$RUN" ] && [ -s "$RUN/car.json" ] || { echo "no car running" >&2; exit 2; }
STACK=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$RUN/car.json")
ADDR=$(car_hub_addr "$RUN") || { echo "car_pc: hub address of $RUN unknown: command NOT sent" >&2; exit 2; }
read -r HUB_HOST HUB_PORT <<< "$ADDR"
TTY=(-i); [ -t 0 ] && TTY=(-it)   # teleop --keys needs a terminal in the container
PY=pc/fpgapc.py   # teleop --keys reads the keyboard: pc/fpgapc_keys.py (fpgapc.py stays Windows-stdlib-only)
[ "${1:-}" = teleop ] && case " $* " in *" --keys "*) PY=pc/fpgapc_keys.py;; esac
exec $CAR_DOCKER exec "${TTY[@]}" -w /src "$STACK" python3 $PY --host "$HUB_HOST" --port "$HUB_PORT" --token-file /o/token.txt "$@"
