#!/bin/bash
# Line E (2026-10-14), TEST ONLY: the D2 user-level bottle scenario (robot_sim/tests/bottle_scenario.py:
# wait for the bottle in memory -> patrol away -> cancel -> "fpgapc find 找瓶子" through the hub -> found) against a
# car started with  car_up.sh sim --record ; run in the stack container (docker exec). Then car_down.sh, then
#   python3 tools/verify_bottle_run.py --layout sim RUN --out RUN/verify.json
# Extra arguments go to bottle_scenario.py unchanged (the D2 stop tests, verify with --layout sim-estop / sim-cut):
#   car_scenario.sh --estop-during-return-s 3 --estop-hold-s 3 --estop-release-after-s 1
#   CAR_FAILSTOP=log car_up.sh sim --record ...;  car_scenario.sh --cut-during-return-s 3 --cut-pid-file /o/producer.pid
#   (the cut test stops the producer: with the default fail-stop the whole stack would go down with it)
# env: CAR_RUN (default: the current car)  SCENARIO_TIMEOUT_S (420)
set -u
source "$(cd "$(dirname "$0")" && pwd)/car_common.sh"; car_default_dirs
: "${CAR_DOCKER:=docker}" "${SCENARIO_TIMEOUT_S:=420}"
RUN=${CAR_RUN:-$(readlink "$CAR_STATE_DIR/car.current" 2>/dev/null)}
[ -n "$RUN" ] && [ -s "$RUN/car.json" ] || { echo "no car running" >&2; exit 2; }
read -r STACK CAMERA RECORD DEVICE DETECTOR < <(python3 -c "
import json,sys;c=json.load(open(sys.argv[1]))
print(c['containers']['stack'],c['camera'],int(c['record']),c['inference_device'],c['detector'])" "$RUN/car.json")
[ "$CAMERA" = sim ] && [ "$RECORD" = 1 ] || { echo "needs a car started with: car_up.sh sim --record" >&2; exit 2; }
[ -e "$RUN/scenario.jsonl" ] && { echo "$RUN/scenario.jsonl exists: one scenario per car_up" >&2; exit 2; }
$CAR_DOCKER exec -w /src "$STACK" bash -c 'source /opt/ros/humble/setup.bash; source /opt/fpga_ws/install/setup.bash;
  export PYTHONPATH=/src:${PYTHONPATH:-} LANG=C.UTF-8 LC_ALL=C.UTF-8   # keep the ROS paths
  exec python3 robot_sim/tests/bottle_scenario.py --out /o/scenario.jsonl --timeout "$0" --expect-device "$1" \
    --query-via hub --hub-port 47431 --token-file /o/token.txt --expect-detector "$2" "${@:3}"' \
  "$SCENARIO_TIMEOUT_S" "$DEVICE" "$DETECTOR" "$@" > "$RUN/scenario.log" 2>&1
RC=$?
echo "scenario_rc=$RC" >> "$RUN/rc_scenario.txt"
exit $RC
