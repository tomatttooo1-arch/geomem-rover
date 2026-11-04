#!/bin/bash
# B1 car shadow run -- DRAFT (2026-11-02), NOT EXECUTED. Owner present; car + board locks taken by the main session.
# DESIGN 5.2 item 1 / NANO_SIDE.md: the producer's /fpga/rolling_map/grid (from the KV260 planes) and the bridge's
# /fpga/rolling_map/grid_sw (shadow, same deltas) side by side on one drive; Nav2 keeps using /grid.
#
# Before: the KV260 must announce "rolling_grid":
#   - until the fused PL grid op is on the board (hw/m7 next package): kv_server --rolling cpu --rolling-grid cmodel
#     (A53 core + the grid C model on a DUMP; NOT the PL path -- a protocol/Nano rehearsal only), or
#   - with the fused core: kv_server --rolling pl on the new bitstream (PlRollingCore.grid_capable then True).
#   The producer warns "no navigation-grid op" and falls back to the in-process bridge otherwise (status
#   grid_source nano_sw) -- check producer.log / the status row before driving.
# Usage (board Nano, repo root):  bash tools/b1_costmap/car_shadow_run.sh <map yaml> [minutes, default 10]
set -eu
MAP_YAML=${1:?map yaml (rtabmap export) required}
MIN=${2:-10}
cd "$(dirname "$0")/../.."
# 1. the car with the PL grid on and the bridge in shadow mode (CAR_PL_GRID=0 would be the car exactly as before)
CAR_PL_GRID=1 CAR_PL_GRID_SHADOW=1 CAR_MAP_YAML="$MAP_YAML" bash tools/car_run/car_up.sh live --pose odom --nav nav2
RUN=$(readlink "${CAR_STATE_DIR:-$HOME/.fpga_car}/car.current" 2>/dev/null || true)
echo "run dir: ${RUN:-unknown}"
# 2. the status row must say grid_source pl (else the KV has no grid op: stop here, nothing to compare)
CONT=${CAR_STACK_CONTAINER:-fpga-car-stack}
docker exec "$CONT" bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo --once /fpga/rolling_map/status' \
  | grep -q '"grid_source": "pl"' || { echo "status grid_source is not pl -- KV260 without the grid op?"; exit 3; }
# 3. the comparator, started BEFORE the drive (no new ROS participant while driving), for MIN minutes
docker exec -d "$CONT" bash -lc "source /opt/ros/humble/setup.bash && cd /src && \
  python3 tools/b1_costmap/shadow_compare.py --duration-s $((MIN * 60)) --out /o/b1_shadow.json > /o/b1_shadow.log 2>&1"
echo "drive now: still, straight, turns, one search; ${MIN} min. Then: cat \$RUN/b1_shadow.json; car_down.sh"
# 4. after: pidstat / py-spy evidence of the retired bridge is a separate step (DESIGN 5.1, owner to schedule)
