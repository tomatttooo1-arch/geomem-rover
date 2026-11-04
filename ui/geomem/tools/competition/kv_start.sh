#!/usr/bin/env bash
# Invoked on the KV260 through Nano SSH (root, by kv_service.py), supervised and before the car stack starts.
#   kv_start.sh <repo> <out> [drm|off] [v3fc2|v3fc2_166|v3fc3_187|v3fc|a2|m1_166|m1|m2_166|m2]   (the two optional words in any order)
# FPGA profile (go-live 2026-10-26, tools/car_run/fpga_profiles.env): absent = a2, which is the command exactly as
#   before (m7_pl_merge_a2, no --rolling); another profile loads its app and serves --rolling <its KV rolling>; the
#   AR start's BUILD_ID gate is the profile's. A profile whose app is not installed starts a2 instead (warning).
# drm (10-26, D-042 car screen): kv_server with the AR core on the 7" screen, colour main view fed by the Nano's
#   nano_colour.py on UDP 47801, the live tune file of this snapshot (kv_ar_tune.json), BUILD_ID gate of the profile.
#   The screen is display only and must never keep the car from starting: if that start fails (no helper binary,
#   no screen / DRM error, helper not ready, wrong bitstream ...) the leftover server is stopped and the server is
#   started again as before 10-26 (no AR, same profile) in <out>-ar-off, with a loud "AR_DISABLED reason=..." line.
# off: the pre-10-26 command line (plus the profile's --rolling).
# AR2NANO (nationals 2026-11-02, owner: the car screen is drawn on the Jetson Orin Nano): when the loaded bitstream
#   has no AR core (sysinfo SLOT_MAP bit 8 clear, e.g. 0x263) a drm request is skipped quietly: no start attempt, no
#   desktop toggle, no AR_DISABLED; the server starts as with off and ar_status.json says reason "ar_core_absent ...".
#   An unreadable SLOT_MAP changes nothing (the drm attempt and its fallback run exactly as before).
# SW_M1 (2026-11-03): a profile marked <P>_AR=nano in fpga_profiles.env (m1_166 / m1: no AR core in the bitstream) takes
#   the same quiet path even when the SLOT_MAP cannot be read (reason "ar_core_absent: profile ..."); every other
#   profile (FPGA_AR=kv) runs exactly as before.
# Either way <run>/ar_status.json = {"ar": "drm"|"off", "requested": ..., "reason": ..., "mode": ...}.
set -euo pipefail
repo=${1:?KV deployment repository required}
out=${2:?new KV run directory required}
ar=off profile=a2
for w in "${@:3}"; do
  case "$w" in
    drm|off) ar=$w ;;
    v3fc2|v3fc2_166|v3fc3_187|v3fc|a2|m1_166|m1|m2_166|m2) profile=$w ;;
    *) echo "AR mode must be drm or off, FPGA profile v3fc2|v3fc2_166|v3fc3_187|v3fc|a2|m1_166|m1|m2_166|m2 (got '$w')" >&2; exit 2;;
  esac
done
[[ $repo = /* && $out = /* && $repo != / && $out != / ]] || exit 2
test -f "$repo/tools/kv_link/kv_board.sh"
if [ -f "$repo/tools/car_run/fpga_profile.sh" ]; then
  source "$repo/tools/car_run/fpga_profile.sh"
  fpga_profile "$profile" || exit 2
else   # a snapshot without the profile file: a2 only (as before 10-26)
  [ "$profile" = a2 ] || echo "WARNING: $repo/tools/car_run/fpga_profile.sh missing; profile $profile -> a2" >&2
  FPGA_PROFILE=a2 FPGA_APP=m7_pl_merge_a2 FPGA_BUILD_ID=0x00da5041 FPGA_KV_ROLLING=none
fi
if pgrep -f '[k]v_server.py' >/dev/null; then
  echo 'A KV server is already running; inspect its ownership and use the existing service or stop it explicitly.' >&2
  exit 3
fi
test ! -e "$out" || { echo 'Choose a new kv_run directory; preserving the previous run.' >&2; exit 4; }
test ! -e "$out-ar-off" || { echo 'Choose a new kv_run directory; preserving the previous run.' >&2; exit 4; }
fw=${FPGA_FW_DIR:-/lib/firmware/xilinx}   # tests only (sudo drops the environment)
if [ ! -d "$fw/$FPGA_APP" ] && [ "$FPGA_PROFILE" != a2 ]; then
  # owner 10-26 (no new refusals): start the a2 bitstream instead; the Nano precheck accepts any registered
  # build and then fuses on its CPU
  echo "WARNING: FPGA app $FPGA_APP is not installed in $fw (profile $profile); starting a2 instead" \
       "(install it with install_kv_service.sh --replace ... --app-dir)." >&2
  fpga_profile a2
fi
roll=()
[ "$FPGA_KV_ROLLING" = none ] || roll=(--rolling "$FPGA_KV_ROLLING")
# SMOKE1 (2026-11-03): the load is judged by the BUILD_ID read back from the loaded bitstream's sysinfo block, not by
#   xmutil's exit code or listapps. BOARD3 (MERGE2 board test) saw "xmutil loadapp" print "load Error: -1" with exit
#   code 0 while the PL still held the previous bitstream (dfx-mgrd was still parsing a just-copied app directory; a
#   second load ~20 s later worked). Readback different from the profile's BUILD_ID (or unreadable) => wait, unload,
#   load again, up to GEOMEM_KV_LOAD_TRIES (3) loads in all, GEOMEM_KV_LOAD_WAIT_S (10 s) apart. Still different =>
#   a loud "FPGA_LOAD_MISMATCH" warning and the start goes on as before (owner 10-26: warn, never a new refusal; the
#   Nano precheck then uses whatever registered build really runs). A non-zero xmutil exit is logged, no longer fatal.
#   Same-BUILD_ID profiles (v3fc2 / v3fc2_166, m2 / m2_166) are not told apart by this; the pl1_ref clock is printed
#   on the FPGA_LOAD line for that. GEOMEM_KV_BUILD_ID_CMD (tests only, sudo drops the environment): a shell line
#   eval'ed instead of the sysinfo read; it prints the read-back BUILD_ID.
build_id_now() {   # the loaded bitstream's sysinfo BUILD_ID as 0x..., empty when unreadable
  if [ -n "${GEOMEM_KV_BUILD_ID_CMD:-}" ]; then eval "$GEOMEM_KV_BUILD_ID_CMD" 2>/dev/null || true; return 0; fi
  PYTHONPATH=$repo timeout 10 python3 - <<'PY' 2>/dev/null || true
from host.kv_regs import PlRegBlock
b = PlRegBlock.open("sysinfo")
if b.available:
    print("%#010x" % b.sysinfo()["build_id"])
PY
}
build_id_is() {   # <read> <want>: same number
  [[ $1 =~ ^0x[0-9a-fA-F]+$ && $2 =~ ^0x[0-9a-fA-F]+$ ]] && (( $1 == $2 ))
}
load_tries=${GEOMEM_KV_LOAD_TRIES:-3} load_wait=${GEOMEM_KV_LOAD_WAIT_S:-10}
load_try=0 got=""
while :; do
  load_try=$((load_try + 1))
  xmutil unloadapp || echo "WARNING: xmutil unloadapp rc=$? (load $load_try)" >&2
  xmutil loadapp "$FPGA_APP" || echo "WARNING: xmutil loadapp $FPGA_APP rc=$? (load $load_try)" >&2
  for _r in 1 2 3; do   # the sysinfo node can take a moment to appear after the overlay
    got=$(build_id_now | tail -1)
    build_id_is "$got" "$FPGA_BUILD_ID" && break
    [ "$_r" = 3 ] || sleep "${GEOMEM_KV_READ_WAIT_S:-1}"
  done
  build_id_is "$got" "$FPGA_BUILD_ID" && break
  echo "WARNING: after load $load_try of $FPGA_APP the PL reads BUILD_ID ${got:-unreadable}, not $FPGA_BUILD_ID" >&2
  [ "$load_try" -lt "$load_tries" ] || break
  sleep "$load_wait"
done
pl1=$(cat /sys/kernel/debug/clk/pl1_ref/clk_rate 2>/dev/null || echo unknown)
if build_id_is "$got" "$FPGA_BUILD_ID"; then
  echo "FPGA_LOAD ok app=$FPGA_APP build_id=$got loads=$load_try pl1_ref_hz=$pl1"
  # DRV5 (m5): S_AXI_LPD prerequisite (clears RST_LPD_TOP bit 19 only on a bitstream with LPD masters) + AFI widths;
  # warnings only, never stops the start
  lpd=$repo/hw/m7/pl/tools/m5_lpd_afi.py
  [ -f "$lpd" ] && timeout 10 python3 "$lpd" 2>&1 || true
else
  echo "FPGA_LOAD_MISMATCH app=$FPGA_APP expected=$FPGA_BUILD_ID read=${got:-unreadable} loads=$load_try" \
       "pl1_ref_hz=$pl1: the bitstream did not load; the KV server starts on whatever the PL holds" \
       "(retry by hand: sudo /usr/local/sbin/geomem-kv-start stop, then start again)" | tee /dev/stderr
fi
comp=$repo/ui/geomem/tools/competition
board=$repo/tools/kv_link/kv_board.sh
helper=$repo/tools/kv_link/ar_helper/build/aarch64/m7ar_present
# INT_SW (2026-11-02) next-bitstream switches: KEY=VALUE lines in $comp/kv_nat_switches.conf of this snapshot
# (sudo drops the environment). File absent or every switch 0/none = the command line exactly as before.
#   KV_FRAME_DLAT2=1      --dlat2 1           G1d ray thinning (a bitstream without the register: warning only)
#   KV_ROI_CONCURRENT=1   --roi-concurrent 1  ROI core started together with the geometry core
#                         + --roi-wait irq    (SW_M1 2026-11-03) the ROI completion by interrupt: the polled wait held
#                                             the interpreter lock and ate the overlap (BOARD2 3.5); KV_ROI_WAIT=poll
#                                             = the SW_M1-before line (--roi-concurrent 1 alone)
#   KV_ROLLING_GRID=pl    --rolling-grid pl   fused-core op GRID (only with a profile serving --rolling pl;
#                                             a bitstream without GRID is probed and served as before)
nat=()
natconf=$comp/kv_nat_switches.conf
natv() { sed -n "s/^$1=\([^ #]*\).*/\1/p" "$natconf" | tail -1; }
if [ -f "$natconf" ]; then
  if [ "$(natv KV_FRAME_DLAT2)" = 1 ]; then nat+=(--dlat2 1); fi
  if [ "$(natv KV_ROI_CONCURRENT)" = 1 ]; then
    nat+=(--roi-concurrent 1)
    [ "$(natv KV_ROI_WAIT)" = poll ] || nat+=(--roi-wait irq)
  fi
  if [ "$(natv KV_ROLLING_GRID)" = pl ]; then
    if [ "$FPGA_KV_ROLLING" = pl ]; then nat+=(--rolling-grid pl)
    else echo "WARNING: KV_ROLLING_GRID=pl ignored: profile $FPGA_PROFILE serves --rolling $FPGA_KV_ROLLING" >&2; fi
  fi
  echo "KV_NAT_SWITCHES ${nat[*]-none}" >&2
fi
# E7 2026-11-02: C++ data plane only when fpga_profiles.env FPGA_KV_FAST_DEFAULT=native (default off: command as before)
fast=()
if [ "${FPGA_KV_FAST:-off}" = native ]; then fast=(--kv-fast native); fi
base=(--core pl --device devmem --wait irq --roi-core pl --port 47800 ${roll[@]+"${roll[@]}"} ${nat[@]+"${nat[@]}"} ${fast[@]+"${fast[@]}"})

slot_map_now() {   # sysinfo SLOT_MAP of the loaded bitstream as 0x..., empty when unreadable (GEOMEM_KV_SLOT_MAP: tests)
  if [ -n "${GEOMEM_KV_SLOT_MAP:-}" ]; then echo "$GEOMEM_KV_SLOT_MAP"; return 0; fi
  PYTHONPATH=$repo timeout 10 python3 - <<'PY' 2>/dev/null || true
from host.kv_regs import PlRegBlock
b = PlRegBlock.open("sysinfo")
if b.available:
    print("%#x" % b.sysinfo()["slot_map"])
PY
}

ar_status() {   # <run> <ar> <reason> <mode> <requested>; a failed write only warns (display bookkeeping)
  python3 - "$@" <<'PY' || echo "WARNING: could not write $1/ar_status.json" >&2
import json, os, sys
run, ar, reason, mode = sys.argv[1:5]
st = {"ar": ar, "requested": sys.argv[5], "reason": reason or None, "mode": mode or None}
if os.environ.get("GEOMEM_KV_SCREEN"):   # 10-27: "sink_count=N edid_bytes=M" (drm requests only)
    st["screen"] = os.environ["GEOMEM_KV_SCREEN"]
with open(run + "/ar_status.json", "w") as fp:
    json.dump(st, fp)
    fp.write("\n")
PY
}

stop_leftover() {   # <run>: the attempt's server, only if that pid really is a kv_server; then its helper
  local run=$1 pid i
  pid=$(cat "$run/pid" 2>/dev/null || true)
  if [[ $pid =~ ^[0-9]+$ ]] && tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q 'kv_link/kv_server.py'; then
    kill -TERM "$pid" 2>/dev/null || true
    for i in $(seq 1 50); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  fi
  # a SIGKILLed server leaves its helper holding the DRM master (it keeps flipping STALE until SIGTERM)
  pkill -TERM -f "$helper" 2>/dev/null || true
}

if [ "$ar" = drm ]; then
  sm=$(slot_map_now)
  if [[ $sm =~ ^0x[0-9a-fA-F]+$ ]] && (( sm & 1 )) && (( !(sm >> 8 & 1) )); then
    echo "AR_ON_NANO slot_map=$sm: this bitstream has no AR core; the car screen is drawn by the Nano (KV starts without AR)"
    bash "$board" start "$repo" "$out" "${base[@]}"
    ar_status "$out" off "ar_core_absent: SLOT_MAP $sm (car screen on the Nano)" "" drm
    exit 0
  fi
  if [ "${FPGA_AR:-kv}" = nano ]; then   # SW_M1: the profile says its bitstream has no AR core
    echo "AR_ON_NANO profile=$FPGA_PROFILE (slot_map=${sm:-unreadable}): this bitstream has no AR core; the car screen is drawn by the Nano (KV starts without AR)"
    bash "$board" start "$repo" "$out" "${base[@]}"
    ar_status "$out" off "ar_core_absent: profile $FPGA_PROFILE has no AR core (car screen on the Nano)" "" drm
    exit 0
  fi
fi
if [ "$ar" = drm ]; then
  reason="" mode=""
  drm=${GEOMEM_KV_DRM_SYSFS:-/sys/class/drm/card0-DP-1}
  # 10-27 screen link check (warning only, never stops the start): the DP->HDMI converter reports SINK_COUNT=0 and
  # no EDID when the screen had no power / HDMI was not seated when the KV booted; the helper then flips normally
  # (ar_status "drm", events LIVE+flipped) while the panel stays dark (AR_SCREEN_BRINGUP_20261015 sec. 1).
  # One forced re-probe first (cheap), then the sink count is logged and stored in ar_status.json.
  if [ ! -s "$drm/edid" ] && [ -e "$drm/status" ]; then echo detect > "$drm/status" 2>/dev/null || true; sleep 1; fi
  aux=${GEOMEM_KV_DP_AUX:-/dev/drm_dp_aux0}
  sink=$(python3 -c 'import os,sys;f=os.open(sys.argv[1],os.O_RDONLY);print(os.pread(f,1,0x200)[0]&0x3f)' "$aux" 2>/dev/null || true)
  edid_b=$(wc -c < "$drm/edid" 2>/dev/null || echo 0)
  export GEOMEM_KV_SCREEN="sink_count=${sink:-unknown} edid_bytes=$edid_b"
  if [ "$sink" = 0 ] || [ "$edid_b" = 0 ]; then
    echo "AR_SCREEN_WARNING $GEOMEM_KV_SCREEN: the converter sees no screen; AR will run but the panel likely stays" \
         "dark -- power the screen, then unplug/replug its HDMI at the KV (no KV restart needed), then restart the KV service" | tee /dev/stderr
  fi
  if [ -s "$drm/edid" ]; then
    marg=(--ar-mode 1024x600); mode=1024x600
  else   # converter without EDID (AR_SCREEN_BRINGUP / ar_screen.sh 1024x600_edid: the panel's own EDID timing, forced)
    marg=(--ar-modeline "49000 1024 1072 1168 1312 600 603 613 624 -hsync -vsync"); mode=1024x600_edid
  fi
  if [ ! -x "$helper" ]; then
    reason="helper_missing: $helper (deploy the KV with tools/car_run/deploy_kv.sh, it cross-builds it)"
  elif [ ! -f "$comp/kv_ar_tune.json" ]; then
    reason="tune_missing: $comp/kv_ar_tune.json"
  else
    # as tools/kv_link/ar_screen.sh start <out> <mode> live --ar-colour main --ar-colour-port 47801 (the live runs of
    # AR_COLOUR_MAIN_20261016 sections 14-18), minus dumps and send log; --max-idle-s stays the competition default
    # 10-27: --ar-colour-tol-ms 17 (half a 30 fps period = the nearest colour frame): on the car the Gemini 2 depth
    # and colour stamps now sit a constant 11.9 ms apart, over the old 10 ms, so no colour ever paired (display only)
    # 10-29: 17 is the 30 fps value; the Nano's colour sender now says its camera rate in the HELLO (car_up
    # CAR_CAMERA_FPS, default 15) and the KV widens the pairing to half that period plus 5 ms (host/kv_colour.py
    # pair_tol_ns: 15 fps -> 39 ms). This start is pinned by sudoers and outlives car stacks, so the rate cannot come in here.
    # 10-27: --ar-max-age-ms 1000 (fail-safe 1001 ms): the car's frame age is ~143 ms median and ~10% of frames passed
    # the old 200 ms, each one flashing the purple STALE checker; now only a real stall over 1 s shows it (display only)
    set +e
    env FPGA_M7_DMA_PROVIDER=dmaheap FPGA_M7_DMA_HEAP=reserved bash "$board" start "$repo" "$out" "${base[@]}" \
      --core-timeout-s 3 --ar drm --ar-source live "${marg[@]}" --ar-tune-file "$comp/kv_ar_tune.json" \
      --ar-colour main --ar-colour-port 47801 --ar-colour-tol-ms 17 --ar-max-age-ms 1000 --expect-build-id "$FPGA_BUILD_ID"
    rc=$?
    set -e
    if [ "$rc" = 0 ]; then
      ar_status "$out" drm "" "$mode" drm
      echo "AR_ENABLED mode=$mode colour_port=47801 run=$out"
      exit 0
    fi
    why=$(grep -h -E 'Not starting|REFUSED|Error|error' "$out/server.err" 2>/dev/null | tail -1 || true)
    [ -n "$why" ] || why=$(tail -1 "$out/server.err" 2>/dev/null || true)
    reason="ar_start_failed rc=$rc: ${why:-no server.err}"
    stop_leftover "$out"
  fi
  reason=$(printf '%s' "$reason" | tr '\n\r' '  ' | cut -c1-300)
  echo "AR_DISABLED reason=$reason" | tee /dev/stderr
  echo "AR_DISABLED: the car screen stays dark; starting the KV service without AR (geometry unchanged)." >&2
  off=$out-ar-off
  bash "$board" start "$repo" "$off" "${base[@]}"
  ar_status "$off" off "$reason" "$mode" drm
  exit 0
fi
bash "$board" start "$repo" "$out" "${base[@]}"
ar_status "$out" off "switched off (kv_ar.conf or /etc/geomem-dev3-kv/ar.conf)" "" off
# Nano car_up separately verifies BUILD_ID (the profile's, tools/car_run/fpga_profiles.env) before starting the stack.
