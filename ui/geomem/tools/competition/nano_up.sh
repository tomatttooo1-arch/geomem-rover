#!/usr/bin/env bash
# Nano/laptop share this startup path. No drive or navigation commands.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=${1:?Nano source repository required}
mode=${2:-mapping}
hub_port=${3:-47801}
kv_config=${4:-/home/robot/.ssh/geomem-kv.conf}
map_yaml=${5:-}
memory_file=${6:-}
map_sha256=${7:-}
case "$mode" in mapping|demo|attach|kv-up) ;; *) exit 2;; esac
[[ "$repo" = /* && "$hub_port" =~ ^[0-9]+$ ]] || exit 2
# A versioned deployment profile travels with the release and rolls back with its symlink.
# Explicit caller environment values still win (the profile uses shell default assignments).
if test -f "$repo/ui/geomem/tools/competition/runtime_detector.env"; then
  source "$repo/ui/geomem/tools/competition/runtime_detector.env"
fi
current=/data/fpga/car/state/car.current
if test -e "$current" || test -L "$current"; then
  if test "$mode" = demo; then
    echo 'Second run refused: a car session is already recorded. Stop and collect run 1 first; use bridge to attach to an existing session.' >&2
    exit 7
  fi
  python3 "$here/check_current.py" --current "$current" --hub-port "$hub_port"
  echo 'Reusing the verified running car stack. No restart or camera reopen.'
else
  test "$mode" != attach || { echo 'No running car stack to attach to.' >&2; exit 5; }
  if test "$mode" = demo; then
    python3 "$here/check_demo.py" --map-yaml "$map_yaml" --memory "$memory_file" --db-sha256 "$map_sha256"
  fi
  # FPGA profile (10-26, tools/car_run/fpga_profiles.env): a2 = the helper with no argument (as before),
  # v3fc2/v3fc2_166/v3fc3_187/v3fc/m1_166/m1/m2_166/m2 = "geomem-kv-start <profile>" (needs the KV helper installed with install_kv_service.sh --replace)
  kvarg=()
  if test -f "$repo/tools/car_run/fpga_profile.sh"; then
    source "$repo/tools/car_run/fpga_profile.sh"; fpga_profile || exit 2
    [ "$FPGA_PROFILE" = a2 ] || kvarg=("$FPGA_PROFILE")
  else   # a source repository without the profile file: the helper with no argument (a2), as before 10-26
    echo "WARNING: $repo/tools/car_run/fpga_profile.sh missing; starting the KV helper with no argument (a2)" >&2
  fi
  if ! kv_out=$(ssh -F "$kv_config" -o BatchMode=yes -o ConnectTimeout=8 geomem-kv260 sudo -n /usr/local/sbin/geomem-kv-start ${kvarg[@]+"${kvarg[@]}"}); then
    printf '%s\n' "$kv_out"
    test ${#kvarg[@]} -gt 0 || exit 1
    # owner 10-26 (no new refusals): an older KV helper does not take the profile -- start it the old way (a2);
    # car_up accepts any registered BUILD_ID and then fuses on the Nano CPU
    echo "WARNING: KV260 start with profile $FPGA_PROFILE failed; starting the KV helper with no argument (a2)" >&2
    kv_out=$(ssh -F "$kv_config" -o BatchMode=yes -o ConnectTimeout=8 geomem-kv260 sudo -n /usr/local/sbin/geomem-kv-start)
  fi
  printf '%s\n' "$kv_out"
  # 10-26 car screen (D-042): the KV service's JSON says whether its AR core really runs (it falls back to off by
  # itself when the screen cannot start). No "ar" key = a KV snapshot from before 10-26 = no AR.
  kv_json=$(printf '%s\n' "$kv_out" | grep '^{' | tail -1 || true)
  case "$kv_json" in
    *'"ar": "drm"'*) kv_ar=drm ;;
    *'"ar": "off"'*) kv_ar=off ;;
    *) kv_ar=unknown ;;
  esac
  if test "$kv_ar" = drm; then
    echo "AR_STATUS on: KV AR core running (car screen colour + boxes); colour sender starts with the stack"
  else
    reason=$(printf '%s' "$kv_json" | sed -n 's/.*"ar_reason": "\([^"]*\)".*/\1/p' || true)
    test "$kv_ar" = off || reason="KV snapshot without the AR switch (pre-10-26): deploy it with tools/car_run/deploy_kv.sh"
    case "$reason" in
      ar_core_absent*)   # AR2NANO 2026-11-02: the bitstream has no AR core; the Nano draws the car screen
        echo "AR_STATUS nano: ${reason} -- the KV has no AR core; car screen drawn by the Nano (CAR_AR_NANO)" ;;
      *) echo "AR_DISABLED reason=${reason:-unknown} -- car screen dark; the car stack starts normally" ;;
    esac
  fi
  # AR2NANO2 (2026-11-02): CAR_AR_NANO given by the caller wins (0|1); default: on exactly when the KV says its
  # bitstream has no AR core (ar_reason ar_core_absent, kv_start.sh SLOT_MAP check). Any other KV state: 0 = as before.
  # SW_M1 (2026-11-03): also on when the build the KV really serves (its ready JSON build_id) is registered in a
  # profile marked <P>_AR=nano (m1_166 / m1 / m2_166 / m2: no AR core), e.g. a reused server whose ar_reason is older; never when the
  # KV runs its own AR (drm) or the screen was switched off (kv_ar.conf / /etc ar.conf: reason "switched off ...").
  ar_nano=${CAR_AR_NANO:-auto}
  if test "$ar_nano" = auto; then
    case "${reason:-}" in ar_core_absent*) ar_nano=1 ;; *) ar_nano=0 ;; esac
    if test "$ar_nano" = 0 && test "$kv_ar" != drm && declare -F fpga_profile_of_build >/dev/null; then
      case "${reason:-}" in
        'switched off'*) ;;
        *) kv_bid=$(printf '%s' "$kv_json" | sed -n 's/.*"build_id": "\(0x[0-9a-fA-F]*\)".*/\1/p')
           if kv_prof=$(fpga_profile_of_build "${kv_bid:-0}") && \
              test "$( (fpga_profile "$kv_prof" && echo "$FPGA_AR") 2>/dev/null | tail -1)" = nano; then
             ar_nano=1
             echo "AR_STATUS nano: the KV serves BUILD_ID $kv_bid (profile $kv_prof, no AR core) -- car screen drawn by the Nano"
           fi ;;
      esac
    fi
  fi
  if test "$ar_nano" = 1; then
    echo "AR_NANO on: the car screen is drawn on this Nano (producer --ar-nano, DisplayPort, logs <run>/ar_nano/)"
    if systemctl is-active --quiet gdm 2>/dev/null || systemctl is-active --quiet gdm3 2>/dev/null; then
      echo "AR_NANO_WARNING: the desktop (gdm) holds the display; the car screen stays dark until: sudo systemctl stop gdm" >&2
    fi
  fi
  if test "$mode" = kv-up; then exit 0; fi
  # CAR_AR_COLOUR given by the caller wins; default: on exactly when the KV AR runs (or, AR2NANO2, the Nano draws it)
  ar_colour=${CAR_AR_COLOUR:-$( (test "$kv_ar" = drm || test "$ar_nano" = 1) && echo 1 || echo 0)}
  args=(env CAR_BOARD_DRAFT=1 CAR_HUB_BIND=127.0.0.1 "HUBPORT=$hub_port" CAR_TELEOP=1 CAR_PREVIEW=1 "CAR_AR_COLOUR=$ar_colour")
  if test "$ar_nano" = 1; then args+=(CAR_AR_NANO=1); fi   # 0: the car_up line exactly as before
  # 10-29 the KV service's ready JSON (car screen on/off, screen link) -> car_up's host_facts.json -> health check
  args+=("CAR_KV_READY_JSON=$kv_json" "CAR_KV_SSH_CONF=$kv_config")
  if test "$mode" = mapping; then
    args+=(CAR_MAPPING=1)
  else
    test -f "$map_yaml" && test -f "$memory_file" || { echo 'Second run requires a map and bound memory.' >&2; exit 6; }
    args+=("CAR_MAP_YAML=$map_yaml" "CAR_MEMORY_IN=$memory_file")
  fi
  cd "$repo"
  "${args[@]}" bash tools/car_run/car_up.sh live --pose odom --nav nav2 --dry-run
  "${args[@]}" bash tools/car_run/car_up.sh live --pose odom --nav nav2
fi
test "$mode" != kv-up || exit 0
bash "$here/start_live_mapping.sh"
# 10-21: direct-LAN operator gateway (bridge + page on the Nano; laptops connect with one ssh -L)
bash "$here/nano_gateway.sh" start || echo 'Operator gateway did not start; the server console path still works.' >&2
echo 'Car and display feed ready; no motion command sent.'
