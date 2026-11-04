#!/usr/bin/env bash
# Runs ON THE KV260 (Kria Ubuntu 22.04), from the unpacked package directory:
#   sudo ./board_bringup.sh [step ...]      default: all steps in order
# Steps follow the acceptance checklist in M7_PL_INTEGRATION_20260926.md
# section 7. Every step prints PASS/FAIL/INFO lines and saves raw readouts to
# ./bringup-<timestamp>/ ; it stops at the first FAIL of steps 1-4 because
# later steps are not interpretable without them.
set -u
# App name under /lib/firmware/xilinx/. Packages built under another name (e.g.
# the 2026-10-11 merge build, m7_pl_merge) must not overwrite the baseline
# m7_pl: run as  sudo M7PL_APP=m7_pl_merge ./board_bringup.sh
APP=${M7PL_APP:-m7_pl}
HERE=$(cd "$(dirname "$0")" && pwd)
OUT="$HERE/bringup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
# DMA heap for m7pl_test (its built-in default is linux,cma). The KV260 used
# here only has /dev/dma_heap/reserved (+ system, not contiguous), so the 10-11
# merge bring-up had to run steps 2-7 by hand with --heap reserved. Override
# with M7PL_HEAP=<name>; otherwise try linux,cma then reserved (same order as
# HEAP_CANDIDATES in host/m7_pl_device.py); if neither exists, pass nothing.
HEAP=${M7PL_HEAP:-}
if [ -z "$HEAP" ]; then
  for h in linux,cma reserved; do [ -e "/dev/dma_heap/$h" ] && { HEAP=$h; break; }; done
fi
T=("$HERE/m7pl_test")
[ -n "$HEAP" ] && T+=(--heap "$HEAP")
log() { echo "$*" | tee -a "$OUT/SUMMARY.txt"; }
need_root() { [ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 2; }; }
need_root

step0_env() {  # readouts only: what the running system provides
  uname -a > "$OUT/uname.txt"; cat /proc/cmdline > "$OUT/cmdline.txt"
  grep -i cma /proc/meminfo > "$OUT/cma.txt" || true
  ls -l /dev/dma_heap/ > "$OUT/dma_heap.txt" 2>&1 || true
  ls /sys/class/u-dma-buf/ > "$OUT/udmabuf.txt" 2>&1 || true
  xmutil listapps > "$OUT/listapps_before.txt" 2>&1 || true
  ls /dev/dri/ > "$OUT/dri.txt" 2>&1 || true
  log "INFO cmdline: $(cat /proc/cmdline)"
  log "INFO $(tr '\n' ' ' < "$OUT/cma.txt")"
  log "INFO generic-uio on cmdline: $(grep -c 'uio_pdrv_genirq.of_id=generic-uio' /proc/cmdline)"
  log "INFO dma_heap: $(ls /dev/dma_heap 2>/dev/null | tr '\n' ' ')  u-dma-buf: $(cat "$OUT/udmabuf.txt" | tr '\n' ' ')"
  log "INFO m7pl_test heap: ${HEAP:-<m7pl_test default>} (M7PL_HEAP overrides)"
}
# Step 1 judges the load by the BUILD_ID read back from sysinfo (m7pl_test info), not by listapps (SMOKE1
# 2026-11-03): BOARD3 saw "xmutil loadapp" print "load Error: -1" right after the app directory was copied (dfx-mgrd
# still parsing it) while the PL kept the previous bitstream -- and the old check printed PASS because the app name
# was listed. Expected BUILD_ID: M7PL_BUILD_ID, else the package's SELFCHECK.txt line "expected sysinfo BUILD_ID on
# the board: 0x...". Readback different (or unreadable) => wait M7PL_LOAD_WAIT_S (20 s), unload, load again, up to
# M7PL_LOAD_TRIES (3) loads in all; still different => FAIL. No expected id known => WARN, listapps check as before.
# M7PL_FW_DIR (default /lib/firmware/xilinx) exists for tests.
FW=${M7PL_FW_DIR:-/lib/firmware/xilinx}
expected_build_id() {
  if [ -n "${M7PL_BUILD_ID:-}" ]; then echo "$M7PL_BUILD_ID"; return; fi
  sed -n 's/^expected sysinfo BUILD_ID on the board: \(0x[0-9a-fA-F]*\).*/\1/p' "$HERE/SELFCHECK.txt" 2>/dev/null | head -1
}
read_build_id() { "${T[@]}" info 2>&1 | sed -n 's/.*BUILD_ID=\(0x[0-9a-fA-F]*\).*/\1/p' | head -1; }
step1_load() {
  sha256sum -c SHA256SUMS > "$OUT/sha256.txt" 2>&1 && log "PASS package sha256" || { log "FAIL package sha256"; return 1; }
  mkdir -p "$FW"
  rm -rf "$FW/$APP"; cp -r "$HERE/$APP" "$FW/"
  want=$(expected_build_id)
  tries=${M7PL_LOAD_TRIES:-3} n=0 got=""
  while :; do
    n=$((n + 1))
    xmutil unloadapp > "$OUT/unload.txt" 2>&1 || true
    dmesg -c > /dev/null 2>&1 || true
    xmutil loadapp "$APP" > "$OUT/loadapp.txt" 2>&1; rc=$?
    dmesg > "$OUT/dmesg_load.txt" 2>&1
    xmutil listapps > "$OUT/listapps_after.txt" 2>&1
    cp "$OUT/loadapp.txt" "$OUT/loadapp_$n.txt"
    [ -n "$want" ] || break
    got=$(read_build_id)
    if [[ $got =~ ^0x[0-9a-fA-F]+$ ]] && (( got == want )); then break; fi
    log "WARN load $n of $APP: sysinfo BUILD_ID ${got:-unreadable}, expected $want ($(tr '\n' ' ' < "$OUT/loadapp.txt" | cut -c1-120))"
    [ "$n" -lt "$tries" ] || break
    sleep "${M7PL_LOAD_WAIT_S:-20}"
  done
  if [ -z "$want" ]; then
    log "WARN expected BUILD_ID unknown (no M7PL_BUILD_ID, no SELFCHECK.txt line): load judged by listapps only"
    if [ $rc = 0 ] && grep -q "$APP" "$OUT/listapps_after.txt"; then log "PASS xmutil loadapp $APP (listapps only)"
    else log "FAIL xmutil loadapp rc=$rc (see loadapp.txt, dmesg_load.txt)"; return 1; fi
  elif [[ $got =~ ^0x[0-9a-fA-F]+$ ]] && (( got == want )); then
    log "PASS xmutil loadapp $APP: sysinfo BUILD_ID $got (load $n)"
  else
    log "FAIL xmutil loadapp $APP: sysinfo BUILD_ID ${got:-unreadable} after $n loads, expected $want (see loadapp_*.txt, dmesg_load.txt)"
    return 1
  fi
  grep -iE "fpga|overlay|afi|fclk|uio|error" "$OUT/dmesg_load.txt" | tail -20 | sed 's/^/  dmesg: /' | tee -a "$OUT/SUMMARY.txt"
  ls /sys/class/uio/ > "$OUT/uio.txt" 2>&1 || true
  for u in /sys/class/uio/uio*; do [ -e "$u/name" ] && echo "$(basename "$u") $(cat "$u/name")"; done >> "$OUT/uio.txt" 2>/dev/null
  return 0   # the uio listing above is a readout, not a check (its last test decided the step's status before)
}
step2_info()     { "${T[@]}" info     > "$OUT/info.txt" 2>&1; r=$?; grep -E "PASS|FAIL|MAGIC|clock" "$OUT/info.txt" | tee -a "$OUT/SUMMARY.txt"; return $r; }
# Step 3 (PL watchdog self-test) is retired: v3fc removed the watchdog core.
step3_watchdog() { log "SKIP step 3: PL watchdog core removed (v3fc)"; return 0; }
# Step 4 (merge build 2026-10-11): ar replaced the legacy display core, roi
# (E2) is new; permit is an RTL block with its own known answers (the encoder
# UART core was removed in v3fc).
step4_small()    { "${T[@]}" --w 64 --h 48 frame ar roi permit > "$OUT/small.txt" 2>&1; r=$?; grep -E "PASS|FAIL|SKIP|buffers" "$OUT/small.txt" | tee -a "$OUT/SUMMARY.txt"; return $r; }
step5_full() {
  r=0
  for sz in "640 400" "1280 800"; do set -- $sz
    "${T[@]}" --timeout 30 --w "$1" --h "$2" frame ar roi > "$OUT/full_$1x$2.txt" 2>&1 || r=1
    grep -E "FAIL|^frame|^ar |^roi" "$OUT/full_$1x$2.txt" | tee -a "$OUT/SUMMARY.txt"
  done
  return $r
}
step6_irq() { "${T[@]}" irq > "$OUT/irq.txt" 2>&1; r=$?; grep -E "PASS|FAIL|SKIP" "$OUT/irq.txt" | tee -a "$OUT/SUMMARY.txt"; return $r; }
# Step 7 (drm demo) is retired: it was driven by the PL watchdog removed in v3fc.
step7_drm() { log "SKIP step 7: drm demo needed the removed PL watchdog (v3fc)"; return 0; }

STEPS=("$@"); [ ${#STEPS[@]} -eq 0 ] && STEPS=(0 1 2 3 4 5 6 7)
for s in "${STEPS[@]}"; do
  case $s in
    0) step0_env ;;
    1) step1_load || { log "STOP at step 1"; exit 1; } ;;
    2) step2_info || { log "STOP at step 2"; exit 1; } ;;
    3) step3_watchdog || { log "STOP at step 3"; exit 1; } ;;
    4) step4_small || { log "STOP at step 4"; exit 1; } ;;
    5) step5_full || log "step 5 had failures" ;;
    6) step6_irq || log "step 6 had failures" ;;
    7) step7_drm || log "step 7 had failures" ;;
    *) echo "unknown step $s" ;;
  esac
done
log "readouts in $OUT"
