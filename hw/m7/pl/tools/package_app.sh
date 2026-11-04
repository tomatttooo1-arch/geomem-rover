#!/usr/bin/env bash
# Assemble the `xmutil loadapp` firmware package from a finished build and run
# every host-side self-check. usage:
#   package_app.sh <build_dir> <ip_repo> <out_dir> [app_name]
# <out_dir>/<app_name>/ receives:  <app>.bit.bin  <app>.dtbo  shell.json
# plus <out_dir>/m7pl_test (aarch64 static), SELFCHECK.txt, SHA256SUMS.
# On the board: sudo cp -r <app> /lib/firmware/xilinx/ && sudo xmutil loadapp <app>
set -euo pipefail
BUILD=$(readlink -f "${1:?build dir}"); IPREPO=$(readlink -f "${2:?ip repo}")
OUT=$(readlink -f -m "${3:?out dir}"); APP=${4:-m7_pl}
HERE=$(cd "$(dirname "$0")/.." && pwd)
[ -f "$BUILD/m7_pl.bit" ] || { echo "no bitstream in $BUILD" >&2; exit 2; }
mkdir -p "$OUT/$APP"
CLK_MHZ=$(sed -n 's/^clk_mhz=//p' "$BUILD/SUMMARY.txt")
CLK_HZ=$(python3 -c "print(int(round(float('$CLK_MHZ')*1e6)))")
# v3b: compute-domain pl_clk1, the PS-reported actual rate (empty = single clock).
CLK1_MHZ=$(sed -n 's/^clk1_mhz_actual=//p' "$BUILD/SUMMARY.txt")
CLK1_ARGS=()
if [ -n "$CLK1_MHZ" ]; then
  CLK1_HZ=$(python3 -c "print(int(round(float('$CLK1_MHZ')*1e6)))")
  CLK1_ARGS=(--clk1-hz "$CLK1_HZ")
fi
# AR2NANO (2026-11-02): a build made with FPGA_M7_PL_NO_AR=1 (CLOCKS.txt no_ar=1) has no ar core: the
# overlay gets no m7_ar node and the check expects slot 8 / irq bit 7 empty. Other builds: unchanged.
NOAR_ARGS=()
if grep -qx 'no_ar=1' "$BUILD/CLOCKS.txt" 2>/dev/null; then NOAR_ARGS=(--no-ar); fi
# MERGE4 (2026-11-03): a build made with FPGA_M7_PL_MEM_DIRECT=1 (CLOCKS.txt mem_direct=1) has HP0/HP1
# at 32 bit and HP3 at 64 bit: the overlay's afi0 programs those widths (check compares with the .hwh).
MEMD_ARGS=(); if grep -qx 'mem_direct=1' "$BUILD/CLOCKS.txt" 2>/dev/null; then MEMD_ARGS=(--mem-direct); fi
# MERGE5A (2026-11-03): FPGA_M7_PL_DETPOST=1 (CLOCKS.txt detpost=1): m7_detpost node (slot 10, SPI 92), HP3 back
# to 128 bit in afi0, and the check covers the detpost mapping/irq/regs header. Other builds: unchanged.
DETP_ARGS=(); if grep -qx 'detpost=1' "$BUILD/CLOCKS.txt" 2>/dev/null; then DETP_ARGS=(--detpost); fi
# MERGE7 (2026-11-04): FPGA_M7_PL_LPD / _PREP / _PLANNER builds (CLOCKS.txt lpd=1, prep=1, planner=1): detpost on
# S_AXI_LPD (HP3 back to the rolling core alone), m7_prep / m7_planner nodes; the check takes the very IP files the
# build used (CLOCKS.txt prep_ip= / planner_ip=) for their register headers. Other builds: unchanged.
M7_OV_ARGS=(); M7_CK_ARGS=()
if grep -qx 'lpd=1' "$BUILD/CLOCKS.txt" 2>/dev/null; then M7_OV_ARGS+=(--lpd); M7_CK_ARGS+=(--lpd); fi
for n in prep planner; do
  if grep -qx "$n=1" "$BUILD/CLOCKS.txt" 2>/dev/null; then
    M7_OV_ARGS+=(--$n); M7_CK_ARGS+=(--$n --$n-ip "$(sed -n "s/^${n}_ip=//p" "$BUILD/CLOCKS.txt")")
  fi
done
# MERGE9P (2026-11-04): FPGA_M7_PL_SRCH / _MEMK / _OBS builds: m7_srch / m7_memk / m7_obs nodes (srch/memk on
# pl_ps_irq1), obs replaces m7_roi; overlay and check both take the very IP the build used (for obs: its m_axi
# width -> HP1 afi). Other builds: unchanged.
for n in srch memk obs; do
  if grep -qx "$n=1" "$BUILD/CLOCKS.txt" 2>/dev/null; then
    IPF=$(sed -n "s/^${n}_ip=//p" "$BUILD/CLOCKS.txt")
    M7_OV_ARGS+=(--$n --$n-ip "$IPF"); M7_CK_ARGS+=(--$n --$n-ip "$IPF")
  fi
done
# MERGE10: FPGA_M7_PL_MEMK_IF=bram builds (CLOCKS.txt memk_if=bram): check N6 (bc_memk/bm_memk in slot 14). Absent = lite.
if grep -q '^memk_if=' "$BUILD/CLOCKS.txt" 2>/dev/null; then
  MIF=$(sed -n 's/^memk_if=//p' "$BUILD/CLOCKS.txt"); M7_OV_ARGS+=(--memk-if "$MIF"); M7_CK_ARGS+=(--memk-if "$MIF")
fi
if grep -q '^obs_port=' "$BUILD/CLOCKS.txt" 2>/dev/null; then
  OP=$(sed -n 's/^obs_port=//p' "$BUILD/CLOCKS.txt"); M7_OV_ARGS+=(--obs-port "$OP"); M7_CK_ARGS+=(--obs-port "$OP")
fi
# DPU1 (2026-11-03): a build with FPGA_M7_PL_DPU_ARCH (CLOCKS.txt dpu_arch=...) gets the DPU node in the
# overlay and the DPU mapping/irq check; arch.json (fingerprint for the xmodel compiler) is copied next
# to the package. Other builds: unchanged.
DPU_OV_ARGS=(); DPU_CK_ARGS=()
if grep -q '^dpu_arch=' "$BUILD/CLOCKS.txt" 2>/dev/null; then
  DPU_HZ=$(python3 -c "print(int(round(float('$(sed -n 's/^dpu_clk_hz_actual=//p' "$BUILD/CLOCKS.txt")'))))")
  DSP_HZ=$(python3 -c "print(int(round(float('$(sed -n 's/^dsp_clk_hz_actual=//p' "$BUILD/CLOCKS.txt")'))))")
  DPU_OV_ARGS=(--dpu-clk-hz "$DPU_HZ" --dpu2x-clk-hz "$DSP_HZ"); DPU_CK_ARGS=(--dpu-clk-hz "$DPU_HZ")
fi
LOG="$OUT/SELFCHECK.txt"
: > "$LOG"
say() { echo "$*" | tee -a "$LOG"; }
say "package $APP from $BUILD at $(date -Is); clk=${CLK_HZ} Hz${CLK1_HZ:+ clk1=${CLK1_HZ} Hz}"

# 1. .bit -> .bit.bin (bootgen, as the ZynqMP fpga_manager expects).
TMP=$(mktemp -d)
cp "$BUILD/m7_pl.bit" "$TMP/$APP.bit"
printf 'all:\n{\n\t%s\n}\n' "$TMP/$APP.bit" > "$TMP/bit.bif"
( source /data1/Xilinx/Vitis/2022.2/settings64.sh >/dev/null 2>&1
  cd "$TMP" && bootgen -image bit.bif -arch zynqmp -process_bitstream bin -w on ) >> "$LOG" 2>&1
cp "$TMP/$APP.bit.bin" "$OUT/$APP/$APP.bit.bin"
say "bit.bin: $(stat -c %s "$OUT/$APP/$APP.bit.bin") bytes (bit: $(stat -c %s "$BUILD/m7_pl.bit"))"

# 2. Overlay: regenerate from the tcl address plan with this build's clock,
#    require it to equal the committed source when the clock matches.
python3 "$HERE/tools/m7pl_gen.py" overlay --clk-hz "$CLK_HZ" "${CLK1_ARGS[@]}" ${NOAR_ARGS[@]+"${NOAR_ARGS[@]}"} ${DPU_OV_ARGS[@]+"${DPU_OV_ARGS[@]}"} ${MEMD_ARGS[@]+"${MEMD_ARGS[@]}"} ${DETP_ARGS[@]+"${DETP_ARGS[@]}"} ${M7_OV_ARGS[@]+"${M7_OV_ARGS[@]}"} --bitname "$APP.bit.bin" --out "$TMP/$APP.dtso" >/dev/null
if [ "$APP" = m7_pl ] && [ "$CLK_HZ" = 100000000 ] && [ ${#NOAR_ARGS[@]} = 0 ]; then
  if cmp -s "$TMP/$APP.dtso" "$HERE/overlay/m7_pl.dtso"; then say "overlay source == committed overlay/m7_pl.dtso"
  else say "FAIL: regenerated overlay differs from committed overlay/m7_pl.dtso"; exit 1; fi
fi
dtc -@ -I dts -O dtb -o "$OUT/$APP/$APP.dtbo" "$TMP/$APP.dtso" 2>> "$LOG"
say "dtc -@ compile: OK ($(stat -c %s "$OUT/$APP/$APP.dtbo") bytes)"
cp "$TMP/$APP.dtso" "$OUT/$APP.dtso.txt"

# 3. shell.json (Kria dfx-mgr flat shell).
printf '{\n  "shell_type" : "XRT_FLAT",\n  "num_slots": "1"\n}\n' > "$OUT/$APP/shell.json"
python3 -c "import json,sys; d=json.load(open(sys.argv[1])); assert d['shell_type']=='XRT_FLAT'" "$OUT/$APP/shell.json"
say "shell.json: valid JSON, XRT_FLAT"

# 4. Consistency: tcl plan vs Vivado address map vs .hwh irq wiring vs dtbo vs driver headers.
python3 "$HERE/tools/m7pl_gen.py" check --build "$BUILD" --ip-repo "$IPREPO" \
    --dtbo "$OUT/$APP/$APP.dtbo" --clk-hz "$CLK_HZ" "${CLK1_ARGS[@]}" ${NOAR_ARGS[@]+"${NOAR_ARGS[@]}"} ${DPU_CK_ARGS[@]+"${DPU_CK_ARGS[@]}"} ${DETP_ARGS[@]+"${DETP_ARGS[@]}"} ${M7_CK_ARGS[@]+"${M7_CK_ARGS[@]}"} | tee -a "$LOG"
# A build resumed from another build's post_synth.dcp carries THAT build's BUILD_ID.
SRC_BUILD=$BUILD
[ -f "$BUILD/RESUMED_FROM.txt" ] && SRC_BUILD=$(cat "$BUILD/RESUMED_FROM.txt")
COMMIT=$(grep "START build out=$SRC_BUILD " "$(dirname "$SRC_BUILD")/LEDGER.txt" | tail -1 | sed -n 's/.*commit=\([0-9a-f]*\).*/\1/p')
say "expected sysinfo BUILD_ID on the board: 0x0${COMMIT:0:7} (source commit $COMMIT)"

# 5. Driver: aarch64 static build + mock runs (host and qemu), incl. corruption control.
make -s -C "$HERE/sw" BUILD="$TMP/sw" check > "$TMP/swcheck.log" 2>&1 && say "driver make check: PASS" \
  || { say "driver make check: FAIL"; cat "$TMP/swcheck.log" >> "$LOG"; exit 1; }
grep RESULT "$TMP/swcheck.log" | sed 's/^/  /' | tee -a "$LOG"
cp "$TMP/sw/aarch64/m7pl_test" "$OUT/m7pl_test"
file "$OUT/m7pl_test" | tee -a "$LOG"

# 6. Timing/resources summary next to the package.
cp "$BUILD/SUMMARY.txt" "$OUT/BUILD_SUMMARY.txt"
cp "$BUILD/utilization_route.rpt" "$BUILD/timing_route.rpt" "$BUILD/address_map.csv" "$OUT/"
cp "$BUILD"/*.hwh "$OUT/"
if [ ${#DPU_CK_ARGS[@]} -gt 0 ]; then cp "$BUILD/arch.json" "$BUILD/CLOCKS.txt" "$OUT/"; say "dpu arch.json: $(cat "$OUT/arch.json")"; fi
cp "$HERE/tools/board_bringup.sh" "$OUT/"
cp "$HERE/tools/m5_lpd_afi.py" "$OUT/"   # DRV5: board_bringup.sh step 1 runs it after the load
# SELFCHECK.txt is itself hashed, so nothing may be appended to it after this
# point (board_bringup.sh step 1 runs `sha256sum -c SHA256SUMS`).
say "writing SHA256SUMS over $(find "$OUT" -type f ! -name SHA256SUMS | wc -l) files"
( cd "$OUT" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS )
( cd "$OUT" && sha256sum --quiet -c SHA256SUMS ) || { echo "FAIL: SHA256SUMS does not verify" >&2; exit 1; }
echo "SHA256SUMS written and verified ($(wc -l < "$OUT/SHA256SUMS") files)"
rm -rf "$TMP"
