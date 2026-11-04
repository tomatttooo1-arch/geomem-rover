#!/usr/bin/env bash
# XSim regression for the drive-permit heartbeat and encoder-UART blocks (2026-10-11).
#
#   hw/m7/pl/sim/run_permit_uart_sim.sh <out_dir> [quick]
#
# Runs, each in its own subdirectory of <out_dir> (must not exist):
#   permit_full   scaled timing, all scenes                     expect PASS
#   permit_real   1000 ms power-on window / 5 ms at 100 MHz     expect PASS (skipped with `quick`)
#   enc           stimulus from enc_uart_model.py gen, check    expect PASS
#   mut_<name>    each positive control from mutate.py          expect FAIL
# Prints one verdict line per run and SIM_SUITE PASS only if every run behaved
# as expected (passing runs passed AND every mutated run failed).
set -u
out=${1:?usage: run_permit_uart_sim.sh <out_dir> [quick]}
mode=${2:-full}
here=$(cd "$(dirname "$0")" && pwd)
rtl=$(cd "$here/../rtl" && pwd)
tools=$(cd "$here/../tools" && pwd)
xil=${FPGA_XILINX_BIN:-/data1/Xilinx/Vivado/2022.2/bin}
export PATH="$xil:$PATH"
[ -e "$out" ] && { echo "out dir exists: $out"; exit 2; }
mkdir -p "$out"
out=$(cd "$out" && pwd)
suite_ok=1

# build <dir> <top> <snapshot> <generics...>; sources from $permit_src/$enc_src
build() {
    local d=$1 top=$2 snap=$3; shift 3
    ( cd "$d" &&
      xvlog "$rtl/m7_timebase.v" "$permit_src" "$enc_src" > compile.log 2>&1 &&
      xvlog -sv "$rtl/$top.sv" >> compile.log 2>&1 &&
      xelab -debug off "$top" -s "$snap" "$@" >> compile.log 2>&1 )
}

run_permit() {   # name src generics... ; echoes PASS/FAIL
    local name=$1 src=$2; shift 2
    local d="$out/$name"; mkdir -p "$d"
    permit_src=$src enc_src="$rtl/m7_enc_uart_axil.v"
    if ! build "$d" tb_m7_permit snap "$@"; then echo COMPILE_FAIL; return; fi
    ( cd "$d" && timeout 1800 xsim snap -R > sim.log 2>&1 )
    if grep -q "^TB_M7_PERMIT PASS" "$d/sim.log" && ! grep -q "^ERROR" "$d/sim.log"; then echo PASS; else echo FAIL; fi
}

run_enc() {      # name src ; echoes PASS/FAIL
    local name=$1 src=$2
    local d="$out/$name"; mkdir -p "$d"
    python3 "$tools/enc_uart_model.py" gen "$d" > "$d/gen.log" 2>&1 || { echo GEN_FAIL; return; }
    permit_src="$rtl/m7_permit_axil.v" enc_src=$src
    if ! build "$d" tb_m7_enc_uart snap; then echo COMPILE_FAIL; return; fi
    ( cd "$d" && timeout 1800 xsim snap -R -testplusarg "STIM=$d/stim.txt" -testplusarg "LOG=$d/tb_log.txt" > sim.log 2>&1 )
    if python3 "$tools/enc_uart_model.py" check "$d" > "$d/check.log" 2>&1; then echo PASS; else echo FAIL; fi
}

verdict() {      # name got want
    local ok=no
    [ "$2" = "$3" ] && ok=yes
    [ "$ok" = yes ] || suite_ok=0
    printf '%-34s got %-12s want %-5s %s\n' "$1" "$2" "$3" "$([ $ok = yes ] && echo OK || echo MISMATCH)"
}

v=$(run_permit permit_full "$rtl/m7_permit_axil.v"); verdict permit_full "$v" PASS
grep -E "^info" "$out/permit_full/sim.log" | sed 's/^/    /'
if [ "$mode" != quick ]; then
    v=$(run_permit permit_real "$rtl/m7_permit_axil.v" -generic_top TIMEOUT=100000000 \
        -generic_top HALF=500000 -generic_top FULL=0)
    verdict permit_real "$v" PASS
    grep -E "^(ok|info)" "$out/permit_real/sim.log" | sed 's/^/    /'
fi
v=$(run_enc enc "$rtl/m7_enc_uart_axil.v"); verdict enc "$v" PASS
grep -E "PASS|latency|counters" "$out/enc/check.log" | sed 's/^/    /'

for m in permit_ps_write_refreshes permit_replay_accepted permit_force_keeps_armed \
         permit_window_write_keeps_armed permit_window_no_clamp; do
    mkdir -p "$out/mut_$m"
    if python3 "$here/mutate.py" "$m" "$rtl/m7_permit_axil.v" "$out/mut_$m/m7_permit_axil.v" > "$out/mut_$m/mutate.log"; then
        v=$(run_permit "mut_$m" "$out/mut_$m/m7_permit_axil.v")
    else v=NOT_APPLIED; fi
    verdict "mut_$m" "$v" FAIL
    grep -m3 "^ERROR" "$out/mut_$m/sim.log" 2>/dev/null | sed 's/^/    /'
done
for m in uart_checksum_skips_flags uart_no_aa_resync uart_lost_off_by_one odom_speed_sign; do
    mkdir -p "$out/mut_$m"
    if python3 "$here/mutate.py" "$m" "$rtl/m7_enc_uart_axil.v" "$out/mut_$m/m7_enc_uart_axil.v" > "$out/mut_$m/mutate.log"; then
        v=$(run_enc "mut_$m" "$out/mut_$m/m7_enc_uart_axil.v")
    else v=NOT_APPLIED; fi
    verdict "mut_$m" "$v" FAIL
    grep -m3 "^ERROR" "$out/mut_$m/check.log" 2>/dev/null | sed 's/^/    /'
done

if [ $suite_ok = 1 ]; then echo "SIM_SUITE PASS"; else echo "SIM_SUITE FAIL"; exit 1; fi
