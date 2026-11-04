#!/bin/bash
# Host (g++) regression of tb_frame_stream.cpp with the forwarding-coverage frames,
# plus the two controls that show the frames still discriminate:
#   1. the C model as is                         -> must PASS
#   2. hazard model, no forwarding, blk_mark write visible 1 pixel late (= the
#      8051f22 RTL schedule)                      -> must FAIL (blk_w1/w2 frames)
#   3. hazard model, no forwarding, ev_pl write visible 1..3 pixels late
#      (hypothetical schedule), blk_mark 2..3 late -> must FAIL
#   4. hazard model with forwarding, both 3 late  -> must PASS (kFwd = 3 covers it)
# make_hz.py refuses (assert) if convert() no longer has the expected text: then
# the controls must be re-derived, not skipped.
# usage: run_host.sh OUTDIR
set -e
out=$1; here=$(cd "$(dirname "$0")" && pwd); top=$(cd "$here/../../.." && pwd); mkdir -p "$out"
H=$top/hw/m7/pl/hls; X=/data1/Xilinx/Vitis_HLS/2022.2/include
cf="-std=c++11 -O2 -ffp-contract=off -w -DFPGA_M7_LANES=8 -I $H -I $top/hw/m7 -I $X"
python3 "$here/make_hz.py" "$H/m7_frame_stream.cpp" "$out/m7_frame_stream_hz.cpp" > /dev/null
g++ $cf "$H/m7_frame_stream.cpp" "$top/hw/m7/frame_depth.cpp" "$H/tb_frame_stream.cpp" -o "$out/tb_plain"
g++ $cf "$out/m7_frame_stream_hz.cpp" "$top/hw/m7/frame_depth.cpp" "$H/tb_frame_stream.cpp" -o "$out/tb_hz"
bad=0
chk() {  # chk WANT(PASS|FAIL) LABEL env...
  want=$1; lab=$2; shift 2
  if env "$@" > "$out/$lab.log" 2> /dev/null; then got=PASS; else got=FAIL; fi
  d=$(grep '^\[fwdcov_' "$out/$lab.log" | sed 's/.*flag_bytes_diff=//' | tr '\n' ' ')
  echo "$lab want=$want got=$got fwdcov_diffs: $d"
  [ "$got" = "$want" ] || bad=1
}
chk PASS plain "$out/tb_plain"
chk FAIL nofwd_blk1 FWDCOV_NOFWD=1 FWDCOV_D_BLK=1 "$out/tb_hz"
chk FAIL nofwd_blk2 FWDCOV_NOFWD=1 FWDCOV_D_BLK=2 "$out/tb_hz"
chk FAIL nofwd_blk3 FWDCOV_NOFWD=1 FWDCOV_D_BLK=3 "$out/tb_hz"
chk FAIL nofwd_ev1 FWDCOV_NOFWD=1 FWDCOV_D_EV=1 "$out/tb_hz"
chk FAIL nofwd_ev2 FWDCOV_NOFWD=1 FWDCOV_D_EV=2 "$out/tb_hz"
chk FAIL nofwd_ev3 FWDCOV_NOFWD=1 FWDCOV_D_EV=3 "$out/tb_hz"
chk PASS fwd_both3 FWDCOV_D_BLK=3 FWDCOV_D_EV=3 "$out/tb_hz"
[ $bad = 0 ] && echo "FWDCOV_HOST PASS" || { echo "FWDCOV_HOST FAIL"; exit 1; }
