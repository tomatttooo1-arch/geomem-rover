#!/usr/bin/env bash
# bitexact positive/negative controls on the toy kernel
set -u
W=<worktree>; S=$W/tools/agent_skills/fpga-hls/scripts; SC=$W/.scratch_ross_scripts; BX=$SC/bx
mkdir -p $BX; rm -rf $BX/golden; cp -a $SC/toy_base/toy_prj/solution/csim/build/tb_out $BX/golden
echo "### 1 baseline cosim (wrapc_pc) vs golden"; python3 $S/bitexact_check.py --elem-size 2 $BX/golden $SC/toy_base/toy_prj/solution/sim/wrapc_pc/tb_out; echo "rc=$?"
for v in rerun:0 f1:1 f2:2; do
  name=${v%%:*}; f=${v##*:}
  rm -rf $SC/toy_$name
  TOY_BUILD=$SC/toy_$name/toy_prj TOY_FAULT=$f bash $S/hls_run.sh --timeout 900 --tcl $SC/toy_src/run_toy.tcl $SC/toy_$name/toy_prj solution csim > $SC/toy_$name.out 2>&1; echo "### run $name (TOY_FAULT=$f) hls_run rc=$?"
  python3 $S/bitexact_check.py --elem-size 2 $BX/golden $SC/toy_$name/toy_prj/solution/csim/build/tb_out; echo "rc=$?"
done
