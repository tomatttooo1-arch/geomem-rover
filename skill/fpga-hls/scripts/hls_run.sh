#!/usr/bin/env bash
# Copyright (C) 2026 the contest team. SPDX-License-Identifier: MIT
# Run-flow semantics (steps, preconditions, cosim argv fallback) derived from AMD Ross
# skill hls-run-flow: Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved. (MIT)
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
#
# Classic-Tcl replacement for the run layer of Ross skill `hls-run-flow`
# (upstream used `vitis-run --mode hls` / `v++ --compile --mode hls`, which do
# not exist in Vitis HLS 2022.2).
#
# Usage:
#   hls_run.sh [options] <proj_dir> <solution> <step>...
#     step in: csim | csynth | cosim | impl   (run in the order given)
#
# Options (may appear anywhere):
#   --tcl <file>         Project-creation Tcl (your run_*.tcl). REQUIRED when
#                        <proj_dir>/hls.app does not exist; ignored (with a note)
#                        when it does. The Tcl is sourced with csim_design /
#                        csynth_design / cosim_design / export_design /
#                        close_project / close_solution / exit stubbed out, so
#                        only its project-setup part (open_project, add_files,
#                        set_top, open_solution, set_part, create_clock,
#                        config_* / set_directive_*) takes effect. Arguments the
#                        Tcl passed to the stubbed step commands are reused as
#                        defaults for those steps in THIS invocation (e.g.
#                        `cosim_design -rtl verilog`, `csim_design -argv {8 8 8}`);
#                        later invocations (hls.app exists) use the generic defaults,
#                        hls.app/<sol>.aps argv, or the options below.
#                        Env vars the Tcl needs (FPGA_HLS_BUILD ...) must be set
#                        by the caller.
#   --argv "<args>"      Testbench argv for csim and cosim. Default: csim uses
#                        hls.app <Simulation argv>; cosim uses solution.aps
#                        <Simulation argv>, falling back to the csim argv
#                        (same semantics as upstream cosim.argv <- csim.argv).
#   --csim-opts "..."    Extra csim_design options.
#   --cosim-opts "..."   cosim_design options (default: from --tcl stub, else "-rtl verilog").
#   --impl-opts "..."    export_design options (default: from --tcl stub, else
#                        "-flow impl -rtl verilog -format ip_catalog"; "-flow impl"
#                        is always ensured).
#   --timeout <sec>      Wall-clock limit per step (default 0 = none).
#
# Preconditions (same semantics as upstream hls-run-flow):
#   cosim requires <proj_dir>/<solution>/syn ; impl requires <proj_dir>/<solution>/sim
#
# Logs: <proj_dir>/<solution>/agent_logs/<step>_<YYYYmmdd_HHMMSS>.log
# Terminal: last 30 lines of each step log + "STEP <step> EXIT_CODE=<rc>".
# Exit code: 0 all steps OK; first failing step's code otherwise
#            (2 = usage/precondition error, 124 = timeout).

set -uo pipefail

SETTINGS_DEFAULT="${HLS_SETTINGS:-/tools/Xilinx/Vitis_HLS/2022.2/settings64.sh}"  # skill package: overridable via HLS_SETTINGS

die() { echo "hls_run.sh: $*" >&2; exit 2; }

TCL=""; ARGV_SET=0; ARGV=""; CSIM_OPTS=""; COSIM_OPTS=""; IMPL_OPTS=""; TIMEOUT=0
POS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --tcl) TCL="${2:?--tcl needs a file}"; shift 2;;
    --argv) ARGV_SET=1; ARGV="${2-}"; shift 2;;
    --csim-opts) CSIM_OPTS="${2-}"; shift 2;;
    --cosim-opts) COSIM_OPTS="${2-}"; shift 2;;
    --impl-opts) IMPL_OPTS="${2-}"; shift 2;;
    --timeout) TIMEOUT="${2:?}"; shift 2;;
    -h|--help) sed -n '2,45p' "$0"; exit 0;;
    --*) die "unknown option $1";;
    *) POS+=("$1"); shift;;
  esac
done
[ ${#POS[@]} -ge 3 ] || die "usage: hls_run.sh [--tcl run.tcl] [--argv \"..\"] <proj_dir> <solution> <step>...  (step: csim|csynth|cosim|impl)"

PROJ_IN="${POS[0]}"; SOL="${POS[1]}"; STEPS=("${POS[@]:2}")
for s in "${STEPS[@]}"; do
  case "$s" in csim|csynth|cosim|impl) ;; *) die "unknown step '$s' (csim|csynth|cosim|impl)";; esac
done
case "$SOL" in */*|"") die "solution must be a bare name (e.g. solution), got '$SOL'";; esac

mkdir -p "$(dirname "$PROJ_IN")" || die "cannot create parent of $PROJ_IN"
PROJ="$(cd "$(dirname "$PROJ_IN")" && pwd)/$(basename "$PROJ_IN")"
SOLDIR="$PROJ/$SOL"
if [ -n "$TCL" ]; then
  [ -f "$TCL" ] || die "--tcl file not found: $TCL"
  TCL="$(cd "$(dirname "$TCL")" && pwd)/$(basename "$TCL")"
fi

if [ -z "${XILINX_HLS:-}" ]; then
  [ -r "$SETTINGS_DEFAULT" ] || die "XILINX_HLS unset and $SETTINGS_DEFAULT not found"
  # shellcheck disable=SC1090
  source "$SETTINGS_DEFAULT" >/dev/null 2>&1
fi
command -v vitis_hls >/dev/null || die "vitis_hls not on PATH after sourcing settings"

TS() { date +%Y%m%d_%H%M%S; }
# short scratch dir on purpose (vitis_hls cwd): long paths can exceed the ~108-byte Unix
# socket path limit used by Vivado/hw_server. TMPDIR is ignored here for that reason.
TMPD="$(mktemp -d /tmp/rh_XXXXXX)"
trap 'rm -rf "$TMPD"' EXIT

# Tcl helper: brace-quote a shell string for Tcl (rejects unbalanced braces).
tq() { case "$1" in *[{}]*) die "value contains braces, not supported: $1";; esac; printf '{%s}' "$1"; }

# Read an XML attribute <Simulation argv="..."> from a file (first match).
xml_argv() { [ -f "$1" ] && python3 - "$1" <<'PY' || true
import re, sys, html
t = open(sys.argv[1], errors="ignore").read()
m = re.search(r'<Simulation\b[^>]*\bargv="([^"]*)"', t)
print(html.unescape(m.group(1)) if m else "")
PY
}

STUB_ARGS_FILE="$TMPD/stub_args.txt"
# Vivado 2022.2 on this host segfaults in the licence library (libudev malloc mix-up) unless
# libudev is preloaded; root cause and fix: docs/design/M7_PL_BITSTREAM_20260927.md.
# Applied to the impl step only (export_design spawns vivado). Disable: HLS_RUN_NO_UDEV_PRELOAD=1
UDEV_LIB=/lib/x86_64-linux-gnu/libudev.so.1
run_vitis() {  # $1=tcl file, $2=log file ; returns rc
  local rc
  if [ "$TIMEOUT" != "0" ]; then
    (cd "$TMPD" && timeout --signal=TERM --kill-after=30 "$TIMEOUT" vitis_hls -nolog -f "$1") > "$2" 2>&1; rc=$?
  else
    (cd "$TMPD" && vitis_hls -nolog -f "$1") > "$2" 2>&1; rc=$?
  fi
  return $rc
}

report_step() {  # $1 step, $2 log, $3 rc
  echo "----- [$1] last 30 lines of $2 -----"
  tail -n 30 "$2"
  echo "STEP $1 EXIT_CODE=$3"
}

# ---------------- project setup via user Tcl (only when hls.app missing) -------------
if [ ! -f "$PROJ/hls.app" ]; then
  [ -n "$TCL" ] || die "$PROJ/hls.app not found: pass --tcl <run_hls.tcl> so the project can be created first"
  SETUP_TCL="$TMPD/setup.tcl"
  cat > "$SETUP_TCL" <<EOF
set __agent_stub_file $(tq "$STUB_ARGS_FILE")
foreach __c {csim_design csynth_design cosim_design export_design close_project close_solution exit} {
  rename \$__c __agent_orig_\$__c
  proc \$__c {args} [string map [list @C@ \$__c] {
    set fh [open \$::__agent_stub_file a]; puts \$fh [list @C@ \$args]; close \$fh
    puts "AGENT_STUB: skipped @C@ \$args"
  }]
}
if {[catch {uplevel #0 [list source $(tq "$TCL")]} __msg]} {
  puts "AGENT_SETUP_RESULT: FAIL sourcing user tcl: \$__msg"
  __agent_orig_exit 1
}
if {![file exists $(tq "$PROJ/hls.app")]} {
  puts "AGENT_SETUP_RESULT: FAIL user tcl did not create project at $(tq "$PROJ") (cwd=[pwd])"
  __agent_orig_exit 1
}
# restore the real commands before closing (close_project calls close_solution internally)
foreach __c {csim_design csynth_design cosim_design export_design close_project close_solution exit} {
  rename \$__c {}
  rename __agent_orig_\$__c \$__c
}
close_project
puts "AGENT_SETUP_RESULT: OK"
exit 0
EOF
  SETUP_LOG="$TMPD/setup_$(TS).log"
  run_vitis "$SETUP_TCL" "$SETUP_LOG"; rc=$?
  if [ $rc -ne 0 ] || ! grep -q "AGENT_SETUP_RESULT: OK" "$SETUP_LOG"; then
    [ $rc -eq 0 ] && rc=1
    report_step setup "$SETUP_LOG" $rc; exit $rc
  fi
  [ -d "$SOLDIR" ] || { report_step setup "$SETUP_LOG" 2; die "user tcl created $PROJ but no solution '$SOL' (have: $(ls "$PROJ" | tr '\n' ' '))"; }
  mkdir -p "$SOLDIR/agent_logs"; cp "$SETUP_LOG" "$SOLDIR/agent_logs/"
  echo "setup: project created by $TCL -> $PROJ (log $SOLDIR/agent_logs/$(basename "$SETUP_LOG"))"
else
  [ -n "$TCL" ] && echo "note: $PROJ/hls.app exists; --tcl ignored (delete the project dir to rebuild from Tcl)"
fi

# solution must exist in project (open_solution on a new name would silently create an empty one)
[ -d "$SOLDIR" ] || die "solution dir $SOLDIR not found (solutions in hls.app: $(grep -o '<solution name="[^"]*"' "$PROJ/hls.app" | sed 's/.*name="//;s/"//' | tr '\n' ' '))"
mkdir -p "$SOLDIR/agent_logs"

# defaults captured from the user tcl stubs (if any)
stub_default() {  # $1 command name -> prints args of last stub call
  [ -f "$STUB_ARGS_FILE" ] || return 0
  python3 - "$STUB_ARGS_FILE" "$1" <<'PY'
import sys
want = sys.argv[2]; last = None
for ln in open(sys.argv[1]):
    ln = ln.strip()
    if ln.split(" ", 1)[0] == want:
        last = ln.split(" ", 1)[1] if " " in ln else ""
if last is not None:
    # Tcl list of args -> strip one level of outer braces if present
    s = last.strip()
    if s.startswith("{") and s.endswith("}"):
        s = s[1:-1]
    print(s)
PY
}
[ -z "$COSIM_OPTS" ] && COSIM_OPTS="$(stub_default cosim_design)"
[ -z "$COSIM_OPTS" ] && COSIM_OPTS="-rtl verilog"
[ -z "$IMPL_OPTS" ] && IMPL_OPTS="$(stub_default export_design)"
[ -z "$IMPL_OPTS" ] && IMPL_OPTS="-rtl verilog -format ip_catalog"
# the impl step always runs the impl flow, whatever -flow the user tcl used
IMPL_OPTS="-flow impl $(python3 -c 'import re,sys; print(re.sub(r"-flow\s+\S+", "", sys.argv[1]).strip())' "$IMPL_OPTS")"
[ -z "$CSIM_OPTS" ] && CSIM_OPTS="$(stub_default csim_design)"
# -argv carried by the stubs becomes the default argv; it is then stripped from the opts
get_argv() { python3 -c 'import re,sys; m=re.search(r"-argv\s+(\{[^}]*\}|\"[^\"]*\"|\S+)", sys.argv[1]); print(m.group(1).strip("{}\"") if m else "")' "$1"; }
strip_argv() { python3 -c 'import re,sys; print(re.sub(r"-argv\s+(\{[^}]*\}|\"[^\"]*\"|\S+)", "", sys.argv[1]).strip())' "$1"; }
STUB_CSIM_ARGV="$(get_argv "$CSIM_OPTS")"; STUB_COSIM_ARGV="$(get_argv "$COSIM_OPTS")"
CSIM_OPTS="$(strip_argv "$CSIM_OPTS")"; COSIM_OPTS="$(strip_argv "$COSIM_OPTS")"

if [ $ARGV_SET -eq 1 ]; then
  CSIM_ARGV="$ARGV"; COSIM_ARGV="$ARGV"
else
  CSIM_ARGV="$STUB_CSIM_ARGV"; [ -z "$CSIM_ARGV" ] && CSIM_ARGV="$(xml_argv "$PROJ/hls.app")"
  COSIM_ARGV="$STUB_COSIM_ARGV"; [ -z "$COSIM_ARGV" ] && COSIM_ARGV="$(xml_argv "$SOLDIR/$SOL.aps")"
  [ -z "$COSIM_ARGV" ] && COSIM_ARGV="$CSIM_ARGV"
fi

FINAL_RC=0
for STEP in "${STEPS[@]}"; do
  case "$STEP" in
    cosim) [ -d "$SOLDIR/syn" ] || { echo "C Synthesis (csynth) must be completed before running Co-Simulation: $SOLDIR/syn missing. Run csynth first."; FINAL_RC=2; break; } ;;
    impl)  [ -d "$SOLDIR/sim" ] || { echo "C/RTL Co-Simulation (cosim) must be completed before running Implementation: $SOLDIR/sim missing. Run cosim first."; FINAL_RC=2; break; } ;;
  esac
  case "$STEP" in
    csim)   CMD="csim_design $CSIM_OPTS"; [ -n "$CSIM_ARGV" ] && CMD="$CMD -argv $(tq "$CSIM_ARGV")";;
    csynth) CMD="csynth_design";;
    cosim)  CMD="cosim_design $COSIM_OPTS"; [ -n "$COSIM_ARGV" ] && CMD="$CMD -argv $(tq "$COSIM_ARGV")";;
    impl)   CMD="export_design $IMPL_OPTS";;
  esac
  STEP_TCL="$TMPD/$STEP.tcl"
  cat > "$STEP_TCL" <<EOF
cd $(tq "$(dirname "$PROJ")")
open_project $(tq "$(basename "$PROJ")")
open_solution $(tq "$SOL")
puts "AGENT_STEP_CMD: $CMD"
if {[catch {$CMD} __msg]} {
  puts "AGENT_STEP_RESULT: FAIL \$__msg"
  exit 1
}
puts "AGENT_STEP_RESULT: OK"
exit 0
EOF
  LOG="$SOLDIR/agent_logs/${STEP}_$(TS).log"
  echo "== [$STEP] $CMD  (log: $LOG)"
  if [ "$STEP" = impl ] && [ -z "${HLS_RUN_NO_UDEV_PRELOAD:-}" ] && [ -e "$UDEV_LIB" ]; then
    echo "   (impl: LD_PRELOAD=$UDEV_LIB to avoid the Vivado 2022.2 licence-lib segfault)"
    LD_PRELOAD="$UDEV_LIB${LD_PRELOAD:+:$LD_PRELOAD}" run_vitis "$STEP_TCL" "$LOG"; rc=$?
  else
    run_vitis "$STEP_TCL" "$LOG"; rc=$?
  fi
  if [ $rc -eq 0 ]; then
    if ! grep -q "AGENT_STEP_RESULT: OK" "$LOG"; then rc=1; fi
    if [ "$STEP" = cosim ] && grep -q "co-simulation finished: FAIL" "$LOG"; then rc=1; fi
    if [ "$STEP" = csim ] && grep -qE "CSim done with [1-9][0-9]* errors" "$LOG"; then rc=1; fi
    if [ "$STEP" = impl ] && grep -qE "vivado returned an error|Abnormal program termination" "$LOG"; then rc=1; fi
  fi
  [ $rc -eq 124 ] && echo "TIMEOUT: step $STEP exceeded ${TIMEOUT}s"
  report_step "$STEP" "$LOG" $rc
  if [ $rc -ne 0 ]; then FINAL_RC=$rc; break; fi
done
exit $FINAL_RC
