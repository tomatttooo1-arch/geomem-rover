#!/bin/bash
# T2 (2026-10-16): run the arms one after another, detached (survives the controlling agent's hand-over).
#   nohup setsid bash t2_chain.sh > runs/hbgap-t2-20261016/chain.log 2>&1 < /dev/null &
# ARMS (default "A B1 B2 B3"); DEPLOY pinned once at the start (later commits in the work tree do not change what the
# remaining arms deploy). A session that fails for any reason but busy locks (rc 3) is rerun once (PREREG: at most one
# rerun per arm). GAP_S between arms: the locks are free then (another agent may take them; the next session waits).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
W=$(cd "$HERE/../../.." && pwd)
R=${R:-${FPGA_SRC}/runs/hbgap-t2-20261016}
: "${ARMS:=A B1 B2 B3}" "${GAP_S:=90}"
export DEPLOY=${DEPLOY:-$(git -C "$W" rev-parse HEAD)} W R
echo "$(date -Is) chain start pid $$ arms '$ARMS' deploy $DEPLOY" | tee -a "$R/chain_status.txt"
first=1
for arm in $ARMS; do
  [ "$first" = 1 ] || sleep "$GAP_S"; first=0
  for try in 1 2; do
    echo "$(date -Is) arm $arm try $try start" | tee -a "$R/chain_status.txt"
    ARM=$arm bash "$HERE/t2_session.sh" run; rc=$?
    echo "$(date -Is) arm $arm try $try rc=$rc" | tee -a "$R/chain_status.txt"
    [ "$rc" = 0 ] && break
    [ "$rc" = 3 ] && { echo "$(date -Is) locks busy too long / not held: chain stops" | tee -a "$R/chain_status.txt"; exit 3; }
    [ "$try" = 1 ] && sleep "$GAP_S"
  done
done
echo "$(date -Is) chain done" | tee -a "$R/chain_status.txt"
