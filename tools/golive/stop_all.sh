#!/bin/bash
# GOLIVE0 (2026-11-04): stop the whole bench product started by start_all.sh, both ends by pid (no pkill), then pull
# the logs to the server: RUN/GOLIVE0/raw/<TAG>/{kv,nano}/.  TAG defaults to golive-<date-time>.
#   bash tools/golive/stop_all.sh            stop KV (TERM, then KILL after 6 s) + Nano console, pull
#   bash tools/golive/stop_all.sh pull       pull only
set -u
RUN=${FPGA_RUN}
G=${GOLIVE_DIR:-$RUN/GOLIVE0}; KB=/home/ubuntu/golive_dev1; NB=/home/robot/fpga/dev1/golive0
SUDO=${FPGA_HOME}/dev1/.secrets/kv260_sudo
SSH=(ssh ${GOLIVE_SSH_F:+-F $GOLIVE_SSH_F} -o ConnectTimeout=30 -o ServerAliveInterval=30)
do_stop() {
  timeout 120 "${SSH[@]}" kv260 "sudo -S -p '' bash $KB/board/kv_golive.sh stop 2>&1 | tail -12" < "$SUDO"
  timeout 30 "${SSH[@]}" jetson "cd $NB && [ -f console.pid ] && kill -TERM \$(cat console.pid) && echo nano console stopped; rm -f console.pid"
}
do_pull() {
  D=$G/raw/${TAG:-golive-$(date +%Y%m%d-%H%M%S)}; mkdir -p "$D/kv" "$D/nano"
  rsync -a -e "${SSH[*]}" kv260:$KB/out/ "$D/kv/"; rsync -a -e "${SSH[*]}" jetson:$NB/ "$D/nano/"
  echo "pull -> $D"; echo "$D" > "$G/raw/LAST"
}
case ${1:-all} in
  stop) do_stop ;; pull) do_pull ;; all) do_stop; do_pull ;;
  *) sed -n 2,6p "$0" ;;
esac
