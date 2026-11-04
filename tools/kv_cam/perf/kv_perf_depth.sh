#!/bin/bash
# NN-CAM1 depth-path CPU breakdown, runs ON THE KV260 as root (written 2026-11-04, NOT yet run).
# Package layout (pkg/kvcam1): bin/{kv_cam,kv_cam_read,perf_ctrl}, lib/ (OrbbecSDK v2.9.3 + extensions),
# OrbbecSDKConfig.xml, this script. PERF = path to an unpacked perf binary (no apt install; see the runbook).
#   kv_perf_depth.sh check                    perf + PMU + camera sanity, prints the perf event to use
#   kv_perf_depth.sh cal                      known-answer perf calibration (perf_ctrl, 30 s)
#   kv_perf_depth.sh arm <tag> [kv_cam args]  one arm: kv_cam warm-up 15 s, window A 30 s without perf (CPU split),
#                                             window B 60 s under perf (cpu-clock, flat), then report
# Outputs: $B/out/<tag>/ (stat_A.txt, stat_B.txt, threads.txt, irq_*.txt, perf.data, report_*.txt, kv_cam.log).
set -u
B=${KVCAM1_BASE:-/home/ubuntu/kvcam1}
PERF=${PERF:-$B/perfdeb/perf}
EV=${PERF_EVENT:-cpu-clock}
phase=${1:?check|cal|arm}; shift
mkdir -p $B/out

pstat() {  # utime stime (clock ticks) of PID, plus wall ns
  awk -v t="$(date +%s%N)" '{print t, $14, $15}' /proc/$1/stat
}
irqs() { grep -i -E 'xhci|dwc3|usb' /proc/interrupts; }
split() {  # $1 = file with two pstat lines -> cores user / sys / total
  awk -v hz="$(getconf CLK_TCK)" 'NR==1{t0=$1;u0=$2;s0=$3} NR==2{dt=($1-t0)/1e9; u=($2-u0)/hz/dt; s=($3-s0)/hz/dt;
       printf "window %.1f s: user %.3f cores, sys %.3f cores, total %.3f cores, sys share %.2f\n", dt, u, s, u+s, (u+s)>0?s/(u+s):0}' "$1"
}
report() {  # $1 = out dir
  $PERF report -i $1/perf.data --stdio --sort dso > $1/report_dso.txt 2>/dev/null
  $PERF report -i $1/perf.data --stdio --sort comm,dso > $1/report_comm_dso.txt 2>/dev/null
  $PERF report -i $1/perf.data --stdio --sort dso,sym > $1/report_dso_sym.txt 2>/dev/null
  echo "## top DSOs"; grep -v '^#' $1/report_dso.txt | grep -v '^$' | head -8
  echo "## top threads x DSO"; grep -v '^#' $1/report_comm_dso.txt | grep -v '^$' | head -8
  echo "## top symbols"; grep -v '^#' $1/report_dso_sym.txt | grep -v '^$' | head -12
}

case $phase in
check)
  uname -r; nproc
  ls /sys/bus/event_source/devices/
  echo "perf_event_paranoid $(cat /proc/sys/kernel/perf_event_paranoid)"
  $PERF --version || { echo "no perf at $PERF"; exit 3; }
  $PERF stat -e $EV -- sleep 1 2>&1 | tail -6
  lsusb -d 2bc5: ; for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && [ "$(cat $d/idVendor)" = 2bc5 ] && echo "$(basename $d) speed=$(cat $d/speed)"; done
  cd $B && timeout 30 ./bin/kv_cam --list | grep -E 'device:|hwd2c|^colour 640x480' ;;
cal)
  O=$B/out/cal; mkdir -p $O
  $B/bin/perf_ctrl --seconds 30 > $O/ctrl.stdout & P=$!
  sleep 2; pstat $P > $O/stat_B.txt
  $PERF record -e $EV -F 199 -p $P -o $O/perf.data -- sleep 20 > $O/perf_record.log 2>&1
  pstat $P >> $O/stat_B.txt; wait $P
  split $O/stat_B.txt | tee $O/split_B.txt
  echo "expected: user ~2.0 cores, sys ~1.0 core; samples ~1/3 user_spin_step, ~1/3 memcpy (libc), ~1/3 [kernel]; no sleeper"
  report $O ;;
arm)
  TAG=${1:?tag}; shift
  O=$B/out/$TAG; mkdir -p $O
  cd $B && ./bin/kv_cam --name kvcam1_perf --status-every-s 10 --alarm-log $O/alarms.jsonl --stats-out $O/kv_cam_stats.json \
    --sdk-log $O/sdk_log "$@" > $O/kv_cam.log 2>&1 &
  P=$!
  sleep 15
  if ! kill -0 $P 2>/dev/null; then echo "kv_cam exited early"; tail -20 $O/kv_cam.log; exit 4; fi
  irqs > $O/irq_A0.txt; pstat $P > $O/stat_A.txt; sleep 30; pstat $P >> $O/stat_A.txt; irqs > $O/irq_A1.txt
  top -H -b -n 1 -d 1 -p $P | head -30 > $O/threads.txt
  pstat $P > $O/stat_B.txt
  $PERF record -e $EV -F 199 -p $P -o $O/perf.data -- sleep 60 > $O/perf_record.log 2>&1
  pstat $P >> $O/stat_B.txt
  kill -TERM $P; wait $P
  rm -f /dev/shm/kvcam1_perf
  echo "## $TAG: kv_cam $*"
  echo -n "A (no perf): "; split $O/stat_A.txt | tee $O/split_A.txt
  echo -n "B (perf):    "; split $O/stat_B.txt | tee $O/split_B.txt
  grep -E 'prop |sets/s' $O/kv_cam.log | head -12
  grep -c ALARM $O/kv_cam.log | sed 's/^/alarms: /'
  report $O ;;
*) echo "check|cal|arm"; exit 2 ;;
esac
