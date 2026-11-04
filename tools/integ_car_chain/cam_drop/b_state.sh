#!/bin/bash
# Read-only board state (Nano). No lock needed.
# Repo copy (2026-10-14); lists ~/fpga of the logged-in Nano user.
date -Is
uptime
echo "--- nvpmodel"; nvpmodel -q 2>&1 | head -3
echo "--- docker ps"; docker ps --format '{{.Names}} {{.Image}} {{.Status}}'
echo "--- images"; docker images --format '{{.Repository}}:{{.Tag}} {{.Size}}' | grep -i fpga
echo "--- usb"; lsusb -d 2bc5: ; lsusb -t 2>/dev/null | grep -B2 -i -E "5000M|480M" | head -30
echo "--- governor"; for c in /sys/devices/system/cpu/cpu[0-9]*; do echo "$(basename $c) $(cat $c/cpufreq/scaling_governor) cur=$(cat $c/cpufreq/scaling_cur_freq) min=$(cat $c/cpufreq/scaling_min_freq) max=$(cat $c/cpufreq/scaling_max_freq) avail=$(cat $c/cpufreq/scaling_available_governors)"; done
echo "--- tools"; for t in stress-ng stress pidstat top taskset chrt perf g++ colcon; do printf "%s=%s " $t "$(command -v $t || echo NO)"; done; echo
echo "--- ntpcmp"; pgrep -a -f fpga_ntpcmp.py | head -3
echo "--- fpga dirs"; ls ~/fpga | head -40
echo "--- mem"; free -m | head -2
