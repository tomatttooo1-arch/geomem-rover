#!/bin/bash
# gov.sh show | set <governor>: read or set the cpufreq governor of every policy on the Nano (set needs root).
# Repo copy (2026-10-14). Prepared for the governor arm, which was NOT run on 10-13 (only "show" is safe without a
# lock).
# A TEMPORARY system setting for one experiment arm: the original value (schedutil, PROGRESS stage3) must be
# set back right after the arm and confirmed with "show".
case "${1:-show}" in
set)
  g=$2
  for p in /sys/devices/system/cpu/cpufreq/policy*; do
    grep -q -w "$g" "$p/scaling_available_governors" || { echo "governor $g not available on $p"; exit 2; }
  done
  for p in /sys/devices/system/cpu/cpufreq/policy*; do echo "$g" > "$p/scaling_governor"; done ;;
esac
for p in /sys/devices/system/cpu/cpufreq/policy*; do
  echo "$(basename "$p") governor=$(cat "$p/scaling_governor") cur=$(cat "$p/scaling_cur_freq") min=$(cat "$p/scaling_min_freq") max=$(cat "$p/scaling_max_freq") cpus=$(cat "$p/affected_cpus")"
done
