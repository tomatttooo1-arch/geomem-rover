#!/bin/bash
# seq.sh LIST_FILE: run the arms of LIST_FILE one after another on the Nano (each line: NAME VAR=VALUE ...;
# empty lines and lines starting with # are skipped). 8 s between arms. Progress to stdout.
B=$HOME/fpga/dev1_camdrop
while read -r line; do
  case "$line" in ""|\#*) continue;; esac
  echo "SEQ_START $(date +%T) $line"
  # shellcheck disable=SC2086
  bash $B/tools/bx.sh $line < /dev/null
  sleep 8
done < "$1"
echo "SEQ_DONE $(date +%T) containers=$(docker ps --format '{{.Names}}' | tr '\n' ' ')"
