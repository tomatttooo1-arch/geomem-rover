#!/usr/bin/env bash
# Restore only this deployment; refuse to interrupt a later unrelated release/session.
set -euo pipefail
root=/data/fpga/dev3/household9-20261027
link=/home/robot/fpga/dev1/console-current
baseline=$(cat "$root/active-baseline.txt" 2>/dev/null || printf '%s\n' "$root/baseline")
baseline=$(readlink -f "$baseline")
case "$baseline" in "$root/baseline"|"$root"/baselines/*) ;; *) echo 'Invalid backup location' >&2; exit 8;; esac
old=$(cat "$baseline/previous-release.txt")
candidate=$(cat "$root/activated-release.txt" 2>/dev/null || cat "$root/staged-release.txt")
current=$(readlink -f "$link")
case "$current" in "$candidate"|"$old") ;; *) echo 'A different release is active; refusing rollback.' >&2; exit 8;; esac
python3 - "$baseline" <<'PY'
import hashlib,json,pathlib,sys
p=pathlib.Path(sys.argv[1])
for f in json.loads((p/'backup-complete.json').read_text())['files']:
    assert hashlib.sha256((p/f['path']).read_bytes()).hexdigest()==f['sha256'],f['path']
PY
test "$(cat "$old/COMMIT")" = "$(cat "$baseline/COMMIT")"
test "$(sha256sum /data/fpga/yolo_engine_20261015/yolov8n-seg-640-fp16.engine | cut -d' ' -f1)" = \
     "$(sha256sum "$baseline/yolov8n-seg-640-fp16.engine" | cut -d' ' -f1)"
if test -L /data/fpga/car/state/car.current; then
  run=$(readlink -f /data/fpga/car/state/car.current)
  python3 - "$run/head.txt" "$candidate/COMMIT" "$old/COMMIT" <<'PY'
import pathlib,sys
d=pathlib.Path(sys.argv[1]).read_text().strip()
allowed=[pathlib.Path(p).read_text().strip() for p in sys.argv[2:]]
assert d in allowed, 'Unknown active session version; refusing stop'
PY
  if test "${1:-}" = --check; then echo 'Rollback checks passed'; exit 0; fi
  KEEP_SRC=1 bash "$current/tools/car_run/car_down.sh" --run "$run" --rollback household9
fi
if test "${1:-}" = --check; then echo 'Rollback checks passed (no current session)'; exit 0; fi
bash "$current/ui/geomem/tools/competition/nano_gateway.sh" stop
ln -s "$old" "${link}.household9-rollback"
mv -Tf "${link}.household9-rollback" "$link"
unset YOLO_PROFILE YOLO_ENGINE
bash "$old/ui/geomem/tools/competition/console_remote.sh" up
bash "$old/ui/geomem/tools/competition/console_remote.sh" status
date -Is > "$root/rollback-complete.txt"
