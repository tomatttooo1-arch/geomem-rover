#!/usr/bin/env bash
set -euo pipefail
root=${FPGA_ROOT:-${FPGA_HOME}}
local_admin=$(cd -- "$(dirname -- "$0")/admin" && pwd -P)/workspace.py
# Clone initialization/checks must also work on an offline host.
case "${1:-}" in
  init-clone|check-index) exec python3 "$local_admin" --root "$root" "$@" ;;
esac
if [ ! -f "$root/.workspace/admin/workspace.py" ]; then
  echo '公共工作区尚未迁移；请勿直接运行同步/锁操作。' >&2
  exit 1
fi
exec python3 "$root/.workspace/admin/workspace.py" --root "$root" "$@"
