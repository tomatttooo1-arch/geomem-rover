#!/usr/bin/env bash
# acquire <资源> <用途> [-u 本人] 返回本次占用 token。
# release <资源> --token <token> [-u 本人]；status 只读。
set -euo pipefail
exec bash "$(dirname -- "$0")/workspace.sh" lock "$@"
