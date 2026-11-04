#!/usr/bin/env bash
#
# 所以底图直接在服务器算（约 30 s），笔记本不用装 Python/Docker。桥每 2 s 扫一次监视目录，包出现即加载；
# 页面在里程/定位档、且地图 id 等于该库 sha256 时才画。
#
# 用法（在服务器、任一含本脚本的 main 副本里执行）：
#   bash ui/geomem/tools/basemap/server_basemap.sh --run <Nano 上第一趟运行目录>    # 经 ssh nano 读 memory_bind.json 并取库
#   bash ui/geomem/tools/basemap/server_basemap.sh --db  <服务器上的 .db> [--sha <64 位 sha256>]
# 选项：
#   --cache DIR   库与包的缓存目录（同一库第二次只复制，不重算）
#   --            其后参数原样传给 make_basemap.py（如 --method pointcloud、--thin-fill on）
# 环境：FPGA_BASEMAP_PY 指定带 open3d 0.19 的 Python（默认服务器上已备好的环境）。
#
# 换趟重建桥连接（connect_existing.sh）会把整个连接目录存档，监视目录随之清空：重建后再跑一次本脚本，
# 命中缓存只需复制，几秒完成。
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PY=${FPGA_BASEMAP_PY:-${FPGA_SRC}/runs/basemap-o3d-py311/bin/python}
WATCH=${FPGA_SRC}/runs/dev3-competition-connection/basemaps
CACHE=${FPGA_SRC}/runs/basemap-server
RUN="" DB="" SHA=""
EXTRA=()

die() { echo "server_basemap: $*" >&2; exit 1; }
while [ $# -gt 0 ]; do
  case "$1" in
    --run) RUN=$2; shift 2 ;;
    --db) DB=$2; shift 2 ;;
    --sha) SHA=$2; shift 2 ;;
    --watch) WATCH=$2; shift 2 ;;
    --cache) CACHE=$2; shift 2 ;;
    --) shift; EXTRA=("$@"); break ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) die "未知参数 $1（--help 看用法）" ;;
  esac
done
[ -n "$RUN" ] || [ -n "$DB" ] || die "需要 --run 或 --db"
[ -x "$PY" ] || die "找不到 Python：$PY（设 FPGA_BASEMAP_PY）"
mkdir -p "$CACHE/src" "$CACHE/pkg"

if [ -n "$RUN" ]; then
  # 第一趟收车后 memory_bind.json 给出宿主路径的库和 sha256（mapping_status.json 里的 /o/... 是容器路径，不能用）
  BIND=$(ssh nano cat "$RUN/memory_bind.json") || die "读不到 $RUN/memory_bind.json（第一趟是否已正常收车并绑定？）"
  read -r BSTATE BDB BSHA < <(printf '%s' "$BIND" | "$PY" -I -c '
import json, sys
b = json.load(sys.stdin)
print(b.get("bind", "-"), b.get("db") or "-", b.get("db_sha256") or "-")')
  case "$BSTATE" in bound|already_bound) ;; *) die "memory_bind.json 的 bind=$BSTATE，不是 bound";; esac
  [ "$BDB" != "-" ] || die "memory_bind.json 没有 db 路径；请手工取库后用 --db"
  SHA=$BSHA
  DB="$CACHE/src/${SHA:0:12}.db"
  if [ ! -f "$DB" ]; then
    echo "取库 jetson:$BDB -> $DB"
    scp -q "jetson:$BDB" "$DB.part" && mv "$DB.part" "$DB"
  fi
fi

[ -f "$DB" ] || die "库不存在：$DB"
ACT=$(sha256sum "$DB" | cut -d' ' -f1)
if [ -n "$SHA" ] && [ "$ACT" != "$SHA" ]; then
  die "库 sha256 不符：期望 $SHA，实际 $ACT（不要换成别的库）"
fi
SHA=$ACT
NAME="basemap-${SHA:0:12}"
PKG="$CACHE/pkg/$NAME"

if [ -d "$PKG" ]; then
  echo "缓存命中：$PKG"
else
  t0=$(date +%s)
  "$PY" "$HERE/make_basemap.py" --db "$DB" --out-dir "$CACHE/pkg" "${EXTRA[@]}"
  echo "生成用时 $(( $(date +%s) - t0 )) s"
fi

if [ "$WATCH" = none ]; then
  echo "完成（未投放）：$PKG"
  exit 0
fi
[ -d "$WATCH" ] || die "桥监视目录不存在：$WATCH（先起桥：connect_existing.sh；或 --watch 指定）"
if [ -d "$WATCH/$NAME" ]; then
  echo "桥目录里已有 $NAME，不重复投放"
else
  # 桥只认 basemap-<12 位十六进制> 名；先写临时名再改名，桥不会读到半个包
  cp -r "$PKG" "$WATCH/$NAME.tmp-$$" && mv "$WATCH/$NAME.tmp-$$" "$WATCH/$NAME"
  echo "已投放 $WATCH/$NAME（桥 2 s 内加载）"
fi
echo "地图库 sha256 $SHA —— 第二趟控制台的地图 id 必须等于它，页面才画底图"
