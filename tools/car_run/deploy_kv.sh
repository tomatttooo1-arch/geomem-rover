#!/usr/bin/env bash
# 把某个提交部署成 KV260 上比赛 KV 服务的固定快照（在服务器上跑，10-26）。与 deploy_nano.sh 配套、同一提交：
#   git archive <sha> → 宿主临时目录，写 COMMIT，交叉编 AR 送显助手（tools/kv_link/ar_helper，make aarch64，
#   板上没有 HLS 头文件编不了）→ 打 tar 经 Nano 送到 KV 暂存目录 <stage>/geomem-<sha7>
#   → KV 上 sudo install_kv_service.sh --update <暂存目录>（旧快照移入 releases/ 留作回滚）→ 校验已装 COMMIT 与助手 sha256。
#
#   bash tools/car_run/deploy_kv.sh [--ref origin/main] [--host jetson] [--kv-conf /home/robot/.ssh/geomem-kv.conf]
#        [--stage /home/ubuntu/fpga/kv-stage] [--sudo-stdin] [--stop-kv] [--repo <本地仓>] [--dry-run]
#
# KV 上 sudo：默认 sudo -n（免密）；要密码时加 --sudo-stdin，脚本在本终端不回显地读一次密码（标准输入不是终端时读其第一行），经 ssh 标准输入交给
#   KV 上的 sudo -S（不进文件、不进命令行、不进日志）。--stop-kv：安装前先 SIGTERM 正在跑的 kv_server
#   （车栈必须先 down；不加则安装器见到 kv_server 在跑会拒绝，退出码 6）。
# 不自动取板锁，只在开头打印取锁命令；--dry-run 不连板、不编译，只按顺序打印每条命令。
# 退出码：0 成功；2 参数错；3 解析 ref 失败；4 Nano/KV 不可达；5 打包/编译/传输失败；6 安装失败；7 校验失败。
set -uo pipefail

REF=origin/main
HOST=jetson
KVCONF=/home/robot/.ssh/geomem-kv.conf
STAGE=/home/ubuntu/fpga/kv-stage
SUDO_STDIN=0
STOP_KV=0
DRY=0
REPO=""
usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; }
while [ $# -gt 0 ]; do
  case "$1" in
    --ref) REF=${2:?}; shift 2 ;;
    --host) HOST=${2:?}; shift 2 ;;
    --kv-conf) KVCONF=${2:?}; shift 2 ;;
    --stage) STAGE=${2:?}; shift 2 ;;
    --sudo-stdin) SUDO_STDIN=1; shift ;;
    --stop-kv) STOP_KV=1; shift ;;
    --repo) REPO=${2:?}; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done
case "$STAGE" in /*) ;; *) echo "--stage 必须是绝对路径" >&2; exit 2 ;; esac
[ -n "$REPO" ] || REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel 2>/dev/null) \
  || { echo "找不到本地仓（用 --repo 指定）" >&2; exit 2; }
SHA=$(git -C "$REPO" rev-parse --verify -q "$REF^{commit}") || { echo "解析不了 ref：$REF（需要最新请先 git -C $REPO fetch origin）" >&2; exit 3; }
SHORT=${SHA:0:7}
NAME=geomem-$SHORT
DEST=$STAGE/$NAME
KVBASE=${FPGA_KV_BASE:-/opt/geomem-dev3-kv}
COMP=ui/geomem/tools/competition
HELPER=tools/kv_link/ar_helper/build/aarch64/m7ar_present
MAKE_CMD=${DEPLOY_KV_MAKE:-make}
LOCK_SH=${FPGA_LOCK_SH:-${FPGA_HOME}/tools/lock.sh}
q() { printf '%q' "$1"; }
# shellcheck disable=SC2206
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 ${DEPLOY_SSH_OPTS:-} "$HOST")
KVSSH="ssh -F $(q "$KVCONF") -o BatchMode=yes -o ConnectTimeout=8 geomem-kv260"
# K "<KV 上的命令串>"：经 Nano 再 ssh 到 KV；stdin 一路透传（只在传 tar / sudo 密码时用到）
K() {
  if [ "$DRY" = 1 ]; then echo "   [${SSH[*]} → $KVSSH] $1"; return 0; fi
  "${SSH[@]}" "$KVSSH $(q "$1")"
}
step() { echo "-- $*"; }

echo "== 部署 KV 快照 $REF = $SHA 经 $HOST 到 KV260（$KVBASE/source$([ $DRY = 1 ] && echo '，dry-run')）"
echo "== 提醒：本脚本不取板锁。先取锁（用完 release）；KV 服务在跑时要先收车栈（console_remote.sh down）："
echo "     bash $LOCK_SH acquire kv260 \"部署 KV $SHORT\" -u dev1"

# 1. KV 可达，已装 COMMIT
step "1. 查 KV 现状（经 $HOST）"
if [ "$DRY" = 1 ]; then K "hostname; head -1 $KVBASE/source/COMMIT; pgrep -af '[k]v_server.py'"
else
  out=$(K "hostname >/dev/null && echo REACH; echo HAVE=\$(head -1 $(q "$KVBASE/source/COMMIT") 2>/dev/null); \
pgrep -af '[k]v_server.py' | sed 's/^/RUN /'" </dev/null)
  rc=$?
  echo "$out" | grep -qx REACH || { echo "FAIL KV 不可达（经 $HOST，ssh 返回 $rc）；未做任何改动" >&2; exit 4; }
  HAVE=$(echo "$out" | sed -n 's/^HAVE=//p')
  echo "   KV 已装快照 COMMIT=${HAVE:-（无）}"
  if echo "$out" | grep -q '^RUN '; then
    echo "   kv_server 正在跑：$(echo "$out" | grep '^RUN ' | head -1 | cut -c5-160)"
    [ "$STOP_KV" = 1 ] || echo "   （不加 --stop-kv 时安装器会拒绝；车栈先 down 再加 --stop-kv 重跑）"
  fi
fi

# 2. 打包 + 交叉编助手
step "2. git archive $SHORT → 宿主临时目录，写 COMMIT，交叉编 AR 助手（$HELPER）"
if [ "$DRY" = 1 ]; then
  echo "   git -C $(q "$REPO") archive --format=tar $SHA | tar -x -C <tmp>; echo $SHA > <tmp>/COMMIT"
  echo "   $MAKE_CMD -C <tmp>/tools/kv_link/ar_helper BUILD=build aarch64; sha256sum <tmp>/$HELPER"
  HSHA="<助手 sha256>"
else
  TMPD=$(mktemp -d "${TMPDIR:-/tmp}/deploy_kv.XXXXXX") || exit 5
  trap 'rm -rf "$TMPD" "$TMPD.make.log"' EXIT
  git -C "$REPO" archive --format=tar "$SHA" | tar -x -C "$TMPD" || { echo "FAIL git archive 解包失败" >&2; exit 5; }
  echo "$SHA" > "$TMPD/COMMIT"
  $MAKE_CMD -C "$TMPD/tools/kv_link/ar_helper" BUILD=build aarch64 >"$TMPD.make.log" 2>&1 \
    || { echo "FAIL 助手交叉编译失败（日志 $TMPD.make.log 末尾）：" >&2; tail -5 "$TMPD.make.log" >&2; exit 5; }
  test -x "$TMPD/$HELPER" || { echo "FAIL 编译后没有 $HELPER" >&2; exit 5; }
  HSHA=$(sha256sum "$TMPD/$HELPER" | cut -d' ' -f1)
  echo "   助手 sha256 $HSHA"
fi

# 3. 传到 KV 暂存目录（同名目录已在就整个换掉：暂存不是已装快照）
step "3. 传到 KV $DEST"
remote_x="set -e; mkdir -p $(q "$STAGE"); rm -rf $(q "$DEST.tmp"); mkdir $(q "$DEST.tmp"); tar -x -C $(q "$DEST.tmp"); \
rm -rf $(q "$DEST"); mv -T $(q "$DEST.tmp") $(q "$DEST"); head -1 $(q "$DEST/COMMIT")"
if [ "$DRY" = 1 ]; then printf '   tar -C <tmp> -cf - . | '; K "$remote_x"
else
  got=$(tar -C "$TMPD" -cf - . | K "$remote_x")
  [ "$got" = "$SHA" ] || { echo "FAIL 传输/解包失败或暂存 COMMIT 不符（得到「$got」）" >&2; exit 5; }
fi

# 4. sudo 安装（--update；旧快照留在 releases/）
step "4. KV 上 sudo install_kv_service.sh --update $DEST"
inst="bash $(q "$DEST/$COMP/install_kv_service.sh") --update $(q "$DEST")"
if [ "$STOP_KV" = 1 ]; then
  inst="for p in \$(pgrep -f '[k]v_server.py'); do kill -TERM \$p; done; \
for i in \$(seq 1 50); do pgrep -f '[k]v_server.py' >/dev/null || break; sleep 0.2; done; $inst"
fi
if [ "$SUDO_STDIN" = 1 ]; then
  if [ "$DRY" = 1 ]; then echo "   （本终端不回显读一次 KV sudo 密码，经 ssh 标准输入交给 sudo -S）"; K "sudo -S -p '' bash -c $(q "$inst")"
  else
    if [ -t 0 ]; then read -r -s -p "KV260 ubuntu 的 sudo 密码（不回显，不落盘）：" PW; echo
    else IFS= read -r PW; fi   # 非终端：从本脚本标准输入读一行（如 < 已有的口令文件），不经命令行
    printf '%s\n' "$PW" | K "sudo -S -p '' bash -c $(q "$inst")"; rc=$?; unset PW
  fi
else
  K "sudo -n bash -c $(q "$inst")" </dev/null; rc=$?
fi
if [ "$DRY" = 0 ] && [ "$rc" != 0 ]; then
  echo "FAIL 安装失败（rc=$rc）。sudo 要密码时加 --sudo-stdin；kv_server 在跑时先收车栈再加 --stop-kv。已装快照未变（安装器失败即停）" >&2
  exit 6
fi

# 5. 校验已装 COMMIT 与助手
step "5. 校验 KV 已装 COMMIT 与助手 sha256"
vchk="head -1 $(q "$KVBASE/source/COMMIT"); sha256sum $(q "$KVBASE/source/$HELPER") | cut -d' ' -f1; \
test -f $(q "$KVBASE/source/$COMP/kv_ar_tune.json") && echo TUNE_OK; ls -1d $(q "$KVBASE/releases")/*-prev-* 2>/dev/null | tail -1"
if [ "$DRY" = 1 ]; then K "$vchk"; echo "== dry-run 结束，未连板"; exit 0; fi
v=$(K "$vchk" </dev/null)
icommit=$(echo "$v" | sed -n 1p); ihelper=$(echo "$v" | sed -n 2p)
if [ "$icommit" != "$SHA" ] || [ "$ihelper" != "$HSHA" ] || ! echo "$v" | grep -qx TUNE_OK; then
  echo "FAIL 校验没过：已装 COMMIT=$icommit（期望 $SHA），助手 $ihelper（期望 $HSHA）" >&2; exit 7
fi
prevdir=$(echo "$v" | sed -n 4p)
echo "== 完成：KV 已装 $SHA，助手 $HSHA"
[ -n "$prevdir" ] && echo "   回滚（装回上一版，用现装版的安装器）：经 Nano 在 KV 上 sudo bash $KVBASE/source/$COMP/install_kv_service.sh --update $prevdir"
echo "   生效：下次 KV 服务启动（console_remote.sh up / nano_up.sh）起；已在跑的服务仍是旧代码。"
exit 0
