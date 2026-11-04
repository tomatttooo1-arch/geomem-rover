#!/usr/bin/env bash
# 一键把某个提交部署到车载 Nano（在服务器上跑，10-25）。现行做法的脚本化：
#   git archive <sha> → Nano <root>/console-direct-<sha7>（写 COMMIT＝完整提交号一行，car_up 缺它退出码 8）
#   → 原子切软链 <root>/console-current → 按 --restart 重启 → 打印 console_remote.sh status。
#
#   bash tools/car_run/deploy_nano.sh [--ref origin/main] [--host jetson] [--restart none|gateway|stack]
#
# 规矩：任何一步失败即停，失败时不切软链；同 sha 目录已在（且 COMMIT 对得上）就跳过传输；
# 不自动取板锁，只在开头打印取锁命令；--dry-run 不连 Nano，只按顺序打印每条命令。
# 环境变量（测试用替身）：FPGA_LOCK_SH（取锁脚本路径，只用于打印）、DEPLOY_SSH_OPTS（额外 ssh 选项）。
# 退出码：0 成功；2 参数错；3 解析 ref 失败；4 Nano 不可达；5 远端同名目录存在但 COMMIT 不符；
#         6 传输/解包失败；7 传输后校验失败；8 切软链失败；9 重启失败。
set -uo pipefail

REF=origin/main
HOST=jetson
RESTART=none
ROOT=/home/robot/fpga/dev1
DRY=0
REPO=""
usage() { sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; }
while [ $# -gt 0 ]; do
  case "$1" in
    --ref) REF=${2:?}; shift 2 ;;
    --host) HOST=${2:?}; shift 2 ;;
    --restart) RESTART=${2:?}; shift 2 ;;
    --root) ROOT=${2:?}; shift 2 ;;
    --repo) REPO=${2:?}; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done
case "$RESTART" in none|gateway|stack) ;; *) echo "--restart 只能是 none|gateway|stack（给的是 $RESTART）" >&2; exit 2 ;; esac
case "$ROOT" in /*) ;; *) echo "--root 必须是绝对路径" >&2; exit 2 ;; esac
[ -n "$REPO" ] || REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel 2>/dev/null) \
  || { echo "找不到本地仓（用 --repo 指定）" >&2; exit 2; }

SHA=$(git -C "$REPO" rev-parse --verify -q "$REF^{commit}") || { echo "解析不了 ref：$REF（需要最新请先 git -C $REPO fetch origin）" >&2; exit 3; }
SHORT=${SHA:0:7}
NAME=console-direct-$SHORT
DEST=$ROOT/$NAME
LINK=$ROOT/console-current
TMP=$ROOT/.tmp-$NAME.$$
COMP=ui/geomem/tools/competition
NANO_HELPER=tools/kv_link/ar_helper/build/nano/m7ar_present   # AR2NANO2 车屏送显助手
NANO_MAKE=${DEPLOY_NANO_MAKE:-make}
LOCK_SH=${FPGA_LOCK_SH:-${FPGA_HOME}/tools/lock.sh}
# shellcheck disable=SC2206
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 ${DEPLOY_SSH_OPTS:-} "$HOST")

echo "== 部署 $REF = $SHA 到 $HOST:$DEST（restart=$RESTART$([ $DRY = 1 ] && echo '，dry-run')）"
echo "== 提醒：本脚本不取板锁。动车栈/网关前先取锁（用完 release）："
echo "     bash $LOCK_SH status"
echo "     bash $LOCK_SH acquire orin-nano \"部署 $SHORT\" -u dev1"
echo "     bash $LOCK_SH acquire kv260 \"部署 $SHORT\" -u dev1    # 起车栈会连 KV260"

step() { echo "-- $*"; }
# R "<远端命令串>"：远端命令以一个字符串交给 ssh（远端 shell 解释）；dry-run 只打印
R() {
  if [ "$DRY" = 1 ]; then echo "   ${SSH[*]} '$1'"; return 0; fi
  "${SSH[@]}" "$1" </dev/null
}
# 同上，但 stdin 交给远端（只用于传 tar 流）
R_in() {
  if [ "$DRY" = 1 ]; then echo "   ${SSH[*]} '$1'"; return 0; fi
  "${SSH[@]}" "$1"
}
q() { printf '%q' "$1"; }

# 1. 可达 + 旧软链目标 + 同 sha 目录是否已在
step "1. 查 Nano 现状"
OLD=""; HAVE=""
if [ "$DRY" = 1 ]; then
  R "hostname; readlink $(q "$LINK"); cat $(q "$DEST/COMMIT")"
  echo "   （dry-run：若远端 $NAME 已在且 COMMIT=$SHA 则跳过第 2 步）"
else
  out=$(R "hostname >/dev/null && echo REACH; echo OLD=\$(readlink $(q "$LINK") 2>/dev/null); \
if [ -e $(q "$DEST") ]; then echo HAVE=\$(head -1 $(q "$DEST/COMMIT") 2>/dev/null || echo MISSING); else echo HAVE=; fi")
  rc=$?
  echo "$out" | grep -qx REACH || { echo "FAIL Nano 不可达（$HOST，ssh 返回 $rc）；未做任何改动" >&2; exit 4; }
  OLD=$(echo "$out" | sed -n 's/^OLD=//p')
  HAVE=$(echo "$out" | sed -n 's/^HAVE=//p')
  echo "   现在 console-current -> ${OLD:-（无）}"
fi

# 2. 传输
if [ "$DRY" = 0 ] && [ -n "$HAVE" ]; then
  if [ "$HAVE" = "$SHA" ]; then
    step "2. 远端已有 $NAME 且 COMMIT 一致，跳过传输"
  else
    echo "FAIL 远端 $DEST 已存在但 COMMIT 是「$HAVE」不是 $SHA；不覆盖，请人工查看。软链未动" >&2; exit 5
  fi
else
  step "2. git archive $SHORT（＋交叉编的车屏送显助手 $NANO_HELPER）→ $HOST:$TMP，写 COMMIT，改名为 $NAME"
  # AR2NANO2（11-02）：车屏改在 Nano 上画，送显助手（静态 aarch64）在宿主交叉编后跟在 git archive 后面一起传
  # （tar -i 读拼接的两段）。编不出来只告警、照常部署（车屏只是显示；owner 10-26 不加新的拒启）。
  remote_x="set -e; mkdir -p $(q "$ROOT"); mkdir $(q "$TMP"); tar -x -i -C $(q "$TMP"); \
echo $SHA > $(q "$TMP/COMMIT"); mv -T $(q "$TMP") $(q "$DEST")"
  if [ "$DRY" = 1 ]; then
    echo "   git -C $(q "$REPO") archive $SHA tools/kv_link/ar_helper hw/m7 | tar -x -C <hb>; $NANO_MAKE -C <hb>/tools/kv_link/ar_helper BUILD=build nano"
    printf '   { git -C %q archive --format=tar %s; tar -C <hb> -cf - %s; } | ' "$REPO" "$SHA" "$NANO_HELPER"; R_in "$remote_x"
  else
    HB=$(mktemp -d "${TMPDIR:-/tmp}/deploy_nano_helper.XXXXXX") || exit 6
    trap 'rm -rf "$HB" "$HB.make.log"' EXIT
    HOK=0
    if git -C "$REPO" archive --format=tar "$SHA" tools/kv_link/ar_helper hw/m7 2>/dev/null | tar -x -C "$HB" 2>/dev/null \
       && $NANO_MAKE -C "$HB/tools/kv_link/ar_helper" BUILD=build nano >"$HB.make.log" 2>&1 && test -x "$HB/$NANO_HELPER"; then
      HOK=1; echo "   车屏送显助手 sha256 $(sha256sum "$HB/$NANO_HELPER" | cut -d' ' -f1)"
    else
      echo "WARNING 车屏送显助手没编出来（$HB.make.log）：照常部署，车屏 CAR_AR_NANO=1 时会黑（AR_NANO_FAILED）" >&2
    fi
    { git -C "$REPO" archive --format=tar "$SHA"; ga=$?; [ "$HOK" = 1 ] && tar -C "$HB" -cf - "$NANO_HELPER"; exit $ga; } | R_in "$remote_x"
    rc=("${PIPESTATUS[@]}")
    if [ "${rc[0]}" != 0 ] || [ "${rc[1]}" != 0 ]; then
      echo "FAIL 传输/解包失败（git archive ${rc[0]}，ssh ${rc[1]}）；软链未动。清理临时目录：" >&2
      R "rm -rf $(q "$TMP")" >&2 && echo "   已删 $TMP" >&2
      exit 6
    fi
  fi
fi

# 3. 校验（切软链前的门）
step "3. 校验 $NAME/COMMIT 与关键脚本"
chk="test \"\$(head -1 $(q "$DEST/COMMIT"))\" = $SHA && test -f $(q "$DEST/$COMP/console_remote.sh") && test -f $(q "$DEST/tools/car_run/car_up.sh")"
if [ "$DRY" = 1 ]; then R "$chk"
else R "$chk" || { echo "FAIL 校验没过（COMMIT 不符或缺 console_remote.sh / car_up.sh）；软链未动" >&2; exit 7; }; fi

# 4. 切软链（原子：建临时链再 mv -T）
step "4. 切软链 console-current -> $NAME"
if [ "$DRY" = 0 ] && [ "$OLD" = "$NAME" ]; then
  echo "   已指向 $NAME，不用切"
else
  R "ln -sfn $(q "$NAME") $(q "$LINK.new.$$") && mv -Tf $(q "$LINK.new.$$") $(q "$LINK")" \
    || { echo "FAIL 切软链失败" >&2; exit 8; }
  if [ "$DRY" = 0 ]; then
    if [ -n "$OLD" ]; then
      echo "   回滚命令（切回旧版本 $OLD，之后按需同样 --restart）："
      echo "     ssh $HOST $(q "ln -sfn $OLD $LINK.new && mv -Tf $LINK.new $LINK")"
    else
      echo "   原先没有 console-current；回滚＝删掉软链：ssh $HOST $(q "rm $LINK")"
    fi
  fi
fi

# 5. 重启
CR=$LINK/$COMP/console_remote.sh
GW=$LINK/$COMP/nano_gateway.sh
case "$RESTART" in
  none)
    step "5. 不重启（--restart none）：正在跑的车栈和网关仍是旧代码，要生效用 --restart gateway|stack" ;;
  gateway)
    step "5. 重启网关（车栈不动）"
    R "timeout 60 bash $(q "$GW") stop && timeout 60 bash $(q "$GW") start" || { echo "FAIL 网关重启失败（软链已切，见上方回滚命令）" >&2; exit 9; } ;;
  stack)
    step "5. 重启车栈（down 再 up，含网关；最长约 6 分钟）"
    R "timeout 200 bash $(q "$CR") down && timeout 480 bash $(q "$CR") up" || { echo "FAIL 车栈重启失败（软链已切，见上方回滚命令）" >&2; exit 9; } ;;
esac

# 6. 状态
step "6. console_remote.sh status"
R "bash $(q "$CR") status"
[ "$DRY" = 1 ] && echo "== dry-run 结束，未连 Nano" || echo "== 完成：$HOST:$LINK -> $NAME（$SHA）"
exit 0
