#!/usr/bin/env bash
# hlsopt_session.sh — HLS 优化会话的 git 隔离区（团队共享仓库安全版）
#
# 取代 Ross hls-optimize 原文里的 git init / git config / 改 .gitignore / 每次尝试提交到当前分支 /
# git checkout HEAD~1 回退。所有逐次尝试只发生在本地实验分支 exp/<本人>/hls-opt-<模块>-<日期>
# 的专用 worktree 里；收尾时补丁+账本存档到 shared/artifacts，再向用户功能分支做一次提交。
#
# 子命令：
#   start   --module M --files "a.cpp b.h" [--date YYYYMMDD] [--wt-root DIR] [--ledger-dir REL] [--allow-new]
#           （在用户的工作副本里、功能分支上运行；--allow-new 允许列出尚不存在的新文件，供架构流程用）
#   baseline --label TEXT                                        （架构产物首次提交，打 hlsopt/<M>/arch；仅在第一次尝试前）
#   attempt --status pass|fail --label TEXT [--message-file F]   （在实验区运行）
#   good    --bitexact pass                                      （在实验区运行）
#   revert                                                       （在实验区运行）
#   finish  --summary-file F [--best TAG|auto] [--remove-worktree] （在实验区运行）
#   status                                                       （在实验区运行）
#
# 环境变量：HLSOPT_ARCHIVE_ROOT（默认 <artifacts>）
#
# 本脚本从不执行：git init、git config（任何作用域的写入）、修改 .gitignore、git push、
# git reset --hard、git clean、git worktree remove --force、带 --force 的任何命令。
set -euo pipefail

ARCHIVE_ROOT="${HLSOPT_ARCHIVE_ROOT:-<artifacts>}"
PROG="hlsopt_session"

die() { echo "[$PROG] 拒绝：$*" >&2; exit 2; }
info() { echo "[$PROG] $*"; }

in_repo() { git rev-parse --show-toplevel >/dev/null 2>&1 || die "当前目录不在 git 仓库里"; }
cur_branch() { git symbolic-ref --quiet --short HEAD 2>/dev/null || echo ""; }
state_file() { echo "$(git rev-parse --absolute-git-dir)/hlsopt-session.env"; }

load_state() {
  local sf; sf="$(state_file)"
  [[ -f "$sf" ]] || die "这里不是 hlsopt 实验区（缺 $sf）。先在功能分支副本里运行 start，再 cd 到它打印的实验区路径"
  # shellcheck disable=SC1090
  source "$sf"
}

save_state() {
  local sf; sf="$(state_file)"
  {
    printf 'S_MODULE=%q\n' "$S_MODULE"; printf 'S_WHO=%q\n' "$S_WHO"; printf 'S_DATE=%q\n' "$S_DATE"
    printf 'S_FILES=%q\n' "$S_FILES"; printf 'S_LEDGER=%q\n' "$S_LEDGER"
    printf 'S_BRANCH=%q\n' "$S_BRANCH"; printf 'S_ORIGIN_DIR=%q\n' "$S_ORIGIN_DIR"
    printf 'S_ORIGIN_BRANCH=%q\n' "$S_ORIGIN_BRANCH"; printf 'S_WT=%q\n' "$S_WT"
    printf 'S_BASE_TAG=%q\n' "$S_BASE_TAG"; printf 'S_N=%q\n' "$S_N"
    printf 'S_LAST_GOOD=%q\n' "$S_LAST_GOOD"; printf 'S_LAST_STATUS=%q\n' "$S_LAST_STATUS"
    printf 'S_BASELINE=%q\n' "${S_BASELINE:-}"
  } > "$sf"
}

# 实验区前置检查：在 start 建的 worktree 里、分支 = 会话分支、以 exp/<本人>/ 开头、不是 main。
precheck_exp() {
  in_repo
  load_state
  local b; b="$(cur_branch)"
  [[ -n "$b" ]] || die "HEAD 处于游离状态，不在任何分支上"
  [[ "$b" != main && "$b" != master ]] || die "当前在 $b 上；逐次尝试只允许在 exp/ 实验分支上"
  [[ "$b" == exp/* ]] || die "当前分支 $b 不以 exp/ 开头；逐次尝试只允许在实验分支上"
  [[ "$b" == "exp/$S_WHO/"* ]] || die "当前分支 $b 不属于本人（$S_WHO）"
  [[ "$b" == "$S_BRANCH" ]] || die "当前分支 $b 不是本会话的实验分支 $S_BRANCH"
  [[ "$(git rev-parse --show-toplevel)" == "$S_WT" ]] || die "不在本会话实验区 $S_WT 内"
}

existing_files() {  # 会话源文件中当前存在于工作树的
  local f
  for f in $S_FILES; do [[ -e "$S_WT/$f" ]] && echo "$f"; done
  return 0
}

ledger_paths() {  # 只列出存在的账本文件（相对仓库根）
  local f
  for f in "$S_LEDGER/optimize_outcomes.md" "$S_LEDGER/qor_history.csv"; do
    [[ -e "$S_WT/$f" ]] && echo "$f"
  done
  return 0
}

cmd_start() {
  local module="" files="" date="" wtroot="" ledger="" allownew=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --module) module="$2"; shift 2;; --files) files="$2"; shift 2;;
      --date) date="$2"; shift 2;; --wt-root) wtroot="$2"; shift 2;;
      --ledger-dir) ledger="$2"; shift 2;; --allow-new) allownew=1; shift;;
      *) die "start 不认识参数 $1";;
    esac
  done
  [[ -n "$module" && -n "$files" ]] || die "start 需要 --module 与 --files"
  [[ "$module" =~ ^[A-Za-z0-9._-]+$ ]] || die "模块名只能含字母数字 . _ -"
  in_repo
  local top; top="$(git rev-parse --show-toplevel)"; cd "$top"
  [[ -f "$(state_file)" ]] && die "当前目录已是 hlsopt 实验区；start 要在用户自己的功能分支副本里运行"
  local ob; ob="$(cur_branch)"
  [[ -n "$ob" ]] || die "原副本 HEAD 游离；请先切到你的功能分支"
  [[ "$ob" != main && "$ob" != master ]] || die "原副本在 $ob 上；请先切到 feat|fix|exp/<本人>/<主题> 功能分支"
  [[ "$ob" != exp/*/hls-opt-* ]] || die "原副本本身就是 hls-opt 实验分支"
  local who; who="$(git config --get user.name || true)"   # 只读，不写
  [[ -n "$who" ]] || die "仓库没有 user.name（本脚本不会替你设置）"
  [[ "$ob" == */"$who"/* ]] || die "原副本分支 $ob 不属于本人 $who（应为 feat|fix|exp/$who/...）"
  local f
  for f in $files; do
    if ! git ls-files --error-unmatch -- "$f" >/dev/null 2>&1; then
      [[ -e "$f" ]] && die "$f 在原副本里是未跟踪文件（可能是别人未提交的工作），拒绝接管"
      [[ "$allownew" == 1 ]] || die "$f 不是已跟踪文件（相对仓库根 $top）；新文件请加 --allow-new"
    fi
  done
  # shellcheck disable=SC2086
  if [[ -n "$(git status --porcelain -- $files)" ]]; then
    git status --short -- $files >&2
    die "被优化的源文件在原副本里有未提交修改；请先由你自己提交或另存，再开工"
  fi
  date="${date:-$(date +%Y%m%d)}"
  [[ "$date" =~ ^[0-9]{8}$ ]] || die "--date 须为 YYYYMMDD"
  local branch="exp/$who/hls-opt-$module-$date"
  wtroot="${wtroot:-$(dirname "$top")/$(basename "$top")-wt}"
  local wt="$wtroot/hls-opt-$module-$date"
  case "$wt/" in "$top"/*) die "实验区 $wt 不能位于原副本 $top 之内";; esac
  git show-ref --verify --quiet "refs/heads/$branch" && die "分支 $branch 已存在（换 --date 或让用户决定是否删除旧分支）"
  git show-ref --verify --quiet "refs/tags/hlsopt/$module/base" && die "标签 hlsopt/$module/base 已存在（上一次会话的标签由用户决定去留）"
  [[ -e "$wt" ]] && die "实验区路径 $wt 已存在"
  ledger="${ledger:-$(dirname "${files%% *}")/.memory}"; ledger="${ledger#./}"
  mkdir -p "$wtroot"
  git worktree add -b "$branch" "$wt" "$ob" >/dev/null
  cd "$wt"
  mkdir -p "$ledger"
  if [[ ! -e "$ledger/optimize_outcomes.md" ]]; then
    {
      echo "# HLS optimize outcomes — $module"
      echo
      echo "| # | technique | csynth before→after | II | bit-exact csim/cosim | hypothesis confirmed | notes |"
      echo "|---|---|---|---|---|---|---|"
    } > "$ledger/optimize_outcomes.md"
  fi
  git add -- "$ledger/optimize_outcomes.md"
  git commit -q --allow-empty -m "hlsopt($module): base — ledger initialised, sources = $ob@$(git rev-parse --short "$ob")"
  git tag "hlsopt/$module/base"
  S_MODULE="$module"; S_WHO="$who"; S_DATE="$date"; S_FILES="$files"; S_LEDGER="$ledger"
  S_BRANCH="$branch"; S_ORIGIN_DIR="$top"; S_ORIGIN_BRANCH="$ob"; S_WT="$wt"
  S_BASE_TAG="hlsopt/$module/base"; S_N=0; S_LAST_GOOD=""; S_LAST_STATUS=""; S_BASELINE=""
  save_state
  info "实验区已建：$wt"
  info "实验分支：$branch（本地，永不推送）；基线标签：$S_BASE_TAG"
  info "账本目录：$wt/$ledger"
  info "下一步：cd $wt 后在其中建基线、跑 golden，再按 attempt/good/revert 推进"
}

cmd_attempt() {
  local status="" label="" msgf=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --status) status="$2"; shift 2;; --label) label="$2"; shift 2;;
      --message-file) msgf="$2"; shift 2;; *) die "attempt 不认识参数 $1";;
    esac
  done
  [[ "$status" == pass || "$status" == fail ]] || die "attempt 需要 --status pass|fail"
  [[ -n "$label" ]] || die "attempt 需要 --label"
  precheck_exp
  cd "$S_WT"
  local n=$((S_N + 1)) tmp; tmp="$(mktemp)"
  if [[ "$status" == pass ]]; then
    echo "attempt-$n($S_MODULE): $label" > "$tmp"
  else
    echo "experiment-$n($S_MODULE): $label — did not help / failed gate" > "$tmp"
  fi
  if [[ -n "$msgf" ]]; then echo >> "$tmp"; cat "$msgf" >> "$tmp"; fi
  # shellcheck disable=SC2046
  git add -- $(existing_files) $(ledger_paths)
  git commit -q --allow-empty -F "$tmp"; rm -f "$tmp"
  git log -1 --oneline
  # shellcheck disable=SC2086
  if [[ -n "$(git status --porcelain -- $S_FILES $(ledger_paths))" ]]; then
    die "提交后源文件/账本仍有未提交内容——提交没有完整落地，停下排查，不要回退"
  fi
  S_N=$n; S_LAST_STATUS="$status"; save_state
  info "第 $n 次尝试已提交（$status）"
}

cmd_good() {
  local be=""
  while [[ $# -gt 0 ]]; do
    case "$1" in --bitexact) be="$2"; shift 2;; *) die "good 不认识参数 $1";; esac
  done
  [[ "$be" == pass ]] || die "good 需要 --bitexact pass（csim 与已跑的 cosim 都已与 golden 逐位一致）"
  precheck_exp
  cd "$S_WT"
  [[ "$S_N" -ge 1 ]] || die "还没有任何尝试提交"
  [[ "$S_LAST_STATUS" == pass ]] || die "最近一次尝试（#$S_N）记为 fail，不能标为 good"
  # shellcheck disable=SC2046,SC2086
  [[ -z "$(git status --porcelain -- $S_FILES $(ledger_paths))" ]] || die "有未提交改动；先 attempt 提交"
  local tag="hlsopt/$S_MODULE/good-$S_N"
  git show-ref --verify --quiet "refs/tags/$tag" && die "标签 $tag 已存在"
  git tag "$tag"
  S_LAST_GOOD="$tag"; save_state
  info "已打标签 $tag（本地）"
}

cmd_revert() {
  precheck_exp
  cd "$S_WT"
  # shellcheck disable=SC2086
  if [[ -n "$(git status --porcelain -- $S_FILES)" ]]; then
    die "源文件有未提交修改——先用 attempt --status fail 把失败尝试提交，再回退"
  fi
  local target="${S_LAST_GOOD:-${S_BASELINE:-$S_BASE_TAG}}" f
  for f in $S_FILES; do
    if git cat-file -e "$target:$f" 2>/dev/null; then git checkout "$target" -- "$f"
    elif [[ -e "$f" ]]; then git rm -q -- "$f"; fi
  done
  # shellcheck disable=SC2086
  if [[ -n "$(git status --porcelain -- $S_FILES)" ]]; then
    # shellcheck disable=SC2046
    git commit -q -m "hlsopt($S_MODULE): revert sources to $target" -- $(git status --porcelain -- $S_FILES | awk '{print $NF}')
    info "源文件已回到 $target 并提交"
  else
    info "源文件已与 $target 一致，无需回退提交"
  fi
  S_LAST_STATUS="reverted"; save_state
}

cmd_finish() {
  local best="auto" summ="" rmwt=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --best) best="$2"; shift 2;; --summary-file) summ="$2"; shift 2;;
      --remove-worktree) rmwt=1; shift;; *) die "finish 不认识参数 $1";;
    esac
  done
  [[ -n "$summ" && -s "$summ" ]] || die "finish 需要非空 --summary-file（写最终 vs 基线 QoR 三元组：延迟/II/资源，及逐位一致结论）"
  precheck_exp
  cd "$S_WT"
  # shellcheck disable=SC2046,SC2086
  [[ -z "$(git status --porcelain -- $S_FILES $(ledger_paths))" ]] || die "实验区有未提交的源文件/账本改动"
  [[ "$best" == auto ]] && best="$S_LAST_GOOD"
  [[ -n "$best" ]] || die "没有任何 good 标签；没有可交付的结果（实验分支与存档仍可由用户查看）"
  [[ "$best" == "hlsopt/$S_MODULE/good-"* ]] || die "--best 必须是本模块的 good 标签"
  git show-ref --verify --quiet "refs/tags/$best" || die "标签 $best 不存在"

  local f
  for f in $S_FILES; do
    git cat-file -e "$best:$f" 2>/dev/null || die "$f 在 $best 中不存在；删除文件的收尾需要人工处理"
  done

  # ① 存档：补丁 + 账本 + 清单
  local arc="$ARCHIVE_ROOT/$S_DATE-$S_WHO-hls-opt-$S_MODULE"
  [[ -e "$arc" ]] && die "存档目录 $arc 已存在（不覆盖旧产物）"
  mkdir -p "$arc/patches" "$arc/ledger"
  git format-patch -q -o "$arc/patches" "$S_BASE_TAG..HEAD"
  local ncommit npatch
  ncommit="$(git rev-list --count "$S_BASE_TAG..HEAD")"
  npatch="$(find "$arc/patches" -name '*.patch' | wc -l)"
  [[ "$ncommit" == "$npatch" ]] || die "存档补丁数 $npatch ≠ 提交数 $ncommit"
  cp -a "$S_WT/$S_LEDGER/." "$arc/ledger/"
  cp "$summ" "$arc/SUMMARY.txt"
  {
    echo "module=$S_MODULE"; echo "who=$S_WHO"; echo "exp_branch=$S_BRANCH (local only, never pushed)"
    echo "origin_branch=$S_ORIGIN_BRANCH"; echo "base_tag=$S_BASE_TAG $(git rev-parse "$S_BASE_TAG")"
    echo "best_tag=$best $(git rev-parse "$best")"; echo "attempts=$S_N"; echo "commits=$ncommit"
    echo "files=$S_FILES"; echo "created=$(date -Is)"
  } > "$arc/MANIFEST.txt"
  (cd "$arc" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
  (cd "$arc" && sha256sum -c --quiet SHA256SUMS) || die "存档校验失败"
  info "存档完成：$arc（$npatch 个补丁）"

  # ② 回到用户功能分支，一次提交只取最终源码
  local od="$S_ORIGIN_DIR"
  [[ "$(git -C "$od" symbolic-ref --quiet --short HEAD || true)" == "$S_ORIGIN_BRANCH" ]] \
    || die "原副本 $od 当前不在 $S_ORIGIN_BRANCH 上；请用户先切回，再重跑 finish（存档已写好，重跑前把 $arc 改名或删除由用户决定）"
  # shellcheck disable=SC2086
  [[ -z "$(git -C "$od" status --porcelain -- $S_FILES)" ]] \
    || die "原副本里这些源文件出现了未提交修改；不覆盖，请用户处理后重跑"
  # shellcheck disable=SC2086
  git -C "$od" diff --quiet "$S_BASE_TAG" "$S_ORIGIN_BRANCH" -- $S_FILES \
    || die "开工后原功能分支上这些源文件已被别的提交改过；需要人工合并（git -C $od merge --squash $best）"
  local msg; msg="$(mktemp)"
  {
    echo "hls-opt($S_MODULE): squash of $S_N attempts, best=$best"
    echo
    cat "$summ"
    echo
    echo "Attempts: $S_N (commits on local branch $S_BRANCH: $ncommit)"
    echo "Archive: $arc"
  } > "$msg"
  # shellcheck disable=SC2086
  git -C "$od" checkout "$best" -- $S_FILES
  # shellcheck disable=SC2086
  if git -C "$od" diff --cached --quiet -- $S_FILES; then
    rm -f "$msg"; die "最佳标签与基线源码相同，没有可提交的改动"
  fi
  # shellcheck disable=SC2086
  git -C "$od" commit -q -F "$msg" -- $S_FILES
  rm -f "$msg"
  info "已在 $S_ORIGIN_BRANCH 上提交一次：$(git -C "$od" log -1 --oneline)"
  info "实验分支 $S_BRANCH 与 hlsopt/$S_MODULE/* 标签保留在本地，未推送；是否删除由用户决定"

  # ④ 可选：移除实验区（只在存档成功之后；有未跟踪构建产物时不强删）
  if [[ "$rmwt" == 1 ]]; then
    if [[ -n "$(git status --porcelain)" ]]; then
      info "实验区有未跟踪/未提交内容（HLS 构建产物等），不强制移除；请用户确认后自行处理：$S_WT"
    else
      cd "$od"; git worktree remove "$S_WT"; info "实验区已移除：$S_WT"
    fi
  fi
}

cmd_baseline() {
  local label=""
  while [[ $# -gt 0 ]]; do
    case "$1" in --label) label="$2"; shift 2;; *) die "baseline 不认识参数 $1";; esac
  done
  [[ -n "$label" ]] || die "baseline 需要 --label"
  precheck_exp
  cd "$S_WT"
  [[ "$S_N" == 0 && -z "${S_BASELINE:-}" ]] || die "baseline 只能在第一次尝试之前做一次"
  [[ -n "$(existing_files)" ]] || die "还没有任何会话源文件"
  # shellcheck disable=SC2046
  git add -- $(existing_files) $(ledger_paths)
  git commit -q --allow-empty -m "hlsopt($S_MODULE): architect baseline — $label"
  local tag="hlsopt/$S_MODULE/arch"
  git show-ref --verify --quiet "refs/tags/$tag" && die "标签 $tag 已存在"
  git tag "$tag"
  S_BASELINE="$tag"; save_state
  info "架构基线已提交并打标签 $tag；revert 在没有 good 标签时回到它"
}

cmd_status() {
  precheck_exp
  echo "module=$S_MODULE branch=$S_BRANCH attempts=$S_N last=$S_LAST_STATUS last_good=${S_LAST_GOOD:-none}"
  echo "origin=$S_ORIGIN_DIR ($S_ORIGIN_BRANCH)  files=$S_FILES  ledger=$S_LEDGER"
}

sub="${1:-}"; [[ $# -gt 0 ]] && shift
case "$sub" in
  start) cmd_start "$@";; attempt) cmd_attempt "$@";; good) cmd_good "$@";;
  revert) cmd_revert "$@";; baseline) cmd_baseline "$@";; finish) cmd_finish "$@";; status) cmd_status "$@";;
  *) sed -n '2,20p' "$0"; exit 1;;
esac
