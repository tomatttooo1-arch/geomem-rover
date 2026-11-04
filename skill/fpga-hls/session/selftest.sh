#!/usr/bin/env bash
# selftest.sh — hlsopt_session.sh 的全流程自测（临时 git 仓库，不碰任何真实仓库）
# 覆盖：start → attempt(fail) → revert → attempt(pass) → good → finish
# 阳性对照：①原副本源文件有未提交修改 ⇒ start 必须拒绝；②非 exp/ 分支上 attempt ⇒ 必须拒绝；
#           ③原副本在 main 上 ⇒ start 必须拒绝；④未提交失败尝试就 revert ⇒ 必须拒绝；
#           ⑤失败尝试后 good ⇒ 必须拒绝。
# 另检查：自测前后临时仓库本地 git config 的 user.name / user.email 未变；没有任何 remote；
#         实验分支与标签保留；功能分支只多了一个提交。
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
S="$HERE/hlsopt_session.sh"
T="$(mktemp -d "${TMPDIR:-/tmp}/hlsopt-selftest.XXXXXX")"
export HLSOPT_ARCHIVE_ROOT="$T/artifacts"
pass=0; fail=0
ok()  { echo "PASS  $*"; pass=$((pass+1)); }
bad() { echo "FAIL  $*"; fail=$((fail+1)); }
expect_ok()   { local d="$1"; shift; if "$@" >"$T/last.log" 2>&1; then ok "$d"; else bad "$d"; sed 's/^/      /' "$T/last.log"; fi; }
expect_fail() { local d="$1"; shift; if "$@" >"$T/last.log" 2>&1; then bad "$d（本应拒绝却成功）"; else
                  if grep -q "拒绝" "$T/last.log"; then ok "$d —— $(grep -m1 "拒绝" "$T/last.log" | sed "s/^.*拒绝：//")"; else bad "$d（失败但不是脚本的拒绝）"; sed 's/^/      /' "$T/last.log"; fi; fi; }

R="$T/work/alice"; mkdir -p "$R"; cd "$R"
git init -q -b main . 2>/dev/null || { git init -q .; git checkout -q -b main; }
git config --local user.name alice           # 仅自测夹具：模拟 new_clone.sh 建好的副本身份
git config --local user.email alice@example.org
mkdir -p hw/k
printf 'int k(int x){return x*3+1;}\n' > hw/k/kernel.cpp
printf 'int main(){return 0;}\n' > hw/k/tb.cpp
git add hw && git commit -q -m init
CFG_BEFORE="$(git config --local --get user.name)|$(git config --local --get user.email)"

# ③ main 上 start 必须拒绝
expect_fail "对照③ 原副本在 main 上 ⇒ start 拒绝" bash "$S" start --module k --files hw/k/kernel.cpp --date 20261102
git checkout -q -b feat/alice/k
# ① 未提交修改 ⇒ start 拒绝
printf '// dirty\n' >> hw/k/kernel.cpp
expect_fail "对照① 源文件有未提交修改 ⇒ start 拒绝" bash "$S" start --module k --files hw/k/kernel.cpp --date 20261102
git checkout -q -- hw/k/kernel.cpp
[[ -z "$(git worktree list | sed -n 2p)" ]] && ok "对照① 被拒后没有建出实验区" || bad "对照① 被拒后仍建了实验区"

expect_ok "start" bash "$S" start --module k --files hw/k/kernel.cpp --date 20261102
WT="$T/work/alice-wt/hls-opt-k-20261102"
[[ -d "$WT" ]] && ok "实验区建在原副本外的同级目录 $WT" || bad "实验区路径不对"
cd "$WT"
[[ "$(git symbolic-ref --short HEAD)" == exp/alice/hls-opt-k-20261102 ]] && ok "实验分支名 exp/alice/hls-opt-k-20261102" || bad "实验分支名不对"
git rev-parse -q --verify refs/tags/hlsopt/k/base >/dev/null && ok "基线标签 hlsopt/k/base" || bad "缺基线标签"
BASE_SRC="$(cat hw/k/kernel.cpp)"

# 尝试 1：失败
printf 'int k(int x){return x*3+2;}\n' > hw/k/kernel.cpp
expect_fail "对照④ 未提交失败尝试就 revert ⇒ 拒绝" bash "$S" revert
expect_ok "attempt #1 (fail)" bash "$S" attempt --status fail --label "bad idea"
expect_fail "对照⑤ 失败尝试后 good ⇒ 拒绝" bash "$S" good --bitexact pass
expect_ok "revert" bash "$S" revert
[[ "$(cat hw/k/kernel.cpp)" == "$BASE_SRC" ]] && ok "revert 后源文件回到 base 标签内容" || bad "revert 后内容不对"
git log --format=%s | grep -q "experiment-1(k)" && ok "失败尝试留在实验分支历史里" || bad "失败尝试未进历史"

# ② 非 exp/ 分支上 attempt ⇒ 拒绝
git checkout -q -b feat/alice/oops
printf '// x\n' >> hw/k/kernel.cpp
expect_fail "对照② 非 exp/ 分支上 attempt ⇒ 拒绝" bash "$S" attempt --status pass --label x
git checkout -q -- hw/k/kernel.cpp; git checkout -q exp/alice/hls-opt-k-20261102; git branch -q -D feat/alice/oops

# 尝试 2：成功
printf 'int k(int x){return (x<<1)+x+1;}\n' > hw/k/kernel.cpp
printf '| 2 | shift-add | 10→8 | 1→1 | yes/yes | yes | |\n' >> hw/k/.memory/optimize_outcomes.md
expect_ok "attempt #2 (pass)" bash "$S" attempt --status pass --label "shift-add"
expect_ok "good" bash "$S" good --bitexact pass
git rev-parse -q --verify refs/tags/hlsopt/k/good-2 >/dev/null && ok "good 标签 hlsopt/k/good-2" || bad "缺 good 标签"
GOOD_SRC="$(cat hw/k/kernel.cpp)"

printf 'Latency: 10 -> 8 cycles; II: 1 -> 1; LUT 120->100 FF 90->90 DSP 1->0\nBit-exact: csim yes, cosim yes\n' > "$T/summary.txt"
ORIG_N_BEFORE="$(git -C "$R" rev-list --count feat/alice/k)"
expect_ok "finish" bash "$S" finish --summary-file "$T/summary.txt" --remove-worktree
ARC="$T/artifacts/20261102-alice-hls-opt-k"
NP="$(find "$ARC/patches" -name '*.patch' 2>/dev/null | wc -l)"
NC="$(git -C "$R" rev-list --count hlsopt/k/base..exp/alice/hls-opt-k-20261102)"
[[ "$NP" -ge 1 && "$NP" == "$NC" ]] && ok "存档补丁数 = 实验分支提交数（$NP）" || bad "存档补丁数 $NP ≠ 提交数 $NC"
[[ -f "$ARC/ledger/optimize_outcomes.md" && -f "$ARC/SHA256SUMS" ]] && ok "账本与 SHA256SUMS 已存档" || bad "存档不完整"
(cd "$ARC" && sha256sum -c --quiet SHA256SUMS) && ok "存档 sha256 校验通过" || bad "存档 sha256 不符"
cd "$R"
[[ "$(git rev-list --count feat/alice/k)" == $((ORIG_N_BEFORE+1)) ]] && ok "功能分支只多一个提交" || bad "功能分支提交数不对"
[[ "$(cat hw/k/kernel.cpp)" == "$GOOD_SRC" ]] && ok "功能分支源码 = 最佳 good 标签" || bad "功能分支源码不对"
git show --stat HEAD | grep -q ".memory" && bad "账本被带进功能分支" || ok "账本未进入功能分支（只取源码）"
git log -1 --format=%B | grep -q "Archive: $ARC" && git log -1 --format=%B | grep -q "Attempts: 2" && ok "提交信息含尝试次数与存档路径" || bad "提交信息缺字段"
git show-ref --verify -q refs/heads/exp/alice/hls-opt-k-20261102 && ok "实验分支保留在本地" || bad "实验分支丢了"
[[ -z "$(git remote)" ]] && ok "没有 remote、没有推送" || bad "出现了 remote"
[[ ! -d "$WT" ]] && ok "存档成功后实验区已移除（--remove-worktree）" || bad "实验区未移除"

# ---- 第二段：架构流程（新文件 + baseline）----
cd "$R"
printf 'scratch\n' > hw/k/arch_kernel.cpp          # 原副本里一个未跟踪的同名文件
expect_fail "对照⑥ 新文件在原副本里已是未跟踪文件 ⇒ start 拒绝" bash "$S" start --module a --files hw/k/arch_kernel.cpp --date 20261102 --allow-new
rm -f hw/k/arch_kernel.cpp
expect_fail "对照⑦ 新文件不加 --allow-new ⇒ start 拒绝" bash "$S" start --module a --files hw/k/arch_kernel.cpp --date 20261102
expect_ok "start --allow-new（架构）" bash "$S" start --module a --files hw/k/arch_kernel.cpp --date 20261102 --allow-new
WT2="$T/work/alice-wt/hls-opt-a-20261102"; cd "$WT2"
printf 'void top(){/*arch v1*/}\n' > hw/k/arch_kernel.cpp
expect_ok "baseline（架构产物）" bash "$S" baseline --label "architect v1"
git rev-parse -q --verify refs/tags/hlsopt/a/arch >/dev/null && ok "架构基线标签 hlsopt/a/arch" || bad "缺 arch 标签"
ARCH_SRC="$(cat hw/k/arch_kernel.cpp)"
printf 'void top(){/*broken*/}\n' > hw/k/arch_kernel.cpp
expect_ok "attempt #1 (fail, 架构)" bash "$S" attempt --status fail --label broken
expect_ok "revert（架构）" bash "$S" revert
[[ "$(cat hw/k/arch_kernel.cpp)" == "$ARCH_SRC" ]] && ok "无 good 时 revert 回到 arch 标签" || bad "revert 目标不对"
printf 'void top(){/*arch v1 + pipeline*/}\n' > hw/k/arch_kernel.cpp
expect_ok "attempt #2 (pass, 架构)" bash "$S" attempt --status pass --label pipeline
expect_ok "good（架构）" bash "$S" good --bitexact pass
expect_ok "finish（架构，保留实验区）" bash "$S" finish --summary-file "$T/summary.txt"
cd "$R"
[[ "$(cat hw/k/arch_kernel.cpp 2>/dev/null)" == 'void top(){/*arch v1 + pipeline*/}' ]] && ok "新文件以最终版本进入功能分支" || bad "新文件未进入功能分支"
[[ -d "$WT2" ]] && ok "未加 --remove-worktree 时实验区保留" || bad "实验区被意外移除"
CFG_AFTER="$(git config --local --get user.name)|$(git config --local --get user.email)"
[[ "$CFG_BEFORE" == "$CFG_AFTER" ]] && ok "本地 user.name/email 未变（$CFG_AFTER）" || bad "本地身份被改：$CFG_BEFORE → $CFG_AFTER"
[[ ! -e .gitignore ]] && ok "没有创建/修改 .gitignore" || bad ".gitignore 被动过"
if grep -nE '^[^#]*git (init|config [^-]*--(global|local|system)? *[a-z.]+ [^ ]|push|clean|reset --hard)' "$S" | grep -v 'config --get' >/dev/null; then
  bad "hlsopt_session.sh 里出现禁用 git 命令"; else ok "hlsopt_session.sh 不含 git init/config 写/push/clean/reset --hard"; fi

echo "----"; echo "selftest: $pass passed, $fail failed  (tmp=$T)"
[[ "$fail" == 0 ]] && rm -rf "$T"
exit $(( fail > 0 ))
