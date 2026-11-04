#!/bin/bash
# fetch_upstream_reference.sh [upstream_dir]
# 把上游 Xilinx/ross-ai-assistant 的三个 Vivado skill 与两个示例拷进本目录，恢复 vivado-checks 的完整 reference/。
# 本提交包只带了我们改过的 parse_lint_report.py（630 个上游参考文件未随包分发，按 MIT 许可可自行取回）。
# 用法：
#   bash fetch_upstream_reference.sh                 # 自动 git clone 上游 main（需联网）
#   bash fetch_upstream_reference.sh <已解压的上游目录>  # 离线：指向含 skills/ 与 examples/ 的目录
# 我们 2026-11-02 校准时用的上游归档 sha256：a20046c7253b38dce9b4df8a3e9f93eedf1e6c62dc1686feff492692faab9d23
# 上游 main 之后若有变动，内容可能与我们校准时不同；以 ../calibration_20261102.md 的已知答案重跑一次确认。
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
U=${1:-}
if [ -z "$U" ]; then
  T=$(mktemp -d); git clone --depth 1 https://github.com/Xilinx/ross-ai-assistant "$T/ross"; U=$T/ross
fi
[ -d "$U/skills/vivado-rtl-lint" ] || { echo "not an upstream checkout: $U" >&2; exit 2; }
OURS=$(mktemp); cp "$HERE/vivado-rtl-lint/parse_lint_report.py" "$OURS"
for s in vivado-rtl-lint vivado-timing-methodology-checks vivado-rtl-elaboration-analysis; do
  mkdir -p "$HERE/$s"; cp -r "$U/skills/$s/." "$HERE/$s/"
  # 上游 SKILL.md 改名，防止被智能体当成独立 skill 加载；frontmatter name 加 upstream- 前缀
  mv "$HERE/$s/SKILL.md" "$HERE/$s/upstream_skill.md"
  sed -i -E '0,/^name: */s//name: upstream-/' "$HERE/$s/upstream_skill.md"
done
mkdir -p "$HERE/examples/rtl-lint" "$HERE/examples/timing-methodology-checks"
cp -r "$U/examples/rtl-lint/multi-violation" "$HERE/examples/rtl-lint/" 2>/dev/null || echo "WARN: upstream examples/rtl-lint/multi-violation missing"
cp -r "$U/examples/timing-methodology-checks/multi-violation" "$HERE/examples/timing-methodology-checks/" 2>/dev/null || echo "WARN: upstream examples/timing-methodology-checks/multi-violation missing"
cp "$OURS" "$HERE/vivado-rtl-lint/parse_lint_report.py"   # 用我们改过的 2022.2 兼容版覆盖上游原版
echo "OK: reference restored under $HERE ($(find "$HERE" -type f | wc -l) files)"
