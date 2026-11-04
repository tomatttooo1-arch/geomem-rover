#!/usr/bin/env bash
# tools/check_large_files.sh —— 拦住不该进 git 的大文件与产物文件
#
# 用法：
#   tools/check_large_files.sh                      # 等同 --staged
#   tools/check_large_files.sh --staged             # 检查本次暂存（git add 过）的新增/修改文件
#   tools/check_large_files.sh <路径>...            # 检查磁盘上的指定文件
#   任一形式都可以追加 --allow <路径>（可重复）来豁免某个文件
#
# 判据（命中任一条即拦下）：
#   1) 文件超过 1 MiB（1048576 字节）
#   2) 扩展名在黑名单里（构建产物 / 模型权重 / 录制数据）
#
# 退出码：0 = 全部通过；1 = 有文件被拦下；2 = 用法错误
#
# 被拦下不等于不能提交，而是这个文件该放到
#   ${FPGA_SHARED}/{data,models,artifacts}/
# 仓里只留清单（tools/manifest.sh 生成的 MANIFEST.sha256）和一句指针。
set -euo pipefail

LIMIT=1048576

# 扩展名黑名单 -> 建议去处
suggest_for_ext() {
  case "$1" in
    xclbin|bit|xsa|hwh)                 echo "shared/artifacts/" ;;
    xmodel|gguf|onnx|engine|pth|pt|tflite) echo "shared/models/" ;;
    bag|mcap|svo|pcd|ply|mp4|avi)       echo "shared/data/" ;;
    *)                                  echo "" ;;
  esac
}

usage() {
  sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
}

MODE=""
declare -a ALLOW=()
declare -a ARGPATHS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --staged)
      if [ "$MODE" = "paths" ]; then
        echo "错误：--staged 不能和具体路径混用。" >&2; exit 2
      fi
      MODE=staged; shift ;;
    --allow)
      if [ $# -lt 2 ]; then echo "错误：--allow 后面要跟一个路径。" >&2; exit 2; fi
      ALLOW+=("${2#./}"); shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    --*)
      echo "错误：未知参数 $1" >&2; usage >&2; exit 2 ;;
    *)
      if [ "$MODE" = "staged" ]; then
        echo "错误：--staged 不能和具体路径混用。" >&2; exit 2
      fi
      MODE=paths; ARGPATHS+=("${1#./}"); shift ;;
  esac
done

[ -n "$MODE" ] || MODE=staged

is_allowed() {
  local p=$1 a
  if [ "${#ALLOW[@]}" -gt 0 ]; then
    for a in "${ALLOW[@]}"; do
      [ "$p" = "$a" ] && return 0
    done
  fi
  return 1
}

# 收集待查文件到数组 FILES
declare -a FILES=()
if [ "$MODE" = "staged" ]; then
  if ! git rev-parse --git-dir >/dev/null 2>&1; then
    echo "错误：--staged 模式要在 git 仓库里运行。" >&2; exit 2
  fi
  while IFS= read -r -d '' p; do
    FILES+=("$p")
  done < <(git diff --cached --name-only --no-renames --diff-filter=AM -z)
else
  FILES=("${ARGPATHS[@]}")
fi

size_of() {
  local p=$1
  if [ "$MODE" = "staged" ]; then
    git cat-file -s ":$p"
  else
    stat -c '%s' "$p"
  fi
}

bad=0
allowed=0
for p in "${FILES[@]:-}"; do
  [ -n "$p" ] || continue

  base=${p##*/}
  ext=""
  case "$base" in
    *.*) ext=$(printf '%s' "${base##*.}" | tr 'A-Z' 'a-z') ;;
  esac

  if [ "$MODE" = "paths" ] && [ ! -f "$p" ]; then
    echo "跳过（不是普通文件）：$p" >&2
    continue
  fi

  sz=$(size_of "$p")
  where=$(suggest_for_ext "$ext")

  reason=""
  if [ -n "$where" ]; then
    reason="扩展名 .$ext 在黑名单里"
  fi
  if [ "$sz" -gt "$LIMIT" ]; then
    if [ -n "$reason" ]; then
      reason="$reason，且超过 1 MiB"
    else
      reason="超过 1 MiB"
    fi
    [ -n "$where" ] || where="shared/data/ 或 shared/artifacts/"
  fi

  [ -n "$reason" ] || continue

  if is_allowed "$p"; then
    printf '豁免：%s（%s 字节，%s）\n' "$p" "$sz" "$reason"
    allowed=$((allowed + 1))
    continue
  fi

  if [ "$bad" -eq 0 ]; then
    echo ""
    echo "✗ 以下文件不该进 git："
  fi
  printf '  %s\n' "$p"
  printf '      字节数：%s（%s）\n' "$sz" "$reason"
  printf '      建议去处：%s\n' "$where"
  bad=$((bad + 1))
done

if [ "$allowed" -gt 0 ]; then
  echo "注意：本次豁免了 $allowed 个文件。豁免需在 DECISIONS.md 留号（写清是哪个文件、为什么、谁批的）。"
fi

if [ "$bad" -gt 0 ]; then
  echo ""
  echo "共 $bad 个文件被拦下。把它们放到 ${FPGA_SHARED}/ 下对应目录，"
  echo "用 tools/manifest.sh 生成 MANIFEST.sha256，仓里只提交清单；"
  echo "--allow 仅为本地诊断；中心例外须由 owner 审阅 path/sha256/decision 并部署 policy。"
  echo ""
  exit 1
fi

exit 0
