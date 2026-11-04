#!/usr/bin/env bash
# tools/manifest.sh —— 为一个目录生成 MANIFEST.sha256（逐文件 sha256 清单）
#
# 用法：tools/manifest.sh <dir> [out]
#   <dir>  要登记的目录（清单里的路径都相对它，形如 ./sub/file）
#   [out]  输出文件，默认 <dir>/MANIFEST.sha256
#
# 说明：
#   - 清单正文由 `find -type f -print0 | sort -z | xargs -0 sha256sum` 生成，顺序稳定。
#   - 头部若干 `#` 注释行记录文件数与总字节；`sha256sum -c` 会忽略 `#` 开头的行（已实测，
#     GNU coreutils 8.30，加 --strict 也忽略），所以带注释的清单仍可直接校验。
#   - 清单文件自身不登记在清单里（它在生成之后才写完，登记进去必然自相矛盾）。
#   - 生成后脚本会自己跑一次 `sha256sum -c` 自校验，不通过就 exit 1。
#   - 重新生成时请只比对正文，头部含时间戳，必然不同。
#
# 校验方法：cd <dir> && sha256sum -c MANIFEST.sha256
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
  echo "用法：$0 <dir> [out]" >&2
  exit 2
fi

dir=$1
if [ ! -d "$dir" ]; then
  echo "错误：目录不存在：$dir" >&2
  exit 1
fi
dir_abs=$(cd "$dir" && pwd)

out=${2:-$dir_abs/MANIFEST.sha256}
out_dir=$(cd "$(dirname "$out")" && pwd)
out_abs="$out_dir/$(basename "$out")"

# 如果输出文件落在被登记的目录里面，算出它的相对名，好把它自己排除掉
self_rel=""
case "$out_abs" in
  "$dir_abs"/*) self_rel="./${out_abs#"$dir_abs"/}" ;;
esac

tmp_body=$(mktemp)
tmp_out=$(mktemp)
trap 'rm -f "$tmp_body" "$tmp_out"' EXIT

( cd "$dir_abs" && find . -type f -print0 | sort -z | xargs -0 sha256sum ) > "$tmp_body"

if [ -n "$self_rel" ]; then
  # sha256sum 的输出是 "<64 位十六进制><两个空格><路径>"，路径从第 67 个字符开始
  while IFS= read -r line; do
    [ "${line:66}" = "$self_rel" ] && continue
    printf '%s\n' "$line"
  done < "$tmp_body" > "$tmp_body.f"
  mv "$tmp_body.f" "$tmp_body"
fi

n_files=$(wc -l < "$tmp_body")
n_bytes=$(
  cd "$dir_abs"
  while IFS= read -r line; do
    p=${line:66}
    [ -n "$self_rel" ] && [ "$p" = "$self_rel" ] && continue
    stat -c '%s' "$p"
  done < "$tmp_body" | awk '{s+=$1} END {print s+0}'
)

{
  printf '# MANIFEST.sha256 —— %s\n' "$(basename "$dir_abs")"
  printf '# 生成时间：%s\n' "$(date -Iseconds)"
  printf '# 文件数：%s   总字节：%s\n' "$n_files" "$n_bytes"
  printf '# 校验方法：cd 本目录 && sha256sum -c %s\n' "$(basename "$out_abs")"
  printf '# 以下为正文；重新生成后请只比对正文（头部含时间戳，必然不同）。\n'
  cat "$tmp_body"
} > "$tmp_out"

mv "$tmp_out" "$out_abs"
trap 'rm -f "$tmp_body"' EXIT

# 自校验：立刻用刚写出的清单核一遍
bad=$( (cd "$dir_abs" && sha256sum -c "$out_abs" 2>&1) | grep -vc ': OK$' || true )
if [ "$bad" -ne 0 ]; then
  echo "自校验失败：$out_abs 有 $bad 行不是 OK" >&2
  exit 1
fi

echo "已生成 $out_abs：$n_files 个文件，$n_bytes 字节，自校验通过。"
