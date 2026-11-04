#!/bin/sh
# SRCHK3/4 搜索核 A53 同算法对照（文档"优化前"列）。板上运行（不用 PL、不加载比特流、不用锁）：
#   ./run_a53_srch_bench.sh <TAG> [passes]
# 依次：阴性对照（翻一位期望镜像，必须 bitexact=false）→ 两份同算法二进制各跑 all。
# 输入是联仿同一份向量 trace_v1.bin（最大实车图）/ trace_v2.bin（小图）；每次调用后与期望 DDR 逐字节比，bitexact=false 时读数作废。
# "优化前"每类操作取两份二进制中位数的较小者（两者输出逐字节相同，只是代码结构不同，见 build_cmds.txt）。
set -e
cd "$(dirname "$0")"
TAG=${1:?tag}; PASSES=${2:-20}
mkdir -p out/$TAG
sha256sum -c MANIFEST.sha256
if sudo -n true 2>/dev/null; then RUN="sudo -n taskset -c 3 chrt -f 50"; else RUN="taskset -c 3"; fi
$RUN ./a53_srch_bench select 1 trace_v2_neg.bin | tee out/$TAG/negative_control.md
grep -q "bitexact=false" out/$TAG/negative_control.md || { echo "阴性对照没报 false：比对失效，停止"; exit 1; }
$RUN ./a53_srch_bench all $PASSES trace_v1.bin trace_v2.bin | tee out/$TAG/results_c2.md
$RUN ./a53_srch_bench_c5 all $PASSES trace_v1.bin trace_v2.bin | tee out/$TAG/results_c5.md
