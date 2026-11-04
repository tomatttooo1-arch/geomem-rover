#!/usr/bin/env bash
# usage: run_ooc.sh <out> <arch> <uram> <clk_mhz>
OUT=$1; L=$(dirname $OUT)/LEDGER.txt
echo "$(date -Is) START dpu-ooc out=$OUT arch=$2 uram=$3 clk=$4" >> $L
source /data1/Xilinx/Vivado/2022.2/settings64.sh >/dev/null 2>&1
mkdir -p $OUT.run && cd $OUT.run
export LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1
FPGA_DPU_OUT=$OUT FPGA_DPU_IPREPO=${FPGA_SHARED}/dpu_ip_vai30_20261102/extract/DPUCZDX8G_VAI_v3.0/dpu_ip \
FPGA_DPU_ARCH=$2 FPGA_DPU_URAM=$3 FPGA_DPU_CLK_MHZ=$4 FPGA_DPU_THREADS=16 \
 vivado -mode batch -nojournal -source ${FPGA_SRC}/hw/m7/pl/dpu/dpu_ooc.tcl > $OUT.log 2>&1
rc=$?; echo "$(date -Is) EXIT dpu-ooc out=$OUT rc=$rc" >> $L
