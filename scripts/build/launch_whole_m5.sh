#!/bin/bash
# Whole-board build of the submitted bitstream (m5: geometry, fusion/cost map, object ROI, detection
# post-processing, pre-processing, planner, DPU B1024, permit). Run from the repository root.
# usage: scripts/build/launch_whole_m5.sh <tag> [lpd_ic samd(default)|sasd]
# Required environment (directories / files produced from the sources in this package):
#   FPGA_IP_REPO     IP repository holding the HLS exports fpga_m7_frame_axi, fpga_m7_rolling_axi,
#                   fpga_m7_object_roi_axi, detpost_top (hw/m7/pl/hls/run_export_hls.tcl,
#                   hw/m7/run_object_roi_hls.tcl, hw/detpost/hls1/run_hls.tcl)
#   FPGA_DPU_IPREPO  DPUCZDX8G IP directory from Vitis AI 3.0 (DPUCZDX8G_VAI_v3.0/dpu_ip)
#   FPGA_PREP_IP     export.zip of the pre-processing core (hw/prep/hls1/run_hls.tcl)
#   FPGA_PLANNER_IP  export.zip of the planner core (hw/planner_dual/m5_68b345d/)
# Optional: FPGA_BUILD_OUT (default ./builds), FPGA_PLANNER_IP_SHA256 (sha256 prefix check of FPGA_PLANNER_IP).
# Output: $FPGA_BUILD_OUT/impl-m5-<tag>/ (bitstream, hwh, reports). Vivado 2022.2.
set -u
TAG=${1:?tag}
LPD_IC=${2:-samd}
: "${FPGA_IP_REPO:?set FPGA_IP_REPO}" "${FPGA_DPU_IPREPO:?set FPGA_DPU_IPREPO}" \
  "${FPGA_PREP_IP:?set FPGA_PREP_IP}" "${FPGA_PLANNER_IP:?set FPGA_PLANNER_IP}"
OUT=${FPGA_BUILD_OUT:-$PWD/builds}
export FPGA_M7_PL_NO_AR=1 FPGA_M7_PL_MEM_DIRECT=1 FPGA_M7_PL_IMPL_STRATEGY=c1 FPGA_M7_PL_CLK1_MHZ=166.67 \
  FPGA_M7_PL_POSTPLACE_WNS_MIN=-0.3 FPGA_M7_PL_DPU_ARCH=1024 FPGA_M7_PL_DPU_URAM=26 FPGA_M7_PL_DPU_MHZ=300 \
  FPGA_M7_PL_DPU_IPREPO=$FPGA_DPU_IPREPO \
  FPGA_M7_PL_DPU_CLK_GATE=1 FPGA_M7_PL_DETPOST=1 \
  FPGA_M7_PL_GEO_FX=1 FPGA_M7_PL_LPD=1 FPGA_M7_PL_LPD_IC=$LPD_IC \
  FPGA_M7_PL_PREP=1 FPGA_M7_PL_PREP_IP=$FPGA_PREP_IP \
  FPGA_M7_PL_PLANNER=1 FPGA_M7_PL_PLANNER_IP=$FPGA_PLANNER_IP \
  FPGA_M7_PL_PLANNER_IP_SHA256=${FPGA_PLANNER_IP_SHA256:-}
mkdir -p "$OUT"
bash hw/m7/pl/tools/launch_build.sh "$OUT/impl-m5-$TAG" "$FPGA_IP_REPO" 100 8
