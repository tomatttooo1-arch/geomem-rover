# DRV5 规划核驱动：libplanner_drv.so（host / aarch64，给 kv_nav 经 ctypes 用）＋宿主对照＋ qemu-aarch64 同一对照。
# GMAP3（11-04）：mock 里跑的核换成本树的 hw/planner_dual/src/kernel_hls.cpp（15 个参数，gmap_op 0x98、path_ctrl 0x90），
# 两组地图向量（vecA/vecB）经驱动每个操作后地图区 = C 模型（tools/gmap/gmap_model.c）。
HLS_INC ?= /data1/Xilinx/Vitis_HLS/2022.2/include
KSRC ?= ../../planner_dual/src
KERNEL_SHA ?= 17079dce6dac911b27f8dba72f433f3c43d49ea0de57620a176d84d97937c90c
GMAP_DIR ?= ../../../tools/gmap
PATH_PARAMS ?= ../../planner_dual/tb/path_params.bin
FROZEN ?= ${FPGA_SHARED}/artifacts/20261104-dev2-hls-opt-plannerdual/ledger/evidence/frozen-pair
VEC_DIR ?= ${FPGA_RUN}/GMAP3/evidence/vectors
LEGACY_SHA = 0036fda9069bfb257c7c9680d305a27ba0c8e5d90f612555af90eae260ec60b1
PATH_SHA = 86b2d252a0421250f2a0992c6ec9b88d363688783f97536710f62142dae770af
BUILD ?= build
CROSS ?= aarch64-linux-gnu-
FLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -fPIC
SRC = planner_core.cpp ../../m7/pl/sw/pl_io.cpp
TARGS = $(FROZEN)/input.bin $(FROZEN)/expected.bin $(PATH_PARAMS) $(VEC_DIR)/vecA.bin $(VEC_DIR)/vecB.bin
.PHONY: lib check
lib: $(BUILD)/host/libplanner_drv.so $(BUILD)/aarch64/libplanner_drv.so
$(BUILD)/host/libplanner_drv.so: $(SRC) planner_core.hpp
	mkdir -p $(dir $@); g++ $(FLAGS) -shared -o $@ $(SRC)
$(BUILD)/aarch64/libplanner_drv.so: $(SRC) planner_core.hpp
	mkdir -p $(dir $@); $(CROSS)g++ $(FLAGS) -shared -o $@ $(SRC)
check: lib
	echo "$(KERNEL_SHA)  $(KSRC)/kernel_hls.cpp" | sha256sum -c -
	mkdir -p $(BUILD)/out_host $(BUILD)/out_a64
	g++ $(FLAGS) -isystem $(HLS_INC) -I$(KSRC) -I$(GMAP_DIR) test_planner_drv.cpp $(KSRC)/kernel_hls.cpp $(SRC) -o $(BUILD)/test_planner_drv
	$(BUILD)/test_planner_drv $(TARGS) $(BUILD)/out_host
	echo "$(LEGACY_SHA)  $(BUILD)/out_host/drv_legacy_outputs.bin" | sha256sum -c -
	echo "$(PATH_SHA)  $(BUILD)/out_host/drv_path_outputs.bin" | sha256sum -c -
	$(CROSS)g++ $(FLAGS) -static -isystem $(HLS_INC) -I$(KSRC) -I$(GMAP_DIR) test_planner_drv.cpp $(KSRC)/kernel_hls.cpp $(SRC) -o $(BUILD)/test_planner_drv.a64
	qemu-aarch64-static $(BUILD)/test_planner_drv.a64 $(TARGS) $(BUILD)/out_a64
	echo "$(LEGACY_SHA)  $(BUILD)/out_a64/drv_legacy_outputs.bin" | sha256sum -c -
	echo "$(PATH_SHA)  $(BUILD)/out_a64/drv_path_outputs.bin" | sha256sum -c -
	@echo "PLANNER DRV MAKE CHECK PASS"
