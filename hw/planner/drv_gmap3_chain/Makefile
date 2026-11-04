# PLNCHAIN0：GMAP3 规划核接进 kv_nav（PLN=gmap3）。
#   make lib    → $(BUILD)/{host,aarch64}/libplanner_drv_gmap3.so（GMAP3 驱动快照 + kv_nav 薄壳，aarch64 为交叉编译）
#   make check  → 宿主对照：旧驱动跑 GMAP3 核（问题 1）＋薄壳关键对照（x86 与 qemu-aarch64 各一次）
HLS_INC ?= /data1/Xilinx/Vitis_HLS/2022.2/include
KSRC ?= ../../planner_dual/src
KERNEL_SHA ?= 17079dce6dac911b27f8dba72f433f3c43d49ea0de57620a176d84d97937c90c
FROZEN ?= ${FPGA_SHARED}/artifacts/20261104-dev2-hls-opt-plannerdual/ledger/evidence/frozen-pair
BUILD ?= build
CROSS ?= aarch64-linux-gnu-
FLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -fPIC
INC = -I../../../tools/kv_nav -I../../planner_dual/path
PLIO = ../../m7/pl/sw/pl_io.cpp
G3SRC = ../drv_gmap3/planner_core.cpp kv_nav_gmap3.cpp $(PLIO)
.PHONY: lib check mocklib
lib: $(BUILD)/host/libplanner_drv_gmap3.so $(BUILD)/aarch64/libplanner_drv_gmap3.so
$(BUILD)/host/libplanner_drv_gmap3.so: $(G3SRC) ../drv_gmap3/planner_core.hpp
	mkdir -p $(dir $@); g++ $(FLAGS) $(INC) -shared -o $@ $(G3SRC) -lpthread
$(BUILD)/aarch64/libplanner_drv_gmap3.so: $(G3SRC) ../drv_gmap3/planner_core.hpp
	mkdir -p $(dir $@); $(CROSS)g++ $(FLAGS) $(INC) -shared -o $@ $(G3SRC) -lpthread
check: lib
	echo "$(KERNEL_SHA)  $(KSRC)/kernel_hls.cpp" | sha256sum -c -
	g++ $(FLAGS) -isystem $(HLS_INC) -I$(KSRC) -I../drv test_old_drv_on_g3.cpp ../drv/planner_core.cpp $(KSRC)/kernel_hls.cpp $(PLIO) -o $(BUILD)/test_old_drv_on_g3
	$(BUILD)/test_old_drv_on_g3 $(FROZEN)/input.bin $(FROZEN)/expected.bin
	g++ $(FLAGS) $(INC) -isystem $(HLS_INC) -I$(KSRC) -I../../../tools/gmap test_chain.cpp $(G3SRC) $(KSRC)/kernel_hls.cpp -o $(BUILD)/test_chain -lpthread
	$(BUILD)/test_chain $(FROZEN)/input.bin $(FROZEN)/expected.bin
	$(CROSS)g++ $(FLAGS) $(INC) -static -isystem $(HLS_INC) -I$(KSRC) -I../../../tools/gmap test_chain.cpp $(G3SRC) $(KSRC)/kernel_hls.cpp -o $(BUILD)/test_chain.a64 -lpthread
	qemu-aarch64-static $(BUILD)/test_chain.a64 $(FROZEN)/input.bin $(FROZEN)/expected.bin
	@echo "PLNCHAIN0 MAKE CHECK PASS"
# 宿主端到端测试用（tests/test_plnchain0_gmap3_20261104.py）：planner_g3_open 开 mock 上的 GMAP3 核 C 源，不上板
mocklib: $(BUILD)/host/libplanner_drv_gmap3_mock.so
$(BUILD)/host/libplanner_drv_gmap3_mock.so: $(G3SRC) kv_nav_gmap3.cpp mock_g3.hpp
	mkdir -p $(dir $@); g++ $(FLAGS) $(INC) -DPLNCHAIN_MOCK_OPEN -isystem $(HLS_INC) -I$(KSRC) -shared -o $@ $(G3SRC) $(KSRC)/kernel_hls.cpp -lpthread
