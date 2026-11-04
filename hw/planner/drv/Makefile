# DRV5 规划核驱动：libplanner_drv.so（host / aarch64，给 kv_nav 经 ctypes 用）＋宿主对照＋ qemu-aarch64 同一对照。
HLS_INC ?= /data1/Xilinx/Vitis_HLS/2022.2/include
dev2_COMMIT ?= 68b345d
KERNEL_SHA ?= bb492f658efac761c774dafb228b925ec8267273a93091850234491fb0c028ff
FROZEN ?= ${FPGA_SHARED}/artifacts/20261104-dev2-hls-opt-plannerdual/ledger/evidence/frozen-pair
BUILD ?= build
CROSS ?= aarch64-linux-gnu-
FLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -fPIC
SRC = planner_core.cpp ../../m7/pl/sw/pl_io.cpp
.PHONY: lib check
lib: $(BUILD)/host/libplanner_drv.so $(BUILD)/aarch64/libplanner_drv.so
$(BUILD)/host/libplanner_drv.so: $(SRC) planner_core.hpp
	mkdir -p $(dir $@); g++ $(FLAGS) -shared -o $@ $(SRC)
$(BUILD)/aarch64/libplanner_drv.so: $(SRC) planner_core.hpp
	mkdir -p $(dir $@); $(CROSS)g++ $(FLAGS) -shared -o $@ $(SRC)
$(BUILD)/dev2/kernel_hls.cpp:
	mkdir -p $(dir $@)
	git show $(dev2_COMMIT):hw/planner_dual/src/kernel_hls.cpp > $@
	git show $(dev2_COMMIT):hw/planner_dual/src/kernel_hls.hpp > $(BUILD)/dev2/kernel_hls.hpp
	echo "$(KERNEL_SHA)  $@" | sha256sum -c -
check: lib $(BUILD)/dev2/kernel_hls.cpp
	g++ $(FLAGS) -isystem $(HLS_INC) -I$(BUILD)/dev2 test_planner_drv.cpp $(BUILD)/dev2/kernel_hls.cpp $(SRC) -o $(BUILD)/test_planner_drv
	$(BUILD)/test_planner_drv $(FROZEN)/input.bin $(FROZEN)/expected.bin
	$(CROSS)g++ $(FLAGS) -static -isystem $(HLS_INC) -I$(BUILD)/dev2 test_planner_drv.cpp $(BUILD)/dev2/kernel_hls.cpp $(SRC) -o $(BUILD)/test_planner_drv.a64
	qemu-aarch64-static $(BUILD)/test_planner_drv.a64 $(FROZEN)/input.bin $(FROZEN)/expected.bin
	@echo "PLANNER DRV MAKE CHECK PASS"
