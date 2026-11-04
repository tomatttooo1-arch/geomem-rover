# PLNPATH0 host-side pieces (no HLS needed):
#   make test   -> key comparison 1 (C model + PS half == PS product extraction), writes ../tb/path_params.bin
#   make a53    -> static aarch64 timing program for the PS extraction (A53 "before" column)
# planner_split.hpp is taken from the car version of kv_nav straight from git, never copied by hand.
BUILD ?= build
CXX ?= g++
CROSS ?= aarch64-linux-gnu-
KVNAV_REF ?= feat/dev1/nat-ctrlps6-20261104
FROZEN ?= ${FPGA_SHARED}/artifacts/20261104-dev2-hls-opt-plannerdual/ledger/evidence/frozen-pair
VECTORS ?= ${FPGA_SRC}/runs/planner-hwref-20261103/vectors
CASES ?= ${FPGA_SRC}/tools/kv_nav/build/cases
CXXFLAGS := -std=c++14 -O2 -Wall -Wextra -Werror -ffp-contract=off -I$(BUILD)/kvnav -I.
HDR := path_model.hpp path_ps.hpp $(BUILD)/kvnav/planner_split.hpp

$(BUILD)/kvnav/planner_split.hpp:
	mkdir -p $(dir $@); git show $(KVNAV_REF):tools/kv_nav/planner_split.hpp > $@
	sha256sum $@

$(BUILD)/host/test_path_model: test_path_model.cpp $(HDR)
	mkdir -p $(dir $@); $(CXX) $(CXXFLAGS) -o $@ $<

test: $(BUILD)/host/test_path_model
	cd $(BUILD) && ./host/test_path_model $(FROZEN) $(VECTORS) $(CASES) ../../tb/path_params.bin

$(BUILD)/aarch64/a53_path_bench: a53_path_bench.cpp $(HDR)
	mkdir -p $(dir $@); $(CROSS)g++ $(CXXFLAGS) -O3 -mcpu=cortex-a53 -static -o $@ $<
$(BUILD)/host/a53_path_bench: a53_path_bench.cpp $(HDR)
	mkdir -p $(dir $@); $(CXX) $(CXXFLAGS) -O3 -o $@ $<
a53: $(BUILD)/aarch64/a53_path_bench
.PHONY: test a53
# side check: kernel source compiled natively over the 80-grid requests (tolerance branch coverage)
$(BUILD)/host/test_kernel_native: test_kernel_native.cpp ../src/kernel_hls.cpp ../src/kernel_hls.hpp $(HDR)
	mkdir -p $(dir $@); $(CXX) -std=c++14 -O2 -ffp-contract=off -I$(BUILD)/kvnav -I. -I../src -I/data1/Xilinx/Vitis_HLS/2022.2/include -o $@ $< ../src/kernel_hls.cpp
native: $(BUILD)/host/test_kernel_native
	./$(BUILD)/host/test_kernel_native $(VECTORS) $(CASES)
