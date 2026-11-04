# kv_nav core library (CTRLPS1). Targets:
#   make host      -> build/host/libkv_nav.so (+ tests, bench, sim)        x86-64
#   make aarch64   -> build/aarch64/libkv_nav.so + static test/bench/sim    KV260 A53 (cross g++ 9.4)
#   make test      -> host key tests;  make test-aarch64 -> the same binaries under qemu-aarch64-static
# On the KV260 itself: make host CXX=g++ builds natively (Ubuntu 22.04 g++), same flags.
# g++ would otherwise fuse multiply-adds (different rounding).
BUILD ?= build
CXX ?= g++
CROSS ?= aarch64-linux-gnu-
dev2_GOLDEN ?= 21069ec
VECTORS ?= ${FPGA_SRC}/runs/planner-hwref-20261103/vectors
CASES ?= $(BUILD)/cases
CXXFLAGS := -std=c++14 -O2 -Wall -Wextra -Werror -ffp-contract=off -fPIC
SRC := kv_nav.cpp kv_nav_c.cpp
HDR := kv_nav.hpp planner_split.hpp smooth_v3.hpp ref/planner_hw_ref.hpp
REF := $(BUILD)/refmodel/model_v3.cpp $(BUILD)/refmodel/model_v3.hpp $(BUILD)/refmodel/model_v2.hpp

host: $(BUILD)/host/libkv_nav.so $(BUILD)/host/test_planner_golden $(BUILD)/host/test_state_machine \
      $(BUILD)/host/kv_nav_bench $(BUILD)/host/kv_nav_sim
aarch64: $(BUILD)/aarch64/libkv_nav.so $(BUILD)/aarch64/test_planner_golden $(BUILD)/aarch64/test_state_machine \
      $(BUILD)/aarch64/kv_nav_bench $(BUILD)/aarch64/kv_nav_sim

$(BUILD)/refmodel/%: | $(BUILD)/refmodel
	git show $(dev2_GOLDEN):tools/planner_model/$* > $@
$(BUILD)/refmodel:
	mkdir -p $@

$(BUILD)/host/libkv_nav.so: $(SRC) $(HDR)
	mkdir -p $(dir $@); $(CXX) $(CXXFLAGS) -shared -o $@ $(SRC)
$(BUILD)/aarch64/libkv_nav.so: $(SRC) $(HDR)
	mkdir -p $(dir $@); $(CROSS)g++ $(CXXFLAGS) -shared -o $@ $(SRC)

$(BUILD)/host/test_planner_golden: tests/test_planner_golden.cpp $(HDR) $(REF)
	mkdir -p $(dir $@); $(CXX) $(CXXFLAGS) -I$(BUILD)/refmodel -o $@ $< $(BUILD)/refmodel/model_v3.cpp
$(BUILD)/aarch64/test_planner_golden: tests/test_planner_golden.cpp $(HDR) $(REF)
	mkdir -p $(dir $@); $(CROSS)g++ $(CXXFLAGS) -static -I$(BUILD)/refmodel -o $@ $< $(BUILD)/refmodel/model_v3.cpp

$(BUILD)/host/%: tests/%.cpp $(SRC) $(HDR)
	mkdir -p $(dir $@); $(CXX) $(CXXFLAGS) -o $@ $< kv_nav.cpp
$(BUILD)/aarch64/%: tests/%.cpp $(SRC) $(HDR)
	mkdir -p $(dir $@); $(CROSS)g++ $(CXXFLAGS) -static -o $@ $< kv_nav.cpp
$(BUILD)/host/kv_nav_sim: sim/kv_nav_sim.cpp $(SRC) $(HDR)
	mkdir -p $(dir $@); $(CXX) $(CXXFLAGS) -o $@ $< kv_nav.cpp
$(BUILD)/aarch64/kv_nav_sim: sim/kv_nav_sim.cpp $(SRC) $(HDR)
	mkdir -p $(dir $@); $(CROSS)g++ $(CXXFLAGS) -static -o $@ $< kv_nav.cpp

test: host
	$(BUILD)/host/test_planner_golden $(VECTORS) $(wildcard $(CASES))
	$(BUILD)/host/test_state_machine
test-aarch64: aarch64
	qemu-aarch64-static $(BUILD)/aarch64/test_planner_golden $(VECTORS) $(wildcard $(CASES))
	qemu-aarch64-static $(BUILD)/aarch64/test_state_machine

clean:
	rm -rf $(BUILD)
.PHONY: host aarch64 test test-aarch64 clean

# CTRLPS5: Nav2 NavFn (vendored upstream, unmodified, from git) as the host lockstep loop's G1 route stand-in
$(BUILD)/navfn/%: | $(BUILD)/navfn/nav2_navfn_planner $(BUILD)/navfn/rclcpp
	git show $(dev2_GOLDEN):tools/planner_model/vendor/$* > $@
$(BUILD)/navfn/nav2_navfn_planner $(BUILD)/navfn/rclcpp:
	mkdir -p $@
$(BUILD)/host/libnavfn_route.so: sim/navfn_route.cpp $(BUILD)/navfn/navfn.cpp $(BUILD)/navfn/nav2_navfn_planner/navfn.hpp $(BUILD)/navfn/rclcpp/rclcpp.hpp
	mkdir -p $(dir $@); $(CXX) -std=c++14 -O2 -fPIC -shared -I$(BUILD)/navfn -o $@ sim/navfn_route.cpp $(BUILD)/navfn/navfn.cpp
