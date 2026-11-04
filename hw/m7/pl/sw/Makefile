# m7pl_test: host build (mock device) and aarch64 static cross build for KV260.
#   make host    -> build/host/m7pl_test      (x86, --provider mock only)
#   make aarch64 -> build/aarch64/m7pl_test   (static, runs on Kria Ubuntu 22.04)
#   make check   -> host + qemu-aarch64 mock runs, incl. the --corrupt control
# -ffp-contract=off: the CPU reference must not fuse multiply-adds, matching the
# HLS C simulation flags; aarch64 GCC would otherwise contract to FMA.
BUILD ?= build
CORE := ../..
# 2026-10-11 merge build: the AR overlay core replaced the legacy display core
# (m7_display_axi.cpp / display_overlay.cpp kept in the tree, not built here);
# the E2 region-check shell is new. ar_overlay.hpp needs the Vitis HLS headers
# (ap_int.h, hls_stream.h; header-only, also for the aarch64 build).
HLS_INC ?= /data1/Xilinx/Vitis_HLS/2022.2/include
# v3fc: m7pl_drm.cpp (drm demo driven by the removed PL watchdog) kept in the tree, not built.
SRCS := m7pl_test.cpp m7pl_dev.cpp \
        ../hls/m7_frame_axi.cpp ../hls/m7_object_roi_axi.cpp ../hls/m7_ar_overlay_axi.cpp \
        $(CORE)/frame_depth.cpp $(CORE)/object_evidence.cpp $(CORE)/ar_overlay.cpp
HDRS := m7pl_dev.hpp m7pl_hls_regs.h m7pl_addrmap.h
CXXFLAGS := -std=c++11 -O2 -Wall -Wextra -Wno-unknown-pragmas -ffp-contract=off -I. -I$(CORE) -isystem $(HLS_INC)
CROSS ?= aarch64-linux-gnu-

all: host aarch64

host: $(BUILD)/host/m7pl_test
aarch64: $(BUILD)/aarch64/m7pl_test

$(BUILD)/host/m7pl_test: $(SRCS) $(HDRS)
	mkdir -p $(dir $@)
	g++ $(CXXFLAGS) -o $@ $(SRCS)

$(BUILD)/aarch64/m7pl_test: $(SRCS) $(HDRS)
	mkdir -p $(dir $@)
	$(CROSS)g++ $(CXXFLAGS) -static -o $@ $(SRCS)

check: host aarch64
	$(BUILD)/host/m7pl_test --provider mock --w 640 --h 400 all
	$(BUILD)/host/m7pl_test --provider mock --w 1280 --h 800 frame ar roi
	! $(BUILD)/host/m7pl_test --provider mock --corrupt frame ar roi
	! $(BUILD)/host/m7pl_test --provider mock --corrupt ar
	! $(BUILD)/host/m7pl_test --provider mock --corrupt roi
	! $(BUILD)/host/m7pl_test --provider mock --flags-pa-offset 2 frame
	qemu-aarch64-static $(BUILD)/aarch64/m7pl_test --provider mock --w 640 --h 400 all
	! qemu-aarch64-static $(BUILD)/aarch64/m7pl_test --provider mock --corrupt frame

clean:
	rm -rf $(BUILD)
.PHONY: all host aarch64 check clean
