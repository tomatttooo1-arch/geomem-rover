# m7ar_present: resident AR presenter for kv_server --ar drm (see m7ar_present.cpp).
#   make host    -> $(BUILD)/host/m7ar_present     (x86; --provider mock --display none only)
#   make aarch64 -> $(BUILD)/aarch64/m7ar_present  (static, KV260 Ubuntu 22.04)
#   make nano    -> $(BUILD)/nano/m7ar_present     (AR2NANO 2026-11-02: static aarch64 for the Jetson Orin
#                   Nano, --provider cpu; the SAME sources compiled against fastsim/ (a small exact subset of
#                   ap_int.h / hls_stream.h) instead of the Xilinx C-simulation headers, ~1.5x faster)
#   make hostfast -> $(BUILD)/hostfast/m7ar_present (x86, fastsim headers: the checker's twin of nano)
# The AR core C sources are linked for the mock provider only (same shell as the HLS export).
BUILD ?= build
CORE := ../../../hw/m7
HLS_INC ?= /data1/Xilinx/Vitis_HLS/2022.2/include
# AR2NANO2: + ar_overlay_fast.cpp, the byte-exact fast twin (--provider cpu --cpu-impl fast, the default).
SRCS := m7ar_present.cpp $(CORE)/pl/hls/m7_ar_overlay_axi.cpp $(CORE)/ar_overlay.cpp $(CORE)/ar_overlay_fast.cpp
HDRS := m7ar_regs.h m7ar_wait.h $(CORE)/ar_overlay.hpp $(CORE)/ar_overlay_lut.inc $(CORE)/ar_overlay_fast.hpp
CXXFLAGS := -std=c++11 -O2 -Wall -Wextra -Werror -Wno-unknown-pragmas -ffp-contract=off -I. -I$(CORE) -isystem $(HLS_INC)
CROSS ?= aarch64-linux-gnu-

FASTFLAGS := -std=c++11 -O2 -Wall -Wextra -Werror -Wno-unknown-pragmas -ffp-contract=off -I. -Ifastsim -I$(CORE)
FASTHDRS := $(HDRS) fastsim/ap_int.h fastsim/hls_stream.h

all: host aarch64
host: $(BUILD)/host/m7ar_present
aarch64: $(BUILD)/aarch64/m7ar_present
nano: $(BUILD)/nano/m7ar_present
hostfast: $(BUILD)/hostfast/m7ar_present

$(BUILD)/host/m7ar_present: $(SRCS) $(HDRS)
	mkdir -p $(dir $@)
	g++ $(CXXFLAGS) -o $@ $(SRCS)

$(BUILD)/aarch64/m7ar_present: $(SRCS) $(HDRS)
	mkdir -p $(dir $@)
	$(CROSS)g++ $(CXXFLAGS) -static -o $@ $(SRCS)

$(BUILD)/nano/m7ar_present: $(SRCS) $(FASTHDRS)
	mkdir -p $(dir $@)
	$(CROSS)g++ $(FASTFLAGS) -static -o $@ $(SRCS)

$(BUILD)/hostfast/m7ar_present: $(SRCS) $(FASTHDRS)
	mkdir -p $(dir $@)
	g++ $(FASTFLAGS) -o $@ $(SRCS)

clean:
	rm -rf $(BUILD)
.PHONY: all host aarch64 nano hostfast clean
