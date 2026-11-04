# DPUPIPE0/1: KV260 DPU input receiver + inference workers. Native build on the KV (Ubuntu 22.04 arm64, g++ 11,
# zlib) or host. GLOG_SHIM=1 compiles VART's headers against glog_shim/ when the board lacks glog -dev headers. VART=1 builds the VART 3.0 engine (KV: /opt/vitis_ai/3.0; VAI=<prefix> to point elsewhere).
# -ffp-contract=off: det_post.cpp must not fuse a*b+c (aarch64 g++ does by default), so its float32 values
# match the numpy reference.
CXX ?= g++
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -pthread -ffp-contract=off
LDLIBS = -lz -pthread -lrt
OUT ?= build
VAI ?= /opt/vitis_ai/3.0
ifeq ($(GLOG_SHIM),1)
CXXFLAGS += -Iglog_shim
endif
ifeq ($(VART),1)
CXXFLAGS += -DHAVE_VART -I$(VAI)/include
LDLIBS += -L$(VAI)/lib -Wl,-rpath,$(VAI)/lib -lvart-runner -lxir -lunilog
EXTRA = $(OUT)/dpu_infer_check
endif

# DRV5: pl_prep.cpp (--prep pl) on the PL pre-processing core driver (hw/prep/drv) and its bus layer (hw/m7/pl/sw/pl_io)
# RECOG641: pl_post.cpp (--post pl) on the detection post-processing core driver (hw/detpost/drv detpost_core + detpost_drv, detpost_host)
SRC_RX = dpu_rx.cpp det_post.cpp infer_engine.cpp local_src.cpp pl_prep.cpp pl_post.cpp ../../hw/prep/drv/prep_core.cpp ../../hw/m7/pl/sw/pl_io.cpp \
         ../../hw/detpost/drv/detpost_core.cpp ../../hw/detpost/drv/detpost_drv.cpp ../../hw/detpost/detpost_host.cpp
HDR = dpu_rx.h dpu_ring.h det_post.h infer_engine.h local_src.h colour_prep.h pl_prep.h pl_post.h ../../hw/detpost/drv/detpost_core.hpp ../../hw/detpost/drv/detpost_drv.hpp ../../hw/detpost/detpost.h ../kv_cam/cam_ring.h \
      ../../hw/prep/drv/prep_core.hpp ../../hw/m7/pl/sw/pl_io.hpp ../../hw/prep/hls1/prep_drv.h

all: $(OUT)/dpu_rx $(OUT)/test_dpu_ring $(OUT)/det_post_cli $(OUT)/soft_prep_cli $(EXTRA)

$(OUT):
	mkdir -p $(OUT)

$(OUT)/dpu_rx: dpu_rx_main.cpp $(SRC_RX) $(HDR) | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ dpu_rx_main.cpp $(SRC_RX) $(LDLIBS)

# NN-CAM2: SoftPrep standalone (YUYV file -> tensor file), for the host cross-check against PREP0/PREP1 references
$(OUT)/soft_prep_cli: soft_prep_cli.cpp colour_prep.h dpu_rx.cpp dpu_rx.h dpu_ring.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ soft_prep_cli.cpp dpu_rx.cpp $(LDLIBS)

$(OUT)/test_dpu_ring: test_dpu_ring.cpp dpu_rx.cpp dpu_rx.h dpu_ring.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ test_dpu_ring.cpp dpu_rx.cpp $(LDLIBS)

$(OUT)/det_post_cli: det_post_cli.cpp det_post.cpp det_post.h | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ det_post_cli.cpp det_post.cpp

$(OUT)/dpu_infer_check: dpu_infer_check.cpp $(SRC_RX) $(HDR) | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ dpu_infer_check.cpp $(SRC_RX) $(LDLIBS)

clean:
	rm -rf $(OUT)

.PHONY: all clean
