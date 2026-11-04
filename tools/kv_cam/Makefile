# kv_cam (NN-CAM1). Host build: replay/synth sources only (no OrbbecSDK on x86). Board build: aarch64 cross
# compile against OrbbecSDK v2.9.3 (same package CAMKV0 measured on the KV260).
#   make host          -> build/host/{kv_cam,kv_cam_read,libcamring.so}
#   make arm OBSDK=... -> build/arm/{kv_cam,kv_cam_read,libcamring.so}  (RUNPATH $ORIGIN/../lib, as camkv0)
OBSDK ?= ${FPGA_RUN}/CAMKV0/ext/OrbbecSDK_v2.9.3_202607151523_2f6561c_linux_arm64
CXXFLAGS ?= -O2 -g -Wall -Wextra -std=c++17
HDR := cam_ring.h kv_cam_common.h

host: build/host/kv_cam build/host/kv_cam_read build/host/libcamring.so
arm: build/arm/kv_cam build/arm/kv_cam_read build/arm/libcamring.so

build/host/kv_cam: kv_cam.cpp kv_cam_orbbec.cpp $(HDR)
	mkdir -p $(@D) && g++ $(CXXFLAGS) -o $@ kv_cam.cpp kv_cam_orbbec.cpp -pthread -lrt
build/host/kv_cam_read: kv_cam_read.cpp $(HDR)
	mkdir -p $(@D) && g++ $(CXXFLAGS) -o $@ kv_cam_read.cpp -pthread -lrt
build/host/libcamring.so: cam_ring_c.cpp cam_ring.h
	mkdir -p $(@D) && g++ $(CXXFLAGS) -shared -fPIC -o $@ cam_ring_c.cpp -lrt

ARMCXX := aarch64-linux-gnu-g++
build/arm/kv_cam: kv_cam.cpp kv_cam_orbbec.cpp $(HDR)
	mkdir -p $(@D) && $(ARMCXX) $(CXXFLAGS) -DKVCAM_ORBBEC -I$(OBSDK)/include -o $@ kv_cam.cpp kv_cam_orbbec.cpp \
	  -L$(OBSDK)/lib -lOrbbecSDK -Wl,-rpath,'$$ORIGIN/../lib' -pthread -lrt
build/arm/kv_cam_read: kv_cam_read.cpp $(HDR)
	mkdir -p $(@D) && $(ARMCXX) $(CXXFLAGS) -o $@ kv_cam_read.cpp -pthread -lrt
build/arm/libcamring.so: cam_ring_c.cpp cam_ring.h
	mkdir -p $(@D) && $(ARMCXX) $(CXXFLAGS) -shared -fPIC -o $@ cam_ring_c.cpp -lrt

clean:
	rm -rf build
.PHONY: host arm clean
