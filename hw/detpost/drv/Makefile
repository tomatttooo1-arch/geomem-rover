# 检测后处理核驱动：宿主单测（x86，需 Vitis HLS 的 ap_int.h）与 aarch64 编译检查（VART 衔接只做 -fsyntax-only）。
# make check  [HLS_INC=/data1/Xilinx/Vitis_HLS/2022.2/include] [VART_ROOT=<解开的 vitis-ai-3.0 tar 的 opt/vitis_ai/3.0>]
HLS_INC ?= /data1/Xilinx/Vitis_HLS/2022.2/include
VART_ROOT ?=
# 宿主无 aarch64 glog 头时给一个只供语法检查的桩目录（只要 glog/logging.h 宏定义）；板上用真 glog。
GLOG_INC ?= syntax_stub
BUILD ?= build
CXX_A64 ?= aarch64-linux-gnu-g++
FLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas
.PHONY: check
check:
	mkdir -p $(BUILD)
	g++ $(FLAGS) -I$(HLS_INC) test_drv.cpp detpost_drv.cpp ../detpost_host.cpp -o $(BUILD)/test_drv
	$(BUILD)/test_drv
	$(CXX_A64) $(FLAGS) -c detpost_drv.cpp -o $(BUILD)/detpost_drv.a64.o
	$(CXX_A64) $(FLAGS) -c ../detpost_host.cpp -o $(BUILD)/detpost_host.a64.o
	$(CXX_A64) $(FLAGS) -c detpost_core.cpp -o $(BUILD)/detpost_core.a64.o   # RECOG641: plio driver used by dpu_rx --post pl
ifneq ($(VART_ROOT),)
	$(CXX_A64) $(FLAGS) -fsyntax-only -isystem $(VART_ROOT)/include $(if $(GLOG_INC),-isystem $(GLOG_INC)) detpost_vart.cpp
endif
	@echo "DRV CHECK PASS"
