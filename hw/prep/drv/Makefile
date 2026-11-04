# DRV5 前处理核驱动：宿主对照（x86）＋ aarch64 交叉编译＋ qemu-aarch64 跑同一对照。
#   make check   [HLS_INC=...] [PREP1=<RUN>/PREP1]
# 黄金：PREP1 s18 联仿的参数块（从 HLS 事务向量文件抽出，extract_cosim_prm.py）与联仿张量 golden/cosim/tb_out。
HLS_INC ?= /data1/Xilinx/Vitis_HLS/2022.2/include
PREP1 ?= ${FPGA_RUN}/PREP1
CDATA ?= $(PREP1)/build/s18ooc/sol1/sim/tv/cdatafile
VEC ?= $(PREP1)/build/s18ooc_src/vec_cosim.txt
GTEN ?= $(PREP1)/golden/cosim/tb_out
BUILD ?= build
CROSS ?= aarch64-linux-gnu-
FLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-function -Wno-unused-label -ffp-contract=off
LIBS = -lz -pthread -lrt
SRC = prep_core.cpp ../../m7/pl/sw/pl_io.cpp ../../../tools/dpu_pipe/pl_prep.cpp ../../../tools/dpu_pipe/dpu_rx.cpp
TEST = test_prep_drv.cpp ../hls1/prep_k.cpp
.PHONY: check
check:
	mkdir -p $(BUILD)/prm
	python3 extract_cosim_prm.py $(CDATA) $(VEC) $(BUILD)/prm
	g++ $(FLAGS) -isystem $(HLS_INC) $(TEST) $(SRC) -o $(BUILD)/test_prep_drv $(LIBS)
	$(BUILD)/test_prep_drv $(BUILD)/prm $(VEC) $(GTEN)
	$(CROSS)gcc -O2 -c zstub_a64.c -o $(BUILD)/zstub_a64.o
	$(CROSS)g++ $(FLAGS) -static -isystem $(HLS_INC) $(TEST) $(SRC) $(BUILD)/zstub_a64.o -o $(BUILD)/test_prep_drv.a64 -pthread -lrt
	qemu-aarch64-static $(BUILD)/test_prep_drv.a64 $(BUILD)/prm $(VEC) $(GTEN)
	@echo "PREP DRV MAKE CHECK PASS"
