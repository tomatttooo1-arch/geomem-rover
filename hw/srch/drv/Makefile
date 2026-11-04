# SRCHCHAIN0：libsrch_drv.so（宿主 mock 版带定点 C 模型；aarch64 版只有驱动，板上用）。板上不编译。
CROSS ?= aarch64-linux-gnu-
BUILD ?= build
FLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -fPIC
SRC = srch_drv.cpp ../../m7/pl/sw/pl_io.cpp
.PHONY: lib
lib: $(BUILD)/host/libsrch_drv.so $(BUILD)/aarch64/libsrch_drv.so
$(BUILD)/host/libsrch_drv.so: $(SRC) ../srch_core.cpp ../srch_core.h
	mkdir -p $(dir $@); g++ $(FLAGS) -DSRCH_DRV_MOCK -DSRCH_SW -shared -o $@ $(SRC) ../srch_core.cpp
$(BUILD)/aarch64/libsrch_drv.so: $(SRC)
	mkdir -p $(dir $@); $(CROSS)g++ $(FLAGS) -shared -o $@ $(SRC)
