// 检测后处理核（MERGE5A 整板，slot 10 0xA00A_0000，pl_ps_irq0 bit 3 / SPI 92）板上驱动 —— 宿主只编译检查，未上板。
//
// 数据通路：VART（/dev/dpu + 00_dnndk 控制器）跑完 DPU 子图后，三个输出头留在 DPU 自己的 CMA 工作区里；
// 本驱动用 RunnerExt::get_outputs() 拿到这三块 TensorBuffer 的物理地址（data_phy，见 detpost_vart.hpp），
// 不拷贝，直接把起点与三个偏移写给核。参数块与框表也放 CMA：经 /dev/dpu 的 DPUIOC_CREATE_BO 分配
// （与 VART BufferObjectDpCma 同一条路，返回物理地址，mmap(/dev/dpu, phy) 得用户态地址，DPUIOC_SYNC_BO 做缓存维护）。
//
// 前提（核接口，DETPOST1 README §9.3）：输入 ≤640、输入缓冲 ≤1 MiB 且起点 16 字节对齐、偏移 < 2^20、
// nc 1..40、iou_num/den 1..255、max_det 1..128、thr 钳到 [-128,128]、exp 表第 0 项 ≥1。
// 块设计只给本核映射了 DDR_LOW（0..2 GiB），所以三块缓冲的物理区间都必须在 2 GiB 以下，驱动拒绝其他情况。
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string>
#include "../detpost.h"

namespace detpost_drv {

constexpr uint64_t BASE = 0xA00A0000ull;     // = M7PL_BASE_DETPOST（hw/m7/pl/sw/m7pl_addrmap.h）
constexpr size_t WINDOW = 0x10000;
constexpr uint64_t DMA_LIMIT = 0x80000000ull; // 块设计 DMA 只映射 DDR_LOW
constexpr int PRM_WORDS = 69;                 // = dpk::PRM_WORDS
constexpr int OUT_HDR_WORDS = 2;              // 状态头 2 个 128 位字
constexpr int MAX_DET = 128;
constexpr size_t PRM_BYTES = PRM_WORDS * 16;
constexpr size_t OUT_BYTES = (OUT_HDR_WORDS + MAX_DET) * 16;
constexpr size_t IN_MAX_BYTES = 1u << 20;

// 参数块打包：与 hw/detpost/hls1/dp_param_pack.h（ap_int 版，testbench 用）逐字节相同，test_pack 核对。
void pack_params(const detpost::Config& cfg, uint8_t* prm /* PRM_BYTES */);

void set_class_floor(detpost::Config* cfg, int cls, double conf_floor, int fp);

// 三个头的位置。level 0/1/2 = P3/P4/P5（网格最大的是 P3），与寄存器 g80/off80、g40/off40、g20/off20 对应。
struct Layout {
  uint64_t in_phy = 0;            // 输入缓冲起点（三头中物理地址最小者）
  uint32_t off[3] = {0, 0, 0};
  int grid[3] = {0, 0, 0};
  size_t span = 0;                // 起点到最后一个头末尾的字节数
};
// 由各头的物理地址、字节数、网格边长推出 Layout 并检查全部前提；失败返回 false 与原因。
bool make_layout(const uint64_t phy[3], const size_t bytes[3], const int grid[3], int nc, Layout* out, std::string* why);

// CMA 缓冲（/dev/dpu DPUIOC_CREATE_BO）。
struct DmaBuf {
  int fd = -1;
  uint64_t phy = 0;
  size_t cap = 0;
  void* va = nullptr;
  bool alloc(size_t size, std::string* why);
  void free();
  bool sync_to_device(size_t off, size_t len);    // CPU 写后
  bool sync_from_device(size_t off, size_t len);  // CPU 读前
};

struct Result {
  detpost::Status st{};
  int n = 0;
  detpost::Box box[MAX_DET];
  uint64_t wait_us = 0;
};

class Core {
 public:
  // uio：/dev/uioN（m7_detpost 节点绑 generic-uio 时）用中断等待；空串则 /dev/mem 映射、轮询 ap_done。
  bool open(const std::string& uio, std::string* why);
  void close();
  // 初始化或换模型时调用：写参数块（打包 + 刷缓存），记下标量。
  bool configure(const detpost::Config& cfg, std::string* why);
  // 每帧：VART wait() 返回后调用。
  bool run(const Layout& lay, Result* res, int timeout_ms, std::string* why);
  uint32_t rd(uint32_t off) const;
  void wr(uint32_t off, uint32_t v);
 private:
  volatile uint32_t* regs_ = nullptr;
  int mem_fd_ = -1, uio_fd_ = -1;
  DmaBuf prm_, out_;
  detpost::Config cfg_{};
  bool configured_ = false;
};

}  // namespace detpost_drv
