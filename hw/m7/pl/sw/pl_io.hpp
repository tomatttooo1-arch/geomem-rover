// DRV5（2026-11-04）：m5 新核（前处理 slot 11、规划 slot 12）PS 驱动共用的访问层——一个控制窗口＋若干 DMA 缓冲。
//
// 做法沿用两套已有代码：
//   寄存器   /dev/mem 映射固定 64 KiB 窗口（m7pl_dev.cpp 的做法，板上已用多轮）；有 generic-uio 节点时可改用
//            UIO（map0 = 同一窗口），并用它等中断（detpost_drv.cpp 的做法）。
//   DMA 缓冲 m5 的 LPD 互连只映射 LPD_DDR_LOW（0..2 GiB），所以缓冲必须物理连续且整段在 2 GiB 以下。
//            提供者按顺序试：dma_heap reserved → dma_heap linux,cma（pagemap 取物理地址、逐页核连续、
//            缓存用 DC CVAC / CIVAC 维护，同 m7pl_dev.cpp，板上 m7pl_test --heap reserved 用过）→ /dev/dpu
//            的 DPUIOC_CREATE_BO（同 detpost_drv.cpp / VART BufferObjectDpCma，要 xlnx_dpu 模块）。
//   mock     宿主单测用：寄存器是普通内存，缓冲 malloc 加假物理地址；写 ap_start 时调用测试给的回调
//            （回调里跑核的 HLS C 源），再置 ap_done。它只验证驱动的寄存器与缓冲协议，不是 RTL 或板上的证据。
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace plio {

constexpr std::uint64_t kDmaLimit = 0x80000000ull;   // LPD_DDR_LOW
constexpr std::uint32_t kWindow = 0x10000;
// ap_ctrl_hs 位（HLS s_axilite 控制寄存器 0x00）
constexpr std::uint32_t AP_START = 1u << 0, AP_DONE = 1u << 1, AP_IDLE = 1u << 2, AP_READY = 1u << 3;

std::uint64_t mono_ns();

struct Buf {
  void* va = nullptr;
  std::uint64_t pa = 0;
  std::size_t size = 0;
  bool cached = false;      // 要显式维护缓存
  int fd = -1;              // dma-buf fd（dmaheap）或 /dev/dpu fd（dpu）
  std::string provider;
};

class Bus {
 public:
  virtual ~Bus() {}
  virtual std::uint32_t rd(std::uint32_t off) = 0;
  virtual void wr(std::uint32_t off, std::uint32_t v) = 0;
  void wr64(std::uint32_t off, std::uint64_t v) {
    wr(off, static_cast<std::uint32_t>(v));
    wr(off + 4, static_cast<std::uint32_t>(v >> 32));
  }
  virtual bool alloc(Buf& out, std::size_t size, std::string* why) = 0;
  virtual void release(Buf& b) = 0;
  // CPU 写完、核要读之前 / 核写完、CPU 要读之前
  virtual void to_device(const Buf& b, std::size_t off, std::size_t len) = 0;
  virtual void from_device(const Buf& b, std::size_t off, std::size_t len) = 0;
  // 写 ap_start 并等 ap_done。irq 模式：开 GIE/IER（ap_done）与 UIO 中断后启动，poll(UIO)；结束写 ISR=1 清中断。
  // poll 模式：轮询 CTRL bit1。返回 true = 见到完成；*ns = 写 ap_start 到看到完成的单调时钟时长。
  virtual bool start_and_wait(int timeout_ms, std::uint64_t* ns) = 0;
  virtual bool irq_mode() const = 0;
  virtual std::string describe() const = 0;
};

struct Options {
  std::uint64_t base = 0;            // 控制窗口物理基址（m7pl_addrmap.h）
  std::string uio_name;              // 设备树节点名（如 m7_prep）；找到 /sys/class/uio/uioN/name 相同者才用 UIO
  std::string wait = "auto";         // auto（有 UIO 用中断，否则轮询）| irq | poll
  std::string provider = "auto";     // auto | dmaheap:<heap> | dpu
};
// 打开真硬件；失败返回 nullptr 并给出原因。
std::unique_ptr<Bus> open_hw(const Options& o, std::string* why);
// /sys/class/uio 里名字等于 name 的设备（"/dev/uioN"），没有则空串。
std::string find_uio(const std::string& name);

// 宿主 mock：on_start 在写 ap_start 时同步调用（测试在里面跑核的 C 源，读写 mock 内存）。
class MockBus : public Bus {
 public:
  using Hook = std::function<void(MockBus&)>;
  explicit MockBus(Hook on_start) : hook_(std::move(on_start)), regs_(kWindow / 4, 0) { regs_[0] = AP_IDLE; }
  ~MockBus() override;
  std::uint32_t rd(std::uint32_t off) override;
  void wr(std::uint32_t off, std::uint32_t v) override;
  bool alloc(Buf& out, std::size_t size, std::string* why) override;
  void release(Buf& b) override;
  void to_device(const Buf&, std::size_t, std::size_t) override { ++syncs_to; }
  void from_device(const Buf&, std::size_t, std::size_t) override { ++syncs_from; }
  bool start_and_wait(int timeout_ms, std::uint64_t* ns) override;
  bool irq_mode() const override { return false; }
  std::string describe() const override { return "mock"; }
  // 测试侧：寄存器原值（不经读清语义）、按物理地址取 mock 内存（越界返回 nullptr）
  std::uint32_t reg(std::uint32_t off) const { return regs_[off / 4]; }
  void set_reg(std::uint32_t off, std::uint32_t v) { regs_[off / 4] = v; }
  std::uint64_t reg64(std::uint32_t off) const { return reg(off) | (std::uint64_t)reg(off + 4) << 32; }
  void* va_of(std::uint64_t pa, std::size_t len);
  bool hang_next = false;            // 测试：下一次启动不置 ap_done（模拟核挂死）
  int starts = 0, syncs_to = 0, syncs_from = 0;
  std::vector<std::uint32_t> clear_on_read;   // 读清寄存器（如 *_CTRL 的 vld 位）

 private:
  Hook hook_;
  std::vector<std::uint32_t> regs_;
  struct Blk { std::uint64_t pa; std::size_t size; void* va; };
  std::vector<Blk> blks_;
  std::uint64_t next_pa_ = 0x10000000ull;
};

}  // namespace plio
