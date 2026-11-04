// 检测后处理核板上驱动实现（见 detpost_drv.hpp）。只用 Linux 系统调用，不依赖 VART。
#include "detpost_drv.hpp"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "../../m7/pl/sw/m7pl_detpost_regs.h"

namespace detpost_drv {

static_assert(sizeof(detpost::Box) == 16, "Box must be 16 bytes (one 128-bit word)");

// ---- xlnx_dpu 内核驱动的 CMA 接口（与 VART 3.0 vart/buffer-object/src/dpu.h 同定义）----
namespace {
struct dpcma_req_alloc { size_t size; uint64_t phy_addr; size_t capacity; };
struct dpcma_req_free { uint64_t phy_addr; size_t capacity; };
struct dpcma_req_sync { uint64_t phy_addr; size_t size; int direction; };
constexpr int DPCMA_FROM_CPU_TO_DEVICE = 0;
constexpr int DPCMA_FROM_DEVICE_TO_CPU = 1;
#define DPU_IOC_MAGIC 'D'
#define DPUIOC_CREATE_BO _IOWR(DPU_IOC_MAGIC, 1, struct dpcma_req_alloc*)
#define DPUIOC_FREE_BO _IOWR(DPU_IOC_MAGIC, 2, struct dpcma_req_free*)
#define DPUIOC_SYNC_BO _IOWR(DPU_IOC_MAGIC, 3, struct dpcma_req_sync*)
const char* DPU_DEV = "/dev/dpu";

uint64_t now_us() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}
}  // namespace

void pack_params(const detpost::Config& cfg, uint8_t* prm) {
  memset(prm, 0, PRM_BYTES);
  auto put16 = [prm](int item, uint16_t v) {   // 项 i 在字 i/8 的第 (i%8)*16 位起，小端
    prm[(item / 8) * 16 + (item % 8) * 2] = (uint8_t)(v & 0xff);
    prm[(item / 8) * 16 + (item % 8) * 2 + 1] = (uint8_t)(v >> 8);
  };
  for (int i = 0; i < 256; i++) {
    put16(i, cfg.exp_lut[i]);
    put16(256 + i, cfg.sig_lut[i]);
  }
  for (int c = 0; c < detpost::NC_MAX; c++)
    put16(512 + c, (uint16_t)((uint8_t)cfg.cls_min_q[c]) | (uint16_t)(cfg.cls_enable[c] ? 0x100 : 0));
}

void set_class_floor(detpost::Config* cfg, int cls, double conf_floor, int fp) {
  if (conf_floor <= 0) { cfg->cls_min_q[cls] = -128; return; }
  // 原先用 conf_to_q（严格 >）在下限恰好可表示时差一档（fp=2 时 0.5 = sigmoid(0)：q=0 的框被误丢）。
  const int q = detpost::score_floor_to_q(conf_floor, fp);
  if (q > 127) {                 // int8 表示不了「全不放行」：关白名单
    cfg->cls_enable[cls] = 0;
    cfg->cls_min_q[cls] = 127;
  } else {
    cfg->cls_min_q[cls] = (int8_t)(q < -128 ? -128 : q);
  }
}

bool make_layout(const uint64_t phy[3], const size_t bytes[3], const int grid[3], int nc, Layout* out,
                 std::string* why) {
  if (nc < 1 || nc > detpost::NC_MAX) { *why = "nc out of 1..40"; return false; }
  const size_t ch = 4 * detpost::REG_MAX + nc;
  // 按网格从大到小排成 P3/P4/P5
  int ord[3] = {0, 1, 2};
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++)
      if (grid[ord[j]] > grid[ord[i]]) { int t = ord[i]; ord[i] = ord[j]; ord[j] = t; }
  const int gmax[3] = {80, 40, 20};
  uint64_t lo = UINT64_MAX, hi = 0;
  for (int k = 0; k < 3; k++) {
    const int s = ord[k];
    if (grid[s] < 1 || grid[s] > gmax[k]) { *why = "grid out of range (input > 640?)"; return false; }
    if (k > 0 && grid[s] * 2 != grid[ord[k - 1]]) { *why = "grids are not a 2x pyramid"; return false; }
    if (bytes[s] < (size_t)grid[s] * grid[s] * ch) { *why = "head smaller than grid*grid*(64+nc)"; return false; }
    lo = phy[s] < lo ? phy[s] : lo;
    hi = phy[s] + bytes[s] > hi ? phy[s] + bytes[s] : hi;
  }
  if (lo % 16) { *why = "input base not 16-byte aligned"; return false; }
  if (hi > DMA_LIMIT) { *why = "DPU output above 2 GiB (only DDR_LOW is mapped for the core)"; return false; }
  if (hi - lo > IN_MAX_BYTES) { *why = "DPU output span > 1 MiB"; return false; }
  out->in_phy = lo;
  out->span = (size_t)(hi - lo);
  for (int k = 0; k < 3; k++) {
    out->off[k] = (uint32_t)(phy[ord[k]] - lo);
    out->grid[k] = grid[ord[k]];
  }
  return true;
}

bool DmaBuf::alloc(size_t size, std::string* why) {
  fd = ::open(DPU_DEV, O_RDWR | O_CLOEXEC);
  if (fd < 0) { *why = std::string("open /dev/dpu: ") + strerror(errno); return false; }
  dpcma_req_alloc r{size, 0, 0};
  if (ioctl(fd, DPUIOC_CREATE_BO, &r)) { *why = std::string("DPUIOC_CREATE_BO: ") + strerror(errno); return false; }
  phy = r.phy_addr;
  cap = r.capacity;
  va = mmap(nullptr, cap, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)phy);
  if (va == MAP_FAILED) { va = nullptr; *why = std::string("mmap /dev/dpu: ") + strerror(errno); return false; }
  if (phy % 16 || phy + cap > DMA_LIMIT) { *why = "CMA buffer unaligned or above 2 GiB"; return false; }
  return true;
}

void DmaBuf::free() {
  if (va) munmap(va, cap);
  if (fd >= 0 && phy) {
    dpcma_req_free r{phy, cap};
    ioctl(fd, DPUIOC_FREE_BO, &r);
  }
  if (fd >= 0) ::close(fd);
  fd = -1; va = nullptr; phy = 0; cap = 0;
}

bool DmaBuf::sync_to_device(size_t off, size_t len) {
  dpcma_req_sync r{phy + off, len, DPCMA_FROM_CPU_TO_DEVICE};
  return ioctl(fd, DPUIOC_SYNC_BO, &r) == 0;
}

bool DmaBuf::sync_from_device(size_t off, size_t len) {
  dpcma_req_sync r{phy + off, len, DPCMA_FROM_DEVICE_TO_CPU};
  return ioctl(fd, DPUIOC_SYNC_BO, &r) == 0;
}

uint32_t Core::rd(uint32_t off) const { return regs_[off / 4]; }
void Core::wr(uint32_t off, uint32_t v) { regs_[off / 4] = v; }

bool Core::open(const std::string& uio, std::string* why) {
  int fd = -1;
  if (!uio.empty()) {
    uio_fd_ = ::open(uio.c_str(), O_RDWR | O_CLOEXEC);
    if (uio_fd_ < 0) { *why = "open " + uio + ": " + strerror(errno); return false; }
    fd = uio_fd_;     // UIO map 0 = 节点 reg（0xA00A_0000，64 KiB），偏移 0
  } else {
    mem_fd_ = ::open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
    if (mem_fd_ < 0) { *why = std::string("open /dev/mem: ") + strerror(errno); return false; }
    fd = mem_fd_;
  }
  void* p = mmap(nullptr, WINDOW, PROT_READ | PROT_WRITE, MAP_SHARED, fd, uio.empty() ? (off_t)BASE : 0);
  if (p == MAP_FAILED) { *why = std::string("mmap regs: ") + strerror(errno); return false; }
  regs_ = (volatile uint32_t*)p;
  if (!prm_.alloc(PRM_BYTES, why) || !out_.alloc(OUT_BYTES, why)) return false;
  return true;
}

void Core::close() {
  if (regs_) munmap((void*)regs_, WINDOW);
  regs_ = nullptr;
  prm_.free();
  out_.free();
  if (mem_fd_ >= 0) ::close(mem_fd_);
  if (uio_fd_ >= 0) ::close(uio_fd_);
  mem_fd_ = uio_fd_ = -1;
}

bool Core::configure(const detpost::Config& cfg, std::string* why) {
  if (cfg.nc < 1 || cfg.nc > 40 || cfg.iou_num < 1 || cfg.iou_num > 255 || cfg.iou_den < 1 || cfg.iou_den > 255 ||
      cfg.max_det < 1 || cfg.max_det > MAX_DET || cfg.exp_lut[0] < 1) {
    *why = "config outside the core's interface range";
    return false;
  }
  cfg_ = cfg;
  pack_params(cfg_, (uint8_t*)prm_.va);
  if (!prm_.sync_to_device(0, PRM_BYTES)) { *why = "DPUIOC_SYNC_BO (params)"; return false; }
  configured_ = true;
  return true;
}

bool Core::run(const Layout& lay, Result* res, int timeout_ms, std::string* why) {
  if (!configured_) { *why = "configure() first"; return false; }
  if (!(rd(XDETPOST_TOP_CONTROL_ADDR_AP_CTRL) & 0x4)) { *why = "core not idle"; return false; }
  const int thr = cfg_.cls_thr_q < -128 ? -128 : (cfg_.cls_thr_q > 128 ? 128 : cfg_.cls_thr_q);
  auto w64 = [this](uint32_t off, uint64_t v) { wr(off, (uint32_t)v); wr(off + 4, (uint32_t)(v >> 32)); };
  w64(XDETPOST_TOP_CONTROL_ADDR_IN_R_DATA, lay.in_phy);
  w64(XDETPOST_TOP_CONTROL_ADDR_PRM_DATA, prm_.phy);
  w64(XDETPOST_TOP_CONTROL_ADDR_OUT_R_DATA, out_.phy);
  wr(XDETPOST_TOP_CONTROL_ADDR_NC_DATA, (uint32_t)cfg_.nc);
  wr(XDETPOST_TOP_CONTROL_ADDR_THR_DATA, (uint32_t)thr & 0x1ff);
  wr(XDETPOST_TOP_CONTROL_ADDR_IOU_NUM_DATA, (uint32_t)cfg_.iou_num);
  wr(XDETPOST_TOP_CONTROL_ADDR_IOU_DEN_DATA, (uint32_t)cfg_.iou_den);
  wr(XDETPOST_TOP_CONTROL_ADDR_MAX_DET_DATA, (uint32_t)cfg_.max_det);
  wr(XDETPOST_TOP_CONTROL_ADDR_G80_DATA, (uint32_t)lay.grid[0]);
  wr(XDETPOST_TOP_CONTROL_ADDR_G40_DATA, (uint32_t)lay.grid[1]);
  wr(XDETPOST_TOP_CONTROL_ADDR_G20_DATA, (uint32_t)lay.grid[2]);
  wr(XDETPOST_TOP_CONTROL_ADDR_OFF80_DATA, lay.off[0]);
  wr(XDETPOST_TOP_CONTROL_ADDR_OFF40_DATA, lay.off[1]);
  wr(XDETPOST_TOP_CONTROL_ADDR_OFF20_DATA, lay.off[2]);
  if (uio_fd_ >= 0) {
    wr(XDETPOST_TOP_CONTROL_ADDR_GIE, 1);
    wr(XDETPOST_TOP_CONTROL_ADDR_IER, 1);       // ap_done
    uint32_t en = 1;
    if (write(uio_fd_, &en, 4) != 4) { *why = "uio irq enable"; return false; }
  }
  const uint64_t t0 = now_us();
  wr(XDETPOST_TOP_CONTROL_ADDR_AP_CTRL, 1);     // ap_start
  bool done = false;
  if (uio_fd_ >= 0) {
    pollfd pf{uio_fd_, POLLIN, 0};
    if (poll(&pf, 1, timeout_ms) == 1) {
      uint32_t cnt;
      done = read(uio_fd_, &cnt, 4) == 4;
    }
    wr(XDETPOST_TOP_CONTROL_ADDR_ISR, 1);       // toggle-on-write 清 ap_done 中断位
  } else {
    while (!(done = (rd(XDETPOST_TOP_CONTROL_ADDR_AP_CTRL) & 0x2) != 0)) {
      if (now_us() - t0 > (uint64_t)timeout_ms * 1000u) break;
    }
  }
  res->wait_us = now_us() - t0;
  if (!done) { *why = "timeout waiting for ap_done"; return false; }
  if (!out_.sync_from_device(0, OUT_BYTES)) { *why = "DPUIOC_SYNC_BO (boxes)"; return false; }
  const int32_t* h = (const int32_t*)out_.va;
  res->st.n_cand = h[0];
  res->st.n_cand_kept = h[1];
  res->st.overflow = h[2];
  res->st.final_level = h[3];
  res->st.n_nms = h[4];
  res->st.n_out = h[5];
  res->n = h[5] < 0 ? 0 : (h[5] > MAX_DET ? MAX_DET : h[5]);
  memcpy(res->box, (const uint8_t*)out_.va + OUT_HDR_WORDS * 16, (size_t)res->n * 16);
  return true;
}

}  // namespace detpost_drv
