// RECOG641：检测后处理核 plio 驱动实现（见 detpost_core.hpp）。
#include "detpost_core.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include "../../m7/pl/sw/m7pl_detpost_regs.h"

namespace detpost_drv {

bool core_in_bitstream(std::string* why, std::uint32_t* slot_map) {
  const int fd = ::open("/dev/mem", O_RDONLY | O_SYNC | O_CLOEXEC);
  if (fd < 0) { *why = std::string("open /dev/mem: ") + strerror(errno) + " (run as root)"; return false; }
  void* p = mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd, (off_t)SYSINFO_BASE);
  if (p == MAP_FAILED) { *why = std::string("mmap sysinfo: ") + strerror(errno); ::close(fd); return false; }
  const volatile std::uint32_t* r = (const volatile std::uint32_t*)p;
  const std::uint32_t magic = r[0], sm = r[SYSINFO_SLOT_MAP / 4];
  munmap(p, 4096);
  ::close(fd);
  if (slot_map) *slot_map = sm;
  char b[160];
  if (magic != SYSINFO_MAGIC) {
    std::snprintf(b, sizeof b, "sysinfo MAGIC %#x is not the m7 sysinfo (bitstream without the m7 PL)", magic);
    *why = b;
    return false;
  }
  if (!((sm >> SLOT_BIT_DETPOST) & 1)) {
    std::snprintf(b, sizeof b, "SLOT_MAP %#06x has no bit %d: this bitstream has no detection post-processing core", sm,
                  SLOT_BIT_DETPOST);
    *why = b;
    return false;
  }
  return true;
}

bool BusCore::open(plio::Options o, std::string* why) {
  if (!o.base) o.base = BASE;
  if (o.uio_name.empty()) o.uio_name = UIO_NAME;
  auto b = plio::open_hw(o, why);
  if (!b) return false;
  return open(std::move(b), why);
}

bool BusCore::open(std::unique_ptr<plio::Bus> bus, std::string* why) {
  close();
  bus_ = std::move(bus);
  if (!bus_->alloc(prm_, PRM_BYTES, why) || !bus_->alloc(out_, OUT_BYTES, why)) return false;
  if (prm_.pa % 16 || out_.pa % 16 || prm_.pa + prm_.size > DMA_LIMIT || out_.pa + out_.size > DMA_LIMIT) {
    *why = "parameter / box buffer misaligned or above 2 GiB (the core only maps DDR_LOW)";
    return false;
  }
  return true;
}

void BusCore::close() {
  if (bus_) {
    bus_->release(prm_);
    bus_->release(out_);
  }
  bus_.reset();
  configured_ = false;
}

bool BusCore::configure(const detpost::Config& cfg, std::string* why) {
  if (!bus_) { *why = "open() first"; return false; }
  if (dead_) { *why = "core marked hung earlier"; return false; }
  // 接口前提（DETPOST1 README §9.3）
  if (cfg.nc < 1 || cfg.nc > detpost::NC_MAX || cfg.iou_num < 1 || cfg.iou_num > 255 || cfg.iou_den < 1 ||
      cfg.iou_den > 255 || cfg.max_det < 1 || cfg.max_det > MAX_DET || cfg.exp_lut[0] < 1) {
    *why = "config outside the core's interface range (nc 1..40, IoU num/den 1..255, max_det 1..128, exp_lut[0] >= 1)";
    return false;
  }
  if (!(bus_->rd(XDETPOST_TOP_CONTROL_ADDR_AP_CTRL) & plio::AP_IDLE)) { *why = "core busy: parameters not rewritten"; return false; }
  pack_params(cfg, (std::uint8_t*)prm_.va);
  bus_->to_device(prm_, 0, PRM_BYTES);
  std::memset(out_.va, 0, OUT_BYTES);       // 清一次脏行：之后 CPU 只读框表，核写回不会被缓存回写覆盖
  bus_->to_device(out_, 0, OUT_BYTES);
  cfg_ = cfg;
  configured_ = true;
  return true;
}

bool BusCore::run(const Layout& lay, RunResult* res, int timeout_ms, std::string* why) {
  const std::uint64_t t0 = plio::mono_ns();
  if (dead_) { *why = "core marked hung earlier"; return false; }
  if (!configured_) { *why = "configure() first"; return false; }
  if (lay.in_phy % 16 || lay.in_phy + lay.span > DMA_LIMIT || lay.span > IN_MAX_BYTES) {
    *why = "head layout outside the core's DMA window";
    return false;
  }
  if (!(bus_->rd(XDETPOST_TOP_CONTROL_ADDR_AP_CTRL) & plio::AP_IDLE)) { *why = "core not idle"; return false; }
  const int thr = cfg_.cls_thr_q < -128 ? -128 : (cfg_.cls_thr_q > 128 ? 128 : cfg_.cls_thr_q);
  bus_->wr64(XDETPOST_TOP_CONTROL_ADDR_IN_R_DATA, lay.in_phy);
  bus_->wr64(XDETPOST_TOP_CONTROL_ADDR_PRM_DATA, prm_.pa);
  bus_->wr64(XDETPOST_TOP_CONTROL_ADDR_OUT_R_DATA, out_.pa);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_NC_DATA, (std::uint32_t)cfg_.nc);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_THR_DATA, (std::uint32_t)thr & 0x1ff);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_IOU_NUM_DATA, (std::uint32_t)cfg_.iou_num);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_IOU_DEN_DATA, (std::uint32_t)cfg_.iou_den);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_MAX_DET_DATA, (std::uint32_t)cfg_.max_det);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_G80_DATA, (std::uint32_t)lay.grid[0]);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_G40_DATA, (std::uint32_t)lay.grid[1]);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_G20_DATA, (std::uint32_t)lay.grid[2]);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_OFF80_DATA, lay.off[0]);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_OFF40_DATA, lay.off[1]);
  bus_->wr(XDETPOST_TOP_CONTROL_ADDR_OFF20_DATA, lay.off[2]);
  std::uint64_t ns = 0;
  const bool done = bus_->start_and_wait(timeout_ms, &ns);
  res->core_ns = ns;
  if (!done) {
    dead_ = true;
    *why = "timeout waiting for ap_done (core marked hung; reload the bitstream to reset the PL)";
    return false;
  }
  bus_->from_device(out_, 0, OUT_BYTES);
  const std::int32_t* h = (const std::int32_t*)out_.va;
  res->st.n_cand = h[0];
  res->st.n_cand_kept = h[1];
  res->st.overflow = h[2];
  res->st.final_level = h[3];
  res->st.n_nms = h[4];
  res->st.n_out = h[5];
  if (h[5] < 0 || h[5] > cfg_.max_det) {
    *why = "core status header: n_out " + std::to_string(h[5]) + " outside 0..max_det";
    return false;
  }
  res->n = h[5];
  std::memcpy(res->box, (const std::uint8_t*)out_.va + OUT_HDR_WORDS * 16, (std::size_t)res->n * 16);
  res->total_ns = plio::mono_ns() - t0;
  return true;
}

std::string BusCore::describe() const {
  std::string b = bus_ ? bus_->describe() : std::string("null");
  if (!b.empty() && b[0] != '{' && b != "null") b = "\"" + b + "\"";   // MockBus describes itself as a bare word
  return std::string("{\"bus\":") + b + ",\"prm_pa\":" +
         std::to_string(prm_.pa) + ",\"out_pa\":" + std::to_string(out_.pa) + ",\"dma\":\"" + prm_.provider +
         "\",\"dead\":" + (dead_ ? "true" : "false") + "}";
}

}  // namespace detpost_drv
