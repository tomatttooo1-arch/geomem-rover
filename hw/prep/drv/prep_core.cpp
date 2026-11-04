// 识别前处理核驱动实现（见 prep_core.hpp）。编译须 -ffp-contract=off（prep_drv.h 的缩放系数用 float32 乘加）。
#include "prep_core.hpp"

#include <cstdio>
#include <cstring>

#include "../../m7/pl/sw/m7pl_prep_regs.h"
#include "../hls1/prep_drv.h"

namespace prep_drv {

static prep_cfg_t to_cfg(const Config& c) {
  prep_cfg_t p;
  std::memset(&p, 0, sizeof p);
  p.w0 = c.w0; p.h0 = c.h0; p.stride = c.stride; p.fmt = c.fmt; p.ups = c.ups; p.bypass = c.bypass;
  p.tw = c.tw; p.th = c.th; p.ow = c.ow; p.oh = c.oh; p.top = c.top; p.left = c.left; p.fp = c.fp;
  std::memcpy(p.k, c.kset ? PREP_K_BT601L : PREP_K_JFIF, sizeof p.k);
  return p;
}

bool check_config(const Config& c, std::string* why) {
  const int bpp = bytes_per_pixel(c.fmt);
  const char* e = nullptr;
  if (c.fmt < 0 || c.fmt > 3) e = "fmt not 0..3";
  else if (c.fp < 0 || c.fp > 7) e = "fix point not 0..7";
  else if (c.w0 < 4 || c.w0 > 2048 || c.h0 < 2 || c.h0 > 2048) e = "#1 source size outside 4..2048 x 2..2048";
  else if (c.ow < 16 || c.ow > 640 || c.oh < 1 || c.oh > 640 || c.tw > 640 || c.th > 640) e = "#2 content/tensor size";
  else if (c.ow > c.w0 || c.oh > c.h0) e = "#3 upscaling (OW>W0 or OH>H0)";
  else if (c.top < 0 || c.left < 0 || c.left + c.ow > c.tw || c.top + c.oh > c.th) e = "#4 content outside the tensor";
  else if ((c.tw * c.th * 3) % 16) e = "#5 TW*TH*3 not a multiple of 16";
  else if (c.stride % 16 || c.stride < c.w0 * bpp || c.stride / 16 > 4095) e = "#6 source stride";
  else if (c.fmt >= YUYV && (c.w0 & 1)) e = "#7 YUYV/UYVY with odd width";
  else if (c.bypass && (c.ow != c.w0 || c.oh != c.h0)) e = "#8 bypass needs OW=W0 and OH=H0";
  if (!e) {
    prep_cfg_t p = to_cfg(c);
    if (prep_drv_check(&p)) e = "prep_drv_check rejected (driver and prep_drv.h disagree)";
    else if (!c.bypass) {
      static uint64_t ent[2048];
      if (prep_drv_axis(c.w0, c.ow, ent) || prep_drv_axis(c.h0, c.oh, ent)) e = "#9 scale table cannot stream";
    }
  }
  if (e) {
    if (why) *why = e;
    return false;
  }
  return true;
}

bool build_params(const Config& c, std::uint8_t* blk, std::string* why, int perturb) {
  if (!check_config(c, why)) return false;
  prep_cfg_t p = to_cfg(c);
  if (prep_drv_params(&p, blk, perturb) != 0) {   // #10：帧头派生字段只由 prep_drv_header() 从这一份配置算出
    if (why) *why = "prep_drv_params failed";
    return false;
  }
  return true;
}

bool Core::open(plio::Options o, std::string* why) {
  if (!o.base) o.base = BASE;
  if (o.uio_name.empty()) o.uio_name = UIO_NAME;
  auto b = plio::open_hw(o, why);
  if (!b) return false;
  return open(std::move(b), why);
}

bool Core::open(std::unique_ptr<plio::Bus> bus, std::string* why) {
  close();
  bus_ = std::move(bus);
  if (!bus_->alloc(prm_, PRM_BYTES, why)) return false;
  return true;
}

void Core::close() {
  if (bus_) {
    bus_->release(src_);
    bus_->release(dst_);
    bus_->release(prm_);
  }
  bus_.reset();
  configured_ = false;
}

bool Core::ensure(plio::Buf& b, std::size_t size, std::string* why) {
  if (b.va && b.size >= size) return true;
  bus_->release(b);
  return bus_->alloc(b, size, why);
}

bool Core::configure(const Config& c, std::string* why) {
  if (!bus_) { *why = "open() first"; return false; }
  if (dead_) { *why = "core marked hung earlier (reload the bitstream to reset the PL)"; return false; }
  if (!check_config(c, why)) return false;
  if (!(bus_->rd(XPREP_TOP_CONTROL_ADDR_AP_CTRL) & plio::AP_IDLE)) { *why = "#12 core busy: parameter block not rewritten"; return false; }
  if (!ensure(src_, src_bytes(c), why) || !ensure(dst_, tensor_bytes(c), why)) return false;
  // #11 三块缓冲：对齐、大小、互不重叠、整段在 LPD_DDR_LOW
  struct R { const plio::Buf* b; std::size_t need; std::uint64_t align; const char* n; } rs[3] = {
      {&src_, src_bytes(c), 16, "src"}, {&dst_, tensor_bytes(c), 4096, "dst"},
      {&prm_, (std::size_t)(3 + (c.ow > c.oh ? c.ow : c.oh)) * 16, 16, "prm"}};
  for (auto& r : rs) {
    if (r.b->pa % r.align || r.b->size < r.need || r.b->pa + r.b->size > plio::kDmaLimit) {
      *why = std::string("#11 buffer ") + r.n + " misaligned, too small or above 2 GiB";
      return false;
    }
  }
  for (int i = 0; i < 3; ++i)
    for (int j = i + 1; j < 3; ++j)
      if (rs[i].b->pa < rs[j].b->pa + rs[j].b->size && rs[j].b->pa < rs[i].b->pa + rs[i].b->size) {
        *why = "#11 buffers overlap";
        return false;
      }
  if (!build_params(c, (std::uint8_t*)prm_.va, why)) return false;
  bus_->to_device(prm_, 0, PRM_BYTES);
  // 源缓冲行尾空隙清零一次；张量缓冲清一次脏行，之后 A53 不写它（#13）
  std::memset(src_.va, 0, src_.size);
  bus_->to_device(src_, 0, src_.size);
  bus_->to_device(dst_, 0, dst_.size);
  cfg_ = c;
  configured_ = true;
  return true;
}

bool Core::run(const std::uint8_t* src, int src_stride, std::uint8_t* out, std::size_t out_bytes, Result* res,
               std::string* why) {
  Result local;
  if (!res) res = &local;
  if (dead_) { *why = "core marked hung earlier (reload the bitstream to reset the PL)"; return false; }
  if (!configured_) { *why = "configure() first"; return false; }
  const Config& c = cfg_;
  const std::size_t rowb = (std::size_t)c.w0 * bytes_per_pixel(c.fmt);
  if (src_stride < (int)rowb || out_bytes < tensor_bytes(c)) { *why = "source stride or output size too small"; return false; }
  std::uint64_t t0 = plio::mono_ns();
  std::uint8_t* s = (std::uint8_t*)src_.va;
  if ((std::size_t)src_stride == (std::size_t)c.stride) std::memcpy(s, src, src_bytes(c));
  else
    for (int r = 0; r < c.h0; ++r) std::memcpy(s + (std::size_t)r * c.stride, src + (std::size_t)r * src_stride, rowb);
  bus_->to_device(src_, 0, src_bytes(c));
  res->copy_in_ns = plio::mono_ns() - t0;
  if (!(bus_->rd(XPREP_TOP_CONTROL_ADDR_AP_CTRL) & plio::AP_IDLE)) { *why = "#12 core busy: not started"; return false; }
  const std::uint32_t before = bus_->rd(XPREP_TOP_CONTROL_ADDR_STATUS_DATA);
  bus_->wr64(XPREP_TOP_CONTROL_ADDR_SRC_DATA, src_.pa);
  bus_->wr64(XPREP_TOP_CONTROL_ADDR_DST_DATA, dst_.pa);
  bus_->wr64(XPREP_TOP_CONTROL_ADDR_PRM_DATA, prm_.pa);
  if (!bus_->start_and_wait(timeout_ms, &res->core_ns)) {
    dead_ = true;   // #14
    char b[160];
    std::snprintf(b, sizeof b, "#14 no ap_done within %d ms: core treated as hung, not started again "
                  "(reload the bitstream to reset the PL)", timeout_ms);
    *why = b;
    return false;
  }
  (void)bus_->rd(XPREP_TOP_CONTROL_ADDR_STATUS_CTRL);   // 读清 STATUS_VLD
  const std::uint32_t st = bus_->rd(XPREP_TOP_CONTROL_ADDR_STATUS_DATA);
  res->status = st;
  if (st & 0xff) { *why = "core rejected the parameter header (STATUS bit0)"; return false; }
  if (((st >> 8) & 0xff) != 0x02) { *why = "STATUS version byte is not 0x02 (not the s18 core?)"; return false; }
  if ((st >> 16) != (((before >> 16) + 1) & 0xffff)) { *why = "STATUS frame count did not advance by 1"; return false; }
  t0 = plio::mono_ns();
  bus_->from_device(dst_, 0, tensor_bytes(c));
  std::memcpy(out, dst_.va, tensor_bytes(c));
  res->copy_out_ns = plio::mono_ns() - t0;
  return true;
}

std::string Core::describe() const {
  char b[320];
  std::snprintf(b, sizeof b, "{\"core\":\"pl_prep\",\"bus\":%s,\"src_pa\":\"0x%llx\",\"dst_pa\":\"0x%llx\",\"prm_pa\":\"0x%llx\"}",
                bus_ ? bus_->describe().c_str() : "null", (unsigned long long)src_.pa, (unsigned long long)dst_.pa,
                (unsigned long long)prm_.pa);
  return b;
}

}  // namespace prep_drv
