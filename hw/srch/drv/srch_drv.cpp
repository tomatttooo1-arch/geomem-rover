// SRCHCHAIN0（2026-11-04）：概率搜索核（c8，m7_srch @ 0xA00D0000）的 PS 侧最小 C 接口，给 tools/golive/srch_chain.py 经 ctypes 用。
// 访问层 = hw/m7/pl/sw/pl_io.cpp（DRV5，规划核/前处理核板上已用）：/dev/mem 固定窗口（有 m7_srch UIO 节点则用 UIO）＋
// 物理连续 DMA 缓冲（dma_heap reserved → linux,cma → /dev/dpu，整块在 2 GiB 以下）。
// 协议同 tools/srch/srch_ps.py PLCore：Python 侧把参数块/输入写进缓冲（地址字段已加 pa），这里：刷缓存 → 写 PRM(0x10/0x14)
// → ap_start → 等 ap_done（轮询）→ 失效缓存 → 读 STATUS（VLD 读清，无效时调用方从结果块取）。
// mock（宿主自检）：MockBus 的 ap_start 回调里按物理地址跑定点 C 模型 srch_run_sw（同一份 hw/srch/srch_core.cpp，-DSRCH_SW），
// 只验证驱动的寄存器与缓冲协议，不是 RTL 或板上的证据。
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "../../m7/pl/sw/pl_io.hpp"

#ifdef SRCH_DRV_MOCK
extern "C" uint32_t srch_run_sw(uint8_t* ddr, uint64_t prm);
#endif

namespace {
constexpr uint32_t REG_CTRL = 0x00, REG_PRM_LO = 0x10, REG_PRM_HI = 0x14, REG_STATUS = 0x1C, REG_STATUS_VLD = 0x20;
constexpr uint64_t BASE_SRCH = 0xA00D0000ull;

struct H {
  std::unique_ptr<plio::Bus> bus;
  plio::Buf buf;
  uint64_t calls = 0, fails = 0, ns_sum = 0;
  std::string desc;
};

void put(char* why, int n, const std::string& s) {
  if (why && n > 0) { std::snprintf(why, n, "%s", s.c_str()); }
}
}  // namespace

extern "C" {

// mock=1：宿主 MockBus＋C 模型（只在带 SRCH_DRV_MOCK 编的宿主库里有）；mock=0：真硬件。
// wait：auto | irq | poll（默认 poll）；provider：auto | dmaheap:<heap> | dpu。返回句柄，失败 nullptr＋why。
void* srch_drv_open(int mock, uint64_t base, const char* wait, const char* provider, uint64_t bytes, char* why, int nwhy) {
  std::unique_ptr<H> h(new H());
  std::string w;
  if (mock) {
#ifdef SRCH_DRV_MOCK
    H* hp = h.get();
    auto* mb = new plio::MockBus([hp](plio::MockBus& m) {
      const uint64_t pa = m.reg64(REG_PRM_LO);
      uint8_t* base0 = static_cast<uint8_t*>(hp->buf.va) - hp->buf.pa;   // 核按物理地址访问，正好落进缓冲
      (void)m.va_of(pa, 8);
      const uint32_t st = srch_run_sw(base0, pa);
      m.set_reg(REG_STATUS, st);
      m.set_reg(REG_STATUS_VLD, 1);
    });
    mb->clear_on_read.push_back(REG_STATUS_VLD);
    h->bus.reset(mb);
#else
    put(why, nwhy, "mock not built in this library (aarch64 build)");
    return nullptr;
#endif
  } else {
    plio::Options o;
    o.base = base ? base : BASE_SRCH;
    o.uio_name = "m7_srch";
    o.wait = (wait && *wait) ? wait : "poll";
    if (provider && *provider) o.provider = provider;
    h->bus = plio::open_hw(o, &w);
    if (!h->bus) { put(why, nwhy, "open_hw: " + w); return nullptr; }
  }
  if (!h->bus->alloc(h->buf, bytes, &w)) { put(why, nwhy, "alloc: " + w); return nullptr; }
  if (h->buf.pa % 16 || h->buf.pa + h->buf.size > plio::kDmaLimit) {
    put(why, nwhy, "buffer misaligned or above 2 GiB");
    h->bus->release(h->buf);
    return nullptr;
  }
  std::memset(h->buf.va, 0, h->buf.size);
  h->bus->to_device(h->buf, 0, h->buf.size);
  if (!mock && !(h->bus->rd(REG_CTRL) & plio::AP_IDLE)) {
    put(why, nwhy, "core not idle at open (CTRL=0x" + std::to_string(h->bus->rd(REG_CTRL)) + ")");
    h->bus->release(h->buf);
    return nullptr;
  }
  h->desc = h->bus->describe() + " buf.provider=" + h->buf.provider;
  return h.release();
}

void* srch_drv_va(void* hv) { return static_cast<H*>(hv)->buf.va; }
uint64_t srch_drv_pa(void* hv) { return static_cast<H*>(hv)->buf.pa; }
uint64_t srch_drv_size(void* hv) { return static_cast<H*>(hv)->buf.size; }
uint64_t srch_drv_calls(void* hv) { return static_cast<H*>(hv)->calls; }
uint64_t srch_drv_fails(void* hv) { return static_cast<H*>(hv)->fails; }
uint64_t srch_drv_ns_sum(void* hv) { return static_cast<H*>(hv)->ns_sum; }
const char* srch_drv_describe(void* hv) { return static_cast<H*>(hv)->desc.c_str(); }

// 一次核调用。返回 0 = 完成（*status = STATUS，vld=0 时 *status_vld=0），-1 = 核忙，-2 = 超时。
int srch_drv_run(void* hv, uint64_t prm_pa, int timeout_ms, uint32_t* status, int* status_vld, uint64_t* ns) {
  H* h = static_cast<H*>(hv);
  plio::Bus& b = *h->bus;
  if (!(b.rd(REG_CTRL) & plio::AP_IDLE)) { ++h->fails; return -1; }
  b.to_device(h->buf, 0, h->buf.size);
  b.wr(REG_PRM_LO, static_cast<uint32_t>(prm_pa));
  b.wr(REG_PRM_HI, static_cast<uint32_t>(prm_pa >> 32));
  uint64_t t = 0;
  const bool ok = b.start_and_wait(timeout_ms, &t);
  b.from_device(h->buf, 0, h->buf.size);
  ++h->calls;
  h->ns_sum += t;
  if (ns) *ns = t;
  if (!ok) { ++h->fails; return -2; }
  const int vld = b.rd(REG_STATUS_VLD) & 1;
  if (status_vld) *status_vld = vld;
  if (status) *status = vld ? b.rd(REG_STATUS) : 0;
  return 0;
}

void srch_drv_close(void* hv) {
  H* h = static_cast<H*>(hv);
  if (!h) return;
  if (h->bus && h->buf.va) h->bus->release(h->buf);
  delete h;
}

}  // extern "C"
