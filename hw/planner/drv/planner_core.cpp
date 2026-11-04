// 规划核驱动实现（见 planner_core.hpp）。
#include "planner_core.hpp"

#include <cstdio>
#include <cstring>

#include "../../m7/pl/sw/m7pl_planner_regs.h"

namespace planner_drv {

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
  if (!bus_->alloc(map_, CELLS_MAX, why) || !bus_->alloc(field_, CELLS_MAX * 4, why) ||
      !bus_->alloc(flags_, FLAGS_MAX * 4, why))
    return false;
  for (const plio::Buf* b : {&map_, &field_, &flags_})
    if (b->pa % 64 || b->pa + b->size > plio::kDmaLimit) { *why = "buffer misaligned or above 2 GiB"; return false; }
  bus_->to_device(field_, 0, field_.size);   // 清掉脏行，之后 A53 只在核写完后读
  bus_->to_device(flags_, 0, flags_.size);
  return true;
}

void Core::close() {
  if (bus_) {
    bus_->release(map_);
    bus_->release(field_);
    bus_->release(flags_);
  }
  bus_.reset();
}

bool Core::run(const Request& q, std::uint32_t* field, std::uint32_t* flags, Result* res, std::string* why) {
  Result local;
  if (!res) res = &local;
  if (!bus_) { *why = "open() first"; return false; }
  if (dead_) { *why = "core marked hung earlier (reload the bitstream to reset the PL)"; return false; }
  if (q.mode != 0 && q.mode != 1) { *why = "grid_mode must be 0 (80) or 1 (200)"; return false; }
  if (!q.map) { *why = "no map"; return false; }
  const int w = width(q.mode), n = cells(q.mode), nf = nflags(q.mode);
  const int stride = q.map_stride ? q.map_stride : w;
  std::uint64_t t0 = plio::mono_ns();
  std::uint8_t* m = (std::uint8_t*)map_.va;
  if (stride == w) std::memcpy(m, q.map, n);
  else
    for (int y = 0; y < w; ++y) std::memcpy(m + (std::size_t)y * w, q.map + (std::size_t)y * stride, w);
  bus_->to_device(map_, 0, n);
  res->copy_in_ns = plio::mono_ns() - t0;
  if (!(bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_AP_CTRL) & plio::AP_IDLE)) { *why = "core busy: not started"; return false; }
  bus_->wr64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_MAP_R_DATA, map_.pa);
  bus_->wr64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_DATA, field_.pa);
  bus_->wr64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FLAGS_DATA, flags_.pa);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_SX_DATA, (std::uint32_t)q.sx);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_SY_DATA, (std::uint32_t)q.sy);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GX_DATA, (std::uint32_t)q.gx);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GY_DATA, (std::uint32_t)q.gy);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_K_MAX_DATA, q.k_max);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GRID_MODE_DATA, (std::uint32_t)q.mode);
  if (!bus_->start_and_wait(timeout_ms[q.mode], &res->core_ns)) {
    dead_ = true;
    char b[160];
    std::snprintf(b, sizeof b, "no ap_done within %d ms: core treated as hung, not started again "
                  "(reload the bitstream to reset the PL)", timeout_ms[q.mode]);
    *why = b;
    return false;
  }
  (void)bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_STATUS_CTRL);   // 读清三个 vld
  (void)bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_ROUNDS_CTRL);
  (void)bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_VALID_CTRL);
  res->status = bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_STATUS_DATA);
  res->rounds = bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_ROUNDS_DATA);
  res->valid = bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_VALID_DATA);
  t0 = plio::mono_ns();
  if (field) {
    bus_->from_device(field_, 0, (std::size_t)n * 4);
    std::memcpy(field, field_.va, (std::size_t)n * 4);
  }
  if (flags) {
    bus_->from_device(flags_, 0, (std::size_t)nf * 4);
    std::memcpy(flags, flags_.va, (std::size_t)nf * 4);
  }
  res->copy_out_ns = plio::mono_ns() - t0;
  return true;
}

std::string Core::describe() const {
  char b[320];
  std::snprintf(b, sizeof b, "{\"core\":\"pl_planner\",\"bus\":%s,\"map_pa\":\"0x%llx\",\"field_pa\":\"0x%llx\",\"flags_pa\":\"0x%llx\"}",
                bus_ ? bus_->describe().c_str() : "null", (unsigned long long)map_.pa, (unsigned long long)field_.pa,
                (unsigned long long)flags_.pa);
  return b;
}

}  // namespace planner_drv

// ------------------------------------------------------------------ C ABI
namespace {
struct Handle {
  planner_drv::Core core;
  std::string err, desc;
  int calls = 0, fails = 0;
};
std::string g_open_error;
}  // namespace

namespace planner_drv {
void* open_handle_on(std::unique_ptr<plio::Bus> bus, std::string* why) {
  Handle* h = new Handle;
  if (!h->core.open(std::move(bus), why)) { delete h; return nullptr; }
  return h;
}
Core* handle_core(void* h) { return &static_cast<Handle*>(h)->core; }
}  // namespace planner_drv

extern "C" {

void* planner_drv_open(const char* wait, const char* dma) {
  Handle* h = new Handle;
  plio::Options o;
  if (wait && *wait) o.wait = wait;
  if (dma && *dma) o.provider = dma;
  if (!h->core.open(o, &g_open_error)) {
    delete h;
    return nullptr;
  }
  return h;
}
const char* planner_drv_open_error(void) { return g_open_error.c_str(); }
void planner_drv_close(void* h) { delete static_cast<Handle*>(h); }
const char* planner_drv_describe(void* h) {
  Handle* p = static_cast<Handle*>(h);
  p->desc = p->core.describe();
  return p->desc.c_str();
}
const char* planner_drv_error(void* h) { return static_cast<Handle*>(h)->err.c_str(); }

int planner_drv_kv_nav_field(void* h, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                             std::uint32_t* potential, unsigned* rounds) {
  Handle* p = static_cast<Handle*>(h);
  ++p->calls;
  planner_drv::Request q;
  q.mode = 0;
  q.map = map;
  q.sx = sx; q.sy = sy; q.gx = gx; q.gy = gy;
  q.k_max = kmax;
  planner_drv::Result r;
  if (!p->core.run(q, potential, nullptr, &r, &p->err)) {
    if (++p->fails <= 5 || p->fails % 100 == 0 || p->core.dead())
      std::fprintf(stderr, "{\"warning\":\"pl planner (failure %d): %s; software field\"}\n", p->fails, p->err.c_str());
    return planner_drv::NOT_CONVERGED;   // kv_nav: software fallback, the plan does not fail
  }
  if (rounds) *rounds = r.rounds;
  return (int)r.status;
}

int planner_drv_run(void* h, int mode, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                    std::uint32_t* field, std::uint32_t* flags, std::uint32_t* out3, std::uint64_t* core_ns) {
  Handle* p = static_cast<Handle*>(h);
  planner_drv::Request q;
  q.mode = mode;
  q.map = map;
  q.sx = sx; q.sy = sy; q.gx = gx; q.gy = gy;
  q.k_max = kmax;
  planner_drv::Result r;
  if (!p->core.run(q, field, flags, &r, &p->err)) return -1;
  if (out3) { out3[0] = r.status; out3[1] = r.rounds; out3[2] = r.valid; }
  if (core_ns) *core_ns = r.core_ns;
  return 0;
}

}  // extern "C"
