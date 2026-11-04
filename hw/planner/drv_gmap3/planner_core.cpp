// 规划核驱动实现（见 planner_core.hpp）。
#include "planner_core.hpp"

#include <cstdio>
#include <cstring>

#include "../../m7/pl/sw/gmap3_v9/m7pl_planner_regs.h"

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
  if (!bus_->alloc(map_, MAP_BYTES, why) || !bus_->alloc(field_, CELLS_MAX * 4, why) ||
      !bus_->alloc(flags_, FLAGS_WORDS * 4, why) || !bus_->alloc(area_, AREA_BYTES, why))
    return false;
  for (const plio::Buf* b : {&map_, &field_, &flags_, &area_})
    if (b->pa % 64 || b->pa + b->size > plio::kDmaLimit) { *why = "buffer misaligned or above 2 GiB"; return false; }
  bus_->to_device(field_, 0, field_.size);   // 清掉脏行，之后 A53 只在核写完后读
  bus_->to_device(flags_, 0, flags_.size);
  bus_->to_device(area_, 0, area_.size);
  return true;
}

void Core::close() {
  if (bus_) {
    bus_->release(map_);
    bus_->release(field_);
    bus_->release(flags_);
    bus_->release(area_);
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
  const bool path = q.path_params != nullptr;
  if (path) std::memcpy(m + n, q.path_params, PATH_PARAM_BYTES);   // PLNPATH0 §4.4：参数块紧跟代价图
  bus_->to_device(map_, 0, (std::size_t)n + (path ? PATH_PARAM_BYTES : 0));
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
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_PATH_CTRL_DATA, path ? 1u : 0u);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GMAP_OP_DATA, 0u);
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
  res->path_header = 0;
  if (path) {
    // 表头 [15:0] n、[23:16] 候选号、[27:24] 取路状态、[31:28] 0xA（本次写过）；n 截到 4096 防越界
    const std::uint32_t* fw = (const std::uint32_t*)flags_.va;
    bus_->from_device(flags_, (std::size_t)nf * 4, 4);
    res->path_header = fw[nf];
    std::uint32_t np = res->path_header & 0xFFFFu;
    if ((res->path_header >> 28) != 0xAu || np > (std::uint32_t)PATH_POINTS_MAX) np = 0;
    if (q.path_out) {
      bus_->from_device(flags_, (std::size_t)nf * 4, (std::size_t)(1 + np) * 4);
      std::memcpy(q.path_out, fw + nf, (std::size_t)(1 + np) * 4);
    }
  }
  res->copy_out_ns = plio::mono_ns() - t0;
  return true;
}

bool Core::gmap(unsigned op, GmapResult* r, std::string* why) {
  GmapResult local;
  if (!r) r = &local;
  if (!bus_) { *why = "open() first"; return false; }
  if (dead_) { *why = "core marked hung earlier (reload the bitstream to reset the PL)"; return false; }
  if (op == 0 || op > 7) { *why = "gmap op must be 1..7 (0 = planning, use run())"; return false; }
  // 输入：PS 写过的区刷到 DDR
  if (op == GMAP_WINDOW) {
    bus_->to_device(area_, OFF_AUX, 6400);
    bus_->to_device(area_, OFF_WPARAM, 256);
  } else {
    bus_->to_device(area_, OFF_PARAM, 16);
    if (op == GMAP_MERGE) bus_->to_device(area_, OFF_FRAME, 6400);
  }
  if (!(bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_AP_CTRL) & plio::AP_IDLE)) { *why = "core busy: not started"; return false; }
  bus_->wr64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_MAP_R_DATA, area_.pa);
  bus_->wr64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_DATA, area_.pa);
  bus_->wr64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FLAGS_DATA, flags_.pa);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GRID_MODE_DATA, 0u);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_PATH_CTRL_DATA, 0u);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GMAP_OP_DATA, op);
  const bool ok = bus_->start_and_wait(timeout_gmap_ms, &r->core_ns);
  bus_->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GMAP_OP_DATA, 0u);   // 规划路径不受上一次地图操作影响
  if (!ok) {
    dead_ = true;
    char b[160];
    std::snprintf(b, sizeof b, "gmap op %u: no ap_done within %d ms: core treated as hung, not started again "
                  "(reload the bitstream to reset the PL)", op, timeout_gmap_ms);
    *why = b;
    return false;
  }
  (void)bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_STATUS_CTRL);
  (void)bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_ROUNDS_CTRL);
  (void)bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_VALID_CTRL);
  const std::uint32_t st = bus_->rd(XPLANNER_FIELD_DUAL_CONTROL_ADDR_STATUS_DATA);
  // 输出：核写的、PS 会读的区作废缓存（T 图 PS 不碰）
  bus_->from_device(area_, OFF_RESULT, 16);
  if (op == GMAP_MERGE || op == GMAP_CLEAR) {
    bus_->from_device(area_, OFF_GLOBAL, 320000);
    bus_->from_device(area_, OFF_COARSE, 40000);
  } else if (op == GMAP_SCAN) {
    bus_->from_device(area_, OFF_COARSE, 40000);
    bus_->from_device(area_, OFF_IMAGE, 160000);
  } else if (op == GMAP_WINDOW) {
    bus_->from_device(area_, OFF_FRAME, 6400);
  }
  std::memcpy(r->w, area() + OFF_RESULT, 16);
  if (r->w[0] != st) {
    char b[120];
    std::snprintf(b, sizeof b, "gmap op %u: STATUS 0x%x != result word 0 0x%x", op, st, r->w[0]);
    *why = b;
    return false;
  }
  return true;
}

std::string Core::describe() const {
  char b[320];
  std::snprintf(b, sizeof b, "{\"core\":\"pl_planner\",\"bus\":%s,\"map_pa\":\"0x%llx\",\"field_pa\":\"0x%llx\",\"flags_pa\":\"0x%llx\",\"area_pa\":\"0x%llx\"}",
                bus_ ? bus_->describe().c_str() : "null", (unsigned long long)map_.pa, (unsigned long long)field_.pa,
                (unsigned long long)flags_.pa, (unsigned long long)area_.pa);
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

int planner_drv_run_path(void* h, int mode, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                         const std::uint8_t* params132, std::uint32_t* field, std::uint32_t* flags,
                         std::uint32_t* path_out, std::uint32_t* out3, std::uint64_t* core_ns) {
  Handle* p = static_cast<Handle*>(h);
  planner_drv::Request q;
  q.mode = mode;
  q.map = map;
  q.sx = sx; q.sy = sy; q.gx = gx; q.gy = gy;
  q.k_max = kmax;
  q.path_params = params132;
  q.path_out = path_out;
  planner_drv::Result r;
  if (!p->core.run(q, field, flags, &r, &p->err)) return -1;
  if (out3) { out3[0] = r.status; out3[1] = r.rounds; out3[2] = r.valid; }
  if (core_ns) *core_ns = r.core_ns;
  return 0;
}

std::uint8_t* planner_drv_gmap_area(void* h) { return static_cast<Handle*>(h)->core.area(); }

int planner_drv_gmap(void* h, unsigned op, std::uint32_t* out4, std::uint64_t* core_ns) {
  Handle* p = static_cast<Handle*>(h);
  planner_drv::GmapResult r;
  if (!p->core.gmap(op, &r, &p->err)) return -1;
  if (out4) std::memcpy(out4, r.w, 16);
  if (core_ns) *core_ns = r.core_ns;
  return 0;
}

}  // extern "C"
