// 驱动本体 = hw/planner/drv_gmap3/planner_core.cpp（GMAP3 快照，未改一个字）；本文件只加 C ABI：
//   planner_g3_open(wait, dma)            打开规划核；开机做一次 CLEAR 自检（结果字 0 = 0x200 才启用全局地图）
//   planner_g3_kv_nav_field(...)          与 libkv_nav 的 PlFieldFn 同签名：建场时同时开核内取路径（PATH_CTRL=1）
//       参数块 = PLNPATH0 的 PS 半边 make_params（hw/planner_dual/path/path_ps.hpp，原样调用），
//       核出的路径点再与 PS 现行取路（planner_split.hpp extract_from_field，同一个势场）逐点比，计数 match / mismatch。
//       返回值与旧驱动 planner_drv_kv_nav_field 完全相同（0 = 场就绪；失败 7 → kv_nav 软件势场）。
//   planner_g3_merge(h, cost80x80, bx, by) 把 kv_nav 的代价窗按 GMAP0 帧编码（254→0x64 占据、255→0xFF 未知、其余空闲）
//       经 MERGE 拼进 400×400 全局图（ix = bx + 200, iy = by + 200：窗基点格号 → 以 odom 原点为中心 ±10 m 的全局格号）。
//   planner_g3_scan(h)                    SCAN 一次，取已见格外接框（结果字 1..）。
//   planner_g3_stats(h, out[24])          计数（见 STAT_*），Python 侧周期性打成一行 JSON 进 kv_server 日志。
// 所有核访问持同一把互斥锁（建场、MERGE、SCAN 串行）。任何失败只计数并告警，返回旧语义，绝不抛到 kv_nav。
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "../drv_gmap3/planner_core.hpp"
#include "../../m7/pl/sw/gmap3_v9/m7pl_planner_regs.h"
#include "planner_split.hpp"
#include "path_ps.hpp"

extern "C" {
void* planner_drv_open(const char* wait, const char* dma);
const char* planner_drv_open_error(void);
void planner_drv_close(void* h);
const char* planner_drv_describe(void* h);
const char* planner_drv_error(void* h);
int planner_drv_run_path(void* h, int mode, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                         const std::uint8_t* params132, std::uint32_t* field, std::uint32_t* flags,
                         std::uint32_t* path_out, std::uint32_t* out3, std::uint64_t* core_ns);
std::uint8_t* planner_drv_gmap_area(void* h);
int planner_drv_gmap(void* h, unsigned op, std::uint32_t* out4, std::uint64_t* core_ns);
}
namespace planner_drv {
void* open_handle_on(std::unique_ptr<plio::Bus> bus, std::string* why);
Core* handle_core(void* h);
}

namespace {
enum Stat {
  ST_FIELD_CALLS, ST_FIELD_OK, ST_FIELD_FAIL, ST_PATH_CALLS, ST_PATH_FOUND, ST_PATH_NOPATH, ST_PATH_OTHER,
  ST_PATH_NOMARK, ST_PATH_MATCH, ST_PATH_MISMATCH, ST_LAST_HEADER, ST_LAST_NPTS, ST_LAST_START, ST_LAST_END,
  ST_GMAP_ENABLED, ST_MERGE_CALLS, ST_MERGE_OK, ST_MERGE_FAIL, ST_MERGE_DROPPED, ST_SCAN_CALLS, ST_SCAN_OK,
  ST_LAST_BBOX0, ST_LAST_BBOX1, ST_LAST_PATH_CORE_NS, ST_N
};
using E = kv_nav_planner::detail::Engine<80, 80, kv_nav_planner::kInfinity>;
using PS = plnpath::PS<80, 80, kv_nav_planner::kInfinity>;

struct G3 {
  void* drv = nullptr;
  std::mutex mu;
  std::uint64_t st[ST_N] = {0};
  std::string desc;
  std::vector<std::uint32_t> flags = std::vector<std::uint32_t>(planner_drv::FLAGS_WORDS);
  std::vector<std::uint32_t> path = std::vector<std::uint32_t>(1 + planner_drv::PATH_POINTS_MAX);
  std::vector<std::uint8_t> scratch = std::vector<std::uint8_t>(6400);
  std::vector<double> ox = std::vector<double>(6402 + 8), oy = std::vector<double>(6402 + 8),
                      cx = std::vector<double>(6402 + 8), cy = std::vector<double>(6402 + 8);
  int warn = 0;
};
std::string g_err;

void warnf(G3* g, const char* what, const char* why) {
  if (++g->warn <= 8 || g->warn % 200 == 0)
    std::fprintf(stderr, "{\"warning\":\"pl planner gmap3 (%d): %s: %s\"}\n", g->warn, what, why ? why : "");
}

bool gmap_selftest(G3* g) {
  // CLEAR：全局图写哨兵、粗图写"未知当可走"码（GMAP0 §5.2 默认表 0,0,254）
  std::uint8_t* a = planner_drv_gmap_area(g->drv);
  const std::uint8_t p[16] = {0x4D, 0x47, 0, 0, 0, 0, 0, 0, 254, 254, 205, 0, 0, 0, 0, 0};
  std::memcpy(a + planner_drv::OFF_PARAM, p, 16);
  std::uint32_t w[4] = {0};
  const int rc = planner_drv_gmap(g->drv, planner_drv::GMAP_CLEAR, w, nullptr);
  if (rc != 0 || w[0] != 0x200u) {
    char b[200];
    std::snprintf(b, sizeof b, "rc %d status 0x%x (%s)", rc, w[0], rc ? planner_drv_error(g->drv) : "want 0x200");
    warnf(g, "CLEAR self-test failed, global map disabled (not a GMAP3 core?)", b);
    return false;
  }
  return true;
}

G3* wrap(void* drv) {
  G3* g = new G3;
  g->drv = drv;
  g->st[ST_GMAP_ENABLED] = gmap_selftest(g) ? 1 : 0;
  return g;
}
}  // namespace

#ifdef PLNCHAIN_MOCK_OPEN
// 宿主端到端测试专用（make mocklib）：planner_g3_open 打开的是 mock 总线上的 GMAP3 v9 核 C 源，板上库不含这一段。
#include <functional>
#include "mock_g3.hpp"
static g3mock::Dev g_mock_dev;
static void* mock_drv_open() {
  std::string why;
  void* d = planner_drv::open_handle_on(std::unique_ptr<plio::Bus>(new plio::MockBus(std::ref(g_mock_dev))), &why);
  if (!d) g_err = why;
  return d;
}
#endif

extern "C" {
void* planner_g3_open(const char* wait, const char* dma) {
#ifdef PLNCHAIN_MOCK_OPEN
  (void)wait; (void)dma;
  void* d = mock_drv_open();
  if (!d) return nullptr;
#else
  void* d = planner_drv_open(wait, dma);
#endif
  if (!d) { g_err = planner_drv_open_error(); return nullptr; }
  return wrap(d);
}
// 测试用：在给定总线（MockBus）上开
void* planner_g3_open_on(void* bus_raw) {
  std::string why;
  void* d = planner_drv::open_handle_on(std::unique_ptr<plio::Bus>(static_cast<plio::Bus*>(bus_raw)), &why);
  if (!d) { g_err = why; return nullptr; }
  return wrap(d);
}
const char* planner_g3_open_error(void) { return g_err.c_str(); }
void planner_g3_close(void* h) {
  G3* g = static_cast<G3*>(h);
  planner_drv_close(g->drv);
  delete g;
}
const char* planner_g3_describe(void* h) {
  G3* g = static_cast<G3*>(h);
  g->desc = std::string("{\"pln\":\"gmap3\",\"gmap_enabled\":") + (g->st[ST_GMAP_ENABLED] ? "true" : "false") +
            ",\"drv\":" + planner_drv_describe(g->drv) + "}";
  return g->desc.c_str();
}

int planner_g3_kv_nav_field(void* h, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                            std::uint32_t* potential, unsigned* rounds) {
  G3* g = static_cast<G3*>(h);
  std::lock_guard<std::mutex> lk(g->mu);
  ++g->st[ST_FIELD_CALLS];
  kv_nav_planner::Request q{sx, sy, double(gx), double(gy)};
  // PS 半边：工作图 + 参数块（只依赖地图与请求）
  E::prepare(map, sx, sy, g->scratch.data());
  const plnpath::PsParams pp = PS::make_params(g->scratch.data(), q, 0.05);
  std::uint32_t out3[3] = {0, 0, 0};
  std::uint64_t ns = 0;
  g->path[0] = 0;
  const int rc = planner_drv_run_path(g->drv, 0, map, sx, sy, gx, gy, kmax, pp.bytes, potential, g->flags.data(),
                                      g->path.data(), out3, &ns);
  // 驱动快照把 PATH_CTRL 留在 1；m5 旧驱动从不写 0x90。同一次加载里之后若有进程用旧驱动，核会按取路径往 flags 写
  // 64+1+4096 字、越过旧驱动 256 字的缓冲（test_old_drv_on_g3 对照 ②）。所以每次用完立刻清 0。
  if (!planner_drv::handle_core(g->drv)->dead())
    planner_drv::handle_core(g->drv)->bus()->wr(XPLANNER_FIELD_DUAL_CONTROL_ADDR_PATH_CTRL_DATA, 0u);
  if (rc != 0) {
    ++g->st[ST_FIELD_FAIL];
    warnf(g, "field", planner_drv_error(g->drv));
    return planner_drv::NOT_CONVERGED;   // kv_nav: software field, the plan does not fail
  }
  ++g->st[ST_FIELD_OK];
  if (rounds) *rounds = out3[1];
  const std::uint32_t hdr = g->path[0];
  g->st[ST_LAST_HEADER] = hdr;
  g->st[ST_LAST_PATH_CORE_NS] = ns;
  if ((hdr >> 28) != 0xAu) {
    ++g->st[ST_PATH_NOMARK];   // 核没写表头：不是 GMAP3/PLNPATH0 核（例如 m5 比特流）
    if (g->st[ST_PATH_NOMARK] == 1 || g->st[ST_PATH_NOMARK] % 500 == 0)
      warnf(g, "path header marker missing (core without path fetch?)", "");
    return (int)out3[0];
  }
  ++g->st[ST_PATH_CALLS];
  const unsigned pst = (hdr >> 24) & 15u, n = hdr & 0xFFFFu;
  g->st[ST_LAST_NPTS] = n;
  if (n) {
    g->st[ST_LAST_START] = g->path[1] & 0xFFFFu;
    g->st[ST_LAST_END] = g->path[n] & 0xFFFFu;
  }
  if (pst == plnpath::P_EXACT || pst == plnpath::P_TOL) ++g->st[ST_PATH_FOUND];
  else if (pst == plnpath::P_NO_PATH || pst == plnpath::P_SAFETY) ++g->st[ST_PATH_NOPATH];
  else ++g->st[ST_PATH_OTHER];
  // 与 PS 现行取路（同一势场、同一请求）逐点比：只在场就绪时
  if (out3[0] == 0) {
    const kv_nav_planner::Result a = PS::assemble(g->path.data(), pp, q, g->cx.data(), g->cy.data());
    const kv_nav_planner::Result b = kv_nav_planner::extract_from_field(map, q, potential, out3[1], g->scratch.data(),
                                                                        g->ox.data(), g->oy.data(), 6402);
    bool same = a.status == b.status;
    if (same && (a.status == kv_nav_planner::EXACT || a.status == kv_nav_planner::WITHIN_TOLERANCE)) {
      same = a.count == b.count;
      for (int i = 0; same && i < a.count; ++i) same = g->cx[i] == g->ox[i] && g->cy[i] == g->oy[i];
    }
    if (same) ++g->st[ST_PATH_MATCH];
    else {
      ++g->st[ST_PATH_MISMATCH];
      char b2[120];
      std::snprintf(b2, sizeof b2, "core status %d n %d vs ps status %d n %d", a.status, a.count, b.status, b.count);
      warnf(g, "path differs from PS extraction", b2);
    }
  }
  return (int)out3[0];
}

// cost: 80x80 kv_nav 代价窗（0-252 空闲/膨胀、253 内切、254 致命、255 未知），bx/by 窗基点格号
int planner_g3_merge(void* h, const std::uint8_t* cost, int bx, int by) {
  G3* g = static_cast<G3*>(h);
  std::lock_guard<std::mutex> lk(g->mu);
  if (!g->st[ST_GMAP_ENABLED]) return -1;
  ++g->st[ST_MERGE_CALLS];
  const int ix = bx + 200, iy = by + 200;
  if (ix < -32768 || ix > 32767 || iy < -32768 || iy > 32767) { ++g->st[ST_MERGE_FAIL]; return -1; }
  std::uint8_t* a = planner_drv_gmap_area(g->drv);
  std::uint8_t* fr = a + planner_drv::OFF_FRAME;
  for (int i = 0; i < 6400; ++i) fr[i] = cost[i] == 255 ? 0xFF : (cost[i] == 254 ? 0x64 : 0x00);
  const std::uint8_t p[16] = {0x4D, 0x47, std::uint8_t(ix & 255), std::uint8_t((ix >> 8) & 255), std::uint8_t(iy & 255),
                              std::uint8_t((iy >> 8) & 255), 0, 0, 254, 254, 205, 0, 0, 0, 0, 0};
  std::memcpy(a + planner_drv::OFF_PARAM, p, 16);
  std::uint32_t w[4] = {0};
  if (planner_drv_gmap(g->drv, planner_drv::GMAP_MERGE, w, nullptr) != 0 || (w[0] & 0xFFu) != 0) {
    ++g->st[ST_MERGE_FAIL];
    warnf(g, "MERGE", planner_drv_error(g->drv));
    return -1;
  }
  ++g->st[ST_MERGE_OK];
  g->st[ST_MERGE_DROPPED] += w[1];
  return 0;
}

int planner_g3_scan(void* h) {
  G3* g = static_cast<G3*>(h);
  std::lock_guard<std::mutex> lk(g->mu);
  if (!g->st[ST_GMAP_ENABLED]) return -1;
  ++g->st[ST_SCAN_CALLS];
  std::uint8_t* a = planner_drv_gmap_area(g->drv);
  const std::uint8_t p[16] = {0x4D, 0x47, 0, 0, 0, 0, 0, 0, 254, 0x00, 0xFF, 0x64, 0, 0, 0, 0};   // 三态表，不翻转
  std::memcpy(a + planner_drv::OFF_PARAM, p, 16);
  std::uint32_t w[4] = {0};
  if (planner_drv_gmap(g->drv, planner_drv::GMAP_SCAN, w, nullptr) != 0 || (w[0] & 0x5u) != 0) {
    warnf(g, "SCAN", planner_drv_error(g->drv));
    return -1;
  }
  ++g->st[ST_SCAN_OK];
  g->st[ST_LAST_BBOX0] = w[2];
  g->st[ST_LAST_BBOX1] = w[3];
  return (int)w[0];
}

int planner_g3_stats(void* h, std::uint64_t* out, int n) {
  G3* g = static_cast<G3*>(h);
  std::lock_guard<std::mutex> lk(g->mu);
  const int k = n < ST_N ? n : ST_N;
  std::memcpy(out, g->st, sizeof(std::uint64_t) * k);
  return ST_N;
}
}  // extern "C"
