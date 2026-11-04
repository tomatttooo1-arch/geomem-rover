// PLNCHAIN0 宿主关键对照：kv_nav 薄壳（kv_nav_gmap3.cpp）＋ GMAP3 驱动快照 ＋ GMAP3 v9 核 C 源（mock，32 位指针）。
//     返回值/轮数与旧驱动钩子语义相同；核内取路径的点串 == PS 现行取路（planner_split.hpp extract_from_field）。
//  2. 覆盖：200 张合成 80×80 代价窗（随机障碍、随机起终点，含不可达/容差终点），核路径与 PS 取路逐点比，mismatch 必须 0。
//  3. 阳性对照：mock 在核跑完后改 1 个路径点 → mismatch 计数必须涨。
//  4. 全局地图：开机 CLEAR 自检过（gmap_enabled=1）；planner_g3_merge 三次（含部分出图）后地图区 == C 模型
//     （tools/gmap/gmap_model.c 同样输入）逐字节；SCAN 外接框非空。
//  5. 回落：同一壳跑在"m5 核"（没有 0x90/0x98）上：CLEAR 自检失败 → 全局地图关；取路径表头缺失只计数；势场仍 == 黄金。
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include "mock_g3.hpp"
extern "C" {
#include "gmap_model.c"
void* planner_g3_open_on(void* bus);
const char* planner_g3_open_error(void);
void planner_g3_close(void* h);
const char* planner_g3_describe(void* h);
int planner_g3_kv_nav_field(void* h, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                            std::uint32_t* potential, unsigned* rounds);
int planner_g3_merge(void* h, const std::uint8_t* cost, int bx, int by);
int planner_g3_scan(void* h);
int planner_g3_stats(void* h, std::uint64_t* out, int n);
std::uint8_t* planner_drv_gmap_area(void* h);
}
using namespace g3mock;
enum { FIELD_CALLS, FIELD_OK, FIELD_FAIL, PATH_CALLS, PATH_FOUND, PATH_NOPATH, PATH_OTHER, PATH_NOMARK, PATH_MATCH,
       PATH_MISMATCH, LAST_HEADER, LAST_NPTS, LAST_START, LAST_END, GMAP_ENABLED, MERGE_CALLS, MERGE_OK, MERGE_FAIL,
       MERGE_DROPPED, SCAN_CALLS, SCAN_OK, BBOX0, BBOX1, PATH_NS, NST };
struct G3View { void* drv; };   // 壳句柄首成员 = 驱动句柄（只用来取地图区）
static std::vector<std::uint64_t> stats(void* h) { std::vector<std::uint64_t> s(NST); planner_g3_stats(h, s.data(), NST); return s; }

int main(int argc, char** argv) {
  if (argc < 3) { std::printf("usage: %s input.bin expected.bin\n", argv[0]); return 2; }
  std::vector<Case> cs;
  EXPECT(load_frozen(argv[1], argv[2], cs), "dev2 frozen-pair parsed");
  const Case& k0 = cs[0];
  EXPECT(k0.mode == 0, "case 0 is the 80 grid (kv_nav window size)");
  // ---------- GMAP3 核
  Dev dev;
  plio::MockBus* mb = new plio::MockBus(std::ref(dev));   // 句柄接管所有权
  void* h = planner_g3_open_on(mb);
  EXPECT(h, "gmap3 chain shim on mock %s", h ? planner_g3_describe(h) : planner_g3_open_error());
  if (!h) return 1;
  auto s = stats(h);
  EXPECT(s[GMAP_ENABLED] == 1 && dev.gmap_calls == 1, "open: CLEAR self-test passed (status 0x200), global map enabled");
  std::vector<std::uint32_t> pot(6400);
  unsigned rounds = 0;
  int st = planner_g3_kv_nav_field(h, k0.map.data(), (int)k0.sx, (int)k0.sy, (int)k0.gx, (int)k0.gy, k0.k, pot.data(), &rounds);
  s = stats(h);
  EXPECT(st == (int)k0.exp[0] && rounds == k0.exp[1] && std::equal(pot.begin(), pot.end(), k0.exp.begin() + 3),
         "golden: field hook with path fetch on: status %d rounds %u, potential == expected.bin (6400 words)", st, rounds);
  EXPECT(dev.path_ctrl == 1 && dev.hi_seen == 0 && dev.oob == 0, "core saw PATH_CTRL=1, 32-bit pointers, no out-of-buffer access");
  EXPECT(mb->reg(g3mock::A_PATH) == 0 && mb->reg(g3mock::A_GMAP) == 0,
         "after the field call the shim left PATH_CTRL 0 and GMAP_OP 0 (safe for a later m5-driver process)");
  EXPECT(s[PATH_CALLS] == 1 && s[PATH_FOUND] == 1 && s[PATH_MATCH] == 1 && s[PATH_MISMATCH] == 0,
         "golden: core path header 0x%08llx, %llu points (start 0x%04llx -> end 0x%04llx) == PS extract_from_field point by point",
         (unsigned long long)s[LAST_HEADER], (unsigned long long)s[LAST_NPTS], (unsigned long long)s[LAST_START],
         (unsigned long long)s[LAST_END]);
  // ---------- 合成代价窗
  std::mt19937 rng(20261104u);
  int nconv = 0;
  for (int t = 0; t < 200; ++t) {
    std::vector<std::uint8_t> m(6400);
    for (auto& v : m) v = std::uint8_t(rng() % 120);
    const int blobs = 3 + rng() % 25;
    for (int b = 0; b < blobs; ++b) {
      const int cx = rng() % 80, cy = rng() % 80, r = 1 + rng() % 6;
      const std::uint8_t val = (rng() % 4 == 0) ? 255 : (rng() % 3 == 0 ? 253 : 254);
      for (int y = std::max(0, cy - r); y < std::min(80, cy + r + 1); ++y)
        for (int x = std::max(0, cx - r); x < std::min(80, cx + r + 1); ++x) m[y * 80 + x] = val;
    }
    int sx, sy, gx, gy;
    do { sx = 1 + rng() % 78; sy = 1 + rng() % 78; } while (m[sy * 80 + sx] >= 253);
    gx = 1 + rng() % 78; gy = 1 + rng() % 78;
    // t%5==0：终点格空闲但被一圈致命格围住（切比雪夫 1）→ 精确终点不可达，走容差候选；
    // t%5==1：围在切比雪夫 4 的方环里（容差 ±3 格全在环内）→ NO_PATH；起点落在环内就换一个
    if (t % 5 <= 1) {
      const int rr = t % 5 == 0 ? 1 : 4;
      for (int y = gy - rr; y <= gy + rr; ++y)
        for (int x = gx - rr; x <= gx + rr; ++x)
          if (x >= 0 && y >= 0 && x < 80 && y < 80) {
            const bool ring = std::max(std::abs(x - gx), std::abs(y - gy)) == rr;
            m[y * 80 + x] = ring ? 254 : 10;
          }
      while (std::max(std::abs(sx - gx), std::abs(sy - gy)) <= rr || m[sy * 80 + sx] >= 253) { sx = 1 + rng() % 78; sy = 1 + rng() % 78; }
    }
    st = planner_g3_kv_nav_field(h, m.data(), sx, sy, gx, gy, 64, pot.data(), &rounds);
    nconv += st == 0;
  }
  s = stats(h);
  EXPECT(s[PATH_MISMATCH] == 0 && s[PATH_NOMARK] == 0 && s[FIELD_FAIL] == 0,
         "200 synthetic windows: fields ready %d, path found %llu, no-path %llu, other %llu; core path == PS extraction in %llu, mismatch %llu",
         nconv, (unsigned long long)s[PATH_FOUND] - 1, (unsigned long long)s[PATH_NOPATH], (unsigned long long)s[PATH_OTHER],
         (unsigned long long)s[PATH_MATCH] - 1, (unsigned long long)s[PATH_MISMATCH]);
  EXPECT(s[PATH_FOUND] > 50 && s[PATH_NOPATH] > 0, "coverage: both found and not-found paths exercised");
  // ---------- 阳性对照：改 1 个路径点
  dev.corrupt_path = true;
  planner_g3_kv_nav_field(h, k0.map.data(), (int)k0.sx, (int)k0.sy, (int)k0.gx, (int)k0.gy, k0.k, pot.data(), &rounds);
  dev.corrupt_path = false;
  const auto s2 = stats(h);
  EXPECT(s2[PATH_MISMATCH] == s[PATH_MISMATCH] + 1, "control: one path point flipped after the core -> mismatch counter +1");
  // ---------- 全局地图 MERGE / SCAN 对 C 模型
  std::uint8_t* area = planner_drv_gmap_area(static_cast<G3View*>(h)->drv);
  std::vector<std::uint8_t> ref(area, area + kGmapAreaBytes);
  const int pos[3][2] = {{-40, -40}, {-35, -38}, {150, 170}};   // 第三次大部分出图（全局 400 格 = ±200）
  bool merges_ok = true;
  for (int i = 0; i < 3; ++i) {
    std::vector<std::uint8_t> cost(6400);
    for (int j = 0; j < 6400; ++j) cost[j] = std::uint8_t(j % 7 == 0 ? 254 : j % 11 == 0 ? 255 : j % 13 == 0 ? 253 : (j * 31) % 200);
    const int rc = planner_g3_merge(h, cost.data(), pos[i][0], pos[i][1]);
    const int ix = pos[i][0] + 200, iy = pos[i][1] + 200;
    for (int j = 0; j < 6400; ++j) ref[GMAP_OFF_FRAME + j] = cost[j] == 255 ? 0xFF : (cost[j] == 254 ? 0x64 : 0);
    const std::uint8_t p[16] = {0x4D, 0x47, std::uint8_t(ix & 255), std::uint8_t(ix >> 8), std::uint8_t(iy & 255),
                                std::uint8_t(iy >> 8), 0, 0, 254, 254, 205, 0, 0, 0, 0, 0};
    std::memcpy(&ref[GMAP_OFF_PARAM], p, 16);
    gmap_run(ref.data(), GMAP_OP_MERGE);
    size_t nd = 0;
    for (int j = 0; j < kGmapAreaBytes; ++j) nd += area[j] != ref[j];
    std::uint32_t dropped; std::memcpy(&dropped, &ref[GMAP_OFF_RESULT + 4], 4);
    std::printf("     merge %d at (%d,%d): rc %d, diff bytes vs C model %zu, dropped %u\n", i, ix, iy, rc, nd, dropped);
    merges_ok = merges_ok && rc == 0 && nd == 0;
  }
  s = stats(h);
  EXPECT(merges_ok && s[MERGE_OK] == 3 && s[MERGE_DROPPED] > 0,
         "3 MERGEs via the shim: whole map area == C model after each, dropped cells total %llu", (unsigned long long)s[MERGE_DROPPED]);
  const int sc = planner_g3_scan(h);
  std::memcpy(&ref[GMAP_OFF_PARAM], area + GMAP_OFF_PARAM, 16);
  const std::uint32_t mst = gmap_run(ref.data(), GMAP_OP_SCAN);
  size_t nd = 0;
  for (int j = 0; j < kGmapAreaBytes; ++j) nd += area[j] != ref[j];
  s = stats(h);
  EXPECT(sc >= 0 && (std::uint32_t)sc == mst && nd == 0 && s[BBOX0] != 0,
         "SCAN: status 0x%x == model, area == model, bbox words 0x%08llx 0x%08llx", sc,
         (unsigned long long)s[BBOX0], (unsigned long long)s[BBOX1]);
  // 规划在地图操作之后仍对（GMAP_OP 被清 0）
  st = planner_g3_kv_nav_field(h, k0.map.data(), (int)k0.sx, (int)k0.sy, (int)k0.gx, (int)k0.gy, k0.k, pot.data(), &rounds);
  EXPECT(st == 0 && std::equal(pot.begin(), pot.end(), k0.exp.begin() + 3) && dev.gmap_op == 0,
         "after map ops: planning still == golden (GMAP_OP back to 0)");
  planner_g3_close(h);
  // ---------- 回落：m5 核
  Dev d5;
  d5.m5_core = true;
  void* h5 = planner_g3_open_on(new plio::MockBus(std::ref(d5)));
  EXPECT(h5 != nullptr, "shim opens on an m5-like core (no 0x90/0x98)");
  if (h5) {
    st = planner_g3_kv_nav_field(h5, k0.map.data(), (int)k0.sx, (int)k0.sy, (int)k0.gx, (int)k0.gy, k0.k, pot.data(), &rounds);
    s = stats(h5);
    EXPECT(s[GMAP_ENABLED] == 0 && s[PATH_NOMARK] == 1 && s[PATH_CALLS] == 0 && st == 0 &&
               std::equal(pot.begin(), pot.end(), k0.exp.begin() + 3) && planner_g3_merge(h5, k0.map.data(), 0, 0) == -1,
           "fallback on m5 core: global map off, path header missing counted (%llu), field still == golden",
           (unsigned long long)s[PATH_NOMARK]);
    planner_g3_close(h5);
  }
  std::printf(g_fails ? "PLNCHAIN0 CHAIN CHECK FAIL (%d)\n" : "PLNCHAIN0 CHAIN CHECK PASS\n", g_fails);
  return g_fails ? 1 : 0;
}
