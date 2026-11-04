// mock 内存上跑，驱动读回的 status/rounds/valid＋势场＋标志对 expected.bin 逐字（186,904 字节）。
// 另核 kv_nav 钩子（planner_drv_kv_nav_field，模式 0）给出的势场＝黄金；阳性对照：改 1 个代价字节必须被输入比较抓到；
// 核挂死时驱动判挂死、钩子返回 7（kv_nav 退回软件势场）、之后不再启动。
//   test_planner_drv <input.bin> <expected.bin>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "../../m7/pl/sw/m7pl_planner_regs.h"
#include "kernel_hls.hpp"
#include "planner_core.hpp"

static int fails = 0;
#define EXPECT(c, ...) do { if (!(c)) { std::printf("FAIL "); fails++; } else std::printf("ok   "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)

static std::vector<uint8_t> slurp(const char* p) {
  std::ifstream f(p, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
static uint32_t u32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

struct Case { uint32_t mode, sx, sy, gx, gy, k; std::vector<uint8_t> map; std::vector<uint32_t> exp; };

struct Dev {
  std::vector<uint8_t> map_seen;
  uint32_t regs_seen[6] = {0};
  void operator()(plio::MockBus& m) {
    const uint32_t mode = m.reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GRID_MODE_DATA);
    const int n = planner_drv::cells(mode == 1), nf = planner_drv::nflags(mode == 1);
    const uint8_t* mp = (const uint8_t*)m.va_of(m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_MAP_R_DATA), n);
    uint8_t* fp = (uint8_t*)m.va_of(m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_DATA), (size_t)n * 4);
    uint8_t* gp = (uint8_t*)m.va_of(m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FLAGS_DATA), (size_t)nf * 4);
    if (!mp || !fp || !gp) { std::printf("MOCK: core address outside the driver's buffers\n"); return; }
    map_seen.assign(mp, mp + n);
    const uint32_t offs[6] = {XPLANNER_FIELD_DUAL_CONTROL_ADDR_SX_DATA, XPLANNER_FIELD_DUAL_CONTROL_ADDR_SY_DATA,
                              XPLANNER_FIELD_DUAL_CONTROL_ADDR_GX_DATA, XPLANNER_FIELD_DUAL_CONTROL_ADDR_GY_DATA,
                              XPLANNER_FIELD_DUAL_CONTROL_ADDR_K_MAX_DATA, XPLANNER_FIELD_DUAL_CONTROL_ADDR_GRID_MODE_DATA};
    for (int i = 0; i < 6; ++i) regs_seen[i] = m.reg(offs[i]);
    static ap_uint<8> map[40000];
    static ap_uint<32> field[40000], flags[256];
    for (int i = 0; i < n; ++i) map[i] = mp[i];
    ap_uint<32> st, rounds, valid;
    planner_field_dual(map, field, flags, (int)regs_seen[0], (int)regs_seen[1], (int)regs_seen[2], (int)regs_seen[3],
                       regs_seen[4], st, rounds, valid, mode);
    for (int i = 0; i < n; ++i) { uint32_t v = field[i].to_uint(); std::memcpy(fp + 4 * i, &v, 4); }
    for (int i = 0; i < nf; ++i) { uint32_t v = flags[i].to_uint(); std::memcpy(gp + 4 * i, &v, 4); }
    m.set_reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_STATUS_DATA, st.to_uint());
    m.set_reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_ROUNDS_DATA, rounds.to_uint());
    m.set_reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_VALID_DATA, valid.to_uint());
  }
};

int main(int argc, char** argv) {
  if (argc != 3) { std::fprintf(stderr, "usage: %s input.bin expected.bin\n", argv[0]); return 2; }
  const auto in = slurp(argv[1]), ex = slurp(argv[2]);
  EXPECT(in.size() == 46460 && ex.size() == 186916 && !std::memcmp(in.data(), "FPGADIN01", 8) &&
         !std::memcmp(ex.data(), "FPGADGO01", 8) && u32(&in[8]) == 2 && u32(&ex[8]) == 2,
         "dev2 frozen-pair input.bin / expected.bin (2 cases)");
  std::vector<Case> cs;
  size_t pi = 12, pe = 12;
  for (int c = 0; c < 2; ++c) {
    Case k;
    k.mode = u32(&in[pi]); k.sx = u32(&in[pi + 4]); k.sy = u32(&in[pi + 8]); k.gx = u32(&in[pi + 12]);
    k.gy = u32(&in[pi + 16]); k.k = u32(&in[pi + 20]);
    pi += 24;
    const int n = planner_drv::cells(k.mode == 1), nf = planner_drv::nflags(k.mode == 1);
    k.map.assign(in.begin() + pi, in.begin() + pi + n);
    pi += n;
    for (int i = 0; i < 3 + n + nf; ++i, pe += 4) k.exp.push_back(u32(&ex[pe]));
    cs.push_back(k);
  }
  EXPECT(pi == in.size() && pe == ex.size(), "parsed both files to the end");

  Dev dev;
  std::string why;
  void* h = planner_drv::open_handle_on(std::unique_ptr<plio::Bus>(new plio::MockBus(std::ref(dev))), &why);
  EXPECT(h != nullptr, "driver handle on the mock bus %s", why.c_str());
  if (!h) return 1;
  planner_drv::Core* core = planner_drv::handle_core(h);
  plio::MockBus* mb = static_cast<plio::MockBus*>(core->bus());
  std::vector<uint32_t> outputs;   // = golden/outputs.bin 的字序
  for (size_t c = 0; c < cs.size(); ++c) {
    const Case& k = cs[c];
    const int n = planner_drv::cells(k.mode == 1), nf = planner_drv::nflags(k.mode == 1);
    std::vector<uint32_t> field(n), flags(nf);
    uint32_t o3[3];
    uint64_t ns = 0;
    const int rc = planner_drv_run(h, (int)k.mode, k.map.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy, k.k,
                                   field.data(), flags.data(), o3, &ns);
    EXPECT(rc == 0, "case %zu mode %u: driver run %s", c, k.mode, rc ? planner_drv_error(h) : "");
    EXPECT(dev.map_seen == k.map, "case %zu: DMA map buffer == dev2 golden input, %d bytes", c, n);
    const uint32_t want[6] = {k.sx, k.sy, k.gx, k.gy, k.k, k.mode};
    EXPECT(!std::memcmp(dev.regs_seen, want, sizeof want), "case %zu: SX SY GX GY K_MAX GRID_MODE = %u %u %u %u %u %u",
           c, dev.regs_seen[0], dev.regs_seen[1], dev.regs_seen[2], dev.regs_seen[3], dev.regs_seen[4], dev.regs_seen[5]);
    std::vector<uint32_t> got = {o3[0], o3[1], o3[2]};
    got.insert(got.end(), field.begin(), field.end());
    got.insert(got.end(), flags.begin(), flags.end());
    EXPECT(got == k.exp, "case %zu: status %u rounds %u valid %u + field + flags (C core via the driver) == expected.bin",
           c, o3[0], o3[1], o3[2]);
    outputs.insert(outputs.end(), got.begin(), got.end());
    if (k.mode == 0) {
      // kv_nav 钩子（模式 0、只要势场）
      std::vector<uint32_t> pot(6400, 0);
      unsigned rounds = 0;
      const int st = planner_drv_kv_nav_field(h, k.map.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy, k.k,
                                              pot.data(), &rounds);
      EXPECT(st == (int)k.exp[0] && rounds == k.exp[1] && std::equal(pot.begin(), pot.end(), k.exp.begin() + 3),
             "case %zu: kv_nav hook status %d rounds %u, potential == golden field", c, st, rounds);
      // 阳性对照：改 1 个代价字节（一个可走格加 1），输入比较必须抓到
      std::vector<uint8_t> m2 = k.map;
      size_t at = 0;
      while (at < m2.size() && m2[at] >= 252) ++at;
      m2[at] += 1;
      planner_drv_kv_nav_field(h, m2.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy, k.k, pot.data(), &rounds);
      size_t nd = 0;
      for (size_t i = 0; i < m2.size(); ++i) nd += dev.map_seen[i] != k.map[i];
      EXPECT(nd == 1, "case %zu: control: one changed cost byte -> input comparison reports %zu byte", c, nd);
    }
  }
  std::printf("outputs: %zu bytes\n", outputs.size() * 4);
  EXPECT(outputs.size() * 4 == 186904, "all outputs = 186,904 bytes (golden/outputs.bin length)");
  // 阳性对照：核挂死 → 钩子返回 7、判挂死、之后不再启动
  mb->hang_next = true;
  std::vector<uint32_t> pot(6400);
  unsigned rounds = 0;
  const Case& k = cs[0];
  int st = planner_drv_kv_nav_field(h, k.map.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy, k.k, pot.data(), &rounds);
  EXPECT(st == planner_drv::NOT_CONVERGED && core->dead(), "control: hung core -> hook returns 7 (software field), marked hung");
  const int starts = mb->starts;
  st = planner_drv_kv_nav_field(h, k.map.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy, k.k, pot.data(), &rounds);
  EXPECT(st == planner_drv::NOT_CONVERGED && mb->starts == starts, "hung core never started again");
  planner_drv_close(h);
  std::printf(fails ? "PLANNER DRV CHECK FAIL (%d)\n" : "PLANNER DRV CHECK PASS\n", fails);
  return fails ? 1 : 0;
}
