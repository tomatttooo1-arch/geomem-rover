// mock 内存上跑，驱动读回的 status/rounds/valid＋势场＋标志对 expected.bin 逐字（186,904 字节）。
// 另核 kv_nav 钩子（planner_drv_kv_nav_field，模式 0）给出的势场＝黄金；阳性对照：改 1 个代价字节必须被输入比较抓到；
// 核挂死时驱动判挂死、钩子返回 7（kv_nav 退回软件势场）、之后不再启动。
// GMAP3（11-04）：核换成本分支的 kernel_hls.cpp（15 个参数，最后一个 gmap_op）。另加：
//   取路径（PATH_CTRL=1，参数块 path_params.bin）：两例经驱动的 scalars＋field＋flags＋表头＋路径点按 testbench
//   的字序写 build/drv_path_outputs.bin，Makefile 核 sha256 = 原生 tb 的 86b2d252…；不取路径的同样写
//   build/drv_legacy_outputs.bin（= 0036fda9…）。
//   地图操作（GMAP_OP 1–4）：两组联仿向量（GMVEC002，vecA/vecB）经驱动的 area() 与 planner_drv_gmap 回放，
//   每个操作之后整个地图区与 C 模型（tools/gmap/gmap_model.c）逐字节相同，MAP = FIELD = 地图区基址、GMAP_OP 写对
//   且事后清 0；阳性对照：驱动侧帧改 1 字节必须被抓到。
//   test_planner_drv <input.bin> <expected.bin> [<path_params.bin> <vecA.bin> <vecB.bin> <outdir>]
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "../../m7/pl/sw/gmap3_v9/m7pl_planner_regs.h"
#include "kernel_hls.hpp"
#include "planner_core.hpp"
extern "C" {
#include "gmap_model.c"
}

static int fails = 0;
#define EXPECT(c, ...) do { if (!(c)) { std::printf("FAIL "); fails++; } else std::printf("ok   "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)

static std::vector<uint8_t> slurp(const char* p) {
  std::ifstream f(p, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
static uint32_t u32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

struct Case { uint32_t mode, sx, sy, gx, gy, k; std::vector<uint8_t> map; std::vector<uint32_t> exp; };

// mock 上的核：快照输入，跑本分支核的 C 源，写回 DDR 与输出寄存器
struct Dev {
  std::vector<uint8_t> map_seen;
  uint32_t regs_seen[6] = {0};
  uint32_t path_ctrl = 0, gmap_op = 0, gmap_calls = 0, gmap_ptr_bad = 0;
  void operator()(plio::MockBus& m) {
    path_ctrl = m.reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_PATH_CTRL_DATA);
    gmap_op = m.reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GMAP_OP_DATA);
    static ap_uint<8> map[kGmapAreaBytes];
    static ap_uint<32> field[kGmapAreaWords], flags[kPlnFlagsDepth];
    ap_uint<32> st, rounds, valid;
    if (gmap_op != 0) {
      ++gmap_calls;
      const uint64_t a = m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_MAP_R_DATA);
      uint8_t* ap = (uint8_t*)m.va_of(a, kGmapAreaBytes);
      if (!ap || m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_DATA) != a) { ++gmap_ptr_bad; return; }
      for (int i = 0; i < kGmapAreaBytes; ++i) map[i] = ap[i];
      for (int i = 0; i < kGmapAreaWords; ++i) { uint32_t v; std::memcpy(&v, ap + 4 * i, 4); field[i] = v; }
      planner_field_dual(map, field, flags, 0, 0, 0, 0, 0, st, rounds, valid, 0, 0, gmap_op);
      for (int i = 0; i < kGmapAreaWords; ++i) { uint32_t v = field[i].to_uint(); std::memcpy(ap + 4 * i, &v, 4); }
    } else {
      const uint32_t mode = m.reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GRID_MODE_DATA);
      const int n = planner_drv::cells(mode == 1), nf = planner_drv::nflags(mode == 1);
      const int nm = n + (path_ctrl & 1 ? planner_drv::PATH_PARAM_BYTES : 0);
      const int nfw = path_ctrl & 1 ? nf + 1 + planner_drv::PATH_POINTS_MAX : nf;
      const uint8_t* mp = (const uint8_t*)m.va_of(m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_MAP_R_DATA), nm);
      uint8_t* fp = (uint8_t*)m.va_of(m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_DATA), (size_t)n * 4);
      uint8_t* gp = (uint8_t*)m.va_of(m.reg64(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FLAGS_DATA), (size_t)nfw * 4);
      if (!mp || !fp || !gp) { std::printf("MOCK: core address outside the driver's buffers\n"); return; }
      map_seen.assign(mp, mp + nm);
      const uint32_t offs[6] = {XPLANNER_FIELD_DUAL_CONTROL_ADDR_SX_DATA, XPLANNER_FIELD_DUAL_CONTROL_ADDR_SY_DATA,
                                XPLANNER_FIELD_DUAL_CONTROL_ADDR_GX_DATA, XPLANNER_FIELD_DUAL_CONTROL_ADDR_GY_DATA,
                                XPLANNER_FIELD_DUAL_CONTROL_ADDR_K_MAX_DATA, XPLANNER_FIELD_DUAL_CONTROL_ADDR_GRID_MODE_DATA};
      for (int i = 0; i < 6; ++i) regs_seen[i] = m.reg(offs[i]);
      for (int i = 0; i < nm; ++i) map[i] = mp[i];
      for (int i = 0; i < nfw; ++i) { uint32_t v; std::memcpy(&v, gp + 4 * i, 4); flags[i] = v; }
      planner_field_dual(map, field, flags, (int)regs_seen[0], (int)regs_seen[1], (int)regs_seen[2], (int)regs_seen[3],
                         regs_seen[4], st, rounds, valid, mode, path_ctrl, 0);
      for (int i = 0; i < n; ++i) { uint32_t v = field[i].to_uint(); std::memcpy(fp + 4 * i, &v, 4); }
      for (int i = 0; i < nfw; ++i) { uint32_t v = flags[i].to_uint(); std::memcpy(gp + 4 * i, &v, 4); }
    }
    m.set_reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_STATUS_DATA, st.to_uint());
    m.set_reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_ROUNDS_DATA, rounds.to_uint());
    m.set_reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_FIELD_VALID_DATA, valid.to_uint());
  }
};

static void put_words(std::vector<uint8_t>& o, const uint32_t* w, size_t n) {
  for (size_t i = 0; i < n; ++i) for (int j = 0; j < 4; ++j) o.push_back(uint8_t(w[i] >> (8 * j)));
}
static bool write_file(const std::string& p, const std::vector<uint8_t>& b) {
  std::ofstream f(p, std::ios::binary);
  f.write((const char*)b.data(), (std::streamsize)b.size());
  return bool(f);
}

// GMVEC002 回放（格式见 hw/planner_dual/tb/gmap_tb.hpp）：经驱动 area()+planner_drv_gmap，对 C 模型逐字节
static int replay_gmap(void* h, Dev& dev, const char* path, int* ops_out) {
  const auto b = slurp(path);
  if (b.size() < 16 || std::memcmp(b.data(), "GMVEC002", 8)) return -1;
  const uint32_t init = u32(&b[8]), nops = u32(&b[12]);
  uint8_t* area = planner_drv_gmap_area(h);
  std::vector<uint8_t> ref(kGmapAreaBytes);
  uint32_t s = 20261104u;
  for (int i = 0; i < kGmapAreaBytes; ++i) { s = s * 1664525u + 1013904223u; ref[i] = uint8_t(s >> 24); }
  if (init == 1 || init == 2) {
    uint8_t p[16] = {0x4D, 0x47, 0, 0, 0, 0, 0, 0, 254, 254, 205, 0, 1, uint8_t(init == 2), 0, 0};
    std::memcpy(&ref[GMAP_OFF_PARAM], p, 16);
    gmap_run(ref.data(), GMAP_OP_CLEAR);
  }
  std::memcpy(area, ref.data(), kGmapAreaBytes);
  size_t o = 16;
  int bad = 0;
  for (uint32_t k = 0; k < nops; ++k) {
    const uint32_t op = u32(&b[o]);
    const uint8_t* prm = &b[o + 4];
    const uint32_t hf = u32(&b[o + 20]);
    o += 24;
    const uint8_t* fr = hf ? &b[o] : nullptr;
    if (hf) o += 6400;
    const uint32_t hw = u32(&b[o]);
    o += 4;
    const uint8_t* aux = hw ? &b[o] : nullptr;
    const uint8_t* wp = hw ? &b[o + 6400] : nullptr;
    if (hw) o += 6656;
    for (uint8_t* m : {area, ref.data()}) {
      std::memcpy(m + GMAP_OFF_PARAM, prm, 16);
      if (fr) std::memcpy(m + GMAP_OFF_FRAME, fr, 6400);
      if (aux) { std::memcpy(m + GMAP_OFF_AUX, aux, 6400); std::memcpy(m + GMAP_OFF_WPARAM, wp, 256); }
    }
    uint32_t out4[4] = {0};
    const int rc = planner_drv_gmap(h, op, out4, nullptr);
    const uint32_t mst = gmap_run(ref.data(), op);
    size_t nd = 0;
    for (int i = 0; i < kGmapAreaBytes; ++i) nd += area[i] != ref[i];
    if (rc != 0 || nd || out4[0] != mst || dev.gmap_op != op) {
      if (bad < 3) std::printf("  op %u code %u: rc %d status 0x%x model 0x%x diff_bytes %zu\n", k, op, rc, out4[0], mst, nd);
      ++bad;
    }
  }
  if (o != b.size()) return -1;
  *ops_out = (int)nops;
  return bad;
}

int main(int argc, char** argv) {
  if (argc != 3 && argc != 7) {
    std::fprintf(stderr, "usage: %s input.bin expected.bin [path_params.bin vecA.bin vecB.bin outdir]\n", argv[0]);
    return 2;
  }
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
  std::vector<uint8_t> legacy_bytes;
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
    put_words(legacy_bytes, got.data(), got.size());
    EXPECT(dev.path_ctrl == 0 && dev.gmap_op == 0, "case %zu: PATH_CTRL %u GMAP_OP %u (both 0 without path)", c,
           dev.path_ctrl, dev.gmap_op);
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
  if (argc == 7) {
    const std::string od = argv[6];
    EXPECT(write_file(od + "/drv_legacy_outputs.bin", legacy_bytes), "wrote drv_legacy_outputs.bin (%zu bytes)",
           legacy_bytes.size());
    // 取路径：两例，参数块 path_params.bin（2 x 132）
    const auto pp = slurp(argv[3]);
    EXPECT(pp.size() == 2u * planner_drv::PATH_PARAM_BYTES, "path_params.bin = 2 x 132 bytes");
    std::vector<uint8_t> path_bytes;
    for (size_t c = 0; c < cs.size() && pp.size() == 264u; ++c) {
      const Case& k = cs[c];
      const int n = planner_drv::cells(k.mode == 1), nf = planner_drv::nflags(k.mode == 1);
      std::vector<uint32_t> field(n), flags(nf), pw(1 + planner_drv::PATH_POINTS_MAX, 0);
      uint32_t o3[3];
      const int rc = planner_drv_run_path(h, (int)k.mode, k.map.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy,
                                          k.k, &pp[c * 132], field.data(), flags.data(), pw.data(), o3, nullptr);
      const uint32_t np = pw[0] & 0xFFFFu;
      EXPECT(rc == 0 && dev.path_ctrl == 1 && dev.gmap_op == 0 && (pw[0] >> 28) == 0xAu,
             "case %zu: path run rc %d PATH_CTRL %u header 0x%08x (%u points)", c, rc, dev.path_ctrl, pw[0], np);
      EXPECT(dev.map_seen.size() == (size_t)n + 132 && !std::memcmp(dev.map_seen.data() + n, &pp[c * 132], 132),
             "case %zu: parameter block right after the %d map bytes", c, n);
      std::vector<uint32_t> got = {o3[0], o3[1], o3[2]};
      got.insert(got.end(), field.begin(), field.end());
      got.insert(got.end(), flags.begin(), flags.end());
      EXPECT(got == k.exp, "case %zu: with path the scalars + field + flags still == expected.bin", c);
      got.insert(got.end(), pw.begin(), pw.begin() + 1 + np);
      put_words(path_bytes, got.data(), got.size());
    }
    EXPECT(write_file(od + "/drv_path_outputs.bin", path_bytes), "wrote drv_path_outputs.bin (%zu bytes)",
           path_bytes.size());
    // 地图操作：两组向量
    for (int v = 4; v <= 5; ++v) {
      int nops = 0;
      const uint32_t calls0 = dev.gmap_calls;
      const int bad = replay_gmap(h, dev, argv[v], &nops);
      EXPECT(bad == 0 && nops > 0 && dev.gmap_ptr_bad == 0 && (int)(dev.gmap_calls - calls0) == nops,
             "%s: %d gmap ops via the driver, area == C model after every op (bad %d), MAP == FIELD == area",
             argv[v], nops, bad);
      EXPECT(mb->reg(XPLANNER_FIELD_DUAL_CONTROL_ADDR_GMAP_OP_DATA) == 0, "GMAP_OP back to 0 after the ops");
    }
    // 阳性对照：驱动侧帧改 1 字节，必须报差
    {
      uint8_t* area = planner_drv_gmap_area(h);
      std::vector<uint8_t> ref(area, area + kGmapAreaBytes);
      const uint8_t p[16] = {0x4D, 0x47, 200, 0, 200, 0, 0, 0, 254, 254, 205, 0, 1, 0, 0, 0};
      for (int i = 0; i < 6400; ++i) ref[GMAP_OFF_FRAME + i] = uint8_t(i % 3 == 0 ? 0x64 : i % 3 == 1 ? 0 : 0xFF);
      std::memcpy(&ref[GMAP_OFF_PARAM], p, 16);
      std::memcpy(area, ref.data(), kGmapAreaBytes);
      area[GMAP_OFF_FRAME + 4321] ^= 0x64;
      planner_drv_gmap(h, 1, nullptr, nullptr);
      gmap_run(ref.data(), 1);
      size_t nd = 0;
      for (int i = 0; i < kGmapAreaBytes; ++i) nd += area[i] != ref[i] && (i < (int)GMAP_OFF_FRAME || i >= (int)GMAP_OFF_PARAM);
      EXPECT(nd > 0, "control: one frame byte changed on the driver side -> %zu map bytes differ", nd);
    }
    // 驱动拒绝 op 0 / 8
    uint32_t o4[4];
    EXPECT(planner_drv_gmap(h, 0, o4, nullptr) == -1 && planner_drv_gmap(h, 8, o4, nullptr) == -1,
           "driver refuses gmap op 0 and 8");
  }
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
