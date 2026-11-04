#pragma once
// PLNCHAIN0 测试共用：mock 总线上的 GMAP3 v9 核（hw/planner_dual/src/kernel_hls.cpp，sha 17079dce…，15 个参数）。
// 与 drv_gmap3/test_planner_drv.cpp 的 Dev 相同，差别两处：
//  ① 指针按 GMAP3 硬件的 32 位口解码：只取 0x10/0x1c/0x28 的低 32 位（0x14/0x20/0x2c 在 v9 导出里是 reserved），
//     高 32 位写了什么另行记录（hi_seen）；
//  ② 核读写超出驱动缓冲时记 oob（真实硬件上就是 DMA 写到缓冲外）。
// 另可在核跑完后改一个路径点（corrupt_path，阳性对照用）。
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "kernel_hls.hpp"
#include "../../m7/pl/sw/pl_io.hpp"

namespace g3mock {
constexpr std::uint32_t A_MAP = 0x10, A_FIELD = 0x1c, A_FLAGS = 0x28, A_SX = 0x34, A_SY = 0x3c, A_GX = 0x44,
                        A_GY = 0x4c, A_K = 0x54, A_STATUS = 0x5c, A_ROUNDS = 0x6c, A_VALID = 0x7c, A_MODE = 0x88,
                        A_PATH = 0x90, A_GMAP = 0x98;
inline int width(int mode) { return mode == 1 ? 200 : 80; }
struct Dev {
  std::vector<std::uint8_t> map_seen;
  std::uint32_t path_ctrl = 0, gmap_op = 0, gmap_calls = 0, plan_calls = 0, oob = 0;
  std::uint32_t hi_seen = 0;   // 三个指针高 32 位按位或
  bool corrupt_path = false;
  bool m5_core = false;        // 模拟没有 0x90/0x98 的 m5 核（写了也没有）
  void operator()(plio::MockBus& m) {
    path_ctrl = m5_core ? 0u : (m.reg(A_PATH) & 1u);   // 硬件只有 bit0
    gmap_op = m5_core ? 0u : (m.reg(A_GMAP) & 7u);     // 硬件只有 [2:0]
    hi_seen |= m.reg(A_MAP + 4) | m.reg(A_FIELD + 4) | m.reg(A_FLAGS + 4);
    const std::uint64_t pm = m.reg(A_MAP), pf = m.reg(A_FIELD), pg = m.reg(A_FLAGS);   // 32 位口
    static ap_uint<8> map[kGmapAreaBytes];
    static ap_uint<32> field[kGmapAreaWords], flags[kPlnFlagsDepth];
    ap_uint<32> st = 0, rounds = 0, valid = 0;
    if (gmap_op != 0) {
      ++gmap_calls;
      std::uint8_t* ap = (std::uint8_t*)m.va_of(pm, kGmapAreaBytes);
      if (!ap || pf != pm) { ++oob; return; }
      for (int i = 0; i < kGmapAreaBytes; ++i) map[i] = ap[i];
      for (int i = 0; i < kGmapAreaWords; ++i) { std::uint32_t v; std::memcpy(&v, ap + 4 * i, 4); field[i] = v; }
      planner_field_dual(map, field, flags, 0, 0, 0, 0, 0, st, rounds, valid, 0, 0, gmap_op);
      for (int i = 0; i < kGmapAreaWords; ++i) { std::uint32_t v = field[i].to_uint(); std::memcpy(ap + 4 * i, &v, 4); }
    } else {
      ++plan_calls;
      const std::uint32_t mode = m.reg(A_MODE);
      const int w = width(mode == 1), n = w * w, nf = mode == 1 ? 256 : 64;
      const int nm = n + (path_ctrl ? 132 : 0), nfw = path_ctrl ? nf + 1 + 4096 : nf;
      const std::uint8_t* mp = (const std::uint8_t*)m.va_of(pm, nm);
      std::uint8_t* fp = (std::uint8_t*)m.va_of(pf, (size_t)n * 4);
      std::uint8_t* gp = (std::uint8_t*)m.va_of(pg, (size_t)nfw * 4);
      if (!mp || !fp || !gp) { ++oob; return; }
      map_seen.assign(mp, mp + nm);
      for (int i = 0; i < nm; ++i) map[i] = mp[i];
      for (int i = 0; i < nfw; ++i) { std::uint32_t v; std::memcpy(&v, gp + 4 * i, 4); flags[i] = v; }
      planner_field_dual(map, field, flags, (int)m.reg(A_SX), (int)m.reg(A_SY), (int)m.reg(A_GX), (int)m.reg(A_GY),
                         m.reg(A_K), st, rounds, valid, mode, path_ctrl, 0);
      for (int i = 0; i < n; ++i) { std::uint32_t v = field[i].to_uint(); std::memcpy(fp + 4 * i, &v, 4); }
      for (int i = 0; i < nfw; ++i) { std::uint32_t v = flags[i].to_uint(); std::memcpy(gp + 4 * i, &v, 4); }
      if (corrupt_path && path_ctrl) {
        std::uint32_t hdr; std::memcpy(&hdr, gp + 4 * nf, 4);
        const std::uint32_t np = hdr & 0xFFFFu;
        if (np >= 3) { std::uint32_t v; std::memcpy(&v, gp + 4 * (nf + 1 + np / 2), 4); v ^= 1u; std::memcpy(gp + 4 * (nf + 1 + np / 2), &v, 4); }
      }
    }
    m.set_reg(A_STATUS, st.to_uint());
    m.set_reg(A_ROUNDS, rounds.to_uint());
    m.set_reg(A_VALID, valid.to_uint());
  }
};

inline std::vector<std::uint8_t> slurp(const char* p) {
  std::vector<std::uint8_t> b;
  if (FILE* f = std::fopen(p, "rb")) {
    std::uint8_t t[65536]; size_t k;
    while ((k = std::fread(t, 1, sizeof t, f)) > 0) b.insert(b.end(), t, t + k);
    std::fclose(f);
  }
  return b;
}
inline std::uint32_t u32(const std::uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (std::uint32_t)p[3] << 24; }
struct Case { std::uint32_t mode, sx, sy, gx, gy, k; std::vector<std::uint8_t> map; std::vector<std::uint32_t> exp; };
inline bool load_frozen(const char* in_p, const char* ex_p, std::vector<Case>& cs) {
  const auto in = slurp(in_p), ex = slurp(ex_p);
  if (in.size() != 46460 || ex.size() != 186916 || std::memcmp(in.data(), "FPGADIN01", 8) ||
      std::memcmp(ex.data(), "FPGADGO01", 8)) return false;
  size_t pi = 12, pe = 12;
  for (int c = 0; c < 2; ++c) {
    Case k;
    k.mode = u32(&in[pi]); k.sx = u32(&in[pi + 4]); k.sy = u32(&in[pi + 8]); k.gx = u32(&in[pi + 12]);
    k.gy = u32(&in[pi + 16]); k.k = u32(&in[pi + 20]);
    pi += 24;
    const int n = width(k.mode == 1) * width(k.mode == 1), nf = k.mode == 1 ? 256 : 64;
    k.map.assign(in.begin() + pi, in.begin() + pi + n);
    pi += n;
    for (int i = 0; i < 3 + n + nf; ++i, pe += 4) k.exp.push_back(u32(&ex[pe]));
    cs.push_back(k);
  }
  return pi == in.size() && pe == ex.size();
}
}  // namespace g3mock
static int g_fails = 0;
#define EXPECT(c, ...) do { if (!(c)) { std::printf("FAIL "); g_fails++; } else std::printf("ok   "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
