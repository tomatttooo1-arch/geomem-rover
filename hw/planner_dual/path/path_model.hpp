#pragma once
// Mirrors hw/planner_dual/src/kernel_hls.cpp extract_path() + the DDR result layout
// (runs/nationals-prelim-20261102/PLNPATH0/DESIGN.md 4.4). Inputs are what the core holds after
// a converged scan: the ORIGINAL map (walk/entry are derived exactly as the core's initialize
// loop does), the complete field, the request cells and the 132-byte parameter block.
#include <cstdint>
#include <vector>
namespace plnpath {
constexpr int kParamBytes = 132, kCandMax = 64, kPathMax = 4096;
enum PathStatus { P_EXACT = 0, P_TOL = 1, P_NO_PATH = 4, P_SAFETY = 5, P_OVERFLOW = 9, P_NOT_RUN = 15 };

struct ModelOut {
  std::uint32_t header = 0;
  std::vector<std::uint32_t> words;  // header followed by n path words (start -> target)
  long accepted_not_equal = 0;       // assertion counter: accepted candidate != p_cur
};

inline std::uint32_t make_header(unsigned st, unsigned chosen, unsigned n) {
  return (0xAu << 28) | ((st & 15u) << 24) | ((chosen & 255u) << 16) | (n & 0xffffu);
}

// tie_variant: 0 = product rule (smallest enumeration index wins a distance tie);
//              1 = positive-control variant (largest index wins).
inline ModelOut model(const std::uint8_t* map, const std::uint32_t* field, int W, std::uint32_t inf,
                      int sx, int sy, int gx, int gy, const std::uint8_t* params, bool field_ready,
                      int tie_variant = 0) {
  ModelOut o;
  auto finish = [&](unsigned st, unsigned ch, const std::vector<std::uint32_t>& desc) {
    const bool ok = st == P_EXACT || st == P_TOL;
    const unsigned n = ok ? unsigned(desc.size()) : 0u;
    o.header = make_header(st, ok ? ch : 255u, n);
    o.words.assign(1, o.header);
    for (unsigned i = 0; i < n; ++i) o.words.push_back(desc[n - 1 - i]);
    return o;
  };
  std::vector<std::uint32_t> desc;
  if (!field_ready) return finish(P_NOT_RUN, 255, desc);
  const int src = sy * W + sx;
  auto walk = [&](int x, int y) {
    if (x == 0 || y == 0 || x == W - 1 || y == W - 1) return false;
    return y * W + x == src || map[y * W + x] < 253;
  };
  auto entry = [&](int x, int y) -> unsigned { return y * W + x == src ? 50u : 50u + (4u * map[y * W + x]) / 5u; };
  const unsigned nc = params[0] > 64 ? 64u : params[0];
  const bool exact_safe = params[1] & 1;
  int x = gx, y = gy;
  unsigned chosen = 255, st;
  if (field[gy * W + gx] != inf) {
    if (!exact_safe) return finish(P_SAFETY, 255, desc);
    st = P_EXACT;
  } else {
    bool have = false;
    for (unsigned j = 0; j < nc && !have; ++j) {
      const int a = params[4 + 2 * j], b = params[5 + 2 * j];
      if (a < W && b < W && field[b * W + a] != inf) { have = true; chosen = j; x = a; y = b; }
    }
    if (!have) return finish(P_NO_PATH, 255, desc);
    st = P_TOL;
  }
  static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  for (;;) {
    desc.push_back(std::uint32_t((y << 8) | x));
    if (x == sx && y == sy) break;
    if (int(desc.size()) == kPathMax) return finish(P_OVERFLOW, 255, desc);
    const std::uint32_t p_cur = field[y * W + x];
    const unsigned e = entry(x, y);
    bool found = false;
    unsigned best = 0;
    int wx = 0, wy = 0;
    for (int k = 0; k < 8; ++k) {
      const int xx = x + dx[k], yy = y + dy[k];
      if (!walk(xx, yy)) continue;
      if (dx[k] && dy[k] && !(walk(xx, y) && walk(x, yy))) continue;
      const std::uint32_t p = field[yy * W + xx];
      if (p == inf) continue;
      const std::uint64_t cand = std::uint64_t(p) + e * (dx[k] && dy[k] ? 14u : 10u);
      if (cand > p_cur) continue;
      if (cand != p_cur) ++o.accepted_not_equal;
      const unsigned d2 = unsigned((xx - sx) * (xx - sx) + (yy - sy) * (yy - sy));
      const unsigned key = d2 * 8u + unsigned(tie_variant ? 7 - k : k);
      if (!found || key < best) { found = true; best = key; wx = xx; wy = yy; }
    }
    if (!found) return finish(P_NO_PATH, 255, desc);
    x = wx; y = wy;
  }
  return finish(st, chosen, desc);
}
}  // namespace plnpath
