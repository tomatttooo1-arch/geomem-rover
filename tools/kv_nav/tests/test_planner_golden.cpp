// CTRLPS1 key test 1: the split planner (planner_split.hpp) and the PS extraction from an external
// smoothing port is byte-identical to the frozen wave_plan_v3(variant 2). With positive controls.
//   test_planner_golden VECTOR_DIR [CASES_DIR]
//             field.u24le path.f64le). CASES_DIR (optional): car1015 cases from sim/prep_car1015.py.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../planner_split.hpp"
#include "../ref/planner_hw_ref.hpp"
#include "../smooth_v3.hpp"
#include "model_v3.hpp"

namespace R = planner_hw_ref;
namespace S = kv_nav_planner;

static bool read_file(const std::string& p, std::vector<unsigned char>& out) {
  std::ifstream f(p, std::ios::binary);
  if (!f) return false;
  out.assign(std::istreambuf_iterator<char>(f), {});
  return true;
}
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { ++fails; std::printf("FAIL %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Plan { int status, count; unsigned rounds; bool fv; double rx, ry; std::vector<double> x, y; std::vector<std::uint32_t> p; std::vector<std::uint8_t> m; };

template <class F> static Plan run(F plan, const std::vector<unsigned char>& map, double sx, double sy, double gx, double gy, unsigned lim) {
  Plan o; o.m.assign(6400, 0); o.p.assign(6400, 0); o.x.assign(6402, 0); o.y.assign(6402, 0);
  auto r = plan(map.data(), int(sx), int(sy), gx, gy, o.m.data(), o.p.data(), o.x.data(), o.y.data(), lim);
  o.status = r.status; o.count = r.count; o.rounds = r.rounds; o.fv = r.field_valid; o.rx = r.resolved_x; o.ry = r.resolved_y;
  o.x.resize(std::max(0, r.count)); o.y.resize(std::max(0, r.count));
  return o;
}
static bool same_bits(double a, double b) { return std::memcmp(&a, &b, 8) == 0; }
static bool path_eq(const Plan& a, const Plan& b) {
  return a.status == b.status && a.count == b.count && std::memcmp(a.x.data(), b.x.data(), 8 * a.x.size()) == 0 &&
         std::memcmp(a.y.data(), b.y.data(), 8 * a.y.size()) == 0 && same_bits(a.rx, b.rx) && same_bits(a.ry, b.ry);
}

int main(int argc, char** argv) {
  if (argc < 2) { std::printf("usage: %s VECTOR_DIR [CASES_DIR]\n", argv[0]); return 2; }
  const std::string dir = argv[1];
  auto ref_plan = [](const unsigned char* m, int sx, int sy, double gx, double gy, std::uint8_t* s, std::uint32_t* p, double* x, double* y, unsigned lim) {
    return R::plan(m, 80, 80, R::Request{sx, sy, gx, gy}, s, p, x, y, 6402, lim, nullptr); };
  auto split_plan = [](const unsigned char* m, int sx, int sy, double gx, double gy, std::uint8_t* s, std::uint32_t* p, double* x, double* y, unsigned lim) {
    auto r = S::plan(m, 80, 80, S::Request{sx, sy, gx, gy}, s, p, x, y, 6402, lim, nullptr);
    R::Result o; o.status = r.status; o.count = r.count; o.rounds = r.rounds; o.field_valid = r.field_valid; o.resolved_x = r.resolved_x; o.resolved_y = r.resolved_y; return o; };
  int n_vec = 0, n_golden_path = 0, n_extract = 0, n_ctrl = 0, n_ctrl_changed = 0, n_m1 = 0, n_m1_changed = 0;
  int n_smooth = 0, n_smooth_changed = 0, n_ref_eq = 0;
  for (int i = 0; i < 200; ++i) {
    char name[8]; std::snprintf(name, sizeof name, "%03d", i);
    const std::string d = dir + "/" + name + "/";
    std::vector<unsigned char> map, fld, pth, req, exp;
    if (!read_file(d + "map.u8", map) || map.size() != 6400) { CHECK(false, "missing %s", name); continue; }
    read_file(d + "field.u24le", fld); read_file(d + "path.f64le", pth);
    std::ifstream fr(d + "request.txt"), fe(d + "expected.txt");
    std::string a, b, c, e, l; fr >> a >> b >> c >> e >> l;
    const double sx = std::stod(a), sy = std::stod(b), gx = std::stod(c), gy = std::stod(e);
    const unsigned lim = unsigned(std::stoul(l));
    int es, ep, efv; unsigned er; fe >> es >> ep >> efv >> er;
    ++n_vec;
    // (1) split plan() == reference plan(), every output byte, incl. field and working map
    Plan pr = run(ref_plan, map, sx, sy, gx, gy, lim), ps = run(split_plan, map, sx, sy, gx, gy, lim);
    const bool eq = path_eq(pr, ps) && pr.rounds == ps.rounds && pr.fv == ps.fv && pr.p == ps.p && pr.m == ps.m;
    CHECK(eq, "%s split != ref", name);
    n_ref_eq += eq;
    // (2) split plan() == golden files
    CHECK(ps.status == es && ps.count == ep && int(ps.fv) == efv && ps.rounds == er, "%s expected %d %d %d %u got %d %d %d %u", name, es, ep, efv, er, ps.status, ps.count, int(ps.fv), ps.rounds);
    std::vector<double> gp(pth.size() / 8);
    if (!pth.empty()) std::memcpy(gp.data(), pth.data(), pth.size());
    if (ps.count > 0) {
      bool same = int(gp.size()) == 2 * ps.count;
      for (int k = 0; same && k < ps.count; ++k) same = same_bits(gp[2 * k], ps.x[k]) && same_bits(gp[2 * k + 1], ps.y[k]);
      CHECK(same, "%s path != golden bytes", name);
      n_golden_path += same;
    } else {
      CHECK(gp.empty(), "%s golden has a path, split has none", name);
    }
    // (3) PL-boundary: golden field -> extract_from_field() == golden path/status
    if (fld.size() == 6400 * 3) {
      std::vector<std::uint32_t> f(6400);
      for (int k = 0; k < 6400; ++k) f[k] = fld[3 * k] | (fld[3 * k + 1] << 8) | (fld[3 * k + 2] << 16);
      CHECK(f == ps.p, "%s golden field != split field", name);
      auto extract = [&](const std::vector<std::uint32_t>& field) {
        Plan o; o.m.assign(6400, 0); o.x.assign(6402, 0); o.y.assign(6402, 0);
        auto r = S::extract_from_field(map.data(), S::Request{int(sx), int(sy), gx, gy}, field.data(), er, o.m.data(), o.x.data(), o.y.data(), 6402);
        o.status = r.status; o.count = r.count; o.rounds = r.rounds; o.fv = r.field_valid; o.rx = r.resolved_x; o.ry = r.resolved_y;
        o.x.resize(std::max(0, r.count)); o.y.resize(std::max(0, r.count)); return o; };
      Plan pe = extract(f);
      const bool ok = path_eq(pe, ps) && pe.rounds == ps.rounds && pe.fv == ps.fv;
      CHECK(ok, "%s extract_from_field != plan", name);
      n_extract += ok;
      // positive controls on successful multi-point paths: the extraction must SEE a field change
      if (ps.count >= 3 && (ps.status == S::EXACT || ps.status == S::WITHIN_TOLERANCE)) {
        const int mid = ps.count / 2;
        const int cx = int(ps.x[mid]), cy = int(ps.y[mid]);
        if (ps.x[mid] == cx && ps.y[mid] == cy && !(cx == int(sx) && cy == int(sy))) {
          std::vector<std::uint32_t> g = f; g[cy * 80 + cx] = S::kInfinity;  // knock a chain cell out
          Plan pc = extract(g);
          ++n_ctrl; const bool changed = !path_eq(pc, pe); n_ctrl_changed += changed;
          CHECK(changed, "%s positive control (chain cell -> INF) not detected", name);
          std::vector<std::uint32_t> h = f; if (h[cy * 80 + cx] > 0) h[cy * 80 + cx] -= 1;  // design's "-1" probe
          Plan pm = extract(h); ++n_m1; n_m1_changed += !path_eq(pm, pe);
        }
      }
    }
    // (4) smoothing port == frozen wave_plan_v3(variant 2), on the unbounded (converged) field
    if (sx == std::floor(sx) && std::isfinite(gx) && std::isfinite(gy)) {
      Plan pu = run(split_plan, map, sx, sy, gx, gy, 0);
      std::vector<double> vx(6402), vy(6402); unsigned rr = 0; int st = -1; double res[2];
      const int vn = wave_plan_v3(map.data(), 80, 80, int(sx), int(sy), gx, gy, 0.05, 0.15, 0, vx.data(), vy.data(), 6402, &rr, &st, res, 2);
      if (pu.status == S::EXACT || pu.status == S::WITHIN_TOLERANCE) {
        std::vector<double> x = pu.x, y = pu.y;
        const int n = kv_nav_smooth::constrained_smooth(pu.m.data(), 80, 80, x.data(), y.data(), pu.count);
        bool same = st == pu.status && vn == n;
        for (int k = 0; same && k < n; ++k) same = same_bits(vx[k], x[k]) && same_bits(vy[k], y[k]);
        CHECK(same, "%s smooth != wave_plan_v3 variant 2 (st %d/%d n %d/%d)", name, st, pu.status, vn, n);
        ++n_smooth; n_smooth_changed += (n != pu.count);
      } else {
        CHECK(vn <= 0 && st == pu.status, "%s v3 status %d split status %d", name, st, pu.status);
      }
    }
  }
  std::printf("vectors %d | split==ref (all bytes incl. field) %d | golden path bytes %d | extract_from_field(golden field)==plan %d\n",
              n_vec, n_ref_eq, n_golden_path, n_extract);
  std::printf("positive control chain-cell->INF changed %d/%d | '-1' probe changed %d/%d (informative only)\n",
              n_ctrl_changed, n_ctrl, n_m1_changed, n_m1);
  std::printf("smooth==wave_plan_v3(v2) %d (of which smoothing changed the path: %d)\n", n_smooth, n_smooth_changed);
  CHECK(n_ctrl > 0 && n_smooth_changed > 0, "vacuous controls");
  // optional: car1015 real maps (prep_car1015.py cases): split==ref and smooth==v3 on real requests
  if (argc > 2) {
    const std::string cd = argv[2];
    int nc = 0, eqc = 0, smc = 0, n_rec = 0;
    for (int i = 0; i < 1000; ++i) {
      char name[8]; std::snprintf(name, sizeof name, "%03d", i);
      std::vector<unsigned char> map; if (!read_file(cd + "/" + name + "/map.u8", map)) continue;
      std::ifstream fr(cd + "/" + name + "/request.txt"); double sx, sy, gx, gy; fr >> sx >> sy >> gx >> gy;
      Plan pr = run(ref_plan, map, sx, sy, gx, gy, 0), ps = run(split_plan, map, sx, sy, gx, gy, 0);
      const bool eq = path_eq(pr, ps) && pr.p == ps.p; ++nc; eqc += eq; CHECK(eq, "car1015 %s split != ref", name);
      std::vector<unsigned char> p1; read_file(cd + "/" + name + "/path1.f64le", p1);
      std::vector<double> q(p1.size() / 8); if (!p1.empty()) std::memcpy(q.data(), p1.data(), p1.size());
      const int off = (q.size() >= 2 && (q[0] != sx || q[1] != sy)) ? 1 : 0;
      bool k1 = int(q.size() / 2) - off == ps.count;
      for (int k = 0; k1 && k < ps.count; ++k) k1 = same_bits(q[2 * (k + off)], ps.x[k]) && same_bits(q[2 * (k + off) + 1], ps.y[k]);
      n_rec += k1; CHECK(k1, "car1015 %s tier-1 != dev2 recorded v3_1", name);
      if (ps.status <= 1) {
        std::vector<double> vx(6402), vy(6402); unsigned rr = 0; int st = -1; double res[2];
        const int vn = wave_plan_v3(map.data(), 80, 80, int(sx), int(sy), gx, gy, 0.05, 0.15, 0, vx.data(), vy.data(), 6402, &rr, &st, res, 2);
        std::vector<double> x = ps.x, y = ps.y;
        const int n = kv_nav_smooth::constrained_smooth(ps.m.data(), 80, 80, x.data(), y.data(), ps.count);
        bool same = vn == n; for (int k = 0; same && k < n; ++k) same = same_bits(vx[k], x[k]) && same_bits(vy[k], y[k]);
        smc += same; CHECK(same, "car1015 %s smooth != v3", name);
      }
    }
    std::printf("car1015 real maps: split==ref %d/%d, == dev2 recorded v3_1 %d, smooth==v3 %d\n", eqc, nc, n_rec, smc);
  }
  std::printf(fails ? "RESULT FAIL (%d)\n" : "RESULT PASS\n", fails);
  return fails ? 1 : 0;
}
