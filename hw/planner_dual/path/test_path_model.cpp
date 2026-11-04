// PLNPATH0 key comparison 1 (preregistered in PLNPATH0/DESIGN.md section 7):
// integer C model (path_model.hpp) + PS half (path_ps.hpp) == PS product extraction, point for
// 127 car1015 real requests. Pass = 0 mismatches, 0 assertion violations, both positive controls hit.
//   test_path_model FROZEN_DIR VECTOR_DIR CASES_DIR [TB_PARAMS_OUT]
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include "path_ps.hpp"

namespace K = kv_nav_planner;
using namespace plnpath;
typedef PS<80, 80, 0xffffffu> PS80;
typedef PS<200, 200, 0x1fffffffu> PS200;

static bool read_file(const std::string& p, std::vector<unsigned char>& o) {
  std::ifstream f(p, std::ios::binary); if (!f) return false;
  o.assign(std::istreambuf_iterator<char>(f), {}); return true;
}
static unsigned u32(const std::vector<unsigned char>& b, size_t& at) {
  unsigned v = b[at] | (b[at + 1] << 8) | (b[at + 2] << 16) | (unsigned(b[at + 3]) << 24); at += 4; return v;
}
static bool same_bits(double a, double b) { return std::memcmp(&a, &b, 8) == 0; }
struct Out { int status, count; bool fv; double rx, ry; std::vector<double> x, y; };
static bool eq(const Out& a, const Out& b) {
  if (a.status != b.status || a.count != b.count || a.fv != b.fv || !same_bits(a.rx, b.rx) || !same_bits(a.ry, b.ry)) return false;
  for (int i = 0; i < a.count; ++i) if (!same_bits(a.x[i], b.x[i]) || !same_bits(a.y[i], b.y[i])) return false;
  return true;
}
static Out pack(const K::Result& r, const double* x, const double* y) {
  Out o{r.status, r.count, r.field_valid, r.resolved_x, r.resolved_y, {}, {}};
  if (r.count > 0) { o.x.assign(x, x + r.count); o.y.assign(y, y + r.count); }
  return o;
}

struct Stats { int req = 0, eq = 0, mism = 0, success = 0, tol = 0, ctrl = 0, ctrl_hit = 0, tie_diff = 0, sanity_bad = 0;
               long assert_bad = 0; long points = 0; int max_n = 0; };
static Stats S;
static FILE* LOG;

// One request. W=80 uses the product extract_from_field() as reference; W=200 uses ref_extract(res .10).
template <int W, std::uint32_t Inf>
static void one(const char* tag, const std::vector<unsigned char>& map, const std::vector<std::uint32_t>& field,
                K::Request q, std::vector<unsigned char>* params_out = nullptr) {
  typedef PS<W, W, Inf> P;
  typedef typename P::E E;
  const double res = W == 80 ? .05 : .10;
  std::vector<std::uint8_t> m(W * W);
  std::vector<double> rx(W * W + 2), ry(W * W + 2), mx(W * W + 2), my(W * W + 2);
  K::Result chk;
  if (!E::check(map.data(), q, m.data(), field.data(), rx.data(), ry.data(), W * W + 2, chk)) return;  // core status 2/3/6
  E::prepare(map.data(), q.sx, q.sy, m.data());
  K::Result rr;
  if (W == 80) {
    std::vector<std::uint8_t> scratch(6400);
    rr = K::extract_from_field(map.data(), q, field.data(), 0, scratch.data(), rx.data(), ry.data(), 6402);
    rr.rounds = 0;
    // sanity: the res-generalised reference is the product reference at res .05
    std::vector<double> sx(6402), sy(6402);
    K::Result r2 = P::ref_extract(m.data(), field.data(), q, .05, sx.data(), sy.data(), 6402);
    if (!eq(pack(r2, sx.data(), sy.data()), pack(rr, rx.data(), ry.data()))) ++S.sanity_bad;
  } else {
    rr = P::ref_extract(m.data(), field.data(), q, res, rx.data(), ry.data(), W * W + 2);
  }
  const Out ref = pack(rr, rx.data(), ry.data());
  const PsParams pp = P::make_params(m.data(), q, res);
  if (params_out) params_out->assign(pp.bytes, pp.bytes + kParamBytes);
  const int gx = int(std::round(q.gx)), gy = int(std::round(q.gy));
  auto run = [&](const std::vector<std::uint32_t>& f, int tie) {
    ModelOut mo = model(map.data(), f.data(), W, Inf, q.sx, q.sy, gx, gy, pp.bytes, true, tie);
    if (tie == 0 && &f == &field) S.assert_bad += mo.accepted_not_equal;
    K::Result r = P::assemble(mo.words.data(), pp, q, mx.data(), my.data());
    if (tie == 0 && &f == &field) S.max_n = std::max<int>(S.max_n, mo.words[0] & 0xffff);
    return pack(r, mx.data(), my.data());
  };
  const Out got = run(field, 0);
  ++S.req;
  const bool ok = eq(got, ref);
  if (ok) ++S.eq; else ++S.mism;
  const bool success = ref.status == K::EXACT || ref.status == K::WITHIN_TOLERANCE;
  if (success) { ++S.success; S.points += ref.count; }
  if (ref.status == K::WITHIN_TOLERANCE) ++S.tol;
  std::fprintf(LOG, "%s W=%d status ref %d model %d count %d/%d %s\n", tag, W, ref.status, got.status, ref.count, got.count, ok ? "EQ" : "MISMATCH");
  if (!ok) std::printf("MISMATCH %s W=%d ref st %d n %d | model st %d n %d\n", tag, W, ref.status, ref.count, got.status, got.count);
  if (run(field, 1).count >= 0 && !eq(run(field, 1), ref)) ++S.tie_diff;
  // positive control (a): mid chain cell -> INF in the model's input only, output must change
  if (success && ref.count >= 3) {
    const int mid = ref.count / 2;
    const int cx = int(ref.x[mid]), cy = int(ref.y[mid]);
    if (ref.x[mid] == cx && ref.y[mid] == cy && !(cx == q.sx && cy == q.sy)) {
      std::vector<std::uint32_t> g = field; g[cy * W + cx] = Inf;
      ++S.ctrl; if (!eq(run(g, 0), ref)) ++S.ctrl_hit; else std::printf("CONTROL MISSED %s\n", tag);
    }
  }
}

int main(int argc, char** argv) {
  if (argc < 4) { std::printf("usage: %s FROZEN_DIR VECTOR_DIR CASES_DIR [TB_PARAMS_OUT]\n", argv[0]); return 2; }
  LOG = std::fopen("test_path_model.log", "w");
  std::vector<unsigned char> in, ex, tbp;
  if (!read_file(std::string(argv[1]) + "/input.bin", in) || !read_file(std::string(argv[1]) + "/expected.bin", ex)) { std::printf("no frozen pair\n"); return 2; }
  size_t ai = 12, ae = 12;
  for (int c = 0; c < 2; ++c) {
    const unsigned mode = u32(in, ai); const int sx = int(u32(in, ai)), sy = int(u32(in, ai)), gx = int(u32(in, ai)), gy = int(u32(in, ai)); u32(in, ai);
    const int n = mode == 0 ? 6400 : 40000, nf = mode == 0 ? 64 : 256;
    std::vector<unsigned char> map(in.begin() + ai, in.begin() + ai + n); ai += n;
    const unsigned st = u32(ex, ae); u32(ex, ae); const unsigned valid = u32(ex, ae);
    std::vector<std::uint32_t> f(n); for (int i = 0; i < n; ++i) f[i] = u32(ex, ae); ae += 4 * nf;
    std::printf("frozen case %d mode %u status %u valid %u start %d,%d goal %d,%d\n", c, mode, st, valid, sx, sy, gx, gy);
    std::vector<unsigned char> pb;
    char tag[32]; std::snprintf(tag, sizeof tag, "frozen%d", c);
    if (mode == 0) one<80, 0xffffffu>(tag, map, f, K::Request{sx, sy, double(gx), double(gy)}, &pb);
    else one<200, 0x1fffffffu>(tag, map, f, K::Request{sx, sy, double(gx), double(gy)}, &pb);
    tbp.insert(tbp.end(), pb.begin(), pb.end());
  }
  const Stats key = S;
  int nvec = 0;
  for (int i = 0; i < 200; ++i) {
    char name[8]; std::snprintf(name, sizeof name, "%03d", i);
    const std::string d = std::string(argv[2]) + "/" + name + "/";
    std::vector<unsigned char> map, fld;
    if (!read_file(d + "map.u8", map) || !read_file(d + "field.u24le", fld) || fld.size() != 19200) continue;
    std::ifstream fr(d + "request.txt"); double sx, sy, gx, gy; fr >> sx >> sy >> gx >> gy;
    if (sx != std::floor(sx) || sy != std::floor(sy)) continue;
    std::vector<std::uint32_t> f(6400);
    for (int k = 0; k < 6400; ++k) f[k] = fld[3 * k] | (fld[3 * k + 1] << 8) | (fld[3 * k + 2] << 16);
    ++nvec; char tag[32]; std::snprintf(tag, sizeof tag, "vec%s", name);
    one<80, 0xffffffu>(tag, map, f, K::Request{int(sx), int(sy), gx, gy});
  }
  int ncase = 0;
  for (int i = 0; i < 1000; ++i) {
    char name[8]; std::snprintf(name, sizeof name, "%03d", i);
    const std::string d = std::string(argv[3]) + "/" + name + "/";
    std::vector<unsigned char> map; if (!read_file(d + "map.u8", map) || map.size() != 6400) continue;
    std::ifstream fr(d + "request.txt"); double sx, sy, gx, gy; fr >> sx >> sy >> gx >> gy;
    std::vector<std::uint8_t> m(6400); std::vector<std::uint32_t> f(6400); unsigned rounds = 0;
    if (!PS80::E::scan(map.data(), int(sx), int(sy), m.data(), f.data(), 0, rounds, nullptr)) continue;
    ++ncase; char tag[32]; std::snprintf(tag, sizeof tag, "car%s", name);
    one<80, 0xffffffu>(tag, map, f, K::Request{int(sx), int(sy), gx, gy});
  }
  // (4) hand-made tie request (DESIGN 7-(b): added because no distance tie occurred above):
  // open map, one lethal cell (40,42) between start (40,40) and goal (40,44); the goal's two accepted
  // neighbours (39,43) k=0 and (41,43) k=2 have equal potential and equal squared distance 10.
  const int tie_before = S.tie_diff;
  {
    std::vector<unsigned char> map(6400, 0); map[42 * 80 + 40] = 254;
    std::vector<std::uint8_t> m(6400); std::vector<std::uint32_t> f(6400); unsigned rounds = 0;
    PS80::E::scan(map.data(), 40, 40, m.data(), f.data(), 0, rounds, nullptr);
    one<80, 0xffffffu>("tie_handmade", map, f, K::Request{40, 40, 40.0, 44.0});
  }
  std::printf("hand-made tie request: reversed tie order differs %s\n", S.tie_diff > tie_before ? "yes" : "NO");
  std::printf("inputs: frozen 2 (requests %d) | dev2 vectors with field %d | car1015 cases %d\n", key.req, nvec, ncase);
  std::printf("requests compared %d | equal %d | MISMATCH %d | success %d (tolerance %d) | points %ld | longest core path %d\n",
              S.req, S.eq, S.mism, S.success, S.tol, S.points, S.max_n);
  std::printf("assertion 'accepted candidate == p_cur' violations %ld | res-generalised ref != product ref %d\n", S.assert_bad, S.sanity_bad);
  std::printf("positive control (a) chain cell -> INF detected %d/%d | control (b) reversed tie order differs on %d requests\n",
              S.ctrl_hit, S.ctrl, S.tie_diff);
  if (argc > 4) { std::ofstream o(argv[4], std::ios::binary); o.write((const char*)tbp.data(), tbp.size()); }
  std::fclose(LOG);
  const bool pass = S.mism == 0 && S.assert_bad == 0 && S.sanity_bad == 0 && S.ctrl > 0 && S.ctrl_hit == S.ctrl && S.tie_diff >= 1 && key.req == 2;
  std::printf("%s\n", pass ? "PASS key comparison 1" : "FAIL key comparison 1");
  return pass ? 0 : 1;
}
