// PLNPATH0 A53 "before" column: the current PS path extraction, timed on the KV260 A53
// (format after RUN/A53BENCH0: one static program, one command, median per request).
//   a53_path_bench pack FROZEN_DIR CASES_DIR OUT.bin   (host: build the input pack, fields included)
//   a53_path_bench run  PACK.bin [passes]                (board: taskset -c 3 ./a53_path_bench run ...)
// Timed items (per request, field already in normal memory; nothing else inside the timer):
//   ps_extract   : check() + prepare() + extract() = what kv_nav runs today after the PL field
//                  (80: product extract_from_field(); 200: same code at 0.10 m cells, path_ps.hpp)
//   ps_after_pl  : what stays on the PS with the in-core extraction: prepare() + make_params()
//                  before the call, assemble() after it (the core's path words come from the C model,
//                  computed outside the timer).
// Self-check (untimed): ps_extract output == C model + assemble output for every request.
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sched.h>
#include <string>
#include <vector>
#include "path_ps.hpp"
namespace K = kv_nav_planner;
using namespace plnpath;

struct Req { std::string group; int W; K::Request q; std::vector<std::uint8_t> map; std::vector<std::uint32_t> field; };
static double now_us() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e6 + t.tv_nsec * 1e-3; }
static bool rd(const std::string& p, std::vector<unsigned char>& o) { std::ifstream f(p, std::ios::binary); if (!f) return false; o.assign(std::istreambuf_iterator<char>(f), {}); return true; }
template <class T> static void wr(std::ofstream& o, const T& v) { o.write((const char*)&v, sizeof v); }
template <class T> static T rv(std::ifstream& i) { T v; i.read((char*)&v, sizeof v); return v; }

template <int W, std::uint32_t Inf>
static void run_one(const Req& r, double& t_ext, double& t_after, bool& equal, unsigned long long& h) {
  typedef PS<W, W, Inf> P; typedef typename P::E E;
  const double res = W == 80 ? .05 : .10;
  static std::vector<std::uint8_t> m(40000), m2(40000);
  static std::vector<double> ox(40002), oy(40002), mx(40002), my(40002);
  // ---- current PS extraction
  double t0 = now_us();
  K::Result chk, a;
  if (E::check(r.map.data(), r.q, m.data(), r.field.data(), ox.data(), oy.data(), W * W + 2, chk)) {
    E::prepare(r.map.data(), r.q.sx, r.q.sy, m.data());
    if (W == 80) a = E::extract(m.data(), r.field.data(), r.q, ox.data(), oy.data(), W * W + 2);
    else a = P::ref_extract(m.data(), r.field.data(), r.q, res, ox.data(), oy.data(), W * W + 2);
  } else a = chk;
  t_ext = now_us() - t0;
  // ---- PS half with the in-core extraction (core output precomputed outside the timer)
  E::prepare(r.map.data(), r.q.sx, r.q.sy, m2.data());
  const PsParams pp0 = P::make_params(m2.data(), r.q, res);
  const ModelOut mo = model(r.map.data(), r.field.data(), W, Inf, r.q.sx, r.q.sy, int(std::round(r.q.gx)), int(std::round(r.q.gy)), pp0.bytes, true);
  t0 = now_us();
  E::prepare(r.map.data(), r.q.sx, r.q.sy, m2.data());
  const PsParams pp = P::make_params(m2.data(), r.q, res);
  K::Result b = P::assemble(mo.words.data(), pp, r.q, mx.data(), my.data());
  t_after = now_us() - t0;
  equal = a.status == b.status && a.count == b.count && std::memcmp(ox.data(), mx.data(), 8 * std::max(0, a.count)) == 0 &&
          std::memcmp(oy.data(), my.data(), 8 * std::max(0, a.count)) == 0;
  h = h * 1099511628211ull ^ unsigned(a.status) ^ (unsigned long long)a.count << 8;
  for (int i = 0; i < a.count; ++i) h = (h ^ (unsigned long long)ox[i] ^ ((unsigned long long)oy[i] << 16)) * 1099511628211ull;
}

int main(int argc, char** argv) {
  if (argc >= 5 && std::string(argv[1]) == "pack") {
    std::vector<Req> v;
    std::vector<unsigned char> in, ex; rd(std::string(argv[2]) + "/input.bin", in); rd(std::string(argv[2]) + "/expected.bin", ex);
    size_t ai = 12, ae = 12; auto u = [](const std::vector<unsigned char>& b, size_t& at) { unsigned x; std::memcpy(&x, &b[at], 4); at += 4; return x; };
    for (int c = 0; c < 2; ++c) {
      const unsigned mode = u(in, ai); int sx = u(in, ai), sy = u(in, ai), gx = u(in, ai), gy = u(in, ai); u(in, ai);
      const int n = mode ? 40000 : 6400, nf = mode ? 256 : 64;
      Req r{mode ? "classroom200" : "pressure80", mode ? 200 : 80, K::Request{sx, sy, double(gx), double(gy)}, {in.begin() + ai, in.begin() + ai + n}, std::vector<std::uint32_t>(n)};
      ai += n; ae += 12; for (int i = 0; i < n; ++i) r.field[i] = u(ex, ae); ae += 4 * nf; v.push_back(r);
    }
    for (int i = 0; i < 1000; ++i) {
      char nm[8]; std::snprintf(nm, sizeof nm, "%03d", i); std::vector<unsigned char> map;
      if (!rd(std::string(argv[3]) + "/" + nm + "/map.u8", map) || map.size() != 6400) continue;
      std::ifstream fr(std::string(argv[3]) + "/" + nm + "/request.txt"); double sx, sy, gx, gy; fr >> sx >> sy >> gx >> gy;
      Req r{"car1015", 80, K::Request{int(sx), int(sy), gx, gy}, map, std::vector<std::uint32_t>(6400)};
      std::vector<std::uint8_t> m(6400); unsigned rounds = 0;
      if (!PS<80, 80, 0xffffffu>::E::scan(map.data(), int(sx), int(sy), m.data(), r.field.data(), 0, rounds, nullptr)) continue;
      v.push_back(r);
    }
    std::ofstream o(argv[4], std::ios::binary); o.write("PLNPB001", 8); wr(o, unsigned(v.size()));
    for (auto& r : v) {
      char g[16] = {0}; std::strncpy(g, r.group.c_str(), 15); o.write(g, 16);
      wr(o, r.W); wr(o, r.q.sx); wr(o, r.q.sy); wr(o, r.q.gx); wr(o, r.q.gy);
      o.write((const char*)r.map.data(), r.map.size()); o.write((const char*)r.field.data(), 4 * r.field.size());
    }
    std::printf("packed %zu requests -> %s\n", v.size(), argv[4]); return 0;
  }
  if (argc < 3 || std::string(argv[1]) != "run") { std::printf("usage: %s pack FROZEN CASES OUT | run PACK [passes]\n", argv[0]); return 2; }
  const int passes = argc > 3 ? std::atoi(argv[3]) : 50;
  cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(3, &cs); const bool aff = sched_setaffinity(0, sizeof cs, &cs) == 0;
  sched_param sp; sp.sched_priority = 50; const bool fifo = sched_setscheduler(0, SCHED_FIFO, &sp) == 0;
  std::ifstream in(argv[2], std::ios::binary); char mg[8]; in.read(mg, 8);
  if (std::string(mg, 8) != "PLNPB001") { std::printf("bad pack\n"); return 2; }
  const unsigned n = rv<unsigned>(in); std::vector<Req> v(n);
  for (auto& r : v) {
    char g[16]; in.read(g, 16); r.group = std::string(g, strnlen(g, 16));
    r.W = rv<int>(in); r.q.sx = rv<int>(in); r.q.sy = rv<int>(in); r.q.gx = rv<double>(in); r.q.gy = rv<double>(in);
    r.map.resize(r.W * r.W); r.field.resize(r.W * r.W);
    in.read((char*)r.map.data(), r.map.size()); in.read((char*)r.field.data(), 4 * r.field.size());
  }
  std::vector<std::vector<double>> te(n), ta(n);
  unsigned eqn = 0; unsigned long long h = 1469598103934665603ull;
  for (int p = -3; p < passes; ++p) {
    for (unsigned i = 0; i < n; ++i) {
      double a, b; bool e; unsigned long long hh = 0;
      if (v[i].W == 80) run_one<80, 0xffffffu>(v[i], a, b, e, hh); else run_one<200, 0x1fffffffu>(v[i], a, b, e, hh);
      if (p == 0) { eqn += e; h ^= hh + i; }
      if (p >= 0) { te[i].push_back(a); ta[i].push_back(b); }
    }
  }
  auto med = [](std::vector<double> x) { std::sort(x.begin(), x.end()); return x[x.size() / 2]; };
  std::printf("a53_path_bench requests %u passes %d affinity_core3 %d fifo %d | self-check ps_extract == core+assemble %u/%u | hash %016llx\n",
              n, passes, int(aff), int(fifo), eqn, n, h);
  std::printf("| group | requests | ps_extract median us (median of per-request medians) | max per-request median us | ps_after_pl median us |\n|---|---|---|---|---|\n");
  for (const char* g : {"pressure80", "car1015", "classroom200"}) {
    std::vector<double> me, ma;
    for (unsigned i = 0; i < n; ++i) if (v[i].group == g) { me.push_back(med(te[i])); ma.push_back(med(ta[i])); }
    if (me.empty()) continue;
    std::printf("| %s | %zu | %.2f | %.2f | %.2f |\n", g, me.size(), med(me), *std::max_element(me.begin(), me.end()), med(ma));
  }
  return eqn == n ? 0 : 1;
}
