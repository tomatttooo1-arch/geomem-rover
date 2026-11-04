// CTRLPS1 closed-loop simulator: the CTRLPS0 vehicle model (proto/sim_rpp.py, ported 1:1) driving the
// kv_nav C++ product code.
//   kv_nav_sim --set gold|car1015 --dir DIR --mode track|core [--tier 1|2] [--adaptive 0|1]
//              [--product 0|1] [--latency S] [--out JSON] [--rows JSONL]
// mode track : kv_nav::Tracker on the fixed recorded path (what the tracker gate judges).
//              --product 1 adds the product pieces around it: per-tick rate limiter, forward
//              footprint check (crawl clamp applied; BLOCKED only counted, since replanning is
//              not modelled in this mode). --product 0 = the CTRLPS0 prototype controller.
// mode core  : the full kv_nav::NavCore (planner + smoothing + tracker + check + state machine) in the
//              loop on the case's static costmap, goal = recorded path end (state machine judged).
// set car1015: prep_car1015.py cases (path1/path2.f64le)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

#include "../kv_nav.hpp"

using namespace kv_nav;
static const double RES = 0.05, PI = 3.14159265358979323846;
static double wrapa(double a) { double r = std::fmod(a + PI, 2 * PI); if (r < 0) r += 2 * PI; return r - PI; }

struct Case { std::string name; std::vector<double> px, py; std::vector<unsigned char> map; bool has_map = false; int scx = -1, scy = -1; double yaw0 = 0, goal_yaw = 0; };

static bool rd(const std::string& p, std::vector<unsigned char>& o) { std::ifstream f(p, std::ios::binary); if (!f) return false; o.assign(std::istreambuf_iterator<char>(f), {}); return true; }
static double start_dir(const std::vector<double>& x, const std::vector<double>& y) {
  for (size_t i = 1; i < x.size(); ++i) if (std::hypot(x[i] - x[0], y[i] - y[0]) > 0.04) return std::atan2(y[i] - y[0], x[i] - x[0]);
  return 0; }
static double end_yaw(const std::vector<double>& x, const std::vector<double>& y) {
  for (size_t i = x.size() - 1; i > 0; --i) { double dx = x[i] - x[i - 1], dy = y[i] - y[i - 1]; if (std::hypot(dx, dy) > 1e-9) return std::atan2(dy, dx); }
  return 0; }
static double pathlen(const std::vector<double>& x, const std::vector<double>& y) { double s = 0; for (size_t i = 1; i < x.size(); ++i) s += std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]); return s; }
static double cross_track(const std::vector<double>& X, const std::vector<double>& Y, double x, double y) {
  double best = 1e300;
  for (size_t i = 0; i + 1 < X.size(); ++i) {
    double ax = X[i], ay = Y[i], bx = X[i + 1] - ax, by = Y[i + 1] - ay, den = std::max(bx * bx + by * by, 1e-12);
    double t = std::min(1.0, std::max(0.0, ((x - ax) * bx + (y - ay) * by) / den));
    best = std::min(best, std::hypot(ax + t * bx - x, ay + t * by - y)); }
  return best; }
static double percentile(std::vector<double> v, double q) {  // numpy default (linear)
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  double idx = q / 100.0 * (v.size() - 1); size_t lo = size_t(std::floor(idx)), hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (v[hi] - v[lo]) * (idx - lo); }

struct Row { std::string name; bool arrived = false, done = false; double t = 0, exy = 0, eyaw = 0, cte_max = 0; int unsafe = 0; double rot_min = -1;
  int blocked = 0, crawl = 0, replans = 0, switches = 0, recoveries = 0; int final_state = 0, reason = 0; double len = 0; std::vector<double> cte; };

static Row simulate(const Case& c, double lat, int mode, const Params& P, int product) {
  Row r; r.name = c.name; r.len = pathlen(c.px, c.py);
  const double dt = 0.001, tau = 0.10, a_v = 0.3, a_w = 0.6, t_max = 60.0;
  double x = c.px[0], y = c.py[0], th = c.yaw0, v = 0, w = 0, t = 0;
  const int lag = std::max(1, int(std::lround(lat / dt)));
  std::deque<std::pair<double, double>> q(lag, {0.0, 0.0});
  const int tick = int(std::lround(1.0 / P.hz / dt));
  std::pair<double, double> cmd{0, 0};
  std::vector<double> rot;
  // mode track
  Tracker tr(P);
  Window win; win.res = RES; win.cell_offset = 0.0; win.base_x = 0; win.base_y = 0;
  if (c.has_map) win.map = c.map.data();
  RateLimiter lim;
  if (mode == 0) {
    std::vector<double> X, Y; std::vector<std::uint8_t> near;
    std::vector<double> cx(c.px.size()), cy(c.py.size());
    for (size_t i = 0; i < cx.size(); ++i) { cx[i] = c.px[i] / RES; cy[i] = c.py[i] / RES; }
    if (c.has_map) build_track_path(P, win, cx, cy, X, Y, near);
    else { X = c.px; Y = c.py; near.assign(X.size(), 0); }
    tr.set_path(X, Y, near, c.goal_yaw, true);
  }
  // mode core
  NavCore core(P);
  std::vector<unsigned char> freemap(6400, 0);
  const unsigned char* cmap = c.has_map ? c.map.data() : freemap.data();
  const double gx = c.px.back(), gy = c.py.back();
  if (mode == 1) { core.update_costmap(cmap, 0, 0, 0.0); core.set_goal(1, gx, gy, c.goal_yaw, 0.0); }
  bool done = false; bool last_tight = false; double last_L = 0;
  for (long k = 0; k < long(t_max / dt); ++k) {
    t = k * dt;
    if (k % tick == 0) {
      if (mode == 0) {
        TrackOut o = tr.step(Pose{x, y, th}, v);
        last_tight = o.tight; last_L = o.lookahead;
        double cv = o.v, cw = o.w;
        if (product) {
          if (c.has_map && cv != 0) {
            CheckResult cr = forward_check(P, win, Pose{x, y, th}, cv, cw);
            if (cr == CHECK_BLOCKED) ++r.blocked;
            if (cr == CHECK_CRAWL && std::fabs(cv) > P.crawl_v) { double s = P.crawl_v / std::fabs(cv); cv *= s; cw *= s; ++r.crawl; }
          }
          if (o.done) { cv = 0; cw = 0; lim.reset(); } else lim.apply(P, cv, cw);
          if ((o.rotating || o.final_rotate) && !o.done && cv == 0 && cw != 0) rot.push_back(std::fabs(cw));
        } else {
          rot.insert(rot.end(), tr.rot_cmds.begin(), tr.rot_cmds.end()); tr.rot_cmds.clear();
        }
        cmd = {cv, cw}; done = o.done;
      } else {
        if (k % (tick * 2) == 0) core.update_costmap(cmap, 0, 0, t);  // 10 Hz costmap, static content
        Command o = core.tick(t, Pose{x, y, th}, v, w, true, false);
        if (o.flags & F_REPLANNED) ++r.replans;
        if (o.flags & F_PATH_SWITCHED) ++r.switches;
        if (o.flags & F_BLOCKED) ++r.blocked;
        if (o.flags & F_CRAWL) ++r.crawl;
        if (o.v == 0 && o.w != 0) rot.push_back(std::fabs(o.w));
        cmd = {o.v, o.w}; r.recoveries = o.recoveries; r.final_state = o.state; r.reason = o.reason;
        done = (o.state == SUCCEEDED || o.state == ABORTED);
      }
      if (k % (tick * 2) == 0) {
        if (mode == 0) r.cte.push_back(cross_track(c.px, c.py, x, y));
        else if (core.tracker().has_path()) r.cte.push_back(core.tracker().cross_track(Pose{x, y, th}));
        if (c.has_map) {
          const int cx = int(std::nearbyint(x / RES)), cy = int(std::nearbyint(y / RES));  // Python round()
          if (!(cx == c.scx && cy == c.scy) && (cx < 0 || cy < 0 || cx >= 80 || cy >= 80 || c.map[cy * 80 + cx] >= 253)) {
            ++r.unsafe;
            if (std::getenv("KVSIM_DEBUG") && c.name == std::getenv("KVSIM_DEBUG"))
              std::fprintf(stderr, "unsafe t=%.2f tight=%d L=%.2f pos=(%.3f,%.3f)cells yaw=%.2f cell=(%d,%d) cost=%d cte=%.3f\n", t, int(last_tight), last_L, x / RES, y / RES, th, cx, cy,
                           (cx < 0 || cy < 0 || cx >= 80 || cy >= 80) ? -1 : c.map[cy * 80 + cx], r.cte.empty() ? -1 : r.cte.back());
          }
        }
      }
      if (done) break;
    }
    q.push_back(cmd); if (int(q.size()) > lag) q.pop_front();
    double cv = q.front().first, cw = q.front().second;
    if (std::fabs(cv) < 0.01) cv = 0.0;
    if (std::fabs(v) < 0.02 && std::fabs(cw) < 0.045) cw = 0.0;
    const double dv = std::max(-a_v * dt, std::min(a_v * dt, (cv - v) * dt / tau));
    const double dw = std::max(-a_w * dt, std::min(a_w * dt, (cw - w) * dt / tau));
    v += dv; w += dw;
    x += v * std::cos(th) * dt; y += v * std::sin(th) * dt; th = wrapa(th + w * dt);
  }
  r.exy = std::hypot(gx - x, gy - y); r.eyaw = std::fabs(wrapa(c.goal_yaw - th)); r.t = t;
  r.done = (mode == 0) ? done : (r.final_state == SUCCEEDED);
  if (mode == 1 && r.done) {  // arrival vs the end of the path the core actually tracked (resolved goal)
    const auto& X = core.tracker().px(); const auto& Y = core.tracker().py();
    if (!X.empty()) r.exy = std::hypot(X.back() - x, Y.back() - y);
  }
  r.arrived = r.done && r.exy <= P.xy_tol + 0.02 && r.eyaw <= P.yaw_tol + 0.02;
  r.cte_max = r.cte.empty() ? 0 : *std::max_element(r.cte.begin(), r.cte.end());
  r.rot_min = rot.empty() ? -1 : *std::min_element(rot.begin(), rot.end());
  return r;
}

int main(int argc, char** argv) {
  std::string set = "car1015", dir, mode_s = "track", out, rows_out; int tier = 1, adaptive = 1, product = 1, synth = 1; double lat = 0.03;
  Params P;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string k = argv[i], v = argv[i + 1];
    if (k == "--set") set = v; else if (k == "--dir") dir = v; else if (k == "--mode") mode_s = v;
    else if (k == "--tier") tier = std::stoi(v); else if (k == "--adaptive") adaptive = std::stoi(v);
    else if (k == "--product") product = std::stoi(v); else if (k == "--latency") lat = std::stod(v);
    else if (k == "--out") out = v; else if (k == "--synth") synth = std::stoi(v); else if (k == "--rows") rows_out = v;
    else if (k == "--param") { auto e = v.find('='); std::string n = v.substr(0, e); double val = std::stod(v.substr(e + 1));
      if (n == "l_min") P.l_min = val; else if (n == "l_max") P.l_max = val; else if (n == "tight_lookahead") P.tight_lookahead = val;
      else if (n == "horizon_s") P.horizon_s = val; else if (n == "collision_check") P.collision_check = int(val);
      else if (n == "crawl_v") P.crawl_v = val; else if (n == "smooth") P.smooth = int(val); else if (n == "acc_w") P.acc_w = val;
      else if (n == "acc_v") P.acc_v = val; else if (n == "crawl_ring") P.crawl_ring = val; else if (n == "footprint_r") P.footprint_r = val; else if (n == "disk_blocks_inscribed") P.disk_blocks_inscribed = int(val);
      else { std::fprintf(stderr, "unknown param %s\n", n.c_str()); return 2; } }
  }
  const int mode = mode_s == "core" ? 1 : 0;
  P.adaptive = adaptive; P.cell_offset = 0.0;
  if (!product && mode == 0) { P.acc_v = 0; P.acc_w = 0; }
  std::vector<Case> cases;
  for (int i = 0; i < 1000; ++i) {
    char nm[8]; std::snprintf(nm, sizeof nm, "%03d", i);
    const std::string d = dir + "/" + nm + "/";
    std::vector<unsigned char> map, pb;
    if (!rd(d + "map.u8", map)) continue;
    const std::string pf = set == "gold" ? "path.f64le" : (tier == 2 ? "path2.f64le" : "path1.f64le");
    if (!rd(d + pf, pb) || pb.empty()) continue;
    Case c; c.name = nm; c.map = map; c.has_map = true;
    std::vector<double> g(pb.size() / 8); std::memcpy(g.data(), pb.data(), pb.size());
    for (size_t k = 0; k < g.size() / 2; ++k) { c.px.push_back(g[2 * k] * RES); c.py.push_back(g[2 * k + 1] * RES); }
    if (pathlen(c.px, c.py) < 1.0) continue;
    std::ifstream fr(d + "request.txt"); std::string a, b; fr >> a >> b; c.scx = int(std::stod(a)); c.scy = int(std::stod(b));
    c.yaw0 = start_dir(c.px, c.py); c.goal_yaw = end_yaw(c.px, c.py);
    cases.push_back(c);
  }
  if (synth) { Case s; s.name = "synth_straight"; for (int k = 0; k <= 40; ++k) { s.px.push_back(2.0 * k / 40); s.py.push_back(0); }
    s.yaw0 = start_dir(s.px, s.py); s.goal_yaw = 0; cases.push_back(s);
    Case b = s; b.name = "synth_back150"; b.yaw0 = 150 * PI / 180; cases.push_back(b); }
  if (mode == 1) {  // core: synthetic cases need a free map with the line inside the window
    for (auto& c : cases) if (!c.has_map) { c.map.assign(6400, 0); c.has_map = true;
      for (auto& v : c.px) v += 1.0;
      for (auto& v : c.py) v += 2.0;
      c.scx = int(std::nearbyint(c.px[0] / RES)); c.scy = int(std::nearbyint(c.py[0] / RES)); }
  }
  std::vector<Row> rows; std::vector<double> all;
  for (auto& c : cases) { rows.push_back(simulate(c, lat, mode, P, product)); all.insert(all.end(), rows.back().cte.begin(), rows.back().cte.end()); }
  int arrived = 0, unsafe = 0, blocked = 0, crawl = 0, blocked_cases = 0; double rot_min = 1e9; std::vector<double> ts;
  std::string unsafe_cases, not_arrived, blocked_list;
  for (auto& r : rows) {
    arrived += r.arrived; unsafe += r.unsafe; blocked += r.blocked; crawl += r.crawl; blocked_cases += r.blocked > 0;
    if (r.rot_min >= 0) rot_min = std::min(rot_min, r.rot_min);
    ts.push_back(r.t);
    if (r.unsafe) unsafe_cases += "\"" + r.name + "\",";
    if (!r.arrived) { char b[160]; std::snprintf(b, sizeof b, "[\"%s\",%.3f,%.3f,%.2f,\"%s\",\"%s\"],", r.name.c_str(), r.exy, r.eyaw, r.t, state_name(r.final_state), reason_name(r.reason)); not_arrived += b; }
  }
  auto trim = [](std::string s) { if (!s.empty()) s.pop_back(); return s; };
  char buf[4096];
  std::snprintf(buf, sizeof buf,
    "{\"set\":\"%s\",\"mode\":\"%s\",\"tier\":%d,\"adaptive\":%d,\"product\":%d,\"latency_s\":%.3f,\"n\":%zu,\"arrived\":%d,"
    "\"cte_p95_all\":%.4f,\"cte_max_all\":%.4f,\"unsafe_total\":%d,\"unsafe_cases\":[%s],\"rot_min\":%.4f,\"t_median\":%.3f,"
    "\"blocked_ticks\":%d,\"blocked_cases\":%d,\"crawl_ticks\":%d,\"not_arrived\":[%s]}",
    set.c_str(), mode_s.c_str(), tier, adaptive, product, lat, rows.size(), arrived, percentile(all, 95),
    all.empty() ? 0 : *std::max_element(all.begin(), all.end()), unsafe, trim(unsafe_cases).c_str(), rot_min > 1e8 ? -1 : rot_min,
    percentile(ts, 50), blocked, blocked_cases, crawl, trim(not_arrived).c_str());
  std::printf("%s\n", buf);
  if (!out.empty()) { std::ofstream f(out); f << buf << "\n"; }
  if (!rows_out.empty()) {
    std::ofstream f(rows_out);
    for (auto& r : rows) {
      char b[640];
      std::snprintf(b, sizeof b, "{\"name\":\"%s\",\"len_m\":%.3f,\"arrived\":%d,\"t\":%.2f,\"exy\":%.4f,\"eyaw\":%.4f,\"cte_max\":%.4f,\"cte_p95\":%.4f,"
        "\"unsafe\":%d,\"rot_min\":%.4f,\"blocked\":%d,\"crawl\":%d,\"replans\":%d,\"switches\":%d,\"recoveries\":%d,\"state\":\"%s\",\"reason\":\"%s\"}\n",
        r.name.c_str(), r.len, int(r.arrived), r.t, r.exy, r.eyaw, r.cte_max, percentile(r.cte, 95), r.unsafe, r.rot_min, r.blocked, r.crawl,
        r.replans, r.switches, r.recoveries, state_name(r.final_state), reason_name(r.reason));
      f << b;
    }
  }
  return 0;
}
