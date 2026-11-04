// CTRLPS1 key test 4: per-cycle host timing of the kv_nav pieces, to set against design §4.
//   kv_nav_bench CASES_DIR [reps]
// On the 127 car1015 cases: (a) planning split into soft field build / extraction / smoothing,
// (b) one control tick split into Tracker::step / forward_check / full NavCore::tick (no replan
// ticks vs replan ticks), from an ideal-kinematics closed loop at 20 Hz.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "../kv_nav.hpp"
#include "../planner_split.hpp"
#include "../smooth_v3.hpp"

using namespace kv_nav;
using clk = std::chrono::steady_clock;
static double us(clk::time_point a, clk::time_point b) { return std::chrono::duration<double, std::micro>(b - a).count(); }
static void report(const char* name, std::vector<double> v) {
  if (v.empty()) { std::printf("%-34s n=0\n", name); return; }
  std::sort(v.begin(), v.end());
  auto q = [&](double p) { return v[std::min(v.size() - 1, size_t(p * (v.size() - 1) + 0.5))]; };
  std::printf("%-34s n=%-6zu median %8.1f us  p95 %8.1f  p99 %8.1f  max %8.1f\n", name, v.size(), q(.5), q(.95), q(.99), v.back());
}
static volatile double sink = 0;

int main(int argc, char** argv) {
  if (argc < 2) { std::printf("usage: %s CASES_DIR [reps]\n", argv[0]); return 2; }
  const int reps = argc > 2 ? std::atoi(argv[2]) : 5;
  std::vector<double> t_scan, t_extract, t_smooth, t_plan_total, t_track, t_check, t_tick, t_tick_replan;
  int ncase = 0;
  for (int i = 0; i < 1000; ++i) {
    char nm[8]; std::snprintf(nm, sizeof nm, "%03d", i);
    const std::string d = std::string(argv[1]) + "/" + nm + "/";
    std::ifstream fm(d + "map.u8", std::ios::binary); if (!fm) continue;
    std::vector<std::uint8_t> map(6400); fm.read(reinterpret_cast<char*>(map.data()), 6400);
    std::ifstream fr(d + "request.txt"); int sx, sy; double gx, gy; fr >> sx >> sy >> gx >> gy;
    ++ncase;
    // (a) planning pieces
    std::vector<std::uint8_t> m(6400); std::vector<std::uint32_t> p(6400); std::vector<double> ox(6402), oy(6402);
    using E = kv_nav_planner::detail::Engine<80, 80, kv_nav_planner::kInfinity>;
    for (int r = 0; r < reps; ++r) {
      unsigned rounds = 0;
      auto a = clk::now();
      bool ok = E::scan(map.data(), sx, sy, m.data(), p.data(), 0, rounds, nullptr);
      auto b = clk::now();
      auto res = kv_nav_planner::extract_from_field(map.data(), kv_nav_planner::Request{sx, sy, gx, gy}, p.data(), rounds, m.data(), ox.data(), oy.data(), 6402);
      auto c = clk::now();
      int n = res.count;
      if (res.status <= 1) n = kv_nav_smooth::constrained_smooth(m.data(), 80, 80, ox.data(), oy.data(), n);
      auto e = clk::now();
      sink += ok + n;
      t_scan.push_back(us(a, b)); t_extract.push_back(us(b, c)); t_smooth.push_back(us(c, e)); t_plan_total.push_back(us(a, e));
    }
    Params P; P.cell_offset = 0.0;
    NavCore core(P);
    Pose pose{sx * 0.05, sy * 0.05, 0};
    core.update_costmap(map.data(), 0, 0, 0); core.set_goal(1, gx * 0.05, gy * 0.05, 0, 0);
    Window win{map.data(), 0, 0, 0.05, 0.0};
    double t = 0, v = 0;
    for (int k = 0; k < 1200; ++k) {
      if (k % 2 == 0) core.update_costmap(map.data(), 0, 0, t);
      auto a = clk::now();
      Command cmd = core.tick(t, pose, v, 0, true, false);
      auto b = clk::now();
      ((cmd.flags & F_REPLANNED) ? t_tick_replan : t_tick).push_back(us(a, b));
      if (core.tracker().has_path()) {  // the two inner pieces, timed on a copy (same inputs)
        Tracker tr = core.tracker();
        auto c0 = clk::now(); TrackOut o = tr.step(pose, v); auto c1 = clk::now();
        CheckResult cr = forward_check(P, win, pose, o.v != 0 ? o.v : 0.15, o.w); auto c2 = clk::now();
        sink += o.v + int(cr);
        t_track.push_back(us(c0, c1)); t_check.push_back(us(c1, c2));
      }
      v = cmd.v;
      pose.x += cmd.v * std::cos(pose.yaw) * 0.05; pose.y += cmd.v * std::sin(pose.yaw) * 0.05; pose.yaw += cmd.w * 0.05;
      t += 0.05;
      if (cmd.state == SUCCEEDED || cmd.state == ABORTED) break;
    }
  }
  std::printf("kv_nav bench: %d car1015 cases, %d reps for planning\n", ncase, reps);
  report("plan: soft field build (scan)", t_scan);
  report("plan: extraction (descent+safety)", t_extract);
  report("plan: v3 tier-2 smoothing", t_smooth);
  report("plan: total", t_plan_total);
  report("tick: Tracker::step", t_track);
  report("tick: forward_check (20 poses)", t_check);
  report("tick: NavCore::tick, no replan", t_tick);
  report("tick: NavCore::tick, with replan", t_tick_replan);
  return sink == 12345.678 ? 1 : 0;
}
