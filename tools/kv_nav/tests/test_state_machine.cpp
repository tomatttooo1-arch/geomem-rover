// CTRLPS1 key test 3: state machine transitions + footprint check + PL hook, each with a control.
// Ideal unicycle (commands applied exactly), 20 Hz, 80x80 window at base (0,0), cell_offset 0.5.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "../kv_nav.hpp"

using namespace kv_nav;
static int fails = 0, checks = 0;
#define CHECK(c, ...) do { ++checks; if (!(c)) { ++fails; std::printf("FAIL %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Sim {
  NavCore core;
  std::vector<std::uint8_t> map = std::vector<std::uint8_t>(6400, 0);
  Pose pose; double t = 0, v = 0, w = 0;
  bool permit = true, teleop = false, frozen = false;
  std::vector<int> states; std::set<int> seen; unsigned flags_or = 0;
  double min_rot = 1e9; int unsafe = 0; Command last;
  explicit Sim(Params p = Params()) : core(p) {}
  // cells -> metres for cell_offset 0.5: centre of cell i is (i + 0.5) * 0.05
  static double m(double cell) { return (cell + 0.5) * 0.05; }
  void push_map() { core.update_costmap(map.data(), 0, 0, t); }
  void step() {
    if (int(std::lround(t * 20)) % 2 == 0) push_map();  // 10 Hz
    last = core.tick(t, pose, v, w, permit, teleop);
    if (states.empty() || states.back() != last.state) states.push_back(last.state);
    seen.insert(last.state); flags_or |= last.flags;
    if (last.v == 0 && last.w != 0) min_rot = std::min(min_rot, std::fabs(last.w));
    v = frozen ? 0 : last.v; w = frozen ? 0 : last.w;
    const double dt = 0.05;
    pose.x += v * std::cos(pose.yaw) * dt; pose.y += v * std::sin(pose.yaw) * dt; pose.yaw += w * dt;
    const int cx = int(std::floor(pose.x / 0.05)), cy = int(std::floor(pose.y / 0.05));
    if (map[cy * 80 + cx] >= 253) ++unsafe;
    t += dt;
  }
  void run_until(double tend, int stop_state = -1) { while (t < tend) { step(); if (last.state == stop_state) break; } }
};
static std::string seq(const std::vector<int>& s) { std::string o; for (int x : s) { o += state_name(x); o += ">"; } return o; }
static void wall(std::vector<std::uint8_t>& m, int x0, int x1, int y0, int y1, std::uint8_t c) {
  for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) m[y * 80 + x] = c; }


// CTRLPS5: a car-centred moving window (base = floor(p / 0.05) - 40, as the rolling map / SimWorldGrid) over a
// world given per world cell; L-shaped route around a block, so the goal enters the window while the old
// (window-exit) path still ends short of it. Returns the goal's detour count and the remaining before/after the
// switch to the final path.
struct MovingRun { int detours = 0; double before = -1, after = -1; int end_state = -1; double t = 0; int unsafe = 0; };
static int world_cost(int ix, int iy, int popup) {
  // block x [0.50, 2.50) y [-1.75, 1.50) m: 254 inside, 253 within 6 cells (0.30 m), then a gradient to 0 at 12
  auto rect = [](int ix_, int iy_, int x0, int x1, int y0, int y1) {
    const int dx = ix_ < x0 ? x0 - ix_ : (ix_ >= x1 ? ix_ - x1 + 1 : 0), dy = iy_ < y0 ? y0 - iy_ : (iy_ >= y1 ? iy_ - y1 + 1 : 0);
    return std::max(dx, dy); };
  int d = rect(ix, iy, 10, 50, -35, 30);
  if (popup) d = std::min(d, rect(ix, iy, popup, popup + 4, -60, -36));
  if (d == 0) return 254;
  if (d <= 6) return 253;
  if (d <= 12) return 252 - 40 * (d - 6);
  return 0;
}
static MovingRun moving_window_run(Params p, double gx, double gy) {
  const int popup = 0; const double popup_t = 1e9;
  NavCore core(p);
  Pose pose{0.025, 0.025, -1.5707963};
  std::vector<double> rx, ry;
  for (double y = 0; y > -2.3; y -= 0.05) { rx.push_back(0.025); ry.push_back(y); }
  for (double x = 0.025; x < gx; x += 0.05) { rx.push_back(x); ry.push_back(gy); }
  rx.push_back(gx); ry.push_back(gy);
  core.set_global_route(rx, ry);
  core.set_goal(1, gx, gy, 0, 0);
  std::vector<std::uint8_t> win(6400);
  MovingRun r; double t = 0, prev_rem = -1; Command c;
  for (int k = 0; k < 20 * 120; ++k, t += 0.05) {
    if (k % 2 == 0) {
      const int bx = int(std::floor(pose.x / 0.05)) - 40, by = int(std::floor(pose.y / 0.05)) - 40;
      for (int ly = 0; ly < 80; ++ly) for (int lx = 0; lx < 80; ++lx)
        win[ly * 80 + lx] = std::uint8_t(world_cost(bx + lx, by + ly, t >= popup_t ? popup : 0));
      core.update_costmap(win.data(), bx, by, t);
    }
    c = core.tick(t, pose, 0, 0, true, false);
    if (c.reason == R_DETOUR && c.state == RECOVER_OBSERVE && r.detours == 0) r.detours = 1;
    if (prev_rem > 0 && c.remaining_m - prev_rem > 0.5 && r.before < 0) { r.before = prev_rem; r.after = c.remaining_m; }
    prev_rem = c.remaining_m;
    pose.x += c.v * std::cos(pose.yaw) * 0.05; pose.y += c.v * std::sin(pose.yaw) * 0.05; pose.yaw += c.w * 0.05;
    if (world_cost(int(std::floor(pose.x / 0.05)), int(std::floor(pose.y / 0.05)), t >= popup_t ? popup : 0) >= 253) ++r.unsafe;
    if (c.state == SUCCEEDED || c.state == ABORTED) break;
  }
  r.end_state = c.state; r.t = t;
  return r;
}

int main() {
  // ---------------------------------------------------------------- footprint check unit + controls
  {
    Params p; std::vector<std::uint8_t> map(6400, 0);
    Window win{map.data(), 0, 0, 0.05, 0.5};
    Pose at{Sim::m(40), Sim::m(40), 0};
    CHECK(forward_check(p, win, at, 0.15, 0) == CHECK_OK, "free map must be OK");
    map[40 * 80 + 42] = 253;  // 0.10 m ahead, inscribed
    CHECK(forward_check(p, win, at, 0.15, 0) == CHECK_BLOCKED, "inscribed 0.10 m ahead must block");
    CHECK(forward_check(p, win, at, 0.0, 0.3) == CHECK_OK, "in-place rotation never blocks");
    CHECK(forward_check(p, win, at, -0.15, 0) == CHECK_OK, "moving away is OK");
    map[40 * 80 + 42] = 0; map[36 * 80 + 41] = 253;  // 0.20 m to the side: centre never enters
    CHECK(forward_check(p, win, at, 0.15, 0) == CHECK_OK, "inscribed beside the line: centre semantics OK");
    p.disk_blocks_inscribed = 1;
    CHECK(forward_check(p, win, at, 0.15, 0) == CHECK_BLOCKED, "literal disk-vs-253 rule blocks it (diagnostic mode)");
    p.disk_blocks_inscribed = 0; map[36 * 80 + 41] = 0;
    map[40 * 80 + 48] = 255;  // unknown 0.40 m ahead: inside the 0.65 m ring -> crawl
    CHECK(forward_check(p, win, at, 0.15, 0) == CHECK_CRAWL, "unknown inside ring must crawl");
    p.crawl_ring = 0.30;      // control: same unknown outside a smaller ring -> block
    CHECK(forward_check(p, win, at, 0.15, 0) == CHECK_BLOCKED, "unknown outside ring must block");
    map[40 * 80 + 48] = 0;
    Pose edge{Sim::m(78), Sim::m(40), 0};  // out-of-window counts as unknown
    p.crawl_ring = 0.65;
    CHECK(forward_check(p, win, edge, 0.15, 0) == CHECK_CRAWL, "window edge ahead treated as unknown");
  }
  // ---------------------------------------------------------------- T1 arrive (goal behind: rotate first; final yaw)
  {
    Sim s; s.pose = {Sim::m(40), Sim::m(40), 3.14159};  // facing -x, goal at +x
    s.push_map(); s.core.set_goal(1, Sim::m(60), Sim::m(40), 1.5708, 0);
    s.run_until(60, SUCCEEDED);
    CHECK(s.last.state == SUCCEEDED, "T1 not succeeded: %s", seq(s.states).c_str());
    CHECK(s.seen.count(ROTATE_TO_PATH) && s.seen.count(TRACK) && s.seen.count(FINAL_ROTATE), "T1 path: %s", seq(s.states).c_str());
    CHECK(s.min_rot >= 0.12 - 1e-12, "T1 in-place |w| %.3f < 0.12", s.min_rot);
    CHECK(std::hypot(s.pose.x - Sim::m(60), s.pose.y - Sim::m(40)) <= 0.10 + 1e-9, "T1 xy error");
    CHECK(s.last.v == 0 && s.last.w == 0, "T1 final command not zero");
    std::printf("T1 arrive: %s t=%.2f s\n", seq(s.states).c_str(), s.t);
  }
  // ---------------------------------------------------------------- T2 stuck -> wait x2 -> observe -> abort
  {
    Sim s; s.pose = {Sim::m(40), Sim::m(40), 0}; s.frozen = true;  // wheels never move
    s.push_map(); s.core.set_goal(2, Sim::m(60), Sim::m(40), 0, 0);
    double first = -1;
    while (s.t < 200 && s.last.state != ABORTED) { s.step(); if (first < 0 && s.last.state == RECOVER_WAIT) first = s.t; }
    CHECK(first >= 20 - 0.1 && first <= 20.2, "T2 first recovery at %.2f (want ~20 s)", first);
    CHECK(s.last.state == ABORTED && s.last.reason == R_STUCK, "T2 end %s/%s", state_name(s.last.state), reason_name(s.last.reason));
    CHECK(s.seen.count(RECOVER_OBSERVE), "T2 no observe step: %s", seq(s.states).c_str());
    std::printf("T2 stuck: %s t=%.2f recoveries=%d\n", seq(s.states).c_str(), s.t, s.last.recoveries);
  }
  // ---------------------------------------------------------------- T3 plan fails -> wait -> new map -> succeed
  {
    Sim s; s.pose = {Sim::m(40), Sim::m(40), 0};
    wall(s.map, 50, 52, 1, 78, 254);  // full wall: NO_PATH
    s.push_map(); s.core.set_goal(3, Sim::m(60), Sim::m(40), 0, 0);
    s.run_until(0.6);
    CHECK(s.last.state == RECOVER_WAIT && s.last.reason == R_PLAN_FAILED, "T3 want RECOVER_WAIT/PLAN_FAILED got %s/%s", state_name(s.last.state), reason_name(s.last.reason));
    wall(s.map, 50, 52, 1, 78, 0);   // the obstacle turns out not to be there
    s.run_until(60, SUCCEEDED);
    CHECK(s.last.state == SUCCEEDED, "T3 not recovered: %s", seq(s.states).c_str());
    std::printf("T3 recover: %s t=%.2f\n", seq(s.states).c_str(), s.t);
  }
  // ---------------------------------------------------------------- T4 permit withheld 30 s mid-track -> no stuck -> replan -> succeed
  {
    Sim s; s.pose = {Sim::m(40), Sim::m(40), 0};
    s.push_map(); s.core.set_goal(4, Sim::m(70), Sim::m(40), 0, 0);
    s.run_until(3);
    CHECK(s.last.state == TRACK, "T4 not tracking at 3 s: %s", state_name(s.last.state));
    s.permit = false; s.run_until(33);
    CHECK(s.last.state == PAUSED && s.last.v == 0 && s.last.w == 0, "T4 not paused/zero");
    CHECK(!s.seen.count(RECOVER_WAIT), "T4 stuck clock ran while paused");
    s.permit = true; s.step();
    CHECK(s.last.flags & F_REPLANNED, "T4 no replan on release");
    s.run_until(80, SUCCEEDED);
    CHECK(s.last.state == SUCCEEDED, "T4 not succeeded: %s", seq(s.states).c_str());
    s.teleop = false;
    std::printf("T4 permit: %s t=%.2f\n", seq(s.states).c_str(), s.t);
  }
  // ---------------------------------------------------------------- T5 obstacle appears on the path -> replan around, never inside
  {
    Sim s; s.pose = {Sim::m(20), Sim::m(40), 0};
    s.push_map(); s.core.set_goal(5, Sim::m(60), Sim::m(40), 0, 0);
    s.run_until(2);
    wall(s.map, 34, 49, 27, 53, 253); wall(s.map, 40, 43, 33, 47, 254);  // inflated box ~0.4 m ahead of the car
    s.run_until(120, SUCCEEDED);
    CHECK(s.last.state == SUCCEEDED, "T5 not succeeded: %s", seq(s.states).c_str());
    CHECK(s.unsafe == 0, "T5 entered >=253 %d ticks", s.unsafe);
    CHECK(s.flags_or & F_PATH_SWITCHED, "T5 never switched path");
    CHECK(s.pose.y < Sim::m(27) || s.pose.y > Sim::m(53) || s.pose.x > Sim::m(49) || s.pose.x < Sim::m(34), "T5 end inside box?");
    std::printf("T5 obstacle: %s t=%.2f blocked_flag=%d\n", seq(s.states).c_str(), s.t, int((s.flags_or & F_BLOCKED) != 0));
    // control: the same box closing the whole corridor -> recover -> abort, never inside
    Sim b; b.pose = {Sim::m(20), Sim::m(40), 0};
    b.push_map(); b.core.set_goal(6, Sim::m(60), Sim::m(40), 0, 0);
    b.run_until(2);
    wall(b.map, 34, 49, 1, 78, 253); wall(b.map, 40, 43, 1, 78, 254);
    b.run_until(120, ABORTED);
    CHECK(b.last.state == ABORTED, "T5b not aborted: %s", seq(b.states).c_str());
    CHECK(b.unsafe == 0, "T5b entered >=253");
    std::printf("T5b closed: %s reason=%s t=%.2f\n", seq(b.states).c_str(), reason_name(b.last.reason), b.t);
  }
  // ---------------------------------------------------------------- T5c only the forward check can see it (path-ahead check off)
  {
    Params p; p.block_lookahead = 0; p.replan_period = 1e9;  // no timer replans either
    Sim s(p); s.pose = {Sim::m(20), Sim::m(40), 0};
    s.push_map(); s.core.set_goal(10, Sim::m(60), Sim::m(40), 0, 0);
    s.run_until(2);
    wall(s.map, 34, 49, 27, 53, 253); wall(s.map, 40, 43, 33, 47, 254);
    s.run_until(120, SUCCEEDED);
    CHECK(s.flags_or & F_BLOCKED, "T5c forward check never fired");
    CHECK(s.unsafe == 0, "T5c entered >=253 %d ticks", s.unsafe);
    CHECK(std::hypot(s.pose.x - Sim::m(60), s.pose.y - Sim::m(40)) <= 0.15 + 1e-9, "T5c xy error %.3f", std::hypot(s.pose.x - Sim::m(60), s.pose.y - Sim::m(40)));
    // CTRLPS1 known liveness weakness, fixed in CTRLPS3 (controller ruling 3): detected late, the detour path
    // turns 90 deg around the box's inscribed corner; any pure-pursuit arc projected 1 s clips that corner
    // cell. Now: blocked -> turn first toward the path, then go straight at cell scale (careful mode) -> the
    // car gets through, still never entering >=253.
    CHECK(s.last.state == SUCCEEDED, "T5c end %s/%s (want SUCCEEDED)", state_name(s.last.state), reason_name(s.last.reason));
    CHECK(s.flags_or & F_CAREFUL, "T5c careful mode never used");
    std::printf("T5c check-only (default): %s reason=%s t=%.2f\n", seq(s.states).c_str(), reason_name(s.last.reason), s.t);
    // control A: careful mode off (CTRLPS1 behaviour) -> same scene ends ABORTED/BLOCKED, never inside
    Params pa = p; pa.careful = 0;
    Sim a(pa); a.pose = {Sim::m(20), Sim::m(40), 0};
    a.push_map(); a.core.set_goal(12, Sim::m(60), Sim::m(40), 0, 0);
    a.run_until(2);
    wall(a.map, 34, 49, 27, 53, 253); wall(a.map, 40, 43, 33, 47, 254);
    a.run_until(120, ABORTED);
    CHECK(a.last.state == ABORTED && a.last.reason == R_BLOCKED, "T5c control A (careful off) end %s/%s", state_name(a.last.state), reason_name(a.last.reason));
    CHECK(a.unsafe == 0, "T5c control A entered >=253 %d ticks", a.unsafe);
    std::printf("T5c control A (careful off): %s reason=%s t=%.2f\n", seq(a.states).c_str(), reason_name(a.last.reason), a.t);
    // control: check disabled too -> the car drives into the box (proves the check is what saved T5c)
    Params q = p; q.collision_check = 0;
    Sim c(q); c.pose = {Sim::m(20), Sim::m(40), 0};
    c.push_map(); c.core.set_goal(11, Sim::m(60), Sim::m(40), 0, 0);
    c.run_until(2);
    wall(c.map, 34, 49, 27, 53, 253); wall(c.map, 40, 43, 33, 47, 254);
    c.run_until(30);
    CHECK(c.unsafe > 0, "T5c control: without the check the car should enter the box");
    std::printf("T5c control (check off): unsafe ticks %d\n", c.unsafe);
  }
  // ---------------------------------------------------------------- T6 cancel
  {
    Sim s; s.pose = {Sim::m(40), Sim::m(40), 0};
    s.push_map(); s.core.set_goal(7, Sim::m(70), Sim::m(40), 0, 0);
    s.run_until(2); s.core.cancel(s.t); s.step();
    CHECK(s.last.state == CANCELED && s.last.v == 0 && s.last.w == 0, "T6 cancel");
  }
  // ---------------------------------------------------------------- T7 PL hook: ok / NOT_CONVERGED fallback / corrupt -> self-check -> disable
  {
    struct U { int mode = 0; int calls = 0; };
    auto pl = [](void* user, const std::uint8_t* map, int sx, int sy, int, int, unsigned, std::uint32_t* pot, unsigned* rounds) -> int {
      U* u = static_cast<U*>(user); ++u->calls;
      if (u->mode == 1) return 7;
      if (!soft_field(map, sx, sy, pot, rounds)) return 7;
      if (u->mode == 2) pot[sy * 80 + sx + 3] += 1;  // corrupt one cell
      return 0;
    };
    for (int mode = 0; mode < 3; ++mode) {
      Params p; p.backend = BACKEND_PL; p.selfcheck_every = 1;
      Sim s(p); U u; u.mode = mode; s.core.set_pl(pl, &u);
      s.pose = {Sim::m(40), Sim::m(40), 0}; s.push_map(); s.core.set_goal(8, Sim::m(60), Sim::m(40), 0, 0);
      std::set<int> src; while (s.t < 60 && s.last.state != SUCCEEDED) { s.step(); if (s.last.flags & F_REPLANNED) src.insert(s.last.field_source); }
      CHECK(s.last.state == SUCCEEDED, "T7 mode %d not succeeded", mode);
      if (mode == 0) CHECK(src == std::set<int>{FS_PL} && !(s.flags_or & F_SELFCHECK_MISMATCH), "T7 ok: PL only, no mismatch");
      if (mode == 1) CHECK(src == std::set<int>{FS_SOFT_FALLBACK}, "T7 NOT_CONVERGED must fall back");
      if (mode == 2) CHECK((s.flags_or & F_SELFCHECK_MISMATCH) && (s.flags_or & F_PL_DISABLED) && src.count(FS_SOFT), "T7 corrupt PL must be caught and disabled");
      std::printf("T7 PL mode %d: pl_calls=%d sources=%zu flags=0x%x\n", mode, u.calls, src.size(), s.flags_or);
    }
  }
  // ---------------------------------------------------------------- T8 goal outside the window: exit-point goal, no early success
  {
    Sim s; s.pose = {Sim::m(40), Sim::m(40), 0};
    s.push_map(); s.core.set_goal(9, Sim::m(40) + 5.0, Sim::m(40), 0, 0);
    s.run_until(3);
    auto xy = s.core.path_odom_xy();
    CHECK(!xy.empty() && xy[xy.size() - 2] >= Sim::m(77), "T8 path does not reach the window exit (end x %.2f)", xy.empty() ? -1 : xy[xy.size() - 2]);
    CHECK(s.last.state == TRACK, "T8 not tracking: %s", state_name(s.last.state));
  }
  // ---------------------------------------------------------------- T9 goal enters the moving window (CTRLPS5)
  {
    const double gx = 1.525, gy = -2.275;
    Params p0; p0.detour_nonfinal_extend = 0;
    MovingRun a = moving_window_run(p0, gx, gy);
    MovingRun b = moving_window_run(Params(), gx, gy);
    std::printf("T9 window entry: CTRLPS1 rule detour=%d (%.3f -> %.3f, ratio %.3f) end %s t=%.1f | fixed detour=%d (%.3f -> %.3f) end %s t=%.1f\n",
                a.detours, a.before, a.after, a.after / a.before, state_name(a.end_state), a.t,
                b.detours, b.before, b.after, state_name(b.end_state), b.t);
    CHECK(a.detours == 1, "T9 control: the CTRLPS1 rule should trip 'detour' at the window entry (ratio %.3f)", a.after / a.before);
    CHECK(b.detours == 0, "T9 fixed rule tripped 'detour' at the window entry");
    CHECK(b.end_state == SUCCEEDED && a.end_state == SUCCEEDED, "T9 end %s / %s", state_name(a.end_state), state_name(b.end_state));
    CHECK(b.t < a.t, "T9 fixed should be faster (%.1f vs %.1f s)", b.t, a.t);
    CHECK(a.unsafe == 0 && b.unsafe == 0, "T9 entered >=253");
    // positive control of the fixed comparison at the same switch: a ratio below 1 must trip it
    Params pr; pr.detour_ratio = 0.9; pr.detour_min_extra = 0;
    MovingRun c = moving_window_run(pr, gx, gy);
    CHECK(c.detours == 1, "T9 control: fixed rule with ratio 0.9 should trip at the window entry");
  }
  // ---------------------------------------------------------------- T10 genuine detour, final -> final: both rules observe
  for (int arm = 0; arm < 2; ++arm) {
    Params p; p.detour_nonfinal_extend = arm;
    Sim s(p); s.pose = {Sim::m(20), Sim::m(40), 0};
    s.push_map(); s.core.set_goal(20 + arm, Sim::m(60), Sim::m(40), 0, 0);
    s.run_until(3);
    wall(s.map, 38, 52, 14, 78, 253); wall(s.map, 44, 46, 20, 78, 254);  // wall across, gap only at the bottom
    bool det = false;
    while (s.t < 120 && s.last.state != SUCCEEDED && s.last.state != ABORTED) { s.step(); det |= s.last.reason == R_DETOUR && s.last.state == RECOVER_OBSERVE; }
    CHECK(det, "T10 arm %d: genuine detour (final -> final) did not observe", arm);
    CHECK(s.unsafe == 0, "T10 arm %d entered >=253", arm);
    std::printf("T10 genuine detour arm %d: observe=%d end %s t=%.2f\n", arm, int(det), state_name(s.last.state), s.t);
  }
  std::printf("%d checks, %s\n", checks, fails ? "RESULT FAIL" : "RESULT PASS");
  return fails ? 1 : 0;
}
