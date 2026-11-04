// kv_nav core implementation. See kv_nav.hpp for the contract and README.md for the design trace.
#include "kv_nav.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "planner_split.hpp"
#include "smooth_v3.hpp"

namespace kv_nav {
namespace {
constexpr double kPi = 3.14159265358979323846;
// Python's (a + pi) % (2 pi) - pi (result in [-pi, pi)), as in the CTRLPS0 prototype.
double wrap(double a) {
  double r = std::fmod(a + kPi, 2 * kPi);
  if (r < 0) r += 2 * kPi;
  return r - kPi;
}
double sgn_copy(double mag, double s) { return std::copysign(mag, s); }
using Engine = kv_nav_planner::detail::Engine<80, 80, kv_nav_planner::kInfinity>;
}  // namespace

const char* state_name(int s) {
  static const char* n[] = {"IDLE", "PLANNING", "ROTATE_TO_PATH", "TRACK", "FINAL_ROTATE", "SUCCEEDED",
                            "RECOVER_WAIT", "RECOVER_OBSERVE", "PAUSED", "ABORTED", "CANCELED"};
  return (s >= 0 && s <= CANCELED) ? n[s] : "?";
}
const char* reason_name(int r) {
  static const char* n[] = {"NONE", "PLAN_FAILED", "BLOCKED", "STUCK", "NO_MAP", "NO_POSE", "GOAL_INVALID", "DETOUR"};
  return (r >= 0 && r <= R_DETOUR) ? n[r] : "?";
}

int Window::cell(double q) const { return int(std::floor(q + 0.5)); }

// ============================================================== planner
bool soft_field(const std::uint8_t* map, int sx, int sy, std::uint32_t* potential, unsigned* rounds) {
  std::vector<std::uint8_t> m(kCells);
  unsigned r = 0;
  // limit 0 = the reference N-round bound (6400): the software fallback has no K cap (design §2.2).
  bool ok = Engine::scan(map, sx, sy, m.data(), potential, 0, r, nullptr);
  if (rounds) *rounds = r;
  return ok;
}

WindowPlanner::WindowPlanner(const Params& p)
    : p_(p), scratch_(kCells), field_(kCells), field2_(kCells), ox_(kCells + 2), oy_(kCells + 2) {}

PlanOut WindowPlanner::plan(const std::uint8_t* map, int sx, int sy, double gx, double gy) {
  PlanOut out;
  kv_nav_planner::Request q{sx, sy, gx, gy};
  kv_nav_planner::Result chk;
  if (!Engine::check(map, q, scratch_.data(), field_.data(), ox_.data(), oy_.data(), kCells + 2, chk)) {
    out.status = chk.status;
    return out;
  }
  unsigned rounds = 0;
  bool have = false;
  if (p_.backend == BACKEND_PL && pl_fn_ && !pl_disabled_) {
    ++pl_calls_;
    const int st = pl_fn_(pl_user_, map, sx, sy, int(std::round(gx)), int(std::round(gy)), p_.pl_kmax,
                          field_.data(), &rounds);
    if (st == 0) {
      have = true;
      out.field_source = FS_PL;
      if (p_.selfcheck_every > 0 && pl_calls_ % p_.selfcheck_every == 0) {
        unsigned r2 = 0;
        const bool ok2 = soft_field(map, sx, sy, field2_.data(), &r2);
        if (!ok2 || !std::equal(field_.begin(), field_.end(), field2_.begin())) {
          out.selfcheck_mismatch = true;
          if (++mismatch_run_ >= p_.selfcheck_disable_after) pl_disabled_ = true;
          field_.swap(field2_);
          rounds = r2;
          out.field_source = FS_SOFT_SELFCHECK;
          have = ok2;
        } else {
          mismatch_run_ = 0;
        }
      }
    } else if (st == kv_nav_planner::NOT_CONVERGED) {
      out.field_source = FS_SOFT_FALLBACK;
    } else {
      out.status = st;
      out.field_source = FS_PL;
      return out;
    }
  }
  if (!have) {
    if (out.field_source == FS_NONE) out.field_source = FS_SOFT;
    if (!soft_field(map, sx, sy, field_.data(), &rounds)) {
      out.status = kv_nav_planner::NOT_CONVERGED;
      out.rounds = rounds;
      return out;
    }
  }
  kv_nav_planner::Result r = kv_nav_planner::extract_from_field(map, q, field_.data(), rounds, scratch_.data(),
                                                                ox_.data(), oy_.data(), kCells + 2);
  out.status = r.status;
  out.rounds = r.rounds;
  if (r.status != kv_nav_planner::EXACT && r.status != kv_nav_planner::WITHIN_TOLERANCE) return out;
  int n = r.count;
  out.raw_points = n;
  // scratch_ now holds the working map (prepare()), exactly what wave_plan_v3 smooths against.
  if (p_.smooth) n = kv_nav_smooth::constrained_smooth(scratch_.data(), kW, kH, ox_.data(), oy_.data(), n);
  out.x.assign(ox_.begin(), ox_.begin() + n);
  out.y.assign(oy_.begin(), oy_.begin() + n);
  return out;
}

// ============================================================== tracker
void Tracker::set_path(const std::vector<double>& x, const std::vector<double>& y,
                       const std::vector<std::uint8_t>& near, double goal_yaw, bool ends_at_goal) {
  x_ = x; y_ = y; near_ = near;
  if (near_.size() != x_.size()) near_.assign(x_.size(), 0);
  s_.assign(x_.size(), 0.0);
  for (size_t i = 1; i < x_.size(); ++i) s_[i] = s_[i - 1] + std::hypot(x_[i] - x_[i - 1], y_[i] - y_[i - 1]);
  goal_yaw_ = goal_yaw; ends_at_goal_ = ends_at_goal;
  idx_ = 0;
  done_ = false;
  // rotating_ / final_ deliberately persist across a path swap (hysteresis, latched final).
}

double Tracker::remaining(const Pose& pose) const {
  if (x_.empty()) return -1;
  const int i = std::min<int>(idx_, int(x_.size()) - 1);
  return s_.back() - s_[i] + std::hypot(x_[i] - pose.x, y_[i] - pose.y);
}

double Tracker::cross_track(const Pose& pose) const {
  if (x_.empty()) return 0;
  if (x_.size() == 1) return std::hypot(x_[0] - pose.x, y_[0] - pose.y);
  double best = 1e300;
  for (size_t i = 0; i + 1 < x_.size(); ++i) {
    const double ax = x_[i], ay = y_[i], bx = x_[i + 1] - ax, by = y_[i + 1] - ay;
    const double den = std::max(bx * bx + by * by, 1e-12);
    const double t = std::min(1.0, std::max(0.0, ((pose.x - ax) * bx + (pose.y - ay) * by) / den));
    best = std::min(best, std::hypot(ax + t * bx - pose.x, ay + t * by - pose.y));
  }
  return best;
}

// Faithful port of CTRLPS0 proto sim_rpp.py Controller.step + diag_adaptive.py Adaptive.step.
TrackOut Tracker::step(const Pose& pose, double v_now) {
  TrackOut o;
  if (x_.empty()) { o.done = done_; return o; }
  const int n = int(x_.size());
  const double gx = x_[n - 1], gy = y_[n - 1];
  const double d_goal = std::hypot(gx - pose.x, gy - pose.y);
  if (done_) { o.done = true; o.final_rotate = true; return o; }
  if (ends_at_goal_ && (final_ || d_goal <= p_.xy_tol)) {
    final_ = true;
    o.final_rotate = true;
    const double e = wrap(goal_yaw_ - pose.yaw);
    if (std::fabs(e) <= p_.yaw_tol) { done_ = true; o.done = true; return o; }
    o.w = sgn_copy(std::min(std::max(p_.k_rot * std::fabs(e), p_.w_rot_min), p_.w_max), e);
    rot_cmds.push_back(std::fabs(o.w));
    return o;
  }
  // closest point, searched forward in a bounded window (no jumping back along the path)
  auto closest = [&](int from) {
    const int hi = std::min(n, from + p_.search_window);
    int best = from; double bd = 1e300;
    for (int i = from; i < hi; ++i) {
      const double d = std::hypot(x_[i] - pose.x, y_[i] - pose.y);
      if (d < bd) { bd = d; best = i; }   // first minimum, as numpy argmin
    }
    return best;
  };
  double l_min = p_.l_min, l_max = p_.l_max, r_reg = p_.r_reg;
  if (p_.adaptive) {
    const int i0 = closest(idx_);
    const double target = s_[i0] + p_.tight_window;
    const int j = int(std::lower_bound(s_.begin(), s_.end(), target) - s_.begin());  // searchsorted left
    bool tight = false;
    for (int k = i0; k <= std::min(j, n - 1); ++k) if (near_[k]) { tight = true; break; }
    if (tight) {
      l_min = l_max = p_.tight_lookahead;
      if (p_.tight_no_regulation) r_reg = 0.0;
      o.tight = true;
    }
  }
  idx_ = closest(idx_);
  const double L = std::min(std::max(l_min + p_.l_gain * std::fabs(v_now), l_min), l_max);
  o.lookahead = L;
  const double target_s = s_[idx_] + L;
  const int j = int(std::lower_bound(s_.begin(), s_.end(), target_s) - s_.begin());
  double cx = x_[std::min(j, n - 1)], cy = y_[std::min(j, n - 1)];
  if (d_goal < L) { cx = gx; cy = gy; }
  const double dx = cx - pose.x, dy = cy - pose.y;
  const double ld = std::max(std::hypot(dx, dy), 1e-6);
  const double alpha = wrap(std::atan2(dy, dx) - pose.yaw);
  if (rotating_ && std::fabs(alpha) < p_.rot_exit) rotating_ = false;
  if (!rotating_ && std::fabs(alpha) > p_.rot_enter) rotating_ = true;
  if (rotating_) {
    o.rotating = true;
    o.w = sgn_copy(std::min(std::max(p_.k_rot * std::fabs(alpha), p_.w_rot_min), p_.w_max), alpha);
    rot_cmds.push_back(std::fabs(o.w));
    return o;
  }
  const double kappa = 2 * std::sin(alpha) / ld;
  double v = p_.v_max;
  if (std::fabs(kappa) > 1e-9 && 1 / std::fabs(kappa) < r_reg) v *= (1 / std::fabs(kappa)) / r_reg;
  if (ends_at_goal_ && d_goal < p_.approach)
    v = std::min(v, std::max(p_.v_max * d_goal / p_.approach, p_.v_approach));
  v = std::max(v, p_.v_min);
  double w = v * kappa;
  if (std::fabs(w) > p_.w_max) { v *= p_.w_max / std::fabs(w); w = sgn_copy(p_.w_max, w); }
  o.v = v; o.w = w;
  return o;
}

// ============================================================== forward footprint check
CheckResult forward_check(const Params& p, const Window& win, const Pose& pose, double v, double w) {
  if (!win.valid() || p.horizon_steps <= 0) return CHECK_OK;
  const double q0x = win.qx(pose.x), q0y = win.qy(pose.y);
  const int c0x = win.cell(q0x), c0y = win.cell(q0y);
  const double dt = p.horizon_s / p.horizon_steps;
  const double rc = p.footprint_r / win.res, rc2 = rc * rc;
  const double ring2 = (p.crawl_ring / win.res) * (p.crawl_ring / win.res);
  const int span = int(std::ceil(rc));
  double x = pose.x, y = pose.y, th = pose.yaw;
  CheckResult res = CHECK_OK;
  if (v == 0.0) return CHECK_OK;  // in-place rotation: the body does not sweep new cells (centre stays)
  for (int k = 1; k <= p.horizon_steps; ++k) {
    x += v * std::cos(th) * dt;
    y += v * std::sin(th) * dt;
    th += w * dt;
    const double qx = win.qx(x), qy = win.qy(y);
    const int cx = win.cell(qx), cy = win.cell(qy);
    if (!(cx == c0x && cy == c0y)) {
      const int c = win.at(cx, cy);
      if (c >= 253 && c != 255) return CHECK_BLOCKED;  // centre into inscribed/lethal (Nav2 circular semantics)
    }
    // body disk vs UNKNOWN (unknown is not inflated, so the disk is needed here)
    for (int yy = cy - span; yy <= cy + span; ++yy)
      for (int xx = cx - span; xx <= cx + span; ++xx) {
        const double ddx = xx - qx, ddy = yy - qy;
        if (ddx * ddx + ddy * ddy > rc2) continue;
        const int cc = win.at(xx, yy);
        if (p.disk_blocks_inscribed && cc >= 253 && cc != 255 && !(xx == c0x && yy == c0y)) return CHECK_BLOCKED;
        if (cc != 255) continue;
        const double ex = xx - q0x, ey = yy - q0y;
        if (ex * ex + ey * ey <= ring2) res = CHECK_CRAWL;
        else return CHECK_BLOCKED;
      }
  }
  return res;
}

// Planner output (cells) -> tracker path (odom metres), densified so that no segment is longer than
// one cell when it was longer than 1.5 cells (tier-2 shortcuts), so the pure-pursuit carrot and the
// closest-point search see tier-1 granularity; near[i] = a >=253 cell within tight_cells (Chebyshev).
void build_track_path(const Params& p, const Window& win, const std::vector<double>& cx, const std::vector<double>& cy,
                      std::vector<double>& X, std::vector<double>& Y, std::vector<std::uint8_t>& near) {
  X.clear(); Y.clear();
  for (size_t i = 0; i < cx.size(); ++i) {
    if (i > 0) {
      const double L = std::hypot(cx[i] - cx[i - 1], cy[i] - cy[i - 1]);
      if (L > 1.5) {
        const int k = int(std::ceil(L));
        for (int j = 1; j < k; ++j) {
          X.push_back(win.wx(cx[i - 1] + (cx[i] - cx[i - 1]) * j / k));
          Y.push_back(win.wy(cy[i - 1] + (cy[i] - cy[i - 1]) * j / k));
        }
      }
    }
    X.push_back(win.wx(cx[i])); Y.push_back(win.wy(cy[i]));
  }
  near.assign(X.size(), 0);
  for (size_t i = 0; i < X.size(); ++i) {
    const int qx = win.cell(win.qx(X[i])), qy = win.cell(win.qy(Y[i]));
    for (int yy = std::max(0, qy - p.tight_cells); yy <= std::min(kH - 1, qy + p.tight_cells) && !near[i]; ++yy)
      for (int xx = std::max(0, qx - p.tight_cells); xx <= std::min(kW - 1, qx + p.tight_cells); ++xx)
        if (win.map[yy * kW + xx] >= 253) { near[i] = 1; break; }
  }
}

void RateLimiter::apply(const Params& p, double& v, double& w) {
  // increases of |v| and |w| are limited (braking is not); an in-place rotation command never
  // lands inside (0, w_rot_min) (chassis dead band 0.045-0.105 rad/s, 10-22).
  const double dt = 1.0 / p.hz;
  const double tv = v, tw = w;
  auto lim = [&](double target, double last, double acc) {
    if (acc <= 0) return target;
    if (target * last < 0) last = 0;                         // sign change: from rest
    if (std::fabs(target) <= std::fabs(last)) return target; // braking unlimited
    const double m = std::fabs(last) + acc * dt;
    return std::fabs(target) > m ? std::copysign(m, target) : target;
  };
  if (tv != 0.0) {
    const double lv = lim(tv, last_v, p.acc_v), lw = lim(tw, last_w, p.acc_w);
    double scale = 1.0;
    if (std::fabs(lv) < std::fabs(tv)) scale = std::min(scale, std::fabs(lv) / std::fabs(tv));
    if (tw != 0.0 && std::fabs(lw) < std::fabs(tw)) scale = std::min(scale, std::fabs(lw) / std::fabs(tw));
    v = tv * scale; w = tw * scale;  // keep the curvature
  } else if (tw != 0.0) {
    w = lim(tw, last_w, p.acc_w);
    if (std::fabs(tw) >= p.w_rot_min && std::fabs(w) < p.w_rot_min) w = std::copysign(p.w_rot_min, tw);
  }
  limited = (v != tv || w != tw);
  last_v = v; last_w = w;
}

// ============================================================== state machine
NavCore::NavCore(const Params& p) : p_(p), planner_(p_), tracker_(p_), map_(kCells, 255) {}

void NavCore::enter(int s, double t) { state_ = s; state_t_ = t; }

void NavCore::reset_stuck(double t, const Pose& pose) { stuck_t_ = t; stuck_pose_ = pose; }

void NavCore::set_goal(std::uint32_t id, double x, double y, double yaw, double t) {
  has_goal_ = true; goal_id_ = id; goal_mx_ = x; goal_my_ = y; goal_myaw_ = yaw;
  reason_ = R_NONE; recoveries_ = 0; waits_ = 0; observed_ = false; sweep_stage_ = 0;
  tracker_.clear();
  need_plan_ = true; last_switch_t_ = -1e18; last_plan_t_ = -1e18; prev_remaining_ = -1;
  limiter_.reset();
  careful_ = false; careful_rot_ = false;
  enter(PLANNING, t);
  stuck_t_ = t; stuck_pose_ = Pose(); stuck_t_ = -1;  // re-anchored on the first active tick
}

void NavCore::cancel(double t) {
  if (!has_goal_) return;
  if (state_ != SUCCEEDED && state_ != ABORTED) enter(CANCELED, t);
}

void NavCore::set_map_to_odom(double x, double y, double yaw) { m2o_x_ = x; m2o_y_ = y; m2o_yaw_ = yaw; }

void NavCore::update_costmap(const std::uint8_t* map, int bx, int by, double stamp) {
  std::copy(map, map + kCells, map_.begin());
  base_x_ = bx; base_y_ = by; map_stamp_ = stamp; has_map_ = true;
}

void NavCore::set_global_route(const std::vector<double>& x, const std::vector<double>& y) {
  route_x_ = x; route_y_ = y;
  if (route_y_.size() != route_x_.size()) route_x_.clear(), route_y_.clear();
}

std::vector<double> NavCore::path_odom_xy() const {
  std::vector<double> o;
  for (size_t i = 0; i < tracker_.px().size(); ++i) { o.push_back(tracker_.px()[i]); o.push_back(tracker_.py()[i]); }
  return o;
}

// Window goal in continuous cell coordinates. Inside [1,78]^2 -> the goal itself (final).
// Outside -> first exit of the global route (or, without a route, of the straight line car->goal)
// from [1,78]^2, walked back toward the car until the cell is < 253 (design §2.2).
bool NavCore::window_goal(const Pose& pose, double& gx, double& gy, bool& is_final) const {
  Window win{map_.data(), base_x_, base_y_, p_.res, p_.cell_offset};
  const double c = std::cos(m2o_yaw_), s = std::sin(m2o_yaw_);
  auto to_odom = [&](double mx, double my, double& ox, double& oy) {
    ox = c * mx - s * my + m2o_x_; oy = s * mx + c * my + m2o_y_;
  };
  double ox, oy; to_odom(goal_mx_, goal_my_, ox, oy);
  const double qx = win.qx(ox), qy = win.qy(oy);
  auto in = [](double a, double b) { return a >= 1.0 && a <= 78.0 && b >= 1.0 && b <= 78.0; };
  if (in(qx, qy)) { gx = qx; gy = qy; is_final = true; return true; }
  is_final = false;
  std::vector<double> px, py;
  px.push_back(win.qx(pose.x)); py.push_back(win.qy(pose.y));
  if (!route_x_.empty()) {
    // start from the route point nearest the car, then follow the route
    size_t k0 = 0; double bd = 1e300;
    for (size_t k = 0; k < route_x_.size(); ++k) {
      double rx, ry; to_odom(route_x_[k], route_y_[k], rx, ry);
      const double d = std::hypot(rx - pose.x, ry - pose.y);
      if (d < bd) { bd = d; k0 = k; }
    }
    for (size_t k = k0; k < route_x_.size(); ++k) {
      double rx, ry; to_odom(route_x_[k], route_y_[k], rx, ry);
      px.push_back(win.qx(rx)); py.push_back(win.qy(ry));
    }
  }
  px.push_back(qx); py.push_back(qy);
  if (!in(px[0], py[0])) return false;
  for (size_t k = 1; k < px.size(); ++k) {
    if (in(px[k], py[k])) continue;
    const double ax = px[k - 1], ay = py[k - 1], dx = px[k] - ax, dy = py[k] - ay;
    double u = 1.0;
    if (dx > 0) u = std::min(u, (78.0 - ax) / dx);
    if (dx < 0) u = std::min(u, (1.0 - ax) / dx);
    if (dy > 0) u = std::min(u, (78.0 - ay) / dy);
    if (dy < 0) u = std::min(u, (1.0 - ay) / dy);
    u = std::max(0.0, u);
    double ex = ax + u * dx, ey = ay + u * dy;
    // walk back along the polyline (toward the car) in half-cell steps until free
    std::vector<double> bx(px.begin(), px.begin() + k), by(py.begin(), py.begin() + k);
    bx.push_back(ex); by.push_back(ey);
    for (int seg = int(bx.size()) - 1; seg >= 1; --seg) {
      const double sx0 = bx[seg], sy0 = by[seg], sx1 = bx[seg - 1], sy1 = by[seg - 1];
      const double L = std::hypot(sx1 - sx0, sy1 - sy0);
      const int steps = std::max(1, int(std::ceil(L / 0.5)));
      for (int t = 0; t <= steps; ++t) {
        const double fx = sx0 + (sx1 - sx0) * t / steps, fy = sy0 + (sy1 - sy0) * t / steps;
        if (win.at(win.cell(fx), win.cell(fy)) < 253) { gx = fx; gy = fy; return true; }
      }
    }
    return false;
  }
  return false;
}

bool NavCore::path_blocked_ahead(const Pose& pose) const {
  if (!tracker_.has_path() || !has_map_) return false;
  Window win{map_.data(), base_x_, base_y_, p_.res, p_.cell_offset};
  const int c0x = win.cell(win.qx(pose.x)), c0y = win.cell(win.qy(pose.y));
  const auto& X = tracker_.px(); const auto& Y = tracker_.py();
  double s = 0;
  for (size_t i = std::max(0, tracker_.index()); i < X.size(); ++i) {
    if (i > size_t(tracker_.index())) s += std::hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]);
    if (s > p_.block_lookahead) break;
    const int cx = win.cell(win.qx(X[i])), cy = win.cell(win.qy(Y[i]));
    if (cx == c0x && cy == c0y) continue;
    if (win.at(cx, cy) >= 253) return true;
  }
  return false;
}

bool NavCore::replan(double t, const Pose& pose, Command& c, bool force_adopt) {
  last_plan_t_ = t; need_plan_ = false;
  c.flags |= F_REPLANNED;
  if (!has_map_) { last_plan_status_ = -1; return false; }
  Window win{map_.data(), base_x_, base_y_, p_.res, p_.cell_offset};
  const int sx = win.cell(win.qx(pose.x)), sy = win.cell(win.qy(pose.y));
  double gx, gy; bool fin;
  if (sx < 1 || sy < 1 || sx > 78 || sy > 78 || !window_goal(pose, gx, gy, fin)) {
    last_plan_status_ = kv_nav_planner::INVALID_ARGUMENTS;
    return false;
  }
  const auto t0 = std::chrono::steady_clock::now();
  PlanOut out = planner_.plan(map_.data(), sx, sy, gx, gy);
  last_plan_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  last_plan_status_ = out.status; last_field_source_ = out.field_source;
  if (out.selfcheck_mismatch) c.flags |= F_SELFCHECK_MISMATCH;
  if (planner_.pl_disabled()) c.flags |= F_PL_DISABLED;
  map_used_stamp_ = map_stamp_;
  if (out.status != kv_nav_planner::EXACT && out.status != kv_nav_planner::WITHIN_TOLERANCE) return false;
  std::vector<double> X, Y;
  std::vector<std::uint8_t> near;
  build_track_path(p_, win, out.x, out.y, X, Y, near);
  double new_len = 0;
  for (size_t i = 1; i < X.size(); ++i) new_len += std::hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]);
  const double old_rem = tracker_.has_path() ? tracker_.remaining(pose) : -1;
  const double gox = win.wx(gx), goy = win.wy(gy);
  const bool goal_moved = std::hypot(gox - planned_goal_ox_, goy - planned_goal_oy_) > p_.goal_jump;
  const bool adopt = force_adopt || !tracker_.has_path() || goal_moved || fin != path_is_final_ ||
                     path_blocked_ahead(pose) || new_len < p_.switch_gain * old_rem ||
                     t - last_switch_t_ >= p_.switch_min_interval;
  if (!adopt) return true;
  // the old path's own end: a window-exit point when it was not final (CTRLPS5)
  const bool old_final = path_is_final_;
  const double old_end_x = planned_goal_ox_, old_end_y = planned_goal_oy_;
  tracker_.set_path(X, Y, near, wrap(goal_myaw_ + m2o_yaw_), fin);
  last_switch_t_ = t; planned_goal_ox_ = gox; planned_goal_oy_ = goy; path_is_final_ = fin;
  c.flags |= F_PATH_SWITCHED;
  // "observe before a detour": new path > 1.5x the old remaining, goal within 3 m, once per goal.
  // CTRLPS5: when the goal has just come into the window, the old path ended at the window exit, short of the
  // goal; its remaining alone is not the way still to go. With detour_nonfinal_extend the straight distance from
  // that exit point to the goal is added (a lower bound of the old plan's rest), so only a real lengthening trips.
  double old_total = old_rem;
  if (p_.detour_nonfinal_extend && old_rem > 0 && fin && !old_final)
    old_total = old_rem + std::hypot(gox - old_end_x, goy - old_end_y);
  const double d_goal = std::hypot(gox - pose.x, goy - pose.y);
  if (old_rem > 0 && new_len > p_.detour_ratio * old_total && new_len - old_total >= p_.detour_min_extra && fin && d_goal <= p_.detour_range && !observed_) {
    observed_ = true; ++recoveries_; reason_ = R_DETOUR; sweep_stage_ = 0;
    enter(RECOVER_OBSERVE, t);
  }
  return true;
}

Command NavCore::finish(Command c, double t, const Pose&) {
  limiter_.apply(p_, c.v, c.w);
  if (limiter_.limited) c.flags |= F_RATE_LIMITED;
  c.state = state_; c.reason = reason_; c.goal_id = goal_id_; c.recoveries = recoveries_;
  c.plan_status = last_plan_status_; c.field_source = last_field_source_;
  c.path_points = int(tracker_.px().size());
  (void)t;
  return c;
}

// Careful mode command: carrot = first path point at least careful_lookahead (arc) past the closest point;
// |heading error| beyond the hysteresis band -> turn in place (never blocked: the centre stays); else go
// straight (w = 0) at a speed whose 1 s projection ends about at the carrot, slowest v_min; every candidate
// through the forward check (CRAWL caps the speed). BLOCKED only when even the slowest straight step is.
CheckResult NavCore::careful_cmd(const Window& win, const Pose& pose, double& v, double& w, bool& rotating) {
  v = w = 0; rotating = false;
  const auto& X = tracker_.px(); const auto& Y = tracker_.py();
  if (X.empty()) return CHECK_BLOCKED;
  const int n = int(X.size());
  // projection of the car onto the path polyline (segments around the tracker's index, a little
  // backwards too: the car may be beside a vertex the tracker already passed), then the carrot
  // careful_lookahead further along the polyline (interpolated, so it is exactly one cell on).
  const int i0 = std::min(std::max(0, tracker_.index()), n - 1);
  const int lo = std::max(0, std::min(i0 - 3, n - 2)), hi = std::min(n - 1, i0 + p_.search_window);
  int seg = lo; double best = 1e300, tseg = 0;
  for (int k = lo; k < std::max(hi, lo + 1) && k + 1 < n; ++k) {
    const double ax = X[k], ay = Y[k], bx = X[k + 1] - ax, by = Y[k + 1] - ay;
    const double L2 = std::max(bx * bx + by * by, 1e-12);
    const double u = std::min(1.0, std::max(0.0, ((pose.x - ax) * bx + (pose.y - ay) * by) / L2));
    const double dd = std::hypot(ax + u * bx - pose.x, ay + u * by - pose.y);
    if (dd < best) { best = dd; seg = k; tseg = u; }
  }
  double tx = X[n - 1], ty = Y[n - 1];
  if (n >= 2) {
    double rem = p_.careful_lookahead;
    double px = X[seg] + tseg * (X[seg + 1] - X[seg]), py = Y[seg] + tseg * (Y[seg + 1] - Y[seg]);
    int k = seg;
    for (;;) {
      const double L = std::hypot(X[k + 1] - px, Y[k + 1] - py);
      if (L >= rem) { const double f = L > 1e-12 ? rem / L : 0; tx = px + f * (X[k + 1] - px); ty = py + f * (Y[k + 1] - py); break; }
      rem -= L; px = X[k + 1]; py = Y[k + 1];
      if (++k + 1 >= n) { tx = X[n - 1]; ty = Y[n - 1]; break; }
    }
  }
  const double dx = tx - pose.x, dy = ty - pose.y, d = std::hypot(dx, dy);
  const double e = d > 1e-6 ? wrap(std::atan2(dy, dx) - pose.yaw) : 0.0;
  if (careful_rot_ ? std::fabs(e) > p_.careful_align_exit : std::fabs(e) > p_.careful_align_enter) {
    careful_rot_ = true; rotating = true;
    w = std::copysign(std::min(std::max(p_.k_rot * std::fabs(e), p_.w_rot_min), p_.w_max), e);
    return CHECK_OK;
  }
  careful_rot_ = false;
  const double v1 = std::min(p_.careful_v, std::max(d / std::max(p_.horizon_s, 1e-3), p_.v_min));
  for (double vv : {v1, p_.v_min}) {
    if (!p_.collision_check) { v = vv; return CHECK_OK; }
    const CheckResult cr = forward_check(p_, win, pose, vv, 0.0);
    if (cr == CHECK_BLOCKED) continue;
    v = cr == CHECK_CRAWL ? std::min(vv, p_.crawl_v) : vv;
    return cr;
  }
  return CHECK_BLOCKED;
}

Command NavCore::tick(double t, const Pose& pose, double v_meas, double, bool permit, bool teleop) {
  Command c;
  auto stop = [&](Command cc) { cc.v = 0; cc.w = 0; limiter_.reset();
    cc.state = state_; cc.reason = reason_; cc.goal_id = goal_id_; cc.recoveries = recoveries_;
    cc.plan_status = last_plan_status_; cc.field_source = last_field_source_;
    cc.path_points = int(tracker_.px().size()); return cc; };
  if (!has_goal_ || state_ == IDLE || state_ == SUCCEEDED || state_ == ABORTED || state_ == CANCELED) return stop(c);
  c.remaining_m = tracker_.remaining(pose);
  // ---- motion permit / teleop: hold still, do not count stuck time, replan on release
  if (!permit || teleop) {
    if (state_ != PAUSED) enter(PAUSED, t);
    return stop(c);
  }
  if (state_ == PAUSED) {
    tracker_.reset_modes(); need_plan_ = true; stuck_t_ = -1; careful_ = false; careful_rot_ = false;
    enter(PLANNING, t);
  }
  if (stuck_t_ < 0) reset_stuck(t, pose);
  auto start_recovery = [&](int why) {
    careful_ = false; careful_rot_ = false;
    ++recoveries_; reason_ = why;
    if (waits_ < p_.wait_max) { ++waits_; enter(RECOVER_WAIT, t); }
    else if (!observed_) { observed_ = true; sweep_stage_ = 0; enter(RECOVER_OBSERVE, t); }
    else { enter(ABORTED, t); }
    stuck_t_ = -1;
  };
  const bool map_fresh = has_map_ && (t - map_stamp_) <= p_.map_max_age;
  // ---- recovery states
  if (state_ == RECOVER_WAIT) {
    if (t - state_t_ >= p_.wait_s && map_fresh && map_stamp_ > state_t_ - 1e-9) {
      if (replan(t, pose, c, true)) { tracker_.reset_modes(); enter(TRACK, t); reset_stuck(t, pose); }
      else start_recovery(reason_ == R_NONE ? R_PLAN_FAILED : reason_);
    }
    return stop(c);
  }
  if (state_ == RECOVER_OBSERVE) {
    if (sweep_stage_ == 0) {
      if (t - state_t_ < p_.observe_wait) return stop(c);
      sweep_stage_ = 1; sweep_target_ = wrap(pose.yaw + p_.sweep1); sweep_t_ = t;
    }
    if (sweep_stage_ >= 1 && sweep_stage_ <= 3) {
      const double e = wrap(sweep_target_ - pose.yaw);
      if (std::fabs(e) > p_.yaw_tol && t - sweep_t_ < p_.sweep_time_allowance) {
        c.w = std::copysign(std::min(std::max(p_.k_rot * std::fabs(e), p_.w_rot_min), p_.w_max), e);
        return finish(c, t, pose);
      }
      ++sweep_stage_; sweep_t_ = t;
      if (sweep_stage_ == 2) sweep_target_ = wrap(pose.yaw + p_.sweep2);
      if (sweep_stage_ == 3) sweep_target_ = wrap(pose.yaw + p_.sweep3);
      if (sweep_stage_ <= 3) return stop(c);
    }
    // sweep done: replan
    if (map_fresh && replan(t, pose, c, true)) { tracker_.reset_modes(); enter(TRACK, t); reset_stuck(t, pose); }
    else { reason_ = (reason_ == R_DETOUR || reason_ == R_NONE) ? R_PLAN_FAILED : reason_; enter(ABORTED, t); }
    return stop(c);
  }
  // ---- active: PLANNING / ROTATE_TO_PATH / TRACK / FINAL_ROTATE
  if (!map_fresh) {  // no (fresh) costmap: hold, keep the stuck clock still
    if (!has_map_) reason_ = R_NO_MAP;
    stuck_t_ = -1;
    if (state_ != FINAL_ROTATE) enter(PLANNING, state_ == PLANNING ? state_t_ : t);
    return stop(c);
  }
  Window win{map_.data(), base_x_, base_y_, p_.res, p_.cell_offset};
  bool force = false;
  if (state_ != FINAL_ROTATE) {
    if (!tracker_.has_path() || t - last_plan_t_ >= p_.replan_period) need_plan_ = true;
    if (map_stamp_ > map_used_stamp_ && path_blocked_ahead(pose)) { need_plan_ = true; force = true; }
    if (tracker_.has_path() && tracker_.cross_track(pose) > p_.off_path) need_plan_ = true;
    if (tracker_.has_path()) {  // window goal drift (map->odom correction or window move)
      double gx, gy; bool fin;
      if (window_goal(pose, gx, gy, fin) &&
          std::hypot(win.wx(gx) - planned_goal_ox_, win.wy(gy) - planned_goal_oy_) > p_.goal_jump) need_plan_ = true;
    }
    if (need_plan_) {
      const bool ok = replan(t, pose, c, force);
      if (state_ == RECOVER_OBSERVE) return stop(c);  // detour observe triggered
      if (!ok && (!tracker_.has_path() || path_blocked_ahead(pose))) {
        start_recovery(R_PLAN_FAILED);
        return stop(c);
      }
    }
  }
  if (!tracker_.has_path()) { start_recovery(R_PLAN_FAILED); return stop(c); }
  TrackOut o = tracker_.step(pose, v_meas);
  if (o.tight) c.flags |= F_TIGHT;
  c.lookahead = o.lookahead;
  if (o.done) { enter(SUCCEEDED, t); reason_ = R_NONE; return stop(c); }
  // end of a non-final (window exit) path reached: stop and replan next tick
  if (!o.final_rotate && !o.rotating && o.v == 0 && o.w == 0) { need_plan_ = true; return stop(c); }
  if (careful_) {  // "turn first, then go" (CTRLPS3): accumulate travel, leave once the tracker's own command passes
    careful_travel_ += std::hypot(pose.x - careful_last_.x, pose.y - careful_last_.y);
    careful_last_ = pose;
    const bool normal_ok = o.final_rotate || !p_.collision_check || o.v == 0 ||
                           forward_check(p_, win, pose, o.v, o.w) != CHECK_BLOCKED;
    if (o.final_rotate || (normal_ok && careful_travel_ >= p_.careful_exit)) { careful_ = false; careful_rot_ = false; }
  }
  if (careful_) {
    c.flags |= F_CAREFUL;
    double v = 0, w = 0; bool rot = false;
    if (careful_cmd(win, pose, v, w, rot) == CHECK_BLOCKED) {
      c.flags |= F_BLOCKED;
      start_recovery(R_BLOCKED);
      return stop(c);
    }
    o.v = v; o.w = w; o.rotating = rot; o.final_rotate = false;
  } else if (p_.collision_check && o.v != 0) {
    CheckResult cr = forward_check(p_, win, pose, o.v, o.w);
    if (cr == CHECK_BLOCKED) {
      c.flags |= F_BLOCKED;
      // replan once right now; if the fresh path's command is still blocked: turn first, then go
      // (careful mode, CTRLPS3); recovery (stop + wait) only when no path at all or that is blocked too
      bool ok = replan(t, pose, c, true);
      if (state_ == RECOVER_OBSERVE) return stop(c);
      if (ok) { o = tracker_.step(pose, v_meas); cr = o.v != 0 ? forward_check(p_, win, pose, o.v, o.w) : CHECK_OK; }
      if (ok && cr == CHECK_BLOCKED && p_.careful) {
        careful_ = true; careful_rot_ = false; careful_travel_ = 0; careful_last_ = pose;
        c.flags |= F_CAREFUL;
        double v = 0, w = 0; bool rot = false;
        cr = careful_cmd(win, pose, v, w, rot);
        if (cr != CHECK_BLOCKED) { o.v = v; o.w = w; o.rotating = rot; o.final_rotate = false; }
      }
      if (!ok || cr == CHECK_BLOCKED) { start_recovery(R_BLOCKED); return stop(c); }
    }
    if (cr == CHECK_CRAWL && std::fabs(o.v) > p_.crawl_v) {
      const double k = p_.crawl_v / std::fabs(o.v);
      o.v *= k; o.w *= k;
      c.flags |= F_CRAWL;
    }
  }
  enter(o.final_rotate ? FINAL_ROTATE : o.rotating ? ROTATE_TO_PATH : TRACK, state_ == (o.final_rotate ? FINAL_ROTATE : o.rotating ? ROTATE_TO_PATH : TRACK) ? state_t_ : t);
  // stuck: PoseProgressChecker semantics (anchor resets once moved >= dist or turned >= rot)
  if (std::hypot(pose.x - stuck_pose_.x, pose.y - stuck_pose_.y) >= p_.stuck_dist ||
      std::fabs(wrap(pose.yaw - stuck_pose_.yaw)) >= p_.stuck_rot) reset_stuck(t, pose);
  else if (t - stuck_t_ >= p_.stuck_window) { start_recovery(R_STUCK); return stop(c); }
  c.v = o.v; c.w = o.w;
  c.remaining_m = tracker_.remaining(pose);
  return finish(c, t, pose);
}

}  // namespace kv_nav
