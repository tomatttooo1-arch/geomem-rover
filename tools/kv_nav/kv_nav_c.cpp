// C ABI of kv_nav for ctypes (kv_server --nav, CTRLPS2/N2). Plain C types only; one handle per
// navigation instance; not thread-safe (call from one thread). See README.md "N2 interface".
#include <cstring>
#include <string>
#include <vector>

#include "kv_nav.hpp"

using kv_nav::NavCore;

extern "C" {

struct kv_nav_cmd {
  double v, w;
  int state, reason;
  unsigned flags;
  unsigned goal_id;
  double remaining_m;
  int recoveries, plan_status, field_source, path_points;
  double lookahead;
};

int kv_nav_abi_version() { return 3; }  // 2 (CTRLPS3): careful-mode params, kv_nav_world_to_cell; 3: kv_nav_inflate

void* kv_nav_create() { return new NavCore(kv_nav::Params()); }
void kv_nav_destroy(void* h) { delete static_cast<NavCore*>(h); }

// Set a numeric parameter by its Params field name (see kv_nav.hpp). Returns 0, or -1 if unknown.
// Call before the first goal; changing parameters mid-goal is allowed but not tested.
int kv_nav_set_param(void* h, const char* name, double value) {
  kv_nav::Params& p = static_cast<NavCore*>(h)->params();
  const std::string n(name ? name : "");
#define D(f) if (n == #f) { p.f = value; return 0; }
#define I(f) if (n == #f) { p.f = int(value); return 0; }
  D(v_max) D(w_max) D(w_rot_min) D(rot_enter) D(rot_exit) D(k_rot) D(l_min) D(l_max) D(l_gain) D(r_reg)
  D(approach) D(v_approach) D(v_min) D(xy_tol) D(yaw_tol) D(hz) I(adaptive) D(tight_lookahead)
  D(tight_window) I(tight_cells) I(tight_no_regulation) I(search_window) D(acc_v) D(acc_w)
  I(collision_check) D(horizon_s) I(horizon_steps) D(footprint_r) D(crawl_ring) D(crawl_v) I(disk_blocks_inscribed)
  D(res) D(cell_offset) D(replan_period) D(block_lookahead) D(goal_jump) D(off_path) D(switch_gain)
  D(switch_min_interval) I(smooth) I(backend) I(selfcheck_every) I(selfcheck_disable_after)
  D(stuck_window) D(stuck_dist) D(stuck_rot) D(wait_s) I(wait_max) D(detour_ratio) D(detour_range)
  D(observe_wait) D(detour_min_extra) I(detour_nonfinal_extend) D(sweep1) D(sweep2) D(sweep3) D(sweep_time_allowance) D(map_max_age)
  I(careful) D(careful_lookahead) D(careful_v) D(careful_align_enter) D(careful_align_exit) D(careful_exit)
  if (n == "pl_kmax") { p.pl_kmax = unsigned(value); return 0; }
#undef D
#undef I
  return -1;
}

void kv_nav_set_goal(void* h, unsigned id, double x_map, double y_map, double yaw_map, double t) {
  static_cast<NavCore*>(h)->set_goal(id, x_map, y_map, yaw_map, t);
}
void kv_nav_cancel(void* h, double t) { static_cast<NavCore*>(h)->cancel(t); }
void kv_nav_set_map_to_odom(void* h, double x, double y, double yaw) {
  static_cast<NavCore*>(h)->set_map_to_odom(x, y, yaw);
}
// grid: 6400 bytes, row-major [y][x], Nav2 semantics (0-252 free, 253 inscribed, 254 lethal, 255 unknown).
void kv_nav_update_costmap(void* h, const unsigned char* grid, int base_x, int base_y, double stamp) {
  static_cast<NavCore*>(h)->update_costmap(grid, base_x, base_y, stamp);
}
// xy: interleaved map-frame metres, n points (n = 0 clears the route).
void kv_nav_set_global_route(void* h, const double* xy, int n) {
  std::vector<double> x, y;
  for (int i = 0; i < n; ++i) { x.push_back(xy[2 * i]); y.push_back(xy[2 * i + 1]); }
  static_cast<NavCore*>(h)->set_global_route(x, y);
}
void kv_nav_set_pl_builder(void* h, kv_nav::PlFieldFn fn, void* user) { static_cast<NavCore*>(h)->set_pl(fn, user); }

void kv_nav_tick(void* h, double t, double x, double y, double yaw, double v, double w, int permit, int teleop,
                 kv_nav_cmd* out) {
  kv_nav::Command c = static_cast<NavCore*>(h)->tick(t, kv_nav::Pose{x, y, yaw}, v, w, permit != 0, teleop != 0);
  out->v = c.v; out->w = c.w; out->state = c.state; out->reason = c.reason; out->flags = c.flags;
  out->goal_id = c.goal_id; out->remaining_m = c.remaining_m; out->recoveries = c.recoveries;
  out->plan_status = c.plan_status; out->field_source = c.field_source; out->path_points = c.path_points;
  out->lookahead = c.lookahead;
}
// Current path, odom metres interleaved; returns the point count (copies min(count, cap)).
int kv_nav_get_path(void* h, double* xy, int cap) {
  std::vector<double> p = static_cast<NavCore*>(h)->path_odom_xy();
  const int n = int(p.size() / 2);
  std::memcpy(xy, p.data(), sizeof(double) * 2 * size_t(n < cap ? n : cap));
  return n;
}
// The core's own odom-metres -> window cell mapping (Window::qx/qy + cell, current params' res and
// cell_offset), for a costmap window at (base_x, base_y). CTRLPS3 cell_offset calibration uses exactly this.
void kv_nav_world_to_cell(void* h, double x, double y, int base_x, int base_y, int* cx, int* cy) {
  const kv_nav::Params& p = static_cast<NavCore*>(h)->params();
  kv_nav::Window win{nullptr, base_x, base_y, p.res, p.cell_offset};
  *cx = win.cell(win.qx(x)); *cy = win.cell(win.qy(y));
}
// CTRLPS3: Nav2 StaticLayer (trinary, track unknown) + InflationLayer in the exact form of the B1 PL cost plane
// (tools/b1_costmap/nav2_ref.inflate_exact): grid int8 [h][w] 100 / 0 / -1 (>= 100 lethal, -1 unknown) ->
// out uint8 Nav2 costs. table[(R+1)*|dx| + |dy|] = cached cost at that offset (0 beyond the inflation radius).
// Every cell takes the largest table value over the lethal cells within R; unknown cells take it only when it
// is >= 253 (inflate_unknown false). Returns 0, or -1 on bad sizes. The host/kv_nav.py window costs come
// from here (numpy was ~6 ms on the host per window, ~10x that on the A53).
int kv_nav_inflate(const signed char* grid, int w, int h, const unsigned char* table, int R, unsigned char* out) {
  if (w <= 0 || h <= 0 || R < 0 || R > 64) return -1;
  std::vector<unsigned char> best(size_t(w) * size_t(h), 0);
  const int T = R + 1;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      if (grid[y * w + x] < 100) continue;
      for (int dy = -R; dy <= R; ++dy) {
        const int yy = y + dy;
        if (yy < 0 || yy >= h) continue;
        for (int dx = -R; dx <= R; ++dx) {
          const int xx = x + dx;
          if (xx < 0 || xx >= w) continue;
          const unsigned char c = table[T * (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy)];
          unsigned char& b = best[size_t(yy) * size_t(w) + size_t(xx)];
          if (c > b) b = c;
        }
      }
    }
  for (int i = 0; i < w * h; ++i) {
    const signed char g = grid[i];
    const unsigned char b = best[i];
    if (g >= 100) out[i] = 254;
    else if (g == -1) out[i] = b >= 253 ? b : 255;
    else out[i] = b;
  }
  return 0;
}
double kv_nav_last_plan_ms(void* h) { return static_cast<NavCore*>(h)->last_plan_ms(); }
const char* kv_nav_state_name(int s) { return kv_nav::state_name(s); }
const char* kv_nav_reason_name(int r) { return kv_nav::reason_name(r); }
}
