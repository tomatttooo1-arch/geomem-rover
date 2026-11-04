#pragma once
// extraction + v3 tier-2 smoothing), rotate-first + clearance-adaptive regulated pure pursuit,
// 20-pose forward footprint check (unknown blocks too, cold-start crawl ring), and the
// goal / arrive / stuck / recover / pause state machine that replaces the Nav2 behaviour tree.
// No ROS, no I/O, no threads, no clock: the caller (kv_server --nav, CTRLPS2/N2) feeds time,
// odometry, costmap windows and goals, and sends out the command of each tick.
// C++14, standard library only; builds for x86-64 and aarch64 (Makefile).
#include <cstdint>
#include <string>
#include <vector>

namespace kv_nav {

constexpr int kW = 80, kH = 80, kCells = kW * kH;

enum State : int {
  IDLE = 0, PLANNING = 1, ROTATE_TO_PATH = 2, TRACK = 3, FINAL_ROTATE = 4, SUCCEEDED = 5,
  RECOVER_WAIT = 6, RECOVER_OBSERVE = 7, PAUSED = 8, ABORTED = 9, CANCELED = 10,
};
enum Reason : int {
  R_NONE = 0, R_PLAN_FAILED = 1, R_BLOCKED = 2, R_STUCK = 3, R_NO_MAP = 4, R_NO_POSE = 5,
  R_GOAL_INVALID = 6, R_DETOUR = 7,
};
enum FieldSource : int { FS_NONE = 0, FS_SOFT = 1, FS_PL = 2, FS_SOFT_FALLBACK = 3, FS_SOFT_SELFCHECK = 4 };
enum Backend : int { BACKEND_SOFT = 0, BACKEND_PL = 1 };
// tick flags (Command::flags)
enum : unsigned {
  F_REPLANNED = 1u, F_CRAWL = 2u, F_BLOCKED = 4u, F_PATH_SWITCHED = 8u, F_SELFCHECK_MISMATCH = 16u,
  F_PL_DISABLED = 32u, F_TIGHT = 64u, F_RATE_LIMITED = 128u, F_CAREFUL = 256u,
};

struct Params {
  // --- tracker (design §2.4; defaults = CTRLPS0 prototype sim_rpp.py P + diag_adaptive.py) ---
  double v_max = 0.15, w_max = 0.4, w_rot_min = 0.12, rot_enter = 0.785, rot_exit = 0.35, k_rot = 1.0;
  double l_min = 0.30, l_max = 0.45, l_gain = 1.0, r_reg = 0.9;
  double approach = 0.6, v_approach = 0.05, v_min = 0.03;
  double xy_tol = 0.10, yaw_tol = 0.10, hz = 20.0;
  int adaptive = 1;               // clearance-adaptive lookahead (diag_adaptive.py)
  double tight_lookahead = 0.15;  // lookahead when tight
  double tight_window = 0.6;      // look this far ahead along the path for "near inscribed"
  int tight_cells = 2;            // path cell has a >=253 cell within this Chebyshev radius
  int tight_no_regulation = 1;    // tight: no curvature slowdown (what diag_adaptive.py ran)
  int search_window = 40;         // closest-point forward search (path points)
  // per-tick command rate limit (design: replaces velocity_smoother); 0 disables
  double acc_v = 0.3, acc_w = 0.6;
  // --- forward footprint check (design §2.4) ---
  int collision_check = 1;
  double horizon_s = 1.0; int horizon_steps = 20;
  double footprint_r = 0.30;        // body disk for UNKNOWN overlap (unknown is not inflated)
  double crawl_ring = 0.65;         // unknown within this distance of the car centre -> crawl
  double crawl_v = 0.03;
  int disk_blocks_inscribed = 0;  // 1 = literal design text (body disk vs 253/254 blocks); diagnostic only
  // --- planning (design §2.2) ---
  double res = 0.05;
  double cell_offset = 0.5;  // continuous cell index q = p/res - base - cell_offset (0.5: B1 voxel
                             // lattice, cell i covers [(base+i)res,(base+i+1)res); 0.0: NavFn round)
  double replan_period = 0.5;
  double block_lookahead = 0.6;  // new costmap blocks the current path within this distance -> replan
  double goal_jump = 0.3;        // window goal moved more than this (odom) -> replan
  double off_path = 0.15;        // car further than this from the path -> replan
  double switch_gain = 0.9;      // adopt a new path if shorter than gain * remaining old
  double switch_min_interval = 1.0;
  int smooth = 1;
  int backend = BACKEND_SOFT;
  unsigned pl_kmax = 64;
  int selfcheck_every = 20;      // every Nth PL field is recomputed in software and compared
  int selfcheck_disable_after = 3;
  // --- behaviour (design §2.6) ---
  double stuck_window = 20.0, stuck_dist = 0.05, stuck_rot = 0.5;
  double wait_s = 1.0; int wait_max = 2;
  double detour_ratio = 1.5, detour_range = 3.0, observe_wait = 5.0;
  double detour_min_extra = 0.3;  // and at least this much longer (m): no 'detour' from goal-side jitter
  int detour_nonfinal_extend = 1;  // CTRLPS5: old path ended at a window exit -> compare against its remaining PLUS
                                   // the straight exit->goal distance (0 = CTRLPS1: the exit remaining alone, which
                                   // trips 'detour' at random when the goal enters the window, see CTRLPS5/README)
  double sweep1 = 0.45, sweep2 = -0.9, sweep3 = 0.45;
  double sweep_time_allowance = 10.0;  // per sweep step (Nav2 Spin default); then go on
  double map_max_age = 1.0;  // costmap older than this (vs tick time) is not planned on
  // --- blocked -> "turn first, then go" (CTRLPS3, controller ruling 3 / T5c). When the forward check
  // blocks the tracker's command and an immediate replan is blocked too, the core does NOT stop: it
  // follows the path at cell scale -- turn in place toward a carrot careful_lookahead ahead on the path,
  // then go straight (w = 0) at a speed whose 1 s projection ends near the carrot, every command still
  // through the forward check. Only when even that is blocked does the recovery chain start.
  int careful = 1;                 // 0 = CTRLPS1 behaviour (blocked twice -> recovery at once)
  double careful_lookahead = 0.05; // carrot distance along the path (one cell)
  double careful_v = 0.10;         // speed cap while going straight in this mode
  double careful_align_enter = 0.20, careful_align_exit = 0.05;  // rad: start / stop turning in place
  double careful_exit = 0.25;      // leave the mode after this much travel once the tracker's own command passes
};

struct Pose { double x = 0, y = 0, yaw = 0; };

struct Command {
  double v = 0, w = 0;
  int state = IDLE, reason = R_NONE;
  unsigned flags = 0;
  std::uint32_t goal_id = 0;
  double remaining_m = -1;   // remaining path length, -1 if none
  int recoveries = 0;
  int plan_status = -1;
  int field_source = FS_NONE;
  double lookahead = 0;
  int path_points = 0;
};

// build_field() semantics: 0 = field ready (complete), 7 = NOT_CONVERGED, other = invalid.
// potential: 6400 x uint32 (24-bit values, 0xffffff = INF). rounds: rounds used.
typedef int (*PlFieldFn)(void* user, const std::uint8_t* map, int sx, int sy, int gx, int gy,
                         unsigned kmax, std::uint32_t* potential, unsigned* rounds);

// ---------------------------------------------------------------- planner (window, cell units)
struct PlanOut {
  int status = -1;
  int field_source = FS_NONE;
  unsigned rounds = 0;
  bool selfcheck_mismatch = false;
  std::vector<double> x, y;   // cell-centre coordinates (tier 1 raw, or tier 2 if smoothed)
  int raw_points = 0;
};

class WindowPlanner {
 public:
  explicit WindowPlanner(const Params& p);
  void set_pl(PlFieldFn fn, void* user) { pl_fn_ = fn; pl_user_ = user; }
  // map: 80x80 inflated costmap (0-252 free, 253 inscribed, 254 lethal, 255 unknown).
  PlanOut plan(const std::uint8_t* map, int sx, int sy, double gx, double gy);
  bool pl_disabled() const { return pl_disabled_; }
  int pl_calls() const { return pl_calls_; }
 private:
  const Params& p_;
  PlFieldFn pl_fn_ = nullptr; void* pl_user_ = nullptr;
  int pl_calls_ = 0, mismatch_run_ = 0; bool pl_disabled_ = false;
  std::vector<std::uint8_t> scratch_;
  std::vector<std::uint32_t> field_, field2_;
  std::vector<double> ox_, oy_;
};

// Pure geometry helpers exposed for tests / sim.
bool soft_field(const std::uint8_t* map, int sx, int sy, std::uint32_t* potential, unsigned* rounds);

// ---------------------------------------------------------------- tracker (odom metres)
struct TrackOut { double v = 0, w = 0; bool rotating = false, final_rotate = false, done = false, tight = false; double lookahead = 0; };

class Tracker {
 public:
  explicit Tracker(const Params& p) : p_(p) {}
  // path in odom metres; near[i]: path point i has a >=253 cell within tight_cells (adaptive).
  void set_path(const std::vector<double>& x, const std::vector<double>& y, const std::vector<std::uint8_t>& near,
                double goal_yaw, bool path_ends_at_goal);
  void reset_modes() { rotating_ = false; final_ = false; done_ = false; }
  void clear() { x_.clear(); y_.clear(); s_.clear(); near_.clear(); idx_ = 0; reset_modes(); rot_cmds.clear(); }
  TrackOut step(const Pose& pose, double v_now);
  bool has_path() const { return !x_.empty(); }
  double remaining(const Pose& pose) const;
  double cross_track(const Pose& pose) const;
  int index() const { return idx_; }
  const std::vector<double>& px() const { return x_; }
  const std::vector<double>& py() const { return y_; }
  std::vector<double> rot_cmds;  // |w| sent while rotating (test/sim evidence)
 private:
  const Params& p_;
  std::vector<double> x_, y_, s_;
  std::vector<std::uint8_t> near_;
  double goal_yaw_ = 0; bool ends_at_goal_ = true;
  int idx_ = 0; bool rotating_ = false, final_ = false, done_ = false;
};

// ---------------------------------------------------------------- forward footprint check
enum CheckResult { CHECK_OK = 0, CHECK_CRAWL = 1, CHECK_BLOCKED = 2 };
struct Window {
  const std::uint8_t* map = nullptr;  // 80x80
  int base_x = 0, base_y = 0;
  double res = 0.05, cell_offset = 0.5;
  bool valid() const { return map != nullptr; }
  // odom metres -> continuous cell index (planner coordinates)
  double qx(double x) const { return x / res - base_x - cell_offset; }
  double qy(double y) const { return y / res - base_y - cell_offset; }
  double wx(double q) const { return (q + base_x + cell_offset) * res; }
  double wy(double q) const { return (q + base_y + cell_offset) * res; }
  int cell(double q) const;
  int at(int cx, int cy) const { return (cx < 0 || cy < 0 || cx >= kW || cy >= kH) ? 255 : map[cy * kW + cx]; }
};
CheckResult forward_check(const Params& p, const Window& win, const Pose& pose, double v, double w);
void build_track_path(const Params& p, const Window& win, const std::vector<double>& cell_x,
                      const std::vector<double>& cell_y, std::vector<double>& X, std::vector<double>& Y,
                      std::vector<std::uint8_t>& near);
struct RateLimiter {
  double last_v = 0, last_w = 0; bool limited = false;
  void reset() { last_v = last_w = 0; limited = false; }
  void apply(const Params& p, double& v, double& w);
};

// ---------------------------------------------------------------- state machine
class NavCore {
 public:
  explicit NavCore(const Params& p = Params());
  Params& params() { return p_; }
  void set_pl(PlFieldFn fn, void* user) { planner_.set_pl(fn, user); }
  // Goal in MAP frame; converted to odom with the latest map->odom every tick.
  void set_goal(std::uint32_t id, double x_map, double y_map, double yaw_map, double t);
  void cancel(double t);
  void set_map_to_odom(double x, double y, double yaw);
  // Window costmap (odom lattice base, cells), stamp in the tick clock.
  void update_costmap(const std::uint8_t* map, int base_x, int base_y, double stamp);
  // Global route (MAP frame metres), e.g. NavFn path from the Nano (G1). Empty = none.
  void set_global_route(const std::vector<double>& x_map, const std::vector<double>& y_map);
  Command tick(double t, const Pose& odom_pose, double v_meas, double w_meas, bool permit, bool teleop);
  // introspection
  int state() const { return state_; }
  const Tracker& tracker() const { return tracker_; }
  std::vector<double> path_odom_xy() const;  // interleaved x,y
  double last_plan_ms() const { return last_plan_ms_; }
 private:
  bool window_goal(const Pose& pose, double& gx, double& gy, bool& is_final) const;
  bool replan(double t, const Pose& pose, Command& c, bool force_adopt);
  bool path_blocked_ahead(const Pose& pose) const;
  void enter(int s, double t);
  void reset_stuck(double t, const Pose& pose);
  Command finish(Command c, double t, const Pose& pose);
  CheckResult careful_cmd(const Window& win, const Pose& pose, double& v, double& w, bool& rotating);

  Params p_;
  WindowPlanner planner_;
  Tracker tracker_;
  // inputs
  bool has_goal_ = false; std::uint32_t goal_id_ = 0;
  double goal_mx_ = 0, goal_my_ = 0, goal_myaw_ = 0;
  double m2o_x_ = 0, m2o_y_ = 0, m2o_yaw_ = 0;
  std::vector<std::uint8_t> map_; int base_x_ = 0, base_y_ = 0; double map_stamp_ = -1e18; bool has_map_ = false;
  double map_used_stamp_ = -1e18;
  std::vector<double> route_x_, route_y_;
  // state
  int state_ = IDLE, reason_ = R_NONE;
  double state_t_ = 0;
  double last_plan_t_ = -1e18, last_switch_t_ = -1e18;
  double planned_goal_ox_ = 0, planned_goal_oy_ = 0; bool path_is_final_ = false;
  bool need_plan_ = true;
  int recoveries_ = 0, waits_ = 0; bool observed_ = false;
  int sweep_stage_ = 0; double sweep_target_ = 0, sweep_t_ = 0;
  double stuck_t_ = 0; Pose stuck_pose_;
  RateLimiter limiter_;
  int last_plan_status_ = -1, last_field_source_ = FS_NONE;
  double last_plan_ms_ = 0;
  double prev_remaining_ = -1;
  bool paused_ = false;
  bool careful_ = false, careful_rot_ = false; double careful_travel_ = 0; Pose careful_last_;
};

const char* state_name(int s);
const char* reason_name(int r);

}  // namespace kv_nav
