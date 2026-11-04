#pragma once
//  make_params(): the field-independent part of extract() R2/R3 (exact-goal safety, tolerance
//                 candidate geometry), expressions copied from planner_split.hpp extract(); the
//                 only change is the cell size: `.05` (cell->metre) becomes `res` (0.05 for the
//                 80 window = unchanged doubles; 0.10 for the 200 grid, controller ruling 11-04:
//                 +-0.15 m, step 0.05 m in metres). The candidates are filtered by the map-only
//                 conditions and stably sorted by (distance, enumeration order).
//  assemble():    core header + points -> the same Result/x/y as extract() (appends the
//                 continuous end point exactly as extract() does).
//  ref_extract(): the PS reference for any size: planner_split.hpp extract() with `.05` -> res.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>
#include "planner_split.hpp"
#include "path_model.hpp"
namespace plnpath {
using kv_nav_planner::Request;
using kv_nav_planner::Result;

struct PsParams {
  std::uint8_t bytes[kParamBytes];
  double px[kCandMax], py[kCandMax];
  int n = 0;
};

template <int W, int H, std::uint32_t Inf>
struct PS {
  using E = kv_nav_planner::detail::Engine<W, H, Inf>;
  // m = working map (E::prepare(input, q.sx, q.sy)).
  static PsParams make_params(const std::uint8_t* m, Request q, double res) {
    PsParams o;
    std::memset(o.bytes, 0, sizeof o.bytes);
    const int gx = int(std::round(q.gx)), gy = int(std::round(q.gy));
    o.bytes[1] = E::safe_segment(m, gx, gy, q.gx, q.gy) ? 1 : 0;
    struct C { double d, px, py; int a, b, order; };
    std::vector<C> c;
    const double goal_mx = q.gx * res, goal_my = q.gy * res;
    int order = 0;
    for (double y = goal_my - .15; y <= goal_my + .15; y += .05) {
      for (double x = goal_mx - .15; x <= goal_mx + .15; x += .05) {
        if (x < 0 || y < 0) continue;
        int a = int(std::round(x / res)), b = int(std::round(y / res));
        if (!E::inside(a, b)) continue;
        double d = (x - goal_mx) * (x - goal_mx) + (y - goal_my) * (y - goal_my);
        if (!E::safe_segment(m, a, b, x / res, y / res)) continue;
        c.push_back(C{d, x / res, y / res, a, b, order++});
      }
    }
    std::stable_sort(c.begin(), c.end(), [](const C& u, const C& v) { return u.d < v.d; });
    o.n = int(std::min<size_t>(c.size(), kCandMax));
    o.bytes[0] = std::uint8_t(o.n);
    for (int j = 0; j < o.n; ++j) {
      o.bytes[4 + 2 * j] = std::uint8_t(c[j].a);
      o.bytes[5 + 2 * j] = std::uint8_t(c[j].b);
      o.px[j] = c[j].px; o.py[j] = c[j].py;
    }
    return o;
  }
  // words[0] = header, words[1..n] = points (start -> target cell).
  static Result assemble(const std::uint32_t* words, const PsParams& pp, Request q, double* ox, double* oy) {
    Result r;
    const unsigned st = (words[0] >> 24) & 15u, ch = (words[0] >> 16) & 255u, n = words[0] & 0xffffu;
    r.field_valid = true;
    if (st == P_NO_PATH || st == P_SAFETY) { r.status = int(st); return r; }
    if (st != P_EXACT && st != P_TOL) { r.status = -int(st); r.field_valid = false; return r; }  // 9/15: PS fallback
    for (unsigned i = 0; i < n; ++i) { ox[i] = double(words[1 + i] & 255u); oy[i] = double((words[1 + i] >> 8) & 255u); }
    const double px = st == P_EXACT ? q.gx : pp.px[ch], py = st == P_EXACT ? q.gy : pp.py[ch];
    int cnt = int(n);
    if (ox[cnt - 1] != px || oy[cnt - 1] != py) { ox[cnt] = px; oy[cnt] = py; ++cnt; }
    r.resolved_x = px; r.resolved_y = py; r.count = cnt;
    r.status = st == P_EXACT ? kv_nav_planner::EXACT : kv_nav_planner::WITHIN_TOLERANCE;
    return r;
  }
  // planner_split.hpp extract() verbatim except `.05` -> res (m = working map, p = complete field).
  static Result ref_extract(const std::uint8_t* m, const std::uint32_t* p, Request q, double res,
                            double* ox, double* oy, int cap) {
    using namespace kv_nav_planner::detail;
    Result r;
    const int gx = int(std::round(q.gx)), gy = int(std::round(q.gy));
    r.field_valid = true;
    int tx = gx, ty = gy; double px = q.gx, py = q.gy;
    const bool exact = p[ty * W + tx] < Inf;
    if (exact) {
      if (!E::safe_segment(m, tx, ty, px, py)) { r.status = kv_nav_planner::SAFETY_REJECTED; return r; }
    } else {
      bool found = false; double best_distance = std::numeric_limits<double>::max();
      const double goal_mx = q.gx * res, goal_my = q.gy * res;
      for (double y = goal_my - .15; y <= goal_my + .15; y += .05) {
        for (double x = goal_mx - .15; x <= goal_mx + .15; x += .05) {
          if (x < 0 || y < 0) continue;
          int a = int(std::round(x / res)), b = int(std::round(y / res));
          if (!E::inside(a, b) || p[b * W + a] >= Inf) continue;
          double d = (x - goal_mx) * (x - goal_mx) + (y - goal_my) * (y - goal_my);
          if (d < best_distance && E::safe_segment(m, a, b, x / res, y / res)) {
            found = true; best_distance = d; tx = a; ty = b; px = x / res; py = y / res;
          }
        }
      }
      if (!found) { r.status = kv_nav_planner::NO_PATH; return r; }
    }
    int x = tx, y = ty, n = 0;
    for (int step = 0; step < W * H; ++step) {
      ox[n] = x; oy[n] = y; ++n;
      if (x == q.sx && y == q.sy) break;
      auto best = p[y * W + x]; int a = -1, b = -1, best_d = 2147483647;
      for (int k = 0; k < 8; ++k) {
        const int xx = x + dx[k], yy = y + dy[k];
        if (!E::edge(m, x, y, xx, yy) || p[yy * W + xx] >= Inf) continue;
        const auto candidate = E::add(p[yy * W + xx], entry(m[y * W + x]) * (dx[k] && dy[k] ? 14u : 10u));
        const int distance = (xx - q.sx) * (xx - q.sx) + (yy - q.sy) * (yy - q.sy);
        if (candidate <= p[y * W + x] && (a < 0 || candidate < best || (candidate == best && distance < best_d))) {
          best = candidate; best_d = distance; a = xx; b = yy;
        }
      }
      if (a < 0) { r.status = kv_nav_planner::NO_PATH; return r; }
      x = a; y = b;
    }
    if (x != q.sx || y != q.sy || n + 1 > cap) { r.status = kv_nav_planner::NO_PATH; return r; }
    for (int i = 0; i < n / 2; ++i) {
      double t = ox[i]; ox[i] = ox[n - 1 - i]; ox[n - 1 - i] = t;
      t = oy[i]; oy[i] = oy[n - 1 - i]; oy[n - 1 - i] = t;
    }
    if (ox[n - 1] != px || oy[n - 1] != py) { ox[n] = px; oy[n] = py; ++n; }
    if (n == 1 && !E::safe_segment(m, ox[0], oy[0], ox[0], oy[0])) { r.status = kv_nav_planner::SAFETY_REJECTED; return r; }
    for (int i = 1; i < n; ++i) if (!E::safe_segment(m, ox[i - 1], oy[i - 1], ox[i], oy[i])) {
      r.status = kv_nav_planner::SAFETY_REJECTED; return r;
    }
    r.resolved_x = px; r.resolved_y = py; r.count = n;
    r.status = exact ? kv_nav_planner::EXACT : kv_nav_planner::WITHIN_TOLERANCE; return r;
  }
};
}  // namespace plnpath
