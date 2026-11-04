// CTRLPS5 host-test stand-in for planner_server's G1 route: Nav2 Humble NavFn (navfn.cpp/hpp unmodified, pulled
// (setCostmap(isROS, allow_unknown false), setStart(goal cell) / setGoal(robot cell), calcNavFnDijkstra(true), then
// getPlanFromPotential's calcPath(4 * max size), path reversed to robot -> goal). Costs = the global costmap's char map (inflated, 253/254, 255 unknown).
// C interface for ctypes (tools/kv_nav/sim/lockstep_loop.py). Returns the number of points (<= cap), 0 = no path.
#include <algorithm>
#include "nav2_navfn_planner/navfn.hpp"

extern "C" int navfn_route(const unsigned char* costs, int nx, int ny, int sx, int sy, int gx, int gy,
                           float* out_x, float* out_y, int cap) {
  static nav2_navfn_planner::NavFn* nf = nullptr;
  if (!nf || nf->nx != nx || nf->ny != ny) { delete nf; nf = new nav2_navfn_planner::NavFn(nx, ny); }
  nf->setNavArr(nx, ny);
  nf->setCostmap(costs, true, false);
  int st[2] = {gx, gy}, gl[2] = {sx, sy};
  nf->setStart(st);
  nf->setGoal(gl);
  nf->calcNavFnDijkstra(true);                       // makePlan ignores its return too; reachability below
  if (nf->potarr[gy * nx + gx] >= POT_HIGH) return 0;   // the goal cell (NavFn "start") has no potential: no path
  // getPlanFromPotential: setStart(goal) again, calcPath(max(size_x, size_y) * 4)
  if (nf->calcPath(std::max(nx, ny) * 4) == 0) return 0;
  const int n = nf->getPathLen();
  if (n <= 0) return 0;
  const float* x = nf->getPathX(); const float* y = nf->getPathY();
  int k = 0;
  for (int i = n - 1; i >= 0 && k < cap; --i, ++k) { out_x[k] = x[i]; out_y[k] = y[i]; }
  return k;
}
