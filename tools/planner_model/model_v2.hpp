#pragma once
#include <cstdint>
enum PlannerV2Status {
  V2_EXACT=0, V2_WITHIN_TOLERANCE=1, V2_INVALID_START=2,
  V2_INVALID_GOAL=3, V2_NO_PATH=4, V2_SAFETY_REJECTED=5,
  V2_INVALID_ARGUMENTS=6, V2_NOT_CONVERGED=7
};
// Coordinates are cell-centre indices, including fractional requested goals.
// Output starts at the requested start cell and ends at the resolved continuous point.
// Both map and goal remain unchanged. cap >= w*h+2. mode 0=float64, 1=uint32.
extern "C" int wave_plan_v2(const uint8_t* map, int w, int h, int sx, int sy,
    double gx, double gy, double resolution, double tolerance, int mode,
    double* out_x, double* out_y, int cap, uint32_t* rounds, int* status,
    double* resolved_xy);
