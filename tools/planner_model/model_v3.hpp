#pragma once
#include "model_v2.hpp"
// mode: 0=float64, 1=uint64. variant: 0=clear-only, 1=10/14 weighted,
// 2=weighted plus clearance-preserving continuous shortcutting.
// Only the internal start cell is cleared. Input remains immutable.
extern "C" int wave_plan_v3(const uint8_t* map,int w,int h,int sx,int sy,
    double gx,double gy,double resolution,double tolerance,int mode,
    double* out_x,double* out_y,int cap,uint32_t* rounds,int* status,
    double* resolved_xy,int variant);
