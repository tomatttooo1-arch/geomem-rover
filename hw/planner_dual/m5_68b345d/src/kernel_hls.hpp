#pragma once
#include <ap_int.h>

// grid_mode=0: 80x80 / 64 flags; grid_mode=1: 200x200 / 256 flags.
// Unsupported modes return status=8 without touching the DDR arrays.
extern "C" void planner_field_dual(
    const ap_uint<8> map[40000], ap_uint<32> field[40000],
    ap_uint<32> flags[256], int sx, int sy, int gx, int gy, unsigned k_max,
    ap_uint<32>& status, ap_uint<32>& rounds, ap_uint<32>& field_valid,
    unsigned grid_mode);
