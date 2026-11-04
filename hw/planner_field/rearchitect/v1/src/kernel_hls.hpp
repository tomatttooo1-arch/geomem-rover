#pragma once
#include <ap_int.h>
extern "C" void planner_field(const ap_uint<8> map[6400],ap_uint<32> field[6400],
  ap_uint<32> flags[64],int sx,int sy,int gx,int gy,unsigned k_max,
  ap_uint<32>& status,ap_uint<32>& rounds,ap_uint<32>& field_valid);
