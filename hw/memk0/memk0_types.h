// MEMK0 role types (MEMK3 HLS form): one set for the C model (g++ -I$XILINX_HLS/include) and the HLS kernel.
// Widths follow MEMK0/DESIGN.md section 4.1 plus the MEMK3 corrections (README "width table").
#pragma once
#include <ap_int.h>
typedef unsigned int u32;
typedef long long i64;
typedef unsigned long long u64;
typedef ap_int<26>  pos_t;     // s7.18 m
typedef ap_int<27>  med_t;     // two middles summed, 2^-19 m
typedef ap_uint<48> sf2_t;     // 1/sum w, 2^-48 m^2, saturating (Q clamps at 2^48-1 anyway)
typedef ap_uint<50> so2_t;     // observation sd^2, 2^-48 m^2 (sd clamped to 2 m)
typedef ap_uint<6>  ns_t;      // samples in ring (<= 32)
typedef ap_uint<5>  hd_t;      // ring head
typedef ap_uint<7>  cls_t;
typedef ap_uint<2>  type_t;    // 0 free, 1 track, 2 candidate
typedef ap_uint<32> odo_t;     // path 2^-22 m / turn 2^-21 rad
typedef ap_int<27>  lo_t;      // existence 2^-24, [-3, 3.5]
typedef ap_int<32>  win_t;
typedef ap_uint<24> pd_t;      // 2^-24
typedef ap_uint<24> skey_t;    // frame seq
typedef ap_uint<24> w_t;       // 2^-12 m^-2, [2^10, 2^23.3]
typedef ap_uint<26> sd_t;      // 2^-24 m, <= 2 m
typedef ap_uint<29> d_t;       // 2^-20 m
typedef ap_uint<58> sq_t;      // isqrt argument
