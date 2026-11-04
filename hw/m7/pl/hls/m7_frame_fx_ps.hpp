// GEO_FX1 (2026-11-03): host / PS side of the fixed-point geometry core (testbench and C-model
// drivers only; never synthesized). GEO_FX3 (2026-11-04): the PS arithmetic now lives in ONE place,
// hw/m7/frame_fx.cpp fpga_m7_fx_prepare (the kv_server driver uses the same file); this header only
// wraps it for the benches, and gives the register prototype of the fixed-point fpga_m7_frame_axi
// (old arguments first, unchanged order; GEO_FX1 arguments appended; GEO_FX3: the 12 constant
// ports became the trailing s_axilite array fx_k[24]).
#ifndef FPGA_M7_FRAME_FX_PS_HPP
#define FPGA_M7_FRAME_FX_PS_HPP

#include <cstdint>

#include <ap_int.h>

#include "../../frame_fx.hpp"

#define FPGA_FX_ARGS_NARROW 1  // GEO_FX1 step 1b register types
#define FPGA_FX_K_ARRAY 1      // GEO_FX3 fx_k[24]

extern "C" std::uint32_t fpga_m7_frame_axi(
    const std::uint32_t* depth_m, std::uint32_t* flags, const float inverse_k[9],
    const float world_camera[12], double grid_origin_x, double grid_origin_y,
    double grid_origin_z, double voxel_size_m, std::uint32_t width, std::uint32_t height,
    std::uint32_t nx, std::uint32_t ny, std::uint32_t nz, std::uint32_t* count_accepted,
    std::uint32_t* count_invalid, std::uint32_t* count_outside, std::uint32_t dlat2_en,
    std::uint32_t depth_fmt, float depth_unit, std::uint32_t* depth_crc,
    ap_uint<13> fx_origin_x, ap_uint<13> fx_origin_y, ap_uint<13> fx_origin_z,
    std::uint32_t fx_codes, ap_uint<16> fx_c20,
    ap_uint<21>* st_npos, ap_uint<21>* st_le20, std::uint32_t* st_mm0, std::uint32_t* st_mm1,
    const std::uint32_t fx_k[FPGA_M7_FX_K_WORDS]);

struct FpgaFxK {
    std::int64_t Gu[3], Gv[3], G0[3];  // round_half_even(G * 2^36), 41-bit signed
    std::int64_t O[3];                 // round_half_even(O * 2^22), 36-bit signed
    std::int32_t origin[3];            // reference origin (Q8 int)
    std::uint32_t c_lo, c_hi, c20;
    double pmax, reach;
    int status;                        // 0 ok, 2/3 as the reference, 5 outside the envelope
    std::uint32_t words[FPGA_M7_FX_K_WORDS];  // fx_k register image (GEO_FX3)
};

// reach_limit: 2^20 (GEO_FX0 envelope) or 2^17 (W1 envelope, DESIGN.md section 4 W1)
inline int fpga_fx_prepare(std::uint32_t w, std::uint32_t h, std::uint32_t nx, std::uint32_t ny,
                          std::uint32_t nz, float unit, const float ik[9], const float wc[12],
                          const double org[3], double voxel, FpgaFxK& k,
                          double reach_limit = 1048576.0) {
    std::int64_t gk[12];
    std::uint32_t thr[3];
    double env[2];
    k.status = fpga_m7_fx_prepare(w, h, nx, ny, nz, unit, ik, wc, org, voxel, reach_limit, gk, k.origin,
                                 thr, env);
    for (int a = 0; a < 3; ++a) {
        k.Gu[a] = gk[a]; k.Gv[a] = gk[3 + a]; k.G0[a] = gk[6 + a]; k.O[a] = gk[9 + a];
    }
    k.c_lo = thr[0]; k.c_hi = thr[1]; k.c20 = thr[2];
    k.pmax = env[0]; k.reach = env[1];
    fpga_m7_fx_pack(gk, k.words);
    return k.status;
}

#endif
