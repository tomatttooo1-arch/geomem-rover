// GEO_FX3 (2026-11-04): fixed-point geometry frame core (GEO_FX1 m7_frame_stream.cpp) -- PS side.
//
// Two things, both plain C++ (no HLS headers), built into one host / A53 shared library by
// host/m7_frame_fx.py and into the HLS testbenches:
//
//   fpga_m7_fx_prepare   the PS driver arithmetic: per-frame constants of
//                       runs/nationals-prelim-20261102/GEO_FX0/DESIGN.md section 2.2 (double, fixed
//                       operation order, nearbyint = round half even). MUST be compiled with
//                       -ffp-contract=off (aarch64 GCC contracts a*b+c to FMA otherwise and the
//                       constants stop matching the golden model). Bit-exact against the GEO_FX0
//                       golden fx_prepare on rec30 / car570 (GEO_FX3/tools/ps_check.cpp).
//   fpga_m7_fx_frame     the bit-exact model of the core from its REGISTER values (what the PL sees):
//                       GEO_FX0 golden FX arm (fx_golden.cpp: fx_E + Dlat2 back end + reference DDA
//                       walk + merge), plus the statistics and depth crc registers. Used as the
//                       expectation of every "PL vs reference" check since GEO_FX3 (mock device,
//                       host tests, board check); verified against the golden FX flags files.
//
// Register image of the per-frame constants (GEO_FX3 AXI-Lite tightening): one s_axilite array
// FX_K of 24 x u32 = 12 slots of 64 bits, slot s at words 2s (low) and 2s+1 (high), two's
// complement; slots 0-2 Gu x/y/z, 3-5 Gv, 6-8 G0 (the core uses bits [40:0]), 9-11 O (bits [35:0]).
#ifndef FPGA_M7_FRAME_FX_HPP
#define FPGA_M7_FRAME_FX_HPP

#include <cstdint>

#define FPGA_M7_FX_K_WORDS 24
// reach limit of the W1 ray record (DESIGN.md section 4 W1; GEO_FX1 W1 / X1 cores): reach < 2^17
#define FPGA_M7_FX_REACH_W1 131072.0

extern "C" {

// Returns status: 0 ok, 2 non-finite input (as the reference), 3 camera origin outside the grid
// (as the reference), 5 outside the fixed-point envelope (c_lo > c_hi, max|P| >= 15.5 or
// reach >= reach_limit). Status != 0: do not start the core, use the CPU reference.
//   gk[12]     Gu x,y,z, Gv x,y,z, G0 x,y,z (s4.36), O x,y,z (s13.22), int64
//   origin[3]  reference Q8 origin (floor((t - org) / voxel * 256)), valid only with status 0 / 5
//   thr[3]     c_lo, c_hi, c20 (cached per unit: the 65535-code scan runs once per new unit)
//   env[2]     max|P| over the image corners, reach (Q8)
int fpga_m7_fx_prepare(std::uint32_t w, std::uint32_t h, std::uint32_t nx, std::uint32_t ny,
                      std::uint32_t nz, float unit, const float ik[9], const float wc[12],
                      const double org[3], double voxel, double reach_limit, std::int64_t gk[12],
                      std::int32_t origin[3], std::uint32_t thr[3], double env[2]);

// gk[12] -> FX_K register words (see the header comment).
void fpga_m7_fx_pack(const std::int64_t gk[12], std::uint32_t words[FPGA_M7_FX_K_WORDS]);

// The core, from register values. codes: w*h uint16 (row-major). Returns the core status
// (1 bad dimensions, else 0); status != 0 leaves flags untouched and every output 0.
//   origin_reg[3]  FX_ORIGIN_X/Y/Z (bits [12:0]); codes_reg FX_CODES (c_lo [15:0] | c_hi [31:16]);
//   c20_reg FX_C20 (bits [15:0]); unit_bits DEPTH_UNIT (float32 bits, crc only)
//   flags[32768]; counts[3] accepted, invalid, clipped; stats[4] ST_NPOS, ST_LE20, ST_MM0, ST_MM1;
//   crc[1] DEPTH_CRC (zlib crc32 of the float32 depth f32(code) * unit, code 0 -> 0x7fc00000)
std::uint32_t fpga_m7_fx_frame(const std::uint16_t* codes, std::uint32_t w, std::uint32_t h,
                              std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
                              const std::uint32_t words[FPGA_M7_FX_K_WORDS],
                              const std::uint32_t origin_reg[3], std::uint32_t codes_reg,
                              std::uint32_t c20_reg, std::uint32_t dlat2_en, std::uint32_t unit_bits,
                              std::uint8_t* flags, std::uint32_t counts[3], std::uint32_t stats[4],
                              std::uint32_t crc[1]);
}

#endif
