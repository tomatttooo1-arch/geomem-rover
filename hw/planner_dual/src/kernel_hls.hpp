#pragma once
#include <ap_int.h>

// grid_mode=0: 80x80 / 64 flags; grid_mode=1: 200x200 / 256 flags.
// Unsupported modes return status=8 without touching the DDR arrays.
// 68b345d core bit for bit. With bit0=1: 132 parameter bytes are read from map[count..count+131]
// (count = 6400 / 40000) and, after the flags, one header word + n path words are written to
// flags[F], flags[F+1 .. F+n] (F = 64 / 256). Layout: runs/.../PLNPATH0/DESIGN.md section 4.4.
constexpr int kPlnParamBytes = 132;
constexpr int kPlnCandMax = 64;
constexpr int kPlnPathMax = 4096;
constexpr int kPlnMapDepth = 40000 + kPlnParamBytes;      // 40132
constexpr int kPlnFlagsDepth = 256 + 1 + kPlnPathMax;     // 4353

// global-map area at map == field == A (runs/.../GMAP0/DESIGN.md section 4.1, model tools/gmap/gmap_model.c);
// any other value: result word 0 = 0x204 (bad op). The status register reads result word 0 after a gmap op.
// GMAP3: 4 WINDOW (body seed + swept clearing, DESIGN_ADD_PLGRID.md, positions Q16), swept time map T, aux slot and
// WINDOW parameter block after the GMAP0 area; CLEAR param byte 13 bit 0 also clears T. Result version 2.
// GMAP3 port widths: path_ctrl 1 bit (only bit 0 was ever used), gmap_op 3 bits ([2:0] as documented; 8.. alias).
constexpr int kGmapAreaBytes = 1173088;
constexpr int kGmapAreaWords = kGmapAreaBytes / 4;        // 293272
constexpr int kGmapOffCoarse = 320000;
constexpr int kGmapOffFrame = 360000;
constexpr int kGmapOffParam = 366400;
constexpr int kGmapOffImage = 366416;
constexpr int kGmapOffResult = 526416;
constexpr int kGmapOffT = 526432;
constexpr int kGmapOffAux = 1166432;
constexpr int kGmapOffWparam = 1172832;
static_assert(kGmapAreaBytes >= kPlnMapDepth, "map port depth");

extern "C" void planner_field_dual(
    const ap_uint<8> map[kGmapAreaBytes], ap_uint<32> field[kGmapAreaWords],
    ap_uint<32> flags[kPlnFlagsDepth], int sx, int sy, int gx, int gy, unsigned k_max,
    ap_uint<32>& status, ap_uint<32>& rounds, ap_uint<32>& field_valid,
    unsigned grid_mode, ap_uint<1> path_ctrl, ap_uint<3> gmap_op);
