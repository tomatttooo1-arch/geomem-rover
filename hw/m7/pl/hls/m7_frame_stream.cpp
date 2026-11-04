// GEO_FX1 (2026-11-03): FIXED-POINT GEOMETRY CORE (design: runs/nationals-prelim-20261102/
// GEO_FX0/DESIGN.md, approved; golden model GEO_FX0/tools/fx_golden.cpp, FX arm). The float32 /
// double conversion stage of the reference (and of G3/D16 below) is replaced by
//   E_a = P27_a(u, v) * code + O_a,  coord_a = E_a >> 22,  valid <=> c_lo <= code <= c_hi
// with P_a(u, v) = G0_a + 2^13 + v * Gv_a + u * Gu_a kept in 41-bit row / pixel accumulators and
// P27_a = P_a >> 14 (DESIGN.md section 2). All per-frame constants (Gu, Gv, G0, O, the Q8 origin,
// c_lo / c_hi / c20) are computed by the PS (section 2.2, host form m7_frame_fx_ps.hpp) and written
// to the registers appended after depth_crc; the old calibration registers (inverse_k,
// world_camera, grid_origin_*, voxel_size_m) keep their offsets and are no longer read. Input is
// 16-bit codes only (depth_fmt must be 1, else status 6); status 2 / 3 / 5 are PS decisions now.
// depth_crc keeps the crc32 of the float32 depth f32(code) * unit (exact integer x float
// multiplier, DESIGN.md section 3.4). New read-only statistics st_npos / st_le20 / st_mm0 / st_mm1
// (section 3.3). The result is the golden FX arm bit for bit, NOT the float reference any more.
// Everything after the ray record (dispatch, lanes, planes, merge, write-out, G2 clear) is the
// GEO_SLIM4 (m4g) code unchanged; the historical notes below describe it.
//
// Streaming implementation of the M7 geometry frame top (A1, 2026-10-11),
// with the ray walk split over FPGA_M7_LANES parallel lanes (A2, 2026-10-11,
// docs/design/M7_RAY_LANES_A2_DESIGN_20261011.md variant A): a round-robin
// dispatcher feeds one depth-4 SRL FIFO per lane; every lane is its own
// DATAFLOW process (walk_lane<ID>, the A1 latch-free walk loop) writing its
// own pair of 32Kx1 bit planes; the write-out ORs the planes of all lanes.
// The OR merge is exact for the same reason as below (marks are idempotent
// and order independent), so any ray-to-lane assignment gives the reference
// result. FPGA_M7_LANES is a compile-time parameter (default 8, 1..16).
//
// Same top function, same arguments, same INTERFACE pragmas as
// m7_frame_axi.cpp (so the s_axilite register map is unchanged); same result,
// bit for bit, as the reference hw/m7/frame_depth.cpp (clipped semantics,
// docs/design/M7_CLIP_SPEC_20261011.md). Only the structure differs:
//
//   checks + origin (sequential, copied from the reference)
//   clear the bit planes free_pl / hit_pl ONLY if planes_dirty (G2,
//   2026-11-02: normally the previous frame's write-out already cleared them)
//   DATAFLOW { read: burst depth_m -> stream (1 pixel / cycle)
//              convert: per pixel, the reference's float/double expressions
//                       verbatim (-ffp-contract=off, no FMA), classification,
//                       counts, and a precomputed ray record per walked ray
//              walk: one DDA step per cycle; the reference's int64 cross
//                    products next[a]*delta[b] are replaced by three pairwise
//                    differences P_ab = next[a]*delta[b] - next[b]*delta[a]
//                    updated incrementally (exact integer arithmetic) }
//   merge: flags[i] = hit ? 2 : (free ? 1 : 0), written as one burst; the
//          same loop writes 0 back to every plane bit it has read (G2).
//
// G2 write-back clear (2026-11-02): the separate 32768-cycle clear pass ran
// before every frame. Now the write-out, which reads every bit of every lane
// plane once anyway, writes it back to 0 through the second BRAM port in the
// same cycle, so after a completed frame all planes are zero. The static
// flag planes_dirty (reset value true, kept under ap_rst by HLS RESET) is set
// before the ray walk and cleared after the write-out; the old full clear
// runs only when it is set: the first frame after power-up / PL reset, and
// the frame after one that did not reach the end of the write-out. Error
// returns (status 1..3) leave before the walk and touch neither the planes
// nor the flag. The write-out covers all 32768 cells whatever nx/ny/nz is,
// so a grid-size change between frames leaves nothing behind.
//
// G1d Dlat2 ray thinning (2026-11-02, register dlat2_en, default 0 = off):
// with dlat2_en = 1 the convert stage forwards to the lanes only
//   * an in-grid ray if it is the first in-grid ray of its 2x2 pixel block
//     (block = (v/2, u/2)) OR the first in-grid ray of the frame whose
//     endpoint voxel is that voxel (endpoint guarantee: keeps every HIT);
//   * a clipped ray if it is the first clipped ray of its 2x2 pixel block.
// In-grid and clipped rays have separate block marks. This is the Dlat2
// variant of runs/nationals-prelim-20261102/G1b tools/g1b.cpp (IN_LAT 2,
// CL_BLK 2), bit for bit. The decision sits after the conversion; the walk
// lanes are unchanged; counts (accepted / invalid / clipped) still count
// every valid pixel, kept or not. State:
//   blk_mark  one 2-bit entry per block column (640 = 1280/2): written by
//             every pixel of an even row (in-grid / clipped ray seen in this
//             block so far), read by the pixels of the following odd row.
//             Every even row rewrites every column it covers before the odd
//             row reads it, so it is never cleared.
//   ev_pl     32768 x 1 endpoint-voxel marks (one RAMB36), set by every
//             in-grid ray; cleared by the write-out (G2 write-back) and by
//             the planes_dirty full clear, like the lane planes.
// Both tables are read-modify-write in the II=1 convert loop. The RAW
// hazard (a pixel reads an entry that one of the last few pixels wrote) is
// resolved by explicit forwarding from the last kFwd writes, and the memory
// dependence is declared false; see convert().
// With dlat2_en = 0 every valid ray is forwarded, as before.
//
// G3 exact integer Q8 conversion (2026-11-02): the per-pixel
// floor((double(world) - org) / vox * 256) with its range gate no longer uses
// a double divider, multiplier, floor ROM or double compares; it is computed
// exactly in integer arithmetic (m7_q8_exact.hpp: one 53x27 and one 27x54
// multiply per axis against the frame constant M = floor(2^79 / D)). The
// frame constants (prepared voxel size, conversion of -org for subnormal
// differences) are computed once per frame by a 27-step restoring division.
// Only the double subtraction double(world) - org stays a float operator.
// Same integer, same gate, bit for bit (host test: G3 README section 1).
//
// Why the merge is exact: in the reference a FREE mark is "set to 1 only if
// still 0" and a HIT mark sets 2 unconditionally. Both are idempotent and
// independent of ray order, so the final byte is 2 if any ray hit the cell,
// else 1 if any ray walked through it, else 0.
//
// Why the step rule is exact: the reference steps every active axis whose
// ratio next/delta equals the minimum over active axes (an axis is active if
// direction != 0 and cell != target). P_ab <= 0 <=> next[a]/delta[a] <=
// next[b]/delta[b] (delta > 0 on active axes), so axis a steps iff it is
// active and P_ab <= 0 (resp. P_ba >= 0) against every other active axis.
//
// Why the clamped target is exact: the walk only compares cell with target
// while cell is inside [0, dims) (it stops at the first outside cell), and
// clamping target to [-1, 32] maps only values outside [0, 31] to -1 or 32.
//
// Fault switches (C model positive controls only, never synthesized):
//   FPGA_M7_FAULT_NOCLIP   clipped rays are counted but not walked
//   FPGA_M7_FAULT_TRUNC    endpoint cell uses C '/' (truncation) instead of floor
//   FPGA_M7_FAULT_CLIPHIT  clipped rays also mark HIT at the target clamped into the grid
//   FPGA_M7_FAULT_TIE      on a tie between x and y only x steps
//   FPGA_M7_FAULT_NEXT     next[] for direction < 0 uses the direction > 0 formula
//   FPGA_M7_FAULT_MERGE    merge gives FREE priority over HIT
//   FPGA_M7_FAULT_LANE_SKIP     merge leaves out the last lane (N > 1)
//   FPGA_M7_FAULT_LANE_NOCLEAR  neither the write-back clear nor the dirty
//                              clear touches the last lane (N > 1; shows on
//                              the second frame run in the same process)
//   FPGA_M7_FAULT_NODIRTY       the dirty-flag clear never runs (shows on the
//                              frame after a C-model abort, see
//                              fpga_m7_stream_sim_abort)
//   FPGA_M7_FAULT_DLAT_NOEP    Dlat2 without the endpoint-voxel guarantee
//                              (G1b PlatOnly2): HIT cells are lost
//   FPGA_M7_FAULT_DLAT_NOEVCLEAR the write-out does not clear ev_pl (shows
//                              from the second Dlat2 frame on)
//   FPGA_M7_FAULT_LANE_NOEND    the last lane gets no end-of-frame record: the C
//                              model blocks for ever on the empty FIFO (in RTL
//                              the DATAFLOW region never finishes)
//   FPGA_M7_FAULT_D16_ODDTAIL  16-bit input: an odd pixel count also emits the
//                              ignored upper half of the last word (stream
//                              one pixel too long)
//   FPGA_M7_FAULT_D16_NONAN     16-bit input: code 0 becomes 0.0f * unit instead
//                              of the NaN of the reference rule (flags and
//                              counts cannot see it: 0 m is invalid too; only
//                              the depth crc can)
// Hardware-only faults (compiled only under __SYNTHESIS__, so C simulation
// still passes; C/RTL co-simulation must fail -- positive controls for the
// co-simulation gate, never for a product build):
//   FPGA_M7_HWFAULT_LANE_SKIP   merge leaves out the last lane
//   FPGA_M7_HWFAULT_LANE_ALIAS  the last lane writes the planes of the lane
//                              before it (two writers on one plane pair; the
//                              OR of a correct RTL is unchanged by this, so it
//                              only fails if the RTL loses writes)
//                              (Vitis HLS 2022.2 refuses this at csynth:
//                              HLS 200-779 / 200-979 single-writer check)
//   FPGA_M7_HWFAULT_PLANE_SWAP  the last lane's free and hit planes are swapped
//                              (wrong-plane wiring that keeps one writer per
//                              plane, so it reaches the RTL)
//   FPGA_M7_HWFAULT_DLAT_NOFWD  convert drops the forwarding of ev_pl / blk_mark
//                              (memory dependence still declared false)
//   FPGA_M7_HWFAULT_DROP_FULL   the dispatcher drops a record when the target
//                              lane FIFO is full instead of waiting
//   FPGA_M7_HWFAULT_D16_SWAP    16-bit input: the two codes of a word are taken
//                              in the wrong order (upper half first)
#include "../../frame_depth.hpp"
#include "../../frame_fx.hpp"

#include <cfloat>
#include <cmath>

#include <ap_int.h>
#include <hls_stream.h>
#include <ap_utils.h>

#ifndef __SYNTHESIS__
#include <cstdio>
#include <cstdlib>
#endif

namespace {

std::uint32_t f32_bits(float value) {
    union { float f; std::uint32_t u; } x;
    x.f = value;
    return x.u;
}

std::uint32_t grid_offset(const std::int32_t cell[3],
                          std::uint32_t nx, std::uint32_t ny) {
    return (static_cast<std::uint32_t>(cell[2]) * ny +
            static_cast<std::uint32_t>(cell[1])) * nx +
            static_cast<std::uint32_t>(cell[0]);
}

#ifndef FPGA_M7_LANES
#define FPGA_M7_LANES 8
#endif
#if FPGA_M7_LANES < 1 || FPGA_M7_LANES > 16
#error "FPGA_M7_LANES must be 1..16"
#endif
const int kLanes = FPGA_M7_LANES;
const unsigned kLaneFifoDepth = 4;

const std::uint32_t kCap = FPGA_M7_FRAME_GRID_CAPACITY;
const unsigned kBlkCols = FPGA_M7_FRAME_MAX_WIDTH / 2;    // 640 2x2 block columns
const int kFwd = 3;  // forwarding depth of the convert-loop read-modify-writes
#if defined(__SYNTHESIS__) && defined(FPGA_M7_HWFAULT_DLAT_NOFWD)
#define FPGA_M7_HWFAULT_DLAT_NOFWD_ACTIVE
#endif
const unsigned kMaxSteps = 3 * FPGA_M7_FRAME_GRID_AXIS;  // 96, the reference cap

// GEO_FX1 W1: under the W1 envelope (PS: reach < 2^17 Q8, status 5 otherwise) every emitted ray
// has |end| <= reach + 1 < 2^17 + 1 and 0 <= origin < 2^13, so delta < 2^18; next <= 256 * (1 + 96)
// < 2^15 during the walk, so |P_ab| = |next_a * delta_b - next_b * delta_a| < 2^33: 34 bits.
// The record stores delta itself; the walk's "+ delta * 256" is a shift (wiring).
typedef ap_int<34> cross_t;
typedef ap_uint<18> dlt_t;    // delta < 2^18
inline cross_t d256(dlt_t d) { return cross_t(ap_uint<26>(d) << 8); }
typedef ap_int<7> cell_t;     // cell in [-1, 32], clamped target in [-1, 32]

// GEO_FX1: per-frame constants of the fixed-point conversion (DESIGN.md section 2.2), packed into
// wide scalars (no array members; see the G3 step 4 note in the history: an array member of a
// by-value DATAFLOW argument becomes one channel per element).
typedef ap_int<41> acc_t;     // s4.36 accumulators and coefficients
struct FxConst {
    ap_uint<3 * 41> gu, gv, g0;   // axis a at [41a+40:41a], two's complement
    ap_uint<3 * 36> o;            // s13.22, axis a at [36a+35:36a]
    ap_uint<16> c_lo, c_hi, c20;
};

struct Ray {
    cross_t pxy, pxz, pyz;
    dlt_t dx, dy, dz;
    cell_t tx, ty, tz;
    ap_int<2> sx, sy, sz;
    ap_uint<15> tidx;
    bool tin;
    bool last;
};

struct Counts {
    std::uint32_t accepted, invalid, clipped;
    std::uint32_t npos, le20, mm0, mm1;  // GEO_FX1 statistics (DESIGN.md section 3.3)
};

#ifndef __SYNTHESIS__
}  // namespace
// C-model instrumentation (not synthesized): walk loop iterations of the last
// frame (= cycles at II=1), walked rays, max steps of one ray, cross-product
// shadow mismatches (must stay 0).
extern "C" {
unsigned long long fpga_m7_stream_walk_iters = 0;
unsigned long long fpga_m7_stream_walk_rays = 0;
unsigned long long fpga_m7_stream_walk_steps = 0;
unsigned fpga_m7_stream_max_ray_steps = 0;
unsigned long long fpga_m7_stream_shadow_errors = 0;
// A2: walk-loop iterations and rays per lane; walk_iters above is the
// maximum over lanes (a lower bound on the walk cycles at II=1, not a
// prediction: the cycle model is tools/m7_lanes/lanes_model.cpp).
unsigned long long fpga_m7_stream_lane_iters[16] = {0};
unsigned long long fpga_m7_stream_lane_rays[16] = {0};
int fpga_m7_stream_lanes = FPGA_M7_LANES;
// G2 C-model test hook: if nonzero, the next frame stops after the ray walk
// (no write-out, no counts, planes left dirty), returns 4, and the hook is
// reset to 0. Models a frame cut off before its write-out completes.
int fpga_m7_stream_sim_abort = 0;
}
namespace {
#endif

// GEO_FX1 (DESIGN.md section 3.4, option C1): the float32 depth of the A53 rule,
// f32(code) * unit with IEEE round to nearest even, code 0 -> quiet NaN 0x7fc00000, computed
// exactly in integer form for the depth crc only (the geometry no longer uses it):
// code (16 bit) x significand of unit (24 bit) = one 40-bit product (one DSP48E2), normalised by
// the position of its leading one (bit 23 + sh, sh in [0, 16]), rounded to 24 bits (guard and
// sticky), exponent = exponent(unit) + sh (+1 on a rounding carry). Valid when unit is normal and
// every product is normal and finite (driver precondition, the D16 d16_unit_ok rule); then the
// bits equal static_cast<float>(code) * unit (host test GEO_FX1/tools/crc_mul_test.cpp, all
// 65536 codes x a set of units).
ap_uint<32> fx_code_unit_bits(ap_uint<16> code, ap_uint<32> ub) {
#pragma HLS INLINE
    const ap_uint<24> mu = (ap_uint<1>(1), ap_uint<23>(ub(22, 0)));
    const ap_uint<40> p = code * mu;
    ap_uint<5> sh = 0;
    for (int b = 1; b <= 16; ++b) {
#pragma HLS UNROLL
        if (p[23 + b]) sh = b;
    }
    const ap_uint<40> n = p << (16 - sh);   // leading one at bit 39
    const ap_uint<24> q = n(39, 16);
    const bool guard = n[15];
    const bool sticky = n(14, 0) != 0;
    const ap_uint<25> m = ap_uint<25>(q) + ap_uint<25>((guard && (sticky || q[0])) ? 1 : 0);
    const ap_uint<8> ex = ap_uint<8>(ub(30, 23)) + ap_uint<8>(sh) + ap_uint<8>(m[24]);
    ap_uint<32> bits;
    bits[31] = ub[31];
    bits(30, 23) = ex;
    bits(22, 0) = m[24] ? ap_uint<23>(0) : ap_uint<23>(m(22, 0));
#ifdef FPGA_M7_FAULT_D16_NONAN
    return code == 0 ? ap_uint<32>(0) : bits;
#else
    return code == 0 ? ap_uint<32>(0x7fc00000u) : bits;
#endif
}

// D16: zlib crc32 (reflected 0xEDB88320) update over the 4 little-endian
// bytes of one float32, i.e. the bytes of the float32 depth buffer the A53
// path hashed (host/kv_link.crc32 of the float32 depth array).
// 32 bit-serial steps c = (c >> 1) ^ (c & 1 ? poly : 0) are linear over GF(2),
// so the whole word step is one 32 x 32 matrix applied to crc ^ data: output
// bit j = parity((crc ^ data) & kCrcRow[j]) (12..17 taps per bit, a 2-level
// XOR tree). The first version unrolled the 32 serial steps as a mux chain:
// csynth estimated 5,179 LUT and -9.8 ns for the read16 loop (README).
// Rows generated and checked against zlib by RUN/D16 tools/crc_rows.py.
const std::uint32_t kCrcRow[32] = {
    0x04d101dfu, 0x09a203beu, 0x1344077du, 0x26880efau, 0x4d101df4u, 0x9a203be9u,
    0x3091760du, 0x6122ec1au, 0xc245d835u, 0x805ab1b5u, 0x046462b5u, 0x08c8c56au,
    0x11918ad4u, 0x232315a9u, 0x46462b53u, 0x8c8c56a6u, 0x1dc9ac92u, 0x3b935924u,
    0x7726b249u, 0xee4d6493u, 0xd84bc8f9u, 0xb446902du, 0x6c5c2184u, 0xd8b84309u,
    0xb5a187ccu, 0x6f920e46u, 0xdf241c8cu, 0xba9938c7u, 0x71e37051u, 0xe3c6e0a3u,
    0xc35cc098u, 0x826880efu};
ap_uint<32> d16_crc_step(ap_uint<32> crc, ap_uint<32> data) {
#pragma HLS INLINE
    const ap_uint<32> x = crc ^ data;
    ap_uint<32> c = 0;
    for (int j = 0; j < 32; ++j) {
#pragma HLS UNROLL
        c[j] = (x & ap_uint<32>(kCrcRow[j])).xor_reduce();
    }
    return c;
}

// D16 / GEO_FX1: 16-bit input only. depth_m holds width*height little-endian uint16 camera codes,
// two per 32-bit word, pixel 2w in bits [15:0] of word w, pixel 2w+1 in bits [31:16]; the buffer
// must be 4-byte aligned and readable up to the next multiple of 4 bytes (an odd pixel count
// reads the upper half of the last word and ignores it). II=2 per word = one pixel per cycle.
// The codes go to convert() as they are; the crc32 of the float32 depth f32(code) * unit is
// returned in depth_crc (the Nano's end-to-end depth check, unchanged).
void read_depth(const std::uint32_t* depth_m, std::uint32_t pixels, ap_uint<32> unit,
                hls::stream<ap_uint<16> >& out, std::uint32_t* crc_out) {
    ap_uint<32> crc = 0xFFFFFFFFu;
    const std::uint32_t words = (pixels + 1) >> 1;
read16_loop:
    for (std::uint32_t w = 0; w < words; ++w) {
#pragma HLS PIPELINE II=2
#pragma HLS LOOP_TRIPCOUNT min=0 max=512000 avg=115200
#pragma HLS ALLOCATION operation instances=mul limit=1
#if defined(__SYNTHESIS__) && defined(FPGA_M7_HWFAULT_D16_SWAP)
        const ap_uint<32> raw = depth_m[w];
        const ap_uint<32> word = (raw(15, 0), raw(31, 16));
#else
        const ap_uint<32> word = depth_m[w];
#endif
        out.write(word(15, 0));
        crc = d16_crc_step(crc, fx_code_unit_bits(word(15, 0), unit));
#ifndef FPGA_M7_FAULT_D16_ODDTAIL
        if (2 * w + 1 < pixels) {
#endif
            out.write(word(31, 16));
            crc = d16_crc_step(crc, fx_code_unit_bits(word(31, 16), unit));
#ifndef FPGA_M7_FAULT_D16_ODDTAIL
        }
#endif
    }
    *crc_out = (~crc).to_uint();
}

void convert(hls::stream<ap_uint<16> >& in, hls::stream<Ray>& rays,
             hls::stream<Counts>& counts_out, FxConst k,
             std::uint32_t width, std::uint32_t pixels,
             std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
             std::int32_t ofx, std::int32_t ofy, std::int32_t ofz,
             bool dlat, ap_uint<1> ev_pl[kCap]) {
    const std::uint32_t dims[3] = {nx, ny, nz};
    const std::int32_t from[3] = {ofx, ofy, ofz};
    // GEO_FX1: unpack the frame constants (wires). Row accumulator R_a(v) and pixel accumulator
    // P_a(u, v), both s4.36 in 41 bits, start at G0_a + 2^13 (the 2^13 makes the >> 14 below a
    // round-half-up to 22 fractional bits, DESIGN.md section 2.3). The sums are exact integers
    // (two's complement mod 2^41; inside the PS envelope |P| < 15.5 every value fits).
    acc_t gu[3], gv[3], racc[3], pacc[3];
    ap_int<36> oc[3];
#pragma HLS ARRAY_PARTITION variable=gu type=complete
#pragma HLS ARRAY_PARTITION variable=gv type=complete
#pragma HLS ARRAY_PARTITION variable=racc type=complete
#pragma HLS ARRAY_PARTITION variable=pacc type=complete
#pragma HLS ARRAY_PARTITION variable=oc type=complete
    for (int a = 0; a < 3; ++a) {
#pragma HLS UNROLL
        gu[a] = acc_t(k.gu(41 * a + 40, 41 * a));
        gv[a] = acc_t(k.gv(41 * a + 40, 41 * a));
        racc[a] = acc_t(k.g0(41 * a + 40, 41 * a)) + acc_t(1 << 13);
        pacc[a] = racc[a];
        oc[a] = ap_int<36>(k.o(36 * a + 35, 36 * a));
    }
    const ap_uint<16> c_lo = k.c_lo, c_hi = k.c_hi, c20 = k.c20;
    // Frame constants of the reference add_ray: start cell and the two
    // possible next[] values (direction > 0 / direction <= 0).
    std::int32_t next_pos[3], next_neg[3];
    for (int axis = 0; axis < 3; ++axis) {
        const std::int32_t cell = from[axis] / FPGA_M7_FRAME_Q8;
        next_pos[axis] = (cell + 1) * FPGA_M7_FRAME_Q8 - from[axis];
        next_neg[axis] = from[axis] - cell * FPGA_M7_FRAME_Q8;
    }
    std::uint32_t accepted = 0, invalid = 0, clipped = 0;
    std::uint32_t ucount = 0, vcount = 0;
    // GEO_FX1 statistics (DESIGN.md section 3.3)
    ap_uint<21> st_npos = 0, st_le20 = 0;
    ap_uint<16> st_maxle20 = 0, st_mingt20 = 0xffff, st_minvalid = 0xffff, st_maxcode = 0;
    bool st_any_gt20 = false, st_any_valid = false;
    // Dlat2 state (see the header). blk_mark needs no clear: see header.
    ap_uint<2> blk_mark[kBlkCols];
#pragma HLS BIND_STORAGE variable=blk_mark type=ram_s2p impl=lutram
    bool prev_in = false, prev_cl = false;  // previous pixel had an in-grid / clipped ray
    // forwarding history: [0] = last iteration
    ap_uint<15> fe_a[kFwd];   // ev_pl write addresses
    bool fe_v[kFwd];          // ... valid (an in-grid ray wrote ev_pl)
    ap_uint<10> fb_a[kFwd];   // blk_mark write addresses
    ap_uint<2> fb_d[kFwd];    // ... data
    bool fb_v[kFwd];          // ... valid (even-row pixel)
#pragma HLS ARRAY_PARTITION variable=fe_a type=complete
#pragma HLS ARRAY_PARTITION variable=fe_v type=complete
#pragma HLS ARRAY_PARTITION variable=fb_a type=complete
#pragma HLS ARRAY_PARTITION variable=fb_d type=complete
#pragma HLS ARRAY_PARTITION variable=fb_v type=complete
    for (int k = 0; k < kFwd; ++k) {
#pragma HLS UNROLL
        fe_a[k] = 0; fe_v[k] = false; fb_a[k] = 0; fb_d[k] = 0; fb_v[k] = false;
    }
convert_loop:
    for (std::uint32_t pixel = 0; pixel < pixels; ++pixel) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=0 max=1024000 avg=230400
        // Distances < kFwd are covered by explicit forwarding below.
#pragma HLS DEPENDENCE variable=ev_pl type=inter false
#pragma HLS DEPENDENCE variable=blk_mark type=inter false
        const ap_uint<16> code = in.read();
        const bool u_odd = (ucount & 1) != 0, v_odd = (vcount & 1) != 0;
        const ap_uint<10> bcol = ucount >> 1;
        // ---- GEO_FX1 conversion: one DSP multiply-add per axis ----
        //   E = P27 * code + O  (27 x 17 signed product + 36-bit C, exact in 44 bits)
        //   end = E >> 22       (floor, |end| < 2^21: exact in 22 bits)
        ap_int<22> end[3];
        for (int a = 0; a < 3; ++a) {
            const ap_int<27> p27 = ap_int<27>(pacc[a] >> 14);
            const ap_int<44> e = ap_int<44>(p27 * ap_int<18>(code)) + ap_int<44>(oc[a]);
            end[a] = ap_int<22>(e >> 22);
        }
        // pixel % width and pixel / width as counters; accumulators follow them
        const bool eol = ucount + 1 == width;
        for (int a = 0; a < 3; ++a) {
            const acc_t rn = racc[a] + gv[a];
            pacc[a] = eol ? rn : acc_t(pacc[a] + gu[a]);
            if (eol) racc[a] = rn;
        }
        if (eol) {
            ucount = 0;
            ++vcount;
        } else {
            ++ucount;
        }
        const bool valid = code >= c_lo && code <= c_hi;
        bool inside = true;
        for (int axis = 0; axis < 3; ++axis)
            if (end[axis] < 0 || end[axis] >= ap_int<22>(dims[axis] * FPGA_M7_FRAME_Q8))
                inside = false;
        // ---- GEO_FX1 statistics ----
        if (code != 0) {
            ++st_npos;
            if (code > st_maxcode) st_maxcode = code;
            if (code <= c20) {
                ++st_le20;
                if (code > st_maxle20) st_maxle20 = code;
            } else {
                st_any_gt20 = true;
                if (code < st_mingt20) st_mingt20 = code;
            }
        }
        if (valid) {
            st_any_valid = true;
            if (code < st_minvalid) st_minvalid = code;
        }
        const bool is_in = valid && inside, is_cl = valid && !inside;
        // ---- Dlat2: 2x2 block marks (separate for in-grid / clipped) ----
        ap_uint<2> above = 0;  // marks of the even row above (odd rows only)
        if (v_odd) {
            above = blk_mark[bcol];
#ifndef FPGA_M7_HWFAULT_DLAT_NOFWD_ACTIVE
            // forward the newest matching write (k = 0 is the newest)
            for (int k = kFwd - 1; k >= 0; --k)
                if (fb_v[k] && fb_a[k] == bcol) above = fb_d[k];
#endif
        }
        const bool in_seen = above[0] || (u_odd && prev_in);
        const bool cl_seen = above[1] || (u_odd && prev_cl);
        ap_uint<2> mark;
        mark[0] = in_seen || is_in;
        mark[1] = cl_seen || is_cl;
        if (!v_odd) blk_mark[bcol] = mark;
        // ---- Dlat2: endpoint-voxel marks (in-grid rays only) ----
        // Endpoint voxel index, also the HIT index r.tidx. Only used when
        // inside: then 0 <= end < dims*256 <= 8192, so floor_q8 = end >> 8 in
        // [0, 31] and the reference grid_offset (uint32) is < 32768; the same
        // value in exact 5/6/10/15-bit arithmetic (no 32-bit multipliers).
        const ap_uint<5> ex = end[0] >> 8, ey = end[1] >> 8, ez = end[2] >> 8;
        const ap_uint<6> nx6 = nx, ny6 = ny;
        const ap_uint<15> eidx = inside
            ? ap_uint<15>((ap_uint<10>)(ez * ny6 + ey) * nx6 + ex) : ap_uint<15>(0);
        bool ev_seen = false;
        if (is_in) {
            ev_seen = ev_pl[eidx] != 0;
#ifndef FPGA_M7_HWFAULT_DLAT_NOFWD_ACTIVE
            for (int k = 0; k < kFwd; ++k)
                if (fe_v[k] && fe_a[k] == eidx) ev_seen = true;
#endif
            ev_pl[eidx] = 1;
        }
        for (int k = kFwd - 1; k > 0; --k) {
            fe_a[k] = fe_a[k - 1]; fe_v[k] = fe_v[k - 1];
            fb_a[k] = fb_a[k - 1]; fb_d[k] = fb_d[k - 1]; fb_v[k] = fb_v[k - 1];
        }
        fe_a[0] = eidx; fe_v[0] = is_in;
        fb_a[0] = bcol; fb_d[0] = mark; fb_v[0] = !v_odd;
        prev_in = is_in;
        prev_cl = is_cl;
#ifdef FPGA_M7_FAULT_DLAT_NOEP
        const bool keep_in = !in_seen;
#else
        const bool keep_in = !in_seen || !ev_seen;
#endif
        const bool keep = !dlat || (inside ? keep_in : !cl_seen);
        // ---- end of Dlat2 ----
        // G1d opt B: no early 'continue'. The ray record below is computed
        // for every pixel (pure function of end[], speculative) and only the
        // stream write is gated, so the cross-product DSPs are not scheduled
        // behind the late Dlat2 decision (BRAM read of ev_pl).
        if (!valid)
            ++invalid;
        else if (inside)
            ++accepted;
        else
            ++clipped;
#ifdef FPGA_M7_FAULT_NOCLIP
        const bool emit = valid && keep && inside;
#else
        const bool emit = valid && keep;
#endif
        // Ray record: the reference add_ray set-up, precomputed.
        std::int32_t target[3], next[3], delta[3], dir[3];
        cell_t tclamp[3];
        for (int axis = 0; axis < 3; ++axis) {
            const std::int32_t to = end[axis].to_int();
            const std::int32_t fr = from[axis];
#ifdef FPGA_M7_FAULT_TRUNC
            target[axis] = to / FPGA_M7_FRAME_Q8;
#else
            target[axis] = (end[axis] >> 8).to_int();  // arithmetic shift = floor
#endif
            dir[axis] = to > fr ? 1 : (to < fr ? -1 : 0);
            delta[axis] = to > fr ? to - fr : fr - to;
#ifdef FPGA_M7_FAULT_NEXT
            next[axis] = dir[axis] != 0 ? next_pos[axis] : next_neg[axis];
#else
            next[axis] = dir[axis] > 0 ? next_pos[axis] : next_neg[axis];
#endif
            const std::int32_t t = target[axis];
            tclamp[axis] = t < -1 ? cell_t(-1) : (t > 32 ? cell_t(32) : cell_t(t));
        }
        Ray r;
        const ap_uint<9> n0 = next[0], n1 = next[1], n2 = next[2];
        const dlt_t e0 = delta[0], e1 = delta[1], e2 = delta[2];
        r.pxy = cross_t(n0 * e1) - cross_t(n1 * e0);
        r.pxz = cross_t(n0 * e2) - cross_t(n2 * e0);
        r.pyz = cross_t(n1 * e2) - cross_t(n2 * e1);
        r.dx = e0;
        r.dy = e1;
        r.dz = e2;
        r.tx = tclamp[0];
        r.ty = tclamp[1];
        r.tz = tclamp[2];
        r.sx = dir[0];
        r.sy = dir[1];
        r.sz = dir[2];
        r.tin = inside;
        r.tidx = eidx;  // == inside ? grid_offset(target, nx, ny) : 0
#ifdef FPGA_M7_FAULT_CLIPHIT
        if (!inside) {
            std::int32_t c[3];
            for (int axis = 0; axis < 3; ++axis) {
                const std::int32_t hi = static_cast<std::int32_t>(dims[axis]) - 1;
                c[axis] = target[axis] < 0 ? 0 : (target[axis] > hi ? hi : target[axis]);
            }
            r.tin = true;
            r.tidx = grid_offset(c, nx, ny);
        }
#endif
        r.last = false;
#ifndef __SYNTHESIS__
        if (emit &&
            (delta[0] >= (1 << 18) || delta[1] >= (1 << 18) || delta[2] >= (1 << 18) ||
             next[0] < 0 || next[0] > 256 || next[1] < 0 || next[1] > 256 ||
             next[2] < 0 || next[2] > 256))
            ++fpga_m7_stream_shadow_errors;
#endif
        if (emit) rays.write(r);
    }
    Ray sentinel;
    sentinel.pxy = 0; sentinel.pxz = 0; sentinel.pyz = 0;
    sentinel.dx = 0; sentinel.dy = 0; sentinel.dz = 0;
    sentinel.tx = 0; sentinel.ty = 0; sentinel.tz = 0;
    sentinel.sx = 0; sentinel.sy = 0; sentinel.sz = 0;
    sentinel.tidx = 0;
    sentinel.tin = false;
    sentinel.last = true;
    rays.write(sentinel);
    Counts c;
    c.accepted = accepted;
    c.invalid = invalid;
    c.clipped = clipped;
    c.npos = st_npos;
    c.le20 = st_le20;
    c.mm0 = (ap_uint<32>(st_any_gt20 ? st_mingt20 : ap_uint<16>(0)) << 16) | ap_uint<32>(st_maxle20);
    c.mm1 = (ap_uint<32>(st_maxcode) << 16) | ap_uint<32>(st_any_valid ? st_minvalid : ap_uint<16>(0));
    counts_out.write(c);
}

// A2 dispatcher: round robin over the lanes, blocking on the target lane's
// FIFO; then one end-of-frame record per lane; then the only reader of the
// counts stream writes the three counts.
void dispatch(hls::stream<Ray>& rays, hls::stream<Ray> lane_s[kLanes],
              hls::stream<Counts>& counts_in, std::uint32_t* accepted,
              std::uint32_t* invalid, std::uint32_t* clipped, std::uint32_t* stats) {
#ifndef __SYNTHESIS__
    fpga_m7_stream_walk_rays = 0;
    fpga_m7_stream_walk_steps = 0;
    fpga_m7_stream_max_ray_steps = 0;
#endif
    ap_uint<4> k = 0;
dispatch_loop:
    for (;;) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=1 max=1024000 avg=230400
        const Ray r = rays.read();
        if (r.last) break;
#if defined(__SYNTHESIS__) && defined(FPGA_M7_HWFAULT_DROP_FULL)
        if (!lane_s[k].full()) lane_s[k].write(r);
#else
        lane_s[k].write(r);
#endif
        k = (k == kLanes - 1) ? ap_uint<4>(0) : ap_uint<4>(k + 1);
    }
    Ray s;
    s.pxy = 0; s.pxz = 0; s.pyz = 0;
    s.dx = 0; s.dy = 0; s.dz = 0;
    s.tx = 0; s.ty = 0; s.tz = 0;
    s.sx = 0; s.sy = 0; s.sz = 0;
    s.tidx = 0;
    s.tin = false;
    s.last = true;
end_loop:
    for (int i = 0; i < kLanes; ++i) {
#pragma HLS UNROLL
#ifdef FPGA_M7_FAULT_LANE_NOEND
        if (kLanes > 1 && i == kLanes - 1) continue;
#endif
        lane_s[i].write(s);  // every lane needs its own end-of-frame record
    }
    const Counts c = counts_in.read();
    *accepted = c.accepted;
    *invalid = c.invalid;
    *clipped = c.clipped;
    stats[0] = c.npos;
    stats[1] = c.le20;
    stats[2] = c.mm0;
    stats[3] = c.mm1;
}

// GEO_SLIM3 item 7: a one-cycle register (same construction as hls::reg of
// utils/x_hls_utils.h, kept local to avoid that header's global names).
template <typename T>
#ifdef __SYNTHESIS__
__attribute__((hls_preserve))
#endif
T walk_reg(T d) {
#pragma HLS PIPELINE II=1
#pragma HLS LATENCY min=1 max=1
#pragma HLS INLINE off
    return d;
}

// One lane = the A1 walk loop on its own record FIFO and its own planes.
//
// GEO_SLIM3 item 7 (timing): in the A1 loop the record FIFO was read with
// "load = !busy", and busy is this iteration's end test (target / grid /
// step-count compares, 4 carry chains). Because II=1 overlaps the next
// iteration's read with this iteration's end test, the end test drove the
// lane FIFO read enable and, through the loop exit, the DATAFLOW done
// handshake (MERGE3 worst paths, 14-15 levels: SLIM_AUDIT2 section 2.1).
// Now:
//  - prefetch register nr/nv: the FIFO is read (non-blocking) only when nr
//    is empty and the end-of-frame record has not been taken; both are
//    registers at the start of the iteration, so the read enable no longer
//    depends on the end test. A ray is always loaded from nr (no bypass of
//    the FIFO output into the walk datapath: a bypass mux cost ~190 LUT per
//    lane in GEO_SLIM3 s7); nr refills while the current ray walks, so a ray
//    waits one extra cycle only when the lane was idle or the previous ray
//    ended in its first iteration (C model: +1 iteration per frame on all
//    five datasets);
//  - delayed exit: the loop leaves one iteration after the end-of-frame
//    record is taken (quit <- stop <- fin), so the exit test is a register;
//  - the bit-plane writes go through walk_reg (one cycle later, same
//    address and value; the planes are only written here, and the pipeline
//    drains before the loop finishes).
// The marks written, and so the flags, are unchanged; per frame the lane
// takes a few more cycles (first record via nr, the quit iteration, one
// more pipeline stage) and may stall the dispatcher less (one more record
// of buffering per lane).
template <int ID>
void walk_lane(hls::stream<Ray>& rays, ap_uint<1> free_pl[kCap],
               ap_uint<1> hit_pl[kCap], std::int32_t ofx, std::int32_t ofy,
               std::int32_t ofz, std::uint32_t nx, std::uint32_t ny,
               std::uint32_t nz) {
#pragma HLS INLINE off
    const cell_t ox = ofx / FPGA_M7_FRAME_Q8;  // from >= 0: truncation == floor
    const cell_t oy = ofy / FPGA_M7_FRAME_Q8;
    const cell_t oz = ofz / FPGA_M7_FRAME_Q8;
    const cell_t dimx = nx, dimy = ny, dimz = nz;
    const ap_uint<6> nx6 = nx, ny6 = ny;
    cell_t cx = 0, cy = 0, cz = 0, tx = 0, ty = 0, tz = 0;
    ap_int<2> sx = 0, sy = 0, sz = 0;
    cross_t pxy = 0, pxz = 0, pyz = 0;
    dlt_t dx = 0, dy = 0, dz = 0;
    ap_uint<15> tidx = 0;
    bool tin = false;
    ap_uint<7> steps = 0;
    bool busy = false, stop = false, quit = false;
    bool nv = false;  // prefetch register nr holds a record
#ifndef __SYNTHESIS__
    fpga_m7_stream_lane_iters[ID] = 0;
    fpga_m7_stream_lane_rays[ID] = 0;
    // int64 shadow of the reference's next[] and delta[] for the cross check
    long long shn[3] = {0, 0, 0}, shd[3] = {0, 0, 0};
#endif
    // Latch-free form (2026-10-11 gate): every loop-carried variable is read
    // once through an explicit "load ? new ray : carried" select and written
    // back unconditionally at the end of the iteration, so no state variable
    // is assigned on only some paths (Vitis HLS 2022.2 otherwise emits
    // 'bx-default combinational loads that Vivado synthesizes as LDCE).
    Ray zero;
    zero.pxy = 0; zero.pxz = 0; zero.pyz = 0;
    zero.dx = 0; zero.dy = 0; zero.dz = 0;
    zero.tx = 0; zero.ty = 0; zero.tz = 0;
    zero.sx = 0; zero.sy = 0; zero.sz = 0;
    zero.tidx = 0;
    zero.tin = false;
    zero.last = false;
    Ray nr = zero;
    // GEO_FX2 X1: one clock boundary before the loop keeps walk_loop in its own pipelined
    // sub-module (as in m4g, where the int32 ofx / 256 preamble did this); without it Vitis HLS
    // pipelines the loop in place (flow_control_loop_delay_pipe, an ap_loop_init select on every
    // carried state bit) and each lane is ~390 LUT larger in OOC at 6 ns (896 vs 503 LUT, GEO_FX1
    // README section 3.3). No effect in C; check the RTL for delay_pipe after a tool upgrade.
    ap_wait();
walk_loop:
    while (!quit) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=1 max=3000000 avg=2300000
        // FIFO -> prefetch register; the condition is registered state only
        Ray in = zero;
        bool rd = false;
        if (!nv && !stop) rd = rays.read_nb(in);
        const Ray r = nr;  // next record (no bypass: a record is used the iteration after it is read)
        const bool load = !busy && nv;
#ifndef __SYNTHESIS__
        // every iteration before the quit iteration, bubbles included
        if (!stop) ++fpga_m7_stream_lane_iters[ID];
#endif
        const bool fin = load && r.last;
        const bool live = (busy || load) && !fin;
        // current ray state: fresh from the record, or carried
        const cell_t c0x = load ? ox : cx, c0y = load ? oy : cy, c0z = load ? oz : cz;
        const cell_t t0x = load ? r.tx : tx, t0y = load ? r.ty : ty, t0z = load ? r.tz : tz;
        const ap_int<2> s0x = load ? r.sx : sx, s0y = load ? r.sy : sy, s0z = load ? r.sz : sz;
        const cross_t p0xy = load ? r.pxy : pxy, p0xz = load ? r.pxz : pxz, p0yz = load ? r.pyz : pyz;
        const dlt_t d0x = load ? r.dx : dx, d0y = load ? r.dy : dy, d0z = load ? r.dz : dz;
        const ap_uint<15> tidx0 = load ? r.tidx : tidx;
        const bool tin0 = load ? r.tin : tin;
        const ap_uint<7> steps0 = load ? ap_uint<7>(0) : steps;
#ifndef __SYNTHESIS__
        if (load && !fin) {
            ++fpga_m7_stream_walk_rays;
            ++fpga_m7_stream_lane_rays[ID];
            shd[0] = (long long)r.dx.to_uint(); shd[1] = (long long)r.dy.to_uint();
            shd[2] = (long long)r.dz.to_uint();
            // recover next0 from the frame constants (same as convert)
            const std::int32_t fr[3] = {ofx, ofy, ofz};
            const int sg[3] = {r.sx.to_int(), r.sy.to_int(), r.sz.to_int()};
            for (int a = 0; a < 3; ++a) {
                const std::int32_t cell = fr[a] / FPGA_M7_FRAME_Q8;
                shn[a] = sg[a] > 0 ? (cell + 1) * FPGA_M7_FRAME_Q8 - fr[a]
                                   : fr[a] - cell * FPGA_M7_FRAME_Q8;
            }
        }
#endif
        const bool ax = live && s0x != 0 && c0x != t0x;
        const bool ay = live && s0y != 0 && c0y != t0y;
        const bool az = live && s0z != 0 && c0z != t0z;
        const bool any = ax || ay || az;
        const bool xy_le = p0xy <= 0, xz_le = p0xz <= 0, yz_le = p0yz <= 0;
        const bool xz_ge = p0xz >= 0, yz_ge = p0yz >= 0;
#ifdef FPGA_M7_FAULT_TIE
        const bool xy_ge_y = p0xy > 0;
#else
        const bool xy_ge_y = p0xy >= 0;
#endif
        const bool stx = ax && (!ay || xy_le) && (!az || xz_le);
        const bool sty = ay && (!ax || xy_ge_y) && (!az || yz_le);
        const bool stz = az && (!ax || xz_ge) && (!ay || yz_ge);
        const cross_t n_xy = p0xy + (stx ? d256(d0y) : cross_t(0)) - (sty ? d256(d0x) : cross_t(0));
        const cross_t n_xz = p0xz + (stx ? d256(d0z) : cross_t(0)) - (stz ? d256(d0x) : cross_t(0));
        const cross_t n_yz = p0yz + (sty ? d256(d0z) : cross_t(0)) - (stz ? d256(d0y) : cross_t(0));
        const cell_t c1x = stx ? cell_t(c0x + s0x) : c0x;
        const cell_t c1y = sty ? cell_t(c0y + s0y) : c0y;
        const cell_t c1z = stz ? cell_t(c0z + s0z) : c0z;
        const ap_uint<7> steps1 = any ? ap_uint<7>(steps0 + 1) : steps0;
        const bool in_grid = c1x >= 0 && c1x < dimx && c1y >= 0 && c1y < dimy &&
                             c1z >= 0 && c1z < dimz;
        const bool at_tgt = c1x == t0x && c1y == t0y && c1z == t0z;
        const bool mark_free = any && in_grid && !at_tgt;
        const bool end = live && (!any || !in_grid || at_tgt || steps1 == kMaxSteps);
#ifndef __SYNTHESIS__
        if (any) {
            ++fpga_m7_stream_walk_steps;
            if (stx) shn[0] += FPGA_M7_FRAME_Q8;
            if (sty) shn[1] += FPGA_M7_FRAME_Q8;
            if (stz) shn[2] += FPGA_M7_FRAME_Q8;
            if ((long long)n_xy.to_int64() != shn[0] * shd[1] - shn[1] * shd[0] ||
                (long long)n_xz.to_int64() != shn[0] * shd[2] - shn[2] * shd[0] ||
                (long long)n_yz.to_int64() != shn[1] * shd[2] - shn[2] * shd[1])
                ++fpga_m7_stream_shadow_errors;
            if (steps1.to_uint() > fpga_m7_stream_max_ray_steps)
                fpga_m7_stream_max_ray_steps = steps1.to_uint();
        }
#endif
        const ap_uint<5> ux = c1x, uy = c1y, uz = c1z;
        const ap_uint<15> idx = (ap_uint<15>)((ap_uint<10>)(uz * ny6 + uy) * nx6 + ux);
        ap_uint<16> fw, hw;
        fw[15] = mark_free;
        fw(14, 0) = idx;
        hw[15] = end && tin0;
        hw(14, 0) = tidx0;
        fw = walk_reg(fw);
        hw = walk_reg(hw);
        if (fw[15]) free_pl[fw(14, 0)] = 1;
        if (hw[15]) hit_pl[hw(14, 0)] = 1;
        // unconditional write-back of the whole state
        cx = c1x; cy = c1y; cz = c1z;
        tx = t0x; ty = t0y; tz = t0z;
        sx = s0x; sy = s0y; sz = s0z;
        pxy = n_xy; pxz = n_xz; pyz = n_yz;
        dx = d0x; dy = d0y; dz = d0z;
        tidx = tidx0;
        tin = tin0;
        steps = steps1;
        busy = live && !end;
        if (!nv) nr = in;
        nv = (nv && !load) || rd;
        quit = stop;
        stop = stop || fin;
    }
}

void frame_rays(const std::uint32_t* depth_m, FxConst k, std::uint32_t width,
                std::uint32_t pixels, std::uint32_t nx, std::uint32_t ny,
                std::uint32_t nz, std::int32_t ofx, std::int32_t ofy,
                std::int32_t ofz, ap_uint<1> free_pl[kLanes][kCap],
                ap_uint<1> hit_pl[kLanes][kCap], bool dlat, ap_uint<1> ev_pl[kCap],
                ap_uint<32> unit, std::uint32_t* depth_crc,
                std::uint32_t* accepted,
                std::uint32_t* invalid, std::uint32_t* clipped, std::uint32_t* stats) {
#pragma HLS DATAFLOW
    // G3 step 3: every scalar input of this region is a frame constant: the
    // top computes it before the call and does not change it until the
    // region has finished (one call per frame). STABLE drops the per-process
    // copies (one depth-2 channel per scalar per consumer, 81 in all).
#pragma HLS STABLE variable=k
#pragma HLS STABLE variable=width
#pragma HLS STABLE variable=pixels
#pragma HLS STABLE variable=nx
#pragma HLS STABLE variable=ny
#pragma HLS STABLE variable=nz
#pragma HLS STABLE variable=ofx
#pragma HLS STABLE variable=ofy
#pragma HLS STABLE variable=ofz
#pragma HLS STABLE variable=dlat
#pragma HLS STABLE variable=unit
    hls::stream<ap_uint<16> > s_depth("s_depth");
    hls::stream<Ray> s_ray("s_ray");
    hls::stream<Counts> s_counts("s_counts");
    hls::stream<Ray> lane_s[kLanes];
#pragma HLS STREAM variable=s_depth depth=64
#pragma HLS STREAM variable=s_ray depth=512
#pragma HLS STREAM variable=s_counts depth=2
#pragma HLS STREAM variable=lane_s depth=4
    // 200 bits x 4 (W1; 275 before): without this HLS puts every lane FIFO in 16 RAMB18
    // (design doc section 5.3).
#pragma HLS BIND_STORAGE variable=lane_s type=fifo impl=srl
    // C simulation runs these in call order: read -> convert -> dispatch ->
    // lane 0 .. lane N-1 (lane FIFOs are unbounded in C simulation).
    read_depth(depth_m, pixels, unit, s_depth, depth_crc);
    convert(s_depth, s_ray, s_counts, k, width, pixels, nx, ny, nz,
            ofx, ofy, ofz, dlat, ev_pl);
    dispatch(s_ray, lane_s, s_counts, accepted, invalid, clipped, stats);
    // One explicit call per lane (a loop of process calls inside DATAFLOW is
    // not reliably unrolled by Vitis HLS 2022.2). Each lane gets only its own
    // FIFO and its own two planes.
#define FPGA_M7_LANE(i) \
    walk_lane<i>(lane_s[i], free_pl[i], hit_pl[i], ofx, ofy, ofz, nx, ny, nz);
#if defined(__SYNTHESIS__) && defined(FPGA_M7_HWFAULT_LANE_ALIAS) && FPGA_M7_LANES > 1
#define FPGA_M7_LANE_LAST(i) \
    walk_lane<i>(lane_s[i], free_pl[i - 1], hit_pl[i - 1], ofx, ofy, ofz, nx, ny, nz);
#elif defined(__SYNTHESIS__) && defined(FPGA_M7_HWFAULT_PLANE_SWAP)
#define FPGA_M7_LANE_LAST(i) \
    walk_lane<i>(lane_s[i], hit_pl[i], free_pl[i], ofx, ofy, ofz, nx, ny, nz);
#else
#define FPGA_M7_LANE_LAST(i) FPGA_M7_LANE(i)
#endif
#if FPGA_M7_LANES > 0
#if FPGA_M7_LANES == 1
    FPGA_M7_LANE_LAST(0)
#else
    FPGA_M7_LANE(0)
#endif
#endif
#if FPGA_M7_LANES > 1
#if FPGA_M7_LANES == 2
    FPGA_M7_LANE_LAST(1)
#else
    FPGA_M7_LANE(1)
#endif
#endif
#if FPGA_M7_LANES > 2
#if FPGA_M7_LANES == 3
    FPGA_M7_LANE_LAST(2)
#else
    FPGA_M7_LANE(2)
#endif
#endif
#if FPGA_M7_LANES > 3
#if FPGA_M7_LANES == 4
    FPGA_M7_LANE_LAST(3)
#else
    FPGA_M7_LANE(3)
#endif
#endif
#if FPGA_M7_LANES > 4
#if FPGA_M7_LANES == 5
    FPGA_M7_LANE_LAST(4)
#else
    FPGA_M7_LANE(4)
#endif
#endif
#if FPGA_M7_LANES > 5
#if FPGA_M7_LANES == 6
    FPGA_M7_LANE_LAST(5)
#else
    FPGA_M7_LANE(5)
#endif
#endif
#if FPGA_M7_LANES > 6
#if FPGA_M7_LANES == 7
    FPGA_M7_LANE_LAST(6)
#else
    FPGA_M7_LANE(6)
#endif
#endif
#if FPGA_M7_LANES > 7
#if FPGA_M7_LANES == 8
    FPGA_M7_LANE_LAST(7)
#else
    FPGA_M7_LANE(7)
#endif
#endif
#if FPGA_M7_LANES > 8
#if FPGA_M7_LANES == 9
    FPGA_M7_LANE_LAST(8)
#else
    FPGA_M7_LANE(8)
#endif
#endif
#if FPGA_M7_LANES > 9
#if FPGA_M7_LANES == 10
    FPGA_M7_LANE_LAST(9)
#else
    FPGA_M7_LANE(9)
#endif
#endif
#if FPGA_M7_LANES > 10
#if FPGA_M7_LANES == 11
    FPGA_M7_LANE_LAST(10)
#else
    FPGA_M7_LANE(10)
#endif
#endif
#if FPGA_M7_LANES > 11
#if FPGA_M7_LANES == 12
    FPGA_M7_LANE_LAST(11)
#else
    FPGA_M7_LANE(11)
#endif
#endif
#if FPGA_M7_LANES > 12
#if FPGA_M7_LANES == 13
    FPGA_M7_LANE_LAST(12)
#else
    FPGA_M7_LANE(12)
#endif
#endif
#if FPGA_M7_LANES > 13
#if FPGA_M7_LANES == 14
    FPGA_M7_LANE_LAST(13)
#else
    FPGA_M7_LANE(13)
#endif
#endif
#if FPGA_M7_LANES > 14
#if FPGA_M7_LANES == 15
    FPGA_M7_LANE_LAST(14)
#else
    FPGA_M7_LANE(14)
#endif
#endif
#if FPGA_M7_LANES > 15
#if FPGA_M7_LANES == 16
    FPGA_M7_LANE_LAST(15)
#else
    FPGA_M7_LANE(15)
#endif
#endif
#undef FPGA_M7_LANE
#undef FPGA_M7_LANE_LAST
#ifndef __SYNTHESIS__
    // C model: the walk-loop iteration count of the busiest lane.
    fpga_m7_stream_walk_iters = 0;
    for (int k = 0; k < kLanes; ++k)
        if (fpga_m7_stream_lane_iters[k] > fpga_m7_stream_walk_iters)
            fpga_m7_stream_walk_iters = fpga_m7_stream_lane_iters[k];
#endif
}

// WB_BURST (2026-11-02): packed burst write-out. Before, write_loop stored
// one std::uint8_t per cell through the m_axi port: csynth listed it under
// "Could not widen ... i8 alignment 1" (HLS 214-307) and NOT in the Inferred
// Burst Summary, so the RTL issued 32768 single-beat AXI writes (AWLEN 1,
// byte strobe), each waiting for its write response. Probes (WB_BURST/README):
// the same byte loop in its own function, or behind a stream, still did not
// burst while depth_m reads share bundle gmem; it burst only on a separate
// bundle (block-design change) or with a 32-bit element type.
// Now: merge_planes (II=1, one cell per cycle, unchanged A1 merge rule and G2
// write-back clear) -> pack_flags (4 consecutive cells -> one little-endian
// word, byte k of word j = cell 4j+k) -> store_flags (8192 words, II=1, one
// inferred write burst of 8192 x 32 bit). The 32768 bytes in memory are the
// same bytes as before at the same addresses PROVIDED the flags base address
// is 4-byte aligned; kCap = 32768 is a multiple of 4, so there is no partial
// tail word. Every caller passes a page-aligned DMA buffer (README section 3).
void merge_planes(ap_uint<1> free_pl[kLanes][kCap], ap_uint<1> hit_pl[kLanes][kCap],
                  ap_uint<1> ev_pl[kCap], hls::stream<std::uint8_t>& fs) {
merge_loop:
    for (std::uint32_t i = 0; i < kCap; ++i) {
#pragma HLS PIPELINE II=1
        // Iteration i reads and clears only address i of each plane: read on
        // one port, write 0 on the other; no carried dependence.
#pragma HLS DEPENDENCE variable=free_pl type=inter false
#pragma HLS DEPENDENCE variable=hit_pl type=inter false
#ifndef FPGA_M7_FAULT_DLAT_NOEVCLEAR
        ev_pl[i] = 0;  // G1d: Dlat2 endpoint marks, same write-back clear
#endif
        // OR over the lanes, then the A1 rule (HIT has priority).
        bool any_free = false, any_hit = false;
        for (int k = 0; k < kLanes; ++k) {
#pragma HLS UNROLL
            const bool f = free_pl[k][i] != 0;
            const bool h = hit_pl[k][i] != 0;
#ifdef FPGA_M7_FAULT_LANE_NOCLEAR
            if (!(kLanes > 1 && k == kLanes - 1))
#endif
            {
                free_pl[k][i] = 0;  // G2 write-back clear
                hit_pl[k][i] = 0;
            }
#if defined(FPGA_M7_FAULT_LANE_SKIP) || \
    (defined(__SYNTHESIS__) && defined(FPGA_M7_HWFAULT_LANE_SKIP))
            if (kLanes > 1 && k == kLanes - 1) continue;
#endif
            any_free = any_free || f;
            any_hit = any_hit || h;
        }
#ifdef FPGA_M7_FAULT_MERGE
        fs.write(any_free ? 1 : (any_hit ? 2 : 0));
#else
        fs.write(any_hit ? 2 : (any_free ? 1 : 0));
#endif
    }
}

void pack_flags(hls::stream<std::uint8_t>& fs, hls::stream<std::uint32_t>& ws) {
    std::uint32_t acc = 0;
pack_loop:
    for (std::uint32_t j = 0; j < kCap; ++j) {
#pragma HLS PIPELINE II=1
        const std::uint32_t b = fs.read();
        const std::uint32_t lane = j & 3u;
        acc = (lane == 0 ? 0u : acc) | (b << (8 * lane));
        if (lane == 3) ws.write(acc);
    }
}

void store_flags(hls::stream<std::uint32_t>& ws, std::uint32_t* flags) {
write_loop:
    for (std::uint32_t i = 0; i < kCap / 4; ++i) {
#pragma HLS PIPELINE II=1
        flags[i] = ws.read();
    }
}

void write_out(std::uint32_t* flags, ap_uint<1> free_pl[kLanes][kCap],
               ap_uint<1> hit_pl[kLanes][kCap], ap_uint<1> ev_pl[kCap]) {
#pragma HLS DATAFLOW
    hls::stream<std::uint8_t> fs("flag_stream");
    hls::stream<std::uint32_t> ws("flag_word_stream");
#pragma HLS STREAM variable=fs depth=16
#pragma HLS STREAM variable=ws depth=16
    merge_planes(free_pl, hit_pl, ev_pl, fs);
    pack_flags(fs, ws);
    store_flags(ws, flags);
}

}  // namespace

// GEO_FX1 register map: every argument up to depth_crc keeps its order, so every old register keeps
// its offset (checked against the generated hw.h). inverse_k, world_camera, grid_origin_*,
// voxel_size_m and depth_unit's float use in geometry are gone: the first five are declared and
// never read (DESIGN.md section 3.1, "only append"); depth_unit now feeds only the crc multiplier.
// Appended (DESIGN.md sections 2.2 and 3.3):
//   fx_k[24]                   GEO_FX3: Gu x/y/z, Gv, G0 (41-bit s4.36), O x/y/z (36-bit s13.22) as
//                              12 two's complement 64-bit slots (low word first), one s_axilite
//                              array (hw/m7/frame_fx.hpp fpga_m7_fx_pack); last argument
//   fx_origin_*                Q8 origin of the reference formula (PS), in [0, dims * 256)
//   fx_codes                   c_lo [15:0] | c_hi [31:16]; valid <=> c_lo <= code <= c_hi
//   fx_c20                     [15:0] max code with f32(code) * unit <= 20 m
//   st_npos / st_le20 / st_mm0 / st_mm1   read-only statistics, see convert()
// Status: 1 bad dimensions (as before), 6 depth_fmt bit 0 != 1 (float input removed). 2 / 3 / 5
// are decided by the PS before the call. Every status != 0 leaves flags untouched and every
// count / crc / statistics register 0.
extern "C" std::uint32_t fpga_m7_frame_axi(
    const std::uint32_t* depth_m, std::uint32_t* flags,
    const float inverse_k[9], const float world_camera[12],
    double grid_origin_x, double grid_origin_y, double grid_origin_z,
    double voxel_size_m, std::uint32_t width, std::uint32_t height,
    std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
    std::uint32_t* count_accepted, std::uint32_t* count_invalid,
    std::uint32_t* count_outside, std::uint32_t dlat2_en,
    std::uint32_t depth_fmt, float depth_unit, std::uint32_t* depth_crc,
    ap_uint<13> fx_origin_x, ap_uint<13> fx_origin_y, ap_uint<13> fx_origin_z,
    std::uint32_t fx_codes, ap_uint<16> fx_c20,
    ap_uint<21>* st_npos, ap_uint<21>* st_le20, std::uint32_t* st_mm0, std::uint32_t* st_mm1,
    const std::uint32_t fx_k[FPGA_M7_FX_K_WORDS]) {
// GEO_FX1 item E: the read side needs 0.5 word / cycle (II=2 per word), 8 outstanding bursts of
// 16 still cover ~256 cycles of read latency; the flag write-out is one long burst (4 outstanding).
#pragma HLS INTERFACE mode=m_axi port=depth_m offset=slave bundle=gmem depth=512000 max_widen_bitwidth=32 num_read_outstanding=8 max_read_burst_length=16 num_write_outstanding=4 max_write_burst_length=16
#pragma HLS INTERFACE mode=m_axi port=flags offset=slave bundle=gmem depth=8192 max_widen_bitwidth=32 num_read_outstanding=8 max_read_burst_length=16 num_write_outstanding=4 max_write_burst_length=16
#pragma HLS INTERFACE mode=s_axilite port=depth_m bundle=control
#pragma HLS INTERFACE mode=s_axilite port=flags bundle=control
#pragma HLS INTERFACE mode=s_axilite port=inverse_k bundle=control
#pragma HLS INTERFACE mode=s_axilite port=world_camera bundle=control
#pragma HLS INTERFACE mode=s_axilite port=grid_origin_x bundle=control
#pragma HLS INTERFACE mode=s_axilite port=grid_origin_y bundle=control
#pragma HLS INTERFACE mode=s_axilite port=grid_origin_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=voxel_size_m bundle=control
#pragma HLS INTERFACE mode=s_axilite port=width bundle=control
#pragma HLS INTERFACE mode=s_axilite port=height bundle=control
#pragma HLS INTERFACE mode=s_axilite port=nx bundle=control
#pragma HLS INTERFACE mode=s_axilite port=ny bundle=control
#pragma HLS INTERFACE mode=s_axilite port=nz bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_accepted bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_invalid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_outside bundle=control
#pragma HLS INTERFACE mode=s_axilite port=dlat2_en bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_fmt bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_unit bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_crc bundle=control
#pragma HLS INTERFACE mode=s_axilite port=fx_origin_x bundle=control
#pragma HLS INTERFACE mode=s_axilite port=fx_origin_y bundle=control
#pragma HLS INTERFACE mode=s_axilite port=fx_origin_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=fx_codes bundle=control
#pragma HLS INTERFACE mode=s_axilite port=fx_c20 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=st_npos bundle=control
#pragma HLS INTERFACE mode=s_axilite port=st_le20 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=st_mm0 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=st_mm1 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=fx_k bundle=control
#pragma HLS INTERFACE mode=s_axilite port=return bundle=control
    *count_accepted = 0;
    *count_invalid = 0;
    *count_outside = 0;
    *depth_crc = 0;
    *st_npos = 0;
    *st_le20 = 0;
    *st_mm0 = 0;
    *st_mm1 = 0;
    if ((width == 0) != (height == 0) || width > FPGA_M7_FRAME_MAX_WIDTH ||
        height > FPGA_M7_FRAME_MAX_HEIGHT || nx == 0 || ny == 0 || nz == 0 ||
        nx > FPGA_M7_FRAME_GRID_AXIS || ny > FPGA_M7_FRAME_GRID_AXIS ||
        nz > FPGA_M7_FRAME_GRID_AXIS) return 1;
    if ((depth_fmt & 1u) == 0) return 6;

    // Two private 32Kx1 planes per lane, each its own BRAM (one RAMB36).
    static ap_uint<1> free_pl[kLanes][kCap];
    static ap_uint<1> hit_pl[kLanes][kCap];
#pragma HLS ARRAY_PARTITION variable=free_pl type=complete dim=1
#pragma HLS ARRAY_PARTITION variable=hit_pl type=complete dim=1
#pragma HLS BIND_STORAGE variable=free_pl type=ram_t2p impl=bram
#pragma HLS BIND_STORAGE variable=hit_pl type=ram_t2p impl=bram
    // G2: see the header. Reset value true => full clear after ap_rst.
    static bool planes_dirty = true;
#pragma HLS RESET variable=planes_dirty
    // G1d: endpoint-voxel marks of Dlat2, cleared with the planes.
    static ap_uint<1> ev_pl[kCap];
#pragma HLS BIND_STORAGE variable=ev_pl type=ram_t2p impl=bram
#ifdef FPGA_M7_FAULT_NODIRTY
    if (false) {
#else
    if (planes_dirty) {
#endif
clear_loop:
        for (std::uint32_t i = 0; i < kCap; ++i) {
#pragma HLS PIPELINE II=1
            for (int k = 0; k < kLanes; ++k) {
#pragma HLS UNROLL
#ifdef FPGA_M7_FAULT_LANE_NOCLEAR
                if (kLanes > 1 && k == kLanes - 1) continue;
#endif
                free_pl[k][i] = 0;
                hit_pl[k][i] = 0;
            }
            ev_pl[i] = 0;
        }
    }
    planes_dirty = true;
    std::uint32_t accepted = 0, invalid = 0, clipped = 0, dcrc = 0;
    std::uint32_t stats[4] = {0, 0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=stats type=complete
    // GEO_FX3: the 12 frame constants come from one s_axilite array (a RAM in control_s_axi, not
    // 12 x 64-bit write/read-back registers): slot s = words 2s (low) / 2s+1 (high), slots 0-2 Gu,
    // 3-5 Gv, 6-8 G0 (bits [40:0]), 9-11 O (bits [35:0]). Fully unrolled constant-address reads:
    // ~24 RAM reads in the prologue, every field lands in its FxConst bits by wiring (no muxes).
    FxConst k;
fx_k_load:
    for (int a = 0; a < 3; ++a) {
#pragma HLS UNROLL
        const ap_uint<32> gu_lo = fx_k[2 * a], gu_hi = fx_k[2 * a + 1];
        const ap_uint<32> gv_lo = fx_k[6 + 2 * a], gv_hi = fx_k[6 + 2 * a + 1];
        const ap_uint<32> g0_lo = fx_k[12 + 2 * a], g0_hi = fx_k[12 + 2 * a + 1];
        const ap_uint<32> o_lo = fx_k[18 + 2 * a], o_hi = fx_k[18 + 2 * a + 1];
        k.gu(41 * a + 40, 41 * a) = (gu_hi(8, 0), gu_lo);
        k.gv(41 * a + 40, 41 * a) = (gv_hi(8, 0), gv_lo);
        k.g0(41 * a + 40, 41 * a) = (g0_hi(8, 0), g0_lo);
        k.o(36 * a + 35, 36 * a) = (o_hi(3, 0), o_lo);
    }
    k.c_lo = ap_uint<16>(fx_codes & 0xffffu);
    k.c_hi = ap_uint<16>(fx_codes >> 16);
    k.c20 = fx_c20;
    // Q8 origin in [0, 8192) (PS-checked, status 3); 13 bits
    const std::int32_t ofx = fx_origin_x.to_int();
    const std::int32_t ofy = fx_origin_y.to_int();
    const std::int32_t ofz = fx_origin_z.to_int();
    // width <= 1280, height <= 800 (status 1 above): 11 x 10 bit product, not 32 x 32 (3 DSP)
    const std::uint32_t pixels = (ap_uint<11>(width) * ap_uint<10>(height)).to_uint();
    frame_rays(depth_m, k, width, pixels, nx, ny, nz,
               ofx, ofy, ofz, free_pl, hit_pl,
               (dlat2_en & 1u) != 0, ev_pl, ap_uint<32>(f32_bits(depth_unit)),
               &dcrc, &accepted, &invalid, &clipped, stats);
#ifndef __SYNTHESIS__
    if (fpga_m7_stream_sim_abort) {
        fpga_m7_stream_sim_abort = 0;
        return 4;
    }
#endif
    write_out(flags, free_pl, hit_pl, ev_pl);
    planes_dirty = false;
    *count_accepted = accepted;
    *count_invalid = invalid;
    *count_outside = clipped;
    *depth_crc = dcrc;
    *st_npos = stats[0];
    *st_le20 = stats[1];
    *st_mm0 = stats[2];
    *st_mm1 = stats[3];
    return 0;
}
