// GEO_FX3 (2026-11-04): synthetic bench of the fixed-point fpga_m7_frame_axi (m7_frame_stream.cpp),
// migrated from the float32 reference bench of A1/G1d/FWD_COV/D16 (same cases, now 16-bit codes).
// The core reads 16-bit codes only and gets its per-frame constants from the PS, so every case is:
//   codes (two per 32-bit word, low half first) + PS constants (hw/m7/frame_fx.cpp fpga_m7_fx_prepare,
//   W1 envelope reach < 2^17) -> DUT; expected status, counts, statistics, depth crc and all 32768
//   flag bytes from fpga_m7_fx_frame (the GEO_FX0 golden FX arm ported to hw/m7/frame_fx.cpp, bit-exact
//   against the golden on rec30 310 + car570 570 frames, GEO_FX3 ps_check), never from the DUT.
// A case whose PS status is not 0 (2 non-finite / bad voxel, 3 origin outside, 5 envelope) is checked
// at the PS only (the driver then runs the CPU reference; the core is not started).
// In C/RTL co-simulation the DUT side is the generated RTL.
//
// Cases (former float32 cases converted with unit 0.001 m: z -> code round(z / 0.001), NaN / 0 -> 0):
//   dense40x30, smallgrid (20x24x28 grid), baddims (core status 1), badvoxel (PS status 2),
//   empty0x0, the 9 FWD_COV frames of tb_frame_stream_fwdcov.h with dlat2_en = 1 (their codes are
//   stored as codes already; their old expected flags were the float reference's, the bench prints
//   how many bytes the FX expectation differs from them, for information), tailcorner,
//   d16_allcodes_a / _b (each of the 65536 codes once, units 0.001 / 0.000125), d16_odd (odd pixel
//   count, edge codes), d16_baddims.
// Env FPGA_M7_TB_FWDCOV=0 skips the FWD_COV frames.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "../../frame_depth.hpp"
#include "m7_frame_fx_ps.hpp"
#include "tb_frame_stream_fwdcov.h"

static std::uint32_t depth16[FPGA_M7_FRAME_MAX_PIXELS / 2 + 1];
static std::uint16_t codes[FPGA_M7_FRAME_MAX_PIXELS + 2];
alignas(4) static std::uint8_t flags_dut[FPGA_M7_FRAME_GRID_CAPACITY];
static std::uint8_t flags_exp[FPGA_M7_FRAME_GRID_CAPACITY];
static std::uint8_t flags_old[FPGA_M7_FRAME_GRID_CAPACITY];

static void pack_codes(std::uint32_t n) {
    for (std::uint32_t w = 0; w < (n + 1) / 2; ++w)
        depth16[w] = static_cast<std::uint32_t>(codes[2 * w]) | (static_cast<std::uint32_t>(codes[2 * w + 1]) << 16);
}

// expect_status: core status (>= 0) or -(PS status) for a PS-rejected case. old: optional former
// expected flags (FWD_COV), compared for information only.
static int run(const char* name, const float ik[9], const float wc[12], const double org[3], double vox,
               std::uint32_t w, std::uint32_t h, std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
               float unit, std::uint32_t dlat2, int expect_status, const std::uint8_t* old = nullptr) {
    const std::uint32_t np = w * h;
    FpgaFxK k;
    // core status 1 cases (bad grid dimensions): the PS constants of a 32^3 grid, the core gets nx/ny/nz
    // as given (with the given dims the PS would answer 3 and never start the core)
    fpga_fx_prepare(w, h, nx ? nx : 32, ny ? ny : 32, nz ? nz : 32, unit, ik, wc, org, vox, k, FPGA_M7_FX_REACH_W1);
    if (k.status != 0) {
        const bool ok = expect_status == -k.status;
        std::printf("[%s] %ux%u PS status=%d want %d (core not started) %s\n", name, w, h, k.status,
                    -expect_status, ok ? "ok" : "BAD");
        return ok ? 0 : 1;
    }
    codes[np] = 0xA5A5;  // the unused half of an odd tail word
    pack_codes(np);
    std::uint32_t ub;
    std::memcpy(&ub, &unit, 4);
    const std::uint32_t orr[3] = {(std::uint32_t)k.origin[0], (std::uint32_t)k.origin[1], (std::uint32_t)k.origin[2]};
    std::uint32_t ec[3], es[4], ecrc[1];
    std::memset(flags_exp, 0x5A, sizeof flags_exp);
    const std::uint32_t est = fpga_m7_fx_frame(codes, w, h, nx, ny, nz, k.words, orr, k.c_lo | (k.c_hi << 16),
                                              k.c20, dlat2, ub, flags_exp, ec, es, ecrc);
    std::memset(flags_dut, 0x5A, sizeof flags_dut);
    std::uint32_t a = 7, b = 7, c = 7, crc = 0xDEADBEEFu, s2 = 7, s3 = 7;
    ap_uint<21> s0 = 7, s1 = 7;
    const std::uint32_t st = fpga_m7_frame_axi(
        depth16, reinterpret_cast<std::uint32_t*>(flags_dut), ik, wc, org[0], org[1], org[2], vox, w, h, nx, ny, nz,
        &a, &b, &c, dlat2, 1, unit, &crc, (std::uint32_t)k.origin[0], (std::uint32_t)k.origin[1],
        (std::uint32_t)k.origin[2], k.c_lo | (k.c_hi << 16), k.c20, &s0, &s1, &s2, &s3, k.words);
    const std::uint32_t got[9] = {st, a, b, c, s0.to_uint(), s1.to_uint(), s2, s3, crc};
    const std::uint32_t want[9] = {est, ec[0], ec[1], ec[2], es[0], es[1], es[2], es[3], ecrc[0]};
    int regs_bad = 0;
    for (int i = 0; i < 9; ++i) regs_bad += got[i] != want[i];
    std::uint32_t diff = 0, marked = 0, olddiff = 0;
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i) {
        diff += flags_exp[i] != flags_dut[i];
        marked += flags_exp[i] == 1 || flags_exp[i] == 2;
        if (old) olddiff += old[i] != flags_exp[i];
    }
    const bool st_ok = static_cast<int>(est) == expect_status;
    std::printf("[%s] dlat2=%u %ux%u status=%u/%u counts=%u/%u/%u want %u/%u/%u stats=%08x/%08x/%08x/%08x "
                "crc=%08x want=%08x marked=%u flag_bytes_diff=%u regs_bad=%d%s",
                name, dlat2, w, h, st, est, a, b, c, ec[0], ec[1], ec[2], got[4], got[5], s2, s3, crc, ecrc[0],
                marked, diff, regs_bad, st_ok ? "" : " EXPECTED_STATUS_BAD");
    if (old) std::printf(" fx_vs_old_reference_bytes=%u", olddiff);
    std::printf("\n");
    return (diff || regs_bad || !st_ok) ? 1 : 0;
}

static std::uint16_t code_of(float z) {  // former float32 depth -> code at 0.001 m
    if (!(z > 0.f)) return 0;
    const double r = std::nearbyint(static_cast<double>(z) / 0.001);
    return r > 65535.0 ? 65535 : static_cast<std::uint16_t>(r);
}

int main() {
    const float unit = 0.001f;
    for (std::uint32_t y = 0; y < 30; ++y)
        for (std::uint32_t x = 0; x < 40; ++x) {
            float z = 1.0f + 0.05f * static_cast<float>((x * 7 + y * 3) % 23);
            if ((x + y) % 11 == 0) z = 0.f;
            if ((x * y) % 17 == 5) z = -1.f;  // was NaN: code 0
            if (x == 39 && y % 5 == 0) z = 12.f;
            if (x == 0 && y % 3 == 0) z = 9.5f;
            codes[y * 40 + x] = code_of(z);
        }
    const float ik[9] = {1.f/40, 0, -20.f/40, 0, 1.f/40, -15.f/40, 0, 0, 1};
    const float pose[12] = {1,0,0,1.6f, 0,1,0,1.6f, 0,0,1,.25f};
    const double origin0[3] = {0., 0., 0.};
    int rc = 0, ncase = 0;
    rc |= run("dense40x30", ik, pose, origin0, 0.1, 40, 30, 32, 32, 32, unit, 0, 0); ++ncase;
    rc |= run("smallgrid", ik, pose, origin0, 0.125, 40, 30, 20, 24, 28, unit, 0, 0); ++ncase;
    rc |= run("baddims", ik, pose, origin0, 0.1, 40, 30, 0, 32, 32, unit, 0, 1); ++ncase;
    rc |= run("badvoxel", ik, pose, origin0, -1.0, 40, 30, 32, 32, 32, unit, 0, -2); ++ncase;
    rc |= run("empty0x0", ik, pose, origin0, 0.1, 0, 0, 32, 32, 32, unit, 0, 0); ++ncase;
    const char* fc = std::getenv("FPGA_M7_TB_FWDCOV");
    if (!(fc && fc[0] == '0' && fc[1] == 0)) {
        for (int f = 0; f < fwdcov_nframes; ++f) {
            const FwdcovFrame& F = fwdcov_frames[f];
            for (std::uint32_t p = 0; p < F.w * F.h; ++p) codes[p] = F.depth_raw[p];
            std::memset(flags_old, 0, sizeof flags_old);
            for (std::uint32_t m = 0; m < F.n_marked; ++m)
                flags_old[F.marked[m] & 0x7fffu] = (F.marked[m] & 0x8000u) ? 2 : 1;
            char name[40];
            std::snprintf(name, sizeof name, "fwdcov_%s", F.name);
            rc |= run(name, F.ik, F.wc, F.origin, F.voxel, F.w, F.h, F.nx, F.ny, F.nz, F.unit, 1, 0, flags_old);
            ++ncase;
        }
    }
    {
        for (std::uint32_t u = 0; u < 4; ++u) codes[u] = code_of(1.1f);
        const float tik[9] = {1.f/11, 0, 8.f/11, 0, 0, 1, 0, 0, 1};
        const float tpose[12] = {1,0,0,2.05f, 0,1,0,2.05f, 0,0,1,2.05f};
        rc |= run("tailcorner", tik, tpose, origin0, 0.1, 4, 1, 32, 32, 32, unit, 0, 0); ++ncase;
        std::printf("[tailcorner] dut bytes 32760..32767:");
        for (std::uint32_t i = 32760; i < 32768; ++i) std::printf(" %u", flags_dut[i]);
        std::printf("\n");
    }
    const float aik[9] = {1.f/256, 0, -0.5f, 0, 1.f/256, -0.5f, 0, 0, 1};
    const float apose[12] = {1,0,0,1.6f, 0,1,0,1.6f, 0,0,1,.25f};
    for (std::uint32_t p = 0; p < 65536; ++p) codes[p] = static_cast<std::uint16_t>(p);
    rc |= run("d16_allcodes_a", aik, apose, origin0, 0.1, 256, 256, 32, 32, 32, 0.001f, 0, 0); ++ncase;
    for (std::uint32_t p = 0; p < 65536; ++p) codes[p] = static_cast<std::uint16_t>(p + 1);
    rc |= run("d16_allcodes_b", aik, apose, origin0, 0.1, 256, 256, 32, 32, 32, 0.000125f, 0, 0); ++ncase;
    const std::uint16_t edge[] = {0, 1, 99, 100, 101, 9999, 10000, 10001, 65535, 0, 2500};
    for (std::uint32_t p = 0; p < 123; ++p) codes[p] = p < 11 ? edge[p] : static_cast<std::uint16_t>(300 + 37 * p);
    const float oik[9] = {1.f/41, 0, -0.5f, 0, 1.f/41, -1.5f/41, 0, 0, 1};
    rc |= run("d16_odd", oik, apose, origin0, 0.1, 41, 3, 32, 32, 32, 0.001f, 0, 0); ++ncase;
    rc |= run("d16_baddims", oik, apose, origin0, 0.1, 41, 3, 0, 32, 32, 0.001f, 0, 1); ++ncase;
    std::printf(rc ? "TB_FRAME_STREAM FAIL rc=%d cases=%d\n" : "TB_FRAME_STREAM PASS rc=%d cases=%d\n", rc, ncase);
    return rc ? 1 : 0;
}
