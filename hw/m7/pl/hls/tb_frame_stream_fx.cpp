// GEO_FX3 (2026-11-04): constants passed as the fx_k[24] array (fpga_fx_prepare -> k.words).
// GEO_FX1 (2026-11-03): bench of the fixed-point fpga_m7_frame_axi (m7_frame_stream.cpp) against
// the golden FX arm of GEO_FX0 (tools/fx_golden.cpp). Used for csim and C/RTL co-simulation.
//
// Cases:
//   fx_baddims  tiny image, nx = 0                      -> status 1, counts / crc / stats 0
//   fx_fmt0     tiny image, depth_fmt = 0 (float input) -> status 6, counts / crc / stats 0
//   then every frame of env FPGA_M7_TB_FX_PACK (FPGAGPK01 pack, written by GEO_FX1/tools/make_tb.py)
//   with dlat2_en = 1, depth_fmt = 1, frame constants from m7_frame_fx_ps.hpp (= the golden's
//   fx_prepare), compared with env FPGA_M7_TB_FX_EXPECT (per frame: 9 x u32 status, accepted,
//   invalid, clipped, st_npos, st_le20, st_mm0, st_mm1, depth_crc; 32768 flag bytes), which holds
//   the golden FX flags / counts / statistics and the zlib crc32 of the float32 depth, never DUT
//   output. Env FPGA_M7_TB_FX_W1=1 uses the W1 envelope (reach < 2^17) for the PS status check.
// Frames run back to back in one process (planes / ev_pl write-back clear between frames).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "../../frame_depth.hpp"
#include "m7_frame_fx_ps.hpp"

static std::uint32_t depth16[FPGA_M7_FRAME_MAX_PIXELS / 2];
alignas(4) static std::uint8_t flags_dut[FPGA_M7_FRAME_GRID_CAPACITY];

struct Frame {
    std::uint32_t w, h, nx, ny, nz;
    float unit, ik[9], wc[12];
    double org[3], vox;
    std::vector<std::uint16_t> codes;
};

static int run_case(const char* name, const Frame& f, const FpgaFxK& k, std::uint32_t fmt,
                    const std::uint32_t want[9], const std::uint8_t* want_flags) {
    const std::uint32_t np = f.w * f.h;
    std::memset(depth16, 0xA5, sizeof(std::uint32_t) * ((np + 1) / 2 + 1 < FPGA_M7_FRAME_MAX_PIXELS / 2
                                                         ? (np + 1) / 2 + 1 : FPGA_M7_FRAME_MAX_PIXELS / 2));
    if (np) std::memcpy(depth16, f.codes.data(), 2 * static_cast<std::size_t>(np));
    std::memset(flags_dut, 0x5A, sizeof flags_dut);
    std::uint32_t a = 7, b = 7, c = 7, crc = 0xDEADBEEFu, s0 = 7, s1 = 7, s2 = 7, s3 = 7;
#ifdef FPGA_FX_ARGS_NARROW
        ap_uint<21> s0n = 7, s1n = 7;
#define FPGA_FX_S0 &s0n
#define FPGA_FX_S1 &s1n
#else
#define FPGA_FX_S0 &s0
#define FPGA_FX_S1 &s1
#endif
    const std::uint32_t st = fpga_m7_frame_axi(
        depth16, reinterpret_cast<std::uint32_t*>(flags_dut), f.ik, f.wc, f.org[0], f.org[1], f.org[2],
        f.vox, f.w, f.h, f.nx, f.ny, f.nz, &a, &b, &c, 1, fmt, f.unit, &crc,
        (std::uint32_t)k.origin[0], (std::uint32_t)k.origin[1],
        (std::uint32_t)k.origin[2], k.c_lo | (k.c_hi << 16), k.c20, FPGA_FX_S0, FPGA_FX_S1, &s2, &s3, k.words);
#ifdef FPGA_FX_ARGS_NARROW
        s0 = s0n.to_uint(); s1 = s1n.to_uint();
#endif
#undef FPGA_FX_S0
#undef FPGA_FX_S1
    const std::uint32_t got[9] = {st, a, b, c, s0, s1, s2, s3, crc};
    std::uint32_t diff = 0;
    if (want_flags)
        for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i) diff += want_flags[i] != flags_dut[i];
    else
        for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i) diff += flags_dut[i] != 0x5A;  // untouched
    int regs_bad = 0;
    for (int i = 0; i < 9; ++i) regs_bad += got[i] != want[i];
    std::printf("[%s] %ux%u status=%u/%u counts=%u/%u/%u want %u/%u/%u stats=%08x/%08x/%08x/%08x want "
                "%08x/%08x/%08x/%08x crc=%08x want=%08x flag_bytes_diff=%u regs_bad=%d\n",
                name, f.w, f.h, st, want[0], a, b, c, want[1], want[2], want[3], s0, s1, s2, s3, want[4],
                want[5], want[6], want[7], crc, want[8], diff, regs_bad);
    return (diff || regs_bad) ? 1 : 0;
}

int main() {
    int rc = 0, ncase = 0;
    // error cases: tiny frame
    Frame t;
    t.w = 4; t.h = 2; t.nx = 32; t.ny = 32; t.nz = 32; t.unit = 0.001f;
    const float tik[9] = {1.f / 4, 0, -0.5f, 0, 1.f / 4, -0.25f, 0, 0, 1};
    const float twc[12] = {1, 0, 0, 0.8f, 0, 1, 0, 0.8f, 0, 0, 1, 0.05f};
    std::memcpy(t.ik, tik, sizeof tik);
    std::memcpy(t.wc, twc, sizeof twc);
    t.org[0] = t.org[1] = t.org[2] = 0.0; t.vox = 0.05;
    t.codes.assign(8, 1500);
    FpgaFxK tk;
    fpga_fx_prepare(t.w, t.h, t.nx, t.ny, t.nz, t.unit, t.ik, t.wc, t.org, t.vox, tk);
    const std::uint32_t zero1[9] = {1, 0, 0, 0, 0, 0, 0, 0, 0};
    const std::uint32_t zero6[9] = {6, 0, 0, 0, 0, 0, 0, 0, 0};
    Frame tb = t; tb.nx = 0;
    rc |= run_case("fx_baddims", tb, tk, 1, zero1, nullptr); ++ncase;
    rc |= run_case("fx_fmt0", t, tk, 0, zero6, nullptr); ++ncase;

    const char* pp = std::getenv("FPGA_M7_TB_FX_PACK");
    const char* ep = std::getenv("FPGA_M7_TB_FX_EXPECT");
    const char* w1e = std::getenv("FPGA_M7_TB_FX_W1");
    const double reach_limit = (w1e && w1e[0] == '1') ? 131072.0 : 1048576.0;
    if (!pp || !*pp || !ep || !*ep) {
        std::printf("TB_FRAME_STREAM_FX needs FPGA_M7_TB_FX_PACK and FPGA_M7_TB_FX_EXPECT\n");
        return 1;
    }
    FILE* fp = std::fopen(pp, "rb");
    FILE* fe = std::fopen(ep, "rb");
    if (!fp || !fe) { std::printf("TB_FRAME_STREAM_FX cannot open inputs\n"); return 1; }
    unsigned char hdr[24];
    if (std::fread(hdr, 1, 24, fp) != 24 || std::memcmp(hdr, "FPGAGPK01", 8) != 0) {
        std::printf("TB_FRAME_STREAM_FX bad pack header\n");
        return 1;
    }
    std::uint32_t n;
    std::memcpy(&n, hdr + 12, 4);
    static std::uint8_t want_flags[FPGA_M7_FRAME_GRID_CAPACITY];
    for (std::uint32_t i = 0; i < n; ++i) {
        Frame f;
        std::uint32_t u[6];
        if (std::fread(u, 4, 6, fp) != 6) { rc |= 1; break; }
        f.w = u[0]; f.h = u[1]; f.nx = u[2]; f.ny = u[3]; f.nz = u[4];  // u[5]: unused pack word
        if (std::fread(&f.unit, 4, 1, fp) != 1 || std::fread(f.ik, 4, 9, fp) != 9 || std::fread(f.wc, 4, 12, fp) != 12 ||
            std::fread(f.org, 8, 3, fp) != 3 || std::fread(&f.vox, 8, 1, fp) != 1 ||
            f.w * f.h > FPGA_M7_FRAME_MAX_PIXELS) { rc |= 1; break; }
        f.codes.resize(f.w * f.h);
        if (std::fread(f.codes.data(), 2, f.w * f.h, fp) != f.w * f.h) { rc |= 1; break; }
        std::uint32_t want[9];
        if (std::fread(want, 4, 9, fe) != 9 || std::fread(want_flags, 1, sizeof want_flags, fe) != sizeof want_flags) {
            std::printf("TB_FRAME_STREAM_FX expect file too short at frame %u\n", i);
            rc |= 1;
            break;
        }
        FpgaFxK k;
        fpga_fx_prepare(f.w, f.h, f.nx, f.ny, f.nz, f.unit, f.ik, f.wc, f.org, f.vox, k, reach_limit);
        if (k.status != 0) { std::printf("TB_FRAME_STREAM_FX frame %u PS status %d\n", i, k.status); rc |= 1; continue; }
        char name[32];
        std::snprintf(name, sizeof name, "fx_frame%u", i);
        rc |= run_case(name, f, k, 1, want, want_flags);
        ++ncase;
    }
    std::fclose(fp);
    std::fclose(fe);
    std::printf(rc ? "TB_FRAME_STREAM_FX FAIL rc=%d cases=%d\n" : "TB_FRAME_STREAM_FX PASS rc=%d cases=%d\n", rc, ncase);
    return rc ? 1 : 0;
}
