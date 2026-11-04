// m7pl_test: minimal PS-side driver/test for the M7 PL design (2026-09-26).
//
//   m7pl_test [opts] info       sysinfo known answers, scratch, PL clock rate
//   m7pl_test [opts] frame      synthetic depth -> frame core -> compare with CPU
//   m7pl_test [opts] ar         synthetic depth+grid+boxes -> AR overlay core -> compare
//   m7pl_test [opts] roi        synthetic depth + box table -> E2 region-check core -> compare
//   m7pl_test [opts] permit     drive-permit heartbeat: identity, frame-done grant, timeout revoke
//   m7pl_test [opts] irq        UIO interrupt delivery from each HLS core
//   m7pl_test [opts] all        info frame ar roi permit
// (2026-10-11 merge build: the AR overlay core replaced the legacy display core;
//  v3a 2026-10-26: the legacy object core and its test are removed;
//  v3fc 2026-10-26: the PL watchdog and encoder UART cores are removed, and with
//  them the watchdog/encoder tests and the watchdog-driven drm demo.)
// opts: --provider auto|udmabuf|dmaheap|physmem|mock  --udmabuf NAME
//       --heap NAME  --phys-base HEX --phys-size HEX  --w N --h N
//       --frames N  --corrupt (mock: prove the comparator fails)  --timeout S
//       --flags-pa-offset N (SW_M2 positive control: add N bytes to the FLAGS_DATA address; N=2 must FAIL
//       the 4-byte alignment check before the core is started)
// The CPU reference is the unchanged HLS core C++ source compiled into this
// program with -ffp-contract=off (same as the HLS C simulation), so a match
// means "the PL produced exactly what the core's C semantics define".
// Exit status 0 only if every requested check passed.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>

#include "m7pl_dev.hpp"
#include "m7pl_hls_regs.h"
#include "../../frame_depth.hpp"
#include "../../object_evidence.hpp"
#include "../../ar_overlay.hpp"

using namespace m7pl;

// ------------------------------------------------------------------ AR overlay
// Register order (24 entries) = the scalar argument order of fpga_m7_ar_overlay:
// width .. mini_y; entry 15 is permit_i, a PL wire (permit/permit_o), not a register.
static const std::uint32_t kArRegOff[24] = {
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_WIDTH_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_HEIGHT_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_WIDTH_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_HEIGHT_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_DX_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_DY_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_FRAME_VALID_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_POSE_VALID_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_AGE_MS_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_AGE_VALID_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_MAX_AGE_MS_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_REPLAY_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_OBJECT_EVIDENCE_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_FRAME_ID_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_PERMIT_PRESENT_DATA, 0xFFFFFFFFu,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_GRID_VALID_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_SLICE_Y_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_SLICE_LO_MM_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_SLICE_HI_MM_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_CAM_CELL_X_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_CAM_CELL_Z_DATA,
    XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_MINI_X_DATA, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_MINI_Y_DATA};

// Shared with m7pl_drm.cpp. Buffers: depth u16 mm (depth_width*depth_height),
// grid 32768 B, desc 196 u32, fb width*height XRGB8888. regs[15] is ignored.
int render_ar(Device& d, const Buffer& bdepth, const Buffer& bgrid, const Buffer& bdesc, const Buffer& bfb,
              const std::uint32_t regs[24], double timeout, std::uint64_t* ns, std::uint64_t* cyc) {
    d.wr64(AR, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_DATA, bdepth.pa);
    d.wr64(AR, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_GRID_DATA, bgrid.pa);
    d.wr64(AR, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_BOXES_DATA, bdesc.pa);
    d.wr64(AR, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_FB_DATA, bfb.pa);
    for (int i = 0; i < 24; ++i) if (i != 15) d.wr(AR, kArRegOff[i], regs[i]);
    if (run_core(d, AR, timeout, ns, cyc) != 0) return -1;
    return static_cast<int>(d.rd(AR, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_AP_RETURN));
}

// CPU reference: the same core C source, beats packed with ap_uint::range().
std::uint32_t ar_cpu(const std::uint16_t* depth, const std::uint8_t* grid, const std::uint32_t* desc,
                     const std::uint32_t regs[24], std::uint32_t permit_i, std::uint32_t* fb) {
    const std::size_t dpx = static_cast<std::size_t>(regs[2]) * regs[3];
    const std::size_t fpx = static_cast<std::size_t>(regs[0]) * regs[1];
    std::vector<fpga_m7_ar_word> dv(dpx / 8 + 2, 0), gv(FPGA_M7_AR_GRID_WORDS, 0), bv(FPGA_M7_AR_DESC_WORDS, 0),
        fv(fpx / 4 + 2, 0);
    for (std::size_t i = 0; i < dpx; ++i) dv[i / 8].range(16 * (i % 8) + 15, 16 * (i % 8)) = depth[i];
    for (std::size_t i = 0; i < 32768; ++i) gv[i / 16].range(8 * (i % 16) + 7, 8 * (i % 16)) = grid[i];
    for (std::size_t i = 0; i < FPGA_M7_AR_DESC_U32; ++i) bv[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)) = desc[i];
    const std::uint32_t* r = regs;
    const std::uint32_t st = fpga_m7_ar_overlay(dv.data(), gv.data(), bv.data(), fv.data(),
        r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14],
        permit_i, r[16], r[17], static_cast<std::int32_t>(r[18]), static_cast<std::int32_t>(r[19]),
        r[20], r[21], r[22], r[23]);
    if (st != 1)
        for (std::size_t i = 0; i < fpx; ++i) fb[i] = fv[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)).to_uint();
    return st;
}

// Synthetic depth (u16 mm) for the AR panel and the DRM demo.
void ar_scene(std::uint16_t* depth, std::uint32_t w, std::uint32_t h, std::uint32_t phase) {
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x) {
            std::uint32_t z = 300 + ((x + phase) * 9000 / w + y * 700 / h) % 9600;
            if ((x / 40 + y / 40 + phase / 8) % 11 == 0) z = 0;  // missing depth tiles
            depth[static_cast<std::size_t>(y) * w + x] = static_cast<std::uint16_t>(z);
        }
}
// Synthetic 32^3 grid (0 unknown, 1 free, 2 hit) and a box descriptor.
void ar_grid(std::uint8_t* g, std::uint32_t phase) {
    for (std::uint32_t z = 0; z < 32; ++z)
        for (std::uint32_t y = 0; y < 32; ++y)
            for (std::uint32_t x = 0; x < 32; ++x) {
                std::uint8_t v = 0;
                if (z < 20 && x > 8 && x < 24) v = 1;
                if (z == 20 && ((x + phase) % 5) != 0) v = 2;
                g[(z * 32 + y) * 32 + x] = v;
            }
}
void ar_desc(std::uint32_t* desc, std::uint32_t dw, std::uint32_t dh, std::uint32_t nbox, std::uint32_t magic) {
    std::memset(desc, 0, FPGA_M7_AR_DESC_U32 * 4);
    desc[0] = magic;
    desc[1] = nbox;
    const char* labels[3] = {"BOTL", "LAPT", "ANC "};
    for (std::uint32_t i = 0; i < nbox && i < 32; ++i) {
        std::uint32_t* b = desc + 2 + 6 * i;
        const std::uint32_t x0 = (dw / 8) * (1 + (i % 5)), y0 = (dh / 6) * (1 + (i % 3));
        b[0] = x0 % dw; b[1] = y0 % dh;
        b[2] = std::min(dw - 1, b[0] + dw / 6); b[3] = std::min(dh - 1, b[1] + dh / 5);
        b[4] = (i % 3 == 0) ? 0 : (i % 3 == 1) ? 2 : 3;
        const char* l = labels[i % 3];
        b[5] = static_cast<std::uint32_t>(l[0]) | (static_cast<std::uint32_t>(l[1]) << 8) |
               (static_cast<std::uint32_t>(l[2]) << 16) | (static_cast<std::uint32_t>(l[3]) << 24);
    }
}

namespace {

struct Opts {
    DevOptions dev;
    std::uint32_t w = 640, h = 400;
    int frames = 120;
    bool corrupt = false;
    double timeout = 5.0;
    std::uint64_t flags_pa_offset = 0;   // SW_M2 positive control only: shifts the FLAGS_DATA address
};

int g_fail = 0;
// AR2NANO2 (2026-11-02): SLOT_MAP bit 8 = the AR overlay core is in the bitstream. FPGA_M7_PL_NO_AR=1 builds
// (car screen drawn on the Nano) have SLOT_MAP 0x263: every AR access is skipped (its window decodes to
// nothing there), info accepts both maps. Set in main from sysinfo before any command runs.
bool g_has_ar = true;
void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_fail;
}

long first_diff(const void* a, const void* b, std::size_t n) {
    const unsigned char* x = static_cast<const unsigned char*>(a);
    const unsigned char* y = static_cast<const unsigned char*>(b);
    for (std::size_t i = 0; i < n; ++i) if (x[i] != y[i]) return static_cast<long>(i);
    return -1;
}

double ms(std::uint64_t ns) { return ns / 1e6; }

// FNV-1a over the CPU reference output: lets x86 and aarch64 builds (and the
// board) be compared for the SAME reference bytes, independent of the PL.
std::uint64_t fnv(const void* p, std::size_t n) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    std::uint64_t h = 1469598103934665603ull;
    for (std::size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
    return h;
}

// ------------------------------------------------------------------ info
int cmd_info(Device& d) {
    const std::uint32_t magic = d.rd(SYSINFO, 0x00);
    std::printf("sysinfo MAGIC=0x%08x VERSION=0x%08x BUILD_ID=0x%08x BUILD_DATE=0x%08x CLK_HZ=%u SLOT_MAP=0x%04x\n",
                magic, d.rd(SYSINFO, 0x04), d.rd(SYSINFO, 0x08), d.rd(SYSINFO, 0x0C),
                d.rd(SYSINFO, 0x10), d.rd(SYSINFO, 0x20));
    check(magic == 0x4D37504C, "sysinfo MAGIC == 'M7PL'");
    bool scratch = true;
    for (std::uint32_t pat : {0xA5A55A5Au, 0x5A5AA5A5u, 0x00000000u, 0xFFFFFFFFu}) {
        d.wr(SYSINFO, 0x1C, pat);
        scratch = scratch && d.rd(SYSINFO, 0x1C) == pat;
    }
    check(scratch, "sysinfo SCRATCH write/readback");
    const std::uint64_t c0 = pl_cycles_now(d), t0 = mono_ns();
    while (mono_ns() - t0 < 200000000ull) {}
    const std::uint64_t c1 = pl_cycles_now(d), t1 = mono_ns();
    const double mhz = (c1 - c0) * 1e3 / (t1 - t0);
    const double nominal = d.rd(SYSINFO, 0x10) / 1e6;
    std::printf("PL clock measured %.3f MHz over %.1f ms (nominal %.3f MHz)\n", mhz, ms(t1 - t0), nominal);
    check(nominal > 0 && std::fabs(mhz - nominal) / nominal < 0.01, "PL clock within 1% of CLK_HZ");
    check(d.rd(PERMIT, 0x00) == 0x4D37504D, "permit MAGIC == 'M7PM'");
    const std::uint32_t slot_map = d.rd(SYSINFO, 0x20);
    // MERGE5A (2026-11-03): bit 10 = detection post-processing core (FPGA_M7_PL_DETPOST=1 -> 0x663); optional.
    // MERGE7 (2026-11-04): bit 11 = pre-processing core, bit 12 = planner (m5: 0x1E63); optional.
    check((slot_map & ~0x1C00u) == 0x363 || (slot_map & ~0x1C00u) == 0x263,
          "sysinfo SLOT_MAP == 0x363 (v3fc: slots 0,1,5,6,8,9; object/display/watchdog/encoder retired) "
          "or 0x263 (the same without the AR core, slot 8: FPGA_M7_PL_NO_AR=1, car screen on the Nano)");
    if (!g_has_ar) std::printf("INFO SLOT_MAP=0x%04x: no AR overlay core (car screen on the Nano); AR checks skipped\n", slot_map);
    for (int b = 0; b < NBLOCKS; ++b) {
        if (!is_hls_core(b) || (b == AR && !g_has_ar)) continue;
        const std::uint32_t ctrl = d.rd(static_cast<Block>(b), 0x0);
        char what[64];
        std::snprintf(what, sizeof what, "%s ap_idle after load (AP_CTRL=0x%x)", kBlockName[b], ctrl);
        check((ctrl & 0x4) != 0, what);
    }
    return 0;
}

// ------------------------------------------------------------------ frame
void frame_scene(std::vector<float>& depth, std::uint32_t w, std::uint32_t h) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    depth.assign(static_cast<std::size_t>(w) * h, 0.f);
    for (std::uint32_t v = 0; v < h; ++v)
        for (std::uint32_t u = 0; u < w; ++u) {
            float z = 1.2f + 0.8f * u / w + 0.3f * v / h;          // slanted wall
            if (u > w / 3 && u < w / 2 && v > h / 3 && v < 2 * h / 3) z = 0.9f;  // box
            if ((u * 7 + v * 3) % 97 == 0) z = 0.f;                // dropouts
            if (u < 8 && v < 8) z = nan;                           // bad patch
            if (u + 4 >= w && v + 4 >= h) z = 12.f;                // beyond range
            depth[static_cast<std::size_t>(v) * w + u] = z;
        }
}

int cmd_frame(Device& d, const Opts& o) {
    const std::uint32_t w = o.w, h = o.h;
    if (w > FPGA_M7_FRAME_MAX_WIDTH || h > FPGA_M7_FRAME_MAX_HEIGHT) { check(false, "frame size within core capacity"); return 1; }
    std::vector<float> depth;
    frame_scene(depth, w, h);
    const float fx = 0.8f * w, fy = 0.8f * w, cx = w / 2.f, cy = h / 2.f;
    const float ik[9] = {1.f / fx, 0, -cx / fx, 0, 1.f / fy, -cy / fy, 0, 0, 1};
    const float pose[12] = {1, 0, 0, 1.6f, 0, 1, 0, 1.6f, 0, 0, 1, .25f};
    const double origin[3] = {0, 0, 0};
    const double voxel = 0.1;
    const std::uint32_t n = 32;

    Buffer bd, bf;
    std::string err;
    if (!d.alloc(bd, depth.size() * 4, err) || !d.alloc(bf, FPGA_M7_FRAME_GRID_CAPACITY, err)) {
        std::printf("FAIL alloc: %s\n", err.c_str()); ++g_fail; return 1;
    }
    std::printf("buffers: depth pa=0x%llx flags pa=0x%llx provider=%s cached=%d\n",
                (unsigned long long)bd.pa, (unsigned long long)bf.pa, bd.provider.c_str(), bd.cached);
    std::memcpy(bd.va, depth.data(), depth.size() * 4);
    // SW_M2 (2026-11-03): since WB_BURST (m7_pl_m2*) the core writes the flags back as 32-bit bursts, so the
    // FLAGS_DATA address must be 4-byte aligned; the circuit does not check it. Fail loudly with the address.
    const std::uint64_t flags_pa = bf.pa + o.flags_pa_offset;
    if (flags_pa % 4 != 0) {
        std::printf("FAIL flags buffer pa=0x%llx is not 4-byte aligned (geometry write-back is 32-bit bursts "
                    "since WB_BURST; the core does not check it)\n", (unsigned long long)flags_pa);
        ++g_fail; return 1;
    }
    std::memset(bf.va, 0x5A, FPGA_M7_FRAME_GRID_CAPACITY);
    d.to_device(bd, 0, depth.size() * 4);
    d.to_device(bf, 0, FPGA_M7_FRAME_GRID_CAPACITY);

    auto program = [&](std::uint32_t nx) {
        d.wr64(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_DEPTH_M_DATA, bd.pa);
        d.wr64(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_FLAGS_DATA, flags_pa);   // alignment checked above
        for (int i = 0; i < 9; ++i) d.wrf(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_INVERSE_K_BASE + 4 * i, ik[i]);
        for (int i = 0; i < 12; ++i) d.wrf(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_WORLD_CAMERA_BASE + 4 * i, pose[i]);
        d.wrd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_GRID_ORIGIN_X_DATA, origin[0]);
        d.wrd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_GRID_ORIGIN_Y_DATA, origin[1]);
        d.wrd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_GRID_ORIGIN_Z_DATA, origin[2]);
        d.wrd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_VOXEL_SIZE_M_DATA, voxel);
        d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_WIDTH_DATA, w);
        d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_HEIGHT_DATA, h);
        d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NX_DATA, nx);
        d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NY_DATA, n);
        d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NZ_DATA, n);
    };

    // CPU reference (same C source as the core).
    std::vector<std::uint8_t> ref(FPGA_M7_FRAME_GRID_CAPACITY, 0x5A);
    std::uint32_t rc3[3] = {0}, rst = 99;
    std::vector<float> depth_full(FPGA_M7_FRAME_MAX_PIXELS, 0.f);
    std::memcpy(depth_full.data(), depth.data(), depth.size() * 4);
    const std::uint64_t tc0 = mono_ns();
    fpga_m7_depth_frame(depth_full.data(), ik, pose, origin, voxel, w, h, n, n, n, ref.data(), rc3, &rst);
    const std::uint64_t tc1 = mono_ns();

    program(n);
    // INT_SW (2026-11-02): the next bitstream's frame core has DLAT2_EN (G1d ray thinning, reset 0); this test
    // compares with the full-ray CPU reference, so the register must be 0. Read first; write only when it is
    // not 0 (a current bitstream has no register there: reads 0, nothing is written -> traffic as before).
    {
        const std::uint32_t dl = d.rd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_DLAT2_EN_DATA);
        if (dl != 0) {
            d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_DLAT2_EN_DATA, 0);
            std::printf("DLAT2_EN was %u, cleared to %u for the full-ray comparison\n", dl,
                        d.rd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_DLAT2_EN_DATA));
        }
    }
    if (o.corrupt) mock_corrupt_next(d, 12345);
    std::uint64_t ns = 0, cyc = 0;
    const int done = run_core(d, FRAME, o.timeout, &ns, &cyc);
    check(done == 0, "frame core ap_done before timeout");
    d.from_device(bf, 0, FPGA_M7_FRAME_GRID_CAPACITY);
    const std::uint32_t st = d.rd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_AP_RETURN);
    const std::uint32_t a = d.rd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_COUNT_ACCEPTED_DATA);
    const std::uint32_t b = d.rd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_COUNT_INVALID_DATA);
    // COUNT_OUTSIDE keeps its HLS register name; since M7_CLIP_SPEC_20261011 it
    // counts CLIPPED rays (被裁剪: endpoint outside the grid, walked as FREE, no HIT).
    const std::uint32_t c = d.rd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_COUNT_OUTSIDE_DATA);
    std::size_t marked = 0;
    for (auto f : ref) marked += f == 1 || f == 2;
    const long diff = first_diff(ref.data(), bf.va, FPGA_M7_FRAME_GRID_CAPACITY);
    std::printf("frame %ux%u: CPU status=%u counts(accepted/invalid/clipped)=%u/%u/%u marked=%zu cpu=%.2f ms | PL status=%u counts=%u/%u/%u "
                "pl=%.2f ms (%llu PL cycles) first_diff=%ld\n", w, h, rst, rc3[0], rc3[1], rc3[2], marked,
                ms(tc1 - tc0), st, a, b, c, ms(ns), (unsigned long long)cyc, diff);
    std::printf("REF_FNV frame %ux%u %016llx\n", w, h, (unsigned long long)fnv(ref.data(), ref.size()));
    check(rst == 0 && marked > 0, "frame CPU reference status 0 with marked voxels");
    check(st == rst && a == rc3[0] && b == rc3[1] && c == rc3[2], "frame PL status/counts == CPU");
    check(diff < 0, "frame PL flags (32768 B) bit-identical to CPU");

    // Error path: nx=0 -> status 1, flags untouched (sentinel stays).
    std::memset(bf.va, 0xC3, FPGA_M7_FRAME_GRID_CAPACITY);
    d.to_device(bf, 0, FPGA_M7_FRAME_GRID_CAPACITY);
    program(0);
    check(run_core(d, FRAME, o.timeout, nullptr, nullptr) == 0, "frame core (bad dims) ap_done");
    d.from_device(bf, 0, FPGA_M7_FRAME_GRID_CAPACITY);
    bool untouched = true;
    for (std::size_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        untouched = untouched && static_cast<std::uint8_t*>(bf.va)[i] == 0xC3;
    check(d.rd(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_AP_RETURN) == 1 && untouched,
          "frame bad dims: status 1 and flags untouched");
    d.release(bd);
    d.release(bf);
    return 0;
}

int cmd_ar(Device& d, const Opts& o) {
    // Canvas w x h; depth image below the 40-row panel (as rec30: dy = 40).
    if (!g_has_ar) { std::printf("SKIP ar: SLOT_MAP without the AR core (car screen on the Nano)\n"); return 0; }
    const std::uint32_t w = o.w & ~3u, h = o.h;
    if (w == 0 || h <= 40 || w > FPGA_M7_AR_MAX_WIDTH || h > FPGA_M7_AR_MAX_HEIGHT) {
        check(false, "ar canvas within core capacity (w%4==0, 40 < h <= 800)"); return 1;
    }
    const std::uint32_t dw = w & ~7u, dh = h - 40, n = w * h, dn = dw * dh;
    Buffer bdep, bgrid, bdesc, bfb;
    std::string err;
    if (!d.alloc(bdep, dn * 2 + 64, err) || !d.alloc(bgrid, 32768, err) || !d.alloc(bdesc, 1024, err) ||
        !d.alloc(bfb, static_cast<std::size_t>(n) * 4 + 64, err)) {
        std::printf("FAIL alloc: %s\n", err.c_str()); ++g_fail; return 1;
    }
    std::vector<std::uint16_t> depth(dn);
    ar_scene(depth.data(), dw, dh, 0);
    std::vector<std::uint8_t> grid(32768);
    ar_grid(grid.data(), 0);
    std::memcpy(bdep.va, depth.data(), dn * 2);
    std::memcpy(bgrid.va, grid.data(), 32768);
    d.to_device(bdep, 0, dn * 2);
    d.to_device(bgrid, 0, 32768);
    // permit_i is the PL wire from the permit block; the reference must use the
    // same value, read from PERMIT STATUS bit 0 (0 when no frame was accepted).
    const std::uint32_t permit = d.rd(PERMIT, 0x10) & 1u;
    struct Case { const char* name; std::uint32_t fv, pv, age, agev, replay, ev, pp, gv, nbox, magic, want; };
    const Case cases[] = {
        {"live", 1, 1, 33, 1, 0, 1, 0, 1, 3, FPGA_M7_AR_MAGIC, 0},
        {"live-permit", 1, 1, 33, 1, 0, 2, 1, 1, 7, FPGA_M7_AR_MAGIC, 0},
        {"stale", 1, 1, 250, 1, 0, 2, 1, 1, 3, FPGA_M7_AR_MAGIC, 0},
        {"noage", 1, 1, 0, 0, 0, 3, 0, 0, 2, FPGA_M7_AR_MAGIC, 0},
        {"replay", 1, 1, 0, 0, 1, 0, 0, 1, 32, FPGA_M7_AR_MAGIC, 0},
        {"posebad", 1, 0, 10, 1, 0, 0, 0, 1, 1, FPGA_M7_AR_MAGIC, 0},
        {"baddesc", 1, 1, 10, 1, 0, 0, 0, 1, 1, 0x12345678u, 3},
        {"badctrl", 2, 1, 10, 1, 0, 0, 0, 1, 1, FPGA_M7_AR_MAGIC, 2}};
    std::vector<std::uint32_t> ref(n), desc(FPGA_M7_AR_DESC_U32);
    for (const Case& c : cases) {
        const std::uint32_t regs[24] = {w, h, dw, dh, 0, 40, c.fv, c.pv, c.age, c.agev, 200, c.replay, c.ev,
                                        1234, c.pp, 0, c.gv, 12, 450, 500, 16, 1,
                                        w >= 136 ? w - 132 : 0u, h >= 172 ? 44u : 0u};
        ar_desc(desc.data(), dw, dh, c.nbox, c.magic);
        std::memcpy(bdesc.va, desc.data(), FPGA_M7_AR_DESC_U32 * 4);
        d.to_device(bdesc, 0, FPGA_M7_AR_DESC_U32 * 4);
        std::memset(bfb.va, 0xEE, n * 4);
        d.to_device(bfb, 0, n * 4);
        std::fill(ref.begin(), ref.end(), 0xEEEEEEEEu);
        const std::uint64_t tc0 = mono_ns();
        const std::uint32_t rst = ar_cpu(depth.data(), grid.data(), desc.data(), regs, permit, ref.data());
        const std::uint64_t tc1 = mono_ns();
        if (o.corrupt && !std::strcmp(c.name, "live")) mock_corrupt_next(d, 777);
        std::uint64_t ns = 0, cyc = 0;
        const int st = render_ar(d, bdep, bgrid, bdesc, bfb, regs, o.timeout, &ns, &cyc);
        d.from_device(bfb, 0, n * 4);
        const long diff = first_diff(ref.data(), bfb.va, static_cast<std::size_t>(n) * 4);
        std::printf("ar %ux%u %s: CPU status=%u cpu=%.2f ms | PL status=%d pl=%.2f ms (%llu cycles) permit_i=%u first_diff=%ld\n",
                    w, h, c.name, rst, ms(tc1 - tc0), st, ms(ns), (unsigned long long)cyc, permit, diff);
        std::printf("REF_FNV ar %ux%u %s %016llx\n", w, h, c.name,
                    (unsigned long long)fnv(ref.data(), static_cast<std::size_t>(n) * 4));
        char what[112];
        std::snprintf(what, sizeof what, "ar %s: status %u and XRGB bit-identical to CPU", c.name, c.want);
        check(rst == c.want && st == static_cast<int>(rst) && diff < 0, what);
    }
    d.release(bdep); d.release(bgrid); d.release(bdesc); d.release(bfb);
    return 0;
}

// ------------------------------------------------------------------ roi (E2)
int cmd_roi(Device& d, const Opts& o) {
    const std::uint32_t w = o.w, h = o.h;
    if (w == 0 || h == 0 || w > FPGA_M7_OBJECT_MAX_WIDTH || h > FPGA_M7_OBJECT_MAX_HEIGHT) {
        check(false, "roi size within core capacity"); return 1;
    }
    std::vector<float> depth;
    frame_scene(depth, w, h);
    Buffer bd;
    std::string err;
    if (!d.alloc(bd, depth.size() * 4, err)) { std::printf("FAIL alloc: %s\n", err.c_str()); ++g_fail; return 1; }
    std::memcpy(bd.va, depth.data(), depth.size() * 4);
    d.to_device(bd, 0, depth.size() * 4);
    const std::uint32_t N = FPGA_M7_ROI_MAX_BOXES;
    std::uint32_t en[N] = {0}, xyxy[4 * N] = {0}, oid[N] = {0};
    double nz[N] = {0}, fz[N] = {0};
    const std::uint32_t count = std::min<std::uint32_t>(N, 24);
    for (std::uint32_t i = 0; i < count; ++i) {
        en[i] = (i % 7 == 6) ? 0 : 1;  // some disabled rows
        const std::uint32_t bw = std::max<std::uint32_t>(4, w / 8), bh = std::max<std::uint32_t>(4, h / 8);  // >= 4x4 (core rule)
        const std::uint32_t x0 = (i * 37) % (w - bw), y0 = (i * 23) % (h - bh);
        xyxy[4 * i] = x0; xyxy[4 * i + 1] = y0; xyxy[4 * i + 2] = x0 + bw - 1; xyxy[4 * i + 3] = y0 + bh - 1;
        nz[i] = 0.8 + 0.05 * i; fz[i] = nz[i] + 0.4 + 0.01 * i;  // per-box thresholds differ
        oid[i] = 1000 + i;
    }
    // Core limits: min_pixels >= 9, min_valid_fraction in [0.7,1], free_fraction in [0.9,1], margin >= 0.04.
    const double margin = 0.04, minvf = 0.7, freef = 0.9;
    const std::uint32_t minpix = 9;
    // CPU reference (same C source as the core). Result pre-filled with a sentinel.
    std::vector<float> depth_full(FPGA_M7_OBJECT_MAX_PIXELS, 0.f);
    std::memcpy(depth_full.data(), depth.data(), depth.size() * 4);
    std::vector<std::uint32_t> ref(M7_RB_FIELDS * N, 0xA5A5A5A5u);
    std::uint32_t rst = 99;
    const std::uint64_t tc0 = mono_ns();
    fpga_m7_object_roi_batch(depth_full.data(), w, h, count, en, xyxy, nz, fz, oid, margin, minpix, minvf, freef,
                            ref.data(), &rst);
    const std::uint64_t tc1 = mono_ns();
    for (std::uint32_t i = 0; i < N; ++i) {
        d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_BOX_ENABLED_BASE + 4 * i, en[i]);
        d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_OBJECT_ID_BASE + 4 * i, oid[i]);
        d.wrd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_NEAR_Z_BASE + 8 * i, nz[i]);
        d.wrd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_FAR_Z_BASE + 8 * i, fz[i]);
        for (int k = 0; k < 4; ++k) d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_BOX_XYXY_BASE + 16 * i + 4 * k, xyxy[4 * i + k]);
    }
    d.wr64(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_DEPTH_DATA, bd.pa);
    d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_WIDTH_DATA, w);
    d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_HEIGHT_DATA, h);
    d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_BOX_COUNT_DATA, count);
    d.wrd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_DEPTH_MARGIN_M_DATA, margin);
    d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_MIN_PIXELS_DATA, minpix);
    d.wrd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_MIN_VALID_FRACTION_DATA, minvf);
    d.wrd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_FREE_FRACTION_DATA, freef);
    if (o.corrupt) mock_corrupt_next(d, 3 * M7_RB_FIELDS + M7_RB_VALID);
    std::uint64_t ns = 0, cyc = 0;
    const int done = run_core(d, ROI, o.timeout, &ns, &cyc);
    check(done == 0, "roi core ap_done before timeout");
    const std::uint32_t st = d.rd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_AP_RETURN);
    // Driver rule (OBJECT_ROI_SPEC 11.5): rows >= box_count and all rows on
    // nonzero status are stale register contents; read only the first count.
    long diff = -1;
    std::uint32_t verdicts[4] = {0};
    for (std::uint32_t i = 0; i < count * M7_RB_FIELDS && diff < 0; ++i)
        if (d.rd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_RESULT_BASE + 4 * i) != ref[i]) diff = i;
    for (std::uint32_t i = 0; i < count; ++i) verdicts[ref[i * M7_RB_FIELDS + M7_RB_VERDICT] & 3]++;
    std::printf("roi %ux%u boxes=%u: CPU status=%u cpu=%.2f ms verdicts(0/2/3)=%u/%u/%u | PL status=%u pl=%.2f ms "
                "(%llu PL cycles) first_diff=%ld\n", w, h, count, rst, ms(tc1 - tc0), verdicts[0], verdicts[2],
                verdicts[3], st, ms(ns), (unsigned long long)cyc, diff);
    std::printf("REF_FNV roi %ux%u %016llx\n", w, h,
                (unsigned long long)fnv(ref.data(), static_cast<std::size_t>(count) * M7_RB_FIELDS * 4));
    check(rst == 0, "roi CPU reference status 0");
    check(st == rst && diff < 0, "roi PL status and result rows bit-identical to CPU");
    // Status-1 run: nothing may change in the result block.
    d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_WIDTH_DATA, 0);
    run_core(d, ROI, o.timeout, nullptr, nullptr);
    long diff1 = -1;
    for (std::uint32_t i = 0; i < count * M7_RB_FIELDS && diff1 < 0; ++i)
        if (d.rd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_RESULT_BASE + 4 * i) != ref[i]) diff1 = i;
    check(d.rd(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_AP_RETURN) == 1 && (diff1 < 0 || o.corrupt),
          "roi status 1 on width 0, result block untouched");
    d.release(bd);
    return 0;
}

// ------------------------------------------------------------------ permit (B3)
// Registers: PL_PERMIT_UART_SPEC_20261011.md section 3.3.
int cmd_permit(Device& d, const Opts& o) {
    const std::uint32_t magic = d.rd(PERMIT, 0x00);
    const std::uint32_t win0 = d.rd(PERMIT, 0x44);
    std::printf("permit MAGIC=0x%08x STATUS=0x%x WINDOW_CYCLES=%u WINDOW_MIN=%u WINDOW_MAX=%u HB_HALF_CYCLES=%u CONFIG=0x%x\n",
                magic, d.rd(PERMIT, 0x10), win0, d.rd(PERMIT, 0x58), d.rd(PERMIT, 0x5C), d.rd(PERMIT, 0x48),
                d.rd(PERMIT, 0x54));
    check(magic == 0x4D37504D, "permit MAGIC == 'M7PM'");
    const double hz = d.rd(SYSINFO, 0x10);
    auto cyc = [&](double s) { return static_cast<std::uint32_t>(hz * s); };
    std::printf("permit window now %.1f ms (power-on default 1000 ms unless kv_server --permit-window-ms wrote it)\n",
                win0 / hz * 1e3);
    check(win0 >= cyc(0.1) && win0 <= cyc(5.0), "permit window inside [100 ms, 5000 ms]");
    check(d.rd(PERMIT, 0x58) == cyc(0.1) && d.rd(PERMIT, 0x5C) == cyc(5.0) &&
          d.rd(PERMIT, 0x48) == cyc(0.005), "permit WINDOW_MIN = 100 ms, WINDOW_MAX = 5000 ms, HB_HALF = 5 ms at CLK_HZ");
    if (d.is_mock()) {
        check(win0 == cyc(1.0), "permit power-on window = 1000 ms (mock)");
        std::printf("SKIP permit behaviour: not emulated by the mock device\n"); return 0;
    }
    // Window register: clamp and readback (each write also disarms, by design).
    d.wr(PERMIT, 0x44, 1);           check(d.rd(PERMIT, 0x44) == cyc(0.1), "permit window write 1 clamps to 100 ms");
    d.wr(PERMIT, 0x44, 0xFFFFFFFFu); check(d.rd(PERMIT, 0x44) == cyc(5.0), "permit window write 0xFFFFFFFF clamps to 5000 ms");
    d.wr(PERMIT, 0x44, cyc(0.3));    check(d.rd(PERMIT, 0x44) == cyc(0.3), "permit window write 300 ms reads back");
    // grant(): one frame-core completion edge with a fresh id. The frame core is
    // started with nx=0 (status 1, no DMA) -- ap_done still raises interrupt.
    auto grant = [&](const char* what) -> std::uint64_t {
        d.wr(PERMIT, 0x04, 0);                     // CTRL: no force stop
        d.wr(PERMIT, 0x3C, 0x7);                   // clear IRQ_STATUS
        const std::uint32_t acc0 = d.rd(PERMIT, 0x1C);
        const std::uint32_t id = (d.rd(PERMIT, 0x18) + 1) | 1u;
        d.wr(PERMIT, 0x08, id);                    // NEXT_FRAME_ID
        d.wr(PERMIT, 0x0C, 0);                     // CAPTURE_AGE
        d.wr(FRAME, 0x0C, d.rd(FRAME, 0x0C));      // clear stale ISR first (rising edge needed)
        d.wr(FRAME, 0x04, 1); d.wr(FRAME, 0x08, 1);
        d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NX_DATA, 0);
        run_core(d, FRAME, o.timeout, nullptr, nullptr);
        const std::uint64_t t = mono_ns();
        const std::uint32_t st = d.rd(PERMIT, 0x10);
        std::printf("permit after frame done (%s): STATUS=0x%x ACCEPTED %u->%u LAST_FRAME_ID=%u\n", what, st, acc0,
                    d.rd(PERMIT, 0x1C), d.rd(PERMIT, 0x18));
        check((st & 1u) && d.rd(PERMIT, 0x1C) == acc0 + 1, "permit granted by one frame-core completion edge");
        d.wr(FRAME, 0x0C, d.rd(FRAME, 0x0C));      // ISR clear: the driver must do this every frame
        d.wr(FRAME, 0x08, 0); d.wr(FRAME, 0x04, 0);
        return t;
    };
    // Autonomous expiry with no new frame, at 300 ms and then at the 1000 ms default.
    for (const double w : {0.3, 1.0}) {
        d.wr(PERMIT, 0x44, cyc(w));
        const std::uint64_t t0 = grant(w < 0.5 ? "window 300 ms" : "window 1000 ms");
        while ((d.rd(PERMIT, 0x10) & 1u) && mono_ns() - t0 < 7000000000ull) {}
        const double revoke_ms = ms(mono_ns() - t0);
        std::printf("permit revoked %.2f ms after grant with no new frame (expected ~%.0f ms), LAST_CAUSE=%u\n",
                    revoke_ms, w * 1e3, d.rd(PERMIT, 0x28));
        char what[96];
        std::snprintf(what, sizeof what, "permit autonomous timeout ~%.0f ms (not before), cause 1", w * 1e3);
        check(revoke_ms > w * 1e3 - 15.0 && revoke_ms < w * 1e3 + 15.0 && d.rd(PERMIT, 0x28) == 1, what);
    }
    // FORCE_STOP revokes at once at the 1000 ms window.
    grant("before FORCE_STOP");
    d.wr(PERMIT, 0x04, 1);
    const std::uint32_t st = d.rd(PERMIT, 0x10);
    check(!(st & 1u) && (st & (1u << 3)) != 0 && d.rd(PERMIT, 0x28) == 2,
          "permit FORCE_STOP revokes at once (first read after the write), STATUS b3, cause 2");
    d.wr(PERMIT, 0x04, 0);
    d.wr(PERMIT, 0x3C, 0x7);
    d.wr(PERMIT, 0x44, win0);                  // leave the window as found
    check(d.rd(PERMIT, 0x44) == win0, "permit window restored");
    return 0;
}

// ------------------------------------------------------------------ irq (UIO)
int find_uio(const char* name) {
    DIR* dir = opendir("/sys/class/uio");
    if (!dir) return -1;
    int found = -1;
    while (dirent* e = readdir(dir)) {
        if (std::strncmp(e->d_name, "uio", 3)) continue;
        char path[256], buf[64] = {0};
        std::snprintf(path, sizeof path, "/sys/class/uio/%s/name", e->d_name);
        FILE* f = std::fopen(path, "r");
        if (!f) continue;
        if (std::fgets(buf, sizeof buf, f)) {
            buf[std::strcspn(buf, "\n")] = 0;
            if (!std::strcmp(buf, name)) found = std::atoi(e->d_name + 3);
        }
        std::fclose(f);
    }
    closedir(dir);
    return found;
}

int cmd_irq(Device& d, const Opts& o) {
    // Every HLS core is started with an invalid size (status 1): nothing is
    // read or written, but ap_done (hence the level interrupt) still fires.
    for (int b = 0; b < NBLOCKS; ++b) {
        if (!is_hls_core(b)) continue;
        if (b == AR && !g_has_ar) { std::printf("SKIP irq m7_ar: SLOT_MAP without the AR core\n"); continue; }
        const char* node = kUioName[b];
        const int u = find_uio(node);
        char what[96];
        if (u < 0) { std::printf("SKIP irq %s: no UIO device (needs uio_pdrv_genirq.of_id=generic-uio)\n", node); continue; }
        char dev[32];
        std::snprintf(dev, sizeof dev, "/dev/uio%d", u);
        const int fd = open(dev, O_RDWR);
        if (fd < 0) { check(false, "open uio"); continue; }
        std::uint32_t one = 1;
        if (write(fd, &one, 4) != 4) { check(false, "uio irq enable"); close(fd); continue; }
        const Block blk = static_cast<Block>(b);
        d.wr(blk, 0x04, 1);  // GIE
        d.wr(blk, 0x08, 1);  // IER ap_done
        if (b == AR) d.wr(AR, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_WIDTH_DATA, 0);
        else if (b == ROI) d.wr(ROI, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_WIDTH_DATA, 0);
        else d.wr(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NX_DATA, 0);
        d.wr(blk, 0x0, 1);
        pollfd p = {fd, POLLIN, 0};
        const int pr = poll(&p, 1, static_cast<int>(o.timeout * 1000));
        std::uint32_t count = 0;
        const bool got = pr == 1 && read(fd, &count, 4) == 4;
        std::snprintf(what, sizeof what, "irq %s delivered via %s (event count %u)", node, dev, count);
        check(got, what);
        d.rd(blk, 0x0);                // consume ap_done
        d.wr(blk, 0x0C, d.rd(blk, 0x0C));  // toggle-on-write clears ISR
        d.wr(blk, 0x08, 0); d.wr(blk, 0x04, 0);
        close(fd);
    }
    return 0;
}

void usage() {
    std::fprintf(stderr, "usage: m7pl_test [--provider auto|udmabuf|dmaheap|physmem|mock] [--udmabuf NAME] [--heap NAME]\n"
                         "       [--phys-base HEX --phys-size HEX] [--w N --h N] [--frames N] [--timeout S] [--corrupt] [--flags-pa-offset N]\n"
                         "       info|frame|ar|roi|permit|irq|all\n");
}

}  // namespace

int main(int argc, char** argv) {
    Opts o;
    std::vector<std::string> cmds;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { if (i + 1 >= argc) { usage(); std::exit(2); } return argv[++i]; };
        if (a == "--provider") o.dev.provider = next();
        else if (a == "--udmabuf") o.dev.udmabuf = next();
        else if (a == "--heap") o.dev.heap = next();
        else if (a == "--phys-base") o.dev.phys_base = std::strtoull(next(), nullptr, 16);
        else if (a == "--phys-size") o.dev.phys_size = std::strtoull(next(), nullptr, 16);
        else if (a == "--w") o.w = std::strtoul(next(), nullptr, 0);
        else if (a == "--h") o.h = std::strtoul(next(), nullptr, 0);
        else if (a == "--frames") o.frames = std::atoi(next());
        else if (a == "--timeout") o.timeout = std::atof(next());
        else if (a == "--corrupt") o.corrupt = true;
        else if (a == "--flags-pa-offset") o.flags_pa_offset = std::strtoull(next(), nullptr, 0);
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else cmds.push_back(a);
    }
    if (cmds.empty()) { usage(); return 2; }
    std::string err;
    std::unique_ptr<Device> d(open_device(o.dev, err));
    if (!d) { std::fprintf(stderr, "open_device: %s\n", err.c_str()); return 3; }
    if (d->rd(SYSINFO, 0x00) == 0x4D37504C) g_has_ar = (d->rd(SYSINFO, 0x20) >> 8) & 1;   // unreadable: as before
    for (const std::string& c : cmds) {
        std::printf("==== %s ====\n", c.c_str());
        if (c == "info") cmd_info(*d);
        else if (c == "frame") cmd_frame(*d, o);
        else if (c == "ar") cmd_ar(*d, o);
        else if (c == "roi") cmd_roi(*d, o);
        else if (c == "permit") cmd_permit(*d, o);
        else if (c == "irq") cmd_irq(*d, o);
        else if (c == "all") {
            cmd_info(*d); cmd_frame(*d, o); cmd_ar(*d, o); cmd_roi(*d, o);
            cmd_permit(*d, o);
        }
        else { usage(); return 2; }
    }
    std::printf("RESULT %s (%d failed checks)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
