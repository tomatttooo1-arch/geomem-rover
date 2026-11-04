// Bench for fpga_m7_object_roi_axi (E2). Full-size frames for C/RTL
// co-simulation cycle counts. Expected counts come from an independent
// per-box float32 reference loop (the E1 predicate written out directly), not
// from the core, so a co-simulated RTL result is checked against it.
//   -DFPGA_M7_ROI_TB_SCENE=0  all scenes + one rejection (default; C sim and,
//                            since SLIM_ROI1c, also valid under C/RTL co-sim)
//   -DFPGA_M7_ROI_TB_SCENE=n  only scene n (one transaction per co-sim run,
//                            except scene 4)
// Scenes: 1 = 640x400, 32 boxes; 2 = 640x480, 1 box; 3 = 1280x800, 32 boxes;
//         4 = 40 small random batches (RTL coverage, not timing).
#include <cstdio>
#include <cstring>
#include <limits>
#include "../../object_evidence.hpp"

#ifndef FPGA_M7_ROI_TB_SCENE
#define FPGA_M7_ROI_TB_SCENE 0
#endif

extern "C" std::uint32_t fpga_m7_object_roi_axi(
    const float*, std::uint32_t, std::uint32_t, std::uint32_t,
    const std::uint32_t[FPGA_M7_ROI_MAX_BOXES], const std::uint32_t[4 * FPGA_M7_ROI_MAX_BOXES],
    const double[FPGA_M7_ROI_MAX_BOXES], const double[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t[FPGA_M7_ROI_MAX_BOXES], double, std::uint32_t, double, double,
    std::uint32_t[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES]);

static const std::uint32_t N = FPGA_M7_ROI_MAX_BOXES;
static float depth[FPGA_M7_OBJECT_MAX_PIXELS];
static std::uint32_t enabled[N], xyxy[4 * N], ids[N];
static double near_z[N], far_z[N];
static std::uint32_t result[M7_RB_FIELDS * N];
static std::uint32_t lcg = 12345u;

static std::uint32_t next_rand() {
    lcg = lcg * 1664525u + 1013904223u;
    return lcg >> 8;
}

static void fill(std::uint32_t w, std::uint32_t x0, std::uint32_t y0, std::uint32_t x1,
                 std::uint32_t y1, float z) {
    for (std::uint32_t y = y0; y <= y1; ++y)
        for (std::uint32_t x = x0; x <= x1; ++x) depth[y * w + x] = z;
}

// Wall at 3.0..3.5 m with sparse invalid returns; boxes are historical object
// volumes around camera Z 1.8..2.4 m (per-box extents). Box k gets one of five
// contents plus a threshold-crossing ramp in its bottom rows.
static void make_scene(std::uint32_t w, std::uint32_t h, std::uint32_t boxes) {
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x) {
            const std::uint32_t r = next_rand();
            float z = 3.0f + 0.5f * static_cast<float>(x) / static_cast<float>(w);
            if (r % 97 == 0) z = std::numeric_limits<float>::quiet_NaN();
            else if (r % 89 == 0) z = 0.f;
            depth[y * w + x] = z;
        }
    const std::uint32_t cols = 8, bw = w / cols, bh = h / 5;
    for (std::uint32_t b = 0; b < N; ++b) {
        enabled[b] = b < boxes ? 1u : 0u;
        ids[b] = 500u + b;
        // Every box has its own thresholds, so a lane that picked up another
        // box's front/back key changes that row's counts.
        near_z[b] = 1.80 + 0.01 * b;
        far_z[b] = near_z[b] + 0.15 + 0.005 * b;
        const std::uint32_t cx = (b % cols) * bw, cy = (b / cols) * bh;
        const std::uint32_t x0 = cx + 2, y0 = cy + 2;
        const std::uint32_t x1 = cx + bw * 3 / 4, y1 = cy + bh * 3 / 4;
        xyxy[4 * b] = x0; xyxy[4 * b + 1] = y0; xyxy[4 * b + 2] = x1; xyxy[4 * b + 3] = y1;
        if (b >= boxes) continue;
        switch (b % 5) {
        case 0: fill(w, x0, y0, x1, y1, 2.0f); break;              // still there
        case 1: break;                                            // removed: wall behind
        case 2: fill(w, x0, y0, x1, (y0 + y1) / 2 + 1, 1.0f); break;  // person in front
        case 3: fill(w, x0, y0, x1, y1, std::numeric_limits<float>::quiet_NaN());
                fill(w, x0, y0, x1, y0 + 1, 3.0f); break;         // mostly invalid
        default: fill(w, x0, y0, (x0 + x1) / 2, y1, 2.0f); break;  // half surface
        }
        // Bottom rows: a depth ramp 1.6..2.4 m crossing every box's own
        // front/back thresholds (0.01 m apart from box to box).
        const std::uint32_t ys = y1 - (y1 - y0) / 12, cols_in = x1 - x0 + 1;
        const float span = static_cast<float>((y1 - ys + 1) * cols_in);
        for (std::uint32_t y = ys; y <= y1; ++y)
            for (std::uint32_t x = x0; x <= x1; ++x)
                depth[y * w + x] = 1.6f + 0.8f * static_cast<float>((y - ys) * cols_in + (x - x0)) / span;
    }
    if (boxes > 1) {  // one overlapping box spanning boxes 0 and 1 (row 0)
        xyxy[4 * (boxes - 1)] = 4;
        xyxy[4 * (boxes - 1) + 2] = bw + bw / 2;
    }
}

#ifdef FPGA_M7_ROI_DEBUG_KEYS
// Same mapping as fpga_roi_key in object_evidence.cpp (spec section 11).
static std::uint32_t debug_key(float t) {
    std::uint32_t bits;
    std::memcpy(&bits, &t, sizeof bits);
    if ((bits & 0x80000000u) || bits <= 0x3DCCCCCDu) return 0u;
    if (bits >= 0x41200000u) return 0x41200000u - 0x3DCCCCCDu;
    return bits - 0x3DCCCCCDu;
}
#endif

// Lane attribution for a mismatching row: which box's front (back) threshold,
// applied to box b's own pixels, reproduces the reported front (behind)
// count. "Z" = threshold 0 (key 0), "M" = threshold 10 m (key max). Several
// sources may tie; up to eight are listed.
static void attribute(std::uint32_t w, std::uint32_t b, std::uint32_t boxes, double margin,
                      std::uint32_t got_front, std::uint32_t got_behind) {
    for (int side = 0; side < 2; ++side) {
        std::printf("    %s src:", side ? "back" : "front");
        int listed = 0;
        for (int j = -2; j < static_cast<int>(boxes) && listed < 8; ++j) {
            float t;
            if (j == -2) t = 0.f;
            else if (j == -1) t = 10.f;
            else if (!enabled[j]) continue;
            else t = side ? static_cast<float>(far_z[j] + margin)
                          : static_cast<float>(near_z[j] - margin);
            std::uint32_t n = 0;
            for (std::uint32_t y = xyxy[4 * b + 1]; y <= xyxy[4 * b + 3]; ++y)
                for (std::uint32_t x = xyxy[4 * b]; x <= xyxy[4 * b + 2]; ++x) {
                    const float z = depth[y * w + x];
                    if (!(z > 0.1f && z < 10.0f)) continue;
                    n += side ? z > t : z < t;
                }
            if (n == (side ? got_behind : got_front)) {
                if (j == -2) std::printf(" Z");
                else if (j == -1) std::printf(" M");
                else std::printf(" %d", j);
                ++listed;
            }
        }
        std::printf("%s", side ? "\n" : "");
    }
}

static int check(const char* name, std::uint32_t w, std::uint32_t h, std::uint32_t boxes,
                 double margin = .04, std::uint32_t min_pixels = 9, double vf = .7,
                 double ff = .9, bool quiet = false) {
    for (std::uint32_t i = 0; i < M7_RB_FIELDS * N; ++i) result[i] = 0xFFFFFFFFu;
    const std::uint32_t status = fpga_m7_object_roi_axi(depth, w, h, boxes, enabled, xyxy,
        near_z, far_z, ids, margin, min_pixels, vf, ff, result);
    int bad = status != 0;
    std::uint32_t tally[4] = {0, 0, 0, 0};
    for (std::uint32_t b = 0; b < N; ++b) {
        const std::uint32_t* r = result + M7_RB_FIELDS * b;
        // Rows past box_count are not written by the core. The result block is
        // an output-only s_axilite RAM, so in C/RTL co-simulation (and on the
        // board) those rows read back whatever the RAM held, not this bench's
        // sentinel; only C simulation can check them (scene 0 rejection below).
        if (b >= boxes) continue;
        if (!enabled[b]) {
            const std::uint32_t off[M7_RB_FIELDS] = {ids[b], 0u, 0u, 0u, 0u, 0u, 0u};
            if (std::memcmp(off, r, sizeof off) != 0) {
                bad = 1;
                std::printf("  %s box %u disabled row mismatch\n", name, b);
            }
            continue;
        }
#ifdef FPGA_M7_ROI_DEBUG_KEYS
        // Test-only (kernel built with the same macro): the front/behind
        // fields carry lane b's threshold keys. Name the box(es) they belong to.
        {
            const std::uint32_t fk = debug_key(static_cast<float>(near_z[b] - margin));
            const std::uint32_t bk = debug_key(static_cast<float>(far_z[b] + margin));
            const bool own = r[4] == fk && r[5] == bk;
            bad |= !own;
            std::printf("  %s lane %u key front=%u back=%u %s; front of box", name, b, r[4], r[5],
                        own ? "own" : "FOREIGN");
            for (std::uint32_t j = 0; j < boxes; ++j)
                if (enabled[j] && debug_key(static_cast<float>(near_z[j] - margin)) == r[4])
                    std::printf(" %u", j);
            std::printf("; back of box");
            for (std::uint32_t j = 0; j < boxes; ++j)
                if (enabled[j] && debug_key(static_cast<float>(far_z[j] + margin)) == r[5])
                    std::printf(" %u", j);
            std::printf("\n");
            continue;
        }
#endif
        const float front = static_cast<float>(near_z[b] - margin);
        const float back = static_cast<float>(far_z[b] + margin);
        std::uint32_t pix = 0, valid = 0, ahead = 0, behind = 0;
        for (std::uint32_t y = xyxy[4 * b + 1]; y <= xyxy[4 * b + 3]; ++y)
            for (std::uint32_t x = xyxy[4 * b]; x <= xyxy[4 * b + 2]; ++x) {
                const float z = depth[y * w + x];
                ++pix;
                if (!(z > 0.1f && z < 10.0f)) continue;
                ++valid;
                ahead += z < front;
                behind += z > back;
            }
        std::uint32_t verdict = M7_RB_INDETERMINATE;
        if (valid >= min_pixels && static_cast<double>(valid) / pix >= vf) {
            if (static_cast<double>(ahead) / valid >= 0.5) verdict = M7_RB_OCCLUDED;
            else if (static_cast<double>(behind) / valid >= ff) verdict = M7_RB_FREE_SPACE_CONFLICT;
        }
        const std::uint32_t expect[M7_RB_FIELDS] = {ids[b], 1u, pix, valid, ahead, behind, verdict};
        if (std::memcmp(expect, r, sizeof expect) != 0) {
            bad = 1;
            std::printf("  %s box %u mismatch: got pix=%u valid=%u front=%u behind=%u code=%u"
                        " | want pix=%u valid=%u front=%u behind=%u code=%u\n",
                        name, b, r[2], r[3], r[4], r[5], r[6], pix, valid, ahead, behind, verdict);
            attribute(w, b, boxes, margin, r[4], r[5]);
        }
        ++tally[verdict];
    }
    if (!quiet || bad)
        std::printf("[%s] %ux%u boxes=%u status=%u verdicts indeterminate=%u free=%u occluded=%u %s\n",
                    name, w, h, boxes, status, tally[0], tally[2], tally[3], bad ? "FAIL" : "ok");
    return bad;
}

// Scene 4: many small random batches (one transaction each): per-box random
// extents and rectangles (overlaps, image edges, disabled rows), depth ramps
// and special values, margins up to 0.5 m (front <= 0). All accepted batches.
static int random_batches(int count) {
    static const double nears[] = {0.1000001, 0.15, 0.3, 0.55, 1.0, 1.86, 1.9, 2.0, 3.3, 9.97, 10.2, 12.0};
    static const double margins[] = {.04, .05, .2, .5};
    int rc = 0;
    std::uint32_t verdicts[4] = {0, 0, 0, 0};
    for (int t = 0; t < count; ++t) {
        const std::uint32_t w = 8 + next_rand() % 57, h = 6 + next_rand() % 43;
        const std::uint32_t boxes = 1 + next_rand() % N;
        const double margin = margins[next_rand() % 4];
        const std::uint32_t min_pixels = 9 + next_rand() % 3;
        for (std::uint32_t i = 0; i < w * h; ++i) {
            const std::uint32_t r = next_rand();
            float z = 0.05f + 0.25f * static_cast<float>(r % 50);   // 0.05 .. 12.3 m
            if (r % 37 == 0) z = std::numeric_limits<float>::quiet_NaN();
            else if (r % 41 == 0) z = -1.f;
            else if (r % 43 == 0) z = std::numeric_limits<float>::infinity();
            depth[i] = z;
        }
        for (std::uint32_t b = 0; b < N; ++b) {
            enabled[b] = next_rand() % 8 != 0;
            ids[b] = next_rand();
            const std::uint32_t x0 = next_rand() % (w - 3), y0 = next_rand() % (h - 3);
            const std::uint32_t x1 = x0 + 3 + next_rand() % (w - x0 - 3);
            const std::uint32_t y1 = y0 + 3 + next_rand() % (h - y0 - 3);
            xyxy[4 * b] = x0; xyxy[4 * b + 1] = y0; xyxy[4 * b + 2] = x1; xyxy[4 * b + 3] = y1;
            near_z[b] = nears[next_rand() % 12];
            far_z[b] = near_z[b] + 0.01 * static_cast<double>(next_rand() % 40);
            if (b < boxes && enabled[b] && next_rand() % 2) {
                // ramp across this box through its own thresholds
                const float lo = static_cast<float>(near_z[b] - margin) - 0.3f;
                const float hi = static_cast<float>(far_z[b] + margin) + 0.3f;
                const std::uint32_t cols_in = x1 - x0 + 1, rows_in = y1 - y0 + 1;
                for (std::uint32_t y = y0; y <= y1; ++y)
                    for (std::uint32_t x = x0; x <= x1; ++x)
                        depth[y * w + x] = lo + (hi - lo) *
                            static_cast<float>((y - y0) * cols_in + (x - x0)) /
                            static_cast<float>(cols_in * rows_in);
            }
        }
        char name[16];
        std::snprintf(name, sizeof name, "r%d", t);
        rc |= check(name, w, h, boxes, margin, min_pixels, .7, .9, true);
        for (std::uint32_t b = 0; b < boxes; ++b)
            if (enabled[b]) ++verdicts[result[M7_RB_FIELDS * b + M7_RB_VERDICT] & 3u];
    }
    std::printf("[s4] %d random batches verdicts indeterminate=%u free=%u occluded=%u %s\n",
                count, verdicts[0], verdicts[2], verdicts[3], rc ? "FAIL" : "ok");
    return rc;
}

int main() {
    int rc = 0;
    const int scene = FPGA_M7_ROI_TB_SCENE;
    if (scene == 0 || scene == 1) { make_scene(640, 400, N); rc |= check("s1", 640, 400, N); }
    if (scene == 0 || scene == 2) { make_scene(640, 480, 1); rc |= check("s2", 640, 480, 1); }
    if (scene == 0 || scene == 3) { make_scene(1280, 800, N); rc |= check("s3", 1280, 800, N); }
    if (scene == 0 || scene == 4) rc |= random_batches(40);
    if (scene == 0) {  // rejection: result block must stay untouched
        // Two equivalent ways to see "untouched" (SLIM_ROI1c, 2026-11-02):
        //  - C sim: the block still holds this bench's sentinel.
        //  - C/RTL co-sim: the output-only s_axilite RAM is never loaded from the
        //    C side (the autotb has no input vector for it), so the sentinel never
        //    reaches the RTL; the block must instead read back exactly what it
        //    held after the previous transaction (`kept`, the full 224 words of
        //    that readback). Same rule as the board test (m7pl_test.cpp, status-1
        //    run) and the driver rule in OBJECT_ROI_SPEC 11.5.
        // A row written by the core would carry this scene's ids (500..503);
        // `armed` makes sure neither reference already holds them.
        static std::uint32_t kept[M7_RB_FIELDS * N];
        std::memcpy(kept, result, sizeof kept);
        make_scene(64, 48, 4);
        xyxy[4 * 2 + 2] = 64;  // x1 == width
        bool armed = true;
        for (std::uint32_t b = 0; b < 4; ++b)
            armed &= kept[M7_RB_FIELDS * b] != ids[b] && 0xABCD0000u + M7_RB_FIELDS * b != ids[b];
        for (std::uint32_t i = 0; i < M7_RB_FIELDS * N; ++i) result[i] = 0xABCD0000u + i;
        const std::uint32_t status = fpga_m7_object_roi_axi(depth, 64, 48, 4, enabled, xyxy,
            near_z, far_z, ids, .04, 9, .7, .9, result);
        bool sentinel = true;
        for (std::uint32_t i = 0; i < M7_RB_FIELDS * N; ++i) sentinel &= result[i] == 0xABCD0000u + i;
        const bool held = std::memcmp(result, kept, sizeof kept) == 0;
        const bool untouched = sentinel || held;
        std::printf("[reject] status=%u untouched=%d (%s) armed=%d\n", status, untouched,
                    sentinel ? "sentinel kept" : held ? "previous block contents kept" : "WRITTEN", armed);
        rc |= !(status == 3 && untouched && armed);
    }
    std::printf(rc ? "TB_OBJECT_ROI_AXI FAIL\n" : "TB_OBJECT_ROI_AXI PASS\n");
    return rc ? 1 : 0;
}
