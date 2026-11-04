// Positive controls for fpga_m7_object_roi_batch (mask-free multi-box check).
// Every FPGA_M7_ROI_MUTANT_* build of the kernel must make this exit nonzero.
#include "object_evidence.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

static float depth[FPGA_M7_OBJECT_MAX_PIXELS];
static std::uint32_t enabled[FPGA_M7_ROI_MAX_BOXES + 1];
static std::uint32_t xyxy[4 * (FPGA_M7_ROI_MAX_BOXES + 1)];
static double near_z[FPGA_M7_ROI_MAX_BOXES + 1];
static double far_z[FPGA_M7_ROI_MAX_BOXES + 1];
static std::uint32_t ids[FPGA_M7_ROI_MAX_BOXES + 1];
static std::uint32_t result[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES];
static std::uint32_t status;
static const std::uint32_t W = 40, H = 30;
static const std::uint32_t SENTINEL = 0xFFFFFFFFu;
static int failures = 0;

static void expect(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

static void fill_rect(std::uint32_t x0, std::uint32_t y0, std::uint32_t x1,
                      std::uint32_t y1, float z) {
    for (std::uint32_t y = y0; y <= y1; ++y)
        for (std::uint32_t x = x0; x <= x1; ++x) depth[y * W + x] = z;
}

static void set_box(std::uint32_t b, std::uint32_t on, std::uint32_t x0,
                    std::uint32_t y0, std::uint32_t x1, std::uint32_t y1,
                    double near_value, double far_value, std::uint32_t id) {
    enabled[b] = on;
    xyxy[4 * b] = x0; xyxy[4 * b + 1] = y0; xyxy[4 * b + 2] = x1; xyxy[4 * b + 3] = y1;
    near_z[b] = near_value; far_z[b] = far_value; ids[b] = id;
}

static void clear_result() {
    for (std::uint32_t i = 0; i < M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES; ++i) result[i] = SENTINEL;
    status = SENTINEL;
}

static void run(std::uint32_t count, std::uint32_t min_pixels = 9,
                double margin = .04, double valid_fraction = .7,
                double free_fraction = .9, std::uint32_t width = W) {
    clear_result();
    fpga_m7_object_roi_batch(depth, width, H, count, enabled, xyxy, near_z, far_z,
                            ids, margin, min_pixels, valid_fraction, free_fraction,
                            result, &status);
}

static bool row_is(std::uint32_t b, std::uint32_t id, std::uint32_t on,
                   std::uint32_t pixels, std::uint32_t valid, std::uint32_t front,
                   std::uint32_t behind, std::uint32_t verdict) {
    const std::uint32_t* r = result + M7_RB_FIELDS * b;
    const bool ok = r[M7_RB_OBJECT_ID] == id && r[M7_RB_ENABLED] == on &&
                    r[M7_RB_PIXELS] == pixels && r[M7_RB_VALID] == valid &&
                    r[M7_RB_FRONT] == front && r[M7_RB_BEHIND] == behind &&
                    r[M7_RB_VERDICT] == verdict;
    if (!ok)
        std::printf("  row %u got id=%u en=%u pix=%u valid=%u front=%u behind=%u code=%u\n",
                    b, r[0], r[1], r[2], r[3], r[4], r[5], r[6]);
    return ok;
}

static bool untouched_from(std::uint32_t first_row) {
    for (std::uint32_t i = M7_RB_FIELDS * first_row; i < M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES; ++i)
        if (result[i] != SENTINEL) return false;
    return true;
}

static void scene() {
    // Wall at 3 m behind everything. Boxes are historical object volumes at
    // camera Z 1.9..2.1 m unless noted (front=1.86, back=2.14 with margin).
    for (std::uint32_t i = 0; i < W * H; ++i) depth[i] = 3.f;
    fill_rect(2, 2, 6, 6, 2.f);                // A: object still there
    // B: object removed; 2 of 25 pixels still hit something at 2 m -> 23/25.
    depth[2 * W + 10] = 2.f; depth[3 * W + 10] = 2.f;
    // C: exactly 12/24 in front (6x4 box, two rows of six at 1 m).
    fill_rect(20, 2, 25, 3, 1.f);
    // D: 11/24 in front, rest on the surface -> no conclusion.
    fill_rect(28, 2, 33, 5, 2.f);
    fill_rect(28, 2, 33, 2, 1.f); fill_rect(28, 3, 32, 3, 1.f);
    // E: removed but 3/25 still at 2 m -> 22/25 behind, below .9.
    depth[10 * W + 2] = 2.f; depth[10 * W + 3] = 2.f; depth[10 * W + 4] = 2.f;
    // F: 8/25 invalid (NaN or 0) -> 17/25 valid fraction .68.
    for (std::uint32_t k = 0; k < 8; ++k)
        depth[(10 + k / 5) * W + 10 + k % 5] = k % 2 ? std::numeric_limits<float>::quiet_NaN() : 0.f;
    // G: x1 column differs from interior (catches inclusive-end mistakes):
    // columns 20..22 wall, column 23 at 1 m -> 5/20 front, 15/20 behind.
    fill_rect(23, 10, 23, 14, 1.f);
    // T: thin object (1 cm) removed; table 2 cm behind its far face.
    fill_rect(28, 10, 32, 14, 2.02f);
}

int main() {
    scene();
    set_box(0, 1, 2, 2, 6, 6, 1.9, 2.1, 100);      // A
    set_box(1, 1, 10, 2, 14, 6, 1.9, 2.1, 101);    // B
    set_box(2, 1, 20, 2, 25, 5, 1.9, 2.1, 102);    // C
    set_box(3, 1, 28, 2, 33, 5, 1.9, 2.1, 103);    // D
    set_box(4, 1, 2, 10, 6, 14, 1.9, 2.1, 104);    // E
    set_box(5, 1, 10, 10, 14, 14, 1.9, 2.1, 105);  // F
    set_box(6, 1, 20, 10, 23, 14, 1.9, 2.1, 106);  // G
    set_box(7, 1, 28, 10, 32, 14, 2.0, 2.01, 107); // T
    // Disabled row: garbage geometry must not be read or rejected.
    set_box(8, 0, 9999, 0, 1, 9999, std::nan(""), -1., 108);
    // Overlap: H covers A and B's left neighbourhood; I == A exactly.
    set_box(9, 1, 4, 2, 12, 6, 1.9, 2.1, 109);
    set_box(10, 1, 2, 2, 6, 6, 1.9, 2.1, 110);
    run(11);
    expect(status == 0, "accepted batch");
    expect(row_is(0, 100, 1, 25, 25, 0, 0, M7_RB_INDETERMINATE), "A present -> 0");
    expect(row_is(1, 101, 1, 25, 25, 0, 23, M7_RB_FREE_SPACE_CONFLICT), "B removed 23/25 -> 2");
    expect(row_is(2, 102, 1, 24, 24, 12, 12, M7_RB_OCCLUDED), "C 12/24 front -> 3");
    expect(row_is(3, 103, 1, 24, 24, 11, 0, M7_RB_INDETERMINATE), "D 11/24 front -> 0");
    expect(row_is(4, 104, 1, 25, 25, 0, 22, M7_RB_INDETERMINATE), "E 22/25 behind -> 0");
    expect(row_is(5, 105, 1, 25, 17, 0, 17, M7_RB_INDETERMINATE), "F 17/25 valid -> 0");
    expect(row_is(6, 106, 1, 20, 20, 5, 15, M7_RB_INDETERMINATE), "G x1 column counted");
    expect(row_is(7, 107, 1, 25, 25, 0, 0, M7_RB_INDETERMINATE), "T thin object never free");
    expect(row_is(8, 108, 0, 0, 0, 0, 0, M7_RB_INDETERMINATE), "disabled row zeroed");
    // H spans x 4..12, y 2..6: 45 pixels. x4..6 at 2 m (15), x7..9 wall
    // (15), x10..12 wall except (10,2),(10,3) at 2 m -> 13 behind.
    expect(row_is(9, 109, 1, 45, 45, 0, 28, M7_RB_INDETERMINATE), "H overlap counted fully");
    expect(row_is(10, 110, 1, 25, 25, 0, 0, M7_RB_INDETERMINATE), "I duplicate of A counted");
    expect(untouched_from(11), "rows past box_count untouched");

    // Exact free boundary: 18/20 behind = .9 -> free.
    for (std::uint32_t i = 0; i < W * H; ++i) depth[i] = 3.f;
    depth[20 * W + 2] = 2.f; depth[20 * W + 3] = 2.f;
    set_box(0, 1, 2, 20, 6, 23, 1.9, 2.1, 7);
    set_box(1, 1, 2, 20, 6, 23, 1.9, 2.1, 8);
    run(2);
    expect(status == 0 && row_is(0, 7, 1, 20, 20, 0, 18, M7_RB_FREE_SPACE_CONFLICT),
           "18/20 behind is free");
    // Same box with stricter free fraction stays indeterminate.
    run(2, 9, .04, .7, .95);
    expect(status == 0 && row_is(1, 8, 1, 20, 20, 0, 18, M7_RB_INDETERMINATE),
           "stricter free fraction respected");
    // Too few valid pixels: 4x4 box with 8 valid.
    for (std::uint32_t i = 0; i < W * H; ++i) depth[i] = 3.f;
    fill_rect(30, 20, 33, 21, std::numeric_limits<float>::quiet_NaN());
    set_box(0, 1, 30, 20, 33, 23, 1.9, 2.1, 9);
    run(1);
    expect(status == 0 && row_is(0, 9, 1, 16, 8, 0, 8, M7_RB_INDETERMINATE),
           "valid pixels below min_pixels");
    expect(untouched_from(1), "single-row batch writes one row");

    // Capacity: N boxes accepted, each with its own counts; N+1 rejected.
    scene();
    for (std::uint32_t b = 0; b < FPGA_M7_ROI_MAX_BOXES; ++b) {
        // Alternate between B (free) and C (occluded) so every row differs
        // from row 0 in at least the verdict.
        if (b % 2) set_box(b, 1, 20, 2, 25, 5, 1.9, 2.1, 1000 + b);
        else set_box(b, 1, 10, 2, 14, 6, 1.9, 2.1, 1000 + b);
    }
    run(FPGA_M7_ROI_MAX_BOXES);
    bool all_rows = status == 0;
    for (std::uint32_t b = 0; b < FPGA_M7_ROI_MAX_BOXES; ++b)
        all_rows = all_rows && (b % 2 ? row_is(b, 1000 + b, 1, 24, 24, 12, 12, M7_RB_OCCLUDED)
                                      : row_is(b, 1000 + b, 1, 25, 25, 0, 23,
                                               M7_RB_FREE_SPACE_CONFLICT));
    expect(all_rows, "N boxes each counted");
    set_box(FPGA_M7_ROI_MAX_BOXES, 1, 10, 2, 14, 6, 1.9, 2.1, 77);
    run(FPGA_M7_ROI_MAX_BOXES + 1);
    expect(status == 3 && untouched_from(0), "N+1 boxes rejected untouched");

    // Rejections leave result untouched.
    set_box(0, 1, 10, 2, 14, 6, 1.9, 2.1, 1);
    run(0);
    expect(status == 3 && untouched_from(0), "zero boxes rejected");
    set_box(0, 2, 10, 2, 14, 6, 1.9, 2.1, 1);
    run(1);
    expect(status == 3 && untouched_from(0), "enabled flag 2 rejected");
    const std::uint32_t bad_boxes[6][4] = {
        {10, 2, 9, 6}, {10, 2, 40, 6}, {10, 2, 12, 6}, {10, 2, 14, 30},
        {10, 2, 14, 4}, {10, 7, 14, 6}};
    for (int k = 0; k < 6; ++k) {
        set_box(0, 1, bad_boxes[k][0], bad_boxes[k][1], bad_boxes[k][2], bad_boxes[k][3],
                1.9, 2.1, 1);
        run(1);
        expect(status == 3 && untouched_from(0), "bad rectangle rejected");
    }
    const double bad_z[5][2] = {{.1, 2.1}, {2.0, 1.9}, {std::nan(""), 2.1},
                                {1.9, std::numeric_limits<double>::infinity()},
                                {1.9, 1e300}};
    for (int k = 0; k < 5; ++k) {
        set_box(0, 1, 10, 2, 14, 6, bad_z[k][0], bad_z[k][1], 1);
        run(1);
        expect(status == 3 && untouched_from(0), "bad camera-Z extent rejected");
    }
    // A bad second row still rejects the whole batch.
    set_box(0, 1, 10, 2, 14, 6, 1.9, 2.1, 1);
    set_box(1, 1, 10, 2, 14, 6, 2.0, 1.9, 2);
    run(2);
    expect(status == 3 && untouched_from(0), "one bad row rejects batch");
    set_box(1, 1, 10, 2, 14, 6, 1.9, 2.1, 2);
    run(2, 8);
    expect(status == 4 && untouched_from(0), "min_pixels below 9 rejected");
    run(2, 9, .03);
    expect(status == 4 && untouched_from(0), "margin below .04 rejected");
    run(2, 9, .04, .69);
    expect(status == 4 && untouched_from(0), "valid fraction below .7 rejected");
    run(2, 9, .04, .7, .89);
    expect(status == 4 && untouched_from(0), "free fraction below .9 rejected");
    run(2, 9, .04, .7, .9, 0);
    expect(status == 1 && untouched_from(0), "zero width rejected");
    run(2, 9, .04, .7, .9, FPGA_M7_OBJECT_MAX_WIDTH + 1);
    expect(status == 1 && untouched_from(0), "oversize width rejected");

    if (failures) {
        std::printf("object ROI testbench: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("object ROI testbench passed\n");
    return 0;
}
