#include "display_overlay.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>

static std::uint16_t depth_mm[FPGA_M7_DISPLAY_MAX_PIXELS];
static std::uint32_t output[FPGA_M7_DISPLAY_MAX_PIXELS];

static bool expect(bool condition, const char* message) {
    if (!condition) std::fprintf(stderr, "display test failure: %s\n", message);
    return condition;
}

#define BITS(a,b,c,d,e) ((a) | ((b)<<3) | ((c)<<6) | ((d)<<9) | ((e)<<12))
static std::uint32_t expected_glyph(char c) {
    switch (c) {
    case '0': case 'O': return BITS(7,5,5,5,7);
    case '1': return BITS(2,6,2,2,7);
    case '2': return BITS(7,1,7,4,7);
    case '3': return BITS(7,1,7,1,7);
    case '5': return BITS(7,4,7,1,7);
    case '9': return BITS(7,5,7,1,7);
    case 'A': return BITS(2,5,7,5,5);
    case 'B': return BITS(6,5,6,5,6);
    case 'D': return BITS(6,5,5,5,6);
    case 'E': return BITS(7,4,6,4,7);
    case 'F': return BITS(7,4,6,4,4);
    case 'H': return BITS(5,5,7,5,5);
    case 'I': return BITS(7,2,2,2,7);
    case 'J': return BITS(1,1,1,5,7);
    case 'K': return BITS(5,5,6,5,5);
    case 'L': return BITS(4,4,4,4,7);
    case 'M': return BITS(5,7,7,5,5);
    case 'N': return BITS(5,7,7,7,5);
    case 'P': return BITS(6,5,6,4,4);
    case 'R': return BITS(6,5,6,5,5);
    case 'S': return BITS(3,4,7,1,6);
    case 'T': return BITS(7,2,2,2,2);
    case 'U': return BITS(5,5,5,5,7);
    case 'Z': return BITS(7,1,2,4,7);
    case 'm': return BITS(0,7,7,5,5);
    case 's': return BITS(0,3,4,1,6);
    case ':': return BITS(0,2,0,2,0);
    case '.': return BITS(0,0,0,0,2);
    case '?': return BITS(7,1,2,0,2);
    case '>': return BITS(4,2,1,2,4);
    default: return 0;
    }
}
#undef BITS

static bool expect_label(std::uint32_t width, std::uint32_t height,
                         std::uint32_t segment, const char* expected) {
    const std::uint32_t scale = width >= 640 && height >= 24 ? 3 : 1;
    const std::uint32_t top = scale == 3 ? 2 : 3;
    const std::uint32_t left = segment * (width/5) + 2*scale;
    for (std::uint32_t index = 0;
         index < static_cast<std::uint32_t>(std::strlen(expected)); ++index) {
        const std::uint32_t glyph = expected_glyph(expected[index]);
        for (std::uint32_t row = 0; row < 5; ++row) {
            for (std::uint32_t col = 0; col < 3; ++col) {
                const bool white = (glyph >> (row*3 + 2-col)) & 1;
                for (std::uint32_t dy = 0; dy < scale; ++dy)
                    for (std::uint32_t dx = 0; dx < scale; ++dx) {
                        const std::uint32_t pixel = output[(top+row*scale+dy)*width +
                            left + index*4*scale + col*scale + dx];
                        if ((pixel == 0xFFFFFF) != white) {
                            std::fprintf(stderr,
                                "label pixel mismatch segment %u char %u (%c) row %u col %u\n",
                                segment, index, expected[index], row, col);
                            return false;
                        }
                    }
            }
        }
    }
    return true;
}

int main() {
    const std::uint32_t w = 32, h = 20, n = w*h;
    std::uint32_t status = 99;
    for (std::uint32_t i = 0; i < n; ++i) {
        depth_mm[i] = 0;
        output[i] = 0xDEADBEEF;
    }
    depth_mm[12*w+0] = 100;
    depth_mm[12*w+1] = 2000;
    depth_mm[12*w+2] = 5000;
    depth_mm[12*w+3] = 9999;
    depth_mm[12*w+4] = 99;
    depth_mm[12*w+5] = 10000;
    fpga_m7_display_overlay(depth_mm, w, h, 1, 1, 199, 200, 1, output, &status);
    bool ok = expect(status == 0, "valid status");
    ok &= expect(output[12*w+0] == 0xFF0000, "100 mm red");
    ok &= expect(output[12*w+1] == 0xFFFF00, "2000 mm yellow");
    ok &= expect(output[12*w+2] == 0x00FFFF, "5000 mm cyan");
    ok &= expect(output[12*w+3] == 0x0000FF, "9999 mm blue");
    ok &= expect(output[12*w+4] == 0 && output[12*w+5] == 0,
                 "invalid depth black");
    for (std::uint32_t i = 0; i < n; ++i)
        ok &= expect(output[i] != 0xDEADBEEF, "every pixel written");

    // The same output allocation must not retain previous colour or text.
    for (std::uint32_t i = 0; i < n; ++i) depth_mm[i] = 0;
    fpga_m7_display_overlay(depth_mm, w, h, 1, 1, 200, 200, 1, output, &status);
    ok &= expect(status == 0 && output[12*w+0] == 0,
                 "all-invalid frame clears previous depth colour");

    depth_mm[12*w+0] = 100;
    fpga_m7_display_overlay(depth_mm, w, h, 1, 1, 201, 200, 1, output, &status);
    const std::uint32_t stale_body = output[12*w+0];
    const std::uint32_t stale_object_cell = output[1*w+22];
    ok &= expect(status == 0 && stale_body != 0 && stale_body != 0xFF0000,
                 "expired frame uses checker rather than old depth");
    fpga_m7_display_overlay(depth_mm, w, h, 1, 1, 201, 200, 0, output, &status);
    ok &= expect(output[12*w+0] == stale_body &&
                 output[1*w+22] == stale_object_cell,
                 "expired object evidence is hidden");
    fpga_m7_display_overlay(depth_mm, w, h, 0, 1, 0, 200, 1, output, &status);
    ok &= expect(status == 0 && output[12*w+0] == stale_body,
                 "bad frame is visibly unavailable");
    fpga_m7_display_overlay(depth_mm, w, h, 1, 0, 0, 200, 1, output, &status);
    ok &= expect(status == 0 && output[12*w+0] == stale_body,
                 "bad pose is visibly unavailable");
    fpga_m7_display_overlay(depth_mm, w, h, 2, 1, 0, 200, 1, output, &status);
    ok &= expect(status == 2 && output[12*w+0] == stale_body,
                 "malformed control paints fail-closed checker");

    output[0] = 0xDEADBEEF;
    fpga_m7_display_overlay(depth_mm, 1281, h, 1, 1, 0, 200, 0, output, &status);
    ok &= expect(status == 1 && output[0] == 0xDEADBEEF,
                 "invalid dimensions do not write output");

    // At 320 pixels each of the five status cells is 64 pixels wide. The
    // original 32-wide RTL scene displayed only the first one or two glyphs.
    // Check every pixel of the complete labels from actual rendered RGB.
    const std::uint32_t wide = 320, small_height = 20;
    for (std::uint32_t i = 0; i < wide*small_height; ++i) {
        depth_mm[i] = 1230;
        output[i] = 0xDEADBEEF;
    }
    fpga_m7_display_overlay(depth_mm, wide, small_height,
                           1, 1, 5, 2000, 1, output, &status);
    ok &= expect(status == 0, "wide status call accepted");
    ok &= expect(expect_label(wide, small_height, 0, "Z:01.23m"), "optical-Z digits and point");
    ok &= expect(expect_label(wide, small_height, 1, "A:005ms"), "real age digits");
    ok &= expect(expect_label(wide, small_height, 2, "POSE OK"), "valid pose marker");
    ok &= expect(expect_label(wide, small_height, 3, "OBJ HIT"), "same-frame object present");
    ok &= expect(expect_label(wide, small_height, 4, "RISK ?"), "risk stays unknown");
    depth_mm[12*wide] = 2000;
    fpga_m7_display_overlay(depth_mm, wide, small_height,
                           1, 1, 1001, 2000, 2, output, &status);
    ok &= expect(expect_label(wide, small_height, 1, "A:>999ms"), "large age not misreported as 999");
    ok &= expect(expect_label(wide, small_height, 3, "OBJ FREE?"), "one free conflict is tentative");
    fpga_m7_display_overlay(depth_mm, wide, small_height,
                           1, 1, 2001, 2000, 2, output, &status);
    ok &= expect(expect_label(wide, small_height, 0, "Z:??.??m"), "stale Z number suppressed");
    ok &= expect(expect_label(wide, small_height, 1, "STALE"), "stale age state");
    ok &= expect(expect_label(wide, small_height, 2, "POSE ?"), "stale pose unknown");
    ok &= expect(expect_label(wide, small_height, 3, "OBJ ?"), "stale object hidden");
    ok &= expect(expect_label(wide, small_height, 4, "RISK ?"), "stale risk remains unknown");
    ok &= expect(output[12*wide] != 0xFF0000 && output[12*wide] != 0xFFFF00,
                 "stale body is checker");

    // Native-width display uses threefold 15-pixel glyphs in a 20-pixel bar.
    const std::uint32_t full = 640, tall = 24;
    for (std::uint32_t i = 0; i < full*tall; ++i) {
        depth_mm[i] = 2000;
        output[i] = 0xDEADBEEF;
    }
    fpga_m7_display_overlay(depth_mm, full, tall,
                           1, 1, 5, 200, 2, output, &status);
    ok &= expect(status == 0, "scaled status call accepted");
    ok &= expect(expect_label(full, tall, 0, "Z:02.00m"), "scaled Z value");
    ok &= expect(expect_label(full, tall, 1, "A:005ms"), "scaled age value");
    ok &= expect(expect_label(full, tall, 2, "POSE OK"), "scaled pose marker");
    ok &= expect(expect_label(full, tall, 3, "OBJ FREE?"), "scaled object state");
    ok &= expect(expect_label(full, tall, 4, "RISK ?"), "scaled unknown risk");
    ok &= expect(output[20*full] == 0xFFFF00, "body begins after enlarged panel");
    if (!ok) return 1;
    std::puts("M7 display overlay C++ testbench PASS");
    return 0;
}
