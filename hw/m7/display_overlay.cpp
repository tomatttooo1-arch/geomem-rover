#include "display_overlay.hpp"

namespace {

static std::uint32_t rgb(std::uint32_t r, std::uint32_t g, std::uint32_t b) {
    return (r << 16) | (g << 8) | b;
}

// Fixed-point red -> yellow -> cyan -> blue. Integer endpoints are exact.
static std::uint32_t depth_colour(std::uint16_t z) {
    if (z < 100 || z > 9999) return 0x000000;
    if (z <= 2000) return rgb(255, (static_cast<std::uint32_t>(z) - 100) * 255 / 1900, 0);
    if (z <= 5000) return rgb((5000 - static_cast<std::uint32_t>(z)) * 255 / 3000,
                              255, (static_cast<std::uint32_t>(z) - 2000) * 255 / 3000);
    return rgb(0, (9999 - static_cast<std::uint32_t>(z)) * 255 / 4999, 255);
}

#define GLYPH(a,b,c,d,e) ((a) | ((b) << 3) | ((c) << 6) | ((d) << 9) | ((e) << 12))
// Three columns, five rows. This tiny fixed alphabet keeps hardware bounded.
static std::uint32_t glyph(char c) {
    switch (c) {
    case '0': return GLYPH(7,5,5,5,7);
    case '1': return GLYPH(2,6,2,2,7);
    case '2': return GLYPH(7,1,7,4,7);
    case '3': return GLYPH(7,1,7,1,7);
    case '4': return GLYPH(5,5,7,1,1);
    case '5': return GLYPH(7,4,7,1,7);
    case '6': return GLYPH(7,4,7,5,7);
    case '7': return GLYPH(7,1,1,1,1);
    case '8': return GLYPH(7,5,7,5,7);
    case '9': return GLYPH(7,5,7,1,7);
    case 'A': return GLYPH(2,5,7,5,5);
    case 'B': return GLYPH(6,5,6,5,6);
    case 'C': return GLYPH(3,4,4,4,3);
    case 'D': return GLYPH(6,5,5,5,6);
    case 'E': return GLYPH(7,4,6,4,7);
    case 'F': return GLYPH(7,4,6,4,4);
    case 'H': return GLYPH(5,5,7,5,5);
    case 'I': return GLYPH(7,2,2,2,7);
    case 'J': return GLYPH(1,1,1,5,7);
    case 'K': return GLYPH(5,5,6,5,5);
    case 'L': return GLYPH(4,4,4,4,7);
    case 'M': return GLYPH(5,7,7,5,5);
    case 'N': return GLYPH(5,7,7,7,5);
    case 'O': return GLYPH(7,5,5,5,7);
    case 'P': return GLYPH(6,5,6,4,4);
    case 'R': return GLYPH(6,5,6,5,5);
    case 'S': return GLYPH(3,4,7,1,6);
    case 'T': return GLYPH(7,2,2,2,2);
    case 'U': return GLYPH(5,5,5,5,7);
    case 'Z': return GLYPH(7,1,2,4,7);
    case 'm': return GLYPH(0,7,7,5,5);
    case 's': return GLYPH(0,3,4,1,6);
    case ':': return GLYPH(0,2,0,2,0);
    case '.': return GLYPH(0,0,0,0,2);
    case '?': return GLYPH(7,1,2,0,2);
    case '>': return GLYPH(4,2,1,2,4);
    default: return 0;
    }
}
#undef GLYPH

static char decimal_digit(std::uint32_t n) { return static_cast<char>('0' + n % 10); }

static char label_character(std::uint32_t segment, std::uint32_t index,
                            std::uint32_t mode, std::uint16_t nearest,
                            std::uint32_t age_ms, std::uint32_t pose_state,
                            std::uint32_t evidence) {
    const bool fresh = mode == 0;
    if (segment == 0) {
        // Z is explicitly optical-axis depth. Omit the previous number on loss.
        if (index == 0) return 'Z';
        if (index == 1) return ':';
        if (!fresh) return index >= 2 && index <= 3 ? '?' :
                           index == 4 ? '.' :
                           index >= 5 && index <= 6 ? '?' :
                           index == 7 ? 'm' : ' ';
        const std::uint32_t cm = nearest / 10; // truncation error < 1 cm
        if (index == 2) return decimal_digit(cm / 1000);
        if (index == 3) return decimal_digit(cm / 100);
        if (index == 4) return '.';
        if (index == 5) return decimal_digit(cm / 10);
        if (index == 6) return decimal_digit(cm);
        if (index == 7) return 'm';
        return ' ';
    }
    if (segment == 1) {
        if (!fresh) {
            static const char bad[] = "BADFRAME";
            static const char stale[] = "STALE";
            static const char pose[] = "POSEBAD";
            static const char nodata[] = "NODATA";
            if (mode == 1) return index < 8 ? bad[index] : ' ';
            if (mode == 2) return index < 5 ? stale[index] : ' ';
            if (mode == 3) return index < 7 ? pose[index] : ' ';
            return index < 6 ? nodata[index] : ' ';
        }
        if (index == 0) return 'A';
        if (index == 1) return ':';
        if (age_ms > 999) {
            static const char large[] = ">999ms";
            return index >= 2 && index < 8 ? large[index-2] : ' ';
        }
        if (index == 2) return decimal_digit(age_ms / 100);
        if (index == 3) return decimal_digit(age_ms / 10);
        if (index == 4) return decimal_digit(age_ms);
        if (index == 5) return 'm';
        if (index == 6) return 's';
        return ' ';
    }
    if (segment == 2) {
        static const char text_ok[] = "POSE OK";
        static const char text_bad[] = "POSE BAD";
        static const char text_unknown[] = "POSE ?";
        if (pose_state == 1) return index < 7 ? text_ok[index] : ' ';
        if (pose_state == 2) return index < 8 ? text_bad[index] : ' ';
        return index < 6 ? text_unknown[index] : ' ';
    }
    if (segment == 3) {
        static const char unknown[] = "OBJ ?";
        static const char present[] = "OBJ HIT";
        static const char conflict[] = "OBJ FREE?";
        static const char occluded[] = "OBJ OCCL";
        if (!fresh || evidence == 0) return index < 5 ? unknown[index] : ' ';
        if (evidence == 1) return index < 7 ? present[index] : ' ';
        // 3 comes only from the mask-free region check (OBJECT_ROI_SPEC s.8).
        if (evidence == 3) return index < 8 ? occluded[index] : ' ';
        return index < 9 ? conflict[index] : ' ';
    }
    static const char risk[] = "RISK ?";
    return index < 6 ? risk[index] : ' ';
}

static std::uint32_t panel_pixel(std::uint32_t x, std::uint32_t y,
                                 std::uint32_t width, std::uint32_t scale,
                                 std::uint32_t panel_height, std::uint32_t mode,
                                 std::uint16_t nearest, std::uint32_t age_ms,
                                 std::uint32_t pose_state, std::uint32_t evidence) {
    const bool fresh = mode == 0;
    const std::uint32_t unit = width / 5;
    const std::uint32_t bounds[4] = {unit, unit*2, unit*3, unit*4};
    const std::uint32_t segment = x < bounds[0] ? 0 :
                                  x < bounds[1] ? 1 :
                                  x < bounds[2] ? 2 :
                                  x < bounds[3] ? 3 : 4;
    const std::uint32_t left = segment * unit;
    std::uint32_t background = 0x102020;
    if (segment == 0) background = fresh ? 0x203040 : 0x604010;
    if (segment == 1) background = fresh ? 0x104020 : 0x701010;
    if (segment == 2) background = pose_state == 1 ? 0x104020 :
                                   pose_state == 2 ? 0x701010 : 0x303030;
    if (segment == 3) background = !fresh || evidence == 0 ? 0x303030 :
                                   evidence == 1 ? 0x104020 :
                                   evidence == 3 ? 0x203060 : 0x605010;
    if (segment == 4) background = 0x502050; // risk algorithm absent
    if (x == left || y == panel_height-1) return 0xFFFFFF;
    const std::uint32_t top = scale == 3 ? 2 : 3;
    const std::uint32_t margin = 2 * scale;
    if (y < top || y >= top + 5*scale || x < left+margin) return background;
    const std::uint32_t local = x - left - margin;
    const std::uint32_t char_column = scale == 3 ? local / 3 : local;
    const std::uint32_t column = char_column & 3;
    if (column >= 3) return background;
    const char c = label_character(segment, char_column >> 2, mode, nearest,
                                   age_ms, pose_state, evidence);
    const std::uint32_t glyph_row = scale == 3 ? (y-top)/3 : y-top;
    const std::uint32_t row_bits = glyph(c) >> (glyph_row*3);
    return (row_bits >> (2-column)) & 1 ? 0xFFFFFF : background;
}

} // namespace

extern "C" void fpga_m7_display_overlay(
    const std::uint16_t depth_mm[FPGA_M7_DISPLAY_MAX_PIXELS],
    std::uint32_t width, std::uint32_t height,
    std::uint32_t frame_valid, std::uint32_t pose_valid,
    std::uint32_t age_ms, std::uint32_t max_age_ms,
    std::uint32_t object_evidence,
    std::uint32_t rgb888[FPGA_M7_DISPLAY_MAX_PIXELS],
    std::uint32_t* status) {
    *status = 1;
    if (width == 0 || height == 0 || width > FPGA_M7_DISPLAY_MAX_WIDTH ||
        height > FPGA_M7_DISPLAY_MAX_HEIGHT) return;
    const bool malformed = frame_valid > 1 || pose_valid > 1 || object_evidence > 3;
    *status = malformed ? 2 : 0;
    const bool controls_fresh = !malformed && frame_valid == 1 &&
                                pose_valid == 1 && age_ms <= max_age_ms;
    std::uint16_t nearest = 65535;
    if (controls_fresh) {
        // Independent same-frame minimum prevents an old scalar from being
        // displayed beside a new depth image.
        const std::uint32_t pixels = width * height;
        for (std::uint32_t i = 0; i < pixels; ++i) {
            const std::uint16_t z = depth_mm[i];
            if (z >= 100 && z <= 9999 && z < nearest) nearest = z;
        }
    }
    const bool has_depth = nearest != 65535;
    // mode: 0 fresh, 1 bad frame/control, 2 expired, 3 bad pose, 4 no depth.
    const std::uint32_t mode = malformed || frame_valid == 0 ? 1 :
                               age_ms > max_age_ms ? 2 :
                               pose_valid == 0 ? 3 :
                               !has_depth ? 4 : 0;
    const bool fresh = mode == 0;
    const std::uint32_t pose_state = mode == 0 ? 1 : mode == 3 ? 2 : 0;
    // Threefold glyphs on normal video sizes are readable at native pixels;
    // the small-scene layout remains defined and clips labels if needed.
    const std::uint32_t scale = width >= 640 && height >= 24 ? 3 : 1;
    const std::uint32_t panel_height = scale == 3 ? 20 : 12;
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::uint32_t i = y * width + x;
            if (y < panel_height) {
                rgb888[i] = panel_pixel(x, y, width, scale, panel_height, mode,
                                        nearest, age_ms, pose_state,
                                        fresh ? object_evidence : 0);
            } else if (!controls_fresh) {
                // Checker is visibly different from an invalid depth's black.
                rgb888[i] = ((x / 8 + y / 8) & 1) ? 0xA000A0 : 0x180018;
            } else {
                rgb888[i] = depth_colour(depth_mm[i]);
            }
        }
    }
}
