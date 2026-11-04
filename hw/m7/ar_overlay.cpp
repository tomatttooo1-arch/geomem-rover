// AR overlay display core (D, 2026-10-11). See ar_overlay.hpp for buffers and
// AR_DISPLAY_SPEC_20261011.md for the per-element data sources.
//
// Structure: prologue (descriptor 49 beats -> registers, slice 64 beats ->
// BRAM, permit sampled once, panel/label text precomputed), minimum pass
// (8 depth pixels per beat), then a DATAFLOW region read -> render -> write.
// Paint priority, high to low: panel (y<40) > minimap (inner > border >
// label) > box labels > box lines (both only in mode 0) > depth rectangle >
// background 0x101010.
#include "ar_overlay.hpp"
#include <hls_stream.h>
#ifndef __SYNTHESIS__
#include <cassert>
#endif

namespace {

typedef std::uint32_t u32;
typedef ap_uint<8> u8;

// Pseudo-colour: legacy depth_colour() sampled at 32 mm segment centres.
static const u32 kLut[313] = {
#include "ar_overlay_lut.inc"
};

// The ONLY definition of valid depth; shared by colour and minimum.
static bool valid_mm(u32 z) { return z >= 100 && z <= 9999; }

#define G(a,b,c,d,e) ((a) | ((b) << 3) | ((c) << 6) | ((d) << 9) | ((e) << 12))
// 3x5 glyphs; row r at bits [3r+2:3r], bit 2 of a row is the left column.
// Unsupported characters are blank.
static u32 glyph(u32 c) {
    switch (c) {
    case '0': return G(7,5,5,5,7);
    case '1': return G(2,6,2,2,7);
    case '2': return G(7,1,7,4,7);
    case '3': return G(7,1,7,1,7);
    case '4': return G(5,5,7,1,1);
    case '5': return G(7,4,7,1,7);
    case '6': return G(7,4,7,5,7);
    case '7': return G(7,1,1,1,1);
    case '8': return G(7,5,7,5,7);
    case '9': return G(7,5,7,1,7);
    case 'A': return G(2,5,7,5,5);
    case 'B': return G(6,5,6,5,6);
    case 'C': return G(3,4,4,4,3);
    case 'D': return G(6,5,5,5,6);
    case 'E': return G(7,4,6,4,7);
    case 'F': return G(7,4,6,4,4);
    case 'G': return G(3,4,5,5,3);
    case 'H': return G(5,5,7,5,5);
    case 'I': return G(7,2,2,2,7);
    case 'J': return G(1,1,1,5,7);
    case 'K': return G(5,5,6,5,5);
    case 'L': return G(4,4,4,4,7);
    case 'M': return G(5,7,7,5,5);
    case 'N': return G(5,7,7,7,5);
    case 'O': return G(7,5,5,5,7);
    case 'P': return G(6,5,6,4,4);
    case 'Q': return G(2,5,5,6,3);
    case 'R': return G(6,5,6,5,5);
    case 'S': return G(3,4,7,1,6);
    case 'T': return G(7,2,2,2,2);
    case 'U': return G(5,5,5,5,7);
    case 'V': return G(5,5,5,5,2);
    case 'W': return G(5,5,7,7,5);
    case 'X': return G(5,5,2,5,5);
    case 'Y': return G(5,5,2,2,2);
    case 'Z': return G(7,1,2,4,7);
    case ':': return G(0,2,0,2,0);
    case '.': return G(0,0,0,0,2);
    case '?': return G(7,1,2,0,2);
    case '>': return G(4,2,1,2,4);
    case '/': return G(1,1,2,4,4);
    case '-': return G(0,0,7,0,0);
    case '+': return G(0,2,7,2,0);
    case '=': return G(0,7,0,7,0);
    default: return 0;
    }
}
#undef G

static bool glyph_on(u32 c, u32 row, u32 col) {
    return ((glyph(c) >> (row * 3)) >> (2 - col)) & 1;
}

static u32 digit(u32 n) { return '0' + n % 10; }

// Fixed per-frame facts consumed by the per-pixel renderer.
struct Facts {
    u32 width, height, dx, dy, dx_end, dy_end;
    u32 mode, n_boxes, grid_on;
    int mx, my;            // minimap origin, clamped to 4095
    u32 cam_x, cam_z;
};

#define TXT(lit) do { static const char t_[] = lit; \
    return idx < sizeof(t_) - 1 ? static_cast<u32>(t_[idx]) : ' '; } while (0)

// Panel character at (row, col); col 0..52, 13 columns per segment.
static u32 panel_char(u32 row, u32 col, u32 mode, u32 replay, u32 nearest,
                      u32 age_valid, u32 age_ms, u32 permit_present,
                      bool permit_on, u32 evidence, const u32 fid[10],
                      u32 fid_n) {
    if (col >= 52) return ' ';
    const u32 seg = col / 13;
    const u32 idx = col % 13;
    const bool fresh = mode == 0;
    if (row == 0) {
        if (seg == 0) {
            if (!fresh) TXT("Z:??.??M");
            const u32 cm = nearest / 10;  // truncation, as in the legacy core
            if (idx == 0) return 'Z';
            if (idx == 1) return ':';
            if (idx == 2) return digit(cm / 1000);
            if (idx == 3) return digit(cm / 100);
            if (idx == 4) return '.';
            if (idx == 5) return digit(cm / 10);
            if (idx == 6) return digit(cm);
            if (idx == 7) return 'M';
            return ' ';
        }
        if (seg == 1) {
            if (fresh) { if (replay) TXT("REPLAY"); TXT("LIVE"); }
            if (mode == 1) TXT("BADFRAME");
            if (mode == 2) TXT("STALE");
            if (mode == 3) TXT("POSEBAD");
            if (mode == 4) TXT("NODATA");
            TXT("NOAGE");
        }
        if (seg == 2) {
            if (!age_valid) TXT("AGE N/A");
            if (age_ms > 999) TXT("AGE>999MS");
            if (idx < 4) TXT("AGE:");
            if (idx == 4) return digit(age_ms / 100);
            if (idx == 5) return digit(age_ms / 10);
            if (idx == 6) return digit(age_ms);
            if (idx == 7) return 'M';
            if (idx == 8) return 'S';
            return ' ';
        }
        if (!permit_present) TXT("DRV N/A");
        if (permit_on) TXT("DRIVE OK");
        TXT("STOP");
    }
    if (seg == 0) TXT("RISK N/A");  // no risk algorithm exists (spec s.4)
    if (seg == 1) {
        if (evidence == 0) TXT("OBJ ?");
        if (evidence == 1) TXT("OBJ HIT");
        if (evidence == 3) TXT("OBJ OCCL");
        TXT("OBJ FREE?");
    }
    if (seg == 2) {
        if (idx == 0) return 'F';
        if (idx == 1) return ':';
        if (idx - 2 < fid_n) return digit(fid[fid_n - 1 - (idx - 2)]);
        return ' ';
    }
    if (mode == 0) TXT("POSE OK");
    if (mode == 3) TXT("POSE BAD");
    TXT("POSE ?");
}
#undef TXT

static u32 signed4(int v, u32 k, u32 out[5]) {
    const bool big = v < -9999 || v > 9999;
    const u32 mag = big ? 0u : static_cast<u32>(v < 0 ? -v : v);
    out[0] = v < 0 ? '-' : '+';
    out[1] = big ? '?' : digit(mag / 1000);
    out[2] = big ? '?' : digit(mag / 100);
    out[3] = big ? '?' : digit(mag / 10);
    out[4] = big ? '?' : digit(mag);
    return out[k];
}

static void read_depth(const fpga_m7_ar_word* depth, u32 n,
                       hls::stream<fpga_m7_ar_word>& s) {
    for (u32 i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=38400 max=38400
        s.write(depth[i]);
    }
}

static void write_fb(hls::stream<fpga_m7_ar_word>& s, fpga_m7_ar_word* fb, u32 n) {
    for (u32 i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=76800 max=76800
        fb[i] = s.read();
    }
}

static void render(hls::stream<fpga_m7_ar_word>& in,
                   hls::stream<fpga_m7_ar_word>& out, const Facts f,
                   const u8 ptext[2][53], const u32 pbg[2][4],
                   const u8 slabel[15], const u8 slice[32][32],
                   const ap_uint<11> bx0[32], const ap_uint<11> bx1[32],
                   const ap_uint<11> by0[32], const ap_uint<11> by1[32],
                   const ap_uint<11> bly[32], const u32 bcol[32],
                   const u8 blab[32][6]) {
#pragma HLS ARRAY_PARTITION variable=bx0 complete
#pragma HLS ARRAY_PARTITION variable=bx1 complete
#pragma HLS ARRAY_PARTITION variable=by0 complete
#pragma HLS ARRAY_PARTITION variable=by1 complete
#pragma HLS ARRAY_PARTITION variable=bly complete
#pragma HLS ARRAY_PARTITION variable=bcol complete
#pragma HLS ARRAY_PARTITION variable=blab complete dim=0
#pragma HLS ARRAY_PARTITION variable=pbg complete dim=0
    const bool fresh = f.mode == 0;
    const u32 total = f.width * f.height;
    fpga_m7_ar_word cur = 0;
    fpga_m7_ar_word packed = 0;
    u32 x = 0, y = 0;
    for (u32 p = 0; p < total; ++p) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=307200 max=307200
        const bool in_depth = x >= f.dx && x < f.dx_end && y >= f.dy && y < f.dy_end;
        const u32 lane = (x - f.dx) & 7;
        if (in_depth && lane == 0) cur = in.read();
        const u32 z = cur.range(16 * lane + 15, 16 * lane);

        u32 colour = 0x101010;
        if (in_depth) {
            if (fresh) colour = valid_mm(z) ? kLut[z >> 5] : 0u;
            else colour = (((x >> 3) + (y >> 3)) & 1) ? 0xA000A0u : 0x180018u;
        }

        // Boxes (mode 0 only): lowest index on top; labels above all lines.
        // Only the hit tests are replicated per box; the winning label's glyph
        // is decoded once after the loop (one glyph decoder, not 32).
        bool line_hit = false, label_hit = false;
        u32 line_col = 0, label_idx = 0;
        for (int i = 31; i >= 0; --i) {
#pragma HLS UNROLL
            if (fresh && static_cast<u32>(i) < f.n_boxes) {
                const u32 X0 = bx0[i], X1 = bx1[i], Y0 = by0[i], Y1 = by1[i];
                const u32 LY = bly[i];
                if (x >= X0 && x <= X1 && y >= Y0 && y <= Y1 &&
                    (x <= X0 + 1 || x + 1 >= X1 || y <= Y0 + 1 || y + 1 >= Y1)) {
                    line_hit = true;
                    line_col = bcol[i];
                }
                if (x >= X0 && x < X0 + 48 && y >= LY && y < LY + 12) {
                    label_hit = true;
                    label_idx = static_cast<u32>(i);
                }
            }
        }
        if (line_hit) colour = line_col;
        if (label_hit) {
            const u32 rx = x - static_cast<u32>(bx0[label_idx]);
            const u32 ry = y - static_cast<u32>(bly[label_idx]);
            const u32 cx = rx & 7;
            const bool ink = cx >= 1 && cx <= 6 && ry >= 1 && ry <= 10 &&
                             glyph_on(blab[label_idx][rx >> 3], (ry - 1) >> 1, (cx - 1) >> 1);
            colour = ink ? 0u : bcol[label_idx];
        }

        // Minimap: inner 128x128 (4 px per cell), 1 px white border, label.
        const int rx = static_cast<int>(x) - f.mx;
        const int ry = static_cast<int>(y) - f.my;
        if (rx >= 0 && rx < 128 && ry >= 0 && ry < 128) {
            if (!fresh || !f.grid_on) {
                colour = ((x + y) & 7) < 2 ? 0xA000A0u : 0x180018u;
            } else {
                const u32 gx = static_cast<u32>(rx) >> 2;
                const u32 gz = 31 - (static_cast<u32>(ry) >> 2);  // far at top
                const u32 v = slice[gz][gx];
                colour = v == 0 ? 0x282828u : v == 1 ? 0x2E6B3Au :
                         v == 2 ? 0xFF3030u : 0xFF00FFu;
                if (gx == f.cam_x && gz == f.cam_z) colour = 0xFFFFFF;
            }
        } else if (rx >= -1 && rx <= 128 && ry >= -1 && ry <= 128) {
            colour = 0xFFFFFF;
        } else if (rx >= 0 && rx < 128 && ry >= -12 && ry < 0) {
            const u32 lx = static_cast<u32>(rx), ly = static_cast<u32>(ry + 12);
            const u32 ch = lx >> 3, cx = lx & 7;
            const u32 c = ch < 15 ? static_cast<u32>(slabel[ch]) : ' ';
            const bool ink = cx >= 1 && cx <= 6 && ly >= 1 && ly <= 10 &&
                             glyph_on(c, (ly - 1) >> 1, (cx - 1) >> 1);
            colour = ink ? 0xFFFFFFu : 0x202020u;
        }

        // Panel: 2 rows x 53 columns, 3x5 glyphs at scale 3, pitch 12.
        if (y < 40) {
            const u32 row = y < 20 ? 0 : 1;
            if (y == 39 || x == 0 || x == 156 || x == 312 || x == 468) {
                colour = 0xFFFFFF;
            } else if (x >= 636) {
                colour = 0x101010;
            } else {
                const u32 col = x / 12, sub = x % 12;
                const u32 gy0 = row == 0 ? 4 : 22;
                const bool ink = sub >= 2 && sub < 11 && y >= gy0 && y < gy0 + 15 &&
                                 glyph_on(ptext[row][col], (y - gy0) / 3, (sub - 2) / 3);
                colour = ink ? 0xFFFFFFu : pbg[row][x >= 468 ? 3 : x / 156];  // seg 3 = [468,636)
            }
        }

        packed.range(32 * (x & 3) + 31, 32 * (x & 3)) = colour;
        if ((x & 3) == 3) out.write(packed);
        if (x + 1 == f.width) { x = 0; ++y; } else { ++x; }
    }
}

static void render_region(const fpga_m7_ar_word* depth, fpga_m7_ar_word* fb,
                          u32 depth_words, u32 fb_words, const Facts f,
                          const u8 ptext[2][53], const u32 pbg[2][4],
                          const u8 slabel[15], const u8 slice[32][32],
                          const ap_uint<11> bx0[32], const ap_uint<11> bx1[32],
                          const ap_uint<11> by0[32], const ap_uint<11> by1[32],
                          const ap_uint<11> bly[32], const u32 bcol[32],
                          const u8 blab[32][6]) {
#pragma HLS DATAFLOW
    hls::stream<fpga_m7_ar_word> s_depth("s_depth");
    hls::stream<fpga_m7_ar_word> s_fb("s_fb");
#pragma HLS STREAM variable=s_depth depth=64
#pragma HLS STREAM variable=s_fb depth=64
    read_depth(depth, depth_words, s_depth);
    render(s_depth, s_fb, f, ptext, pbg, slabel, slice,
           bx0, bx1, by0, by1, bly, bcol, blab);
    write_fb(s_fb, fb, fb_words);
#ifndef __SYNTHESIS__
    // A pop-count mismatch would deadlock co-simulation; catch it in C.
    assert(s_depth.empty());
    assert(s_fb.empty());
#endif
}

} // namespace

std::uint32_t fpga_m7_ar_overlay(
    const fpga_m7_ar_word* depth, const fpga_m7_ar_word* grid,
    const fpga_m7_ar_word* boxes, fpga_m7_ar_word* fb,
    std::uint32_t width, std::uint32_t height,
    std::uint32_t depth_width, std::uint32_t depth_height,
    std::uint32_t depth_dx, std::uint32_t depth_dy,
    std::uint32_t frame_valid, std::uint32_t pose_valid,
    std::uint32_t age_ms, std::uint32_t age_valid, std::uint32_t max_age_ms,
    std::uint32_t replay, std::uint32_t object_evidence,
    std::uint32_t frame_id, std::uint32_t permit_present,
    std::uint32_t permit_i, std::uint32_t grid_valid, std::uint32_t slice_y,
    std::int32_t slice_lo_mm, std::int32_t slice_hi_mm,
    std::uint32_t cam_cell_x, std::uint32_t cam_cell_z,
    std::uint32_t mini_x, std::uint32_t mini_y) {
    // Status 1: dimensions. Nothing is read or written.
    if (width == 0 || height == 0 || width > FPGA_M7_AR_MAX_WIDTH ||
        height > FPGA_M7_AR_MAX_HEIGHT || (width & 3) != 0 ||
        depth_width == 0 || depth_height == 0 ||
        depth_width > FPGA_M7_AR_MAX_WIDTH || depth_height > FPGA_M7_AR_MAX_HEIGHT ||
        (depth_width & 7) != 0 || depth_width > width || depth_height > height ||
        depth_dx > width - depth_width || depth_dy > height - depth_height)
        return 1;

    const bool permit_on = permit_i != 0;  // sampled once per frame

    const bool ctrl_bad = frame_valid > 1 || pose_valid > 1 || age_valid > 1 ||
                          replay > 1 || permit_present > 1 || grid_valid > 1 ||
                          object_evidence > 3 || slice_y > 31;

    // Descriptor: always 49 beats.
    u32 desc[FPGA_M7_AR_DESC_U32];
    for (u32 i = 0; i < FPGA_M7_AR_DESC_WORDS; ++i) {
#pragma HLS PIPELINE II=1
        const fpga_m7_ar_word w = boxes[i];
        for (u32 j = 0; j < 4; ++j) {
#pragma HLS UNROLL
            desc[4 * i + j] = w.range(32 * j + 31, 32 * j);
        }
    }
    ap_uint<11> bx0[32], bx1[32], by0[32], by1[32], bly[32];
    u32 bcol[32];
    u8 blab[32][6];
#pragma HLS ARRAY_PARTITION variable=bx0 complete
#pragma HLS ARRAY_PARTITION variable=bx1 complete
#pragma HLS ARRAY_PARTITION variable=by0 complete
#pragma HLS ARRAY_PARTITION variable=by1 complete
#pragma HLS ARRAY_PARTITION variable=bly complete
#pragma HLS ARRAY_PARTITION variable=bcol complete
#pragma HLS ARRAY_PARTITION variable=blab complete dim=0
    const u32 count = desc[1];
    bool desc_bad = desc[0] != FPGA_M7_AR_MAGIC || count > FPGA_M7_AR_MAX_BOXES;
    for (u32 i = 0; i < 32; ++i) {
#pragma HLS UNROLL
        const u32 x0 = desc[2 + 6 * i], y0 = desc[3 + 6 * i];
        const u32 x1 = desc[4 + 6 * i], y1 = desc[5 + 6 * i];
        const u32 v = desc[6 + 6 * i], lab = desc[7 + 6 * i];
        if (i < count && (x0 > x1 || y0 > y1 || x1 >= depth_width ||
                          y1 >= depth_height || !(v == 0 || v == 2 || v == 3)))
            desc_bad = true;
        const u32 X0 = (x0 & 0x7FF) + depth_dx, Y0 = (y0 & 0x7FF) + depth_dy;
        bx0[i] = X0;
        bx1[i] = (x1 & 0x7FF) + depth_dx;
        by0[i] = Y0;
        by1[i] = (y1 & 0x7FF) + depth_dy;
        bly[i] = Y0 >= 52 ? Y0 - 12 : Y0 + 2;
        bcol[i] = v == 2 ? 0xFF00FFu : v == 3 ? 0x808080u : 0xFFFFFFu;
        for (u32 k = 0; k < 4; ++k) {
#pragma HLS UNROLL
            blab[i][k] = (lab >> (8 * k)) & 0xFF;
        }
        blab[i][4] = ' ';
        blab[i][5] = v == 2 ? 'F' : v == 3 ? 'O' : '?';
    }
    const std::uint32_t status = ctrl_bad ? 2 : desc_bad ? 3 : 0;

    // Slice: layer slice_y, 2 beats per z.
    u8 slice[32][32];
    const bool grid_on = grid_valid == 1 && slice_y <= 31;
    if (grid_on) {
        for (u32 z = 0; z < 32; ++z) {
            for (u32 b = 0; b < 2; ++b) {
#pragma HLS PIPELINE II=1
                const fpga_m7_ar_word w = grid[(z * 32 + slice_y) * 2 + b];
                for (u32 j = 0; j < 16; ++j) slice[z][16 * b + j] = w.range(8 * j + 7, 8 * j);
            }
        }
    }

    // Minimum pass over the whole depth rectangle (panel-covered rows too).
    const u32 depth_words = depth_width * depth_height / 8;
    u32 nearest = 0xFFFF;
    for (u32 i = 0; i < depth_words; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=38400 max=38400
        const fpga_m7_ar_word w = depth[i];
        u32 m = 0xFFFF;
        for (u32 j = 0; j < 8; ++j) {
            const u32 z = w.range(16 * j + 15, 16 * j);
            if (valid_mm(z) && z < m) m = z;
        }
        if (m < nearest) nearest = m;
    }
    const bool has_depth = nearest != 0xFFFF;

    // Mode: 0 fresh, 1 bad frame/control/descriptor, 2 stale, 3 pose bad,
    // 4 no depth, 5 no age. replay=1 skips only the two age checks.
    const u32 mode = (status != 0 || frame_valid == 0) ? 1 :
                     (replay == 0 && age_valid == 0) ? 5 :
                     (replay == 0 && age_ms > max_age_ms) ? 2 :
                     pose_valid == 0 ? 3 :
                     !has_depth ? 4 : 0;
    const bool fresh = mode == 0;

    // Frame id, decimal without padding.
    u32 fid[10];
    u32 fid_n = 0;
    {
        u32 v = frame_id;
        for (u32 k = 0; k < 10; ++k) {
            fid[k] = v % 10;
            if (v != 0 || k == 0) fid_n = k + 1;
            v /= 10;
        }
    }
    u8 ptext[2][53];
    for (u32 r = 0; r < 2; ++r)
        for (u32 c = 0; c < 53; ++c)
            ptext[r][c] = panel_char(r, c, mode, replay, nearest, age_valid,
                                     age_ms, permit_present, permit_on,
                                     fresh ? object_evidence : 0, fid, fid_n);
    u32 pbg[2][4];
    pbg[0][0] = fresh ? 0x203040 : 0x604010;
    pbg[0][1] = fresh ? 0x104020 : 0x701010;
    pbg[0][2] = age_valid == 0 ? 0x303030 : age_ms > max_age_ms ? 0x701010 : 0x104020;
    pbg[0][3] = permit_present == 0 ? 0x303030 : permit_on ? 0x104020 : 0x701010;
    const u32 ev = fresh ? object_evidence : 0;
    pbg[1][0] = 0x502050;
    pbg[1][1] = ev == 0 ? 0x303030 : ev == 1 ? 0x104020 : ev == 3 ? 0x203060 : 0x605010;
    pbg[1][2] = 0x203040;
    pbg[1][3] = mode == 0 ? 0x104020 : mode == 3 ? 0x701010 : 0x303030;

    // Slice label "Y-0750..-0700MM": camera-y layer bounds (y points down).
    u8 slabel[15];
    {
        u32 tmp[5];
        slabel[0] = 'Y';
        for (u32 k = 0; k < 5; ++k) slabel[1 + k] = signed4(slice_lo_mm, k, tmp);
        slabel[6] = '.';
        slabel[7] = '.';
        for (u32 k = 0; k < 5; ++k) slabel[8 + k] = signed4(slice_hi_mm, k, tmp);
        slabel[13] = 'M';
        slabel[14] = 'M';
    }

    Facts f;
    f.width = width;
    f.height = height;
    f.dx = depth_dx;
    f.dy = depth_dy;
    f.dx_end = depth_dx + depth_width;
    f.dy_end = depth_dy + depth_height;
    f.mode = mode;
    f.n_boxes = desc_bad ? 0 : count;
    f.grid_on = grid_on ? 1 : 0;
    f.mx = static_cast<int>(mini_x > 4095 ? 4095 : mini_x);
    f.my = static_cast<int>(mini_y > 4095 ? 4095 : mini_y);
    f.cam_x = cam_cell_x;
    f.cam_z = cam_cell_z;

    render_region(depth, fb, depth_words, width * height / 4, f, ptext, pbg,
                  slabel, slice, bx0, bx1, by0, by1, bly, bcol, blab);
    return status;
}
