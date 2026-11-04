// Fast CPU twin of hw/m7/ar_overlay.cpp (AR2NANO2, 2026-11-02). See ar_overlay_fast.hpp.
// Every per-frame fact below (status, mode, panel text and colours, box tables, slice, slice label)
// is computed with the reference's own formulas; only the pixel loop is reorganised. The 11-bit box
// registers of the reference (ap_uint<11>) are reproduced with & 0x7FF.
#include "ar_overlay_fast.hpp"
#include <cstring>
#ifdef FPGA_AR_FAST_THREADS
#include <thread>
#include <vector>
#endif

namespace {

typedef std::uint32_t u32;

static const u32 kLut[313] = {
#include "ar_overlay_lut.inc"
};

static inline bool valid_mm(u32 z) { return z >= 100 && z <= 9999; }

#define G(a,b,c,d,e) ((a) | ((b) << 3) | ((c) << 6) | ((d) << 9) | ((e) << 12))
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

// glyph bits g (as glyph()): pixel (row, col) of the 3x5 cell
static inline bool gbit(u32 g, u32 row, u32 col) { return ((g >> (row * 3)) >> (2 - col)) & 1; }

static u32 digit(u32 n) { return '0' + n % 10; }

#define TXT(lit) do { static const char t_[] = lit; \
    return idx < sizeof(t_) - 1 ? static_cast<u32>(t_[idx]) : ' '; } while (0)

// identical to the reference's panel_char
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
            const u32 cm = nearest / 10;
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
    if (seg == 0) TXT("RISK N/A");
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

static u32 signed4(int v, u32 k) {
    const bool big = v < -9999 || v > 9999;
    const u32 mag = big ? 0u : static_cast<u32>(v < 0 ? -v : v);
    switch (k) {
    case 0: return v < 0 ? '-' : '+';
    case 1: return big ? '?' : digit(mag / 1000);
    case 2: return big ? '?' : digit(mag / 100);
    case 3: return big ? '?' : digit(mag / 10);
    default: return big ? '?' : digit(mag);
    }
}

struct Frame {
    u32 W, H, dw, dh, dx, dy, dx_end, dy_end;
    bool fresh, grid_on;
    u32 n;                       // boxes drawn (0 unless mode 0 and descriptor good)
    u32 bx0[32], bx1[32], by0[32], by1[32], bly[32], bcol[32];
    u32 bglyph[32][6];           // glyph bits of the label characters
    int mx, my;
    u32 cam_x, cam_z;
    u32 pglyph[2][53];           // glyph bits of the panel characters
    u32 pbg[2][4];
    u32 sglyph[16];              // glyph bits of the slice label (15 chars + blank)
    unsigned char slice[32][32];
    const std::uint16_t* depth;
};

static void panel_row(const Frame& f, u32 y, u32* row) {
    const u32 r = y < 20 ? 0 : 1;
    const u32 gy0 = r == 0 ? 4 : 22;
    const bool gy_in = y >= gy0 && y < gy0 + 15;
    const u32 grow = gy_in ? (y - gy0) / 3 : 0;
    for (u32 x = 0; x < f.W; ++x) {
        u32 c;
        if (y == 39 || x == 0 || x == 156 || x == 312 || x == 468) {
            c = 0xFFFFFF;
        } else if (x >= 636) {
            c = 0x101010;
        } else {
            const u32 col = x / 12, sub = x % 12;
            const bool ink = sub >= 2 && sub < 11 && gy_in && gbit(f.pglyph[r][col], grow, (sub - 2) / 3);
            c = ink ? 0xFFFFFFu : f.pbg[r][x >= 468 ? 3 : x / 156];
        }
        row[x] = c;
    }
}

static void body_row(const Frame& f, u32 y, u32* row) {
    const u32 W = f.W;
    // background + depth rectangle
    const bool yd = y >= f.dy && y < f.dy_end;
    if (!yd) {
        for (u32 x = 0; x < W; ++x) row[x] = 0x101010;
    } else {
        for (u32 x = 0; x < f.dx; ++x) row[x] = 0x101010;
        for (u32 x = f.dx_end; x < W; ++x) row[x] = 0x101010;
        u32* o = row + f.dx;
        if (f.fresh) {
            const std::uint16_t* d = f.depth + static_cast<std::size_t>(y - f.dy) * f.dw;
            for (u32 i = 0; i < f.dw; ++i) {
                const u32 z = d[i];
                o[i] = valid_mm(z) ? kLut[z >> 5] : 0u;
            }
        } else {
            const u32 yb = y >> 3;
            for (u32 x = f.dx; x < f.dx_end; ++x)
                row[x] = (((x >> 3) + yb) & 1) ? 0xA000A0u : 0x180018u;
        }
    }
    // box lines, then labels; index 31..0 so that the lowest index is painted last (on top)
    for (int i = static_cast<int>(f.n) - 1; i >= 0; --i) {
        const u32 X0 = f.bx0[i], X1 = f.bx1[i], Y0 = f.by0[i], Y1 = f.by1[i];
        if (y < Y0 || y > Y1 || X0 > X1) continue;
        const u32 c = f.bcol[i];
        const u32 xe = X1 < W ? X1 : W - 1;     // X1 < W whenever boxes are drawn; clip anyway
        if (y <= Y0 + 1 || y + 1 >= Y1) {
            for (u32 x = X0; x <= xe; ++x) row[x] = c;
        } else {
            // x <= X0 + 1  or  x + 1 >= X1, within [X0, X1]: the two left and two right columns
            const u32 l1 = X0 + 1 < xe ? X0 + 1 : xe;
            for (u32 x = X0; x <= l1; ++x) row[x] = c;
            const u32 r0 = X1 >= 1 && X1 - 1 > X0 ? X1 - 1 : X0;
            for (u32 x = r0; x <= xe; ++x) row[x] = c;
        }
    }
    for (int i = static_cast<int>(f.n) - 1; i >= 0; --i) {
        const u32 LY = f.bly[i];
        if (y < LY || y >= LY + 12) continue;
        const u32 X0 = f.bx0[i];
        const u32 ry = y - LY;
        const bool rowink = ry >= 1 && ry <= 10;
        const u32 grow = rowink ? (ry - 1) >> 1 : 0;
        const u32 c = f.bcol[i];
        const u32 xe = X0 + 48 < W ? X0 + 48 : W;
        for (u32 x = X0; x < xe; ++x) {
            const u32 rx = x - X0, cx = rx & 7;
            const bool ink = cx >= 1 && cx <= 6 && rowink && gbit(f.bglyph[i][rx >> 3], grow, (cx - 1) >> 1);
            row[x] = ink ? 0u : c;
        }
    }
    // mini map
    const int ry = static_cast<int>(y) - f.my;
    if (ry >= -12 && ry <= 128) {
        const int iW = static_cast<int>(W);
        if (ry >= 0 && ry < 128) {
            int xs = f.mx, xe = f.mx + 128;          // inner [xs, xe)
            if (xs < 0) xs = 0;
            if (xe > iW) xe = iW;
            if (!f.fresh || !f.grid_on) {
                for (int x = xs; x < xe; ++x)
                    row[x] = ((static_cast<u32>(x) + y) & 7) < 2 ? 0xA000A0u : 0x180018u;
            } else {
                const u32 gz = 31 - (static_cast<u32>(ry) >> 2);
                for (int x = xs; x < xe; ++x) {
                    const u32 gx = static_cast<u32>(x - f.mx) >> 2;
                    const u32 v = f.slice[gz][gx];
                    u32 c = v == 0 ? 0x282828u : v == 1 ? 0x2E6B3Au : v == 2 ? 0xFF3030u : 0xFF00FFu;
                    if (gx == f.cam_x && gz == f.cam_z) c = 0xFFFFFF;
                    row[x] = c;
                }
            }
            if (f.mx - 1 >= 0 && f.mx - 1 < iW) row[f.mx - 1] = 0xFFFFFF;
            if (f.mx + 128 >= 0 && f.mx + 128 < iW) row[f.mx + 128] = 0xFFFFFF;
        } else if (ry == -1 || ry == 128) {
            int xs = f.mx - 1, xe = f.mx + 129;
            if (xs < 0) xs = 0;
            if (xe > iW) xe = iW;
            for (int x = xs; x < xe; ++x) row[x] = 0xFFFFFF;
        } else {                                  // ry in [-12, -2]: label strip
            const u32 ly = static_cast<u32>(ry + 12);
            const bool rowink = ly >= 1 && ly <= 10;
            const u32 grow = rowink ? (ly - 1) >> 1 : 0;
            int xs = f.mx, xe = f.mx + 128;
            if (xs < 0) xs = 0;
            if (xe > iW) xe = iW;
            for (int x = xs; x < xe; ++x) {
                const u32 lx = static_cast<u32>(x - f.mx);
                const u32 ch = lx >> 3, cx = lx & 7;
                const bool ink = cx >= 1 && cx <= 6 && rowink && gbit(f.sglyph[ch < 15 ? ch : 15], grow, (cx - 1) >> 1);
                row[x] = ink ? 0xFFFFFFu : 0x202020u;
            }
        }
    }
}

static void rows(const Frame& f, std::uint32_t* fb, u32 y0, u32 y1) {
    for (u32 y = y0; y < y1; ++y) {
        u32* row = fb + static_cast<std::size_t>(y) * f.W;
        if (y < 40) panel_row(f, y, row);
        else body_row(f, y, row);
    }
}

} // namespace

std::uint32_t fpga_m7_ar_overlay_fast(const std::uint16_t* depth, const std::uint8_t* grid,
                                     const std::uint32_t* desc, std::uint32_t* fb,
                                     const std::uint32_t regs[24], int threads) {
    const u32 width = regs[0], height = regs[1], depth_width = regs[2], depth_height = regs[3];
    const u32 depth_dx = regs[4], depth_dy = regs[5], frame_valid = regs[6], pose_valid = regs[7];
    const u32 age_ms = regs[8], age_valid = regs[9], max_age_ms = regs[10], replay = regs[11];
    const u32 object_evidence = regs[12], frame_id = regs[13], permit_present = regs[14];
    const u32 permit_i = regs[15], grid_valid = regs[16], slice_y = regs[17];
    const int slice_lo_mm = static_cast<std::int32_t>(regs[18]), slice_hi_mm = static_cast<std::int32_t>(regs[19]);
    const u32 cam_cell_x = regs[20], cam_cell_z = regs[21], mini_x = regs[22], mini_y = regs[23];

    if (width == 0 || height == 0 || width > 1280 || height > 800 || (width & 3) != 0 ||
        depth_width == 0 || depth_height == 0 || depth_width > 1280 || depth_height > 800 ||
        (depth_width & 7) != 0 || depth_width > width || depth_height > height ||
        depth_dx > width - depth_width || depth_dy > height - depth_height)
        return 1;

    const bool permit_on = permit_i != 0;
    const bool ctrl_bad = frame_valid > 1 || pose_valid > 1 || age_valid > 1 ||
                          replay > 1 || permit_present > 1 || grid_valid > 1 ||
                          object_evidence > 3 || slice_y > 31;

    static Frame f;   // large; one renderer per process (the presenter is single-threaded)
    const u32 count = desc[1];
    bool desc_bad = desc[0] != 0x4D374152u || count > 32;
    for (u32 i = 0; i < 32; ++i) {
        const u32 x0 = desc[2 + 6 * i], y0 = desc[3 + 6 * i];
        const u32 x1 = desc[4 + 6 * i], y1 = desc[5 + 6 * i];
        const u32 v = desc[6 + 6 * i], lab = desc[7 + 6 * i];
        if (i < count && (x0 > x1 || y0 > y1 || x1 >= depth_width ||
                          y1 >= depth_height || !(v == 0 || v == 2 || v == 3)))
            desc_bad = true;
        const u32 X0 = (x0 & 0x7FF) + depth_dx, Y0 = (y0 & 0x7FF) + depth_dy;
        f.bx0[i] = X0 & 0x7FF;
        f.bx1[i] = ((x1 & 0x7FF) + depth_dx) & 0x7FF;
        f.by0[i] = Y0 & 0x7FF;
        f.by1[i] = ((y1 & 0x7FF) + depth_dy) & 0x7FF;
        f.bly[i] = (Y0 >= 52 ? Y0 - 12 : Y0 + 2) & 0x7FF;
        f.bcol[i] = v == 2 ? 0xFF00FFu : v == 3 ? 0x808080u : 0xFFFFFFu;
        for (u32 k = 0; k < 4; ++k) f.bglyph[i][k] = glyph((lab >> (8 * k)) & 0xFF);
        f.bglyph[i][4] = glyph(' ');
        f.bglyph[i][5] = glyph(v == 2 ? 'F' : v == 3 ? 'O' : '?');
    }
    const std::uint32_t status = ctrl_bad ? 2 : desc_bad ? 3 : 0;

    const bool grid_on = grid_valid == 1 && slice_y <= 31;
    if (grid_on)
        for (u32 z = 0; z < 32; ++z)
            std::memcpy(f.slice[z], grid + (z * 32 + slice_y) * 32, 32);

    const std::size_t npx = static_cast<std::size_t>(depth_width) * depth_height;
    u32 nearest = 0xFFFF;
    for (std::size_t i = 0; i < npx; ++i) {
        const u32 z = depth[i];
        if (z >= 100 && z <= 9999 && z < nearest) nearest = z;
    }
    const bool has_depth = nearest != 0xFFFF;

    const u32 mode = (status != 0 || frame_valid == 0) ? 1 :
                     (replay == 0 && age_valid == 0) ? 5 :
                     (replay == 0 && age_ms > max_age_ms) ? 2 :
                     pose_valid == 0 ? 3 :
                     !has_depth ? 4 : 0;
    const bool fresh = mode == 0;

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
    for (u32 r = 0; r < 2; ++r)
        for (u32 c = 0; c < 53; ++c)
            f.pglyph[r][c] = glyph(panel_char(r, c, mode, replay, nearest, age_valid, age_ms, permit_present,
                                              permit_on, fresh ? object_evidence : 0, fid, fid_n) & 0xFF);
    f.pbg[0][0] = fresh ? 0x203040 : 0x604010;
    f.pbg[0][1] = fresh ? 0x104020 : 0x701010;
    f.pbg[0][2] = age_valid == 0 ? 0x303030 : age_ms > max_age_ms ? 0x701010 : 0x104020;
    f.pbg[0][3] = permit_present == 0 ? 0x303030 : permit_on ? 0x104020 : 0x701010;
    const u32 ev = fresh ? object_evidence : 0;
    f.pbg[1][0] = 0x502050;
    f.pbg[1][1] = ev == 0 ? 0x303030 : ev == 1 ? 0x104020 : ev == 3 ? 0x203060 : 0x605010;
    f.pbg[1][2] = 0x203040;
    f.pbg[1][3] = mode == 0 ? 0x104020 : mode == 3 ? 0x701010 : 0x303030;

    {
        u32 sl[15];
        sl[0] = 'Y';
        for (u32 k = 0; k < 5; ++k) sl[1 + k] = signed4(slice_lo_mm, k);
        sl[6] = '.';
        sl[7] = '.';
        for (u32 k = 0; k < 5; ++k) sl[8 + k] = signed4(slice_hi_mm, k);
        sl[13] = 'M';
        sl[14] = 'M';
        for (u32 k = 0; k < 15; ++k) f.sglyph[k] = glyph(sl[k]);
        f.sglyph[15] = glyph(' ');
    }

    f.W = width;
    f.H = height;
    f.dw = depth_width;
    f.dh = depth_height;
    f.dx = depth_dx;
    f.dy = depth_dy;
    f.dx_end = depth_dx + depth_width;
    f.dy_end = depth_dy + depth_height;
    f.fresh = fresh;
    f.grid_on = grid_on;
    f.n = (fresh && !desc_bad) ? count : 0;
    f.mx = static_cast<int>(mini_x > 4095 ? 4095 : mini_x);
    f.my = static_cast<int>(mini_y > 4095 ? 4095 : mini_y);
    f.cam_x = cam_cell_x;
    f.cam_z = cam_cell_z;
    f.depth = depth;

#ifdef FPGA_AR_FAST_THREADS
    if (threads > 1) {
        if (threads > 8) threads = 8;
        std::vector<std::thread> ts;
        const u32 band = (height + threads - 1) / threads;
        for (int t = 1; t < threads; ++t) {
            const u32 a = band * t, b = a + band < height ? a + band : height;
            if (a < b) ts.emplace_back(rows, std::cref(f), fb, a, b);
        }
        rows(f, fb, 0, band < height ? band : height);
        for (auto& t : ts) t.join();
        return status;
    }
#else
    (void)threads;
#endif
    rows(f, fb, 0, height);
    return status;
}
