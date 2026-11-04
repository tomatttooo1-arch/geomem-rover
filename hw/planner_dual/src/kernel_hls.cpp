#include "kernel_hls.hpp"
#include <hls_stream.h>

namespace {
typedef ap_uint<29> Potential;
typedef ap_uint<38> Cell;
typedef ap_uint<30> RowCell;
typedef ap_uint<16> Index;
typedef ap_uint<8> Coord;
typedef ap_uint<9> Round;

static Index cell_index(Coord x, Coord y, bool global) {
#pragma HLS INLINE
  const Index yy = y;
  const Index base = global
      ? Index((yy << 7) + (yy << 6) + (yy << 3))
      : Index((yy << 6) + (yy << 4));
  return Index(base + Index(x));
}

static ap_uint<8> encode_entry(ap_uint<8> raw) {
#pragma HLS INLINE
  const ap_uint<10> x = ap_uint<10>(raw) << 2;
  const ap_uint<18> wide = x;
  const ap_uint<18> product =
      (wide << 7) + (wide << 6) + (wide << 3) + (wide << 2) + wide;
  return ap_uint<8>(50) + ap_uint<8>(product >> 10);
}

static Potential add_sat(Potential p, ap_uint<12> weight, Potential inf) {
#pragma HLS INLINE
  const ap_uint<30> sum = ap_uint<30>(p) + ap_uint<30>(weight);
  return sum >= ap_uint<30>(inf) ? inf : Potential(sum);
}

static RowCell row_cell(Potential p, bool walk) {
#pragma HLS INLINE
  RowCell packed = p;
  packed[29] = walk;
  return packed;
}

static void load_input(const ap_uint<8> map[kGmapAreaBytes],
                       hls::stream<ap_uint<8> >& pixels, unsigned grid_mode,
                       unsigned path_ctrl) {
#pragma HLS INLINE off
  const bool path_en = (path_ctrl & 1u) != 0;
  // PLNPATH0: the parameter block sits right after the live map bytes, so the same single
  // burst loop simply runs 132 bytes longer (no second access site, no new address logic).
  const Index count = grid_mode == 0 ? Index(path_en ? 6400 + kPlnParamBytes : 6400)
      : grid_mode == 1 ? Index(path_en ? 40000 + kPlnParamBytes : 40000) : Index(0);
  load_loop: for (Index i = 0; i < count; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40132 avg=40000
    pixels.write(map[i]);
  }
}

// Sweep coordinates are oriented along the active pass. W and N always refer
// to already-updated neighbors. The two-register lookahead consumes one fused
// Cell read per column and retains the old potential until this cell's write.
static bool scan_pass(Cell cells[40000], Index source, Coord width,
                      Index count, Potential inf, bool reverse) {
#pragma HLS INLINE off
  RowCell previous[200];
#pragma HLS BIND_STORAGE variable=previous type=ram_s2p impl=bram latency=1
  reset_row: for (Coord x = 0; x < width; ++x) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=80 max=200 avg=200
    previous[x] = row_cell(inf, false);
  }
  bool changed = false;
  const Coord last = width - 1;
  const Coord lookahead_end = width - 2;
  const Index first = reverse ? Index(count - 1) : Index(0);
  Potential nw = inf, n = inf, ne = inf, west = inf;
  bool nw_walk = false, n_walk = false, ne_walk = false, west_walk = false;
  Cell current = cells[first];
  Cell east = cells[reverse ? Index(first - 1) : Index(first + 1)];
  Coord col = 0;
  // One uninterrupted pass: the lookahead reads straddle row boundaries.
  column_loop: for (Index step = 0; step < count; ++step) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40000 avg=40000
    const Index i = reverse ? Index(count - 1 - step) : step;
    // At the last two columns, previous[0]/[1] already contain this row's
    // newly written values. Prefetch them as the next row's first N/NE pair.
    const Coord future_col = col < lookahead_end
        ? Coord(col + 2) : Coord(col - lookahead_end);
    const RowCell future_north = previous[future_col];
    Cell future_cell = 0;
    if (step < Index(count - 2))
      future_cell = cells[reverse ? Index(i - 2) : Index(i + 2)];
    const bool walk = current[37];
    const bool east_walk = east[37];
    const bool west_valid = col != 0 && west_walk;
    const bool northwest_valid = col != 0 && nw_walk;
    const bool northeast_valid = col != last && ne_walk;
    const ap_uint<8> entry = current.range(36, 29);
    const ap_uint<12> wide_entry = entry;
    const ap_uint<12> straight = (wide_entry << 3) + (wide_entry << 1);
    const ap_uint<12> diagonal = (wide_entry << 4) - (wide_entry << 1);
    const Potential old = current.range(28, 0);
    const Potential a = (northwest_valid && west_valid && n_walk)
        ? add_sat(nw, diagonal, inf) : inf;
    const Potential b = n_walk ? add_sat(n, straight, inf) : inf;
    const Potential c = (northeast_valid && east_walk && n_walk)
        ? add_sat(ne, diagonal, inf) : inf;
    // Balance the four non-feedback candidates into two compare/select
    // levels before the final W-feedback comparison.
    const Potential upper_left = a < old ? a : old;
    const Potential upper_right = c < b ? c : b;
    const Potential upper = upper_right < upper_left ? upper_right : upper_left;
    // The only numeric feedback is the immediately preceding column's best.
    const Potential w = west_valid ? add_sat(west, straight, inf) : inf;
    Potential best = w < upper ? w : upper;
    if (!walk || i == source) best = old;
    changed = changed || best < old;
    Cell updated = current;
    updated.range(28, 0) = best;
    cells[i] = updated;  // Full 38-bit write; no RAM bitfield read-modify-write.
    previous[col] = row_cell(best, walk);
    west = best;
    west_walk = walk;
    nw = n; n = ne; ne = future_north.range(28, 0);
    nw_walk = n_walk; n_walk = ne_walk; ne_walk = future_north[29];
    current = east;
    east = future_cell;
    col = col == last ? Coord(0) : Coord(col + 1);
  }
  return changed;
}

// ---------------------------------------------------------------- PLNPATH0 path extraction
// Runs on the converged field only (status 0). Semantics = PS extract() (planner_split.hpp):
// goal pick (exact goal if reachable, else first reachable entry of the PS-sorted candidate
// list), then steepest descent to the source. Per step the 8 neighbours are read once each
// (orthogonals first so the diagonal no-corner-cut test can use their walk bits); among the
// accepted ones (walk, corner ok, p<INF, p+entry*w <= p_cur) the smallest key
// 8*(squared distance to the source) + original enumeration index k wins. On a converged field
// every accepted candidate equals p_cur (DESIGN.md 2.2-3), so this key is the PS tie order.
static Index mul_index(Coord x, Coord y, Coord width) {
#pragma HLS INLINE
  const Index row = Index(y) * Index(width);
#pragma HLS BIND_OP variable=row op=mul impl=dsp
  return Index(row + Index(x));
}

static void extract_path(const Cell cells[40000], const ap_uint<8> param_ram[kPlnParamBytes],
                         ap_uint<16> path_ram[kPlnPathMax], Coord sx, Coord sy,
                         Coord gx, Coord gy, Coord width, Potential inf,
                         ap_uint<4>& pstatus, ap_uint<8>& chosen, ap_uint<13>& n) {
#pragma HLS INLINE off
  const ap_uint<8> nc_raw = param_ram[0];
  const ap_uint<7> nc = nc_raw > 64 ? ap_uint<7>(64) : ap_uint<7>(nc_raw);
  const bool exact_safe = param_ram[1][0];
  // Goal pick as one short sequential probe: j = 0 is the exact goal cell, j >= 1 is candidate j-1.
  bool have = false;
  ap_uint<4> st = 4;
  ap_uint<8> ch = 255;
  Coord x = gx, y = gy;
  probe: for (ap_uint<7> j = 0; j <= nc; ++j) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT min=1 max=65 avg=1
    const ap_uint<8> base = ap_uint<8>(j) << 1;
    const Coord a = j == 0 ? gx : Coord(param_ram[base + 2]);
    const Coord b = j == 0 ? gy : Coord(param_ram[base + 3]);
    const bool in = a < width && b < width;
    const Cell c = cells[mul_index(in ? a : Coord(0), in ? b : Coord(0), width)];
    if (in && Potential(c.range(28, 0)) != inf) {
      if (j == 0 && !exact_safe) { st = 5; break; }
      have = true; st = j == 0 ? 0 : 1; ch = j == 0 ? ap_uint<8>(255) : ap_uint<8>(j - 1);
      x = a; y = b;
      break;
    }
  }
  ap_uint<13> len = 0;
  if (have) {
    descend: while (true) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=4096 avg=200
      ap_uint<16> pt = 0;
      pt.range(7, 0) = x; pt.range(15, 8) = y;
      path_ram[ap_uint<12>(len)] = pt;
      ++len;
      if (x == sx && y == sy) break;
      if (len == ap_uint<13>(kPlnPathMax)) { st = 9; break; }
      // GMAP2 (lowering 1+2): one cells read site per step. Beat j = 0 reads the current cell
      // (p_cur, entry) and beats j = 1..8 read the 8 neighbours; the per-beat offset, enumeration
      // index k and the orthogonal/diagonal kind come from a 9-entry constant ROM (no switch muxes).
      // ROM word: [2:0] k, [4:3] dx (2-bit signed), [6:5] dy (2-bit signed).
      static const ap_uint<7> kNbr[9] = {
          0x00,                                   // j0: self
          (3u << 5) | (0u << 3) | 1u,             // j1: k1 ( 0,-1) N
          (0u << 5) | (3u << 3) | 3u,             // j2: k3 (-1, 0) W
          (0u << 5) | (1u << 3) | 4u,             // j3: k4 (+1, 0) E
          (1u << 5) | (0u << 3) | 6u,             // j4: k6 ( 0,+1) S
          (3u << 5) | (3u << 3) | 0u,             // j5: k0 (-1,-1)
          (3u << 5) | (1u << 3) | 2u,             // j6: k2 (+1,-1)
          (1u << 5) | (3u << 3) | 5u,             // j7: k5 (-1,+1)
          (1u << 5) | (1u << 3) | 7u};            // j8: k7 (+1,+1)
      Potential p_cur = 0;
      ap_uint<12> c10 = 0, c14 = 0;
      bool found = false;
      ap_uint<20> best_key = 0;
      Coord wx = 0, wy = 0;
      bool walk_n = false, walk_w = false, walk_e = false, walk_s = false;
      neighbors: for (ap_uint<4> j = 0; j < 9; ++j) {
#pragma HLS PIPELINE II=1
        const ap_uint<7> t = kNbr[j];
        const ap_uint<3> k = t.range(2, 0);
        const ap_uint<2> dxr = t.range(4, 3), dyr = t.range(6, 5);
        const Coord xx = Coord(x + Coord(dxr[1] ? Coord(0xFC | dxr) : Coord(dxr)));
        const Coord yy = Coord(y + Coord(dyr[1] ? Coord(0xFC | dyr) : Coord(dyr)));
        const bool self = j == 0;
        const bool diag = j > 4;
        const Cell c = cells[mul_index(xx, yy, width)];
        const bool walk = c[37];
        const Potential p = c.range(28, 0);
        if (self) {
          p_cur = p;
          const ap_uint<12> wide_e = c.range(36, 29);
          c10 = (wide_e << 3) + (wide_e << 1);
          c14 = (wide_e << 4) - (wide_e << 1);
        }
        if (j == 1) walk_n = walk;
        if (j == 2) walk_w = walk;
        if (j == 3) walk_e = walk;
        if (j == 4) walk_s = walk;
        const bool corner_ok = !diag ||
            ((dxr[1] ? walk_w : walk_e) && (dyr[1] ? walk_n : walk_s));
        const ap_uint<30> cand = ap_uint<30>(p) + ap_uint<30>(diag ? c14 : c10);
        const ap_int<9> ddx = ap_int<9>(ap_uint<9>(xx)) - ap_int<9>(ap_uint<9>(sx));
        const ap_int<9> ddy = ap_int<9>(ap_uint<9>(yy)) - ap_int<9>(ap_uint<9>(sy));
        const ap_uint<17> d2x = ddx * ddx;
#pragma HLS BIND_OP variable=d2x op=mul impl=dsp
        const ap_uint<17> d2y = ddy * ddy;
#pragma HLS BIND_OP variable=d2y op=mul impl=dsp
        const ap_uint<17> d2 = d2x + d2y;
        ap_uint<20> key = 0;
        key.range(19, 3) = d2;
        key.range(2, 0) = k;
        // GMAP2 (lowering 4): on the converged field every accepted candidate has cand == p_cur
        // (PLNPATH0 DESIGN 2.2-3, C model assertion 0 violations), so an equality compare suffices.
        const bool accept = !self && walk && corner_ok && p != inf && cand == ap_uint<30>(p_cur);
        if (accept && (!found || key < best_key)) {
          found = true; best_key = key; wx = xx; wy = yy;
        }
      }
      if (!found) { st = 4; break; }
      x = wx; y = wy;
    }
  }
  if (st != 0 && st != 1) { len = 0; ch = 255; }
  pstatus = st; chosen = ch; n = len;
}

static void compute(hls::stream<ap_uint<8> >& pixels,
                    hls::stream<ap_uint<32> >& output,
                    int sx, int sy, int gx, int gy, unsigned k_max,
                    unsigned grid_mode, unsigned path_ctrl) {
#pragma HLS INLINE off
  const bool path_en = (path_ctrl & 1u) != 0;
  ap_uint<8> param_ram[kPlnParamBytes];
#pragma HLS BIND_STORAGE variable=param_ram type=ram_s2p impl=bram
  ap_uint<16> path_ram[kPlnPathMax];
#pragma HLS BIND_STORAGE variable=path_ram type=ram_s2p impl=bram
  Cell cells[40000];
#pragma HLS BIND_STORAGE variable=cells type=ram_s2p impl=bram latency=1
  ap_uint<8> flags[256];
#pragma HLS BIND_STORAGE variable=flags type=ram_1p impl=lutram
  const bool supported = grid_mode <= 1;
  const bool global = grid_mode == 1;
  const Coord width = global ? Coord(200) : Coord(80);
  const Index count = supported ? (global ? Index(40000) : Index(6400)) : Index(0);
  const Round flag_count = supported ? (global ? Round(256) : Round(64)) : Round(0);
  const Potential inf = global ? Potential(0x1fffffffu) : Potential(0x00ffffffu);
  const bool args = supported && k_max > 0 && k_max <= unsigned(flag_count)
      && sx >= 0 && sy >= 0 && gx >= 0 && gy >= 0
      && sx < int(width) && sy < int(width)
      && gx < int(width) && gy < int(width);
  const Index source = args ? cell_index(Coord(sx), Coord(sy), global) : Index(0);
  const Index goal = args ? cell_index(Coord(gx), Coord(gy), global) : Index(0);
  bool raw_goal_walk = false;
  Coord x = 0, y = 0;
  const Coord last = width - 1;
  initialize: for (Index i = 0; i < count; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40000 avg=40000
    const ap_uint<8> raw = pixels.read();
    const bool raw_walk = raw < 253;
    if (args && i == goal) raw_goal_walk = raw_walk;
    const bool border = x == 0 || y == 0 || x == last || y == last;
    const bool is_source = args && i == source;
    const bool walk = !border && (is_source || raw_walk);
    const ap_uint<8> entry = is_source ? ap_uint<8>(50) : encode_entry(raw);
    Cell packed = is_source ? Potential(0) : inf;
    packed.range(36, 29) = entry;
    packed[37] = walk;
    cells[i] = packed;
    if (x == last) { x = 0; ++y; } else { ++x; }
  }
  // PLNPATH0 parameter block, stored as bytes: [0] nc, [1] bit0 exact-goal safe, [2..3] 0,
  // [4+2j] = x_j, [5+2j] = y_j. Decoded only inside extract_path().
  if (supported && path_en) {
    param_loop: for (ap_uint<8> j = 0; j < ap_uint<8>(kPlnParamBytes); ++j) {
#pragma HLS PIPELINE II=1
      param_ram[j] = pixels.read();
    }
  }
  clear_flags: for (Round r = 0; r < flag_count; ++r) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=64 max=256 avg=256
    flags[r] = 255;
  }
  ap_uint<32> status = supported ? 6 : 8;
  Round rounds = 0;
  bool valid = false;
  if (args) {
    if (sx == 0 || sy == 0 || sx == int(last) || sy == int(last)) status = 2;
    else if (!raw_goal_walk) status = 3;
    else {
      status = 7;
      round_loop: for (Round r = 0; r < Round(k_max); ++r) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=256 avg=6
        bool changed = false;
        phase_loop: for (ap_uint<2> phase = 0; phase < 2; ++phase) {
          const bool pass_changed = scan_pass(cells, source, width, count, inf, phase != 0);
          changed = changed || pass_changed;
        }
        flags[r] = changed ? 1 : 0;
        rounds = r + 1;
        if (!changed) { status = 0; valid = true; break; }
      }
    }
  }
  ap_uint<4> pstatus = 15;
  ap_uint<8> chosen = 255;
  ap_uint<13> path_n = 0;
  if (valid && path_en)
    extract_path(cells, param_ram, path_ram, Coord(sx), Coord(sy), Coord(gx), Coord(gy),
                 width, inf, pstatus, chosen, path_n);
  export_field: for (Index i = 0; i < count; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40000 avg=40000
    const Cell packed = cells[i];
    const Potential phi = packed.range(28, 0);
    output.write(valid ? ap_uint<32>(phi) : ap_uint<32>(inf));
  }
  // flags block length token: F flags, then (path_en) 1 header + n path words.
  const ap_uint<13> block = ap_uint<13>(flag_count)
      + ((supported && path_en) ? ap_uint<13>(path_n + 1) : ap_uint<13>(0));
  output.write(ap_uint<32>(block));
  ap_uint<32> header = 0;
  header.range(15, 0) = path_n;
  header.range(23, 16) = chosen;
  header.range(27, 24) = pstatus;
  header.range(31, 28) = 0xA;
  ap_uint<13> back = path_n;
  export_flags: for (ap_uint<13> r = 0; r < block; ++r) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=64 max=4353 avg=457
    ap_uint<32> word;
    if (r < ap_uint<13>(flag_count)) {
      word = flags[ap_uint<8>(r)];
    } else if (r == ap_uint<13>(flag_count)) {
      word = header;
    } else {
      --back;  // r = F+1 -> point n-1 (descent stored target first): start -> target order
      word = path_ram[ap_uint<12>(back)];
    }
    output.write(word);
  }
  output.write(status);
  output.write(ap_uint<32>(rounds));
  output.write(ap_uint<32>(valid));
}

static void store_output(hls::stream<ap_uint<32> >& input,
                         ap_uint<32> field[kGmapAreaWords], ap_uint<32> flags[kPlnFlagsDepth],
                         ap_uint<32>& status, ap_uint<32>& rounds,
                         ap_uint<32>& field_valid, unsigned grid_mode) {
#pragma HLS INLINE off
  const bool supported = grid_mode <= 1;
  const Index count = supported ? (grid_mode == 1 ? Index(40000) : Index(6400)) : Index(0);
  store_field: for (Index i = 0; i < count; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40000 avg=40000
    field[i] = input.read();
  }
  // PLNPATH0: one access site, variable length (F flags, or F + 1 + n with the path).
  const ap_uint<13> block = ap_uint<13>(input.read());
  store_flags: for (ap_uint<13> r = 0; r < block; ++r) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=64 max=4353 avg=457
    flags[r] = input.read();
  }
  status = input.read();
  rounds = input.read();
  field_valid = input.read();
}
// ---------------------------------------------------------------- GMAP global map (gmap_op 1-4)
// Design: runs/nationals-prelim-20261102/GMAP0/DESIGN.md sections 3.3, 4, 5 and GMAP0/DESIGN_ADD_PLGRID.md (WINDOW,
// positions Q16 since GMAP3). Reference: tools/gmap/gmap_model.c (byte reads on the 8-bit map port, whole 32-bit
// words on the field port, same regions, same word contents).
// One row engine serves the four operations:
//   MERGE  steps over rows iy-1 .. iy+80 (frame rows read once; region rows [R0, R1) merged), columns [C0, C1)
//          8-aligned; global words and coarse words of the region written back whole. dropped = known frame cells
//          read - known frame cells merged.
//   SCAN   rows 0..399, columns 0..399, no frame; image words (flip optional), coarse rebuilt, bbox of seen cells.
//   CLEAR  as SCAN but every cell is taken as the sentinel without reading; global + coarse words written, and with
//          param byte 13 bit 0 the T row (400 words "never") too.
//   WINDOW prologue: 48 header bytes + 21 points (216 bytes) -> segment 1 (16 x 16 box per point -> mask);
//          rows ly = 0..79: aux row + in-map T cells read, per-cell seed / swept clearing, T row and frame row written.
// Exactly ONE map read site and ONE field write site (GMAP2 attempt 2): every step runs up to two read segments
// through the same loop, computes from on-chip buffers only, then up to three write segments through the same loop.
// Addresses are running pointers (no row multiplies); the result words go out through a shift register.
typedef ap_int<18> GInt;
typedef ap_uint<9> GCol;
typedef ap_uint<21> GAddr;      // byte address in the area (< 2^21), modular
typedef ap_uint<19> GWord;      // word address
typedef ap_uint<49> WD2;        // Q16 squared distance sum

static GInt gclamp(GInt v) {
#pragma HLS INLINE
  return v < 0 ? GInt(0) : v > GInt(400) ? GInt(400) : v;
}

static ap_uint<8> sel3(ap_uint<8> a, ap_uint<8> b, ap_uint<8> c, ap_uint<2> k) {
#pragma HLS INLINE
  return k[1] ? c : k[0] ? b : a;
}

// closest-point distance along one axis from cell k to P (Q16), signed (only its square is used).
// Exact for |P| < 2^23, -128 <= k < 128 (the table fill keeps k in range).
typedef ap_int<26> WD;
static WD axis_d(ap_int<8> k, ap_int<24> P) {
#pragma HLS INLINE
  const WD a = (WD(k) << 16) - WD(P);
  const WD b = a + WD(65536);
  return a > 0 ? a : b < 0 ? b : WD(0);
}

static void gmap_core(const ap_uint<8> map[kGmapAreaBytes], ap_uint<32> field[kGmapAreaWords],
                      ap_uint<3> op, ap_uint<32>& st_out) {
#pragma HLS INLINE off
  static ap_uint<1> mask[6400];   // swept-this-frame mask: set by segment 1, read and cleared by segment 2
#pragma HLS BIND_STORAGE variable=mask type=ram_t2p impl=bram
  ap_uint<8> frow[80];
#pragma HLS BIND_STORAGE variable=frow type=ram_s2p impl=lutram
  ap_uint<32> rbuf[400];
#pragma HLS BIND_STORAGE variable=rbuf type=ram_s2p impl=bram
  ap_uint<2> lb[400];
#pragma HLS BIND_STORAGE variable=lb type=ram_s2p impl=lutram
  ap_uint<32> wrow[200];
#pragma HLS BIND_STORAGE variable=wrow type=ram_s2p impl=bram
  ap_uint<32> wco[64];
#pragma HLS BIND_STORAGE variable=wco type=ram_s2p impl=lutram
  // WINDOW per-axis distance tables (one fill unit): dtab [0, 80) dx to the body, [80, 160) dy to the body,
  // [160, 176) dx and [176, 192) dy of the current capsule point's 16 x 16 box (one BRAM18, simple dual port)
  WD dtab[192];
#pragma HLS BIND_STORAGE variable=dtab type=ram_s2p impl=bram
  const bool op_ok = op >= 1 && op <= 4;
  const bool merge = op == 1, scan = op == 2, clear = op == 3, win = op == 4;
  const bool asm4 = win;                      // T cells are 4 bytes (rows); global cells 2 bytes
  // parameter fields (read from rbuf after the prologue read) and the state set by the prologue
  ap_uint<8> ct0 = 0, ct1 = 0, ct2 = 0, it0 = 0, it1 = 0, it2 = 0;
  ap_uint<48> rs2 = 0, re2 = 0;
  ap_int<32> now = 0, thr = 0;
  ap_int<16> ix = 0;
  GInt R0 = 0, R1 = 0, rbase = 0;
  GCol C0 = 0, ncell = 0;
  ap_uint<7> toff = 0, tend = 0;
  ap_uint<9> nrows = 0;
  bool run = false, flip = false, clr_t = false, seed_on = false, swept_on = false;
  // row addresses = base + row x stride, the products in DSPs (no running-pointer registers / update muxes)
  GAddr fbase = 0, cbase = 0;
  ap_uint<12> cstride = 0;
  GWord w1base = 0;
  ap_uint<6> w1mul = 0;
  ap_uint<13> known = 0, merged = 0, ncl = 0, nev = 0;
  GCol rmin = 399, rmax = 0, cmin = 399, cmax = 0;
  bool any = false;
  ap_uint<3> st3 = 0;
  ap_uint<18> rw1 = 0, rw2 = 0, rw3 = 0;   // result words 1..3 (value ranges: 13-bit counts, 9+9-bit boxes)
  // step 0 = prologue, steps 1..nrows = rows t = step-1, step nrows+1 = final (result words)
  steps: for (ap_uint<10> step = 0; step <= ap_uint<10>(nrows) + 1; ++step) {
#pragma HLS LOOP_TRIPCOUNT min=82 max=402 avg=84
    const bool pro = step == 0;
    const bool fin = !pro && step == ap_uint<10>(nrows) + 1;
    const ap_uint<9> t = ap_uint<9>(step - 1);
    const GInt r = rbase + GInt(t);
    const bool rowstep = !pro && !fin;
    const bool rin = r >= 0 && r < 400;
    const bool frame_rd = rowstep && (win || (merge && t != 0 && t <= 80));
    const bool inreg = rowstep && rin && (merge ? (r >= R0 && r < R1) : true) && ncell != 0;
    const ap_uint<9> rr = ap_uint<9>(r);
    const bool odd = rr[0];
    const ap_uint<7> ly = win ? ap_uint<7>(t) : ap_uint<7>(t - 1);
    const ap_uint<13> ly80 = ap_uint<13>(ly) * 80;
#pragma HLS BIND_OP variable=ly80 op=mul impl=dsp
    const ap_int<24> rc = ap_int<24>(r) * ap_int<24>(cstride);
#pragma HLS BIND_OP variable=rc op=mul impl=dsp
    const GAddr fptr = fbase + GAddr(ly80);
    const GAddr cptr = cbase + GAddr(rc);
    const ap_uint<9> orow = flip ? ap_uint<9>(399 - rr) : rr;
    const ap_uint<16> or100 = ap_uint<16>(orow) * 100;
#pragma HLS BIND_OP variable=or100 op=mul impl=dsp
    const ap_uint<9> w1idx = win ? ap_uint<9>(ly) : ap_uint<9>(rr >> 1);
    const ap_uint<15> w1off = ap_uint<15>(w1idx) * ap_uint<15>(w1mul);
#pragma HLS BIND_OP variable=w1off op=mul impl=dsp
    const ap_uint<18> rr400 = ap_uint<18>(rr) * 400;
#pragma HLS BIND_OP variable=rr400 op=mul impl=dsp
    // ---- read segments (one access site)
    rd_seg: for (ap_uint<2> s = 0; s < 2; ++s) {
      const bool seg0 = s == 0;
      GAddr ra = 0;
      ap_uint<10> rn = 0;
      if (pro) { if (seg0) { ra = win ? GAddr(kGmapOffWparam) : GAddr(kGmapOffParam); rn = 216; } }
      else if (seg0) { if (frame_rd) { ra = fptr; rn = 80; } }
      else if (inreg && !clear) { ra = cptr; rn = asm4 ? ap_uint<10>(ap_uint<10>(ncell) << 2) : ap_uint<10>(ap_uint<10>(ncell) << 1); }
      ap_uint<32> sr = 0;
      rd: for (ap_uint<10> i = 0; i < rn; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=80 max=800 avg=216
        const ap_uint<8> b = map[ra + i];
        sr = (ap_uint<32>(b) << 24) | (sr >> 8);
        if (!pro && seg0) {
          frow[ap_uint<7>(i)] = b;
          if (merge && b != 0xFF) ++known;
        } else if (pro || asm4) {
          if (i.range(1, 0) == 3) rbuf[ap_uint<9>(i >> 2)] = sr;
        } else if (i[0]) {
          rbuf[ap_uint<9>(i >> 1)] = sr >> 16;
        }
      }
    }
    // ---- compute (on-chip only)
    if (pro) {
      // header words rbuf[0..9] (bytes 0..39, little-endian): one read site, shifted into one register (no decode)
      ap_uint<320> hreg = 0;
      hdr: for (ap_uint<4> w = 0; w < 10; ++w) {
#pragma HLS PIPELINE II=1
        hreg = (ap_uint<320>(rbuf[w]) << 288) | (hreg >> 32);
      }
      const ap_uint<32> h0 = hreg.range(31, 0), h1 = hreg.range(63, 32), h2 = hreg.range(95, 64),
                        h3 = hreg.range(127, 96), h4 = hreg.range(159, 128), h5 = hreg.range(191, 160),
                        h6 = hreg.range(223, 192), h7 = hreg.range(255, 224), h8 = hreg.range(287, 256),
                        h9 = hreg.range(319, 288);
      const ap_uint<16> magic = h0.range(15, 0);
      const ap_uint<8> npts = h1.range(31, 24);
      run = op_ok && (win ? (magic == 0x5747 && npts <= 21) : magic == 0x474D);
      ix = ap_int<16>(h0.range(31, 16));
      const ap_int<16> iy = ap_int<16>(h1.range(15, 0));
      ct0 = h1.range(23, 16); ct1 = h1.range(31, 24); ct2 = h2.range(7, 0);
      it0 = h2.range(15, 8); it1 = h2.range(23, 16); it2 = h2.range(31, 24);
      flip = h3[0];
      clr_t = clear && h3[8];
      seed_on = h1[16];
      swept_on = h1[17];
      now = ap_int<32>(h2);
      thr = ap_int<32>(h3);
      rs2 = (ap_uint<48>(h5.range(15, 0)) << 32) | ap_uint<48>(h4);
      re2 = (ap_uint<48>(h7.range(15, 0)) << 32) | ap_uint<48>(h6);
      const ap_uint<48> rb2 = (ap_uint<48>(h9.range(15, 0)) << 32) | ap_uint<48>(h8);
      const GInt ixg = ix, iyg = iy;
      R0 = gclamp(iyg - GInt(iyg & 1));
      R1 = gclamp(iyg + 80 + GInt((iyg + 80) & 1));
      const GInt c0 = merge ? gclamp(GInt((ixg >> 3) << 3)) : win ? gclamp(ixg) : GInt(0);
      const GInt c1 = merge ? gclamp(GInt(((ixg + 87) >> 3) << 3)) : win ? gclamp(ixg + 80) : GInt(400);
      C0 = GCol(c0);
      ncell = c1 > c0 ? GCol(c1 - c0) : GCol(0);
      toff = ap_uint<7>(c0 - ixg);
      tend = ap_uint<7>(c1 - ixg);
      rbase = merge ? GInt(iyg - 1) : win ? GInt(iyg) : GInt(0);
      nrows = !run ? ap_uint<9>(0) : merge ? ap_uint<9>(82) : win ? ap_uint<9>(80) : ap_uint<9>(400);
      // running pointers at row rbase (modular, may start "negative")
      cbase = win ? GAddr(kGmapOffT + (c0 << 2)) : GAddr(c0 << 1);   // + r x 1600 / 800 per row
      cstride = win ? ap_uint<12>(1600) : ap_uint<12>(800);
      fbase = win ? GAddr(kGmapOffAux) : GAddr(kGmapOffFrame);       // + ly x 80
      w1base = win ? GWord(kGmapOffFrame / 4) : GWord(kGmapOffCoarse / 4 + (c0 >> 3));
      w1mul = win ? ap_uint<6>(20) : ap_uint<6>(50);
      // WINDOW jobs: job 0 fills the body tables [0, 160); job p >= 1 fills the box tables of capsule point p - 1
      // [160, 192) and runs its 16 x 16 box -> mask (exact for r_b <= 7 cells). One fill unit, one box unit.
      // The body (cx, cy) sits at words 10, 11, right before the points (words 12 + 2k), so every job reads its
      // coordinates at words 10 + 2 jb.
      if (run && win) {
        const ap_uint<5> njobs = swept_on ? ap_uint<5>(npts.range(4, 0) + 1) : ap_uint<5>(1);
        jobs: for (ap_uint<5> jb = 0; jb < njobs; ++jb) {
#pragma HLS LOOP_TRIPCOUNT min=2 max=22 avg=4
          const bool body = jb == 0;
          const ap_uint<9> pw = ap_uint<9>(10) + (ap_uint<9>(jb) << 1);    // point jb-1 at words 12, 13 + 2(jb-1)
          const ap_int<24> pa = ap_int<24>(rbuf[pw]);
          const ap_int<24> pb = ap_int<24>(rbuf[pw + 1]);
          const ap_int<8> ka = body ? ap_int<8>(0) : ap_int<8>(ap_int<8>(pa >> 16) - 8);
          const ap_int<8> kb = body ? ap_int<8>(0) : ap_int<8>(ap_int<8>(pb >> 16) - 8);
          const ap_uint<8> half = body ? ap_uint<8>(80) : ap_uint<8>(16);
          const ap_uint<8> tb = body ? ap_uint<8>(0) : ap_uint<8>(160);
          fill: for (ap_uint<8> j = 0; j < (half << 1); ++j) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=32 max=160 avg=48
            const bool second = j >= half;
            const ap_uint<8> jj = second ? ap_uint<8>(j - half) : j;
            dtab[tb + j] = axis_d(ap_int<8>((second ? kb : ka) + ap_int<8>(jj)), second ? pb : pa);
          }
          if (!body) {
            // 16 box rows x 17 slots: slot 0 loads the row's dy (one read port), slots 1..16 the cells
            WD ey = 0;
            ap_uint<5> slot = 0;
            ap_uint<4> v = 0;
            mk_box: for (ap_uint<9> j = 0; j < 272; ++j) {
#pragma HLS PIPELINE II=1
              const ap_uint<4> u = ap_uint<4>(slot - 1);
              const WD e = dtab[slot == 0 ? ap_uint<8>(ap_uint<8>(176) + v) : ap_uint<8>(ap_uint<8>(160) + u)];
              if (slot == 0) {
                ey = e;
              } else {
                const ap_int<9> lx = ap_int<9>(ka) + ap_int<9>(u), ly = ap_int<9>(kb) + ap_int<9>(v);
                const WD2 d2 = WD2(e * e) + WD2(ey * ey);
                if (lx >= 0 && lx < 80 && ly >= 0 && ly < 80 && d2 <= WD2(rb2))
                  mask[ap_uint<13>(ly) * 80 + ap_uint<13>(lx)] = 1;
              }
              if (slot == 16) { slot = 0; ++v; } else { ++slot; }
            }
          }
        }
      }
    }
    if (inreg && !win) {
      ap_uint<32> sh = 0, cb = 0;
      ap_uint<2> rk_prev = 0, lb_prev = 0;
      const GInt iyw = rbase + 1;   // MERGE: frame row ly = t - 1
      cells: for (GCol k = 0; k < ncell; ++k) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=88 max=400 avg=88
        const GCol c = C0 + k;
        ap_int<16> L = clear ? ap_int<16>(-32768) : ap_int<16>(rbuf[k].range(15, 0));
        const GInt lx = GInt(c) - GInt(ix);
        if (frame_rd && lx >= 0 && lx < 80) {
          const ap_uint<8> fb = frow[ap_uint<7>(lx)];
          if (fb != 0xFF) {
            ++merged;
            const ap_int<17> v = L == ap_int<16>(-32768) ? ap_int<17>(0) : ap_int<17>(L);
            const ap_int<17> s = v + (fb == 0x64 ? ap_int<17>(6941) : ap_int<17>(-3322));
            L = s < ap_int<17>(-16384) ? ap_int<16>(-16384) : s > ap_int<17>(28762) ? ap_int<16>(28762)
                                                                                     : ap_int<16>(s);
          }
        }
        const bool unseen = L == ap_int<16>(-32768);
        const ap_uint<2> rk = unseen ? ap_uint<2>(1) : L > 0 ? ap_uint<2>(2) : ap_uint<2>(0);
        if (scan && !unseen) {
          any = true;
          if (rr < rmin) rmin = rr;
          if (rr > rmax) rmax = rr;
          if (c < cmin) cmin = c;
          if (c > cmax) cmax = c;
        }
        if (scan) {
          sh = (ap_uint<32>(sel3(it0, it1, it2, rk)) << 24) | (sh >> 8);
          if (c.range(1, 0) == 3) wrow[ap_uint<8>(k >> 2)] = sh;
        } else {
          sh = (ap_uint<32>(ap_uint<16>(L)) << 16) | (sh >> 16);
          if (c[0]) wrow[ap_uint<8>(k >> 1)] = sh;
        }
        if (!odd) {
          lb[k] = rk;
        } else {
          const ap_uint<2> a1 = lb[k];
          if (c[0]) {
            const ap_uint<2> m0 = lb_prev < a1 ? lb_prev : a1, m1 = rk_prev < rk ? rk_prev : rk;
            const ap_uint<2> mn = m0 < m1 ? m0 : m1;
            cb = (ap_uint<32>(sel3(ct0, ct1, ct2, mn)) << 24) | (cb >> 8);
            if (c.range(2, 1) == 3) wco[ap_uint<6>(k >> 3)] = cb;
          }
          lb_prev = a1;
        }
        rk_prev = rk;
      }
    }
    if (rowstep && win) {
      const WD dy = dtab[ap_uint<8>(80) + ap_uint<8>(t)];
      const WD2 dy2 = WD2(dy * dy);
      ap_uint<32> fw = 0;
      wcells: for (ap_uint<7> k = 0; k < 80; ++k) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=mask type=inter false
        const ap_uint<8> aux = frow[k];
        const bool inm = rin && k >= toff && k < tend;
        const ap_uint<7> ti = k - toff;
        const ap_int<32> T = inm ? ap_int<32>(rbuf[ti]) : ap_int<32>(0x80000000u);
        const WD dx = dtab[k];
        const WD2 d2 = WD2(dx * dx) + dy2;
        const ap_uint<13> mi = ly80 + k;
        const bool mk = mask[mi];
        mask[mi] = 0;
        const bool r0 = swept_on && T >= thr;
        const bool strong = r0 && aux[2] && d2 <= WD2(re2);
        const bool r1 = mk || (r0 && !strong);
        const ap_int<32> Tn = mk ? now : strong ? ap_int<32>(0x80000000u) : T;
        if (inm) wrow[ti] = ap_uint<32>(Tn);
        ap_uint<2> f = aux.range(1, 0);
        if (seed_on && f == 0 && d2 <= WD2(rs2)) f = 1;
        if (r1 && f != 1) ++ncl;
        if (strong) ++nev;
        if (r1) f = 1;
        const ap_uint<8> code = f == 2 ? ap_uint<8>(0x64) : f == 1 ? ap_uint<8>(0x00) : ap_uint<8>(0xFF);
        fw = (ap_uint<32>(code) << 24) | (fw >> 8);
        if (k.range(1, 0) == 3) wco[ap_uint<6>(k >> 2)] = fw;
      }
    }
    if (fin) {
      st3 = !op_ok ? ap_uint<3>(4) : !run ? ap_uint<3>(1) : (scan && !any) ? ap_uint<3>(2) : ap_uint<3>(0);
      const bool box = run && scan && any;
      rw1 = run && merge ? ap_uint<18>(known - merged) : run && win ? ap_uint<18>(ncl) : ap_uint<18>(0);
      rw2 = box ? ap_uint<18>((ap_uint<18>(rmax) << 9) | rmin) : run && win ? ap_uint<18>(nev) : ap_uint<18>(0);
      rw3 = box ? ap_uint<18>((ap_uint<18>(cmax) << 9) | cmin) : ap_uint<18>(0);
      st_out = ap_uint<32>(0x200) | st3;
    }
    // ---- write segments (one access site)
    wr_seg: for (ap_uint<2> s = 0; s < 3; ++s) {
      GWord wa = 0;
      ap_uint<9> wn = 0;
      if (fin) { if (s == 0) { wa = kGmapOffResult / 4; wn = 4; } }
      else if (rowstep) {
        if (s == 0) {
          if (inreg) { wa = scan ? GWord(kGmapOffImage / 4 + or100) : GWord(cptr >> 2); wn = scan ? ap_uint<9>(100) : win ? ap_uint<9>(ncell) : ap_uint<9>(ncell >> 1); }
        } else if (s == 1) {
          if (win) { wa = w1base + w1off; wn = 20; }
          else if (inreg && odd) { wa = w1base + w1off; wn = ap_uint<9>(ncell >> 3); }
        } else if (clr_t) { wa = GWord(kGmapOffT / 4) + rr400; wn = 400; }
      }
      wr: for (ap_uint<9> i = 0; i < wn; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=4 max=400 avg=44
        ap_uint<32> d;
        if (fin) {
          // result words: 0x200 | status, w1, w2 / w3 (boxes as max << 16 | min)
          const ap_uint<2> q = i.range(1, 0);
          const ap_uint<18> v = q == 1 ? rw1 : q == 2 ? rw2 : rw3;
          d = q == 0 ? ap_uint<32>(ap_uint<32>(0x200) | st3)
                     : (scan && q != 1) ? ap_uint<32>((ap_uint<32>(v.range(17, 9)) << 16) | v.range(8, 0))
                                        : ap_uint<32>(v);
        }
        else if (s == 0) d = wrow[ap_uint<8>(i)];
        else if (s == 1) d = wco[ap_uint<6>(i)];
        else d = 0x80000000u;
        field[wa + i] = d;
      }
    }
  }
}

// The 68b345d/PLNPATH0 top body, unchanged, as its own dataflow region (gmap_op == 0).
static void planner_df(const ap_uint<8> map[kGmapAreaBytes], ap_uint<32> field[kGmapAreaWords],
                       ap_uint<32> flags[kPlnFlagsDepth], int sx, int sy, int gx, int gy, unsigned k_max,
                       ap_uint<32>& status, ap_uint<32>& rounds, ap_uint<32>& field_valid,
                       unsigned grid_mode, unsigned path_ctrl) {
#pragma HLS INLINE off
#pragma HLS DATAFLOW
  // GMAP3: the arguments come from AXI-Lite registers that the PS never changes while the core runs, so the
  // dataflow region does not need its per-argument synchronisation channels (~0.5k LUT of FIFOs in the OOC).
#pragma HLS STABLE variable=sx
#pragma HLS STABLE variable=sy
#pragma HLS STABLE variable=gx
#pragma HLS STABLE variable=gy
#pragma HLS STABLE variable=k_max
#pragma HLS STABLE variable=grid_mode
#pragma HLS STABLE variable=path_ctrl
  hls::stream<ap_uint<8> > pixels;
  hls::stream<ap_uint<32> > output;
#pragma HLS STREAM variable=pixels depth=32
#pragma HLS STREAM variable=output depth=32
  load_input(map, pixels, grid_mode, path_ctrl);
  compute(pixels, output, sx, sy, gx, gy, k_max, grid_mode, path_ctrl);
  store_output(output, field, flags, status, rounds, field_valid, grid_mode);
}
}  // namespace

extern "C" void planner_field_dual(
    const ap_uint<8> map[kGmapAreaBytes], ap_uint<32> field[kGmapAreaWords],
    ap_uint<32> flags[kPlnFlagsDepth], int sx, int sy, int gx, int gy, unsigned k_max,
    ap_uint<32>& status, ap_uint<32>& rounds, ap_uint<32>& field_valid,
    unsigned grid_mode, ap_uint<1> path_ctrl, ap_uint<3> gmap_op) {
#pragma HLS INTERFACE m_axi port=map offset=slave bundle=map_mem depth=1173088 max_read_burst_length=16 num_read_outstanding=2
#pragma HLS INTERFACE m_axi port=field offset=slave bundle=field_mem depth=293272 max_write_burst_length=16 num_write_outstanding=2
#pragma HLS INTERFACE m_axi port=flags offset=slave bundle=field_mem depth=4353 max_write_burst_length=16 num_write_outstanding=2
#pragma HLS INTERFACE s_axilite port=map bundle=control offset=0x10
#pragma HLS INTERFACE s_axilite port=field bundle=control offset=0x1c
#pragma HLS INTERFACE s_axilite port=flags bundle=control offset=0x28
#pragma HLS INTERFACE s_axilite port=sx bundle=control offset=0x34
#pragma HLS INTERFACE s_axilite port=sy bundle=control offset=0x3c
#pragma HLS INTERFACE s_axilite port=gx bundle=control offset=0x44
#pragma HLS INTERFACE s_axilite port=gy bundle=control offset=0x4c
#pragma HLS INTERFACE s_axilite port=k_max bundle=control offset=0x54
#pragma HLS INTERFACE s_axilite port=status bundle=control offset=0x5c
#pragma HLS INTERFACE s_axilite port=rounds bundle=control offset=0x6c
#pragma HLS INTERFACE s_axilite port=field_valid bundle=control offset=0x7c
#pragma HLS INTERFACE s_axilite port=grid_mode bundle=control offset=0x88
#pragma HLS INTERFACE s_axilite port=path_ctrl bundle=control offset=0x90
#pragma HLS INTERFACE s_axilite port=gmap_op bundle=control offset=0x98
#pragma HLS INTERFACE s_axilite port=return bundle=control
  if (gmap_op == 0) {
    planner_df(map, field, flags, sx, sy, gx, gy, k_max, status, rounds, field_valid, grid_mode, unsigned(path_ctrl));
  } else {
    ap_uint<32> st;
    gmap_core(map, field, gmap_op, st);
    status = st; rounds = 0; field_valid = 0;
  }
}
