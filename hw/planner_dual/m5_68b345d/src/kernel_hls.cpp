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

static void load_input(const ap_uint<8> map[40000],
                       hls::stream<ap_uint<8> >& pixels, unsigned grid_mode) {
#pragma HLS INLINE off
  const Index count = grid_mode == 0 ? Index(6400)
      : grid_mode == 1 ? Index(40000) : Index(0);
  load_loop: for (Index i = 0; i < count; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40000 avg=40000
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

static void compute(hls::stream<ap_uint<8> >& pixels,
                    hls::stream<ap_uint<32> >& output,
                    int sx, int sy, int gx, int gy, unsigned k_max,
                    unsigned grid_mode) {
#pragma HLS INLINE off
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
  export_field: for (Index i = 0; i < count; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40000 avg=40000
    const Cell packed = cells[i];
    const Potential phi = packed.range(28, 0);
    output.write(valid ? ap_uint<32>(phi) : ap_uint<32>(inf));
  }
  export_flags: for (Round r = 0; r < flag_count; ++r) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=64 max=256 avg=256
    output.write(ap_uint<32>(flags[r]));
  }
  output.write(status);
  output.write(ap_uint<32>(rounds));
  output.write(ap_uint<32>(valid));
}

static void store_output(hls::stream<ap_uint<32> >& input,
                         ap_uint<32> field[40000], ap_uint<32> flags[256],
                         ap_uint<32>& status, ap_uint<32>& rounds,
                         ap_uint<32>& field_valid, unsigned grid_mode) {
#pragma HLS INLINE off
  const bool supported = grid_mode <= 1;
  const Index count = supported ? (grid_mode == 1 ? Index(40000) : Index(6400)) : Index(0);
  const Round flag_count = supported ? (grid_mode == 1 ? Round(256) : Round(64)) : Round(0);
  store_field: for (Index i = 0; i < count; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6400 max=40000 avg=40000
    field[i] = input.read();
  }
  store_flags: for (Round r = 0; r < flag_count; ++r) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=64 max=256 avg=256
    flags[r] = input.read();
  }
  status = input.read();
  rounds = input.read();
  field_valid = input.read();
}
}  // namespace

extern "C" void planner_field_dual(
    const ap_uint<8> map[40000], ap_uint<32> field[40000],
    ap_uint<32> flags[256], int sx, int sy, int gx, int gy, unsigned k_max,
    ap_uint<32>& status, ap_uint<32>& rounds, ap_uint<32>& field_valid,
    unsigned grid_mode) {
#pragma HLS INTERFACE m_axi port=map offset=slave bundle=map_mem depth=40000 max_read_burst_length=16 num_read_outstanding=2
#pragma HLS INTERFACE m_axi port=field offset=slave bundle=field_mem depth=40000 max_write_burst_length=16 num_write_outstanding=2
#pragma HLS INTERFACE m_axi port=flags offset=slave bundle=field_mem depth=256 max_write_burst_length=16 num_write_outstanding=2
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
#pragma HLS INTERFACE s_axilite port=return bundle=control
#pragma HLS DATAFLOW
  hls::stream<ap_uint<8> > pixels;
  hls::stream<ap_uint<32> > output;
#pragma HLS STREAM variable=pixels depth=32
#pragma HLS STREAM variable=output depth=32
  load_input(map, pixels, grid_mode);
  compute(pixels, output, sx, sy, gx, gy, k_max, grid_mode);
  store_output(output, field, flags, status, rounds, field_valid, grid_mode);
}
