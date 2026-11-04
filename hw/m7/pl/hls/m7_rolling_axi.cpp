// AXI shell around the unchanged fpga_m7_rolling_map core (2026-10-26, v3 L2).
// B1h (2026-11-02): with -DFPGA_M7_ROLL_GRID the shell also serves op GRID (two
// #ifdef blocks below, no new port or register); without it the file preprocesses
// to the pre-B1h code.
//
// Why a wrapper: the core's arrays synthesize to ap_memory ports, which a
// Zynq MPSoC block design cannot attach to DDR. The resident map stays inside
// the core (static array bound to URAM); this file only moves the per-call
// buffers onto one m_axi master (bundle gmem, base addresses programmed by the
// PS through s_axilite) and adds NO computation:
//   * obs (read, 64-bit words = 8 cells, layout (z*ny+y)*nx+x, the frame
//     core's flags layout) and query_xyz (read) are burst-copied into local
//     arrays first. The core reads obs one byte per cycle under a condition
//     (ox<onx) and scans it twice (validate + row load); on an m_axi port that
//     would break bursts. Only ceil(onx*ony*onz/8) words are read, and only
//     for INTEGRATE; QUERY reads 3*query_count int32 words;
//   * delta / query_out / header are written to local arrays by the core and
//     burst-copied out afterwards (n_delta words, query_count words, 24 words).
//     dump goes straight to m_axi: the core writes it in order at II=1;
//   * scalars and the two coordinate triples are s_axilite registers; status
//     (= header[0]) is also ap_return.
// Every m_axi port is 64-bit (2026-10-26 trim): query_xyz, query_out and
// header_out are 32-bit arrays in DDR, read/written here as little-endian u64
// pairs (same bytes, so the host buffers are unchanged). With i32 ports HLS
// inferred no burst at all for them ("could not widen ... alignment 1") and
// added sub-word lane logic; u64 ports burst like obs/delta. query_out/header
// are written in whole words: for an odd query_count the u32 after the last
// result is overwritten with an unspecified value (inside the 1024-word
// buffer; the host reads only query_count words). query_xyz likewise reads
// one u32 past 3*query_count when that is odd (inside the 3072-word buffer).
// Buffer contracts are the core's (hw/m7/rolling_map.hpp). obs may be the
// frame core's output buffer in DDR (zero copy, host/kv_link.py ROLL).
#include "../../rolling_map.hpp"

extern "C" std::uint32_t fpga_m7_rolling_axi(
    const std::uint64_t* obs, const std::uint64_t* query_xyz,
    std::uint64_t* query_out, std::uint64_t* delta, std::uint64_t* dump,
    std::uint64_t* header_out,
    std::uint32_t op, std::uint32_t epoch, std::uint32_t frame_seq,
    std::int32_t window_base_x, std::int32_t window_base_y, std::int32_t window_base_z,
    std::int32_t obs_base_x, std::int32_t obs_base_y, std::int32_t obs_base_z,
    std::uint32_t onx, std::uint32_t ony, std::uint32_t onz,
    std::uint32_t query_count) {
// 4 outstanding bursts per direction (default 16): csynth 2026-10-26 m_axi
// adapter 1,460 -> 1,002 LUT. 4 x 16 beats in flight covers a ~64-cycle DDR
// round trip at 100 MHz; longer latency only slows the obs / dump copies.
#pragma HLS INTERFACE mode=m_axi port=obs offset=slave bundle=gmem depth=4096 max_widen_bitwidth=64 num_read_outstanding=4 num_write_outstanding=4
#pragma HLS INTERFACE mode=m_axi port=query_xyz offset=slave bundle=gmem depth=1536 max_widen_bitwidth=64
#pragma HLS INTERFACE mode=m_axi port=query_out offset=slave bundle=gmem depth=512 max_widen_bitwidth=64
#pragma HLS INTERFACE mode=m_axi port=delta offset=slave bundle=gmem depth=5120 max_widen_bitwidth=64
#pragma HLS INTERFACE mode=m_axi port=dump offset=slave bundle=gmem depth=25600 max_widen_bitwidth=64
#pragma HLS INTERFACE mode=m_axi port=header_out offset=slave bundle=gmem depth=12 max_widen_bitwidth=64
#pragma HLS INTERFACE mode=s_axilite port=obs bundle=control
#pragma HLS INTERFACE mode=s_axilite port=query_xyz bundle=control
#pragma HLS INTERFACE mode=s_axilite port=query_out bundle=control
#pragma HLS INTERFACE mode=s_axilite port=delta bundle=control
#pragma HLS INTERFACE mode=s_axilite port=dump bundle=control
#pragma HLS INTERFACE mode=s_axilite port=header_out bundle=control
#pragma HLS INTERFACE mode=s_axilite port=op bundle=control
#pragma HLS INTERFACE mode=s_axilite port=epoch bundle=control
#pragma HLS INTERFACE mode=s_axilite port=frame_seq bundle=control
#pragma HLS INTERFACE mode=s_axilite port=window_base_x bundle=control
#pragma HLS INTERFACE mode=s_axilite port=window_base_y bundle=control
#pragma HLS INTERFACE mode=s_axilite port=window_base_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=obs_base_x bundle=control
#pragma HLS INTERFACE mode=s_axilite port=obs_base_y bundle=control
#pragma HLS INTERFACE mode=s_axilite port=obs_base_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=onx bundle=control
#pragma HLS INTERFACE mode=s_axilite port=ony bundle=control
#pragma HLS INTERFACE mode=s_axilite port=onz bundle=control
#pragma HLS INTERFACE mode=s_axilite port=query_count bundle=control
#pragma HLS INTERFACE mode=s_axilite port=return bundle=control
    // item 6: the checked block as 2-bit cells, 8 per 16-bit entry (one entry
    // per DDR word), entry k in bank k % 2 (2 x 2,048 x 16 bits, was 8 banks of
    // bytes); the GRID cost table bytes by lane (op GRID only, 8 x 32 bytes).
    static std::uint16_t lobs[2][FPGA_M7_ROLL_OBS2_BANK];
#pragma HLS ARRAY_PARTITION variable=lobs type=complete dim=1
    static std::uint8_t ltab[FPGA_M7_ROLL_LANES][FPGA_M7_ROLL_GTAB_BANK];
#pragma HLS ARRAY_PARTITION variable=ltab type=complete dim=1
    static std::uint64_t ldelta[FPGA_M7_ROLL_DELTA_CAPACITY];
#ifdef FPGA_M7_ROLL_GRID
// B1h: GRID planes need all 64 bits, so HLS can no longer narrow ldelta to 45 bits
// (14 -> 22 BRAM18); 5,120 x 64 fits two URAMs instead.
#pragma HLS BIND_STORAGE variable=ldelta type=ram_2p impl=uram
#endif
    std::uint32_t lheader[FPGA_M7_ROLL_HEADER_WORDS];
// Two banks so the u64 pack loop below runs at II=1; item 6: LUTRAM instead of
// two block RAMs for 24 words.
#pragma HLS ARRAY_PARTITION variable=lheader type=cyclic factor=2
#pragma HLS BIND_STORAGE variable=lheader type=ram_2p impl=lutram

    // Words to fetch. Out-of-range dims fetch nothing: the core rejects them
    // (BAD_ARGS) before it looks at obs.
    // dims are 1..32 when used: 6-bit operands, LUT multipliers (a 32x32
    // product took 2 x 3 DSP)
    std::uint32_t total = static_cast<std::uint32_t>(static_cast<std::uint16_t>(onx & 63u) *
                                                     static_cast<std::uint16_t>(ony & 63u)) * (onz & 63u);
#pragma HLS BIND_OP variable=total op=mul impl=fabric
    std::uint32_t obs_words = 0;
    if (op == FPGA_M7_ROLL_OP_INTEGRATE && onx >= 1 && onx <= FPGA_M7_ROLL_OBS_AXIS &&
        ony >= 1 && ony <= FPGA_M7_ROLL_OBS_AXIS && onz >= 1 && onz <= FPGA_M7_ROLL_OBS_AXIS)
        obs_words = (total + 7) / 8;
#ifdef FPGA_M7_ROLL_GRID
    // B1h op GRID: the 196-byte Nav2 cost table rides in obs (rolling_map.hpp).
    if (op == FPGA_M7_ROLL_OP_GRID)
        obs_words = (FPGA_M7_ROLL_GRID_LUT_SIZE + 7) / 8;
#endif
    // SLIM_FUSE2 item 1: the 0/1/2 check of the block rides on this copy (one
    // word = 8 cells per cycle) instead of a separate 1-byte-per-cycle pass in
    // the core. Only the onx*ony*onz block bytes count: in the last word the
    // lanes past the block (tail = block bytes in it, 0 = all 8) are ignored,
    // exactly as the core's old check over [0, onx*ony*onz). The GRID table
    // (op GRID) is never checked here (obs_check stays false).
    const bool obs_check = op == FPGA_M7_ROLL_OP_INTEGRATE;
    const std::uint32_t last = obs_words - 1;
    const std::uint32_t tail_n = total & 7u;
    bool obs_bad = false;
    std::uint16_t obs_nonzero = 0;   // non-zero block cells (<= 32,768)
obs_in_loop:
    for (std::uint32_t w = 0; w < obs_words; ++w) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=4096
        const std::uint64_t v = obs[w];
        std::uint8_t nz = 0;
        std::uint16_t cells = 0;
        for (std::uint32_t j = 0; j < 8; ++j) {
#pragma HLS UNROLL
            const std::uint8_t b = static_cast<std::uint8_t>(v >> (8 * j));
            cells = static_cast<std::uint16_t>(cells | ((b & 3u) << (2 * j)));
#ifdef FPGA_M7_ROLL_GRID
            if (op == FPGA_M7_ROLL_OP_GRID) ltab[j][w & (FPGA_M7_ROLL_GTAB_BANK - 1)] = b;
#endif
            const bool lane_in = w != last || tail_n == 0 || j < tail_n;
            if (obs_check && lane_in && b > FPGA_M7_ROLL_OCCUPIED) obs_bad = true;
            if (lane_in && b != 0) ++nz;
        }
        lobs[w & 1u][w >> 1] = cells;
        obs_nonzero = static_cast<std::uint16_t>(obs_nonzero + nz);
    }
    const std::int32_t wb[3] = {window_base_x, window_base_y, window_base_z};
    const std::int32_t ob[3] = {obs_base_x, obs_base_y, obs_base_z};
    // QUERY reads query_xyz and writes query_out itself (item 6), like DUMP.
    fpga_m7_rolling_core(op, epoch, frame_seq, wb, ob, onx, ony, onz, obs_bad, obs_nonzero, lobs,
                        ltab, query_count, query_xyz, query_out, ldelta, dump, lheader);

    std::uint32_t n_delta = 0;
    if (op == FPGA_M7_ROLL_OP_INTEGRATE && lheader[FPGA_M7_ROLL_H_STATUS] == FPGA_M7_ROLL_OK)
        n_delta = lheader[FPGA_M7_ROLL_H_N_DELTA];
#ifdef FPGA_M7_ROLL_GRID
    // B1h op GRID: the packed grid / aux / cost planes leave through the delta burst.
    if (op == FPGA_M7_ROLL_OP_GRID && lheader[FPGA_M7_ROLL_H_STATUS] == FPGA_M7_ROLL_OK)
        n_delta = lheader[FPGA_M7_ROLL_H_N_DELTA];
#endif
    if (n_delta > FPGA_M7_ROLL_DELTA_CAPACITY) n_delta = FPGA_M7_ROLL_DELTA_CAPACITY;
delta_out_loop:
    for (std::uint32_t i = 0; i < n_delta; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=5120
        delta[i] = ldelta[i];
    }
header_out_loop:
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_HEADER_WORDS / 2; ++i) {
#pragma HLS PIPELINE II=1
        header_out[i] = static_cast<std::uint64_t>(lheader[2 * i]) |
                        (static_cast<std::uint64_t>(lheader[2 * i + 1]) << 32);
    }
    return lheader[FPGA_M7_ROLL_H_STATUS];
}
