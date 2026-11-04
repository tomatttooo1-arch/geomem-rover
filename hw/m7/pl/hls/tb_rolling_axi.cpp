// Trace-replay testbench for fpga_m7_rolling_axi (2026-10-26, v3 L2).
//
// The expected values are NOT computed here: they were recorded from the host
// CPU build of the same core (host/m7_rolling.py RollingKernelHost, g++ -O2,
// cross-checked call by call against the independent RollingReference) by
// tools/m7_rolling_pl/make_trace.py. Replaying recorded values keeps the
// reference out of this binary, so a -D mutant flag given to the design (and,
// by Vitis, to the testbench) cannot change the expectation.
//
// Trace (little endian): "M7RT", u32 version=1, u32 records; per record
//   u32 op, epoch, frame_seq; i32 window_base[3], obs_base[3];
//   u32 onx, ony, onz, query_count, n_obs_bytes, n_delta, n_qout, n_dump;
//   n_obs_bytes uint8 (padded to 8), 3*query_count i32 (only for QUERY),
//   expected header[24] u32, n_delta u64, n_qout u32, n_dump u64.
// Bit-exact = header (all 24 words), the first n_delta delta words, n_qout
// query words and n_dump dump words are equal; the outputs are pre-filled with
// sentinels so an unwritten word cannot pass. Exit 0 only if every record is
// equal. FPGA_M7_TB_TRACE names the file; FPGA_M7_TB_EXPECT_FAIL=1 inverts the
// verdict (positive control: a mutant core must be caught).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../../rolling_map.hpp"

extern "C" std::uint32_t fpga_m7_rolling_axi(
    const std::uint64_t* obs, const std::uint64_t* query_xyz,
    std::uint64_t* query_out, std::uint64_t* delta, std::uint64_t* dump,
    std::uint64_t* header_out, std::uint32_t op, std::uint32_t epoch,
    std::uint32_t frame_seq, std::int32_t window_base_x, std::int32_t window_base_y,
    std::int32_t window_base_z, std::int32_t obs_base_x, std::int32_t obs_base_y,
    std::int32_t obs_base_z, std::uint32_t onx, std::uint32_t ony, std::uint32_t onz,
    std::uint32_t query_count);

namespace {
bool rd(FILE* f, void* p, std::size_t n) { return std::fread(p, 1, n, f) == n; }
}

int main() {
    const char* path = std::getenv("FPGA_M7_TB_TRACE");
    const char* inv = std::getenv("FPGA_M7_TB_EXPECT_FAIL");
    const bool expect_fail = inv != nullptr && std::strcmp(inv, "1") == 0;
    if (path == nullptr) { std::printf("TB_ERROR FPGA_M7_TB_TRACE not set\n"); return 2; }
    FILE* f = std::fopen(path, "rb");
    if (f == nullptr) { std::printf("TB_ERROR cannot open %s\n", path); return 2; }
    char magic[4];
    std::uint32_t version = 0, records = 0;
    if (!rd(f, magic, 4) || std::memcmp(magic, "M7RT", 4) != 0 || !rd(f, &version, 4) ||
        version != 1 || !rd(f, &records, 4)) {
        std::printf("TB_ERROR bad trace header\n");
        return 2;
    }
    std::vector<std::uint64_t> obs(FPGA_M7_ROLL_OBS_CAPACITY / 8);
    std::vector<std::int32_t> qxyz(3 * FPGA_M7_ROLL_QUERY_CAPACITY);
    std::vector<std::uint32_t> qout(FPGA_M7_ROLL_QUERY_CAPACITY), eqout(FPGA_M7_ROLL_QUERY_CAPACITY);
    std::vector<std::uint64_t> delta(FPGA_M7_ROLL_DELTA_CAPACITY), edelta(FPGA_M7_ROLL_DELTA_CAPACITY);
    std::vector<std::uint64_t> dump(FPGA_M7_ROLL_WORDS), edump(FPGA_M7_ROLL_WORDS);
    std::vector<std::uint32_t> header(FPGA_M7_ROLL_HEADER_WORDS), eheader(FPGA_M7_ROLL_HEADER_WORDS);
    unsigned long mism = 0, words = 0, n_int = 0, n_dump = 0, n_query = 0;
    long first_bad = -1;
    for (std::uint32_t r = 0; r < records; ++r) {
        std::uint32_t h[17];
        if (!rd(f, h, sizeof(h))) { std::printf("TB_ERROR truncated record %u\n", r); return 2; }
        const std::uint32_t op = h[0], epoch = h[1], frame = h[2];
        std::int32_t wb[3], ob[3];
        std::memcpy(wb, &h[3], 12);
        std::memcpy(ob, &h[6], 12);
        const std::uint32_t onx = h[9], ony = h[10], onz = h[11], qc = h[12];
        const std::uint32_t nob = h[13], nd = h[14], nq = h[15], ndump = h[16];
        if (nob > FPGA_M7_ROLL_OBS_CAPACITY || nd > FPGA_M7_ROLL_DELTA_CAPACITY ||
            nq > FPGA_M7_ROLL_QUERY_CAPACITY || qc > FPGA_M7_ROLL_QUERY_CAPACITY + 1 ||
            (ndump != 0 && ndump != FPGA_M7_ROLL_WORDS)) {
            std::printf("TB_ERROR bad sizes record %u\n", r);
            return 2;
        }
        std::fill(obs.begin(), obs.end(), 0x5A5A5A5A5A5A5A5Aull);  // bytes past the block: garbage
        const std::uint32_t nob_pad = (nob + 7) / 8 * 8;
        if (nob_pad && !rd(f, obs.data(), nob_pad)) return 2;
        const std::uint32_t qin = op == FPGA_M7_ROLL_OP_QUERY ? 3 * (qc > FPGA_M7_ROLL_QUERY_CAPACITY ? 0 : qc) : 0;
        if (qin && !rd(f, qxyz.data(), 4 * qin)) return 2;
        if (!rd(f, eheader.data(), 4 * FPGA_M7_ROLL_HEADER_WORDS)) return 2;
        if (nd && !rd(f, edelta.data(), 8 * nd)) return 2;
        if (nq && !rd(f, eqout.data(), 4 * nq)) return 2;
        if (ndump && !rd(f, edump.data(), 8 * ndump)) return 2;
        std::fill(header.begin(), header.end(), 0xDEADBEEFu);
        std::fill(delta.begin(), delta.end(), 0xA5A5A5A5A5A5A5A5ull);
        std::fill(qout.begin(), qout.end(), 0xC3C3C3C3u);
        std::fill(dump.begin(), dump.end(), 0x3C3C3C3C3C3C3C3Cull);
        const std::uint32_t st = fpga_m7_rolling_axi(
            // the 32-bit DDR arrays as the shell's u64 views (same bytes)
            obs.data(), reinterpret_cast<const std::uint64_t*>(qxyz.data()),
            reinterpret_cast<std::uint64_t*>(qout.data()), delta.data(), dump.data(),
            reinterpret_cast<std::uint64_t*>(header.data()),
            op, epoch, frame, wb[0], wb[1], wb[2], ob[0], ob[1], ob[2], onx, ony, onz, qc);
        unsigned long bad = 0;
        if (st != eheader[0]) ++bad;
        for (std::uint32_t i = 0; i < FPGA_M7_ROLL_HEADER_WORDS; ++i) bad += header[i] != eheader[i];
        for (std::uint32_t i = 0; i < nd; ++i) bad += delta[i] != edelta[i];
        for (std::uint32_t i = 0; i < nq; ++i) bad += qout[i] != eqout[i];
        for (std::uint32_t i = 0; i < ndump; ++i) bad += dump[i] != edump[i];
        words += 1 + FPGA_M7_ROLL_HEADER_WORDS + nd + nq + ndump;
        if (op == FPGA_M7_ROLL_OP_INTEGRATE) ++n_int;
        if (ndump) ++n_dump;
        if (nq) ++n_query;
        if (bad) {
            if (mism < 8)
                std::printf("TB_MISMATCH record %u op %u frame %u: %lu words (status got %u want %u, "
                            "n_delta got %u want %u)\n", r, op, frame, bad, st, eheader[0],
                            header[FPGA_M7_ROLL_H_N_DELTA], eheader[FPGA_M7_ROLL_H_N_DELTA]);
            if (first_bad < 0) first_bad = r;
            ++mism;
        }
    }
    std::fclose(f);
    std::printf("TB_SUMMARY records %u integrate %lu dump %lu query %lu words %lu mismatched_records %lu "
                "first_bad %ld\n", records, n_int, n_dump, n_query, words, mism, first_bad);
    const bool equal = mism == 0;
    if (expect_fail) {
        std::printf(equal ? "TB_FAIL positive control NOT caught\n" : "TB_PASS positive control caught\n");
        return equal ? 1 : 0;
    }
    std::printf(equal ? "TB_PASS bit-exact\n" : "TB_FAIL\n");
    return equal ? 0 : 1;
}
