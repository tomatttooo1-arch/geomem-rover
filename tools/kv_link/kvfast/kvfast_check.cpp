// kvfast_check.cpp -- stand-alone driver of kv_fast.cpp for the cross-compiled (aarch64) check under
// qemu-aarch64 on the host and for the board (E7, 2026-11-02). Links kv_fast.cpp directly (no Python).
//   kvfast_check convert <codes.bin> <n_per_frame> <unit_bits_hex> <byte_offset> <out_f32.bin> <out_crc.txt> [table]
//       codes.bin = frames of n_per_frame little-endian u16; byte_offset (0 or 1) shifts the codes in memory
//       (odd = an unaligned request payload, as on the wire). Writes all floats and per frame
//       "<codes_crc> <depth_crc>" (hex). "table" forces the slicing-by-8 CRC even when the CPU has crc32.
//   kvfast_check bench <codes.bin> <n_per_frame> <unit_bits_hex> <repeats>
//       per-frame timings (ms) of kvfast_crc32(codes) and kvfast_convert_crc, medians on stdout (board).
//   kvfast_check info   -> crc impl, simd, selftest
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

extern "C" {
int kvfast_selftest();
int kvfast_crc_impl();
int kvfast_simd();
void kvfast_force_table(int on);
uint32_t kvfast_crc32(const void* data, int64_t n);
int kvfast_convert_crc(const void* codes, int64_t n, uint32_t unit_bits, float* out, uint32_t* depth_crc);
}

static std::vector<uint8_t> read_file(const char* path) {
    std::vector<uint8_t> buf;
    FILE* f = std::fopen(path, "rb");
    if (!f) { std::perror(path); std::exit(2); }
    std::fseek(f, 0, SEEK_END);
    long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    buf.resize(static_cast<size_t>(sz) + 16);
    if (std::fread(buf.data() + 8, 1, static_cast<size_t>(sz), f) != static_cast<size_t>(sz)) { std::exit(2); }
    std::fclose(f);
    buf.resize(static_cast<size_t>(sz) + 8);
    return buf;   // data starts at +8 (8-byte aligned); callers add their own offset
}

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    if (!std::strcmp(argv[1], "info")) {
        std::printf("selftest %d crc %s simd %s\n", kvfast_selftest(), kvfast_crc_impl() ? "hw" : "table",
                    kvfast_simd() ? "neon" : "portable");
        return 0;
    }
    if (argc < 6) return 2;
    std::vector<uint8_t> raw = read_file(argv[2]);
    const int64_t n = std::atoll(argv[3]);
    const uint32_t unit_bits = static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 16));
    const size_t bytes = raw.size() - 8;
    const int64_t frames = static_cast<int64_t>(bytes / (2 * n));
    if (kvfast_selftest() != 0) { std::fprintf(stderr, "selftest failed\n"); return 3; }
    std::vector<float> out(static_cast<size_t>(n));
    if (!std::strcmp(argv[1], "convert")) {
        if (argc < 8) return 2;
        const int off = std::atoi(argv[5]);
        if (argc > 8 && !std::strcmp(argv[8], "table")) kvfast_force_table(1);
        std::vector<uint8_t> shifted(bytes + 16);
        std::memcpy(shifted.data() + 8 + off, raw.data() + 8, bytes);
        FILE* fo = std::fopen(argv[6], "wb");
        FILE* fc = std::fopen(argv[7], "w");
        if (!fo || !fc) return 2;
        for (int64_t f = 0; f < frames; ++f) {
            const uint8_t* src = shifted.data() + 8 + off + static_cast<size_t>(f) * 2 * n;
            uint32_t ccrc = kvfast_crc32(src, 2 * n), dcrc = 0;
            if (kvfast_convert_crc(src, n, unit_bits, out.data(), &dcrc) != 0) return 4;
            std::fwrite(out.data(), 4, static_cast<size_t>(n), fo);
            std::fprintf(fc, "%08x %08x\n", ccrc, dcrc);
        }
        std::fclose(fo);
        std::fclose(fc);
        std::printf("frames %lld crc %s simd %s\n", static_cast<long long>(frames),
                    kvfast_crc_impl() ? "hw" : "table", kvfast_simd() ? "neon" : "portable");
        return 0;
    }
    if (!std::strcmp(argv[1], "bench")) {
        const int reps = std::atoi(argv[5]);
        std::vector<double> tc, tv;
        for (int r = 0; r < reps; ++r)
            for (int64_t f = 0; f < frames; ++f) {
                const uint8_t* src = raw.data() + 8 + static_cast<size_t>(f) * 2 * n;
                auto t0 = std::chrono::steady_clock::now();
                volatile uint32_t ccrc = kvfast_crc32(src, 2 * n);
                (void)ccrc;
                auto t1 = std::chrono::steady_clock::now();
                uint32_t dcrc = 0;
                kvfast_convert_crc(src, n, unit_bits, out.data(), &dcrc);
                auto t2 = std::chrono::steady_clock::now();
                tc.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
                tv.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
            }
        std::sort(tc.begin(), tc.end());
        std::sort(tv.begin(), tv.end());
        std::printf("{\"frames\": %zu, \"pixels\": %lld, \"crc\": \"%s\", \"codes_crc_ms_median\": %.4f, "
                    "\"convert_crc_ms_median\": %.4f, \"convert_crc_ms_p95\": %.4f}\n", tc.size(),
                    static_cast<long long>(n), kvfast_crc_impl() ? "hw" : "table", tc[tc.size() / 2],
                    tv[tv.size() / 2], tv[tv.size() * 95 / 100]);
        return 0;
    }
    return 2;
}
