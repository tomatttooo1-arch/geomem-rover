// kv_fast.cpp -- the KV260 frame server's per-pixel data plane in C++ (E7, nationals prelim 2026-11-02).
//
// What it replaces (host/kv_link.py KvLinkServer._serve, the Python path that stays the reference):
//   decode_request : crc32(codes) == the Nano's codes crc            -> kvfast_crc32
//   self.convert   : codes_to_m(codes, unit)  f32(code) * f32(unit), code 0 -> NaN (0x7fc00000)
//   crc32(depth)   : crc32 of the float32 depth bytes                -> both in ONE pass: kvfast_convert_crc
// Bit-exact contract (tests/test_kv_fast_native.py, tools/kv_link/kvfast/rec30_equivalence.py):
//   * crc = zlib crc32 (reflected polynomial 0xEDB88320, init/final ~0), byte stream = little-endian memory;
//   * one IEEE-754 single-precision multiply per pixel, round to nearest, no FMA (-ffp-contract=off),
//     AArch64 Advanced SIMD is IEEE (no flush-to-zero under Linux's default FPCR), as numpy's float32 multiply;
//   * code 0 -> 0x7fc00000 (numpy's float32(np.nan)), whatever the unit.
// Fast paths: AArch64 NEON for the conversion; the ARMv8 CRC32 instructions (crc32x/crc32b, same polynomial as
// zlib) when the CPU says so at run time (getauxval HWCAP_CRC32, the KV260's Cortex-A53 has them); otherwise a
// slicing-by-8 table. x86 hosts run the portable code (same results; tests compare against zlib/numpy).
// Plain C ABI for ctypes (the GIL is released during the call). No allocation, no state but the CRC tables.
#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(__aarch64__)
#include <arm_neon.h>
#include <sys/auxv.h>
#ifndef HWCAP_CRC32
#define HWCAP_CRC32 (1 << 7)
#endif
#endif

#ifndef KVFAST_MUTANT
#define KVFAST_MUTANT 0   // tests' positive controls only: 1 = zero code not NaN, 2 = crc of the last byte skipped
#endif

namespace {

const int kAbi = 1;
const uint32_t kNanBits = 0x7fc00000u;

uint32_t g_table[8][256];
bool g_table_ready = false;
int g_hw_crc = -1;   // -1 unknown, 0 table, 1 ARMv8 crc32 instructions
int g_force_table = 0;

void build_tables() {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        g_table[0][i] = c;
    }
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = g_table[0][i];
        for (int t = 1; t < 8; ++t) {
            c = g_table[0][c & 0xff] ^ (c >> 8);
            g_table[t][i] = c;
        }
    }
    g_table_ready = true;
}

// running crc (already inverted state), slicing-by-8
uint32_t crc_table_update(uint32_t c, const uint8_t* p, size_t n) {
    while (n && (reinterpret_cast<uintptr_t>(p) & 7)) {
        c = g_table[0][(c ^ *p++) & 0xff] ^ (c >> 8);
        --n;
    }
    while (n >= 8) {
        uint64_t v;
        std::memcpy(&v, p, 8);   // little-endian hosts only (x86-64, aarch64 LE): checked in kvfast_selftest
        uint32_t lo = static_cast<uint32_t>(v) ^ c;
        uint32_t hi = static_cast<uint32_t>(v >> 32);
        c = g_table[7][lo & 0xff] ^ g_table[6][(lo >> 8) & 0xff] ^ g_table[5][(lo >> 16) & 0xff] ^
            g_table[4][lo >> 24] ^ g_table[3][hi & 0xff] ^ g_table[2][(hi >> 8) & 0xff] ^
            g_table[1][(hi >> 16) & 0xff] ^ g_table[0][hi >> 24];
        p += 8;
        n -= 8;
    }
    while (n--) c = g_table[0][(c ^ *p++) & 0xff] ^ (c >> 8);
    return c;
}

#if defined(__aarch64__)
__attribute__((target("+crc")))
uint32_t crc_hw_update(uint32_t c, const uint8_t* p, size_t n) {
    while (n && (reinterpret_cast<uintptr_t>(p) & 7)) {
        c = __builtin_aarch64_crc32b(c, *p++);
        --n;
    }
    while (n >= 32) {   // one dependent chain; 4 per iteration only to cut loop overhead
        uint64_t a, b, d, e;
        std::memcpy(&a, p, 8);
        std::memcpy(&b, p + 8, 8);
        std::memcpy(&d, p + 16, 8);
        std::memcpy(&e, p + 24, 8);
        c = __builtin_aarch64_crc32x(c, a);
        c = __builtin_aarch64_crc32x(c, b);
        c = __builtin_aarch64_crc32x(c, d);
        c = __builtin_aarch64_crc32x(c, e);
        p += 32;
        n -= 32;
    }
    while (n >= 8) {
        uint64_t a;
        std::memcpy(&a, p, 8);
        c = __builtin_aarch64_crc32x(c, a);
        p += 8;
        n -= 8;
    }
    while (n--) c = __builtin_aarch64_crc32b(c, *p++);
    return c;
}
#endif

void ensure_init() {
    if (!g_table_ready) build_tables();
    if (g_hw_crc < 0) {
#if defined(__aarch64__)
        g_hw_crc = (getauxval(AT_HWCAP) & HWCAP_CRC32) ? 1 : 0;
#else
        g_hw_crc = 0;
#endif
    }
}

inline uint32_t crc_update(uint32_t c, const uint8_t* p, size_t n) {
#if defined(__aarch64__)
    if (g_hw_crc == 1 && !g_force_table) return crc_hw_update(c, p, n);
#endif
    return crc_table_update(c, p, n);
}

inline uint32_t f32_bits(float f) {
    uint32_t b;
    std::memcpy(&b, &f, 4);
    return b;
}

inline float bits_f32(uint32_t b) {
    float f;
    std::memcpy(&f, &b, 4);
    return f;
}

// scalar reference of one pixel (also the tail of the vector loop)
inline uint32_t convert_one(uint16_t code, float unit) {
#if KVFAST_MUTANT == 1
    return f32_bits(static_cast<float>(code) * unit);
#else
    return code == 0 ? kNanBits : f32_bits(static_cast<float>(code) * unit);
#endif
}

}  // namespace

extern "C" {

int kvfast_abi() { return kAbi; }
int kvfast_mutant() { return KVFAST_MUTANT; }

// 1 = ARMv8 crc32 instructions in use, 0 = table; also what describe() reports
int kvfast_crc_impl() {
    ensure_init();
    return (g_hw_crc == 1 && !g_force_table) ? 1 : 0;
}

// tests: 1 forces the table path even on a CPU with the instructions (both paths compared on the board/qemu)
void kvfast_force_table(int on) { g_force_table = on ? 1 : 0; }

int kvfast_simd() {
#if defined(__aarch64__)
    return 1;   // NEON conversion
#else
    return 0;   // portable loop (the compiler may vectorize it)
#endif
}

// zlib-compatible crc32 of n bytes
uint32_t kvfast_crc32(const void* data, int64_t n) {
    ensure_init();
    if (n <= 0) return 0;
    uint32_t c = crc_update(0xFFFFFFFFu, static_cast<const uint8_t*>(data), static_cast<size_t>(n));
    return ~c;
}

// codes[n] (u16, little-endian, any byte alignment) -> out[n] (4-byte aligned) float32 bits = f32(code) * f32(unit_bits), code 0 -> NaN; returns crc32 of out's
// n*4 bytes in *depth_crc. out and codes must not overlap. Returns 0, or -1 on a bad argument (nothing written).
int kvfast_convert_crc(const void* codes, int64_t n, uint32_t unit_bits, float* out, uint32_t* depth_crc) {
    ensure_init();
    if (n < 0 || (n > 0 && (codes == nullptr || out == nullptr)) || depth_crc == nullptr) return -1;
    const float unit = bits_f32(unit_bits);
    const uint8_t* cb = reinterpret_cast<const uint8_t*>(codes);
    uint32_t* o = reinterpret_cast<uint32_t*>(out);
    uint32_t c = 0xFFFFFFFFu;
    int64_t i = 0;
    // blocks of 1024 pixels: convert (vector), then crc the 4 KiB just written while it is in L1
    const int64_t BLOCK = 1024;
    while (i < n) {
        const int64_t end = (n - i > BLOCK) ? i + BLOCK : n;
        int64_t j = i;
#if defined(__aarch64__)
        const float32x4_t vu = vdupq_n_f32(unit);
        const uint32x4_t vnan = vdupq_n_u32(kNanBits);
        for (; j + 8 <= end; j += 8) {
            // byte load: the codes sit at any offset of the request (odd when the identity strings are)
            uint16x8_t v = vreinterpretq_u16_u8(vld1q_u8(cb + 2 * j));
            uint32x4_t lo = vmovl_u16(vget_low_u16(v));
            uint32x4_t hi = vmovl_u16(vget_high_u16(v));
            uint32x4_t flo = vreinterpretq_u32_f32(vmulq_f32(vcvtq_f32_u32(lo), vu));
            uint32x4_t fhi = vreinterpretq_u32_f32(vmulq_f32(vcvtq_f32_u32(hi), vu));
#if KVFAST_MUTANT != 1
            flo = vbslq_u32(vceqq_u32(lo, vdupq_n_u32(0)), vnan, flo);
            fhi = vbslq_u32(vceqq_u32(hi, vdupq_n_u32(0)), vnan, fhi);
#else
            (void)vnan;
#endif
            vst1q_u32(o + j, flo);
            vst1q_u32(o + j + 4, fhi);
        }
#endif
        for (; j < end; ++j) {
            uint16_t code;
            std::memcpy(&code, cb + 2 * j, 2);
            o[j] = convert_one(code, unit);
        }
        size_t bytes = static_cast<size_t>(end - i) * 4;
#if KVFAST_MUTANT == 2
        if (end == n && bytes) --bytes;
#endif
        c = crc_update(c, reinterpret_cast<const uint8_t*>(o + i), bytes);
        i = end;
    }
    *depth_crc = (n > 0) ? ~c : 0u;
    return 0;
}

// self test on fixed vectors (crc of "123456789" = 0xCBF43926; byte order). 0 = pass.
int kvfast_selftest() {
    ensure_init();
    const char* s = "123456789";
    if (kvfast_crc32(s, 9) != 0xCBF43926u) return 1;
    uint32_t one = 1;
    unsigned char b0;
    std::memcpy(&b0, &one, 1);
    if (b0 != 1) return 2;   // big-endian host: the wire is little-endian, refuse
    return 0;
}

}  // extern "C"
