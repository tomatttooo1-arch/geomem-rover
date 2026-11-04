// m7ar_fill: the AR sender's per-pixel depth work as two table lookups per pixel (2026-10-16, docs/design/AR_COLOUR_MAIN_20261016.md
// section 13). Bit-for-bit the numpy chain it replaces in host/kv_ar.fill_ar_inputs (kept there as the reference):
//   host.kv_link.codes_to_m          m = float32(code) * float32(unit); code 0 -> NaN
//   host.m7_display_native.quantize_optical_z
//                                    valid = isfinite(m) && m > 0.1f && m < 10.0f   (numpy compares a float32 array
//                                    with a Python float in float32: 1.24 value-based casting, 2.x NEP 50)
//                                    mm = clip(floor(double(m) * 1000.0 + 0.5), 100, 9999), invalid -> 0
//   host.kv_ar.stretch_depth_mm      k > 0: zmin = min valid mm; mm -> min(zmin + floor(double(mm - zmin) * k), 9999)
//                                    (k <= 0 or no valid pixel: unchanged)
// Compiled at kv_server start by host/kv_ar_native.py with -ffp-contract=off (an FMA in floor(x*1000+0.5) would
// round boundary pixels differently on aarch64). C headers only, no libstdc++ use.
// Positive-control mutants (tests only): -DM7AR_FILL_MUTANT=1 stretch without k, =2 truncate instead of round,
// =3 validity compared in double.
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef M7AR_FILL_MUTANT
#define M7AR_FILL_MUTANT 0
#endif

extern "C" {

int m7ar_fill_abi(void) { return 1; }
int m7ar_fill_mutant(void) { return M7AR_FILL_MUTANT; }

// Per-code table: lut[c] = the quantized mm of code c (0 = invalid) for this unit, by the rule above. The codes
// are u16, so 65536 evaluations cover every pixel of any frame; the caller keeps the table while the unit stays.
void m7ar_mm_lut(uint32_t unit_bits, uint16_t* lut) {
    float unit;
    memcpy(&unit, &unit_bits, sizeof unit);
    const float lo = 0.1f, hi = 10.0f;
    lut[0] = 0;                         // code 0 -> NaN -> invalid
    for (int c = 1; c < 65536; ++c) {
        const float m = (float)c * unit;
#if M7AR_FILL_MUTANT == 3
        const bool ok = isfinite(m) && (double)m > 0.1 && (double)m < 10.0;
        (void)lo; (void)hi;
#else
        const bool ok = isfinite(m) && m > lo && m < hi;
#endif
        uint16_t q = 0;
        if (ok) {
#if M7AR_FILL_MUTANT == 2
            double mm = floor((double)m * 1000.0);
#else
            double mm = floor((double)m * 1000.0 + 0.5);
#endif
            if (mm < 100.0) mm = 100.0;
            if (mm > 9999.0) mm = 9999.0;
            q = (uint16_t)mm;
        }
        lut[c] = q;
    }
}

// codes: n uint16 depth codes; lut: m7ar_mm_lut of the frame's unit; k: stretch factor (the caller passes 1.0
// when no stretch is wanted: an exact identity); out: n uint16 mm.
// stats[0] = valid pixels, stats[1] = nearest valid mm (0 = none; unchanged by the stretch: f(zmin) = zmin).
// Returns 0.
int m7ar_depth_apply(const uint16_t* codes, int64_t n, const uint16_t* lut, double k, uint16_t* out,
                     int64_t* stats) {
    int64_t valid = 0;
    unsigned zmin = 0xffffu;
    for (int64_t i = 0; i < n; ++i) {
        const unsigned q = lut[codes[i]];
        out[i] = (uint16_t)q;
        valid += q != 0;
        const unsigned qq = q ? q : 0xffffu;
        zmin = qq < zmin ? qq : zmin;
    }
    stats[0] = valid;
    stats[1] = valid ? (int64_t)zmin : 0;
    if (valid == 0 || !(k > 0.0) || k == 1.0) return 0;
    // valid quantized values are 100..9999 (0 = invalid, kept): a table over them, then one lookup per pixel
    uint16_t s[10000];
    s[0] = 0;
    for (unsigned d = 1; d < 10000; ++d) {
        if (d < zmin) { s[d] = (uint16_t)d; continue; }   // never looked up (zmin is the minimum)
#if M7AR_FILL_MUTANT == 1
        int64_t v = (int64_t)zmin + (int64_t)floor((double)(int64_t)(d - zmin));
#else
        int64_t v = (int64_t)zmin + (int64_t)floor((double)(int64_t)(d - zmin) * k);
#endif
        if (v > 9999) v = 9999;
        s[d] = (uint16_t)v;
    }
    for (int64_t i = 0; i < n; ++i) out[i] = s[out[i]];
    return 0;
}

}  // extern "C"
