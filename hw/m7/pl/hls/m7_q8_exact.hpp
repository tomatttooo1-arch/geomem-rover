// G3 (2026-11-02): exact integer replacement of the reference Q8 coordinate
// conversion of the M7 frame top, without a floating-point divider.
//
//     coordinate = std::floor(a / vox * 256)   (IEEE double, round to nearest
//                                               even; a = double(world) - org)
//     ok = coordinate >= -2^24 && coordinate < 2^24,  c = (int32) coordinate
//
// Only (ok, c) is consumed by the frame top (end[], inside, representable;
// the origin check), so these functions return exactly that. Host-tested
// against the double expression in runs/nationals-prelim-20261102/G3
// (tools/q8_exhaustive.cpp, README section 1). Derivation (from G1d
// tools/proto_m7_q8_exact.hpp, x = a / vox, z = 256 |x|):
//   RN(x) * 256 is exact and equals RN(z) for |x| >= 2^-1022.
//   a >= 0: c = floor(RN(z));  a < 0: c = -ceil(RN(z)); a = -0 gives c = 0.
//   |a| = A 2^ea, vox = V 2^ev, A, V in [2^52, 2^53), s = ea - ev + 8, so
//   z = (A/V) 2^s, A/V in (1/2, 2).
//   * s >= 26: z > 2^25: out of range.
//   * s <= -2: z < 1/2: a >= 0 -> 0; a < 0 -> -1 unless RN(|x|) = 0, i.e.
//     |x| <= 2^-1075 <=> (A/V) 2^(s+1067) <= 1: true if s+1067 <= -1,
//     A <= V if s+1067 = 0, false if s+1067 >= 1.
//   * s in [-1, 25]: z = N / D, N = A << (s+1) < 2^79, D = 2V in
//     [2^53, 2^54); K = floor(N/D) <= 2^26, R = N - K D; z >= 1/4 so x is
//     normal. Let m = floor(log2 K) for K >= 1 and T = floor(D / 2^(53-m))
//     (T = 0 for K = 0).
//       floor(RN(z)) = K + 1 iff D - R <= T   (z within half a gap below
//         K+1; the gap below K+1 is 2^(m-52) for every K >= 1 -- also when
//         K+1 is a power of two, where it is 2^(m'-53) with m' = m+1 -- and
//         for K = 0 the condition is z >= 1 - 2^-54, impossible as R < D;
//         ties go to the even K+1),
//       ceil(RN(z)) = K iff R = 0 or (K >= 1 and R <= T); else K + 1.
//   K without a divider: M = floor(2^79 / D) (frame constant, in
//   (2^25, 2^26]); K' = floor(N M / 2^79) = (A M) >> (78 - s) is K or K-1,
//   corrected by one compare of R' = N - K' D with D. R' < 2D < 2^55, so N
//   and K' D are only needed modulo 2^56.
// Per-pixel path (q8_coord_px): a normal (or zero / non-finite). A subnormal
// nonzero a = RN(double(w) - org) with w a float is only possible for w = +-0
// (then a = -org exactly): if w != 0, |w| >= 2^-149, so |w - org| <
// 2^-1022 forces org to be normal with |org| > 2^-150, and w - org is a
// nonzero multiple of 2^-202 > 2^-1022, a contradiction. The per-pixel path
// therefore returns the frame constant q8_coord_serial(-org) for subnormal a.
// C-model faults (positive controls of the host test, never synthesized):
//   FPGA_M7_FAULT_Q8_NOCORR   per-pixel path skips the K' -> K correction
//   FPGA_M7_FAULT_Q8_HALFGAP  rounding tail uses half the true gap
// Serial path (q8_coord_serial, once per frame): any finite a, subnormals
// normalized, K and R by a 27-step restoring division (no multiplier).
#ifndef FPGA_M7_Q8_EXACT_HPP
#define FPGA_M7_Q8_EXACT_HPP

#include <ap_int.h>
#include <cstdint>

namespace q8x {

struct Q8Vox {
    ap_uint<54> D;     // 2V, V = significand of vox, in [2^53, 2^54)
    ap_uint<27> M;     // floor(2^79 / D), in (2^25, 2^26]
    ap_int<13> ev;     // vox = V * 2^ev
};

inline ap_uint<64> q8_bits(double d) {
    union { double d; std::uint64_t u; } x;
    x.d = d;
    return ap_uint<64>(x.u);
}

// K = floor(N / D), R = N - K D for N < 2^80, D in [2^53, 2^54), K < 2^27:
// 27-step restoring division (frame-rate use only).
inline void q8_div27(ap_uint<80> N, ap_uint<54> D, ap_uint<27>& K, ap_uint<55>& R) {
    ap_uint<55> rem = N >> 27;  // < 2^53 <= D
    ap_uint<27> q = 0;
q8_div_loop:
    for (int i = 26; i >= 0; --i) {
#pragma HLS PIPELINE off
        rem = (rem << 1) | ap_uint<55>(N[i]);
        const bool ge = rem >= ap_uint<55>(D);
        if (ge) rem -= ap_uint<55>(D);
        q[i] = ge;
    }
    K = q;
    R = rem;
}

// highest set bit of a nonzero value of width W
template <int W>
inline int q8_msb(ap_uint<W> f) {
    int p = 0;
    for (int i = 0; i < W; ++i)
#pragma HLS UNROLL
        if (f[i]) p = i;
    return p;
}

// Shared tail: from neg, K (= floor(N/D)), R (= N - K D, R < D) and D, the
// integer c of the reference (before the range gate).
inline ap_int<29> q8_round(bool neg, ap_uint<27> K, ap_uint<55> R, ap_uint<54> D) {
    const int m = q8_msb<27>(K);
#ifdef FPGA_M7_FAULT_Q8_HALFGAP
    const ap_uint<27> T = K == 0 ? ap_uint<27>(0) : ap_uint<27>(D >> (54 - m));  // C-model fault: gap halved
#else
    const ap_uint<27> T = K == 0 ? ap_uint<27>(0) : ap_uint<27>(D >> (53 - m));
#endif
    const ap_uint<55> DR = ap_uint<55>(D) - R;  // in (0, D]
    if (!neg) return ap_int<29>(K) + (DR <= ap_uint<55>(T) ? 1 : 0);
    const bool stay = R == 0 || (K != 0 && R <= ap_uint<55>(T));
    return stay ? ap_int<29>(-ap_int<29>(K)) : ap_int<29>(-ap_int<29>(K) - 1);
}

// s <= -2 tail (z < 1/2)
inline ap_int<29> q8_small(bool neg, ap_int<14> s, ap_uint<53> A, ap_uint<54> D) {
    if (!neg) return 0;
    const ap_int<14> t = s + 1067;
    return (t <= -1 || (t == 0 && A <= ap_uint<53>(D >> 1))) ? ap_int<29>(0) : ap_int<29>(-1);
}

inline bool q8_gate(ap_int<29> r, std::int32_t& c) {
    if (r < -(1 << 24) || r >= (1 << 24)) return false;
    c = r.to_int();
    return true;
}

// (significand in [2^52, 2^53), exponent) of a positive finite nonzero double
inline void q8_unpack_any(ap_uint<11> e, ap_uint<52> f, ap_uint<53>& sig, ap_int<13>& ex) {
    if (e != 0) {
        sig = (ap_uint<53>(1) << 52) | ap_uint<53>(f);
        ex = ap_int<13>(e) - 1075;
    } else {  // subnormal: value = f * 2^-1074
        const int sh = 52 - q8_msb<53>(ap_uint<53>(f));
        sig = ap_uint<53>(f) << sh;
        ex = ap_int<13>(-1074) - sh;
    }
}

// One frame-rate step (G3 step 2: the only serial call site in the top).
// recip = true: K = floor(2^79 / D) (the frame constant M; b unused).
// recip = false: the exact conversion of the double with bits b: returns ok
// and c (non-finite -> false). One 27-step restoring division either way.
inline void q8_serial_step(bool recip, ap_uint<64> b, ap_uint<54> D, ap_int<13> ev,
                           ap_uint<27>& K, bool& ok, std::int32_t& c) {
    const bool neg = b[63];
    const ap_uint<11> e = b(62, 52);
    const ap_uint<52> f = b(51, 0);
    const bool zero = e == 0 && f == 0;
    ap_uint<53> A = 0;
    ap_int<13> ea = 0;
    if (!zero) q8_unpack_any(e, f, A, ea);
    const ap_int<14> s = ap_int<14>(ea) - ev + 8;
    const bool mid = s >= -1 && s <= 25;
    const ap_uint<5> sh = mid ? ap_uint<5>(s + 1) : ap_uint<5>(0);
    const ap_uint<80> N = recip ? (ap_uint<80>(1) << 79) : ap_uint<80>(ap_uint<80>(A) << sh);
    ap_uint<55> R;
    q8_div27(N, D, K, R);
    const ap_int<29> r = s <= -2 ? q8_small(neg, s, A, D) : q8_round(neg, K, R, D);
    std::int32_t cg = 0;
    const bool okg = q8_gate(r, cg);
    ok = e != 0x7ff && (zero || (s < 26 && okg));
    c = zero ? 0 : cg;
}

// D and ev of vox (finite, > 0): no division.
inline void q8_unpack_vox(double vox, Q8Vox& p) {
    const ap_uint<64> b = q8_bits(vox);
    ap_uint<53> V;
    q8_unpack_any(b(62, 52), b(51, 0), V, p.ev);
    p.D = ap_uint<54>(V) << 1;
}

// Frame constant (vox finite, > 0).
inline Q8Vox q8_prepare(double vox) {
    Q8Vox p;
    q8_unpack_vox(vox, p);
    bool ok;
    std::int32_t c;
    q8_serial_step(true, 0, p.D, p.ev, p.M, ok, c);
    return p;
}

// Any a given by its bits (non-finite -> false); frame-rate use.
inline bool q8_coord_serial_bits(ap_uint<64> b, const Q8Vox& p, std::int32_t& c) {
    ap_uint<27> K;
    bool ok;
    q8_serial_step(false, b, p.D, p.ev, K, ok, c);
    return ok;
}

inline bool q8_coord_serial(double a, const Q8Vox& p, std::int32_t& c) {
    return q8_coord_serial_bits(q8_bits(a), p, c);
}

// The per-pixel frame constant for a subnormal a: q8_coord_serial(-org),
// with the sign flipped on the bits (no floating-point operator).
inline bool q8_coord_neg_org(double org, const Q8Vox& p, std::int32_t& c) {
    ap_uint<64> b = q8_bits(org);
    b[63] = !b[63];
    return q8_coord_serial_bits(b, p, c);
}

// Per-pixel path (II=1): a normal, zero or non-finite; a subnormal a returns
// the frame constant (sub_ok, sub_c) = q8_coord_serial(-org) (see header).
inline bool q8_coord_px(double a, const Q8Vox& p, bool sub_ok, std::int32_t sub_c,
                        std::int32_t& c) {
    const ap_uint<64> b = q8_bits(a);
    const bool neg = b[63];
    const ap_uint<11> e = b(62, 52);
    const ap_uint<52> f = b(51, 0);
    if (e == 0x7ff) return false;
    if (e == 0) {
        if (f == 0) { c = 0; return true; }
        c = sub_c;
        return sub_ok;
    }
    const ap_uint<53> A = (ap_uint<53>(1) << 52) | ap_uint<53>(f);
    const ap_int<14> s = ap_int<14>(ap_int<13>(e) - 1075) - p.ev + 8;
    if (s >= 26) return false;
    ap_int<29> r;
    if (s <= -2) {
        r = q8_small(neg, s, A, p.D);
    } else {
        const ap_uint<5> sh = s + 1;               // 0..26
        const ap_uint<80> P = ap_uint<80>(A) * p.M;  // 53 x 27 bits
        ap_uint<27> K = P >> (ap_uint<7>(78 - s));  // (A M 2^sh) >> 79
        const ap_uint<56> N = ap_uint<56>(A) << sh;  // modulo 2^56
        const ap_uint<56> KD = ap_uint<56>(K) * ap_uint<56>(p.D);
        ap_uint<56> R = N - KD;                    // < 2D < 2^55
#ifdef FPGA_M7_FAULT_Q8_NOCORR
        if (false) {  // C-model fault: no correction of K' = K - 1
#else
        if (R >= ap_uint<56>(p.D)) {
#endif
            K = K + 1;
            R = R - ap_uint<56>(p.D);
        }
        r = q8_round(neg, K, ap_uint<55>(R), p.D);
    }
    return q8_gate(r, c);
}

}  // namespace q8x

#endif
