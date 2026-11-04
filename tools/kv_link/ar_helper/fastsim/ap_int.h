// AR2NANO (nationals 2026-11-02) fast stand-in for the Vitis HLS ap_int.h, ONLY for compiling the
// unchanged AR core (hw/m7/ar_overlay.cpp, hw/m7/pl/hls/m7_ar_overlay_axi.cpp) and m7ar_present on the
// Jetson Orin Nano CPU (make nano). It implements exactly the subset those files use: ap_uint<W> for
// W <= 64 (value kept modulo 2^W) and W = 128 (two 64-bit halves), construction/assignment from
// integers, implicit conversion to an integer, range(hi, lo) read/write for fields inside one 64-bit
// half, to_uint(). Anything else does not compile (by design). The Xilinx header stays the reference:
// tools/ar_display/nano_ar_check.py proves the picture byte-identical against the C core built with the
// Xilinx headers and against the independent numpy golden.
#ifndef AR2NANO_FAST_AP_INT_H
#define AR2NANO_FAST_AP_INT_H
#include <cassert>
#include <cstdint>
#include <type_traits>

namespace ar2nano_detail {
template <int W> struct Mask { static const std::uint64_t v = (W >= 64) ? ~0ull : ((1ull << W) - 1); };
}

template <int W> struct ap_uint {
    static_assert(W > 0 && W <= 64, "fast ap_uint: W <= 64 or W == 128 only");
    std::uint64_t v;
    ap_uint() : v(0) {}
    ap_uint(unsigned long long x) : v(x & ar2nano_detail::Mask<W>::v) {}
    ap_uint(unsigned long x) : v(static_cast<std::uint64_t>(x) & ar2nano_detail::Mask<W>::v) {}
    ap_uint(unsigned x) : v(x & ar2nano_detail::Mask<W>::v) {}
    ap_uint(int x) : v(static_cast<std::uint64_t>(static_cast<long long>(x)) & ar2nano_detail::Mask<W>::v) {}
    ap_uint(long x) : v(static_cast<std::uint64_t>(x) & ar2nano_detail::Mask<W>::v) {}
    ap_uint(long long x) : v(static_cast<std::uint64_t>(x) & ar2nano_detail::Mask<W>::v) {}
    // a part-select of the 128-bit word (its proxy converts to an integer)
    template <typename T, typename = typename std::enable_if<std::is_class<T>::value>::type>
    ap_uint(const T& x) : v(static_cast<unsigned long long>(x) & ar2nano_detail::Mask<W>::v) {}
    operator unsigned long long() const { return v; }
    unsigned to_uint() const { return static_cast<unsigned>(v); }
};

template <> struct ap_uint<128> {
    std::uint64_t w[2];
    ap_uint() { w[0] = w[1] = 0; }
    ap_uint(int x) { w[0] = static_cast<std::uint64_t>(static_cast<long long>(x)); w[1] = x < 0 ? ~0ull : 0; }
    ap_uint(unsigned x) { w[0] = x; w[1] = 0; }
    struct Ref {
        std::uint64_t* p;
        int sh;
        std::uint64_t m;
        Ref(std::uint64_t* p_, int sh_, std::uint64_t m_) : p(p_), sh(sh_), m(m_) {}
        Ref(const Ref&) = default;
        Ref& operator=(unsigned long long x) { *p = (*p & ~(m << sh)) | ((x & m) << sh); return *this; }
        Ref& operator=(const Ref& o) { return *this = static_cast<unsigned long long>(o); }
        operator unsigned long long() const { return (*p >> sh) & m; }
        unsigned to_uint() const { return static_cast<unsigned>((*p >> sh) & m); }
    };
    struct CRef {
        std::uint64_t val;
        operator unsigned long long() const { return val; }
        unsigned to_uint() const { return static_cast<unsigned>(val); }
    };
    static void check(int hi, int lo) {
        assert(hi >= lo && lo >= 0 && hi < 128 && (hi >> 6) == (lo >> 6));
        (void)hi; (void)lo;
    }
    Ref range(int hi, int lo) {
        check(hi, lo);
        const int n = hi - lo + 1;
        return Ref(&w[lo >> 6], lo & 63, n >= 64 ? ~0ull : ((1ull << n) - 1));
    }
    CRef range(int hi, int lo) const {
        check(hi, lo);
        const int n = hi - lo + 1;
        CRef r = {(w[lo >> 6] >> (lo & 63)) & (n >= 64 ? ~0ull : ((1ull << n) - 1))};
        return r;
    }
};
#endif
