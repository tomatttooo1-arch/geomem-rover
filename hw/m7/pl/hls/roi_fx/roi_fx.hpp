#ifndef ROI_FX_HPP
#define ROI_FX_HPP
#include <cstdint>
#include <ap_int.h>
#include <hls_stream.h>
namespace roi_fx {
constexpr unsigned MAX_W=1280, MAX_H=800, MAX_N=1024000, BOXES=32, WORDS=224;
constexpr std::uint32_t COOKIE=0x524f4931u;
// Shared, raw-code-only front end; future fanout is after this decode.
struct Pixel { ap_uint<16> code; ap_uint<11> x; ap_uint<10> y; };
struct Descriptor {
 std::uint32_t w,h,n,minpix,en[32],xy[128],id[32],front[32],back[32];
 std::uint32_t cookie,vbegin,vend,ps_status,ties;
 std::uint64_t valid_m,free_m;
};
}
extern "C" std::uint32_t fpga_m7_object_roi_fx_axi(
 const ap_uint<32>* depth, std::uint32_t width, std::uint32_t height,
 std::uint32_t box_count, const std::uint32_t box_enabled[32],
 const std::uint32_t box_xyxy[128], const std::uint32_t object_id[32],
 std::uint32_t min_pixels, const std::uint32_t front_cut[32],
 const std::uint32_t behind_cut[32], std::uint32_t fx_cookie,
 std::uint32_t v_begin, std::uint32_t v_end, std::uint32_t ps_status,
 std::uint64_t valid_ratio_m, std::uint64_t free_ratio_m,
 std::uint32_t ratio_ties, std::uint32_t result[224]);
#endif
