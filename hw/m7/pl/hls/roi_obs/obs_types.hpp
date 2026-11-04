#ifndef ROI_OBS_TYPES_HPP
#define ROI_OBS_TYPES_HPP
#include <ap_int.h>
#include <hls_stream.h>
#include <cstdint>
namespace roi_obs {
constexpr unsigned PARAM_WORDS=914, MAX_BOXES=32;
enum Phase : unsigned { DEPTH_HIST=0, COMPONENT=1, COARSE=2, FINE=3 };
struct Descriptor {
 ap_uint<32> id; ap_int<32> x0,y0,x1,y1;
 ap_uint<17> area; ap_uint<2> flags;
 ap_int<40> extent_x,extent_y;
};
struct Geometry { ap_int<40> a[9], rz[3]; };
struct Profile {
 ap_uint<20> min_pixels; ap_uint<56> valid_m; bool valid_tie;
 ap_uint<17> lower[201],upper[201];
};
struct BoxState {
 ap_uint<3> status; ap_uint<3> error;
 ap_uint<17> valid,selected; ap_uint<8> mode;
 ap_uint<17> cut_lo,cut_hi;
 ap_uint<15> root; bool root_seen;
 ap_uint<17> ranks[6];
 ap_uint<5> rank_rem; // Shared n/20 remainder, saved across phases.
 ap_uint<8> windows[3][6];
 ap_uint<17> below[3][6];
 ap_int<16> values[3][6];
 ap_uint<9> hsv2[3];
 ap_int<32> position[3],lower[3],upper[3],range,extent[2];
 ap_uint<3> color;
};
// One phase call site owns all work memories. Input is raster D16 in bits15:0;
// COARSE also carries RGB bytes in bits39:16. No coordinates ride the FIFO.
void obs_compute(ap_uint<2> phase, const Descriptor& desc,
 const Geometry& geometry,const Profile& profile,
 hls::stream<ap_uint<64>>& input, BoxState& state,
 hls::stream<ap_uint<32>>& output);
}
#endif
