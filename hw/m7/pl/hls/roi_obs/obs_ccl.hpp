#ifndef ROI_OBS_CCL_HPP
#define ROI_OBS_CCL_HPP
#include <ap_int.h>
#include <hls_stream.h>
namespace roi_obs {
struct CclSummary {
    ap_uint<16> births;    // 0..32768; parent indices remain zero based 15 bit.
    ap_uint<17> max_area;  // Includes 65536, unlike a 16 bit area.
    bool error;
};
// One call consumes exactly width*height D16 codes. Fast profile requires
// 1<=width<=1280, 1<=height<=800 and width*height<=65536.
// Labels: four unsigned 16 bit positive labels per 64 bit word, background 0.
// Parents: two 36 bit records per 72 bit word: parent[14:0], area[31:15],
// rank[35:32]. On return all parent pointers are flattened, and rank bit 0
// means root.area==max_area (rank bits 3:1 are zero). The subsequent raster
// pass picks the first such pixel and freezes its root to break size ties.
// runs is two 640-entry row banks; each record is x0[10:0], x1[21:11],
// positive label[37:22]. These memories are supplied/owned by the caller.
void ccl_fast(hls::stream<ap_uint<16> >& depth,
              ap_uint<11> width, ap_uint<10> height,
              ap_uint<17> lower, ap_uint<17> upper,
              ap_uint<64> labels[16384],
              ap_uint<72> parents[16384],
              ap_uint<38> runs[1280], CclSummary& result);
}
#endif
