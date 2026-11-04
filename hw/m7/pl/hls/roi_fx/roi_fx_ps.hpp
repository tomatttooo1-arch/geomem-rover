#ifndef ROI_FX_PS_HPP
#define ROI_FX_PS_HPP
#include <cstdint>
namespace roi_fx_ps {
struct Input {
 std::uint32_t w,h,n,en[32],xy[128],id[32],minpix;
 double near_z[32],far_z[32],margin,mvf,ff;
};
struct Compiled {
 std::uint32_t front[32],back[32],vbegin,vend,status,ties;
 std::uint64_t valid_m,free_m;
};
Compiled compile(const Input&,float unit);
bool ratio_ge(std::uint32_t a,std::uint32_t b,std::uint64_t m,bool tie);
void model(const std::uint16_t*,const Input&,const Compiled&,std::uint32_t out[224],std::uint32_t* status);
}
#endif
