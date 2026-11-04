#ifndef ROI_OBS_PS_HPP
#define ROI_OBS_PS_HPP
#include <array>
#include <cstdint>
#include <string>
namespace roi_obs_ps {
struct Box { double x0,y0,x1,y1; std::uint32_t id; };
struct Input {
 std::uint32_t width,height,count,min_pixels=9;
 double valid_fraction=.7;
 double K[9],T[16]; const Box* boxes;
 std::uint64_t frame_id=0,scratch_base=0,scratch_bytes=0;
};
struct Compiled {
 std::array<std::uint32_t,914> words{};
 double anchor[3]{};std::uint32_t status=0;std::string reason;
};
Compiled compile(const Input&);
}
#endif
