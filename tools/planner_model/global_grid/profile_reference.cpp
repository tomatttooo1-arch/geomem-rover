// Host-only capacity characterization. No HLS circuit or vendor headers.
#include "../planner_hw_ref.hpp"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
int main(int argc,char** argv){
  try {
    if(argc!=5)throw std::runtime_error("usage: profile_reference map.u8 sx sy output_prefix");
    constexpr unsigned N=40000,Inf=0x1fffffffu;
    std::vector<std::uint8_t> input(N),scratch(N),flags(N,255);
    std::vector<std::uint32_t> field(N);
    std::ifstream in(argv[1],std::ios::binary);
    if(!in.read(reinterpret_cast<char*>(input.data()),N) || in.peek()!=EOF)throw std::runtime_error("map size");
    int sx=std::stoi(argv[2]),sy=std::stoi(argv[3]);
    if(sx<=0 || sy<=0 || sx>=199 || sy>=199)throw std::runtime_error("start out of interior");
    unsigned rounds=0;
    bool valid=planner_hw_ref::detail::Engine<200,200,Inf>::scan(input.data(),sx,sy,scratch.data(),field.data(),0,rounds,flags.data());
    std::uint32_t max_finite=0;unsigned finite_cells=0;
    for(auto v:field)if(v<Inf){++finite_cells;max_finite=std::max(max_finite,v);}
    std::string prefix(argv[4]);
    std::ofstream out(prefix+".field.u32le",std::ios::binary);
    for(auto v:field)for(int byte=0;byte<4;++byte)out.put(char((v>>(8*byte))&255));
    std::ofstream changed(prefix+".changed.u8",std::ios::binary);
    changed.write(reinterpret_cast<const char*>(flags.data()),rounds);
    if(!out || !changed)throw std::runtime_error("output write");
    std::cout<<"{\"width\":200,\"height\":200,\"infinity\":"<<Inf
      <<",\"reference_round_bound\":40000,\"rounds\":"<<rounds
      <<",\"field_valid\":"<<(valid?"true":"false")<<",\"status\":"<<(valid?0:7)
      <<",\"finite_cells\":"<<finite_cells<<",\"max_finite\":"<<max_finite<<"}\n";
    return valid ? 0 : 2;
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
