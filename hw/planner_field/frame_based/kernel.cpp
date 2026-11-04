#include "kernel_hls.hpp"
#include "tools/planner_model/planner_hw_ref.hpp"
extern "C" void planner_field(const ap_uint<8> map[6400],ap_uint<32> field[6400],
    ap_uint<32> flags[64],int sx,int sy,int gx,int gy,unsigned k_max,
    ap_uint<32>& status,ap_uint<32>& rounds,ap_uint<32>& field_valid){
  std::uint8_t input[6400],scratch[6400],trace[64];std::uint32_t potential[6400];
  for(int i=0;i<6400;++i)input[i]=map[i].to_uint();
  auto r=planner_hw_ref::build_field(input,sx,sy,gx,gy,scratch,potential,trace,k_max);
  for(int i=0;i<6400;++i)field[i]=potential[i];
  for(int i=0;i<64;++i)flags[i]=trace[i];
  status=r.status;rounds=r.rounds;field_valid=r.field_valid;
}
