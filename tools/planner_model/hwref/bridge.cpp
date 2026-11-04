#include "../planner_hw_ref.hpp"
#include <vector> // host test bridge only; excluded from PL scan call tree
extern "C" int hwref_plan(const std::uint8_t* map,int w,int h,int sx,int sy,
    double gx,double gy,unsigned limit,std::uint32_t* field,double* ox,double* oy,
    std::uint8_t* flags,int* state,unsigned* rounds,int* field_valid,double* resolved,int diagnostic){
  std::vector<std::uint8_t> scratch(w*h);
  planner_hw_ref::Request q{sx,sy,gx,gy};planner_hw_ref::Result r;
  if(diagnostic && w==400 && h==400)
    r=planner_hw_ref::detail::Engine<400,400,0x1fffffffu>::plan(map,q,scratch.data(),field,ox,oy,w*h+2,limit,flags);
  else r=planner_hw_ref::plan(map,w,h,q,scratch.data(),field,ox,oy,w*h+2,limit,flags);
  *state=r.status;*rounds=r.rounds;*field_valid=r.field_valid;
  resolved[0]=r.resolved_x;resolved[1]=r.resolved_y;return r.count;
}
extern "C" unsigned hwref_entry(unsigned c){return planner_hw_ref::detail::entry(c);}
extern "C" unsigned hwref_add(unsigned a,unsigned b){return planner_hw_ref::detail::Engine<80,80,0xffffffu>::add(a,b);}
extern "C" unsigned hwref_k(){return planner_hw_ref::kMaxRounds;}
