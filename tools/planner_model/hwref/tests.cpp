#include "../planner_hw_ref.hpp"
#include <stdexcept>
#include <iostream>
#include <cstring>
void check(bool b){if(!b)throw std::runtime_error("hardware reference control failed");}
int main(){
  using namespace planner_hw_ref;
  std::uint8_t map[6400]={},scratch[6400],flags[6400]={};
  std::uint32_t p[6400];double x[6402],y[6402];
  auto r=plan(map,80,80,{10,10,65,65},scratch,p,x,y,6402,kMaxRounds,flags);
  check(r.status==EXACT && r.field_valid && r.count==56 && r.rounds==2);
  check(p[65*80+65]==55u*700 && p[0]==kInfinity && flags[0]==1 && flags[1]==0);
  check(x[0]==10 && x[55]==65 && y[55]==65);
  for(auto c:map)check(c==0);
  r=plan(map,80,80,{10,10,65,65},scratch,p,x,y,6402,1,flags);
  check(r.status==NOT_CONVERGED && !r.field_valid && r.count==0 && flags[0]==1);
  r=plan(map,400,400,{10,10,65,65},scratch,p,x,y,6402);
  check(r.status==UNSUPPORTED_SIZE && r.rounds==0);
  r=plan(map,80,80,{0,40,65,65},scratch,p,x,y,6402);
  check(r.status==INVALID_START && !r.field_valid);
  map[10*80+10]=255;
  r=plan(map,80,80,{10,10,65,65},scratch,p,x,y,6402);
  check(r.status==EXACT && map[10*80+10]==255 && scratch[10*80+10]==0);
  map[65*80+65]=253;
  r=plan(map,80,80,{10,10,65,65},scratch,p,x,y,6402);
  check(r.status==INVALID_GOAL && !r.field_valid);
  for(unsigned c=0;c<253;++c)check(detail::entry(c)==unsigned(50+.8*c));
  check(detail::Engine<80,80,kInfinity>::add(kInfinity-2,3)==kInfinity);
  check(detail::Engine<80,80,kInfinity>::add(kInfinity-3515,3514)==kInfinity-1);
  std::cout<<"hardware reference controls PASS\n";
}
