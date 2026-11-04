// Host-only full-field comparison. Never used as the HLS top or a GPU baseline.
#include "../planner_hw_ref.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
extern "C" int v3_field_only(const uint8_t*, int, int, int, int, uint64_t*, uint32_t*);
namespace {
unsigned get(std::istream& in, int n=4) {
  unsigned v=0; for(int j=0;j<n;++j){int b=in.get();if(b<0)throw std::runtime_error("truncated input");v|=unsigned(b)<<(8*j);}return v;
}
struct Case {
  std::array<uint8_t,6400> map;
  std::array<uint32_t,6400> expected;
  int sx,sy,gx,gy;unsigned k,index;
};
std::vector<Case> read(const char* input, const char* golden) {
  std::ifstream in(input,std::ios::binary),go(golden,std::ios::binary);
  char a[8],b[8];in.read(a,8);go.read(b,8);
  if(std::string(a,8)!="FPGAFIN01"||std::string(b,8)!="FPGAFGO01")throw std::runtime_error("bad magic");
  unsigned count=get(in);if(count!=get(go))throw std::runtime_error("bad count");
  std::vector<Case> result;
  for(unsigned c=0;c<count;++c){
    Case q;q.index=c;q.sx=int(int32_t(get(in)));q.sy=int(int32_t(get(in)));q.gx=int(int32_t(get(in)));q.gy=int(int32_t(get(in)));q.k=get(in);
    for(auto& v:q.map)v=uint8_t(get(in,1));
    unsigned status=get(go),rounds=get(go),valid=get(go);(void)rounds;
    for(auto& v:q.expected)v=get(go,3);
    for(int i=0;i<64;++i)get(go,1);
    if(valid){if(status!=0)throw std::runtime_error("unexpected field status");result.push_back(q);}
  }
  if(in.get()!=EOF||go.get()!=EOF)throw std::runtime_error("trailing bytes");
  return result;
}
struct Buffers {
  std::array<uint8_t,6400> scratch;
  std::array<uint8_t,64> flags;
  std::array<uint32_t,6400> field;
  std::array<uint64_t,6400> v3;
  std::array<float,6400> nav;
};
void run(unsigned mode,const Case& q,Buffers& b){
  if(mode==0){
    uint32_t rounds=0;
    if(v3_field_only(q.map.data(),q.sx,q.sy,q.gx,q.gy,b.v3.data(),&rounds)!=0||!rounds)throw std::runtime_error("v3 field failed");
  }else if(mode==1){
    auto r=planner_hw_ref::build_field(q.map.data(),q.sx,q.sy,q.gx,q.gy,b.scratch.data(),b.field.data(),b.flags.data(),q.k);
    if(r.status||!r.field_valid)throw std::runtime_error("B field failed");
  }else{
    std::vector<uint8_t> map(q.map.begin(),q.map.end());map[q.sy*80+q.sx]=0;
    nav2_navfn_planner::NavFn nav(80,80);nav.setCostmap(map.data(),true,false);
    int source[2]={q.sx,q.sy},target[2]={q.gx,q.gy};nav.setGoal(source);nav.setStart(target);
    nav.setupNavFn(true);
    bool exhausted=nav.propNavFnDijkstra(6400*6400,false);
    if(!exhausted||nav.curPe||nav.nextPe||nav.overPe)throw std::runtime_error("NavFn field incomplete");
    for(int i=0;i<6400;++i)b.nav[i]=nav.potarr[i];
  }
}
void verify(unsigned mode,const Case& q,const Buffers& b){
  for(int i=0;i<6400;++i){
    bool same=false;
    if(mode==0){uint64_t v=b.v3[i];if(v==UINT64_MAX/4)v=0xffffff;same=v==q.expected[i];}
    else if(mode==1)same=b.field[i]==q.expected[i];
    else same=(b.nav[i]<POT_HIGH)==(q.expected[i]!=0xffffff);
    if(!same)throw std::runtime_error("field/mask mismatch case="+std::to_string(q.index)+" cell="+std::to_string(i)+" mode="+std::to_string(mode));
  }
}
}
int main(int argc,char** argv){try{
  if(argc!=4||(std::string(argv[3])!="verify"&&std::string(argv[3])!="time"))return 2;
  auto cases=read(argv[1],argv[2]);Buffers b;bool timing=std::string(argv[3])=="time";
  const char* names[3]={"v3_full_field","B_full_field","NavFn_full_field"};
  for(const auto& q:cases)for(unsigned m=0;m<3;++m){run(m,q,b);verify(m,q,b);}
  if(!timing){std::cout<<"{\"verified_valid_fields\":"<<cases.size()<<",\"v3_and_B_integer_fields_equal\":true,\"NavFn_reachability_masks_equal\":true,\"NavFn_all_queues_exhausted\":true,\"timing_performed\":false}\n";return 0;}
  for(const auto& q:cases)for(unsigned w=0;w<3;++w)for(unsigned m=0;m<3;++m)run(m,q,b);
  for(unsigned r=0;r<20;++r)for(const auto& q:cases)for(unsigned j=0;j<3;++j){
    unsigned m=(r+j)%3;auto t0=std::chrono::steady_clock::now();run(m,q,b);auto t1=std::chrono::steady_clock::now();
    verify(m,q,b);auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count();
    std::cout<<"{\"case\":"<<q.index<<",\"repeat\":"<<r<<",\"algorithm\":\""<<names[m]<<"\",\"ns\":"<<ns<<"}\n";
  }
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
