#include "../planner_hw_ref.hpp"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
static unsigned get(std::istream& f){unsigned v=0;for(int j=0;j<4;++j){int b=f.get();if(b<0)throw std::runtime_error("truncated");v|=unsigned(b)<<(8*j);}return v;}
static void put(std::ostream& f,unsigned v,int n){for(int j=0;j<n;++j)f.put(char((v>>(8*j))&255));}
int main(int argc,char** argv){
  if(argc!=3)return 2;
  std::ifstream in(argv[1],std::ios::binary);std::ofstream out(argv[2],std::ios::binary);
  char magic[8];in.read(magic,8);if(std::string(magic,8)!="FPGAFIN01")return 3;
  unsigned count=get(in);out.write("FPGAFGO01",8);put(out,count,4);
  std::uint8_t map[6400],scratch[6400],flags[64];std::uint32_t field[6400];
  for(unsigned n=0;n<count;++n){
    int sx=int(std::int32_t(get(in))),sy=int(std::int32_t(get(in))),gx=int(std::int32_t(get(in))),gy=int(std::int32_t(get(in)));unsigned k=get(in);
    in.read(reinterpret_cast<char*>(map),6400);if(!in)return 4;
    auto r=planner_hw_ref::build_field(map,sx,sy,gx,gy,scratch,field,flags,k);
    put(out,r.status,4);put(out,r.rounds,4);put(out,unsigned(r.field_valid),4);
    for(auto v:field)put(out,v,3);
    for(auto v:flags)put(out,v,1);
  }
  if(in.get()!=EOF || !out)return 5;
  std::cout<<"golden full outputs "<<count<<" cases\n";
}
