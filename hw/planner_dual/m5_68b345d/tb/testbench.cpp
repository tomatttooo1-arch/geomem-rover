#include "kernel_hls.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
static unsigned get(std::istream& f,int n=4){
 unsigned v=0;for(int j=0;j<n;++j){int b=f.get();if(b<0)throw std::runtime_error("truncated");v|=unsigned(b)<<(8*j);}return v;
}
static void put(std::ostream& f,unsigned v){for(int j=0;j<4;++j)f.put(char((v>>(8*j))&255));}
int main(int argc,char** argv){
 try{
  if(argc!=3)throw std::runtime_error("input.bin expected.bin required");
  std::ifstream input(argv[1],std::ios::binary),expected(argv[2],std::ios::binary);
  char im[8],gm[8];input.read(im,8);expected.read(gm,8);
  if(std::string(im,8)!="FPGADIN01" || std::string(gm,8)!="FPGADGO01")return 3;
  unsigned count=get(input);if(count!=2 || get(expected)!=count)return 4;
  mkdir("tb_out",0700);std::ofstream output("tb_out/outputs.bin",std::ios::binary);
  static ap_uint<8> map[40000];static ap_uint<32> field[40000],flags[256];
  ap_uint<32> status,rounds,valid;
  for(unsigned c=0;c<count;++c){
   unsigned mode=get(input),n=mode==0?6400:40000,nflags=mode==0?64:256;
   if(mode!=c)return 6;
   int sx=int(get(input)),sy=int(get(input)),gx=int(get(input)),gy=int(get(input));unsigned k=get(input);
   for(unsigned i=0;i<n;++i)map[i]=get(input,1);
   planner_field_dual(map,field,flags,sx,sy,gx,gy,k,status,rounds,valid,mode);
   unsigned errors=0,scalars[3]={status.to_uint(),rounds.to_uint(),valid.to_uint()};
   for(auto v:scalars){errors+=get(expected)!=v;put(output,v);}
   for(unsigned i=0;i<n;++i){unsigned v=field[i].to_uint();errors+=get(expected)!=v;put(output,v);}
   for(unsigned i=0;i<nflags;++i){unsigned v=flags[i].to_uint();errors+=get(expected)!=v;put(output,v);}
   std::cout<<"case="<<c<<" mode="<<mode<<" status="<<scalars[0]<<" rounds="<<scalars[1]<<" valid="<<scalars[2]<<" mismatches="<<errors<<"\n";
   if(errors)return 1;
  }
  if(input.get()!=EOF || expected.get()!=EOF || !output)return 5;
  std::cout<<"PASS 2 frozen cases, all uint32 fields/flags/scalars, 80 to 200\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}
}
