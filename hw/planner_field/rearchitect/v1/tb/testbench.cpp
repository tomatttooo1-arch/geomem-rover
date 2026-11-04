#include "kernel_hls.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <sys/stat.h>
static unsigned get(std::istream& f,int n=4){unsigned v=0;for(int j=0;j<n;++j){int b=f.get();if(b<0)throw std::runtime_error("truncated");v|=unsigned(b)<<(8*j);}return v;}
static void put(std::ostream& f,unsigned v,int n){for(int j=0;j<n;++j)f.put(char((v>>(8*j))&255));}
int main(int argc,char** argv){
 if(argc!=3){std::cerr<<"input.bin expected.bin required\n";return 2;}
 std::ifstream input(argv[1],std::ios::binary),expected(argv[2],std::ios::binary);
 char im[8],gm[8];input.read(im,8);expected.read(gm,8);
 if(std::string(im,8)!="FPGAFIN01" || std::string(gm,8)!="FPGAFGO01")return 3;
 unsigned n=get(input);if(get(expected)!=n)return 4;
 mkdir("tb_out",0700);std::ofstream output("tb_out/outputs.bin",std::ios::binary);
 ap_uint<8> map[6400];ap_uint<32> field[6400],flags[64],status,rounds,valid;
 unsigned errors=0;
 for(unsigned c=0;c<n;++c){
  int sx=int(get(input)),sy=int(get(input)),gx=int(get(input)),gy=int(get(input));unsigned k=get(input);
  for(int i=0;i<6400;++i)map[i]=get(input,1);
  planner_field(map,field,flags,sx,sy,gx,gy,k,status,rounds,valid);
  unsigned values[3]={status.to_uint(),rounds.to_uint(),valid.to_uint()};
  for(auto v:values){errors+=get(expected)!=v;put(output,v,4);}
  for(int i=0;i<6400;++i){unsigned v=field[i].to_uint();errors+=get(expected,3)!=v;put(output,v,3);}
  for(int i=0;i<64;++i){unsigned v=flags[i].to_uint();errors+=get(expected,1)!=v;put(output,v,1);}
  if(errors){std::cerr<<"FAIL case "<<c<<" output mismatches "<<errors<<"\n";return 1;}
 }
 if(input.get()!=EOF || expected.get()!=EOF || !output)return 5;
 std::cout<<"PASS "<<n<<" complete fields/flags/status/rounds, repeated calls\n";return 0;
}
