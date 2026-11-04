// The same two frozen frames (slots0/33) used for necessary C/RTL check.
#include "roi_obs_ps.hpp"
#include <ap_int.h>
#include <fstream>
#include <vector>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sys/stat.h>
extern "C" void fpga_m7_roi_obs_phase(ap_uint<32>*,const ap_uint<32>[914],ap_uint<5>,ap_uint<2>,ap_uint<64>,ap_uint<64>,ap_uint<64>,ap_uint<32>&);
namespace {
template<class T>T rd(std::ifstream& f){T x{};if(!f.read((char*)&x,sizeof x))throw std::runtime_error("short input");return x;}
void bytes(std::ifstream& f,void* p,size_t n){if(!f.read((char*)p,n))throw std::runtime_error("short bytes");}
void u32(std::ofstream& f,std::uint32_t x){f.write((char*)&x,4);}
}
int main(int argc,char**argv){try{
 if(argc!=3)return 2;std::ifstream f(argv[1],std::ios::binary),gold(argv[2],std::ios::binary);
 char magic[8];bytes(f,magic,8);if(std::memcmp(magic,"OBSV0001",8))return 2;
 unsigned nf=rd<std::uint32_t>(f),w=rd<std::uint32_t>(f),h=rd<std::uint32_t>(f);(void)rd<std::uint32_t>(f);
 mkdir("tb_out",0777);std::ofstream records("tb_out/records.bin",std::ios::binary),statuses("tb_out/status.bin",std::ios::binary);
 unsigned boxes_checked=0,bad=0;
 for(unsigned slot=0;slot<nf;++slot){
  auto source=rd<std::uint32_t>(f);(void)rd<std::uint32_t>(f);
  double K[9],T[16];bytes(f,K,sizeof K);bytes(f,T,sizeof T);unsigned count=rd<std::uint32_t>(f);
  std::vector<roi_obs_ps::Box> boxes(count);
  for(unsigned b=0;b<count;++b){double xy[4];bytes(f,xy,sizeof xy);boxes[b]={xy[0],xy[1],xy[2],xy[3],b};(void)rd<float>(f);(void)rd<std::int32_t>(f);}
  if(slot!=0&&slot!=33){f.seekg(size_t(w)*h*5,std::ios::cur);continue;}
  std::vector<std::uint16_t> depth(w*h);std::vector<std::uint8_t> rgb(w*h*3);
  bytes(f,depth.data(),depth.size()*2);bytes(f,rgb.data(),rgb.size());
  roi_obs_ps::Input in{};in.width=w;in.height=h;in.count=count;in.boxes=boxes.data();in.frame_id=source;
  std::memcpy(in.K,K,sizeof K);std::memcpy(in.T,T,sizeof T);auto compiled=roi_obs_ps::compile(in);if(compiled.status)throw std::runtime_error(compiled.reason);
  ap_uint<32> params[914];for(unsigned i=0;i<914;++i)params[i]=compiled.words[i];
  static ap_uint<32> memory[1048576];
  const unsigned depth_base=64,rgb_base=depth_base+w*h*2+64,output_base=(rgb_base+w*h*3+127)&~63U;
  std::vector<std::uint32_t> before((output_base>>2)+32,0);
  for(unsigned i=0;i<depth.size();++i){unsigned at=(depth_base+2*i)>>2;before[at]|=std::uint32_t(depth[i])<<(16*(i&1));}
  for(unsigned i=0;i<rgb.size();++i){unsigned at=(rgb_base+i)>>2;before[at]|=std::uint32_t(rgb[i])<<(8*((rgb_base+i)&3));}
  for(unsigned i=0;i<before.size();++i)memory[i]=before[i];
  for(unsigned b=0;b<count;++b){
   for(unsigned j=0;j<32;++j)memory[(output_base>>2)+j]=0xdeadbeefU;
   bool complete=false;
   for(unsigned phase=0;phase<4;++phase){
    ap_uint<32> status=0;
    fpga_m7_roi_obs_phase(memory,params,b,phase,depth_base,rgb_base,output_base,status);
    u32(statuses,status.to_uint());bad+=status!=0;
    if(memory[output_base>>2]==b){complete=true;break;}
   }
   if(!complete)throw std::runtime_error("no result record");
   for(unsigned j=0;j<32;++j){auto value=memory[(output_base>>2)+j].to_uint(),expected=rd<std::uint32_t>(gold);u32(records,value);if(value!=expected){++bad;std::cerr<<"record difference slot="<<slot<<" box="<<b<<" word="<<j<<" got="<<value<<" expected="<<expected<<'\n';}}
   for(unsigned j=0;j<(output_base>>2);++j)if(memory[j]!=before[j])throw std::runtime_error("unexpected DDR input write");
   for(unsigned j=0;j<914;++j)if(params[j]!=compiled.words[j])throw std::runtime_error("parameter modified");
   ++boxes_checked;
  }
 }
 std::cout<<"ROI phase representative boxes="<<boxes_checked<<" bad="<<bad<<'\n';return bad||boxes_checked!=8?1:0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
