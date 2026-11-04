// One fixed 236-box quality gate, frozen obsref C numerical golden.
// Usage: obs_quality VEC.bin OUTPUT_DIR ; outputs are deterministic.
#include "obs_types.hpp"
#include "roi_obs_ps.hpp"
#include "obs_ref.h"
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <cstdlib>
#include <chrono>
namespace {
template<class T>T rd(std::ifstream& f){T x{};if(!f.read((char*)&x,sizeof x))throw std::runtime_error("short vector");return x;}
void read_bytes(std::ifstream& f,void* p,std::size_t n){if(!f.read((char*)p,n))throw std::runtime_error("short bytes");}
std::int64_t i64(const std::array<std::uint32_t,914>&w,unsigned i){return std::int64_t(std::uint64_t(w[i])|(std::uint64_t(w[i+1])<<32));}
void unpack(const roi_obs_ps::Compiled& p,unsigned b,roi_obs::Descriptor& d,roi_obs::Geometry& g,roi_obs::Profile& cfg){
 const auto&w=p.words;unsigned base=64+8*b;d.id=w[base];d.x0=std::int32_t(w[base+1]);d.y0=std::int32_t(w[base+2]);d.x1=std::int32_t(w[base+3]);d.y1=std::int32_t(w[base+4]);d.area=w[base+7];d.flags=w[base+6];
 d.extent_x=i64(w,320+4*b);d.extent_y=i64(w,322+4*b);
 for(unsigned a=0;a<9;++a)g.a[a]=i64(w,16+2*a);for(unsigned a=0;a<3;++a)g.rz[a]=i64(w,34+2*a);
 cfg.min_pixels=w[6];cfg.valid_m=i64(w,7);cfg.valid_tie=w[9];for(unsigned a=0;a<201;++a){cfg.lower[a]=w[512+2*a];cfg.upper[a]=w[513+2*a];}
}
}
int main(int argc,char**argv){try{
 if(argc!=3)return 2;std::ifstream f(argv[1],std::ios::binary);char magic[8];read_bytes(f,magic,8);if(std::memcmp(magic,"OBSV0001",8))return 2;
 unsigned nf=rd<std::uint32_t>(f),w=rd<std::uint32_t>(f),h=rd<std::uint32_t>(f),total=rd<std::uint32_t>(f);
 std::string dir=argv[2];std::ofstream candidate(dir+"/candidate.bin",std::ios::binary),representative(dir+"/representative.bin",std::ios::binary);
 std::ofstream diag(dir+"/quality.csv");diag<<"slot,box,status,cstatus,valid,cvalid,selected,cselected,color,ccolor,truncated,ctruncated,position_error_mm,bounds_error_mm\n";
 unsigned compared=0,bad=0,existsbad=0,colorbad=0,truncbad=0,diagbad=0;double maxpos=0,maxbound=0;
 obsref::Workspace ws;
 for(unsigned slot=0;slot<nf;++slot){
  unsigned source=rd<std::uint32_t>(f),pose=rd<std::uint32_t>(f);(void)pose;
  obsref::Frame frame{};frame.width=w;frame.height=h;read_bytes(f,frame.K,sizeof frame.K);read_bytes(f,frame.T,sizeof frame.T);
  unsigned count=rd<std::uint32_t>(f);std::vector<roi_obs_ps::Box> boxes(count);std::vector<obsref::Box> rb(count);
  for(unsigned b=0;b<count;++b){read_bytes(f,&rb[b],sizeof rb[b]);(void)rd<float>(f);(void)rd<std::int32_t>(f);boxes[b]={rb[b].x1,rb[b].y1,rb[b].x2,rb[b].y2,b};}
  std::vector<std::uint16_t> depth(w*h);std::vector<std::uint8_t> rgb(w*h*3);std::vector<float> z(w*h);
  read_bytes(f,depth.data(),depth.size()*2);read_bytes(f,rgb.data(),rgb.size());obsref::depth_from_mm(depth.data(),depth.size(),z.data());frame.depth_m=z.data();frame.rgb=rgb.data();
  roi_obs_ps::Input in{};in.width=w;in.height=h;in.count=count;in.boxes=boxes.data();in.frame_id=source;std::memcpy(in.K,frame.K,sizeof in.K);std::memcpy(in.T,frame.T,sizeof in.T);
  auto p=roi_obs_ps::compile(in);if(p.status)throw std::runtime_error(p.reason);
  for(unsigned b=0;b<count;++b){
   roi_obs::Descriptor d;roi_obs::Geometry g;roi_obs::Profile cfg;unpack(p,b,d,g,cfg);
   roi_obs::BoxState s{};hls::stream<ap_uint<64>> input;hls::stream<ap_uint<32>> output;
   for(unsigned phase=0;phase<4;++phase){
    for(int y=d.y0;y<d.y1;++y)for(int x=d.x0;x<d.x1;++x){unsigned i=y*w+x;ap_uint<64> pix=depth[i];pix.range(23,16)=rgb[3*i];pix.range(31,24)=rgb[3*i+1];pix.range(39,32)=rgb[3*i+2];input.write(pix);}
    roi_obs::obs_compute(phase,d,g,cfg,input,s,output);
    if(!input.empty())throw std::runtime_error("unconsumed pixels");
    if(s.status||s.error||phase==3)break;
   }
   std::uint32_t record[32];for(auto&x:record)x=output.read();if(!output.empty())throw std::runtime_error("extra record");
   candidate.write((char*)record,sizeof record);if(slot==0||slot==33)representative.write((char*)record,sizeof record);
   obsref::Result gold{};obsref::Config conf;obsref::process_box(frame,rb[b],gold,conf,ws);
   bool exists=record[1]==0,cexists=gold.status==0;
   bool eb=exists!=cexists,cb=exists&&cexists&&int(record[3])!=gold.color,tb=exists&&cexists&&int(record[2])!=gold.truncated;
   double pe=0,be=0;
   if(exists&&cexists){for(unsigned a=0;a<3;++a){double v=p.anchor[a]+std::int32_t(record[8+a])*1e-6-gold.position[a];pe+=v*v;
    be=std::max(be,std::fabs(p.anchor[a]+std::int32_t(record[11+a])*1e-6-gold.lower[a])*1000);
    be=std::max(be,std::fabs(p.anchor[a]+std::int32_t(record[14+a])*1e-6-gold.upper[a])*1000);}
    pe=std::sqrt(pe)*1000;maxpos=std::max(maxpos,pe);maxbound=std::max(maxbound,be);}
   bool db=int(record[1])!=gold.status||int(record[5])!=gold.n_valid||int(record[6])!=gold.n_mask;
   bool fail=eb||cb||tb||pe>5||s.error;bad+=fail;existsbad+=eb;colorbad+=cb;truncbad+=tb;diagbad+=db;++compared;
   diag<<slot<<','<<b<<','<<record[1]<<','<<gold.status<<','<<record[5]<<','<<gold.n_valid<<','<<record[6]<<','<<gold.n_mask<<','<<std::int32_t(record[3])<<','<<gold.color<<','<<record[2]<<','<<gold.truncated<<','<<pe<<','<<be<<'\n';
  }
 }
 std::cout<<"{\"scope\":\"AP-integer phase compute versus frozen C, no AXI/RTL/OOC\",\"expected_boxes\":"<<total<<",\"compared\":"<<compared<<",\"bad\":"<<bad<<",\"existence_mismatches\":"<<existsbad<<",\"color_mismatches\":"<<colorbad<<",\"truncated_mismatches\":"<<truncbad<<",\"status_count_differences\":"<<diagbad<<",\"max_position_error_mm\":"<<maxpos<<",\"max_bounds_error_mm\":"<<maxbound<<"}\n";
 return bad||compared!=total?1:0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
