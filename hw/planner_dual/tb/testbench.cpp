//   testbench input.bin expected.bin [path_params.bin]
// (sha256 0036fda9...). With it: path_ctrl=1; field/flags/scalars must still equal the frozen golden and
// header + path words must equal the integer C model (../path/path_model.hpp) run on the golden field.
// In both modes flags words after the written block must keep their sentinel (no stray DDR writes).
#include "kernel_hls.hpp"
#include "../path/path_model.hpp"
#include "gmap_tb.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/stat.h>
static unsigned get(std::istream& f,int n=4){
 unsigned v=0;for(int j=0;j<n;++j){int b=f.get();if(b<0)throw std::runtime_error("truncated");v|=unsigned(b)<<(8*j);}return v;
}
static void put(std::ostream& f,unsigned v){for(int j=0;j<4;++j)f.put(char((v>>(8*j))&255));}
int gmap_main(const char* vec);
int main(int argc,char** argv){
 if(argc==3 && std::string(argv[1])=="--gmap")return gmap_main(argv[2]);
 try{
  if(argc!=3 && argc!=4)throw std::runtime_error("input.bin expected.bin [path_params.bin] required");
  std::ifstream input(argv[1],std::ios::binary),expected(argv[2],std::ios::binary);
  const bool path=argc==4;
  std::vector<unsigned char> params;
  if(path){std::ifstream p(argv[3],std::ios::binary);params.assign(std::istreambuf_iterator<char>(p),{});
   if(params.size()!=2*plnpath::kParamBytes)throw std::runtime_error("path_params.bin must hold 2 x 132 bytes");}
  char im[8],gm[8];input.read(im,8);expected.read(gm,8);
  if(std::string(im,8)!="FPGADIN01" || std::string(gm,8)!="FPGADGO01")return 3;
  unsigned count=get(input);if(count!=2 || get(expected)!=count)return 4;
  mkdir("tb_out",0700);std::ofstream output("tb_out/outputs.bin",std::ios::binary);
  static ap_uint<8> map[kGmapAreaBytes];static ap_uint<32> field[kGmapAreaWords],flags[kPlnFlagsDepth];
  static std::uint8_t raw[40000];static std::uint32_t gfield[40000];
  ap_uint<32> status,rounds,valid;
  for(unsigned c=0;c<count;++c){
   unsigned mode=get(input),n=mode==0?6400:40000,nflags=mode==0?64:256;
   if(mode!=c)return 6;
   int sx=int(get(input)),sy=int(get(input)),gx=int(get(input)),gy=int(get(input));unsigned k=get(input);
   for(unsigned i=0;i<n;++i){raw[i]=get(input,1);map[i]=raw[i];}
   for(unsigned i=n;i<unsigned(kGmapAreaBytes);++i)map[i]=0;
   if(path)for(int i=0;i<plnpath::kParamBytes;++i)map[n+i]=params[c*plnpath::kParamBytes+i];
   for(int i=0;i<kPlnFlagsDepth;++i)flags[i]=0xDEADBEEFu;
   planner_field_dual(map,field,flags,sx,sy,gx,gy,k,status,rounds,valid,mode,path?1u:0u,0u);
   unsigned errors=0,scalars[3]={status.to_uint(),rounds.to_uint(),valid.to_uint()};
   unsigned exp_scalars[3];
   for(int s=0;s<3;++s){exp_scalars[s]=get(expected);errors+=exp_scalars[s]!=scalars[s];put(output,scalars[s]);}
   for(unsigned i=0;i<n;++i){unsigned v=field[i].to_uint();gfield[i]=get(expected);errors+=gfield[i]!=v;put(output,v);}
   for(unsigned i=0;i<nflags;++i){unsigned v=flags[i].to_uint();errors+=get(expected)!=v;put(output,v);}
   unsigned written=nflags,path_errors=0,pn=0;
   if(path){
    const plnpath::ModelOut mo=plnpath::model(raw,gfield,mode==0?80:200,mode==0?0xffffffu:0x1fffffffu,sx,sy,gx,gy,
        &params[c*plnpath::kParamBytes],exp_scalars[0]==0 && exp_scalars[2]==1);
    pn=unsigned(mo.words.size());
    for(unsigned i=0;i<pn;++i){unsigned v=flags[nflags+i].to_uint();path_errors+=v!=mo.words[i];put(output,v);}
    written+=pn;
   }
   unsigned stray=0;
   for(int i=int(written);i<kPlnFlagsDepth;++i)stray+=flags[i].to_uint()!=0xDEADBEEFu;
   std::cout<<"case="<<c<<" mode="<<mode<<" status="<<scalars[0]<<" rounds="<<scalars[1]<<" valid="<<scalars[2]
            <<" legacy_mismatches="<<errors;
   if(path)std::cout<<" header=0x"<<std::hex<<flags[nflags].to_uint()<<std::dec<<" path_words="<<pn<<" path_mismatches="<<path_errors;
   std::cout<<" stray_writes="<<stray<<"\n";
   if(errors || path_errors || stray)return 1;
  }
  if(input.get()!=EOF || expected.get()!=EOF || !output)return 5;
  std::cout<<"PASS 2 frozen cases"<<(path?" with path extraction (header+points == C model)":" legacy (path_ctrl=0)")<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}
}
