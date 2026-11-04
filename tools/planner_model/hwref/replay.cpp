// Standalone vector consumer: only the deliverable header and C++ standard lib.
#include "../planner_hw_ref.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <cstdio>
namespace {
void require(bool ok,const std::string& why){if(!ok)throw std::runtime_error(why);}
std::vector<unsigned char> read(const std::string& file){
  std::ifstream f(file,std::ios::binary);require(bool(f),"missing "+file);
  return std::vector<unsigned char>(std::istreambuf_iterator<char>(f),{});
}
}
int main(int argc,char** argv){
  try{
    require(argc==2,"usage: planner_hwref_replay VECTOR_DIRECTORY");
    for(int number=0;number<200;++number){
      char suffix[16];std::snprintf(suffix,sizeof(suffix),"/%03d/",number);
      const std::string root=std::string(argv[1])+suffix;
      auto m=read(root+"map.u8");require(m.size()==6400,"map length");
      const auto original=m;
      std::ifstream request(root+"request.txt"),expected(root+"expected.txt");
      int sx,sy,state,n,valid;unsigned limit,rounds;std::string gx,gy;
      require(bool(request>>sx>>sy>>gx>>gy>>limit),"bad request");
      require(bool(expected>>state>>n>>valid>>rounds),"bad expected result");
      std::uint8_t scratch[6400];std::uint32_t field[6400];
      double x[6402],y[6402];std::vector<std::uint8_t> flags(limit ? limit : 6400,0);
      auto r=planner_hw_ref::plan(m.data(),80,80,{sx,sy,std::stod(gx),std::stod(gy)},
        scratch,field,x,y,6402,limit,flags.data());
      require(r.status==state && r.count==n && int(r.field_valid)==valid && r.rounds==rounds,"state mismatch at "+root);
      require(m==original,"map was modified");
      const auto potential=read(root+"field.u24le");
      require(potential.size()==(valid ? 19200u : 0u),"field presence");
      if(valid)for(int i=0;i<6400;++i){
        auto v=std::uint32_t(potential[3*i]) | (std::uint32_t(potential[3*i+1])<<8) | (std::uint32_t(potential[3*i+2])<<16);
        require(v==field[i],"field mismatch at "+root);
      }
      const auto path=read(root+"path.f64le");
      const std::uint16_t byte_order=1;
      require(*reinterpret_cast<const unsigned char*>(&byte_order)==1 && sizeof(double)==8,"vectors require little-endian IEEE binary64 host");
      require(path.size()==unsigned(n)*16,"path length");
      for(int i=0;i<n;++i){
        require(std::memcmp(path.data()+i*16,&x[i],8)==0 && std::memcmp(path.data()+i*16+8,&y[i],8)==0,"path bits mismatch at "+root);
      }
      const auto changed=read(root+"changed.u8");
      require(changed.size()==r.rounds,"flags length");
      for(unsigned i=0;i<r.rounds;++i)require(changed[i]==flags[i],"flags mismatch");
    }
    std::cout<<"200 vectors: state, full valid field, path bits, flags, immutable input PASS\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
