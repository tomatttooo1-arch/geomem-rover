#include "roi_fx.hpp"
#include "roi_fx_ps.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <sys/stat.h>
static std::uint32_t run(const roi_fx_ps::Input& a,const roi_fx_ps::Compiled& c,const ap_uint<32>* depth,std::uint32_t out[224]){
 return fpga_m7_object_roi_fx_axi(depth,a.w,a.h,a.n,a.en,a.xy,a.id,a.minpix,c.front,c.back,roi_fx::COOKIE,c.vbegin,c.vend,c.status,c.valid_m,c.free_m,c.ties,out);
}
int main(int argc,char** argv){
 if(argc!=2)return 2;mkdir("tb_out",0777);FILE* dump=std::fopen("tb_out/all.bin","wb");if(!dump)return 2;
 std::uint32_t out[224];std::memset(out,0xab,sizeof out);unsigned bad=0;
 const char* names[]={"rec30.bin","odd.bin"};
 for(unsigned k=0;k<2;++k){char path[4096];std::snprintf(path,sizeof path,"%s/%s",argv[1],names[k]);FILE* f=std::fopen(path,"rb");if(!f)return 2;
  roi_fx_ps::Input a;roi_fx_ps::Compiled c;std::uint32_t golden[224],status;
  if(std::fread(&a,sizeof a,1,f)!=1||std::fread(&c,sizeof c,1,f)!=1||std::fread(&status,4,1,f)!=1||std::fread(golden,4,224,f)!=224)return 2;
  const unsigned words=(a.w*a.h+1)/2;std::vector<ap_uint<32>> depth(512000);
  for(unsigned i=0;i<words;++i){std::uint32_t q;if(std::fread(&q,4,1,f)!=1)return 2;depth[i]=q;}std::fclose(f);
  const auto st=run(a,c,depth.data(),out);
  if(st!=status){std::printf("status %u expected %u\n",st,status);++bad;}
  // First call unused rows may retain C sentinel or RTL reset. Prefix every legal row in second call.
  for(unsigned i=0;i<a.n*7;++i)if(out[i]!=golden[i]){if(bad<8)std::printf("%s word%u got%08x expected%08x\n",names[k],i,out[i],golden[i]);++bad;}
  std::fwrite(&st,4,1,dump);std::fwrite(out,4,a.n*7,dump);
  if(k==1){std::uint32_t before[224];std::memcpy(before,out,sizeof before);a.xy[0]=999; // lane0 disabled, corrupt enabled lane1
   a.xy[4]=999;const auto err=run(a,c,depth.data(),out);bad+=err!=3||std::memcmp(before,out,sizeof before)!=0;
   std::fwrite(&err,4,1,dump);std::fwrite(out,4,224,dump);}
 }
 std::fclose(dump);std::printf("two representative vectors + retained-result error: %s mismatches=%u\n",bad?"FAIL":"PASS",bad);return bad?1:0;
}
