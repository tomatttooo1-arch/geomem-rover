#include "roi_fx_ps.hpp"
#include ".memory/ref/object_evidence.hpp"
#include <vector>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <cstdlib>
extern "C" void ref_roi_batch(const float*,std::uint32_t,std::uint32_t,std::uint32_t,const std::uint32_t*,const std::uint32_t*,const double*,const double*,const std::uint32_t*,double,std::uint32_t,double,double,std::uint32_t*,std::uint32_t*);
static std::vector<float> fdepth(1024000);
static void golden(const std::vector<std::uint16_t>& raw,const roi_fx_ps::Input& a,float unit,std::uint32_t out[224],std::uint32_t* st){
 for(unsigned i=0;i<a.w*a.h;++i)fdepth[i]=raw[i]?float(raw[i])*unit:NAN;
 ref_roi_batch(fdepth.data(),a.w,a.h,a.n,a.en,a.xy,a.near_z,a.far_z,a.id,a.margin,a.minpix,a.mvf,a.ff,out,st);
}
static void save(const char* path,const std::vector<std::uint16_t>& raw,const roi_fx_ps::Input& a,float unit){
 auto c=roi_fx_ps::compile(a,unit);std::uint32_t out[224],st=99;
 std::memset(out,0xab,sizeof out);golden(raw,a,unit,out,&st);
 FILE* f=std::fopen(path,"wb");if(!f)std::abort();
 std::fwrite(&a,sizeof a,1,f);std::fwrite(&c,sizeof c,1,f);std::fwrite(&st,4,1,f);std::fwrite(out,4,224,f);
 std::vector<std::uint32_t> words((a.w*a.h+1)/2);
 for(unsigned i=0;i<a.w*a.h;++i)words[i/2]|=std::uint32_t(raw[i])<<((i&1)*16);
 std::fwrite(words.data(),4,words.size(),f);std::fclose(f);
}
int main(int argc,char** argv){
 if(argc!=3)return 2; // depth_u16.bin and output directory
 roi_fx_ps::Input a={};a.w=640;a.h=360;a.n=19;a.minpix=9;a.margin=.04;a.mvf=.7;a.ff=.9;
 // Frozen m5a projection from tools/roi_fixedpoint/fixtures/rec30_boxes.json.
 a.en[0]=1;a.id[0]=0;a.near_z[0]=0.42898132443675197;a.far_z[0]=0.4850187098955234;
a.xy[0]=178;a.xy[1]=157;a.xy[2]=235;a.xy[3]=203;
a.en[1]=1;a.id[1]=1;a.near_z[1]=0.49698737775367857;a.far_z[1]=0.5350126870961749;
a.xy[4]=256;a.xy[5]=166;a.xy[6]=287;a.xy[7]=194;
a.en[2]=1;a.id[2]=2;a.near_z[2]=0.5742236476364994;a.far_z[2]=0.6717764038619137;
a.xy[8]=307;a.xy[9]=4;a.xy[10]=367;a.xy[11]=80;
a.en[3]=1;a.id[3]=3;a.near_z[3]=0.4345745549374114;a.far_z[3]=0.47742550514407056;
a.xy[12]=319;a.xy[13]=162;a.xy[14]=354;a.xy[15]=197;
a.en[4]=1;a.id[4]=4;a.near_z[4]=0.425538728127372;a.far_z[4]=0.4544613267089012;
a.xy[16]=448;a.xy[17]=28;a.xy[18]=481;a.xy[19]=61;
a.en[5]=1;a.id[5]=5;a.near_z[5]=0.3931585093379635;a.far_z[5]=0.454841540253101;
a.xy[20]=431;a.xy[21]=104;a.xy[22]=503;a.xy[23]=162;
a.en[6]=1;a.id[6]=6;a.near_z[6]=0.4528935807646649;a.far_z[6]=0.5131064397393329;
a.xy[24]=375;a.xy[25]=109;a.xy[26]=429;a.xy[27]=158;
a.en[7]=1;a.id[7]=7;a.near_z[7]=0.9716621582432028;a.far_z[7]=1.0251378829555278;
a.xy[28]=326;a.xy[29]=78;a.xy[30]=346;a.xy[31]=102;
a.en[8]=1;a.id[8]=8;a.near_z[8]=0.2555286626615994;a.far_z[8]=0.3153713960847386;
a.xy[32]=225;a.xy[33]=39;a.xy[34]=310;a.xy[35]=132;
a.en[9]=1;a.id[9]=9;a.near_z[9]=0.7252690056547633;a.far_z[9]=0.7563310881867895;
a.xy[36]=391;a.xy[37]=172;a.xy[38]=410;a.xy[39]=188;
a.en[10]=1;a.id[10]=10;a.near_z[10]=0.2512372575775891;a.far_z[10]=0.2899627573712559;
a.xy[40]=371;a.xy[41]=56;a.xy[42]=433;a.xy[43]=120;
a.en[11]=1;a.id[11]=11;a.near_z[11]=0.6943133204919042;a.far_z[11]=0.7264867931860745;
a.xy[44]=197;a.xy[45]=79;a.xy[46]=219;a.xy[47]=100;
a.en[12]=1;a.id[12]=12;a.near_z[12]=0.69719621598231;a.far_z[12]=0.7300038152982078;
a.xy[48]=197;a.xy[49]=33;a.xy[50]=219;a.xy[51]=56;
a.en[13]=1;a.id[13]=13;a.near_z[13]=0.6778438779799608;a.far_z[13]=0.7045561578781935;
a.xy[52]=454;a.xy[53]=81;a.xy[54]=474;a.xy[55]=99;
a.en[14]=1;a.id[14]=14;a.near_z[14]=0.20115524061903445;a.far_z[14]=0.2564447688223413;
a.xy[56]=411;a.xy[57]=132;a.xy[58]=531;a.xy[59]=228;
a.en[15]=1;a.id[15]=15;a.near_z[15]=0.9290920896273688;a.far_z[15]=1.0261079149502679;
a.xy[60]=116;a.xy[61]=19;a.xy[62]=169;a.xy[63]=68;
a.en[16]=1;a.id[16]=16;a.near_z[16]=0.3000880712618032;a.far_z[16]=0.38191193398340556;
a.xy[64]=290;a.xy[65]=81;a.xy[66]=385;a.xy[67]=177;
a.en[17]=1;a.id[17]=17;a.near_z[17]=0.7942344414670508;a.far_z[17]=0.8601656210939845;
a.xy[68]=255;a.xy[69]=118;a.xy[70]=288;a.xy[71]=150;
a.en[18]=1;a.id[18]=18;a.near_z[18]=0.2241124392898692;a.far_z[18]=0.2686875759212362;
a.xy[72]=162;a.xy[73]=96;a.xy[74]=247;a.xy[75]=168;

 std::vector<std::uint16_t> raw(a.w*a.h);FILE* f=std::fopen(argv[1],"rb");if(!f)return 2;
 long bad=0;for(unsigned frame=0;frame<310;++frame){
  if(std::fread(raw.data(),2,raw.size(),f)!=raw.size())return 2;
  std::uint32_t ref[224],got[224],sr=99,sg=99;std::memset(ref,0xab,sizeof ref);std::memset(got,0xab,sizeof got);
  golden(raw,a,.001f,ref,&sr);auto c=roi_fx_ps::compile(a,.001f);roi_fx_ps::model(raw.data(),a,c,got,&sg);
  bad+=sr!=sg||std::memcmp(ref,got,sizeof ref)!=0;
  if(frame==0){char name[4096];std::snprintf(name,sizeof name,"%s/rec30.bin",argv[2]);save(name,raw,a,.001f);}
 }
 std::fclose(f);std::printf("rec30 frames=310 boxes=19 mismatches=%ld\n",bad);
 // Odd length, odd start index, all slots with different keys; disabled garbage ignored.
 a={};a.w=17;a.h=19;a.n=32;a.minpix=9;a.margin=.04;a.mvf=.7;a.ff=.9;
 raw.resize(a.w*a.h);for(unsigned i=0;i<raw.size();++i)raw[i]=i%13?std::uint16_t(1000+(i*173)%5000):0;
 for(unsigned b=0;b<32;++b){a.id[b]=0x123400+b;a.en[b]=b%11!=0;
  a.xy[4*b]=b%4;a.xy[4*b+1]=1+b%4;a.xy[4*b+2]=12+b%5;a.xy[4*b+3]=14+b%5;
  a.near_z[b]=1.101+.113*b;a.far_z[b]=a.near_z[b]+.001*(b+1);}
 char name[4096];std::snprintf(name,sizeof name,"%s/odd.bin",argv[2]);save(name,raw,a,.001f);
 return bad?1:0;
}
