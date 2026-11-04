#include "roi_fx_ps.hpp"
#include <cmath>
#include <cstring>
#include <cfloat>
namespace roi_fx_ps {
static std::uint64_t midpoint(double f) {
 std::uint64_t bits; std::memcpy(&bits,&f,8);
 const std::uint64_t m=bits&((1ULL<<52)-1);
 return f==1.0 ? (1ULL<<55)-2 : f==0.5 ? (1ULL<<54)-1 : (1ULL<<54)+4*m-2;
}
static bool tie(double f) { std::uint64_t b;std::memcpy(&b,&f,8);return !(b&1); }
static float z(std::uint32_t c,float u) {volatile float q=float(c)*u;return q;}
static std::uint32_t lb(float target,float unit,bool strict) {
 std::uint32_t lo=1,hi=65536;
 while(lo<hi){const auto mid=(lo+hi)>>1;const float q=z(mid,unit);
  if(strict ? q>target : q>=target)hi=mid;else lo=mid+1;}
 return lo;
}
Compiled compile(const Input& a,float unit) {
 Compiled c={};
 if(!a.w||a.w>1280||!a.h||a.h>800){c.status=1;return c;}
 if(a.minpix<9||a.minpix>1024000||!std::isfinite(a.mvf)||a.mvf<.7||a.mvf>1||
 !std::isfinite(a.ff)||a.ff<.9||a.ff>1||!std::isfinite(a.margin)||a.margin<.04){c.status=4;return c;}
 if(!a.n||a.n>32){c.status=3;return c;}
 float front[32]={},back[32]={};
 for(unsigned b=0;b<a.n;++b){
  if(a.en[b]>1){c.status=3;return c;} if(!a.en[b])continue;
  const auto* p=a.xy+4*b;
  if(p[0]>p[2]||p[1]>p[3]||p[2]>=a.w||p[3]>=a.h||p[2]-p[0]<3||p[3]-p[1]<3||
   !std::isfinite(a.near_z[b])||!std::isfinite(a.far_z[b])||a.near_z[b]<=.1||a.far_z[b]<a.near_z[b]){c.status=3;return c;}
  volatile double f=a.near_z[b]-a.margin, k=a.far_z[b]+a.margin;
  front[b]=float(f);back[b]=float(k);
  if(!std::isfinite(front[b])||!std::isfinite(back[b])||front[b]>back[b]){c.status=3;return c;}
 }
 if(!(unit>0)||!std::isfinite(unit)){c.status=5;return c;}
 c.vbegin=lb(.1f,unit,true);c.vend=lb(10.f,unit,false);
 c.valid_m=midpoint(a.mvf);c.free_m=midpoint(a.ff);c.ties=tie(a.mvf)|(tie(a.ff)<<1);
 for(unsigned b=0;b<a.n;++b)if(a.en[b]){c.front[b]=lb(front[b],unit,false);c.back[b]=lb(back[b],unit,true);}
 return c;
}
bool ratio_ge(std::uint32_t a,std::uint32_t b,std::uint64_t m,bool tie) {
 const std::uint64_t mh=m>>28,ml=m&((1ULL<<28)-1);
 std::uint64_t p=0,q=0;
 for(int i=19;i>=0;--i){bool bit=(b>>i)&1;p=(p<<1)+(bit?mh:0);q=(q<<1)+(bit?ml:0);}
 const auto h=p+(q>>28),lhs=std::uint64_t(a)<<27;
 return lhs>h||(lhs==h&&!(q&((1ULL<<28)-1))&&tie);
}
void model(const std::uint16_t* depth,const Input& a,const Compiled& c,std::uint32_t out[224],std::uint32_t* status){
 *status=c.status;if(c.status)return;
 // Native integer mirror of the approved ordinal predicates, no AP simulation.
 // Counts use 8x4 interleaving but per-box ROI scan avoids full-width masked work on CPU.
 for(unsigned b=0;b<a.n;++b){auto* r=out+7*b;r[0]=a.id[b];for(unsigned j=1;j<7;++j)r[j]=0;
  if(!a.en[b])continue;r[1]=1;const auto* p=a.xy+4*b;r[2]=(p[2]-p[0]+1)*(p[3]-p[1]+1);
  for(unsigned y=p[1];y<=p[3];++y)for(unsigned x=p[0];x<=p[2];++x){
   const auto code=depth[y*a.w+x];if(code<c.vbegin||code>=c.vend)continue;
   ++r[3];r[4]+=code<c.front[b];r[5]+=code>=c.back[b];}
  if(r[3]>=a.minpix&&ratio_ge(r[3],r[2],c.valid_m,c.ties&1)){
   if(ratio_ge(r[4],r[3],(1ULL<<54)-1,true))r[6]=3;
   else if(ratio_ge(r[5],r[3],c.free_m,c.ties&2))r[6]=2;}
 }
}
}
