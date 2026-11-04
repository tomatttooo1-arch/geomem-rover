#include "roi_obs_ps.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cfenv>
#include <limits>
namespace roi_obs_ps {
namespace {
std::uint64_t midpoint(double f) {
 std::uint64_t b;std::memcpy(&b,&f,8);auto m=b&((1ULL<<52)-1);
 return f==1?(1ULL<<55)-2:f==.5?(1ULL<<54)-1:(1ULL<<54)+4*m-2;
}
bool even(double f){std::uint64_t b;std::memcpy(&b,&f,8);return !(b&1);}
bool coefficient(double x,std::int64_t& q){
 if(!std::isfinite(x)||x< -128||x>=128)return false;
 double y=std::nearbyint(std::ldexp(x,32));
 if(y< -549755813888.||y>=549755813888.)return false;
 q=static_cast<std::int64_t>(y);return true;
}
float depth(unsigned c){volatile float z=float(c)/1000.0f;return z;}
unsigned cut(double mode,bool upper){
 unsigned l=1,r=65536;
 while(l<r){unsigned m=(l+r)/2;volatile double d=double(depth(m))-mode;
  bool yes=upper?d>=.08:d>-.08;if(yes)r=m;else l=m+1;}
 return l;
}
void put64(std::array<std::uint32_t,914>&w,unsigned i,std::uint64_t v){w[i]=v;w[i+1]=v>>32;}
}
Compiled compile(const Input& a){
 Compiled c;auto& w=c.words;
 auto fail=[&](unsigned s,const char* why){c.status=s;c.reason=why;return c;};
 if(std::fegetround()!=FE_TONEAREST)return fail(2,"requires FE_TONEAREST");
 if(!a.width||a.width>1280||!a.height||a.height>800)return fail(1,"image dimensions");
 if(a.count>32||(a.count&&!a.boxes)||a.min_pixels<9||a.min_pixels>1024000||
 !std::isfinite(a.valid_fraction)||a.valid_fraction<.7||a.valid_fraction>1)return fail(2,"observation profile");
 for(double v:a.K)if(!std::isfinite(v))return fail(2,"nonfinite K");
 for(double v:a.T)if(!std::isfinite(v))return fail(2,"nonfinite T");
 const double *k=a.K;
 double A=k[4]*k[8]-k[5]*k[7],B=k[5]*k[6]-k[3]*k[8],C=k[3]*k[7]-k[4]*k[6];
 double det=k[0]*A+k[1]*B+k[2]*C;
 if(!std::isfinite(det)||det==0||k[0]<=0||k[4]<=0)return fail(2,"singular K");
 double ki[9]={A/det,(k[2]*k[7]-k[1]*k[8])/det,(k[1]*k[5]-k[2]*k[4])/det,
 B/det,(k[0]*k[8]-k[2]*k[6])/det,(k[2]*k[3]-k[0]*k[5])/det,
 C/det,(k[1]*k[6]-k[0]*k[7])/det,(k[0]*k[4]-k[1]*k[3])/det};
 w[0]=0x4f425331;w[1]=3;w[2]=914;w[3]=a.width;w[4]=a.height;w[5]=a.count;w[6]=a.min_pixels;
 put64(w,7,midpoint(a.valid_fraction));w[9]=even(a.valid_fraction);w[10]=0;
 put64(w,11,a.frame_id);
 for(unsigned axis=0;axis<3;++axis){
  c.anchor[axis]=a.T[axis*4+3];
  std::int64_t qa[3];
  for(unsigned j=0;j<3;++j){
   double v=0;for(unsigned r=0;r<3;++r)v+=a.T[axis*4+r]*ki[r*3+j];
   if(!coefficient(v,qa[j]))return fail(4,"A coefficient range");
   put64(w,16+2*(axis*3+j),static_cast<std::uint64_t>(qa[j]));
  }
  // Affine extrema occur at image corners. Include coefficient and depth
  // quantization margins; do not silently saturate the world histogram.
  for(unsigned yy: {0U,a.height-1})for(unsigned xx:{0U,a.width-1}){
   long double ray=(static_cast<long double>(qa[0])*xx+static_cast<long double>(qa[1])*yy+qa[2])/4294967296.L;
   if(std::fabs(ray)*10000.L+.502L>32767.L)return fail(4,"world histogram range");
  }
  std::int64_t qr;if(!coefficient(a.T[axis*4+2],qr))return fail(4,"range coefficient");
  put64(w,34+2*axis,static_cast<std::uint64_t>(qr));
 }
 w[44]=1;w[45]=0;w[46]=512;
 bool slow=false;std::uint32_t sum=0;
 for(unsigned b=0;b<a.count;++b){
  const auto& z=a.boxes[b];
  if(!std::isfinite(z.x0)||!std::isfinite(z.x1)||!std::isfinite(z.y0)||!std::isfinite(z.y1)||
   std::max({std::fabs(z.x0),std::fabs(z.x1),std::fabs(z.y0),std::fabs(z.y1)})>1e8)return fail(2,"box coordinates");
  double dx=(z.x1-z.x0)*.15,dy=(z.y1-z.y0)*.15;
  std::int32_t x0=std::max(0.,std::ceil(z.x0+dx)),y0=std::max(0.,std::ceil(z.y0+dy));
  std::int32_t x1=std::min(double(a.width),std::floor(z.x1-dx)),y1=std::min(double(a.height),std::floor(z.y1-dy));
  bool empty=x1<=x0||y1<=y0;
  bool trunc=z.x0<=2.||z.y0<=2.||z.x1>=double(a.width)-3.||z.y1>=double(a.height)-3.;
  unsigned base=64+8*b;w[base]=z.id;w[base+1]=x0;w[base+2]=y0;w[base+3]=x1;w[base+4]=y1;
  w[base+5]=0;w[base+6]=unsigned(trunc)|(unsigned(empty)<<1);
  std::uint32_t area=empty?0:std::uint32_t(x1-x0)*std::uint32_t(y1-y0);
  w[base+7]=area;sum+=area;slow|=area>65536;
  std::int64_t ex,ey;
  if(!coefficient((z.x1-z.x0)/a.K[0],ex)||!coefficient((z.y1-z.y0)/a.K[4],ey))return fail(4,"extent coefficient");
  put64(w,320+4*b,static_cast<std::uint64_t>(ex));put64(w,322+4*b,static_cast<std::uint64_t>(ey));
 }
 w[47]=sum;
 if(slow){
  if(!a.scratch_base||(a.scratch_base&63)||a.scratch_bytes<6176768||
     a.scratch_base>std::numeric_limits<std::uint64_t>::max()-a.scratch_bytes)return fail(3,"DDR scratch layout");
 }
 put64(w,48,a.scratch_base);put64(w,50,a.scratch_bytes);
 for(unsigned b=0;b<201;++b){double mode=double(b)*.05+.025;w[512+2*b]=cut(mode,false);w[513+2*b]=cut(mode,true);}
 return c;
}
}
