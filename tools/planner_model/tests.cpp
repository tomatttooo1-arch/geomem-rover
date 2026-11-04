#include "model.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
void require(bool yes,const char* why){if(!yes) throw std::runtime_error(why);}
int main(){
  const int w=48,h=48,cap=w*h+2;
  std::vector<uint8_t> m(w*h,0);std::vector<double>x(cap),y(cap),a(cap),b(cap);
  uint32_t rounds=0,r2=0;
  int n=wave_plan(m.data(),w,h,4,4,40,36,x.data(),y.data(),cap,0,&rounds);
  int k=wave_plan(m.data(),w,h,4,4,40,36,a.data(),b.data(),cap,1,&r2);
  require(n>0 && n==k && x==a && y==b && rounds==r2,"float/fixed mismatch");
  require(navfn_plan(m.data(),w,h,4,4,40,36,a.data(),b.data(),cap,.15,1)>0,"NavFn open map failed");
  m[36*w+40]=254;
  require(wave_plan(m.data(),w,h,4,4,40,36,x.data(),y.data(),cap,0,&rounds)==0,"lethal goal accepted");
  require(navfn_plan(m.data(),w,h,4,4,40,36,a.data(),b.data(),cap,.15,1)==0,"NavFn lethal precheck failed");
  m[36*w+40]=255;
  require(wave_plan(m.data(),w,h,4,4,40,36,x.data(),y.data(),cap,0,&rounds)==0,"unknown goal accepted");
  m[36*w+40]=0;for(int yy=0;yy<h;++yy)m[yy*w+24]=254;
  require(wave_plan(m.data(),w,h,4,4,40,36,x.data(),y.data(),cap,0,&rounds)==0,"wall crossed");
  require(navfn_plan(m.data(),w,h,4,4,40,36,a.data(),b.data(),cap,.15,1)==0,"NavFn wall crossed");
  m.assign(w*h,254);m[4*w+4]=m[5*w+5]=0;
  require(wave_plan(m.data(),w,h,4,4,5,5,x.data(),y.data(),cap,0,&rounds)==0,"diagonal corner cut");
  std::cout<<"C++ controls PASS: float/fixed, upstream NavFn, lethal/unknown, wall, corner\n";
}
