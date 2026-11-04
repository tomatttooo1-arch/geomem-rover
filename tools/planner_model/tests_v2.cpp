#include "model_v2.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
void check(bool v,const char* msg){if(!v)throw std::runtime_error(msg);}
int main(){
  const int w=48,h=48,cap=w*h+2;
  std::vector<uint8_t> m(w*h,0);
  std::vector<double> x(cap),y(cap),fx(cap),fy(cap);
  uint32_t rounds=0,fr=0;int status=-1,fs=-1;double end[2],fe[2];
  auto run=[&](double gx,double gy,int mode,double tolerance){
    return wave_plan_v2(m.data(),w,h,8,20,gx,gy,.05,tolerance,mode,x.data(),y.data(),cap,&rounds,&status,end);
  };
  int n=run(40,35,0,.15);
  int k=wave_plan_v2(m.data(),w,h,8,20,40,35,.05,.15,1,fx.data(),fy.data(),cap,&fr,&fs,fe);
  check(n>0 && n==k && status==V2_EXACT && fs==status && fr==rounds,"open float/fixed");
  for(int i=0;i<n;++i)check(x[i]==fx[i] && y[i]==fy[i],"path mismatch");
  check(x[0]==8 && y[0]==20 && x[n-1]==40 && y[n-1]==35,"exact endpoints");
  for(int yy=0;yy<h;++yy)m[yy*w+24]=255;
  n=run(25,20,1,.15);
  check(n>0 && status==V2_WITHIN_TOLERANCE && end[0]<24 && std::abs(end[0]-25)<=3.00000001,"safe tolerance");
  check(run(25,20,1,0)==0 && status==V2_NO_PATH,"zero tolerance must fail");
  check(run(40,20,1,.15)==0 && status==V2_NO_PATH,"far target must fail");
  check(run(24,20,1,.15)==0 && status==V2_INVALID_GOAL,"unknown target");
  m[20*w+24]=253;
  check(run(24,20,1,.15)==0 && status==V2_INVALID_GOAL,"inscribed target");
  m[20*w+8]=253;
  check(run(20,20,1,.15)==0 && status==V2_INVALID_START,"blocked start must not clear");
  m.assign(w*h,0);m[20*w+21]=254;
  check(run(20.5,20,1,.15)==0 && status==V2_INVALID_GOAL,"goal quantization");
  check(run(19.5,20,1,.15)>0 && status==V2_EXACT,"fractional safe endpoint");
  check(run(18,20,1,-.1)==-1 && status==V2_INVALID_ARGUMENTS,"negative tolerance");
  check(run(std::nan(""),20,1,.15)==-1,"nan goal");
  std::cout<<"v2 controls PASS: exact/fixed, tolerance, unknown/lethal, start, fractional, invalid args\n";
}
