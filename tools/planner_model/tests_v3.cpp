#include "model_v3.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
void check(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(){
 const int w=48,h=48,cap=w*h+2;std::vector<uint8_t> m(w*h,0);
 std::vector<double>x(cap),y(cap),xx(cap),yy(cap);uint32_t rounds=0,rr=0;
 int status=-1,ss=-1;double end[2],ee[2];
 auto run=[&](int variant,int mode,double gx=38,double gy=20){return wave_plan_v3(
  m.data(),w,h,8,20,gx,gy,.05,.15,mode,x.data(),y.data(),cap,&rounds,&status,end,variant);};
 int n=run(1,1);check(n==31 && status==V2_EXACT,"weighted axial path");
 for(int i=0;i<n;++i)check(y[i]==20 && x[i]==8+i,"axial detour");
 check(run(2,1)==2,"open straight shortcut");
 m[20*w+8]=253;const auto original=m;
 n=run(2,0);check(n>0 && m==original,"clear only copy");
 int k=wave_plan_v3(m.data(),w,h,8,20,38,20,.05,.15,1,xx.data(),yy.data(),cap,&rr,&ss,ee,2);
 check(n==k && rounds==rr && status==ss,"float/fixed status");
 for(int i=0;i<n;++i)check(x[i]==xx[i] && y[i]==yy[i],"float/fixed path");
 check(run(2,1,8,20)==0 && status==V2_INVALID_GOAL,"blocked original goal before clearing");
 m[20*w+8]=255;check(run(2,1)>0,"unknown start matches NavFn clearing");
 for(int y0=19;y0<=21;++y0)for(int x0=7;x0<=9;++x0)m[y0*w+x0]=254;
 check(run(2,1)==0 && status==V2_NO_PATH,"enclosed start must fail");
 m.assign(w*h,0);for(int y0=0;y0<h;++y0)m[y0*w+24]=255;
 check(run(2,1,25,20)>0 && status==V2_WITHIN_TOLERANCE && end[0]<24,"tolerance");
 check(run(2,1,24,20)==0 && status==V2_INVALID_GOAL,"unknown goal");
 check(run(2,1,40,20)==0 && status==V2_NO_PATH,"far unreachable");
 check(run(3,1)==-1 && status==V2_INVALID_ARGUMENTS,"invalid variant");
 std::cout<<"v3 PASS: axial, constrained shortcut, clear-copy, fixed/float, original goal, unknown start, enclosed, tolerance\n";
}
