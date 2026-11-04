#include "model.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace {
const int DX[8]={-1,0,1,-1,1,-1,0,1};
const int DY[8]={-1,-1,-1,0,0,1,1,1};
bool inside(int x,int y,int w,int h){return x>=0 && y>=0 && x<w && y<h;}
bool valid(const uint8_t* m,int w,int h,int sx,int sy,int gx,int gy,int cap){
  return m && w>=3 && h>=3 && w<=4096 && h<=4096 &&
    inside(sx,sy,w,h) && inside(gx,gy,w,h) && cap>=w*h+2;
}
// A diagonal may not cut a blocked corner. Border is closed, as NavFn::setupNavFn does.
bool edge(const std::vector<uint8_t>& m,int w,int h,int x,int y,int a,int b){
  if(!inside(a,b,w,h) || m[b*w+a]>=253) return false;
  if(a!=x && b!=y && (m[y*w+a]>=253 || m[b*w+x]>=253)) return false;
  return true;
}
std::vector<uint8_t> bordered(const uint8_t* input,int w,int h){
  std::vector<uint8_t> m(input,input+w*h);
  for(int x=0;x<w;++x) m[x]=m[(h-1)*w+x]=254;
  for(int y=0;y<h;++y) m[y*w]=m[y*w+w-1]=254;
  return m;
}
template<class T> int wave(const uint8_t* input,int w,int h,int sx,int sy,int gx,int gy,
                          double* ox,double* oy,int cap,int four,uint32_t* rounds){
  *rounds=0;
  if(!valid(input,w,h,sx,sy,gx,gy,cap)) return -1;
  // Shared task-level validity rule. Never clear an obstacle/unknown goal.
  if(input[gy*w+gx]>=253 || input[sy*w+sx]>=253) return 0;
  auto m=bordered(input,w,h);
  if(m[gy*w+gx]>=253 || m[sy*w+sx]>=253) return 0;
  const T inf=std::numeric_limits<T>::max()/4;
  std::vector<T> p(w*h,inf);
  p[gy*w+gx]=0;
  // Exact NavFn ROS cost conversion: truncate 50 + 0.8*c to integer before summing.
  std::vector<unsigned> entry(w*h);
  for(int i=0;i<w*h;++i) entry[i]=static_cast<unsigned>(50+0.8*m[i]);
  bool changed=true;
  // Positive edge weights: shortest simple path <= (N-1)*251, below uint32 sentinel.
  while(changed){
    changed=false;
    for(int dir=0;dir<2;++dir){
      for(int ii=0;ii<w*h;++ii){
        int i=dir ? w*h-1-ii : ii, x=i%w,y=i/w;
        if(m[i]>=253 || (x==gx && y==gy)) continue;
        T best=p[i];
        for(int k=0;k<8;++k){
          if(four && DX[k] && DY[k]) continue;
          int a=x+DX[k],b=y+DY[k];
          if(!edge(m,w,h,x,y,a,b)) continue;
          T nb=p[b*w+a];
          if(nb<inf && nb+entry[i]<best) best=nb+entry[i];
        }
        if(best<p[i]) {p[i]=best;changed=true;}
      }
    }
    ++*rounds;
    if(*rounds>static_cast<unsigned>(w*h)) return -1;
  }
  if(p[sy*w+sx]==inf) return 0;
  int x=sx,y=sy,n=0;
  for(int step=0;step<w*h;++step){
    ox[n]=x;oy[n]=y;++n;
    if(x==gx && y==gy) return n;
    T best=p[y*w+x]; int bx=-1,by=-1;
    // Literal lowest-potential 8-neighbour descent; fixed row-major tie order.
    for(int k=0;k<8;++k){
      if(four && DX[k] && DY[k]) continue;
      int a=x+DX[k],b=y+DY[k];
      if(edge(m,w,h,x,y,a,b) && p[b*w+a]<best){best=p[b*w+a];bx=a;by=b;}
    }
    if(bx<0) return 0;
    x=bx;y=by;
  }
  return 0;
}
}
extern "C" int wave_plan(const uint8_t* m,int w,int h,int sx,int sy,int gx,int gy,
 double* ox,double* oy,int cap,int mode,uint32_t* rounds){
  if(!rounds || !ox || !oy) return -1;
  // mode 0 = float64 reference, 1 = uint32 fixed point, 2 = deliberate 4-neighbour mutant.
  if(mode==1) return wave<uint32_t>(m,w,h,sx,sy,gx,gy,ox,oy,cap,0,rounds);
  return wave<double>(m,w,h,sx,sy,gx,gy,ox,oy,cap,mode==2,rounds);
}
namespace { thread_local int nav_diagnostics[7] = {}; }
extern "C" int navfn_debug(int field) {
  return (field>=0 && field<7) ? nav_diagnostics[field] : -1;
}
extern "C" int navfn_plan(const uint8_t* input,int w,int h,int sx,int sy,int gx,int gy,
 double* ox,double* oy,int cap,double tolerance,int enforce_goal){
  std::fill(nav_diagnostics,nav_diagnostics+7,0);
  if(!ox || !oy || !valid(input,w,h,sx,sy,gx,gy,cap)) {nav_diagnostics[0]=1;return -1;}
  if(enforce_goal && input[gy*w+gx]>=253) {nav_diagnostics[0]=2;return 0;}
  std::vector<uint8_t> m(input,input+w*h);
  m[sy*w+sx]=0; // NavfnPlanner::clearRobotCell, untouched original is used by safety scorer.
  nav2_navfn_planner::NavFn nav(w,h);
  nav.setCostmap(m.data(),true,false);
  int start[2]={sx,sy}, goal[2]={gx,gy};
  // Humble ROS wrapper propagates FROM robot and extracts/reverses FROM requested goal.
  nav.setStart(goal);nav.setGoal(start);nav_diagnostics[1]=nav.calcNavFnDijkstra(true);
  double tx=gx,ty=gy,bx=tx,by=ty;
  bool found=nav.potarr[gy*w+gx]<POT_HIGH;
  nav_diagnostics[3]=found;
  if(!found){
    double best=std::numeric_limits<double>::max();
    // Exact wrapper loops in metres, resolution=0.05; retain its floating endpoint semantics.
    for(double py=ty*0.05-tolerance;py<=ty*0.05+tolerance;py+=0.05){
      for(double px=tx*0.05-tolerance;px<=tx*0.05+tolerance;px+=0.05){
        if(px<0 || py<0) continue;
        int x=static_cast<int>(std::round(px/0.05)),y=static_cast<int>(std::round(py/0.05));
        double d=(px-tx*0.05)*(px-tx*0.05)+(py-ty*0.05)*(py-ty*0.05);
        if(inside(x,y,w,h) && nav.potarr[y*w+x]<POT_HIGH && d<best){
          best=d;bx=px/0.05;by=py/0.05;found=true;
        }
      }
    }
  }
  if(!found) {nav_diagnostics[0]=3;return 0;}
  int target[2]={static_cast<int>(std::round(bx)),static_cast<int>(std::round(by))};
  nav_diagnostics[4]=target[0];nav_diagnostics[5]=target[1];
  nav.setStart(target);
  int len=nav.calcPath(std::max(w,h)*4);
  nav_diagnostics[2]=len;
  if(!len) {
    nav_diagnostics[0]=4;
    // Audit only: test whether default path-buffer cycle limit caused failure.
    // Never promote this diagnostic result to a successful formal baseline.
    nav_diagnostics[6]=nav.calcPath(w*h*4);
    return 0;
  }
  nav_diagnostics[0]=5;
  int n=0;
  for(int i=len-1;i>=0;--i){ox[n]=nav.getPathX()[i];oy[n]=nav.getPathY()[i];++n;}
  // smoothApproachToGoal, preserving Humble's corner-coordinate mapToWorld convention.
  if(n>=2 && std::hypot(ox[n-1]-ox[n-2],oy[n-1]-oy[n-2])>
             std::hypot(bx-ox[n-2],by-oy[n-2])){ox[n-1]=bx;oy[n-1]=by;}
  else {ox[n]=bx;oy[n]=by;++n;}
  return n;
}
