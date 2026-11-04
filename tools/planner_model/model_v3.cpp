#include "model_v3.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace {
const int dx[8]={-1,0,1,-1,1,-1,0,1};
const int dy[8]={-1,-1,-1,0,0,1,1,1};
bool inside(int x,int y,int w,int h){return x>=0 && y>=0 && x<w && y<h;}
bool edge(const std::vector<uint8_t>& m,int w,int h,int x,int y,int a,int b){
  return inside(a,b,w,h) && m[b*w+a]<253 &&
    (x==a || y==b || (m[y*w+a]<253 && m[b*w+x]<253));
}
// Closed-square slab intersection, independent of descent. Includes corner contacts.
bool safe_segment(const uint8_t* m,int w,int h,double ax,double ay,double bx,double by){
  const double eps=1e-10;
  int x0=static_cast<int>(std::ceil(std::min(ax,bx)-.5-eps));
  int x1=static_cast<int>(std::floor(std::max(ax,bx)+.5+eps));
  int y0=static_cast<int>(std::ceil(std::min(ay,by)-.5-eps));
  int y1=static_cast<int>(std::floor(std::max(ay,by)+.5+eps));
  for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x){
    double low=0,high=1;
    const double a[2]={ax,ay},d[2]={bx-ax,by-ay},c[2]={double(x),double(y)};
    for(int k=0;k<2;++k){
      if(std::abs(d[k])<1e-14){
        if(a[k]<c[k]-.5-eps || a[k]>c[k]+.5+eps) high=-1;
      } else {
        double u=(c[k]-.5-a[k])/d[k],v=(c[k]+.5-a[k])/d[k];
        low=std::max(low,std::min(u,v));high=std::min(high,std::max(u,v));
      }
    }
    if(low<=high+eps && (!inside(x,y,w,h) || m[y*w+x]>=253)) return false;
  }
  return true;
}

int move_cost(int variant,int a,int b){return variant ? (a && b ? 14 : 10) : 1;}
// Distance to obstacle centres, exactly the existing clearance metric (cell units).
double point_segment_distance2(double x,double y,double ax,double ay,double bx,double by){
  double dx=bx-ax,dy=by-ay,den=dx*dx+dy*dy;
  double t=den>0 ? ((x-ax)*dx+(y-ay)*dy)/den : 0;
  t=std::max(0.,std::min(1.,t));
  double ex=x-ax-t*dx,ey=y-ay-t*dy;return ex*ex+ey*ey;
}
double limited_clearance2(const uint8_t* m,int w,int h,double ax,double ay,
    double bx,double by,double best){
  double radius=std::sqrt(best)+1e-9;
  int x0=std::max(0,int(std::ceil(std::min(ax,bx)-radius)));
  int x1=std::min(w-1,int(std::floor(std::max(ax,bx)+radius)));
  int y0=std::max(0,int(std::ceil(std::min(ay,by)-radius)));
  int y1=std::min(h-1,int(std::floor(std::max(ay,by)+radius)));
  for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x)if(m[y*w+x]>=253)
    best=std::min(best,point_segment_distance2(x,y,ax,ay,bx,by));
  return best;
}
int constrained_smooth(const uint8_t* m,int w,int h,double* x,double* y,int n){
  if(n<3)return n;
  double floor2=double(w)*w+double(h)*h;
  floor2=limited_clearance2(m,w,h,x[0],y[0],x[0],y[0],floor2);
  for(int i=1;i<n;++i)floor2=limited_clearance2(m,w,h,x[i-1],y[i-1],x[i],y[i],floor2);
  std::vector<double> outx(1,x[0]),outy(1,y[0]);
  for(int i=0;i<n-1;){
    int chosen=i+1;
    for(int j=n-1;j>i+1;--j){
      if(safe_segment(m,w,h,x[i],y[i],x[j],y[j]) &&
          limited_clearance2(m,w,h,x[i],y[i],x[j],y[j],floor2)+1e-10>=floor2){
        chosen=j;break;
      }
    }
    outx.push_back(x[chosen]);outy.push_back(y[chosen]);i=chosen;
  }
  std::copy(outx.begin(),outx.end(),x);std::copy(outy.begin(),outy.end(),y);
  return int(outx.size());
}
template<class T> int plan(const uint8_t* input,int w,int h,int sx,int sy,
    double gx,double gy,double resolution,double tolerance,double* ox,double* oy,
    int cap,uint32_t* rounds,int* status,double* resolved,int variant){
  const int goal_x=static_cast<int>(std::round(gx)),goal_y=static_cast<int>(std::round(gy));
  if(sx==0 || sy==0 || sx==w-1 || sy==h-1){
    *status=V2_INVALID_START;return 0;
  }
  if(input[goal_y*w+goal_x]>=253){*status=V2_INVALID_GOAL;return 0;}
  std::vector<uint8_t> m(input,input+w*h);
  m[sy*w+sx]=0; // User-authorized NavFn clearRobotCell; no other cell is cleared.
  for(int x=0;x<w;++x)m[x]=m[(h-1)*w+x]=254;
  for(int y=0;y<h;++y)m[y*w]=m[y*w+w-1]=254;
  const T inf=std::numeric_limits<T>::max()/4;
  std::vector<T> potential(w*h,inf);
  std::vector<uint32_t> entry(w*h);
  for(int i=0;i<w*h;++i)entry[i]=static_cast<uint32_t>(50+0.8*m[i]);
  potential[sy*w+sx]=0;
  bool changed=true;
  while(changed){
    changed=false;
    for(int direction=0;direction<2;++direction){
      for(int step=0;step<w*h;++step){
        const int i=direction ? w*h-1-step : step,x=i%w,y=i/w;
        if(m[i]>=253 || i==sy*w+sx)continue;
        T best=potential[i];
        for(int k=0;k<8;++k){
          int a=x+dx[k],b=y+dy[k];
          if(edge(m,w,h,x,y,a,b) && potential[b*w+a]<inf &&
             potential[b*w+a]+T(entry[i])*move_cost(variant,dx[k],dy[k])<best)
            best=potential[b*w+a]+T(entry[i])*move_cost(variant,dx[k],dy[k]);
        }
        if(best<potential[i]){potential[i]=best;changed=true;}
      }
    }
    ++*rounds;
    if(*rounds>static_cast<uint32_t>(w*h)){*status=V2_NOT_CONVERGED;return 0;}
  }
  int tx=goal_x,ty=goal_y;
  double px=gx,py=gy;
  bool exact=potential[ty*w+tx]<inf;
  if(exact){
    if(!safe_segment(m.data(),w,h,tx,ty,px,py)){*status=V2_SAFETY_REJECTED;return 0;}
  } else {
    bool found=false;
    double best_distance=std::numeric_limits<double>::max();
    // Literal fixed-Humble loop order and floating arithmetic in metres.
    const double goal_mx=gx*resolution,goal_my=gy*resolution;
    for(double y=goal_my-tolerance;y<=goal_my+tolerance;y+=resolution){
      for(double x=goal_mx-tolerance;x<=goal_mx+tolerance;x+=resolution){
        if(x<0 || y<0)continue;
        int a=static_cast<int>(std::round(x/resolution));
        int b=static_cast<int>(std::round(y/resolution));
        if(!inside(a,b,w,h) || potential[b*w+a]>=inf)continue;
        double d=(x-goal_mx)*(x-goal_mx)+(y-goal_my)*(y-goal_my);
        if(d<best_distance && safe_segment(m.data(),w,h,a,b,x/resolution,y/resolution)){
          found=true;best_distance=d;tx=a;ty=b;px=x/resolution;py=y/resolution;
        }
      }
    }
    if(!found){*status=V2_NO_PATH;return 0;}
  }
  int x=tx,y=ty,n=0;
  for(int step=0;step<w*h;++step){
    ox[n]=x;oy[n]=y;++n;
    if(x==sx && y==sy)break;
    T best=potential[y*w+x];int a=-1,b=-1;
    long long best_distance=std::numeric_limits<long long>::max();
    for(int k=0;k<8;++k){
      int xx=x+dx[k],yy=y+dy[k];
      if(!edge(m,w,h,x,y,xx,yy) || potential[yy*w+xx]>=inf)continue;
      T candidate=potential[yy*w+xx];
      if(variant)candidate+=T(entry[y*w+x])*move_cost(variant,dx[k],dy[k]);
      long long distance=1LL*(xx-sx)*(xx-sx)+1LL*(yy-sy)*(yy-sy);
      if((!variant && candidate<best) || (variant && candidate<=potential[y*w+x] &&
          (a<0 || candidate<best || (candidate==best && distance<best_distance)))){
        best=candidate;best_distance=distance;a=xx;b=yy;
      }
    }
    if(a<0){*status=V2_NO_PATH;return 0;}
    x=a;y=b;
  }
  if(x!=sx || y!=sy || n+1>cap){*status=V2_NO_PATH;return 0;}
  std::reverse(ox,ox+n);std::reverse(oy,oy+n);
  if(ox[n-1]!=px || oy[n-1]!=py){ox[n]=px;oy[n]=py;++n;}
  if(n==1 && !safe_segment(m.data(),w,h,ox[0],oy[0],ox[0],oy[0])){
    *status=V2_SAFETY_REJECTED;return 0;
  }
  for(int i=1;i<n;++i){
    if(!safe_segment(m.data(),w,h,ox[i-1],oy[i-1],ox[i],oy[i])){
      *status=V2_SAFETY_REJECTED;return 0;
    }
  }
  if(variant==2)n=constrained_smooth(m.data(),w,h,ox,oy,n);
  resolved[0]=px;resolved[1]=py;
  *status=exact ? V2_EXACT : V2_WITHIN_TOLERANCE;
  return n;
}
}
extern "C" int wave_plan_v3(const uint8_t* m,int w,int h,int sx,int sy,
    double gx,double gy,double resolution,double tolerance,int mode,
    double* ox,double* oy,int cap,uint32_t* rounds,int* status,double* resolved,int variant){
  if(!rounds || !status || !resolved)return -1;
  *rounds=0;*status=V2_INVALID_ARGUMENTS;
  resolved[0]=resolved[1]=std::numeric_limits<double>::quiet_NaN();
  // 1024^2 * 251 * 14 < UINT64_MAX/4 and 2^53; fixed/float integer sums are exact.
  if(!m || !ox || !oy || w<3 || h<3 || w>1024 || h>1024 || cap<w*h+2 ||
     !inside(sx,sy,w,h) || !std::isfinite(gx) || !std::isfinite(gy) ||
     gx<-.5 || gy<-.5 || gx>w || gy>h ||
     !inside(static_cast<int>(std::round(gx)),static_cast<int>(std::round(gy)),w,h) ||
     !std::isfinite(resolution) || resolution<1e-6 ||
     !std::isfinite(std::max(w,h)*resolution) ||
     !std::isfinite(tolerance) || tolerance<0 || tolerance>std::max(w,h)*resolution ||
     (mode!=0 && mode!=1) || variant<0 || variant>2)return -1;
  if(mode==1)return plan<uint64_t>(m,w,h,sx,sy,gx,gy,resolution,tolerance,ox,oy,cap,rounds,status,resolved,variant);
  return plan<double>(m,w,h,sx,sy,gx,gy,resolution,tolerance,ox,oy,cap,rounds,status,resolved,variant);
}
