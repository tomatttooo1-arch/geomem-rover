#pragma once
// golden model (tools/planner_model/model_v3.cpp @ 21069ec, sha256 6232bdaf...; there the
// functions sit in an anonymous namespace and only run inside wave_plan_v3(variant=2)).
// Only change: free functions in namespace kv_nav_smooth, std::vector scratch kept.
// tests/test_planner_golden.cpp checks: kv_nav raw path -> smooth() == wave_plan_v3(variant 2) path,
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
namespace kv_nav_smooth {
inline bool inside(int x,int y,int w,int h){return x>=0 && y>=0 && x<w && y<h;}
// Closed-square slab intersection, independent of descent. Includes corner contacts.
inline bool safe_segment(const uint8_t* m,int w,int h,double ax,double ay,double bx,double by){
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
// Distance to obstacle centres, exactly the existing clearance metric (cell units).
inline double point_segment_distance2(double x,double y,double ax,double ay,double bx,double by){
  double dx=bx-ax,dy=by-ay,den=dx*dx+dy*dy;
  double t=den>0 ? ((x-ax)*dx+(y-ay)*dy)/den : 0;
  t=std::max(0.,std::min(1.,t));
  double ex=x-ax-t*dx,ey=y-ay-t*dy;return ex*ex+ey*ey;
}
inline double limited_clearance2(const uint8_t* m,int w,int h,double ax,double ay,
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
// m = the planner's WORKING map (start cleared, border lethal), as in wave_plan_v3.
// In-place; returns the new point count. Output point i is always an input point.
inline int constrained_smooth(const uint8_t* m,int w,int h,double* x,double* y,int n){
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
} // kv_nav_smooth
