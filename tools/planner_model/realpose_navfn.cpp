// Continuous requested goal wrapper for the unchanged pinned Humble NavFn core.
#include "nav2_navfn_planner/navfn.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
extern "C" int navfn_realpose(const uint8_t* input,int w,int h,int sx,int sy,
 double gx,double gy,double resolution,double tolerance,double* ox,double* oy,int cap){
 if(!input || !ox || !oy || w<3 || h<3 || w>1024 || h>1024 || cap<w*h+2 ||
    sx<0 || sy<0 || sx>=w || sy>=h || !std::isfinite(gx) || !std::isfinite(gy) ||
    gx<-.5 || gy<-.5 || gx>w || gy>h || !std::isfinite(resolution) || resolution<1e-6 ||
    !std::isfinite(tolerance) || tolerance<0 || tolerance>std::max(w,h)*resolution)return -1;
 int gxi=int(std::round(gx)),gyi=int(std::round(gy));
 if(gxi<0 || gyi<0 || gxi>=w || gyi>=h)return -1;
 if(input[gyi*w+gxi]>=253)return 0;
 std::vector<uint8_t> m(input,input+w*h);m[sy*w+sx]=0;
 nav2_navfn_planner::NavFn nav(w,h);nav.setCostmap(m.data(),true,false);
 int start[2]={sx,sy},goal[2]={gxi,gyi};nav.setStart(goal);nav.setGoal(start);
 nav.calcNavFnDijkstra(true);
 bool found=nav.potarr[gyi*w+gxi]<POT_HIGH;double bx=gx,by=gy;
 if(!found){
  double best=std::numeric_limits<double>::max();
  for(double y=gy*resolution-tolerance;y<=gy*resolution+tolerance;y+=resolution)
   for(double x=gx*resolution-tolerance;x<=gx*resolution+tolerance;x+=resolution){
    if(x<0 || y<0)continue;
    int a=int(std::round(x/resolution)),b=int(std::round(y/resolution));
    double d=(x-gx*resolution)*(x-gx*resolution)+(y-gy*resolution)*(y-gy*resolution);
    if(a>=0 && b>=0 && a<w && b<h && nav.potarr[b*w+a]<POT_HIGH && d<best){
     found=true;best=d;bx=x/resolution;by=y/resolution;
    }
   }
 }
 if(!found)return 0;
 int target[2]={int(std::round(bx)),int(std::round(by))};nav.setStart(target);
 int n=nav.calcPath(std::max(w,h)*4);if(!n)return 0;
 for(int i=0;i<n;++i){ox[i]=nav.getPathX()[n-1-i];oy[i]=nav.getPathY()[n-1-i];}
 if(n>=2 && std::hypot(ox[n-1]-ox[n-2],oy[n-1]-oy[n-2])>
    std::hypot(bx-ox[n-2],by-oy[n-2])){ox[n-1]=bx;oy[n-1]=by;}
 else {ox[n]=bx;oy[n]=by;++n;}
 return n;
}
