#pragma once
// verbatim in ref/planner_hw_ref.hpp). The ONLY change is a mechanical split of Engine::plan()
// into check() + prepare() + scan() + extract(), so that a field built elsewhere (PL planner
// core, or the software scan) can be handed to the same PS path extraction. Every expression is
// unchanged; tests/test_planner_golden.cpp proves byte equality against the original plan() and
// CPU bit-exact specification, no vendor/third-party headers. Integer scan is
// the prospective PL boundary; continuous-goal selection/descent stays on PS.
#include <cstdint>
#include <cmath>
#include <limits>
namespace kv_nav_planner {
constexpr int kWidth=80, kHeight=80, kCells=6400;
constexpr std::uint32_t kInfinity=0xffffffu;
constexpr unsigned kMaxRounds=6; // Observed max 5 + 1; NOT_CONVERGED is required.
enum Status { EXACT=0, WITHIN_TOLERANCE=1, INVALID_START=2, INVALID_GOAL=3,
  NO_PATH=4, SAFETY_REJECTED=5, INVALID_ARGUMENTS=6, NOT_CONVERGED=7,
  UNSUPPORTED_SIZE=8 };
struct Request { int sx,sy; double gx,gy; };
struct Result {
  int status=INVALID_ARGUMENTS, count=0;
  unsigned rounds=0;
  bool field_valid=false;
  double resolved_x=std::numeric_limits<double>::quiet_NaN();
  double resolved_y=std::numeric_limits<double>::quiet_NaN();
};
namespace detail {
constexpr int dx[8]={-1,0,1,-1,1,-1,0,1};
constexpr int dy[8]={-1,-1,-1,0,0,1,1,1};
inline int imin(int a,int b){return a<b?a:b;}
inline int imax(int a,int b){return a>b?a:b;}
inline double dmin(double a,double b){return a<b?a:b;}
inline double dmax(double a,double b){return a>b?a:b;}
// Exhaustively checked against the frozen v3 expression for all 253 inputs.
// Integer table-equivalent conversion keeps floating arithmetic out of PL.
inline unsigned entry(std::uint8_t c){return 50u+(4u*c)/5u;}
template<int W,int H,std::uint32_t Inf> struct Engine {
  static_assert(W>=3 && H>=3 && W<=400 && H<=400,"reference dimensions");
  static_assert((W-2)*(H-2)-1 <= (Inf-1)/2510u,"potential bound exceeds sentinel");
  enum { N=W*H };
  static bool inside(int x,int y){return x>=0 && y>=0 && x<W && y<H;}
  static std::uint32_t add(std::uint32_t a,unsigned b){
    return a>=Inf || b>=Inf-a ? Inf : a+b;
  }
  static bool edge(const std::uint8_t* m,int x,int y,int a,int b){
    return inside(a,b) && m[b*W+a]<253 &&
      (x==a || y==b || (m[y*W+a]<253 && m[b*W+x]<253));
  }
  // Caller owns disjoint map scratch[N], potential[N], flags[>=limit or N].
  // limit=0 is the reference-only N-round bound, not an unlimited hardware mode.
  // [split] The working map the scan and the extraction both see: start cell cleared, border lethal.
  static void prepare(const std::uint8_t* input,int sx,int sy,std::uint8_t* m){
    for(int i=0;i<N;++i)m[i]=input[i];
    m[sy*W+sx]=0;
    for(int x=0;x<W;++x)m[x]=m[(H-1)*W+x]=254;
    for(int y=0;y<H;++y)m[y*W]=m[y*W+W-1]=254;
  }
  static bool scan(const std::uint8_t* input,int sx,int sy,std::uint8_t* m,
      std::uint32_t* p,unsigned limit,unsigned& rounds,std::uint8_t* flags){
    prepare(input,sx,sy,m);
    for(int i=0;i<N;++i)p[i]=Inf;
    p[sy*W+sx]=0;rounds=0;
    const unsigned bound=limit ? limit : unsigned(N);
    for(unsigned round=0;round<bound;++round){
      bool changed=false;
      for(int direction=0;direction<2;++direction){
        for(int step=0;step<N;++step){
          const int i=direction ? N-1-step : step,x=i%W,y=i/W;
          if(m[i]>=253 || i==sy*W+sx)continue;
          std::uint32_t best=p[i];
          // Forward W,NW,N,NE; reverse E,SE,S,SW (exact opposites).
          for(int k=0;k<4;++k){
            const int fx=k==0 ? -1 : k-2;
            const int fy=k==0 ? 0 : -1;
            const int a=x+(direction ? -fx : fx),b=y+(direction ? -fy : fy);
            if(!edge(m,x,y,a,b))continue;
            const auto v=add(p[b*W+a],entry(m[i])*(fx && fy ? 14u : 10u));
            if(v<best)best=v;
          }
          if(best<p[i]){p[i]=best;changed=true;}
        }
      }
      ++rounds;
      if(flags)flags[round]=changed ? 1 : 0;
      if(!changed)return true;
    }
    return false;
  }
  // Closed-square collision rule deliberately preserves v3 floating operations.
  static bool safe_segment(const std::uint8_t* m,double ax,double ay,double bx,double by){
    const double eps=1e-10;
    int x0=int(std::ceil(dmin(ax,bx)-.5-eps));
    int x1=int(std::floor(dmax(ax,bx)+.5+eps));
    int y0=int(std::ceil(dmin(ay,by)-.5-eps));
    int y1=int(std::floor(dmax(ay,by)+.5+eps));
    for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x){
      double low=0,high=1;
      const double a[2]={ax,ay},d[2]={bx-ax,by-ay},c[2]={double(x),double(y)};
      for(int k=0;k<2;++k){
        if(std::abs(d[k])<1e-14){
          if(a[k]<c[k]-.5-eps || a[k]>c[k]+.5+eps)high=-1;
        }else{
          double u=(c[k]-.5-a[k])/d[k],v=(c[k]+.5-a[k])/d[k];
          low=dmax(low,dmin(u,v));high=dmin(high,dmax(u,v));
        }
      }
      if(low<=high+eps && (!inside(x,y) || m[y*W+x]>=253))return false;
    }
    return true;
  }
  // [split] Request checks of the original plan(), in the original order. Returns false with
  // r.status set (INVALID_ARGUMENTS / INVALID_START / INVALID_GOAL) when the request is rejected.
  static bool check(const std::uint8_t* input,Request q,const std::uint8_t* m,
      const std::uint32_t* p,const double* ox,const double* oy,int cap,Result& r){
    if(!input || !m || !p || !ox || !oy || cap<N+2 || !inside(q.sx,q.sy) ||
       !std::isfinite(q.gx) || !std::isfinite(q.gy) || q.gx<-.5 || q.gy<-.5 ||
       q.gx>W || q.gy>H || !inside(int(std::round(q.gx)),int(std::round(q.gy))))return false;
    if(q.sx==0 || q.sy==0 || q.sx==W-1 || q.sy==H-1){r.status=INVALID_START;return false;}
    const int gx=int(std::round(q.gx)),gy=int(std::round(q.gy));
    if(input[gy*W+gx]>=253){r.status=INVALID_GOAL;return false;}
    return true;
  }
  // Full wrapper/PS reference; map, p, ox, oy, flags must not overlap.
  static Result plan(const std::uint8_t* input,Request q,std::uint8_t* m,
      std::uint32_t* p,double* ox,double* oy,int cap,unsigned limit,std::uint8_t* flags){
    Result r;
    if(!check(input,q,m,p,ox,oy,cap,r))return r;
    if(!scan(input,q.sx,q.sy,m,p,limit,r.rounds,flags)){r.status=NOT_CONVERGED;return r;}
    const unsigned rounds=r.rounds;
    r=extract(m,p,q,ox,oy,cap);
    r.rounds=rounds;
    return r;
  }
  // [split] PS half: goal resolution + steepest descent + closed-square safety, on the working
  // map m (= prepare(input, q.sx, q.sy)) and a COMPLETE field p (scan converged). Precondition:
  // check() passed. rounds is left 0; the caller copies the field builder's round count.
  static Result extract(const std::uint8_t* m,const std::uint32_t* p,Request q,
      double* ox,double* oy,int cap){
    Result r;
    const int gx=int(std::round(q.gx)),gy=int(std::round(q.gy));
    r.field_valid=true;
    int tx=gx,ty=gy;double px=q.gx,py=q.gy;
    const bool exact=p[ty*W+tx]<Inf;
    if(exact){
      if(!safe_segment(m,tx,ty,px,py)){r.status=SAFETY_REJECTED;return r;}
    }else{
      bool found=false;double best_distance=std::numeric_limits<double>::max();
      const double goal_mx=q.gx*.05,goal_my=q.gy*.05;
      for(double y=goal_my-.15;y<=goal_my+.15;y+=.05){
        for(double x=goal_mx-.15;x<=goal_mx+.15;x+=.05){
          if(x<0 || y<0)continue;
          int a=int(std::round(x/.05)),b=int(std::round(y/.05));
          if(!inside(a,b) || p[b*W+a]>=Inf)continue;
          double d=(x-goal_mx)*(x-goal_mx)+(y-goal_my)*(y-goal_my);
          if(d<best_distance && safe_segment(m,a,b,x/.05,y/.05)){
            found=true;best_distance=d;tx=a;ty=b;px=x/.05;py=y/.05;
          }
        }
      }
      if(!found){r.status=NO_PATH;return r;}
    }
    int x=tx,y=ty,n=0;
    for(int step=0;step<N;++step){
      ox[n]=x;oy[n]=y;++n;
      if(x==q.sx && y==q.sy)break;
      auto best=p[y*W+x];int a=-1,b=-1,best_distance=2147483647;
      for(int k=0;k<8;++k){
        const int xx=x+dx[k],yy=y+dy[k];
        if(!edge(m,x,y,xx,yy) || p[yy*W+xx]>=Inf)continue;
        const auto candidate=add(p[yy*W+xx],entry(m[y*W+x])*(dx[k] && dy[k] ? 14u : 10u));
        const int distance=(xx-q.sx)*(xx-q.sx)+(yy-q.sy)*(yy-q.sy);
        if(candidate<=p[y*W+x] && (a<0 || candidate<best ||
            (candidate==best && distance<best_distance))){
          best=candidate;best_distance=distance;a=xx;b=yy;
        }
      }
      if(a<0){r.status=NO_PATH;return r;}
      x=a;y=b;
    }
    if(x!=q.sx || y!=q.sy || n+1>cap){r.status=NO_PATH;return r;}
    for(int i=0;i<n/2;++i){
      double t=ox[i];ox[i]=ox[n-1-i];ox[n-1-i]=t;
      t=oy[i];oy[i]=oy[n-1-i];oy[n-1-i]=t;
    }
    if(ox[n-1]!=px || oy[n-1]!=py){ox[n]=px;oy[n]=py;++n;}
    if(n==1 && !safe_segment(m,ox[0],oy[0],ox[0],oy[0])){r.status=SAFETY_REJECTED;return r;}
    for(int i=1;i<n;++i)if(!safe_segment(m,ox[i-1],oy[i-1],ox[i],oy[i])){
      r.status=SAFETY_REJECTED;return r;
    }
    r.resolved_x=px;r.resolved_y=py;r.count=n;
    r.status=exact ? EXACT : WITHIN_TOLERANCE;return r;
  }
};
} // detail
// Only supported hardware-facing shape is 80x80. cap >= 6402 doubles per axis.
// Scratch: 6400 bytes + 6400 uint32. flags optional, length >= (K ? K : 6400).
// field_valid=false: potential is absent or partial, never consume as a solution.
// Caller must reject NOT_CONVERGED/fallback on PS; no partial path is returned.
inline Result plan(const std::uint8_t* map,int w,int h,Request q,std::uint8_t* scratch,
    std::uint32_t* potential,double* out_x,double* out_y,int cap,
    unsigned max_rounds=kMaxRounds,std::uint8_t* flags=nullptr){
  if(w!=kWidth || h!=kHeight){Result r;r.status=UNSUPPORTED_SIZE;return r;}
  return detail::Engine<80,80,kInfinity>::plan(map,q,scratch,potential,out_x,out_y,cap,max_rounds,flags);
}
// [split] PS extraction from an externally built field (PL core or software scan). input is the
// ORIGINAL map (as given to the field builder); m is caller scratch[6400] that receives prepare().
// The field must be complete (builder reported field_valid); a partial field is never consumed.
inline Result extract_from_field(const std::uint8_t* input,Request q,const std::uint32_t* field,
    unsigned rounds,std::uint8_t* scratch,double* out_x,double* out_y,int cap){
  using E=detail::Engine<80,80,kInfinity>;
  Result r;
  if(!E::check(input,q,scratch,field,out_x,out_y,cap,r))return r;
  E::prepare(input,q.sx,q.sy,scratch);
  r=E::extract(scratch,field,q,out_x,out_y,cap);
  r.rounds=rounds;
  return r;
}
// New PL field-only contract. Historical plan() default K=6 remains intact.
constexpr unsigned kHardwareMaxRounds=64;
struct FieldResult { unsigned status=INVALID_ARGUMENTS, rounds=0; bool field_valid=false; };
inline FieldResult build_field(const std::uint8_t* input,int sx,int sy,int gx,int gy,
    std::uint8_t* scratch,std::uint32_t* potential,std::uint8_t* flags,
    unsigned max_rounds=kHardwareMaxRounds){
  FieldResult r;
  for(int i=0;i<kCells;++i)potential[i]=kInfinity;
  for(unsigned i=0;i<kHardwareMaxRounds;++i)flags[i]=255;
  if(max_rounds==0 || max_rounds>kHardwareMaxRounds || sx<0 || sy<0 || sx>=80 || sy>=80 ||
      gx<0 || gy<0 || gx>=80 || gy>=80)return r;
  if(sx==0 || sy==0 || sx==79 || sy==79){r.status=INVALID_START;return r;}
  if(input[gy*80+gx]>=253){r.status=INVALID_GOAL;return r;}
  r.field_valid=detail::Engine<80,80,kInfinity>::scan(input,sx,sy,scratch,potential,max_rounds,r.rounds,flags);
  r.status=r.field_valid ? EXACT : NOT_CONVERGED; // EXACT here means FIELD_READY; PS resolves path status.
  if(!r.field_valid)for(int i=0;i<kCells;++i)potential[i]=kInfinity;
  return r;
}
} // kv_nav_planner
