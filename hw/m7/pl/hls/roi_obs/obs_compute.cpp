#include "obs_types.hpp"
#include "obs_math.hpp"
#include "obs_ccl.hpp"
namespace roi_obs {
namespace {
void ccl_depth(hls::stream<ap_uint<64>>& input,hls::stream<ap_uint<16>>& depth,ap_uint<17> area){
 #pragma HLS INLINE off
 for(unsigned i=0;i<area;++i){
  #pragma HLS PIPELINE II=1
  ap_uint<64> p=input.read();depth.write(p.range(15,0));
 }
}
void ccl_phase(hls::stream<ap_uint<64>>& input,ap_uint<17> area,
 ap_uint<11> width,ap_uint<10> height,ap_uint<17> lo,ap_uint<17> hi,
 ap_uint<64> labels[16384],ap_uint<72> parents[16384],ap_uint<38> runs[1280],CclSummary& summary){
 #pragma HLS INLINE off
 #pragma HLS DATAFLOW
 hls::stream<ap_uint<16>> depth;
 #pragma HLS STREAM variable=depth depth=16
 ccl_depth(input,depth,area);
 ccl_fast(depth,width,height,lo,hi,labels,parents,runs,summary);
}
// Three physical lanes. Keep bin while an ordered rank is captured; no
// 18-comparator capture tree. The same accumulator/cursor datapath serves
// coarse, fine and HSV scans. Header/ranks are stable for the entire scan.
void cdf(ap_uint<2> kind,ap_uint<17> world[3][2048],ap_uint<17> hsv[3][256],BoxState& s){
 #pragma HLS INLINE off
 #pragma HLS ARRAY_PARTITION variable=world complete dim=1
 #pragma HLS ARRAY_PARTITION variable=hsv complete dim=1
 ap_uint<18> cumulative[3];ap_uint<3> cursor[3]={0,0,0};
 for(unsigned a=0;a<3;++a){
  #pragma HLS UNROLL
  // Fine ranks are global; account for all bins below the first window.
  cumulative[a]=kind==1?s.below[a][0]:ap_uint<17>(0);
 }
 #pragma HLS ARRAY_PARTITION variable=cumulative complete
 #pragma HLS ARRAY_PARTITION variable=cursor complete
 ap_uint<11> address=0;ap_uint<3> window=0;
 unsigned limit=kind==1?1536:256;
 while(address<limit){
  // No pipeline over the variable hold loop: it is an explicit scalar FSM.
  ap_uint<17> count[3];bool hold[3];
  #pragma HLS ARRAY_PARTITION variable=count complete
  #pragma HLS ARRAY_PARTITION variable=hold complete
  for(unsigned a=0;a<3;++a){
   #pragma HLS UNROLL
   count[a]=kind==2?hsv[a][address.range(7,0)]:world[a][kind==1?unsigned(address)+256:unsigned(address)];
   hold[a]=false;
   ap_uint<3> k=cursor[a];
   if(k<(kind==2?2:6)){
    ap_uint<17> rank=kind==2?(k==0?ap_uint<17>((s.selected-1)>>1):ap_uint<17>(s.selected>>1)):s.ranks[k];
    bool active=kind!=1 || (window==k || (window<k && s.windows[a][window]==s.windows[a][k]));
    if(active && cumulative[a]+count[a]>rank){
     hold[a]=true;
     if(kind==0){s.windows[a][k]=address;s.below[a][k]=cumulative[a];}
     else if(kind==1){ap_uint<16> q=(ap_uint<16>(s.windows[a][k])<<8)|address.range(7,0);s.values[a][k]=ap_int<17>(q)-32768;}
     else if(k==0)s.hsv2[a]=address;else s.hsv2[a]+=address;
     cursor[a]=k+1;
    }
   }
  }
  bool any=hold[0]||hold[1]||hold[2];
  // Lanes which did not capture must not consume the same bin again while
  // a different lane holds it. Commit cumulative only on shared advance.
  if(!any){
   for(unsigned a=0;a<3;++a){
    #pragma HLS UNROLL
    cumulative[a]+=count[a];
   }
   ++address;
   if(kind==1 && address.range(7,0)==0){
    ++window;
    for(unsigned a=0;a<3;++a){
     #pragma HLS UNROLL
     if(window<6)cumulative[a]=s.below[a][window];
    }
   }
  }
 }
}
void emit(const Descriptor& d,const BoxState& s,hls::stream<ap_uint<32>>& output){
 #pragma HLS INLINE off
 // One monotonically increasing output counter; payload words are prepared
 // in a tiny SRL table to avoid wide per-word multiway comparisons.
 ap_uint<32> record[32];
 #pragma HLS BIND_STORAGE variable=record type=ram_1p impl=lutram
 for(unsigned j=0;j<32;++j)record[j]=0;
 record[0]=d.id;record[1]=s.status;record[3]=0xffffffffU;
 record[4]=d.area;record[5]=s.valid;record[6]=s.selected;
 record[20]=d.x0;record[21]=d.y0;record[22]=d.x1;record[23]=d.y1;record[27]=s.mode;
 if(s.status==0 && s.error==0){
  record[2]=d.flags&1;record[3]=s.color;
  for(unsigned a=0;a<3;++a){record[8+a]=s.position[a];record[11+a]=s.lower[a];record[14+a]=s.upper[a];record[24+a]=s.hsv2[a];}
  record[17]=s.range;record[18]=s.extent[0];record[19]=s.extent[1];
 }
 for(unsigned j=0;j<32;++j){
  #pragma HLS PIPELINE II=1
  output.write(record[j]);
 }
}
}
// One frame-level multiplier; pixel projection remains three parallel lanes.
static ap_int<73> mul40x33_shared(ap_int<40> a,ap_int<33> b){
 #pragma HLS INLINE off
 ap_int<73> product=a*b;
 #pragma HLS BIND_OP variable=product op=mul impl=dsp latency=4
 return product;
}
void obs_compute(ap_uint<2> phase,const Descriptor& d,const Geometry& g,const Profile& profile,
 hls::stream<ap_uint<64>>& input,BoxState& s,hls::stream<ap_uint<32>>& output){
 #pragma HLS INLINE off
 #pragma HLS ALLOCATION function instances=mul40x33_shared limit=1
 static ap_uint<17> world[3][2048],hsv[3][256];
 static ap_uint<64> labels[16384];static ap_uint<72> parents[16384];static ap_uint<38> runs[1280];
 #pragma HLS ARRAY_PARTITION variable=world complete dim=1
 #pragma HLS ARRAY_PARTITION variable=hsv complete dim=1
 #pragma HLS BIND_STORAGE variable=world type=ram_t2p impl=bram latency=2
 #pragma HLS BIND_STORAGE variable=hsv type=ram_t2p impl=bram latency=2
 #pragma HLS BIND_STORAGE variable=labels type=ram_2p impl=uram latency=2
 #pragma HLS BIND_STORAGE variable=parents type=ram_2p impl=uram latency=2
 #pragma HLS BIND_STORAGE variable=runs type=ram_2p impl=uram latency=2
 if(phase==DEPTH_HIST){
  s.status=(d.flags&2)?1:0;s.error=0;s.valid=0;s.selected=0;s.root_seen=false;s.mode=0;
  for(unsigned i=0;i<256;++i)world[0][i]=0;
  ap_uint<8> tag0=0,tag1=0;ap_uint<17> val0=0,val1=0;bool live0=false,live1=false;
  for(unsigned i=0;i<d.area;++i){
   #pragma HLS PIPELINE II=1
   #pragma HLS DEPENDENCE variable=world type=inter direction=RAW dependent=true distance=3
   ap_uint<16> code=input.read().range(15,0);ap_uint<8> bin=depth_bin(code);
   if(bin!=255){
    ap_uint<17> old;
    if(live0&&tag0==bin)old=val0;
    else if(live1&&tag1==bin)old=val1;
    else old=world[0][bin];
    ap_uint<17> v=old+1;world[0][bin]=v;
    tag1=tag0;val1=val0;live1=live0;tag0=bin;val0=v;live0=true;++s.valid;
   }
  }
  if(s.status==0 && (s.valid<profile.min_pixels || !valid_ratio(s.valid,d.area,profile.valid_m,profile.valid_tie)))s.status=2;
  ap_uint<17> maximum=0;ap_uint<8> fallback=0,nearest=0;bool found=false;
  ap_uint<17> prev=0;
  // A one-bin delay supplies c[b+1] without a second memory read call site.
  // ceil(N/5) is exact for this <=65536 default profile (proved in design).
  ap_uint<17> need=(ap_uint<20>(s.valid)+4)/5;
  for(unsigned i=0;i<=201;++i){
   ap_uint<17> n=i<201?world[0][i]:ap_uint<17>(0);
   if(i<201&&n>maximum){maximum=n;fallback=i;}
   if(i&&prev&&!found&&ap_uint<18>(prev)+n>=need){nearest=prev>=n?i-1:i;found=true;}
   prev=n;
  }
  s.mode=found?nearest:fallback;s.cut_lo=profile.lower[s.mode];s.cut_hi=profile.upper[s.mode];
 }else if(phase==COMPONENT){
  CclSummary summary;
  ap_uint<11> w=d.x1-d.x0;ap_uint<10> h=d.y1-d.y0;
  ccl_phase(input,d.area,w,h,s.cut_lo,s.cut_hi,labels,parents,runs,summary);
  s.selected=summary.max_area;s.root_seen=false;
  if(summary.error)s.error=3;
  if(!s.selected)s.status=3;else if(s.selected<profile.min_pixels)s.status=4;
  const ap_uint<21> nm=ap_uint<21>(s.selected)-1;
  // ceil(2^25/20), exact floor for all 21-bit nm including selected==0.
  const ap_uint<42> reciprocal_product=nm*ap_uint<21>(1677722);
  const ap_uint<17> q=reciprocal_product.range(41,25);
  const ap_uint<25> twenty_q=(ap_uint<25>(q)<<4)+(ap_uint<25>(q)<<2);
  const ap_uint<5> r=ap_uint<25>(nm)-twenty_q;
  s.rank_rem=r;
  s.ranks[0]=q;s.ranks[1]=q+ap_uint<1>(r!=0);
  s.ranks[2]=nm>>1;s.ranks[3]=(ap_uint<22>(nm)+1)>>1;
  s.ranks[4]=nm-q-ap_uint<1>(r!=0);s.ranks[5]=nm-q;
 }else{
  bool fine=phase==FINE;
  unsigned clear_limit=fine?1536:256;
  for(unsigned i=0;i<clear_limit;++i){
   #pragma HLS PIPELINE II=1
   for(unsigned a=0;a<3;++a){
    #pragma HLS UNROLL
    world[a][fine?i+256:i]=0;if(!fine)hsv[a][i]=0;
   }
  }
  ap_int<53> ray[3],row[3];
  #pragma HLS ARRAY_PARTITION variable=ray complete
  #pragma HLS ARRAY_PARTITION variable=row complete
  for(ap_uint<2> a=0;a<3;++a){
   #pragma HLS UNROLL factor=1
   const ap_uint<4> j=(ap_uint<4>(a)<<1)+a;
   const ap_int<52> px=mul40x33_shared(g.a[j],ap_int<33>(ap_int<12>(d.x0)));
   const ap_int<51> py=mul40x33_shared(g.a[j+1],ap_int<33>(ap_int<11>(d.y0)));
   row[a]=px+py+g.a[j+2];ray[a]=row[a];
  }
  ap_uint<11> x=0;ap_uint<64> labelword=0;
  ap_uint<11> tag[3][2];ap_uint<17> cached[3][2];bool live[3][2]={};
  ap_uint<8> htag[3][2];ap_uint<17> hcached[3][2];bool hlive[3][2]={};
  #pragma HLS ARRAY_PARTITION variable=tag complete
  #pragma HLS ARRAY_PARTITION variable=cached complete
  #pragma HLS ARRAY_PARTITION variable=live complete
  #pragma HLS ARRAY_PARTITION variable=htag complete
  #pragma HLS ARRAY_PARTITION variable=hcached complete
  #pragma HLS ARRAY_PARTITION variable=hlive complete
  ap_uint<11> width=d.x1-d.x0;
  for(unsigned i=0;i<d.area;++i){
   #pragma HLS PIPELINE II=1
   #pragma HLS DEPENDENCE variable=world type=inter direction=RAW dependent=true distance=3
   #pragma HLS DEPENDENCE variable=hsv type=inter direction=RAW dependent=true distance=3
   ap_uint<64> pixel=input.read();ap_uint<16> code=pixel.range(15,0);
   if((i&3)==0)labelword=labels[i>>2];
   ap_uint<16> label=labelword.range(15,0);labelword>>=16;
   ap_uint<15> index=label-1;
   ap_uint<72> packed=parents[index>>1];
   ap_uint<36> node=index[0]?ap_uint<36>(packed>>36):ap_uint<36>(packed);
   bool best=false;
   if(label){
    ap_uint<15> root=node.range(14,0);
    if(!s.root_seen&&node[32]){s.root=root;s.root_seen=true;}
    best=s.root_seen&&root==s.root;
   }
   ap_int<16> mm[3];ap_uint<8> hv[3];
   #pragma HLS ARRAY_PARTITION variable=mm complete
   #pragma HLS ARRAY_PARTITION variable=hv complete
   rgb_hsv(pixel.range(39,16),hv[0],hv[1],hv[2]);
   for(unsigned a=0;a<3;++a){
    #pragma HLS UNROLL
    ap_int<71> product=ray[a]*ap_int<18>(code);
    ap_int<32> value=round_q32(product);
    mm[a]=value;
    if(best&&(value< -32768||value>32767))s.error=4;
    ap_uint<16> biased=ap_int<17>(mm[a])+32768;
    ap_uint<8> coarse=biased.range(15,8),low=biased.range(7,0);
    ap_uint<3> slot=0;bool match=!fine;
    if(fine){
     bool seen=false;
     for(unsigned j=0;j<6;++j){
      #pragma HLS UNROLL
      if(!seen&&s.windows[a][j]==coarse){slot=j;seen=true;}
     }
     match=seen;
    }
    ap_uint<11> address=fine?ap_uint<11>(256+unsigned(slot)*256+low):ap_uint<11>(coarse);
    if(best&&match){
     ap_uint<17> old;
     if(live[a][0]&&tag[a][0]==address)old=cached[a][0];
     else if(live[a][1]&&tag[a][1]==address)old=cached[a][1];
     else old=world[a][address];
     ap_uint<17> v=old+1;world[a][address]=v;
     tag[a][1]=tag[a][0];cached[a][1]=cached[a][0];live[a][1]=live[a][0];tag[a][0]=address;cached[a][0]=v;live[a][0]=true;
    }
    if(best&&!fine){
     ap_uint<8> address_h=hv[a];ap_uint<17> old;
     if(hlive[a][0]&&htag[a][0]==address_h)old=hcached[a][0];
     else if(hlive[a][1]&&htag[a][1]==address_h)old=hcached[a][1];
     else old=hsv[a][address_h];
     ap_uint<17> v=old+1;hsv[a][address_h]=v;
     htag[a][1]=htag[a][0];hcached[a][1]=hcached[a][0];hlive[a][1]=hlive[a][0];htag[a][0]=address_h;hcached[a][0]=v;hlive[a][0]=true;
    }
    // Update on every raster pixel, including invalid/background pixels.
    if(x+1==width){row[a]+=g.a[a*3+1];ray[a]=row[a];}else ray[a]+=g.a[a*3];
   }
   if(x+1==width)x=0;else ++x;
  }
  cdf(fine?ap_uint<2>(1):ap_uint<2>(0),world,hsv,s);
  if(!fine){cdf(2,world,hsv,s);s.color=classify(s.hsv2[0],s.hsv2[1],s.hsv2[2]);}
  else{
   // Preserve the original mixed-signed selected==0 diagnostic behavior.
   const ap_uint<5> lo_rem=s.selected?s.rank_rem:ap_uint<5>(31);
   const ap_uint<5> hi_rem=s.selected?
    (s.rank_rem?ap_uint<5>(20-s.rank_rem):ap_uint<5>(0)):ap_uint<5>(9);
   ap_int<80> range=0;
   for(unsigned a=0;a<3;++a){
    s.position[a]=500*(ap_int<18>(s.values[a][2])+s.values[a][3]);
    s.lower[a]=50*((20-lo_rem)*ap_int<23>(s.values[a][0])+lo_rem*ap_int<23>(s.values[a][1]))-30000;
    s.upper[a]=50*((20-hi_rem)*ap_int<23>(s.values[a][4])+hi_rem*ap_int<23>(s.values[a][5]))+30000;
    const ap_int<73> range_product=mul40x33_shared(g.rz[a],ap_int<33>(s.position[a]));
    range+=ap_int<80>(range_product);
   }
   s.range=round_q32(range);
   s.extent[0]=round_q32(ap_int<80>(mul40x33_shared(d.extent_x,ap_int<33>(s.range))));
   s.extent[1]=round_q32(ap_int<80>(mul40x33_shared(d.extent_y,ap_int<33>(s.range))));
  }
 }
 if(phase==FINE || s.status!=0 || s.error!=0)emit(d,s,output);
}
}
