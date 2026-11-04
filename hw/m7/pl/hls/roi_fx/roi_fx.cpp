#include "roi_fx.hpp"
namespace {
using U32=ap_uint<32>;
using Stream=hls::stream<U32>;
static std::uint32_t validate(std::uint32_t w,std::uint32_t h,std::uint32_t b,
 const std::uint32_t en[32],const std::uint32_t xy[128],std::uint32_t minpix,
 const std::uint32_t f[32],const std::uint32_t k[32],std::uint32_t cookie,
 std::uint32_t vb,std::uint32_t ve,std::uint32_t ps,std::uint64_t vm,
 std::uint64_t fm,std::uint32_t ties,ap_uint<10>& yf,ap_uint<10>& yl,ap_uint<32>& active){
#pragma HLS INLINE
 yf=799;yl=0;active=0;
 if(!w||w>1280||!h||h>800)return 1;
 if(ps)return (ps==1||ps==3||ps==4)?ps:5;
 if(minpix<9||minpix>1024000)return 4;
 if(!b||b>32)return 3;
 std::uint32_t error=0;
 VALIDATE_BOX:for(unsigned i=0;i<32;++i){
#pragma HLS PIPELINE off
  if(i>=b)continue;
  const unsigned e=en[i];
  if(e>1){error=3;continue;}if(!e)continue;
  const auto x0=xy[4*i],y0=xy[4*i+1],x1=xy[4*i+2],y1=xy[4*i+3];
  if(x0>x1||y0>y1||x1>=w||y1>=h||x1-x0<3||y1-y0<3){error=3;continue;}
  if(f[i]<1||f[i]>65536||k[i]<1||k[i]>65536){if(!error)error=5;continue;}
  active[i]=1;if(y0<yf)yf=y0;if(y1>yl)yl=y1;
 }
 if(error)return error;
 // Exact binary64 midpoint thresholds for .7 and .9.
 if(cookie!=roi_fx::COOKIE||vb<1||vb>ve||ve>65536||ties>3||
 vm<0x59999999999996ULL||vm>((1ULL<<55)-2)||
 fm<0x73333333333332ULL||fm>((1ULL<<55)-2))return 5;
 return 0;
}
static void load_input(const U32* depth,std::uint32_t w,std::uint32_t h,std::uint32_t b,
 const std::uint32_t en[32],const std::uint32_t xy[128],const std::uint32_t id[32],
 std::uint32_t minpix,const std::uint32_t f[32],const std::uint32_t k[32],
 std::uint32_t cookie,std::uint32_t vb,std::uint32_t ve,std::uint32_t ps,
 std::uint64_t vm,std::uint64_t fm,std::uint32_t ties,Stream& out){
#pragma HLS INLINE off
 ap_uint<10> yf,yl;ap_uint<32> active;
 const auto st=validate(w,h,b,en,xy,minpix,f,k,cookie,vb,ve,ps,vm,fm,ties,yf,yl,active);
 out.write(st);if(st)return;
 out.write(w);out.write(h);out.write(b);out.write(minpix);out.write(vb);out.write(ve);
 out.write(vm);out.write(vm>>32);out.write(fm);out.write(fm>>32);out.write(ties);
 out.write(yf);out.write(yl);out.write(active);
 CONFIG:for(unsigned i=0;i<32;++i){
#pragma HLS PIPELINE off
  const bool on=active[i];
  out.write(i<b?id[i]:0);
  ap_uint<32> a=0,d=0;
  if(on){a.range(10,0)=xy[4*i];a.range(21,11)=xy[4*i+2];a[22]=1;
   d.range(9,0)=xy[4*i+1];d.range(19,10)=xy[4*i+3];}
  out.write(a);out.write(d);out.write(on?f[i]:1);out.write(on?k[i]:1);
 }
 // Contiguous global-index read. Starting odd halfword belongs to preceding row.
 const ap_uint<21> first=ap_uint<11>(w)*ap_uint<10>(yf);
 const ap_uint<21> end=ap_uint<11>(w)*(ap_uint<11>(yl)+1);
 const ap_uint<20> start_word=first>>1;
 const ap_uint<20> end_word=(end+1)>>1;
 const ap_uint<20> count=active ? ap_uint<20>(end_word-start_word):ap_uint<20>(0);
 READ_ROWS:for(ap_uint<21> i=0;i<count;++i){
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=0 max=512000 avg=72000
  out.write(depth[start_word+i]);
 }
}
static bool ratio_ge(ap_uint<20> a,ap_uint<20> b,ap_uint<55> m,bool tie){
#pragma HLS INLINE off
 ap_uint<48> p=0,q=0;const ap_uint<27> mh=m.range(54,28);const ap_uint<28> ml=m.range(27,0);
 RATIO20:for(int i=19;i>=0;--i){
#pragma HLS PIPELINE II=1
  ap_uint<48> pn=(p<<1)+ap_uint<48>(b[i]?mh:ap_uint<27>(0));
  ap_uint<48> qn=(q<<1)+ap_uint<48>(b[i]?ml:ap_uint<28>(0));
#pragma HLS BIND_OP variable=pn op=add impl=dsp latency=1
#pragma HLS BIND_OP variable=qn op=add impl=dsp latency=1
  p=pn;q=qn;
 }
 const ap_uint<48> h=p+(q>>28),lhs=ap_uint<48>(a)<<27;
 return lhs>h||(lhs==h&&q.range(27,0)==0&&tie);
}
// The single decode/fanout point is deliberately independent of ROI predicates.
// A future second consumer must receive this exact raw code/global-coordinate tuple.
static roi_fx::Pixel decode_pixel(Stream& input,U32& packed,bool& high,
 ap_uint<11> x,ap_uint<10> y,bool first){
#pragma HLS INLINE
 if(first||!high)packed=input.read();
 roi_fx::Pixel px;px.code=high?packed.range(31,16):packed.range(15,0);px.x=x;px.y=y;high=!high;return px;
}
static void compute(Stream& input,Stream& output){
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances=ratio_ge limit=1
 const auto st=input.read();output.write(st);if(st)return;
 const ap_uint<11> w=input.read();const ap_uint<10> h=input.read();
 const ap_uint<6> bcount=input.read();const ap_uint<20> minpix=input.read();
 const ap_uint<17> vb=input.read(),ve=input.read();
 ap_uint<64> vm=input.read();vm|=ap_uint<64>(input.read())<<32;
 ap_uint<64> fm=input.read();fm|=ap_uint<64>(input.read())<<32;
 const ap_uint<2> ties=input.read();const ap_uint<10> yf=input.read(),yl=input.read();
 const ap_uint<32> active=input.read();
 ap_uint<32> col[1281],row[801];
 ap_uint<34> keys[8][4];ap_uint<60> counts[8][4];
 ap_uint<32> ids[32];ap_uint<20> areas[32];
#pragma HLS BIND_STORAGE variable=col type=ram_s2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=row type=ram_s2p impl=bram latency=1
#pragma HLS ARRAY_PARTITION variable=keys complete dim=1
#pragma HLS ARRAY_PARTITION variable=counts complete dim=1
#pragma HLS BIND_STORAGE variable=keys type=ram_s2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=counts type=ram_s2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=ids type=ram_s2p impl=bram latency=1
#pragma HLS BIND_STORAGE variable=areas type=ram_s2p impl=bram latency=1
 const ap_uint<11> s=w>h?w:ap_uint<11>(h);
 CLEAR_COVER:for(ap_uint<12> i=0;i<=s;++i){
#pragma HLS PIPELINE II=1
  if(i<=w)col[i]=0;if(i<=h)row[i]=0;
 }
 CONFIG_BANK:for(unsigned b=0;b<32;++b){
#pragma HLS PIPELINE off
  const U32 id=input.read(),cx=input.read(),cy=input.read();
  const ap_uint<17> f=input.read(),k=input.read();
  const ap_uint<3> e=b&7;const ap_uint<2> slot=b>>3;
  ap_uint<34> key;key.range(16,0)=f;key.range(33,17)=k;
  keys[e][slot]=key;counts[e][slot]=0;ids[b]=id;
  const bool on=cx[22];const ap_uint<11> x0=cx.range(10,0),x1=cx.range(21,11);
  const ap_uint<10> y0=cy.range(9,0),y1=cy.range(19,10);
  const ap_uint<11> dx=on?ap_uint<11>(x1-x0+1):ap_uint<11>(0);
  const ap_uint<10> dy=on?ap_uint<10>(y1-y0+1):ap_uint<10>(0);
  ap_uint<21> area=dx*dy;
#pragma HLS BIND_OP variable=area op=mul impl=dsp latency=1
  areas[b]=area;
  if(on){const U32 bit=U32(1)<<b;
   col[x0]^=bit;col[x1+1]^=bit;row[y0]^=bit;row[y1+1]^=bit;}
 }
 U32 cr=0,rr=0;
 PREFIX:for(ap_uint<12> i=0;i<s;++i){
#pragma HLS PIPELINE II=1
  if(i<w){cr^=col[i];col[i]=cr;}if(i<h){rr^=row[i];row[i]=rr;}
 }
 const ap_uint<21> pixels=active?ap_uint<21>(w*(ap_uint<11>(yl)-yf+1)):ap_uint<21>(0);
 const ap_uint<23> micro=ap_uint<23>(pixels)<<2;
 ap_uint<11> x=0;ap_uint<10> y=yf;U32 packed=0,membership=0;
 ap_uint<16> code=0;bool high=(ap_uint<21>(w*yf)&1)!=0;bool valid=false;
 MICRO4:for(ap_uint<23> t=0;t<micro;++t){
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=0 max=4096000 avg=576000
#pragma HLS DEPENDENCE variable=counts inter RAW distance=4 true
  const ap_uint<2> slot=t.range(1,0);
  if(slot==0){const auto px=decode_pixel(input,packed,high,x,y,t==0);
   code=px.code;membership=col[px.x]&row[px.y];valid=(ap_uint<17>(code)>=vb&&ap_uint<17>(code)<ve);}
  ENGINES:for(unsigned e=0;e<8;++e){
#pragma HLS UNROLL
   const auto key=keys[e][slot];const auto old=counts[e][slot];
   const ap_uint<17> f=key.range(16,0),k=key.range(33,17);
   const bool hit=membership[unsigned(slot)*8+e]&&valid;
   ap_uint<48> fb=0;fb.range(19,0)=old.range(39,20);fb.range(43,24)=old.range(59,40);
   ap_uint<48> inc=0;inc[0]=hit&&ap_uint<17>(code)<f;inc[24]=hit&&ap_uint<17>(code)>=k;
   ap_uint<48> next_fb=fb+inc;
   ap_uint<48> next_valid=ap_uint<48>(old.range(19,0))+ap_uint<48>(hit);
#pragma HLS BIND_OP variable=next_fb op=add impl=dsp latency=1
#pragma HLS BIND_OP variable=next_valid op=add impl=dsp latency=1
   ap_uint<60> next;next.range(19,0)=next_valid.range(19,0);
   next.range(39,20)=next_fb.range(19,0);next.range(59,40)=next_fb.range(43,24);
   counts[e][slot]=next;
  }
  if(slot==3){if(x+1==w){x=0;++y;}else ++x;}
 }
 output.write(bcount);
 RESULTS:for(unsigned b=0;b<bcount;++b){
#pragma HLS PIPELINE off
  const auto c=counts[b&7][b>>3];const ap_uint<20> v=c.range(19,0),f=c.range(39,20),k=c.range(59,40);
  const ap_uint<20> area=areas[b];const bool on=active[b];
  bool vr=false,fr=false,br=false;
  COMPARE3:for(unsigned phase=0;phase<3;++phase){
#pragma HLS PIPELINE off
   const ap_uint<20> a=phase==0?v:phase==1?f:k;
   const ap_uint<20> denominator=phase==0?area:v;
   const ap_uint<55> midpoint=phase==0?ap_uint<55>(vm.range(54,0)):
    phase==1?ap_uint<55>((ap_uint<55>(1)<<54)-1):ap_uint<55>(fm.range(54,0));
   const bool tie=phase==0?bool(ties[0]):phase==1?true:bool(ties[1]);
   const bool ge=ratio_ge(a,denominator,midpoint,tie);
   if(phase==0)vr=ge;else if(phase==1)fr=ge;else br=ge;
  }
  const unsigned verdict=on&&v>=minpix&&vr?(fr?3:br?2:0):0;
  output.write(ids[b]);output.write(on);output.write(area);output.write(v);output.write(f);output.write(k);output.write(verdict);
 }
}
static void store_output(Stream& input,std::uint32_t result[224],std::uint32_t& status){
#pragma HLS INLINE off
 const auto st=input.read();status=st.to_uint();if(st)return;
 const ap_uint<6> b=input.read();
 WRITE_RESULT:for(unsigned i=0;i<unsigned(b)*7;++i){
#pragma HLS PIPELINE II=1
  result[i]=input.read();
 }
}
static void run_pipeline(
 const ap_uint<32>* depth,std::uint32_t width,std::uint32_t height,
 std::uint32_t box_count,const std::uint32_t box_enabled[32],
 const std::uint32_t box_xyxy[128],const std::uint32_t object_id[32],
 std::uint32_t min_pixels,const std::uint32_t front_cut[32],
 const std::uint32_t behind_cut[32],std::uint32_t fx_cookie,
 std::uint32_t v_begin,std::uint32_t v_end,std::uint32_t ps_status,
 std::uint64_t valid_ratio_m,std::uint64_t free_ratio_m,
 std::uint32_t ratio_ties,std::uint32_t result[224],std::uint32_t& status){
#pragma HLS INLINE off
#pragma HLS DATAFLOW
 Stream input("roi_input"),output("roi_output");
#pragma HLS STREAM variable=input depth=64
#pragma HLS STREAM variable=output depth=16
#pragma HLS BIND_STORAGE variable=input type=fifo impl=bram
#pragma HLS BIND_STORAGE variable=output type=fifo impl=bram
 load_input(depth,width,height,box_count,box_enabled,box_xyxy,object_id,min_pixels,front_cut,behind_cut,fx_cookie,v_begin,v_end,ps_status,valid_ratio_m,free_ratio_m,ratio_ties,input);
 compute(input,output);
 store_output(output,result,status);
}
}
extern "C" std::uint32_t fpga_m7_object_roi_fx_axi(
 const ap_uint<32>* depth,std::uint32_t width,std::uint32_t height,
 std::uint32_t box_count,const std::uint32_t box_enabled[32],
 const std::uint32_t box_xyxy[128],const std::uint32_t object_id[32],
 std::uint32_t min_pixels,const std::uint32_t front_cut[32],
 const std::uint32_t behind_cut[32],std::uint32_t fx_cookie,
 std::uint32_t v_begin,std::uint32_t v_end,std::uint32_t ps_status,
 std::uint64_t valid_ratio_m,std::uint64_t free_ratio_m,
 std::uint32_t ratio_ties,std::uint32_t result[224]){
#pragma HLS INTERFACE m_axi port=depth offset=slave bundle=gmem depth=512000 max_read_burst_length=64 num_read_outstanding=2 max_write_burst_length=2 num_write_outstanding=1
#pragma HLS INTERFACE s_axilite port=depth bundle=control offset=0x018
#pragma HLS INTERFACE s_axilite port=width bundle=control offset=0x024
#pragma HLS INTERFACE s_axilite port=height bundle=control offset=0x02c
#pragma HLS INTERFACE s_axilite port=box_count bundle=control offset=0x034
#pragma HLS INTERFACE s_axilite port=min_pixels bundle=control offset=0x048
#pragma HLS INTERFACE s_axilite port=box_enabled bundle=control offset=0x080
#pragma HLS INTERFACE s_axilite port=box_xyxy bundle=control offset=0x200
#pragma HLS INTERFACE s_axilite port=object_id bundle=control offset=0x500
#pragma HLS INTERFACE s_axilite port=result bundle=control offset=0x800
#pragma HLS INTERFACE s_axilite port=front_cut bundle=control offset=0xc00
#pragma HLS INTERFACE s_axilite port=behind_cut bundle=control offset=0xc80
#pragma HLS INTERFACE s_axilite port=fx_cookie bundle=control offset=0xd00
#pragma HLS INTERFACE s_axilite port=v_begin bundle=control offset=0xd08
#pragma HLS INTERFACE s_axilite port=v_end bundle=control offset=0xd10
#pragma HLS INTERFACE s_axilite port=ps_status bundle=control offset=0xd18
#pragma HLS INTERFACE s_axilite port=valid_ratio_m bundle=control offset=0xd20
#pragma HLS INTERFACE s_axilite port=free_ratio_m bundle=control offset=0xd2c
#pragma HLS INTERFACE s_axilite port=ratio_ties bundle=control offset=0xd38
#pragma HLS INTERFACE s_axilite port=return bundle=control
 std::uint32_t status;
 run_pipeline(depth,width,height,box_count,box_enabled,box_xyxy,object_id,min_pixels,front_cut,behind_cut,fx_cookie,v_begin,v_end,ps_status,valid_ratio_m,free_ratio_m,ratio_ties,result,status);
 return status;
}
