// Development phase engine. This is not the final merged evidence/observation
// IP: CSR arbitration, shared first pass and DDR slow path are integrated later.
#include "obs_types.hpp"
namespace roi_obs {
namespace {
void load_input(const ap_uint<32>* memory,
    ap_uint<64> depth_base, ap_uint<64> rgb_base, ap_uint<11> frame_width,
    const Descriptor& d, ap_uint<2> phase, hls::stream<ap_uint<64>>& pixels) {
#pragma HLS INLINE off
    ap_uint<32> tile[84];
#pragma HLS BIND_STORAGE variable=tile type=ram_2p impl=bram
    const ap_uint<11> x0=d.x0,x1=d.x1;
    const ap_uint<10> y0=d.y0,y1=d.y1;
    ap_uint<21> row_offset=0;
    // Fixed ten-step Horner product; no64bit multiplies, no extraDSP.
    for(ap_uint<4> step=0;step<10;++step) {
#pragma HLS LOOP_FLATTEN off
        ap_uint<4> bit=ap_uint<4>(9-step);
        row_offset=(row_offset<<1)+(y0[bit]?ap_uint<21>(frame_width):ap_uint<21>(0));
    }
    row_offset+=ap_uint<21>(x0);
    for(ap_uint<10> y=y0;y<y1;++y) {
#pragma HLS LOOP_FLATTEN off
        ap_uint<21> tile_offset=row_offset;
        for(ap_uint<11> x=x0;x<x1;x+=ap_uint<11>(64)) {
#pragma HLS LOOP_FLATTEN off
            const ap_uint<11> remaining=x1-x;
            const ap_uint<7> count=remaining>64?ap_uint<7>(64):ap_uint<7>(remaining);
            // All geometric offsets fit22 bits, including RGB3*N.
            const ap_uint<22> wide_offset=tile_offset;
            const ap_uint<22> depth_offset=wide_offset<<1;
            const ap_uint<22> rgb_offset=(wide_offset<<1)+wide_offset;
            const ap_uint<64> depth_byte=depth_base+ap_uint<64>(depth_offset);
            const ap_uint<64> rgb_byte=rgb_base+ap_uint<64>(rgb_offset);
            const ap_uint<2> depth_lane=depth_byte.range(1,0);
            const ap_uint<2> rgb_lane=rgb_byte.range(1,0);
            // Sums up to133/198 fit8 bits; quotients up to33/49 fit6.
            const ap_uint<8> depth_span=ap_uint<8>(depth_lane)+(ap_uint<8>(count)<<1)+3;
            const ap_uint<8> rgb_span=ap_uint<8>(rgb_lane)+(ap_uint<8>(count)<<1)+count+3;
            const ap_uint<6> nd=depth_span>>2;
            const ap_uint<6> nr=phase==COARSE?ap_uint<6>(rgb_span>>2):ap_uint<6>(0);
            ap_uint<2> transfer=0;
            while(transfer<2) {
#pragma HLS LOOP_FLATTEN off
                const ap_uint<6> n=transfer==0?nd:nr;
                const ap_uint<64> word_base=transfer==0?(depth_byte>>2):(rgb_byte>>2);
                const ap_uint<7> target=transfer==0?ap_uint<7>(0):ap_uint<7>(34);
                for(ap_uint<7> index=0;index<n;++index) {
#pragma HLS PIPELINE II=1
                    // Same sole full-width external read callsite.
                    tile[target+index]=memory[word_base+ap_uint<64>(index)];
                }
                ++transfer;
            }
            for(ap_uint<7> i=0;i<count;++i) {
#pragma HLS PIPELINE II=1
                const ap_uint<8> ii=i;
                const ap_uint<8> db=ap_uint<8>(depth_lane)+(ii<<1);
                const ap_uint<8> rb=ap_uint<8>(rgb_lane)+(ii<<1)+ii;
                const ap_uint<7> da=db>>2,ra=ap_uint<7>(34)+(rb>>2);
                // db&3 remains0 or2 forhalfword aligneddepth; exact oldlane extraction.
                ap_uint<32> dw=tile[da];
                const ap_uint<5> ds=ap_uint<5>(db.range(1,0))<<3;
                ap_uint<64> pix=ap_uint<16>(dw>>ds);
                if(phase==COARSE) {
                    ap_uint<64> pair=tile[ra];
                    // Deliberately preserves unconditional secondword read,
                    // including finalpixel cases wherehighword isnotneeded.
                    pair.range(63,32)=tile[ra+ap_uint<7>(1)];
                    const ap_uint<5> rs=ap_uint<5>(rb.range(1,0))<<3;
                    pix.range(39,16)=ap_uint<24>(pair>>rs);
                }
                pixels.write(pix);
            }
            tile_offset+=ap_uint<21>(64);
        }
        row_offset+=ap_uint<21>(frame_width);
    }
}
void compute(ap_uint<2> phase,const Descriptor& d,const Geometry& g,const Profile& cfg,
 hls::stream<ap_uint<64>>& input,BoxState& state,hls::stream<ap_uint<32>>& result,
 hls::stream<ap_uint<2>>& words){
 #pragma HLS INLINE off
 obs_compute(phase,d,g,cfg,input,state,result);
 words.write(phase==FINE||state.status||state.error?1:0);
}
void store_output(ap_uint<32>* memory,ap_uint<64> output_base,hls::stream<ap_uint<32>>& input,hls::stream<ap_uint<2>>& words){
 #pragma HLS INLINE off
 // The record may fill its FIFO before compute sends the count; 32-word
 // capacity is explicit to avoid a hidden dataflow deadlock.
 ap_uint<2> count=words.read();
 for(unsigned i=0;i<unsigned(count)*32;++i){
  #pragma HLS PIPELINE II=1
  memory[(output_base>>2)+i]=input.read();
 }
}
void run_phase(ap_uint<32>* memory,ap_uint<64> depth_base,ap_uint<64> rgb_base,ap_uint<64> output_base,
 ap_uint<11> frame_width,ap_uint<2> phase,const Descriptor& d,const Geometry& g,const Profile& cfg,BoxState& state){
 #pragma HLS INLINE off
 #pragma HLS DATAFLOW
 hls::stream<ap_uint<64>> input;
 hls::stream<ap_uint<32>> result;hls::stream<ap_uint<2>> words;
 #pragma HLS STREAM variable=input depth=16
 #pragma HLS STREAM variable=result depth=32
 #pragma HLS STREAM variable=words depth=2
 load_input(memory,depth_base,rgb_base,frame_width,d,phase,input);
 compute(phase,d,g,cfg,input,state,result,words);
 store_output(memory,output_base,result,words);
}
}
}
// Standalone engineering entry for inspecting the observation phase engine.
// Caller invokes 0..3 sequentially for one box; state/work RAM stay in this IP.
// NOT a released product ABI. Whole-IP resource/timing claims are prohibited.
extern "C" void fpga_m7_roi_obs_phase(ap_uint<32>* memory,const ap_uint<32> params[914],
 ap_uint<5> box,ap_uint<2> phase,ap_uint<64> depth_base,ap_uint<64> rgb_base,
 ap_uint<64> output_base,ap_uint<32>& status){
 #pragma HLS INTERFACE m_axi port=memory bundle=gmem offset=off depth=1048576 max_read_burst_length=64 max_write_burst_length=32 num_read_outstanding=2 num_write_outstanding=2 max_widen_bitwidth=32
 #pragma HLS INTERFACE ap_memory port=params storage_type=ram_1p latency=1
 #pragma HLS INTERFACE ap_none port=box
 #pragma HLS INTERFACE ap_none port=phase
 #pragma HLS INTERFACE ap_none port=depth_base
 #pragma HLS INTERFACE ap_none port=rgb_base
 #pragma HLS INTERFACE ap_none port=output_base
 #pragma HLS INTERFACE ap_vld port=status
 #pragma HLS INTERFACE ap_ctrl_hs port=return
 static roi_obs::BoxState state;
 roi_obs::Descriptor d;roi_obs::Geometry g;roi_obs::Profile cfg;
 unsigned base=64+unsigned(box)*8;
 d.id=params[base];d.x0=params[base+1];d.y0=params[base+2];d.x1=params[base+3];d.y1=params[base+4];d.flags=params[base+6];d.area=params[base+7];
 ap_uint<64> extx=params[320+4*unsigned(box)],exty=params[322+4*unsigned(box)];extx.range(63,32)=params[321+4*unsigned(box)];exty.range(63,32)=params[323+4*unsigned(box)];d.extent_x=extx;d.extent_y=exty;
 for(unsigned a=0;a<12;++a){ap_uint<64> v=params[16+2*a];v.range(63,32)=params[17+2*a];if(a<9)g.a[a]=v;else g.rz[a-9]=v;}
 cfg.min_pixels=params[6];ap_uint<64> m=params[7];m.range(63,32)=params[8];cfg.valid_m=m;cfg.valid_tie=params[9];
 for(unsigned a=0;a<201;++a){cfg.lower[a]=params[512+2*a];cfg.upper[a]=params[513+2*a];}
 ap_uint<11> width=params[3];
 roi_obs::run_phase(memory,depth_base,rgb_base,output_base,width,phase,d,g,cfg,state);
 status=state.error;
}
