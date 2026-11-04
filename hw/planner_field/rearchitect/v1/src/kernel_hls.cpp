#include "kernel_hls.hpp"
#include <hls_stream.h>
namespace {
typedef ap_uint<24> Potential;
typedef ap_uint<9> Cell;
typedef ap_uint<13> Index;
const unsigned INF=0xffffffu;
static void load_input(const ap_uint<8> map[6400],hls::stream<ap_uint<8> >& pixels){
#pragma HLS INLINE off
  for(int i=0;i<6400;++i)pixels.write(map[i]);
}
static Cell encode_cost(ap_uint<8> raw){
#pragma HLS INLINE
  ap_uint<10> x=ap_uint<10>(raw)<<2;
  ap_uint<18> wide=x;
  ap_uint<18> product=(wide<<7)+(wide<<6)+(wide<<3)+(wide<<2)+wide;
  ap_uint<8> entry=ap_uint<8>(50)+ap_uint<8>(product>>10);
  Cell cell=entry;
  cell[8]=raw<253;
  return cell;
}
static Potential add(Potential p,ap_uint<12> weight){
#pragma HLS INLINE
  ap_uint<25> sum=ap_uint<25>(p)+ap_uint<25>(weight);
  return sum>=INF ? Potential(INF) : Potential(sum);
}
static bool scan_pass(Cell map[6400],Potential field[6400],Index source,bool Reverse){
#pragma HLS INLINE off
  // Coordinates below follow the sweep: west and north have already been updated.
  Potential previous[80];
  bool previous_walk[80];
  for(ap_uint<7> x=0;x<80;++x){previous[x]=INF;previous_walk[x]=false;}
  bool changed=false;
  row_loop: for(ap_uint<7> row=0;row<80;++row){
    ap_uint<7> physical_row=Reverse ? ap_uint<7>(79-row) : row;
    Index row_base=(Index(physical_row)<<6)+(Index(physical_row)<<4);
    if(Reverse)row_base+=79;
    Potential nw=INF,n=previous[0],ne=previous[1],west=INF;
    bool nw_walk=false,n_walk=previous_walk[0],ne_walk=previous_walk[1],west_walk=false;
    Cell current_cost=map[row_base];
    Cell east_cost=map[Reverse ? Index(row_base-1) : Index(row_base+1)];
    column_loop: for(ap_uint<7> col=0;col<80;++col){
#pragma HLS PIPELINE II=1
      const Index i=Reverse ? Index(row_base-col) : Index(row_base+col);
      // Pre-read two ahead before overwriting this column in the previous row.
      Potential future=col<78 ? previous[col+2] : Potential(INF);
      bool future_walk=col<78 ? previous_walk[col+2] : false;
      Cell future_cost=col<78 ? map[Reverse ? Index(i-2) : Index(i+2)] : Cell(0);
      const bool walk=current_cost[8];
      const bool east_walk=east_cost[8];
      ap_uint<8> entry=current_cost.range(7,0);
      ap_uint<12> wide_entry=entry;
      ap_uint<12> straight=(wide_entry<<3)+(wide_entry<<1);
      ap_uint<12> diagonal=(wide_entry<<4)-(wide_entry<<1);
      Potential old=field[i];
      Potential best=old;
      Potential a=(nw_walk && west_walk && n_walk) ? add(nw,diagonal) : Potential(INF);
      Potential b=n_walk ? add(n,straight) : Potential(INF);
      Potential c=(ne_walk && east_walk && n_walk) ? add(ne,diagonal) : Potential(INF);
      if(a<best)best=a;
      if(b<best)best=b;
      if(c<best)best=c;
      Potential w=west_walk ? add(west,straight) : Potential(INF);
      if(w<best)best=w;
      if(!walk || i==source)best=old;
      if(best<old)changed=true;
      field[i]=best;
      previous[col]=best;
      previous_walk[col]=walk;
      west=best;west_walk=walk;
      nw=n;n=ne;ne=future;
      nw_walk=n_walk;n_walk=ne_walk;ne_walk=future_walk;
      current_cost=east_cost;east_cost=future_cost;
    }
  }
  return changed;
}
static void compute(hls::stream<ap_uint<8> >& pixels,hls::stream<ap_uint<32> >& output,
    int sx,int sy,int gx,int gy,unsigned k_max){
#pragma HLS INLINE off
  Cell map[6400];Potential field[6400];ap_uint<8> flags[64];
  for(Index i=0;i<6400;++i){map[i]=encode_cost(pixels.read());field[i]=INF;}
  for(int i=0;i<64;++i)flags[i]=255;
  unsigned status=6;
  ap_uint<7> rounds=0;
  bool valid=false;
  bool args=k_max>0 && k_max<=64 && sx>=0 && sy>=0 && sx<80 && sy<80 && gx>=0 && gy>=0 && gx<80 && gy<80;
  if(args){
    ap_uint<7> xs=sx,ys=sy,xg=gx,yg=gy;
    const Index goal=(Index(yg)<<6)+(Index(yg)<<4)+Index(xg);
    if(sx==0 || sy==0 || sx==79 || sy==79)status=2;
    else if(!map[goal][8])status=3;
    else{
      const Index source=(Index(ys)<<6)+(Index(ys)<<4)+Index(xs);
      Cell source_cell=50;source_cell[8]=1;map[source]=source_cell;
      for(ap_uint<7> x=0;x<80;++x){map[x]=0;map[6320+x]=0;}
      for(ap_uint<7> y=0;y<80;++y){
        Index base=(Index(y)<<6)+(Index(y)<<4);
        map[base]=0;map[Index(base+79)]=0;
      }
      field[source]=0;status=7;
      for(ap_uint<7> r=0;r<64 && r<k_max;++r){
#pragma HLS LOOP_TRIPCOUNT min=1 max=64 avg=64
        bool changed=false;
        // One call site shares the scan datapath and row cache between directions.
        phase_loop: for(ap_uint<2> phase=0;phase<2;++phase){
          bool pass_changed=scan_pass(map,field,source,phase!=0);
          changed=changed || pass_changed;
        }
        flags[r]=changed ? 1 : 0;rounds=r+1;
        if(!changed){status=0;valid=true;break;}
      }
    }
  }
  for(int i=0;i<6400;++i)output.write(valid ? ap_uint<32>(field[i]) : ap_uint<32>(INF));
  for(int i=0;i<64;++i)output.write(ap_uint<32>(flags[i]));
  output.write(ap_uint<32>(status));output.write(ap_uint<32>(rounds));output.write(ap_uint<32>(valid));
}
static void store_output(hls::stream<ap_uint<32> >& input,ap_uint<32> field[6400],ap_uint<32> flags[64],
    ap_uint<32>& status,ap_uint<32>& rounds,ap_uint<32>& field_valid){
#pragma HLS INLINE off
  for(int i=0;i<6400;++i)field[i]=input.read();
  for(int i=0;i<64;++i)flags[i]=input.read();
  status=input.read();rounds=input.read();field_valid=input.read();
}
}
extern "C" void planner_field(const ap_uint<8> map[6400],ap_uint<32> field[6400],
    ap_uint<32> flags[64],int sx,int sy,int gx,int gy,unsigned k_max,
    ap_uint<32>& status,ap_uint<32>& rounds,ap_uint<32>& field_valid){
#pragma HLS INTERFACE m_axi port=map offset=slave bundle=map_mem depth=6400
#pragma HLS INTERFACE m_axi port=field offset=slave bundle=field_mem depth=6400
#pragma HLS INTERFACE m_axi port=flags offset=slave bundle=field_mem depth=64
#pragma HLS INTERFACE s_axilite port=map bundle=control offset=0x10
#pragma HLS INTERFACE s_axilite port=field bundle=control offset=0x1c
#pragma HLS INTERFACE s_axilite port=flags bundle=control offset=0x28
#pragma HLS INTERFACE s_axilite port=sx bundle=control offset=0x34
#pragma HLS INTERFACE s_axilite port=sy bundle=control offset=0x3c
#pragma HLS INTERFACE s_axilite port=gx bundle=control offset=0x44
#pragma HLS INTERFACE s_axilite port=gy bundle=control offset=0x4c
#pragma HLS INTERFACE s_axilite port=k_max bundle=control offset=0x54
#pragma HLS INTERFACE s_axilite port=status bundle=control offset=0x5c
#pragma HLS INTERFACE s_axilite port=rounds bundle=control offset=0x6c
#pragma HLS INTERFACE s_axilite port=field_valid bundle=control offset=0x7c
#pragma HLS INTERFACE s_axilite port=return bundle=control
#pragma HLS DATAFLOW
  hls::stream<ap_uint<8> > pixels;
  hls::stream<ap_uint<32> > output;
#pragma HLS STREAM variable=pixels depth=32
#pragma HLS STREAM variable=output depth=32
  load_input(map,pixels);
  compute(pixels,output,sx,sy,gx,gy,k_max);
  store_output(output,field,flags,status,rounds,field_valid);
}
