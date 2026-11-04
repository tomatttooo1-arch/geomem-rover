// MEMK4 value gate 1: the bare shell (m_axi 32-bit copy-in / copy-out into core BRAM + control registers).
// The "compute" is one pass that keeps both BRAMs alive; it is not the core.
typedef unsigned int u32;
static u32 RES[1024];
void memk4_shella(const u32 *cmd_g, u32 *res_g) {
#pragma HLS interface mode=m_axi port=cmd_g bundle=gmem offset=slave depth=1024 max_widen_bitwidth=32 num_read_outstanding=1 num_write_outstanding=1 max_read_burst_length=16 max_write_burst_length=16 latency=32
#pragma HLS interface mode=m_axi port=res_g bundle=gmem offset=slave depth=1024 max_widen_bitwidth=32 num_read_outstanding=1 num_write_outstanding=1 max_read_burst_length=16 max_write_burst_length=16 latency=32
#pragma HLS interface mode=s_axilite port=return
  u32 CMD[1024];
#pragma HLS bind_storage variable=CMD type=ram_2p impl=bram
#pragma HLS bind_storage variable=RES type=ram_2p impl=bram
CI: for (int k = 0; k < 1024; k++) {
#pragma HLS pipeline II=1
    CMD[k] = cmd_g[k];
  }
  u32 acc = 0;
CP: for (int k = 0; k < 1024; k++) { acc += CMD[(k * 37) & 1023]; RES[k] = RES[k] ^ acc; }
CO: for (int k = 0; k < 1024; k++) {
#pragma HLS pipeline II=1
    res_g[k] = RES[k];
  }
}
