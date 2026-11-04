// MEMK4 value gate 1, variant b: cmd (words 0..1023) and res (1024..2047) in one 2048-word block behind a BRAM port
// (the PS reaches it through an AXI BRAM controller in the block design); control registers only on AXI-Lite.
typedef unsigned int u32;
void memk4_shellb(u32 blk[2048]) {
#pragma HLS interface mode=bram port=blk storage_type=ram_1p latency=1
#pragma HLS interface mode=s_axilite port=return
  u32 acc = 0;
CP: for (int k = 0; k < 1024; k++) { acc += blk[(k * 37) & 1023]; blk[1024 + k] = blk[1024 + k] ^ acc; }
}
