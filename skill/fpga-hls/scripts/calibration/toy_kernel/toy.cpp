#include <ap_int.h>
#define TOY_N 64
void toy(const ap_uint<8> in[TOY_N], ap_uint<16> out[TOY_N], ap_int<12> bias) {
  for (int i = 0; i < TOY_N; i++) {
#pragma HLS PIPELINE II=1
#if TOY_FAULT == 2
    ap_uint<15> v = in[i] * in[i] + bias;   /* fault: one bit narrower */
#else
    ap_uint<16> v = in[i] * in[i] + bias;
#endif
#if TOY_FAULT == 1
    if (i == 37) v = v + 1;                 /* fault: one element +1 */
#endif
    out[i] = v;
  }
}
