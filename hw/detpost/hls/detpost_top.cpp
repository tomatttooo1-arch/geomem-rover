// 只为拿资源/II 量级的 csynth 外壳：把参考模型 detpost_ref 原样包成顶层（未做任何 HLS 优化，
// 只加接口 pragma）。寄存器口：标量与查表；数据口：读 DPU 输出缓冲、写框表。
#include "../detpost.h"

using namespace detpost;

void detpost_top(const int8_t* in, Box* out, int nc, int cls_thr_q, int iou_num, int iou_den, int max_det,
                 int off80, int off40, int off20, const uint16_t exp_lut[LUT_N], const uint16_t sig_lut[LUT_N],
                 const int8_t cls_min_q[NC_MAX], const uint8_t cls_enable[NC_MAX], int* n_out, int* n_cand,
                 int* overflow) {
#pragma HLS INTERFACE mode=m_axi port=in offset=slave bundle=gmem0 depth=814800 max_widen_bitwidth=128
#pragma HLS INTERFACE mode=m_axi port=out offset=slave bundle=gmem1 depth=128
#pragma HLS INTERFACE mode=s_axilite port=nc
#pragma HLS INTERFACE mode=s_axilite port=cls_thr_q
#pragma HLS INTERFACE mode=s_axilite port=iou_num
#pragma HLS INTERFACE mode=s_axilite port=iou_den
#pragma HLS INTERFACE mode=s_axilite port=max_det
#pragma HLS INTERFACE mode=s_axilite port=off80
#pragma HLS INTERFACE mode=s_axilite port=off40
#pragma HLS INTERFACE mode=s_axilite port=off20
#pragma HLS INTERFACE mode=s_axilite port=exp_lut
#pragma HLS INTERFACE mode=s_axilite port=sig_lut
#pragma HLS INTERFACE mode=s_axilite port=cls_min_q
#pragma HLS INTERFACE mode=s_axilite port=cls_enable
#pragma HLS INTERFACE mode=s_axilite port=n_out
#pragma HLS INTERFACE mode=s_axilite port=n_cand
#pragma HLS INTERFACE mode=s_axilite port=overflow
#pragma HLS INTERFACE mode=s_axilite port=return
  Config cfg;
  cfg.nc = nc;
  const int g[3] = {80, 40, 20}, sd[3] = {8, 16, 32};
  for (int l = 0; l < 3; l++) { cfg.grid_w[l] = g[l]; cfg.grid_h[l] = g[l]; cfg.stride[l] = sd[l]; }
  cfg.offset[0] = off80; cfg.offset[1] = off40; cfg.offset[2] = off20;
  cfg.cls_thr_q = cls_thr_q; cfg.iou_num = iou_num; cfg.iou_den = iou_den; cfg.max_det = max_det;
  for (int i = 0; i < LUT_N; i++) { cfg.exp_lut[i] = exp_lut[i]; cfg.sig_lut[i] = sig_lut[i]; }
  for (int c = 0; c < NC_MAX; c++) { cfg.cls_min_q[c] = cls_min_q[c]; cfg.cls_enable[c] = cls_enable[c]; }
  Status st;
  *n_out = detpost_ref(in, cfg, out, &st);
  *n_cand = st.n_cand;
  *overflow = st.overflow;
}
