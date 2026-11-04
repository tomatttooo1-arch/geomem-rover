// 主机侧工具：阈值换算、查表生成、默认配置（不参与综合）。
#include <math.h>
#include <string.h>
#include "detpost.h"

namespace detpost {

int conf_to_q(double conf, int fp) {
  // sigmoid(q*s) > conf  <=>  q*s > logit(conf)  <=>  q > logit(conf)/s
  const double s = ldexp(1.0, -fp);
  const double L = log(conf / (1.0 - conf)) / s;
  int q = (int)floor(L) + 1;
  // 与浮点参考（float32 sigmoid 后与 conf 比较）在边界上逐一核对，防 floor 误差
  while (q > -128 && 1.0f / (1.0f + expf(-(float)((q - 1) * s))) > (float)conf) q--;
  while (q < 128 && !(1.0f / (1.0f + expf(-(float)(q * s))) > (float)conf)) q++;
  return q;
}

void make_luts(int fp, Config* cfg) {
  const double s = ldexp(1.0, -fp);
  for (int d = 0; d < LUT_N; d++) {
    double v = floor(32768.0 * exp(-d * s) + 0.5);
    cfg->exp_lut[d] = (uint16_t)(v > 65535 ? 65535 : v);
  }
  for (int i = 0; i < LUT_N; i++) {
    double v = floor(65535.0 / (1.0 + exp(-(i - 128) * s)) + 0.5);
    cfg->sig_lut[i] = (uint16_t)v;
  }
}

void default_config(Config* cfg, int nc, int fp, double conf, int iou_num, int iou_den, int max_det) {
  memset(cfg, 0, sizeof(*cfg));
  cfg->nc = nc;
  const int g[3] = {80, 40, 20}, sd[3] = {8, 16, 32};
  const int ch = 4 * REG_MAX + nc;
  // 本工程 xmodel 的 DDR 排列：20x20 在最前，40x40 次之，80x80 最后（读自 compiled xmodel 的 ddr_addr）
  const uint32_t sz20 = 20 * 20 * ch, sz40 = 40 * 40 * ch;
  const uint32_t off[3] = {sz20 + sz40, sz20, 0};
  for (int l = 0; l < 3; l++) {
    cfg->grid_w[l] = cfg->grid_h[l] = g[l];
    cfg->stride[l] = sd[l];
    cfg->offset[l] = off[l];
  }
  cfg->cls_thr_q = conf_to_q(conf, fp);
  cfg->iou_num = iou_num;
  cfg->iou_den = iou_den;
  cfg->max_det = max_det;
  make_luts(fp, cfg);
  for (int c = 0; c < NC_MAX; c++) { cfg->cls_min_q[c] = -128; cfg->cls_enable[c] = 1; }
}

int score_floor_to_q(double floor, int fp) {
  const double s = ldexp(1.0, -fp);
  for (int q = -128; q < 128; q++)
    if (1.0f / (1.0f + expf(-(float)(q * s))) >= (float)floor) return q;
  return 128;
}

void recog_config(Config* cfg, int nc, int fp, double floor, const double* cls_thr, int iou_num, int iou_den,
                  int max_det, const uint32_t off[N_LEVELS], const int gw[N_LEVELS], const int gh[N_LEVELS]) {
  default_config(cfg, nc, fp, floor, iou_num, iou_den, max_det);
  int qmin = 128;
  for (int c = 0; c < nc; c++) {
    const int q = score_floor_to_q(cls_thr[c], fp);
    if (q > 127) { cfg->cls_enable[c] = 0; cfg->cls_min_q[c] = 127; }
    else cfg->cls_min_q[c] = (int8_t)q;
    if (q < qmin) qmin = q;
  }
  if (qmin > cfg->cls_thr_q) cfg->cls_thr_q = qmin > 127 ? 127 : qmin;
  for (int l = 0; l < N_LEVELS; l++) {
    cfg->offset[l] = off[l];
    cfg->grid_w[l] = gw[l];
    cfg->grid_h[l] = gh[l];
    cfg->stride[l] = 8 << l;   // P3/P4/P5 = stride 8/16/32 (YOLOv8)
  }
}

}  // namespace detpost
