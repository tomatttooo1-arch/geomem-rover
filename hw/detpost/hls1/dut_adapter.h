// testbench 与当前这一步顶层接口之间的适配（每一步改接口时只改这里）。
#pragma once
#include <string.h>
#include "detpost_k.h"
#include "dp_param_pack.h"
using namespace detpost;
static void dut_run(const int8_t* buf, const Config& cfg, Box* out, Status* st) {
  static dp_word_t inw[DETPOST_TB_BUF_BYTES / 16];
  static dp_word_t prm[dpk::PRM_WORDS];
  static dp_word_t ow[dpk::OUT_WORDS];
  for (int i = 0; i < DETPOST_TB_BUF_BYTES / 16; i++)
    for (int b = 0; b < 16; b++) inw[i].range(8 * b + 7, 8 * b) = (uint8_t)buf[16 * i + b];
  dp_pack_params(cfg, prm);
  for (int i = 0; i < dpk::OUT_WORDS; i++) ow[i] = 0;
  int thr = cfg.cls_thr_q < -128 ? -128 : (cfg.cls_thr_q > 128 ? 128 : cfg.cls_thr_q);  // 主机钳位（README）
  detpost_top(inw, prm, ow, cfg.nc, thr, cfg.iou_num, cfg.iou_den, cfg.max_det, cfg.grid_w[0], cfg.grid_w[1],
              cfg.grid_w[2], cfg.offset[0], cfg.offset[1], cfg.offset[2]);
  st->n_cand = (int)ow[0].range(31, 0); st->n_cand_kept = (int)ow[0].range(63, 32);
  st->overflow = (int)ow[0].range(95, 64); st->final_level = (int)(int32_t)(uint32_t)ow[0].range(127, 96);
  st->n_nms = (int)ow[1].range(31, 0); st->n_out = (int)ow[1].range(63, 32);
  for (int i = 0; i < st->n_out && i < MAX_DET_MAX; i++) {
    const dp_word_t& o = ow[dpk::OUT_HDR + i];
    out[i].x1 = (int16_t)(uint16_t)o.range(15, 0); out[i].y1 = (int16_t)(uint16_t)o.range(31, 16);
    out[i].x2 = (int16_t)(uint16_t)o.range(47, 32); out[i].y2 = (int16_t)(uint16_t)o.range(63, 48);
    out[i].cls = (uint8_t)o.range(71, 64); out[i].level = (int8_t)(uint8_t)o.range(79, 72);
    out[i].score = (uint16_t)o.range(95, 80); out[i].anchor = (uint16_t)o.range(111, 96);
    out[i].pad = (uint16_t)o.range(127, 112);
  }
}
