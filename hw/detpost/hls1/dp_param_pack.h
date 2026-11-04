// 主机侧：把 detpost::Config 的查表与类别项打包成核的 DDR 参数块（69 个 128 位字，布局见 detpost_k.h）。
// testbench 与板上驱动共用这一份打包规则。不参与综合。
#pragma once
#include <ap_int.h>
#include "../detpost.h"

static inline void dp_pack_params(const detpost::Config& cfg, ap_uint<128>* prm /* [69] */) {
  for (int w = 0; w < 69; w++) prm[w] = 0;
  for (int i = 0; i < 256; i++) {
    prm[i / 8].range((i % 8) * 16 + 15, (i % 8) * 16) = cfg.exp_lut[i];
    prm[32 + i / 8].range((i % 8) * 16 + 15, (i % 8) * 16) = cfg.sig_lut[i];
  }
  for (int c = 0; c < detpost::NC_MAX; c++) {
    const uint16_t e = (uint16_t)((uint8_t)cfg.cls_min_q[c]) | (uint16_t)(cfg.cls_enable[c] ? 0x100 : 0);
    prm[64 + c / 8].range((c % 8) * 16 + 15, (c % 8) * 16) = e;
  }
}
