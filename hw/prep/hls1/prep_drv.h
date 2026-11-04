/* PREP1 驱动侧（A53）参数块生成：缩放系数表、量化表、色彩系数。testbench 用它，也是板上驱动的原型。
 * 系数算法与 PREP0/model/prep_model.c 的 coefs()/prep_tables()/make_lut() 逐行相同（cv2 resize.cpp 8U INTER_LINEAR）。
 * 编译须 -ffp-contract=off（float32 的乘加不得被融合成 FMA，否则个别系数差 1）。 */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#define PREP_DRV_COEF_SCALE 2048
#define PREP_DRV_HDR_WORDS 3
#define PREP_DRV_PRM_WORDS (PREP_DRV_HDR_WORDS + 640)   /* 643 字 = 10,288 字节 */

typedef struct {
  int w0, h0, stride, fmt, ups, bypass;   /* fmt: 0 RGB 1 BGR 2 YUYV 3 UYVY */
  int tw, th, ow, oh, top, left, fp;
  int k[6];                               /* KY, YOFF, KRV, KGU, KGV, KBU */
} prep_cfg_t;

static const int PREP_K_JFIF[6] = {65536, 0, 91881, -22554, -46802, 116130};      /* libjpeg jdcolor.c，全范围 */
static const int PREP_K_BT601L[6] = {76309, 16, 104597, -25675, -53279, 132201};  /* BT.601 有限范围 */

static void prep_drv_coefs(int dst_n, int src_n, int *ofs, int *c0, int *c1) {
  double scale = 1.0 / ((double)dst_n / (double)src_n);
  for (int d = 0; d < dst_n; d++) {
    float f = (float)((d + 0.5) * scale - 0.5);
    int s = (int)floorf(f);
    f = f - (float)s;
    if (s < 0) { f = 0.f; s = 0; }
    if (s >= src_n - 1) { f = 0.f; s = src_n - 1; }
    float p0 = (1.f - f) * (float)PREP_DRV_COEF_SCALE;
    float p1 = f * (float)PREP_DRV_COEF_SCALE;
    c0[d] = (int)lrintf(p0);
    c1[d] = (int)lrintf(p1);
    ofs[d] = s;
  }
}

/* 一个方向的表：返回 0 可流式；-1 不满足（放大、右夹取、x1 不严格递增）。ent = x1 | c0<<12 | c1<<24 */
static int prep_drv_axis(int src_n, int dst_n, uint64_t *ent) {
  static int ofs[2048], c0[2048], c1[2048];
  if (dst_n > src_n) return -1;
  prep_drv_coefs(dst_n, src_n, ofs, c0, c1);
  int last = -1;
  for (int d = 0; d < dst_n; d++) {
    int x0 = ofs[d], x1 = x0 + 1 < src_n ? x0 + 1 : src_n - 1;
    if (x1 != x0 + 1 || x1 <= last) return -1;
    last = x1;
    ent[d] = (uint64_t)x1 | ((uint64_t)c0[d] << 12) | ((uint64_t)c1[d] << 24);
  }
  return 0;
}

static uint8_t prep_drv_padq(int fp);

static void prep_drv_put(uint8_t *blk, int word, int lo, int nbits, uint64_t v) {   /* 小端位域写入 */
  for (int b = 0; b < nbits; b++)
    if ((v >> b) & 1) { int bit = word * 128 + lo + b; blk[bit >> 3] |= (uint8_t)(1u << (bit & 7)); }
}

/* 几何合法性（驱动必须查；核只复查魔数、OW/OH 非 0 且不大于源、装载字数）。返回 0 合法。
 * 驱动约束：源行步长与张量字节数都是 16 的倍数（核不做末拍掩码）；源/张量/参数块基址 16 字节对齐（张量建议 4 KB）。 */
static int prep_drv_check(const prep_cfg_t *c) {
  int bpp = c->fmt >= 2 ? 2 : 3;
  if (c->w0 < 4 || c->w0 > 2048 || c->h0 < 2 || c->h0 > 2048) return -1;
  if (c->ow < 16 || c->ow > 640 || c->oh < 1 || c->oh > 640 || c->tw > 640 || c->th > 640) return -1;
  if (c->ow > c->w0 || c->oh > c->h0) return -1;
  if (c->left + c->ow > c->tw || c->top + c->oh > c->th) return -1;
  if ((c->tw * c->th * 3) % 16) return -1;
  if (c->stride % 16 || c->stride < c->w0 * bpp || c->stride / 16 > 4095) return -1;
  if (c->fmt >= 2 && (c->w0 & 1)) return -1;
  if (c->bypass && (c->ow != c->w0 || c->oh != c->h0)) return -1;
  return 0;
}

/* 帧头（字 0..2）：几何与各级循环次数由驱动算好，核里不再做这些乘法。不查合法性（testbench 用它构造非法帧头）。 */
static void prep_drv_header(const prep_cfg_t *c, uint8_t *blk) {
  int bpp = c->fmt >= 2 ? 2 : 3;
  long rowb = (long)c->w0 * bpp;
  long sw = c->stride / 16, nwr = (rowb + 15) / 16;
  long tend = c->fmt >= 2 ? c->w0 + 2 : c->w0;
  long nload = c->ow > c->oh ? c->ow : c->oh;   /* >= 16 > 6（色彩系数在字 3..8） */
  long right = c->tw - c->left - c->ow, bottom = c->th - c->top - c->oh;
  memset(blk, 0, PREP_DRV_HDR_WORDS * 16);
  prep_drv_put(blk, 0, 0, 16, 0x5052);
  prep_drv_put(blk, 0, 16, 12, c->w0);
  prep_drv_put(blk, 0, 28, 12, c->h0);
  prep_drv_put(blk, 0, 40, 10, c->ow);
  prep_drv_put(blk, 0, 50, 10, c->oh);
  prep_drv_put(blk, 0, 60, 2, c->fmt);
  prep_drv_put(blk, 0, 62, 1, c->ups);
  prep_drv_put(blk, 0, 63, 1, c->bypass);
  prep_drv_put(blk, 0, 64, 8, prep_drv_padq(c->fp));
  prep_drv_put(blk, 0, 72, 12, sw);
  prep_drv_put(blk, 0, 84, 12, nwr);
  prep_drv_put(blk, 0, 96, 17, (long)c->tw * c->th * 3 / 16);
  prep_drv_put(blk, 0, 113, 10, nload);
  prep_drv_put(blk, 1, 0, 23, (long)c->h0 * sw);
  prep_drv_put(blk, 1, 23, 23, (long)c->w0 * c->h0);
  prep_drv_put(blk, 1, 46, 23, (long)c->h0 * tend);
  prep_drv_put(blk, 1, 69, 21, (long)c->h0 * c->ow);
  prep_drv_put(blk, 1, 90, 21, ((long)c->top * c->tw + c->left) * 3);
  prep_drv_put(blk, 2, 0, 21, (long)(c->tw - c->ow) * 3);
  prep_drv_put(blk, 2, 21, 21, ((long)bottom * c->tw + right) * 3);
  prep_drv_put(blk, 2, 42, 24, 32897UL << (c->fp + 1));   /* 量化 K：q = min(127, (x·K + C) >> 24) = round(x·2^fp/255) */
  prep_drv_put(blk, 2, 66, 24, 255UL * 32897);             /* 量化 C */
}

/* 生成整个参数块（643 个 16 字节字，小端：字 d 的字节 0..15 = 位 [7:0]..[127:120]）：帧头 + 表。
 * perturb=1：第一个非零水平 c1 +1（阳性对照用）。返回 0 成功，-1 几何非法或不能流式（此时块内容无意义，不得启动核）。 */
static int prep_drv_params(const prep_cfg_t *c, uint8_t *blk, int perturb) {
  static uint64_t xe[640], ye[640];
  memset(blk, 0, PREP_DRV_PRM_WORDS * 16);
  memset(xe, 0, sizeof xe);
  memset(ye, 0, sizeof ye);
  if (prep_drv_check(c)) return -1;
  if (!c->bypass) {
    if (prep_drv_axis(c->w0, c->ow, xe) || prep_drv_axis(c->h0, c->oh, ye)) return -1;
    if (perturb)
      for (int d = 0; d < c->ow; d++)
        if ((xe[d] >> 24) & 0xfff) { xe[d] += (uint64_t)1 << 24; break; }
  }
  prep_drv_header(c, blk);
  for (int d = 0; d < 640; d++) {
    /* 表项位域：[35:0] x、[71:36] y、[79:72] lut、[99:80] 色彩系数（20 位有符号） */
    unsigned __int128 w = 0;
    w |= (unsigned __int128)(xe[d] & 0xfffffffffULL);
    w |= (unsigned __int128)(ye[d] & 0xfffffffffULL) << 36;
    if (d < 256) {
      long q = (2L * d * (1L << c->fp) + 255) / 510;   /* round(d*2^fp/255) */
      int8_t lq = (int8_t)(q > 127 ? 127 : q);
      w |= (unsigned __int128)(uint8_t)lq << 72;
    }
    if (d < 6) w |= (unsigned __int128)((uint32_t)c->k[d] & 0xfffff) << 80;
    uint8_t *o = blk + (PREP_DRV_HDR_WORDS + d) * 16;
    for (int b = 0; b < 16; b++) o[b] = (uint8_t)(w >> (8 * b));
  }
  return 0;
}

static uint8_t prep_drv_padq(int fp) {
  long q = (2L * 114 * (1L << fp) + 255) / 510;
  return (uint8_t)(int8_t)(q > 127 ? 127 : q);
}
