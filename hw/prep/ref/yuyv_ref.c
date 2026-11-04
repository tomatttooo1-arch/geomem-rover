/* PREP1：YUYV/UYVY -> RGB888 参考（C 模型的 YUYV 前端；之后接 PREP0/model/prep_model 做缩放/灰边/量化）。2026-11-03
 * 写法刻意与 HLS 不同，用来交叉核对：
 *   色彩：kset 0 用 libjpeg-turbo jdcolor.c 的查表写法（build_ycc_rgb_table + range_limit）；
 *         kset 1（BT.601 有限范围）用通用式 clamp((KY(Y-YOFF) + KX(C-128) + 2^15) >> 16)。
 *   上采样：ups 1 = libjpeg jdsample.c h2v1_fancy_upsample 原样（首列、末列特例）；ups 0 = 复制。
 * 用法：yuyv_ref <in> <w> <h> <fmt 2=YUYV 3=UYVY> <ups> <kset> <out.rgb>
 *       yuyv_ref --selftest   （穷举 256^3 个 (Y,U,V)：通用式取 JFIF 系数 == libjpeg 查表式）
 * 编译：gcc -O2 -std=c99 -o yuyv_ref yuyv_ref.c */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SCALEBITS 16
#define ONE_HALF ((int32_t)1 << (SCALEBITS - 1))
#define FIX(x) ((int32_t)((x) * (1L << SCALEBITS) + 0.5))

static int Cr_r_tab[256], Cb_b_tab[256];
static int32_t Cr_g_tab[256], Cb_g_tab[256];

static void build_ycc_rgb_table(void) {
  for (int i = 0, x = -128; i <= 255; i++, x++) {
    Cr_r_tab[i] = (int)((FIX(1.40200) * x + ONE_HALF) >> SCALEBITS);
    Cb_b_tab[i] = (int)((FIX(1.77200) * x + ONE_HALF) >> SCALEBITS);
    Cr_g_tab[i] = (-FIX(0.71414)) * x;
    Cb_g_tab[i] = (-FIX(0.34414)) * x + ONE_HALF;
  }
}
static uint8_t lim(int v) { return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v; }

static void jfif_px(int y, int cb, int cr, uint8_t *o) {
  o[0] = lim(y + Cr_r_tab[cr]);
  o[1] = lim(y + (int)((Cb_g_tab[cb] + Cr_g_tab[cr]) >> SCALEBITS));
  o[2] = lim(y + Cb_b_tab[cb]);
}
static void gen_px(const int *k, int y, int u, int v, uint8_t *o) {
  long yk = (long)k[0] * (y - k[1]);
  o[0] = lim((int)((yk + (long)k[2] * (v - 128) + 32768) >> 16));
  o[1] = lim((int)((yk + (long)k[3] * (u - 128) + (long)k[4] * (v - 128) + 32768) >> 16));
  o[2] = lim((int)((yk + (long)k[5] * (u - 128) + 32768) >> 16));
}

/* libjpeg h2v1_fancy_upsample：in 有 n 个样本，out 有 2n 个 */
static void h2v1_fancy(const uint8_t *in, int n, uint8_t *out) {
  const uint8_t *inptr = in;
  uint8_t *outptr = out;
  int invalue = *inptr++;
  *outptr++ = (uint8_t)invalue;
  *outptr++ = (uint8_t)((invalue * 3 + inptr[0] + 2) >> 2);
  for (int colctr = n - 2; colctr > 0; colctr--) {
    invalue = (*inptr++) * 3;
    *outptr++ = (uint8_t)((invalue + inptr[-2] + 1) >> 2);
    *outptr++ = (uint8_t)((invalue + inptr[0] + 2) >> 2);
  }
  invalue = *inptr;
  *outptr++ = (uint8_t)((invalue * 3 + inptr[-1] + 1) >> 2);
  *outptr++ = (uint8_t)invalue;
}

static const int K_JFIF[6] = {65536, 0, 91881, -22554, -46802, 116130};
static const int K_601L[6] = {76309, 16, 104597, -25675, -53279, 132201};

int main(int argc, char **argv) {
  build_ycc_rgb_table();
  if (argc == 2 && !strcmp(argv[1], "--selftest")) {
    long bad = 0;
    for (int y = 0; y < 256; y++)
      for (int u = 0; u < 256; u++)
        for (int v = 0; v < 256; v++) {
          uint8_t a[3], b[3];
          jfif_px(y, u, v, a);
          gen_px(K_JFIF, y, u, v, b);
          bad += memcmp(a, b, 3) != 0;
        }
    printf("selftest JFIF general-vs-libjpeg-table: %ld mismatches of 16777216\n", bad);
    return bad != 0;
  }
  if (argc < 8) { fprintf(stderr, "usage: see header\n"); return 2; }
  int w = atoi(argv[2]), h = atoi(argv[3]), fmt = atoi(argv[4]), ups = atoi(argv[5]), kset = atoi(argv[6]);
  if (w % 2 || w < 4) { fprintf(stderr, "width must be even >= 4\n"); return 2; }
  long nin = (long)w * h * 2;
  uint8_t *in = malloc(nin), *out = malloc((long)w * h * 3);
  FILE *f = fopen(argv[1], "rb");
  if (!f || fread(in, 1, nin, f) != (size_t)nin) { fprintf(stderr, "read fail\n"); return 2; }
  fclose(f);
  int n = w / 2;
  uint8_t *Y = malloc(w), *U = malloc(n), *V = malloc(n), *Uu = malloc(w), *Vu = malloc(w);
  for (int r = 0; r < h; r++) {
    const uint8_t *p = in + (long)r * w * 2;
    for (int i = 0; i < n; i++) {
      const uint8_t *q = p + 4 * i;
      if (fmt == 3) { U[i] = q[0]; Y[2 * i] = q[1]; V[i] = q[2]; Y[2 * i + 1] = q[3]; }
      else          { Y[2 * i] = q[0]; U[i] = q[1]; Y[2 * i + 1] = q[2]; V[i] = q[3]; }
    }
    if (ups) { h2v1_fancy(U, n, Uu); h2v1_fancy(V, n, Vu); }
    else for (int i = 0; i < w; i++) { Uu[i] = U[i / 2]; Vu[i] = V[i / 2]; }
    for (int x = 0; x < w; x++) {
      uint8_t *o = out + ((long)r * w + x) * 3;
      if (kset == 0) jfif_px(Y[x], Uu[x], Vu[x], o);
      else gen_px(K_601L, Y[x], Uu[x], Vu[x], o);
    }
  }
  f = fopen(argv[7], "wb");
  fwrite(out, 1, (long)w * h * 3, f);
  fclose(f);
  return 0;
}
