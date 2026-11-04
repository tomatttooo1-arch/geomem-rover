// 检测后处理核 testbench（csim / cosim 共用；-DGOLDEN 时直接调定点参考模型 detpost_ref 生成 golden）。
//
// 用法：tb_detpost <vectors.txt>
//   每行一个向量：f=<帧文件> [nc=33] [fp=2] [conf=0.25] [iou=7/10] [md=100] [shift=0]
//                 [grid=80,40,20] [off=auto|o80,o40,o20] [min=cls:floor,...] [en=all|c1,c2,...]
//   off=auto：按 xmodel 的排列（20x20 在前、40x40、80x80 最后）首尾相接；shift>0 时整块缓冲后移 shift 字节、
//   偏移同加 shift（造不按 16 字节对齐的偏移，覆盖 416 输入一类的排列）。
// 输出：tb_out/v%03d.bin = 6 个 int32 状态（n_cand, n_cand_kept, overflow, final_level, n_nms, n_out）
//       + n_out 条 16 字节框记录（小端：x1,y1,x2,y2 int16；cls u8；level i8；score u16；anchor u16；pad u16）。
//       tb_out/summary.txt：每向量一行文本摘要。
// 对 golden（参考模型，CAND_MAX=256）逐字节比对用 fpga-hls 的 bitexact_check.py。
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifndef DETPOST_TB_BUF_BYTES
#define DETPOST_TB_BUF_BYTES 815104
#endif

#ifdef GOLDEN
#include "../detpost.h"
using namespace detpost;
static void dut_run(const int8_t* buf, const Config& cfg, Box* out, Status* st) { detpost_ref(buf, cfg, out, st); }
#else
#include "dut_adapter.h"
#endif

static const int BUF_BYTES = DETPOST_TB_BUF_BYTES;  // 与 HLS 接口 depth 一致

static int floor_to_q(double f, int fp) {
  const double s = ldexp(1.0, -fp);
  for (int q = -128; q < 128; q++)
    if (1.0f / (1.0f + expf(-(float)(q * s))) >= (float)f) return q;
  return 127;
}

static void put16(FILE* f, uint16_t v) { fputc(v & 255, f); fputc(v >> 8, f); }
static void put32(FILE* f, int32_t v) { uint32_t u = (uint32_t)v; for (int i = 0; i < 4; i++) fputc((u >> (8 * i)) & 255, f); }

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s vectors.txt\n", argv[0]); return 2; }
  FILE* vf = fopen(argv[1], "r");
  if (!vf) { perror(argv[1]); return 2; }
  mkdir("tb_out", 0755);
  FILE* sum = fopen("tb_out/summary.txt", "w");
  static int8_t buf[BUF_BYTES];
  static Config cfg;
  static Box out[MAX_DET_MAX];
  char line[2048];
  int vi = 0, total_boxes = 0;
  while (fgets(line, sizeof line, vf)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    char path[1024] = "", offs[128] = "auto", mins[256] = "", ens[256] = "all";
    int nc = 33, fp = 2, num = 7, den = 10, md = 100, shift = 0, g[3] = {80, 40, 20};
    double conf = 0.25;
    for (char* t = strtok(line, " \t\n"); t; t = strtok(nullptr, " \t\n")) {
      if (!strncmp(t, "f=", 2)) snprintf(path, sizeof path, "%s", t + 2);
      else if (!strncmp(t, "nc=", 3)) nc = atoi(t + 3);
      else if (!strncmp(t, "fp=", 3)) fp = atoi(t + 3);
      else if (!strncmp(t, "conf=", 5)) conf = atof(t + 5);
      else if (!strncmp(t, "iou=", 4)) sscanf(t + 4, "%d/%d", &num, &den);
      else if (!strncmp(t, "md=", 3)) md = atoi(t + 3);
      else if (!strncmp(t, "shift=", 6)) shift = atoi(t + 6);
      else if (!strncmp(t, "grid=", 5)) sscanf(t + 5, "%d,%d,%d", &g[0], &g[1], &g[2]);
      else if (!strncmp(t, "off=", 4)) snprintf(offs, sizeof offs, "%s", t + 4);
      else if (!strncmp(t, "min=", 4)) snprintf(mins, sizeof mins, "%s", t + 4);
      else if (!strncmp(t, "en=", 3)) snprintf(ens, sizeof ens, "%s", t + 3);
      else { fprintf(stderr, "bad token %s\n", t); return 2; }
    }
    // 配置（与 detpost_host.cpp default_config 同一套换算）
    memset(&cfg, 0, sizeof cfg);
    default_config(&cfg, nc, fp, conf, num, den, md);
    const int ch = 4 * REG_MAX + nc;
    for (int l = 0; l < 3; l++) { cfg.grid_w[l] = cfg.grid_h[l] = g[l]; }
    if (!strcmp(offs, "auto")) {
      cfg.offset[2] = 0;
      cfg.offset[1] = (uint32_t)(g[2] * g[2] * ch);
      cfg.offset[0] = cfg.offset[1] + (uint32_t)(g[1] * g[1] * ch);
    } else {
      unsigned a, b, c;
      if (sscanf(offs, "%u,%u,%u", &a, &b, &c) != 3) return 2;
      cfg.offset[0] = a; cfg.offset[1] = b; cfg.offset[2] = c;
    }
    for (int l = 0; l < 3; l++) cfg.offset[l] += shift;
    if (mins[0]) {
      for (char* t = strtok(mins, ","); t; t = strtok(nullptr, ",")) {
        int c; double f;
        if (sscanf(t, "%d:%lf", &c, &f) == 2 && c >= 0 && c < nc) cfg.cls_min_q[c] = (int8_t)floor_to_q(f, fp);
      }
    }
    if (strcmp(ens, "all")) {
      for (int c = 0; c < NC_MAX; c++) cfg.cls_enable[c] = 0;
      for (char* t = strtok(ens, ","); t; t = strtok(nullptr, ",")) { int c = atoi(t); if (c >= 0 && c < nc) cfg.cls_enable[c] = 1; }
    }
    // 输入缓冲：固定大小，先填一个与帧无关的确定图样（检验核不越界读到无关字节也不受影响），再放帧
    for (int i = 0; i < BUF_BYTES; i++) buf[i] = (int8_t)(((i * 131) ^ (i >> 7)) & 255);
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return 1; }
    size_t r = fread(buf + shift, 1, BUF_BYTES - shift, f);
    fclose(f);
    size_t need = 0;
    for (int l = 0; l < 3; l++) {
      size_t e = cfg.offset[l] + (size_t)g[l] * g[l] * ch;
      if (e > need) need = e;
    }
    if (r + shift < need || need > (size_t)BUF_BYTES) { fprintf(stderr, "%s: %zu bytes < %zu\n", path, r, need); return 1; }
    for (int i = 0; i < MAX_DET_MAX; i++) memset(&out[i], 0, sizeof(Box));
    Status st;
    memset(&st, 0, sizeof st);
    dut_run(buf, cfg, out, &st);
    char name[64];
    snprintf(name, sizeof name, "tb_out/v%03d.bin", vi);
    FILE* of = fopen(name, "wb");
    put32(of, st.n_cand); put32(of, st.n_cand_kept); put32(of, st.overflow);
    put32(of, st.final_level); put32(of, st.n_nms); put32(of, st.n_out);
    for (int i = 0; i < st.n_out && i < MAX_DET_MAX; i++) {
      const Box& b = out[i];
      put16(of, (uint16_t)b.x1); put16(of, (uint16_t)b.y1); put16(of, (uint16_t)b.x2); put16(of, (uint16_t)b.y2);
      fputc(b.cls, of); fputc((uint8_t)b.level, of); put16(of, b.score); put16(of, b.anchor); put16(of, b.pad);
    }
    fclose(of);
    const char* base = strrchr(path, '/');
    fprintf(sum, "v%03d %s nc=%d thr=%d cand=%d kept=%d ovf=%d fl=%d nms=%d out=%d\n", vi, base ? base + 1 : path, nc,
            cfg.cls_thr_q, st.n_cand, st.n_cand_kept, st.overflow, st.final_level, st.n_nms, st.n_out);
    total_boxes += st.n_out;
    vi++;
  }
  fclose(vf);
  fclose(sum);
  printf("TB DONE vectors=%d boxes=%d\n", vi, total_boxes);
  return 0;
}
