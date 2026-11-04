// 命令行：读一帧 DPU 输出原样字节（.bin），跑定点参考模型，按 JSON 打印框表。
// 用法：detpost_cli <in.bin> [--nc 33] [--fp 2] [--conf 0.25] [--iou 7/10] [--max-det 100]
//                  [--offsets o80,o40,o20] [--min-score cls=0.35,...] [--enable c1,c2,...]
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "detpost.h"

using namespace detpost;

static int floor_to_q(double f, int fp) {
  // 车上按类下限：score >= f 才留；返回最小的 q 使 sigmoid(q*s) >= f
  const double s = ldexp(1.0, -fp);
  for (int q = -128; q < 128; q++)
    if (1.0f / (1.0f + expf(-(float)(q * s))) >= (float)f) return q;
  return 127;
}

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s in.bin [opts]\n", argv[0]); return 2; }
  int nc = 33, fp = 2, num = 7, den = 10, max_det = 100;
  double conf = 0.25;
  const char *offs = nullptr, *mins = nullptr, *ena = nullptr;
  for (int i = 2; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--nc")) nc = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--fp")) fp = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--conf")) conf = atof(argv[i + 1]);
    else if (!strcmp(argv[i], "--iou")) sscanf(argv[i + 1], "%d/%d", &num, &den);
    else if (!strcmp(argv[i], "--max-det")) max_det = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--offsets")) offs = argv[i + 1];
    else if (!strcmp(argv[i], "--min-score")) mins = argv[i + 1];
    else if (!strcmp(argv[i], "--enable")) ena = argv[i + 1];
    else { fprintf(stderr, "bad opt %s\n", argv[i]); return 2; }
  }
  if (nc < 1 || nc > NC_MAX || max_det < 1 || max_det > MAX_DET_MAX) { fprintf(stderr, "bad nc/max_det\n"); return 2; }
  static Config cfg;
  default_config(&cfg, nc, fp, conf, num, den, max_det);
  if (offs) {
    unsigned a, b, c;
    if (sscanf(offs, "%u,%u,%u", &a, &b, &c) != 3) return 2;
    cfg.offset[0] = a; cfg.offset[1] = b; cfg.offset[2] = c;
  }
  if (mins) {  // "cls_index=floor,..."
    char buf[512]; strncpy(buf, mins, sizeof buf - 1); buf[sizeof buf - 1] = 0;
    for (char* t = strtok(buf, ","); t; t = strtok(nullptr, ",")) {
      int c; double f;
      if (sscanf(t, "%d=%lf", &c, &f) == 2 && c >= 0 && c < nc) cfg.cls_min_q[c] = (int8_t)floor_to_q(f, fp);
    }
  }
  if (ena) {
    for (int c = 0; c < NC_MAX; c++) cfg.cls_enable[c] = 0;
    char buf[512]; strncpy(buf, ena, sizeof buf - 1); buf[sizeof buf - 1] = 0;
    for (char* t = strtok(buf, ","); t; t = strtok(nullptr, ",")) { int c = atoi(t); if (c >= 0 && c < nc) cfg.cls_enable[c] = 1; }
  }
  FILE* f = fopen(argv[1], "rb");
  if (!f) { perror(argv[1]); return 1; }
  std::vector<int8_t> buf;
  int8_t tmp[65536];
  size_t r;
  while ((r = fread(tmp, 1, sizeof tmp, f)) > 0) buf.insert(buf.end(), tmp, tmp + r);
  fclose(f);
  const size_t ch = 4 * REG_MAX + nc;
  size_t need = 0;
  for (int l = 0; l < N_LEVELS; l++) {
    size_t e = cfg.offset[l] + (size_t)cfg.grid_w[l] * cfg.grid_h[l] * ch;
    if (e > need) need = e;
  }
  if (buf.size() < need) { fprintf(stderr, "input %zu bytes < needed %zu\n", buf.size(), need); return 1; }
  static Box out[MAX_DET_MAX];
  Status st;
  int n = detpost_ref(buf.data(), cfg, out, &st);
  printf("{\"cls_thr_q\": %d, \"n_cand\": %d, \"n_cand_kept\": %d, \"overflow\": %d, \"final_level\": %d, "
         "\"n_nms\": %d, \"n_out\": %d, \"boxes\": [",
         cfg.cls_thr_q, st.n_cand, st.n_cand_kept, st.overflow, st.final_level, st.n_nms, st.n_out);
  for (int i = 0; i < n; i++) {
    const Box& b = out[i];
    printf("%s[%.4f, %.4f, %.4f, %.4f, %d, %d, %.6f, %d]", i ? ", " : "", b.x1 / 16.0, b.y1 / 16.0, b.x2 / 16.0,
           b.y2 / 16.0, b.cls, b.level, b.score / 65535.0, b.anchor);
  }
  printf("]}\n");
  return 0;
}
