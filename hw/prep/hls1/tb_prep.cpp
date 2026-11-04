// PREP1 testbench：逐行读向量表，按驱动规则生成参数块、装源帧（含行步长填充），调 prep_top，
// 把整块张量写进 tb_out/<id>.bin，状态与越界检查写进 tb_out/meta.txt（csim/cosim 都比这两类文件）。
// 向量行：id infile fmt ups bypass w0 h0 stride tw th ow oh top left fp kset    （kset 0 JFIF、1 BT.601 有限范围）
// 每帧：驱动 prep_drv_params 生成参数块（帧头+表），调 prep_top(src,dst,prm,&status)。
// 环境变量 PREP_PERTURB=1：参数块第一个非零水平 c1 +1（阳性对照）。
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "prep_k.h"
#include "prep_drv.h"

static const long SRC_WORDS = 2048L * 2048 * 3 / 16;
static const long DST_WORDS = 640L * 640 * 3 / 16;
static const long GUARD = 64;

static word_t to_word(const uint8_t* b) {
  word_t w = 0;
  for (int i = 0; i < 16; i++) w(8 * i + 7, 8 * i) = b[i];
  return w;
}

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: tb <vec.txt>\n"); return 2; }
  FILE* vf = fopen(argv[1], "r");
  if (!vf) { fprintf(stderr, "cannot open %s\n", argv[1]); return 2; }
  const char* pe = getenv("PREP_PERTURB");
  int perturb = pe && atoi(pe);
  mkdir("tb_out", 0755);
  FILE* meta = fopen("tb_out/meta.txt", "w");
  word_t* src = new word_t[SRC_WORDS];
  word_t* dst = new word_t[DST_WORDS + GUARD];
  word_t* prm = new word_t[PREP_DRV_PRM_WORDS];
  static uint8_t blk[PREP_DRV_PRM_WORDS * 16];
  int nfail = 0, ncase = 0;
  uint8_t* raw = (uint8_t*)malloc(2048L * 2048 * 3);
  uint8_t* row = (uint8_t*)malloc(8192 + 16);
  char line[1024];
  while (fgets(line, sizeof line, vf)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    char id[128], path[768];
    prep_cfg_t c;
    int kset;
    if (sscanf(line, "%127s %767s %d %d %d %d %d %d %d %d %d %d %d %d %d %d", id, path, &c.fmt, &c.ups, &c.bypass,
               &c.w0, &c.h0, &c.stride, &c.tw, &c.th, &c.ow, &c.oh, &c.top, &c.left, &c.fp, &kset) != 16) {
      fprintf(stderr, "bad line: %s", line);
      return 2;
    }
    memcpy(c.k, kset ? PREP_K_BT601L : PREP_K_JFIF, sizeof c.k);
    int bpp = (c.fmt >= 2) ? 2 : 3;
    long rowb = (long)c.w0 * bpp;
    FILE* f = fopen(path, "rb");
    if (!f || fread(raw, 1, rowb * c.h0, f) != (size_t)(rowb * c.h0)) { fprintf(stderr, "read fail %s\n", path); return 2; }
    fclose(f);
    // 源帧：每行从 stride 对齐处开始，行间空隙填 0xA5（核不得读进内容）
    for (long r = 0; r < c.h0; r++) {
      memset(row, 0xA5, c.stride);
      memcpy(row, raw + r * rowb, rowb);
      for (long w = 0; w < c.stride / 16; w++) src[r * (c.stride / 16) + w] = to_word(row + 16 * w);
    }
    // 参数块每帧都给（核每帧都读）；几何非法时驱动本不该启动核，这里只写帧头去测核自己的拒绝路径
    if (prep_drv_params(&c, blk, perturb) != 0) {
      memset(blk, 0, sizeof blk);
      prep_drv_header(&c, blk);
    }
    for (int d = 0; d < PREP_DRV_PRM_WORDS; d++) prm[d] = to_word(blk + 16 * d);
    uint8_t pat[16];
    memset(pat, 0x5A, 16);
    for (long i = 0; i < DST_WORDS + GUARD; i++) dst[i] = to_word(pat);
    ap_uint<32> status = 0;
    prep_top(src, dst, prm, &status);
    long nw = (long)c.tw * c.th * 3 / 16;
    int err = (int)(status & 0xff);
    int guard_ok = 1;
    long first_touch = -1;
    word_t wp = to_word(pat);
    for (long i = (err ? 0 : nw); i < DST_WORDS + GUARD; i++)
      if (dst[i] != wp) { guard_ok = 0; first_touch = i; break; }
    if (!err) {
      char op[300];
      snprintf(op, sizeof op, "tb_out/%s.bin", id);
      FILE* o = fopen(op, "wb");
      for (long i = 0; i < nw; i++) {
        uint8_t b[16];
        for (int k = 0; k < 16; k++) b[k] = dst[i](8 * k + 7, 8 * k);
        fwrite(b, 1, 16, o);
      }
      fclose(o);
    }
    fprintf(meta, "%s status=%08x untouched_outside=%d\n", id, (unsigned)status, guard_ok);
    if (!guard_ok) { fprintf(stderr, "%s: write outside tensor at word %ld\n", id, first_touch); nfail++; }
    printf("case %-28s status=%08x\n", id, (unsigned)status);
    ncase++;
  }
  fclose(meta);
  printf("TB DONE cases=%d guard_fail=%d\n", ncase, nfail);
  delete[] src;
  delete[] dst;
  delete[] prm;
  free(raw);
  free(row);
  return 0;
}
