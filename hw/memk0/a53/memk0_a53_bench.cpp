// MEMK3 A53 reference timing ("before" column): the fixed-point memory core C model (the same source as the PL
// kernel) replays a recorded call stream (tools/memk0/record_calls.py, converted by mk_bench_data.py) on one core.
//   memk0_a53_bench DATA.bin [passes=5] [--flip]
// DATA: u32 n_calls, then per call: u32 seq, cmd[1024], golden res[1024]. The result buffer persists across calls
// (as in PLMemory); every call's whole result block is checked against the golden (from the host C model).
// Timing wraps memk0_top only (CLOCK_MONOTONIC). Reports per call and per frame (calls with the same frame seq
// summed) median and p99 over all passes after one warm-up pass. --flip: positive control (observation 0 x moved
// by 2^-6 m in every call; the golden check must fail).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include "../memk0.h"
typedef unsigned int u32;
void memk0_top(const u32 cmd[MK_CMD_WORDS], u32 res[MK_RES_WORDS]);
static double now_us() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e6 + t.tv_nsec * 1e-3; }
static double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0; std::sort(v.begin(), v.end());
  size_t k = (size_t)(p * (v.size() - 1) + 0.5); return v[k];
}
int main(int argc, char **argv) {
  if (argc < 2) { printf("usage: %s DATA.bin [passes] [--flip]\n", argv[0]); return 2; }
  int passes = 5; bool flip = false;
  for (int k = 2; k < argc; k++) { if (!strcmp(argv[k], "--flip")) flip = true; else passes = atoi(argv[k]); }
  FILE *fp = fopen(argv[1], "rb"); if (!fp) { printf("cannot open %s\n", argv[1]); return 2; }
  u32 n = 0; if (fread(&n, 4, 1, fp) != 1) return 2;
  std::vector<u32> seq(n), cmd((size_t)n * MK_CMD_WORDS), gold((size_t)n * MK_RES_WORDS);
  for (u32 c = 0; c < n; c++) {
    if (fread(&seq[c], 4, 1, fp) != 1 || fread(&cmd[(size_t)c * MK_CMD_WORDS], 4, MK_CMD_WORDS, fp) != MK_CMD_WORDS ||
        fread(&gold[(size_t)c * MK_RES_WORDS], 4, MK_RES_WORDS, fp) != MK_RES_WORDS) { printf("truncated\n"); return 2; }
  }
  fclose(fp);
  if (flip) for (u32 c = 0; c < n; c++) cmd[(size_t)c * MK_CMD_WORDS + MK_OBS_BASE + 2] ^= 1u << 12;   // obs 0 x + 1.6 cm, every call
  static u32 res[MK_RES_WORDS];
  std::vector<double> tcall, tframe; long bad = 0;
  for (int p = 0; p <= passes; p++) {               // pass 0 is the warm-up
    memset(res, 0, sizeof(res));
    double fsum = 0; u32 cur = seq.empty() ? 0 : seq[0];
    for (u32 c = 0; c < n; c++) {
      if (seq[c] != cur) { if (p) tframe.push_back(fsum); fsum = 0; cur = seq[c]; }
      double t0 = now_us();
      memk0_top(&cmd[(size_t)c * MK_CMD_WORDS], res);
      double dt = now_us() - t0;
      fsum += dt; if (p) tcall.push_back(dt);
      if (memcmp(res, &gold[(size_t)c * MK_RES_WORDS], sizeof(res))) bad++;
    }
    if (p) tframe.push_back(fsum);
  }
  printf("calls %u passes %d frames/pass %zu\n", n, passes, tframe.size() / (passes ? passes : 1));
  printf("per_call_us median %.2f p99 %.2f\n", pct(tcall, 0.5), pct(tcall, 0.99));
  printf("per_frame_us median %.2f p99 %.2f\n", pct(tframe, 0.5), pct(tframe, 0.99));
  printf("golden %s (%ld call results differ over %d passes)\n", bad ? "MISMATCH" : "MATCH", bad, passes + 1);
  return bad ? 1 : 0;
}
