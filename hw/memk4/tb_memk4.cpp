// MEMK4 replay testbench: replays a call record (tools/memk0/record_calls.py) through memk4_top and compares the
// whole result block word by word with the recorded one (recorded from the C model). Used for g++ / HLS csim and
// for cosim.   tb_memk4 RECORD.bin [first_call [last_call]]      (exit code 0 = every call identical)
// The record must start with an INIT call (the core state is only built up through calls; INIT clears the result
// block, the recorded block before INIT must therefore be zero -- checked).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef unsigned int u32;
#ifndef MK4_IF
#define MK4_IF 1
#endif
#if MK4_IF == 1
void memk4_top(u32 blk[2048]);
#else
void memk4_top(const u32 *cmd_g, u32 *res_g);
#endif

int main(int argc, char **argv) {
  const char *path = argc > 1 ? argv[1] : "vectors.bin";
  long first = argc > 2 ? atol(argv[2]) : 0, last = argc > 3 ? atol(argv[3]) : -1;
  FILE *fp = fopen(path, "rb");
  if (!fp) { printf("TB: cannot open %s\n", path); return 2; }
  static u32 hdr[4], cmd[1024], resb[1024], resa[1024], blk[2048], res[1024];
  long calls = 0, bad_calls = 0, bad_words = 0, shown = 0;
  while (fread(hdr, 4, 4, fp) == 4) {
    if (fread(cmd, 4, 1024, fp) != 1024 || fread(resb, 4, 1024, fp) != 1024 || fread(resa, 4, 1024, fp) != 1024) {
      printf("TB: truncated record\n"); return 2;
    }
    long idx = (long)hdr[1];
    if (last >= 0 && idx > last) break;
    if (((cmd[0] >> 16) & 15) == 1)
      for (int w = 4; w < 1024; w++) if (resb[w]) { printf("TB: call %ld INIT with a non-zero result block\n", idx); return 2; }
#if MK4_IF == 1
    memcpy(blk, cmd, 4096);
    memk4_top(blk);
    memcpy(res, blk + 1024, 4096);
#else
    memk4_top(cmd, res);
#endif
    if (idx < first) continue;
    int nb = 0;
    for (int w = 0; w < 1024; w++) {
      if (res[w] != resa[w]) {
        nb++;
        if (shown < 12) { printf("TB: call %ld seq %u word %d: got %08x want %08x\n", idx, hdr[2], w, res[w], resa[w]); shown++; }
      }
    }
    calls++;
    if (nb) { bad_calls++; bad_words += nb; }
  }
  fclose(fp);
  printf("TB: %ld calls replayed, %ld calls differ, %ld words differ -> %s\n", calls, bad_calls, bad_words,
         bad_calls ? "FAIL" : "PASS");
  return bad_calls ? 1 : 0;
}
