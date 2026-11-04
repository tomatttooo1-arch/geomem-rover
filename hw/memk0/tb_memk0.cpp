// MEMK0 replay testbench: replays a call record (tools/memk0/record_calls.py) through memk0_top and compares the
// whole result block word by word with the recorded one. Used for the C-model differential check (g++) and for
// HLS csim / cosim (key comparison 2: RTL vs the C model's recorded result blocks, byte for byte).
//   tb_memk0 RECORD.bin [first_call [last_call]]      (exit code 0 = every call identical)
// The record must start with an INIT call (the core state is only built up through calls).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "memk0.h"
typedef unsigned int u32;
void memk0_top(const u32 cmd[MK_CMD_WORDS], u32 res[MK_RES_WORDS]);

int main(int argc, char **argv) {
  const char *path = argc > 1 ? argv[1] : "vectors.bin";
  long first = argc > 2 ? atol(argv[2]) : 0, last = argc > 3 ? atol(argv[3]) : -1;
  FILE *fp = fopen(path, "rb");
  if (!fp) { printf("TB: cannot open %s\n", path); return 2; }
  static u32 hdr[4], cmd[MK_CMD_WORDS], resb[MK_RES_WORDS], resa[MK_RES_WORDS], res[MK_RES_WORDS];
  long calls = 0, bad_calls = 0, bad_words = 0, shown = 0;
  while (fread(hdr, 4, 4, fp) == 4) {
    if (fread(cmd, 4, MK_CMD_WORDS, fp) != MK_CMD_WORDS || fread(resb, 4, MK_RES_WORDS, fp) != MK_RES_WORDS ||
        fread(resa, 4, MK_RES_WORDS, fp) != MK_RES_WORDS) { printf("TB: truncated record\n"); return 2; }
    long idx = (long)hdr[1];
    if (idx < first) continue;
    if (last >= 0 && idx > last) break;
    memcpy(res, resb, sizeof(res));
    memk0_top(cmd, res);
    int nb = 0;
    for (int w = 0; w < MK_RES_WORDS; w++) {
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
