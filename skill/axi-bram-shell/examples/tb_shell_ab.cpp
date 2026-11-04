#include <stdio.h>
typedef unsigned int u32;
#ifdef VB
void memk4_shellb(u32 blk[2048]);
int main() { static u32 b[2048]; for (int k = 0; k < 2048; k++) b[k] = k; memk4_shellb(b); printf("TB: %u PASS\n", b[1029]); return 0; }
#else
void memk4_shella(const u32 *cmd_g, u32 *res_g);
int main() { static u32 c[1024], r[1024]; for (int k = 0; k < 1024; k++) c[k] = k; memk4_shella(c, r); printf("TB: %u PASS\n", r[5]); return 0; }
#endif
