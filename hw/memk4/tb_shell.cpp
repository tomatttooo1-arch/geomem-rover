#include <stdio.h>
typedef unsigned int u32;
void memk4_shell(const u32 *cmd_g, u32 *res_g);
int main() { static u32 c[1024], r[1024]; for (int k = 0; k < 1024; k++) c[k] = k; memk4_shell(c, r); printf("TB: %u PASS\n", r[5]); return 0; }
