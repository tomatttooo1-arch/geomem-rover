#include <ap_int.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#define TOY_N 64
void toy(const ap_uint<8> in[TOY_N], ap_uint<16> out[TOY_N], ap_int<12> bias);
int main() {
  ap_uint<8> in[TOY_N]; ap_uint<16> out[TOY_N];
  for (int i = 0; i < TOY_N; i++) in[i] = (i * 37 + 11) & 0xff;
  toy(in, out, ap_int<12>(-5));
  const char *env = std::getenv("HLS_TB_OUT_DIR");
  std::string dir = env && *env ? env : "tb_out";
  mkdir(dir.c_str(), 0755);
  FILE *fb = std::fopen((dir + "/out.bin").c_str(), "wb");
  FILE *ft = std::fopen((dir + "/out.txt").c_str(), "w");
  if (!fb || !ft) { std::printf("cannot open %s\n", dir.c_str()); return 1; }
  for (int i = 0; i < TOY_N; i++) {
    unsigned short w = (unsigned short)out[i].to_uint();
    std::fwrite(&w, 2, 1, fb);
    std::fprintf(ft, "%d %u\n", i, (unsigned)w);
  }
  std::fclose(fb); std::fclose(ft);
  std::printf("toy tb wrote %s\n", dir.c_str());
  return 0;
}
