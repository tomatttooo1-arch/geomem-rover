// SRCHK 搜索核 testbench：回放 tools/srch/srch_harness.py trace 录下的向量（DESIGN §10.2，V1/V2）。
// 每条记录 = 一次核调用：把调用前的整块 DDR 镜像（0x60000 字节）拷进 gmem，调核，再把 gmem 前 0x60000 字节与
// 调用后镜像逐字节比。片上状态（GRID/VISW/ENTRY/PST/MST/FLAGS）跨记录保留，所以记录必须从 INIT_MAP 起按序回放；
// 状态本身经每轮 DUMP 记录（PST|MST 全量导出）与 SELECT 结果块（增益、质量、编号）间接逐位核对。
//   SW 自检：g++ -O2 -std=c++14 -DSRCH_SW -Wno-sign-compare -o tb hw/srch/srch_tb.cpp hw/srch/srch_core.cpp && ./tb v1.bin v2.bin
//   HLS：csim/cosim 的 testbench 就是本文件（不加 -DSRCH_SW），参数为向量文件路径（argv），无参数时读 SRCH_TB_VEC 环境变量
//        （冒号分隔）。gmem 按顶层 m_axi depth=24576 拍分配（0x60000 字节，与向量镜像同大）。
// 返回 0 = 全部记录逐位相同。
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "srch_core.h"

#ifdef SRCH_SW
extern "C" uint32_t srch_run_sw(uint8_t* ddr, uint64_t prm);
#else
void srch_top(w128_t* gmem, ap_uint<64> prm, ap_uint<32>& status);
#endif

static const size_t TB_BEATS = 24576;  // = m_axi depth（SRCHK3 从 1048576 改小，向量只用 0x60000 字节）

static void to_gmem(w128_t* g, const uint8_t* b, size_t nbytes) {
  for (size_t k = 0; k < nbytes / 16; k++) {
    uint64_t lo, hi;
    memcpy(&lo, b + 16 * k, 8);
    memcpy(&hi, b + 16 * k + 8, 8);
#ifdef SRCH_SW
    g[k] = ((w128_t)hi << 64) | (w128_t)lo;
#else
    w128_t w = 0;
    w.range(63, 0) = lo;
    w.range(127, 64) = hi;
    g[k] = w;
#endif
  }
}

static void from_gmem(const w128_t* g, uint8_t* b, size_t nbytes) {
  for (size_t k = 0; k < nbytes / 16; k++) {
#ifdef SRCH_SW
    uint64_t lo = (uint64_t)g[k], hi = (uint64_t)(g[k] >> 64);
#else
    uint64_t lo = g[k].range(63, 0).to_uint64(), hi = g[k].range(127, 64).to_uint64();
#endif
    memcpy(b + 16 * k, &lo, 8);
    memcpy(b + 16 * k + 8, &hi, 8);
  }
}

static int replay(const char* path, w128_t* gmem) {
  FILE* f = fopen(path, "rb");
  if (!f) { printf("TB cannot open %s\n", path); return 1; }
  char magic[4]; uint32_t n, nbytes, pad;
  if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "SRTR", 4) || fread(&n, 4, 1, f) != 1 ||
      fread(&nbytes, 4, 1, f) != 1 || fread(&pad, 4, 1, f) != 1) {
    printf("TB bad header %s\n", path); fclose(f); return 1;
  }
  std::vector<uint8_t> before(nbytes), after(nbytes), got(nbytes);
  int bad = 0;
  // SRCHK5：SRCH_TB_MAXREC=N 只回放每个向量的前 N 条记录（二分挂起用；默认全量）
  const char* mr = getenv("SRCH_TB_MAXREC");
  if (mr && atoi(mr) > 0 && (uint32_t)atoi(mr) < n) n = (uint32_t)atoi(mr);
  for (uint32_t r = 0; r < n; r++) {
    uint32_t op, prm;
    if (fread(&op, 4, 1, f) != 1 || fread(&prm, 4, 1, f) != 1 || fread(before.data(), 1, nbytes, f) != nbytes ||
        fread(after.data(), 1, nbytes, f) != nbytes) {
      printf("TB short read %s rec %u\n", path, r); fclose(f); return 1;
    }
    to_gmem(gmem, before.data(), nbytes);
    uint32_t st;
#ifdef SRCH_SW
    st = srch_run_sw((uint8_t*)gmem, prm);
#else
    ap_uint<32> s;
    srch_top(gmem, (ap_uint<64>)prm, s);
    st = s.to_uint();
#endif
    from_gmem(gmem, got.data(), nbytes);
    size_t nd = 0, first = 0;
    for (size_t i = 0; i < nbytes; i++)
      if (got[i] != after[i]) { if (!nd) first = i; nd++; }
    printf("TB %s rec %u op %u done\n", path, r, op); fflush(stdout);
    if (nd || (st & 0xFF)) {
      bad++;
      printf("TB %s rec %u op %u status 0x%x: %zu bytes differ (first at 0x%zx: got %02x want %02x)\n", path, r, op, st,
             nd, first, nd ? got[first] : 0, nd ? after[first] : 0);
    }
  }
  fclose(f);
  printf("TB %s: %u records, %d mismatching\n", path, n, bad);
  return bad ? 1 : 0;
}

int main(int argc, char** argv) {
  std::vector<std::string> vec;
  for (int i = 1; i < argc; i++) vec.push_back(argv[i]);
  if (vec.empty()) {
    const char* e = getenv("SRCH_TB_VEC");
    if (!e) { printf("TB usage: tb <trace.bin>... (or SRCH_TB_VEC=a.bin:b.bin)\n"); return 1; }
    std::string s(e);
    size_t p = 0;
    while (p <= s.size()) {
      size_t q = s.find(':', p);
      if (q == std::string::npos) q = s.size();
      if (q > p) vec.push_back(s.substr(p, q - p));
      p = q + 1;
    }
  }
  w128_t* gmem = new w128_t[TB_BEATS];
  int rc = 0;
  for (size_t i = 0; i < vec.size(); i++) rc |= replay(vec[i].c_str(), gmem);
  delete[] gmem;
  printf("TB %s\n", rc ? "FAIL" : "PASS");
  return rc;
}
