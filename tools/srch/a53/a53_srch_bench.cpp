// SRCHK3：搜索核 A53 同算法计时程序（文档"优化前"列）。算法就是定点 C 模型 hw/srch/srch_core.cpp（-DSRCH_SW），
// 与 PL 核逐位相同（HLS csim/cosim 对同一模型逐字节）。输入是联仿向量 trace_v1.bin / trace_v2.bin（DESIGN §10.2）。
// 每遍从 INIT_MAP 起按序回放全部记录（片上状态是静态数组，INIT_MAP→RESET 重建，所以每遍结果相同），
// 每条记录：拷调用前 DDR 镜像（不计时）→ 计时 srch_run_sw → 与调用后镜像逐字节比（不计时）。
// 用法：a53_srch_bench <observe|select|all> <passes> <trace>...
// 输出 markdown 表：每类操作的次数、每次中位数与 p99（微秒），以及 bitexact（全部遍全部记录逐字节同）。
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <string>
#include <algorithm>
#include "../../../hw/srch/srch_core.cpp"

struct Rec { uint32_t op, prm; std::vector<uint8_t> before, after; };

static bool load(const char* path, std::vector<Rec>& out, uint32_t& nbytes) {
  FILE* f = fopen(path, "rb");
  if (!f) { fprintf(stderr, "cannot open %s\n", path); return false; }
  char magic[4]; uint32_t n, pad;
  if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "SRTR", 4) || fread(&n, 4, 1, f) != 1 ||
      fread(&nbytes, 4, 1, f) != 1 || fread(&pad, 4, 1, f) != 1) { fclose(f); return false; }
  for (uint32_t r = 0; r < n; r++) {
    Rec x; x.before.resize(nbytes); x.after.resize(nbytes);
    if (fread(&x.op, 4, 1, f) != 1 || fread(&x.prm, 4, 1, f) != 1 || fread(x.before.data(), 1, nbytes, f) != nbytes ||
        fread(x.after.data(), 1, nbytes, f) != nbytes) { fclose(f); return false; }
    out.push_back(std::move(x));
  }
  fclose(f);
  return true;
}

static double now_ns() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e9 + t.tv_nsec; }

static void stat(const char* tname, const char* name, std::vector<double>& v) {
  if (v.empty()) { printf("| %s | %s | 0 | - | - |\n", tname, name); return; }
  std::sort(v.begin(), v.end());
  double med = v.size() & 1 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]);
  size_t k = (size_t)((v.size() * 99 + 99) / 100); if (k < 1) k = 1;
  printf("| %s | %s | %zu | %.1f | %.1f |\n", tname, name, v.size(), med / 1e3, v[k - 1] / 1e3);
}

int main(int argc, char** argv) {
  if (argc < 4) { fprintf(stderr, "usage: %s <observe|select|all> <passes> <trace>...\n", argv[0]); return 2; }
  std::string what = argv[1];
  int passes = atoi(argv[2]);
  std::vector<std::vector<Rec>> tr;
  uint32_t nbytes = 0;
  for (int i = 3; i < argc; i++) { tr.emplace_back(); if (!load(argv[i], tr.back(), nbytes)) return 2; }
  std::vector<uint8_t> ddr(nbytes + 16);
  size_t NT = tr.size();
  std::vector<std::vector<double>> t_obs(NT), t_sel(NT), t_init(NT), t_other(NT);
  long bad = 0;
  for (int p = -1; p < passes; p++) {   // p = -1 热身
    for (size_t ti = 0; ti < NT; ti++) {
      auto& t = tr[ti];
      double init_acc = 0;
      for (auto& r : t) {
        memcpy(ddr.data(), r.before.data(), nbytes);
        double t0 = now_ns();
        uint32_t st = srch_run_sw(ddr.data(), r.prm);
        double dt = now_ns() - t0;
        if ((st & 0xFF) || memcmp(ddr.data(), r.after.data(), nbytes)) bad++;
        if (p < 0) continue;
        if (r.op == OP_OBSERVE) t_obs[ti].push_back(dt);
        else if (r.op == OP_SELECT) t_sel[ti].push_back(dt);
        else if (r.op <= OP_INIT_VIEWS) init_acc += dt;
        else t_other[ti].push_back(dt);
      }
      if (p >= 0) t_init[ti].push_back(init_acc);
    }
  }
  printf("# a53_srch_bench %s passes=%d traces=%d\n\n| 向量 | 项 | 次数 | 中位数 us | p99 us |\n|---|---|---|---|---|\n", what.c_str(),
         passes, argc - 3);
  for (size_t ti = 0; ti < NT; ti++) {
    std::string b = argv[3 + ti]; size_t sl = b.find_last_of('/'); if (sl != std::string::npos) b = b.substr(sl + 1);
    const char* tn = b.c_str();
    if (what == "observe" || what == "all") stat(tn, "OBSERVE（每帧观测）", t_obs[ti]);
    if (what == "select" || what == "all") stat(tn, "SELECT（一次选点，不含路程）", t_sel[ti]);
    if (what == "all") { stat(tn, "初始化合计（INIT_MAP..INIT_VIEWS）", t_init[ti]); stat(tn, "其他（RESET/PRIOR/CELLOF/DUMP）", t_other[ti]); }
  }
  printf("\nbitexact=%s（%ld 条记录不同）\n", bad ? "false" : "true", bad);
  return bad ? 1 : 0;
}
