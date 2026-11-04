// A53 上的检测后处理计时程序（价值闸的测量仪器，README §4）。
// 两个实现：
//   ours：hw/detpost/detpost_ref.cpp（定点参考模型本身；先判阈值再解码，全整数）——「最好的软件版」
//   fast：bench/detpost_fast.cpp（同一参考模型，扫描取最大改用 NEON；输出与 ours 逐字节一致，--variant check 核对）
//   vai ：bench/vitis_yolov8_port.cpp（AMD Vitis AI Library 官方 YOLOv8 后处理原样移植）——「开箱即用版」
// 每次调用前可把这一帧的输入缓冲从缓存里清掉（--cold 1，默认），模拟 DPU 刚写完 DDR、CPU 第一次读。
// --period-ms P 时按固定周期调用（模拟识别帧率，用于与几何核并发的干扰测量）；0 = 背靠背。
//
// 编译（KV260 上）：g++ -O2 -mcpu=cortex-a53 -std=c++14 -I.. -o bench_detpost bench_detpost.cpp （接下行）
//                    vitis_yolov8_port.cpp detpost_fast.cpp ../detpost_ref.cpp ../detpost_host.cpp
// 用法：bench_detpost --variant ours|fast|vai|check --iters 2000 --cpu 3 --cold 1 --period-ms 0 --nc 33 （接下行）
//                    --out res.json f1.bin [f2.bin ...]
#include <errno.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <algorithm>
#include <string>
#include <vector>
#include "../detpost.h"
#if defined(__x86_64__)
#include <immintrin.h>
#endif

namespace detpost {
int detpost_fast(const int8_t* in, const Config& cfg, Box* out, Status* st);
}
namespace vaiport {
std::vector<std::vector<float>> yolov8_post_process(const int8_t* buf, const uint32_t offset[3], int nc, int fix_point,
                                                    float conf_thresh, float nms_thresh, size_t max_boxes_num,
                                                    size_t max_nms_num);
}

static inline uint64_t now_ns() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000000ull + t.tv_nsec;
}

static void flush_range(const void* p, size_t n) {
#if defined(__aarch64__)
  // Linux 允许用户态 DC CIVAC（SCTLR_EL1.UCI=1）；按 64 字节缓存行逐行清并作废
  const uintptr_t a0 = (uintptr_t)p & ~(uintptr_t)63, a1 = (uintptr_t)p + n;
  for (uintptr_t a = a0; a < a1; a += 64) asm volatile("dc civac, %0" ::"r"(a) : "memory");
  asm volatile("dsb ish" ::: "memory");
#elif defined(__x86_64__)
  const uintptr_t a0 = (uintptr_t)p & ~(uintptr_t)63, a1 = (uintptr_t)p + n;
  for (uintptr_t a = a0; a < a1; a += 64) _mm_clflush((const void*)a);
  _mm_mfence();
#else
  (void)p; (void)n;
#endif
}

static double pct(std::vector<double> v, double q) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = (size_t)(q * (v.size() - 1) + 0.5);
  return v[i];
}

int main(int argc, char** argv) {
  std::string variant = "ours", out_path;
  int iters = 2000, cpu = -1, cold = 1, nc = 33, fp = 2, max_det = 100, warmup = 20;
  double period_ms = 0, conf = 0.25;
  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", a.c_str()); exit(2); } return std::string(argv[++i]); };
    if (a == "--variant") variant = nx();
    else if (a == "--iters") iters = atoi(nx().c_str());
    else if (a == "--warmup") warmup = atoi(nx().c_str());
    else if (a == "--cpu") cpu = atoi(nx().c_str());
    else if (a == "--cold") cold = atoi(nx().c_str());
    else if (a == "--period-ms") period_ms = atof(nx().c_str());
    else if (a == "--nc") nc = atoi(nx().c_str());
    else if (a == "--fp") fp = atoi(nx().c_str());
    else if (a == "--conf") conf = atof(nx().c_str());
    else if (a == "--max-det") max_det = atoi(nx().c_str());
    else if (a == "--out") out_path = nx();
    else files.push_back(a);
  }
  if (files.empty() || (variant != "ours" && variant != "fast" && variant != "vai" && variant != "check")) {
    fprintf(stderr, "need --variant ours|fast|vai|check and frames\n");
    return 2;
  }
  if (cpu >= 0) {
    cpu_set_t s; CPU_ZERO(&s); CPU_SET(cpu, &s);
    if (sched_setaffinity(0, sizeof s, &s)) { perror("sched_setaffinity"); return 1; }
  }
  static detpost::Config cfg;
  detpost::default_config(&cfg, nc, fp, conf, 7, 10, max_det);
  const size_t ch = 64 + nc;
  const size_t need = (size_t)(20 * 20 + 40 * 40 + 80 * 80) * ch;
  std::vector<std::vector<int8_t>> frames;
  for (auto& f : files) {
    FILE* fh = fopen(f.c_str(), "rb");
    if (!fh) { perror(f.c_str()); return 1; }
    std::vector<int8_t> b(need);
    size_t r = fread(b.data(), 1, need, fh);
    fclose(fh);
    if (r != need) { fprintf(stderr, "%s: %zu bytes, need %zu\n", f.c_str(), r, need); return 1; }
    frames.push_back(std::move(b));
  }
  static detpost::Box boxes[detpost::MAX_DET_MAX];
  if (variant == "check") {  // 快速版与参考模型逐字节核对（框表与状态）
    static detpost::Box b2[detpost::MAX_DET_MAX];
    int bad = 0;
    for (size_t i = 0; i < frames.size(); i++) {
      detpost::Status s1, s2;
      memset(boxes, 0, sizeof boxes); memset(b2, 0, sizeof b2);
      int n1 = detpost::detpost_ref(frames[i].data(), cfg, boxes, &s1);
      int n2 = detpost::detpost_fast(frames[i].data(), cfg, b2, &s2);
      if (n1 != n2 || memcmp(boxes, b2, sizeof boxes) || memcmp(&s1, &s2, sizeof s1)) { bad++; fprintf(stderr, "MISMATCH %s\n", files[i].c_str()); }
    }
    printf("{\"check\": \"fast_vs_ref\", \"frames\": %zu, \"mismatch\": %d}\n", frames.size(), bad);
    return bad ? 1 : 0;
  }
  std::vector<double> t_ms;
  std::vector<int> nbox;
  t_ms.reserve(iters);
  const uint64_t period_ns = (uint64_t)(period_ms * 1e6);
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  long overruns = 0;
  for (int it = -warmup; it < iters; it++) {
    const auto& fr = frames[(size_t)(it + warmup) % frames.size()];
    if (cold) flush_range(fr.data(), fr.size());
    const uint64_t t0 = now_ns();
    int n;
    if (variant == "ours") {
      n = detpost::detpost_ref(fr.data(), cfg, boxes, nullptr);
    } else if (variant == "fast") {
      n = detpost::detpost_fast(fr.data(), cfg, boxes, nullptr);
    } else {
      auto r = vaiport::yolov8_post_process(fr.data(), cfg.offset, nc, fp, (float)conf, 0.7f, 30000, (size_t)max_det);
      n = (int)r.size();
    }
    const uint64_t t1 = now_ns();
    if (it >= 0) { t_ms.push_back((t1 - t0) / 1e6); nbox.push_back(n); }
    if (period_ns) {
      next.tv_nsec += period_ns % 1000000000ull;
      next.tv_sec += period_ns / 1000000000ull + next.tv_nsec / 1000000000;
      next.tv_nsec %= 1000000000;
      timespec cur; clock_gettime(CLOCK_MONOTONIC, &cur);
      if (cur.tv_sec > next.tv_sec || (cur.tv_sec == next.tv_sec && cur.tv_nsec > next.tv_nsec)) { overruns++; next = cur; }
      else while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr) == EINTR) {}
    }
  }
  double sum = 0; for (double v : t_ms) sum += v;
  long bsum = 0; for (int v : nbox) bsum += v;
  char js[1024];
  snprintf(js, sizeof js,
           "{\"variant\": \"%s\", \"iters\": %zu, \"frames\": %zu, \"cpu\": %d, \"cold\": %d, \"period_ms\": %.3f, "
           "\"overruns\": %ld, \"ms_p50\": %.4f, \"ms_p90\": %.4f, \"ms_p99\": %.4f, \"ms_max\": %.4f, \"ms_mean\": %.4f, "
           "\"boxes_mean\": %.2f, \"cls_thr_q\": %d}",
           variant.c_str(), t_ms.size(), frames.size(), cpu, cold, period_ms, overruns, pct(t_ms, .5), pct(t_ms, .9),
           pct(t_ms, .99), pct(t_ms, 1.0), sum / t_ms.size(), (double)bsum / nbox.size(), cfg.cls_thr_q);
  printf("%s\n", js);
  if (!out_path.empty()) {
    FILE* o = fopen(out_path.c_str(), "w");
    if (!o) { perror(out_path.c_str()); return 1; }
    fprintf(o, "{\"summary\": %s, \"ms\": [", js);
    for (size_t i = 0; i < t_ms.size(); i++) fprintf(o, "%s%.4f", i ? ", " : "", t_ms[i]);
    fprintf(o, "]}\n");
    fclose(o);
  }
  return 0;
}
