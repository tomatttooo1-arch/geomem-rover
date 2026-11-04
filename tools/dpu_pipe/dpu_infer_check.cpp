// DPUPIPE1 (2026-11-03): dpu_infer_check — the C++ VART engine (infer_engine.cpp) on stored input tensors, for the
// board correctness check against DPUB1's Python VART path (board_check.py):
//   * raw outputs: written to <out-dir>/<name>.out.bin (heads concatenated in runner order) and summed per head
//     (DPUB1 full1.py's img_out_sum), so they can be compared with Python VART on the same input;
//   * boxes: the C++ output processing (det_post.cpp) on those raw outputs, printed exactly (%.9g).
//   dpu_infer_check --xmodel M.xmodel --out-dir D [--repeat N] a.in.bin b.in.bin ...
// Each *.in.bin is one whole input tensor (H*W*3 int8, letterbox pad included).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "infer_engine.h"

using namespace dpupipe;

int main(int argc, char** argv) {
  std::string xmodel, out_dir = ".";
  int repeat = 1;
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--xmodel" && i + 1 < argc) xmodel = argv[++i];
    else if (a == "--out-dir" && i + 1 < argc) out_dir = argv[++i];
    else if (a == "--repeat" && i + 1 < argc) repeat = std::max(1, std::stoi(argv[++i]));
    else files.push_back(a);
  }
  auto model = VartEngine::load_model(xmodel);
  VartEngine e(model);
  printf("{\"engine\":%s}\n", e.describe().c_str());
  for (const auto& f : files) {
    std::ifstream in(f, std::ios::binary);
    std::vector<char> buf((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (buf.size() != e.input_bytes()) {
      fprintf(stderr, "%s: %zu bytes, tensor needs %zu\n", f.c_str(), buf.size(), e.input_bytes());
      return 2;
    }
    std::vector<double> stage_ms, infer_ms, post_us;
    std::vector<Det> d;
    PostTiming pt;
    for (int r = 0; r < repeat; ++r) {
      auto t0 = std::chrono::steady_clock::now();
      e.stage(reinterpret_cast<const uint8_t*>(buf.data()), buf.size());
      auto t1 = std::chrono::steady_clock::now();
      if (!e.infer()) {
        fprintf(stderr, "infer failed\n");
        return 3;
      }
      auto t2 = std::chrono::steady_clock::now();
      d = e.post(&pt);
      auto t3 = std::chrono::steady_clock::now();
      stage_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
      infer_ms.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
      post_us.push_back(std::chrono::duration<double, std::micro>(t3 - t2).count());
    }
    auto med = [](std::vector<double> v) {
      std::sort(v.begin(), v.end());
      return v[v.size() / 2];
    };
    std::string base = f.substr(f.find_last_of('/') + 1);
    if (base.size() > 7 && base.compare(base.size() - 7, 7, ".in.bin") == 0) base = base.substr(0, base.size() - 7);
    std::ofstream o(out_dir + "/" + base + ".out.bin", std::ios::binary);
    printf("{\"name\":\"%s\",\"out_sum\":[", base.c_str());
    for (size_t k = 0; k < e.heads().size(); ++k) {
      const auto& h = e.heads()[k];
      size_t n = static_cast<size_t>(h.h) * h.w * h.c;
      long long s = 0;
      for (size_t i = 0; i < n; ++i) s += h.data[i];
      o.write(reinterpret_cast<const char*>(h.data), static_cast<std::streamsize>(n));
      printf("%s%lld", k ? "," : "", s);
    }
    printf("],\"heads\":\"");
    for (size_t k = 0; k < e.heads().size(); ++k)
      printf("%s%dx%dx%dx%d", k ? "," : "", e.heads()[k].h, e.heads()[k].w, e.heads()[k].c, e.heads()[k].fix_point);
    printf("\",\"stage_ms\":%.3f,\"infer_ms\":%.3f,\"post_us\":%.1f,\"candidates\":%zu,\"dets\":[", med(stage_ms),
           med(infer_ms), med(post_us), pt.candidates);
    for (size_t i = 0; i < d.size(); ++i)
      printf("%s[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]", i ? "," : "", d[i].x1, d[i].y1, d[i].x2, d[i].y2, d[i].score,
             d[i].cls);
    printf("]}\n");
    fflush(stdout);
  }
  return 0;
}
