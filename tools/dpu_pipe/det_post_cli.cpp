// DPUPIPE1 (2026-11-03): det_post_cli — run the C++ output processing (det_post.cpp) on raw DPU output heads
// stored in a file, for the value-by-value comparison with the Python reference (tests, board check).
//
//   det_post_cli --size 512 --heads 64x64x70x2,32x32x70x2,16x16x70x2 --in outs.bin [--repeat N]
//   outs.bin = the heads' int8 bytes concatenated in the --heads order (each h*w*c, NHWC).
// RECOG640: [--conf F] [--iou F] [--max-det N] [--cls-thr t0,t1,..] [--sort-heads 1] (det_post.h PostConfig;
//   absent = DPUPIPE1 behaviour, bit for bit).
// Prints {"dets":[[x1,y1,x2,y2,score,cls],...] (%.9g, exact float32), "candidates":n, "us_median":t}.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "det_post.h"

using namespace dpupipe;

int main(int argc, char** argv) {
  int size = 0, repeat = 1;
  std::string heads_s, in;
  PostConfig pc;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string a = argv[i], v = argv[i + 1];
    if (a == "--size") size = std::atoi(v.c_str());
    else if (a == "--heads") heads_s = v;
    else if (a == "--in") in = v;
    else if (a == "--repeat") repeat = std::max(1, std::atoi(v.c_str()));
    else if (a == "--conf") pc.conf = std::stof(v);
    else if (a == "--iou") pc.iou = std::stof(v);
    else if (a == "--max-det") pc.max_det = std::atoi(v.c_str());
    else if (a == "--sort-heads") pc.sort_heads = std::atoi(v.c_str()) != 0;
    else if (a == "--cls-thr") {
      std::stringstream ts(v);
      std::string x;
      while (std::getline(ts, x, ',')) pc.cls_thr.push_back(std::stof(x));
    }
    else {
      fprintf(stderr, "unknown %s\n", a.c_str());
      return 2;
    }
  }
  std::ifstream f(in, std::ios::binary);
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::vector<HeadView> heads;
  size_t off = 0;
  std::stringstream ss(heads_s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    HeadView h{};
    if (sscanf(item.c_str(), "%dx%dx%dx%d", &h.h, &h.w, &h.c, &h.fix_point) != 4) {
      fprintf(stderr, "bad head %s\n", item.c_str());
      return 2;
    }
    size_t n = static_cast<size_t>(h.h) * h.w * h.c;
    if (off + n > buf.size()) {
      fprintf(stderr, "input too short\n");
      return 2;
    }
    h.data = reinterpret_cast<const int8_t*>(buf.data() + off);
    off += n;
    heads.push_back(h);
  }
  if (off != buf.size()) {
    fprintf(stderr, "input size %zu != heads %zu\n", buf.size(), off);
    return 2;
  }
  std::vector<double> us;
  std::vector<Det> d;
  PostTiming t;
  for (int r = 0; r < repeat; ++r) {
    auto t0 = std::chrono::steady_clock::now();
    d = postprocess_cfg(heads, size, pc, &t);
    us.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
  }
  std::sort(us.begin(), us.end());
  printf("{\"candidates\":%zu,\"us_median\":%.1f,\"dets\":[", t.candidates, us[us.size() / 2]);
  for (size_t i = 0; i < d.size(); ++i)
    printf("%s[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]", i ? "," : "", d[i].x1, d[i].y1, d[i].x2, d[i].y2, d[i].score,
           d[i].cls);
  printf("]}\n");
  return 0;
}
