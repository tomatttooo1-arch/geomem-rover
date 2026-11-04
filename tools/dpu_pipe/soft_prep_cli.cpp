// NN-CAM2: SoftPrep on one YUYV file -> tensor file (host cross-check against PREP1 yuyv_ref.c + PREP0 prep_model.c).
//   soft_prep_cli <in.yuyv> <w> <h> <size> <fix_point> <kset> <ups> <out.bin> [--perturb] [--rgb out.rgb]
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "colour_prep.h"
#include "dpu_rx.h"

int main(int argc, char** argv) {
  if (argc < 9) {
    std::fprintf(stderr, "usage: soft_prep_cli in.yuyv w h size fix kset ups out.bin [--perturb] [--rgb f]\n");
    return 2;
  }
  int w = std::atoi(argv[2]), h = std::atoi(argv[3]), size = std::atoi(argv[4]), fp = std::atoi(argv[5]);
  int kset = std::atoi(argv[6]), ups = std::atoi(argv[7]);
  bool perturb = false;
  std::string rgb_out;
  for (int i = 9; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--perturb")) perturb = true;
    else if (!std::strcmp(argv[i], "--rgb") && i + 1 < argc) rgb_out = argv[++i];
  }
  std::vector<uint8_t> in(size_t(w) * h * 2);
  FILE* f = std::fopen(argv[1], "rb");
  if (!f || std::fread(in.data(), 1, in.size(), f) != in.size()) return 2;
  std::fclose(f);
  dpupipe::Geometry g = dpupipe::letterbox(w, h, size);
  dpupipe::SoftPrep p(w, h, size, g.w, g.h, g.top, g.left, fp, kset, ups);
  if (perturb) p.perturb();
  std::vector<uint8_t> out(g.tensor_bytes());
  dpupipe::PrepInput pi;
  pi.data = in.data();
  pi.w = w;
  pi.h = h;
  pi.stride = w * 2;
  pi.fourcc = dpupipe::kFourccYUYV;
  if (!p.run(pi, out.data(), out.size())) return 3;
  f = std::fopen(argv[8], "wb");
  std::fwrite(out.data(), 1, out.size(), f);
  std::fclose(f);
  if (!rgb_out.empty()) {
    f = std::fopen(rgb_out.c_str(), "wb");
    std::fwrite(p.last_rgb().data(), 1, p.last_rgb().size(), f);
    std::fclose(f);
  }
  std::printf("%s\n", p.describe_json().c_str());
  return 0;
}
