// PLNPATH0 side check (not one of the preregistered comparisons): the HLS kernel source compiled
// natively (g++ + ap_int headers) on all 80-grid requests of key comparison 1, so the tolerance
// branch of the kernel code runs at least once in C (the two frozen cosim vectors have exact goals).
// Kernel header + path words must equal the C model on the software field.
//   test_kernel_native VECTOR_DIR CASES_DIR
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include "kernel_hls.hpp"
#include "path_ps.hpp"
namespace K = kv_nav_planner;
using namespace plnpath;
typedef PS<80, 80, 0xffffffu> P;
static ap_uint<8> gmap[kGmapAreaBytes];
static ap_uint<32> gfield[kGmapAreaWords], gflags[kPlnFlagsDepth];
static int nreq = 0, nbad = 0, ntol = 0, nrun = 0;
static void one(const std::vector<unsigned char>& map, K::Request q, const char* tag) {
  std::vector<std::uint8_t> m(6400);
  P::E::prepare(map.data(), q.sx, q.sy, m.data());
  const PsParams pp = P::make_params(m.data(), q, .05);
  for (int i = 0; i < 6400; ++i) gmap[i] = map[i];
  for (int i = 0; i < kPlnParamBytes; ++i) gmap[6400 + i] = pp.bytes[i];
  ap_uint<32> st, rd, va;
  const int gx = int(std::round(q.gx)), gy = int(std::round(q.gy));
  planner_field_dual(gmap, gfield, gflags, q.sx, q.sy, gx, gy, 64, st, rd, va, 0, 1, 0);
  std::vector<std::uint32_t> f(6400);
  for (int i = 0; i < 6400; ++i) f[i] = gfield[i].to_uint();
  const bool ready = st.to_uint() == 0 && va.to_uint() == 1;
  if (ready) {
    std::vector<std::uint8_t> m2(6400); std::vector<std::uint32_t> sf(6400); unsigned r = 0;
    P::E::scan(map.data(), q.sx, q.sy, m2.data(), sf.data(), 0, r, nullptr);
    if (sf != f) { ++nbad; std::printf("FIELD MISMATCH %s\n", tag); }
  }
  const ModelOut mo = model(map.data(), f.data(), 80, 0xffffffu, q.sx, q.sy, gx, gy, pp.bytes, ready);
  ++nreq; nrun += ready; ntol += ((mo.header >> 24) & 15) == 1;
  bool ok = true;
  for (size_t i = 0; i < mo.words.size(); ++i) ok = ok && gflags[64 + i].to_uint() == mo.words[i];
  if (!ok) { ++nbad; std::printf("PATH MISMATCH %s header kernel %08x model %08x\n", tag, gflags[64].to_uint(), mo.header); }
}
int main(int argc, char** argv) {
  if (argc < 3) return 2;
  for (int i = 0; i < 1000; ++i) {
    for (int set = 0; set < 2; ++set) {
      char name[8]; std::snprintf(name, sizeof name, "%03d", i);
      const std::string d = std::string(argv[1 + set]) + "/" + name + "/";
      std::ifstream mf(d + "map.u8", std::ios::binary); if (!mf) continue;
      std::vector<unsigned char> map((std::istreambuf_iterator<char>(mf)), {}); if (map.size() != 6400) continue;
      std::ifstream fr(d + "request.txt"); double sx, sy, gx, gy; fr >> sx >> sy >> gx >> gy;
      if (sx != std::floor(sx) || sy != std::floor(sy) || int(sx) < 1 || int(sy) < 1 || int(sx) > 78 || int(sy) > 78) continue;
      const int rgx = int(std::round(gx)), rgy = int(std::round(gy));
      if (rgx < 0 || rgy < 0 || rgx > 79 || rgy > 79 || map[rgy * 80 + rgx] >= 253) continue;
      char tag[16]; std::snprintf(tag, sizeof tag, "%s%s", set ? "car" : "vec", name);
      one(map, K::Request{int(sx), int(sy), gx, gy}, tag);
    }
  }
  std::printf("kernel-native requests %d | field ready %d | tolerance-goal paths %d | mismatches %d\n", nreq, nrun, ntol, nbad);
  return nbad ? 1 : 0;
}
