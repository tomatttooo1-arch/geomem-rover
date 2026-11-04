// 的 Engine<200,200,0x1fffffff>::scan 包成 C 接口，供 srch_harness.py 以机器人为源建全局势场。
#include "planner_hw_ref.hpp"
#include <vector>
extern "C" int pl_scan(const unsigned char* in, int sx, int sy, unsigned* pot, unsigned limit, unsigned* rounds) {
  static std::vector<unsigned char> m(40000), flags(4096);
  unsigned r = 0;
  bool ok = planner_hw_ref::detail::Engine<200, 200, 0x1fffffffu>::scan(in, sx, sy, m.data(), pot, limit, r, flags.data());
  *rounds = r;
  return ok ? 1 : 0;
}
