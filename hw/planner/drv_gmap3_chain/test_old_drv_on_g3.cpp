// PLNCHAIN0 问题 1：m5 的旧驱动（hw/planner/drv，m5 寄存器头，64 位指针写法、不碰 0x90/0x98）直接跑在 GMAP3 v9 核上，
// 势场功能还对不对。核 = 本树 hw/planner_dual/src/kernel_hls.cpp（GMAP3 v9，sha 17079dce…），指针按 32 位口解码。
// 80 档经 kv_nav 钩子 planner_drv_kv_nav_field 的势场 == 黄金。
// 两个阳性对照（证明 mock 能看出"寄存器残留"的危害）：
//   ① GMAP_OP 预置 1（假设之前有进程跑过地图操作没清）→ 旧驱动不写 0x98，核走地图分支，旧驱动拿到的不是黄金；
//   ② PATH_CTRL 预置 1（之前有进程跑过取路径没清）→ 核往 flags 缓冲写 64+1+4096 字，超出旧驱动 256 字的缓冲（oob）。
#include "planner_core.hpp"
#include "mock_g3.hpp"
#include <functional>
#include <string>
using namespace g3mock;
int main(int argc, char** argv) {
  if (argc < 3) { std::printf("usage: %s input.bin expected.bin\n", argv[0]); return 2; }
  std::vector<Case> cs;
  EXPECT(load_frozen(argv[1], argv[2], cs), "dev2 frozen-pair parsed (2 cases)");
  if (cs.size() != 2) return 1;
  Dev dev;
  std::string why;
  void* h = planner_drv::open_handle_on(std::unique_ptr<plio::Bus>(new plio::MockBus(std::ref(dev))), &why);
  EXPECT(h, "old driver on mock %s", why.c_str());
  if (!h) return 1;
  plio::MockBus* mb = static_cast<plio::MockBus*>(planner_drv::handle_core(h)->bus());
  auto run_case = [&](const Case& k, std::vector<std::uint32_t>& got) {
    const int n = width(k.mode == 1) * width(k.mode == 1), nf = k.mode == 1 ? 256 : 64;
    std::vector<std::uint32_t> field(n), flags(nf);
    std::uint32_t o3[3] = {0, 0, 0};
    const int rc = planner_drv_run(h, (int)k.mode, k.map.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy, k.k,
                                   field.data(), flags.data(), o3, nullptr);
    got = {o3[0], o3[1], o3[2]};
    got.insert(got.end(), field.begin(), field.end());
    got.insert(got.end(), flags.begin(), flags.end());
    return rc;
  };
  for (size_t c = 0; c < cs.size(); ++c) {
    const Case& k = cs[c];
    std::vector<std::uint32_t> got;
    const int rc = run_case(k, got);
    EXPECT(rc == 0 && got == k.exp && dev.oob == 0,
           "case %zu (mode %u): old driver on GMAP3 core: status %u rounds %u valid %u + field + flags == expected.bin",
           c, k.mode, got[0], got[1], got[2]);
    EXPECT(dev.path_ctrl == 0 && dev.gmap_op == 0 && dev.hi_seen == 0,
           "case %zu: core saw PATH_CTRL %u GMAP_OP %u (reset values, never written by the old driver), pointer high words 0x%x",
           c, dev.path_ctrl, dev.gmap_op, dev.hi_seen);
    if (k.mode == 0) {
      std::vector<std::uint32_t> pot(6400);
      unsigned rounds = 0;
      const int st = planner_drv_kv_nav_field(h, k.map.data(), (int)k.sx, (int)k.sy, (int)k.gx, (int)k.gy, k.k,
                                              pot.data(), &rounds);
      EXPECT(st == (int)k.exp[0] && rounds == k.exp[1] && std::equal(pot.begin(), pot.end(), k.exp.begin() + 3),
             "case %zu: kv_nav hook (what --nav pl calls): status %d rounds %u potential == golden", c, st, rounds);
    }
  }
  // 阳性对照 ①：GMAP_OP 残留 1
  {
    mb->set_reg(g3mock::A_GMAP, 1);
    std::vector<std::uint32_t> got;
    const std::uint32_t oob0 = dev.oob, g0 = dev.gmap_calls;
    const int rc = run_case(cs[0], got);
    EXPECT(!(rc == 0 && got == cs[0].exp) && dev.gmap_calls == g0 + 1,
           "control 1: stale GMAP_OP=1 -> core runs a map op instead of planning (rc %d, gmap calls +%u, oob +%u): old driver output != golden",
           rc, dev.gmap_calls - g0, dev.oob - oob0);
    mb->set_reg(g3mock::A_GMAP, 0);
  }
  // 阳性对照 ②：PATH_CTRL 残留 1
  {
    mb->set_reg(g3mock::A_PATH, 1);
    std::vector<std::uint32_t> got;
    const std::uint32_t oob0 = dev.oob;
    run_case(cs[0], got);
    EXPECT(dev.oob == oob0 + 1,
           "control 2: stale PATH_CTRL=1 -> core would write 64+1+4096 flag words, beyond the old driver's 256-word buffer (oob +%u)",
           dev.oob - oob0);
    mb->set_reg(g3mock::A_PATH, 0);
    std::vector<std::uint32_t> got2;
    EXPECT(run_case(cs[0], got2) == 0 && got2 == cs[0].exp, "registers back to 0 -> golden again");
  }
  planner_drv_close(h);
  std::printf(g_fails ? "OLD DRV ON GMAP3 CHECK FAIL (%d)\n" : "OLD DRV ON GMAP3 CHECK PASS\n", g_fails);
  return g_fails ? 1 : 0;
}
