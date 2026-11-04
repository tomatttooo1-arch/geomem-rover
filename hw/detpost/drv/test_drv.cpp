// 宿主单测：参数块打包与 HLS testbench 用的 dp_pack_params 逐字节相同；布局推导与前提检查（含阴性用例）。
#include <stdio.h>
#include <string.h>
#include <ap_int.h>
#include "detpost_drv.hpp"
#include "../hls1/dp_param_pack.h"

static int fails = 0;
#define EXPECT(c, msg) do { if (!(c)) { printf("FAIL %s\n", msg); fails++; } else printf("ok   %s\n", msg); } while (0)

int main() {
  // 1. 打包：两套配置（33 类 fp=2、6 类 fp=3 带下限与白名单）
  for (int t = 0; t < 2; t++) {
    detpost::Config cfg;
    detpost::default_config(&cfg, t ? 6 : 33, t ? 3 : 2, 0.25, 7, 10, 100);
    if (t) {
      detpost_drv::set_class_floor(&cfg, 1, 0.35, 3);
      detpost_drv::set_class_floor(&cfg, 2, 0.45, 3);
      cfg.cls_enable[4] = 0;
    }
    ap_uint<128> ref[69];
    dp_pack_params(cfg, ref);
    uint8_t a[detpost_drv::PRM_BYTES], b[detpost_drv::PRM_BYTES];
    detpost_drv::pack_params(cfg, a);
    for (int w = 0; w < 69; w++)
      for (int k = 0; k < 16; k++) b[w * 16 + k] = (uint8_t)ref[w].range(k * 8 + 7, k * 8).to_uint();
    EXPECT(memcmp(a, b, sizeof a) == 0, t ? "pack_params == dp_pack_params (nc=6 fp=3, floors, whitelist)"
                                          : "pack_params == dp_pack_params (nc=33 fp=2)");
  }
  // 阳性对照：改一项必须被比出来
  {
    detpost::Config cfg;
    detpost::default_config(&cfg, 33, 2, 0.25, 7, 10, 100);
    ap_uint<128> ref[69];
    dp_pack_params(cfg, ref);
    cfg.sig_lut[200] ^= 1;
    uint8_t a[detpost_drv::PRM_BYTES];
    detpost_drv::pack_params(cfg, a);
    bool diff = false;
    for (int w = 0; w < 69 && !diff; w++)
      for (int k = 0; k < 16; k++) diff |= a[w * 16 + k] != (uint8_t)ref[w].range(k * 8 + 7, k * 8).to_uint();
    EXPECT(diff, "positive control: one LUT bit flipped is detected");
  }
  // 2. 下限不可表示 → 白名单关
  {
    detpost::Config cfg;
    detpost::default_config(&cfg, 6, 3, 0.25, 7, 10, 100);
    detpost_drv::set_class_floor(&cfg, 0, 0.999999999, 3);
    EXPECT(cfg.cls_enable[0] == 0, "floor above q=127 disables the class");
    detpost_drv::set_class_floor(&cfg, 1, 0.35, 2);
    EXPECT(cfg.cls_enable[1] == 1 && cfg.cls_min_q[1] == detpost::conf_to_q(0.35, 2), "representable floor kept");
    // RECOG640: ">=" semantics -- 0.5 is exactly sigmoid(0), so q = 0 passes (strict ">" would give 1)
    detpost_drv::set_class_floor(&cfg, 2, 0.5, 2);
    EXPECT(cfg.cls_min_q[2] == 0 && detpost::conf_to_q(0.5, 2) == 1, "floor 0.5 at fp=2 -> q 0 (>=), strict conf_to_q 1");
  }
  // 3. 布局：本工程 xmodel 的 ddr_addr（20x20@0、40x40@38800、80x80@194000，nc=33），输出顺序打乱
  const uint64_t base = 0x7e000000ull;
  {
    uint64_t phy[3] = {base + 38800, base + 194000, base};
    size_t bytes[3] = {40 * 40 * 97, 80 * 80 * 97, 20 * 20 * 97};
    int grid[3] = {40, 80, 20};
    detpost_drv::Layout L;
    std::string why;
    bool ok = detpost_drv::make_layout(phy, bytes, grid, 33, &L, &why);
    EXPECT(ok && L.in_phy == base && L.off[0] == 194000 && L.off[1] == 38800 && L.off[2] == 0 &&
               L.grid[0] == 80 && L.grid[1] == 40 && L.grid[2] == 20 && L.span == 814800,
           "layout from xmodel ddr_addr (640, nc=33)");
    uint64_t p2[3] = {phy[0] + 8, phy[1] + 8, phy[2] + 8};
    EXPECT(!detpost_drv::make_layout(p2, bytes, grid, 33, &L, &why), "reject: base not 16-byte aligned");
    uint64_t p3[3] = {phy[0] + 0x80000000ull, phy[1] + 0x80000000ull, phy[2] + 0x80000000ull};
    EXPECT(!detpost_drv::make_layout(p3, bytes, grid, 33, &L, &why), "reject: above 2 GiB");
    uint64_t p4[3] = {phy[0], phy[1] + (1 << 20), phy[2]};
    EXPECT(!detpost_drv::make_layout(p4, bytes, grid, 33, &L, &why), "reject: span > 1 MiB");
    int g5[3] = {40, 80, 30};
    EXPECT(!detpost_drv::make_layout(phy, bytes, g5, 33, &L, &why), "reject: not a 2x pyramid");
    int g6[3] = {32, 64, 16};   // 512 输入
    size_t b6[3] = {32 * 32 * 70, 64 * 64 * 70, 16 * 16 * 70};
    uint64_t p6[3] = {base + 16 * 16 * 70, base + 16 * 16 * 70 + 32 * 32 * 70, base};
    ok = detpost_drv::make_layout(p6, b6, g6, 6, &L, &why);
    EXPECT(ok && L.grid[0] == 64 && L.off[0] == 16 * 16 * 70 + 32 * 32 * 70 && L.off[2] == 0, "layout 512 input nc=6");
  }
  printf("RESULT %s (%d failed)\n", fails ? "FAIL" : "PASS", fails);
  return fails != 0;
}
