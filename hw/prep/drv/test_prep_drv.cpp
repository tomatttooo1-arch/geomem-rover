// DRV5 宿主对照（x86 与 qemu-aarch64）：前处理驱动在 DMA 参数块缓冲里写出的字节 == PREP1 s18 联仿时 RTL 读到的参数块，
// 两组配置：方 512（RGB888 源 640x480，走驱动通用接口）、YUYV→416（走 dpu_rx 的 make_pl_prep 同一路：letterbox → 配置）。
// mock 总线在 ap_start 时把参数块快照下来，再用核的 HLS C 源（prep_k.cpp）在 mock 内存上跑一帧，张量对 PREP1 联仿黄金。
// 阳性对照：系数 +1 的参数块必须与黄金不同；核不回 ap_done 时驱动必须判挂死、之后不再启动，dpu_rx 适配层改走 SoftPrep。
//   test_prep_drv <golden prm dir> <vec_cosim.txt> <golden tensor dir>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../hls1/prep_k.h"
#include "../../../tools/dpu_pipe/pl_prep.h"
#include "../../m7/pl/sw/m7pl_prep_regs.h"
#include "prep_core.hpp"

static int fails = 0;
#define EXPECT(c, ...) do { if (!(c)) { std::printf("FAIL "); fails++; } else std::printf("ok   "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)

static std::vector<uint8_t> slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

struct Vec { std::string id, path; int fmt, ups, bypass, w0, h0, stride, tw, th, ow, oh, top, left, fp, kset; };
static std::vector<Vec> read_vec(const std::string& p) {
  std::vector<Vec> v;
  std::ifstream f(p);
  std::string l;
  while (std::getline(f, l)) {
    if (l.empty() || l[0] == '#') continue;
    std::istringstream s(l);
    Vec x;
    s >> x.id >> x.path >> x.fmt >> x.ups >> x.bypass >> x.w0 >> x.h0 >> x.stride >> x.tw >> x.th >> x.ow >> x.oh >> x.top >>
        x.left >> x.fp >> x.kset;
    v.push_back(x);
  }
  return v;
}

// mock 上的核：按寄存器里的物理地址取 mock 内存，跑 prep_top，写回张量与 STATUS（同 tb_prep.cpp 的字节↔字转换）
struct Dev {
  std::vector<uint8_t> prm_seen;
  uint32_t* last_status;
  void operator()(plio::MockBus& m) {
    const uint64_t sp = m.reg64(XPREP_TOP_CONTROL_ADDR_SRC_DATA), dp = m.reg64(XPREP_TOP_CONTROL_ADDR_DST_DATA),
                   pp = m.reg64(XPREP_TOP_CONTROL_ADDR_PRM_DATA);
    const uint8_t* prm = (const uint8_t*)m.va_of(pp, prep_drv::PRM_BYTES);
    prm_seen.assign(prm, prm + prep_drv::PRM_BYTES);
    std::vector<word_t> p(prep_drv::PRM_WORDS), s(2048L * 2048 * 3 / 16), d(640L * 640 * 3 / 16);
    auto to_w = [](const uint8_t* b) { word_t w = 0; for (int i = 0; i < 16; i++) w(8 * i + 7, 8 * i) = b[i]; return w; };
    for (int i = 0; i < prep_drv::PRM_WORDS; ++i) p[i] = to_w(prm + 16 * i);
    // 源字数、张量字数取自帧头（字 1 [22:0]、字 0 [112:96]），即核自己会读写的范围
    const long nsrc = (long)(p[1].range(22, 0).to_uint()), ndst = (long)(p[0].range(112, 96).to_uint());
    const uint8_t* src = (const uint8_t*)m.va_of(sp, nsrc * 16);
    uint8_t* dst = (uint8_t*)m.va_of(dp, ndst * 16);
    if (!src || !dst) { std::printf("MOCK: core address outside the driver's buffers\n"); return; }
    for (long i = 0; i < nsrc; ++i) s[i] = to_w(src + 16 * i);
    ap_uint<32> st = 0;
    prep_top(s.data(), d.data(), p.data(), &st);
    for (long i = 0; i < ndst; ++i)
      for (int k = 0; k < 16; ++k) dst[16 * i + k] = (uint8_t)d[i](8 * k + 7, 8 * k).to_uint();
    m.set_reg(XPREP_TOP_CONTROL_ADDR_STATUS_DATA, st.to_uint());
    m.set_reg(XPREP_TOP_CONTROL_ADDR_STATUS_CTRL, 1);
    *last_status = st.to_uint();
  }
};

int main(int argc, char** argv) {
  if (argc != 4) { std::fprintf(stderr, "usage: %s <golden prm dir> <vec_cosim.txt> <golden tensor dir>\n", argv[0]); return 2; }
  const std::string gprm = argv[1], gten = argv[3];
  const auto vecs = read_vec(argv[2]);
  EXPECT(vecs.size() == 2, "vec_cosim.txt has the 2 co-simulated frames");
  uint32_t last_status = 0;   // 硬件 STATUS 保持上一帧的值（核内帧计数在整个进程里连续）
  for (const auto& v : vecs) {
    const auto golden_prm = slurp(gprm + "/" + v.id + ".prm.bin");
    const auto golden_ten = slurp(gten + "/" + v.id + ".bin");
    const auto raw = slurp(v.path);
    EXPECT(golden_prm.size() == prep_drv::PRM_BYTES && !golden_ten.empty() && !raw.empty(), "%s: golden files read", v.id.c_str());
    prep_drv::Config c;
    c.w0 = v.w0; c.h0 = v.h0; c.stride = v.stride; c.fmt = v.fmt; c.ups = v.ups; c.bypass = v.bypass;
    c.tw = v.tw; c.th = v.th; c.ow = v.ow; c.oh = v.oh; c.top = v.top; c.left = v.left; c.fp = v.fp; c.kset = v.kset;
    Dev dev;
    dev.last_status = &last_status;
    std::unique_ptr<plio::MockBus> bus(new plio::MockBus(std::ref(dev)));
    bus->set_reg(XPREP_TOP_CONTROL_ADDR_STATUS_DATA, last_status);
    plio::MockBus* mb = bus.get();
    std::vector<uint8_t> out(prep_drv::tensor_bytes(c));
    std::string why;
    bool ok;
    const int rowb = v.w0 * prep_drv::bytes_per_pixel(v.fmt);
    std::unique_ptr<dpupipe::ColourPrep> cp;
    prep_drv::Core core;   // lives to the end of the case: it owns the mock bus mb points into
    if (v.fmt == prep_drv::YUYV) {
      // dpu_rx 路径：make_pl_prep_on 从 (640,480,416,fp,kset,ups) 自己推配置，必须推出与联仿向量相同的几何
      const prep_drv::Config d = dpupipe::pl_prep_config(v.w0, v.h0, v.tw, v.fp, v.kset, v.ups);
      EXPECT(d.ow == c.ow && d.oh == c.oh && d.top == c.top && d.left == c.left && d.stride == c.stride &&
             d.bypass == c.bypass && d.fmt == c.fmt && d.ups == c.ups,
             "%s: dpu_rx letterbox -> config == co-sim vector (content %dx%d at %d,%d stride %d)", v.id.c_str(), d.ow, d.oh,
             d.top, d.left, d.stride);
      cp = dpupipe::make_pl_prep_on(std::move(bus), v.w0, v.h0, v.tw, v.fp, v.kset, v.ups, &why);
      EXPECT(cp != nullptr, "%s: make_pl_prep_on opened the (mock) core %s", v.id.c_str(), why.c_str());
      if (!cp) continue;
      dpupipe::PrepInput in;
      in.data = raw.data(); in.w = v.w0; in.h = v.h0; in.stride = rowb; in.fourcc = dpupipe::kFourccYUYV;
      ok = cp->run(in, out.data(), out.size());
      EXPECT(ok && std::string(cp->name()) == "pl", "%s: ColourPrep pl ran the core", v.id.c_str());
    } else {
      ok = core.open(std::move(bus), &why) && core.configure(c, &why);
      EXPECT(ok, "%s: driver open+configure %s", v.id.c_str(), why.c_str());
      prep_drv::Result r;
      ok = ok && core.run(raw.data(), rowb, out.data(), out.size(), &r, &why);
      EXPECT(ok, "%s: driver run %s status=%08x", v.id.c_str(), why.c_str(), r.status);
    }
    EXPECT(dev.prm_seen == golden_prm, "%s: parameter block in the DMA buffer == co-sim RTL input, %zu bytes", v.id.c_str(),
           golden_prm.size());
    EXPECT(out == golden_ten, "%s: tensor (C core on the driver's buffers) == co-sim golden, %zu bytes", v.id.c_str(),
           golden_ten.size());
    EXPECT(mb->starts == 1, "%s: one ap_start", v.id.c_str());
    // 阳性对照 1：系数 +1 的参数块必须与黄金不同（比较器能看见 1 LSB）
    std::vector<uint8_t> pert(prep_drv::PRM_BYTES);
    EXPECT(prep_drv::build_params(c, pert.data(), &why, 1) && pert != golden_prm, "%s: control: perturbed block differs",
           v.id.c_str());
    if (v.fmt == prep_drv::YUYV) {
      // 阳性对照 2：核挂死（不回 ap_done）→ 驱动判 #14、不再启动；适配层这一帧及以后走 SoftPrep，张量仍对黄金
      mb->hang_next = true;
      std::vector<uint8_t> out2(out.size(), 0);
      dpupipe::PrepInput in;
      in.data = raw.data(); in.w = v.w0; in.h = v.h0; in.stride = rowb; in.fourcc = dpupipe::kFourccYUYV;
      ok = cp->run(in, out2.data(), out2.size());
      prep_drv::Core* pc = dpupipe::pl_prep_core(cp.get());
      EXPECT(pc && pc->dead() && std::string(cp->name()) == "pl->soft", "%s: control: hung core detected (#14)",
             v.id.c_str());
      EXPECT(ok && out2 == golden_ten, "%s: after the hang the frame comes from SoftPrep and still == golden", v.id.c_str());
      const int starts = mb->starts;
      ok = cp->run(in, out2.data(), out2.size());
      EXPECT(ok && mb->starts == starts, "%s: hung core never started again", v.id.c_str());
    }
  }
  // 阴性：违反 §6.4 的配置一律拒绝（各挑一条）
  {
    prep_drv::Config c;
    c.w0 = 640; c.h0 = 480; c.stride = 1288; c.fmt = prep_drv::YUYV; c.tw = c.th = 416; c.ow = 416; c.oh = 312; c.top = 52; c.fp = 7;
    std::string why;
    EXPECT(!prep_drv::check_config(c, &why) && why.find("#6") == 0, "reject stride not multiple of 16 (%s)", why.c_str());
    c.stride = 1280; c.ow = 700;
    EXPECT(!prep_drv::check_config(c, &why), "reject OW > 640 (%s)", why.c_str());
    c.ow = 416; c.tw = c.th = 418;
    EXPECT(!prep_drv::check_config(c, &why) && why.find("#5") == 0, "reject TW*TH*3 %% 16 != 0 (%s)", why.c_str());
    c.tw = c.th = 416; c.w0 = 641; c.stride = 1296;
    EXPECT(!prep_drv::check_config(c, &why), "reject odd YUYV width (%s)", why.c_str());
  }
  std::printf(fails ? "PREP DRV CHECK FAIL (%d)\n" : "PREP DRV CHECK PASS\n", fails);
  return fails ? 1 : 0;
}
