// DRV5 (2026-11-04): dpu_rx --prep pl on the PL pre-processing core (see colour_prep.h, pl_prep.h).
#include "pl_prep.h"

#include <cstdio>
#include <cstdlib>

#include "dpu_rx.h"

namespace dpupipe {

prep_drv::Config pl_prep_config(int src_w, int src_h, int size, int fix_point, int kset, int ups) {
  const Geometry g = letterbox(src_w, src_h, size);
  prep_drv::Config c;
  c.w0 = src_w;
  c.h0 = src_h;
  c.stride = (src_w * 2 + 15) / 16 * 16;
  c.fmt = prep_drv::YUYV;
  c.ups = ups ? 1 : 0;
  c.bypass = (g.w == src_w && g.h == src_h) ? 1 : 0;
  c.tw = c.th = size;
  c.ow = g.w;
  c.oh = g.h;
  c.top = g.top;
  c.left = g.left;
  c.fp = fix_point;
  c.kset = kset ? 1 : 0;
  return c;
}

namespace {

class PlPrep : public ColourPrep {
 public:
  PlPrep(const prep_drv::Config& c, int size, int kset, int ups) : cfg_(c), size_(size), kset_(kset), ups_(ups) {}
  prep_drv::Core core;
  const char* name() const override { return core.dead() ? "pl->soft" : "pl"; }
  bool run(const PrepInput& in, uint8_t* tensor, size_t tensor_bytes) override {
    if (in.fourcc != kFourccYUYV || in.w != cfg_.w0 || in.h != cfg_.h0 || tensor_bytes != prep_drv::tensor_bytes(cfg_))
      return false;
    if (!core.dead()) {
      std::string why;
      prep_drv::Result r;
      if (core.run(in.data, in.stride, tensor, tensor_bytes, &r, &why)) {
        last_ = r;
        return true;
      }
      ++pl_fail_;   // this frame goes through SoftPrep; a hung core (#14) is never started again
      if (pl_fail_ <= 5 || pl_fail_ % 100 == 0 || core.dead())   // rate-limited
        std::fprintf(stderr, "{\"warning\":\"dpu_rx pl prep (failure %d): %s; %s\"}\n", pl_fail_, why.c_str(),
                     core.dead() ? "SoftPrep from now on" : "SoftPrep for this frame");
    }
    if (!soft_)
      soft_.reset(new SoftPrep(cfg_.w0, cfg_.h0, size_, cfg_.ow, cfg_.oh, cfg_.top, cfg_.left, cfg_.fp, kset_, ups_));
    ++soft_frames_;
    return soft_->run(in, tensor, tensor_bytes);
  }
  std::string describe_json() const override {
    char b[640];
    std::snprintf(b, sizeof b,
                  "{\"impl\":\"%s\",\"src\":\"%dx%d\",\"size\":%d,\"fix_point\":%d,\"kset\":%d,\"ups\":%d,"
                  "\"content\":\"%dx%d\",\"top\":%d,\"left\":%d,\"bypass\":%d,\"pl_fail\":%d,\"soft_frames\":%d,\"drv\":%s}",
                  name(), cfg_.w0, cfg_.h0, size_, cfg_.fp, kset_, ups_, cfg_.ow, cfg_.oh, cfg_.top, cfg_.left,
                  cfg_.bypass, pl_fail_, soft_frames_, core.describe().c_str());
    return b;
  }
  prep_drv::Result last_;

 private:
  prep_drv::Config cfg_;
  int size_, kset_, ups_;
  int pl_fail_ = 0, soft_frames_ = 0;
  std::unique_ptr<SoftPrep> soft_;
};

}  // namespace

std::unique_ptr<ColourPrep> make_pl_prep_on(std::unique_ptr<plio::Bus> bus, int src_w, int src_h, int size,
                                            int fix_point, int kset, int ups, std::string* why) {
  const prep_drv::Config c = pl_prep_config(src_w, src_h, size, fix_point, kset, ups);
  std::string e;
  if (!prep_drv::check_config(c, &e)) {
    if (why) *why = "pl prep geometry: " + e;
    return nullptr;
  }
  std::unique_ptr<PlPrep> p(new PlPrep(c, size, kset, ups));
  bool ok;
  if (bus) {
    ok = p->core.open(std::move(bus), &e);
  } else {
    plio::Options o;
    if (const char* w = std::getenv("DPU_RX_PL_PREP_WAIT")) o.wait = w;
    if (const char* d = std::getenv("DPU_RX_PL_PREP_DMA")) o.provider = d;
    ok = p->core.open(o, &e);
  }
  if (!ok || !p->core.configure(c, &e)) {
    if (why) *why = "pl prep driver: " + e;
    return nullptr;
  }
  return std::unique_ptr<ColourPrep>(p.release());
}

std::unique_ptr<ColourPrep> make_pl_prep(int src_w, int src_h, int size, int fix_point, int kset, int ups,
                                         std::string* why) {
  return make_pl_prep_on(nullptr, src_w, src_h, size, fix_point, kset, ups, why);
}

prep_drv::Core* pl_prep_core(ColourPrep* p) {
  auto* q = dynamic_cast<PlPrep*>(p);
  return q ? &q->core : nullptr;
}

}  // namespace dpupipe
