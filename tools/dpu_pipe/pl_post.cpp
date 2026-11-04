// RECOG641 (2026-11-04): see pl_post.h.
#include "pl_post.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace dpupipe {

bool iou_fraction(float iou, int* num, int* den) {
  for (int d = 1; d <= 255; ++d) {
    const int n = static_cast<int>(std::lround(static_cast<double>(iou) * d));
    if (n >= 1 && n <= 255 && std::fabs(static_cast<double>(n) / d - static_cast<double>(iou)) < 1e-6) {
      *num = n;
      *den = d;
      return true;
    }
  }
  return false;
}

bool pl_post_config(int nc, int fp, const PostConfig& pc, detpost::Config* cfg, std::string* why) {
  if (nc < 1 || nc > detpost::NC_MAX) { *why = "nc " + std::to_string(nc) + " outside 1..40"; return false; }
  int num = 0, den = 0;
  if (!iou_fraction(pc.iou, &num, &den)) { *why = "IoU threshold not a fraction n/d with d <= 255"; return false; }
  const int max_det = pc.max_det > 0 ? (pc.max_det > detpost_drv::MAX_DET ? detpost_drv::MAX_DET : pc.max_det)
                                     : detpost_drv::MAX_DET;
  if (pc.max_det > detpost_drv::MAX_DET) { *why = "max_det > 128 (the core's cap)"; return false; }
  const uint32_t off[3] = {0, 0, 0};
  const int gw[3] = {80, 40, 20}, gh[3] = {80, 40, 20};   // placeholders: the per-frame layout writes the registers
  if (!pc.cls_thr.empty()) {
    if (pc.cls_thr.size() != static_cast<size_t>(nc)) { *why = "cls_thr has a value count != nc"; return false; }
    double thr[detpost::NC_MAX];
    for (int c = 0; c < nc; ++c) thr[c] = pc.cls_thr[c];
    detpost::recog_config(cfg, nc, fp, pc.conf, thr, num, den, max_det, off, gw, gh);
  } else {
    detpost::default_config(cfg, nc, fp, pc.conf, num, den, max_det);
  }
  return true;
}

std::shared_ptr<PlPost> PlPost::make(std::unique_ptr<plio::Bus> bus, int nc, int fp, const PostConfig& pc,
                                     std::string* why) {
  std::shared_ptr<PlPost> p(new PlPost);
  p->nc_ = nc;
  p->fp_ = fp;
  if (!pl_post_config(nc, fp, pc, &p->cfg_, why)) return nullptr;
  std::string e;
  bool ok;
  if (bus) {
    ok = p->core_.open(std::move(bus), &e);
  } else {
    if (!detpost_drv::core_in_bitstream(&e)) {
      *why = "pl post: " + e;
      return nullptr;
    }
    plio::Options o;
    if (const char* w = std::getenv("DPU_RX_PL_POST_WAIT")) o.wait = w;
    if (const char* d = std::getenv("DPU_RX_PL_POST_DMA")) o.provider = d;
    ok = p->core_.open(o, &e);
  }
  if (!ok || !p->core_.configure(p->cfg_, &e)) {
    *why = "pl post driver: " + e;
    return nullptr;
  }
  return p;
}

bool PlPost::layout(const std::vector<HeadView>& heads, const std::vector<uint64_t>& phy, detpost_drv::Layout* lay,
                    std::string* why) const {
  if (heads.size() != 3 || phy.size() != 3) { *why = "expected 3 output heads"; return false; }
  uint64_t p[3];
  size_t bytes[3];
  int grid[3];
  for (int i = 0; i < 3; ++i) {
    const HeadView& h = heads[i];
    if (h.c != 4 * detpost::REG_MAX + nc_) { *why = "head channels " + std::to_string(h.c) + " != 64 + nc"; return false; }
    if (h.fix_point != fp_) { *why = "head fix_point differs from the configured one"; return false; }
    if (h.h != h.w) {
      *why = "rectangular head " + std::to_string(h.w) + "x" + std::to_string(h.h) +
             ": the core has one grid register per level (square grids only)";
      return false;
    }
    p[i] = phy[i];
    bytes[i] = static_cast<size_t>(h.h) * h.w * h.c;
    grid[i] = h.h;
  }
  return detpost_drv::make_layout(p, bytes, grid, nc_, lay, why);
}

bool PlPost::dead() {
  std::lock_guard<std::mutex> lk(mu_);
  return core_.dead();
}

bool PlPost::run(const detpost_drv::Layout& lay, std::vector<Det>* out, PostTiming* t) {
  static thread_local detpost_drv::RunResult r;
  std::string why;
  bool ok;
  {
    std::lock_guard<std::mutex> lk(mu_);
    ok = core_.run(lay, &r, timeout_ms, &why);
    ++runs_;
    if (!ok) {
      ++fails_;
      if (fails_ <= 5 || fails_ % 100 == 0 || core_.dead())
        std::fprintf(stderr, "{\"warning\":\"dpu_rx pl post (failure %llu): %s; %s\"}\n",
                     (unsigned long long)fails_, why.c_str(),
                     core_.dead() ? "CPU output processing from now on" : "CPU for this frame");
      return false;
    }
    if (r.st.overflow) ++overflow_frames_;
    core_us_sum_ += r.core_ns / 1e3;
  }
  out->clear();
  out->reserve(r.n);
  for (int i = 0; i < r.n; ++i) {
    const detpost::Box& b = r.box[i];
    out->push_back({b.x1 / 16.0f, b.y1 / 16.0f, b.x2 / 16.0f, b.y2 / 16.0f, b.score / 65535.0f,
                    static_cast<float>(b.cls)});
  }
  if (t) {
    *t = PostTiming();
    t->pl = true;
    t->pl_us = r.total_ns / 1e3;
    t->core_us = r.core_ns / 1e3;
    t->candidates = static_cast<size_t>(r.st.n_cand < 0 ? 0 : r.st.n_cand);
    t->overflow = r.st.overflow;
  }
  return true;
}

std::string PlPost::describe_json() {
  std::lock_guard<std::mutex> lk(mu_);
  char b[512];
  std::snprintf(b, sizeof b,
                "{\"impl\":\"pl\",\"nc\":%d,\"fp\":%d,\"cls_thr_q\":%d,\"iou\":\"%d/%d\",\"max_det\":%d,\"runs\":%llu,"
                "\"fails\":%llu,\"overflow_frames\":%llu,\"core_us_mean\":%.1f,\"cls_min_q\":[",
                nc_, fp_, cfg_.cls_thr_q, cfg_.iou_num, cfg_.iou_den, cfg_.max_det, (unsigned long long)runs_,
                (unsigned long long)fails_, (unsigned long long)overflow_frames_,
                runs_ > fails_ ? core_us_sum_ / double(runs_ - fails_) : -1.0);
  std::string s = b;
  for (int c = 0; c < nc_; ++c) s += (c ? "," : "") + std::to_string(cfg_.cls_min_q[c]);
  return s + "],\"drv\":" + core_.describe() + "}";
}

}  // namespace dpupipe
