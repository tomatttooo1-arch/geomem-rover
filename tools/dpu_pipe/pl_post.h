// RECOG641 (2026-11-04): dpu_rx --post pl -- output processing on the PL detection post-processing core (DETPOST1,
// slot 10 of the m5 bitstream) instead of the A53 (det_post.cpp). One PlPost per process (one core), shared by all
// workers (a mutex serialises the core); each worker's VART runner has its own output buffers, so each worker keeps
// its own head layout (pl_layout()).
//
// Zero copy: the core reads the three DPU output heads straight from the VART runner's own CMA output buffers
// (TensorBuffer::data_phy); the CPU does not touch (or cache-sync) the heads on this path. Head mapping: VART returns
// the round-4 640 heads as 80, 20, 40; detpost_drv::make_layout sorts them by grid (P3/P4/P5 = 80/40/20 = registers
// g80/off80, g40/off40, g20/off20), offsets relative to the lowest head (xmodel ddr: 20x20 @0, 40x40 @28000,
// 80x80 @140000, 588,000 B).
// Configuration from det_post.h PostConfig (same flags as the CPU path):
//     floor), per-class score >= thr as cls_min_q, IoU as num/den, max_det);
//   cls_thr empty (y8s512 legacy): detpost::default_config (single conf, strict >), max_det = 128 (the core's cap;
//     the CPU legacy path has no cap -- only differs when > 128 boxes survive NMS).
// Boxes come back as Det like the CPU path: coordinates Q4 / 16 (1/16 px), score = sigmoid LUT / 65535, class.
// Never refuses: make_pl_post() returning nullptr, a layout the core cannot take (e.g. rectangular 80x60 heads: the
// core has one grid register per level) or a failed frame -> the caller runs det_post.cpp (warnings rate-limited).
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "../../hw/detpost/drv/detpost_core.hpp"
#include "det_post.h"

namespace dpupipe {

// PostConfig -> core configuration (offsets/grids are per frame registers, taken from the layout).
bool pl_post_config(int nc, int fp, const PostConfig& pc, detpost::Config* cfg, std::string* why);
// IoU threshold as num/den (1..255), the exact fraction with the smallest denominator within 1e-6.
bool iou_fraction(float iou, int* num, int* den);

class PlPost {
 public:
  // bus == nullptr: the real core (sysinfo SLOT_MAP bit 10 checked first; env DPU_RX_PL_POST_WAIT=auto|irq|poll,
  // DPU_RX_PL_POST_DMA=auto|dmaheap:<heap>|dpu). Returns nullptr with why when the core cannot be used.
  static std::shared_ptr<PlPost> make(std::unique_ptr<plio::Bus> bus, int nc, int fp, const PostConfig& pc,
                                      std::string* why);
  // heads in any order (VART order) with their physical addresses -> the core's layout; false = core cannot take it.
  bool layout(const std::vector<HeadView>& heads, const std::vector<uint64_t>& phy, detpost_drv::Layout* lay,
              std::string* why) const;
  // one frame; false = this frame must go through the CPU (warning printed here, rate-limited).
  bool run(const detpost_drv::Layout& lay, std::vector<Det>* out, PostTiming* t);
  bool dead();
  std::string describe_json();
  const detpost::Config& config() const { return cfg_; }
  int timeout_ms = 50;

 private:
  int nc_ = 0, fp_ = 0;
  detpost::Config cfg_{};
  detpost_drv::BusCore core_;
  std::mutex mu_;
  uint64_t runs_ = 0, fails_ = 0, overflow_frames_ = 0;
  double core_us_sum_ = 0;
};

}  // namespace dpupipe
