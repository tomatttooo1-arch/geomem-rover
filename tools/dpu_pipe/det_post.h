// DPUPIPE1 (2026-11-03): CPU output processing of the six-class YOLOv8s DPU xmodels on the KV260 A53.
// Same arithmetic as DPUB1 full1.py (the "threshold first" fast path, dfl_decode_f + nms):
//   fix2float      v = int8 * 2^-fix_point                     (float32, exact)
//   threshold      max class logit > logit(0.25) on raw logits  (no sigmoid on rejected cells)
//   DFL            softmax over 16 bins per side, expectation   (float32; numpy's 8-accumulator pairwise sum order)
//   decode         x1 = (gx - l) * stride, ...                  (gx = col + 0.5, stride = input / head size)
//   score          sigmoid(max logit), class = first argmax
//   NMS            greedy, class-offset trick (box + cls * 4096), IoU > 0.7 suppressed, no max_det, no clipping
// Output heads: NHWC int8 (1, h, w, 64 + nc), contiguous; heads concatenated in the order given.
// Tie order: candidates are sorted by score descending with a STABLE sort (concatenation order breaks ties);
// numpy's default argsort is not stable, so the Python reference used for exact comparison passes kind="stable".
// Build with -ffp-contract=off (no fused multiply-add), otherwise aarch64 g++ contracts a*b+c and the float32
// values drift from numpy's.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dpupipe {

struct Det {
  float x1, y1, x2, y2, score, cls;
};

struct HeadView {
  const int8_t* data;   // (h, w, c) int8, contiguous
  int h, w, c;          // c = 64 + num_classes
  int fix_point;
};

struct PostTiming {
  double fix_decode_us = 0, nms_us = 0;
  size_t candidates = 0;
  // RECOG641: PL core path (pl_post.h): pl_us = whole core call (registers + core + box read back), core_us = ap_start
  // to ap_done; pl = this frame went through the core; overflow = the core's candidate buffer (256) overflowed
  double pl_us = 0, core_us = 0;
  bool pl = false;
  int overflow = 0;
};

// input_size: the DPU input width (640 / 512 / 416; 640 for the 640x480 rectangular model); stride of a head =
// input_size / w (integer division; RECOG640 changed h -> w, same value for square inputs).
std::vector<Det> decode_fast(const std::vector<HeadView>& heads, int input_size, float conf = 0.25f);
std::vector<Det> nms_greedy(const std::vector<Det>& d, float iou_thr = 0.7f);
std::vector<Det> postprocess(const std::vector<HeadView>& heads, int input_size, PostTiming* t = nullptr,
                             float conf = 0.25f, float iou_thr = 0.7f);

// multi_label=False, max_det=100, nc) on the decoded heads, THEN per-class "score >= thr" on those detections).
// Default-constructed = the DPUPIPE1 behaviour above (single conf 0.25 before NMS, no max_det, heads as given),
// and postprocess_cfg() then runs exactly postprocess().
//   With cls_thr non-empty: the candidate gate is score > floor AND score >= min(cls_thr). This is exact w.r.t.
//   "floor -> NMS -> top max_det -> per-class >=": NMS is per class and greedy by score, so a box with score
//   s >= t_c is only ever suppressed by same-class boxes scoring >= s (all >= min thr, so all still present), and
//   its max_det rank only counts boxes scoring >= s (same argument). Boxes below min(cls_thr) can never be output.
//   sort_heads: heads ordered by grid size descending (P3 80x80, P4 40x40, P5 20x20 = ultralytics anchor order;
//   VART returns the round-4 640 xmodel heads as 80, 20, 40). Ties of equal score keep anchor order (stable).
struct PostConfig {
  float conf = 0.25f;            // floor: max class score > conf (strict, = ultralytics conf_thres)
  float iou = 0.7f;              // IoU > iou suppressed
  int max_det = 0;               // 0 = no cap
  std::vector<float> cls_thr;    // empty = none; else size nc, kept iff score >= cls_thr[cls] (after NMS, max_det)
  bool sort_heads = false;
  bool legacy() const { return cls_thr.empty() && max_det == 0 && !sort_heads; }
};
std::vector<Det> postprocess_cfg(const std::vector<HeadView>& heads, int input_size, const PostConfig& pc,
                                 PostTiming* t = nullptr);

}  // namespace dpupipe
