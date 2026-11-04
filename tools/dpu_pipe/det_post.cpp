// DPUPIPE1 (2026-11-03): see det_post.h. Mirrors DPUB1 full1.py dfl_decode_f + nms operation by operation in
// float32 so the boxes can be compared value by value with the Python reference.
#include "det_post.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace dpupipe {

namespace {

// numpy float32 add-reduce over a contiguous run of 16: pairwise_sum with 8 accumulators
// r[j] = a[j] + a[j + 8]; ((r0 + r1) + (r2 + r3)) + ((r4 + r5) + (r6 + r7)); added to the identity 0.
inline float np_sum16(const float* a) {
  float r[8];
  for (int j = 0; j < 8; ++j) r[j] = a[j] + a[j + 8];
  float s = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
  return 0.0f + s;
}

// one side: softmax over 16 int8 logits (already scaled), expectation over bin index
inline float dfl_side(const float* v) {
  float m = v[0];
  for (int k = 1; k < 16; ++k) m = v[k] > m ? v[k] : m;
  float e[16];
  for (int k = 0; k < 16; ++k) e[k] = std::exp(v[k] - m);
  float s = np_sum16(e);
  float p[16];
  for (int k = 0; k < 16; ++k) p[k] = (e[k] / s) * static_cast<float>(k);
  return np_sum16(p);
}

}  // namespace

std::vector<Det> decode_fast(const std::vector<HeadView>& heads, int input_size, float conf) {
  std::vector<Det> out;
  const double th = std::log(static_cast<double>(conf) / (1.0 - static_cast<double>(conf)));
  for (const auto& hd : heads) {
    const int nc = hd.c - 64;
    if (nc <= 0 || hd.h <= 0 || hd.w <= 0) continue;
    const float scale = std::ldexp(1.0f, -hd.fix_point);  // 2^-fp, exact
    // RECOG640: stride from the width (input_size = tensor width); identical for square inputs, and right for the
    // 640x480 rectangular model (80x60 / 40x30 / 20x15 heads, stride 8/16/32)
    const float stride = static_cast<float>(input_size / hd.w);
    const int cells = hd.h * hd.w;
    for (int idx = 0; idx < cells; ++idx) {
      const int8_t* cell = hd.data + static_cast<size_t>(idx) * hd.c;
      // max class logit and first argmax, on the scaled value (scaling by 2^-fp keeps the order)
      int8_t qmax = cell[64];
      int ci = 0;
      for (int k = 1; k < nc; ++k)
        if (cell[64 + k] > qmax) {
          qmax = cell[64 + k];
          ci = k;
        }
      const float mx = static_cast<float>(qmax) * scale;
      if (!(static_cast<double>(mx) > th)) continue;
      float v[64];
      for (int k = 0; k < 64; ++k) v[k] = static_cast<float>(cell[k]) * scale;
      const float l = dfl_side(v), t = dfl_side(v + 16), r = dfl_side(v + 32), b = dfl_side(v + 48);
      const float gy = static_cast<float>(idx / hd.w) + 0.5f, gx = static_cast<float>(idx % hd.w) + 0.5f;
      const float sc = 1.0f / (1.0f + std::exp(-mx));
      out.push_back({(gx - l) * stride, (gy - t) * stride, (gx + r) * stride, (gy + b) * stride, sc,
                     static_cast<float>(ci)});
    }
  }
  return out;
}

std::vector<Det> nms_greedy(const std::vector<Det>& d, float iou_thr) {
  const size_t n = d.size();
  if (n == 0) return {};
  std::vector<float> bx(n * 4), area(n);
  for (size_t i = 0; i < n; ++i) {
    const float off = d[i].cls * 4096.0f;
    bx[4 * i + 0] = d[i].x1 + off;
    bx[4 * i + 1] = d[i].y1 + off;
    bx[4 * i + 2] = d[i].x2 + off;
    bx[4 * i + 3] = d[i].y2 + off;
    area[i] = (bx[4 * i + 2] - bx[4 * i + 0]) * (bx[4 * i + 3] - bx[4 * i + 1]);
  }
  std::vector<size_t> o(n);
  for (size_t i = 0; i < n; ++i) o[i] = i;
  std::stable_sort(o.begin(), o.end(), [&](size_t a, size_t b) { return d[a].score > d[b].score; });
  std::vector<Det> keep;
  std::vector<size_t> rest;
  while (!o.empty()) {
    const size_t i = o[0];
    keep.push_back(d[i]);
    rest.clear();
    for (size_t k = 1; k < o.size(); ++k) {
      const size_t j = o[k];
      const float xx1 = std::max(bx[4 * i + 0], bx[4 * j + 0]), yy1 = std::max(bx[4 * i + 1], bx[4 * j + 1]);
      const float xx2 = std::min(bx[4 * i + 2], bx[4 * j + 2]), yy2 = std::min(bx[4 * i + 3], bx[4 * j + 3]);
      const float w = std::max(xx2 - xx1, 0.0f), h = std::max(yy2 - yy1, 0.0f);
      const float inter = w * h;
      const float iou = inter / (((area[i] + area[j]) - inter) + 1e-9f);
      if (iou <= iou_thr) rest.push_back(j);
    }
    o.swap(rest);
  }
  return keep;
}

std::vector<Det> postprocess(const std::vector<HeadView>& heads, int input_size, PostTiming* t, float conf,
                             float iou_thr) {
  using clk = std::chrono::steady_clock;
  auto t0 = clk::now();
  std::vector<Det> c = decode_fast(heads, input_size, conf);
  auto t1 = clk::now();
  std::vector<Det> k = nms_greedy(c, iou_thr);
  auto t2 = clk::now();
  if (t) {
    t->fix_decode_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    t->nms_us = std::chrono::duration<double, std::micro>(t2 - t1).count();
    t->candidates = c.size();
  }
  return k;
}

std::vector<Det> postprocess_cfg(const std::vector<HeadView>& heads_in, int input_size, const PostConfig& pc,
                                 PostTiming* t) {
  if (pc.legacy()) return postprocess(heads_in, input_size, t, pc.conf, pc.iou);
  using clk = std::chrono::steady_clock;
  auto t0 = clk::now();
  std::vector<HeadView> heads = heads_in;
  if (pc.sort_heads)
    std::stable_sort(heads.begin(), heads.end(), [](const HeadView& a, const HeadView& b) { return a.h > b.h; });
  // gate: the decode keeps logit > ln(g/(1-g)); take g just below min(cls_thr) so that "score >= min thr" survives,
  // then re-check both conditions on the float32 score (the same value the final per-class test uses)
  float gate = pc.conf;
  if (!pc.cls_thr.empty()) {
    float mn = pc.cls_thr[0];
    for (float v : pc.cls_thr) mn = std::min(mn, v);
    const float below = std::nextafter(mn, 0.0f) * 0.999f;
    gate = std::max(pc.conf, below);
  }
  std::vector<Det> c0 = decode_fast(heads, input_size, gate), c;
  c.reserve(c0.size());
  float mn = 2.0f;
  for (float v : pc.cls_thr) mn = std::min(mn, v);
  for (const Det& d : c0)
    if (d.score > pc.conf && (pc.cls_thr.empty() || d.score >= mn)) c.push_back(d);
  auto t1 = clk::now();
  std::vector<Det> k = nms_greedy(c, pc.iou);  // output in score order (stable)
  if (pc.max_det > 0 && k.size() > static_cast<size_t>(pc.max_det)) k.resize(pc.max_det);
  if (!pc.cls_thr.empty()) {
    std::vector<Det> f;
    for (const Det& d : k) {
      const size_t ci = static_cast<size_t>(d.cls);
      if (ci < pc.cls_thr.size() && d.score >= pc.cls_thr[ci]) f.push_back(d);
    }
    k.swap(f);
  }
  auto t2 = clk::now();
  if (t) {
    t->fix_decode_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    t->nms_us = std::chrono::duration<double, std::micro>(t2 - t1).count();
    t->candidates = c.size();
  }
  return k;
}

}  // namespace dpupipe
