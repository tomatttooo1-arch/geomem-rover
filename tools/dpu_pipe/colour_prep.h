// NN-CAM2 (2026-11-03): colour preprocessing on the KV260 for dpu_rx's local frame source (kv_cam ring).
//
// With the Nano gone the camera's YUYV frame has to become the DPU input tensor on the KV260 itself.
// The product path is the PL preprocessing core (PREP1: YUYV -> RGB -> cv2 INTER_LINEAR letterbox -> data-fix int8,
// hw/prep/hls1, not merged yet). Until its driver is merged, dpu_rx uses SoftPrep below: the same arithmetic in C++
// on the A53, written from the two reference models the PL core is verified against:
//   * YUYV -> RGB888: PREP1 hw/prep/ref/yuyv_ref.c (general form clamp((KY(Y-YOFF) + KX(C-128) + 2^15) >> 16);
//     kset 0 = JFIF full range (libjpeg jdcolor.c, == what cv2.imdecode gave the Nano from MJPG), kset 1 = BT.601
//     limited range; ups 1 = libjpeg h2v1_fancy_upsample, ups 0 = chroma replicated).
//   * resize + letterbox + quantise: PREP0 runs/.../PREP0/model/prep_model.c (prep_tables / prep_stream, proven
//     byte-equal to host/dpu_prep.py cv2 / numpy and to ultralytics LetterBox(114) + xmodel data-fix).
// Build with -ffp-contract=off (the coefficient table's float32 multiply-add must not be fused; PREP1 prep_drv.h).
//
// Interface for the PL driver (DRV5 2026-11-04, implemented in pl_prep.cpp, declared at the end of this file):
// make_pl_prep() returns a ColourPrep on the m5 bitstream's PL pre-processing core. One call = one frame: the YUYV
// bytes in, the whole tensor (pad included) out. dpu_rx --prep pl asks for it; when make_pl_prep returns nullptr
// dpu_rx warns once and runs SoftPrep (never refuses to start). PrepInput.phys is not used yet (the frame is copied).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace dpupipe {

struct PrepInput {
  const uint8_t* data = nullptr;  // YUYV (Y0 U Y1 V), w*h*2 bytes, rows `stride` bytes apart
  uint64_t phys = 0;              // physical address of data[0] when known (contiguous ring), else 0
  int w = 0, h = 0, stride = 0;
  uint32_t fourcc = 0;            // 'YUYV'
};

class ColourPrep {
 public:
  virtual ~ColourPrep() = default;
  virtual const char* name() const = 0;
  // Writes the whole size*size*3 tensor (letterbox pad included). false = this frame could not be prepared.
  virtual bool run(const PrepInput& in, uint8_t* tensor, size_t tensor_bytes) = 0;
  virtual std::string describe_json() const = 0;
};

constexpr uint32_t kFourccYUYV = uint32_t('Y') | uint32_t('U') << 8 | uint32_t('Y') << 16 | uint32_t('V') << 24;

class SoftPrep : public ColourPrep {
 public:
  static constexpr int kCoefScale = 2048;  // Q11 (PREP0 COEF_BITS 11)
  // src_w x src_h YUYV -> size x size letterbox; content out_w x out_h at (top, left) comes from dpu_rx.cpp letterbox()
  // (ultralytics geometry, one resize for the car's 640x480 at 640/512/416), data-fix fp.
  SoftPrep(int src_w, int src_h, int size, int out_w, int out_h, int top, int left, int fix_point, int kset, int ups)
      : src_w_(src_w), src_h_(src_h), size_(size), fp_(fix_point), kset_(kset), ups_(ups), out_w_(out_w),
        out_h_(out_h), top_(top), left_(left) {
    static const int kJfif[6] = {65536, 0, 91881, -22554, -46802, 116130};      // libjpeg jdcolor.c
    static const int kBt601l[6] = {76309, 16, 104597, -25675, -53279, 132201};  // BT.601 limited range
    std::memcpy(k_, kset == 1 ? kBt601l : kJfif, sizeof k_);
    for (int x = 0; x < 256; ++x) {  // round(x * 2^fp / 255), clipped at 127
      long q = (2L * x * (1L << fp_) + 255) / 510;
      lut_[x] = int8_t(q > 127 ? 127 : q);
    }
    pad_q_ = uint8_t(lut_[114]);
    xt_.resize(out_w_);
    yt_.resize(out_h_);
    ok_ = tables() == 0;
    rgb_.resize(size_t(src_w) * src_h * 3);
    line_.resize(size_t(out_w_) * 3);
    u_.resize(src_w);
    v_.resize(src_w);
  }
  const char* name() const override { return "soft"; }
  bool ok() const { return ok_; }
  int top() const { return top_; }
  int left() const { return left_; }
  int out_w() const { return out_w_; }
  int out_h() const { return out_h_; }
  uint8_t pad_q() const { return pad_q_; }
  // perturb (positive control only): first non-zero horizontal a1 += 1, as PREP0/PREP1's PERTURB
  void perturb() {
    for (auto& e : xt_)
      if (e.a1) {
        e.a1 += 1;
        return;
      }
  }

  bool run(const PrepInput& in, uint8_t* tensor, size_t tensor_bytes) override {
    if (!ok_ || in.fourcc != kFourccYUYV || in.w != src_w_ || in.h != src_h_ ||
        tensor_bytes != size_t(size_) * size_ * 3)
      return false;
    yuyv_to_rgb(in);
    stream(tensor);
    return true;
  }
  std::string describe_json() const override {
    char b[256];
    std::snprintf(b, sizeof b,
                  "{\"impl\":\"soft\",\"src\":\"%dx%d\",\"size\":%d,\"fix_point\":%d,\"kset\":%d,\"ups\":%d,"
                  "\"content\":\"%dx%d\",\"top\":%d,\"left\":%d,\"bypass\":%d}",
                  src_w_, src_h_, size_, fp_, kset_, ups_, out_w_, out_h_, top_, left_, int(bypass_));
    return b;
  }
  const std::vector<uint8_t>& last_rgb() const { return rgb_; }

 private:
  struct Ent {
    int x1, a0, a1;
  };
  static uint8_t lim(long v) { return v < 0 ? 0 : v > 255 ? 255 : uint8_t(v); }

  // libjpeg h2v1_fancy_upsample: n chroma samples -> 2n
  static void fancy(const uint8_t* in, int n, uint8_t* out) {
    const uint8_t* ip = in;
    uint8_t* op = out;
    int v = *ip++;
    *op++ = uint8_t(v);
    *op++ = uint8_t((v * 3 + ip[0] + 2) >> 2);
    for (int c = n - 2; c > 0; --c) {
      v = (*ip++) * 3;
      *op++ = uint8_t((v + ip[-2] + 1) >> 2);
      *op++ = uint8_t((v + ip[0] + 2) >> 2);
    }
    v = *ip;
    *op++ = uint8_t((v * 3 + ip[-1] + 1) >> 2);
    *op++ = uint8_t(v);
  }

  void yuyv_to_rgb(const PrepInput& in) {
    const int w = src_w_, n = w / 2;
    std::vector<uint8_t> cu(n), cv(n);
    for (int y = 0; y < src_h_; ++y) {
      const uint8_t* row = in.data + size_t(y) * in.stride;
      for (int i = 0; i < n; ++i) {
        cu[i] = row[4 * i + 1];
        cv[i] = row[4 * i + 3];
      }
      if (ups_) {
        fancy(cu.data(), n, u_.data());
        fancy(cv.data(), n, v_.data());
      } else {
        for (int i = 0; i < n; ++i) {
          u_[2 * i] = u_[2 * i + 1] = cu[i];
          v_[2 * i] = v_[2 * i + 1] = cv[i];
        }
      }
      uint8_t* o = rgb_.data() + size_t(y) * w * 3;
      for (int x = 0; x < w; ++x) {
        int yy = row[2 * x], u = u_[x], v = v_[x];
        long yk = long(k_[0]) * (yy - k_[1]);
        o[3 * x + 0] = lim((yk + long(k_[2]) * (v - 128) + 32768) >> 16);
        o[3 * x + 1] = lim((yk + long(k_[3]) * (u - 128) + long(k_[4]) * (v - 128) + 32768) >> 16);
        o[3 * x + 2] = lim((yk + long(k_[5]) * (u - 128) + 32768) >> 16);
      }
    }
  }

  // cv2 resize.cpp 8U INTER_LINEAR coefficients (PREP0 coefs())
  static void coefs(int dst_n, int src_n, std::vector<int>& ofs, std::vector<int>& c0, std::vector<int>& c1) {
    double scale = 1.0 / (double(dst_n) / double(src_n));
    for (int d = 0; d < dst_n; ++d) {
      float f = float((d + 0.5) * scale - 0.5);
      int s = int(std::floor(f));
      f = f - float(s);
      if (s < 0) {
        f = 0.f;
        s = 0;
      }
      if (s >= src_n - 1) {
        f = 0.f;
        s = src_n - 1;
      }
      float p0 = (1.f - f) * float(kCoefScale);
      float p1 = f * float(kCoefScale);
      c0[d] = int(std::lrint(p0));
      c1[d] = int(std::lrint(p1));
      ofs[d] = s;
    }
  }
  int axis(int src_n, int dst_n, std::vector<Ent>& t) {
    std::vector<int> ofs(dst_n), c0(dst_n), c1(dst_n);
    coefs(dst_n, src_n, ofs, c0, c1);
    for (int d = 0; d < dst_n; ++d) {
      int x0 = ofs[d], x1 = x0 + 1 < src_n ? x0 + 1 : src_n - 1;
      if (x1 != x0 + 1) return -1;
      t[d] = {x1, c0[d], c1[d]};
      if (d && t[d].x1 <= t[d - 1].x1) return -1;
    }
    return 0;
  }
  int tables() {  // PREP0 prep_tables()
    bypass_ = src_w_ == out_w_ && src_h_ == out_h_;
    if (bypass_) return 0;
    if (out_w_ > src_w_ || out_h_ > src_h_) return -1;
    return axis(src_w_, out_w_, xt_) || axis(src_h_, out_h_, yt_) ? -1 : 0;
  }
  // PREP0 prep_stream(): integer data path, source rows in order, tensor bytes in order
  void stream(uint8_t* out) {
    long pos = 0;
    auto wb = [&](uint8_t b) { out[pos++] = b; };
    const int t_w = size_, right = size_ - left_ - out_w_, bottom = size_ - top_ - out_h_;
    const long pad_row = long(t_w) * 3;
    std::memset(out, pad_q_, size_t(top_) * pad_row);
    pos = long(top_) * pad_row;
    int dy = 0;
    for (int r = 0; r < src_h_ && dy < out_h_; ++r) {
      const uint8_t* row = rgb_.data() + size_t(r) * src_w_ * 3;
      int emit = bypass_ ? 1 : (yt_[dy].x1 == r);
      if (emit)
        for (int i = 0; i < left_ * 3; ++i) wb(pad_q_);
      int dx = 0;
      int32_t prev[3] = {0, 0, 0};
      for (int x = 0; x < src_w_; ++x) {
        int32_t px[3] = {row[3 * x], row[3 * x + 1], row[3 * x + 2]};
        if (bypass_) {
          for (int c = 0; c < 3; ++c) wb(uint8_t(lut_[px[c]]));
          continue;
        }
        if (dx < out_w_ && xt_[dx].x1 == x) {
          for (int c = 0; c < 3; ++c) {
            int32_t h = (prev[c] * xt_[dx].a0 + px[c] * xt_[dx].a1) >> 4;
            if (emit) {
              int32_t hp = line_[size_t(dx) * 3 + c];
              int32_t v = ((yt_[dy].a0 * hp) >> 16) + ((yt_[dy].a1 * h) >> 16);
              v = (v + 2) >> 2;
              if (v < 0) v = 0;
              if (v > 255) v = 255;
              wb(uint8_t(lut_[v]));
            }
            line_[size_t(dx) * 3 + c] = h;
          }
          ++dx;
        }
        for (int c = 0; c < 3; ++c) prev[c] = px[c];
      }
      if (emit) {
        for (int i = 0; i < right * 3; ++i) wb(pad_q_);
        ++dy;
      }
    }
    std::memset(out + pos, pad_q_, size_t(bottom) * pad_row);
  }

  int src_w_, src_h_, size_, fp_, kset_, ups_;
  int out_w_, out_h_, top_, left_;
  int k_[6];
  bool bypass_ = false, ok_ = false;
  int8_t lut_[256];
  uint8_t pad_q_ = 0;
  std::vector<Ent> xt_, yt_;
  std::vector<uint8_t> rgb_, u_, v_;
  std::vector<int32_t> line_;
};

// The PL preprocessing core's driver (DRV5 2026-11-04): tools/dpu_pipe/pl_prep.cpp, on hw/prep/drv/prep_core.{hpp,cpp}
// (parameter block from PREP1 hw/prep/hls1/prep_drv.h prep_drv_params(), the 14 driver checks of PREP1 README 6.4).
// One call = one frame: the YUYV rows are copied into the core's contiguous (LPD_DDR_LOW) source buffer, the core is
// started, and the whole tensor (pad included) is copied out of its DMA buffer. letterbox() gives the content box
// (same geometry as SoftPrep). Returns nullptr (with why) when the core cannot be opened or the geometry cannot
// stream: dpu_rx then warns once and runs SoftPrep (never refuses to start). If the core later times out it is never
// started again and the returned object runs SoftPrep itself for every following frame (one warning).
// Environment (tests / bench): DPU_RX_PL_PREP_WAIT=auto|irq|poll, DPU_RX_PL_PREP_DMA=auto|dmaheap:<heap>|dpu.
std::unique_ptr<ColourPrep> make_pl_prep(int src_w, int src_h, int size, int fix_point, int kset, int ups,
                                         std::string* why);

}  // namespace dpupipe
