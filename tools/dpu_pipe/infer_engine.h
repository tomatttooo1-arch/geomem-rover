// DPUPIPE1 (2026-11-03): inference engines for the dpu_rx worker threads.
//
// A worker takes the newest READY ring slot, stage()s it (copies the input tensor into memory the engine
// owns, here the VART runner's own input TensorBuffer), RELEASES the slot at once (so the receiver can pull
// the next frame while the DPU works), then infer()s (DPU) and post()s (CPU output processing, det_post.cpp).
// Several workers, each with its own engine (own VART runner), keep the single DPU core busy: while worker A
// waits for its job, worker B's job is queued in the driver, which hides VART's per-job fixed cost
// (DPUB1: ~7 ms of cache sync / ioctl per job) and the A53 copy + post-processing behind DPU time.
//
//   StubEngine: no DPU. infer() holds a process-wide mutex for infer_ms (one simulated DPU core, so N
//               workers do not run N fake DPUs in parallel); post() returns no boxes.
//   VartEngine: VART 3.0 (/opt/vitis_ai/3.0) runner on the xmodel's DPU subgraph; only built with HAVE_VART.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "det_post.h"
#include "pl_post.h"

namespace dpupipe {

class Engine {
 public:
  virtual ~Engine() = default;
  virtual bool stage(const uint8_t* input, size_t bytes) = 0;  // copy the input tensor in (slot reusable after)
  virtual bool infer() = 0;                                     // DPU on the staged input (blocking)
  virtual std::vector<Det> post(PostTiming* t) = 0;             // CPU output processing of the last infer()
  virtual uint32_t staged_crc() const { return 0; }             // crc32 of the staged tensor (when asked for)
  virtual std::string describe() const = 0;                     // JSON object
  // RECOG640: output processing parameters (default = DPUPIPE1 behaviour: conf 0.25, IoU 0.7, no max_det)
  void set_post(const PostConfig& p) { post_cfg_ = p; }
  const PostConfig& post_config() const { return post_cfg_; }
  // RECOG641: output processing on the PL core (pl_post.h). false (with why) = this engine stays on the CPU path.
  virtual bool attach_pl_post(std::shared_ptr<PlPost> p, std::string* why) {
    (void)p;
    *why = "engine has no DPU output buffers (stub)";
    return false;
  }
  uint64_t pl_frames() const { return pl_frames_; }
  uint64_t cpu_frames() const { return cpu_frames_; }

 protected:
  PostConfig post_cfg_;
  uint64_t pl_frames_ = 0, cpu_frames_ = 0;
};

class StubEngine : public Engine {
 public:
  StubEngine(double infer_ms, bool tensor_crc) : infer_ms_(infer_ms), tensor_crc_(tensor_crc) {}
  bool stage(const uint8_t* input, size_t bytes) override;
  bool infer() override;
  std::vector<Det> post(PostTiming* t) override;
  uint32_t staged_crc() const override { return crc_; }
  std::string describe() const override;

 private:
  static std::mutex& dpu();
  double infer_ms_;
  bool tensor_crc_;
  uint32_t crc_ = 0;
};

#ifdef HAVE_VART
struct VartShared;  // the deserialised xir graph, shared by all workers' runners

class VartEngine : public Engine {
 public:
  // model: loaded once with load_model(); every engine creates its own runner on the DPU subgraph.
  static std::shared_ptr<VartShared> load_model(const std::string& xmodel);
  explicit VartEngine(std::shared_ptr<VartShared> m, bool tensor_crc = false);
  ~VartEngine() override;
  bool stage(const uint8_t* input, size_t bytes) override;
  bool infer() override;
  std::vector<Det> post(PostTiming* t) override;
  uint32_t staged_crc() const override { return crc_; }
  std::string describe() const override;
  bool attach_pl_post(std::shared_ptr<PlPost> p, std::string* why) override;

  int input_size() const { return in_h_; }
  int input_w() const { return in_w_; }
  int input_h() const { return in_h_; }
  int input_fix_point() const { return in_fix_; }
  size_t input_bytes() const { return in_bytes_; }
  // raw output heads of the last infer() (int8 NHWC, VART output TensorBuffer memory)
  const std::vector<HeadView>& heads() const { return heads_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::shared_ptr<VartShared> model_;
  int in_h_ = 0, in_w_ = 0, in_fix_ = 0;
  size_t in_bytes_ = 0;
  std::vector<HeadView> heads_;
  std::shared_ptr<PlPost> pl_;     // RECOG641: PL output processing (shared core), nullptr = CPU
  detpost_drv::Layout pl_lay_{};   // this runner's output buffers on the core
  bool outputs_synced_ = false;    // CPU cache synced for the last infer()'s heads
  bool tensor_crc_;
  uint32_t crc_ = 0;
};
#endif

}  // namespace dpupipe
