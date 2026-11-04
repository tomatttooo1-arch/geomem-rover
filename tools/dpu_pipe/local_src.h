// NN-CAM2 (2026-11-03): dpu_rx's local frame source. Instead of pulling preprocessed frames from the Nano over UDP
// (DpuRx), read the newest YUYV frame from kv_cam's shared-memory ring (tools/kv_cam/cam_ring.h RingReader, the
// protocol's own reader; no second implementation), preprocess it on the KV260 (colour_prep.h: SoftPrep now, the PL
// core's driver when merged) straight into a FrameRing slot, publish. Same flow control as DpuRx (a slot only while
// READY + in the workers < --inflight, or READY < --ahead), so the DPU rate sets how many camera frames are used;
// the rest are skipped (counted, "only the newest frame" like every consumer of the product ring).
//
// Never refuses: no ring yet / writer gone / wrong geometry -> warning on stderr (rate-limited) and retry.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "colour_prep.h"
#include "dpu_ring.h"
#include "dpu_rx.h"

namespace dpupipe {

struct LocalConfig {
  std::string ring_name = "kv_cam_v1";  // /dev/shm/<name>
  std::string ring_path;                // or a file / udmabuf (kv_cam --path)
  int size = 640, fix_point = 6;
  int kset = 0, ups = 1;                // YUYV -> RGB: 0 JFIF full range (= MJPG decode) / 1 BT.601 limited; fancy chroma
  std::string prep = "soft";            // soft | pl (pl: the PREP1 driver; not merged -> warning, soft)
  size_t max_ahead = 1, max_inflight = 0;
  int stale_reopen_ms = 3000;           // writer heartbeat older than this: warn, reopen the ring
  bool perturb = false;                 // positive control only: SoftPrep coefficient +1
};

struct LocalCounts {
  std::atomic<uint64_t> frames{0}, skipped{0}, overwritten{0}, not_yet{0}, prep_fail{0}, opens{0}, open_fail{0},
      reopen_stale{0}, geometry_bad{0}, no_colour{0};
};

class LocalColourSource {
 public:
  LocalColourSource(const LocalConfig& cfg, FrameRing* ring, const Geometry& g);
  ~LocalColourSource();
  void start();
  void stop();
  const LocalCounts& counts() const { return counts_; }
  uint64_t cpu_ns() const { return cpu_ns_.load(); }
  std::string describe_json() const;

 private:
  void run();
  bool ensure_open();
  void warn(const std::string& msg);

  LocalConfig cfg_;
  FrameRing* ring_;
  Geometry geo_;
  std::unique_ptr<ColourPrep> prep_;
  std::string prep_note_;
  struct ReaderBox;
  std::unique_ptr<ReaderBox> rd_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> cpu_ns_{0};
  std::thread th_;
  LocalCounts counts_;
  uint64_t last_warn_ns_ = 0;
};

}  // namespace dpupipe
