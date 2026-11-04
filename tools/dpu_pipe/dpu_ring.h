// DPUPIPE0 (2026-11-03): page-aligned frame ring between the colour receiver (producer) and the DPU
// inference loop (consumer) on the KV260. Each slot holds one whole DPU input tensor
// (size x size x 3 int8, NHWC, letterbox pad included), so the inference loop can hand slot memory to
// VART directly (or memcpy it into a VART TensorBuffer, see dpu_rx.h InferBackend).
//
// Slot states: FREE -> FILLING (receiver) -> READY -> INFER (inference loop) -> FREE.
//   * acquire_free(): receiver takes a FREE slot to receive the next frame into.
//   * publish(): FILLING -> READY. Any older READY slot is dropped back to FREE ("stale", counted):
//     the inference loop always gets the newest complete frame.
//   * acquire_ready(): inference loop takes the newest READY slot (READY -> INFER).
//   * release(): INFER -> FREE.
//   * wait_ready_below(k): receiver back-pressure, "do not run more than k frames ahead of the DPU".
// With 3 slots one frame is being inferred, one is waiting and one is on the wire: transfer of frame
// N+1 overlaps inference of frame N.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace dpupipe {

struct FrameMeta {
  uint64_t source_key = 0;   // camera stamp (ns) from the Nano
  uint32_t seq = 0;          // sender's frame counter
  uint64_t t_want_ns = 0;    // receiver CLOCK_MONOTONIC: WANT sent
  uint64_t t_first_ns = 0;   //   first chunk of the answer received
  uint64_t t_done_ns = 0;    //   last chunk received
  uint32_t prep_ns = 0;      // Nano: preprocessing time
  uint32_t hold_ns = 0;      // Nano: WANT held waiting for a newer camera frame
  uint32_t crc32 = 0;        // of the content bytes (from the head)
  bool crc_checked = false, crc_ok = false;
  uint16_t size = 0, w = 0, h = 0, top = 0, left = 0;
  uint8_t fix_point = 0;
  int8_t pad_q = 0;
  uint64_t rx_cpu_ns = 0;    // receiver thread CPU spent on this frame
  // NN-CAM2 local frame source (dpu_rx --frame-source kvcam; all 0 on the UDP path). Camera stamps are kept in
  // their own columns and never mixed with CLOCK_MONOTONIC.
  uint64_t cam_index = 0;      // kv_cam slot colour_index (camera frame index; replay: pack frame index)
  uint64_t cam_dev_ts_us = 0;  // colour device timestamp (camera clock)
  uint64_t cam_sys_ts_us = 0;  // colour SDK host timestamp (SDK's system clock)
  uint64_t cam_arrive_ns = 0;  // CLOCK_MONOTONIC: kv_cam got the frameset
  uint64_t cam_publish_ns = 0; // CLOCK_MONOTONIC: kv_cam published it into the ring
  uint64_t ring_seq = 0;       // kv_cam ring sequence
  uint64_t t_read_ns = 0;      // CLOCK_MONOTONIC: dpu_rx saw the frame (after wait_newer)
  uint64_t t_prep_ns = 0;      // CLOCK_MONOTONIC: preprocessing done (tensor in the slot)
};

class FrameRing {
 public:
  enum State { FREE = 0, FILLING = 1, READY = 2, INFER = 3 };

  // Own page-aligned slots, every byte initialised to fill (the letterbox pad value).
  FrameRing(size_t nslots, size_t slot_bytes, int8_t fill, size_t align = 4096)
      : slot_bytes_(slot_bytes), owned_(true) {
    if (nslots < 2) throw std::invalid_argument("need at least 2 slots");
    size_t stride = (slot_bytes + align - 1) / align * align;
    for (size_t i = 0; i < nslots; ++i) {
      void* p = nullptr;
      if (posix_memalign(&p, align, stride) != 0) throw std::bad_alloc();
      std::memset(p, static_cast<uint8_t>(fill), stride);
      data_.push_back(static_cast<uint8_t*>(p));
    }
    init_state();
  }
  // External buffers (e.g. VART input TensorBuffers' data()); caller keeps them alive and pre-filled.
  FrameRing(const std::vector<uint8_t*>& external, size_t slot_bytes)
      : data_(external), slot_bytes_(slot_bytes), owned_(false) {
    if (external.size() < 2) throw std::invalid_argument("need at least 2 slots");
    init_state();
  }
  ~FrameRing() {
    if (owned_)
      for (auto* p : data_) free(p);
  }
  FrameRing(const FrameRing&) = delete;
  FrameRing& operator=(const FrameRing&) = delete;

  size_t slots() const { return data_.size(); }
  size_t slot_bytes() const { return slot_bytes_; }
  uint8_t* data(int slot) { return data_.at(slot); }
  const FrameMeta& meta(int slot) const { return meta_.at(slot); }

  // ---- producer
  int acquire_free(int timeout_ms) {
    std::unique_lock<std::mutex> lk(mu_);
    int got = -1;
    cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] {
      if (closed_) return true;
      got = find(FREE);
      return got >= 0;
    });
    if (got >= 0) state_[got] = FILLING;
    return closed_ ? -1 : got;
  }
  // DPUPIPE1: frames taken by the inference workers and not yet finish()ed (DPU queued/running, post-processing)
  // plus READY ones; the receiver can pull only while this is below k (--inflight), so with N workers it asks
  // for a frame exactly when a worker is about to need one instead of parking one more frame in READY.
  bool wait_inflight_below(size_t k, int timeout_ms) {
    std::unique_lock<std::mutex> lk(mu_);
    return cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                        [&] { return closed_ || count(READY) + busy_ < k; }) && !closed_;
  }
  void finish() {
    std::lock_guard<std::mutex> lk(mu_);
    if (busy_ > 0) --busy_;
    cv_.notify_all();
  }
  bool wait_ready_below(size_t k, int timeout_ms) {
    std::unique_lock<std::mutex> lk(mu_);
    return cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                        [&] { return closed_ || count(READY) < k; }) && !closed_;
  }
  void publish(int slot, const FrameMeta& m) {
    std::lock_guard<std::mutex> lk(mu_);
    expect(slot, FILLING);
    for (size_t i = 0; i < state_.size(); ++i)
      if (state_[i] == READY) {
        state_[i] = FREE;
        ++stale_dropped_;
      }
    meta_[slot] = m;
    state_[slot] = READY;
    ready_order_[slot] = ++publish_seq_;
    ++published_;
    cv_.notify_all();
  }
  void abort(int slot) {
    std::lock_guard<std::mutex> lk(mu_);
    expect(slot, FILLING);
    state_[slot] = FREE;
    cv_.notify_all();
  }

  // ---- consumer
  int acquire_ready(int timeout_ms, FrameMeta* m = nullptr) {
    std::unique_lock<std::mutex> lk(mu_);
    int got = -1;
    cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] {
      if (closed_) return true;
      got = newest_ready();
      return got >= 0;
    });
    if (closed_ || got < 0) return -1;
    state_[got] = INFER;
    if (m) *m = meta_[got];
    ++consumed_;
    ++busy_;
    cv_.notify_all();
    return got;
  }
  void release(int slot) {
    std::lock_guard<std::mutex> lk(mu_);
    expect(slot, INFER);
    state_[slot] = FREE;
    cv_.notify_all();
  }

  void close() {
    std::lock_guard<std::mutex> lk(mu_);
    closed_ = true;
    cv_.notify_all();
  }
  uint64_t published() { std::lock_guard<std::mutex> lk(mu_); return published_; }
  uint64_t consumed() { std::lock_guard<std::mutex> lk(mu_); return consumed_; }
  uint64_t stale_dropped() { std::lock_guard<std::mutex> lk(mu_); return stale_dropped_; }
  int state(int slot) { std::lock_guard<std::mutex> lk(mu_); return state_.at(slot); }

 private:
  void init_state() {
    state_.assign(data_.size(), FREE);
    meta_.assign(data_.size(), FrameMeta());
    ready_order_.assign(data_.size(), 0);
  }
  int find(State s) const {
    for (size_t i = 0; i < state_.size(); ++i)
      if (state_[i] == s) return static_cast<int>(i);
    return -1;
  }
  size_t count(State s) const {
    size_t n = 0;
    for (auto v : state_) n += (v == s);
    return n;
  }
  int newest_ready() const {
    int best = -1;
    for (size_t i = 0; i < state_.size(); ++i)
      if (state_[i] == READY && (best < 0 || ready_order_[i] > ready_order_[best])) best = static_cast<int>(i);
    return best;
  }
  void expect(int slot, State s) const {
    if (slot < 0 || static_cast<size_t>(slot) >= state_.size() || state_[slot] != s)
      throw std::logic_error("slot state");
  }

  std::vector<uint8_t*> data_;
  size_t slot_bytes_;
  bool owned_;
  std::vector<int> state_;
  std::vector<FrameMeta> meta_;
  std::vector<uint64_t> ready_order_;
  uint64_t publish_seq_ = 0, published_ = 0, consumed_ = 0, stale_dropped_ = 0;
  size_t busy_ = 0;  // acquire_ready()d and not yet finish()ed (only meaningful when the consumer calls finish())
  bool closed_ = false;
  std::mutex mu_;
  std::condition_variable cv_;
};

}  // namespace dpupipe
