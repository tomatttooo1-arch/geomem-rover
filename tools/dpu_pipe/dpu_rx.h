// DPUPIPE0 (2026-11-03): KV260-side receiver of the DPU input frames that the Nano's colour sender
// (host/kv_colour.py ColourSource, WANT_DPU / FRAME_DPU, types 44/45) preprocesses and quantises.
// Its own process (not kv_server: no Python GIL shared with the geometry service).
//
// Pull model, one frame ahead: the receiver asks (WANT_DPU, after_key = last frame's camera stamp) only
// when it has a FREE ring slot and fewer than `max_ahead` frames are waiting for the DPU, so the DPU
// rate sets the colour traffic and frame N+1 is on the wire while frame N is in the DPU.
//
// Zero-copy placement: the WANT tells the Nano the chunk size; every FRAME_DPU chunk i is received
// (MSG_PEEK of the 32-byte kv_link chunk header, then recvmsg with an iovec) straight into
//   slot + content_offset - 64 + i * chunk_bytes
// i.e. the 64-byte head lands in the last 64 bytes of the top letterbox pad and the content rows land in
// place. After the frame is complete the head is copied out and those 64 pad bytes are rewritten with
// pad_q. The rest of the pad is written once when the ring is created. Frames whose content does not
// start at least 64 bytes into the tensor, or is not whole tensor rows (left/right pad), go through a
// staging buffer and a row copy instead (not the car camera's case: 640x480 at 640/512/416 is direct).
#pragma once

#include <netinet/in.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "dpu_ring.h"

namespace dpupipe {

constexpr uint8_t T_HELLO = 40, T_NONE = 43, T_WANT_DPU = 44, T_FRAME_DPU = 45;
constexpr size_t CHUNK_HEAD = 32, DPU_HEAD = 64, DPU_WANT = 32;

#pragma pack(push, 1)
struct ChunkHead {      // host/kv_link.py CHUNK "<4sBBHQQIHH"
  char magic[4];
  uint8_t type, flags;
  uint16_t rsv;
  uint64_t session, frame_id;
  uint32_t total;
  uint16_t idx, n;
};
struct DpuWant {        // host/kv_colour.py DPU_WANT "<BBHBB2sQIIQ"
  uint8_t ver, fmt;
  uint16_t size;
  uint8_t fix_point, flags;
  uint8_t rsv[2];
  uint64_t after_key;
  uint32_t wait_ms, chunk_bytes;
  uint64_t kv_t_ns;
};
struct DpuHead {        // host/kv_colour.py DPU_HEAD "<BBHBbBBQQQIIIIHHHHHHI"
  uint8_t ver, fmt;
  uint16_t size;
  uint8_t fix_point;
  int8_t pad_q;
  uint8_t backend, rsv;
  uint64_t source_key, after_key, kv_t_ns;
  uint32_t prep_ns, hold_ns, crc32, content_bytes;
  uint16_t src_w, src_h, w, h, top, left;
  uint32_t seq;
};
#pragma pack(pop)
static_assert(sizeof(ChunkHead) == CHUNK_HEAD, "chunk head");
static_assert(sizeof(DpuWant) == DPU_WANT, "want");
static_assert(sizeof(DpuHead) == DPU_HEAD, "dpu head");

// Ultralytics validator letterbox geometry (host/dpu_prep.py LetterboxGeometry), for the receiver's
// expectations and placement.
struct Geometry {
  int src_w = 0, src_h = 0, size = 0, w = 0, h = 0, top = 0, left = 0;
  size_t content_offset() const { return (static_cast<size_t>(top) * size + left) * 3; }
  size_t content_bytes() const { return static_cast<size_t>(w) * h * 3; }
  size_t tensor_bytes() const { return static_cast<size_t>(size) * size * 3; }
  bool direct() const { return left == 0 && w == size && content_offset() >= DPU_HEAD; }
};
Geometry letterbox(int src_w, int src_h, int size);
int8_t quant_pad(int fix_point);  // data-fix of 114/255
uint32_t crc32_bytes(const uint8_t* p, size_t n);

struct RxConfig {
  uint16_t bind_port = 47900;
  std::string source;            // "host:port" of the Nano sender; empty = learn from its HELLO
  int size = 640, fix_point = 6, src_w = 640, src_h = 480;
  uint32_t chunk_bytes = 60000;  // 60000: IP-fragmented datagrams; 1400: one Ethernet frame each
  uint32_t wait_ms = 200;        // long-poll hold on the Nano when no newer frame exists
  int frame_timeout_ms = 100;    // first chunk -> complete
  int rcvbuf = 8 << 20;
  size_t max_ahead = 1;          // READY frames allowed to wait for the DPU
  size_t max_inflight = 0;       // DPUPIPE1: >0 = pull only while READY + frames in the workers < this (FrameRing::finish)
  bool verify_crc = false;       // crc32 of the content per frame: ~2.5 ms on the host at 640, off in the product (UDP checksums cover each datagram)
  std::string log_path;          // per-frame JSONL (empty: none)
};

struct RxCounts {
  std::atomic<uint64_t> wants{0}, frames{0}, none_no_new{0}, none_other{0}, incomplete{0}, want_timeout{0},
      bad_head{0}, crc_bad{0}, stale_chunks{0}, malformed{0}, hellos{0}, chunks{0};
};

class DpuRx {
 public:
  DpuRx(const RxConfig& cfg, FrameRing* ring);
  ~DpuRx();
  void start();
  void stop();
  const RxCounts& counts() const { return counts_; }
  int effective_rcvbuf() const { return rcvbuf_eff_; }
  uint64_t rx_cpu_ns() const { return cpu_ns_.load(); }
  std::string describe_json() const;

 private:
  void run();
  bool send_want(uint64_t want_id, uint64_t after_key);
  // receives one FRAME_DPU for want_id into slot; 1 complete, 0 NONE(no_new), -1 failure/timeout
  int receive_frame(uint64_t want_id, int slot, uint64_t t_want, FrameMeta* m);
  void drain_other(const ChunkHead& h, ssize_t len, const sockaddr_in& from);

  RxConfig cfg_;
  FrameRing* ring_;
  Geometry geo_;
  int fd_ = -1, rcvbuf_eff_ = 0;
  sockaddr_in src_{};
  bool have_src_ = false;
  uint64_t session_ = 0;
  std::vector<uint8_t> scratch_, staging_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> cpu_ns_{0};
  std::thread th_;
  RxCounts counts_;
  FILE* log_ = nullptr;
};

// ---- inference side ---------------------------------------------------------------------------
// What the inference loop needs from a DPU runtime. The real one (VART 3.0, DPURT0 route) is not on
// the board yet; StubBackend stands in so the pipeline can be measured end to end.
class InferBackend {
 public:
  virtual ~InferBackend() = default;
  // Run the DPU on one input tensor (size x size x 3 int8 NHWC, pad included) and its CPU
  // post-processing. Returns false on a runtime error.
  //
  // VART plan (to fill in once VART is installed, see DPURT0/prep/dpu_probe.py for the Python form):
  //   runner = vart::Runner::create_runner(subgraph, "run");  inputs/outputs = runner->get_inputs()/...
  //   copy (or, zero-copy: build the ring over the TensorBuffers' data() via the external-buffer
  //   FrameRing constructor and pick the matching TensorBuffer here) -> sync_for_write ->
  //   runner->execute_async(in, out) -> runner->wait(job) -> sync_for_read -> post-process
  //   (DETPOST0 C reference / DETPOST1 PL core).
  virtual bool run(int slot, const uint8_t* input, size_t bytes, const FrameMeta& meta) = 0;
};

class StubBackend : public InferBackend {
 public:
  explicit StubBackend(double infer_ms, bool tensor_crc) : infer_ms_(infer_ms), tensor_crc_(tensor_crc) {}
  bool run(int slot, const uint8_t* input, size_t bytes, const FrameMeta& meta) override;
  uint32_t last_crc = 0;  // crc32 of the whole tensor (pad included) when tensor_crc

 private:
  double infer_ms_;
  bool tensor_crc_;
};

uint64_t mono_ns();
uint64_t thread_cpu_ns();

}  // namespace dpupipe
