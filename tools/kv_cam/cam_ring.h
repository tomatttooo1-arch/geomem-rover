// NN-CAM1 (2026-11-04): kv_cam shared-memory frame ring, protocol version 1.
// One writer (kv_cam), any number of readers (kv_server, dpu_rx / PREP driver, front end, car screen, preview,
// ground calibration). Readers never block the writer and never write to the segment.
// The byte layout is fixed and documented in protocol.md; the static_asserts below pin every offset.
//
// Per-slot seqlock (two counters):
//   writer: seq_begin = s  -> release fence -> payload + meta -> seq_end = s (release) -> latest_seq = s (release)
//           -> futex_word = (uint32)s -> FUTEX_WAKE
//   reader: e = seq_end (acquire) == s ? -> copy payload/meta -> acquire fence -> seq_begin == s ? -> meta.seq == s ?
//   Any mismatch = the slot was overwritten while it was being read: the copy is discarded (OVERWRITTEN), never
//   returned. Slot of sequence s (s >= 1) = (s - 1) % nslots.
#pragma once

#include <atomic>
#include <cerrno>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <linux/futex.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace kvcam {

constexpr uint64_t kMagic = 0x3147524d4143564bull;  // "KVCAMRG1" little-endian
constexpr uint32_t kVersion = 1;
constexpr uint32_t kPage = 4096;
constexpr const char* kDefaultName = "kv_cam_v1";  // /dev/shm/kv_cam_v1

constexpr uint32_t fourcc(char a, char b, char c, char d) {
  return uint32_t(uint8_t(a)) | uint32_t(uint8_t(b)) << 8 | uint32_t(uint8_t(c)) << 16 | uint32_t(uint8_t(d)) << 24;
}
constexpr uint32_t kFmtYUYV = fourcc('Y', 'U', 'Y', 'V');  // 2 B/px, Y0 U Y1 V
constexpr uint32_t kFmtY16 = fourcc('Y', '1', '6', ' ');   // u16 little-endian, unit = depth_scale_mm
constexpr uint32_t kFmtRGB = fourcc('R', 'G', 'B', '3');   // 3 B/px (only for a non-default colour mode)

enum SlotFlags : uint32_t {
  kHasColour = 1u << 0,
  kHasDepth = 1u << 1,
  kDepthNearZero = 1u << 2,   // LDP suspect: sampled valid-depth fraction below the threshold
  kRateSuspect = 1u << 3,     // published while the start-up rate check said "half"
  kCrcValid = 1u << 4,        // crc_colour / crc_depth filled (kv_cam --crc)
  kDepthUnaligned = 1u << 5,  // depth not registered to colour (no HW D2C profile): see header depth_w/h
};

enum WriterState : uint32_t { kStarting = 1, kStreaming = 2, kRestarting = 3, kStopped = 4 };
enum SourceKind : uint32_t { kSrcOrbbec = 1, kSrcReplay = 2, kSrcSynth = 3 };

// ---- page 0 of the segment ----
struct RingHeader {
  uint64_t magic;                    // 0
  uint32_t version;                  // 8
  uint32_t header_bytes;             // 12   = 4096
  uint32_t nslots;                   // 16
  uint32_t slot_hdr_bytes;           // 20   = 4096
  uint64_t slot_stride;              // 24   bytes per slot (page multiple)
  uint64_t slots_offset;             // 32   = 4096
  uint64_t total_bytes;              // 40
  uint32_t colour_w, colour_h, colour_fourcc, colour_stride, colour_bytes;  // 48..67
  uint32_t depth_w, depth_h, depth_fourcc, depth_stride, depth_bytes;       // 68..87
  uint32_t colour_off;               // 88   payload offset inside a slot (page aligned)
  uint32_t depth_off;                // 92
  uint64_t phys_base;                // 96   physical address of byte 0 when the backing is contiguous, else 0
  uint32_t writer_pid;               // 104
  uint32_t writer_epoch;             // 108  +1 every time a writer attaches
  std::atomic<uint64_t> heartbeat_ns;  // 112  writer CLOCK_MONOTONIC, >= 2 Hz even without frames
  std::atomic<uint64_t> latest_seq;    // 120  newest published sequence, 0 = none yet
  std::atomic<uint32_t> futex_word;    // 128  low 32 bits of latest_seq (futex wait target)
  std::atomic<uint32_t> state;         // 132  WriterState
  std::atomic<uint32_t> alarm_count;   // 136  +1 per alarm (alarm_text written before the increment)
  std::atomic<uint32_t> alarm_code;    // 140
  std::atomic<uint32_t> restarts;      // 144  stream restarts since the writer attached
  std::atomic<uint32_t> rate_events;   // 148  start-up rate check said "half"
  std::atomic<uint64_t> frames_published;    // 152
  std::atomic<uint64_t> near_zero_frames;    // 160
  std::atomic<uint64_t> incomplete_sets;     // 168  framesets missing colour or depth
  float fx, fy, cx, cy;              // 176  colour intrinsics (depth is registered to colour)
  float k1, k2, k3, k4, k5, k6, p1, p2;  // 192
  uint32_t intr_valid;               // 224
  uint32_t dist_model;               // 228  OBCameraDistortionModel
  float depth_scale_mm;              // 232  depth unit in mm (Gemini 2: 1.0)
  uint32_t source_kind;              // 236
  char alarm_text[128];              // 240  last alarm, NUL-terminated (diagnostic, may be torn)
  char writer_info[128];             // 368  device name / sn / fw / connection, NUL-terminated
  uint32_t nominal_fps;              // 496
  uint32_t reserved0;                // 500
  uint8_t reserved[4096 - 504];
};

// ---- page 0 of every slot ----
struct SlotHeader {
  std::atomic<uint64_t> seq_begin;   // 0
  std::atomic<uint64_t> seq_end;     // 8
  uint64_t seq;                      // 16   == seq_begin == seq_end when consistent
  uint64_t colour_index;             // 24   camera frame index (replay: pack frame index)
  uint64_t depth_index;              // 32
  uint64_t colour_dev_ts_us;         // 40   device clock
  uint64_t depth_dev_ts_us;          // 48
  uint64_t colour_sys_ts_us;         // 56   SDK host timestamp (system clock domain of the SDK)
  uint64_t depth_sys_ts_us;          // 64
  uint64_t arrive_ns;                // 72   CLOCK_MONOTONIC when kv_cam got the frameset
  uint64_t publish_ns;               // 80   CLOCK_MONOTONIC right before seq_end
  uint32_t flags;                    // 88   SlotFlags
  uint32_t colour_bytes;             // 92   valid bytes (0 when !kHasColour)
  uint32_t depth_bytes;              // 96
  uint32_t crc_colour;               // 100  zlib crc32, only with kCrcValid
  uint32_t crc_depth;                // 104
  uint32_t epoch;                    // 108  writer_epoch at publish
  float depth_scale_mm;              // 112
  uint32_t depth_valid_permille;     // 116  sampled fraction of non-zero depth (every 16th pixel)
  uint8_t reserved[4096 - 120];
};

static_assert(sizeof(RingHeader) == 4096, "RingHeader must be one page");
static_assert(sizeof(SlotHeader) == 4096, "SlotHeader must be one page");
static_assert(offsetof(RingHeader, heartbeat_ns) == 112 && offsetof(RingHeader, latest_seq) == 120 &&
                  offsetof(RingHeader, futex_word) == 128 && offsetof(RingHeader, fx) == 176 &&
                  offsetof(RingHeader, alarm_text) == 240 && offsetof(RingHeader, writer_info) == 368 &&
                  offsetof(RingHeader, nominal_fps) == 496,
              "RingHeader layout");
static_assert(offsetof(SlotHeader, seq) == 16 && offsetof(SlotHeader, arrive_ns) == 72 &&
                  offsetof(SlotHeader, flags) == 88 && offsetof(SlotHeader, depth_valid_permille) == 116,
              "SlotHeader layout");
static_assert(std::atomic<uint64_t>::is_always_lock_free && std::atomic<uint32_t>::is_always_lock_free,
              "lock-free atomics required in shared memory");

inline uint64_t mono_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return uint64_t(ts.tv_sec) * 1000000000ull + uint64_t(ts.tv_nsec);
}
inline uint64_t page_up(uint64_t n) { return (n + kPage - 1) / kPage * kPage; }

struct Geometry {
  uint32_t nslots = 8;
  uint32_t colour_w = 640, colour_h = 480, colour_fourcc = kFmtYUYV, colour_bpp = 2;
  uint32_t depth_w = 640, depth_h = 480, depth_fourcc = kFmtY16, depth_bpp = 2;
  uint32_t colour_bytes() const { return colour_w * colour_h * colour_bpp; }
  uint32_t depth_bytes() const { return depth_w * depth_h * depth_bpp; }
  uint64_t slot_stride() const { return kPage + page_up(colour_bytes()) + page_up(depth_bytes()); }
  uint64_t total() const { return kPage + uint64_t(nslots) * slot_stride(); }
};

// zlib-compatible CRC-32 (reflected 0xEDB88320). Slice-by-1 table; only used with kv_cam --crc.
inline uint32_t crc32(const uint8_t* p, size_t n, uint32_t crc = 0) {
  static uint32_t table[256];
  static bool init = [] {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      table[i] = c;
    }
    return true;
  }();
  (void)init;
  crc = ~crc;
  for (size_t i = 0; i < n; ++i) crc = table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

inline long futex(std::atomic<uint32_t>* addr, int op, uint32_t val, const timespec* ts) {
  return syscall(SYS_futex, reinterpret_cast<uint32_t*>(addr), op, val, ts, nullptr, 0);
}

// ================= writer =================
class RingWriter {
 public:
  ~RingWriter() { detach(); }

  // Open or create the segment. An existing segment with the same geometry is reused (sequence numbers continue,
  // epoch +1) so readers keep their mapping across a kv_cam restart; a different geometry is unlinked and
  // re-created (readers see the old segment's heartbeat stop and reopen). `path` empty = POSIX shm `name`;
  // otherwise a file / device to mmap (e.g. a udmabuf) with `phys_base` its physical address (0 = unknown).
  bool attach(const std::string& name, const Geometry& g, std::string* err, const std::string& path = "",
              uint64_t phys_base = 0, mode_t mode = 0644) {
    g_ = g;
    name_ = name;
    size_t total = g.total();
    bool by_path = !path.empty();
    int fd = by_path ? ::open(path.c_str(), O_RDWR | O_CREAT, mode) : shm_open(name.c_str(), O_RDWR | O_CREAT, mode);
    if (fd < 0) return fail(err, "open " + (by_path ? path : name) + ": " + strerror(errno));
    fchmod(fd, mode);  // umask must not make the ring unreadable for non-root readers
    struct stat st;
    fstat(fd, &st);
    bool reuse = false;
    if (size_t(st.st_size) == total) {
      void* p = mmap(nullptr, total, PROT_READ, MAP_SHARED, fd, 0);
      if (p != MAP_FAILED) {
        auto* h = static_cast<const RingHeader*>(p);
        reuse = h->magic == kMagic && h->version == kVersion && h->nslots == g.nslots &&
                h->slot_stride == g.slot_stride() && h->colour_w == g.colour_w && h->colour_h == g.colour_h &&
                h->colour_fourcc == g.colour_fourcc && h->depth_w == g.depth_w && h->depth_h == g.depth_h &&
                h->depth_fourcc == g.depth_fourcc;
        munmap(p, total);
      }
    }
    if (!reuse && !by_path && st.st_size != 0) {
      // geometry changed: fresh object so old readers keep a consistent (dead) mapping instead of a resized one
      ::close(fd);
      shm_unlink(name.c_str());
      fd = shm_open(name.c_str(), O_RDWR | O_CREAT | O_EXCL, mode);
      if (fd < 0) return fail(err, "recreate " + name + ": " + strerror(errno));
      fchmod(fd, mode);
    }
    if (!reuse && ftruncate(fd, off_t(total)) != 0 && !by_path) {
      int e = errno;
      ::close(fd);
      return fail(err, std::string("ftruncate: ") + strerror(e));
    }
    void* p = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) return fail(err, std::string("mmap: ") + strerror(errno));
    base_ = static_cast<uint8_t*>(p);
    total_ = total;
    h_ = reinterpret_cast<RingHeader*>(base_);
    if (!reuse) {
      // build the header with magic last, so a reader racing the creation sees magic 0 and retries
      std::memset(base_, 0, total);
      h_->version = kVersion;
      h_->header_bytes = kPage;
      h_->nslots = g.nslots;
      h_->slot_hdr_bytes = kPage;
      h_->slot_stride = g.slot_stride();
      h_->slots_offset = kPage;
      h_->total_bytes = total;
      h_->colour_w = g.colour_w;
      h_->colour_h = g.colour_h;
      h_->colour_fourcc = g.colour_fourcc;
      h_->colour_stride = g.colour_w * g.colour_bpp;
      h_->colour_bytes = g.colour_bytes();
      h_->depth_w = g.depth_w;
      h_->depth_h = g.depth_h;
      h_->depth_fourcc = g.depth_fourcc;
      h_->depth_stride = g.depth_w * g.depth_bpp;
      h_->depth_bytes = g.depth_bytes();
      h_->colour_off = kPage;
      h_->depth_off = uint32_t(kPage + page_up(g.colour_bytes()));
      h_->depth_scale_mm = 1.0f;
      std::atomic_thread_fence(std::memory_order_release);
      reinterpret_cast<std::atomic<uint64_t>*>(&h_->magic)->store(kMagic, std::memory_order_release);
    }
    h_->phys_base = phys_base;
    h_->writer_pid = uint32_t(getpid());
    h_->writer_epoch += 1;
    h_->state.store(kStarting, std::memory_order_release);
    heartbeat();
    return true;
  }

  void detach() {
    if (base_) {
      h_->state.store(kStopped, std::memory_order_release);
      munmap(base_, total_);
    }
    base_ = nullptr;
    h_ = nullptr;
  }

  RingHeader* header() { return h_; }
  const Geometry& geometry() const { return g_; }
  uint8_t* slot_base(uint64_t seq) { return base_ + kPage + ((seq - 1) % g_.nslots) * g_.slot_stride(); }

  // Reserve the next sequence: marks the slot as being written. Returns the sequence; pointers via out-args.
  uint64_t begin(SlotHeader** sh, uint8_t** colour, uint8_t** depth) {
    uint64_t s = h_->latest_seq.load(std::memory_order_relaxed) + 1;
    uint8_t* b = slot_base(s);
    auto* hdr = reinterpret_cast<SlotHeader*>(b);
    hdr->seq_begin.store(s, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);  // seq_begin visible before any payload byte
    *sh = hdr;
    *colour = b + h_->colour_off;
    *depth = b + h_->depth_off;
    pending_ = s;
    return s;
  }
  // Seal the slot reserved by begin() and announce it. `seq_override` != 0 is for fault injection only.
  void publish(uint64_t seq_override = 0) {
    uint64_t s = pending_;
    auto* hdr = reinterpret_cast<SlotHeader*>(slot_base(s));
    hdr->seq = seq_override ? seq_override : s;
    hdr->epoch = h_->writer_epoch;
    hdr->publish_ns = mono_ns();
    hdr->seq_end.store(s, std::memory_order_release);
    h_->latest_seq.store(s, std::memory_order_release);
    h_->frames_published.fetch_add(1, std::memory_order_relaxed);
    h_->futex_word.store(uint32_t(s), std::memory_order_release);
    futex(&h_->futex_word, FUTEX_WAKE, INT_MAX, nullptr);
    pending_ = 0;
  }

  void heartbeat() { h_->heartbeat_ns.store(mono_ns(), std::memory_order_release); }
  void set_state(WriterState s) { h_->state.store(s, std::memory_order_release); }
  void alarm(uint32_t code, const std::string& text) {
    std::snprintf(h_->alarm_text, sizeof(h_->alarm_text), "%s", text.c_str());
    h_->alarm_code.store(code, std::memory_order_relaxed);
    h_->alarm_count.fetch_add(1, std::memory_order_release);
  }

 private:
  static bool fail(std::string* err, const std::string& m) {
    if (err) *err = m;
    return false;
  }
  Geometry g_;
  std::string name_;
  uint8_t* base_ = nullptr;
  size_t total_ = 0;
  RingHeader* h_ = nullptr;
  uint64_t pending_ = 0;
};

// ================= reader =================
enum ReadStatus { kOk = 0, kNotYet = 1, kOverwritten = 2, kCorrupt = 3 };
inline const char* status_name(int s) {
  switch (s) {
    case kOk: return "ok";
    case kNotYet: return "not_yet";
    case kOverwritten: return "overwritten";
    default: return "corrupt";
  }
}

class RingReader {
 public:
  ~RingReader() { close(); }

  // Map read-only and validate. Fails (retry later) while the writer has not finished creating the segment.
  bool open(const std::string& name, std::string* err, const std::string& path = "") {
    close();
    int fd = path.empty() ? shm_open(name.c_str(), O_RDONLY, 0) : ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return fail(err, "open: " + std::string(strerror(errno)));
    struct stat st;
    fstat(fd, &st);
    if (size_t(st.st_size) < sizeof(RingHeader)) {
      ::close(fd);
      return fail(err, "segment too small (writer not ready)");
    }
    void* p = mmap(nullptr, size_t(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) return fail(err, std::string("mmap: ") + strerror(errno));
    base_ = static_cast<const uint8_t*>(p);
    total_ = size_t(st.st_size);
    h_ = reinterpret_cast<const RingHeader*>(base_);
    uint64_t magic = reinterpret_cast<const std::atomic<uint64_t>*>(&h_->magic)->load(std::memory_order_acquire);
    if (magic != kMagic || h_->version != kVersion || h_->total_bytes != total_ || h_->nslots == 0) {
      close();
      return fail(err, "bad magic/version/size (writer not ready or incompatible)");
    }
    return true;
  }
  void close() {
    if (base_) munmap(const_cast<uint8_t*>(base_), total_);
    base_ = nullptr;
    h_ = nullptr;
  }
  bool is_open() const { return base_ != nullptr; }
  const RingHeader* header() const { return h_; }

  uint64_t latest() const { return h_->latest_seq.load(std::memory_order_acquire); }
  // Writer gone? (no heartbeat for max_age_ns). A restarted writer with the same geometry keeps the segment;
  // a reader seeing stale() for long should close() and open() again.
  bool stale(uint64_t max_age_ns = 2000000000ull) const {
    return mono_ns() - h_->heartbeat_ns.load(std::memory_order_acquire) > max_age_ns;
  }

  // Wait until latest() > after or timeout. Returns latest().
  uint64_t wait_newer(uint64_t after, int timeout_ms) {
    uint64_t deadline = mono_ns() + uint64_t(timeout_ms) * 1000000ull;
    for (;;) {
      uint32_t w = h_->futex_word.load(std::memory_order_acquire);
      uint64_t l = latest();
      if (l > after) return l;
      uint64_t now = mono_ns();
      if (now >= deadline) return l;
      uint64_t left = deadline - now;
      if (left > 100000000ull) left = 100000000ull;  // re-check at least every 100 ms
      timespec ts{time_t(left / 1000000000ull), long(left % 1000000000ull)};
      futex(const_cast<std::atomic<uint32_t>*>(&h_->futex_word), FUTEX_WAIT, w, &ts);
    }
  }

  const SlotHeader* slot(uint64_t seq) const {
    return reinterpret_cast<const SlotHeader*>(base_ + h_->slots_offset + ((seq - 1) % h_->nslots) * h_->slot_stride);
  }
  const uint8_t* colour_ptr(uint64_t seq) const { return reinterpret_cast<const uint8_t*>(slot(seq)) + h_->colour_off; }
  const uint8_t* depth_ptr(uint64_t seq) const { return reinterpret_cast<const uint8_t*>(slot(seq)) + h_->depth_off; }

  // Zero-copy use: check before (state), use the pointers, then call still_valid(seq); false = discard the result.
  int state(uint64_t seq) const {
    if (seq == 0) return kNotYet;
    uint64_t e = slot(seq)->seq_end.load(std::memory_order_acquire);
    if (e == seq) return kOk;
    return e > seq ? kOverwritten : kNotYet;  // e < seq: not sealed yet (or a crashed writer's half slot)
  }
  bool still_valid(uint64_t seq) const {
    std::atomic_thread_fence(std::memory_order_acquire);
    return slot(seq)->seq_begin.load(std::memory_order_relaxed) == seq;
  }

  // Copy one frame out. colour/depth may be null (skip). `mid_copy_hook` is for tests (called between the
  // colour and the depth copy). `verify=false` skips the seqlock re-check (tests only: the negative control).
  template <typename Hook>
  int read_copy(uint64_t seq, SlotHeader* meta, uint8_t* colour, uint8_t* depth, Hook mid_copy_hook,
                bool verify = true) const {
    int st = state(seq);
    if (st != kOk) return st;
    const SlotHeader* sh = slot(seq);
    if (meta) std::memcpy(static_cast<void*>(meta), static_cast<const void*>(sh), sizeof(SlotHeader));
    if (colour) std::memcpy(colour, colour_ptr(seq), h_->colour_bytes);
    mid_copy_hook();
    if (depth) std::memcpy(depth, depth_ptr(seq), h_->depth_bytes);
    if (!verify) return kOk;
    if (!still_valid(seq)) return kOverwritten;
    if (meta && meta->seq != seq) return kCorrupt;
    return kOk;
  }
  int read_copy(uint64_t seq, SlotHeader* meta, uint8_t* colour, uint8_t* depth) const {
    return read_copy(seq, meta, colour, depth, [] {});
  }

 private:
  static bool fail(std::string* err, const std::string& m) {
    if (err) *err = m;
    return false;
  }
  const uint8_t* base_ = nullptr;
  size_t total_ = 0;
  const RingHeader* h_ = nullptr;
};

}  // namespace kvcam
