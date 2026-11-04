// NN-CAM1 (2026-11-04): pieces shared by kv_cam (writer) and kv_cam_read (reader / verifier):
// the frame-source interface, the replay pack (KVCAMPK1) and the deterministic synthetic frame generator.
#pragma once

#include "cam_ring.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace kvcam {

// ---------------- one frameset handed from a source to the ring ----------------
struct FrameView {
  const uint8_t* colour = nullptr;  // null = no colour in this frameset
  uint32_t colour_bytes = 0;
  const uint8_t* depth = nullptr;   // null = no depth
  uint32_t depth_bytes = 0;
  uint32_t depth_w = 0, depth_h = 0;  // actual depth size (640x400 when unaligned)
  uint64_t colour_index = 0, depth_index = 0;
  uint64_t colour_dev_us = 0, depth_dev_us = 0, colour_sys_us = 0, depth_sys_us = 0;
  uint64_t arrive_ns = 0;
  float depth_scale_mm = 1.0f;
  bool depth_unaligned = false;
};
using FrameSink = std::function<void(const FrameView&)>;

struct CamInfo {
  std::string text;           // device name / sn / fw / connection
  bool usb3 = true;           // connection type reports USB3.x
  bool hw_align = true;       // depth registered to colour by the camera
  bool intr_valid = false;
  float fx = 0, fy = 0, cx = 0, cy = 0, k[6] = {0, 0, 0, 0, 0, 0}, p1 = 0, p2 = 0;
  uint32_t dist_model = 0;
  std::string colour_fmt, depth_fmt;
};

struct SourceOptions {
  int fps = 30;
  std::string colour_format = "yuyv";  // yuyv | rgb
  std::string align = "hw";            // hw | none
  bool sync = true;
  std::string aggregate = "any";       // any | full
  std::string sdk_log_dir;             // empty = SDK console warnings only
  std::string streams = "both";        // both | depth | colour  (diagnosis arms only; the ring keeps its geometry)
  bool dump_props = false;             // print the depth-processing properties at every start
  std::vector<std::pair<int, int>> set_props;  // --set-prop ID:VAL, applied before the stream starts (diagnosis)
  // replay / synth
  std::string pack;                    // replay pack file
  int replay_fps = 30;                 // 0 = as fast as possible
  long frames = 0;                     // stop after this many framesets (0 = endless / pack x loops)
  int loops = 1;                       // replay: passes over the pack (0 = endless)
  int fail_starts = 0;                 // replay / synth: simulate this many failed starts (no depth) first
  // fault injection (tests only; never on by default): from published frame K on, the content of STREAM comes
  // from source frame index+1 while the meta still says index -> "sequence misalignment"
  std::string inject_stream;           // "" | colour | depth | both
  long inject_from = 0;
};

class Source {
 public:
  virtual ~Source() = default;
  // Start streaming into sink (called on the source's own thread). false + err = failed start.
  virtual bool start(const FrameSink& sink, CamInfo* info, std::string* err) = 0;
  virtual void stop() = 0;
  virtual bool finished() const { return false; }  // replay ran out: clean exit
};

std::unique_ptr<Source> make_orbbec_source(const SourceOptions& o);  // kv_cam_orbbec.cpp or the stub
bool orbbec_available();
int orbbec_list(const SourceOptions& o);                              // --list

// ---------------- deterministic synthetic frames ----------------
inline uint64_t splitmix64(uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}
// stream 0 = colour, 1 = depth. Every 8-byte word depends on (index, stream, word position).
inline void synth_fill(uint8_t* dst, size_t n, uint64_t index, int stream) {
  uint64_t key = splitmix64(index * 2 + uint64_t(stream) + 0x5EED);
  size_t words = n / 8;
  for (size_t j = 0; j < words; ++j) {
    uint64_t v = splitmix64(key ^ (uint64_t(j) * 0x100000001B3ull));
    std::memcpy(dst + j * 8, &v, 8);
  }
  for (size_t j = words * 8; j < n; ++j) dst[j] = uint8_t(splitmix64(key + j));
}

// ---------------- replay pack KVCAMPK1 ----------------
// [PackHeader 64 B] then nframes x ([PackRecord 32 B][colour bytes][depth bytes]). Little-endian.
constexpr uint64_t kPackMagic = 0x314b504d4143564bull;  // "KVCAMPK1"
struct PackHeader {
  uint64_t magic;
  uint32_t version, nframes;
  uint32_t colour_w, colour_h, colour_fourcc, colour_bytes;
  uint32_t depth_w, depth_h, depth_fourcc, depth_bytes;
  uint8_t reserved[64 - 48];
};
struct PackRecord {
  uint64_t index, colour_dev_us, depth_dev_us, reserved;
};
static_assert(sizeof(PackHeader) == 64 && sizeof(PackRecord) == 32, "pack layout");

class Pack {
 public:
  ~Pack() {
    if (base_) munmap(const_cast<uint8_t*>(base_), size_);
  }
  bool open(const std::string& path, std::string* err) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
      if (err) *err = "open pack " + path + ": " + strerror(errno);
      return false;
    }
    struct stat st;
    fstat(fd, &st);
    size_ = size_t(st.st_size);
    void* p = size_ >= sizeof(PackHeader) ? mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd, 0) : MAP_FAILED;
    ::close(fd);
    if (p == MAP_FAILED) {
      if (err) *err = "mmap pack failed / too small";
      return false;
    }
    base_ = static_cast<const uint8_t*>(p);
    h_ = reinterpret_cast<const PackHeader*>(base_);
    stride_ = sizeof(PackRecord) + size_t(h_->colour_bytes) + h_->depth_bytes;
    if (h_->magic != kPackMagic || h_->version != 1 || sizeof(PackHeader) + stride_ * h_->nframes != size_) {
      if (err) *err = "bad pack header or size";
      return false;
    }
    return true;
  }
  const PackHeader& header() const { return *h_; }
  uint32_t n() const { return h_ ? h_->nframes : 0; }
  const PackRecord& rec(uint32_t i) const {
    return *reinterpret_cast<const PackRecord*>(base_ + sizeof(PackHeader) + stride_ * i);
  }
  const uint8_t* colour(uint32_t i) const { return base_ + sizeof(PackHeader) + stride_ * i + sizeof(PackRecord); }
  const uint8_t* depth(uint32_t i) const { return colour(i) + h_->colour_bytes; }

 private:
  const uint8_t* base_ = nullptr;
  size_t size_ = 0, stride_ = 0;
  const PackHeader* h_ = nullptr;
};

}  // namespace kvcam
