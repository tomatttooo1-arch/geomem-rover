// C API over cam_ring.h for ctypes (cam_ring.py) and other non-C++ consumers (NN-CAM1, 2026-11-04).
// The seqlock ordering (acquire loads / fences) lives here, so Python readers get the same guarantees as C++.
#include "cam_ring.h"

using kvcam::RingReader;

extern "C" {
void* kvcam_open(const char* name, const char* path) {
  auto* r = new RingReader;
  std::string err;
  if (!r->open(name ? name : kvcam::kDefaultName, &err, path ? path : "")) {
    delete r;
    return nullptr;
  }
  return r;
}
void kvcam_close(void* h) { delete static_cast<RingReader*>(h); }
// pointer to the 4096-byte ring header (read-only; parse with the documented offsets)
const void* kvcam_header(void* h) { return static_cast<RingReader*>(h)->header(); }
uint64_t kvcam_latest(void* h) { return static_cast<RingReader*>(h)->latest(); }
uint64_t kvcam_wait_newer(void* h, uint64_t after, int timeout_ms) {
  return static_cast<RingReader*>(h)->wait_newer(after, timeout_ms);
}
int kvcam_stale(void* h, uint64_t max_age_ns) { return static_cast<RingReader*>(h)->stale(max_age_ns) ? 1 : 0; }
// copy one frame: meta4096 / colour / depth may be NULL. Returns 0 ok, 1 not_yet, 2 overwritten, 3 corrupt.
int kvcam_read(void* h, uint64_t seq, void* meta4096, void* colour, void* depth) {
  return static_cast<RingReader*>(h)->read_copy(seq, static_cast<kvcam::SlotHeader*>(meta4096),
                                                 static_cast<uint8_t*>(colour), static_cast<uint8_t*>(depth));
}
}
