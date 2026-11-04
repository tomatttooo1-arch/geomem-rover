// DPUPIPE0 (2026-11-03): see dpu_rx.h.
#include "dpu_rx.h"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <sstream>

namespace dpupipe {

uint64_t mono_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}
uint64_t thread_cpu_ns() {
  timespec ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

uint32_t crc32_bytes(const uint8_t* p, size_t n) {
  uLong c = crc32(0L, Z_NULL, 0);
  while (n) {  // zlib takes uInt lengths
    uInt k = static_cast<uInt>(std::min<size_t>(n, 1u << 30));
    c = crc32(c, p, k);
    p += k;
    n -= k;
  }
  return static_cast<uint32_t>(c);
}

static int py_round(double x) { return static_cast<int>(std::nearbyint(x)); }  // half-to-even like round()

Geometry letterbox(int src_w, int src_h, int size) {
  // ultralytics 8.4.69 load_image (ceil) + LetterBox(scaleup=False, center=True); host/dpu_prep.py
  Geometry g;
  g.src_w = src_w;
  g.src_h = src_h;
  g.size = size;
  double r = static_cast<double>(size) / std::max(src_h, src_w);
  int lw = src_w, lh = src_h;
  if (r != 1.0) {
    lw = std::min(static_cast<int>(std::ceil(src_w * r)), size);
    lh = std::min(static_cast<int>(std::ceil(src_h * r)), size);
  }
  double r2 = std::min({static_cast<double>(size) / lh, static_cast<double>(size) / lw, 1.0});
  g.w = py_round(lw * r2);
  g.h = py_round(lh * r2);
  double dw = (size - g.w) / 2.0, dh = (size - g.h) / 2.0;
  g.top = py_round(dh - 0.1);
  g.left = py_round(dw - 0.1);
  return g;
}

int8_t quant_pad(int fp) {
  long q = (2L * 114 * (1L << fp) + 255) / 510;
  return static_cast<int8_t>(std::min(127L, q));
}

DpuRx::DpuRx(const RxConfig& cfg, FrameRing* ring) : cfg_(cfg), ring_(ring) {
  geo_ = letterbox(cfg.src_w, cfg.src_h, cfg.size);
  if (ring_->slot_bytes() < geo_.tensor_bytes()) throw std::invalid_argument("ring slot smaller than tensor");
  if (cfg_.chunk_bytes < 1 || cfg_.chunk_bytes > 65507 - CHUNK_HEAD) throw std::invalid_argument("chunk_bytes");
  scratch_.resize(65536);
  if (!geo_.direct()) staging_.resize(DPU_HEAD + geo_.content_bytes());
  std::random_device rd;
  session_ = (static_cast<uint64_t>(rd()) << 32) ^ rd();
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) throw std::runtime_error("socket");
  int v = cfg_.rcvbuf;
  if (setsockopt(fd_, SOL_SOCKET, SO_RCVBUFFORCE, &v, sizeof v) != 0)
    setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &v, sizeof v);
  socklen_t sl = sizeof rcvbuf_eff_;
  getsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_eff_, &sl);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  a.sin_port = htons(cfg_.bind_port);
  if (bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) throw std::runtime_error("bind");
  if (!cfg_.source.empty()) {
    auto pos = cfg_.source.rfind(':');
    src_.sin_family = AF_INET;
    src_.sin_port = htons(static_cast<uint16_t>(std::stoi(cfg_.source.substr(pos + 1))));
    if (inet_pton(AF_INET, cfg_.source.substr(0, pos).c_str(), &src_.sin_addr) != 1)
      throw std::invalid_argument("source address");
    have_src_ = true;
  }
  if (!cfg_.log_path.empty()) log_ = fopen(cfg_.log_path.c_str(), "w");
}

DpuRx::~DpuRx() {
  stop();
  if (fd_ >= 0) close(fd_);
  if (log_) fclose(log_);
}

void DpuRx::start() { th_ = std::thread([this] { run(); }); }
void DpuRx::stop() {
  stop_ = true;
  if (th_.joinable()) th_.join();
}

bool DpuRx::send_want(uint64_t want_id, uint64_t after_key) {
  uint8_t buf[CHUNK_HEAD + DPU_WANT];
  ChunkHead h{};
  std::memcpy(h.magic, "KVL1", 4);
  h.type = T_WANT_DPU;
  h.session = session_;
  h.frame_id = want_id;
  h.total = DPU_WANT;
  h.idx = 0;
  h.n = 1;
  DpuWant w{};
  w.ver = 1;
  w.fmt = 2;
  w.size = static_cast<uint16_t>(cfg_.size);
  w.fix_point = static_cast<uint8_t>(cfg_.fix_point);
  w.after_key = after_key;
  w.wait_ms = cfg_.wait_ms;
  w.chunk_bytes = cfg_.chunk_bytes;
  w.kv_t_ns = mono_ns();
  std::memcpy(buf, &h, CHUNK_HEAD);
  std::memcpy(buf + CHUNK_HEAD, &w, DPU_WANT);
  ssize_t k = sendto(fd_, buf, sizeof buf, 0, reinterpret_cast<sockaddr*>(&src_), sizeof src_);
  if (k == static_cast<ssize_t>(sizeof buf)) {
    ++counts_.wants;
    return true;
  }
  return false;
}

void DpuRx::drain_other(const ChunkHead& h, ssize_t len, const sockaddr_in& from) {
  (void)len;
  if (h.type == T_HELLO) {
    ++counts_.hellos;
    if (cfg_.source.empty()) {
      src_ = from;
      have_src_ = true;
    }
  } else if (h.type == T_FRAME_DPU || h.type == T_NONE) {
    ++counts_.stale_chunks;  // answer to an older WANT
  }
}

int DpuRx::receive_frame(uint64_t want_id, int slot, uint64_t t_want, FrameMeta* m) {
  const bool direct = geo_.direct();
  uint8_t* tensor = ring_->data(slot);
  uint8_t* base = direct ? tensor + geo_.content_offset() - DPU_HEAD : staging_.data();
  const uint32_t total = static_cast<uint32_t>(DPU_HEAD + geo_.content_bytes());
  const uint32_t cb = cfg_.chunk_bytes;
  const uint16_t n_exp = static_cast<uint16_t>((total + cb - 1) / cb);
  std::vector<uint8_t> seen(n_exp, 0);
  size_t have = 0;
  uint64_t t_first = 0;
  const int8_t pad = quant_pad(cfg_.fix_point);
  auto restore_pad = [&] {
    if (direct) std::memset(base, static_cast<uint8_t>(pad), DPU_HEAD);
  };
  const uint64_t first_deadline = t_want + (cfg_.wait_ms + 300ull) * 1000000ull;
  for (;;) {
    if (stop_) {
      restore_pad();
      return -1;
    }
    uint64_t now = mono_ns();
    uint64_t dl = t_first ? t_first + cfg_.frame_timeout_ms * 1000000ull : first_deadline;
    if (now >= dl) {
      if (t_first)
        ++counts_.incomplete;
      else
        ++counts_.want_timeout;
      restore_pad();
      return -1;
    }
    pollfd p{fd_, POLLIN, 0};
    int to = static_cast<int>((dl - now) / 1000000ull) + 1;
    if (poll(&p, 1, std::min(to, 50)) <= 0) continue;
    ChunkHead h;
    sockaddr_in from{};
    socklen_t fl = sizeof from;
    ssize_t k = recvfrom(fd_, &h, CHUNK_HEAD, MSG_PEEK | MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&from), &fl);
    if (k < 0) continue;
    if (k < static_cast<ssize_t>(CHUNK_HEAD) || std::memcmp(h.magic, "KVL1", 4) != 0) {
      recv(fd_, scratch_.data(), scratch_.size(), 0);
      ++counts_.malformed;
      continue;
    }
    if (h.type == T_FRAME_DPU && h.frame_id == want_id) {
      if (h.total != total || h.n != n_exp || h.idx >= h.n) {
        recv(fd_, scratch_.data(), scratch_.size(), 0);
        ++counts_.bad_head;  // wrong geometry / chunking for this receiver
        continue;
      }
      size_t off = static_cast<size_t>(h.idx) * cb;
      size_t len = std::min<size_t>(cb, total - off);
      ChunkHead h2;
      iovec iov[2] = {{&h2, CHUNK_HEAD}, {base + off, len}};
      msghdr msg{};
      msg.msg_iov = iov;
      msg.msg_iovlen = 2;
      ssize_t got = recvmsg(fd_, &msg, 0);
      ++counts_.chunks;
      if (got != static_cast<ssize_t>(CHUNK_HEAD + len) || (msg.msg_flags & MSG_TRUNC)) {
        ++counts_.malformed;
        continue;
      }
      if (!t_first) t_first = mono_ns();
      if (!seen[h.idx]) {
        seen[h.idx] = 1;
        ++have;
      }
      if (have == n_exp) break;
      continue;
    }
    ssize_t len = recvfrom(fd_, scratch_.data(), scratch_.size(), 0, reinterpret_cast<sockaddr*>(&from), &fl);
    if (h.type == T_NONE && h.frame_id == want_id) {
      uint8_t reason = len > static_cast<ssize_t>(CHUNK_HEAD + 1) ? scratch_[CHUNK_HEAD + 1] : 0;
      restore_pad();
      if (reason == 4) {
        ++counts_.none_no_new;
        return 0;
      }
      ++counts_.none_other;
      usleep(20000);
      return -1;
    }
    drain_other(h, len, from);
  }
  uint64_t t_done = mono_ns();
  DpuHead hd;
  std::memcpy(&hd, base, DPU_HEAD);
  restore_pad();
  if (hd.ver != 1 || hd.fmt != 2 || hd.size != geo_.size || hd.fix_point != cfg_.fix_point || hd.w != geo_.w ||
      hd.h != geo_.h || hd.top != geo_.top || hd.left != geo_.left || hd.content_bytes != geo_.content_bytes() ||
      hd.pad_q != pad) {
    ++counts_.bad_head;
    return -1;
  }
  if (!direct) {
    const uint8_t* src = staging_.data() + DPU_HEAD;
    for (int r = 0; r < geo_.h; ++r)
      std::memcpy(tensor + ((static_cast<size_t>(geo_.top) + r) * geo_.size + geo_.left) * 3,
                  src + static_cast<size_t>(r) * geo_.w * 3, static_cast<size_t>(geo_.w) * 3);
  }
  m->source_key = hd.source_key;
  m->seq = hd.seq;
  m->t_want_ns = t_want;
  m->t_first_ns = t_first;
  m->t_done_ns = t_done;
  m->prep_ns = hd.prep_ns;
  m->hold_ns = hd.hold_ns;
  m->crc32 = hd.crc32;
  m->size = hd.size;
  m->w = hd.w;
  m->h = hd.h;
  m->top = hd.top;
  m->left = hd.left;
  m->fix_point = hd.fix_point;
  m->pad_q = hd.pad_q;
  if (cfg_.verify_crc) {
    m->crc_checked = true;
    m->crc_ok = crc32_bytes(tensor + geo_.content_offset(), geo_.content_bytes()) == hd.crc32;
    if (!m->crc_ok) {
      ++counts_.crc_bad;
      return -1;
    }
  }
  return 1;
}

void DpuRx::run() {
  uint64_t want_id = 0, last_key = 0;
  while (!stop_) {
    if (!have_src_) {  // wait for the sender's HELLO
      pollfd p{fd_, POLLIN, 0};
      if (poll(&p, 1, 200) <= 0) continue;
      ChunkHead h;
      sockaddr_in from{};
      socklen_t fl = sizeof from;
      ssize_t len = recvfrom(fd_, scratch_.data(), scratch_.size(), 0, reinterpret_cast<sockaddr*>(&from), &fl);
      if (len >= static_cast<ssize_t>(CHUNK_HEAD)) {
        std::memcpy(&h, scratch_.data(), CHUNK_HEAD);
        if (std::memcmp(h.magic, "KVL1", 4) == 0) drain_other(h, len, from);
      }
      continue;
    }
    if (cfg_.max_inflight ? !ring_->wait_inflight_below(cfg_.max_inflight, 200)
                          : !ring_->wait_ready_below(cfg_.max_ahead, 200))
      continue;
    int slot = ring_->acquire_free(200);
    if (slot < 0) continue;
    for (;;) {
      if (stop_) {
        ring_->abort(slot);
        break;
      }
      uint64_t c0 = dpupipe::thread_cpu_ns();
      uint64_t t_want = mono_ns();
      if (!send_want(++want_id, last_key)) {
        usleep(10000);
        continue;
      }
      FrameMeta m;
      int r = receive_frame(want_id, slot, t_want, &m);
      uint64_t dc = dpupipe::thread_cpu_ns() - c0;
      cpu_ns_ += dc;
      if (r == 1) {
        m.rx_cpu_ns = dc;
        ++counts_.frames;
        last_key = m.source_key;
        if (log_) {
          fprintf(log_,
                  "{\"seq\":%u,\"source_key\":%llu,\"t_want_ns\":%llu,\"want_to_first_us\":%.1f,"
                  "\"first_to_done_us\":%.1f,\"want_to_done_us\":%.1f,\"hold_us\":%.1f,\"prep_us\":%.1f,"
                  "\"rx_cpu_us\":%.1f,\"crc_ok\":%d}\n",
                  m.seq, (unsigned long long)m.source_key, (unsigned long long)m.t_want_ns,
                  (m.t_first_ns - m.t_want_ns) / 1e3, (m.t_done_ns - m.t_first_ns) / 1e3,
                  (m.t_done_ns - m.t_want_ns) / 1e3, m.hold_ns / 1e3, m.prep_ns / 1e3, dc / 1e3,
                  m.crc_checked ? (int)m.crc_ok : -1);
          fflush(log_);
        }
        ring_->publish(slot, m);
        break;
      }
      // r == 0: no newer camera frame within wait_ms; r < 0: lost/incomplete: ask again, same slot
    }
  }
}

std::string DpuRx::describe_json() const {
  std::ostringstream o;
  o << "{\"size\":" << geo_.size << ",\"fix_point\":" << cfg_.fix_point << ",\"top\":" << geo_.top
    << ",\"content_bytes\":" << geo_.content_bytes() << ",\"direct\":" << (geo_.direct() ? "true" : "false")
    << ",\"chunk_bytes\":" << cfg_.chunk_bytes << ",\"rcvbuf\":" << rcvbuf_eff_ << ",\"wants\":" << counts_.wants
    << ",\"frames\":" << counts_.frames << ",\"none_no_new\":" << counts_.none_no_new
    << ",\"none_other\":" << counts_.none_other << ",\"incomplete\":" << counts_.incomplete
    << ",\"want_timeout\":" << counts_.want_timeout << ",\"bad_head\":" << counts_.bad_head
    << ",\"crc_bad\":" << counts_.crc_bad << ",\"stale_chunks\":" << counts_.stale_chunks
    << ",\"malformed\":" << counts_.malformed << ",\"hellos\":" << counts_.hellos << ",\"chunks\":" << counts_.chunks
    << ",\"rx_thread_cpu_ms\":" << cpu_ns_.load() / 1e6 << "}";
  return o.str();
}

bool StubBackend::run(int, const uint8_t* input, size_t bytes, const FrameMeta&) {
  if (tensor_crc_) last_crc = crc32_bytes(input, bytes);
  if (infer_ms_ > 0) {
    timespec ts{static_cast<time_t>(infer_ms_ / 1000), static_cast<long>(std::fmod(infer_ms_, 1000.0) * 1e6)};
    nanosleep(&ts, nullptr);  // the DPU works, the A53 waits (VART wait() blocks the same way)
  }
  return true;
}

}  // namespace dpupipe
