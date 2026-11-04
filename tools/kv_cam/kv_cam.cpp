// kv_cam (NN-CAM1, 2026-11-04): the KV260 camera process. Orbbec Gemini 2 -> shared-memory frame ring
// (cam_ring.h, protocol.md) for kv_server, dpu_rx / the PL preprocessing driver, the front end, the car screen.
//
// Default = the decided car configuration (NONANO0 DESIGN section 7): colour 640x480 YUYV @30 (no host decode, the
// colour conversion is in the PL preprocessing kernel), depth 640x400 Y16 @30 with hardware D2C (delivered 640x480),
// frame sync on, aggregate ANY (as measured by CAMKV0).
// Health (alarm + automatic restart, never a refusal to run - FPGA rule):
//   * start retry: a start that does not deliver --start-depth-frames depth frames in --start-wait-s is stopped and
//     tried again after --retry-gap-s, without limit (the Gemini 2 "depth stream does not come up" quirk)
//   * start-up rate check (30 fps only): device-timestamp intervals over --rate-window-s; the half-rate state
//     (depth median < 32.8 ms or colour p25 > 50 ms, tools/car_run/depth_probe.py) -> restart, up to
//     --rate-max-restarts times in a row, then keep streaming with the slots flagged kRateSuspect
//   * stall: no frameset for --stall-s -> restart
//   * near-range laser protection (LDP) suspect: sampled valid depth < --ldp-permille for --ldp-frames frames ->
//     alarm (+ slot flag); restart only with --ldp-restart (default off: a restart cannot clear a physical cause)
// Sources: --source orbbec (board; built with OrbbecSDK) | replay --pack FILE | synth (deterministic frames).
// Self-check: --selftest (no camera: ring round trip, classifier, LDP sampler) and an on-start profile check.
#include "kv_cam_common.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sched.h>
#include <sstream>
#include <sys/resource.h>
#include <thread>
#include <vector>

using namespace kvcam;

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

struct Options {
  std::string source = "orbbec";
  std::string name = kDefaultName;
  std::string path;          // mmap a file / device instead of POSIX shm
  uint64_t phys_base = 0;
  uint32_t nslots = 8;
  SourceOptions src;
  double start_wait_s = 20, retry_gap_s = 10, stall_s = 3;
  int start_depth_frames = 30;
  bool rate_check = true;
  double rate_window_s = 3;
  int rate_max_restarts = 5;
  int ldp_permille = 20, ldp_frames = 30;
  bool ldp_restart = false;
  double ldp_restart_s = 5;
  bool crc = false;
  int cpu = -1, nice_v = 0;
  std::string alarm_log, stats_out;
  double status_every_s = 10;
  bool selftest = false, list = false;
};

// ---------------- alarms ----------------
enum AlarmCode : uint32_t {
  kAlStartFail = 1, kAlDepthNotStarted = 2, kAlRateHalf = 3, kAlRateKeep = 4, kAlStall = 5, kAlLdp = 6,
  kAlProfile = 7, kAlUsb2 = 8, kAlNoHwAlign = 9, kAlFrameSize = 10, kAlSourceError = 11, kAlInfo = 100,
};
struct Alarms {
  RingWriter* ring = nullptr;
  std::string path;
  std::mutex mu;
  uint64_t count = 0;
  void raise(uint32_t code, const std::string& text) {
    std::lock_guard<std::mutex> lk(mu);
    ++count;
    std::fprintf(stderr, "KVCAM ALARM code=%u %s\n", code, text.c_str());
    if (ring && ring->header()) ring->alarm(code, text);
    if (!path.empty()) {
      FILE* f = std::fopen(path.c_str(), "a");
      if (f) {
        std::string t = text;
        for (auto& c : t)
          if (c == '"' || c == '\\') c = '\'';
        std::fprintf(f, "{\"mono_ns\": %llu, \"wall\": %lld, \"code\": %u, \"text\": \"%s\"}\n",
                     (unsigned long long)mono_ns(), (long long)time(nullptr), code, t.c_str());
        std::fclose(f);
      }
    }
  }
};

// ---------------- start-up rate classifier (same rule as tools/car_run/depth_probe.py) ----------------
constexpr double kDepthHalfMs = 32.8, kColourHalfMs = 50.0;
constexpr int kMinDepthIv = 20, kMinColourIv = 10;
double quantile(std::vector<double> v, double q) {  // numpy "linear"
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  double pos = q * double(v.size() - 1);
  size_t i = size_t(pos);
  double fr = pos - double(i);
  return i + 1 < v.size() ? v[i] * (1 - fr) + v[i + 1] * fr : v[i];
}
// returns "normal" | "half" | "insufficient"
std::string classify_rate(const std::vector<double>& depth_iv_ms, const std::vector<double>& colour_iv_ms,
                          std::string* why) {
  if (int(depth_iv_ms.size()) < kMinDepthIv || int(colour_iv_ms.size()) < kMinColourIv) {
    if (why) *why = "intervals depth " + std::to_string(depth_iv_ms.size()) + " colour " + std::to_string(colour_iv_ms.size());
    return "insufficient";
  }
  double dm = quantile(depth_iv_ms, 0.5), c25 = quantile(colour_iv_ms, 0.25);
  char b[160];
  std::snprintf(b, sizeof b, "depth median %.2f ms (half < %.1f), colour p25 %.2f ms (half > %.1f)", dm, kDepthHalfMs,
                c25, kColourHalfMs);
  if (why) *why = b;
  return (dm < kDepthHalfMs || c25 > kColourHalfMs) ? "half" : "normal";
}

// sampled valid-depth fraction (every 16th pixel), in permille
uint32_t depth_valid_permille(const uint8_t* depth, uint32_t bytes) {
  const uint16_t* d = reinterpret_cast<const uint16_t*>(depth);
  uint32_t n = bytes / 2, tot = 0, ok = 0;
  for (uint32_t i = 0; i < n; i += 16, ++tot) ok += d[i] != 0;
  return tot ? uint32_t(uint64_t(ok) * 1000 / tot) : 0;
}

// ---------------- the ring sink: one frameset -> one slot ----------------
struct Sink {
  Sink(RingWriter* r, Alarms* a, const Options* op) : ring(r), alarms(a), o(op) {}
  RingWriter* ring;
  Alarms* alarms;
  const Options* o;
  std::mutex mu;  // the SDK may call from more than one thread
  std::atomic<uint64_t> sets{0}, depth_frames{0}, colour_frames{0}, last_arrive_ns{0};
  std::atomic<bool> rate_suspect{false};
  // rate-check collection
  std::mutex rmu;
  bool collecting = false;
  uint64_t collect_skip = 0;
  uint64_t last_d_dev = 0, last_c_dev = 0;
  std::vector<double> d_iv, c_iv;
  // LDP episode
  uint32_t near_run = 0;
  bool ldp_active = false;
  uint64_t ldp_since_ns = 0;
  std::atomic<bool> ldp_restart_wanted{false};
  uint64_t size_alarms = 0;

  void start_collect() {
    std::lock_guard<std::mutex> lk(rmu);
    collecting = true;
    collect_skip = 5;  // skip the first-frames transient (depth_probe.py section 3.3)
    last_d_dev = last_c_dev = 0;
    d_iv.clear();
    c_iv.clear();
  }
  void stop_collect(std::vector<double>* d, std::vector<double>* c) {
    std::lock_guard<std::mutex> lk(rmu);
    collecting = false;
    *d = d_iv;
    *c = c_iv;
  }

  void operator()(const FrameView& f) {
    std::lock_guard<std::mutex> lk(mu);
    const RingHeader* h = ring->header();
    SlotHeader* sh;
    uint8_t *col, *dep;
    ring->begin(&sh, &col, &dep);
    uint32_t flags = 0;
    if (f.colour) {
      uint32_t n = std::min(f.colour_bytes, h->colour_bytes);
      std::memcpy(col, f.colour, n);
      if (n < h->colour_bytes) std::memset(col + n, 0, h->colour_bytes - n);
      if (f.colour_bytes != h->colour_bytes && size_alarms++ < 5)
        alarms->raise(kAlFrameSize, "colour frame " + std::to_string(f.colour_bytes) + " B, ring slot " +
                                        std::to_string(h->colour_bytes) + " B");
      flags |= kHasColour;
      sh->colour_bytes = n;
      colour_frames++;
    } else {
      sh->colour_bytes = 0;
    }
    uint32_t vp = 0;
    if (f.depth) {
      uint32_t n = std::min(f.depth_bytes, h->depth_bytes);
      std::memcpy(dep, f.depth, n);
      if (n < h->depth_bytes) std::memset(dep + n, 0, h->depth_bytes - n);
      if (f.depth_bytes != h->depth_bytes && !f.depth_unaligned && size_alarms++ < 5)
        alarms->raise(kAlFrameSize, "depth frame " + std::to_string(f.depth_bytes) + " B, ring slot " +
                                        std::to_string(h->depth_bytes) + " B");
      flags |= kHasDepth;
      if (f.depth_unaligned) flags |= kDepthUnaligned;
      sh->depth_bytes = n;
      vp = depth_valid_permille(f.depth, n);
      depth_frames++;
    } else {
      sh->depth_bytes = 0;
    }
    if (!f.colour || !f.depth) ring->header()->incomplete_sets.fetch_add(1, std::memory_order_relaxed);
    // LDP suspect
    if (f.depth) {
      if (int(vp) < o->ldp_permille) {
        flags |= kDepthNearZero;
        ring->header()->near_zero_frames.fetch_add(1, std::memory_order_relaxed);
        if (++near_run == uint32_t(o->ldp_frames) && !ldp_active) {
          ldp_active = true;
          ldp_since_ns = mono_ns();
          alarms->raise(kAlLdp, "depth near-zero for " + std::to_string(near_run) + " frames (valid " +
                                    std::to_string(vp) + " permille): LDP suspect (object too close?)");
        }
        if (ldp_active && o->ldp_restart && mono_ns() - ldp_since_ns > uint64_t(o->ldp_restart_s * 1e9))
          ldp_restart_wanted.store(true);
      } else {
        if (ldp_active) alarms->raise(kAlInfo, "LDP suspect cleared after " + std::to_string(near_run) + " frames");
        near_run = 0;
        ldp_active = false;
      }
    }
    if (rate_suspect.load(std::memory_order_relaxed)) flags |= kRateSuspect;
    sh->colour_index = f.colour_index;
    sh->depth_index = f.depth_index;
    sh->colour_dev_ts_us = f.colour_dev_us;
    sh->depth_dev_ts_us = f.depth_dev_us;
    sh->colour_sys_ts_us = f.colour_sys_us;
    sh->depth_sys_ts_us = f.depth_sys_us;
    sh->arrive_ns = f.arrive_ns ? f.arrive_ns : mono_ns();
    sh->depth_scale_mm = f.depth_scale_mm;
    sh->depth_valid_permille = vp;
    if (o->crc) {
      sh->crc_colour = f.colour ? crc32(col, h->colour_bytes) : 0;
      sh->crc_depth = f.depth ? crc32(dep, h->depth_bytes) : 0;
      flags |= kCrcValid;
    }
    sh->flags = flags;
    ring->publish();
    sets++;
    last_arrive_ns.store(mono_ns(), std::memory_order_release);
    // rate-check intervals on device timestamps
    std::lock_guard<std::mutex> rl(rmu);
    if (collecting) {
      if (collect_skip) {
        --collect_skip;
      } else {
        if (f.depth && f.depth_dev_us) {
          if (last_d_dev && f.depth_dev_us > last_d_dev) d_iv.push_back((f.depth_dev_us - last_d_dev) / 1000.0);
          last_d_dev = f.depth_dev_us;
        }
        if (f.colour && f.colour_dev_us) {
          if (last_c_dev && f.colour_dev_us > last_c_dev) c_iv.push_back((f.colour_dev_us - last_c_dev) / 1000.0);
          last_c_dev = f.colour_dev_us;
        }
      }
    }
  }
};

// ---------------- replay / synth source ----------------
class ReplaySource : public Source {
 public:
  explicit ReplaySource(const SourceOptions& o, const Geometry& g) : o_(o), g_(g) {}
  ~ReplaySource() override { stop(); }
  bool start(const FrameSink& sink, CamInfo* info, std::string* err) override {
    stop();
    if (!o_.pack.empty() && !pack_) {
      pack_.reset(new Pack);
      if (!pack_->open(o_.pack, err)) {
        pack_.reset();
        return false;
      }
      const auto& ph = pack_->header();
      if (ph.colour_bytes != g_.colour_bytes() || ph.depth_bytes != g_.depth_bytes() ||
          ph.colour_fourcc != g_.colour_fourcc || ph.depth_fourcc != g_.depth_fourcc) {
        if (err) *err = "pack geometry differs from the ring geometry";
        pack_.reset();
        return false;
      }
    }
    if (info) {
      info->text = pack_ ? "replay " + o_.pack : std::string("synth");
      info->colour_fmt = g_.colour_fourcc == kFmtYUYV ? "YUYV" : "RGB";
      info->depth_fmt = "Y16";
    }
    bool fail_this = starts_++ < o_.fail_starts;  // simulated "depth stream did not come up"
    run_.store(true);
    th_ = std::thread([this, sink, fail_this] { loop(sink, fail_this); });
    return true;
  }
  void stop() override {
    run_.store(false);
    if (th_.joinable()) th_.join();
  }
  bool finished() const override { return done_.load(); }

 private:
  void loop(FrameSink sink, bool no_depth) {
    std::vector<uint8_t> cbuf(g_.colour_bytes()), dbuf(g_.depth_bytes()), cnext(g_.colour_bytes()),
        dnext(g_.depth_bytes());
    uint64_t period_ns = o_.replay_fps > 0 ? 1000000000ull / uint64_t(o_.replay_fps) : 0;
    uint64_t next_t = mono_ns();
    uint64_t n_pack = pack_ ? pack_->n() : 0;
    while (run_.load()) {
      if (o_.frames > 0 && long(emitted_) >= o_.frames) break;
      if (pack_ && o_.loops > 0 && g_index_ >= n_pack * uint64_t(o_.loops)) break;
      uint64_t idx = g_index_;  // global frame index (replay: loop * n + i)
      const uint8_t *c = nullptr, *d = nullptr;
      auto content = [&](uint64_t k, std::vector<uint8_t>& cb, std::vector<uint8_t>& db, const uint8_t** cp,
                         const uint8_t** dp) {
        if (pack_) {
          *cp = pack_->colour(uint32_t(k % n_pack));
          *dp = pack_->depth(uint32_t(k % n_pack));
        } else {
          synth_fill(cb.data(), cb.size(), k, 0);
          synth_fill(db.data(), db.size(), k, 1);
          *cp = cb.data();
          *dp = db.data();
        }
      };
      content(idx, cbuf, dbuf, &c, &d);
      // fault injection: from emitted frame inject_from on, the chosen stream carries frame idx+1's bytes
      if (!o_.inject_stream.empty() && long(emitted_) >= o_.inject_from) {
        const uint8_t *c2, *d2;
        content(idx + 1, cnext, dnext, &c2, &d2);
        if (o_.inject_stream == "colour" || o_.inject_stream == "both") c = c2;
        if (o_.inject_stream == "depth" || o_.inject_stream == "both") d = d2;
      }
      FrameView f;
      f.colour = c;
      f.colour_bytes = g_.colour_bytes();
      if (!no_depth) {
        f.depth = d;
        f.depth_bytes = g_.depth_bytes();
      }
      f.depth_w = g_.depth_w;
      f.depth_h = g_.depth_h;
      f.colour_index = f.depth_index = idx;
      // device clock: nominal 33.40 ms steps (the Gemini 2's real 30 fps period), depth 3.3 ms before colour
      f.colour_dev_us = 1000000 + idx * 33400;
      f.depth_dev_us = f.colour_dev_us - 3300;
      // (the pack's recorded device stamps are not used: recordings are often subsampled, which would trip the
      // start-up rate check; the replay clock is the nominal camera clock)
      f.arrive_ns = mono_ns();
      sink(f);
      ++emitted_;
      ++g_index_;
      if (period_ns) {
        next_t += period_ns;
        uint64_t now = mono_ns();
        if (next_t > now) std::this_thread::sleep_for(std::chrono::nanoseconds(next_t - now));
        else next_t = now;
      }
    }
    if (run_.load()) done_.store(true);
  }
  SourceOptions o_;
  Geometry g_;
  std::unique_ptr<Pack> pack_;
  std::thread th_;
  std::atomic<bool> run_{false}, done_{false};
  int starts_ = 0;
  uint64_t emitted_ = 0, g_index_ = 0;
};

// ---------------- helpers ----------------
double proc_cpu_s() {
  std::ifstream f("/proc/self/stat");
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  auto p = s.rfind(')');
  if (p == std::string::npos) return 0;
  std::istringstream is(s.substr(p + 2));
  std::string tok;
  unsigned long long ut = 0, st = 0;
  for (int i = 3; i <= 15 && is >> tok; i++) {
    if (i == 14) ut = std::stoull(tok);
    if (i == 15) st = std::stoull(tok);
  }
  return double(ut + st) / double(sysconf(_SC_CLK_TCK));
}

// sleep in small steps, keeping the heartbeat alive; false if asked to stop
bool idle_for(RingWriter& ring, double s) {
  uint64_t end = mono_ns() + uint64_t(s * 1e9);
  while (!g_stop.load() && mono_ns() < end) {
    ring.heartbeat();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return !g_stop.load();
}

Geometry geometry_for(const Options& o) {
  Geometry g;
  g.nslots = o.nslots;
  if (o.src.colour_format == "rgb") {
    g.colour_fourcc = kFmtRGB;
    g.colour_bpp = 3;
  }
  return g;
}

void publish_info(RingWriter& ring, const CamInfo& ci) {
  RingHeader* h = ring.header();
  std::snprintf(h->writer_info, sizeof h->writer_info, "%s", ci.text.c_str());
  if (ci.intr_valid) {
    h->fx = ci.fx;
    h->fy = ci.fy;
    h->cx = ci.cx;
    h->cy = ci.cy;
    h->k1 = ci.k[0]; h->k2 = ci.k[1]; h->k3 = ci.k[2]; h->k4 = ci.k[3]; h->k5 = ci.k[4]; h->k6 = ci.k[5];
    h->p1 = ci.p1;
    h->p2 = ci.p2;
    h->dist_model = ci.dist_model;
  }
  h->intr_valid = ci.intr_valid ? 1 : 0;
}

// ---------------- the supervisor ----------------
int run(const Options& o) {
  Geometry g = geometry_for(o);
  RingWriter ring;
  std::string err;
  if (!ring.attach(o.name, g, &err, o.path, o.phys_base)) {
    std::fprintf(stderr, "kv_cam: cannot create ring: %s\n", err.c_str());
    return 2;  // nothing to publish into: the service manager restarts the process
  }
  RingHeader* h = ring.header();
  h->nominal_fps = uint32_t(o.src.fps);
  h->source_kind = o.source == "orbbec" ? kSrcOrbbec : o.source == "replay" ? kSrcReplay : kSrcSynth;
  Alarms alarms;
  alarms.ring = &ring;
  alarms.path = o.alarm_log;
  Sink sink(&ring, &alarms, &o);
  std::printf("kv_cam: ring /%s %u slots x %llu B (%.1f MB), colour %ux%u %s, depth %ux%u Y16, epoch %u, source %s\n",
              o.name.c_str(), g.nslots, (unsigned long long)g.slot_stride(), g.total() / 1e6, g.colour_w, g.colour_h,
              g.colour_fourcc == kFmtYUYV ? "YUYV" : "RGB", g.depth_w, g.depth_h, h->writer_epoch, o.source.c_str());
  std::fflush(stdout);

  std::unique_ptr<Source> src;
  if (o.source == "orbbec") src = make_orbbec_source(o.src);
  else src.reset(new ReplaySource(o.src, g));
  if (!src) {
    std::fprintf(stderr, "kv_cam: source %s not available in this build\n", o.source.c_str());
    return 2;
  }
  FrameSink fs = [&sink](const FrameView& f) { sink(f); };

  uint64_t tries = 0, consecutive_fail = 0;
  int rate_restarts = 0;
  bool first_ok_start = true;
  double cpu0 = proc_cpu_s();
  uint64_t t_run0 = mono_ns();
  while (!g_stop.load()) {
    ++tries;
    ring.set_state(tries == 1 ? kStarting : kRestarting);
    CamInfo ci;
    uint64_t d0 = sink.depth_frames.load();
    if (!src->start(fs, &ci, &err)) {
      ++consecutive_fail;
      alarms.raise(kAlStartFail, "start try " + std::to_string(tries) + " failed: " + err +
                                     (consecutive_fail >= 10 ? " (still retrying, no limit)" : ""));
      if (!idle_for(ring, o.retry_gap_s)) break;
      continue;
    }
    publish_info(ring, ci);
    // ---- start check: depth frames arriving ----
    uint64_t t0 = mono_ns();
    while (!g_stop.load() && !src->finished() && sink.depth_frames.load() - d0 < uint64_t(o.start_depth_frames) &&
           mono_ns() - t0 < uint64_t(o.start_wait_s * 1e9)) {
      ring.heartbeat();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (g_stop.load()) break;
    uint64_t got = sink.depth_frames.load() - d0;
    if (got < uint64_t(o.start_depth_frames) && !src->finished()) {
      src->stop();
      ++consecutive_fail;
      h->restarts.fetch_add(1);
      alarms.raise(kAlDepthNotStarted, "start try " + std::to_string(tries) + ": " + std::to_string(got) +
                                           " depth frames in " + std::to_string(int(o.start_wait_s)) +
                                           " s; stop, wait, start again");
      if (!idle_for(ring, o.retry_gap_s)) break;
      continue;
    }
    // ---- on-start self-check (alarms only) ----
    if (first_ok_start) {
      first_ok_start = false;
      if (!ci.usb3) alarms.raise(kAlUsb2, "camera link is not USB3: " + ci.text);
      if (!ci.hw_align) alarms.raise(kAlNoHwAlign, "no hardware D2C profile; depth is unaligned (slot flag)");
      std::printf("kv_cam: started (try %llu): %s colour %s depth %s\n", (unsigned long long)tries, ci.text.c_str(),
                  ci.colour_fmt.c_str(), ci.depth_fmt.c_str());
      std::fflush(stdout);
    }
    // ---- start-up rate check (30 fps only; the half-rate rule is defined for 33.4 ms) ----
    if (o.rate_check && o.src.fps == 30 && !src->finished()) {
      sink.start_collect();
      idle_for(ring, o.rate_window_s);
      std::vector<double> div, civ;
      sink.stop_collect(&div, &civ);
      std::string why;
      std::string v = classify_rate(div, civ, &why);
      if (v == "half") {
        h->rate_events.fetch_add(1);
        if (++rate_restarts <= o.rate_max_restarts) {
          src->stop();
          h->restarts.fetch_add(1);
          alarms.raise(kAlRateHalf, "half-rate state at start (" + why + "); restart " +
                                        std::to_string(rate_restarts) + "/" + std::to_string(o.rate_max_restarts));
          if (!idle_for(ring, o.retry_gap_s)) break;
          continue;
        }
        sink.rate_suspect.store(true);
        alarms.raise(kAlRateKeep, "half-rate state persists after " + std::to_string(o.rate_max_restarts) +
                                      " restarts; streaming on with slots flagged rate-suspect (" + why + ")");
      } else if (v == "insufficient") {
        alarms.raise(kAlInfo, "rate check insufficient (" + why + "); streaming on");
      } else {
        rate_restarts = 0;
        sink.rate_suspect.store(false);
      }
    }
    consecutive_fail = 0;
    ring.set_state(kStreaming);
    // ---- streaming: heartbeat, stall, LDP restart, status ----
    uint64_t last_status = mono_ns();
    double cpu_last = proc_cpu_s();
    uint64_t sets_last = sink.sets.load();
    bool restart = false;
    while (!g_stop.load()) {
      ring.heartbeat();
      if (src->finished()) break;
      uint64_t now = mono_ns(), la = sink.last_arrive_ns.load(std::memory_order_acquire);
      if (la && now > la && now - la > uint64_t(o.stall_s * 1e9)) {
        alarms.raise(kAlStall, "no frameset for " + std::to_string(int(o.stall_s)) + " s; restart");
        restart = true;
        break;
      }
      if (sink.ldp_restart_wanted.exchange(false)) {
        alarms.raise(kAlLdp, "LDP suspect longer than --ldp-restart-s; restart (--ldp-restart)");
        restart = true;
        break;
      }
      if (o.status_every_s > 0 && now - last_status > uint64_t(o.status_every_s * 1e9)) {
        double c = proc_cpu_s();
        uint64_t s = sink.sets.load();
        double dt = (now - last_status) / 1e9;
        std::printf("kv_cam: %.1f sets/s, %.3f cores, seq %llu, restarts %u, alarms %llu\n", (s - sets_last) / dt,
                    (c - cpu_last) / dt, (unsigned long long)h->latest_seq.load(), h->restarts.load(),
                    (unsigned long long)alarms.count);
        std::fflush(stdout);
        last_status = now;
        cpu_last = c;
        sets_last = s;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    src->stop();
    if (src->finished() && !restart) break;  // replay ran out
    if (g_stop.load()) break;
    h->restarts.fetch_add(1);
    if (!idle_for(ring, std::min(o.retry_gap_s, 2.0))) break;
  }
  src->stop();
  ring.set_state(kStopped);
  double wall = (mono_ns() - t_run0) / 1e9, cpu = proc_cpu_s() - cpu0;
  if (!o.stats_out.empty()) {
    FILE* f = std::fopen(o.stats_out.c_str(), "w");
    if (f) {
      std::fprintf(f,
                   "{\"published\": %llu, \"latest_seq\": %llu, \"epoch\": %u, \"tries\": %llu, \"restarts\": %u, "
                   "\"rate_events\": %u, \"near_zero_frames\": %llu, \"incomplete_sets\": %llu, \"alarms\": %llu, "
                   "\"wall_s\": %.3f, \"cpu_cores\": %.4f}\n",
                   (unsigned long long)sink.sets.load(), (unsigned long long)h->latest_seq.load(), h->writer_epoch,
                   (unsigned long long)tries, h->restarts.load(), h->rate_events.load(),
                   (unsigned long long)h->near_zero_frames.load(), (unsigned long long)h->incomplete_sets.load(),
                   (unsigned long long)alarms.count, wall, wall > 0 ? cpu / wall : 0.0);
      std::fclose(f);
    }
  }
  std::printf("kv_cam: stopped; published %llu sets, latest seq %llu, restarts %u, alarms %llu\n",
              (unsigned long long)sink.sets.load(), (unsigned long long)h->latest_seq.load(), h->restarts.load(),
              (unsigned long long)alarms.count);
  ring.heartbeat();
  return 0;
}

// ---------------- --selftest (no camera, no shared state touched except a private ring) ----------------
int selftest() {
  int fails = 0;
  auto check = [&](bool ok, const std::string& what) {
    std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what.c_str());
    if (!ok) ++fails;
  };
  std::printf("kv_cam selftest\n");
  // 1. ring round trip: 20 synth frames through a private segment, read back byte for byte
  {
    std::string name = "kv_cam_selftest_" + std::to_string(getpid());
    Geometry g;
    g.nslots = 4;
    RingWriter w;
    std::string err;
    bool ok = w.attach(name, g, &err);
    check(ok, "create private ring /" + name + (ok ? "" : ": " + err));
    if (ok) {
      RingReader r;
      check(r.open(name, &err), "reader opens the ring");
      std::vector<uint8_t> c(g.colour_bytes()), d(g.depth_bytes()), rc(c.size()), rd(d.size()), ec(c.size()), ed(d.size());
      int bad = 0, overwritten_ok = 0;
      for (uint64_t i = 0; i < 20; ++i) {
        SlotHeader* sh;
        uint8_t *cp, *dp;
        uint64_t s = w.begin(&sh, &cp, &dp);
        synth_fill(cp, c.size(), i, 0);
        synth_fill(dp, d.size(), i, 1);
        sh->colour_index = sh->depth_index = i;
        sh->flags = kHasColour | kHasDepth;
        w.publish();
        SlotHeader m;
        int st = r.read_copy(s, &m, rc.data(), rd.data());
        synth_fill(ec.data(), ec.size(), i, 0);
        synth_fill(ed.data(), ed.size(), i, 1);
        if (st != kOk || m.seq != s || rc != ec || rd != ed) ++bad;
        if (s > 4 && r.read_copy(s - 4, &m, rc.data(), rd.data()) == kOverwritten) ++overwritten_ok;
      }
      check(bad == 0, "20 frames written and read back byte for byte (" + std::to_string(bad) + " bad)");
      check(overwritten_ok == 16, "a sequence older than nslots reads as overwritten (" + std::to_string(overwritten_ok) + "/16)");
      check(r.read_copy(21, nullptr, nullptr, nullptr) == kNotYet, "a future sequence reads as not_yet");
      // positive control for the comparison itself: one flipped byte must be seen
      SlotHeader m;
      r.read_copy(20, &m, rc.data(), rd.data());
      synth_fill(ec.data(), ec.size(), 19, 0);
      rc[12345] ^= 1;
      check(rc != ec, "comparison sees a single flipped byte (positive control)");
      // label/content misalignment: frame 20's bytes compared with frame 18's expectation must differ
      synth_fill(ec.data(), ec.size(), 18, 0);
      r.read_copy(20, &m, rc.data(), rd.data());
      check(rc != ec, "content of frame 19 differs from frame 18 (misalignment is visible)");
      // seq label override (writer bug) is reported corrupt
      SlotHeader* sh;
      uint8_t *cp, *dp;
      uint64_t s = w.begin(&sh, &cp, &dp);
      w.publish(s + 1);
      check(r.read_copy(s, &m, rc.data(), rd.data()) == kCorrupt, "slot whose meta seq differs is reported corrupt");
      w.detach();
    }
    shm_unlink(name.c_str());
  }
  // 2. rate classifier on known intervals (depth_probe.py numbers)
  {
    std::vector<double> dn(90, 33.40), cn(90, 33.40), dh(93, 32.30), ch(46, 64.61);
    check(classify_rate(dn, cn, nullptr) == "normal", "normal 33.40/33.40 ms -> normal");
    check(classify_rate(dh, ch, nullptr) == "half", "half-rate 32.30/64.61 ms -> half");
    check(classify_rate(dn, ch, nullptr) == "half", "colour-only half (p25 64.61 ms) -> half");
    check(classify_rate(std::vector<double>(5, 33.4), cn, nullptr) == "insufficient", "5 depth intervals -> insufficient");
  }
  // 3. LDP sampler
  {
    std::vector<uint16_t> z(640 * 480, 0), full(640 * 480, 1500);
    check(depth_valid_permille(reinterpret_cast<uint8_t*>(z.data()), uint32_t(z.size() * 2)) == 0, "all-zero depth -> 0 permille");
    check(depth_valid_permille(reinterpret_cast<uint8_t*>(full.data()), uint32_t(full.size() * 2)) == 1000, "full depth -> 1000 permille");
  }
  std::printf("selftest: %s (%d failed)\n", fails ? "FAIL" : "PASS", fails);
  return fails ? 1 : 0;
}

void usage() {
  std::printf(
      "kv_cam [--source orbbec|replay|synth] [--pack FILE] [--name kv_cam_v1] [--path FILE --phys-base ADDR]\n"
      "       [--slots 8] [--fps 30] [--colour-format yuyv|rgb] [--align hw|none] [--no-sync] [--aggregate any|full]\n"
      "       [--start-wait-s 20] [--start-depth-frames 30] [--retry-gap-s 10] [--stall-s 3]\n"
      "       [--no-rate-check] [--rate-window-s 3] [--rate-max-restarts 5]\n"
      "       [--ldp-permille 20] [--ldp-frames 30] [--ldp-restart] [--ldp-restart-s 5] [--crc]\n"
      "       [--cpu N] [--nice N] [--alarm-log FILE] [--stats-out FILE] [--status-every-s 10] [--sdk-log DIR]\n"
      "       diagnosis (board perf arms): [--streams both|depth|colour] [--dump-props] [--set-prop ID:VAL]...\n"
      "       replay/synth: [--replay-fps 30] [--frames N] [--loops 1] [--fail-starts N]\n"
      "       tests only:   [--inject-misalign colour|depth|both:K]\n"
      "       [--selftest] [--list]\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto nx = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--source") o.source = nx();
    else if (a == "--pack") o.src.pack = nx();
    else if (a == "--name") o.name = nx();
    else if (a == "--path") o.path = nx();
    else if (a == "--phys-base") o.phys_base = std::stoull(nx(), nullptr, 0);
    else if (a == "--slots") o.nslots = uint32_t(std::stoul(nx()));
    else if (a == "--fps") o.src.fps = std::stoi(nx());
    else if (a == "--colour-format") o.src.colour_format = nx();
    else if (a == "--align") o.src.align = nx();
    else if (a == "--no-sync") o.src.sync = false;
    else if (a == "--aggregate") o.src.aggregate = nx();
    else if (a == "--sdk-log") o.src.sdk_log_dir = nx();
    else if (a == "--streams") o.src.streams = nx();
    else if (a == "--dump-props") o.src.dump_props = true;
    else if (a == "--set-prop") {
      std::string v = nx();
      auto p = v.find(':');
      if (p == std::string::npos) {
        std::fprintf(stderr, "--set-prop ID:VAL\n");
        return 2;
      }
      o.src.set_props.emplace_back(std::stoi(v.substr(0, p)), std::stoi(v.substr(p + 1)));
    }
    else if (a == "--start-wait-s") o.start_wait_s = std::stod(nx());
    else if (a == "--start-depth-frames") o.start_depth_frames = std::stoi(nx());
    else if (a == "--retry-gap-s") o.retry_gap_s = std::stod(nx());
    else if (a == "--stall-s") o.stall_s = std::stod(nx());
    else if (a == "--no-rate-check") o.rate_check = false;
    else if (a == "--rate-window-s") o.rate_window_s = std::stod(nx());
    else if (a == "--rate-max-restarts") o.rate_max_restarts = std::stoi(nx());
    else if (a == "--ldp-permille") o.ldp_permille = std::stoi(nx());
    else if (a == "--ldp-frames") o.ldp_frames = std::stoi(nx());
    else if (a == "--ldp-restart") o.ldp_restart = true;
    else if (a == "--ldp-restart-s") o.ldp_restart_s = std::stod(nx());
    else if (a == "--crc") o.crc = true;
    else if (a == "--cpu") o.cpu = std::stoi(nx());
    else if (a == "--nice") o.nice_v = std::stoi(nx());
    else if (a == "--alarm-log") o.alarm_log = nx();
    else if (a == "--stats-out") o.stats_out = nx();
    else if (a == "--status-every-s") o.status_every_s = std::stod(nx());
    else if (a == "--replay-fps") o.src.replay_fps = std::stoi(nx());
    else if (a == "--frames") o.src.frames = std::stol(nx());
    else if (a == "--loops") o.src.loops = std::stoi(nx());
    else if (a == "--fail-starts") o.src.fail_starts = std::stoi(nx());
    else if (a == "--inject-misalign") {
      std::string v = nx();
      auto p = v.find(':');
      o.src.inject_stream = v.substr(0, p);
      o.src.inject_from = p == std::string::npos ? 0 : std::stol(v.substr(p + 1));
    } else if (a == "--selftest") o.selftest = true;
    else if (a == "--list") o.list = true;
    else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "unknown arg %s\n", a.c_str());
      usage();
      return 2;
    }
  }
  if (o.selftest) return selftest();
  if (o.list) return orbbec_list(o.src);
  if (o.source != "orbbec" && o.source != "replay" && o.source != "synth") {
    std::fprintf(stderr, "--source orbbec|replay|synth\n");
    return 2;
  }
  if (o.source == "replay" && o.src.pack.empty()) {
    std::fprintf(stderr, "--source replay needs --pack FILE\n");
    return 2;
  }
  if (!o.src.inject_stream.empty() && o.source == "orbbec") {
    std::fprintf(stderr, "--inject-misalign is for replay/synth only\n");
    return 2;
  }
  if (o.src.streams != "both") {  // diagnosis arms: no pair to rate-check; colour-only has no depth to wait for
    o.rate_check = false;
    if (o.src.streams == "colour") o.start_depth_frames = 0;
  }
  if (o.cpu >= 0) {  // before any SDK thread exists, so every thread inherits it
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(o.cpu, &set);
    if (sched_setaffinity(0, sizeof set, &set) != 0) std::perror("kv_cam: sched_setaffinity");
  }
  if (o.nice_v != 0 && setpriority(PRIO_PROCESS, 0, o.nice_v) != 0) std::perror("kv_cam: setpriority");
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  return run(o);
}
