// NN-CAM2: dpu_rx local frame source (see local_src.h).
#include "local_src.h"

#include <unistd.h>

#include <cstdio>
#include <sstream>

#include "../kv_cam/cam_ring.h"

namespace dpupipe {

struct LocalColourSource::ReaderBox {
  kvcam::RingReader r;
  bool open = false;
};

LocalColourSource::LocalColourSource(const LocalConfig& cfg, FrameRing* ring, const Geometry& g)
    : cfg_(cfg), ring_(ring), geo_(g), rd_(new ReaderBox) {
  if (cfg_.prep == "pl") {
    std::string why;
    prep_ = make_pl_prep(g.src_w, g.src_h, g.size, cfg_.fix_point, cfg_.kset, cfg_.ups, &why);
    if (!prep_) {
      prep_note_ = "prep pl unavailable (" + why + "): soft";
      std::fprintf(stderr, "{\"warning\":\"dpu_rx local: %s\"}\n", prep_note_.c_str());
    }
  } else if (cfg_.prep != "soft") {
    prep_note_ = "unknown --prep " + cfg_.prep + ": soft";
    std::fprintf(stderr, "{\"warning\":\"dpu_rx local: %s\"}\n", prep_note_.c_str());
  }
  if (!prep_) {
    auto* s = new SoftPrep(g.src_w, g.src_h, g.size, g.w, g.h, g.top, g.left, cfg_.fix_point, cfg_.kset, cfg_.ups);
    if (cfg_.perturb) s->perturb();
    if (!s->ok())
      std::fprintf(stderr, "{\"warning\":\"dpu_rx local: soft prep cannot stream this geometry; frames dropped\"}\n");
    prep_.reset(s);
  }
}

LocalColourSource::~LocalColourSource() { stop(); }

void LocalColourSource::start() {
  stop_ = false;
  th_ = std::thread([this] { run(); });
}

void LocalColourSource::stop() {
  stop_ = true;
  if (th_.joinable()) th_.join();
}

void LocalColourSource::warn(const std::string& msg) {
  uint64_t now = mono_ns();
  if (now - last_warn_ns_ < 5000000000ull) return;
  last_warn_ns_ = now;
  std::fprintf(stderr, "{\"warning\":\"dpu_rx local: %s\"}\n", msg.c_str());
}

bool LocalColourSource::ensure_open() {
  auto& b = *rd_;
  if (b.open && b.r.stale(uint64_t(cfg_.stale_reopen_ms) * 1000000ull)) {
    ++counts_.reopen_stale;
    warn("kv_cam ring heartbeat stale; reopening");
    b.r.close();
    b.open = false;
  }
  if (b.open) return true;
  std::string err;
  if (!b.r.open(cfg_.ring_name, &err, cfg_.ring_path)) {
    ++counts_.open_fail;
    warn("cannot open kv_cam ring (" + err + "); retrying");
    return false;
  }
  const auto* h = b.r.header();
  if (h->colour_fourcc != kvcam::kFmtYUYV || int(h->colour_w) != geo_.src_w || int(h->colour_h) != geo_.src_h) {
    ++counts_.geometry_bad;
    char m[160];
    std::snprintf(m, sizeof m, "kv_cam ring colour %ux%u fourcc %#x, expected YUYV %dx%d; retrying", h->colour_w,
                  h->colour_h, h->colour_fourcc, geo_.src_w, geo_.src_h);
    warn(m);
    b.r.close();
    return false;
  }
  b.open = true;
  ++counts_.opens;
  return true;
}

void LocalColourSource::run() {
  uint64_t last_seq = 0;
  while (!stop_) {
    if (!ensure_open()) {
      usleep(200000);
      continue;
    }
    if (cfg_.max_inflight ? !ring_->wait_inflight_below(cfg_.max_inflight, 200)
                          : !ring_->wait_ready_below(cfg_.max_ahead, 200))
      continue;
    int slot = ring_->acquire_free(200);
    if (slot < 0) continue;
    bool published = false;
    while (!stop_ && !published) {
      auto& r = rd_->r;
      uint64_t seq = r.wait_newer(last_seq, 200);
      if (seq <= last_seq) {
        if (!ensure_open()) break;  // stale writer: reopen, keep the slot
        continue;
      }
      uint64_t c0 = thread_cpu_ns();
      uint64_t t_read = mono_ns();
      if (last_seq && seq > last_seq + 1) counts_.skipped += seq - last_seq - 1;
      kvcam::SlotHeader meta;
      int st = r.read_copy(seq, &meta, nullptr, nullptr);  // header + validity only; colour is read in place
      last_seq = seq;
      if (st != kvcam::kOk) {
        ++(st == kvcam::kNotYet ? counts_.not_yet : counts_.overwritten);
        continue;
      }
      if (!(meta.flags & kvcam::kHasColour) || meta.colour_bytes == 0) {
        ++counts_.no_colour;
        continue;
      }
      const auto* h = r.header();
      PrepInput in;
      in.data = r.colour_ptr(seq);
      in.phys = h->phys_base ? h->phys_base + h->slots_offset + ((seq - 1) % h->nslots) * h->slot_stride +
                                   h->colour_off
                             : 0;
      in.w = int(h->colour_w);
      in.h = int(h->colour_h);
      in.stride = int(h->colour_stride);
      in.fourcc = h->colour_fourcc;
      bool ok = prep_->run(in, ring_->data(slot), geo_.tensor_bytes());
      if (!r.still_valid(seq)) {  // overwritten while we read it in place: discard, take the next newest
        ++counts_.overwritten;
        cpu_ns_ += thread_cpu_ns() - c0;
        continue;
      }
      if (!ok) {
        ++counts_.prep_fail;
        cpu_ns_ += thread_cpu_ns() - c0;
        warn("preprocessing failed for a frame");
        continue;
      }
      FrameMeta m;
      m.t_prep_ns = mono_ns();
      m.source_key = meta.colour_index;
      m.seq = uint32_t(seq);
      m.t_want_ns = m.t_first_ns = t_read;  // the UDP path's "want" / "first chunk" = "frame seen" here
      m.t_done_ns = m.t_prep_ns;
      m.size = uint16_t(geo_.size);
      m.w = uint16_t(geo_.w);
      m.h = uint16_t(geo_.h);
      m.top = uint16_t(geo_.top);
      m.left = uint16_t(geo_.left);
      m.fix_point = uint8_t(cfg_.fix_point);
      m.pad_q = quant_pad(cfg_.fix_point);
      m.cam_index = meta.colour_index;
      m.cam_dev_ts_us = meta.colour_dev_ts_us;
      m.cam_sys_ts_us = meta.colour_sys_ts_us;
      m.cam_arrive_ns = meta.arrive_ns;
      m.cam_publish_ns = meta.publish_ns;
      m.ring_seq = seq;
      m.t_read_ns = t_read;
      uint64_t dc = thread_cpu_ns() - c0;
      m.rx_cpu_ns = dc;
      cpu_ns_ += dc;
      ++counts_.frames;
      ring_->publish(slot, m);
      published = true;
    }
    if (!published) ring_->abort(slot);
  }
}

std::string LocalColourSource::describe_json() const {
  std::ostringstream o;
  o << "{\"source\":\"kvcam\",\"ring\":\"" << (cfg_.ring_path.empty() ? cfg_.ring_name : cfg_.ring_path)
    << "\",\"prep\":" << prep_->describe_json() << ",\"prep_note\":\"" << prep_note_
    << "\",\"frames\":" << counts_.frames << ",\"skipped\":" << counts_.skipped
    << ",\"overwritten\":" << counts_.overwritten << ",\"not_yet\":" << counts_.not_yet
    << ",\"prep_fail\":" << counts_.prep_fail << ",\"no_colour\":" << counts_.no_colour
    << ",\"opens\":" << counts_.opens << ",\"open_fail\":" << counts_.open_fail
    << ",\"reopen_stale\":" << counts_.reopen_stale << ",\"geometry_bad\":" << counts_.geometry_bad
    << ",\"cpu_ms\":" << cpu_ns_.load() / 1e6 << "}";
  return o.str();
}

}  // namespace dpupipe
