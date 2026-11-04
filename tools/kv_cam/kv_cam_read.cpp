// kv_cam_read (NN-CAM1, 2026-11-04): reference reader of the kv_cam ring and the protocol verifier.
//   kv_cam_read [--name kv_cam_v1] [--frames N] [--timeout-s 30] [--mode all|newest] [--from-latest]
//               [--verify none|synth|pack:FILE] [--mid-copy-delay-ms D [--jitter]] [--no-seqlock] [--json FILE]
//               [--info]
// --verify: every delivered frame's colour bytes must equal the source frame meta.colour_index and its depth bytes
//   the source frame meta.depth_index (memcmp against the pack file, or the regenerated synthetic frame).
// --mid-copy-delay-ms D (--jitter: uniform in [0, D)) / --no-seqlock: test knobs (a reader slowed between the colour
//   and the depth copy; skipping the seqlock re-check = the negative control).
// Exit 0 = no content mismatch and no corrupt slot; 1 = mismatch/corrupt seen; 2 = usage / could not open.
#include "kv_cam_common.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

using namespace kvcam;

int main(int argc, char** argv) {
  std::string name = kDefaultName, verify = "none", json, mode = "all";
  long frames = 0;
  double timeout_s = 30, delay_ms = 0;
  bool seqlock = true, from_latest = false, info = false, jitter = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto nx = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--name") name = nx();
    else if (a == "--frames") frames = std::stol(nx());
    else if (a == "--timeout-s") timeout_s = std::stod(nx());
    else if (a == "--mode") mode = nx();
    else if (a == "--from-latest") from_latest = true;
    else if (a == "--verify") verify = nx();
    else if (a == "--mid-copy-delay-ms") delay_ms = std::stod(nx());
    else if (a == "--no-seqlock") seqlock = false;
    else if (a == "--jitter") jitter = true;
    else if (a == "--json") json = nx();
    else if (a == "--info") info = true;
    else {
      std::fprintf(stderr, "unknown arg %s\n", a.c_str());
      return 2;
    }
  }
  Pack pack;
  std::string err;
  if (verify.rfind("pack:", 0) == 0 && !pack.open(verify.substr(5), &err)) {
    std::fprintf(stderr, "kv_cam_read: %s\n", err.c_str());
    return 2;
  }
  RingReader r;
  uint64_t t_end = mono_ns() + uint64_t(timeout_s * 1e9);
  while (!r.open(name, &err)) {  // the writer may not have created the ring yet
    if (mono_ns() > t_end) {
      std::fprintf(stderr, "kv_cam_read: cannot open /%s: %s\n", name.c_str(), err.c_str());
      return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const RingHeader* h = r.header();
  if (info) {
    std::printf("ring /%s v%u: %u slots x %llu B; colour %ux%u fourcc %08x %u B @%u; depth %ux%u %u B @%u; epoch %u "
                "pid %u; latest %llu; state %u; restarts %u; alarms %u (%s); info '%s'; intr %d fx %.2f fy %.2f "
                "cx %.2f cy %.2f\n",
                name.c_str(), h->version, h->nslots, (unsigned long long)h->slot_stride, h->colour_w, h->colour_h,
                h->colour_fourcc, h->colour_bytes, h->colour_off, h->depth_w, h->depth_h, h->depth_bytes, h->depth_off,
                h->writer_epoch, h->writer_pid, (unsigned long long)r.latest(), h->state.load(), h->restarts.load(),
                h->alarm_count.load(), h->alarm_text, h->writer_info, h->intr_valid, h->fx, h->fy, h->cx, h->cy);
    if (frames == 0) return 0;
  }
  if (pack.n() && (pack.header().colour_bytes != h->colour_bytes || pack.header().depth_bytes != h->depth_bytes)) {
    std::fprintf(stderr, "kv_cam_read: pack geometry differs from the ring\n");
    return 2;
  }
  std::vector<uint8_t> col(h->colour_bytes), dep(h->depth_bytes), ec(h->colour_bytes), ed(h->depth_bytes);
  uint64_t ok = 0, overwritten = 0, corrupt = 0, mism_c = 0, mism_d = 0, verified_bytes = 0, skipped = 0;
  uint64_t first_seq = 0, last_seq = 0, nonmono = 0, idle_timeouts = 0;
  std::string first_bad;
  uint64_t next = from_latest ? r.latest() + 1 : 1;
  uint64_t rng = 0x243F6A8885A308D3ull;
  auto hook = [&] {
    if (delay_ms <= 0) return;
    double d = delay_ms;
    if (jitter) d *= double((rng = splitmix64(rng)) >> 11) / 9007199254740992.0;  // uniform [0, D)
    std::this_thread::sleep_for(std::chrono::microseconds(long(d * 1000)));
  };
  uint64_t deadline_idle = mono_ns() + uint64_t(timeout_s * 1e9);
  while (frames == 0 || long(ok + overwritten + corrupt) < frames) {
    uint64_t l = r.wait_newer(next - 1, 500);
    if (l < next) {
      if (mono_ns() > deadline_idle) break;  // writer went quiet
      ++idle_timeouts;
      continue;
    }
    deadline_idle = mono_ns() + uint64_t(timeout_s * 1e9);
    uint64_t seq = mode == "newest" ? l : next;
    if (mode == "newest" && seq > next) skipped += seq - next;
    SlotHeader m;
    int st = r.read_copy(seq, &m, col.data(), dep.data(), hook, seqlock);
    if (st == kNotYet) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (st == kOverwritten) {
      ++overwritten;
      uint64_t lat = r.latest(), oldest = lat >= h->nslots ? lat - h->nslots + 2 : 1;  // +2: leave the writer a slot
      next = std::max(seq + 1, oldest);
      continue;
    }
    if (st == kCorrupt) {
      ++corrupt;
      if (first_bad.empty()) first_bad = "seq " + std::to_string(seq) + " corrupt (meta seq " + std::to_string(m.seq) + ")";
      next = seq + 1;
      continue;
    }
    ++ok;
    if (!first_seq) first_seq = seq;
    if (last_seq && seq <= last_seq) ++nonmono;
    last_seq = seq;
    next = seq + 1;
    if (verify != "none") {
      bool bc = false, bd = false;
      if (m.flags & kHasColour) {
        const uint8_t* exp;
        if (pack.n()) exp = pack.colour(uint32_t(m.colour_index % pack.n()));
        else {
          synth_fill(ec.data(), ec.size(), m.colour_index, 0);
          exp = ec.data();
        }
        bc = std::memcmp(col.data(), exp, col.size()) != 0;
        verified_bytes += col.size();
      }
      if (m.flags & kHasDepth) {
        const uint8_t* exp;
        if (pack.n()) exp = pack.depth(uint32_t(m.depth_index % pack.n()));
        else {
          synth_fill(ed.data(), ed.size(), m.depth_index, 1);
          exp = ed.data();
        }
        bd = std::memcmp(dep.data(), exp, dep.size()) != 0;
        verified_bytes += dep.size();
      }
      mism_c += bc;
      mism_d += bd;
      if ((bc || bd) && first_bad.empty())
        first_bad = "seq " + std::to_string(seq) + " colour_index " + std::to_string(m.colour_index) + (bc ? " COLOUR" : "") +
                    " depth_index " + std::to_string(m.depth_index) + (bd ? " DEPTH" : "") + " bytes differ";
      if ((m.flags & kCrcValid) && (crc32(col.data(), col.size()) != m.crc_colour || crc32(dep.data(), dep.size()) != m.crc_depth)) {
        ++corrupt;
        if (first_bad.empty()) first_bad = "seq " + std::to_string(seq) + " crc mismatch";
      }
    }
  }
  bool bad = mism_c || mism_d || corrupt || nonmono;
  char buf[1024];
  std::snprintf(buf, sizeof buf,
                "{\"name\": \"%s\", \"mode\": \"%s\", \"verify\": \"%s\", \"seqlock\": %s, \"mid_copy_delay_ms\": %.3f, "
                "\"ok\": %llu, \"overwritten\": %llu, \"corrupt\": %llu, \"mismatch_colour\": %llu, "
                "\"mismatch_depth\": %llu, \"nonmonotonic\": %llu, \"skipped_newest\": %llu, \"verified_bytes\": %llu, "
                "\"first_seq\": %llu, \"last_seq\": %llu, \"epoch\": %u, \"first_bad\": \"%s\", \"pass\": %s}",
                name.c_str(), mode.c_str(), verify.c_str(), seqlock ? "true" : "false", delay_ms,
                (unsigned long long)ok, (unsigned long long)overwritten, (unsigned long long)corrupt,
                (unsigned long long)mism_c, (unsigned long long)mism_d, (unsigned long long)nonmono,
                (unsigned long long)skipped, (unsigned long long)verified_bytes, (unsigned long long)first_seq,
                (unsigned long long)last_seq, h->writer_epoch, first_bad.c_str(), bad ? "false" : "true");
  std::printf("%s\n", buf);
  if (!json.empty()) {
    FILE* f = std::fopen(json.c_str(), "w");
    if (f) {
      std::fprintf(f, "%s\n", buf);
      std::fclose(f);
    }
  }
  return bad ? 1 : 0;
}
