// DPUPIPE0 (2026-11-03): dpu_rx — KV260 DPU input receiver + inference loop. One process: receiver thread
// (dpu_rx.cpp) -> FrameRing -> inference workers (infer_engine.h).
// DPUPIPE1 (2026-11-03): N worker threads, each with its own engine (stub, or a VART runner when built with
// HAVE_VART), CPU output processing (det_post.cpp), per-frame result log ("boxes out"), steady-window stats.
//
//   dpu_rx --port 47900 [--source NANO_IP:PORT] --size 640 --fix 6 [--src-wh 640x480] [--chunk 60000]
//          [--slots 3] [--ahead 1] [--inflight 0] [--wait-ms 200] [--seconds 60] [--warm-s 0]
//          [--engine stub|vart] [--infer-ms 100] [--xmodel M.xmodel] [--workers 1]
//          [--log rx.jsonl] [--infer-log infer.jsonl] [--out-log boxes.jsonl] [--crc] [--tensor-crc]
//   --crc: check the content crc32 from the FRAME_DPU head on the A53 (default off: costs CPU per frame)
//   --tensor-crc: crc32 of the whole staged input tensor (pad included) into --infer-log, so a test can compare it
//                 with host/dpu_prep.py's full tensor of the same camera frame
//   --inflight N: pull a frame only while READY + frames inside the workers < N (0: the DPUPIPE0 rule, READY < --ahead).
//                 With W workers, N = W keeps the DPU queue full without parking an extra stale frame in READY.
//   --warm-s: the "steady" statistics (out_fps_steady, CPU) only count from this many seconds after start
//
// Prints one JSON summary line at the end. Per-frame result line (--out-log), all times KV CLOCK_MONOTONIC ns:
//   seq source_key worker t_want t_first t_done(rx complete) t_acq t_staged t_infer t_out ndet cand post_us dets
// NN-CAM2 (2026-11-03): --frame-source kvcam (default udp = exactly as before): frames come from kv_cam's shared-memory
//   ring on this board (local_src.h), YUYV preprocessed here (--prep soft: colour_prep.h SoftPrep; --prep pl: the
//   PL core's driver when merged -- not yet: warning, soft). [--kvcam-name kv_cam_v1] [--kvcam-path FILE]
//   [--prep-kset 0|1] [--prep-ups 0|1]. --stamp-log FILE: one JSONL row per frame with the camera stamps (own
//   columns) and the CLOCK_MONOTONIC instants read -> prep -> DPU -> post (tools/nn_cam2/chain_report.py).
//   --dump-tensors DIR (tests): the exact tensor handed to the DPU stage, DIR/<frame key>.bin (key: kvcam = camera
//   frame index, udp = the Nano's source_key). --prep-perturb (positive control): SoftPrep coefficient +1.
// RECOG640 (2026-11-04): output processing options (det_post.h PostConfig; all absent = DPUPIPE1 behaviour):
//   [--conf 0.25] [--iou 0.7] [--max-det 0] [--cls-thr t0,t1,...] [--sort-heads] [--nc N] [--model-tag NAME]
//   class count must equal N (and --cls-thr must have N values), checked on the VART output heads at start.
//   tools/dpu_pipe/recog/recog_args.py turns RECOG_MODEL=y8s640|y8s512 into these flags (+ --size/--fix/--xmodel).
// RECOG641 (2026-11-04): [--post pl|cpu] (default pl): output processing on the PL detection post-processing core
//   (pl_post.h, same PostConfig, zero copy from the VART output buffers) or on the A53 (det_post.cpp). pl needs
//   --engine vart and a bitstream with the core (sysinfo SLOT_MAP bit 10); when the core cannot be opened, an engine's
//   heads do not fit it, or a frame fails, that engine / frame runs det_post.cpp (warning, never refuses).
//   Summary: "post_impl" {requested, active, why, pl_frames, cpu_frames, core}; post_us = whole output processing.
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dpu_rx.h"
#include "infer_engine.h"
#include "local_src.h"

using namespace dpupipe;

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

// all-CPU softirq time (jiffies, /proc/stat "cpu" line, 7th value): the kernel receive path (IP fragment
// reassembly, UDP queueing) runs there, not in the receiver thread
struct CpuStat {
  long long v[10] = {0};
  bool ok = false;
  long long total() const {
    long long s = 0;
    for (int i = 0; i < 8; ++i) s += v[i];  // user nice system idle iowait irq softirq steal
    return s;
  }
  long long idle() const { return v[3] + v[4]; }
};
static CpuStat read_stat() {
  CpuStat c;
  FILE* f = fopen("/proc/stat", "r");
  if (!f) return c;
  int n = fscanf(f, "cpu %lld %lld %lld %lld %lld %lld %lld %lld", &c.v[0], &c.v[1], &c.v[2], &c.v[3], &c.v[4],
                 &c.v[5], &c.v[6], &c.v[7]);
  fclose(f);
  c.ok = n == 8;
  return c;
}
static double proc_cpu_ms() {
  rusage ru{};
  getrusage(RUSAGE_SELF, &ru);
  auto tv = [](const timeval& t) { return t.tv_sec * 1e3 + t.tv_usec / 1e3; };
  return tv(ru.ru_utime) + tv(ru.ru_stime);
}

static double pct(std::vector<double> v, double q) {
  if (v.empty()) return -1;
  std::sort(v.begin(), v.end());
  size_t i = static_cast<size_t>(q * (v.size() - 1) + 0.5);
  return v[std::min(i, v.size() - 1)];
}

struct Rec {
  FrameMeta m;
  int worker;
  uint64_t t_acq, t_staged, t_infer, t_out;
  size_t ndet, cand;
  double post_us;
};

int main(int argc, char** argv) {
  RxConfig cfg;
  size_t slots = 0, workers = 1;
  double infer_ms = 100, seconds = 60, warm_s = 0;
  bool tensor_crc = false;
  std::string infer_log, out_log, engine = "stub", xmodel;
  std::string frame_source = "udp", stamp_log, dump_dir;
  LocalConfig lcfg;
  PostConfig post;
  int expect_nc = 0;
  std::string model_tag, post_impl = "pl";
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        fprintf(stderr, "missing value for %s\n", a.c_str());
        exit(2);
      }
      return argv[++i];
    };
    if (a == "--port") cfg.bind_port = static_cast<uint16_t>(std::stoi(next()));
    else if (a == "--source") cfg.source = next();
    else if (a == "--size") cfg.size = std::stoi(next());
    else if (a == "--fix") cfg.fix_point = std::stoi(next());
    else if (a == "--src-wh") {
      std::string s = next();
      auto x = s.find('x');
      cfg.src_w = std::stoi(s.substr(0, x));
      cfg.src_h = std::stoi(s.substr(x + 1));
    } else if (a == "--chunk") cfg.chunk_bytes = static_cast<uint32_t>(std::stoul(next()));
    else if (a == "--wait-ms") cfg.wait_ms = static_cast<uint32_t>(std::stoul(next()));
    else if (a == "--frame-timeout-ms") cfg.frame_timeout_ms = std::stoi(next());
    else if (a == "--slots") slots = std::stoul(next());
    else if (a == "--ahead") cfg.max_ahead = std::stoul(next());
    else if (a == "--inflight") cfg.max_inflight = std::stoul(next());
    else if (a == "--infer-ms") infer_ms = std::stod(next());
    else if (a == "--seconds") seconds = std::stod(next());
    else if (a == "--warm-s") warm_s = std::stod(next());
    else if (a == "--log") cfg.log_path = next();
    else if (a == "--infer-log") infer_log = next();
    else if (a == "--out-log") out_log = next();
    else if (a == "--rcvbuf") cfg.rcvbuf = std::stoi(next());
    else if (a == "--engine") engine = next();
    else if (a == "--xmodel") xmodel = next();
    else if (a == "--workers") workers = std::max<size_t>(1, std::stoul(next()));
    else if (a == "--no-crc") cfg.verify_crc = false;
    else if (a == "--crc") cfg.verify_crc = true;
    else if (a == "--tensor-crc") tensor_crc = true;
    else if (a == "--frame-source") frame_source = next();
    else if (a == "--kvcam-name") lcfg.ring_name = next();
    else if (a == "--kvcam-path") lcfg.ring_path = next();
    else if (a == "--prep") lcfg.prep = next();
    else if (a == "--prep-kset") lcfg.kset = std::stoi(next());
    else if (a == "--prep-ups") lcfg.ups = std::stoi(next());
    else if (a == "--prep-perturb") lcfg.perturb = true;
    else if (a == "--stamp-log") stamp_log = next();
    else if (a == "--dump-tensors") dump_dir = next();
    else if (a == "--conf") post.conf = std::stof(next());
    else if (a == "--iou") post.iou = std::stof(next());
    else if (a == "--max-det") post.max_det = std::stoi(next());
    else if (a == "--sort-heads") post.sort_heads = true;
    else if (a == "--nc") expect_nc = std::stoi(next());
    else if (a == "--model-tag") model_tag = next();
    else if (a == "--post") post_impl = next();
    else if (a == "--cls-thr") {
      std::string v = next();
      post.cls_thr.clear();
      size_t p0 = 0;
      while (p0 <= v.size()) {
        size_t p1 = v.find(',', p0);
        if (p1 == std::string::npos) p1 = v.size();
        if (p1 > p0) post.cls_thr.push_back(std::stof(v.substr(p0, p1 - p0)));
        p0 = p1 + 1;
      }
    } else {
      fprintf(stderr, "unknown argument %s\n", a.c_str());
      return 2;
    }
  }
  if (slots == 0) slots = std::max<size_t>(3, workers + 2);
  if (expect_nc > 0 && !post.cls_thr.empty() && post.cls_thr.size() != static_cast<size_t>(expect_nc)) {
    fprintf(stderr, "--cls-thr has %zu values, --nc %d\n", post.cls_thr.size(), expect_nc);
    return 2;
  }
  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);
  Geometry g = letterbox(cfg.src_w, cfg.src_h, cfg.size);

  // engines first (VART runner creation loads the xmodel: seconds, must not count as pipeline time)
  std::vector<std::unique_ptr<Engine>> eng;
  if (engine == "stub") {
    for (size_t k = 0; k < workers; ++k) eng.emplace_back(new StubEngine(infer_ms, tensor_crc));
  } else if (engine == "vart") {
#ifdef HAVE_VART
    try {
      auto model = VartEngine::load_model(xmodel);
      for (size_t k = 0; k < workers; ++k) {
        auto* e = new VartEngine(model, tensor_crc);
        eng.emplace_back(e);
        if (e->input_size() != cfg.size || e->input_fix_point() != cfg.fix_point || e->input_bytes() != g.tensor_bytes()) {
          fprintf(stderr, "xmodel input %d fix %d does not match --size %d --fix %d\n", e->input_size(),
                  e->input_fix_point(), cfg.size, cfg.fix_point);
          return 2;
        }
        for (const auto& h : e->heads()) {
          const int nc = h.c - 64;
          if ((expect_nc > 0 && nc != expect_nc) || (!post.cls_thr.empty() && static_cast<size_t>(nc) != post.cls_thr.size())) {
            fprintf(stderr, "xmodel head %dx%dx%d: %d classes, expected --nc %d / --cls-thr %zu values\n", h.h, h.w,
                    h.c, nc, expect_nc, post.cls_thr.size());
            return 2;
          }
        }
      }
    } catch (const std::exception& ex) {
      fprintf(stderr, "VART: %s\n", ex.what());
      return 3;
    }
#else
    fprintf(stderr, "built without HAVE_VART\n");
    return 2;
#endif
  } else {
    fprintf(stderr, "unknown engine %s\n", engine.c_str());
    return 2;
  }
  for (auto& e : eng) e->set_post(post);
  // RECOG641: PL output processing core
  std::shared_ptr<PlPost> plpost;
  std::string post_why;
  if (post_impl != "pl" && post_impl != "cpu") {
    post_why = "unknown --post " + post_impl + ": cpu";
    post_impl = "cpu";
  } else if (post_impl == "pl" && engine != "vart") {
    post_why = "--post pl needs --engine vart (stub has no DPU output): cpu";
  }
#ifdef HAVE_VART
  if (post_impl == "pl" && engine == "vart" && !eng.empty()) {
    auto* v0 = static_cast<VartEngine*>(eng[0].get());
    const int nc = v0->heads().empty() ? 0 : v0->heads()[0].c - 64;
    const int fp = v0->heads().empty() ? 0 : v0->heads()[0].fix_point;
    plpost = PlPost::make(nullptr, nc, fp, post, &post_why);
    if (plpost) {
      for (size_t k = 0; k < eng.size(); ++k) {
        std::string w;
        if (!eng[k]->attach_pl_post(plpost, &w)) post_why += (post_why.empty() ? "" : "; ") + ("worker " + std::to_string(k) + ": " + w + ": cpu");
      }
    }
  }
#endif
  if (!post_why.empty()) {
    for (auto& c : post_why) if (c == '"') c = '\'';
    fprintf(stderr, "{\"warning\":\"dpu_rx post: %s\"}\n", post_why.c_str());
  }

  FrameRing ring(slots, g.tensor_bytes(), quant_pad(cfg.fix_point));
  if (frame_source != "udp" && frame_source != "kvcam") {
    fprintf(stderr, "{\"warning\":\"unknown --frame-source %s: udp\"}\n", frame_source.c_str());
    frame_source = "udp";
  }
  std::unique_ptr<DpuRx> rx;
  std::unique_ptr<LocalColourSource> lsrc;
  if (frame_source == "kvcam") {
    lcfg.size = cfg.size;
    lcfg.fix_point = cfg.fix_point;
    lcfg.max_ahead = cfg.max_ahead;
    lcfg.max_inflight = cfg.max_inflight;
    lsrc.reset(new LocalColourSource(lcfg, &ring, g));
  } else {
    rx.reset(new DpuRx(cfg, &ring));
  }
  FILE* slog = stamp_log.empty() ? nullptr : fopen(stamp_log.c_str(), "w");
  FILE* ilog = infer_log.empty() ? nullptr : fopen(infer_log.c_str(), "w");
  FILE* olog = out_log.empty() ? nullptr : fopen(out_log.c_str(), "w");
  std::mutex rec_mu;
  std::vector<Rec> recs;
  uint64_t engine_errors = 0;

  double cpu0 = proc_cpu_ms();
  CpuStat st0 = read_stat();
  const uint64_t t_start = mono_ns();
  const uint64_t t_warm = t_start + static_cast<uint64_t>(warm_s * 1e9);
  const uint64_t t_stop = t_start + static_cast<uint64_t>(seconds * 1e9);
  if (rx) rx->start();
  if (lsrc) lsrc->start();
  std::vector<std::thread> th;
  for (size_t k = 0; k < workers; ++k)
    th.emplace_back([&, k] {
      Engine& e = *eng[k];
      while (!g_stop && mono_ns() < t_stop) {
        FrameMeta m;
        int s = ring.acquire_ready(100, &m);
        if (s < 0) continue;
        Rec r{};
        r.m = m;
        r.worker = static_cast<int>(k);
        r.t_acq = mono_ns();
        if (!dump_dir.empty()) {  // tests: the exact bytes handed to the DPU stage
          std::string fn = dump_dir + "/" + std::to_string(m.source_key) + ".bin";
          if (FILE* df = fopen(fn.c_str(), "wb")) {
            fwrite(ring.data(s), 1, g.tensor_bytes(), df);
            fclose(df);
          }
        }
        bool ok = e.stage(ring.data(s), g.tensor_bytes());
        uint32_t crc = e.staged_crc();
        ring.release(s);
        r.t_staged = mono_ns();
        ok = ok && e.infer();
        r.t_infer = mono_ns();
        PostTiming pt;
        std::vector<Det> d;
        if (ok) d = e.post(&pt);
        r.t_out = mono_ns();
        ring.finish();
        r.ndet = d.size();
        r.cand = pt.candidates;
        r.post_us = pt.fix_decode_us + pt.nms_us + pt.pl_us;
        std::lock_guard<std::mutex> lk(rec_mu);
        if (slog) {  // NN-CAM2 chain stamps: camera stamps in their own columns, the rest CLOCK_MONOTONIC ns
          fprintf(slog,
                  "{\"chain\":\"colour\",\"src\":\"%s\",\"frame\":%llu,\"ring_seq\":%llu,\"cam_dev_ts_us\":%llu,"
                  "\"cam_sys_ts_us\":%llu,\"t_cam_arrive_ns\":%llu,\"t_cam_publish_ns\":%llu,\"t_read_ns\":%llu,"
                  "\"t_prep_ns\":%llu,\"t_acq_ns\":%llu,\"t_dpu_start_ns\":%llu,\"t_dpu_ns\":%llu,\"t_post_ns\":%llu,"
                  "\"ok\":%d,\"ndet\":%zu,\"worker\":%zu,\"prep_cpu_us\":%.1f}\n",
                  frame_source.c_str(), (unsigned long long)m.source_key, (unsigned long long)m.ring_seq,
                  (unsigned long long)m.cam_dev_ts_us, (unsigned long long)m.cam_sys_ts_us,
                  (unsigned long long)m.cam_arrive_ns, (unsigned long long)m.cam_publish_ns,
                  (unsigned long long)m.t_read_ns, (unsigned long long)(m.t_prep_ns ? m.t_prep_ns : m.t_done_ns),
                  (unsigned long long)r.t_acq, (unsigned long long)r.t_staged, (unsigned long long)r.t_infer,
                  (unsigned long long)r.t_out, ok ? 1 : 0, r.ndet, k, m.rx_cpu_ns / 1e3);
          fflush(slog);
        }
        if (!ok) {
          ++engine_errors;
          continue;
        }
        recs.push_back(r);
        if (ilog) {
          fprintf(ilog,
                  "{\"seq\":%u,\"source_key\":%llu,\"slot\":%d,\"worker\":%zu,\"t_infer0_ns\":%llu,\"t_infer1_ns\":%llu,"
                  "\"t_done_rx_ns\":%llu,\"tensor_crc\":%u}\n",
                  m.seq, (unsigned long long)m.source_key, s, k, (unsigned long long)r.t_staged,
                  (unsigned long long)r.t_infer, (unsigned long long)m.t_done_ns, crc);
          fflush(ilog);
        }
        if (olog) {
          fprintf(olog,
                  "{\"seq\":%u,\"source_key\":%llu,\"worker\":%zu,\"t_want\":%llu,\"t_first\":%llu,\"t_done\":%llu,"
                  "\"t_acq\":%llu,\"t_staged\":%llu,\"t_infer\":%llu,\"t_out\":%llu,\"prep_ns\":%u,\"hold_ns\":%u,"
                  "\"rx_cpu_ns\":%llu,\"ndet\":%zu,\"cand\":%zu,\"post_us\":%.1f,\"dets\":[",
                  m.seq, (unsigned long long)m.source_key, k, (unsigned long long)m.t_want_ns,
                  (unsigned long long)m.t_first_ns, (unsigned long long)m.t_done_ns, (unsigned long long)r.t_acq,
                  (unsigned long long)r.t_staged, (unsigned long long)r.t_infer, (unsigned long long)r.t_out,
                  m.prep_ns, m.hold_ns, (unsigned long long)m.rx_cpu_ns, r.ndet, r.cand, r.post_us);
          for (size_t i = 0; i < d.size(); ++i)
            fprintf(olog, "%s[%.2f,%.2f,%.2f,%.2f,%.4f,%d]", i ? "," : "", d[i].x1, d[i].y1, d[i].x2, d[i].y2,
                    d[i].score, static_cast<int>(d[i].cls));
          fprintf(olog, "]}\n");
        }
      }
    });
  // steady-window CPU snapshot at t_warm
  double cpu_w = cpu0;
  CpuStat st_w = st0;
  bool warm_taken = warm_s <= 0;
  while (!g_stop && mono_ns() < t_stop) {
    if (!warm_taken && mono_ns() >= t_warm) {
      cpu_w = proc_cpu_ms();
      st_w = read_stat();
      warm_taken = true;
    }
    usleep(20000);
  }
  for (auto& t : th) t.join();
  const uint64_t t_end = mono_ns();
  double cpu1 = proc_cpu_ms();
  CpuStat st1 = read_stat();
  ring.close();
  if (rx) rx->stop();
  if (lsrc) lsrc->stop();
  if (slog) fclose(slog);
  long hz = sysconf(_SC_CLK_TCK);
  long ncpu = sysconf(_SC_NPROCESSORS_ONLN);

  std::sort(recs.begin(), recs.end(), [](const Rec& a, const Rec& b) { return a.t_out < b.t_out; });
  std::vector<double> rx_cpu_us, want_to_done_us, transport_us, lat_first_out_ms, lat_want_out_ms, post_us, infer_ms_v,
      queue_ms;
  size_t n_steady = 0;
  uint64_t first_steady = 0, last_steady = 0;
  size_t dets_total = 0;
  for (const auto& r : recs) {
    const auto& m = r.m;
    rx_cpu_us.push_back(m.rx_cpu_ns / 1e3);
    double wtd = (m.t_done_ns - m.t_want_ns) / 1e3;
    want_to_done_us.push_back(wtd);
    transport_us.push_back(wtd - m.hold_ns / 1e3 - m.prep_ns / 1e3);
    if (r.t_out >= t_warm) {
      ++n_steady;
      if (!first_steady) first_steady = r.t_out;
      last_steady = r.t_out;
      lat_first_out_ms.push_back((r.t_out - m.t_first_ns) / 1e6);
      lat_want_out_ms.push_back((r.t_out - m.t_want_ns) / 1e6);
      post_us.push_back(r.post_us);
      infer_ms_v.push_back((r.t_infer - r.t_staged) / 1e6);
      queue_ms.push_back((r.t_acq - m.t_done_ns) / 1e6);
      dets_total += r.ndet;
    }
  }
  double fps_all = -1, fps_steady = -1;
  if (recs.size() >= 2) fps_all = (recs.size() - 1) / ((recs.back().t_out - recs.front().t_out) / 1e9);
  if (n_steady >= 2) fps_steady = (n_steady - 1) / ((last_steady - first_steady) / 1e9);
  const double steady_wall_ms = (t_end - (warm_taken ? std::max(t_warm, t_start) : t_start)) / 1e6;
  const double cpu_steady_ms = cpu1 - cpu_w;
  double kv_busy = -1;
  if (st_w.ok && st1.ok && st1.total() > st_w.total())
    kv_busy = 1.0 - double(st1.idle() - st_w.idle()) / double(st1.total() - st_w.total());
  std::string engines = "[";
  for (size_t k = 0; k < eng.size(); ++k) engines += (k ? "," : "") + eng[k]->describe();
  engines += "]";
  std::string post_js = "{\"model_tag\":\"" + model_tag + "\",\"conf\":" + std::to_string(post.conf) +
                        ",\"iou\":" + std::to_string(post.iou) + ",\"max_det\":" + std::to_string(post.max_det) +
                        ",\"sort_heads\":" + (post.sort_heads ? "true" : "false") + ",\"cls_thr\":[";
  for (size_t k = 0; k < post.cls_thr.size(); ++k) post_js += (k ? "," : "") + std::to_string(post.cls_thr[k]);
  post_js += "]}";
  uint64_t plf = 0, cpuf = 0;
  for (auto& e : eng) { plf += e->pl_frames(); cpuf += e->cpu_frames(); }
  std::string impl_js = "{\"requested\":\"" + post_impl + "\",\"active\":\"" + std::string(plf ? (cpuf ? "pl+cpu" : "pl") : "cpu") +
                        "\",\"why\":\"" + post_why + "\",\"pl_frames\":" + std::to_string(plf) + ",\"cpu_frames\":" +
                        std::to_string(cpuf) + ",\"core\":" + (plpost ? plpost->describe_json() : std::string("null")) + "}";
  printf(
      "{\"post\":%s,\"post_impl\":%s,\"rx\":%s,\"engines\":%s,\"workers\":%zu,\"inflight\":%zu,\"slots\":%zu,\"inferred\":%zu,\"engine_errors\":%llu,"
      "\"published\":%llu,\"stale_dropped\":%llu,\"pipeline_fps\":%.3f,\"out_fps_steady\":%.3f,\"n_steady\":%zu,"
      "\"warm_s\":%.1f,\"steady_span_s\":%.2f,\"infer_ms\":%.2f,\"run_s\":%.2f,\"process_cpu_ms\":%.1f,"
      "\"process_cpu_ms_per_frame\":%.3f,\"process_cpu_steady_pct_of_one_core\":%.1f,\"kv_busy_steady_frac_allcpu\":%.4f,"
      "\"ncpu\":%ld,\"rx_cpu_us_p50\":%.1f,\"rx_cpu_us_p95\":%.1f,\"want_to_done_us_p50\":%.1f,"
      "\"want_to_done_us_p95\":%.1f,\"transport_us_p50\":%.1f,\"transport_us_p95\":%.1f,"
      "\"lat_first_to_out_ms_p50\":%.2f,\"lat_first_to_out_ms_p95\":%.2f,\"lat_want_to_out_ms_p50\":%.2f,"
      "\"lat_want_to_out_ms_p95\":%.2f,\"infer_wall_ms_p50\":%.2f,\"queue_ms_p50\":%.2f,\"queue_ms_p95\":%.2f,"
      "\"post_us_p50\":%.1f,\"post_us_p95\":%.1f,\"dets_per_frame_mean\":%.2f,\"softirq_ms_allcpu\":%.1f,"
      "\"softirq_ms_steady_allcpu\":%.1f}\n",
      post_js.c_str(), impl_js.c_str(), (rx ? rx->describe_json() : lsrc->describe_json()).c_str(), engines.c_str(), workers, cfg.max_inflight, slots, recs.size(), (unsigned long long)engine_errors,
      (unsigned long long)ring.published(), (unsigned long long)ring.stale_dropped(), fps_all, fps_steady, n_steady,
      warm_s, (last_steady - first_steady) / 1e9, infer_ms, (t_end - t_start) / 1e9, cpu1 - cpu0,
      recs.empty() ? -1.0 : (cpu1 - cpu0) / recs.size(), 100.0 * cpu_steady_ms / steady_wall_ms, kv_busy, ncpu,
      pct(rx_cpu_us, .5), pct(rx_cpu_us, .95), pct(want_to_done_us, .5), pct(want_to_done_us, .95),
      pct(transport_us, .5), pct(transport_us, .95), pct(lat_first_out_ms, .5), pct(lat_first_out_ms, .95),
      pct(lat_want_out_ms, .5), pct(lat_want_out_ms, .95), pct(infer_ms_v, .5), pct(queue_ms, .5), pct(queue_ms, .95),
      pct(post_us, .5), pct(post_us, .95), n_steady ? double(dets_total) / n_steady : -1.0,
      (st0.ok && st1.ok) ? 1e3 * (st1.v[6] - st0.v[6]) / hz : -1.0,
      (st_w.ok && st1.ok) ? 1e3 * (st1.v[6] - st_w.v[6]) / hz : -1.0);
  if (ilog) fclose(ilog);
  if (olog) fclose(olog);
  return 0;
}
