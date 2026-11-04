// DPUPIPE1 (2026-11-03): see infer_engine.h.
#include "infer_engine.h"

#include <time.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "dpu_rx.h"  // crc32_bytes

#ifdef HAVE_VART
#include <vart/runner.hpp>
#include <vart/runner_ext.hpp>
#include <vart/tensor_buffer.hpp>
#include <xir/graph/graph.hpp>
#endif

namespace dpupipe {

// ---- stub ------------------------------------------------------------------------------------
std::mutex& StubEngine::dpu() {
  static std::mutex m;
  return m;
}

bool StubEngine::stage(const uint8_t* input, size_t bytes) {
  if (tensor_crc_) crc_ = crc32_bytes(input, bytes);
  return true;
}

bool StubEngine::infer() {
  if (infer_ms_ > 0) {
    std::lock_guard<std::mutex> lk(dpu());  // one DPU core
    timespec ts{static_cast<time_t>(infer_ms_ / 1000), static_cast<long>(std::fmod(infer_ms_, 1000.0) * 1e6)};
    nanosleep(&ts, nullptr);
  }
  return true;
}

std::vector<Det> StubEngine::post(PostTiming* t) {
  if (t) *t = PostTiming();
  return {};
}

std::string StubEngine::describe() const {
  char b[96];
  snprintf(b, sizeof b, "{\"engine\":\"stub\",\"infer_ms\":%.2f}", infer_ms_);
  return b;
}

#ifdef HAVE_VART
// ---- VART ------------------------------------------------------------------------------------
struct VartShared {
  std::unique_ptr<xir::Graph> graph;
  const xir::Subgraph* dpu = nullptr;
  std::string path;
};

std::shared_ptr<VartShared> VartEngine::load_model(const std::string& xmodel) {
  auto m = std::make_shared<VartShared>();
  m->path = xmodel;
  m->graph = xir::Graph::deserialize(xmodel);
  for (auto* s : m->graph->get_root_subgraph()->children_topological_sort())
    if (s->has_attr("device") && s->get_attr<std::string>("device") == "DPU") {
      if (m->dpu) throw std::runtime_error("more than one DPU subgraph in " + xmodel);
      m->dpu = s;
    }
  if (!m->dpu) throw std::runtime_error("no DPU subgraph in " + xmodel);
  return m;
}

struct VartEngine::Impl {
  std::unique_ptr<vart::Runner> runner;
  vart::RunnerExt* ext = nullptr;
  std::vector<vart::TensorBuffer*> in, out;
  uint8_t* in_ptr = nullptr;
  std::vector<size_t> out_bytes;
};

VartEngine::VartEngine(std::shared_ptr<VartShared> m, bool tensor_crc)
    : impl_(new Impl), model_(std::move(m)), tensor_crc_(tensor_crc) {
  impl_->runner = vart::Runner::create_runner(model_->dpu, "run");
  impl_->ext = dynamic_cast<vart::RunnerExt*>(impl_->runner.get());
  if (!impl_->ext) throw std::runtime_error("runner is not a RunnerExt (no own TensorBuffers)");
  impl_->in = impl_->ext->get_inputs();
  impl_->out = impl_->ext->get_outputs();
  if (impl_->in.size() != 1) throw std::runtime_error("expected one input tensor");
  const xir::Tensor* it = impl_->in[0]->get_tensor();
  auto ish = it->get_shape();
  if (ish.size() != 4 || ish[0] != 1 || ish[3] != 3) throw std::runtime_error("input shape not (1,H,W,3)");
  in_h_ = ish[1];
  in_w_ = ish[2];
  in_fix_ = it->get_attr<int>("fix_point");
  in_bytes_ = static_cast<size_t>(in_h_) * in_w_ * 3;
  auto d = impl_->in[0]->data({0, 0, 0, 0});
  if (d.second < in_bytes_) throw std::runtime_error("input TensorBuffer smaller than the tensor");
  impl_->in_ptr = reinterpret_cast<uint8_t*>(d.first);
  for (auto* tb : impl_->out) {
    const xir::Tensor* t = tb->get_tensor();
    auto sh = t->get_shape();
    if (sh.size() != 4 || sh[0] != 1) throw std::runtime_error("output shape not (1,h,w,c)");
    HeadView h{};
    h.h = sh[1];
    h.w = sh[2];
    h.c = sh[3];
    h.fix_point = t->get_attr<int>("fix_point");
    size_t n = static_cast<size_t>(h.h) * h.w * h.c;
    auto od = tb->data({0, 0, 0, 0});
    if (od.second < n) throw std::runtime_error("output TensorBuffer smaller than its tensor");
    h.data = reinterpret_cast<const int8_t*>(od.first);
    heads_.push_back(h);
    impl_->out_bytes.push_back(n);
  }
}

VartEngine::~VartEngine() = default;

bool VartEngine::stage(const uint8_t* input, size_t bytes) {
  if (bytes != in_bytes_) return false;
  std::memcpy(impl_->in_ptr, input, bytes);
  if (tensor_crc_) crc_ = crc32_bytes(impl_->in_ptr, bytes);
  return true;
}

bool VartEngine::infer() {
  impl_->in[0]->sync_for_write(0, in_bytes_);
  auto job = impl_->runner->execute_async(impl_->in, impl_->out);
  if (job.second != 0) return false;
  if (impl_->runner->wait(static_cast<int>(job.first), -1) != 0) return false;
  // RECOG641: with the PL core the CPU never reads the heads (the core reads DDR), so no cache sync here; the CPU
  // fallback in post() syncs first
  outputs_synced_ = false;
  if (!pl_) {
    for (size_t i = 0; i < impl_->out.size(); ++i) impl_->out[i]->sync_for_read(0, impl_->out_bytes[i]);
    outputs_synced_ = true;
  }
  return true;
}

bool VartEngine::attach_pl_post(std::shared_ptr<PlPost> p, std::string* why) {
  std::vector<uint64_t> phy;
  for (auto* tb : impl_->out) {
    if (tb->get_location() != vart::TensorBuffer::location_t::HOST_PHY) {
      *why = "output " + tb->get_tensor()->get_name() + " is not HOST_PHY (no physical address for the core)";
      return false;
    }
    phy.push_back(tb->data_phy(std::vector<std::int32_t>(4, 0)).first);
  }
  if (!p->layout(heads_, phy, &pl_lay_, why)) return false;
  pl_ = std::move(p);
  return true;
}

std::vector<Det> VartEngine::post(PostTiming* t) {
  if (pl_ && !pl_->dead()) {
    std::vector<Det> d;
    if (pl_->run(pl_lay_, &d, t)) {
      ++pl_frames_;
      return d;
    }
  }
  if (!outputs_synced_) {
    for (size_t i = 0; i < impl_->out.size(); ++i) impl_->out[i]->sync_for_read(0, impl_->out_bytes[i]);
    outputs_synced_ = true;
  }
  ++cpu_frames_;
  // RECOG641: stride = tensor width / head width (in_w_; in_h_ was only right for square inputs)
  return postprocess_cfg(heads_, in_w_, post_cfg_, t);
}

std::string VartEngine::describe() const {
  std::string s = "{\"engine\":\"vart\",\"xmodel\":\"" + model_->path + "\",\"input\":[" + std::to_string(in_h_) +
                  "," + std::to_string(in_w_) + "],\"in_fix\":" + std::to_string(in_fix_) + ",\"heads\":[";
  for (size_t i = 0; i < heads_.size(); ++i)
    s += (i ? "," : "") + std::string("[") + std::to_string(heads_[i].h) + "," + std::to_string(heads_[i].w) + "," +
         std::to_string(heads_[i].c) + "," + std::to_string(heads_[i].fix_point) + "]";
  s += "],\"post\":{\"impl\":\"" + std::string(pl_ ? "pl" : "cpu") + "\",\"pl_frames\":" + std::to_string(pl_frames_) +
       ",\"cpu_frames\":" + std::to_string(cpu_frames_);
  if (pl_) s += ",\"layout\":{\"in_phy\":" + std::to_string(pl_lay_.in_phy) + ",\"span\":" + std::to_string(pl_lay_.span) +
                ",\"off\":[" + std::to_string(pl_lay_.off[0]) + "," + std::to_string(pl_lay_.off[1]) + "," +
                std::to_string(pl_lay_.off[2]) + "],\"grid\":[" + std::to_string(pl_lay_.grid[0]) + "," +
                std::to_string(pl_lay_.grid[1]) + "," + std::to_string(pl_lay_.grid[2]) + "]}";
  return s + "}}";
}
#endif

}  // namespace dpupipe
