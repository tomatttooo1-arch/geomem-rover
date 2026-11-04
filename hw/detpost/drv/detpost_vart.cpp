#include "detpost_vart.hpp"

#include <vart/runner_ext.hpp>
#include <vart/tensor_buffer.hpp>
#include <xir/tensor/tensor.hpp>

namespace detpost_drv {

bool heads_from_vart(const std::vector<vart::TensorBuffer*>& outs, VartHeads* out, std::string* why) {
  if (outs.size() != 3) { *why = "expected 3 DPU output tensors, got " + std::to_string(outs.size()); return false; }
  uint64_t phy[3];
  size_t bytes[3];
  int grid[3];
  int ch = -1, fp = 0;
  for (int i = 0; i < 3; i++) {
    auto* tb = outs[i];
    const xir::Tensor* t = tb->get_tensor();
    const auto shape = t->get_shape();
    if (shape.size() != 4 || shape[0] != 1 || shape[1] != shape[2]) { *why = "unexpected shape for " + t->get_name(); return false; }
    if (tb->get_location() != vart::TensorBuffer::location_t::HOST_PHY) {
      *why = "output " + t->get_name() + " is not HOST_PHY (use RunnerExt::get_outputs())";
      return false;
    }
    if (ch >= 0 && shape[3] != ch) { *why = "channel count differs between heads"; return false; }
    ch = shape[3];
    const int this_fp = t->has_attr("fix_point") ? t->get_attr<int>("fix_point") : 0;
    if (i > 0 && this_fp != fp) { *why = "fix_point differs between heads"; return false; }
    fp = this_fp;
    const auto p = tb->data_phy(std::vector<std::int32_t>(4, 0));
    phy[i] = p.first;
    bytes[i] = p.second;            // 自该点起可访问的字节数，>= H*W*C
    grid[i] = shape[1];
    if (bytes[i] > (size_t)shape[1] * shape[2] * shape[3]) bytes[i] = (size_t)shape[1] * shape[2] * shape[3];
  }
  out->nc = ch - 4 * detpost::REG_MAX;
  out->fp = fp;
  return make_layout(phy, bytes, grid, out->nc, &out->lay, why);
}

}  // namespace detpost_drv
