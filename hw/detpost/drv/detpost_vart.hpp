// VART 3.0 衔接：从 DPU runner 取三个输出头的物理地址，零拷贝交给后处理核。宿主只编译检查（aarch64 -fsyntax-only）。
//
// 做法（读 VART 3.0 源码 vart/dpu-runner/src/imp/dpu_runner_ddr.cpp、vart/runner/include/vart/tensor_buffer.hpp）：
//  1. runner = vart::Runner::create_runner(dpu_subgraph, "run")；dynamic_cast 成 vart::RunnerExt。
//  2. ins = ext->get_inputs()、outs = ext->get_outputs()：runner 自己在 /dev/dpu CMA 里分配的 TensorBuffer，
//     location = HOST_PHY（BufferObjectDpCma：DPUIOC_CREATE_BO 给出物理地址）。
//  3. 每帧把图像写进 ins[0]->data()，ins[0]->sync_for_write()，execute_async(ins, outs) + wait()：
//     传入的就是 runner 自己的缓冲，DpuRunnerDdr::prepare_output 对 HOST_PHY 走「zero copy, do nothing」分支，不拷贝。
//  4. outs[i]->data_phy({0,0,0,0}).first = 第 i 个头的物理地址（= 工作区基址 + xmodel ddr_addr），
//     shape = {1,H,W,C}，C = 64+nc，fix_point 属性给出 fp。DPU 经 HP 口写 DDR，PL 核经 HP3 读 DDR，
//     两者都不经 CPU 缓存，所以 PL 读之前不需要 sync_for_read（CPU 不读输出头时）。
//  5. make_layout() 由三个物理地址推出起点与偏移；起点不满足 16 字节对齐、超 1 MiB 或高于 2 GiB 时拒绝（退回 CPU 后处理）。
#pragma once
#include <string>
#include <vector>
#include "detpost_drv.hpp"

namespace vart { class RunnerExt; class TensorBuffer; }

namespace detpost_drv {

struct VartHeads {
  Layout lay;
  int nc = 0;
  int fp = 0;
};

// outs = RunnerExt::get_outputs()；按 shape 认出三个检测头（C = 64+nc、H 构成 2 倍金字塔），返回布局与 fp。
bool heads_from_vart(const std::vector<vart::TensorBuffer*>& outs, VartHeads* out, std::string* why);

}  // namespace detpost_drv
