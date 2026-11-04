// RECOG641（2026-11-04）：检测后处理核（slot 10，0xA00A_0000）在 plio 访问层上的驱动，dpu_rx --post pl 用它。
//
// 与 detpost_drv.hpp 的 Core 的区别：寄存器与 DMA 缓冲都经 hw/m7/pl/sw/pl_io.hpp 的 plio::Bus（与前处理核
// prep_core 同一套：/dev/mem 或 UIO 窗口；缓冲按 dma_heap reserved → linux,cma → /dev/dpu 顺序取，整段在 2 GiB 以下；
// 宿主 MockBus 可以把核的 HLS C 源挂在 ap_start 上跑），所以驱动的寄存器与缓冲协议能在宿主上整条验证。
// 打包（pack_params）、三头布局（make_layout）与前提检查沿用 detpost_drv.cpp，不另写一份。原 Core 不再被产品代码使用。
//
// 每帧：DPU（VART wait() 返回）→ run(布局)：写 in/prm/out 地址、nc、thr、IoU、max_det、三个网格边长与偏移 → ap_start →
// 等 ap_done → 刷缓存读回状态头与框表。核挂死（超时）后标记 dead，之后不再启动（调用方回落 CPU 后处理）。
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "../../m7/pl/sw/pl_io.hpp"
#include "detpost_drv.hpp"

namespace detpost_drv {

constexpr const char* UIO_NAME = "m7_detpost";        // 设备树节点名（绑 generic-uio 时用中断等待）
constexpr std::uint64_t SYSINFO_BASE = 0xA0000000ull;  // = M7PL_BASE_SYSINFO
constexpr std::uint32_t SYSINFO_MAGIC = 0x4D37504Cu;   // "M7PL"
constexpr std::uint32_t SYSINFO_SLOT_MAP = 0x20;
constexpr int SLOT_BIT_DETPOST = 10;                   // MERGE5A：SLOT_MAP bit 10 = 本核在比特流里

// 读 sysinfo（MAGIC + SLOT_MAP bit 10）判断当前比特流有没有本核；没有就别去碰 0xA00A_0000。
bool core_in_bitstream(std::string* why, std::uint32_t* slot_map = nullptr);

struct RunResult {
  detpost::Status st{};
  int n = 0;
  detpost::Box box[MAX_DET];
  std::uint64_t core_ns = 0;   // 写 ap_start 到看到 ap_done
  std::uint64_t total_ns = 0;  // 写寄存器 + 核 + 读回
};

class BusCore {
 public:
  ~BusCore() { close(); }
  // 真硬件：o.base 缺省 BASE，o.uio_name 缺省 UIO_NAME；wait / provider 按 plio::Options。
  bool open(plio::Options o, std::string* why);
  // 给定总线（宿主 MockBus）。
  bool open(std::unique_ptr<plio::Bus> bus, std::string* why);
  void close();
  // 初始化或换模型时：检查接口前提、打包参数块并刷缓存；框表缓冲清一次脏行（之后 CPU 只读它）。
  bool configure(const detpost::Config& cfg, std::string* why);
  // 每帧。lay 由 make_layout 推出（三头物理地址必须在 bus 能访问的内存里）。
  bool run(const Layout& lay, RunResult* res, int timeout_ms, std::string* why);
  bool dead() const { return dead_; }
  bool configured() const { return configured_; }
  plio::Bus* bus() { return bus_.get(); }
  const detpost::Config& config() const { return cfg_; }
  std::string describe() const;

 private:
  std::unique_ptr<plio::Bus> bus_;
  plio::Buf prm_, out_;
  detpost::Config cfg_{};
  bool configured_ = false, dead_ = false;
};

}  // namespace detpost_drv
