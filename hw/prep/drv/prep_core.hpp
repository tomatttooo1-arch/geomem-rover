// DRV5（2026-11-04）：识别前处理核（PREP1 s18，m5 slot 11 0xA00B_0000，pl_ps_irq0 bit 6 / SPI 95）板上驱动。
//
// 接口以 runs/nationals-prelim-20261102/PREP1/README.md §6 为准：寄存器只有 SRC/DST/PRM 三个 64 位地址、CTRL/中断、
// 只读 STATUS（[31:16] 成功帧数、[15:8] 版本 0x02、bit0 帧头非法）；其余全部在 DDR 参数块（643 个 128 位字），
// 由 hw/prep/hls1/prep_drv.h 的 prep_drv_params() 生成（与 PREP1 csim / 联仿用的是同一份代码）。
// §6.4 的 14 条检查在本驱动里的落点：
//   1–10  check_config()：prep_drv_check() 的 1–8 条逐条给原因、prep_drv_axis() 的第 9 条、
//         第 10 条由 build_params() 只经 prep_drv_params() 生成整块参数块保证（不提供改单个字段的接口）；
//   11    configure() 分配三块缓冲后核对齐（16 字节；DST 4 KB）、大小、互不重叠、整段在 2 GiB 下；
//   12    configure() / run() 写参数块、地址、ap_start 之前都先确认 CTRL.ap_idle=1；
//   13    参数块写完、源帧拷完都 to_device()（刷出缓存）；张量缓冲 A53 只读、只在核写完后 from_device()；
//   14    run() 超时（默认 50 ms）未见 ap_done ⇒ 判核挂死，之后不再启动它（dead()=true），需要重载比特流复位 PL。
// 帧计数：每帧启动前读 STATUS 的帧数，完成后要求 bit0=0 且帧数 +1（16 位回绕）才算成功。
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "../../m7/pl/sw/pl_io.hpp"

namespace prep_drv {

constexpr std::uint64_t BASE = 0xA00B0000ull;   // = M7PL_BASE_PREP（hw/m7/pl/sw/m7pl_addrmap.h）
constexpr const char* UIO_NAME = "m7_prep";
constexpr int PRM_WORDS = 643;                  // = PREP_DRV_PRM_WORDS
constexpr std::size_t PRM_BYTES = PRM_WORDS * 16;
constexpr int DEFAULT_TIMEOUT_MS = 50;          // §6.4 第 14 条（正常一帧约 3.2 ms 的 15 倍）
enum Fmt { RGB = 0, BGR = 1, YUYV = 2, UYVY = 3 };

// 与 prep_drv.h 的 prep_cfg_t 一一对应（这里另写一份，免得把 prep_drv.h 的 static 函数带进每个编译单元）。
struct Config {
  int w0 = 0, h0 = 0, stride = 0, fmt = YUYV, ups = 0, bypass = 0;
  int tw = 0, th = 0, ow = 0, oh = 0, top = 0, left = 0, fp = 0;
  int kset = 0;                                 // 0 = JFIF 全范围（PREP_K_JFIF），1 = BT.601 有限范围（PREP_K_BT601L）
};
inline int bytes_per_pixel(int fmt) { return fmt >= YUYV ? 2 : 3; }
inline std::size_t src_bytes(const Config& c) { return (std::size_t)c.h0 * c.stride; }
inline std::size_t tensor_bytes(const Config& c) { return (std::size_t)c.tw * c.th * 3; }

// §6.4 第 1–9 条；失败返回 false 并给出是哪一条。
bool check_config(const Config& c, std::string* why);
// 检查后生成整块参数块（PRM_BYTES 字节，字 d 的字节 0..15 = 位 [7:0]..[127:120]）。perturb=1 为阳性对照。
bool build_params(const Config& c, std::uint8_t* blk, std::string* why, int perturb = 0);

struct Result {
  std::uint32_t status = 0;      // 本帧完成后的 STATUS
  std::uint64_t copy_in_ns = 0;  // 源帧拷进 DMA 缓冲＋刷缓存
  std::uint64_t core_ns = 0;     // 写 ap_start 到看到完成（A53 单调时钟）
  std::uint64_t copy_out_ns = 0; // 张量失效缓存＋拷出
};

class Core {
 public:
  ~Core() { close(); }
  // 打开真硬件（Options 的 base/uio 名缺省填本核的）；或交给它一个 Bus（宿主 mock）。
  bool open(plio::Options o, std::string* why);
  bool open(std::unique_ptr<plio::Bus> bus, std::string* why);
  void close();
  // 换几何或模型时调用：检查、写参数块、按本配置分配源与张量缓冲（大小不够才重分配）、核对第 11 条。
  bool configure(const Config& c, std::string* why);
  // 一帧：src 每行 src_stride 字节（只拷每行 w0·bpp 字节，行尾用 0 补到驱动自己的步长），结果整块张量写进 out。
  bool run(const std::uint8_t* src, int src_stride, std::uint8_t* out, std::size_t out_bytes, Result* res,
           std::string* why);
  bool dead() const { return dead_; }
  int timeout_ms = DEFAULT_TIMEOUT_MS;
  // 宿主测试与板上冒烟用
  const plio::Buf& prm_buf() const { return prm_; }
  const plio::Buf& src_buf() const { return src_; }
  const plio::Buf& dst_buf() const { return dst_; }
  const Config& config() const { return cfg_; }
  plio::Bus* bus() { return bus_.get(); }
  std::string describe() const;

 private:
  bool ensure(plio::Buf& b, std::size_t size, std::string* why);
  std::unique_ptr<plio::Bus> bus_;
  plio::Buf src_, dst_, prm_;
  Config cfg_;
  bool configured_ = false, dead_ = false;
};

}  // namespace prep_drv
