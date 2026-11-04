// m5 slot 12 0xA00C_0000，pl_ps_irq0 bit 1 / SPI 90）板上驱动。
//
//   MAP  0x10  代价图 u8，行优先 y*W+x，W = 80（grid_mode 0）或 200（grid_mode 1），读 W*W 字节
//   FIELD 0x1c 势场 u32 x W*W（valid 时为势值，否则全 INF：模式 0 = 0x00ffffff，模式 1 = 0x1fffffff）
//   FLAGS 0x28 每轮"有变化"标志 u32 x 64 / 256（未用的轮为 255）
//   SX/SY/GX/GY 0x34/0x3c/0x44/0x4c、K_MAX 0x54（模式 0 ≤64，模式 1 ≤256）、GRID_MODE 0x88（0/1，其余 → status 8 且不碰 DDR）
//   STATUS 0x5c / ROUNDS 0x6c / FIELD_VALID 0x7c（各带 ap_vld，*_CTRL 读清）
//   status：0 收敛（field 有效）、7 K 轮内未收敛、2 起点在边框、3 终点不可走、6 参数非法、8 grid_mode 不支持。
// 核对参数自己做检查、非法参数不会挂死，所以驱动只查 grid_mode（决定缓冲大小）与指针；其余原样交给核，由 status 报。
// 挂死保护：超时（模式 0 50 ms、模式 1 1000 ms，均为最坏轮数拍数的 4 倍以上）未见 ap_done ⇒ 判挂死，之后不再启动。
// 缓冲：三块物理连续、整段在 2 GiB 下（LPD 互连只映 LPD_DDR_LOW），按模式 1 的大小一次分配，两种模式共用。
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "../../m7/pl/sw/pl_io.hpp"

namespace planner_drv {

constexpr std::uint64_t BASE = 0xA00C0000ull;   // = M7PL_BASE_PLANNER（hw/m7/pl/sw/m7pl_addrmap.h）
constexpr const char* UIO_NAME = "m7_planner";
constexpr int CELLS_MAX = 40000, FLAGS_MAX = 256;
inline int width(int mode) { return mode == 1 ? 200 : 80; }
inline int cells(int mode) { return width(mode) * width(mode); }
inline int nflags(int mode) { return mode == 1 ? 256 : 64; }
inline std::uint32_t inf(int mode) { return mode == 1 ? 0x1fffffffu : 0x00ffffffu; }
enum Status { OK = 0, START_ON_BORDER = 2, GOAL_BLOCKED = 3, NOT_CONVERGED = 7, BAD_ARGS = 6, BAD_MODE = 8 };

struct Request {
  int mode = 0;                       // grid_mode
  const std::uint8_t* map = nullptr;  // W x W 代价图
  int map_stride = 0;                 // 行步长（字节）；0 = W
  int sx = 0, sy = 0, gx = 0, gy = 0;
  unsigned k_max = 64;
};
struct Result {
  std::uint32_t status = 0, rounds = 0, valid = 0;
  std::uint64_t core_ns = 0;          // 写 ap_start 到看到完成（A53 单调时钟）
  std::uint64_t copy_in_ns = 0, copy_out_ns = 0;
};

class Core {
 public:
  ~Core() { close(); }
  bool open(plio::Options o, std::string* why);
  bool open(std::unique_ptr<plio::Bus> bus, std::string* why);
  void close();
  // 一次求场。field（cells(mode) 个 u32）与 flags（nflags(mode) 个 u32）可为 nullptr（不拷出）。
  // 返回 false = 驱动层失败（未打开、挂死、核忙）；核自己的结果看 res->status。
  bool run(const Request& q, std::uint32_t* field, std::uint32_t* flags, Result* res, std::string* why);
  bool dead() const { return dead_; }
  int timeout_ms[2] = {50, 1000};
  const plio::Buf& map_buf() const { return map_; }
  plio::Bus* bus() { return bus_.get(); }
  std::string describe() const;

 private:
  std::unique_ptr<plio::Bus> bus_;
  plio::Buf map_, field_, flags_;
  bool dead_ = false;
};

// C ABI 句柄建在给定总线上（宿主 mock 测试用）；失败返回 nullptr。
void* open_handle_on(std::unique_ptr<plio::Bus> bus, std::string* why);
Core* handle_core(void* h);

}  // namespace planner_drv

// C ABI（ctypes / kv_nav）。kv_nav 的 PL 势场钩子 kv_nav::PlFieldFn（tools/kv_nav/kv_nav.hpp）语义：
//   返回 0 = 场就绪；7 = NOT_CONVERGED（kv_nav 改用软件势场）；其他 = 请求非法。
// 驱动层失败（挂死、打不开）一律返回 7，让 kv_nav 退回软件势场，不让规划失败。
extern "C" {
void* planner_drv_open(const char* wait, const char* dma);   // nullptr = 失败，原因见 planner_drv_open_error()
const char* planner_drv_open_error(void);
void planner_drv_close(void* h);
const char* planner_drv_describe(void* h);
int planner_drv_kv_nav_field(void* h, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                             std::uint32_t* potential, unsigned* rounds);
// 通用一次求场：out3 = {status, rounds, valid}；返回 0 = 驱动层成功，-1 = 驱动层失败（原因见 planner_drv_error）
int planner_drv_run(void* h, int mode, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                    std::uint32_t* field, std::uint32_t* flags, std::uint32_t* out3, std::uint64_t* core_ns);
const char* planner_drv_error(void* h);
}
