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
//
//   PATH_CTRL 0x90 bit0 = 核内取路径（PLNPATH0 DESIGN §4.4）：132 字节参数块紧跟在代价图后面（map 偏移 W*W），
//     结果在 flags 字 F = nflags 起：表头 + n 个路径点。所以 map 缓冲 ≥ 40,132 字节、flags 缓冲 ≥ 4,353 字。
//   GMAP_OP 0x98：0 = 规划（上面的行为，逐位不变）；1 MERGE、2 SCAN、3 CLEAR、4 WINDOW 作用在"地图区"
//     （GMAP0 DESIGN §4.1 + DESIGN_ADD_PLGRID §2.1，GMAP3 定 Q16，共 1,173,088 字节），MAP = FIELD = 地图区基址；
//     STATUS 读到结果字 0（0x200 = 版本 2 成功；|1 魔数错/点数 >21、|2 SCAN 外接框空、|4 操作码非法），
//     结果 4 字在地图区 526,416。地图区单独分配一块，PS 往 area() 里写输入（参数块、帧、aux），gmap() 负责缓存维护。
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
constexpr int PATH_PARAM_BYTES = 132, PATH_POINTS_MAX = 4096;
constexpr int MAP_BYTES = CELLS_MAX + PATH_PARAM_BYTES;          // 40,132
constexpr int FLAGS_WORDS = FLAGS_MAX + 1 + PATH_POINTS_MAX;             // 4,353
// 地图区（字节偏移）
constexpr std::size_t AREA_BYTES = 1173088;
constexpr std::size_t OFF_GLOBAL = 0, OFF_COARSE = 320000, OFF_FRAME = 360000, OFF_PARAM = 366400,
                      OFF_IMAGE = 366416, OFF_RESULT = 526416, OFF_T = 526432, OFF_AUX = 1166432,
                      OFF_WPARAM = 1172832;
enum GmapOp { GMAP_MERGE = 1, GMAP_SCAN = 2, GMAP_CLEAR = 3, GMAP_WINDOW = 4 };
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
  const std::uint8_t* path_params = nullptr;   // 非空 = 核内取路径（PATH_CTRL bit0），132 字节参数块
  std::uint32_t* path_out = nullptr;           // 取路径结果：表头 + n 点（≤ 1 + 4096 字）；可为 nullptr
};
struct Result {
  std::uint32_t status = 0, rounds = 0, valid = 0;
  std::uint64_t core_ns = 0;          // 写 ap_start 到看到完成（A53 单调时钟）
  std::uint64_t copy_in_ns = 0, copy_out_ns = 0;
  std::uint32_t path_header = 0;      // 取路径表头（未取路径时 0）
};
struct GmapResult {
  std::uint32_t w[4] = {0, 0, 0, 0};  // 结果字 0..3（w[0] = STATUS）
  std::uint64_t core_ns = 0;
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
  // 地图区一次操作（GMAP_OP = op）。调用前把输入写进 area()（MERGE：参数块＋帧；SCAN/CLEAR：参数块；
  // WINDOW：aux＋WINDOW 参数块）；返回后 area() 里结果字和该操作写的区（MERGE/CLEAR：全局图＋粗图；SCAN：图像＋
  // 粗图；WINDOW：帧槽）已可读。T 图 PS 不读不写。返回 false = 驱动层失败。
  bool gmap(unsigned op, GmapResult* r, std::string* why);
  std::uint8_t* area() const { return (std::uint8_t*)area_.va; }
  bool dead() const { return dead_; }
  int timeout_ms[2] = {50, 1000};
  int timeout_gmap_ms = 100;          // 最长 SCAN ≈ 5.3 ms、CLEAR(含 T) ≈ 3 ms，留 ≥ 15 倍
  const plio::Buf& map_buf() const { return map_; }
  plio::Bus* bus() { return bus_.get(); }
  std::string describe() const;

 private:
  std::unique_ptr<plio::Bus> bus_;
  plio::Buf map_, field_, flags_, area_;
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
// GMAP3：带取路径的一次求场（params132 = PLNPATH0 参数块；path_out ≥ 4097 字：表头 + 点）
int planner_drv_run_path(void* h, int mode, const std::uint8_t* map, int sx, int sy, int gx, int gy, unsigned kmax,
                         const std::uint8_t* params132, std::uint32_t* field, std::uint32_t* flags,
                         std::uint32_t* path_out, std::uint32_t* out3, std::uint64_t* core_ns);
// GMAP3：地图区指针（1,173,088 字节，DMA 内存）与一次地图操作（out4 = 结果字 0..3；返回 0 / -1）
std::uint8_t* planner_drv_gmap_area(void* h);
int planner_drv_gmap(void* h, unsigned op, std::uint32_t* out4, std::uint64_t* core_ns);
}
