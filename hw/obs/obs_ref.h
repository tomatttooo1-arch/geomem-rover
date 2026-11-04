// obs_ref.h -- C++ 参考：识别框 -> 三维观测（产品 perception/fpga_replay/core.py 的 process_frame 同算法）
//
// 每个框：内缩 15% -> 有效深度 (0.1, 10) m -> 5 cm 直方图找"最近显著表面"（identity_v2）-> 距表面 8 cm 内的掩码
// -> 8 连通最大连通域 -> 反投影到世界系 -> 5%/95% 分位 ±3 cm 作范围、中位数作位置 -> 掩码内 RGB 转 HSV（OpenCV 8 位公式）
// 取各通道中位数 -> 颜色类别。浮点参考，不依赖 OpenCV；数值类型逐步对齐 numpy 2.x 的提升规则（见 obs_ref.cpp 注释）。
#pragma once
#include <cstdint>
#include <vector>

namespace obsref {

struct Config {
    int    min_pixels = 9;               // Config.min_pixels
    double min_valid_fraction = 0.7;     // Config.min_valid_fraction
    bool   identity_v2 = true;           // Config.identity_v2（最近显著表面）
    double near_surface_fraction = 0.2;  // Config.near_surface_fraction
    double edge_margin_px = 2.0;         // Config.edge_margin_px（截断判定）
};

struct Frame {
    int width = 0, height = 0;
    const float*   depth_m = nullptr;    // H*W，光轴 Z（米），无效 = NaN（用 depth_from_mm 由 u16 毫米生成）
    const uint8_t* rgb = nullptr;        // H*W*3，RGB 顺序
    double K[9];                         // 行主序内参
    double T[16];                        // 行主序 T_world_camera（相机 -> 世界）
};

struct Box { double x1, y1, x2, y2; };   // 检测框原值（像素，浮点）

enum Status { OK = 0, EMPTY_INNER = 1, LOW_VALID = 2, NO_COMPONENT = 3, SMALL_COMPONENT = 4 };
enum Color  { BLACK = 0, WHITE, GRAY, RED, YELLOW, GREEN, BLUE, PURPLE };
const char* color_name(int c);

struct Result {
    int    status;                       // Status；只有 OK 才算"出观测"（Python 返回 1 条）
    int    inner[4];                     // 内缩后的整数框 x1,y1,x2,y2（半开）
    int    n_valid, n_mask;              // 有效深度像素数、最大连通域像素数
    double valid_fraction;               // Observation.valid_fraction
    double position[3], lower[3], upper[3];   // 世界系，米（lower/upper 已含 ±0.03 m）
    double extent_m[2];                  // Observation.extent_m（框在观测深度处的宽高，米）
    double range_m;                      // Observation.range_m（相机系 Z）
    int    truncated;                    // Observation.truncated
    int    color;                        // Color
    double hsv_median[3];                // H(0..179) S V 的中位数（诊断用）
};

// 可复用的工作区（避免每框分配；计时程序用）
struct Workspace {
    std::vector<uint8_t>  mask;
    std::vector<int32_t>  label, parent;
    std::vector<int64_t>  area;
    std::vector<int32_t>  counts;
    std::vector<double>   xs, ys, zs;
    std::vector<uint8_t>  hh, ss, vv;
};

// u16 毫米 -> float 米（0 -> NaN），与 io.load_frame 的 depth.astype(float32)/1000 相同
void depth_from_mm(const uint16_t* mm, int n, float* out_m);

// OpenCV COLOR_RGB2HSV（8 位）单像素
void rgb2hsv_u8(const uint8_t rgb[3], uint8_t hsv[3]);

// 一个框；返回 status
int process_box(const Frame& f, const Box& b, Result& r, const Config& cfg, Workspace& ws);

// 一帧若干框 -> 观测列表：res[i] 对应 boxes[i]；返回出观测的框数
int process_frame(const Frame& f, const Box* boxes, int n, Result* res,
                  const Config& cfg = Config(), Workspace* ws = nullptr);

}  // namespace obsref
