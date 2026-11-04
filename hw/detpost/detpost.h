// 检测后处理核（YOLOv8 无锚框检测头）定点参考模型 —— 公共定义
//
// 输入：DPU 写进 DDR 的三个输出头，原样字节（int8，NHWC，每格 4*REG_MAX+NC 个通道，
//       通道顺序 = 64 个框分布（DFL）logit 在前、NC 个类别 logit 在后）。
//       三个头在 DDR 里的排列由编译器决定（本工程 xmodel：20x20 在偏移 0、40x40 在 38800、
//       80x80 在 194000），所以偏移由寄存器给出，不写死。
// 输出：按分数从高到低排好的框表（最多 max_det 个），坐标是 640 letterbox 图上的像素，Q4 定点。
//
// 语义对齐 ultralytics YOLOv8 predict 的后处理（见 README §5）：
//   取每格最大类别（同分取下标小的）→ 分数 > conf → 按分数降序（同分按格子序，稳定）
//   → 按类 NMS（IoU > iou 抑制）→ 前 max_det 个 → （车上做法）类别白名单与按类下限。
// 全部是整数运算；需要指数/sigmoid 的两处都换成主机算好的查表（寄存器口写入）。
//
// 写法约束（可综合风格）：无动态内存、无 STL 容器、循环上界固定；主机侧工具函数单独放在
// detpost_host.cpp，不参与综合。
#pragma once
#include <stdint.h>

namespace detpost {

// ---- 编译期上限（硬件里决定存储大小） ----
constexpr int REG_MAX = 16;          // DFL 每边 16 个 bin
constexpr int NC_MAX = 40;           // 类别数上限（现 xmodel 33 类，车上目标 6 类）
constexpr int CH_MAX = 4 * REG_MAX + NC_MAX;
constexpr int N_LEVELS = 3;          // 三个尺度：stride 8 / 16 / 32
#ifndef DETPOST_CAND_MAX
#define DETPOST_CAND_MAX 1024
#endif
constexpr int CAND_MAX = DETPOST_CAND_MAX;  // 过阈值候选上限（超了按分数从低往高丢，见 README §6.3）；硬件核用 256（-DDETPOST_CAND_MAX=256）
constexpr int MAX_DET_MAX = 128;     // 输出框上限（车上 max_det = 100）
constexpr int LUT_N = 256;           // int8 差值/取值的查表长度
constexpr int DIST_FRAC = 10;        // DFL 距离（格为单位）的小数位：Q.10（Q.8 时 60 帧合成场景有 2 帧在 IoU 边界翻转，README §7）
constexpr int COORD_FRAC = 4;        // 输出坐标的小数位：Q4（1/16 像素）

// ---- 运行期配置（硬件里是 AXI-Lite 寄存器） ----
struct Config {
  int nc;                            // 类别数 NC（<= NC_MAX）
  int grid_w[N_LEVELS];              // 按 ultralytics 锚点顺序：P3(80) / P4(40) / P5(20)
  int grid_h[N_LEVELS];
  int stride[N_LEVELS];              // 8 / 16 / 32
  uint32_t offset[N_LEVELS];         // 每个头在输入缓冲里的字节偏移（读自 xmodel ddr_addr）
  int cls_thr_q;                     // 类别 logit 的整数阈值：保留 q >= cls_thr_q（主机由 conf 精确换算）
  int iou_num, iou_den;              // IoU 阈值写成分数 num/den，抑制条件 inter*den > num*union（0.7 = 7/10）
  int max_det;                       // <= MAX_DET_MAX
  uint16_t exp_lut[LUT_N];           // exp_lut[d] = round(2^15 * exp(-d * 2^-fp))，d = 本边最大 logit - 当前 logit
  uint16_t sig_lut[LUT_N];           // sig_lut[q+128] = round(65535 * sigmoid(q * 2^-fp))，仅用于输出分数
  int8_t cls_min_q[NC_MAX];          // NMS 之后的按类下限（车上 backpack 0.35），-128 = 不设
  uint8_t cls_enable[NC_MAX];        // NMS 之后的类别白名单，1 = 输出
};

// ---- 输出记录（硬件里每条 16 字节写回 DDR） ----
struct Box {
  int16_t x1, y1, x2, y2;            // letterbox 640 图像素，Q4
  uint8_t cls;
  int8_t level;                      // 类别 logit 的 int8 原值（排序键）
  uint16_t score;                    // sigmoid 分数 UQ0.16（查表）
  uint16_t anchor;                   // ultralytics 锚点序号（0..8399），便于对照
  uint16_t pad;
};

struct Status {
  int n_cand;                        // 过阈值的候选数（含溢出前）
  int n_cand_kept;                   // 进入排序/NMS 的候选数
  int overflow;                      // 1 = 候选超过 CAND_MAX，按分数截断过
  int final_level;                   // 溢出时抬高后的有效阈值（未溢出 = cls_thr_q）
  int n_nms;                         // NMS 后（截 max_det 前后都 <= max_det）
  int n_out;                         // 白名单/下限之后写出的框数
};

// 参考模型主函数：in 指向 DPU 输出缓冲起点（字节）。返回写出的框数。
int detpost_ref(const int8_t* in, const Config& cfg, Box* out, Status* st);

// ---- 主机侧工具（不综合） ----
// 由浮点阈值换整数 logit 阈值：返回最小的 q，使 sigmoid(q*2^-fp) > conf 严格成立。
int conf_to_q(double conf, int fp);
void make_luts(int fp, Config* cfg);
// 返回最小的 q（-128..127）使 float32 sigmoid(q*2^-fp) >= floor；没有这样的 q 返回 128。
int score_floor_to_q(double floor, int fp);
// 候选门 cls_thr_q = max(conf_to_q(floor), min_c score_floor_to_q(cls_thr[c]))：低于最小类阈值的格子不可能输出，
// 提前丢掉与"先 floor、NMS、max_det 再按类"逐框等价（NMS 按类贪心、max_det 按分数排名，只受分数更高的框影响）。
// off：三个头（P3/P4/P5）在输入缓冲里的字节偏移；gw/gh：网格宽高（方 640：80/40/20；矩 640×480：宽 80/40/20、高 60/30/15）。
void recog_config(Config* cfg, int nc, int fp, double floor, const double* cls_thr, int iou_num, int iou_den,
                  int max_det, const uint32_t off[N_LEVELS], const int gw[N_LEVELS], const int gh[N_LEVELS]);
// 按本工程 xmodel（y8n_33_b1024，sha256 912d295c…）填默认配置：nc=33、fp=2、偏移 194000/38800/0。
void default_config(Config* cfg, int nc, int fp, double conf, int iou_num, int iou_den, int max_det);

}  // namespace detpost
