// 检测后处理核（YOLOv8 DFL 头，DPU int8 输出 → 框表）HLS 顶层接口。
// 语义与定点参考模型 detpost_ref（../detpost_ref.cpp，CAND_MAX=256）逐位相同；前提条件见 README §接口。
#pragma once
#include <ap_int.h>
#include <stdint.h>

typedef ap_uint<128> dp_word_t;

namespace dpk {
constexpr int CMAX = 256;       // 候选上限（参考模型 -DDETPOST_CAND_MAX=256）
constexpr int KMAX = 128;       // max_det 上限
constexpr int NCMAX = 40;       // 类别数上限
// 参数块（主机写进 DDR，核每帧开头读一次；128 位字）：
//   字 [0,32)  ：exp_lut[256]，u16，每字 8 项，项 d 在字 d/8 的第 (d%8)*16 位起
//   字 [32,64) ：sig_lut[256]，同上
//   字 [64,69) ：类别项[40]，u16：低 8 位 = 按类下限 cls_min_q（int8），bit8 = 白名单（1 输出）
constexpr int PRM_WORDS = 69;
constexpr int TAB_N = 552;      // 69 字 × 8 项
constexpr int TAB_SIG = 256;
constexpr int TAB_CLS = 512;
// 输出：out[0..1] 是状态头（8 个 int32：n_cand, n_cand_kept, overflow, final_level, n_nms, n_out, 0, 0），
// out[2 + i] 是第 i 个框（16 字节，布局同 detpost::Box）。
constexpr int OUT_HDR = 2;
constexpr int OUT_WORDS = OUT_HDR + KMAX;
}  // namespace dpk

// in：DPU 输出缓冲（字节地址需 16 字节对齐；三个头的偏移任意，可不对齐）。prm：参数块。out：框表。
// nc：类别数 1..40；thr：类别 logit 整数阈值（主机钳到 [-128,128]）；iou_num/iou_den：1..255；
// max_det：1..128；g80/g40/g20：三个头的网格边长（<=80/40/20，输入 <=640）；off*：各头字节偏移（< 2^20）。
void detpost_top(const dp_word_t* in, const dp_word_t* prm, dp_word_t* out, ap_uint<6> nc, ap_int<9> thr,
                 ap_uint<8> iou_num, ap_uint<8> iou_den, ap_uint<8> max_det, ap_uint<7> g80, ap_uint<7> g40,
                 ap_uint<7> g20, ap_uint<20> off80, ap_uint<20> off40, ap_uint<20> off20);
