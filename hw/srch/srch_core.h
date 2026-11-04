// SRCHK0 概率搜索核：单一源码。SRCH_SW 定义时是纯整数 C++（定点 C 模型、A53 同算法版），
// 否则是 Vitis HLS 2022.2 源（ap_int）。设计见 runs/nationals-prelim-20261102/SRCHK0/DESIGN.md、HANDOVER.md。
// 两版用同一套表达式；HLS 版按声明位宽截断，SW 版全 64 位（宽积用 __int128），csim 对 SW 逐位比对抓位宽错。
#pragma once
#include <stdint.h>

#ifdef SRCH_SW
typedef unsigned __int128 w128_t;
typedef unsigned __int128 W72;
#define UT(n) uint64_t
#define ST(n) int64_t
#define WU(n) unsigned __int128
#define WS(n) __int128
#else
#include <ap_int.h>
typedef ap_uint<128> w128_t;
typedef ap_uint<72> W72;
#define UT(n) ap_uint<n>
#define ST(n) ap_int<n>
#define WU(n) ap_uint<n>
#define WS(n) ap_int<n>
#endif

// ---- 操作码 ----
enum {
  OP_INIT_MAP = 1,   // 装 OCC 窗口、裁剪区粗格字节（暂存 ENTRY）、RSQ/EXP 表；R-A 栅格（块归并、两次膨胀、编码）
  OP_INIT_UNIV = 3,  // 近节点圈（GRID 读改写）＋每节点可见窗射线 -> VISWIN、SEEN
  OP_INIT_FINAL = 4, // R-B 压缩编号（光栅序）＋ 200x200 代价图输出 ＋ 节点最近可走格
  OP_INIT_VIEWS = 5, // 清 REACH；逐视点扫描建 ENTRY（视点项）与 REACH
  OP_RESET = 6,      // 信念置初值
  OP_PRIOR0 = 7,     // 先验：到记忆位置的最小 d^2
  OP_PRIOR1 = 8,     // 先验：三个和
  OP_PRIOR2 = 9,     // 先验：写每格先验
  OP_OBSERVE = 10,   // 一帧观测（视场＋遮挡射线＋信念写回）
  OP_CELLOF = 11,    // 一点的最近可走格
  OP_SELECT = 12,    // 剩余质量、未看质量、全部视点增益与新面积
  OP_DUMP = 13       // 每可见域格导出 P|M（界面热图、联仿全量比对）
};

// ---- 容量 ----
#define SR_MAXN 198
#define SR_MINN 8
#define SR_GRIDW 10240       // GRID 字（4 格一字）上限，其后是 RSQ 表
#define SR_RSQ_BASE 10240    // GRID URAM 空闲字：768 项 1/sqrt 初值
#define SR_GRID_DEPTH 12288
#define SR_UMAX 8191         // 可见域编号 0..8190（0x1FFF 作视点头标记）
#define SR_VIEWS 640
#define SR_SLOTS 32768       // ENTRY 半字（36 位）数
#define SR_ENTRY_OBS 31744   // 观测临时区起点
#define SR_VISW_BASE_EXP 3328// VISWIN URAM 空闲字：512 项 2^-f 表
#define SR_OCCW 4096
#define SR_REACH 40960       // FLAGS 中 REACH 区起点（SEEN 区为 0..39999）
#define SR_FLAGS 49152
#define SR_HDR 0x1FFF

// ---- 参数块（u64 下标；PS 每次调用写全）----
enum {
  P_HDR = 0,   // magic16 | ver8<<16 | op8<<24 | nseg4<<32
  P_RES = 1,   // 结果块字节地址（16 对齐）
  P_AUX = 2,   // 作业表 / OCC 位图
  P_AUX2 = 3,  // 裁剪区粗格字节（INIT_MAP）/ 代价图输出（INIT_FINAL）/ 导出（DUMP）
  P_GEO1 = 4,  // ni16 | nj16<<16 | u0 16<<32 | v0 16<<48
  P_GEO2 = 5,  // mapW16 | mapH16<<16 | ou16<<32 | ov16<<48
  P_CNT = 6,   // U16 | 视点区半字数16<<16 | 观测区半字数16<<32 | 作业数16<<48
  P_R2 = 7,    // (1.6 m)^2，Q.22 像素^2
  P_RW2 = 8,   // (1.6 m + 相机前移 + 1e-6)^2
  P_RN2 = 9,   // (0.3 m)^2
  P_LM = 10,   // (同看 0.5 m)^2，Q.12（位置 Q.6）
  P_RQ = 11,   // 物体半径 r Q.11 | 下限 f Q.30 << 32
  P_QT = 12,   // 12..19：q 分段 D18 | Q23<<18 | S23(有符号)<<41
  P_ORG = 20,  // 先验记忆位置 Tx | Ty<<32（Q.11 有符号 32）
  P_K = 21,    // K1 | K2<<32（log2e/(2σ_px^2)·2^32）
  P_I0 = 22,   // I1 | I2<<32
  P_CM = 23, P_CN = 24, P_CW = 25, P_CC = 26,  // 先验系数 Q.39
  P_TAB = 37   // RSQ 768 u64 ＋ EXP 512 u64 的字节地址（INIT_MAP）
};
#define PB_BEATS 19
#define SR_MAGIC 0x5352u
#define SR_VERSION 2u

// 作业描述（12 个 u64 = 6 拍）：
// [0] ox24 | oy24<<24 | node16<<48（原点 Q.11 有符号：节点或相机地面点）
// [1] si0 | sj0<<16 | vi0<<32 | vj0<<48（扫描窗与节点可见窗起点，有符号 16）
// [2+2m] A18 | B18<<18（有符号）；[3+2m] F0 28 | F0' 28 <<32（有符号），m = 0..4
#define JOB_U64 12

// 结果块（u64 下标）
enum { R_STAT = 0, R_AUX = 1, R_MASS = 2, R_UNSEEN = 3, R_SM = 4, R_SN = 5, R_SW = 6, R_MIN = 7,
       R_NODECELL = 16, R_VIEW = 96 };   // 视点 v：R_VIEW+2v 增益 Q.48，+1 新面积格数
#define R_U64 (96 + 2 * 640)

// 错误位
enum { E_MAGIC = 1, E_UOVER = 2, E_EOVER = 4, E_OOVER = 8, E_SIZE = 16, E_WIN = 32 };

#ifdef SRCH_SW
#ifdef __cplusplus
extern "C" {
#endif
uint32_t srch_run_sw(uint8_t* ddr, uint64_t prm);
// 测试用：GRID 字段（每格 uidx | univ<<13 | free<<14 | trav<<15 | nocc<<16 | edge<<17）
void srch_dump_grid(uint32_t* out, uint32_t n);
// 测试用：ENTRY 半字 [s0, s0+n)
void srch_dump_slots(uint64_t* out, uint32_t s0, uint32_t n);
#ifdef __cplusplus
}
#endif
#else
void srch_top(w128_t* gmem, ap_uint<64> prm, ap_uint<32>& status);
#endif
