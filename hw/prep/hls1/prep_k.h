// PREP1 识别前处理核（彩色帧 -> DPU int8 输入张量），HLS 实现。设计：runs/nationals-prelim-20261102/PREP0/DESIGN.md
// 只缩小或等大（cv2 8U INTER_LINEAR 定点）、灰边 114、仿射量化（DSP，等价 256 项表）；RGB888/BGR888 旁路，YUYV/UYVY 经可配色彩矩阵转换。
#pragma once
#include <ap_int.h>
#include <hls_stream.h>

typedef ap_uint<128> word_t;   // m_axi 数据宽度：16 字节/拍
typedef ap_uint<36> tab_t;     // 一项缩放表：[11:0] x1（或 y1）、[23:12] c0、[35:24] c1（Q11）

#define PREP_MAX_OUT 640       // 内容区/张量宽高上限
#define PREP_MAX_SRC 2048      // 源宽高上限
#define PREP_HDR_WORDS 3       // 参数块帧头字数
#define PREP_PRM_WORDS (PREP_HDR_WORDS + 640)
#define PREP_MAGIC 0x5052       // 帧头字 0 [15:0]
#define PREP_VERSION8 0x02      // STATUS[15:8]

// 源格式（帧头字 0 [61:60]）
#define PREP_FMT_RGB 0
#define PREP_FMT_BGR 1
#define PREP_FMT_YUYV 2
#define PREP_FMT_UYVY 3

// STATUS（只读）：[31:16] 帧计数（成功处理的帧）、[15:8] 版本 0x02、[7:0] 错误位
#define PREP_ST_ERR_HDR 0x1     // 帧头非法（魔数错、OW/OH 为 0 或大于源、装载字数 >640）：本帧未处理、未写任何字节

// 参数块（DDR，128 位字，16 字节对齐；A53 驱动 prep_drv.h 生成，几何或模型变了才重写，核每帧都读）：
//   字 0：[15:0] 魔数 0x5052  [27:16] W0  [39:28] H0  [49:40] OW  [59:50] OH  [61:60] 格式（0 RGB 1 BGR 2 YUYV 3 UYVY）
//         [62] 色度三角上采样  [63] 不缩放旁路  [71:64] 灰边 int8 码  [83:72] 源行步长（16 字节字）  [95:84] 每行有效字数
//         [112:96] 张量字数 TW·TH·3/16  [122:113] 表项装载字数 max(OW,OH)
//   字 1：[22:0] 读源字数 H0·步长字  [45:23] 源像素数 W0·H0  [68:46] 齿轮箱拍数 H0·(W0 或 YUYV 时 W0+2)
//         [89:69] H0·OW  [110:90] 上灰边字节数 (top·TW+left)·3
//   字 2：[20:0] 行间灰边字节数 (TW−OW)·3  [41:21] 下灰边字节数 (bottom·TW+right)·3
//         [65:42] 量化 K = 32897·2^(fp+1)  [89:66] 量化 C = 255·32897（q = min(127, (x·K+C)>>24)）
//   字 3+d（d < 装载字数）：[35:0] x 表第 d 项（d < OW）  [71:36] y 表第 d 项（d < OH）  （[79:72] 旧量化表位，核已不读）
//         [99:80] 色彩系数第 d 个（d < 6，有符号 20 位）：0 KY, 1 YOFF, 2 KRV, 3 KGU, 4 KGV, 5 KBU
void prep_top(const word_t* src, word_t* dst, const word_t* prm, ap_uint<32>* status);
