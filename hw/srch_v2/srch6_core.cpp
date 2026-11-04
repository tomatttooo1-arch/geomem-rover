// SRCHK6 概率搜索核：一个共用标量运算单元 + 微程序顺序器（微程序 srch6_urom.h，由 uprog6.py 生成），
// 两个专用单元：SCAN（窗口扫描：几何前端、锥体 5 式、遮挡射线、动作，顺序执行，锥体/射线内层 II=1）、
// BEL（信念：质量/视点增益/观测写回，II=2）。功能与 hw/srch/srch_core.cpp（c8）逐字节相同；外部接口同 c8。
// 同一份源码既是 HLS 核也是 C 模型（g++ -I$XILINX_HLS/include，配 hw/srch/srch_tb.cpp 回放向量）。
// 设计：runs/nationals-prelim-20261102/SRCHK6/ARCH.md。
#include "../srch/srch_core.h"
#include "srch6_urom.h"

typedef ap_uint<64> w64;
typedef ap_int<64> s64;

enum { O_HALT, O_ADD, O_SUB, O_AND, O_OR, O_SHL, O_SHR, O_SRA, O_MUL, O_MIN, O_MAX, O_SLT, O_SEQ, O_SEL, O_LD, O_ST,
       O_BEQ, O_BNE, O_BLT, O_BGE, O_JALR, O_DLD, O_DST, O_MFD, O_MTD, O_MTS, O_MFS, O_SCAN, O_BEL, O_XOR };
enum { MD_UNIV = 0, MD_VIEW = 1, MD_OBS = 2 };
enum { BM_MASS = 0, BM_SEL = 1, BM_OBS = 2 };

// ---------------------------------------------------------------- 片上状态
static W72 GRIDM[SR_GRID_DEPTH];   // 4 格一字；RSQ 表 10240..；参数镜像/列历史/EXP 表 11008..（微程序原样字）
static w64 OCC[SR_OCCW];
static ap_uint<40> VISW[4096];     // 节点可见窗行（EXP 表挪到 GRIDM）
static W72 ENTRY[SR_SLOTS / 2];
static w64 PST[16384];             // 2u：DUMP 拍低 64 位（prior|lx|ly 低 8），2u+1：高 64 位（ly 高 8|m0|lq|valid）
static ap_uint<1> SEEN[40960];
static ap_uint<1> REACH[8192];
static w64 RA[32], RB[32];
static w128_t DB;
// SCAN 参数与状态
static w64 s_geo1, s_geo2, s_job0, s_job1, s_vw, s_rq;
static ap_uint<36> s_r2, s_rw2, s_lm, s_cfa;
static w64 QT[8];                  // 分段表：(17,0)=阈值 QD，(40,18)=截距，(63,41)=斜率（MTS 原字）
static ap_uint<4> QN;
static ap_uint<3> s_mode;
static ap_uint<16> s_ptr, s_lim, s_nent;
static ap_uint<36> s_pend;
static ap_uint<8> s_err;
static ap_uint<92> CF[5];
// BEL 参数与状态
static ap_uint<2> b_mode;
static ap_uint<16> b_n, b_base, b_t;
static bool b_h2, b_have;
static ap_int<16> b_cx, b_cy;
static w64 b_acc, b_acc2;
static ap_uint<32> s_status;

#define MSK(n) ((((ap_uint<64>)1) << (n)) - 1)
template <int N> static inline ap_int<64> sxt(ap_uint<64> v) {
#pragma HLS INLINE
  return ((ap_int<64>)(v << (64 - N))) >> (64 - N);
}
static inline ap_uint<18> lane_get(W72 w, ap_uint<2> k) {
#pragma HLS INLINE
  ap_uint<18> a = w(17, 0), b = w(35, 18), c = w(53, 36), d = w(71, 54);
  return k == 0 ? a : (k == 1 ? b : (k == 2 ? c : d));
}
static inline W72 lane_set(W72 w, ap_uint<2> k, ap_uint<18> v) {
#pragma HLS INLINE
  W72 r = w;
  if (k == 0) r(17, 0) = v;
  else if (k == 1) r(35, 18) = v;
  else if (k == 2) r(53, 36) = v;
  else r(71, 54) = v;
  return r;
}

// ---------------------------------------------------------------- 几何前端（c8 geom/qofn 原式，HLS 位宽）
static void geom(ap_uint<36> X, ap_int<18> DX, ap_int<18> DY, ap_uint<18>& n, ap_int<26>& ux, ap_int<26>& uy) {
#pragma HLS INLINE
  ap_uint<3> sc = X < ((ap_uint<36>)1 << 26) ? 0 : (X < ((ap_uint<36>)1 << 28) ? 1 : (X < ((ap_uint<36>)1 << 30) ? 2 :
                  (X < ((ap_uint<36>)1 << 32) ? 3 : (X < ((ap_uint<36>)1 << 34) ? 4 : 5))));
  ap_uint<42> X6 = ((ap_uint<42>)X) << 6;
  ap_uint<42> mw = X6 >> (2 * sc);
  ap_uint<32> m = (ap_uint<32>)mw;
  ap_int<12> ix = (ap_int<12>)((ap_int<16>)(m >> 22) - 256);
  ap_uint<10> idx = ix < 0 ? (ap_uint<10>)0 : (ix > 767 ? (ap_uint<10>)767 : (ap_uint<10>)ix);
  ap_uint<32> Y = GRIDM[SR_RSQ_BASE + idx](31, 0);
  ap_uint<64> yy = (ap_uint<64>)Y * Y;
#pragma HLS BIND_OP variable=yy op=mul impl=dsp latency=3
  ap_uint<32> u = (ap_uint<32>)(yy >> 31);
  ap_uint<64> mu = (ap_uint<64>)m * u;
#pragma HLS BIND_OP variable=mu op=mul impl=dsp latency=3
  ap_uint<34> v = (ap_uint<34>)(mu >> 30);
  ap_uint<34> fac = (ap_uint<34>)((((ap_uint<34>)3) << 31) - v);
  ap_uint<66> yf = (ap_uint<66>)Y * fac;
#pragma HLS BIND_OP variable=yf op=mul impl=dsp latency=3
  ap_uint<32> Y1 = (ap_uint<32>)(yf >> 32);
  ap_uint<37> Rs = ((ap_uint<37>)Y1) << (5 - sc);
  ap_uint<37> R = X == 0 ? (ap_uint<37>)0 : Rs;
  ap_uint<73> xr = (ap_uint<73>)X * R;
#pragma HLS BIND_OP variable=xr op=mul impl=dsp latency=3
  ap_int<56> dxr = (ap_int<56>)DX * (ap_int<56>)R;
#pragma HLS BIND_OP variable=dxr op=mul impl=dsp latency=3
  ap_int<56> dyr = (ap_int<56>)DY * (ap_int<56>)R;
#pragma HLS BIND_OP variable=dyr op=mul impl=dsp latency=3
  n = (ap_uint<18>)(xr >> 48);
  ux = (ap_int<26>)(dxr >> 24);
  uy = (ap_int<26>)(dyr >> 24);
}

static ap_uint<23> qofn(ap_uint<18> n) {
#pragma HLS INLINE
  // 顺序比较：拍 0..7 逐项比 n > QD[i] 计数，拍 8 读选中项（QT 一个读点，LUTRAM）
  ap_uint<4> cnt = 0;
  w64 e = 0;
QOF:
  for (ap_uint<4> i = 0; i < 9; i++) {
#pragma HLS PIPELINE II=1
    ap_uint<3> ix = i[3] ? (cnt == 0 ? (ap_uint<3>)0 : (ap_uint<3>)(cnt - 1)) : (ap_uint<3>)i(2, 0);
    w64 r = QT[ix];
    if (!i[3]) { if (i < QN && n > r(17, 0)) cnt++; }
    else e = r;
  }
  ap_uint<18> d = e(17, 0);
  if (cnt == 0) return (ap_uint<23>)e(40, 18);
  if (cnt >= QN) return 0;
  ap_int<20> dn = (ap_int<20>)((ap_int<20>)n - (ap_int<20>)d);
  ap_int<23> S = (ap_int<23>)e(63, 41);
  ap_int<44> prod = (ap_int<44>)((ap_int<44>)S * dn);
  ap_int<26> q = (ap_int<26>)((ap_int<26>)e(40, 18) + (ap_int<26>)(prod >> 14));
  return q < 0 ? (ap_uint<23>)0 : (ap_uint<23>)q;
}

// ---------------------------------------------------------------- SCAN：一个作业的窗口（c8 eng_a + eng_b + eng_c，顺序执行）
static void scan_unit() {
#pragma HLS INLINE off
  const ap_uint<8> ni = s_geo1(7, 0), nj = s_geo1(23, 16);
  const ap_uint<16> u0 = s_geo1(47, 32), v0 = s_geo1(63, 48);
  const ap_uint<16> mapw = s_geo2(15, 0), maph = s_geo2(31, 16), ou = s_geo2(47, 32), ov = s_geo2(63, 48);
  const ap_uint<12> rq = s_rq(11, 0);
  const ap_int<24> ox = (ap_int<24>)s_job0(23, 0), oy = (ap_int<24>)s_job0(47, 24);
  const ap_int<16> si0 = (ap_int<16>)s_job1(15, 0), sj0 = (ap_int<16>)s_job1(31, 16);
  const ap_uint<13> vrow = s_vw(12, 0), wrow = s_vw(44, 32);
  const ap_uint<6> vbit0 = s_vw(21, 16);
  const ap_uint<2> mode = s_mode(1, 0);
  const bool last = s_mode[2];
  const bool univ_m = mode == MD_UNIV;
  const ap_uint<6> HW = univ_m ? 40 : 33;
  const ap_uint<11> total = univ_m ? 1600 : 1089;
  const ap_uint<2> nh = univ_m ? 0 : 2, nt = (!univ_m && last) ? 2 : 0;
  const ap_uint<12> c_end = nh + total, t_end = c_end + nt, n_it = t_end + (last ? 1 : 0);
  ap_uint<16> ptr = s_ptr, nent = 0;
  ap_uint<36> pend = s_pend;
  ap_uint<8> err = s_err;
  ap_uint<6> row = 0, col = 0;
  ap_uint<40> rowbits = 0;
  const ap_uint<16> cx16 = (ap_uint<16>)(ox >> 5), cy16 = (ap_uint<16>)(oy >> 5);
SCAN_IT:
  for (ap_uint<12> k = 0; k < n_it; k++) {
#pragma HLS LOOP_TRIPCOUNT min=1090 max=1600
    bool emit = false, pad = false;
    ap_uint<36> slot = 0;
    if (k < nh) {
      slot = k == 0 ? (ap_uint<36>)SR_HDR : (ap_uint<36>)(((ap_uint<36>)cy16 << 16) | cx16);
      emit = true;
    } else if (k < c_end) {
      ap_int<17> i = (ap_int<17>)(si0 + row), j = (ap_int<17>)(sj0 + col);
      bool ins = i >= 0 && j >= 0 && i < (ap_int<17>)ni && j < (ap_int<17>)nj;
      ap_uint<16> cell = ins ? (ap_uint<16>)((ap_uint<16>)i * nj + (ap_uint<16>)j) : (ap_uint<16>)0;
      ap_uint<18> g = lane_get(GRIDM[cell >> 2], cell(1, 0));
      bool univ = g[13], fr = g[14];
      ap_int<24> cxp = (ap_int<24>)((((ap_int<24>)u0 + 2 * j + 1)) << 11), cyp = (ap_int<24>)((((ap_int<24>)v0 + 2 * i + 1)) << 11);
      ap_int<25> dx25 = (ap_int<25>)(cxp - ox), dy25 = (ap_int<25>)(cyp - oy);
      bool near = dx25 > -(1 << 17) && dx25 < (1 << 17) && dy25 > -(1 << 17) && dy25 < (1 << 17);
      ap_int<18> DX = (ap_int<18>)dx25, DY = (ap_int<18>)dy25;
      ap_int<36> dx2 = (ap_int<36>)DX * DX;
      ap_int<36> dy2 = (ap_int<36>)DY * DY;
      ap_uint<36> X = (ap_uint<36>)((ap_uint<36>)dx2 + (ap_uint<36>)dy2);
      bool inr = near && X <= s_r2;
      bool pass = ins && near && (univ_m ? (fr && X <= s_rw2) : (univ && inr && X != 0));
      ap_uint<18> n = 0; ap_int<26> ux = 0, uy = 0; ap_uint<23> q = 0; ap_uint<8> K = 0;
      bool cfail = false, blk = false;
      if (pass) {
        geom(X, DX, DY, n, ux, uy);
        q = qofn(n);
        if (mode != MD_VIEW && n > 6144) {
          ap_uint<8> t = (ap_uint<8>)(((ap_uint<18>)(n - 6145)) >> 9);
          K = (ap_uint<8>)((((ap_uint<16>)t * 171) >> 9) + 1);
        }
        if (!univ_m) {
        CONE:
          for (ap_uint<3> m = 0; m < 5; m++) {
#pragma HLS PIPELINE II=2
            ap_uint<92> cf = CF[m];
            ap_int<18> CA = (ap_int<18>)cf(17, 0), CB = (ap_int<18>)cf(35, 18);
            ap_int<28> C0 = (ap_int<28>)cf(63, 36), C1 = (ap_int<28>)cf(91, 64);
            ap_int<37> pax = (ap_int<37>)CA * DX, pby = (ap_int<37>)CB * DY;
            ap_int<37> pbx = (ap_int<37>)CB * DX, pay = (ap_int<37>)CA * DY;
            ap_int<26> al = (ap_int<26>)((ap_int<37>)(pax + pby) >> 11);
            ap_int<26> be = (ap_int<26>)((ap_int<37>)(pbx - pay) >> 11);
            ap_int<29> G = (ap_int<29>)(C0 + al), Gp = (ap_int<29>)(C1 + al);
            ap_int<48> nG = (ap_int<48>)((ap_int<48>)G * (ap_int<48>)n);
            ap_int<48> nGp = (ap_int<48>)((ap_int<48>)Gp * (ap_int<48>)n);
            ap_int<40> ra = (ap_int<40>)((ap_int<40>)al * (ap_int<40>)rq);
            ap_int<40> rb = (ap_int<40>)((ap_int<40>)be * (ap_int<40>)rq);
            ap_int<48> t1 = (ap_int<48>)(nG - ra), t2 = (ap_int<48>)(nG + rb), t3 = (ap_int<48>)(nG - rb), t4 = (ap_int<48>)(nGp + ra);
            if (t1[47] | t2[47] | t3[47] | t4[47]) cfail = true;   // 只要符号位
          }
        }
      RAY:
        for (ap_uint<8> kk = 0; kk < K; kk++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=0 max=40
          ap_int<11> tt = (ap_int<11>)(8 + 3 * kk);
          ap_int<36> px = (ap_int<36>)ox + (ap_int<36>)(((ap_int<36>)ux * tt) >> 15);
          ap_int<36> py = (ap_int<36>)oy + (ap_int<36>)(((ap_int<36>)uy * tt) >> 15);
          ap_int<14> pu = (ap_int<14>)(px >> 11), pv = (ap_int<14>)(py >> 11);
          bool inmap = pu >= 0 && pv >= 0 && pu < (ap_int<18>)mapw && pv < (ap_int<18>)maph;
          ap_int<15> wu = (ap_int<15>)(pu - (ap_int<15>)ou), wv = (ap_int<15>)(pv - (ap_int<15>)ov);
          bool inwin = wu >= 0 && wv >= 0 && wu < 512 && wv < 512;
          ap_uint<12> oa = (ap_uint<12>)((((ap_uint<12>)wv(8, 0)) << 3) | (ap_uint<12>)(wu(8, 0) >> 6));
          w64 ow = OCC[oa];
          bool bit = ow[wu(5, 0)];
          if (!inmap || !inwin || bit) blk = true;
        }
      }
      bool ps = pass && !cfail, vb = ps && !blk;
      // 动作（c8 eng_c）
      ap_uint<40> vw = VISW[(ap_uint<12>)(vrow + row)];
      bool ok = false;
      if (univ_m) {
        ap_uint<40> nr = (ap_uint<40>)((rowbits >> 1) | (((ap_uint<40>)(vb ? 1 : 0)) << 39));
        rowbits = nr;
        if (col == HW - 1) { VISW[(ap_uint<12>)(wrow + row)] = nr; rowbits = 0; }
        if (vb && inr) SEEN[cell] = 1;
      } else if (mode == MD_VIEW) {
        ap_uint<7> vb7 = (ap_uint<7>)(vbit0 + col);
        bool wb = vb7 < 40 && vw[(ap_uint<6>)vb7];
        ok = ps && wb;
        if (ok && q >= (1u << 22)) REACH[g(12, 0)] = 1;
      } else {
        ok = vb;
      }
      if (ok) { emit = true; slot = (ap_uint<36>)(((ap_uint<36>)q << 13) | g(12, 0)); nent++; }
      if (col == HW - 1) { col = 0; row++; } else col++;
    } else if (k < t_end) {
      slot = k == c_end ? (ap_uint<36>)(SR_HDR | (1u << 13)) : (ap_uint<36>)0;
      emit = true;
    } else {
      pad = true;
    }
    // ENTRY 追加（一处写口）：半字指针 ptr，偶半字暂存 pend；末尾补齐只写不计数
    bool ovf = emit && ptr >= s_lim;
    if (ovf) err |= E_EOVER;
    bool we = ((emit && !ovf) || pad) && ptr[0];
    if (we) ENTRY[ptr >> 1] = pad ? (W72)pend : (W72)(((W72)slot << 36) | (W72)pend);
    if (emit && !ovf) {
      if (!ptr[0]) pend = slot;
      ptr++;
    }
  }
  s_ptr = ptr; s_pend = pend; s_nent = nent; s_err = err;
}

// ---------------------------------------------------------------- BEL：信念（c8 belief_step + belief_acc），遇视点结束返回 true
static bool bel_unit() {
#pragma HLS INLINE off
  const ap_uint<2> mode = b_mode;
  const ap_uint<16> n = b_n, base = b_base;
  const ap_uint<31> f = s_rq(62, 32);
  const ap_uint<36> lm = s_lm;
  ap_uint<16> t = b_t;
  bool h2 = b_h2, have = b_have, done = false;
  ap_int<16> cx = b_cx, cy = b_cy;
  w64 acc = b_acc, acc2 = b_acc2;
BEL_L:
  while (t < n) {
#pragma HLS PIPELINE II=4
#pragma HLS DEPENDENCE variable=PST inter false
#pragma HLS LOOP_TRIPCOUNT min=1 max=32000
    ap_uint<16> sl = (ap_uint<16>)(base + t);
    W72 ew = ENTRY[sl >> 1];
    ap_uint<36> slot = sl[0] ? (ap_uint<36>)ew(71, 36) : (ap_uint<36>)ew(35, 0);
    bool isent = false, ishdr = false;
    ap_uint<13> u = 0; ap_uint<23> q = 0;
    if (mode == BM_MASS) {
      isent = true; u = (ap_uint<13>)t;
    } else if (h2) {
      cx = (ap_int<16>)slot(15, 0); cy = (ap_int<16>)slot(31, 16); h2 = false;
    } else if (slot(12, 0) == SR_HDR) {
      ishdr = true; h2 = true;
    } else {
      isent = true; u = slot(12, 0); q = slot(35, 13);
    }
    w64 PA = PST[((ap_uint<14>)u) << 1], PB = PST[(((ap_uint<14>)u) << 1) | 1];
    bool rch = REACH[u];
    ap_uint<40> prior = PA(39, 0);
    ap_int<16> lx = (ap_int<16>)PA(55, 40);
    ap_int<16> ly = (ap_int<16>)(((ap_uint<16>)PB(7, 0) << 8) | (ap_uint<16>)PA(63, 56));
    ap_uint<31> m0 = PB(38, 8);
    ap_uint<23> lq = PB(61, 39);
    bool valid = PB[62];
    ap_uint<31> fm0 = m0 < f ? m0 : f;
    ap_uint<55> pm1 = (ap_uint<55>)m0 * (ap_uint<24>)((((ap_uint<24>)1) << 23) - lq);
    ap_uint<31> a1 = (ap_uint<31>)(pm1 >> 23);
    ap_uint<31> mc = a1 > fm0 ? a1 : fm0;
    ap_int<17> dx = (ap_int<17>)(lx - cx), dy = (ap_int<17>)(ly - cy);
    ap_int<34> sdx = (ap_int<34>)dx * dx;
    ap_int<34> sdy = (ap_int<34>)dy * dy;
    bool same = valid && (ap_uint<36>)((ap_uint<36>)sdx + (ap_uint<36>)sdy) < lm;
    ap_uint<31> bsv = same ? m0 : mc;
    ap_uint<23> qq = same ? (q > lq ? q : lq) : q;
    ap_uint<31> fb = bsv < f ? bsv : f;
    ap_uint<55> pm2 = (ap_uint<55>)bsv * (ap_uint<24>)((((ap_uint<24>)1) << 23) - qq);
    ap_uint<31> a2 = (ap_uint<31>)(pm2 >> 23);
    ap_uint<31> mn = a2 > fb ? a2 : fb;
    ap_uint<31> dm = mode == BM_MASS ? mc : (ap_uint<31>)(mc - mn);
    ap_uint<71> pdm = (ap_uint<71>)prior * dm;
#pragma HLS BIND_OP variable=pdm op=mul impl=dsp latency=3
    ap_uint<50> t1 = (ap_uint<50>)(pdm >> 21);
    ap_uint<49> t2 = mode == BM_MASS ? ((rch && mc > (1u << 29)) ? (ap_uint<49>)(((ap_uint<49>)prior) << 9) : (ap_uint<49>)0)
                                     : ((q >= (1u << 22) && mc > (1u << 29)) ? (ap_uint<49>)1 : (ap_uint<49>)0);
    if (isent && mode == BM_OBS) {
      w64 nb = 0;
      nb(7, 0) = same ? (ap_uint<8>)PB(7, 0) : (ap_uint<8>)((ap_uint<16>)cy)(15, 8);
      nb(38, 8) = bsv; nb(61, 39) = qq; nb[62] = 1;
      PST[(((ap_uint<14>)u) << 1) | 1] = nb;
      if (!same) {
        w64 na = 0;
        na(39, 0) = prior; na(55, 40) = (ap_uint<16>)cx; na(63, 56) = ((ap_uint<16>)cy)(7, 0);
        PST[((ap_uint<14>)u) << 1] = na;
      }
    }
    // SEL：遇表头且已有视点 → 停在该表头（t 不前进、have 清零），acc/acc2 即该视点结果，下次调用从表头重来
    bool dn = mode == BM_SEL && ishdr && have;
    if (!dn) t++;
    if (mode != BM_OBS && ishdr) {
      if (dn) { have = false; h2 = false; done = true; }
      else { have = true; acc = 0; acc2 = 0; }
    } else if (mode != BM_OBS && isent) {
      acc += t1; acc2 += t2;
    }
    if (done) break;
  }
  b_t = t; b_h2 = h2; b_have = have; b_cx = cx; b_cy = cy; b_acc = acc; b_acc2 = acc2;
  return done;
}

// ---------------------------------------------------------------- 顺序器
#ifndef __SYNTHESIS__
#include <stdio.h>
unsigned long long g_steps = 0;   // C 模型统计：微指令条数（拍数估计用）
#endif
// 按 op 查的常量 ROM（每项一个 5 输入 LUT）
static const bool WRT[32] = {   // 写回 rd
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,   // HALT ADD SUB AND OR SHL SHR SRA MUL MIN MAX SLT SEQ SEL LD ST
  0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0 }; // BEQ BNE BLT BGE JALR DLD DST MFD MTD MTS MFS SCAN BEL XOR
static const bool SUBT[32] = {  // 加法器做减法
  0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0,
  1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static void seq(w128_t* gmem, ap_uint<32> prm) {
#pragma HLS INLINE off
  ap_uint<10> pc = 0;
  bool FR = false;
  DB = (w128_t)prm;
#ifndef __SYNTHESIS__
  unsigned long long steps = 0;
#endif
SEQ:
  while (true) {
#pragma HLS PIPELINE off
#ifndef __SYNTHESIS__
    if (++steps > 400000000ULL) { printf("SEQ runaway at pc %d\n", (int)pc); break; }
    g_steps++;
#endif
    ap_uint<48> w = UROM[pc];
    ap_uint<5> op = w(4, 0);
    bool useimm = w[5];
    ap_uint<5> rd = w(10, 6), ra = w(15, 11), rb = w(20, 16);
    ap_int<27> imm27 = w(47, 21);
    w64 imm = (w64)(s64)imm27;
    w64 a = RA[ra], bv = RB[rb];
    w64 b = useimm ? imm : bv;
    bool sub = SUBT[op];
    w64 bx = sub ? (w64)~b : b;
    ap_uint<65> s65 = (((ap_uint<65>)a) << 1 | 1) + (((ap_uint<65>)bx) << 1 | (ap_uint<65>)sub);
    w64 sum = s65(64, 1);
    bool neg = sum[63], zero = sum == 0;
    ap_uint<6> amt = b(5, 0);
    ap_uint<18> addr = sum(17, 0);
    ap_uint<4> reg = addr(17, 14);
    bool pick_a = (op == O_MIN) ? neg : (op == O_MAX) ? !neg : FR;
    // 逻辑单元：每位一个 4 项真值表（下标 {a_i,b_i}），AND/OR/XOR 与 MIN/MAX/SEL 的取 a/取 b 共用
    ap_uint<4> T = op == O_AND ? (ap_uint<4>)8 : op == O_OR ? (ap_uint<4>)14 : op == O_XOR ? (ap_uint<4>)6 :
                   pick_a ? (ap_uint<4>)12 : (ap_uint<4>)10;
    w64 lu;
    for (int i = 0; i < 64; i++) {
#pragma HLS UNROLL
      lu[i] = T[(ap_uint<2>)((a[i] ? 2 : 0) | (b[i] ? 1 : 0))];
    }
    // 一个右移器兼做左移：左移 = 位反转后右移再反转
    bool isl = op == O_SHL;
    w64 arev, ain;
    for (int i = 0; i < 64; i++) {
#pragma HLS UNROLL
      arev[i] = a[63 - i];
    }
    ain = isl ? arev : a;
    ap_int<65> sx = ((ap_int<65>)(op == O_SRA && a[63]) << 64) | (ap_int<65>)ain;
    w64 shr0 = (w64)(sx >> amt), shrev;
    for (int i = 0; i < 64; i++) {
#pragma HLS UNROLL
      shrev[i] = shr0[63 - i];
    }
    w64 shv = isl ? shrev : shr0;
    ap_int<36> ma = (s64)a, mb = (s64)b;
    w64 mul = (w64)(s64)(ma * mb);
    bool flag = (op == O_SLT) ? neg : zero;
    w64 narrow = (op == O_JALR) ? (w64)(pc + 1) : (w64)flag;
    w64 xv = 0;   // LD / MFD / MFS / BEL 的结果（同一时刻只一个有效）
    ap_uint<10> npc = pc + 1;
    switch (op) {
      case O_SLT: case O_SEQ: FR = flag; break;
      case O_LD: case O_ST: {
        ap_uint<14> ga = reg == 0 ? (ap_uint<14>)addr(15, 2) : (ap_uint<14>)addr(13, 0);
        if (op == O_LD) {
          if (reg == 0 || reg == 4) {
            W72 gw = GRIDM[ga];
            xv = reg == 0 ? (w64)lane_get(gw, addr(1, 0)) : (w64)gw(63, 0);
          } else if (reg == 5) xv = ENTRY[addr(13, 0)](63, 0);
          else if (reg == 7) xv = PST[addr(13, 0)];
          else if (reg >= 8 && reg <= 10) xv = (w64)SEEN[addr(15, 0)];
        } else {
          if (reg == 0 || reg == 4) {
            W72 go = GRIDM[ga], gb = (W72)bv, gn;
            ap_uint<2> ln = addr(1, 0);
            gn(17, 0) = reg != 0 ? gb(17, 0) : (ln == 0 ? bv(17, 0) : go(17, 0));
            gn(35, 18) = reg != 0 ? gb(35, 18) : (ln == 1 ? bv(17, 0) : go(35, 18));
            gn(53, 36) = reg != 0 ? gb(53, 36) : (ln == 2 ? bv(17, 0) : go(53, 36));
            gn(71, 54) = reg != 0 ? gb(71, 54) : (ln == 3 ? bv(17, 0) : go(71, 54));
            GRIDM[ga] = gn;
          } else if (reg == 5) ENTRY[addr(13, 0)] = (W72)bv;
          else if (reg == 6) OCC[addr(11, 0)] = bv;
          else if (reg == 7) PST[addr(13, 0)] = bv;
          else if (reg >= 8 && reg <= 10) SEEN[addr(15, 0)] = bv[0];
          else if (reg == 11) REACH[addr(12, 0)] = bv[0];
        }
        break;
      }
      case O_BEQ: if (zero) npc = imm(9, 0); break;
      case O_BNE: if (!zero) npc = imm(9, 0); break;
      case O_BLT: if (neg) npc = imm(9, 0); break;
      case O_BGE: if (!neg) npc = imm(9, 0); break;
      case O_JALR: npc = sum(9, 0); break;
      case O_DLD: DB = gmem[sum(31, 4)]; break;
      case O_DST: gmem[sum(31, 4)] = DB; break;
      case O_MFD: xv = imm[0] ? (w64)DB(127, 64) : (w64)DB(63, 0); break;
      case O_MTD: if (imm[0]) DB(127, 64) = a; else DB(63, 0) = a; break;
      case O_MTS: {
        ap_uint<6> tg = imm(5, 0);
        if (tg == 0) s_geo1 = a;
        else if (tg == 1) s_geo2 = a;
        else if (tg == 2) s_r2 = a(35, 0);
        else if (tg == 3) s_rw2 = a(35, 0);
        else if (tg == 4) s_rq = a;
        else if (tg == 5) s_lm = a(35, 0);
        else if (tg == 6) QN = a(3, 0);
        else if (tg >= 7 && tg <= 14) QT[tg - 7] = a;
        else if (tg == 15) s_job0 = a;
        else if (tg == 16) s_job1 = a;
        else if (tg == 17) s_vw = a;
        else if (tg == 18) s_mode = a(2, 0);
        else if (tg == 19) s_ptr = a(15, 0);
        else if (tg == 20) s_pend = a(35, 0);
        else if (tg == 21) s_lim = a(15, 0);
        else if (tg == 22) s_err = a(7, 0);
        else if (tg == 23) s_cfa = a(35, 0);
        else if (tg >= 24 && tg <= 28) {
          ap_uint<92> c = 0;
          c(35, 0) = s_cfa; c(63, 36) = a(27, 0); c(91, 64) = a(59, 32);
          CF[tg - 24] = c;
        }
        else if (tg == 29) b_mode = a(1, 0);
        else if (tg == 30) b_n = a(15, 0);
        else if (tg == 31) b_base = a(15, 0);
        else if (tg == 32) { b_t = 0; b_h2 = false; b_have = false; b_cx = 0; b_cy = 0; b_acc = 0; b_acc2 = 0; }
        else if (tg == 33) s_status = a(31, 0);
        break;
      }
      case O_MFS: {
        ap_uint<3> sr = imm(2, 0);
        xv = sr == 0 ? (w64)s_ptr : sr == 1 ? (w64)s_pend : sr == 2 ? (w64)s_nent : sr == 3 ? (w64)s_err :
              sr[0] ? b_acc2 : b_acc;
        break;
      }
      case O_SCAN: scan_unit(); break;
      case O_BEL: xv = (w64)bel_unit(); break;
      default: break;
    }
    w64 v;
    switch (op) {
      case O_ADD: case O_SUB: v = sum; break;
      case O_AND: case O_OR: case O_XOR: case O_MIN: case O_MAX: case O_SEL: v = lu; break;
      case O_SHL: case O_SHR: case O_SRA: v = shv; break;
      case O_MUL: v = mul; break;
      case O_LD: case O_MFD: case O_MFS: case O_BEL: v = xv; break;
      default: v = narrow; break;
    }
    if (WRT[op]) { RA[rd] = v; RB[rd] = v; }
    if (op == O_HALT) break;
    pc = npc;
  }
}

void srch_top(w128_t* gmem, ap_uint<64> prm, ap_uint<32>& status) {
#pragma HLS INTERFACE m_axi port=gmem offset=off bundle=gmem depth=24576 max_read_burst_length=1 max_write_burst_length=1 num_read_outstanding=1 num_write_outstanding=1 latency=1
#pragma HLS INTERFACE s_axilite port=prm
#pragma HLS INTERFACE s_axilite port=status
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS BIND_STORAGE variable=GRIDM type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=OCC type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=VISW type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=ENTRY type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=PST type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=SEEN type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=REACH type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=RA type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=RB type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=QT type=ram_1p impl=lutram
#pragma HLS BIND_STORAGE variable=CF type=ram_1p impl=lutram
  seq(gmem, (ap_uint<32>)prm);
  status = s_status;
}
