// SRCHK0 概率搜索核（单一源码，见 srch_core.h 头注释）。
// 结构：一个"窗口扫描引擎"（A 进程扫格＋几何前端＋锥体判定，B 进程走遮挡射线并执行动作，dataflow），
// 一个"信念引擎"（质量求和 / 视点增益 / 观测写回，II=1），以及 R-A、R-B、近节点圈、代价图、最近格、先验、装载等小循环。
// 每个函数只在一处调用；所有大表常驻片上（URAM 13 块，BRAM 两个小数组）。
#include "srch_core.h"

#ifdef SRCH_SW
#include <string.h>
// SW 版流：定长数组（窗口引擎每次调用 ≤1600 项；信念引擎 SW 版不走流，见 belief()）
template <class T> struct Stream {
  T buf[2048]; int h = 0, t = 0;
  void write(const T& v) { buf[t++] = v; }
  T read() { return buf[h++]; }
};
#else
#include <hls_stream.h>
template <class T> using Stream = hls::stream<T>;
#endif

// ---------------- 小工具（两版同式） ----------------
#define MSK(n) ((((UT(64))1) << (n)) - 1)
#define FLD(x, lo, n) (((UT(64))((x) >> (lo))) & MSK(n))
// 符号扩展：位宽 N 是模板常数（SRCHK3：原先 n 是函数参数，HLS 不内联时每个调用点各生成一个 64 位桶形移位器，约 370 LUT）
template <int N> static inline ST(64) sxt(UT(64) v) {
#ifndef SRCH_SW
#pragma HLS INLINE
#endif
  return ((ST(64))(v << (64 - N))) >> (64 - N);
}
// GRID 4 格一字：取/置第 k 格的 18 位（SRCHK3：显式 4 选 1，替代 72 位变量移位）
static inline UT(18) lane_get(W72 w, UT(2) k) {
#ifndef SRCH_SW
#pragma HLS INLINE
#endif
  UT(18) a = (UT(18))FLD(w, 0, 18), b = (UT(18))FLD(w, 18, 18), c = (UT(18))FLD(w, 36, 18), d = (UT(18))FLD(w, 54, 18);
  return k == 0 ? a : (k == 1 ? b : (k == 2 ? c : d));
}
static inline W72 lane_set(W72 w, UT(2) k, UT(18) v) {
#ifndef SRCH_SW
#pragma HLS INLINE
#endif
  W72 m0 = (W72)MSK(18);
  if (k == 0) return (W72)((w & ~m0) | (W72)v);
  if (k == 1) return (W72)((w & ~(m0 << 18)) | ((W72)v << 18));
  if (k == 2) return (W72)((w & ~(m0 << 36)) | ((W72)v << 36));
  return (W72)((w & ~(m0 << 54)) | ((W72)v << 54));
}
// 取一位（HLS 用位下标 = 多路选择，不生成移位器）
#ifdef SRCH_SW
#define BIT(w, i) ((bool)(((w) >> (i)) & 1))
#else
#define BIT(w, i) ((bool)((w)[(i)]))
#endif

// SRCHK5：数据通路乘法器逐个绑定 DSP、延迟 4（全局 config_op mul -latency 4 会让联仿在 INIT_UNIV 挂起，见 SRCHK5/README）。
// 只绑宽乘法（c5 OOC 里在 fabric 拼部分积的那些）；下标/地址类小乘法保持工具默认。
// c8：c7 OOC 实测绑定无效（9,901 LUT，比 c5 多 365：宽乘法的 fabric 加法仍在、深流水多出 1,493 个 SRL），关掉；
// 宽乘法的 LUT 只能靠把操作数截到 DSP 原生 27x18 / 26x17 以内去掉。打开：-DSRCH_MULB。
#if !defined(SRCH_SW) && defined(SRCH_MULB)
#define PRAGMA_(x) _Pragma(#x)
#define MULB(v) PRAGMA_(HLS BIND_OP variable=v op=mul impl=dsp latency=4)
#else
#define MULB(v)
#endif

// DDR 字节地址按 32 位算（SRCHK3：KV260 低端 DDR 0..2 GB，驱动的缓冲都在 4 GB 以下；64 位地址加法每处约 70 LUT）
#define AD UT(32)

enum { MD_UNIV = 0, MD_VIEW = 1, MD_OBS = 2 };
enum { BM_MASS = 0, BM_SEL = 1, BM_OBS = 2 };

// ---------------- 片上状态 ----------------
static W72 GRIDM[SR_GRID_DEPTH];  // 4 格一字 + RSQ 表
static UT(64) OCC[SR_OCCW];       // 512x512 位占据窗口
static W72 VISW[4096];            // 节点可见窗 80x40 行（40 位）+ EXP 表
static W72 ENTRY[SR_SLOTS / 2];   // 视点项（两个 36 位半字一字）；INIT_MAP 时暂存裁剪区字节
static W72 PST[8192];             // prior Q.39 (40) | lx Q.6 (16) | ly Q.6 (16)
static W72 MST[8192];             // m0 Q.30 (31) | lq Q.23 (23) <<31 | valid <<54
static UT(1) FLAGS[SR_FLAGS];     // SEEN[0..39999] | REACH[40960..]
static UT(12) DIL[256];           // R-A 列历史

struct Cst {
  UT(8) ni, nj; UT(16) u0, v0, mapw, maph, ou, ov;
  UT(36) r2, rw2; UT(12) rq;
};
// SRCHK4：锥体判定挪到 B 进程按式时分（每候选格 5 拍），Item 带 DX/DY/n 与"需锥体判定"位；pass 为锥体前的预通过。
struct Item { UT(16) cell; UT(13) uidx; UT(1) pass, seen, cand; UT(8) K; ST(26) ux, uy; UT(23) q; ST(18) DX, DY; UT(18) n; };
struct Cf { ST(18) a, b; ST(28) f0, f1; };
struct Hdr { ST(24) ox, oy; UT(13) vrow; UT(6) vbit0; UT(13) wrow; ST(16) cx6, cy6; };

// ---------------- 几何前端 ----------------
// X（Q.22 像素^2）-> R = 2^48/sqrt(X)（X 按整数看），n = sqrt(X)（Q.11），用表初值＋一次牛顿，无除法器。
static void geom(UT(36) X, ST(18) DX, ST(18) DY, UT(18)& n, ST(26)& ux, ST(26)& uy) {
  // SRCHK4：m = (X<<6) >> s2，s2 = 6 - sh，sh = 2*floor((31-e)/2)，e = floor(log2 X)，X<2^24 时按 e=24。
  // 原先 36 位前导零循环＋区间判断，改成 5 个常数门限比较（e<=25 <=> X<2^26，依此类推），逐位等价。
  UT(3) sc = X < ((UT(36))1 << 26) ? 0 : (X < ((UT(36))1 << 28) ? 1 : (X < ((UT(36))1 << 30) ? 2 :
             (X < ((UT(36))1 << 32) ? 3 : (X < ((UT(36))1 << 34) ? 4 : 5))));
  UT(42) X6 = ((UT(42))X) << 6;
  UT(42) mw = sc == 0 ? X6 : (sc == 1 ? (UT(42))(X6 >> 2) : (sc == 2 ? (UT(42))(X6 >> 4) :
              (sc == 3 ? (UT(42))(X6 >> 6) : (sc == 4 ? (UT(42))(X6 >> 8) : (UT(42))(X6 >> 10)))));
  UT(32) m = (UT(32))mw;
  ST(12) ix = (ST(12))((ST(16))(m >> 22) - 256);
  UT(10) idx = ix < 0 ? (UT(10))0 : (ix > 767 ? (UT(10))767 : (UT(10))ix);
  UT(32) Y = (UT(32))FLD(GRIDM[SR_RSQ_BASE + idx], 0, 32);
  UT(64) yy = (UT(64))Y * Y;
  MULB(yy)
  UT(32) u = (UT(32))(yy >> 31);
  UT(64) mu = (UT(64))m * u;
  MULB(mu)
  UT(34) v = (UT(34))(mu >> 30);
  UT(34) fac = (UT(34))((((UT(34))3) << 31) - v);
  UT(66) yf = (UT(66))Y * fac;
  MULB(yf)
  UT(32) Y1 = (UT(32))(yf >> 32);
  // R = Y1 << (sh/2 + 2)，sh/2 + 2 = (10 - s2)/2
  // R = Y1 << (5 - sc)：6 选 1 常数移位
  UT(37) Rs = sc == 0 ? (UT(37))((UT(37))Y1 << 5) : (sc == 1 ? (UT(37))((UT(37))Y1 << 4) : (sc == 2 ? (UT(37))((UT(37))Y1 << 3) :
              (sc == 3 ? (UT(37))((UT(37))Y1 << 2) : (sc == 4 ? (UT(37))((UT(37))Y1 << 1) : (UT(37))Y1))));
  UT(37) R = X == 0 ? (UT(37))0 : Rs;
  WU(73) xr = (WU(73))X * R;
  MULB(xr)
  ST(56) dxr = (ST(56))DX * (ST(56))R;
  MULB(dxr)
  ST(56) dyr = (ST(56))DY * (ST(56))R;
  MULB(dyr)
  n = (UT(18))(xr >> 48);
  ux = (ST(26))(dxr >> 24);
  uy = (ST(26))(dyr >> 24);
}

// q(n)：分段线性，cnt = #{i : n > D_i}。SRCHK4：表放片上（断点 QD 8 个寄存器，端值|斜率 QE 小 LUTRAM），
// 不再把 8 个 64 位表项当标量穿过 dataflow（c3 的 FIFO 1,004 LUT 估计大半来自这里）。
static UT(18) QD[8];
static UT(46) QE[8];   // Q23 | S23<<23
static UT(4) QN;
static UT(23) qofn(UT(18) n) {
  UT(4) cnt = 0;
  for (int i = 0; i < 8; i++) {
#ifndef SRCH_SW
#pragma HLS UNROLL
#endif
    if (i < QN && n > QD[i]) cnt++;
  }
  UT(3) ix = cnt == 0 ? (UT(3))0 : (UT(3))(cnt - 1);
  UT(46) e = QE[ix];
  UT(18) d = QD[ix];
  if (cnt == 0) return (UT(23))FLD(e, 0, 23);
  if (cnt >= QN) return 0;
  ST(20) dn = (ST(20))((ST(20))n - (ST(20))d);
  ST(23) S = (ST(23))sxt<23>(FLD(e, 23, 23));
  ST(44) prod = (ST(44))((ST(44))S * dn);
  MULB(prod)
  ST(26) q = (ST(26))((ST(26))FLD(e, 0, 23) + (ST(26))(prod >> 14));
  return q < 0 ? (UT(23))0 : (UT(23))q;
}

// ---------------- 窗口扫描引擎 ----------------
static void eng_a(w128_t* gmem, AD jaddr, int mode, int H, int W, Cst c, Stream<Hdr>& hs, Stream<Hdr>& hs2, Stream<Cf>& cs, Stream<Item>& s) {
  UT(64) jd[JOB_U64];
#ifndef SRCH_SW
#pragma HLS ARRAY_PARTITION variable=jd complete
#endif
  for (int b = 0; b < JOB_U64 / 2; b++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
    w128_t w = gmem[(jaddr >> 4) + b];
    jd[2 * b] = (UT(64))w;
    jd[2 * b + 1] = (UT(64))(w >> 64);
  }
  ST(24) ox = (ST(24))sxt<24>(FLD(jd[0], 0, 24)), oy = (ST(24))sxt<24>(FLD(jd[0], 24, 24));
  UT(7) node = (UT(7))FLD(jd[0], 48, 7);
  ST(16) si0 = (ST(16))sxt<16>(FLD(jd[1], 0, 16)), sj0 = (ST(16))sxt<16>(FLD(jd[1], 16, 16));
  ST(16) vi0 = (ST(16))sxt<16>(FLD(jd[1], 32, 16)), vj0 = (ST(16))sxt<16>(FLD(jd[1], 48, 16));
  // 判定式系数交给 B 进程（锥体判定在那里按式时分）
  if (mode != MD_UNIV) {
    for (int m = 0; m < 5; m++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
      Cf cf;
      cf.a = (ST(18))sxt<18>(FLD(jd[2 + 2 * m], 0, 18));
      cf.b = (ST(18))sxt<18>(FLD(jd[2 + 2 * m], 18, 18));
      cf.f0 = (ST(28))sxt<28>(FLD(jd[3 + 2 * m], 0, 28));
      cf.f1 = (ST(28))sxt<28>(FLD(jd[3 + 2 * m], 32, 28));
      cs.write(cf);
    }
  }
  Hdr h;
  h.ox = ox; h.oy = oy;
  h.vrow = (UT(13))((UT(13))node * 40 + (UT(13))(si0 - vi0));
  h.vbit0 = (UT(6))(sj0 - vj0);
  h.wrow = (UT(13))((UT(13))node * 40);
  h.cx6 = (ST(16))(ox >> 5); h.cy6 = (ST(16))(oy >> 5);
  hs.write(h); hs2.write(h);
  for (int r = 0; r < H; r++) {
    for (int cc = 0; cc < W; cc++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=33 max=40
#endif
      ST(17) i = (ST(17))(si0 + r), j = (ST(17))(sj0 + cc);
      bool ins = i >= 0 && j >= 0 && i < (ST(17))c.ni && j < (ST(17))c.nj;
      UT(16) cell = ins ? (UT(16))((UT(16))i * c.nj + (UT(16))j) : (UT(16))0;
      W72 gw = GRIDM[cell >> 2];
      UT(18) g = lane_get(gw, (UT(2))(cell & 3));
      bool univ = (g >> 13) & 1, fr = (g >> 14) & 1;
      ST(24) cxp = (ST(24))((((ST(24))c.u0 + 2 * j + 1)) << 11), cyp = (ST(24))((((ST(24))c.v0 + 2 * i + 1)) << 11);
      ST(25) dx25 = (ST(25))(cxp - ox), dy25 = (ST(25))(cyp - oy);
      bool near = dx25 > -(1 << 17) && dx25 < (1 << 17) && dy25 > -(1 << 17) && dy25 < (1 << 17);
      ST(18) DX = (ST(18))dx25, DY = (ST(18))dy25;
      ST(36) dx2 = (ST(36))DX * DX;
      MULB(dx2)
      ST(36) dy2 = (ST(36))DY * DY;
      MULB(dy2)
      UT(36) X = (UT(36))((UT(36))dx2 + (UT(36))dy2);
      UT(18) n; ST(26) ux, uy;
      geom(X, DX, DY, n, ux, uy);
      UT(23) q = qofn(n);
      bool inr = near && X <= c.r2;
      // 预通过（锥体判定前）；VIEW/OBS 的锥体判定在 B 进程做，X == 0 的格锥体判定恒假，这里直接剔除
      bool pass = ins && near && (mode == MD_UNIV ? (fr && X <= c.rw2) : (univ && inr && X != 0));
      UT(8) K = 0;
      if (pass && mode != MD_VIEW && n > 6144) {
        UT(8) t = (UT(8))(((UT(18))(n - 6145)) >> 9);
        K = (UT(8))((((UT(16))t * 171) >> 9) + 1);
      }
      Item it;
      it.cell = cell; it.uidx = (UT(13))FLD(g, 0, 13); it.pass = pass; it.seen = inr; it.K = K;
      it.ux = ux; it.uy = uy; it.q = q; it.cand = pass && mode != MD_UNIV;
      it.DX = DX; it.DY = DY; it.n = n;
      s.write(it);
    }
  }
}

// B 进程：每格逐拍推进（锥体判定 5 拍 / 遮挡射线每拍一步），每格出一个结果 Res 给 C 进程。
// SRCHK4：动作（VISW/FLAGS/ENTRY 写、半字指针）挪到 C 进程，锥体乘法链不再进指针/行位的循环递推（c3 估 10.6 ns 的根因）。
struct Res { UT(16) cell; UT(13) uidx; UT(23) q; UT(1) ps, vb, seen; };
static void eng_b(int H, int W, UT(16) mapw, UT(16) maph, UT(16) ou, UT(16) ov, UT(12) rq, int mode,
                  Stream<Hdr>& hs, Stream<Cf>& cs, Stream<Item>& s, Stream<Res>& rs) {
  Hdr h = hs.read();
  ST(18) CA[5], CB[5]; ST(28) C0[5], C1[5];
  if (mode != MD_UNIV) {
    for (int m = 0; m < 5; m++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
      Cf cf = cs.read();
      CA[m] = cf.a; CB[m] = cf.b; C0[m] = cf.f0; C1[m] = cf.f1;
    }
  }
  const UT(11) total = (UT(11))(H * W);
  UT(11) it = 0;
  bool busy = false, blk = false, cfail = false, cph = false;
  UT(8) k = 0; UT(3) m = 0;
  Item cur;
  cur.K = 0; cur.pass = 0; cur.seen = 0; cur.cand = 0; cur.cell = 0; cur.uidx = 0; cur.ux = 0; cur.uy = 0; cur.q = 0;
  cur.DX = 0; cur.DY = 0; cur.n = 0;
  while (it < total) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=1089 max=8000
#endif
    bool ph;  // 本拍是锥体判定拍
    if (!busy) { cur = s.read(); k = 0; m = 0; ph = cur.cand; } else ph = cph;
    // 锥体：第 m 式 x 4 角点的符号判定（产品 4 点全在画面内；推导见 DESIGN §5.2）。
    // |nG| < 2^28·2^17 = 2^45，|ra|,|rb| < 2^25·2^12 = 2^37，和 < 2^46：48 位有符号精确。
    ST(37) pax = (ST(37))CA[m] * cur.DX;
    MULB(pax)
    ST(37) pby = (ST(37))CB[m] * cur.DY;
    MULB(pby)
    ST(37) pbx = (ST(37))CB[m] * cur.DX;
    MULB(pbx)
    ST(37) pay = (ST(37))CA[m] * cur.DY;
    MULB(pay)
    ST(26) al = (ST(26))((ST(37))(pax + pby) >> 11);
    ST(26) be = (ST(26))((ST(37))(pbx - pay) >> 11);
    ST(29) G = (ST(29))(C0[m] + al), Gp = (ST(29))(C1[m] + al);
    ST(48) nG = (ST(48))((ST(48))G * (ST(48))cur.n);
    MULB(nG)
    ST(48) nGp = (ST(48))((ST(48))Gp * (ST(48))cur.n);
    MULB(nGp)
    ST(40) ra = (ST(40))((ST(40))al * (ST(40))rq);
    MULB(ra)
    ST(40) rb = (ST(40))((ST(40))be * (ST(40))rq);
    MULB(rb)
    ST(48) t1 = (ST(48))(nG - ra), t2 = (ST(48))(nG + rb), t3 = (ST(48))(nG - rb), t4 = (ST(48))(nGp + ra);
    bool neg = t1 < 0 || t2 < 0 || t3 < 0 || t4 < 0;
    bool cfn = cfail || (ph && neg);
    bool fin = ph ? (m == 4 && cur.K == 0) : (cur.K == 0 || k == cur.K - 1);
    // 射线一步
    ST(11) tt = (ST(11))(8 + 3 * k);
    ST(36) px = (ST(36))h.ox + (ST(36))(((ST(36))cur.ux * tt) >> 15);
    ST(36) py = (ST(36))h.oy + (ST(36))(((ST(36))cur.uy * tt) >> 15);
    ST(14) pu = (ST(14))(px >> 11), pv = (ST(14))(py >> 11);
    bool inmap = pu >= 0 && pv >= 0 && pu < (ST(18))mapw && pv < (ST(18))maph;
    ST(15) wu = (ST(15))(pu - (ST(15))ou), wv = (ST(15))(pv - (ST(15))ov);
    bool inwin = wu >= 0 && wv >= 0 && wu < 512 && wv < 512;
    UT(12) oa = (UT(12))((FLD(wv, 0, 9) << 3) | (FLD(wu, 0, 9) >> 6));
    UT(64) ow = OCC[oa];
    bool bit = BIT(ow, FLD(wu, 0, 6));
    bool hit = !ph && cur.K != 0 && (!inmap || !inwin || bit);
    bool bn = blk || hit;
    if (fin) {
      Res r;
      r.cell = cur.cell; r.uidx = cur.uidx; r.q = cur.q; r.seen = cur.seen;
      r.ps = cur.pass && !cfn; r.vb = cur.pass && !cfn && !bn;
      rs.write(r);
      blk = false; cfail = false; busy = false; it++;
    } else {
      blk = bn; cfail = cfn; busy = true;
      if (ph) { if (m == 4) cph = false; else { cph = true; m++; } } else { cph = false; k++; }
    }
  }
}

// C 进程：视点头/尾半字、VISW 行位、FLAGS（SEEN / REACH）、ENTRY 追加。
static void eng_c(int mode, int H, int W, int last, Stream<Hdr>& hs, Stream<Res>& rs,
                  UT(16)& ptr_io, UT(36)& pend_io, UT(16)& nent_o, UT(8)& err_io, UT(16) lim) {
  Hdr h = hs.read();
  UT(16) ptr = ptr_io; UT(36) pend = pend_io; UT(16) nent = 0; UT(8) err = err_io;
  // SRCHK3：计数器按值域截位（窗口 ≤40x40 = 1600 格，头/尾各 ≤2 个半字）
  const UT(11) total = (UT(11))(H * W);
  const UT(2) nh = mode == MD_UNIV ? 0 : 2, nt = (mode != MD_UNIV && last) ? 2 : 0;
  UT(2) hc = 0, tc = 0; UT(11) it = 0; UT(6) row = 0, col = 0;
  const UT(6) wl = (UT(6))(W - 1);
  UT(40) rowbits = 0;
  for (;;) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=VISW inter false
#pragma HLS DEPENDENCE variable=ENTRY inter false
#pragma HLS DEPENDENCE variable=FLAGS inter false
#endif
    bool emit = false; UT(36) slot = 0;
    if (hc < nh) {
      slot = hc == 0 ? (UT(36))SR_HDR : (UT(36))(FLD(h.cx6, 0, 16) | (FLD(h.cy6, 0, 16) << 16));
      emit = true; hc++;
    } else if (it < total) {
      Res r = rs.read();
      W72 vw = VISW[h.vrow + row];
      if (mode == MD_UNIV) {
        UT(40) nr = (UT(40))((rowbits >> 1) | (((UT(40))(r.vb ? 1 : 0)) << 39));
        rowbits = nr;
        if (col == wl) { VISW[h.wrow + row] = (W72)nr; rowbits = 0; }
        if (r.vb && r.seen) FLAGS[r.cell] = 1;
      } else {
        bool ok;
        if (mode == MD_VIEW) {
          bool wb = BIT(vw, (UT(7))(h.vbit0 + col));
          ok = r.ps && wb;
          if (ok && r.q >= (1u << 22)) FLAGS[SR_REACH + r.uidx] = 1;
        } else {
          ok = r.vb;
        }
        if (ok) { emit = true; slot = (UT(36))(FLD(r.uidx, 0, 13) | (FLD(r.q, 0, 23) << 13)); nent++; }
      }
      it++;
      if (col == wl) { col = 0; row++; } else col++;
    } else if (tc < nt) {
      slot = tc == 0 ? (UT(36))(SR_HDR | (1u << 13)) : (UT(36))0;
      emit = true; tc++;
    } else {
      break;
    }
    if (emit) {
      if (ptr >= lim) {
        err |= E_EOVER;
      } else {
        if (ptr & 1) ENTRY[ptr >> 1] = (W72)(((W72)slot << 36) | (W72)pend);
        else pend = slot;
        ptr++;
      }
    }
  }
  // 末尾半字补齐只写字、不计数：ptr 是有效半字数（nso/nsv）。SRCHK2 修：原先 ptr++ 把补齐的 0 半字算进 nso，
  // 观测引擎把它当成 u=0、q=0 的项，每次 OBSERVE 都把格 0 的'上次看'位置改成 END 头后的 (0,0)、lq 清零。
  if (last && (ptr & 1)) ENTRY[ptr >> 1] = (W72)pend;
  ptr_io = ptr; pend_io = pend; nent_o = nent; err_io = err;
}

static void engine(w128_t* gmem, AD jaddr, int mode, int last, Cst c,
                   UT(16)& ptr, UT(36)& pend, UT(16)& nent, UT(8)& err, UT(16) lim) {
#ifndef SRCH_SW
#pragma HLS DATAFLOW
#endif
  Stream<Item> s;
  Stream<Hdr> hs, hs2;
  Stream<Cf> cs;
  Stream<Res> rs;
#ifndef SRCH_SW
#pragma HLS STREAM variable=s depth=4
#pragma HLS STREAM variable=hs depth=2
#pragma HLS STREAM variable=hs2 depth=2
#pragma HLS STREAM variable=cs depth=5
#pragma HLS STREAM variable=rs depth=4
#endif
  const int HW = mode == MD_UNIV ? 40 : 33;
  eng_a(gmem, jaddr, mode, HW, HW, c, hs, hs2, cs, s);
  eng_b(HW, HW, c.mapw, c.maph, c.ou, c.ov, c.rq, mode, hs, cs, s, rs);
  eng_c(mode, HW, HW, last, hs2, rs, ptr, pend, nent, err, lim);
}

// ---------------- 信念引擎 ----------------
// SRCHK4：拆成两个数据流进程。A：读 ENTRY / PST / MST / REACH，算 miss 与两项增量，OBSERVE 时写回；
// B：只做两个累加与视点结果写出。原先累加器的循环递推把"读 PST → 三次乘 → 加"整条链压进一拍（c2 实测 20.6 ns）。
struct BT { UT(2) kind; UT(50) t1; UT(49) t2; };   // kind 0 无，1 项，2 视点头
struct BAs { bool h2; ST(16) cx, cy; };
struct BBs { UT(64) acc, acc2; UT(10) v; bool have; };
static inline BT belief_step(int mode, UT(16) t, UT(16) base, UT(31) f, UT(36) lm, BAs& st) {
#ifndef SRCH_SW
#pragma HLS INLINE
#endif
    UT(16) sl = (UT(16))(base + t);
    W72 ew = ENTRY[sl >> 1];
    UT(36) slot = (UT(36))FLD(ew, (sl & 1) ? 36 : 0, 36);
    bool isent = false, ishdr = false;
    UT(13) u = 0; UT(23) q = 0;
    if (mode == BM_MASS) {
      isent = true; u = (UT(13))t;
    } else if (st.h2) {
      st.cx = (ST(16))sxt<16>(FLD(slot, 0, 16)); st.cy = (ST(16))sxt<16>(FLD(slot, 16, 16)); st.h2 = false;
    } else if (FLD(slot, 0, 13) == SR_HDR) {
      ishdr = true; st.h2 = true;
    } else {
      isent = true; u = (UT(13))FLD(slot, 0, 13); q = (UT(23))FLD(slot, 13, 23);
    }
    W72 P = PST[u], M = MST[u];
    bool rch = FLAGS[SR_REACH + u];
    UT(40) prior = (UT(40))FLD(P, 0, 40);
    ST(16) lx = (ST(16))sxt<16>(FLD(P, 40, 16)), ly = (ST(16))sxt<16>(FLD(P, 56, 16));
    UT(31) m0 = (UT(31))FLD(M, 0, 31);
    UT(23) lq = (UT(23))FLD(M, 31, 23);
    bool valid = FLD(M, 54, 1);
    UT(31) fm0 = m0 < f ? m0 : f;
    UT(55) pm1 = (UT(55))m0 * (UT(24))((((UT(24))1) << 23) - lq);
    MULB(pm1)
    UT(31) a1 = (UT(31))(pm1 >> 23);
    UT(31) mc = a1 > fm0 ? a1 : fm0;
    ST(17) dx = (ST(17))(lx - st.cx), dy = (ST(17))(ly - st.cy);
    ST(34) sdx = (ST(34))dx * dx;
    MULB(sdx)
    ST(34) sdy = (ST(34))dy * dy;
    MULB(sdy)
    bool same = valid && (UT(36))((UT(36))sdx + (UT(36))sdy) < lm;
    UT(31) bsv = same ? m0 : mc;
    UT(23) qq = same ? (q > lq ? q : lq) : q;
    UT(31) fb = bsv < f ? bsv : f;
    UT(55) pm2 = (UT(55))bsv * (UT(24))((((UT(24))1) << 23) - qq);
    MULB(pm2)
    UT(31) a2 = (UT(31))(pm2 >> 23);
    UT(31) mn = a2 > fb ? a2 : fb;
    // 增量值域：prior < 2^40，miss < 2^31，积 >> 21 < 2^50；未看项 prior << 9 < 2^49（与原 64 位累加逐位同）
    UT(31) dm = mode == BM_MASS ? mc : (UT(31))(mc - mn);
    WU(71) pdm = (WU(71))prior * dm;
    MULB(pdm)
    UT(50) t1 = (UT(50))(pdm >> 21);
    UT(49) t2 = mode == BM_MASS ? ((rch && mc > (1u << 29)) ? (UT(49))(((UT(49))prior) << 9) : (UT(49))0)
                                : ((q >= (1u << 22) && mc > (1u << 29)) ? (UT(49))1 : (UT(49))0);
    if (isent && mode == BM_OBS) {
      MST[u] = (W72)((W72)bsv | ((W72)qq << 31) | ((W72)1 << 54));
      if (!same) PST[u] = (W72)((W72)prior | ((W72)FLD(st.cx, 0, 16) << 40) | ((W72)FLD(st.cy, 0, 16) << 56));
    }
    BT o;
    o.kind = (mode == BM_OBS) ? (UT(2))0 : (isent ? (UT(2))1 : (ishdr ? (UT(2))2 : (UT(2))0));
    o.t1 = t1; o.t2 = t2;
    return o;
}
static inline void belief_acc(w128_t* gmem, AD res, int mode, const BT& o, BBs& st) {
#ifndef SRCH_SW
#pragma HLS INLINE
#endif
    if (o.kind == 2) {
      if (mode == BM_SEL && st.have) {
        gmem[(res >> 4) + R_VIEW / 2 + st.v] = (w128_t)(((w128_t)st.acc2 << 64) | (w128_t)st.acc);
        st.v++;
      }
      st.have = true; st.acc = 0; st.acc2 = 0;
    } else if (o.kind == 1) {
      st.acc += o.t1; st.acc2 += o.t2;
    }
}
#ifndef SRCH_SW
static void belief_a(int mode, UT(16) n, UT(16) base, UT(31) f, UT(36) lm, Stream<BT>& bs) {
  BAs st; st.h2 = false; st.cx = 0; st.cy = 0;
  for (UT(16) t = 0; t < n; t++) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=PST inter false
#pragma HLS DEPENDENCE variable=MST inter false
    bs.write(belief_step(mode, t, base, f, lm, st));
  }
}
static void belief_b(w128_t* gmem, AD res, int mode, UT(16) n, Stream<BT>& bs, UT(64)& mass_o, UT(64)& unseen_o) {
  BBs st; st.acc = 0; st.acc2 = 0; st.v = 0; st.have = false;
  for (UT(16) t = 0; t < n; t++) {
#pragma HLS PIPELINE II=1
    belief_acc(gmem, res, mode, bs.read(), st);
  }
  mass_o = st.acc; unseen_o = st.acc2;
}
#endif

static void belief(w128_t* gmem, AD res, int mode, UT(16) n, UT(16) base, UT(31) f, UT(36) lm,
                   UT(64)& mass_o, UT(64)& unseen_o) {
#ifndef SRCH_SW
#pragma HLS DATAFLOW
  Stream<BT> bs;
#pragma HLS STREAM variable=bs depth=4
  belief_a(mode, n, base, f, lm, bs);
  belief_b(gmem, res, mode, n, bs, mass_o, unseen_o);
#else
  BAs sa; sa.h2 = false; sa.cx = 0; sa.cy = 0;
  BBs sb; sb.acc = 0; sb.acc2 = 0; sb.v = 0; sb.have = false;
  for (UT(16) t = 0; t < n; t++) belief_acc(gmem, res, mode, belief_step(mode, t, base, f, lm, sa), sb);
  mass_o = sb.acc; unseen_o = sb.acc2;
#endif
}

// ---------------- 小循环 ----------------
static void load(w128_t* gmem, AD addr, UT(16) nb, int tgt, UT(16) off) {
  for (UT(16) b = 0; b < nb; b++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=2
#endif
    w128_t w = gmem[(addr >> 4) + b];
    UT(64) lo = (UT(64))w, hi = (UT(64))(w >> 64);
    UT(16) a = (UT(16))(off + 2 * b);
    if (tgt == 0) { OCC[a & 4095] = lo; OCC[(a + 1) & 4095] = hi; }
    else if (tgt == 1) { ENTRY[a] = (W72)lo; ENTRY[a + 1] = (W72)hi; }
    else if (tgt == 2) { GRIDM[a] = (W72)lo; GRIDM[a + 1] = (W72)hi; }
    else { VISW[a & 4095] = (W72)lo; VISW[(a + 1) & 4095] = (W72)hi; }
  }
}

// R-A：块归并、避让十字（r=1）、贴边圆盘（r=3），单遍 (ni+3)x(nj+3)
static void grid_a(UT(8) ni, UT(8) nj) {
  UT(7) win[7]; UT(2) fa[4];
#ifndef SRCH_SW
#pragma HLS ARRAY_PARTITION variable=win complete
#pragma HLS ARRAY_PARTITION variable=fa complete
#endif
  for (int k = 0; k < 7; k++) win[k] = 0;
  for (int k = 0; k < 4; k++) fa[k] = 0;
  UT(16) oc = 0; W72 acc = 0;
  const UT(16) ncell = (UT(16))ni * nj;
  for (int i = 0; i < ni + 3; i++) {
    for (int j = 0; j < nj + 3; j++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=40 max=201
#pragma HLS DEPENDENCE variable=DIL inter false
#endif
      bool inb = i < ni && j < nj;
      UT(16) c = inb ? (UT(16))((UT(16))i * nj + j) : (UT(16))0;
      W72 bw = ENTRY[c >> 3];
      UT(8) bsel[8];
#ifndef SRCH_SW
#pragma HLS ARRAY_PARTITION variable=bsel complete
#endif
      for (int k = 0; k < 8; k++) bsel[k] = (UT(8))FLD(bw, 8 * k, 8);
      UT(8) byte = inb ? bsel[c & 7] : (UT(8))0;
      bool occ = (byte & 15) != 0;
      UT(3) fc = (UT(3))(((byte >> 4) & 1) + ((byte >> 5) & 1) + ((byte >> 6) & 1) + ((byte >> 7) & 1));
      bool fre = !occ && fc >= 2, any = fc > 0;
      UT(12) hst = i == 0 ? (UT(12))0 : DIL[j];
      UT(7) colo = (UT(7))(((FLD(hst, 0, 6)) << 1) | (occ ? 1 : 0));
      UT(8) colf = (UT(8))(((FLD(hst, 6, 6)) << 2) | ((fre ? 2 : 0) | (any ? 1 : 0)));
      DIL[j] = (UT(12))(FLD(colo, 0, 6) | (FLD(colf, 0, 6) << 6));
      if (j == 0) {
        for (int k = 0; k < 7; k++) win[k] = 0;
        for (int k = 0; k < 4; k++) fa[k] = 0;
      }
      for (int k = 0; k < 6; k++) win[k] = win[k + 1];
      win[6] = colo;
      for (int k = 0; k < 3; k++) fa[k] = fa[k + 1];
      fa[3] = (UT(2))FLD(colf, 6, 2);
      if (i >= 3 && j >= 3) {
        // 输出 (i-3, j-3)：行偏移 di -> 位 3-di；列偏移 dj -> win[3+dj]
        bool edge = false;
        const int hw[7] = {0, 2, 2, 3, 2, 2, 0};  // di = -3..3 的半宽（r=3 圆盘）
        for (int di = 0; di < 7; di++)
          for (int dj = -3; dj <= 3; dj++)
            if (dj >= -hw[di] && dj <= hw[di] && ((win[3 + dj] >> (6 - di)) & 1)) edge = true;
        bool o = (win[3] >> 3) & 1;
        bool cross = o || ((win[2] >> 3) & 1) || ((win[4] >> 3) & 1) || ((win[3] >> 2) & 1) || ((win[3] >> 4) & 1);
        UT(2) ofa = fa[0];
        bool ofr = (ofa >> 1) & 1, oany = ofa & 1;
        bool trav = !o && !cross && oany;
        UT(18) lane = (UT(18))(((UT(18))(ofr ? 1 : 0) << 14) | ((UT(18))(trav ? 1 : 0) << 15) |
                               ((UT(18))(o ? 0 : 1) << 16) | ((UT(18))(edge ? 1 : 0) << 17));
        acc = lane_set(acc, (UT(2))(oc & 3), lane);
        if ((oc & 3) == 3 || oc == ncell - 1) { GRIDM[oc >> 2] = acc; acc = 0; }
        oc++;
      }
    }
  }
}

// R-B：光栅序压缩编号，置可见域位，清 SEEN
static UT(16) grid_b(UT(16) ncell, UT(8)& err) {
  UT(16) U = 0; W72 acc = 0;
  for (UT(16) c = 0; c < ncell; c++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=GRIDM inter false
#endif
    W72 gw = GRIDM[c >> 2];
    UT(18) g = lane_get(gw, (UT(2))(c & 3));
    bool sn = FLAGS[c];
    FLAGS[c] = 0;
    if (sn) {
      if (U < SR_UMAX) { g = (UT(18))((g & ~(UT(18))0x3FFF) | (UT(18))U | (1u << 13)); U++; }
      else err |= E_UOVER;
    }
    acc = lane_set(acc, (UT(2))(c & 3), g);
    if ((c & 3) == 3 || c == ncell - 1) { GRIDM[c >> 2] = acc; acc = 0; }
  }
  return U;
}

// 200x200 代价图（规划核全局档）：内部 (i+1, j+1) 可走 0，否则 254
static void costmap(w128_t* gmem, AD out, UT(8) ni, UT(8) nj) {
  w128_t acc = 0;
  int r = 0, cl = 0;
  for (UT(16) t = 0; t < 40000; t++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
    bool ins = r >= 1 && cl >= 1 && r <= ni && cl <= nj;
    UT(16) cell = ins ? (UT(16))((UT(16))(r - 1) * nj + (UT(16))(cl - 1)) : (UT(16))0;
    W72 gw = GRIDM[cell >> 2];
    bool tv = ins && ((lane_get(gw, (UT(2))(cell & 3)) >> 15) & 1);
    UT(8) by = tv ? 0 : 254;
    acc = (w128_t)((acc >> 8) | ((w128_t)by << 120));  // 移位寄存器：16 拍后第 t&15 字节正好落在第 t&15 位置
    if ((t & 15) == 15) gmem[(out >> 4) + (t >> 4)] = acc;
    if (cl == 199) { cl = 0; r++; } else cl++;
  }
}

// SRCHK4：近节点圈与最近可走格合成一个"节点（点）周围窗口"小循环（原先两个函数各自取作业、算中心格、读 GRID）。
// md 0：近节点圈，节点格 ±3，0.3 m 内且无占据子格的格置可走位（GRID 读改写）；
// md 1：最近可走格（产品 _cell_of：0.5 m 内，di 外 dj 内，严格更小者胜），结果两两写结果块。
static const UT(5) SQ5[11] = {25, 16, 9, 4, 1, 0, 1, 4, 9, 16, 25};
static void nbr(w128_t* gmem, AD aux, AD res, UT(16) njobs, int md, const Cst& c, UT(36) rn2) {
  const int R = md ? 5 : 3;
  UT(64) prev = 0;
  for (UT(16) jb = 0; jb < njobs; jb++) {
    w128_t w = gmem[(aux >> 4) + jb * (JOB_U64 / 2)];
    ST(24) ox = (ST(24))sxt<24>(FLD((UT(64))w, 0, 24)), oy = (ST(24))sxt<24>(FLD((UT(64))w, 24, 24));
    ST(14) ic = (ST(14))((ST(25))(oy - ((ST(25))c.v0 << 11)) >> 12);
    ST(14) jc = (ST(14))((ST(25))(ox - ((ST(25))c.u0 << 11)) >> 12);
    UT(6) bk = 63; UT(8) ba = 0, bb = 0;
    for (int di = -R; di <= R; di++) {
      for (int dj = -R; dj <= R; dj++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE
#pragma HLS LOOP_TRIPCOUNT min=7 max=11
#endif
        ST(15) i = (ST(15))(ic + di), j = (ST(15))(jc + dj);
        bool ins = i >= 0 && j >= 0 && i < (ST(15))c.ni && j < (ST(15))c.nj;
        UT(16) cell = ins ? (UT(16))((UT(16))i * c.nj + (UT(16))j) : (UT(16))0;
        W72 gw = GRIDM[cell >> 2];
        UT(18) g = lane_get(gw, (UT(2))(cell & 3));
        if (md == 0) {
          // 格在节点格 ±3 内，|DX|,|DY| < 8 px = 2^14（Q.11），18 位有符号足够
          ST(18) DX = (ST(18))((((ST(25))c.u0 + 2 * j + 1) << 11) - ox);
          ST(18) DY = (ST(18))((((ST(25))c.v0 + 2 * i + 1) << 11) - oy);
          UT(36) X = (UT(36))((UT(36))((ST(36))DX * DX) + (UT(36))((ST(36))DY * DY));
          if (ins && X <= rn2 && ((g >> 16) & 1)) GRIDM[cell >> 2] = lane_set(gw, (UT(2))(cell & 3), (UT(18))(g | (1u << 15)));
        } else {
          bool tv = ins && ((g >> 15) & 1);
          UT(6) kk = (UT(6))(SQ5[di + 5] + SQ5[dj + 5]);
          if (tv && kk <= 25 && kk < bk) { bk = kk; ba = (UT(8))i; bb = (UT(8))j; }
        }
      }
    }
    if (md) {
      UT(64) val = bk == 63 ? (UT(64))0 : (UT(64))(((UT(64))1 << 32) | ((UT(64))ba << 16) | (UT(64))bb);
      if (jb & 1) gmem[(res >> 4) + R_NODECELL / 2 + (jb >> 1)] = (w128_t)(((w128_t)val << 64) | (w128_t)prev);
      else if (jb == njobs - 1) gmem[(res >> 4) + R_NODECELL / 2 + (jb >> 1)] = (w128_t)val;
      prev = val;
    }
  }
}

// 先验：每格两拍（s=0 主高斯 x 权，s=1 近高斯）。mode 0 最小 d^2；1 三个和；2 写 prior。
static void prior(int mode, const Cst& c, ST(32) tx, ST(32) ty, UT(32) k1, UT(32) k2, UT(32) i1, UT(32) i2,
                  UT(40) cm, UT(40) cn, UT(40) cw, UT(40) ccst, UT(64)& o0, UT(64)& o1, UT(64)& o2) {
  UT(42) mn = ~(UT(42))0;
  // SRCHK4：和按值域截位（可见域格 ≤8191：Σ e·w < 8191·3·2^25 < 2^40，Σ e < 2^38，Σ w < 2^15），逐位同原 64 位
  UT(40) sm = 0; UT(38) sn = 0; UT(15) sw = 0;
  UT(25) e1 = 0;
  UT(16) cell = 0;
  for (int i = 0; i < c.ni; i++) {
    for (int j = 0; j < c.nj; j++) {
      for (int s = 0; s < 2; s++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=2 max=2
#pragma HLS DEPENDENCE variable=PST inter false
#endif
        W72 gw = GRIDM[cell >> 2];
        UT(18) g = lane_get(gw, (UT(2))(cell & 3));
        bool univ = (g >> 13) & 1, edge = (g >> 17) & 1;
        UT(13) u = (UT(13))FLD(g, 0, 13);
        // SRCHK4：DX/DY 22 位有符号。原式 X 取 42 位（|DX|,|DY| ≥ 2^20.5 时原本就回绕），驱动保证记忆位置与裁剪区
        // 任一格中心相距 < 2^21 Q.11 = 1024 px（51 m）时与原 33 位逐位同。
        ST(22) DX = (ST(22))((((ST(33))c.u0 + 2 * j + 1) << 11) - tx);
        ST(22) DY = (ST(22))((((ST(33))c.v0 + 2 * i + 1) << 11) - ty);
        ST(44) qdx = (ST(44))DX * DX;
        MULB(qdx)
        ST(44) qdy = (ST(44))DY * DY;
        MULB(qdy)
        UT(42) X = (UT(42))((UT(44))qdx + (UT(44))qdy);
        UT(32) K = s ? k2 : k1, I = s ? i2 : i1;
        UT(64) xk = (UT(64))(X >> 10) * K;
        MULB(xk)
        UT(32) xr = (UT(32))(xk >> 28);
        UT(48) Ish = ((UT(48))I) << 16;
        UT(32) x = xr > Ish ? (UT(32))(xr - Ish) : (UT(32))0;
        UT(16) ip = (UT(16))(x >> 16), fr = (UT(16))(x & 0xFFFF);
        W72 tw = VISW[SR_VISW_BASE_EXP + (fr >> 7)];
        UT(25) T0 = (UT(25))FLD(tw, 0, 25), T1 = (UT(25))FLD(tw, 32, 25);
        UT(25) ev = (UT(25))(T0 - (((UT(32))(T0 - T1) * (UT(7))(fr & 127)) >> 7));
        // SRCHK4：ev >> ip 用乘 2^(24-ip) 再右移 24（ip ≥ 25 时结果本就为 0），替代 25 位桶形移位器
        UT(25) p2 = ip >= 25 ? (UT(25))0 : (UT(25))(((UT(25))1) << (24 - (int)ip));
        UT(50) ep = (UT(50))ev * p2;
        MULB(ep)
        UT(25) e = (UT(25))(ep >> 24);
        UT(27) ew = edge ? (UT(27))((UT(27))e * 3) : (UT(27))e;
        UT(2) w = edge ? 3 : 1;
        if (univ) {
          if (mode == 0) { if (X < mn) mn = X; }
          else if (mode == 1) {
            if (s == 0) { sm += ew; sw += w; } else sn += e;
          } else if (s == 1) {
            UT(27) e1w = edge ? (UT(27))((UT(27))e1 * 3) : (UT(27))e1;
            WU(68) pa1 = (WU(68))cm * e1w;
            MULB(pa1)
            WU(68) pa2 = (WU(68))cn * e;
            MULB(pa2)
            WU(68) a = (WU(68))(pa1 + pa2);
            UT(40) pr = (UT(40))((UT(44))(a >> 24) + (UT(44))((UT(44))cw * w) + ccst);
            W72 P = PST[u];
            PST[u] = (W72)((P & ~(W72)MSK(40)) | (W72)pr);
          }
        }
        if (s == 0) e1 = e;
        else cell++;
      }
    }
  }
  o0 = mode == 0 ? (UT(64))mn : (UT(64))sw; o1 = sm; o2 = sn;
}

// 清零：RESET 写 MST 初值（m0=1）；INIT_VIEWS 前清 REACH
static void clear(int mode, UT(16) U) {
  for (UT(16) u = 0; u < U; u++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
    // SRCHK2：RESET 同时清 PST（先验与'上次看'位置），使 DUMP 只由本次 RESET 以来的输入决定（原先留着上个任务的
    // lx/ly 位，功能上被 valid=0 屏蔽，但导出状态依赖历史，联仿向量不自洽）。PRIOR2 只改低 40 位，OBSERVE 写的位置保留。
    if (mode == 0) { MST[u] = (W72)((W72)1 << 30); PST[u] = 0; }
    else FLAGS[SR_REACH + u] = 0;
  }
}

static void dump(w128_t* gmem, AD out, UT(16) U) {
  for (UT(16) u = 0; u < U; u++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
    gmem[(out >> 4) + u] = (w128_t)((w128_t)PST[u] | ((w128_t)FLD(MST[u], 0, 56) << 72));
  }
}

// ---------------- 顶层 ----------------
static UT(32) srch_core(w128_t* gmem, AD prm) {
  UT(64) p[2 * PB_BEATS];
#ifndef SRCH_SW
#pragma HLS ARRAY_PARTITION variable=p complete
#endif
  for (int b = 0; b < PB_BEATS; b++) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
    w128_t w = gmem[(prm >> 4) + b];
    p[2 * b] = (UT(64))w; p[2 * b + 1] = (UT(64))(w >> 64);
  }
  UT(8) err = 0;
  int op = (int)FLD(p[P_HDR], 24, 8);
  AD res = (AD)p[P_RES], aux = (AD)p[P_AUX], aux2 = (AD)p[P_AUX2], tab = (AD)p[P_TAB];
  Cst c;
  c.ni = (UT(8))FLD(p[P_GEO1], 0, 8); c.nj = (UT(8))FLD(p[P_GEO1], 16, 8);
  c.u0 = (UT(16))FLD(p[P_GEO1], 32, 16); c.v0 = (UT(16))FLD(p[P_GEO1], 48, 16);
  c.mapw = (UT(16))FLD(p[P_GEO2], 0, 16); c.maph = (UT(16))FLD(p[P_GEO2], 16, 16);
  c.ou = (UT(16))FLD(p[P_GEO2], 32, 16); c.ov = (UT(16))FLD(p[P_GEO2], 48, 16);
  c.r2 = (UT(36))FLD(p[P_R2], 0, 36); c.rw2 = (UT(36))FLD(p[P_RW2], 0, 36);
  c.rq = (UT(12))FLD(p[P_RQ], 0, 12);
  QN = (UT(4))FLD(p[P_HDR], 32, 4);
  for (int i = 0; i < 8; i++) {
#ifndef SRCH_SW
#pragma HLS UNROLL
#endif
    QD[i] = (UT(18))FLD(p[P_QT + i], 0, 18);
    QE[i] = (UT(46))FLD(p[P_QT + i], 18, 46);
  }
  UT(16) U = (UT(16))FLD(p[P_CNT], 0, 16), nsv = (UT(16))FLD(p[P_CNT], 16, 16);
  UT(16) nso = (UT(16))FLD(p[P_CNT], 32, 16), njobs = (UT(16))FLD(p[P_CNT], 48, 16);
  UT(31) f = (UT(31))FLD(p[P_RQ], 32, 31);
  UT(64) raux = 0, r2a = 0, r2b = 0, r3a = 0, r3b = 0, r4a = 0, r4b = 0;
  bool ok = FLD(p[P_HDR], 0, 16) == SR_MAGIC && op >= OP_INIT_MAP && op <= OP_DUMP && op != 2;
  if (!ok) err |= E_MAGIC;
  if (c.ni < SR_MINN || c.nj < SR_MINN || c.ni > SR_MAXN || c.nj > SR_MAXN || njobs > SR_VIEWS) err |= E_SIZE;
  if (err == 0) {
    if (op == OP_INIT_MAP) {
      UT(16) ncell = (UT(16))c.ni * c.nj;
      for (int sg = 0; sg < 4; sg++) {
        AD a = sg == 0 ? aux : (sg == 1 ? aux2 : (sg == 2 ? tab : (AD)(tab + 768 * 8)));
        UT(16) nb = sg == 0 ? (UT(16))(SR_OCCW / 2) : (sg == 1 ? (UT(16))((ncell + 15) >> 4) : (sg == 2 ? (UT(16))384 : (UT(16))256));
        UT(16) off = sg == 2 ? (UT(16))SR_RSQ_BASE : (sg == 3 ? (UT(16))SR_VISW_BASE_EXP : (UT(16))0);
        load(gmem, a, nb, sg, off);
      }
      grid_a(c.ni, c.nj);
    }
    if (op == OP_INIT_FINAL) {
      U = grid_b((UT(16))c.ni * c.nj, err);
      costmap(gmem, aux2, c.ni, c.nj);
      raux = U;
    }
    if (op == OP_INIT_UNIV || op == OP_INIT_FINAL || op == OP_CELLOF)
      nbr(gmem, aux, res, op == OP_CELLOF ? (UT(16))1 : njobs, op == OP_INIT_UNIV ? 0 : 1, c, (UT(36))FLD(p[P_RN2], 0, 36));
    if (op == OP_RESET || op == OP_INIT_VIEWS) clear(op == OP_RESET ? 0 : 1, U);
    if (op == OP_INIT_UNIV || op == OP_INIT_VIEWS || op == OP_OBSERVE) {
      int mode = op == OP_INIT_UNIV ? MD_UNIV : (op == OP_INIT_VIEWS ? MD_VIEW : MD_OBS);
      UT(16) ptr = op == OP_OBSERVE ? (UT(16))SR_ENTRY_OBS : (UT(16))0;
      UT(16) lim = op == OP_OBSERVE ? (UT(16))(SR_SLOTS - 1) : (UT(16))(SR_ENTRY_OBS - 1);
      UT(36) pend = 0; UT(16) nent = 0, ntot = 0;
      UT(16) nj2 = op == OP_OBSERVE ? (UT(16))1 : njobs;
      for (UT(16) jb = 0; jb < nj2; jb++) {
        engine(gmem, (AD)(aux + jb * (JOB_U64 * 8)), mode, jb == nj2 - 1, c, ptr, pend, nent, err, lim);
        ntot += nent;
      }
      raux = (UT(64))ptr | ((UT(64))ntot << 32);
      nso = (UT(16))(ptr - SR_ENTRY_OBS);
    }
    if (op == OP_PRIOR0 || op == OP_PRIOR1 || op == OP_PRIOR2) {
      UT(64) o0, o1, o2;
      prior(op - OP_PRIOR0, c, (ST(32))sxt<32>(FLD(p[P_ORG], 0, 32)), (ST(32))sxt<32>(FLD(p[P_ORG], 32, 32)),
            (UT(32))FLD(p[P_K], 0, 32), (UT(32))FLD(p[P_K], 32, 32), (UT(32))FLD(p[P_I0], 0, 32),
            (UT(32))FLD(p[P_I0], 32, 32), (UT(40))FLD(p[P_CM], 0, 40), (UT(40))FLD(p[P_CN], 0, 40),
            (UT(40))FLD(p[P_CW], 0, 40), (UT(40))FLD(p[P_CC], 0, 40), o0, o1, o2);
      if (op == OP_PRIOR0) r3b = o0;
      else { r4a = o1; r4b = o2; r3a = o0; }   // R_SM/R_SN 在第 2 拍，R_SW 在第 3 拍
    }
    if (op == OP_OBSERVE || op == OP_SELECT) {
      int nph = op == OP_SELECT ? 2 : 1;
      for (int ph = 0; ph < nph; ph++) {
        int bm = op == OP_OBSERVE ? BM_OBS : (ph == 0 ? BM_MASS : BM_SEL);
        UT(16) n = bm == BM_MASS ? U : (bm == BM_SEL ? nsv : nso);
        UT(16) base = bm == BM_OBS ? (UT(16))SR_ENTRY_OBS : (UT(16))0;
        UT(64) m1, m2;
        belief(gmem, res, bm, n, base, f, (UT(36))FLD(p[P_LM], 0, 36), m1, m2);
        if (bm == BM_MASS) { r2a = m1; r2b = m2; }
      }
    }
    if (op == OP_DUMP) dump(gmem, aux2, U);
  }
  UT(32) st = (UT(32))((UT(32))err | ((UT(32))SR_VERSION << 8) | ((UT(32))op << 16));
  // SRCHK3：结果块头 4 拍并成一个写口的小循环（原先 4 个独立写点，各带一串 m_axi 等待状态）；写序 3,2,1,0 同原先（第 0 拍最后）。
  w128_t rb[4];
  bool en[4];
#ifndef SRCH_SW
#pragma HLS ARRAY_PARTITION variable=rb complete
#pragma HLS ARRAY_PARTITION variable=en complete
#endif
  rb[0] = (w128_t)(((w128_t)raux << 64) | (w128_t)((UT(64))st | ((UT(64))U << 32))); en[0] = true;
  rb[1] = (w128_t)(((w128_t)r2b << 64) | (w128_t)r2a); en[1] = op == OP_SELECT;
  rb[2] = (w128_t)(((w128_t)r4b << 64) | (w128_t)r4a); en[2] = op == OP_PRIOR1;
  rb[3] = (w128_t)(((w128_t)r3b << 64) | (w128_t)r3a); en[3] = op == OP_PRIOR0 || op == OP_PRIOR1;
  for (int k = 3; k >= 0; k--) {
#ifndef SRCH_SW
#pragma HLS PIPELINE II=1
#endif
    if (en[k]) gmem[(res >> 4) + k] = rb[k];
  }
  return st;
}

#ifdef SRCH_SW
extern "C" uint32_t srch_run_sw(uint8_t* ddr, uint64_t prm) { return (uint32_t)srch_core((w128_t*)ddr, (AD)prm); }
extern "C" void srch_dump_grid(uint32_t* out, uint32_t n) {
  for (uint32_t c = 0; c < n; c++) out[c] = (uint32_t)FLD(GRIDM[c >> 2], (c & 3) * 18, 18);
}
extern "C" void srch_dump_slots(uint64_t* out, uint32_t s0, uint32_t n) {
  for (uint32_t s = s0; s < s0 + n; s++) out[s - s0] = FLD(ENTRY[s >> 1], (s & 1) ? 36 : 0, 36);
}
#else
void srch_top(w128_t* gmem, ap_uint<64> prm, ap_uint<32>& status) {
#pragma HLS INTERFACE m_axi port=gmem offset=off bundle=gmem depth=24576 max_read_burst_length=8 max_write_burst_length=8 num_read_outstanding=1 num_write_outstanding=1 latency=32
#pragma HLS INTERFACE s_axilite port=prm
#pragma HLS INTERFACE s_axilite port=status
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS BIND_STORAGE variable=GRIDM type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=OCC type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=VISW type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=ENTRY type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=PST type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=MST type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=FLAGS type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=DIL type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=QD complete
#pragma HLS BIND_STORAGE variable=QE type=ram_1p impl=lutram
  status = srch_core(gmem, (AD)prm);
}
#endif
