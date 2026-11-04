// PREP1 识别前处理核：HLS 源码。数据通路按 PREP0/DESIGN.md §2.1 分级，每级一个 dataflow 进程、1 像素/拍：
//   读源帧(m_axi 128b) -> 齿轮箱 128->像素(YUYV 含色度上采样) -> 色彩变换 -> 水平 2 抽头 -> 竖向 2 抽头(1 行缓冲读后写)+量化表
//   -> 灰边插入+齿轮箱 24->128 -> 写张量(m_axi 128b)
// 缩放表、量化表、色彩系数由 A53 生成放进 DDR 参数块，PARAM_RELOAD=1 时装进片上（静态数组跨帧保持）。
#include "prep_k.h"

typedef ap_uint<24> px_t;    // 一个像素三字节（RGB 或 Y/U/V）
typedef ap_uint<45> h3_t;    // 三通道水平结果，各 15 位

// ---------------- 1. 读源帧：整帧一段连续地址（h0 行 × 行步长 stride_w 字）一个扁平循环，适配器按 64 拍拆突发并提前发请求；
//                   每行只把前 nw_row 字送下游，行步长里多出的字读了丢掉（行间不再有「等下一行突发」的空档，s_in 只需 2 深）
static void rd_src(const word_t* src, ap_uint<23> nrd, ap_uint<12> stride_w, ap_uint<12> nw_row,
                   hls::stream<word_t>& out) {
  ap_uint<12> w = 0;
rw:
  for (ap_uint<23> i = 0; i < nrd; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=57600 max=57600
    word_t v = src[i];
    if (w < nw_row) out.write(v);
    w = (w == stride_w - 1) ? ap_uint<12>(0) : ap_uint<12>(w + 1);
  }
}

// ---------------- 2. 齿轮箱 128 -> 像素 ----------------
// RGB/BGR：18 字节寄存器 B，字从顶端整字移入、像素按位置取出（见循环内）；行尾丢弃本行最后一字的剩余字节。
// YUYV/UYVY：每字 4 个像素对；每行多跑 2 拍，以便三角上采样看到下一对的色度（libjpeg h2v1_fancy_upsample）。
//   输出 [7:0] Y、[15:8] U、[23:16] V（已上采样）。
static void unpack(hls::stream<word_t>& in, ap_uint<12> w0, ap_uint<23> total, ap_uint<2> fmt, bool ups,
                   hls::stream<px_t>& out) {
  const bool yuv = fmt[1];
  const bool uyvy = fmt[0];
  const ap_uint<13> tend = yuv ? ap_uint<13>(w0 + 2) : ap_uint<13>(w0);
  const ap_uint<11> npair = w0 >> 1;
  ap_uint<144> B = 0;
  ap_uint<5> a = 16;
  if (!yuv && total != 0) B = (in.read(), B(143, 128));   // 预装第一行第一个字
  word_t wreg = 0;
  ap_uint<13> t = 0;
  ap_uint<8> cy0 = 0, cy1 = 0, cu = 0, cv = 0;   // 像素对 j-1
  ap_uint<8> pu = 0, pv = 0;                     // 像素对 j-2 的色度
  ap_uint<8> ny0 = 0, ny1 = 0, nu = 0, nv = 0;   // 像素对 j
upk:
  for (ap_uint<23> i = 0; i < total; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=307200 max=308160
    if (!yuv) {
      // 有效字节 a 个在 B 顶端 [18-a,18)，a>=3：取窗口 [18-a, 21-a) 输出（16 种位置的 24 位多路选择）；
      // 取完若剩不足 3 字节（或到行尾）就预装下一个字：B 整体下移 16 字节、新字放顶端（固定移位，不耗 LUT）
      const ap_uint<4> o = ap_uint<5>(18) - a;
      const ap_uint<144> sh = B >> (ap_uint<8>(o) << 3);   // 只用低 24 位：综合后是 24 个 16 选 1
      out.write(sh(23, 0));
      const bool eol = t == w0 - 1;
      const ap_uint<5> ar = a - 3;
      const bool ld = eol ? (i != total - 1) : (ar < 3);
      if (ld) B = (in.read(), B(143, 128));
      a = eol ? ap_uint<5>(16) : ld ? ap_uint<5>(ar + 16) : ar;
      if (eol) t = 0; else t++;
    } else {
      ap_uint<12> j = t >> 1;
      if (!t[0]) {
        if (j < npair) {
          if (j(1, 0) == 0) wreg = in.read();
          ap_uint<32> q;   // 4 选 1 取 32 位对（不用 128 位通用右移）
          switch (j(1, 0)) {
            case 0: q = wreg(31, 0); break;
            case 1: q = wreg(63, 32); break;
            case 2: q = wreg(95, 64); break;
            default: q = wreg(127, 96); break;
          }
          if (uyvy) { nu = q(7, 0); ny0 = q(15, 8); nv = q(23, 16); ny1 = q(31, 24); }
          else      { ny0 = q(7, 0); nu = q(15, 8); ny1 = q(23, 16); nv = q(31, 24); }
        }
        if (j != 0) {   // 输出像素 2(j-1)：(3c[i] + c[i-1] + 1) >> 2，首列取 c[0]
          ap_uint<8> ou = cu, ov = cv;
          if (ups && j != 1) {
            ou = (ap_uint<10>(cu) * 3 + pu + 1) >> 2;
            ov = (ap_uint<10>(cv) * 3 + pv + 1) >> 2;
          }
          out.write((ov, ou, cy0));
        }
      } else {
        if (j != 0) {   // 输出像素 2(j-1)+1：(3c[i] + c[i+1] + 2) >> 2，末列取 c[n-1]
          ap_uint<8> ou = cu, ov = cv;
          if (ups && j != npair) {
            ou = (ap_uint<10>(cu) * 3 + nu + 2) >> 2;
            ov = (ap_uint<10>(cv) * 3 + nv + 2) >> 2;
          }
          out.write((ov, ou, cy1));
        }
        pu = cu; pv = cv; cy0 = ny0; cy1 = ny1; cu = nu; cv = nv;
      }
      if (t == tend - 1) t = 0; else t++;
    }
  }
}

// ---------------- 3. 色彩变换（YUYV）/ 通道换序（BGR）/ 直通（RGB） ----------------
// X = clamp((KY·(Y−YOFF) + KX·(C−128) + 2^15) >> 16)；KY=65536、YOFF=0 与 libjpeg JFIF 整数实现逐位相同。
static ap_uint<8> clamp8(ap_int<32> s) {
#pragma HLS INLINE
  ap_int<16> v = s >> 16;
  return v < 0 ? ap_uint<8>(0) : v > 255 ? ap_uint<8>(255) : ap_uint<8>(v);
}

static void cvt(hls::stream<px_t>& in, ap_uint<23> npix, ap_uint<2> fmt, ap_int<20> ky, ap_int<20> yoff,
                ap_int<20> krv, ap_int<20> kgu, ap_int<20> kgv, ap_int<20> kbu, hls::stream<px_t>& out) {
cv:
  for (ap_uint<23> i = 0; i < npix; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=307200 max=307200
    px_t p = in.read();
    ap_uint<8> a = p(7, 0), b = p(15, 8), c = p(23, 16);
    px_t o;
    if (fmt[1]) {
      ap_int<10> y = ap_int<10>(a) - ap_int<10>(yoff);
      ap_int<9> u = ap_int<9>(ap_int<10>(b) - 128), v = ap_int<9>(ap_int<10>(c) - 128);
      ap_int<30> py = ky * y;
      ap_int<30> prv = krv * v;
      ap_int<30> pgu = kgu * u;
      ap_int<30> pgv = kgv * v;
      ap_int<30> pbu = kbu * u;
      ap_uint<8> R = clamp8(ap_int<32>(py) + prv + 32768);
      ap_uint<8> G = clamp8(ap_int<32>(py) + pgu + pgv + 32768);
      ap_uint<8> B = clamp8(ap_int<32>(py) + pbu + 32768);
      o = (B, G, R);
    } else if (fmt[0]) {
      o = (a, b, c);          // BGR888：字节 0 是 B
    } else {
      o = p;                  // RGB888
    }
    out.write(o);
  }
}

// ---------------- 4. 水平 2 抽头：源列 x 等于下一输出列的 x1 时，用前一像素寄存器 (x1-1) 与当前像素算 H ----------------
static void xfeed(const tab_t xt[PREP_MAX_OUT], ap_uint<10> ow, ap_uint<12> h0, bool bypass, hls::stream<tab_t>& out) {
  if (bypass) return;
xr:
  for (ap_uint<12> r = 0; r < h0; r++) {
#pragma HLS LOOP_TRIPCOUNT min=480 max=480
  xd:
    for (ap_uint<10> d = 0; d < ow; d++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=512 max=512
      out.write(xt[d]);
    }
  }
}

static void hpass(hls::stream<px_t>& in, hls::stream<tab_t>& xs, ap_uint<23> npix, ap_uint<12> w0, ap_uint<21> nx,
                  bool bypass, hls::stream<h3_t>& out) {
  tab_t cur = 0;
  ap_uint<21> got = 0;
  if (!bypass) { cur = xs.read(); got = 1; }
  px_t prev = 0;
  ap_uint<12> x = 0;
hp:
  for (ap_uint<23> i = 0; i < npix; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=307200 max=307200
    px_t p = in.read();
    if (bypass) {
      out.write((ap_uint<15>(p(23, 16)), ap_uint<15>(p(15, 8)), ap_uint<15>(p(7, 0))));
    } else if (cur(11, 0) == x) {
      ap_uint<12> c0 = cur(23, 12), c1 = cur(35, 24);
      ap_uint<20> m00 = prev(7, 0) * c0, m01 = p(7, 0) * c1;
      ap_uint<20> m10 = prev(15, 8) * c0, m11 = p(15, 8) * c1;
      ap_uint<20> m20 = prev(23, 16) * c0, m21 = p(23, 16) * c1;
      ap_uint<15> h0v = (ap_uint<21>(m00) + m01) >> 4;
      ap_uint<15> h1v = (ap_uint<21>(m10) + m11) >> 4;
      ap_uint<15> h2v = (ap_uint<21>(m20) + m21) >> 4;
      out.write((h2v, h1v, h0v));
      if (got < nx) { cur = xs.read(); got++; }
    }
    prev = p;
    if (x == w0 - 1) x = 0; else x++;
  }
}

// ---------------- 5. 竖向 2 抽头 + 量化：1 行缓冲，同址先读（上一源行 H）后写（本行 H） ----------------
static void yfeed(const tab_t yt[PREP_MAX_OUT], ap_uint<10> oh, bool bypass, hls::stream<tab_t>& out) {
  if (bypass) return;
yd:
  for (ap_uint<10> d = 0; d < oh; d++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=384 max=384
    out.write(yt[d]);
  }
}

static ap_uint<8> vtap(ap_uint<12> c0, ap_uint<15> hp, ap_uint<12> c1, ap_uint<15> h) {
#pragma HLS INLINE
  ap_uint<27> a = c0 * hp;
  ap_uint<27> b = c1 * h;
  ap_uint<12> s = ap_uint<12>(a >> 16) + ap_uint<12>(b >> 16) + 2;
  ap_uint<10> v = s >> 2;
  return v > 255 ? ap_uint<8>(255) : ap_uint<8>(v);
}

// 量化：q = min(127, (x·K + C) >> 24)，K = 32897·2^(fp+1)、C = 255·32897，由驱动放进帧头。
// 与 256 项表 LUT[x] = min(127, floor((2x·2^fp + 255)/510)) 对 fp 0..7、x 0..255 全部相同（32897 = ceil(2^24/510)，
// 对 N < 65536 有 floor(N·32897 / 2^24) = floor(N/510)，宿主穷举 0 差）。每通道 1 个 DSP（x 8 位 × K 24 位 + C），替代 BRAM 查表。
static ap_uint<8> quant(ap_uint<8> x, ap_uint<24> k, ap_uint<24> c) {
#pragma HLS INLINE
  const ap_uint<32> p = x * k + c;
  const ap_uint<8> q = p(31, 24);
  return q > 127 ? ap_uint<8>(127) : q;
}

static void vpass(hls::stream<h3_t>& in, hls::stream<tab_t>& ys, ap_uint<12> h0, ap_uint<10> ow, ap_uint<10> oh,
                  bool bypass, ap_uint<24> qk, ap_uint<24> qc,
                  hls::stream<px_t>& out) {
  h3_t line[PREP_MAX_OUT];
#pragma HLS BIND_STORAGE variable=line type=ram_s2p impl=bram
  tab_t cy = 0;
  ap_uint<10> goty = 0;
  if (!bypass) { cy = ys.read(); goty = 1; }
  const ap_uint<21> n = h0 * ow;
  ap_uint<12> r = 0;
  ap_uint<10> dx = 0;
  bool emit = bypass;   // 源行 0 不可能是任何输出行的 y1（y1 >= 1）
vp:
  for (ap_uint<21> i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=196608 max=307200
#pragma HLS DEPENDENCE variable=line type=inter false
    h3_t h = in.read();
    h3_t hp = line[dx];
    if (emit) {
      ap_uint<8> v0, v1, v2;
      if (bypass) {
        v0 = h(7, 0); v1 = h(22, 15); v2 = h(37, 30);
      } else {
        ap_uint<12> c0 = cy(23, 12), c1 = cy(35, 24);
        v0 = vtap(c0, hp(14, 0), c1, h(14, 0));
        v1 = vtap(c0, hp(29, 15), c1, h(29, 15));
        v2 = vtap(c0, hp(44, 30), c1, h(44, 30));
      }
      out.write((quant(v2, qk, qc), quant(v1, qk, qc), quant(v0, qk, qc)));
    }
    line[dx] = h;
    if (dx == ow - 1) {
      dx = 0;
      if (emit && !bypass && goty < oh) { cy = ys.read(); goty++; }
      r++;
      emit = bypass || cy(11, 0) == r;
    } else {
      dx++;
    }
  }
}

// ---------------- 6. 灰边插入 + 齿轮箱 24->128：按张量字节序产出 ----------------
// 字节序：run0（上灰边整行 + 首行左灰边）-> [内容行 OW 像素 -> mid（右灰边 + 下一行左灰边）]... -> fin（末行右灰边 + 下灰边整行）
// 结构：18 字节移入寄存器 R，新字节从顶端移入（固定移位＝连线，不耗 LUT），有效字节数 k 个在顶端 [18-k,18)。
//   每拍移入 3 字节（一个像素或 3 个灰边字节），或在 k<=2 时整拍移入 16 个灰边字节（大段灰边仍 16 字节/拍）；
//   k>=16 就输出窗口 R[18-k .. 33-k]（只 3 种位置：3 选 1）。16 字节整拍三个一组用（48 字节），保证灰边余量始终是 3 的倍数。
static void pack(hls::stream<px_t>& in, ap_uint<10> ow, ap_uint<10> oh, ap_uint<8> padq, ap_uint<21> run0,
                 ap_uint<21> mid, ap_uint<21> fin, ap_uint<17> nwords, hls::stream<word_t>& out,
                 hls::stream<bool>& tok) {
  ap_uint<144> R = 0;
  ap_uint<5> k = 0;
  ap_uint<17> cnt = 0;
  bool inpad = run0 != 0;
  ap_uint<21> rem = run0;
  ap_uint<10> col = 0, row = 0;
  ap_uint<2> c16 = 0;
  const ap_uint<24> p3 = (padq, padq, padq);
  ap_uint<128> p16;
  for (int b = 0; b < 16; b++) {
#pragma HLS UNROLL
    p16(8 * b + 7, 8 * b) = padq;
  }
pk:
  while (cnt < nwords) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=49152 max=76800
    bool big = false;
    ap_uint<24> in3 = p3;
    if (inpad) {
      big = k <= 2 && (c16 != 0 || rem >= 48);
      rem -= big ? 16 : 3;
      if (big) c16 = (c16 == 2) ? ap_uint<2>(0) : ap_uint<2>(c16 + 1);
      if (rem == 0 && row != oh) inpad = false;
    } else {
      in3 = in.read();
      if (col == ow - 1) {
        col = 0;
        row++;
        ap_uint<21> nr = (row == oh) ? fin : mid;
        rem = nr;
        inpad = nr != 0;
      } else {
        col++;
      }
    }
    const ap_uint<144> Rn = big ? ap_uint<144>((p16, R(143, 128))) : ap_uint<144>((in3, R(143, 24)));
    ap_uint<5> kn = big ? ap_uint<5>(k + 16) : ap_uint<5>(k + 3);
    if (kn >= 16) {
      word_t w;
      if (kn == 18) w = Rn(127, 0);
      else if (kn == 17) w = Rn(135, 8);
      else w = Rn(143, 16);
      out.write(w);
      cnt++;
      if (cnt(5, 0) == 0 || cnt == nwords) tok.write(true);   // 每满 64 字（及帧末）发一个令牌给 wr
      kn -= 16;
    }
    R = Rn;
    k = kn;
  }
}

// ---------------- 7. 写张量：线性地址，整块（含灰边）每帧写满 ----------------
// pack 每产出 64 字（及帧末）发一个令牌；s_out 是 128 深 BRAM FIFO。wr 拿到令牌时这一块 64 字已经全在 FIFO 里，
// 于是 64 拍突发的数据连续给出、不占着写通道等数据——这就是适配器保守模式做的事，所以可以关掉保守模式
// （m_axi_conservative_mode=false），省掉适配器那个 64 深×145 位的 SRL 数据 FIFO。
static void wr(hls::stream<word_t>& in, hls::stream<bool>& tok, ap_uint<17> nwords, word_t* dst) {
  ap_uint<17> left = nwords, base = 0;
wb:
  while (left != 0) {
#pragma HLS LOOP_TRIPCOUNT min=768 max=1200
    const ap_uint<7> n = left > 64 ? ap_uint<7>(64) : ap_uint<7>(left);
    tok.read();
  wd:
    for (ap_uint<7> i = 0; i < n; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=64 max=64
      dst[base + i] = in.read();
    }
    base += n;
    left -= n;
  }
}

static void frame(const word_t* src, ap_uint<23> nrd, ap_uint<12> stride_w, ap_uint<12> nw_row, ap_uint<12> w0, ap_uint<12> h0,
                  ap_uint<2> fmt, bool ups, bool bypass, ap_uint<23> ntu, ap_uint<23> npix, const tab_t xt[PREP_MAX_OUT],
                  const tab_t yt[PREP_MAX_OUT], ap_uint<10> ow, ap_uint<10> oh, ap_uint<21> nx,
                  ap_uint<24> qk, ap_uint<24> qc, ap_int<20> ky,
                  ap_int<20> yoff, ap_int<20> krv, ap_int<20> kgu, ap_int<20> kgv, ap_int<20> kbu, ap_uint<8> padq,
                  ap_uint<21> run0, ap_uint<21> mid, ap_uint<21> fin, ap_uint<17> nwords, word_t* dst) {
#pragma HLS DATAFLOW
  // 标量与指针在一帧内不变（frame 每次 ap_start 只调一次）：标 STABLE，免得 HLS 给每个标量建同步 FIFO
#pragma HLS STABLE variable=src
#pragma HLS STABLE variable=nrd
#pragma HLS STABLE variable=stride_w
#pragma HLS STABLE variable=nw_row
#pragma HLS STABLE variable=w0
#pragma HLS STABLE variable=h0
#pragma HLS STABLE variable=fmt
#pragma HLS STABLE variable=ups
#pragma HLS STABLE variable=bypass
#pragma HLS STABLE variable=ntu
#pragma HLS STABLE variable=npix
#pragma HLS STABLE variable=ow
#pragma HLS STABLE variable=oh
#pragma HLS STABLE variable=nx
#pragma HLS STABLE variable=ky
#pragma HLS STABLE variable=yoff
#pragma HLS STABLE variable=krv
#pragma HLS STABLE variable=kgu
#pragma HLS STABLE variable=kgv
#pragma HLS STABLE variable=kbu
#pragma HLS STABLE variable=padq
#pragma HLS STABLE variable=qk
#pragma HLS STABLE variable=qc
#pragma HLS STABLE variable=run0
#pragma HLS STABLE variable=mid
#pragma HLS STABLE variable=fin
#pragma HLS STABLE variable=nwords
#pragma HLS STABLE variable=dst
  hls::stream<word_t> s_in("s_in");
  hls::stream<px_t> s_px("s_px");
  hls::stream<px_t> s_rgb("s_rgb");
  hls::stream<tab_t> s_xt("s_xt");
  hls::stream<h3_t> s_h("s_h");
  hls::stream<tab_t> s_yt("s_yt");
  hls::stream<px_t> s_q("s_q");
  hls::stream<word_t> s_out("s_out");
  hls::stream<bool> s_tok("s_tok");
#pragma HLS STREAM variable=s_out depth=128
#pragma HLS BIND_STORAGE variable=s_out type=fifo impl=bram
#pragma HLS STREAM variable=s_tok depth=4
  // 宽 FIFO 默认 2 深「移位寄存器」实现每位约 2 LUT（s13 OOC：128 位 s_in 251 LUT、36 位 s_xt/s_yt 各 77）；
  // 改放 LUTRAM（RAM32M16 每 14 位 8 LUT），深度随之给到 16，顺带恢复 s_in 对读侧的解耦
#pragma HLS STREAM variable=s_in depth=16
#pragma HLS BIND_STORAGE variable=s_in type=fifo impl=lutram
#pragma HLS STREAM variable=s_xt depth=16
#pragma HLS BIND_STORAGE variable=s_xt type=fifo impl=lutram
#pragma HLS STREAM variable=s_yt depth=16
#pragma HLS BIND_STORAGE variable=s_yt type=fifo impl=lutram
  rd_src(src, nrd, stride_w, nw_row, s_in);
  unpack(s_in, w0, ntu, fmt, ups, s_px);
  cvt(s_px, npix, fmt, ky, yoff, krv, kgu, kgv, kbu, s_rgb);
  xfeed(xt, ow, h0, bypass, s_xt);
  hpass(s_rgb, s_xt, npix, w0, nx, bypass, s_h);
  yfeed(yt, oh, bypass, s_yt);
  vpass(s_h, s_yt, h0, ow, oh, bypass, qk, qc, s_q);
  pack(s_q, ow, oh, padq, run0, mid, fin, nwords, s_out, s_tok);
  wr(s_out, s_tok, nwords, dst);
}

// ---------------- 参数装载：每帧都装（最多 643 字，约 6.5 us@100 MHz），不再有 PARAM_RELOAD 寄存器与「已装」状态 ----------------
// 参数块布局见 prep_k.h：字 0..2 帧头（几何与各级循环次数，A53 驱动算好），字 3+d 为下标 d 的表项。
static void load_params(const word_t* prm, ap_uint<10> n, tab_t xt[PREP_MAX_OUT], tab_t yt[PREP_MAX_OUT],
                        ap_int<20> kc[6]) {
ld:
  for (ap_uint<10> d = 0; d < n; d++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=256 max=640
    word_t w = prm[PREP_HDR_WORDS + d];
    xt[d] = w(35, 0);
    yt[d] = w(71, 36);
    if (d < 6) kc[d] = w(99, 80);
  }
}

void prep_top(const word_t* src, word_t* dst, const word_t* prm, ap_uint<32>* status) {
#pragma HLS INTERFACE mode=m_axi port=src offset=slave bundle=gmem depth=57600 max_read_burst_length=64 num_read_outstanding=2 max_write_burst_length=64 num_write_outstanding=2
#pragma HLS INTERFACE mode=m_axi port=dst offset=slave bundle=gmem depth=49152 max_read_burst_length=64 num_read_outstanding=2 max_write_burst_length=64 num_write_outstanding=2
#pragma HLS INTERFACE mode=m_axi port=prm offset=slave bundle=gmem depth=643 max_read_burst_length=64 num_read_outstanding=2 max_write_burst_length=64 num_write_outstanding=2
#pragma HLS INTERFACE mode=s_axilite port=status
#pragma HLS INTERFACE mode=s_axilite port=return
  static tab_t xt[PREP_MAX_OUT];
  static tab_t yt[PREP_MAX_OUT];
  static ap_int<20> kc[6];
  static ap_uint<16> frames = 0;
#pragma HLS BIND_STORAGE variable=xt type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=yt type=ram_1p impl=bram
#pragma HLS ARRAY_PARTITION variable=kc type=complete

  word_t hd[PREP_HDR_WORDS];
#pragma HLS ARRAY_PARTITION variable=hd type=complete
hdr:
  for (int k = 0; k < PREP_HDR_WORDS; k++) {
#pragma HLS PIPELINE II=1
    hd[k] = prm[k];
  }
  const ap_uint<16> magic = hd[0](15, 0);
  const ap_uint<12> w0 = hd[0](27, 16), h0 = hd[0](39, 28);
  const ap_uint<10> ow = hd[0](49, 40), oh = hd[0](59, 50);
  const ap_uint<2> fmt = hd[0](61, 60);
  const bool ups = hd[0][62], bypass = hd[0][63];
  const ap_uint<8> padq = hd[0](71, 64);
  const ap_uint<12> stride_w = hd[0](83, 72), nw_row = hd[0](95, 84);
  const ap_uint<17> nwords = hd[0](112, 96);
  const ap_uint<10> nload = hd[0](122, 113);
  const ap_uint<23> nrd = hd[1](22, 0), npix = hd[1](45, 23), ntu = hd[1](68, 46);
  const ap_uint<21> nx = hd[1](89, 69), run0 = hd[1](110, 90);
  const ap_uint<21> mid = hd[2](20, 0), fin = hd[2](41, 21);
  const ap_uint<24> qk = hd[2](65, 42), qc = hd[2](89, 66);

  // 核内只留最便宜、能防越界/死锁的几项（完整几何检查在驱动 prep_drv_header）；不合法就整帧不动、不写任何字节
  const bool bad = magic != PREP_MAGIC || ow == 0 || oh == 0 || ow > w0 || oh > h0 || nload > PREP_MAX_OUT;
  ap_uint<8> err = 0;
  if (bad) {
    err = PREP_ST_ERR_HDR;
  } else {
    load_params(prm, nload, xt, yt, kc);
    frame(src, nrd, stride_w, nw_row, w0, h0, fmt, ups, bypass, ntu, npix, xt, yt, ow, oh, nx, qk, qc, kc[0],
          kc[1], kc[2], kc[3], kc[4], kc[5], padq, run0, mid, fin, nwords, dst);
    frames++;
  }
  *status = (frames, ap_uint<8>(PREP_VERSION8), err);
}
