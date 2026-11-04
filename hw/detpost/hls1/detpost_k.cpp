// 检测后处理核（DETPOST1 第 4 步起的结构）。语义与定点参考模型 detpost_ref（CAND_MAX=256）逐位相同。
//
// 结构（相对参考模型的改写，逐项理由见 README）：
//   0. 参数块（两张查表＋类别项）每帧开头从 DDR 突发读进一块 BRAM（AXI-Lite 里不再放表）。
//   1. 扫描（dataflow 进程 scan）：128 位突发读整块 DPU 输出，每拍 16 字节；每拍对本格落在这一字里的类别字节做
//      16 路「取最大、同分取下标小」，跨字累计；一格结束时与阈值比，过阈值的格只把位置信息（不含 DFL）送进 FIFO。
//      一个 16 字节字里最多只有一个格结束（每格 >= 65 字节），而且下一格开头 16 字节以内都是 DFL 通道，
//      所以一字之内只需要算「当前格」的类别字节。
//   2. 候选管理（dataflow 进程 manage）：写候选表与 256 档直方图；候选满时按参考模型的规则整档丢最低档并抬阈值，
//      或在同一档内顶掉最后一个（逐位同参考模型）。
//   3. 计数排序（按 int8 分数档降序、同档按扫描序）。
//   4. 按序逐个候选：从 DDR 回读该格 64 字节 DFL logit → 查 exp 表 → 14 步移位减法 → 坐标；与已保留的同类框
//      逐个比 IoU（整数判定 inter*den > num*union）；保留即按白名单/下限过滤后写出一条框记录。
//      参考模型先解码所有候选再 NMS；这里只在 NMS 需要时解码，解码是格的纯函数，结果相同，省掉 256 行坐标存储。
#include <hls_stream.h>
#include "detpost_k.h"

using namespace dpk;

typedef ap_uint<64> crec_t;  // 候选记录：[19:0] 格起点字节偏移 [33:20] 锚点号 [40:34] x [47:41] y [49:48] 尺度 [55:50] 类别 [63:56] 分数档
typedef ap_uint<65> ctok_t;  // FIFO 令牌：bit64 = 结束

static void scan(const dp_word_t* in, ap_uint<6> nc, ap_int<9> thr, ap_uint<7> g80, ap_uint<7> g40, ap_uint<7> g20,
                 ap_uint<20> off80, ap_uint<20> off40, ap_uint<20> off20, hls::stream<ctok_t>& cs) {
  const ap_uint<7> ch = (ap_uint<7>)64 + nc;
  ap_uint<14> anchor = 0;
LEVEL:
  for (int l = 0; l < 3; l++) {
    const ap_uint<7> g = l == 0 ? g80 : (l == 1 ? g40 : g20);
    const ap_uint<20> off = l == 0 ? off80 : (l == 1 ? off40 : off20);
    const ap_uint<13> ncell = g * g;
    const ap_uint<4> s0 = off.range(3, 0);
    const ap_uint<21> nbytes = (ap_uint<21>)s0 + (ap_uint<21>)(ncell * ch);
    const ap_uint<17> nw = (nbytes + 15) >> 4;
    const ap_uint<16> wbase = off >> 4;
    ap_int<8> p = -(ap_int<8>)s0;  // 本字第 0 字节在当前格里的通道号（可为负：头偏移不对齐时字首几个字节不属于本头）
    ap_uint<7> x = 0, y = 0;
    ap_uint<20> coff = off;  // 当前格起点的字节偏移
    ap_int<8> rmax = -128;
    ap_uint<6> rcls = 0;
  WORDS:
    for (ap_uint<17> w = 0; w < nw; w++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=2425 max=38800
      const dp_word_t v = in[wbase + w];
      ap_int<8> vv[16];
      ap_uint<4> ix[16];
#pragma HLS ARRAY_PARTITION variable=vv type=complete
#pragma HLS ARRAY_PARTITION variable=ix type=complete
      // 第 7 步：屏蔽改成每字算一次的两个界（字节 j 是本格类别字节 <=> lo <= j < hi），树里只带 4 位字节号，
      // 类别号在树后一次算（= p + j - 64）。
      const ap_int<9> lo = (ap_int<9>)64 - p;
      const ap_int<9> hi = (ap_int<9>)ch - p;
      for (int j = 0; j < 16; j++) {
        const bool isc = (ap_int<9>)j >= lo && (ap_int<9>)j < hi;
        vv[j] = isc ? (ap_int<8>)v.range(8 * j + 7, 8 * j) : (ap_int<8>)-128;
        ix[j] = j;
      }
      // 16 路取最大：同分保留下标小的（= 通道号小 = 类别号小）
      for (int st = 1; st < 16; st *= 2) {
        for (int j = 0; j < 16; j += 2 * st) {
          if (vv[j + st] > vv[j]) { vv[j] = vv[j + st]; ix[j] = ix[j + st]; }
        }
      }
      const ap_uint<6> cbest = (ap_uint<6>)((ap_int<9>)p + ix[0] - 64);
      // 跨字：严格大于才换（早出现的通道优先）。被屏蔽的字节取 -128，永远不会严格大于 rmax（>= -128），
      // 所以全为 -128 的格得到类别 0、分数档 -128，与参考模型「qmax=pc[0]、cmax=0」一致。
      ap_int<8> m = rmax;
      ap_uint<6> mc = rcls;
      if (vv[0] > m) { m = vv[0]; mc = cbest; }
      if ((ap_int<9>)p + 16 >= (ap_int<9>)ch) {  // 本格最后一个字节在这一字里
        if ((ap_int<9>)m >= thr) {
          crec_t r;
          r.range(19, 0) = coff;
          r.range(33, 20) = anchor;
          r.range(40, 34) = x;
          r.range(47, 41) = y;
          r.range(49, 48) = l;
          r.range(55, 50) = mc;
          r.range(63, 56) = m;
          cs.write((ctok_t)r);
        }
        anchor++;
        coff += ch;
        if (x == g - 1) { x = 0; y++; } else { x++; }
        rmax = -128;
        rcls = 0;
        p = p + 16 - (ap_int<8>)ch;
      } else {
        rmax = m;
        rcls = mc;
        p += 16;
      }
    }
  }
  ctok_t last = 0;
  last[64] = 1;
  cs.write(last);
}

static void manage(hls::stream<ctok_t>& cs, ap_int<9> thr0, crec_t cand[CMAX], ap_uint<9> hist[256], ap_uint<9>& n_o,
                   ap_uint<14>& nseen_o, ap_uint<1>& ovf_o, ap_int<9>& thr_o) {
CLR:
  for (int i = 0; i < 256; i++) {
#pragma HLS PIPELINE II=1
    hist[i] = 0;
  }
  ap_uint<9> n = 0;
  ap_uint<14> nseen = 0;
  ap_uint<1> ovf = 0;
  ap_int<9> thr = thr0;
MAIN:
  for (;;) {
#pragma HLS LOOP_TRIPCOUNT min=0 max=8400
    const ctok_t t = cs.read();
    if (t[64]) break;
    const crec_t r = t.range(63, 0);
    const ap_int<8> lvl = r.range(63, 56);
    nseen++;
    if ((ap_int<9>)lvl < thr) continue;
    ap_uint<9> slot = n;
    if (n == CMAX) {
      // 候选满：规则逐位同参考模型 detpost_ref（README §6.3）
      ovf = 1;
      ap_uint<8> lo = 255;
    FIND:
      for (int i = 0; i < 256; i++) {
#pragma HLS PIPELINE II=1
        if (hist[i] != 0) { lo = i; break; }
      }
      const ap_uint<9> hlo = hist[lo];
      const ap_int<9> lov = (ap_int<9>)lo - 128;
      if (hlo < n) {
        thr = lov + 1;
        ap_uint<9> k = 0;
      COMPACT:
        for (int i = 0; i < CMAX; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=cand type=inter false
          const crec_t c = cand[i];
          if ((ap_int<9>)(ap_int<8>)c.range(63, 56) >= thr) { cand[k] = c; k++; }
        }
        hist[lo] = 0;
        n = k;
        slot = n;
        if ((ap_int<9>)lvl < thr) continue;
      } else {
        if ((ap_int<9>)lvl <= lov) continue;
        hist[lo] = hlo - 1;
        slot = n - 1;
        n = n - 1;
      }
    }
    cand[slot] = r;
    const ap_uint<8> hb = (ap_uint<8>)(lvl + 128);
    hist[hb] = hist[hb] + 1;
    n++;
  }
  n_o = n;
  nseen_o = nseen;
  ovf_o = ovf;
  thr_o = thr;
}

static void scan_manage(const dp_word_t* in, ap_uint<6> nc, ap_int<9> thr, ap_uint<7> g80, ap_uint<7> g40,
                        ap_uint<7> g20, ap_uint<20> off80, ap_uint<20> off40, ap_uint<20> off20, crec_t cand[CMAX],
                        ap_uint<9> hist[256], ap_uint<9>& n, ap_uint<14>& nseen, ap_uint<1>& ovf, ap_int<9>& thrf) {
#pragma HLS DATAFLOW
  hls::stream<ctok_t> cs;
#pragma HLS STREAM variable=cs depth=8
  scan(in, nc, thr, g80, g40, g20, off80, off40, off20, cs);
  manage(cs, thr, cand, hist, n, nseen, ovf, thrf);
}

// 一个候选的 DFL 解码：回读该格 64 字节 → 四边期望距离（格为单位，Q.10）
static void decode(const dp_word_t* in, ap_uint<20> coff, const ap_uint<16> tab[TAB_N], ap_uint<14> d[4]) {
  const ap_uint<16> wa = coff >> 4;
  const ap_uint<4> s0 = coff.range(3, 0);
  const ap_uint<3> nwr = (ap_uint<7>)(s0 + 64 + 15) >> 4;  // 4 或 5 个字
  dp_word_t wb[5];
#pragma HLS BIND_STORAGE variable=wb type=ram_1p impl=lutram
  ap_int<8> dq[64];
  ap_int<8> mx[4];
#pragma HLS ARRAY_PARTITION variable=mx type=complete
RD:
  for (int k = 0; k < nwr; k++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=4 max=5
    wb[k] = in[wa + k];
  }
  // 第 5 步：逐字节取数改成「每 16 拍从 LUTRAM 取一个字进移位寄存器、每拍右移 8 位取低字节」，
  // 去掉 80 选 1 的字节多路器与 640 位寄存器阵列。
  dp_word_t sh = 0;
EXT:
  for (int b = 0; b < 80; b++) {
#pragma HLS PIPELINE II=1
    if ((b & 15) == 0) sh = wb[b >> 4]; else sh >>= 8;
    const ap_int<8> kk = (ap_int<8>)b - (ap_int<8>)s0;
    if (kk >= 0 && kk < 64) {
      const ap_int<8> q = sh.range(7, 0);
      dq[kk] = q;
      const ap_uint<2> t = kk.range(5, 4);
      if (kk.range(3, 0) == 0 || q > mx[t]) mx[t] = q;
    }
  }
  ap_uint<20> S[4];
  ap_uint<24> W[4];
#pragma HLS ARRAY_PARTITION variable=S type=complete
#pragma HLS ARRAY_PARTITION variable=W type=complete
  ap_uint<20> s = 0;
  ap_uint<24> w = 0;
ACC:
  for (int k = 0; k < 64; k++) {
#pragma HLS PIPELINE II=1
    const ap_uint<2> t = k >> 4;
    const ap_uint<4> i = k & 15;
    const ap_uint<8> dd = (ap_uint<8>)(mx[t] - dq[k]);  // 0..255
    const ap_uint<16> e = tab[dd];
    s += e;
    w += i * e;
    if (i == 15) { S[t] = s; W[t] = w; s = 0; w = 0; }
  }
  // 四舍五入的 (W << 10) / S：商 <= 15360 < 2^14（W <= 15*S），被除数 N < 2^14 * S，14 步恢复余数除法
SIDE:
  for (int t = 0; t < 4; t++) {
    const ap_uint<34> N = ((ap_uint<34>)W[t] << 10) + (S[t] >> 1);
    const ap_uint<20> sv = S[t];
    ap_uint<21> rem = N >> 14;
    ap_uint<14> qt = 0;
  DIV:
    for (int k = 13; k >= 0; k--) {
#pragma HLS PIPELINE II=1
      rem = (rem << 1) | N[k];
      const bool ge = rem >= sv;
      if (ge) rem -= sv;
      qt[k] = ge;
    }
    d[t] = qt;
  }
}

// Q.10 格坐标（已乘 512 的锚点中心加减距离）→ Q4 像素坐标，四舍五入：l=0/1/2 对应 stride 8/16/32
static ap_int<16> q4(ap_int<19> u, ap_uint<2> l) {
#pragma HLS INLINE
  const ap_int<19> r0 = (u + 4) >> 3, r1 = (u + 2) >> 2, r2 = (u + 1) >> 1;
  return (ap_int<16>)(l == 0 ? r0 : (l == 1 ? r1 : r2));
}

void detpost_top(const dp_word_t* in, const dp_word_t* prm, dp_word_t* out, ap_uint<6> nc, ap_int<9> thr,
                 ap_uint<8> iou_num, ap_uint<8> iou_den, ap_uint<8> max_det, ap_uint<7> g80, ap_uint<7> g40,
                 ap_uint<7> g20, ap_uint<20> off80, ap_uint<20> off40, ap_uint<20> off20) {
// 第 8 步：m_axi 参数收窄。写只有 <=130 个单字写（框表），1 个在途、突发 2 足够；读保留 8 个在途 x 16 拍（128 拍在途），
// 读缓冲宽 128 位、深 128 仍是同样的 BRAM，扫描吞吐的风险最小（DDR 往返 <128 拍时不掉速）。
#pragma HLS INTERFACE mode=m_axi port=in offset=slave bundle=gmem depth=50944 num_read_outstanding=8 max_read_burst_length=16 num_write_outstanding=1 max_write_burst_length=2
#pragma HLS INTERFACE mode=m_axi port=prm offset=slave bundle=gmem depth=69 num_read_outstanding=8 max_read_burst_length=16 num_write_outstanding=1 max_write_burst_length=2
#pragma HLS INTERFACE mode=m_axi port=out offset=slave bundle=gmem depth=130 num_read_outstanding=8 max_read_burst_length=16 num_write_outstanding=1 max_write_burst_length=2
#pragma HLS INTERFACE mode=s_axilite port=nc
#pragma HLS INTERFACE mode=s_axilite port=thr
#pragma HLS INTERFACE mode=s_axilite port=iou_num
#pragma HLS INTERFACE mode=s_axilite port=iou_den
#pragma HLS INTERFACE mode=s_axilite port=max_det
#pragma HLS INTERFACE mode=s_axilite port=g80
#pragma HLS INTERFACE mode=s_axilite port=g40
#pragma HLS INTERFACE mode=s_axilite port=g20
#pragma HLS INTERFACE mode=s_axilite port=off80
#pragma HLS INTERFACE mode=s_axilite port=off40
#pragma HLS INTERFACE mode=s_axilite port=off20
#pragma HLS INTERFACE mode=s_axilite port=return
  ap_uint<16> tab[TAB_N];  // [0,256) exp_lut，[256,512) sig_lut，[512,552) 类别项
  crec_t cand[CMAX];
  ap_uint<9> hist[256];
  ap_uint<8> order[CMAX];
  ap_uint<64> kbox[KMAX];  // 已保留框：x1,y1,x2,y2（int16 Q4）
  ap_uint<34> kac[KMAX];   // 已保留框：[27:0] 面积，[33:28] 类别

LOAD_PRM:
  for (int w = 0; w < PRM_WORDS; w++) {
#pragma HLS PIPELINE II=8
    const dp_word_t v = prm[w];
    for (int k = 0; k < 8; k++) tab[w * 8 + k] = v.range(16 * k + 15, 16 * k);
  }

  ap_uint<9> n;
  ap_uint<14> nseen;
  ap_uint<1> ovf;
  ap_int<9> thrf;
  scan_manage(in, nc, thr, g80, g40, g20, off80, off40, off20, cand, hist, n, nseen, ovf, thrf);

  // 计数排序：档位从高到低的前缀和，再按候选表顺序（= 扫描序）散列，得到稳定的降序下标表
  ap_uint<9> acc = 0;
PFX:
  for (int i = 255; i >= 0; i--) {
#pragma HLS PIPELINE II=1
    const ap_uint<9> h = hist[i];
    hist[i] = acc;
    acc += h;
  }
SCAT:
  for (ap_uint<9> i = 0; i < n; i++) {
#pragma HLS PIPELINE
#pragma HLS LOOP_TRIPCOUNT min=0 max=256
    const ap_uint<8> hb = (ap_uint<8>)((ap_int<8>)cand[i].range(63, 56) + 128);
    const ap_uint<9> p = hist[hb];
    order[(ap_uint<8>)p] = i;
    hist[hb] = p + 1;
  }

  ap_uint<8> nk = 0, no = 0;
NMS:
  for (ap_uint<9> r = 0; r < n; r++) {
#pragma HLS LOOP_TRIPCOUNT min=0 max=256
    if (nk == max_det) break;
    const crec_t c = cand[order[(ap_uint<8>)r]];
    const ap_uint<20> coff = c.range(19, 0);
    const ap_uint<14> anchor = c.range(33, 20);
    const ap_uint<7> x = c.range(40, 34), y = c.range(47, 41);
    const ap_uint<2> l = c.range(49, 48);
    const ap_uint<6> cls = c.range(55, 50);
    const ap_int<8> lvl = c.range(63, 56);
    ap_uint<14> d[4];
#pragma HLS ARRAY_PARTITION variable=d type=complete
    decode(in, coff, tab, d);
    // 坐标：锚点中心 (x+0.5)*stride 的 Q.10 = (2x+1) << (9 + log2 stride)；距离 × stride = d << log2 stride；
    // Q.10 → Q4 四舍五入（加 32 算术右移 6）。第 6 步改写为整数恒等式（sh = 3 + l <= 5）：
    //   ((u << sh) + 32) >> 6 == (u + 2^(5-sh)) >> (6-sh)，u = (2x+1)*512 -/+ d
    // 每个坐标只剩三个常数移位按尺度选一个，去掉可变移位器。范围见 README：int16 足够（输入 <= 640）。
    const ap_int<19> cx = (ap_int<19>)((ap_uint<18>)(2 * x + 1) << 9);
    const ap_int<19> cy = (ap_int<19>)((ap_uint<18>)(2 * y + 1) << 9);
    const ap_int<16> x1 = q4(cx - (ap_int<19>)d[0], l);
    const ap_int<16> y1 = q4(cy - (ap_int<19>)d[1], l);
    const ap_int<16> x2 = q4(cx + (ap_int<19>)d[2], l);
    const ap_int<16> y2 = q4(cy + (ap_int<19>)d[3], l);
    const ap_uint<28> area = (ap_uint<14>)(x2 - x1) * (ap_uint<14>)(y2 - y1);
    bool keep = true;
  CMP:
    for (ap_uint<8> j = 0; j < nk; j++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=0 max=100
      const ap_uint<64> kb = kbox[j];
      const ap_uint<34> ka = kac[j];
      const ap_int<16> kx1 = kb.range(15, 0), ky1 = kb.range(31, 16), kx2 = kb.range(47, 32), ky2 = kb.range(63, 48);
      const ap_int<17> iw = (ap_int<17>)(x2 < kx2 ? x2 : kx2) - (ap_int<17>)(x1 > kx1 ? x1 : kx1);
      const ap_int<17> ih = (ap_int<17>)(y2 < ky2 ? y2 : ky2) - (ap_int<17>)(y1 > ky1 ? y1 : ky1);
      if (ka.range(33, 28) == cls && iw > 0 && ih > 0) {
        const ap_uint<28> inter = (ap_uint<14>)iw * (ap_uint<14>)ih;
        const ap_uint<29> uni = (ap_uint<29>)area + (ap_uint<28>)ka.range(27, 0) - inter;
        if ((ap_uint<36>)inter * iou_den > (ap_uint<37>)(iou_num * uni)) keep = false;
      }
    }
    if (!keep) continue;
    ap_uint<64> kb;
    kb.range(15, 0) = x1; kb.range(31, 16) = y1; kb.range(47, 32) = x2; kb.range(63, 48) = y2;
    kbox[nk] = kb;
    ap_uint<34> ka;
    ka.range(27, 0) = area;
    ka.range(33, 28) = cls;
    kac[nk] = ka;
    nk++;
    const ap_uint<16> ce = tab[TAB_CLS + cls];
    if (ce[8] && lvl >= (ap_int<8>)ce.range(7, 0)) {
      dp_word_t o = 0;
      o.range(63, 0) = kb;
      o.range(71, 64) = cls;
      o.range(79, 72) = lvl;
      o.range(95, 80) = tab[TAB_SIG + (ap_uint<8>)(lvl + 128)];
      o.range(111, 96) = anchor;
      out[OUT_HDR + no] = o;
      no++;
    }
  }
  dp_word_t h0 = 0, h1 = 0;
  h0.range(31, 0) = nseen;
  h0.range(63, 32) = n;
  h0.range(95, 64) = ovf;
  h0.range(127, 96) = (ap_int<32>)thrf;
  h1.range(31, 0) = nk;
  h1.range(63, 32) = no;
  out[0] = h0;
  out[1] = h1;
}
