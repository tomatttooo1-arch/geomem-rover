// MEMK0 semantic-memory core: fixed-point numeric part of perception/fpga_replay/core.py Memory (7fe36e1).
// One source for the C model (g++ -I$XILINX_HLS/include, ctypes) and the HLS kernel. Design: MEMK0/DESIGN.md;
// HLS form (MEMK3): every unit has exactly one call site. The op loop decodes each opcode into action bits
// (constant ROM) and runs the actions in a fixed order, so add-sample, fusion, existence, the slot scan engine,
// the divider and the square root each exist once in hardware. ASSOC is an internal opcode of the same loop.
// State (slot tables, sample rings, hit rings) persists across calls in static arrays.
#include "memk0.h"
#include "memk0_consts.h"
#include "memk0_types.h"
#ifdef MEMK0_CHI2_24_OVR                                // positive control (a) of the key comparison only
#define K_CHI2 ((long long)(MEMK0_CHI2_24_OVR))
#else
#define K_CHI2 K_CHI2_24
#endif
#ifndef MK_PIPE_SQRT
#define MK_PIPE_SQRT 0                                  // 0: sequential digit recurrence (LUT-light); 1: Newton
#endif

// ------------------------------------------------------------------ persistent state
struct SlotPos { pos_t p[3]; sf2_t sf2; bool fval; ns_t ns; hd_t head; };     // fusion / sample ops / copies
struct SlotMeta { cls_t cls; type_t type; odo_t op, ot; };                      // touch / new / copies
struct SlotEx {                                                                 // existence / copies
  lo_t L; bool lval; win_t win; bool wval; bool hit, fre, mval; pd_t mpd;
  ap_uint<5> rh, rc; bool rval; win_t lh; bool lhval; ap_uint<5> rn; ap_uint<4> rhd;
};
struct Sample { skey_t key; pos_t p[3]; w_t w; };
struct RingE { win_t w; bool h; };
#define MK_SCR (MK_SLOTS * MK_KSAMP)                    // merge scratch rows after the 64 rings

static SlotPos S_POS[MK_SLOTS];
static SlotMeta S_META[MK_SLOTS];
static SlotEx S_EX[MK_SLOTS];
static Sample SAMP[MK_SLOTS * MK_KSAMP + MK_KSAMP];
static RingE RING[MK_SLOTS * MK_RING];
static ap_uint<6> N_OBS_LAST;


// explicit zero records (ap_int members are not value-initialised by "= {}")
static SlotPos zpos() { SlotPos z; z.p[0] = 0; z.p[1] = 0; z.p[2] = 0; z.sf2 = 0; z.fval = false; z.ns = 0; z.head = 0; return z; }
static SlotMeta zmeta(int cls, int type) { SlotMeta z; z.cls = cls; z.type = type; z.op = 0; z.ot = 0; return z; }
static SlotEx zex() {
  SlotEx z; z.L = 0; z.lval = false; z.win = 0; z.wval = false; z.hit = false; z.fre = false; z.mval = false; z.mpd = 0;
  z.rh = 0; z.rc = 0; z.rval = false; z.lh = 0; z.lhval = false; z.rn = 0; z.rhd = 0; return z;
}

static inline i64 imin(i64 a, i64 b) { return a < b ? a : b; }
static inline i64 imax(i64 a, i64 b) { return a > b ? a : b; }
typedef ap_int<34> s34; typedef ap_int<30> s30; typedef ap_int<21> s21;
static inline s34 min34(s34 a, s34 b) { return a < b ? a : b; }
static inline s30 min30(s30 a, s30 b) { return a < b ? a : b; }
static inline s30 max30(s30 a, s30 b) { return a > b ? a : b; }
static inline s21 min21(s21 a, s21 b) { return a < b ? a : b; }
static inline s21 max21(s21 a, s21 b) { return a > b ? a : b; }

// ------------------------------------------------------------------ arithmetic units (one instance each)
// SDU: restoring division, one quotient bit per step: q = floor(n / d), also whether the remainder is zero
static void sdu_div(ap_uint<62> n, ap_uint<51> d, ap_uint<62> &q, bool &exact) {
#pragma HLS inline off
  ap_uint<52> rem = 0; ap_uint<62> nn = n, qq = 0;
DIV: for (int k = 0; k < 62; k++) {
#pragma HLS pipeline off
    rem = (rem << 1) | (ap_uint<52>)nn[61];
    nn <<= 1;
    ap_int<53> t = (ap_int<53>)rem - (ap_int<53>)d;
    bool ge = t >= 0;
    if (ge) rem = (ap_uint<52>)t;
    qq = (qq << 1) | (ap_uint<62>)ge;
  }
  q = qq; exact = rem == 0;
}
// sequential square root (digit recurrence): floor(sqrt(x)), x < 2^58
static d_t isqrt_seq(sq_t x) {
  ap_uint<33> rem = 0; ap_uint<30> root = 0; sq_t v = x;
SQ: for (int k = 0; k < 29; k++) {
#pragma HLS pipeline off
    rem = (rem << 2) | (ap_uint<33>)v(57, 56);
    v <<= 2;
    ap_int<34> t = (ap_int<34>)rem - (ap_int<34>)(((ap_uint<33>)root << 2) | 1);
    bool ge = t >= 0;
    if (ge) rem = (ap_uint<33>)t;
    root = (root << 1) | (ap_uint<30>)ge;
  }
  return (d_t)root;
}
// pipelined Newton square root (DESIGN 3.3): normalise, 1/sqrt seed ROM, two Newton steps, one-step exact
// floor correction (y*y > x -> y-1; else (y+1)^2 <= x -> y+1). Same result as isqrt_seq (checked on 200k values).
#include "memk0_rsqrt_rom.h"
static d_t isqrt_newton(sq_t x) {
  if (x == 0) return 0;
  ap_uint<64> x64 = x;
  int lz = x64.countLeadingZeros();
  int e = (63 - lz) >> 1;                              // x in [4^e, 4^(e+1))
  ap_uint<32> m = (e >= 15) ? (ap_uint<32>)(x >> (2 * e - 30)) : (ap_uint<32>)(x << (30 - 2 * e));   // Q2.30
  int idx = (int)((m >> 20) & 0xFFF) - 1024;
  ap_uint<64> r = (ap_uint<64>)RSQRT_ROM[idx >> 2] << 12;
NT: for (int it = 0; it < 2; it++) {
    ap_uint<64> r2 = (r * r) >> 30;
    ap_uint<64> mr2 = ((ap_uint<64>)m * r2) >> 30;
    ap_uint<64> t = ((ap_uint<64>)3 << 30) - mr2;
    r = (r * t) >> 31;
  }
  ap_uint<64> s = ((ap_uint<64>)m * r) >> 30;
  ap_uint<64> y = (e >= 30) ? (ap_uint<64>)(s << (e - 30)) : (ap_uint<64>)(s >> (30 - e));
  ap_uint<64> yy = y * y;
  if (yy > x) y = y - 1;
  else if (yy + 2 * y + 1 <= x) y = y + 1;
  return (d_t)y;
}
static d_t isqrt(sq_t x) {
#pragma HLS inline off
#if MK_PIPE_SQRT
  return isqrt_newton(x);
#else
  return isqrt_seq(x);
#endif
}

// ------------------------------------------------------------------ frame header
struct Frame { odo_t op, ot; win_t W; skey_t seq; pos_t T[3]; ap_int<26> R2[3]; pos_t ground; bool pose, det, skip; };

// ------------------------------------------------------------------ actions of one opcode (constant ROM)
enum {
  A_CLEAR = 1 << 0, A_MERGE_S = 1 << 1, A_COPY_S = 1 << 2, A_CLR_S = 1 << 3, A_COPY_EX = 1 << 4,
  A_COPY_POS = 1 << 5, A_ADD = 1 << 6, A_FUSE = 1 << 7, A_ODO = 1 << 8, A_EXHIT = 1 << 9, A_SETL = 1 << 10,
  A_RESETL = 1 << 11, A_FREE = 1 << 12, A_PREP = 1 << 13, A_SCAN = 1 << 14, A_GREEDY = 1 << 15,
  A_EXROLL = 1 << 16, A_DUMP = 1 << 17, A_VALID = 1 << 18, A_NEEDOBS = 1 << 19, A_BFREE = 1 << 20
};
#define OP_ASSOC_INT 16                                 // internal: the whole ASSOC call
static const ap_uint<21> ACT[17] = {
  0,                                                                       // END
  A_VALID | A_NEEDOBS | A_ADD | A_FUSE | A_ODO | A_EXHIT,                  // TOUCH (ADD only with flag bit 0)
  A_VALID | A_NEEDOBS | A_ADD | A_FUSE | A_ODO,                            // CAND
  A_VALID | A_BFREE | A_CLEAR,                                             // NEW
  A_VALID | A_COPY_S,                                                      // COPY_SAMPLES
  A_VALID | A_CLR_S,                                                       // CLEAR_SAMPLES
  A_VALID | A_MERGE_S | A_COPY_EX | A_FUSE,                                // MERGE (COPY_EX only with flag bit 0)
  A_VALID | A_COPY_S | A_COPY_EX | A_COPY_POS,                             // COPY_PLACE
  A_VALID | A_RESETL,                                                      // RESET_L
  A_VALID | A_FREE,                                                        // FREE
  A_VALID | A_BFREE | A_CLEAR | A_ADD | A_FUSE | A_ODO | A_SETL,           // RESTORE (two op slots)
  A_VALID | A_NEEDOBS | A_SCAN,                                            // QUERY (NEEDOBS unless kind 2)
  A_VALID | A_SCAN,                                                        // MERGE_SCAN
  A_VALID | A_SCAN | A_EXROLL,                                             // EXIST
  A_VALID | A_DUMP,                                                        // DUMP
  0,                                                                       // STOP
  A_VALID | A_PREP | A_SCAN | A_GREEDY                                     // ASSOC (internal)
};
enum { SM_ASSOC = 0, SM_QUERY = 1, SM_MSCAN = 2, SM_BLOCK = 3 };

// ------------------------------------------------------------------ top
void memk0_top(const u32 cmd[MK_CMD_WORDS], u32 res[MK_RES_WORDS]) {
#pragma HLS interface mode=s_axilite port=cmd
#pragma HLS interface mode=s_axilite port=res
#pragma HLS interface mode=s_axilite port=return
#pragma HLS aggregate variable=S_POS
#pragma HLS aggregate variable=S_META
#pragma HLS aggregate variable=S_EX
#pragma HLS bind_storage variable=S_POS type=ram_1p impl=lutram
#pragma HLS bind_storage variable=S_META type=ram_1p impl=lutram
#pragma HLS bind_storage variable=S_EX type=ram_1p impl=lutram
#pragma HLS aggregate variable=SAMP
#pragma HLS bind_storage variable=SAMP type=ram_t2p impl=uram
#pragma HLS aggregate variable=RING
#pragma HLS bind_storage variable=RING type=ram_1p impl=bram
#pragma HLS bind_storage variable=LN_TAB type=rom_1p impl=bram
#pragma HLS allocation function instances=sdu_div limit=1
#pragma HLS allocation function instances=isqrt limit=1
  // per-call local tables (LUTRAM): per observation [reason 1:0 | dup 2 | elig 3 | kept 4 | oused 5 |
  // rank 10:6 | assigned 17:11], dedupe order, per slot [sused 0 | blocked 1 | below 2 | free_at_vote 3]
  ap_uint<18> OBSL[MK_OBS];
  ap_uint<5> ORD[MK_OBS];
  ap_uint<4> SLOTL[MK_SLOTS];
#pragma HLS bind_storage variable=OBSL type=ram_1p impl=lutram
#pragma HLS bind_storage variable=ORD type=ram_1p impl=lutram
#pragma HLS bind_storage variable=SLOTL type=ram_1p impl=lutram
  ap_uint<56> DD[MK_KSAMP];
#pragma HLS bind_storage variable=DD type=ram_t2p impl=bram

  u32 h0 = cmd[0], h1 = cmd[1];
  int cmdk = (h0 >> 16) & 15, n = h1 & 0xFF, mu = (h1 >> 8) & 0xFF, nops = (h1 >> 16) & 0xFF;
  u32 status = 0;
  int npairs = 0, ndump = 0;
  bool run = true;
  if ((h0 & 0xFFFF) != MK_MAGIC) { status = ERR_MAGIC; run = false; }
  else if (n > MK_OBS || mu > MK_SLOTS || nops > MK_OPS) { status = ERR_SIZE; run = false; }
  else if (cmdk == MK_CMD_EXEC && n > (int)N_OBS_LAST) { status = ERR_SIZE; run = false; }
  else if (cmdk != MK_CMD_INIT && cmdk != MK_CMD_ASSOC && cmdk != MK_CMD_EXEC) { status = ERR_OP; run = false; }

  Frame F;
  {
    u32 fl = (h0 >> 20) & 0xFF;
    F.pose = fl & MK_F_POSE; F.det = (fl & MK_F_DET) != 0; F.skip = (fl & MK_F_SKIPMISS) != 0;
    F.W = (int)cmd[2]; F.seq = cmd[3] & 0xFFFFFF; F.op = cmd[4]; F.ot = cmd[5]; F.ground = (int)cmd[6];
  HDR: for (int k = 0; k < 3; k++) { F.T[k] = (int)cmd[8 + k]; F.R2[k] = (int)cmd[11 + k]; }
  }
  if (run && cmdk == MK_CMD_INIT) {
  INIT: for (int j = 0; j < MK_SLOTS; j++) {
      S_POS[j] = zpos(); S_META[j] = zmeta(0, 0); S_EX[j] = zex();
    }
    N_OBS_LAST = 0;
    run = false;
  }
SL0: for (int j = 0; j < MK_SLOTS; j++) SLOTL[j] = 0;
  int nloop = (cmdk == MK_CMD_ASSOC) ? 1 : nops;
  if (!run) nloop = 0;

OPS: for (int k = 0; k < nloop; k++) {
#pragma HLS loop_tripcount max=58
    u32 p0 = 0, f = 0, p2 = 0, p3 = 0, q0 = 0, q1 = 0;
    int opc;
    if (cmdk == MK_CMD_ASSOC) opc = OP_ASSOC_INT;
    else {
      int base = MK_OP_BASE + MK_OP_W * k;
      p0 = cmd[base]; f = cmd[base + 1]; p2 = cmd[base + 2]; p3 = cmd[base + 3];
      opc = p0 & 0xFF;
    }
    int a = (p0 >> 8) & 0xFF, b = (p0 >> 16) & 0xFF, i = (p0 >> 24) & 0xFF;
    if (opc == OP_STOP || opc == OP_END) break;
    ap_uint<21> act = (opc <= OP_ASSOC_INT) ? ACT[opc] : (ap_uint<21>)0;
    if (!(act & A_VALID)) { status |= ERR_OP; break; }
    bool needobs = (act & A_NEEDOBS) && !(opc == OP_QUERY && (f & 3) == 2);   // QUERY kind 2 reads no observation
    if (a >= MK_SLOTS || (b >= MK_SLOTS && !(act & A_BFREE)) || i >= MK_OBS || (needobs && i >= (int)N_OBS_LAST)) {
      status |= ERR_SLOT; break;
    }
    if (opc == OP_RESTORE) {
      if (k + 1 >= nops) { status |= ERR_OP; break; }
      int base2 = MK_OP_BASE + MK_OP_W * (k + 1);
      q0 = cmd[base2]; q1 = cmd[base2 + 1];
      k++;
    }
    if (opc == OP_TOUCH && !(f & 1)) act &= ~(ap_uint<21>)A_ADD;
    if (opc == OP_MERGE && !(f & 1)) act &= ~(ap_uint<21>)A_COPY_EX;

    // ---- observation i (position and class from the command block, sd and weight from the result block)
    pos_t op_[3]; w_t ow = 0;
    if (act & (A_ADD | A_SCAN)) {
      int ob = MK_OBS_BASE + MK_OBS_W * i;
    OL: for (int x = 0; x < 3; x++) op_[x] = (int)cmd[ob + 2 + x];
      ow = res[MK_RES_OBS + MK_RES_OBS_W * i + 2];
    }

    // ---- A_CLEAR: NEW / RESTORE (class b, type flags[1:0])
    if (act & A_CLEAR) {
      S_POS[a] = zpos(); S_META[a] = zmeta(b & 0x7F, f & 3); S_EX[a] = zex();
    }
    // ---- A_MERGE_S: sorted(keep + drop, key=t)[-K:], stable; tails first into the scratch rows, then back
    if (act & A_MERGE_S) {
      SlotPos pa = S_POS[a], pb = S_POS[b];
      int ia = pa.ns, ib = pb.ns, nn = 0;
    MG: for (int k2 = 0; k2 < 2 * MK_KSAMP; k2++) {
        if (nn < K_FUSE_KEEP && (ia > 0 || ib > 0)) {
          Sample sa = SAMP[a * MK_KSAMP + ((pa.head + ia - 1) & (MK_KSAMP - 1))];
          Sample sb = SAMP[b * MK_KSAMP + ((pb.head + ib - 1) & (MK_KSAMP - 1))];
          bool takeb = ib > 0 && (ia == 0 || sb.key >= sa.key);
          SAMP[MK_SCR + nn] = takeb ? sb : sa;
          nn++;
          if (takeb) ib--; else ia--;
        }
      }
    WB: for (int k2 = 0; k2 < MK_KSAMP; k2++) {
        if (k2 < nn) SAMP[a * MK_KSAMP + k2] = SAMP[MK_SCR + nn - 1 - k2];
      }
      pa.ns = nn; pa.head = 0; S_POS[a] = pa;
    }
    // ---- A_COPY_S: ring copy b -> a
    if (act & A_COPY_S) {
    CS: for (int k2 = 0; k2 < MK_KSAMP; k2++) {
        SAMP[a * MK_KSAMP + k2] = SAMP[b * MK_KSAMP + k2];
      }
      SlotPos pa = S_POS[a], pb = S_POS[b];
      pa.ns = pb.ns; pa.head = pb.head; S_POS[a] = pa;
    }
    if (act & A_CLR_S) { SlotPos pa = S_POS[a]; pa.ns = 0; pa.head = 0; S_POS[a] = pa; }
    // ---- A_COPY_EX: existence block, hit ring and odometer stamps b -> a
    if (act & A_COPY_EX) {
      S_EX[a] = S_EX[b];
    CR: for (int k2 = 0; k2 < MK_RING; k2++) RING[a * MK_RING + k2] = RING[b * MK_RING + k2];
      SlotMeta ma = S_META[a], mb = S_META[b];
      ma.op = mb.op; ma.ot = mb.ot; S_META[a] = ma;
    }
    if (act & A_COPY_POS) {
      SlotPos pa = S_POS[a], pb = S_POS[b];
      pa.sf2 = pb.sf2; pa.fval = pb.fval; pa.p[0] = pb.p[0]; pa.p[1] = pb.p[1]; pa.p[2] = pb.p[2];
      S_POS[a] = pa;
    }
    // ---- A_ADD: ring tail; full -> the oldest leaves (del samples[:-K]). RESTORE: op words, w 400 (sd 0.05)
    if (act & A_ADD) {
      Sample s;
      if (opc == OP_RESTORE) { s.key = 0; s.p[0] = (int)p2; s.p[1] = (int)p3; s.p[2] = (int)q0; s.w = 400 << 12; }
      else { s.key = F.seq; s.p[0] = op_[0]; s.p[1] = op_[1]; s.p[2] = op_[2]; s.w = ow; }
      SlotPos pa = S_POS[a];
      SAMP[a * MK_KSAMP + ((pa.head + pa.ns) & (MK_KSAMP - 1))] = s;
      if (pa.ns < K_FUSE_KEEP) pa.ns++; else pa.head++;
      S_POS[a] = pa;
    }
    // ---- A_FUSE: rank-count medians, MAD by rank count, inverse-variance mean of the inliers
    if (act & A_FUSE) {
      SlotPos pa = S_POS[a];
      int nn = pa.ns;
      if (nn > 0) {
        int k1 = (nn - 1) >> 1, k2 = nn >> 1;
        int base = a * MK_KSAMP;
        med_t v1[3] = {0, 0, 0}, v2[3] = {0, 0, 0}, med[3];
        ap_uint<6> lt[3], le[3];
      MD: for (int x = 0; x < nn; x++) {
#pragma HLS loop_tripcount max=20
        MDI: for (int y = 0; y < nn; y++) {
#pragma HLS loop_tripcount max=20
            Sample sx = SAMP[base + ((pa.head + x) & (MK_KSAMP - 1))];
            Sample sy = SAMP[base + ((pa.head + y) & (MK_KSAMP - 1))];
            for (int ax = 0; ax < 3; ax++) {
              ap_uint<6> l0 = y == 0 ? (ap_uint<6>)0 : lt[ax], e0 = y == 0 ? (ap_uint<6>)0 : le[ax];
              l0 += sy.p[ax] < sx.p[ax]; e0 += sy.p[ax] <= sx.p[ax];
              lt[ax] = l0; le[ax] = e0;
              if (y == nn - 1) {
                if (l0 <= k1 && k1 < e0) v1[ax] = sx.p[ax];
                if (l0 <= k2 && k2 < e0) v2[ax] = sx.p[ax];
              }
            }
          }
        }
      MS: for (int ax = 0; ax < 3; ax++) med[ax] = v1[ax] + v2[ax];               // 2^-19
      DL: for (int x = 0; x < nn; x++) {
#pragma HLS loop_tripcount max=20
          Sample s = SAMP[base + ((pa.head + x) & (MK_KSAMP - 1))];
          ap_uint<56> acc = 0;
          for (int ax = 0; ax < 3; ax++) { ap_int<28> dl = 2 * (ap_int<28>)s.p[ax] - med[ax]; acc += (ap_uint<56>)(dl * dl); }
          DD[x] = acc;                                                              // 2^-38
        }
        ap_uint<56> m1 = 0, m2 = 0; ap_uint<6> clt = 0, cle = 0;
      MAD: for (int x = 0; x < nn; x++) {
#pragma HLS loop_tripcount max=20
        MADI: for (int y = 0; y < nn; y++) {
#pragma HLS loop_tripcount max=20
            ap_uint<56> dx = DD[x], dy = DD[y];
            ap_uint<6> l0 = y == 0 ? (ap_uint<6>)0 : clt, e0 = y == 0 ? (ap_uint<6>)0 : cle;
            l0 += dy < dx; e0 += dy <= dx;
            clt = l0; cle = e0;
            if (y == nn - 1) {
              if (l0 <= k1 && k1 < e0) m1 = dx;
              if (l0 <= k2 && k2 < e0) m2 = dx;
            }
          }
        }
        ap_uint<31> msum = 0;
      MQ: for (int t = 0; t < 2; t++) msum += isqrt(((sq_t)(t == 0 ? m1 : m2)) << 2);   // 2 x median d, 2^-20
        i64 thr = imax(K_INLIER20, (i64)(((ap_uint<50>)msum * (ap_uint<50>)K_MAD3_16) >> 17));
        if (thr > (1LL << 29)) thr = 1LL << 29;                                     // thr^2 > every d^2 anyway
        ap_uint<60> thr2 = (ap_uint<60>)((ap_uint<30>)thr) * (ap_uint<30>)thr;     // 2^-40
        ap_uint<30> sw = 0; ap_int<58> swd[3] = {0, 0, 0};
      SUM: for (int x = 0; x < nn; x++) {
#pragma HLS loop_tripcount max=20
          Sample s = SAMP[base + ((pa.head + x) & (MK_KSAMP - 1))];
          if (((ap_uint<60>)DD[x] << 2) <= thr2) {
            sw += s.w;
            for (int ax = 0; ax < 3; ax++) {
              ap_int<28> dl = 2 * (ap_int<28>)s.p[ax] - med[ax];
              swd[ax] += (ap_int<53>)((ap_int<25>)s.w * dl);                         // 2^-31
            }
          }
        }
      DV: for (int t = 0; t < 4; t++) {                                             // 1/sum w, then 3 offsets
          ap_int<63> nsig = (t == 0) ? (ap_int<63>)((i64)1 << 60) : (ap_int<63>)(swd[t == 1 ? 0 : t == 2 ? 1 : 2] * 32);
          bool neg = nsig < 0;
          ap_uint<62> mag = neg ? (ap_uint<62>)(-nsig) : (ap_uint<62>)nsig;
          ap_uint<62> q; bool exact;
          sdu_div(mag, (ap_uint<51>)sw, q, exact);
          if (t == 0) { pa.sf2 = (q >> 48) ? (sf2_t)0xFFFFFFFFFFFFULL : (sf2_t)q; }
          else {
            ap_int<64> qs = (ap_int<64>)q;
            ap_int<64> off = neg ? (ap_int<64>)(exact ? (ap_int<64>)(-qs) : (ap_int<64>)(-qs - 1)) : qs;   // floor, 2^-24
            pa.p[t - 1] = (pos_t)(((ap_int<64>)med[t - 1] * 32 + off + 32) >> 6);
          }
        }
        pa.fval = true;
        S_POS[a] = pa;
      }
    }
    // ---- A_ODO: odometer stamps of this frame
    if (act & A_ODO) { SlotMeta ma = S_META[a]; ma.op = F.op; ma.ot = F.ot; S_META[a] = ma; }

    if (act & A_SETL) { if ((f >> 8) & 1) { SlotEx x = S_EX[a]; x.L = (int)q1; x.lval = true; S_EX[a] = x; } }
    if (act & A_RESETL) { SlotEx x = S_EX[a]; x.lval = false; S_EX[a] = x; }
    if (act & A_FREE) { SlotMeta ma = S_META[a]; ma.type = 0; S_META[a] = ma; }

    // ---- A_PREP: ASSOC preprocessing, admission and same-frame dedupe
    if (act & A_PREP) {
    LO: for (int x = 0; x < n; x++) {
#pragma HLS loop_tripcount max=32
        int ob = MK_OBS_BASE + MK_OBS_W * x;
        u32 o0 = cmd[ob];
        int c = o0 & 0x7F; bool rk = (o0 >> 9) & 1, ek = (o0 >> 10) & 1;
        for (int ax = 0; ax < 3; ax++) {
          i64 v = (int)cmd[ob + 2 + ax];
          if (v > (60LL << 18) || v < -(60LL << 18)) status |= ERR_RANGE;
        }
        if (c >= MK_CLASSES) status |= ERR_SIZE;
        i64 r = rk ? (i64)(int)cmd[ob + 5] : K_R_DEFAULT20;                         // _obs_sd: 0.02 + 0.01 r^2
        ap_uint<64> r2 = (ap_uint<64>)((ap_int<26>)r * (ap_int<26>)r) >> 16;        // 2^-24 m^2 (PS clamps r to 16 m)
        i64 so = K_SIG0_24 + (i64)((r2 * (ap_uint<64>)K_SIGK_32) >> 32);
        if (so > (2LL << 24)) so = 2LL << 24;                                       // sd <= 2 m (gate capped by C)
        ap_uint<62> wq; bool ex_;
        sdu_div((ap_uint<62>)1 << 60, (ap_uint<51>)((ap_uint<26>)so * (ap_uint<26>)so), wq, ex_);
        res[MK_RES_OBS + MK_RES_OBS_W * x + 1] = (u32)so;
        res[MK_RES_OBS + MK_RES_OBS_W * x + 2] = (u32)wq;
        int cb = MK_CLS_BASE + MK_CLS_W * (c & (MK_CLASSES - 1));
        u32 mw = cmd[cb + 1], mh = cmd[cb + 2], cf5 = cmd[cb + 5];
        int reason = 0;
        if ((i64)(int)cmd[ob + 4] < (i64)F.ground - K_BELOW_TOL18) reason = 1;
        else if (ek && (cf5 & 1) && ((i64)cmd[ob + 6] > (i64)mw || (i64)cmd[ob + 7] > (i64)mh)) reason = 2;
        OBSL[x] = (ap_uint<18>)reason | ((ap_uint<18>)127 << 11);
      }
      // dedupe order among admitted: (truncated, -score, i)
      int na = 0;
    RK: for (int x = 0; x < n; x++) {
#pragma HLS loop_tripcount max=32
        u32 ox0 = cmd[MK_OBS_BASE + MK_OBS_W * x], oxs = cmd[MK_OBS_BASE + MK_OBS_W * x + 1];
        ap_uint<18> lx = OBSL[x];
        int r = 0;
      RKI: for (int y = 0; y < n; y++) {
#pragma HLS loop_tripcount max=32
          u32 oy0 = cmd[MK_OBS_BASE + MK_OBS_W * y], oys = cmd[MK_OBS_BASE + MK_OBS_W * y + 1];
          if (OBSL[y](1, 0) != 0 || lx(1, 0) != 0) continue;
          int ty = (oy0 >> 8) & 1, tx = (ox0 >> 8) & 1;
          bool lt = (ty < tx) || (ty == tx && (oys > oxs || (oys == oxs && y < x)));
          r += lt;
        }
        lx(10, 6) = r; OBSL[x] = lx;
        if (lx(1, 0) == 0) { ORD[r] = x; na++; }
      }
    DD_: for (int rr = 0; rr < na; rr++) {
#pragma HLS loop_tripcount max=32
        int x = ORD[rr]; bool conflict = false;
        int obx = MK_OBS_BASE + MK_OBS_W * x;
        int cx = cmd[obx] & 0x7F;
        pos_t px0 = (int)cmd[obx + 2], px1 = (int)cmd[obx + 3], px2 = (int)cmd[obx + 4];
        s21 i0 = (int)cmd[obx + 8], i1 = (int)cmd[obx + 9], i2 = (int)cmd[obx + 10], i3 = (int)cmd[obx + 11];
      DDI: for (int y = 0; y < n; y++) {
#pragma HLS loop_tripcount max=32
          int oby = MK_OBS_BASE + MK_OBS_W * y;
          if (!OBSL[y][4] || (int)(cmd[oby] & 0x7F) != cx) continue;
          ap_int<27> dx = (pos_t)(int)cmd[oby + 2] - px0, dy = (pos_t)(int)cmd[oby + 3] - px1, dz = (pos_t)(int)cmd[oby + 4] - px2;
          ap_uint<56> d2 = (ap_uint<56>)(dx * dx) + (ap_uint<56>)(dy * dy) + (ap_uint<56>)(dz * dz);
          s21 k0 = (int)cmd[oby + 8], k1 = (int)cmd[oby + 9], k2 = (int)cmd[oby + 10], k3 = (int)cmd[oby + 11];
          s21 iw = max21(0, min21(k2, i2) - max21(k0, i0));
          s21 ih = max21(0, min21(k3, i3) - max21(k1, i1));
          ap_int<42> inter = iw * ih;
          ap_int<42> aa = max21(0, k2 - k0) * max21(0, k3 - k1);
          ap_int<42> ab = max21(0, i2 - i0) * max21(0, i3 - i1);
          ap_int<44> uni = (ap_int<44>)aa + ab - inter;
          if (d2 < (ap_uint<56>)K_DEDUPE2_36 || (uni > 0 && (ap_int<48>)K_IOU_DEN * inter > (ap_int<48>)K_IOU_NUM * uni)) conflict = true;
        }
        ap_uint<18> lx = OBSL[x];
        if (conflict) lx[2] = 1; else lx[4] = 1;
        OBSL[x] = lx;
      }
    EL: for (int x = 0; x < n; x++) {                                              // eligibility
#pragma HLS loop_tripcount max=32
        int ob = MK_OBS_BASE + MK_OBS_W * x;
        bool rk = (cmd[ob] >> 9) & 1; i64 rr = (int)cmd[ob + 5];
        ap_uint<18> lx = OBSL[x];
        bool beyond = rk && rr > K_OBS_MAX_R20;
        lx[3] = lx(1, 0) == 0 && !lx[2] && !beyond;
        OBSL[x] = lx;
      }
    }

    // ---- A_SCAN: the slot scan engine (one per call site of the old model: ASSOC pairs, QUERY, merge scan,
    // the "same-class box in gate" pass of EXIST). Outer loop over references, inner loop over slots.
    if (act & A_SCAN) {
      int smode = (opc == OP_ASSOC_INT) ? SM_ASSOC : (opc == OP_QUERY) ? SM_QUERY : (opc == OP_MERGE_SCAN) ? SM_MSCAN : SM_BLOCK;
      int kind = f & 3, mask = (f >> 2) & 3, tag = (f >> 8) & 255;
      int nout = (smode == SM_QUERY) ? 1 : (smode == SM_MSCAN) ? mu : n;
      int before = npairs;
      if (smode == SM_BLOCK) {
      SLC: for (int j = 0; j < MK_SLOTS; j++) SLOTL[j] = 0;                          // blocked, below, free_at_vote
      }
    SO: for (int o = 0; o < nout; o++) {
#pragma HLS loop_tripcount max=64
        // reference: an observation (ASSOC, BLOCK, QUERY kind 0) or a slot (QUERY kind 1/2, merge scan)
        bool refslot = (smode == SM_MSCAN) || (smode == SM_QUERY && kind != 0);
        int rs = (smode == SM_MSCAN) ? o : a;
        int ro = (smode == SM_QUERY) ? i : o;
        bool useobs = smode == SM_ASSOC || smode == SM_BLOCK || (smode == SM_QUERY && kind != 2);
        pos_t ref[3]; cls_t rcls = 0; so2_t so2 = 0;
        int ob = MK_OBS_BASE + MK_OBS_W * ro;
        u32 oc = cmd[ob];
        sd_t so = res[MK_RES_OBS + MK_RES_OBS_W * ro + 1];
        SlotPos rp = S_POS[rs]; SlotMeta rm = S_META[rs];
      RF: for (int ax = 0; ax < 3; ax++) ref[ax] = refslot ? rp.p[ax] : (pos_t)(int)cmd[ob + 2 + ax];
        rcls = useobs ? (cls_t)(oc & 0x7F) : rm.cls;
        so2 = (so2_t)so * so;
        if (smode == SM_MSCAN && rm.type != 1) continue;
        s34 slack = (int)cmd[MK_CLS_BASE + MK_CLS_W * (rcls & (MK_CLASSES - 1))];
        int ex = refslot ? rs : -1;
        int msk = (smode == SM_QUERY) ? mask : (smode == SM_ASSOC) ? 3 : 1;
        int q = (smode == SM_ASSOC) ? o : (smode == SM_BLOCK) ? o : (smode == SM_QUERY && kind == 0) ? i : rs;
        int jstart = (smode == SM_MSCAN) ? o + 1 : 0;
        s34 C = K_ASSOC_MAX_M20 + slack, MRG = K_MERGE20 + slack, EXC = K_EXCL20 + slack;
      SJ: for (int j = jstart; j < mu; j++) {
#pragma HLS loop_tripcount max=64
          SlotMeta sm = S_META[j];
          int ty = sm.type;
          if (j == ex || sm.cls != rcls || !((ty == 1 && (msk & 1)) || (ty == 2 && (msk & 2)))) continue;
          SlotPos sp = S_POS[j];
          ap_int<27> dx = (ap_int<27>)ref[0] - sp.p[0], dy = (ap_int<27>)ref[1] - sp.p[1], dz = (ap_int<27>)ref[2] - sp.p[2];
          ap_uint<56> d2 = (ap_uint<56>)(dx * dx) + (ap_uint<56>)(dy * dy) + (ap_uint<56>)(dz * dz);   // 2^-36
          s34 d = isqrt(((sq_t)d2) << 4);                                                              // 2^-20
          int fl = 0;
          if (smode == SM_MSCAN) {
            if (d > MRG) continue;
            fl = PF_MERGE;
          } else {
            odo_t dp = F.op - sm.op, dt = F.ot - sm.ot;                            // odometer is monotone
            s34 raw = (s34)(ap_uint<28>)(((ap_uint<52>)dp * (ap_uint<20>)K_DRIFT_M24) >> 26) + (s34)(ap_uint<28>)(((ap_uint<52>)dt * (ap_uint<20>)K_DRIFT_RAD24) >> 25);
            s34 rad = min34(K_ASSOC_MAX_M20, K_ASSOC_M20 + raw);
            s34 drift = rad - K_ASSOC_M20;
            if (d <= rad) fl |= PF_RADIUS;
            if (d <= K_ASSOC_M20) fl |= PF_NEAR;
            if (d <= MRG) fl |= PF_MERGE;
            if (useobs) {
              ap_uint<52> ssum = (ap_uint<52>)so2 + (sp.fval ? (ap_uint<52>)sp.sf2 : (ap_uint<52>)so2);
              if (ssum > (ap_uint<52>)0xFFFFFFFFFFFFULL) ssum = 0xFFFFFFFFFFFFULL;
              ap_uint<52> Q = (((ap_uint<32>)(ssum >> 16)) * (ap_uint<27>)K_CHI2) >> 8;   // chi^2 (so^2+sf^2), 2^-48
              s34 B = slack + drift;
              s34 e = min34(d - B, (s34)(1 << 22));
              ap_int<24> e_ = e;
              bool gate = d <= C && (e <= K_ASSOC_M20 || (((ap_uint<54>)(ap_uint<46>)(e_ * e_)) << 8) <= Q);
              if (gate) fl |= PF_GATE;
              s34 e2 = min34(d - slack, (s34)(1 << 22));
              ap_int<24> e2_ = e2;
              bool nr = e2 <= K_MOVE_FAR20 || (((ap_uint<54>)(ap_uint<46>)(e2_ * e2_)) << 8) <= Q;
              if (sp.ns > 0 && !nr) fl |= PF_FAR;
              if (d <= EXC || gate) fl |= PF_EXCL;
            }
            if (ty == 2) fl |= PF_CAND;
          }
          if (smode == SM_BLOCK) {
            if (fl & PF_GATE) SLOTL[j] = SLOTL[j] | 2;
          } else if (npairs >= MK_PAIRS) status |= ERR_PAIRS;
          else {
            res[MK_RES_PAIRS + 2 * npairs] = (u32)(q & 127) | ((u32)(j & 127) << 7) | ((u32)(tag & 255) << 14)
                                             | ((u32)(fl & 1023) << 22);
            res[MK_RES_PAIRS + 2 * npairs + 1] = (u32)d;
            npairs++;
          }
        }
      }
      if (smode == SM_MSCAN && (f & 1) && npairs > before) { status |= 1u << 8; break; }
    }

    // ---- A_EXHIT / A_EXROLL: existence step of slot a (hit) or of every track (frame)
    if (act & (A_EXHIT | A_EXROLL)) {
      bool hitmode = (act & A_EXHIT) != 0;
      int j0 = hitmode ? a : 0, j1 = hitmode ? a + 1 : mu;
    EX: for (int j = j0; j < j1; j++) {
#pragma HLS loop_tripcount max=64
        SlotMeta m = S_META[j];
        if (!hitmode && m.type != 1) continue;
        u32 g = (cmd[MK_GEO_BASE + (j >> 3)] >> (4 * (j & 7))) & 15;
        int code = g & 3; bool prot = (g >> 2) & 1, sinv = hitmode ? false : (bool)((g >> 3) & 1);
        SlotEx x = S_EX[j];
        // window roll
        bool below = false, fa = false;
        if (!(x.wval && F.W <= x.win)) {
          if (x.wval && !x.hit && (x.fre || x.mval)) {
            if (!x.lval) { x.L = sinv ? K_INVALID24 : K_EX_INIT24; x.lval = true; }
            s30 base = x.fre ? (s30)K_EX_FREE24 : (s30)0, delta = base;
            if (x.mval) {
              int idx = x.mpd >> 14; ap_uint<14> fr = x.mpd & 16383;                       // 0.5 ln(1 - pd), 1024 points
              s30 la = LN_TAB[idx], lb = LN_TAB[idx + 1];
              s30 mv = la + (s30)(((ap_int<20>)(lb - la) * (ap_int<16>)fr) >> 14);
              delta += mv;
              bool rel = x.rval && x.rh >= K_HMIN[x.rc] && x.mpd >= K_DECISIVE24 && x.lhval
                         && (ap_int<33>)x.win - (ap_int<33>)x.lh <= K_RECENT_WIN;
              if (rel) delta = base + min30(mv, K_REL_VOTE24);
            }
            RING[j * MK_RING + x.rhd] = RingE{x.win, false};
            x.rhd++; if (x.rn < MK_RING) x.rn++;
            s30 v = (s30)x.L + delta;
            if (!x.fre && x.mpd < K_DECISIVE24) v = max30(v, min30((s30)x.L, K_MISS_FLOOR24));
            x.L = (lo_t)min30(K_EX_MAX24, max30(K_EX_MIN24, v));
            if (x.L <= K_INVALID24) { below = true; fa = x.fre; }
          }
          x.win = F.W; x.wval = true; x.hit = false; x.fre = false; x.mval = false;
        }
        if (hitmode) {
          if (!x.lval) { x.L = K_EX_INIT24; x.lval = true; }
          else if (!x.hit) x.L = (lo_t)min30(K_EX_MAX24, (s30)x.L + K_EX_HIT24);
          if (!x.hit) {                                                             // reliable-removal record
            RING[j * MK_RING + x.rhd] = RingE{x.win, true};
            x.rhd++; if (x.rn < MK_RING) x.rn++;
            ap_uint<5> cnt = 0, hits = 0;
          RC: for (int k2 = 0; k2 < MK_RING; k2++) {
              if (k2 < x.rn) {
                RingE e = RING[j * MK_RING + ((x.rhd - 1 - k2) & (MK_RING - 1))];
                if ((ap_int<33>)e.w > (ap_int<33>)x.win - K_SPAN) { cnt++; hits += e.h; }
              }
            }
            x.rval = cnt >= K_MIN_WIN; x.rh = hits; x.rc = cnt; x.lh = x.win; x.lhval = true;
          }
          x.hit = true;
        } else {
          SLOTL[j] = (SLOTL[j] & 3) | ((ap_uint<4>)below << 2) | ((ap_uint<4>)fa << 3);
          bool blocked = SLOTL[j][1];
          if (!(prot || code < 2 || !F.pose || blocked)) {
            if (code == 2) x.fre = true;
            if (F.det && !F.skip) {
              SlotPos sp = S_POS[j];
              ap_int<56> r = 0;
            RD: for (int ax = 0; ax < 3; ax++) r += (ap_int<54>)((ap_int<27>)sp.p[ax] - F.T[ax]) * F.R2[ax];   // 2^-42
              s34 rr = (s34)(r >> 22);                                              // 2^-20
              if (rr > 0 && rr <= K_MISS_MAXR20) {
                ap_uint<25> cf = cmd[MK_CLS_BASE + MK_CLS_W * (m.cls & (MK_CLASSES - 1)) + 3];
                ap_int<26> c;                                                              // detection curve
                if (rr <= K_CX0_20) c = K_CY0_24;
                else if (rr >= K_CX2_20) c = K_CY2_24;
                else if (rr <= K_CX1_20) c = K_CY0_24 + (((ap_int<26>)K_CS0_24 * (ap_int<21>)(rr - K_CX0_20)) >> 20);
                else c = K_CY1_24 + (((ap_int<26>)K_CS1_24 * (ap_int<21>)(rr - K_CX1_20)) >> 20);
                ap_int<27> pd = (ap_int<27>)(((ap_int<52>)c * cf) >> 24);
                if (pd > K_PD_CAP24) pd = K_PD_CAP24;
                if (pd > 0) { if (!(x.mval && (ap_int<27>)x.mpd < pd)) x.mpd = pd; x.mval = true; }
              }
            }
          }
        }
        S_EX[j] = x;
      }
    }
    // ---- A_GREEDY: one-to-one over gate pairs to tracks of eligible observations (smallest d, record order)
    if (act & A_GREEDY) {
    GR: for (int round = 0; round < MK_OBS; round++) {
        int best = -1; u32 bd = 0;
      GRI: for (int k2 = 0; k2 < npairs; k2++) {
#pragma HLS loop_tripcount max=256
          u32 w0 = res[MK_RES_PAIRS + 2 * k2], d = res[MK_RES_PAIRS + 2 * k2 + 1];
          int q = w0 & 127, j = (w0 >> 7) & 127, fl = (w0 >> 22) & 1023;
          ap_uint<18> lq = OBSL[q & (MK_OBS - 1)];
          if (!(fl & PF_GATE) || (fl & PF_CAND) || !lq[3] || lq[5] || SLOTL[j & (MK_SLOTS - 1)][0]) continue;
          if (best < 0 || d < bd) { best = k2; bd = d; }
        }
        if (best < 0) break;
        u32 w0 = res[MK_RES_PAIRS + 2 * best];
        int q = w0 & 127, j = (w0 >> 7) & 127;
        ap_uint<18> lq = OBSL[q]; lq[5] = 1; lq(17, 11) = j; OBSL[q] = lq;
        SLOTL[j] = SLOTL[j] | 1;
        res[MK_RES_PAIRS + 2 * best] = w0 | ((u32)PF_ASSIGN << 22);
      }
    OR: for (int x = 0; x < n; x++) {
#pragma HLS loop_tripcount max=32
        int ob = MK_OBS_BASE + MK_OBS_W * x;
        u32 o0 = cmd[ob], sc = cmd[ob + 1]; i64 rr = (int)cmd[ob + 5];
        bool trunc = (o0 >> 8) & 1, rk = (o0 >> 9) & 1;
        int cb = MK_CLS_BASE + MK_CLS_W * ((o0 & 0x7F) & (MK_CLASSES - 1));
        u32 c4 = cmd[cb + 4], c5 = cmd[cb + 5];
        ap_uint<18> lx = OBSL[x];
        bool beyond = rk && rr > K_OBS_MAX_R20;
        bool too_far = rk && rr > K_CREATE_MAX_R20;
        bool may_create = !trunc || ((c5 >> 1) & 1);
        bool score_ok = (i64)sc >= (i64)c4;
        res[MK_RES_OBS + MK_RES_OBS_W * x] = (u32)lx(1, 0) | ((u32)lx[2] << 2) | ((u32)lx[3] << 3)
            | ((u32)may_create << 4) | ((u32)too_far << 5) | ((u32)beyond << 6) | ((u32)score_ok << 7)
            | ((u32)lx(10, 6) << 8) | ((u32)lx(17, 11) << 14);
      }
      N_OBS_LAST = n;
    }

    // ---- A_DUMP: slot words
    if (act & A_DUMP) {
    DU: for (int j = 0; j < mu; j++) {
#pragma HLS loop_tripcount max=64
        SlotPos sp = S_POS[j]; SlotEx x = S_EX[j]; SlotMeta m = S_META[j]; ap_uint<4> sl = SLOTL[j];
        int base = MK_RES_DUMP + ndump;
        res[base] = (u32)(int)sp.p[0]; res[base + 1] = (u32)(int)sp.p[1]; res[base + 2] = (u32)(int)sp.p[2];
        res[base + 3] = x.lval ? (u32)(int)x.L : 0;
        res[base + 4] = (u32)x.lval | ((u32)sl[2] << 1) | ((u32)sl[3] << 2) | ((u32)(sp.ns & 63) << 3)
                        | ((u32)sp.fval << 9) | ((u32)(m.type & 3) << 10);
        ndump += MK_DUMP_W;
      }
    }
  }
  res[0] = status; res[1] = npairs; res[2] = (cmdk == MK_CMD_EXEC && run) ? ndump : 0; res[3] = K_DIGEST_LO;
}

#ifndef __SYNTHESIS__
#include <string.h>
extern "C" {
int memk0_call(const u32 *cmd, u32 *res) { memk0_top(cmd, res); return (int)(res[0] & 0xFF); }
unsigned long long memk0_isqrt_pipe(unsigned long long x) { return (unsigned long long)isqrt_newton((sq_t)x); }
unsigned long long memk0_isqrt_sdu(unsigned long long x) { return (unsigned long long)isqrt_seq((sq_t)x); }
// whole persistent state, so several memories (unit tests) can share one library
int memk0_state_size(void) {
  return (int)(sizeof(S_POS) + sizeof(S_META) + sizeof(S_EX) + sizeof(SAMP) + sizeof(RING) + sizeof(N_OBS_LAST));
}
void memk0_state_save(unsigned char *b) {
  memcpy(b, (const void *)S_POS, sizeof(S_POS)); b += sizeof(S_POS); memcpy(b, (const void *)S_META, sizeof(S_META)); b += sizeof(S_META);
  memcpy(b, (const void *)S_EX, sizeof(S_EX)); b += sizeof(S_EX); memcpy(b, (const void *)SAMP, sizeof(SAMP)); b += sizeof(SAMP);
  memcpy(b, (const void *)RING, sizeof(RING)); b += sizeof(RING); memcpy(b, (const void *)&N_OBS_LAST, sizeof(N_OBS_LAST));
}
void memk0_state_load(const unsigned char *b) {
  memcpy((void *)S_POS, b, sizeof(S_POS)); b += sizeof(S_POS); memcpy((void *)S_META, b, sizeof(S_META)); b += sizeof(S_META);
  memcpy((void *)S_EX, b, sizeof(S_EX)); b += sizeof(S_EX); memcpy((void *)SAMP, b, sizeof(SAMP)); b += sizeof(SAMP);
  memcpy((void *)RING, b, sizeof(RING)); b += sizeof(RING); memcpy((void *)&N_OBS_LAST, b, sizeof(N_OBS_LAST));
}
}
#endif
