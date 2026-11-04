// 检测后处理核定点参考模型（可综合风格，先求对、不求快；HLS 优化另做）。
// 五级：扫描取最大类别并过阈值 → 候选的 DFL 解码 → 按分数计数排序 → 按类贪心 NMS → 白名单/下限后写出。
// 语义与浮点参考 detpost_float.py 的对应关系写在 README §5、§7。
#include "detpost.h"

namespace detpost {

namespace {

// 一个候选（硬件里是候选存储的一行，约 94 位）
struct Cand {
  int16_t x1, y1, x2, y2;  // Q4
  uint8_t cls;
  int8_t level;
  uint16_t anchor;
};

// DFL：16 个 int8 logit → 期望距离（格为单位，Q.8）。
// softmax 期望 = sum(i*exp(q_i))/sum(exp(q_i))；减去本边最大值后查 exp 表，整数累加，一次除法。
static int dfl_side_q8(const int8_t* q, const uint16_t* exp_lut) {
  int m = q[0];
  for (int i = 1; i < REG_MAX; i++)
    if (q[i] > m) m = q[i];
  uint32_t s = 0, w = 0;  // s <= 16*2^15 = 2^19，w <= 15*2^19 < 2^23
  for (int i = 0; i < REG_MAX; i++) {
    uint32_t e = exp_lut[m - q[i]];  // m - q[i] 在 0..255
    s += e;
    w += (uint32_t)i * e;
  }
  // 四舍五入的 (w << 8) / s；s >= 2^15（最大那一项本身就是 2^15），不会除零
  return (int)(((uint64_t)w * (1u << DIST_FRAC) + s / 2) / s);
}

// 距离小数位 → 坐标小数位（Q.DIST_FRAC → Q4），四舍五入（半数向上），负数按算术移位
static inline int16_t q8_to_q4(int32_t v) {
  constexpr int sh = DIST_FRAC - COORD_FRAC;
  return (int16_t)((v + (1 << (sh - 1))) >> sh);
}

}  // namespace

int detpost_ref(const int8_t* in, const Config& cfg, Box* out, Status* st) {
  static Cand cand[CAND_MAX];
  static uint16_t hist[LUT_N];
  static uint16_t order[CAND_MAX];
  static int16_t kb[MAX_DET_MAX][4];
  static int64_t karea[MAX_DET_MAX];
  static uint8_t kcls[MAX_DET_MAX];
  static int8_t klvl[MAX_DET_MAX];
  static uint16_t kanc[MAX_DET_MAX];

  const int nc = cfg.nc;
  const int ch = 4 * REG_MAX + nc;
  int thr = cfg.cls_thr_q;  // 当前有效阈值（溢出时抬高）
  int n = 0, n_seen = 0, overflow = 0;

  for (int i = 0; i < LUT_N; i++) hist[i] = 0;

  // ---- 第 1、2 级：扫描 + 候选 DFL 解码 ----
  int anchor = 0;
  for (int l = 0; l < N_LEVELS; l++) {
    const int gw = cfg.grid_w[l], gh = cfg.grid_h[l], sd = cfg.stride[l];
    const int8_t* base = in + cfg.offset[l];
    for (int y = 0; y < gh; y++) {
      for (int x = 0; x < gw; x++, anchor++) {
        const int8_t* p = base + (y * gw + x) * ch;
        const int8_t* pc = p + 4 * REG_MAX;
        int qmax = pc[0], cmax = 0;
        for (int c = 1; c < nc; c++)
          if (pc[c] > qmax) { qmax = pc[c]; cmax = c; }  // 严格大于：同分取下标小的
        if (qmax < cfg.cls_thr_q) continue;
        n_seen++;  // 过原始阈值的候选总数（溢出诊断用）
        if (qmax < thr) continue;
        int slot = n;
        if (n == CAND_MAX) {
          // 候选满（真实模型在 conf 0.25 下预计不会发生，README §6.3）。
          overflow = 1;
          int lo = 127;
          for (int i = 0; i < LUT_N; i++)
            if (hist[i]) { lo = i - 128; break; }
          if (hist[lo + 128] < n) {
            // 存储里不止一档：把最低一档整档丢掉、阈值抬到它上面一档，压缩（保持扫描序）。
            // 不变式：存储里 = 迄今见过的、分数 >= thr 的全部候选。
            thr = lo + 1;
            int k = 0;
            for (int i = 0; i < n; i++)
              if (cand[i].level >= thr) cand[k++] = cand[i];
            hist[lo + 128] = 0;
            n = k;
            slot = n;
            if (qmax < thr) continue;
          } else {
            // 全在同一档：同分留先来的（与稳定排序一致）；更高分的新候选顶掉最后进来的那个（它在最后一格，扫描序不乱）。
            if (qmax <= lo) continue;
            hist[lo + 128]--;
            slot = n - 1;
            n = n - 1;
          }
        }
        int d[4];
        for (int t = 0; t < 4; t++) d[t] = dfl_side_q8(p + t * REG_MAX, cfg.exp_lut);
        // 锚点中心 (x+0.5)*stride，Q8：(2x+1)*stride*128
        const int32_t cx = (2 * x + 1) * sd * (1 << (DIST_FRAC - 1));
        const int32_t cy = (2 * y + 1) * sd * (1 << (DIST_FRAC - 1));
        Cand& c = cand[slot];
        c.x1 = q8_to_q4(cx - d[0] * sd);
        c.y1 = q8_to_q4(cy - d[1] * sd);
        c.x2 = q8_to_q4(cx + d[2] * sd);
        c.y2 = q8_to_q4(cy + d[3] * sd);
        c.cls = (uint8_t)cmax;
        c.level = (int8_t)qmax;
        c.anchor = (uint16_t)anchor;
        hist[qmax + 128]++;
        n++;
      }
    }
  }

  // ---- 第 3 级：按分数档计数排序（降序；同档保持扫描序 = 稳定排序） ----
  {
    uint16_t pos[LUT_N];
    int acc = 0;
    for (int i = LUT_N - 1; i >= 0; i--) { pos[i] = (uint16_t)acc; acc += hist[i]; }
    for (int i = 0; i < n; i++) order[pos[cand[i].level + 128]++] = (uint16_t)i;
  }

  // ---- 第 4 级：按类贪心 NMS（只和已保留的同类框比；IoU > num/den 抑制） ----
  int nk = 0;
  for (int r = 0; r < n && nk < cfg.max_det; r++) {
    const Cand& c = cand[order[r]];
    const int64_t area = (int64_t)(c.x2 - c.x1) * (c.y2 - c.y1);
    bool keep = true;
    for (int j = 0; j < nk; j++) {
      if (kcls[j] != c.cls) continue;
      const int iw = (c.x2 < kb[j][2] ? c.x2 : kb[j][2]) - (c.x1 > kb[j][0] ? c.x1 : kb[j][0]);
      const int ih = (c.y2 < kb[j][3] ? c.y2 : kb[j][3]) - (c.y1 > kb[j][1] ? c.y1 : kb[j][1]);
      if (iw <= 0 || ih <= 0) continue;
      const int64_t inter = (int64_t)iw * ih;
      const int64_t uni = area + karea[j] - inter;
      if (inter * cfg.iou_den > (int64_t)cfg.iou_num * uni) { keep = false; break; }
    }
    if (!keep) continue;
    kb[nk][0] = c.x1; kb[nk][1] = c.y1; kb[nk][2] = c.x2; kb[nk][3] = c.y2;
    karea[nk] = area; kcls[nk] = c.cls; klvl[nk] = c.level; kanc[nk] = c.anchor;
    nk++;
  }

  // ---- 第 5 级：车上做法——NMS、截 max_det 之后再按类白名单与下限过滤，写出 ----
  int no = 0;
  for (int j = 0; j < nk; j++) {
    if (!cfg.cls_enable[kcls[j]] || klvl[j] < cfg.cls_min_q[kcls[j]]) continue;
    Box& b = out[no++];
    b.x1 = kb[j][0]; b.y1 = kb[j][1]; b.x2 = kb[j][2]; b.y2 = kb[j][3];
    b.cls = kcls[j]; b.level = klvl[j];
    b.score = cfg.sig_lut[klvl[j] + 128];
    b.anchor = kanc[j]; b.pad = 0;
  }

  if (st) {
    st->n_cand = n_seen; st->n_cand_kept = n; st->overflow = overflow;
    st->final_level = thr; st->n_nms = nk; st->n_out = no;
  }
  return no;
}

}  // namespace detpost
