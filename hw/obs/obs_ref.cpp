// obs_ref.cpp -- 见 obs_ref.h。
// 数值类型对齐（宿主 numpy 2.2，NEP 50 提升规则；Python 标量是"弱类型"、numpy 标量是"强类型"）：
//   深度        float32（u16/1000 在 float32 上做）
//   有效判定    float32 比较：p > 0.1f、p < 10.0f
//   直方图分箱  floorf(p / 0.05f)（float32 除法）
//   表面众数    double：mode = bin*0.05 + 0.025（numpy int64 * Python float -> float64）
//   表面掩码    double：|double(p) - mode| < 0.08（float32 数组减 float64 标量 -> float64）
//   反投影      double：ray = inv(K)·[u v 1]，cam = ray * double(z)，world = R·cam + t
//   分位数      numpy 'linear'：idx = q*(n-1)，a + (b-a)*g（g >= 0.5 时 b - (b-a)*(1-g)，与 numpy _lerp 同）
// 连通域：8 连通，标签按各连通域首个像素的光栅顺序编号（OpenCV 同面积并列时可能顺序不同，只影响并列最大的极少情形）。
// 编译须带 -ffp-contract=off（aarch64 GCC 默认合成 FMA）。
#include "obs_ref.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace obsref {

const char* color_name(int c) {
    static const char* n[] = {"black", "white", "gray", "red", "yellow", "green", "blue", "purple"};
    return (c >= 0 && c < 8) ? n[c] : "?";
}

void depth_from_mm(const uint16_t* mm, int n, float* out_m) {
    const float nanf_ = std::nanf("");
    for (int i = 0; i < n; ++i) {
        float v = (float)mm[i] / 1000.0f;
        out_m[i] = (v <= 0.0f) ? nanf_ : v;
    }
}

// OpenCV RGB2HSV_b（hsv_shift = 12，180 度色相），表与 imgproc/src/color_hsv.simd.hpp 同式
namespace {
struct HsvTables {
    int sdiv[256], hdiv[256];
    HsvTables() {
        sdiv[0] = hdiv[0] = 0;
        for (int i = 1; i < 256; ++i) {
            sdiv[i] = (int)std::lrint((255 << 12) / (1. * i));
            hdiv[i] = (int)std::lrint((180 << 12) / (6. * i));
        }
    }
};
const HsvTables& hsv_tables() { static HsvTables t; return t; }
}  // namespace

void rgb2hsv_u8(const uint8_t rgb[3], uint8_t hsv[3]) {
    const HsvTables& T = hsv_tables();
    const int shift = 12, half = 1 << (shift - 1);
    int r = rgb[0], g = rgb[1], b = rgb[2];
    int v = std::max(r, std::max(g, b));
    int vmin = std::min(r, std::min(g, b));
    int diff = v - vmin;
    int vr = (v == r) ? -1 : 0;
    int vg = (v == g) ? -1 : 0;
    int s = (diff * T.sdiv[v] + half) >> shift;
    int h = (vr & (g - b)) + (~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
    h = (h * T.hdiv[diff] + half) >> shift;
    h += (h < 0) ? 180 : 0;
    hsv[0] = (uint8_t)h; hsv[1] = (uint8_t)s; hsv[2] = (uint8_t)v;
}

namespace {

double quantile_sorted(const std::vector<double>& a, double q) {
    const size_t n = a.size();
    double idx = q * (double)(n - 1);
    double prev = std::floor(idx);
    size_t i0 = (size_t)prev, i1 = std::min(i0 + 1, n - 1);
    double g = idx - prev;
    double x0 = a[i0], x1 = a[i1], d = x1 - x0;
    return (g >= 0.5) ? (x1 - d * (1.0 - g)) : (x0 + d * g);
}

double median_sorted(const std::vector<double>& a) {
    const size_t n = a.size();
    return (n & 1) ? a[n / 2] : (a[n / 2 - 1] + a[n / 2]) / 2.0;
}

double median_u8(const std::vector<uint8_t>& a) {
    int hist[256] = {0};
    for (uint8_t x : a) hist[x]++;
    const size_t n = a.size();
    auto kth = [&](size_t k) { size_t c = 0; for (int i = 0; i < 256; ++i) { c += hist[i]; if (c > k) return i; } return 255; };
    return (n & 1) ? (double)kth(n / 2) : ((double)kth(n / 2 - 1) + (double)kth(n / 2)) / 2.0;
}

int32_t uf_find(std::vector<int32_t>& p, int32_t x) {
    while (p[x] != x) { p[x] = p[p[x]]; x = p[x]; }
    return x;
}

void inv3(const double K[9], double o[9]) {
    double a = K[0], b = K[1], c = K[2], d = K[3], e = K[4], f = K[5], g = K[6], h = K[7], i = K[8];
    double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    double det = a * A + b * B + c * C;
    o[0] = A / det; o[1] = -(b * i - c * h) / det; o[2] = (b * f - c * e) / det;
    o[3] = B / det; o[4] = (a * i - c * g) / det;  o[5] = -(a * f - c * d) / det;
    o[6] = C / det; o[7] = -(a * h - b * g) / det; o[8] = (a * e - b * d) / det;
}

int classify(double hue, double sat, double val) {
    if (val < 50) return BLACK;
    if (sat < 40 && val > 180) return WHITE;
    if (sat < 40) return GRAY;
    if (hue < 10 || hue > 170) return RED;
    if (hue < 35) return YELLOW;
    if (hue < 85) return GREEN;
    if (hue < 135) return BLUE;
    return PURPLE;
}

}  // namespace

int process_box(const Frame& f, const Box& bx, Result& r, const Config& cfg, Workspace& ws) {
    std::memset(&r, 0, sizeof(r));
    const int w = f.width, h = f.height;
    // 内缩 15%
    double dx = (bx.x2 - bx.x1) * 0.15, dy = (bx.y2 - bx.y1) * 0.15;
    int x1 = std::max(0, (int)std::ceil(bx.x1 + dx)), x2 = std::min(w, (int)std::floor(bx.x2 - dx));
    int y1 = std::max(0, (int)std::ceil(bx.y1 + dy)), y2 = std::min(h, (int)std::floor(bx.y2 - dy));
    r.inner[0] = x1; r.inner[1] = y1; r.inner[2] = x2; r.inner[3] = y2;
    if (x2 <= x1 || y2 <= y1) return r.status = EMPTY_INNER;
    const int pw = x2 - x1, ph = y2 - y1, np_ = pw * ph;

    // 有效深度与 5 cm 直方图
    ws.mask.assign(np_, 0);
    int n_valid = 0, maxbin = -1;
    for (int yy = 0; yy < ph; ++yy)
        for (int xx = 0; xx < pw; ++xx) {
            float p = f.depth_m[(size_t)(y1 + yy) * w + (x1 + xx)];
            if (std::isfinite(p) && p > 0.1f && p < 10.0f) {
                ws.mask[yy * pw + xx] = 1; ++n_valid;
                maxbin = std::max(maxbin, (int)std::floor(p / 0.05f));
            }
        }
    r.n_valid = n_valid;
    r.valid_fraction = (double)n_valid / (double)np_;
    if (n_valid < cfg.min_pixels || r.valid_fraction < cfg.min_valid_fraction) return r.status = LOW_VALID;
    ws.counts.assign(maxbin + 1, 0);
    for (int k = 0; k < np_; ++k)
        if (ws.mask[k]) {
            float p = f.depth_m[(size_t)(y1 + k / pw) * w + (x1 + k % pw)];
            ws.counts[(int)std::floor(p / 0.05f)]++;
        }
    const int nb = (int)ws.counts.size();
    int mode_bin = (int)(std::max_element(ws.counts.begin(), ws.counts.end()) - ws.counts.begin());
    if (cfg.identity_v2) {
        double need = cfg.near_surface_fraction * (double)n_valid;
        for (int b = 0; b < nb; ++b) {
            if (!ws.counts[b]) continue;
            int pair = ws.counts[b] + (b + 1 < nb ? ws.counts[b + 1] : 0);
            if ((double)pair >= need) {
                mode_bin = (b + 1 >= nb || ws.counts[b] >= ws.counts[b + 1]) ? b : b + 1;
                break;
            }
        }
    }
    const double mode = (double)mode_bin * 0.05 + 0.025;
    int n_fg = 0;
    for (int k = 0; k < np_; ++k)
        if (ws.mask[k]) {
            float p = f.depth_m[(size_t)(y1 + k / pw) * w + (x1 + k % pw)];
            ws.mask[k] = (std::fabs((double)p - mode) < 0.08) ? 1 : 0;
            n_fg += ws.mask[k];
        }
    if (n_fg == 0) return r.status = NO_COMPONENT;

    // 8 连通连通域（并查集，两遍）
    ws.label.assign(np_, -1);
    ws.parent.clear();
    for (int yy = 0; yy < ph; ++yy)
        for (int xx = 0; xx < pw; ++xx) {
            int k = yy * pw + xx;
            if (!ws.mask[k]) continue;
            int nbr[4], m = 0;
            if (xx > 0 && ws.mask[k - 1]) nbr[m++] = ws.label[k - 1];
            if (yy > 0) {
                int kk = k - pw;
                if (xx > 0 && ws.mask[kk - 1]) nbr[m++] = ws.label[kk - 1];
                if (ws.mask[kk]) nbr[m++] = ws.label[kk];
                if (xx + 1 < pw && ws.mask[kk + 1]) nbr[m++] = ws.label[kk + 1];
            }
            if (m == 0) { int id = (int)ws.parent.size(); ws.parent.push_back(id); ws.label[k] = id; continue; }
            int root = uf_find(ws.parent, nbr[0]);
            for (int j = 1; j < m; ++j) {
                int rj = uf_find(ws.parent, nbr[j]);
                if (rj != root) { if (rj < root) std::swap(rj, root); ws.parent[rj] = root; }
            }
            ws.label[k] = root;
        }
    // 最终编号：按首像素光栅顺序；数面积，取最先出现的最大者
    const int nprov = (int)ws.parent.size();
    std::vector<int32_t>& fin = ws.parent;  // 复用：先求根，再映射
    std::vector<int32_t> roots(nprov);
    for (int i = 0; i < nprov; ++i) roots[i] = uf_find(fin, i);
    std::vector<int32_t> final_of_root(nprov, -1);
    ws.area.clear();
    for (int k = 0; k < np_; ++k) {
        if (!ws.mask[k]) continue;
        int rt = roots[ws.label[k]];
        if (final_of_root[rt] < 0) { final_of_root[rt] = (int)ws.area.size(); ws.area.push_back(0); }
        ws.label[k] = final_of_root[rt];
        ws.area[ws.label[k]]++;
    }
    int best = (int)(std::max_element(ws.area.begin(), ws.area.end()) - ws.area.begin());
    r.n_mask = (int)ws.area[best];
    if (r.n_mask < cfg.min_pixels) return r.status = SMALL_COMPONENT;

    // 反投影、HSV
    double Ki[9]; inv3(f.K, Ki);
    const double* T = f.T;
    ws.xs.clear(); ws.ys.clear(); ws.zs.clear(); ws.hh.clear(); ws.ss.clear(); ws.vv.clear();
    for (int k = 0; k < np_; ++k) {
        if (!ws.mask[k] || ws.label[k] != best) continue;
        int u = x1 + k % pw, v = y1 + k / pw;
        size_t gi = (size_t)v * w + u;
        double z = (double)f.depth_m[gi];
        double U = (double)u, V = (double)v;
        double rx = U * Ki[0] + V * Ki[1] + 1.0 * Ki[2];
        double ry = U * Ki[3] + V * Ki[4] + 1.0 * Ki[5];
        double rz = U * Ki[6] + V * Ki[7] + 1.0 * Ki[8];
        double cx = rx * z, cy = ry * z, cz = rz * z;
        ws.xs.push_back(cx * T[0] + cy * T[1] + cz * T[2] + T[3]);
        ws.ys.push_back(cx * T[4] + cy * T[5] + cz * T[6] + T[7]);
        ws.zs.push_back(cx * T[8] + cy * T[9] + cz * T[10] + T[11]);
        uint8_t hsv[3]; rgb2hsv_u8(f.rgb + gi * 3, hsv);
        ws.hh.push_back(hsv[0]); ws.ss.push_back(hsv[1]); ws.vv.push_back(hsv[2]);
    }
    std::vector<double>* ax[3] = {&ws.xs, &ws.ys, &ws.zs};
    for (int a = 0; a < 3; ++a) {
        std::sort(ax[a]->begin(), ax[a]->end());
        r.lower[a] = quantile_sorted(*ax[a], 0.05) - 0.03;
        r.upper[a] = quantile_sorted(*ax[a], 0.95) + 0.03;
        r.position[a] = median_sorted(*ax[a]);
    }
    r.hsv_median[0] = median_u8(ws.hh); r.hsv_median[1] = median_u8(ws.ss); r.hsv_median[2] = median_u8(ws.vv);
    r.color = classify(r.hsv_median[0], r.hsv_median[1], r.hsv_median[2]);
    double cam_z = 0;
    for (int i = 0; i < 3; ++i) cam_z += (r.position[i] - T[i * 4 + 3]) * T[i * 4 + 2];
    r.range_m = cam_z;
    r.extent_m[0] = (bx.x2 - bx.x1) * cam_z / f.K[0];
    r.extent_m[1] = (bx.y2 - bx.y1) * cam_z / f.K[4];
    const double m = cfg.edge_margin_px;
    r.truncated = (bx.x1 <= m || bx.y1 <= m || bx.x2 >= w - 1 - m || bx.y2 >= h - 1 - m) ? 1 : 0;
    return r.status = OK;
}

int process_frame(const Frame& f, const Box* boxes, int n, Result* res, const Config& cfg, Workspace* ws) {
    Workspace local;
    Workspace& W = ws ? *ws : local;
    int ok = 0;
    for (int i = 0; i < n; ++i) ok += (process_box(f, boxes[i], res[i], cfg, W) == OK);
    return ok;
}

}  // namespace obsref
