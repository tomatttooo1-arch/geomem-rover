// obs_vec_main.cpp -- 观测生成 C 参考的向量驱动（宿主对照 + aarch64 计时，同一源）
//   obs_vec check VEC OUT_BIN OUT_CSV      跑全部 (帧,框)，按 obs_golden_py.bin 同格式写结果，另写诊断 csv
//   obs_vec hsvdump OUT                     2^24 种 RGB 的 HSV（3 B/色，R 最高位），用于对 cv2 标定 HSV 公式
//   obs_vec bench VEC GOLD REPS [CPU]       每框计时：先热身 1 遍，再 REPS 遍；同时把结果对黄金（位置/范围 <= 1 mm、颜色同、出观测同）
#include "../../hw/obs/obs_ref.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>
#ifdef __linux__
#include <sched.h>
#include <unistd.h>
#endif

using namespace obsref;

struct VFrame {
    uint32_t rec_frame, pose_kind;
    double K[9], T[16];
    std::vector<Box> boxes;
    std::vector<float> score; std::vector<int32_t> cls;
    std::vector<float> depth;
    std::vector<uint8_t> rgb;
};

#pragma pack(push, 1)
struct Rec { uint32_t slot, box; int32_t ok, color, truncated, pad; double v[13]; };
#pragma pack(pop)
static_assert(sizeof(Rec) == 128, "record size");

static std::vector<VFrame> load_vectors(const char* path, int& W, int& H, int& total) {
    FILE* fp = std::fopen(path, "rb");
    if (!fp) { std::perror(path); std::exit(2); }
    char magic[8]; uint32_t hdr[4];
    if (std::fread(magic, 1, 8, fp) != 8 || std::memcmp(magic, "OBSV0001", 8) || std::fread(hdr, 4, 4, fp) != 4) {
        std::fprintf(stderr, "bad vector file\n"); std::exit(2);
    }
    W = hdr[1]; H = hdr[2]; total = hdr[3];
    std::vector<VFrame> fr(hdr[0]);
    std::vector<uint16_t> mm((size_t)W * H);
    auto rd = [&](void* p, size_t n) { if (std::fread(p, 1, n, fp) != n) { std::fprintf(stderr, "short read\n"); std::exit(2); } };
    for (auto& f : fr) {
        rd(&f.rec_frame, 4); rd(&f.pose_kind, 4); rd(f.K, 72); rd(f.T, 128);
        uint32_t nb; rd(&nb, 4);
        f.boxes.resize(nb); f.score.resize(nb); f.cls.resize(nb);
        for (uint32_t i = 0; i < nb; ++i) { rd(&f.boxes[i], 32); rd(&f.score[i], 4); rd(&f.cls[i], 4); }
        rd(mm.data(), mm.size() * 2);
        f.depth.resize(mm.size()); depth_from_mm(mm.data(), (int)mm.size(), f.depth.data());
        f.rgb.resize(mm.size() * 3); rd(f.rgb.data(), f.rgb.size());
    }
    std::fclose(fp);
    return fr;
}

static Rec to_rec(uint32_t slot, uint32_t bi, const Result& r) {
    Rec o; std::memset(&o, 0, sizeof(o));
    o.slot = slot; o.box = bi; o.ok = r.status == OK; o.color = o.ok ? r.color : -1; o.truncated = o.ok ? r.truncated : 0;
    if (o.ok) {
        double* v = o.v;
        for (int a = 0; a < 3; ++a) { v[a] = r.position[a]; v[3 + a] = r.lower[a]; v[6 + a] = r.upper[a]; }
        v[9] = r.valid_fraction; v[10] = r.extent_m[0]; v[11] = r.extent_m[1]; v[12] = r.range_m;
    }
    return o;
}

static Frame view(const VFrame& f, int W, int H) {
    Frame v; v.width = W; v.height = H; v.depth_m = f.depth.data(); v.rgb = f.rgb.data();
    std::memcpy(v.K, f.K, sizeof v.K); std::memcpy(v.T, f.T, sizeof v.T);
    return v;
}

static double now_s() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "usage: see header\n"); return 2; }
    std::string mode = argv[1];
    if (mode == "hsvdump") {
        FILE* fp = std::fopen(argv[2], "wb");
        std::vector<uint8_t> buf(256 * 256 * 3);
        for (int r = 0; r < 256; ++r) {
            size_t k = 0;
            for (int g = 0; g < 256; ++g) for (int b = 0; b < 256; ++b) {
                uint8_t c[3] = {(uint8_t)r, (uint8_t)g, (uint8_t)b}; rgb2hsv_u8(c, &buf[k]); k += 3;
            }
            std::fwrite(buf.data(), 1, buf.size(), fp);
        }
        std::fclose(fp); return 0;
    }
    int W, H, total;
    std::vector<VFrame> fr = load_vectors(argv[2], W, H, total);
    Config cfg; Workspace ws;
    if (mode == "check") {
        FILE* ob = std::fopen(argv[3], "wb"); FILE* oc = std::fopen(argv[4], "w");
        std::fwrite("OBSG0001", 1, 8, ob); uint32_t n = total; std::fwrite(&n, 4, 1, ob);
        std::fprintf(oc, "slot,rec_frame,box,status,inner_x1,inner_y1,inner_x2,inner_y2,n_valid,n_mask,valid_fraction,h_med,s_med,v_med,color\n");
        for (size_t s = 0; s < fr.size(); ++s) {
            Frame v = view(fr[s], W, H);
            for (size_t i = 0; i < fr[s].boxes.size(); ++i) {
                Result r; process_box(v, fr[s].boxes[i], r, cfg, ws);
                Rec o = to_rec((uint32_t)s, (uint32_t)i, r); std::fwrite(&o, sizeof o, 1, ob);
                std::fprintf(oc, "%zu,%u,%zu,%d,%d,%d,%d,%d,%d,%d,%.6f,%.1f,%.1f,%.1f,%s\n", s, fr[s].rec_frame, i, r.status,
                             r.inner[0], r.inner[1], r.inner[2], r.inner[3], r.n_valid, r.n_mask, r.valid_fraction,
                             r.hsv_median[0], r.hsv_median[1], r.hsv_median[2], r.status == OK ? color_name(r.color) : "-");
            }
        }
        std::fclose(ob); std::fclose(oc); return 0;
    }
    if (mode == "bench") {
        if (argc < 5) return 2;
        int reps = std::atoi(argv[4]);
        int cpu = argc > 5 ? std::atoi(argv[5]) : -1;
        int fifo = 0;
#ifdef __linux__
        if (cpu >= 0) { cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(cpu, &cs); sched_setaffinity(0, sizeof cs, &cs); }
        sched_param sp; sp.sched_priority = 50; fifo = sched_setscheduler(0, SCHED_FIFO, &sp) == 0;
#endif
        // 黄金
        FILE* fg = std::fopen(argv[3], "rb"); char mg[8]; uint32_t ng = 0;
        if (!fg || std::fread(mg, 1, 8, fg) != 8 || std::fread(&ng, 4, 1, fg) != 1 || (int)ng != total) { std::fprintf(stderr, "bad golden\n"); return 2; }
        std::vector<Rec> gold(ng); if (std::fread(gold.data(), sizeof(Rec), ng, fg) != ng) return 2; std::fclose(fg);
        // 对照（同时当热身）
        int k = 0, bad = 0; double maxd = 0;
        std::vector<Result> res(total);
        for (size_t s = 0; s < fr.size(); ++s) {
            Frame v = view(fr[s], W, H);
            for (size_t i = 0; i < fr[s].boxes.size(); ++i, ++k) {
                process_box(v, fr[s].boxes[i], res[k], cfg, ws);
                Rec o = to_rec((uint32_t)s, (uint32_t)i, res[k]); const Rec& g = gold[k];
                bool same = o.ok == g.ok && o.color == g.color && o.truncated == g.truncated;
                for (int a = 0; a < 9 && same; ++a) { double d = std::fabs(o.v[a] - g.v[a]); maxd = std::max(maxd, d); same = d <= 1e-3; }
                bad += !same;
            }
        }
        // 计时
        std::vector<std::vector<double>> tb(total);
        std::vector<double> pass_s;
        for (int rep = 0; rep < reps; ++rep) {
            double p0 = now_s(); k = 0;
            for (size_t s = 0; s < fr.size(); ++s) {
                Frame v = view(fr[s], W, H);
                for (size_t i = 0; i < fr[s].boxes.size(); ++i, ++k) {
                    Result r; double t0 = now_s(); process_box(v, fr[s].boxes[i], r, cfg, ws); tb[k].push_back(now_s() - t0);
                }
            }
            pass_s.push_back(now_s() - p0);
        }
        // 每框取各遍中位数，再对全体框求中位数/p99（最近秩），按是否出观测分组
        auto stats = [](std::vector<double> x, double& med, double& p99, double& mean) {
            std::sort(x.begin(), x.end()); med = x[x.size() / 2];
            p99 = x[std::min(x.size() - 1, (size_t)std::ceil(0.99 * x.size()) - 1)];
            mean = 0; for (double y : x) mean += y; mean /= x.size();
        };
        std::vector<double> all, okv, nov;
        for (int j = 0; j < total; ++j) {
            std::vector<double> t = tb[j]; std::sort(t.begin(), t.end()); double m = t[t.size() / 2] * 1e6;
            all.push_back(m); (res[j].status == OK ? okv : nov).push_back(m);
        }
        double a, b, c;
        std::printf("{\"item\":\"obs_ref\",\"boxes\":%d,\"frames\":%zu,\"reps\":%d,\"cpu\":%d,\"fifo\":%d,\"check_bad\":%d,\"check_max_abs_m\":%.3g", total, fr.size(), reps, cpu, fifo, bad, maxd);
        stats(all, a, b, c); std::printf(",\"us_per_box_med\":%.2f,\"us_per_box_p99\":%.2f,\"us_per_box_mean\":%.2f", a, b, c);
        if (!okv.empty()) { stats(okv, a, b, c); std::printf(",\"obs_n\":%zu,\"obs_us_med\":%.2f,\"obs_us_p99\":%.2f", okv.size(), a, b); }
        if (!nov.empty()) { stats(nov, a, b, c); std::printf(",\"noobs_n\":%zu,\"noobs_us_med\":%.2f", nov.size(), a); }
        stats(pass_s, a, b, c); std::printf(",\"pass_ms_med\":%.3f}\n", a * 1e3);
        return bad ? 1 : 0;
    }
    return 2;
}
