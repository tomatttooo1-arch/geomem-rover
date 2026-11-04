// SLIM_ROI1 C-level equivalence harness (run artefact, not product code).
// Product source is #included (so the internal ratio helpers are reachable);
// the reference is the frozen pre-change fpga_m7_object_roi_batch (tag
// nat-slim-roi-pre-20261102), compiled separately under the name ref_roi_batch.
// Build: see run_ctest.sh. Exit 0 only if every part has 0 mismatches.
#include "roi_fx_ps.hpp"
#include ".memory/ref/object_evidence.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

extern "C" void ref_roi_batch(
    const float depth[FPGA_M7_OBJECT_MAX_PIXELS], std::uint32_t width, std::uint32_t height,
    std::uint32_t box_count, const std::uint32_t box_enabled[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t box_xyxy[4 * FPGA_M7_ROI_MAX_BOXES], const double near_z[FPGA_M7_ROI_MAX_BOXES],
    const double far_z[FPGA_M7_ROI_MAX_BOXES], const std::uint32_t object_id[FPGA_M7_ROI_MAX_BOXES],
    double depth_margin_m, std::uint32_t min_pixels, double min_valid_fraction, double free_fraction,
    std::uint32_t result[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES], std::uint32_t* status);

static std::vector<float> depth(FPGA_M7_OBJECT_MAX_PIXELS);

static std::vector<std::uint16_t> raw(FPGA_M7_OBJECT_MAX_PIXELS);
static float current_unit=.001f;
void fpga_m7_object_roi_batch(const float*,std::uint32_t w,std::uint32_t h,std::uint32_t n,
 const std::uint32_t en[32],const std::uint32_t xy[128],const double nz[32],const double fz[32],
 const std::uint32_t id[32],double margin,std::uint32_t minpix,double mvf,double ff,
 std::uint32_t out[224],std::uint32_t* status){
 roi_fx_ps::Input a={};a.w=w;a.h=h;a.n=n;a.minpix=minpix;a.margin=margin;a.mvf=mvf;a.ff=ff;
 std::memcpy(a.en,en,sizeof a.en);std::memcpy(a.xy,xy,sizeof a.xy);std::memcpy(a.id,id,sizeof a.id);
 std::memcpy(a.near_z,nz,sizeof a.near_z);std::memcpy(a.far_z,fz,sizeof a.far_z);
 auto c=roi_fx_ps::compile(a,current_unit);roi_fx_ps::model(raw.data(),a,c,out,status);
}
struct Call {
    std::uint32_t w, h, n, en[32], xyxy[128], oid[32], minpix;
    double nz[32], fz[32], margin, mvf, ff;
};

// Returns true if product and reference agree on status and all 224 result words
// (both start from the same sentinel, so untouched rows are compared too).
static bool same(const Call& c, std::uint32_t* ref_out = nullptr) {
    std::uint32_t ref[7 * 32], got[7 * 32], sr = 0xDEAD, sg = 0xBEEF;
    std::memset(ref, 0xAB, sizeof ref);
    std::memset(got, 0xAB, sizeof got);
    ref_roi_batch(depth.data(), c.w, c.h, c.n, c.en, c.xyxy, c.nz, c.fz, c.oid, c.margin, c.minpix,
                  c.mvf, c.ff, ref, &sr);
    fpga_m7_object_roi_batch(depth.data(), c.w, c.h, c.n, c.en, c.xyxy, c.nz, c.fz, c.oid, c.margin,
                            c.minpix, c.mvf, c.ff, got, &sg);
    if (ref_out) std::memcpy(ref_out, ref, sizeof ref);
    return sr == sg && std::memcmp(ref, got, sizeof ref) == 0;
}

int main(int argc, char** argv) {
    const int seed = argc > 1 ? std::atoi(argv[1]) : 1;
    const bool partA = seed==7;
    std::mt19937_64 rng(seed);
    auto U = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
    auto I = [&](long lo, long hi) { return (long)std::uniform_int_distribution<long>(lo, hi)(rng); };
    long total_bad = 0;

    // Reproduce only RNG consumption of original A for seed7. No ratio-A assertions.
    if(partA){for(int i=0;i<20;++i)(void)U(.7,1.0);
      for(long k=0;k<4000000;++k){if(k%3)(void)U(.5,1.0);
       const auto b=I(1,(1<<20)-1);(void)I(0,b);}}
    // ---- Part B: whole-core rows on random scenes (SLIM_ROI prototype generator,
    // float depth = float(code)*unit, code 0 -> NaN), 400 scenes per seed.
    // B2 re-runs each scene with min_valid_fraction / free_fraction set exactly
    // at (and one ulp around) a box's own ratio, so the verdict boundary is hit.
    {
        const float units[] = {0.001f, 0.0005f, 0.0001f, 0.00025f, 0.000125f};
        long scenes = 0, bad = 0, rows = 0, b2 = 0, b2bad = 0, nonzero = 0;
        for (int s = 0; s < 400; ++s) {
            Call c={};
            c.w = I(16, 320); c.h = I(16, 240);
            const float unit = units[I(0, 4)];current_unit=unit;
            const long cmax = std::min(65535L, (long)(12.0 / unit));
            const double base = U(0.3, 8.0);
            for (std::uint32_t i = 0; i < c.w * c.h; ++i) {
                long r = I(0, 99), code;
                if (r < 5) code = 0;
                else if (r < 70) code = std::min(cmax, std::max(1L, (long)std::lround((base + U(-0.3, 0.3)) / unit)));
                else code = I(1, cmax);
                raw[i]=code;depth[i] = code ? (float)code * unit : NAN;
            }
            c.n = I(1, 32);
            for (std::uint32_t b = 0; b < c.n; ++b) {
                c.en[b] = I(0, 4) > 0; c.oid[b] = I(0, 1000000);
                std::uint32_t x0 = I(0, c.w - 4), y0 = I(0, c.h - 4);
                c.xyxy[4*b] = x0; c.xyxy[4*b+1] = y0; c.xyxy[4*b+2] = I(x0 + 3, c.w - 1); c.xyxy[4*b+3] = I(y0 + 3, c.h - 1);
                double nzv = U(0.11, 9.0);
                if (I(0, 2) == 0) nzv = (double)((float)std::lround(base / unit) * unit) + 0.04;
                c.nz[b] = std::max(0.1000001, nzv); c.fz[b] = c.nz[b] + U(0.0, 1.0);
            }
            c.margin = I(0, 2) ? 0.04 : U(0.04, 0.2);
            c.minpix = I(9, 40);
            c.mvf = I(0, 3) ? U(0.7, 1.0) : 0.7; c.ff = I(0, 3) ? U(0.9, 1.0) : 0.9;
            std::uint32_t ref[7 * 32];
            const bool ok = same(c, ref);
            ++scenes; rows += c.n; bad += !ok;
            // status of the reference (0 expected for this generator)
            {
                std::uint32_t st = 1, tmp[7 * 32];
                ref_roi_batch(depth.data(), c.w, c.h, c.n, c.en, c.xyxy, c.nz, c.fz, c.oid, c.margin,
                              c.minpix, c.mvf, c.ff, tmp, &st);
                nonzero += st != 0;
            }
            // B2: thresholds placed on a box's own ratios.
            for (std::uint32_t b = 0; b < c.n; ++b) {
                const std::uint32_t* r = ref + 7 * b;
                if (!r[M7_RB_ENABLED] || r[M7_RB_VALID] == 0) continue;
                const double rv = (double)r[M7_RB_VALID] / r[M7_RB_PIXELS];
                const double rf = (double)r[M7_RB_BEHIND] / r[M7_RB_VALID];
                const double cand_v[3] = {rv, std::nextafter(rv, 0.0), std::nextafter(rv, 2.0)};
                const double cand_f[3] = {rf, std::nextafter(rf, 0.0), std::nextafter(rf, 2.0)};
                for (int i = 0; i < 3; ++i) {
                    Call d = c;
                    if (cand_v[i] >= 0.7 && cand_v[i] <= 1.0) d.mvf = cand_v[i];
                    if (cand_f[i] >= 0.9 && cand_f[i] <= 1.0) d.ff = cand_f[i];
                    if (d.mvf == c.mvf && d.ff == c.ff) continue;
                    ++b2; b2bad += !same(d);
                }
            }
        }
        std::printf("B seed=%d scenes=%ld rows=%ld scenes_with_mismatch=%ld ref_nonzero_status=%ld | "
                    "B2 boundary-threshold runs=%ld mismatches=%ld\n",
                    seed, scenes, rows, bad, nonzero, b2, b2bad);
        total_bad += bad + b2bad;
    }

    // Fixed one-at-a-time PS validation boundaries, replacing old Cartesian C.
    {
      current_unit=.001f;for(unsigned i=0;i<400;++i){raw[i]=i%7?1400+i:0;depth[i]=raw[i]?float(raw[i])*current_unit:NAN;}
      Call base={};base.w=base.h=20;base.n=2;base.en[0]=base.en[1]=1;base.oid[0]=1;base.oid[1]=2;
      base.minpix=9;base.margin=.04;base.mvf=.7;base.ff=.9;
      for(unsigned b=0;b<2;++b){base.xyxy[4*b]=1+b;base.xyxy[4*b+1]=1+b;base.xyxy[4*b+2]=15;base.xyxy[4*b+3]=15;base.nz[b]=1.5;base.fz[b]=1.6;}
      long checks=0,bad=0;auto check=[&](Call c){++checks;bad+=!same(c);};check(base);
      for(double v:{std::nan(""),double(INFINITY),double(-INFINITY),0.,-0.,DBL_MAX,std::numeric_limits<double>::denorm_min()}){
       Call c=base;c.margin=v;check(c);c=base;c.mvf=v;check(c);c=base;c.ff=v;check(c);c=base;c.nz[1]=v;check(c);c=base;c.fz[1]=v;check(c);}
      for(double t:{.04,.1,.7,.9,1.})for(double v:{std::nextafter(t,-INFINITY),t,std::nextafter(t,INFINITY)}){
       Call c=base;if(t==.04)c.margin=v;else if(t==.1)c.nz[1]=v;else if(t==.7)c.mvf=v;else c.ff=v;check(c);}
      {Call c=base;c.w=0;c.mvf=NAN;check(c);c=base;c.n=33;check(c);c=base;c.en[0]=2;check(c);c=base;c.minpix=8;check(c);c=base;c.en[1]=0;c.nz[1]=NAN;c.xyxy[5]=999;check(c);}
      std::printf("Fixed-validation checks=%ld mismatches=%ld\n",checks,bad);total_bad+=bad;
    }
    std::printf("TOTAL mismatches=%ld -> %s\n", total_bad, total_bad ? "FAIL" : "PASS");
    return total_bad ? 1 : 0;
}
