// GEO_FX0 (2026-11-03): fixed-point golden model of the redesigned geometry front end, and its
// comparison against the float32 reference, on the host (g++ -O2 -ffp-contract=off).
//
// usage: fx_golden PACK GOLDEN REFFLAGS TAG OUTDIR DLAT(0|1) [MAXFRAMES]
//
// Three arms per frame, all through the SAME back end (Dlat2 thinning exactly as the HLS convert
// stage, reference add_ray DDA walk, A1 merge 2 > 1 > 0, counts):
//   REF  front end = the reference float expressions of hw/m7/frame_depth.cpp, depth from the codes
//        by the A53 rule f32(code) * f32(unit), code 0 -> NaN. Instrument check I0: REF must equal
//        the product core (REFFLAGS = G1d Dlat2 flags when DLAT=1, GOLDEN dump flags when DLAT=0;
//        counts = GOLDEN dump counts) byte for byte.
//   FX   front end = the fixed-point design of DESIGN.md section 2 (bit exact definition below).
//   FXC  positive control: FX with the per-pixel ray coefficient P quantized to 13 instead of 22
//        fractional bits (the "P on the 18-bit DSP port" design point, error bound x 512).
// Per valid pixel the analysis computes E_ex (long double, exact up to ~2^-60 relative), the
// reference's pre-floor double value E_ref, FX's exact fixed-point E_fx, the a-priori bounds
// B_fx (design) and B_ref (float32 error analysis), and attributes every endpoint-voxel mismatch.
// Outputs: OUTDIR/TAG.csv (one row per frame), OUTDIR/TAG.{ref,fx,fxc}.flags (32768 B per frame),
// summary line on stderr. Exit 0 always unless inputs are bad (gates are judged by gates.py).
#include "frame_depth.hpp"

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <zlib.h>

namespace {

struct PF {
    uint32_t w, h, nx, ny, nz;
    float unit, ik[9], wc[12];
    double origin[3], voxel;
    const uint16_t* depth;
};

const uint8_t* map_file(const char* p, size_t* n) {
    int fd = open(p, O_RDONLY);
    if (fd < 0) return nullptr;
    struct stat s;
    fstat(fd, &s);
    *n = s.st_size;
    void* m = mmap(nullptr, *n, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    return m == MAP_FAILED ? nullptr : static_cast<const uint8_t*>(m);
}

std::vector<PF> load_pack(const uint8_t* b, size_t size) {  // tools/gpu_geom/make_pack.py format
    std::vector<PF> out;
    if (size < 24 || std::memcmp(b, "FPGAGPK01", 8) != 0) return out;
    uint32_t n;
    std::memcpy(&n, b + 12, 4);
    size_t off = 24;
    for (uint32_t i = 0; i < n; ++i) {
        PF f;
        uint32_t u[6];
        std::memcpy(u, b + off, 24);
        f.w = u[0]; f.h = u[1]; f.nx = u[2]; f.ny = u[3]; f.nz = u[4];
        std::memcpy(&f.unit, b + off + 24, 4);
        std::memcpy(f.ik, b + off + 28, 36);
        std::memcpy(f.wc, b + off + 64, 48);
        std::memcpy(f.origin, b + off + 112, 24);
        std::memcpy(&f.voxel, b + off + 136, 8);
        off += 144;
        f.depth = reinterpret_cast<const uint16_t*>(b + off);
        off += static_cast<size_t>(f.w) * f.h * 2;
        out.push_back(f);
    }
    if (off != size) out.clear();
    return out;
}

bool finite_float(float v) { return v >= -FLT_MAX && v <= FLT_MAX; }
bool finite_double(double v) { return v >= -DBL_MAX && v <= DBL_MAX; }
const int Q8 = FPGA_M7_FRAME_Q8;
const uint32_t kCap = FPGA_M7_FRAME_GRID_CAPACITY;

int32_t floor_q8(int32_t value) {
    return value >= 0 ? value / Q8 : -((-value + Q8 - 1) / Q8);
}

// ---------------- reference add_ray (hw/m7/frame_depth.cpp, verbatim + step counter) -------------
bool cell_in_grid(const int32_t c[3], const uint32_t d[3]) {
    return c[0] >= 0 && c[0] < (int32_t)d[0] && c[1] >= 0 && c[1] < (int32_t)d[1] &&
           c[2] >= 0 && c[2] < (int32_t)d[2];
}
uint32_t grid_offset(const int32_t c[3], uint32_t nx, uint32_t ny) {
    return ((uint32_t)c[2] * ny + (uint32_t)c[1]) * nx + (uint32_t)c[0];
}
unsigned long long g_steps = 0;
void add_ray(const int32_t origin[3], const int32_t end[3], const uint32_t dims[3], uint8_t* grid) {
    int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for (int a = 0; a < 3; ++a) {
        const int32_t from = origin[a], to = end[a];
        cell[a] = from / Q8;
        target[a] = floor_q8(to);
        direction[a] = to > from ? 1 : (to < from ? -1 : 0);
        delta[a] = to > from ? to - from : from - to;
        next[a] = direction[a] > 0 ? (cell[a] + 1) * Q8 - from : from - cell[a] * Q8;
    }
    for (uint32_t step = 0; step < 3 * FPGA_M7_FRAME_GRID_AXIS; ++step) {
        if (cell[0] == target[0] && cell[1] == target[1] && cell[2] == target[2]) break;
        int first = -1;
        for (int a = 0; a < 3; ++a) {
            if (direction[a] == 0 || cell[a] == target[a]) continue;
            if (first < 0 || (int64_t)next[a] * delta[first] < (int64_t)next[first] * delta[a]) first = a;
        }
        if (first < 0) break;
        ++g_steps;
        const int32_t num = next[first], den = delta[first];
        for (int a = 0; a < 3; ++a) {
            if (direction[a] == 0 || cell[a] == target[a]) continue;
            if ((int64_t)next[a] * den == (int64_t)num * delta[a]) {
                cell[a] += direction[a];
                next[a] += Q8;
            }
        }
        if (!cell_in_grid(cell, dims)) break;
        if (cell[0] != target[0] || cell[1] != target[1] || cell[2] != target[2]) {
            const uint32_t idx = grid_offset(cell, dims[0], dims[1]);
            if (grid[idx] == 0) grid[idx] = 1;
        }
    }
    if (cell_in_grid(target, dims)) grid[grid_offset(target, dims[0], dims[1])] = 2;
}

// ---------------- front ends: per pixel (valid, end[3]) --------------------------------------------
struct Px {
    bool valid;
    int32_t end[3];
};

// REF: hw/m7/frame_depth.cpp expressions (status 0 frames only; origin computed separately).
// Also returns the pre-floor double coordinate per axis for the analysis.
void ref_front(const PF& f, uint32_t pixel, float z, Px& o, double eref[3]) {
    o.valid = false;
    o.end[0] = o.end[1] = o.end[2] = 0;
    if (!(z > 0.1f && z < 10.0f)) return;
    const float u = static_cast<float>(pixel % f.w);
    const float v = static_cast<float>(pixel / f.w);
    float camera[3], world[3];
    for (int r = 0; r < 3; ++r)
        camera[r] = (f.ik[3*r]*u + f.ik[3*r+1]*v + f.ik[3*r+2]) * z;
    bool good = true;
    for (int r = 0; r < 3; ++r) {
        world[r] = f.wc[4*r]*camera[0] + f.wc[4*r+1]*camera[1] + f.wc[4*r+2]*camera[2] + f.wc[4*r+3];
        good = good && finite_float(world[r]);
    }
    if (!good) return;
    for (int a = 0; a < 3; ++a) {
        const double pre = (static_cast<double>(world[a]) - f.origin[a]) / f.voxel * Q8;
        eref[a] = pre;
        const double c = std::floor(pre);
        if (!(c >= -FPGA_M7_FRAME_Q8_LIMIT && c < FPGA_M7_FRAME_Q8_LIMIT)) return;
        o.end[a] = static_cast<int32_t>(c);
    }
    o.valid = true;
}

// FX frame constants (computed by the PS driver; DESIGN.md section 2.2, exact operation order).
const int kFacc = 36;   // accumulator fractional bits (Gu, Gv, G0)
const int kFp = 22;     // fractional bits of P (27-bit signed DSP A operand) and of O, E
struct FxK {
    int64_t Gu[3], Gv[3], G0[3];  // round_half_even(G * 2^36), |G| < 16  -> 41-bit signed
    int64_t O[3];                 // round_half_even(O * 2^22), 0 <= O < 8192 -> 36-bit
    int32_t origin[3];            // reference origin (Q8 int), status 3 as before
    uint32_t c_lo, c_hi, c20;     // valid <=> c_lo <= code <= c_hi; c20 = max code with f(code) <= 20
    double pmax;                  // max |P| over the image corners (all axes), real
    double reach;                 // max over axes of max|P_a| * c_hi + |O_a|, Q8 units
    int status;                   // 0 ok, 2/3 as reference, 5 = outside the fixed-point envelope
};

double rne_scale(double x, int bits) { return std::nearbyint(std::ldexp(x, bits)); }  // FE_TONEAREST

float f_of_code(uint32_t code, float unit) { return static_cast<float>(code) * unit; }

int fx_prepare(const PF& f, FxK& k) {
    // status 2 checks (reference)
    if (!finite_double(f.voxel) || f.voxel <= 0) return k.status = 2;
    for (int a = 0; a < 3; ++a) if (!finite_double(f.origin[a])) return k.status = 2;
    for (int i = 0; i < 9; ++i) if (!finite_float(f.ik[i])) return k.status = 2;
    for (int i = 0; i < 12; ++i) if (!finite_float(f.wc[i])) return k.status = 2;
    const uint32_t dims[3] = {f.nx, f.ny, f.nz};
    for (int a = 0; a < 3; ++a) {  // reference origin, verbatim
        const double c = std::floor((static_cast<double>(f.wc[4*a+3]) - f.origin[a]) / f.voxel * Q8);
        if (!finite_double(c) || c < 0 || c >= dims[a] * Q8) return k.status = 3;
        k.origin[a] = static_cast<int32_t>(c);
    }
    // per-frame coefficients, double, fixed order (no FMA: -ffp-contract=off)
    const double s = static_cast<double>(f.unit) * 256.0 / f.voxel;
    k.pmax = 0; k.reach = 0;
    // validity thresholds (exact: f32 product is monotone in code)
    k.c_lo = 65536; k.c_hi = 0; k.c20 = 0;
    for (uint32_t c = 1; c < 65536; ++c) {
        const float z = f_of_code(c, f.unit);
        if (z > 0.1f && z < 10.0f) { if (c < k.c_lo) k.c_lo = c; k.c_hi = c; }
        if (z <= 20.0f) k.c20 = c;
    }
    for (int a = 0; a < 3; ++a) {
        double A[3];
        for (int j = 0; j < 3; ++j) {
            double acc = static_cast<double>(f.wc[4*a]) * static_cast<double>(f.ik[j]);
            acc = acc + static_cast<double>(f.wc[4*a+1]) * static_cast<double>(f.ik[3+j]);
            acc = acc + static_cast<double>(f.wc[4*a+2]) * static_cast<double>(f.ik[6+j]);
            A[j] = acc * s;
        }
        k.Gu[a] = static_cast<int64_t>(rne_scale(A[0], kFacc));
        k.Gv[a] = static_cast<int64_t>(rne_scale(A[1], kFacc));
        k.G0[a] = static_cast<int64_t>(rne_scale(A[2], kFacc));
        const double Oa = (static_cast<double>(f.wc[4*a+3]) - f.origin[a]) * 256.0 / f.voxel;
        k.O[a] = static_cast<int64_t>(rne_scale(Oa, kFp));
        double pm = 0;
        for (int cu = 0; cu < 2; ++cu)
            for (int cv = 0; cv < 2; ++cv) {
                const double u = cu ? f.w - 1.0 : 0.0, v = cv ? f.h - 1.0 : 0.0;
                pm = std::fmax(pm, std::fabs(A[0] * u + A[1] * v + A[2]));
            }
        k.pmax = std::fmax(k.pmax, pm);
        k.reach = std::fmax(k.reach, pm * k.c_hi + std::fabs(Oa));
    }
    if (k.c_lo > k.c_hi) return k.status = 5;
    if (!(k.pmax < 15.5)) return k.status = 5;          // P27 envelope with margin
    if (!(k.reach < 1048576.0)) return k.status = 5;    // |E| < 2^20 (end fits 22-bit signed)
    return k.status = 0;
}

// FX per pixel (bit exact model of the datapath). fp = fractional bits of P (22 design, 13 control).
// Returns E (Q8 units, fixed point with 22 fractional bits) for the analysis.
inline int64_t fx_E(const FxK& k, int a, uint32_t u, uint32_t v, uint32_t code, int fp) {
    // accumulators: R(v) = G0 + 2^(35-fp) + v*Gv, P(u,v) = R(v) + u*Gu (integer, exact; the hardware
    // accumulates the same integers row by row / pixel by pixel)
    const int sh = kFacc - fp;
    const int64_t pacc = k.G0[a] + (int64_t(1) << (sh - 1)) + int64_t(v) * k.Gv[a] + int64_t(u) * k.Gu[a];
    const int64_t p = pacc >> sh;                   // arithmetic: round half up to fp bits
    if (fp == kFp && (p >= (int64_t(1) << 26) || p < -(int64_t(1) << 26))) { std::fprintf(stderr, "P27 overflow\n"); std::exit(3); }
    return (p << (kFp - fp)) * int64_t(code) + k.O[a];   // DSP: A*B + C, F = 22
}

// ---------------- back end: Dlat2 + walk + merge ----------------------------------------------------
struct Back {
    std::vector<uint8_t> grid;
    std::vector<uint8_t> ev;
    std::vector<uint8_t> blk;
    uint32_t accepted, invalid, clipped;
    unsigned long long rays, steps;
};

void back_run(const PF& f, const int32_t origin[3], const std::vector<Px>& px, bool dlat, Back& b) {
    const uint32_t dims[3] = {f.nx, f.ny, f.nz};
    b.grid.assign(kCap, 0);
    b.ev.assign(kCap, 0);
    b.blk.assign(FPGA_M7_FRAME_MAX_WIDTH / 2, 0);  // never cleared in HW either (even row rewrites)
    b.accepted = b.invalid = b.clipped = 0;
    b.rays = 0;
    const unsigned long long s0 = g_steps;
    bool prev_in = false, prev_cl = false;
    const uint32_t np = f.w * f.h;
    for (uint32_t p = 0; p < np; ++p) {
        const uint32_t u = p % f.w, v = p / f.w;
        const Px& x = px[p];
        bool inside = true;
        for (int a = 0; a < 3; ++a)
            if (x.end[a] < 0 || x.end[a] >= (int32_t)(dims[a] * Q8)) inside = false;
        const bool valid = x.valid;
        const bool is_in = valid && inside, is_cl = valid && !inside;
        const bool u_odd = u & 1, v_odd = v & 1;
        const uint32_t bcol = u >> 1;
        uint8_t above = v_odd ? b.blk[bcol] : 0;
        const bool in_seen = (above & 1) || (u_odd && prev_in);
        const bool cl_seen = (above & 2) || (u_odd && prev_cl);
        const uint8_t mark = (in_seen || is_in ? 1 : 0) | (cl_seen || is_cl ? 2 : 0);
        if (!v_odd) b.blk[bcol] = mark;
        bool ev_seen = false;
        if (is_in) {
            const uint32_t e = ((uint32_t)(x.end[2] >> 8) * f.ny + (uint32_t)(x.end[1] >> 8)) * f.nx +
                               (uint32_t)(x.end[0] >> 8);
            ev_seen = b.ev[e] != 0;
            b.ev[e] = 1;
        }
        prev_in = is_in;
        prev_cl = is_cl;
        const bool keep_in = !in_seen || !ev_seen;
        const bool keep = !dlat || (inside ? keep_in : !cl_seen);
        if (!valid) ++b.invalid; else if (inside) ++b.accepted; else ++b.clipped;
        if (valid && keep) {
            ++b.rays;
            add_ray(origin, x.end, dims, b.grid.data());
        }
    }
    b.steps = g_steps - s0;
}

struct Cmp {  // grid comparison inside nx*ny*nz; rows = REF state, cols = other arm
    long m[3][3];
};
void cmp_grid(const PF& f, const uint8_t* a, const uint8_t* b, Cmp& c) {
    std::memset(&c, 0, sizeof c);
    const uint32_t n = f.nx * f.ny * f.nz;
    for (uint32_t i = 0; i < n; ++i) c.m[a[i]][b[i]]++;
}

const long double kU = std::ldexp(1.0L, -24);

}  // namespace

int main(int argc, char** argv) {
    if (argc < 7) { std::fprintf(stderr, "usage: fx_golden PACK GOLDEN REFFLAGS TAG OUTDIR DLAT [MAXFRAMES]\n"); return 2; }
    size_t psz = 0, gsz = 0, rsz = 0;
    const uint8_t* pb = map_file(argv[1], &psz);
    const uint8_t* gb = map_file(argv[2], &gsz);
    const uint8_t* rb = map_file(argv[3], &rsz);
    if (!pb || !gb || !rb) { std::fprintf(stderr, "cannot map inputs\n"); return 2; }
    std::vector<PF> fr = load_pack(pb, psz);
    const int kRec = 12 + kCap;
    if (fr.empty() || gsz != fr.size() * (size_t)kRec || rsz != fr.size() * (size_t)kCap) {
        std::fprintf(stderr, "bad sizes\n");
        return 2;
    }
    const std::string tag = argv[4], outdir = argv[5];
    const bool dlat = std::atoi(argv[6]) != 0;
    size_t nfr = fr.size();
    if (argc > 7) nfr = std::min(nfr, (size_t)std::atol(argv[7]));
    FILE* csv = std::fopen((outdir + "/" + tag + ".csv").c_str(), "w");
    FILE* ffr = std::fopen((outdir + "/" + tag + ".ref.flags").c_str(), "wb");
    FILE* ffx = std::fopen((outdir + "/" + tag + ".fx.flags").c_str(), "wb");
    FILE* ffc = std::fopen((outdir + "/" + tag + ".fxc.flags").c_str(), "wb");
    std::fprintf(csv,
        "frame,st_fx,i0_bytes,valid_ref,valid_fx,valid_diff,"
        "endd1,endd_gt1,ep_mis,ep_unattr,bfx_viol,max_efx,max_eref,max_bref,"
        "known_union,cell_mis,occ_ref,occ_lost,occ_extra,free_ref,free_to_unk,unk_to_free,free_to_occ,"
        "rays_ref,rays_fx,steps_ref,steps_fx,acc_ref,acc_fx,cl_ref,cl_fx,"
        "c_bfx_viol,c_ep_mis,c_ep_unattr,c_cell_mis,c_known_union,c_occ_lost,c_endd_gt1,"
        "pmax,reach,c_lo,c_hi,c20,"
        "st_npos,st_le20,st_maxle20,st_mingt20,st_valid,st_minvalid,st_maxcode,code_crc,"
        "ep_fx_right,ep_ref_right,ex_occ,ex_fx_occ_lost,ex_fx_occ_extra,ex_ref_occ_lost,ex_ref_occ_extra,ex_fx_mis,ex_ref_mis\n");
    std::vector<float> zbuf(FPGA_M7_FRAME_MAX_PIXELS);
    std::vector<Px> pref, pfx, pfc, pex;
    Back bref, bfx, bfc, bex;
    const bool diag = argc > 8 && std::string(argv[8]) == "diag";
    long tot_i0 = 0, tot_bv = 0, tot_unattr = 0, tot_cbv = 0, tot_cun = 0, st_bad = 0;
    for (size_t i = 0; i < nfr; ++i) {
        const PF& f = fr[i];
        const uint32_t np = f.w * f.h;
        const uint32_t qnan = 0x7fc00000u;
        for (uint32_t p = 0; p < np; ++p) {
            if (f.depth[p] == 0) std::memcpy(&zbuf[p], &qnan, 4);
            else zbuf[p] = static_cast<float>(f.depth[p]) * f.unit;
        }
        FxK k;
        fx_prepare(f, k);
        if (k.status) { ++st_bad; std::fprintf(stderr, "frame %zu fx status %d\n", i, k.status); continue; }
        pref.resize(np); pfx.resize(np); pfc.resize(np); pex.resize(np);
        long ep_fx_right = 0, ep_ref_right = 0;
        long valid_ref = 0, valid_fx = 0, valid_diff = 0, endd1 = 0, endd_gt1 = 0, ep_mis = 0, ep_un = 0, bv = 0;
        long c_bv = 0, c_ep = 0, c_un = 0, c_gt1 = 0;
        long double max_efx = 0, max_eref = 0, max_bref = 0;
        // stats (PL front-end statistics, DESIGN.md section 3.3)
        uint32_t st_npos = 0, st_le20 = 0, st_maxle20 = 0, st_mingt20 = 65535, st_valid = 0, st_minvalid = 65535,
                 st_maxcode = 0;
        bool any_gt20 = false;
        for (uint32_t p = 0; p < np; ++p) {
            const uint32_t u = p % f.w, v = p / f.w, code = f.depth[p];
            double eref[3] = {0, 0, 0};
            ref_front(f, p, zbuf[p], pref[p], eref);
            // FX
            Px& x = pfx[p];
            Px& xc = pfc[p];
            x.valid = code >= k.c_lo && code <= k.c_hi;
            xc.valid = x.valid;
            pex[p].valid = false; pex[p].end[0] = pex[p].end[1] = pex[p].end[2] = 0;
            int64_t E[3], Ec[3];
            for (int a = 0; a < 3; ++a) {
                E[a] = fx_E(k, a, u, v, code, kFp);
                Ec[a] = fx_E(k, a, u, v, code, 13);
                x.end[a] = x.valid ? static_cast<int32_t>(E[a] >> kFp) : 0;
                xc.end[a] = xc.valid ? static_cast<int32_t>(Ec[a] >> kFp) : 0;
            }
            if (code >= 1) {
                ++st_npos;
                if (code > st_maxcode) st_maxcode = code;
                if (code <= k.c20) { ++st_le20; if (code > st_maxle20) st_maxle20 = code; }
                else { any_gt20 = true; if (code < st_mingt20) st_mingt20 = code; }
            }
            if (x.valid) { ++st_valid; if (code < st_minvalid) st_minvalid = code; }
            valid_ref += pref[p].valid;
            valid_fx += x.valid;
            if (pref[p].valid != x.valid) { ++valid_diff; continue; }
            if (!x.valid) continue;
            // analysis: exact value and bounds
            const long double z_ex = (long double)code * (long double)f.unit;
            const long double s = 256.0L / (long double)f.voxel;
            const long double uv[3] = {(long double)u, (long double)v, 1.0L};
            long double cam[3], camabs[3];
            for (int r = 0; r < 3; ++r) {
                long double c = 0, ca = 0;
                for (int j = 0; j < 3; ++j) {
                    c += (long double)f.ik[3*r+j] * uv[j];
                    ca += std::fabs((long double)f.ik[3*r+j] * uv[j]);
                }
                cam[r] = c * z_ex;
                camabs[r] = ca * z_ex;
            }
            const long double g9 = 9 * kU / (1 - 9 * kU);
            bool ep_differs = false, ep_attr = true, c_ep_differs = false, c_ep_attr = true;
            bool fx_right = true, ref_right = true;
            pex[p].valid = true;
            for (int a = 0; a < 3; ++a) {
                long double w = (long double)f.wc[4*a+3], wa = std::fabs((long double)f.wc[4*a+3]);
                for (int b = 0; b < 3; ++b) {
                    w += (long double)f.wc[4*a+b] * cam[b];
                    wa += std::fabs((long double)f.wc[4*a+b]) * camabs[b];
                }
                const long double Eex = (w - (long double)f.origin[a]) * s;
                pex[p].end[a] = (int32_t)floorl(Eex);
                { const int32_t ve = floor_q8(pex[p].end[a]); if (ve != floor_q8(x.end[a])) fx_right = false; if (ve != floor_q8(pref[p].end[a])) ref_right = false; }
                const long double bref = s * g9 * wa + std::fabs(Eex) * std::ldexp(1.0L, -50);
                const long double efx = std::fabs(std::ldexp((long double)E[a], -kFp) - Eex);
                const long double efc = std::fabs(std::ldexp((long double)Ec[a], -kFp) - Eex);
                const long double eref_err = std::fabs((long double)eref[a] - Eex);
                const long double bfx = (long double)code * (std::ldexp(1.0L, -23) + std::ldexp(1.0L, -25)) +
                                        std::ldexp(1.0L, -23) + std::ldexp(1.0L, -37);
                if (efx > bfx) ++bv;
                if (efc > bfx) ++c_bv;
                if (efx > max_efx) max_efx = efx;
                if (eref_err > max_eref) max_eref = eref_err;
                if (bref > max_bref) max_bref = bref;
                const int32_t dq = x.end[a] - pref[p].end[a];
                if (dq == 1 || dq == -1) ++endd1; else if (dq != 0) ++endd_gt1;
                const int32_t dqc = xc.end[a] - pref[p].end[a];
                if (dqc > 1 || dqc < -1) ++c_gt1;
                const int32_t vr = floor_q8(pref[p].end[a]), vf = floor_q8(x.end[a]), vc = floor_q8(xc.end[a]);
                if (vr != vf) {
                    ep_differs = true;
                    const long double kb = 256.0L * (long double)(vr > vf ? vr : vf);
                    if (std::fabs(Eex - kb) > bfx + bref) ep_attr = false;
                }
                if (vr != vc) {
                    c_ep_differs = true;
                    const long double kb = 256.0L * (long double)(vr > vc ? vr : vc);
                    if (std::fabs(Eex - kb) > bfx + bref) c_ep_attr = false;
                }
            }
            if (ep_differs) { ++ep_mis; if (!ep_attr) ++ep_un; ep_fx_right += fx_right; ep_ref_right += ref_right; }
            if (c_ep_differs) { ++c_ep; if (!c_ep_attr) ++c_un; }
        }
        back_run(f, k.origin, pref, dlat, bref);
        back_run(f, k.origin, pfx, dlat, bfx);
        back_run(f, k.origin, pfc, dlat, bfc);
        Cmp ce_fx, ce_ref;
        if (diag) { back_run(f, k.origin, pex, dlat, bex); cmp_grid(f, bex.grid.data(), bfx.grid.data(), ce_fx); cmp_grid(f, bex.grid.data(), bref.grid.data(), ce_ref); }
        else { std::memset(&ce_fx, 0, sizeof ce_fx); std::memset(&ce_ref, 0, sizeof ce_ref); }
        // I0: REF arm vs product
        const uint8_t* g = gb + i * (size_t)kRec;
        const uint8_t* want = dlat ? rb + i * (size_t)kCap : g + 12;
        uint32_t cnt[3] = {bref.accepted, bref.invalid, bref.clipped};
        long i0 = 0;
        for (int b = 0; b < 12; ++b) i0 += reinterpret_cast<uint8_t*>(cnt)[b] != g[b];
        for (uint32_t c = 0; c < kCap; ++c) i0 += bref.grid[c] != want[c];
        tot_i0 += i0;
        Cmp cf, cc;
        cmp_grid(f, bref.grid.data(), bfx.grid.data(), cf);
        cmp_grid(f, bref.grid.data(), bfc.grid.data(), cc);
        const long known_union = (long)(f.nx * f.ny * f.nz) - cf.m[0][0];
        const long cell_mis = known_union - cf.m[1][1] - cf.m[2][2];
        const long c_known = (long)(f.nx * f.ny * f.nz) - cc.m[0][0];
        const long c_mis = c_known - cc.m[1][1] - cc.m[2][2];
        const uint32_t crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0), reinterpret_cast<const Bytef*>(f.depth), 2 * np);
        tot_bv += bv; tot_unattr += ep_un; tot_cbv += c_bv; tot_cun += c_un;
        std::fprintf(csv,
            "%zu,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.6Le,%.6Le,%.6Le,"
            "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
            "%llu,%llu,%llu,%llu,%u,%u,%u,%u,"
            "%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
            "%.6f,%.1f,%u,%u,%u,"
            "%u,%u,%u,%u,%u,%u,%u,%08x,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
            i, k.status, i0, valid_ref, valid_fx, valid_diff, endd1, endd_gt1, ep_mis, ep_un, bv, max_efx, max_eref,
            max_bref, known_union, cell_mis, cf.m[2][0] + cf.m[2][1] + cf.m[2][2], cf.m[2][0] + cf.m[2][1],
            cf.m[0][2] + cf.m[1][2], cf.m[1][0] + cf.m[1][1] + cf.m[1][2], cf.m[1][0], cf.m[0][1], cf.m[1][2],
            bref.rays, bfx.rays, bref.steps, bfx.steps, bref.accepted, bfx.accepted, bref.clipped, bfx.clipped,
            c_bv, c_ep, c_un, c_mis, c_known, cc.m[2][0] + cc.m[2][1], c_gt1,
            k.pmax, k.reach, k.c_lo, k.c_hi, k.c20,
            st_npos, st_le20, st_maxle20, any_gt20 ? st_mingt20 : 0u, st_valid, st_valid ? st_minvalid : 0u,
            st_maxcode, crc, ep_fx_right, ep_ref_right,
            ce_fx.m[2][0] + ce_fx.m[2][1] + ce_fx.m[2][2], ce_fx.m[2][0] + ce_fx.m[2][1], ce_fx.m[0][2] + ce_fx.m[1][2],
            ce_ref.m[2][0] + ce_ref.m[2][1], ce_ref.m[0][2] + ce_ref.m[1][2],
            ce_fx.m[0][1] + ce_fx.m[0][2] + ce_fx.m[1][0] + ce_fx.m[1][2] + ce_fx.m[2][0] + ce_fx.m[2][1],
            ce_ref.m[0][1] + ce_ref.m[0][2] + ce_ref.m[1][0] + ce_ref.m[1][2] + ce_ref.m[2][0] + ce_ref.m[2][1]);
        std::fwrite(bref.grid.data(), 1, kCap, ffr);
        std::fwrite(bfx.grid.data(), 1, kCap, ffx);
        std::fwrite(bfc.grid.data(), 1, kCap, ffc);
    }
    std::fclose(csv); std::fclose(ffr); std::fclose(ffx); std::fclose(ffc);
    std::fprintf(stderr, "FX %s dlat=%d frames=%zu st_bad=%ld i0_bytes=%ld bfx_viol=%ld ep_unattr=%ld ctl_bfx_viol=%ld ctl_ep_unattr=%ld\n",
                 tag.c_str(), (int)dlat, nfr, st_bad, tot_i0, tot_bv, tot_unattr, tot_cbv, tot_cun);
    return 0;
}
