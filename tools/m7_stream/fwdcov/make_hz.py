#!/usr/bin/env python3
"""FWD_COV instrument (not product code): write a copy of m7_frame_stream.cpp whose
convert-loop read-modify-writes of blk_mark and ev_pl follow a configurable RAM
write-visibility model, to predict what an RTL without forwarding would output.

Env at run time (read once per convert() call):
  FWDCOV_NOFWD=1   drop the explicit forwarding (like -DFPGA_M7_HWFAULT_DLAT_NOFWD in RTL)
  FWDCOV_D_BLK=D   a blk_mark read does not see the writes of the last D iterations
  FWDCOV_D_EV=D    same for ev_pl
D = 0 is an ideal RAM (C semantics). The schedule of the 8051f22 RTL corresponds to
D_BLK = 1 (load issued in ST_54, store in ST_55, LUTRAM read-first) and D_EV = 0
(load and store on the same BRAM port in ST_59, read-first).
Pending writes are committed when the loop ends (pipeline drain).
usage: make_hz.py SRC OUT"""
import sys

src, out = sys.argv[1], sys.argv[2]
s = open(src).read()


def rep(old, new, count=1):
    global s
    n = s.count(old)
    assert n == count, (old, n)
    s = s.replace(old, new)


# 1. runtime switches + delayed-write model, before convert()
rep("void convert(hls::stream<float>& in, hls::stream<Ray>& rays,",
    """struct HzRam {  // FWD_COV: writes become visible D iterations late
    int D;
    int n;
    unsigned addr[8];
    unsigned data[8];
    bool valid[8];
};
static int hz_env(const char* k) { const char* v = std::getenv(k); return (v && *v) ? std::atoi(v) : 0; }
template <typename T>
static void hz_push(HzRam& q, T* mem, unsigned a, unsigned d, bool v) {
    if (q.D == 0) { if (v) mem[a] = d; return; }
    q.addr[q.n] = a; q.data[q.n] = d; q.valid[q.n] = v; ++q.n;
    if (q.n > q.D) {  // commit the oldest
        if (q.valid[0]) mem[q.addr[0]] = q.data[0];
        for (int i = 1; i < q.n; ++i) { q.addr[i-1] = q.addr[i]; q.data[i-1] = q.data[i]; q.valid[i-1] = q.valid[i]; }
        --q.n;
    }
}
template <typename T>
static void hz_drain(HzRam& q, T* mem) {
    for (int i = 0; i < q.n; ++i) if (q.valid[i]) mem[q.addr[i]] = q.data[i];
    q.n = 0;
}

void convert(hls::stream<float>& in, hls::stream<Ray>& rays,""")

rep("""    for (int k = 0; k < kFwd; ++k) {
#pragma HLS UNROLL
        fe_a[k] = 0; fe_v[k] = false; fb_a[k] = 0; fb_d[k] = 0; fb_v[k] = false;
    }
""", """    for (int k = 0; k < kFwd; ++k) {
#pragma HLS UNROLL
        fe_a[k] = 0; fe_v[k] = false; fb_a[k] = 0; fb_d[k] = 0; fb_v[k] = false;
    }
    const bool hz_nofwd = hz_env("FWDCOV_NOFWD") != 0;
    HzRam hq_blk = {hz_env("FWDCOV_D_BLK"), 0, {}, {}, {}};
    HzRam hq_ev = {hz_env("FWDCOV_D_EV"), 0, {}, {}, {}};
    if (hq_blk.D < 0 || hq_blk.D > 7 || hq_ev.D < 0 || hq_ev.D > 7) std::abort();
    unsigned hz_emit = 0;
""")

# 2. blk_mark read: forwarding only if !hz_nofwd
rep("""            above = blk_mark[bcol];
#ifndef FPGA_M7_HWFAULT_DLAT_NOFWD_ACTIVE
            // forward the newest matching write (k = 0 is the newest)
            for (int k = kFwd - 1; k >= 0; --k)
                if (fb_v[k] && fb_a[k] == bcol) above = fb_d[k];
#endif""", """            above = blk_mark[bcol];
            if (!hz_nofwd)
            // forward the newest matching write (k = 0 is the newest)
            for (int k = kFwd - 1; k >= 0; --k)
                if (fb_v[k] && fb_a[k] == bcol) above = fb_d[k];""")
rep("        if (!v_odd) blk_mark[bcol] = mark;",
    "        hz_push(hq_blk, blk_mark, bcol, mark, !v_odd);")

# 3. ev_pl
rep("""            ev_seen = ev_pl[eidx] != 0;
#ifndef FPGA_M7_HWFAULT_DLAT_NOFWD_ACTIVE
            for (int k = 0; k < kFwd; ++k)
                if (fe_v[k] && fe_a[k] == eidx) ev_seen = true;
#endif
            ev_pl[eidx] = 1;
        }""", """            ev_seen = ev_pl[eidx] != 0;
            if (!hz_nofwd)
            for (int k = 0; k < kFwd; ++k)
                if (fe_v[k] && fe_a[k] == eidx) ev_seen = true;
        }
        hz_push(hq_ev, ev_pl, eidx, 1, is_in);""")

# 4. drain after the loop: find the end of convert_loop = the counts write
rep("""        if (emit) rays.write(r);
    }
    Ray sentinel;""", """        if (emit) rays.write(r);
    }
    hz_drain(hq_blk, blk_mark);
    hz_drain(hq_ev, ev_pl);
    std::fprintf(stderr, "HZ_EMIT %u\\n", hz_emit);
    Ray sentinel;""")
rep("        if (emit) rays.write(r);\n    }\n    hz_drain", "        if (emit) { rays.write(r); ++hz_emit; }\n    }\n    hz_drain")
open(out, "w").write(s)
print("wrote", out)
