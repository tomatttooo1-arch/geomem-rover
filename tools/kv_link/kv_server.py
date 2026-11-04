#!/usr/bin/env python3
"""KV260 end of the Nano <-> KV260 frame channel (host/kv_link.py, B1 2026-10-11).

  --core pl    the frame core through M7BackendSelector("pl") (--device devmem on the
               board, --device mock = g++ core behind the register map, host only)
  --core cpu   the g++ CPU reference core on this machine's CPU
  --core null  no core: answers at once with every voxel unknown and every pixel
               counted invalid (channel cost only; the Nano gets no_valid_grid_samples)
  --wait irq|poll   how the PL core's completion is awaited (pl only; explicit, no fallback)
  --pl-timebase permit|encoder|none   PL timebase block paired with KV mono in SYNC answers
               (B2 clock mapping). Read only through a UIO node of that name whose MAGIC
               matches; otherwise degraded, reason in every SYNC_ACK and in summary.json.
  --pl-clk-hz  nominal PL clock (default: sysinfo CLK_HZ when the m7_sysinfo UIO node is there)
  --roi-core pl|cpu|none   region (object ROI) core run on the same float32 depth after the
               geometry core for requests that carry boxes (PC_DATAPLANE A.2). Explicit, no
               fallback: cpu = E2 C reference (g++); none = ROI requests answered 0x100;
               pl = the E2 core of the merged bitstream m7_pl_merge (host/kv_roi_pl.py): needs
               --core pl (its DMA provider holds the depth copy), the sysinfo block with
               SLOT_MAP bit 9 and the UIO node --roi-uio (m7_roi) at 0xA009_0000 -- refused at
               startup otherwise (e.g. on 0x0117bc0a). With --device mock the ROI core is the
               host mock (--roi-mock-mutant row_shift|threshold = positive controls).
  --rolling none|cpu|pl   (v3 L2, 2026-10-26; default none = exactly as before) serve ROLL
               (host/kv_rolling.py): the rolling map fused on the KV260 after the Nano has
               validated each frame. cpu = the g++ core on the A53; pl = the PL core of the v3
               candidate bitstream m7_pl_v3f (slot 5, UIO --rolling-uio m7_rolling at
               0xA005_0000, SLOT_MAP bit 5 -- refused at startup otherwise), reading the frame
               core's DDR output buffer in place (needs --core pl). --device mock: host mock
               (--rolling-mock-mutant drop_last_delta = positive control).
  --rolling-grid none|cmodel  (B1, 2026-11-02; default none = exactly as before) with --rolling cpu:
               also answer the navigation-grid op (ROLL v2, HELLO "rolling_grid") with the C model
               of hw/b1_costmap on a DUMP after each INTEGRATE -- the A53 / host stand-in for the PL
               grid op (runs/nationals-prelim-20261102/B1/NANO_SIDE.md).
               pl (INT_SW 2026-11-02) with --rolling pl: the fused core's own op GRID (B1h, next bitstream),
               started right after each accepted INTEGRATE that asks for it; one read-only GRID at start
               probes the bitstream -- BAD_ARGS (current bitstreams) => no "rolling_grid", all as with none.
               --device mock: the mock shell/core are the GRID build (= the next bitstream).
  --nav none|soft|pl  (CTRLPS2 N2, 2026-11-03; default none = exactly as before) serve the navigation messages
               NAV_GOAL / NAV_ODOM / NAV_PATH -> NAV_CMD (host/kv_nav_service.py, HELLO feature "nav";
               runs/nationals-prelim-20261102/CTRLPS0/DESIGN.md 2.1/3): the KV260 PS navigation core gets the goal,
               the Nano's odometry and the G1 global route and answers (v, w, state) at 20 Hz. soft = potential
               field built on the A53 (dev2 planner_hw_ref), pl = by the PL planner core; the field is the core's
               business (--nav-core kv_nav: CTRLPS1 host.kv_nav.make_core, refused at startup when absent).
               --nav-core mock = the host test stand-in (turn, then drive along the route; NOT the design's tracker).
               CTRLPS3: kv_nav needs a costmap window -- the B1 grid plane of --rolling cpu --rolling-grid cmodel or
               --rolling pl --rolling-grid pl (Nano CAR_PL_GRID=1), refused otherwise; pl (the PL potential field,
               DRV5: m5 planner core via hw/planner/drv/libplanner_drv.so; driver absent -> warning + soft). --nav-sim-world JSON = host tests only (host/kv_nav.SimWorldGrid).
               CTRLPS4: the window gets the Nano bridge's swept clearing (--nav-swept-clear-s, default 300 s) and
               front-sector seed (--nav-front-seed-m, default off) on the KV (host/kv_nav.KvWindowLayer).
  --dlat2 0|1  (INT_SW 2026-11-02; default 0) G1d ray thinning of the next bitstream's frame core
               (register DLAT2_EN, host/m7_backend.setup_dlat2). 0 never writes the register on a current
               or freshly loaded bitstream; 1 on a bitstream without it: warning, runs without thinning.
  (GEO_FX3 2026-11-04: --d16 is gone.) --core pl drives the fixed-point frame core, which reads only the
               request's uint16 codes (no A53 conversion; the float32 depth crc comes from the core). The
               per-frame constants are computed in C++ (hw/m7/frame_fx.cpp, host/m7_frame_fx.py, built here
               with -ffp-contract=off). The A53 CPU reference runs instead when the unit fails d16_unit_ok,
               when the PS status is not 0 (2/3 as the reference, 5 = outside the fixed-point envelope) or
               when the loaded bitstream has no fixed-point registers (warning); the A53 still makes float32
               depth for the ROI core and for a frame the core rejects. The 4 depth-statistics registers
               (ST_NPOS / ST_LE20 / ST_MM0 / ST_MM1) are read after every PL frame: "fx_stats" of the last
               geometry record (live info) and stderr every --fx-stats-log-every frames (0 = never).
  --roi-concurrent 0|1  (INT_SW 2026-11-02; default 0 = exactly as before) 1: the ROI core starts in a
               worker thread together with the geometry core (separate registers and DMA buffers; the ROI
               only needs the depth, the geometry status gates its answer afterwards as before); the
               answer leaves when both are done.
  --roi-wait poll|irq  (SW_M1 2026-11-03; default poll = exactly as before) how the PL ROI core's completion is
               awaited. irq: its interrupt (UIO m7_roi, GIC SPI 91), blocking in poll() so the interpreter lock
               is free for the geometry thread (host/kv_roi_pl.py); a lost interrupt costs at most
               --roi-irq-slice-s (ap_done read anyway, counted irq_rescued). The line cannot be opened: warning,
               poll as before. Meant for --roi-concurrent 1 (BOARD2 3.5: the poll held the lock, +3.9 ms).
  --frame-source udp|kvcam  (NN-CAM2 2026-11-03; default udp = exactly as before) kvcam: depth frames from kv_cam's
               shared-memory ring on this board (--kvcam-name kv_cam_v1 | --kvcam-path FILE), turned into the request
               the Nano would send (--local-params JSON: camera pose / intrinsics / grid, host/kv_local_frames.py) and
               served by the same worker; after the geometry core the frame is fused in process (with --rolling;
               + GRID with --rolling-grid) and, with --nav and a "nav_goal" in --local-params, the 20 Hz navigation
               core builds its field from that grid. The UDP side keeps serving (ROLL / NAV / TELEM from a Nano).
               --stamp-log FILE: per-frame JSONL (camera stamps in their own columns; CLOCK_MONOTONIC instants
               read -> geometry -> GRID -> field), tools/nn_cam2/chain_report.py. --dump-core-input DIR (tests):
               the exact geometry-core input of every frame (also with --frame-source udp).
  Permit feed: with --core pl and the permit block present, every geometry start is
               preceded by a NEXT_FRAME_ID write (PU §3.4); the completion edge that grants
               permit needs --wait irq (GIE/IER). --permit-id-source nano (default, owner
               10-11): the id comes from the Nano's frame id per Nano session
               (identity.stream_instance), kv_regs.NanoFrameFeed -- a duplicated / older Nano
               frame id is judged a replay by the PL, a late packet of an ended session is
               dropped. kv-counter: the old KV-side counter (kv_regs.KvCounterFrameFeed),
               positive control only. --no-permit-feed turns it off.
  --ar off|drm   car AR display (docs/design/CAR_AR_RESULTS_20261011.md). off (default): nothing
               changes. drm: after each geometry answer has left, the frame (depth, voxel flags,
               ROI boxes/verdicts) goes to the resident presenter tools/kv_link/ar_helper/
               m7ar_present (host/kv_ar.py), which runs the AR core of m7_pl_merge and page-flips
               the DP output; it shows the STALE fail-safe frame on its own after --ar-stale-ms
               without a new frame (also if kv_server dies). Needs m7_pl_merge (SLOT_MAP bit 8,
               UIO m7_ar) -- refused at startup otherwise. Age = the Nano's AGE block (feature
               age_ext, announced only with --ar drm) + KV time, live or replay alike. A frame
               is shown as REPLAY when --ar-source replay or when the request carries the
               replay mark (ext_flags bit3, feature replay_ext, also only with --ar drm; the
               producer --data-source replay and nano_replay set it): REPLAY + its age while
               fresh, NOAGE / STALE by the same age rules as a live frame. Box labels: class names from the Nano's REQ
               LABEL block (feature label_ext, also only with --ar drm), else R+row_id. With --device mock the presenter runs its
               mock provider without a display (--ar-mock: the same with any --core). `xmutil desktop_disable` before, `desktop_enable`
               after (board only; --no-ar-desktop skips both).
  --ar-colour off|main   AR colour main view (owner 10-15, 2026-10-16; host/kv_colour.py): main = the
               colour picture (Nano D2C-aligned RGB888, pulled per served frame over its own UDP port
               --ar-colour-port, never inside the geometry request) as the main view, the PL's boxes /
               labels / panel / mini map kept on top, the PL depth picture as a PiP top right (PS
               composition in the helper). off (default) = exactly as before. The bypass port opens
               when --ar-colour main or --ar-colour-port >= 0 is given (then tune.json colour_main 0/1
               switches live, and HELLO announces stamp_ext); a frame without its own colour shows
               the last paired colour while that is at most --ar-colour-hold-ms old (owner 10-16,
               default 1000), after that the depth picture (TELEM ar.colour: unpaired reasons, held,
               held_age_ms_max, fallback).
               No-depth self pull (2026-10-17, docs/design/AR_DEPTH_INVALID_STALL_20261017.md): a depth frame
               with no valid pixel never reaches the KV (the Nano refuses it before the request), so the
               screen used to freeze and flip STALE. Now, --ar-nodepth-after-ms (150) after the last real
               frame, the AR sender itself WANTs the newest colour (same WANT bytes, its own tolerance) and
               shows it with an all-zero depth and no geometry: the core's mode 4 NODATA (no boxes,
               hatched mini map), composed with the colour by the helper; every --ar-nodepth-period-ms
               until a real frame. No geometry core runs and no NEXT_FRAME_ID is written: the permit
               (heartbeat) still lapses as before. --ar-nodepth-after-ms 0 = off.
               Colour refresh (2026-10-27, AR_LATENCY_PLAN_20261027 plan C): --ar-colour-refresh-ms / tune
               colour_refresh_ms > 0: between two geometry frames the last geometry picture (its boxes, depth
               inset, panel and its own growing age) is re-sent with the newest colour frame every that many ms,
               so the background follows the camera at 20-30 Hz; a real frame always wins; only while the last
               real frame is younger than nodepth_after_ms and its age stays under max_age_ms. 0 = off.
  --ar-fill native|python   AR sender depth chain (request codes -> u16 mm -> display stretch, 2026-10-16):
               native (default) = tools/kv_link/ar_helper/m7ar_fill.cpp compiled at start from the deployed
               source (host/kv_ar_native.py; no build -> no start), python = the numpy reference. Same bytes
               to the helper either way (docs/design/AR_COLOUR_MAIN_20261016.md section 13).
  --kv-fast off|native   E7 (2026-11-02): native = codes crc32, codes -> float32 conversion and depth crc32 in C++
               (host/kv_fast_native.py, tools/kv_link/kvfast/kv_fast.cpp built at start; NEON + ARMv8 crc32 on the
               A53), same bytes as the Python path; off (default, or $FPGA_KV_FAST) = exactly as before. A failed
               build warns and serves the Python path. Car switch: tools/car_run/fpga_profiles.env FPGA_KV_FAST_DEFAULT.
  --permit-model off|realtime   HOST ONLY (kv-permit 10-14). off (default): nothing changes.
               realtime: the permit block is host.kv_regs.RealtimePermitModel (the RTL's id /
               freshness window (power-on 1000 ms, as v3fc2) / FORCE_STOP / heartbeat-pin rules, PL clock = host clock at
               --pl-clk-hz, default 100 MHz) behind PlRegBlock("permit"), so the permit feed
               (NEXT_FRAME_ID), the e-stop latch (CTRL), SYNC (--pl-timebase permit) and TELEM
               all see a live permit block; the completion edge is raised after every geometry
               core call that returns (any --core). Refused with --device devmem and whenever
               a real permit UIO block was found (never shadows hardware).
  --permit-window-ms MS   v3fc2 (2026-10-26): write the PL permit freshness window (permit 0x44)
               once at start, before any frame, then read it back. Range [100, 5000] ms: a value
               outside is clamped with a warning (the PL also clamps). Not given: nothing is written
               and the bitstream's power-on window holds (v3fc2: 1000 ms). Owner 10-26: never a start
               refusal -- no permit block / unknown PL clock = warning, nothing written; a readback
               that differs (bitstream older than v3fc2: 0x44 read-only) = warning, the server runs
               with the PL's window. The write disarms; permission resumes at the next frame.
  E-stop (types 12/13) is latched by host/kv_estop.py, the only writer of permit CTRL; register
  blocks (sysinfo/permit/encoder) are opened once through UIO + MAGIC (host/kv_regs.py) and
  shared by SYNC, TELEM, e-stop and pl_cycles.

Receive buffer: --rcvbuf (8 MB) needs SO_RCVBUFFORCE, i.e. root (the board) or CAP_NET_ADMIN (host
containers: setpriv ambient net_admin, see tools/car_run/car_up.sh). As a plain user the kernel caps it
at rmem_max (416 KB read back on this host), less than one 640x400 request (9 x 60000 B chunks): the
tail chunk overflows whenever the receive thread is not scheduled during the burst, the request stays
incomplete and the Nano side times out (LinkTimeout; kv-timeout dig 10-14: rx_incomplete = timeouts in
every smoke-2 style host chain). Then a {"rcvbuf_warning": ...} line goes to stderr (rcvbuf_warning()).

Prints one JSON line {"port", "pid", ...} on stdout when ready. Exits on SIGTERM/SIGINT
or after --max-idle-s without any datagram, writing <out>/summary.json.
DevMem needs root; the board's DMA provider is chosen by FPGA_M7_DMA_PROVIDER/_HEAP
(10-10 bring-up: dmaheap/reserved).
Usage: kv_server.py --core pl|cpu|null [--device devmem|mock] [--wait irq|poll]
                    [--port 0] [--chunk-bytes 60000] [--out DIR] [--max-idle-s 900]
"""
import argparse
import json
import os
import signal
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from host.kv_link import CHUNK_FRAGMENTED, NUDGE_IDLE_S, KvLinkServer, NullCore  # noqa: E402


class RowInterleavedCpuCore(object):
    """Host stand-in only (--core cpu --cpu-threads T>1; line D1 server simulation, 2026-10-14).

    The same g++ CPU reference core, run on T threads at once (ctypes releases the GIL): thread i
    gets a full-size copy of the depth in which only image rows i, i+T, i+2T, ... keep their
    values and every other pixel is 0 m (= invalid to the core), so each valid pixel is traced
    exactly once, with its own u, v and z. The merge is exact, not approximate: the core's grid
    cell ends at 2 if any ray ends there, else 1 if any ray crosses it, else 0, whatever the pixel
    order, so the flags are the element-wise maximum of the T grids; accepted and clipped add up,
    and invalid = pixels - accepted - clipped (each pixel lands in exactly one of the three).
    The argument checks do not read the depth, so every thread returns the same status; on a
    nonzero status the outputs are left as the single core leaves them (flags untouched, counts 0).
    Why: at 640x400 the single-thread core takes ~100-130 ms on this host, the fake KV then
    answers too late for the runner's 200 ms geometry age gate (smoke 07); the board's PL core
    is not a CPU loop. tests/test_sim_loop_d1.py checks equality with the single core."""

    def __init__(self, core_fn, threads):
        from concurrent.futures import ThreadPoolExecutor
        from host.kv_link import GRID_CAPACITY, MAX_PIXELS
        if int(threads) < 2:
            raise ValueError("RowInterleavedCpuCore needs at least 2 threads")
        self.fn, self.threads = core_fn, int(threads)
        self.capacity = GRID_CAPACITY
        self._pool = ThreadPoolExecutor(max_workers=self.threads)
        self._depth = [np.zeros(MAX_PIXELS, dtype=np.float32) for _ in range(self.threads)]
        self._flags = [np.zeros(GRID_CAPACITY, dtype=np.uint8) for _ in range(self.threads)]

    def __call__(self, depth, inverse_k, world_camera, origin, voxel, width, height,
                 nx, ny, nz, flags, counts, status):
        width, height = int(width), int(height)
        pixels = width * height
        if pixels == 0:                         # nothing to split: the single core as is
            return self.fn(depth, inverse_k, world_camera, origin, voxel, width, height,
                           nx, ny, nz, flags, counts, status)
        image = np.asarray(depth)[:pixels].reshape(height, width)
        T = self.threads

        def run(i):
            buf = self._depth[i]
            buf[:pixels] = 0.0
            band = buf[:pixels].reshape(height, width)
            band[i::T] = image[i::T]
            c = np.zeros(3, dtype=np.uint32)
            s = np.full(1, 99, dtype=np.uint32)
            self.fn(buf, inverse_k, world_camera, origin, voxel, width, height,
                    nx, ny, nz, self._flags[i], c, s)
            return c, int(s[0])

        outs = list(self._pool.map(run, range(T)))
        codes = set(s for _, s in outs)
        if len(codes) != 1:
            raise RuntimeError("row-interleaved core: threads disagree on status %r" % sorted(codes))
        code = codes.pop()
        counts[0] = counts[1] = counts[2] = 0
        status[0] = code
        if code != 0:
            return
        np.maximum.reduce([f[:self.capacity] for f in self._flags], out=flags[:self.capacity])
        accepted = sum(int(c[0]) for c, _ in outs)
        clipped = sum(int(c[2]) for c, _ in outs)
        counts[0], counts[1], counts[2] = accepted, pixels - accepted - clipped, clipped


def parse_wxh(text):
    """'640x480' -> (640, 480); None -> None."""
    if text is None:
        return None
    w, h = (int(v) for v in str(text).lower().split("x"))
    if w <= 0 or h <= 0:
        raise ValueError("WxH must be positive")
    return w, h


def build_core(a, build_dir):
    """Returns (core_call, core_name, timing_source, describe, live)."""
    if a.core == "null":
        return NullCore(), "null", None, {}, None
    from host.m7_frame_native import FrameNativeM7, compile_frame_native
    lib = compile_frame_native(os.path.join(build_dir, "m7_frame.so"))
    if a.core == "cpu":
        fn = FrameNativeM7(lib).lib.fpga_m7_depth_frame
        if a.cpu_threads > 1:
            return (RowInterleavedCpuCore(fn, a.cpu_threads), "cpu", None,
                    {"cpu_threads": a.cpu_threads, "split": "row_interleaved"}, None)
        return fn, "cpu", None, {}, None
    from host.m7_backend import M7BackendSelector
    from host.m7_pl_device import DevMemPlDevice, MockPlDevice
    from host.m7_frame_fx import compile_frame_fx
    fxlib = compile_frame_fx(os.path.join(build_dir, "m7_frame_fx.so"))   # GEO_FX3 PS constants (C++)
    if a.device == "mock":
        def factory():
            return MockPlDevice(frame_library=lib, frame_fx_library=fxlib, irq=(a.wait == "irq"),
                                delay_s=a.mock_delay_s)
    elif a.device == "devmem":
        factory = DevMemPlDevice.open
    else:
        raise SystemExit("--core pl needs --device devmem|mock")
    sel = M7BackendSelector(pl_device_factory=factory, pl_wait=a.wait, pl_timeout_s=a.core_timeout_s,
                            pl_frame_dlat2=getattr(a, "dlat2", 0), cpu_frame_library=lib,
                            pl_frame_fx_library=fxlib)
    backend = sel.frame_backend("pl")
    desc = {"device": backend.device.describe(), "wait": backend.wait_mode,
            "irq": backend.lib.irq.describe() if backend.lib.irq is not None else None,
            "selector_counts": dict(sel.counts)}
    if getattr(a, "dlat2", 0) or backend.dlat2_state != "untouched":   # default: keys exactly as before
        desc["dlat2"] = backend.dlat2_state
        print(json.dumps({"dlat2": backend.dlat2_state}), file=sys.stderr, flush=True)
    desc["fx"] = backend.fx_state                                      # GEO_FX3: "on" / "absent"
    print(json.dumps({"fx": backend.fx_state}), file=sys.stderr, flush=True)
    # GEO_FX3: the codes entry point is the only PL path (it falls back to the CPU reference itself)
    build_core.codes_call = backend.lib.fpga_m7_depth_frame_codes

    def live():
        return {"backend_counts": dict(backend.counts), "device": backend.device.describe(),
                "irq": backend.lib.irq.describe() if backend.lib.irq is not None else None}
    build_core.device = backend.device
    return (backend.lib.fpga_m7_depth_frame, "pl:%s:%s" % (a.device, backend.wait_mode),
            backend.lib, desc, live)


def rcvbuf_warning(rcvbuf, requested, chunk_bytes):
    """None when the server socket got at least what --rcvbuf asked for, else a dict for one stderr
    line. ``rcvbuf`` = KvLinkServer.rcvbuf (method, effective bytes as read back; the kernel reports
    twice the accounting limit it set), ``requested`` = --rcvbuf. Short only when capped below the
    request: on the board (root, SO_RCVBUFFORCE) and with CAP_NET_ADMIN it never fires."""
    method, effective = rcvbuf
    if effective is None or int(effective) >= int(requested):
        return None
    chunks_640x400 = -(-640 * 400 * 2 // int(chunk_bytes))
    return {"rcvbuf_warning": "receive buffer capped below --rcvbuf", "method": method,
            "effective_bytes": int(effective), "requested_bytes": int(requested),
            "request_640x400_bytes": chunks_640x400 * int(chunk_bytes),
            "effect": "request tail chunks overflow when the receive thread lags -> incomplete "
                      "requests (rx_incomplete_missing_chunks) -> LinkTimeout on the Nano side",
            "fix": "run as root or with CAP_NET_ADMIN (setpriv ambient net_admin, tools/car_run/car_up.sh)"}


def attach_permit_model(a, regs, core_call):
    """--permit-model realtime (host only): put a RealtimePermitModel behind regs["permit"] and
    wrap the core so each returned call raises the completion edge (the PL core's ap_done,
    tests/test_permit_nano_frameid.py EdgeCore). Off: returns (core_call, None) and leaves
    ``regs`` untouched -- the very same objects as before the option existed."""
    if a.permit_model == "off":
        return core_call, None
    from host.kv_regs import PlRegBlock, RealtimePermitModel
    if regs["permit"].available:
        raise SystemExit("--permit-model realtime: a real permit block is present (%s); the "
                         "host model never shadows hardware. Not starting." % regs["permit"].detail)
    clk = int(a.pl_clk_hz or 100000000)
    model = RealtimePermitModel(clk_hz=clk, timeout=clk, hb_half=clk // 200,   # 1000 ms, 5 ms
                                window_min=clk // 10, window_max=clk * 5)       # 100 ms, 5000 ms
    regs["permit"] = PlRegBlock("permit", model, detail={
        "model": "RealtimePermitModel", "host_only": True, "clk_hz": clk,
        "timeout_cycles": model.timeout, "hb_half_cycles": model.hb_half,
        "replaced": regs["permit"].reason})
    inner = core_call

    def core_with_edge(*args):
        out = inner(*args)              # an exception = no completion edge (a failed core)
        model.complete()
        return out
    return core_with_edge, model


def gate_roi_pl(a, regs):
    """--roi-core pl on the board: the UIO + MAGIC-equivalent gate, before any device is
    opened. Refuses to start (SystemExit) on any miss -- never a fallback."""
    if a.roi_core != "pl" or a.device == "mock":
        return None
    from host.kv_roi_pl import RoiUnavailable, open_roi_window
    try:
        return open_roi_window(regs.get("sysinfo"), name=a.roi_uio, sysfs_root=a.uio_sysfs,
                               dev_root=a.uio_dev)
    except RoiUnavailable as exc:
        raise SystemExit("--roi-core pl: %s. Not starting." % exc)


# BUILD_ID -> the xmutil app that carries it (hint text only; the gate compares the number)
# 0x03c087c9 is shared by m7_pl_v3fc2 (pl_clk1 142.86 MHz) and m7_pl_v3fc2_166 (166.66 MHz); tell them apart by the clk1 rate.
# 0x042a646f = m7_pl_v3fc3 (pl_clk1 187.50 MHz, AR on pl_clk0 100 MHz, post-route phys_opt build); hardware source
# 0x08051f22 = m7_pl_m1 / m7_pl_m1_166 (nationals MERGE1, 2026-11-03): no AR core (SLOT_MAP 0x263, car screen on the Nano),
# G3 slim frame core with Dlat2 (dlat2_en 0x13c), rolling core with GRID; the two apps share the id, told apart by clk1.
# 0x07da3d4a = m7_pl_m2 / m7_pl_m2_166 (nationals MERGE2, 2026-11-03): MERGE1 + SLIM_ROI1 slim ROI core
# + WB_BURST 32-bit write-back burst in the frame core (flags buffer must be 4-byte aligned); registers unchanged.
# 0x0b1cfcbc = m7_pl_dpu1 (nationals DPU1, 2026-11-03): MERGE2 + DPUCZDX8G B1024 @300/600 MHz (ar removed,
# DPU regs 0x8F00_0000 on HPM0_LPD, irq bit 7 = DPU); m7 core registers unchanged from MERGE2.
# 0x003e0b9f = m7_pl_m3 (nationals MERGE3, 2026-11-03): DPU1 + fusion-core slimming (SLIM_FUSE2 0ed0693),
#   DPU 300/600 MHz, clk_pl_1 142.86 MHz; hardware source 03e0b9f.
KNOWN_BUILDS = {0x0f35f07f: "m7_pl_m5_166", 0x0a4fc999: "m7_pl_m5a_166", 0x08880589: "m7_pl_m4", 0x003e0b9f: "m7_pl_m3", 0x0b1cfcbc: "m7_pl_dpu1", 0x07da3d4a: "m7_pl_m2", 0x08051f22: "m7_pl_m1", 0x042a646f: "m7_pl_v3fc3", 0x03c087c9: "m7_pl_v3fc2", 0x001256b1: "m7_pl_v3fc", 0x01cec6ef: "m7_pl_v3f", 0x0771aaf2: "m7_pl_v3b", 0x0a9492fc: "m7_pl_v3a", 0x00da5041: "m7_pl_merge_a2", 0x07784f75: "m7_pl_merge", 0x0117bc0a: "m7_pl"}


PERMIT_WINDOW_MS_RANGE = (100.0, 5000.0)       # PL clamp of v3fc2 (m7_bd.tcl)


def clamp_permit_window_ms(ms):
    """(clamped ms, warning or None): owner 10-26 (success rate, no new refusals) -- a value outside
    PERMIT_WINDOW_MS_RANGE is clamped (as the PL does) and warned about, never a start refusal."""
    lo, hi = PERMIT_WINDOW_MS_RANGE
    if lo <= ms <= hi:
        return ms, None
    c = min(max(ms, lo), hi)
    return c, "--permit-window-ms %g outside [%g, %g]: clamped to %g" % (ms, lo, hi, c)


def _permit_window_warn(out, msg):
    out.setdefault("warnings", []).append(msg)
    print("WARNING kv_server: " + msg, file=sys.stderr, flush=True)


def apply_permit_window(a, regs, clk_hz):
    """--permit-window-ms (v3fc2): write the permit window once, read it back. None when the
    option is not given (nothing written). Owner 10-26 (success rate): never stops the start --
    an out-of-range value is clamped and warned; no permit block / unknown PL clock = warning, nothing
    written; a readback that differs (bitstream older than v3fc2: 0x44 read-only, or a PL clamp) =
    warning and the server goes on with whatever window the PL holds (out["readback_ms"])."""
    if a.permit_window_ms is None:
        return None
    ms, why = clamp_permit_window_ms(a.permit_window_ms)
    out = {"requested_ms": a.permit_window_ms, "applied_ms": ms, "written": False}
    if why:
        _permit_window_warn(out, why)
    blk = regs["permit"]
    if not blk.available:
        _permit_window_warn(out, "--permit-window-ms: permit block unavailable (%s); nothing written, "
                                 "continuing" % blk.reason)
        return out
    if not clk_hz:
        _permit_window_warn(out, "--permit-window-ms: PL clock unknown (no sysinfo CLK_HZ, no --pl-clk-hz); "
                                 "nothing written, the bitstream's power-on window holds; continuing")
        return out
    want = int(round(ms * clk_hz / 1000.0))
    before = blk.window.rd(0x44)
    back, lo, hi = blk.write_permit_window(want)
    out.update({"written": True, "clk_hz": int(clk_hz), "written_cycles": want, "before_cycles": before,
                "readback_cycles": back, "readback_ms": back * 1000.0 / clk_hz,
                "min_cycles": lo, "max_cycles": hi})
    if back != want:
        _permit_window_warn(out, "--permit-window-ms %.1f: wrote %d cycles, read back %d (min %d max %d): the "
                                 "loaded bitstream has no writable window (older than v3fc2) or clamped it; "
                                 "continuing with the PL's window (%.1f ms). Load m7_pl_v3fc2 to set it."
                                 % (ms, want, back, lo, hi, out["readback_ms"]))
    return out


def read_build_id(regs):
    """(build_id int or None, reason) from the sysinfo block opened by open_blocks (UIO + MAGIC;
    no other PL address is touched)."""
    blk = regs.get("sysinfo")
    if blk is None or not blk.available:
        return None, "sysinfo block unavailable (%s)" % (getattr(blk, "reason", None) or "not opened")
    try:
        return int(blk.sysinfo()["build_id"]), ""
    except Exception as exc:            # noqa: BLE001  (any read failure = unidentified)
        return None, "sysinfo read failed: %s" % exc


def gate_build_id(a, regs):
    """--expect-build-id (2026-10-16, controller): refuse to start (SystemExit, before any core or
    device is opened) unless the loaded bitstream's sysinfo BUILD_ID is the expected one. The AR
    screen ran the single-lane A1 core (0x07784f75) for a day because nothing checked; the car
    chain's bitstream is A2 m7_pl_merge_a2 0x00da5041 (car_up.sh KV_BUILD_ID). Not given: no
    check (behaviour as before; the base bitstream / host stand-ins have their own callers)."""
    bid, why = read_build_id(regs)
    if a.expect_build_id is None:
        return bid
    want = a.expect_build_id
    app = KNOWN_BUILDS.get(want, "the app with BUILD_ID %#010x" % want)
    fix = "Fix (root): xmutil unloadapp; xmutil loadapp %s; then start again." % app
    if bid is None:
        raise SystemExit("--expect-build-id %#010x: %s, cannot identify the loaded bitstream. %s "
                         "Not starting." % (want, why, fix))
    if bid != want:
        have = KNOWN_BUILDS.get(bid)
        raise SystemExit("--expect-build-id %#010x: loaded bitstream BUILD_ID is %#010x%s, not %#010x (%s). "
                         "%s Not starting." % (want, bid, " (%s)" % have if have else "", want, app, fix))
    return bid


def build_roi_core(a, build_dir, gate=None, device=None):
    """None for --roi-core none; the CPU reference; or the PL core -- never a fallback."""
    if a.roi_core == "none":
        return None
    from host.m7_object_native import compile_object_native
    lib = compile_object_native(os.path.join(build_dir, "m7_object.so"))
    if a.roi_core == "cpu":
        from host.kv_link import CpuRoiCore
        return CpuRoiCore(lib)
    from host.kv_roi_pl import MockRoiWindow, PlRoiCore
    wait = getattr(a, "roi_wait", "poll")
    kw = {} if wait == "poll" else {"irq_slice_s": a.roi_irq_slice_s}   # poll: the call exactly as before
    if a.device == "mock":
        from host.m7_object_native import NativeObjectEvidence
        mutant = None if a.roi_mock_mutant == "none" else a.roi_mock_mutant
        extra = {}
        if getattr(a, "roi_mock_delay_s", 0) or getattr(a, "roi_mock_irq_lost", False):
            extra = {"delay_s": a.roi_mock_delay_s, "irq_lost": a.roi_mock_irq_lost}
        win = MockRoiWindow(device._memory, NativeObjectEvidence(lib).lib, mutant=mutant, **extra)
        if wait == "irq":
            kw.update(wait="irq", irq=win.irq_line())
        return PlRoiCore(win, device, timeout_s=a.roi_timeout_s,
                         detail={"mock": True, "mutant": mutant}, **kw)
    win, detail = gate
    if wait == "irq":
        from host.m7_pl_device import PlDeviceUnavailable, UioIrq
        try:
            kw.update(wait="irq", irq=UioIrq.open(a.roi_uio, a.uio_sysfs, a.uio_dev))
        except (PlDeviceUnavailable, OSError) as exc:   # success rate first: the poll of before, warned
            print("WARNING --roi-wait irq: %s; the ROI core is polled as before" % exc, file=sys.stderr, flush=True)
            kw = {}
            detail = dict(detail, roi_wait_irq_failed=str(exc))
    return PlRoiCore(win, device, timeout_s=a.roi_timeout_s, detail=detail, **kw)


def build_kv_fast(a, build_dir):
    """--kv-fast native -> host.kv_fast_native.KvFastNative built from the deployed source; off -> None.
    A build or load failure warns on stderr and returns None (the Python path; owner 10-26: no new start
    refusals). ``a.kv_fast_status``: "off", "native" or "failed: <why>"."""
    from host.kv_fast_native import KvFastNative, default_mode
    mode = a.kv_fast if a.kv_fast is not None else default_mode()
    a.kv_fast_status = mode
    if mode != "native":
        return None
    try:
        return KvFastNative.build(build_dir)
    except Exception as exc:            # noqa: BLE001 -- any failure: warn, serve the Python path
        a.kv_fast_status = "failed: %s: %s" % (type(exc).__name__, str(exc)[:200])
        print(json.dumps({"kv_fast_warning": a.kv_fast_status}), file=sys.stderr, flush=True)
        return None


def gate_rolling_pl(a, regs):
    """--rolling pl on the board: slot-5 gate (host/kv_rolling.open_rolling_window)."""
    if a.rolling != "pl" or a.device == "mock":
        return None
    from host.kv_rolling import RollingUnavailable, open_rolling_window
    try:
        return open_rolling_window(regs.get("sysinfo"), name=a.rolling_uio, sysfs_root=a.uio_sysfs,
                                   dev_root=a.uio_dev)
    except RollingUnavailable as exc:
        raise SystemExit("--rolling pl: %s. Not starting." % exc)


def build_rolling(a, build_dir, gate=None, device=None, timing=None):
    """None for --rolling none; the A53 core; or the PL core (zero copy) -- never a fallback."""
    if a.rolling == "none":
        return None
    from host.kv_rolling import (CpuRollingCore, MockRollingWindow, PlRollingCore, RollingService,
                                 kv_config_words)
    from host.m7_rolling import compile_rolling
    if a.rolling == "cpu":
        grid_so = (os.path.join(build_dir, "b1_grid_kv.so")
                   if getattr(a, "rolling_grid", "none") == "cmodel" else None)
        return RollingService(CpuRollingCore(compile_rolling(os.path.join(build_dir, "roll_kv.so")),
                                             grid_library=grid_so))
    frame_buffer = (lambda: timing._buffers.get("flags")) if timing is not None else None
    gkw = {"grid": "probe"} if getattr(a, "rolling_grid", "none") == "pl" else {}   # none: as before
    if a.device == "mock":
        mutant = None if a.rolling_mock_mutant == "none" else a.rolling_mock_mutant
        mkw = {"grid": True} if gkw else {}   # the mock is the GRID build (= next bitstream) only then
        win = MockRollingWindow(device._memory, compile_rolling(os.path.join(build_dir, "roll_mock.so"), **mkw),
                                mutant=mutant, **mkw)
        core = PlRollingCore(win, device, kv_config_words(build_dir), timeout_s=a.rolling_timeout_s,
                             detail={"mock": True, "mutant": mutant}, **gkw)
    else:
        win, detail = gate
        core = PlRollingCore(win, device, kv_config_words(build_dir), timeout_s=a.rolling_timeout_s,
                             detail=detail, **gkw)
    if gkw:
        print(json.dumps({"rolling_grid": core.grid_probe}), file=sys.stderr, flush=True)
    return RollingService(core, frame_buffer=frame_buffer)


def start_ar(a, stale_ms):
    """--ar drm: desktop off (board), then the presenter; refuses (SystemExit) if the helper
    does not come up -- never a silent run without the display."""
    import shlex
    import subprocess
    from host.kv_ar import ArPresenter, ArUnavailable, helper_path
    mock = a.device == "mock" or a.ar_mock
    helper = a.ar_helper or helper_path(ROOT, "host" if mock else None)
    cmd = [helper, "--stale-ms", str(stale_ms), "--refresh-ms", str(a.ar_refresh_ms)]
    if mock:
        cmd += ["--provider", "mock", "--display", "none", "--mock-size", a.ar_mock_size,
                "--mock-permit", str(a.ar_mock_permit)]
    else:
        cmd += ["--provider", "dmaheap", "--heap", a.ar_heap, "--display", "drm",
                "--wait", a.ar_wait, "--uio-sysfs", a.uio_sysfs, "--uio-dev", a.uio_dev]
        if a.ar_mode:
            cmd += ["--mode", a.ar_mode]
        if a.ar_modeline:
            cmd += ["--modeline", a.ar_modeline]
    if a.out:                   # the helper's own copy of its events (survives a killed kv_server)
        cmd += ["--event-log", os.path.join(a.out, "ar_helper_events.jsonl")]
    if a.ar_colour_mutant != "none":
        cmd += ["--colour-mutant", a.ar_colour_mutant]
    if a.ar_mini_scale != 1:    # 1: the helper command line as before
        cmd += ["--mini-scale", str(a.ar_mini_scale)]
    if a.ar_dump_dir:
        os.makedirs(a.ar_dump_dir, exist_ok=True)
        cmd += ["--dump-dir", a.ar_dump_dir, "--dump-seq", a.ar_dump_seq or "",
                "--dump-failsafe", str(a.ar_dump_failsafe)]
    desktop = None
    if not mock and not a.no_ar_desktop:
        def desktop(verb):
            r = subprocess.run(["xmutil", verb], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            return {"cmd": "xmutil " + verb, "rc": r.returncode,
                    "out": r.stdout.decode("utf-8", "replace")[-400:]}
        res = desktop("desktop_disable")
        print(json.dumps({"ar_desktop": res}), file=sys.stderr, flush=True)
    colour = None
    if a.ar_colour == "main" or a.ar_colour_port >= 0:
        from host.kv_colour import ColourSink
        colour = ColourSink((a.bind, max(0, a.ar_colour_port)), rcvbuf=a.rcvbuf,
                            tol_ns=int(a.ar_colour_tol_ms * 1e6)).start()
        print(json.dumps({"ar_colour_port": colour.port, "rcvbuf": colour.rcvbuf}), file=sys.stderr, flush=True)
    fill_native = None
    if a.ar_fill == "native":           # compiled from the deployed source now; no build -> no start
        from host.kv_ar_native import ArFillNative
        try:
            fill_native = ArFillNative.build(tempfile.mkdtemp(prefix="kv_ar_fill_"))
        except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
            if colour is not None:
                colour.stop()
            if desktop:
                desktop("desktop_enable")
            raise SystemExit("--ar-fill native: the depth fill library did not build (%s). Not starting." % exc)
        print(json.dumps({"ar_fill": fill_native.describe()}), file=sys.stderr, flush=True)
    try:
        ar = ArPresenter(cmd, source=a.ar_source, max_age_ms=a.ar_max_age_ms,
                         permit_present=1, out_dir=a.out, tune_file=a.ar_tune_file,
                         box_max=a.ar_box_max, box_nms=None if a.ar_box_nms < 0 else a.ar_box_nms,
                         depth_stretch=a.ar_depth_stretch, colour=colour,
                         colour_main=a.ar_colour == "main", colour_wait_ms=a.ar_colour_wait_ms,
                         colour_hold_ms=a.ar_colour_hold_ms,
                         depth_upsample_to=parse_wxh(a.ar_depth_upsample_to), fill_native=fill_native,
                         nodepth_after_ms=a.ar_nodepth_after_ms, nodepth_period_ms=a.ar_nodepth_period_ms,
                         nodepth_tol_ms=a.ar_nodepth_tol_ms, nodepth_lead_ms=a.ar_nodepth_lead_ms,
                         nodepth_max_s=a.ar_nodepth_max_s, colour_refresh_ms=a.ar_colour_refresh_ms,
                         mini_scale=a.ar_mini_scale)
    except ArUnavailable as exc:
        if colour is not None:
            colour.stop()
        if desktop:
            desktop("desktop_enable")
        raise SystemExit("--ar drm: %s (cmd %s). Not starting." % (exc, shlex.join(cmd)
                                                                     if hasattr(shlex, "join") else cmd))
    print(json.dumps({"ar_ready": ar.ready}), file=sys.stderr, flush=True)
    return ar, desktop


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--core", choices=("pl", "cpu", "null"), required=True)
    ap.add_argument("--cpu-threads", type=int, default=1,
                    help="--core cpu only (host stand-in): run the CPU core on this many threads, "
                         "rows interleaved, merged exactly (RowInterleavedCpuCore). Default 1 = as before")
    ap.add_argument("--device", choices=("devmem", "mock"), default=None)
    ap.add_argument("--wait", choices=("irq", "poll"), default=None)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=0)
    ap.add_argument("--chunk-bytes", type=int, default=CHUNK_FRAGMENTED)
    ap.add_argument("--rcvbuf", type=int, default=8 << 20)
    ap.add_argument("--out", default=None)
    ap.add_argument("--max-idle-s", type=float, default=900.0)
    ap.add_argument("--mock-delay-s", type=float, default=0.0)
    ap.add_argument("--core-timeout-s", type=float, default=2.0)
    ap.add_argument("--pl-timebase", choices=("permit", "encoder", "none"), default="permit")
    ap.add_argument("--pl-clk-hz", type=int, default=None)
    ap.add_argument("--roi-core", choices=("pl", "cpu", "none"), default="none")
    ap.add_argument("--roi-uio", default="m7_roi",
                    help="UIO node name of the PL ROI IP (m7_pl_merge overlay: m7_roi)")
    ap.add_argument("--roi-timeout-s", type=float, default=0.5)
    ap.add_argument("--roi-mock-mutant", choices=("none", "row_shift", "threshold"),
                    default="none", help="host mock only: positive-control mutants")
    ap.add_argument("--rolling", choices=("none", "cpu", "pl"), default="none")
    ap.add_argument("--rolling-uio", default="m7_rolling")
    ap.add_argument("--rolling-grid", choices=("none", "cmodel", "pl"), default="none")
    ap.add_argument("--nav", choices=("none", "soft", "pl"), default="none")
    ap.add_argument("--nav-core", choices=("kv_nav", "mock"), default="kv_nav")
    ap.add_argument("--nav-swept-clear-s", type=float, default=None,
                    help="CTRLPS4: KV-side swept clearing of the navigation window, seconds (default 300 = the Nano "
                         "bridge's SWEPT_CLEAR_S; 0 = off). Only with --nav soft|pl --nav-core kv_nav")
    ap.add_argument("--nav-front-seed-m", type=float, default=None,
                    help="CTRLPS4: KV-side front-sector seed radius, metres (default off = the Nano's CAR_FRONT_SEED_M "
                         "empty; runbook value 0.65). Only with --nav soft|pl --nav-core kv_nav")
    ap.add_argument("--nav-odom-hold-s", type=float, default=None,
                    help="CTRLPS6: no NAV_ODOM for this long while a goal runs -> the core holds (PAUSED, no stuck "
                         "time) and replans when odometry is back (default 1.0 = the Nano's link-lost time; 0 = off). "
                         "Only with --nav soft|pl")
    ap.add_argument("--nav-sim-world", default=None,
                    help="CTRLPS3 host tests only: costmap window from a known world JSON (host/kv_nav.SimWorldGrid)")
    ap.add_argument("--dlat2", type=int, choices=(0, 1), default=0)
    ap.add_argument("--fx-stats-log-every", type=int, default=0,
                    help="GEO_FX3: print the PL depth statistics registers to stderr every N PL frames (0 = never)")
    ap.add_argument("--roi-concurrent", type=int, choices=(0, 1), default=0)
    ap.add_argument("--roi-wait", choices=("poll", "irq"), default="poll")
    ap.add_argument("--roi-irq-slice-s", type=float, default=0.01)
    ap.add_argument("--roi-mock-delay-s", type=float, default=0.0, help="host mock only: the ROI core finishes this late")
    ap.add_argument("--roi-mock-irq-lost", action="store_true", help="host mock only: the ROI line never fires")
    ap.add_argument("--rolling-timeout-s", type=float, default=0.2)
    ap.add_argument("--rolling-mock-mutant", choices=("none", "drop_last_delta"), default="none")
    ap.add_argument("--no-permit-feed", action="store_true")
    ap.add_argument("--permit-id-source", choices=("nano", "kv-counter"), default="nano",
                    help="NEXT_FRAME_ID source (default nano; kv-counter = positive control)")
    ap.add_argument("--permit-model", choices=("off", "realtime"), default="off",
                    help="HOST ONLY: realtime = permit block modelled in software (see above)")
    ap.add_argument("--permit-window-ms", type=float, default=None,
                    help="v3fc2: PL permit freshness window written once at start, [100, 5000] ms (outside: clamped, warned); "
                         "not given = the bitstream's power-on window (1000 ms)")
    ap.add_argument("--expect-build-id", type=lambda v: int(v, 0), default=None,
                    help="refuse to start unless the loaded bitstream's sysinfo BUILD_ID is this (e.g. "
                         "0x00da5041 = m7_pl_merge_a2, the car chain's; ar_screen.sh start passes it). "
                         "Default: no check (as before)")
    ap.add_argument("--send-timeout-s", type=float, default=None,
                    help="how long one sendto may block on a full send buffer before that answer is "
                         "dropped (counted send_timeout); default host.kv_link.SERVER_SEND_TIMEOUT_S")
    ap.add_argument("--uio-sysfs", default="/sys/class/uio")
    ap.add_argument("--uio-dev", default="/dev", help="where the uioN device nodes live (tests)")
    ap.add_argument("--nudge", choices=("on", "burst", "idle", "off"), default="on",
                    help="tail release (docs/design/KV_LINK_BURST_TAIL_20261011.md): NUDGE after every "
                         "multi-frame answer (burst), after --nudge-idle-ms without sends (idle), both "
                         "(on, default) or none (off = the wire behaviour before the fix)")
    ap.add_argument("--nudge-idle-ms", type=float, default=None,
                    help="idle NUDGE delay (default host.kv_link.NUDGE_IDLE_S)")
    ap.add_argument("--send-log", default=None,
                    help="jsonl: one line per multi-frame answer (serve stages, sendto stamps; "
                         "KV monotonic). Local file only")
    ap.add_argument("--send-log-iface", default=None,
                    help="with --send-log: also read this NIC's sysfs tx_packets around each such send")
    ap.add_argument("--kv-fast", choices=("off", "native"), default=None,
                    help="E7 (2026-11-02): native = the codes crc32 and the codes -> float32 conversion + depth "
                         "crc32 in C++ (host/kv_fast_native.py, tools/kv_link/kvfast/kv_fast.cpp built at start; "
                         "same bytes as the Python path); off = exactly as before. Default: $FPGA_KV_FAST, else off. "
                         "A failed build only warns and serves the Python path (never a start refusal)")
    ap.add_argument("--ar", choices=("off", "drm"), default="off")
    ap.add_argument("--ar-source", choices=("live", "replay"), default="live")
    ap.add_argument("--ar-helper", default=None, help="m7ar_present binary (default: build/<arch>)")
    ap.add_argument("--ar-wait", choices=("irq", "poll"), default="irq")
    ap.add_argument("--ar-heap", default="reserved")
    ap.add_argument("--ar-max-age-ms", type=int, default=200)
    ap.add_argument("--ar-stale-ms", type=float, default=None,
                    help="fail-safe after this long without a new frame (default max-age + 1)")
    ap.add_argument("--ar-refresh-ms", type=float, default=500.0)
    ap.add_argument("--ar-dump-dir", default=None)
    ap.add_argument("--ar-dump-seq", default=None, help="helper render numbers to dump, e.g. 50,150,250")
    ap.add_argument("--ar-dump-failsafe", type=int, default=0)
    ap.add_argument("--ar-mock-size", default="1024x768")
    ap.add_argument("--ar-mock-permit", type=int, choices=(0, 1), default=0)
    ap.add_argument("--no-ar-desktop", action="store_true")
    ap.add_argument("--ar-mode", default=None, help="screen bring-up: DRM mode WxH from the connector list")
    ap.add_argument("--ar-modeline", default=None,
                    help="screen bring-up: custom mode 'clock_khz hd hss hse ht vd vss vse vt -hsync +vsync'")
    ap.add_argument("--ar-box-max", type=int, default=5,
                    help="display only: at most this many boxes on the AR screen (owner 10-15: 5; 32 = all, "
                         "the old behaviour); the ROI core still checks every row")
    ap.add_argument("--ar-box-nms", type=float, default=0.5,
                    help="display only: same-class 2D IoU above this hides the later box (O11 order); "
                         "negative = off (old behaviour)")
    ap.add_argument("--ar-depth-upsample-to", default=None, metavar="WxH",
                    help="display only (2026-10-16): a frame whose depth is WxH decimated by an integer s (Nano "
                         "producer --kv-depth-stride s) is shown at WxH (codes repeated s x s, ROI boxes scaled), so "
                         "the colour main view pairs and the tuned layout holds; other sizes as they come. "
                         "Default off (old behaviour)")
    ap.add_argument("--ar-depth-stretch", type=float, default=1.0,
                    help="display only: colour stretch k, depth f(z)=zmin+(z-zmin)*k (nearest-Z reading kept); "
                         "1 = off (old behaviour)")
    ap.add_argument("--ar-fill", choices=("native", "python"), default="native",
                    help="AR sender depth chain (codes -> mm -> stretch): native = m7ar_fill.cpp compiled from "
                         "the deployed source at start (refuses to start when it does not build; 2026-10-16), "
                         "python = the numpy reference (before 2026-10-16). Same bytes either way")
    ap.add_argument("--ar-tune-file", default=None,
                    help="screen bring-up: JSON overrides (depth_dx/dy, mini_x/y, slice_y, max_age_ms), re-read on change")
    ap.add_argument("--ar-colour", choices=("off", "main"), default="off",
                    help="AR main view: off (default, depth as before) | main (colour, PL overlay on top, "
                         "depth as PiP); tune.json colour_main 0/1 switches live when the bypass is open")
    ap.add_argument("--ar-colour-port", type=int, default=-1,
                    help="colour bypass UDP port (0 = any, printed in the ready line); -1 = closed unless "
                         "--ar-colour main (then 0)")
    ap.add_argument("--ar-colour-wait-ms", type=float, default=40.0,
                    help="how long the AR sender waits for the paired colour frame before showing depth")
    ap.add_argument("--ar-colour-hold-ms", type=float, default=1000.0,
                    help="owner 10-16: an unpaired frame shows the last paired colour while it is at most "
                         "this old (0 = never; then the depth picture as before); tune colour_hold_ms")
    ap.add_argument("--ar-colour-tol-ms", type=float, default=10.0,
                    help="colour/depth sensor stamp pairing tolerance")
    ap.add_argument("--ar-nodepth-after-ms", type=float, default=150.0,
                    help="no-depth self pull (AR_DEPTH_INVALID_STALL_20261017): this long after the last real "
                         "frame the AR sender pulls the newest colour itself and shows it as NODATA (no geometry); "
                         "0 = off (as before); tune nodepth_after_ms")
    ap.add_argument("--ar-nodepth-period-ms", type=float, default=100.0, help="pull period while no real frame")
    ap.add_argument("--ar-nodepth-tol-ms", type=float, default=67.0, help="WANT tolerance of a pull")
    ap.add_argument("--ar-nodepth-lead-ms", type=float, default=33.0,
                    help="a pull asks for the colour acquired this long before now (sensor clock estimate)")
    ap.add_argument("--ar-nodepth-max-s", type=float, default=60.0,
                    help="stop pulling after this long without a real frame (then STALE, as before)")
    ap.add_argument("--ar-colour-refresh-ms", type=float, default=0.0,
                    help="colour refresh (AR_LATENCY_PLAN_20261027 plan C): between geometry frames re-send the last "
                         "geometry picture (boxes, depth inset, panel) with the newest colour this often; 0 = off; "
                         "tune colour_refresh_ms (the competition tune sets it)")
    ap.add_argument("--ar-mini-scale", type=int, choices=(1, 2), default=2,
                    help="owner 10-29: the screen mini map in the colour main view enlarged this many times "
                         "(helper --mini-scale; the PL still draws the 128 px version); 1 = as before. "
                         "The depth main view (colour_main 0) always shows it at 1x")
    ap.add_argument("--ar-colour-mutant", choices=("none", "mask_shift", "pip_row", "mini_row"), default="none",
                    help="tests only: helper composition mutants (positive controls of ar_dump_check)")
    ap.add_argument("--ar-mock", action="store_true",
                    help="presenter with its mock provider and no display, with any --core (tests)")
    ap.add_argument("--frame-source", choices=("udp", "kvcam"), default="udp",
                    help="NN-CAM2: udp = frames from the Nano (default, as before); kvcam = kv_cam's ring on this board")
    ap.add_argument("--kvcam-name", default="kv_cam_v1", help="kv_cam ring name (/dev/shm/<name>)")
    ap.add_argument("--kvcam-path", default=None, help="kv_cam ring as a file / udmabuf (kv_cam --path)")
    ap.add_argument("--local-params", default=None, help="JSON with the bench camera pose / intrinsics / grid")
    ap.add_argument("--stamp-log", default=None, help="NN-CAM2 per-frame chain stamps (JSONL)")
    ap.add_argument("--dump-core-input", default=None, help="tests: directory for every frame's geometry-core input")
    # CHASSIS0 (2026-11-04, runs/nationals-prelim-20261102/CHASSIS0): the chassis serial on the KV260
    ap.add_argument("--chassis", choices=("off", "kv", "nano"), default="off",
                    help="off (default; the bench: no chassis port is opened, exactly as before) | kv: this process "
                         "opens --chassis-port (host/kv_chassis_bridge.py: kv_nav (v, w) down, wheel/gyro odometry "
                         "up as the pose of kv_nav and the map) | nano: the Nano's car stack owns the chassis "
                         "(nothing opened here; only noted)")
    ap.add_argument("--chassis-port", default="/dev/fpga_chassis", help="CHASSIS0: the chassis USB serial on the KV")
    ap.add_argument("--chassis-odom-args", default="--heading gyro --wheel-diam-m 0.081 --counts-per-rev 1560 --track-m 0.394",
                    help="CHASSIS0: host/ros_wheel_odom.py arguments for the odometry core (as CAR_WHEEL_ODOM_ARGS; "
                         "default = car_up's board defaults: --heading gyro, real-car wheels 0.081 m / 1560 / 0.394 m, D-041)")
    ap.add_argument("--chassis-record", default=None, help="CHASSIS0: append every received frame (JSONL)")
    ap.add_argument("--chassis-v-limit-mm-s", type=int, default=500,
                    help="CHASSIS0: |v| clip of every command frame (500 = the Nano driver / MCU limit; first on-car "
                         "tests: 150)")
    ap.add_argument("--chassis-w-limit-mrad-s", type=int, default=1500, help="CHASSIS0: |w| clip (first tests: 500)")
    a = ap.parse_args()
    if a.core == "pl" and a.wait is None:
        ap.error("--core pl needs an explicit --wait irq|poll")
    if a.core != "pl" and (a.wait or a.device):
        ap.error("--wait/--device only apply to --core pl")
    if a.cpu_threads < 1 or (a.cpu_threads > 1 and a.core != "cpu"):
        ap.error("--cpu-threads is >= 1 and only applies to --core cpu")
    if a.roi_core == "pl" and a.core != "pl":
        ap.error("--roi-core pl needs --core pl (the ROI depth copy lives in its DMA provider)")
    if a.roi_mock_mutant != "none" and not (a.roi_core == "pl" and a.device == "mock"):
        ap.error("--roi-mock-mutant only with --roi-core pl --device mock")
    if a.rolling == "pl" and a.core != "pl":
        ap.error("--rolling pl needs --core pl (zero copy reads the frame core's DMA buffer)")
    if a.rolling_grid == "pl" and a.rolling != "pl":
        ap.error("--rolling-grid pl needs --rolling pl (the fused core's op GRID)")
    if a.dlat2 and a.core != "pl":
        ap.error("--dlat2 1 needs --core pl (a register of the PL frame core)")
    if a.roi_concurrent and a.roi_core == "none":
        ap.error("--roi-concurrent 1 needs a ROI core")
    if a.roi_wait == "irq" and a.roi_core != "pl":
        ap.error("--roi-wait irq needs --roi-core pl")
    if (a.roi_mock_delay_s or a.roi_mock_irq_lost) and not (a.roi_core == "pl" and a.device == "mock"):
        ap.error("--roi-mock-delay-s / --roi-mock-irq-lost only with --roi-core pl --device mock")
    if a.rolling_mock_mutant != "none" and not (a.rolling == "pl" and a.device == "mock"):
        ap.error("--rolling-mock-mutant only with --rolling pl --device mock")
    if a.nav == "none" and a.nav_core != "kv_nav":
        ap.error("--nav-core only with --nav soft|pl")
    if (a.nav_swept_clear_s is not None or a.nav_front_seed_m is not None) and (a.nav == "none" or a.nav_core != "kv_nav"):
        ap.error("--nav-swept-clear-s / --nav-front-seed-m only with --nav soft|pl --nav-core kv_nav")
    if a.nav_odom_hold_s is not None and a.nav == "none":
        ap.error("--nav-odom-hold-s only with --nav soft|pl")
    if a.nav_sim_world and (a.nav == "none" or a.nav_core != "kv_nav"):
        ap.error("--nav-sim-world only with --nav soft --nav-core kv_nav")
    if a.permit_model != "off" and a.device == "devmem":
        ap.error("--permit-model is host only (not with --device devmem)")
    if a.ar == "off" and (a.ar_dump_dir or a.ar_dump_seq):
        ap.error("--ar-dump-* need --ar drm")
    if a.ar == "off" and (a.ar_colour != "off" or a.ar_colour_port >= 0):
        ap.error("--ar-colour* need --ar drm")
    stale_ms = a.ar_stale_ms if a.ar_stale_ms is not None else a.ar_max_age_ms + 1.0
    if a.ar == "drm" and stale_ms <= a.ar_max_age_ms:
        ap.error("--ar-stale-ms must exceed --ar-max-age-ms (the fail-safe frame is the STALE mode)")
    if a.out:
        os.makedirs(a.out, exist_ok=True)
    from host.kv_clock_sync import PlTimebase, SyncResponder
    from host.kv_regs import PERMIT_FEEDS, open_blocks
    regs = open_blocks(sysfs_root=a.uio_sysfs, dev_root=a.uio_dev)        # each block once, shared by every user
    build_id = gate_build_id(a, regs)                 # refuses before any device is opened
    roi_gate = gate_roi_pl(a, regs)                   # refuses before any device is opened
    rolling_gate = gate_rolling_pl(a, regs)           # refuses before any device is opened
    build = tempfile.mkdtemp(prefix="kv_server_")
    build_core.codes_call = None
    core_call, core_name, timing, desc, live = build_core(a, build)
    codes_call = build_core.codes_call
    core_call, permit_model = attach_permit_model(a, regs, core_call)
    if codes_call is not None and permit_model is not None:      # D16: same completion edge
        inner_codes = codes_call

        def codes_call(*args):
            out = inner_codes(*args)
            permit_model.complete()
            return out
    if codes_call is not None and a.fx_stats_log_every > 0 and timing is not None:   # GEO_FX3 statistics log
        inner_log, n_log = codes_call, [0]

        def codes_call(*args):
            out = inner_log(*args)
            n_log[0] += 1
            if n_log[0] % a.fx_stats_log_every == 0:
                print(json.dumps({"fx_stats": getattr(timing, "last_stats", None),
                                  "fx": getattr(timing, "last_fx", None), "fx_calls": n_log[0]}),
                      file=sys.stderr, flush=True)
            return out
    info = {"core": core_name, "core_detail": desc, "pid": os.getpid(),
            "python": sys.version.split()[0], "numpy": np.__version__,
            "argv": sys.argv[1:], "started_unix": time.time()}
    if build_id is not None:            # no sysinfo block (host stand-ins): info keys as before
        info["build_id"] = "%#010x" % build_id
        info["expect_build_id"] = (None if a.expect_build_id is None else "%#010x" % a.expect_build_id)
    roi_core = build_roi_core(a, build, roi_gate, getattr(build_core, "device", None))
    info["roi_core"] = roi_core.describe() if hasattr(roi_core, "describe") else (
        roi_core.name if roi_core is not None else "none")
    rolling = build_rolling(a, build, rolling_gate, getattr(build_core, "device", None), timing)
    info["rolling"] = rolling.describe() if rolling is not None else "none"
    ar, desktop = None, None
    if a.ar == "drm":
        ar, desktop = start_ar(a, stale_ms)
    info["ar"] = ar.describe() if ar is not None else "off"
    feed = None
    if ((a.core == "pl" or permit_model is not None) and not a.no_permit_feed
            and regs["permit"].available):
        feed = PERMIT_FEEDS[a.permit_id_source](regs["permit"])
    info["permit_feed"] = feed.describe() if feed is not None else None
    if permit_model is not None:        # off: info keys exactly as before
        info["permit_model"] = "realtime"
    info["registers"] = {k: v.describe() for k, v in regs.items()}
    clk = a.pl_clk_hz
    if clk is None and regs["sysinfo"].available:
        clk = regs["sysinfo"].sysinfo()["clk_hz"] or None
    window = apply_permit_window(a, regs, clk if clk else (
        int(a.pl_clk_hz or 100000000) if permit_model is not None else None))
    if window is not None:              # not given: info keys exactly as before
        info["permit_window"] = window
        print(json.dumps({"permit_window": window}), file=sys.stderr, flush=True)
    pl = None if a.pl_timebase == "none" else PlTimebase.from_block(regs[a.pl_timebase], clk_hz=clk)
    responder = SyncResponder(pl=pl)
    info["pl_timebase"] = pl.describe() if pl is not None else {"available": False,
                                                                "reason": responder.pl_reason()}
    print(json.dumps({"pl_timebase": info["pl_timebase"], "registers": info["registers"]}),
          file=sys.stderr, flush=True)
    fast = build_kv_fast(a, build)
    if fast is not None:
        info["kv_fast"] = fast.describe()
    elif a.kv_fast_status != "off":
        info["kv_fast"] = {"impl": "python", "requested": "native", "reason": a.kv_fast_status}
    send_log, log_fp = None, None
    if a.send_log:
        log_fp = open(a.send_log, "w")
        log_lock = threading.Lock()

        def send_log(rec):
            with log_lock:
                log_fp.write(json.dumps(rec, sort_keys=True) + "\n")
                log_fp.flush()
    stamps = None
    chain_holder = [None]
    if a.frame_source == "kvcam" or a.stamp_log:   # NN-CAM2 (off: nothing created, exactly as before)
        from host.kv_local_frames import StampLog
        stamps = StampLog(a.stamp_log)
    nav = None
    if a.nav != "none":   # CTRLPS2: none = no NavService, no info key, exactly as before
        from host.kv_nav_service import NavService, build_core as build_nav_core
        gp = {"swept_clear_s": a.nav_swept_clear_s, "front_seed_m": a.nav_front_seed_m}
        nav_core = build_nav_core(a.nav, a.nav_core, rolling=rolling, grid_params=gp,
                                  **({"sim_world": a.nav_sim_world} if a.nav_sim_world else {}))
        if a.frame_source == "kvcam":   # NN-CAM2: note the first tick that used each local frame's grid
            from host.kv_local_frames import StampedNavCore
            nav_core = StampedNavCore(nav_core, lambda: chain_holder[0], stamps)
        nav = NavService(nav_core,
                         **({"odom_hold_s": a.nav_odom_hold_s} if a.nav_odom_hold_s is not None else {}))
        info["nav"] = {"field": a.nav, "core": a.nav_core}
        if a.nav_sim_world:
            info["nav"]["sim_world"] = a.nav_sim_world
    server = KvLinkServer(core_call, core_name, bind=(a.bind, a.port), chunk_bytes=a.chunk_bytes,
                          rcvbuf=a.rcvbuf, info=info, timing_source=timing, live_info=live,
                          sync_responder=responder, roi_core=roi_core, regs=regs,
                          geometry_desc={"core": a.core, "wait": a.wait},
                          send_log=send_log, send_log_iface=a.send_log_iface,
                          nudge_after_burst=a.nudge in ("on", "burst"),
                          nudge_idle_s=(0.0 if a.nudge in ("off", "burst") else
                                        NUDGE_IDLE_S if a.nudge_idle_ms is None else a.nudge_idle_ms / 1e3),
                          permit_feed=feed, ar=ar, rolling=rolling, fast=fast,
                          **({"nav": nav} if nav is not None else {}),
                          **({"roi_concurrent": True} if a.roi_concurrent else {}),
                          **({"core_call_codes": codes_call} if codes_call is not None else {}),
                          **({} if a.send_timeout_s is None else {"send_timeout_s": a.send_timeout_s}))
    if a.dump_core_input:
        os.makedirs(a.dump_core_input, exist_ok=True)
        server.dump_core_input = a.dump_core_input
    chassis = None
    if a.chassis == "kv":   # CHASSIS0: never refuses -- a missing port is a warning and the static pose
        try:
            import shlex
            from host.kv_chassis_bridge import KvChassisBridge
            chassis = KvChassisBridge(a.chassis_port, nav=nav, odom_args=shlex.split(a.chassis_odom_args),
                                      record=a.chassis_record, v_limit_mm_s=a.chassis_v_limit_mm_s,
                                      w_limit_mrad_s=a.chassis_w_limit_mrad_s)
        except Exception as exc:
            print(json.dumps({"warning": "kv_chassis: bridge not started (%r); pose stays static" % (exc,)}),
                  file=sys.stderr, flush=True)
    if a.chassis != "off":
        info["chassis"] = {"mode": a.chassis, "port": a.chassis_port if a.chassis == "kv" else None,
                           "bridge": chassis is not None}
    local_src = None
    if a.frame_source == "kvcam":
        from host.kv_local_frames import LocalDepthChain, LocalDepthSource
        params_cfg = None
        if a.local_params:
            with open(a.local_params) as fp:
                params_cfg = json.load(fp)
        local_src = LocalDepthSource(server, ring_name=a.kvcam_name, ring_path=a.kvcam_path, params_cfg=params_cfg,
                                     stamps=stamps,
                                     **({"pose_fn": chassis.last_pose} if chassis is not None else {}))
        chain_holder[0] = LocalDepthChain(server, local_src, stamps, nav=nav,
                                          **({"chassis": chassis} if chassis is not None else {}))
        server.local_sink = chain_holder[0]
        info["frame_source"] = "kvcam"
    server.start()
    if chassis is not None:
        chassis.start()
    if local_src is not None:
        local_src.start()
    # The handler only stores into a list: it runs on the main thread between bytecodes, and if that
    # lands inside threading.Event.wait() (which holds the Event's non-reentrant lock), Event.set()
    # in the handler deadlocks the process for good (SIGTERM never exits; seen ~4 % of immediate
    # SIGTERMs on 3.8, ~0.5 % on 3.13). time.sleep holds no lock; the signal cuts it short only if
    # the handler raises, so stop latency is one poll period. Installed before the ready line so a
    # caller that sends SIGTERM right after reading it still gets a clean exit and summary.json.
    stop_signal = []
    reason = ["?"]

    def on_signal(signum, _frame):
        stop_signal.append(signum)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    warn = rcvbuf_warning(server.rcvbuf, a.rcvbuf, a.chunk_bytes)
    if warn is not None:
        print(json.dumps(warn), file=sys.stderr, flush=True)
    ready = {"port": server.port, "pid": os.getpid(), "core": core_name, "rcvbuf": server.rcvbuf}
    if ar is not None and ar.colour is not None:     # off: the ready line as before
        ready["ar_colour_port"] = ar.colour.port
    if build_id is not None:            # the board: which bitstream this server is serving on
        ready["build_id"] = "%#010x" % build_id
    print(json.dumps(ready), flush=True)
    while True:
        time.sleep(0.1)
        if stop_signal:
            reason[0] = "signal %d" % stop_signal[0]
            break
        if server.worker_error:
            reason[0] = "worker error"
            break
        if time.monotonic() - server.last_activity > a.max_idle_s:
            reason[0] = "idle %.0f s" % a.max_idle_s
            break
    stats = server.window_stats()
    stats["total_counts"] = dict(server.counts)
    stats["exit_reason"] = reason[0]
    stats["clock_sync_server"] = {"counts": dict(responder.counts),
                                  "pl_timebase": pl.describe() if pl is not None else None}
    stats["estop"] = {"state": server.estop.snapshot(), "counts": dict(server.estop.counts)}
    stats["registers_end"] = {k: v.describe() for k, v in regs.items()}
    stats["permit_feed"] = feed.describe() if feed is not None else None
    if permit_model is not None:
        stats["permit_model_end"] = regs["permit"].permit_snapshot()
    stats["roi_core"] = roi_core.describe() if hasattr(roi_core, "describe") else None
    stats["rolling"] = rolling.describe() if rolling is not None else None
    if nav is not None:   # CTRLPS2: only with --nav soft|pl (summary.json keys as before otherwise)
        stats["nav"] = nav.describe()
    if fast is not None or a.kv_fast_status != "off":   # off: summary keys exactly as before
        stats["kv_fast"] = fast.describe() if fast is not None else {"impl": "python",
                                                                     "reason": a.kv_fast_status}
    stats["session_counts"] = {"%016x" % k: dict(v) for k, v in server.session_counts.items()}
    if local_src is not None:    # NN-CAM2 (udp: summary keys as before)
        local_src.stop()
        stats["local_source"] = local_src.describe()
        stats["local_chain"] = chain_holder[0].describe()
    if chassis is not None:      # CHASSIS0: stop frames (enable 0) before the port closes
        chassis.stop()
        stats["chassis"] = chassis.describe()
    server.stop()
    if stamps is not None:
        stamps.close()
    if log_fp is not None:
        log_fp.close()
    if ar is not None:
        stats["ar_telem_end"] = ar.telem()
        ar.close()
        if ar.colour is not None:
            ar.colour.stop()
        stats["ar"] = ar.describe()
        if desktop:
            stats["ar_desktop_enable"] = desktop("desktop_enable")
    import shutil
    shutil.rmtree(build, ignore_errors=True)
    if a.out:
        with open(os.path.join(a.out, "summary.json"), "w") as fp:
            json.dump(stats, fp, indent=1, sort_keys=True, default=str)
    print(json.dumps({"exit": reason[0]}), flush=True)


if __name__ == "__main__":
    main()
