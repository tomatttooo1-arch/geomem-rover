"""CTRLPS3 cell_offset calibration (controller ruling 7): which continuous-coordinate convention does the B1
navigation window use? Measured on a known cell, not inferred.

Chain (all production code, host builds): a synthetic depth image of a box whose front face lies at a KNOWN odom x
-> host/m7_pipeline M7Pipeline + FrameNativeM7 (the frame core's C reference, hw/m7/frame_depth.cpp) -> Nano
RollingFusion with enable_grid (B1 v2 requests) -> RemotePlRollingBackend over an in-process link -> KV
RollingService + CpuRollingCore with the B1 grid C model (hw/b1_costmap) -> the grid plane the KV keeps for the
navigation core (RollingService.nav_planes, CTRLPS3) -> host/kv_nav body seed + inflation -> the cost window.
Then libkv_nav's own mapping (kv_nav_world_to_cell, i.e. Window::qx/qy + cell) is asked where the known surface
point falls, with cell_offset 0.5 and with 0.0.

Discriminating design: the face is at x = 0.6375 m (voxel 12 covers [0.60, 0.65); 0.6375 is 3/4 into it) and the
probed y is 0.1875 (3/4 into voxel 3). Under "cell i covers [i res, (i+1) res)" (offset 0.5) the point maps to the
lethal cell; under "cell i centred on i res" (offset 0, NavFn round) it maps one cell further in x AND y. Control
points at 1/4 of a cell map to the same cell under both conventions (the instrument agrees where it must).
Also: the KV-side numpy inflation (host/kv_nav.Inflation) vs the B1 C model's own cost plane on the same dump
(bit for bit), and a positive control (inflation radius 0.30 instead of 0.45 must differ).

Run: PYTHONPATH=. python3 tools/kv_nav/sim/cell_offset_probe.py OUT.json
"""
import json
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from host import rolling_nav_layer as rnl  # noqa: E402
from host.kv_nav import Inflation, KvNavCore, body_seed, load_library  # noqa: E402
from host.kv_rolling import (CpuRollingCore, RemotePlRollingBackend, RollingService, run_grid_cmodel)  # noqa: E402
from host.m7_frame_native import FrameNativeM7, compile_frame_native  # noqa: E402
from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture  # noqa: E402
from host.m7_rolling import H as HDR, OP_DUMP, RollingFusion, compile_rolling  # noqa: E402
from host.pl_grid_layer import GRID_F_COST, GRID_F_GRID, band_params, default_lut  # noqa: E402

V = 0.05
CLOCK = "ctrlps3_probe"
FACE_X = 0.6375                       # box front face (odom x), 3/4 into voxel 12
BOX = (FACE_X, 0.80, 0.1125, 0.2625, 0.0, 0.30)   # x0 x1 y0 y1 z0 z1
# a floor and a back wall so that rays see the obstacle band FREE in front of the box (else every cell but the
# hits stays UNKNOWN and the inflation comparison below has no gradient cells to compare)
SCENE = (BOX, (-1.0, 3.0, -2.0, 2.0, -0.05, 0.0), (1.5125, 1.6, -2.0, 2.0, 0.0, 1.0))
CAM = (0.10, 0.0, 0.20)               # camera centre (odom), looking along +x
IMG_W, IMG_H, F = 160, 120, 100.0


class InProcLink:
    def __init__(self, svc):
        from collections import Counter
        self.svc, self.counts = svc, Counter()
        self.feats = {"rolling": 1, "rolling_replay": 1, "rolling_grid": 1 if svc.grid_capable else 0}

    def require(self, feature, hello_timeout_s=3.0):
        return {"features": dict(self.feats)}

    def _control_resend(self, msg_type, ack_type, body, timeout_s, first_gap_s, flags=0):
        return self.svc.handle(body)

    def stats(self, timeout_s=0.5):
        return {"rolling": self.svc.describe()}

    def close(self):
        pass


def depth_image():
    """Optical-z depth of the box seen from CAM (optical z = +x, optical x = -y, optical y = -z); NaN = no return."""
    K = np.array([[F, 0, (IMG_W - 1) / 2], [0, F, (IMG_H - 1) / 2], [0, 0, 1.0]])
    R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)      # columns: optical x, y, z in odom
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, CAM
    vv, uu = np.mgrid[0:IMG_H, 0:IMG_W]
    d_opt = np.stack([(uu - K[0, 2]) / F, (vv - K[1, 2]) / F, np.ones_like(uu, float)], -1)
    d = d_opt @ R.T                                                 # odom direction per unit optical z
    o = np.array(CAM)
    best = np.full(d.shape[:2], np.inf)
    for box in SCENE:
        lo, hi = np.array(box[0::2]), np.array(box[1::2])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (lo - o) / d, (hi - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= tmin) & (tmin > 0)
        best = np.where(hit & (tmin < best), tmin, best)
    depth = np.where(np.isfinite(best), best, np.nan).astype(np.float32)   # t is the optical z (d_opt z = 1)
    return depth, K, T


def main(out):
    tmp = Path(tempfile.mkdtemp(prefix="ctrlps3_probe_"))
    core = CpuRollingCore(compile_rolling(tmp / "roll.so"), grid_library=str(tmp / "grid.so"))
    svc = RollingService(core)
    remote = RemotePlRollingBackend(None, link=InProcLink(svc))
    fusion = RollingFusion(remote, voxel_m=V)
    band = rnl.HeightBand(ground_z_m=0.0, robot_height_m=0.31)
    assert fusion.enable_grid(band, skip_changes=True)
    grid_spec = GridSpec((0.0, -0.8, -0.05), V, (32, 32, 32))
    pipe = M7Pipeline(FrameNativeM7(compile_frame_native(tmp / "frame.so")), grid_spec)
    depth, K, T = depth_image()
    robot = (0.10, 0.0, 0.0)
    for fid in range(1, 7):                                         # six identical frames (log-odds build-up)
        ns = time.monotonic_ns()
        ident = CaptureIdentity("s1", fid, ns, CLOCK, "cal", "cam", "e1")
        frame = DepthFrame(ident, depth, K, PoseAtCapture(ns, CLOCK, "cam", "e1", True, T))
        nav = pipe.ingest_geometry(frame, now_ns=ns + 1000, now_clock_id=CLOCK).navigation
        assert nav.valid, nav.reason
        r = fusion.ingest(nav, pipe, now_ns=ns + 2000, clock_id=CLOCK, robot_position_m=robot)
        assert r.accepted, r.reason
    t_planes, base, grid = svc.nav_planes[:3]
    grid = np.asarray(grid, np.int8).reshape(80, 80)
    res = {"face_x": FACE_X, "box": BOX, "camera": CAM, "window_base": list(base), "frames": 6,
           "hits": int(np.isfinite(depth).sum())}
    occ = np.argwhere(grid == 100)
    res["occupied_cells_world_box_region"] = sorted({(int(x) + base[0], int(y) + base[1]) for y, x in occ
                                                     if int(x) + base[0] < 25})
    # KV-side cost window (what the navigation core is fed), body seed at the robot
    infl = Inflation()
    cost = infl.costs(body_seed(grid, base, robot[:2]))
    lib = load_library()
    probes = {"discriminating (3/4 cell)": (FACE_X, 0.1875), "control (1/4 cell)": (0.6125, 0.1625)}
    res["probes"] = {}
    for off in (0.5, 0.0):
        nav_core = KvNavCore(lib, grid_source=None, params={"cell_offset": off})
        for name, (x, y) in probes.items():
            cx, cy = nav_core.world_to_cell(x, y, base[0], base[1])
            res["probes"]["%s offset=%g" % (name, off)] = {
                "point": [x, y], "cell_local": [cx, cy], "cell_world": [cx + base[0], cy + base[1]],
                "grid": int(grid[cy, cx]) if 0 <= cx < 80 and 0 <= cy < 80 else None,
                "cost": int(cost[cy, cx]) if 0 <= cx < 80 and 0 <= cy < 80 else None}
    # floor() lattice the rolling map itself uses (RollingFusion.window_base_for / m7_grid_policy)
    res["lattice_floor_of_point"] = [int(np.floor(FACE_X / V)), int(np.floor(0.1875 / V))]
    # numpy inflation vs the B1 C model's own cost plane on the same map (DUMP), no body seed on either
    dh = core.k._call(OP_DUMP)
    offs = [int(dh[HDR["OFFSET"] + a]) for a in range(3)]
    lo, hi, tall = band_params(band, base[2], core.config.dims[2], V)
    spec = {"flags": GRID_F_GRID | GRID_F_COST, "band_lo": lo, "band_hi": hi, "tall_lo": tall,
            "body_cx": 0, "body_cy": 0, "body_r": 0}
    st, g2, aux2, cost_c = run_grid_cmodel(core.grid_fn, core.k._dump, offs, spec, default_lut(), 6400)
    g2 = g2.reshape(80, 80)
    cost_np = infl.costs_numpy(grid)
    res["inflation_c_vs_cmodel_diff_cells"] = int((infl.costs(grid) != cost_c.reshape(80, 80)).sum())
    res["cmodel_status"] = int(st)
    res["cmodel_grid_equals_kept_plane"] = bool((g2 == grid).all())
    res["inflation_numpy_vs_cmodel_diff_cells"] = int((cost_np != cost_c.reshape(80, 80)).sum())
    res["inflation_cells_ge253"] = int((cost_np >= 253).sum())
    res["inflation_cells_gradient_1_252"] = int(((cost_np >= 1) & (cost_np <= 252)).sum())
    res["grid_counts"] = {"occupied": int((grid == 100).sum()), "free": int((grid == 0).sum()),
                          "unknown": int((grid == -1).sum())}
    pos_ctrl = Inflation(inflation_radius=0.30).costs(grid)
    res["positive_control_radius_030_diff_cells"] = int((pos_ctrl != cost_c.reshape(80, 80)).sum())
    # the production wiring: make_core(field="soft", grid_source=<the RollingService>) -> RollingGridSource
    from host.kv_nav import make_core
    kv = make_core("soft", svc, lib=lib)
    kv.on_odom({"odom_pose": (robot[0], robot[1], 0.0), "v": 0.0, "w": 0.0, "map_odom": (0.0, 0.0, 0.0),
                "permit": True, "teleop": False})
    kv.step(time.monotonic())
    src_cost = kv.grid.latest(time.monotonic(), robot[:2])
    res["make_core_rolling"] = {"grid_source": type(kv.grid).__name__, "maps_fed": kv.counts["map"],
                                "base": [src_cost[1], src_cost[2]], "same_cost_as_probe": bool((src_cost[0] == cost).all())}
    ok05 = res["probes"]["discriminating (3/4 cell) offset=0.5"]["grid"] == 100
    ok00 = res["probes"]["discriminating (3/4 cell) offset=0"]["grid"] == 100
    ctrl = (res["probes"]["control (1/4 cell) offset=0.5"]["cell_local"] ==
            res["probes"]["control (1/4 cell) offset=0"]["cell_local"])
    res["verdict"] = {"offset_0.5_hits_lethal": ok05, "offset_0_hits_lethal": ok00, "control_same_cell": ctrl,
                      "cell_offset": 0.5 if (ok05 and not ok00 and ctrl) else None}
    Path(out).write_text(json.dumps(res, indent=1))
    print(json.dumps(res["verdict"]), res["inflation_numpy_vs_cmodel_diff_cells"],
          res["positive_control_radius_030_diff_cells"])


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "cell_offset_probe.json")
