#!/usr/bin/env python3
"""SRCHK1 关键对照①（DESIGN §10.1 判据 1–5，测前写死）＋ 联仿向量录制。
用法：srch_harness.py crit  [--maps N]      -> data/crit_runs.jsonl、crit_summary.json
      srch_harness.py trace                 -> data/trace_v1.bin、trace_v2.bin（HLS testbench 回放）"""
import ctypes
import glob
import json
import math
import os
import struct
import sys
import time

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [WT, os.path.join(WT, "tools", "car_run"), os.path.join(WT, "tools", "srch")]
import numpy as np  # noqa: E402
from host.search_belief import BeliefSearch, SearchConfig, _wrap1  # noqa: E402
import search_preview as sp  # noqa: E402
import mapping_ctl  # noqa: E402
from tools.car_geo_producer.common import base_camera  # noqa: E402
import srch_ps as S  # noqa: E402

RUN = "${FPGA_RUN}/SRCHK1"
K = np.array([[517.17, 0, 320.0], [0, 517.17, 240.65], [0, 0, 1.]])
IMG = (640, 480)
TBC = np.asarray(base_camera((0.10, 0.0, 0.39), 16.0), float)
CFG = SearchConfig.from_file(os.path.join(WT, "tools/car_run/search_config.json"))
BASE = "${FPGA_SRC}/runs"
MAPS = sorted(glob.glob(BASE + "/nationals-prelim-20261102/ORBVAL0/data/logs/run_*/export/*.yaml")) + \
    sorted(glob.glob(BASE + "/real-car-integ-20261015-results/gate_dig/*/*/export/*.yaml"))

PL = ctypes.CDLL(os.path.join(RUN, "build", "libplanner.so"))
PL.pl_scan.restype = ctypes.c_int
PL.pl_scan.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p]
PL_INF = 0x1fffffff
PL_STATS = {"calls": 0, "not_conv": 0, "rounds_max": 0}


def planner_lengths(self, start_xy):
    """规划核语义的路程：CELLOF（核）得机器人格，dev2 Engine<200,200>::scan 以它为源建场，读节点格（核 INIT_FINAL 给）。"""
    out = [math.inf] * len(self.nodes)
    src = self.cellof_c(*start_xy)
    if src is None:
        return out
    pot = np.zeros(40000, np.uint32)
    rounds = ctypes.c_uint(0)
    ok = PL.pl_scan(self.costmap_c.ctypes.data_as(ctypes.c_void_p), src[1] + 1, src[0] + 1,
                    pot.ctypes.data_as(ctypes.c_void_p), 256, ctypes.byref(rounds))
    PL_STATS["calls"] += 1
    PL_STATS["rounds_max"] = max(PL_STATS["rounds_max"], rounds.value)
    if not ok:
        PL_STATS["not_conv"] += 1
        return BeliefSearch.path_lengths(self, start_xy)
    for n, c in enumerate(self.node_cell_c):
        if c is not None:
            v = int(pot[(c[0] + 1) * 200 + c[1] + 1])
            if v < PL_INF:
                out[n] = v / 5000.0
    return out


def frames_of(poses):
    rows = sorted(mapping_ctl.poses_nodes(poses))
    return [(float(t), (float(x), float(y), float(yaw))) for t, x, y, yaw in rows]


def feed(bs, frames):
    for t, bp in frames:
        bs.observe(bs.camera_pose(*bp), t, base_pose=bp)


def start_robot(bs, target):
    ref = target or bs.nodes[0]
    n = min(bs.nodes, key=lambda p: (p[0] - ref[0]) ** 2 + (p[1] - ref[1]) ** 2)
    return (n[0], n[1], 0.)


def yaw_k(bs, yaw):
    return min(range(len(bs.yaw_set)), key=lambda k: abs(_wrap1(bs.yaw_set[k] - yaw)))


def score_of(bs, gains, new_m2, paths, robot, n, k, sweep):
    rx, ry, ryaw = robot
    x, y = bs.nodes[n]
    heading = math.atan2(y - ry, x - rx) if paths[n] > .2 else ryaw
    turn = abs(_wrap1(bs.yaw_set[k] - heading)) if heading is not None else 0.
    cost = paths[n] + bs.cfg.turn_cost_m_per_rad * turn + bs.cfg.step_cost_m
    return (new_m2[n, k] if sweep else gains[n, k]) / cost


def teacher_forced(P, F, target, frames):
    P.reset(); F.reset()
    P.begin(target); F.begin(target)
    feed(P, frames); feed(F, frames)
    robot = start_robot(P, target)
    P.robot = F.robot = robot
    steps, ties = [], []
    for step in range(12):
        F.visited = set(P.visited); F.failed_nodes = set(P.failed_nodes)
        mp, mf = P.mass_left(), F.mass_left()
        up, uf = P.unseen_mass(), F.unseen_mass()
        gp = P.gains()
        gf = F.gains()
        pp, pi = P.next_goal()
        fp, fi = F.next_goal()
        F.goals.pop() if fp is not None else None
        rec = dict(step=step, mass_rel=abs(mf - mp) / max(mp, 1e-300), unseen_abs=abs(uf - up),
                   p_stop=pi.get("stop"), f_stop=fi.get("stop"), p_sweep=pi.get("sweep"), f_sweep=fi.get("sweep"))
        if pp is not None:
            n, k = pi["node"], yaw_k(P, pi["yaw"])
            rec.update(p=[n, k], gain_rel=abs(gf[n, k] - gp[n, k]) / max(abs(gp[n, k]), 1e-300))
        if fp is not None:
            rec["f"] = [fi["node"], yaw_k(F, fi["yaw"])]
        rec["same"] = (rec.get("p") == rec.get("f")) and rec["p_stop"] == rec["f_stop"]
        if not rec["same"] and pp is not None and fp is not None:
            g2, a2 = P.gains(new_area=True)
            paths = P.path_lengths(robot[:2])
            sw = pi.get("sweep") is not None
            s1 = score_of(P, g2, a2, paths, robot, rec["p"][0], rec["p"][1], sw)
            s2 = score_of(P, g2, a2, paths, robot, rec["f"][0], rec["f"][1], sw)
            rec["tie_rel"] = abs(s1 - s2) / max(abs(s1), 1e-300)
            ties.append(rec["tie_rel"])
        steps.append(rec)
        if pp is None:
            break
        P.apply_base_view(pp.x, pp.y, pp.yaw)
        F.apply_base_view(pp.x, pp.y, pp.yaw)
        robot = (pp.x, pp.y, pp.yaw)
        P.robot = F.robot = robot
    mp, mf = P.mass_left(), F.mass_left()
    steps.append(dict(step="end", mass_rel=abs(mf - mp) / max(mp, 1e-300)))
    return steps


def free_run(bs, target, frames):
    bs.reset()
    bs.begin(target)
    feed(bs, frames)
    order = bs.preview(robot=start_robot(bs, target), max_goals=12)
    seq = [(g["node"], yaw_k(bs, g["yaw"])) for g in order if "x" in g]
    cum = [g["cum_prob"] for g in order if "x" in g]
    path = sum(g["path_m"] for g in order if "x" in g)
    return seq, (cum[-1] if cum else 0.), path, [g.get("stop") for g in order if "stop" in g]


def init_checks(P, F):
    univ = bool(np.array_equal(F.univ_flat_c, P.flat))
    cm = np.full((200, 200), 254, np.uint8)
    cm[1:P.ni + 1, 1:P.nj + 1] = np.where(P.trav, 0, 254)
    cost = bool(np.array_equal(F.costmap_c, cm.ravel()))
    nodec = [P._cell_of(x, y) for x, y in P.nodes] == F.node_cell_c
    ve = F.view_entries()
    tot = dif = 0
    for n in range(len(P.nodes)):
        idx, q, _cx, _cy = P._view_q(n)
        for k in range(len(P.yaw_set)):
            a = set(idx[q[k] > 0].tolist())
            e = ve.get(n * len(P.yaw_set) + k)
            b = set(e[0].tolist()) if e is not None else set()
            tot += len(a)
            dif += len(a ^ b)
    return dict(univ_same=univ, U=int(P.ux.size), costmap_same=cost, nodecell_same=nodec, entries=tot,
                entry_diff=dif, entry_diff_frac=dif / max(tot, 1), nsv=F.nsv)


def crit(nmaps=None):
    core = S.Core(os.path.join(RUN, "build", "libsrch.so"))
    out = open(os.path.join(RUN, "data", "crit_runs.jsonl"), "w")
    summ = dict(runs=0, init_fail=[], c2_fail=[], c3_steps=0, c3_mismatch=[], c4_fail=[], c5_same=0, c5_cum_fail=[],
                c5_path_fail=[])
    for y in MAPS[:nmaps]:
        name = os.path.basename(y)[:-5]
        poses = y[:-5] + "_poses.txt"
        fm = sp.load_map(y)
        nodes = sp.poses_to_nodes(poses)
        frames = frames_of(poses)
        P = BeliefSearch(fm, nodes, CFG)
        P.set_camera(K, IMG, TBC)
        t0 = time.time()
        F = S.make_fx(fm, nodes, CFG, K, IMG, TBC, core)
        ic = init_checks(P, F)
        ic.update(map=name, nodes=len(nodes), frames=len(frames), init_s=round(time.time() - t0, 2))
        out.write(json.dumps(dict(kind="init", **ic)) + "\n"); out.flush()
        print("INIT", json.dumps(ic)); sys.stdout.flush()
        if not (ic["univ_same"] and ic["costmap_same"] and ic["nodecell_same"]):
            summ["init_fail"].append(name)
        if ic["entry_diff_frac"] > 1e-3:
            summ["c2_fail"].append(name)
        U = P.ux.size
        targets = [("none", None), ("U/3", (float(P.ux[U // 3]), float(P.uy[U // 3]))),
                   ("2U/3", (float(P.ux[2 * U // 3]), float(P.uy[2 * U // 3])))]
        for tname, tgt in targets:
            for sname, fr in (("A", []), ("B", frames)):
                rid = "%s/%s/%s" % (name, tname, sname)
                F.path_lengths = BeliefSearch.path_lengths.__get__(F)
                F.trav = F.trav_c
                st = teacher_forced(P, F, tgt, fr)
                mism = [s for s in st if s.get("step") != "end" and not s["same"]]
                for s in st:
                    if s.get("step") == "end":
                        continue
                    summ["c3_steps"] += 1
                bad4 = [s for s in st if s["mass_rel"] > 1e-3 or s.get("unseen_abs", 0) > 1e-4 or
                        s.get("gain_rel", 0) > 1e-3]
                if mism:
                    summ["c3_mismatch"].append(dict(run=rid, steps=mism))
                if bad4:
                    summ["c4_fail"].append(dict(run=rid, steps=bad4))
                # 判据 5：路程换规划核，各自自由走 12 步
                sp_seq, sp_cum, sp_path, sp_stop = free_run(P, tgt, fr)
                F.path_lengths = planner_lengths.__get__(F)
                sf_seq, sf_cum, sf_path, sf_stop = free_run(F, tgt, fr)
                same5 = sp_seq == sf_seq
                summ["c5_same"] += int(same5)
                if abs(sp_cum - sf_cum) > 0.02:
                    summ["c5_cum_fail"].append(rid)
                if abs(sp_path - sf_path) > 0.05 * max(sp_path, 1e-9):
                    summ["c5_path_fail"].append(dict(run=rid, p=sp_path, f=sf_path))
                rec = dict(kind="run", run=rid, tf_steps=len(st) - 1, tf_mismatch=len(mism),
                           tf_mass_rel_max=max(s["mass_rel"] for s in st),
                           tf_unseen_abs_max=max(s.get("unseen_abs", 0) for s in st),
                           tf_gain_rel_max=max(s.get("gain_rel", 0) for s in st),
                           c5_same=same5, p_seq=sp_seq, f_seq=sf_seq, p_cum=sp_cum, f_cum=sf_cum,
                           p_path=round(sp_path, 3), f_path=round(sf_path, 3), p_stop=sp_stop, f_stop=sf_stop,
                           mism=mism)
                summ["runs"] += 1
                out.write(json.dumps(rec) + "\n"); out.flush()
                print("RUN %s tf_mis=%d mass=%.1e unseen=%.1e gain=%.1e c5=%s cum %.4f/%.4f path %.2f/%.2f" % (
                    rid, len(mism), rec["tf_mass_rel_max"], rec["tf_unseen_abs_max"], rec["tf_gain_rel_max"],
                    same5, sp_cum, sf_cum, sp_path, sf_path))
                sys.stdout.flush()
    n = summ["runs"]
    tie_ok = all(all(s.get("tie_rel", 1) < 1e-6 for s in m["steps"]) for m in summ["c3_mismatch"])
    nmis = sum(len(m["steps"]) for m in summ["c3_mismatch"])
    summ["planner"] = dict(PL_STATS)
    summ["verdict"] = {
        "c1": not summ["init_fail"],
        "c2": not summ["c2_fail"],
        "c3": tie_ok and nmis <= 0.01 * max(summ["c3_steps"], 1),
        "c4": not summ["c4_fail"],
        "c5": summ["c5_same"] >= 0.8 * n and not summ["c5_cum_fail"] and not summ["c5_path_fail"]}
    json.dump(summ, open(os.path.join(RUN, "data", "crit_summary.json"), "w"), indent=1)
    print("SUMMARY", json.dumps(dict(runs=n, c3_steps=summ["c3_steps"], c3_mismatch_steps=nmis,
                                     c5_same=summ["c5_same"], verdict=summ["verdict"], planner=summ["planner"])))


def write_trace(path, trace):
    with open(path, "wb") as f:
        f.write(struct.pack("<4sIII", b"SRTR", len(trace), S.DDR_BYTES, 0))
        for op, prm, before, after in trace:
            f.write(struct.pack("<II", op, prm))
            f.write(before.tobytes())
            f.write(after.tobytes())


def trace():
    for tag, mapname, target_frac, cycles in (("v1", "20261016_160423", 1 / 3., 3), ("v2", "20261031_072519", None, 4)):
        y = [m for m in MAPS if mapname in m][0]
        poses = y[:-5] + "_poses.txt"
        fm = sp.load_map(y)
        nodes = sp.poses_to_nodes(poses)
        frames = frames_of(poses)
        core = S.Core(os.path.join(RUN, "build", "libsrch.so"))
        core.trace = []
        F = S.make_fx(fm, nodes, CFG, K, IMG, TBC, core)
        tgt = None
        if target_frac is not None:
            U = F.ux.size
            tgt = (float(F.ux[int(U * target_frac)]), float(F.uy[int(U * target_frac)]))
        F.begin(tgt)
        # 观测流：建图轨迹按 0.2 s 取帧（产品 observe 的限频），取前 5*cycles 帧
        sel, last = [], -1e9
        for t, bp in frames:
            if t - last >= 0.2:
                sel.append((t, bp)); last = t
        k = 0
        for c in range(cycles):
            for _ in range(5):
                t, bp = sel[k]; k += 1
                T = F.camera_pose(*bp)
                F.apply_view(T[0, 3], T[1, 3], 0., T_world_camera=T)
            F.cellof_c(*sel[k - 1][1][:2])
            F._sel = None
            F.mass_left()
            F.dump_state()
        write_trace(os.path.join(RUN, "data", "trace_%s.bin" % tag), core.trace)
        print(tag, mapname, "ops", len(core.trace), [op for op, *_ in core.trace])


def plmock():
    """SRCHK3 驱动自检：同一 V2 序列分别走 Core（偏移地址）与 PLCore＋MockRegs（物理基址 0x7000_0000，寄存器表同 HLS），
    每次调用后比 DDR（参数块区除外，那里地址字段本应不同）。"""
    lib = os.environ.get("SRCH_LIB", os.path.join(RUN, "build", "libsrch.so"))
    outs = []
    for kind in ("core", "pl"):
        y = [m for m in MAPS if "20261031_072519" in m][0]
        poses = y[:-5] + "_poses.txt"
        fm, nodes, frames = sp.load_map(y), sp.poses_to_nodes(poses), frames_of(poses)
        if kind == "core":
            core = S.Core(lib)
        else:
            phys = 0x70000000
            buf = np.zeros(S.DDR_BYTES, np.uint8)
            core = S.PLCore(S.MockRegs(lib, buf, phys), buf, phys)
        snaps = []
        orig = core.run

        def run(op, prm, orig=orig, core=core, snaps=snaps):
            r = orig(op, prm)
            snaps.append((op, core.ddr[0x400:].copy()))
            return r
        core.run = run
        F = S.make_fx(fm, nodes, CFG, K, IMG, TBC, core)
        F.begin(None)
        sel, last = [], -1e9
        for t, bp in frames:
            if t - last >= 0.2:
                sel.append((t, bp)); last = t
        for t, bp in sel[:10]:
            T = F.camera_pose(*bp)
            F.apply_view(T[0, 3], T[1, 3], 0., T_world_camera=T)
        F.cellof_c(*sel[9][1][:2])
        F._sel = None
        F.mass_left()
        F.dump_state()
        outs.append(snaps)
    a, b = outs
    bad = sum(1 for (o1, d1), (o2, d2) in zip(a, b) if o1 != o2 or not np.array_equal(d1, d2))
    print("plmock calls", len(a), len(b), "mismatch", bad, "PASS" if bad == 0 and len(a) == len(b) else "FAIL")


if __name__ == "__main__":
    if sys.argv[1] == "plmock":
        plmock()
        sys.exit(0)
    if sys.argv[1] == "crit":
        crit(int(sys.argv[3]) if len(sys.argv) > 3 else None)
    else:
        trace()
