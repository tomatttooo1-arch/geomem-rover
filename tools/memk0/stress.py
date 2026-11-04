"""MEMK0 data set D3: seeded stress streams for the memory (product Config), ~3000 frames at 10 Hz each.

A simulated car drives a loop with stops and fast spins past objects on floor and tables; detections carry the
product's observation model (sd 0.02 + 0.01 r^2, see-through outliers, duplicate boxes, clipped boxes, false
boxes, below-ground and oversized boxes, unknown range); geometry words come from the true scene (free where an
object was taken away, surface where it stands, occluded / out of view / invalid depth). Scheduled events:
same-class twins, a move inside the exclusion zone, a move beyond it, take-away and put-back, a far-reading move,
untrusted / invalid poses, geometry-only calls, queries, keep_ids. Seed 3 adds 40 extra classes (slot load).
run(seed) drives a core.Memory; tools/memk0/shadow.py compares it with PLMemory call by call.
"""
import math

import numpy as np

from perception.fpga_replay import core, memory_gates, coco_zh

K = np.array([[380.0, 0, 320.0], [0, 380.0, 240.0], [0, 0, 1]])
W, H = 640, 480
SIZE = {"bottle": (.07, .22), "cup": (.09, .11), "chair": (.5, .9), "backpack": (.35, .45), "banana": (.18, .05),
        "cell phone": (.08, .15), "book": (.2, .27), "laptop": (.33, .23), "apple": (.08, .08), "bowl": (.16, .07)}
PD = {"cell phone": 0.35, "banana": 0.8}


def cam_pose(x, y, yaw):
    T = np.eye(4)
    z = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    xx = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    yy = np.array([0.0, 0.0, -1.0])
    T[:3, 0], T[:3, 1], T[:3, 2] = xx, yy, z
    T[:3, 3] = [x, y, 0.39]
    return T


def project(T, p):
    c = (p - T[:3, 3]) @ T[:3, :3]
    if c[2] <= 0.1:
        return None, c
    uv = K @ (c / c[2])
    return uv[:2], c


class World:
    def __init__(self, seed):
        self.rng = np.random.default_rng(seed)
        r = self.rng
        self.objs = []       # dict(label, pos, present)
        base = [("bottle", (2.0, 0.6, 0.05)), ("cup", (2.2, -0.9, 0.42)), ("cup", (2.2, -0.55, 0.42)),
                ("chair", (-1.8, 1.6, 0.45)), ("backpack", (-2.2, -0.4, 0.2)), ("banana", (0.3, 2.3, 0.05)),
                ("cell phone", (-0.4, -2.3, 0.75)), ("book", (1.0, -2.4, 0.75)), ("laptop", (-1.5, -1.9, 0.75)),
                ("apple", (2.3, 1.8, 0.42)), ("bowl", (-2.4, 1.0, 0.42))]
        for lab, p in base:
            q = np.array(p) + np.r_[r.normal(0, 0.1, 2), 0]
            self.objs.append({"label": lab, "pos": q, "present": True})
        if seed == 3:
            for k in range(40):
                a = r.uniform(0, 2 * math.pi)
                rad = r.uniform(2.0, 3.0)
                self.objs.append({"label": "item%02d" % k, "pos": np.array([rad * math.cos(a), rad * math.sin(a),
                                  r.choice([0.05, 0.42])]), "present": True})
        self.plan = self._plan(seed)

    def _plan(self, seed):
        r = self.rng
        n = len(self.objs)
        ev = []
        # (time s, kind, object index, arg)
        ev.append((40 + 5 * seed, "move", 0, (0.45, 0.0)))          # inside the exclusion zone
        ev.append((80 + 3 * seed, "move", 5, (2.0, -1.0)))          # beyond it
        ev.append((110, "remove", 3, None))                          # taken away (chair)
        ev.append((170, "restore", 3, None))                         # put back
        ev.append((130 + seed, "remove", 6, None))                   # phone taken away (low P_D)
        ev.append((200, "move", 9, (0.0, 0.7)))                      # apple moved 0.7 m
        ev.append((150, "remove", 1, None))                          # one twin taken away
        ev.append((230, "restore", 1, None))
        ev.append((250, "move", 8, (1.2, 0.3)))
        for k in range(4):
            ev.append((float(r.uniform(20, 280)), "move", int(r.integers(0, n)), tuple(r.normal(0, 0.4, 2))))
        return sorted(ev)


def path(t, seed):
    """Car (x, y, yaw) at time t: a loop of radius 1.0 m around the centre, stops, spins."""
    period = 60.0 + 5 * seed
    ph = (t % period) / period
    stop = (t % 37.0) < 6.0
    if stop:
        t = t - (t % 37.0)
    a = 2 * math.pi * ((t % period) / period)
    x, y = 1.0 * math.cos(a), 1.0 * math.sin(a)
    yaw = a + math.pi / 2 + 0.9 * math.sin(0.21 * t)
    if (t % 53.0) < 2.0:                                             # fast spin, ~90 deg/s
        yaw += math.radians(90.0) * (t % 53.0)
    return x, y, yaw


def observe(world, T, fid, t, cfg):
    r = world.rng
    out = []
    for ob in world.objs:
        if not ob["present"]:
            continue
        uv, c = project(T, ob["pos"])
        if uv is None or not (0.3 < c[2] < 3.6):
            continue
        wm, hm = SIZE.get(ob["label"], (0.12, 0.12))
        pw, ph = K[0, 0] * wm / c[2], K[1, 1] * hm / c[2]
        x1, y1, x2, y2 = uv[0] - pw / 2, uv[1] - ph / 2, uv[0] + pw / 2, uv[1] + ph / 2
        if x2 < 0 or y2 < 0 or x1 > W - 1 or y1 > H - 1:
            continue
        pd = PD.get(ob["label"], 0.92) * (1.0 if c[2] < 1.5 else 0.6 if c[2] < 2.5 else 0.3)
        if r.random() > pd:
            continue
        box = [max(0.0, x1), max(0.0, y1), min(W - 1.0, x2), min(H - 1.0, y2)]
        z = c[2]
        sd = 0.02 + 0.01 * z * z
        p = ob["pos"] + r.normal(0, sd, 3)
        if r.random() < 0.05:                                         # see-through reading
            ray = (ob["pos"] - T[:3, 3]) / np.linalg.norm(ob["pos"] - T[:3, 3])
            p = p + 0.24 * ray
        if r.random() < 0.01:
            p[2] = -0.1                                               # below ground
        if r.random() < 0.01:
            box = [box[0], box[1], min(W - 1.0, box[2] + 4 * pw), min(H - 1.0, box[3] + 4 * ph)]   # oversized
        out.append(make_obs(ob["label"], r.uniform(0.35, 0.95), p, box, T, fid, t, cfg, r))
        if r.random() < 0.05:                                         # second box on the same object
            q = p + r.normal(0, 0.02, 3)
            out.append(make_obs(ob["label"], r.uniform(0.3, 0.9), q, [b + r.normal(0, 3) for b in box], T, fid, t,
                                cfg, r))
    if r.random() < 0.03:                                             # false box
        lab = r.choice(["bottle", "cup", "banana", "book", "cell phone"])
        p = T[:3, 3] + T[:3, 2] * r.uniform(0.8, 2.8) + T[:3, 0] * r.normal(0, 0.3) + np.r_[0, 0, -0.3]
        uv, c = project(T, p)
        if uv is not None:
            box = [float(np.clip(uv[0] - 20, 0, W - 1)), float(np.clip(uv[1] - 25, 0, H - 1)),
                   float(np.clip(uv[0] + 20, 0, W - 1)), float(np.clip(uv[1] + 25, 0, H - 1))]
            out.append(make_obs(str(lab), r.uniform(0.2, 0.5), p, box, T, fid, t, cfg, r))
    return out


def make_obs(label, score, p, box, T, fid, t, cfg, r):
    z = float(((p - T[:3, 3]) @ T[:3, :3])[2])
    half = np.array([0.04, 0.04, 0.06])
    rng = None if r.random() < 0.02 else z
    return core.Observation(label, float(score), p, p - half, p + half, "red", 0.9, fid, t, tuple(map(float, box)),
                            "stress", extent_m=memory_gates.apparent_extent(box, max(z, 0.05), K),
                            truncated=memory_gates.is_truncated(box, (H, W), cfg.edge_margin_px), range_m=rng)


def evidence(world, memory, T, pose_valid):
    r = world.rng
    ev = {}
    for key, tr in memory.tracks.items():
        if not pose_valid:
            ev[key] = "invalid_pose"; continue
        uv, c = project(T, tr.position)
        if uv is None or not (0 <= uv[0] < W and 0 <= uv[1] < H) or c[2] > 4.0:
            ev[key] = "out_of_view"; continue
        x = r.random()
        if x < 0.03:
            ev[key] = "invalid_depth"; continue
        if x < 0.06:
            ev[key] = "too_small"; continue
        here = any(o["present"] and o["label"] == tr.label and np.linalg.norm(o["pos"] - tr.position) < 0.2
                   for o in world.objs)
        if here:
            ev[key] = "occluded" if r.random() < 0.1 else "surface_or_uncertain"
        else:
            ev[key] = "free" if r.random() < 0.8 else "surface_or_uncertain"
    return ev


def run(seed, frames=3000):
    cfg = core.Config()
    world = World(seed)
    r = world.rng
    memory = core.Memory("geometry", cfg)
    rgb, depth = np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32)
    plan = list(world.plan)
    t0 = 1792091000.0 + 1000 * seed
    gt = t0
    zh = [coco_zh.CLASS_ZH[k] for k in ("bottle", "cup", "chair", "banana", "cell phone", "apple")
          if k in coco_zh.CLASS_ZH]
    counts = {"frames": 0, "obs": 0, "geometry_only": 0, "queries": 0, "untrusted": 0, "pose_invalid": 0,
              "events": len(plan), "max_tracks": 0, "max_candidates": 0, "max_slots": 0}
    for i in range(frames):
        t = 0.1 * i + float(r.uniform(-0.02, 0.02))
        while plan and plan[0][0] <= t:
            _, kind, k, arg = plan.pop(0)
            ob = world.objs[k]
            if kind == "move":
                ob["pos"] = ob["pos"] + np.r_[arg[0], arg[1], 0.0]
            elif kind == "remove":
                ob["present"] = False
            elif kind == "restore":
                ob["present"] = True
        x, y, yaw = path(t, seed)
        T = cam_pose(x, y, yaw)
        fid = "s%d-%05d" % (seed, i)
        stamp = t0 + t
        frame = core.Frame(fid, stamp, rgb, depth, K, T, True, "stress", "stress")
        frame.ground_z_m = 0.0
        u = r.random()
        if u < 0.01:
            frame.pose_trusted = False; counts["untrusted"] += 1
        elif u < 0.015:
            frame.pose_valid = False; counts["pose_invalid"] += 1
        obs = observe(world, T, fid, stamp, cfg) if frame.pose_valid else []
        obs = obs[:30]
        counts["obs"] += len(obs)
        ev = evidence(world, memory, T, frame.pose_valid)
        if i == 900 and memory.tracks:
            memory.keep_ids = {sorted(memory.tracks)[0]}
        memory.update_memory(obs, ev, frame)
        if i % 7 == 3:
            gt = max(gt + 1e-3, stamp + 0.05)
            gframe = core.Frame("g" + fid, gt, rgb, depth, K, T, frame.pose_valid, "stress", "stress")
            if r.random() < 0.05:
                gframe.pose_trusted = False
            gev = evidence(world, memory, T, gframe.pose_valid)
            gev = {k: v for k, v in gev.items() if r.random() < 0.7}
            memory.update_geometry_only(gev, gframe)
            counts["geometry_only"] += 1
        if i % 50 == 25 and zh:
            memory.query_memory("找" + str(r.choice(zh)), after=stamp - 5.0)
            counts["queries"] += 1
        counts["frames"] += 1
        counts["max_tracks"] = max(counts["max_tracks"], len(memory.tracks))
        counts["max_candidates"] = max(counts["max_candidates"], len(memory.candidates))
        counts["max_slots"] = max(counts["max_slots"], len(memory.tracks) + len(memory.candidates))
    return counts
