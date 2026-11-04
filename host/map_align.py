"""Post-mapping map alignment (2026-10-25, owner plan A): one correction when the in-stack mapping run stops.

During the in-stack mapping run (car_inner MAPPING=1) map -> odom is bench_pose.py's identity: rtabmap runs with
publish_tf false, so the car and every object the runner remembers are written in the wheel-odometry frame. At the
stop, rtabmap_car.sh exports the map of the GLOBALLY OPTIMIZED graph; without a correction that map and the
odometry-frame car/objects are offset by the drift the optimization removed (10-22: ~0.4 m; a start cell that is
unknown/occupied in the new map makes the planner fail).

The correction of node i is C_i = opt_i x odom_i^-1 (opt_i: the exported optimized pose; odom_i: the odometry pose
rtabmap stored for that node). C_last (the last node of the graph) becomes map -> odom; every remembered position is
moved by the C_i of the node nearest in time to the moment it was observed (per node, not one T for all; the node and
observation stamps are both the camera's header stamps).

Data sources (the steadiest pair available after the export):
  optimized  OUT/export/*poses*.txt  rtabmap-export --poses --poses_format 11 ('stamp x y z qx qy qz qw id'), written
             with the same --opt as the --map export (rtabmap_car.sh), so it is the graph the exported grid was drawn from
  odometry   the database's Node table, column pose: 12 float32, the 3x4 [R|t] row-major (rtabmap DBDriverSqlite3;
             checked on a host database: the translations equal the exported odometry poses)
Root check (positive control): the graph's first node is the optimizer's fixed root, so C_first must be identity
(ROOT_TOL_M / ROOT_TOL_RAD). When it is not, the two pose sets are not in one frame and NO correction is applied.

No ROS. Python 3.8 compatible. Any failure -> AlignError with the reason; callers keep the identity and say why.
"""
import bisect
import math
import os
import sqlite3
import struct

import numpy as np

TOPIC = "/fpga/mapping/map_align"
ROOT_TOL_M = 0.01
ROOT_TOL_RAD = 0.01
# an observation stamp further than this before the first node / after the last node is not on the graph's clock
# (it gets C_last and is counted out_of_span)
SPAN_BEFORE_S = 30.0
SPAN_AFTER_S = 3600.0


class AlignError(ValueError):
    pass


def pose7_matrix(x, y, z, qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not n > 0 or not all(math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw)):
        raise AlignError("bad pose (%r)" % ((x, y, z, qx, qy, qz, qw),))
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                 [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                 [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]
    m[:3, 3] = (x, y, z)
    return m


def matrix_pose7(m):
    """[x, y, z, qx, qy, qz, qw] of a 4x4 rigid transform (qw >= 0)."""
    r = np.asarray(m, dtype=float)[:3, :3]
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        qw, qx, qy, qz = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        qw, qx, qy, qz = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        qw, qx, qy, qz = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    if qw < 0:
        qw, qx, qy, qz = -qw, -qx, -qy, -qz
    t = np.asarray(m, dtype=float)[:3, 3]
    return [float(t[0]), float(t[1]), float(t[2]), float(qx), float(qy), float(qz), float(qw)]


def rotation_angle(m):
    r = np.asarray(m, dtype=float)[:3, :3]
    return math.acos(max(-1.0, min(1.0, (r[0, 0] + r[1, 1] + r[2, 2] - 1) / 2)))


def read_opt_poses(path):
    """{node id: (stamp, 4x4)} of a format-11 poses file."""
    out = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p or p[0].startswith("#"):
                continue
            if len(p) < 9:
                raise AlignError("poses file %s: line %r is not 'stamp x y z qx qy qz qw id' (format 11)" % (path, line))
            out[int(float(p[8]))] = (float(p[0]), pose7_matrix(*map(float, p[1:8])))
    if not out:
        raise AlignError("poses file %s: no pose" % path)
    return out


def read_db_odom(db):
    """{node id: (stamp, 4x4 odometry pose)} from an rtabmap database's Node table (read only)."""
    if not os.path.isfile(db) or os.path.getsize(db) == 0:
        raise AlignError("database %s missing or empty" % db)
    try:
        con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
        try:
            rows = con.execute("SELECT id, stamp, pose FROM Node").fetchall()
        finally:
            con.close()
    except sqlite3.Error as exc:
        raise AlignError("database %s: Node table not readable (%s)" % (db, exc))
    out = {}
    for nid, stamp, blob in rows:
        if blob is None or len(blob) != 48:
            continue                                   # a null pose: not an odometry node
        v = struct.unpack("<12f", blob)
        m = np.eye(4)
        m[:3, :] = np.asarray(v, dtype=float).reshape(3, 4)
        out[int(nid)] = (float(stamp), m)
    if not out:
        raise AlignError("database %s: no node with a pose" % db)
    return out


def compute_tables(opt, odom, stamps, check_root=True):
    """The alignment of one graph from pose tables (no files): opt {node id: 4x4 optimized pose}, odom {node id:
    (stamp, 4x4 odometry pose)}, stamps {node id: stamp} (the node stamps the result is sorted/looked up by).
    -> {"map_odom", "node_id", "nodes", "count", "unmatched", "root"}; raises AlignError. check_root false: the
    caller checks the frames another way (10-27 live alignment when the graph's root node was not received)."""
    ids = sorted(set(opt) & set(odom))
    if not ids:
        raise AlignError("no node id in both the poses file (%d) and the database (%d)" % (len(opt), len(odom)))
    corr = {}
    for i in ids:
        c = opt[i] @ np.linalg.inv(odom[i][1])
        if not np.all(np.isfinite(c)):
            raise AlignError("node %d: correction not finite" % i)
        corr[i] = c
    first, last = ids[0], ids[-1]
    root_m = float(np.linalg.norm(corr[first][:3, 3]))
    root_rad = rotation_angle(corr[first])
    root = {"node_id": first, "offset_m": root_m, "offset_rad": root_rad, "tol_m": ROOT_TOL_M, "tol_rad": ROOT_TOL_RAD}
    if check_root and not (root_m <= ROOT_TOL_M and root_rad <= ROOT_TOL_RAD):
        raise AlignError("root check failed: the first graph node %d's correction is %.4f m / %.4f rad, not identity "
                         "-- the optimized and odometry poses are not in one frame" % (first, root_m, root_rad))
    nodes = sorted([stamps[i]] + matrix_pose7(corr[i]) for i in ids)
    return {"map_odom": matrix_pose7(corr[last]), "node_id": last, "nodes": nodes, "count": len(ids),
            "unmatched": {"poses_only": len(set(opt) - set(odom)), "db_only": len(set(odom) - set(opt))},
            "root": root}


def compute(poses_path, db_path):
    """The alignment of one mapping export: {"map_odom": pose7 (C_last), "node_id", "nodes": [[stamp, *pose7]] sorted
    by stamp, "count", "root": {...}, "source": {...}}. Raises AlignError (never returns a partial result)."""
    if not poses_path or not os.path.isfile(poses_path):
        raise AlignError("no exported poses file (%s)" % (poses_path or "none found"))
    if not db_path:
        raise AlignError("no database path")
    opt, odom = read_opt_poses(poses_path), read_db_odom(db_path)
    out = compute_tables({i: v[1] for i, v in opt.items()}, odom, {i: v[0] for i, v in opt.items()})
    out["source"] = {"optimized": os.path.abspath(poses_path), "odometry": os.path.abspath(db_path)}
    return out


def is_identity(align):
    eye = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    return list(align["map_odom"]) == eye and all(list(n[1:]) == eye for n in align["nodes"])


class Corrections(object):
    """Per-stamp lookup: the correction of the node nearest in time; C_last outside the graph's span."""

    def __init__(self, align):
        nodes = sorted(align["nodes"])
        self.stamps = [float(n[0]) for n in nodes]
        self.mats = [pose7_matrix(*n[1:]) for n in nodes]
        self.last = pose7_matrix(*align["map_odom"])
        self.out_of_span = 0

    def at(self, stamp):
        if not self.stamps or stamp is None or not math.isfinite(stamp) \
                or stamp < self.stamps[0] - SPAN_BEFORE_S or stamp > self.stamps[-1] + SPAN_AFTER_S:
            self.out_of_span += 1
            return self.last
        k = bisect.bisect_left(self.stamps, stamp)
        if k == 0:
            return self.mats[0]
        if k == len(self.stamps):
            return self.mats[-1]
        return self.mats[k] if self.stamps[k] - stamp < stamp - self.stamps[k - 1] else self.mats[k - 1]


def _point(c, p):
    return c[:3, :3] @ np.asarray(p, dtype=float) + c[:3, 3]


def _box(c, lower, upper):
    """The axis-aligned box enclosing the moved box (its 8 corners moved): under a rotation it can only grow."""
    lo, hi = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    centre, half = (lo + hi) / 2, (hi - lo) / 2
    centre, half = _point(c, centre), np.abs(c[:3, :3]) @ half
    return centre - half, centre + half


class Delta(object):
    """What moves a position written under the alignment ``prev`` (None: the odometry frame, the run's start) to the
    alignment ``new``: D(s) = C_new(s) x C_prev(s)^-1 at stamp s (10-27 live alignment: several alignments in one run).
    prev None -> D(s) = C_new(s) exactly (the 10-25 single-alignment arithmetic, unchanged)."""

    def __init__(self, new, prev=None):
        self.new = Corrections(new)
        self.prev = None if prev is None or is_identity(prev) else Corrections(prev)
        self.last = self.new.last if self.prev is None else self.new.last @ np.linalg.inv(self.prev.last)

    def absolute(self, stamp):
        """(C_new(s), C_prev(s)) -- C_prev identity when prev is the odometry frame."""
        return self.new.at(stamp), (np.eye(4) if self.prev is None else self.prev.at(stamp))

    def at(self, stamp):
        c = self.new.at(stamp)
        return c if self.prev is None else c @ np.linalg.inv(self.prev.at(stamp))

    @property
    def out_of_span(self):
        return self.new.out_of_span + (0 if self.prev is None else self.prev.out_of_span)


def apply_memory(memory, align, prev=None):
    """Move what this run's memory wrote into the frame of ``align``, in place.

    prev None (10-25, the one alignment at the mapping stop): memory is in the odometry frame and every position p
    becomes C(s) p. prev = the alignment the memory was last moved to (10-27 live alignment, host/live_align.py):
    p becomes C_new(s) C_prev(s)^-1 p, so after any number of alignments every position = C_latest(s) x its odometry
    position -- never corrected twice, never left behind.
    Moved: every track and creation-gate candidate (position, bounds, history position_m, identity-v2 fusion samples,
    negative-view camera poses),
    each by the correction at its own stamp; the memory odometer's last pose by C_last (else the next frame would count
    the jump as travel). Not moved: tracks never seen in this run (last_seen <= 0: loaded from an earlier run, already
    in their map's frame). Identity alignment from the odometry frame -> nothing touched at all. Returns counts.
    Bounds: an axis-aligned box can only grow under a rotation; the box an observation set is kept with the correction
    it was in (track attribute _align_box), so repeated alignments rotate THAT box once instead of growing it each time."""
    if prev is None and is_identity(align):
        return {"applied": False, "identity": True}
    corr = Delta(align, prev)
    moved = skipped = 0
    for t in list(memory.tracks.values()) + list(getattr(memory, "candidates", {}).values()):
        if not t.last_seen > 0:
            skipped += 1
            continue
        c = corr.at(float(t.last_seen))
        t.position = _point(c, t.position)
        base = getattr(t, "_align_box", None)
        if base is not None and np.array_equal(base[2], t.lower) and np.array_equal(base[3], t.upper):
            lo0, hi0, cbase = base[0], base[1], base[4]          # the observed box, untouched since: rotate it once
            c_new = corr.absolute(float(t.last_seen))[0]
            box_m = c_new @ np.linalg.inv(cbase)
        else:
            lo0, hi0 = np.asarray(t.lower, dtype=float), np.asarray(t.upper, dtype=float)
            c_new, cbase = corr.absolute(float(t.last_seen))
            box_m = c
        t.lower, t.upper = _box(box_m, lo0, hi0)
        try:
            t._align_box = (lo0, hi0, np.array(t.lower, dtype=float), np.array(t.upper, dtype=float), cbase)
        except AttributeError:                                  # a slotted stand-in: boxes may grow, nothing else
            pass
        hist = []
        for h in t.history:
            if isinstance(h, dict) and h.get("position_m") is not None:
                h = dict(h, position_m=_point(corr.at(float(h.get("timestamp") or 0.0)), h["position_m"]).tolist())
            hist.append(h)
        t.history = hist
        # identity v2 (D-050) recomputes the position from these samples at the next sighting: they move too, each by
        # the correction at its own stamp, or the old odometry-frame samples would pull the track back
        if getattr(t, "samples", None):
            t.samples = [(s, _point(corr.at(float(s)), p), sd) for s, p, sd in t.samples]
        t.negative_poses = [(s, corr.at(float(s)) @ np.asarray(p, dtype=float)) for s, p in t.negative_poses]
        moved += 1
    if getattr(memory, "_odometer_pose", None) is not None:
        memory._odometer_pose = corr.last @ memory._odometer_pose
    info = {"applied": True, "identity": is_identity(align), "moved": moved, "skipped_not_seen_this_run": skipped,
            "out_of_span_lookups": corr.out_of_span, "map_odom": list(align["map_odom"])}
    if prev is not None:
        info["incremental"] = True
    return info


def frame_correction(cur, tag, stamp):
    """10-27 live alignment: the 4x4 D that brings a frame whose pose was looked up with map -> odom = ``tag`` (4x4)
    into the memory's current alignment ``cur`` (None: the odometry frame): D = C_cur(stamp) x tag^-1."""
    c = np.eye(4) if cur is None or is_identity(cur) else Corrections(cur).at(float(stamp))
    return c @ np.linalg.inv(np.asarray(tag, dtype=float))


def find_poses(out_dir):
    """The one OUT/export/*poses*.txt, or None (with the candidates found)."""
    import glob
    found = sorted(glob.glob(os.path.join(out_dir, "export", "*poses*.txt")))
    return (found[0] if len(found) == 1 else None), found
