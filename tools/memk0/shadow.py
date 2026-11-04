"""MEMK0 key comparison 1: every Memory call of a run is replayed on a PLMemory shadow and compared.

Arm A is the product core.Memory (float64, unchanged); arm B is memory_pl.PLMemory (PS control + the C model of
the core). install() wraps update_memory / update_geometry_only / query_memory of core.Memory at class level, so
any runner or unit test that builds a Memory is covered without changing it. A shadow starts at the memory's
first call when its strategy/Config are supported and it is still empty; it stops ("tainted") when code outside
the three entries changed the memory between calls (unit tests that poke tracks, store restores), or when the
core reports a capacity limit. Criteria (MEMK0/DESIGN.md section 10, fixed before the runs):
  outputs and snapshots equal field by field; positions, bounds, distances <= 5 mm; raw existence <= 1e-4.
"""
import json
import math
from collections import Counter

import numpy as np

from perception.fpga_replay import core
from perception.fpga_replay import memory_pl

POS_KEYS = {"position_m", "bounds_m", "distance_m", "from_m", "to_m", "offset_m"}
TOL_POS, TOL_EX = 5e-3, 1e-4


class Stats:
    def __init__(self):
        self.calls = Counter()          # compared calls per entry
        self.mismatch = []              # first mismatches (details)
        self.n_mismatch = 0
        self.mismatch_calls = 0
        self.max_diff = Counter()       # per numeric key: max |A - B|
        self.skipped = Counter()        # memories / calls not compared, by reason
        self.memories = 0
        self.events = Counter()         # branch coverage of arm A outputs
        self.core_calls = Counter()     # arm B core calls
        self.frames = 0

    def as_dict(self):
        return {"compared_calls": dict(self.calls), "mismatching_calls": self.mismatch_calls,
                "mismatches": self.n_mismatch, "first_mismatches": self.mismatch[:30],
                "max_abs_diff": {k: float(v) for k, v in self.max_diff.items()},
                "skipped": dict(self.skipped), "shadowed_memories": self.memories,
                "events": dict(self.events), "core_calls": dict(self.core_calls), "frames": self.frames}


STATS = Stats()
_ORIG = {}
_OPTS = {"bias": 0.0, "chi2_24": None}


def _num_diff(key, a, b, path, out):
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            out.append("%s: length %d vs %d" % (path, len(a), len(b)))
            return
        for k, (x, y) in enumerate(zip(a, b)):
            _num_diff(key, x, y, "%s[%d]" % (path, k), out)
        return
    if a is None or b is None:
        if a is not b:
            out.append("%s: %r vs %r" % (path, a, b))
        return
    d = abs(float(a) - float(b))
    STATS.max_diff[key] = max(STATS.max_diff[key], d)
    if not d <= TOL_POS:
        out.append("%s: %r vs %r" % (path, a, b))


def compare(a, b, path, out, key=None):
    if key in POS_KEYS:
        _num_diff(key, a, b, path, out)
        return
    if key == "existence" and isinstance(a, float) and isinstance(b, float):   # rounded to 3 places in dicts
        d = abs(a - b)
        STATS.max_diff["existence_rounded"] = max(STATS.max_diff["existence_rounded"], d)
        if d > 1.0e-3 + 1e-9:
            out.append("%s: %r vs %r" % (path, a, b))
        return
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            out.append("%s: keys %s vs %s" % (path, sorted(set(a) - set(b)), sorted(set(b) - set(a))))
        for k in a:
            if k in b:
                compare(a[k], b[k], "%s.%s" % (path, k), out, k)
        return
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            out.append("%s: length %d vs %d" % (path, len(a), len(b)))
            return
        for k, (x, y) in enumerate(zip(a, b)):
            compare(x, y, "%s[%d]" % (path, k), out, key)
        return
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        if not np.array_equal(np.asarray(a), np.asarray(b)):
            out.append("%s: arrays differ" % path)
        return
    if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
        return
    if a != b:
        out.append("%s: %r vs %r" % (path, a, b))


def compare_state(A, B, out):
    """Internal state the snapshot does not show (DESIGN section 10, criterion 2) and raw existence."""
    compare(list(A.candidates), list(B.candidates), "candidates", out)
    for k, c in A.candidates.items():
        if k in B.candidates:
            cb = B.candidates[k]
            compare([len(c.history), c.move_of, c.last_seen], [len(cb.history), cb.move_of, cb.last_seen],
                    "candidate." + k, out)
            _num_diff("candidate_position", c.position.tolist(), cb.position.tolist(), "candidate.%s.pos" % k, out)
    compare(dict(A.merged), dict(B.merged), "merged_map", out)
    compare(dict(A.identity_stats), dict(B.identity_stats), "identity_stats", out)
    compare(dict(A.candidate_stats), dict(B.candidate_stats), "candidate_stats", out)
    compare(dict(A.rejected), dict(B.rejected), "rejected_counts", out)
    for k, t in A.tracks.items():
        tb = B.tracks.get(k)
        if tb is None:
            continue
        compare(sorted(t.covis), sorted(tb.covis), "covis." + k, out)
        if (t.existence is None) != (tb.existence is None):
            out.append("existence.%s: %r vs %r" % (k, t.existence, tb.existence))
        elif t.existence is not None:
            d = abs(float(t.existence) - float(tb.existence))
            STATS.max_diff["existence_raw"] = max(STATS.max_diff["existence_raw"], d)
            if d > TOL_EX:
                out.append("existence.%s: %r vs %r" % (k, t.existence, tb.existence))


def _signature(m):
    return repr((m.snapshot_orig() if hasattr(m, "snapshot_orig") else [t.as_dict() for t in m.tracks.values()],
                 sorted(m.candidates), sorted(m.merged.items())))


def _events(out):
    if not isinstance(out, dict):
        return
    for r in out.get("rejected", []) or []:
        STATS.events["rejected:" + r["reason"]] += 1
    for r in out.get("candidates", []) or []:
        STATS.events["held_move" if "move_of" in r else "held"] += 1
    for r in out.get("merged", []) or []:
        STATS.events["merged:" + r.get("reason", "merge_pass")] += 1
    for r in out.get("relocated", []) or []:
        STATS.events["relocated:" + r["via"]] += 1
    for r in out.get("ambiguous", []) or []:
        STATS.events["ambiguous"] += 1
    for t in out.get("tracks") or []:
        STATS.events["state:" + t["state"]] += 1


def _shadow(A):
    sh = A.__dict__.get("_mk_shadow")
    if sh is not None:
        return sh
    sh = {"B": None, "why": None, "sig": None}
    A.__dict__["_mk_shadow"] = sh
    if A.tracks or A.candidates or A.seen_frames or A.geometry_seen_frames:
        sh["why"] = "not_empty_at_first_call"
    else:
        why = memory_pl.supported(A.cfg, A.strategy)
        if why:
            sh["why"] = "unsupported:" + why
        else:
            sh["B"] = memory_pl.PLMemory(A.strategy, A.cfg, backend=memory_pl.CModel.get(_OPTS["chi2_24"]),
                                         pos_bias_m=_OPTS["bias"])
            STATS.memories += 1
    if sh["why"]:
        STATS.skipped["memory:" + sh["why"]] += 1
    return sh


def _wrap(name):
    orig = _ORIG[name]

    def wrapper(self, *args, **kw):
        if isinstance(self, memory_pl.PLMemory):
            return orig(self, *args, **kw)
        sh = _shadow(self)
        B = sh["B"]
        if B is not None and sh["sig"] is not None and _signature(self) != sh["sig"]:
            sh["B"], B = None, None
            STATS.skipped["memory:tainted_by_outside_change"] += 1
        errA = outA = None
        try:
            outA = orig(self, *args, **kw)
        except Exception as e:      # the same error must come from arm B
            errA = e
        if B is not None:
            B.keep_ids = set(self.keep_ids)
            errB = outB = None
            try:
                outB = getattr(B, name)(*args, **kw)
            except memory_pl.Unsupported as e:
                sh["B"] = None
                STATS.skipped["memory:core_limit:%s" % e] += 1
                B = None
            except Exception as e:
                errB = e
            if B is not None:
                diffs = []
                if (errA is None) != (errB is None) or (errA is not None and str(errA) != str(errB)):
                    diffs.append("error: %r vs %r" % (errA, errB))
                elif errA is None:
                    compare(outA, outB, name, diffs)
                    if name != "query_memory":
                        compare_state(self, B, diffs)
                STATS.calls[name] += 1
                if name == "update_memory":
                    STATS.frames += 1
                if diffs:
                    STATS.mismatch_calls += 1
                    STATS.n_mismatch += len(diffs)
                    if len(STATS.mismatch) < 30:
                        fr = args[2] if name == "update_memory" else args[1] if name == "update_geometry_only" else None
                        STATS.mismatch.append({"call": name, "frame": getattr(fr, "frame_id", None),
                                               "t": getattr(fr, "timestamp", None), "diffs": diffs[:8]})
                for k, v in B.calls.items():
                    STATS.core_calls[k] += v
                B.calls = Counter()
        if name != "query_memory":
            _events(outA)
        if sh.get("B") is not None:
            sh["sig"] = _signature(self)
        if errA is not None:
            raise errA
        return outA
    return wrapper


def install(bias=0.0, chi2_24=None):
    _OPTS["bias"], _OPTS["chi2_24"] = float(bias), chi2_24
    for name in ("update_memory", "update_geometry_only", "query_memory"):
        if name not in _ORIG:
            _ORIG[name] = getattr(core.Memory, name)
            setattr(core.Memory, name, _wrap(name))
    # PLMemory overrides update_memory / update_geometry_only itself; query_memory is inherited (wrapped: passes)


def report(path=None, extra=None):
    d = STATS.as_dict()
    if extra:
        d.update(extra)
    if path:
        with open(path, "w") as f:
            json.dump(d, f, indent=1, ensure_ascii=False, default=str)
    return d
