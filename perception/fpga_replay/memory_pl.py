"""MEMK0: the object memory with its numeric part in the PL memory core (PS control + core).

runs/nationals-prelim-20261102/MEMK0/DESIGN.md (sections 2, 8). ``PLMemory`` keeps every control decision of
core.Memory (7fe36e1) line for line -- ids, numbers, states, reasons, history, co-visibility, candidates'
lifecycle, which branch an observation takes, who survives a merge -- and takes every number from the core:
admission and dedupe flags, distances and gate flags, greedy assignment, sample rings, robust fusion, the
existence score. The core is the HLS kernel on the board and the same C++ source as a shared library here
(hw/memk0/memk0_core.cpp via ctypes). The plain Memory is unchanged; CAR_MEMORY_CORE=py|pl picks one.

Not supported (``Unsupported`` is raised; the caller keeps the Python memory): a Config whose core constants
differ from the build (digest), switches other than the product setting, strategy other than geometry, frames
with pose_sigma_m, more than 32 observations in a frame, 64 slots, 64 labels or 256 pair records.
"""
import ctypes
import importlib.util
import os
import subprocess
from pathlib import Path

import numpy as np

from perception.fpga_replay import memory_gates
from perception.fpga_replay.core import Memory, Track

ROOT = Path(__file__).resolve().parents[2]
HW = ROOT / "hw" / "memk0"

# ---- block layout (hw/memk0/memk0.h)
MAGIC, CMD_INIT, CMD_ASSOC, CMD_EXEC = 0x4D4B, 1, 2, 3
WORDS, SLOTS, NOBS, NCLS, NPAIRS = 1024, 64, 32, 64, 256
OBS_BASE, OBS_W, CLS_BASE, CLS_W, GEO_BASE, OP_BASE, OP_W, NOPS = 16, 12, 400, 6, 784, 792, 4, 58
RES_OBS, RES_OBS_W, RES_PAIRS, RES_DUMP, DUMP_W = 8, 3, 104, 616, 5
F_POSE, F_DET, F_SKIP = 1, 2, 4
OP_TOUCH, OP_CAND, OP_NEW, OP_COPY_SAMPLES, OP_CLEAR_SAMPLES, OP_MERGE, OP_COPY_PLACE = 1, 2, 3, 4, 5, 6, 7
OP_RESET_L, OP_FREE, OP_RESTORE, OP_QUERY, OP_MERGE_SCAN, OP_EXIST, OP_DUMP = 8, 9, 10, 11, 12, 13, 14
PF_GATE, PF_FAR, PF_RADIUS, PF_EXCL, PF_MERGE, PF_NEAR, PF_ASSIGN, PF_CAND = 1, 2, 4, 8, 16, 32, 64, 128
TYPE_TRACK, TYPE_CAND = 1, 2


class Unsupported(Exception):
    """This configuration or input is outside what the core implements; use core.Memory."""


def _gen_consts():
    spec = importlib.util.spec_from_file_location("memk0_gen_consts", HW / "gen_consts.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_GEN = _gen_consts()


def supported(cfg, strategy="geometry"):
    """None when PLMemory implements (strategy, cfg), else the reason."""
    if strategy != "geometry":
        return "strategy"
    for k, v in _GEN.PS_REQUIRED.items():
        if getattr(cfg, k) != v:
            return "switch %s" % k
    if _GEN.config_digest(cfg) != _GEN.config_digest(type(cfg)()):
        return "core constants"
    if cfg.below_ground_tol_m is None or cfg.size_margin is None:
        return "gates off"
    return None


def _q(x, frac):
    return int(np.floor(float(x) * (1 << frac) + 0.5))


def _u(x):
    return int(x) & 0xFFFFFFFF


def _s(x):
    x = int(x) & 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


class CModel:
    """The core as a shared library; one per build. The C state is per process, so the library swaps the
    persistent state of the PLMemory that calls it (several memories in one test process)."""
    _libs = {}

    def __init__(self, chi2_24=None):
        tag = "" if chi2_24 is None else "_chi%d" % chi2_24
        out = Path(os.environ.get("MEMK0_LIB_DIR", str(HW))) / ("libmemk0%s.so" % tag)
        src = HW / "memk0_core.cpp"
        deps = [src, HW / "memk0.h", HW / "memk0_consts.h", HW / "memk0_rsqrt_rom.h", HW / "memk0_types.h"]
        if not out.exists() or out.stat().st_mtime < max(p.stat().st_mtime for p in deps):
            flags = [] if chi2_24 is None else ["-DMEMK0_CHI2_24_OVR=%d" % chi2_24]
            subprocess.run([str(HW / "build_lib.sh"), str(out)] + flags, check=True)
        self.lib = ctypes.CDLL(str(out))
        self.lib.memk0_call.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.memk0_state_save.argtypes = [ctypes.c_void_p]
        self.lib.memk0_state_load.argtypes = [ctypes.c_void_p]
        self.size = self.lib.memk0_state_size()
        self.active = None

    @classmethod
    def get(cls, chi2_24=None):
        if chi2_24 not in cls._libs:
            cls._libs[chi2_24] = CModel(chi2_24)
        return cls._libs[chi2_24]

    def call(self, owner, cmd, res):
        if self.active is not owner:
            if self.active is not None and self.active._cstate is not None:
                self.lib.memk0_state_save(self.active._cstate.ctypes.data)
            if owner._cstate is None:
                owner._cstate = np.zeros(self.size, np.uint8)
            else:
                self.lib.memk0_state_load(owner._cstate.ctypes.data)
            self.active = owner
        self.lib.memk0_call(cmd.ctypes.data, res.ctypes.data)


# ---- MERGE10 (2026-11-04): the core on the board, behind the m7 control path (slot 14 of the address map,
# hw/m7/pl/bd/m7_bd.tcl). The command / result block layout is the same on both shells (hw/memk0/memk0.h): only
# where the two 1024-word halves sit in the slot differs.
#   "lite": MEMK3-type core (xmemk0_top_hw.h): s_axilite arrays, cmd at slot + 0x1000, res at slot + 0x2000.
#   "bram": MEMK4-type core (FPGA_M7_PL_MEMK_IF=bram): ap_ctrl only at slot + 0; the 2048-word block behind the
#           AXI BRAM controller bc_memk at slot + 0x8000 (cmd words 0-1023 = 0x8000, res words 1024-2047 = 0x9000).
# Either way the PS writes the cmd half, starts the core, waits for ap_done and reads the res half; it never writes
# the res half (the core keeps per-observation values there across calls, like the C model's result store).
MEMK_BASE, MEMK_WINDOW = 0xA00E0000, 0x10000
LAYOUTS = {"lite": {"cmd": 0x1000, "res": 0x2000}, "bram": {"cmd": 0x8000, "res": 0x9000}}
AP_CTRL, AP_START, AP_DONE, AP_IDLE = 0x00, 1, 2, 4


class DevMemWindow:
    """The 64 KiB slot through /dev/mem; every access is one aligned 32-bit load or store."""

    def __init__(self, base=MEMK_BASE, size=MEMK_WINDOW, path="/dev/mem"):
        import mmap
        self._fd = os.open(path, os.O_RDWR | os.O_SYNC)
        self._mm = mmap.mmap(self._fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        self._w = memoryview(self._mm).cast("I")

    def read32(self, off):
        return self._w[off >> 2]

    def write32(self, off, v):
        self._w[off >> 2] = int(v) & 0xFFFFFFFF

    def write_words(self, off, words):
        w, k0 = self._w, off >> 2
        for k, v in enumerate(np.asarray(words, np.uint32).tolist()):
            w[k0 + k] = v

    def read_words(self, off, out):
        w, k0 = self._w, off >> 2
        for k in range(len(out)):
            out[k] = w[k0 + k]

    def close(self):
        self._w.release(); self._mm.close(); os.close(self._fd)


class MMIOBackend:
    """PLMemory backend for the core on the board (``backend=MMIOBackend(DevMemWindow(), "bram")``). The core holds
    one memory's state, so one PLMemory per backend (the C model swaps states, the board cannot)."""

    def __init__(self, win, layout="bram", timeout_s=1.0):
        if layout not in LAYOUTS:
            raise ValueError("layout: %s, got %r" % ("|".join(LAYOUTS), layout))
        self.win, self.layout, self.off = win, layout, LAYOUTS[layout]
        self.timeout_s, self.owner = float(timeout_s), None

    def call(self, owner, cmd, res):
        import time
        if self.owner is None:
            self.owner = owner
        elif self.owner is not owner:
            raise Unsupported("one PLMemory per core on the board")
        w = self.win
        w.write_words(self.off["cmd"], cmd)
        w.write32(AP_CTRL, AP_START)
        t0 = time.monotonic()
        # ap_done (bit 1) is clear-on-read and stays set until this loop reads it; ap_idle is not used here because
        # it can still read 1 between the start write and the core taking ap_start.
        while not (w.read32(AP_CTRL) & AP_DONE):
            if time.monotonic() - t0 > self.timeout_s:
                raise Unsupported("core did not finish in %.1f s" % self.timeout_s)
        w.read_words(self.off["res"], res)


class PLMemory(Memory):
    def __init__(self, strategy="geometry", cfg=None, backend=None, pos_bias_m=0.0):
        if cfg is None:
            from perception.fpga_replay.core import Config
            cfg = Config()
        why = supported(cfg, strategy)
        if why:
            raise Unsupported(why)
        super().__init__(strategy, cfg)
        self.be = backend or CModel.get()
        self._cstate = None
        self.cmd = np.zeros(WORDS, np.uint32)
        self.res = np.zeros(WORDS, np.uint32)
        self.pos_bias = float(pos_bias_m)       # positive control (b) of the key comparison only
        self._slot = {}                          # id -> slot (tracks and candidates)
        self._obj = [None] * SLOTS               # slot -> Track
        self._cls = {}                           # label -> class index
        self._ops, self._fix = [], []
        self._dirty = set()                      # slots changed since this frame's ASSOC
        self._w0 = None                          # window base (int32 header)
        self._seq = 0
        self._hdr = None
        self.calls = {"init": 0, "assoc": 0, "exec": 0}
        self._exist_ctx, self._exist_done, self._protected_now = None, False, set()
        self._call(CMD_INIT, 0, 0)

    # ------------------------------------------------------------------ core calls
    def _class(self, label):
        k = self._cls.get(label)
        if k is not None:
            return k
        k = len(self._cls)
        if k >= NCLS:
            raise Unsupported("labels")
        self._cls[label] = k
        cfg, row = self.cfg, memory_gates.CLASS_MAX_APPARENT_M.get(label)
        base = CLS_BASE + CLS_W * k
        self.cmd[base + 0] = _u(_q(memory_gates.class_slack(label), 20))
        mw, mh = (row[0] if row else (0.0, 0.0))
        self.cmd[base + 1] = _u(_q(mw * cfg.size_margin, 16))
        self.cmd[base + 2] = _u(_q(mh * cfg.size_margin, 16))
        self.cmd[base + 3] = _u(_q(dict(cfg.miss_class_pd).get(label, cfg.miss_class_default), 24))
        self.cmd[base + 4] = _u(_q(self._create_min_score(label), 24))
        self.cmd[base + 5] = (1 if row is not None else 0) | ((2 if row is not None and row[1] else 0))
        return k

    def _mu(self):
        used = [j for j, o in enumerate(self._obj) if o is not None]
        return max(used) + 1 if used else 0

    def _frame_header(self, frame, pose_valid, det, skip, nb):
        w = self._ex_window(frame.timestamp)
        if self._w0 is None:
            self._w0 = w
        T = frame.T_world_camera
        ground = getattr(frame, "ground_z_m", None)
        ground = self.cfg.ground_z_m if ground is None else float(ground)
        self._hdr = dict(flags=(F_POSE if pose_valid else 0) | (F_DET if det else 0) | (F_SKIP if skip else 0),
                         nb=nb, W=w - self._w0, seq=self._seq,
                         op=_q(self.odometer[0], 22), ot=_q(self.odometer[1], 21), ground=_q(ground, 18),
                         T=[_q(v, 18) for v in T[:3, 3]], R2=[_q(v, 24) for v in T[:3, 2]])
        for v in (self._hdr["op"], self._hdr["ot"]):
            if not 0 <= v < (1 << 32):
                raise Unsupported("odometer range")
        if not -(1 << 31) <= self._hdr["W"] < (1 << 31):
            raise Unsupported("window range")

    def _call(self, kind, n, nops):
        c, h = self.cmd, self._hdr
        mu = self._mu()
        if h is None:
            h = dict(flags=0, nb=0, W=0, seq=0, op=0, ot=0, ground=0, T=[0, 0, 0], R2=[0, 0, 0])
        c[0] = MAGIC | (kind << 16) | (h["flags"] << 20)
        c[1] = (n & 0xFF) | (mu << 8) | (nops << 16)
        c[2] = _u(h["W"]); c[3] = h["seq"] & 0xFFFFFF; c[4] = _u(h["op"]); c[5] = _u(h["ot"])
        c[6] = _u(h["ground"])
        for k in range(3):
            c[8 + k] = _u(h["T"][k]); c[11 + k] = _u(h["R2"][k])
        self.be.call(self, c, self.res)
        self.calls[{CMD_INIT: "init", CMD_ASSOC: "assoc", CMD_EXEC: "exec"}[kind]] += 1
        st = int(self.res[0])
        if st & 0xFF:
            raise Unsupported("core error 0x%x" % (st & 0xFF))
        recs = []
        for k in range(int(self.res[1])):
            w0, d = int(self.res[RES_PAIRS + 2 * k]), int(self.res[RES_PAIRS + 2 * k + 1])
            recs.append((w0 & 127, (w0 >> 7) & 127, (w0 >> 14) & 255, (w0 >> 22) & 1023, d))
        return st, recs, mu

    def _op(self, opc, a=0, b=0, i=0, f=0, p2=0, p3=0):
        self._ops.append([opc | (a << 8) | (b << 16) | (i << 24), _u(f), _u(p2), _u(p3)])

    def _flush(self, extra=(), dump=True):
        """Run the queued ops plus ``extra`` (one EXEC call per NOPS ops); returns (status, records) of the
        last call. The slot dump updates every Track's position and existence, then the deferred PS updates
        that need the new numbers run in order."""
        ops = self._ops + [list(x) for x in extra]
        self._ops = []
        if dump:
            ops.append([OP_DUMP, 0, 0, 0])
        st, recs = 0, []
        while True:
            chunk, ops = ops[:NOPS], ops[NOPS:]
            for k, w in enumerate(chunk):
                self.cmd[OP_BASE + OP_W * k: OP_BASE + OP_W * (k + 1)] = w
            st, recs, mu = self._call(CMD_EXEC, self._hdr["nb"] if self._hdr else 0, len(chunk))
            if not ops:
                break
        if dump:
            for j in range(mu):
                t = self._obj[j]
                if t is None:
                    continue
                d = self.res[RES_DUMP + DUMP_W * j: RES_DUMP + DUMP_W * (j + 1)]
                fl = int(d[4])
                if fl & 512:
                    t.position = np.array([_s(d[0]), _s(d[1]), _s(d[2])], float) / (1 << 18) + self.pos_bias
                t.existence = (_s(d[3]) / float(1 << 24)) if fl & 1 else None
                t._mk_below, t._mk_free = bool(fl & 2), bool(fl & 4)
            fix, self._fix = self._fix, []
            for f in fix:
                f()
        return st, recs

    def _alloc(self, t, typ, label):
        free = [j for j in range(SLOTS) if self._obj[j] is None]
        if not free:
            raise Unsupported("slots")
        j = free[0]
        self._obj[j] = t
        self._slot[t.id] = j
        self._dirty.add(j)
        self._op(OP_NEW, a=j, b=self._class(label), f=typ)
        return j

    def _release(self, t):
        j = self._slot.pop(t.id)
        self._obj[j] = None
        self._dirty.add(j)
        self._op(OP_FREE, a=j)

    # ------------------------------------------------------------------ update_memory
    def update_memory(self, observations, geometry_evidence, frame):
        frame.validate()
        if frame.frame_id in self.seen_frames:
            return {"ignored": "duplicate_frame", "tracks": self.snapshot()}
        if frame.timestamp <= self.last_timestamp:
            raise ValueError("out-of-order acquisition timestamp")
        if any(o.frame_id != frame.frame_id or o.timestamp != frame.timestamp for o in observations):
            raise ValueError("observation belongs to a different acquisition frame")
        if getattr(frame, "pose_sigma_m", None) is not None:
            raise Unsupported("pose_sigma_m")
        self.seen_frames.add(frame.frame_id); self.last_timestamp = frame.timestamp
        self._seq += 1
        rejected = []
        untrusted = self._untrusted(frame)
        pose_valid = frame.pose_valid and not untrusted
        if untrusted:
            rejected += [{"label": o.label, "reason": "pose_untrusted", "position_m": o.position.tolist()}
                         for o in observations]
            observations = []
        self._relocations = []
        seen_boxes = list(observations)
        skip_miss = self._turn_frame(frame, pose_valid)
        if pose_valid:
            self._advance_odometer(frame)
        boxes = seen_boxes if pose_valid else []
        if len(boxes) > NOBS:
            raise Unsupported("observations")
        self._frame_header(frame, pose_valid, True, skip_miss, len(boxes))
        self._oidx = {id(o): i for i, o in enumerate(boxes)}
        self._q0 = {}
        self._assoc_recs, self._assoc_ids = [], list(self._obj)
        self._dirty = set()
        if boxes:
            self._assoc(boxes)
            for i, o in enumerate(boxes):
                why = self._q0[i] & 3
                if why:
                    rejected.append({"label": o.label, "reason": ("below_ground", "size_exceeds_class")[why - 1],
                                     "position_m": o.position.tolist()})
            dups = sorted((i for i in range(len(boxes)) if not (self._q0[i] & 3) and self._q0[i] & 4),
                          key=lambda i: (self._q0[i] >> 8) & 31)
            rejected += [{"label": boxes[i].label, "reason": "same_frame_duplicate",
                          "position_m": boxes[i].position.tolist()} for i in dups]
            observations = [o for i, o in enumerate(boxes) if not (self._q0[i] & 7)]
        elif pose_valid:
            observations = []
        gated = self._create_gated()
        held = []
        self._expire_candidates(frame.timestamp)
        for t in self.tracks.values():
            if t.state in ("current_confirmed", "temporarily_occluded"):
                t.state = "historical_memory"
        self._exist_ctx = (geometry_evidence, True)
        protected, ambiguous, merged = self._associate_v2(observations if pose_valid else [], None, gated,
                                                          rejected, held)
        fell = self._existence_frame(geometry_evidence, frame, protected, boxes, None, pose_valid, untrusted,
                                     skip_miss=skip_miss)
        merged += self._relocate_invalid(fell, frame.timestamp)
        for row in rejected:
            self.rejected[row["reason"]] = self.rejected.get(row["reason"], 0) + 1
        out = {"ignored": None, "ambiguous": ambiguous, "rejected": rejected,
               "geometry": geometry_evidence, "tracks": self.snapshot()}
        if merged:
            out["merged"] = merged
        if self._relocations:
            out["relocated"] = self._relocations
        if gated:
            out["candidates"] = held
        return out

    def _assoc(self, boxes):
        c = self.cmd
        for i, o in enumerate(boxes):
            base = OBS_BASE + OBS_W * i
            r = getattr(o, "range_m", None)
            rk = r is not None and not np.isnan(r)
            ext = getattr(o, "extent_m", None)
            c[base] = self._class(o.label) | (int(bool(getattr(o, "truncated", False))) << 8) | (int(rk) << 9) \
                | (int(ext is not None) << 10)
            c[base + 1] = _u(_q(min(max(o.score, 0.0), 255.0), 24))
            for k in range(3):
                c[base + 2 + k] = _u(_q(o.position[k], 18))
            c[base + 5] = _u(_q(min(float(r), 16.0), 20)) if rk else 0   # core: r <= 16 m (r^2 in 48 bits)
            # extent words are unsigned in the core: a degenerate box (x2 < x1) gives a negative extent, which
            # never exceeds the class limit in Python; clamping it to 0 keeps that (unclamped it wrapped to
            # 2^32 - k and read as oversized -- D3 seed 2 frame 411)
            c[base + 6] = _u(_q(min(max(ext[0], 0.0), 60000.0), 16)) if ext is not None else 0
            c[base + 7] = _u(_q(min(max(ext[1], 0.0), 60000.0), 16)) if ext is not None else 0
            for k in range(4):
                c[base + 8 + k] = _u(_q(o.bbox[k], 8))
        st, recs, _ = self._call(CMD_ASSOC, len(boxes), 0)
        self._q0 = {i: int(self.res[RES_OBS + RES_OBS_W * i]) for i in range(len(boxes))}
        self._assoc_recs = recs

    def _qi(self, o):
        return self._oidx[id(o)]

    def _qbit(self, o, bit):
        return bool(self._q0[self._qi(o)] & bit)

    # ------------------------------------------------------------------ association (control of core.Memory)
    def _associate_v2(self, observations, sigma, gated, rejected, held):
        kept = []
        for o in observations:
            if self._qbit(o, 64):
                rejected.append({"label": o.label, "reason": "beyond_range", "position_m": o.position.tolist()})
            else:
                kept.append(o)
        observations = kept
        idx = {self._qi(o): n for n, o in enumerate(observations)}
        pairs = []
        for q, j, tag, f, d in self._assoc_recs:
            t = self._assoc_ids[j]
            if q in idx and f & PF_GATE and not f & PF_CAND and t is not None and t.id in self.tracks:
                pairs.append((d, idx[q], t.id))
        pairs.sort()
        assigned, used = {}, set()
        for o in observations:
            j = (self._q0[self._qi(o)] >> 14) & 127
            if j != 127:
                assigned[idx[self._qi(o)]] = self._assoc_ids[j].id
                used.add(self._assoc_ids[j].id)
        for k in used:
            self.tracks[k].covis |= used - {k}
        seen_now = set(used)
        protected, ambiguous = set(), []
        for i, o in enumerate(observations):
            near = [k for _, j, k in pairs if j == i]
            protected.update(near)
            if i in assigned:
                t = self.tracks[assigned[i]]
                if len(near) > 1:
                    ambiguous.append({"frame_id": o.frame_id, "candidate_ids": near, "resolved": t.id})
                    self.identity_stats["resolved_ambiguous"] += 1
                if not self._far_flag(t, o):
                    self._touch(t, o, sigma)
                    continue
                if self._far_reading(t, o, sigma, seen_now, held, rejected) is not None:
                    continue
            if not self._qbit(o, 16):
                rejected.append({"label": o.label, "reason": "clipped_box_cannot_create",
                                 "position_m": o.position.tolist()})
                continue
            if self._qbit(o, 32):
                rejected.append({"label": o.label, "reason": "too_far_to_create", "position_m": o.position.tolist()})
                continue
            c = None
            if gated:
                if not self._qbit(o, 128):
                    rejected.append({"label": o.label, "reason": "score_below_create",
                                     "position_m": o.position.tolist()})
                    continue
                c = self._candidate_for(o, sigma)
                c.covis |= seen_now
                if not self._candidate_ready(c):
                    held.append({"candidate_id": c.id, "label": o.label, "hits": len(c.history),
                                 "position_m": o.position.tolist()})
                    continue
                del self.candidates[c.id]
                self.candidate_stats["promoted"] += 1
            t = self._promote(o, c, seen_now, sigma, rejected)
            if t is not None:
                protected.add(t.id)
                for k in seen_now - {t.id}:
                    t.covis.add(k); self.tracks[k].covis.add(t.id)
                seen_now.add(t.id)
        self._protected_now = protected
        return protected, ambiguous, self._merge_pass()

    def _far_flag(self, t, o):
        j, q = self._slot[t.id], self._qi(o)
        for qq, jj, tag, f, d in self._assoc_recs:
            if qq == q and jj == j:
                return bool(f & PF_FAR)
        raise AssertionError("assigned pair without a record")

    def _touch(self, t, o, sigma, append=True):
        if append:
            t.history.append({"frame_id": o.frame_id, "timestamp": o.timestamp, "bbox": list(o.bbox),
                              "score": o.score, "source": o.source, "position_m": o.position.tolist(),
                              "valid_depth_fraction": o.valid_fraction})
        j = self._slot[t.id]
        self._op(OP_TOUCH, a=j, i=self._qi(o), f=1 if append else 0)
        self._dirty.add(j)

        def bounds():
            shift = t.position - o.position
            t.lower = o.lower + shift; t.upper = o.upper + shift
        self._fix.append(bounds)
        t.color = o.color
        t.odo = self.odometer; t.pose_sigma = sigma
        t.last_seen = o.timestamp; t.last_seen_frame = o.frame_id
        t.state = "current_confirmed"; t.reason = "new_visual_observation"; t.negative_poses.clear()
        self._observed(t, o, False)

    def _observed(self, t, o, far, weak=False):
        # deferred: offset_m needs the position after this frame's ops (display only)
        self._fix.append(lambda: Memory._observed(self, t, o, far, weak))

    def _far_reading(self, t, o, sigma, seen_now, held, rejected):
        cfg = self.cfg
        if not (self._qbit(o, 16) and self._qbit(o, 128) and not self._qbit(o, 32)):
            self._observed(t, o, True, weak=True)
            rejected.append({"label": o.label, "reason": "far_reading_too_weak_to_move", "track_id": t.id,
                             "position_m": o.position.tolist()})
            return t
        self._observed(t, o, True)
        c = self._candidate_for(o, sigma)
        c.covis |= seen_now - {t.id}
        if c.move_of not in (None, t.id) and c.move_of in self.tracks:
            return None
        first = c.history[0]["timestamp"] if c.history else o.timestamp
        if t.last_seen >= first or t.id in c.covis:
            c.move_of = None
            return None
        c.move_of = t.id
        t.reason = "seen_elsewhere"
        if len(c.history) < cfg.move_confirm_hits:
            held.append({"candidate_id": c.id, "label": o.label, "hits": len(c.history), "move_of": t.id,
                         "position_m": o.position.tolist()})
            return t
        del self.candidates[c.id]
        self.candidate_stats["moved"] = self.candidate_stats.get("moved", 0) + 1
        return self._take_over(t, o, c, sigma, "far_association")

    def _candidate_for(self, o, sigma):
        q = self._qi(o)
        if any(self._slot[c.id] in self._dirty for c in self.candidates.values()):
            _, recs = self._flush([(OP_QUERY | (q << 24), 0 | (2 << 2) | (1 << 8), 0, 0)])
            recs = [(r[1], r[3], r[4]) for r in recs if r[2] == 1]
            dist = {self._obj[j].id: (f, d) for j, f, d in recs if self._obj[j] is not None}
        else:
            dist = {}
            for qq, j, tag, f, d in self._assoc_recs:
                t = self._assoc_ids[j]
                if qq == q and f & PF_CAND and t is not None:
                    dist[t.id] = (f, d)
        best, best_d = None, None
        for c in self.candidates.values():
            if c.label != o.label or c.id not in dist:
                continue
            f, d = dist[c.id]
            if f & PF_RADIUS and (best is None or d < best_d):
                best, best_d = c, d
        if best is None:
            self._candidate_serial += 1
            self.candidate_stats["started"] += 1
            best = Track(f"candidate-{self._candidate_serial:04d}", o.label, o.color, o.position.copy(),
                         o.lower.copy(), o.upper.copy(), o.timestamp, state="candidate", reason="creation_gate")
            self.candidates[best.id] = best
            self._alloc(best, TYPE_CAND, o.label)
        elif best.last_seen_frame == o.frame_id:
            return best
        best.position = o.position.copy(); best.lower = o.lower.copy(); best.upper = o.upper.copy()
        best.color = o.color; best.last_seen = o.timestamp; best.last_seen_frame = o.frame_id
        best.odo = self.odometer; best.pose_sigma = sigma
        best.history.append({"frame_id": o.frame_id, "timestamp": o.timestamp, "bbox": list(o.bbox),
                             "score": o.score, "source": o.source, "position_m": o.position.tolist(),
                             "valid_depth_fraction": o.valid_fraction})
        j = self._slot[best.id]
        self._op(OP_CAND, a=j, i=q)
        self._dirty.add(j)
        return best

    def _expire_candidates(self, now):
        for key in [k for k, c in self.candidates.items() if now - c.last_seen > self.cfg.candidate_ttl_s]:
            self._release(self.candidates.pop(key))
            self.candidate_stats["expired"] += 1

    def _near_records(self, o, c):
        """id -> (flags, d) for the tracks of o's class, measured from the promotion position (the observation,
        or the candidate's fused position after this frame's updates)."""
        if c is None:
            q = self._qi(o)
            return {self._assoc_ids[j].id: (f, d) for qq, j, tag, f, d in self._assoc_recs
                    if qq == q and not f & PF_CAND and self._assoc_ids[j] is not None}
        a = self._slot[c.id]
        _, recs = self._flush([(OP_QUERY | (a << 8) | (self._qi(o) << 24), 1 | (1 << 2) | (2 << 8), 0, 0)])
        return {self._obj[j].id: (f, d) for qq, j, tag, f, d in recs if tag == 2 and self._obj[j] is not None}

    def _promote(self, o, c, used, sigma, rejected):
        cfg = self.cfg
        covis = set(used) if c is None else set(c.covis) | set(used)
        rec = self._near_records(o, c)
        pos = o.position if c is None else c.position
        near = sorted((rec[t.id][1], t.id) for t in self.tracks.values()
                      if t.label == o.label and t.id not in covis and t.id in rec and rec[t.id][0] & PF_EXCL)
        if near:
            tn = self.tracks[near[0][1]]
            first = c.history[0]["timestamp"] if c is not None and c.history else o.timestamp
            if tn.last_seen >= first:
                rejected.append({"label": o.label, "reason": "near_existing_track", "track_id": tn.id,
                                 "position_m": o.position.tolist()})
                self.identity_stats["suppressed_near"] += 1
                self._drop_candidate(c)
                return None
            return self._take_over(tn, o, c, sigma, "near")
        gone = sorted((rec[t.id][1], t.id) for t in self.tracks.values()
                      if t.label == o.label and t.state == "position_invalid" and t.id not in covis)
        if gone:
            return self._take_over(self.tracks[gone[0][1]], o, c, sigma, "position_invalid")
        if cfg.one_per_class:
            same = [t for t in self.tracks.values() if t.label == o.label]
            if same:
                far = sorted((rec[t.id][1], t.id) for t in same if t.id not in covis)
                if not far:
                    rejected.append({"label": o.label, "reason": "one_per_class_covisible",
                                     "track_id": min(t.id for t in same), "position_m": o.position.tolist()})
                    self._count("suppressed_one_per_class")
                    self._drop_candidate(c)
                    return None
                tn = self.tracks[far[0][1]]
                first = c.history[0]["timestamp"] if c is not None and c.history else o.timestamp
                if tn.last_seen >= first:
                    rejected.append({"label": o.label, "reason": "one_per_class_existing_still_seen",
                                     "track_id": tn.id, "position_m": o.position.tolist()})
                    self._count("suppressed_one_per_class")
                    self._drop_candidate(c)
                    return None
                return self._take_over(tn, o, c, sigma, "one_per_class")
        key = self._new_id()
        t = Track(key, o.label, o.color, pos.copy(), o.lower.copy(), o.upper.copy(), o.timestamp,
                  history=[] if c is None else list(c.history), created_at=o.timestamp,
                  number=self._next_number(o.label))
        t.covis = {k for k in covis if k in self.tracks}
        for k in t.covis:
            self.tracks[k].covis.add(key)
        self.tracks[key] = t
        j = self._alloc(t, TYPE_TRACK, o.label)
        if c is not None:
            self._op(OP_COPY_SAMPLES, a=j, b=self._slot[c.id])
        self._touch(t, o, sigma, append=c is None)
        self._drop_candidate(c)
        return t

    def _drop_candidate(self, c):
        """The promoted candidate (already removed from self.candidates) gives its slot back."""
        if c is not None and c.id in self._slot:
            self._release(c)

    def _take_over(self, tn, o, c, sigma, why):
        before = tn.position.copy()
        j = self._slot[tn.id]
        if c is None:
            self._op(OP_CLEAR_SAMPLES, a=j)
        else:
            self._op(OP_COPY_SAMPLES, a=j, b=self._slot[c.id])
            tn.history.extend(c.history)
            tn.covis |= {k for k in c.covis if k in self.tracks and k != tn.id}
            for k in tn.covis:
                self.tracks[k].covis.add(tn.id)
        self._op(OP_RESET_L, a=j)
        self._touch(tn, o, sigma, append=c is None)
        tn.reason = "relocated"
        if why == "near":
            self.identity_stats["relocated"] += 1
        elif why == "far_association":
            self.identity_stats["relocated_far"] = self.identity_stats.get("relocated_far", 0) + 1
        elif why == "one_per_class":
            self._count("relocated_one_per_class")
        else:
            self.identity_stats["relocated_invalid"] = self.identity_stats.get("relocated_invalid", 0) + 1
        row = {"track_id": tn.id, "from_m": before.tolist(), "to_m": None, "distance_m": None, "via": why}

        def fill():
            row["to_m"] = tn.position.tolist()
            row["distance_m"] = float(np.linalg.norm(tn.position - before))
        self._fix.append(fill)
        self._relocations.append(row)
        self._drop_candidate(c)
        return tn

    # ------------------------------------------------------------------ merge pass and existence
    def _exist_ops(self, geometry_evidence, protected, detections_known):
        words = [0] * 8
        for key, t in self.tracks.items():
            j = self._slot[key]
            if not detections_known and key not in geometry_evidence:
                code = 0
            else:
                ev = geometry_evidence.get(key, "unknown")
                code = 2 if ev == "free" else 3 if ev == "surface_or_uncertain" else 1
            g = code | (4 if key in protected else 0) | (8 if t.state == "position_invalid" else 0)
            words[j >> 3] |= g << (4 * (j & 7))
        self.cmd[GEO_BASE:GEO_BASE + 8] = words
        return [(OP_EXIST, 0, 0, 0)]

    def _merge_pass(self, protected=None):
        """One call normally: merge scan (stop when it finds same-class pairs in the merge gate), then this
        frame's existence step and the slot dump. Pairs seen together are filtered here (co-visibility)."""
        cfg, out = self.cfg, []
        while True:
            ctx = getattr(self, "_exist_ctx", None)
            extra = [(OP_MERGE_SCAN, 1 | (3 << 8), 0, 0)]
            if ctx is not None:
                extra += self._exist_ops(ctx[0], self._protected_now, ctx[1])
            st, recs = self._flush(extra)
            stopped = bool(st & 256)
            if not stopped and ctx is not None:
                self._exist_done = True
            order = {k: n for n, k in enumerate(self.tracks)}
            groups = {}
            for t in self.tracks.values():
                groups.setdefault(t.label, len(groups))
            best = None
            for x, y, tag, f, d in recs:
                if tag != 3:
                    continue
                a, b = self._obj[x], self._obj[y]
                if order[a.id] > order[b.id]:
                    a, b = b, a
                if b.id in a.covis or a.id in b.covis:
                    continue
                k = (d, groups[a.label], order[a.id], order[b.id])
                if best is None or k < best[0]:
                    best = (k, a, b)
            if best is None:
                if stopped and ctx is not None:
                    self._exist_done = False          # scan stopped the call before the existence step
                return out
            (d, *_), a, b = best
            keep, drop = self._survivor(a, b)
            self._merge(keep, drop)
            out.append({"kept": keep.id, "merged": drop.id, "distance_m": d / float(1 << 20)})

    def _merge(self, keep, drop):
        keep.history = sorted(keep.history + drop.history, key=lambda h: h.get("timestamp", 0.0))
        newer = drop.last_seen > keep.last_seen
        if newer:
            for name in ("last_seen", "last_seen_frame", "state", "reason", "odo", "pose_sigma", "color",
                         "lower", "upper"):
                setattr(keep, name, getattr(drop, name))
            keep.negative_poses = list(drop.negative_poses)
        old = keep.position.copy()
        self._op(OP_MERGE, a=self._slot[keep.id], b=self._slot[drop.id], f=1 if newer else 0)

        def shift():
            keep.lower = keep.lower + (keep.position - old); keep.upper = keep.upper + (keep.position - old)
        self._fix.append(shift)
        self._retire(keep, drop)
        self.identity_stats["merged"] += 1

    def _retire(self, keep, drop):
        Memory._retire(self, keep, drop)
        self._release(drop)

    def _existence_frame(self, geometry_evidence, frame, protected, boxes, sigma, pose_valid, untrusted,
                         detections_known=True, skip_miss=False):
        if not getattr(self, "_exist_done", False):
            self._flush(self._exist_ops(geometry_evidence, protected, detections_known))
        self._exist_done = False
        self._exist_ctx = None
        fell = []
        if not self.tracks:
            return fell
        for key, t in self.tracks.items():
            if getattr(t, "_mk_below", False):
                fell_now = t.state != "position_invalid"
                t.state = "position_invalid"
                t.reason = "free" if getattr(t, "_mk_free", False) else "not_detected"
                if fell_now:
                    fell.append(t)
            if key in protected:
                continue
            if key not in geometry_evidence and not detections_known:
                continue
            ev = geometry_evidence.get(key, "unknown")
            sticky = t.state == "position_invalid"
            if not pose_valid:
                if not sticky:
                    t.reason = "pose_untrusted" if untrusted and frame.pose_valid else "invalid_pose"
                continue
            if not sticky:
                t.reason = ev
            if ev == "occluded" and not sticky:
                t.state = "temporarily_occluded"
        return fell

    def _relocate_invalid(self, fell, now):
        cfg, out = self.cfg, []
        for x in sorted(fell, key=lambda t: t.id):
            if x.id not in self.tracks or x.state != "position_invalid":
                continue
            ok = [y for y in self.tracks.values()
                  if y.label == x.label and y.id != x.id and y.state != "position_invalid" and
                  y.created_at is not None and now - y.created_at <= cfg.relocate_window_s and
                  (x.created_at is None or y.created_at > x.created_at) and
                  y.id not in x.covis and x.id not in y.covis]
            if not ok:
                continue
            _, recs = self._flush([(OP_QUERY | (self._slot[x.id] << 8), 2 | (1 << 2) | (4 << 8), 0, 0)])
            dist = {self._obj[j].id: d for q, j, tag, f, d in recs if tag == 4}
            ys = sorted((dist[y.id], y.id) for y in ok)
            d, yid = ys[0]
            keep, drop = self._relocate_merge(x, self.tracks[yid])
            out.append({"kept": keep.id, "merged": drop.id, "distance_m": d / float(1 << 20), "reason": "relocated"})
        if self._ops or self._fix:
            self._flush()
        return out

    def _relocate_merge(self, old, new):
        keep, drop = self._survivor(old, new)
        if keep is not new:
            for name in ("last_seen", "last_seen_frame", "state", "odo", "pose_sigma", "color"):
                setattr(keep, name, getattr(new, name))
            self._op(OP_COPY_PLACE, a=self._slot[keep.id], b=self._slot[new.id])
            keep.position = new.position.copy(); keep.lower = new.lower.copy(); keep.upper = new.upper.copy()
            keep.existence = new.existence
            keep.negative_poses = []
        keep.reason = "relocated"
        keep.history = sorted(keep.history + drop.history, key=lambda h: h.get("timestamp", 0.0))
        if keep.created_at is None or (drop.created_at is not None and drop.created_at < keep.created_at):
            keep.created_at = drop.created_at
        self._retire(keep, drop)
        self.identity_stats["relocated_merged"] = self.identity_stats.get("relocated_merged", 0) + 1
        return keep, drop

    def update_geometry_only(self, geometry_evidence, frame):
        frame.validate()
        if frame.frame_id in self.geometry_seen_frames:
            return {"ignored": "duplicate_frame", "tracks": self.snapshot()}
        if frame.timestamp <= self.last_geometry_timestamp:
            raise ValueError("out-of-order geometry timestamp")
        geometry_evidence = {k: v for k, v in geometry_evidence.items() if k not in self.merged}
        unknown = set(geometry_evidence) - set(self.tracks)
        if unknown:
            raise ValueError("geometry evidence for unknown tracks")
        self.geometry_seen_frames.add(frame.frame_id)
        self.last_geometry_timestamp = frame.timestamp
        untrusted = self._untrusted(frame)
        self._relocations = []
        pv = frame.pose_valid and not untrusted
        self._frame_header(frame, pv, False, False, 0)
        fell = self._existence_frame(geometry_evidence, frame, set(), [], None, pv, untrusted, detections_known=False)
        out = {"ignored": None, "geometry": dict(geometry_evidence), "tracks": None}
        merged = self._relocate_invalid(fell, frame.timestamp)
        out["tracks"] = self.snapshot()
        if merged:
            out["merged"] = merged
        return out

    def snapshot(self):
        if self._ops or self._fix:
            self._flush()
        return [t.as_dict() for t in self.tracks.values()]

    def _fuse(self, t):
        raise AssertionError("PLMemory fuses in the core")
