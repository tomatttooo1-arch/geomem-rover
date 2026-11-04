"""Metric RGB-D geometry and conservative object memory. No ROS or torch required."""
from dataclasses import dataclass, field
from typing import Optional
import re
import numpy as np

from perception.fpga_replay import coco_zh, memory_gates


@dataclass(frozen=True)
class Config:
    association_m: float = 0.25
    depth_margin_m: float = 0.04
    min_valid_fraction: float = 0.7
    min_pixels: int = 9
    free_fraction: float = 0.9
    independent_translation_m: float = 0.08
    independent_angle_deg: float = 8.0
    independent_seconds: float = 0.25
    negative_views: int = 2
    # 10-16 memory quality (docs/design/MEMORY_QUALITY_20261016.md); memory_quality=False
    # restores the earlier behaviour exactly (fixed association_m, no admission gates).
    memory_quality: bool = True
    drift_per_m: float = 0.03          # radius growth per metre travelled since the track's last update
    drift_per_rad: float = 0.05        # radius growth per radian turned (5 % heading error x 1 m range)
    association_max_m: float = 0.8    # cap; 1.25 x the largest drift measured on the 10-16 real run
    pose_sigma_gate: float = 2.0       # used instead of the odometry terms when frames carry pose_sigma_m
    ground_z_m: float = 0.0            # world z of the ground when the frame does not say (frame.ground_z_m)
    below_ground_tol_m: Optional[float] = 0.03
    size_margin: Optional[float] = 1.5
    edge_margin_px: float = 2.0
    dedupe_m: float = 0.10
    # 10-17 pose trust (docs/design/MEMORY_POSE_TRUST_20261017.md): a frame the runner marked
    # frame.pose_trusted = False (wheel vs gyro yaw disagree) stores nothing; frames without the
    # attribute, or pose_trust_gate=False, behave exactly as before. Independent of memory_quality.
    pose_trust_gate: bool = True
    # D-044 revised (2026-10-21) creation gate, same for every class: an observation that matches no
    # track may start a track only with score >= create_min_score; it first becomes a candidate
    # (Memory.candidates: not in tracks, so not in the snapshot, the PL ROI plan, the kiosk or the
    # store) and is promoted to a track once it has create_min_hits observations in distinct frames
    # spanning >= create_min_span_s seconds. A candidate unseen for candidate_ttl_s is dropped.
    # Updates of existing tracks are not gated. create_min_hits <= 1, create_min_span_s <= 0 and
    # create_min_score <= 0 restore single-observation creation exactly; memory_quality=False also
    # bypasses the gate. Values: runs/coco-open-20261021-results README, second leg.
    # Default (S, N, T, TTL) = (0, 3, 2 s, 3 s): 10-16 classroom sweep, all three bottles kept 2 s later,
    # the 9 hand-marked false tracks gone (0 left), chairs 20 -> 7. T is in seconds so it carries to the
    # board's ~10 Hz; N counts frames (1 Hz there, so N=3 and T=2 coincide on that recording).
    create_min_score: float = 0.0
    create_min_hits: int = 3
    create_min_span_s: float = 2.0
    candidate_ttl_s: float = 3.0
    # classes the runner's find text may name (D-044 revised: the default detector set); labels of
    # existing tracks are always added. Words of other classes are ignored, so 马上 is not "horse".
    query_classes: tuple = coco_zh.VENUE_CLASSES
    # 10-27 object identity (docs/design/OBJECT_IDENTITY_20261027.md). Real-car logs 10-26: one bottle
    # became several tracks because (1) a single see-through depth reading (+0.24 m) or far-range jitter
    # moved the track (position = last observation) and the next reading fell outside 0.25 m, (2) the
    # outlier readings formed their own candidate and were promoted, (3) twins froze each other as
    # ambiguous and were never merged. identity_v2=False restores the earlier behaviour exactly.
    identity_v2: bool = True
    near_surface_fraction: float = 0.2  # depth: nearest 5 cm bin pair holding >= this share of valid pixels
    obs_sigma0_m: float = 0.02         # observation sd = sigma0 + k * range^2 (Gemini 2: <= 2 % at 2 m)
    obs_sigma_k: float = 0.01
    gate_chi: float = 2.45             # sqrt(chi2 2-dof 95 % = 5.99), as in DeepSORT
    fuse_keep: int = 20                # observations kept per track for the robust position
    fuse_inlier_m: float = 0.10        # inlier: within max(this, 3 x 1.4826 MAD) of the median
    obs_max_range_m: float = 3.0       # farther observations are dropped (10-26 log: 0.2-0.3 m jitter at 3.5 m)
    create_max_range_m: float = 2.0    # farther observations update but never start a track (range table 1.6 m)
    exclusion_m: float = 0.6           # no second same-class instance this close unless both were seen in one frame
    merge_max_m: float = 0.6           # two never-co-visible same-class tracks this close are merged (= exclusion)
    dedupe_iou: float = 0.3            # same-class boxes overlapping more are one object (co-visible means apart)
    # 10-27 existence score (docs/design/MEMORY_EXISTENCE_20261027.md, owner approved steps 1+2). Car tests
    # 10-27: an object taken away in front of a parked car, or found gone on return, never left memory --
    # free needed two views >= 8 cm / 8 deg apart (a parked car never gets them), one surface_or_uncertain
    # frame erased the free views, and "should have seen it but did not" counted for nothing. Now every
    # track keeps a log-odds score updated by one vote per existence_window_s of acquisition time (no
    # camera motion needed); <= invalid_below -> position_invalid, a detection brings it back. Step 2: a
    # track that turns invalid absorbs a recent never-co-visible same-class track (the object moved), and
    # a promoted candidate takes over an invalid same-class track, so the display number survives a move.
    # existence_v3=False restores the earlier behaviour exactly (negative_views / _independent).
    existence_v3: bool = True
    # 10-31 owner: at the defence every class is on the field exactly once. A banana moved 2 m (beyond the
    # exclusion zone, old place not yet judged empty) became "banana 2"; with one_per_class a promoted candidate
    # of a class that already has a track takes that track over instead (id and number kept, place = the new
    # one). Downside: a second same-class object (two cups in a test, or a co-visible false detection) never
    # enters memory -- a candidate seen in one frame with the existing track, or while the existing track is
    # still being seen at its old place, is dropped. Turn this off to test several instances of one class
    # (runtime --one-per-class off / car_up CAR_ONE_PER_CLASS=0). False = the behaviour before, unchanged.
    one_per_class: bool = True
    existence_min: float = -3.0        # clamps (OctoMap-style): a long-seen object can still leave in ~1-2 s
    existence_max: float = 3.5
    existence_init: float = 1.5        # at promotion (the candidate already had 3 hits over 2 s): 2 windows
    existence_window_s: float = 0.3    # one vote per track per window (car 10-27: 0.134 s median frame gap)
    existence_hit: float = 2.0         # window with a detection associated to the track
    existence_free: float = -1.5       # window with geometry free and no same-class box in the track's gate
    invalid_below: float = -1.5        # L <= this -> position_invalid
    miss_weight: float = 0.5           # missed detection: L += miss_weight * ln(1 - P_D)
    miss_max_range_m: float = 1.7      # farther (optical-axis depth) misses are not counted
    miss_pd_cap: float = 0.9
    # P_D range curve, piecewise linear (range m, factor); 10-27 car logs: classes found reliably to ~1.2 m
    miss_range_curve: tuple = ((1.2, 1.0), (1.5, 0.64), (2.0, 0.0))
    # P_D class factor: detection rate with the box in view, depth valid, unoccluded (10-26/10-27 car logs,
    # a lower bound: far and already-removed objects are mixed in); classes not listed get miss_class_default
    miss_class_pd: tuple = (("backpack", 1.0), ("glasses", 1.0), ("bottle", 1.0), ("chair", 0.9),
                            ("medicine bottle", 0.3), ("cell phone", 0.15), ("remote", 0.05),
                            ("plastic packaging", 0.05), ("handbag", 0.05))
    miss_class_default: float = 0.5
    # A window whose only negative is a miss with P_D < miss_decisive_pd lowers L at most to miss_floor
    # (above invalid_below): with P_D 0.15 a phone would otherwise fall after ~62 windows (~19 s) of
    # staring, and misses of one viewpoint are not independent. Such misses only prime the score; free or a
    # likely miss decides. (10-27, deviation from the plan's pure sum; see the design note.)
    miss_decisive_pd: float = 0.5
    miss_floor: float = -1.0
    # 10-27 turn gate (owner approved; design note "转弯漏检"): host simulation had a bottle in full view
    # invalidated twice at the same bend -- ~1 s of missed detections while the car turned (blurred frames),
    # found again ~1 s later. A missed detection is no evidence while the camera turns faster than
    # miss_max_turn_dps (rate from consecutive valid poses, acquisition timestamps), nor for miss_turn_hold_s
    # after (blur / pose lag); a frame gap > miss_turn_max_gap_s gives no rate. Hits and free count as before.
    # False = no turn gate (6d76a3d exactly).
    miss_turn_gate: bool = True
    miss_max_turn_dps: float = 25.0
    miss_turn_hold_s: float = 0.3
    miss_turn_max_gap_s: float = 1.0
    relocate_window_s: float = 60.0    # a track created this recently may be the moved one (step 2)
    # ---- 10-27 geometry v2 (docs/design/GEOMETRY_V2_20261027.md; owner 10-27, real-car A/B before it becomes
    # the default). False = compute_geometry / the KV ROI plan / the Nano word exactly as before. True: probe the
    # object's own surface, not the box's far face (Panoptic Multi-TSDFs, Khronos): the box without its observation
    # pad, shrunk about its centre, its bottom >= ground + clearance; behind = expected surface depth + max(margin,
    # 3 sigma(d)); free when >= free_fraction_v2 of the valid pixels are behind. Synthetic 10-27 (camera 0.39 m,
    # 7 deg down, bottle at 1 m): v1 sees only 67 % of a removed floor bottle's box behind (needs 90 %).
    geometry_v2: bool = False
    probe_shrink: float = 0.6          # keep the central 60 % of each probe extent (silhouette edges, neck)
    probe_pad_m: float = 0.03          # = the pad process_frame adds around the observed surface quantiles
    probe_ground_clear_m: float = 0.03  # probe only >= 3 cm above the ground (floor pixels stay out)
    probe_margin_m: float = 0.04       # behind margin floor = v1's depth_margin_m
    occlusion_margin_m: float = 0.08   # nearer than surface - 8 cm is an occluder (hand, another object)
    free_fraction_v2: float = 0.5      # Khronos-style: a fixed share of see-through pixels is a conflict
    see_through_conflicts: int = 2     # see_through.SeeThroughFilter: detected-and-free this often -> no more free
    # 10-27 candidate merge: the filter runs whenever existence_v3 (v1 or v2 geometry) or geometry_v2 is on; the
    # 10-27 car log replay had 13 of 16 existence invalidations decided by v1 free on flat small objects that were
    # still being detected. False = the filter never changes a word.
    see_through_filter: bool = True
    # 10-27 sustained free (owner approved; see_through.SustainedFree): the filter's block on a track's free
    # was permanent, so a blocked low-P_D object (phone: misses stop at miss_floor) that is really taken away
    # never left memory. A blocked track's free counts again when, over the last sustained_free_window_s, it
    # had >= sustained_free_min_windows in-view vote windows, >= sustained_free_fraction of them raw free, and
    # no detection at all. Real-car logs (10-26/10-27): the longest false free run with the object still there
    # was ~3.8 s (plastic packaging, 27 frames), then glasses / medicine bottle ~2.1 s -- 5 s clears them, with
    # a small margin. False = the block is permanent (as before).
    sustained_free_override: bool = True
    sustained_free_window_s: float = 5.0
    sustained_free_min_windows: int = 6
    sustained_free_fraction: float = 0.8
    # 10-27 night, fast removal (owner: "taken away in front of the car, gone from memory too slowly"). Car log
    # 20261027-205546: a key and a drink can (class factor 0.5 -> each miss window only -0.35) took 9.6 s / 8.1 s
    # from the last detection to position_invalid (15 miss windows from 3.5). The class factor is a prior; a
    # track the detector has just been finding in >= miss_reliable_rate of its in-view vote windows (over
    # miss_reliable_window_s before its last detection, >= miss_reliable_min_windows windows) is reliably
    # detectable HERE, so while that detection is recent (<= miss_reliable_recent_s) each of its decisive misses
    # (P_D >= miss_decisive_pd: near, not a low-P_D class) votes at least miss_reliable_vote (= a P_D 0.9 miss,
    # 5 windows from 3.5). Replay of 7 car logs: removal 9.6 -> 3.4 s / 8.1 -> 2.6 s, invalidations re-detected
    # within 30 s 78 -> 79. Stronger votes (-1.7: 3 windows, -2.5: 2) were faster but +10 / +18 of those (design
    # note FAST_REMOVAL_20261027). False = 036f82a exactly.
    miss_reliable: bool = True
    miss_reliable_window_s: float = 3.0
    miss_reliable_min_windows: int = 4
    miss_reliable_rate: float = 0.8
    miss_reliable_recent_s: float = 5.0
    miss_reliable_vote: float = -1.15
    # ---- 10-28 move by far association (owner approved; car run 20261028-164456-live): a banana moved 0.64 m was
    # seen once at the new place, associated to its track (drift-grown gate) and taken as a confirmation of the OLD
    # place -- the robust fusion dropped the far reading as an outlier, the find ended found and the approach drove to
    # the empty old place; the next readings started a second banana. Now an associated observation farther than
    # max(move_far_m, gate_chi x sd(obs, fused)) from the track's fused position is not a confirmation (no
    # current_confirmed, no odometer / drift-gate reset, no existence hit): it feeds a candidate (the creation pool,
    # same fusion) marked as "this track moved"; move_confirm_hits distinct frames of it (candidate_ttl_s keeps them
    # close in time) move the track there through _take_over -- id and number kept, old samples dropped -- unless the
    # old place was confirmed after the first far reading (then the candidate is an ordinary creation candidate).
    # Creation gates (clipped box, score, range) do not apply to a move. False = 06ab8ac exactly.
    move_v2: bool = True
    move_far_m: float = 0.25           # = association_m; > 3 sd of two readings to 1.8 m (0.02 + 0.01 r^2)
    move_confirm_hits: int = 3
    # ---- 10-28 tighter creation (owner approved): per-class minimum score to START a track (updates are not
    # gated); max(create_min_score, the class value). banana: the 20261028 car run's false banana (box on the top
    # image edge, z 0.46 m) scored 0.25-0.39, the real one 0.45-0.77 when fully in view.
    create_min_score_by_class: tuple = (("banana", 0.4),)


# Config fields that switch the D-044 creation gate off (single-observation creation, the behaviour
# before 2026-10-21): dataclasses.replace(cfg, **UNGATED_CREATION). For unit tests and known-answer
# replays whose expected values were written before the gate existed; the runner keeps the gate on.
UNGATED_CREATION = dict(create_min_score=0.0, create_min_hits=1, create_min_span_s=0.0)


# Largest colour/depth stamp tolerance a Frame accepts (owner 10-29): host/ros_runtime.camera_pairing gives
# 0.5 / fps + 0.005 s below 30 fps and caps it here, so every --camera-fps (> 0) yields a value inside (0, 1 s];
# above 1 s is a unit error (ms given as s), not a pairing.
PAIR_TOLERANCE_MAX_S = 1.0


@dataclass
class Frame:
    frame_id: str
    timestamp: float
    rgb: np.ndarray
    depth: np.ndarray  # aligned optical-axis Z, metres; invalid = NaN
    K: np.ndarray
    T_world_camera: np.ndarray
    pose_valid: bool = True
    pose_source: str = "unknown"
    calibration_id: str = "unknown"
    depth_timestamp: Optional[float] = None
    imu: object = None
    encoder: object = None
    # colour/depth stamp tolerance: the runtime's pair slop at its camera rate (host/ros_runtime.camera_pairing; 10-29:
    # 15 fps stamps sat 26.6 / 32.1 ms apart -> 0.038 s); at most PAIR_TOLERANCE_MAX_S
    pair_tolerance_s: float = 0.02

    def validate(self):
        h, w = self.depth.shape
        if self.rgb.shape != (h, w, 3) or self.rgb.dtype != np.uint8:
            raise ValueError("RGB must be uint8 HxWx3, aligned to depth")
        if not np.issubdtype(self.depth.dtype, np.floating):
            raise ValueError("depth must be floating optical-axis Z in metres")
        if not self.frame_id or not np.isfinite(self.timestamp):
            raise ValueError("frame ID and finite acquisition time required")
        if self.K.shape != (3, 3) or not np.isfinite(self.K).all():
            raise ValueError("invalid intrinsics")
        if self.K[0, 0] <= 0 or self.K[1, 1] <= 0 or not np.allclose(self.K[2], [0, 0, 1]):
            raise ValueError("invalid focal lengths or intrinsics last row")
        if self.T_world_camera.shape != (4, 4) or not np.isfinite(self.T_world_camera).all():
            raise ValueError("invalid camera-to-world transform")
        r = self.T_world_camera[:3, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1, atol=1e-5):
            raise ValueError("pose rotation must be a proper orthogonal matrix")
        if not np.allclose(self.T_world_camera[3], [0, 0, 0, 1]):
            raise ValueError("invalid homogeneous transform")
        tol = float(self.pair_tolerance_s)
        if not 0 < tol <= PAIR_TOLERANCE_MAX_S:
            raise ValueError("pair tolerance must be in (0, %.0f] ms" % (PAIR_TOLERANCE_MAX_S * 1000))
        if self.depth_timestamp is not None and abs(self.depth_timestamp - self.timestamp) > tol:
            raise ValueError("RGB/depth timestamp mismatch > {:.0f} ms".format(tol * 1000))
        valid = self.depth[np.isfinite(self.depth) & (self.depth > 0)]
        if valid.size and np.median(valid) > 20:
            raise ValueError("depth outside indoor range; check declared units")


@dataclass
class Detection:
    label: str
    score: float
    bbox: tuple
    source: str = "detector"
    frame_id: str = ""
    timestamp: Optional[float] = None


@dataclass
class Observation:
    label: str
    score: float
    position: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    color: str
    valid_fraction: float
    frame_id: str
    timestamp: float
    bbox: tuple
    source: str
    extent_m: Optional[tuple] = None   # detector box in metres at the observation depth (width, height)
    truncated: bool = False            # detector box touches the image border
    range_m: Optional[float] = None    # optical-axis depth of the observation (None: unknown, no range gates)


def backproject(uv, z, K, T):
    rays = np.c_[np.asarray(uv), np.ones(len(uv))] @ np.linalg.inv(K).T
    camera = rays * np.asarray(z)[:, None]
    return camera @ T[:3, :3].T + T[:3, 3]


def process_frame(frame, detections, cfg=Config()):
    """Extract dominant connected depth surface inside each detection's inner box."""
    import cv2
    frame.validate()
    if not frame.pose_valid:
        return []
    h, w = frame.depth.shape
    out = []
    for det in detections:
        if det.frame_id != frame.frame_id or det.timestamp != frame.timestamp:
            raise ValueError("detection belongs to a different acquisition frame")
        if not np.isfinite(det.bbox).all() or not np.isfinite(det.score):
            raise ValueError("non-finite detection")
        x1, y1, x2, y2 = map(float, det.bbox)
        full_box = (x1, y1, x2, y2)
        dx, dy = (x2-x1)*0.15, (y2-y1)*0.15
        x1, x2 = max(0, int(np.ceil(x1+dx))), min(w, int(np.floor(x2-dx)))
        y1, y2 = max(0, int(np.ceil(y1+dy))), min(h, int(np.floor(y2-dy)))
        if x2 <= x1 or y2 <= y1:
            continue
        patch = frame.depth[y1:y2, x1:x2]
        valid = np.isfinite(patch) & (patch > 0.1) & (patch < 10)
        fraction = float(valid.mean())
        if valid.sum() < cfg.min_pixels or fraction < cfg.min_valid_fraction:
            continue
        values = patch[valid]
        bins = np.floor(values / 0.05).astype(int)
        counts = np.bincount(bins)
        mode_bin = counts.argmax()
        if cfg.identity_v2:
            # Nearest significant surface, not the global mode: through a transparent bottle the wall
            # behind can hold the most pixels (10-26 car log: one reading 0.24 m too far).
            need = cfg.near_surface_fraction * values.size
            for b in np.flatnonzero(counts):
                pair = counts[b] + (counts[b+1] if b+1 < counts.size else 0)
                if pair >= need:
                    mode_bin = b if b+1 >= counts.size or counts[b] >= counts[b+1] else b+1
                    break
        mode = mode_bin*0.05 + 0.025
        mask = valid & (np.abs(patch-mode) < 0.08)
        n, components = cv2.connectedComponents(mask.astype(np.uint8))
        if n <= 1:
            continue
        areas = np.bincount(components.ravel()); areas[0] = 0
        mask = components == areas.argmax()
        if mask.sum() < cfg.min_pixels:
            continue
        v, u = np.where(mask)
        xyz = backproject(np.c_[u+x1, v+y1], patch[mask], frame.K, frame.T_world_camera)
        lower, upper = np.quantile(xyz, [0.05, 0.95], axis=0)
        # Surface extent only; uncertainty pad is a reference-model parameter, not object thickness.
        lower -= 0.03; upper += 0.03
        color_pixels = frame.rgb[y1:y2, x1:x2][mask]
        hsv = cv2.cvtColor(color_pixels.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)
        hue, sat, val = np.median(hsv, axis=0)
        color = ("black" if val < 50 else "white" if sat < 40 and val > 180 else
                 "gray" if sat < 40 else "red" if hue < 10 or hue > 170 else
                 "yellow" if hue < 35 else "green" if hue < 85 else "blue" if hue < 135 else "purple")
        position = np.median(xyz, axis=0)
        camera_z = float(((position - frame.T_world_camera[:3, 3]) @ frame.T_world_camera[:3, :3])[2])
        out.append(Observation(det.label, float(det.score), position, lower,
                               upper, color, fraction, frame.frame_id, frame.timestamp,
                               det.bbox, det.source,
                               extent_m=memory_gates.apparent_extent(full_box, camera_z, frame.K),
                               truncated=memory_gates.is_truncated(full_box, (h, w), cfg.edge_margin_px),
                               range_m=camera_z))
    return out


@dataclass
class Track:
    id: str
    label: str
    color: str
    position: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    last_seen: float
    state: str = "current_confirmed"
    reason: str = "new_visual_observation"
    history: list = field(default_factory=list)
    negative_poses: list = field(default_factory=list)
    last_seen_frame: str = ""
    odo: tuple = (0.0, 0.0)            # memory odometer (path m, turned rad) at the last update
    pose_sigma: Optional[float] = None  # frame pose standard deviation at the last update, if known
    created_at: Optional[float] = None  # acquisition time the track entered Memory.tracks (not exported)
    # 10-27 identity (Config.identity_v2): recent (timestamp, position, sd) for the robust position;
    # class-local display number (瓶子 2), given at promotion, never reused; ids of tracks seen in the
    # same frame by a different box (never merged with); ids merged into this one.
    samples: list = field(default_factory=list)
    number: Optional[int] = None
    covis: set = field(default_factory=set)
    aliases: list = field(default_factory=list)
    fused_sd: Optional[float] = None
    # 10-27 existence (Config.existence_v3; None while it is off): log-odds score and the open vote
    # window -- its index (floor(t / window)), whether a detection already voted in it, free seen in it,
    # and the smallest P_D of a counted miss in it (None: no miss).
    existence: Optional[float] = None
    ex_window: Optional[int] = None
    ex_hit: bool = False
    ex_free: bool = False
    ex_miss_pd: Optional[float] = None
    # 10-27 fast removal (Config.miss_reliable): recent in-view vote windows [(window, 1 hit / 0 miss)], the hit
    # rate over miss_reliable_window_s at the last detection (None: too few windows), that detection's window
    ex_seen: list = field(default_factory=list)
    ex_rate: Optional[float] = None
    ex_last_hit: Optional[int] = None
    # 10-28 (Config.move_v2): the latest observation associated to this track, confirming or far (not exported in
    # the snapshot; query_memory adds it as "observed" so the find can drive to where the object really is)
    observed: Optional[dict] = None
    move_of: Optional[str] = None      # candidates only: the track whose far readings this candidate collects

    def as_dict(self):
        out = {"id": self.id, "label": self.label, "color": self.color,
               "position_m": self.position.tolist(), "bounds_m": [self.lower.tolist(), self.upper.tolist()],
               "last_seen": self.last_seen, "last_seen_frame": self.last_seen_frame,
               "state": self.state, "reason": self.reason, "negative_views": len(self.negative_poses),
               "history": self.history.copy()}
        if self.number is not None:
            out["number"] = self.number
        if self.existence is not None:
            out["existence"] = round(float(self.existence), 3)
        return out


def compute_geometry(frame, memory, cfg=None):
    """Project stored surface volumes: unknown / occluded / surface / free.

    Deliberately conservative: require the entire projected box in view and most
    valid pixels behind its farthest extent before reporting free space.
    cfg None: Config() as before, unless the memory's own Config has geometry_v2 on
    (10-27: the runtime calls compute_geometry(frame, memory) without a cfg).
    """
    frame.validate()
    if cfg is None:
        mcfg = getattr(memory, "cfg", None)
        cfg = mcfg if getattr(mcfg, "geometry_v2", False) else Config()
    if cfg.geometry_v2:
        from perception.fpga_replay import geometry_v2
        return geometry_v2.compute(frame, memory, cfg)
    evidence = {}
    h, w = frame.depth.shape
    for t in memory.tracks.values():
        if not frame.pose_valid:
            evidence[t.id] = "invalid_pose"; continue
        corners = np.array([[x, y, z] for x in (t.lower[0], t.upper[0])
                            for y in (t.lower[1], t.upper[1]) for z in (t.lower[2], t.upper[2])])
        camera = (corners-frame.T_world_camera[:3, 3]) @ frame.T_world_camera[:3, :3]
        if (camera[:, 2] <= 0.1).any():
            evidence[t.id] = "out_of_view"; continue
        uvz = camera @ frame.K.T; uv = uvz[:, :2]/uvz[:, 2:]
        lo = np.floor(uv.min(axis=0)).astype(int); hi = np.ceil(uv.max(axis=0)).astype(int)
        if lo[0] < 0 or lo[1] < 0 or hi[0] >= w or hi[1] >= h:
            evidence[t.id] = "out_of_view"; continue
        if np.any(hi-lo < 3):
            evidence[t.id] = "too_small"; continue
        d = frame.depth[lo[1]:hi[1]+1, lo[0]:hi[0]+1]
        good = np.isfinite(d) & (d > 0.1) & (d < 10)
        if good.sum() < cfg.min_pixels or good.mean() < cfg.min_valid_fraction:
            evidence[t.id] = "invalid_depth"; continue
        d = d[good]
        front = camera[:, 2].min()-cfg.depth_margin_m
        back = camera[:, 2].max()+cfg.depth_margin_m
        if np.mean(d < front) >= 0.5:
            evidence[t.id] = "occluded"
        elif np.mean(d > back) >= cfg.free_fraction:
            evidence[t.id] = "free"
        else:
            evidence[t.id] = "surface_or_uncertain"
    return evidence


_ZH_DIGITS = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def requested_number(text):
    """Class-local number asked for in a find text, or None. Arabic digits anywhere (瓶子2, 瓶子 #2,
    2号瓶子); a Chinese numeral only before 号 or at the very end (二号瓶子, 瓶子二) -- 一个瓶子 is not 1."""
    m = re.search(r"([0-9]+)", text)
    if m:
        return int(m.group(1))
    m = re.search(r"([一二两三四五六七八九十])\s*号", text) or re.search(r"[^一二两三四五六七八九十]([一二三四五六七八九十])\s*$", text)
    return _ZH_DIGITS[m.group(1)] if m else None


class Memory:
    def __init__(self, strategy="geometry", cfg=Config()):
        if strategy not in ("static", "miss", "geometry"):
            raise ValueError("unknown memory strategy")
        self.strategy, self.cfg = strategy, cfg
        self.tracks = {}
        self.seen_frames = set()
        self.last_timestamp = -float("inf")
        # Geometry-only entry keeps its own clock so a late detector batch for
        # the same frame is not mistaken for a duplicate (OBJECT_ROI_SPEC s.6).
        self.geometry_seen_frames = set()
        self.last_geometry_timestamp = -float("inf")
        # Memory-quality state (10-16): odometer from consecutive valid frame poses
        # (path m, turned rad) and rejection counts by reason.
        self.odometer = (0.0, 0.0)
        self._odometer_pose = None
        self.rejected = {}
        # creation-gate candidates (Config.create_*): label-matched, never visible to consumers of tracks
        self.candidates = {}
        self._candidate_serial = 0
        self.candidate_stats = {"started": 0, "promoted": 0, "expired": 0}
        # 10-27 identity: merged id -> surviving id; ids the caller's task refers to (never merged away);
        # last used object serial and class-local display number.
        self.merged = {}
        self.keep_ids = set()
        self._serial = 0
        self._numbers = {}
        self.identity_stats = {"suppressed_near": 0, "relocated": 0, "merged": 0, "resolved_ambiguous": 0}
        self._relocations = []           # 10-27 existence step 2: takeovers of the current update (out["relocated"])
        # 10-27 turn gate (Config.miss_turn_gate): camera turn rate of the last update_memory frame (deg/s, None:
        # no rate) and whether its misses were skipped; previous valid (timestamp, rotation); end of the hold.
        self.turn_dps = None
        self.turn_skip_miss = False
        self._turn_prev = None
        self._turn_hold_until = -float("inf")

    def _independent(self, frame, t):
        for stamp, pose in t.negative_poses:
            distance = np.linalg.norm(frame.T_world_camera[:3, 3]-pose[:3, 3])
            r = frame.T_world_camera[:3, :3].T @ pose[:3, :3]
            angle = np.degrees(np.arccos(np.clip((np.trace(r)-1)/2, -1, 1)))
            if (frame.timestamp-stamp < self.cfg.independent_seconds or
                (distance < self.cfg.independent_translation_m and angle < self.cfg.independent_angle_deg)):
                return False
        return True

    def update_memory(self, observations, geometry_evidence, frame):
        frame.validate()
        if frame.frame_id in self.seen_frames:
            return {"ignored": "duplicate_frame", "tracks": self.snapshot()}
        if frame.timestamp <= self.last_timestamp:
            raise ValueError("out-of-order acquisition timestamp")
        if any(o.frame_id != frame.frame_id or o.timestamp != frame.timestamp for o in observations):
            raise ValueError("observation belongs to a different acquisition frame")
        self.seen_frames.add(frame.frame_id); self.last_timestamp = frame.timestamp
        quality = self.cfg.memory_quality
        rejected = []
        untrusted = self._untrusted(frame)
        pose_valid = frame.pose_valid and not untrusted
        if untrusted:
            rejected += [{"label": o.label, "reason": "pose_untrusted", "position_m": o.position.tolist()}
                         for o in observations]
            observations = []
        v3 = self._v3()
        if v3:
            self._relocations = []
            seen_boxes = list(observations)     # before the admission gates: any same-class box blocks a miss
            skip_miss = self._turn_frame(frame, pose_valid)
        if quality and pose_valid:
            self._advance_odometer(frame)
            ground = getattr(frame, "ground_z_m", None)
            ground = self.cfg.ground_z_m if ground is None else float(ground)
            admitted = []
            for o in observations:
                why = memory_gates.admission_reason(o, ground, self.cfg)
                if why is None:
                    admitted.append(o)
                else:
                    rejected.append({"label": o.label, "reason": why, "position_m": o.position.tolist()})
            admitted, duplicates = memory_gates.dedupe(admitted, self.cfg,
                                                       iou=self.cfg.dedupe_iou if self.cfg.identity_v2 else None)
            rejected += [{"label": o.label, "reason": "same_frame_duplicate", "position_m": o.position.tolist()}
                         for o in duplicates]
            observations = admitted
        sigma = getattr(frame, "pose_sigma_m", None) if quality else None
        gated = quality and self._create_gated()
        held = []
        if gated or (quality and self.cfg.identity_v2 and self.cfg.move_v2):
            self._expire_candidates(frame.timestamp)
        for t in self.tracks.values():
            if t.state in ("current_confirmed", "temporarily_occluded"):
                t.state = "historical_memory"
        merged = []
        if quality and self.cfg.identity_v2:
            protected, ambiguous, merged = self._associate_v2(observations if pose_valid else [], sigma, gated,
                                                              rejected, held)
        else:
            # Mutual uniqueness avoids arbitrary assignment between similar nearby instances.
            possible = []
            for o in observations if pose_valid else []:
                ids = [t.id for t in self.tracks.values() if t.label == o.label and
                       np.linalg.norm(t.position-o.position) <= self._radius(t, sigma)]
                possible.append((o, ids))
            counts = {k: sum(k in ids for _, ids in possible) for k in self.tracks}
            protected = set(); ambiguous = []
            for o, ids in possible:
                protected.update(ids)
                if len(ids) > 1 or (ids and counts[ids[0]] > 1):
                    ambiguous.append({"frame_id": o.frame_id, "candidate_ids": ids})
                    for key in ids:
                        self.tracks[key].reason = "ambiguous_association"
                    continue
                if not ids:
                    if quality and not memory_gates.may_create(o):
                        rejected.append({"label": o.label, "reason": "clipped_box_cannot_create",
                                         "position_m": o.position.tolist()})
                        continue
                    history = []
                    if gated:
                        if o.score < self._create_min_score(o.label):
                            rejected.append({"label": o.label, "reason": "score_below_create",
                                             "position_m": o.position.tolist()})
                            continue
                        c = self._candidate_for(o, sigma)
                        if not self._candidate_ready(c):
                            held.append({"candidate_id": c.id, "label": o.label, "hits": len(c.history),
                                         "position_m": o.position.tolist()})
                            continue
                        del self.candidates[c.id]
                        self.candidate_stats["promoted"] += 1
                        # The current observation is appended below. Drop the candidate's last entry only
                        # when it is this observation (same frame); a same-frame second observation did
                        # not add a hit, so slicing blindly would lose a real earlier one.
                        history = list(c.history)
                        if history and history[-1]["frame_id"] == o.frame_id:
                            history.pop()
                    key = self._new_id()
                    t = Track(key, o.label, o.color, o.position.copy(), o.lower.copy(), o.upper.copy(), o.timestamp,
                              history=history, created_at=o.timestamp, number=self._next_number(o.label))
                    self.tracks[key] = t; protected.add(key)
                else:
                    t = self.tracks[ids[0]]
                    t.position = o.position.copy(); t.lower = o.lower.copy(); t.upper = o.upper.copy(); t.color = o.color
                t.odo = self.odometer; t.pose_sigma = sigma
                t.last_seen = o.timestamp; t.last_seen_frame = o.frame_id
                t.state = "current_confirmed"; t.reason = "new_visual_observation"; t.negative_poses.clear()
                if v3:
                    self._ex_hit(t, o.timestamp)
                t.history.append({"frame_id": o.frame_id, "timestamp": o.timestamp, "bbox": list(o.bbox),
                                  "score": o.score, "source": o.source, "position_m": o.position.tolist(),
                                  "valid_depth_fraction": o.valid_fraction})
        if v3:
            fell = self._existence_frame(geometry_evidence, frame, protected, seen_boxes if pose_valid else [],
                                         sigma, pose_valid, untrusted, skip_miss=skip_miss)
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
        for key, t in self.tracks.items():
            if key in protected:
                continue
            ev = geometry_evidence.get(key, "unknown")
            t.reason = ev
            if not pose_valid:
                t.reason = "pose_untrusted" if untrusted and frame.pose_valid else "invalid_pose"; continue
            if self.strategy == "static":
                continue
            if self.strategy == "miss":
                t.negative_poses.append((frame.timestamp, frame.T_world_camera.copy()))
            elif ev == "free" and self._independent(frame, t):
                t.negative_poses.append((frame.timestamp, frame.T_world_camera.copy()))
            elif ev == "occluded" and t.state != "position_invalid":
                t.state = "temporarily_occluded"
            elif ev == "surface_or_uncertain":
                t.negative_poses.clear()  # contradictory geometry resets prior free-space evidence
            if len(t.negative_poses) >= self.cfg.negative_views:
                t.state = "position_invalid"
        for row in rejected:
            self.rejected[row["reason"]] = self.rejected.get(row["reason"], 0) + 1
        out = {"ignored": None, "ambiguous": ambiguous, "rejected": rejected,
               "geometry": geometry_evidence, "tracks": self.snapshot()}
        if merged:
            out["merged"] = merged
        if gated:
            out["candidates"] = held
        return out

    # ------------------------------------------------------------------ 10-27 identity (identity_v2)

    def _new_id(self):
        """Monotonic object serial (merged ids are never reused; restored ids may have gaps)."""
        used = [int(k[7:]) for k in list(self.tracks) + list(self.merged)
                if k.startswith("object-") and k[7:].isdigit()]
        self._serial = max([self._serial] + used) + 1
        return "object-%04d" % self._serial

    def _next_number(self, label):
        """Class-local display number (瓶子 1, 瓶子 2, ...), never reused within this memory."""
        used = [t.number for t in self.tracks.values() if t.label == label and t.number is not None]
        n = max([self._numbers.get(label, 0)] + used) + 1
        self._numbers[label] = n
        return n

    def _obs_sd(self, o):
        r = getattr(o, "range_m", None)
        r = 1.5 if r is None or not np.isfinite(r) else float(r)
        return self.cfg.obs_sigma0_m + self.cfg.obs_sigma_k * r * r

    def _drift(self, odo_a, sigma_a, odo_b, sigma_b):
        """Pose drift between two updates, as association_radius computes it (cap applied there)."""
        cfg = self.cfg
        return memory_gates.association_radius(odo_a, sigma_a, odo_b, sigma_b, cfg) - cfg.association_m

    def _gate(self, t, o, sigma):
        cfg = self.cfg
        spread = cfg.gate_chi * float(np.hypot(self._obs_sd(o), t.fused_sd or self._obs_sd(o)))
        slack = memory_gates.class_slack(t.label)
        gate = max(cfg.association_m, spread) + slack + self._drift(t.odo, t.pose_sigma, self.odometer, sigma)
        return min(cfg.association_max_m + slack, gate)

    def _add_sample(self, t, o):
        t.samples.append((float(o.timestamp), o.position.copy(), self._obs_sd(o)))
        del t.samples[:-self.cfg.fuse_keep]

    def _fuse(self, t):
        """Inverse-variance mean of the inliers around the median (single readings no longer move a track)."""
        if not t.samples:
            return
        p = np.array([s[1] for s in t.samples]); w = 1.0 / np.square([s[2] for s in t.samples])
        med = np.median(p, axis=0)
        d = np.linalg.norm(p - med, axis=1)
        inl = d <= max(self.cfg.fuse_inlier_m, 3 * 1.4826 * float(np.median(d)))
        t.position = (p[inl] * w[inl, None]).sum(axis=0) / w[inl].sum()
        t.fused_sd = float(np.sqrt(1.0 / w[inl].sum()))

    def _touch(self, t, o, sigma, append=True):
        """Record an observation of ``t``; its position becomes the robust fused one."""
        if append:
            self._add_sample(t, o)
            t.history.append({"frame_id": o.frame_id, "timestamp": o.timestamp, "bbox": list(o.bbox),
                              "score": o.score, "source": o.source, "position_m": o.position.tolist(),
                              "valid_depth_fraction": o.valid_fraction})
        self._fuse(t)
        shift = t.position - o.position          # keep the observed surface extent around the fused centre
        t.lower = o.lower + shift; t.upper = o.upper + shift; t.color = o.color
        t.odo = self.odometer; t.pose_sigma = sigma
        t.last_seen = o.timestamp; t.last_seen_frame = o.frame_id
        t.state = "current_confirmed"; t.reason = "new_visual_observation"; t.negative_poses.clear()
        if self._v3():
            self._ex_hit(t, o.timestamp)
        self._observed(t, o, False)

    def _associate_v2(self, observations, sigma, gated, rejected, held):
        """Greedy nearest one-to-one association against fused positions; co-visibility; creation with an
        exclusion zone; merge pass. Returns (protected ids, ambiguous rows, merged rows)."""
        cfg = self.cfg
        kept = []
        for o in observations:
            r = getattr(o, "range_m", None)
            if r is not None and r > cfg.obs_max_range_m:
                rejected.append({"label": o.label, "reason": "beyond_range", "position_m": o.position.tolist()})
            else:
                kept.append(o)
        observations = kept
        pairs = []
        for i, o in enumerate(observations):
            reach = cfg.association_max_m + memory_gates.class_slack(o.label)   # no gate is wider
            for t in self.tracks.values():
                if t.label == o.label:
                    d = float(np.linalg.norm(t.position - o.position))
                    if d <= reach and d <= self._gate(t, o, sigma):
                        pairs.append((d, i, t.id))
        pairs.sort()
        assigned, used = {}, set()
        for _, i, k in pairs:
            if i not in assigned and k not in used:
                assigned[i] = k; used.add(k)
        for k in used:                          # distinct boxes in one frame = distinct objects
            self.tracks[k].covis |= used - {k}
        seen_now = set(used)                    # tracks some box of this frame updated, started or moved
        protected, ambiguous = set(), []
        for i, o in enumerate(observations):
            near = [k for _, j, k in pairs if j == i]
            protected.update(near)
            if i in assigned:
                t = self.tracks[assigned[i]]
                if len(near) > 1:
                    ambiguous.append({"frame_id": o.frame_id, "candidate_ids": near, "resolved": t.id})
                    self.identity_stats["resolved_ambiguous"] += 1
                if not self._far(t, o):
                    self._touch(t, o, sigma)
                    continue
                if self._far_reading(t, o, sigma, seen_now, held, rejected) is not None:
                    continue                    # move pending, or done (the track took the new place)
                # the old place was confirmed after the first far reading: an ordinary creation candidate
            # Nothing in the gate, or every track in it was taken by another box of this frame.
            if not memory_gates.may_create(o):
                rejected.append({"label": o.label, "reason": "clipped_box_cannot_create",
                                 "position_m": o.position.tolist()})
                continue
            r = getattr(o, "range_m", None)
            if r is not None and r > cfg.create_max_range_m:
                rejected.append({"label": o.label, "reason": "too_far_to_create", "position_m": o.position.tolist()})
                continue
            c = None
            if gated:
                if o.score < self._create_min_score(o.label):
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
        return protected, ambiguous, self._merge_pass()

    def _create_min_score(self, label):
        """Minimum detector score for an observation to start (or count towards) a new track of ``label``."""
        return max(self.cfg.create_min_score, dict(self.cfg.create_min_score_by_class).get(label, 0.0))

    def _far(self, t, o):
        """10-28 move_v2: ``o`` (associated to ``t``) lies too far from the fused position to confirm it there:
        farther than max(move_far_m, gate_chi x the combined reading/fused sd) -- the gate's own spread term."""
        cfg = self.cfg
        if not cfg.move_v2 or not t.samples:
            return False
        spread = cfg.gate_chi * float(np.hypot(self._obs_sd(o), t.fused_sd or self._obs_sd(o)))
        limit = max(cfg.move_far_m, spread) + memory_gates.class_slack(t.label)
        return float(np.linalg.norm(o.position - t.position)) > limit

    def _observed(self, t, o, far, weak=False):
        """Remember the latest observation associated to ``t`` (query_memory exports it as "observed")."""
        t.observed = {"frame_id": o.frame_id, "timestamp": float(o.timestamp), "position_m": o.position.tolist(),
                      "bounds_m": [o.lower.tolist(), o.upper.tolist()], "bbox": list(o.bbox),
                      "score": float(o.score), "far": bool(far), "weak": bool(weak),
                      "offset_m": round(float(np.linalg.norm(o.position - t.position)), 3)}

    def _may_seed(self, o):
        """The evidence that may START a track (complete box, class minimum score, creation range) -- 10-28: the
        same evidence is needed to MOVE one. Replay of car run 20261028-164456-live without this: three top-edge
        boxes of a false banana (score 0.25-0.3, z 0.46 m) dragged the real banana's track 0.71 m onto it."""
        r = getattr(o, "range_m", None)
        return (memory_gates.may_create(o) and o.score >= self._create_min_score(o.label)
                and (r is None or r <= self.cfg.create_max_range_m))

    def _far_reading(self, t, o, sigma, seen_now, held, rejected):
        """10-28 move_v2: a far reading of ``t`` is not a confirmation of its place; it feeds a candidate marked
        move_of = t (one pool with creation candidates, same fusion). Returns the track once moved (``t``, id and
        number kept), ``t`` while the move is pending or the reading is too weak to move anything (_may_seed: it
        is then only remembered as observed/weak), or None when the old place was confirmed after the first far
        reading (the caller then treats ``o`` as an ordinary unmatched observation, creation gates and all)."""
        cfg = self.cfg
        if not self._may_seed(o):
            self._observed(t, o, True, weak=True)
            rejected.append({"label": o.label, "reason": "far_reading_too_weak_to_move", "track_id": t.id,
                             "position_m": o.position.tolist()})
            return t
        self._observed(t, o, True)
        c = self._candidate_for(o, sigma)
        c.covis |= seen_now - {t.id}
        if c.move_of not in (None, t.id) and c.move_of in self.tracks:
            return None                     # this candidate is already another track's move: create as usual
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

    def _promote(self, o, c, used, sigma, rejected):
        """A ready candidate (or, ungated, one observation) becomes a track -- unless a same-class track
        that was never seen together with it lies inside the exclusion zone: then it was that object
        (dropped if the track kept being seen meanwhile, else the object moved and the track follows)."""
        cfg = self.cfg
        pos = o.position if c is None else c.position
        covis = set(used) if c is None else set(c.covis) | set(used)
        slack = memory_gates.class_slack(o.label)
        near = sorted((float(np.linalg.norm(t.position - pos)), t.id) for t in self.tracks.values()
                      if t.label == o.label and t.id not in covis and
                      np.linalg.norm(t.position - pos) <= max(cfg.exclusion_m + slack, self._gate(t, o, sigma)))
        if near:
            tn = self.tracks[near[0][1]]
            first = c.history[0]["timestamp"] if c is not None and c.history else o.timestamp
            if tn.last_seen >= first:
                rejected.append({"label": o.label, "reason": "near_existing_track", "track_id": tn.id,
                                 "position_m": o.position.tolist()})
                self.identity_stats["suppressed_near"] += 1
                return None
            return self._take_over(tn, o, c, sigma, "near")
        if self._v3():
            # 10-27 step 2: the object moved farther than the exclusion zone -- the same-class track whose
            # place was found empty (never seen together with this candidate) is it; nearest first.
            gone = sorted((float(np.linalg.norm(t.position - pos)), t.id) for t in self.tracks.values()
                          if t.label == o.label and t.state == "position_invalid" and t.id not in covis)
            if gone:
                return self._take_over(self.tracks[gone[0][1]], o, c, sigma, "position_invalid")
        if cfg.one_per_class:
            # 10-31: one object per class on the field -- any same-class track is this candidate's object.
            same = [t for t in self.tracks.values() if t.label == o.label]
            if same:
                far = sorted((float(np.linalg.norm(t.position - pos)), t.id) for t in same if t.id not in covis)
                if not far:
                    # seen in one frame with every same-class track: one of the two is a false detection
                    rejected.append({"label": o.label, "reason": "one_per_class_covisible",
                                     "track_id": min(t.id for t in same), "position_m": o.position.tolist()})
                    self._count("suppressed_one_per_class")
                    return None
                tn = self.tracks[far[0][1]]
                first = c.history[0]["timestamp"] if c is not None and c.history else o.timestamp
                if tn.last_seen >= first:
                    # the object is still being seen at its old place: the candidate is not a move
                    rejected.append({"label": o.label, "reason": "one_per_class_existing_still_seen",
                                     "track_id": tn.id, "position_m": o.position.tolist()})
                    self._count("suppressed_one_per_class")
                    return None
                return self._take_over(tn, o, c, sigma, "one_per_class")
        key = self._new_id()
        t = Track(key, o.label, o.color, pos.copy(), o.lower.copy(), o.upper.copy(), o.timestamp,
                  history=[] if c is None else list(c.history), created_at=o.timestamp,
                  number=self._next_number(o.label))
        t.samples = [] if c is None else list(c.samples)
        t.covis = {k for k in covis if k in self.tracks}
        for k in t.covis:
            self.tracks[k].covis.add(key)
        self.tracks[key] = t
        self._touch(t, o, sigma, append=c is None)
        return t

    def _count(self, key):
        self.identity_stats[key] = self.identity_stats.get(key, 0) + 1

    def _take_over(self, tn, o, c, sigma, why):
        """``tn`` was this candidate's object: its id and number stay, its place becomes the new one."""
        before = tn.position.copy()
        tn.samples = [] if c is None else list(c.samples)
        if c is not None:
            tn.history.extend(c.history)
            tn.covis |= {k for k in c.covis if k in self.tracks and k != tn.id}
            for k in tn.covis:
                self.tracks[k].covis.add(tn.id)
        v3 = self._v3()
        if v3:
            tn.existence = None             # _touch restarts the score at existence_init
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
        if v3:
            self._relocations.append({"track_id": tn.id, "from_m": before.tolist(), "to_m": tn.position.tolist(),
                                      "distance_m": float(np.linalg.norm(tn.position - before)), "via": why})
        return tn

    def _merge_pass(self):
        """Merge same-class tracks closer than the merge gate that were never seen in one frame."""
        cfg, out = self.cfg, []
        while True:
            best = None
            groups = {}
            for t in self.tracks.values():
                groups.setdefault(t.label, []).append(t)
            for label, ts in groups.items():
                if len(ts) < 2:
                    continue
                p = np.array([t.position for t in ts])
                d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=2)
                # same rule as creation: inside the exclusion zone and never seen together = one object
                for i, j in zip(*np.nonzero(np.triu(d <= cfg.merge_max_m + memory_gates.class_slack(label), 1))):
                    a, b = ts[i], ts[j]
                    if b.id not in a.covis and a.id not in b.covis and (best is None or d[i, j] < best[0]):
                        best = (float(d[i, j]), a, b)
            if best is None:
                return out
            d, a, b = best
            keep, drop = self._survivor(a, b)
            self._merge(keep, drop)
            out.append({"kept": keep.id, "merged": drop.id, "distance_m": d})

    def _survivor(self, a, b):
        def rank(t):
            return (t.id not in self.keep_ids, t.number if t.number is not None else float("inf"), t.id)
        return (a, b) if rank(a) <= rank(b) else (b, a)

    def _merge(self, keep, drop):
        keep.samples = sorted(keep.samples + drop.samples, key=lambda s: s[0])[-self.cfg.fuse_keep:]
        keep.history = sorted(keep.history + drop.history, key=lambda h: h.get("timestamp", 0.0))
        if drop.last_seen > keep.last_seen:
            for name in ("last_seen", "last_seen_frame", "state", "reason", "odo", "pose_sigma", "color",
                         "lower", "upper", "existence", "ex_window", "ex_hit", "ex_free", "ex_miss_pd",
                         "ex_seen", "ex_rate", "ex_last_hit"):
                setattr(keep, name, getattr(drop, name))
            keep.negative_poses = list(drop.negative_poses)
        old = keep.position.copy()
        self._fuse(keep)
        keep.lower = keep.lower + (keep.position - old); keep.upper = keep.upper + (keep.position - old)
        self._retire(keep, drop)
        self.identity_stats["merged"] += 1

    def _retire(self, keep, drop):
        """``drop`` lives on as an alias of ``keep``: covisibility, aliases, merged map."""
        keep.covis |= drop.covis - {keep.id}
        keep.aliases += [drop.id] + drop.aliases
        del self.tracks[drop.id]
        for k, v in list(self.merged.items()):
            if v == drop.id:
                self.merged[k] = keep.id
        self.merged[drop.id] = keep.id
        for t in list(self.tracks.values()) + list(self.candidates.values()):
            if drop.id in t.covis:
                t.covis.discard(drop.id); t.covis.add(keep.id)
        keep.covis.discard(keep.id)

    # ------------------------------------------------------------------ 10-27 existence (existence_v3)

    def _v3(self):
        return self.cfg.existence_v3 and self.strategy == "geometry"

    def _ex_window(self, now):
        return int(np.floor(float(now) / self.cfg.existence_window_s))

    def _ex_clip(self, value):
        return float(min(self.cfg.existence_max, max(self.cfg.existence_min, value)))

    def _ex_start(self, t):
        """Score of a track that has none yet (restored from a file, or made while the switch was off)."""
        if t.existence is None:
            t.existence = self.cfg.invalid_below if t.state == "position_invalid" else self.cfg.existence_init

    def _ex_roll(self, t, window):
        """Close ``t``'s vote window if ``window`` is later; True when that vote made it position_invalid."""
        cfg = self.cfg
        if t.ex_window is not None and window <= t.ex_window:
            return False                     # same window (or a late frame of an earlier one): keep adding
        fell = False
        if t.ex_window is not None and not t.ex_hit and (t.ex_free or t.ex_miss_pd is not None):
            self._ex_start(t)
            delta = (cfg.existence_free if t.ex_free else 0.0)
            if t.ex_miss_pd is not None:
                delta += cfg.miss_weight * float(np.log(1.0 - t.ex_miss_pd))
            if t.ex_miss_pd is not None and self._reliable_miss(t):
                delta = (cfg.existence_free if t.ex_free else 0.0) + min(
                    cfg.miss_weight * float(np.log(1.0 - t.ex_miss_pd)), cfg.miss_reliable_vote)
            if cfg.miss_reliable:
                t.ex_seen.append((t.ex_window, 0))
                del t.ex_seen[:-64]
            value = t.existence + delta
            if not t.ex_free and t.ex_miss_pd < cfg.miss_decisive_pd:
                value = max(value, min(t.existence, cfg.miss_floor))   # weak misses alone never decide
            t.existence = self._ex_clip(value)
            if t.existence <= cfg.invalid_below:
                fell = t.state != "position_invalid"
                t.state = "position_invalid"
                t.reason = "free" if t.ex_free else "not_detected"
        t.ex_window, t.ex_hit, t.ex_free, t.ex_miss_pd = window, False, False, None
        return fell

    def _ex_hit(self, t, now):
        """A detection was associated to ``t``: the window votes +existence_hit once (a new or taken-over
        track starts at existence_init instead, the promoting detection being its first vote).
        The caller has already set the track's state/reason for this detection; closing the previous window
        may count its negative vote but must not leave a detected track position_invalid (10-27 host
        simulation: it did, with the box on it, until the next detected frame)."""
        state, reason = t.state, t.reason
        self._ex_roll(t, self._ex_window(now))
        t.state, t.reason = state, reason
        if t.existence is None:
            t.existence = self.cfg.existence_init
        elif not t.ex_hit:
            t.existence = self._ex_clip(t.existence + self.cfg.existence_hit)
        if self.cfg.miss_reliable and not t.ex_hit:
            self._reliable_hit(t, t.ex_window)
        t.ex_hit = True

    def _reliable_hit(self, t, window):
        """Fast removal: record a detection window and the track's recent in-view hit rate."""
        cfg = self.cfg
        t.ex_seen.append((window, 1))
        del t.ex_seen[:-64]
        span = int(round(cfg.miss_reliable_window_s / cfg.existence_window_s))
        recent = {}
        for w, h in t.ex_seen:
            if w > window - span:
                recent[w] = max(recent.get(w, 0), h)
        t.ex_rate = (sum(recent.values()) / len(recent)) if len(recent) >= cfg.miss_reliable_min_windows else None
        t.ex_last_hit = window

    def _reliable_miss(self, t):
        """Fast removal: the closing miss window of ``t`` votes at least miss_reliable_vote."""
        cfg = self.cfg
        return (cfg.miss_reliable and t.ex_rate is not None and t.ex_rate >= cfg.miss_reliable_rate
                and t.ex_miss_pd >= cfg.miss_decisive_pd and t.ex_last_hit is not None
                and (t.ex_window - t.ex_last_hit) * cfg.existence_window_s <= cfg.miss_reliable_recent_s)

    def _miss_pd(self, label, range_m):
        cfg = self.cfg
        xs, ys = zip(*cfg.miss_range_curve)
        curve = float(np.interp(range_m, xs, ys))
        return min(cfg.miss_pd_cap, curve * dict(cfg.miss_class_pd).get(label, cfg.miss_class_default))

    def _blocked(self, t, boxes, sigma):
        """A same-class box of this frame lies inside ``t``'s association gate (it may be this object)."""
        for o in boxes:
            if o.label != t.label:
                continue
            d = float(np.linalg.norm(t.position - o.position))
            gate = (self._gate(t, o, sigma) if self.cfg.memory_quality and self.cfg.identity_v2
                    else self._radius(t, sigma))
            if d <= gate:
                return True
        return False

    def _turn_frame(self, frame, pose_valid):
        """Turn gate: camera turn rate between this and the previous valid frame (deg/s); True when this
        frame's missed detections are no evidence (rate > miss_max_turn_dps, or within miss_turn_hold_s of
        such a frame). Only records the rate when Config.miss_turn_gate is off."""
        cfg, rate = self.cfg, None
        if pose_valid:
            R = frame.T_world_camera[:3, :3]
            if self._turn_prev is not None:
                dt = float(frame.timestamp) - self._turn_prev[0]
                if 0.0 < dt <= cfg.miss_turn_max_gap_s:
                    rate = float(np.degrees(memory_gates.rotation_angle(self._turn_prev[1], R))) / dt
            self._turn_prev = (float(frame.timestamp), R.copy())
        self.turn_dps = rate
        if not cfg.miss_turn_gate:
            self.turn_skip_miss = False
            return False
        if rate is not None and rate > cfg.miss_max_turn_dps:
            self._turn_hold_until = float(frame.timestamp) + cfg.miss_turn_hold_s
            skip = True
        else:
            skip = float(frame.timestamp) <= self._turn_hold_until
        self.turn_skip_miss = skip
        return skip

    def _existence_frame(self, geometry_evidence, frame, protected, boxes, sigma, pose_valid, untrusted,
                         detections_known=True, skip_miss=False):
        """One frame of existence evidence for every track not updated by a box (v3 replacement of the
        negative_views branch). Returns the tracks this frame's window roll made position_invalid.
        ``skip_miss``: the camera turns fast (turn gate) -- this frame gives no missed-detection evidence."""
        cfg, fell = self.cfg, []
        window = self._ex_window(frame.timestamp)
        T = frame.T_world_camera
        for key, t in self.tracks.items():
            if self._ex_roll(t, window):
                fell.append(t)
            if key in protected:
                continue
            if key not in geometry_evidence and not detections_known:
                continue                     # geometry-only entry: a track without a word is not touched
            ev = geometry_evidence.get(key, "unknown")
            sticky = t.state == "position_invalid"          # keeps the reason of its last negative vote
            if not pose_valid:
                if not sticky:
                    t.reason = "pose_untrusted" if untrusted and frame.pose_valid else "invalid_pose"
                continue
            if not sticky:
                t.reason = ev
            if ev == "occluded" and not sticky:
                t.state = "temporarily_occluded"
            if ev not in ("free", "surface_or_uncertain") or self._blocked(t, boxes, sigma):
                continue                     # occluded / out_of_view / too_small / invalid_depth / unknown: no vote
            if ev == "free":
                t.ex_free = True
            if detections_known and not skip_miss:
                r = float(((t.position - T[:3, 3]) @ T[:3, :3])[2])   # optical-axis depth
                if 0.0 < r <= cfg.miss_max_range_m:
                    pd = self._miss_pd(t.label, r)
                    if pd > 0.0:
                        t.ex_miss_pd = pd if t.ex_miss_pd is None else min(t.ex_miss_pd, pd)
        return fell

    def _relocate_invalid(self, fell, now):
        """Step 2: a track whose place was just found empty absorbs the same-class track that appeared
        elsewhere within relocate_window_s and was never seen together with it (the object moved). The
        surviving id follows _survivor (the older number, task ids never dropped); the place is the new one."""
        cfg, out = self.cfg, []
        for x in sorted(fell, key=lambda t: t.id):
            if x.id not in self.tracks or x.state != "position_invalid":
                continue
            ys = sorted((float(np.linalg.norm(y.position - x.position)), y.id) for y in self.tracks.values()
                        if y.label == x.label and y.id != x.id and y.state != "position_invalid" and
                        y.created_at is not None and now - y.created_at <= cfg.relocate_window_s and
                        (x.created_at is None or y.created_at > x.created_at) and
                        y.id not in x.covis and x.id not in y.covis)
            if not ys:
                continue
            d, yid = ys[0]
            keep, drop = self._relocate_merge(x, self.tracks[yid])
            out.append({"kept": keep.id, "merged": drop.id, "distance_m": d, "reason": "relocated"})
        return out

    def _relocate_merge(self, old, new):
        """Merge for a move: identity of the survivor, place (samples, bounds, state, score) of ``new`` --
        the two places are not averaged and the old place's samples are dropped."""
        keep, drop = self._survivor(old, new)
        if keep is not new:
            for name in ("last_seen", "last_seen_frame", "state", "odo", "pose_sigma", "color", "fused_sd",
                         "existence", "ex_window", "ex_hit", "ex_free", "ex_miss_pd", "ex_seen", "ex_rate", "ex_last_hit"):
                setattr(keep, name, getattr(new, name))
            keep.position = new.position.copy(); keep.lower = new.lower.copy(); keep.upper = new.upper.copy()
            keep.samples = list(new.samples)
            keep.negative_poses = []
        keep.reason = "relocated"
        keep.history = sorted(keep.history + drop.history, key=lambda h: h.get("timestamp", 0.0))
        if keep.created_at is None or (drop.created_at is not None and drop.created_at < keep.created_at):
            keep.created_at = drop.created_at
        self._retire(keep, drop)
        self.identity_stats["relocated_merged"] = self.identity_stats.get("relocated_merged", 0) + 1
        return keep, drop

    def resolve(self, track_id):
        """Current id of ``track_id`` (itself, or the track it was merged into), or None."""
        track_id = self.merged.get(track_id, track_id)
        return track_id if track_id in self.tracks else None

    def _create_gated(self):
        c = self.cfg
        return c.create_min_hits > 1 or c.create_min_span_s > 0 or c.create_min_score > 0

    def _expire_candidates(self, now):
        for key in [k for k, c in self.candidates.items() if now - c.last_seen > self.cfg.candidate_ttl_s]:
            del self.candidates[key]
            self.candidate_stats["expired"] += 1

    def _candidate_for(self, o, sigma):
        """The nearest same-label candidate within the association radius, updated with ``o``;
        a new candidate when there is none. One hit per frame per candidate."""
        best, best_d = None, None
        for c in self.candidates.values():
            d = float(np.linalg.norm(c.position - o.position))
            if c.label == o.label and d <= self._radius(c, sigma) and (best is None or d < best_d):
                best, best_d = c, d
        if best is None:
            self._candidate_serial += 1
            self.candidate_stats["started"] += 1
            best = Track(f"candidate-{self._candidate_serial:04d}", o.label, o.color, o.position.copy(),
                         o.lower.copy(), o.upper.copy(), o.timestamp, state="candidate", reason="creation_gate")
            self.candidates[best.id] = best
        elif best.last_seen_frame == o.frame_id:
            return best
        best.position = o.position.copy(); best.lower = o.lower.copy(); best.upper = o.upper.copy()
        best.color = o.color; best.last_seen = o.timestamp; best.last_seen_frame = o.frame_id
        best.odo = self.odometer; best.pose_sigma = sigma
        best.history.append({"frame_id": o.frame_id, "timestamp": o.timestamp, "bbox": list(o.bbox),
                             "score": o.score, "source": o.source, "position_m": o.position.tolist(),
                             "valid_depth_fraction": o.valid_fraction})
        if self.cfg.identity_v2 and self.cfg.memory_quality:
            self._add_sample(best, o)
            self._fuse(best)
        return best

    def _candidate_ready(self, c):
        span = c.history[-1]["timestamp"] - c.history[0]["timestamp"]
        return len(c.history) >= self.cfg.create_min_hits and span >= self.cfg.create_min_span_s

    def _untrusted(self, frame):
        """The runner's wheel/gyro verdict for this frame (None = not checked = trusted)."""
        return bool(self.cfg.pose_trust_gate and getattr(frame, "pose_trusted", None) is False)

    def _advance_odometer(self, frame):
        """Accumulate camera path and turn between consecutive valid frames (pose-source agnostic)."""
        pose = frame.T_world_camera
        if self._odometer_pose is not None:
            step = float(np.linalg.norm(pose[:3, 3] - self._odometer_pose[:3, 3]))
            turn = memory_gates.rotation_angle(self._odometer_pose[:3, :3], pose[:3, :3])
            self.odometer = (self.odometer[0] + step, self.odometer[1] + turn)
        self._odometer_pose = pose.copy()

    def _radius(self, track, sigma):
        if not self.cfg.memory_quality:
            return self.cfg.association_m
        return memory_gates.association_radius(track.odo, track.pose_sigma, self.odometer, sigma, self.cfg)

    def update_geometry_only(self, geometry_evidence, frame):
        """Consume mask-free region evidence without any detector observation.

        Transitions are copied from update_memory's unprotected-track branch;
        update_memory itself is unchanged. Unlike update_memory there is no
        leading downgrade of current_confirmed/temporarily_occluded: a missing
        detection is not this entry's business. See OBJECT_ROI_SPEC s.6.
        Negative evidence still stores frame.timestamp, the time base that
        _independent() shares with update_memory.
        """
        frame.validate()
        if frame.frame_id in self.geometry_seen_frames:
            return {"ignored": "duplicate_frame", "tracks": self.snapshot()}
        if frame.timestamp <= self.last_geometry_timestamp:
            raise ValueError("out-of-order geometry timestamp")
        # evidence computed before a merge may still name the merged id (10-27): ignore it, not an error
        geometry_evidence = {k: v for k, v in geometry_evidence.items() if k not in self.merged}
        unknown = set(geometry_evidence) - set(self.tracks)
        if unknown:
            raise ValueError("geometry evidence for unknown tracks")
        self.geometry_seen_frames.add(frame.frame_id)
        self.last_geometry_timestamp = frame.timestamp
        untrusted = self._untrusted(frame)
        if self._v3():
            # no detector here, so no miss votes: only free counts; a box for the same frame arriving
            # through update_memory still turns the window into a detection vote.
            self._relocations = []
            fell = self._existence_frame(geometry_evidence, frame, set(), [], None,
                                         frame.pose_valid and not untrusted, untrusted, detections_known=False)
            out = {"ignored": None, "geometry": dict(geometry_evidence), "tracks": None}
            merged = self._relocate_invalid(fell, frame.timestamp)
            out["tracks"] = self.snapshot()
            if merged:
                out["merged"] = merged
            return out
        for key, t in self.tracks.items():
            if key not in geometry_evidence:
                continue
            ev = geometry_evidence[key]
            t.reason = ev
            if not frame.pose_valid or untrusted:
                t.reason = "pose_untrusted" if frame.pose_valid else "invalid_pose"; continue
            if self.strategy == "static":
                continue
            if self.strategy == "miss":
                t.negative_poses.append((frame.timestamp, frame.T_world_camera.copy()))
            elif ev == "free" and self._independent(frame, t):
                t.negative_poses.append((frame.timestamp, frame.T_world_camera.copy()))
            elif ev == "occluded" and t.state != "position_invalid":
                t.state = "temporarily_occluded"
            elif ev == "surface_or_uncertain":
                t.negative_poses.clear()
            if len(t.negative_poses) >= self.cfg.negative_views:
                t.state = "position_invalid"
        return {"ignored": None, "geometry": dict(geometry_evidence), "tracks": self.snapshot()}

    def snapshot(self):
        return [t.as_dict() for t in self.tracks.values()]

    def query_memory(self, goal, after=None):
        # D-044 revised (2026-10-21): class words of the enabled classes (Config.query_classes) and of
        # existing tracks; the last class word is the target unless joined by 和/或/、/与/跟/,
        # (coco_zh.goal_labels). 10-22 (owner): colour words no longer filter the result -- the HSV colour
        # guess (Track.color, kept as data) was often wrong, so 红色杯子 means every cup.
        text = goal.lower()
        track_labels = {t.label for t in self.tracks.values()}
        requested_labels = coco_zh.goal_labels(text, set(self.cfg.query_classes) | track_labels,
                                               extra_phrases=track_labels)
        if not requested_labels:
            return []
        number = requested_number(text)          # 10-27: 瓶子2 / 2号瓶子 / 瓶子二 -> class-local number
        result = []
        for t in self.tracks.values():
            if t.label not in requested_labels:
                continue
            if number is not None and t.number != number:
                continue
            item = t.as_dict()
            item["confirmed_after_query"] = after is not None and t.last_seen > after and t.state == "current_confirmed"
            if t.observed is not None:      # 10-28: the latest associated reading (far = not at position_m)
                item["observed"] = dict(t.observed)
                item["observed_after_query"] = after is not None and t.observed["timestamp"] > after
            result.append(item)
        return result
