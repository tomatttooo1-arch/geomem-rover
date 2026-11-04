"""ROS 2 RGB-D/Memory/Nav2 adapter. Importable helpers need no ROS installation.

Run as ``python -m host.ros_runtime --socket PATH --pose-source reference
--sensor-source simulation``.
Use ``--pose-source estimated`` only with an actual localization TF provider.
Publish String to /fpga/query or /fpga/select_candidate, Empty to /fpga/cancel or
/fpga/patrol, String JSON {"forward_m", "turn_rad"} to /fpga/goto (one goal relative to the
car, 2026-10-17 test entry: the same ready(), motion permit and guard as the task chain). Status JSON is published on /fpga/status. The simulation's motor
watchdog must enforce both command age and /fpga/motion_permit age on the
simulator/motor host. Permission is false outside healthy active navigation.
Simulation automatically enables use_sim_time; hardware and replay (recorded frames on wall
time, tagged replay) reject simulation time.

The default tour is map coverage, independent of object locations. Nav2 performs
path planning; local occupancy/FOV filtering only proposes observation positions.
No ROS graph, actual navigation, SLAM or detector accuracy is simulated here.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import itertools
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import sys
import time
import threading

import numpy as np

from host.task_control import PoseGoal, TaskConfig, TaskController, chain_waypoints
from host.localization_gate import LocalizationLease
from host.nav_localization import lease_correction_reference, lease_correction_verdict
from host.runtime_diagnostics import RuntimeDiagnostics, lease_snapshot, queue_snapshot, status_snapshot
from host.memory_publish import MemoryFeed, new_boot_id, target_label_of
from host.region_verdicts import RegionMerger
from host.geometry_age_gate import GeometryAgeGate
from host.fpga_clock import now_ns as raw_now_ns
from host import gc_policy
from host import memory_store
from host import live_align, map_align
from host.trajectory_nodes import LiveTrack
from host import map_reset
from host.record_writer import DropReporter, MODES as RECORD_WRITE_MODES, RecordWriter
from host.backend_log import BackendLog
from host.search_belief_feed import BeliefFeed, dumps as belief_dumps


def stamp_of(message):
    """Header stamp of a ROS message in integer ns (stall diagnostics keys)."""
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def odometry_yaw(message):
    """Yaw of a nav_msgs/Odometry pose (planar wheel odometry)."""
    q = message.pose.pose.orientation
    return math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))


def feed_pose_trust(monitor, kind, message):
    """One /imu/data_raw (kind "imu") or wheel Odometry (kind "odom") message into a PoseTrustMonitor
    (docs/design/MEMORY_POSE_TRUST_20261017.md); stamps in ROS seconds."""
    t = stamp_of(message) / 1e9
    if kind == "imu":
        monitor.add_gyro_rate(t, float(message.angular_velocity.z))
    else:
        monitor.add_wheel_yaw(t, odometry_yaw(message))


def feed_pose_trust_slip(monitor, body):
    """One /fpga/wheel_odom/slip String body (host/ros_wheel_odom.slip_message, gyro mode only) into the
    monitor's external verdicts. Returns the (t, trusted) fed, or None for a malformed body."""
    try:
        row = json.loads(body)
        t, trusted = int(row["stamp_ns"]) / 1e9, bool(row["trusted"])
    except (ValueError, TypeError, KeyError):
        return None
    monitor.add_external(t, trusted)
    return t, trusted


def mark_pose_trust(monitor, frame):
    """Before Memory.update_memory: the monitor's verdict at the frame's acquisition time (None = off)."""
    if monitor is not None:
        frame.pose_trusted = monitor.trusted_at(frame.timestamp)


def enqueue_inference_result(runtime, frame, future):
    """Worker hands off results; only the ROS executor mutates task and memory."""
    with runtime.result_lock:
        if runtime.closed:
            return
        diag = getattr(runtime, "sdiag", None)
        if diag is not None:
            diag.queued(frame)
        runtime.results.put((frame, future))
        guard = runtime.result_guard
        if guard is not None and runtime.context.ok():
            try:
                guard.trigger()
            except Exception:
                if runtime.context.ok():
                    raise


class AcquisitionPending(Exception):
    """Clock/TF may arrive after its image on different ROS topics."""


class LocalizationSkipped(Exception):
    """The localization lease moved past this acquisition without approving it (2026-10-15).

    The gate approves in stamp order only, so the frame can never be approved: drop it (not a
    fault). The camera runs faster than odometry (board 30 fps vs ~10 Hz) and in correction
    mode only odometry stamps are approved, so most camera frames end here. Backstops stay the
    existing ones: lease unhealthy -> localization_invalid_or_timeout (<= 1 s) and no processed
    frame for 2 s while a task/navigation runs -> processed_stream_wall_timeout.

    Also counted here (2026-10-15 第 5 棒, stall-diag reason `expired_before_localization`): a
    frame that expires in the queue without its own try having waited for clock/TF, while the
    last status says the lease is only waiting or not ready yet and the clock and the static
    extrinsics the frame needs are fine (Runtime.expired_only_for_localization). Host g4r/g4r_re:
    the first approved frame stalled ~1.3 s in the runner, the frame queued behind it was never
    tried and expired on the clock/TF branch.
    """


class StaleWithoutLocalization(Exception):
    """An approved frame went stale (> 1 s old) inside the runner while the last status said
    waiting / not ready / no correction (2026-10-15 第 5 棒续; host g4r/g4r_re: the first
    approved frame stalled ~1.3 s). Skipped like LocalizationSkipped, stall-diag reason
    `stale_after_approval`; an odometry/clock/other status keeps sensor_or_pose_invalid."""


# Gate status (state, reason) -> skip class for a frame that has no localization (2026-10-15
# 第 5 棒续, controller ruling on owner's rule "the search program skips frames without
# localization, no fault; backstops: lease void in 1 s, no processed frame for 2 s").
# Whitelist: exactly these pairs skip; every other pair -- including any reason the gate adds
# later -- keeps the fault branch (wait_expired / sensor_or_pose_invalid). Reasons come from
# host/localization_gate.py (all count modes) and host/ros_localization_gate.py.
#   waiting     valid: three_healthy_acquisitions | three_consistent_corrections |
#               correction_refreshed (only while the lease kept that row: healthy)
#   not_ready   initializing: awaiting_acquisition | entry_reset:{entry_stale,unlinked,ref_gap,
#               inconsistent} (unlinked: since 第 8 棒 only for a lock started by a jump-rejected correction);  confirming: confirming_healthy_acquisitions |
#               confirming_corrections | confirming_after_correction_jump
#   no_correction  stale:correction_too_old;  lost:correction_jump (the gate turns a jump into
#               confirming_after_correction_jump within the same input, so it is not published
#               today; kept as a defensive entry so that it skips if it ever is);
#               lost:incomplete_output; stale:localization_stream_timeout (2026-10-16 第 5 棒收口,
#               controller ruling: rtabmap on the loaded board runs at 0.3-1.3 Hz, broken output
#               is normal there and means "no localization", not a fault; backstops are the lease
#               1 s void and the 2 s no-processed-frame check)
# Fault (not skipped), by group (LOCALIZATION_FAULT_REASONS):
#   odometry    stale:odometry_stale; lost:odom:* and lost:odom_info:* (odometry_lost,
#               invalid_covariance, invalid_pose, wrong_frame, wrong_child_frame);
#               lost:odometry_jump and initializing:entry_reset:odometry_jump (2026-10-17 第 10 棒: the
#               gate's odometry jump check; fault like the other odometry reasons -- the ruling keeps
#               every odometry reason out of the skip list -- until the controller/owner moves them)
#   clock       initializing:clock_rollback; initializing:replay_discontinuity:<n>;
#               lost:invalid_discontinuity_request; lost:future_stamp
#   unnamed by the ruling, fault by default (controller may move them):
#               stale:incomplete_acquisition_timeout; lost:rtabmap_output_gap;
#               lost:info:* and lost:localization_pose:* (wrong_frame, invalid_pose,
#               invalid_covariance, missing_ref_id, no_prior_map_match); lost:malformed_message;
#               lost:pending_overflow; lost:duplicate_message; lost:unexpected_or_retired_stamp;
#               lost:acquisition_stamp_invalid_or_duplicate; lost:unhealthy_acquisition;
#               lost:newer_acquisition_unhealthy
#   runner side (no status involved): a static extrinsic missing, frame stamp ahead of the clock,
#               a status the lease rejected, no status yet.
LOCALIZATION_SKIP_REASONS = {
    ("valid", "three_healthy_acquisitions"): "waiting",
    ("valid", "three_consistent_corrections"): "waiting",
    ("valid", "correction_refreshed"): "waiting",
    ("initializing", "awaiting_acquisition"): "not_ready",
    ("initializing", "entry_reset:entry_stale"): "not_ready",
    # since 2026-10-16 第 8 棒 emitted again, but only when the lock was started by a correction
    # a jump revoked on (第 7 棒 had stopped resetting the entry on an unlinked output otherwise)
    ("initializing", "entry_reset:unlinked"): "not_ready",
    ("initializing", "entry_reset:ref_gap"): "not_ready",
    ("initializing", "entry_reset:inconsistent"): "not_ready",
    ("confirming", "confirming_healthy_acquisitions"): "not_ready",
    ("confirming", "confirming_corrections"): "not_ready",
    ("confirming", "confirming_after_correction_jump"): "not_ready",
    ("stale", "correction_too_old"): "no_correction",
    ("lost", "correction_jump"): "no_correction",
    ("lost", "incomplete_output"): "no_correction",
    ("stale", "localization_stream_timeout"): "no_correction",
}
LOCALIZATION_FAULT_REASONS = {
    "odometry": ("stale:odometry_stale", "lost:odom:*", "lost:odom_info:*",
                 "lost:odometry_jump", "initializing:entry_reset:odometry_jump"),
    "clock": ("initializing:clock_rollback", "initializing:replay_discontinuity:*",
              "lost:invalid_discontinuity_request", "lost:future_stamp"),
    "unnamed": ("stale:incomplete_acquisition_timeout", "lost:rtabmap_output_gap", "lost:info:*",
                "lost:localization_pose:*", "lost:malformed_message", "lost:pending_overflow",
                "lost:duplicate_message", "lost:unexpected_or_retired_stamp",
                "lost:acquisition_stamp_invalid_or_duplicate", "lost:unhealthy_acquisition",
                "lost:newer_acquisition_unhealthy"),
}


def localization_skip_class(view):
    """Skip class of the last delivered status `view` (Runtime.localization_view), or None
    (fault branch). A valid row counts only while the lease kept it (healthy right after)."""
    if view is None:
        return None
    kind = LOCALIZATION_SKIP_REASONS.get((view["state"], view["reason"]))
    if kind == "waiting" and not view["kept"]:
        return None
    return kind


@dataclass
class WaitingAcquisition:
    stamp: float
    received_at: float
    payload: object
    clock_tf_waited: bool = False   # a try of this frame waited for its ROS clock or a TF lookup


# 10-26: consecutive bounded clock/TF wait expiries during a task / navigation before it is faulted
TF_EXPIRY_FAULT_RUN = 3


class AcquisitionQueue:
    """Keep the waiting frame and one newest successor; arrivals cannot reset its deadline."""

    def __init__(self, timeout=.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("positive finite acquisition wait required")
        self.timeout = timeout
        self.current = self.latest = None

    def push(self, stamp, payload, now):
        entry = WaitingAcquisition(stamp, now, payload)
        if self.current is None:
            self.current = entry
            return 0
        if stamp <= self.current.stamp or (self.latest and stamp <= self.latest.stamp):
            return 1
        dropped = int(self.latest is not None)
        self.latest = entry
        return dropped

    def expired(self, now):
        return self.current is not None and now-self.current.received_at >= self.timeout

    def pop(self):
        self.current, self.latest = self.latest, None


@dataclass
class MotionPermit:
    """Fail-stop latch; sensor recovery alone cannot resume a retired navigation."""

    inhibited: bool = True
    # 10-26 owner：重成功率，放宽自 2 s / 1 s: the car (--input-hiccup drop) builds it with --stream-timeout-s (5 s)
    processed_max_s: float = 2.0
    acquisition_max_s: float = 1.0

    def stop(self):
        self.inhibited = True

    def arm(self):
        """Only the explicit new-navigation path may arm after checking readiness."""
        self.inhibited = False

    def allowed(self, navigating, cancelled, processed_age, acquisition_age,
                pose_valid, inference_healthy):
        return bool(not self.inhibited and navigating and not cancelled and pose_valid
                    and inference_healthy and math.isfinite(processed_age)
                    and math.isfinite(acquisition_age)
                    and 0 <= processed_age <= self.processed_max_s
                    and 0 <= acquisition_age <= self.acquisition_max_s)


def runner_tf_qos(reliable=False, qos=None):
    """QoS of the runner's tf2 listener for the DYNAMIC /tf topic (reference-pose branch).

    Same profile as the geometry producer (host.geo_producer.tf_listener_qos, adopted and
    board-tested 10-11): BEST_EFFORT + VOLATILE + KEEP_LAST 100 by default; ``reliable=True``
    (CLI --tf-reliable) gives the tf2_ros default RELIABLE (the behaviour before 10-12).
    Why: a reliable reader delivers in order, so one /tf sample lost on the UDP loopback
    (receive-buffer overflow next to the depth/RGB images on the same socket) holds back every
    later sample until the writer's periodic heartbeat (Fast DDS 3 s) triggers the resend; the
    runner then waits for TF and drops frames. Semantics unchanged: tf2 interpolates only
    between two real samples, never extrapolates; a frame whose stamp is not bracketed is still
    dropped after the bounded wait. /tf_static is not touched (the caller passes no
    static_qos: tf2_ros keeps it RELIABLE + TRANSIENT_LOCAL). ``qos`` = rclpy.qos (imported
    lazily so importing this module needs no ROS installation)."""
    if qos is None:
        import rclpy.qos as qos
    from host.geo_producer import tf_listener_qos
    return tf_listener_qos(qos, best_effort=not reliable)


# --camera-qos (2026-10-15, camera-drop batch 2 follow-up): QoS of the runner's three camera subscriptions
# (/fpga/camera_info, /fpga/rgb, /fpga/depth; after remapping, whatever topics they are mapped to).
#   sensor_data: qos_profile_sensor_data (BEST_EFFORT, KEEP_LAST 5, VOLATILE), the behaviour before 10-15
#   reliable:    RELIABLE, KEEP_LAST --camera-qos-depth (default 5), VOLATILE
#   auto (default): reliable for --sensor-source hardware (the Orbbec driver offers RELIABLE: board record
#                camera_topics.txt, 10-15), sensor_data for replay / simulation (the replay cameras publish
#                BEST_EFFORT, which a reliable reader would not match: no frames at all)
# Same rule and the same profile as the geometry producer's --depth-qos (host.geo_producer.depth_sub_qos,
# adopted after the 10-15 board pair: best effort lost 29.3 % of the depth frames, reliable 0).
CAMERA_QOS_MODES = ("auto", "reliable", "sensor_data")
CAMERA_SUB_DEPTH = 5


def camera_reliable(args):
    """--camera-qos resolved (args without the field: auto)."""
    mode = getattr(args, "camera_qos", None) or "auto"
    if mode not in CAMERA_QOS_MODES:
        raise ValueError("unknown camera qos: %s" % mode)
    return mode == "reliable" or (mode == "auto" and getattr(args, "sensor_source", None) == "hardware")


def camera_qos_depth(args):
    return int(getattr(args, "camera_qos_depth", None) or CAMERA_SUB_DEPTH)


def camera_qos_label(args):
    """What runtime_info / the startup event record: "reliable:<depth>" or "sensor_data"."""
    return ("reliable:%d" % camera_qos_depth(args)) if camera_reliable(args) else "sensor_data"


def runner_camera_qos(reliable=False, depth=CAMERA_SUB_DEPTH, qos=None):
    """The one profile of the three camera subscriptions. ``qos`` = rclpy.qos (imported lazily).
    Depth 5 for reliable too: the same as sensor_data (the A/B changes reliability only) and the same as the
    producer. Why not deeper: a reliable reader delivers in order and the runner works on the newest pair
    only (AcquisitionQueue), so a deeper reader history only delivers a longer backlog of older frames after
    an executor stall; 5 frames at the driver's 30 fps = 167 ms, next to the 200 ms geometry age gate, past
    which such a frame is refused anyway. The pair synchronizer's queue (5) is the same bound for how late
    one partner may arrive, so it is left as it is."""
    if qos is None:
        import rclpy.qos as qos
    from host.geo_producer import depth_sub_qos
    return depth_sub_qos(qos, reliable=reliable, depth=depth)


def input_log_summary_path(path):
    """--input-log rx_runner.csv -> rx_runner.json (the name tools/integ_car_chain/cam_drop/bx_readout.py
    looks for next to an rx_*.csv receiver)."""
    path = Path(path)
    return path.with_suffix(".json")


INPUT_LOG_LOST_ROWS = 4096


class RunnerInputLog(object):
    """--input-log (2026-10-15, measurement only): one CSV row per camera message that reaches the runner's
    subscription callback, in the columns of tools/integ_car_chain/rx_lite.py (topic, header stamp ns, wall ns,
    RAW ns) so bx_readout.py reads the file as one more receiver and matches it against the driver's per-frame
    record; and per topic the rmw "message lost" event (a sample lost in transport; NOT a keep-last overflow
    inside the reader), written with the QoS in effect to <name>.json at close. Rows go through a RecordWriter
    (the --record-write mode; async = a background thread), so the executor thread only formats one line.
    Wrapping composes with the stall diagnostics' wrap of the same callbacks in either order."""

    HEADER = "topic,stamp_ns,wall_ns,arrival_raw_ns\n"

    def __init__(self, path, writer, receiver, wall=time.time_ns, raw=None):
        self.path, self.writer, self.receiver = Path(path), writer, dict(receiver)
        self.wall, self.raw = wall, raw or raw_now_ns
        self.lost = {}
        self.topics = {}
        self.counts = {}
        self.errors = 0
        self.writer.write(self.HEADER)

    def wrap(self, subscription, nominal):
        """Stamp every message of ``subscription`` (an rclpy Subscription; message_filters keeps its own in
        ``.sub``) before its callback runs. The row's topic is the resolved name (after remapping)."""
        topic = getattr(subscription, "topic_name", None) or nominal
        self.topics[nominal] = topic
        self.counts.setdefault(topic, 0)
        inner = subscription.callback

        def logged(msg, _inner=inner, _topic=topic):
            try:
                st = msg.header.stamp
                self.writer.write("%s,%d,%d,%d\n" % (_topic, st.sec * 1000000000 + st.nanosec, self.wall(), self.raw()))
                self.counts[_topic] += 1
            except Exception:               # noqa: BLE001 - a measurement never breaks the runner
                self.errors += 1
            return _inner(msg)
        subscription.callback = logged
        return inner

    def lost_callback(self, nominal):
        """message_lost event callback for one topic (as rx_lite.lost_callback; duck-typed status)."""
        rec = self.lost.setdefault(nominal, {"supported": True, "total_count": 0, "events": 0, "change_sum": 0,
                                             "log": []})

        def on(status):
            n = int(getattr(status, "total_count", -1))
            ch = int(getattr(status, "total_count_change", -1))
            rec["total_count"] = n
            rec["events"] += 1
            rec["change_sum"] += max(ch, 0)
            if len(rec["log"]) < INPUT_LOG_LOST_ROWS:
                rec["log"].append([self.wall(), ch, n])
        return on

    def lost_unsupported(self, nominal, exc):
        self.lost[nominal] = {"supported": False, "error": repr(exc)}

    def close(self):
        """Drain the CSV, write <name>.json: {"_receiver": ..., <resolved topic>: {"lost": ..., "rows": n}}."""
        summary = self.writer.close() or {}
        summary.pop("slow", None)
        out = {"_receiver": dict(self.receiver, errors=self.errors, writer=summary)}
        for nominal, topic in self.topics.items():
            out[topic] = {"nominal": nominal, "rows": self.counts.get(topic, 0), "lost": self.lost.get(nominal)}
        with open(input_log_summary_path(self.path), "x") as f:
            json.dump(out, f, indent=1, sort_keys=True)
        return out


# --tf-retry (2026-10-12 v6): when a frame that waits for TF is tried again
#   arrival: also on every new /tf sample (hook_tf_arrival), not only at the next image / 100 ms poll
#   poll:    only at the next image pair or the 100 ms steady poll (the behaviour before v6)
TF_RETRY_MODES = ("arrival", "poll")


def hook_tf_arrival(listener, after):
    """Make ``after()`` run right after the tf2_ros listener has put a /tf message into its
    buffer: wrap the listener's /tf subscription callback (rclpy's executor reads
    ``subscription.callback`` per message; the stall diagnostics wrap the same attribute and
    compose with this in either order). No second /tf subscription is created (a second reader
    would change the DDS / shared-memory load the 10-12 /tf gap work is about). Returns False
    (nothing hooked) when the listener has no callable ``tf_sub.callback``.

    Threading: tf2_ros (Humble) TransformListener without ``spin_thread`` runs its callbacks on
    the node's executor, and the runner spins with plain ``rclpy.spin`` (single-threaded), so
    ``after`` runs on the same thread as on_images and the steady poll -- no lock needed."""
    sub = getattr(listener, "tf_sub", None)
    inner = getattr(sub, "callback", None)
    if sub is None or not callable(inner):
        return False

    def callback(msg, _inner=inner):
        result = _inner(msg)
        after()
        return result
    sub.callback = callback
    return True


RUNTIME_INFO_TOPIC = "/fpga/runtime_info"
RUNTIME_INFO_VERSION = 1
# age origin of the 200 ms geometry gate per --geometry-source (docs/design/RUNNER_AGE_GATE_20261011.md sec 2)
AGE_GATE_ORIGIN = {"cpu": "runner_arrival", "kv": "runner_arrival", "kv_topic": "producer_acquired"}


def code_version(root=None):
    """{"commit": the deployed COMMIT file at the repo root (board deploys write it) or None,
    "runtime_sha256": first 16 hex of this file} -- no git call (the board tree has no .git)."""
    root = Path(root) if root is not None else Path(__file__).resolve().parents[1]
    commit = None
    try:
        commit = (root / "COMMIT").read_text().strip() or None
    except OSError:
        pass
    try:
        sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:16]
    except OSError:
        sha = None
    return {"commit": commit, "runtime_sha256": sha}


# --detector (2026-10-14): names the inference service's declared ``detector``. Given -> the
# InferenceClient refuses replies from any other detector; omitted -> unchecked (stub/fixture
# services keep working) and recorded as "ssdlite", the value the runner always wrote before.
DETECTORS = ("ssdlite", "yolo")


def detector_selection(args):
    """(recorded detector name, expected_detector for InferenceClient or None)."""
    chosen = getattr(args, "detector", None)
    if chosen is not None and chosen not in DETECTORS:
        raise ValueError("unknown detector: %s" % chosen)
    return (chosen or "ssdlite"), chosen


def runtime_info_body(args, runtime_boot_id, geometry_source, raw_ns, limit_ms, version=None, tf_retry=None,
                      gc_mode=None):
    """Body of the latched /fpga/runtime_info (TRANSIENT_LOCAL, depth 1; 2026-10-12 v5): the
    runner's one-time start-up facts, so a hub (or any tool) that joins later still has them --
    the /fpga/status ``startup`` event is volatile and was missed when the hub came up late.
    The /fpga/status stream itself is unchanged. ``tf_retry`` (v6) = the retry mode in effect
    (the runner passes it; None -> taken from args); null with estimated poses (no /tf reader).
    v7: ``record_write`` (async | sync: how events/stall-diag files are written) and ``gc``
    (default | freeze, host/gc_policy.py). 2026-10-14: ``detector`` (recorded name, see
    detector_selection) and ``detector_checked`` (true when --detector was given and every
    inference reply is checked against it). 2026-10-15: ``camera_qos`` (camera_qos_label) and
    ``camera_qos_switch`` (--camera-qos as given, auto by default)."""
    detector, expected_detector = detector_selection(args)
    estimated = getattr(args, "pose_source", None) == "estimated"
    if tf_retry is None and not estimated:
        tf_retry = getattr(args, "tf_retry", None) or "arrival"
    return {"v": RUNTIME_INFO_VERSION, "runtime_boot_id": runtime_boot_id, "start_raw_ns": int(raw_ns),
            "sensor_source": args.sensor_source, "pose_source": args.pose_source,
            "inference_device": args.inference_device,
            "detector": detector, "detector_checked": expected_detector is not None,
            "geometry_source": geometry_source,
            "geometry_fallback": getattr(args, "geometry_fallback", None),
            "region_source": getattr(args, "region_source", None),
            "age_gate": {"limit_ms": limit_ms, "missing_origin": "refused",
                         "origin": AGE_GATE_ORIGIN.get(geometry_source)},
            "tf_qos": None if estimated else ("reliable" if getattr(args, "tf_reliable", False) else "best_effort"),
            "tf_retry": None if estimated else tf_retry,
            # 2026-10-15: camera subscriptions' QoS in effect and the switch that chose it
            "camera_qos": camera_qos_label(args), "camera_qos_switch": getattr(args, "camera_qos", None) or "auto",
            "stall_diag": bool(getattr(args, "stall_diag", None)),
            "record_write": getattr(args, "record_write", None) or "async",
            "gc": gc_mode or getattr(args, "gc", None) or "default",
            "version": version if version is not None else code_version()}


# --sensor-source: simulation (Gazebo, /clock, use_sim_time on); hardware (a live camera);
# replay (recorded frames re-published by a replay stand-in on wall time, e.g. the integration
# rig's replay camera: same clock rules as hardware, only the provenance tag differs, so the hub's
# --data-source replay and the producer's --data-source replay agree with it; 2026-10-12)
SENSOR_SOURCES = ("simulation", "hardware", "replay")


# 10-27 candidate merge: one-letter geometry words for frame_processed.track_ev (the log is ~47 MB a run)
TRACK_EV_CODES = {"free": "f", "surface_or_uncertain": "s", "occluded": "o", "out_of_view": "v",
                  "too_small": "t", "invalid_depth": "d", "unknown": "u"}


def track_ev_row(memory, see_through, raw, used, last_existence):
    """frame_processed.track_ev: {track id: [raw word, word given to memory, existence, blocked]} for every
    track that had a geometry word other than out_of_view this frame (detected ones included -- their reason
    in ``tracks`` is new_visual_observation and hides the word) or whose existence changed since it was last
    logged (10-27 car log: ~4.3 tracks a frame, ~2.9 kB a frame_processed row; this adds ~0.1 kB).
    ``last_existence`` (id -> value) is updated in place. Words are TRACK_EV_CODES letters ("-" = none)."""
    def code(word):
        return "-" if word is None else TRACK_EV_CODES.get(word, str(word))
    out = {}
    for tid, track in memory.tracks.items():
        if getattr(memory, "merged", {}).get(tid) is not None:
            continue
        ex = getattr(track, "existence", None)
        ex = None if ex is None else round(float(ex), 2)
        w_raw = raw.get(tid) if raw else None
        if w_raw in (None, "out_of_view") and ex == last_existence.get(tid):   # no vote, no change: skip
            continue
        last_existence[tid] = ex
        w_used = used.get(tid) if used else None
        out[tid] = [code(w_raw), code(w_used), ex, 1 if see_through.blocked(memory, tid) else 0]
    return out


def miss_turn_row(memory):
    """frame_processed.miss_turn: [camera turn rate deg/s (1 decimal) or null, 1 = this frame's missed
    detections were not counted (Config.miss_turn_gate)]; null when the memory has no existence score."""
    if not getattr(getattr(memory, "cfg", None), "existence_v3", False) or not hasattr(memory, "turn_dps"):
        return None
    dps = memory.turn_dps
    return [None if dps is None else round(float(dps), 1), 1 if memory.turn_skip_miss else 0]


def memory_config(args):
    """Memory Config of the runner. D-044 revised (2026-10-21): the creation gate (Config.create_*,
    a new track needs >= 3 sightings spanning >= 2 s) is on unless --memory-creation-gate off;
    a namespace without the attribute (fixtures, older callers) gets the gate too."""
    from dataclasses import replace
    from perception.fpga_replay.core import Config, UNGATED_CREATION
    cfg = Config()
    if getattr(args, 'yolo_profile', 'coco-seg') == 'household9':
        from perception.fpga_replay.household_names import NAMES
        cfg = replace(cfg, query_classes=NAMES)
    if getattr(args, 'yolo_profile', 'coco-seg') == 'combined':
        from perception.fpga_replay.yolo_detector import combined_classes
        cfg = replace(cfg, query_classes=combined_classes())
    if getattr(args, 'yolo_profile', 'coco-seg') == 'combined33':
        from perception.fpga_replay.yolo_detector import combined_classes, HOUSEHOLD_NAMES33
        cfg = replace(cfg, query_classes=combined_classes(household=HOUSEHOLD_NAMES33))
    if getattr(args, "geometry_v2", False):
        cfg = replace(cfg, geometry_v2=True)   # 10-27 geometry v2 (car_up CAR_GEOMETRY_V2=1; off = unchanged)
    if getattr(args, "existence_v3", "on") == "off":
        cfg = replace(cfg, existence_v3=False)   # 10-27 candidate: car_up CAR_EXISTENCE_V3=0 (= 0f377be memory rules)
    if getattr(args, "one_per_class", "on") == "off":
        cfg = replace(cfg, one_per_class=False)   # 10-31: car_up CAR_ONE_PER_CLASS=0 (several objects of one class)
    if getattr(args, "see_through_filter", "on") == "off":
        cfg = replace(cfg, see_through_filter=False)   # 10-27 candidate: car_up CAR_SEE_THROUGH_FILTER=0
    if getattr(args, "fast_removal", "on") == "off":
        cfg = replace(cfg, miss_reliable=False)   # 10-27 night: car_up CAR_FAST_REMOVAL=0 (= 036f82a memory rules)
    if getattr(args, "fast_removal_vote", None) is not None:
        cfg = replace(cfg, miss_reliable_vote=float(args.fast_removal_vote))   # car_up CAR_FAST_REMOVAL_VOTE
    if getattr(args, "memory_creation_gate", "on") == "off":
        return replace(cfg, **UNGATED_CREATION)
    return cfg


def use_simulation_clock(sensor_source, configured):
    """Bind clock semantics to the actual input mode, not just provenance text."""
    if sensor_source == "simulation":
        return True
    if sensor_source not in ("hardware", "replay") or configured:
        raise ValueError("%s acquisition requires use_sim_time=false" % sensor_source)
    return False


def image_array(message):
    """Decode supported ROS Image encodings, respecting row padding and byte order."""
    formats = {"rgb8": ("u1", 3), "bgr8": ("u1", 3),
               "32FC1": ("f4", 1), "16UC1": ("u2", 1)}
    if message.encoding not in formats:
        raise ValueError("unsupported image encoding: " + message.encoding)
    name, channels = formats[message.encoding]
    dtype = np.dtype(name).newbyteorder(">" if message.is_bigendian else "<")
    width, height, step = message.width, message.height, message.step
    required = width * channels * dtype.itemsize
    if width <= 0 or height <= 0 or step < required or len(message.data) != height * step:
        raise ValueError("invalid image dimensions, stride or payload length")
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
    result = raw[:, :required].copy().view(dtype).reshape(height, width, channels)
    if channels == 1:
        return result[:, :, 0].astype(np.dtype(name), copy=True)
    return result[:, :, ::-1].copy() if message.encoding == "bgr8" else result.copy()


def transform_matrix(transform):
    """Geometry Transform -> rigid matrix; reject an invalid quaternion."""
    from perception.fpga_replay.io import quaternion_matrix
    q, t = transform.rotation, transform.translation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError("nonfinite transform")
    return matrix


@dataclass
class FreeMap:
    """OccupancyGrid with rotated origin; unknown cells count as occupied."""

    cells: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def __post_init__(self):
        if (self.cells.ndim != 2 or not self.cells.size or self.resolution <= 0
                or not all(math.isfinite(x) for x in
                           (self.resolution, self.origin_x, self.origin_y, self.origin_yaw))):
            raise ValueError("invalid occupancy map")

    def free(self, x, y, radius=0.30):
        """Conservative square footprint around a circular base including safety margin."""
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0:
            return False
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = x - self.origin_x, y - self.origin_y
        u, v = (c*dx+s*dy)/self.resolution, (-s*dx+c*dy)/self.resolution
        r = radius / self.resolution
        left, right = math.floor(u-r), math.floor(u+r)
        bottom, top = math.floor(v-r), math.floor(v+r)
        h, w = self.cells.shape
        if left < 0 or bottom < 0 or right >= w or top >= h:
            return False
        patch = self.cells[bottom:top+1, left:right+1]
        return bool(np.all((patch >= 0) & (patch < 50)))

    def not_occupied(self, x, y, radius=0.30):
        """Like ``free`` but an UNKNOWN cell is allowed: the footprint lies on the grid and holds no occupied cell.
        For goals the car itself stood on while mapping (its own cells are often never observed)."""
        if not self.inside(x, y, radius):
            return False
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = x - self.origin_x, y - self.origin_y
        u, v = (c*dx+s*dy)/self.resolution, (-s*dx+c*dy)/self.resolution
        r = radius / self.resolution
        patch = self.cells[math.floor(v-r):math.floor(v+r)+1, math.floor(u-r):math.floor(u+r)+1]
        return bool(np.all(patch < 50))

    def inside(self, x, y, radius=0.30):
        """True when the same square footprint as ``free`` lies wholly on the grid."""
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0:
            return False
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = x - self.origin_x, y - self.origin_y
        u, v = (c*dx+s*dy)/self.resolution, (-s*dx+c*dy)/self.resolution
        r = radius / self.resolution
        h, w = self.cells.shape
        return (math.floor(u-r) >= 0 and math.floor(v-r) >= 0
                and math.floor(u+r) < w and math.floor(v+r) < h)


# teleop/Nav2 arbitration (2026-10-18, docs/design/TELEOP_NAV_ARB_20261018.md): the remote control's in-hand flag.
TELEOP_ACTIVE_TOPIC = "/fpga/teleop/active"
TELEOP_REFUSAL = "teleop_active"
TELEOP_ACTIVE_MAX_AGE_S = 0.5


def teleop_in_hand(sample, now_s, max_age_s=TELEOP_ACTIVE_MAX_AGE_S):
    """sample (in hand, monotonic receipt) or None -> True while the newest sample says in hand and is at most
    max_age_s old. A teleop node that stopped publishing releases it (its exit is a fail-stop of the stack anyway)."""
    return sample is not None and bool(sample[0]) and 0 <= now_s - sample[1] <= max_age_s


def navigation_refusal(busy, server_ready, input_ready):
    """Why a navigate command cannot be sent (one cause each), or None when it can.

    Checked in this order; before 2026-10-16 all three shared one reason word
    ("navigation_busy_server_unavailable_or_input_unhealthy"), so a select on a car
    without a NavigateToPose server read the same as a busy or blind one.
    """
    if busy:
        return "navigation_busy_previous_goal_active"
    if not server_ready:
        return "navigation_server_unavailable"
    if not input_ready:
        return "navigation_input_unhealthy"
    return None


# 10-28 blind zone: a base pose nearer the object than this (the 0.6 m ring; a remembered view taken from close by)
# is tried only after every 0.9 / 1.2 m pose -- up close the floor object sits at the image's bottom edge or below it
# (camera 0.39 m high, ~16 deg down: the floor is visible from ~0.46 m ahead of the lens, 0.56 m ahead of the base).
OBSERVATION_NEAR_M = 0.8


def observation_poses(candidate, free_map, K, image_size, T_base_camera,
                      robot_xy, distances=(.6, .9, 1.2), preferred_poses=(), rejected=None):
    """Filter a fixed ring by base clearance and calibrated target bounds in view.

    Inputs are online memory, map and calibration. This function has no scene
    object list. Nav2 must still validate/reach each proposed pose. Map occupancy
    can be stale, and an in-frame target may still be occluded; fresh RGB-D decides.
    ``rejected`` (optional dict) receives per-cause counts of the dropped poses:
    outside_map (footprint leaves the grid), occupied_or_unknown, target_behind_camera,
    target_out_of_frame -- recorded so an empty plan says why it is empty.
    """
    target = np.asarray(candidate["position_m"], dtype=float)
    lower, upper = np.asarray(candidate["bounds_m"], dtype=float)
    corners = np.array(list(itertools.product(*zip(lower, upper))))
    width, height = image_size
    proposed = list(preferred_poses)
    for distance in distances:
        for angle in np.linspace(-math.pi, math.pi, 12, endpoint=False):
            x, y = target[:2] + distance*np.array([math.cos(angle), math.sin(angle)])
            yaw = math.atan2(target[1]-y, target[0]-x)
            proposed.append(PoseGoal(float(x), float(y), yaw))
    result = []
    counts = {"outside_map": 0, "occupied_or_unknown": 0, "target_behind_camera": 0,
              "target_out_of_frame": 0}
    for pose in dict.fromkeys(proposed):
        x, y, yaw = pose.x, pose.y, pose.yaw
        if not free_map.free(x, y):
            inside = getattr(free_map, "inside", None)
            counts["occupied_or_unknown" if inside is None or inside(x, y) else "outside_map"] += 1
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        T_world_base = np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])
        T_camera_world = np.linalg.inv(T_world_base @ T_base_camera)
        camera = corners @ T_camera_world[:3, :3].T + T_camera_world[:3, 3]
        if np.any(camera[:, 2] <= .1):
            counts["target_behind_camera"] += 1
            continue
        projected = camera @ np.asarray(K).T
        uv = projected[:, :2] / projected[:, 2:3]
        if (np.isfinite(uv).all() and np.all(uv[:, 0] >= 8) and np.all(uv[:, 0] < width-8)
                and np.all(uv[:, 1] >= 8) and np.all(uv[:, 1] < height-8)):
            result.append(pose)
        else:
            counts["target_out_of_frame"] += 1
    if rejected is not None:
        rejected.update(counts)

    def near(pose):        # 10-28: the 0.6 m ring (and a close remembered view) after the 0.9 / 1.2 m rings
        return math.hypot(pose.x-target[0], pose.y-target[1]) < OBSERVATION_NEAR_M
    result.sort(key=lambda pose: (near(pose), pose not in preferred_poses,
                                  (pose.x-robot_xy[0])**2 + (pose.y-robot_xy[1])**2))
    return result


# /fpga/goto (2026-10-17, REAL_CAR_NAV_GOLIVE_20261016.md §3 item 7): ONE goal relative to the car's
# current pose (turn first, then forward along the new heading), for the real-car acceptance steps "1 m straight ahead" and "turn 90 deg in place".
# It goes through Runtime.navigate() like a task goal: ready(), MotionPermit.arm(), the permit topic
# the velocity guard reads, Nav2's speed limits -- it relaxes none of them. Bounds: forward 0..2 m
# (DWB does not reverse), |turn| <= pi, not both zero.
GOTO_MAX_FORWARD_M = 2.0


def goto_request(text):
    """Parse a /fpga/goto body -> (forward_m, turn_rad); ValueError with the reason otherwise."""
    try:
        body = json.loads(text)
    except ValueError:
        raise ValueError("not JSON")
    if not isinstance(body, dict) or set(body) - {"forward_m", "turn_rad"}:
        raise ValueError("body must be {forward_m, turn_rad}")
    values = []
    for key in ("forward_m", "turn_rad"):
        value = body.get(key, 0.0)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("%s must be a finite number" % key)
        values.append(float(value))
    forward, turn = values
    if not 0.0 <= forward <= GOTO_MAX_FORWARD_M:
        raise ValueError("forward_m must be in [0, %g]" % GOTO_MAX_FORWARD_M)
    if abs(turn) > math.pi:
        raise ValueError("turn_rad must be in [-pi, pi]")
    if forward == 0.0 and turn == 0.0:
        raise ValueError("forward_m and turn_rad both zero")
    return forward, turn


def goto_pose(world_base, forward_m, turn_rad):
    """Turn by `turn_rad`, then `forward_m` along the new heading: the map-frame goal (yaw wrapped).
    forward only = straight ahead; turn only = in place; both = e.g. "1 m to the left, facing left"."""
    world_base = np.asarray(world_base, dtype=float)
    yaw = math.atan2(world_base[1, 0], world_base[0, 0])
    goal_yaw = math.atan2(math.sin(yaw + turn_rad), math.cos(yaw + turn_rad))
    x = world_base[0, 3] + forward_m*math.cos(goal_yaw)
    y = world_base[1, 3] + forward_m*math.sin(goal_yaw)
    return PoseGoal(float(x), float(y), float(goal_yaw))


# 10-26 find-near (owner: "stop ~20 cm from the object, the camera facing it"). After found, ONE more goal through
# Runtime.navigate() (Nav2 + the motion permit + the velocity guard + the KV heartbeat/MCU latch, nothing relaxed):
# straight along the line base -> object centre, stopping with the car front APPROACH_GAP_TARGET_M from the object's
# box -- or farther when the safety chain needs it. Numbers (docs/design/FIND_NEAR_20261026.md):
#   * car front = camera + LENS_TO_FRONT_M (car.json camera_mount.lens_to_front_m, owner 10-16); camera x = the live
#     base_link->camera TF (car: 0.10 m, tools/car_geo_producer/common.MOUNT_XYZ) => front 0.107 m ahead of base_link
#   * Nav2 footprint: circle robot_radius 0.30 around base_link (nav2_overrides.yaml, rolling_layer_nav2.yaml,
#     car_inner.sh robot_radius_m), i.e. 0.193 m AHEAD of the physical front; the object stays LETHAL in the rolling
#     map after it leaves the view (no decay, CAM_TILT_10_20261017.md 3.1) and DWB ObstacleFootprint refuses any
#     overlap; one 0.05 m costmap cell of rasterisation margin => front gap >= 0.30 + 0.05 - 0.107 = 0.243 m
#   * Gemini 2 near-range laser guard: tripped with something 20-42 cm in front of the lens (10-13, depth p5 196 mm,
#     INTEG_CAR_CHAIN_20261011.md), the laser then stays OFF (depth all 0, the permit lapses, 10-22: 850 s stopped).
#     When any corner of the object's box would still be in the image at the stop pose, the lens keeps
#     APPROACH_LASER_LENS_M (the largest distance seen to trip) from the box.
APPROACH_GAP_TARGET_M = 0.20
LENS_TO_FRONT_M = 0.007
NAV_FOOTPRINT_RADIUS_M = 0.30
COSTMAP_CELL_M = 0.05
APPROACH_LASER_LENS_M = 0.42
APPROACH_MIN_FORWARD_M = 0.05     # closer than this already: no goal (DWB/goal tolerance 0.10 would not move)
APPROACH_MAX_FORWARD_M = 1.5      # en-route verdicts are <= 1.2 m from the centre; a longer leg is not an approach
APPROACH_TIMEOUT_S = 20.0         # wall clock of the approach goal; past it: stop, approach result "timeout"
APPROACH_WAIT_S = 5.0             # waiting for the cancelled task goal's result before the approach starts


def frame_base_xy(frame, T_base_camera):
    """The car base's map-frame (x, y) at a frame's acquisition, or None without a pose / mount transform."""
    try:
        if T_base_camera is None or not getattr(frame, "pose_valid", False):
            return None
        world_base = np.asarray(frame.T_world_camera, dtype=float) @ np.linalg.inv(np.asarray(T_base_camera, dtype=float))
        xy = world_base[:2, 3]
        return (float(xy[0]), float(xy[1])) if np.all(np.isfinite(xy)) else None
    except (AttributeError, TypeError, ValueError, np.linalg.LinAlgError):
        return None


def _box_entry_distance(origin, direction, lower, upper):
    """Distance along the unit xy ``direction`` from ``origin`` to the xy box [lower, upper]; None when the origin
    is inside the box or the ray misses it."""
    t_near, t_far = -math.inf, math.inf
    for o, d, lo, hi in zip(origin, direction, lower, upper):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        t_near, t_far = max(t_near, min(t1, t2)), min(t_far, max(t1, t2))
    if t_near > t_far or t_far < 0 or t_near <= 0:
        return None
    return t_near


def box_in_view(row, pose, K, image_size, T_base_camera):
    """True when any part of the row's box projects into the image from base pose ``pose`` (same projection as
    observation_poses, but ANY point of a 7 x 7 x 7 lattice over the box, not all eight corners: a box taller than
    the view at close range has no corner in the image yet fills it -- something the projector still lights)."""
    lower, upper = np.asarray(row["bounds_m"], dtype=float)
    corners = np.array(list(itertools.product(*(np.linspace(lo, hi, 7) for lo, hi in zip(lower, upper)))))
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    T_world_base = np.array([[c, -s, 0, pose.x], [s, c, 0, pose.y], [0, 0, 1, 0], [0, 0, 0, 1]])
    T_camera_world = np.linalg.inv(T_world_base @ np.asarray(T_base_camera, dtype=float))
    camera = corners @ T_camera_world[:3, :3].T + T_camera_world[:3, 3]
    front = camera[:, 2] > .1
    if not np.any(front):
        return False
    projected = camera[front] @ np.asarray(K, dtype=float).T
    uv = projected[:, :2] / projected[:, 2:3]
    width, height = image_size
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return bool(np.any(inside))


def approach_centre(row):
    """(centre, bounds) the final approach aims at: the found frame's reading when the task says the object was
    seen away from its remembered place (10-28: row["approach_position_m"/"approach_bounds_m"]), else the row's."""
    if row.get("approach_position_m") is not None and row.get("approach_bounds_m") is not None:
        return list(row["approach_position_m"]), row["approach_bounds_m"]
    return list(row["position_m"]), row["bounds_m"]


def approach_goal(world_base, row, T_base_camera, K=None, image_size=None, target_gap_m=APPROACH_GAP_TARGET_M):
    """The final-approach goal after found: (PoseGoal or None, info dict). ``info`` always carries the numbers
    (``reason`` when there is no goal): gap_m (planned car-front-to-box gap), gap_floor_m (footprint rule),
    laser_rule (the in-view laser-guard rule raised the gap), surface_m (base to box now), forward_m, and (10-28)
    goal_source (observation / memory) with target_m, the box centre aimed at (see approach_centre)."""
    world_base = np.asarray(world_base, dtype=float)
    base = world_base[:2, 3]
    centre_m, bounds = approach_centre(row)
    row = dict(row, position_m=centre_m, bounds_m=bounds)       # box_in_view below sees the same box
    centre = np.asarray(centre_m, dtype=float)[:2]
    lower, upper = (np.asarray(b, dtype=float) for b in bounds)
    lens_x = float(np.asarray(T_base_camera, dtype=float)[0, 3])
    front = lens_x + LENS_TO_FRONT_M
    floor = NAV_FOOTPRINT_RADIUS_M + COSTMAP_CELL_M - front
    gap = max(float(target_gap_m), floor)
    info = {"base_to_front_m": round(front, 3), "gap_floor_m": round(floor, 3), "laser_rule": False,
            "goal_source": "observation" if row.get("approach_source") == "observation" else "memory",
            "target_m": [round(float(v), 3) for v in centre_m]}
    if not (np.all(np.isfinite(base)) and np.all(np.isfinite(centre)) and np.all(np.isfinite(lower))
            and np.all(np.isfinite(upper))):
        return None, dict(info, reason="non_finite_input")
    delta = centre - base
    distance = float(np.hypot(*delta))
    if distance < 1e-6:
        return None, dict(info, reason="at_object_centre")
    unit = delta / distance
    yaw = math.atan2(unit[1], unit[0])
    surface = _box_entry_distance(base, unit, lower[:2], upper[:2])
    if surface is None:
        return None, dict(info, reason="inside_object_box")

    def pose_for(g):
        forward = surface - (g + front)
        return forward, PoseGoal(float(base[0] + forward*unit[0]), float(base[1] + forward*unit[1]), yaw)

    forward, pose = pose_for(gap)
    if K is not None and image_size is not None and box_in_view(row, pose, K, image_size, T_base_camera):
        laser_gap = APPROACH_LASER_LENS_M - LENS_TO_FRONT_M
        if laser_gap > gap:
            gap, info["laser_rule"] = laser_gap, True
            forward, pose = pose_for(gap)
    info.update(gap_m=round(gap, 3), surface_m=round(surface, 3), forward_m=round(forward, 3),
                yaw=round(yaw, 4))
    if forward < APPROACH_MIN_FORWARD_M:
        return None, dict(info, reason="already_within_gap")
    if forward > APPROACH_MAX_FORWARD_M:
        return None, dict(info, reason="too_far_for_approach")
    return pose, info


def load_waypoints(path=None):
    """JSON list of [x,y,yaw] map coverage goals, independent of object truth."""
    rows = json.loads(Path(path).read_text()) if path else [
        [0, 0, 0], [0, 1.8, -math.pi/2], [0, -1.8, math.pi/2],
        [-1.8, -1.8, 0], [-1.8, 1.8, 0]]
    if not isinstance(rows, list) or any(not isinstance(r, list) or len(r) != 3 for r in rows):
        raise ValueError("waypoints must be a JSON list of [x,y,yaw]")
    return tuple(PoseGoal(*row) for row in rows)


# Search waypoints (10-25, find-object reliability). load_waypoints() without a file = the SIMULATION world's five
# points: on the real car they are unrelated to the room (10-22: the first task was navigate (0,0,0) reason search) and
# FreeMap.free (unknown = not free) usually drops them all on a real map -> search_exhausted at once. The real car:
#   mapping run   tools/car_run/mapping_ctl.py thins the robot's optimized trajectory (the exported grid's = Nav2's map
#                 frame) to <= 6 goals and puts them in the latched /fpga/mapping/status (--search-waypoints-from-mapping)
#   otherwise     --search-fallback spin: three in-place quarter turns at the current pose (look around; no travel into
#                 unknown space) -- car_inner.sh passes it on the board; 'none' = no search goal (no_search_waypoints)
# The simulation (no flag) keeps the five points, filtered by FreeMap.free as before.
MAPPING_STATUS_TOPIC = "/fpga/mapping/status"     # = tools/car_run/mapping_ctl.STATUS_TOPIC (tested equal)
SEARCH_FALLBACKS = ("fixed", "spin", "none")


def mapping_search_waypoints(text):
    """PoseGoals of a /fpga/mapping/status document's "search_waypoints", or None (absent, failed, empty, invalid)."""
    try:
        doc = json.loads(text)
        sw = doc.get("search_waypoints") if isinstance(doc, dict) else None
        rows = sw.get("waypoints") if isinstance(sw, dict) and sw.get("ok") else None
        if not rows or not isinstance(rows, list):
            return None
        poses = []
        for row in rows:
            if not isinstance(row, list) or len(row) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in row):
                return None
            poses.append(PoseGoal(float(row[0]), float(row[1]), float(row[2])))
        return tuple(poses)
    except (ValueError, TypeError, AttributeError):
        return None


SEARCH_MODES = ("prob", "mvp")                  # 10-26: probabilistic search (host/search_belief.py) / the MVP list
SEARCH_BELIEF_TOPIC = "/fpga/search/belief"       # 10-26: down-sampled belief (BeliefSearch.belief_message), latched


def mapping_search_nodes(text):
    """10-26: [(x, y, yaw)] of a /fpga/mapping/status document's "search_nodes" (dense trajectory nodes for the
    probabilistic search), or None (absent, failed, empty, invalid)."""
    try:
        doc = json.loads(text)
        sn = doc.get("search_nodes") if isinstance(doc, dict) else None
        rows = sn.get("nodes") if isinstance(sn, dict) and sn.get("ok") else None
        if not rows or not isinstance(rows, list):
            return None
        out = []
        for row in rows:
            if not isinstance(row, list) or len(row) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in row):
                return None
            out.append((float(row[0]), float(row[1]), float(row[2])))
        return tuple(out)
    except (ValueError, TypeError, AttributeError):
        return None


def build_search_planner(mode, free_map, nodes, config, robot_pose, cache=None, camera=None):
    """10-26: (BeliefSearch or None, info) for one find task. None (the MVP's waypoint list) when the mode is mvp,
    there is no map or no dense trajectory nodes, or the belief cannot be built (info["reason"] says why). ``cache``
    (a dict): the geometry is rebuilt only when the map object or the nodes change; reset() per task. ``camera``
    (10-28): (K, (width, height), T_base_camera) -> BeliefSearch.set_camera (whole-object-in-frame views)."""
    from host.search_belief import BeliefSearch
    info = {"mode": mode}
    if mode != "prob":
        return None, dict(info, used="mvp", reason="search_mode_mvp")
    if free_map is None or not nodes:
        return None, dict(info, used="mvp", reason="no_map" if free_map is None else "no_search_nodes")
    key = (id(free_map), tuple(nodes))
    planner = cache.get(key) if cache is not None else None
    try:
        if planner is None:
            planner = BeliefSearch(free_map, nodes, config)
            planner.source_map = free_map      # held: the cache key's id() is not reused while cached
            if cache is not None:
                cache.clear()
                cache[key] = planner
            info["built"] = True
        planner.reset()
        planner.robot = tuple(robot_pose) if robot_pose is not None else None
        if camera is not None:
            planner.set_camera(*camera)
    except Exception as exc:                             # noqa: BLE001 - the MVP list stays usable
        return None, dict(info, used="mvp", reason="belief_build_failed: %s" % exc)
    return planner, dict(info, used="prob", cells=int(planner.ux.size), nodes=len(planner.nodes),
                         build_ms=round(planner.build_ms, 1), camera_model=planner.has_camera())


def spin_waypoints(x, y, yaw):
    """Three in-place goals at (x, y): yaw + 90, + 180, + 270 deg (wrapped) -- the four directions with the current."""
    return [PoseGoal(float(x), float(y), math.atan2(math.sin(yaw + k*math.pi/2), math.cos(yaw + k*math.pi/2)))
            for k in (1, 2, 3)]


def outward_twin(p):
    """10-26 search MVP: the same place facing the other way (mapping goals face the trajectory centroid, i.e.
    away from the walls; the twin looks at the wall side)."""
    return PoseGoal(p.x, p.y, math.atan2(math.sin(p.yaw + math.pi), math.cos(p.yaw + math.pi)), p.frame)


def select_search_waypoints(fixed, mapped, fallback, free_map, robot_pose, outward=False):
    """(goals, info) for one search/patrol. ``mapped``: the mapping trajectory's goals or None; ``robot_pose``:
    (x, y, yaw) in the map frame. Mapped goals: kept when not occupied (unknown allowed: the car stood there), nearest
    first (greedy chain from the robot). ``outward`` (10-26, the find search): each kept mapped goal also gets its
    outward_twin, visited right after it (the heading nearer the arrival direction first). None kept / none mapped ->
    ``fallback``: fixed (simulation: the fixed list filtered by FreeMap.free, unchanged), spin (spin_waypoints at the
    robot), none (empty: no_search_waypoints)."""
    info = {}
    if mapped:
        kept, dropped = [], []
        for p in mapped:
            check = getattr(free_map, "not_occupied", free_map.free)
            (kept if check(p.x, p.y) else dropped).append(p)
        info.update(mapped=len(mapped), mapped_dropped=[p.__dict__ for p in dropped])
        if kept:
            if outward:
                kept = [q for p in kept for q in (p, outward_twin(p))]
                info["outward_twins"] = len(kept) // 2
            ordered = chain_waypoints(kept, robot_pose[:2], robot_pose[2] if len(robot_pose) > 2 else None)
            info["source"] = "mapping_trajectory"
            return ordered, info
    if fallback == "spin":
        info["source"] = "spin_in_place"
        return spin_waypoints(*robot_pose), info
    if fallback == "none":
        info.update(source="none", reason="no_search_waypoints")
        return [], info
    info["source"] = "fixed"
    return [p for p in fixed if free_map.free(p.x, p.y)], info


def runner_memory_inputs(args, parser):
    """(this run's map identity, the validated memory document to load or None). A refusal is a parser
    error (exit 2) before rclpy starts: no node, no motion permit, nothing to roll back."""
    identity = memory_store.map_identity()
    if getattr(args, "memory_map_identity", None):
        try:
            with open(str(args.memory_map_identity), encoding="utf8") as stream:
                identity = json.load(stream)
        except (OSError, ValueError) as exc:
            parser.error("--memory-map-identity unreadable: %s" % exc)
        if not isinstance(identity, dict) or identity.get("kind") not in memory_store.IDENTITY_KINDS:
            parser.error("--memory-map-identity: not a memory_store identity")
    if getattr(args, "memory_out_dir", None) and not args.memory_out_dir.is_dir():
        parser.error("--memory-out-dir %s is not a directory" % args.memory_out_dir)
    if not getattr(args, "memory_in", None):
        return identity, None
    try:
        doc = memory_store.read_document(str(args.memory_in))
        memory_store.check(doc, identity)
    except memory_store.MemoryStoreError as exc:
        parser.error("--memory-in refused (%s): %s" % (exc.code, exc))
    return identity, doc


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def spin_until_interrupt(rclpy, make_node, ros_args, no_signal_handlers):
    """Spin ``make_node()``; on SIGINT/SIGTERM close it while the ROS context is still valid.

    rclpy's own handlers (the default, SignalHandlerOptions.ALL) shut the context down
    inside the signal: spin then raises ExternalShutdownException and close() runs on a
    dead context -- stop()'s motion_permit/Twist publishes and the geometry_close event
    fail with RCLError and the process exits 1 (2026-10-11 Nano runs). So rclpy installs
    no handler (``no_signal_handlers`` = SignalHandlerOptions.NO) and both signals raise
    KeyboardInterrupt in the main thread (set explicitly: a background job of a
    non-interactive shell inherits SIGINT ignored); the runtime's 0.1 s steady watchdog
    timer returns spin to Python, close() publishes and records, then rclpy.shutdown()."""
    rclpy.init(args=ros_args, signal_handler_options=no_signal_handlers)
    previous = {sig: signal.signal(sig, _interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = make_node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node:
                try:
                    node.close()
                finally:
                    if node.diagnostics:
                        node.diagnostics.flush()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                if handler is not None:         # None: installed outside Python, leave ours
                    signal.signal(sig, handler)


RETRY_FIELDS = ("nav_retries_same_goal", "max_nav_failures", "min_attempt_time_s", "retry_pause_s",
                "fault_nav_fail_first")
# 10-30 owner: the find's verdict knobs (not-seen frames + minimum observation at an observation pose; en-route caps
# for the target itself and for same-class search candidates); TaskConfig's defaults on the car unless flagged
FIND_FIELDS = ("not_seen_frames", "observe_min_s", "en_route_target_max_distance_m", "en_route_max_distance_m")
FAULT_NAV_ENV = "FPGA_FAULT_NAV_FAIL_FIRST"   # fault injection for host simulation (off unless set)


# --input-hiccup (10-26 owner：重成功率，放宽过度防御): what an input hiccup does to a running task / navigation.
#   fault (default, the host and the tests as before): a TF/clock wait past 0.5 s, an invalid sensor frame, a frame
#          with an invalid pose, or a task goal sent while the input is not ready ends the task (fault / failed)
#   drop  (the car, tools/car_run/car_inner.sh board): such a frame is dropped with an event, a task goal sent while the
#          input is not ready is a failed attempt (the 10-25 retry path); the motion permit and ready() accept input up
#          to --stream-timeout-s old (5 s on the car; fault keeps 2 s / 1 s). The backstops stay: the processed-stream
#          watchdog (--stream-timeout-s), the velocity guard, the PL heartbeat and the MCU.
INPUT_HICCUP_MODES = ("fault", "drop")


def input_age_limits(args):
    """(processed_max_s, acquisition_max_s) of MotionPermit / ready() for a parsed command line."""
    if getattr(args, "input_hiccup", "fault") == "drop":
        s = float(getattr(args, "stream_timeout_s", TaskConfig.stream_timeout_s))
        return max(2.0, s), max(1.0, s)
    return 2.0, 1.0


def camera_pairing(fps):
    """(colour/depth pair slop s, default producer-frame match tolerance s) at a camera rate (10-29).

    30 fps (and faster): 0.02 / 0.05 as before. Slower: the slop covers half a camera period plus 5 ms (owner 10-29:
    on the car at 15 fps the colour/depth stamps sat 26.6 / 32.1 ms apart in two starts -- the two sensors' phase
    looks random, so the gap can reach half a period, 33.3 ms, above the earlier 33 ms slop), capped at
    the largest stamp tolerance Frame.validate accepts; the match takes the neighbouring producer frame like the
    30 fps 50 ms did (1.1 periods, always >= the slop below 30 fps): 15 fps -> 0.038 / 0.073 s, 10 -> 0.055 / 0.11."""
    from perception.fpga_replay.core import PAIR_TOLERANCE_MAX_S
    fps = float(fps)
    if fps >= 30.0:
        return 0.02, 0.05
    return (max(0.02, min(PAIR_TOLERANCE_MAX_S, round(0.5 / fps + 0.005, 3))),
            max(0.05, round(1.1 / fps, 3)))


def task_config(args):
    """TaskConfig of a parsed command line: --navigation-timeout-s / --task-timeout-s / --stream-timeout-s and the
    10-25 retry options (TaskConfig's own defaults when the namespace lacks them); TaskConfig refuses bad values
    (ValueError)."""
    return TaskConfig(task_timeout_s=getattr(args, "task_timeout_s", TaskConfig.task_timeout_s),
                      navigation_timeout_s=getattr(args, "navigation_timeout_s", TaskConfig.navigation_timeout_s),
                      stream_timeout_s=getattr(args, "stream_timeout_s", TaskConfig.stream_timeout_s),
                      drop_invalid_pose_frames=getattr(args, "input_hiccup", "fault") == "drop",
                      # 10-29 owner: on the car a processed-frame gap warns and waits instead of failed:stream_lost
                      stream_stall_waits=getattr(args, "input_hiccup", "fault") == "drop",
                      **{k: getattr(TaskConfig, k) if getattr(args, k, None) is None else getattr(args, k)
                         for k in RETRY_FIELDS + FIND_FIELDS})


def fault_nav_default(environ=None):
    """--fault-nav-fail-first default: the environment's FPGA_FAULT_NAV_FAIL_FIRST (absent/empty -> 0, off)."""
    value = (os.environ if environ is None else environ).get(FAULT_NAV_ENV, "").strip()
    return value or "0"


def task_config_record(config):
    """The effective navigation/task budgets and retry options, as the startup event records them."""
    return {"navigation_timeout_s": config.navigation_timeout_s, "task_timeout_s": config.task_timeout_s,
            "stream_timeout_s": config.stream_timeout_s,
            **{k: getattr(config, k) for k in RETRY_FIELDS + FIND_FIELDS}}


# MAPPING_STATUS_TOPIC (defined with the search waypoints above): mapping_ctl status, latched, re-sent 1 s
MAP_SWITCH_REFUSAL = "map_switching"
MAPPING_STATUS_MAX_AGE_S = 3.0
# 10-29 find while mapping (owner: the acceptance may never press 结束建图): rtabmap's live grid (rtabmap_car.sh
# MAP_NS=rtabmap, car_inner MAPPING=1) is the runner's PLANNING map while mapping_ctl reports state "mapping" -- the
# observation poses, the search waypoints' occupancy check and the probabilistic search's belief grid then see the
# room as mapped so far (the same grid the stop would export) instead of Nav2's blank 20 m start map (every pose
# "free", walls invisible). Only with --live-align: then TF map -> odom follows rtabmap's correction, so the grid's
# map frame is the memory's. No live grid / not mapping -> /map as before (map_server: the blank start map while
# mapping, the export after the stop). Nav2's own costmaps are unchanged.
LIVE_GRID_TOPIC = "/fpga/rtabmap/map"


def map_switch_refusal(row, age_s, memory_align_pending=False, max_age_s=MAPPING_STATUS_MAX_AGE_S):
    """10-25 find-retry (audit nav4): MAP_SWITCH_REFUSAL while the mapping stop is saving / aligning / loading the
    exported map (mapping_ctl state ``saving``, or ``exported`` with ``map_loaded`` still None) or this runner's
    memory alignment is pending; a navigation started then has its start pose moved under it by load_map.

    ``age_s`` = seconds since the last status row arrived (mapping_ctl re-sends every 1 s); a row older than
    ``max_age_s`` (controller gone) or no row at all never refuses -- no map switch can come from a dead
    controller. Otherwise None (finding may start)."""
    if memory_align_pending:
        return MAP_SWITCH_REFUSAL
    if not isinstance(row, dict) or age_s is None or not 0 <= age_s <= max_age_s:
        return None
    state = row.get("state")
    if state in ("saving", "resetting") or (state == "exported" and row.get("map_loaded") is None):
        return MAP_SWITCH_REFUSAL      # resetting (10-26): mapping_ctl is restarting the map (host/map_reset.py)
    return None


def nav_failure_reason(status, codes):
    """Reason word of a non-succeeded NavigateToPose result status (codes = GoalStatus)."""
    if status == codes.STATUS_ABORTED:
        return "navigation_aborted"
    if status == codes.STATUS_CANCELED:
        return "navigation_canceled_by_server"
    return "navigation_failed"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--yolo-profile', choices=('coco-seg', 'household9', 'combined', 'household33', 'combined33'), default='coco-seg')
    parser.add_argument("--socket", required=True)
    parser.add_argument("--pose-source", choices=("reference", "estimated"), required=True)
    parser.add_argument("--sensor-source", choices=SENSOR_SOURCES, required=True)
    parser.add_argument("--inference-device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--result-wakeup", choices=("poll", "event"), default="poll")
    parser.add_argument("--detector", choices=DETECTORS, default=None,
                        help="recognition model the inference service must declare (checked per reply); "
                             "omitted: legacy unchecked, recorded as ssdlite (stub/fixture services)")
    parser.add_argument("--view-refinement", choices=("none",), default="none")
    parser.add_argument("--waypoints")
    parser.add_argument("--search-waypoints-from-mapping", action="store_true",
                        help="10-25 mapping run (car_inner MAPPING=1): search/patrol goals from the mapping trajectory "
                             "(mapping_ctl.py, latched %s)" % MAPPING_STATUS_TOPIC)
    parser.add_argument("--search-fallback", choices=SEARCH_FALLBACKS, default="fixed",
                        help="no trajectory goals: fixed = --waypoints / the simulation's five points (default), "
                             "spin = in-place quarter turns (the real car), none = no search")
    # 10-26 probabilistic search (host/search_belief.py): needs the mapping run's dense trajectory nodes
    # (--search-waypoints-from-mapping, status "search_nodes"); without them the MVP's waypoint list as before
    parser.add_argument("--search-mode", choices=SEARCH_MODES, default="prob",
                        help="prob = belief search over the mapped, seen area (default; falls back to mvp without "
                             "trajectory nodes), mvp = the 10-26 search MVP's waypoint list")
    parser.add_argument("--search-config", type=Path,
                        help="JSON object overriding host/search_belief.SearchConfig fields (q_table as "
                             "{\"0.6\": 0.9, ...}: the measured detection rate by distance)")
    parser.add_argument("--events", help="new JSONL path; existing runs are never overwritten")
    parser.add_argument("--diagnostics", type=Path, help="opt-in bounded memory observations, flushed only on exit")
    parser.add_argument("--frames-dir", type=Path, help="new directory for exact processed RGB-D/calibration/pose inputs")
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--localization-map-id", help="required pinned database SHA256 for estimated poses")
    parser.add_argument("--localization-tracked-frame", default="camera_link")
    parser.add_argument("--camera-frame", default="camera_optical_frame")
    parser.add_argument("--uint16-depth-scale", type=float, default=.001,
                        help="metres per unit for 16UC1 only; 32FC1 is already metres")
    # 10-17 pose trust (docs/design/MEMORY_POSE_TRUST_20261017.md): wheel-odometry yaw vs gyro yaw;
    # a disagreement pauses memory writes (navigation unchanged). No gyro messages = no effect.
    parser.add_argument("--memory-creation-gate", choices=("on", "off"), default="on",
                        help="on (D-044, 2026-10-21): an unseen object becomes a memory entry only after "
                             ">= 3 sightings spanning >= 2 s (candidates dropped 3 s after the last "
                             "sighting); off: created on the first sighting (before 10-21)")
    parser.add_argument("--pose-trust", choices=("on", "off"), default="on",
                        help="on: frames whose wheel/gyro yaw disagree are not written to memory; off: unchanged")
    parser.add_argument("--pose-trust-imu-topic", default="/imu/data_raw",
                        help="sensor_msgs/Imu, angular_velocity.z (host/ros_chassis_driver.py)")
    # v0.2 (2026-10-21): the wheel-only heading topic, NOT /fpga/wheel_odom -- with --heading gyro the
    # latter carries the gyro heading and the gate would compare the gyro with itself. In wheel mode
    # host/ros_wheel_odom.py publishes the same pose on both, so the gate is unchanged there.
    parser.add_argument("--pose-trust-odom-topic", default="/fpga/wheel_odom/wheel_heading",
                        help="nav_msgs/Odometry of the wheels alone (host/ros_wheel_odom.py --wheel-heading-topic)")
    parser.add_argument("--pose-trust-slip-topic", default="/fpga/wheel_odom/slip",
                        help="String JSON slip/crash verdicts of host/ros_wheel_odom.py --heading gyro, AND-ed "
                             "into the gate ('' = ignore; nothing published in wheel mode)")
    parser.add_argument("--pose-trust-gyro-period-s", type=float, default=0.01,
                        help="time one Imu message covers (MCU status frame period); serial batches share a stamp")
    parser.add_argument("--region-source", choices=("none", "kv", "kv_only"), default="none",
                        help="none: unchanged; kv: also merge /fpga/region_verdicts; kv_only: PL is the only geometry")
    parser.add_argument("--max-region-age-s", type=float, default=1.0,
                        help="placeholder (O12): re-derive as p95 x 1.5 of measured frame age")
    # Car geometry on the KV260 (docs/design/CAR_GEOMETRY_KV_RESULTS_20261011.md). Default cpu = unchanged.
    parser.add_argument("--geometry-source", choices=("cpu", "kv", "kv_topic"), default="cpu",
                        help="cpu: compute_geometry on the Nano (unchanged); kv: KV260 region core via kv-link "
                             "(this process is the KV client); kv_topic: the KV260 answers published by "
                             "host/ros_geo_producer.py on /fpga/geometry_frame (docs/design/"
                             "CAR_GEO_PRODUCER_RESULTS_20261011.md), this process never calls the KV260")
    parser.add_argument("--geometry-v2", action="store_true",
                        help="10-27 geometry v2 (Config.geometry_v2, docs/design/GEOMETRY_V2_20261027.md): the CPU "
                             "words follow v2, the see-through filter runs before update_memory, and a kv_topic "
                             "producer of the other version is warned about (never stopped). Off = unchanged")
    parser.add_argument("--existence-v3", choices=("on", "off"), default="on",
                        help="10-27 memory existence score (Config.existence_v3, docs/design/"
                             "MEMORY_EXISTENCE_20261027.md); off = the 0f377be two-independent-free rule")
    parser.add_argument("--one-per-class", choices=("on", "off"), default="on",
                        help="10-31 one object per class (Config.one_per_class): a same-class candidate elsewhere "
                             "takes the existing track over (id and number kept) instead of becoming a second "
                             "one; off = the earlier rules (needed to test several objects of one class)")
    parser.add_argument("--see-through-filter", choices=("on", "off"), default="on",
                        help="10-27 see-through filter (Config.see_through_filter): with existence_v3 or geometry v2 "
                             "on, a track detected while judged free twice no longer has its free believed")
    parser.add_argument("--fast-removal", choices=("on", "off"), default="on",
                        help="10-27 night fast removal (Config.miss_reliable, docs/design/FAST_REMOVAL_20261027.md): "
                             "a near track detected in >= 80 %% of its recent in-view windows loses at least "
                             "--fast-removal-vote per missed window; off = the 036f82a rules")
    parser.add_argument("--fast-removal-vote", type=float, default=None,
                        help="Config.miss_reliable_vote (default -1.15 = 5 windows; -1.7 = 3, -2.5 = 2)")
    parser.add_argument("--geometry-fallback", choices=("none", "cpu"), default="none",
                        help="kv only: when the KV260 is unavailable, none = geometry marked unavailable "
                             "(update_memory gets {}), cpu = compute_geometry; both are recorded as events")
    parser.add_argument("--kv-server", help="KV260 kv_server HOST:PORT (required with --geometry-source kv)")
    parser.add_argument("--kv-timeout-s", type=float, default=0.3, help="per-frame answer timeout")
    parser.add_argument("--kv-hello-timeout-s", type=float, default=0.5)
    parser.add_argument("--kv-retry-s", type=float, default=5.0, help="minimum gap between reconnect attempts")
    parser.add_argument("--kv-expect-build-id", help="refuse the KV260 unless its sysinfo BUILD_ID equals this")
    parser.add_argument("--kv-depth-unit-m", type=float,
                        help="depth code unit sent to the KV260 (default: --uint16-depth-scale)")
    parser.add_argument("--kv-grid", help="local grid ox,oy,oz,voxel,nx,ny,nz (default 32^3 x 5 cm camera-local)")
    parser.add_argument("--geometry-topic-tolerance-s", type=float, default=None,
                        help="kv_topic: the producer's frame with the same depth stamp, else the nearest within this "
                             "(default 0.05 s; 10-29: below 30 fps max(0.05, 1.1 camera periods))")
    # 10-29 owner (Nano CPU): the camera's configured rate (car_inner CAMERA_FPS, passed only when not 30)
    parser.add_argument("--camera-fps", type=float, default=30.0,
                        help="configured colour/depth rate: the colour/depth pair slop (0.02 s; below 30 fps half a "
                             "period) and the default --geometry-topic-tolerance-s follow it")
    parser.add_argument("--geometry-topic-wait-s", type=float, default=0.25,
                        help="kv_topic: bounded wait for the producer's frame after detection is done")
    parser.add_argument("--kv-shadow-check", action="store_true",
                        help="kv only, measurement: also run compute_geometry and record per-frame equality "
                             "(costs Nano CPU; keep off for CPU-usage runs)")
    parser.add_argument("--stall-diag", type=Path,
                        help="opt-in, measurement: new JSONL of per-frame timing (arrival, TF wait, inference, "
                             "poll sections), poll gaps, /tf arrival gaps, GC pauses, GIL watchdog "
                             "(host/runner_stall_diag.py); off = unchanged behaviour")
    parser.add_argument("--tf-reliable", action="store_true",
                        help="reference poses: read the dynamic /tf topic RELIABLE (tf2_ros default, the "
                             "behaviour before 10-12) instead of BEST_EFFORT; /tf_static is reliable + "
                             "transient_local either way (runner_tf_qos)")
    parser.add_argument("--tf-retry", choices=TF_RETRY_MODES, default="arrival",
                        help="reference poses: a frame waiting for its TF is tried again when a new /tf "
                             "sample arrives (arrival, 2026-10-12 v6) or only at the next image pair / "
                             "100 ms poll (poll, the behaviour before). Same lookup either way: exact "
                             "stamp, interpolation between two real samples, no extrapolation, 0.5 s bound")
    parser.add_argument("--record-write", choices=RECORD_WRITE_MODES, default="async",
                        help="--events / --stall-diag files: async (v7, 2026-10-12: a bounded queue written by "
                             "one background thread; a full queue drops the row, counted and reported as a "
                             "record_drops event) or sync (write+flush on the executor thread, the behaviour "
                             "before v7; control arm). Rows and their order are the same either way")
    parser.add_argument("--gc", choices=gc_policy.MODES, default="freeze",
                        help="garbage collector: freeze (default since 2026-10-12 host measurement: gc.collect + "
                             "gc.freeze once the node is built, host/gc_policy.py; gen2 pauses >=20 ms 34 -> 0 "
                             "per 300 s run) or default (unchanged interpreter; control arm). Programmatic args "
                             "without a gc field keep the interpreter default")
    parser.add_argument("--camera-qos", choices=CAMERA_QOS_MODES, default="auto",
                        help="/fpga/camera_info, /fpga/rgb, /fpga/depth subscriptions (runner_camera_qos): reliable = "
                             "RELIABLE KEEP_LAST --camera-qos-depth; sensor_data = best effort depth 5 (before "
                             "2026-10-15); auto (default) = reliable for --sensor-source hardware (the Orbbec "
                             "driver offers RELIABLE), sensor_data for replay/simulation (their cameras publish "
                             "BEST_EFFORT, which a reliable reader would not match)")
    parser.add_argument("--camera-qos-depth", type=int, default=CAMERA_SUB_DEPTH,
                        help="KEEP_LAST depth of the reliable camera subscriptions (default 5, see runner_camera_qos)")
    parser.add_argument("--input-log", type=Path,
                        help="opt-in, measurement (2026-10-15): new CSV, one row per camera message reaching the "
                             "runner's subscription callbacks (topic as resolved after remapping, header stamp ns, "
                             "wall ns, RAW arrival ns: the rx_lite columns), plus <same name>.json at exit with the "
                             "DDS message-lost count per topic; off = unchanged behaviour")
    parser.add_argument("--stall-diag-inject", help=argparse.SUPPRESS)   # positive control: SECTION:EVERY:MS[:free|hold]
    # 10-18 object memory across runs (host/memory_store.py, docs/design/MEMORY_PERSIST_20261018.md); all off = unchanged
    parser.add_argument("--memory-out-dir", type=Path,
                        help="write memory_final.json here at a normal stop (SIGINT/SIGTERM included) and "
                             "memory_snapshot.json while running (on change, at most every --memory-snapshot-s)")
    parser.add_argument("--memory-snapshot-s", type=float, default=5.0,
                        help="minimum gap between periodic snapshots (0 = no snapshots, final file only)")
    parser.add_argument("--memory-in", type=Path,
                        help="load this memory file at start; refused (exit 2, nothing started) when it is "
                             "malformed, of another version, a snapshot, or of another map than --memory-map-identity")
    parser.add_argument("--memory-map-identity", type=Path,
                        help="this run's map identity (memory_store identity, written by car_up.sh); recorded in "
                             "the files written, compared with the loaded file's; omitted = kind none")
    parser.add_argument("--pose-mode", help="car_up --pose (recorded in the memory file only)")
    parser.add_argument("--memory-align", action="store_true",
                        help="10-25 in-stack mapping run (car_inner MAPPING=1): on the mapping stop's alignment "
                             "(host/map_align.TOPIC) move this run's memory into the optimized map frame, once the "
                             "TF map -> odom has become that alignment (host/map_align.apply_memory)")
    parser.add_argument("--live-align", action="store_true",
                        help="10-27 (with --memory-align; car_up CAR_LIVE_ALIGN=1 default): several alignments per "
                             "run (mapping_ctl --live-align): each moves the memory by the difference to the one "
                             "before; every frame is tagged with the map -> odom its pose was looked up with and "
                             "brought into the memory's current alignment before it is merged "
                             "(docs/design/LIVE_ALIGN_20261027.md)")
    # 2026-10-22 find-nav dig: one value for BOTH per-goal clocks -- the runner's wall-clock navigation watchdog and
    # the task controller's navigating budget (ROS time) -- and the task's total budget. Defaults = the simulation's
    # 30/120; the real car gets 60/180 from tools/car_run/car_inner.sh (board).
    parser.add_argument("--navigation-timeout-s", type=float, default=TaskConfig.navigation_timeout_s,
                        help="per-goal navigation budget (s): wall-clock watchdog and task controller")
    parser.add_argument("--task-timeout-s", type=float, default=TaskConfig.task_timeout_s,
                        help="whole-task budget (s) of the task controller")
    # 10-26: processed-frame gap that ends a task as failed:stream_lost (no retry). Default 2.0 = the simulation's;
    # the real car (Nano under load) gets 5.0 from tools/car_run/car_inner.sh (board).
    parser.add_argument("--stream-timeout-s", type=float, default=TaskConfig.stream_timeout_s,
                        help="processed-frame gap (s) that ends a task as stream_lost")
    # 10-26 owner：重成功率，放宽自 200 ms -- the runner's geometry age gate; default 200 = unchanged (host, tests),
    # the real car gets 500 from tools/car_run/car_inner.sh (board). Range (0, 500].
    parser.add_argument("--input-hiccup", choices=INPUT_HICCUP_MODES, default="fault",
                        help="fault: an input hiccup ends the task (default); drop: the frame is dropped (the car)")
    parser.add_argument("--geometry-age-limit-ms", type=float, default=200.0,
                        help="runner geometry age gate (ms; older geometry is used as unavailable)")
    # 10-25 find-retry: a failed goal -> stop, clear costmaps, pause, retry the same goal, then the next pose
    parser.add_argument("--nav-retries-same-goal", type=int, default=TaskConfig.nav_retries_same_goal,
                        help="retries of a failed goal before the next observation pose / search waypoint")
    # 10-26: the cap counts FAILED navigations only (a goal reached costs nothing); --max-nav-attempts is the old
    # name, kept as an alias so older car scripts still start
    parser.add_argument("--max-nav-failures", "--max-nav-attempts", dest="max_nav_failures", type=int,
                        default=TaskConfig.max_nav_failures,
                        help="failed navigations per task (aborted / rejected / timed out / injected; goals reached "
                             "not counted); reset by an explicit candidate selection. Alias: --max-nav-attempts")
    parser.add_argument("--min-attempt-time-s", type=float, default=TaskConfig.min_attempt_time_s,
                        help="no attempt after the first is started with less task time left than this")
    parser.add_argument("--retry-pause-s", type=float, default=TaskConfig.retry_pause_s,
                        help="stopped pause after clearing the costmaps, before the next attempt")
    # 10-30 owner: find verdict knobs (defaults = TaskConfig: 5 frames, 6.0 s, 1.6 m target, 1.2 m candidates)
    parser.add_argument("--not-seen-frames", type=int, default=TaskConfig.not_seen_frames,
                        help="not-seen frames at an observation pose before searching")
    parser.add_argument("--observe-min-s", type=float, default=TaskConfig.observe_min_s,
                        help="minimum time at an observation pose before the not-seen verdict (s)")
    parser.add_argument("--en-route-target-max-distance-m", type=float,
                        default=TaskConfig.en_route_target_max_distance_m,
                        help="seen while driving counts as found for the target itself within this distance (m)")
    parser.add_argument("--en-route-max-distance-m", type=float, default=TaskConfig.en_route_max_distance_m,
                        help="same, for a single same-class search candidate (m)")
    parser.add_argument("--fault-nav-fail-first", type=int, default=None,
                        help="FAULT INJECTION (host simulation): judge the first N successful task navigation "
                             "results failed; default $%s or 0 (off)" % FAULT_NAV_ENV)
    args, ros_args = parser.parse_known_args(argv)
    if args.fault_nav_fail_first is None:
        try:
            args.fault_nav_fail_first = int(fault_nav_default())
        except ValueError:
            parser.error("%s must be an integer" % FAULT_NAV_ENV)
    try:
        task_config(args)
    except ValueError as exc:
        parser.error("--navigation-timeout-s / --task-timeout-s / --stream-timeout-s / retry options: %s" % exc)
    if args.camera_qos_depth < 1:
        parser.error("--camera-qos-depth must be >= 1")
    if not (math.isfinite(args.camera_fps) and args.camera_fps > 0):
        parser.error("--camera-fps must be finite and > 0")
    if args.geometry_topic_tolerance_s is None:
        args.geometry_topic_tolerance_s = camera_pairing(args.camera_fps)[1]
    if getattr(args, "stall_diag_inject", None) and not getattr(args, "stall_diag", None):
        parser.error("--stall-diag-inject needs --stall-diag")
    if args.world_frame != "map":
        parser.error("this version requires the occupancy and goal frame 'map'")
    if not math.isfinite(args.uint16_depth_scale) or args.uint16_depth_scale <= 0:
        parser.error("depth scale must be finite and positive")
    if args.pose_source == "estimated" and (not args.localization_map_id or len(args.localization_map_id) != 64
            or any(c not in "0123456789abcdef" for c in args.localization_map_id)):
        parser.error("estimated poses require --localization-map-id SHA256")
    if args.geometry_source == "kv":
        if not args.kv_server or ":" not in args.kv_server:
            parser.error("--geometry-source kv needs --kv-server HOST:PORT")
        if args.region_source == "kv_only":
            parser.error("--geometry-source kv with --region-source kv_only is contradictory "
                         "(kv_only hands update_memory an empty geometry)")
        if not (args.kv_timeout_s > 0 and args.kv_hello_timeout_s > 0 and args.kv_retry_s >= 0):
            parser.error("kv timeouts must be positive")
    elif args.geometry_source == "kv_topic":
        if args.kv_server or args.kv_expect_build_id or args.kv_grid or args.kv_depth_unit_m is not None:
            parser.error("--geometry-source kv_topic never connects to the KV260; --kv-server/--kv-* belong "
                         "to host/ros_geo_producer.py")
        if args.region_source != "none":
            parser.error("--geometry-source kv_topic with --region-source kv/kv_only would merge the same "
                         "KV260 answers twice (or none)")
        if not (args.geometry_topic_tolerance_s >= 0 and args.geometry_topic_wait_s >= 0):
            parser.error("geometry topic tolerance/wait must be >= 0")
    elif (args.geometry_fallback != "none" or args.kv_server or args.kv_shadow_check
          or args.kv_expect_build_id or args.kv_grid or args.kv_depth_unit_m is not None):
        parser.error("--geometry-fallback/--kv-* only apply to --geometry-source kv/kv_topic")
    if not args.pose_trust_gyro_period_s > 0:
        parser.error("--pose-trust-gyro-period-s must be positive")
    waypoints = load_waypoints(args.waypoints)
    try:
        from host.search_belief import SearchConfig
        args.search_config_obj = (SearchConfig.from_file(str(args.search_config)) if args.search_config
                                  else SearchConfig())
    except (OSError, ValueError, TypeError) as exc:
        parser.error("--search-config: %s" % exc)
    # on args (not main locals): the offline Runtime harnesses bind only the globals they know
    args.memory_identity_doc, args.memory_in_doc = runner_memory_inputs(args, parser)
    if args.frames_dir:
        args.frames_dir.mkdir(parents=True, exist_ok=False)

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from message_filters import Subscriber, ApproximateTimeSynchronizer
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool, Empty, String
    from tf2_ros import Buffer, TransformException, TransformListener
    from tf2_msgs.msg import TFMessage
    from host.inference_service import InferenceClient
    from perception.fpga_replay.core import Frame, Memory, process_frame, compute_geometry

    class Runtime(Node):
        sdiag = None                        # host/runner_stall_diag.py (--stall-diag), off by default
        sdiag_frame = None

        def __init__(self):
            super().__init__("fpga_runtime")
            self.diagnostics = RuntimeDiagnostics(args.diagnostics) if args.diagnostics else None
            self.sdiag = None
            if getattr(args, "stall_diag", None):
                from host.runner_stall_diag import RunnerStallDiag
                self.sdiag = RunnerStallDiag(args.stall_diag, getattr(args, "stall_diag_inject", None),
                                             record_write=getattr(args, "record_write", None) or "async")
            self.diagnostic_attempt = 0
            self.diagnostic_ready = (self.create_publisher(String, '/fpga/diagnostic_infrastructure_ready', 10)
                                     if self.diagnostics else None)
            configured = self.get_parameter("use_sim_time").value
            self.sim_clock = use_simulation_clock(args.sensor_source, configured)
            if self.sim_clock != configured:
                results = self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, self.sim_clock)])
                if not all(result.successful for result in results):
                    raise RuntimeError("cannot configure acquisition clock")
            self.memory, self.task = Memory(cfg=memory_config(args)), None
            from perception.fpga_replay.see_through import SeeThroughFilter   # local: no new Runtime global
            self.see_through = SeeThroughFilter(self.memory.cfg)       # 10-27 geometry v2 only; off = pass-through
            self.geometry_version = "v2" if self.memory.cfg.geometry_v2 else "v1"
            self.geometry_version_seen = None                          # producer's version last seen (kv_topic)
            self.track_ev_last = {}                                    # 10-27: track_ev logs existence changes only
            # 10-18: a memory of an earlier run on this map (checked in main before rclpy started)
            memory_in_doc = getattr(args, "memory_in_doc", None)
            self.memory_identity = getattr(args, "memory_identity_doc", None) or memory_store.map_identity()
            self.memory_loaded = (memory_store.restore(memory_in_doc, self.memory)
                                  if memory_in_doc is not None else None)
            self.memory_loaded_from = (None if memory_in_doc is None else
                                       {"path": str(args.memory_in),
                                        "file_sha256": memory_store.sha256_file(str(args.memory_in))})
            self.task_config = task_config(args)   # one per-goal budget for both clocks (10-22)
            self.memory_snapshots = (memory_store.SnapshotPolicy(args.memory_snapshot_s)
                                     if getattr(args, "memory_out_dir", None) and args.memory_snapshot_s > 0
                                     else None)
            self.observation_views = {}
            self.map = self.camera_info = self.frame = self.base_camera = None
            self.robot_xy = (0., 0.)
            self.nav = None
            self.costmap_clients = {}        # 10-25 find-retry: lazily created ClearEntireCostmap clients
            self.patrol = []
            self.closed = False
            self.result_lock = threading.Lock()
            self.result_guard = (self.create_guard_condition(self.poll)
                                 if getattr(args, "result_wakeup", "poll") == "event" else None)
            # 10-26 owner：重成功率，放宽自 2 s / 1 s（--input-hiccup drop: --stream-timeout-s）
            self.input_limits = input_age_limits(args)
            self.motion = MotionPermit(processed_max_s=self.input_limits[0], acquisition_max_s=self.input_limits[1])
            self.input_drop = getattr(args, "input_hiccup", "fault") == "drop"
            self.input_hiccups = 0      # frames dropped instead of a fault (--input-hiccup drop)
            # 10-29 owner: --input-hiccup drop: no processed frame for --stream-timeout-s during a task/navigation is
            # a warning (processed_stream_stalled / _resumed), not the processed_stream_wall_timeout fault; the wall
            # time the current stall was reported (None = no stall) and how many there were
            self.stream_stall_wall = None
            self.stream_stalls = 0
            self.inference_healthy = False
            self.last_processed_wall = time.monotonic()
            self.processed = self.dropped = 0
            self.last_submitted = -math.inf
            self.acquisitions = AcquisitionQueue()
            self.tf_wait_entry = None           # the queue entry whose last try waited for TF (v6)
            # 10-26 real car: /tf is read best-effort and loses a sample now and then (~1 per 30 s), so a
            # single frame's bounded clock/TF wait expires; during a task that used to fault the task at once.
            # Now only TF_EXPIRY_FAULT_RUN consecutive expiries fault it (the 5 s stream timeout stays the backstop).
            self.tf_expiry_run = 0
            # the queue entry whose last try waited for its localization approval, and the frames
            # dropped because no approval could come for them (LocalizationSkipped / that wait expired)
            self.localization_wait_entry = None
            self.localization_skipped = 0
            # what the last delivered status said, for expired_only_for_localization: accepted by the
            # lease, its state and reason, and whether the lease kept it (healthy) right after it
            self.localization_view = None
            self.tf_retry = None                # --tf-retry in effect (reference poses only)
            self.buffer = Buffer()
            self.localization = (LocalizationLease(args.localization_map_id, args.localization_tracked_frame)
                                 if args.pose_source == "estimated" else None)
            if self.localization:
                # Estimated geometry consumes certified poses plus static extrinsics only.
                # Raw /tf and its cached/interpolated world poses cannot bypass the gate.
                static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                       reliability=ReliabilityPolicy.RELIABLE)
                self.create_subscription(TFMessage, "/tf_static", self.on_static_tf, static_qos)
                self.create_subscription(String, "/fpga/localization_status", self.on_localization, 10)
            else:
                # dynamic /tf best effort unless --tf-reliable; static_qos left to tf2_ros
                tf_reliable = bool(getattr(args, "tf_reliable", False))
                self.listener = TransformListener(self.buffer, self, qos=runner_tf_qos(tf_reliable))
                self.get_logger().info("dynamic /tf reader QoS: %s" % ("reliable" if tf_reliable else "best_effort"))
                # v6: a frame waiting for its TF is retried as soon as a new /tf sample is in the buffer
                self.tf_retry = getattr(args, "tf_retry", None) or "arrival"
                if self.tf_retry == "arrival" and not hook_tf_arrival(self.listener, self.on_tf_arrival):
                    self.tf_retry = "poll"          # no tf_sub to hook: the old retry points only
                    self.get_logger().warning("tf2 listener has no tf_sub callback; TF wait retry falls back to poll")
                self.get_logger().info("TF wait retry: %s" % self.tf_retry)
            self.actions = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.velocity = self.create_publisher(Twist, "/cmd_vel", 10)
            self.permit = self.create_publisher(Bool, "/fpga/motion_permit", 1)
            self.permit.publish(Bool(data=False))
            self.status = self.create_publisher(String, "/fpga/status", 10)
            # v7: events file through a RecordWriter (async default: a bounded queue + one writer
            # thread; sync = the old write+flush here); slow writes land in the stall diagnostics
            self.events = (RecordWriter(open(args.events, "x", encoding="utf8"),
                                        mode=getattr(args, "record_write", None) or "async", name="events",
                                        on_slow=self.sdiag.events.put if self.sdiag is not None else None)
                           if args.events else None)
            if self.sdiag is not None and self.events is not None:
                self.sdiag.peer_writers.append(self.events)
            self.record_drops = DropReporter(1.0)
            self.reporting_drops = False
            self.backend_log = BackendLog()   # 10-29: backend metadata in full once, then by reference
            self.detector_name, expected_detector = detector_selection(args)
            self.client = InferenceClient(args.socket, timeout=10, expected_device=args.inference_device,
                                          expected_detector=expected_detector)
            self.pool = ThreadPoolExecutor(max_workers=1)
            self.pending = None
            self.inference = None
            self.results = queue.Queue(maxsize=1)
            # camera subscriptions (2026-10-15 --camera-qos): one profile for all three, auto = reliable for
            # hardware (live camera), sensor_data (best effort, the behaviour before) for replay / simulation;
            # recorded in /fpga/runtime_info (camera_qos), on stderr, and in the --input-log summary
            self.camera_qos = camera_qos_label(args)
            camera_qos = (runner_camera_qos(True, camera_qos_depth(args)) if camera_reliable(args)
                          else qos_profile_sensor_data)
            # stderr, not the events stream (whose content the replay regressions compare line by line)
            print("camera subscriptions QoS: %s (--camera-qos %s)" % (self.camera_qos, getattr(args, "camera_qos", None)
                                                                      or "auto"), file=sys.stderr, flush=True)
            self.input_log = None
            if getattr(args, "input_log", None):
                self.input_log = RunnerInputLog(
                    args.input_log, RecordWriter(open(args.input_log, "x", encoding="utf8"),
                                                 mode=getattr(args, "record_write", None) or "async",
                                                 name="input", flush_each=False),
                    {"kind": "ros_runtime camera subscriptions", "camera_qos": self.camera_qos,
                     "camera_qos_switch": getattr(args, "camera_qos", None) or "auto"})

            def camera_sub(make, nominal):
                """make(**kw) creates the subscription; with --input-log it also takes the message-lost event
                (rx_lite's way: an rmw without the event -> made without it, recorded as unsupported)."""
                if self.input_log is None:
                    return make()
                try:
                    from rclpy.qos_event import SubscriptionEventCallbacks
                    return make(event_callbacks=SubscriptionEventCallbacks(
                        message_lost=self.input_log.lost_callback(nominal)))
                except Exception as exc:                # noqa: BLE001
                    self.input_log.lost_unsupported(nominal, exc)
                    return make()
            info_sub = camera_sub(lambda **kw: self.create_subscription(
                CameraInfo, "/fpga/camera_info", self.on_info, camera_qos, **kw), "/fpga/camera_info")
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
            self.rgb = camera_sub(lambda **kw: Subscriber(self, Image, "/fpga/rgb", qos_profile=camera_qos, **kw),
                                  "/fpga/rgb")
            self.depth = camera_sub(lambda **kw: Subscriber(self, Image, "/fpga/depth", qos_profile=camera_qos, **kw),
                                    "/fpga/depth")
            if self.input_log is not None:
                for sub, nominal in ((info_sub, "/fpga/camera_info"), (getattr(self.rgb, "sub", None), "/fpga/rgb"),
                                     (getattr(self.depth, "sub", None), "/fpga/depth")):
                    if sub is not None:
                        self.input_log.wrap(sub, nominal)
            slop = camera_pairing(getattr(args, "camera_fps", 30.0))[0]
            self.sync = ApproximateTimeSynchronizer([self.rgb, self.depth], 5, slop)
            self.sync.registerCallback(self.on_images)
            if self.sdiag is not None:          # arrival stamps (wrapped rclpy subscription callbacks)
                for kind, sub in (("depth", getattr(self.depth, "sub", None)), ("rgb", getattr(self.rgb, "sub", None)),
                                  ("tf", getattr(getattr(self, "listener", None), "tf_sub", None))):
                    if sub is not None:
                        self.sdiag.wrap_subscription(sub, kind)
            self.create_subscription(String, "/fpga/query", lambda msg: self.query(msg.data), 10)
            self.create_subscription(String, "/fpga/select_candidate", self.select, 10)
            self.create_subscription(Empty, "/fpga/cancel", self.cancel, 10)
            self.create_subscription(Empty, "/fpga/patrol", self.begin_patrol, 10)
            self.create_subscription(String, "/fpga/goto", self.goto, 10)
            # 10-26「重置地图」(host/map_reset.py): hub op map_reset -> here; answered on the latched status topic
            self.map_reset_job = None
            self.map_reset_count = 0
            self.map_reset_hold_until = None     # frames acquired before this ROS time are not merged
            self.create_subscription(String, map_reset.REQUEST_TOPIC, self.on_map_reset, 10)
            self.map_reset_status = self.create_publisher(String, map_reset.STATUS_TOPIC, QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            self.mapping_reset_topic = self.create_publisher(String, map_reset.MAPPING_RESET_TOPIC, 10)
            self.create_timer(.2, self.check_map_reset, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.goto_count = 0
            self.approach_pending = None     # 10-26 find-near: {"command", "task", "since"} until the goal can start
            self.mapping_waypoints = None
            self.mapping_nodes = None            # 10-26 dense trajectory nodes (probabilistic search)
            # 10-29 find while mapping: the car's own trajectory (search nodes before an export exists), rtabmap's
            # live grid (planning map while mapping), a memory move the task has not yet been told about
            self.live_track = LiveTrack()
            self.live_grid = None                # (FreeMap, monotonic arrival)
            self.live_grid_problem = None
            self.map_moved_pending = False
            self.search_planner = None           # 10-26 the active task's BeliefSearch (None = MVP list)
            self.search_planner_cache = {}
            self.search_goals_recorded = 0
            self.search_belief_topic = self.create_publisher(String, SEARCH_BELIEF_TOPIC, QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            # 10-26 search heat map for the console: preview / search (<= 1 Hz) / done (host/search_belief_feed.py)
            self.belief_feed = BeliefFeed(self.publish_belief, on_error=lambda exc: self.record(
                "search_belief_publish_failed", reason=repr(exc)))
            # 10-25: mapping_ctl's status, ONE subscription for both uses (on_mapping_status): the map-switch gate
            # (no find/select while the exported map is being saved/aligned/loaded; always on) and, with
            # --search-waypoints-from-mapping, the trajectory's search goals
            self.mapping_status = None           # (row, monotonic arrival)
            self.create_subscription(String, MAPPING_STATUS_TOPIC, self.on_mapping_status, QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            # 10-25 map alignment at the mapping stop (host/map_align.py; owner plan A)
            # (10-27 live alignment: + the alignment the memory is in, its seq, frame counts -- host/live_align.py)
            live_align.new_runner_state(self)
            self.live_align = bool(getattr(args, "live_align", False)) and bool(getattr(args, "memory_align", False))
            if self.live_align and getattr(args, "search_waypoints_from_mapping", False):
                # rtabmap's grid publisher is latched (transient local), as tools/car_run/rtabmap_watch.py reads it
                self.create_subscription(OccupancyGrid, LIVE_GRID_TOPIC, self.on_live_grid, QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            if getattr(args, "memory_align", False):
                self.create_subscription(String, map_align.TOPIC, self.on_memory_align, QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
                self.create_timer(.2, self.check_memory_align, clock=Clock(clock_type=ClockType.STEADY_TIME))
            # /fpga/teleop/active (2026-10-18, teleop/Nav2 arbitration; host/ros_teleop_safe.py --nav-topic): while
            # the remote control is in hand, NEW navigation is refused with reason teleop_active (controller ruling:
            # refused, never held back and run after the release). No publisher (every other run) = never active.
            self.teleop_sample = None      # (in hand, monotonic receipt)
            self.create_subscription(Bool, TELEOP_ACTIVE_TOPIC, self.on_teleop_active, 10)
            # /fpga/memory (spec D.1) is an added topic; --region-source none keeps every other behavior.
            self.region_source = getattr(args, "region_source", "none")
            self.runtime_boot_id = new_boot_id()
            self.memory_feed = MemoryFeed(self.runtime_boot_id, args.world_frame, args.sensor_source)
            self.memory_topic = self.create_publisher(String, "/fpga/memory", map_qos)
            self.region = None
            if self.region_source != "none":
                self.region = RegionMerger(self.runtime_boot_id, args.world_frame,
                                           getattr(args, "max_region_age_s", 1.0))
                self.create_subscription(String, "/fpga/region_verdicts", self.on_region, 10)
            self.pose_trust = None
            if getattr(args, "pose_trust", "off") == "on":
                from nav_msgs.msg import Odometry
                from sensor_msgs.msg import Imu
                from perception.fpga_replay.pose_trust import PoseTrustMonitor
                self.pose_trust = PoseTrustMonitor(nominal_period_s=args.pose_trust_gyro_period_s)
                self.create_subscription(Imu, args.pose_trust_imu_topic, self.on_pose_trust_imu,
                                         qos_profile_sensor_data)
                self.create_subscription(Odometry, args.pose_trust_odom_topic, self.on_pose_trust_odom,
                                         qos_profile_sensor_data)
                if getattr(args, "pose_trust_slip_topic", ""):
                    self.create_subscription(String, args.pose_trust_slip_topic, self.on_pose_trust_slip, 10)
            self.geometry_source = getattr(args, "geometry_source", "cpu")
            # RAW arrival of every image pair (all sources): origin of the 200 ms geometry age
            # gate at update_memory (host/geometry_age_gate.py); kv/kv_topic setups keep it.
            self.kv_now_ns = raw_now_ns
            # 10-26 owner：重成功率，放宽自 200 ms（车上 --geometry-age-limit-ms 500）
            self.age_gate = GeometryAgeGate(int(round(getattr(args, "geometry_age_limit_ms", 200.0) * 1e6)))
            if self.geometry_source == "kv":
                self.kv_setup()
            elif self.geometry_source == "kv_topic":
                self.kv_topic_setup()
            # A steady timer still runs when /clock pauses or disappears.
            self.watchdog = self.create_timer(.1, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.record("startup", pose_source=args.pose_source, waypoints=[p.__dict__ for p in waypoints],
                        result_wakeup=getattr(args, "result_wakeup", "poll"),
                        camera_fps=getattr(args, "camera_fps", 30.0),       # 10-29: and what follows from it
                        pair_slop_s=camera_pairing(getattr(args, "camera_fps", 30.0))[0],
                        geometry_topic_tolerance_s=getattr(args, "geometry_topic_tolerance_s", None),
                        detector=self.detector_name, view_refinement="none", memory_tracks=self.memory.snapshot(),
                        task_active=self.active(), patrol=list(self.patrol),
                        **task_config_record(self.task_config))   # 10-25: the effective budgets / retry options
            if self.memory_loaded is not None:
                self.record("memory_loaded", path=str(args.memory_in),
                    file_sha256=self.memory_loaded_from["file_sha256"], from_runtime_boot_id=memory_in_doc["runtime_boot_id"],
                    written_wall=memory_in_doc["written_wall"], map_identity_kind=self.memory_identity["kind"],
                    identity=memory_store.identity_match(memory_in_doc["map_identity"], self.memory_identity)[1],
                    **self.memory_loaded)
            # v7: garbage-collector policy once the node is built (default: unchanged)
            self.gc_info = gc_policy.apply(getattr(args, "gc", None) or "default")
            if self.gc_info["mode"] != "default":
                self.get_logger().info("gc policy: %s" % json.dumps(self.gc_info, sort_keys=True))
            # latched start-up facts (v5): late joiners (the hub) still get them
            self.runtime_info_topic = self.create_publisher(String, RUNTIME_INFO_TOPIC, map_qos)
            self.runtime_info = runtime_info_body(args, self.runtime_boot_id, self.geometry_source, raw_now_ns(),
                                                  self.age_gate.limit_ns / 1e6, tf_retry=self.tf_retry,
                                                  gc_mode=self.gc_info["mode"])
            self.runtime_info_topic.publish(String(data=json.dumps(self.runtime_info, allow_nan=False)))
            if self.geometry_source == "kv":
                self.record("geometry_source", source="kv", fallback=args.geometry_fallback,
                            shadow_check=bool(args.kv_shadow_check), **self.kv.describe())
            elif self.geometry_source == "kv_topic":
                self.record("geometry_source", source="kv_topic", fallback=args.geometry_fallback,
                            shadow_check=bool(args.kv_shadow_check), topic="/fpga/geometry_frame",
                            tolerance_s=args.geometry_topic_tolerance_s, wait_s=args.geometry_topic_wait_s)
            self.record("geometry_version", version=self.geometry_version, source=self.geometry_source,
                        existence_v3=bool(self.memory.cfg.existence_v3),
                        one_per_class=bool(getattr(self.memory.cfg, "one_per_class", False)),
                        see_through_filter=bool(self.see_through.active()),
                        miss_turn_gate=bool(getattr(self.memory.cfg, "miss_turn_gate", False)),
                        miss_max_turn_dps=getattr(self.memory.cfg, "miss_max_turn_dps", None),
                        miss_reliable=getattr(self.memory.cfg, "miss_reliable", None),
                        miss_reliable_vote=getattr(self.memory.cfg, "miss_reliable_vote", None),
                        track_ev_legend={"value": "[raw word, word given to memory, existence or null, "
                                                  "1 = filter no longer believes this track's free]",
                                         "words": TRACK_EV_CODES})

        def sdiag_tf_latest(self):
            """Newest stamp (ns) at which this runner's TF buffer can give world->camera (stall
            diagnostics only: how far the runner's TF lags the image it waits for)."""
            if self.localization:
                return None
            try:
                t = self.buffer.lookup_transform(args.world_frame, args.camera_frame, Time())
                return Time.from_msg(t.header.stamp).nanoseconds
            except Exception:
                return None

        def kv_setup(self):
            """--geometry-source kv: one kv-link session (host/kv_geometry.py). A failed handshake
            does not stop the runtime; every frame then reports geometry unavailable (or the
            explicit CPU fallback) until a rate-limited reconnect succeeds."""
            from host.kv_geometry import DEFAULT_GRID, KvGeometry, parse_build_id, parse_grid
            from host.fpga_clock import now_ns
            host, port = args.kv_server.rsplit(":", 1)
            unit = args.kv_depth_unit_m if args.kv_depth_unit_m is not None else args.uint16_depth_scale
            self.kv_now_ns = now_ns
            self.kv = KvGeometry((host, int(port)), depth_unit_m=unit, timeout_s=args.kv_timeout_s,
                                 hello_timeout_s=args.kv_hello_timeout_s, retry_s=args.kv_retry_s,
                                 grid=parse_grid(args.kv_grid) if args.kv_grid else DEFAULT_GRID,
                                 expect_build_id=parse_build_id(args.kv_expect_build_id))
            self.kv_codes_ok = args.kv_depth_unit_m is None or args.kv_depth_unit_m == args.uint16_depth_scale
            self.kv_arrivals = {}
            self.kv_state = None            # (available, reason) last recorded, for transition events
            self.kv_shadow = {"frames": 0, "equal": 0}
            self.kv_stats_topic = self.create_publisher(String, "/fpga/kv_link/frame_stats", 10)
            self.kv.connect()

        def kv_topic_setup(self):
            """--geometry-source kv_topic (host/geometry_topic.py): bodies from the producer node,
            keyed by depth header stamp; no KV260 connection in this process."""
            from host.geometry_topic import GeometryFrameCache
            from host.fpga_clock import now_ns
            self.kv_now_ns = now_ns
            self.kv_topic = GeometryFrameCache(self.runtime_boot_id, args.world_frame,
                                               tolerance_ns=int(round(args.geometry_topic_tolerance_s * 1e9)))
            self.kv_hold = None             # (frame, future, first try RAW ns) waiting for its body
            self.kv_state = None
            self.kv_shadow = {"frames": 0, "equal": 0}
            self.create_subscription(String, "/fpga/geometry_frame", self.on_geometry_frame, 50)

        def on_geometry_frame(self, message):
            self.kv_topic.on_message(message.data, self.kv_now_ns())
            if self.kv_hold is not None:
                self.poll()                 # a held recognition frame may be complete now

        def now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def record(self, kind, **fields):
            now = self.now()
            meta = self.client.last_metadata if hasattr(self, "client") else None
            row = {"event": kind, "time": now, "pose_source": args.pose_source,
                   "sensor_source": args.sensor_source, "inference_device": args.inference_device,
                   "clock_source": "ros_simulation" if self.sim_clock else "system_ros",
                   # 10-29: full metadata on first use / change / every 60 s, else a short row with
                   # backend_ref (host/backend_log.py; readers: BackendExpander / expand_rows)
                   "inference_backend": (self.backend_log.field(meta, now) if hasattr(self, "backend_log")
                                         else meta),
                   **fields}
            text = json.dumps(row, ensure_ascii=False, allow_nan=False)
            if self.sdiag is None:
                if self.events:                 # the file row first: a failing publish must not lose it
                    self.events.write(text + "\n")     # v7: queued (async) or written+flushed (sync)
                self.status.publish(String(data=text))
                self.note_record_drops()
                return
            # stall diagnostics: same steps, timed (frame_processed as poll sections, any other
            # kind as a slow_record row when its file write or publish takes >= 50 ms)
            timed = kind == "frame_processed"
            if timed:
                self.sdiag.mark("record_dumps")
                self.sdiag.maybe_inject("record")
            t0 = self.sdiag.clock()
            if self.events:
                self.events.write(text + "\n")
            if timed:
                self.sdiag.mark("record_write")
            t1 = self.sdiag.clock()
            self.status.publish(String(data=text))
            if timed:
                self.sdiag.mark("record_publish")
            else:
                self.sdiag.record_timing(kind, t0, t1, self.sdiag.clock())
            self.note_record_drops()

        def note_record_drops(self, final=False):
            """v7: rows the async writers dropped (queue full) are never silent: a record_drops
            event on /fpga/status (hub STATE task.record_drops; forwarded as EVENT) at the first
            drop, then at most once a second while the count grows, and at close."""
            if self.reporting_drops:
                return
            ev = self.events.counts["dropped"] if self.events else 0
            sd = self.sdiag.out.counts["dropped"] if self.sdiag is not None and self.sdiag.out is not None else 0
            if not self.record_drops.due(ev + sd, time.monotonic(), final=final):
                return
            self.reporting_drops = True
            try:
                self.record("record_drops", process="runner", dropped=ev + sd, events_dropped=ev,
                            events_dropped_bytes=self.events.counts["dropped_bytes"] if self.events else 0,
                            stall_diag_dropped=sd)
            finally:
                self.reporting_drops = False

        def active(self):
            return self.task is not None and self.task.state not in self.task.TERMINAL

        def stop(self):
            self.motion.stop()
            self.permit.publish(Bool(data=False))
            self.patrol = []
            self.approach_pending = None
            self.velocity.publish(Twist())
            if self.nav:
                self.nav["cancelled"] = True
                self.cancel_navigation(self.nav)

        def cancel_navigation(self, context):
            handle = context.get("handle")
            if handle is None or not handle.accepted:
                return
            # A cancellation request/response is not a terminal action result.
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.record("navigation_cancel_transport_failed", command_id=context["id"], detail=str(exc))

        def navigation_authorized(self, context):
            if context is not self.nav or context["cancelled"]:
                return False
            if not self.localization_ok():
                return False
            lease = self.localization
            current = (lease.generation, lease.row['instance'], lease.row['epoch']) if lease else None
            return current == context["authorization"]

        def correction_fault(self, context):
            """D-035: a same-map correction past the shared limits revokes the goal.

            Checked only while the authorization triple still holds; the rule and limits are
            host.nav_localization's, shared with the point-to-point consumer. The M5 task
            fails closed with this reason, like any localization continuity loss (M5_HANDOFF
            §4 item 4, §9.1); re-planning on the new map is the caller's new query.
            """
            if (self.localization is None or context is None or context is not self.nav
                    or context["cancelled"] or not self.navigation_authorized(context)):
                return None
            return lease_correction_verdict(self.localization, context.get("correction_reference"),
                                            time.monotonic(), self.get_clock().now().nanoseconds)

        def fault(self, reason):
            self.inference_healthy = False
            if self.active():
                try:
                    self.execute(self.task.fail(reason, self.now()))
                except ValueError:
                    # Clock reset invalidates all acquisition comparisons.
                    self.task = None
            self.stop()
            self.record("fault", reason=reason)
            self.publish_task()

        def diagnostic(self, event, **fields):
            if self.diagnostics:
                self.diagnostics.record(event, queue=queue_snapshot(self.acquisitions),
                    lease=lease_snapshot(self.localization), last_submitted=self.last_submitted,
                    attempt_id=self.diagnostic_attempt, **fields)

        def diagnostic_ack(self):
            d = self.diagnostics
            if d and not d.ack_sent and d.infrastructure_ready(args.base_frame, args.localization_tracked_frame, args.camera_frame):
                d.ack_sent = True
                self.diagnostic('infrastructure_ack')
                self.diagnostic_ready.publish(String(data=json.dumps({'event': 'infrastructure_ready',
                    'monotonic_ns': time.monotonic_ns(), 'static_edges': sorted(d.static_edges),
                    'map_received': d.map_seen, 'status_received': d.status_seen})))

        def on_mapping_status(self, msg):
            """/fpga/mapping/status (latched, re-sent every 1 s): the row and its arrival for the map-switch gate
            (map_switching); with --search-waypoints-from-mapping also the trajectory goals, taken when new, never
            cleared."""
            try:
                row = json.loads(msg.data)
            except ValueError:
                row = None
            if isinstance(row, dict):
                self.mapping_status = (row, time.monotonic())
            if getattr(self, "map_reset_job", None) is not None:
                return      # 10-26 map reset running: rows of the old session must not refill the cleared goals
            if not getattr(args, "search_waypoints_from_mapping", False):
                return
            poses = mapping_search_waypoints(msg.data)
            if poses and poses != getattr(self, "mapping_waypoints", None):
                self.mapping_waypoints = poses
                self.record("search_waypoints_updated", source="mapping_trajectory",
                            waypoints=[p.__dict__ for p in poses])
            nodes = mapping_search_nodes(msg.data)
            if nodes and nodes != getattr(self, "mapping_nodes", None):
                self.mapping_nodes = nodes
                self.record("search_nodes_updated", source="mapping_trajectory", count=len(nodes))

        def search_goals(self, purpose):
            """select_search_waypoints at the current base pose (ready() holds: frame and base_camera exist).
            Simulation (neither flag): the fixed list filtered by FreeMap.free, exactly as before 10-25."""
            fallback = getattr(args, "search_fallback", "fixed")
            if not getattr(args, "search_waypoints_from_mapping", False) and fallback == "fixed":
                return [p for p in waypoints if self.map.free(p.x, p.y)]
            base = self.frame.T_world_camera @ np.linalg.inv(self.base_camera)
            pose = (float(base[0, 3]), float(base[1, 3]), math.atan2(base[1, 0], base[0, 0]))
            _nodes, mapped, nodes_source = self.search_trajectory()
            free_map, map_source = self.planning_map()
            goals, info = select_search_waypoints(waypoints, mapped, fallback, free_map, pose,
                                                  outward=purpose == "search")
            if mapped is not None and nodes_source == "live_track":
                info = dict(info, trajectory="live_track", map_source=map_source)
            self.record("search_waypoints_selected", purpose=purpose, goals=[p.__dict__ for p in goals], **info)
            return goals

        # ---------------------------------------------------------------- 10-29 find while mapping
        def mapping_in_progress(self):
            """True while mapping_ctl's (fresh) status says state "mapping" (no export loaded yet)."""
            row, at = self.mapping_status or (None, None)
            return (isinstance(row, dict) and row.get("state") == "mapping" and at is not None
                    and 0 <= time.monotonic() - at <= MAPPING_STATUS_MAX_AGE_S)

        def planning_map(self):
            """(FreeMap, source): rtabmap's live grid while mapping (LIVE_GRID_TOPIC note), else /map."""
            grid = self.live_grid
            if grid is not None and self.mapping_in_progress():
                return grid[0], "live_grid"
            return self.map, "map"

        def search_trajectory(self):
            """(dense nodes or None, waypoint PoseGoals or None, source): the export's (mapping_ctl status) when
            there, else -- mapping run, before the stop -- this runner's own trajectory (host/trajectory_nodes)."""
            nodes = getattr(self, "mapping_nodes", None)
            mapped = getattr(self, "mapping_waypoints", None)
            if nodes or mapped:
                return nodes, mapped, "mapping_trajectory"
            if getattr(args, "search_waypoints_from_mapping", False) and len(self.live_track):
                dense, wps = self.live_track.nodes()
                goals = tuple(PoseGoal(float(x), float(y), float(yaw)) for x, y, yaw in wps) or None
                return dense, goals, "live_track"
            return None, None, None

        def on_live_grid(self, message):
            """LIVE_GRID_TOPIC: rtabmap's grid while mapping -> self.live_grid. A bad grid is recorded (once per
            reason) and the previous one kept: never a fault, the planning falls back to /map."""
            try:
                if message.header.frame_id != args.world_frame:
                    raise ValueError("frame %r, not %r" % (message.header.frame_id, args.world_frame))
                origin = message.info.origin
                q = origin.orientation
                if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or abs(q.z*q.z+q.w*q.w-1) > 1e-5:
                    raise ValueError("nonplanar/invalid map origin")
                grid = FreeMap(np.asarray(message.data, dtype=np.int8).reshape(message.info.height, message.info.width),
                               message.info.resolution, origin.position.x, origin.position.y, 2*math.atan2(q.z, q.w))
            except Exception as exc:                    # noqa: BLE001
                reason = str(exc)
                if reason != self.live_grid_problem:
                    self.live_grid_problem = reason
                    self.record("live_grid_ignored", topic=LIVE_GRID_TOPIC, reason=reason)
                return
            first = self.live_grid is None
            self.live_grid = (grid, time.monotonic())
            if first or self.live_grid_problem is not None:
                self.live_grid_problem = None
                self.record("live_grid_received", topic=LIVE_GRID_TOPIC, width=int(message.info.width),
                            height=int(message.info.height), resolution=float(message.info.resolution))

        def after_memory_move(self, prev, new):
            """The memory was moved from alignment ``prev`` to ``new`` (live or stop alignment): move what else this
            runner keeps in map coordinates -- its trajectory, the remembered observation views -- by the same
            per-stamp difference, and tell the running task (poll: TaskController.map_moved)."""
            try:
                delta = map_align.Delta(new, prev)
                self.live_track.move(delta.at)
                for view in self.observation_views.values():
                    stamp = view.get("stamp")
                    if stamp is None:
                        continue
                    d = delta.at(float(stamp))
                    p = view["pose"]
                    xy = d[:3, :3] @ np.array([p.x, p.y, 0.0]) + d[:3, 3]
                    yaw = p.yaw + math.atan2(d[1, 0], d[0, 0])
                    view["pose"] = PoseGoal(float(xy[0]), float(xy[1]), math.atan2(math.sin(yaw), math.cos(yaw)),
                                            p.frame)
            except Exception as exc:                    # noqa: BLE001 - recorded; memory and task go on
                self.record("map_move_followers_failed", reason=repr(exc))
            self.map_moved_pending = True

        def on_memory_align(self, msg):
            """The mapping stop's alignment (latched): held until this node's TF shows map -> odom = its map_odom."""
            try:
                doc = json.loads(msg.data)
                map_align.Corrections(doc)                  # validates map_odom and nodes
                key = doc.get("key")
            except Exception as exc:                        # noqa: BLE001 - recorded, memory untouched
                self.record("memory_align_refused", reason=repr(exc))
                return
            if key in self.memory_align_done or (self.memory_align_pending or {}).get("key", object()) == key:
                return
            if self.localization is not None:
                self.record("memory_align_refused", key=key, reason="estimated pose source: map -> odom is the gate's")
                return
            if self.live_align:                             # 10-27: ordered by seq, held until TF shows it
                what, fields = live_align.runner_receive(self, doc, key, time.monotonic())
                if what == "stale":
                    self.record("memory_align_stale", **fields)
                    return
                self.record("memory_align_received", map_odom=doc["map_odom"], nodes=len(doc["nodes"]), **fields)
                return
            self.memory_align_pending = {"doc": doc, "key": key, "t": time.monotonic()}
            self.record("memory_align_received", key=key, map_odom=doc["map_odom"], nodes=len(doc["nodes"]))

        def check_memory_align(self):
            """Apply the held alignment in the executor thread (the only thread that changes memory) when the TF
            map -> odom equals it; 30 s without that -> skipped (memory stays consistent with the unchanged TF).
            A frame whose pose was looked up before the switch and merged after it writes one observation in the old
            frame; the object's next observation replaces that position."""
            p = self.memory_align_pending
            if p is None:
                return
            want = [float(v) for v in p["doc"]["map_odom"]]
            try:
                t = self.buffer.lookup_transform(args.world_frame, "odom", Time()).transform
                seen = [t.translation.x, t.translation.y, t.translation.z,
                        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
            except Exception:                               # noqa: BLE001 - not (yet) in the buffer
                seen = None
            if seen is not None and all(abs(a - b) <= 1e-6 for a, b in zip(seen, want)):
                self.apply_memory_alignment(p)
            elif time.monotonic() - p["t"] > 30.0:
                self.memory_align_pending = None
                self.memory_align_done.add(p["key"])
                self.record("memory_align_skipped", key=p["key"], seen=seen,
                            reason="TF map -> odom did not become the alignment in 30 s; memory left as it was")

        def apply_memory_alignment(self, p):
            """Move the memory to the held alignment p (TF map -> odom has become it). 10-27 live: by the difference
            to the alignment the memory is in (host/live_align.runner_apply)."""
            if self.live_align:
                prev = self.memory_align_cur
                info = live_align.runner_apply(self, self.memory)
                self.record("memory_aligned", **info)
                self.after_memory_move(prev, self.memory_align_cur)
            else:
                self.memory_align_pending = None
                self.memory_align_done.add(p["key"])
                info = map_align.apply_memory(self.memory, p["doc"])
                self.record("memory_aligned", key=p["key"], **info)
                self.after_memory_move(None, p["doc"])
            self.publish_memory()

        def map_odom_tag(self):
            """10-27 live: map -> odom in this node's TF buffer now (4x4), None when not there."""
            try:
                return transform_matrix(self.buffer.lookup_transform(args.world_frame, "odom", Time()).transform)
            except Exception:                               # noqa: BLE001 - untagged frame (merged as before)
                return None

        def align_frame(self, frame):
            """10-27 live: the frame into the memory's current alignment before it is merged (a frame tagged with the
            held alignment applies it first) -- host/live_align.runner_align_frame."""
            prev = self.memory_align_cur
            info = live_align.runner_align_frame(self, self.memory, frame)
            if info is not None:
                self.record("memory_aligned", by_frame=frame.frame_id, **info)
                self.after_memory_move(prev, self.memory_align_cur)
                self.publish_memory()

        # ---------------------------------------------------------------- 10-26「重置地图」(host/map_reset.py)
        def map_reset_holds(self, frame):
            """True: this frame is not merged into the memory (a reset is running, or the frame was acquired
            before the reset finished)."""
            job = self.map_reset_job
            if job is not None and job.holding_memory:
                job.frames_held += 1
                return True
            until = self.map_reset_hold_until
            if until is not None:
                if frame.timestamp < until:
                    return True
                self.map_reset_hold_until = None
            return False

        def publish_map_reset_status(self, doc):
            try:
                self.map_reset_status.publish(String(data=json.dumps(doc, ensure_ascii=False, allow_nan=False)))
            except Exception as exc:                # noqa: BLE001 - recorded; the hub then answers unknown
                self.record("map_reset_status_publish_failed", reason=repr(exc))

        def on_map_reset(self, message):
            """/fpga/map_reset {"id"}: stop, clear the memory (new scope) and the map caches, ask mapping_ctl to
            restart the map; check_map_reset finishes when it answers."""
            try:
                req = map_reset.parse_request(message.data)
            except ValueError as exc:
                self.record("map_reset_refused", refused="bad_request", reason=str(exc))
                return
            row, at = self.mapping_status or (None, None)
            key = map_reset.refusal(row, None if at is None else time.monotonic() - at,
                                    self.map_reset_job is not None, self.localization is not None)
            if key is not None:
                self.record("map_reset_refused", id=req["id"], refused=key, reason_zh=map_reset.REFUSE_ZH[key],
                            mapping_state=(row or {}).get("state"))
                self.publish_map_reset_status(map_reset.refused_status(req["id"], key))
                return
            job = self.map_reset_job = map_reset.MapResetJob(req["id"], clock=time.monotonic, wall=self.now)
            self.map_reset_count += 1
            self.record("map_reset_started", id=job.id, by=req.get("by"), count=self.map_reset_count,
                        mapping_state=row.get("state"), mapping_session=row.get("session"),
                        task_state=self.task.state if self.task else None, navigating=self.nav is not None,
                        patrol=len(self.patrol), tracks=len(self.memory.tracks))
            self.map_reset_step(job, "stop", self.map_reset_stop)
            self.map_reset_step(job, "memory", self.map_reset_memory)
            self.map_reset_step(job, "caches", self.map_reset_caches)
            job.begin()
            try:
                self.mapping_reset_topic.publish(String(data=json.dumps(
                    {"id": job.id, "align_key": job.align_key}, sort_keys=True)))
                job.mapping_sent_at = time.monotonic()
                self.record("map_reset_mapping_requested", id=job.id, align_key=job.align_key)
            except Exception as exc:                # noqa: BLE001 - the step fails, the others still run
                self.map_reset_record_step(job, job.step("mapping", False, "请求没有发出：%s" % exc))
                self.finish_map_reset(job)
                return
            self.publish_map_reset_status(job.status())

        def map_reset_step(self, job, name, fn):
            job.begin()
            try:
                ok, reason_zh, detail = fn(job)
            except Exception as exc:                # noqa: BLE001 - reported, not rolled back
                ok, reason_zh, detail = False, "%s: %s" % (type(exc).__name__, exc), {}
            self.map_reset_record_step(job, job.step(name, ok, reason_zh, **detail))

        def map_reset_record_step(self, job, row):
            self.record("map_reset_step", id=job.id, **row)

        def map_reset_stop(self, job):
            info = {"task_state": self.task.state if self.task else None, "task_active": self.active(),
                    "navigation": self.nav is not None, "patrol": len(self.patrol),
                    "approach_pending": self.approach_pending is not None}
            self.cancel(None)               # the existing stop path: task cancel + stop() + task row
            return True, None, info

        def map_reset_memory(self, job):
            old = self.runtime_boot_id
            tracks = len(self.memory.tracks)
            self.memory = Memory(cfg=memory_config(args))
            from perception.fpga_replay.see_through import SeeThroughFilter
            self.see_through = SeeThroughFilter(self.memory.cfg)
            self.track_ev_last = {}
            self.memory_loaded = self.memory_loaded_from = None
            self.runtime_boot_id = new_boot_id()
            self.memory_feed = MemoryFeed(self.runtime_boot_id, args.world_frame, args.sensor_source)
            if self.region is not None:
                self.region = RegionMerger(self.runtime_boot_id, args.world_frame,
                                           getattr(args, "max_region_age_s", 1.0))
            if getattr(self, "kv_topic", None) is not None:
                self.kv_topic.runtime_boot_id = self.runtime_boot_id
            job.previous_scope, job.scope = old, self.runtime_boot_id
            self.runtime_info = dict(self.runtime_info, runtime_boot_id=self.runtime_boot_id,
                                     map_reset={"count": self.map_reset_count, "id": job.id,
                                                "previous_runtime_boot_id": old})
            self.runtime_info_topic.publish(String(data=json.dumps(self.runtime_info, allow_nan=False)))
            self.publish_memory()
            return True, None, {"tracks_cleared": tracks, "previous_scope": old, "scope": self.runtime_boot_id}

        def map_reset_caches(self, job):
            cleared = {"search_waypoints": self.mapping_waypoints is not None,
                       "search_nodes": self.mapping_nodes is not None,
                       "search_planner_cache": len(self.search_planner_cache),
                       "observation_views": len(self.observation_views),
                       "memory_align_pending": self.memory_align_pending is not None}
            self.mapping_waypoints = self.mapping_nodes = None
            cleared["live_track"] = len(self.live_track)
            self.live_track.clear()         # 10-29: the old map's trajectory (the live grid restarts with rtabmap)
            self.live_grid = None
            self.search_planner, self.search_planner_cache = None, {}
            self.search_goals_recorded = 0
            self.observation_views = {}
            # mapping_ctl's identity map -> odom is not a memory move; 10-27: the memory is in no alignment any more
            live_align.runner_reset(self, job.align_key)
            # self.map stays: it mirrors map_server's /map, which mapping_ctl puts back to the start map when the
            # stop had swapped it (a new /map replaces it in on_map); during mapping it is the blank start map
            return True, None, cleared

        def check_map_reset(self):
            job = self.map_reset_job
            if job is None or job.mapping_sent_at is None:
                return
            row = (self.mapping_status or (None,))[0]
            done, ok, reason_zh, session = map_reset.mapping_answer(row, job.id)
            if done:
                job.mapping_session = session
                self.map_reset_record_step(job, job.step(
                    "mapping", ok, None if ok else (reason_zh or "建图进程报告重置失败"),
                    session=session, method=(row.get("reset") or {}).get("method"),
                    mapping_state=row.get("state"), mapping_steps=(row.get("reset") or {}).get("steps")))
            elif job.mapping_timed_out():
                self.map_reset_record_step(job, job.step(
                    "mapping", False, "建图进程 %.0f 秒内没有回应（mapping_ctl 在运行吗？）" % map_reset.MAPPING_WAIT_S,
                    mapping_state=(row or {}).get("state")))
            else:
                return
            self.finish_map_reset(job)

        def finish_map_reset(self, job):
            self.map_reset_hold_until = self.now()       # frames acquired before this are not merged
            try:
                self.map_reset_caches(job)                # again: anything refilled while mapping_ctl worked
            except Exception as exc:                      # noqa: BLE001 - the first pass is the step of record
                self.record("map_reset_caches_second_pass_failed", id=job.id, reason=repr(exc))
            self.map_reset_step(job, "costmaps", self.map_reset_costmaps)
            self.map_reset_step(job, "display", self.map_reset_display)
            state = job.finish()
            self.map_reset_job = None
            self.record("map_reset_done", id=job.id, state=state, reason_zh=job.reason_zh(),
                        elapsed_s=round(time.monotonic() - job.t0, 2), steps=job.steps, scope=job.scope,
                        previous_scope=job.previous_scope, mapping_session=job.mapping_session,
                        frames_held=job.frames_held)
            self.publish_memory()
            self.publish_map_reset_status(job.status())

        def map_reset_costmaps(self, job):
            before = len(self.costmap_clients)
            self.clear_costmaps()           # asynchronous; costmap_clear_requested / _skipped rows say what happened
            return True, None, {"clients": len(self.costmap_clients), "new_clients": len(self.costmap_clients) - before}

        def map_reset_display(self, job):
            msg = self.belief_feed.clear("map_reset")     # phase none: hub MAP which=belief -> the page clears
            return True, None, {"belief": "cleared", "seq": msg["seq"]}

        def on_static_tf(self, message):
            if self.diagnostics:
                self.diagnostic('static_tf_callback')
            for transform in message.transforms:
                # A static map/odom edge must never supply localization.
                if {transform.header.frame_id, transform.child_frame_id} & {args.world_frame, "odom"}:
                    continue
                self.buffer.set_transform_static(transform, "robot_extrinsics")
                if self.diagnostics:
                    edge = (transform.header.frame_id, transform.child_frame_id)
                    self.diagnostics.static_edges.add(edge)
                    self.diagnostic('static_tf_inserted', edge=edge)
            self.diagnostic_ack()

        def localization_ok(self, frame=None):
            if self.localization is None:
                return True
            wall, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
            if frame is None:
                result = self.localization.check(wall, ros_ns)
            else:
                result = self.localization.permits(getattr(frame, "localization_token", None), wall, ros_ns)
            if self.diagnostics:
                self.diagnostic('lease_check' if frame is None else 'lease_permits',
                    input_wall=wall, input_ros_ns=ros_ns, result=result,
                    token=getattr(frame, 'localization_token', None) if frame is not None else None)
            return result

        def on_localization(self, message):
            if self.diagnostics:
                self.diagnostic('status_callback')
                self.diagnostics.status_seen = True
            before = self.localization.last_status
            try:
                if self.diagnostics:
                    row = json.loads(message.data)
                    wall, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
                    self.localization.update(row, wall, ros_ns)
                    self.diagnostic('status_update', input_wall=wall, input_ros_ns=ros_ns,
                        input_status=status_snapshot(row))
                else:
                    self.localization.update(json.loads(message.data), time.monotonic(),
                                             self.get_clock().now().nanoseconds)
            except (ValueError, TypeError):
                self.localization.invalidate()
            status = self.localization.last_status
            # update() stores a new row object only when it accepts the status (identity, clock,
            # approvals); a rejected one leaves last_status as it was
            self.localization_view = (None if status is None or status is before else
                                      {"state": status["state"], "reason": status["reason"],
                                       "kept": self.localization.row is status, "row": status})
            if not self.localization_ok() or (self.nav and not self.nav["cancelled"]
                                             and not self.navigation_authorized(self.nav)):
                self.pending = None
                if self.active() or self.nav:
                    self.fault("localization_invalid")
                else:
                    self.stop()
            else:
                reason = self.correction_fault(self.nav)
                if reason:
                    self.fault(reason)
            # A frame waiting for its approval is retried as soon as a status arrives: it is
            # either approved now or the lease moved past it (skipped, and the frame queued
            # behind it is tried at once). Waiting for the next image instead lets the queue drop
            # exactly the frames that do carry an approval (camera 30 fps vs odometry 10 Hz).
            entry = self.acquisitions.current
            if entry is not None and entry is self.localization_wait_entry and not self.closed:
                self.prepare_images("localization")
            self.diagnostic_ack()

        def on_info(self, message):
            self.camera_info = message
            if self.diagnostics:
                self.diagnostic('camera_info_callback', frame=message.header.frame_id)

        def on_map(self, message):
            try:
                if message.header.frame_id != args.world_frame:
                    raise ValueError("map frame mismatch")
                origin = message.info.origin
                q = origin.orientation
                # Occupancy is planar; reject tilt rather than flattening it silently.
                if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or abs(q.z*q.z+q.w*q.w-1) > 1e-5:
                    raise ValueError("nonplanar/invalid map origin")
                self.map = FreeMap(np.asarray(message.data).reshape(message.info.height, message.info.width),
                                   message.info.resolution, origin.position.x, origin.position.y,
                                   2*math.atan2(q.z, q.w))
                if self.diagnostics:
                    self.diagnostics.map_seen = True
                    self.diagnostic('map_accepted')
            except Exception as exc:
                self.map = None
                self.fault("map_invalid: " + str(exc))
            self.diagnostic_ack()

        def on_images(self, rgb, depth):
            if self.diagnostics:
                self.diagnostic('images_callback', stamp_ns=rgb.header.stamp.sec*10**9+rgb.header.stamp.nanosec,
                                camera_info_present=self.camera_info is not None)
            if self.camera_info is None:
                return
            stamp = rgb.header.stamp.sec + rgb.header.stamp.nanosec/1e9
            if stamp <= self.last_submitted:
                self.dropped += 1
                return
            rx_raw = self.kv_now_ns()
            if self.geometry_source == "kv":
                # Nano arrival on CLOCK_MONOTONIC_RAW: start of the frame-age chain (header.stamp
                # is ROS time, never subtracted from it). Bounded: only the newest few are kept.
                # integer ns key (the float stamp here and Time.nanoseconds/1e9 in prepare_images
                # can differ in the last ulp for realtime stamps -> the pop would miss silently)
                self.kv_arrivals[rgb.header.stamp.sec * 10**9 + rgb.header.stamp.nanosec] = rx_raw
                while len(self.kv_arrivals) > 8:
                    self.kv_arrivals.pop(next(iter(self.kv_arrivals)))
            # the RAW arrival rides with the pair (never evicted while the pair waits for clock/TF)
            self.dropped += self.acquisitions.push(stamp, (rgb, depth, self.camera_info, rx_raw), time.monotonic())
            if self.diagnostics:
                self.diagnostic('queue_push', dropped_frames=self.dropped)
            self.prepare_images("image")

        def on_tf_arrival(self):
            """A /tf message is now in the tf2 buffer (hook_tf_arrival; executor thread). If the
            waiting frame's last try failed on a TF lookup, try it again now rather than at the next
            image pair / 100 ms poll. The same prepare_images: bounded-wait expiry first, exact-stamp
            lookups (tf2 interpolates between two real samples only, never extrapolates), age origin
            = the pair's arrival, unchanged; a frame waiting for its ROS clock is not retried here."""
            entry = self.acquisitions.current
            if entry is not None and entry is self.tf_wait_entry and not self.closed:
                self.prepare_images("tf")

        def expired_only_for_localization(self, entry):
            """True when the expired queue entry `entry` waited only for a localization approval that
            no status covered (2026-10-15 第 5 棒; controller ruling on owner's rule "skip frames
            without localization, no fault"). All of:
              1. estimated poses (a lease exists) and no try of this frame waited for its ROS clock
                 or a TF lookup;
              2. the last delivered status was accepted by the lease and its (state, reason) is in
                 LOCALIZATION_SKIP_REASONS (第 5 棒续: by reason, no longer by state): waiting
                 (`valid`, kept by the lease right after it), not ready (initializing/confirming,
                 not a clock rollback / replay discontinuity), or no correction
                 (stale:correction_too_old, lost:correction_jump, lost:incomplete_output,
                 stale:localization_stream_timeout). Odometry, clock and every other
                 reason, and a rejected status, do not qualify. That row carries no approval for
                 this stamp. Lease timing after that
                 status (heartbeat age) is not read here: a runner stall ages it, and a silent gate
                 is the lease backstop's job (localization_invalid_or_timeout in poll());
              3. the clock and the static extrinsics this frame needs are fine now: nonzero stamp
                 not ahead of ROS now, base<-camera and tracked<-camera look up at the stamp.
            The runner holds no odom<-base_link (on_static_tf drops odom edges, no dynamic /tf
            in estimated mode): odometry health reaches it only through the gate's status (2.).
            Reads only; consumes nothing."""
            view = self.localization_view
            if self.localization is None or entry.clock_tf_waited or localization_skip_class(view) is None:
                return False
            try:
                stamp = Time.from_msg(entry.payload[0].header.stamp)
                if stamp.nanoseconds <= 0 or stamp.nanoseconds / 1e9 > self.now():
                    return False
                if any(item["stamp_ns"] == stamp.nanoseconds for item in view["row"]["approved"]):
                    return False
                self.buffer.lookup_transform(args.base_frame, args.camera_frame, stamp)
                self.buffer.lookup_transform(args.localization_tracked_frame, args.camera_frame, stamp)
            except TransformException:
                return False
            return True

        def prepare_images(self, trigger="poll"):
            entry = self.acquisitions.current
            if entry is None:
                return
            if self.diagnostics:
                wall = time.monotonic()
                expired = self.acquisitions.expired(wall)
                self.diagnostic_attempt += 1
                self.diagnostic('prepare_expiry', input_wall=wall, expired=expired)
            else:
                expired = self.acquisitions.expired(time.monotonic())
            if expired:
                self.dropped += 1
                # A frame that only waited for its localization approval is not a clock/TF failure:
                # no approval came for it. Dropped without a fault; the lease (<= 1 s) and the 2 s
                # processed-stream timeouts in poll() remain the backstop (LocalizationSkipped).
                no_localization = entry is self.localization_wait_entry
                self.localization_wait_entry = None
                # 第 5 棒 (owner: frames without localization are skipped, not faulted): a frame that
                # expired untried behind a stalled one, only for want of its approval -> skipped too
                before_localization = not no_localization and self.expired_only_for_localization(entry)
                if self.sdiag is not None:
                    self.sdiag.acquisition_dropped(stamp_of(entry.payload[0]), entry.payload[3],
                                                    "no_localization_for_frame" if no_localization else
                                                    "expired_before_localization" if before_localization
                                                    else "wait_expired", trigger=trigger)
                if self.diagnostics and before_localization:
                    self.diagnostic('prepare_skip', branch='expired_before_localization')
                self.acquisitions.pop()
                if no_localization or before_localization:
                    self.localization_skipped += 1
                elif (self.active() or self.nav) and self.input_drop:
                    # 10-26 owner：重成功率，放宽自 fault: the frame is dropped, the stream watchdog is the backstop
                    self.input_hiccups += 1
                    self.record("acquisition_dropped", reason="acquisition_clock_or_tf_wait_expired",
                                total=self.input_hiccups)
                elif self.active() or self.nav:
                    self.tf_expiry_run += 1
                    if self.tf_expiry_run >= TF_EXPIRY_FAULT_RUN:
                        self.fault("acquisition_clock_or_tf_wait_expired")
                    else:
                        self.record('acquisition_dropped', reason='clock_or_tf_wait_expired',
                                    consecutive=self.tf_expiry_run, fault_at=TF_EXPIRY_FAULT_RUN)
                        self.get_logger().warning("Dropped acquisition after bounded wait for clock/TF "
                                                  "during a task (%d in a row)" % self.tf_expiry_run)
                else:
                    self.get_logger().warning("Dropped acquisition after bounded wait for clock/TF")
                return
            rgb, depth, info, rx_raw = entry.payload
            consumed = True
            skipped = False
            branch = 'validation'
            try:
                stamp = Time.from_msg(rgb.header.stamp)
                timestamp = stamp.nanoseconds / 1e9
                if stamp.nanoseconds <= 0:
                    # tf2 treats Time(0) as "latest", which is not acquisition-time TF.
                    raise ValueError("nonzero acquisition time required for exact TF lookup")
                if timestamp <= self.last_submitted:
                    self.dropped += 1
                    if self.sdiag is not None:
                        self.sdiag.acquisition_dropped(stamp.nanoseconds, rx_raw, "not_newer_than_submitted")
                    return
                if (rgb.header.frame_id != args.camera_frame or depth.header.frame_id != args.camera_frame
                        or info.header.frame_id != args.camera_frame):
                    raise ValueError("RGB, aligned depth and intrinsics must use the declared optical frame")
                if (rgb.width, rgb.height) != (info.width, info.height) or any(abs(v) > 1e-8 for v in info.d):
                    raise ValueError("rectified RGB with matching CameraInfo required")
                branch = 'clock'
                now = self.now()
                if self.diagnostics:
                    self.diagnostic('prepare_clock', stamp_ns=stamp.nanoseconds, input_ros_seconds=now)
                if timestamp > now:
                    raise AcquisitionPending("waiting for this acquisition's ROS clock")
                token = None
                branch = 'base_camera_static_tf'
                base_camera = self.buffer.lookup_transform(args.base_frame, args.camera_frame, stamp)
                if self.localization:
                    branch = 'localization_token'
                    if self.diagnostics:
                        wall, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
                        token = self.localization.token(stamp.nanoseconds, wall, ros_ns)
                        self.diagnostic('token_result', stamp_ns=stamp.nanoseconds, input_wall=wall,
                                        input_ros_ns=ros_ns, result=token)
                    else:
                        token = self.localization.token(stamp.nanoseconds, time.monotonic(),
                                                        self.get_clock().now().nanoseconds)
                    if token is None and self.localization.superseded(stamp.nanoseconds, time.monotonic(),
                                                                      self.get_clock().now().nanoseconds):
                        raise LocalizationSkipped("the localization lease moved past this acquisition")
                    if token is None:
                        raise AcquisitionPending("waiting for this acquisition's localization approval")
                    branch = 'tracked_camera_static_tf'
                    tracked_camera = self.buffer.lookup_transform(args.localization_tracked_frame, args.camera_frame, stamp)
                    branch = 'matrix'
                    if self.diagnostics:
                        wall, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
                        matrix = self.localization.matrix(stamp.nanoseconds, wall, ros_ns)
                        self.diagnostic('matrix_result', stamp_ns=stamp.nanoseconds, input_wall=wall, input_ros_ns=ros_ns)
                        world_camera_matrix = matrix @ transform_matrix(tracked_camera.transform)
                    else:
                        world_camera_matrix = self.localization.matrix(stamp.nanoseconds, time.monotonic(),
                            self.get_clock().now().nanoseconds) @ transform_matrix(tracked_camera.transform)
                    world_base_matrix = world_camera_matrix @ np.linalg.inv(transform_matrix(base_camera.transform))
                else:
                    tag0 = self.map_odom_tag() if self.live_align else None
                    world_camera = self.buffer.lookup_transform(args.world_frame, args.camera_frame, stamp)
                    world_base = self.buffer.lookup_transform(args.world_frame, args.base_frame, stamp)
                    world_camera_matrix = transform_matrix(world_camera.transform)
                    world_base_matrix = transform_matrix(world_base.transform)
                    if self.live_align:      # 10-27: the map -> odom these lookups used (same before and after)
                        tag1 = self.map_odom_tag()
                        live_tag = (tag1 if tag0 is not None and tag1 is not None
                                    and np.allclose(tag0, tag1, atol=1e-9) else None)
                rgb_array, depth_array = image_array(rgb), image_array(depth)
                if rgb.encoding not in ("rgb8", "bgr8") or depth.encoding not in ("32FC1", "16UC1"):
                    raise ValueError("RGB/depth encoding roles do not match")
                depth_array = depth_array.astype(np.float32)
                if depth.encoding == "16UC1":
                    depth_array *= args.uint16_depth_scale
                depth_array[depth_array <= 0] = np.nan
                calibration = hashlib.sha256(json.dumps(
                    {"K": list(info.k), "frame": args.camera_frame,
                     "base_camera": transform_matrix(base_camera.transform).tolist()},
                    sort_keys=True).encode()).hexdigest()
                frame = Frame("rgb-{}".format(stamp.nanoseconds), timestamp, rgb_array, depth_array,
                              np.asarray(info.k).reshape(3, 3), world_camera_matrix,
                              True, args.pose_source, "ros-camera-info-" + calibration,
                              Time.from_msg(depth.header.stamp).nanoseconds/1e9)
                frame.localization_token = token
                frame.pair_tolerance_s = camera_pairing(getattr(args, "camera_fps", 30.0))[0]
                if self.live_align and self.localization is None:
                    frame.map_odom_tag, frame.world_base_matrix = live_tag, world_base_matrix
                # the memory's below-ground gate measures object height from the base's ground
                # (base_link sits on the floor), not from world z = 0 (10-16, MEMORY_QUALITY_20261016)
                frame.ground_z_m = float(world_base_matrix[2, 3])
                frame.geometry_origin_ns = rx_raw      # geometry age gate origin (cpu/kv)
                if self.geometry_source == "kv_topic":
                    frame.kv_depth_stamp_ns = Time.from_msg(depth.header.stamp).nanoseconds
                if self.geometry_source == "kv":
                    frame.kv_rx_ns = self.kv_arrivals.pop(stamp.nanoseconds, None)
                    if frame.kv_rx_ns is None:      # evicted while the pair waited (> 8 newer arrivals)
                        frame.kv_rx_ns = rx_raw
                    # the camera's own codes go to the KV260 (no requantization); the KV260's
                    # crc of its converted depth must still equal this frame's float32 depth
                    frame.kv_codes = (np.array(image_array(depth), dtype=np.uint16, copy=True)
                                      if depth.encoding == "16UC1" and self.kv_codes_ok else None)
                frame.validate()
                now = self.now()
                if (timestamp <= now and now-timestamp > 1 and self.localization is not None
                        and localization_skip_class(self.localization_view) is not None):
                    # 第 5 棒续: stale only because it waited inside the runner while the status says
                    # waiting / not ready / no correction -> skipped, not sensor_or_pose_invalid
                    raise StaleWithoutLocalization("approved acquisition went stale in the runner")
                if timestamp > now or now-timestamp > 1:
                    raise ValueError("RGB acquisition time is stale or in the future")
                self.base_camera = transform_matrix(base_camera.transform)
                self.robot_xy = tuple(world_base_matrix[:2, 3])
                self.last_submitted = timestamp
                if self.sdiag is not None:
                    self.sdiag.maybe_inject("acquire")
                    self.sdiag.acquisition_done(frame, stamp.nanoseconds,
                                                Time.from_msg(depth.header.stamp).nanoseconds, rx_raw,
                                                trigger=trigger)
                if self.pending is not None:
                    self.dropped += 1
                    if self.sdiag is not None:
                        self.sdiag.superseded(self.pending)
                self.pending = frame
                self.tf_expiry_run = 0
                self.submit()
            except (TransformException, AcquisitionPending) as exc:
                if self.diagnostics:
                    self.diagnostic('prepare_wait', branch=branch, exception=type(exc).__name__, detail=str(exc)[:300])
                if self.sdiag is not None:
                    self.sdiag.acquisition_wait(stamp_of(rgb), rx_raw, branch, type(exc).__name__, self.sdiag_tf_latest(),
                                                trigger=trigger)
                # a new /tf sample may complete this lookup: on_tf_arrival retries this entry (v6)
                self.tf_wait_entry = entry if isinstance(exc, TransformException) else None
                self.localization_wait_entry = (entry if isinstance(exc, AcquisitionPending)
                                                and branch == 'localization_token' else None)
                if branch != 'localization_token':
                    entry.clock_tf_waited = True
                # Preserve the exact timestamp, retry from the steady timer, never use latest TF.
                consumed = False
            except LocalizationSkipped:
                # Not a fault and no events row: the frame is dropped like a superseded one.
                skipped = True
                self.dropped += 1
                self.localization_skipped += 1
                self.localization_wait_entry = None
                if self.diagnostics:
                    self.diagnostic('prepare_skip', branch=branch)
                if self.sdiag is not None:
                    self.sdiag.acquisition_dropped(stamp_of(rgb), rx_raw, "no_localization_for_frame", trigger=trigger)
            except StaleWithoutLocalization:
                # 第 5 棒续: same handling as a skip; its own stall-diag label (not C1/C2 labels)
                skipped = True
                self.dropped += 1
                self.localization_skipped += 1
                self.localization_wait_entry = None
                if self.diagnostics:
                    self.diagnostic('prepare_skip', branch='stale_after_approval')
                if self.sdiag is not None:
                    self.sdiag.acquisition_dropped(stamp_of(rgb), rx_raw, "stale_after_approval", trigger=trigger)
            except Exception as exc:
                if self.diagnostics:
                    self.diagnostic('prepare_error', branch=branch, exception=type(exc).__name__, detail=str(exc)[:300])
                if self.sdiag is not None:
                    self.sdiag.acquisition_dropped(stamp_of(rgb), rx_raw, "invalid:" + type(exc).__name__)
                if (self.active() or self.nav) and self.input_drop:
                    # 10-26 owner：重成功率，放宽自 fault: the frame is dropped, the stream watchdog is the backstop
                    self.input_hiccups += 1
                    self.record("acquisition_dropped", reason="sensor_or_pose_invalid: " + str(exc)[:200],
                                total=self.input_hiccups)
                elif self.active() or self.nav:
                    self.fault("sensor_or_pose_invalid: " + str(exc))
                else:
                    self.get_logger().warning("Waiting for valid synchronized input: " + str(exc))
            finally:
                if consumed:
                    self.acquisitions.pop()
                if self.diagnostics:
                    self.diagnostic('prepare_end', consumed=consumed, branch=branch)
            if skipped:
                # the newer frame queued behind a skipped one may be approved already; at most one
                # more level (the queue holds two entries)
                self.prepare_images(trigger)

        def submit(self):
            if self.diagnostics:
                self.diagnostic('submit_enter', pending=self.pending is not None, inference=self.inference is not None)
            if self.inference is None and self.pending is not None:
                frame, self.pending = self.pending, None
                if not self.localization_ok(frame):
                    self.dropped += 1
                    return
                if self.sdiag is not None:
                    self.sdiag.submitted(frame)
                    self.inference = self.pool.submit(self.sdiag.detect_wrapper(self.client.detect), frame)
                else:
                    self.inference = self.pool.submit(self.client.detect, frame)
                if self.diagnostics:
                    self.diagnostic('submit_started', frame_id=frame.frame_id)
                self.inference.add_done_callback(lambda future: enqueue_inference_result(self, frame, future))

        def planner(self, candidate):
            free_map, map_source = self.planning_map()
            if free_map is None or self.base_camera is None or self.frame is None:
                return []
            view = self.observation_views.get(candidate['id'])
            rejected = {}
            if map_source != "map":
                rejected["map_source"] = map_source   # 10-29: recorded with an empty plan only (as rejected)
            poses = observation_poses(candidate, free_map, self.frame.K,
                                     (self.frame.rgb.shape[1], self.frame.rgb.shape[0]),
                                     self.base_camera, self.robot_xy,
                                     preferred_poses=(view['pose'],) if view else (), rejected=rejected)
            # rejected only on an empty plan (why it is empty, SELECT_RESULT_DIG_20261016.md); a
            # non-empty plan's row stays byte-identical to the streams the regression tests pin.
            extra = {} if poses else {"rejected": rejected}
            self.record('observation_plan', target_id=candidate['id'],
                        remembered_view_frame=view['frame_id'] if view else None,
                        poses=[p.__dict__ for p in poses], **extra)
            return poses

        def on_teleop_active(self, message):
            self.teleop_sample = (bool(message.data), time.monotonic())

        def teleop_active(self):
            return teleop_in_hand(self.teleop_sample, time.monotonic())

        def ready(self):
            return (self.frame is not None and self.map is not None and self.base_camera is not None
                    and self.inference_healthy and self.frame.pose_valid and self.localization_ok(self.frame)
                    and 0 <= self.now()-self.frame.timestamp <= self.input_limits[1]
                    and time.monotonic()-self.last_processed_wall <= self.input_limits[0])   # 10-26: was 1 / 2

        def map_switching(self):
            if getattr(self, "map_reset_job", None) is not None:
                return MAP_SWITCH_REFUSAL            # 10-26: the map is being reset (host/map_reset.py)
            row, at = self.mapping_status or (None, None)
            # 10-27: a live alignment (while mapping) moves no map under a navigation -- not a map switch
            pending = self.memory_align_pending
            return map_switch_refusal(row, None if at is None else time.monotonic() - at,
                                      pending is not None and not pending.get("live"))

        def query(self, goal):
            if self.teleop_active():
                self.record("query_rejected", reason=TELEOP_REFUSAL)
                return
            if self.map_switching():
                self.record("query_rejected", reason=MAP_SWITCH_REFUSAL, goal=goal,
                            mapping_state=(self.mapping_status or ({},))[0].get("state"))
                return
            if self.active() and self.task.state == "ambiguous" and not self.patrol:
                # owner 10-29: a new find while the old one waits for a selection replaces it (no cancel first);
                # its goal is retired as recover() does, a late result of it is ignored
                previous = {"goal": getattr(self.task, "goal", None), "reason": getattr(self.task, "reason", None)}
                self.execute(self.task.cancel(self.now()))
                self.stop()
                context = self.nav
                if context is not None:
                    self.nav = None
                    self.record("navigation_retired", command_id=context["id"], cancel_requested=True,
                                for_replace=True)
                self.publish_task()
                self.record("query_replaced", goal=goal, previous=previous)
            if self.active() or self.nav or self.patrol:
                self.record("query_rejected", reason="cancel current operation and wait for navigation to stop")
                return
            if not self.ready():
                self.record("query_rejected", reason="recent processed input, TF and map required")
                return
            try:
                search = self.search_goals("search")
                # 10-26 probabilistic search: the belief over the mapped, seen area (the MVP list = its fallback)
                base = self.frame.T_world_camera @ np.linalg.inv(self.base_camera)
                nodes, _mapped, nodes_source = self.search_trajectory()
                free_map, map_source = self.planning_map()
                self.search_planner, planner_info = build_search_planner(
                    getattr(args, "search_mode", "mvp"), free_map, nodes,
                    getattr(args, "search_config_obj", None), (float(base[0, 3]), float(base[1, 3]),
                                                               math.atan2(base[1, 0], base[0, 0])),
                    self.search_planner_cache,
                    camera=(self.frame.K, (self.frame.rgb.shape[1], self.frame.rgb.shape[0]), self.base_camera))
                self.search_goals_recorded = 0
                if nodes_source == "live_track" or map_source != "map":   # 10-29 find while mapping
                    planner_info = dict(planner_info, nodes_source=nodes_source, map_source=map_source)
                self.record("search_planner", **planner_info)
                # navigation_available: a known target on a car without a NavigateToPose server ends
                # needs_manual_approach before any planning (SELECT_SEMANTICS_20261016.md section 1)
                self.task = TaskController(self.memory, self.planner, search, config=self.task_config,
                                           navigation_available=self.actions.server_is_ready,
                                           search_planner=self.search_planner)
                self.execute(self.task.start(goal, self.now(), self.frame))
                self.publish_task()
            except Exception as exc:
                self.fault("query_failed: " + str(exc))

        def select(self, message):
            if not self.active():
                return
            if self.teleop_active():
                self.record("selection_rejected", reason=TELEOP_REFUSAL)
                return
            if self.map_switching():
                self.record("selection_rejected", reason=MAP_SWITCH_REFUSAL)
                return
            try:
                self.execute(self.task.select_candidate(message.data, self.now()))
                self.publish_task()
            except (ValueError, RuntimeError) as exc:
                self.record("selection_rejected", reason=str(exc))

        def cancel(self, _message):
            if self.active():
                self.execute(self.task.cancel(self.now()))
            self.stop()
            self.publish_task()

        def begin_patrol(self, _message):
            if self.teleop_active():
                self.record("patrol_rejected", reason=TELEOP_REFUSAL)
                return
            if self.map_reset_job is not None:
                self.record("patrol_rejected", reason=MAP_SWITCH_REFUSAL)
                return
            if self.active() or self.nav or not self.ready():
                self.record("patrol_rejected", reason="busy or input not ready",
                            active_task=self.active(), navigation_busy=self.nav is not None,
                            map_ready=self.map is not None, input_ready=self.ready())
                return
            self.patrol = self.search_goals("patrol")
            self.next_patrol()

        def goto(self, message):
            """/fpga/goto: one relative goal through navigate() (test entry, module note above)."""
            try:
                forward, turn = goto_request(message.data)
            except ValueError as exc:
                self.record("goto_rejected", reason="bad_args: " + str(exc))
                return
            if self.teleop_active():
                self.record("goto_rejected", reason=TELEOP_REFUSAL)
                return
            if self.map_reset_job is not None:
                self.record("goto_rejected", reason=MAP_SWITCH_REFUSAL)
                return
            if self.active() or self.nav or self.patrol:
                self.record("goto_rejected", reason="cancel current operation and wait for navigation to stop")
                return
            if not self.ready():
                self.record("goto_rejected", reason="recent processed input, TF and map required")
                return
            pose = goto_pose(self.frame.T_world_camera @ np.linalg.inv(self.base_camera), forward, turn)
            if not self.map.free(pose.x, pose.y):
                self.record("goto_rejected", reason="goal_not_free", pose=pose.__dict__)
                return
            self.goto_count += 1
            command_id = "goto-%d" % self.goto_count
            self.record("goto_accepted", command_id=command_id, forward_m=forward, turn_rad=turn, pose=pose.__dict__)
            self.navigate(pose, command_id, goto=True)

        def next_patrol(self):
            if self.patrol:
                self.navigate(self.patrol.pop(0), None)
            else:
                self.record("patrol_complete")

        def execute(self, commands):
            for command in commands:
                self.record("command", command={**command.__dict__, "pose": command.pose.__dict__ if command.pose else None})
                if command.kind == "stop":
                    self.stop()
                elif command.kind == "navigate":
                    self.navigate(command.pose, command.command_id)
                elif command.kind == "recover":
                    self.recover(command)
                elif command.kind == "retarget":
                    self.retarget(command)
                elif command.kind == "approach":
                    self.request_approach(command)
            if commands:
                self.publish_task()

        def request_approach(self, command):
            """10-26 find-near: queue the final approach; it starts once no goal is live (the en-route stop has
            cancelled the task goal and its result has to come back first)."""
            self.approach_pending = {"command": command, "task": self.task, "since": time.monotonic()}
            self.try_approach()

        def try_approach(self):
            """Start (or give up) the pending approach; True when anything changed (the caller publishes the task
            row -- execute() does after its commands)."""
            pending = self.approach_pending
            if pending is None:
                return False
            command = pending["command"]
            if self.nav is not None:
                if time.monotonic() - pending["since"] > APPROACH_WAIT_S:
                    self.approach_pending = None
                    self.approach_done(command.command_id, "skipped", "previous_goal_not_stopped")
                    return True
                return False
            self.approach_pending = None
            task = self.task
            if task is not pending["task"] or task is None or task.state != "found" or task.found_row is None:
                self.record("approach_result", command_id=command.command_id, result="skipped",
                            reason="task_changed")
                return True
            if self.teleop_active():
                self.approach_done(command.command_id, "skipped", TELEOP_REFUSAL)
                return True
            if self.map_switching():
                self.approach_done(command.command_id, "skipped", MAP_SWITCH_REFUSAL)
                return True
            refusal = navigation_refusal(False, self.actions.server_is_ready(), self.ready())
            if refusal:
                self.approach_done(command.command_id, "skipped", refusal)
                return True
            try:
                world_base = self.frame.T_world_camera @ np.linalg.inv(self.base_camera)
                pose, info = approach_goal(world_base, task.found_row, self.base_camera, self.frame.K,
                                           (self.frame.rgb.shape[1], self.frame.rgb.shape[0]))
            except (KeyError, TypeError, ValueError, IndexError, np.linalg.LinAlgError) as exc:
                self.approach_done(command.command_id, "skipped", "bad_input: %s" % exc)
                return True
            if pose is None:
                reason = info.pop("reason")
                self.approach_done(command.command_id, "skipped", reason, **info)
                return True
            self.record("approach_started", command_id=command.command_id, target_id=task.target_id,
                        pose=pose.__dict__, **info)
            task.set_approach(dict(result="running", command_id=command.command_id, pose=pose.__dict__, **info))
            self.navigate(pose, command.command_id, approach=True)
            if not (self.nav is not None and self.nav.get("approach") and self.nav["id"] == command.command_id):
                self.approach_done(command.command_id, "refused", "navigate_refused")
            return True

        def approach_done(self, command_id, result, reason, **info):
            """Record the approach outcome on the (still found) task; the caller publishes the task row."""
            self.record("approach_result", command_id=command_id, result=result, reason=reason, **info)
            task = self.task
            if task is not None and task.state == "found":
                record = dict(task.approach or {})
                record.update(info, result=result, reason=reason, command_id=command_id)
                task.set_approach(record)

        def recover(self, command):
            """10-25 find-retry: a failed task goal -- stop, retire its navigation context (a late result of it is
            ignored: result()/accepted() check ``context is self.nav``), clear both costmaps; the task controller
            sends the next navigate after its pause."""
            context = self.nav
            if context is not None:         # a live goal (timeout): stop as stop() does; after a Nav2 failure
                self.motion.stop()          # navigation_done has already stopped motion and dropped the permit
                self.permit.publish(Bool(data=False))
                self.velocity.publish(Twist())
                context["cancelled"] = True
                self.cancel_navigation(context)
                self.nav = None
                self.record("navigation_retired", command_id=context["id"], cancel_requested=True, for_retry=True)
            attempt = self.task.attempts[-1] if self.task is not None and self.task.attempts else None
            self.record("navigation_attempt_failed", command_id=command.command_id, reason=command.reason,
                        attempt=attempt)
            self.clear_costmaps()

        def retarget(self, command):
            """10-29 find while mapping: the memory moved under the task (live alignment) and the goal being driven
            to was planned around the target's old place -- retire it as recover() does (stop, cancel; a late
            result is ignored), no costmap clearing; the task sends the new pose after its pause."""
            context = self.nav
            if context is not None:
                self.motion.stop()
                self.permit.publish(Bool(data=False))
                self.velocity.publish(Twist())
                context["cancelled"] = True
                self.cancel_navigation(context)
                self.nav = None
                self.record("navigation_retired", command_id=context["id"], cancel_requested=True, for_retarget=True)
            self.record("navigation_retargeted", command_id=command.command_id, reason=command.reason,
                        pose=command.pose.__dict__ if command.pose else None)

        def clear_costmaps(self):
            """Nav2 ClearEntireCostmap on the global and the local costmap (asynchronous; a service that is not
            available is recorded and skipped)."""
            try:
                from nav2_msgs.srv import ClearEntireCostmap
            except ImportError as exc:
                self.record("costmap_clear_skipped", reason="nav2_msgs.srv unavailable: %s" % exc)
                return
            for name in ("/global_costmap/clear_entirely_global_costmap",
                         "/local_costmap/clear_entirely_local_costmap"):
                try:
                    client = self.costmap_clients.get(name)
                    if client is None:
                        client = self.costmap_clients[name] = self.create_client(ClearEntireCostmap, name)
                    if not client.service_is_ready():
                        self.record("costmap_clear_skipped", service=name, reason="service_not_ready")
                        continue
                    future = client.call_async(ClearEntireCostmap.Request())
                    future.add_done_callback(
                        lambda f, _n=name: self.record("costmap_cleared", service=_n, ok=f.exception() is None))
                    self.record("costmap_clear_requested", service=name)
                except Exception as exc:
                    self.record("costmap_clear_skipped", service=name, reason=repr(exc))

        def navigate(self, pose, command_id, goto=False, approach=False):
            if self.teleop_active():
                # backstop for navigation the task starts on its own (e.g. the next search pose): no goal while the
                # remote control is in hand; the task ends failed/teleop_active (not a fault: input stays healthy)
                self.record("navigation_refused", reason=TELEOP_REFUSAL, command_id=command_id)
                if self.active():
                    try:
                        self.task.fail(TELEOP_REFUSAL, self.now())
                    except ValueError:
                        self.task = None
                self.stop()
                self.publish_task()
                return
            if (self.input_drop and not goto and self.active() and self.task.state == "navigating"
                    and command_id == getattr(self.task, "_active_id", None)
                    and navigation_refusal(self.nav is not None, self.actions.server_is_ready(), self.ready())
                    == "navigation_input_unhealthy"):
                # 10-26 owner：重成功率，放宽自 fault: a task goal sent while the input is not ready is not sent;
                # 10-29 owner: held (the same goal is resent after the retry pause, once frames are processed again),
                # no longer a failed attempt -- a long stall burnt the failure cap and ended the task
                self.record("navigation_refused", reason="navigation_input_unhealthy", command_id=command_id,
                            for_retry=True, held=True)
                self.motion.stop()
                self.permit.publish(Bool(data=False))
                self.execute(self.task.navigation_held(command_id, self.now()))
                self.publish_task()
                return
            reason = navigation_refusal(self.nav is not None, self.actions.server_is_ready(), self.ready())
            if reason:
                self.fault(reason)
                return
            message = NavigateToPose.Goal()
            message.pose.header.frame_id = pose.frame
            message.pose.header.stamp = self.get_clock().now().to_msg()
            message.pose.pose.position.x, message.pose.pose.position.y = float(pose.x), float(pose.y)
            message.pose.pose.orientation.z = math.sin(pose.yaw/2)
            message.pose.pose.orientation.w = math.cos(pose.yaw/2)
            reference = (lease_correction_reference(self.localization, time.monotonic(),
                                                    self.get_clock().now().nanoseconds)
                         if self.localization else None)
            if self.localization and reference is None:
                # D-035 item 2: an estimated goal needs the newest approval's map->odom.
                self.fault("map_correction_reference_unavailable")
                return
            context = {"id": command_id, "cancelled": False, "handle": None,
                       "task": self.task, "started_wall": time.monotonic(),
                       "authorization": ((self.localization.generation, self.localization.row['instance'],
                                          self.localization.row['epoch']) if self.localization else None),
                       "correction_reference": reference}
            if goto:
                context["goto"] = True
            if approach:
                context["approach"] = True
            self.nav = context
            self.record("navigation_started", command_id=command_id, pose=pose.__dict__)
            self.motion.arm()
            try:
                future = self.actions.send_goal_async(message)
                future.add_done_callback(lambda result: self.accepted(context, result))
            except Exception as exc:
                if context is self.nav:
                    self.fault("navigation_transport_failed: " + str(exc))

        def accepted(self, context, future):
            try:
                handle = future.result()
                context["handle"] = handle
                if context is not self.nav:
                    self.cancel_navigation(context)
                    return
                if not handle.accepted:
                    self.navigation_done(context, False, "navigation_goal_rejected")
                    return
                if not context["cancelled"] and not self.navigation_authorized(context):
                    self.fault("navigation_localization_continuity_lost")
                if context["cancelled"]:
                    self.cancel_navigation(context)
                # May complete immediately and legitimately start another request.
                # No context-independent mutations may follow this registration.
                handle.get_result_async().add_done_callback(lambda result: self.result(context, result))
            except Exception as exc:
                if context is self.nav:
                    self.fault("navigation_transport_failed: " + str(exc))

        def result(self, context, future):
            if context is not self.nav:
                return
            try:
                status = future.result().status
                if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                    raise ValueError("nonterminal navigation result")
            except Exception as exc:
                self.fault("navigation_result_transport_failed: " + str(exc))
                return
            self.navigation_done(context, status == GoalStatus.STATUS_SUCCEEDED,
                                 nav_failure_reason(status, GoalStatus))

        def navigation_done(self, context, succeeded, reason="navigation_failed"):
            if context is not self.nav:
                return
            if not context["cancelled"] and not self.navigation_authorized(context):
                self.fault("navigation_localization_continuity_lost")
            reason = self.correction_fault(context)
            if reason:
                # An arrival judged against a moved map is not an arrival; the fault cancels
                # the context, so the result below is recorded as retired.
                self.fault(reason)
            self.nav = None
            self.record("navigation_result", command_id=context["id"], succeeded=bool(succeeded),
                        cancelled=context["cancelled"],
                        **({} if succeeded or context["cancelled"] else {"reason": reason}))
            self.motion.stop()
            self.permit.publish(Bool(data=False))
            if context.get("approach"):         # 10-26 find-near: the task stays found whatever happens here
                if not context.get("approach_closed"):
                    self.approach_done(context["id"], "succeeded" if succeeded and not context["cancelled"]
                                       else ("cancelled" if context["cancelled"] else "failed"),
                                       None if succeeded and not context["cancelled"] else
                                       ("cancelled" if context["cancelled"] else reason))
                    self.publish_task()
                return
            if context["cancelled"]:
                self.record("navigation_retired", cancel_requested=True, result_succeeded=succeeded)
                return
            if context.get("goto"):             # /fpga/goto: one goal, no task and no patrol behind it
                self.record("goto_result", command_id=context["id"], succeeded=bool(succeeded))
                return
            if context["id"] is None:
                if succeeded:
                    self.next_patrol()
                else:
                    self.fault("patrol_navigation_failed")
            elif context["task"] is self.task:
                # 10-25: a failure goes to the controller's retry path (recover, pause, same goal / next pose)
                self.execute(self.task.navigation_result(context["id"], bool(succeeded), self.now(), reason))
                self.publish_task()

        def task_navigation(self, context):
            """A live goal of the active task (not /fpga/goto, not patrol, not cancelled)."""
            return (context is not None and not context["cancelled"] and not context.get("goto")
                    and context["id"] is not None and context.get("task") is self.task and self.active())

        def nav_wall_budget(self, context):
            """Wall-clock budget of a goal: the task attempt's min(--navigation-timeout-s, task time left), else
            --navigation-timeout-s."""
            if context is not None and context.get("approach"):     # 10-26 find-near: the final approach
                return APPROACH_TIMEOUT_S
            if self.task_navigation(context):
                return self.task.attempt_timeout_s
            return self.task_config.navigation_timeout_s

        def publish_belief(self, msg):
            text = belief_dumps(msg)
            self.search_belief_topic.publish(String(data=text))
            if msg["phase"] != "search" or msg["seq"] % 10 == 1:     # every non-search message, 1 in 10 search ones
                self.record("search_belief", phase=msg["phase"], seq=msg["seq"], task=msg["task"],
                            bytes=len(text), build_ms=msg["build_ms"], cells=msg["width"] * msg["height"],
                            result=msg["result"])

        def publish_task(self):
            if self.task:
                # target (where the remembered object is) only on needs_manual_approach
                extra = ({"target": self.task.target} if self.task.state == "needs_manual_approach"
                         and self.task.target is not None else {})
                # 10-25 find-retry: the query text and the navigation attempts (count, distinct goals, each one)
                extra.update(goal=self.task.goal, attempts=self.task.attempts_snapshot())
                # 10-26 find-near: the en-route verdict's distance and the final approach's outcome
                if self.task.en_route is not None:
                    extra["en_route"] = self.task.en_route
                if self.task.approach is not None:
                    extra["approach"] = self.task.approach
                # 10-28: where the final approach drives to (observation = this frame's reading away from memory)
                found = self.task.found_row
                if self.task.state == "found" and found is not None:
                    extra["approach_target"] = {"source": found.get("approach_source", "memory"),
                                                "position_m": found.get("approach_position_m") or found.get("position_m"),
                                                "memory_position_m": found.get("position_m")}
                # 10-26 search MVP: why a remembered target went searching (target_position_invalid /
                # not_seen_at_last_location); absent for a plain search or before it
                if self.task.search_trigger is not None:
                    extra["search_trigger"] = self.task.search_trigger
                # 10-29: processed frames missing past --stream-timeout-s (car: warn and wait); absent otherwise
                stall = self.task.stall_snapshot() if self.active() else None
                if stall is not None:
                    extra["stream_stall"] = stall
                # 10-26 probabilistic search: belief mass left and each chosen look (score, gain, path)
                planner = self.task.search_planner
                if planner is not None:
                    extra["search"] = planner.summary(compact=True)
                    if planner.began() and len(planner.goals) != self.search_goals_recorded:
                        self.search_goals_recorded = len(planner.goals)
                        self.record("search_goal_selected", **planner.goals[-1])
                # 10-26 heat map: preview at the start, the search at <= 1 Hz, done at the end (display only)
                self.belief_feed.tick(self.task, time.monotonic())
                self.record("task", state=self.task.state, reason=self.task.reason,
                            target_id=self.task.target_id,
                            candidates=[{k: r.get(k) for k in ("id", "label", "color", "position_m", "state", "number")}
                                        for r in self.task.candidates],
                            processed_frames=self.processed, dropped_frames=self.dropped, **extra)
            else:
                self.belief_feed.tick(None, time.monotonic())    # a task dropped (clock reset): done, once

        def poll(self):
            if self.closed:
                return
            if self.sdiag is not None:
                self.sdiag.poll_enter()
                self.sdiag_frame = None
                self.sdiag.graph_tick(getattr(self, "get_node_names_and_namespaces", list))
            try:
                if not self.localization_ok() or (self.nav and not self.nav["cancelled"]
                                                 and not self.navigation_authorized(self.nav)):
                    self.pending = None
                    if self.active() or self.nav:
                        self.fault("localization_invalid_or_timeout")
                    else:
                        self.stop()
                else:
                    reason = self.correction_fault(self.nav)
                    if reason:
                        self.fault(reason)
                self.prepare_images()
                if self.active() or self.nav:
                    # 10-26 owner：重成功率，放宽自硬编码 2 s: --stream-timeout-s (the car 5 s; host default 2 s)
                    gap = time.monotonic()-self.last_processed_wall
                    stalled = gap > self.task_config.stream_timeout_s
                    if stalled and not self.input_drop:
                        self.fault("processed_stream_wall_timeout")
                    elif stalled and self.stream_stall_wall is None:
                        # 10-29 owner: warn, keep the goal and the task; the motion permit (processed age) and the
                        # FPGA drive permit stop the car while no frame comes, both lift again with the next frame
                        self.stream_stall_wall = time.monotonic()
                        self.stream_stalls += 1
                        self.record("processed_stream_stalled", gap_s=round(gap, 3), n=self.stream_stalls,
                                    navigating=bool(self.nav and not self.nav["cancelled"]),
                                    message="画面处理中断，等待恢复")
                    if (self.input_drop or not stalled) and self.nav and \
                            time.monotonic()-self.nav["started_wall"] > self.nav_wall_budget(self.nav):
                        if self.task_navigation(self.nav):
                            # 10-25 find-retry: a task goal past its budget is a failed attempt (retry path)
                            context = self.nav
                            self.record("navigation_wall_timeout", command_id=context["id"],
                                        budget_s=self.nav_wall_budget(context))
                            self.execute(self.task.navigation_result(context["id"], False, self.now(),
                                                                     "navigation_wall_timeout"))
                            self.publish_task()
                        elif self.nav.get("approach"):
                            # 10-26 find-near: an approach past its budget stops; the task stays found
                            context = self.nav
                            if not context.get("approach_closed"):
                                context["approach_closed"] = True
                                self.approach_done(context["id"], "timeout", "approach_wall_timeout")
                                self.stop()
                                self.publish_task()
                        else:
                            self.fault("navigation_wall_timeout")
                if self.try_approach():
                    self.publish_task()
                if self.map_moved_pending:
                    self.map_moved_pending = False
                    if self.active():
                        try:
                            self.execute(self.task.map_moved(self.now()))
                        except Exception as exc:        # noqa: BLE001 - the plan stays as it was
                            self.record("map_moved_task_update_failed", reason=repr(exc))
                if self.active():
                    stall = self.task.stream_stall
                    self.execute(self.task.tick(self.now()))
                    if self.task is not None and self.task.stream_stall is not stall:
                        self.publish_task()     # 10-29: the console sees the stall warning without a new frame
                if getattr(self, "kv_hold", None) is not None:
                    frame, future, _ = self.kv_hold
                else:
                    try:
                        frame, future = self.results.get_nowait()
                    except queue.Empty:
                        return
                if self.geometry_source == "kv_topic" and not self.kv_topic_ready(frame, future):
                    return                  # held: bounded wait for the producer's body of this frame
                if self.sdiag is not None:
                    self.sdiag.frame_begin(frame, getattr(frame, "kv_waited_ms", None))
                    self.sdiag_frame = {"result": "exception", "gate": None, "origin": None, "basis": None}
                self.inference = None
                try:
                    detections = future.result()
                    if not self.localization_ok(frame):
                        self.dropped += 1
                        self.record("frame_rejected", frame_id=frame.frame_id,
                                    reason="localization_approval_revoked_before_memory_write")
                        if self.sdiag is not None:
                            self.sdiag_frame["result"] = "rejected_localization"
                        return
                    if self.now()-frame.timestamp > 1 or frame.timestamp > self.now():
                        self.dropped += 1
                        if self.sdiag is not None:
                            self.sdiag_frame["result"] = "dropped_older_than_1s_or_future"
                        return
                    if self.sdiag is not None:
                        self.sdiag.mark("result")
                        self.sdiag.maybe_inject("process")
                    if self.live_align:
                        self.align_frame(frame)     # 10-27: into the memory's current alignment first
                    observations = process_frame(frame, detections)
                    if self.sdiag is not None:
                        self.sdiag.mark("process")
                    if self.geometry_source == "kv":
                        kv_outcome, geometry = self.kv_geometry(frame)
                    elif self.geometry_source == "kv_topic":
                        kv_outcome, geometry = self.topic_geometry(frame)
                    else:
                        kv_outcome = None
                        geometry = ({} if self.region_source == "kv_only"
                                    else compute_geometry(frame, self.memory))
                    if self.sdiag is not None:
                        self.sdiag.maybe_inject("geometry")
                        self.sdiag.mark("geometry")
                    # 200 ms age gate at the point of use (after the geometry step, so a frame
                    # that was fresh on arrival but expired while being processed is caught)
                    geometry, gate = self.geometry_age_check(frame, kv_outcome, geometry)
                    raw_geometry = geometry             # 10-27: logged next to the filtered word (track_ev)
                    if self.see_through.active():   # 10-27: see-through filter, right before the memory write
                        geometry = self.see_through.apply(geometry, observations, self.memory,
                                                         now=frame.timestamp)
                        if self.see_through.last:
                            self.record("geometry_see_through", frame_id=frame.frame_id, changed=self.see_through.last,
                                        see_through_tracks=sorted(self.see_through.see_through))
                    if self.sdiag is not None:
                        self.sdiag.maybe_inject("gate")
                        self.sdiag.mark("gate")
                        self.sdiag_frame.update(result="processed", gate=gate)
                        self.sdiag.maybe_inject("memory")
                    mark_pose_trust(self.pose_trust, frame)
                    if self.map_reset_holds(frame):
                        pass                # 10-26 map reset: nothing seen in the old map frame enters the new memory
                    else:
                        # 10-27 identity: the task's target and offered candidates are never merged away
                        task = self.task
                        self.memory.keep_ids = set() if task is None else (
                            {task.target_id} | {c.get("id") for c in (getattr(task, "candidates", None) or ())
                                                if isinstance(c, dict)}) - {None}
                        mem_out = self.memory.update_memory(observations, geometry, frame)
                        # 10-27: identity-v2 merges were not in the log (a vanished track could not be told apart)
                        merged = mem_out.get("merged") if isinstance(mem_out, dict) else None
                        # 10-27 existence step 2: a promoted box taking over a position_invalid track
                        relocated = mem_out.get("relocated") if isinstance(mem_out, dict) else None
                        if merged or relocated:
                            self.record("memory_merged", frame_id=frame.frame_id, merged=merged or [],
                                        identity_stats=dict(getattr(self.memory, "identity_stats", {}) or {}),
                                        **({"relocated": relocated} if relocated else {}))
                    if self.sdiag is not None:
                        self.sdiag.mark("memory")
                    if self.geometry_source == "kv":
                        self.kv_after_memory(frame, kv_outcome, gate)
                    elif self.geometry_source == "kv_topic":
                        reason = gate["reason"] if gate["reason"] is not None else kv_outcome.reason
                        frame.kv_stats = dict(kv_outcome.info, result=("fused" if reason is None
                                                                       else "unavailable:%s" % reason),
                                              frame_age_ms=(None if kv_outcome.info.get("acquired_raw_ns") is None
                                                            else (self.kv_now_ns() - kv_outcome.info["acquired_raw_ns"]) / 1e6),
                                              geometry_age_ms=gate["age_ms"], age_gate=gate["verdict"])
                    # The first successful observation supplies a real, previously reached
                    # viewpoint. It is only a proposal: map/FOV/Nav2 and fresh confirmation
                    # still apply, and this memory is discarded with the runtime session.
                    world_base = frame.T_world_camera @ np.linalg.inv(self.base_camera)
                    observed_pose = PoseGoal(float(world_base[0, 3]), float(world_base[1, 3]),
                                             math.atan2(world_base[1, 0], world_base[0, 0]))
                    for track in self.memory.snapshot():
                        if track['last_seen_frame'] == frame.frame_id and track['state'] == 'current_confirmed':
                            self.observation_views.setdefault(track['id'], {'pose': observed_pose, 'frame_id': frame.frame_id,
                                                                            'stamp': frame.timestamp})
                    if (getattr(args, "search_waypoints_from_mapping", False) and frame.pose_valid
                            and getattr(frame, "pose_trusted", True) is not False):
                        # 10-29 find while mapping: the car's own trajectory = search nodes before an export
                        self.live_track.add(frame.timestamp, observed_pose.x, observed_pose.y, observed_pose.yaw)
                    self.frame = frame
                    if self.stream_stall_wall is not None:
                        self.record("processed_stream_resumed", n=self.stream_stalls,
                                    gap_s=round(time.monotonic()-self.last_processed_wall, 3),
                                    reported_for_s=round(time.monotonic()-self.stream_stall_wall, 3))
                        self.stream_stall_wall = None
                    self.last_processed_wall = time.monotonic()
                    self.inference_healthy = True
                    self.processed += 1
                    evidence = None
                    if args.frames_dir:
                        path = args.frames_dir / (frame.frame_id + ".npz")
                        with path.open("xb") as stream:
                            np.savez_compressed(stream, rgb=frame.rgb, depth=frame.depth,
                                                K=frame.K, T_world_camera=frame.T_world_camera,
                                                timestamp=frame.timestamp)
                        evidence = {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                    if self.sdiag is not None:
                        self.sdiag.mark("stats")
                    self.record("frame_processed", frame_id=frame.frame_id,
                                evidence=evidence,
                                input_ready=self.ready(), map_ready=self.map is not None,
                                navigation_available=self.actions.server_is_ready(),
                                acquisition_timestamp=frame.timestamp,
                                detections=[{"label": detection.label, "score": float(detection.score),
                                             "bbox": list(detection.bbox)} for detection in detections],
                                observation_count=len(observations),
                                geometry_version=self.geometry_version,   # 10-27 A/B: which geometry judged
                                track_ev=track_ev_row(self.memory, self.see_through, raw_geometry, geometry,
                                                      self.track_ev_last),
                                miss_turn=miss_turn_row(self.memory),   # 10-27 turn gate: [deg/s, skipped]
                                tracks=[{key: row.get(key) for key in
                                         ("id", "label", "color", "position_m", "state", "reason", "last_seen", "last_seen_frame",
                                          "number")}
                                        for row in self.memory.snapshot()],
                                dropped_frames=self.dropped,
                                **({"geometry": {k: frame.kv_stats.get(k) for k in
                                                 ("result", "fallback", "frame_age_ms", "chain_ms", "rtt_ms",
                                                  "roi_core", "boxes_sent", "overflow", "shadow_equal",
                                                  "kv_frame_id", "capture_nano_ns", "kv_times_ms",
                                                  "roi_times_ms", "geometry_age_ms", "age_gate")}}
                                   if self.geometry_source == "kv" else
                                   {"geometry": {k: frame.kv_stats.get(k) for k in
                                                 ("result", "fallback", "frame_age_ms", "match", "match_dt_ms",
                                                  "waited_ms", "producer_frame_id", "roi_core", "boxes",
                                                  "uncovered", "memory_revision_used", "shadow_equal",
                                                  "geometry_age_ms", "age_gate")}}
                                   if self.geometry_source == "kv_topic" else {}))
                    if self.sdiag is not None:
                        self.sdiag.maybe_inject("task")
                    if self.active() and self.search_planner is not None:
                        # 10-26 probabilistic search: what this frame saw lowers the belief (<= update_hz, pose
                        # trusted only); before the task sees the frame, so its next search goal knows it
                        try:
                            self.search_planner.observe(
                                frame.T_world_camera, frame.timestamp,
                                base_pose=(observed_pose.x, observed_pose.y, observed_pose.yaw),
                                trusted=bool(frame.pose_valid) and getattr(frame, "pose_trusted", True) is not False,
                                K=frame.K, image_size=(frame.rgb.shape[1], frame.rgb.shape[0]),
                                T_base_camera=self.base_camera)
                        except Exception as exc:      # noqa: BLE001 - the belief misses one frame, the task goes on
                            self.record("search_belief_update_failed", frame_id=frame.frame_id, reason=repr(exc))
                    if self.active():
                        # 10-26 find-near: the base position at this frame's acquisition (en-route verdict)
                        self.execute(self.task.observe(frame, self.now(), robot_xy=frame_base_xy(frame, self.base_camera)))
                    self.publish_task()
                    if self.sdiag is not None:
                        self.sdiag.mark("task")
                finally:
                    if self.sdiag is not None:
                        self.sdiag.skip()
                    self.submit()
                    if self.sdiag is not None:
                        self.sdiag.mark("submit")
            except Exception as exc:
                self.fault("processing_failed: " + str(exc))
            finally:
                context = self.nav
                if context and not context["cancelled"] and not self.navigation_authorized(context):
                    self.fault("navigation_localization_continuity_lost")
                handle = context.get("handle") if context else None
                allowed = self.motion.allowed(
                    navigating=bool(handle and handle.accepted),
                    cancelled=bool(context and context["cancelled"]),
                    processed_age=time.monotonic()-self.last_processed_wall,
                    acquisition_age=self.now()-self.frame.timestamp if self.frame else math.inf,
                    pose_valid=bool(self.frame and self.frame.pose_valid and self.localization_ok(self.frame)),
                    inference_healthy=self.inference_healthy)
                if self.sdiag is not None:
                    self.sdiag.skip()
                    self.sdiag.maybe_inject("permit")
                self.permit.publish(Bool(data=allowed))
                if self.sdiag is not None:
                    self.sdiag.mark("permit")
                self.publish_memory()
                if self.sdiag is not None and self.sdiag_frame is not None:
                    f = self.sdiag_frame
                    self.sdiag.frame_end(gate=f["gate"], origin_ns=f["origin"], basis=f["basis"],
                                         tracks=len(self.memory.tracks), result=f["result"])
                    self.sdiag_frame = None

        def publish_memory(self):
            """Throttled full /fpga/memory (D.1); a failure here never touches the permit path."""
            try:
                message = self.memory_feed.step(
                    self.memory, time.monotonic(), self.now(),
                    target_label_of(self.task, self.memory) if self.active() else None,
                    self.region.last_region if self.region else None)
                if message is not None:
                    text = json.dumps(message, ensure_ascii=False, allow_nan=False)
                    if self.sdiag is not None:
                        self.sdiag.mark("memory_feed")
                        self.sdiag.maybe_inject("memory_publish")
                    self.memory_topic.publish(String(data=text))
                    if self.sdiag is not None:
                        self.sdiag.mark("memory_publish")
            except Exception as exc:
                self.memory_feed.counts["publish_error:" + type(exc).__name__] += 1
            self.memory_snapshot()

        def memory_document(self, complete):
            return memory_store.dump(
                self.memory, complete=complete, runtime_boot_id=self.runtime_boot_id, world_frame=args.world_frame,
                pose_source=args.pose_source, sensor_source=args.sensor_source,
                map_identity_doc=self.memory_identity, pose_mode=getattr(args, "pose_mode", None),
                code_commit=os.environ.get("HEAD") or None, written_ros_s=self.now(),
                loaded_from=self.memory_loaded_from)

        def memory_snapshot(self):
            """10-18: memory_snapshot.json when the memory content changed, at most every --memory-snapshot-s
            (no fsync: a process crash keeps the last complete file; a snapshot never fails the frame path)."""
            policy = self.memory_snapshots
            if policy is None or self.memory_feed.key is None:
                return
            key, now = self.memory_feed.key[0], time.monotonic()
            if not policy.due(key, now):
                return
            policy.done(key, now)
            try:
                memory_store.save_atomic(self.memory_document(False),
                                         str(args.memory_out_dir / "memory_snapshot.json"), fsync=False)
            except Exception as exc:                # noqa: BLE001
                policy.errors += 1
                if policy.errors == 1:
                    self.record("memory_snapshot_failed", error=repr(exc))

        def save_memory_final(self):
            """10-18: memory_final.json at a normal stop (close(): SIGINT/SIGTERM and spin exit)."""
            if not getattr(args, "memory_out_dir", None):
                return
            path = str(args.memory_out_dir / "memory_final.json")
            try:
                memory_store.save_atomic(self.memory_document(True), path)
                self.record("memory_saved", path=path, file_sha256=memory_store.sha256_file(path),
                            tracks=len(self.memory.tracks), map_identity_kind=self.memory_identity["kind"],
                            snapshots=None if self.memory_snapshots is None else self.memory_snapshots.writes)
            except Exception as exc:                # noqa: BLE001
                self.record("memory_save_failed", path=path, error=repr(exc))
                print("memory save failed: %r" % (exc,), file=sys.stderr, flush=True)

        def kv_geometry(self, frame):
            """(outcome, evidence for update_memory). Never silently CPU: the fallback is the
            explicit --geometry-fallback switch and every use of it is an event."""
            outcome = self.kv.evaluate(frame, self.memory, codes=getattr(frame, "kv_codes", None),
                                       rx_ns=getattr(frame, "kv_rx_ns", None))
            state = (outcome.evidence is not None, outcome.reason)
            if state != self.kv_state:
                if outcome.evidence is None:
                    self.record("geometry_unavailable", frame_id=frame.frame_id, reason=outcome.reason,
                                fallback=args.geometry_fallback, kv=self.kv.describe())
                else:
                    self.record("geometry_available", frame_id=frame.frame_id, kv=self.kv.describe())
                self.kv_state = state
            if outcome.evidence is not None:
                if args.kv_shadow_check:
                    cpu = compute_geometry(frame, self.memory)
                    same = list(cpu.items()) == list(outcome.evidence.items())
                    self.kv_shadow["frames"] += 1
                    self.kv_shadow["equal"] += same
                    outcome.info["shadow_equal"] = same
                    if not same:
                        self.record("geometry_shadow_mismatch", frame_id=frame.frame_id,
                                    diff={k: [cpu.get(k), outcome.evidence.get(k)] for k in set(cpu) | set(outcome.evidence)
                                          if cpu.get(k) != outcome.evidence.get(k)})
                return outcome, outcome.evidence
            if args.geometry_fallback == "cpu":
                self.record("geometry_fallback_cpu", frame_id=frame.frame_id, reason=outcome.reason)
                outcome.info["fallback"] = "cpu"
                return outcome, compute_geometry(frame, self.memory)
            return outcome, {}

        def kv_topic_key(self, frame):
            stamp = getattr(frame, "kv_depth_stamp_ns", None)
            if stamp is None:               # frames that never passed prepare_images (replay fixtures)
                stamp = int(round((frame.depth_timestamp if frame.depth_timestamp is not None
                                   else frame.timestamp) * 1e9))
            return stamp

        def kv_topic_ready(self, frame, future):
            """True: process now (body found, or the bounded wait ran out). False: frame held."""
            now = self.kv_now_ns()
            first = self.kv_hold[2] if self.kv_hold is not None else now
            if (self.kv_topic.match(self.kv_topic_key(frame)) is None
                    and now - first < args.geometry_topic_wait_s * 1e9):
                self.kv_hold = (frame, future, first)
                return False
            self.kv_hold = None
            frame.kv_waited_ms = (now - first) / 1e6
            return True

        def topic_geometry(self, frame):
            """kv_topic twin of kv_geometry: (outcome, evidence for update_memory); the fallback is
            the explicit --geometry-fallback switch, every use recorded."""
            from host.kv_geometry import GeometryOutcome
            evidence, reason, info = self.kv_topic.lookup(self.kv_topic_key(frame), self.memory,
                                                          frame.T_world_camera, now_raw_ns=self.kv_now_ns())
            info["waited_ms"] = getattr(frame, "kv_waited_ms", None)
            seen = info.get("producer_geometry_version")
            if seen is not None and seen != self.geometry_version_seen:
                self.geometry_version_seen = seen
                if seen != self.geometry_version:
                    # 10-27: producer and runtime must run the same geometry (car_up sets both); warn, never stop
                    self.record("geometry_version_mismatch", frame_id=frame.frame_id, runtime=self.geometry_version,
                                producer=seen)
                    self.get_logger().warning("geometry version mismatch: runtime %s, producer %s (words used as "
                                              "the producer sent them)" % (self.geometry_version, seen))
            outcome = GeometryOutcome(evidence, reason, info)
            state = (evidence is not None, reason)
            if state != self.kv_state:
                if evidence is None:
                    self.record("geometry_unavailable", frame_id=frame.frame_id, reason=reason,
                                fallback=args.geometry_fallback, source="kv_topic")
                else:
                    self.record("geometry_available", frame_id=frame.frame_id, source="kv_topic")
                self.kv_state = state
            if evidence is not None:
                if args.kv_shadow_check:
                    cpu = compute_geometry(frame, self.memory)
                    same = list(cpu.items()) == list(evidence.items())
                    self.kv_shadow["frames"] += 1
                    self.kv_shadow["equal"] += same
                    info["shadow_equal"] = same
                    if not same:
                        self.record("geometry_shadow_mismatch", frame_id=frame.frame_id,
                                    diff={k: [cpu.get(k), evidence.get(k)] for k in set(cpu) | set(evidence)
                                          if cpu.get(k) != evidence.get(k)})
                return outcome, evidence
            if args.geometry_fallback == "cpu":
                self.record("geometry_fallback_cpu", frame_id=frame.frame_id, reason=reason)
                info["fallback"] = "cpu"
                return outcome, compute_geometry(frame, self.memory)
            return outcome, {}

        def geometry_age_check(self, frame, kv_outcome, geometry):
            """(geometry for update_memory, gate row). The 200 ms rule of the producer
            (host/geometry_age_gate.py), applied to whatever geometry is about to be used:
            older than 200 ms -> {} (geometry unavailable), event ``geometry_stale``; no origin
            stamp (age unknown) -> {} too, event ``geometry_age_unknown``."""
            if kv_outcome is None:                              # cpu source
                used = self.region_source != "kv_only"
                origin, clock_id, basis = getattr(frame, "geometry_origin_ns", None), None, "runner_arrival"
            else:
                fallback = kv_outcome.info.get("fallback") == "cpu"
                used = kv_outcome.reason is None or fallback
                if self.geometry_source == "kv_topic" and not fallback:
                    origin = kv_outcome.info.get("acquired_raw_ns")
                    clock_id, basis = kv_outcome.info.get("clock_id"), "producer_acquired"
                else:
                    origin = getattr(frame, "kv_rx_ns", None)
                    if origin is None:
                        origin = getattr(frame, "geometry_origin_ns", None)
                    clock_id, basis = None, "runner_arrival"
            if not used:
                return geometry, {"verdict": "not_used", "age_ms": None, "reason": None}
            if self.sdiag is not None and getattr(self, "sdiag_frame", None) is not None:
                self.sdiag_frame.update(origin=origin, basis=basis)
            age, reason = self.age_gate.check(origin, self.kv_now_ns(), origin_clock_id=clock_id)
            age_ms = None if age is None else age / 1e6
            if reason is None:
                return geometry, {"verdict": "pass", "age_ms": age_ms, "reason": None}
            self.record(reason, frame_id=frame.frame_id,
                        source=self.geometry_source, reason=reason, age_ms=age_ms,
                        limit_ms=self.age_gate.limit_ns / 1e6, age_basis=basis,
                        fallback=(kv_outcome.info.get("fallback") if kv_outcome is not None else None),
                        refused_total=self.age_gate.refused)
            return {}, {"verdict": "refused", "age_ms": age_ms, "reason": reason}

        def kv_after_memory(self, frame, outcome, gate=None):
            from host.kv_geometry import frame_stats_body
            body = frame_stats_body(outcome, memory_ns=self.kv_now_ns(),
                                    client_counts=self.kv.client.counts if self.kv.client else None)
            body.update(frame_id=frame.frame_id, fallback=outcome.info.get("fallback"),
                        shadow_equal=outcome.info.get("shadow_equal"),
                        shadow_totals=dict(self.kv_shadow) if args.kv_shadow_check else None)
            if gate is not None:
                body.update(geometry_age_ms=gate["age_ms"], age_gate=gate["verdict"])
                if gate["reason"] is not None:
                    body["result"] = "unavailable:%s" % gate["reason"]
            try:
                self.kv_stats_topic.publish(String(data=json.dumps(body, allow_nan=False)))
            except Exception as exc:
                self.kv.counts["stats_publish_error:" + type(exc).__name__] += 1
            frame.kv_stats = body

        def on_pose_trust_imu(self, message):
            feed_pose_trust(self.pose_trust, "imu", message)
            self.pose_trust_report()

        def on_pose_trust_odom(self, message):
            feed_pose_trust(self.pose_trust, "odom", message)
            self.pose_trust_report()

        def on_pose_trust_slip(self, message):
            fed = feed_pose_trust_slip(self.pose_trust, message.data)
            if fed is not None:
                self.record("pose_trust_slip", trusted=fed[1], at=fed[0])

        def pose_trust_report(self):
            for t, trusted, magnitude in self.pose_trust.drain_transitions():
                self.record("pose_trust", trusted=trusted, at=t,
                            discrepancy_deg=None if magnitude is None else math.degrees(magnitude))

        def on_region(self, message):
            """D.3: same executor thread as update_memory; a bad message is recorded, not fatal."""
            try:
                row = self.region.apply(self.memory, message.data, self.now())
            except Exception as exc:
                row = {"event": "region_rejected", "reason": "internal:" + type(exc).__name__,
                       "detail": str(exc)[:200]}
            kind = row.pop("event")
            self.record(kind, region_source=self.region_source, **row)

        def close(self):
            with self.result_lock:
                self.closed = True
            try:
                self.stop()
            finally:
                self.pool.shutdown(wait=True)
                gate = self.age_gate.summary() if getattr(self, "age_gate", None) is not None else None
                if getattr(self, "kv", None) is not None:
                    self.record("geometry_close", counts=dict(self.kv.counts), shadow=self.kv_shadow,
                                age_gate=gate)
                    self.kv.close()
                elif getattr(self, "kv_topic", None) is not None:
                    self.record("geometry_close", source="kv_topic", counts=dict(self.kv_topic.counts),
                                shadow=self.kv_shadow, held_at_close=self.kv_hold is not None, age_gate=gate)
                elif gate is not None:
                    self.record("geometry_close", source="cpu", age_gate=gate)
                self.save_memory_final()
                self.note_record_drops(final=True)
                if getattr(self, "input_log", None) is not None:
                    try:
                        il = self.input_log.close()
                        print("input log: %s" % json.dumps(il.get("_receiver"), sort_keys=True), file=sys.stderr,
                              flush=True)
                    except Exception as exc:        # noqa: BLE001 - measurement only
                        print("input log close failed: %r" % (exc,), file=sys.stderr, flush=True)
                    self.input_log = None
                if self.sdiag is not None:
                    sd = dict(self.sdiag.close() or {})
                    sd.pop("slow", None)
                    print("record writer: %s" % json.dumps(sd, sort_keys=True), file=sys.stderr, flush=True)
                if self.result_guard is not None:
                    self.destroy_guard_condition(self.result_guard)
                    self.result_guard = None
                if self.events:
                    # v7: drain (normal exit and SIGINT/SIGTERM both come here), then say how it went
                    summary = self.events.close()
                    summary.pop("slow", None)
                    print("record writer: %s" % json.dumps(summary, sort_keys=True), file=sys.stderr, flush=True)

    from rclpy.signals import SignalHandlerOptions
    spin_until_interrupt(rclpy, Runtime, ros_args, SignalHandlerOptions.NO)


if __name__ == "__main__":
    main()
