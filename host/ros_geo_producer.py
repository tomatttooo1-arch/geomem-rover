"""Car geometry producer node: /fpga/depth -> KV260 -> RollingFusion -> Nav2 bridge (2026-10-11).

The car's only KV260 frame client (docs/design/HETEROGENEOUS_PARTITION_20260920.md:66,78:
"Gemini 2 depth -> PL local geometry / obstacle update -> Nav2"). All decisions live in
host/geo_producer.py; this file moves messages, looks up TF and runs one worker thread.

  in   /fpga/depth (16UC1 camera codes; 32FC1 is accepted only when exactly representable),
       /fpga/camera_info, /fpga/memory (D.1, transient_local), TF odom<-base_link (at the
       depth stamp), base_link<-camera, map<-camera (ROI planning only; optional)
       (dynamic /tf read best effort by default, --tf-reliable for the tf2_ros default;
       /tf_static reliable + transient_local; host.geo_producer.tf_listener_qos)
  out  /fpga/rolling_map/delta, /fpga/rolling_map/snapshot (RollingProducerLink: the bridge
       host/ros_rolling_nav.py turns them into the Nav2 StaticLayer grid)
       /fpga/geometry_frame   std_msgs/String JSON, one per processed frame: D.2 region
                             verdict body + "producer" block (depth stamp, acquisition RAW
                             ns, frame id, geometry/fusion result); ros_runtime
                             --geometry-source kv_topic consumes it
       /fpga/kv_link/frame_stats  D.2 body per processed frame, plus "source" (--data-source),
                             "source_marker" (newest /fpga/sensor_source body or null) and
                             "source_mismatch" (marker names another source), "source_warning"
                             (replay_without_marker | marker_mismatch | null; never blocks; one
                             /fpga/status event source_warning / source_warning_cleared per change)
  in   /fpga/sensor_source    (optional, latched) the replay stand-in's marker
                             (host.geo_producer.SourceMarker)

Threads: the depth callback only stamps the RAW arrival (host.fpga_clock; the frame-age
origin, same definition as ros_runtime's kv path), applies the rate limit (--rate-hz) and
drops the frame into a one-slot mailbox (an unconsumed older frame is overwritten and
counted). The worker takes the newest, waits up to --tf-wait-s for its TF, runs
GeoProducer.process and publishes. 200 ms frame age (D-035) is enforced by M7Pipeline;
nothing here relaxes it.

Operating rule (v3, 10-11 night; host/README.md "车上链路运行规程"): while the car runs, start no new ROS
process or ROS CLI (ros2 topic/node ..., rviz2, a script calling rclpy.init): a DDS participant joining or
leaving can block this node's publish ~230-320 ms without releasing the GIL. --dds-publish-mode sets
RMW_FASTRTPS_PUBLICATION_MODE for this process only, before rclpy.init (an XML <publishMode> alone is
overridden by the rmw's SYNCHRONOUS default).

--data-source live|replay (required, no default): where the depth comes from. replay: every
KV260 request carries the replay mark (AR panel REPLAY instead of LIVE).

--record-write async|sync (v7, 2026-10-12): the --stats-out rows (per frame, events, diagnostics)
go through host/record_writer.RecordWriter -- async (default): a bounded queue written by one
background thread, so an SD-card writeback stall no longer stops the worker (board v6: 0.7 s);
a full queue drops the row, counted (summary "records", a record_drops event on /fpga/status ->
hub STATE geometry_nano.record_drops). sync: the write on the calling thread as before (control).
--gc freeze|default (v7): host/gc_policy.py, applied once the node is built; default freeze (2026-10-12:
host 300 s pairs, gen2 pauses >=20 ms 6 -> 0 per run, RSS growth unchanged).

Input accounting (v8, 2026-10-12): the depth callback feeds every arrival's header stamp to
host.geo_producer.InputGapCounter before the rate limit and the mailbox. A camera frame that never
reached the callback (lost in the DDS transport / reader) is an ``input_gap`` row in --stats-out and
summary "input" {arrivals, gaps, missed_frames, period_ms}; a frame that arrived and was skipped by
--rate-hz is counts.rate_skipped; one overwritten in the mailbox is slot.superseded. Evidence only.
--input-log FILE (diagnosis): one CSV row per arrival (stamp ns, RAW arrival ns, put | rebase |
rate_skipped | stamp_repeat | stamp_older).

Rate gate (2026-10-15, runs/car-bench-20261015-results/FUSED_DROP_DIG.md): --rate-gate auto (default) =
stamp for --data-source live, arrival (the gate before 10-15) for replay and sim. stamp
releases a frame by its header.stamp step, threshold (k - 0.5) camera periods (host.geo_producer.RateGate);
before, the callback arrival time against 90 ms released every 4th frame when depth arrived in pairs.
A repeated or older stamp is never released (counts stamp_repeat_skipped / stamp_older_skipped); every
frame_stats body carries "gate" (mode in effect, switch, basis, k, threshold_ms, step_ms of this frame, cumulative skip counts by
reason), the summary "rate_gate".

B1 navigation grid (2026-11-02, runs/nationals-prelim-20261102/B1/NANO_SIDE.md; default --pl-grid off = all of the
above unchanged): --pl-grid on|shadow asks the KV260 for the navigation planes with every INTEGRATE (ROLL v2,
HELLO "rolling_grid") and publishes /fpga/rolling_map/grid, /grid_raw and /status from here (host/ros_rolling_nav
.GridOutput over host/pl_grid_layer.PlGridLayer: the bridge's body seed, swept clearing and freshness with the
PL's strict + tall planes; no 3-D mirror on the Nano). on: the bridge is not running -- no deltas are published
and their per-lane change lists are not decoded. shadow: deltas published as before for the bridge running on
the _sw topics (car comparison). A KV260 without the grid op (or --rolling-backend cpu): warned, and the same
outputs come from an in-process bridge fed with this node's own deltas (status grid_source "nano_sw").
--grid-check-s N: every N s a DUMP of the KV map projected in Python is compared with the latest PL planes
(pl_grid_check rows / a pl_grid_check_mismatch event on /fpga/status; warn only, never stops anything).

Usage (inside the ROS image, from the repo root):
  python3 -m host.ros_geo_producer --data-source live --kv-server 198.51.100.79:47800 \
      --kv-expect-build-id 0x07784f75 --ground-z-m 0.0 [--rate-hz 10] [--stats-out run/frames.jsonl] \
      [--summary-out run/summary.json]
"""

import argparse
import collections
import json
import os
import signal
import sys
import threading
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)



def depth_reliable(a):
    """--depth-qos resolved: auto = reliable for --data-source live only (2026-10-15, owner)."""
    return a.depth_qos == "reliable" or (a.depth_qos == "auto" and a.data_source == "live")

def rate_gate_mode(a):
    """--rate-gate resolved -> (mode, basis). auto (default, controller 2026-10-15) = stamp for
    --data-source live only (the real camera's header.stamp), arrival otherwise: the sim's stamps are
    sim time (RTF 0.5 would halve the stamp gate's rate, 5 Hz instead of the accepted 7.5 Hz) and
    replay / sim keep the gate their D1/D2 closed loops were accepted with. Same test as --depth-qos auto."""
    if a.rate_gate != "auto":
        return a.rate_gate, "--rate-gate %s" % a.rate_gate
    return ("stamp" if a.data_source == "live" else "arrival"), "auto: --data-source %s" % a.data_source


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-source", choices=("live", "replay", "sim"), required=True,
                    help="where /fpga/depth comes from: live (camera driver), replay (a recording / "
                         "replay stand-in) or sim (Gazebo via robot_sim/fpga_sim/scripts/sim_adapter.py); "
                         "replay and sim set the KV260 replay mark on every request (the AR core can only "
                         "say LIVE or REPLAY: a simulated frame is never shown as LIVE). No default")
    ap.add_argument("--backend", choices=("remote_pl", "cpu"), default="remote_pl",
                    help="remote_pl: the KV260 (car); cpu: local reference core (host rig only)")
    ap.add_argument("--kv-server", help="KV260 kv_server HOST:PORT")
    ap.add_argument("--rolling-backend", choices=("cpu", "remote_pl"), default="cpu",
                    help="rolling map: cpu = this Nano (car default); remote_pl = the KV260 PL "
                         "rolling core (v3 candidate bitstream m7_pl_v3f, kv_server --rolling pl), "
                         "fused after this node validated each frame (host/kv_rolling.py)")
    ap.add_argument("--kv-expect-build-id", default=None)
    ap.add_argument("--kv-timeout-s", type=float, default=0.3)
    ap.add_argument("--kv-hello-timeout-s", type=float, default=0.5)
    ap.add_argument("--kv-retry-s", type=float, default=5.0)
    ap.add_argument("--kv-connect-wait-s", type=float, default=30.0, help="give up at startup after this")
    ap.add_argument("--reconnect-after", type=int, default=3, help="consecutive link failures before re-HELLO")
    ap.add_argument("--depth-topic", default="/fpga/depth")
    ap.add_argument("--info-topic", default="/fpga/camera_info")
    ap.add_argument("--camera-frame", default="camera_optical_frame")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--world-frame", default="map")
    ap.add_argument("--uint16-depth-scale", type=float, default=0.001, help="metres per 16UC1 code")
    ap.add_argument("--rate-hz", type=float, default=10.0)
    # 2026-10-15 (runs/car-bench-20261015-results/FUSED_DROP_DIG.md): the old gate (callback arrival
    # < 0.9/rate_hz = 90 ms after the last release -> skip) had 7 ms of margin on the 3rd camera frame
    # (96.9 ms at 30.96 fps); with depth frames delivered in pairs 64.6 ms apart it released every 4th
    # frame: 929 fused / 120 s (7.75 Hz). Default now (auto; live only): header.stamp, threshold (k - 0.5) camera
    # periods, k = fewest periods >= 0.9/rate_hz (30.96 fps, 10 Hz: 80.75 ms, every 3rd frame; the
    # board period moves between 32.3 and 33.4 ms, so check k = 3 and threshold = (k - 0.5) x the
    # measured period, not a fixed ms window);
    # host.geo_producer.RateGate
    ap.add_argument("--rate-gate", choices=("auto", "stamp", "arrival"), default="auto",
                    help="auto (default): stamp for --data-source live, arrival for replay and sim (sim "
                         "stamps are sim time; replay/sim keep their accepted gate). "
                         "stamp: gate on header.stamp steps, threshold (k - 0.5) source periods "
                         "(k = fewest periods >= 0.9 / --rate-hz; period measured from the stamps, "
                         "--rate-nominal-fps until known); repeated or older stamps never released. "
                         "arrival: the gate before 2026-10-15 (callback arrival < 0.9 / --rate-hz -> skip)")
    ap.add_argument("--rate-min-step-ms", type=float, default=0.0,
                    help="stamp gate: fixed threshold in ms of stamp time instead of the derived one (0 = auto)")
    ap.add_argument("--rate-nominal-fps", type=float, default=30.0,
                    help="stamp gate: source rate assumed until 8 stamp steps give the measured period")
    ap.add_argument("--rate-rebase-s", type=float, default=2.0,
                    help="stamp gate: after this long (RAW) of only older stamps (the source clock went "
                         "back), re-anchor on the newest frame (event depth_stamp_rebase)")
    ap.add_argument("--shape", default="32,32,10")
    ap.add_argument("--height-cell", type=int, default=7)
    ap.add_argument("--ground-z-m", type=float, required=True, help="ground height in odom (bridge's ground_z_m)")
    ap.add_argument("--tf-wait-s", type=float, default=0.05)
    ap.add_argument("--tf-listener-thread", action="store_true",
                    help="TF listener on its own executor thread (tf2_ros spin_thread=True) instead of "
                         "the main executor shared with the depth/memory callbacks")
    ap.add_argument("--tf-reliable", action="store_true",
                    help="dynamic /tf reader RELIABLE (tf2_ros default) instead of best effort; /tf_static "
                         "is reliable + transient_local either way (host.geo_producer.tf_listener_qos)")
    ap.add_argument("--depth-qos", choices=("auto", "reliable", "sensor_data"), default="auto",
                    help="depth image + depth CameraInfo subscriptions (host.geo_producer.depth_sub_qos): "
                         "reliable = RELIABLE KEEP_LAST --depth-qos-depth; sensor_data = best effort depth 5 "
                         "(before 2026-10-15); auto (default) = reliable for --data-source live (the Orbbec "
                         "driver offers RELIABLE), sensor_data otherwise (the replay camera and the sim "
                         "adapter offer BEST_EFFORT, which a reliable reader would not match)")
    ap.add_argument("--depth-reliable", dest="depth_qos", action="store_const", const="reliable",
                    help="same as --depth-qos reliable")
    ap.add_argument("--depth-qos-depth", type=int, default=5)
    ap.add_argument("--kv-depth-stride", type=int, default=1,
                    help="decimate the depth frame by this factor before the KV260 (block minimum of the valid "
                         "codes, intrinsics scaled; host.geo_producer.decimate_depth_codes). 1 = full frame "
                         "(default, the behaviour before 2026-10-16). The PL core time grows with the in-grid "
                         "samples: 2 cuts them to a quarter")
    ap.add_argument("--no-roi-labels", action="store_true", help="send no class names with the ROI rows")
    ap.add_argument("--no-roi", action="store_true")
    ap.add_argument("--geometry-v2", action="store_true",
                    help="10-27 geometry v2 ROI/thresholds/words (Config.geometry_v2; car_up CAR_GEOMETRY_V2=1 sets "
                         "it here and in the runtime together); off = unchanged")
    ap.add_argument("--depth-oor", choices=("invalidate", "reject"), default="invalidate",
                    help="depth pixels > 20 m (docs/design/DEPTH_OOR_INVALID_20261016.md): invalidate "
                         "(default) = mark them no-return (code 0, like the camera's near-range laser "
                         "protection) unless a gate fires; reject = refuse the whole frame as before")
    ap.add_argument("--depth-oor-frac-max", type=float, default=0.05,
                    help="invalidate: refuse the frame when OOR pixels / all pixels exceed this")
    ap.add_argument("--depth-oor-near-m", type=float, default=0.8,
                    help="invalidate: refuse the frame when any OOR pixel's ray meets the ground "
                         "(--ground-z-m) within this horizontal distance of the camera")
    ap.add_argument("--stats-out", default=None, help="JSONL, one row per processed frame (appended: a supervised restart keeps the earlier rows)")
    ap.add_argument("--summary-out", default=None)
    ap.add_argument("--duration-s", type=float, default=0.0)
    ap.add_argument("--stall-dump-ms", type=float, default=0.0,
                    help="faulthandler dump of all thread stacks when no Python thread could run for this "
                         "long (file <stats-out>.stacks.txt); 0 = off (default: re-arming faulthandler every "
                         "40 ms crashed the producer 4/4 on host within 400-960 s, off 4/4 survived 1500 s; "
                         "diagnostic use only, short runs)")
    ap.add_argument("--no-stall-diag", action="store_true",
                    help="no GC/GIL-watchdog/callback/graph diagnostics and no stack dumps")
    ap.add_argument("--graph-poll-s", type=float, default=0.5,
                    help="ROS graph poll period (node names, delta/frame subscriber counts) for the diagnostics")
    ap.add_argument("--dds-publish-mode", choices=("env", "sync", "async"), default="env",
                    help="Fast DDS publication mode of THIS process's writers (RMW_FASTRTPS_PUBLICATION_MODE set "
                         "before rclpy.init): async = publish() queues and a DDS thread sends (10-11 night "
                         "candidate fix (d) for the ~300 ms publish stall after a new participant joins); "
                         "env = leave the environment (rmw default SYNCHRONOUS)")
    ap.add_argument("--record-write", choices=("async", "sync"), default="async",
                    help="--stats-out rows: async (background writer thread, bounded queue, drops counted) or "
                         "sync (written on the calling thread, before v7)")
    ap.add_argument("--gc", choices=("default", "freeze"), default="freeze",
                    help="garbage collector: freeze (default since v7 host measurement, 2026-10-12: gc.collect + "
                         "gc.freeze once the node is built) or default (unchanged interpreter; control arm)")
    ap.add_argument("--input-log", default=None,
                    help="new CSV, one row per depth arrival in the callback: stamp_ns, arrival_raw_ns, "
                         "disposition (put | rebase | rate_skipped | stamp_repeat | stamp_older); written like --stats-out (--record-write). Diagnosis")
    ap.add_argument("--debug-publish-stall", default=None, help=argparse.SUPPRESS)   # FRAME:MS:hold|free
    # B1 navigation grid (2026-11-02): off = exactly as before
    ap.add_argument("--pl-grid", choices=("off", "on", "shadow"), default="off",
                    help="publish /fpga/rolling_map/grid + status from the KV260 navigation planes (see above)")
    ap.add_argument("--grid-robot-height-m", type=float, default=None,
                    help="obstacle band robot height (the bridge's robot_height_m; required with --pl-grid)")
    ap.add_argument("--grid-robot-radius-m", type=float, default=0.30)
    ap.add_argument("--grid-body-margin-m", type=float, default=0.05)
    ap.add_argument("--grid-swept-clear-s", type=float, default=300.0)
    ap.add_argument("--grid-front-seed-m", type=float, default=0.0)
    ap.add_argument("--grid-max-map-age-ms", type=float, default=200.0)
    ap.add_argument("--grid-status-hz", type=float, default=20.0)
    ap.add_argument("--grid-check-s", type=float, default=10.0,
                    help="DUMP cross-check period (0 = off); only with the PL grid source")
    # AR2NANO2 (2026-11-02): the car screen drawn on this Nano (host/nano_ar.py; car_up CAR_AR_NANO=1). Off = as before.
    ap.add_argument("--ar-nano", action="store_true",
                    help="draw the car screen here (m7ar_present --provider cpu on DRM) from every KV answer; "
                         "display only: a failed start is reported (AR_NANO_FAILED) and the producer runs on")
    ap.add_argument("--ar-nano-out", default=None, help="presenter logs (ar_events.jsonl, ar_helper_events.jsonl)")
    ap.add_argument("--ar-nano-mode", default="1024x600", help="DRM mode WxH (the 7-inch screen)")
    ap.add_argument("--ar-nano-modeline", default=None, help="forced modeline for a converter without EDID")
    ap.add_argument("--ar-nano-colour-port", type=int, default=47801,
                    help="colour main view: ColourSink on 127.0.0.1:PORT (nano_colour.py --kv 127.0.0.1:PORT); -1 = off")
    ap.add_argument("--ar-nano-tune-file", default=None,
                    help="default: ui/geomem/tools/competition/kv_ar_tune.json (the KV's live tune file)")
    a, ros_args = ap.parse_known_args(argv)
    if a.backend == "remote_pl" and (not a.kv_server or ":" not in a.kv_server):
        ap.error("--backend remote_pl needs --kv-server HOST:PORT")
    if a.kv_depth_stride < 1:
        ap.error("--kv-depth-stride must be >= 1")
    if not (a.rate_hz > 0 and a.uint16_depth_scale > 0 and a.tf_wait_s >= 0):
        ap.error("rate, depth scale and TF wait must be positive")
    if not (a.rate_min_step_ms >= 0 and a.rate_nominal_fps > 0 and a.rate_rebase_s > 0):
        ap.error("--rate-min-step-ms >= 0, --rate-nominal-fps and --rate-rebase-s > 0")
    return a, ros_args


def transform_matrix(t):
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
    return T


class _LockedFusion(object):
    """RollingProducerLink answers resync requests from the executor thread while the worker
    integrates: both go through one lock."""

    def __init__(self, fusion, lock):
        self._f, self._lock, self.voxel_m = fusion, lock, fusion.voxel_m

    def snapshot(self, **kw):
        with self._lock:
            return self._f.snapshot(**kw)


def start_ar_nano(a, kv_server):
    """AR2NANO2: NanoArScreen (host/nano_ar.py) or None. Never stops the producer (owner 10-26: no new
    refusals; the screen is display only)."""
    try:
        from host.nano_ar import NanoArScreen
        tune = a.ar_nano_tune_file or os.path.join(ROOT, "ui", "geomem", "tools", "competition", "kv_ar_tune.json")
        out = a.ar_nano_out
        if out:
            os.makedirs(out, exist_ok=True)
        scr = NanoArScreen.start(ROOT, out_dir=out, tune_file=tune if os.path.isfile(tune) else None,
                                 mode=a.ar_nano_mode, modeline=a.ar_nano_modeline, kv_server=kv_server,
                                 colour_port=None if a.ar_nano_colour_port < 0 else a.ar_nano_colour_port)
        sys.stderr.write("AR_NANO_READY %s\n" % json.dumps({"ready": scr.ar.ready, "out": out}, default=str))
        return scr
    except Exception as exc:        # noqa: BLE001 -- display only
        sys.stderr.write("AR_NANO_FAILED %s: %s -- car screen dark, the producer runs on\n"
                         % (type(exc).__name__, exc))
        return None


def main(argv=None):
    a, ros_args = parse_args(argv)
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    import rclpy.qos as rclpy_qos
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    from host.geo_producer import (SENSOR_SOURCE_TOPIC, FrameGapTracker, GeoProducer, InputGapCounter, RateGate,
                                   depth_sub_qos,
                                   KvConnection, KvReconnect, SourceMarker, calibration_id,
                                   check_block_height, run_worker, tf_listener_qos)
    from host.geometry_region_glue import MemoryWatch, frame_stats
    from host.fpga_metrics import frame_fields as fpga_frame_fields
    from host.kv_link import LatestSlot, m_to_codes
    from host.m7_backend import M7BackendSelector
    from host.m7_rolling import compile_rolling
    from host.fpga_clock import now_ns
    from host.ros_rolling_nav import RollingProducerLink
    from host import gc_policy, stall_watch
    from host.record_writer import DropReporter, RecordWriter
    import tempfile

    shape = tuple(int(v) for v in a.shape.split(","))
    build = tempfile.mkdtemp(prefix="geoprod_")
    roll = None
    if a.rolling_backend == "remote_pl":
        # owner 10-26: no new refusals -- if the KV260 rolling core is not usable the producer warns
        # and fuses on the Nano CPU (the a2 path) instead of exiting.
        if a.backend != "remote_pl" or not a.kv_server:
            sys.stderr.write("--rolling-backend remote_pl needs --backend remote_pl --kv-server; "
                             "fusing on the Nano CPU instead\n")
        else:
            from host.kv_rolling import remote_rolling_factory
            from host.m7_backend import PlBackendUnavailable
            rh, rp = a.kv_server.rsplit(":", 1)
            try:
                roll = M7BackendSelector(remote_rolling_factory=remote_rolling_factory(
                    (rh, int(rp)), hello_timeout_s=a.kv_connect_wait_s)).rolling_backend("remote_pl")
            except PlBackendUnavailable as exc:
                sys.stderr.write("WARNING: KV260 rolling core not usable (%s); fusing on the Nano CPU instead "
                                 "(start the KV260 with geomem-kv-start v3fc2 to fuse on the PL)\n" % exc)
        if roll is None:
            a.rolling_backend = "cpu"
    if roll is None:
        roll = M7BackendSelector().rolling_backend("cpu", cpu_library=compile_rolling(os.path.join(build, "roll.so")))
    conn = None
    ar_nano = None
    if a.backend == "remote_pl":
        from host.kv_geometry import parse_build_id
        host, port = a.kv_server.rsplit(":", 1)
        conn = KvConnection((host, int(port)), depth_unit_m=a.uint16_depth_scale, timeout_s=a.kv_timeout_s,
                            hello_timeout_s=a.kv_hello_timeout_s,
                            expect_build_id=parse_build_id(a.kv_expect_build_id),
                            data_source=a.data_source)
        if a.ar_nano:
            ar_nano = start_ar_nano(a, (host, int(port)))
            if ar_nano is not None:
                conn.ar_sink = ar_nano.offer
        deadline = time.monotonic() + a.kv_connect_wait_s
        while not conn.connect():
            if time.monotonic() > deadline:
                sys.stderr.write("KV260 not usable: %s\n" % conn.reason)
                return 2
            time.sleep(1.0)
        conn.backend.roi_enabled = conn.roi_available and not a.no_roi
        backend = conn.backend
    else:
        from host.m7_frame_native import compile_frame_native
        backend = M7BackendSelector(cpu_frame_library=compile_frame_native(
            os.path.join(build, "frame.so"))).frame_backend("cpu")
    geo_cfg = None
    if a.geometry_v2:
        from perception.fpga_replay.core import Config
        geo_cfg = Config(geometry_v2=True)
        sys.stderr.write("geometry producer: geometry v2 ON (probe ROI, surface thresholds, 50 %% rule)\n")
    core = GeoProducer(backend, roll, depth_unit_m=a.uint16_depth_scale, shape=shape, cfg=geo_cfg,
                       height_cell=a.height_cell, world_frame=a.world_frame, roi=not a.no_roi,
                       roi_labels=not a.no_roi_labels, depth_oor=a.depth_oor, ground_z_m=a.ground_z_m,
                       oor_frac_max=a.depth_oor_frac_max, oor_near_m=a.depth_oor_near_m,
                       depth_stride=a.kv_depth_stride)
    grid_layer, grid_source, grid_band = None, None, None
    if a.pl_grid != "off":
        from host import rolling_nav_layer as rnl
        from host.pl_grid_layer import PlGridLayer
        from host.fpga_clock import CLOCK_ID as _GRID_CLOCK
        if a.grid_robot_height_m is None:
            sys.stderr.write("--pl-grid needs --grid-robot-height-m\n")
            return 2
        grid_band = rnl.HeightBand(ground_z_m=a.ground_z_m, robot_height_m=a.grid_robot_height_m)
        layer_kw = dict(dims=core.fusion.backend.config.dims, voxel_m=core.voxel_m, band=grid_band,
                        clock_id=_GRID_CLOCK, frame_id=a.odom_frame,
                        max_map_age_ns=int(a.grid_max_map_age_ms * 1e6), body_radius_m=a.grid_robot_radius_m,
                        body_margin_m=a.grid_body_margin_m, front_seed_m=a.grid_front_seed_m,
                        swept_clear_s=float(a.grid_swept_clear_s))
        if core.fusion.enable_grid(grid_band, skip_changes=(a.pl_grid == "on")):
            grid_layer, grid_source = PlGridLayer(**layer_kw), "pl"
        else:
            # owner 10-26: no new refusals -- the same outputs from an in-process bridge (the software merge)
            sys.stderr.write("WARNING --pl-grid %s: the KV260 has no navigation-grid op (rolling backend %s, no "
                             "\"rolling_grid\" in its HELLO); the grid comes from an in-process bridge on this "
                             "node's deltas instead\n" % (a.pl_grid, a.rolling_backend))
            grid_layer, grid_source = rnl.RollingLayerBridge(**layer_kw), "nano_sw"
    source = SourceMarker(a.data_source)
    lock = threading.Lock()
    diag = not a.no_stall_diag
    debug_stall = stall_watch.parse_debug_stall(a.debug_publish_stall)
    diag_events = stall_watch.EventQueue()
    # v7: rows through a RecordWriter; sync keeps the old unflushed buffered write under a lock
    stats = (RecordWriter(open(a.stats_out, "a"), mode=a.record_write, name="frames", flush_each=False,
                          on_slow=diag_events.put if diag else None) if a.stats_out else None)

    def write_row(row):
        if stats is not None:
            stats.write(json.dumps(row, default=str) + "\n")

    # v8: optional per-arrival log (diagnosis of input drops), same writer discipline as --stats-out
    input_log = (RecordWriter(open(a.input_log, "a"), mode=a.record_write, name="input", flush_each=False)
                 if a.input_log else None)
    if input_log is not None:
        input_log.write("stamp_ns,arrival_raw_ns,disposition\n")

    gc_pauses = stall_watch.GcPauses(diag_events, min_ms=10.0)
    calls = stall_watch.CallTimer(diag_events, warn_ms=20.0, event_ms=50.0)
    # section timing (rows for long post-fusion publish sections) without faulthandler; the stack
    # dumps come from the dead man's switch re-armed by the GIL watchdog (any thread, any call)
    dump = stall_watch.StallDump(None, timeout_s=0.1, events=diag_events)
    deadman = stall_watch.DeadmanDump((a.stats_out + ".stacks.txt") if (a.stats_out and diag and a.stall_dump_ms > 0)
                                      else None, timeout_s=a.stall_dump_ms / 1e3)
    if diag:
        gc_pauses.install()

    rmw_pub_mode = stall_watch.apply_dds_publish_mode(a.dds_publish_mode, os.environ)
    rclpy.init(args=ros_args)

    class RecordingBuffer(Buffer):
        """tf2 Buffer that also keeps the RAW arrival of the last odom<-base_link samples, so a
        failed lookup can say what the buffer had (evidence for TF gaps, not used for poses)."""

        def __init__(self, *args, **kw):
            super().__init__(*args, **kw)
            self.arrivals = collections.deque(maxlen=64)     # (arrival RAW ns, stamp ns)

        def set_transform(self, transform, authority):
            if transform.child_frame_id == a.base_frame and transform.header.frame_id == a.odom_frame:
                st = transform.header.stamp
                self.arrivals.append((now_ns(), st.sec * 1000000000 + st.nanosec))
            return super().set_transform(transform, authority)

    class TimedLink(RollingProducerLink):
        def on_request(self, msg):            # resync answers run on the main executor thread
            t0 = now_ns()
            try:
                return super().on_request(msg)
            finally:
                calls.record("on_resync_request", t0, now_ns())

    class GraphPoll(threading.Thread):
        """Node names and subscriber counts of the producer's two main writers, polled; every
        change is an event row with RAW ns, and the poll's own duration is kept (a graph query
        that blocks during a stall is evidence too)."""

        def __init__(self, node, period_s):
            super().__init__(name="graph-poll", daemon=True)
            self.node, self.period_s = node, period_s
            self.stop_event = threading.Event()
            self.last = None
            self.polls, self.max_poll_ms, self.changes = 0, 0.0, 0

        def run(self):
            while not self.stop_event.wait(self.period_s):
                t0 = now_ns()
                try:
                    names = sorted("%s%s" % (ns.rstrip("/") + "/", n) for n, ns in
                                   self.node.get_node_names_and_namespaces())
                    cur = {"nodes": names, "delta_subs": self.node.link.delta_pub.get_subscription_count(),
                           "frame_subs": self.node.frame_pub.get_subscription_count()}
                except Exception as exc:  # noqa: BLE001 - diagnostics never stop the producer
                    diag_events.put({"event": "graph_poll_error", "error": repr(exc)[:200]})
                    continue
                t1 = now_ns()
                self.polls += 1
                self.max_poll_ms = max(self.max_poll_ms, (t1 - t0) / 1e6)
                if (t1 - t0) >= 50e6:
                    diag_events.put({"event": "slow_call", "name": "graph_poll", "ms": (t1 - t0) / 1e6,
                                     "start_raw_ns": t0, "end_raw_ns": t1})
                if self.last is None or cur != self.last:
                    prev = self.last or {"nodes": [], "delta_subs": None, "frame_subs": None}
                    self.changes += 1
                    diag_events.put({"event": "graph_change", "raw_ns": t1, "poll_ms": (t1 - t0) / 1e6,
                                     "n_nodes": len(names),
                                     "added": sorted(set(names) - set(prev["nodes"])),
                                     "removed": sorted(set(prev["nodes"]) - set(names)),
                                     "delta_subs": cur["delta_subs"], "frame_subs": cur["frame_subs"]})
                    self.last = cur

        def summary(self):
            return {"polls": self.polls, "max_poll_ms": self.max_poll_ms, "changes": self.changes,
                    "nodes": len((self.last or {}).get("nodes") or [])}

    class Producer(Node):
        def __init__(self):
            super().__init__("fpga_geo_producer")
            self.tf = RecordingBuffer()
            # dynamic /tf best effort unless --tf-reliable; static_qos left to tf2_ros (reliable,
            # transient_local). Lookups below are unchanged: interpolation only, no extrapolation.
            self.tf_listener = TransformListener(self.tf, self, spin_thread=a.tf_listener_thread,
                                                 qos=tf_listener_qos(rclpy_qos, best_effort=not a.tf_reliable))
            self.gaps = FrameGapTracker()
            self.input_gaps = InputGapCounter()      # v8: frames lost before the depth callback
            self.status_pub = self.create_publisher(String, "/fpga/status", 10)
            self.info = None
            self.slot = LatestSlot()
            gate_mode, gate_basis = rate_gate_mode(a)    # auto: stamp for live only
            self.gate = RateGate(a.rate_hz, mode=gate_mode, min_step_ms=a.rate_min_step_ms,
                                 nominal_fps=a.rate_nominal_fps, rebase_s=a.rate_rebase_s,
                                 switch=a.rate_gate, basis=gate_basis)
            self.counts = {"depth_rx": 0, "rate_skipped": 0, "no_info": 0, "tf_missing": 0,
                           "world_tf_missing": 0, "bad_depth": 0, "processed": 0, "published_deltas": 0,
                           "block_refused": 0, "reconnects": 0}
            self.block = None
            self.calib = None
            self.memory = MemoryWatch(a.world_frame)
            self.link = (TimedLink if diag else RollingProducerLink)(
                self, _LockedFusion(core.fusion, lock),
                answer_thread=(a.rolling_backend == "remote_pl"))   # DUMP off the executor thread
            self.grid_out = None
            self.grid_check_next = None
            if grid_layer is not None:
                from host.ros_rolling_nav import GridOutput
                self.grid_out = GridOutput(self, grid_layer, pose=self.latest_pose, source=grid_source,
                                           status_hz=a.grid_status_hz)
                self.counts.update(grid_applied=0, grid_missing=0, grid_checks=0, grid_check_mismatch=0)
                if grid_source == "pl" and a.grid_check_s > 0:
                    self.grid_check_next = time.monotonic() + a.grid_check_s
            self.frame_pub = self.create_publisher(String, "/fpga/geometry_frame", 50)
            self.stats_pub = self.create_publisher(String, "/fpga/kv_link/frame_stats", 10)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            cb = (lambda name, fn: calls.wrap(name, fn)) if diag else (lambda name, fn: fn)
            self.create_subscription(String, "/fpga/memory", cb("on_memory", self.on_memory), map_qos)
            self.create_subscription(String, SENSOR_SOURCE_TOPIC, lambda m: source.on_marker(m.data), map_qos)
            depth_qos = depth_sub_qos(rclpy_qos, reliable=depth_reliable(a), depth=a.depth_qos_depth)
            self.create_subscription(CameraInfo, a.info_topic, cb("on_info", self.on_info), depth_qos)
            self.create_subscription(Image, a.depth_topic, cb("on_depth", self.on_depth), depth_qos)
            # link health (re-HELLO after consecutive link failures, rate limited): host.geo_producer
            self.health = (KvReconnect(conn, reconnect_after=a.reconnect_after, retry_s=a.kv_retry_s,
                                       roi_enabled=not a.no_roi) if conn is not None else None)
            self.drop_report = DropReporter(1.0)
            self.stop = threading.Event()
            self.fatal = None          # why the worker gave up (fail-stop: main returns 4)
            self.worker = threading.Thread(target=self.run, name="geo-producer", daemon=True)
            self.worker.start()

        def on_memory(self, msg):
            self.memory.on_memory(msg.data, time.monotonic())

        def on_info(self, msg):
            self.info = msg

        def on_depth(self, msg):
            t = now_ns()                                   # acquisition = RAW arrival here
            self.counts["depth_rx"] += 1
            st = msg.header.stamp
            stamp_ns = st.sec * 1000000000 + st.nanosec
            gap = self.input_gaps.feed(stamp_ns, t)        # v8: lost before this callback (evidence only)
            if gap is not None:
                write_row(gap)
            # 2026-10-15: stamp-based gate (host.geo_producer.RateGate; FUSED_DROP_DIG.md)
            release, reason, ev = self.gate.decide(stamp_ns, t, self.input_gaps.period_ns())
            if ev is not None:
                self.event(ev)                            # depth_stamp_regression / _rebase on /fpga/status
            if not release:
                if reason in ("rate", "rate_arrival"):
                    self.counts["rate_skipped"] += 1
                else:                                     # stamp_repeat | stamp_older: never released
                    self.counts[reason + "_skipped"] = self.counts.get(reason + "_skipped", 0) + 1
                if input_log is not None:
                    input_log.write("%d,%d,%s\n" % (stamp_ns, t, "rate_skipped" if reason.startswith("rate")
                                                      else reason))
                return
            if input_log is not None:
                input_log.write("%d,%d,%s\n" % (stamp_ns, t, reason or "put"))
            self.slot.put({"msg": msg, "acquired_ns": t, "info": self.info})

        def lookup(self, target, source, stamp, wait_s):
            return transform_matrix(self.tf.lookup_transform(target, source, stamp,
                                                             timeout=Duration(seconds=wait_s)).transform)

        def run(self):
            # one bad frame never stops the producer (10-26: WORKER_ERROR_LIMIT 0 = no limit); the loop itself
            # failing, does: stop + self.fatal -> main returns 4 (car_inner.sh fail-stop), never a
            # live node that produces no frame (2026-10-15)
            fatal = run_worker(self.stop, lambda: self.slot.take(timeout=0.1), self.one,
                               on_error=self.frame_error, on_fatal=lambda why: setattr(self, "fatal", why))
            if fatal is not None:
                try:
                    self.get_logger().fatal("worker stopped: %s" % fatal)
                    self.event({"event": "producer_fatal", "source": "geo_producer", "reason": fatal,
                                "raw_ns": now_ns()})
                except Exception:  # noqa: BLE001 - the exit code carries it anyway
                    pass

        def frame_error(self, exc, streak):
            self.counts["worker_error:" + type(exc).__name__] = \
                self.counts.get("worker_error:" + type(exc).__name__, 0) + 1
            self.get_logger().error("frame failed (%d in a row): %r" % (streak, exc))

        def event(self, ev):
            if ev is not None:
                self.status_pub.publish(String(data=json.dumps(ev, allow_nan=False)))
                write_row(ev)

        def note_record_drops(self, final=False):
            """v7: dropped --stats-out rows (async queue full) -> record_drops on /fpga/status
            (hub STATE geometry_nano.record_drops) at the first drop, then <= 1/s, and at exit."""
            if stats is not None and self.drop_report.due(stats.dropped, time.monotonic(), final=final):
                self.event({"event": "record_drops", "process": "producer", "dropped": stats.dropped,
                            "dropped_bytes": stats.counts["dropped_bytes"], "raw_ns": now_ns()})

        def tf_evidence(self, stamp_ns, acq):
            """What the buffer had when a lookup failed: newest odom<-base_link sample (stamp,
            RAW arrival) and how many arrived in the last second."""
            arr = list(self.tf.arrivals)
            if not arr:
                return {"tf_samples": 0}
            last_rx, last_stamp = arr[-1]
            return {"tf_latest_stamp_lag_ms": (stamp_ns - last_stamp) / 1e6,
                    "tf_last_arrival_ms_before_acq": (acq - last_rx) / 1e6,
                    "tf_arrivals_1s": sum(1 for rx, _ in arr if acq - rx <= 1000000000)}

        def latest_pose(self):
            """B1 GridOutput: the latest odom<-base_link (the bridge's body_pose: Time() = newest), or None."""
            try:
                tf = self.tf.lookup_transform(a.odom_frame, a.base_frame, Time())
            except Exception:  # noqa: BLE001 - no pose: the strict grid (as the bridge)
                return None
            q = tf.transform.rotation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (tf.transform.translation.x, tf.transform.translation.y, float(yaw))

        def grid_update(self, res):
            """B1: the frame's map -> grid layer -> /fpga/rolling_map/grid + status (the bridge's on_delta)."""
            delta = res["delta"]
            if delta is None:
                self.grid_out.publish_status()
                return
            if grid_source == "pl":
                planes = res.get("pl_grid")
                with self.grid_out.lock:
                    ok, _why = grid_layer.on_planes(epoch=delta.epoch, delta_seq=delta.delta_seq,
                                                    window_base=delta.window_base, acquired_ns=delta.acquired_ns,
                                                    clock_id=delta.clock_id, planes=planes)
            else:
                with self.grid_out.lock:
                    ok, _why, request = grid_layer.on_delta(delta, now_ns=now_ns())
                if request is not None:             # in-process resync: the snapshot straight from the fusion
                    from host.rolling_nav_layer import decode_snapshot, encode_snapshot
                    with lock:
                        snap = core.fusion.snapshot(now_ns=now_ns(), clock_id=delta.clock_id)
                    if snap.valid:
                        wire = decode_snapshot(encode_snapshot(snap, acquired_ns=delta.acquired_ns,
                                                               clock_id=delta.clock_id, voxel_m=core.voxel_m))
                        with self.grid_out.lock:
                            ok, _why, _r = grid_layer.on_snapshot(wire, now_ns=now_ns())
            self.counts["grid_applied" if ok else "grid_missing"] += 1
            if ok:
                self.grid_out.publish_grid()
            else:
                self.grid_out.publish_status()

        def grid_check(self):
            """B1 coverage (DESIGN 6.5): DUMP of the KV map, projected in Python, vs the latest PL planes."""
            from host.pl_grid_layer import dump_check
            t0 = now_ns()
            with lock:
                snap = core.fusion.snapshot(now_ns=now_ns(), clock_id=core.clock_id)
            if not snap.valid:
                row = {"result": "not_comparable", "reason": snap.reason}
            else:
                with self.grid_out.lock:
                    row = dump_check(snap, grid_layer, grid_band)
            row.update(event="pl_grid_check", raw_ns=now_ns(), ms=(now_ns() - t0) / 1e6)
            self.counts["grid_checks"] += 1
            write_row(row)
            if row["result"] == "mismatch":
                self.counts["grid_check_mismatch"] += 1
                self.get_logger().warn("PL grid differs from the DUMP projection: %s" % json.dumps(row))
                self.event(dict(row, event="pl_grid_check_mismatch", source="geo_producer"))

        def one(self, item):
            msg, acq, info = item["msg"], item["acquired_ns"], item["info"]
            t_take = now_ns()
            if info is None:
                self.counts["no_info"] += 1
                self.event(self.gaps.missing("no_camera_info", acq))
                return
            stamp = Time.from_msg(msg.header.stamp)
            try:
                T_ob = self.lookup(a.odom_frame, a.base_frame, stamp, a.tf_wait_s)
                T_bc = self.lookup(a.base_frame, a.camera_frame, stamp, a.tf_wait_s)
            except Exception as exc:  # noqa: BLE001 - TF exceptions: no pose at this acquisition
                self.counts["tf_missing"] += 1
                phase = "running" if self.gaps.ready else "startup"
                ev = self.tf_evidence(stamp.nanoseconds, acq)
                if stats is not None:
                    row = {"event": "tf_missing", "phase": phase, "depth_stamp_ns": stamp.nanoseconds,
                           "acquired_ns": acq, "waited_ms": (now_ns() - t_take) / 1e6,
                           "error": "%s: %s" % (type(exc).__name__, str(exc)[:200])}
                    row.update(ev)
                    write_row(row)
                self.event(self.gaps.missing("tf_missing", acq, **ev))
                return
            self.event(self.gaps.ok(acq))
            T_wc = None
            if not a.no_roi:
                try:
                    T_wc = self.lookup(a.world_frame, a.camera_frame, stamp, 0.0)
                except Exception:  # noqa: BLE001
                    self.counts["world_tf_missing"] += 1
            t_tf = now_ns()
            T_oc = T_ob @ T_bc
            if self.block is None:
                ok, self.block = check_block_height(T_oc[2, 3], a.ground_z_m, shape=shape, height_cell=a.height_cell)
                self.get_logger().info("block check %s: %s" % (ok, json.dumps(self.block)))
                if not ok:
                    self.counts["block_refused"] += 1
                    self.get_logger().error("per-frame block does not hold the ground; refusing to produce")
                    self.stop.set()
                    return
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint16 if msg.encoding == "16UC1" else np.float32)
            try:
                if msg.encoding == "16UC1":
                    codes = raw.reshape(msg.height, msg.width)
                elif msg.encoding == "32FC1":
                    d = raw.reshape(msg.height, msg.width).copy()
                    d[~(d > 0)] = np.nan
                    codes = m_to_codes(d, np.float32(a.uint16_depth_scale)).reshape(msg.height, msg.width)
                else:
                    raise ValueError("encoding %s" % msg.encoding)
            except ValueError:
                self.counts["bad_depth"] += 1
                return
            K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            codes, K = core.prepare_depth(codes, K)      # --kv-depth-stride (identity at 1)
            calib = calibration_id(K, T_bc, codes.shape[1], codes.shape[0])
            mem, mem_reason = self.memory.usable(time.monotonic())
            with lock:
                res = core.process(codes, K, depth_stamp_ns=stamp.nanoseconds, acquired_ns=acq,
                                   T_odom_camera=T_oc, robot_position_m=tuple(T_ob[:3, 3]),
                                   calibration_id=calib, T_world_camera=T_wc, memory_msg=mem,
                                   memory_reason=mem_reason)
            t_fused = dump.begin() if diag else now_ns()
            c_fused = time.thread_time_ns()
            tm = {}
            if debug_stall is not None and res["frame_id"] == debug_stall[0]:
                (stall_watch.hold_gil_sleep if debug_stall[2] == "hold" else stall_watch.free_gil_sleep)(debug_stall[1])
            if res["delta"] is not None and a.pl_grid != "on":   # B1 "on": no bridge, no deltas
                self.link.publish(res["delta"], timings=tm)
                self.counts["published_deltas"] += 1
            t_pub = now_ns()
            body = res["body"]
            body["producer"]["publish_raw_ns"] = t_pub
            frame_msg = String(data=json.dumps(body, allow_nan=False))
            t_fjson = now_ns()
            self.frame_pub.publish(frame_msg)
            t_fpub = now_ns()
            last = res["client_last"] or {}
            fs = frame_stats(kv_frame_id=res["frame_id"], capture_nano_ns=acq, done_nano_ns=t_pub,
                             result=res["result"], mailbox_wait_ms=(t_take - acq) / 1e6,
                             client_last=last if "kv_ns" in last else None,
                             client_counts=conn.client.counts if conn is not None and conn.client else None)
            fs.update(source.stats_fields())
            self.event(source.transition(fs))      # source_warning(_cleared) on /fpga/status, edge only
            fs["gate"] = self.gate.stats()          # 2026-10-15: why arrivals were not released (cumulative)
            # 10-29 console FPGA panel (display only): this frame's PL core times / pixels / boxes / changed cells
            fs["fpga"] = fpga_frame_fields(client_last=last if "kv_ns" in last else None, roi=res.get("roi_outcome"),
                                           roll_times_ns=res.get("roll_times_ns"), roll_core=res.get("roll_core"),
                                           roll_changes=body["producer"]["n_changes"], nav_counts=res.get("nav_counts"))
            t_sjson = now_ns()
            self.stats_pub.publish(String(data=json.dumps(fs, allow_nan=False)))
            t_spub = now_ns()
            t_grid = None
            if self.grid_out is not None:           # B1: after the frame topics (their consumers are not delayed)
                self.grid_update(res)
                t_grid = now_ns()
            c_end = time.thread_time_ns()
            split = {"enc_ms": (tm["enc1"] - tm["enc0"]) / 1e6 if "enc1" in tm else None,
                     "dpub_ms": (tm["pub1"] - tm["enc1"]) / 1e6 if "pub1" in tm else None,
                     "fjson_ms": (t_fjson - t_pub) / 1e6, "fpub_ms": (t_fpub - t_fjson) / 1e6,
                     "spub_ms": (t_spub - t_sjson) / 1e6, "post_ms": (t_spub - t_fused) / 1e6,
                     "post_cpu_ms": (c_end - c_fused) / 1e6}
            if t_grid is not None:
                split["grid_ms"] = (t_grid - t_spub) / 1e6
            if diag:
                dump.end(t_fused, "post_fusion_publish", frame_id=res["frame_id"],
                         **dict((k, round(v, 3)) for k, v in split.items() if v is not None))
            self.counts["processed"] += 1
            if stats is not None:
                row = {"frame_id": res["frame_id"], "depth_stamp_ns": stamp.nanoseconds, "acquired_ns": acq,
                       "mailbox_wait_ms": (t_take - acq) / 1e6, "tf_ms": (t_tf - t_take) / 1e6,
                       "geometry_ms": (res["t_geo"] - res["t_start"]) / 1e6,
                       "fusion_ms": (res["t_fused"] - res["t_geo"]) / 1e6,
                       "age_ms": (t_pub - acq) / 1e6 if res["delta"] is not None else None,
                       "done_age_ms": (t_pub - acq) / 1e6, "result": res["result"],
                       "nav_reason": res["nav_reason"], "fusion_reason": res["fusion_reason"],
                       "roi": body["failure"] or "ok", "roi_core": body.get("roi_core"),
                       "rtt_ms": last["rtt_s"] * 1e3 if last.get("rtt_s") is not None else None,
                       "kv_core_device_ms": (last.get("kv_ns") or {}).get("core_device", 0) / 1e6
                       if last.get("kv_ns") else None,
                       "n_changes": body["producer"]["n_changes"], "superseded": self.slot.superseded,
                       "input_missed": self.input_gaps.counts["missed_frames"],
                       "publish_ms": (t_pub - t_fused) / 1e6, "roi_rows": len((body.get("verdicts") or [])),
                       "labels_sent": core.counts.get("roi_labels_sent", 0),
                       "source_mismatch": fs["source_mismatch"],
                       "gate": fs["gate"]}                # 2026-10-15: same body as the frame_stats topic
                row.update(res["oor"])                    # 2026-10-16: oor_* (DEPTH_OOR_INVALID_20261016)
                row.update(split)
                write_row(row)
                self.note_record_drops()
            if self.grid_check_next is not None and time.monotonic() >= self.grid_check_next:
                self.grid_check_next = time.monotonic() + a.grid_check_s
                try:
                    self.grid_check()
                except Exception as exc:  # noqa: BLE001 - a check never stops the producer
                    self.counts["grid_check_error"] = self.counts.get("grid_check_error", 0) + 1
                    self.get_logger().warn("PL grid check failed: %r" % (exc,))
            # link health: re-HELLO after consecutive link failures (rate limited; a failed one keeps
            # the old session, see KvConnection.connect)
            if self.health is not None:
                act = self.health.after_frame(res["nav_reason"], core.set_backend)
                if act == "reconnected":
                    self.counts["reconnects"] += 1
                elif act == "reconnect_failed":
                    self.counts["reconnect_failed"] = self.counts.get("reconnect_failed", 0) + 1

    node = Producer()
    threads_after_node = stall_watch.thread_count()
    gc_info = gc_policy.apply(a.gc)         # v7: once the node is built (default: unchanged)

    def flush_events(_t=None):
        for ev in diag_events.drain():
            write_row(ev)

    watchdog = graph = None
    if diag:
        watchdog = stall_watch.GilWatchdog(diag_events, period_s=0.02, min_ms=30.0, on_tick=flush_events,
                                           deadman=deadman)
        watchdog.start()
        if a.graph_poll_s > 0:
            graph = GraphPoll(node, a.graph_poll_s)
            graph.start()
    end = time.monotonic() + a.duration_s if a.duration_s > 0 else None
    # 10-16: the SIGTERM handler only appends (no lock). It runs on the main thread between bytecodes,
    # and node.stop.set() in it deadlocks for good if it lands inside the main thread's own
    # node.stop.set() in the finally below (Event's non-reentrant lock), e.g. car_stop_pid's TERM after
    # an INT that already ended the loop. The loop checks the list every spin_once (<= 50 ms), as before.
    term = []
    signal.signal(signal.SIGTERM, lambda *_: term.append(1))
    try:
        while (rclpy.ok() and not term and not node.stop.is_set() and node.worker.is_alive() and
               (end is None or time.monotonic() < end)):
            if diag:
                t0 = now_ns()
                rclpy.spin_once(node, timeout_sec=0.05)
                calls.record("spin_once", t0, now_ns())       # wait (<= 50 ms) + at most one callback
            else:
                rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        worker_died = not node.worker.is_alive() and not node.stop.is_set() and not term
        node.stop.set()
        node.worker.join(timeout=2.0)
        if worker_died and node.fatal is None:
            node.fatal = "worker_exited"
        for th in (watchdog, graph):
            if th is not None:
                th.stop() if hasattr(th, "stop") else th.stop_event.set()
                th.join(timeout=2.0)
        gc_pauses.uninstall()
        dump.close()
        deadman.close()
        flush_events()
        node.note_record_drops(final=True)
        summary = {"counts": dict(node.counts), "producer": dict(core.counts), "describe": core.describe(),
                   "data_source": a.data_source, "source_marker": {"last": source.marker,
                                                                  "counts": dict(source.counts)},
                   "kv": conn.describe() if conn else None, "kv_counts": dict(conn.counts) if conn else None,
                   "link": dict(node.link.counts), "memory": dict(node.memory.counts), "block": node.block,
                   "gaps": dict(node.gaps.counts), "gap_state": node.gaps.state(),
                   "tf_listener_thread": a.tf_listener_thread,
                   "tf_qos": "reliable" if a.tf_reliable else "best_effort",
                   "depth_qos": ("reliable:%d" % a.depth_qos_depth) if depth_reliable(a) else "sensor_data",
                   "depth_qos_switch": a.depth_qos,
                   "dds_publish_mode": {"switch": a.dds_publish_mode, "rmw_env": rmw_pub_mode,
                                        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
                                        "threads_after_node": threads_after_node},
                   "stall_diag": None if not diag else {
                       "gc": gc_pauses.summary(), "calls": calls.summary(), "sections": dump.summary(), "stack_dump": deadman.summary(),
                       "watchdog": watchdog.summary() if watchdog else None,
                       "graph": graph.summary() if graph else None, "events_dropped": diag_events.dropped},
                   "slot": {"puts": node.slot.puts, "superseded": node.slot.superseded, "takes": node.slot.takes},
                   "input": dict(node.input_gaps.summary(), log=None),
                   "rate_gate": dict(node.gate.stats(), counts=dict(node.gate.counts),
                                     rate_hz=a.rate_hz, min_step_ms=a.rate_min_step_ms,
                                     nominal_fps=a.rate_nominal_fps, rebase_s=a.rate_rebase_s),
                   "gc": dict(gc_info, rss_kb_end=gc_policy.rss_kb()),
                   "fatal": node.fatal,
                   "records": None}
        if node.grid_out is not None:      # B1 (only with --pl-grid; the summary is unchanged otherwise)
            summary["pl_grid"] = {"mode": a.pl_grid, "source": grid_source, "out": dict(node.grid_out.counts),
                                  "layer": dict(grid_layer.counts),
                                  "backend": {k: v for k, v in core.fusion.backend.counts.items()
                                              if str(k).startswith(("grid", "changes_not"))}
                                  if hasattr(core.fusion.backend, "counts") else None}
        if conn is not None and conn.client is not None:
            summary["client_counts"] = dict(conn.client.counts)
            summary["late_answers"] = list(getattr(conn.client, "late_answers", []))
        if input_log is not None:
            summary["input"]["log"] = input_log.close()
        if stats is not None:
            # v7: drain the rows (normal exit, SIGINT and SIGTERM all come here) before the summary
            summary["records"] = stats.close()
        if a.summary_out:
            with open(a.summary_out, "w") as fp:
                json.dump(summary, fp, indent=1, sort_keys=True, default=str)
        if conn is not None:
            conn.close()
        if ar_nano is not None:
            summary_ar = dict(ar_nano.counts)
            ar_nano.close()
            sys.stderr.write("AR_NANO closed %s\n" % json.dumps(summary_ar))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.fatal is not None:
        return 4
    return 0 if node.counts.get("block_refused", 0) == 0 else 3


if __name__ == "__main__":
    sys.exit(main())
