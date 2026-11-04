"""10-26 owner「重成功率，删掉过度防御」(runs/fpga-innov-20261025/defense-audit/REPORT.md): minimal tests of
#3 (car_inner supervision: a non-critical child exiting is restarted, the stack keeps running), #2 (an all-invalid
depth frame still sends its geometry request) and #1 (runner input hiccups drop a frame instead of failing the task).

Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_defense_relax_20261026.py
"""
import json
import os
import subprocess
import tempfile
import textwrap
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMMON = os.path.join(ROOT, 'tools', 'car_run', 'car_common.sh')

SUPERVISE = textwrap.dedent('''\
    source "$COMMON"
    mark() { echo "$*" >> "$O/marks.txt"; }
    fail() { echo "FAIL $1" >> "$O/marks.txt"; for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; exit "${2:-3}"; }
    NAMES=(); PIDS=(); STARTS=(); CMDS=(); OUTS=()
    launch() { local n=$1 out=$2; shift 2; "$@" > "$O/$out" 2>&1 & local p=$!
      NAMES+=("$n"); PIDS+=("$p"); STARTS+=("$(car_pid_start "$p")"); CMDS+=("$(printf '%q ' "$@")"); OUTS+=("$out"); }
    launch hub hub.log sleep 30
    launch runtime runtime.log sleep 30
    ONESHOT=" runtime_info "; declare -A RESTARTS=() RESTART_AT=()
    kill "${PIDS[0]}"; sleep 0.3
    car_supervise_once
    echo "after-exit hub_pid=${PIDS[0]}"
    sleep 1.2
    car_supervise_once
    car_pid_alive "${PIDS[0]}" "${STARTS[0]}" && echo HUB_ALIVE
    car_pid_alive "${PIDS[1]}" "${STARTS[1]}" && echo RUNTIME_ALIVE
    kill "${PIDS[1]}"; sleep 0.3
    car_supervise_once
    echo NOT_REACHED
''')


def supervise(failstop):
    with tempfile.TemporaryDirectory() as o:
        env = dict(os.environ, COMMON=COMMON, O=o, CAR_FAILSTOP=failstop)
        r = subprocess.run(['bash', '-c', SUPERVISE], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=30)
        marks = open(os.path.join(o, 'marks.txt')).read() if os.path.exists(os.path.join(o, 'marks.txt')) else ''
        return r, marks


class SupervisionRestart(unittest.TestCase):
    def test_hub_exit_restarts_stack_keeps_running_critical_still_stops(self):
        r, marks = supervise('0')
        self.assertIn('EXITED hub', marks)
        self.assertIn('RESTART hub n=1 in 1s', marks)
        self.assertIn('RESTARTED hub n=1', marks)
        self.assertIn('HUB_ALIVE', r.stdout)
        self.assertIn('RUNTIME_ALIVE', r.stdout)            # the hub exit did not take the stack down
        self.assertIn('FAIL runtime exited after READY', marks)   # a critical child still stops it
        self.assertEqual(r.returncode, 5)
        self.assertNotIn('NOT_REACHED', r.stdout)

    def test_failstop_1_keeps_the_old_behaviour(self):
        r, marks = supervise('1')
        self.assertIn('FAIL hub exited after READY', marks)
        self.assertEqual(r.returncode, 5)
        self.assertNotIn('RESTART', marks)

    def test_actions_and_backoff(self):
        script = ('source "$COMMON"; for a in "0 hub" "0 bridge" "0 preview" "0 producer" "1 hub" "log producer"; do '
                  'set -- $a; echo "$1/$2=$(car_exit_action "$1" "$2")"; done; '
                  'for n in 1 2 3 4 5 6 7 50; do echo -n "$(car_restart_backoff $n) "; done')
        out = subprocess.run(['bash', '-c', script], env=dict(os.environ, COMMON=COMMON), stdout=subprocess.PIPE,
                             universal_newlines=True, timeout=10).stdout
        for want in ('0/hub=restart', '0/bridge=restart', '0/preview=restart', '0/producer=restart', '1/hub=stop',
                     'log/producer=log'):
            self.assertIn(want, out)
        self.assertIn('1 2 4 8 16 30 30 30 ', out)

    def test_producer_exit_restarts_with_the_same_command_line(self):
        """10-26 2nd pass: the producer is on the restart list; the relaunch is the first launch's argv verbatim
        (golive --rolling-backend / --kv-expect-build-id included), producer.pid follows, the stack keeps running"""
        script = textwrap.dedent('''\
            source "$COMMON"
            mark() { echo "$*" >> "$O/marks.txt"; }
            fail() { echo "FAIL $1" >> "$O/marks.txt"; for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; exit "${2:-3}"; }
            NAMES=(); PIDS=(); STARTS=(); CMDS=(); OUTS=()
            launch() { local n=$1 out=$2; shift 2; "$@" > "$O/$out" 2>&1 & local p=$!
              NAMES+=("$n"); PIDS+=("$p"); STARTS+=("$(car_pid_start "$p")"); CMDS+=("$(printf '%q ' "$@")"); OUTS+=("$out"); }
            launch runtime runtime.log sleep 30
            launch producer producer.log python3 -c 'import sys, time; print("start", *sys.argv[1:], flush=True); time.sleep(30)' --rolling-backend remote_pl \\
              --kv-expect-build-id 0x1234abcd --summary-out "$O/a b.json"
            echo "${PIDS[1]}" > "$O/producer.pid"
            sleep 0.3; tr '\\0' ' ' < "/proc/${PIDS[1]}/cmdline" > "$O/cmd0"
            ONESHOT=" runtime_info "; declare -A RESTARTS=() RESTART_AT=()
            kill "${PIDS[1]}"; sleep 0.3
            car_supervise_once
            sleep 1.2
            car_supervise_once
            sleep 0.3; tr '\\0' ' ' < "/proc/${PIDS[1]}/cmdline" > "$O/cmd1"
            car_pid_alive "${PIDS[1]}" "${STARTS[1]}" && echo PRODUCER_ALIVE
            car_pid_alive "${PIDS[0]}" "${STARTS[0]}" && echo RUNTIME_ALIVE
            [ "$(cat "$O/producer.pid")" = "${PIDS[1]}" ] && echo PIDFILE_FOLLOWS
            car_supervise_once; echo SUPERVISING
            kill "${PIDS[@]}" 2>/dev/null
        ''')
        with tempfile.TemporaryDirectory() as o:
            r = subprocess.run(['bash', '-c', script], env=dict(os.environ, COMMON=COMMON, O=o, CAR_FAILSTOP='0'),
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
            def read(f):
                with open(os.path.join(o, f)) as fp:
                    return fp.read()
            marks, cmd0, cmd1, log = (read(f) for f in ('marks.txt', 'cmd0', 'cmd1', 'producer.log'))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('EXITED producer', marks)
        self.assertIn('RESTART producer n=1 in 1s', marks)
        self.assertIn('RESTARTED producer n=1', marks)
        self.assertNotIn('FAIL', marks)
        for want in ('PRODUCER_ALIVE', 'RUNTIME_ALIVE', 'PIDFILE_FOLLOWS', 'SUPERVISING'):
            self.assertIn(want, r.stdout)
        self.assertEqual(cmd0, cmd1)
        self.assertIn('--rolling-backend remote_pl --kv-expect-build-id 0x1234abcd', cmd1)
        self.assertEqual(log.count('start --rolling-backend remote_pl --kv-expect-build-id 0x1234abcd'), 2, log)


# ---- #2: an all-invalid depth frame still reaches the backend (blank request) -------------------------------------
import numpy as np  # noqa: E402

from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture  # noqa: E402

GRID = GridSpec(origin_m=(-0.1, -0.8, -0.2), voxel_m=0.05, shape=(32, 32, 8))
KM = np.array([[40., 0., 4.], [0., 40., 4.], [0., 0., 1.]])
TM = np.eye(4)
TM[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]


class FakeBackend(object):
    def __init__(self, remote=False, fail=False):
        self.calls, self.fail = [], fail
        if remote:
            self.accepts_capture_identity = True

    def frame_from_depth(self, **kw):
        self.calls.append(kw)
        if self.fail:
            raise RuntimeError("kv timeout")
        return object()     # a blank frame's result is never read (no attribute access on it)


def blank_frame(pipeline, fid, depth):
    t = 1_000_000_000 + fid * 100_000_000
    ident = CaptureIdentity("cam0", fid, t, "mono", "cal-1", "odom", "e1")
    pose = PoseAtCapture(t, "mono", "odom", "e1", True, TM.copy())
    return pipeline.ingest_geometry(DepthFrame(ident, depth, KM, pose), now_ns=t + 1_000_000, now_clock_id="mono")


class BlankDepthRequest(unittest.TestCase):
    def test_all_invalid_depth_still_calls_backend_reason_unchanged(self):
        for depth in (np.zeros((9, 9), np.float32), np.full((9, 9), np.nan, np.float32),
                      np.full((9, 9), 12.0, np.float32)):          # 0 m, no return, beyond 10 m
            backend = FakeBackend()
            pipe = M7Pipeline(backend, GRID)
            out = blank_frame(pipe, 1, depth)
            self.assertFalse(out.navigation.valid)
            self.assertEqual(out.navigation.reason, "no_valid_depth")    # the console prompt's word
            self.assertEqual(len(backend.calls), 1)
            self.assertTrue(np.isnan(backend.calls[0]["depth_m"]).all())  # no ray can be carved
            self.assertEqual(pipe.blank_requests, 1)

    def test_remote_gets_all_zero_codes(self):
        backend = FakeBackend(remote=True)
        pipe = M7Pipeline(backend, GRID)
        out = blank_frame(pipe, 1, np.full((9, 9), 15.0, np.float32))
        self.assertEqual(out.navigation.reason, "no_valid_depth")
        codes = backend.calls[0]["codes"]
        self.assertEqual((codes.dtype, codes.shape, int(codes.max())), (np.dtype(np.uint16), (9, 9), 0))
        self.assertIs(backend.calls[0]["identity"].frame_id, 1)

    def test_backend_failure_on_a_blank_frame_keeps_the_word(self):
        backend = FakeBackend(remote=True, fail=True)
        pipe = M7Pipeline(backend, GRID)
        self.assertEqual(blank_frame(pipe, 1, np.zeros((9, 9), np.float32)).navigation.reason, "no_valid_depth")
        self.assertEqual((pipe.blank_requests, pipe.blank_request_failures), (1, 1))


# ---- #1: a processed-stream gap shorter than --stream-timeout-s no longer ends the navigation --------------------
class RuntimeGap(unittest.TestCase):
    """Same production Runtime harness as test_runner_localization_skip.py (Drive): navigate, then no camera
    image for 3 s. --input-hiccup drop --stream-timeout-s 5 (the car): no fault; 10-29 owner: past 5 s still no
    fault -- a processed_stream_stalled warning, the goal stays live, processed_stream_resumed when images come back;
    the default (fault, 2 s): processed_stream_wall_timeout after ~2 s (negative control)."""

    def drive(self, extra):
        import sys
        from pathlib import Path
        root = Path(ROOT)
        sys.path.insert(0, str(root / "tests"))
        import test_runner_localization_skip as base
        import test_region_source_none_regression as reg
        real = reg.make_loader
        reg.make_loader = lambda source, module, extra_args=None: real(source, module, extra)
        try:
            d = base.Drive(base.current_source(), "rt_gap_%d" % len(extra))
        finally:
            reg.make_loader = real
        self.addCleanup(d.close)
        d.warm_and_navigate()
        d.camera = d.world.image
        d.world.image = lambda stamp: None          # the camera stalls from here on
        return d

    def records(self, d, name):
        return [e["payload"] for e in d.world.events if e["event"] == "publish"
                and e.get("topic") == "/fpga/status" and e["payload"].get("event") == name]

    def stream_faults(self, d):
        return [r for _, r in d.faults() if r == "processed_stream_wall_timeout"]

    def test_three_second_gap_is_not_a_fault_on_the_car(self):
        d = self.drive({"input_hiccup": "drop", "stream_timeout_s": 5.0})
        self.assertEqual(d.node.motion.processed_max_s, 5.0)
        last = d.node.last_processed_wall
        d.run(d.t + 1e-6, d.t + 3.0)
        self.assertEqual(d.faults(), [])
        self.assertTrue(d.node.nav and not d.node.nav["cancelled"])
        d.run(d.t + 1e-6, d.t + 2.6)                  # past 5 s: 10-29 a warning, the goal stays live
        self.assertEqual(d.faults(), [])
        self.assertTrue(d.node.nav and not d.node.nav["cancelled"])
        stalled = self.records(d, "processed_stream_stalled")
        self.assertEqual(len(stalled), 1, stalled)
        self.assertGreater(stalled[0]["gap_s"], 5.0)
        self.assertGreater(d.node.stream_stall_wall - last, 5.0)
        d.run(d.t + 1e-6, d.t + 2.0)
        self.assertEqual(len(self.records(d, "processed_stream_stalled")), 1)      # once per stall
        d.world.image = d.camera                       # the camera is back
        processed = d.node.processed
        d.run(d.t + 1e-6, d.t + 1.5)
        self.assertGreater(d.node.processed, processed)
        resumed = self.records(d, "processed_stream_resumed")
        self.assertEqual(len(resumed), 1, resumed)
        self.assertGreater(resumed[0]["gap_s"], 7.0)
        self.assertIsNone(d.node.stream_stall_wall)
        self.assertEqual(d.faults(), [])
        self.assertTrue(d.node.nav and not d.node.nav["cancelled"])

    def test_default_still_faults_after_two_seconds(self):
        d = self.drive({})
        self.assertEqual(d.node.motion.processed_max_s, 2.0)
        d.run(d.t + 1e-6, d.t + 3.0)
        self.assertTrue(self.stream_faults(d), d.faults())


class TaskInvalidPose(unittest.TestCase):
    def test_invalid_pose_frame_dropped_when_configured(self):
        from host.task_control import PoseGoal, TaskConfig, TaskController
        from types import SimpleNamespace as NS

        class Mem(object):
            def query_memory(self, goal, after=None):
                return []

        for drop, want in ((True, "navigating"), (False, "failed")):
            task = TaskController(Mem(), None, search_waypoints=[PoseGoal(1., 0., 0.)],
                                  config=TaskConfig(drop_invalid_pose_frames=drop))
            task.start("瓶子", 10., NS(frame_id="f1", timestamp=9.9, pose_valid=True))
            self.assertEqual(task.state, "navigating")
            task.observe(NS(frame_id="f2", timestamp=10.1, pose_valid=False), 10.2)
            self.assertEqual(task.state, want, task.reason)


if __name__ == '__main__':
    unittest.main()


# ---- #3a / #2 follow-ups (10-26 2nd pass): the real GeoProducer core on the CPU frame + rolling libraries ---------
def _producer(remote):
    import sys
    import types
    from host.geo_producer import GeoProducer
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_rolling import compile_rolling
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    tmp = tempfile.mkdtemp(prefix="defrelax_prod_")
    sel = M7BackendSelector(cpu_frame_library=compile_frame_native(os.path.join(tmp, "f.so")))
    frame = sel.frame_backend("cpu")
    if remote:
        from test_geo_producer import _RecordingBackend
        frame = _RecordingBackend(frame)
    roll = sel.rolling_backend("cpu", cpu_library=compile_rolling(os.path.join(tmp, "r.so")))
    now = [10 ** 12]
    return GeoProducer(frame, roll, depth_unit_m=0.001, clock=lambda: now[0]), frame, now


def _scene():
    import sys
    from tools.car_geo_producer.common import base_camera, odom_base
    sys.path.insert(0, os.path.join(ROOT, "robot_sim", "tests"))
    import rolling_nav2_smoke as smoke
    depth, _, _ = smoke.render((-0.6, 0.0, 0.0), smoke.WALLS + [smoke.BOX_A])
    codes = np.where(np.isfinite(depth), np.rint(depth / 0.001), 0).astype(np.uint16)
    Toc = odom_base(0) @ base_camera()
    return codes, smoke.intrinsics(), Toc


class ProducerSelfHeal(unittest.TestCase):
    """#3a: with WORKER_ERROR_LIMIT 0 a frame exception no longer ends the node, so a REPEATING one must not
    leave it alive without frames. The known repeating one: the rolling map's sequence out of step with the
    host bookkeeping (RuntimeError on every later frame, before). Now: that frame drops, the next one fuses."""

    def test_sequence_divergence_recovers_on_the_next_frame(self):
        prod, _, now = _producer(remote=False)
        codes, K, Toc = _scene()
        kw = dict(T_odom_camera=Toc, robot_position_m=(0., 0., 0.), calibration_id="c")

        def frame(i):
            now[0] += 100_000_000
            return prod.process(codes, K, depth_stamp_ns=i, acquired_ns=now[0] - 1_000_000, **kw)

        r1 = frame(1)
        self.assertEqual(r1["result"], "fused")
        epoch1 = r1["delta"].epoch
        prod.fusion._delta_seq = 7               # host bookkeeping out of step with the rolling map
        r2 = frame(2)                            # before: RuntimeError here and on every later frame
        self.assertIsNone(r2["delta"])
        self.assertTrue(r2["fusion_reason"].startswith("fusion_error:RuntimeError"), r2["fusion_reason"])
        self.assertEqual(prod.counts["fusion_error_recovered"], 1)
        for i in (3, 4, 5):
            r = frame(i)
            self.assertEqual(r["result"], "fused", r["result"])
        self.assertGreater(r["delta"].epoch, epoch1)   # rebuilt on a fresh epoch (the mirror clears on it)
        self.assertEqual(prod.counts["fusion_error_recovered"], 1)

    def test_worker_loop_keeps_producing_after_a_burst_of_frame_errors(self):
        """run_worker itself (the node's loop): 60 raising frames (old limit 50 would have ended it), then good ones."""
        import threading
        from host.geo_producer import run_worker
        stop, items, done, errors = threading.Event(), list(range(80)), [], []

        def take():
            if not items:
                stop.set()
                return None
            return items.pop(0)

        def handle(i):
            if i < 60:
                raise RuntimeError("rolling map sequence diverged from host bookkeeping")
            done.append(i)

        self.assertIsNone(run_worker(stop, take, handle, on_error=lambda e, s: errors.append(s)))
        self.assertEqual((len(errors), done[0], len(done)), (60, 60, 20))


class BlankFrameNoRoi(unittest.TestCase):
    """#2 follow-up: a blank geometry request carries no ROI, so no verdict reaches the object memory."""

    def test_all_invalid_frame_sends_codes_without_roi(self):
        prod, backend, now = _producer(remote=True)
        codes, K, Toc = _scene()
        mem = {"runtime_boot_id": "b", "revision": 1, "world_frame": "map", "tracks": [
            {"id": "t1", "label": "cup", "bounds_m": [[0.55, -0.25, 0.0], [0.75, 0.05, 0.35]],
             "last_seen": 1.0, "position_m": [0.65, -0.1, 0.17]}]}
        kw = dict(T_odom_camera=Toc, robot_position_m=(0., 0., 0.), calibration_id="c", T_world_camera=Toc,
                  memory_msg=mem)
        now[0] += 100_000_000
        good = prod.process(codes, K, depth_stamp_ns=1, acquired_ns=now[0] - 1_000_000, **kw)
        self.assertIsNotNone(backend.calls[-1]["roi"])             # positive control: ROI rides a normal frame
        self.assertEqual(good["nav_reason"], "geometry_ready")
        now[0] += 100_000_000
        blank = prod.process(np.zeros_like(codes), K, depth_stamp_ns=2, acquired_ns=now[0] - 1_000_000, **kw)
        self.assertEqual(len(backend.calls), 2)                    # the blank request still went out
        self.assertIsNone(backend.calls[-1]["roi"])
        self.assertEqual(int(backend.calls[-1]["codes"].max()), 0)
        self.assertEqual(blank["nav_reason"], "no_valid_depth")
        self.assertEqual(blank["body"]["failure"], "roi_skipped:no_valid_depth")
        self.assertEqual(blank["body"]["verdicts"], [])


# ---- #1: a task goal sent while the input is not ready is a failed attempt (retry), not a fault ------------------
class RuntimeInputNotReadyRetry(unittest.TestCase):
    """Production Runtime.navigate with a real TaskController in ``navigating``: the input is not ready when
    the task's own goal goes out. --input-hiccup drop: navigation_refused(for_retry) + one failed attempt, no
    fault, task still active. Default (fault): fault navigation_input_unhealthy (negative control)."""

    drive = RuntimeGap.drive

    def setup_task(self, extra):
        from types import SimpleNamespace as NS
        from host.task_control import PoseGoal, TaskConfig, TaskController
        d = self.drive(extra)
        d.world.image = lambda stamp: None
        d.node.nav = None                          # the warm-up goal is done; the task sends the next one

        class Mem(object):
            def query_memory(self, goal, after=None):
                return []

        now = d.node.now()
        task = TaskController(Mem(), None, search_waypoints=[PoseGoal(1., 0., 0.), PoseGoal(2., 0., 0.)],
                              config=TaskConfig(drop_invalid_pose_frames=extra.get("input_hiccup") == "drop"))
        task.start("瓶子", now, NS(frame_id="f1", timestamp=now - .05, pose_valid=True))
        self.assertEqual(task.state, "navigating")
        d.node.task = task
        d.node.ready = lambda: False               # the input is not ready at this instant
        return d, task

    def refusals(self, d):
        return [e for e in d.world.events if "navigation_refused" in json.dumps(e, default=str)]

    def test_drop_mode_turns_it_into_a_retry(self):
        from host.task_control import PoseGoal
        d, task = self.setup_task({"input_hiccup": "drop", "stream_timeout_s": 5.0})
        d.node.navigate(PoseGoal(1., 0., 0.), task._active_id)
        self.assertEqual(d.faults(), [])
        self.assertTrue(d.node.active(), (task.state, task.reason))
        # 10-29 owner: held for a resend, not a failed attempt (a long stall no longer burns the failure cap)
        self.assertEqual((task.state, task.reason), ("recovering", "navigation_input_wait"))
        self.assertEqual(task._failures(), 0)
        self.assertEqual(task.attempts, [])
        refused = [e for e in self.refusals(d) if "for_retry" in json.dumps(e, default=str)]
        self.assertTrue(refused, [e.get("event") for e in d.world.events][-20:])

    def test_default_still_faults(self):
        from host.task_control import PoseGoal
        d, task = self.setup_task({})
        d.node.navigate(PoseGoal(1., 0., 0.), task._active_id)
        self.assertIn("navigation_input_unhealthy", [r for _, r in d.faults()])

