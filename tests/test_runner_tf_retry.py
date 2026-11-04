"""Runner: a frame waiting for its TF is retried when a new /tf sample arrives (--tf-retry, v6).

Docs: docs/design/INTEG_CAR_CHAIN_20261011.md "第 6 版"; RUNNER_AGE_GATE_20261011.md sec 13.2/13.8
(the "wait for the next image / 100 ms poll" slow frames: TF only ~1 ms behind the image, the
retry came 60-90 ms later).

The production Runtime class (reference-pose branch, the one with a tf2 listener) runs on a
scripted world: wall / ROS clocks set by the test, a fake listener whose ``tf_sub.callback`` is
what rclpy's executor would call per /tf message, and a buffer with tf2's rule for a dynamic
edge -- a lookup at stamp t needs a real sample <= t AND a real sample >= t (linear interpolation
between them), never extrapolation. Every image pair, /tf message and 100 ms poll is delivered
in time order, as the single-threaded executor would.

1. Latency (positive control): images at 10 Hz, the sample covering each image arrives 1 ms
   after it, polls at phase +70 ms. New (default arrival): every frame prepared 1 ms after its
   image. Old runner (4a247b6, before v6) and the new runner with --tf-retry poll: every frame
   waits >= 60 ms (until the next poll).
2. No extrapolation: samples older than the image arrive and trigger retries that still fail;
   every lookup is at the image's exact stamp; the frame is built only when a sample >= its stamp
   arrives, with the pose interpolated between the two real samples around it.
3. Bounded wait unchanged: without a covering sample the frame is dropped after 0.5 s (same
   warning); a covering sample arriving after the bound does not revive it (dropped, not used),
   with or without a poll in between.
4. Only TF waits are retried on /tf: a frame waiting for its ROS clock (AcquisitionPending) is
   not; an empty queue is not; --tf-retry poll installs no hook.
5. Wiring: one TransformListener, built without spin_thread (its callbacks stay on the runner's
   single-threaded executor), hooked once; the stall diagnostics' own tf_sub wrapper composes
   with the hook (diag rows name who prepared the frame: tf).
6. ROS image only (skipped without rclpy): with the real tf2_ros listener and a real /tf
   publisher, the hook runs on the spinning thread after the sample is in the buffer (a lookup at
   the published stamp succeeds inside the hook).
Python 3.8+.
"""

import ast
import builtins
from pathlib import Path
import subprocess
import symtable
import sys
import tempfile
import textwrap
import threading
import time
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

NS = types.SimpleNamespace
OLD = "4a247b6"
MS = 1000000
BASE = 5 * 10 ** 9          # ROS ns at wall 0


def old_source():
    try:
        return subprocess.check_output(["git", "show", "%s:host/ros_runtime.py" % OLD], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None


def current_source():
    return (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")


def tf_msg(stamp_ns, x):
    from tools.rtabmap_runtime_probe_support import Time, transform
    t = transform("map", "base_link", (x, 0., 0.))
    t.header.stamp = Time(stamp_ns).to_msg()
    return NS(transforms=[t])


def make_buffer(world, probe, production):
    class DynBuffer:
        """Static edges any time; the dynamic map->base_link edge only between two real samples."""

        def __init__(self):
            self.samples = []               # sorted (stamp_ns, x)
            self.lookups = []

        def set_transform(self, t, authority):
            stamp = t.header.stamp.sec * 10 ** 9 + t.header.stamp.nanosec
            self.samples.append((stamp, t.transform.translation.x))
            self.samples.sort()

        def lookup_transform(self, target, source, stamp):
            ns = stamp.nanoseconds
            self.lookups.append((target, source, ns, world.wall))
            if ns <= 0:
                raise probe.TransformException("zero/latest lookup forbidden")
            if (target, source) == ("base_link", "camera_optical_frame"):
                out = probe.transform("base_link", "camera_optical_frame", (0., 0., 0.))
            elif (target, source) in (("map", "camera_optical_frame"), ("map", "base_link")):
                before = [s for s in self.samples if s[0] <= ns]
                after = [s for s in self.samples if s[0] >= ns]
                if not before or not after:
                    raise probe.TransformException("Lookup would require extrapolation (stamp %d, samples %s)"
                                                   % (ns, [s[0] for s in self.samples[-3:]]))
                (s0, x0), (s1, x1) = before[-1], after[0]
                x = x0 if s1 == s0 else x0 + (x1 - x0) * (ns - s0) / (s1 - s0)
                out = probe.transform(target, source, (x, 0., 0.))
            else:
                raise probe.TransformException("unknown edge")
            out.header.stamp = probe.Time(ns).to_msg()
            return out
    return DynBuffer


class FakeListener:
    made = []

    def __init__(self, buffer, node, **kw):
        self.kw = kw
        self.buffer = buffer
        self.tf_sub = NS(topic="/tf", callback=self.callback)
        FakeListener.made.append(self)

    def callback(self, msg):                # tf2_ros TransformListener.callback: into the buffer
        for t in msg.transforms:
            self.buffer.set_transform(t, "default_authority")


def load_runtime(source, name, tf_retry=None, stall_diag=None):
    """Runtime class of ``source`` on a scripted world, reference poses, kv-free cpu geometry."""
    from tools import rtabmap_runtime_probe_support as probe
    import test_region_source_none_regression as reg
    module = reg.module_from_source(source, name)
    world = probe.World("original")
    tree = ast.parse(source)
    main = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main"][0]
    klass = [n for n in main.body if isinstance(n, ast.ClassDef) and n.name == "Runtime"][0]
    text = textwrap.dedent("\n".join(source.splitlines()[klass.lineno - 1:klass.end_lineno])) + "\n"
    ns = vars(module).copy()
    ns.update(probe.bindings(world))
    base_node = ns["Node"]

    class Node(base_node):
        def get_logger(self):
            return NS(warning=lambda m: world.emit("warning", message=m),
                      info=lambda m: world.emit("info", message=m))
    ns["Node"] = Node
    ns["Buffer"] = make_buffer(world, probe, module)
    ns["TransformListener"] = FakeListener
    ns["runner_tf_qos"] = lambda reliable=False, qos=None: "qos(reliable=%s)" % reliable
    extra = {} if tf_retry is None else {"tf_retry": tf_retry}
    ns["args"] = NS(pose_source="reference", sensor_source="replay", inference_device="cpu",
                    diagnostics=None, frames_dir=None, events=None, localization_map_id=None,
                    localization_tracked_frame="camera_link", camera_frame="camera_optical_frame",
                    base_frame="base_link", world_frame="map", uint16_depth_scale=.001,
                    socket="offline-no-socket", tf_reliable=False, stall_diag=stall_diag, **extra)
    ns["waypoints"] = []
    required = set()

    def visit(table):
        required.update(s.get_name() for s in table.get_symbols() if s.is_global() and s.is_referenced())
        for child in table.get_children():
            visit(child)
    visit(symtable.symtable(text, "Runtime", "exec"))
    missing = required - set(ns) - set(vars(builtins))
    assert not missing, "unprovided Runtime globals: %s" % sorted(missing)
    world.clock(0., BASE)
    exec(compile(text, "Runtime@" + name, "exec"), ns)
    FakeListener.made = []
    node = ns["Runtime"]()
    sys.modules.pop(name, None)
    node.prepared = []                      # (stamp_ns, wall) of every frame prepare_images built
    node.prepare_calls = []                 # (trigger, wall) -- only for sources that take a trigger
    orig_submit = node.submit

    def submit():
        if node.pending is not None:
            node.prepared.append((int(round(node.pending.timestamp * 1e9)), world.wall, node.pending))
        return orig_submit()
    node.submit = submit
    return node, world


def run(node, world, schedule):
    """schedule: [(wall_s, kind, arg)] with kind image (stamp ns) | tf ((stamp ns, x)) | poll."""
    from tools.rtabmap_runtime_probe_support import images
    for wall, kind, arg in sorted(schedule, key=lambda e: (e[0], {"tf": 0, "image": 1, "poll": 2}[e[1]])):
        world.clock(wall, BASE + int(round(wall * 1e9)))
        if kind == "image":
            rgb, depth, info = images(arg)
            node.on_info(info)
            node.on_images(rgb, depth)
        elif kind == "tf":
            node.listener.tf_sub.callback(tf_msg(*arg))     # what rclpy's executor calls per /tf message
        else:
            node.poll()


def periodic(n=10, tf_late_ms=1.0, poll_phase_ms=70.0):
    """10 Hz images stamped at arrival; 60 Hz /tf (every 6th sample = an image stamp) each arriving
    tf_late_ms after its stamp; a 100 ms steady poll at poll_phase_ms."""
    s = []
    for k in range(n):
        s.append((k * .1, "image", BASE + k * 100 * MS))
        s.append((k * .1 + poll_phase_ms / 1e3, "poll", None))
    for j in range(-6, 6 * n + 6):
        stamp = BASE + int(round(j * 100 * MS / 6))
        s.append(((stamp - BASE) / 1e9 + tf_late_ms / 1e3, "tf", (stamp, j * .01)))
    return [e for e in s if e[0] >= 0]


def waits_ms(node, schedule):
    arrive = {arg: wall for wall, kind, arg in schedule if kind == "image"}
    return {stamp: round((wall - arrive[stamp]) * 1e3, 3) for stamp, wall, _ in node.prepared}


class Latency(unittest.TestCase):
    def test_new_prepares_on_arrival(self):
        sched = periodic()
        node, world = load_runtime(current_source(), "tfr_new")
        self.assertEqual(node.tf_retry, "arrival")
        run(node, world, sched)
        w = waits_ms(node, sched)
        self.assertEqual(len(w), 10, w)
        self.assertTrue(all(v == 1.0 for v in w.values()), w)
        # node.dropped counts the fixture's never-finishing detector superseding pending frames (8);
        # no acquisition was dropped after the bounded wait:
        self.assertEqual([e for e in world.events if e["event"] == "warning"], [])
        self.assertEqual(node.runtime_info["tf_retry"], "arrival")
        self.assertIn("TF wait retry: arrival", [e.get("message") for e in world.events if e["event"] == "info"])

    def assert_waits_next_poll(self, label, src, mode):
        sched = periodic()
        node, world = load_runtime(src, "tfr_" + label, tf_retry=mode)
        run(node, world, sched)
        w = waits_ms(node, sched)
        self.assertEqual(len(w), 10, (label, w))
        self.assertTrue(all(v >= 60.0 for v in w.values()), (label, w))
        self.assertTrue(all(v == 70.0 for v in w.values()), (label, w))   # the next poll

    def test_positive_control_poll_mode_waits(self):
        self.assert_waits_next_poll("poll_mode", current_source(), "poll")

    def test_positive_control_old_runner_waits(self):
        old = old_source()
        if old is None:
            self.skipTest("old-runner control needs git history (%s)" % OLD)
        self.assert_waits_next_poll("old_" + OLD, old, None)

    def test_old_source_really_has_no_retry_hook(self):
        old = old_source()
        if old is None:
            self.skipTest("needs git history")
        self.assertNotIn("hook_tf_arrival", old)
        self.assertIn("hook_tf_arrival(self.listener, self.on_tf_arrival)", current_source())


class NoExtrapolation(unittest.TestCase):
    def test_older_samples_do_not_complete_and_lookup_is_exact(self):
        T = BASE + 200 * MS
        sched = [(0., "tf", (BASE, 0.)), (.2, "image", T)]
        sched += [(.2 + i * .004, "tf", (T - (10 - i) * MS, 1. + i)) for i in range(1, 10)]   # all < T
        sched += [(.25, "tf", (T + 16 * MS, 5.))]
        node, world = load_runtime(current_source(), "tfr_noextra")
        calls = []
        orig = node.prepare_images

        def spy(trigger="poll"):
            calls.append((trigger, world.wall))
            return orig(trigger)
        node.prepare_images = spy
        run(node, world, sched)
        self.assertEqual(len(node.prepared), 1)
        stamp, wall, frame = node.prepared[0]
        self.assertEqual((stamp, wall), (T, .25))          # only once a sample >= T is in
        self.assertEqual(sum(1 for t, _ in calls if t == "tf"), 10)   # each sample retried it
        dyn = [l for l in node.buffer.lookups if l[0] == "map"]
        self.assertTrue(dyn and all(l[2] == T for l in dyn), dyn[:3])  # never latest / zero / another stamp
        # interpolated between the two real samples around T: (T-1 ms, x=10) and (T+16 ms, x=5)
        self.assertAlmostEqual(frame.T_world_camera[0, 3], 10. + (5. - 10.) * 1 / 17, places=9)


class BoundedWait(unittest.TestCase):
    def lagging(self, upto):
        T = BASE + 100 * MS
        s = [(0., "tf", (BASE, 0.)), (.1, "image", T)]
        s += [(.1 + i * .016, "tf", (T - MS, 1.)) for i in range(1, int(upto / .016) + 1)]
        return T, s

    def test_dropped_after_bound_without_covering_sample(self):
        for mode in ("arrival", "poll"):
            T, s = self.lagging(.62)
            s += [(.1 + k * .1 + .07, "poll", None) for k in range(7)]
            node, world = load_runtime(current_source(), "tfr_drop_" + mode, tf_retry=mode)
            run(node, world, s)
            self.assertEqual(node.prepared, [], mode)
            warn = [e for e in world.events if e["event"] == "warning"]
            self.assertEqual([e["message"] for e in warn], ["Dropped acquisition after bounded wait for clock/TF"], mode)
            self.assertIsNone(node.acquisitions.current, mode)
            self.assertEqual(node.dropped, 1, mode)

    def test_covering_sample_after_bound_does_not_revive(self):
        # no poll at all: the tf-triggered try itself finds the bound passed and drops the frame
        T, s = self.lagging(.48)
        s += [(.61, "tf", (T + 5 * MS, 2.))]
        node, world = load_runtime(current_source(), "tfr_late_cover")
        run(node, world, s)
        self.assertEqual(node.prepared, [])
        self.assertEqual(node.dropped, 1)
        self.assertEqual([e["message"] for e in world.events if e["event"] == "warning"],
                         ["Dropped acquisition after bounded wait for clock/TF"])
        # with a poll after the bound and before the sample: dropped by the poll, sample ignored
        T, s = self.lagging(.48)
        s += [(.605, "poll", None), (.61, "tf", (T + 5 * MS, 2.))]
        node, world = load_runtime(current_source(), "tfr_late_cover_poll")
        run(node, world, s)
        self.assertEqual((node.prepared, node.dropped), ([], 1))


class OnlyTfWaits(unittest.TestCase):
    def spy(self, node, world):
        calls = []
        orig = node.prepare_images

        def spy(trigger="poll"):
            calls.append((trigger, world.wall))
            return orig(trigger)
        node.prepare_images = spy
        return calls

    def test_clock_wait_not_retried_on_tf(self):
        node, world = load_runtime(current_source(), "tfr_clock")
        calls = self.spy(node, world)
        from tools.rtabmap_runtime_probe_support import images
        T = BASE + 300 * MS
        world.clock(.1, BASE + 100 * MS)            # ROS clock behind the image stamp
        rgb, depth, info = images(T)
        node.on_info(info)
        node.on_images(rgb, depth)                  # AcquisitionPending (clock)
        self.assertIsNotNone(node.acquisitions.current)
        self.assertIsNone(node.tf_wait_entry)
        for i in range(5):
            world.clock(.1 + (i + 1) * .01, BASE + 100 * MS)
            node.listener.tf_sub.callback(tf_msg(T + i * MS, 1.))
        self.assertEqual([c[0] for c in calls], ["image"])
        self.assertEqual(node.prepared, [])

    def test_empty_queue_and_poll_mode(self):
        node, world = load_runtime(current_source(), "tfr_empty")
        calls = self.spy(node, world)
        node.listener.tf_sub.callback(tf_msg(BASE, 0.))
        self.assertEqual(calls, [])
        node, world = load_runtime(current_source(), "tfr_pollmode", tf_retry="poll")
        self.assertEqual(node.tf_retry, "poll")
        self.assertEqual(node.listener.tf_sub.callback, node.listener.callback)    # nothing hooked
        self.assertEqual(node.runtime_info["tf_retry"], "poll")


class Wiring(unittest.TestCase):
    SRC = current_source()

    def test_one_listener_no_spin_thread_single_hook(self):
        self.assertEqual(self.SRC.count("TransformListener("), 1)
        line = [ln for ln in self.SRC.splitlines() if "TransformListener(self.buffer" in ln][0]
        self.assertNotIn("spin_thread", line)
        self.assertEqual(self.SRC.count("hook_tf_arrival(self.listener"), 1)
        self.assertEqual(self.SRC.count("create_subscription(TFMessage"), 1)     # only the estimated branch's /tf_static
        self.assertIn("rclpy.spin(node)", self.SRC)                                 # single-threaded executor

    def test_hook_helper(self):
        from host.ros_runtime import hook_tf_arrival
        order = []
        sub = NS(callback=lambda m: order.append(("inner", m)) or "r")
        self.assertTrue(hook_tf_arrival(NS(tf_sub=sub), lambda: order.append(("after",))))
        self.assertEqual(sub.callback("m"), "r")
        self.assertEqual(order, [("inner", "m"), ("after",)])
        self.assertFalse(hook_tf_arrival(NS(), lambda: None))
        self.assertFalse(hook_tf_arrival(NS(tf_sub=NS(callback=None)), lambda: None))

    def test_stall_diag_wrapper_composes(self):
        d = Path(tempfile.mkdtemp(prefix="tfr_diag_")) / "diag.jsonl"
        node, world = load_runtime(current_source(), "tfr_diag", stall_diag=d)
        try:
            sched = periodic(n=3)
            run(node, world, sched)
            self.assertEqual(sorted(waits_ms(node, sched).values()), [1.0, 1.0, 1.0])
            self.assertGreater(node.sdiag.counts["tf_messages"], 10)               # the diag wrapper ran too
            by = [f.stall_diag["acq_prepared_by"] for _, _, f in node.prepared]
            self.assertEqual(by, ["tf", "tf", "tf"])
            self.assertEqual(node.prepared[0][2].stall_diag["acq_tries_by"], {"image": 1})
        finally:
            node.sdiag.close()


def have_rclpy():
    try:
        import rclpy  # noqa: F401
        import tf2_ros  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(have_rclpy(), "needs rclpy + tf2_ros (ROS image fpga-dev1-sim:humble)")
class LiveTf2(unittest.TestCase):
    def test_hook_runs_on_spin_thread_after_buffer_insert(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.time import Time
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener
        from host.ros_runtime import hook_tf_arrival, runner_tf_qos
        rclpy.init(args=["--ros-args", "-r", "__ns:=/tfr%d" % (time.time_ns() % 100000)])
        try:
            node = rclpy.create_node("tfr_listener")
            pub = rclpy.create_node("tfr_pub")
            buf = Buffer()
            listener = TransformListener(buf, node, qos=runner_tf_qos(False))
            seen = []
            stamps = [Time(seconds=100 + i * .05).to_msg() for i in range(5)]

            def after():
                ok = []
                for st in stamps:
                    try:
                        buf.lookup_transform("map", "base_link", Time.from_msg(st))
                        ok.append(True)
                    except Exception:
                        ok.append(False)
                seen.append((threading.get_ident(), ok))
            self.assertTrue(hook_tf_arrival(listener, after))
            br = TransformBroadcaster(pub)
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            ex.add_node(pub)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and not (node.count_publishers("/tf") and pub.count_subscribers("/tf")):
                ex.spin_once(timeout_sec=.05)
            for i, st in enumerate(stamps):
                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = st, "map", "base_link"
                t.transform.rotation.w = 1.
                t.transform.translation.x = float(i)
                br.sendTransform(t)
                end = time.monotonic() + 1
                while time.monotonic() < end and len(seen) <= i:
                    ex.spin_once(timeout_sec=.05)
            self.assertGreaterEqual(len(seen), 5, seen)
            main = threading.get_ident()
            self.assertTrue(all(tid == main for tid, _ in seen))
            # inside the hook for message i the buffer already brackets stamps[0..i] (exact hit
            # for the newest), never a later one (no extrapolation)
            for i, (_, ok) in enumerate(seen[:5]):
                self.assertEqual(ok, [True] * (i + 1) + [False] * (4 - i), (i, ok))
            ex.shutdown()
            node.destroy_node()
            pub.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
