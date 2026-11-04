"""Default ``--region-source none`` leaves the runtime's behaviour unchanged (spec D, pkg 2).

The M5 software replay (tools/m5_software_replay.py, 9 scenarios) is run twice: once with
the Runtime class from ``host/ros_runtime.py`` as of 58661b9 (before package 2) and once with
the current file. Every other module is today's, so the comparison isolates the
ros_runtime change. The event streams must be identical row by row after removing what
package 2 adds on purpose: the ``loader`` row (it carries source hashes) and publishes on the
new ``/fpga/memory`` topic; ``sequence`` is dropped because those extra rows shift it.

Python 3.8: the production probe loader needs ``ast.unparse`` (3.9+, D-028) only to feed
``symtable`` with the class text. This loader takes the class text from the source lines
instead (``end_lineno`` exists since 3.8), so the test runs on both interpreters.

Positive controls: (1) one field of one event changed -> the comparator reports it;
(2) a Runtime variant that hands memory updates an empty geometry (the ``kv_only`` branch
taken unconditionally) -> the stream differs in ``occlusion_changes_view``.
"""

import ast
import builtins
import hashlib
import json
from pathlib import Path
import subprocess
import symtable
import sys
import textwrap
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
BASELINE = "58661b9"
SEARCH_MVP_SCENARIO = "scenario_empty_old_position_new_id_selection"
GEOMETRY_CALL = ('{} if self.region_source == "kv_only"\n'
                 '                                    else compute_geometry(frame, self.memory)')


def baseline_source():
    try:
        return subprocess.check_output(["git", "show", "%s:host/ros_runtime.py" % BASELINE], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None


def module_from_source(source, name):
    module = types.ModuleType(name)
    module.__file__ = str(ROOT / "host" / "ros_runtime.py")
    sys.modules[name] = module            # dataclasses resolve their module through sys.modules
    try:
        exec(compile(source, module.__file__, "exec"), module.__dict__)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


def make_loader(source, module, extra_args=None):
    """tools.rtabmap_runtime_probe_support.load_runtime, policy 'original', without ast.unparse."""
    from tools import rtabmap_runtime_probe_support as probe
    from types import SimpleNamespace as NS

    def load_runtime(world, diagnostic_path):
        assert world.policy == "original"
        tree = ast.parse(source)
        main = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main"]
        assert len(main) == 1
        classes = [n for n in main[0].body if isinstance(n, ast.ClassDef) and n.name == "Runtime"]
        assert len(classes) == 1
        klass = classes[0]
        start = min([klass.lineno] + [d.lineno for d in klass.decorator_list])
        text = textwrap.dedent("\n".join(source.splitlines()[start - 1:klass.end_lineno])) + "\n"
        namespace = vars(module).copy()
        namespace.update(probe.bindings(world))
        namespace["args"] = NS(pose_source="estimated", sensor_source="simulation", inference_device="cpu",
                               diagnostics=Path(diagnostic_path), frames_dir=None, events=None,
                               localization_map_id="a" * 64, localization_tracked_frame="camera_link",
                               camera_frame="camera_optical_frame", base_frame="base_link",
                               world_frame="map", uint16_depth_scale=.001, socket="offline-no-socket",
                               **(extra_args or {}))
        namespace["waypoints"] = world.waypoints

        def forbidden_reference_listener(*args, **kwargs):
            raise RuntimeError("offline fixture only supports the estimated-pose branch")
        namespace["TransformListener"] = forbidden_reference_listener
        required = set()

        def visit(table):
            required.update(s.get_name() for s in table.get_symbols() if s.is_global() and s.is_referenced())
            for child in table.get_children():
                visit(child)
        visit(symtable.symtable(text, "Runtime", "exec"))
        missing = required - set(namespace) - set(vars(builtins))
        assert not missing, "unprovided Runtime global dependencies: " + str(sorted(missing))
        exec(compile(text, "Runtime@" + module.__name__, "exec"), namespace)
        world.emit("loader", production_sha256=hashlib.sha256(source.encode()).hexdigest())
        node = namespace["Runtime"]()
        assert isinstance(node.memory, probe.Memory)
        return node

    return load_runtime


def capture(source, module_name, extra_args=None):
    """Run every M5 scenario with the given ros_runtime source; return {name: events}."""
    import tools.m5_software_replay as replay
    module = module_from_source(source, module_name)
    streams, passed = {}, {}
    original_scenarios = replay.SCENARIOS

    def recording(function):
        def run():
            try:
                return function()
            finally:          # also when the scenario's own check fails; same list, so close() rows count
                world = getattr(replay.Replay.last_created, "world", None)
                if world is not None:
                    streams[function.__name__] = world.events
        run.__name__ = function.__name__
        return run

    try:
        with patch.object(replay, "load_runtime", make_loader(source, module, extra_args)), \
                patch.object(replay, "SCENARIOS", tuple(recording(f) for f in original_scenarios)):
            for row in replay.run_scenarios():
                passed[row["name"]] = (row["passed"], row["error"])
    finally:
        sys.modules.pop(module_name, None)
    return streams, passed


def normalise(events):
    from test_car_geometry_kv import retry_normalise    # 10-25 find-retry rows/keys (see there)
    out = []
    for row in events:
        row = retry_normalise(row)
        if row is None or row.get("event") == "loader":
            continue
        if row.get("event") == "publish" and row.get("topic") in ("/fpga/memory", "/fpga/runtime_info"):
            continue        # /fpga/runtime_info: latched start-up facts (integration v5, test_hub_v5_fixes)
        payload = row.get("payload") or {}
        if (row.get("event") == "publish" and row.get("topic") == "/fpga/status"
                and payload.get("event") == "geometry_close" and payload.get("source") == "cpu"):
            continue
        row = dict(row)
        row.pop("sequence", None)
        out.append(json.dumps(row, sort_keys=True, ensure_ascii=False, default=repr))
    return out


def differences(before, after):
    diffs = []
    for name in sorted(set(before) | set(after)):
        a, b = normalise(before.get(name, [])), normalise(after.get(name, []))
        if a != b:
            k = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), min(len(a), len(b)))
            diffs.append((name, len(a), len(b), k))
    return diffs


@unittest.skipIf(baseline_source() is None, "git history with %s not available" % BASELINE)
class RegionSourceNoneRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_source = baseline_source()
        cls.new_source = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        cls.before, cls.before_ok = capture(cls.old_source, "ros_runtime_baseline_58661b9")
        cls.after, cls.after_ok = capture(cls.new_source, "ros_runtime_current")

    def test_both_arms_run_every_scenario(self):
        import tools.m5_software_replay as replay
        self.assertEqual(len(self.before), len(replay.SCENARIOS))
        self.assertEqual(set(self.before), set(self.after))
        self.assertTrue(all(ok for ok, _ in self.before_ok.values()), self.before_ok)
        self.assertTrue(all(ok for ok, _ in self.after_ok.values()), self.after_ok)

    def test_event_streams_identical_without_memory_topic(self):
        # 0480eb5 intentionally changed the moved-target search to auto-accept
        # one fresh candidate. Keep exact event comparison for the other eight
        # scenarios, and check this declared difference explicitly below.
        self.assertEqual([d[0] for d in differences(self.before, self.after)], [SEARCH_MVP_SCENARIO])
        before = {k: v for k, v in self.before.items() if k != SEARCH_MVP_SCENARIO}
        after = {k: v for k, v in self.after.items() if k != SEARCH_MVP_SCENARIO}
        self.assertEqual(differences(before, after), [])
        def last_task(stream):
            return [r["payload"] for r in stream if r.get("topic") == "/fpga/status" and
                    r.get("payload", {}).get("event") == "task"][-1]
        old_task = last_task(self.before[SEARCH_MVP_SCENARIO])
        new_task = last_task(self.after[SEARCH_MVP_SCENARIO])
        self.assertEqual((old_task["state"], old_task["reason"]), ("found", "search_single_candidate"))
        self.assertEqual((new_task["state"], new_task["reason"]), ("found", "search_single_candidate"))
        self.assertEqual(new_task["target_id"], old_task["target_id"])
        n = sum(len(normalise(v)) for v in before.values())
        added = sum(1 for v in self.after.values() for r in v
                    if r.get("event") == "publish" and r.get("topic") == "/fpga/memory")
        sys.stderr.write("\n[none-regression] %d scenarios, %d events compared, %d /fpga/memory publishes "
                         "removed (plus one declared search-MVP delta)\n" % (len(before), n, added))
        self.assertGreater(n, 300)
        self.assertGreater(added, 0)            # the new topic is really there, and only it was removed
        self.assertFalse(any(r.get("topic") == "/fpga/memory" for v in self.before.values() for r in v))

    def test_positive_control_one_changed_event_field_is_caught(self):
        name = sorted(self.after)[0]
        mutated = dict(self.after)
        events = [dict(r) for r in mutated[name]]
        k = next(i for i, r in enumerate(events) if r.get("event") not in ("loader",)
                 and r.get("topic") != "/fpga/memory")
        events[k]["wall"] = events[k]["wall"] + 1e-9
        mutated[name] = events
        self.assertEqual([d[0] for d in differences(self.before, mutated)], sorted([name, SEARCH_MVP_SCENARIO]))

    def test_positive_control_geometry_dropped_variant_differs(self):
        self.assertIn(GEOMETRY_CALL, self.new_source)
        variant = self.new_source.replace(GEOMETRY_CALL, "{}")
        streams, _ = capture(variant, "ros_runtime_variant_no_geometry")
        names = [d[0] for d in differences(self.before, streams)]
        self.assertIn("scenario_occlusion_changes_view", names)


if __name__ == "__main__":
    unittest.main()
