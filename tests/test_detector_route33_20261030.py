"""10-30 (owner): combined33 split switch CAR_YOLO_ROUTE / --yolo-route.

all33 (default): every enabled class the 33-class model knows goes to it; the COCO model only gets the rest and,
for the six demo items, gets nothing and is not built. primary: the pre-10-30 PRIMARY33 split (rollback).
Known answers per layer: build_detector's per-model classes and score floors, the ready line, the health check,
car_up's argv / refusal, the versioned runtime_detector.env default.
"""
import json
import os
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

from host import inference_service as svc   # noqa: E402
from perception.fpga_replay import yolo_detector as yd   # noqa: E402

SIX = ("backpack", "bottle", "cup", "banana", "laptop")   # owner 10-31: medicine box dropped (name kept)


class FakeYolo(object):
    """Stands in for YoloSegDetector: records what each model was built with (no model file needed)."""
    built = []

    def __init__(self, model, backend, conf, iou, classes, threads, class_min_scores, profile):
        self.model, self.classes, self.class_min_scores, self.profile = model, classes, class_min_scores, profile
        self.execution = {"device": "cpu"}
        FakeYolo.built.append(self)


def args(**kw):
    a = dict(detector="yolo", onnx="/m/h33.onnx", engine=None, pt=None, device="cpu", classes=None,
             exclude_classes=None, yolo_profile="combined33", class_min_score=None, coco_model="/m/coco.onnx",
             conf=0.25, iou=0.7, threads=1, yolo_route=None)
    a.update(kw)
    return SimpleNamespace(**a)


def build(environ=None, **kw):
    FakeYolo.built = []
    with mock.patch.object(yd, "YoloSegDetector", FakeYolo), \
            mock.patch.dict(os.environ, environ or {}, clear=False):
        if environ is None:
            os.environ.pop(svc.YOLO_ROUTE_ENV, None)
        det = svc.build_detector(args(**kw))
    return det, {d.profile: d for d in FakeYolo.built}


class RouteSplit(unittest.TestCase):
    def test_default_classes_are_the_six(self):
        self.assertEqual(yd.parse_classes(None, yd.DEFAULT_EXCLUDE_CLASSES, "combined33"), SIX)

    def test_all33_default_everything_on_33_coco_not_built(self):
        det, by = build()                                     # no env, no flag -> all33
        self.assertEqual(list(by), ["household33"])            # COCO never constructed
        self.assertEqual(by["household33"].classes, SIX)
        self.assertEqual(det.yolo_route, "all33")
        self.assertEqual(det.assignment, {"household33": list(SIX), "coco-seg": []})
        self.assertEqual(svc.served_classes(det), list(SIX))

    def test_primary_restores_old_split(self):
        for how in (dict(yolo_route="primary"), dict(environ={"CAR_YOLO_ROUTE": "primary"})):
            det, by = build(**how)
            self.assertNotIn("household33", by)                # medicine box gone: nothing left for it
            self.assertEqual(by["coco-seg"].classes, ("backpack", "bottle", "cup", "banana", "laptop"))
            self.assertEqual(det.yolo_route, "primary")
            self.assertEqual(sorted(svc.served_classes(det)), sorted(SIX))

    def test_env_and_flag(self):
        self.assertEqual(build(environ={"CAR_YOLO_ROUTE": "all33"})[0].yolo_route, "all33")
        self.assertEqual(build(environ={"CAR_YOLO_ROUTE": ""})[0].yolo_route, "all33")   # empty = default
        # the flag wins over the environment
        self.assertEqual(build(environ={"CAR_YOLO_ROUTE": "primary"}, yolo_route="all33")[0].yolo_route, "all33")

    def test_bad_value_refused_not_silently_default(self):
        for how in (dict(environ={"CAR_YOLO_ROUTE": "All33"}), dict(environ={"CAR_YOLO_ROUTE": "coco"}),
                    dict(yolo_route="nope")):
            with self.assertRaisesRegex(ValueError, "all33\\|primary"):
                build(**how)
        r = subprocess.run([sys.executable, "-m", "host.inference_service", "--socket", "/nonexistent/x.sock",
                            "--device", "cpu", "--detector", "yolo", "--onnx", "/m/h.onnx", "--yolo-profile",
                            "combined33", "--coco-model", "/m/c.onnx"], cwd=ROOT,
                           env=dict(os.environ, PYTHONPATH=ROOT, CAR_YOLO_ROUTE="bogus"),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 2, r.stderr)            # parser.error -> exit 2, before any model loads
        self.assertIn("CAR_YOLO_ROUTE must be one of all33|primary", r.stderr)

    def test_route_none_is_the_old_combined_rule(self):
        # profile 'combined' (household9) passes route None: each model keeps its own enabled classes, as before.
        # (building that profile end to end fails at 4eb1da4 already: a known baseline failure, not touched here)
        from perception.fpga_replay.household_names import NAMES
        self.assertEqual(svc.route_classes("household9", NAMES, ("bottle", "key"), None), ("key",))
        self.assertEqual(svc.route_classes("coco-seg", yd.coco_names(), ("bottle", "key"), None), ("bottle",))
        self.assertIsNone(svc.route_classes("coco-seg", yd.coco_names(), None, None))

    def test_unknown_class_still_goes_to_coco_under_all33(self):
        # 'cell phone' is in NAMES33 (household ids); 'tv' is COCO-only -> the COCO model is built for it alone
        det, by = build(classes="bottle,tv")
        self.assertEqual(by["household33"].classes, ("bottle",))
        self.assertEqual(by["coco-seg"].classes, ("tv",))
        self.assertEqual(det.assignment, {"household33": ["bottle"], "coco-seg": ["tv"]})


class BackpackFloor(unittest.TestCase):
    def test_default_floor_on_33_model_under_all33(self):
        _, by = build()
        self.assertEqual(by["household33"].class_min_scores, {"backpack": 0.35})   # owner 10-30
        self.assertEqual(yd.DEFAULT_CLASS_MIN_SCORES, {"backpack": 0.45})   # COCO default unchanged

    def test_default_floor_on_coco_under_primary(self):
        _, by = build(yolo_route="primary")
        self.assertEqual(by["coco-seg"].class_min_scores, {"backpack": 0.45})
        self.assertNotIn("household33", by)   # 10-31: medicine box dropped, primary leaves the 33-class model nothing

    def test_override_and_none(self):
        _, by = build(class_min_score="backpack=0.45")
        self.assertEqual(by["household33"].class_min_scores, {"backpack": 0.45})
        _, by = build(class_min_score="")
        self.assertEqual(by["household33"].class_min_scores, {})


class ReadyAndHealth(unittest.TestCase):
    def ready_extra(self, det):
        """main()'s ready-line extras for this detector (same code path, without serving)."""
        sent = {}
        with mock.patch.object(svc, "build_detector", return_value=det), \
                mock.patch.object(svc, "serve", side_effect=lambda s, d, dev, ready_extra=None: sent.update(ready_extra)), \
                mock.patch.object(sys, "argv", ["x", "--socket", "/x", "--device", "cpu", "--detector", "yolo",
                                                "--onnx", "/m/h.onnx", "--yolo-profile", "combined33"]):
            svc.main()
        return sent

    def test_ready_line_names_route_and_assignment(self):
        det, _ = build()
        extra = self.ready_extra(det)
        self.assertEqual(extra["yolo_route"], "all33")
        self.assertEqual(extra["assignment"], {"household33": list(SIX), "coco-seg": []})
        self.assertEqual(extra["classes"], list(SIX))
        det, _ = build(yolo_route="primary")
        self.assertEqual(self.ready_extra(det)["assignment"]["household33"], [])

    def test_health_green_without_coco_model(self):
        import test_health_20261029 as th
        det, _ = build()
        extra = self.ready_extra(det)

        class T(th.Base):
            def runTest(self):
                pass
        t = T()
        t.setUp()
        try:
            t.write("inference.log", json.dumps(dict({"ready": True, "socket": "/x", "device": "cuda"}, **extra)) + "\n")
            t.ready()
            c = t.check(t.steps(3), "detector")
            self.assertEqual(c["level"], "green", c)
            self.assertIn("5 类", c["detail"])
        finally:
            t.doCleanups()


class LaunchChain(unittest.TestCase):
    def dry(self, **env):
        e = dict(os.environ, YOLO_PROFILE="combined33", YOLO_COCO_MODEL="/models/coco.onnx", CAR_ARCH="x86_64")
        e.pop("CAR_YOLO_ROUTE", None)
        e.update(env)
        return subprocess.run(["bash", "tools/car_run/car_up.sh", "sim", "--dry-run"],
                              cwd=ROOT, env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True, timeout=300)

    def test_car_up_passes_route(self):
        r = self.dry(CAR_YOLO_ROUTE="primary")
        self.assertEqual(r.returncode, 0, r.stdout[-2000:] + r.stderr[-2000:])
        self.assertIn("--yolo-route primary", r.stdout)
        r = self.dry()
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])
        self.assertNotIn("--yolo-route", r.stdout)            # unset: inference_service default all33

    def test_car_up_refuses_bad_route(self):
        r = self.dry(CAR_YOLO_ROUTE="all-33")
        self.assertEqual(r.returncode, 2, r.stdout[-1000:] + r.stderr[-1000:])
        self.assertIn("CAR_YOLO_ROUTE 只能是 all33|primary", r.stderr)

    def test_runtime_detector_env_default_all33_and_caller_wins(self):
        env_file = os.path.join(ROOT, "ui/geomem/tools/competition/runtime_detector.env")
        out = lambda pre: subprocess.run(["bash", "-c", '%s source "$0"; echo "$CAR_YOLO_ROUTE"' % pre, env_file],
                                         stdout=subprocess.PIPE, universal_newlines=True,
                                         env={"PATH": os.environ["PATH"]}).stdout.strip()
        self.assertEqual(out(""), "all33")
        self.assertEqual(out("export CAR_YOLO_ROUTE=primary;"), "primary")


if __name__ == "__main__":
    unittest.main()
