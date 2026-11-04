"""RECOG640 (2026-11-04): the RECOG_MODEL switch (tools/dpu_pipe/recog/recog_args.py).
Host checks of the 640 pre-/post-processing against dev3's code live in RUN/RECOG640 (check_prep640.py, check_post640.py)."""
import json, os, subprocess, sys, unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RA = os.path.join(ROOT, "tools", "dpu_pipe", "recog", "recog_args.py")


def run(env, *a):
    e = dict(os.environ); e.update(env)
    for k in ("RECOG_MODEL", "RECOG_THR_MEDBOX", "RECOG_CLS_THR", "RECOG_XMODEL"):
        if k not in env: e.pop(k, None)
    r = subprocess.run([sys.executable, RA] + list(a), capture_output=True, text=True, env=e, check=True)
    return r.stdout, r.stderr


class RecogSwitch(unittest.TestCase):
    def test_default_is_640_with_json_thresholds(self):
        j = json.loads(run({}, "--format", "json")[0])
        self.assertEqual((j["model"], j["size"], j["fix"], j["nc"]), ("y8s640", 640, 6, 6))
        self.assertEqual(j["cls_thr"], [0.5, 0.35, 0.4, 0.7, 0.25, 0.15])
        out = run({}, "--format", "args")[0]
        for flag in ("--conf 0.01", "--iou 0.7", "--max-det 100", "--sort-heads", "--cls-thr 0.5,0.35,0.4,0.7,0.25,0.15"):
            self.assertIn(flag, out)

    def test_medicine_box_override_only_touches_class_5(self):
        j = json.loads(run({"RECOG_THR_MEDBOX": "0.05"}, "--format", "json")[0])
        self.assertEqual(j["cls_thr"], [0.5, 0.35, 0.4, 0.7, 0.25, 0.05])
        self.assertEqual(j["thr_source"]["medicine box"], "RECOG_THR_MEDBOX")

    def test_legacy_512_keeps_dpupipe1_post(self):
        out = run({"RECOG_MODEL": "y8s512"}, "--format", "args")[0]
        self.assertIn("--size 512 --fix 7", out)
        self.assertNotIn("--cls-thr", out)

    def test_reserved_rect_and_unknown_fall_back_with_warning(self):
        for m in ("y8s640r", "nope"):
            o, e = run({"RECOG_MODEL": m}, "--format", "json")
            self.assertEqual(json.loads(o)["model"], "y8s640"); self.assertIn("WARN", e)


if __name__ == "__main__":
    unittest.main()
