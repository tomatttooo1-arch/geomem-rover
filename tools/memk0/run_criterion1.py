#!/usr/bin/env python3
"""MEMK0 key comparison 1 runner: one data set, both arms (core.Memory vs PLMemory + C model), JSON report.

  python3 tools/memk0/run_criterion1.py d2 --out RUN/MEMK0/criterion1/d2.json
data sets (MEMK0/DESIGN.md section 10):
  d1      tools/m5_software_replay.py, 9 scenarios as they are (their memory config: existence_v3 off)
  d1prod  the same scenarios with existence_v3 left on (product existence; creation still ungated as m5 sets)
  d2      tests/data/memq_realcar_20261016_compact.json through replay_stream's new arm (product config)
  d3      seeded stress streams (tools/memk0/stress.py), seeds 1-3
  d4      tests/test_memory_*.py + tests/test_m7_memory.py under unittest (product-config memories only)
--bias M     arm B read-back positions + M metres (positive control b)
--chi X      arm B core built with association coefficient X instead of 2.45 (positive control a)
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.memk0 import shadow  # noqa: E402


def run_d1(prod):
    import tools.m5_software_replay as m5
    from perception.fpga_replay import core
    if prod:
        real = m5.replace

        def keep_v3(obj, **kw):
            if isinstance(obj, core.Config):
                kw.pop("existence_v3", None)
            return real(obj, **kw)
        m5.replace = keep_v3
    res = m5.run_scenarios(None)
    return {"scenarios": {r["name"]: r["passed"] for r in res}}


def run_d2():
    from tools.memory_quality import replay_stream as rs
    from perception.fpga_replay import core
    K, T_bc, image, frames, doc = rs.load_stream(ROOT / "tests/data/memq_realcar_20261016_compact.json")
    memory, snaps, applied = rs.run_arm(core, core.Config(), frames, K, T_bc, image, {}, True)
    return {"frames_in_stream": len(frames), "final_tracks": len(memory.tracks)}


def run_d3(seeds=(1, 2, 3)):
    from tools.memk0 import stress
    info = {}
    for seed in seeds:
        info["seed%d" % seed] = stress.run(seed)
    return info


def run_d4():
    """unittest in this process (pytest is not installed for this interpreter); the shadow is installed first."""
    import unittest
    tests = sorted(str(p.relative_to(ROOT)) for p in (ROOT / "tests").glob("test_memory_*.py"))
    tests.append("tests/test_m7_memory.py")
    # tests/ is not a package (the files import each other by bare name), so load them as top-level modules;
    # the first version used "tests.<name>" and every module failed to import (11 errors, nothing compared)
    sys.path.insert(0, str(ROOT / "tests"))
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for t in tests:
        suite.addTests(loader.loadTestsFromName(Path(t).stem))
    import io
    buf = io.StringIO()
    r = unittest.TextTestRunner(stream=buf, verbosity=0).run(suite)
    return {"tests": tests, "run": r.testsRun, "failures": len(r.failures), "errors": len(r.errors),
            "skipped": len(r.skipped), "first_problems": [str(x[0]) + ": " + x[1].strip().splitlines()[-1]
                                                          for x in (r.failures + r.errors)[:10]]}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("data", choices=["d1", "d1prod", "d2", "d3", "d4"])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--bias", type=float, default=0.0)
    p.add_argument("--chi", type=float, default=None)
    p.add_argument("--seeds", default="1,2,3", help="d3 only: comma-separated seeds (debugging; the data set is 1,2,3)")
    a = p.parse_args(argv)
    chi2 = None if a.chi is None else int(round(a.chi * a.chi * (1 << 24)))
    t0 = time.time()
    if True:
        shadow.install(bias=a.bias, chi2_24=chi2)
        info = {"d1": lambda: run_d1(False), "d1prod": lambda: run_d1(True), "d2": run_d2,
                "d3": lambda: run_d3(tuple(int(x) for x in a.seeds.split(","))),
                "d4": run_d4}[a.data]()
        d = shadow.report(None, {"data": a.data, "run": info, "bias_m": a.bias, "chi": a.chi,
                                 "seeds": a.seeds if a.data == "d3" else None,
                                 "seconds": round(time.time() - t0, 1)})
    # a memory that hits a core limit (Unsupported) silently leaves the comparison; that is lost coverage,
    # so it can no longer read as PASS (it is reported as INCOMPLETE with the count)
    d["core_limit_skips"] = sum(v for k, v in d["skipped"].items() if k.startswith("memory:core_limit:"))
    d["verdict"] = ("FAIL" if d["mismatching_calls"] else
                    "NOTHING_COMPARED" if sum(d["compared_calls"].values()) == 0 else
                    "INCOMPLETE" if d["core_limit_skips"] else "PASS")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(d, indent=1, ensure_ascii=False, default=str))
    print(json.dumps({k: d[k] for k in ("data", "verdict", "compared_calls", "mismatching_calls", "core_limit_skips", "skipped",
                                        "max_abs_diff", "core_calls", "seconds")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
