"""10-18 object memory across runs: car_up.sh CAR_MEMORY_IN / CAR_MEMORY_SAVE (docs/design/MEMORY_PERSIST_20261018.md).
Dry runs and refusals only (fake docker / lock from test_car_run.Base); nothing is started."""
import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from host import memory_store as ms  # noqa: E402
from test_car_run import Base  # noqa: E402
from test_memory_persist import _doc  # noqa: E402

SIM_MAP = os.path.join(ROOT, "robot_sim", "fpga_sim", "maps", "room.yaml")


class CarUpMemory(Base):
    def memory_file(self, name, ident=None, **change):
        doc = _doc(ident=ident or ms.map_identity(sim_map_yaml=SIM_MAP))
        doc.update(change)
        path = os.path.join(self.tmp, name)
        ms.save_atomic(doc, path)
        return path

    def runtime_line(self, out):
        return next(line for line in out.splitlines() if line.startswith("    [runtime]"))

    def test_default_saves_and_passes_identity(self):
        r = self.up("sim", "--dry-run")
        self.assertEqual(r.returncode, 0, r.stderr)
        line = self.runtime_line(r.stdout)
        self.assertIn("--memory-map-identity", line)
        self.assertIn("--memory-out-dir", line)
        self.assertNotIn("--memory-in", line)
        self.assertIn("map identity: sim map", r.stdout)
        self.assertIn("sim_map", r.stdout)
        r = self.up("sim", "--dry-run", CAR_MEMORY_SAVE="0")
        self.assertNotIn("--memory-out-dir", self.runtime_line(r.stdout))
        self.assertEqual(self.up("sim", "--dry-run", CAR_MEMORY_SAVE="yes").returncode, 2)

    def test_matching_file_accepted(self):
        r = self.up("sim", "--dry-run", CAR_MEMORY_IN=self.memory_file("good.json"))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("ACCEPTED", r.stdout)
        self.assertIn("--memory-in", self.runtime_line(r.stdout))

    def test_refusals_exit_2_before_anything(self):
        cases = {
            "map_mismatch": self.memory_file("other_map.json", ident=dict(ms.map_identity(sim_map_yaml=SIM_MAP),
                                                                          map_yaml_sha256="0" * 64)),
            "wrong_version": self.memory_file("v2.json", v=2),
            "incomplete": self.memory_file("snap.json", complete=False),
        }
        bad = os.path.join(self.tmp, "bad.json")
        with open(bad, "w") as stream:
            stream.write('{"format": "fpga-object-memory", "v": 1, "tr')
        cases["bad_json"] = bad
        for code, path in sorted(cases.items()):
            for dry in (True, False):
                r = self.up("sim", *(["--dry-run"] if dry else []), CAR_MEMORY_IN=path)
                self.assertEqual(r.returncode, 2, (code, dry, r.stdout[-500:], r.stderr))
                self.assertIn(code, r.stderr)
                self.assertIn("不降级为空记忆", r.stderr)
        self.assertFalse(os.path.isdir(self.runs) and os.listdir(self.runs))   # no run directory was made
        r = self.up("sim", "--dry-run", CAR_MEMORY_IN=os.path.join(self.tmp, "missing.json"))
        self.assertEqual(r.returncode, 2)

    def test_no_map_run_refuses_any_file(self):
        r = self.up("replay", "--dry-run", CAR_MEMORY_IN=self.memory_file("good.json"))
        self.assertEqual(r.returncode, 2)
        self.assertIn("map_mismatch", r.stderr)


if __name__ == "__main__":
    unittest.main()
