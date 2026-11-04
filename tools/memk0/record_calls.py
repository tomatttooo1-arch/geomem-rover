#!/usr/bin/env python3
"""Record every core call of PLMemory (cmd block, result block before and after) to a binary file.

Used for (a) the C-model differential check (old source vs HLS-form source, tb_memk0.cpp replays the file) and
(b) the HLS co-simulation vectors (key comparison 2). One memory per stream (the C state is not swapped).

  python3 tools/memk0/record_calls.py d2 OUT.bin [--first A --last B]       frames A..B of the 10-16 stream
  python3 tools/memk0/record_calls.py d3 OUT.bin --seed S [--frames N]      stress stream seed S, first N frames
File: per call 4 + 3*1024 little-endian u32: [0x4D4B5243, call index, frame seq (cmd h3), 0], cmd, res before,
res after. A summary (calls per frame seq, ops and observations per call) goes to OUT.bin.json.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from perception.fpga_replay import memory_pl  # noqa: E402
from tools.memk0 import shadow  # noqa: E402

REC = {"f": None, "n": 0, "rows": []}


def _install_recorder():
    orig = memory_pl.CModel.call

    def call(self, owner, cmd, res):
        before = np.array(res, dtype=np.uint32, copy=True)
        orig(self, owner, cmd, res)
        c = np.asarray(cmd, dtype=np.uint32)
        hdr = np.array([0x4D4B5243, REC["n"], int(c[3]) & 0xFFFFFF, 0], np.uint32)
        REC["f"].write(hdr.tobytes() + c.tobytes() + before.tobytes() + np.asarray(res, np.uint32).tobytes())
        kind = (int(c[0]) >> 16) & 15
        REC["rows"].append([REC["n"], int(c[3]) & 0xFFFFFF, kind, int(c[1]) & 0xFF, (int(c[1]) >> 16) & 0xFF,
                            int(res[0]) & 0xFF, int(res[1])])
        REC["n"] += 1
    memory_pl.CModel.call = call


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("data", choices=["d2", "d3"])
    p.add_argument("out", type=Path)
    p.add_argument("--first", type=int, default=0)
    p.add_argument("--last", type=int, default=None)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--frames", type=int, default=3000)
    a = p.parse_args(argv)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    REC["f"] = open(a.out, "wb")
    _install_recorder()
    shadow.install()
    if a.data == "d2":
        from tools.memory_quality import replay_stream as rs
        from perception.fpga_replay import core
        K, T_bc, image, frames, doc = rs.load_stream(ROOT / "tests/data/memq_realcar_20261016_compact.json")
        frames = frames[a.first:(None if a.last is None else a.last + 1)]
        rs.run_arm(core, core.Config(), frames, K, T_bc, image, {}, True)
    else:
        from tools.memk0 import stress
        stress.run(a.seed, frames=a.frames)
    REC["f"].close()
    d = shadow.report(None, {})
    summary = {"calls": REC["n"], "mismatching_calls": d["mismatching_calls"], "skipped": d["skipped"],
               "columns": ["call", "seq", "kind", "n_obs", "n_ops", "err", "n_pairs"], "rows": REC["rows"]}
    Path(str(a.out) + ".json").write_text(json.dumps(summary))
    print(json.dumps({k: summary[k] for k in ("calls", "mismatching_calls", "skipped")}))


if __name__ == "__main__":
    sys.exit(main())
