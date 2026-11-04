"""MEMK0 C model: EXEC's operand check. QUERY kind 2 (slot against slots, relocation absorption) reads no
observation, so it must pass in a geometry-only EXEC (no observations this frame); kinds 0/1 read observation
i and must still be refused when i >= the observation count (ERR_SLOT = 4)."""
import os
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception.fpga_replay import memory_pl as mp  # noqa: E402

ERR_SLOT = 4


def call(lib, kind, n, mu, ops):
    c = np.zeros(mp.WORDS, np.uint32); r = np.zeros(mp.WORDS, np.uint32)
    c[0] = mp.MAGIC | (kind << 16)
    c[1] = (n & 0xFF) | (mu << 8) | (len(ops) << 16)
    for k, w in enumerate(ops):
        c[mp.OP_BASE + mp.OP_W * k: mp.OP_BASE + mp.OP_W * k + len(w)] = w
    lib.memk0_call(c.ctypes.data, r.ctypes.data)
    return int(r[0]) & 0xFF


class ErrSlotTests(unittest.TestCase):
    def setUp(self):
        lib = Path(os.environ["MEMK0_TEST_LIB"]) if "MEMK0_TEST_LIB" in os.environ else None
        if lib is None:
            self.lib = mp.CModel.get().lib
        else:
            import ctypes
            self.lib = ctypes.CDLL(str(lib)); self.lib.memk0_call.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.assertEqual(call(self.lib, mp.CMD_INIT, 0, 0, []), 0)
        # slot 0 becomes a track (OP_NEW a=0, class 1, type 1) in an EXEC without observations
        self.assertEqual(call(self.lib, mp.CMD_EXEC, 0, 1, [(mp.OP_NEW | (1 << 16), 1, 0, 0)]), 0)

    def test_slot_query_passes_without_observations(self):
        q = (mp.OP_QUERY | (0 << 8), 2 | (1 << 2) | (4 << 8), 0, 0)
        self.assertEqual(call(self.lib, mp.CMD_EXEC, 0, 1, [q]), 0)

    def test_observation_query_still_refused_without_observations(self):
        q = (mp.OP_QUERY | (0 << 8) | (0 << 24), 1 | (1 << 2) | (2 << 8), 0, 0)
        self.assertEqual(call(self.lib, mp.CMD_EXEC, 0, 1, [q]) & ERR_SLOT, ERR_SLOT)


if __name__ == "__main__":
    unittest.main()
