#!/usr/bin/env python3
"""record_calls.py output -> A53 bench data (u32 n, then per call: seq, cmd[1024], golden res after[1024]).
Asserts that every call's result-before equals the previous call's result-after (one persistent buffer)."""
import sys
import numpy as np
REC = 4 + 3 * 1024
a = np.fromfile(sys.argv[1], dtype=np.uint32).reshape(-1, REC)
for k in range(1, len(a)):
    assert (a[k, 4 + 1024:4 + 2048] == a[k - 1, 4 + 2048:]).all(), "result buffer not persistent at call %d" % k
assert not a[0, 4 + 1024:4 + 2048].any(), "first call must start from a zero result buffer"
out = np.concatenate([a[:, 2:3], a[:, 4:4 + 1024], a[:, 4 + 2048:]], axis=1)
with open(sys.argv[2], "wb") as f:
    np.array([len(a)], np.uint32).tofile(f)
    out.astype(np.uint32).tofile(f)
print(len(a), "calls")
