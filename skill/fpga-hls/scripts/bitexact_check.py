#!/usr/bin/env python3
# Copyright (C) 2026 the contest team. SPDX-License-Identifier: MIT
# New tool (not in upstream Ross): byte-exact output gate for HLS optimisation attempts.
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
"""
bitexact_check.py <golden_dir> <candidate_dir> [--elem-size N] [--allow-extra]

Compares every file under <golden_dir> (recursively) byte-for-byte with the file of the
same relative path under <candidate_dir>. Prints one line per file:
    SAME     <rel>  (<n> bytes, sha256 <12 hex>)
    DIFF     <rel>  first difference at byte offset <off> (golden 0x.. vs candidate 0x..)
                    [element <off//N> if --elem-size N] [line <L> for text files]
    MISSING  <rel>  (absent in candidate)
    EXTRA    <rel>  (only in candidate; counts as failure unless --allow-extra)
Exit code: 0 all SAME; 1 any DIFF/MISSING/EXTRA; 2 usage error (incl. empty golden dir).

Typical 2022.2 locations (testbench writes relative "tb_out/", see bitexact.md):
    csim : <proj>/<sol>/csim/build/tb_out
    cosim: <proj>/<sol>/sim/wrapc_pc/tb_out   (post-check pass, built from RTL output vectors)
"""
from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

CHUNK = 1 << 20


def files_under(root: Path) -> list[str]:
    return sorted(str(p.relative_to(root)) for p in root.rglob("*") if p.is_file())


def first_diff(a: Path, b: Path):
    """Return (offset, byte_a, byte_b) of first difference, or None if identical.
    byte_x is None when that file ended first."""
    off = 0
    with a.open("rb") as fa, b.open("rb") as fb:
        while True:
            ca, cb = fa.read(CHUNK), fb.read(CHUNK)
            if ca == cb:
                if not ca:
                    return None
                off += len(ca)
                continue
            n = min(len(ca), len(cb))
            for i in range(n):
                if ca[i] != cb[i]:
                    return off + i, ca[i], cb[i]
            return off + n, (ca[n] if len(ca) > n else None), (cb[n] if len(cb) > n else None)


def sha(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for c in iter(lambda: f.read(CHUNK), b""):
            h.update(c)
    return h.hexdigest()


def line_of(p: Path, off: int):
    with p.open("rb") as f:
        head = f.read(off)
    if b"\0" in head[:4096]:
        return None
    return head.count(b"\n") + 1


def fmt(b):
    return "EOF" if b is None else f"0x{b:02x}"


def main() -> int:
    ap = argparse.ArgumentParser(description="byte-exact comparison of HLS testbench outputs")
    ap.add_argument("golden_dir")
    ap.add_argument("candidate_dir")
    ap.add_argument("--elem-size", type=int, default=0, help="also report element index = offset // N")
    ap.add_argument("--allow-extra", action="store_true", help="files only in candidate are not a failure")
    a = ap.parse_args()
    g, c = Path(a.golden_dir), Path(a.candidate_dir)
    if not g.is_dir():
        print(f"golden 目录不存在: {g}", file=sys.stderr)
        return 2
    gfiles = files_under(g)
    if not gfiles:
        print(f"golden 目录为空: {g}（没有可比对的输出，不能判一致）", file=sys.stderr)
        return 2
    if not c.is_dir():
        print(f"candidate 目录不存在: {c}（testbench 没写出输出？）", file=sys.stderr)
        return 1
    cfiles = set(files_under(c))
    bad = 0
    for rel in gfiles:
        gp, cp = g / rel, c / rel
        if rel not in cfiles:
            print(f"MISSING  {rel}  (absent in candidate)")
            bad += 1
            continue
        d = first_diff(gp, cp)
        if d is None:
            print(f"SAME     {rel}  ({gp.stat().st_size} bytes, sha256 {sha(gp)[:12]})")
            continue
        bad += 1
        off, bg, bc = d
        extra = []
        if a.elem_size > 0:
            extra.append(f"element {off // a.elem_size}")
        ln = line_of(gp, off)
        if ln is not None:
            extra.append(f"line {ln}")
        print(
            f"DIFF     {rel}  first difference at byte offset {off} (golden {fmt(bg)} vs candidate {fmt(bc)})"
            f"{'  ' + ', '.join(extra) if extra else ''}  sizes {gp.stat().st_size}/{cp.stat().st_size}"
        )
    for rel in sorted(cfiles - set(gfiles)):
        print(f"EXTRA    {rel}  (only in candidate)")
        if not a.allow_extra:
            bad += 1
    print(f"RESULT {'BITEXACT' if bad == 0 else 'MISMATCH'}: {len(gfiles)} golden files, {bad} problem(s)")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
