#!/usr/bin/env python3
"""DRV5: pull the parameter blocks the PREP1 s18 co-simulation fed the RTL out of the HLS transaction-vector file
(build/s18ooc/sol1/sim/tv/cdatafile/c.prep_top.autotvin_gmem.dat) -- the blocks the RTL actually read, not a
re-run of prep_drv.h.

Format (Vitis HLS 2022.2, binary m_axi bundle): per transaction an 8-byte big-endian word count (= sum of the
port depths src 57600 + dst 49152 + prm 643 = 107395) and then the words, 16 bytes each, most significant byte
first. Port offsets inside a transaction come from autotvin_{src,dst,prm}.dat (byte offsets 0 / 0xe1000 /
0x1a1000). Transactions follow vec_cosim.txt order (sq512_randA, sq416_yuyv_fancy_rand1).

  extract_cosim_prm.py <cdatafile dir> <vec_cosim.txt> <out dir>   -> <out>/<id>.prm.bin (643*16 bytes each)
"""
import hashlib
import os
import re
import sys


def offsets(path):
    return [int(x, 16) for x in re.findall(r"^0x([0-9a-fA-F]+)$", open(path).read(), re.M)]


def main():
    cd, vec, out = sys.argv[1:4]
    ids = [l.split()[0] for l in open(vec) if l.strip() and not l.startswith("#")]
    prm_off = offsets(os.path.join(cd, "c.prep_top.autotvin_prm.dat"))
    d = open(os.path.join(cd, "c.prep_top.autotvin_gmem.dat"), "rb").read()
    pos = 0
    os.makedirs(out, exist_ok=True)
    for t, name in enumerate(ids):
        n = int.from_bytes(d[pos:pos + 8], "big")
        body = d[pos + 8:pos + 8 + 16 * n]
        assert len(body) == 16 * n, "truncated transaction %d" % t
        words = [body[16 * i:16 * i + 16][::-1] for i in range(n)]   # MSB-first -> byte 0 = bits [7:0]
        w0 = prm_off[t] // 16
        blk = b"".join(words[w0:w0 + 643])
        assert blk[:2] == b"\x52\x50", "%s: no 0x5052 magic at the prm offset" % name
        p = os.path.join(out, name + ".prm.bin")
        open(p, "wb").write(blk)
        print("%s %d bytes sha256 %s" % (p, len(blk), hashlib.sha256(blk).hexdigest()))
        pos += 8 + 16 * n
    assert len(ids) == len(prm_off) == 2


if __name__ == "__main__":
    main()
