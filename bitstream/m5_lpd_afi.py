#!/usr/bin/env python3
"""DRV5 (2026-11-04): S_AXI_LPD prerequisite and AFI width readout after loading a bitstream. Runs ON THE KV260 as
root (python3 + /dev/mem: the KV has no devmem). Called by board_bringup.sh step 1 and kv_start.sh after the
BUILD_ID readback; usable by hand:  sudo python3 m5_lpd_afi.py [--json out.json] [--force-m5] [--no-clear]

What it does (MERGE6 DESIGN.md section 8 items 2-3, controller ruling 2 of 11-03 23:4x):
  1. reads sysinfo (0xA000_0000) MAGIC and SLOT_MAP; the bitstream is "m5-like" when SLOT_MAP has bit 11 (pre-
     processing) or bit 12 (planner), i.e. it carries masters on S_AXI_LPD (--force-m5 skips the sysinfo read);
  2. m5-like: reads RST_LPD_TOP 0xFF5E023C; if bit 19 (the LPD AFI, AFIFM6, held in reset after power-up per
     psu_init) is 1, clears ONLY that bit (read-modify-write) and reads back to confirm. Other bitstreams: read only;
  3. reads the AFI FM read/write width registers (low 2 bits: 0 = 128, 1 = 64, 2 = 32 bit) of HP0..HP3, HPC0/1 and
     LPD (0xFF9B0000 / 0xFF9B0014) and, for m5-like builds, compares them with the m5 expectation
     (HP0/HP1 = 2, HP3 = 1, HP2/HPC0/HPC1 = 0, LPD = 0);
  4. lists /sys/class/remoteproc/*/state (the RPU must not compete for the LPD).
Owner rule (10-26): anomalies are WARNINGs only; the exit code is always 0 and nothing is refused or stopped.
"""
import argparse
import glob
import json
import mmap
import os
import struct
import sys
import time

RST_LPD_TOP = 0xFF5E023C
AFI_LPD_BIT = 19
SYSINFO = 0xA0000000
AFI = [  # name, read-width register (RDCTRL), write-width register (WRCTRL), m5 expectation (low 2 bits)
    ("HPC0", 0xFD360000, 0xFD360014, 0),
    ("HPC1", 0xFD370000, 0xFD370014, 0),
    ("HP0", 0xFD380000, 0xFD380014, 2),
    ("HP1", 0xFD390000, 0xFD390014, 2),
    ("HP2", 0xFD3A0000, 0xFD3A0014, 0),
    ("HP3", 0xFD3B0000, 0xFD3B0014, 1),
    ("LPD", 0xFF9B0000, 0xFF9B0014, 0),
]


class Mem:
    def __init__(self, write=False):
        # M5_LPD_DEVMEM: a sparse file standing in for /dev/mem (host test only)
        self.fd = os.open(os.environ.get("M5_LPD_DEVMEM", "/dev/mem"), (os.O_RDWR if write else os.O_RDONLY) | os.O_SYNC)
        self.write = write
        self.maps = {}

    def _page(self, a):
        base = a & ~0xFFF
        if base not in self.maps:
            prot = mmap.PROT_READ | (mmap.PROT_WRITE if self.write else 0)
            self.maps[base] = mmap.mmap(self.fd, 4096, mmap.MAP_SHARED, prot, offset=base)
        return self.maps[base], a - base

    def rd(self, a):
        m, o = self._page(a)
        return struct.unpack_from("<I", m, o)[0]

    def wr(self, a, v):
        m, o = self._page(a)
        struct.pack_into("<I", m, o, v & 0xFFFFFFFF)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--json", help="also write the readout here")
    ap.add_argument("--force-m5", action="store_true", help="treat the bitstream as m5 without reading sysinfo")
    ap.add_argument("--no-clear", action="store_true", help="read only, never clear bit 19")
    a = ap.parse_args()
    out = {"t": time.strftime("%Y-%m-%dT%H:%M:%S"), "warnings": []}
    warn = out["warnings"].append
    try:
        mem = Mem(write=not a.no_clear)
    except OSError as exc:
        print("LPD_AFI WARNING cannot open /dev/mem (%s): no LPD / AFI readout" % exc)
        return 0
    m5 = a.force_m5
    if not m5:
        magic = mem.rd(SYSINFO)
        slot = mem.rd(SYSINFO + 0x20) if magic == 0x4D37504C else None
        out.update(sysinfo_magic="%#010x" % magic, slot_map=None if slot is None else "%#06x" % slot)
        m5 = slot is not None and bool(slot & ((1 << 11) | (1 << 12)))
        if slot is None:
            warn("sysinfo MAGIC %#x is not the m7 sysinfo: treated as not m5 (read only)" % magic)
    out["m5_like"] = m5
    v = mem.rd(RST_LPD_TOP)
    out["rst_lpd_top_before"] = "%#010x" % v
    out["afi_lpd_reset_before"] = (v >> AFI_LPD_BIT) & 1
    if (v >> AFI_LPD_BIT) & 1:
        if m5 and not a.no_clear:
            mem.wr(RST_LPD_TOP, v & ~(1 << AFI_LPD_BIT))
            after = mem.rd(RST_LPD_TOP)
            out["rst_lpd_top_after"] = "%#010x" % after
            out["afi_lpd_cleared"] = True
            if (after >> AFI_LPD_BIT) & 1:
                warn("RST_LPD_TOP bit 19 still 1 after the clear (read back %#010x)" % after)
            if (after ^ v) != (1 << AFI_LPD_BIT):
                warn("RST_LPD_TOP changed other bits: before %#010x after %#010x" % (v, after))
        elif m5:
            warn("LPD AFI (RST_LPD_TOP bit 19) is in reset and --no-clear was given: LPD masters will not reach DDR")
    afi = {}
    for name, r, w, exp in AFI:
        rv, wv = mem.rd(r), mem.rd(w)
        afi[name] = {"rd": rv & 3, "wr": wv & 3, "rd_raw": "%#x" % rv, "wr_raw": "%#x" % wv}
        if m5 and ((rv & 3) != exp or (wv & 3) != exp):
            warn("AFI %s width rd=%d wr=%d, m5 expects %d (0=128,1=64,2=32 bit)" % (name, rv & 3, wv & 3, exp))
    out["afi"] = afi
    rp = {}
    for p in sorted(glob.glob("/sys/class/remoteproc/*/state")):
        try:
            rp[p.split("/")[4]] = open(p).read().strip()
        except OSError:
            pass
    out["remoteproc"] = rp
    for k, s in rp.items():
        if m5 and s != "offline":
            warn("remoteproc %s is %s (the RPU may compete for the LPD)" % (k, s))
    widths = " ".join("%s=%d/%d" % (n, afi[n]["rd"], afi[n]["wr"]) for n, _, _, _ in AFI)
    print("LPD_AFI m5=%d rst_lpd_top=%s bit19_before=%d%s afi(rd/wr) %s remoteproc=%s" % (
        int(m5), out["rst_lpd_top_before"], out["afi_lpd_reset_before"],
        (" cleared->%s" % out["rst_lpd_top_after"]) if out.get("afi_lpd_cleared") else "", widths,
        ",".join("%s:%s" % kv for kv in rp.items()) or "none"))
    for w_ in out["warnings"]:
        print("LPD_AFI WARNING " + w_)
    print("LPD_AFI %s" % ("OK" if not out["warnings"] else "WARN (%d)" % len(out["warnings"])))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:   # never break the caller's load sequence
        print("LPD_AFI WARNING readout failed: %r" % (exc,))
        sys.exit(0)
