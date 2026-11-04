"""Two-slot frame ring in a shared-memory file (N4, docs/design/NANO_FULLLOAD_20260927.md).

The ROS measurement node inside the container writes the newest colour frame; the host-side
detector (no rclpy on the Nano host) reads it. Layout (little endian):
  header 64 B : magic u32 'FPGAR', width u32, height u32, channels u32, seq u64 (last complete frame)
  meta  2x32 B: per slot seq u64, stamp_ns u64 (message header), rx_ns u64 (CLOCK_REALTIME at receipt), pad
  data  2 slots of width*height*channels bytes
Writer: fill slot (seq+1)%2 data and meta, then publish seq. Reader: read seq, copy the slot, re-read
the slot's meta seq; if it changed the writer lapped us and the copy is discarded.
"""
import os
import struct
import time

import numpy as np

MAGIC = 0x52444C50
HDR, META = 64, 32


class RingWriter:
    def __init__(self, path, width, height, channels=3):
        self.w, self.h, self.c = width, height, channels
        self.n = width * height * channels
        size = HDR + 2 * META + 2 * self.n
        fd = os.open(path + ".tmp", os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o666)
        os.ftruncate(fd, size)
        self.mm = np.memmap(path + ".tmp", dtype=np.uint8, mode="r+", shape=(size,))
        os.close(fd)
        self.mm[:24] = np.frombuffer(struct.pack("<IIIIQ", MAGIC, width, height, channels, 0), np.uint8)
        os.replace(path + ".tmp", path)
        self.seq = 0

    def write(self, data, stamp_ns, rx_ns):
        s = self.seq + 1
        slot = s % 2
        off = HDR + 2 * META + slot * self.n
        self.mm[HDR + slot * META: HDR + slot * META + 8] = np.frombuffer(struct.pack("<Q", 0), np.uint8)
        self.mm[off:off + self.n] = np.frombuffer(data, np.uint8, count=self.n)
        self.mm[HDR + slot * META: HDR + slot * META + 24] = np.frombuffer(
            struct.pack("<QQQ", s, stamp_ns, rx_ns), np.uint8)
        self.mm[16:24] = np.frombuffer(struct.pack("<Q", s), np.uint8)
        self.seq = s


class RingReader:
    def __init__(self, path, timeout_s=60.0):
        end = time.time() + timeout_s
        while not os.path.exists(path):
            if time.time() > end:
                raise RuntimeError("ring %s not found" % path)
            time.sleep(0.2)
        self.mm = np.memmap(path, dtype=np.uint8, mode="r")
        magic, self.w, self.h, self.c, _ = struct.unpack("<IIIIQ", self.mm[:24].tobytes())
        if magic != MAGIC:
            raise RuntimeError("bad ring magic")
        self.n = self.w * self.h * self.c
        self.lapped = 0

    def seq(self):
        return struct.unpack("<Q", self.mm[16:24].tobytes())[0]

    def read(self, s):
        """Copy frame s (must be the newest or the one before); None if it was overwritten."""
        slot = s % 2
        m = HDR + slot * META
        s0, stamp, rx = struct.unpack("<QQQ", self.mm[m:m + 24].tobytes())
        if s0 != s:
            self.lapped += 1
            return None
        off = HDR + 2 * META + slot * self.n
        arr = np.array(self.mm[off:off + self.n]).reshape(self.h, self.w, self.c)
        s1 = struct.unpack("<Q", self.mm[m:m + 8].tobytes())[0]
        if s1 != s:
            self.lapped += 1
            return None
        return arr, stamp, rx
