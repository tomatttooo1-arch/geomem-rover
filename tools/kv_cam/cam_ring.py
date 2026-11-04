"""Python reader of the kv_cam shared-memory ring (NN-CAM1, 2026-11-04; protocol in protocol.md).

The seqlock ordering is done in C (libcamring.so, cam_ring_c.cpp) through ctypes, so a Python consumer
(front end, ground calibration, preview) gets the same "never a torn frame" guarantee as the C++ reader.

    ring = CamRing.open()                      # /dev/shm/kv_cam_v1
    seq = ring.wait_newer(0, 1000)
    st, meta, yuyv, depth = ring.read(seq)     # st == "ok": yuyv (480, 640, 2) uint8, depth (480, 640) uint16
"""
import ctypes
import os
import struct

import numpy as np

STATUS = {0: "ok", 1: "not_yet", 2: "overwritten", 3: "corrupt"}
FLAG_HAS_COLOUR, FLAG_HAS_DEPTH, FLAG_NEAR_ZERO, FLAG_RATE_SUSPECT, FLAG_CRC, FLAG_UNALIGNED = 1, 2, 4, 8, 16, 32

# slot header fields at offset 16 (after seq_begin / seq_end): see SlotHeader in cam_ring.h
_SLOT = struct.Struct("<QQQQQQQQQIIIIIIfI")
_SLOT_FIELDS = ("seq", "colour_index", "depth_index", "colour_dev_ts_us", "depth_dev_ts_us", "colour_sys_ts_us",
                "depth_sys_ts_us", "arrive_ns", "publish_ns", "flags", "colour_bytes", "depth_bytes", "crc_colour",
                "crc_depth", "epoch", "depth_scale_mm", "depth_valid_permille")


def _default_lib():
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (os.environ.get("KVCAM_LIB", ""), os.path.join(here, "libcamring.so"),
              os.path.join(here, "build", "host", "libcamring.so"), os.path.join(here, "build", "arm", "libcamring.so")):
        if p and os.path.exists(p):
            return p
    raise FileNotFoundError("libcamring.so not found (make host / make arm, or set KVCAM_LIB)")


class CamRing:
    def __init__(self, handle, lib):
        self._h, self._lib = handle, lib
        hdr = ctypes.string_at(lib.kvcam_header(handle), 512)
        (self.magic, self.version, _hb, self.nslots, _shb, self.slot_stride, _so, self.total_bytes) = \
            struct.unpack_from("<QIIIIQQQ", hdr, 0)
        (self.colour_w, self.colour_h, self.colour_fourcc, self.colour_stride, self.colour_bytes,
         self.depth_w, self.depth_h, self.depth_fourcc, self.depth_stride, self.depth_bytes) = \
            struct.unpack_from("<10I", hdr, 48)
        self.colour_bpp = self.colour_stride // self.colour_w
        self._meta = ctypes.create_string_buffer(4096)

    @classmethod
    def open(cls, name="kv_cam_v1", path=None, lib_path=None):
        lib = ctypes.CDLL(lib_path or _default_lib())
        lib.kvcam_open.restype = ctypes.c_void_p
        lib.kvcam_open.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        lib.kvcam_close.argtypes = [ctypes.c_void_p]
        lib.kvcam_header.restype = ctypes.c_void_p
        lib.kvcam_header.argtypes = [ctypes.c_void_p]
        lib.kvcam_latest.restype = ctypes.c_uint64
        lib.kvcam_latest.argtypes = [ctypes.c_void_p]
        lib.kvcam_wait_newer.restype = ctypes.c_uint64
        lib.kvcam_wait_newer.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int]
        lib.kvcam_stale.restype = ctypes.c_int
        lib.kvcam_stale.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        lib.kvcam_read.restype = ctypes.c_int
        lib.kvcam_read.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
        h = lib.kvcam_open(name.encode(), path.encode() if path else None)
        if not h:
            raise OSError("cannot open kv_cam ring %r (writer not running or incompatible)" % name)
        return cls(h, lib)

    def close(self):
        if self._h:
            self._lib.kvcam_close(self._h)
            self._h = None

    def latest(self):
        return self._lib.kvcam_latest(self._h)

    def wait_newer(self, after, timeout_ms):
        return self._lib.kvcam_wait_newer(self._h, after, timeout_ms)

    def stale(self, max_age_s=2.0):
        return bool(self._lib.kvcam_stale(self._h, int(max_age_s * 1e9)))

    def read(self, seq, colour=True, depth=True):
        """-> (status, meta dict or None, colour ndarray or None, depth ndarray or None)."""
        c = np.empty((self.colour_h, self.colour_w, self.colour_bpp), np.uint8) if colour else None
        d = np.empty((self.depth_h, self.depth_w), np.uint16) if depth else None
        st = self._lib.kvcam_read(self._h, seq, self._meta,
                                  c.ctypes.data if c is not None else None, d.ctypes.data if d is not None else None)
        if st != 0:
            return STATUS.get(st, "corrupt"), None, None, None
        meta = dict(zip(_SLOT_FIELDS, _SLOT.unpack_from(self._meta.raw, 16)))
        return "ok", meta, c, d
