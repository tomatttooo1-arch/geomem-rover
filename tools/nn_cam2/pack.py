"""NN-CAM2: read a kv_cam replay pack (KVCAMPK1, tools/kv_cam/kv_cam_common.h) with numpy memmap."""
import struct

import numpy as np

MAGIC = 0x314b504d4143564b


class Pack:
    def __init__(self, path):
        self.mm = np.memmap(path, np.uint8, "r")
        (magic, ver, self.n, self.cw, self.ch, _cf, self.cb, self.dw, self.dh, _df, self.db) = struct.unpack_from(
            "<QII8I", self.mm[:64].tobytes())
        assert magic == MAGIC and ver == 1
        self.stride = 32 + self.cb + self.db

    def colour(self, i):
        o = 64 + (i % self.n) * self.stride + 32
        return self.mm[o:o + self.cb].tobytes()

    def depth(self, i):
        o = 64 + (i % self.n) * self.stride + 32 + self.cb
        return np.frombuffer(self.mm[o:o + self.db].tobytes(), "<u2").reshape(self.dh, self.dw)
