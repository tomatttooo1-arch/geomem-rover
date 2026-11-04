"""DPU input preprocessing on the Nano (DPUPIPE0, 2026-11-03): colour frame -> the int8 bytes the
DPU input tensor wants, byte-identical to dev3's evaluator (shared/models/20261103-dev3-recog-quant2,
evaluation/frozen/protocol.json "preprocess": Ultralytics 8.4.69 validator pixels, ceil initial
resize, square letterbox 114, RGB / 255) followed by the xmodel's input data-fix (int8, fix_point
read from the compiled xmodel: 640 -> 6, 512/416 -> 7, round_mode DPU_ROUND).

Pipeline (ultralytics/data/base.py load_image + augment.py LetterBox(scaleup=False, center=True)):
  1. r = size / max(h0, w0); if r != 1: cv2.resize(INTER_LINEAR) to (min(ceil(w0 r), size), min(ceil(h0 r), size))
  2. LetterBox: r2 = min(1, size/h, size/w); new_unpad = (round(w r2), round(h r2)); resize again if different
     (INTER_LINEAR); pad dw/2, dh/2 with top/left = round(d - 0.1), bottom/right = round(d + 0.1), value 114.
  3. RGB (camera frames are RGB already), x / 255 in float32, then data-fix: q = DPU_ROUND(x * 2^fp) clamped
     to [-128, 127]. For x in 0..255 the product x*2^fp/255 is never within 1/510 of a .5 tie, so the
     rounding mode does not matter and the whole step is the 256-entry table ``quant_lut`` (255 -> 128
     saturates to 127 at fix_point 7).

Resize backends:
  * "numpy": a re-implementation of OpenCV's 8-bit INTER_LINEAR (11-bit fixed-point coefficients,
    vertical pass ((b0*(h0>>4))>>16 + (b1*(h1>>4))>>16 + 2) >> 2). Bit-exact against host cv2 4.12 on
    random images (tests/test_dpu_prep.py); pure integer, so it gives the same bytes on any CPU.
  * "cv2": cv2.resize. Faster, but an ARM build may route through the Carotene HAL, so it is only used
    after ``selftest_cv2`` shows it equals the numpy path on this machine (``backend="auto"``).

Wire layout (what the sender ships): only the content rectangle, int8, NHWC, row-major. For landscape
frames whose resized width equals ``size`` (the car camera: 640x480 at 640/512/416) the content is one
contiguous byte range of the size x size x 3 tensor starting at ``content_offset``; the receiver fills
the pad once with ``pad_q`` and never touches it again.
Python 3.8 compatible.
"""
import math
import zlib

import numpy as np

PAD_VALUE = 114
COEF_BITS = 11
COEF_SCALE = 1 << COEF_BITS
# fix_point of the compiled xmodels' input data-fix (DPUPIPE0/tools/xmodel_input_fix.py, 2026-11-03):
# qat/compiled-selected/y8s_six_qat640_b1024.xmodel and dpu/640: 6; dpu/512, dpu/416: 7.
DEFAULT_FIX_POINT = {640: 6, 512: 7, 416: 7}


def quant_lut(fix_point):
    """int8 data-fix of x/255 for x = 0..255: round(x * 2^fp / 255) (exact integer form), saturated."""
    fp = int(fix_point)
    if not 0 <= fp <= 7:
        raise ValueError("fix_point %d outside 0..7" % fp)
    x = np.arange(256, dtype=np.int64)
    q = (2 * x * (1 << fp) + 255) // 510          # floor(x*2^fp/255 + 0.5); never a tie for 0 < x < 255
    return np.clip(q, -128, 127).astype(np.int8)


class LetterboxGeometry:
    """Ultralytics validator geometry for one source size and one square input size."""

    def __init__(self, src_w, src_h, size):
        self.src_w, self.src_h, self.size = int(src_w), int(src_h), int(size)
        r = self.size / max(self.src_h, self.src_w)
        if r != 1:
            self.load_w = min(math.ceil(self.src_w * r), self.size)
            self.load_h = min(math.ceil(self.src_h * r), self.size)
        else:
            self.load_w, self.load_h = self.src_w, self.src_h
        r2 = min(self.size / self.load_h, self.size / self.load_w, 1.0)
        self.w, self.h = int(round(self.load_w * r2)), int(round(self.load_h * r2))
        dw, dh = (self.size - self.w) / 2, (self.size - self.h) / 2
        self.top, self.bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        self.left, self.right = int(round(dw - 0.1)), int(round(dw + 0.1))
        assert self.top + self.h + self.bottom == self.size and self.left + self.w + self.right == self.size

    @property
    def contiguous(self):
        """True when the content rows are full tensor rows (one contiguous byte range)."""
        return self.left == 0 and self.right == 0

    @property
    def content_offset(self):
        return (self.top * self.size + self.left) * 3

    @property
    def content_bytes(self):
        return self.w * self.h * 3

    def as_dict(self):
        return {k: getattr(self, k) for k in ("src_w", "src_h", "size", "load_w", "load_h", "w", "h",
                                              "top", "bottom", "left", "right")}


def _coefs(dst_n, src_n):
    """OpenCV resize INTER_LINEAR 8U tables: source index and two Q11 coefficients per output index."""
    scale = 1.0 / (dst_n / src_n)
    ofs = np.empty(dst_n, np.int64)
    a = np.empty((dst_n, 2), np.int64)
    one, cs = np.float32(1.0), np.float32(COEF_SCALE)
    for d in range(dst_n):
        f = np.float32((d + 0.5) * scale - 0.5)
        s = int(math.floor(f))
        f = np.float32(f - np.float32(s))
        if s < 0:
            f, s = np.float32(0.0), 0
        if s >= src_n - 1:
            f, s = np.float32(0.0), src_n - 1
        a[d, 0] = int(np.rint(np.float32(np.float32(one - f) * cs)))
        a[d, 1] = int(np.rint(np.float32(f * cs)))
        ofs[d] = s
    return ofs, a


class _LinearPlan:
    def __init__(self, src_w, src_h, w, h):
        xo, xa = _coefs(w, src_w)
        yo, ya = _coefs(h, src_h)
        self.x0, self.x1 = xo, np.minimum(xo + 1, src_w - 1)
        self.xa0 = xa[:, 0].astype(np.int32)[None, :, None]
        self.xa1 = xa[:, 1].astype(np.int32)[None, :, None]
        self.y0, self.y1 = yo, np.minimum(yo + 1, src_h - 1)
        self.yb0 = ya[:, 0].astype(np.int32)[:, None, None]
        self.yb1 = ya[:, 1].astype(np.int32)[:, None, None]
        # rows actually read by the vertical pass: horizontal pass only on those
        self.rows = np.unique(np.concatenate([self.y0, self.y1]))
        pos = {int(r): i for i, r in enumerate(self.rows)}
        self.r0 = np.array([pos[int(r)] for r in self.y0])
        self.r1 = np.array([pos[int(r)] for r in self.y1])

    def run(self, img):
        S = img[self.rows].astype(np.int32)
        H = S[:, self.x0] * self.xa0 + S[:, self.x1] * self.xa1           # <= 255*2048, int32 safe
        H >>= 4
        out = ((self.yb0 * H[self.r0]) >> 16) + ((self.yb1 * H[self.r1]) >> 16)
        out += 2
        out >>= 2
        np.clip(out, 0, 255, out=out)
        return out.astype(np.uint8)


_PLANS = {}


def resize_linear_numpy(img, w, h):
    """cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR) for HxWxC uint8, bit-exact (numpy only)."""
    if int(w) > img.shape[1] or int(h) > img.shape[0]:
        # cv2 takes another path when enlarging (seen: 383 bytes differ at 320x240 -> 416x312); the car
        # camera (640x480) is never enlarged at 640/512/416, so only shrinking is replicated.
        raise NotImplementedError("numpy INTER_LINEAR replica only shrinks (%dx%d -> %dx%d)"
                                  % (img.shape[1], img.shape[0], w, h))
    key = (img.shape[1], img.shape[0], int(w), int(h))
    plan = _PLANS.get(key)
    if plan is None:
        plan = _PLANS[key] = _LinearPlan(img.shape[1], img.shape[0], int(w), int(h))
    return plan.run(img)


def _resize(img, w, h, backend):
    if (img.shape[1], img.shape[0]) == (w, h):
        return img
    if backend == "cv2":
        import cv2
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
    return resize_linear_numpy(img, w, h)


def resized_uint8(rgb, size, backend="numpy"):
    """(geometry, h x w x 3 uint8 content) of the ultralytics validator for this frame and size."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    g = LetterboxGeometry(rgb.shape[1], rgb.shape[0], size)
    im = _resize(rgb, g.load_w, g.load_h, backend)
    im = _resize(im, g.w, g.h, backend)
    return g, im


class DpuPrep:
    """Frame -> int8 content bytes for one (size, fix_point). Reuses tables between frames."""

    def __init__(self, size, fix_point=None, backend="auto"):
        self.size = int(size)
        self.fix_point = DEFAULT_FIX_POINT[self.size] if fix_point is None else int(fix_point)
        self.lut = quant_lut(self.fix_point)
        self._lut_u8 = np.ascontiguousarray(self.lut.view(np.uint8))   # same bytes, for cv2.LUT / take
        self.pad_q = int(self.lut[PAD_VALUE])
        self.backend = backend
        self._selftested = {}

    def _backend_for(self, rgb):
        if self.backend != "auto":
            return self.backend
        key = rgb.shape
        if key not in self._selftested:
            self._selftested[key] = "cv2" if selftest_cv2(rgb.shape[1], rgb.shape[0], self.size) else "numpy"
        return self._selftested[key]

    def geometry(self, src_w, src_h):
        return LetterboxGeometry(src_w, src_h, self.size)

    def content(self, rgb):
        """(geometry, int8 content as bytes (h*w*3), backend used)."""
        be = self._backend_for(np.asarray(rgb))
        g, im = resized_uint8(rgb, self.size, be)
        if be == "cv2":
            import cv2
            q = cv2.LUT(np.ascontiguousarray(im), self._lut_u8)
        else:
            q = self._lut_u8[im]          # fancy indexing: ~4x faster than take() here
        return g, q.tobytes(), be

    def full_tensor(self, rgb):
        """size x size x 3 int8 tensor (pad included): what the DPU input buffer must hold."""
        g, content, _ = self.content(rgb)
        return place_content(content, g, self.pad_q)


def place_content(content, g, pad_q):
    t = np.full((g.size, g.size, 3), pad_q, dtype=np.int8)
    t[g.top:g.top + g.h, g.left:g.left + g.w] = np.frombuffer(content, np.int8).reshape(g.h, g.w, 3)
    return t


def selftest_cv2(src_w, src_h, size, seed=20261103):
    """True if this machine's cv2.resize equals the numpy reference for this geometry (random image,
    plus a gradient). False when cv2 is missing or differs (then the numpy path is used)."""
    try:
        import cv2  # noqa: F401
    except Exception:
        return False
    rng = np.random.RandomState(seed)
    imgs = [rng.randint(0, 256, (src_h, src_w, 3)).astype(np.uint8)]
    y, x = np.mgrid[0:src_h, 0:src_w]
    imgs.append(np.stack([x % 256, y % 256, (x + y) % 256], -1).astype(np.uint8))
    for im in imgs:
        _, a = resized_uint8(im, size, "cv2")
        _, b = resized_uint8(im, size, "numpy")
        if a.shape != b.shape or not np.array_equal(a, b):
            return False
    return True


def crc32(b):
    return zlib.crc32(b) & 0xFFFFFFFF
