"""DMA buffer providers of the Python PL device layer (P9, 2026-09-27).

Runs DevMemPlDevice.open against a mocked /sys + /dev tree, a sparse fake
/dev/mem, a fake DMA_HEAP_IOCTL_ALLOC and a fake pagemap. Every rule is a
check function; each check is run on the real class and on a deliberately
broken variant that must make it fail
(docs/design/PL_DMA_PROVIDERS_20260927.md section 8).
"""

from collections import Counter
import mmap
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

import numpy as np

from host import m7_pl_device as pd
from host.m7_backend import M7BackendSelector, PlBackendUnavailable, _PlCore
from host.m7_pl_device import (DMA_HEAP_IOCTL_ALLOC, DMA_LIMIT, FRAME, FRAME_REGS, PAGE,
                               DevMemPlDevice, MockPlDevice, PlDeviceError,
                               PlDeviceUnavailable)

MEM_BYTES = 0xA0030000          # covers both register windows (sparse file)
UDMABUF_PA = 0x70000000


class RecordingCache:
    def __init__(self):
        self.log = []

    def clean(self, va, nbytes):
        self.log.append(("clean", va, nbytes))

    def clean_invalidate(self, va, nbytes):
        self.log.append(("clean_invalidate", va, nbytes))

    def barrier(self):
        self.log.append(("barrier",))


class FakeOps:
    """ioctl / pagemap / cache helper stand-ins.

    pfn_mode: ok | hidden (non-root: PFN reads 0) | gap (page n/2 of a
    multi-page buffer is displaced) | high (above the 2 GiB DMA window).
    """

    def __init__(self, tmp, pfn_mode="ok", cache_ok=True):
        self.tmp, self.pfn_mode, self.cache_ok = Path(tmp), pfn_mode, cache_ok
        self.next_pfn = (DMA_LIMIT // PAGE) if pfn_mode == "high" else 0x40000
        self.cache = RecordingCache()
        self.allocs = []

    def ioctl(self, fd, request, buf):
        if request != DMA_HEAP_IOCTL_ALLOC:
            raise OSError(22, "fake: unexpected ioctl 0x%x" % request)
        length, _, flags, _ = pd._HEAP_ALLOC.unpack(bytes(buf))
        path = self.tmp / ("dmabuf%d" % len(self.allocs))
        with open(path, "wb") as fp:
            fp.truncate(length)
        dfd = os.open(str(path), os.O_RDWR)
        os.unlink(str(path))
        buf[:] = pd._HEAP_ALLOC.pack(length, dfd, flags, 0)
        self.allocs.append(length)

    def pagemap(self, va, npages):
        base = self.next_pfn
        self.next_pfn += npages + 16
        entries = []
        for k in range(npages):
            pfn = base + k
            if self.pfn_mode == "hidden":
                pfn = 0
            if self.pfn_mode == "gap" and npages > 1 and k == npages // 2:
                pfn += 1
            entries.append((1 << 63) | pfn)
        return entries

    def cache_ops(self):
        if not self.cache_ok:
            raise OSError("fake: no cache helper")
        return self.cache


class Board:
    """A mocked /sys/class/u-dma-buf, /dev and /dev/mem."""

    def __init__(self, tmp, *, udmabuf_bytes=0, udmabuf_pa=UDMABUF_PA, heaps=(),
                 pfn_mode="ok", cache_ok=True):
        self.root = Path(tempfile.mkdtemp(dir=tmp))
        self.sysfs = self.root / "sys/class/u-dma-buf"
        self.dev = self.root / "dev"
        (self.dev / "dma_heap").mkdir(parents=True)
        self.sysfs.mkdir(parents=True)
        self.mem = self.root / "mem"
        with open(self.mem, "wb") as fp:
            fp.truncate(MEM_BYTES)
        if udmabuf_bytes:
            node = self.sysfs / "udmabuf0"
            node.mkdir()
            (node / "phys_addr").write_text("0x%016x\n" % udmabuf_pa)
            (node / "size").write_text("%d\n" % udmabuf_bytes)
            with open(self.dev / "udmabuf0", "wb") as fp:
                fp.truncate(udmabuf_bytes)
        for heap in heaps:
            (self.dev / "dma_heap" / heap).write_bytes(b"")
        self.ops = FakeOps(self.root, pfn_mode=pfn_mode, cache_ok=cache_ok)

    def open(self, cls=DevMemPlDevice, env=None, **kw):
        return cls.open(sysfs_root=str(self.sysfs), dev_root=str(self.dev),
                        mem_path=str(self.mem), env={} if env is None else env,
                        ops=self.ops, **kw)


def expect(cond, message):
    if not cond:
        raise AssertionError(message)


def expect_unavailable(fn, *needles):
    try:
        device = fn()
    except PlDeviceUnavailable as exc:
        for needle in needles:
            expect(needle in str(exc), "%r not in %s" % (needle, exc))
        return exc
    device.close()
    raise AssertionError("open succeeded with provider %s" % device.provider)


# ---------------------------------------------------------------- checks
def check_auto_order(tmp, cls):
    both = Board(tmp, udmabuf_bytes=1 << 20, heaps=("reserved",))
    d = both.open(cls)
    try:
        expect(d.provider == "udmabuf", "auto with udmabuf+heap chose %s" % d.provider)
        expect(d.provider_counts["provider_tried:dmaheap"] == 0, "auto tried dmaheap after udmabuf ok")
    finally:
        d.close()
    heap_only = Board(tmp, heaps=("reserved",))
    d = heap_only.open(cls)
    try:
        expect(d.provider == "dmaheap", "auto with only a heap chose %s" % d.provider)
        expect(d.describe()["heap"] == "reserved", "heap name not reported")
        expect(d.provider_attempts[0][0] == "udmabuf" and "u-dma-buf" in d.provider_attempts[0][1],
               "udmabuf failure not recorded first: %s" % d.provider_attempts)
        expect(d.provider_counts["provider_failed:udmabuf"] == 1
               and d.provider_counts["provider_selected:dmaheap"] == 1, dict(d.provider_counts))
    finally:
        d.close()


def check_auto_skips_physmem(tmp, cls):
    board = Board(tmp)
    exc = expect_unavailable(lambda: board.open(cls, phys_base=0x60000000, phys_size=0x100000),
                             "no DMA buffer provider", "udmabuf[", "dmaheap[",
                             "physmem[not tried in auto mode")
    expect(exc.counts["provider_tried:physmem"] == 0, "auto tried physmem")
    expect(exc.counts["provider_failed:udmabuf"] == 1 and exc.counts["provider_failed:dmaheap"] == 1,
           dict(exc.counts))


def check_explicit(tmp, cls):
    both = Board(tmp, udmabuf_bytes=1 << 20, heaps=("linux,cma",))
    d = both.open(cls, provider="dmaheap")
    try:
        expect(d.provider == "dmaheap", "explicit dmaheap gave %s" % d.provider)
    finally:
        d.close()
    heap_only = Board(tmp, heaps=("linux,cma",))
    exc = expect_unavailable(lambda: heap_only.open(cls, provider="udmabuf"), "udmabuf[no ")
    expect(exc.counts["provider_tried:dmaheap"] == 0, "explicit udmabuf fell through to dmaheap")
    d = Board(tmp, udmabuf_bytes=1 << 20).open(cls, provider="physmem", phys_base=0x60000000,
                                               phys_size=0x40000)
    try:
        expect(d.provider == "physmem" and d.describe()["region_pa"] == "0x60000000",
               d.describe())
    finally:
        d.close()


def check_env(tmp, cls):
    board = Board(tmp, udmabuf_bytes=1 << 20, heaps=("reserved",))
    d = board.open(cls, env={"FPGA_M7_DMA_PROVIDER": "dmaheap", "FPGA_M7_DMA_HEAP": "reserved"})
    try:
        expect(d.provider == "dmaheap", "env provider ignored: %s" % d.provider)
    finally:
        d.close()
    d = board.open(cls, env={"FPGA_M7_DMA_PROVIDER": "dmaheap"}, provider="udmabuf")
    try:
        expect(d.provider == "udmabuf", "argument did not override env: %s" % d.provider)
    finally:
        d.close()
    d = board.open(cls, env={"FPGA_M7_DMA_PROVIDER": "physmem", "FPGA_M7_PHYS_BASE": "0x60000000",
                             "FPGA_M7_PHYS_SIZE": "40000"})
    try:
        expect(d.provider == "physmem" and d.describe()["region_size"] == 0x40000, d.describe())
    finally:
        d.close()


def check_no_fallback(tmp, factory_for):
    board = Board(tmp)
    selector = M7BackendSelector(pl_device_factory=factory_for(board))
    try:
        backend = selector.frame_backend("pl")
    except PlBackendUnavailable as exc:
        expect("no DMA buffer provider" in str(exc), str(exc))
        expect(selector.counts["pl_unavailable:frame"] == 1, dict(selector.counts))
        expect("selected:frame:cpu" not in selector.counts, "CPU backend selected")
        return
    raise AssertionError("backend='pl' without any DMA provider returned %s"
                         % getattr(backend, "backend_name", backend))


def check_pfn_hidden(tmp, cls):
    board = Board(tmp, heaps=("reserved",), pfn_mode="hidden")
    exc = expect_unavailable(lambda: board.open(cls), "PFN hidden (need root/CAP_SYS_ADMIN)")
    expect(exc.counts["provider_failed:dmaheap"] == 1, dict(exc.counts))


def check_contiguity(tmp, cls):
    board = Board(tmp, heaps=("reserved",), pfn_mode="gap")
    d = board.open(cls)   # the one-page probe cannot show a gap
    try:
        try:
            d.alloc(3 * PAGE)
        except PlDeviceError as exc:
            expect("not physically contiguous" in str(exc), str(exc))
            expect(d.provider_counts["alloc_failed"] == 1, dict(d.provider_counts))
            expect(d.describe()["live_buffers"] == 0, "failed buffer left live")
            return
        raise AssertionError("non-contiguous dma-heap buffer accepted")
    finally:
        d.close()


def check_limit(tmp, cls):
    high_heap = Board(tmp, heaps=("reserved",), pfn_mode="high")
    expect_unavailable(lambda: high_heap.open(cls), "outside the PL DMA window")
    high_udmabuf = Board(tmp, udmabuf_bytes=1 << 20, udmabuf_pa=DMA_LIMIT - (1 << 19))
    expect_unavailable(lambda: high_udmabuf.open(cls, provider="udmabuf"), "outside the PL DMA window")
    expect_unavailable(lambda: Board(tmp).open(cls, provider="physmem", phys_base=DMA_LIMIT - PAGE,
                                               phys_size=2 * PAGE), "outside the PL DMA window")


def check_cache(tmp, cls):
    board = Board(tmp, heaps=("reserved",))
    d = board.open(cls)
    try:
        log = board.ops.cache.log
        buf = d.alloc(10000)
        expect(buf.cached and buf.size == 3 * PAGE, (buf.cached, buf.size))
        expect(log[-1] == ("clean", buf.va, buf.size), "zeroing not cleaned: %s" % log[-1:])
        n = len(log)
        d.to_device(buf)
        expect(log[n:] == [("clean", buf.va, buf.size)], "to_device did not clean: %s" % log[n:])
        n = len(log)
        d.from_device(buf)
        expect(log[n:] == [("clean_invalidate", buf.va, buf.size)],
               "from_device did not clean+invalidate: %s" % log[n:])
        expect(d.describe()["cached"] is True, d.describe())
    finally:
        d.close()
    no_helper = Board(tmp, heaps=("reserved",), cache_ok=False)
    expect_unavailable(lambda: no_helper.open(cls), "needs cache maintenance")


def check_recycle(tmp, cls):
    board = Board(tmp, udmabuf_bytes=16 * PAGE)
    d = board.open(cls)
    try:
        core = _PlCore(d, FRAME, FRAME_REGS, timeout_s=1.0, counts=Counter())
        flags = core.buffer("flags", PAGE)
        for pages in (1, 2, 3, 4, 5, 6, 7, 8, 8, 8):   # frame grows, then stays
            try:
                depth = core.buffer("depth", pages * PAGE)
            except PlDeviceError as exc:
                raise AssertionError("16-page region exhausted while one 8-page frame "
                                     "buffer grew: %s" % exc)
            expect(depth.size >= pages * PAGE, "short buffer")
        expect(core.buffer("depth", 8 * PAGE) is depth, "same size not reused")
        expect(flags.pa != depth.pa and core.buffer("flags", PAGE) is flags, "flags buffer lost")
        live = d.describe()["live_bytes"]
        expect(live == 9 * PAGE, "live bytes %d after growth (expected 9 pages)" % live)
        # Buffer arrays alias the region exactly at pa - region_pa.
        depth.array[:4] = [1, 2, 3, 4]
        with open(board.dev / "udmabuf0", "rb") as fp:
            fp.seek(depth.pa - UDMABUF_PA)
            expect(fp.read(4) == bytes([1, 2, 3, 4]), "buffer does not alias its pa offset")
        d.release(depth)
        d.release(flags)
        expect(d._buffers.region.free == [(0, 16 * PAGE)], "freed blocks did not coalesce: %s"
               % d._buffers.region.free)
        try:
            d.release(flags)
        except PlDeviceError as exc:
            expect("double release" in str(exc), str(exc))
        else:
            raise AssertionError("double release accepted")
        try:
            d.alloc(17 * PAGE)
        except PlDeviceError as exc:
            expect("region exhausted" in str(exc) and "largest free block %d" % (16 * PAGE) in str(exc),
                   str(exc))
        else:
            raise AssertionError("over-size allocation accepted")
    finally:
        d.close()
    heap = Board(tmp, heaps=("reserved",))
    d = heap.open(cls)
    try:
        bufs = [d.alloc(PAGE) for _ in range(3)]
        for b in bufs:
            d.release(b)
        expect(d.describe()["live_buffers"] == 0, "dma-heap buffers not released")
    finally:
        d.close()


# ---------------------------------------------------------------- broken variants
class ReversedOrder(DevMemPlDevice):
    AUTO_ORDER = ("dmaheap", "udmabuf")


class AutoTriesPhysmem(DevMemPlDevice):
    AUTO_ORDER = ("udmabuf", "dmaheap", "physmem")


class IgnoresExplicit(DevMemPlDevice):
    @classmethod
    def _order(cls, provider):
        return cls.AUTO_ORDER


class ExplicitFallsBack(DevMemPlDevice):
    @classmethod
    def _order(cls, provider):
        if provider == "auto":
            return cls.AUTO_ORDER
        return (provider,) + tuple(p for p in pd.PROVIDERS if p != provider)


class IgnoresEnv(DevMemPlDevice):
    @classmethod
    def _config(cls, env, **given):
        return super()._config({}, **given)


class NoPfnCheckHeap(pd._HeapProvider):
    def physical(self, va, nbytes):
        entries = self.ops.pagemap(va, nbytes // PAGE)
        return (entries[0] & pd._PFN_MASK) * PAGE


class NoContiguityHeap(pd._HeapProvider):
    def physical(self, va, nbytes):
        self.ops.pagemap(va, nbytes // PAGE)
        return super().physical(va, PAGE)


class NoCleanHeap(pd._HeapProvider):
    def to_device(self, buf):
        pass


class NoInvalidateHeap(pd._HeapProvider):
    def from_device(self, buf):
        pass


class BumpRegion(pd._Region):
    def give_back(self, off):
        pass   # the pre-P9 behaviour: carve forward, never reuse


class BumpRegionProvider(pd._RegionProvider):
    region_cls = BumpRegion


def heap_variant(provider_cls):
    return type("Device" + provider_cls.__name__, (DevMemPlDevice,),
                {"heap_provider_cls": provider_cls})


def strict_factory(board):
    return lambda: board.open()


def silent_fallback_factory(board):
    def factory():
        try:
            return board.open()
        except PlDeviceUnavailable:
            return MockPlDevice()
    return factory


# ---------------------------------------------------------------- tests
class PlDmaProviderTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="pl-dma-")
        self.addCleanup(self._tmp.cleanup)
        self.tmp = self._tmp.name

    def run_pair(self, check, good, bad):
        check(self.tmp, good)
        with self.assertRaises(AssertionError):
            check(self.tmp, bad)

    def test_auto_order_udmabuf_then_dmaheap(self):
        self.run_pair(check_auto_order, DevMemPlDevice, ReversedOrder)

    def test_auto_does_not_try_physmem_like_cpp(self):
        self.run_pair(check_auto_skips_physmem, DevMemPlDevice, AutoTriesPhysmem)

    def test_explicit_provider_is_the_only_one_tried(self):
        self.run_pair(check_explicit, DevMemPlDevice, IgnoresExplicit)
        with self.assertRaises(AssertionError):
            check_explicit(self.tmp, ExplicitFallsBack)

    def test_env_selects_and_argument_overrides(self):
        self.run_pair(check_env, DevMemPlDevice, IgnoresEnv)

    def test_no_provider_raises_counts_and_never_falls_back(self):
        self.run_pair(check_no_fallback, strict_factory, silent_fallback_factory)

    def test_dmaheap_needs_real_pfn(self):
        self.run_pair(check_pfn_hidden, DevMemPlDevice, heap_variant(NoPfnCheckHeap))

    def test_dmaheap_needs_contiguity(self):
        self.run_pair(check_contiguity, DevMemPlDevice, heap_variant(NoContiguityHeap))

    def test_buffers_stay_in_low_2gib(self):
        check_limit(self.tmp, DevMemPlDevice)
        with mock.patch.object(pd, "_check_limit", lambda *a: None):
            with self.assertRaises(AssertionError):
                check_limit(self.tmp, DevMemPlDevice)

    def test_dmaheap_cache_maintenance(self):
        self.run_pair(check_cache, DevMemPlDevice, heap_variant(NoCleanHeap))
        with self.assertRaises(AssertionError):
            check_cache(self.tmp, heap_variant(NoInvalidateHeap))

    def test_buffers_are_reused_and_recycled(self):
        bump = type("BumpDevice", (DevMemPlDevice,), {"region_provider_cls": BumpRegionProvider})
        self.run_pair(check_recycle, DevMemPlDevice, bump)

    def test_unknown_provider_is_a_config_error(self):
        with self.assertRaises(ValueError):
            Board(self.tmp).open(provider="cma")

    def test_udmabuf_region_is_uncached_and_described(self):
        board = Board(self.tmp, udmabuf_bytes=1 << 20)
        d = board.open()
        try:
            info = d.describe()
            self.assertEqual((info["provider"], info["cached"], info["region_pa"]),
                             ("udmabuf", False, "0x%x" % UDMABUF_PA))
            self.assertEqual(info["barrier"], "dsb")   # fake helper present
            self.assertEqual(d.kind, "devmem")
            buf = d.alloc(100)
            d.to_device(buf)
            self.assertEqual(board.ops.cache.log[-1], ("barrier",))
            self.assertFalse(buf.cached)
        finally:
            d.close()

    @unittest.skipIf(shutil.which("aarch64-linux-gnu-gcc") is None, "no aarch64 cross gcc")
    def test_cache_helper_builds_for_aarch64_with_dc_ops(self):
        """The helper the board compiles: same source and flags, cross-built here."""
        src, lib = Path(self.tmp) / "c.c", Path(self.tmp) / "c.so"
        src.write_text(pd._CACHE_SOURCE)
        subprocess.run(["aarch64-linux-gnu-gcc", *pd._CACHE_CFLAGS, str(src), "-o", str(lib)],
                       check=True, capture_output=True, text=True)
        dis = subprocess.run(["aarch64-linux-gnu-objdump", "-d", str(lib)], check=True,
                             capture_output=True, text=True).stdout
        for symbol, op in (("m7pl_dcache_clean", "dc\tcvac"),
                           ("m7pl_dcache_clean_invalidate", "dc\tcivac"),
                           ("m7pl_barrier", "dsb\tsy")):
            body = dis.split("<%s>:" % symbol)[1].split("\n\n")[0]
            self.assertIn(op, body, symbol)

    def test_cache_helper_refuses_non_aarch64(self):
        if platform.machine() in ("aarch64", "arm64"):
            self.skipTest("aarch64 host")
        with self.assertRaisesRegex(OSError, "aarch64-only"):
            pd.SystemOps().cache_ops()

    @unittest.skipIf(os.geteuid() == 0, "root sees real PFNs")
    def test_real_pagemap_hides_pfn_without_root(self):
        """Positive control for the 'PFN hidden' path on the real kernel interface."""
        region = mmap.mmap(-1, 2 * PAGE)
        try:
            region[:] = b"\x01" * (2 * PAGE)
            view = np.frombuffer(region, dtype=np.uint8)
            va = view.ctypes.data
            entries = pd.SystemOps().pagemap(va, 2)
            del view
            self.assertTrue(all(e >> 63 for e in entries), [hex(e) for e in entries])
            self.assertTrue(all(e & pd._PFN_MASK == 0 for e in entries), [hex(e) for e in entries])
            heap = pd._HeapProvider("real", None, pd.SystemOps(), RecordingCache())
            with self.assertRaisesRegex(PlDeviceError, "PFN hidden"):
                heap.physical(va, 2 * PAGE)
        finally:
            region.close()


if __name__ == "__main__":
    unittest.main()
