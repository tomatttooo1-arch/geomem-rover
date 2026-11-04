"""AR2NANO (nationals 2026-11-02, owner: the car screen is drawn on the Jetson Orin Nano, not by the PL).

NanoArScreen turns every answered remote_pl frame on the Nano (host/kv_link.py _RemoteFrameCall.ar_sink)
into exactly the hand-off record kv_server built on the KV260 (KvLinkServer -> ArPresenter.submit), and
drives the same presenter stack: host/kv_ar.ArPresenter -> tools/kv_link/ar_helper m7ar_present
--provider cpu --display drm (hw/m7/ar_overlay.cpp unchanged, on the Nano CPU; build: make nano).

Exactness by construction: the request is encoded with ALL extensions (labels, replay mark, sensor
stamp) and decoded with host.kv_link.decode_request, i.e. the very dict the KV260 decoded from the wire
(float32 world_camera, ROI rows, cleaned labels); the answer fields (status, voxel flags, ROI rows) are
the KV260's own answer. Age: Nano capture -> now (one clock), first_mono_ns = now.
AR2NANO2 (2026-11-02): wired into the car stack (host/ros_geo_producer.py --ar-nano, car_up CAR_AR_NANO=1,
nano_up.sh turns it on by itself when the KV bitstream has no AR core). The renderer is the byte-exact fast
twin of the core (m7ar_present --provider cpu, --cpu-impl fast by default). permit: PermitPoller reads the KV
TELEM permit block (STATUS bit 0 = the permit wire the PL AR core sampled) at 5 Hz; present = an answer with
the block available within the last PERMIT_FRESH_S, else the panel says DRV N/A. Colour main view: a
host.kv_colour.ColourSink on 127.0.0.1:47801 here, nano_colour.py sends to 127.0.0.1 (car_inner AR_NANO=1).
Off unless the caller creates it (geo_producer KvConnection.ar_sink); never fails a geometry frame.
"""
import os
import threading
import time

from host import kv_ar
from host import kv_link as kv


def snapshot_of(raw):
    """The KV260's ArPresenter.submit record, rebuilt on the Nano from one answered frame."""
    ident = raw["ident"]
    batch = ident.get("roi")
    labels = ident.get("labels") if batch is not None else None   # already wire-clean (set_next_roi)
    req = kv.encode_request(frame_id=ident["frame_id"], capture_ns=ident["capture_ns"], pose_ns=ident["pose_ns"],
                            identity=ident["identity"], width=raw["width"], height=raw["height"], nx=raw["nx"],
                            ny=raw["ny"], nz=raw["nz"], unit_f32=raw["unit_f32"], voxel=raw["voxel"],
                            origin=raw["origin"], inverse_k=raw["inverse_k"], world_camera=raw["world_camera"],
                            codes=raw["codes"], depth_crc=raw["depth_crc"], roi=batch, labels=labels,
                            replay=bool(ident.get("replay")), sensor_key=ident.get("sensor_key"))
    q = kv.decode_request(req, allow_age=True, allow_label=True, allow_replay=True, allow_stamp=True)
    resp = raw["resp"]
    nvox = q["nx"] * q["ny"] * q["nz"]
    roi_ans = resp.get("roi")
    return {"frame_id": q["frame_id"], "codes": q["codes"], "unit_bits": q["unit_bits"],
            "width": q["width"], "height": q["height"], "nx": q["nx"], "ny": q["ny"], "nz": q["nz"],
            "origin": q["origin"], "voxel": q["voxel"], "world_camera": q["world_camera"],
            "flags": resp["flags"][:nvox], "geometry_status": int(resp["status"]), "roi": q["roi"],
            "roi_status": roi_ans["roi_status"] if roi_ans is not None else None,
            "roi_rows": roi_ans["rows"] if roi_ans is not None else [],
            "nano_elapsed_ns": max(0, int(raw["sent_ns"]) - int(q["capture_ns"])),
            "labels": q["labels"], "replay": q["replay"], "first_mono_ns": time.monotonic_ns(),
            "capture_ns": q["capture_ns"], "sensor_stamp_ns": q["sensor_stamp_ns"]}


PERMIT_POLL_HZ = 5.0
PERMIT_FRESH_S = 1.0


class PermitPoller(object):
    """KV TELEM permit block -> (permit_present, permit_i) for the Nano renderer (AR2NANO2).

    The PL AR core sampled the permit block's permit_o wire once per frame; on the Nano the same bit
    arrives as TELEM permit STATUS bit 0 (host/kv_status.decode_permit "permit"). One poll every
    1/PERMIT_POLL_HZ s on its own KV link (read-only TELEM, like the hub and the wheel odometry);
    present only while the last good answer with the block available is at most PERMIT_FRESH_S old,
    so a dead link or a bitstream without the permit block shows DRV N/A, never a stale DRIVE OK."""

    def __init__(self, server, *, client_factory=None, hz=PERMIT_POLL_HZ, fresh_s=PERMIT_FRESH_S,
                 clock=time.monotonic):
        self.server, self.hz, self.fresh_s, self.clock = server, float(hz), float(fresh_s), clock
        self._factory = client_factory or (lambda srv: kv.KvTelemetryClient(srv, timeout_s=0.2,
                                                                            hello_timeout_s=1.0))
        self.counts = {"polls": 0, "ok": 0, "unavailable": 0, "errors": 0}
        self._last = None               # (t, available, on)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._client = None
        self._t = None

    def start(self):
        self._t = threading.Thread(target=self._loop, name="nano-ar-permit", daemon=True)
        self._t.start()
        return self

    def poll_once(self):
        self.counts["polls"] += 1
        try:
            if self._client is None:
                self._client = self._factory(self.server)
            tm = self._client.poll(want=kv.blocks_mask(["permit"]))
            b = tm.blocks.get("permit") or {}
            if b.get("available"):
                on = bool(b["status_bits"]["permit"])
                self.counts["ok"] += 1
            else:
                on = False
                self.counts["unavailable"] += 1
            with self._lock:
                self._last = (self.clock(), bool(b.get("available")), on)
        except Exception:               # noqa: BLE001 -- display only; next poll retries (new client)
            self.counts["errors"] += 1
            try:
                if self._client is not None:
                    self._client.close()
            except Exception:           # noqa: BLE001
                pass
            self._client = None

    def _loop(self):
        while not self._stop.is_set():
            t0 = self.clock()
            self.poll_once()
            self._stop.wait(max(0.0, 1.0 / self.hz - (self.clock() - t0)))

    def __call__(self):
        """(permit_present, permit_i) for the next message (kv_ar.ArPresenter permit_source)."""
        with self._lock:
            last = self._last
        if last is None or not last[1] or self.clock() - last[0] > self.fresh_s:
            return 0, 0
        return 1, 1 if last[2] else 0

    def close(self):
        self._stop.set()
        if self._client is not None:
            try:
                self._client.close()
            except Exception:           # noqa: BLE001
                pass


def helper_cmd(repo_root, *, display="drm", mode=None, modeline=None, out_dir=None, stale_ms=1001,
               refresh_ms=500, mini_scale=1, cpu_impl="fast"):
    helper = os.path.join(repo_root, "tools", "kv_link", "ar_helper", "build", "nano", "m7ar_present")
    cmd = [helper, "--provider", "cpu", "--display", display, "--stale-ms", str(stale_ms),
           "--refresh-ms", str(refresh_ms)]
    if cpu_impl != "fast":
        cmd += ["--cpu-impl", cpu_impl]
    if mini_scale != 1:
        cmd += ["--mini-scale", str(mini_scale)]
    if mode:
        cmd += ["--mode", mode]
    if modeline:
        cmd += ["--modeline", modeline]
    if out_dir:
        cmd += ["--event-log", os.path.join(out_dir, "ar_helper_events.jsonl")]
    return cmd


class NanoArScreen(object):
    """offer(raw) from the geometry thread (cheap: newest wins); a worker thread builds the record."""

    def __init__(self, presenter, permit=None, colour=None):
        self.ar = presenter
        self.permit, self.colour = permit, colour
        self.counts = {"offered": 0, "submitted": 0, "errors": 0}
        self._raw, self._cv, self._stop = None, threading.Condition(), False
        self._t = threading.Thread(target=self._loop, name="nano-ar", daemon=True)
        self._t.start()

    @classmethod
    def start(cls, repo_root, *, out_dir=None, max_age_ms=1000, tune_file=None, display="drm", mode="1024x600",
              modeline=None, kv_server=None, colour_port=47801, colour_tol_ms=17.0, mini_scale=2,
              cpu_impl="fast", permit=None, popen=None, fill="native"):
        """The KV260's competition AR configuration (kv_start.sh: --ar-colour main --ar-colour-port 47801
        --ar-colour-tol-ms 17 --ar-max-age-ms 1000 --ar-tune-file kv_ar_tune.json, kv_server's default
        --ar-mini-scale 2) on the Nano. colour_port None = no colour main view (depth main view);
        kv_server (host, port) = permit from the KV TELEM (None and no ``permit`` = DRV N/A)."""
        cmd = helper_cmd(repo_root, display=display, mode=mode, modeline=modeline, out_dir=out_dir,
                         stale_ms=max_age_ms + 1, mini_scale=mini_scale, cpu_impl=cpu_impl)
        if permit is None and kv_server is not None:
            permit = PermitPoller(kv_server).start()
        colour = None
        if colour_port is not None:
            from host.kv_colour import ColourSink
            colour = ColourSink(("127.0.0.1", int(colour_port)), tol_ns=int(colour_tol_ms * 1e6)).start()
        kw = {} if popen is None else {"popen": popen}
        if fill == "native":            # as the KV (kv_server --ar-fill native); no compiler -> numpy reference
            try:
                import tempfile
                from host.kv_ar_native import ArFillNative
                kw["fill_native"] = ArFillNative.build(tempfile.mkdtemp(prefix="nano_ar_fill_"))
            except Exception:           # noqa: BLE001 -- the numpy fill paints the same depth
                pass
        try:
            pres = kv_ar.ArPresenter(cmd, source="live", max_age_ms=max_age_ms, permit_present=0, out_dir=out_dir,
                                     tune_file=tune_file, colour=colour, colour_main=colour is not None,
                                     mini_scale=mini_scale, permit_source=permit, **kw)
        except Exception:
            if colour is not None:
                colour.stop()
            if permit is not None and hasattr(permit, "close"):
                permit.close()
            raise
        return cls(pres, permit=permit, colour=colour)

    def offer(self, raw):
        with self._cv:
            self._raw = raw
            self.counts["offered"] += 1
            self._cv.notify()

    def _loop(self):
        while True:
            with self._cv:
                while self._raw is None and not self._stop:
                    self._cv.wait()
                if self._stop:
                    return
                raw, self._raw = self._raw, None
            try:
                self.ar.submit(snapshot_of(raw))
                self.counts["submitted"] += 1
            except Exception:       # noqa: BLE001 -- display only
                self.counts["errors"] += 1

    def close(self):
        with self._cv:
            self._stop = True
            self._cv.notify()
        self.ar.close()
        if self.colour is not None:
            self.colour.stop()
        if self.permit is not None and hasattr(self.permit, "close"):
            self.permit.close()
