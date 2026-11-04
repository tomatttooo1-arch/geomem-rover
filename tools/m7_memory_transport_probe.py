"""Bounded, local cross-process M7 -> M4 acceptance probe; not a deployment service.

The client sends synthetic raw RGB/depth/calibration/pose and later masks through
an owner-only Unix socket. The server runs the native M7 CPU reference, M7 host
pipeline, the production M7/M4 adapter, and M4's real persistent store.
"""

import argparse
from dataclasses import asdict
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import time
import zipfile

import numpy as np

from host.inference_service import MAX_BYTES, receive_message, send_message


ROOT = Path(__file__).resolve().parents[1]
MAX_METADATA = 16 * 1024
READ_SECONDS = 3.0


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False,
                      separators=(",", ":"), allow_nan=False).encode("utf8")


def sha(body):
    return hashlib.sha256(body).hexdigest()


def file_sha(path):
    path = Path(path)
    return sha(path.read_bytes()) if path.exists() else None


def write_new(path, body):
    path = Path(path)
    with path.open("xb") as stream:
        stream.write(body)
    return sha(body)


def write_json(path, value):
    return write_new(path, json.dumps(value, ensure_ascii=False, indent=2,
                                      allow_nan=False).encode("utf8") + b"\n")


def encode_request(metadata, arrays=None):
    """No pickle; the NPZ member list and expanded size are checked on receive."""
    arrays = arrays or {}
    header = canonical(metadata)
    if len(header) > MAX_METADATA:
        raise ValueError("probe metadata exceeds 16 KiB")
    output = io.BytesIO()
    np.savez(output, metadata=np.array(header.decode("utf8")), **arrays)
    body = output.getvalue()
    if not 0 < len(body) <= MAX_BYTES:
        raise ValueError("probe request exceeds 32 MiB")
    return body


def decode_request(body):
    if not 0 < len(body) <= MAX_BYTES:
        raise ValueError("probe request exceeds 32 MiB")
    with zipfile.ZipFile(io.BytesIO(body)) as archive:
        names = archive.namelist()
        if (len(names) != len(set(names)) or len(names) > 8 or
                sum(info.file_size for info in archive.infolist()) > MAX_BYTES or
                not all(name.endswith(".npy") and "/" not in name for name in names)):
            raise ValueError("invalid or expanded probe archive")
        members = {name[:-4] for name in names}
    with np.load(io.BytesIO(body), allow_pickle=False) as loaded:
        if "metadata" not in members:
            raise ValueError("missing probe metadata")
        raw = loaded["metadata"]
        if raw.shape != () or raw.dtype.kind != "U":
            raise ValueError("invalid probe metadata array")
        header = str(raw.item()).encode("utf8")
        if len(header) > MAX_METADATA:
            raise ValueError("probe metadata exceeds 16 KiB")
        metadata = json.loads(header)
        arrays = {name: loaded[name].copy() for name in members - {"metadata"}}
    if not isinstance(metadata, dict):
        raise ValueError("probe metadata must be an object")
    expected = {"geometry": {"rgb", "depth", "K", "T"},
                "semantic": {"mask"}, "revision": set(),
                "query": set(), "validate": set(), "shutdown": set()}.get(metadata.get("kind"))
    if expected is None or set(arrays) != expected:
        raise ValueError("probe archive members do not match request kind")
    return metadata, arrays


def identity_from(row):
    from host.m7_pipeline import CaptureIdentity
    if not isinstance(row, dict) or set(row) != set(CaptureIdentity.__dataclass_fields__):
        raise ValueError("complete M7 capture identity required")
    return CaptureIdentity(**row)


def reference(path):
    path = Path(path)
    return {"path": str(path.resolve()), "sha256": file_sha(path)}


class ProbeService:
    def __init__(self, store, library):
        from host.m7_native import NativeM7
        from host.m7_pipeline import GridSpec, M7Pipeline
        from perception.offline_memory import OfflineMemory
        self.store = Path(store)
        self.memory = OfflineMemory.load(self.store) if self.store.exists() else OfflineMemory()
        self.pipeline = M7Pipeline(NativeM7(library),
                                   GridSpec(origin_m=(-2.0, -2.0, -1.0), voxel_m=0.25,
                                            shape=(32, 32, 32)))
        self.frames = {}
        self.outputs = {}

    def handle(self, metadata, arrays):
        from host.m7_pipeline import DepthFrame, PoseAtCapture, SemanticPacket
        from host.m7_memory import build_m4_document, apply_map_revision
        kind = metadata["kind"]
        if kind == "geometry":
            identity = identity_from(metadata["identity"])
            rgb, depth, k, t = (arrays[x] for x in ("rgb", "depth", "K", "T"))
            if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or
                    depth.dtype != np.float32 or depth.shape != rgb.shape[:2] or
                    k.shape != (3, 3) or t.shape != (4, 4) or
                    sha(rgb.tobytes()) != metadata["capture"]["image_sha256"]):
                raise ValueError("raw RGB-D/calibration/acquisition mismatch")
            pose = PoseAtCapture(stamp_ns=metadata["pose_stamp_ns"],
                                 clock_id=identity.clock_id,
                                 local_frame_id=identity.local_frame_id,
                                 pose_epoch=identity.pose_epoch,
                                 valid=metadata["pose_valid"], T_local_camera=t)
            frame = DepthFrame(identity, depth, k, pose)
            result = self.pipeline.ingest_geometry(frame, now_ns=metadata["now_ns"],
                                                   now_clock_id=identity.clock_id)
            if result.navigation.valid:
                self.frames[identity] = (result, metadata["capture"], metadata["anchor"])
            return {"valid": result.navigation.valid, "reason": result.navigation.reason,
                    "frame_token": result.navigation.frame_token,
                    "identity": asdict(identity),
                    "accepted_rays": result.navigation.payload.get("valid_grid_samples")}
        if kind == "semantic":
            identity = identity_from(metadata["identity"])
            if identity not in self.frames:
                raise ValueError("semantic frame has no accepted geometry")
            geometry, capture, anchor = self.frames[identity]
            mask = arrays["mask"]
            if mask.dtype != np.bool_ or mask.ndim != 2:
                raise ValueError("semantic mask must be bool HxW")
            packet = SemanticPacket(identity, metadata["frame_token"],
                                    metadata["object_id"], mask)
            output = self.pipeline.ingest_semantic(packet, now_ns=metadata["now_ns"],
                                                   now_clock_id=identity.clock_id)
            if not output.valid:
                return {"valid": False, "reason": output.reason, "imported": False}
            document, audit = build_m4_document(
                self.pipeline, geometry, {metadata["object_id"]: output}, capture=capture,
                detection=metadata["detection"], anchor=anchor,
                now_ns=metadata["now_ns"], clock_id=identity.clock_id)
            audit_path = self.store.parent / (metadata["request_id"] + ".m4-audit.json")
            write_json(audit_path, audit)
            if document is None:
                return {"valid": True, "reason": output.reason, "imported": False,
                        "audit": audit, "audit_path": str(audit_path)}
            document_path = self.store.parent / (metadata["request_id"] + ".m4-input.json")
            write_json(document_path, document)
            candidate = deepcopy(self.memory)
            imported = candidate.import_document(document)
            candidate.save(self.store)
            self.memory = candidate
            self.outputs[metadata["save_as"]] = output
            return {"valid": True, "reason": output.reason, "imported": True,
                    "import_result": imported, "audit": audit,
                    "audit_path": str(audit_path), "document_path": str(document_path),
                    "document_sha256": sha(canonical(document)),
                    "frame_token": output.frame_token}
        if kind == "revision":
            revision_path = self.store.parent / (metadata["request_id"] + ".revision.json")
            write_json(revision_path, metadata["document"])
            result = apply_map_revision(self.memory, self.pipeline, metadata["document"],
                                        store_path=self.store)
            return {"revision": result, "revision_path": str(revision_path)}
        if kind == "query":
            return {"query": self.memory.query()}
        if kind == "validate":
            output = self.outputs[metadata["save_as"]]
            checked = output.validate_at(self.pipeline, metadata["now_ns"], metadata["clock_id"])
            return {"valid": checked.valid, "reason": checked.reason,
                    "frame_token": checked.frame_token}
        if kind == "shutdown":
            return {"stopping": True}
        raise ValueError("unsupported request kind")


def serve(socket_path, store, library, log_path):
    path = Path(socket_path)
    if path.exists() or path.is_symlink():
        raise FileExistsError("probe socket already exists")
    service = ProbeService(store, library)
    stop = False
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        inode = path.stat().st_ino
        os.chmod(path, 0o600)
        listener.listen(1)
        print(json.dumps({"ready": True, "pid": os.getpid(), "socket": str(path)}), flush=True)
        try:
            with Path(log_path).open("x", encoding="utf8") as log:
                while not stop:
                    connection, _ = listener.accept()
                    with connection:
                        connection.settimeout(READ_SECONDS)
                        while not stop:
                            before = file_sha(store)
                            try:
                                raw = receive_message(connection,
                                                      deadline=time.monotonic()+READ_SECONDS)
                            except (ConnectionError, TimeoutError, OSError, ValueError) as exc:
                                log.write(json.dumps({"event": "read_end", "pid": os.getpid(),
                                                      "error": type(exc).__name__,
                                                      "detail": str(exc), "store_sha256": before})+"\n")
                                log.flush()
                                break
                            request_sha = sha(raw)
                            metadata = {}
                            try:
                                metadata, arrays = decode_request(raw)
                                result = service.handle(metadata, arrays)
                                reply = {"ok": True, "request_id": metadata["request_id"],
                                         "server_pid": os.getpid(), "result": result,
                                         "store_sha256": file_sha(store)}
                            except Exception as exc:
                                reply = {"ok": False, "request_id": metadata.get("request_id"),
                                         "server_pid": os.getpid(), "error": type(exc).__name__,
                                         "reason": str(exc), "store_sha256": file_sha(store)}
                            payload = canonical(reply)
                            row = {"event": "request", "request_id": metadata.get("request_id"),
                                   "kind": metadata.get("kind"), "pid": os.getpid(),
                                   "request_sha256": request_sha, "reply_sha256": sha(payload),
                                   "ok": reply["ok"], "store_before": before,
                                   "store_after": file_sha(store),
                                   "ack_dropped": bool(metadata.get("drop_ack_after_save"))}
                            log.write(json.dumps(row, ensure_ascii=False)+"\n")
                            log.flush()
                            os.fsync(log.fileno())
                            if metadata.get("drop_ack_after_save"):
                                break
                            try:
                                send_message(connection, payload)
                            except OSError:
                                break
                            stop = metadata.get("kind") == "shutdown"
        finally:
            if path.exists() and path.stat().st_ino == inode:
                path.unlink()


class ProbeClient:
    def __init__(self, path, directory):
        self.path, self.directory = str(path), Path(directory)
        self.stream = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.stream.settimeout(READ_SECONDS)
        self.stream.connect(self.path)
        self.rows = []

    def close(self):
        self.stream.close()

    def request(self, request_id, kind, *, arrays=None, drop_ack=False, **fields):
        metadata = {"request_id": request_id, "kind": kind, **fields}
        if drop_ack:
            metadata["drop_ack_after_save"] = True
        body = encode_request(metadata, arrays)
        raw = struct.pack("!I", len(body)) + body
        raw_path = self.directory / (request_id + ".request.bin")
        write_new(raw_path, raw)
        row = {"request_id": request_id, "kind": kind, "client_pid": os.getpid(),
               "raw_path": str(raw_path), "raw_sha256": sha(raw),
               "payload_sha256": sha(body),
               "store_before_sha256": file_sha(self.directory / "memory.json")}
        self.rows.append(row)
        send_message(self.stream, body)
        try:
            reply_body = receive_message(self.stream, deadline=time.monotonic()+READ_SECONDS)
        except (ConnectionError, OSError, TimeoutError) as exc:
            if not drop_ack:
                raise
            row["ack_error"] = type(exc).__name__ + ": " + str(exc)
            row["store_sha256"] = file_sha(self.directory / "memory.json")
            return None
        if drop_ack:
            raise AssertionError("ACK was expected to be deliberately lost")
        ack_path = self.directory / (request_id + ".ack.bin")
        write_new(ack_path, reply_body)
        reply = json.loads(reply_body)
        row.update(ack_path=str(ack_path), ack_sha256=sha(reply_body),
                   server_pid=reply.get("server_pid"),
                   store_sha256=reply.get("store_sha256"))
        if reply.get("request_id") != request_id:
            raise AssertionError("ACK belongs to another request")
        if reply.get("store_sha256") != file_sha(self.directory / "memory.json"):
            raise AssertionError("ACK store SHA256 differs from persisted bytes")
        return reply


def expect_ok(reply, *, valid=None):
    if reply is None or not reply.get("ok"):
        raise AssertionError("expected successful ACK: " + repr(reply))
    if valid is not None and reply["result"].get("valid") is not valid:
        raise AssertionError("unexpected output validity: " + repr(reply))
    return reply["result"]


def expect_rejected(reply):
    if reply is None or reply.get("ok"):
        raise AssertionError("expected rejection: " + repr(reply))


def array_file(path, array):
    out = io.BytesIO()
    np.save(out, array, allow_pickle=False)
    write_new(path, out.getvalue())
    return reference(path)


def frame_fixture(out, label, *, instance, frame_id, acquired_ns,
                  pose_epoch="pose-v1", map_version="v1", pose_version="p1",
                  depth_value=2.0, object_id="cup-1", pixel_seed=None):
    """Client-owned known geometry; source files are retained with their hashes."""
    from host.m7_pipeline import CaptureIdentity
    identity = CaptureIdentity(instance, frame_id, acquired_ns, "probe-clock-ns",
                               "probe-calibration-v1", "probe-local", pose_epoch)
    rgb = np.full((8, 8, 3), frame_id * 17 if pixel_seed is None else pixel_seed,
                  dtype=np.uint8)
    depth = np.full((8, 8), depth_value, dtype=np.float32)
    k = np.array([[8., 0., 3.5], [0., 8., 3.5], [0., 0., 1.]], dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    mask = np.zeros((8, 8), dtype=np.bool_)
    mask[2:6, 2:6] = True
    refs = {name: array_file(out / (label + "." + name + ".npy"), value)
            for name, value in (("rgb", rgb), ("depth", depth), ("K", k),
                                ("T", transform), ("mask", mask))}
    rgb_content = out / (label + ".rgb")
    write_new(rgb_content, rgb.tobytes())
    capture = {"acquisition_id": instance + ":" + str(frame_id),
               "source_id": "synthetic-transport-probe",
               "identity": asdict(identity),
               "image_sha256": sha(rgb.tobytes()),
               "time_source": "client_declared_integer_nanoseconds",
               "evidence_refs": [reference(rgb_content), refs["depth"], refs["K"]]}
    anchor = {"map_id": "synthetic-transport-map", "map_version": map_version,
              "world_frame": "probe-map-frame", "pose_source": "synthetic_known_pose",
              "source_kind": "synthetic", "source_ref": reference(__file__),
              "local_frame_id": "probe-local", "pose_epoch": pose_epoch,
              "pose_version": pose_version, "calibration_id": identity.calibration_id,
              "T_map_local": transform.tolist(),
              "evidence_refs": [refs["T"]]}
    detection = {"prediction_set": "synthetic-mask-set-v1", "status": "ok",
                 "error": None, "identity": asdict(identity),
                 "expected_instance_ids": [object_id],
                 "instances": [{"instance_id": object_id, "label": object_id.split("-")[0],
                                "score": 0.9, "source": "synthetic_binary_mask",
                                "mask_content_sha256": sha(mask.astype(np.uint8).tobytes()),
                                "evidence_refs": [refs["mask"]]}]}
    return {"identity": asdict(identity), "rgb": rgb, "depth": depth, "K": k,
            "T": transform, "mask": mask, "capture": capture, "anchor": anchor,
            "detection": detection, "object_id": object_id, "refs": refs}


def geometry(client, name, fixture, now_ns):
    return client.request(name, "geometry",
                          arrays={k: fixture[k] for k in ("rgb", "depth", "K", "T")},
                          identity=fixture["identity"], capture=fixture["capture"],
                          anchor=fixture["anchor"], pose_stamp_ns=fixture["identity"]["acquired_ns"],
                          pose_valid=True, now_ns=now_ns)


def semantic(client, name, fixture, token, now_ns, *, drop_ack=False, detection=None,
             identity=None):
    return client.request(name, "semantic", arrays={"mask": fixture["mask"]},
                          drop_ack=drop_ack, identity=identity or fixture["identity"],
                          frame_token=token, object_id=fixture["object_id"],
                          detection=detection or fixture["detection"],
                          save_as=name, now_ns=now_ns)


def launch_server(out, index, library):
    # Unix sockaddr length is about 108 bytes. Both PIDs work in this unique
    # run directory, so the actual bound address is only "sN.sock".
    path = Path("s" + str(index) + ".sock")
    log_path = out / ("server-" + str(index) + ".jsonl")
    argv = [sys.executable, "-B", str(Path(__file__).resolve()), "server",
            "--socket", str(path), "--store", str(out / "memory.json"),
            "--library", str(library), "--log", str(log_path)]
    pythonpath = str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", "")
    process = subprocess.Popen(argv, cwd=out, stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1",
                                               "PYTHONPATH": pythonpath})
    try:
        import select
        ready, _, _ = select.select([process.stdout], [], [], 10)
        if not ready:
            raise RuntimeError("server did not announce readiness within 10 seconds")
        line = process.stdout.readline()
        if not line:
            raise RuntimeError("server exited before ready")
        receipt = json.loads(line)
        if not receipt.get("ready") or receipt.get("socket") != str(path) or not path.exists():
            raise RuntimeError("invalid server readiness receipt")
        if path.stat().st_mode & 0o077:
            raise AssertionError("probe socket is not owner-only")
        return process, path, log_path, argv, receipt
    except BaseException as error:
        if process.poll() is None:
            process.terminate()
        try:
            _, stderr = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            _, stderr = process.communicate(timeout=3)
        raise RuntimeError(str(error) + "; server stderr: " + stderr[-2000:]) from error


def stop_server(client, process, name):
    expect_ok(client.request(name, "shutdown"))
    client.close()
    stdout, stderr = process.communicate(timeout=10)
    if process.returncode != 0:
        raise RuntimeError("server failed to stop cleanly: " + stderr)
    return {"pid": process.pid, "exit_code": process.returncode,
            "stdout_tail": stdout, "stderr": stderr}


def object_summary(query):
    return sorted((row["id"], row["label"], row["physical_observation_count"],
                   row["state"], row["position_m"]) for row in query["objects"])


def write_manifest(out):
    rows = []
    for path in sorted(out.iterdir()):
        if path.is_file() and path.name != "manifest.json":
            rows.append({"path": path.name, "bytes": path.stat().st_size,
                         "sha256": file_sha(path)})
    write_json(out / "manifest.json", {"files": rows})


def make_revision(out, first):
    new_pose = np.eye(4)
    new_pose[0, 3] = 1.0
    pose_ref = array_file(out / "revision-v2-T.npy", new_pose)
    return {"schema": "dev3.m4.revision.v1", "revision_id": "synthetic-v1-to-v2",
            "map_id": first["anchor"]["map_id"], "from_version": "v1", "to_version": "v2",
            "reason": "known one-metre synthetic map correction; second anchor deliberately omitted",
            "poses": [{"map_id": first["anchor"]["map_id"],
                       "keyframe_id": first["capture"]["acquisition_id"],
                       "version": "p2", "frame_id": first["identity"]["local_frame_id"],
                       "T_map_local": new_pose.tolist(),
                       "source": "synthetic_known_map_revision",
                       "evidence_refs": [pose_ref]}],
            "unchanged_keyframes": []}


def run_probe(out):
    from host.m7_native import compile_native
    from perception.offline_memory import OfflineMemory
    out = Path(out).resolve()
    if not str(out).startswith(str((ROOT / "runs").resolve()) + os.sep):
        raise ValueError("output must be a new directory under this clone's runs/")
    if out.exists():
        raise FileExistsError(out)
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True)
    if dirty.strip():
        raise RuntimeError("commit and tag the frozen source before the formal probe")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    tags = subprocess.check_output(["git", "tag", "--points-at", "HEAD"], cwd=ROOT,
                                   text=True).splitlines()
    if not any("transport" in tag or "continuous" in tag for tag in tags):
        raise RuntimeError("a source-freeze experiment tag at HEAD is required")
    out.mkdir(parents=True)
    os.chdir(out)
    clients = []
    servers = []
    trace = {"source_commit": commit, "source_tags": tags, "client_pid": os.getpid(),
             "python": sys.version, "runner_command": sys.argv,
             "servers": [], "requests": [], "checks": []}
    trace["source_sha256"] = {name: file_sha(ROOT / name) for name in
                              ("tools/m7_memory_transport_probe.py", "host/m7_memory.py",
                               "host/m7_pipeline.py", "host/m7_native.py",
                               "perception/offline_memory.py")}
    try:
        library = compile_native(out / "native-m7.so")
        trace["native_library_sha256"] = file_sha(library)
        trace["native_compile_command"] = [shutil.which("g++"), "-std=c++11", "-O2",
                                           "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
                                           "-fPIC", "-shared", str(ROOT / "hw/geometry/geometry.cpp"),
                                           str(ROOT / "hw/m7/ray_flags.cpp"), "-o", str(library)]
        first = frame_fixture(out, "frame-1", instance="probe-stream-a", frame_id=1,
                              acquired_ns=1_000_000_000, object_id="cup-1")
        second = frame_fixture(out, "frame-2", instance="probe-stream-a", frame_id=2,
                               acquired_ns=1_020_000_000, depth_value=3.0,
                               object_id="bottle-1")
        third = frame_fixture(out, "frame-3", instance="probe-stream-b", frame_id=1,
                              acquired_ns=1_100_000_000, pose_epoch="pose-v2",
                              map_version="v2", pose_version="p2", object_id="cup-3",
                              pixel_seed=51)
        fourth = frame_fixture(out, "frame-4", instance="probe-stream-b", frame_id=2,
                               acquired_ns=1_120_000_000, pose_epoch="pose-v2",
                               map_version="v2", pose_version="p2", object_id="cup-4",
                               pixel_seed=68)
        process, path, log, argv, ready = launch_server(out, 1, library)
        servers.append(process)
        trace["servers"].append({"argv": argv, "ready": ready, "log": str(log)})

        # A partial request must fail before any store exists, then allow a fresh connection.
        truncated = encode_request({"request_id": "partial", "kind": "query"})
        write_new(out / "partial.request.bin", struct.pack("!I", len(truncated)) + truncated[:10])
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as broken:
            broken.connect(str(path))
            broken.sendall(struct.pack("!I", len(truncated)) + truncated[:10])
            broken.shutdown(socket.SHUT_WR)
        if (out / "memory.json").exists():
            raise AssertionError("truncated request changed the store")
        trace["checks"].append("partial request did not create store")

        client = ProbeClient(path, out)
        clients.append(client)
        g1 = expect_ok(geometry(client, "a-geometry-1", first, 1_001_000_000), valid=True)
        g2 = expect_ok(geometry(client, "a-geometry-2", second, 1_021_000_000), valid=True)
        if not g1["frame_token"] or g1["frame_token"] == g2["frame_token"]:
            raise AssertionError("M7 did not mint distinct frame tokens")
        # The first mask arrives after the second geometry, but still uses first-frame depth.
        s1 = expect_ok(semantic(client, "a-semantic-late-1", first, g1["frame_token"],
                                1_022_000_000), valid=True)
        if not s1["imported"]:
            raise AssertionError("valid delayed semantic result was not persisted")
        before_drop = file_sha(out / "memory.json")
        if semantic(client, "a-semantic-2-ack-lost", second, g2["frame_token"],
                    1_023_000_000, drop_ack=True) is not None:
            raise AssertionError("ACK loss fault did not occur")
        after_drop = file_sha(out / "memory.json")
        if before_drop == after_drop:
            raise AssertionError("ACK was lost before the second observation was saved")
        client.close()
        client = ProbeClient(path, out)
        clients.append(client)
        stop = stop_server(client, process, "a-shutdown")
        trace["servers"][-1]["stop"] = stop
        first_log = [json.loads(line) for line in log.read_text().splitlines()]
        if not any(row.get("event") == "read_end" and
                   row.get("store_sha256") is None and
                   "incomplete" in row.get("detail", "") for row in first_log):
            raise AssertionError("truncated request was not observed as a partial read")
        stored_before_restart = OfflineMemory.load(out / "memory.json").query()
        baseline_objects = object_summary(stored_before_restart)
        if len(baseline_objects) != 2:
            raise AssertionError("two source frames did not create two candidates")
        original_objects = {o["label"]: o for o in stored_before_restart["objects"]}
        for label, expected in (("cup", [0., 0., 2.]), ("bottle", [0., 0., 3.])):
            row = original_objects[label]
            if (row["physical_observation_count"] != 1 or
                    not np.allclose(row["position_m"], expected, atol=1e-9, rtol=0)):
                raise AssertionError("M4 object differs from independent original-frame geometry")
        stored_acquisitions = OfflineMemory.load(out / "memory.json").data["acquisitions"]
        actual_ids = {item["batch"]["acquisition"]["id"] for item in stored_acquisitions.values()}
        if actual_ids != {first["capture"]["acquisition_id"],
                          second["capture"]["acquisition_id"]}:
            raise AssertionError("M4 acquisition identities differ from sent raw frames")
        trace["checks"].append("two real-process acquisitions persisted; ACK loss after save")

        process2, path2, log2, argv2, ready2 = launch_server(out, 2, library)
        servers.append(process2)
        trace["servers"].append({"argv": argv2, "ready": ready2, "log": str(log2)})
        if process2.pid == process.pid:
            raise AssertionError("server restart did not use a new PID")
        client2 = ProbeClient(path2, out)
        clients.append(client2)
        g1b = expect_ok(geometry(client2, "b-geometry-1", first, 1_001_000_000), valid=True)
        g2b = expect_ok(geometry(client2, "b-geometry-2", second, 1_021_000_000), valid=True)
        if g2b["frame_token"] == g2["frame_token"]:
            raise AssertionError("new M7 process reused a prior frame token")
        duplicate = expect_ok(semantic(client2, "b-retransmit-after-lost-ack", second,
                                       g2b["frame_token"], 1_023_000_000), valid=True)
        if not duplicate["imported"] or any(
                x["status"] != "duplicate" for x in duplicate["import_result"]["batches"]):
            raise AssertionError("ACK-loss retransmit was not an M4 duplicate")
        if file_sha(out / "memory.json") != after_drop or \
                object_summary(OfflineMemory.load(out / "memory.json").query()) != baseline_objects:
            raise AssertionError("retransmit changed stable M4 evidence")
        trace["checks"].append("new PID minted new token; same source bytes did not add evidence")

        unchanged = file_sha(out / "memory.json")
        expect_ok(geometry(client2, "b-duplicate-geometry", second, 1_024_000_000), valid=False)
        expect_ok(geometry(client2, "b-reversed-geometry", first, 1_024_000_000), valid=False)
        expect_ok(semantic(client2, "b-wrong-frame-token", first, g2b["frame_token"],
                           1_024_000_000), valid=False)
        expect_ok(semantic(client2, "b-duplicate-semantic", second, g2b["frame_token"],
                           1_024_000_000), valid=False)
        if file_sha(out / "memory.json") != unchanged:
            raise AssertionError("bad frame or repeated semantic changed store")
        trace["checks"].append("duplicate/reversed/wrong-frame requests left store unchanged")

        # Reconstruct first output in this process to test its revocation independently.
        first_again = expect_ok(semantic(client2, "b-retransmit-first", first,
                                         g1b["frame_token"], 1_024_000_000), valid=True)
        if any(x["status"] != "duplicate" for x in first_again["import_result"]["batches"]):
            raise AssertionError("first replay did not preserve identity")
        pre_revision = expect_ok(client2.request("b-before-revision-valid", "validate",
                                                 save_as="b-retransmit-first",
                                                 now_ns=1_025_000_000,
                                                 clock_id="probe-clock-ns"), valid=True)
        if not pre_revision["valid"]:
            raise AssertionError("unexpired old output unexpectedly invalid")

        pose_keys = {row["original"]["keyframe_id"] for row in
                     OfflineMemory.load(out / "memory.json").data["poses"].values()}
        if pose_keys != {first["capture"]["acquisition_id"],
                         second["capture"]["acquisition_id"]}:
            raise AssertionError("stored map anchors do not match client acquisition IDs")
        write_new(out / "memory-before-revision.json", (out / "memory.json").read_bytes())
        revision = make_revision(out, first)
        bad_revision = {**revision, "revision_id": "bad-from-version", "from_version": "v0"}
        before_bad = file_sha(out / "memory.json")
        expect_rejected(client2.request("b-invalid-revision", "revision", document=bad_revision))
        expect_ok(client2.request("b-after-invalid-revision-valid", "validate",
                                  save_as="b-retransmit-first", now_ns=1_026_000_000,
                                  clock_id="probe-clock-ns"), valid=True)
        if file_sha(out / "memory.json") != before_bad:
            raise AssertionError("failed revision changed persistent store")
        revised = expect_ok(client2.request("b-revision-v2", "revision", document=revision))
        if revised["revision"]["status"] != "revised":
            raise AssertionError("valid revision did not commit")
        expect_ok(client2.request("b-old-output-revoked", "validate",
                                  save_as="b-retransmit-first", now_ns=1_027_000_000,
                                  clock_id="probe-clock-ns"), valid=False)
        revised_query = expect_ok(client2.request("b-query-revised", "query"))["query"]
        if revised_query["maps"][0]["map_version"] != "v2":
            raise AssertionError("revision did not advance map version")
        revised_objects = {o["label"]: o for o in revised_query["objects"]}
        original_objects = {o["label"]: o for o in stored_before_restart["objects"]}
        cup_shift = np.asarray(revised_objects["cup"]["position_m"]) - \
            np.asarray(original_objects["cup"]["position_m"])
        if (revised_objects["cup"]["id"] != original_objects["cup"]["id"] or
                not np.allclose(cup_shift, [1., 0., 0.], atol=1e-9, rtol=0) or
                revised_objects["bottle"]["position_m"] is not None or
                revised_objects["bottle"]["state"] != "pending_revision"):
            raise AssertionError("revision did not shift one anchor and pend the omitted one")
        trace["checks"].append("failed revision preserved output; valid revision revoked old output and pended missing anchor")

        g3 = expect_ok(geometry(client2, "b-new-epoch-geometry", third, 1_101_000_000), valid=True)
        s3 = expect_ok(semantic(client2, "b-new-epoch-semantic", third,
                                g3["frame_token"], 1_102_000_000), valid=True)
        if not s3["imported"]:
            raise AssertionError("new epoch observation was not imported")
        repeated_revision = expect_ok(client2.request("b-duplicate-revision", "revision",
                                                      document=revision))
        if repeated_revision["revision"]["status"] != "duplicate_revision":
            raise AssertionError("same revision was not idempotent")
        expect_ok(client2.request("b-new-epoch-still-valid", "validate",
                                  save_as="b-new-epoch-semantic", now_ns=1_103_000_000,
                                  clock_id="probe-clock-ns"), valid=True)
        trace["checks"].append("duplicate revision did not revoke new-epoch output")

        g4 = expect_ok(geometry(client2, "b-incomplete-geometry", fourth, 1_121_000_000), valid=True)
        incomplete = json.loads(json.dumps(fourth["detection"]))
        incomplete["expected_instance_ids"].append("missing-instance")
        before_incomplete = file_sha(out / "memory.json")
        expect_rejected(semantic(client2, "b-incomplete-batch", fourth,
                                 g4["frame_token"], 1_122_000_000,
                                 detection=incomplete))
        if file_sha(out / "memory.json") != before_incomplete:
            raise AssertionError("incomplete semantic batch partially committed")
        trace["checks"].append("incomplete batch rejected atomically")

        stale = frame_fixture(out, "frame-stale-v1", instance="probe-stream-b", frame_id=3,
                              acquired_ns=1_140_000_000, pose_epoch="pose-v2",
                              map_version="v1", pose_version="p2", object_id="cup-stale",
                              pixel_seed=85)
        stale_g = expect_ok(geometry(client2, "b-stale-version-geometry", stale,
                                     1_141_000_000), valid=True)
        before_stale = file_sha(out / "memory.json")
        expect_rejected(semantic(client2, "b-stale-version-semantic", stale,
                                 stale_g["frame_token"], 1_142_000_000))
        if file_sha(out / "memory.json") != before_stale:
            raise AssertionError("post-revision old-version frame changed store")
        trace["checks"].append("post-revision old map version was rejected")

        trace["final_query"] = expect_ok(client2.request("b-final-query", "query"))["query"]
        trace["final_reloaded_query"] = OfflineMemory.load(out / "memory.json").query()
        trace["servers"][-1]["stop"] = stop_server(client2, process2, "b-shutdown")
        trace["requests"] = [row for client in clients for row in client.rows]
        trace["store_sha256"] = file_sha(out / "memory.json")
        trace["ok"] = True
        write_json(out / "run-info.json", {key: trace[key] for key in
                                          ("source_commit", "source_tags", "client_pid", "python",
                                           "runner_command", "native_library_sha256",
                                           "native_compile_command", "source_sha256", "servers")})
        write_json(out / "summary.json", trace)
    except BaseException as exc:
        trace["ok"] = False
        trace["error"] = {"type": type(exc).__name__, "message": str(exc)}
        trace["requests"] = [row for client in clients for row in client.rows]
        write_json(out / "run-info.json", {key: trace.get(key) for key in
                                          ("source_commit", "source_tags", "client_pid", "python",
                                           "runner_command", "native_library_sha256",
                                           "native_compile_command", "source_sha256", "servers")})
        write_json(out / "failure.json", trace)
        raise
    finally:
        for client in clients:
            client.close()
        for process in servers:
            if process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=3)
        write_manifest(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    server = sub.add_parser("server")
    for name in ("socket", "store", "library", "log"):
        server.add_argument("--"+name, type=Path, required=True)
    run = sub.add_parser("run")
    run.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.mode == "server":
        serve(args.socket, args.store, args.library, args.log)
    else:
        run_probe(args.out)


if __name__ == "__main__":
    main()
