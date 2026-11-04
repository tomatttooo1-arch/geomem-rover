"""Object memory across runs (10-18): save the runner's memory, load it into a later run on the same map.

No ROS. Used by host/ros_runtime.py (--memory-out-dir / --memory-in / --memory-map-identity) and, as a CLI,
by tools/car_run/car_up.sh (identity / check) and car_down.sh (bind). Design and the state semantics:
docs/design/MEMORY_PERSIST_20261018.md.

File = one JSON document, format FORMAT version VERSION:
  header   format, v, complete (true: written at a normal stop; false: periodic snapshot), written_wall
           (ISO time), written_unix_s, written_ros_s, code_commit, runtime_boot_id (= the GeoMem memory.scope of
           the run that wrote it), world_frame, pose_source, pose_mode (car_up --pose), sensor_source,
           map_identity (see map_identity()), loaded_from (the file this run had loaded, or null)
  memory   strategy, cfg (every Config field), odometer [path m, turned rad] at the write, rejected counts
  tracks   per object: id, label, color, position_m (world_frame, metres), bounds_m [lower, upper],
           last_seen (acquisition time on the writing run's clock), last_seen_frame, state, reason,
           negative_views, odo [path m, turned rad] at its last update, pose_sigma, history_len,
           history (the last HISTORY_KEEP entries), prior (the earlier run's facts when this track was itself
           loaded, else absent); optional (10-27): number, covis, aliases, existence (log-odds score)

Loading never degrades silently: a bad file, a wrong version, a missing field, non-contiguous ids or a map
identity that does not match the current run's raises MemoryStoreError (the caller refuses to start).
"""
import argparse
import dataclasses
import datetime
import hashlib
import json
import math
import os
import re
import sys

FORMAT = "fpga-object-memory"
VERSION = 1
HISTORY_KEEP = 20
STATES = ("current_confirmed", "temporarily_occluded", "historical_memory", "position_invalid")
LOADED_REASON = "loaded_prior_run"
IDENTITY_KINDS = ("rtabmap_db", "map_export", "sim_map", "mapping_pending", "none")
TRACK_FIELDS = ("id", "label", "color", "position_m", "bounds_m", "last_seen", "last_seen_frame", "state",
                "reason", "negative_views", "odo", "pose_sigma", "history_len", "history")
HEADER_FIELDS = ("format", "v", "complete", "written_wall", "written_unix_s", "written_ros_s", "code_commit",
                 "runtime_boot_id", "world_frame", "pose_source", "pose_mode", "sensor_source", "map_identity",
                 "loaded_from", "memory", "tracks")


class MemoryStoreError(ValueError):
    """A refusal; ``code`` is a short machine-readable reason."""

    def __init__(self, code, message):
        ValueError.__init__(self, "%s: %s" % (code, message))
        self.code = code


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


# ---------------------------------------------------------------- map identity
def _yaml_image(yaml_path):
    """The image a map_server yaml names (relative to the yaml), without a YAML library."""
    with open(yaml_path, encoding="utf8") as stream:
        for line in stream:
            m = re.match(r"\s*image\s*:\s*(.+?)\s*$", line)
            if m:
                name = m.group(1).strip("'\"")
                return name if os.path.isabs(name) else os.path.join(os.path.dirname(yaml_path), name)
    raise MemoryStoreError("map_yaml_no_image", "%s names no image" % yaml_path)


def _sha_table(path):
    """sha256sum output -> {basename: (sha, path as written)}."""
    rows = {}
    with open(path, encoding="utf8") as stream:
        for line in stream:
            m = re.match(r"([0-9a-f]{64})\s+\*?(.+?)\s*$", line)
            if m:
                rows[os.path.basename(m.group(2))] = (m.group(1), m.group(2))
    return rows


def _export_part(yaml_path):
    yaml_path = os.path.abspath(yaml_path)
    if not os.path.isfile(yaml_path):
        raise MemoryStoreError("map_yaml_missing", "%s is not a file" % yaml_path)
    image = _yaml_image(yaml_path)
    if not os.path.isfile(image):
        raise MemoryStoreError("map_image_missing", "%s (named by %s) is not a file" % (image, yaml_path))
    return {"map_yaml": yaml_path, "map_yaml_sha256": sha256_file(yaml_path),
            "map_image": os.path.abspath(image), "map_image_sha256": sha256_file(image)}


def export_source_db(yaml_path, yaml_sha):
    """The source database of an rtabmap_car.sh export: <export>/../sha256.txt lists the db and the export
    files. Returns (db_sha256, db path as written, note); (None, None, why) when it cannot be told."""
    table_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(yaml_path))), "sha256.txt")
    if not os.path.isfile(table_path):
        return None, None, "no %s" % table_path
    rows = _sha_table(table_path)
    listed = rows.get(os.path.basename(yaml_path))
    if listed is None:
        return None, None, "%s does not list %s" % (table_path, os.path.basename(yaml_path))
    if listed[0] != yaml_sha:
        return None, None, "%s lists another %s (the export was changed)" % (table_path, os.path.basename(yaml_path))
    dbs = [v for k, v in rows.items() if k.endswith(".db")]
    if len(dbs) != 1:
        return None, None, "%s lists %d databases" % (table_path, len(dbs))
    return dbs[0][0], dbs[0][1], "from %s" % table_path


def map_identity(map_yaml=None, rtabmap_db=None, sim_map_yaml=None, sim_world=None, mapping_db_dir=None):
    """The identity of the map whose frame the memory positions are in.

    map_yaml     the served mapping export (odom: CAR_MAP_YAML / the RTABMAP_DB export): kind rtabmap_db when
                 its sha256.txt names the source database, else map_export (yaml + image sha256)
    rtabmap_db   a database without a served export (rtabmap localization): kind rtabmap_db
    sim_map_yaml the simulator's Nav2 map (with sim_world, recorded only): kind sim_map
    mapping_db_dir  this run builds the map: kind mapping_pending (bind() fills the database later)
    nothing      kind none
    """
    ident = {"v": 1, "kind": "none", "db_sha256": None, "db_path": None, "map_yaml": None, "map_yaml_sha256": None,
             "map_image": None, "map_image_sha256": None, "sim_world": None, "sim_world_sha256": None, "note": None}
    if mapping_db_dir:
        ident.update(kind="mapping_pending", db_path=os.path.abspath(mapping_db_dir),
                     note="map built by this run; car_down binds the database (memory_store bind)")
    elif map_yaml:
        ident.update(_export_part(map_yaml))
        db_sha, db_path, note = export_source_db(map_yaml, ident["map_yaml_sha256"])
        ident.update(kind="rtabmap_db" if db_sha else "map_export", db_sha256=db_sha, db_path=db_path, note=note)
    elif rtabmap_db:
        if not os.path.isfile(rtabmap_db):
            raise MemoryStoreError("db_missing", "%s is not a file" % rtabmap_db)
        ident.update(kind="rtabmap_db", db_sha256=sha256_file(rtabmap_db), db_path=os.path.abspath(rtabmap_db))
    elif sim_map_yaml:
        ident.update(_export_part(sim_map_yaml))
        ident["kind"] = "sim_map"
        if sim_world:
            ident.update(sim_world=os.path.basename(sim_world),
                         sim_world_sha256=sha256_file(sim_world) if os.path.isfile(sim_world) else None)
        ident["note"] = "simulator world frame; the world file is recorded, not compared (objects may move)"
    return ident


def identity_match(stored, current):
    """(True, how) or (False, why). Never matches none or a pending (unbound) identity."""
    if not isinstance(stored, dict) or not isinstance(current, dict):
        return False, "identity missing"
    for side, ident in (("file", stored), ("this run", current)):
        if ident.get("kind") not in IDENTITY_KINDS:
            return False, "%s: unknown identity kind %r" % (side, ident.get("kind"))
        if ident["kind"] == "none":
            return False, "%s has no map (identity none): a memory cannot be tied to a frame" % side
        if ident["kind"] == "mapping_pending":
            return False, "%s identity is mapping_pending (not bound to a database yet: memory_store bind)" % side
    if (stored["kind"] == "sim_map") != (current["kind"] == "sim_map"):
        return False, "simulator map vs real map (%s / %s)" % (stored["kind"], current["kind"])
    if stored.get("db_sha256") and current.get("db_sha256"):
        if stored["db_sha256"] == current["db_sha256"]:
            return True, "same mapping database %s" % current["db_sha256"][:12]
        return False, "mapping database differs (file %s, this run %s)" % (stored["db_sha256"][:12],
                                                                          current["db_sha256"][:12])
    keys = ("map_yaml_sha256", "map_image_sha256")
    if all(stored.get(k) for k in keys) and all(current.get(k) for k in keys):
        if all(stored[k] == current[k] for k in keys):
            return True, "same map export (yaml + image sha256)"
        return False, "map export differs (yaml/image sha256)"
    return False, "identities share no comparable field (file %s, this run %s)" % (stored["kind"], current["kind"])


# ---------------------------------------------------------------- save
def _num(value):
    value = float(value)
    if not math.isfinite(value):
        raise MemoryStoreError("non_finite", "non-finite number in memory")
    return value


def _plain(value):
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if hasattr(value, "tolist"):
        return _plain(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def track_record(t):
    row = {"id": t.id, "label": t.label, "color": t.color, "position_m": [_num(v) for v in t.position],
           "bounds_m": [[_num(v) for v in t.lower], [_num(v) for v in t.upper]],
           "last_seen": _num(t.last_seen), "last_seen_frame": t.last_seen_frame, "state": t.state,
           "reason": t.reason, "negative_views": len(t.negative_poses), "odo": [_num(v) for v in t.odo],
           "pose_sigma": None if t.pose_sigma is None else _num(t.pose_sigma), "history_len": len(t.history),
           "history": _plain(t.history[-HISTORY_KEEP:])}
    # 10-27 identity (optional on read): class-local display number, ids seen in the same frame, merged ids
    if getattr(t, "number", None) is not None:
        row["number"] = int(t.number)
    if getattr(t, "covis", None):
        row["covis"] = sorted(t.covis)
    if getattr(t, "aliases", None):
        row["aliases"] = list(t.aliases)
    # 10-27 existence score (Config.existence_v3; optional on read, absent while the switch is off)
    if getattr(t, "existence", None) is not None:
        row["existence"] = round(_num(float(t.existence)), 3)
    prior = getattr(t, "prior", None)
    if prior is not None:
        row["prior"] = _plain(prior)
    return row


def dump(memory, *, complete, runtime_boot_id, world_frame, pose_source, sensor_source, map_identity_doc,
         pose_mode=None, code_commit=None, written_ros_s=None, loaded_from=None, now=None):
    now = datetime.datetime.now(datetime.timezone.utc) if now is None else now
    return {"format": FORMAT, "v": VERSION, "complete": bool(complete),
            "written_wall": now.isoformat(), "written_unix_s": now.timestamp(),
            "written_ros_s": None if written_ros_s is None else float(written_ros_s),
            "code_commit": code_commit, "runtime_boot_id": runtime_boot_id, "world_frame": world_frame,
            "pose_source": pose_source, "pose_mode": pose_mode, "sensor_source": sensor_source,
            "map_identity": map_identity_doc, "loaded_from": loaded_from,
            "memory": {"strategy": memory.strategy, "cfg": _plain(dataclasses.asdict(memory.cfg)),
                       "odometer": [_num(v) for v in memory.odometer], "rejected": dict(memory.rejected)},
            "tracks": [track_record(t) for t in memory.tracks.values()]}


def save_atomic(doc, path, fsync=True):
    """Write next to the target and rename over it: a reader never sees half a file. fsync=False (periodic
    snapshots): survives a process crash, not a power cut."""
    text = json.dumps(doc, ensure_ascii=False, allow_nan=False, indent=1) + "\n"
    tmp = "%s.tmp-%d" % (path, os.getpid())
    try:
        with open(tmp, "w", encoding="utf8") as stream:
            stream.write(text)
            if fsync:
                stream.flush()
                os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


class SnapshotPolicy(object):
    """Periodic snapshot: only when the content changed and at least ``min_interval_s`` after the last write."""

    def __init__(self, min_interval_s=5.0):
        self.min_interval_s = float(min_interval_s)
        self.last_key = None
        self.last_write = None
        self.writes = self.errors = 0

    def due(self, key, now_mono):
        if key == self.last_key:
            return False
        return self.last_write is None or now_mono - self.last_write >= self.min_interval_s

    def done(self, key, now_mono):
        self.last_key, self.last_write = key, now_mono
        self.writes += 1


# ---------------------------------------------------------------- load
def _need(cond, code, message):
    if not cond:
        raise MemoryStoreError(code, message)


def _vec(value, name, n=3):
    _need(isinstance(value, list) and len(value) == n, "bad_field", "%s must be a list of %d numbers" % (name, n))
    out = []
    for v in value:
        _need(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v), "bad_field",
              "%s has a non-finite or non-number entry" % name)
        out.append(float(v))
    return out


def read_document(path):
    try:
        with open(path, encoding="utf8") as stream:
            doc = json.load(stream)
    except OSError as exc:
        raise MemoryStoreError("unreadable", "%s: %s" % (path, exc))
    except ValueError as exc:
        raise MemoryStoreError("bad_json", "%s: %s" % (path, exc))
    validate_document(doc)
    return doc


def validate_document(doc):
    _need(isinstance(doc, dict), "bad_json", "top level is not an object")
    _need(doc.get("format") == FORMAT, "wrong_format", "format %r, expected %r" % (doc.get("format"), FORMAT))
    _need(doc.get("v") == VERSION, "wrong_version", "version %r, this code reads %r" % (doc.get("v"), VERSION))
    missing = [k for k in HEADER_FIELDS if k not in doc]
    _need(not missing, "missing_field", "header lacks %s" % ", ".join(missing))
    _need(doc["complete"] is True, "incomplete",
          "periodic snapshot (complete=false): the run did not stop normally; load it only after a look "
          "(memory_store promote writes a complete copy)")
    _need(doc["world_frame"] == "map", "bad_field", "world_frame %r (this runner keeps memory in 'map')"
          % doc["world_frame"])
    mem = doc["memory"]
    _need(isinstance(mem, dict) and all(k in mem for k in ("strategy", "cfg", "odometer", "rejected")),
          "missing_field", "memory needs strategy, cfg, odometer, rejected")
    _vec(mem["odometer"], "memory.odometer", 2)
    _need(isinstance(doc["tracks"], list), "bad_field", "tracks is not a list")
    ids = []
    for i, row in enumerate(doc["tracks"]):
        _need(isinstance(row, dict), "bad_field", "track %d is not an object" % i)
        missing = [k for k in TRACK_FIELDS if k not in row]
        _need(not missing, "missing_field", "track %d lacks %s" % (i, ", ".join(missing)))
        _vec(row["position_m"], "track %s position_m" % row["id"])
        _need(isinstance(row["bounds_m"], list) and len(row["bounds_m"]) == 2, "bad_field",
              "track %s bounds_m" % row["id"])
        lo, hi = _vec(row["bounds_m"][0], "bounds"), _vec(row["bounds_m"][1], "bounds")
        _need(all(a <= b for a, b in zip(lo, hi)), "bad_field", "track %s bounds lower > upper" % row["id"])
        _need(row["state"] in STATES, "bad_field", "track %s state %r" % (row["id"], row["state"]))
        _need(isinstance(row["label"], str) and row["label"], "bad_field", "track %d label" % i)
        _vec([row["last_seen"]], "track last_seen", 1)
        _vec(row["odo"], "track odo", 2)
        _need(row["pose_sigma"] is None or _vec([row["pose_sigma"]], "pose_sigma", 1), "bad_field", "pose_sigma")
        _need(isinstance(row["history"], list), "bad_field", "track %s history" % row["id"])
        ids.append(row["id"])
    # 10-27: merged tracks leave gaps; update_memory now takes the next serial above every id it holds, so
    # ids need only be well formed and unique (a duplicate would still overwrite one).
    bad = [k for k in ids if not (isinstance(k, str) and re.fullmatch(r"object-[0-9]{4,}", k))]
    _need(not bad, "bad_track_id", "track ids must look like object-0001, got %s" % bad[:5])
    _need(len(set(ids)) == len(ids), "duplicate_track_id", "track ids repeat: %s"
          % sorted({k for k in ids if ids.count(k) > 1})[:5])
    for row in doc["tracks"]:
        _need(row.get("number") is None or (isinstance(row["number"], int) and not isinstance(row["number"], bool)
                                             and row["number"] > 0), "bad_field", "track %s number" % row["id"])
        _need(row.get("existence") is None or _vec([row["existence"]], "track existence", 1), "bad_field",
              "track %s existence" % row["id"])


def check(doc, current_identity):
    ok, how = identity_match(doc.get("map_identity"), current_identity)
    _need(ok, "map_mismatch", how)
    return how


def restore(doc, memory):
    """Fill an empty Memory (the runner's own, its cfg/strategy kept) from a validated document.

    State semantics (docs/design/MEMORY_PERSIST_20261018.md section 3):
      * current_confirmed / temporarily_occluded -> historical_memory (nothing is current before this run sees
        it; update_memory would do the same at the first frame); historical_memory, position_invalid kept
      * reason -> loaded_prior_run until the first evidence of this run
      * last_seen -> 0.0 (earlier than any acquisition of this run on either clock: the simulator restarts at
        0, the board runs on wall time); the earlier value is kept in track.prior
      * negative evidence cleared (its stamps are on the earlier clock); the count is kept in prior
      * odo -> the drift already accumulated (track odo - final odometer, <= 0): the association radius keeps
        growing from where the earlier run left it (still capped at association_max_m)
    """
    import numpy as np
    from perception.fpga_replay.core import Track
    _need(not memory.tracks, "memory_not_empty", "load into an empty memory only")
    end_odo = [float(v) for v in doc["memory"]["odometer"]]
    for row in doc["tracks"]:
        lo, hi = row["bounds_m"]
        t = Track(row["id"], row["label"], row["color"], np.array(row["position_m"], dtype=float),
                  np.array(lo, dtype=float), np.array(hi, dtype=float), 0.0)
        t.state = "historical_memory" if row["state"] in ("current_confirmed", "temporarily_occluded") else row["state"]
        t.reason = LOADED_REASON
        t.history = list(row["history"])
        t.last_seen_frame = row["last_seen_frame"]
        t.odo = (float(row["odo"][0]) - end_odo[0], float(row["odo"][1]) - end_odo[1])
        t.pose_sigma = row["pose_sigma"]
        t.prior = {"runtime_boot_id": doc["runtime_boot_id"], "written_wall": doc["written_wall"],
                   "last_seen": row["last_seen"], "state": row["state"], "reason": row["reason"],
                   "negative_views": row["negative_views"], "history_len": row["history_len"]}
        t.number = row.get("number")
        t.covis = set(row.get("covis") or ())
        t.aliases = list(row.get("aliases") or ())
        # one prior sample at the stored position: later readings refine it instead of jumping
        t.samples = [(0.0, t.position.copy(), 0.05)]
        t.fused_sd = 0.05
        # 10-27 existence: the stored score when there is one, else the start value for the restored state
        # (current/occluded -> historical: existence_init; position_invalid: invalid_below); off: none
        if getattr(memory.cfg, "existence_v3", False):
            t.existence = row.get("existence")
            if t.existence is None:
                memory._ex_start(t)
            t.existence = float(t.existence)
        memory.tracks[t.id] = t
    for t in memory.tracks.values():
        t.covis &= set(memory.tracks)
        for a in t.aliases:
            memory.merged[a] = t.id
    for t in sorted(memory.tracks.values(), key=lambda t: t.id):   # files from before 10-27 carry no number
        if t.number is None:
            t.number = memory._next_number(t.label)
    return {"tracks": len(memory.tracks), "states": {s: sum(t.state == s for t in memory.tracks.values())
                                                     for s in STATES},
            "cfg_differs": sorted(k for k, v in dataclasses.asdict(memory.cfg).items()
                                  if doc["memory"]["cfg"].get(k, _plain(v)) != _plain(v)),
            "strategy_differs": doc["memory"]["strategy"] != memory.strategy}


def load_into(memory, path, current_identity):
    """read + validate + identity check + restore. Raises MemoryStoreError, never returns a half memory."""
    doc = read_document(path)
    how = check(doc, current_identity)
    info = restore(doc, memory)
    info.update(path=os.path.abspath(path), file_sha256=sha256_file(path), identity=how,
                runtime_boot_id=doc["runtime_boot_id"], written_wall=doc["written_wall"])
    return info


# ---------------------------------------------------------------- bind (mapping run, after car_down)
def bind(doc, export_yaml):
    """A mapping run's memory (identity mapping_pending) + the export rtabmap_car.sh wrote after the database
    was saved -> the same memory with the database identity. Only for a pending identity whose database
    directory holds that export's database."""
    ident = doc.get("map_identity") or {}
    _need(ident.get("kind") == "mapping_pending", "not_pending", "identity is %r, not mapping_pending"
          % ident.get("kind"))
    new = map_identity(map_yaml=export_yaml)
    _need(new["kind"] == "rtabmap_db", "bind_no_db", "export does not name its database (%s)" % new["note"])
    # both paths as the stack saw them (sha256.txt is written inside the stack, the pending dir is the runner's)
    _need(ident.get("db_path") and os.path.normpath(os.path.dirname(new["db_path"])) ==
          os.path.normpath(ident["db_path"]), "bind_wrong_dir",
          "database %s (export %s) is not in the mapping directory %s" % (new["db_path"], export_yaml,
                                                                         ident.get("db_path")))
    out = dict(doc)
    out["map_identity"] = dict(new, bound_from={"pending": ident, "bound_wall":
                                                datetime.datetime.now(datetime.timezone.utc).isoformat()})
    return out


# ---------------------------------------------------------------- CLI
def _identity_from_args(a):
    return map_identity(map_yaml=a.map_yaml or None, rtabmap_db=a.rtabmap_db or None,
                        sim_map_yaml=a.sim_map_yaml or None, sim_world=a.sim_world or None,
                        mapping_db_dir=a.mapping_db_dir or None)


def main(argv=None):
    ap = argparse.ArgumentParser(description="object memory files (identity | check | bind | promote | show)")
    ap.add_argument("command", choices=("identity", "check", "bind", "promote", "show"))
    ap.add_argument("--memory", help="memory file (check, bind, promote, show)")
    ap.add_argument("--out", help="identity/bind/promote: new file, never overwritten")
    ap.add_argument("--identity", help="check: the current run's identity file (else from the map switches)")
    for name in ("map-yaml", "rtabmap-db", "sim-map-yaml", "sim-world", "mapping-db-dir"):
        ap.add_argument("--" + name, default="")
    ap.add_argument("--export-yaml", help="bind: the export of the database the mapping run built")
    a = ap.parse_args(argv)
    try:
        if a.out and os.path.exists(a.out):
            raise MemoryStoreError("out_exists", "%s exists (never overwritten)" % a.out)
        if a.command == "identity":
            result = _identity_from_args(a)
        else:
            _need(a.memory, "usage", "--memory required")
            if a.command == "check":
                doc = read_document(a.memory)
                if a.identity:
                    with open(a.identity, encoding="utf8") as stream:
                        current = json.load(stream)
                else:
                    current = _identity_from_args(a)
                how = check(doc, current)
                result = {"ok": True, "identity": how, "tracks": len(doc["tracks"]),
                          "states": {s: sum(r["state"] == s for r in doc["tracks"]) for s in STATES},
                          "labels": sorted({r["label"] for r in doc["tracks"]}),
                          "written_wall": doc["written_wall"], "code_commit": doc["code_commit"],
                          "file_sha256": sha256_file(a.memory)}
            elif a.command == "bind":
                with open(a.memory, encoding="utf8") as stream:
                    doc = json.load(stream)
                _need(a.export_yaml, "usage", "--export-yaml required")
                result = bind(doc, a.export_yaml)
            elif a.command == "promote":
                with open(a.memory, encoding="utf8") as stream:
                    doc = json.load(stream)
                _need(doc.get("complete") is False, "not_snapshot", "only a periodic snapshot is promoted")
                doc["complete"] = True
                doc["promoted_from_snapshot"] = {"path": os.path.abspath(a.memory),
                                                 "wall": datetime.datetime.now(datetime.timezone.utc).isoformat()}
                validate_document(doc)
                result = doc
            else:
                with open(a.memory, encoding="utf8") as stream:
                    result = json.load(stream)
        text = json.dumps(result, ensure_ascii=False, allow_nan=False, indent=1) + "\n"
        if a.out and a.command != "check":
            with open(a.out, "x", encoding="utf8") as stream:
                stream.write(text)
        else:
            sys.stdout.write(text)
        return 0
    except MemoryStoreError as exc:
        print(json.dumps({"ok": False, "code": exc.code, "reason": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 3
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps({"ok": False, "code": type(exc).__name__, "reason": str(exc)}, ensure_ascii=False),
              file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
