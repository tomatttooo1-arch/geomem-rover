"""dev3's offline M4 draft; not a shared ROS/M2/M7 interface.

Reuse Config's metric/quality defaults and Memory's mutual-unique association
rule. Keep the public Frame, Observation, Memory and their online users intact.
Raw observations are immutable; map positions are derived from absolute poses.
No inference, depth projection, free-space inference, or movement confirmation.
"""
from contextlib import contextmanager
from copy import deepcopy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np

from perception.fpga_replay.core import Config


SCHEMA = "dev3.m4.store.v1"


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name}: nonempty string required")
    return value


def label_name(value):
    return " ".join(text(value, "label").lower().split())


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name}: finite number required")
    return value


def vector(value):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("position: three numbers required")
    return np.array([number(v, "position") for v in value], dtype=float)


def transform(value):
    a = np.asarray(value, dtype=float)
    if a.shape != (4, 4) or not np.isfinite(a).all():
        raise ValueError("T_map_local: finite 4x4 matrix required")
    r = a[:3, :3]
    if (not np.allclose(a[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-6, rtol=0)):
        raise ValueError("T_map_local: proper rigid transform required")
    return a


def references(refs):
    if not isinstance(refs, list) or not refs:
        raise ValueError("evidence_refs: nonempty provenance required")
    for ref in refs:
        if not isinstance(ref, dict):
            raise ValueError("evidence reference must be an object")
        text(ref.get("path"), "evidence path")
        sha = text(ref.get("sha256"), "evidence sha256")
        if len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
            raise ValueError("evidence sha256: lowercase SHA256 required")


def physical_key(acq):
    # Conservative: identical RGB bytes in one source are never independent
    # physical evidence, even if an adapter changes the frame ID or run name.
    identity = [acq["source_id"], acq.get("image_sha256") or acq["id"]]
    return digest(identity)


def batch_fingerprint(batch):
    value = deepcopy(batch)
    if value["acquisition"].get("image_sha256"):
        # A replay tool may rename the acquisition while retaining exact input
        # content. The recorded original ID stays in the first imported raw row.
        value["acquisition"].pop("id", None)
    return digest(value)


def pose_key(map_id, keyframe_id):
    return canonical([map_id, keyframe_id])


def scope(batch):
    a = batch["acquisition"]
    return [batch["map_id"], a["source_id"], a["clock_domain"], a["calibration_id"]]


class OfflineMemory:
    def __init__(self, config=None):
        defaults = Config()
        cfg = {"association_m": defaults.association_m,
               "min_valid_fraction": defaults.min_valid_fraction,
               "class_conflict_m": .10}
        if config:
            if set(config) - set(cfg):
                raise ValueError("unknown configuration field")
            cfg.update(config)
        if (number(cfg["association_m"], "association_m") <= 0
                or not 0 <= number(cfg["min_valid_fraction"], "min_valid_fraction") <= 1
                or number(cfg["class_conflict_m"], "class_conflict_m") <= 0):
            raise ValueError("invalid association configuration")
        self.data = {"schema": SCHEMA, "config": cfg, "maps": {}, "poses": {},
                     "acquisitions": {}, "observations": {}, "objects": {},
                     "events": [], "revisions": {}}
        self._loaded_path = None
        self._loaded_hash = None

    @staticmethod
    def _map(row):
        if not isinstance(row, dict):
            raise ValueError("map must be an object")
        for key in ("map_id", "map_version", "world_frame", "pose_source", "source_kind", "calibration_id"):
            text(row.get(key), key)
        if row.get("units") != "m":
            raise ValueError("map units must be m")
        references([row.get("source_ref", {})])

    @staticmethod
    def _pose(row, maps):
        if not isinstance(row, dict):
            raise ValueError("pose must be an object")
        for key in ("map_id", "keyframe_id", "version", "frame_id", "source"):
            text(row.get(key), key)
        if row["map_id"] not in maps:
            raise ValueError("pose references unknown map")
        transform(row.get("T_map_local"))
        references(row.get("evidence_refs"))

    def import_document(self, doc):
        candidate = deepcopy(self)
        result = candidate._import(doc)
        self.data = candidate.data
        return result

    def _import(self, doc):
        if not isinstance(doc, dict) or doc.get("schema") != "dev3.m4.input.v1":
            raise ValueError("unsupported input schema")
        canonical(doc)  # Reject NaN/Infinity before any mutation.
        if not isinstance(doc.get("maps", []), list) or not isinstance(doc.get("poses", []), list):
            raise ValueError("maps and poses must be lists")
        declared_maps = {}
        for row in doc.get("maps", []):
            self._map(row)
            if row["map_id"] in declared_maps:
                raise ValueError("duplicate map declaration")
            declared_maps[row["map_id"]] = row
            old = self.data["maps"].get(row["map_id"])
            if old and old != row:
                # Re-import of immutable original observations after revision is
                # permitted only when original map metadata agrees exactly.
                if row not in self._known_maps(row["map_id"]):
                    raise ValueError("map metadata collision; use explicit revision")
            elif not old:
                self.data["maps"][row["map_id"]] = deepcopy(row)
        for row in doc.get("poses", []):
            self._pose(row, self.data["maps"])
            key = pose_key(row["map_id"], row["keyframe_id"])
            old = self.data["poses"].get(key)
            if old:
                if row != old["versions"].get(row["version"]):
                    raise ValueError("pose collision; use explicit revision")
            else:
                declaration = declared_maps.get(row["map_id"])
                if declaration != self.data["maps"][row["map_id"]]:
                    raise ValueError("new pose requires explicit current map metadata")
                self.data["poses"][key] = {"original": deepcopy(row), "current": deepcopy(row),
                                            "versions": {row["version"]: deepcopy(row)}, "pending": False}
        reports = []
        if not isinstance(doc.get("batches"), list):
            raise ValueError("batches must be a list")
        for batch in doc["batches"]:
            reports.append(self._batch(batch))
        self._refresh()
        return {"ok": True, "batches": reports, "objects": len(self.data["objects"]),
                "observations": len(self.data["observations"])}

    def _known_maps(self, map_id):
        return [self.data["maps"][map_id]] + [r["previous_map"] for r in self.data["revisions"].values()
                                             if r["request"]["map_id"] == map_id]

    def _validate_batch(self, batch):
        if not isinstance(batch, dict):
            raise ValueError("batch must be an object")
        map_id = text(batch.get("map_id"), "map_id")
        text(batch.get("map_version"), "map_version")
        if map_id not in self.data["maps"]:
            raise ValueError("unknown map")
        acq = batch.get("acquisition", {})
        if not isinstance(acq, dict):
            raise ValueError("acquisition must be an object")
        if "timestamp" not in acq or "time_source" not in acq:
            raise ValueError("explicit timestamp/time_source required; use null when unknown")
        for k in ("id", "source_id", "clock_domain", "calibration_id"):
            text(acq.get(k), "acquisition." + k)
        sha = acq.get("image_sha256")
        if sha is not None:
            references([{"path": "acquisition image", "sha256": sha}])
        if acq.get("timestamp") is not None:
            number(acq["timestamp"], "acquisition timestamp")
            text(acq.get("time_source"), "time_source")
        elif acq.get("time_source") is not None:
            text(acq["time_source"], "time_source")
        if acq["calibration_id"] != self.data["maps"][map_id]["calibration_id"]:
            raise ValueError("calibration incompatible with map")
        text(batch.get("prediction_set"), "prediction_set")
        obs = batch.get("observations")
        if not isinstance(obs, list) or batch.get("status") not in ("ok", "failed"):
            raise ValueError("status and observations required")
        if batch["status"] == "failed":
            text(batch.get("error"), "failed batch error")
            if obs:
                raise ValueError("failed batch cannot contain usable observations")
        elif batch.get("error") is not None:
            raise ValueError("ok batch cannot have an error")
        instance_ids = set()
        for o in obs:
            if not isinstance(o, dict):
                raise ValueError("observation must be an object")
            instance = o.get("instance_id")
            if isinstance(instance, bool) or not isinstance(instance, (str, int)) or not str(instance):
                raise ValueError("single-frame instance ID required")
            if str(instance) in instance_ids:
                raise ValueError("duplicate instance ID in prediction set")
            instance_ids.add(str(instance))
            text(o.get("instance_kind"), "instance_kind")
            text(o.get("source"), "observation source")
            if o.get("label") != label_name(o.get("label")):
                raise ValueError("label must use canonical lowercase name")
            if not 0 <= number(o.get("score"), "class score") <= 1:
                raise ValueError("class score outside [0,1]")
            references(o.get("evidence_refs"))
            g = o.get("geometry", {})
            if not isinstance(g, dict):
                raise ValueError("geometry must be an object")
            if not isinstance(g.get("valid"), bool):
                raise ValueError("geometry.valid must be boolean")
            if not g["valid"]:
                text(g.get("reason"), "invalid geometry reason")
                continue
            if g.get("units") != "m":
                raise ValueError("geometry units must be m")
            text(g.get("frame_id"), "geometry frame")
            text(g.get("summary_method"), "summary_method")
            if not 0 <= number(g.get("valid_fraction"), "valid_fraction") <= 1:
                raise ValueError("valid_fraction outside [0,1]")
            local = g.get("position_local_m") is not None
            world = g.get("position_m") is not None
            if local == world:
                raise ValueError("exactly one local or map position required")
            vector(g["position_local_m"] if local else g["position_m"])
            if local:
                text(g.get("keyframe_id"), "keyframe_id")
                text(g.get("pose_version"), "pose_version")
            elif g["frame_id"] != self.data["maps"][map_id]["world_frame"]:
                raise ValueError("unanchored position must declare map world_frame")

    def _derive(self, item):
        g, batch = item["raw"]["geometry"], item["batch"]
        if not g["valid"]:
            return None, "invalid_geometry:" + g["reason"], None
        if g["valid_fraction"] < self.data["config"]["min_valid_fraction"]:
            return None, "insufficient_geometry_quality", None
        map_row = self.data["maps"][batch["map_id"]]
        if g.get("position_local_m") is not None:
            pose = self.data["poses"].get(pose_key(batch["map_id"], g["keyframe_id"]))
            if not pose or pose["pending"]:
                return None, "pending_pose_revision", None
            declared = pose["versions"].get(g["pose_version"])
            current = pose["current"]
            if not declared or g["frame_id"] != declared["frame_id"]:
                return None, "incompatible_pose_version_or_frame", None
            T = transform(current["T_map_local"])
            p = T[:3, :3] @ vector(g["position_local_m"]) + T[:3, 3]
            return p.tolist(), "derived_from_local_anchor", current["version"]
        if batch["map_version"] != map_row["map_version"]:
            return None, "pending_revision_missing_local_anchor", None
        return deepcopy(g["position_m"]), "provided_map_summary", None

    def _batch(self, batch):
        self._validate_batch(batch)
        batch = deepcopy(batch)
        physical = physical_key(batch["acquisition"])
        identity = digest([physical, batch["map_id"], batch["prediction_set"]])
        fingerprint = batch_fingerprint(batch)
        # IDs are not caller-controlled; changed bytes under one prediction
        # identity are an explicit conflict, never silently overwritten.
        prior = self.data["acquisitions"].get(identity)
        if prior:
            if prior["fingerprint"] != fingerprint:
                raise ValueError("acquisition/prediction collision with different content")
            return {"physical_id": physical, "status": "duplicate", "decisions": []}
        if batch["map_version"] != self.data["maps"][batch["map_id"]]["map_version"]:
            raise ValueError("new acquisition uses stale map version")
        for previous in self.data["acquisitions"].values():
            old_acq = previous["batch"]["acquisition"]
            acq = batch["acquisition"]
            if (old_acq["source_id"] == acq["source_id"] and old_acq["id"] == acq["id"]
                    and old_acq.get("image_sha256") != acq.get("image_sha256")):
                raise ValueError("acquisition ID reused with different image content")
        same_physical = [x for x in self.data["acquisitions"].values()
                         if x["physical_id"] == physical and x["batch"]["map_id"] == batch["map_id"]]
        alternate = bool(same_physical)
        self.data["acquisitions"][identity] = {"fingerprint": fingerprint, "physical_id": physical,
                                                "batch": batch}
        if batch["status"] == "failed" or not batch["observations"]:
            event = {"physical_id": physical, "status": "failed" if batch["status"] == "failed" else "empty",
                     "error": batch.get("error"), "decisions": []}
            self.data["events"].append(event)
            return event
        self._refresh()
        new = []
        meta = {k: deepcopy(v) for k, v in batch.items() if k != "observations"}
        for raw in batch["observations"]:
            oid = digest([identity, str(raw["instance_id"])])
            item = {"id": oid, "raw": deepcopy(raw), "batch": deepcopy(meta), "physical_id": physical,
                    "object_id": None, "decision": "pending", "candidate_ids": [], "conflicts": []}
            p, why, pv = self._derive(item)
            item.update(position_m=p, geometry_status=why, pose_version=pv)
            self.data["observations"][oid] = item
            new.append(item)
        cfg = self.data["config"]
        existing = list(self.data["objects"].values())
        possible = {}
        for item in new:
            if item["position_m"] is None:
                continue
            p, name = np.array(item["position_m"]), item["raw"]["label"]
            matches = []
            for obj in existing:
                if obj["scope"] != scope(batch) or obj["position_m"] is None:
                    continue
                dist = float(np.linalg.norm(p - obj["position_m"]))
                if obj["label"] == name and dist <= cfg["association_m"]:
                    matches.append(obj["id"])
                elif obj["label"] != name and dist <= cfg["class_conflict_m"]:
                    item["conflicts"].append(obj["id"])
                    obj["class_conflicts"].append(item["id"])
            possible[item["id"]] = matches
            for other in new:
                if (other is not item and other["position_m"] is not None
                        and other["raw"]["label"] != name
                        and np.linalg.norm(p - other["position_m"]) <= cfg["class_conflict_m"]):
                    item["conflicts"].append(other["id"])
        counts = {obj["id"]: sum(obj["id"] in ids for ids in possible.values()) for obj in existing}
        decisions = []
        for item in new:
            ids = possible.get(item["id"], [])
            item["candidate_ids"] = ids
            if item["position_m"] is None:
                item["decision"] = item["geometry_status"]
            elif alternate:
                item["decision"] = "alternate_prediction_same_physical_capture"
            elif item["conflicts"]:
                item["decision"] = "class_conflict_pending"
            elif len(ids) > 1 or (ids and counts[ids[0]] > 1):
                item["decision"] = "ambiguous_association"
            else:
                if ids:
                    obj = self.data["objects"][ids[0]]
                    item["decision"] = "associated_mutual_unique"
                else:
                    key = f"object-{len(self.data['objects'])+1:04d}"
                    obj = {"id": key, "label": item["raw"]["label"], "scope": scope(batch),
                           "observation_ids": [], "class_conflicts": []}
                    self.data["objects"][key] = obj
                    item["decision"] = "new_candidate"
                item["object_id"] = obj["id"]
                obj["observation_ids"].append(item["id"])
            decisions.append({k: deepcopy(item[k]) for k in ("id", "object_id", "decision", "candidate_ids", "conflicts")})
        event = {"physical_id": physical, "status": "ok", "decisions": decisions}
        self.data["events"].append(event)
        self._refresh()
        return event

    def _refresh(self):
        for item in self.data["observations"].values():
            p, why, pv = self._derive(item)
            item.update(position_m=p, geometry_status=why, pose_version=pv)
        for obj in self.data["objects"].values():
            items = [self.data["observations"][key] for key in obj["observation_ids"]]
            known = [x for x in items if x["batch"]["acquisition"]["timestamp"] is not None]
            # Never let an unknown or older time overwrite newer evidence.
            latest = max(known, key=lambda x: (x["batch"]["acquisition"]["timestamp"], x["id"])) if known else items[0]
            acq = latest["batch"]["acquisition"]
            map_row = self.data["maps"][obj["scope"][0]]
            pending = any(x["position_m"] is None for x in items)
            obj.update(map_id=map_row["map_id"], map_version=map_row["map_version"],
                       world_frame=map_row["world_frame"], units="m", pose_source=map_row["pose_source"],
                       position_m=None if pending else deepcopy(latest["position_m"]),
                       last_seen=acq["timestamp"], last_seen_frame=acq["id"], time_source=acq.get("time_source"),
                       physical_observation_count=len({x["physical_id"] for x in items}),
                       position_method="latest_dated_observation_summary" if known else "first_unknown_time_summary",
                       state="candidate", reason="identity_and_class_not_independently_confirmed")
            if pending:
                obj.update(state="pending_revision", reason="one_or_more_observations_have_no_valid_map_position")
            elif obj["class_conflicts"]:
                obj.update(state="needs_review", reason="class_conflict_no_score_voting")
            elif any(np.linalg.norm(np.array(x["position_m"])-obj["position_m"]) > self.data["config"]["association_m"]
                     for x in items):
                obj.update(state="needs_review", reason="associated_observations_spatially_inconsistent")
            obj["evidence_refs"] = [deepcopy(r) for x in items for r in x["raw"]["evidence_refs"]]

    def revise(self, doc):
        candidate = deepcopy(self)
        result = candidate._revise(doc)
        self.data = candidate.data
        return result

    def _revise(self, doc):
        if not isinstance(doc, dict) or doc.get("schema") != "dev3.m4.revision.v1":
            raise ValueError("unsupported revision schema")
        canonical(doc)
        for k in ("revision_id", "map_id", "from_version", "to_version", "reason"):
            text(doc.get(k), k)
        prior = self.data["revisions"].get(doc["revision_id"])
        if prior:
            if prior["request"] != doc:
                raise ValueError("revision ID collision")
            return {"ok": True, "status": "duplicate_revision"}
        if doc["map_id"] not in self.data["maps"]:
            raise ValueError("unknown revision map")
        m = self.data["maps"][doc["map_id"]]
        if m["map_version"] != doc["from_version"] or doc["to_version"] == doc["from_version"]:
            raise ValueError("revision version chain mismatch")
        used_versions = {r["request"]["from_version"] for r in self.data["revisions"].values()
                         if r["request"]["map_id"] == doc["map_id"]}
        if doc["to_version"] in used_versions:
            raise ValueError("map version cannot be recycled")
        updates = {}
        for row in doc.get("poses", []):
            self._pose(row, self.data["maps"])
            key = pose_key(row["map_id"], row["keyframe_id"])
            if row["map_id"] != doc["map_id"] or key not in self.data["poses"] or key in updates:
                raise ValueError("unknown or duplicate revision keyframe")
            old = self.data["poses"][key]
            if row["frame_id"] != old["original"]["frame_id"] or row["version"] == old["current"]["version"]:
                raise ValueError("frame mismatch or unchanged pose version")
            if row["version"] in old["versions"]:
                raise ValueError("pose version cannot be recycled")
            updates[key] = row
        unchanged = doc.get("unchanged_keyframes")
        if not isinstance(unchanged, list) or len(set(unchanged)) != len(unchanged):
            raise ValueError("explicit unchanged_keyframes list required")
        unchanged = {pose_key(doc["map_id"], text(k, "unchanged keyframe")) for k in unchanged}
        if unchanged & updates.keys() or any(k not in self.data["poses"] for k in unchanged):
            raise ValueError("invalid unchanged keyframe declaration")
        before = deepcopy(self.query())
        self.data["revisions"][doc["revision_id"]] = {"request": deepcopy(doc), "previous_map": deepcopy(m)}
        m["map_version"] = doc["to_version"]
        for key, row in self.data["poses"].items():
            if row["original"]["map_id"] != doc["map_id"]:
                continue
            if key in updates:
                row["current"] = deepcopy(updates[key]); row["pending"] = False
                row["versions"][updates[key]["version"]] = deepcopy(updates[key])
            elif key not in unchanged:
                row["pending"] = True
        self._refresh()
        changes = [{"id": old["id"], "before_m": old["position_m"],
                    "after_m": self.data["objects"][old["id"]]["position_m"],
                    "state": self.data["objects"][old["id"]]["state"]}
                   for old in before["objects"] if old["map_id"] == doc["map_id"]]
        result = {"ok": True, "status": "revised", "revision_id": doc["revision_id"], "changes": changes}
        self.data["events"].append(deepcopy(result))
        return result

    def query(self, label=None):
        name = label_name(label) if label is not None else None
        objects = [deepcopy(x) for x in self.data["objects"].values() if name is None or x["label"] == name]
        pending = [deepcopy(x) for x in self.data["observations"].values()
                   if x["object_id"] is None and (name is None or x["raw"]["label"] == name)]
        return {"ok": True, "category": name, "objects": objects, "pending_observations": pending,
                "maps": deepcopy(list(self.data["maps"].values())),
                "scope": "offline_candidates_not_current_presence_or_navigation"}

    @contextmanager
    def _lock(self, path):
        with open(str(path) + ".lock", "a", encoding="utf8") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            yield

    def save(self, path):
        path = Path(path).absolute()
        payload = {"schema": SCHEMA, "sha256": digest(self.data), "data": self.data}
        body = (json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()
        with self._lock(path):
            if path.exists():
                current = hashlib.sha256(path.read_bytes()).hexdigest()
                if self._loaded_path != str(path) or current != self._loaded_hash:
                    raise ValueError("store changed or exists without load; refusing overwrite")
            elif self._loaded_path == str(path):
                raise ValueError("loaded store disappeared")
            fd, tmp = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
            try:
                with os.fdopen(fd, "wb") as stream:
                    stream.write(body); stream.flush(); os.fsync(stream.fileno())
                os.replace(tmp, path)
            finally:
                if os.path.exists(tmp):
                    os.unlink(tmp)
            self._loaded_path = str(path)
            self._loaded_hash = hashlib.sha256(body).hexdigest()

    @classmethod
    def load(cls, path):
        path = Path(path).absolute()
        body = path.read_bytes()
        wrapper = json.loads(body)
        if not isinstance(wrapper, dict) or wrapper.get("schema") != SCHEMA or not isinstance(wrapper.get("data"), dict):
            raise ValueError("unsupported store schema")
        data = wrapper["data"]
        if wrapper.get("sha256") != digest(data) or data.get("schema") != SCHEMA:
            raise ValueError("store integrity mismatch")
        instance = cls(data["config"])
        for k in instance.data:
            if k not in data or not isinstance(data[k], type(instance.data[k])):
                raise ValueError("invalid store structure")
        instance.data = data
        for m in data["maps"].values():
            instance._map(m)
        for p in data["poses"].values():
            instance._pose(p["original"], data["maps"])
            instance._pose(p["current"], data["maps"])
            for version in p["versions"].values():
                instance._pose(version, data["maps"])
        for obj in data["objects"].values():
            if not obj["observation_ids"] or any(k not in data["observations"] for k in obj["observation_ids"]):
                raise ValueError("broken observation links")
            if any(data["observations"][k]["object_id"] != obj["id"] for k in obj["observation_ids"]):
                raise ValueError("broken object links")
        original = deepcopy(data)
        instance._refresh()
        if original != instance.data:
            raise ValueError("stored derived data inconsistent with raw evidence")
        instance._loaded_path = str(path)
        instance._loaded_hash = hashlib.sha256(body).hexdigest()
        return instance
