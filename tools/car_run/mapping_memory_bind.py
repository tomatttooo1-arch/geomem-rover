#!/usr/bin/env python3
"""car_down step (2026-10-18): bind an in-stack mapping run's memory file to the database it built.

    python3 tools/car_run/mapping_memory_bind.py RUN

docs/design/TELEOP_NAV_ARB_20261018.md section 2.4; the memory file and bind() are host/memory_store.py
(docs/design/MEMORY_PERSIST_20261018.md sections 4 and 7). A CAR_MAPPING=1 run starts with map identity
mapping_pending (car_up); the runner writes RUN/memory_final.json at the stack stop with that identity; the mapping
process saves the database and exports (mapping_ctl, RUN/mapping_status.json, OUT/sha256.txt). car_down calls this
after the stack container has exited (both are on disk then). Steps, each refusal a reason (exit 1), never an exception:
  RUN/mapping_status.json absent                    -> skipped (not a mapping run; exit 0, nothing printed)
  status not exported / no yaml                     -> not bound
  RUN/memory_final.json absent                      -> not bound (CAR_MEMORY_SAVE=0, or the runner did not stop cleanly)
  its identity: rtabmap_db with bound_from          -> already bound (exit 0)
                none / anything but mapping_pending -> not bound (memory_store bind takes only pending)
  the export's sha256.txt names a database whose file (host path) is missing or whose sha256 differs -> not bound
  memory_store.bind (pending dir == the database's directory, as the stack saw both) -> bound:
    RUN/memory_final.bound.json (validated), then memory_final.json -> memory_final.pending.json (kept, never
    overwritten) and memory_final.bound.json -> memory_final.json; RUN/memory_bind.json records both sha256.
Stack paths (mapping_status.json, sha256.txt) are mapped to host paths with car.json rtabmap.db_dir_in_stack ->
rtabmap.db_dir, and /o -> RUN (the stack's /o is RUN). Prints one JSON line. Python 3.8+, standard library + host/.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host import memory_store  # noqa: E402


def host_path(path, run, car):
    """A path as the stack saw it -> the same file on this host."""
    if not path:
        return path
    rt = car.get("rtabmap") if isinstance(car.get("rtabmap"), dict) else {}
    pairs = []
    if rt.get("db_dir_in_stack") and rt.get("db_dir"):
        pairs.append((rt["db_dir_in_stack"], rt["db_dir"]))
    pairs.append(("/o", run))
    for inner, outer in pairs:
        inner = inner.rstrip("/")
        if path == inner or path.startswith(inner + "/"):
            return outer.rstrip("/") + path[len(inner):]
    return path


def bind_run(run):
    """(exit code, result dict)."""
    status_path = os.path.join(run, "mapping_status.json")
    if not os.path.isfile(status_path):
        return 0, {"bind": "skipped", "why": "no mapping_status.json (not a mapping run)"}
    out = {"bind": "not_bound"}
    try:
        with open(status_path, encoding="utf8") as f:
            status = json.load(f)
        car = {}
        if os.path.isfile(os.path.join(run, "car.json")):
            with open(os.path.join(run, "car.json"), encoding="utf8") as f:
                car = json.load(f)
        if status.get("state") != "exported" or not status.get("yaml"):
            out["why"] = "mapping state %r (no export to bind to)" % status.get("state")
            return 1, out
        yaml = host_path(status["yaml"], run, car)
        out["export_yaml"] = yaml
        mem = os.path.join(run, "memory_final.json")
        if not os.path.isfile(mem):
            out["why"] = "no memory_final.json (CAR_MEMORY_SAVE=0 or the runner did not stop cleanly)"
            return 1, out
        with open(mem, encoding="utf8") as f:
            doc = json.load(f)
        ident = doc.get("map_identity") or {}
        kind = ident.get("kind")
        out["identity_before"] = kind
        if kind == "rtabmap_db" and ident.get("bound_from"):
            return 0, {"bind": "already_bound", "db_sha256": ident.get("db_sha256")}
        if kind != "mapping_pending":
            out["why"] = "identity %r, not mapping_pending (memory_store bind takes only a pending identity)" % kind
            return 1, out
        db_sha, db_path, note = memory_store.export_source_db(yaml, memory_store.sha256_file(yaml))
        if not db_sha:
            out["why"] = "export does not name its database: %s" % note
            return 1, out
        db_host = host_path(db_path, run, car)
        out.update(db=db_host, db_sha256=db_sha)
        if not os.path.isfile(db_host):
            out["why"] = "database %s (stack %s) is not on disk" % (db_host, db_path)
            return 1, out
        if memory_store.sha256_file(db_host) != db_sha:
            out["why"] = "database %s differs from its sha256.txt line" % db_host
            return 1, out
        new = memory_store.bind(doc, yaml)
        memory_store.validate_document(new)
        bound = os.path.join(run, "memory_final.bound.json")
        pending = os.path.join(run, "memory_final.pending.json")
        if os.path.exists(bound) or os.path.exists(pending):
            out["why"] = "memory_final.bound.json / memory_final.pending.json already exist (never overwritten)"
            return 1, out
        pending_sha = memory_store.sha256_file(mem)
        memory_store.save_atomic(new, bound)
        os.rename(mem, pending)
        os.rename(bound, mem)
        out = {"bind": "bound", "db_sha256": db_sha, "db": db_host, "export_yaml": yaml,
               "memory_final_sha256": memory_store.sha256_file(mem), "pending_copy": pending,
               "pending_sha256": pending_sha, "tracks": len(new.get("tracks") or [])}
        with open(os.path.join(run, "memory_bind.json"), "w", encoding="utf8") as f:
            json.dump(out, f, indent=1)
        return 0, out
    except memory_store.MemoryStoreError as exc:
        out.update(why="%s: %s" % (exc.code, exc))
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        out.update(why="%s: %s" % (type(exc).__name__, exc))
    return 1, out


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        sys.stderr.write("usage: mapping_memory_bind.py RUN\n")
        return 2
    rc, out = bind_run(argv[0])
    if out.get("bind") != "skipped":
        print(json.dumps(out, ensure_ascii=False))
    return rc


if __name__ == "__main__":
    sys.exit(main())
