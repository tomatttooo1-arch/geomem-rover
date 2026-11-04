#!/usr/bin/env python3
"""In-stack mapping controller (2026-10-18, car_up CAR_MAPPING=1; docs/design/TELEOP_NAV_ARB_20261018.md section 2).

    python3 tools/car_run/mapping_ctl.py --status-file F --stop-marker M [--stop-wait-s 240] -- CMD...

Starts CMD (car_inner: `rtabmap_car.sh map` with ODOM=wheel RTAB_PUBLISH_TF=false MAP_NS=rtabmap; on the host the
stand-in tools/car_run/mapping_standin.py) as its child, in its own session, output inherited (car_inner's
mapping.log), and:
  /fpga/mapping/status  std_msgs/String JSON, latched (TRANSIENT_LOCAL depth 1), on every change and every 1 s:
                       {"state": mapping | saving | exported | failed, "db_dir", "out", "yaml", "map_loaded",
                        "load_result", "reason", "standin", "since_s", "db", "db_sha256", "sha_table"}
  /fpga/mapping/stop    std_msgs/Empty ("结束建图", hub op mapping_stop): only while mapping -- writes the marker file M,
                       sends ONE SIGINT to CMD (rtabmap_car.sh saves the database and exports pgm/yaml), state saving;
                       CMD exit 0 with exactly one OUT/export/*.yaml -> exported, then /map_server/load_map
                       (nav2_msgs/srv/LoadMap) with that yaml: map_loaded true, or false + load_result/reason (the blank
                       start map stays; the stack is NOT stopped -- controller ruling 10-18); otherwise failed.
  Map identity (10-18, docs/design/MEMORY_PERSIST_20261018.md section 4): on exported, OUT/sha256.txt must name the
  database and the export yaml (host/memory_store.export_source_db -- rtabmap_car.sh writes it); when it does not
  (missing, or the database line lost), this writes it (sha256sum format, the --db database + every export file; a
  differing old table is kept as sha256.rtabmap_car.txt). db_sha256 / sha_table go into the status; car_down binds
  the run's memory file to that database (tools/car_run/mapping_memory_bind.py).
  Map alignment (10-25, owner plan A; host/map_align.py): on exported, the correction C = optimized x odometry^-1
  of every graph node (OUT/export/*poses*.txt + the --db Node table). Not identity -> published latched on
  /fpga/mapping/map_align (bench_pose.py --map-align re-sends map -> odom = C_last; ros_runtime --memory-align moves
  this run's memory), then load_map only after /tf_static shows map -> odom = C_last (or --align-wait-s passed:
  load anyway, align_applied false + reason -- the state before 10-25). Not computable -> nothing published, the
  identity stays, the reason is in status "align" and the log (never silent). Identity -> nothing published.
  Search waypoints (10-25, find-object reliability): on exported, the robot's own optimized trajectory
  (the same OUT/export/*poses*.txt -- the exported grid's frame = the map frame Nav2 and the aligned runner use) thinned
  to search goals (search_waypoints below: >= 1 m apart, <= 6, none within 0.5 m of the start, each facing the
  trajectory's centroid) -> status "search_waypoints" (ros_runtime --search-waypoints-from-mapping takes them from the
  latched status) and --search-waypoints-out (JSON, the run directory). Places the car stood on: reachable, mapped.
  Search nodes (10-26, probabilistic search, host/search_belief.py): the same trajectory thinned to >= 0.5 m apart,
  <= 80, own heading -> status "search_nodes" {"ok", "nodes": [[x, y, yaw]], "spacing_m", "frame"} (or ok false +
  reason); "search_waypoints" unchanged (the MVP fallback and patrol).
  Map reset (10-26「重置地图」, host/map_reset.py): /fpga/mapping/reset (String JSON {"id", "align_key"}, from the
  runner) in mapping / exported / failed -> state resetting, then, in order: map -> odom back to identity (latched
  /fpga/mapping/map_align {"key": align_key, identity}, waited on /tf_static like the stop's alignment) when the stop
  had published an alignment; Nav2's start map (--start-map-yaml, load_map) when the stop had loaded the export;
  the map itself -- mapping: RTAB-Map's /rtabmap/reset (std_srvs/Empty: database emptied, graph and map -> odom
  reset, the process and the camera stay up), on no answer or after exported/failed: a NEW mapping process
  (CMD with the --run-id replaced by a new one: new database DB_DIR/<new>.db, new OUT; the old one killed --
  its map is being discarded); the display assembler's reset (--display-reset-service, ui/geomem live_mapping).
  Then state mapping, "session" + 1, "reset" {"id", "ok", "method", "steps", "reason_zh", "elapsed_s"}; the export
  fields (yaml, map_loaded, align, search_waypoints, search_nodes, db_sha256) are cleared. saving: refused (status
  reset.ok false, refused "mapping_saving"). A step that fails is reported, not rolled back; the mapping
  process not running at the end -> state failed (the stack is not stopped).
  Live alignment (10-27, --live-align, car_up CAR_LIVE_ALIGN=1 default; host/live_align.py,
  docs/design/LIVE_ALIGN_20261027.md): while mapping, rtabmap's /mapData (rtabmap_msgs/MapData) gives the optimized
  graph + each new node's odometry pose; the same per-node correction is computed live and, when it changed (> 2 cm /
  0.5 deg, >= 2 s apart), published on /fpga/mapping/map_align with key "live:<run>:<session>:<seq>", "seq", "live"
  true (bench_pose re-sends map -> odom, the runner moves its memory by the difference to the previous alignment; no
  wait, no load_map). Every message of this process then carries "seq" (live, the stop's, the reset's identity); the
  stop's alignment is published even when it is identity if a live one was sent (map -> odom must return); status
  "live_align". A problem (no /mapData, root check) only means no live message (status reason + log). Off
  (--live-align absent): everything as before 10-27.
  The process keeps running after exported/failed (the status stays published; car_inner's supervision sees it alive).
  CMD exiting while still mapping (no stop asked) -> failed, exit 3 (car_inner fail-stop, as any child).
  SIGINT/SIGTERM (the stack stops): while mapping, the same stop as above (no load_map), wait up to --stop-wait-s for
  CMD, exit 0; CMD still running then -> left alone (killing it would lose the database), exit 4.
MappingCore holds the state machine (tested without ROS). Python 3.8 / 3.10.
"""
import argparse
import glob
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

STATUS_TOPIC = "/fpga/mapping/status"
STOP_TOPIC = "/fpga/mapping/stop"
RESET_TOPIC = "/fpga/mapping/reset"           # 10-26 map reset (host/map_reset.MAPPING_RESET_TOPIC)
LOAD_SERVICE = "/map_server/load_map"
RTABMAP_RESET_SERVICE = "/rtabmap/reset"     # rtabmap_slam (node rtabmap, root namespace; 0.23.7: ~2.4 s, checked)
DISPLAY_RESET_SERVICE = "/operator_map/map_assembler/reset"   # ui/geomem start_live_mapping.sh's map_assembler
IDENTITY_POSE7 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# search waypoints / dense nodes from the mapping trajectory (10-25 / 10-26): host/trajectory_nodes.py since 10-29
# (the runner thins its own trajectory with the same functions while mapping, before an export exists)
from host.trajectory_nodes import (SEARCH_SPACING_M, SEARCH_MAX, SEARCH_MIN_FROM_START_M,  # noqa: E402,F401
                                   SEARCH_CENTRE_MIN_M, NODES_SPACING_M, NODES_MAX, search_waypoints, dense_nodes)


def poses_nodes(poses_path):
    """[(stamp, x, y, yaw)] of a format-11 poses file (host/map_align.read_opt_poses)."""
    from host import map_align
    nodes = []
    for _nid, (stamp, m) in map_align.read_opt_poses(poses_path).items():
        nodes.append((stamp, float(m[0, 3]), float(m[1, 3]), math.atan2(float(m[1, 0]), float(m[0, 0]))))
    return nodes


RESET_REFUSE_ZH = {"mapping_saving": "地图正在保存，请等保存完成后再重置", "busy": "上一次重置地图还没结束",
                   "bad_request": "重置请求格式不对"}
RESET_STEP_ZH = {"map_odom": "坐标变换复位", "nav_map": "导航地图换回起步空白图", "rtabmap_reset": "RTAB-Map 清图",
                 "respawn": "重开建图进程", "display_reset": "三维显示清空"}


def new_run_id(old, clock=time.time, taken=()):
    """A RUN_TIME for a new mapping process: %Y%m%d_%H%M%S now, never the old one nor a taken one."""
    base = time.strftime("%Y%m%d_%H%M%S", time.localtime(clock()))
    run, k = base, 1
    while run == old or run in taken:
        k += 1
        run = "%s_r%d" % (base, k)
    return run


def respawn_args(cmd, out, db, old_run, new_run):
    """(cmd, out, db) of the new mapping process: every occurrence of the old run id replaced (car_inner names the
    database DB_DIR/<run>.db and OUT DB_DIR/run_<run> after it; rtabmap_car.sh refuses an existing database)."""
    if not old_run:
        raise ValueError("no --run-id: the mapping command cannot be restarted under a new name")
    if not any(old_run in c for c in cmd):
        raise ValueError("run id %s not in the mapping command" % old_run)
    sub = lambda v: None if v is None else v.replace(old_run, new_run)   # noqa: E731
    return [c.replace(old_run, new_run) for c in cmd], sub(out), sub(db)


class MappingCore(object):
    """mapping -> saving -> exported | failed; load_map result recorded on exported. No ROS, no processes."""

    def __init__(self, out, db_dir, standin=False, clock=time.monotonic, db=None, waypoints_out=None, live=None):
        self.out, self.db_dir, self.standin, self.clock = out, db_dir, bool(standin), clock
        self.live = live           # 10-27 host/live_align.LiveAlign (--live-align) or None (off: as before)
        self.map_odom_sent = None  # 10-27 (live only): the map_odom of the last alignment message published
        self.waypoints_out = waypoints_out
        self.search = None         # {"ok", "waypoints", "frame", "source"} or {"ok": False, "reason"}
        self.nodes = None          # 10-26 {"ok", "nodes", "spacing_m", "frame"} or {"ok": False, "nodes": [], "reason"}
        self.db = db
        self.db_sha256 = None
        self.sha_table = None
        self.state, self.since = "mapping", clock()
        self.yaml = None
        self.reason = None
        self.map_loaded = None
        self.load_result = None
        self.fatal = False
        self.align = None          # host/map_align.compute result, or {"ok": False, "reason"}
        self.align_applied = None  # True: /tf_static showed map -> odom = C_last before load_map
        self.session = 0           # 10-26: + 1 per map reset
        self.reset = None          # the last reset's summary (status "reset")
        self.reset_from = None     # state the running reset started from

    def _set(self, state, reason=None):
        self.state, self.since, self.reason = state, self.clock(), reason

    def request_stop(self):
        """True when this request starts the save (only while mapping)."""
        if self.state != "mapping":
            return False
        self._set("saving")
        return True

    def find_yaml(self):
        found = sorted(glob.glob(os.path.join(self.out, "export", "*.yaml")))
        return found[0] if len(found) == 1 else None, found

    def on_child_exit(self, rc):
        if self.state == "mapping":
            self._set("failed", "mapping process exited without a stop request (rc=%s)" % rc)
            self.fatal = True
            return
        if self.state != "saving":
            return
        if rc != 0:
            self._set("failed", "mapping process exit %s (database not saved or export failed: mapping.log)" % rc)
            return
        yml, found = self.find_yaml()
        if yml is None:
            self._set("failed", "export: %d yaml file(s) in %s/export (exactly one expected)" % (len(found), self.out))
            return
        self.yaml = yml
        self.identify()
        self.compute_align()
        self.compute_search_waypoints()
        self._set("exported")

    def compute_search_waypoints(self):
        """self.search from the export's optimized poses (+ the --search-waypoints-out file); never raises."""
        try:
            from host import map_align
            if self.standin:
                raise map_align.AlignError("stand-in mapping: no rtabmap trajectory")
            poses, found = map_align.find_poses(self.out)
            if poses is None:
                raise map_align.AlignError("export: %d poses file(s) in %s/export (exactly one expected)"
                                           % (len(found), self.out))
            nodes = poses_nodes(poses)
            try:
                dense = dense_nodes(nodes)
                self.nodes = ({"ok": True, "nodes": dense, "spacing_m": NODES_SPACING_M, "frame": "map"} if dense
                              else {"ok": False, "nodes": [], "reason": "empty trajectory"})
            except Exception as exc:              # noqa: BLE001 - the MVP's waypoints still stand
                self.nodes = {"ok": False, "nodes": [], "reason": "%s" % exc}
            wps = search_waypoints(nodes)
            if not wps:
                raise map_align.AlignError("trajectory of %d node(s) has no point %.1f m from its start"
                                           % (len(nodes), SEARCH_MIN_FROM_START_M))
            self.search = {"ok": True, "waypoints": wps, "frame": "map", "nodes": len(nodes),
                           "source": os.path.abspath(poses)}
        except Exception as exc:                  # noqa: BLE001 - no waypoints: the runner's fallback, said why
            self.search = {"ok": False, "waypoints": [], "reason": "%s" % exc}
            if self.nodes is None:
                self.nodes = {"ok": False, "nodes": [], "reason": "%s" % exc}
        if self.waypoints_out:
            try:
                with open(self.waypoints_out + ".tmp", "w") as f:
                    json.dump(self.search, f)
                os.replace(self.waypoints_out + ".tmp", self.waypoints_out)
            except OSError as exc:
                self.search = dict(self.search, file_error=repr(exc))

    def compute_align(self):
        """self.align from the export; never raises (an alignment problem must not lose the export)."""
        try:
            from host import map_align
            if self.standin:
                raise map_align.AlignError("stand-in mapping: no rtabmap graph")
            poses, found = map_align.find_poses(self.out)
            if poses is None:
                raise map_align.AlignError("export: %d poses file(s) in %s/export (exactly one expected)"
                                           % (len(found), self.out))
            a = map_align.compute(poses, self.db)
            self.align = dict(a, ok=True, identity=map_align.is_identity(a))
        except Exception as exc:                  # noqa: BLE001 - AlignError or anything else: identity stays
            self.align = {"ok": False, "reason": "%s" % exc}

    def align_message(self):
        """The /fpga/mapping/map_align payload, or None (nothing to publish: no alignment, or identity)."""
        a = self.align
        if not a or not a.get("ok"):
            return None
        if a.get("identity") and not self.map_odom_moved():
            return None
        msg = {"key": self.yaml, "map_odom": a["map_odom"], "node_id": a["node_id"], "nodes": a["nodes"],
               "count": a["count"], "root": a["root"], "source": a["source"]}
        if self.live is not None:              # 10-27: one ordering for live / stop / reset messages
            msg["seq"] = self.live.next_seq()
        return msg

    def map_odom_moved(self):
        """10-27 live: a published alignment left map -> odom off identity (always False with live off)."""
        return self.live is not None and self.map_odom_sent is not None \
            and [float(v) for v in self.map_odom_sent] != IDENTITY_POSE7

    def on_live_map_data(self, nodes, graph, map_to_odom=None):
        """10-27: one /mapData (host/live_align.tables_from_map_data) -- only while mapping."""
        if self.live is None or self.state != "mapping":
            return None
        return self.live.on_map_data(nodes, graph, map_to_odom)

    def live_message(self, now=None):
        """10-27: the live alignment message to publish now (only while mapping), or None."""
        if self.live is None or self.state != "mapping":
            return None
        msg = self.live.poll(now)
        if msg is not None:
            self.map_odom_sent = list(msg["map_odom"])
        return msg

    def on_align_applied(self, ok, reason=None):
        self.align_applied = bool(ok)
        if not ok:
            self.align = dict(self.align or {}, applied_reason=reason)

    def align_status(self):
        a = self.align
        if a is None:
            return None
        if not a.get("ok"):
            return {"ok": False, "reason": a.get("reason")}
        return {"ok": True, "identity": a.get("identity"), "map_odom": a["map_odom"], "node_id": a["node_id"],
                "count": a["count"], "root": a["root"], "unmatched": a.get("unmatched"),
                "applied": self.align_applied, "applied_reason": a.get("applied_reason")}

    def identify(self):
        """db_sha256 from OUT/sha256.txt (the memory_store convention), written here when it does not name the
        database and this yaml. Never fails the export: a problem is recorded in sha_table."""
        table = os.path.join(self.out, "sha256.txt")
        try:
            from host.memory_store import export_source_db, sha256_file
        except Exception as e:   # an import failure must not end the stack (fail-stop) at the moment of export
            self.sha_table = "not checked (memory_store import: %s)" % e
            return
        try:
            db_sha, _db_path, note = export_source_db(self.yaml, sha256_file(self.yaml))
            if db_sha:
                self.db_sha256, self.sha_table = db_sha, "ok (%s)" % note
                return
            if not self.db or not os.path.isfile(self.db):
                self.sha_table = "not usable (%s) and no database file %s to write one" % (note, self.db)
                return
            if os.path.exists(table):
                os.replace(table, os.path.join(self.out, "sha256.rtabmap_car.txt"))
            exp = os.path.join(self.out, "export")
            rows = []
            for path in [self.db] + [os.path.join(exp, n) for n in sorted(os.listdir(exp))]:
                h = hashlib.sha256()
                with open(path, "rb") as f:
                    for block in iter(lambda: f.read(1 << 20), b""):
                        h.update(block)
                rows.append("%s  %s\n" % (h.hexdigest(), path))
            with open(table + ".tmp", "w") as f:
                f.write("".join(rows))
                f.flush()
                os.fsync(f.fileno())
            os.replace(table + ".tmp", table)
            db_sha, _db_path, note2 = export_source_db(self.yaml, sha256_file(self.yaml))
            self.db_sha256 = db_sha
            self.sha_table = ("written by mapping_ctl (%s)" % note) if db_sha else ("written, still unusable: %s" % note2)
        except Exception as exc:                  # noqa: BLE001 - the export itself stands; car_down will not bind
            self.sha_table = "error: %r" % (exc,)

    # ---------------------------------------------------------------- 10-26 map reset
    def begin_reset(self, req_id):
        """None when the reset starts (state resetting), else the refusal key (nothing changed)."""
        if self.state == "saving" or (self.state == "exported" and self.map_loaded is None):
            return "mapping_saving"               # still saving / aligning / loading the export
        if self.state == "resetting":
            return "busy"
        if self.state not in ("mapping", "exported", "failed"):
            return "state_%s" % self.state
        self.reset_from = self.state
        self._set("resetting")
        if self.live is not None:
            self.live.reset()                     # 10-27: the old graph's nodes go; no live message while resetting
        self.reset = {"id": req_id, "ok": None, "from": self.reset_from, "steps": []}
        return None

    def reset_plan(self):
        """Steps of the running reset, in order (see the module note)."""
        a = self.align or {}
        plan = []
        if (a.get("ok") and not a.get("identity")) or self.map_odom_moved():
            plan.append("map_odom")               # the stop (10-27: or a live one) published an alignment: identity
        if self.map_loaded:
            plan.append("nav_map")                # Nav2 has the export: back to the start map
        plan.append("rtabmap_reset" if self.reset_from == "mapping" else "respawn")
        plan.append("display_reset")
        return plan

    def refuse_reset(self, req_id, key):
        self.reset = {"id": req_id, "ok": False, "refused": key, "steps": [],
                      "reason_zh": RESET_REFUSE_ZH.get(key, key)}

    def finish_reset(self, steps, running, elapsed_s, out=None, db=None):
        """steps [{"step", "ok", "ms", "reason_zh"?}]; running: the mapping process is up afterwards."""
        failed = [s for s in steps if s.get("ok") is False]
        method = next((s["step"] for s in steps if s["step"] in ("rtabmap_reset", "respawn") and s.get("ok")), None)
        if out is not None:
            self.out = out
        if db is not None:
            self.db = db
        self.session += 1
        self.yaml = self.map_loaded = self.load_result = None
        self.align = self.align_applied = None
        self.search = self.nodes = None
        self.db_sha256 = self.sha_table = None
        self.fatal = False
        if self.live is not None:
            self.live.reset()                     # 10-27: the new map's graph starts empty
        reason = "；".join("%s：%s" % (RESET_STEP_ZH.get(s["step"], s["step"]), s.get("reason_zh") or "失败")
                          for s in failed) or None
        self.reset = {"id": (self.reset or {}).get("id"), "ok": not failed and running, "from": self.reset_from,
                      "method": method, "steps": steps, "elapsed_s": round(elapsed_s, 2),
                      "reason_zh": reason if running else ("建图进程没有起来" + ("；" + reason if reason else ""))}
        self._set("mapping" if running else "failed",
                  None if running else "map reset: the mapping process is not running (%s)" % reason)
        self.reset_from = None
        if self.waypoints_out:
            try:
                with open(self.waypoints_out + ".tmp", "w") as f:
                    json.dump({"ok": False, "waypoints": [], "reason": "map reset (session %d)" % self.session}, f)
                os.replace(self.waypoints_out + ".tmp", self.waypoints_out)
            except OSError:
                pass

    def on_load(self, ok, result=None, reason=None):
        self.map_loaded, self.load_result = bool(ok), result
        if not ok:
            self.reason = "load_map failed, the blank start map stays: %s" % (reason or "result %s" % result)

    def status(self):
        row = {"state": self.state, "db_dir": self.db_dir, "out": self.out, "yaml": self.yaml,
                "map_loaded": self.map_loaded, "load_result": self.load_result, "reason": self.reason,
                "standin": self.standin, "since_s": round(self.clock() - self.since, 1),
                "db": self.db, "db_sha256": self.db_sha256, "sha_table": self.sha_table,
                "align": self.align_status(), "search_waypoints": self.search, "search_nodes": self.nodes,
                "session": self.session, "reset": self.reset}
        if self.live is not None:                 # 10-27 (absent with live off: the status as before)
            row["live_align"] = self.live.status()
        return row


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--" not in argv:
        sys.stderr.write("mapping_ctl: give the mapping command after --\n")
        return 2
    i = argv.index("--")
    own, cmd = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--status-file", required=True)
    ap.add_argument("--stop-marker", required=True)
    ap.add_argument("--out", required=True, help="the mapping OUT directory (export/*.yaml is looked for there)")
    ap.add_argument("--db-dir", required=True)
    ap.add_argument("--db", default=None, help="the database the mapping process writes (DB_DIR/<RUN_TIME>.db)")
    ap.add_argument("--stop-wait-s", type=float, default=240.)
    ap.add_argument("--load-timeout-s", type=float, default=20.)
    ap.add_argument("--standin", action="store_true")
    ap.add_argument("--align-wait-s", type=float, default=10.,
                    help="wait this long for /tf_static map -> odom = the alignment before load_map")
    ap.add_argument("--search-waypoints-out", default=None,
                    help="write the search waypoints taken from the trajectory here (JSON) on exported")
    # 10-26 map reset (module note)
    ap.add_argument("--run-id", default=None,
                    help="the RUN_TIME in CMD / --out / --db (car_inner MP_RUN): a reset restarts CMD under a new one")
    ap.add_argument("--start-map-yaml", default=None,
                    help="Nav2's start map (car_inner's blank map): a reset after the stop's load_map loads it back")
    ap.add_argument("--rtabmap-reset-service", default=RTABMAP_RESET_SERVICE)
    ap.add_argument("--display-reset-service", default=DISPLAY_RESET_SERVICE)
    ap.add_argument("--reset-wait-s", type=float, default=10., help="answer wait of the RTAB-Map reset service")
    ap.add_argument("--respawn-ready-s", type=float, default=25.,
                    help="a new mapping process must offer the RTAB-Map reset service (= its node is up) in this")
    ap.add_argument("--live-align", action="store_true",
                    help="10-27: align map -> odom and the runner's memory while mapping, from rtabmap's /mapData")
    ap.add_argument("--live-map-data-topic", default="/mapData")
    a = ap.parse_args(own)
    if not cmd:
        ap.error("empty mapping command")

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Empty, String
    from nav2_msgs.srv import LoadMap
    from std_srvs.srv import Empty as EmptySrv
    from tf2_msgs.msg import TFMessage
    from host import map_align

    live = None
    if a.live_align:
        from host import live_align
        live = live_align.LiveAlign(run=a.run_id or "")
    core = MappingCore(a.out, a.db_dir, a.standin, db=a.db, waypoints_out=a.search_waypoints_out, live=live)
    proc = {"child": subprocess.Popen(cmd, start_new_session=True), "run": a.run_id, "cmd": list(cmd)}
    stopping = []
    resets = []

    def on_signal(signum, frame):
        stopping.append(signum)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)
    node = rclpy.create_node("fpga_mapping_ctl")
    log = node.get_logger()
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(String, STATUS_TOPIC, latched)
    client = node.create_client(LoadMap, LOAD_SERVICE)
    last = [None]
    load = {"future": None, "t": None}
    align_pub = node.create_publisher(String, map_align.TOPIC, latched)
    wait = {"t": None, "want": None, "seen": None}

    def on_tf_static(msg):
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                t, q = tr.transform.translation, tr.transform.rotation
                wait["seen"] = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

    node.create_subscription(TFMessage, "/tf_static", on_tf_static,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))

    def request_load():
        if client.wait_for_service(timeout_sec=2.0):
            req = LoadMap.Request()
            req.map_url = core.yaml
            load["future"], load["t"] = client.call_async(req), time.monotonic()
        else:
            core.on_load(False, reason="service %s not available" % LOAD_SERVICE)
            publish()

    def publish(force=False):
        row = core.status()
        key = json.dumps({k: v for k, v in row.items() if k != "since_s"}, sort_keys=True)
        if force or key != last[0]:
            if key != last[0]:
                with open(a.status_file + ".tmp", "w") as f:
                    json.dump(row, f)
                os.replace(a.status_file + ".tmp", a.status_file)
                log.info("mapping status: %s" % key)
            last[0] = key
        pub.publish(String(data=json.dumps(row)))

    def stop_child(why):
        if core.request_stop():
            with open(a.stop_marker, "w") as f:
                f.write("%s %d\n" % (why, time.time()))
            os.kill(proc["child"].pid, signal.SIGINT)
            log.warn("mapping stop (%s): SIGINT to the mapping process, saving + export" % why)
            publish()

    node.create_subscription(Empty, STOP_TOPIC, lambda _m: stop_child("mapping_stop"), 10)

    # ------------------------------------------------------------ 10-27 live alignment (module note)
    if live is not None:
        if a.standin:
            live.reason = "stand-in mapping: no rtabmap graph"
        else:
            try:
                from rtabmap_msgs.msg import MapData

                def on_map_data(msg):
                    try:
                        nodes, graph, m2o = live_align.tables_from_map_data(msg)
                        core.on_live_map_data(nodes, graph, m2o)
                    except Exception as exc:     # noqa: BLE001 - one bad message: reported, mapping goes on
                        live.reason = "mapData: %s: %s" % (type(exc).__name__, exc)

                node.create_subscription(MapData, a.live_map_data_topic, on_map_data, QoSProfile(
                    depth=5, reliability=ReliabilityPolicy.RELIABLE))
                log.info("live alignment on: %s" % a.live_map_data_topic)
            except Exception as exc:             # noqa: BLE001 - no rtabmap_msgs: the stop's alignment still runs
                live.reason = "no live alignment (%s: %s)" % (type(exc).__name__, exc)
                log.error(live.reason)

    # ------------------------------------------------------------ 10-26 map reset (module note)
    def on_reset(msg):
        try:
            doc = json.loads(msg.data)
            if not isinstance(doc, dict) or not isinstance(doc.get("id"), str) or not doc["id"]:
                raise ValueError("no id")
        except ValueError as exc:
            log.error("map reset request refused (%s): %r" % (exc, msg.data[:200]))
            return
        resets.append(doc)

    node.create_subscription(String, RESET_TOPIC, on_reset, 10)
    empty_clients = {}

    def spin_for(pred, timeout_s):
        """Spin (the status still re-sent every 1 s) until pred() or timeout_s; -> pred()."""
        end = time.monotonic() + timeout_s
        while not pred():
            if time.monotonic() >= end:
                return bool(pred())
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_pub[0]:
                next_pub[0] = time.monotonic() + 1.0
                publish(True)
        return True

    def empty_client(name):
        if name not in empty_clients:
            empty_clients[name] = node.create_client(EmptySrv, name)
        return empty_clients[name]

    def call_empty(name, wait_s, timeout_s):
        c = empty_client(name)
        if not c.wait_for_service(timeout_sec=wait_s):
            return None, "服务 %s 不在" % name
        f = c.call_async(EmptySrv.Request())
        if not spin_for(f.done, timeout_s):
            return False, "服务 %s %.0f 秒内没有回应" % (name, timeout_s)
        try:
            f.result()
        except Exception as exc:                 # noqa: BLE001
            return False, "服务 %s 出错：%r" % (name, exc)
        return True, None

    def reset_map_odom(doc):
        wait["t"] = None                          # a stop's alignment wait (if any) is over
        msg = {"key": doc.get("align_key") or "reset:%s" % doc["id"], "map_odom": IDENTITY_POSE7, "node_id": 0,
               "nodes": [], "count": 0, "root": None, "source": "map_reset", "reset": True}
        if live is not None:
            msg["seq"] = live.next_seq()
            core.map_odom_sent = list(IDENTITY_POSE7)
        align_pub.publish(String(data=json.dumps(msg)))
        ok = spin_for(lambda: wait["seen"] is not None and all(
            abs(x - y) <= 1e-6 for x, y in zip(wait["seen"], IDENTITY_POSE7)), a.align_wait_s)
        return ok, None if ok else ("map -> odom %.0f 秒内没变回单位变换（bench_pose --map-align 在运行吗？）"
                                    % a.align_wait_s), {"key": msg["key"]}

    def reset_nav_map(_doc):
        load["future"] = None
        if not a.start_map_yaml:
            return False, "没有起步空白图（mapping_ctl 未给 --start-map-yaml）", {}
        if not client.wait_for_service(timeout_sec=2.0):
            return False, "服务 %s 不在" % LOAD_SERVICE, {}
        req = LoadMap.Request()
        req.map_url = a.start_map_yaml
        f = client.call_async(req)
        if not spin_for(f.done, a.load_timeout_s):
            return False, "load_map %.0f 秒内没有回应" % a.load_timeout_s, {}
        res = f.result()
        ok = res.result == LoadMap.Response.RESULT_SUCCESS
        return ok, None if ok else "load_map 结果 %d" % res.result, {"yaml": a.start_map_yaml}

    def reset_rtabmap(_doc):
        if a.standin:
            return False, "替身建图没有 RTAB-Map 清图服务", {}
        if proc["child"].poll() is not None:
            return False, "建图进程已退出", {}
        ok, why = call_empty(a.rtabmap_reset_service, 2.0, a.reset_wait_s)
        return bool(ok), why, {"service": a.rtabmap_reset_service}

    def reset_respawn(_doc):
        old = proc["child"]
        if old.poll() is None:                    # the old map is being discarded: no save, no export
            try:
                os.killpg(old.pid, signal.SIGKILL)
            except OSError:
                pass
            try:
                old.wait(5.0)
            except subprocess.TimeoutExpired:
                return False, "旧建图进程 5 秒内没有退出", {}
        new = new_run_id(proc["run"])
        try:
            cmd2, out2, db2 = respawn_args(proc["cmd"], core.out, core.db, proc["run"], new)
        except ValueError as exc:
            return False, "不能用新名字重开建图进程：%s" % exc, {}
        os.makedirs(out2, exist_ok=True)
        p = subprocess.Popen(cmd2, start_new_session=True)
        proc.update(child=p, run=new, cmd=cmd2, out=out2, db=db2)
        if a.standin:
            spin_for(lambda: p.poll() is not None, 1.0)
            ok = p.poll() is None
        else:
            svc = empty_client(a.rtabmap_reset_service)
            spin_for(lambda: p.poll() is not None or svc.service_is_ready(), a.respawn_ready_s)
            ok = p.poll() is None and svc.service_is_ready()
        why = None
        if not ok:
            why = ("新建图进程退出（rc=%s，看 mapping.log）" % p.poll() if p.poll() is not None
                   else "新建图进程 %.0f 秒内没有起来（没有 %s）" % (a.respawn_ready_s, a.rtabmap_reset_service))
        return ok, why, {"run": new, "out": out2, "db": db2}

    def reset_display(_doc):
        ok, why = call_empty(a.display_reset_service, 0.5, 3.0)
        if ok is None:
            why = "三维显示通道未启动（没有 %s），跳过" % a.display_reset_service
        return ok, why, {"service": a.display_reset_service}

    RESET_STEPS = {"map_odom": reset_map_odom, "nav_map": reset_nav_map, "rtabmap_reset": reset_rtabmap,
                   "respawn": reset_respawn, "display_reset": reset_display}

    def do_reset(doc):
        t0 = time.monotonic()
        key = core.begin_reset(doc["id"])
        if key is not None:
            core.refuse_reset(doc["id"], key)
            log.warn("map reset %s refused: %s (state %s)" % (doc["id"], key, core.state))
            publish()
            return
        log.warn("map reset %s: from %s, plan %s" % (doc["id"], core.reset_from, core.reset_plan()))
        publish()
        plan, steps = core.reset_plan(), []
        proc.pop("out", None)
        proc.pop("db", None)
        while plan:
            name = plan.pop(0)
            t = time.monotonic()
            try:
                ok, why, extra = RESET_STEPS[name](doc)
            except Exception as exc:              # noqa: BLE001 - reported, the next steps still run
                ok, why, extra = False, "%s: %s" % (type(exc).__name__, exc), {}
            row = {"step": name, "ok": ok, "ms": round((time.monotonic() - t) * 1000.0, 1)}
            if why:
                row["reason_zh"] = why
            row.update(extra or {})
            steps.append(row)
            core.reset["steps"] = steps
            (log.info if ok is not False else log.error)("map reset %s step %s" % (doc["id"], json.dumps(row)))
            if name == "rtabmap_reset" and not ok:
                plan.insert(0, "respawn")         # no answer: a new mapping process instead
            publish()
        core.finish_reset(steps, proc["child"].poll() is None, time.monotonic() - t0,
                          out=proc.get("out"), db=proc.get("db"))
        log.warn("map reset %s done: %s" % (doc["id"], json.dumps(core.reset)))
        publish()

    rc = 0
    next_pub = [time.monotonic() + 1.0]
    try:
        publish(True)
        tick = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if resets and not stopping:
                do_reset(resets.pop(0))
            lmsg = None if stopping else core.live_message()
            if lmsg is not None:                 # 10-27 live alignment: map -> odom + the runner's memory follow
                align_pub.publish(String(data=json.dumps(lmsg)))
                log.warn("live map alignment %s: map -> odom = %s (node %d, %d nodes, change %.3f m / %.2f deg)" % (
                    lmsg["key"], [round(v, 4) for v in lmsg["map_odom"]], lmsg["node_id"], lmsg["count"],
                    lmsg["change"]["m"], math.degrees(lmsg["change"]["rad"])))
                publish()
            if stopping and core.state == "mapping":
                stop_child("stack_stop")
            exit_rc = proc["child"].poll()
            if exit_rc is not None and core.state in ("mapping", "saving"):
                core.on_child_exit(exit_rc)
                publish()
                if core.fatal:
                    log.error(core.reason)
                    rc = 3
                    break
                if core.state == "exported" and not stopping:
                    msg = core.align_message()
                    if msg is None:
                        al = core.align or {}
                        if not al.get("ok"):
                            log.error("map alignment NOT computed, map -> odom stays identity: %s" % al.get("reason"))
                        request_load()
                    else:
                        # map -> odom first (bench_pose re-sends it), the new map only once Nav2's TF shows it
                        log.warn("map alignment: map -> odom = %s (node %d, %d nodes) -- waiting for /tf_static "
                                 "before load_map" % (msg["map_odom"], msg["node_id"], msg["count"]))
                        align_pub.publish(String(data=json.dumps(msg)))
                        if live is not None:
                            core.map_odom_sent = list(msg["map_odom"])
                        wait["t"], wait["want"] = time.monotonic(), msg["map_odom"]
            if wait["t"] is not None:
                seen = wait["seen"]
                if seen is not None and all(abs(x - y) <= 1e-6 for x, y in zip(seen, wait["want"])):
                    wait["t"] = None
                    core.on_align_applied(True)
                    log.info("map alignment applied (map -> odom on /tf_static); load_map")
                    publish()
                    request_load()
                elif time.monotonic() - wait["t"] > a.align_wait_s:
                    wait["t"] = None
                    why = "map -> odom not seen on /tf_static in %.0f s (last %s): bench_pose --map-align running?" % (
                        a.align_wait_s, seen)
                    core.on_align_applied(False, why)
                    log.error("map alignment not applied, map loaded unaligned: %s" % why)
                    publish()
                    request_load()
            f = load["future"]
            if f is not None:
                if f.done():
                    load["future"] = None
                    try:
                        res = f.result()
                        ok = res.result == LoadMap.Response.RESULT_SUCCESS
                        core.on_load(ok, int(res.result))
                    except Exception as exc:             # transport/service error: reported, the blank map stays
                        core.on_load(False, reason=repr(exc))
                    publish()
                elif time.monotonic() - load["t"] > a.load_timeout_s:
                    load["future"] = None
                    core.on_load(False, reason="no answer in %.0f s" % a.load_timeout_s)
                    publish()
            if stopping:
                if exit_rc is not None or proc["child"].poll() is not None:
                    break
                if time.monotonic() - tick > a.stop_wait_s:
                    log.error("mapping process still saving after %.0f s: left running" % a.stop_wait_s)
                    rc = 4
                    break
            else:
                tick = time.monotonic()
            if time.monotonic() >= next_pub[0]:
                next_pub[0] = time.monotonic() + 1.0
                publish(True)
    finally:
        try:
            publish(True)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
