"""「重置地图」(2026-10-26, owner): clear the 3-D map and the object memory WITHOUT restarting the stack.

Chain (docs: ui/geomem/vehicle_interface_protocol.md「重置地图」):
  console button -> bridge command ``map_reset {confirm: true}`` -> hub op ``map_reset`` -> /fpga/map_reset
  (String JSON {"id": hub cmd id}) -> the runner (host/ros_runtime.py, the one that owns the task, the memory and
  the scope) runs the steps below and answers on /fpga/map_reset/status (String JSON, latched) -> hub CMD_RESULT.

Runner steps, in order, each one timed and recorded in runtime.jsonl (event ``map_reset_step``); a failed step is
reported, never rolled back, and the next steps still run:
  stop      cancel the task / patrol / goto / final approach through the existing stop path (zero /cmd_vel, permit
            false, Nav2 goal cancelled)
  memory    a fresh Memory, a NEW runtime_boot_id (= the console's memory.scope: candidates/selection clear on it),
            /fpga/memory and /fpga/runtime_info republished
  caches    map-bound caches: mapping search waypoints / nodes, the search planner and its per-map cache,
            observation views, the memory alignment pending; the alignment key of this reset marked done
  mapping   /fpga/mapping/reset -> tools/car_run/mapping_ctl.py (RTAB-Map /rtabmap/reset, or a new mapping
            process after an exported/failed session; map -> odom back to identity and Nav2's start map when the
            stop had switched them; the display assembler reset) -> its status row with reset.id = ours
  costmaps  Nav2 ClearEntireCostmap, global and local
  display   /fpga/search/belief phase "none" (host/search_belief_feed.BeliefFeed.clear), so the page drops the heat map
Memory writes are held from the start until the mapping step is answered (frames acquired before that are not
merged), so nothing seen in the old map's frame lands in the new memory.

Localization mode (a stored map, no in-stack mapping): refused, nothing touched -- there is no mapping session to
restart and clearing the memory of a stored map is not what the button promises (the hub refuses first:
map_reset_no_mapping). Pure Python, no ROS: the runner and the tests use it.
"""
import json
import time

REQUEST_TOPIC = "/fpga/map_reset"
STATUS_TOPIC = "/fpga/map_reset/status"
MAPPING_RESET_TOPIC = "/fpga/mapping/reset"       # = tools/car_run/mapping_ctl.RESET_TOPIC (tested equal)
STEPS = ("stop", "memory", "caches", "mapping", "costmaps", "display")
MAPPING_WAIT_S = 30.0          # mapping_ctl: rtabmap reset <= 10 s, or a new mapping process <= ~25 s
ALIGN_KEY_PREFIX = "reset:"

REFUSE_ZH = {
    "no_mapping": "定位模式（使用已存地图）或本趟没有启用建图，不能重置地图；要重新建图请用建图模式起栈",
    "mapping_saving": "地图正在保存，请等保存完成后再重置",
    "busy": "上一次重置地图还没结束",
    "bad_request": "重置地图请求格式不对",
    "estimated_pose": "定位模式（使用已存地图）下不能重置地图",
}
STEP_ZH = {"stop": "停车", "memory": "清空记忆", "caches": "清空地图缓存", "mapping": "重置建图",
           "costmaps": "清空导航代价地图", "display": "清空界面热力图"}


def parse_request(text):
    """{"id": str, ...} -> the dict; ValueError otherwise."""
    doc = json.loads(text)
    if not isinstance(doc, dict) or not isinstance(doc.get("id"), str) or not doc["id"] or len(doc["id"]) > 128:
        raise ValueError("map_reset request needs a string id")
    return doc


def refusal(mapping_row, mapping_age_s, busy, estimated_pose, max_age_s=5.0):
    """The REFUSE_ZH key that stops a reset before any step, or None."""
    if busy:
        return "busy"
    if estimated_pose:
        return "estimated_pose"
    if not isinstance(mapping_row, dict) or mapping_age_s is None or not 0 <= mapping_age_s <= max_age_s:
        return "no_mapping"
    if mapping_row.get("state") == "saving" or (mapping_row.get("state") == "exported"
                                                 and mapping_row.get("map_loaded") is None):
        return "mapping_saving"                   # saving / aligning / loading the export: wait for it
    if mapping_row.get("state") == "resetting":
        return "busy"
    return None


class MapResetJob(object):
    """One reset: the steps done so far, the mapping wait and the status document."""

    def __init__(self, req_id, clock=time.monotonic, wall=time.time):
        self.id, self.clock = req_id, clock
        self.t0, self.wall0 = clock(), wall()
        self.steps = []
        self.state = "running"
        self.mapping_sent_at = None
        self.mapping_session = None
        self.scope = self.previous_scope = None
        self.frames_held = 0
        self._t = self.t0

    @property
    def align_key(self):
        return ALIGN_KEY_PREFIX + self.id

    @property
    def holding_memory(self):
        """Memory writes held: from the start until the mapping step has its answer."""
        return self.state == "running" and not any(s["step"] == "mapping" for s in self.steps)

    def begin(self):
        self._t = self.clock()

    def step(self, name, ok, reason_zh=None, **detail):
        """Close the step started at the last begin()/step(): ok True | False | None (skipped)."""
        now = self.clock()
        row = {"step": name, "ok": ok, "ms": round((now - self._t) * 1000.0, 1)}
        if reason_zh:
            row["reason_zh"] = reason_zh
        if detail:
            row["detail"] = detail
        self.steps.append(row)
        self._t = now
        return row

    def mapping_timed_out(self, wait_s=MAPPING_WAIT_S):
        return self.mapping_sent_at is not None and self.clock() - self.mapping_sent_at > wait_s

    def finish(self):
        failed = [s for s in self.steps if s["ok"] is False]
        self.state = "failed" if failed else "done"
        return self.state

    def reason_zh(self):
        failed = [s for s in self.steps if s["ok"] is False]
        if not failed:
            return None
        return "；".join("%s没有完成%s" % (STEP_ZH.get(s["step"], s["step"]),
                                       "（%s）" % s["reason_zh"] if s.get("reason_zh") else "") for s in failed)

    def status(self):
        return {"v": 1, "id": self.id, "state": self.state, "steps": list(self.steps),
                "reason_zh": self.reason_zh(), "scope": self.scope, "previous_scope": self.previous_scope,
                "mapping_session": self.mapping_session, "elapsed_s": round(self.clock() - self.t0, 2),
                "started_wall": self.wall0, "frames_held": self.frames_held}


def refused_status(req_id, key):
    return {"v": 1, "id": req_id, "state": "failed", "steps": [], "refused": key,
            "reason_zh": REFUSE_ZH.get(key, key), "scope": None, "previous_scope": None, "mapping_session": None,
            "elapsed_s": 0.0}


def mapping_answer(row, job_id):
    """mapping_ctl's status row -> (finished, ok, reason_zh, session) for this reset id."""
    if not isinstance(row, dict):
        return False, None, None, None
    r = row.get("reset")
    if not isinstance(r, dict) or r.get("id") != job_id or row.get("state") == "resetting":
        return False, None, None, None
    return True, bool(r.get("ok")), r.get("reason_zh"), row.get("session")
