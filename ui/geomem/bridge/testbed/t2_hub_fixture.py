"""T2 试验台：无 ROS 的 hub 夹具（只 import host/，不改它）。

进程内 ``HubCore(<data-source>, actions=RosActionsLog()) + HubServer(host=127.0.0.1)``，按场景往 hub 喂
STATE/MEMORY/MAP 的输入（与 hub_ros 喂的同一组 ``feed_*``），再加一个「替身 runner」：看 ``actions.published``
里的 /fpga/query、/fpga/select_candidate、/fpga/cancel，回喂 task 事件，让 hub 的 2 s 观察窗出 CMD_RESULT。

**只在 ``unshare -rn`` 隔离网络命名空间里跑**（宿主任何接口都不监听 hub 端口，owner 10-14）。
控制口 ``http://127.0.0.1:<ctl-port>/ctl?op=...``（同在命名空间里），返回 JSON 状态（含真值）：

  op=state                         只读状态（scenario、published、ros_now、objects 真值 lastSeen）
  op=tier&tier=loc|lost|odom|none  定位档（estimated+valid）/定位丢失/里程档（reference）/判不出
  op=map&kind=real|placeholder     全局栅格：有占据格的真图 / 全 -1 的 8 m 替身（rev 自增）
  op=objects&set=default|single|<n> 记忆内容（default：两个杯子+瓶子+书包+键盘+手机；single：只一个杯子）
  op=scope&id=<runtime_boot_id>    换记忆作用域（物体重新编号同名）
  op=source&sensor=hardware|replay 喂 startup 事件（live hub 收 replay → 整帧 inconsistent）
  op=bad&kind=<A3 变体>&hold=<秒>  坏 MEMORY（nan_pos/dup_id/over_1000/inf_score/bad_frame/nulls/weird_state）保持 hold 秒后自动恢复
  op=hold&s=<秒>                   占住 server.lock N 秒（hub 收命令不回执 → 桥 no_ack 的仪器）
  op=runner&on=0|1                 替身 runner 开关（关了 hub 观察窗到点出 unknown）
  op=task&state=..&reason=..&target=..&cands=<类名>   直接喂一条 task 事件（可带候选）
  op=squat                         起一个占控制权的 FpgacClient（须在桥连上之前）
  op=yield                         占位客户端让出控制权（hub yield_control）
  op=clear                         清 published 记录
  —— 展台（10-17，kiosk_*.mjs 用）——
  op=objects&file=<json>&age=<秒>   记忆换成文件里的物体列表（如 map3d-look 的 memory_final.json），last_seen = 现在 − age
  op=pose&x=..&y=..&yaw=..         位姿固定在这一点（定位档 loc_pose 与 odom 同喂）；op=pose&mode=circle 恢复默认绕圈
  op=walk&file=<gm_poses.txt>&upto=<i>&dt=<秒>  沿建图轨迹（按 stamp 排序）从第 0 点走到第 i 点，每 dt 秒一步，停在第 i 点
  op=kv&geo=<ms>&rows=<n>&hb=0|1&roi_age=<ms>   每拍喂一条 KV TELEM（几何核每帧、ROI 行数及其年龄、许可/停车心跳位）；op=kv&off=1 停喂
"""

import argparse
import json
import math
import os
import random
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from pc import fpgac                                   # noqa: E402
from host.hub.hub_core import HubCore                 # noqa: E402
from host.hub.hub_server import HubServer, RosActionsLog, read_token   # noqa: E402

ZH = {"杯子": "cup", "杯": "cup", "瓶子": "bottle", "瓶": "bottle", "书包": "backpack", "背包": "backpack",
      "书": "book", "鼠标": "mouse", "键盘": "keyboard", "手机": "cell phone"}
COLORS = ("red", "blue", "green", "yellow", "white", "black")
ROS_T0 = 86400.0          # 车上 ROS 时间轴（像 Nano 墙钟停在 1970），与浏览器钟毫无关系
T_MONO0 = time.monotonic()


def ros_now():
    return ROS_T0 + (time.monotonic() - T_MONO0)


OBJECT_SETS = {
    "default": [("cup", "blue", [1.2, 0.8, 0.45]), ("cup", "red", [3.4, -1.1, 0.75]),
                ("bottle", "green", [2.2, 1.9, 0.3]), ("backpack", "black", [-0.8, 2.4, 0.2]),
                ("keyboard", "white", [0.4, -2.0, 0.7]), ("cell phone", "black", [-1.6, -0.6, 0.5])],
    "single": [("cup", "blue", [1.2, 0.8, 0.45]), ("bottle", "green", [2.2, 1.9, 0.3])],
}


class Fixture(object):
    def __init__(self, args):
        self.args = args
        self.core = HubCore(args.data_source, actions=RosActionsLog())
        self.server = HubServer(self.core, read_token(args.token_file), port=args.port, host="127.0.0.1")
        self.lock = self.server.lock
        self.tier = args.tier
        self.map_kind = "placeholder" if args.tier == "odom" else "real"
        self.map_rev = 0
        self.scope = "scope-A-" + os.urandom(3).hex()
        self.revision = 1
        self.set_objects("default")
        # 10-18: --data-source sim (kiosk K7c: the sim tier) -> runner sensor "sim"
        self.sensor = {"live": "hardware", "replay": "replay", "sim": "sim"}[args.data_source]
        self.runner_on = True
        self.seen_actions = 0
        self.task = None
        self.permit = False
        self.bad_once = None
        self.bad_until = 0.0
        self.squatter = None
        self.fixed_pose = None              # (x, y, yaw) or None = circle
        self.kv = None                      # dict for op=kv
        self.kv_frame = 0
        self.log = []
        self._stop = threading.Event()

    # ------------------------------------------------------------------ scenario
    def set_objects(self, name):
        if name in OBJECT_SETS:
            spec = OBJECT_SETS[name]
        else:
            n = int(name)
            spec = [(("cup", "bottle", "book", "mouse")[k % 4], COLORS[k % 6], [0.1 * (k % 40) - 2, 0.1 * (k // 40) - 2, 0.3])
                    for k in range(n)]
        now = ros_now()
        self.objects = []
        for k, (label, color, pos) in enumerate(spec):
            self.objects.append({"id": "object-%04d" % (k + 1), "label": label, "color": color,
                                 "position_m": list(pos), "state": "current_confirmed" if k % 3 else "historical_memory",
                                 "history_len": 3 + k, "last_seen": now - (12.0 + 7.0 * k),
                                 "history_last": {"score": round(0.9 - 0.05 * (k % 8), 3)}})
        self.revision += 1

    def load_objects(self, path, age_s):
        with open(path, "r", encoding="utf-8") as f:
            rows = json.load(f)
        now = ros_now()
        self.objects = []
        for o in rows:
            self.objects.append({"id": o["id"], "label": o["label"], "color": o.get("color"),
                                 "position_m": list(o["position_m"]), "state": o.get("state"),
                                 "history_len": 3, "last_seen": now - age_s,
                                 "history_last": {"score": 0.8}})
        self.revision += 1

    def walk(self, path, upto, dt):
        pts = []
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                t, x, y, z, qx, qy, qz, qw = (float(a) for a in line.split()[:8])
                pts.append((t, x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))))
        pts.sort()
        pts = [p[1:] for p in pts[:upto + 1]]

        def run():
            for p in pts:
                with self.lock:
                    self.fixed_pose = p
                time.sleep(dt)
        threading.Thread(target=run, name="t2-walk", daemon=True).start()
        return len(pts)

    def feed_kv(self, now):
        k = self.kv
        self.kv_frame += 1
        from host.kv_status import Telemetry
        blocks = {"geometry": {"core": "pl", "last": {"frame_id": self.kv_frame, "status": 0, "counts": [256, 0, 0],
                                                      "times_ms": {"core_device": k["geo"], "core_total": k["geo"] + 1.0}}},
                  "permit": {"status": 1 if k["hb"] else 0, "frames_accepted": self.kv_frame},
                  "roi": {"core": "pl", "last": {"frame_id": self.kv_frame, "roi_status": 0, "rows": k["rows"]}}}
        self.core.feed_telemetry(Telemetry(avail=["geometry", "permit", "roi"], blocks=blocks,
                                           age_upper_ms={"geometry": 30.0, "permit": 30.0, "roi": k["roi_age"]},
                                           rtt_ms=2.0, kv_boot_id="kv-t2"), now)

    def memory_body(self):
        body = {"v": 1, "runtime_boot_id": self.scope, "revision": self.revision, "world_frame": "map",
                "sensor_source": self.sensor, "digest": "rev%d" % self.revision, "stamp_ros_s": ros_now(),
                "tracks": [dict(o) for o in self.objects]}
        if self.tier == "odom":
            body["world_frame"] = "odom"
        kind = self.bad_once
        if kind is not None and time.monotonic() >= self.bad_until:
            self.bad_once = kind = None
        if kind is None:
            return body, None
        t = body["tracks"]
        if kind == "nan_pos":
            t.append({"id": "object-0900", "label": "cup", "color": "blue", "position_m": [float("nan"), 1.0, 0.2]})
        elif kind == "dup_id":
            t.append(dict(t[0], position_m=[9.0, 9.0, 0.0]))
        elif kind == "over_1000":
            body["tracks"] = t + [{"id": "object-%04d" % (2000 + k), "label": "book", "color": "red",
                                   "position_m": [0.01 * k, 3.0, 0.1], "state": "historical_memory",
                                   "last_seen": ros_now() - 30 - k} for k in range(1001)]
        elif kind == "inf_score":
            t[0] = dict(t[0], history_last={"score": float("inf")}, last_seen=float("nan"))
        elif kind == "bad_frame":
            body["world_frame"] = "base_link"
        elif kind == "nulls":
            t[0] = {"id": t[0]["id"], "label": None, "color": None, "position_m": t[0]["position_m"],
                    "state": None, "last_seen": None, "history_last": None}
        elif kind == "weird_state":
            t[0] = dict(t[0], state="teleported")
        body["revision"] = self.revision + 1000      # 坏帧也是新 revision，hub 会转发
        return body, kind

    def grid(self, kind):
        if kind == "placeholder":
            w = h = 160
            return {"rev": self.map_rev, "width": w, "height": h, "resolution_m": 0.05,
                    "origin": [-4.0, -4.0, 0.0], "frame_id": "map"}, bytes([0xFF]) * (w * h)
        w, h = 120, 100
        cells = bytearray(w * h)          # 0 = free
        for j in range(h):
            for i in range(w):
                wall = i in (0, 1, w - 1, w - 2) or j in (0, 1, h - 1, h - 2) or (i == 60 and 20 < j < 70)
                if wall:
                    cells[j * w + i] = 100
                elif (i * 7 + j * 3) % 97 == 0:
                    cells[j * w + i] = 0xFF   # -1 unknown
        return {"rev": self.map_rev, "width": w, "height": h, "resolution_m": 0.05,
                "origin": [-3.0, -2.5, 0.0], "frame_id": "map"}, bytes(cells)

    def rolling(self, i):
        w = h = 80
        cells = bytearray(w * h)
        for j in range(h):
            for k in range(w):
                d = math.hypot(k - 40, j - 40)
                if 30 < d < 32.5 and (k + j) % 5:
                    cells[j * w + k] = 100
        return {"rev": None, "width": w, "height": h, "resolution_m": 0.05, "origin": [-2.0, -2.0, 0.0],
                "frame_id": "odom"}, bytes(cells)

    def push_map(self):
        self.map_rev += 1
        meta, cells = self.grid(self.map_kind)
        meta["rev"] = self.map_rev
        self.core.feed_map("global", meta, cells, self.server.clock())

    def push_tier(self):
        now = self.server.clock()
        lat = self.core.latest
        for key in ("loc_status", "loc_pose", "runtime_info"):
            lat.pop(key, None)
        if self.tier in ("loc", "lost"):
            self.core.feed_runtime_info(json.dumps({"pose_source": "estimated", "sensor_source": self.sensor,
                                                    "world_frame": "map"}), now)
        elif self.tier == "odom":
            self.core.feed_runtime_info(json.dumps({"pose_source": "reference", "sensor_source": self.sensor,
                                                    "world_frame": "odom"}), now)

    def feed_task(self, state, reason=None, target=None, candidates=None):
        row = {"event": "task", "state": state, "reason": reason, "target_id": target,
               "candidates": candidates or []}
        self.task = row
        self.core.feed_status(json.dumps(row), self.server.clock())
        self.log.append(("task", state, reason, target))

    # ------------------------------------------------------------------ fake runner
    def match(self, text):
        t = text.strip().lower()
        color = next((c for c in COLORS if c in t.split()), None)
        label = None
        for zh, en in sorted(ZH.items(), key=lambda kv: -len(kv[0])):
            if zh in text:
                label = en
                break
        if label is None:
            for o in self.objects:
                if o["label"] and o["label"] in t:
                    label = o["label"]
                    break
        return [o for o in self.objects if o["label"] == label and (color is None or o["color"] == color)]

    def runner_step(self):
        pub = self.core.actions.published
        while self.seen_actions < len(pub):
            topic, arg = pub[self.seen_actions]
            self.seen_actions += 1
            if not self.runner_on:
                continue
            active = self.task is not None and self.task["state"] in ("ambiguous", "navigating", "observing")
            if topic == "/fpga/query":
                if active:
                    self.core.feed_status(json.dumps({"event": "query_rejected", "reason":
                                                      "cancel current operation and wait for navigation to stop"}),
                                          self.server.clock())
                    continue
                hits = self.match(arg)
                if not hits:
                    self.feed_task("not_found", "search_exhausted")
                elif len(hits) == 1:
                    self.feed_task("navigating", "reobserve", hits[0]["id"])
                    self.permit = True
                else:
                    cands = [{k: o[k] for k in ("id", "label", "color", "position_m", "state")} for o in hits]
                    self.feed_task("ambiguous", "candidate_selection_required", None, cands)
            elif topic == "/fpga/select_candidate":
                if not (self.task and self.task["state"] == "ambiguous"):
                    self.core.feed_status(json.dumps({"event": "selection_rejected",
                                                      "reason": "no candidate selection is pending"}), self.server.clock())
                    continue
                self.feed_task("navigating", "reobserve", arg)
                self.permit = True
            elif topic == "/fpga/cancel":
                if active:
                    self.feed_task("cancelled", "user_cancelled", self.task.get("target_id"))
                self.permit = False
                self.core.feed_motion_permit(False, self.server.clock())

    # ------------------------------------------------------------------ loops
    def start(self):
        with self.lock:
            self.core.feed_status(json.dumps({"event": "startup", "sensor_source": self.sensor,
                                              "replay_id": "t2-fixture"}), self.server.clock())
            self.push_tier()
            self.push_map()
        self.server.start()
        threading.Thread(target=self.loop, name="t2-feed", daemon=True).start()
        return self

    def loop(self):
        i = 0
        while not self._stop.wait(0.1):
            i += 1
            with self.lock:
                now = self.server.clock()
                self.runner_step()
                self.core.feed_motion_permit(self.permit, now)
                self.core.feed_cmd_vel(0.15 if self.permit else 0.0, 0.0, now)
                if self.fixed_pose is not None:
                    x, y, yaw = self.fixed_pose
                else:
                    x, y, yaw = 0.3 * math.cos(i / 40.0), 0.3 * math.sin(i / 40.0), (i / 40.0) % 6.283
                if self.kv is not None:
                    self.feed_kv(now)
                if self.tier in ("loc", "lost"):
                    self.core.feed_loc_status(json.dumps({"state": "valid" if self.tier == "loc" else "lost",
                                                          "reason": None if self.tier == "loc" else "tracking_lost",
                                                          "epoch": 1}), now)
                    self.core.feed_loc_pose({"x": x, "y": y, "yaw": yaw, "cov_xy_trace": 0.01,
                                             "stamp_ros_s": ros_now()}, now)
                if self.tier in ("loc", "lost"):
                    self.core.feed_odom(0.15 if self.permit else 0.0, 0.0, ros_now(), now)
                else:   # 10-18: odometry pose rides along (hub H2) so the odometry tier shows the car
                    self.core.feed_odom(0.15 if self.permit else 0.0, 0.0, ros_now(), now, x=x, y=y, yaw=yaw,
                                        frame_id="odom")
                if i % 5 == 0:
                    body, bad = self.memory_body()
                    self.core.feed_memory(json.dumps(body), now)
                    if bad:
                        self.log.append(("bad", bad))
                if i % 3 == 0:
                    meta, cells = self.rolling(i)
                    self.core.feed_map("rolling", meta, cells, now)

    def squat(self):
        self.squatter = fpgac.FpgacClient(self.args.port, read_token(self.args.token_file), hosts=["127.0.0.1"],
                                        client_id="t2-squatter", want=("state",))
        self.squatter.start()

    # ------------------------------------------------------------------ control
    def control(self, q):
        op = q.get("op", "state")
        with self.lock:
            if op == "tier":
                self.tier = q["tier"]
                self.push_tier()
            elif op == "map":
                self.map_kind = q["kind"]
                self.push_map()
            elif op == "objects" and "file" in q:
                self.load_objects(q["file"], float(q.get("age", "120")))
            elif op == "objects":
                self.set_objects(q["set"])
            elif op == "scope":
                self.scope = q.get("id") or ("scope-" + os.urandom(3).hex())
                self.set_objects(q.get("set", "default"))
            elif op == "source":
                self.sensor = q["sensor"]
                self.core.feed_status(json.dumps({"event": "startup", "sensor_source": self.sensor,
                                                  "replay_id": "t2-fixture"}), self.server.clock())
            elif op == "bad":
                self.bad_once = q["kind"]
                self.bad_until = time.monotonic() + float(q.get("hold", "0.3"))
            elif op == "runner":
                self.runner_on = q.get("on", "1") == "1"
            elif op == "task":
                cands = None
                if q.get("cands"):          # cands=<类名>：把记忆里该类的物体作候选
                    cands = [{k: o[k] for k in ("id", "label", "color", "position_m", "state")}
                             for o in self.objects if o["label"] == q["cands"]]
                self.feed_task(q["state"], q.get("reason") or None, q.get("target") or None, cands)
                self.permit = q["state"] in ("navigating",)
            elif op == "pose":
                self.fixed_pose = None if q.get("mode") == "circle" else (float(q["x"]), float(q["y"]),
                                                                         float(q.get("yaw", "0")))
            elif op == "kv":
                self.kv = None if q.get("off") == "1" else {"geo": float(q.get("geo", "12")),
                                                            "rows": int(q.get("rows", "8")),
                                                            "hb": q.get("hb", "1") == "1",
                                                            "roi_age": float(q.get("roi_age", "200"))}
            elif op == "clear":
                del self.core.actions.published[:]
                self.seen_actions = 0
        if op == "hold":
            s = float(q.get("s", "4"))
            def hold():
                with self.lock:
                    time.sleep(s)
            threading.Thread(target=hold, daemon=True).start()
            time.sleep(0.05)
            return {"held_s": s}
        if op == "walk":
            n = self.walk(q["file"], int(q.get("upto", "16")), float(q.get("dt", "0.1")))
            return dict(self.state(), walk_points=n)
        if op == "squat":
            self.squat()
            time.sleep(1.0)
        if op == "yield":
            cid = self.squatter.command("yield_control", {})
            ack = self.squatter.wait_ack(cid, 3.0)
            return {"yield_ack": ack}
        return self.state()

    def state(self):
        with self.lock:
            now = ros_now()
            ctl = self.server.controller
            return {"tier": self.tier, "map_kind": self.map_kind, "map_rev": self.map_rev, "scope": self.scope,
                    "revision": self.revision, "sensor": self.sensor, "hub_boot_id": self.core.hub_boot_id,
                    "frame_source": self.core.frame_source(),
                    "controller": None if ctl is None else ctl.client_id,
                    "published": [list(p) for p in self.core.actions.published],
                    "task": self.task, "permit": self.permit, "ros_now": now,
                    "fixed_pose": self.fixed_pose, "kv": self.kv,
                    "objects": [{"id": o["id"], "label": o["label"], "lastSeen": o["last_seen"],
                                 "ageS": now - o["last_seen"]} for o in self.objects[:20]],
                    "n_objects": len(self.objects), "log": self.log[-30:]}


def serve_control(fx, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            u = urlparse(self.path)
            if u.path != "/ctl":
                self.send_error(404)
                return
            q = {k: v[-1] for k, v in parse_qs(u.query).items()}
            try:
                body = json.dumps(fx.control(q), default=repr).encode()
                code = 200
            except Exception as e:      # noqa: BLE001 -- report to the test script
                body, code = json.dumps({"error": repr(e)}).encode(), 500
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = HTTPServer(("127.0.0.1", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=47431)
    ap.add_argument("--ctl-port", type=int, default=19291)
    ap.add_argument("--token-file", required=True)
    ap.add_argument("--data-source", default="live", choices=("live", "replay", "sim"))
    ap.add_argument("--tier", default="loc", choices=("loc", "lost", "odom", "none"))
    args = ap.parse_args()
    if not os.environ.get("GEOMEM_TB_NETNS"):
        raise SystemExit("拒绝运行：只在 unshare -rn 隔离网络命名空间里跑（由 run_tb.sh 设 GEOMEM_TB_NETNS=1）")
    fx = Fixture(args).start()
    serve_control(fx, args.ctl_port)
    sys.stdout.write("fixture up hub=127.0.0.1:%d boot=%s ctl=%d data_source=%s\n" % (
        fx.server.port, fx.core.hub_boot_id, args.ctl_port, args.data_source))
    sys.stdout.flush()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    random.seed(0)
    main()
