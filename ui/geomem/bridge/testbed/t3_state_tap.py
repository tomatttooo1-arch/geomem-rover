#!/usr/bin/env python3
"""T3 仪器：STATE 旁听（GeoMem 联调第 2 棒，A5 真 runner 版）。

桥连上 hub 之后再连的 FPGAC 客户端（observer），每帧 STATE 记一行 JSONL：
``{"t": 墙钟秒, "seq": seq_state, "task": task.state, "target": task.target_id, "permit": nav.motion_permit,
"n_tracks": memory.n_tracks, "stale": 帧是否过期}``。墙钟用 time.time()，与页面驱动脚本的 Date.now() 同一内核时钟。

  t3_state_tap.py <host:port> <口令文件> <out.jsonl> [--seconds N]

只在车的网络命名空间（试验台容器，GEOMEM_TB_NETNS=1）里跑；按 pid 停（SIGTERM）。
"""

import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from pc import fpgac  # noqa: E402

K = fpgac.KIND


def row_of(obj, stale):
    def g(group, key):
        d = obj.get(group)
        return d.get(key) if isinstance(d, dict) else None
    return {"t": time.time(), "seq": obj.get("seq_state"), "task": g("task", "state"), "target": g("task", "target_id"),
            "reason": g("task", "reason_task"), "permit": g("nav", "motion_permit"), "n_tracks": g("memory", "n_tracks"),
            "odom_v": g("odom", "v"), "odom_x": g("odom", "x"), "odom_y": g("odom", "y"),   # 第 3 棒：停下的旁证
            "stale": bool(stale)}


def main(argv=None):
    if not os.environ.get("GEOMEM_TB_NETNS"):
        raise SystemExit("拒绝运行：只在车的网络命名空间里的试验台容器中跑（GEOMEM_TB_NETNS=1）")
    ap = argparse.ArgumentParser()
    ap.add_argument("hub")
    ap.add_argument("token_file")
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=0.0)
    a = ap.parse_args(argv)
    host, port = a.hub.rsplit(":", 1)
    token = Path(a.token_file).read_text(encoding="utf-8").strip()
    out = open(a.out, "a", encoding="utf-8", buffering=1)
    lock = threading.Lock()
    stop = []

    def on_frame(frame, obj):
        if frame.kind != K["STATE"]:
            return
        line = json.dumps(row_of(obj, getattr(frame, "stale", False)), ensure_ascii=False)
        with lock:
            out.write(line + "\n")

    def on_link(state, detail):
        with lock:
            out.write(json.dumps({"t": time.time(), "link": state}) + "\n")

    # 10-16: the handler only appends (no lock). It runs on the main thread between bytecodes; an
    # Event.set() here deadlocked when it landed inside the main loop's own Event.wait() (holds the
    # Event's non-reentrant lock): immediate SIGTERM after "tap up" never exited, 3.8 12/400, 3.13 9/400.
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    c = fpgac.FpgacClient(int(port), token, hosts=[host], on_frame=on_frame, on_link=on_link,
                        want=("state",)).start()
    print("tap up", flush=True)
    t0 = time.monotonic()
    try:
        while not stop and (a.seconds <= 0 or time.monotonic() - t0 < a.seconds):
            time.sleep(0.1)
    finally:
        c.stop()
        out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
