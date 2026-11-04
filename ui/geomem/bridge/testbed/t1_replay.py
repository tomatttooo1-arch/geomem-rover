"""T1 试验台：``fpgac.ReplayServer`` 播一份车上真实录制的 ``pc.fpgarec``（fpgac 没有命令行入口，这里补一个最小的）。

**只在 ``unshare -rn`` 隔离网络命名空间里跑**（宿主任何接口都不监听 hub 端口，owner 10-14）。
用法：``python t1_replay.py <pc.fpgarec> <口令文件> [--port 47431] [--rate 1.0]``；SIGTERM 退出。
回放只作工程验证，不是比赛或答辩画面（owner 10-14）。
"""

import argparse
import os
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from pc import fpgac                     # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("token_file")
    ap.add_argument("--port", type=int, default=fpgac.DEFAULT_PORT)
    ap.add_argument("--rate", type=float, default=1.0)
    args = ap.parse_args()
    if not os.environ.get("GEOMEM_TB_NETNS"):
        raise SystemExit("拒绝运行：只在 unshare -rn 隔离网络命名空间里跑（由 run_tb.sh 设 GEOMEM_TB_NETNS=1）")
    with open(args.token_file, "r", encoding="utf-8") as f:
        token = f.read().strip()
    srv = fpgac.ReplayServer(args.rec, token, host="127.0.0.1", port=args.port, rate=args.rate).start()
    sys.stdout.write("replay up port=%d boot=%s rec=%s\n" % (srv.port, srv.boot_id, args.rec))
    sys.stdout.flush()
    stop = []
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    try:
        while not stop:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    srv.close()
    sys.stdout.write("replay stopped counts=%s\n" % dict(srv.counts))
    sys.stdout.flush()


if __name__ == "__main__":
    main()
