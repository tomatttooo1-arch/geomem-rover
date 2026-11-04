"""Signal handlers must not take a lock (10-16 sweep after kv_server 71be2c8).

A Python signal handler runs on the main thread between two bytecodes. If it calls Event.set() (or
anything else that takes a lock) while the main thread is itself inside that lock -- Event.wait(),
Event.set() in a finally, a ``with lock`` block, logging, a buffered write -- the non-reentrant lock is
never released and the process ignores SIGTERM for good. Seen: kv_server (3.8 8/200), t3_state_tap
(3.8 12/400, 3.13 9/400), chain_watch (3.8 10/400, 3.13 4/400; part of these were a TERM that tripped
just before the long Event.wait() blocked, lost until --seconds ran out).

L1: every signal.signal(sig, handler) in the repo (tests excluded) whose handler is a lambda or a def in
    the same module: the handler body (and same-module functions it calls) makes no locking call and
    has no ``with`` statement. Positive controls: the pre-fix shapes are flagged, flag/raise shapes are
    not, and the scan finds the known handlers (a blind scanner would pass trivially).
L2: chain_watch.py CLI (no hub, refused KV port 127.0.0.1:1): SIGTERM right at the wait exits rc 0
    with final.json and stopped_by_signal true (the behaviour the poll loop has to keep).
"""
import ast
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# calls that take (or may take) a lock: threading / queue / logging / buffered IO / thread or client stop
LOCKING = {"set", "clear", "wait", "wait_for", "acquire", "release", "notify", "notify_all", "join", "put",
           "put_nowait", "get", "stop", "close", "shutdown", "write", "flush", "print", "info", "warning",
           "warn", "error", "debug", "exception", "critical", "log", "emit"}


def _call_name(call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def _defs(tree):
    out = {}
    for n in ast.walk(tree):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            out.setdefault(n.name, []).append(n)
    return out


def _problems(node, defs, depth=0, seen=None):
    """Locking calls / with-blocks reachable from a handler body (same-module calls followed 2 deep)."""
    seen = set() if seen is None else seen
    body = node.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) else [node.body]
    found = []
    for stmt in body:
        for n in ast.walk(stmt):
            if isinstance(n, (ast.With, ast.AsyncWith)):
                found.append("with-block line %d" % n.lineno)
            elif isinstance(n, ast.Call):
                name = _call_name(n)
                if name in LOCKING:
                    found.append("%s() line %d" % (name, n.lineno))
                elif isinstance(n.func, ast.Name) and name in defs and depth < 2 and name not in seen:
                    seen.add(name)
                    for d in defs[name]:
                        found += _problems(d, defs, depth + 1, seen)
    return found


def scan_source(src, filename="<src>"):
    """[(line, handler description, problems)] for every resolvable signal.signal handler."""
    tree = ast.parse(src, filename)
    defs = _defs(tree)
    rows = []
    for n in ast.walk(tree):
        if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "signal"
                and isinstance(n.func.value, ast.Name) and n.func.value.id == "signal" and len(n.args) == 2):
            continue
        h = n.args[1]
        if isinstance(h, ast.Lambda):
            rows.append((n.lineno, "lambda", _problems(h, defs)))
        elif isinstance(h, ast.Name) and h.id in defs:
            probs = []
            for d in defs[h.id]:
                probs += _problems(d, defs)
            rows.append((n.lineno, h.id, probs))
    return rows


# Reviewed exceptions (file -> reason). Keep this list short: each entry was checked by hand that the main
# thread never holds the lock the handler takes.
ALLOWED = {
    # stop.set() in the handler; the main thread only calls stop.is_set() (lock-free) and sleeps, never
    # stop.wait()/set()/clear(); the only other set() runs on the fpgac thread (on_link), which a blocked
    # handler simply waits out. Not the deadlock shape; left as is in the 10-16 sweep.
    "tools/demo_shot5/shot5_view.py",
}


def repo_files():
    for base in ("host", "tools", "ui", "robot_sim", "pc", "board", "nano"):
        d = ROOT / base
        if not d.is_dir():
            continue
        for p in sorted(d.rglob("*.py")):
            if "node_modules" not in p.parts:
                yield p


class HandlerScanTests(unittest.TestCase):
    def test_positive_controls(self):
        bad = {
            "event_set": "import signal, threading\nstop = threading.Event()\n"
                         "signal.signal(signal.SIGTERM, lambda *_: stop.set())\n",
            "with_lock": "import signal, threading\nL = threading.Lock()\n"
                         "def h(s, f):\n    with L:\n        pass\nsignal.signal(signal.SIGTERM, h)\n",
            "logging": "import signal, logging\ndef h(s, f):\n    logging.info('bye')\n"
                       "signal.signal(signal.SIGINT, h)\n",
            "indirect": "import signal\ndef _term(s, f):\n    shutdown_all()\ndef shutdown_all():\n    srv.stop()\n"
                        "signal.signal(signal.SIGTERM, _term)\n",
            "print": "import signal\nsignal.signal(signal.SIGTERM, lambda *_: print('x'))\n",
        }
        for name, src in bad.items():
            rows = scan_source(src)
            self.assertEqual(len(rows), 1, name)
            self.assertTrue(rows[0][2], name)
        good = {
            "append": "import signal\nstop = []\nsignal.signal(signal.SIGTERM, lambda *_: stop.append(1))\n",
            "setitem": "import signal\nstop = {}\nsignal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('f', 1))\n",
            "raise": "import signal\ndef h(s, f):\n    raise KeyboardInterrupt\nsignal.signal(signal.SIGTERM, h)\n",
            "reinstall": "import signal\ndef h(s, f):\n    signal.signal(s, signal.SIG_IGN)\n    raise KeyboardInterrupt\n"
                         "signal.signal(signal.SIGTERM, h)\n",
        }
        for name, src in good.items():
            rows = scan_source(src)
            self.assertEqual(len(rows), 1, name)
            self.assertEqual(rows[0][2], [], name)

    def test_repo_handlers_take_no_lock(self):
        rows = []
        for p in repo_files():
            try:
                src = p.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            if "signal.signal" not in src:
                continue
            for line, what, probs in scan_source(src, str(p)):
                rows.append((str(p.relative_to(ROOT)), line, what, probs))
        files = {r[0] for r in rows}
        # the scan must see the handlers this sweep looked at (else it is blind and passes trivially)
        for f in ("tools/kv_link/kv_server.py", "tools/integ_car_chain/chain_watch.py",
                  "ui/geomem/bridge/testbed/t3_state_tap.py", "host/ros_geo_producer.py",
                  "tools/demo_shot5/shot5_view.py", "host/hub/hub_ros.py", "ui/geomem/bridge/geomem_bridge.py"):
            self.assertIn(f, files)
        self.assertGreaterEqual(len(rows), 35)
        bad = [r for r in rows if r[3] and r[0] not in ALLOWED]
        self.assertTrue(any(r[3] for r in rows if r[0] in ALLOWED))      # the exception is still needed
        self.assertEqual(bad, [], "\n".join("%s:%d %s -> %s" % r for r in bad))


class ChainWatchSignalTests(unittest.TestCase):
    def test_sigterm_at_wait_exits_clean(self):
        d = tempfile.mkdtemp(prefix="cw_sig_")
        tok = os.path.join(d, "tok")
        open(tok, "w").close()
        out = os.path.join(d, "o")
        p = subprocess.Popen([sys.executable, str(ROOT / "tools/integ_car_chain/chain_watch.py"),
                              "--server", "127.0.0.1:1", "--estop-at-s", "-1", "--seconds", "60",
                              "--token-file", tok, "--out", out],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        try:
            t_end = time.monotonic() + 30
            while not os.path.exists(os.path.join(out, "estop.json")):   # written right before the wait
                self.assertIsNone(p.poll(), "chain_watch exited before the wait")
                self.assertLess(time.monotonic(), t_end, "no estop.json")
                time.sleep(0.005)
        finally:
            p.send_signal(signal.SIGTERM)
            try:
                so, se = p.communicate(timeout=20)   # watcher HELLO 3 s + final_block HELLO 3 s
            except subprocess.TimeoutExpired:        # never leave a hung watcher behind
                p.kill()
                p.communicate()
                raise
        self.assertEqual(p.returncode, 0, se[-2000:])
        self.assertTrue(json.loads(so.strip().splitlines()[-1])["stopped_by_signal"])
        self.assertTrue(os.path.exists(os.path.join(out, "final.json")))


if __name__ == "__main__":
    unittest.main()
