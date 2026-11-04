#!/usr/bin/env python3
"""Detached exit supervisor for long M7 HLS/RTL runs (2026-09-26).

Why: the 2026-09-21 dense co-simulation ended with shell status 143 and left
no exit record, no command record, no process-group snapshot and no signal
sender, so the termination could not be attributed afterwards.  This wrapper
makes every such run leave, in a fresh run directory:

  COMMAND.txt / ENVIRONMENT.txt   exact argv, cwd and selected environment
  supervisor.json                 start time, supervisor pid/pgid/sid
  heartbeat.jsonl                 periodic process table of the session and
                                  the last RTL progress line seen in a log
  signals.jsonl                   every signal delivered to the supervisor,
                                  with the sender pid/uid from sigwaitinfo
  exit.json / exit.txt            the child's precise wait status: exit code,
                                  or the terminating signal number
  post.json                       optional post-command (classifier) and its
                                  exit code

The supervisor never forwards signals and never kills the child: it only
records.  Stop a run deliberately with ``kill -TERM -- -<pgid>`` (the whole
process group), which is itself recorded.  Launch it detached, e.g.
``setsid nohup python3 tools/m7_supervise.py --run-dir R -- cmd ... &``.
"""

import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time

WATCHED_SIGNALS = (signal.SIGTERM, signal.SIGHUP, signal.SIGINT,
                   signal.SIGQUIT, signal.SIGUSR1, signal.SIGUSR2)
PROGRESS_TOKEN = b"RTL Simulation :"


def now():
    return {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "local": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
            "epoch": round(time.time(), 3)}


def append_jsonl(path, record):
    with open(path, "a") as stream:
        stream.write(json.dumps(record, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def write_new(path, text):
    with open(path, "x") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())


def session_table(sid):
    """Every process of this session, read from /proc (no external tools)."""
    rows = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/%s/stat" % entry) as stream:
                stat = stream.read()
            close = stat.rindex(")")
            comm = stat[stat.index("(") + 1:close]
            fields = stat[close + 2:].split()
            if int(fields[3]) != sid:
                continue
            rss_kb = None
            with open("/proc/%s/status" % entry) as stream:
                for line in stream:
                    if line.startswith("VmRSS:"):
                        rss_kb = int(line.split()[1])
            rows.append({"pid": int(entry), "ppid": int(fields[1]),
                         "pgid": int(fields[2]), "state": fields[0],
                         "comm": comm, "rss_kb": rss_kb})
        except (OSError, ValueError, IndexError):
            continue
    return sorted(rows, key=lambda row: row["pid"])


class ProgressScanner:
    """Incrementally scan growing logs for the last Vitis RTL progress line."""

    def __init__(self, patterns):
        self.patterns = patterns
        self.offsets = {}
        self.last = None

    def scan(self):
        for pattern in self.patterns:
            for path in sorted(Path("/").glob(pattern.lstrip("/"))):
                try:
                    size = path.stat().st_size
                    offset = self.offsets.get(str(path), 0)
                    if size < offset:
                        offset = 0
                    with open(path, "rb") as stream:
                        stream.seek(offset)
                        chunk = stream.read(size - offset)
                    cut = chunk.rfind(b"\n")
                    if cut < 0:
                        continue
                    self.offsets[str(path)] = offset + cut + 1
                    for line in chunk[:cut].splitlines():
                        if PROGRESS_TOKEN in line:
                            self.last = {"file": str(path),
                                         "line": line.decode(errors="replace").strip()}
                except OSError:
                    continue
        return self.last


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--cwd", type=Path, help="child working directory (default run dir)")
    parser.add_argument("--heartbeat-seconds", type=float, default=60.0)
    parser.add_argument("--progress-glob", action="append", default=[],
                        help="absolute glob of logs to scan for RTL progress lines")
    parser.add_argument("--post", help="shell command run after the child exits; "
                        "its exit code is recorded in post.json")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("missing child command after --")
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    if any(run_dir.iterdir()):
        sys.exit("run directory must be fresh and empty: %s" % run_dir)
    cwd = (args.cwd or run_dir).resolve()

    # Block the watched signals in the supervisor so a dedicated thread can
    # read them with sigwaitinfo (which reports the sender).  The child gets
    # an unblocked mask so its own signal behaviour is unchanged.
    signal.pthread_sigmask(signal.SIG_BLOCK, WATCHED_SIGNALS)

    def unblock():
        signal.pthread_sigmask(signal.SIG_UNBLOCK, WATCHED_SIGNALS)

    write_new(run_dir / "COMMAND.txt", " ".join(shlex.quote(part) for part in command) + "\n"
              + "cwd=%s\n" % cwd + ("post=%s\n" % args.post if args.post else ""))
    selected = {key: value for key, value in sorted(os.environ.items())
                if key.startswith(("FPGA_", "XILINX", "LD_LIBRARY", "PATH", "HOME",
                                   "USER", "TMPDIR", "PYTHON"))}
    write_new(run_dir / "ENVIRONMENT.txt", json.dumps({
        "host": socket.gethostname(), "uname": list(os.uname()),
        "python": sys.version, "nproc": os.cpu_count(), "env": selected},
        indent=2, sort_keys=True) + "\n")
    sid = os.getsid(0)
    start = now()
    stdout = open(run_dir / "console.log", "xb")
    child = subprocess.Popen(command, cwd=str(cwd), stdout=stdout,
                             stderr=subprocess.STDOUT, preexec_fn=unblock)
    write_new(run_dir / "supervisor.json", json.dumps({
        "start": start, "supervisor_pid": os.getpid(), "supervisor_pgid": os.getpgid(0),
        "supervisor_sid": sid, "supervisor_ppid": os.getppid(),
        "child_pid": child.pid, "child_pgid": os.getpgid(child.pid),
        "heartbeat_seconds": args.heartbeat_seconds,
        "progress_globs": args.progress_glob}, indent=2, sort_keys=True) + "\n")

    done = threading.Event()

    def signal_reader():
        while True:
            info = signal.sigwaitinfo(WATCHED_SIGNALS)
            record = {"time": now(), "signal": signal.Signals(info.si_signo).name,
                      "sender_pid": info.si_pid, "sender_uid": info.si_uid,
                      "si_code": info.si_code, "child_alive": child.poll() is None,
                      "action": "recorded_only_not_forwarded"}
            try:
                with open("/proc/%d/cmdline" % info.si_pid, "rb") as stream:
                    record["sender_cmdline"] = stream.read().replace(b"\0", b" ").decode(
                        errors="replace").strip()
            except OSError:
                record["sender_cmdline"] = None
            append_jsonl(run_dir / "signals.jsonl", record)

    scanner = ProgressScanner(args.progress_glob)

    def heartbeat():
        while not done.wait(args.heartbeat_seconds):
            append_jsonl(run_dir / "heartbeat.jsonl", {
                "time": now(), "child_alive": child.poll() is None,
                "progress": scanner.scan(), "session": session_table(sid)})

    threading.Thread(target=signal_reader, daemon=True).start()
    threading.Thread(target=heartbeat, daemon=True).start()
    append_jsonl(run_dir / "heartbeat.jsonl", {"time": start, "child_alive": True,
                                               "progress": None,
                                               "session": session_table(sid)})
    returncode = child.wait()
    done.set()
    end = now()
    stdout.close()
    record = {"start": start, "end": end,
              "elapsed_seconds": round(end["epoch"] - start["epoch"], 3),
              "popen_returncode": returncode,
              "exited_normally": returncode >= 0,
              "exit_code": returncode if returncode >= 0 else None,
              "terminated_by_signal": (signal.Signals(-returncode).name
                                       if returncode < 0 else None),
              "last_progress": scanner.scan()}
    write_new(run_dir / "exit.json", json.dumps(record, indent=2, sort_keys=True) + "\n")
    shell_status = returncode if returncode >= 0 else 128 - returncode
    write_new(run_dir / "exit.txt", "%d\n" % shell_status)
    append_jsonl(run_dir / "heartbeat.jsonl", {"time": end, "child_alive": False,
                                               "progress": record["last_progress"],
                                               "session": session_table(sid)})
    post_code = None
    if args.post:
        post_start = now()
        post_code = subprocess.call(args.post, shell=True, cwd=str(cwd),
                                    stdout=open(run_dir / "post.log", "xb"),
                                    stderr=subprocess.STDOUT, preexec_fn=unblock)
        write_new(run_dir / "post.json", json.dumps({
            "command": args.post, "start": post_start, "end": now(),
            "exit_code": post_code}, indent=2, sort_keys=True) + "\n")
    sys.exit(post_code if post_code is not None else
             (returncode if returncode >= 0 else 128 - returncode))


if __name__ == "__main__":
    main()
