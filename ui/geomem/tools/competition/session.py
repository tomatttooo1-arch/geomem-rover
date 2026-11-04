#!/usr/bin/env python3
"""Laptop orchestration: direct Nano SSH, no laboratory server dependency.
Plan is the default; --execute performs the chosen operation. No passwords in config.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import shutil
import shlex
import subprocess
import sys
import time
import webbrowser

ROOT = Path(__file__).resolve().parents[4]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=("start", "check", "kv-up", "mapping", "demo", "bridge", "down", "collect", "basemap"))
    p.add_argument("--config", required=True)
    p.add_argument("--execute", action="store_true")
    p.add_argument("--no-browser", action="store_true")
    p.add_argument("--observe-seconds", type=int, default=0, help="Attach read-only for a bounded connectivity check")
    a = p.parse_args(argv)
    config_path = Path(a.config).resolve()
    c = json.loads(config_path.read_text(encoding="utf-8-sig"))
    work = config_path.parent / "session-data"
    for key in ("nano_ssh", "nano_ip", "kv_ssh_from_nano"):
        if not re.fullmatch(r"[a-zA-Z0-9_@.:-]+", str(c.get(key, ""))) or c[key].startswith("-"):
            p.error("invalid " + key)
    for key in ("nano_repo", "kv_repo", "kv_run"):
        if not re.fullmatch(r"/[a-zA-Z0-9_./-]+", str(c.get(key, ""))) or ".." in Path(c[key]).parts:
            p.error("absolute deployment path required: " + key)
    for key in ("hub_port", "bridge_port"):
        if type(c.get(key)) is not int or not 1024 <= c[key] <= 65535:
            p.error("invalid " + key)
    for key in ("map_yaml", "memory_file", "database"):
        if c.get(key) and not re.fullmatch(r"/[a-zA-Z0-9_./-]+", c[key]):
            p.error("unsupported remote path: " + key)
    map_sha = c.get("map_sha256", "")
    if not isinstance(map_sha, str) or (map_sha and not re.fullmatch(r"[0-9a-f]{64}", map_sha)):
        p.error("map_sha256 must be a full lowercase SHA256")
    if a.action == "demo" and not all(c.get(k) for k in ("map_yaml", "memory_file", "map_sha256")):
        p.error("demo requires map_yaml, memory_file, map_sha256 from run 1")
    q = shlex.quote
    # Ignore ambient jump hosts/reverse forwards: this path must be direct to Nano.
    options = ["-F", "NUL" if os.name == "nt" else "/dev/null", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
               "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3"]
    if c.get("ssh_identity"):
        options += ["-i", str(Path(os.path.expandvars(c["ssh_identity"])).expanduser()), "-o", "IdentitiesOnly=yes"]
    if c.get("ssh_known_hosts"):
        known = Path(os.path.expandvars(c["ssh_known_hosts"])).expanduser()
        if not known.is_absolute():
            known = config_path.parent / known
        options += ["-o", "UserKnownHostsFile=" + str(known), "-o", "HostKeyAlias=geomem-nano", "-o", "StrictHostKeyChecking=yes"]
    ssh = ["ssh"] + options + [c["nano_ssh"]]

    def run(args, capture=False, quiet=False):
        if not quiet:
            print(("RUN " if a.execute else "PLAN ") + subprocess.list2cmdline([str(x) for x in args]), flush=True)
        if not a.execute:
            return ""
        result = subprocess.run([str(x) for x in args], check=True, stdout=subprocess.PIPE if capture else None,
                                text=True, encoding="utf-8", errors="replace")
        return result.stdout or ""

    def remote(cmd, capture=False, quiet=False):
        return run(ssh + [cmd], capture, quiet)

    repo = q(c["nano_repo"])
    current = "/data/fpga/car/state/car.current"
    kv_config = c.get("kv_ssh_config", "/home/robot/.ssh/geomem-kv.conf")
    if not re.fullmatch(r"/[a-zA-Z0-9_./-]+", kv_config):
        p.error("invalid kv_ssh_config")
    if a.execute:
        work.mkdir(parents=True, exist_ok=True)
        (work / "basemaps").mkdir(exist_ok=True)

    def nano(mode):
        args = ["bash", c["nano_repo"] + "/ui/geomem/tools/competition/nano_up.sh", c["nano_repo"], mode,
                str(c["hub_port"]), kv_config, c.get("map_yaml", ""), c.get("memory_file", ""), map_sha]
        command = " ".join(q(x) for x in args)
        if mode == "demo":
            # Protect clients pointing at an older Nano deployment too. Missing
            # preflight code must fail before an old nano_up can reuse run 1.
            check = ["python3", c["nano_repo"] + "/ui/geomem/tools/competition/check_demo.py",
                     "--map-yaml", c["map_yaml"], "--memory", c["memory_file"], "--db-sha256", map_sha]
            command = ("set -e; if test -e " + current + " || test -L " + current +
                       "; then echo 'Second run refused: stop and collect the recorded first run before demo.' >&2; exit 7; fi; " +
                       " ".join(q(x) for x in check) + "; " + command)
        remote(command)

    def bridge():
        # Token goes straight to a local file through scp; never to stdout or a URL.
        hub_local = c.get("hub_local_port", 18791)
        mapping_local = c.get("mapping_local_port", 8792)
        operator_port = c.get("operator_port", 8788)
        ports = [hub_local, mapping_local, operator_port, c["bridge_port"]]
        if any(type(port) is not int or not 1024 <= port <= 65535 for port in ports) or len(set(ports)) != len(ports):
            p.error("local hub, mapping, operator and bridge ports must be distinct valid ports")
        tunnel = ["ssh"] + options + ["-N", "-o", "ExitOnForwardFailure=yes", "-L",
                  "127.0.0.1:%d:127.0.0.1:%d" % (hub_local, c["hub_port"]), "-L",
                  "127.0.0.1:%d:127.0.0.1:47802" % mapping_local, c["nano_ssh"]]
        cmd = [sys.executable, ROOT / "ui/geomem/bridge/geomem_bridge.py", "--hub", "127.0.0.1:" + str(hub_local),
               "--token-file", work / "token.txt", "--listen", "127.0.0.1:" + str(c["bridge_port"]), "--basemap-watch", work / "basemaps",
               "--stats-out", work / "bridge-stats.json"]
        if c.get("map_sha256"):
            if not re.fullmatch("[0-9a-f]{64}", c["map_sha256"]):
                p.error("map_sha256 must be a full lowercase SHA256")
            cmd.extend(["--map-id", c["map_sha256"]])
        if a.execute:
            # Do not silently reuse a lab bridge or another user's occupied ports.
            for port in ports:
                with socket.socket() as probe:
                    try:
                        probe.bind(("127.0.0.1", port))
                    except OSError:
                        raise RuntimeError("Local port %d is occupied; close the earlier laptop launcher before reconnecting" % port)
            operator = Path(c.get("operator_root", str(ROOT / "ui/geomem/operator"))) / "server.mjs"
            if not operator.is_file():
                p.error("original operator website is missing from this source package")
            url = "http://127.0.0.1:%d/#remote" % operator_port
            processes = []
            logs = []
            def start(name, args, env=None):
                log = (work / (name + '.log')).open('ab')
                logs.append(log)
                proc = subprocess.Popen([str(x) for x in args], stdout=log, stderr=subprocess.STDOUT, env=env)
                processes.append(proc)
                return proc
            def wait_port(port):
                for _ in range(80):
                    if any(proc.poll() is not None for proc in processes):
                        raise RuntimeError("A laptop service exited; inspect logs in " + str(work))
                    try:
                        with socket.create_connection(('127.0.0.1', port), timeout=.25):
                            return
                    except OSError:
                        time.sleep(.1)
                raise RuntimeError("Local service did not become ready: " + str(port))
            try:
                remote("python3 " + q(c["nano_repo"] + "/ui/geomem/tools/competition/check_current.py") +
                       " --current " + current + " --hub-port " + str(c["hub_port"]))
                run(["scp"] + options + [c["nano_ssh"] + ":" + current + "/token.txt", work / "token.txt"])
                if os.name != 'nt':
                    os.chmod(work / 'token.txt', 0o600)
                start('direct-tunnel', tunnel)
                wait_port(hub_local)
                start('direct-bridge', cmd)
                wait_port(c["bridge_port"])
                env = dict(os.environ, PORT=str(operator_port), GEOMEM_BRIDGE_PORT=str(c["bridge_port"]), GEOMEM_MAPPING_PORT=str(mapping_local))
                node = c.get('node') or shutil.which('node')
                if not node:
                    candidates = [ROOT.parent / 'runtime/node.exe', Path.home() / '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe']
                    node = next((str(path) for path in candidates if path.is_file()), 'node')
                start('direct-website', [node, str(operator)], env)
                wait_port(operator_port)
                print("READY direct laptop -> Nano -> KV260: " + url, flush=True)
                print("Keep this window open. Ctrl+C closes only laptop services; the car stack keeps running.", flush=True)
                if not a.no_browser:
                    webbrowser.open(url)
                started = time.monotonic()
                while not a.observe_seconds or time.monotonic() - started < a.observe_seconds:
                    if any(proc.poll() is not None for proc in processes):
                        raise RuntimeError("A direct session process exited; inspect " + str(work))
                    time.sleep(.5)
            finally:
                for proc in reversed(processes):
                    if proc.poll() is None:
                        proc.terminate()
                        try:
                            proc.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                            proc.wait(timeout=5)
                for log in logs:
                    log.close()
        else:
            run(["scp"] + options + [c["nano_ssh"] + ":" + current + "/token.txt", work / "token.txt"])
            run(tunnel)
            run(cmd)

    if a.action == "check":
        remote("set -e; hostname; test -f " + repo + "/COMMIT; test -e /dev/fpga_chassis; docker version --format '{{.Server.Version}}'; "
               "ssh -F " + q(kv_config) + " -o BatchMode=yes geomem-kv260 'set -e; hostname; test -x /usr/local/sbin/geomem-kv-start; sudo -n -l /usr/local/sbin/geomem-kv-start'")
        print("Laptop Docker is needed only for offline basemap generation, not startup or live views.")
    elif a.action == "kv-up":
        nano("kv-up")
    elif a.action in ("start", "mapping", "demo"):
        nano("demo" if a.action == "demo" else "mapping")
        bridge()
    elif a.action == "bridge":
        nano("attach")
        bridge()
    elif a.action == "down":
        remote("cd " + repo + " && bash tools/car_run/car_down.sh")
    elif a.action == "collect":
        if not c.get("database") or not c.get("memory_file"):
            p.error("collect requires database and memory_file")
        remote("test ! -e " + current)
        for key in ("database", "memory_file"):
            run(["scp"] + options + [c["nano_ssh"] + ":" + c[key], work / Path(c[key]).name])
    elif a.action == "basemap":
        if not c.get("database"):
            p.error("basemap requires database")
        db = work / Path(c["database"]).name
        cmd = ["docker", "run", "--rm", "--network", "none", "-v", str(ROOT) + ":/wt:ro", "-v", str(work) + ":/in:ro",
               "-v", str(work / "basemaps") + ":/out", "--entrypoint", "bash", c["docker_image"], "-c",
               "source /opt/ros/humble/setup.bash && python3 /wt/ui/geomem/tools/basemap/make_basemap.py --db " + q("/in/" + db.name) + " --out-dir /out --export native"]
        started = time.monotonic()
        run(cmd)
        if a.execute:
            h = hashlib.sha256()
            with db.open("rb") as f:
                for chunk in iter(lambda: f.read(1024 * 1024), b""):
                    h.update(chunk)
            result = {"db_sha256": h.hexdigest(), "elapsed_s": time.monotonic() - started, "database": str(db)}
            (work / "basemap-result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
            print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("Laptop session closed; the car stack was not stopped.")
        sys.exit(130)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print("Stopped: " + str(exc), file=sys.stderr)
        sys.exit(1)
