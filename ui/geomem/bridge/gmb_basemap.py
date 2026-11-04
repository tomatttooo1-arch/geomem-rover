"""展台底图包（10-17）：桥启动时读入、核对，按已加载包的 sha256 托管两个文件，并给快照 ``map.basemap``。

包由 ui/geomem/tools/basemap/make_basemap.py 生成（服务器上跑）；这里只读成品，Python 3.8+ 纯标准库。
设计：ui/geomem/docs/PROTOCOL_V2_DESIGN.md §2.10。

托管只开放 ``/basemap/<64 位小写十六进制>/basemap.json`` 与 ``…/cells.bin``，且 sha256 必须是已加载的包；
文件内容加载时读进内存并核过 sha256，之后不再碰磁盘（不开放任意路径）。

10-18（owner）：① 里程档也画底图（车从建图起点同朝向出发、map->odom 恒等，与里程档导航同一假设），
``basis: "mapping_start"`` 标明「按建图起点对齐，未做定位校正」；② 运行中加载：``PackageDir`` 监视操作员给的一个
目录，出现新的 ``basemap-<库 sha 前 12 位>/`` 且核对通过就加进来（不重启桥）；只认这一层、这种名字的子目录，
不跟符号链接，不开放任意路径。
"""

import hashlib
import json
import os
import re

FORMAT = "geomem-basemap"
VERSION = 1
CELL_BYTES = 9                      # 3 x int16 index + 3 x uint8 RGB
MAX_JSON_BYTES = 4 << 20
MAX_CELLS_BYTES = 64 << 20
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
PATH_RE = re.compile(r"^/basemap/([0-9a-f]{64})/(basemap\.json|cells\.bin)$")
STATUSES = ("ok", "not_localized", "no_map_id", "mismatch")
BASES = ("localized", "mapping_start")      # ok 的两种依据：定位档 / 里程档按建图起点对齐（10-18）
DIR_RE = re.compile(r"^basemap-([0-9a-f]{12})$")
MAX_PACKAGES = 8                    # 运行中加载的上限（含启动时 --basemap 的包）
MAX_TOTAL_BYTES = 512 << 20
CONTENT_TYPES = {"basemap.json": "application/json", "cells.bin": "application/octet-stream"}


class BasemapError(Exception):
    pass


def _read(path, limit):
    size = os.path.getsize(path)
    if size > limit:
        raise BasemapError("%s 超过 %d 字节上限" % (path, limit))
    with open(path, "rb") as f:
        return f.read()


def load_package(path):
    """Read and verify one package directory. Returns a dict; raises BasemapError with a Chinese reason."""
    jpath = os.path.join(path, "basemap.json")
    try:
        raw = _read(jpath, MAX_JSON_BYTES)
        man = json.loads(raw.decode("utf-8"))
    except (OSError, ValueError) as exc:
        raise BasemapError("读不了 %s：%s" % (jpath, exc))
    if not isinstance(man, dict) or man.get("format") != FORMAT or man.get("version") != VERSION:
        raise BasemapError("%s 不是 %s 第 %d 版" % (jpath, FORMAT, VERSION))
    if man.get("frame") != "map":
        raise BasemapError("%s 的坐标系不是 map" % jpath)
    src = man.get("source_db") if isinstance(man.get("source_db"), dict) else {}
    sha = src.get("sha256")
    if not isinstance(sha, str) or not SHA_RE.match(sha):
        raise BasemapError("%s 缺源库 sha256（64 位小写十六进制）" % jpath)
    cell = man.get("cell_m")
    if isinstance(cell, bool) or not isinstance(cell, (int, float)) or not 0 < cell <= 1:
        raise BasemapError("%s 的 cell_m 无效" % jpath)
    files = man.get("files") if isinstance(man.get("files"), dict) else {}
    fc = files.get("cells") if isinstance(files.get("cells"), dict) else {}
    if fc.get("name") != "cells.bin":
        raise BasemapError("%s 没登记 cells.bin" % jpath)
    cpath = os.path.join(path, "cells.bin")
    try:
        cells = _read(cpath, MAX_CELLS_BYTES)
    except OSError as exc:
        raise BasemapError("读不了 %s：%s" % (cpath, exc))
    if len(cells) != fc.get("bytes") or len(cells) % CELL_BYTES:
        raise BasemapError("%s 长度 %d 与登记 %r 不符或不是 %d 的倍数" % (cpath, len(cells), fc.get("bytes"),
                                                                     CELL_BYTES))
    csha = hashlib.sha256(cells).hexdigest()
    if csha != fc.get("sha256"):
        raise BasemapError("%s 的 sha256 与 basemap.json 登记不符（文件被改过或不完整）" % cpath)
    count = len(cells) // CELL_BYTES
    if fc.get("count") not in (None, count):
        raise BasemapError("%s 格数 %d 与登记 %r 不符" % (cpath, count, fc.get("count")))
    name = src.get("name") if isinstance(src.get("name"), str) else None
    return {"sha256": sha, "name": name, "dir": os.path.abspath(path), "json": raw, "cells": cells,
            "cells_sha256": csha, "count": count, "cell_m": float(cell)}


def index_packages(pkgs):
    """{source db sha256: package}; two packages of the same library are refused."""
    out = {}
    for p in pkgs:
        if p["sha256"] in out:
            raise BasemapError("两个底图包对应同一个地图库 %s：%s、%s" % (p["sha256"][:12], out[p["sha256"]]["dir"],
                                                                     p["dir"]))
        out[p["sha256"]] = p
    return out


def url_for(sha, name="basemap.json"):
    return "/basemap/%s/%s" % (sha, name)


def basemap_status(tier, map_id, id_source, pkgs, configured=False):
    """``map.basemap`` of one snapshot, or None when no package is loaded and no watch directory is configured.

    ok: (a) localisation tier and ``map.id`` equals a loaded package's source-library sha256 -> ``basis``
        "localized"; (b) odometry tier (10-18, owner) and the map id (STATE pose.map_id, else --map-id) equals a
        loaded package -> ``basis`` "mapping_start": the car started at the mapping start with the same heading
        (map->odom identity, the odometry-tier navigation assumption), drawn without any localisation correction;
    not_localized: sim / undecidable tier (``tier`` says which);
    no_map_id: localisation or odometry tier but no map id (neither STATE pose.map_id nor --map-id);
    mismatch: map id names no loaded package (also while a watched directory has none yet: ``loaded`` empty).
    ``idSource``: "car" (STATE pose.map_id) | "operator" (bridge --map-id) | None."""
    if not pkgs and not configured:
        return None
    loaded = sorted(pkgs)
    out = {"status": None, "tier": tier, "basis": None, "idSource": id_source if map_id else None, "sha256": None,
           "url": None, "cellsUrl": None, "count": None, "name": None, "loaded": loaded}
    if tier not in ("loc", "odom"):
        out["status"] = "not_localized"
    elif not map_id:
        out["status"] = "no_map_id"
    elif map_id not in pkgs:
        out["status"] = "mismatch"
    else:
        p = pkgs[map_id]
        out.update({"status": "ok", "basis": "localized" if tier == "loc" else "mapping_start", "sha256": map_id,
                    "url": url_for(map_id), "cellsUrl": url_for(map_id, "cells.bin"), "count": p["count"],
                    "name": p["name"]})
    return out


def resolve(path, pkgs):
    """(body bytes, content type) for a request path, or None (404). Only loaded packages."""
    m = PATH_RE.match(path or "")
    if not m:
        return None
    p = pkgs.get(m.group(1))
    if p is None:
        return None
    body = p["json"] if m.group(2) == "basemap.json" else p["cells"]
    return body, CONTENT_TYPES[m.group(2)]


def _sig(path):
    out = []
    for name in ("basemap.json", "cells.bin"):
        try:
            st = os.stat(os.path.join(path, name))
            out.append((getattr(st, "st_mtime_ns", int(st.st_mtime * 1e9)), st.st_size))
        except OSError:
            out.append(None)
    return tuple(out)


class PackageDir(object):
    """Runtime loading (10-18): one operator-given directory, polled by the bridge.

    ``scan(loaded)`` returns the new packages to add: only immediate sub-directories named
    ``basemap-<12 lower-case hex>`` (make_basemap.py writes ``….tmp-<pid>`` first and renames, so a half-written
    package never has this name), not symbolic links, whose manifest's library sha256 starts with the name's 12
    digits, that ``load_package`` accepts (cells.bin length and sha256 against the manifest), whose library is not
    loaded yet, and within MAX_PACKAGES / MAX_TOTAL_BYTES. A directory whose load failed (e.g. still being copied) is
    retried once the (mtime, size) of its two files change; each outcome is logged once per change."""

    def __init__(self, root, log=None):
        self.root = os.path.abspath(root)
        self.log = log or (lambda msg: None)
        self.seen = {}                   # dir name -> (signature, outcome)
        self.root_error = None

    def _note(self, name, sig, outcome, msg=None):
        if msg and self.seen.get(name) != (sig, outcome):
            self.log(msg)
        self.seen[name] = (sig, outcome)

    def scan(self, loaded):
        try:
            names = sorted(os.listdir(self.root))
        except OSError as exc:
            if self.root_error != str(exc):
                self.root_error = str(exc)
                self.log("底图监视目录读不了：%s" % exc)
            return []
        self.root_error = None
        new = []
        total = sum(len(p["cells"]) + len(p["json"]) for p in loaded.values())
        count = len(loaded)
        for name in names:
            m = DIR_RE.match(name)
            if not m:
                continue
            path = os.path.join(self.root, name)
            if os.path.islink(path) or not os.path.isdir(path):
                self._note(name, None, "not_dir", "底图监视：%s 是符号链接或不是目录，忽略" % path)
                continue
            sig = _sig(path)
            if name in self.seen and self.seen[name][0] == sig:
                continue
            if None in sig:
                self._note(name, sig, "incomplete")       # still being copied: quiet, retried on change
                continue
            try:
                p = load_package(path)
            except BasemapError as exc:
                self._note(name, sig, "error", "底图监视：%s 暂不可用（文件变了会重试）：%s" % (path, exc))
                continue
            if not p["sha256"].startswith(m.group(1)):
                self._note(name, sig, "name_mismatch", "底图监视：%s 的库 sha256 %s… 与目录名不符，忽略"
                           % (path, p["sha256"][:12]))
            elif p["sha256"] in loaded or any(q["sha256"] == p["sha256"] for q in new):
                self._note(name, sig, "duplicate", "底图监视：库 %s… 的包已加载，忽略 %s（换包须重启桥）"
                           % (p["sha256"][:12], path))
            elif count + 1 > MAX_PACKAGES or total + len(p["cells"]) + len(p["json"]) > MAX_TOTAL_BYTES:
                self._note(name, sig, "limit", "底图监视：已达上限（%d 个包 / %d MiB），不加载 %s"
                           % (MAX_PACKAGES, MAX_TOTAL_BYTES >> 20, path))
            else:
                new.append(p)
                count += 1
                total += len(p["cells"]) + len(p["json"])
                self._note(name, sig, "loaded", "底图监视：加载 %s… %d 格，来自 %s" % (p["sha256"][:12], p["count"], path))
        return new
