#!/usr/bin/env python3
"""Search preview (10-26, probabilistic search): where the car will look, in which order, for staging a scene.

    python3 tools/car_run/search_preview.py --mapping-out RUN/rtabmap_out --target 1.2 0.4 --png preview.png
    python3 tools/car_run/search_preview.py --map export/map.yaml --nodes RUN/mapping_status.json --png p.png

Inputs: the map (map_server YAML + PGM: rtabmap_car.sh's OUT/export/*.yaml), the dense trajectory nodes (the
mapping status file's "search_nodes" -- mapping_ctl --status-file --, a JSON [[x, y, yaw]] list, or the export's
poses file with --poses; --mapping-out finds the yaml and the poses file in OUT/export), the remembered position
(--target X Y; omit for an object not in memory) and where the car starts (--robot X Y YAW; default the node
nearest the target). Same planner as the car (host/search_belief.py, --config = the car's search_config.json).

Output: the look order (position, heading, gain, cumulative probability of having seen the object if it is where
the prior says and is never detected earlier) on stdout / --json, and --png: the map, the prior as a heat map, the
trajectory nodes (blue), the looks numbered with their heading. No ROS. Python 3.8+, numpy; the PNG needs Pillow
(or OpenCV).
"""
import argparse
import glob
import json
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.search_belief import BeliefSearch, SearchConfig  # noqa: E402


def read_pgm(path):
    """uint8 image (row 0 = top) of a binary (P5) or ASCII (P2) PGM, 8 or 16 bit (16 bit scaled to 8)."""
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":
        pos += 1
        dt = np.dtype(">u2") if maxval > 255 else np.uint8
        img = np.frombuffer(data, dt, w * h, pos).reshape(h, w)
    elif magic == b"P2":
        img = np.array(data[pos:].split()[:w * h], int).reshape(h, w)
    else:
        raise ValueError("%s: not a PGM (P5/P2)" % path)
    return (img.astype(float) * 255. / maxval).round().astype(np.uint8)


def read_yaml_simple(path):
    """The map_server YAML keys (flat key: value, origin as a [x, y, yaw] list); no PyYAML needed."""
    doc = {}
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            v = v.strip()
            if v.startswith("["):
                doc[k.strip()] = [float(x) for x in v.strip("[]").split(",")]
            else:
                try:
                    doc[k.strip()] = float(v)
                except ValueError:
                    doc[k.strip()] = v.strip("'\"")
    return doc


def load_map(yaml_path):
    """FreeMap-like (cells row 0 = origin, 0 free / 100 occupied / -1 unknown) of a map_server YAML + image."""
    doc = read_yaml_simple(yaml_path)
    image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), doc["image"])
    if image.lower().endswith((".pgm", ".pnm")):
        img = read_pgm(image)
    else:
        from PIL import Image
        img = np.asarray(Image.open(image).convert("L"))
    p = img / 255. if int(doc.get("negate", 0)) else (255. - img) / 255.
    cells = np.full(img.shape, -1, np.int8)
    cells[p > float(doc.get("occupied_thresh", 0.65))] = 100
    cells[p < float(doc.get("free_thresh", 0.196))] = 0
    origin = doc.get("origin", [0., 0., 0.])
    return SimpleNamespace(cells=cells[::-1].copy(), resolution=float(doc["resolution"]),
                           origin_x=float(origin[0]), origin_y=float(origin[1]),
                           origin_yaw=float(origin[2]) if len(origin) > 2 else 0.)


def load_nodes(path):
    """[(x, y, yaw)] of a mapping status JSON ("search_nodes"), {"nodes": [...]}, or a bare [[x, y, yaw]] list."""
    with open(path) as f:
        doc = json.load(f)
    if isinstance(doc, dict):
        doc = (doc.get("search_nodes") or {}).get("nodes") if "search_nodes" in doc else doc.get("nodes")
    if not doc:
        raise ValueError("%s: no nodes (mapping status without search_nodes?)" % path)
    return [(float(r[0]), float(r[1]), float(r[2]) if len(r) > 2 else 0.) for r in doc]


def poses_to_nodes(poses_path):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import mapping_ctl
    return [tuple(n) for n in mapping_ctl.dense_nodes(mapping_ctl.poses_nodes(poses_path))]


def heat(v):
    """0..1 -> RGB (pale yellow -> orange -> dark red)."""
    v = np.clip(v, 0, 1)[..., None]
    lo, mid, hi = np.array([255, 245, 160.]), np.array([250, 140, 30.]), np.array([170, 0, 20.])
    return np.where(v < .5, lo + (mid - lo) * (v / .5), mid + (hi - mid) * ((v - .5) / .5))


def render(planner, free_map, prior_grid, order, target, path, px_per_m=100):
    """PNG of the crop the planner works on: map, prior heat, nodes, numbered looks."""
    f, res = planner.f, planner.res
    u0, v0, ni, nj = planner.u0, planner.v0, planner.ni, planner.nj
    cells = np.asarray(free_map.cells)[v0:v0 + ni * f, u0:u0 + nj * f]
    rgb = np.full(cells.shape + (3,), 150., float)                      # unknown grey
    rgb[(cells >= 0) & (cells < 50)] = 245.
    rgb[cells >= 50] = 25.
    hg = np.kron(np.nan_to_num(prior_grid, nan=-1), np.ones((f, f)))
    m = hg >= 0
    alpha = np.where(m, .25 + .6 * np.clip(hg, 0, 1), 0)[..., None]
    rgb = np.where(m[..., None], rgb * (1 - alpha) + heat(hg) * alpha, rgb)
    scale = max(1, int(round(px_per_m * res)))
    img = np.kron(rgb[::-1], np.ones((scale, scale, 1))).astype(np.uint8)   # row 0 = top
    H = img.shape[0]

    def pix(x, y):
        u, v = planner._uv(x, y)
        return float((u - u0) * scale), float(H - (v - v0) * scale)

    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        Image = None
    if Image is None:
        import cv2
        for x, y in planner.nodes:
            cv2.circle(img, tuple(int(c) for c in pix(x, y)), 3, (40, 90, 220), -1)
        for r in order:
            if "x" not in r:
                continue
            p = pix(r["x"], r["y"])
            q = pix(r["x"] + .35 * math.cos(r["yaw"]), r["y"] + .35 * math.sin(r["yaw"]))
            t = pix(r["x"] + .45 * math.cos(r["yaw"]), r["y"] + .45 * math.sin(r["yaw"]))
            cv2.arrowedLine(img, tuple(int(c) for c in p), tuple(int(c) for c in q), (0, 120, 0), 2)
            cv2.putText(img, str(r["n"]), (int(t[0]) - 5, int(t[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)
        cv2.imwrite(path, img[..., ::-1])
        return
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.load_default(size=13)
    except TypeError:
        font = ImageFont.load_default()
    for x, y in planner.nodes:
        a, b = pix(x, y)
        d.ellipse([a - 3, b - 3, a + 3, b + 3], fill=(40, 90, 220))
    if target is not None:
        a, b = pix(*target)
        d.line([a - 7, b - 7, a + 7, b + 7], fill=(200, 0, 200), width=3)
        d.line([a - 7, b + 7, a + 7, b - 7], fill=(200, 0, 200), width=3)
    for r in order:
        if "x" not in r:
            continue
        # the look's number at its arrow tip (several looks at one node point different ways)
        a, b = pix(r["x"], r["y"])
        c, e = pix(r["x"] + .3 * math.cos(r["yaw"]), r["y"] + .3 * math.sin(r["yaw"]))
        g, k = pix(r["x"] + .45 * math.cos(r["yaw"]), r["y"] + .45 * math.sin(r["yaw"]))
        d.ellipse([a - 4, b - 4, a + 4, b + 4], fill=(0, 130, 0))
        d.line([a, b, c, e], fill=(0, 130, 0), width=3)
        ang = math.atan2(e - b, c - a)
        for s in (2.6, -2.6):
            d.line([c, e, c + 8 * math.cos(ang + s), e + 8 * math.sin(ang + s)], fill=(0, 130, 0), width=3)
        d.ellipse([g - 11, k - 11, g + 11, k + 11], fill=(255, 255, 255), outline=(0, 100, 0), width=2)
        d.text((g, k), str(r["n"]), fill=(0, 0, 0), font=font, anchor="mm")
    im.save(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", help="map_server YAML (rtabmap export)")
    ap.add_argument("--nodes", help="mapping status JSON (search_nodes) or a [[x, y, yaw]] JSON list")
    ap.add_argument("--poses", help="rtabmap export poses file (format 11): nodes thinned as mapping_ctl does")
    ap.add_argument("--mapping-out", help="mapping OUT directory: export/*.yaml and the poses file found there")
    ap.add_argument("--target", nargs=2, type=float, metavar=("X", "Y"), help="remembered position (omit: unknown)")
    ap.add_argument("--robot", nargs=3, type=float, metavar=("X", "Y", "YAW"), help="car start (default: nearest "
                    "node to the target, or the first node)")
    ap.add_argument("--config", help="search config JSON (tools/car_run/search_config.json)")
    ap.add_argument("--max-goals", type=int, default=12, help="looks to plan (the car's max_search_visits: 12)")
    ap.add_argument("--png", help="write the heat map here")
    ap.add_argument("--json", help="write the order (and the summary) here")
    ap.add_argument("--px-per-m", type=float, default=100.)
    a = ap.parse_args(argv)
    if a.mapping_out:
        yamls = sorted(glob.glob(os.path.join(a.mapping_out, "export", "*.yaml")))
        if len(yamls) != 1:
            ap.error("--mapping-out: %d yaml file(s) in export/ (exactly one expected)" % len(yamls))
        a.map = a.map or yamls[0]
        if not a.nodes and not a.poses:
            from host import map_align
            poses, found = map_align.find_poses(a.mapping_out)
            if poses is None:
                ap.error("--mapping-out: %d poses file(s) in export/" % len(found))
            a.poses = poses
    if not a.map or not (a.nodes or a.poses):
        ap.error("give --map and --nodes / --poses (or --mapping-out)")
    cfg = SearchConfig.from_file(a.config) if a.config else SearchConfig()
    free_map = load_map(a.map)
    nodes = load_nodes(a.nodes) if a.nodes else poses_to_nodes(a.poses)
    planner = BeliefSearch(free_map, nodes, cfg)
    target = tuple(a.target) if a.target else None
    planner.begin(target)
    prior_grid = planner.belief_grid()
    if a.robot:
        robot = tuple(a.robot)
    else:
        ref = target or planner.nodes[0]
        n = min(planner.nodes, key=lambda p: (p[0] - ref[0]) ** 2 + (p[1] - ref[1]) ** 2)
        robot = (n[0], n[1], 0.)
    order = planner.preview(robot=robot, max_goals=a.max_goals)
    print("map %s  nodes %d  belief cells %d  build %.0f ms  target %s  start %s"
          % (a.map, len(planner.nodes), planner.ux.size, planner.build_ms, target, robot))
    print("%3s %8s %8s %7s %7s %7s %9s" % ("#", "x", "y", "yaw_deg", "gain", "path_m", "cum_prob"))
    for r in order:
        if "x" in r:
            print("%3d %8.2f %8.2f %7.0f %7.3f %7.2f %9.3f" % (r["n"], r["x"], r["y"], math.degrees(r["yaw"]),
                                                               r["gain"], r["path_m"], r["cum_prob"]))
        else:
            print("stop: %s (mass left %.3f)" % (r["stop"], r["mass_left"]))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"order": order, "summary": planner.summary(), "config": cfg.record(),
                       "target": target, "robot": robot}, f, indent=1)
    if a.png:
        render(planner, free_map, prior_grid, order, target, a.png, a.px_per_m)
        print("png: %s" % a.png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
