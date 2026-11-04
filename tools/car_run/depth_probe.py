#!/usr/bin/env python3
"""Line E, live preset (2026-10-15): did the Orbbec driver's depth stream really start? -- count depth messages.

The board scripts judged a driver start by the line "depth Frame" in driver.log (the depth profile). Controller,
board 10-15 00:03 (integration l_chain32): that line was printed (try 1 counted as good) and then not one depth frame
came (drv_frames.csv only colour rows, colour dropped by the frame sync; no /camera/depth/camera_info in 60 s).
So car_inner.sh judges a start by data: this probe subscribes to the depth image topic (sensor_msgs/Image,
sensor-data QoS: best effort, keep last 5 -- matches a reliable or a best-effort publisher, as cam_probe.py) and exits
0 as soon as --count messages have arrived, 1 when --timeout-s passes first (2: rclpy / message import failed).
One JSON line on stdout: received, needed, seconds, the last message's size/encoding and its non-zero share (the
Gemini 2 gives all-zero depth when something is too close -- recorded, not judged). Python 3.8 compatible.

Half-rate check (10-17, --rate-s S > 0; car_inner.sh CAMERA_RATE_CHECK=1, docs/design/COLOUR_RATE_NAVMODE_20261017.md
section 7): the Gemini 2 enters, at random on a start and for the whole run, a "half-rate state" -- depth every
32.30 ms, colour every 64.61 ms (15.5 Hz) -- instead of the normal 33.40 ms / 33.40 ms. Once --count depth images
are in, the probe keeps both subscriptions (colour: --color-topic, subscribed from the start so no discovery wait)
for S more seconds, takes the HEADER stamps (the driver's time_domain global = the device clock mapped to the host;
not the arrival times, which carry DDS / scheduling jitter), and judges the intervals (classify below):
  half-rate    depth median < 32.8 ms, or more than 3/4 of the colour intervals > 50 ms (colour 25th percentile > 50)
  insufficient fewer than 20 depth or 10 colour intervals in the window
  normal       otherwise
Exit 3 = half-rate, 4 = insufficient: car_start_retry treats both as a failed start (stop, pause, start again).
The colour rule uses the 25th percentile, not the median: a subscriber that loses frames only makes intervals LONGER,
so a lossy but normal stream would need > 3/4 of its colour frames lost to be called half-rate (the depth rule is
immune: losses never shorten an interval). The window starts after --count depth images (about 1 s), past the
first-5-frames colour transient of a half-rate start (section 3.3). Only header stamps are kept in the colour
callback (never the 921 KB data). The JSON line gets "rate": the verdict and the numbers.

Camera rate (10-29, --rate-fps, car_up CAR_CAMERA_FPS; default 30 = everything above, byte for byte): the half-rate
signature above was only ever seen at 30 fps (a 32.30 ms trigger period that cannot hold a ~33 ms exposure). At
another rate the depth rule is off (its normal period is not known to the ms) and the colour rule scales with the
period: colour 25th percentile > 50 ms x 30 / fps (15 fps: 100 ms = more than 3/4 of the colour frames missing) =
half; the minimum interval counts scale the same way (15 fps: 10 depth / 5 colour). The probe also records
pair_offset_ms: the median of (nearest colour stamp - depth stamp) -- what the runner's and rtabmap's colour/depth
pairing tolerances must cover.

--summarize LOG...: one line per driver_probe_N.log (its last JSON line): "try N: <verdict> (...)" -- car_inner.sh
puts it in the refusal message when every start failed."""
import argparse
import json
import re
import sys
import time

DEPTH_HALF_MS = 32.8      # depth median below this: half-rate (half 32.30 ms, normal 33.40 ms)
COLOUR_HALF_MS = 50.0     # colour 25th percentile above this: half-rate (half 64.61 ms, normal 33.40 ms)
MIN_DEPTH_IV = 20         # in a 3 s window: about 90 (normal) / 93 (half) depth intervals
MIN_COLOUR_IV = 10        # in a 3 s window: about 90 (normal) / 46 (half) colour intervals
RC_HALF = 3
RC_INSUFFICIENT = 4


def nonzero_share(data):
    """share of non-zero bytes of an image buffer (no numpy needed; None for an empty buffer)."""
    b = bytes(data)
    return None if not b else round(1.0 - b.count(0) / float(len(b)), 4)


def quantile(xs, q):
    """the q-quantile of a non-empty list, linear between order statistics (numpy's default)."""
    s = sorted(xs)
    pos = (len(s) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (pos - lo)


def intervals_ms(stamps):
    """consecutive differences (ms) of stamps in seconds, in arrival order after sorting; repeated stamps dropped."""
    s = sorted(set(stamps))
    return [(b - a) * 1000.0 for a, b in zip(s, s[1:])]


def rate_limits(fps=30.0):
    """(depth_half_ms or None, colour_half_ms, min_depth_iv, min_colour_iv) at a camera rate (10-29): 30 fps = the
    10-17 constants; any other rate: no depth rule, the colour threshold and the minimum counts scaled by 30 / fps."""
    fps = float(fps)
    if not (fps > 0):
        raise ValueError("fps must be > 0")
    if fps == 30.0:
        return DEPTH_HALF_MS, COLOUR_HALF_MS, MIN_DEPTH_IV, MIN_COLOUR_IV
    k = 30.0 / fps
    return (None, round(COLOUR_HALF_MS * k, 1), max(5, int(round(MIN_DEPTH_IV / k))),
            max(3, int(round(MIN_COLOUR_IV / k))))


def pair_offset_ms(depth_stamps, colour_stamps):
    """median of (nearest colour stamp - depth stamp) in ms over the window's depth frames, or None."""
    c = sorted(set(colour_stamps))
    if not c or not depth_stamps:
        return None
    offs = []
    for d in sorted(set(depth_stamps)):
        best = min(c, key=lambda x: abs(x - d))
        offs.append((best - d) * 1000.0)
    return round(quantile(offs, 0.5), 3)


def classify(depth_iv_ms, colour_iv_ms, fps=30.0):
    """the verdict on one window's intervals (ms): dict with verdict normal | half | insufficient and the numbers.
    ``fps`` (10-29): the configured camera rate (rate_limits); 30 = the 10-17 rule exactly."""
    d = list(depth_iv_ms)
    c = list(colour_iv_ms)
    depth_half, colour_half, min_d, min_c = rate_limits(fps)
    if depth_half is None:
        rule = ("%g fps: half if colour p25 > %.1f ms (no depth rule off 30 fps); insufficient if < %d depth or < %d "
                "colour intervals" % (float(fps), colour_half, min_d, min_c))
    else:
        rule = ("half if depth median < %.1f ms or colour p25 > %.1f ms; insufficient if < %d depth or < %d colour "
                "intervals" % (depth_half, colour_half, min_d, min_c))
    out = {"depth_intervals": len(d), "colour_intervals": len(c),
           "depth_median_ms": round(quantile(d, 0.5), 3) if d else None,
           "colour_median_ms": round(quantile(c, 0.5), 3) if c else None,
           "colour_p25_ms": round(quantile(c, 0.25), 3) if c else None,
           "rule": rule}
    if float(fps) != 30.0:
        out["fps"] = float(fps)
    reasons = []
    if len(d) < min_d or len(c) < min_c:
        out["verdict"] = "insufficient"
        reasons.append("%d depth / %d colour intervals (need %d / %d)" % (len(d), len(c), min_d, min_c))
    else:
        if depth_half is not None and out["depth_median_ms"] < depth_half:
            reasons.append("depth median %.2f ms < %.1f" % (out["depth_median_ms"], depth_half))
        if out["colour_p25_ms"] > colour_half:
            reasons.append("colour p25 %.2f ms > %.1f" % (out["colour_p25_ms"], colour_half))
        out["verdict"] = "half" if reasons else "normal"
    out["reasons"] = reasons
    return out


def rate_rc(verdict):
    return {"normal": 0, "half": RC_HALF}.get(verdict, RC_INSUFFICIENT)


def describe(row):
    """one readable line for a probe JSON row (the --summarize text)."""
    if not isinstance(row, dict):
        return "no probe result"
    if "error" in row:
        return "probe error: %s" % row["error"]
    if row.get("received", 0) < row.get("needed", 1):
        return "no depth stream (%s of %s depth images in %s s)" % (row.get("received"), row.get("needed"),
                                                                   row.get("seconds"))
    r = row.get("rate")
    if not r:
        return "depth stream up (no rate check)"
    nums = "depth median %s ms, colour p25 %s ms / median %s ms, %s depth / %s colour intervals" % (
        r.get("depth_median_ms"), r.get("colour_p25_ms"), r.get("colour_median_ms"), r.get("depth_intervals"),
        r.get("colour_intervals"))
    name = {"half": "HALF-RATE", "insufficient": "too few frames for the rate check"}.get(r.get("verdict"), "normal")
    return "%s (%s)" % (name, nums)


def last_json(path):
    try:
        with open(path) as f:
            lines = f.read().splitlines()
    except OSError:
        return None
    for line in reversed(lines):
        line = line.strip()
        if line.startswith("{"):
            try:
                return json.loads(line)
            except ValueError:
                continue
    return None


def summarize(paths):
    def tryno(p):
        m = re.search(r"_probe_(\d+)\.log$", p)
        return int(m.group(1)) if m else 0
    return ["try %s: %s" % (tryno(p) or "?", describe(last_json(p))) for p in sorted(paths, key=tryno)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic")
    ap.add_argument("--count", type=int, default=3)
    ap.add_argument("--timeout-s", type=float, default=20.0)
    ap.add_argument("--rate-s", type=float, default=0.0, help="half-rate check window after --count (0 = off)")
    ap.add_argument("--color-topic", default="/camera/color/image_raw")
    ap.add_argument("--rate-fps", type=float, default=30.0,
                    help="10-29: the configured camera rate the half-rate rule is judged at (30 = the 10-17 rule)")
    ap.add_argument("--summarize", nargs="+", metavar="LOG", help="print one line per driver_probe_N.log and exit")
    a = ap.parse_args(argv)
    if a.summarize:
        print("; ".join(summarize(a.summarize)))
        return 0
    if not a.topic:
        ap.error("--topic is required")
    if not a.rate_fps > 0:
        ap.error("--rate-fps must be > 0")
    t0 = time.monotonic()
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        print(json.dumps({"error": "import: %s" % exc}))
        return 2
    state = {"n": 0, "last": None, "rec": False, "d": [], "c": []}

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def cb(msg):
        state["n"] += 1
        state["last"] = msg
        if state["rec"]:
            state["d"].append(stamp(msg))

    def ccb(msg):
        if state["rec"]:
            state["c"].append(stamp(msg))

    rclpy.init(args=None)
    node = rclpy.create_node("fpga_depth_probe")
    node.create_subscription(Image, a.topic, cb, qos_profile_sensor_data)
    if a.rate_s > 0:
        node.create_subscription(Image, a.color_topic, ccb, qos_profile_sensor_data)
    rate = None
    try:
        while state["n"] < a.count and time.monotonic() - t0 < a.timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        depth_ok = state["n"] >= a.count
        t_depth = time.monotonic() - t0
        if depth_ok and a.rate_s > 0:
            state["rec"] = True
            t1 = time.monotonic()
            while time.monotonic() - t1 < a.rate_s:
                rclpy.spin_once(node, timeout_sec=0.05)
            state["rec"] = False
            rate = classify(intervals_ms(state["d"]), intervals_ms(state["c"]), fps=a.rate_fps)
            rate.update({"window_s": a.rate_s, "color_topic": a.color_topic, "stamps": "header (driver time_domain)",
                         "pair_offset_ms": pair_offset_ms(state["d"], state["c"])})
    finally:
        m = state["last"]
        row = {"topic": a.topic, "received": state["n"], "needed": a.count, "seconds": round(time.monotonic() - t0, 3),
               "last": None if m is None else {"width": int(m.width), "height": int(m.height), "encoding": m.encoding,
                                                "nonzero_share": nonzero_share(m.data)}}
        if a.rate_s > 0:
            row["depth_seconds"] = round(t_depth, 3) if "t_depth" in locals() else None
            row["rate"] = rate
        print(json.dumps(row), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    if state["n"] < a.count:
        return 1
    return rate_rc(rate["verdict"]) if rate is not None else 0


if __name__ == "__main__":
    sys.exit(main())
