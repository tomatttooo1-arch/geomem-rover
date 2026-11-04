#!/usr/bin/env python3
"""Camera mount check for the rolling map (2026-09-27).

Given the camera height and down-tilt, where it sits relative to the car
front, and the car's size, print from how far ahead the floor and the D-035
obstacle band (floor +5 cm .. car height +5 cm) become visible, how wide the
blind ring in front of the car outline is, and how far ahead the view reaches.
See docs/team/CAMERA_MOUNT_FOR_dev2_20260927.md and
docs/design/ROLLING_NAV2_LAYER_20260927.md section 12.

Model: flat floor, side view plus a top view for the car width; the camera
is on the car's centre line. Defaults are the Gemini 2 in the mode we run
(hardware depth-to-colour alignment at 640x360, so the colour camera's field
of view applies: about 86 x 55 degrees, datasheet v1.0; our measured
intrinsics fx = 344.8 px give 85.7 x 55.1 degrees). The minimum range is
0.15 m (datasheet). Car sizes in the examples are placeholders: dev2 must
measure the real car.

A column counts as FREE in the rolling map only if every voxel of the band
was crossed by a depth ray that ends on a surface inside the per-frame grid
(about 1.5 m ahead of the camera). With a floor only, that needs the band
below the camera and the floor behind the column in view.
"""
import argparse
import json
import math

GEMINI2_D2C_VFOV_DEG = 55.1   # 2*atan(180/344.74), measured intrinsics, 640x360 D2C
GEMINI2_D2C_HFOV_DEG = 85.7   # 2*atan(320/344.78)
GEMINI2_MIN_RANGE_M = 0.15    # datasheet v1.0, "0.15m - 10m"
BAND_LOW_M = 0.05             # D-035: floor + 5 cm
BAND_MARGIN_M = 0.05          # D-035: car height + 5 cm
GRID_REACH_M = 1.5            # per-frame grid depth ahead of the camera (32 x 5 cm, one cell behind)


def check(*, cam_height_m, tilt_deg, cam_behind_front_m, car_height_m, car_width_m,
          vfov_deg=GEMINI2_D2C_VFOV_DEG, hfov_deg=GEMINI2_D2C_HFOV_DEG,
          min_range_m=GEMINI2_MIN_RANGE_M, grid_reach_m=GRID_REACH_M):
    """All distances are horizontal, measured forward from the car front."""
    h, c = float(cam_height_m), float(cam_behind_front_m)
    a = math.radians(vfov_deg) / 2
    t = math.radians(tilt_deg)
    low_edge, up_edge = t + a, t - a          # angles below the horizon
    band_top = car_height_m + BAND_MARGIN_M
    out = {"inputs": dict(cam_height_m=h, tilt_deg=tilt_deg, cam_behind_front_m=c,
                          car_height_m=car_height_m, car_width_m=car_width_m,
                          vfov_deg=vfov_deg, hfov_deg=hfov_deg)}
    warnings = []
    if band_top >= h:
        warnings.append("band top (car height + 5 cm) is at or above the camera: the upper band "
                        "can only be judged free when a wall stands behind it; raise the camera")
    if low_edge >= math.pi / 2:
        warnings.append("lower field-of-view edge points behind the camera")
    lowest = min(low_edge, math.radians(89.9))

    def near_for(z):
        """Nearest horizontal distance (from the camera) at which height z is in view."""
        x = (h - z) / math.tan(lowest) if z < h else 0.0
        # minimum range along the optical axis: Z = x cos t + (h - z) sin t >= min_range
        z_axis = lambda xx: xx * math.cos(t) + (h - z) * math.sin(t)  # noqa: E731
        if z_axis(x) < min_range_m:
            x = (min_range_m - (h - z) * math.sin(t)) / math.cos(t)
        return x

    ground_near = near_for(0.0)
    band_near = near_for(BAND_LOW_M)
    # the whole car width is inside the horizontal field of view from here on
    width_near = (car_width_m / 2) / math.tan(math.radians(hfov_deg) / 2)
    free_near = max(band_near, width_near)
    # far limits: the upper edge dips below the horizon when tilt > vfov/2
    if up_edge > 0:
        ground_far = h / math.tan(up_edge)
        band_far = (h - band_top) / math.tan(up_edge)
    else:
        ground_far = band_far = float("inf")
    if band_top >= h:
        band_far = 0.0  # with only a floor in view, the band top is never judged free
    # a band voxel at the floor + 5 cm is judged free only if the floor hit behind it
    # is inside the per-frame grid: x * h / (h - 0.05) <= grid reach
    free_far = min(band_far, grid_reach_m * (h - BAND_LOW_M) / h if h > BAND_LOW_M else 0.0,
                   ground_far * (h - BAND_LOW_M) / h if math.isfinite(ground_far) else
                   float("inf"))
    front = lambda x: x - c  # noqa: E731  camera distance -> distance ahead of the car front
    out.update(
        ground_visible_from_m=round(front(ground_near), 3),
        obstacle_5cm_visible_from_m=round(front(band_near), 3),
        full_width_visible_from_m=round(front(width_near), 3),
        blind_ring_depth_m=round(max(0.0, front(free_near)), 3),
        free_judged_up_to_m=(round(front(free_far), 3) if math.isfinite(free_far) else None),
        ground_visible_up_to_m=(round(front(ground_far), 3) if math.isfinite(ground_far)
                                else None),
        warnings=warnings)
    if math.isfinite(free_far) and free_far <= free_near:
        out["warnings"].append("no distance where a column can be judged free: tilt too steep "
                               "or camera too low")
    return out


def table(*, heights, tilts, **kw):
    rows = []
    for h in heights:
        for t in tilts:
            r = check(cam_height_m=h, tilt_deg=t, **kw)
            rows.append((h, t, r))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cam-height", type=float, help="optical centre height above floor, m")
    ap.add_argument("--tilt", type=float, help="down-tilt, degrees")
    ap.add_argument("--cam-behind-front", type=float, default=0.0,
                    help="how far the lens sits behind the car front, m (0 = flush)")
    ap.add_argument("--car-height", type=float, default=0.25, help="EXAMPLE, dev2 to measure")
    ap.add_argument("--car-width", type=float, default=0.30, help="EXAMPLE, dev2 to measure")
    ap.add_argument("--vfov", type=float, default=GEMINI2_D2C_VFOV_DEG)
    ap.add_argument("--hfov", type=float, default=GEMINI2_D2C_HFOV_DEG)
    ap.add_argument("--table", action="store_true", help="print the height x tilt table")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    kw = dict(cam_behind_front_m=args.cam_behind_front, car_height_m=args.car_height,
              car_width_m=args.car_width, vfov_deg=args.vfov, hfov_deg=args.hfov)
    if args.table or args.cam_height is None:
        heights = [0.25, 0.30, 0.35, 0.40, 0.45]
        tilts = [0, 10, 20, 25, 27.5, 30, 35]
        print("car height %.2f m, width %.2f m (EXAMPLE, dev2 to measure); lens %.2f m behind front"
              % (args.car_height, args.car_width, args.cam_behind_front))
        print("h[m] tilt | blind ring | 5cm obstacle from | free judged up to | floor up to")
        for h, t, r in table(heights=heights, tilts=tilts, **kw):
            fmt = lambda v: "   inf" if v is None else "%6.2f" % v  # noqa: E731
            flag = " !" if r["warnings"] else ""
            print("%.2f %5.1f | %6.2f     | %6.2f            | %s            | %s%s"
                  % (h, t, r["blind_ring_depth_m"], r["obstacle_5cm_visible_from_m"],
                     fmt(r["free_judged_up_to_m"]), fmt(r["ground_visible_up_to_m"]), flag))
        print("! = see warnings (use --cam-height/--tilt --json for details)")
        return
    r = check(cam_height_m=args.cam_height, tilt_deg=args.tilt, **kw)
    print(json.dumps(r, indent=1, ensure_ascii=False) if args.json else
          "\n".join("%s: %s" % kv for kv in r.items()))


if __name__ == "__main__":
    main()
