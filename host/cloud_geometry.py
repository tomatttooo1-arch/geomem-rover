"""Independent same-pixel cloud/depth geometry check for the simulation sensor contract."""
import math
import struct


def check_cloud_depth(cloud, depth, info, pixels=((200, 90), (280, 40))):
    stamps = [(m.header.stamp.sec, m.header.stamp.nanosec) for m in (cloud, depth, info)]
    if len(set(stamps)) != 1:
        raise ValueError('cloud/depth/intrinsics must have the same acquisition stamp')
    if depth.encoding != '32FC1' or (cloud.width, cloud.height) != (depth.width, depth.height):
        raise ValueError('invalid aligned cloud/depth layout')
    if info.header.frame_id != 'camera_optical_frame' or depth.header.frame_id != 'camera_optical_frame':
        raise ValueError('depth and intrinsics must be optical')
    if cloud.header.frame_id not in ('camera_link', 'camera_optical_frame'):
        raise ValueError('unknown cloud axes')
    fields = {f.name: f for f in cloud.fields}
    fx, fy, cx, cy = (float(info.k[j]) for j in (0, 4, 2, 5))
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
        raise ValueError('invalid intrinsics')
    rows = []
    for u, v in pixels:
        if not (0 <= u < cloud.width and 0 <= v < cloud.height):
            raise ValueError('calibration pixel outside image')
        d = struct.unpack_from(('>' if depth.is_bigendian else '<')+'f', depth.data, v*depth.step+u*4)[0]
        xyz = []
        for axis in ('x', 'y', 'z'):
            field = fields[axis]
            if field.datatype != 7 or field.count != 1 or not 0 <= field.offset <= cloud.point_step-4:
                raise ValueError('expected scalar float32 XYZ')
            xyz.append(struct.unpack_from(('>' if cloud.is_bigendian else '<')+'f', cloud.data,
                       v*cloud.row_step+u*cloud.point_step+field.offset)[0])
        optical = [-xyz[1], -xyz[2], xyz[0]] if cloud.header.frame_id == 'camera_link' else xyz
        expected = [(u-cx)*d/fx, (v-cy)*d/fy, d]
        valid = d > 0 and all(math.isfinite(n) for n in optical+expected)
        residual = max(abs(a-b) for a,b in zip(optical, expected)) if valid else None
        rows.append(dict(pixel=[u,v], cloud_optical_xyz=optical if valid else None,
                         expected_from_depth=expected if valid else None,
                         max_axis_error_m=residual, passed=valid and residual < .04))
    return dict(passed=bool(rows) and all(r['passed'] for r in rows), samples=rows,
                stamp_ns=stamps[0][0]*1000000000+stamps[0][1], cloud_frame=cloud.header.frame_id,
                max_axis_error_limit_m=.04)
