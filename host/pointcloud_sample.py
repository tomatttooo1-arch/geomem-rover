"""Bounded XYZ sampling from the original PointCloud2 byte buffer."""
import numpy as np


def sample_xyz(message, stride=16, limit=20000):
    if stride < 1 or limit < 1:
        raise ValueError('stride and limit must be positive')
    if message.width < 0 or message.height < 0 or message.point_step <= 0:
        raise ValueError('invalid cloud dimensions')
    if message.row_step < message.width * message.point_step:
        raise ValueError('row shorter than point layout')
    if len(message.data) < message.row_step * message.height:
        raise ValueError('truncated cloud')
    endian = '>' if message.is_bigendian else '<'
    fields = {field.name: field for field in message.fields}
    axes = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('XYZ must be scalar float32/float64')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > message.point_step:
            raise ValueError('XYZ exceeds point layout')
        values = np.ndarray((message.height, message.width), dtype=endian+'f'+str(size),
                            buffer=message.data, offset=field.offset,
                            strides=(message.row_step, message.point_step))
        axes.append(values.reshape(-1))
    # Same row-major order: discard invalid XYZ, then take every stride-th point.
    valid = np.isfinite(axes[0]) & np.isfinite(axes[1]) & np.isfinite(axes[2])
    indices = np.flatnonzero(valid)[::stride][:limit]
    return np.column_stack([axis[indices] for axis in axes]).tolist()
