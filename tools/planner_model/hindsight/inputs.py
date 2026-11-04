"""Strict, streaming FPGAGPK01 reader. A matrix's field name is not pose provenance."""
import hashlib
import json
from pathlib import Path
import struct
import numpy as np

FH = struct.Struct('<8sIIII')
RH = struct.Struct('<6If9f12f3dd')


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def write_json(path, data):
    from tools.planner_model.validate_v2 import serial
    Path(path).write_text(json.dumps(data, default=serial, indent=2, allow_nan=False)+'\n')


class Pack:
    def __init__(self, path):
        self.path = Path(path)
        self.offsets = []
        with self.path.open('rb') as f:
            b = f.read(FH.size)
            if len(b) != FH.size:
                raise ValueError('truncated_pack_header')
            magic, ver, n, a, b = FH.unpack(b)
            if magic != b'FPGAGPK01' or ver != 1 or a or b:
                raise ValueError('invalid_pack_header')
            size = self.path.stat().st_size
            for _ in range(n):
                pos = f.tell()
                b = f.read(RH.size)
                if len(b) != RH.size:
                    raise ValueError('truncated_frame_header')
                h = RH.unpack(b)
                w, hh, nx, ny, nz, reserved = h[:6]
                if (not (1 <= w <= 1280 and 1 <= hh <= 800) or reserved or
                        not all(1 <= x <= 32 for x in (nx, ny, nz)) or
                        not np.isfinite(h[6:]).all() or h[6] <= 0 or h[-1] <= 0):
                    raise ValueError('invalid_frame_header')
                end = pos + RH.size + 2*w*hh
                if end > size:
                    raise ValueError('truncated_depth')
                self.offsets.append((pos, h))
                f.seek(end)
            if f.tell() != size:
                raise ValueError('trailing_pack_bytes')

    def __len__(self):
        return len(self.offsets)

    def frame(self, i):
        off, h = self.offsets[i]
        w, hh, nx, ny, nz = h[:5]
        raw = np.memmap(self.path, mode='r', offset=off+RH.size, shape=(hh, w), dtype='<u2')
        wc = np.eye(4)
        wc[:3] = np.asarray(h[16:28]).reshape(3, 4)
        return dict(raw=raw, unit=h[6], ik=np.asarray(h[7:16], np.float32).reshape(3, 3),
                    wc=wc, origin=np.array(h[28:31]), voxel=h[31], dims=(nx, ny, nz))

    def audit(self):
        poses = np.array([h[16:28] for _, h in self.offsets]).reshape(-1, 3, 4)
        changes = np.flatnonzero(np.any(poses[1:] != poses[:-1], axis=(1, 2)))+1
        return dict(path=str(self.path.resolve()), sha256=sha(self.path), frames=len(self),
                    sampled_frames=len(range(0, len(self), 5)), unique_matrices=len(np.unique(poses.reshape(-1, 12), axis=0)),
                    matrix_change_frames=changes.tolist(), translation_min=poses[:, :, 3].min(0).tolist(),
                    translation_max=poses[:, :, 3].max(0).tolist(), first_matrix=poses[0].tolist(),
                    unique_dimensions=sorted(set(h[:5] for _, h in self.offsets)))


def body_poses(camera, mount):
    camera = np.asarray(camera, float)
    mount = np.asarray(mount, float)
    for m in (camera, mount):
        if m.shape[-2:] != (4, 4) or not np.isfinite(m).all():
            raise ValueError('invalid_rigid_transform')
        if not np.allclose(m[..., 3, :], [0, 0, 0, 1], atol=1e-6, rtol=0):
            raise ValueError('invalid_homogeneous_row')
        rot = m[..., :3, :3]
        if not np.allclose(rot @ np.swapaxes(rot, -1, -2), np.eye(3), atol=1e-4, rtol=0):
            raise ValueError('nonrigid_transform')
        if not np.allclose(np.linalg.det(rot), 1., atol=1e-4, rtol=0):
            raise ValueError('improper_rotation')
    return camera @ np.linalg.inv(mount)
