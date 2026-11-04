"""CPU-only C geometry -> C rolling fusion -> pinned C B1, with byte-exact source calibration."""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess
import zipfile
import numpy as np
from host.geo_producer import DEFAULT_HEIGHT_CELL
from host.kv_rolling import CpuRollingCore
from host.m7_frame_native import FrameNativeM7, compile_frame_native, MAX_PIXELS, GRID_CAPACITY
from host.m7_grid_policy import FollowCameraGrid
from host.m7_rolling import compile_rolling
from host import rolling_nav_layer as rnl
from tools.b1_costmap import bench as b, nav2_ref
from tools.car_geo_producer.common import base_camera, odom_base
from tools.m7_rolling_pl.replay_trace import read_trace
from tools.planner_model.hindsight.inputs import Pack, sha, write_json, body_poses


class Chain:
    def __init__(self, out):
        out = Path(out)
        out.mkdir(parents=True, exist_ok=False)
        self.geo = FrameNativeM7(compile_frame_native(out/'geo.so')).lib.fpga_m7_depth_frame
        self.roll = CpuRollingCore(compile_rolling(out/'roll.so'))
        self.cost = b.PlCore(b.build_costmap(str(out/'cost.so')))
        self.lut = nav2_ref.InflationParams().pl_lut(b.LUT_DIM)
        self.band = rnl.HeightBand(ground_z_m=b.GROUND_Z, robot_height_m=b.ROBOT_H)

    def flags(self, rec, wc=None, origin=None, dims=None):
        depth = np.zeros(MAX_PIXELS, np.float32)
        raw = rec['raw']
        values = np.asarray(raw, dtype=np.float32)*np.float32(rec['unit'])
        values[raw == 0] = np.nan
        depth[:raw.size] = values.ravel()
        dims = rec['dims'] if dims is None else dims
        flags = np.full(GRID_CAPACITY, 99, np.uint8)
        counts = np.full(3, 99, np.uint32)
        status = np.full(1, 99, np.uint32)
        self.geo(depth, np.ascontiguousarray(rec['ik'], np.float32),
                 np.ascontiguousarray((rec['wc'] if wc is None else wc)[:3], np.float32),
                 np.ascontiguousarray(rec['origin'] if origin is None else origin, np.float64),
                 rec['voxel'], raw.shape[1], raw.shape[0], *dims, flags, counts, status)
        if status[0] != 0:
            raise ValueError('geometry_status_%d' % status[0])
        if int(counts.astype(np.uint64).sum()) != raw.size:
            raise ValueError('nonconservative_pixel_counts')
        return flags[:int(np.prod(dims))].copy(), counts

    def costs(self, base, xy, body=True):
        dh, _, _, words, _ = self.roll.call(3, 0, 0, (0, 0, 0), (0, 0, 0), (0, 0, 0))
        assert int(dh[0]) == 0
        bp, tall = b.band_params(int(base[2]), self.band)
        circle = b.body_q(base, xy, b.BODY_R+b.BODY_MARGIN) if body else (0, 0, 0, 0)
        st, grid, aux, costs = self.cost.run(words, dh[8:11], bp, tall, circle, self.lut)
        if st != 0:
            raise ValueError('costmap_status_%d' % st)
        return costs.reshape(80, 80)


def calibration(pack, trace, old_maps, out):
    """Reproduce original source recipe, not the unrelated identity-pose pack geometry."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    chain = Chain(out/'build')
    records = read_trace(str(trace))
    trace_meta = json.loads(Path(str(trace)+'.json').read_text())
    old = np.load(old_maps)
    reference = old['maps'][old['labels'] == 'rec30']
    p = Pack(pack)
    assert len(p) == len(reference) == 310
    policy = FollowCameraGrid(.05, (32, 32, 10), back_cells=1, height_cell=DEFAULT_HEIGHT_CELL)
    details, maps, before_body, bases, poses = [], [], [], [], []
    failures = dict(flags=0, observation_base=0, window_base=0, header=0, delta=0, cost_frames=0, cost_bytes=0)
    k = 0
    for r, mr in zip(records, trace_meta['records']):
        if mr['seg'] != 'rec30':
            # Initial uninitialised/bad-reset controls are part of the source history.
            if k:
                break
        kw = {}
        if r['obs'] is not None:
            kw['obs'] = r['obs']
        if r['op'] == 2:
            kw.update(query=r['query'], qc=r['qc'])
        dims = r['dims']
        if mr['seg'] == 'rec30' and r['op'] == 1:
            rec = p.frame(k)
            tb = odom_base(k)
            wc = tb @ base_camera()
            gf = policy.grid_for(wc)
            flags, counts = chain.flags(rec, wc, gf.origin_m, gf.shape)
            ob = tuple(int(round(x/.05)) for x in gf.origin_m)
            wb = tuple((np.floor(tb[:3, 3]/.05).astype(int)-[40, 40, 8]).tolist())
            failures['flags'] += int(not np.array_equal(flags, np.frombuffer(r['obs'], np.uint8)))
            failures['observation_base'] += int(ob != tuple(r['ob']))
            failures['window_base'] += int(wb != tuple(r['wb']))
            kw['obs'] = flags.tobytes()
        else:
            ob, wb = r['ob'], r['wb']
        hdr, delta, _, _, _ = chain.roll.call(r['op'], r['epoch'], r['frame'], wb, ob, dims, **kw)
        failures['header'] += int(not np.array_equal(hdr, r['header']))
        failures['delta'] += int(not np.array_equal(delta, r['delta']))
        if mr['seg'] == 'rec30' and r['op'] == 1:
            base = tuple(b.signed(hdr[5+j]) for j in range(3))
            cost = chain.costs(base, b.body_pose(base, k+1))
            changed = int(np.count_nonzero(cost != reference[k]))
            failures['cost_frames'] += int(changed > 0)
            failures['cost_bytes'] += changed
            maps.append(cost); bases.append(base); poses.append(tb)
            before_body.append(chain.costs(base, tb[:3, 3], body=False))
            details.append(dict(frame=k, counts=counts.tolist(), cost_differing_bytes=changed,
                                cost_sha256=__import__('hashlib').sha256(cost.tobytes()).hexdigest()))
            k += 1
            if k % 50 == 0:
                print('rec30 calibrated', k, flush=True)
    assert k == 310
    np.savez_compressed(out/'rec30.npz', maps=maps, unseeded=before_body, bases=bases, body=poses)
    result = dict(code_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                  pack=Pack(pack).audit(), trace_sha256=sha(trace), trace_meta_sha256=sha(str(trace)+'.json'),
                  previous_maps_sha256=sha(old_maps), compared_frames=k, compared_bytes=k*80*80,
                  failures=failures, passed=not any(failures.values()), frames=details,
                  pose_source='synthetic odom_base; regression only; pack wc is identity')
    write_json(out/'calibration.json', result)
    if not result['passed']:
        raise SystemExit('rec30 calibration FAILED; confirmation maps prohibited')
    print('rec30 all calibration bytes match', flush=True)


def source_audit(e2, g1d, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    audits = {name: Pack(path).audit() for name, path in [
        ('rec30', Path(e2)/'rec30.pack'), ('car570', Path(e2)/'car570.pack'),
        ('car1015', Path(g1d)/'car1015.pack')]}
    meta_path = Path(g1d)/'car1015.pack.meta.json'
    meta = json.loads(meta_path.read_text())
    audits['car1015'].update(meta_sha256=sha(meta_path), mount=meta['mount_local_transform'],
                              trajectory_source=meta['depth_npz'])
    write_json(out/'pack_audit.json', audits)
    print(json.dumps({k: {s:v[s] for s in ['frames', 'unique_matrices', 'translation_min', 'translation_max']} for k,v in audits.items()}, indent=2))


def generate_car1015(g1d, calibration_file, out):
    if not json.loads(Path(calibration_file).read_text())['passed']:
        raise ValueError('calibration_not_passed')
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    meta_path = Path(g1d)/'car1015.pack.meta.json'
    meta = json.loads(meta_path.read_text())
    p = Pack(Path(g1d)/'car1015.pack')
    source = np.load(meta['depth_npz'])
    ids, stamps, camera, k = (source[v] for v in ['ids', 'stamps', 'c2w', 'K'])
    if (not np.array_equal(ids, meta['ids']) or not np.array_equal(stamps, meta['stamps']) or
            not np.array_equal(k, meta['K']) or len(camera) != len(p) or not np.all(np.diff(stamps) > 0)):
        raise ValueError('source_identity_mismatch')
    mount = np.asarray(meta['mount_local_transform'])
    body = body_poses(camera, mount)
    policy = FollowCameraGrid(.05, (32, 32, 10), back_cells=1, height_cell=8)
    chain = Chain(out/'build')
    maps, unseeded, bases, details = [], [], [], []
    depth_mismatch = 0
    # Stream only depth.npy, never materialise the multi-GB RGB array.
    with zipfile.ZipFile(meta['depth_npz']) as archive, archive.open('depth.npy') as f:
        version = np.lib.format.read_magic(f)
        shape, fortran, dtype = np.lib.format._read_array_header(f, version)
        if shape != (len(p), 480, 640) or fortran or dtype != np.dtype('float32'):
            raise ValueError('source_depth_format')
        for i in range(len(p)):
            rec = p.frame(i)
            src = np.frombuffer(f.read(480*640*4), dtype=dtype).reshape(480, 640)
            mismatch = int(np.count_nonzero(np.rint(src.astype(np.float64)*1000).astype(np.uint16) != rec['raw']))
            depth_mismatch += mismatch
            if mismatch:
                raise ValueError('source_depth_mismatch_frame_%d' % i)
            if not np.allclose(rec['wc'], mount, atol=1e-7, rtol=0):
                raise ValueError('pack_mount_not_matching_metadata')
            if not np.array_equal(rec['ik'], np.linalg.inv(k).astype(np.float32)):
                raise ValueError('pack_intrinsics_mismatch')
            gf = policy.grid_for(camera[i])
            flags, counts = chain.flags(rec, camera[i], gf.origin_m, gf.shape)
            ob = tuple(int(round(v/.05)) for v in gf.origin_m)
            base = tuple((np.floor(body[i, :3, 3]/.05).astype(int)-[40, 40, 8]).tolist())
            if i == 0:
                h, *_ = chain.roll.call(0, 1, 0, base, (0, 0, 0), (0, 0, 0))
                assert int(h[0]) == 0
            h, delta, _, _, _ = chain.roll.call(1, 1, i+1, base, ob, gf.shape, obs=flags.tobytes())
            assert int(h[0]) == 0
            assert base == tuple(b.signed(h[5+j]) for j in range(3))
            try:
                costs = chain.costs(base, body[i, :3, 3])
                un = chain.costs(base, body[i, :3, 3], body=False)
                maps.append(costs); unseeded.append(un); bases.append(base)
                details.append(dict(frame=i, valid=True, counts=counts.tolist(), delta_words=len(delta)))
            except ValueError as exc:
                maps.append(np.full((80, 80), 255, np.uint8)); unseeded.append(maps[-1].copy()); bases.append(base)
                details.append(dict(frame=i, valid=False, reason=str(exc)))
            if (i+1) % 100 == 0:
                print('car1015 generated', i+1, flush=True)
        if f.read(1):
            raise ValueError('source_depth_trailing_bytes')
    np.savez_compressed(out/'car1015.npz', maps=maps, unseeded=unseeded, bases=bases, body=body,
                        camera=camera, stamps=stamps, ids=ids)
    write_json(out/'generation.json', dict(code_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        pack_sha256=sha(p.path), meta_sha256=sha(meta_path), source_npz_sha256=sha(meta['depth_npz']),
        source_npz=meta['depth_npz'], depth_mismatch_pixels=depth_mismatch, frames=details,
        pose_source='RTAB-Map optimized keyframes plus nearest-keyframe odometry correction, offline',
        translation_min=body[:, :3, 3].min(0), translation_max=body[:, :3, 3].max(0),
        maximum_step_xy=float(np.linalg.norm(np.diff(body[:, :2, 3], axis=0), axis=1).max()),
        params=dict(ground_z=b.GROUND_Z, robot_height=b.ROBOT_H, body_radius=b.BODY_R+b.BODY_MARGIN,
                    height_cell=8, observation_dims=[32,32,10], window_dims=[80,80,32], resolution=.05)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=['audit','calibrate','car1015'])
    ap.add_argument('--e2', required=True); ap.add_argument('--g1d', required=True)
    ap.add_argument('--out', required=True); ap.add_argument('--trace'); ap.add_argument('--old-maps')
    ap.add_argument('--calibration')
    a = ap.parse_args()
    if a.mode == 'audit':
        source_audit(a.e2, a.g1d, a.out)
    elif a.mode == 'calibrate':
        calibration(Path(a.e2)/'rec30.pack', a.trace, a.old_maps, a.out)
    else:
        generate_car1015(a.g1d, a.calibration, a.out)


if __name__ == '__main__':
    main()
