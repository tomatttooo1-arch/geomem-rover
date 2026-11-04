"""Replay B1's pinned C model, verify arithmetic, keep source-segment labels. CPU only."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from tools.b1_costmap import bench as b
from tools.b1_costmap import nav2_ref
from host.kv_rolling import CpuRollingCore
from host.m7_rolling import compile_rolling
from host import rolling_nav_layer as rnl
from tools.m7_rolling_pl.replay_trace import read_trace


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def generate(trace, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    recs = read_trace(str(trace))
    meta = json.loads(Path(str(trace) + '.json').read_text())
    assert len(meta['records']) == len(recs)
    roll = CpuRollingCore(compile_rolling(str(out / 'roll.so')))
    pl = b.PlCore(b.build_costmap(str(out / 'costmap.so')))
    band = rnl.HeightBand(ground_z_m=b.GROUND_Z, robot_height_m=b.ROBOT_H)
    params = nav2_ref.InflationParams()
    lut = params.pl_lut(b.LUT_DIM)
    expected = [254, 253, 253, 253, 253, 253, 247, 213, 183, 157, 0, 0, 0, 0]
    assert lut.reshape(14, 14)[0].tolist() == expected
    # Isolated obstacle in known free space exercises actual inflate_nav2 arithmetic, not just LUT.
    isolated = np.zeros((80, 80), np.int32)
    isolated[40, 40] = 254
    axial = nav2_ref.inflate_nav2(isolated, params)[40, 40:54].tolist()
    assert axial == expected
    maps, starts, labels, details = [], [], [], []
    accepted = skipped = differing = 0
    max_abs = 0
    for idx, (r, mr) in enumerate(zip(recs, meta['records'])):
        kw = {}
        if r['obs'] is not None:
            kw['obs'] = r['obs']
        if r['op'] == 2:
            kw.update(query=r['query'], qc=r['qc'])
        hdr, delta, q, dump, _ = roll.call(r['op'], r['epoch'], r['frame'], r['wb'], r['ob'], r['dims'], **kw)
        assert np.array_equal(hdr, r['header']), ('header mismatch', idx)
        assert np.array_equal(delta, r['delta']), ('delta mismatch', idx)
        if r['op'] == 0 and int(hdr[0]) == 0:
            bridge = b.new_bridge(band, False)
        if r['op'] != 1 or int(hdr[0]) != 0:
            continue
        accepted += 1
        base = tuple(b.signed(hdr[b.H_BASE + k]) for k in range(3))
        now = accepted * 100000000
        nd = b.NavigationDelta(int(r['epoch']), int(hdr[b.H_DELTA_SEQ]), int(r['frame']), now, 'b1',
                               now + 10 ** 15, base, (80, 80, 32), .05,
                               b.decode_delta(hdr, delta), 'trace')
        ok, why, _ = bridge.on_delta(nd, now_ns=now)
        assert ok, why
        try:
            bp, tall = b.band_params(base[2], band)
        except ValueError:
            skipped += 1
            continue
        xy = b.body_pose(base, accepted)
        dh, _, _, words, _ = roll.call(3, 0, 0, (0, 0, 0), (0, 0, 0), (0, 0, 0))
        status, grid, aux, cost = pl.run(words, dh[8:11], bp, tall,
                                        b.body_q(base, xy, b.BODY_R + b.BODY_MARGIN), lut)
        assert status == 0
        sw = bridge.grid(body_xy=xy[:2], now_ns=now, body_yaw=xy[2]).data.astype(np.int8)
        assert np.array_equal(grid, sw), ('projection/body mismatch', idx)
        master = nav2_ref.master_from_grid(grid.reshape(80, 80))
        exact = nav2_ref.inflate_exact(master, params).astype(np.uint8)
        nav = nav2_ref.inflate_nav2(master, params).astype(np.uint8)
        cost = cost.reshape(80, 80)
        assert np.array_equal(cost, exact), ('C model inflation mismatch', idx)
        diff = int(np.count_nonzero(cost != nav))
        differing += diff
        max_abs = max(max_abs, int(np.abs(cost.astype(int) - nav).max()))
        sx, sy = (int(np.floor(xy[a] / .05 - base[a])) for a in range(2))
        maps.append(cost)
        starts.append((sx, sy))
        labels.append(mr['seg'])
        details.append(dict(index=len(maps), accepted_index=accepted, record=idx, trace_frame=r['frame'],
                            segment=mr['seg'], start=[sx, sy], start_cost=int(cost[sy, sx]),
                            histogram={str(int(v)): int(n) for v, n in zip(*np.unique(cost, return_counts=True))},
                            nav2_differing_cells=diff))
        if len(maps) % 50 == 0:
            print('generated', len(maps), 'source', mr['seg'], flush=True)
    assert len(maps) == 376, len(maps)
    dest = out / 'maps.npz'
    np.savez_compressed(str(dest), maps=np.asarray(maps, np.uint8), starts=np.asarray(starts),
                        labels=np.asarray(labels))
    result = dict(trace=str(Path(trace).resolve()), trace_sha256=sha(trace), metadata_sha256=sha(str(trace)+'.json'),
                  source_b1_commit='4cfaed7bcb56c2b38ec62a3233a88b3443582ee3', accepted=accepted,
                  skipped_band=skipped, maps=len(maps), isolated_axial=axial,
                  source_counts={x: labels.count(x) for x in sorted(set(labels))},
                  cost_vs_nav2_differing_cells=differing, cost_vs_nav2_max_abs=max_abs,
                  maps_sha256=sha(dest), frames=details)
    (out / 'generation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'frames'}, indent=2), flush=True)
    return dest


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trace', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    generate(args.trace, args.out)
