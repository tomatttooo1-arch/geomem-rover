"""Deterministic hindsight targets, explicit pose controls and frozen v3 comparison."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import numpy as np
from tools.planner_model.aligned_v2 import MapMetadata, PoseSample, world_to_grid
from tools.planner_model.aligned_v3 import PlannerV3
from tools.planner_model.evaluate import Geometry, distribution
from tools.planner_model.realpose.compare import NavFn, verify_frozen, append_actual_start, metric, summarize
from tools.planner_model.realpose.core import first_exit, inside, MissingInput
from tools.planner_model.hindsight.inputs import write_json, sha
from tools.planner_model.validate_v2 import serial


def target(points, base):
    """First observed future point >=1m, or first exit before that point (no resampling)."""
    points = np.asarray(points, float)
    if len(points) < 2:
        raise MissingInput('no_future_point_at_least_1m')
    raw = points/.05 - np.asarray(base[:2])
    low, high = np.array([1., 1.]), np.array([78., 78.])
    if not inside(raw[0], (low, high)):
        raise MissingInput('start_outside_effective_window')
    for j in range(1, len(points)):
        if not inside(raw[j], (low, high)):
            exit_point = first_exit(raw[:j+1], low, high)
            try:
                physical = first_exit(raw[:j+1], [0., 0.], [80., 80.])
            except MissingInput:
                physical = None
            return exit_point, dict(kind='first_trajectory_exit', future_offset=j,
                                    physical_exit=physical, trajectory_grid=raw[:j+1])
        if np.linalg.norm(points[j]-points[0]) >= 1.0:
            return raw[j], dict(kind='first_future_point_at_least_1m', future_offset=j,
                               physical_exit=None, trajectory_grid=raw[:j+1])
    raise MissingInput('no_future_point_at_least_1m')


def point_cost(cost, xy, base):
    q = np.asarray(xy)/.05-np.asarray(base[:2])
    cell = np.floor(q+.5).astype(int)
    if np.any(q < 0) or np.any(cell < 0) or np.any(cell >= 80):
        return -1
    return int(cost[cell[1], cell[0]])


def pose_controls(unseeded, body, bases, valid):
    details = []
    for i in range(len(body)):
        xy = body[i, :2, 3]
        samples = dict(normal=xy, x_plus_1m=xy+[1., 0.], y_plus_1m=xy+[0., 1.],
                       rotation_90=np.array([-xy[1], xy[0]]))
        details.append(dict(frame=i, valid=bool(valid[i]),
                            **{k: point_cost(unseeded[i], p, bases[i]) for k,p in samples.items()}))
    stats = {}
    for key in ['normal','x_plus_1m','y_plus_1m','rotation_90']:
        values = [r[key] for r in details if r['valid']]
        n = len(values)
        bad = sum(v < 0 or v >= 253 for v in values)
        stats[key] = dict(n=n, bad=bad, bad_rate=bad/n if n else None,
                          nonfatal_rate=(n-bad)/n if n else None,
                          cost_histogram=dict(Counter(str(v) for v in values)))
    normal, shifted = stats['normal'], stats['x_plus_1m']
    difference = shifted['bad_rate']-normal['bad_rate'] if normal['n'] else None
    return dict(planes='before body seeding and request-start clearing', stats=stats,
                normal_pass=bool(normal['n']) and normal['nonfatal_rate'] >= .95,
                x_positive_increase=difference, positive_pass=difference is not None and difference >= .10,
                passed=bool(normal['n']) and normal['nonfatal_rate'] >= .95 and difference >= .10,
                excluded_invalid_maps=len(body)-normal['n']), details


def evaluate(dataset, path, generation, planner, nav, out):
    data = np.load(path)
    maps, body, bases = data['maps'], data['body'], data['bases']
    valid = np.ones(len(maps), bool)
    if generation:
        valid = np.array([r['valid'] for r in json.loads(Path(generation).read_text())['frames']])
    controls, control_rows = pose_controls(data['unseeded'], body, bases, valid)
    write_json(out/(dataset+'_pose_controls.json'), controls)
    seeded, _ = pose_controls(maps, body, bases, valid)
    seeded['planes'] = 'formal body-seeded maps; diagnostic only, cannot substitute unseeded gate'
    write_json(out/(dataset+'_body_seeded_controls.json'), seeded)
    rows, paths, rejects = [], [], []
    requests = len(range(0, len(maps), 5))
    for i in range(0, len(maps), 5):
        source_id = int(data['ids'][i]) if 'ids' in data else i
        source_stamp = float(data['stamps'][i]) if 'stamps' in data else i*.1
        stamp_ns = int(round(source_stamp*1e9))
        try:
            if not valid[i]:
                raise MissingInput('invalid_height_band_or_map')
            m = np.ascontiguousarray(maps[i], np.uint8)
            base = bases[i]
            meta = MapMetadata('offline_world', i, stamp_ns, .05, float(base[0]*.05), float(base[1]*.05), 0., 'navfn_round')
            xy = body[i, :2, 3]
            start, actual = world_to_grid(meta, xy, m.shape)
            goal, proof = target(body[i:, :2, 3], base)
            goalcell = np.floor(goal+.5).astype(int)
            gcost = int(m[goalcell[1], goalcell[0]])
            if gcost >= 253:
                raise MissingInput('goal_unknown' if gcost == 255 else 'goal_lethal_or_inscribed')
            effective = m.copy(); effective[start[1], start[0]] = 0
            geom, original = Geometry(effective), Geometry(m)
            navpath = append_actual_start(nav.plan(m, start, goal, .05), actual)
            row = dict(dataset=dataset, frame=i, source_id=source_id, source_stamp_s=source_stamp, source_stamp_ns_rounded=stamp_ns,
                       window_base=base, start_world=xy, start_cell=start, start_cost=int(m[start[1], start[0]]),
                       goal=goal, goal_cost=gcost, goal_kind=proof['kind'], future_frame=i+proof['future_offset'],
                       navfn=dict(metric(navpath, geom, .05), original_map_hits=original.collisions(navpath)))
            pp = dict(dataset=dataset, frame=i, navfn=navpath, trajectory=proof['trajectory_grid'])
            pose = PoseSample(meta.frame_id, i, stamp_ns, float(xy[0]), float(xy[1]),
                              'recorded_pose' if dataset == 'car1015' else 'synthetic_pose')
            for v in [1, 2]:
                rr = []
                for mode in ['float','fixed']:
                    if v == 2:
                        result = planner.plan(m, meta, pose, (goal+base[:2])*.05, meta.frame_id, mode=mode)
                    else:
                        result = planner.plan_cells(m, start, goal, .05, .15, mode, variant=v)
                        result['path'] = append_actual_start(result['path'], actual)
                        if geom.collisions(result['path']):
                            result.update(success=False, status='safety_rejected', path=np.empty((0,2)))
                    rr.append(result)
                f, r = rr
                row['v3_%d'%v] = dict(metric(r['path'], geom, .05), status=r['status'],
                    fixed_equal=f['status'] == r['status'] and np.array_equal(f['path'], r['path']),
                    original_map_hits=original.collisions(r['path']))
                pp['v3_%d'%v] = r['path']
            rows.append(row); paths.append(pp)
        except (MissingInput, ValueError) as exc:
            rejects.append(dict(dataset=dataset, frame=i, source_id=source_id, source_stamp_s=source_stamp, reason=str(exc)))
    variants = {}
    for v in [1, 2]:
        name = 'v3_%d'%v
        summary = summarize(rows, name, requests)
        matches = sum(r[name]['fixed_equal'] for r in rows)
        summary.update(covered_navfn_success=sum(r['navfn']['success'] and r[name]['success'] for r in rows), fixed_matches=matches, fixed_compared=len(rows), fixed_rate=matches/len(rows) if rows else None,
                       all_request_success_rate=summary['candidate_success']/requests if requests else None,
                       policy_collision_outputs=sum(bool(r[name]['hits']) for r in rows),
                       original_collision_outputs=sum(bool(r[name]['original_map_hits']) for r in rows),
                       statuses=dict(Counter(r[name]['status'] for r in rows)))
        summary['gates']['fixed'] = bool(rows) and matches/len(rows) >= .99
        variants[str(v)] = summary
    summary = dict(dataset=dataset, frames=len(maps), requests=requests, accepted=len(rows), rejected=len(rejects),
        rejection_reasons=dict(Counter(r['reason'] for r in rejects)), below_200=len(rows)<200,
        navfn_all_request_success=sum(r['navfn']['success'] for r in rows)/requests,
        navfn_policy_collision_outputs=sum(bool(r['navfn']['hits']) for r in rows),
        pose_calibration=controls['passed'], variants=variants,
        real_pose_confirmation=dataset == 'car1015' and controls['passed'] and len(rows)>=200 and
                               all(variants['2']['gates'].values()),
        evidence_kind='offline_reconstructed_pose_diagnostic' if dataset == 'car1015' else 'synthetic_motion_regression',
        input_sha256=sha(path))
    write_json(out/(dataset+'_summary.json'), summary)
    for key, entries in [('pairs', rows), ('paths', paths), ('rejected', rejects), ('control_rows', control_rows)]:
        (out/(dataset+'_'+key+'.jsonl')).write_text(''.join(json.dumps(r, default=serial, allow_nan=False)+'\n' for r in entries))
    return summary, rows, paths


def car570_rejections(out):
    audit = json.loads((out.parent/'audit'/'pack_audit.json').read_text())['car570']
    n = audit['frames']
    summary = dict(dataset='car570', frames=n, requests=len(range(0,n,5)), accepted=0,
        rejected=len(range(0,n,5)), rejection_reasons={'missing_world_trajectory':len(range(0,n,5))},
        real_pose_confirmation=False, below_200=True, variants=None, pose_calibration=None)
    write_json(out/'car570_summary.json', summary)
    (out/'car570_rejected.jsonl').write_text(''.join(json.dumps(dict(dataset='car570',frame=i,
        reason='missing_world_trajectory'))+'\n' for i in range(0,n,5)))
    return summary


def render_examples(all_rows, all_paths, sources, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    def rank(row):
        n, v = row['navfn'], row['v3_2']
        mismatch = n['success'] != v['success']
        a = row['v3_1']
        deviation = abs(a['length']/n['length']-1) if n['success'] and a['success'] and n['length']>1e-12 else 0.
        return (-int(mismatch), -deviation, row['dataset'], row['frame'])
    rows = sorted(all_rows, key=rank)[:5]
    path_by_key = {(r['dataset'],r['frame']):r for r in all_paths}
    selected = []
    for j, row in enumerate(rows):
        ds, i = row['dataset'], row['frame']
        pp = path_by_key[ds,i]
        m = np.load(sources[ds])['maps'][i]
        fig, ax = plt.subplots(figsize=(7,7))
        ax.imshow(m, origin='lower', vmin=0, vmax=255, cmap='gray_r')
        for key, label, color, style in [('trajectory','future observed trajectory','#e28c00','--'),
                ('navfn','NavFn','#2171b5','-'),('v3_1','v3 variant 1','#de2d26','-'),('v3_2','v3 variant 2','#31a354','-')]:
            p = np.asarray(pp[key])
            if len(p):
                ax.plot(p[:,0],p[:,1],style,color=color,label=label,lw=1.7)
            else:
                ax.plot([],[],style,color=color,label=label+' (no path)')
        ax.scatter(*row['start_cell'],c='cyan',edgecolors='black',zorder=5)
        ax.scatter(*row['goal'],c='magenta',marker='x',zorder=5)
        ax.set(xlim=(0,79),ylim=(0,79),xlabel='local grid x (0.05 m)',ylabel='local grid y (0.05 m)',
               title='%s frame %d | diagnostic; pose gate not substituted' % (ds,i))
        ax.legend(loc='best',fontsize=8)
        name='difference_%d_%s_%d.png'%(j+1,ds,i)
        fig.tight_layout(); fig.savefig(out/name,dpi=140); plt.close(fig)
        selected.append(dict(image=name, row=row, ranking=list(rank(row))))
    write_json(out/'selected_examples.json', selected)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', required=True); ap.add_argument('--lib',required=True); ap.add_argument('--navfn-lib',required=True)
    a=ap.parse_args(); verify_frozen()
    root=Path(a.root); out=root/'comparison'; out.mkdir(parents=True,exist_ok=False)
    if not json.loads((root/'calibration'/'calibration.json').read_text())['passed']:
        raise ValueError('rec30_calibration_failed')
    p, nav=PlannerV3(a.lib),NavFn(a.navfn_lib)
    summaries=[car570_rejections(out)]; rows=[]; paths=[]
    sources={'rec30':root/'calibration'/'rec30.npz','car1015':root/'car1015'/'car1015.npz'}
    for ds in ['rec30','car1015']:
        ss,rr,pp=evaluate(ds,sources[ds],root/'car1015'/'generation.json' if ds=='car1015' else None,p,nav,out)
        summaries.append(ss); rows.extend(rr); paths.extend(pp)
        print(ds,json.dumps(ss,default=serial),flush=True)
    render_examples(rows,paths,sources,out)
    write_json(out/'summary.json',dict(code_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        frozen_v3='21069ecedb8865e32ca801b6fc9244e5ffbc3835',datasets=summaries,real_pose_confirmation=False))


if __name__=='__main__':
    main()
