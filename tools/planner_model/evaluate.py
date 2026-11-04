"""Deterministic metrics, continuous collision checks, controls and PNG examples (no ROS)."""
import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS')
F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
SEED = 20261102


class Planner:
    def __init__(self, lib):
        self.lib = ctypes.CDLL(str(Path(lib).resolve()))
        common = [U8] + [ctypes.c_int]*6 + [F64, F64, ctypes.c_int]
        self.lib.wave_plan.argtypes = common + [ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
        self.lib.navfn_plan.argtypes = common + [ctypes.c_double, ctypes.c_int]
        self.lib.wave_plan.restype = self.lib.navfn_plan.restype = ctypes.c_int

    def run(self, m, start, goal, mode):
        m = np.ascontiguousarray(m, np.uint8)
        h, w = m.shape
        cap = w*h+2
        x, y = np.zeros(cap), np.zeros(cap)
        args = [m, w, h, int(start[0]), int(start[1]), int(goal[0]), int(goal[1]), x, y, cap]
        rounds = ctypes.c_uint32()
        if mode in ('navfn', 'navfn_native'):
            n = self.lib.navfn_plan(*(args + [.15, int(mode != 'navfn_native')]))
        else:
            n = self.lib.wave_plan(*(args + [{'float': 0, 'fixed': 1, 'mutant': 2}[mode], ctypes.byref(rounds)]))
        if n < 0:
            raise RuntimeError(('invalid planner call', mode, m.shape, start, goal, n))
        return np.column_stack((x[:n], y[:n])), int(rounds.value)


def length(path):
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()*.05) if len(path) else None


def segment_cells(a, b, shape):
    """Exact slab intersection against closed squares centred at integer cell indices.

    Includes corner contacts, intermediate cells, and out-of-bounds cells; no vertex-only sampling.
    """
    lo = np.ceil(np.minimum(a, b)-.5-1e-10).astype(int)
    hi = np.floor(np.maximum(a, b)+.5+1e-10).astype(int)
    xx, yy = np.meshgrid(np.arange(lo[0], hi[0]+1), np.arange(lo[1], hi[1]+1))
    cells = np.column_stack((xx.ravel(), yy.ravel()))
    tlo, thi = np.zeros(len(cells)), np.ones(len(cells))
    d = b-a
    for axis in range(2):
        lower, upper = cells[:, axis]-.5, cells[:, axis]+.5
        if abs(d[axis]) < 1e-14:
            keep = (a[axis] >= lower-1e-10) & (a[axis] <= upper+1e-10)
            thi[~keep] = -1
        else:
            t1, t2 = (lower-a[axis])/d[axis], (upper-a[axis])/d[axis]
            tlo = np.maximum(tlo, np.minimum(t1, t2))
            thi = np.minimum(thi, np.maximum(t1, t2))
    return cells[tlo <= thi+1e-10]


class Geometry:
    def __init__(self, m):
        self.m = m
        yy, xx = np.nonzero(m >= 253)
        self.obstacles = np.column_stack((xx, yy)).astype(float)
        self.tree = cKDTree(self.obstacles) if len(xx) else None

    def collisions(self, path):
        if len(path) == 0:
            return []
        segments = zip(path[:-1], path[1:]) if len(path)>1 else [(path[0], path[0])]
        hits = set()
        h, w = self.m.shape
        for a, b in segments:
            for x, y in segment_cells(a, b, self.m.shape):
                if not (0 <= x < w and 0 <= y < h):
                    hits.add((int(x), int(y), -1))
                elif self.m[y, x] >= 253:
                    hits.add((int(x), int(y), int(self.m[y, x])))
        return sorted(hits)

    def clearance(self, path):
        if not len(path):
            return None
        if self.tree is None:
            return math.hypot(*self.m.shape)  # map-diagonal sentinel; no-obstacle count reported separately
        best = float(np.min(self.tree.query(path)[0]))
        for a, b in zip(path[:-1], path[1:]):
            d = b-a
            den = float(np.dot(d, d))
            if den < 1e-20:
                continue
            # Any improving obstacle centre lies within best + half segment length of midpoint.
            ids = self.tree.query_ball_point((a+b)*.5, best + math.sqrt(den)*.5 + 1e-9)
            if not ids:
                continue
            q = self.obstacles[ids]-a
            t = np.clip(q.dot(d)/den, 0, 1)
            best = min(best, float(np.linalg.norm(q-t[:, None]*d, axis=1).min()))
        return best

    def smooth(self, path):
        if len(path)<3:
            return path.copy()
        result, i = [path[0]], 0
        while i < len(path)-1:
            for j in range(len(path)-1, i, -1):
                if not self.collisions(np.array([path[i], path[j]])):
                    break
            result.append(path[j]); i = j
        return np.asarray(result)


def goals(m, start, index):
    yy, xx = np.nonzero(m < 253)
    all_points = [(int(x), int(y)) for x, y in zip(xx, yy) if (x, y) != tuple(start)]
    rng = np.random.RandomState(SEED+index)
    chosen = [all_points[int(i)] for i in rng.choice(len(all_points), min(20, len(all_points)), replace=False)]
    h, w = m.shape
    edge = [p for p in all_points if min(p[0], p[1], w-1-p[0], h-1-p[1]) <= 3 and p not in chosen]
    edge.sort(key=lambda p: (-((p[0]-start[0])**2+(p[1]-start[1])**2), p[1], p[0]))
    return [(p, 'random') for p in chosen] + [(p, 'far_edge') for p in edge[:5]]


def distribution(values):
    if not values:
        return dict(n=0, median=None, p95=None, max=None)
    return dict(n=len(values), median=float(np.median(values)), p95=float(np.percentile(values, 95)), max=float(np.max(values)))


def summarize(rows):
    base = [r for r in rows if r['navfn_success']]
    result = dict(pairs=len(rows), navfn_success=len(base), goal_exact_navfn=sum(r['navfn_exact'] for r in base),
                  wave_success=sum(r['float_success'] for r in base),
                  success_rate=sum(r['float_success'] for r in base)/len(base) if base else None,
                  safety_cells=sum(len(r['float_hits']) for r in base),
                  navfn_safety_cells=sum(len(r['navfn_hits']) for r in base),
                  all_wave_safety_cells=sum(len(r['float_hits']) for r in rows),
                  fixed_match_rate=sum(r['fixed_equal'] for r in base)/len(base) if base else None,
                  both_success_fixed_match_rate=None,
                  navfn_fail_wave_success=sum(not r['navfn_success'] and r['float_success'] for r in rows),
                  rounds=distribution([r['rounds'] for r in rows if r['rounds']]),
                  cycles=distribution([r['rounds']*r['cells']*2 for r in rows if r['rounds']]),
                  no_obstacle_pairs=sum(r['no_obstacles'] for r in rows),
                  fewer_than_25_frames=len(set(r['frame'] for r in rows if r['goals_in_frame']<25)))
    both = [r for r in base if r['float_success']]
    if both:
        result['both_success_fixed_match_rate'] = sum(r['fixed_equal'] for r in both)/len(both)
    for mode in ['float', 'smooth', 'mutant']:
        selected = [r for r in base if r[mode+'_success'] and r['navfn_length']>1e-12]
        nc = [r['navfn_clearance'] for r in base]  # Full NavFn-success denominator, including wave failures.
        wc = [r[mode+'_clearance'] for r in selected]
        result[mode] = dict(length_ratio=distribution([r[mode+'_length']/r['navfn_length'] for r in selected]),
                            clearance_navfn_median=float(np.median(nc)) if nc else None,
                            clearance_wave_median=float(np.median(wc)) if wc else None,
                            safety_cells=sum(len(r[mode+'_hits']) for r in base),
                            success_rate=sum(r[mode+'_success'] for r in base)/len(base) if base else None)
        s = result[mode]
        ratio = s['length_ratio']
        s['gates'] = dict(safety=s['safety_cells']==0 if base else False,
                          success=s['success_rate'] is not None and s['success_rate']>=.99,
                          length=ratio['n']>0 and ratio['median']<=1.05 and ratio['p95']<=1.15,
                          clearance=bool(nc) and bool(wc) and s['clearance_wave_median']>=s['clearance_navfn_median']-1,
                          fixed=result['fixed_match_rate'] is not None and result['fixed_match_rate']>=.99)
    return result


def controls(planner):
    m = np.zeros((64, 64), np.uint8)
    start, goal = (8, 12), (54, 48)
    old, _ = planner.run(m, start, goal, 'float')
    mid = np.rint(old[len(old)//2]).astype(int)
    wall = m.copy(); wall[max(1, mid[1]-12):min(63, mid[1]+13), mid[0]] = 254
    g = Geometry(wall)
    checks = {}
    for mode in ['float', 'fixed', 'navfn']:
        path, _ = planner.run(wall, start, goal, mode)
        checks[mode] = dict(success=bool(len(path)), hits=g.collisions(path))
    injected = g.collisions(old)
    assert injected and all(not x['hits'] for x in checks.values())
    lethal = m.copy(); lethal[goal[1], goal[0]] = 254
    rejected = {mode: len(planner.run(lethal, start, goal, mode)[0])==0 for mode in ['float','fixed','navfn']}
    native = bool(len(planner.run(lethal, start, goal, 'navfn_native')[0]))
    unknown = lethal.copy(); unknown[goal[1], goal[0]] = 255
    unknown_rejected = {mode: len(planner.run(unknown, start, goal, mode)[0])==0 for mode in ['float','fixed','navfn']}
    assert all(rejected.values()) and all(unknown_rejected.values())
    ratios, broken = [], []
    for target in [(48, 48), (48, 40), (40, 48), (50, 50)]:
        nav, _ = planner.run(m, (8, 8), target, 'navfn')
        wave, _ = planner.run(m, (8, 8), target, 'float')
        mutant, _ = planner.run(m, (8, 8), target, 'mutant')
        ratios.append(length(wave)/length(nav)); broken.append(length(mutant)/length(nav))
    assert np.median(broken) > np.median(ratios)+.2
    # Segment check must catch a wall skipped by both endpoints and unknown corner contact.
    test = np.zeros((8, 8), np.uint8);test[3, 3]=255
    assert Geometry(test).collisions(np.array([[1., 1.], [5., 5.]]))
    return dict(wall=checks, injected_wall_hits=injected, lethal_rejected=rejected,
                native_navfn_lethal_tolerance_success=native, unknown_rejected=unknown_rejected,
                diagonal_control=dict(normal_ratios=ratios, mutant_ratios=broken,
                                      normal_median=float(np.median(ratios)), mutant_median=float(np.median(broken))),
                intermediate_unknown_segment_detected=True, verdict='PASS')


def synthetic_maps():
    from scipy.ndimage import distance_transform_edt
    maps = []
    for kind in range(3):
        m = np.zeros((400, 400), np.uint8)
        m[:2, :] = m[-2:, :] = m[:, :2] = m[:, -2:] = 254
        if kind == 0:  # rooms, doors and furniture
            for x in [100, 200, 300]:
                m[2:398, x:x+3]=254
                m[45+kind*20:75+kind*20, x:x+3]=0
                m[275:310, x:x+3]=0
            for x,y in [(50,180),(150,130),(250,220),(345,180)]:
                m[y:y+25,x:x+18]=254
        elif kind == 1:  # alternating corridor turns
            for i,x in enumerate([70,140,210,280,350]):
                m[(2 if i%2==0 else 80):(320 if i%2==0 else 398),x:x+4]=254
        else:  # furniture islands and cross-room walls
            m[195:199, 2:398]=254;m[195:199,180:220]=0
            for y in [65,120,270,325]:
                for x in [60,140,240,320]:
                    m[y:y+20,x:x+25]=254
        dist = distance_transform_edt(m != 254)
        from tools.b1_costmap.nav2_ref import InflationParams
        p = InflationParams()
        mask = (dist>0) & (dist<=9)
        m[mask] = np.asarray([p.compute_cost(d) for d in dist[mask]], np.uint8)
        maps.append(m)
    return maps


def evaluate(planner, m, start, frame, label, large=False):
    geom = Geometry(m)
    rows, paths = [], []
    targets = goals(m, start, frame)
    for j, (goal, kind) in enumerate(targets):
        nav, _ = planner.run(m, start, goal, 'navfn')
        diagnostics = [planner.lib.navfn_debug(k) for k in range(7)]
        wave, rounds = planner.run(m, start, goal, 'float')
        fixed, fr = planner.run(m, start, goal, 'fixed')
        smoothed = geom.smooth(wave)
        mutant, _ = planner.run(m, start, goal, 'mutant')
        assert rounds == fr
        row = dict(frame=frame, source=label, pair=j, goal=list(goal), goal_kind=kind,
                   navfn_diagnostics=diagnostics, start_cost=int(m[start[1],start[0]]), goal_cost=int(m[goal[1],goal[0]]),
                   start=list(map(int, start)), cells=m.size, rounds=rounds, goals_in_frame=len(targets),
                   no_obstacles=geom.tree is None, fixed_equal=bool(np.array_equal(wave,fixed)),
                   navfn_exact=bool(len(nav) and np.linalg.norm(nav[-1]-goal)<1e-6))
        for mode,path in [('navfn',nav),('float',wave),('smooth',smoothed),('mutant',mutant)]:
            row.update({mode+'_success':bool(len(path)),mode+'_length':length(path),
                        mode+'_hits':geom.collisions(path),mode+'_clearance':geom.clearance(path)})
        rows.append(row)
        paths.append(dict(frame=frame, pair=j, navfn=nav.tolist(), wave=wave.tolist(), smooth=smoothed.tolist()))
    return rows, paths


def plot_example(m, row, path, dest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(m, origin='lower', cmap='gray_r', vmin=0,vmax=255,
              extent=(-.5,m.shape[1]-.5,-.5,m.shape[0]-.5))
    for key, color, text in [('navfn','cyan','NavFn'),('wave','red','wave raw'),('smooth','lime','wave smoothed')]:
        p=np.asarray(path[key])
        if len(p):ax.plot(p[:,0],p[:,1],color=color,label=text,linewidth=1.4)
    ax.plot(*row['start'],'bo');ax.plot(*row['goal'],'m*')
    ratio=row['float_length']/row['navfn_length'] if row['float_success'] and row['navfn_success'] and row['navfn_length'] else None
    ax.set_title('frame %s (%s), pair %s; ratio %s' % (row['frame'],row['source'],row['pair'], '%.3f'%ratio if ratio else 'failed'))
    ax.legend();fig.tight_layout();fig.savefig(str(dest),dpi=130);plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--maps',required=True);ap.add_argument('--lib',required=True);ap.add_argument('--out',required=True)
    ap.add_argument('--limit',type=int,default=0)
    a=ap.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    planner=Planner(a.lib)
    ctrl=controls(planner);(out/'controls.json').write_text(json.dumps(ctrl,indent=2)+'\n')
    data=np.load(a.maps);maps,starts,labels=data['maps'],data['starts'],data['labels']
    rows=[];allpaths=[];inventory=[]
    for i,(m,s,l) in enumerate(zip(maps,starts,labels),1):
        if a.limit and i>a.limit:break
        rr,pp=evaluate(planner,m,s,i,str(l));rows.extend(rr);allpaths.extend(pp)
        inventory.append(dict(frame=i, source=str(l), goals=len(rr), start_cost=int(m[s[1],s[0]]),
                              traversable=int(np.count_nonzero(m<253)),
                              edge_goals=sum(r['goal_kind']=='far_edge' for r in rr)))
        if i%10==0:print('evaluated',i,'pairs',len(rows),flush=True)
    (out/'local_milestone.json').write_text(json.dumps(dict(summaries={n:summarize(rr) for n,rr in [('all',rows),('holdout',[r for r in rows if r['frame']>100])]}, inventory=inventory),indent=2)+'\n')
    print('FLOAT MILESTONE: local metrics and controls saved',flush=True)
    large_rows=[];large_paths=[]
    if not a.limit:
        large=synthetic_maps()
        for i,m in enumerate(large):
            rr,pp=evaluate(planner,m,(30,30),1001+i,'synthetic400_'+str(i+1),True)
            large_rows.extend(rr);large_paths.extend(pp)
            print('large map',i+1,'done',flush=True)
        np.savez_compressed(str(out/'large_maps.npz'),maps=np.asarray(large))
    (out/'pairs.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows+large_rows))
    (out/'paths.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in allpaths+large_paths))
    summaries={name:summarize(rr) for name,rr in [
        ('all',rows),('development',[r for r in rows if r['frame']<=100]),
        ('holdout',[r for r in rows if r['frame']>100]),
        ('rec30',[r for r in rows if r['source']=='rec30']),
        ('rec30_holdout',[r for r in rows if r['source']=='rec30' and r['frame']>100]),
        ('exact_goal_holdout',[r for r in rows if r['frame']>100 and r['navfn_exact']]),
        ('safe_navfn_holdout',[r for r in rows if r['frame']>100 and r['navfn_success'] and not r['navfn_hits']]),
        ('large',large_rows)]}
    # Persist all outcomes including failures; never discard unsafe NavFn baselines.
    exceptional=[r for r in rows+large_rows if not r['navfn_success'] and r['float_success']]
    (out/'navfn_fail_wave_success.json').write_text(json.dumps(exceptional,indent=2)+'\n')
    candidates=[r for r in rows if r['frame']>100 and r['navfn_success']]
    def severity(r):
        if not r['float_success']:return (2,0)
        return (1,r['float_length']/r['navfn_length']) if r['navfn_length'] else (0,0)
    worst=sorted(candidates,key=severity,reverse=True)[:5]
    lookup={(p['frame'],p['pair']):p for p in allpaths}
    for i,r in enumerate(worst,1):
        plot_example(maps[r['frame']-1],r,lookup[(r['frame'],r['pair'])],out/('worst_%d.png'%i))
    quality=sorted([r for r in candidates if r['float_success'] and r['navfn_length']>0],key=lambda r:r['float_length']/r['navfn_length'],reverse=True)[:5]
    for i,r in enumerate(quality,1):
        plot_example(maps[r['frame']-1],r,lookup[(r['frame'],r['pair'])],out/('quality_%d.png'%i))
    result=dict(seed=SEED,map_sha256=hashlib.sha256(Path(a.maps).read_bytes()).hexdigest(),
                controls=ctrl,summaries=summaries,worst=worst,quality=quality,frame_inventory=inventory,
                large_per_map={str(i):summarize([r for r in large_rows if r['frame']==1000+i]) for i in range(1,4)})
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(summaries['holdout'],indent=2),flush=True)


if __name__=='__main__':main()
