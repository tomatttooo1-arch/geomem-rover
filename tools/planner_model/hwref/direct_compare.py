"""Explicit byte comparisons supplement persisted SHA256 fingerprints."""
import argparse
from pathlib import Path
from tools.planner_model.hwref.verify import Candidate,Golden,dataset,equivalent,dump


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args();out=args.out.resolve()
    c=Candidate(out/'build/libplanner_hwref.so');g=Golden(out/'libgolden_observed.so','golden_plan_v3',True)
    failures=[];cells=0
    for i,row,m,_ in dataset():
        expected=g.plan(m,row['start'],row['goal'])
        unrestricted=c.plan(m,row['start'],row['goal'],0)
        bounded=c.plan(m,row['start'],row['goal'],c.k)
        if not(equivalent(expected,unrestricted) and equivalent(expected,bounded)):failures.append(i)
        cells+=m.size if expected['field_valid'] else 0
    dump(out/'direct_bytes.json',dict(requests=2024,K=c.k,finite_and_mapped_infinity_cells=cells,status_field_path_resolved_equal=not failures,failures=failures))
    assert not failures
    print('2024 direct byte comparisons: golden == half scan == K-bounded; field cells',cells)


if __name__=='__main__':main()
