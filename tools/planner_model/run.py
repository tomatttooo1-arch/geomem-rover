"""One-command clean reproduction. Run from repository root with python -m tools.planner_model.run."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trace',required=True)
    ap.add_argument('--out',default='runs/planner-model-20261102')
    a=ap.parse_args();root=Path(__file__).resolve().parents[2]
    trace=Path(a.trace).resolve();out=Path(a.out).resolve()
    if out.exists() and any(out.iterdir()):
        ap.error('--out must be a new or empty directory; preserve previous experiment artifacts')
    out.mkdir(parents=True,exist_ok=True)
    def run(cmd,**kw):subprocess.run(cmd,cwd=str(root),check=True,**kw)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=str(root),text=True).strip()
    (out/'CODE_COMMIT.txt').write_text(commit+'\n')
    run(['cmake','-S','tools/planner_model','-B',str(out/'build'),'-DCMAKE_BUILD_TYPE=Release'])
    run(['cmake','--build',str(out/'build'),'-j2'])
    subprocess.run(['ctest','--output-on-failure'],cwd=str(out/'build'),check=True)
    run([sys.executable,'-m','tools.planner_model.generate','--trace',str(trace),'--out',str(out/'input')])
    run([sys.executable,'-m','tools.planner_model.evaluate','--maps',str(out/'input/maps.npz'),
         '--lib',str(out/'build/libplanner_model.so'),'--out',str(out/'results')])


if __name__=='__main__':main()
