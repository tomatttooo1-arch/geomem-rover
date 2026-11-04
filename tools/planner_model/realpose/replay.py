"""One command: a ROS2 bag -> fixed-v3/NavFn comparison, preserving extraction and rejections."""
import argparse
from pathlib import Path
import subprocess
import sys
from tools.planner_model.realpose.bag import extract


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ['bag','out','lib','navfn-lib']:ap.add_argument('--'+name,required=True)
    ap.add_argument('--source-kind',required=True,choices=['real','simulation','test_fixture'])
    ap.add_argument('--map-topic',default='/local_costmap/costmap_raw')
    ap.add_argument('--request-topic',default='/dev2/planner_request')
    ap.add_argument('--plan-topic',default='/plan')
    ap.add_argument('--inflation-params')
    a=ap.parse_args();out=Path(a.out)
    if out.exists():ap.error('output must not exist')
    out.mkdir(parents=True)
    import json
    params=json.loads(Path(a.inflation_params).read_text()) if a.inflation_params else None
    extract(a.bag,out/'extraction',a.map_topic,a.request_topic,a.plan_topic,a.source_kind,params)
    subprocess.run([sys.executable,'-m','tools.planner_model.realpose.compare','--input',str(out/'extraction'),
        '--lib',a.lib,'--navfn-lib',a.navfn_lib,'--out',str(out/'comparison')],check=True)


if __name__=='__main__':main()
