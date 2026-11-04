#!/usr/bin/env python3
"""Standalone Nav2 costmap params for the producer rig, from the delivered local costmap.

Takes robot_sim/fpga_sim/config/rolling_layer_nav2.yaml's local_costmap ros__parameters
UNCHANGED (dev2 navigation parameters are not edited), checks them with
host/rolling_nav_layer.check_costmap_config (refuses on any problem), and writes them under
the node name Nav2's standalone costmap executable pins (/costmap/costmap) plus a lifecycle
manager block, like robot_sim/tests/rolling_nav2_smoke.py does. Prints the check result.
Usage: costmap_params.py --out params.yaml
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def main(argv=None):
    import yaml
    from host.rolling_nav_layer import check_costmap_config
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--source", default=os.path.join(ROOT, "robot_sim/fpga_sim/config/rolling_layer_nav2.yaml"))
    a = ap.parse_args(argv)
    with open(a.source) as fp:
        cfg = yaml.safe_load(fp)
    local = cfg["local_costmap"]["local_costmap"]["ros__parameters"]
    problems = check_costmap_config(local, follow_path=cfg["controller_server"]["ros__parameters"]["FollowPath"],
                                    planner=cfg["planner_server"]["ros__parameters"]["GridBased"])
    print(json.dumps({"source": a.source, "problems": problems}))
    if problems:
        return 2
    params = {"costmap": {"costmap": {"ros__parameters": dict(json.loads(json.dumps(local)), use_sim_time=False)}},
              "check_lifecycle": {"ros__parameters": {"use_sim_time": False, "autostart": True,
                                                      "bond_timeout": 0.0, "node_names": ["/costmap/costmap"]}}}

    class NoAlias(yaml.SafeDumper):       # rcl's params parser rejects YAML aliases
        def ignore_aliases(self, data):
            return True
    with open(a.out, "x") as fp:
        yaml.dump(params, fp, Dumper=NoAlias)
    return 0


if __name__ == "__main__":
    sys.exit(main())
