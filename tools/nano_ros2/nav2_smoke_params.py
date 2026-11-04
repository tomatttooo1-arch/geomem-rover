#!/usr/bin/env python3
"""Derive a Nav2 smoke-test params file from nav2_bringup's default nav2_params.yaml.

Changes (and only these): no sensors in the smoke test, so both costmaps use
static_layer (the saved RTAB-Map grid) + inflation_layer; obstacle/voxel layers are removed
(the local obstacle layer is N3's job: KV260 rolling map). use_sim_time is forced False.
Usage: nav2_smoke_params.py DEFAULT_YAML OUT_YAML
"""
import sys
import yaml

src, dst = sys.argv[1], sys.argv[2]
p = yaml.safe_load(open(src))


def walk(d):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == 'use_sim_time':
                d[k] = False
            else:
                walk(v)
walk(p)
static = {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local': True}
for name in ('local_costmap', 'global_costmap'):
    c = p[name][name]['ros__parameters']
    for layer in ('voxel_layer', 'obstacle_layer'):
        c.pop(layer, None)
    c['plugins'] = ['static_layer', 'inflation_layer']
    c['static_layer'] = dict(static)
yaml.safe_dump(p, open(dst, 'w'), sort_keys=False)
print('wrote', dst)
