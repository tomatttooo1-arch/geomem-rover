#!/usr/bin/env python3
"""Read-only check of the map export and complete memory required for run 2."""
import argparse
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from host import memory_store as ms


def sha256_argument(value):
    if not isinstance(value, str) or not re.fullmatch(r'[0-9a-f]{64}', value):
        raise argparse.ArgumentTypeError('db-sha256 must be a full lowercase SHA256')
    return value


def check_demo(map_yaml, memory, db_sha256):
    """Read existing materials only; refuse a changed export even when its DB matches."""
    sha256_argument(db_sha256)
    document = ms.read_document(memory)
    current = ms.map_identity(map_yaml=map_yaml)
    if current.get('kind') != 'rtabmap_db':
        raise ms.MemoryStoreError('demo_export_unbound',
                                  'map export must name its database through sha256.txt')
    if current.get('db_sha256') != db_sha256:
        raise ms.MemoryStoreError('demo_db_mismatch',
                                  'map export differs from the configured database SHA256')
    stored = document.get('map_identity')
    if (not isinstance(stored, dict) or stored.get('kind') != 'rtabmap_db'
            or not stored.get('db_sha256')):
        raise ms.MemoryStoreError('demo_memory_unbound',
                                  'memory must be bound to a mapping database')
    ms.check(document, current)
    # identity_match accepts the same DB alone. Run 2 also requires the exact
    # YAML and image that were present when the mapping memory was bound.
    for field in ('map_yaml_sha256', 'map_image_sha256'):
        if not stored.get(field) or stored[field] != current.get(field):
            raise ms.MemoryStoreError('demo_export_changed',
                                      'map export differs from the bound memory: ' + field)
    return {'ok': True, 'db_sha256': db_sha256, 'tracks': len(document['tracks'])}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--memory', required=True)
    parser.add_argument('--db-sha256', required=True, type=sha256_argument)
    args = parser.parse_args(argv)
    try:
        result = check_demo(args.map_yaml, args.memory, args.db_sha256)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print('Demo preflight refused: ' + str(exc), file=sys.stderr)
        return 6
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
