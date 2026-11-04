"""Read-only inventory of stored RTAB-Map features, not a matching evaluator."""
import argparse
from collections import Counter
from contextlib import closing
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys


def audit(database, node_ids=(35, 38), query_id=None):
    node_ids = tuple(node_ids)
    if len(set(node_ids)) != len(node_ids):
        raise ValueError('duplicate requested node ids')
    database = Path(database).resolve(strict=True)
    # immutable is safe only for stopped/frozen databases; reject sidecars rather
    # than silently ignoring uncheckpointed records.
    if any(Path(str(database) + suffix).exists() for suffix in ('-wal', '-journal')):
        raise ValueError('requires frozen database without WAL/journal sidecars')
    before = hashlib.sha256(database.read_bytes()).hexdigest()
    with closing(sqlite3.connect(database.as_uri() + '?mode=ro&immutable=1', uri=True)) as db:
        schema = {name: sql for name, sql in db.execute("SELECT name, sql FROM sqlite_master WHERE type='table'")}
        fields = [r[1] for r in db.execute('PRAGMA table_info(Feature)')]
        required = {'node_id', 'word_id', 'descriptor', 'descriptor_size', 'depth_x', 'depth_y', 'depth_z'}
        missing = sorted(required - set(fields))
        if missing:
            raise ValueError('missing Feature fields: ' + ', '.join(missing))
        ids = {r[0] for r in db.execute('SELECT id FROM Node')}
        rows = []
        for node_id in node_ids:
            features = list(db.execute('SELECT word_id, descriptor_size, descriptor, depth_x, depth_y, depth_z FROM Feature WHERE node_id=? ORDER BY rowid', (node_id,)))
            descriptors = Counter()
            float_values = Counter()
            sizes = Counter()
            depths = Counter()
            words = Counter()
            word_anomalies = Counter()
            digest = hashlib.sha256()
            for word, declared, blob, x, y, z in features:
                if word is None:
                    word_anomalies['missing'] += 1
                elif not isinstance(word, int):
                    word_anomalies['non_integer'] += 1
                else:
                    words[word] += 1
                sizes[str(declared)] += 1
                if blob is None:
                    descriptors['missing'] += 1
                elif not isinstance(blob, bytes):
                    descriptors['non_blob'] += 1
                else:
                    digest.update(len(blob).to_bytes(8, 'little'))
                    digest.update(blob)
                    if not blob:
                        descriptors['empty'] += 1
                    elif not isinstance(declared, int) or declared <= 0:
                        descriptors['invalid_declared_size'] += 1
                    elif len(blob) == declared:
                        descriptors['binary_uint8'] += 1
                    elif len(blob) == 4 * declared:
                        descriptors['float32'] += 1
                        values = struct.unpack('=' + str(declared) + 'f', blob)
                        float_values['finite_elements'] += sum(math.isfinite(v) for v in values)
                        float_values['nonfinite_elements'] += sum(not math.isfinite(v) for v in values)
                        float_values['descriptors_with_nonfinite_elements'] += any(not math.isfinite(v) for v in values)
                    else:
                        descriptors['length_mismatch'] += 1
                xyz = (x, y, z)
                if any(v is None for v in xyz):
                    depths['null_xyz'] += 1
                elif any(not isinstance(v, (int, float)) for v in xyz):
                    depths['non_numeric_xyz'] += 1
                elif not all(math.isfinite(v) for v in xyz):
                    depths['nonfinite_xyz'] += 1
                else:
                    depths['finite_xyz'] += 1
                    depths['finite_z_positive' if z > 0 else 'finite_z_zero' if z == 0 else 'finite_z_negative'] += 1
            rows.append({'node_id': node_id, 'node_exists': node_id in ids, 'feature_count': len(features), 'descriptor_classification': dict(descriptors), 'float32_value_inventory_native_endian': dict(float_values), 'declared_descriptor_size_histogram': dict(sizes), 'raw_feature_descriptor_stream_sha256': digest.hexdigest(), 'depth_classification': dict(depths), 'word_id_anomalies': dict(word_anomalies), 'distinct_word_ids': len(words), 'duplicate_word_ids': sum(n > 1 for n in words.values()), 'extra_word_occurrences': sum(n - 1 for n in words.values())})
        version = list(db.execute('SELECT version FROM Admin'))
        query = {'requested_database_node_id': query_id, 'node_exists': query_id in ids if query_id is not None else None, 'feature_count': db.execute('SELECT count(*) FROM Feature WHERE node_id=?', (query_id,)).fetchone()[0] if query_id is not None else None, 'limitation': 'input frame index is not a database node id; no online query signature is inferred or synthesized'}
    after = hashlib.sha256(database.read_bytes()).hexdigest()
    if before != after:
        raise RuntimeError('database changed during audit')
    tool = Path(__file__).resolve()
    head = subprocess.run(['git', '-C', str(tool.parent), 'rev-parse', 'HEAD'], text=True, capture_output=True)
    return {'tool_sha256': hashlib.sha256(tool.read_bytes()).hexdigest(), 'git_head': head.stdout.strip() if head.returncode == 0 else None, 'host_byteorder': sys.byteorder, 'descriptor_format_reference': 'https://github.com/introlab/rtabmap/blob/0.23.7/corelib/src/DBDriverSqlite3.cpp#L3922-L3946', 'database': str(database), 'database_sha256_before': before, 'database_sha256_after': after, 'database_versions': [r[0] for r in version], 'node_ids': sorted(ids), 'feature_schema': schema['Feature'], 'feature_fields': fields, 'nodes': rows, 'query_signature': query, 'limits': ['Uses Feature raw descriptors, never Word vocabulary centers.', 'descriptor_size is columns: exact byte counts classify uint8 or float32 format only, not matching quality.', 'Float32 values are interpreted in host byte order; database producer endianness must match for value interpretation.', 'Stored z is a coordinate component, not necessarily optical-axis camera depth; finite/zero/negative labels are not online acceptance thresholds.', 'SQLite may normalize NaN to NULL; NULL is not assigned an original cause.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('database', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--nodes', nargs='+', type=int, default=[35, 38])
    parser.add_argument('--query-node-id', type=int)
    args = parser.parse_args()
    result = audit(args.database, args.nodes, args.query_node_id)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
