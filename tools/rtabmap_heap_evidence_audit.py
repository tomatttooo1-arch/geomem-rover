"""Audit whether Massif exports contain attribution, not merely heap totals.

Does not infer a trace-to-replay clock alignment or interpret unfreed bytes as
leaks. Input artifacts are read-only; the optional report must be a new file.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re


NODE = re.compile(r'^( *)(n\d+): (\d+) (.*)$')


def audit(text):
    snapshots = []
    current = None
    for lineno, line in enumerate(text.splitlines(), 1):
        if line.startswith('snapshot='):
            current = {'id': int(line.split('=', 1)[1]), 'line': lineno,
                       'nodes': []}
            snapshots.append(current)
        elif current is not None:
            for key, converter in [('time', float), ('mem_heap_B', int),
                                   ('heap_tree', str)]:
                if line.startswith(key + '='):
                    current[key] = converter(line.split('=', 1)[1])
            match = NODE.match(line)
            if match:
                current['nodes'].append({'depth': len(match[1]), 'declared_children': int(match[2][1:]),
                    'bytes': int(match[3]), 'label': match[4], 'line': lineno})
    issues = []
    reports = []
    previous = None
    for row in snapshots:
        missing = [key for key in ('time', 'mem_heap_B', 'heap_tree') if key not in row]
        if missing:
            issues.append({'snapshot': row['id'], 'missing': missing})
            continue
        if previous is not None and row['time'] < previous:
            issues.append({'snapshot': row['id'], 'error': 'time decreases'})
        previous = row['time']
        nodes = row['nodes']
        actual = [n for n in nodes if n['depth'] > 0 and 'below massif' not in n['label']]
        roots = [n for n in nodes if n['depth'] == 0]
        children = [n for n in nodes if n['depth'] == 1]
        child_count_errors = []
        structural_errors = []
        if row['heap_tree'] in ('detailed', 'peak'):
            if len(roots) != 1 or (nodes and nodes[0]['depth'] != 0):
                structural_errors.append('detailed tree must have one leading root')
            elif roots[0]['bytes'] != row['mem_heap_B']:
                structural_errors.append('root bytes differ from heap total')
        for left, right in zip(nodes, nodes[1:]):
            if right['depth'] > left['depth'] + 1:
                structural_errors.append('tree depth skips a parent')
        for index, node in enumerate(nodes):
            direct = 0
            direct_bytes = 0
            for following in nodes[index + 1:]:
                if following['depth'] <= node['depth']:
                    break
                direct += following['depth'] == node['depth'] + 1
                if following['depth'] == node['depth'] + 1:
                    direct_bytes += following['bytes']
            if direct != node['declared_children']:
                child_count_errors.append({'line': node['line'],
                    'declared': node['declared_children'], 'actual': direct})
            if node['declared_children'] and direct_bytes != node['bytes']:
                structural_errors.append({'line': node['line'], 'parent_child_gap_bytes': node['bytes'] - direct_bytes})
        if structural_errors:
            issues.append({'snapshot': row['id'], 'tree_errors': structural_errors})
        if child_count_errors:
            issues.append({'snapshot': row['id'], 'child_count_errors': child_count_errors})
        reports.append({'snapshot': row['id'], 'line': row['line'],
            'time': row['time'], 'heap_bytes': row['mem_heap_B'],
            'tree_kind': row['heap_tree'], 'actual_stack_nodes': len(actual),
            'root_bytes': roots[0]['bytes'] if len(roots) == 1 else None,
            'root_child_bytes': sum(n['bytes'] for n in children),
            'root_child_gap_bytes': (roots[0]['bytes'] - sum(n['bytes'] for n in children)) if len(roots) == 1 else None,
            'child_count_errors': child_count_errors,
            'tree_errors': structural_errors,
            'attributed_root_child_bytes': sum(n['bytes'] for n in children
                if 'below massif' not in n['label'])})
    detailed = [r for r in reports if r['tree_kind'] in ('detailed', 'peak')]
    return {'snapshots': len(snapshots), 'detailed_snapshots': len(detailed),
        'actual_stack_nodes': sum(r['actual_stack_nodes'] for r in reports),
        'detailed_without_actual_stacks': sum(r['actual_stack_nodes'] == 0 for r in detailed),
        'metadata_complete_records': len(reports), 'issues': issues, 'records': reports,
        'limits': ['Heap totals alone do not attribute growth to a stack.',
                   'Relative trace time is not aligned to replay monotonic time.',
                   'Nested node byte counts must not be added across depths.',
                   'An empty tree may be deliberate sparse export, not corruption.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    data = args.input.read_bytes()
    result = {'source': str(args.input.resolve()),
              'sha256': hashlib.sha256(data).hexdigest(), **audit(data.decode())}
    if args.output:
        with args.output.open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({**{k: v for k, v in result.items() if k not in ('records', 'issues')},
                      'issue_count': len(result['issues'])}, indent=2))


if __name__ == '__main__':
    main()
