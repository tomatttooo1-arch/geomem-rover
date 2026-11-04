"""Select recorded FPGAF cases without regenerating inputs or expected values."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def read_records(path, magic, size):
    data = path.read_bytes()
    if data[:8] != magic or len(data) < 12:
        raise ValueError('invalid header: %s' % path)
    count, = struct.unpack_from('<I', data, 8)
    if len(data) != 12 + count * size:
        raise ValueError('invalid record count/length: %s' % path)
    return data, [data[12+i*size:12+(i+1)*size] for i in range(count)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('output', type=Path)
    ap.add_argument('--indices', required=True, help='ordered comma-separated zero-based indices')
    args = ap.parse_args()
    indices = [int(i) for i in args.indices.split(',')]
    raw_in, inputs = read_records(args.source/'input.bin', b'FPGAFIN01', 6420)
    raw_go, expected = read_records(args.source/'expected.bin', b'FPGAFGO01', 19276)
    if len(inputs) != len(expected) or not indices or len(set(indices)) != len(indices):
        raise ValueError('mismatched counts or empty/duplicate indices')
    if min(indices) < 0 or max(indices) >= len(inputs):
        raise ValueError('index out of range')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'input.bin').write_bytes(b'FPGAFIN01'+struct.pack('<I', len(indices))+b''.join(inputs[i] for i in indices))
    body = b''.join(expected[i] for i in indices)
    (args.output/'expected.bin').write_bytes(b'FPGAFGO01'+struct.pack('<I', len(indices))+body)
    (args.output/'golden_tb_out').mkdir()
    (args.output/'golden_tb_out'/'outputs.bin').write_bytes(body)
    manifest = dict(source=str(args.source.resolve()), indices=indices,
                    input_sha256=hashlib.sha256(raw_in).hexdigest(),
                    expected_sha256=hashlib.sha256(raw_go).hexdigest(),
                    output_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in [args.output/'input.bin', args.output/'expected.bin']})
    (args.output/'selection.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
