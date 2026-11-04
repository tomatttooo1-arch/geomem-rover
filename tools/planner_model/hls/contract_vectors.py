"""Add six K-register boundary checks, preserving the frozen 213 primary cases."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('output', type=Path)
    a = ap.parse_args()
    raw_in = (a.source/'input.bin').read_bytes()
    raw_go = (a.source/'expected.bin').read_bytes()
    assert raw_in[:8] == b'FPGAFIN01' and raw_go[:8] == b'FPGAFGO01'
    assert struct.unpack_from('<I', raw_in, 8)[0] == 213
    assert struct.unpack_from('<I', raw_go, 8)[0] == 213
    assert len(raw_in) == 12+213*6420 and len(raw_go) == 12+213*19276
    request_map = raw_in[12+187*6420:12+188*6420]
    expected = raw_go[12+187*19276:12+188*19276]
    assert struct.unpack_from('<5i', request_map) == (10, 10, 65, 65, 64)
    assert struct.unpack_from('<3I', expected) == (0, 2, 1)
    assert expected[-64:] == bytes([1, 0]+[255]*62)
    limits = [0, 1, 2, 63, 64, 65]
    inputs = []
    outputs = []
    for k in limits:
        inputs.append(request_map[:16]+struct.pack('<I', k)+request_map[20:])
        if k in (0, 65):
            record = struct.pack('<3I', 6, 0, 0)+bytes([255])*19200+bytes([255])*64
        elif k == 1:
            record = struct.pack('<3I', 7, 1, 0)+bytes([255])*19200+bytes([1]+[255]*63)
        else:
            record = expected
        assert len(record) == 19276
        outputs.append(record)
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'input.bin').write_bytes(b'FPGAFIN01'+struct.pack('<I', 6)+b''.join(inputs))
    (a.output/'expected.bin').write_bytes(b'FPGAFGO01'+struct.pack('<I', 6)+b''.join(outputs))
    (a.output/'golden_tb_out').mkdir()
    (a.output/'golden_tb_out'/'outputs.bin').write_bytes(b''.join(outputs))
    manifest = dict(source=str(a.source.resolve()), source_index=187, limits=limits,
                    oracle='K0/K65 and K1 from preregistered invalid/unconverged contract; K2/63/64 reuse frozen K64 field after round2 convergence',
                    source_sha256={n:hashlib.sha256((a.source/n).read_bytes()).hexdigest() for n in ['input.bin', 'expected.bin']},
                    output_sha256={n:hashlib.sha256((a.output/n).read_bytes()).hexdigest() for n in ['input.bin', 'expected.bin']})
    (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
