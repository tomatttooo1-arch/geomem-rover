"""Stream a validated heaptrack v3 prefix ending at the first c >= target.

Only events preceding that c are included. EOF is synthetic, so heaptrack's
leaked output means bytes alive at this cutoff, not a leak diagnosis.
Parser semantics: KDE/heaptrack v1.3.0 src/analyze/accumulatedtracedata.cpp.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import re


SOURCE_URL = 'https://raw.githubusercontent.com/KDE/heaptrack/v1.3.0/src/analyze/accumulatedtracedata.cpp'
HEX = re.compile(r'^[0-9a-fA-F]+$')
EVENT = re.compile(rb'^[+-] [0-9a-fA-F]+\n$')


def prefix(lines, output, target_ms):
    version = None
    infos = []
    counts = []
    total = 0
    timestamp = 0
    strings = traces = ips = 0
    debuggee = False
    for number, raw in enumerate(lines, 1):
        # Allocation events dominate large traces. Keep the same validation
        # without decoding/tokenizing the whole line for this fixed grammar.
        if raw[:1] in (b'+', b'-'):
            if version is None or not EVENT.fullmatch(raw):
                raise ValueError(f'malformed allocation event line {number}')
            index = int(raw[2:], 16)
            if index >= len(infos):
                raise ValueError(f'invalid allocation index line {number}')
            delta = 1 if raw[0] == 43 else -1
            counts[index] += delta
            if counts[index] < 0:
                raise ValueError(f'unmatched release line {number}')
            total += delta * infos[index]
            output.write(raw)
            continue
        if not raw.endswith(b'\n'):
            raise ValueError(f'truncated line {number}')
        line = raw.decode('utf-8').rstrip('\n')
        if not line or line.startswith('#'):
            output.write(raw)
            continue
        mode, sep, rest = line.partition(' ')
        if not sep or mode not in ('v', 'X', 'I', 's', 'i', 't', 'a', '+', '-', 'c', 'R', 'S'):
            raise ValueError(f'unsupported/malformed line {number}')
        if mode == 'v':
            if version is not None or rest != '10300 3':
                raise ValueError('only heaptrack 1.3.0 file format 3 is supported')
            version = 3
        elif version is None:
            raise ValueError('missing version before data')
        elif mode == 'X':
            if debuggee:
                raise ValueError('duplicate debuggee')
            debuggee = True
        elif mode == 's':
            size, sep, value = rest.partition(' ')
            if not sep or not HEX.fullmatch(size) or len(value.encode()) != int(size, 16):
                raise ValueError(f'invalid sized string line {number}')
            strings += 1
        elif mode not in ('X', 'S'):
            fields = rest.split()
            if not fields or any(not HEX.fullmatch(x) for x in fields):
                raise ValueError(f'invalid hex line {number}')
            values = [int(x, 16) for x in fields]
            lengths = {'I': 2, 't': 2, 'a': 2, '+': 1, '-': 1, 'c': 1, 'R': 1}
            if mode in lengths and len(values) != lengths[mode]:
                raise ValueError(f'wrong field count line {number}')
            if mode == 'i':
                # Symbol-only frames omit file/line; official reader retains
                # functionIndex even when its optional file/line read fails.
                if len(values) < 2 or (len(values) != 3 and (len(values) - 2) % 3) or values[1] > strings:
                    raise ValueError(f'invalid instruction pointer line {number}')
                if any(values[i] > strings for i in range(2, len(values)) if (i - 2) % 3 != 2):
                    raise ValueError(f'invalid frame string reference line {number}')
                ips += 1
            elif mode == 't':
                if values[0] > ips or values[1] > traces:
                    raise ValueError(f'invalid trace reference line {number}')
                traces += 1
            elif mode == 'a':
                if values[1] > traces:
                    raise ValueError(f'invalid allocation trace line {number}')
                infos.append(values[0]); counts.append(0)
            elif mode in ('+', '-'):
                index = values[0]
                if index >= len(infos):
                    raise ValueError(f'invalid allocation index line {number}')
                delta = 1 if mode == '+' else -1
                counts[index] += delta
                if counts[index] < 0:
                    raise ValueError(f'unmatched release line {number}')
                total += delta * infos[index]
            elif mode == 'c':
                if values[0] < timestamp:
                    raise ValueError('time decreases')
                previous = timestamp
                timestamp = values[0]
                output.write(raw)
                if timestamp >= target_ms:
                    return {'requested_ms': target_ms, 'previous_c_ms': previous,
                        'selected_c_ms': timestamp, 'last_line': number,
                        'alive_bytes_from_events': total,
                        'boundary': 'includes selected c; excludes all following events',
                        'replay_monotonic_zero': None, 'source_parser': SOURCE_URL}
                continue
        output.write(raw)
    raise ValueError('trace ended before requested c boundary')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--milliseconds', type=int, required=True)
    args = parser.parse_args()
    if args.milliseconds < 0:
        raise ValueError('negative cutoff')
    digest = hashlib.sha256()
    with args.input.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    with gzip.open(args.input, 'rb') as source, args.output.open('xb') as target:
        with gzip.GzipFile(fileobj=target, mode='wb', mtime=0, compresslevel=1) as compressed:
            report = prefix(source, compressed, args.milliseconds)
    report.update(source=str(args.input.resolve()), source_sha256=digest.hexdigest(), output=str(args.output.resolve()))
    after = hashlib.sha256()
    with args.input.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            after.update(chunk)
    if after.hexdigest() != digest.hexdigest():
        raise ValueError('source changed during prefix generation')
    report.update(source_sha256_after=after.hexdigest(), validation_scope='selected prefix only; source suffix not parsed')
    with Path(str(args.output) + '.json').open('x') as target:
        json.dump(report, target, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
