# MEMK4 key check 2 (generator correctness): run the microprogram in the Python interpreter (uisa.Machine) over a
# recorded call stream and compare every result block word by word with the C model's recorded block.
#   python3 uref.py RECORD.bin [first [last]]      exit 0 = every call identical
import sys, os, struct, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import uprog
from uisa import Machine

def calls(path):
    with open(path, 'rb') as f:
        while True:
            h = f.read(16)
            if len(h) < 16:
                return
            hdr = struct.unpack('<4I', h)
            blk = f.read(4 * 3072)
            w = struct.unpack('<3072I', blk)
            yield hdr, list(w[:1024]), list(w[1024:2048]), list(w[2048:])

def main():
    path = sys.argv[1]
    first = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    last = int(sys.argv[3]) if len(sys.argv) > 3 else -1
    p = uprog.build()
    m = Machine(p.assemble(), uprog.LN_TAB)
    n = bad = shown = 0; t0 = time.time(); steps = []
    for hdr, cmd, rb, ra in calls(path):
        idx = hdr[1]
        if last >= 0 and idx > last:
            break
        if (cmd[0] >> 16) & 15 == 1 and any(rb[4:]):
            print('WARN call %d: INIT with a non-zero result block (the core clears it)' % idx)
        s = m.run(cmd)
        if idx < first:
            continue
        steps.append(s)
        diff = [w for w in range(1024) if m.res[w] != ra[w]]
        n += 1
        if diff:
            bad += 1
            if shown < 6:
                print('call %d seq %d cmd %d: %d words differ, first %s' % (idx, hdr[2], (cmd[0] >> 16) & 15, len(diff),
                      ' '.join('w%d got %08x want %08x' % (w, m.res[w], ra[w]) for w in diff[:4])))
                shown += 1
    steps.sort()
    print('UREF: %d calls compared, %d differ -> %s  (%.1f s; microinstructions per call median %d max %d)' % (
        n, bad, 'FAIL' if bad else 'PASS', time.time() - t0, steps[len(steps) // 2] if steps else 0, steps[-1] if steps else 0))
    return 1 if bad else 0

if __name__ == '__main__':
    sys.exit(main())
