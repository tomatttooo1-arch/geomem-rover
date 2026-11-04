# MEMK4 cycle estimate: microinstructions per call and per frame (calls with the same frame seq summed) from the
# Python reference; cycles = 4 x microinstructions (csynth: every SEQ iteration is 4 cycles at 100 MHz).
import sys, os, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import uprog, uref
from uisa import Machine
p = uprog.build(); rom = p.assemble()
labels = sorted((v, k) for k, v in p.labels.items())
for path in sys.argv[1:]:
    m = Machine(rom, uprog.LN_TAB); per_call = []; per_frame = collections.OrderedDict()
    for hdr, cmd, rb, ra in uref.calls(path):
        s = m.run(cmd); per_call.append(s)
        per_frame[hdr[2]] = per_frame.get(hdr[2], 0) + s
    def q(v, f): v = sorted(v); return v[min(len(v) - 1, int(f * len(v)))]
    fr = list(per_frame.values())
    print('%s: calls %d, frames %d' % (os.path.basename(path), len(per_call), len(fr)))
    for name, v in (('per call', per_call), ('per frame', fr)):
        print('  %-9s microinstr median %6d p99 %6d max %6d  -> at 4 cycles / 100 MHz: median %.1f us, p99 %.1f us, max %.1f us'
              % (name, q(v, .5), q(v, .99), max(v), q(v, .5) * .04, q(v, .99) * .04, max(v) * .04))
