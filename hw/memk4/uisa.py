# MEMK4 microcode ISA: encoding, two-pass assembler and the bit-exact Python interpreter (the reference that the
# HLS sequencer in memk4_core.cpp must match). Design: runs/nationals-prelim-20261102/MEMK4/ARCH.md.
#
# word (48 bits): op[4:0] | I[5] | rd[10:6] | ra[15:11] | rb[20:16] | imm[47:21] (27-bit signed)
#   B = I ? imm : R[rb].  All arithmetic is 64-bit two's complement.  Compares use the sign / zero of ra - B.
OPS = ['HALT', 'ADD', 'SUB', 'AND', 'OR', 'SHL', 'SHR', 'SRA', 'MUL', 'MIN', 'MAX', 'SLT', 'SLE', 'SEQ', 'SEL',
       'ADC', 'LD', 'LDU', 'ST', 'BEQ', 'BNE', 'BLT', 'BGE', 'JALR', 'MTX', 'MFX', 'DIVS', 'SQS', 'RANK', 'MFRK']
OPC = {n: k for k, n in enumerate(OPS)}
M64 = (1 << 64) - 1
IMM_MIN, IMM_MAX = -(1 << 26), (1 << 26) - 1

# address map (word addresses, 15 bits).  DM is the URAM data memory (64-bit words).
DM_WORDS = 8192
CMD0, RES0, LN0 = 0x2000, 0x2400, 0x2800


def s64(x):
    x &= M64
    return x - (1 << 64) if x >> 63 else x


def sext(x, bits):
    x &= (1 << bits) - 1
    return x - (1 << bits) if x >> (bits - 1) else x


class Asm:
    def __init__(self):
        self.code = []      # [op, I, rd, ra, rb, imm(int|label|(label, off)), comment]
        self.labels = {}
        self.uid = 0

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def new(self, p='L'):
        self.uid += 1
        return '%s_%d' % (p, self.uid)

    def emit(self, op, rd=0, ra=0, rb=0, imm=0, I=0, c=''):
        if op == 'JALR':
            I = 1                      # target = ra + imm through the adder
        for r in (rd, ra, rb):
            assert 0 <= r < 32
        self.code.append([OPC[op], I, rd, ra, rb, imm, c])

    def resolve(self, imm):
        if isinstance(imm, str):
            return self.labels[imm]
        if isinstance(imm, tuple):
            return self.labels[imm[0]] + imm[1]
        return imm

    def assemble(self):
        words = []
        for op, I, rd, ra, rb, imm, c in self.code:
            v = self.resolve(imm)
            assert IMM_MIN <= v <= IMM_MAX, (OPS[op], v, c)
            words.append(op | (I << 5) | (rd << 6) | (ra << 11) | (rb << 16) | ((v & ((1 << 27) - 1)) << 21))
        return words

    def listing(self):
        inv = {}
        for k, v in self.labels.items():
            inv.setdefault(v, []).append(k)
        out = []
        for pc, (op, I, rd, ra, rb, imm, c) in enumerate(self.code):
            for l in inv.get(pc, []):
                out.append('%s:' % l)
            out.append('%5d  %-5s rd=%2d ra=%2d rb=%2d I=%d imm=%s  %s' % (pc, OPS[op], rd, ra, rb, I, imm, c))
        return '\n'.join(out)


class Machine:
    """Bit-exact interpreter. cmd: list of 1024 u32 (read only); res: list of 1024 u32 (persistent);
    dm: list of DM_WORDS signed 64-bit (persistent); ln: list of 1024 signed ints (ROM)."""

    def __init__(self, rom, ln):
        self.rom = rom
        self.ln = ln
        self.dm = [0] * DM_WORDS
        self.res = [0] * 1024
        self.count = 0
        self.opcount = [0] * 32
        self.R = [0] * 32          # register file, X, F and the rank counters persist across calls (as in the RTL)
        self.X = 0; self.F = 0; self.CLT = 0; self.CLE = 0

    def load(self, addr, signed):
        addr &= 0x7FFF
        if addr < DM_WORDS:
            return self.dm[addr]
        if CMD0 <= addr < CMD0 + 1024:
            v = self.cmd[addr - CMD0]
        elif RES0 <= addr < RES0 + 1024:
            v = self.res[addr - RES0]
        elif LN0 <= addr < LN0 + 1024:
            return self.ln[addr - LN0]
        else:
            return 0
        return sext(v, 32) if signed else v

    def run(self, cmd, maxsteps=50_000_000):
        self.cmd = cmd
        R = self.R
        X, F, CLT, CLE = self.X, self.F, self.CLT, self.CLE
        pc = 0
        rom, dm, res = self.rom, self.dm, self.res
        steps = 0
        while True:
            w = rom[pc]
            op = w & 31; I = (w >> 5) & 1; rd = (w >> 6) & 31; ra = (w >> 11) & 31; rb = (w >> 16) & 31
            imm = sext(w >> 21, 27)
            a = R[ra]; rbv = R[rb]
            b = imm if I else rbv
            npc = pc + 1
            steps += 1
            self.opcount[op] += 1
            v = None
            if op == 1: v = s64(a + b)
            elif op == 2: v = s64(a - b)
            elif op == 3: v = s64(a & b)
            elif op == 4: v = s64(a | b)
            elif op == 5: v = s64(a << (imm & 63))
            elif op == 6: v = s64((a & M64) >> (imm & 63))
            elif op == 7: v = a >> (imm & 63)
            elif op == 8:
                assert -(1 << 35) <= a < (1 << 35) and -(1 << 35) <= b < (1 << 35), ('MUL operand > s36', pc, a, b)
                v = s64(a * b)
            elif op in (9, 10, 11, 12, 13):
                d = s64(a - b); lt = d < 0; eq = d == 0
                if op == 9: v = a if lt else b
                elif op == 10: v = b if lt else a
                else:
                    F = int(lt if op == 11 else (lt or eq) if op == 12 else eq)
                    v = F
            elif op == 14: v = a if F else b
            elif op == 15: v = s64(a + b + F)
            elif op in (16, 17): v = self.load(a + imm, op == 16)
            elif op == 18:
                addr = (a + imm) & 0x7FFF
                if addr < DM_WORDS: dm[addr] = rbv
                elif RES0 <= addr < RES0 + 1024: res[addr - RES0] = rbv & 0xFFFFFFFF
                else: raise RuntimeError('store outside DM/RES at pc %d addr %x' % (pc, addr))
            elif op in (19, 20, 21, 22):
                d = s64(a - rbv)
                t = (d == 0, d != 0, d < 0, d >= 0)[op - 19]
                if t: npc = imm & 0x7FF
            elif op == 23:
                v = pc + 1; npc = (a + imm) & 0x7FF
            elif op == 24: X = a & M64
            elif op == 25: v = s64(X)
            elif op == 26:
                ap = s64((a << 1) | (X >> 63)); t = s64(ap - rbv); F = int(t >= 0)
                v = t if F else ap; X = ((X << 1) | F) & M64
            elif op == 27:
                ap = s64((a << 2) | (X >> 62)); bp = s64((rbv << 2) | 1); t = s64(ap - bp); F = int(t >= 0)
                v = t if F else ap; X = (X << 2) & M64
            elif op == 28:
                d = s64(a - b)
                CLT = (CLT + (d > 0)) & 63; CLE = (CLE + (d >= 0)) & 63
            elif op == 29:
                v = CLT | (CLE << 16); CLT = 0; CLE = 0
            elif op == 0:
                break
            else:
                raise RuntimeError('bad op %d at %d' % (op, pc))
            if v is not None:
                assert rd != 0, ('write to r0', pc)
                R[rd] = v
            pc = npc
            if steps > maxsteps:
                raise RuntimeError('runaway')
        self.X, self.F, self.CLT, self.CLE = X, F, CLT, CLE
        self.count += steps
        return steps
