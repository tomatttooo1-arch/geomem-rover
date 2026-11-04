# MEMK4 microprogram: the whole MEMK0 core (hw/memk0/memk0_core.cpp @ d3cf7fe, the C model) written as a readable
# operation table for the shared datapath of uisa.py.  `python3 uprog.py` writes memk4_urom.h (ROM init) and
# memk4_urom.lst (listing).  Every block below names the C block it replaces; semantics are bit-exact.
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uisa import Asm, CMD0, RES0, LN0, OPC

HERE = os.path.dirname(os.path.abspath(__file__))
_src = open(os.path.join(HERE, '..', 'memk0', 'memk0_consts.h')).read()
K = {m.group(1): int(m.group(2)) for m in re.finditer(r'(K_\w+)\s*=\s*(-?\d+)(?:LL|U)?;', _src)}
K['K_DIGEST_LO'] = int(re.search(r'K_DIGEST_LO = (0x[0-9a-fA-F]+)U', _src).group(1), 16)
LN_TAB = [int(x) for x in re.search(r'LN_TAB\[1024\] = \{([^}]*)\}', _src).group(1).split(',')]
CHI2 = int(os.environ.get('MEMK4_CHI2_24_OVR', K['K_CHI2_24']))      # positive control hook (as MEMK0_CHI2_24_OVR)

# ---- block layout (memk0.h)
MAGIC = 0x4D4B; MK_OBS_BASE = 16; MK_OBS_W = 12; MK_CLS_BASE = 400; MK_CLS_W = 6; MK_GEO_BASE = 784
MK_OP_BASE = 792; MK_RES_OBS = 8; MK_RES_PAIRS = 104; MK_RES_DUMP = 616
ERR_MAGIC, ERR_SIZE, ERR_SLOT, ERR_PAIRS, ERR_RANGE, ERR_OP = 1, 2, 4, 8, 16, 32
PF_GATE, PF_FAR, PF_RADIUS, PF_EXCL, PF_MERGE, PF_NEAR, PF_ASSIGN, PF_CAND = 1, 2, 4, 8, 16, 32, 64, 128

# ---- DM map (64-bit words)
SAMP = 0                    # sample (slot s, ring k) at 64 s + 2 k; scratch rows k at SCR + 2 k
SCR = 4096
RING = 0x1040               # slot j entry k at RING + 16 j + k, value (w << 1) | hit
SLOT = 0x1440               # slot j field f at SLOT + 32 j + f
(P0, P1, P2, SF2, FVAL, NS, HEAD, CT, OP_, OT_, L, LVAL, WIN, WVAL, HIT, FRE, MVAL, MPD, RH, RC, RVAL, LH, LHVAL,
 RN, RHD) = range(25)       # CT = cls | type << 7
LOC = 0x1C40
OREASON, ODUP, OELIG, OKEPT, OUSED, ORANK, OASG, ORD = [LOC + 32 * k for k in range(8)]
SUSED, SBLK, SBELOW, SFAV = [LOC + 256 + 64 * k for k in range(4)]
DD, FX, FY, FZ, FW = [LOC + 512 + 32 * k for k in range(5)]
FR = LOC + 672
F_W, F_SEQ, F_OP, F_OT, F_T0, F_T1, F_T2, F_R0, F_R1, F_R2, F_GND, F_POSE, F_DET, F_SKIP = [FR + k for k in range(14)]
NOBSL = LOC + 700
SV = LOC + 704              # spills
(SV_SMODE, SV_KIND, SV_MASK, SV_TAG, SV_NOUT, SV_BEFORE, SV_O, SV_REFSLOT, SV_USEOBS, SV_RS, SV_RO, SV_SO2,
 SV_SLACK, SV_C, SV_MRG, SV_EXC, SV_Q, SV_REF0, SV_REF1, SV_REF2, SV_MED0, SV_MED1, SV_MED2, SV_K1, SV_K2,
 SV_NN, SV_M1, SV_M2, SV_THR2Q, SV_SW, SV_SWD0, SV_SWD1, SV_SWD2, SV_EX, SV_JL, SV_P2, SV_P3, SV_Q0, SV_Q1,
 SV_HIT, SV_GW) = [SV + k for k in range(41)]
assert SV + 41 < 0x2000
M26 = (1 << 26) - 1

# ---- registers
Z, STS, NP, ND, N, MU, NOPS, CMDK, KK, A, B, I, FL, SA = range(14)
t = list(range(14, 24))      # t0..t9: free inside an action routine (clobbered by routines)
LBASE, LV, LTMP, LREM, RT, LM, LL, JK = 24, 25, 26, 27, 28, 29, 30, 31   # leaf regs, links, junk
SM_ASSOC, SM_QUERY, SM_MSCAN, SM_BLOCK = 0, 1, 2, 3


class P(Asm):
    def _w(self, rd):
        assert rd != 0, 'write to r0'

    # ALU
    def add(s, rd, ra, rb): s._w(rd); s.emit('ADD', rd, ra, rb)
    def addi(s, rd, ra, v): s._w(rd); s.emit('ADD', rd, ra, 0, v, 1)
    def sub(s, rd, ra, rb): s._w(rd); s.emit('SUB', rd, ra, rb)
    def subi(s, rd, ra, v): s._w(rd); s.emit('SUB', rd, ra, 0, v, 1)
    def and_(s, rd, ra, rb): s._w(rd); s.emit('AND', rd, ra, rb)
    def andi(s, rd, ra, v): s._w(rd); s.emit('AND', rd, ra, 0, v, 1)
    def or_(s, rd, ra, rb): s._w(rd); s.emit('OR', rd, ra, rb)
    def ori(s, rd, ra, v): s._w(rd); s.emit('OR', rd, ra, 0, v, 1)
    def shl(s, rd, ra, k): s._w(rd); s.emit('SHL', rd, ra, 0, k, 1)
    def shr(s, rd, ra, k): s._w(rd); s.emit('SHR', rd, ra, 0, k, 1)
    def sra(s, rd, ra, k): s._w(rd); s.emit('SRA', rd, ra, 0, k, 1)
    def mul(s, rd, ra, rb): s._w(rd); s.emit('MUL', rd, ra, rb)
    def muli(s, rd, ra, v): s._w(rd); s.emit('MUL', rd, ra, 0, v, 1)
    def min_(s, rd, ra, rb): s._w(rd); s.emit('MIN', rd, ra, rb)
    def mini(s, rd, ra, v): s._w(rd); s.emit('MIN', rd, ra, 0, v, 1)
    def max_(s, rd, ra, rb): s._w(rd); s.emit('MAX', rd, ra, rb)
    def maxi(s, rd, ra, v): s._w(rd); s.emit('MAX', rd, ra, 0, v, 1)
    def slt(s, rd, ra, rb): s._w(rd); s.emit('SLT', rd, ra, rb)
    def slti(s, rd, ra, v): s._w(rd); s.emit('SLT', rd, ra, 0, v, 1)
    def sle(s, rd, ra, rb): s._w(rd); s.emit('SLE', rd, ra, rb)
    def slei(s, rd, ra, v): s._w(rd); s.emit('SLE', rd, ra, 0, v, 1)
    def seq(s, rd, ra, rb): s._w(rd); s.emit('SEQ', rd, ra, rb)
    def seqi(s, rd, ra, v): s._w(rd); s.emit('SEQ', rd, ra, 0, v, 1)
    def sel(s, rd, ra, rb): s._w(rd); s.emit('SEL', rd, ra, rb)          # rd = F ? ra : rb
    def seli(s, rd, ra, v): s._w(rd); s.emit('SEL', rd, ra, 0, v, 1)
    def adc(s, rd, ra, rb): s._w(rd); s.emit('ADC', rd, ra, rb)
    def mov(s, rd, ra): s.addi(rd, ra, 0)
    # memory
    def ld(s, rd, ra, off): s._w(rd); s.emit('LD', rd, ra, 0, off, 1)
    def ldu(s, rd, ra, off): s._w(rd); s.emit('LDU', rd, ra, 0, off, 1)
    def st(s, ra, off, rb): s.emit('ST', 0, ra, rb, off, 1)
    # control
    def beq(s, ra, rb, l): s.emit('BEQ', 0, ra, rb, l)
    def bne(s, ra, rb, l): s.emit('BNE', 0, ra, rb, l)
    def blt(s, ra, rb, l): s.emit('BLT', 0, ra, rb, l)
    def bge(s, ra, rb, l): s.emit('BGE', 0, ra, rb, l)
    def j(s, l): s.emit('JALR', JK, Z, 0, l)
    def call(s, l, lr): s.emit('JALR', lr, Z, 0, l)
    def ret(s, lr): s.emit('JALR', JK, lr, 0, 0)
    def jr(s, ra, off, lr=JK): s.emit('JALR', lr, ra, 0, off)
    # specials
    def mtx(s, ra): s.emit('MTX', 0, ra)
    def mfx(s, rd): s._w(rd); s.emit('MFX', rd)
    def divs(s, rd, rb): s._w(rd); s.emit('DIVS', rd, rd, rb)
    def sqs(s, rd, rb): s._w(rd); s.emit('SQS', rd, rd, rb)
    def rank(s, ra, rb): s.emit('RANK', 0, ra, rb)
    def mfrk(s, rd): s._w(rd); s.emit('MFRK', rd)
    def halt(s): s.emit('HALT')

    # macros
    def li(s, rd, v):
        if -(1 << 26) <= v < (1 << 26):
            s.addi(rd, Z, v)
        else:
            s.li(rd, v >> 26); s.shl(rd, rd, 26)
            if v & M26:
                s.ori(rd, rd, v & M26)

    def sext(s, rd, ra, bits):
        s.shl(rd, ra, 64 - bits); s.sra(rd, rd, 64 - bits)

    def field(s, rd, ra, sh, width):     # unsigned bit field
        if sh:
            s.shr(rd, ra, sh); s.andi(rd, rd, (1 << width) - 1)
        else:
            s.andi(rd, ra, (1 << width) - 1)

    def bnz(s, ra, l): s.bne(ra, Z, l)
    def bz(s, ra, l): s.beq(ra, Z, l)


def build():
    p = P()
    t0, t1, t2, t3, t4, t5, t6, t7, t8, t9 = t

    # ================================================================== top: header checks (memk0_top lines 179-202)
    p.label('ENTRY')
    p.ldu(t0, Z, CMD0 + 0)
    p.ldu(t1, Z, CMD0 + 1)
    p.field(CMDK, t0, 16, 4)
    p.field(N, t1, 0, 8); p.field(MU, t1, 8, 8); p.field(NOPS, t1, 16, 8)
    p.mov(STS, Z); p.mov(NP, Z); p.mov(ND, Z)
    p.field(t2, t0, 0, 16); p.li(t3, MAGIC); p.beq(t2, t3, 'H1')
    p.li(STS, ERR_MAGIC); p.j('FIN')
    p.label('H1')
    p.li(t3, 32); p.blt(t3, N, 'ESIZE')
    p.li(t3, 64); p.blt(t3, MU, 'ESIZE')
    p.li(t3, 58); p.blt(t3, NOPS, 'ESIZE')
    p.li(t3, 3); p.bne(CMDK, t3, 'H2')
    p.ld(t4, Z, NOBSL); p.blt(t4, N, 'ESIZE')
    p.label('H2')
    p.li(t3, 1); p.beq(CMDK, t3, 'H3')
    p.li(t3, 2); p.beq(CMDK, t3, 'H3')
    p.li(t3, 3); p.beq(CMDK, t3, 'H3')
    p.li(STS, ERR_OP); p.j('FIN')
    p.label('ESIZE'); p.li(STS, ERR_SIZE); p.j('FIN')
    # frame (struct Frame)
    p.label('H3')
    p.field(t2, t0, 20, 8)
    p.andi(t3, t2, 1); p.st(Z, F_POSE, t3)
    p.field(t3, t2, 1, 1); p.st(Z, F_DET, t3)
    p.field(t3, t2, 2, 1); p.st(Z, F_SKIP, t3)
    p.ld(t3, Z, CMD0 + 2); p.st(Z, F_W, t3)
    p.ldu(t3, Z, CMD0 + 3); p.andi(t3, t3, 0xFFFFFF); p.st(Z, F_SEQ, t3)
    p.ldu(t3, Z, CMD0 + 4); p.st(Z, F_OP, t3)
    p.ldu(t3, Z, CMD0 + 5); p.st(Z, F_OT, t3)
    for src, dst in ((6, F_GND), (8, F_T0), (9, F_T1), (10, F_T2), (11, F_R0), (12, F_R1), (13, F_R2)):
        p.ld(t3, Z, CMD0 + src); p.sext(t3, t3, 26); p.st(Z, dst, t3)
    # INIT: clear the slot tables (S_POS, S_META, S_EX = 0), N_OBS_LAST = 0, and (MEMK4) the result block
    p.li(t3, 1); p.bne(CMDK, t3, 'H4')
    p.li(t4, SLOT); p.li(t5, SLOT + 32 * 64)
    p.label('INIT1')
    for k in range(8):
        p.st(t4, k, Z)
    p.addi(t4, t4, 8); p.blt(t4, t5, 'INIT1')
    p.st(Z, NOBSL, Z)
    p.li(t4, RES0 + 4); p.li(t5, RES0 + 1024)
    p.label('INIT2')
    for k in range(4):
        p.st(t4, k, Z)
    p.addi(t4, t4, 4); p.blt(t4, t5, 'INIT2')
    p.j('FIN')
    # SL0: per-call slot flags (only j < mu is ever read)
    p.label('H4')
    p.call('CLRSL', LL)

    # ================================================================== op loop (OPS)
    p.mov(KK, Z)
    p.label('OPL')
    p.li(t3, 2); p.bne(CMDK, t3, 'OPX')
    # ASSOC call: one internal op 16 with p0 = f = 0
    p.bne(KK, Z, 'OPEND')
    p.li(t0, 16); p.mov(A, Z); p.mov(B, Z); p.mov(I, Z); p.mov(FL, Z)
    p.j('OPD')
    p.label('OPX')
    p.bge(KK, NOPS, 'OPEND')
    p.shl(t1, KK, 2)
    p.ldu(t2, t1, CMD0 + MK_OP_BASE)
    p.ldu(FL, t1, CMD0 + MK_OP_BASE + 1)
    p.ldu(t3, t1, CMD0 + MK_OP_BASE + 2); p.st(Z, SV_P2, t3)
    p.ldu(t3, t1, CMD0 + MK_OP_BASE + 3); p.st(Z, SV_P3, t3)
    p.field(t0, t2, 0, 8); p.field(A, t2, 8, 8); p.field(B, t2, 16, 8); p.field(I, t2, 24, 8)
    p.label('OPD')
    p.bz(t0, 'OPEND')                               # END
    p.li(t3, 15); p.beq(t0, t3, 'OPEND')            # STOP
    p.li(t3, 16); p.blt(t3, t0, 'EOP')              # no action bits -> ERR_OP
    p.shl(SA, A, 5)
    p.jr(t0, 'JT')
    p.label('JT')
    for h in ['EOP', 'H_TOUCH', 'H_CAND', 'H_NEW', 'H_COPYS', 'H_CLRS', 'H_MERGE', 'H_COPYP', 'H_RESETL', 'H_FREE',
              'H_RESTORE', 'H_QUERY', 'H_MSCAN', 'H_EXIST', 'H_DUMP', 'EOP', 'H_ASSOC']:
        p.j(h)
    p.label('OPNEXT')
    p.addi(KK, KK, 1); p.j('OPL')
    p.label('EOP'); p.ori(STS, STS, ERR_OP); p.j('OPEND')
    p.label('ESLOT'); p.ori(STS, STS, ERR_SLOT); p.j('OPEND')

    def check(needobs, bfree, kind2_exempt=False):
        # a >= 64 || (b >= 64 && !BFREE) || i >= 32 || (needobs && i >= N_OBS_LAST)
        p.li(t3, 63); p.blt(t3, A, 'ESLOT')
        if not bfree:
            p.blt(t3, B, 'ESLOT')
        p.li(t3, 31); p.blt(t3, I, 'ESLOT')
        if needobs:
            skip = p.new('ck')
            if kind2_exempt:
                p.andi(t3, FL, 3); p.li(t4, 2); p.beq(t3, t4, skip)
            p.ld(t4, Z, NOBSL); p.bge(I, t4, 'ESLOT')
            p.label(skip)

    def flag1(l):            # branch to l if f bit0 == 0
        p.andi(t3, FL, 1); p.bz(t3, l)

    p.label('H_TOUCH'); check(1, 0)
    sk = p.new('t'); flag1(sk); p.call('ADD_OBS', LM); p.label(sk)
    p.call('FUSE', LM); p.call('ODO', LM); p.call('EXHIT', LM); p.j('OPNEXT')
    p.label('H_CAND'); check(1, 0)
    p.call('ADD_OBS', LM); p.call('FUSE', LM); p.call('ODO', LM); p.j('OPNEXT')
    p.label('H_NEW'); check(0, 1); p.call('CLEAR', LM); p.j('OPNEXT')
    p.label('H_COPYS'); check(0, 0); p.call('COPY_S', LM); p.j('OPNEXT')
    p.label('H_CLRS'); check(0, 0)
    p.st(SA, SLOT + NS, Z); p.st(SA, SLOT + HEAD, Z); p.j('OPNEXT')
    p.label('H_MERGE'); check(0, 0)
    p.call('MERGE_S', LM)
    sk = p.new('m'); flag1(sk); p.call('COPY_EX', LM); p.label(sk)
    p.call('FUSE', LM); p.j('OPNEXT')
    p.label('H_COPYP'); check(0, 0)
    p.call('COPY_S', LM); p.call('COPY_EX', LM); p.call('COPY_POS', LM); p.j('OPNEXT')
    p.label('H_RESETL'); check(0, 0); p.st(SA, SLOT + LVAL, Z); p.j('OPNEXT')
    p.label('H_FREE'); check(0, 0)
    p.ld(t3, SA, SLOT + CT); p.andi(t3, t3, 127); p.st(SA, SLOT + CT, t3); p.j('OPNEXT')
    p.label('H_RESTORE'); check(0, 1)
    p.addi(t3, KK, 1); p.bge(t3, NOPS, 'EOP')
    p.shl(t1, t3, 2)
    p.ldu(t3, t1, CMD0 + MK_OP_BASE); p.st(Z, SV_Q0, t3)
    p.ldu(t3, t1, CMD0 + MK_OP_BASE + 1); p.st(Z, SV_Q1, t3)
    p.addi(KK, KK, 1)
    p.call('CLEAR', LM); p.call('ADD_RESTORE', LM); p.call('FUSE', LM); p.call('ODO', LM)
    sk = p.new('r'); p.field(t3, FL, 8, 1); p.bz(t3, sk)
    p.ld(t3, Z, SV_Q1); p.sext(t3, t3, 27); p.st(SA, SLOT + L, t3); p.li(t3, 1); p.st(SA, SLOT + LVAL, t3)
    p.label(sk); p.j('OPNEXT')
    p.label('H_QUERY'); check(1, 0, kind2_exempt=True)
    p.li(t0, SM_QUERY); p.call('SCAN', LM); p.j('OPNEXT')
    p.label('H_MSCAN'); check(0, 0)
    p.li(t0, SM_MSCAN); p.call('SCAN', LM)
    # stop flag: records found -> end the call
    p.andi(t3, FL, 1); p.bz(t3, 'OPNEXT')
    p.ld(t4, Z, SV_BEFORE); p.bge(t4, NP, 'OPNEXT')
    p.ori(STS, STS, 256); p.j('OPEND')
    p.label('H_EXIST'); check(0, 0)
    p.li(t0, SM_BLOCK); p.call('SCAN', LM); p.call('EXROLL', LM); p.j('OPNEXT')
    p.label('H_DUMP'); check(0, 0); p.call('DUMP', LM); p.j('OPNEXT')
    p.label('H_ASSOC'); check(0, 0)
    p.call('PREP', LM); p.li(t0, SM_ASSOC); p.call('SCAN', LM); p.call('GREEDY', LM); p.j('OPNEXT')

    p.label('OPEND')
    p.label('FIN')
    p.st(Z, RES0 + 0, STS); p.st(Z, RES0 + 1, NP)
    p.li(t3, 3); p.seq(t3, CMDK, t3); p.sel(t4, ND, Z); p.st(Z, RES0 + 2, t4)
    p.li(t3, K['K_DIGEST_LO']); p.st(Z, RES0 + 3, t3)
    p.halt()

    # ================================================================== small routines
    p.label('CLRSL')                                  # SUSED, SBLK, SBELOW, SFAV = 0 for j < mu
    p.mov(t3, Z)
    p.label('CLRSL1'); p.bge(t3, MU, 'CLRSL2')
    for arr in (SUSED, SBLK, SBELOW, SFAV):
        p.st(t3, arr, Z)
    p.addi(t3, t3, 1); p.j('CLRSL1')
    p.label('CLRSL2'); p.ret(LL)

    p.label('CLEAR')                                  # A_CLEAR: zero slot a, cls = b & 0x7F, type = f & 3
    p.addi(t3, SA, 25)
    p.label('CL1'); p.subi(t3, t3, 1); p.st(t3, SLOT, Z); p.bne(t3, SA, 'CL1')
    p.andi(t3, B, 127); p.andi(t4, FL, 3); p.shl(t4, t4, 7); p.or_(t3, t3, t4); p.st(SA, SLOT + CT, t3)
    p.ret(LM)

    p.label('ODO')
    p.ld(t3, Z, F_OP); p.st(SA, SLOT + OP_, t3); p.ld(t3, Z, F_OT); p.st(SA, SLOT + OT_, t3); p.ret(LM)

    p.label('COPY_POS')
    p.shl(t1, B, 5)
    for f in (SF2, FVAL, P0, P1, P2):
        p.ld(t3, t1, SLOT + f); p.st(SA, SLOT + f, t3)
    p.ret(LM)

    p.label('COPY_EX')                                # S_EX[a] = S_EX[b], ring b -> a, odometer stamps
    p.shl(t1, B, 5)
    for f in (L, LVAL, WIN, WVAL, HIT, FRE, MVAL, MPD, RH, RC, RVAL, LH, LHVAL, RN, RHD, OP_, OT_):
        p.ld(t3, t1, SLOT + f); p.st(SA, SLOT + f, t3)
    p.shl(t1, B, 4); p.shl(t2, A, 4); p.addi(t5, t1, 16)
    p.label('CE1')
    for k in range(4):
        p.ld(t3, t1, RING + k); p.st(t2, RING + k, t3)
    p.addi(t1, t1, 4); p.addi(t2, t2, 4); p.blt(t1, t5, 'CE1')
    p.ret(LM)

    p.label('COPY_S')                                 # ring copy b -> a (all 32 entries), ns, head
    p.shl(t1, B, 6); p.shl(t2, A, 6); p.addi(t5, t1, 64)
    p.label('CS1')
    for k in range(4):
        p.ld(t3, t1, SAMP + k); p.st(t2, SAMP + k, t3)
    p.addi(t1, t1, 4); p.addi(t2, t2, 4); p.blt(t1, t5, 'CS1')
    p.shl(t1, B, 5)
    p.ld(t3, t1, SLOT + NS); p.st(SA, SLOT + NS, t3); p.ld(t3, t1, SLOT + HEAD); p.st(SA, SLOT + HEAD, t3)
    p.ret(LM)

    # sample pack: w0 = p0 | p1 << 26 | (w & 0xFFF) << 52 ; w1 = p2 | key << 26 | (w >> 12) << 50
    def pack_store(base_reg, x0, x1, x2, w, key, ta, tb):
        p.andi(ta, x0, M26); p.andi(tb, x1, M26); p.shl(tb, tb, 26); p.or_(ta, ta, tb)
        p.andi(tb, w, 0xFFF); p.shl(tb, tb, 52); p.or_(ta, ta, tb); p.st(base_reg, SAMP, ta)
        p.andi(ta, x2, M26); p.shl(tb, key, 26); p.or_(ta, ta, tb)
        p.shr(tb, w, 12); p.shl(tb, tb, 50); p.or_(ta, ta, tb); p.st(base_reg, SAMP + 1, ta)

    def add_sample():       # t0..t2 position, t4 w, t5 key -> ring tail of slot a
        p.ld(t6, SA, SLOT + NS); p.ld(t7, SA, SLOT + HEAD)
        p.add(t8, t6, t7); p.andi(t8, t8, 31); p.shl(t8, t8, 1); p.shl(t9, A, 6); p.add(t8, t8, t9)
        pack_store(t8, t0, t1, t2, t4, t5, t3, t9)
        l1, l2 = p.new('as'), p.new('as')
        p.li(t3, K['K_FUSE_KEEP']); p.bge(t6, t3, l1)
        p.addi(t6, t6, 1); p.st(SA, SLOT + NS, t6); p.j(l2)
        p.label(l1); p.addi(t7, t7, 1); p.andi(t7, t7, 31); p.st(SA, SLOT + HEAD, t7)
        p.label(l2)

    p.label('ADD_OBS')      # s = {seq, obs i position, w = res weight}
    p.muli(t3, I, MK_OBS_W)
    for k, r in enumerate((t0, t1, t2)):
        p.ld(r, t3, CMD0 + MK_OBS_BASE + 2 + k); p.sext(r, r, 26)
    p.muli(t3, I, 3); p.ldu(t4, t3, RES0 + MK_RES_OBS + 2); p.andi(t4, t4, 0xFFFFFF)
    p.ld(t5, Z, F_SEQ)
    add_sample(); p.ret(LM)

    p.label('ADD_RESTORE')  # s = {0, p2, p3, q0, w = 400 << 12}
    p.ld(t0, Z, SV_P2); p.sext(t0, t0, 26); p.ld(t1, Z, SV_P3); p.sext(t1, t1, 26)
    p.ld(t2, Z, SV_Q0); p.sext(t2, t2, 26); p.li(t4, 400 << 12); p.mov(t5, Z)
    add_sample(); p.ret(LM)

    # ---- MERGE_S: sorted(keep + drop, key=t)[-K:], stable
    p.label('MERGE_S')
    p.shl(t9, B, 5)
    p.ld(t0, SA, SLOT + NS); p.ld(t1, t9, SLOT + NS)          # ia, ib
    p.ld(t2, SA, SLOT + HEAD); p.ld(t3, t9, SLOT + HEAD)      # ha, hb
    p.mov(t4, Z)                                              # nn
    p.label('MG1')
    p.li(t5, K['K_FUSE_KEEP']); p.bge(t4, t5, 'MG3')
    p.add(t5, t0, t1); p.bz(t5, 'MG3')
    # sa = slot a entry (ha + ia - 1) & 31, sb = slot b entry (hb + ib - 1) & 31: word addresses t6, t7
    p.add(t6, t2, t0); p.subi(t6, t6, 1); p.andi(t6, t6, 31); p.shl(t6, t6, 1); p.shl(t5, A, 6); p.add(t6, t6, t5)
    p.add(t7, t3, t1); p.subi(t7, t7, 1); p.andi(t7, t7, 31); p.shl(t7, t7, 1); p.shl(t5, B, 6); p.add(t7, t7, t5)
    # takeb = ib > 0 && (ia == 0 || keyb >= keya)
    p.bz(t1, 'MGA')
    p.bz(t0, 'MGB')
    p.ld(t5, t6, SAMP + 1); p.field(t5, t5, 26, 24)
    p.ld(t8, t7, SAMP + 1); p.field(t8, t8, 26, 24)
    p.bge(t8, t5, 'MGB')
    p.label('MGA'); p.subi(t0, t0, 1); p.mov(t7, t6); p.j('MGC')
    p.label('MGB'); p.subi(t1, t1, 1)
    p.label('MGC')                                            # SCR[nn] = chosen sample
    p.shl(t5, t4, 1)
    p.ld(t8, t7, SAMP); p.st(t5, SCR, t8); p.ld(t8, t7, SAMP + 1); p.st(t5, SCR + 1, t8)
    p.addi(t4, t4, 1); p.j('MG1')
    p.label('MG3')                                            # WB: ring a [k] = SCR[nn - 1 - k]
    p.mov(t0, Z); p.shl(t6, A, 6); p.shl(t7, t4, 1)
    p.label('MG4'); p.bge(t0, t4, 'MG5')
    p.subi(t7, t7, 2)
    p.ld(t8, t7, SCR); p.st(t6, SAMP, t8); p.ld(t8, t7, SCR + 1); p.st(t6, SAMP + 1, t8)
    p.addi(t0, t0, 1); p.addi(t6, t6, 2); p.j('MG4')
    p.label('MG5')
    p.st(SA, SLOT + NS, t4); p.st(SA, SLOT + HEAD, Z); p.ret(LM)

    # ================================================================== leaves
    # SQRT: X preloaded with x << 4 (x < 2^58); RT = floor(sqrt(x)). clobbers LREM.
    # 30 steps on x << 4 (the first step sees two zero bits) = the C model's 29 steps on x
    p.label('SQRT')
    p.mov(LREM, Z); p.mov(RT, Z); p.li(LTMP, 2)
    p.label('SQ1')
    for k in range(15):
        p.sqs(LREM, RT); p.adc(RT, RT, RT)
    p.subi(LTMP, LTMP, 1); p.bnz(LTMP, 'SQ1')
    p.ret(LL)
    # DIV: X preloaded with n << 2 (n < 2^62), LV = d; RT = floor(n / d), LREM = remainder
    p.label('DIV')
    p.mov(LREM, Z); p.li(LTMP, 2)
    p.label('DV1')
    for k in range(31):
        p.divs(LREM, LV)
    p.subi(LTMP, LTMP, 1); p.bnz(LTMP, 'DV1')
    p.mfx(RT); p.ret(LL)
    # RANKBLK: entry RANKE - 2 nn; counts over A[0..nn-1] (A at LBASE) against LV; RT = lt | le << 16
    p.label('RANKB')
    for y in range(19, -1, -1):
        p.ld(LTMP, LBASE, y); p.rank(LV, LTMP)
    p.label('RANKE')
    p.mfrk(RT); p.ret(LL)

    def rank_call(nnreg, tmp):
        p.add(tmp, nnreg, nnreg); p.sub(tmp, Z, tmp); p.jr(tmp, 'RANKE', LL)

    # ================================================================== FUSE (A_FUSE)
    p.label('FUSE')
    p.ld(t0, SA, SLOT + NS); p.bnz(t0, 'FU0'); p.ret(LM)
    p.label('FU0')
    p.st(Z, SV_NN, t0)
    # unpack the ring into FX, FY, FZ, FW
    p.ld(t1, SA, SLOT + HEAD); p.shl(t2, A, 6); p.mov(t3, Z)
    p.label('FU1')
    p.add(t4, t1, t3); p.andi(t4, t4, 31); p.shl(t4, t4, 1); p.add(t4, t4, t2)
    p.ld(t5, t4, SAMP); p.ld(t6, t4, SAMP + 1)
    p.sext(t7, t5, 26); p.st(t3, FX, t7)
    p.shl(t7, t5, 12); p.sra(t7, t7, 38); p.st(t3, FY, t7)
    p.sext(t7, t6, 26); p.st(t3, FZ, t7)
    p.shr(t7, t5, 52); p.shr(t8, t6, 50); p.andi(t8, t8, 0xFFF); p.shl(t8, t8, 12); p.or_(t7, t7, t8)
    p.st(t3, FW, t7)
    p.addi(t3, t3, 1); p.blt(t3, t0, 'FU1')
    p.subi(t1, t0, 1); p.shr(t1, t1, 1); p.shr(t2, t0, 1)          # k1, k2 (t1, t2 live through the medians)

    def select_two(arr, v1, v2):
        # v1 / v2 = values x with lt <= k <= le - 1 over arr[0..nn-1]
        l = p.new('sel')
        p.mov(v1, Z); p.mov(v2, Z); p.mov(t3, Z)
        p.label(l)
        p.li(LBASE, arr); p.ld(LV, t3, arr); rank_call(t0, t4)
        p.andi(t5, RT, 63); p.shr(t6, RT, 16)                       # lt, le
        for k, v in ((t1, v1), (t2, v2)):
            nx = p.new('sl')
            p.blt(k, t5, nx); p.bge(k, t6, nx); p.mov(v, LV)
            p.label(nx)
        p.addi(t3, t3, 1); p.blt(t3, t0, l)

    for arr, sv in ((FX, SV_MED0), (FY, SV_MED1), (FZ, SV_MED2)):
        select_two(arr, t7, t8); p.add(t7, t7, t8); p.st(Z, sv, t7)
    # DL: DD[x] = sum (2 p - med)^2
    p.ld(t7, Z, SV_MED0); p.ld(t8, Z, SV_MED1); p.ld(t9, Z, SV_MED2); p.mov(t3, Z)
    p.label('FU2')
    p.mov(t4, Z)
    for arr, m in ((FX, t7), (FY, t8), (FZ, t9)):
        p.ld(t5, t3, arr); p.add(t5, t5, t5); p.sub(t5, t5, m); p.mul(t5, t5, t5); p.add(t4, t4, t5)
    p.st(t3, DD, t4)
    p.addi(t3, t3, 1); p.blt(t3, t0, 'FU2')
    # MAD by rank count; msum = isqrt(4 m1) + isqrt(4 m2)
    select_two(DD, t7, t8)
    p.shl(t7, t7, 6); p.mtx(t7); p.call('SQRT', LL); p.mov(t9, RT)
    p.shl(t8, t8, 6); p.mtx(t8); p.call('SQRT', LL); p.add(t9, t9, RT)
    p.li(t3, K['K_MAD3_16']); p.mul(t9, t9, t3); p.shr(t9, t9, 17)
    p.maxi(t9, t9, K['K_INLIER20']); p.li(t3, 1 << 29); p.min_(t9, t9, t3)
    p.mul(t9, t9, t9); p.shr(t9, t9, 2)                             # thr^2 / 4 (DD << 2 <= thr2)
    # SUM over inliers
    p.ld(t7, Z, SV_MED0); p.ld(t8, Z, SV_MED1); p.mov(t1, Z); p.mov(t3, Z)
    p.st(Z, SV_SWD0, Z); p.st(Z, SV_SWD1, Z); p.st(Z, SV_SWD2, Z)
    p.label('FU3')
    p.ld(t4, t3, DD); p.blt(t9, t4, 'FU4')
    p.ld(t5, t3, FW); p.add(t1, t1, t5)
    for arr, sv, m in ((FX, SV_SWD0, SV_MED0), (FY, SV_SWD1, SV_MED1), (FZ, SV_SWD2, SV_MED2)):
        p.ld(t6, t3, arr); p.add(t6, t6, t6); p.ld(t2, Z, m); p.sub(t6, t6, t2); p.mul(t6, t6, t5)
        p.ld(t2, Z, sv); p.add(t2, t2, t6); p.st(Z, sv, t2)
    p.label('FU4')
    p.addi(t3, t3, 1); p.blt(t3, t0, 'FU3')
    # DV: sf2 = 2^60 / sw (saturate at 2^48 - 1); offsets swd * 32 / sw, floor
    p.mov(LV, t1)
    p.li(t3, 1 << 62); p.mtx(t3); p.call('DIV', LL)
    p.shr(t3, RT, 48); p.seq(t3, t3, Z); p.li(t4, (1 << 48) - 1); p.sel(t3, RT, t4); p.st(SA, SLOT + SF2, t3)
    for sv, m, f in ((SV_SWD0, SV_MED0, P0), (SV_SWD1, SV_MED1, P1), (SV_SWD2, SV_MED2, P2)):
        p.ld(t2, Z, sv); p.slt(t5, t2, Z)                           # neg
        p.sub(t6, Z, t2); p.sel(t2, t6, t2)                         # |swd|
        p.shl(t2, t2, 7); p.mtx(t2); p.call('DIV', LL)               # q = |swd| * 32 / sw
        p.seq(t6, LREM, Z)                                          # exact
        p.sub(t7, Z, RT); p.subi(t8, t7, 1); p.sel(t7, t7, t8)      # neg: exact ? -q : -q - 1
        p.seq(t6, t5, Z); p.sel(t7, RT, t7)                          # !neg -> q
        p.ld(t8, Z, m); p.shl(t8, t8, 5); p.add(t8, t8, t7); p.addi(t8, t8, 32); p.sra(t8, t8, 6)
        p.sext(t8, t8, 26); p.st(SA, SLOT + f, t8)
    p.li(t3, 1); p.st(SA, SLOT + FVAL, t3)
    p.ret(LM)

    # ================================================================== PREP (A_PREP)
    p.label('PREP')
    p.mov(t0, Z)
    p.label('PR1'); p.bge(t0, N, 'PR9')
    p.muli(t1, t0, MK_OBS_W)
    p.ldu(t2, t1, CMD0 + MK_OBS_BASE)                               # o0
    for k in range(3):
        l = p.new('rg')
        p.ld(t3, t1, CMD0 + MK_OBS_BASE + 2 + k)
        p.li(t4, 60 << 18); p.blt(t4, t3, l + 'e')
        p.sub(t4, Z, t4); p.bge(t3, t4, l)
        p.label(l + 'e'); p.ori(STS, STS, ERR_RANGE)
        p.label(l)
    p.andi(t3, t2, 127); p.li(t4, 63); p.bge(t4, t3, 'PR2'); p.ori(STS, STS, ERR_SIZE)
    p.label('PR2')
    # so = min(SIG0 + ((r26^2 >> 16) * SIGK >> 32), 2 m)
    p.field(t4, t2, 9, 1); p.li(t5, K['K_R_DEFAULT20']); p.ld(t6, t1, CMD0 + MK_OBS_BASE + 5)
    p.seq(t4, t4, Z); p.sel(t5, t5, t6); p.sext(t5, t5, 26)
    p.mul(t5, t5, t5); p.shr(t5, t5, 16); p.li(t6, K['K_SIGK_32']); p.mul(t5, t5, t6); p.shr(t5, t5, 32)
    p.addi(t5, t5, K['K_SIG0_24']); p.li(t6, 2 << 24); p.min_(t5, t5, t6)
    p.muli(t6, t0, 3); p.st(t6, RES0 + MK_RES_OBS + 1, t5)
    p.mul(LV, t5, t5); p.li(t7, 1 << 62); p.mtx(t7); p.call('DIV', LL)
    p.st(t6, RES0 + MK_RES_OBS + 2, RT)
    # admission reason
    p.andi(t3, t2, 63); p.muli(t3, t3, MK_CLS_W)
    p.ld(t4, t1, CMD0 + MK_OBS_BASE + 4); p.ld(t5, Z, F_GND); p.subi(t5, t5, K['K_BELOW_TOL18'])
    p.li(t7, 1); p.blt(t4, t5, 'PR3')
    p.mov(t7, Z)
    p.field(t4, t2, 10, 1); p.bz(t4, 'PR3')
    p.ldu(t4, t3, CMD0 + MK_CLS_BASE + 5); p.andi(t4, t4, 1); p.bz(t4, 'PR3')
    p.ldu(t4, t1, CMD0 + MK_OBS_BASE + 6); p.ldu(t5, t3, CMD0 + MK_CLS_BASE + 1); p.blt(t5, t4, 'PR2b')
    p.ldu(t4, t1, CMD0 + MK_OBS_BASE + 7); p.ldu(t5, t3, CMD0 + MK_CLS_BASE + 2); p.bge(t5, t4, 'PR3')
    p.label('PR2b'); p.li(t7, 2)
    p.label('PR3')
    p.st(t0, OREASON, t7)
    for arr in (ODUP, OELIG, OKEPT, OUSED, ORANK):
        p.st(t0, arr, Z)
    p.li(t3, 127); p.st(t0, OASG, t3)
    p.addi(t0, t0, 1); p.j('PR1')
    # RK: dedupe order among admitted: (truncated, -score, i)
    p.label('PR9')
    p.mov(t9, Z); p.mov(t0, Z)                                     # na, x
    p.label('RK1'); p.bge(t0, N, 'RK9')
    p.mov(t8, Z)                                                    # r
    p.ld(t7, t0, OREASON); p.bnz(t7, 'RK5')
    p.muli(t1, t0, MK_OBS_W); p.ldu(t2, t1, CMD0 + MK_OBS_BASE); p.field(t2, t2, 8, 1)    # tx
    p.ldu(t3, t1, CMD0 + MK_OBS_BASE + 1)                           # oxs
    p.mov(t4, Z)                                                    # y
    p.label('RK2'); p.bge(t4, N, 'RK5')
    p.ld(t5, t4, OREASON); p.bnz(t5, 'RK4')
    p.muli(t5, t4, MK_OBS_W); p.ldu(t6, t5, CMD0 + MK_OBS_BASE); p.field(t6, t6, 8, 1)    # ty
    p.blt(t6, t2, 'RK3'); p.blt(t2, t6, 'RK4')
    p.ldu(t6, t5, CMD0 + MK_OBS_BASE + 1)                           # oys
    p.blt(t3, t6, 'RK3'); p.blt(t6, t3, 'RK4')
    p.bge(t4, t0, 'RK4')
    p.label('RK3'); p.addi(t8, t8, 1)
    p.label('RK4'); p.addi(t4, t4, 1); p.j('RK2')
    p.label('RK5')
    p.andi(t8, t8, 31); p.st(t0, ORANK, t8)
    p.bnz(t7, 'RK6'); p.st(t8, ORD, t0); p.addi(t9, t9, 1)
    p.label('RK6'); p.addi(t0, t0, 1); p.j('RK1')
    # DD_: same-frame dedupe in rank order
    p.label('RK9')
    p.st(Z, SV_NN, t9); p.mov(t0, Z)                                # rr
    p.label('DQ1'); p.ld(t9, Z, SV_NN); p.bge(t0, t9, 'DQ9')
    p.st(Z, SV_O, t0)
    p.ld(t1, t0, ORD); p.st(Z, SV_RS, t1)                           # x
    p.muli(t1, t1, MK_OBS_W); p.st(Z, SV_RO, t1)                    # obx (cmd word offset)
    p.mov(t2, Z); p.st(Z, SV_Q, Z)                                  # y, conflict
    p.label('DQ2'); p.bge(t2, N, 'DQ8')
    p.ld(t3, t2, OKEPT); p.bz(t3, 'DQ7')
    p.muli(t3, t2, MK_OBS_W); p.ld(t1, Z, SV_RO)
    p.ldu(t4, t3, CMD0 + MK_OBS_BASE); p.andi(t4, t4, 127)
    p.ldu(t5, t1, CMD0 + MK_OBS_BASE); p.andi(t5, t5, 127); p.bne(t4, t5, 'DQ7')
    p.mov(t9, Z)
    for k in range(3):
        p.ld(t4, t3, CMD0 + MK_OBS_BASE + 2 + k); p.sext(t4, t4, 26)
        p.ld(t5, t1, CMD0 + MK_OBS_BASE + 2 + k); p.sext(t5, t5, 26)
        p.sub(t4, t4, t5); p.mul(t4, t4, t4); p.add(t9, t9, t4)
    p.li(t4, K['K_DEDUPE2_36']); p.blt(t9, t4, 'DQ6')
    # IoU: k = box y (t3 base), i = box x (t1 base)
    def box(base, k, r):
        p.ld(r, base, CMD0 + MK_OBS_BASE + 8 + k); p.sext(r, r, 21)

    def span(lo_k, hi_k, r, ra, rb):        # r = max(0, min(hi_y, hi_x) - max(lo_y, lo_x))
        box(t3, hi_k, ra); box(t1, hi_k, rb); p.min_(r, ra, rb)
        box(t3, lo_k, ra); box(t1, lo_k, rb); p.max_(ra, ra, rb); p.sub(r, r, ra); p.maxi(r, r, 0)
    span(0, 2, t4, t6, t7); span(1, 3, t5, t6, t7); p.mul(t4, t4, t5)            # inter
    box(t3, 2, t5); box(t3, 0, t6); p.sub(t5, t5, t6); p.maxi(t5, t5, 0)
    box(t3, 3, t6); box(t3, 1, t7); p.sub(t6, t6, t7); p.maxi(t6, t6, 0); p.mul(t5, t5, t6)   # aa
    box(t1, 2, t6); box(t1, 0, t7); p.sub(t6, t6, t7); p.maxi(t6, t6, 0)
    box(t1, 3, t7); box(t1, 1, t8); p.sub(t7, t7, t8); p.maxi(t7, t7, 0); p.mul(t6, t6, t7)   # ab
    p.add(t5, t5, t6); p.sub(t5, t5, t4)                             # uni
    p.bge(Z, t5, 'DQ7')
    p.shl(t6, t4, 3); p.shl(t7, t4, 1); p.add(t6, t6, t7)            # 10 inter
    p.shl(t7, t5, 1); p.add(t7, t7, t5)                              # 3 uni
    p.bge(t7, t6, 'DQ7')
    p.label('DQ6'); p.li(t3, 1); p.st(Z, SV_Q, t3)
    p.label('DQ7'); p.addi(t2, t2, 1); p.j('DQ2')
    p.label('DQ8')
    p.ld(t1, Z, SV_RS); p.ld(t3, Z, SV_Q); p.li(t4, 1)
    l = p.new('dq'); p.bz(t3, l); p.st(t1, ODUP, t4); p.j(l + 'x')
    p.label(l); p.st(t1, OKEPT, t4)
    p.label(l + 'x')
    p.ld(t0, Z, SV_O); p.addi(t0, t0, 1); p.j('DQ1')
    # EL: eligibility
    p.label('DQ9')
    p.mov(t0, Z)
    p.label('EL1'); p.bge(t0, N, 'EL9')
    p.muli(t1, t0, MK_OBS_W); p.ldu(t2, t1, CMD0 + MK_OBS_BASE); p.field(t2, t2, 9, 1)
    p.ld(t3, t1, CMD0 + MK_OBS_BASE + 5); p.li(t4, K['K_OBS_MAX_R20']); p.slt(t3, t4, t3); p.and_(t2, t2, t3)  # beyond
    p.ld(t3, t0, OREASON); p.or_(t2, t2, t3); p.ld(t3, t0, ODUP); p.or_(t2, t2, t3)
    p.seq(t2, t2, Z); p.st(t0, OELIG, t2)
    p.addi(t0, t0, 1); p.j('EL1')
    p.label('EL9'); p.ret(LM)

    # ================================================================== SCAN (A_SCAN). t0 = smode on entry.
    p.label('SCAN')
    p.st(Z, SV_SMODE, t0)
    p.andi(t1, FL, 3); p.st(Z, SV_KIND, t1)
    p.field(t2, FL, 2, 2); p.st(Z, SV_MASK, t2)
    p.field(t3, FL, 8, 8); p.st(Z, SV_TAG, t3)
    p.st(Z, SV_BEFORE, NP)
    # nout
    p.mov(t4, N)
    l = p.new('sc'); p.li(t5, SM_QUERY); p.bne(t0, t5, l); p.li(t4, 1)
    p.label(l)
    l = p.new('sc'); p.li(t5, SM_MSCAN); p.bne(t0, t5, l); p.mov(t4, MU)
    p.label(l)
    p.st(Z, SV_NOUT, t4)
    l = p.new('sc'); p.li(t5, SM_BLOCK); p.bne(t0, t5, l); p.call('CLRSL', LL)   # SLC
    p.label(l)
    p.st(Z, SV_O, Z)
    p.label('SO1')
    p.ld(t0, Z, SV_O); p.ld(t4, Z, SV_NOUT); p.bge(t0, t4, 'SO9')
    p.ld(t9, Z, SV_SMODE); p.ld(t1, Z, SV_KIND)
    # refslot = MSCAN || (QUERY && kind != 0); useobs = ASSOC || BLOCK || (QUERY && kind != 2)
    p.li(t5, SM_QUERY); p.seq(t5, t9, t5)                       # isq
    p.seq(t6, t1, Z); p.seqi(t6, t6, 0); p.and_(t6, t6, t5)      # QUERY && kind != 0
    p.li(t7, SM_MSCAN); p.seq(t7, t9, t7); p.or_(t6, t6, t7); p.st(Z, SV_REFSLOT, t6)
    p.seqi(t8, t1, 2); p.seqi(t8, t8, 0); p.and_(t8, t8, t5)    # QUERY && kind != 2
    p.seqi(t2, t9, SM_ASSOC); p.or_(t8, t8, t2); p.seqi(t2, t9, SM_BLOCK); p.or_(t8, t8, t2)
    p.st(Z, SV_USEOBS, t8)
    # rs = MSCAN ? o : a ; ro = QUERY ? i : o
    p.seqi(t2, t7, 0); p.sel(t2, A, t0); p.st(Z, SV_RS, t2)
    p.seqi(t3, t5, 0); p.sel(t3, t0, I); p.st(Z, SV_RO, t3)
    # reference position, class, so2
    p.shl(t4, t2, 5)                                              # rs slot addr
    p.muli(t3, t3, MK_OBS_W)                                       # ro cmd offset
    for k, sv in enumerate((SV_REF0, SV_REF1, SV_REF2)):
        p.ld(t1, t3, CMD0 + MK_OBS_BASE + 2 + k); p.sext(t1, t1, 26); p.ld(t2, t4, SLOT + P0 + k)
        p.seq(LTMP, t6, Z); p.sel(t1, t1, t2); p.st(Z, sv, t1)
    p.ldu(t1, t3, CMD0 + MK_OBS_BASE); p.andi(t1, t1, 127)        # obs class
    p.ld(t2, t4, SLOT + CT); p.andi(t2, t2, 127)                   # slot class
    p.seq(LTMP, t8, Z); p.sel(t1, t2, t1)                          # rcls
    p.ld(t3, Z, SV_RO); p.muli(t3, t3, 3); p.ldu(t2, t3, RES0 + MK_RES_OBS + 1); p.andi(t2, t2, M26)
    p.mul(t2, t2, t2); p.li(t3, (1 << 50) - 1); p.and_(t2, t2, t3); p.st(Z, SV_SO2, t2)
    # MSCAN && type(rs) != 1 -> next reference
    l = p.new('sc')
    p.bz(t7, l); p.ld(t2, t4, SLOT + CT); p.shr(t2, t2, 7); p.li(t3, 1); p.bne(t2, t3, 'SO8')
    p.label(l)
    p.andi(t2, t1, 63); p.muli(t2, t2, MK_CLS_W); p.ld(t2, t2, CMD0 + MK_CLS_BASE); p.st(Z, SV_SLACK, t2)
    p.addi(t3, t2, K['K_ASSOC_MAX_M20']); p.st(Z, SV_C, t3)
    p.addi(t3, t2, K['K_MERGE20']); p.st(Z, SV_MRG, t3)
    p.addi(t3, t2, K['K_EXCL20']); p.st(Z, SV_EXC, t3)
    # ex = refslot ? rs : -1
    p.ld(t2, Z, SV_RS); p.seq(LTMP, t6, Z); p.li(t3, -1); p.sel(t2, t3, t2); p.st(Z, SV_EX, t2)
    # msk = QUERY ? mask : ASSOC ? 3 : 1
    p.seqi(t2, t9, SM_ASSOC); p.li(t3, 3); p.li(t2, 1); p.sel(t2, t3, t2)
    p.ld(t3, Z, SV_MASK); p.seq(LTMP, t5, Z); p.sel(t2, t2, t3)
    # k1 = msk & 1 ? 128 + rcls : -1 ; k2 = msk & 2 ? 256 + rcls : -1
    p.andi(t3, t2, 1); p.seq(t3, t3, Z); p.li(t3, -1); p.addi(LTMP, t1, 128); p.sel(t3, t3, LTMP); p.st(Z, SV_K1, t3)
    p.andi(t3, t2, 2); p.seq(t3, t3, Z); p.li(t3, -1); p.addi(LTMP, t1, 256); p.sel(t3, t3, LTMP); p.st(Z, SV_K2, t3)
    # q = (ASSOC || BLOCK) ? o : (QUERY && kind == 0) ? i : rs
    p.ld(t3, Z, SV_RS); p.ld(t2, Z, SV_KIND); p.seq(t2, t2, Z); p.and_(t2, t2, t5); p.seqi(t2, t2, 0)
    p.sel(t3, t3, I)
    p.seqi(t2, t9, SM_ASSOC); p.seqi(LTMP, t9, SM_BLOCK); p.or_(t2, t2, LTMP); p.seqi(t2, t2, 0); p.sel(t3, t3, t0)
    p.st(Z, SV_Q, t3)
    # inner loop over slots j = jstart .. mu-1 (t0 = j, t1 = j << 5, t2 = k1, t3 = k2)
    p.seqi(t2, t7, 0); p.addi(t1, t0, 1); p.sel(t0, Z, t1)        # jstart
    p.shl(t1, t0, 5); p.ld(t2, Z, SV_K1); p.ld(t3, Z, SV_K2)
    p.label('SJ1')
    p.bge(t0, MU, 'SO8')
    p.ld(t4, t1, SLOT + CT)
    p.beq(t4, t2, 'SJM'); p.beq(t4, t3, 'SJM')
    p.label('SJN')
    p.addi(t0, t0, 1); p.addi(t1, t1, 32); p.j('SJ1')
    p.label('SJM')
    p.ld(t5, Z, SV_EX); p.beq(t0, t5, 'SJN')
    p.st(Z, SV_JL, t4)                                             # ct of slot j
    # d = isqrt(d2 << 4)
    p.mov(t9, Z)
    for k, sv in enumerate((SV_REF0, SV_REF1, SV_REF2)):
        p.ld(t5, Z, sv); p.ld(t6, t1, SLOT + P0 + k); p.sub(t5, t5, t6); p.mul(t5, t5, t5); p.add(t9, t9, t5)
    p.shl(t9, t9, 8); p.mtx(t9); p.call('SQRT', LL)               # RT = d
    p.ld(t5, Z, SV_SMODE); p.li(t6, SM_MSCAN); p.bne(t5, t6, 'SJ2')
    p.ld(t6, Z, SV_MRG); p.blt(t6, RT, 'SJN')
    p.li(t9, PF_MERGE); p.j('SJ5')
    p.label('SJ2')                                                 # t9 = fl
    # drift radius
    p.ld(t5, Z, F_OP); p.ld(t6, t1, SLOT + OP_); p.sub(t5, t5, t6); p.li(t6, 0xFFFFFFFF); p.and_(t5, t5, t6)
    p.li(t7, K['K_DRIFT_M24']); p.mul(t5, t5, t7); p.shr(t5, t5, 26)
    p.ld(t7, Z, F_OT); p.ld(t8, t1, SLOT + OT_); p.sub(t7, t7, t8); p.and_(t7, t7, t6)
    p.li(t8, K['K_DRIFT_RAD24']); p.mul(t7, t7, t8); p.shr(t7, t7, 25); p.add(t5, t5, t7)
    p.addi(t5, t5, K['K_ASSOC_M20']); p.mini(t5, t5, K['K_ASSOC_MAX_M20'])       # rad
    p.mov(t9, Z)
    p.sle(t6, RT, t5); p.li(t7, PF_RADIUS); p.sel(t6, t7, Z); p.or_(t9, t9, t6)
    p.li(t7, K['K_ASSOC_M20']); p.sle(t6, RT, t7); p.li(t7, PF_NEAR); p.sel(t6, t7, Z); p.or_(t9, t9, t6)
    p.ld(t7, Z, SV_MRG); p.sle(t6, RT, t7); p.li(t7, PF_MERGE); p.sel(t6, t7, Z); p.or_(t9, t9, t6)
    p.subi(t5, t5, K['K_ASSOC_M20'])                               # drift
    p.ld(t6, Z, SV_USEOBS); p.bz(t6, 'SJ4')
    # Q = ((min(so2 + (fval ? sf2 : so2), 2^48 - 1) >> 16) * chi2) >> 8
    p.ld(t6, Z, SV_SO2); p.ld(t7, t1, SLOT + FVAL); p.ld(t8, t1, SLOT + SF2)
    p.seq(t7, t7, Z); p.sel(t7, t6, t8); p.add(t6, t6, t7)
    p.li(t7, (1 << 48) - 1); p.min_(t6, t6, t7); p.shr(t6, t6, 16); p.li(t7, CHI2); p.mul(t6, t6, t7); p.shr(t6, t6, 8)
    # gate: d <= C && (e <= assoc || e^2 << 8 <= Q), e = min(d - (slack + drift), 2^22)
    p.ld(t7, Z, SV_SLACK); p.add(t5, t5, t7); p.sub(t5, RT, t5); p.mini(t5, t5, 1 << 22)
    p.ld(t8, Z, SV_C); p.blt(t8, RT, 'SJ3')
    p.li(t8, K['K_ASSOC_M20']); p.bge(t8, t5, 'SJ3g')
    p.mul(t5, t5, t5); p.shl(t5, t5, 8); p.blt(t6, t5, 'SJ3')
    p.label('SJ3g'); p.ori(t9, t9, PF_GATE | PF_EXCL)
    p.label('SJ3')
    # FAR: ns > 0 && !(e2 <= move_far || e2^2 << 8 <= Q), e2 = min(d - slack, 2^22)
    p.sub(t5, RT, t7); p.mini(t5, t5, 1 << 22)
    l = p.new('far')
    p.ld(t8, t1, SLOT + NS); p.bz(t8, l)
    p.li(t8, K['K_MOVE_FAR20']); p.bge(t8, t5, l)
    p.mul(t5, t5, t5); p.shl(t5, t5, 8); p.bge(t6, t5, l)
    p.ori(t9, t9, PF_FAR)
    p.label(l)
    p.ld(t8, Z, SV_EXC); p.blt(t8, RT, 'SJ4'); p.ori(t9, t9, PF_EXCL)
    p.label('SJ4')
    p.ld(t5, Z, SV_JL); p.shr(t5, t5, 7); p.li(t6, 2); p.bne(t5, t6, 'SJ5'); p.ori(t9, t9, PF_CAND)
    p.label('SJ5')
    p.ld(t5, Z, SV_SMODE); p.li(t6, SM_BLOCK); p.bne(t5, t6, 'SJ6')
    p.andi(t5, t9, PF_GATE); p.bz(t5, 'SJN'); p.li(t5, 1); p.st(t0, SBLK, t5); p.j('SJN')
    p.label('SJ6')
    p.li(t5, 256); p.blt(NP, t5, 'SJ7'); p.ori(STS, STS, ERR_PAIRS); p.j('SJN')
    p.label('SJ7')
    p.ld(t5, Z, SV_Q); p.andi(t5, t5, 127)
    p.andi(t6, t0, 127); p.shl(t6, t6, 7); p.or_(t5, t5, t6)
    p.ld(t6, Z, SV_TAG); p.shl(t6, t6, 14); p.or_(t5, t5, t6)
    p.andi(t6, t9, 1023); p.shl(t6, t6, 22); p.or_(t5, t5, t6)
    p.add(t6, NP, NP); p.st(t6, RES0 + MK_RES_PAIRS, t5); p.st(t6, RES0 + MK_RES_PAIRS + 1, RT)
    p.addi(NP, NP, 1); p.j('SJN')
    p.label('SO8')
    p.ld(t0, Z, SV_O); p.addi(t0, t0, 1); p.st(Z, SV_O, t0); p.j('SO1')
    p.label('SO9'); p.ret(LM)

    # ================================================================== GREEDY (A_GREEDY) + per-observation words
    p.label('GREEDY')
    p.label('GR1')
    p.li(t0, -1); p.mov(t1, Z); p.mov(t2, Z)                      # best, bd, k2
    p.label('GR2'); p.bge(t2, NP, 'GR5')
    p.add(t3, t2, t2); p.ldu(t4, t3, RES0 + MK_RES_PAIRS)
    p.field(t5, t4, 22, 10); p.andi(t6, t5, PF_GATE); p.bz(t6, 'GR4')
    p.andi(t6, t5, PF_CAND); p.bnz(t6, 'GR4')
    p.andi(t5, t4, 31)                                             # q & 31
    p.ld(t6, t5, OELIG); p.bz(t6, 'GR4'); p.ld(t6, t5, OUSED); p.bnz(t6, 'GR4')
    p.field(t5, t4, 7, 6); p.ld(t6, t5, SUSED); p.bnz(t6, 'GR4')  # j & 63
    p.ldu(t5, t3, RES0 + MK_RES_PAIRS + 1)
    l = p.new('gr'); p.blt(t0, Z, l); p.bge(t5, t1, 'GR4')
    p.label(l); p.mov(t0, t2); p.mov(t1, t5)
    p.label('GR4'); p.addi(t2, t2, 1); p.j('GR2')
    p.label('GR5'); p.blt(t0, Z, 'GR9')
    p.add(t3, t0, t0); p.ldu(t4, t3, RES0 + MK_RES_PAIRS)
    p.andi(t5, t4, 127); p.field(t6, t4, 7, 7)
    p.li(t7, 1); p.st(t5, OUSED, t7); p.st(t5, OASG, t6); p.st(t6, SUSED, t7)
    p.li(t7, PF_ASSIGN << 22); p.or_(t4, t4, t7); p.st(t3, RES0 + MK_RES_PAIRS, t4)
    p.j('GR1')                       # at most MK_OBS assignments (each needs a fresh eligible observation)
    p.label('GR9')
    p.mov(t0, Z)
    p.label('OR1'); p.bge(t0, N, 'OR9')
    p.muli(t1, t0, MK_OBS_W); p.ldu(t2, t1, CMD0 + MK_OBS_BASE)
    p.andi(t3, t2, 63); p.muli(t3, t3, MK_CLS_W)
    p.ld(t9, t0, OREASON)
    p.ld(t4, t0, ODUP); p.shl(t4, t4, 2); p.or_(t9, t9, t4)
    p.ld(t4, t0, OELIG); p.shl(t4, t4, 3); p.or_(t9, t9, t4)
    p.field(t4, t2, 8, 1); p.seqi(t4, t4, 0); p.ldu(t5, t3, CMD0 + MK_CLS_BASE + 5); p.field(t5, t5, 1, 1)
    p.or_(t4, t4, t5); p.shl(t4, t4, 4); p.or_(t9, t9, t4)                                   # may_create
    p.field(t4, t2, 9, 1); p.ld(t5, t1, CMD0 + MK_OBS_BASE + 5)
    p.li(t6, K['K_CREATE_MAX_R20']); p.slt(t6, t6, t5); p.and_(t6, t6, t4); p.shl(t6, t6, 5); p.or_(t9, t9, t6)
    p.li(t6, K['K_OBS_MAX_R20']); p.slt(t6, t6, t5); p.and_(t6, t6, t4); p.shl(t6, t6, 6); p.or_(t9, t9, t6)
    p.ldu(t4, t1, CMD0 + MK_OBS_BASE + 1); p.ldu(t5, t3, CMD0 + MK_CLS_BASE + 4)
    p.sle(t4, t5, t4); p.shl(t4, t4, 7); p.or_(t9, t9, t4)                                   # score_ok
    p.ld(t4, t0, ORANK); p.shl(t4, t4, 8); p.or_(t9, t9, t4)
    p.ld(t4, t0, OASG); p.shl(t4, t4, 14); p.or_(t9, t9, t4)
    p.muli(t4, t0, 3); p.st(t4, RES0 + MK_RES_OBS, t9)
    p.addi(t0, t0, 1); p.j('OR1')
    p.label('OR9'); p.st(Z, NOBSL, N); p.ret(LM)

    # ================================================================== DUMP
    p.label('DUMP')
    p.mov(t0, Z); p.mov(t1, Z)
    p.label('DU1'); p.bge(t0, MU, 'DU9')
    p.addi(t2, ND, 0)
    for k in range(3):
        p.ld(t3, t1, SLOT + P0 + k); p.st(t2, RES0 + MK_RES_DUMP + k, t3)
    p.ld(t4, t1, SLOT + LVAL); p.ld(t3, t1, SLOT + L); p.seq(t5, t4, Z); p.sel(t3, Z, t3)
    p.st(t2, RES0 + MK_RES_DUMP + 3, t3)
    p.ld(t3, t0, SBELOW); p.shl(t3, t3, 1); p.or_(t4, t4, t3)
    p.ld(t3, t0, SFAV); p.shl(t3, t3, 2); p.or_(t4, t4, t3)
    p.ld(t3, t1, SLOT + NS); p.andi(t3, t3, 63); p.shl(t3, t3, 3); p.or_(t4, t4, t3)
    p.ld(t3, t1, SLOT + FVAL); p.shl(t3, t3, 9); p.or_(t4, t4, t3)
    p.ld(t3, t1, SLOT + CT); p.shr(t3, t3, 7); p.andi(t3, t3, 3); p.shl(t3, t3, 10); p.or_(t4, t4, t3)
    p.st(t2, RES0 + MK_RES_DUMP + 4, t4)
    p.addi(ND, ND, 5); p.addi(t0, t0, 1); p.addi(t1, t1, 32); p.j('DU1')
    p.label('DU9'); p.ret(LM)

    # ================================================================== existence (A_EXHIT / A_EXROLL)
    # one slot step; t0 = j, t1 = j << 5, t2 = geo nibble, SV_HIT = hitmode.  Uses t3..t9, leaf regs.
    p.label('EXONE')
    p.st(Z, SV_JL, LL)                                           # this routine is called with LL
    p.mov(t9, Z)                                                 # below | fa << 1
    p.ld(t3, t1, SLOT + WVAL); p.bz(t3, 'EX1')
    p.ld(t4, Z, F_W); p.ld(t5, t1, SLOT + WIN); p.bge(t5, t4, 'EX8')     # wval && W <= win -> no roll
    p.label('EX1')
    p.bz(t3, 'EX7')
    p.ld(t4, t1, SLOT + HIT); p.bnz(t4, 'EX7')
    p.ld(t4, t1, SLOT + FRE); p.ld(t5, t1, SLOT + MVAL); p.or_(t6, t4, t5); p.bz(t6, 'EX7')
    l = p.new('ex')
    p.ld(t6, t1, SLOT + LVAL); p.bnz(t6, l)
    p.ld(t6, Z, SV_HIT); p.field(t7, t2, 3, 1); p.seq(t6, t6, Z); p.and_(t7, t7, t6)   # sinv (frame mode only)
    p.li(t6, K['K_INVALID24']); p.li(t8, K['K_EX_INIT24']); p.seq(t7, t7, Z); p.sel(t6, t8, t6)
    p.st(t1, SLOT + L, t6); p.li(t6, 1); p.st(t1, SLOT + LVAL, t6)
    p.label(l)
    p.li(t6, K['K_EX_FREE24']); p.seq(t7, t4, Z); p.sel(t6, Z, t6)      # base
    p.mov(t7, t6)                                                        # delta
    l = p.new('ex'); p.bz(t5, l)
    p.ld(t8, t1, SLOT + MPD); p.shr(LTMP, t8, 14)
    p.ld(LV, LTMP, LN0); p.ld(LBASE, LTMP, LN0 + 1); p.sub(LBASE, LBASE, LV)
    p.andi(LTMP, t8, 16383); p.mul(LBASE, LBASE, LTMP); p.sra(LBASE, LBASE, 14); p.add(LV, LV, LBASE)   # mv
    p.add(t7, t7, LV)
    # rel = rval && 5 rh >= 4 rc && mpd >= decisive && lhval && win - lh <= recent
    p.ld(LTMP, t1, SLOT + RVAL); p.bz(LTMP, l)
    p.ld(LTMP, t1, SLOT + RH); p.muli(LTMP, LTMP, 5); p.ld(LBASE, t1, SLOT + RC); p.muli(LBASE, LBASE, 4)
    p.blt(LTMP, LBASE, l)
    p.li(LTMP, K['K_DECISIVE24']); p.blt(t8, LTMP, l)
    p.ld(LTMP, t1, SLOT + LHVAL); p.bz(LTMP, l)
    p.ld(LTMP, t1, SLOT + WIN); p.ld(LBASE, t1, SLOT + LH); p.sub(LTMP, LTMP, LBASE); p.li(LBASE, K['K_RECENT_WIN'])
    p.blt(LBASE, LTMP, l)
    p.mini(LV, LV, K['K_REL_VOTE24']); p.add(t7, t6, LV)
    p.label(l)
    # ring record (win, miss)
    p.call('RINGPUT0', LM)
    # v = L + delta; floor; clamp
    p.ld(t6, t1, SLOT + L); p.add(t8, t6, t7)
    l = p.new('ex'); p.bnz(t4, l)
    p.ld(LTMP, t1, SLOT + MPD); p.li(LBASE, K['K_DECISIVE24']); p.bge(LTMP, LBASE, l)
    p.mini(LTMP, t6, K['K_MISS_FLOOR24']); p.max_(t8, t8, LTMP)
    p.label(l)
    p.maxi(t8, t8, K['K_EX_MIN24']); p.mini(t8, t8, K['K_EX_MAX24']); p.st(t1, SLOT + L, t8)
    l = p.new('ex'); p.li(LTMP, K['K_INVALID24']); p.blt(LTMP, t8, l)
    p.shl(t9, t4, 1); p.ori(t9, t9, 1)
    p.label(l)
    p.label('EX7')
    p.ld(t4, Z, F_W); p.st(t1, SLOT + WIN, t4); p.li(t4, 1); p.st(t1, SLOT + WVAL, t4)
    p.st(t1, SLOT + HIT, Z); p.st(t1, SLOT + FRE, Z); p.st(t1, SLOT + MVAL, Z)
    p.label('EX8')
    p.ld(t3, Z, SV_HIT); p.bz(t3, 'EXF')
    # ---- hit
    p.ld(t3, t1, SLOT + LVAL); p.ld(t4, t1, SLOT + HIT)
    l = p.new('eh'); l2 = p.new('eh')
    p.bnz(t3, l); p.li(t5, K['K_EX_INIT24']); p.st(t1, SLOT + L, t5); p.li(t5, 1); p.st(t1, SLOT + LVAL, t5); p.j(l2)
    p.label(l); p.bnz(t4, l2)
    p.ld(t5, t1, SLOT + L); p.li(t6, K['K_EX_HIT24']); p.add(t5, t5, t6); p.mini(t5, t5, K['K_EX_MAX24'])
    p.st(t1, SLOT + L, t5)
    p.label(l2)
    p.bnz(t4, 'EH9')
    p.call('RINGPUT1', LM)
    # count windows: e.w > win - span over the last rn records
    p.ld(t3, t1, SLOT + RN); p.ld(t4, t1, SLOT + RHD); p.ld(t5, t1, SLOT + WIN); p.subi(t5, t5, K['K_SPAN'])
    p.mov(t6, Z); p.mov(t7, Z); p.mov(t8, Z)                     # k2, cnt, hits
    p.shl(LBASE, t0, 4)
    p.label('EH1'); p.bge(t6, t3, 'EH2')
    p.sub(LTMP, t4, t6); p.subi(LTMP, LTMP, 1); p.andi(LTMP, LTMP, 15); p.add(LTMP, LTMP, LBASE)
    p.ld(LV, LTMP, RING); p.sra(LTMP, LV, 1)
    l = p.new('eh'); p.bge(t5, LTMP, l)
    p.addi(t7, t7, 1); p.andi(LV, LV, 1); p.add(t8, t8, LV)
    p.label(l); p.addi(t6, t6, 1); p.j('EH1')
    p.label('EH2')
    p.slti(LTMP, t7, K['K_MIN_WIN']); p.seqi(LTMP, LTMP, 0); p.st(t1, SLOT + RVAL, LTMP)
    p.st(t1, SLOT + RH, t8); p.st(t1, SLOT + RC, t7)
    p.ld(LTMP, t1, SLOT + WIN); p.st(t1, SLOT + LH, LTMP); p.li(LTMP, 1); p.st(t1, SLOT + LHVAL, LTMP)
    p.label('EH9')
    p.li(t3, 1); p.st(t1, SLOT + HIT, t3)
    p.ld(LL, Z, SV_JL); p.ret(LL)
    # ---- frame (miss) part
    p.label('EXF')
    p.andi(t3, t9, 1); p.st(t0, SBELOW, t3); p.shr(t3, t9, 1); p.st(t0, SFAV, t3)
    p.field(t3, t2, 2, 1); p.bnz(t3, 'EXZ')                     # protected
    p.andi(t3, t2, 3); p.li(t4, 2); p.blt(t3, t4, 'EXZ')         # code < 2
    p.ld(t4, Z, F_POSE); p.bz(t4, 'EXZ')
    p.ld(t4, t0, SBLK); p.bnz(t4, 'EXZ')
    l = p.new('ef'); p.li(t4, 2); p.bne(t3, t4, l); p.li(t4, 1); p.st(t1, SLOT + FRE, t4)
    p.label(l)
    p.ld(t4, Z, F_DET); p.bz(t4, 'EXZ'); p.ld(t4, Z, F_SKIP); p.bnz(t4, 'EXZ')
    p.mov(t5, Z)
    for k in range(3):
        p.ld(t6, t1, SLOT + P0 + k); p.ld(t7, Z, F_T0 + k); p.sub(t6, t6, t7); p.ld(t7, Z, F_R0 + k)
        p.mul(t6, t6, t7); p.add(t5, t5, t6)
    p.sra(t5, t5, 22)                                            # rr
    p.bge(Z, t5, 'EXZ'); p.li(t6, K['K_MISS_MAXR20']); p.blt(t6, t5, 'EXZ')
    # detection curve c(rr)
    l1, l2, l3 = p.new('cv'), p.new('cv'), p.new('cv')
    p.li(t6, K['K_CX0_20']); p.blt(t6, t5, l1); p.li(t7, K['K_CY0_24']); p.j(l3)
    p.label(l1); p.li(t6, K['K_CX2_20']); p.blt(t5, t6, l2); p.li(t7, K['K_CY2_24']); p.j(l3)
    p.label(l2)
    lb = p.new('cv')
    p.li(t6, K['K_CX1_20']); p.blt(t6, t5, lb)
    p.subi(t7, t5, K['K_CX0_20']); p.muli(t7, t7, K['K_CS0_24']); p.sra(t7, t7, 20); p.addi(t7, t7, K['K_CY0_24'])
    p.j(l3)
    p.label(lb)
    p.subi(t7, t5, K['K_CX1_20']); p.muli(t7, t7, K['K_CS1_24']); p.sra(t7, t7, 20); p.addi(t7, t7, K['K_CY1_24'])
    p.label(l3)
    p.ld(t6, t1, SLOT + CT); p.andi(t6, t6, 63); p.muli(t6, t6, MK_CLS_W); p.ldu(t6, t6, CMD0 + MK_CLS_BASE + 3)
    p.li(t8, (1 << 25) - 1); p.and_(t6, t6, t8)
    p.mul(t7, t7, t6); p.sra(t7, t7, 24); p.mini(t7, t7, K['K_PD_CAP24'])       # pd
    p.bge(Z, t7, 'EXZ')
    l = p.new('pd')
    p.ld(t6, t1, SLOT + MVAL); p.bz(t6, l); p.ld(t6, t1, SLOT + MPD); p.blt(t6, t7, l + 'k')
    p.label(l); p.st(t1, SLOT + MPD, t7)
    p.label(l + 'k'); p.li(t6, 1); p.st(t1, SLOT + MVAL, t6)
    p.label('EXZ')
    p.ld(LL, Z, SV_JL); p.ret(LL)

    # ring put: RING[16 j + rhd] = (win << 1) | h; rhd = (rhd + 1) & 15; rn = min(rn + 1, 16).  t0 = j, t1 = j << 5
    for h in (0, 1):
        p.label('RINGPUT%d' % h)
        p.ld(LTMP, t1, SLOT + RHD); p.shl(LV, t0, 4); p.add(LV, LV, LTMP)
        p.ld(LBASE, t1, SLOT + WIN); p.shl(LBASE, LBASE, 1)
        if h:
            p.ori(LBASE, LBASE, 1)
        p.st(LV, RING, LBASE)
        p.addi(LTMP, LTMP, 1); p.andi(LTMP, LTMP, 15); p.st(t1, SLOT + RHD, LTMP)
        p.ld(LTMP, t1, SLOT + RN); p.addi(LTMP, LTMP, 1); p.mini(LTMP, LTMP, 16); p.st(t1, SLOT + RN, LTMP)
        p.ret(LM)

    p.label('EXHIT')
    p.st(Z, SV_GW, LM)
    p.li(t3, 1); p.st(Z, SV_HIT, t3)
    p.mov(t0, A); p.mov(t1, SA); p.mov(t2, Z)
    p.call('EXONE', LL)
    p.ld(LM, Z, SV_GW); p.ret(LM)

    p.label('EXROLL')
    p.st(Z, SV_GW, LM)
    p.st(Z, SV_HIT, Z); p.mov(t0, Z); p.mov(t1, Z)
    p.st(Z, SV_RS, Z)                                            # running geo word
    p.label('ER1'); p.bge(t0, MU, 'ER9')
    l = p.new('er')
    p.andi(t3, t0, 7); p.bnz(t3, l)
    p.shr(t3, t0, 3); p.ldu(t3, t3, CMD0 + MK_GEO_BASE); p.st(Z, SV_RS, t3)
    p.label(l)
    p.ld(t3, Z, SV_RS); p.andi(t2, t3, 15); p.shr(t3, t3, 4); p.st(Z, SV_RS, t3)
    p.ld(t3, t1, SLOT + CT); p.shr(t3, t3, 7); p.li(t4, 1); p.bne(t3, t4, 'ER2')
    p.call('EXONE', LL)
    p.label('ER2'); p.addi(t0, t0, 1); p.addi(t1, t1, 32); p.j('ER1')
    p.label('ER9'); p.ld(LM, Z, SV_GW); p.ret(LM)
    # K_HMIN[rc] <= rh  <=>  5 rh >= 4 rc  whenever rval (rc >= K_MIN_WIN)
    hm = [int(x) for x in re.search(r'K_HMIN\[17\] = \{([^}]*)\}', _src).group(1).split(',')]
    assert all((rh >= hm[rc]) == (5 * rh >= 4 * rc) for rc in range(K['K_MIN_WIN'], 17) for rh in range(17))
    return p


def emit_rom(p, path_h, path_lst):
    words = p.assemble()
    assert len(words) <= 4096, len(words)
    with open(path_h, 'w') as f:
        f.write('// generated by uprog.py -- do not edit. %d microinstructions, 48 bits each.\n' % len(words))
        f.write('#pragma once\n#define MK4_UROM_N %d\n' % len(words))
        f.write('static const ap_uint<48> UROM[%d] = {\n' % len(words))
        for k in range(0, len(words), 6):
            f.write('  ' + ', '.join('0x%012xULL' % w for w in words[k:k + 6]) + ',\n')
        f.write('};\n')
    with open(path_lst, 'w') as f:
        f.write(p.listing() + '\n')
    return words


if __name__ == '__main__':
    p = build()
    w = emit_rom(p, os.path.join(HERE, 'memk4_urom.h'), os.path.join(HERE, 'memk4_urom.lst'))
    with open(os.path.join(HERE, 'memk4_ln.h'), 'w') as f:
        f.write('// generated by uprog.py from memk0_consts.h LN_TAB -- do not edit\n#pragma once\n'
                'static const ap_int<27> LNROM[1024] = {%s};\n' % ','.join(str(x) for x in LN_TAB))
    print('microinstructions:', len(w))
