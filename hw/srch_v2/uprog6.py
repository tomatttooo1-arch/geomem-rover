# SRCHK6 概率搜索核微程序：指令集、两遍汇编器、微程序本体。`python3 uprog6.py` 写 srch6_urom.h（ROM 初值）与 srch6_urom.lst。
# 功能基准：hw/srch/srch_core.cpp（c8 9d53224，-DSRCH_SW 定点 C 模型）。设计：runs/nationals-prelim-20261102/SRCHK6/ARCH.md。
# 指令 48 位（同 MEMK4）：op[4:0] | I[5] | rd[10:6] | ra[15:11] | rb[20:16] | imm[47:21]（27 位有符号）。B = I ? imm : R[rb]。
# 全部 64 位补码；比较看 ra - B 的符号/零。移位量 = B[5:0]（I=1 立即数，I=0 寄存器）。
import os, sys

OPS = ['HALT', 'ADD', 'SUB', 'AND', 'OR', 'SHL', 'SHR', 'SRA', 'MUL', 'MIN', 'MAX', 'SLT', 'SEQ', 'SEL', 'LD', 'ST',
       'BEQ', 'BNE', 'BLT', 'BGE', 'JALR', 'DLD', 'DST', 'MFD', 'MTD', 'MTS', 'MFS', 'SCAN', 'BEL', 'XOR']
OPC = {n: k for k, n in enumerate(OPS)}
IMM_MIN, IMM_MAX = -(1 << 26), (1 << 26) - 1
M26 = (1 << 26) - 1

# LD/ST 字地址空间（18 位）：区 = addr[17:14]
A_LANE, A_GRAW, A_ENT, A_OCC, A_PST, A_SEEN, A_REACH = 0x00000, 0x10000, 0x14000, 0x18000, 0x1C000, 0x20000, 0x2C000
# GRIDM 空闲字（原样字访问）：RSQ 表 10240..11007（同 c8）；本核新增参数镜像、R-A 列历史、EXP 表
RSQ, PRM, RSV, DIL, EXPT = 10240, 11008, 11048, 11264, 11520
RAUX, R2A, R2B, R3A, R3B, R4A, R4B, SNSO = [RSV + k for k in range(8)]
# MTS 目标
(S_GEO1, S_GEO2, S_R2, S_RW2, S_RQ, S_LM, S_QN) = range(7)
S_QT = 7                      # 7..14
S_JOB0, S_JOB1, S_VW, S_MODE, S_PTR, S_PEND, S_LIM, S_ERR, S_CFA = range(15, 24)
S_CF = 24                     # 24..28
S_BMODE, S_BN, S_BBASE, S_BRST, S_STATUS = range(29, 34)
# MFS 源
F_PTR, F_PEND, F_NENT, F_ERR, F_OA, F_OB, F_ACC, F_ACC2 = range(8)

# 与 srch_core.h 一致的常数
OP_INIT_MAP, OP_INIT_UNIV, OP_INIT_FINAL, OP_INIT_VIEWS, OP_RESET, OP_PRIOR0, OP_PRIOR1, OP_PRIOR2, OP_OBSERVE, \
    OP_CELLOF, OP_SELECT, OP_DUMP = 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
SR_MAGIC, SR_VERSION, SR_UMAX, SR_ENTRY_OBS, SR_SLOTS = 0x5352, 2, 8191, 31744, 32768
E_MAGIC, E_UOVER, E_EOVER, E_SIZE = 1, 2, 4, 16
MD_UNIV, MD_VIEW, MD_OBS = 0, 1, 2
BM_MASS, BM_SEL, BM_OBS = 0, 1, 2
(P_HDR, P_RES, P_AUX, P_AUX2, P_GEO1, P_GEO2, P_CNT, P_R2, P_RW2, P_RN2, P_LM, P_RQ, P_QT) = range(13)
P_ORG, P_K, P_I0, P_CM, P_CN, P_CW, P_CC, P_TAB = 20, 21, 22, 23, 24, 25, 26, 37


class Asm:
    def __init__(self):
        self.code, self.labels, self.uid = [], {}, 0

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def new(self, p='L'):
        self.uid += 1
        return '%s_%d' % (p, self.uid)

    def emit(self, op, rd=0, ra=0, rb=0, imm=0, I=0, c=''):
        if op == 'JALR':
            I = 1
        for r in (rd, ra, rb):
            assert 0 <= r < 32
        self.code.append([OPC[op], I, rd, ra, rb, imm, c])

    def resolve(self, imm):
        if isinstance(imm, str):
            return self.labels[imm]
        return imm

    def assemble(self):
        out = []
        for op, I, rd, ra, rb, imm, c in self.code:
            v = self.resolve(imm)
            assert IMM_MIN <= v <= IMM_MAX, (OPS[op], v, c)
            out.append(op | (I << 5) | (rd << 6) | (ra << 11) | (rb << 16) | ((v & ((1 << 27) - 1)) << 21))
        return out

    def listing(self):
        inv = {}
        for k, v in self.labels.items():
            inv.setdefault(v, []).append(k)
        o = []
        for pc, (op, I, rd, ra, rb, imm, c) in enumerate(self.code):
            for l in inv.get(pc, []):
                o.append('%s:' % l)
            o.append('%5d  %-5s rd=%2d ra=%2d rb=%2d I=%d imm=%s' % (pc, OPS[op], rd, ra, rb, I, imm))
        return '\n'.join(o)


Z = 0
OP, ERR, U, NI, NJ, RES, AUX, AUX2, NJOBS = range(1, 10)
T = list(range(10, 30))
LR, JK = 30, 31


class P(Asm):
    def _w(s, rd):
        assert rd != 0

    def add(s, rd, ra, rb): s._w(rd); s.emit('ADD', rd, ra, rb)
    def addi(s, rd, ra, v): s._w(rd); s.emit('ADD', rd, ra, 0, v, 1)
    def sub(s, rd, ra, rb): s._w(rd); s.emit('SUB', rd, ra, rb)
    def subi(s, rd, ra, v): s._w(rd); s.emit('SUB', rd, ra, 0, v, 1)
    def and_(s, rd, ra, rb): s._w(rd); s.emit('AND', rd, ra, rb)
    def andi(s, rd, ra, v): s._w(rd); s.emit('AND', rd, ra, 0, v, 1)
    def or_(s, rd, ra, rb): s._w(rd); s.emit('OR', rd, ra, rb)
    def ori(s, rd, ra, v): s._w(rd); s.emit('OR', rd, ra, 0, v, 1)
    def xori(s, rd, ra, v): s._w(rd); s.emit('XOR', rd, ra, 0, v, 1)
    def shl(s, rd, ra, k): s._w(rd); s.emit('SHL', rd, ra, 0, k, 1)
    def shr(s, rd, ra, k): s._w(rd); s.emit('SHR', rd, ra, 0, k, 1)
    def sra(s, rd, ra, k): s._w(rd); s.emit('SRA', rd, ra, 0, k, 1)
    def shlr(s, rd, ra, rb): s._w(rd); s.emit('SHL', rd, ra, rb)
    def shrr(s, rd, ra, rb): s._w(rd); s.emit('SHR', rd, ra, rb)
    def mul(s, rd, ra, rb): s._w(rd); s.emit('MUL', rd, ra, rb)
    def muli(s, rd, ra, v): s._w(rd); s.emit('MUL', rd, ra, 0, v, 1)
    def min_(s, rd, ra, rb): s._w(rd); s.emit('MIN', rd, ra, rb)
    def max_(s, rd, ra, rb): s._w(rd); s.emit('MAX', rd, ra, rb)
    def slt(s, rd, ra, rb): s._w(rd); s.emit('SLT', rd, ra, rb)
    def slti(s, rd, ra, v): s._w(rd); s.emit('SLT', rd, ra, 0, v, 1)
    def seq(s, rd, ra, rb): s._w(rd); s.emit('SEQ', rd, ra, rb)
    def seqi(s, rd, ra, v): s._w(rd); s.emit('SEQ', rd, ra, 0, v, 1)
    def sel(s, rd, ra, rb): s._w(rd); s.emit('SEL', rd, ra, rb)       # rd = F ? ra : rb
    def seli(s, rd, ra, v): s._w(rd); s.emit('SEL', rd, ra, 0, v, 1)
    def mov(s, rd, ra): s.addi(rd, ra, 0)
    def ld(s, rd, ra, off): s._w(rd); s.emit('LD', rd, ra, 0, off, 1)
    def st(s, ra, off, rb): s.emit('ST', 0, ra, rb, off, 1)
    def beq(s, ra, rb, l): s.emit('BEQ', 0, ra, rb, l)
    def bne(s, ra, rb, l): s.emit('BNE', 0, ra, rb, l)
    def blt(s, ra, rb, l): s.emit('BLT', 0, ra, rb, l)
    def bge(s, ra, rb, l): s.emit('BGE', 0, ra, rb, l)
    def bz(s, ra, l): s.beq(ra, Z, l)
    def bnz(s, ra, l): s.bne(ra, Z, l)
    def j(s, l): s.emit('JALR', JK, Z, 0, l)
    def call(s, l): s.emit('JALR', LR, Z, 0, l)
    def ret(s): s.emit('JALR', JK, LR, 0, 0)
    def dld(s, ra, off=0): s.emit('DLD', 0, ra, 0, off, 1)          # DB = gmem[(ra+off)>>4]
    def dst(s, ra, off=0): s.emit('DST', 0, ra, 0, off, 1)          # gmem[(ra+off)>>4] = DB
    def mfd(s, rd, k): s._w(rd); s.emit('MFD', rd, 0, 0, k, 1)      # rd = DB 第 k 个 64 位
    def mtd(s, k, ra): s.emit('MTD', 0, ra, 0, k, 1)                # DB 第 k 个 64 位 = ra
    def mts(s, tgt, ra): s.emit('MTS', 0, ra, 0, tgt, 1)
    def mfs(s, rd, src): s._w(rd); s.emit('MFS', rd, 0, 0, src, 1)
    def scan(s): s.emit('SCAN')
    def bel(s, rd): s._w(rd); s.emit('BEL', rd)                     # rd = 1：一个视点结束（OA/OB 有效）
    def halt(s): s.emit('HALT')

    def li(s, rd, v):
        if IMM_MIN <= v <= IMM_MAX:
            s.addi(rd, Z, v)
        else:
            s.li(rd, v >> 26); s.shl(rd, rd, 26)
            if v & M26:
                s.ori(rd, rd, v & M26)

    def field(s, rd, ra, sh, w):
        if sh:
            s.shr(rd, ra, sh)
            if sh + w < 64:
                s.andi(rd, rd, (1 << w) - 1) if w <= 26 else s._andw(rd, rd, w)
        else:
            s.andi(rd, ra, (1 << w) - 1) if w <= 26 else s._andw(rd, ra, w)

    def _andw(s, rd, ra, w):          # rd = ra & (2^w - 1)，w > 26：左移再逻辑右移
        s.shl(rd, ra, 64 - w); s.shr(rd, rd, 64 - w)

    def sext(s, rd, ra, bits, sh=0):  # rd = 有符号位段 ra[sh+bits-1:sh]
        s.shl(rd, ra, 64 - bits - sh); s.sra(rd, rd, 64 - bits)

    def prm(s, rd, k): s.ld(rd, Z, A_GRAW + PRM + k)
    def rsv_ld(s, rd, a): s.ld(rd, Z, A_GRAW + a)
    def rsv_st(s, a, rb): s.st(Z, A_GRAW + a, rb)


def build(ovr=None):
    p = P()
    t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19 = T

    # ======================================================== 参数块（19 拍）-> GRIDM 参数镜像
    p.label('ENTRY')
    p.mts(S_STATUS, Z)
    # t0 = 参数块字节地址：硬件在启动时把 prm（低 32 位）放进 DB 低 64 位
    p.mfd(t0, 0)
    p.li(t1, A_GRAW + PRM); p.li(t2, 19)
    p.label('PL')
    p.dld(t0); p.mfd(t3, 0); p.st(t1, 0, t3); p.mfd(t3, 1); p.st(t1, 1, t3)
    p.addi(t0, t0, 16); p.addi(t1, t1, 2); p.subi(t2, t2, 1); p.bnz(t2, 'PL')
    # 结果暂存清零
    for a in (RAUX, R2A, R2B, R3A, R3B, R4A, R4B):
        p.rsv_st(a, Z)
    # 解码
    p.prm(t0, P_HDR)
    p.field(OP, t0, 24, 8)
    p.mov(ERR, Z)
    p.prm(RES, P_RES); p.field(RES, RES, 0, 32)
    p.prm(AUX, P_AUX); p.field(AUX, AUX, 0, 32)
    p.prm(AUX2, P_AUX2); p.field(AUX2, AUX2, 0, 32)
    p.prm(t1, P_GEO1); p.field(NI, t1, 0, 8); p.field(NJ, t1, 16, 8); p.mts(S_GEO1, t1)
    p.prm(t1, P_GEO2); p.mts(S_GEO2, t1)
    p.prm(t1, P_R2); p.mts(S_R2, t1)
    p.prm(t1, P_RW2); p.mts(S_RW2, t1)
    p.prm(t1, P_RQ); p.mts(S_RQ, t1)
    p.prm(t1, P_LM); p.mts(S_LM, t1)
    p.field(t1, t0, 32, 4); p.mts(S_QN, t1)
    for i in range(8):
        p.prm(t1, P_QT + i); p.mts(S_QT + i, t1)
    p.prm(t1, P_CNT)
    p.field(U, t1, 0, 16); p.field(t2, t1, 16, 16); p.rsv_st(SNSO + 1, t2)       # nsv
    p.field(t2, t1, 32, 16); p.rsv_st(SNSO, t2)                                  # nso
    p.field(NJOBS, t1, 48, 16)
    # ok = magic && op in [1,13] && op != 2
    p.field(t1, t0, 0, 16); p.li(t2, SR_MAGIC); p.bne(t1, t2, 'BADH')
    p.bz(OP, 'BADH'); p.li(t2, 13); p.blt(t2, OP, 'BADH'); p.li(t2, 2); p.beq(OP, t2, 'BADH')
    p.j('CHKS')
    p.label('BADH'); p.ori(ERR, ERR, E_MAGIC)
    p.label('CHKS')
    p.li(t2, 8); p.blt(NI, t2, 'BADS'); p.blt(NJ, t2, 'BADS')
    p.li(t2, 198); p.blt(t2, NI, 'BADS'); p.blt(t2, NJ, 'BADS')
    p.li(t2, 640); p.blt(t2, NJOBS, 'BADS')
    p.j('CHK2')
    p.label('BADS'); p.ori(ERR, ERR, E_SIZE)
    p.label('CHK2')
    p.bnz(ERR, 'FIN')

    def ifop(ops, skip):           # 不在 ops 内则跳到 skip
        l = p.new('IO')
        for o in ops:
            p.li(t19, o); p.beq(OP, t19, l)
        p.j(skip)
        p.label(l)

    # ======================================================== INIT_MAP：装载 4 段 + R-A
    ifop([OP_INIT_MAP], 'B2')
    # 段：(源地址寄存器/参数, 拍数, 目标字地址)
    p.mul(t5, NI, NJ)                 # ncell
    p.rsv_st(SNSO + 2, t5)
    # 段 0：OCC <- aux，2048 拍
    p.mov(t0, AUX); p.li(t1, A_OCC); p.li(t2, 2048); p.call('LOAD')
    # 段 1：ENTRY <- aux2，(ncell+15)>>4 拍
    p.mov(t0, AUX2); p.li(t1, A_ENT); p.addi(t2, t5, 15); p.shr(t2, t2, 4); p.call('LOAD')
    # 段 2：RSQ 768 字 <- tab，384 拍
    p.prm(t0, P_TAB); p.field(t0, t0, 0, 32); p.li(t1, A_GRAW + RSQ); p.li(t2, 384); p.call('LOAD')
    # 段 3：EXP 512 字 <- tab + 6144，256 拍
    p.prm(t0, P_TAB); p.field(t0, t0, 0, 32); p.addi(t0, t0, 768 * 8); p.field(t0, t0, 0, 32)
    p.li(t1, A_GRAW + EXPT); p.li(t2, 256); p.call('LOAD')
    p.call('GRIDA')
    p.label('B2')

    # ======================================================== INIT_FINAL：R-B 编号 + 代价图
    ifop([OP_INIT_FINAL], 'B3')
    p.call('GRIDB')
    p.call('COSTMAP')
    p.rsv_st(RAUX, U)
    p.label('B3')

    # ======================================================== 近节点圈 / 最近格
    ifop([OP_INIT_UNIV, OP_INIT_FINAL, OP_CELLOF], 'B4')
    p.call('NBR')
    p.label('B4')

    # ======================================================== 清零
    ifop([OP_RESET, OP_INIT_VIEWS], 'B5')
    p.mov(t0, Z)
    p.bz(U, 'B5')
    p.li(t2, OP_RESET); p.bne(OP, t2, 'CLR1')
    p.li(t3, 1 << 38)
    p.label('CLR0')
    p.shl(t1, t0, 1); p.st(t1, A_PST, Z); p.st(t1, A_PST + 1, t3)
    p.addi(t0, t0, 1); p.blt(t0, U, 'CLR0')
    p.j('B5')
    p.label('CLR1')
    p.st(t0, A_REACH, Z); p.addi(t0, t0, 1); p.blt(t0, U, 'CLR1')
    p.label('B5')

    # ======================================================== 窗口扫描（UNIV / VIEWS / OBSERVE）
    ifop([OP_INIT_UNIV, OP_INIT_VIEWS, OP_OBSERVE], 'B6')
    p.call('ENGINE')
    p.label('B6')

    # ======================================================== 先验
    ifop([OP_PRIOR0, OP_PRIOR1, OP_PRIOR2], 'B7')
    p.call('PRIOR')
    p.label('B7')

    # ======================================================== 信念（OBSERVE / SELECT）
    ifop([OP_OBSERVE, OP_SELECT], 'B8')
    p.call('BELIEF')
    p.label('B8')

    # ======================================================== 导出
    ifop([OP_DUMP], 'FIN')
    p.mov(t0, Z); p.mov(t1, AUX2)
    p.bz(U, 'FIN')
    p.label('DMP')
    p.shl(t2, t0, 1); p.ld(t3, t2, A_PST); p.ld(t4, t2, A_PST + 1)
    p.mtd(0, t3); p.mtd(1, t4); p.dst(t1)
    p.addi(t1, t1, 16); p.addi(t0, t0, 1); p.blt(t0, U, 'DMP')

    # ======================================================== 结果块头（写序 3,2,1,0）
    p.label('FIN')
    p.li(t0, SR_VERSION << 8); p.or_(t0, t0, ERR); p.shl(t1, OP, 16); p.or_(t0, t0, t1)   # st
    p.mts(S_STATUS, t0)
    p.li(t2, OP_PRIOR0); p.beq(OP, t2, 'W3')
    p.li(t2, OP_PRIOR1); p.bne(OP, t2, 'W1')
    p.label('W3')
    p.rsv_ld(t3, R3A); p.mtd(0, t3); p.rsv_ld(t3, R3B); p.mtd(1, t3); p.dst(RES, 48)
    p.li(t2, OP_PRIOR1); p.bne(OP, t2, 'W1')
    p.rsv_ld(t3, R4A); p.mtd(0, t3); p.rsv_ld(t3, R4B); p.mtd(1, t3); p.dst(RES, 32)
    p.label('W1')
    p.li(t2, OP_SELECT); p.bne(OP, t2, 'W0')
    p.rsv_ld(t3, R2A); p.mtd(0, t3); p.rsv_ld(t3, R2B); p.mtd(1, t3); p.dst(RES, 16)
    p.label('W0')
    p.shl(t1, U, 32); p.or_(t1, t1, t0); p.mtd(0, t1); p.rsv_ld(t3, RAUX); p.mtd(1, t3); p.dst(RES, 0)
    p.halt()

    # ======================================================== 子程序 LOAD：t0 字节地址, t1 目标字地址, t2 拍数（≥1）
    p.label('LOAD')
    p.bz(t2, 'LDX')
    p.label('LD1')
    p.dld(t0); p.mfd(t3, 0); p.st(t1, 0, t3); p.mfd(t3, 1); p.st(t1, 1, t3)
    p.addi(t0, t0, 16); p.addi(t1, t1, 2); p.subi(t2, t2, 1); p.bnz(t2, 'LD1')
    p.label('LDX')
    p.ret()

    # ======================================================== R-A：块归并、避让十字、贴边圆盘（单遍 (ni+3)x(nj+3)）
    # win：7 列 x 7 位打包（列 k 在位 7k..7k+6，k=6 最新）；fa：4 x 2 位
    MEDGE = (0x08 << 0) | (0x3E << 7) | (0x3E << 14) | (0x7F << 21) | (0x3E << 28) | (0x3E << 35) | (0x08 << 42)
    MCROSS = (1 << 24) | (1 << 17) | (1 << 31) | (1 << 23) | (1 << 25)
    p.label('GRIDA')
    # t0 i, t1 j, t2 cbase = i*nj, t3 oc, t4 win, t5 fa, t6 ni+3, t7 nj+3, t8 MEDGE, t9 MCROSS, t10 ncell-1
    p.mov(t0, Z); p.mov(t3, Z); p.addi(t6, NI, 3); p.addi(t7, NJ, 3); p.mov(t2, Z)
    p.li(t8, MEDGE); p.li(t9, MCROSS)
    p.label('GA_I')
    p.mov(t1, Z); p.mov(t4, Z); p.mov(t5, Z)
    p.label('GA_J')
    # byte（inb 时取裁剪区字节，否则 0）
    p.mov(t11, Z)
    p.bge(t0, NI, 'GA_B'); p.bge(t1, NJ, 'GA_B')
    p.add(t12, t2, t1)                         # c
    p.shr(t13, t12, 3); p.ld(t13, t13, A_ENT)  # 裁剪区字（64 位，8 字节）
    p.andi(t12, t12, 7); p.shl(t12, t12, 3); p.shrr(t13, t13, t12); p.andi(t11, t13, 255)
    p.label('GA_B')
    # occ = (byte & 15) != 0；fc = popcount(byte >> 4)
    p.andi(t12, t11, 15); p.slt(t12, Z, t12)                       # occ（0/1）
    p.shr(t13, t11, 4); p.shr(t14, t13, 1); p.andi(t14, t14, 5); p.sub(t13, t13, t14)   # 4 位 popcount（2 位和）
    p.andi(t14, t13, 3); p.shr(t13, t13, 2); p.add(t13, t13, t14)                       # fc
    p.slti(t14, t13, 2); p.xori(t14, t14, 1); p.slt(t15, Z, t12); p.xori(t15, t15, 1); p.and_(t14, t14, t15)  # fre
    p.slt(t15, Z, t13)                                                                  # any
    # hst = i==0 ? 0 : DIL[j]
    p.mov(t16, Z); p.bz(t0, 'GA_H'); p.ld(t16, t1, A_GRAW + DIL)
    p.label('GA_H')
    # colo = ((hst & 63) << 1 | occ) & 127 ; colf = ((hst>>6 & 63) << 2 | fre<<1 | any) & 255
    p.andi(t17, t16, 63); p.shl(t17, t17, 1); p.or_(t17, t17, t12)
    p.shr(t18, t16, 6); p.andi(t18, t18, 63); p.shl(t18, t18, 2); p.shl(t14, t14, 1); p.or_(t18, t18, t14); p.or_(t18, t18, t15)
    # DIL[j] = (colo & 63) | (colf & 63) << 6
    p.andi(t14, t17, 63); p.andi(t15, t18, 63); p.shl(t15, t15, 6); p.or_(t14, t14, t15); p.st(t1, A_GRAW + DIL, t14)
    # win/fa 移位（j==0 时先清）
    p.shr(t4, t4, 7); p.shl(t14, t17, 42); p.or_(t4, t4, t14)
    p.shr(t5, t5, 2); p.shr(t14, t18, 6); p.shl(t14, t14, 6); p.or_(t5, t5, t14)
    # 输出 (i-3, j-3)
    p.slti(t14, t0, 3); p.bnz(t14, 'GA_N'); p.slti(t14, t1, 3); p.bnz(t14, 'GA_N')
    p.and_(t14, t4, t8); p.slt(t14, Z, t14)                     # edge（win 打包后非负）
    p.and_(t15, t4, t9); p.slt(t15, Z, t15)                     # o || cross
    p.andi(t16, t5, 3)                                          # ofa
    # trav = !(o||cross) && oany；lane = ofr<<14 | trav<<15 | (!o)<<16 | edge<<17
    p.xori(t15, t15, 1); p.andi(t17, t16, 1); p.and_(t15, t15, t17); p.shl(t15, t15, 15)
    p.shr(t17, t16, 1); p.shl(t17, t17, 14); p.or_(t15, t15, t17)
    p.shr(t17, t4, 24); p.andi(t17, t17, 1); p.xori(t17, t17, 1); p.shl(t17, t17, 16); p.or_(t15, t15, t17)
    p.shl(t14, t14, 17); p.or_(t15, t15, t14)
    p.st(t3, A_LANE, t15); p.addi(t3, t3, 1)
    p.label('GA_N')
    p.addi(t1, t1, 1); p.blt(t1, t7, 'GA_J')
    p.add(t2, t2, NJ); p.addi(t0, t0, 1); p.blt(t0, t6, 'GA_I')
    p.ret()

    # ======================================================== R-B：光栅序压缩编号，置可见域位，清 SEEN
    p.label('GRIDB')
    p.mul(t1, NI, NJ); p.mov(t0, Z); p.mov(U, Z); p.li(t3, SR_UMAX); p.li(t4, ~0x3FFF)
    p.bz(t1, 'GB_X')
    p.label('GB_L')
    p.ld(t2, t0, A_SEEN); p.bz(t2, 'GB_N')
    p.st(t0, A_SEEN, Z)
    p.bge(U, t3, 'GB_O')
    p.ld(t5, t0, A_LANE); p.and_(t5, t5, t4); p.or_(t5, t5, U); p.ori(t5, t5, 1 << 13); p.st(t0, A_LANE, t5)
    p.addi(U, U, 1); p.j('GB_N')
    p.label('GB_O'); p.ori(ERR, ERR, E_UOVER)
    p.label('GB_N')
    p.addi(t0, t0, 1); p.blt(t0, t1, 'GB_L')
    p.label('GB_X')
    p.ret()

    # ======================================================== 200x200 代价图：内部 (i+1, j+1) 可走 0，否则 254
    p.label('COSTMAP')
    # t0 r, t1 cl, t2 lo, t3 hi, t4 byte, t5 (r-1)*nj, t6 out, t7 200, t8 254, t12 线性计数
    p.mov(t0, Z); p.mov(t2, Z); p.mov(t3, Z); p.mov(t6, AUX2); p.li(t7, 200); p.li(t8, 254); p.sub(t5, Z, NJ); p.mov(t12, Z)
    p.label('CM_R')
    p.mov(t1, Z)
    p.label('CM_C')
    p.mov(t4, t8)
    p.bz(t0, 'CM_B'); p.bz(t1, 'CM_B'); p.blt(NI, t0, 'CM_B'); p.blt(NJ, t1, 'CM_B')
    p.add(t9, t5, t1); p.ld(t9, t9, A_LANE - 1)
    p.shr(t9, t9, 15); p.andi(t9, t9, 1); p.seqi(t9, t9, 0); p.sel(t4, t8, Z)
    p.label('CM_B')
    p.shr(t2, t2, 8); p.shl(t9, t3, 56); p.or_(t2, t2, t9)
    p.shr(t3, t3, 8); p.shl(t9, t4, 56); p.or_(t3, t3, t9)
    p.addi(t12, t12, 1); p.andi(t9, t12, 15); p.bnz(t9, 'CM_N')
    p.mtd(0, t2); p.mtd(1, t3); p.dst(t6); p.addi(t6, t6, 16)
    p.label('CM_N')
    p.addi(t1, t1, 1); p.blt(t1, t7, 'CM_C')
    p.add(t5, t5, NJ); p.addi(t0, t0, 1); p.blt(t0, t7, 'CM_R')
    p.ret()

    # ======================================================== NBR：近节点圈（md0，INIT_UNIV）/ 最近可走格（md1）
    p.label('NBR')
    # t0 jb, t1 nj2, t2 R, t3 md, t4 prev, t5 jaddr; per job: t6 ox, t7 oy, t8 ic, t9 jc, t10 bk, t11 ba, t12 bb
    p.rsv_st(SNSO + 3, LR)
    p.mov(t0, Z); p.mov(t4, Z); p.mov(t5, AUX)
    p.mov(t1, NJOBS); p.li(t2, OP_CELLOF); p.bne(OP, t2, 'NB0'); p.li(t1, 1)
    p.label('NB0')
    p.li(t3, 1); p.li(t2, 5); p.li(t13, OP_INIT_UNIV); p.bne(OP, t13, 'NB1'); p.mov(t3, Z); p.li(t2, 3)
    p.label('NB1')
    p.bz(t1, 'NB_X')
    p.label('NB_J')
    p.dld(t5); p.mfd(t13, 0)
    p.sext(t6, t13, 24, 0); p.sext(t7, t13, 24, 24)
    p.prm(t13, P_GEO1)
    p.field(t14, t13, 48, 16); p.shl(t14, t14, 11); p.sub(t8, t7, t14); p.sext(t8, t8, 25); p.sra(t8, t8, 12)   # ic
    p.field(t14, t13, 32, 16); p.shl(t14, t14, 11); p.sub(t9, t6, t14); p.sext(t9, t9, 25); p.sra(t9, t9, 12)   # jc
    p.li(t10, 63); p.mov(t11, Z); p.mov(t12, Z)
    p.sub(t14, Z, t2)                     # di
    p.label('NB_DI')
    p.sub(t15, Z, t2)                     # dj
    p.label('NB_DJ')
    p.add(t16, t8, t14); p.add(t17, t9, t15)          # i, j
    p.blt(t16, Z, 'NB_N'); p.blt(t17, Z, 'NB_N'); p.bge(t16, NI, 'NB_N'); p.bge(t17, NJ, 'NB_N')
    p.mul(t18, t16, NJ); p.add(t18, t18, t17)          # cell
    p.ld(t19, t18, A_LANE)
    p.bnz(t3, 'NB_M1')
    # md0：DX = ((u0 + 2j + 1) << 11) - ox，DY 同；X <= rn2 且无占据子格 -> 置可走位
    p.shr(t13, t19, 16); p.andi(t13, t13, 1); p.bz(t13, 'NB_N')
    p.prm(t13, P_GEO1); p.field(t13, t13, 32, 16); p.add(t13, t13, t17); p.add(t13, t13, t17); p.addi(t13, t13, 1)
    p.shl(t13, t13, 11); p.sub(t13, t13, t6); p.mul(t13, t13, t13)
    p.st(Z, A_GRAW + SNSO + 4, t13)
    p.prm(t13, P_GEO1); p.field(t13, t13, 48, 16); p.add(t13, t13, t16); p.add(t13, t13, t16); p.addi(t13, t13, 1)
    p.shl(t13, t13, 11); p.sub(t13, t13, t7); p.mul(t13, t13, t13)
    p.ld(LR, Z, A_GRAW + SNSO + 4); p.add(t13, t13, LR)
    p.prm(LR, P_RN2); p.field(LR, LR, 0, 36); p.blt(LR, t13, 'NB_N')
    p.ori(t19, t19, 1 << 15); p.st(t18, A_LANE, t19)
    p.j('NB_N')
    p.label('NB_M1')
    # md1：tv && kk <= 25 && kk < bk
    p.shr(t13, t19, 15); p.andi(t13, t13, 1); p.bz(t13, 'NB_N')
    p.mul(t13, t14, t14); p.mul(LR, t15, t15); p.add(t13, t13, LR)
    p.slti(LR, t13, 26); p.bz(LR, 'NB_N')
    p.bge(t13, t10, 'NB_N')
    p.mov(t10, t13); p.mov(t11, t16); p.mov(t12, t17)
    p.label('NB_N')
    p.addi(t15, t15, 1); p.bge(t2, t15, 'NB_DJ')
    p.addi(t14, t14, 1); p.bge(t2, t14, 'NB_DI')
    # md1：结果两两写结果块
    p.bz(t3, 'NB_NX')
    p.mov(t13, Z); p.li(t15, 63); p.beq(t10, t15, 'NB_V')
    p.li(t13, 1); p.shl(t13, t13, 32); p.andi(t11, t11, 255); p.shl(t11, t11, 16); p.or_(t13, t13, t11)
    p.andi(t12, t12, 255); p.or_(t13, t13, t12)
    p.label('NB_V')
    p.andi(t15, t0, 1); p.bz(t15, 'NB_E')
    p.mtd(0, t4); p.mtd(1, t13); p.shr(t15, t0, 1); p.shl(t15, t15, 4); p.add(t15, t15, RES); p.dst(t15, 16 * 8)
    p.j('NB_P')
    p.label('NB_E')
    p.addi(t15, t0, 1); p.bne(t15, t1, 'NB_P')
    p.mtd(0, t13); p.mtd(1, Z); p.shr(t15, t0, 1); p.shl(t15, t15, 4); p.add(t15, t15, RES); p.dst(t15, 16 * 8)
    p.label('NB_P')
    p.mov(t4, t13)
    p.label('NB_NX')
    p.addi(t5, t5, 96); p.addi(t0, t0, 1); p.blt(t0, t1, 'NB_J')
    p.label('NB_X')
    p.rsv_ld(LR, SNSO + 3)
    p.ret()

    # ======================================================== ENGINE：逐作业调 SCAN
    p.label('ENGINE')
    # t0 mode, t1 nj2, t2 jb, t3 jaddr, t4 ntot
    p.li(t0, MD_OBS); p.li(t1, 1); p.li(t5, SR_ENTRY_OBS); p.li(t6, SR_SLOTS - 1)
    p.li(t7, OP_OBSERVE); p.beq(OP, t7, 'EN0')
    p.mov(t1, NJOBS); p.mov(t5, Z); p.li(t6, SR_ENTRY_OBS - 1)
    p.li(t0, MD_VIEW); p.li(t7, OP_INIT_VIEWS); p.beq(OP, t7, 'EN0')
    p.li(t0, MD_UNIV)
    p.label('EN0')
    p.mts(S_PTR, t5); p.mts(S_PEND, Z); p.mts(S_LIM, t6); p.mts(S_ERR, ERR)
    p.mov(t2, Z); p.mov(t3, AUX); p.mov(t4, Z)
    p.bz(t1, 'EN_X')
    p.label('EN_J')
    p.dld(t3); p.mfd(t8, 0); p.mfd(t9, 1)
    p.mts(S_JOB0, t8); p.mts(S_JOB1, t9)
    # vrow = node*40 + (si0 - vi0)，vbit0 = sj0 - vj0，wrow = node*40
    p.field(t10, t8, 48, 7); p.muli(t10, t10, 40)
    p.sext(t11, t9, 16, 0); p.sext(t12, t9, 16, 32); p.sub(t11, t11, t12); p.add(t11, t11, t10); p.andi(t11, t11, 0x1FFF)
    p.sext(t12, t9, 16, 16); p.sext(t13, t9, 16, 48); p.sub(t12, t12, t13); p.andi(t12, t12, 63); p.shl(t12, t12, 16)
    p.or_(t11, t11, t12); p.shl(t10, t10, 32); p.or_(t11, t11, t10); p.mts(S_VW, t11)
    # 锥体系数（UNIV 不用）
    p.li(t7, MD_UNIV); p.beq(t0, t7, 'EN_S')
    for m in range(5):
        # jd[2+2m], jd[3+2m] 在第 1+m 拍
        p.dld(t3, 16 * (1 + m)); p.mfd(t10, 0); p.mts(S_CFA, t10); p.mfd(t10, 1); p.mts(S_CF + m, t10)
    p.label('EN_S')
    p.addi(t10, t2, 1); p.seq(t10, t10, t1); p.shl(t10, t10, 2); p.or_(t10, t10, t0); p.mts(S_MODE, t10)
    p.scan()
    p.mfs(t10, F_NENT); p.add(t4, t4, t10)
    p.addi(t3, t3, 96); p.addi(t2, t2, 1); p.blt(t2, t1, 'EN_J')
    p.label('EN_X')
    p.mfs(ERR, F_ERR)
    p.mfs(t10, F_PTR); p.shl(t11, t4, 32); p.or_(t11, t11, t10); p.rsv_st(RAUX, t11)
    p.subi(t10, t10, SR_ENTRY_OBS); p.andi(t10, t10, 0xFFFF); p.rsv_st(SNSO, t10)
    p.ret()

    # ======================================================== PRIOR：mode = op - PRIOR0
    p.label('PRIOR')
    p.rsv_st(SNSO + 3, LR)
    # 常驻：t0 i, t1 j, t2 cell, t3 mode, t4 mn/sm, t5 sn, t6 sw, t7 tx, t8 ty, t9 cxb=(u0<<1)+1 基, t10 cyb
    p.subi(t3, OP, OP_PRIOR0)
    p.prm(t11, P_ORG); p.sext(t7, t11, 32, 0); p.sext(t8, t11, 32, 32)
    p.li(t4, (1 << 42) - 1); p.mov(t5, Z); p.mov(t6, Z); p.bnz(t3, 'PR_0'); p.j('PR_1')
    p.label('PR_0'); p.mov(t4, Z)
    p.label('PR_1')
    p.mov(t0, Z); p.mov(t2, Z)
    p.bz(NI, 'PR_X')
    p.label('PR_I')
    p.mov(t1, Z)
    p.label('PR_J')
    p.ld(t11, t2, A_LANE)
    p.shr(t12, t11, 13); p.andi(t12, t12, 1); p.bz(t12, 'PR_N')       # 非可见域格：两拍都无可观测效果
    # DX, DY, X
    p.prm(t12, P_GEO1); p.field(t13, t12, 32, 16); p.add(t13, t13, t1); p.add(t13, t13, t1); p.addi(t13, t13, 1)
    p.shl(t13, t13, 11); p.sub(t13, t13, t7); p.mul(t13, t13, t13)
    p.field(t14, t12, 48, 16); p.add(t14, t14, t0); p.add(t14, t14, t0); p.addi(t14, t14, 1)
    p.shl(t14, t14, 11); p.sub(t14, t14, t8); p.mul(t14, t14, t14); p.add(t12, t13, t14)      # t12 = X
    p.bnz(t3, 'PR_E')
    p.min_(t4, t4, t12); p.j('PR_N')
    p.label('PR_E')
    # e(s)：t13 = e(s=0)，t14 = e(s=1)；子程序 PEXP：入 t12 X, t15 K, t16 I；出 t17 e（用 t18 t19 LR）
    p.rsv_st(SNSO + 5, t11)
    p.prm(t15, P_K); p.field(t15, t15, 0, 32); p.prm(t16, P_I0); p.field(t16, t16, 0, 32)
    p.call('PEXP'); p.mov(t13, t17)
    p.prm(t15, P_K); p.field(t15, t15, 32, 32); p.prm(t16, P_I0); p.field(t16, t16, 32, 32)
    p.call('PEXP'); p.mov(t14, t17)
    p.rsv_ld(t11, SNSO + 5)
    p.shr(t15, t11, 17); p.andi(t15, t15, 1)          # edge
    p.li(t16, 1); p.beq(t3, t16, 'PR_M1')
    # mode 2：a = cm*e1w + cn*e；pr = ((a >> 24) + cw*w + ccst) & MSK40；PST[u] 低 40 位 = pr
    p.bz(t15, 'PR_2A'); p.muli(t13, t13, 3)
    p.label('PR_2A')
    # cm = cmh*2^20 + cml（cm < 2^40），cn 同：Hi = cmh*e1w + cnh*e，Lo = cml*e1w + cnl*e，a>>24 = (Hi + (Lo >> 20)) >> 4
    p.prm(t16, P_CM); p.shr(t17, t16, 20); p.andi(t17, t17, (1 << 20) - 1); p.andi(t16, t16, (1 << 20) - 1)
    p.mul(t17, t17, t13); p.mul(t16, t16, t13)
    p.prm(t18, P_CN); p.shr(t19, t18, 20); p.andi(t19, t19, (1 << 20) - 1); p.andi(t18, t18, (1 << 20) - 1)
    p.mul(t19, t19, t14); p.mul(t18, t18, t14)
    p.add(t17, t17, t19); p.add(t16, t16, t18); p.shr(t16, t16, 20); p.add(t17, t17, t16); p.shr(t17, t17, 4)
    p.prm(t16, P_CW); p.field(t16, t16, 0, 40)
    p.add(t17, t17, t16)
    p.bz(t15, 'PR_2B'); p.shl(t16, t16, 1); p.add(t17, t17, t16)
    p.label('PR_2B')
    p.prm(t16, P_CC); p.field(t16, t16, 0, 40); p.add(t17, t17, t16); p.field(t17, t17, 0, 40)
    p.andi(t11, t11, 0x1FFF); p.shl(t11, t11, 1)
    p.ld(t16, t11, A_PST); p.shr(t16, t16, 40); p.shl(t16, t16, 40); p.or_(t16, t16, t17); p.st(t11, A_PST, t16)
    p.j('PR_N')
    p.label('PR_M1')
    # mode 1：sm += ew，sw += w，sn += e(s=1)
    p.add(t5, t5, t14); p.addi(t6, t6, 1); p.add(t4, t4, t13)
    p.bz(t15, 'PR_N'); p.addi(t6, t6, 2); p.shl(t13, t13, 1); p.add(t4, t4, t13)
    p.label('PR_N')
    p.addi(t2, t2, 1); p.addi(t1, t1, 1); p.blt(t1, NJ, 'PR_J')
    p.addi(t0, t0, 1); p.blt(t0, NI, 'PR_I')
    p.label('PR_X')
    p.rsv_ld(LR, SNSO + 3)
    p.bnz(t3, 'PR_R1')
    p.rsv_st(R3B, t4); p.ret()
    p.label('PR_R1')
    p.rsv_st(R4A, t4); p.rsv_st(R4B, t5); p.rsv_st(R3A, t6); p.ret()

    # PEXP：e = 先验指数插值（srch_core prior() 内一拍）。入 t12 X, t15 K, t16 I；出 t17
    p.label('PEXP')
    p.shr(t17, t12, 10); p.mul(t17, t17, t15); p.shr(t17, t17, 28)     # xr
    p.shl(t16, t16, 16); p.sub(t17, t17, t16); p.max_(t17, t17, Z)     # x = max(xr - Ish, 0)
    p.shr(t18, t17, 16)                                                # ip
    p.andi(t17, t17, 0xFFFF)                                           # fr
    p.shr(t19, t17, 7); p.ld(t19, t19, A_GRAW + EXPT)                  # tw
    p.andi(t17, t17, 127); p.shr(t16, t19, 32); p.andi(t16, t16, (1 << 25) - 1); p.andi(t19, t19, (1 << 25) - 1)  # t16 T1, t19 T0
    p.sub(t16, t19, t16); p.mul(t16, t16, t17); p.shr(t16, t16, 7); p.sub(t19, t19, t16)   # ev
    p.slti(t16, t18, 25); p.bz(t16, 'PE_Z')
    p.li(t16, 24); p.sub(t16, t16, t18); p.li(t17, 1); p.shlr(t17, t17, t16)               # p2
    p.mul(t17, t17, t19); p.shr(t17, t17, 24); p.ret()
    p.label('PE_Z'); p.mov(t17, Z); p.ret()

    # ======================================================== BELIEF
    p.label('BELIEF')
    p.li(t0, OP_OBSERVE); p.bne(OP, t0, 'BE_S')
    p.li(t1, BM_OBS); p.mts(S_BMODE, t1); p.rsv_ld(t1, SNSO); p.mts(S_BN, t1); p.li(t1, SR_ENTRY_OBS); p.mts(S_BBASE, t1)
    p.mts(S_BRST, Z); p.bel(t2); p.ret()
    p.label('BE_S')
    p.li(t1, BM_MASS); p.mts(S_BMODE, t1); p.mts(S_BN, U); p.mts(S_BBASE, Z); p.mts(S_BRST, Z); p.bel(t2)
    p.mfs(t2, F_ACC); p.rsv_st(R2A, t2); p.mfs(t2, F_ACC2); p.rsv_st(R2B, t2)
    p.li(t1, BM_SEL); p.mts(S_BMODE, t1); p.rsv_ld(t1, SNSO + 1); p.mts(S_BN, t1); p.mts(S_BRST, Z)
    p.addi(t3, RES, 16 * 48)
    p.label('BE_L')
    p.bel(t2); p.bz(t2, 'BE_X')
    p.mfs(t4, F_ACC); p.mtd(0, t4); p.mfs(t4, F_ACC2); p.mtd(1, t4); p.dst(t3); p.addi(t3, t3, 16)
    p.j('BE_L')
    p.label('BE_X')
    p.ret()
    return p


def emit_rom(p, path_h, path_lst):
    words = p.assemble()
    mut = os.environ.get('UPROG6_MUT')          # 阳性对照：'pc:新立即数'，改一条微指令的立即数
    if mut:
        pc, v = (int(x) for x in mut.split(':'))
        words[pc] = (words[pc] & ((1 << 21) - 1)) | ((v & ((1 << 27) - 1)) << 21)
    n = len(words)
    assert n <= 1024, n
    with open(path_h, 'w') as f:
        f.write('// generated by uprog6.py (%d words)\n#pragma once\n#define UROM_N %d\n' % (n, n))
        f.write('static const ap_uint<48> UROM[1024] = {\n')
        for k in range(0, n, 6):
            f.write('  ' + ', '.join('0x%012xULL' % w for w in words[k:k + 6]) + ',\n')
        f.write('};\n')
    with open(path_lst, 'w') as f:
        f.write(p.listing() + '\n')
    return n


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    p = build()
    out = os.environ.get('UPROG6_OUT', here)
    n = emit_rom(p, os.path.join(out, 'srch6_urom.h'), os.path.join(out, 'srch6_urom.lst'))
    print('ROM words', n)
