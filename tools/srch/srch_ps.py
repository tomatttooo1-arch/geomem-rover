"""SRCHK1 搜索核的 PS 侧（驱动）＋ FX 对象：产品 BeliefSearch 的子类，可见域/视点表/观测/先验/质量/增益全部改调核
（定点 C 模型 libsrch.so，SRCH_SW 编译；与 HLS 源同一份 hw/srch/srch_core.cpp）。next_goal、打分、扫尾、停止、记账
仍是产品原码。PS 只做：地图打包、位姿 -> 判定式、先验三个系数、作业表编排。"""
import ctypes
import math
import os
import struct

import numpy as np

from host.search_belief import BeliefSearch

OP = dict(INIT_MAP=1, INIT_UNIV=3, INIT_FINAL=4, INIT_VIEWS=5, RESET=6, PRIOR0=7, PRIOR1=8, PRIOR2=9,
          OBSERVE=10, CELLOF=11, SELECT=12, DUMP=13)
MAGIC, VER = 0x5352, 2
# DDR 布局（字节）
A_PRM, A_RES, A_OCC, A_CROP, A_TAB, A_JOB, A_COST, A_DUMP, A_JOB1 = (0x0, 0x400, 0x3000, 0xB000, 0x15000, 0x18000,
                                                                   0x27000, 0x31000, 0x51000)
DDR_BYTES = 0x60000
R_VIEW, R_NODECELL = 96, 16
Q11 = 2048.0


def _q11(v):
    return int(round(v * Q11))


def tables():
    rsq = [int(round(2 ** 46 / math.sqrt((i + 256 + 0.5) * 2 ** 22))) for i in range(768)]
    t = [int(round(2 ** 24 * 2 ** (-i / 512.0))) for i in range(513)]
    exp = [t[i] | (t[i + 1] << 32) for i in range(512)]
    return np.array(rsq + exp, np.uint64)


def u16(v):
    return int(v) & 0xFFFF


class Core(object):
    """核的一次实例（片上状态在 .so 的静态数组里，所以一个进程一次只用一个 Core）。"""

    def __init__(self, lib_path):
        self.lib = ctypes.CDLL(lib_path)
        self.lib.srch_run_sw.restype = ctypes.c_uint32
        self.lib.srch_run_sw.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        self.ddr = np.zeros(DDR_BYTES, np.uint8)
        self.trace = None          # list of (prm, before, after)
        self.calls = []

    def u64(self, addr, n):
        return self.ddr[addr:addr + 8 * n].view(np.uint64)

    def run(self, op, prm_u64):
        p = np.zeros(38, np.uint64)
        prm_u64 = dict(prm_u64)
        nseg = int(prm_u64.pop("nseg", 0))
        for k, v in prm_u64.items():
            p[k] = np.uint64(int(v) & 0xFFFFFFFFFFFFFFFF)
        p[0] = np.uint64(MAGIC | (VER << 16) | (op << 24) | (nseg << 32))
        self.ddr[A_PRM:A_PRM + 38 * 8] = p.view(np.uint8)
        before = self.ddr.copy() if self.trace is not None else None
        st = self.lib.srch_run_sw(self.ddr.ctypes.data_as(ctypes.c_void_p), A_PRM)
        if self.trace is not None:
            self.trace.append((op, A_PRM, before, self.ddr.copy()))
        if st & 0xFF:
            raise RuntimeError("core op %d error 0x%x" % (op, st & 0xFF))
        return self.u64(A_RES, 96 + 1280)


# ---- 板上驱动（SRCHK3 按 HLS 生成的寄存器表核过：srch_top_control_s_axi.v） ----------------------------------
REG_CTRL, REG_GIE, REG_IER, REG_ISR = 0x00, 0x04, 0x08, 0x0C
REG_PRM_LO, REG_PRM_HI = 0x10, 0x14     # 参数块物理地址（16 对齐）
REG_STATUS, REG_STATUS_VLD = 0x1C, 0x20 # status[7:0] 错误位、[15:8] 版本、[23:16] 操作码；VLD 读清
AP_START, AP_DONE, AP_IDLE = 0x1, 0x2, 0x4
ADDR_FIELDS = (1, 2, 3, 37)             # 参数块里放 DDR 地址的 u64 下标（P_RES、P_AUX、P_AUX2、P_TAB）


class PLCore(Core):
    """板上 PL 核。regs 有 read32(off)/write32(off, v)（如 host/m7_pl_device 的寄存器窗口）；buf 是一块物理连续、
    非缓存（或调用方负责刷/失效）的 DMA 缓冲的 numpy uint8 视图，至少 DDR_BYTES 字节，物理基址 phys。
    FX 照旧把输入写进 self.ddr、用相对偏移填参数块；这里在下发前把地址字段加上 phys。
    核内 DDR 地址按 32 位算（SRCHK3），缓冲必须整块在 4 GB 以下。"""

    def __init__(self, regs, buf, phys, timeout_s=2.0):
        assert buf.dtype == np.uint8 and buf.size >= DDR_BYTES
        assert phys % 16 == 0 and phys + DDR_BYTES <= 1 << 32, "缓冲须 16 对齐且整块在 4 GB 以下"
        self.regs, self.ddr, self.phys, self.timeout_s = regs, buf[:DDR_BYTES], int(phys), timeout_s
        self.trace = None
        self.calls = []

    def run(self, op, prm_u64):
        import time as _t
        p = np.zeros(38, np.uint64)
        prm_u64 = dict(prm_u64)
        nseg = int(prm_u64.pop("nseg", 0))
        for k, v in prm_u64.items():
            v = int(v)
            if k in ADDR_FIELDS:
                v += self.phys
            p[k] = np.uint64(v & 0xFFFFFFFFFFFFFFFF)
        p[0] = np.uint64(MAGIC | (VER << 16) | (op << 24) | (nseg << 32))
        self.ddr[A_PRM:A_PRM + 38 * 8] = p.view(np.uint8)
        pa = self.phys + A_PRM
        r = self.regs
        assert r.read32(REG_CTRL) & AP_IDLE, "核忙"
        r.write32(REG_PRM_LO, pa & 0xFFFFFFFF)
        r.write32(REG_PRM_HI, pa >> 32)
        r.write32(REG_CTRL, AP_START)
        t0 = _t.monotonic()
        while not (r.read32(REG_CTRL) & AP_DONE):
            if _t.monotonic() - t0 > self.timeout_s:
                raise RuntimeError("core op %d timeout" % op)
        st = r.read32(REG_STATUS) if r.read32(REG_STATUS_VLD) & 1 else int(self.u64(A_RES, 1)[0]) & 0xFFFFFFFF
        if st & 0xFF:
            raise RuntimeError("core op %d error 0x%x" % (op, st & 0xFF))
        return self.u64(A_RES, 96 + 1280)


class MockRegs(object):
    """宿主自检用：寄存器表同 HLS，ap_start 时用定点 C 模型（libsrch.so）按物理地址执行。"""

    def __init__(self, lib_path, buf, phys):
        self.lib = ctypes.CDLL(lib_path)
        self.lib.srch_run_sw.restype = ctypes.c_uint32
        self.lib.srch_run_sw.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        self.buf, self.phys, self.r = buf, phys, {REG_CTRL: AP_IDLE}

    def read32(self, off):
        v = self.r.get(off, 0)
        if off == REG_STATUS_VLD:
            self.r[off] = 0
        if off == REG_CTRL:
            self.r[off] = v & ~AP_DONE
        return v

    def write32(self, off, v):
        if off == REG_CTRL and v & AP_START:
            pa = self.r[REG_PRM_LO] | (self.r[REG_PRM_HI] << 32)
            # 让核看到的"地址 0"落在 buf 起点之前 phys 字节处：核按物理地址访问，正好落进 buf
            st = self.lib.srch_run_sw(ctypes.c_void_p(self.buf.ctypes.data - self.phys), pa)
            self.r[REG_STATUS], self.r[REG_STATUS_VLD], self.r[REG_CTRL] = st, 1, AP_IDLE | AP_DONE
        else:
            self.r[off] = v


class FX(BeliefSearch):
    """产品 BeliefSearch，逐格/逐射线/逐视点计算全部改走核。构造后必须 set_camera 再 core_init。"""

    def core_attach(self, core):
        self.core = core
        self._sel = None

    # ---- PS 侧打包 ------------------------------------------------------------------------------------------
    def _base_prm(self):
        cfg = self.cfg
        h, w = self.occ.shape
        self.ou, self.ov = max(0, self.u0 - 40), max(0, self.v0 - 40)
        assert self.u0 + 2 * self.nj <= self.ou + 512 and self.v0 + 2 * self.ni <= self.ov + 512
        assert self.f == 2 and abs(self.res - 0.05) < 1e-9
        res = self.res
        R = cfg.max_range_m / res
        off = round(float(np.hypot(*self.T_base_camera[:2, 3])), 3)
        RW = (cfg.max_range_m + off + 1e-6) / res
        qd, qv = [d for d, _ in cfg.q_table], [q for _, q in cfg.q_table]
        qt = []
        for i in range(len(qd)):
            D = _q11(qd[i] / res)
            Q = int(round(qv[i] * 2 ** 23))
            S = 0
            if i + 1 < len(qd):
                S = int(round((qv[i + 1] - qv[i]) / ((qd[i + 1] - qd[i]) / res) * 2 ** 26))
            assert D < 2 ** 18 and -2 ** 22 <= S < 2 ** 22
            qt.append(D | (Q << 18) | ((S & (2 ** 23 - 1)) << 41))
        lm = (cfg.look_move_m / res * 64) ** 2
        p = {1: A_RES, 4: self.ni | (self.nj << 16) | (self.u0 << 32) | (self.v0 << 48),
             5: w | (h << 16) | (self.ou << 32) | (self.ov << 48),
             7: int(math.floor(R * R * 2 ** 22)), 8: int(math.floor(RW * RW * 2 ** 22)),
             9: int(math.floor((cfg.node_clear_m / res) ** 2 * 2 ** 22)), 10: int(math.ceil(lm)),
             11: _q11(cfg.object_radius_m / res) | (int(round(cfg.miss_floor * 2 ** 30)) << 32),
             "nseg": len(qd)}
        for i, v in enumerate(qt):
            p[12 + i] = v
        self._p = p
        return p

    def _cnt(self, njobs=0):
        return self.U | (self.nsv << 16) | (self.nso << 32) | (njobs << 48)

    def _prm(self, **kw):
        p = dict(self._p)
        p[6] = self._cnt(kw.pop("njobs", 0))
        for k, v in kw.items():
            p[int(k[1:])] = v
        return p

    def _cell_ij(self, ox, oy):
        return (oy - (self.v0 << 11)) >> 12, (ox - (self.u0 << 11)) >> 12

    def _job(self, ox, oy, node, win, vwin, forms):
        w = np.zeros(12, np.uint64)
        w[0] = (ox & 0xFFFFFF) | ((oy & 0xFFFFFF) << 24) | (node << 48)
        w[1] = u16(win[0]) | (u16(win[1]) << 16) | (u16(vwin[0]) << 32) | (u16(vwin[1]) << 48)
        for m, (A, B, F0, F1) in enumerate(forms):
            w[2 + 2 * m] = (A & 0x3FFFF) | ((B & 0x3FFFF) << 18)
            w[3 + 2 * m] = (F0 & 0xFFFFFFF) | ((F1 & 0xFFFFFFF) << 32)
        return w

    def _origin(self, x, y):
        u, v = self._uv(x, y)
        return _q11(float(u)), _q11(float(v))

    def _forms(self, T, ox, oy):
        """5 个判定式（Z、U0、U1、V0、V1）在相机地面点处展开：A、B 每像素，F0、F0' 在 (ox, oy)（Q.11 取整后）处。"""
        K = self.K
        wimg, himg = self.image_size
        Rt = T[:3, :3].T
        tc = -Rt @ T[:3, 3]
        gs = [((0., 0., 1.), -0.05), ((K[0, 0], K[0, 1], K[0, 2]), 0.),
              ((-K[0, 0], -K[0, 1], -(K[0, 2] - wimg)), 0.), ((0., K[1, 1], K[1, 2]), 0.),
              ((0., -K[1, 1], -(K[1, 2] - himg)), 0.)]
        cx, cy = self._xy(ox / Q11, oy / Q11)
        rot = np.array([[self.c, -self.s], [self.s, self.c]])
        H = self.cfg.object_height_m
        out = []
        for g, h0 in gs:
            g = np.array(g)
            a = g @ Rt
            A, B = self.res * (a[:2] @ rot)
            F0 = a[0] * cx + a[1] * cy + g @ tc + h0
            F1 = F0 + a[2] * H
            k = 40
            while k > -20 and (max(abs(A), abs(B)) * 2 ** k >= 2 ** 16 or max(abs(F0), abs(F1)) * 2 ** k >= 2 ** 26):
                k -= 1
            Af, Bf, F0f, F1f = (int(round(v * 2 ** k)) for v in (A, B, F0, F1))
            out.append((Af, Bf, F0f, F1f))
        return out

    # ---- 初始化 ---------------------------------------------------------------------------------------------
    def core_init(self):
        core = self.core
        assert self.has_camera() and self.T_base_camera is not None
        self.U = self.nsv = self.nso = 0
        p = self._base_prm()
        h, w = self.occ.shape
        occw = np.zeros((512, 512), bool)
        hh, ww = min(512, h - self.ov), min(512, w - self.ou)
        occw[:hh, :ww] = self.occ[self.ov:self.ov + hh, self.ou:self.ou + ww]
        core.ddr[A_OCC:A_OCC + 32768] = np.packbits(occw, axis=1, bitorder="little").ravel()
        cells = np.asarray(self._cells)
        kf = (cells >= 0) & (cells < 50)
        blk = lambda a: a[self.v0:self.v0 + 2 * self.ni, self.u0:self.u0 + 2 * self.nj].reshape(  # noqa: E731
            self.ni, 2, self.nj, 2).transpose(0, 2, 1, 3).reshape(self.ni, self.nj, 4)
        ob, fb = blk(self.occ).astype(np.uint8), blk(kf).astype(np.uint8)
        byte = (ob * np.array([1, 2, 4, 8], np.uint8)).sum(axis=2) + (fb * np.array([16, 32, 64, 128])).sum(axis=2)
        n = self.ni * self.nj
        core.ddr[A_CROP:A_CROP + n] = byte.astype(np.uint8).ravel()
        tb = tables()
        core.ddr[A_TAB:A_TAB + tb.size * 8] = tb.view(np.uint8)
        core.run(OP["INIT_MAP"], self._prm(_2=A_OCC, _3=A_CROP, _37=A_TAB))
        # 节点作业（可见窗 40x40，起点 = 节点格 - 19）
        jobs = []
        self.node_q = []
        for nidx, (x, y) in enumerate(self.nodes):
            ox, oy = self._origin(x, y)
            i, j = self._cell_ij(ox, oy)
            self.node_q.append((ox, oy, i - 19, j - 19))
            jobs.append(self._job(ox, oy, nidx, (i - 19, j - 19), (i - 19, j - 19), [(0, 0, 0, 0)] * 5))
        nn = len(jobs)
        core.ddr[A_JOB:A_JOB + 96 * nn] = np.concatenate(jobs).view(np.uint8)
        core.run(OP["INIT_UNIV"], self._prm(_2=A_JOB, njobs=nn))
        r = core.run(OP["INIT_FINAL"], self._prm(_2=A_JOB, _3=A_COST, njobs=nn))
        self.U = int(r[0]) >> 32
        self.node_cell_c = [None if not (int(r[R_NODECELL + k]) >> 32) & 1 else
                            ((int(r[R_NODECELL + k]) >> 16) & 0xFFFF, int(r[R_NODECELL + k]) & 0xFFFF)
                            for k in range(nn)]
        self.costmap_c = core.ddr[A_COST:A_COST + 40000].copy()
        # GOLIVE1: the board driver (aarch64 libsrch_drv.so) also has .lib but no srch_dump_grid (C model only);
        # probe the symbol instead of the attribute, else the board path raised AttributeError at init
        dump = getattr(getattr(core, "lib", None), "srch_dump_grid", None) if getattr(core, "lib", None) is not None else None
        if dump is not None:                           # 宿主 C 模型：直接读片上 GRID（判据 1 用）
            g = np.zeros(n, np.uint32)
            dump(g.ctypes.data_as(ctypes.c_void_p), ctypes.c_uint32(n))
            self.grid_c = g
            self.trav_c = ((g >> 15) & 1).astype(bool).reshape(self.ni, self.nj)
            self.univ_flat_c = np.flatnonzero((g >> 13) & 1)
        else:                                          # 板上 PL：可走位从代价图读回（内部 (i+1, j+1) 为 0 即可走）
            self.grid_c = self.univ_flat_c = None
            self.trav_c = (self.costmap_c.reshape(200, 200)[1:self.ni + 1, 1:self.nj + 1] == 0)
        self.build_views()
        self.core_reset()

    def build_views(self):
        core = self.core
        jobs = []
        for nidx, (x, y) in enumerate(self.nodes):
            nox, noy, vi0, vj0 = self.node_q[nidx]
            for yaw in self.yaw_set:
                T = self.camera_pose(x, y, yaw)
                ox, oy = self._origin(T[0, 3], T[1, 3])
                i, j = self._cell_ij(ox, oy)
                si0, sj0 = i - 16, j - 16
                assert 0 <= si0 - vi0 <= 7 and 0 <= sj0 - vj0 <= 7
                jobs.append(self._job(ox, oy, nidx, (si0, sj0), (vi0, vj0), self._forms(T, ox, oy)))
        nv = len(jobs)
        assert nv <= 640
        core.ddr[A_JOB:A_JOB + 96 * nv] = np.concatenate(jobs).view(np.uint8)
        r = core.run(OP["INIT_VIEWS"], self._prm(_2=A_JOB, njobs=nv))
        self.nsv = int(r[1]) & 0xFFFF
        self.nviews = nv

    def view_entries(self):
        """{v: (uidx array, q array)}，从 ENTRY 直接解析（判据 2）。"""
        s = np.zeros(self.nsv, np.uint64)
        self.core.lib.srch_dump_slots(s.ctypes.data_as(ctypes.c_void_p), ctypes.c_uint32(0), ctypes.c_uint32(self.nsv))
        s = s.astype(np.int64)
        out, v, k, cur = {}, -1, 0, []
        while k < len(s):
            if (s[k] & 0x1FFF) == 0x1FFF:
                if v >= 0:
                    out[v] = np.array(cur, np.int64)
                v += 1
                cur = []
                k += 2
                continue
            cur.append(s[k])
            k += 1
        return {kk: ((vv & 0x1FFF), (vv >> 13) / 2. ** 23) for kk, vv in out.items()}

    # ---- 产品接口覆写 -----------------------------------------------------------------------------------------
    def reset(self):
        BeliefSearch.reset(self)
        if getattr(self, "core", None) is not None and getattr(self, "U", 0):
            self.core_reset()

    def core_reset(self):
        self.core.run(OP["RESET"], self._prm())
        self._sel = None

    def begin(self, target_xy=None):
        cfg, core = self.cfg, self.core
        res = self.res
        k1 = int(round(math.log2(math.e) / (2 * (cfg.prior_sigma_m / res) ** 2) * 2 ** 32))
        k2 = int(round(math.log2(math.e) / (2 * (cfg.near_sigma_m / res) ** 2) * 2 ** 32))
        has = target_xy is not None and all(math.isfinite(float(v)) for v in target_xy[:2])
        tx, ty = (self._origin(float(target_xy[0]), float(target_xy[1])) if has else (0, 0))
        org = (tx & 0xFFFFFFFF) | ((ty & 0xFFFFFFFF) << 32)
        kk = k1 | (k2 << 32)
        i1 = i2 = 0
        if has:
            r = core.run(OP["PRIOR0"], self._prm(_20=org, _21=kk))
            xmin = int(r[7])
            i1 = (((xmin >> 10) * k1) >> 28) >> 16
            i2 = (((xmin >> 10) * k2) >> 28) >> 16
        r = core.run(OP["PRIOR1"], self._prm(_20=org, _21=kk, _22=i1 | (i2 << 32)))
        sw, sm, sn = int(r[6]), int(r[4]), int(r[5])
        uf, nm = cfg.uniform_floor, cfg.near_mass
        S39 = 2. ** 39
        if has:
            main_ok = sm / 2. ** 24 * 2. ** (-i1) > 1e-12
            near_ok = nm > 0 and sn / 2. ** 24 * 2. ** (-i2) > 1e-6
            wm = (1 - uf) * ((1 - nm) if near_ok else 1.)
            cm = wm / (sm / 2. ** 24) * S39 if main_ok else 0.
            cc = 0. if main_ok else wm / self.U * S39
            cn = (1 - uf) * nm / (sn / 2. ** 24) * S39 if near_ok else 0.
            cw = uf / sw * S39
            self.target_xy = (float(target_xy[0]), float(target_xy[1]))
        else:
            cm = cn = cc = 0.
            cw = S39 / sw
            self.target_xy = None
        assert all(0 <= v < 2. ** 40 for v in (cm, cn, cw, cc)), (cm, cn, cw, cc)   # 核里系数是 UT(40)
        core.run(OP["PRIOR2"], self._prm(_20=org, _21=kk, _22=i1 | (i2 << 32), _23=int(round(cm)),
                                         _24=int(round(cn)), _25=int(round(cw)), _26=int(round(cc))))
        self.prior = True       # 产品只看 is None
        self._sel = None

    def _select(self):
        if self._sel is None:
            r = self.core.run(OP["SELECT"], self._prm())
            nv = self.nviews
            v = r[R_VIEW:R_VIEW + 2 * nv].astype(np.float64).reshape(nv, 2)
            Y = len(self.yaw_set)
            self._sel = (int(r[2]) / 2. ** 48, int(r[3]) / 2. ** 48, (v[:, 0] / 2. ** 48).reshape(-1, Y),
                         v[:, 1].reshape(-1, Y) * self.cell ** 2, r[R_VIEW:R_VIEW + 2 * nv].copy())
        return self._sel

    def mass_left(self):
        return self._select()[0]

    def unseen_mass(self):
        return self._select()[1]

    def gains(self, new_area=False):
        s = self._select()
        return (s[2].copy(), s[3].copy()) if new_area else s[2].copy()

    def apply_view(self, x, y, yaw, T_world_camera=None):
        assert T_world_camera is not None and self.has_camera()
        T = np.asarray(T_world_camera, float)
        ox, oy = self._origin(T[0, 3], T[1, 3])
        i, j = self._cell_ij(ox, oy)
        job = self._job(ox, oy, 0, (i - 16, j - 16), (i - 16, j - 16), self._forms(T, ox, oy))
        self.core.ddr[A_JOB1:A_JOB1 + 96] = job.view(np.uint8)
        r = self.core.run(OP["OBSERVE"], self._prm(_2=A_JOB1))
        self.nso = (int(r[1]) & 0xFFFF) - 31744
        self.views += 1
        self._sel = None
        return int(r[1]) >> 32

    def cellof_c(self, x, y):
        ox, oy = self._origin(x, y)
        job = self._job(ox, oy, 0, (0, 0), (0, 0), [(0, 0, 0, 0)] * 5)
        self.core.ddr[A_JOB1:A_JOB1 + 96] = job.view(np.uint8)
        r = self.core.run(OP["CELLOF"], self._prm(_2=A_JOB1))
        v = int(r[R_NODECELL])
        return None if not (v >> 32) & 1 else ((v >> 16) & 0xFFFF, v & 0xFFFF)

    def dump_state(self):
        self.core.run(OP["DUMP"], self._prm(_3=A_DUMP))
        return self.core.ddr[A_DUMP:A_DUMP + 16 * self.U].copy()


def make_fx(free_map, nodes, cfg, K, img, Tbc, core):
    fx = FX.__new__(FX)
    fx.core = None
    fx._cells = free_map.cells
    BeliefSearch.__init__(fx, free_map, nodes, cfg)
    fx.set_camera(K, img, Tbc)
    fx.core_attach(core)
    fx.core_init()
    return fx
