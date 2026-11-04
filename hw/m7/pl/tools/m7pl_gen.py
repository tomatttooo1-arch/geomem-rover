#!/usr/bin/env python3
"""Generate / cross-check the M7 PL overlay, driver register header and address plan.

Single source of truth for addresses and interrupt bits is bd/m7_bd.tcl
(``array set ::m7_addr`` / ``::m7_irq``). HLS register offsets come from the
exported IP drivers (``*_hw.h``). Subcommands:

  regs     --ip-repo DIR --out sw/m7pl_hls_regs.h
  overlay  --clk-hz HZ [--clk1-hz HZ] --bitname NAME --out overlay/m7_pl.dtso
  addrh    --out sw/m7pl_addrmap.h
  check    --build DIR --ip-repo DIR --dtbo FILE [--clk-hz HZ] [--clk1-hz HZ]

--clk1-hz (v3b, 2026-10-26): the build also uses pl_clk1 (compute domain);
the overlay then carries a clocking1 node (zynqmp_clk 72) at that rate.

``check`` compares, for every block: the tcl address plan, the address map the
Vivado build actually assigned (address_map.csv and the .hwh), the compiled
overlay (decompiled with dtc), and the driver headers; plus the interrupt
wiring in the .hwh against the overlay SPI numbers. Any mismatch -> exit 1.
Standard library only (the board's Python may be old).
"""
import argparse
import glob
import hashlib
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PL = os.path.dirname(HERE)
BD_TCL = os.path.join(PL, "bd", "m7_bd.tcl")
SPI_BASE = 89  # pl_ps_irq0[0] -> GIC SPI 89 on Zynq UltraScale+
# block -> (HLS top or None, BD cell name, overlay node name)
BLOCKS = {
    "sysinfo": (None, "sysinfo", "m7_sysinfo"),
    "frame": ("fpga_m7_frame_axi", "frame", "m7_frame"),
    "permit": (None, "permit", "m7_permit"),
    # v3fc (2026-10-26): watchdog (slot 4) and encoder (slot 7) removed from the design.
    "ar": ("fpga_m7_ar_overlay_axi", "ar", "m7_ar"),
    # E2 region check; cell "roi" (not "object_roi": check() matches
    # address segments by the prefix SEG_<cell>_, which "object" would share).
    "roi": ("fpga_m7_object_roi_axi", "roi", "m7_roi"),
    # v3 L2 (2026-10-26): rolling fusion core in slot 5 / HP3 / irq bit 4
    # (was RESERVED since 2026-09-26).
    "rolling": ("fpga_m7_rolling_axi", "rolling", "m7_rolling"),
}
# MERGE5A (2026-11-03): YOLOv8 detection post-processing core (DETPOST1), only in builds made with
# FPGA_M7_PL_DETPOST=1 (--detpost). Kept out of BLOCKS so m7pl_hls_regs.h and every older build's check
# stay exactly as before; its register offsets go to sw/m7pl_detpost_regs.h (subcommand detpost-regs).
DETPOST_BLOCKS = {"detpost": ("detpost_top", "detpost", "m7_detpost")}
DETPOST_REGS_H = os.path.join(PL, "sw", "m7pl_detpost_regs.h")
# MERGE7 (2026-11-04): pre-processing core (slot 11, irq 6) and planner_field_dual (slot 12, irq 1), only in
# builds made with FPGA_M7_PL_PREP=1 / FPGA_M7_PL_PLANNER=1 (--prep / --planner). Their IPs are not in the
# shared ip repo (s15/s18 swap = another IP path), so their headers are generated from --prep-ip /
# --planner-ip (an IP directory with drivers/, or an export.zip).
PREP_BLOCKS = {"prep": ("prep_top", "prep", "m7_prep")}
PLANNER_BLOCKS = {"planner": ("planner_field_dual", "planner", "m7_planner")}
EXTRA_REGS = {  # subcommand -> (block, top, header, title)
    "prep-regs": ("prep", "prep_top", os.path.join(PL, "sw", "m7pl_prep_regs.h"),
                  "M7PL_PREP_REGS_H", "Recognition pre-processing core (MERGE7, FPGA_M7_PL_PREP=1)"),
    "planner-regs": ("planner", "planner_field_dual", os.path.join(PL, "sw", "m7pl_planner_regs.h"),
                     "M7PL_PLANNER_REGS_H", "planner_field_dual (MERGE7, FPGA_M7_PL_PLANNER=1)"),
}
# MERGE9P (2026-11-04): search core (slot 13, pl_ps_irq1 bit 0), memory core (slot 14, pl_ps_irq1 bit 1) and
# FPGA_M7_PL_SRCH / _MEMK / _OBS (--srch / --memk / --obs, each with --<b>-ip = the IP the build used). Their HLS
# top names are taken from the IP (the one drivers/*/src/x*_hw.h), so the tool does not hard-code them.
NEW_BLOCKS = {"srch": (None, "srch", "m7_srch"), "memk": (None, "memk", "m7_memk"), "obs": (None, "obs", "m7_obs")}
EXTRA_REGS.update({
    "srch-regs": ("srch", None, os.path.join(PL, "sw", "m7pl_srch_regs.h"),
                  "M7PL_SRCH_REGS_H", "Probabilistic search core (MERGE9P, FPGA_M7_PL_SRCH=1)"),
    "memk-regs": ("memk", None, os.path.join(PL, "sw", "m7pl_memk_regs.h"),
                  "M7PL_MEMK_REGS_H", "Semantic memory core (MERGE9P, FPGA_M7_PL_MEMK=1)"),
    "obs-regs": ("obs", None, os.path.join(PL, "sw", "m7pl_obs_regs.h"),
                 "M7PL_OBS_REGS_H", "Observation-and-verification core (MERGE9P, FPGA_M7_PL_OBS=1)"),
})
# MERGE10 (2026-11-04): --memk-if bram = a MEMK4-type memk (ap_ctrl + one BRAM port) built with FPGA_M7_PL_MEMK_IF=bram:
# its control slave keeps the slot base with a smaller range, the AXI BRAM controller bc_memk sits at ::m7_addr(memk_bram)
# inside the same 64 KiB slot, and bm_memk (blk_mem_gen) joins the two (bd/m7_bd.tcl header). Check N6.
MEMK_BRAM_CELLS = ("bc_memk", "bm_memk")
SPI_BASE1 = 104  # pl_ps_irq1[0] -> GIC SPI 104 (MERGE6 section 2.1, common_proc.tcl)
RESERVED = ()
WINDOW = 0x10000
# DPU1 (2026-11-03): DPUCZDX8G (D0 design) on M_AXI_HPM0_LPD, outside the 64 KiB slot grid,
# 16 MiB at the DPU TRD address; its interrupt takes ar's pl_ps_irq0 bit 7 (GIC SPI 96).
# --dpu-clk-hz / --dpu2x-clk-hz come from the build's CLOCKS.txt (MMCM actual rates).
DPU_BASE = 0x8F000000
DPU_SIZE = 0x1000000
DPU_CELL = "dpu"


def blocks_of(a):
    """BLOCKS, without ar for a build made with FPGA_M7_PL_NO_AR=1 (AR2NANO 2026-11-02: car screen on
    the Nano; --no-ar). Without the flag: BLOCKS unchanged."""
    out = BLOCKS
    if getattr(a, "no_ar", False) or getattr(a, "dpu_clk_hz", None):
        out = {k: v for k, v in BLOCKS.items() if k != "ar"}
    if getattr(a, "detpost", False):
        out = dict(out, **DETPOST_BLOCKS)
    if getattr(a, "prep", False):
        out = dict(out, **PREP_BLOCKS)
    if getattr(a, "planner", False):
        out = dict(out, **PLANNER_BLOCKS)
    for b in ("srch", "memk", "obs"):   # MERGE9P
        if getattr(a, b, False):
            out = dict(out, **{b: NEW_BLOCKS[b]})
    if getattr(a, "obs", False):
        out = {k: v for k, v in out.items() if k != "roi"}
    return out


def ip_file(ip, pattern):
    """(name, bytes) of the one file matching the regex <pattern> in an IP directory or export.zip (MERGE9P)."""
    if os.path.isfile(ip):
        import zipfile
        with zipfile.ZipFile(ip) as z:
            names = [n for n in z.namelist() if re.fullmatch(pattern, n)]
            if len(names) != 1:
                sys.exit("expected one %s in %s, got %r" % (pattern, ip, names))
            return names[0], z.read(names[0])
    names = [os.path.relpath(os.path.join(d, f), ip) for d, _, fs in os.walk(ip) for f in fs]
    names = [n for n in names if re.fullmatch(pattern, n)]
    if len(names) != 1:
        sys.exit("expected one %s under %s, got %r" % (pattern, ip, names))
    return names[0], open(os.path.join(ip, names[0]), "rb").read()


def ip_facts(ip):
    """VLNV, AXI4 masters {busif: data width} and the control slave address width of an exported HLS IP (MERGE9P)."""
    x = ip_file(ip, r"component\.xml")[1].decode()
    vlnv = ":".join(re.search(r"<spirit:%s>([^<]+)</spirit:%s>" % (t, t), x).group(1)
                    for t in ("vendor", "library", "name", "version"))
    masters = {}
    for m in re.finditer(r"<spirit:modelParameter[^>]*>\s*<spirit:name>C_M_AXI_(\w+)_DATA_WIDTH</spirit:name>.*?"
                         r"<spirit:value[^>]*>(\d+)</spirit:value>", x, re.S):
        masters["m_axi_" + m.group(1).lower()] = int(m.group(2))
    aw = re.search(r"<spirit:modelParameter[^>]*>\s*<spirit:name>C_S_AXI_\w+_ADDR_WIDTH</spirit:name>.*?"
                   r"<spirit:value[^>]*>(\d+)</spirit:value>", x, re.S)
    return vlnv, masters, int(aw.group(1)) if aw else None


def tcl_arrays():
    text = open(BD_TCL).read()
    out = {}
    for name in ("m7_addr", "m7_irq", "m7_irq1"):
        m = re.search(r"array set ::%s \{(.*?)\n\}" % name, text, re.S)
        if not m:
            sys.exit("cannot find ::%s in %s" % (name, BD_TCL))
        toks = m.group(1).split()
        out[name] = {toks[i]: int(toks[i + 1], 0) for i in range(0, len(toks), 2)}
    return out["m7_addr"], out["m7_irq"]


def tcl_memk_ranges():
    """MERGE10: (ctrl range, block range) in bytes of memk in slot 14 when memk_if is bram."""
    text = open(BD_TCL).read()
    out = []
    for n in ("ctrl", "bram"):
        m = re.search(r"set ::m7_memk_%s_range (\d+)K" % n, text)
        if not m:
            sys.exit("cannot find ::m7_memk_%s_range in %s" % (n, BD_TCL))
        out.append(int(m.group(1)) * 1024)
    return tuple(out)


def tcl_irq1():
    """MERGE9P: {block: pl_ps_irq1 bit}."""
    text = open(BD_TCL).read()
    m = re.search(r"array set ::m7_irq1 \{(.*?)\n\}", text, re.S)
    toks = m.group(1).split() if m else []
    return {toks[i]: int(toks[i + 1], 0) for i in range(0, len(toks), 2)}


def hw_headers(ip_repo):
    found = {}
    for block, (top, _, _) in BLOCKS.items():
        if top is None:
            continue
        paths = glob.glob(os.path.join(ip_repo, top, "drivers", "*", "src", "x%s_hw.h" % top))
        if len(paths) != 1:
            sys.exit("expected one hw.h for %s under %s, got %r" % (top, ip_repo, paths))
        found[block] = paths[0]
    return found


def cmd_regs(a):
    lines = ["/* GENERATED by tools/m7pl_gen.py regs -- do not edit.",
             " * HLS s_axilite register offsets copied from the exported IP drivers.",
             " * Source headers (sha256):"]
    body = []
    for block, path in sorted(hw_headers(a.ip_repo).items()):
        data = open(path, "rb").read()
        lines.append(" *   %s %s" % (os.path.basename(path), hashlib.sha256(data).hexdigest()))
        body.append("/* ---- %s ---- */" % block)
        for ln in data.decode().splitlines():
            if ln.startswith("#define X"):
                body.append(ln)
        body.append("")
    lines += [" */", "#ifndef M7PL_HLS_REGS_H", "#define M7PL_HLS_REGS_H", ""]
    lines += body + ["#endif", ""]
    write(a.out, "\n".join(lines))


def cmd_detpost_regs(a):
    top = DETPOST_BLOCKS["detpost"][0]
    paths = glob.glob(os.path.join(a.ip_repo, top, "drivers", "*", "src", "x%s_hw.h" % top))
    if len(paths) != 1:
        sys.exit("expected one hw.h for %s under %s, got %r" % (top, a.ip_repo, paths))
    data = open(paths[0], "rb").read()
    lines = ["/* GENERATED by tools/m7pl_gen.py detpost-regs -- do not edit.",
             " * Detection post-processing core (MERGE5A, FPGA_M7_PL_DETPOST=1) s_axilite offsets",
             " * copied from the exported IP driver:",
             " *   %s %s" % (os.path.basename(paths[0]), hashlib.sha256(data).hexdigest()),
             " */", "#ifndef M7PL_DETPOST_REGS_H", "#define M7PL_DETPOST_REGS_H", ""]
    lines += [ln for ln in data.decode().splitlines() if ln.startswith("#define X")]
    lines += ["", "#endif", ""]
    write(a.out, "\n".join(lines))


def ip_hw_header(ip, top):
    """(file name, bytes) of x<top>_hw.h in an IP directory (drivers/*/src) or an export.zip.
    top None (MERGE9P new cores): the one x*_hw.h of the IP."""
    if top is None:
        n, d = ip_file(ip, r"drivers/[^/]+/src/x[^/]*_hw\.h")
        return os.path.basename(n), d
    if os.path.isfile(ip):
        import zipfile
        with zipfile.ZipFile(ip) as z:
            names = [n for n in z.namelist()
                     if re.fullmatch(r"drivers/[^/]+/src/x%s_hw\.h" % re.escape(top), n)]
            if len(names) != 1:
                sys.exit("expected one hw.h for %s in %s, got %r" % (top, ip, names))
            return os.path.basename(names[0]), z.read(names[0])
    paths = glob.glob(os.path.join(ip, "drivers", "*", "src", "x%s_hw.h" % top))
    if len(paths) != 1:
        sys.exit("expected one hw.h for %s under %s, got %r" % (top, ip, paths))
    return os.path.basename(paths[0]), open(paths[0], "rb").read()


def cmd_extra_regs(a, gen):
    block, top, _, guard, title = EXTRA_REGS[gen]
    name, data = ip_hw_header(a.ip, top)
    lines = ["/* GENERATED by tools/m7pl_gen.py %s -- do not edit." % gen,
             " * %s s_axilite offsets" % title,
             " * copied from the exported IP driver:",
             " *   %s %s" % (name, hashlib.sha256(data).hexdigest()),
             " */", "#ifndef %s" % guard, "#define %s" % guard, ""]
    lines += [ln for ln in data.decode().splitlines() if ln.startswith("#define X")]
    lines += ["", "#endif", ""]
    write(a.out, "\n".join(lines))


def cmd_addrh(a):
    addr, irq = tcl_arrays()
    lines = ["/* GENERATED by tools/m7pl_gen.py addrh from bd/m7_bd.tcl -- do not edit. */",
             "#ifndef M7PL_ADDRMAP_H", "#define M7PL_ADDRMAP_H", "",
             "#define M7PL_WINDOW_BYTES 0x%XU" % WINDOW]
    for k in sorted(addr):
        lines.append("#define M7PL_BASE_%s 0x%08XU%s" % (k.upper(), addr[k],
                     "  /* RESERVED, nothing attached */" if k in RESERVED else ""))
    for k in sorted(irq):
        lines.append("#define M7PL_IRQ_BIT_%s %d  /* GIC SPI %d */%s" % (
            k.upper(), irq[k], SPI_BASE + irq[k], "  /* RESERVED */" if k in RESERVED else ""))
    cr, br = tcl_memk_ranges()   # MERGE10: slot 14 split when memk_if is bram
    lines.append("#define M7PL_MEMK_BRAM_CTRL_BYTES 0x%XU  /* memk_if bram: core control range at M7PL_BASE_MEMK */" % cr)
    lines.append("#define M7PL_MEMK_BRAM_BYTES 0x%XU  /* memk_if bram: bc_memk block (cmd words 0-1023, res 1024-2047) at M7PL_BASE_MEMK_BRAM */" % br)
    irq1 = tcl_irq1()   # MERGE9P
    for k in sorted(irq1):
        lines.append("#define M7PL_IRQ1_BIT_%s %d  /* pl_ps_irq1, GIC SPI %d */" % (k.upper(), irq1[k], SPI_BASE1 + irq1[k]))
    lines += ["", "#endif", ""]
    write(a.out, "\n".join(lines))


def cmd_overlay(a):
    addr, irq = tcl_arrays()
    nodes = []
    for block, (_, _, node) in blocks_of(a).items():
        base = addr[block]
        irq_lines = ""
        spi = SPI_BASE + irq[block] if block in irq else None
        if block in NEW_BLOCKS and block in tcl_irq1():   # MERGE9P: srch / memk on pl_ps_irq1
            spi = SPI_BASE1 + tcl_irq1()[block]
        if spi is not None:
            irq_lines = ("\t\tinterrupt-parent = <&gic>;\n"
                         "\t\tinterrupts = <0 %d 4>;\n" % spi)
        nodes.append(
            "\t%s: %s@%x {\n"
            "\t\tcompatible = \"generic-uio\";\n"
            "\t\treg = <0x0 0x%08x 0x0 0x%x>;\n"
            "%s"
            "\t};\n" % (node, node, base, base, WINDOW, irq_lines))
    text = """// GENERATED by tools/m7pl_gen.py overlay from bd/m7_bd.tcl -- do not edit.
// Device-tree overlay for `xmutil loadapp` / dfx-mgr on KV260 (XRT_FLAT shell).
// fpga_full/afi0/clocking0 follow the layout of the Kria base-platform
// overlays (checked against a decompiled U19 KV260 pl.dtbo, a third-party
// package; the values here are ours). afi0: HP ports and HPM0/1_FPD 128-bit,
// matching the PS port widths in bd/m7_bd.tcl. clocking0 = pl_clk0.
// Blocks bind to generic-uio only when the kernel command line has
// uio_pdrv_genirq.of_id=generic-uio; the test program also works through
// /dev/mem without UIO.
/dts-v1/;
/plugin/;

&fpga_full {
\t#address-cells = <2>;
\t#size-cells = <2>;
\tfirmware-name = "%s";
\tresets = <&zynqmp_reset 116>, <&zynqmp_reset 117>, <&zynqmp_reset 118>, <&zynqmp_reset 119>;
};

&amba {
\t#address-cells = <2>;
\t#size-cells = <2>;
\tafi0: afi0 {
\t\tcompatible = "xlnx,afi-fpga";
\t\tconfig-afi = %s;
\t};
\tclocking0: clocking0 {
\t\tcompatible = "xlnx,fclk";
\t\t#clock-cells = <0>;
\t\tassigned-clocks = <&zynqmp_clk 71>;
\t\tassigned-clock-rates = <%d>;
\t\tclocks = <&zynqmp_clk 71>;
\t\tclock-output-names = "fabric_clk";
\t};
%s%s%s};
""" % (a.bitname, config_afi(a), a.clk_hz, clocking1(a), "".join(nodes), dpu_nodes(a))
    write(a.out, text)


def dpu_nodes(a):
    """DPU1: DPU node after the D0 RESULTS.md section 4 item 2 plan (xlnx_dpu kernel driver, Vivado
    flow): fixed-clock nodes for the PL MMCM outputs. The binding (compatible / clock-names /
    interrupt-names) is NOT verified against the board kernel; with no matching driver the node is
    inert. Without --dpu-clk-hz: nothing (output unchanged)."""
    if not getattr(a, "dpu_clk_hz", None):
        return ""
    _, irq = tcl_arrays()
    return ("\tdpu_clk_1x: dpu_clk_1x {\n"
            "\t\tcompatible = \"fixed-clock\";\n"
            "\t\t#clock-cells = <0>;\n"
            "\t\tclock-frequency = <%d>;\n"
            "\t};\n"
            "\tdpu_clk_2x: dpu_clk_2x {\n"
            "\t\tcompatible = \"fixed-clock\";\n"
            "\t\t#clock-cells = <0>;\n"
            "\t\tclock-frequency = <%d>;\n"
            "\t};\n"
            "\tm7_dpu: dpuczdx8g@%x {\n"
            "\t\tcompatible = \"xlnx,dpuczdx8g-4.1\";\n"
            "\t\treg = <0x0 0x%08x 0x0 0x%x>;\n"
            "\t\tinterrupt-parent = <&gic>;\n"
            "\t\tinterrupts = <0 %d 4>;\n"
            "\t\tinterrupt-names = \"dpu0_interrupt\";\n"
            "\t\tclocks = <&zynqmp_clk 71>, <&dpu_clk_1x>, <&dpu_clk_2x>;\n"
            "\t\tclock-names = \"s_axi_aclk\", \"m_axi_dpu_aclk\", \"dpu_2x_clk\";\n"
            "\t};\n" % (a.dpu_clk_hz, a.dpu2x_clk_hz, DPU_BASE, DPU_BASE, DPU_SIZE, SPI_BASE + irq["ar"]))


# xlnx,afi-fpga config-afi register ids per PS slave port SAXIGP<n> (read, write) and the fabric
# width code (0 = 128, 1 = 64, 2 = 32 bit): Vitis 2022.2 system-device-tree-xlnx device_tree.tcl
# gen_afi_node / common_proc.tcl get_afi_val (SAXIGP0..6 -> ids 0/1 .. 12/13).
AFI_CODE = {128: 0, 64: 1, 32: 2}
# MERGE4 (2026-11-03): with --mem-direct, HP0 (SAXIGP2) and HP1 (SAXIGP3) are 32 bit, HP3 (SAXIGP5)
# 64 bit (bd/m7_bd.tcl mem_direct). Every other port stays 128 bit.
MEM_DIRECT_WIDTHS = {2: 32, 3: 32, 5: 64}


def afi_widths(a):
    w = {n: 128 for n in range(7)}
    if getattr(a, "mem_direct", False):
        w.update(MEM_DIRECT_WIDTHS)
    if getattr(a, "detpost", False) and not getattr(a, "lpd", False):
        w[5] = 128      # MERGE5A: HP3 shared by rolling (64) and detpost (128) via sc_hp3 at 128 bit
    # MERGE7 --lpd: detpost (and prep / planner) on S_AXI_LPD (SAXIGP6, 128 bit, ids 12/13 = 0: the same
    # text as before); HP3 keeps the rolling core alone (64 bit with --mem-direct, as m4).
    if getattr(a, "obs", False) and getattr(a, "obs_port", "hp1") == "hp1":
        # MERGE9P: HP1 (SAXIGP3) carries obs's one m_axi master at its width, read from the IP (not from the build)
        ms = ip_facts(a.obs_ip)[1]
        if len(ms) != 1:
            sys.exit("--obs (hp1): the obs IP has masters %r, want exactly one" % ms)
        w[3] = list(ms.values())[0]
    return w


def config_afi(a):
    """Without --mem-direct: the same text as before (all ports 128 bit, HPM0/1_FPD 0xa00)."""
    w = afi_widths(a)
    pairs = []
    for n in range(7):
        pairs += ["<%d %d>" % (2 * n, AFI_CODE[w[n]]), "<%d %d>" % (2 * n + 1, AFI_CODE[w[n]])]
    pairs += ["<14 0xa00>", "<15 0x000>"]
    return ", ".join(pairs)


def clocking1(a):
    if not getattr(a, "clk1_hz", None):
        return ""
    return ("\tclocking1: clocking1 {\n"
            "\t\tcompatible = \"xlnx,fclk\";\n"
            "\t\t#clock-cells = <0>;\n"
            "\t\tassigned-clocks = <&zynqmp_clk 72>;\n"
            "\t\tassigned-clock-rates = <%d>;\n"
            "\t\tclocks = <&zynqmp_clk 72>;\n"
            "\t\tclock-output-names = \"fabric_clk1\";\n"
            "\t};\n" % a.clk1_hz)


def write(path, text):
    with open(path, "w") as f:
        f.write(text)
    print("wrote", path)


def parse_dtbo(dtbo):
    dts = subprocess.run(["dtc", "-q", "-I", "dtb", "-O", "dts", dtbo],
                         check=True, stdout=subprocess.PIPE).stdout.decode()
    nodes = {}
    for m in re.finditer(r"(m7_\w+)@([0-9a-f]+) \{(.*?)\};", dts, re.S):
        name, unit, body = m.group(1), int(m.group(2), 16), m.group(3)
        reg = re.search(r"reg = <([^>]*)>", body)
        cells = [int(x, 0) for x in reg.group(1).split()]
        base = (cells[0] << 32) | cells[1]
        size = (cells[2] << 32) | cells[3]
        spi = None
        it = re.search(r"interrupts = <([^>]*)>", body)
        if it:
            spi = int(it.group(1).split()[1], 0)
        nodes[name] = (unit, base, size, spi)
    m = re.search(r"dpuczdx8g@([0-9a-f]+) \{(.*?)\};", dts, re.S)
    if m:
        cells = [int(x, 0) for x in re.search(r"reg = <([^>]*)>", m.group(2)).group(1).split()]
        it = re.search(r"interrupts = <([^>]*)>", m.group(2))
        nodes["dpu"] = (int(m.group(1), 16), (cells[0] << 32) | cells[1], (cells[2] << 32) | cells[3],
                        int(it.group(1).split()[1], 0) if it else None)
    fw = re.search(r'firmware-name = "([^"]+)"', dts)
    afi = re.search(r"config-afi = <([^;]*)>;", dts)
    if afi:     # dtc may print the pairs as one cell list: <0x00 0x00 0x01 0x00 ...>
        cells = [int(x, 0) for x in re.sub(r"[<>,]", " ", afi.group(1)).split()]
        nodes["afi"] = dict(zip(cells[0::2], cells[1::2]))
    rates = {}
    for m in re.finditer(r"clocking(\d+) \{(.*?)\};", dts, re.S):
        r = re.search(r"assigned-clock-rates = <([^>]*)>", m.group(2))
        c = re.search(r"assigned-clocks = <\S+ (\S+)>", m.group(2))
        rates[int(m.group(1))] = (int(r.group(1), 0) if r else None, int(c.group(1), 0) if c else None)
    return nodes, fw.group(1) if fw else None, rates


def parse_hwh(build):
    paths = glob.glob(os.path.join(build, "*.hwh"))
    if len(paths) != 1:
        sys.exit("expected one .hwh in %s, got %r" % (build, paths))
    root = ET.parse(paths[0]).getroot()
    ranges = {}
    for mod in root.iter("MODULE"):
        if mod.get("INSTANCE") != "ps":
            continue
        for mr in mod.iter("MEMRANGE"):
            if mr.get("MASTERBUSINTERFACE") == "M_AXI_HPM0_FPD":
                ranges[mr.get("INSTANCE")] = (int(mr.get("BASEVALUE"), 0),
                                              int(mr.get("HIGHVALUE"), 0))
    # interrupt wiring: which signal drives irq_concat/In<k>
    irq_src = {}
    for mod in root.iter("MODULE"):
        if mod.get("INSTANCE") != "irq_concat":
            continue
        for port in mod.iter("PORT"):
            m = re.fullmatch(r"In(\d+)", port.get("NAME", ""))
            if not m:
                continue
            for con in port.iter("CONNECTION"):
                irq_src[int(m.group(1))] = con.get("INSTANCE")
    return ranges, irq_src


def parse_hwh_saxi(build):
    """{n: data width} for every enabled PS slave port SAXIGP<n> in the .hwh (MERGE4)."""
    root = ET.parse(glob.glob(os.path.join(build, "*.hwh"))[0]).getroot()
    for mod in root.iter("MODULE"):
        if mod.get("INSTANCE") != "ps":
            continue
        p = {q.get("NAME"): q.get("VALUE") for q in mod.iter("PARAMETER")}
        return {n: int(p["PSU__SAXIGP%d__DATA_WIDTH" % n]) for n in range(7)
                if p.get("PSU__USE__S_AXI_GP%d" % n) == "1"}
    return {}


def parse_hwh_dma(build):
    """{(instance, master interface): [(slave interface, address block, base, high)]} for every PS
    memory range a PL master maps in the .hwh (MERGE7)."""
    root = ET.parse(glob.glob(os.path.join(build, "*.hwh"))[0]).getroot()
    out = {}
    for mod in root.iter("MODULE"):
        inst = mod.get("INSTANCE")
        for mr in mod.iter("MEMRANGE"):
            if mr.get("INSTANCE") != "ps" or inst == "ps":
                continue
            out.setdefault((inst, mr.get("MASTERBUSINTERFACE")), []).append(
                (mr.get("SLAVEBUSINTERFACE"), mr.get("ADDRESSBLOCK"),
                 int(mr.get("BASEVALUE"), 0), int(mr.get("HIGHVALUE"), 0)))
    return out


def parse_hwh_slot_map(build):
    root = ET.parse(glob.glob(os.path.join(build, "*.hwh"))[0]).getroot()
    for mod in root.iter("MODULE"):
        if mod.get("INSTANCE") != "sysinfo":
            continue
        for q in mod.iter("PARAMETER"):
            if q.get("NAME") == "SLOT_MAP":
                return int(q.get("VALUE"), 0)
    return None


def parse_hwh_irq1(build):
    """MERGE9P: {bit: instance driving irq_concat1/In<bit>} (pl_ps_irq1)."""
    root = ET.parse(glob.glob(os.path.join(build, "*.hwh"))[0]).getroot()
    out = {}
    for mod in root.iter("MODULE"):
        if mod.get("INSTANCE") != "irq_concat1":
            continue
        for port in mod.iter("PORT"):
            m = re.fullmatch(r"In(\d+)", port.get("NAME", ""))
            if m:
                for con in port.iter("CONNECTION"):
                    out[int(m.group(1))] = con.get("INSTANCE")
    return out


def parse_hwh_modules(build):
    """MERGE9P: {instance: (VLNV, {busif: (type, data width)})}."""
    root = ET.parse(glob.glob(os.path.join(build, "*.hwh"))[0]).getroot()
    out = {}
    for mod in root.iter("MODULE"):
        buses = {}
        for b in mod.iter("BUSINTERFACE"):
            ps = {q.get("NAME"): q.get("VALUE") for q in b.iter("PARAMETER")}
            buses[b.get("NAME")] = (b.get("TYPE"), int(ps["DATA_WIDTH"]) if (ps.get("DATA_WIDTH") or "").isdigit() else None,
                                    b.get("VLNV") or "", ps, b.get("BUSNAME"))   # MERGE10: + bus VLNV, params, bus
        out[mod.get("INSTANCE")] = (mod.get("VLNV"), buses)
    return out


def check_new_cores(a, blocks, irq1, irq1_src, report):
    """MERGE9P checks, one line each in the report:
    N1 presence: every new core expected is in the .hwh with the VLNV of the IP the build used; obs and roi never
       both; a new core not expected must be absent.
    N2 DMA mapping: every AXI4 master of a new core maps exactly one segment: S_AXI_LPD LPD_DDR_LOW 0-0x7fffffff
       (obs with --obs-port hp1: S_AXI_HP1_FPD HP1_DDR_LOW 0-0x7fffffff); the master set equals the IP's.
    N3 widths: each new master's width in the .hwh = the IP's; srch masters 128 bit (HWDESIGN_RULES); obs hp1:
       SAXIGP3 width = obs width; obs lpd: SAXIGP3 off; SAXIGP6 128.
    N4 address windows: no two PS-master address ranges overlap (HPM0_FPD slots and the DPU window); every new core's
       control address space (2^ADDR_WIDTH of the IP) fits its 64 KiB slot.
    N5 interrupts: pl_ps_irq1 bits are driven by the expected cores only, the rest by irq_zero (or absent).
    (SLOT_MAP of the new slots is covered by the generic SLOT_MAP check above.)"""
    errs = []
    mods = parse_hwh_modules(a.build)
    dma = parse_hwh_dma(a.build)
    saxi = parse_hwh_saxi(a.build)
    # N1
    n1 = []
    for b in ("srch", "memk", "obs"):
        cell = NEW_BLOCKS[b][1]
        if b in blocks:
            ip = getattr(a, b + "_ip", None)
            if not ip:
                errs.append("N1: --%s needs --%s-ip" % (b, b)); continue
            vlnv = ip_facts(ip)[0]
            got = mods.get(cell, (None,))[0]
            n1.append("%s=%s" % (b, got))
            if got != vlnv:
                errs.append("N1: %s in hwh has VLNV %s, the IP the build used is %s" % (cell, got, vlnv))
        elif cell in mods:
            errs.append("N1: %s is in the hwh but the check was run without --%s" % (cell, b))
    if "obs" in mods and "roi" in mods:
        errs.append("N1: both obs and roi in the hwh")
    if "obs" in blocks and "roi" in mods:
        errs.append("N1: --obs but roi is still in the hwh")
    report.append("N1 new cores present: %s -> %s" % (" ".join(n1) or "none", "MISMATCH" if [e for e in errs if e.startswith("N1")] else "OK"))
    # N2 / N3
    n2 = []
    obs_hp1 = getattr(a, "obs_port", "hp1") == "hp1"
    for b in ("srch", "memk", "obs"):
        if b not in blocks or not getattr(a, b + "_ip", None):
            continue
        cell = NEW_BLOCKS[b][1]
        _, ipm, aw = ip_facts(getattr(a, b + "_ip"))
        hw_m = {k: v[1] for k, v in mods.get(cell, (None, {}))[1].items()
                if v[0] == "MASTER" and ":bram" not in v[2]}   # MERGE10: a BRAM master port is checked in N6
        if set(hw_m) != set(ipm):
            errs.append("N2: %s masters in hwh %s != IP %s" % (cell, sorted(hw_m), sorted(ipm)))
        if b in ("srch", "obs") and not ipm:
            errs.append("N2: %s has no m_axi master" % cell)
        want = ("S_AXI_HP1_FPD", "HP1_DDR_LOW", 0, 0x7FFFFFFF) if (b == "obs" and obs_hp1) else ("S_AXI_LPD", "LPD_DDR_LOW", 0, 0x7FFFFFFF)
        for mbi in sorted(ipm):
            m = dma.get((cell, mbi), [])
            n2.append("%s/%s=%s" % (cell, mbi, ",".join("%s:%s@0x%x-0x%x" % x for x in m) or None))
            if m != [want]:
                errs.append("N2: %s/%s maps %s (want only %s %s 0-0x7fffffff)" % (cell, mbi, m, want[0], want[1]))
            if hw_m.get(mbi) != ipm[mbi]:
                errs.append("N3: %s/%s width %s in hwh, IP says %s" % (cell, mbi, hw_m.get(mbi), ipm[mbi]))
            if b == "srch" and ipm[mbi] != 128:
                errs.append("N3: srch/%s is %s bit, the shell rule is one 128-bit m_axi" % (mbi, ipm[mbi]))
        if b == "obs":
            if obs_hp1:
                w = list(ipm.values())[0] if len(ipm) == 1 else None
                if saxi.get(3) != w:
                    errs.append("N3: obs on hp1: SAXIGP3 width %s, obs m_axi %s" % (saxi.get(3), w))
            elif 3 in saxi:
                errs.append("N3: obs on lpd but SAXIGP3 (HP1) still enabled at %s" % saxi[3])
        if aw is not None and (1 << aw) > WINDOW:
            errs.append("N4: %s control space 2^%d bytes > the 64 KiB slot" % (cell, aw))
        n2.append("%s:ctrl_aw=%s" % (cell, aw))
    if [b for b in ("srch", "memk", "obs") if b in blocks] and getattr(a, "lpd", False) and saxi.get(6) != 128:
        errs.append("N3: SAXIGP6 width %s, want 128" % saxi.get(6))
    report.append("N2/N3 new masters: %s -> %s" % (" ".join(n2) or "none",
                  "MISMATCH" if [e for e in errs if e[:2] in ("N2", "N3")] else "OK"))
    # N4: pairwise overlap of every PS-master address range
    root = ET.parse(glob.glob(os.path.join(a.build, "*.hwh"))[0]).getroot()
    rs = []
    for mod in root.iter("MODULE"):
        if mod.get("INSTANCE") != "ps":
            continue
        for mr in mod.iter("MEMRANGE"):
            if mr.get("MASTERBUSINTERFACE") in ("M_AXI_HPM0_FPD", "M_AXI_HPM0_LPD", "M_AXI_HPM1_FPD"):
                rs.append((int(mr.get("BASEVALUE"), 0), int(mr.get("HIGHVALUE"), 0), mr.get("INSTANCE")))
    rs.sort()
    for (lo1, hi1, i1), (lo2, hi2, i2) in zip(rs, rs[1:]):
        if lo2 <= hi1:
            errs.append("N4: address ranges overlap: %s 0x%x-0x%x and %s 0x%x-0x%x" % (i1, lo1, hi1, i2, lo2, hi2))
    report.append("N4 address windows: %d PS-master ranges, overlap-free -> %s" % (
        len(rs), "MISMATCH" if [e for e in errs if e.startswith("N4")] else "OK"))
    # N5: pl_ps_irq1 wiring
    for k in range(8):
        src = irq1_src.get(k)
        expect = [b for b, bit in irq1.items() if bit == k]
        if expect and src != NEW_BLOCKS[expect[0]][1]:
            errs.append("N5: irq1 bit %d driven by %s, expected %s" % (k, src, expect[0]))
        if not expect and src not in (None, "irq_zero"):
            errs.append("N5: irq1 bit %d unexpectedly driven by %s" % (k, src))
    report.append("N5 pl_ps_irq1: %s -> %s" % (" ".join("%d:%s" % kv for kv in sorted(irq1_src.items())) or "absent",
                  "MISMATCH" if [e for e in errs if e.startswith("N5")] else "OK"))
    errs += check_memk_bram(a, blocks, mods, report)   # MERGE10
    return errs


def ip_master_aw(ip):
    """MERGE10: {m_axi_<bundle>: ADDR_WIDTH} of an exported HLS IP."""
    x = ip_file(ip, r"component\.xml")[1].decode()
    return {"m_axi_" + m.group(1).lower(): int(m.group(2)) for m in re.finditer(
        r"<spirit:modelParameter[^>]*>\s*<spirit:name>C_M_AXI_(\w+)_ADDR_WIDTH</spirit:name>.*?"
        r"<spirit:value[^>]*>(\d+)</spirit:value>", x, re.S)}


def ip_bram_ports(ip):
    """MERGE10: {busif: MEM_WIDTH} of the BRAM master ports in an exported HLS IP's component.xml."""
    x = ip_file(ip, r"component\.xml")[1].decode()
    out = {}
    for m in re.finditer(r"<spirit:busInterface>(.*?)</spirit:busInterface>", x, re.S):
        b = m.group(1)
        if 'spirit:name="bram"' in b and "<spirit:master/>" in b:
            w = re.search(r"<spirit:name>MEM_WIDTH</spirit:name>\s*<spirit:value[^>]*>(\d+)<", b)
            out[re.search(r"<spirit:name>([^<]+)</spirit:name>", b).group(1)] = int(w.group(1)) if w else None
    return out


def check_memk_bram(a, blocks, mods, report):
    """MERGE10 N6 (memk_if bram), one report line:
    N6a presence: the memk IP has exactly one BRAM master port iff --memk-if bram; bc_memk (axi_bram_ctrl: AXI4LITE,
        32 bit, single port, no ECC) and bm_memk (blk_mem_gen, true dual port) are in the hwh iff --memk-if bram.
    N6b windows: memk's control range = [slot, slot + ctrl range), bc_memk's = [::m7_addr(memk_bram), + block range),
        both inside slot 14 and disjoint (the global pairwise overlap check is N4).
    N6c port: memk's BRAM port is 32 bit in the hwh and the IP; bm_memk is 32 bit on both ports and 2048 words deep
        (= the block range / 4: cmd 0-1023, res 1024-2047); memk/<port> and bm_memk/BRAM_PORTB share one bus,
        bc_memk/BRAM_PORTA and bm_memk/BRAM_PORTA share another."""
    errs = []
    want_b = "memk" in blocks and getattr(a, "memk_if", "lite") == "bram"
    have = [c for c in MEMK_BRAM_CELLS if c in mods]
    ipb = ip_bram_ports(a.memk_ip) if ("memk" in blocks and getattr(a, "memk_ip", None)) else {}
    if not want_b:
        if have:
            errs.append("N6: %s in the hwh but the check was run without --memk-if bram" % have)
        if ipb:
            errs.append("N6: the memk IP has BRAM port(s) %s (MEMK4-type) but the check was run without --memk-if bram" % sorted(ipb))
        if "memk" in blocks or have:
            report.append("N6 memk BRAM port: memk_if lite -> %s" % ("MISMATCH" if errs else "OK"))
        return errs
    addr, _ = tcl_arrays()
    ctrl_r, blk_r = tcl_memk_ranges()
    # N6a
    if len(ipb) != 1:
        errs.append("N6: the memk IP has %d BRAM master ports %s, want 1" % (len(ipb), sorted(ipb)))
    for c, vl in (("bc_memk", "xilinx.com:ip:axi_bram_ctrl:"), ("bm_memk", "xilinx.com:ip:blk_mem_gen:")):
        if c not in mods:
            errs.append("N6: BRAM path cell %s missing from the hwh" % c)
        elif not (mods[c][0] or "").startswith(vl):
            errs.append("N6: %s is %s, want %s*" % (c, mods[c][0], vl))
    root = ET.parse(glob.glob(os.path.join(a.build, "*.hwh"))[0]).getroot()
    par = {}
    for mod in root.iter("MODULE"):
        if mod.get("INSTANCE") in MEMK_BRAM_CELLS:
            pe = mod.find("PARAMETERS")
            par[mod.get("INSTANCE")] = {q.get("NAME"): q.get("VALUE") for q in (pe if pe is not None else [])}
    bc, bm = par.get("bc_memk", {}), par.get("bm_memk", {})
    if "bc_memk" in mods:
        for k, v in (("C_S_AXI_PROTOCOL", "AXI4LITE"), ("C_S_AXI_DATA_WIDTH", "32"), ("C_SINGLE_PORT_BRAM", "1"),
                     ("C_ECC", "0")):
            if bc.get(k) != v:
                errs.append("N6: bc_memk %s = %s, want %s" % (k, bc.get(k), v))
    caw = ip_facts(a.memk_ip)[2] if getattr(a, "memk_ip", None) else None
    if caw is not None and (1 << caw) > ctrl_r:
        errs.append("N6: memk control space 2^%d bytes > its 0x%x range in slot 14 (bc_memk follows)" % (caw, ctrl_r))
    # N6b
    hr, _ = parse_hwh(a.build)
    lo_c, lo_b, slot = addr["memk"], addr["memk_bram"], addr["memk"]
    for inst, lo, rng in (("memk", lo_c, ctrl_r), ("bc_memk", lo_b, blk_r)):
        got = hr.get(inst)
        if got != (lo, lo + rng - 1):
            errs.append("N6: %s HPM0 range %s, want 0x%x-0x%x" % (
                inst, "0x%x-0x%x" % got if got else None, lo, lo + rng - 1))
        if not (slot <= lo and lo + rng <= slot + WINDOW):
            errs.append("N6: %s plan 0x%x+0x%x is outside slot 14 0x%x+0x%x" % (inst, lo, rng, slot, WINDOW))
    if "memk" in hr and "bc_memk" in hr:
        (a0, a1), (b0, b1) = hr["memk"], hr["bc_memk"]
        if a0 <= b1 and b0 <= a1:
            errs.append("N6: memk 0x%x-0x%x and bc_memk 0x%x-0x%x overlap" % (a0, a1, b0, b1))
        for inst, (x0, x1) in (("memk", hr["memk"]), ("bc_memk", hr["bc_memk"])):
            if not (slot <= x0 and x1 < slot + WINDOW):
                errs.append("N6: %s hwh range 0x%x-0x%x leaves slot 14" % (inst, x0, x1))
    # N6c
    mb = {k: v for k, v in mods.get("memk", (None, {}))[1].items() if v[0] in ("MASTER", "INITIATOR") and ":bram" in v[2]}
    if len(mb) != 1:
        errs.append("N6: memk has %d BRAM master ports in the hwh %s, want 1" % (len(mb), sorted(mb)))
    for k, v in mb.items():
        w = v[3].get("MEM_WIDTH")
        if w != "32" or ipb.get(k) != 32:
            errs.append("N6: memk/%s width hwh %s, IP %s, want 32" % (k, w, ipb.get(k)))
    depth = blk_r // 4
    if "bm_memk" in mods:
        for k, v in (("C_MEM_TYPE", "2"), ("C_WRITE_WIDTH_A", "32"), ("C_READ_WIDTH_A", "32"),
                     ("C_WRITE_WIDTH_B", "32"), ("C_READ_WIDTH_B", "32"),
                     ("C_WRITE_DEPTH_A", str(depth)), ("C_WRITE_DEPTH_B", str(depth))):
            if bm.get(k) != v:
                errs.append("N6: bm_memk %s = %s, want %s (2048 x 32 true dual port)" % (k, bm.get(k), v))
    bus = lambda c, n: (mods.get(c, (None, {}))[1].get(n) or (None,) * 5)[4]
    pb = bus("memk", list(mb)[0]) if len(mb) == 1 else None
    if pb is None or pb != bus("bm_memk", "BRAM_PORTB"):
        errs.append("N6: memk BRAM port bus %s != bm_memk/BRAM_PORTB bus %s" % (pb, bus("bm_memk", "BRAM_PORTB")))
    if bus("bc_memk", "BRAM_PORTA") is None or bus("bc_memk", "BRAM_PORTA") != bus("bm_memk", "BRAM_PORTA"):
        errs.append("N6: bc_memk/BRAM_PORTA bus %s != bm_memk/BRAM_PORTA bus %s" % (
            bus("bc_memk", "BRAM_PORTA"), bus("bm_memk", "BRAM_PORTA")))
    report.append("N6 memk BRAM port: memk ctrl %s, bc_memk %s, bm_memk %sx%s type %s, ports %s -> %s" % (
        "0x%x-0x%x" % hr["memk"] if "memk" in hr else None, "0x%x-0x%x" % hr["bc_memk"] if "bc_memk" in hr else None,
        bm.get("C_WRITE_DEPTH_A"), bm.get("C_WRITE_WIDTH_A"), bm.get("C_MEM_TYPE"),
        {k: v[3].get("MEM_WIDTH") for k, v in mb.items()}, "MISMATCH" if errs else "OK"))
    return errs


def cmd_check(a):
    addr, irq = tcl_arrays()
    errors = []
    csv = {}
    for ln in open(os.path.join(a.build, "address_map.csv")).read().splitlines()[1:]:
        space, seg, off, rng = ln.split(",")
        if not off:  # slave-side rows (no offset) describe the slave, not a mapping
            continue
        csv.setdefault(space, []).append((seg, int(off, 0), int(rng, 0) if rng.isdigit() else rng))
    hwh_ranges, irq_src = parse_hwh(a.build)
    nodes, fw, rates = parse_dtbo(a.dtbo)
    rate = rates.get(0, (None, None))[0]
    rate1 = rates.get(1, (None, None))[0]
    report = []
    blocks = blocks_of(a)
    if "ar" not in blocks:
        report.append("no_ar: ar block, its slot 8 and irq bit 7 are expected ABSENT")
    if "detpost" not in blocks:
        report.append("no detpost: slot 10 and irq bit 3 are expected ABSENT")
    irq = {b: bit for b, bit in irq.items() if b in blocks}      # absent blocks' irq bits must be tied 0
    irq1 = {b: bit for b, bit in tcl_irq1().items() if b in blocks}   # MERGE9P
    irq1_src = parse_hwh_irq1(a.build)
    for block, (top, cell, node) in blocks.items():
        want = addr[block]
        hr = hwh_ranges.get(cell)
        dn = nodes.get(node)
        segs = [s for s in csv.get("Data", []) if s[0].startswith("SEG_%s_" % cell)]
        span = WINDOW
        if block == "memk" and getattr(a, "memk_if", "lite") == "bram":
            span = tcl_memk_ranges()[0]   # MERGE10: the core's control range; bc_memk is checked in N6
        ok = (hr is not None and hr[0] == want and hr[1] - hr[0] + 1 == span and
              dn is not None and dn[0] == want and dn[1] == want and dn[2] == WINDOW and
              len(segs) == 1 and segs[0][1] == want)
        spi_ok = True
        if block in irq:
            spi_ok = (dn is not None and dn[3] == SPI_BASE + irq[block] and
                      irq_src.get(irq[block]) == cell)
        elif block in NEW_BLOCKS and block in irq1:   # MERGE9P: pl_ps_irq1
            spi_ok = (dn is not None and dn[3] == SPI_BASE1 + irq1[block] and
                      irq1_src.get(irq1[block]) == cell)
        report.append("%-9s plan=0x%08x hwh=%s csv=%s dtbo=%s spi=%s irq_src=%s -> %s" % (
            block, want, "0x%08x-0x%08x" % hr if hr else None,
            "0x%08x" % segs[0][1] if len(segs) == 1 else segs,
            "0x%08x/0x%x" % (dn[1], dn[2]) if dn else None,
            dn[3] if dn else None, irq_src.get(irq.get(block, -1)) if block in irq else irq1_src.get(irq1.get(block, -1)),
            "OK" if ok and spi_ok else "MISMATCH"))
        if not (ok and spi_ok):
            errors.append(block)
    # Nothing may be mapped in the reserved slot, and every HPM0 range is known.
    for inst, (lo, hi) in hwh_ranges.items():
        if inst == "bc_memk" and "memk" in blocks and getattr(a, "memk_if", "lite") == "bram":
            continue   # MERGE10: checked in N6
        if inst not in [c for _, c, _ in blocks.values()]:
            errors.append("unexpected HPM0 slave %s" % inst)
        for r in RESERVED:
            if lo <= addr[r] <= hi:
                errors.append("reserved %s slot occupied by %s" % (r, inst))
    dpu = bool(getattr(a, "dpu_clk_hz", None))
    if dpu:
        _, irq_all = tcl_arrays()
        dbit = irq_all["ar"]
        dn = nodes.get("dpu")
        segs = [s for s in csv.get("Data", []) if s[0].startswith("SEG_%s_" % DPU_CELL)]
        ok = (dn is not None and dn[0] == DPU_BASE and dn[1] == DPU_BASE and dn[2] == DPU_SIZE and
              dn[3] == SPI_BASE + dbit and irq_src.get(dbit) == DPU_CELL and
              len(segs) == 1 and segs[0][1] == DPU_BASE and int(str(segs[0][2]), 0) == DPU_SIZE)
        report.append("%-9s plan=0x%08x/0x%x csv=%s dtbo=%s spi=%s irq_src(bit %d)=%s -> %s" % (
            "dpu", DPU_BASE, DPU_SIZE, ["0x%08x/%s" % (s[1], s[2]) for s in segs],
            "0x%08x/0x%x" % (dn[1], dn[2]) if dn else None, dn[3] if dn else None, dbit,
            irq_src.get(dbit), "OK" if ok else "MISMATCH"))
        if not ok:
            errors.append("dpu")
        dspaces = [sp for sp in csv if sp.startswith("DPU0_M_AXI")]
        report.append("dpu DMA spaces: %s" % sorted(dspaces))
        if sorted(dspaces) != ["DPU0_M_AXI_DATA0", "DPU0_M_AXI_DATA1", "DPU0_M_AXI_INSTR"]:
            errors.append("dpu DMA spaces %s" % dspaces)
        for sp in dspaces:
            for seg, off, rng in csv[sp]:
                if "DDR_LOW" not in seg or off != 0:
                    errors.append("DMA space %s maps %s @0x%x" % (sp, seg, off))
        arch = os.path.join(a.build, "arch.json")
        report.append("dpu arch.json: %s" % (open(arch).read().strip() if os.path.exists(arch) else "MISSING"))
        if not os.path.exists(arch):
            errors.append("arch.json missing")
    for k in range(8):
        src = irq_src.get(k)
        expect = [b for b, bit in irq.items() if bit == k and b not in RESERVED]
        if dpu and k == irq_all["ar"]:
            continue  # checked above (driven by the DPU)
        if expect and src != blocks[expect[0]][1]:
            errors.append("irq bit %d driven by %s, expected %s" % (k, src, expect))
        if not expect and src not in (None, "irq_zero"):
            errors.append("irq bit %d unexpectedly driven by %s" % (k, src))
    # DMA masters: only low DDR.
    for space, segs in csv.items():
        if space.startswith("Data_m_axi"):
            for seg, off, rng in segs:
                if "DDR_LOW" not in seg or off != 0:
                    errors.append("DMA space %s maps %s @0x%x" % (space, seg, off))
    if a.clk_hz is not None and rate != a.clk_hz:
        errors.append("overlay clock %s != build clock %s" % (rate, a.clk_hz))
    clk1 = getattr(a, "clk1_hz", None)
    if clk1:
        if rates.get(1) != (clk1, 72):
            errors.append("overlay clocking1 %s != (build clk1 %s, zynqmp_clk 72)" % (rates.get(1), clk1))
    elif 1 in rates:
        errors.append("overlay has clocking1 %s but the build has no pl_clk1" % (rates[1],))
    report.append("overlay firmware-name=%s clock=%s clock1=%s" % (fw, rate, rate1))
    # MERGE7: with --lpd, S_AXI_LPD (SAXIGP6) must be on at 128 bit and every master on it (detpost, prep,
    # planner) must map exactly ps LPD_DDR_LOW at 0; without --lpd the port must be off.
    saxi6 = parse_hwh_saxi(a.build).get(6)
    if getattr(a, "lpd", False):
        lpd_masters = {"detpost": ["m_axi_gmem"], "prep": ["m_axi_gmem"],
                       "planner": ["m_axi_map_mem", "m_axi_field_mem"]}
        dma = parse_hwh_dma(a.build)
        got = []
        for b in ("detpost", "prep", "planner"):
            if b not in blocks:
                continue
            for mbi in lpd_masters[b]:
                m = dma.get((blocks[b][1], mbi), [])
                ok = m == [("S_AXI_LPD", "LPD_DDR_LOW", 0, 0x7FFFFFFF)]
                got.append("%s/%s=%s" % (b, mbi, ",".join("%s:%s@0x%x-0x%x" % x for x in m) or None))
                if not ok:
                    errors.append("lpd: %s/%s maps %s (want only S_AXI_LPD LPD_DDR_LOW 0-0x7fffffff)" % (b, mbi, m))
        # MERGE10 P1 (GMAP3 planner, 32-bit m_axi addresses): every planner master's address width in the .hwh equals
        # the IP's, and what it maps (LPD_DDR_LOW, checked above) lies inside 2^ADDR_WIDTH, i.e. the 32-bit pointer
        # reaches every byte of the window the PS driver may hand it (buffers below 2 GiB).
        if "planner" in blocks and getattr(a, "planner_ip", None):
            mods = parse_hwh_modules(a.build)
            ipaw = ip_master_aw(a.planner_ip)
            p1 = []
            for mbi in lpd_masters["planner"]:
                hw = (mods.get(blocks["planner"][1], (None, {}))[1].get(mbi) or (None,) * 5)
                haw = (hw[3] or {}).get("ADDR_WIDTH") if len(hw) > 3 else None
                aw = ipaw.get(mbi)
                m = dma.get((blocks["planner"][1], mbi), [])
                p1.append("%s aw ip=%s hwh=%s" % (mbi, aw, haw))
                if aw is None or haw != str(aw):
                    errors.append("P1: planner/%s address width hwh %s != IP %s" % (mbi, haw, aw))
                elif not m or any(hi >= (1 << aw) or blk != "LPD_DDR_LOW" for _, blk, lo, hi in m):
                    errors.append("P1: planner/%s maps %s, not inside DDR_LOW below 2^%d" % (mbi, m, aw))
            report.append("P1 planner masters: %s -> %s" % ("; ".join(p1),
                          "MISMATCH" if [e for e in errors if e.startswith("P1")] else "OK"))
        report.append("lpd: SAXIGP6 width %s; masters %s -> %s" % (
            saxi6, " ".join(got), "OK" if saxi6 == 128 and not [e for e in errors if e.startswith("lpd")] else "MISMATCH"))
        if saxi6 != 128:
            errors.append("lpd: SAXIGP6 width %s, want 128" % saxi6)
    elif saxi6 is not None:
        errors.append("SAXIGP6 (S_AXI_LPD) enabled at %s bit but the check was run without --lpd" % saxi6)
    # MERGE7: sysinfo SLOT_MAP in the .hwh = the slots of the blocks this build is expected to have.
    sm = parse_hwh_slot_map(a.build)
    want_sm = 0
    for b in blocks:
        want_sm |= 1 << ((addr[b] - addr["sysinfo"]) // WINDOW)
    report.append("SLOT_MAP hwh=%s expected=0x%x -> %s" % (
        "0x%x" % sm if sm is not None else None, want_sm, "OK" if sm == want_sm else "MISMATCH"))
    if sm != want_sm:
        errors.append("SLOT_MAP %s != expected 0x%x" % (sm, want_sm))
    # MERGE4: the AFI fabric width the overlay programs must equal the width of every PS slave port
    # the .hwh enables (a 32/64-bit port left at the 128-bit AFI setting moves data in the wrong lanes).
    saxi = parse_hwh_saxi(a.build)
    afi = nodes.get("afi")
    bad = []
    if afi is None:
        bad.append("no config-afi in the overlay")
    else:
        for n, width in sorted(saxi.items()):
            want = AFI_CODE.get(width)
            got = (afi.get(2 * n), afi.get(2 * n + 1))
            if want is None or got != (want, want):
                bad.append("SAXIGP%d width %s: config-afi %d/%d = %s, want %s" % (n, width, 2 * n, 2 * n + 1, got, want))
    report.append("afi widths: hwh SAXIGP %s, overlay config-afi %s -> %s" % (
        ",".join("%d:%d" % kv for kv in sorted(saxi.items())),
        " ".join("%d:%d" % kv for kv in sorted((afi or {}).items()) if kv[0] < 14), "MISMATCH" if bad else "OK"))
    errors += bad
    # MERGE9P: the new cores (srch, memk, obs). Each check has a negative control in MERGE9P/tools/validate_m6.sh.
    errors += check_new_cores(a, blocks, irq1, irq1_src, report)
    # Driver headers must equal what the generators produce now.
    gens = [("regs", os.path.join(PL, "sw", "m7pl_hls_regs.h")),
            ("addrh", os.path.join(PL, "sw", "m7pl_addrmap.h"))]
    if "detpost" in blocks:
        gens.append(("detpost-regs", DETPOST_REGS_H))
    for gen, (b, _, path, _, _) in sorted(EXTRA_REGS.items()):
        if b in NEW_BLOCKS and getattr(a, "no_new_headers", False):
            if b in blocks:
                report.append("header %s: NOT CHECKED (--no-new-headers: stub IP in a validate-only run)" % os.path.basename(path))
            continue
        if b in blocks:
            ip = getattr(a, b + "_ip", None)
            if not ip:
                errors.append("--%s needs --%s-ip (the IP the build used)" % (b, b))
                continue
            gens.append((gen, path))
    for gen, path in gens:
        tmp = path + ".check"
        ns = argparse.Namespace(ip_repo=a.ip_repo, out=tmp)
        if gen in EXTRA_REGS:
            ns.ip = getattr(a, EXTRA_REGS[gen][0] + "_ip")
            cmd_extra_regs(ns, gen)
        else:
            {"regs": cmd_regs, "addrh": cmd_addrh, "detpost-regs": cmd_detpost_regs}[gen](ns)
        same = open(tmp).read() == open(path).read()
        os.remove(tmp)
        report.append("header %s up to date: %s" % (os.path.basename(path), same))
        if not same:
            errors.append("stale header " + path)
    print("\n".join(report))
    if errors:
        print("CHECK FAILED:", errors)
        return 1
    print("CHECK PASS")
    return 0


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("regs"); r.add_argument("--ip-repo", required=True); r.add_argument("--out", required=True)
    h = sub.add_parser("addrh"); h.add_argument("--out", required=True)
    dr = sub.add_parser("detpost-regs"); dr.add_argument("--ip-repo", required=True); dr.add_argument("--out", required=True)
    o = sub.add_parser("overlay"); o.add_argument("--clk-hz", type=int, required=True)
    o.add_argument("--clk1-hz", type=int)
    o.add_argument("--no-ar", action="store_true", help="build without the ar core (FPGA_M7_PL_NO_AR=1)")
    o.add_argument("--dpu-clk-hz", type=int, help="DPU build: clk_dpu actual (also implies no ar)")
    o.add_argument("--dpu2x-clk-hz", type=int, help="DPU build: clk_dsp actual")
    o.add_argument("--mem-direct", action="store_true",
                   help="MERGE4 build (FPGA_M7_PL_MEM_DIRECT=1): HP0/HP1 32 bit, HP3 64 bit in afi0")
    o.add_argument("--detpost", action="store_true",
                   help="MERGE5A build (FPGA_M7_PL_DETPOST=1): m7_detpost node, HP3 128 bit in afi0")
    for x in (o,):
        x.add_argument("--lpd", action="store_true", help="MERGE7 build (FPGA_M7_PL_LPD=1): detpost on S_AXI_LPD, HP3 rolling only")
        x.add_argument("--prep", action="store_true", help="MERGE7 build (FPGA_M7_PL_PREP=1): m7_prep node")
        x.add_argument("--planner", action="store_true", help="MERGE7 build (FPGA_M7_PL_PLANNER=1): m7_planner node")
        for b in ("srch", "memk", "obs"):   # MERGE9P
            x.add_argument("--" + b, action="store_true", help="MERGE9P build (FPGA_M7_PL_%s=1)" % b.upper())
            x.add_argument("--%s-ip" % b, dest=b + "_ip", help="the %s IP the build used (dir or export.zip)" % b)
        x.add_argument("--obs-port", dest="obs_port", default="hp1", choices=("hp1", "lpd"))
        x.add_argument("--memk-if", dest="memk_if", default="lite", choices=("lite", "bram"),
                       help="MERGE10 build (FPGA_M7_PL_MEMK_IF): the overlay is the same for both")
    o.add_argument("--bitname", required=True); o.add_argument("--out", required=True)
    c = sub.add_parser("check"); c.add_argument("--build", required=True)
    c.add_argument("--ip-repo", required=True); c.add_argument("--dtbo", required=True)
    c.add_argument("--clk-hz", type=int)
    c.add_argument("--clk1-hz", type=int)
    c.add_argument("--no-ar", action="store_true", help="build without the ar core (FPGA_M7_PL_NO_AR=1)")
    c.add_argument("--dpu-clk-hz", type=int, help="DPU build (FPGA_M7_PL_DPU_ARCH): check the DPU mapping/irq")
    c.add_argument("--detpost", action="store_true", help="MERGE5A build (FPGA_M7_PL_DETPOST=1)")
    c.add_argument("--lpd", action="store_true", help="MERGE7 build (FPGA_M7_PL_LPD=1)")
    c.add_argument("--prep", action="store_true", help="MERGE7 build (FPGA_M7_PL_PREP=1)")
    c.add_argument("--planner", action="store_true", help="MERGE7 build (FPGA_M7_PL_PLANNER=1)")
    c.add_argument("--prep-ip", dest="prep_ip", help="the pre-processing IP the build used (dir or export.zip)")
    c.add_argument("--planner-ip", dest="planner_ip", help="the planner IP the build used (dir or export.zip)")
    for b in ("srch", "memk", "obs"):   # MERGE9P
        c.add_argument("--" + b, action="store_true", help="MERGE9P build (FPGA_M7_PL_%s=1)" % b.upper())
        c.add_argument("--%s-ip" % b, dest=b + "_ip", help="the %s IP the build used (dir or export.zip)" % b)
    c.add_argument("--obs-port", dest="obs_port", default="hp1", choices=("hp1", "lpd"))
    c.add_argument("--memk-if", dest="memk_if", default="lite", choices=("lite", "bram"),
                   help="MERGE10 build (FPGA_M7_PL_MEMK_IF=bram): memk ctrl + bc_memk/bm_memk in slot 14 (check N6)")
    c.add_argument("--no-new-headers", dest="no_new_headers", action="store_true",
                   help="MERGE9P validate-only with stub IPs: skip the srch/memk/obs register header comparison")
    for gen in sorted(EXTRA_REGS):
        e = sub.add_parser(gen); e.add_argument("--ip", required=True); e.add_argument("--out", required=True)
    a = p.parse_args()
    if a.cmd in EXTRA_REGS:
        rc = cmd_extra_regs(a, a.cmd)
    else:
        rc = {"regs": cmd_regs, "addrh": cmd_addrh, "overlay": cmd_overlay, "check": cmd_check,
              "detpost-regs": cmd_detpost_regs}[a.cmd](a)
    sys.exit(rc or 0)


if __name__ == "__main__":
    main()
