# M7 PL block design for KV260 (K26 SOM, xck26-sfvc784-2LV-c), 2026-09-26.
#
# Sourced by build_bitstream.tcl; defines m7_create_bd. Rebuilds the whole BD
# from scratch, no GUI state. Structure (see M7_PL_INTEGRATION_20260926.md):
#
#   PS M_AXI_HPM0_FPD (128b) -> sc_ctrl -> sysinfo | frame | ar | permit
#                                          | roi | rolling
#   frame   m_axi_gmem -> sc_mem0 -> PS S_AXI_HP0_FPD -> DDR
#   roi     m_axi_gmem -> sc_mem1 (S00) -> PS S_AXI_HP1_FPD -> DDR
#   The E2 object-evidence region check (fpga_m7_object_roi_axi,
#   OBJECT_ROI_SPEC_20261011.md section 11.5): slot 9 (0xA0090000), irq bit 2
#   (SPI 91, the bit the retired display core left free; merge build 2026-10-11).
#   v3a (2026-10-26, owner-approved 2026-10-11): the legacy object core
#   (fpga_m7_object_axi, slot 2 0xA0020000, irq bit 1, HP1 S00) is REMOVED; it
#   had no caller on the car. Slot 2 and irq bit 1 stay empty (irq tied 0).
#   ar      m_axi_gmem0 (read: depth, grid, boxes) + m_axi_gmem1 (write: fb)
#           -> sc_mem2 (2 SI) -> PS S_AXI_HP2_FPD -> DDR; permit_i <- permit/permit_o
#   The AR overlay core (fpga_m7_ar_overlay_axi, AR_DISPLAY_SPEC_20261011.md)
#   REPLACES the legacy display core (owner-approved 2026-10-11): control
#   slot 3 (0xA0030000) stays unused; ar is slot 8, irq 7. Irq bit 2 went
#   to roi (below).
#   interrupts -> xlconcat -> PS pl_ps_irq0[7:0]
#   permit  (drive-permit heartbeat, slot 6, irq 5): frame/interrupt rising edge
#           = the ONLY freshness refresh; output port permit_hb_o (Pmod)
#   timebase: one 64-bit PL cycle counter (permit stamps, SYNC pairing)
#   (PL_PERMIT_UART_SPEC_20261011.md; external pin in xdc/m7_pmod_permit_uart.xdc)
#   v3fc (2026-10-26, owner: no unused core in the bitstream): the legacy PL
#   watchdog (slot 4, irq 3; replaced by permit, never opened by kv_server/kv_regs)
#   and the encoder UART (slot 7, irq 6; Pmod B10 never wired, frames always 0;
#   wheel odometry reaches the Nano over the chassis UART) are REMOVED. Their
#   slots stay empty, irq bits 3 and 6 are tied 0, port enc_uart_rx_i is gone.
#   v3b (2026-10-26): optional second PL clock. clk1_mhz > 0 enables pl_clk1
#   for the compute domain: frame/roi/ar (ap_clk), their SmartConnects
#   sc_mem0/1/2 and S_AXI_HP0/1/2_FPD; sc_ctrl gets a second clock (its own
#   clock-converter CDC). AXI-Lite RTL blocks, timebase, permit,
#   sysinfo and M_AXI_HPM0_FPD stay on pl_clk0. Two PL-internal
#   level crossings use m7_cdc_single (xpm_cdc_single): frame/interrupt ->
#   permit/frame_done_i and permit/permit_o -> ar/permit_i. HLS interrupts go
#   straight to pl_ps_irq0 (synchronized inside the PS). clk1_mhz = 0 builds
#   exactly the single-clock design.
#   S_AXI_HP3_FPD, control slot 5 and irq bit 4: rolling fusion core
#   fpga_m7_rolling_axi (v3 L2, 2026-10-26), on pl_clk0.
#   car screen is drawn on the Jetson Orin Nano, not in the PL): no_ar = 1 REMOVES
#   the ar overlay core and nothing else: slot 8 cleared in SLOT_MAP (0x363 ->
#   0x263), sc_ctrl 6 -> 5 masters (permit M02, roi M03, rolling M04), sc_mem2 and
#   S_AXI_HP2_FPD (SAXIGP4) gone, irq bit 7 tied 0, permit/permit_o left unloaded
#   (the PS reads permit STATUS; cdc_permit not created). Same idea as the D0 DPU
#   no_ar = 0 (default) issues exactly the commands of the design with ar.
#   REMOVES the ar overlay core (slot 8 cleared in SLOT_MAP, sc_mem2 gone,
#   permit_o no longer drives anything) and adds DPUCZDX8G v4.1 (Vitis AI 3.0)
#   following the DPU TRD: clk_wiz_dpu (MMCM fed by pl_clk0) gives clk_dsp =
#   2 x clk_dpu with matched routing; DPU S_AXI on M_AXI_HPM0_LPD (pl_clk0,
#   S_AXI_CLK_INDEPENDENT), 16 MiB at 0x8F00_0000 (round 1 used sc_ctrl M02 at 0xA100_0000); DATA0 -> S_AXI_HP2_FPD,
#   DATA1 -> S_AXI_HPC1_FPD, INSTR (32b) -> sc_dpu_instr -> S_AXI_HPC0_FPD, all on
#   clk_dpu; dpu0_interrupt -> pl_ps_irq0 bit 7 (the bit ar used). HP0 (frame)
#   keeps its own DDR port (HP0 = DDR port 3; HP1/HP2 share port 4; HPC via CCI).
#   dpu_arch = 0 builds exactly the design without the DPU.
#   (AR2NANO2 merge plan): has_ar = !no_ar && dpu_arch == 0, has_dpu = dpu_arch > 0.
#   clk_wiz_dpu is an explicit MMCM (D0 Auto picked a PLLE4, CLKC-58) with
#   PRIM_SOURCE Global_buffer: pl_clk0 already arrives on a global buffer, so the
#   wizard inserts no input buffer and its XDC writes no create_clock on clk_in1
#   (D0 TIMING-4/27: that primary clock overrode clk_pl_0); the MMCM outputs are
#   then generated clocks of clk_pl_0.
#   (FPGA_M7_PL_MEM_DIRECT=1) drops the three 1:1 memory SmartConnects sc_mem0/1/3:
#   S_AXI_HP0_FPD (SAXIGP2) and S_AXI_HP1_FPD (SAXIGP3) are set to 32 bit, S_AXI_HP3_FPD
#   (SAXIGP5) to 64 bit, the width of the frame / roi (32) and rolling (64) m_axi masters,
#   which connect straight to them (same clock as before: frame/roi on fclk, rolling on
#   pl_clk0). The overlay's afi0 config-afi must then program the same widths
#   (tools/m7pl_gen.py overlay --mem-direct; check compares it with the .hwh).
#   mem_direct = 0 (default) issues exactly the commands as before.
#   default 0 = exactly the commands as before.
#   dpu_clk_gate = 1 (FPGA_M7_PL_DPU_CLK_GATE=1, needs the DPU): CLK_GATING_ENA 1 (not part of the
#   arch fingerprint; SLIM_AUDIT2 v4_clkgate), clk_wiz_dpu clk_dsp output "Buffer_with_CE" and
#   dpu/dpu_2x_clk_ce -> clk_wiz_dpu/clk_dsp_ce, as the Vitis AI 3.0 TRD trd_bd.tcl lp branch.
#   detpost = 1 (FPGA_M7_PL_DETPOST=1): the YOLOv8 detection post-processing core
#   (xilinx.com:hls:detpost_top:1.0, DETPOST1 ooc_s8) on pl_clk0: control slot 10
#   (0xA00A_0000, SLOT_MAP bit 10), irq bit 3 (SPI 92, was tied 0), m_axi (128 bit) shares HP3
#   with the rolling core through a 2-SI SmartConnect sc_hp3 (HP3 / SAXIGP5 then 128 bit, also with
#   mem_direct: the rolling core's 64-bit master is upsized there). DETPOST1 README section 9.
#   controller rulings at its end): four more switches, all default 0 = exactly the commands as before.
#   geo_fx = 1 (FPGA_M7_PL_GEO_FX=1): asserts that the frame IP in the repo is the GEO_FX3 fixed-point core
#   (10-bit control address: FX_K[] at 0x200); the IP itself comes from FPGA_M7_PL_IPREPO (ip_repo-m5).
#   No command changes; it only refuses a build that would silently pick the old core.
#   lpd = 1 (FPGA_M7_PL_LPD=1): opens S_AXI_LPD (SAXIGP6, 128 bit, pl_clk0) behind a NUM_SI-to-1
#   axi_interconnect ic_lpd (STRATEGY lpd_ic: 1 = SASD shared access, 2 = SAMD; crossbar 128 bit);
#   detpost moves from HP3 to ic_lpd S00 and sc_hp3 is not created, so the rolling core goes back to
#   its m4 connection (mem_direct: 64-bit HP3 direct). SI order: detpost, prep, planner map_mem,
#   planner field_mem; the SI count follows the switches. DMA masters on it map ps/SAXIGP6/LPD_DDR_LOW.
#   prep = 1 (FPGA_M7_PL_PREP=1, needs lpd): recognition pre-processing core xilinx.com:hls:prep_top:1.0
#   (PREP1; s15 or s18 by the IP path given to build_bitstream.tcl): slot 11 0xA00B_0000, irq bit 6
#   (SPI 95), m_axi_gmem (128 bit) on ic_lpd.
#   planner = 1 (FPGA_M7_PL_PLANNER=1, needs lpd): planner_field_dual fpga:navigation:planner_field_dual:1.0
#   m_axi_field_mem (32 bit each, upsized in ic_lpd).
#   switches, all default 0 = exactly the commands as before. The new cores' VLNVs come from their IP
#   (::m7_vlnv, set by build_bitstream.tcl from component.xml); their ports are found on the cell
#   (every AXI4 master pin, the one AXI4-Lite slave pin, the interrupt pin), so the BD does not
#   srch = 1 (needs lpd): probabilistic search core (SRCHK0): slot 13 0xA00D_0000, pl_ps_irq1 bit 0
#   (SPI 104, PSU__USE__IRQ1), its m_axi master(s) appended to ic_lpd after the planner (SI order of the
#   m5 masters unchanged).
#   memk = 1: semantic memory core (MEMK0, AXI-Lite arrays only): slot 14 0xA00E_0000, pl_ps_irq1 bit 1
#   (SPI 105); any m_axi master it may have goes to ic_lpd (needs lpd then).
#   (FPGA_M7_PL_MEMK_IF=bram) is the MEMK4 shell: the core has only the ap_ctrl AXI-Lite slave plus one
#   BRAM master port (cmd words 0-1023, res words 1024-2047, byte addresses). The block design adds
#   bm_memk (blk_mem_gen, true dual port, 2048 x 32 = 2 RAMB36) and bc_memk (axi_bram_ctrl, AXI4-Lite,
#   32 bit, single port) on the next sc_ctrl master (the HPM0_FPD control path the MEMK3 s_axilite
#   arrays used). memk/blk_PORTA -> bm_memk/BRAM_PORTB, bc_memk/BRAM_PORTA -> bm_memk/BRAM_PORTA. Both
#   stay in slot 14: core control 0xA00E_0000 (4 KiB), block 0xA00E_8000 (8 KiB; the controller's range
#   sets the depth), so SLOT_MAP and the m7_memk overlay node (64 KiB) are unchanged. memk_if lite
#   (default) = exactly the MERGE9P commands (MEMK3-type core, its arrays inside the 64 KiB slot).
#   slot 9 0xA009_0000 / irq bit 2 / sc_ctrl port as roi, but on pl_clk0. obs_port hp1 (default): its one
#   m_axi master straight onto S_AXI_HP1_FPD (SAXIGP3), port width = the master's width, saxihp1_fpd_aclk
#   on pl_clk0; obs_port lpd: its master(s) on ic_lpd and S_AXI_HP1_FPD is turned off.
#
# The PS configuration is written by hand (no KV260 board files on this host).
# Under `xmutil loadapp` the PS is already configured by the KV260 boot
# firmware; only the PL bitstream and the overlay are applied. Therefore the
# PS settings below only have to (a) enable the PS-PL ports used here with the
# SAME widths the running system programs (128-bit HPM0_FPD and HP ports, as
# the Kria base overlay's afi0 node does) and (b) define pl_clk0. DDR/MIO
# values are tool defaults and must NOT be used to build a boot image.

# Address plan (M_AXI_HPM0_FPD window 0xA000_0000..0xAFFF_FFFF), 64 KiB each.
array set ::m7_addr {
    sysinfo   0xA0000000
    frame     0xA0010000
    rolling   0xA0050000
    permit    0xA0060000
    ar        0xA0080000
    roi       0xA0090000
    detpost   0xA00A0000
    prep      0xA00B0000
    planner   0xA00C0000
    srch      0xA00D0000
    memk      0xA00E0000
    memk_bram 0xA00E8000
    obs       0xA0090000
}
# MERGE10: ranges inside slot 14 when memk_if is bram (m7pl_gen.py check reads these two lines)
set ::m7_memk_ctrl_range 4K
set ::m7_memk_bram_range 8K
# pl_ps_irq0 bit assignment; GIC SPI = 89 + bit.
array set ::m7_irq {
    frame 0
    rolling 4
    permit 5
    ar 7
    roi 2
    detpost 3
    prep 6
    planner 1
    obs 2
}
# MERGE9P: pl_ps_irq1 bit assignment; GIC SPI = 104 + bit (pl_ps_irq0 is full in m5).
array set ::m7_irq1 {
    srch 0
    memk 1
}
# MERGE9P: VLNV per extra IP; build_bitstream.tcl overrides an entry with the VLNV in the IP's component.xml.
array set ::m7_vlnv {
    prep    xilinx.com:hls:prep_top:1.0
    planner fpga:navigation:planner_field_dual:1.0
}
proc m7_vlnv_of {c} {
    if {![info exists ::m7_vlnv($c)]} { error "$c: no VLNV (build_bitstream.tcl sets ::m7_vlnv($c) from the IP's component.xml)" }
    return $::m7_vlnv($c)
}
# MERGE9P: interface pins of a new core, found on the cell (sorted by name).
proc m7_maxi_pins {c} {
    set out {}
    foreach p [get_bd_intf_pins -quiet -of_objects [get_bd_cells $c] -filter {MODE == Master}] {
        if {[string match *aximm* [get_property VLNV $p]]} { lappend out $p }
    }
    return [lsort $out]
}
proc m7_lite_pin {c} {
    set out {}
    foreach p [get_bd_intf_pins -quiet -of_objects [get_bd_cells $c] -filter {MODE == Slave}] {
        if {[string match *aximm* [get_property VLNV $p]]} { lappend out $p }
    }
    if {[llength $out] != 1} { error "$c: expected one AXI slave (control) pin, got '$out'" }
    return [lindex $out 0]
}

# D0 round 2: DPU S_AXI on its own PS master M_AXI_HPM0_LPD (32 bit, the TRD's
# choice), at the TRD/PetaLinux address in the LPD window.
set ::m7_dpu_base 0x8F000000
set ::m7_bd_dir [file dirname [file normalize [info script]]]
# D0 probe: DPUCZDX8G per the Vitis AI 3.0 DPU TRD (trd_bd.tcl lib_dpu_ips_clk, no clock gating).
proc m7_add_dpu {arch uram mhz clk aresetn {clk_gate 0}} {
    source [file join $::m7_bd_dir .. dpu dpu_config.tcl]
    set dpu [create_bd_cell -type ip -vlnv xilinx.com:ip:dpuczdx8g:4.1 dpu]
    set_property -dict [m7_dpu_props $arch $uram] $dpu
    # clocks: explicit MMCM (DPU1) from pl_clk0, which is already on a global buffer (PRIM_SOURCE Global_buffer: no IBUF/BUFG, no create_clock on clk_in1); CLKOUT1 = clk_dsp (2x), CLKOUT2 = clk_dpu (1x), matched routing (TRD, PG338)
    set cw [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz clk_wiz_dpu]
    set_property -dict [list \
        CONFIG.PRIMITIVE {MMCM} CONFIG.PRIM_SOURCE {Global_buffer} \
        CONFIG.USE_LOCKED {true} CONFIG.USE_RESET {true} CONFIG.RESET_TYPE {ACTIVE_LOW} \
        CONFIG.CLK_OUT1_PORT {clk_dsp} CONFIG.CLKOUT1_REQUESTED_OUT_FREQ [expr {2 * $mhz}] \
        CONFIG.CLKOUT2_USED {true} CONFIG.CLK_OUT2_PORT {clk_dpu} CONFIG.CLKOUT2_REQUESTED_OUT_FREQ $mhz \
        CONFIG.CLKOUT1_MATCHED_ROUTING {true} CONFIG.CLKOUT2_MATCHED_ROUTING {true} \
    ] $cw
    if {$clk_gate} {
        # MERGE5A: DPU clock gating (TRD lp branch); the fingerprint does not contain CLK_GATING_ENA
        set_property CONFIG.CLK_GATING_ENA {1} $dpu
        set_property CONFIG.CLKOUT1_DRIVES {Buffer_with_CE} $cw
    }
    connect_bd_net $clk [get_bd_pins clk_wiz_dpu/clk_in1]
    connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins clk_wiz_dpu/resetn]
    set dclk [get_bd_pins clk_wiz_dpu/clk_dpu]
    set xclk [get_bd_pins clk_wiz_dpu/clk_dsp]
    foreach {r c} [list rst_dpu $dclk rst_dsp $xclk] {
        create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset $r
        connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins $r/ext_reset_in]
        connect_bd_net $c [get_bd_pins $r/slowest_sync_clk]
        connect_bd_net [get_bd_pins clk_wiz_dpu/locked] [get_bd_pins $r/dcm_locked]
    }
    connect_bd_net $xclk [get_bd_pins dpu/dpu_2x_clk]
    if {$clk_gate} {
        connect_bd_net [get_bd_pins dpu/dpu_2x_clk_ce] [get_bd_pins clk_wiz_dpu/clk_dsp_ce]
    }
    connect_bd_net [get_bd_pins rst_dsp/peripheral_aresetn] [get_bd_pins dpu/dpu_2x_resetn]
    connect_bd_net $dclk [get_bd_pins dpu/m_axi_dpu_aclk]
    connect_bd_net [get_bd_pins rst_dpu/peripheral_aresetn] [get_bd_pins dpu/m_axi_dpu_aresetn]
    connect_bd_net $clk [get_bd_pins dpu/s_axi_aclk]
    connect_bd_net $aresetn [get_bd_pins dpu/s_axi_aresetn]
    # Round 1 (5b46066) hung the DPU's full-AXI4 32-bit slave on sc_ctrl (128-bit HPM0_FPD):
    # sc_ctrl grew from ~1.2k to 10.7k LUT (burst-capable downsizer). Round 2: direct
    # 32-bit HPM0_LPD -> DPU S_AXI (same width, same AXI4, ID 16 = DPU S_AXI_ID_BW 16).
    set_property -dict [list CONFIG.PSU__USE__M_AXI_GP2 {1} CONFIG.PSU__MAXIGP2__DATA_WIDTH {32}] [get_bd_cells ps]
    connect_bd_net $clk [get_bd_pins ps/maxihpm0_lpd_aclk]
    connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_LPD] [get_bd_intf_pins dpu/S_AXI]
    # PS slave ports for the DPU, 128 bit, clocked by clk_dpu
    set_property -dict [list \
        CONFIG.PSU__USE__S_AXI_GP0 {1} CONFIG.PSU__SAXIGP0__DATA_WIDTH {128} \
        CONFIG.PSU__USE__S_AXI_GP1 {1} CONFIG.PSU__SAXIGP1__DATA_WIDTH {128} \
    ] [get_bd_cells ps]
    foreach p {saxihp2_fpd_aclk saxihpc0_fpd_aclk saxihpc1_fpd_aclk} {
        connect_bd_net $dclk [get_bd_pins ps/$p]
    }
    connect_bd_intf_net [get_bd_intf_pins dpu/DPU0_M_AXI_DATA0] [get_bd_intf_pins ps/S_AXI_HP2_FPD]
    connect_bd_intf_net [get_bd_intf_pins dpu/DPU0_M_AXI_DATA1] [get_bd_intf_pins ps/S_AXI_HPC1_FPD]
    # INSTR is 32 bit: SmartConnect upsizes to the 128-bit HPC0 port
    set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_dpu_instr]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $sc
    connect_bd_net $dclk [get_bd_pins sc_dpu_instr/aclk]
    connect_bd_net [get_bd_pins rst_dpu/interconnect_aresetn] [get_bd_pins sc_dpu_instr/aresetn]
    connect_bd_intf_net [get_bd_intf_pins dpu/DPU0_M_AXI_INSTR] [get_bd_intf_pins sc_dpu_instr/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins sc_dpu_instr/M00_AXI] [get_bd_intf_pins ps/S_AXI_HPC0_FPD]
}

proc m7_create_bd {bd_dir clk_mhz build_id build_date {clk1_mhz 0} {dpu_arch 0} {dpu_uram 0} {dpu_mhz 300} {no_ar 0} {mem_direct 0} {dpu_clk_gate 0} {detpost 0} {geo_fx 0} {lpd 0} {prep 0} {planner 0} {lpd_ic 1} {srch 0} {memk 0} {obs 0} {obs_port hp1} {lpd_fifo_new {}} {memk_if lite}} {
    set has_dpu [expr {$dpu_arch > 0}]
    if {$dpu_clk_gate && !$has_dpu} { error "dpu_clk_gate needs the DPU (dpu_arch > 0)" }
    # MERGE7: prep / planner live on the S_AXI_LPD interconnect only
    if {($prep || $planner) && !$lpd} { error "prep / planner need lpd (FPGA_M7_PL_LPD=1)" }
    # MERGE9P
    if {$srch && !$lpd} { error "srch needs lpd (FPGA_M7_PL_LPD=1)" }
    if {$obs && $obs_port ne "hp1" && $obs_port ne "lpd"} { error "obs_port: hp1 or lpd, got '$obs_port'" }
    if {$obs && $obs_port eq "lpd" && !$lpd} { error "obs_port lpd needs lpd (FPGA_M7_PL_LPD=1)" }
    if {$obs && $obs_port eq "hp1" && !$mem_direct} { error "obs on hp1 needs mem_direct (FPGA_M7_PL_MEM_DIRECT=1)" }
    set obs_hp1 [expr {$obs && $obs_port eq "hp1"}]
    # MERGE10
    if {[lsearch -exact {lite bram} $memk_if] < 0} { error "memk_if: lite or bram, got '$memk_if'" }
    if {$memk_if eq "bram" && !$memk} { error "memk_if bram needs memk (FPGA_M7_PL_MEMK=1)" }
    set memk_bram [expr {$memk && $memk_if eq "bram"}]
    set lpd_si [expr {$detpost + $prep + 2 * $planner}]
    if {$lpd && $lpd_si == 0} { error "lpd: no master for ic_lpd (needs detpost, prep or planner)" }
    if {$lpd && $lpd_ic != 1 && $lpd_ic != 2} { error "lpd_ic: STRATEGY 1 (SASD) or 2 (SAMD), got '$lpd_ic'" }
    set detpost_hp3 [expr {$detpost && !$lpd}]   ;# MERGE5A sc_hp3 only without the LPD port
    set has_ar  [expr {!$no_ar && !$has_dpu}]
    set design m7_pl
    create_bd_design -dir $bd_dir $design
    current_bd_design $design

    # ---------------- Zynq UltraScale+ PS ----------------
    set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps]
    set_property -dict [list \
        CONFIG.PSU__USE__M_AXI_GP0 {1} \
        CONFIG.PSU__MAXIGP0__DATA_WIDTH {128} \
        CONFIG.PSU__USE__M_AXI_GP1 {0} \
        CONFIG.PSU__USE__M_AXI_GP2 {0} \
        CONFIG.PSU__USE__S_AXI_GP2 {1} \
        CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
        CONFIG.PSU__USE__S_AXI_GP3 {1} \
        CONFIG.PSU__SAXIGP3__DATA_WIDTH {128} \
        CONFIG.PSU__USE__S_AXI_GP4 [expr {($has_ar || $has_dpu) ? 1 : 0}] \
        CONFIG.PSU__SAXIGP4__DATA_WIDTH {128} \
        CONFIG.PSU__USE__S_AXI_GP5 {1} \
        CONFIG.PSU__SAXIGP5__DATA_WIDTH {128} \
        CONFIG.PSU__USE__IRQ0 {1} \
        CONFIG.PSU__FPGA_PL0_ENABLE {1} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__SRCSEL {IOPLL} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ $clk_mhz \
        CONFIG.PSU__FPGA_PL1_ENABLE {0} \
    ] $ps
    if {$mem_direct} {
        # MERGE4: PS slave port widths = the cores' m_axi widths (no SmartConnect upsizer)
        set_property -dict [list \
            CONFIG.PSU__SAXIGP2__DATA_WIDTH {32} \
            CONFIG.PSU__SAXIGP3__DATA_WIDTH {32} \
            CONFIG.PSU__SAXIGP5__DATA_WIDTH {64} \
        ] $ps
    }
    if {$detpost_hp3} {
        # MERGE5A: HP3 is shared by rolling (64) and detpost (128) through sc_hp3 -> 128-bit port
        set_property CONFIG.PSU__SAXIGP5__DATA_WIDTH {128} $ps
    }
    if {$lpd} {
        # MERGE7: S_AXI_LPD (SAXIGP6), 128 bit, behind ic_lpd on pl_clk0
        set_property -dict [list CONFIG.PSU__USE__S_AXI_GP6 {1} CONFIG.PSU__SAXIGP6__DATA_WIDTH {128}] $ps
    }

    set clk [get_bd_pins ps/pl_clk0]
    set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_pl0]
    connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins rst_pl0/ext_reset_in]
    connect_bd_net $clk [get_bd_pins rst_pl0/slowest_sync_clk]
    set aresetn [get_bd_pins rst_pl0/peripheral_aresetn]
    set icresetn [get_bd_pins rst_pl0/interconnect_aresetn]
    if {$clk1_mhz > 0} {
        set_property -dict [list \
            CONFIG.PSU__FPGA_PL1_ENABLE {1} \
            CONFIG.PSU__CRL_APB__PL1_REF_CTRL__SRCSEL {IOPLL} \
            CONFIG.PSU__CRL_APB__PL1_REF_CTRL__FREQMHZ $clk1_mhz \
        ] $ps
        set fclk [get_bd_pins ps/pl_clk1]
        create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_pl1
        connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins rst_pl1/ext_reset_in]
        connect_bd_net $fclk [get_bd_pins rst_pl1/slowest_sync_clk]
        set faresetn [get_bd_pins rst_pl1/peripheral_aresetn]
        set ficresetn [get_bd_pins rst_pl1/interconnect_aresetn]
    } else {
        set fclk $clk
        set faresetn $aresetn
        set ficresetn $icresetn
    }
    connect_bd_net $clk [get_bd_pins ps/maxihpm0_fpd_aclk]
    set hp_fast {saxihp0_fpd_aclk saxihp1_fpd_aclk saxihp2_fpd_aclk}
    if {!$has_ar} { set hp_fast {saxihp0_fpd_aclk saxihp1_fpd_aclk} }  ;# HP2 (SAXIGP4) disabled, or on clk_dpu (m7_add_dpu)
    if {$obs} {
        # MERGE9P: roi (the only other HP1 user) is replaced by obs on pl_clk0: HP1 on pl_clk0 (hp1) or off (lpd)
        set hp_fast [lsearch -all -inline -not -exact $hp_fast saxihp1_fpd_aclk]
        if {$obs_hp1} {
            connect_bd_net $clk [get_bd_pins ps/saxihp1_fpd_aclk]
        } else {
            set_property CONFIG.PSU__USE__S_AXI_GP3 {0} $ps
        }
    }
    foreach p $hp_fast {
        connect_bd_net $fclk [get_bd_pins ps/$p]
    }
    # v3 L2: the rolling core and its HP3 port stay on pl_clk0 (also with pl_clk1 on)
    connect_bd_net $clk [get_bd_pins ps/saxihp3_fpd_aclk]

    # ---------------- Compute / display cores (HLS AXI shells) ----------------
    create_bd_cell -type ip -vlnv fpga.lab:m7:fpga_m7_frame_axi:1.0 frame
    if {$geo_fx} {
        # MERGE7: the GEO_FX3 fixed-point core decodes 10 control address bits (FX_K[] at 0x200..0x27f)
        set aw [get_property CONFIG.ADDR_WIDTH [get_bd_intf_pins frame/s_axi_control]]
        if {$aw != 10} { error "geo_fx: frame IP s_axi_control ADDR_WIDTH is '$aw', the GEO_FX3 core has 10 (wrong ip repo?)" }
    }
    set fast_cores {frame ar roi}
    if {!$has_ar} {
        set fast_cores {frame roi}
    } else {
        create_bd_cell -type ip -vlnv fpga.lab:m7:fpga_m7_ar_overlay_axi:1.0 ar
    }
    if {$obs} {
        # MERGE9P: obs replaces roi (slot 9, irq bit 2, roi's sc_ctrl port), on pl_clk0
        set fast_cores [lsearch -all -inline -not -exact $fast_cores roi]
        create_bd_cell -type ip -vlnv [m7_vlnv_of obs] obs
        connect_bd_net $clk [get_bd_pins obs/ap_clk]
        connect_bd_net $aresetn [get_bd_pins obs/ap_rst_n]
        set obs_m [m7_maxi_pins obs]
        if {$obs_hp1} {
            if {[llength $obs_m] != 1} { error "obs on hp1: expected one m_axi master, got '$obs_m' (use obs_port lpd)" }
            set w [get_property CONFIG.DATA_WIDTH [lindex $obs_m 0]]
            if {[lsearch -exact {32 64 128} $w] < 0} { error "obs m_axi width '$w' is not 32/64/128" }
            set_property CONFIG.PSU__SAXIGP3__DATA_WIDTH $w $ps
        }
    } else {
    create_bd_cell -type ip -vlnv fpga.lab:m7:fpga_m7_object_roi_axi:1.0 roi
    }
    # ---------------- RTL blocks (module references) ----------------
    set slot_map [expr {0x363}]  ;# slots 0,1,5,6,8,9 populated; 2 retired (legacy object, v3a), 3 retired (legacy display), 4 retired (watchdog, v3fc), 7 retired (encoder, v3fc); 5 rolling (v3 L2)
    if {!$has_ar} { set slot_map [expr {$slot_map & ~(1 << 8)}] }  ;# AR2NANO: slot 8 (ar) absent -> 0x263
    if {$detpost} { set slot_map [expr {$slot_map | (1 << 10)}] }   ;# MERGE5A: slot 10 detpost -> 0x663
    if {$prep}    { set slot_map [expr {$slot_map | (1 << 11)}] }   ;# MERGE7: slot 11 prep
    if {$planner} { set slot_map [expr {$slot_map | (1 << 12)}] }   ;# MERGE7: slot 12 planner -> 0x1E63
    if {$srch}    { set slot_map [expr {$slot_map | (1 << 13)}] }   ;# MERGE9P: slot 13 srch
    if {$memk}    { set slot_map [expr {$slot_map | (1 << 14)}] }   ;# MERGE9P: slot 14 memk -> 0x7E63 (obs keeps slot 9)
    create_bd_cell -type module -reference m7_sysinfo_axil sysinfo
    set_property -dict [list \
        CONFIG.BUILD_ID [format 0x%08X $build_id] \
        CONFIG.BUILD_DATE [format 0x%08X $build_date] \
        CONFIG.CLK_HZ [expr {int($clk_mhz * 1000000)}] \
        CONFIG.SLOT_MAP [format 0x%08X $slot_map] \
    ] [get_bd_cells sysinfo]
    # Timebase + drive-permit heartbeat (2026-10-11). Freshness window: runtime register
    # 0x44 (v3fc2 2026-10-26), power-on 1000 ms, writes clamped to [100 ms, 5000 ms].
    create_bd_cell -type module -reference m7_timebase timebase
    create_bd_cell -type module -reference m7_permit_axil permit
    set_property -dict [list \
        CONFIG.TIMEOUT_CYCLES [expr {int($clk_mhz * 1000000)}] \
        CONFIG.TIMEOUT_MIN_CYCLES [expr {int($clk_mhz * 100000)}] \
        CONFIG.TIMEOUT_MAX_CYCLES [expr {int($clk_mhz * 5000000)}] \
        CONFIG.HB_HALF_CYCLES [expr {int($clk_mhz * 5000)}] \
        CONFIG.EXT_FRAME_ID {0} \
    ] [get_bd_cells permit]

    foreach c $fast_cores {
        connect_bd_net $fclk [get_bd_pins $c/ap_clk]
        connect_bd_net $faresetn [get_bd_pins $c/ap_rst_n]
    }
    foreach c {sysinfo permit} {
        connect_bd_net $clk [get_bd_pins $c/s_axi_aclk]
        connect_bd_net $aresetn [get_bd_pins $c/s_axi_aresetn]
    }
    connect_bd_net $clk [get_bd_pins timebase/clk]
    connect_bd_net $aresetn [get_bd_pins timebase/rst_n]
    connect_bd_net [get_bd_pins timebase/timebase] [get_bd_pins permit/timebase_i]
    # Freshness source: the geometry core's completion interrupt (level; the
    # permit block takes its rising edge). PS must enable GIE/IER and clear ISR
    # every frame. frame_id_i is unused while EXT_FRAME_ID=0 (id from the PS
    # register NEXT_FRAME_ID); after A1 it should come from the frame header.
    if {$clk1_mhz > 0} {
        create_bd_cell -type module -reference m7_cdc_single cdc_frame_done
        connect_bd_net $fclk [get_bd_pins cdc_frame_done/src_clk]
        connect_bd_net $clk [get_bd_pins cdc_frame_done/dest_clk]
        connect_bd_net [get_bd_pins frame/interrupt] [get_bd_pins cdc_frame_done/src_in]
        connect_bd_net [get_bd_pins cdc_frame_done/dest_out] [get_bd_pins permit/frame_done_i]
    } else {
        connect_bd_net [get_bd_pins frame/interrupt] [get_bd_pins permit/frame_done_i]
    }
    set id0 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant permit_id_zero]
    set_property -dict [list CONFIG.CONST_WIDTH {32} CONFIG.CONST_VAL {0}] $id0
    connect_bd_net [get_bd_pins permit_id_zero/dout] [get_bd_pins permit/frame_id_i]
    create_bd_port -dir O permit_hb_o
    connect_bd_net [get_bd_pins permit/permit_hb_o] [get_bd_ports permit_hb_o]
    # Drive permit shown on the AR panel as wired (level = fresh && !FORCE_STOP);
    # the core samples it once at ap_start. permit_present is the PS register.
    if {!$has_ar} {
        # no ar core: permit_o stays an unloaded output (the PS reads permit STATUS)
    } elseif {$clk1_mhz > 0} {
        create_bd_cell -type module -reference m7_cdc_single cdc_permit
        connect_bd_net $clk [get_bd_pins cdc_permit/src_clk]
        connect_bd_net $fclk [get_bd_pins cdc_permit/dest_clk]
        connect_bd_net [get_bd_pins permit/permit_o] [get_bd_pins cdc_permit/src_in]
        connect_bd_net [get_bd_pins cdc_permit/dest_out] [get_bd_pins ar/permit_i]
    } else {
        connect_bd_net [get_bd_pins permit/permit_o] [get_bd_pins ar/permit_i]
    }

    # ---------------- Control interconnect ----------------
    set scc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_ctrl]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI [expr {($has_ar ? 6 : 5) + ($detpost ? 1 : 0) + $prep + $planner + $srch + $memk + $memk_bram}]] $scc
    # sc_ctrl master port per core (without ar, its M02 is dropped and the rest shift down)
    if {!$has_ar} {
        array set mi {permit M02 roi M03 rolling M04 detpost M05}
    } else {
        array set mi {permit M03 roi M04 rolling M05 detpost M06}
    }
    # MERGE7: prep / planner take the next free sc_ctrl masters after the ones above
    set nmi [expr {($has_ar ? 6 : 5) + ($detpost ? 1 : 0)}]
    foreach {c on} [list prep $prep planner $planner srch $srch memk $memk memk_bram $memk_bram] {
        if {$on} { set mi($c) [format M%02d $nmi]; incr nmi }
    }
    connect_bd_net $clk [get_bd_pins sc_ctrl/aclk]
    if {$clk1_mhz > 0} {
        set_property CONFIG.NUM_CLKS {2} $scc
        connect_bd_net $fclk [get_bd_pins sc_ctrl/aclk1]
    }
    connect_bd_net $icresetn [get_bd_pins sc_ctrl/aresetn]
    connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins sc_ctrl/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M00_AXI] [get_bd_intf_pins sysinfo/s_axi]
    connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M01_AXI] [get_bd_intf_pins frame/s_axi_control]
    if {$has_ar} {
        connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M02_AXI] [get_bd_intf_pins ar/s_axi_control]
    }
    connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(permit)_AXI] [get_bd_intf_pins permit/s_axi]
    if {$obs} {
        connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(roi)_AXI] [m7_lite_pin obs]
    } else {
    connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(roi)_AXI] [get_bd_intf_pins roi/s_axi_control]
    }

    # ---------------- Memory paths: one SmartConnect + HP port per core ----------------
    # sc_mem1 used to be shared with the legacy object core (S00, roi on S01);
    # since v3a roi is its only master (S00).
    set i 0
    foreach {c hp} {frame S_AXI_HP0_FPD roi S_AXI_HP1_FPD} {
        if {$mem_direct} {          ;# MERGE4: 32-bit master straight onto the 32-bit HP port
            if {$c eq "roi" && $obs} {   ;# MERGE9P: obs instead of roi (on HP1 or on ic_lpd)
                if {$obs_hp1} { connect_bd_intf_net [lindex $obs_m 0] [get_bd_intf_pins ps/$hp] }
                incr i
                continue
            }
            connect_bd_intf_net [get_bd_intf_pins $c/m_axi_gmem] [get_bd_intf_pins ps/$hp]
            incr i
            continue
        }
        set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_mem$i]
        set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $sc
        connect_bd_net $fclk [get_bd_pins sc_mem$i/aclk]
        connect_bd_net $ficresetn [get_bd_pins sc_mem$i/aresetn]
        connect_bd_intf_net [get_bd_intf_pins $c/m_axi_gmem] [get_bd_intf_pins sc_mem$i/S00_AXI]
        connect_bd_intf_net [get_bd_intf_pins sc_mem$i/M00_AXI] [get_bd_intf_pins ps/$hp]
        incr i
    }
    if {$has_ar} {
    # ar: read master gmem0 and write master gmem1 share HP2 through sc_mem2.
    set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_mem2]
    set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] $sc
    connect_bd_net $fclk [get_bd_pins sc_mem2/aclk]
    connect_bd_net $ficresetn [get_bd_pins sc_mem2/aresetn]
    connect_bd_intf_net [get_bd_intf_pins ar/m_axi_gmem0] [get_bd_intf_pins sc_mem2/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins ar/m_axi_gmem1] [get_bd_intf_pins sc_mem2/S01_AXI]
    connect_bd_intf_net [get_bd_intf_pins sc_mem2/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP2_FPD]
    }
    if {$has_dpu} {
        m7_add_dpu $dpu_arch $dpu_uram $dpu_mhz $clk $aresetn $dpu_clk_gate
    }

    # ---------------- Interrupts ----------------
    set cat [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat irq_concat]
    set_property CONFIG.NUM_PORTS {8} $cat
    connect_bd_net [get_bd_pins frame/interrupt]    [get_bd_pins irq_concat/In$::m7_irq(frame)]
    if {$has_ar} {
        connect_bd_net [get_bd_pins ar/interrupt]   [get_bd_pins irq_concat/In$::m7_irq(ar)]
    }
    connect_bd_net [get_bd_pins permit/irq]         [get_bd_pins irq_concat/In$::m7_irq(permit)]
    if {$obs} {
        connect_bd_net [get_bd_pins obs/interrupt]  [get_bd_pins irq_concat/In$::m7_irq(obs)]
    } else {
    connect_bd_net [get_bd_pins roi/interrupt]      [get_bd_pins irq_concat/In$::m7_irq(roi)]
    }
    set zero [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant irq_zero]
    set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {0}] $zero
    # bit 1: retired legacy object core (v3a); bits 3/6: retired watchdog/encoder (v3fc).
    # Bit 4 is the rolling core (v3 L2).
    if {!$prep && !$planner} {
        connect_bd_net [get_bd_pins irq_zero/dout] [get_bd_pins irq_concat/In1] [get_bd_pins irq_concat/In6]
    } elseif {!$planner} {
        connect_bd_net [get_bd_pins irq_zero/dout] [get_bd_pins irq_concat/In1]   ;# MERGE7: bit 6 = prep
    } elseif {!$prep} {
        connect_bd_net [get_bd_pins irq_zero/dout] [get_bd_pins irq_concat/In6]   ;# MERGE7: bit 1 = planner
    }
    if {!$detpost} {
        connect_bd_net [get_bd_pins irq_zero/dout] [get_bd_pins irq_concat/In3]
    }
    # AR2NANO: without ar, bit 7 is tied 0 as well
    # DPU1: with the DPU, bit 7 is dpu0_interrupt (GIC SPI 96, D0)
    if {$has_dpu} {
        connect_bd_net [get_bd_pins dpu/dpu0_interrupt] [get_bd_pins irq_concat/In$::m7_irq(ar)]
    } elseif {!$has_ar} {
        connect_bd_net [get_bd_pins irq_zero/dout] [get_bd_pins irq_concat/In$::m7_irq(ar)]
    }
    # Slot 5 (control 0xA005_0000 on sc_ctrl M05), its own HP3 port (S_AXI_HP3_FPD =
    # SAXIGP5, sc_mem3), interrupt bit 4 (was tied to irq_zero). The map stays in the
    # core's URAM; the m_axi master reads the frame core's output block and writes
    # header/delta/query/dump buffers in low DDR.
    create_bd_cell -type ip -vlnv fpga.lab:m7:fpga_m7_rolling_axi:1.0 rolling
    connect_bd_net $clk [get_bd_pins rolling/ap_clk]
    connect_bd_net $aresetn [get_bd_pins rolling/ap_rst_n]
    connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(rolling)_AXI] [get_bd_intf_pins rolling/s_axi_control]
    if {$detpost_hp3} {             ;# MERGE5A: rolling S00 + detpost S01 -> sc_hp3 -> 128-bit HP3
        create_bd_cell -type ip -vlnv xilinx.com:hls:detpost_top:1.0 detpost
        connect_bd_net $clk [get_bd_pins detpost/ap_clk]
        connect_bd_net $aresetn [get_bd_pins detpost/ap_rst_n]
        connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(detpost)_AXI] [get_bd_intf_pins detpost/s_axi_control]
        set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_hp3]
        set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] $sc
        connect_bd_net $clk [get_bd_pins sc_hp3/aclk]
        connect_bd_net $icresetn [get_bd_pins sc_hp3/aresetn]
        connect_bd_intf_net [get_bd_intf_pins rolling/m_axi_gmem] [get_bd_intf_pins sc_hp3/S00_AXI]
        connect_bd_intf_net [get_bd_intf_pins detpost/m_axi_gmem] [get_bd_intf_pins sc_hp3/S01_AXI]
        connect_bd_intf_net [get_bd_intf_pins sc_hp3/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP3_FPD]
        connect_bd_net [get_bd_pins detpost/interrupt] [get_bd_pins irq_concat/In$::m7_irq(detpost)]
    } elseif {$mem_direct} {        ;# MERGE4: 64-bit master straight onto the 64-bit HP3 port
        connect_bd_intf_net [get_bd_intf_pins rolling/m_axi_gmem] [get_bd_intf_pins ps/S_AXI_HP3_FPD]
    } else {
    set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_mem3]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $sc
    connect_bd_net $clk [get_bd_pins sc_mem3/aclk]
    connect_bd_net $icresetn [get_bd_pins sc_mem3/aresetn]
    connect_bd_intf_net [get_bd_intf_pins rolling/m_axi_gmem] [get_bd_intf_pins sc_mem3/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins sc_mem3/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP3_FPD]
    }
    connect_bd_net [get_bd_pins rolling/interrupt] [get_bd_pins irq_concat/In$::m7_irq(rolling)]
    # ---- end v3 L2 ----
    # ---- MERGE9P: memk (AXI-Lite only) and the pl_ps_irq1 interrupts of srch / memk ----
    set memk_m {}
    if {$memk} {
        create_bd_cell -type ip -vlnv [m7_vlnv_of memk] memk
        connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(memk)_AXI] [m7_lite_pin memk]
        set memk_m [m7_maxi_pins memk]
        if {[llength $memk_m] > 0 && !$lpd} { error "memk has m_axi master(s) '$memk_m': needs lpd (FPGA_M7_PL_LPD=1)" }
        # MERGE10: the BRAM master port of a MEMK4-type core (none on a MEMK3-type one)
        set memk_b {}
        foreach p [get_bd_intf_pins -quiet -of_objects [get_bd_cells memk] -filter {MODE == Master}] {
            if {[string match *:bram* [get_property VLNV $p]]} { lappend memk_b $p }
        }
        if {$memk_bram} {
            if {[llength $memk_b] != 1} { error "memk_if bram: expected one BRAM master port on memk, got '$memk_b'" }
            set bc [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl bc_memk]
            set_property -dict [list CONFIG.PROTOCOL {AXI4LITE} CONFIG.SINGLE_PORT_BRAM {1} CONFIG.DATA_WIDTH {32} \
                                     CONFIG.ECC_TYPE {0}] $bc
            set bm [create_bd_cell -type ip -vlnv xilinx.com:ip:blk_mem_gen bm_memk]
            set_property -dict [list CONFIG.Memory_Type {True_Dual_Port_RAM}] $bm
            foreach {k v} [list PROTOCOL AXI4LITE SINGLE_PORT_BRAM 1 DATA_WIDTH 32] {
                if {[get_property CONFIG.$k $bc] ne $v} { error "bc_memk CONFIG.$k is '[get_property CONFIG.$k $bc]', want '$v'" }
            }
            connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(memk_bram)_AXI] [get_bd_intf_pins bc_memk/S_AXI]
            connect_bd_intf_net [get_bd_intf_pins bc_memk/BRAM_PORTA] [get_bd_intf_pins bm_memk/BRAM_PORTA]
            connect_bd_intf_net [lindex $memk_b 0] [get_bd_intf_pins bm_memk/BRAM_PORTB]
            connect_bd_net $clk [get_bd_pins bc_memk/s_axi_aclk]
            connect_bd_net $aresetn [get_bd_pins bc_memk/s_axi_aresetn]
        } elseif {[llength $memk_b] > 0} {
            error "memk has a BRAM master port '$memk_b' (MEMK4-type core): set memk_if bram (FPGA_M7_PL_MEMK_IF=bram)"
        }
    }
    if {$srch || $memk} {
        set_property CONFIG.PSU__USE__IRQ1 {1} $ps
        set cat1 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat irq_concat1]
        set_property CONFIG.NUM_PORTS {8} $cat1
        set used1 {}
        foreach {c on} [list srch $srch memk $memk] { if {$on} { lappend used1 $::m7_irq1($c) } }
        for {set k 0} {$k < 8} {incr k} {
            if {[lsearch -exact $used1 $k] < 0} { connect_bd_net [get_bd_pins irq_zero/dout] [get_bd_pins irq_concat1/In$k] }
        }
        connect_bd_net [get_bd_pins irq_concat1/dout] [get_bd_pins ps/pl_ps_irq1]
    }
    if {$lpd} {
        # ---- MERGE7: small-traffic cores on S_AXI_LPD through one axi_interconnect (MERGE6 section 2.3) ----
        set lsi {}
        if {$detpost} {
            create_bd_cell -type ip -vlnv xilinx.com:hls:detpost_top:1.0 detpost
            connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(detpost)_AXI] [get_bd_intf_pins detpost/s_axi_control]
            lappend lsi detpost/m_axi_gmem
        }
        if {$prep} {
            create_bd_cell -type ip -vlnv [m7_vlnv_of prep] prep
            connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(prep)_AXI] [get_bd_intf_pins prep/s_axi_control]
            lappend lsi prep/m_axi_gmem
        }
        if {$planner} {
            create_bd_cell -type ip -vlnv [m7_vlnv_of planner] planner
            connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(planner)_AXI] [get_bd_intf_pins planner/s_axi_control]
            lappend lsi planner/m_axi_map_mem planner/m_axi_field_mem
        }
        # MERGE9P: new masters after the m5 ones (S00..S03 keep their m5 masters)
        if {$srch} {
            create_bd_cell -type ip -vlnv [m7_vlnv_of srch] srch
            connect_bd_intf_net [get_bd_intf_pins sc_ctrl/$mi(srch)_AXI] [m7_lite_pin srch]
            set m [m7_maxi_pins srch]
            if {[llength $m] == 0} { error "srch: no m_axi master" }
            foreach x $m { lappend lsi [string range $x 1 end] }
        }
        if {$obs && !$obs_hp1} {
            foreach x $obs_m { lappend lsi [string range $x 1 end] }
        }
        foreach x $memk_m { lappend lsi [string range $x 1 end] }
        foreach c {detpost prep planner} {
            if {[llength [get_bd_cells -quiet $c]] == 0} { continue }
            connect_bd_net $clk [get_bd_pins $c/ap_clk]
            connect_bd_net $aresetn [get_bd_pins $c/ap_rst_n]
            connect_bd_net [get_bd_pins $c/interrupt] [get_bd_pins irq_concat/In$::m7_irq($c)]
        }
        set ic [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect ic_lpd]
        set_property -dict [list CONFIG.NUM_SI [llength $lsi] CONFIG.NUM_MI {1} CONFIG.STRATEGY $lpd_ic \
                                 CONFIG.XBAR_DATA_WIDTH {128}] $ic
        # MERGE9Q: data FIFO of the new SIs only (S04 and later). Empty = unchanged (SAMD forces 2 = 512-deep packet
        # FIFO on every SI, 4.5 BRAM36 per 128-bit SI, and S<k>_HAS_DATA_FIFO is then a disabled parameter whose set is
        # silently ignored, IP_Flow 19-3374). Set: STRATEGY 0 (custom) with the advanced options, S00-S03 = 2 as SAMD
        # gives them, S04+ = lpd_fifo_new (1 = 32 deep, no BRAM; 0 = none); every value is read back.
        # MERGE9Q OOC: custom with all SIs at 2 is the calibration control for "custom == SAMD" (DESIGN_M6.md sec 7).
        if {$lpd_fifo_new ne ""} {
            if {[lsearch -exact {0 1 2} $lpd_fifo_new] < 0} { error "lpd_fifo_new: 0, 1 or 2, got '$lpd_fifo_new'" }
            if {$lpd_ic != 2} { error "lpd_fifo_new is for SAMD (FPGA_M7_PL_LPD_IC=samd)" }
            set_property -dict [list CONFIG.STRATEGY {0} CONFIG.ENABLE_ADVANCED_OPTIONS {1}] $ic
            for {set k 0} {$k < [llength $lsi]} {incr k} {
                set v [expr {$k < 4 ? 2 : $lpd_fifo_new}]
                set_property [format CONFIG.S%02d_HAS_DATA_FIFO $k] $v $ic
                set got [get_property [format CONFIG.S%02d_HAS_DATA_FIFO $k] $ic]
                if {$got ne $v} { error "ic_lpd S[format %02d $k]_HAS_DATA_FIFO is '$got' after setting '$v'" }
            }
        }
        connect_bd_net $clk [get_bd_pins ic_lpd/ACLK]
        connect_bd_net $icresetn [get_bd_pins ic_lpd/ARESETN]
        set k 0
        foreach m $lsi {
            set s [format S%02d $k]
            connect_bd_intf_net [get_bd_intf_pins $m] [get_bd_intf_pins ic_lpd/${s}_AXI]
            connect_bd_net $clk [get_bd_pins ic_lpd/${s}_ACLK]
            connect_bd_net $aresetn [get_bd_pins ic_lpd/${s}_ARESETN]
            incr k
        }
        connect_bd_net $clk [get_bd_pins ic_lpd/M00_ACLK]
        connect_bd_net $aresetn [get_bd_pins ic_lpd/M00_ARESETN]
        connect_bd_net $clk [get_bd_pins ps/saxi_lpd_aclk]
        connect_bd_intf_net [get_bd_intf_pins ic_lpd/M00_AXI] [get_bd_intf_pins ps/S_AXI_LPD]
    }
    foreach {c on} [list srch $srch memk $memk] {   ;# MERGE9P: pl_clk0, irq on pl_ps_irq1
        if {!$on} { continue }
        connect_bd_net $clk [get_bd_pins $c/ap_clk]
        connect_bd_net $aresetn [get_bd_pins $c/ap_rst_n]
        connect_bd_net [get_bd_pins $c/interrupt] [get_bd_pins irq_concat1/In$::m7_irq1($c)]
    }
    connect_bd_net [get_bd_pins irq_concat/dout] [get_bd_pins ps/pl_ps_irq0]

    # ---------------- Address map ----------------
    set ctrl_space [get_bd_addr_spaces ps/Data]
    set ctrl_slaves {sysinfo sysinfo/s_axi frame frame/s_axi_control
                     ar ar/s_axi_control
                     permit permit/s_axi roi roi/s_axi_control
                     rolling rolling/s_axi_control}
    if {!$has_ar} {
        set ctrl_slaves {sysinfo sysinfo/s_axi frame frame/s_axi_control
                         permit permit/s_axi roi roi/s_axi_control
                         rolling rolling/s_axi_control}
    }
    if {$detpost} { lappend ctrl_slaves detpost detpost/s_axi_control }
    if {$prep}    { lappend ctrl_slaves prep prep/s_axi_control }
    if {$planner} { lappend ctrl_slaves planner planner/s_axi_control }
    if {$obs} {   ;# MERGE9P: obs takes roi's window
        set k [lsearch -exact $ctrl_slaves roi]
        set ctrl_slaves [lreplace $ctrl_slaves $k [expr {$k + 1}] obs [string range [m7_lite_pin obs] 1 end]]
    }
    foreach {c on} [list srch $srch memk $memk] {
        if {$on} { lappend ctrl_slaves $c [string range [m7_lite_pin $c] 1 end] }
    }
    if {$memk_bram} { lappend ctrl_slaves memk_bram bc_memk/S_AXI }   ;# MERGE10
    foreach {c pin} $ctrl_slaves {
        set seg [get_bd_addr_segs -of_objects [get_bd_intf_pins $pin]]
        if {[llength $seg] != 1} { error "expected one address segment on $pin, got '$seg'" }
        set rng 64K
        if {$memk_bram && $c eq "memk"} { set rng $::m7_memk_ctrl_range }   ;# MERGE10: both inside slot 14
        if {$c eq "memk_bram"} { set rng $::m7_memk_bram_range }
        assign_bd_address -offset $::m7_addr($c) -range $rng \
            -target_address_space $ctrl_space $seg
    }
    # DMA masters may reach low DDR only (0x0000_0000..0x7FFF_FFFF) through
    # their own HP port: no OCM, QSPI, PCIe or PS peripheral windows. The
    # hand-written PS config has no 4 GiB K26 DDR preset, so the tool offers
    # no DDR_HIGH aperture; CMA/u-dma-buf buffers live in low DDR anyway and
    # the PS driver refuses any buffer whose physical range is not below 2 GiB.
    set dma_spaces {frame/Data_m_axi_gmem SAXIGP2 HP0
                        ar/Data_m_axi_gmem0 SAXIGP4 HP2 ar/Data_m_axi_gmem1 SAXIGP4 HP2
                        roi/Data_m_axi_gmem SAXIGP3 HP1
                        rolling/Data_m_axi_gmem SAXIGP5 HP3}
    if {!$has_ar} {
        set dma_spaces {frame/Data_m_axi_gmem SAXIGP2 HP0
                        roi/Data_m_axi_gmem SAXIGP3 HP1
                        rolling/Data_m_axi_gmem SAXIGP5 HP3}
    }
    if {$detpost_hp3} { lappend dma_spaces detpost/Data_m_axi_gmem SAXIGP5 HP3 }
    if {$lpd && $detpost} { lappend dma_spaces detpost/Data_m_axi_gmem SAXIGP6 LPD }
    if {$prep}    { lappend dma_spaces prep/Data_m_axi_gmem SAXIGP6 LPD }
    if {$planner} { lappend dma_spaces planner/Data_m_axi_map_mem SAXIGP6 LPD planner/Data_m_axi_field_mem SAXIGP6 LPD }
    if {$obs} {   ;# MERGE9P: roi's HP1 space is gone; obs on HP1 or on LPD
        set k [lsearch -exact $dma_spaces roi/Data_m_axi_gmem]
        set dma_spaces [lreplace $dma_spaces $k [expr {$k + 2}]]
        foreach x $obs_m {
            if {$obs_hp1} { lappend dma_spaces obs/Data_[get_property NAME $x] SAXIGP3 HP1 } else { lappend dma_spaces obs/Data_[get_property NAME $x] SAXIGP6 LPD }
        }
    }
    if {$srch} { foreach x [m7_maxi_pins srch] { lappend dma_spaces srch/Data_[get_property NAME $x] SAXIGP6 LPD } }
    foreach x $memk_m { lappend dma_spaces memk/Data_[get_property NAME $x] SAXIGP6 LPD }
    if {$has_dpu} {
        set seg [get_bd_addr_segs -of_objects [get_bd_intf_pins dpu/S_AXI]]
        if {[llength $seg] != 1} { error "expected one address segment on dpu/S_AXI, got '$seg'" }
        assign_bd_address -offset $::m7_dpu_base -range 16M -target_address_space [get_bd_addr_spaces ps/Data] $seg
        lappend dma_spaces dpu/DPU0_M_AXI_DATA0 SAXIGP4 HP2 dpu/DPU0_M_AXI_DATA1 SAXIGP1 HPC1 \
                           dpu/DPU0_M_AXI_INSTR SAXIGP0 HPC0
    }
    foreach {sp gp hp} $dma_spaces {
        set space [get_bd_addr_spaces $sp]
        if {[llength $space] != 1} { error "missing address space $sp" }
        foreach ap {DDR_LOW} {
            set seg [get_bd_addr_segs ps/$gp/${hp}_$ap]
            if {[llength $seg] != 1} { error "missing segment ps/$gp/${hp}_$ap" }
            assign_bd_address -target_address_space $space $seg
        }
    }

    validate_bd_design
    save_bd_design
    return [get_files $bd_dir/$design/$design.bd]
}
