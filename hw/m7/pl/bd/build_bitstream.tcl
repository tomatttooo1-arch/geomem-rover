# Non-project Vivado 2022.2 build: BD -> synth -> place/route -> bitstream.
#
# Deliberately avoids `launch_runs`: on this host the project launch_runs
# parent crashed in libudev/WebTalk host-info (M7_DENSE_WATCHDOG_RESULTS
# 2026-09-21). The BD is generated with synth_checkpoint_mode None, so every
# IP is synthesized inside one synth_design call; no out-of-context runs.
#
# Required environment:
#   FPGA_M7_PL_BUILD     fresh output directory (must not exist)
#   FPGA_M7_PL_IPREPO    directory with the exported HLS IPs
#   FPGA_M7_PL_CLK_MHZ   pl_clk0 frequency the design is timed for
#   FPGA_M7_PL_BUILD_ID  32-bit hex build id (git short hash), e.g. 0x054b2860
#   FPGA_M7_PL_BUILD_DATE 0xYYYYMMDD
# Optional: FPGA_M7_PL_THREADS (default 8), FPGA_M7_PL_PART.
# Optional FPGA_M7_PL_CLK1_MHZ (v3b, 2026-10-26): enable pl_clk1 at that
# frequency for the compute domain (see m7_bd.tcl); unset/0 = single clock.
# Optional implementation directives (2026-10-26, timing-closure ablation;
# unset = the tool default, i.e. the same commands as before):
#   FPGA_M7_PL_PLACE_DIRECTIVE  place_design -directive
#   FPGA_M7_PL_PHYSOPT_DIRECTIVE phys_opt_design -directive
#   FPGA_M7_PL_ROUTE_DIRECTIVE  route_design -directive
# Build-speed options (2026-11-02, nationals; unset = off):
#   FPGA_M7_PL_INCR_DCP=<routed.dcp>  incremental implementation: after opt_design,
#       read_checkpoint -incremental <dcp>; writes incremental_reuse_pre_place.rpt
#       and incremental_reuse_post_place.rpt. The reference must be the same part
#       and the same block-design structure; heavily changed modules reuse little.
#   FPGA_M7_PL_INCR_DIRECTIVE  read_checkpoint -incremental -directive
#       (RuntimeOptimized | TimingClosure | Quick); unset = tool default.
#   FPGA_M7_PL_POSTPLACE_WNS_MIN=<ns>  post-place early stop: after place and
#       phys_opt, write post_place.dcp; if setup WNS < <ns>, stamp and exit 3
#       (no routing). The launcher records exit 3 as POSTPLACE_STOP, not a crash.
# timing_place.rpt (post-place timing summary) is always written.
# nationals MERGE4 (2026-11-03; unset = off, exactly the commands as before):
#   FPGA_M7_PL_MEM_DIRECT=1  no sc_mem0/1/3: HP0/HP1 32 bit, HP3 64 bit, cores connected
#       straight to them (m7_bd.tcl); CLOCKS.txt gets mem_direct=1 (package_app then
#       programs the matching afi0 widths in the overlay).
#   FPGA_M7_PL_IMPL_STRATEGY=c1  the SLIM_AUDIT2 exp/c1 implementation: after the default
#       opt_design a second opt_design -control_set_merge -merge_equivalent_drivers;
#       place AltSpreadLogic_high, phys_opt AggressiveExplore, route AlternateCLBRouting
#       (each only where its FPGA_M7_PL_*_DIRECTIVE is unset); after route, if setup WNS < 0,
#       phys_opt_design -directive AggressiveExplore once more. Writes cs_opt_default.rpt,
#       cs_opt_merge.rpt, cs_route.rpt (control sets) and congestion_place.rpt. Meant for a
#       from-scratch build (do not combine with FPGA_M7_PL_INCR_DCP).
# nationals MERGE7 (2026-11-04; unset/0 = off, exactly the commands as before; m7_bd.tcl header):
#   FPGA_M7_PL_GEO_FX=1   assert the frame IP in FPGA_M7_PL_IPREPO is the GEO_FX3 fixed-point core.
#   FPGA_M7_PL_LPD=1      S_AXI_LPD (128 bit) + axi_interconnect ic_lpd; detpost moves there from sc_hp3.
#   FPGA_M7_PL_LPD_IC=sasd|samd  ic_lpd STRATEGY 1 (shared access, default) or 2 (shared address multiple data).
#   FPGA_M7_PL_PREP=1     pre-processing core (slot 11, irq 6) from FPGA_M7_PL_PREP_IP.
#   FPGA_M7_PL_PLANNER=1  planner_field_dual (slot 12, irq 1) from FPGA_M7_PL_PLANNER_IP.
#   FPGA_M7_PL_PREP_IP / FPGA_M7_PL_PLANNER_IP = an exported IP: either the HLS export.zip (extracted into
#       <build>/ip_extra/<name>) or a directory holding its component.xml. Switching the pre-processing core
#       from s15 to s18 = pointing FPGA_M7_PL_PREP_IP at the other export. Optional
#       FPGA_M7_PL_PREP_IP_SHA256 / FPGA_M7_PL_PLANNER_IP_SHA256 = required sha256 prefix of that zip.
#       CLOCKS.txt records the path and the sha256 (zip, or the directory's component.xml).
#   FPGA_M7_PL_WRITE_BD_TCL=<file>  write_bd_tcl of the generated block design (default-off diff check).
# nationals MERGE9P (2026-11-04; unset/0 = off, exactly the commands as before; m7_bd.tcl header, MERGE9P/DESIGN_M6.md):
#   FPGA_M7_PL_SRCH=1 + FPGA_M7_PL_SRCH_IP   probabilistic search core (slot 13, pl_ps_irq1 bit 0, ic_lpd), needs LPD.
#   FPGA_M7_PL_MEMK=1 + FPGA_M7_PL_MEMK_IP   semantic memory core (slot 14, pl_ps_irq1 bit 1, AXI-Lite only).
#   FPGA_M7_PL_MEMK_IF=bram   MERGE10: MEMK4-type memk (ap_ctrl + one BRAM port): adds bc_memk (AXI BRAM controller,
#                            AXI4-Lite) + bm_memk (2048x32 true dual port) in slot 14; unset/lite = as before.
#   FPGA_M7_PL_OBS=1  + FPGA_M7_PL_OBS_IP    observation-and-verification core, replaces roi (slot 9, irq bit 2);
#       FPGA_M7_PL_OBS_PORT=hp1 (default: its m_axi straight onto HP1, port width = the master's) | lpd (ic_lpd).
#   Each *_IP takes an export.zip or an IP directory like PREP_IP; *_IP_SHA256 = required sha256 prefix.
#   The VLNV of every extra IP (prep, planner too) is read from its component.xml (::m7_vlnv), so a planner IP
#   with a new version number needs no BD change.
#   FPGA_M7_PL_LPD_FIFO_NEW=0|1|2 (MERGE9Q) data FIFO depth of the NEW ic_lpd SIs (S04+): unset = tool default (SAMD: 2,
#       512-deep packet FIFO = 4.5 BRAM36 per 128-bit SI), 1 = 32 deep, 0 = none. S00-S03 (m5) untouched.
#   FPGA_M7_PL_INCR_WITH_C1=1  allow FPGA_M7_PL_INCR_DCP together with FPGA_M7_PL_IMPL_STRATEGY=c1: keeps c1's second
#       opt_design (so the netlist names match a c1 reference) and its post-route phys_opt, but does NOT set c1's
#       place/route directives (the incremental flow takes FPGA_M7_PL_INCR_DIRECTIVE instead).
foreach key {FPGA_M7_PL_BUILD FPGA_M7_PL_IPREPO FPGA_M7_PL_CLK_MHZ FPGA_M7_PL_BUILD_ID
             FPGA_M7_PL_BUILD_DATE} {
    if {![info exists ::env($key)] || $::env($key) eq ""} { error "Set $key" }
}
set part xck26-sfvc784-2LV-c
if {[info exists ::env(FPGA_M7_PL_PART)]} { set part $::env(FPGA_M7_PL_PART) }
set threads 8
if {[info exists ::env(FPGA_M7_PL_THREADS)]} { set threads $::env(FPGA_M7_PL_THREADS) }
set out [file normalize $::env(FPGA_M7_PL_BUILD)]
if {[file exists $out]} { error "Build destination exists: $out" }
file mkdir $out
set here [file dirname [file normalize [info script]]]
set pl [file normalize "$here/.."]
set m7 [file normalize "$pl/.."]
set clk_mhz $::env(FPGA_M7_PL_CLK_MHZ)
set clk1_mhz 0
if {[info exists ::env(FPGA_M7_PL_CLK1_MHZ)] && $::env(FPGA_M7_PL_CLK1_MHZ) ne ""} { set clk1_mhz $::env(FPGA_M7_PL_CLK1_MHZ) }
set clk1_act ""
# nationals AR2NANO (2026-11-02): FPGA_M7_PL_NO_AR=1 removes the ar overlay core only (car screen
# drawn on the Nano; see m7_bd.tcl). Unset/0 = the design with ar, same commands as before.
set no_ar 0
if {[info exists ::env(FPGA_M7_PL_NO_AR)] && $::env(FPGA_M7_PL_NO_AR) ne "" && $::env(FPGA_M7_PL_NO_AR) ne "0"} { set no_ar 1 }
# nationals D0 (2026-11-02): optional DPUCZDX8G. FPGA_M7_PL_DPU_ARCH (e.g. 1024) removes the
# ar core and adds the DPU; FPGA_M7_PL_DPU_IPREPO = folder holding DPUCZDX8G_v4_1_0.
set dpu_arch 0; set dpu_uram 0; set dpu_mhz 300
if {[info exists ::env(FPGA_M7_PL_DPU_ARCH)] && $::env(FPGA_M7_PL_DPU_ARCH) ne ""} { set dpu_arch $::env(FPGA_M7_PL_DPU_ARCH) }
if {[info exists ::env(FPGA_M7_PL_DPU_URAM)] && $::env(FPGA_M7_PL_DPU_URAM) ne ""} { set dpu_uram $::env(FPGA_M7_PL_DPU_URAM) }
if {[info exists ::env(FPGA_M7_PL_DPU_MHZ)] && $::env(FPGA_M7_PL_DPU_MHZ) ne ""} { set dpu_mhz $::env(FPGA_M7_PL_DPU_MHZ) }
set mem_direct 0
if {[info exists ::env(FPGA_M7_PL_MEM_DIRECT)] && $::env(FPGA_M7_PL_MEM_DIRECT) ne "" && $::env(FPGA_M7_PL_MEM_DIRECT) ne "0"} { set mem_direct 1 }
# nationals MERGE5A (2026-11-03; unset/0 = off, exactly the commands as before):
# FPGA_M7_PL_DPU_CLK_GATE=1 DPU clock gating (needs FPGA_M7_PL_DPU_ARCH); FPGA_M7_PL_DETPOST=1 adds the
# detection post-processing core (slot 10, irq bit 3, HP3 shared with rolling via sc_hp3).
set dpu_clk_gate 0; set detpost 0
if {[info exists ::env(FPGA_M7_PL_DPU_CLK_GATE)] && $::env(FPGA_M7_PL_DPU_CLK_GATE) ne "" && $::env(FPGA_M7_PL_DPU_CLK_GATE) ne "0"} { set dpu_clk_gate 1 }
if {[info exists ::env(FPGA_M7_PL_DETPOST)] && $::env(FPGA_M7_PL_DETPOST) ne "" && $::env(FPGA_M7_PL_DETPOST) ne "0"} { set detpost 1 }
proc env_on {key} {
    return [expr {[info exists ::env($key)] && $::env($key) ne "" && $::env($key) ne "0"}]
}
set geo_fx [env_on FPGA_M7_PL_GEO_FX]; set lpd [env_on FPGA_M7_PL_LPD]
set prep [env_on FPGA_M7_PL_PREP]; set planner [env_on FPGA_M7_PL_PLANNER]
set srch [env_on FPGA_M7_PL_SRCH]; set memk [env_on FPGA_M7_PL_MEMK]; set obs [env_on FPGA_M7_PL_OBS]
set memk_if lite   ;# MERGE10: FPGA_M7_PL_MEMK_IF=lite|bram (bram = MEMK4 shell, m7_bd.tcl header)
if {[info exists ::env(FPGA_M7_PL_MEMK_IF)] && $::env(FPGA_M7_PL_MEMK_IF) ne ""} { set memk_if $::env(FPGA_M7_PL_MEMK_IF) }
set obs_port hp1
if {[info exists ::env(FPGA_M7_PL_OBS_PORT)] && $::env(FPGA_M7_PL_OBS_PORT) ne ""} { set obs_port $::env(FPGA_M7_PL_OBS_PORT) }
set lpd_fifo_new ""   ;# MERGE9Q: FPGA_M7_PL_LPD_FIFO_NEW (DESIGN_M6.md section 8 option A)
if {[info exists ::env(FPGA_M7_PL_LPD_FIFO_NEW)]} { set lpd_fifo_new $::env(FPGA_M7_PL_LPD_FIFO_NEW) }
set lpd_ic 1; set lpd_ic_name sasd
if {[info exists ::env(FPGA_M7_PL_LPD_IC)] && $::env(FPGA_M7_PL_LPD_IC) ne ""} {
    set lpd_ic_name $::env(FPGA_M7_PL_LPD_IC)
    if {$lpd_ic_name eq "sasd"} { set lpd_ic 1 } elseif {$lpd_ic_name eq "samd"} { set lpd_ic 2 } else {
        error "FPGA_M7_PL_LPD_IC: unknown '$lpd_ic_name' (known: sasd, samd)"
    }
}
# MERGE7: an extra IP (export.zip or IP directory) -> {repo_dir path sha256}
proc extra_ip {name key out} {
    if {![info exists ::env($key)] || $::env($key) eq ""} { error "$name switch on: set $key (export.zip or IP directory)" }
    set p [file normalize $::env($key)]
    if {[file isfile $p]} {
        set sha [lindex [exec sha256sum $p] 0]
        set d "$out/ip_extra/$name"
        file mkdir $d
        exec unzip -q -o $p -d $d
    } elseif {[file isdirectory $p]} {
        set d $p
        if {![file exists "$d/component.xml"]} { error "$key: no component.xml in $d" }
        set sha [lindex [exec sha256sum "$d/component.xml"] 0]
    } else { error "$key not found: $p" }
    if {![file exists "$d/component.xml"]} { error "$key: no component.xml after extracting $p" }
    # MERGE9P: the IP's own VLNV (first vendor/library/name/version in component.xml) -> ::m7_vlnv(<name>)
    set fh [open "$d/component.xml"]; set x [read $fh]; close $fh
    set v {}
    foreach t {vendor library name version} {
        if {![regexp "<spirit:$t>(\[^<\]+)</spirit:$t>" $x -> f]} { error "$key: no spirit:$t in $d/component.xml" }
        lappend v $f
    }
    set ::m7_vlnv($name) [join $v :]
    set want "${key}_SHA256"
    if {[info exists ::env($want)] && $::env($want) ne ""} {
        if {[string first [string tolower $::env($want)] $sha] != 0} { error "$key sha256 $sha does not start with $::env($want)" }
    }
    return [list $d $p $sha]
}
set impl_strategy ""
if {[info exists ::env(FPGA_M7_PL_IMPL_STRATEGY)] && $::env(FPGA_M7_PL_IMPL_STRATEGY) ne ""} {
    set impl_strategy $::env(FPGA_M7_PL_IMPL_STRATEGY)
    if {$impl_strategy ne "c1"} { error "FPGA_M7_PL_IMPL_STRATEGY: unknown '$impl_strategy' (known: c1)" }
    set c1_incr 0
    if {[info exists ::env(FPGA_M7_PL_INCR_DCP)] && $::env(FPGA_M7_PL_INCR_DCP) ne ""} {
        if {![env_on FPGA_M7_PL_INCR_WITH_C1]} {
            error "FPGA_M7_PL_IMPL_STRATEGY=c1 is a from-scratch strategy; unset FPGA_M7_PL_INCR_DCP (or set FPGA_M7_PL_INCR_WITH_C1=1)"
        }
        set c1_incr 1   ;# MERGE9P: c1 opt steps only; the incremental directive governs place/route
    }
    if {!$c1_incr} {
    foreach {k d} {FPGA_M7_PL_PLACE_DIRECTIVE AltSpreadLogic_high FPGA_M7_PL_PHYSOPT_DIRECTIVE AggressiveExplore
                   FPGA_M7_PL_ROUTE_DIRECTIVE AlternateCLBRouting} {
        if {![info exists ::env($k)] || $::env($k) eq ""} { set ::env($k) $d }
    }
    }
}
set_param general.maxThreads $threads
# WebTalk off (no usage upload). NOTE: this alone does NOT avoid the host's
# segfault at the end of route_design (libudev called from the license
# library); the fix is the libudev LD_PRELOAD in tools/launch_build.sh.
config_webtalk -user off

proc directive_args {key} {
    if {[info exists ::env($key)] && $::env($key) ne ""} { return [list -directive $::env($key)] }
    return {}
}
proc stamp {msg} { puts "FPGA_M7_PL_STAGE [clock format [clock seconds] -format %Y-%m-%dT%H:%M:%S] $msg" }

stamp "start part=$part clk=${clk_mhz}MHz out=$out"
# Optional resume: FPGA_M7_PL_FROM_SYNTH=<earlier build dir> reuses that build's
# post_synth.dcp (same sources) and its BD-derived address map/.hwh.
set from_synth ""
if {[info exists ::env(FPGA_M7_PL_FROM_SYNTH)]} { set from_synth [file normalize $::env(FPGA_M7_PL_FROM_SYNTH)] }
if {$from_synth ne ""} {
    foreach f [concat [glob $from_synth/*.hwh] [list $from_synth/address_map.csv \
                   $from_synth/utilization_synth.rpt $from_synth/timing_synth.rpt] \
                   [glob -nocomplain $from_synth/CLOCKS.txt]] {
        file copy $f $out/
    }
    set fh [open "$out/RESUMED_FROM.txt" w]; puts $fh $from_synth; close $fh
    stamp "resume from $from_synth/post_synth.dcp"
    open_checkpoint "$from_synth/post_synth.dcp"
} else {
create_project -in_memory -part $part
set_property target_language Verilog [current_project]
# Module references need automatic compile order (in-memory default is manual).
set_property source_mgmt_mode All [current_project]
set repos [list [file normalize $::env(FPGA_M7_PL_IPREPO)]]
if {$dpu_arch > 0} { lappend repos [file normalize $::env(FPGA_M7_PL_DPU_IPREPO)] }
set extra_ips {}
if {$prep}    { set prep_ip [extra_ip prep FPGA_M7_PL_PREP_IP $out]; lappend repos [lindex $prep_ip 0]; lappend extra_ips prep $prep_ip }
if {$planner} { set planner_ip [extra_ip planner FPGA_M7_PL_PLANNER_IP $out]; lappend repos [lindex $planner_ip 0]; lappend extra_ips planner $planner_ip }
foreach {n on} [list srch $srch memk $memk obs $obs] {   ;# MERGE9P
    if {$on} { set ip [extra_ip $n FPGA_M7_PL_[string toupper $n]_IP $out]; lappend repos [lindex $ip 0]; lappend extra_ips $n $ip }
}
set_property ip_repo_paths $repos [current_project]
update_ip_catalog
# v3fc: the watchdog (m7_watchdog_axil + display_watchdog.sv) and encoder UART
# (m7_enc_uart_axil) shells are no longer in the block design, so not read.
read_verilog [list "$pl/rtl/m7_sysinfo_axil.v" \
                  "$pl/rtl/m7_timebase.v" "$pl/rtl/m7_permit_axil.v" \
                  "$pl/rtl/m7_cdc_single.v"]
update_compile_order -fileset sources_1

source "$here/m7_bd.tcl"
# DPU1: both switches (AR2NANO2 merge plan); defaults reproduce the previous calls'
# commands exactly (dpu_arch 0, no_ar 0 = the design with ar).
set bd [m7_create_bd "$out/bd" $clk_mhz $::env(FPGA_M7_PL_BUILD_ID) $::env(FPGA_M7_PL_BUILD_DATE) $clk1_mhz $dpu_arch $dpu_uram $dpu_mhz $no_ar $mem_direct $dpu_clk_gate $detpost $geo_fx $lpd $prep $planner $lpd_ic $srch $memk $obs $obs_port $lpd_fifo_new $memk_if]
if {[info exists ::env(FPGA_M7_PL_WRITE_BD_TCL)] && $::env(FPGA_M7_PL_WRITE_BD_TCL) ne ""} {
    write_bd_tcl -force [file normalize $::env(FPGA_M7_PL_WRITE_BD_TCL)]
}
set clk0_act [get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ [get_bd_cells ps]]
if {$clk1_mhz > 0} {
    set clk1_act [get_property CONFIG.PSU__CRL_APB__PL1_REF_CTRL__ACT_FREQMHZ [get_bd_cells ps]]
}
set fh [open "$out/CLOCKS.txt" w]
puts $fh "clk0_mhz_actual=$clk0_act"
if {$clk1_mhz > 0} { puts $fh "clk1_mhz_target=$clk1_mhz"; puts $fh "clk1_mhz_actual=$clk1_act" }
# no_ar=1 whenever ar is absent (also for DPU builds): package_app drops the m7_ar DT node on it
if {$no_ar || $dpu_arch > 0} { puts $fh "no_ar=1" }
if {$mem_direct} {
    set psc [get_bd_cells ps]
    puts $fh "mem_direct=1"
    puts $fh "saxi_widths=HP0 [get_property CONFIG.PSU__SAXIGP2__DATA_WIDTH $psc] HP1 [get_property CONFIG.PSU__SAXIGP3__DATA_WIDTH $psc] HP3 [get_property CONFIG.PSU__SAXIGP5__DATA_WIDTH $psc]"
}
if {$impl_strategy ne ""} { puts $fh "impl_strategy=$impl_strategy" }
if {$dpu_clk_gate} { puts $fh "dpu_clk_gate=1" }
if {$geo_fx} { puts $fh "geo_fx=1" }
if {$lpd} { puts $fh "lpd=1"; puts $fh "lpd_ic=$lpd_ic_name" }
foreach {n ip} $extra_ips {
    puts $fh "$n=1"; puts $fh "${n}_ip=[lindex $ip 1]"; puts $fh "${n}_ip_sha256=[lindex $ip 2]"; puts $fh "${n}_ip_repo=[lindex $ip 0]"
    if {[lsearch -exact {srch memk obs} $n] >= 0} { puts $fh "${n}_vlnv=$::m7_vlnv($n)" }   ;# MERGE9P
}
if {$obs} { puts $fh "obs_port=$obs_port" }
if {$lpd_fifo_new ne ""} { puts $fh "lpd_fifo_new=$lpd_fifo_new" }
if {$memk_if ne "lite"} { puts $fh "memk_if=$memk_if" }   ;# MERGE10 (absent = lite, the MERGE9P CLOCKS.txt)
if {$detpost || $lpd || $obs} {
    set psc [get_bd_cells ps]
    if {$detpost} { puts $fh "detpost=1" }
    set ws {}
    foreach n {0 1 2 3 4 5 6} {   ;# Vivado 2022.2 Tcl 8.5: no lmap
        if {[get_property CONFIG.PSU__USE__S_AXI_GP$n $psc] eq 1} {
            lappend ws "GP$n:[get_property CONFIG.PSU__SAXIGP${n}__DATA_WIDTH $psc]"
        } else { lappend ws "GP$n:off" }
    }
    puts $fh "saxi_widths_all=[join $ws { }]"
}
if {$dpu_arch > 0} {
    set cw [get_bd_cells clk_wiz_dpu]
    puts $fh "dpu_arch=B$dpu_arch"
    puts $fh "dpu_uram_user=$dpu_uram"
    puts $fh "dpu_clk_mhz_target=$dpu_mhz"
    # DPU1: CONFIG.CLKOUT*_OUT_FREQ is empty in a batch BD (D0 CLOCKS.txt); the output pins carry FREQ_HZ
    puts $fh "dpu_clk_hz_actual=[get_property CONFIG.FREQ_HZ [get_bd_pins clk_wiz_dpu/clk_dpu]]"
    puts $fh "dsp_clk_hz_actual=[get_property CONFIG.FREQ_HZ [get_bd_pins clk_wiz_dpu/clk_dsp]]"
    puts $fh "dpu_clk_wiz=PRIMITIVE [get_property CONFIG.PRIMITIVE $cw] PRIM_SOURCE [get_property CONFIG.PRIM_SOURCE $cw] MULT_F [get_property CONFIG.MMCM_CLKFBOUT_MULT_F $cw] DIVCLK [get_property CONFIG.MMCM_DIVCLK_DIVIDE $cw] OUT0_DIV [get_property CONFIG.MMCM_CLKOUT0_DIVIDE_F $cw] OUT1_DIV [get_property CONFIG.MMCM_CLKOUT1_DIVIDE $cw]"
    set d [get_bd_cells dpu]
    puts $fh "dpu_sum=URAM [get_property CONFIG.SUM_URAM_N $d] BRAM [get_property CONFIG.SUM_BRAM_N $d] DSP [get_property CONFIG.SUM_DSP_NUM $d] HPBW [get_property CONFIG.SUM_HP_DATA_BW $d] GPBW [get_property CONFIG.SUM_GP_DATA_BW $d] PROTO [get_property CONFIG.SUM_AXI_PROTOCOL $d]"
}
close $fh
stamp "clocks pl0=$clk0_act MHz pl1=[expr {$clk1_mhz > 0 ? $clk1_act : {off}}]"
set_property synth_checkpoint_mode None $bd
generate_target all $bd
stamp "bd generated"
if {$dpu_arch > 0} {
    foreach a [glob -nocomplain $out/bd/m7_pl/ip/*dpu*/arch.json] { file copy -force $a $out/arch.json }
}

# Address map as seen by each master, for the overlay/driver consistency check.
set fh [open "$out/address_map.csv" w]
puts $fh "master_space,segment,offset,range"
foreach space [get_bd_addr_spaces -of_objects [get_bd_cells *]] {
    foreach seg [get_bd_addr_segs -of_objects $space] {
        puts $fh "[get_property NAME $space],[get_property NAME $seg],[get_property OFFSET $seg],[get_property RANGE $seg]"
    }
}
close $fh
file copy -force {*}[glob "$out/bd/m7_pl/hw_handoff/*.hwh"] $out/
# FPGA_M7_PL_VALIDATE_ONLY=1 (2026-10-26): stop after the block design validated and its
# address map / .hwh were written (bd integration check, no synthesis).
if {[info exists ::env(FPGA_M7_PL_VALIDATE_ONLY)] && $::env(FPGA_M7_PL_VALIDATE_ONLY) eq "1"} {
    stamp "validate only: done"
    exit
}

set wrapper [make_wrapper -files $bd -top]
read_verilog $wrapper
# Board-pin constraints (Pmod: permit heartbeat out; the encoder UART input was removed in v3fc).
read_xdc "$pl/xdc/m7_pmod_permit_uart.xdc"
# DPU1: fixed site for the DPU clock MMCM (CLKC-56), implementation only (the cell exists after synthesis).
if {$dpu_arch > 0} {
    read_xdc "$pl/xdc/m7_dpu_clk.xdc"
    set_property used_in_synthesis false [get_files "$pl/xdc/m7_dpu_clk.xdc"]
}
stamp "synth_design"
synth_design -top m7_pl_wrapper -part $part -flatten_hierarchy rebuilt
write_checkpoint -force "$out/post_synth.dcp"
report_utilization -file "$out/utilization_synth.rpt"
report_timing_summary -max_paths 10 -file "$out/timing_synth.rpt"
}
stamp "opt_design"
opt_design
if {$impl_strategy eq "c1"} {
    report_control_sets -file "$out/cs_opt_default.rpt"
    stamp "opt_design -control_set_merge -merge_equivalent_drivers (strategy c1)"
    opt_design -control_set_merge -merge_equivalent_drivers
    report_control_sets -file "$out/cs_opt_merge.rpt"
}
set incr_dcp ""
if {[info exists ::env(FPGA_M7_PL_INCR_DCP)] && $::env(FPGA_M7_PL_INCR_DCP) ne ""} {
    set incr_dcp [file normalize $::env(FPGA_M7_PL_INCR_DCP)]
    if {![file exists $incr_dcp]} { error "FPGA_M7_PL_INCR_DCP not found: $incr_dcp" }
    set incr_args {}
    if {[info exists ::env(FPGA_M7_PL_INCR_DIRECTIVE)] && $::env(FPGA_M7_PL_INCR_DIRECTIVE) ne ""} {
        set incr_args [list -directive $::env(FPGA_M7_PL_INCR_DIRECTIVE)]
    }
    stamp "read_checkpoint -incremental $incr_dcp $incr_args"
    read_checkpoint -incremental $incr_dcp {*}$incr_args
    set fh [open "$out/INCREMENTAL_FROM.txt" w]; puts $fh "$incr_dcp $incr_args"; close $fh
    report_incremental_reuse -file "$out/incremental_reuse_pre_place.rpt"
    stamp "incremental reuse report written (pre-place)"
}
stamp "place_design [directive_args FPGA_M7_PL_PLACE_DIRECTIVE]"
place_design {*}[directive_args FPGA_M7_PL_PLACE_DIRECTIVE]
phys_opt_design {*}[directive_args FPGA_M7_PL_PHYSOPT_DIRECTIVE]
if {$incr_dcp ne ""} { report_incremental_reuse -file "$out/incremental_reuse_post_place.rpt" }
set postplace_min ""
if {[info exists ::env(FPGA_M7_PL_POSTPLACE_WNS_MIN)] && $::env(FPGA_M7_PL_POSTPLACE_WNS_MIN) ne ""} {
    set postplace_min $::env(FPGA_M7_PL_POSTPLACE_WNS_MIN)
    if {![string is double -strict $postplace_min]} { error "FPGA_M7_PL_POSTPLACE_WNS_MIN not a number: $postplace_min" }
    # Checkpoint first, so a stopped probe can still be inspected or routed later.
    write_checkpoint -force "$out/post_place.dcp"
}
report_timing_summary -max_paths 20 -file "$out/timing_place.rpt"
if {$impl_strategy ne ""} { report_design_analysis -congestion -file "$out/congestion_place.rpt" }
set wns_place [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
stamp "post-place wns=$wns_place (stop below: [expr {$postplace_min eq {} ? {off} : $postplace_min}])"
if {$postplace_min ne "" && $wns_place ne "" && $wns_place < $postplace_min} {
    set fh [open "$out/POSTPLACE_STOP.txt" w]
    puts $fh "post_place_setup_wns_ns=$wns_place"
    puts $fh "threshold_ns=$postplace_min"
    close $fh
    stamp "POSTPLACE_STOP wns=$wns_place < $postplace_min ns: not routing (exit 3)"
    exit 3
}
stamp "route_design [directive_args FPGA_M7_PL_ROUTE_DIRECTIVE] (phys_opt [directive_args FPGA_M7_PL_PHYSOPT_DIRECTIVE])"
route_design {*}[directive_args FPGA_M7_PL_ROUTE_DIRECTIVE]
if {$impl_strategy eq "c1"} {
    set wns_r [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
    if {$wns_r ne "" && $wns_r < 0} {
        stamp "post-route wns=$wns_r < 0: phys_opt_design -directive AggressiveExplore (strategy c1)"
        phys_opt_design -directive AggressiveExplore
    }
}
# Checkpoint first, before any report, so a later crash cannot cost the route.
write_checkpoint -force "$out/routed.dcp"
stamp "routed.dcp written"
report_timing_summary -max_paths 20 -report_unconstrained -file "$out/timing_route.rpt"
report_utilization -file "$out/utilization_route.rpt"
report_utilization -hierarchical -hierarchical_depth 2 -file "$out/utilization_route_hier.rpt"
report_clock_utilization -file "$out/clock_utilization.rpt"
report_drc -file "$out/drc_route.rpt"
report_methodology -file "$out/methodology.rpt"
report_clock_interaction -file "$out/clock_interaction.rpt"
report_cdc -details -file "$out/cdc.rpt"
report_design_analysis -congestion -complexity -file "$out/congestion.rpt"
if {$impl_strategy ne ""} { report_control_sets -file "$out/cs_route.rpt" }
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
set fh [open "$out/SUMMARY.txt" w]
puts $fh "clk_mhz=$clk_mhz"
puts $fh "setup_wns_ns=$wns"
puts $fh "hold_whs_ns=$whs"
# Per-clock setup WNS (the worst path ending in each clock; includes the
# crossings INTO that clock that are timed).
foreach c [get_clocks] {
    set p [get_timing_paths -setup -to $c -max_paths 1 -nworst 1]
    if {$p ne ""} { puts $fh "setup_wns_[get_property NAME $c]_ns=[get_property SLACK $p]" }
    set p [get_timing_paths -hold -to $c -max_paths 1 -nworst 1]
    if {$p ne ""} { puts $fh "hold_whs_[get_property NAME $c]_ns=[get_property SLACK $p]" }
}
if {[file exists "$out/CLOCKS.txt"]} {
    set ch [open "$out/CLOCKS.txt"]; puts -nonewline $fh [read $ch]; close $ch
}
close $fh
stamp "timing wns=$wns whs=$whs"
write_bitstream -force "$out/m7_pl.bit"
stamp "bitstream written"
# Hardware platform (.xsa with the bitstream) needs the block design in the
# in-memory project, so only a from-source run can write it.
if {$from_synth eq ""} {
    write_hw_platform -fixed -include_bit -force -file "$out/m7_pl.xsa"
    stamp "xsa written"
} else {
    stamp "xsa skipped (resumed from post_synth.dcp, no block design in memory)"
}
stamp "all done"
