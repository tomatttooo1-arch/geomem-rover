# Object xyz_local store-bank diagnostic, one arm per fresh build (2026-09-26).
# A and B reproduce run_object_port_probe.tcl (A: production source, flat
# two-port xyz_local; B: same source + cyclic factor-3 partition).  C and D
# each change exactly one thing relative to B:
#   C  explicit bank view: xyz_local declared [N][3] (same bytes), complete
#      partition of dim 2, so the bank of each store is the constant row index;
#   D  B's flat cyclic-3 layout, store index computed as 3ull*n+row.
# No dependence pragma is used and no check is disabled.  Diagnostics only.
foreach key {FPGA_M7_OBJECT_BANK_ARM FPGA_M7_OBJECT_BANK_PART
             FPGA_M7_OBJECT_BANK_BUILD FPGA_M7_OBJECT_BANK_CLOCK_NS
             FPGA_M7_OBJECT_BANK_SOURCE_COMMIT} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running the object bank probe"
    }
}
set arm $::env(FPGA_M7_OBJECT_BANK_ARM)
array set design_of {A object_evidence.cpp B object_evidence.cpp
                     C probe/object_evidence_bank2d.cpp D probe/object_evidence_idx64.cpp}
array set header_of {A object_evidence.hpp B object_evidence.hpp
                     C probe/object_evidence_bank2d.hpp D object_evidence.hpp}
array set tb_of {A object_port_testbench.cpp B object_port_testbench.cpp
                 C probe/object_port_testbench_bank2d.cpp D object_port_testbench.cpp}
array set directive_of {A none_original_flat_xyz_local
                        B array_partition_cyclic_factor3_dim1_xyz_local
                        C explicit_bank_view_n3_partition_complete_dim2_xyz_local
                        D array_partition_cyclic_factor3_dim1_xyz_local_index64}
if {![info exists design_of($arm)]} { error "Probe arm must be A, B, C or D" }
if {![regexp {^[0-9a-f]{40}$} $::env(FPGA_M7_OBJECT_BANK_SOURCE_COMMIT)]} {
    error "FPGA_M7_OBJECT_BANK_SOURCE_COMMIT must be a full lowercase Git SHA"
}
if {![string is double -strict $::env(FPGA_M7_OBJECT_BANK_CLOCK_NS)] ||
    $::env(FPGA_M7_OBJECT_BANK_CLOCK_NS) <= 0} {
    error "FPGA_M7_OBJECT_BANK_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_M7_OBJECT_BANK_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside the source directory"
}
set run_dir [file dirname $build]
file mkdir $run_dir
set metadata [file join $run_dir PROBE_METADATA.txt]
if {[file exists $metadata]} { error "Probe metadata already exists" }
set meta [open $metadata w]
puts $meta "probe=object_xyz_bank_probe"
puts $meta "arm=$arm"
puts $meta "source_commit=$::env(FPGA_M7_OBJECT_BANK_SOURCE_COMMIT)"
puts $meta "part=$::env(FPGA_M7_OBJECT_BANK_PART)"
puts $meta "clock_ns=$::env(FPGA_M7_OBJECT_BANK_CLOCK_NS)"
foreach name [list $design_of($arm) $header_of($arm) $tb_of($arm)] {
    puts $meta "source_[exec sha256sum [file join $src $name]]"
}
puts $meta "source_[exec sha256sum [file normalize [info script]]]"
puts $meta "directive=$directive_of($arm)"
close $meta

cd $run_dir
open_project [file tail $build]
set_top fpga_m7_object_evidence
add_files [file join $src $design_of($arm)] -cflags "-std=c++11 -ffp-contract=off"
add_files -tb [file join $src $tb_of($arm)] -cflags "-std=c++11 -ffp-contract=off"
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_OBJECT_BANK_PART)
create_clock -period $::env(FPGA_M7_OBJECT_BANK_CLOCK_NS)
if {$arm eq "B" || $arm eq "D"} {
    set_directive_array_partition -type cyclic -factor 3 -dim 1 \
        fpga_m7_object_evidence xyz_local
} elseif {$arm eq "C"} {
    set_directive_array_partition -type complete -dim 2 \
        fpga_m7_object_evidence xyz_local
}
csim_design
csynth_design
cosim_design -rtl verilog
exit
