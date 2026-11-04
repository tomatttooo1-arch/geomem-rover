# Diagnostic A/B build over the unchanged production object-evidence source.
foreach key {FPGA_M7_OBJECT_PORT_ARM FPGA_M7_OBJECT_PORT_PART
             FPGA_M7_OBJECT_PORT_BUILD FPGA_M7_OBJECT_PORT_CLOCK_NS
             FPGA_M7_OBJECT_PORT_SOURCE_COMMIT} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running the object port probe"
    }
}
set arm $::env(FPGA_M7_OBJECT_PORT_ARM)
if {$arm ne "A" && $arm ne "B"} { error "Probe arm must be exactly A or B" }
if {![regexp {^[0-9a-f]{40}$} $::env(FPGA_M7_OBJECT_PORT_SOURCE_COMMIT)]} {
    error "FPGA_M7_OBJECT_PORT_SOURCE_COMMIT must be a full lowercase Git SHA"
}
if {![string is double -strict $::env(FPGA_M7_OBJECT_PORT_CLOCK_NS)] ||
    $::env(FPGA_M7_OBJECT_PORT_CLOCK_NS) <= 0} {
    error "FPGA_M7_OBJECT_PORT_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_M7_OBJECT_PORT_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside the source directory"
}
set run_dir [file dirname $build]
file mkdir $run_dir
set metadata [file join $run_dir PROBE_METADATA.txt]
if {[file exists $metadata]} { error "Probe metadata already exists" }
set meta [open $metadata w]
puts $meta "probe=object_xyz_port_counterfactual"
puts $meta "arm=$arm"
puts $meta "source_commit=$::env(FPGA_M7_OBJECT_PORT_SOURCE_COMMIT)"
puts $meta "part=$::env(FPGA_M7_OBJECT_PORT_PART)"
puts $meta "clock_ns=$::env(FPGA_M7_OBJECT_PORT_CLOCK_NS)"
foreach name {object_evidence.cpp object_evidence.hpp object_port_testbench.cpp
              run_object_port_probe.tcl} {
    set path [file join $src $name]
    puts $meta "source_[exec sha256sum $path]"
}
if {$arm eq "A"} {
    puts $meta "directive=none_original_flat_xyz_local"
} else {
    puts $meta "directive=array_partition_cyclic_factor3_dim1_xyz_local"
}
close $meta

cd $run_dir
open_project [file tail $build]
set_top fpga_m7_object_evidence
add_files "$src/object_evidence.cpp" -cflags "-std=c++11 -ffp-contract=off"
add_files -tb "$src/object_port_testbench.cpp" -cflags "-std=c++11 -ffp-contract=off"
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_OBJECT_PORT_PART)
create_clock -period $::env(FPGA_M7_OBJECT_PORT_CLOCK_NS)
if {$arm eq "B"} {
    # Diagnostic only: this intentionally changes the physical RTL port banks.
    # It does not modify the C ABI or the production source and is not a
    # candidate interface for host or board integration.
    set_directive_array_partition -type cyclic -factor 3 -dim 1 \
        fpga_m7_object_evidence xyz_local
}
csim_design
csynth_design
cosim_design -rtl verilog
exit
