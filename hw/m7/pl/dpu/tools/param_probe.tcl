create_project -in_memory -part xck26-sfvc784-2LV-c
set_property ip_repo_paths ${FPGA_SHARED}/dpu_ip_vai30_20261102/extract/DPUCZDX8G_VAI_v3.0/dpu_ip [current_project]
update_ip_catalog
puts "VLNV [get_ipdefs *dpuczdx8g*]"
create_ip -vlnv xilinx.com:ip:dpuczdx8g:4.1 -module_name dpu_probe -dir ./ipdir
set ip [get_ips dpu_probe]
foreach arch {800 1024 1152} {
  foreach u {0 1 2 4 8 16 26 32 40 44 48} {
    if {[catch {set_property -dict [list CONFIG.ARCH $arch CONFIG.URAM_N_USER $u CONFIG.CONV_DSP_ACCU_ENA 1 CONFIG.ARCH_IMG_BKGRP 2] $ip} e]} { puts "PROBE arch=$arch uram=$u ERR [string range $e 0 200]"; continue }
    puts "PROBE arch=$arch uram_user=$u SUM_URAM_N=[get_property CONFIG.SUM_URAM_N $ip] SUM_BRAM_N=[get_property CONFIG.SUM_BRAM_N $ip] SUM_DSP_NUM=[get_property CONFIG.SUM_DSP_NUM $ip] UBANK_IMG=[get_property CONFIG.UBANK_IMG_N $ip] UBANK_WGT=[get_property CONFIG.UBANK_WGT_N $ip] UBANK_BIAS=[get_property CONFIG.UBANK_BIAS $ip]"
  }
}
set_property -dict [list CONFIG.ARCH 1024 CONFIG.URAM_N_USER 0] $ip
puts "DNNDK [get_property CONFIG.DNNDK_PRINT $ip]"
foreach p [lsort [list_property $ip CONFIG.*]] { puts "CFG $p=[get_property $p $ip]" }
