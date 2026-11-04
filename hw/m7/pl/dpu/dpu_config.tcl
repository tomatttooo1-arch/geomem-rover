# DPUCZDX8G v4.1 (Vitis AI 3.0 DPU TRD, Vivado 2022.2) configuration for the
# nationals D0 probe (2026-11-02). Shared by the OOC probe and m7_bd.tcl so the
# out-of-context numbers and the full-board build use the same IP settings.
# TRD defaults (prj/Vivado/hw/scripts/trd_prj.tcl) except: size (ARCH), one core,
# URAM per DPU (K26 BRAM is short: B1024 all-BRAM needs 104 BRAM36), and clock
# gating off (two MMCM outputs with matched routing instead of BUFGCE_DIV).
# zynquplus values that the IP's bd.tcl init proc sets inside a block design are
# repeated here so an OOC create_ip matches the BD instance.
proc m7_dpu_props {arch uram} {
    return [list \
        CONFIG.VER_DPU_NUM {1} \
        CONFIG.ARCH $arch \
        CONFIG.ARCH_IMG_BKGRP {2} \
        CONFIG.LOAD_AUGM {1} \
        CONFIG.CONV_RELU_ADDON {3} \
        CONFIG.ALU_PARALLEL_USER {4} \
        CONFIG.ALU_LEAKYRELU {0} \
        CONFIG.SFM_ENA {0} \
        CONFIG.SAVE_ARGMAX_ENA {1} \
        CONFIG.S_AXI_CLK_INDEPENDENT {1} \
        CONFIG.CLK_GATING_ENA {0} \
        CONFIG.CONV_DSP_CASC_MAX {4} \
        CONFIG.CONV_DSP_ACCU_ENA {1} \
        CONFIG.URAM_N_USER $uram \
        CONFIG.TIMESTAMP_ENA {1} \
    ]
}
proc m7_dpu_zynquplus_props {} {
    return [list CONFIG.AXI_PROTOCOL {1} CONFIG.ARCH_HP_BW {3} CONFIG.VER_CHIP_PART {3} CONFIG.DSP48_VER {DSP48E2}]
}
