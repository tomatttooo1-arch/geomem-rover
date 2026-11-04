## DPU1 (2026-11-03): read only when the DPU is built (FPGA_M7_PL_DPU_ARCH > 0), implementation only.
## clk_wiz_dpu is an explicit MMCME4 fed by pl_clk0 straight off its BUFG_PS (PRIM_SOURCE Global_buffer,
## see m7_bd.tcl). Vivado methodology CLKC-56 ("MMCME4 with global clock driver has no LOC"; D0 had the
## PLLE4 twin CLKC-58) asks for a fixed site. MMCM_X0Y3 is clock region X1Y3: the CMT column where the
## D0 round-2 placer itself put the PLLE4 (PLL_X0Y7, X1Y3), next to pl_clk0's BUFG_PS (X0Y3) and to the
## DPU's main placement (D0 round 2 DPU leaf cells: X*Y2-X*Y3 hold ~81%; probe DPU1/clkprobe/out.txt).
## No CLOCK_DEDICATED_ROUTE: D0 routed the same BUFG_PS -> CMT X1Y3 connection without one (no DRC).
set_property LOC MMCM_X0Y3 [get_cells -hierarchical -filter {REF_NAME == MMCME4_ADV && NAME =~ *clk_wiz_dpu*}]
