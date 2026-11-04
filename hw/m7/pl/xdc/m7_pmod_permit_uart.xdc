## In use since the 2026-10-11 merge build: hw/m7/pl/bd/build_bitstream.tcl
## reads this file (`read_xdc`) before synthesis. Checked on the routed merge
## build 7784f75: permit_hb_o = H12 LVCMOS33 PULLDOWN, enc_uart_rx_i = B10
## LVCMOS33 PULLUP (docs/design/PL_MERGE_20261011.md sections 1 and 7'.6).
##
## KV260 carrier Pmod (J2), PL bank 45 (HDIO, 3.3 V). Sources:
##  - FPGA package pins of the Pmod: Xilinx/kria-vitis-platforms, branch
##    xlnx_rel_v2022.1, kv260/platforms/vivado/kv260_ispMipiRx_vcu_DP/xdc/pin.xdc
##    ("#Digilent PMOD pins": H12 E10 D10 C11 / B10 E12 D11 B11, LVCMOS33,
##    SLEW SLOW, DRIVE 4), sha256 295559b9...0d5.
##  - SOM connector names: Xilinx/XilinxBoardStore 2022.2 boards/Xilinx/k26c/1.4/
##    part0_pins.xml (H12 = som240_1_a17, B10 = som240_1_b20).
##  - Bank/type: Vivado 2022.2 on xck26-sfvc784-2LV-c: H12 IO_L4N_AD12N_45,
##    B10 IO_L9N_AD11N_45, both bank 45 BT_HIGH_DENSITY.
## Physical position on the header (MUST be checked by hand before wiring):
##  H12 is "pin 1" in both public descriptions found. B10 is the pin of the
##  OTHER row in the same column as H12: Digilent numbering pin 7, or pin 2 in
##  an odd/even (1-2 / 3-4 ...) numbering (third-party hackster.io example).
##  Confirm on the KV260 carrier-card schematic (AMD Kria KV260 documentation)
##  or with a continuity meter
##  before connecting the level shifter.

## Drive-permit heartbeat -> level shifter -> chassis MCU input (MCU side pull-down).
set_property PACKAGE_PIN H12      [get_ports permit_hb_o]
set_property IOSTANDARD  LVCMOS33 [get_ports permit_hb_o]
set_property SLEW        SLOW     [get_ports permit_hb_o]
set_property DRIVE       4        [get_ports permit_hb_o]
## Only matters if the output is ever tri-stated after configuration. Before and
## during configuration this property has NO effect (pin state is set by the
## device's PUDC_B strap -- see spec section 7, manual check item 2).
set_property PULLDOWN    TRUE     [get_ports permit_hb_o]

## v3fc (2026-10-26): the encoder UART core and its input port enc_uart_rx_i
## (B10) are removed from the design (never wired; wheel odometry reaches the
## Nano over the chassis UART), so B10 is unused again.

## The heartbeat is a 100 Hz level whose timing is set by the RTL divider.
set_false_path -to   [get_ports permit_hb_o]
